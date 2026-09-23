"""Export der Aufzeichnungen ins LeRobotDataset-Format (AP 2.3 / 0.9 Punkt 6).

Aufgezeichnet wird weiter im neutralen Format (npz + json, dataset.py) --
unabhaengig von lerobot und ohne Videokodierung im 15-Hz-Takt. Dieser
Schritt fasst eine oder mehrere Aufzeichnungen (z. B. ein Block je
Objektlage) zu EINEM LeRobotDataset zusammen, das auf jedem Rechner mit
der gepinnten lerobot-Version trainierbar ist (5070 Ti, 5090, Linux).

Was der Export absichert, weil es sich spaeter nicht mehr korrigieren laesst:

* **Schema und Rate:** alle Quellen gleiche ``SCHEMA_VERSION``, gleiche
  Rate, gleiche Kameras -- sonst Abbruch (nie mischen, AP 0.10).
* **Einheitliches Tempo (AP 2.6):** Planer-Geschwindigkeiten, Rampe,
  servo_j-Rate, Override und Robotertyp muessen in allen Episoden gleich
  sein. Das Tempo wird mitgelernt; ein Datensatz aus zwei Tempi waere
  multimodal. Abweichungen brechen ab (``allow_mixed`` nur fuer bewusste
  Vergleiche). Aeltere Episoden ohne diese Metadaten werden gemeldet.
* **Episodenauswahl (AP 5.2):** verworfene Episoden und solche mit
  ``success == False`` fliessen nie ein; unbewertete nur, solange
  ``require_success`` nicht gesetzt ist -- und werden gezaehlt.
* **Nachvollziehbarkeit:** ``meta/bc_export.json`` haelt lerobot- und
  Schema-Version, Quellen und je LeRobot-Episode die Herkunft samt
  Metadaten (Objektlage, Block, Licht, Kamerapose, Rauschfaktor, Seed).

Das Datensatzformat selbst (``codebase_version`` v3.0) schreibt lerobot in
``meta/info.json``. lerobot wird erst in :func:`export` importiert, damit
bc/ ohne lerobot importierbar bleibt.
"""

import json
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np

from . import config, dataset

#: Dateiname der Export-Metadaten neben lerobots eigenen in ``meta/``.
EXPORT_INFO = "bc_export.json"

#: Metadaten, die ueber alle Episoden eines Datensatzes GLEICH sein muessen
#: (AP 2.6 Tempo, AP 1.3 Takt, Robotertyp). Pfad in ``episode.metadata``.
CONSISTENCY_KEYS = (
    ("robot",),
    ("cameras",),  # Kamera-Backends: echte und Platzhalterbilder nie mischen
    ("in_simulation",),
    ("override",),
    ("servo_rate_hz",),
    ("rate_hz",),
    ("planner", "transit_speed_ms"),
    ("planner", "approach_speed_ms"),
    ("planner", "ptp_joint_speed_rads"),
    ("planner", "segment_ramp_s"),
    ("planner", "gripper_dwell_s"),
)

_STATE_NAMES = [
    "q1", "q2", "q3", "q4", "q5", "q6",
    "tcp_x", "tcp_y", "tcp_z", "tcp_qw", "tcp_qx", "tcp_qy", "tcp_qz",
    "gripper",
]
_ACTION_NAMES = ["q1", "q2", "q3", "q4", "q5", "q6", "gripper"]
_POSE_NAMES = ["x", "y", "z", "qw", "qx", "qy", "qz"]
_JOINT_NAMES = _ACTION_NAMES[:6]


class ExportError(ValueError):
    """Quellen nicht zusammen exportierbar (Schema, Tempo, keine Episoden)."""


def lerobot_features(camera_names, use_video=True):
    """Feature-Beschreibung im LeRobot-Format, abgeleitet aus dataset.features().

    Namen je Kanal machen den Datensatz ohne dieses Repo lesbar.
    """
    names = {
        "observation.state": _STATE_NAMES,
        "action": _ACTION_NAMES,
        "aux.joints_ideal": _JOINT_NAMES,
        "aux.joints_command": _JOINT_NAMES,
        "aux.pose_ideal": _POSE_NAMES,
        "aux.pose_noisy": _POSE_NAMES,
        "aux.sync_ok": ["ok"],
        "next.done": ["done"],
    }
    out = {}
    for key, spec in dataset.features(camera_names).items():
        if key.startswith("observation.images."):
            out[key] = {
                "dtype": "video" if use_video else "image",
                "shape": tuple(spec["shape"]),
                "names": ["height", "width", "channels"],
            }
        else:
            out[key] = {
                "dtype": spec["dtype"],
                "shape": tuple(spec["shape"]),
                "names": names[key],
            }
    return out


def _get(meta, path):
    value = meta
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def scan_sources(sources, require_success=False):
    """Prueft die Quellen und waehlt Episoden aus -- ohne lerobot.

    Rueckgabe: dict mit ``episodes`` (Liste von (root, index, meta)),
    ``cameras``, ``skipped`` (Grund -> Anzahl), ``unlabeled``,
    ``consistency`` (Schluessel -> {Wert: Anzahl}), ``conflicts``
    (Schluessel mit mehr als einem bekannten Wert), ``missing_meta``.
    """
    sources = [Path(s) for s in sources]
    if not sources:
        raise ExportError("Keine Quelle angegeben")

    cameras = None
    episodes = []
    skipped = {}
    unlabeled = 0
    consistency = {"/".join(k): {} for k in CONSISTENCY_KEYS}
    missing_meta = {"/".join(k): 0 for k in CONSISTENCY_KEYS}

    for root in sources:
        index_path = root / "index.json"
        if not index_path.is_file():
            raise ExportError("%s ist keine Aufzeichnung (index.json fehlt)" % root)
        index = json.loads(index_path.read_text(encoding="utf-8"))
        if index["schema_version"] != config.SCHEMA_VERSION:
            raise ExportError(
                "%s hat Schema-Version %s, Code erwartet %s -- nicht mischen"
                % (root, index["schema_version"], config.SCHEMA_VERSION)
            )
        if float(index["rate_hz"]) != config.CONTROL_RATE_HZ:
            raise ExportError(
                "%s wurde mit %.1f Hz aufgezeichnet, Festlegung ist %.1f Hz"
                % (root, index["rate_hz"], config.CONTROL_RATE_HZ)
            )
        if cameras is None:
            cameras = list(index["cameras"])
        elif list(index["cameras"]) != cameras:
            raise ExportError(
                "%s hat Kameras %s, vorherige Quellen %s"
                % (root, index["cameras"], cameras)
            )

        for i, entry in enumerate(index["episodes"]):
            meta = json.loads((root / entry["dir"] / "meta.json").read_text(encoding="utf-8"))
            if meta["discarded"]:
                reason = "verworfen (%s)" % (meta["discard_reason"] or "unbekannt")
                skipped[reason] = skipped.get(reason, 0) + 1
                continue
            if meta["success"] is False:
                skipped["fehlgeschlagen"] = skipped.get("fehlgeschlagen", 0) + 1
                continue
            if meta["success"] is None:
                if require_success:
                    skipped["unbewertet"] = skipped.get("unbewertet", 0) + 1
                    continue
                unlabeled += 1
            info = meta["metadata"]
            for key in CONSISTENCY_KEYS:
                name = "/".join(key)
                value = _get(info, key)
                if value is None:
                    if not (key == ("override",) and info.get("robot") == "sim"):
                        missing_meta[name] += 1
                    continue
                token = json.dumps(value)
                consistency[name][token] = consistency[name].get(token, 0) + 1
            episodes.append((root, i, meta))

    conflicts = {k: v for k, v in consistency.items() if len(v) > 1}
    return {
        "episodes": episodes,
        "cameras": cameras or [],
        "skipped": skipped,
        "unlabeled": unlabeled,
        "consistency": consistency,
        "conflicts": conflicts,
        "missing_meta": {k: v for k, v in missing_meta.items() if v},
    }


def format_scan(scan):
    lines = [
        "Episoden fuer den Export: %d (davon unbewertet: %d)"
        % (len(scan["episodes"]), scan["unlabeled"]),
        "Kameras: %s" % ", ".join(scan["cameras"]),
    ]
    for reason, count in sorted(scan["skipped"].items()):
        lines.append("  ausgelassen, %s: %d" % (reason, count))
    for key, values in sorted(scan["consistency"].items()):
        if values:
            shown = ", ".join("%s (%dx)" % (v, n) for v, n in values.items())
            flag = "  !! UNEINHEITLICH" if key in scan["conflicts"] else ""
            lines.append("  %-30s %s%s" % (key, shown, flag))
    for key, count in sorted(scan["missing_meta"].items()):
        lines.append("  %-30s fehlt in %d Episode(n) (aeltere Aufzeichnung)" % (key, count))
    return "\n".join(lines)


def _task_of(meta):
    info = meta["metadata"]
    return str(info.get("sequence") or info.get("task") or "pick_demo")


def export(
    sources,
    output,
    require_success=False,
    allow_mixed=False,
    use_video=True,
    crf=None,
    overwrite=False,
    repo_id="local/bc_lara5",
    progress=print,
):
    """Schreibt die ausgewaehlten Episoden als LeRobotDataset nach ``output``.

    Rueckgabe: der Inhalt von ``meta/bc_export.json`` (dict).
    """
    import lerobot
    from lerobot.datasets import LeRobotDataset

    if lerobot.__version__ != config.LEROBOT_VERSION:
        raise ExportError(
            "lerobot %s installiert, gepinnt ist %s (config.LEROBOT_VERSION) -- "
            "das Datensatzformat haengt an der Version (AP 0.9 Punkt 6)."
            % (lerobot.__version__, config.LEROBOT_VERSION)
        )

    scan = scan_sources(sources, require_success=require_success)
    progress(format_scan(scan))
    if not scan["episodes"]:
        raise ExportError("Keine verwendbare Episode in den Quellen")
    if scan["conflicts"] and not allow_mixed:
        raise ExportError(
            "Uneinheitliche Aufzeichnungsbedingungen %s -- das Tempo wird "
            "mitgelernt (AP 2.6). Nur fuer bewusste Vergleiche allow_mixed."
            % sorted(scan["conflicts"])
        )

    output = Path(output)
    if output.exists():
        if not overwrite:
            raise ExportError("%s existiert bereits (overwrite nicht gesetzt)" % output)
        shutil.rmtree(output)

    encoder = None
    if use_video and crf is not None:
        from lerobot.configs.video import rgb_encoder_defaults

        encoder = rgb_encoder_defaults()
        encoder.crf = int(crf)

    cameras = scan["cameras"]
    feats = lerobot_features(cameras, use_video=use_video)
    robot_type = "neura_lara5" if _get(scan["episodes"][0][2]["metadata"], ("robot",)) == "neura" else "sim_lara5"
    # Streaming-Kodierung: Frames direkt in den Encoder statt erst als PNG
    # auf die Platte. Gemessen 2026-09-17 (3 Sim-Episoden): 5 s statt 45 s,
    # dekodierte Bilder identisch (mittlere Abweichung zum Original 0.0038).
    lr = LeRobotDataset.create(
        repo_id=repo_id,
        fps=int(config.CONTROL_RATE_HZ),
        features=feats,
        root=output,
        robot_type=robot_type,
        use_videos=use_video,
        rgb_encoder=encoder,
        streaming_encoding=use_video,
    )

    records, starts, ends, grippers = [], [], [], []
    reference_path, z_min = None, float("inf")
    try:
        for k, (root, i, meta) in enumerate(scan["episodes"]):
            ep = dataset.load_episode(root, i)
            ep.validate(cameras)
            starts.append(ep.arrays["observation.state"][0, :6])
            ends.append(ep.arrays["aux.joints_ideal"][-1])
            grippers.append(ep.arrays["observation.state"][[0, -1], 13])
            task = _task_of(meta)
            for t in range(len(ep)):
                frame = {key: ep.arrays[key][t] for key in feats}
                frame["task"] = task
                lr.add_frame(frame)
            lr.save_episode()
            ref = episode_reference(ep)
            if reference_path is None:
                reference_path = ref.pop("path")
            else:
                ref.pop("path")
            z_min = min(z_min, ref.pop("z_min"))
            records.append(
                {
                    "episode_index": k,
                    **ref,
                    "source": str(root),
                    "source_dir": json.loads((root / "index.json").read_text(encoding="utf-8"))[
                        "episodes"
                    ][i]["dir"],
                    "length": len(ep),
                    "task": task,
                    "success": meta["success"],
                    "metadata": meta["metadata"],
                }
            )
            progress("  Episode %d/%d exportiert (%s, %d Schritte)"
                     % (k + 1, len(scan["episodes"]), root.name, len(ep)))
    finally:
        lr.finalize()

    info = {
        "exported_at": datetime.now().isoformat(timespec="seconds"),
        "lerobot_version": lerobot.__version__,
        "codebase_version": _codebase_version(),
        "schema_version": config.SCHEMA_VERSION,
        "rate_hz": config.CONTROL_RATE_HZ,
        "cameras": cameras,
        "image_size_hw": [config.IMAGE_HEIGHT, config.IMAGE_WIDTH],
        "use_video": bool(use_video),
        "crf": crf,
        "robot_type": robot_type,
        "sources": [str(Path(s)) for s in sources],
        "require_success": bool(require_success),
        "allow_mixed": bool(allow_mixed),
        "unlabeled_episodes": scan["unlabeled"],
        "skipped": scan["skipped"],
        "consistency": scan["consistency"],
        "conflicts": sorted(scan["conflicts"]),
        "missing_meta": scan["missing_meta"],
        # Median der Start- bzw. End-Stellungen: die Inferenz faehrt vor dem
        # Start dorthin und erkennt das Ende daran (AP 5.2 Reset).
        "start_joints": np.median(np.asarray(starts), axis=0).tolist(),
        "end_joints": np.median(np.asarray(ends), axis=0).tolist(),
        "start_joints_spread_rad": float(np.max(np.ptp(np.asarray(starts), axis=0))),
        # Greiferzustand am Start (Reset vor jeder Fahrt, AP 5.2) und am Ende.
        # Das Ende ist nur mit Stellung UND Greifer eindeutig: in einer Bahn
        # kann dieselbe Stellung mehrfach vorkommen (Befund Sim-Demo 2026-09-17:
        # "transport" == "rueckzug", nur der Greifer unterscheidet).
        "start_gripper": float(np.median(np.asarray(grippers)[:, 0])),
        "end_gripper": float(np.median(np.asarray(grippers)[:, 1])),
        # Tiefster TCP-Punkt der Idealbahnen -- Tischhoehe fuers Geofencing
        # der Inferenz nach derselben Regel wie in apps/record.py.
        "ideal_z_min": z_min,
        # Idealbahn (TCP xyz) der ersten Episode als Referenz fuer die
        # Bewertung einer Policy-Fahrt ohne Datensatz (bc.metrics).
        "reference_path": reference_path,
        "episodes": records,
    }
    (output / "meta" / EXPORT_INFO).write_text(
        json.dumps(info, indent=2, ensure_ascii=False, default=_json_default), encoding="utf-8"
    )
    return info


def episode_reference(ep):
    """Referenzgroessen einer Episode aus der IDEALEN Bahn (aux.pose_ideal).

    ``grasp_tcp``: TCP beim ersten Schliessen des Greifers (Objektlage),
    ``end_tcp``: TCP am Uebergabepunkt, ``path``: TCP-Bahn (xyz, mm-genau),
    ``z_min``: tiefster Punkt.
    """
    poses = np.asarray(ep.arrays["aux.pose_ideal"], dtype=float)
    gripper = np.asarray(ep.arrays["observation.state"][:, 13], dtype=float)
    closing = np.flatnonzero((gripper[1:] > config.GRIPPER_THRESHOLD)
                             & (gripper[:-1] <= config.GRIPPER_THRESHOLD))
    grasp = poses[closing[0] + 1, :3].round(4).tolist() if len(closing) else None
    return {
        "grasp_tcp": grasp,
        "end_tcp": poses[-1, :3].round(4).tolist(),
        "path": poses[:, :3].round(4).tolist(),
        "z_min": float(poses[:, 2].min()),
    }


def _codebase_version():
    from lerobot.datasets.dataset_metadata import CODEBASE_VERSION

    return CODEBASE_VERSION


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    raise TypeError("nicht serialisierbar: %r" % type(value))


def read_export_info(root):
    """Liest ``meta/bc_export.json`` eines exportierten Datensatzes."""
    path = Path(root) / "meta" / EXPORT_INFO
    if not path.is_file():
        raise ExportError(
            "%s ist kein mit bc exportierter Datensatz (%s fehlt)" % (root, EXPORT_INFO)
        )
    return json.loads(path.read_text(encoding="utf-8"))

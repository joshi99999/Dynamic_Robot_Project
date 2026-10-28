"""Datensatz-Schema und Episoden-Ablage (AP 2.3 / 5.2 / 0.9 Punkt 1-2).

Das Schema ist die verbindliche Definition des State-/Action-Raums --
Aufzeichnung und Inferenz muessen EXAKT dieselben Groessen, Einheiten,
Reihenfolgen und Bezugsframes verwenden (AP 1.5.1). Jede Aenderung
erfordert das Hochzaehlen von ``config.SCHEMA_VERSION``.

Ablageformat: ein neutrales, abhaengigkeitsfreies Format (npz + json) je
Episode. Die Konvertierung ins ``LeRobotDataset``-Format ist ein separater
Schritt (:func:`to_lerobot`, lerobot gepinnt in config.LEROBOT_VERSION) --
so haengt die Aufzeichnung nicht an einer schweren Abhaengigkeit, und das
Format ist im Test ohne lerobot pruefbar.
"""

import json
import shutil
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import config, geometry

# --------------------------------------------------------------------------
# Schema (verbindlich, AP 0.10)
# --------------------------------------------------------------------------

#: observation.state: [q1..q6, tcp_x, tcp_y, tcp_z, qw, qx, qy, qz, greifer]
#: Einheiten rad / m / Einheitsquaternion / {0.0, 1.0}. Der Greifer ist der
#: KOMMANDIERTE Zustand (keine Ist-Rueckmeldung, AP 2.1). Das Quaternion ist
#: kanonisch zu config.QUAT_HEMISPHERE_REF (ab Schema 2, geometry.py).
STATE_LAYOUT = (
    ("joints", 6),
    ("tcp_pos", 3),
    ("tcp_quat", 4),
    ("gripper", 1),
)

#: action: [q1..q6, greifer] -- absolute Zielwinkel der IDEALEN
#: Soll-Trajektorie zum Zeitpunkt t+1 (AP 2.4, asymmetrische Paarung).
ACTION_LAYOUT = (
    ("joints", 6),
    ("gripper", 1),
)


def features(camera_names=("wrist", "scene")):
    """Vollstaendige Feature-Beschreibung des Datensatzes."""
    feats = {
        "observation.state": {
            "shape": (config.STATE_DIM,),
            "dtype": "float32",
            "layout": [list(x) for x in STATE_LAYOUT],
        },
        "action": {
            "shape": (config.ACTION_DIM,),
            "dtype": "float32",
            "layout": [list(x) for x in ACTION_LAYOUT],
        },
        # Zusatzkanaele (AP 0.9 Punkt 2): Soll- UND Ist-Bahn, damit die
        # asymmetrische Label-Logik ueberpruef- und umlabelbar bleibt.
        "aux.joints_ideal": {"shape": (6,), "dtype": "float32"},
        # Tatsaechlich per servo_j gesendete (verrauschte) Sollwinkel --
        # zusammen mit observation.state zeigt das den Nachlauf des Arms.
        "aux.joints_command": {"shape": (6,), "dtype": "float32"},
        "aux.pose_ideal": {"shape": (7,), "dtype": "float32"},
        "aux.pose_noisy": {"shape": (7,), "dtype": "float32"},
        "aux.sync_ok": {"shape": (1,), "dtype": "bool"},
        # LeRobot-Konvention: True genau im letzten Schritt einer
        # vollstaendigen Episode. Markiert den Uebergabepunkt ans
        # Hauptprogramm (Ende des BC-Abschnitts, z. B. PRE_PLACE).
        "next.done": {"shape": (1,), "dtype": "bool"},
    }
    for name in camera_names:
        feats["observation.images.%s" % name] = {
            "shape": (config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3),
            "dtype": "uint8",
        }
    return feats


def build_state(joints, tcp_quat, gripper_value):
    """Baut den State-Vektor exakt nach STATE_LAYOUT."""
    state = np.concatenate(
        [
            np.asarray(joints, dtype=np.float32),
            canonical_pose(tcp_quat).astype(np.float32),
            np.array([gripper_value], dtype=np.float32),
        ]
    )
    if state.shape != (config.STATE_DIM,):
        raise ValueError(
            "State hat %s Werte, Schema verlangt %d" % (state.shape, config.STATE_DIM)
        )
    return state


def canonical_pose(pose_quat):
    """[X,Y,Z,QW,QX,QY,QZ] mit Quaternion kanonisch zur Schema-Referenz.

    Gleiche Orientierung -> gleiche Zahlen, egal ob FK, Controller oder
    Interpolation q oder -q geliefert hat (Schema 2).
    """
    pose = np.asarray(pose_quat, dtype=float).copy()
    pose[3:7] = geometry.quat_canonical(pose[3:7], config.QUAT_HEMISPHERE_REF)
    return pose


def build_action(joints, gripper_value):
    """Baut den Action-Vektor exakt nach ACTION_LAYOUT."""
    action = np.concatenate(
        [
            np.asarray(joints, dtype=np.float32),
            np.array([gripper_value], dtype=np.float32),
        ]
    )
    if action.shape != (config.ACTION_DIM,):
        raise ValueError(
            "Action hat %s Werte, Schema verlangt %d" % (action.shape, config.ACTION_DIM)
        )
    return action


def split_state(state):
    """Zerlegt einen State-Vektor in seine benannten Teile (dict)."""
    out = {}
    i = 0
    for name, width in STATE_LAYOUT:
        out[name] = np.asarray(state)[i : i + width]
        i += width
    return out


def gripper_from_action(action):
    """Binarisiert den Greiferkanal einer Action (Inferenz, AP 0.10)."""
    return float(action[-1]) > config.GRIPPER_THRESHOLD


# --------------------------------------------------------------------------
# Episoden-Container
# --------------------------------------------------------------------------


@dataclass
class Episode:
    """Eine aufgezeichnete Episode vor dem Schreiben.

    ``arrays``: dict Feature-Name -> ndarray (N, ...) gemaess features().
    ``discarded``/``discard_reason``: Verwerf-Logik nach AP 5.2.
    ``success``: manuelles Label (Dashboard, AP 1.2); None = unbewertet.
    """

    arrays: dict
    metadata: dict = field(default_factory=dict)
    success: bool = None
    discarded: bool = False
    discard_reason: str = ""

    def __len__(self):
        return len(self.arrays["observation.state"])

    def validate(self, camera_names=("wrist", "scene")):
        """Prueft die Episode gegen das Schema. Wirft ValueError."""
        feats = features(camera_names)
        for name, spec in feats.items():
            if name not in self.arrays:
                raise ValueError("Feature '%s' fehlt in der Episode" % name)
            arr = self.arrays[name]
            expected = (len(self),) + tuple(spec["shape"])
            if tuple(arr.shape) != expected:
                raise ValueError(
                    "Feature '%s': Form %s, erwartet %s"
                    % (name, arr.shape, expected)
                )
            if str(arr.dtype) != spec["dtype"]:
                raise ValueError(
                    "Feature '%s': dtype %s, erwartet %s"
                    % (name, arr.dtype, spec["dtype"])
                )
        return True


# --------------------------------------------------------------------------
# Ablage
# --------------------------------------------------------------------------


class DatasetWriter(object):
    """Schreibt Episoden in ein Wurzelverzeichnis.

    Struktur::

        root/
          index.json            Schema, Rate, Episodenliste
          ep_00000/arrays.npz   alle Features als Arrays
          ep_00000/meta.json    Metadaten, Erfolg, Verwerf-Status

    Eine Episode wird erst unter ``ep_XXXXX.partial`` geschrieben, dann
    umbenannt und zuletzt in den Index eingetragen. Bricht ein Lauf
    dazwischen ab, bleibt ein Ordner ohne Indexeintrag zurueck. Frueher
    scheiterte der NAECHSTE Lauf daran (FileExistsError beim ersten
    ``append``, Befund 27.10.2026) -- nach der ersten, fertig gefahrenen
    Episode. Solche Reste schiebt der Konstruktor jetzt beiseite
    (``*.verwaist_<Zeit>``, nichts wird geloescht) und listet sie in
    :attr:`set_aside`.
    """

    def __init__(self, root, camera_names=("wrist", "scene")):
        self.root = Path(root)
        self.camera_names = tuple(camera_names)
        self.root.mkdir(parents=True, exist_ok=True)
        self._index_path = self.root / "index.json"
        if self._index_path.exists():
            self._index = json.loads(self._index_path.read_text(encoding="utf-8"))
            if self._index["schema_version"] != config.SCHEMA_VERSION:
                raise ValueError(
                    "Datensatz hat Schema-Version %s, Code erwartet %s -- "
                    "Datensaetze verschiedener Versionen nicht mischen!"
                    % (self._index["schema_version"], config.SCHEMA_VERSION)
                )
        else:
            self._index = {
                "schema_version": config.SCHEMA_VERSION,
                "rate_hz": config.CONTROL_RATE_HZ,
                "features": features(self.camera_names),
                "cameras": list(self.camera_names),
                "episodes": [],
            }
            self._write_index()
        #: (alter Name, neuer Name) der beim Oeffnen beiseitegelegten Reste
        self.set_aside = self._set_aside_orphans()

    def _set_aside_orphans(self):
        """Episodenordner ohne Indexeintrag umbenennen (siehe Klassendoku)."""
        known = {entry["dir"] for entry in self._index["episodes"]}
        stamp = time.strftime("%Y%m%d-%H%M%S")
        moved = []
        for path in sorted(self.root.glob("ep_*")):
            if not path.is_dir() or path.name in known or ".verwaist_" in path.name:
                continue
            target = path.with_name("%s.verwaist_%s" % (path.name, stamp))
            path.rename(target)
            moved.append((path.name, target.name))
        return moved

    def _write_index(self):
        self._index_path.write_text(
            json.dumps(self._index, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    def append(self, episode):
        """Schreibt eine Episode (nach Schema-Validierung)."""
        episode.validate(self.camera_names)
        idx = len(self._index["episodes"])
        ep_dir = self.root / ("ep_%05d" % idx)
        if ep_dir.exists():
            # Nur moeglich, wenn waehrend des Laufs jemand anderes in
            # denselben Ordner schreibt -- dann nichts ueberschreiben.
            raise FileExistsError(
                "%s existiert schon, steht aber nicht im Index -- schreibt ein "
                "zweiter Lauf in denselben Ordner?" % ep_dir)
        partial = ep_dir.with_name(ep_dir.name + ".partial")
        if partial.exists():
            shutil.rmtree(partial)
        partial.mkdir(parents=True)

        # npz erlaubt keine Punkte in Schluesseln ohne Weiteres -> ersetzen
        np.savez_compressed(
            partial / "arrays.npz",
            **{k.replace(".", "__"): v for k, v in episode.arrays.items()}
        )
        meta = {
            "length": len(episode),
            "success": episode.success,
            "discarded": episode.discarded,
            "discard_reason": episode.discard_reason,
            "metadata": episode.metadata,
        }
        (partial / "meta.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        partial.rename(ep_dir)

        self._index["episodes"].append(
            {"dir": ep_dir.name, "length": len(episode), "discarded": episode.discarded}
        )
        self._write_index()
        return idx


#: Nachtraegliche Bewertung -> (success, discarded)
RELABEL = {
    "erfolg": (True, False),
    "fehler": (False, False),
    "verwerfen": (None, True),
}


def relabel(root, folder, label, reason="nachtraeglich verworfen"):
    """Bewertung einer abgelegten Episode aendern (``label`` aus RELABEL).

    Fuer "Standard erfolgreich, Fehlschlag bewusst setzen": Fehlschlaege
    werden nach der Fahrt in der Episodenliste markiert, ohne den Lauf
    anzuhalten. Geschrieben wird NUR die meta.json -- danach richtet sich
    der Export (bc/lerobot_io.scan_sources). Der Merker ``discarded`` im
    index.json bleibt stehen: den schreibt ein noch laufendes record.py
    bei jeder Episode aus seinem Speicher neu, eine Aenderung dort waere
    sofort wieder weg. Die vorherige Bewertung steht danach unter
    ``relabeled_from`` in der meta.json.
    """
    success, discarded = RELABEL[label]
    path = Path(root) / folder / "meta.json"
    meta = json.loads(path.read_text(encoding="utf-8"))
    meta.setdefault("relabeled_from", {
        "success": meta["success"], "discarded": meta["discarded"],
        "discard_reason": meta["discard_reason"]})
    meta["success"] = success
    meta["discarded"] = discarded
    meta["discard_reason"] = reason if discarded else ""
    path.write_text(json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
    return meta


def load_episode(root, index):
    """Laedt eine Episode zurueck (fuer Tests und Auswertung)."""
    root = Path(root)
    idx = json.loads((root / "index.json").read_text(encoding="utf-8"))
    ep_dir = root / idx["episodes"][index]["dir"]
    with np.load(ep_dir / "arrays.npz") as npz:
        arrays = {k.replace("__", "."): npz[k] for k in npz.files}
    meta = json.loads((ep_dir / "meta.json").read_text(encoding="utf-8"))
    ep = Episode(
        arrays=arrays,
        metadata=meta["metadata"],
        success=meta["success"],
        discarded=meta["discarded"],
        discard_reason=meta["discard_reason"],
    )
    return ep


def to_lerobot(roots, output, **kwargs):
    """Konvertierung ins LeRobotDataset-Format (lerobot gepinnt auf
    config.LEROBOT_VERSION, AP 0.9 Punkt 6).

    ``roots``: eine Aufzeichnung oder eine Liste (z. B. ein Block je
    Objektlage). Pruefungen und Optionen: :func:`bc.lerobot_io.export`.
    """
    from . import lerobot_io

    if isinstance(roots, (str, Path)):
        roots = [roots]
    return lerobot_io.export(roots, output, **kwargs)

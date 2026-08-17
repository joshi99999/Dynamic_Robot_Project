"""Datensatz-Schema und Episoden-Ablage (AP 2.3 / 5.2 / 0.9 Punkt 1-2).

Das Schema ist die verbindliche Definition des State-/Action-Raums --
Aufzeichnung und Inferenz muessen EXAKT dieselben Groessen, Einheiten,
Reihenfolgen und Bezugsframes verwenden (AP 1.5.1). Jede Aenderung
erfordert das Hochzaehlen von ``config.SCHEMA_VERSION``.

Ablageformat: ein neutrales, abhaengigkeitsfreies Format (npz + json) je
Episode. Die Konvertierung ins ``LeRobotDataset``-Format erfolgt in einem
separaten Schritt, sobald die LeRobot-Version gepinnt ist (AP 0.9 Punkt 6)
-- so haengt die Aufzeichnung nicht an einer schweren Abhaengigkeit und
das Format ist im Test ohne lerobot pruefbar.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import config

# --------------------------------------------------------------------------
# Schema (verbindlich, AP 0.10)
# --------------------------------------------------------------------------

#: observation.state: [q1..q6, tcp_x, tcp_y, tcp_z, qw, qx, qy, qz, greifer]
#: Einheiten rad / m / Einheitsquaternion / {0.0, 1.0}. Der Greifer ist der
#: KOMMANDIERTE Zustand (keine Ist-Rueckmeldung, AP 2.1).
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
        "aux.pose_ideal": {"shape": (7,), "dtype": "float32"},
        "aux.pose_noisy": {"shape": (7,), "dtype": "float32"},
        "aux.sync_ok": {"shape": (1,), "dtype": "bool"},
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
            np.asarray(tcp_quat, dtype=np.float32),
            np.array([gripper_value], dtype=np.float32),
        ]
    )
    if state.shape != (config.STATE_DIM,):
        raise ValueError(
            "State hat %s Werte, Schema verlangt %d" % (state.shape, config.STATE_DIM)
        )
    return state


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

    def _write_index(self):
        self._index_path.write_text(
            json.dumps(self._index, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    def append(self, episode):
        """Schreibt eine Episode (nach Schema-Validierung)."""
        episode.validate(self.camera_names)
        idx = len(self._index["episodes"])
        ep_dir = self.root / ("ep_%05d" % idx)
        ep_dir.mkdir(parents=True, exist_ok=False)

        # npz erlaubt keine Punkte in Schluesseln ohne Weiteres -> ersetzen
        np.savez_compressed(
            ep_dir / "arrays.npz",
            **{k.replace(".", "__"): v for k, v in episode.arrays.items()}
        )
        meta = {
            "length": len(episode),
            "success": episode.success,
            "discarded": episode.discarded,
            "discard_reason": episode.discard_reason,
            "metadata": episode.metadata,
        }
        (ep_dir / "meta.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8"
        )

        self._index["episodes"].append(
            {"dir": ep_dir.name, "length": len(episode), "discarded": episode.discarded}
        )
        self._write_index()
        return idx


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


def to_lerobot(root, output):
    """Konvertierung ins LeRobotDataset-Format.

    Bewusst noch nicht implementiert: die LeRobot-Version (und damit das
    Zielformat) ist zu pinnen, BEVOR echte Daten aufgezeichnet werden
    (AP 0.9 Punkt 6). Das neutrale Ablageformat oben enthaelt alle dafuer
    noetigen Informationen.
    """
    raise NotImplementedError(
        "LeRobot-Version zuerst pinnen (AP 0.9 Punkt 6), dann diese "
        "Konvertierung gegen die gepinnte API implementieren."
    )

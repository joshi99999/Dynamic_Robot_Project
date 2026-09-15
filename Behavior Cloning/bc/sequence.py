"""Ablaufdatei: generische Punktfolge fuer die Datenaufzeichnung (AP 2.2).

Die Pipeline kennt den ABLAUF nur als Folge von Punktnamen. Die Koordinaten
kommen vor jeder Episode frisch aus einer :class:`bc.ports.PointSourcePort`
(am Neura: die Punkte-Datenbank der Control-Box). Wird ein Punkt am Pendant
per Touch-up nachgeteacht, faehrt schon die naechste Episode die neue Lage
an -- ohne Export und ohne Abhaengigkeit vom Namen des Neura-Programms.

Format (JSON)::

    {
      "name": "pick_to_station",
      "gripper_start": "open",
      "sequence": [
        {"point": "CLEAR_FOV",   "motion": "ptp"},
        {"point": "APPROACH_01", "motion": "ptp"},
        {"point": "APPROACH_02", "motion": "ptp", "optional": true},
        {"point": "PRE_GRASP",   "motion": "ptp"},
        {"point": "PICK",        "motion": "lin", "approach": true, "gripper": "close"},
        {"point": "PRE_GRASP",   "motion": "lin"},
        {"point": "PRE_PLACE",   "motion": "ptp"}
      ]
    }

* Der erste Schritt ist die Startstellung (seine ``motion`` wird ignoriert),
  der letzte der Uebergabepunkt ans Hauptprogramm. Beide duerfen nicht
  ``optional`` sein.
* ``motion``: Bewegungsart des Segments ZU diesem Punkt ("ptp" | "lin").
* ``gripper``: "close" | "open" -- Greiferbefehl nach Erreichen des
  Punkts; fehlt er, bleibt der Zustand.
* ``optional``: Punkt wird uebersprungen, wenn er in der Datenbank fehlt
  (z. B. zusaetzliche Approach-Punkte fuer mehr Variation). Fehlt ein
  Pflichtpunkt, bricht die Aufloesung mit allen fehlenden Namen ab.
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .trajectory import MOTION_PTP, MOTIONS, Waypoint

_GRIPPER_VALUES = {"close": True, "open": False}
_STEP_KEYS = {"point", "motion", "approach", "gripper", "optional"}


class SequenceError(ValueError):
    """Ablaufdatei ungueltig oder nicht gegen die Punkte aufloesbar."""


@dataclass
class SequenceStep:
    point: str
    motion: str = MOTION_PTP
    approach: bool = False
    gripper: str = None
    optional: bool = False


@dataclass
class Sequence:
    steps: list
    name: str = ""
    gripper_start: str = "open"

    @property
    def point_names(self):
        """Alle referenzierten Punktnamen in Reihenfolge, ohne Dubletten."""
        return list(dict.fromkeys(step.point for step in self.steps))


@dataclass
class ResolvedSequence:
    """Ergebnis der Aufloesung fuer EINE Episode."""

    waypoints: list
    #: Name -> {"joints": [...], "pose_quat": [...]} -- die tatsaechlich
    #: verwendeten Koordinaten, fuer die Episoden-Metadaten.
    points: dict = field(default_factory=dict)
    #: Uebersprungene optionale Punkte.
    skipped: list = field(default_factory=list)

    @property
    def start_joints(self):
        return self.waypoints[0].joints


def parse_sequence(data):
    """dict (geladenes JSON) -> :class:`Sequence`, mit Validierung."""
    if not isinstance(data, dict) or "sequence" not in data:
        raise SequenceError("Ablaufdatei braucht ein Objekt mit Schluessel 'sequence'")
    raw_steps = data["sequence"]
    if not isinstance(raw_steps, list) or len(raw_steps) < 2:
        raise SequenceError("'sequence' braucht mindestens zwei Schritte")

    gripper_start = data.get("gripper_start", "open")
    if gripper_start not in _GRIPPER_VALUES:
        raise SequenceError("'gripper_start' muss 'open' oder 'close' sein")

    steps = []
    for i, raw in enumerate(raw_steps):
        where = "Schritt %d" % i
        if not isinstance(raw, dict) or not isinstance(raw.get("point"), str):
            raise SequenceError("%s: braucht einen Punktnamen ('point')" % where)
        where = "Schritt %d (%s)" % (i, raw["point"])
        unknown = set(raw) - _STEP_KEYS
        if unknown:
            raise SequenceError("%s: unbekannte Schluessel %s" % (where, sorted(unknown)))
        step = SequenceStep(
            point=raw["point"],
            motion=raw.get("motion", MOTION_PTP),
            approach=bool(raw.get("approach", False)),
            gripper=raw.get("gripper"),
            optional=bool(raw.get("optional", False)),
        )
        if step.motion not in MOTIONS:
            raise SequenceError(
                "%s: motion muss eines von %s sein" % (where, ", ".join(MOTIONS))
            )
        if step.gripper is not None and step.gripper not in _GRIPPER_VALUES:
            raise SequenceError("%s: gripper muss 'open' oder 'close' sein" % where)
        steps.append(step)

    for label, step in (("erste", steps[0]), ("letzte", steps[-1])):
        if step.optional:
            raise SequenceError(
                "Der %s Schritt (%s) darf nicht optional sein -- er legt "
                "Start bzw. Uebergabepunkt fest." % (label, step.point)
            )
    return Sequence(steps=steps, name=str(data.get("name", "")), gripper_start=gripper_start)


def load_sequence(path):
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SequenceError("%s ist kein gueltiges JSON: %s" % (path, exc)) from exc
    return parse_sequence(data)


def resolve_sequence(sequence, point_source):
    """Fragt die Koordinaten aller Punkte ab und baut die Wegpunkte.

    Pro Episode aufrufen -- so wirken Touch-ups sofort.
    """
    available = set(point_source.point_names())
    missing = [
        s.point for s in sequence.steps if not s.optional and s.point not in available
    ]
    if missing:
        raise SequenceError(
            "Pflichtpunkte fehlen in der Punkte-Datenbank: %s (vorhanden: %s)"
            % (", ".join(dict.fromkeys(missing)), ", ".join(sorted(available)))
        )

    cache = {}
    waypoints, skipped = [], []
    closed = _GRIPPER_VALUES[sequence.gripper_start]
    for step in sequence.steps:
        if step.point not in available:
            skipped.append(step.point)
            continue
        if step.point not in cache:
            joints, pose = point_source.get_point(step.point)
            cache[step.point] = (np.asarray(joints, dtype=float), np.asarray(pose, dtype=float))
        joints, pose = cache[step.point]
        if step.gripper is not None:
            closed = _GRIPPER_VALUES[step.gripper]
        waypoints.append(
            Waypoint(
                pose.copy(),
                gripper_closed=closed,
                approach=step.approach,
                name=step.point,
                motion=step.motion,
                joints=joints.copy(),
            )
        )

    points = {
        name: {"joints": joints.tolist(), "pose_quat": pose.tolist()}
        for name, (joints, pose) in cache.items()
    }
    return ResolvedSequence(waypoints=waypoints, points=points, skipped=skipped)

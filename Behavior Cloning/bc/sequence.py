"""Ablaufdatei: generische Punktfolge fuer die Datenaufzeichnung (AP 2.2).

Die Pipeline kennt den ABLAUF nur als Folge von Punktnamen. Die Koordinaten
kommen beim Start jedes Aufrufs frisch aus einer
:class:`bc.ports.PointSourcePort` (am Neura: die Punkte-Datenbank der
Control-Box). Wird ein Punkt am Pendant per Touch-up nachgeteacht, faehrt
schon der naechste Aufruf die neue Lage an -- ohne Export und ohne
Abhaengigkeit vom Namen des Neura-Programms. Seit 2026-10-23 plant
``apps/record.py`` alle Episoden eines Aufrufs vorab; ein Touch-up WAEHREND
des Laufs beendet ihn (``same_points``), statt mit altem Plan weiterzufahren.

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
* ``blend``: Ueberschleif-Radius in m an diesem Punkt -- der Arm haelt dort
  nicht an, sondern verrundet die Ecke (trajectory.py). Nicht erlaubt am
  ersten und letzten Schritt und zusammen mit ``gripper``. Sinnvoll an
  reinen Durchfahrpunkten (APPROACH_*, PRE_GRASP nach dem Anheben).
  Vor einer LIN-Anfahrt ans Objekt (PRE_GRASP im Anflug) nur mit KLEINEM
  Radius: Entscheidung 2026-09-23, 10 mm. Ein Halt dort war die Ursache des
  Stillstands-Deadlocks im Policy-Durchstich (AP 2.6) -- im Zustand ist
  "angekommen" nicht von "losfahren" zu unterscheiden. Nachgerechnet mit den
  VM-Punkten (PTP-Anflug, 145 mm LIN-Abstieg): Ecke um 0,4 mm verfehlt, ab
  1,6 mm unter PRE_GRASP exakt auf der Abstiegsachse, Orientierung 0,02 Grad,
  Mindestgeschwindigkeit 0,028 m/s statt Stillstand. Groessere Radien sind
  NICHT besser (40 mm: 0,013 m/s, laengere Ueberlappung = tiefer in beiden
  Rampen). PRE_GRASP muss mindestens 2 x Radius ueber PICK liegen, sonst
  kappt trajectory.py den Radius auf die halbe Segmentlaenge.
* ``above`` + ``dz``: BERECHNETER Punkt, nicht geteacht -- ``dz`` Meter
  senkrecht (Basis-z) ueber dem Punkt ``above``, gleiche Orientierung,
  Gelenke per IK, geseedet mit denen des Bezugspunkts. Ein Name braucht die
  Angabe nur an einer Stelle; weitere Schritte mit demselben Namen meinen
  denselben Punkt. Ein gleichnamiger Punkt in der Datenbank wird dann
  NICHT benutzt.

  Festlegung Anwender 2026-10-26: Je Objektlage wird nur PICK geteacht,
  PRE_GRASP und der Anflugpunkt werden berechnet. Grund (Auswertung
  23./26.10.): Die geteachten APPROACH_01/02 lagen je Lage 30-68 mm neben
  der Geraden CLEAR_FOV -> PRE_GRASP, an willkuerlicher Stelle (32-78 %),
  PRE_GRASP 34-62 mm ueber PICK. Diese Unterschiede haengen nicht an der
  Objektlage, sondern am Teachen -- fuer die Policy widerspruechliche
  Labels, und Teach-Aufwand. Berechnet ist die Bahn eine eindeutige
  Funktion der Objektlage und kommt immer senkrecht von oben.
* ``"table"`` (oben, neben ``sequence``, optional)::

      "table": {"points": ["ID0", "ID1", "ID2", "ID3"],
                "pick_height": 0.008, "keepout": ["IGNORE"]}

  Tischpunkte, 1 mm ueber dem Tisch geteacht (mittig ueber den
  ArUco-Markern). Daraus die Tischebene (bc/table.py): kein Punkt darf
  darunter oder zu nah daran liegen, das Kollisionsmodell prueft die
  Werkzeugspitze dagegen, und "PICK ausrichten" setzt PICK auf
  ``pick_height`` ueber den Tisch. Fehlen die Punkte in der Datenbank,
  gilt der Tisch als unbekannt -- wie bisher. ``keepout``: Punkte (Name
  oder ``{"point": NAME, "size": m}`` mit eigener halber Kantenlaenge), um die
  herum (+-``config.KEEPOUT_HALF_SIZE_M``, eigene Groesse mit
  ``keepout_size``) nicht gegriffen wird -- dort muss das Werkzeug
  ``config.KEEPOUT_MIN_HEIGHT_M`` ueber dem Tisch bleiben (Schraube).
"""

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import config
from .table import Keepout, fit_table
from .trajectory import MOTION_PTP, MOTIONS, Waypoint

_GRIPPER_VALUES = {"close": True, "open": False}
_STEP_KEYS = {"point", "motion", "approach", "gripper", "optional", "blend",
              "above", "dz"}


class SequenceError(ValueError):
    """Ablaufdatei ungueltig oder nicht gegen die Punkte aufloesbar."""


@dataclass
class SequenceStep:
    point: str
    motion: str = MOTION_PTP
    approach: bool = False
    gripper: str = None
    optional: bool = False
    blend: float = 0.0
    #: Berechneter Punkt: Name des Bezugspunkts und Abstand darueber (m)
    above: str = None
    dz: float = 0.0


@dataclass
class Sequence:
    steps: list
    name: str = ""
    gripper_start: str = "open"
    #: Tischpunkte (Namen in der Datenbank) und Greifhoehe ueber dem Tisch
    table_points: list = field(default_factory=list)
    pick_height: float = config.PICK_HEIGHT_DEFAULT_M
    #: Sperrzonen: Punktnamen und halbe Kantenlaenge
    keepout_points: list = field(default_factory=list)
    keepout_size: float = config.KEEPOUT_HALF_SIZE_M
    #: eigene halbe Kantenlaenge je Sperrpunkt (sonst ``keepout_size``)
    keepout_sizes: dict = field(default_factory=dict)

    @property
    def point_names(self):
        """Alle referenzierten Punktnamen in Reihenfolge, ohne Dubletten."""
        return list(dict.fromkeys(step.point for step in self.steps))

    @property
    def computed(self):
        """Berechnete Punkte: ``{Name: (Bezugspunkt, dz)}``."""
        return {step.point: (step.above, step.dz) for step in self.steps if step.above}

    @property
    def taught_names(self):
        """Punkte, die in der Datenbank stehen muessen (geteacht), in Reihenfolge."""
        computed = self.computed
        names = [n for n in self.point_names if n not in computed]
        for ref, _dz in computed.values():
            if ref not in computed and ref not in names:
                names.append(ref)
        return names


@dataclass
class ResolvedSequence:
    """Ergebnis der Aufloesung fuer EINE Episode."""

    waypoints: list
    #: Name -> {"joints": [...], "pose_quat": [...]} -- die tatsaechlich
    #: verwendeten Koordinaten, fuer die Episoden-Metadaten.
    points: dict = field(default_factory=dict)
    #: Uebersprungene optionale Punkte.
    skipped: list = field(default_factory=list)
    #: Tischebene (bc/table.TablePlane) oder None; dazu die verwendeten
    #: Tischpunkte wie ``points`` und die, die in der Datenbank fehlen.
    table: object = None
    table_points: dict = field(default_factory=dict)
    table_missing: list = field(default_factory=list)
    #: Sperrzonen (bc/table.Keepout) und ihre Punkte wie ``table_points``
    keepouts: list = field(default_factory=list)
    keepout_points: dict = field(default_factory=dict)

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
        blend = raw.get("blend", 0.0)
        if isinstance(blend, bool) or not isinstance(blend, (int, float)) or blend < 0:
            raise SequenceError("%s: blend muss ein Radius in m >= 0 sein" % where)
        step.blend = float(blend)
        if step.blend > 0 and step.gripper is not None:
            raise SequenceError(
                "%s: blend und gripper schliessen sich aus -- am Greifpunkt "
                "wird angehalten." % where
            )
        if ("above" in raw) != ("dz" in raw):
            raise SequenceError("%s: berechneter Punkt braucht 'above' UND 'dz'" % where)
        if "above" in raw:
            dz = raw["dz"]
            if not isinstance(raw["above"], str) or not raw["above"].strip():
                raise SequenceError("%s: 'above' braucht einen Punktnamen" % where)
            if isinstance(dz, bool) or not isinstance(dz, (int, float)) or not 0 < dz <= 1.0:
                raise SequenceError("%s: dz muss ein Abstand in m sein, 0 < dz <= 1" % where)
            if raw["above"] == step.point:
                raise SequenceError("%s: ein Punkt kann nicht ueber sich selbst liegen" % where)
            if step.optional:
                raise SequenceError("%s: ein berechneter Punkt ist nie optional" % where)
            step.above, step.dz = raw["above"].strip(), float(dz)
        steps.append(step)

    # Berechnete Punkte: je Name eine Angabe, keine Zirkel
    definitions = {}
    for step in steps:
        if step.above is None:
            continue
        known = definitions.setdefault(step.point, (step.above, step.dz))
        if known != (step.above, step.dz):
            raise SequenceError(
                "%s ist zweimal verschieden berechnet (%s + %g m / %s + %g m)"
                % (step.point, known[0], known[1], step.above, step.dz))
    for name in definitions:
        seen, current = [name], name
        while current in definitions:
            current = definitions[current][0]
            if current in seen:
                raise SequenceError("Berechnete Punkte im Kreis: %s"
                                    % " -> ".join(seen + [current]))
            seen.append(current)

    for label, step in (("erste", steps[0]), ("letzte", steps[-1])):
        if step.optional:
            raise SequenceError(
                "Der %s Schritt (%s) darf nicht optional sein -- er legt "
                "Start bzw. Uebergabepunkt fest." % (label, step.point)
            )
        if step.blend > 0:
            raise SequenceError(
                "Der %s Schritt (%s) darf nicht ueberschliffen werden -- dort "
                "steht der Arm." % (label, step.point)
            )
    table = data.get("table") or {}
    if not isinstance(table, dict):
        raise SequenceError("'table' muss ein Objekt sein ({\"points\": [...], ...})")
    unknown = set(table) - {"points", "pick_height", "keepout", "keepout_size"}
    if unknown:
        raise SequenceError("'table': unbekannte Schluessel %s" % sorted(unknown))
    table_points = table.get("points") or []
    if not isinstance(table_points, list) or not all(
            isinstance(n, str) and n.strip() for n in table_points):
        raise SequenceError("'table.points' braucht eine Liste von Punktnamen")
    pick_height = table.get("pick_height", config.PICK_HEIGHT_DEFAULT_M)
    if isinstance(pick_height, bool) or not isinstance(pick_height, (int, float)) \
            or not 0 < pick_height <= 0.1:
        raise SequenceError("'table.pick_height' muss eine Hoehe in m sein, 0 < h <= 0.1")
    keepout, keepout_sizes = [], {}
    raw_keepout = table.get("keepout") or []
    if not isinstance(raw_keepout, list):
        raise SequenceError("'table.keepout' braucht eine Liste von Punktnamen")
    for entry in raw_keepout:
        size = None
        if isinstance(entry, dict):
            entry, size = entry.get("point"), entry.get("size")
            if isinstance(size, bool) or not isinstance(size, (int, float)) \
                    or not 0 < size <= 0.5:
                raise SequenceError("'table.keepout': size muss eine halbe "
                                    "Kantenlaenge in m sein (0 < size <= 0.5)")
        if not isinstance(entry, str) or not entry.strip():
            raise SequenceError("'table.keepout' braucht Punktnamen")
        keepout.append(entry.strip())
        if size is not None:
            keepout_sizes[entry.strip()] = float(size)
    keepout_size = table.get("keepout_size", config.KEEPOUT_HALF_SIZE_M)
    if isinstance(keepout_size, bool) or not isinstance(keepout_size, (int, float)) \
            or not 0 < keepout_size <= 0.5:
        raise SequenceError("'table.keepout_size' muss eine halbe Kantenlaenge in m sein")
    clash = [n for n in list(table_points) + list(keepout) if any(s.point == n for s in steps)]
    if clash:
        raise SequenceError("Tisch- und Sperrpunkte duerfen nicht im Ablauf vorkommen: %s"
                            % clash)
    return Sequence(steps=steps, name=str(data.get("name", "")), gripper_start=gripper_start,
                    table_points=[n.strip() for n in table_points],
                    pick_height=float(pick_height),
                    keepout_points=keepout,
                    keepout_size=float(keepout_size),
                    keepout_sizes=keepout_sizes)


def load_sequence(path):
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SequenceError("%s ist kein gueltiges JSON: %s" % (path, exc)) from exc
    return parse_sequence(data)


def resolve_sequence(sequence, point_source):
    """Fragt die Koordinaten aller Punkte ab und baut die Wegpunkte.

    Je Aufruf vor der Planung; zwischen den Episoden erneut, um Touch-ups
    waehrend des Laufs zu erkennen (:func:`same_points`).
    """
    computed = sequence.computed
    available = set(point_source.point_names())
    missing = [
        s.point for s in sequence.steps
        if not s.optional and s.point not in computed and s.point not in available
    ] + [ref for ref, _dz in computed.values()
         if ref not in computed and ref not in available]
    if missing:
        raise SequenceError(
            "Pflichtpunkte fehlen in der Punkte-Datenbank: %s (vorhanden: %s)"
            % (", ".join(dict.fromkeys(missing)), ", ".join(sorted(available)))
        )

    cache = {}

    def lookup(name):
        if name not in cache:
            if name in computed:
                cache[name] = _computed_point(name, computed[name], lookup, point_source)
            else:
                joints, pose = point_source.get_point(name)
                cache[name] = (np.asarray(joints, dtype=float), np.asarray(pose, dtype=float))
        return cache[name]

    waypoints, skipped = [], []
    closed = _GRIPPER_VALUES[sequence.gripper_start]
    for step in sequence.steps:
        if step.point not in available and step.point not in computed:
            skipped.append(step.point)
            continue
        joints, pose = lookup(step.point)
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
                blend_m=step.blend,
            )
        )

    points = {
        name: {"joints": joints.tolist(), "pose_quat": pose.tolist()}
        for name, (joints, pose) in cache.items()
    }
    for name, (ref, dz) in computed.items():
        if name in points:
            points[name]["computed"] = {"above": ref, "dz": dz}

    # Tischpunkte: optional -- was fehlt, wird genannt, nicht verlangt
    table_points, table_missing = {}, []
    for name in sequence.table_points:
        if name not in available:
            table_missing.append(name)
            continue
        joints, pose = point_source.get_point(name)
        table_points[name] = {"joints": [float(v) for v in joints],
                              "pose_quat": [float(v) for v in pose]}
    table = fit_table([p["pose_quat"] for p in table_points.values()])
    keepout_points, keepouts = {}, []
    for name in sequence.keepout_points:
        if name not in available:
            table_missing.append(name)
            continue
        joints, pose = point_source.get_point(name)
        keepout_points[name] = {"joints": [float(v) for v in joints],
                                "pose_quat": [float(v) for v in pose]}
        keepouts.append(Keepout(name, (float(pose[0]), float(pose[1])),
                                half_size=sequence.keepout_sizes.get(
                                    name, sequence.keepout_size)))
    return ResolvedSequence(waypoints=waypoints, points=points, skipped=skipped,
                            table=table, table_points=table_points,
                            table_missing=table_missing, keepouts=keepouts,
                            keepout_points=keepout_points)


def _computed_point(name, definition, lookup, robot):
    """Pose ``dz`` senkrecht ueber dem Bezugspunkt, Gelenke per IK.

    Geseedet mit den Gelenken des Bezugspunkts -- die IK liefert die
    naechstgelegene Loesung, ein Zweigwechsel faellt an der Schranke auf.
    """
    ref, dz = definition
    ref_joints, ref_pose = lookup(ref)
    pose = ref_pose.copy()
    pose[2] += dz
    try:
        joints = np.asarray(robot.ik(pose, ref_joints), dtype=float)
    except Exception as exc:
        raise SequenceError(
            "Berechneter Punkt %s (%.0f mm ueber %s) nicht erreichbar: %s"
            % (name, dz * 1e3, ref, exc)) from exc
    jump = float(np.max(np.abs(joints - ref_joints)))
    if jump > config.COMPUTED_POINT_MAX_JUMP_RAD:
        raise SequenceError(
            "Berechneter Punkt %s: IK springt %.2f rad weg von %s (Grenze %.2f) "
            "-- anderer Gelenkzweig? Abstand verkleinern oder %s teachen."
            % (name, jump, ref, config.COMPUTED_POINT_MAX_JUMP_RAD, name))
    return joints, pose


def same_points(a, b, tol=1e-6):
    """Gleiche Punkte (Namen und Gelenke) in zwei ``ResolvedSequence.points``?"""
    if set(a) != set(b):
        return False
    return all(
        np.allclose(a[name]["joints"], b[name]["joints"], rtol=0.0, atol=tol)
        for name in a
    )


def approach_problems(waypoints,
                      min_drop=config.APPROACH_MIN_DROP_M,
                      max_lateral=config.APPROACH_MAX_LATERAL_M):
    """Plausibilitaet der Endanfluege, VOR jeder Bewegung (AP 2.2/4.2).

    Ein ``approach``-Punkt (PICK) wird von oben angefahren: der Punkt davor
    (PRE_GRASP) muss mindestens ``min_drop`` hoeher liegen und darf seitlich
    hoechstens ``max_lateral`` versetzt sein. Anlage 2026-10-23: vertauscht
    geteacht lag PRE_GRASP 49 mm UNTER PICK -- der Arm fuhr von unten an und
    stiess an. IK und Kollisionsmodell lassen das durch (erreichbar, Tisch
    weit darunter), die Geometrie des Ablaufs nicht.

    Rueckgabe: Liste von Textzeilen, leer wenn alles passt.
    """
    out = []
    for k in range(1, len(waypoints)):
        wp, prev = waypoints[k], waypoints[k - 1]
        if not wp.approach:
            continue
        drop = float(prev.pose_quat[2] - wp.pose_quat[2])
        lateral = float(np.linalg.norm(prev.pose_quat[:2] - wp.pose_quat[:2]))
        name = wp.name or "Punkt %d" % k
        prev_name = prev.name or "Punkt %d" % (k - 1)
        if drop < min_drop:
            out.append(
                "Endanflug %s -> %s kommt nicht von oben: %s liegt %.0f mm %s %s "
                "(mindestens %.0f mm darueber erwartet). Punkte vertauscht?"
                % (prev_name, name, prev_name, abs(drop) * 1e3,
                   "ueber" if drop >= 0 else "UNTER", name, min_drop * 1e3))
        if lateral > max_lateral:
            out.append(
                "Endanflug %s -> %s nicht senkrecht: %.0f mm seitlicher Versatz "
                "(hoechstens %.0f mm). %s senkrecht ueber %s teachen."
                % (prev_name, name, lateral * 1e3, max_lateral * 1e3, prev_name, name))
    return out

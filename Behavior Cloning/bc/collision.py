"""Quader-Kollisionsmodell der Umgebung (AP 2.4 / AP 4.2).

NeuraPy bringt weder ein Umgebungs-Kollisionsmodell noch Geofencing mit
(im Funktionsindex existiert keine Workspace-Boundary-Funktion). Beides
muss daher hier abgebildet werden -- und zwar mit demselben Modell:

* **offline** als Vorabpruefung jeder verrauschten Trajektorie, bevor sie
  physisch ausgefuehrt wird (Rejection Sampling),
* **online** als Geofencing-Waechter waehrend Aufzeichnung und Inferenz.

Die Stuetzpunkte entlang des Arms liefert ``RobotPort.link_positions()`` --
das Modell selbst kennt keine Roboter-API, nur Namen, Punkte und Radien
(Schichtentrennung AP 0.3).
"""

import numpy as np

from . import config


class Box(object):
    """Achsparalleler Quader in Weltkoordinaten.

    ``center`` und ``half_extents`` in Metern. ``solid=True`` bedeutet
    "Hindernis, da darf nichts hinein" (Tisch, Kistenwand). ``solid=False``
    kehrt die Logik um: der Arm muss INNERHALB bleiben -- damit laesst sich
    derselbe Typ als Arbeitsraumbegrenzung (Geofence) verwenden.
    """

    __slots__ = ("name", "center", "half_extents", "solid")

    def __init__(self, name, center, half_extents, solid=True):
        self.name = name
        self.center = np.asarray(center, dtype=float)
        self.half_extents = np.asarray(half_extents, dtype=float)
        self.solid = solid
        if self.center.shape != (3,) or self.half_extents.shape != (3,):
            raise ValueError("center und half_extents brauchen je 3 Werte")
        if np.any(self.half_extents < 0):
            raise ValueError("half_extents duerfen nicht negativ sein")

    @classmethod
    def from_bounds(cls, name, lo, hi, solid=True):
        """Erzeugt einen Quader aus Minimal- und Maximalecke."""
        lo = np.asarray(lo, dtype=float)
        hi = np.asarray(hi, dtype=float)
        return cls(name, (lo + hi) / 2.0, (hi - lo) / 2.0, solid=solid)

    def distance_to_point(self, point):
        """Abstand eines Punktes zur Quaderoberflaeche.

        Positiv ausserhalb, 0 auf der Oberflaeche, negativ innerhalb
        (Betrag = Abstand zur naechsten Flaeche).
        """
        point = np.asarray(point, dtype=float)
        delta = np.abs(point - self.center) - self.half_extents
        outside = np.linalg.norm(np.maximum(delta, 0.0))
        if outside > 0.0:
            return outside
        return float(np.max(delta))  # negativ

    def clearance(self, point, radius):
        """Freiraum einer Huellkugel gegenueber diesem Quader.

        Positiv = Abstand bleibt gewahrt, negativ = Durchdringung.
        Beruecksichtigt automatisch ``solid``.
        """
        d = self.distance_to_point(point)
        if self.solid:
            return d - radius
        # Geofence: der Punkt muss drin bleiben, -d ist der Abstand zur Wand
        return -d - radius


class ArmPoint(object):
    """Ein Stuetzpunkt am Arm: Name aus link_positions() plus Huellradius."""

    __slots__ = ("frame", "radius")

    def __init__(self, frame, radius):
        self.frame = frame
        self.radius = radius

    def __repr__(self):
        return "ArmPoint(%s, r=%.3f)" % (self.frame, self.radius)


#: Standard-Stuetzpunkte. Radien sind grobe Huellgeometrien und sollten
#: nach dem ersten Aufbau anhand der realen Abmessungen nachgezogen werden.
DEFAULT_ARM_POINTS = (
    ArmPoint("tool", 0.06),
    ArmPoint("wrist", 0.08),
    ArmPoint("elbow", 0.09),
)


class Violation(object):
    """Eine konkrete Verletzung: welcher Schritt, welcher Punkt, welcher Quader."""

    __slots__ = ("index", "frame", "box", "clearance", "position")

    def __init__(self, index, frame, box, clearance, position):
        self.index = index
        self.frame = frame
        self.box = box
        self.clearance = clearance
        self.position = position

    def __repr__(self):
        return (
            "Violation(Schritt %s, Frame '%s', Quader '%s', Freiraum %.4f m, "
            "Position %s)"
            % (
                self.index,
                self.frame,
                self.box,
                self.clearance,
                np.round(self.position, 4).tolist(),
            )
        )


class CollisionModel(object):
    """Sammlung von Quadern plus Huellgeometrie des Arms."""

    def __init__(
        self,
        boxes=(),
        arm_points=DEFAULT_ARM_POINTS,
        margin=config.COLLISION_MARGIN_M,
    ):
        self.boxes = list(boxes)
        self.arm_points = list(arm_points)
        self.margin = margin

    def add_box(self, box):
        self.boxes.append(box)
        return self

    # -- Pruefungen --------------------------------------------------------

    def check_positions(self, positions, index=None):
        """Prueft einen Satz benannter Punkte (Kern der Pruefung).

        ``positions``: dict Name -> Punkt(3,), z. B. das Ergebnis von
        ``RobotPort.link_positions()``. Es werden nur die Namen geprueft,
        fuer die ein :class:`ArmPoint` konfiguriert ist.

        Rueckgabe: Liste von :class:`Violation` (leer = frei).
        """
        violations = []
        for point in self.arm_points:
            pos = positions.get(point.frame)
            if pos is None:
                raise KeyError(
                    "link_positions() liefert keinen Punkt '%s' "
                    "(vorhanden: %s)" % (point.frame, sorted(positions))
                )
            for box in self.boxes:
                clearance = box.clearance(pos, point.radius) - self.margin
                if clearance < 0.0:
                    violations.append(
                        Violation(index, point.frame, box.name, clearance, pos)
                    )
        return violations

    def check_joints(self, robot, joints, index=None):
        """Prueft eine Gelenkkonfiguration ueber ``robot.link_positions()``.

        Beim NeuraPy-Adapter kostet das einen FK-Roundtrip je Stuetzpunkt
        (~2 ms, tools/log.txt), also rund 6 ms pro Konfiguration.
        """
        return self.check_positions(robot.link_positions(joints), index=index)

    def check_path(self, robot, joint_path, stop_at_first=True, stride=1):
        """Prueft eine ganze Gelenkraum-Trajektorie.

        ``stride`` erlaubt Grobpruefung mit anschliessender Feinpruefung --
        bei langen Bahnen deutlich schneller. Vorsicht: ein zu grosser
        Stride kann kurze Durchdringungen uebersehen.

        Rueckgabe: Liste von :class:`Violation`.
        """
        joint_path = np.asarray(joint_path, dtype=float)
        found = []
        for i in range(0, len(joint_path), stride):
            violations = self.check_joints(robot, joint_path[i], index=i)
            if violations:
                found.extend(violations)
                if stop_at_first:
                    return found
        return found

    def is_path_free(self, robot, joint_path, stride=1):
        return not self.check_path(robot, joint_path, stop_at_first=True, stride=stride)


def default_workspace(table_height_m, geofence=None):
    """Minimales Startmodell: Tischplatte plus optionale Arbeitsraumgrenze.

    Bewusst klein gehalten -- Kiste, Vorrichtung und weitere Hindernisse
    kommen dazu, sobald der physische Aufbau steht. Die Tischplatte wird als
    grosser flacher Quader unterhalb ``table_height_m`` modelliert.

    ``geofence`` ist optional ein (lo, hi)-Paar, das den erlaubten
    Arbeitsraum begrenzt (AP 4.2).
    """
    model = CollisionModel()
    model.add_box(
        Box.from_bounds(
            "Tisch",
            lo=(-1.5, -1.5, table_height_m - 0.5),
            hi=(1.5, 1.5, table_height_m),
            solid=True,
        )
    )
    if geofence is not None:
        lo, hi = geofence
        model.add_box(Box.from_bounds("Arbeitsraum", lo=lo, hi=hi, solid=False))
    return model

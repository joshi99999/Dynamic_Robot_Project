"""Tischreferenz aus geteachten Punkten (AP 2.2/4.2, Anwender 2026-10-26).

Der Anwender teacht einige Punkte knapp (``config.TABLE_POINT_OFFSET_M``)
ueber dem Tisch, mittig ueber den ArUco-Markern. Daraus:

* **Tischebene** ``z = a*x + b*y + c`` (ab drei Punkten, sonst waagerecht).
  Bisher lag der "Tisch" im Kollisionsmodell pauschal 15 cm unter dem
  tiefsten Wegpunkt und fing nichts ab.
* **Untergrenze**: Kein Wegpunkt und keine Werkzeugspitze darf naeher als
  ``config.TABLE_MIN_CLEARANCE_M`` an die Ebene -- vor jeder Bewegung
  (:func:`table_problems`) und im Kollisionsmodell (:class:`TableSurface`).
* **Ausrichten** von PICK (:func:`align_pose`): Werkzeugachse senkrecht zur
  Tischebene, Drehung um die Hochachse wie geteacht, Hoehe = feste
  Greifhoehe ueber dem Tisch. Begruendung: Neigung (gemessen 0,5-1,3 Grad)
  und Hoehe (5-11 mm) schwankten beim Teachen, ohne dass die Kamera einen
  Grund dafuer sieht -- fuer die Policy widerspruechliche Labels genau an
  der Stelle, an der das Rauschen bewusst auf null gedaempft ist.

Hardwarefrei; Posen als [x, y, z, qw, qx, qy, qz].
"""

from dataclasses import dataclass

import numpy as np

from . import config, geometry


@dataclass
class TablePlane:
    """Tischoberflaeche ``z = a*x + b*y + c`` im Basis-KS."""

    a: float
    b: float
    c: float
    #: Groesste Abweichung der Punkte von der Ebene (m) -- Pruefgroesse
    residual_m: float = 0.0
    #: Anzahl der Punkte, aus denen sie bestimmt wurde
    count: int = 0

    def height_at(self, x, y):
        return self.a * float(x) + self.b * float(y) + self.c

    @property
    def normal(self):
        """Einheitsnormale nach oben."""
        n = np.array([-self.a, -self.b, 1.0])
        return n / np.linalg.norm(n)

    @property
    def tilt_deg(self):
        return float(np.degrees(np.arccos(self.normal[2])))

    def describe(self):
        return ("Tisch aus %d Punkt(en): Hoehe %.1f mm am Ursprung, Neigung %.2f Grad, "
                "groesste Abweichung %.1f mm"
                % (self.count, 1e3 * self.c, self.tilt_deg, 1e3 * self.residual_m))

    def as_dict(self):
        return {"a": self.a, "b": self.b, "c": self.c, "residual_m": self.residual_m,
                "count": self.count, "tilt_deg": self.tilt_deg}


def fit_table(positions, offset=config.TABLE_POINT_OFFSET_M):
    """Ebene durch die Punkte (TCP-Positionen), um ``offset`` nach unten versetzt.

    Ein oder zwei Punkte: waagerecht auf ihrer mittleren Hoehe. Ab drei:
    kleinste Quadrate. None ohne Punkte.
    """
    pts = np.asarray([p[:3] for p in positions], dtype=float).reshape(-1, 3)
    if len(pts) == 0:
        return None
    if len(pts) < 3:
        c = float(np.mean(pts[:, 2])) - offset
        residual = float(np.max(np.abs(pts[:, 2] - offset - c)))
        return TablePlane(0.0, 0.0, c, residual, len(pts))
    design = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
    (a, b, c), *_ = np.linalg.lstsq(design, pts[:, 2] - offset, rcond=None)
    residual = float(np.max(np.abs(design @ np.array([a, b, c]) - (pts[:, 2] - offset))))
    return TablePlane(float(a), float(b), float(c), residual, len(pts))


def clearance(plane, position):
    """Hoehe eines Punkts ueber der Tischebene (m)."""
    return float(position[2]) - plane.height_at(position[0], position[1])


def table_problems(waypoints, plane, min_clearance=config.TABLE_MIN_CLEARANCE_M,
                   max_residual=config.TABLE_MAX_RESIDUAL_M):
    """Textzeilen fuer Punkte zu nah am (oder unter dem) Tisch. Leer = in Ordnung."""
    if plane is None:
        return []
    out = []
    if plane.residual_m > max_residual:
        out.append("Tischpunkte liegen nicht in einer Ebene (Abweichung %.1f mm, "
                   "erlaubt %.1f) -- einen davon falsch geteacht?"
                   % (1e3 * plane.residual_m, 1e3 * max_residual))
    seen = set()
    for wp in waypoints:
        name = wp.name or "?"
        if name in seen:
            continue
        seen.add(name)
        height = clearance(plane, wp.pose_quat)
        if height < min_clearance:
            out.append("%s liegt %.1f mm %s dem Tisch (mindestens %.1f mm darueber)"
                       % (name, abs(1e3 * height), "ueber" if height >= 0 else "UNTER",
                          1e3 * min_clearance))
    return out


def align_pose(pose_quat, plane=None, pick_height=None):
    """PICK ausrichten: Werkzeugachse senkrecht zur Tischebene, Drehung um die
    Hochachse und x/y bleiben, Hoehe = ``pick_height`` ueber dem Tisch.

    Ohne Tisch: senkrecht zum Basis-KS, Hoehe bleibt. Welche Richtung der
    Werkzeugachse "unten" ist, wird aus der geteachten Pose uebernommen.
    """
    pose = np.asarray(pose_quat, dtype=float).copy()
    original = pose[3:].copy()
    rot = geometry.quat_to_matrix(original)
    up = plane.normal if plane is not None else np.array([0.0, 0.0, 1.0])
    z_new = up if float(np.dot(rot[:, 2], up)) >= 0 else -up
    # x-Achse des Werkzeugs in die Ebene senkrecht zu z_new projizieren:
    # so bleibt die Drehung um die Hochachse erhalten
    x_old = rot[:, 0]
    x_new = x_old - np.dot(x_old, z_new) * z_new
    if np.linalg.norm(x_new) < 1e-6:  # x zeigte senkrecht -- dann y nehmen
        y_old = rot[:, 1]
        y_new = y_old - np.dot(y_old, z_new) * z_new
        y_new /= np.linalg.norm(y_new)
        x_new = np.cross(y_new, z_new)
    x_new /= np.linalg.norm(x_new)
    y_new = np.cross(z_new, x_new)
    pose[3:] = geometry.quat_canonical(
        geometry.matrix_to_quat(np.column_stack([x_new, y_new, z_new])), original)
    if plane is not None and pick_height is not None:
        pose[2] = plane.height_at(pose[0], pose[1]) + float(pick_height)
    return pose


def yaw_deg(pose_quat, plane=None):
    """Drehung des Greifers um die Tischnormale (Grad, 0..180).

    Winkel der Werkzeug-x-Achse, in die Tischebene projiziert, gegen die
    Basis-x-Achse. Modulo 180: ein Parallelgreifer greift bei 10 und 190
    Grad gleich -- als Objektorientierung zaehlt nur die Achse.
    """
    rot = geometry.quat_to_matrix(np.asarray(pose_quat, dtype=float)[3:])
    up = plane.normal if plane is not None else np.array([0.0, 0.0, 1.0])
    x = rot[:, 0] - np.dot(rot[:, 0], up) * up
    return float(np.degrees(np.arctan2(x[1], x[0])) % 180.0)


def tilt_deg(pose_quat, plane=None):
    """Neigung der Werkzeugachse gegen die Tischnormale (Grad)."""
    rot = geometry.quat_to_matrix(np.asarray(pose_quat, dtype=float)[3:])
    up = plane.normal if plane is not None else np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(float(np.dot(rot[:, 2], up)))))))


@dataclass
class Keepout:
    """Sperrzone: Quadrat ``+-half_size`` um ``center`` (x, y) in der Tischebene.

    Darin muss die Werkzeugspitze mindestens ``min_height`` ueber dem Tisch
    bleiben (Anlage 2026-10-26: Schraube im Tisch an Punkt IGNORE).
    """

    name: str
    center: tuple
    half_size: float = config.KEEPOUT_HALF_SIZE_M
    min_height: float = config.KEEPOUT_MIN_HEIGHT_M

    def contains_xy(self, position):
        return (abs(float(position[0]) - self.center[0]) <= self.half_size
                and abs(float(position[1]) - self.center[1]) <= self.half_size)

    def as_dict(self):
        return {"name": self.name, "center": [float(v) for v in self.center],
                "half_size": self.half_size, "min_height": self.min_height}


def keepout_problems(waypoints, plane, zones):
    """Wegpunkte in einer Sperrzone und zu tief. Leer = in Ordnung."""
    out = []
    if not zones:
        return out
    seen = set()
    for wp in waypoints:
        name = wp.name or "?"
        if name in seen:
            continue
        seen.add(name)
        height = (clearance(plane, wp.pose_quat) if plane is not None
                  else float(wp.pose_quat[2]))
        for zone in zones:
            if zone.contains_xy(wp.pose_quat) and (plane is None or height < zone.min_height):
                out.append("%s liegt in der Sperrzone um %s (+-%.0f mm)%s -- dort nicht greifen"
                           % (name, zone.name, 1e3 * zone.half_size,
                              "" if plane is None else ", nur %.0f mm ueber dem Tisch" % (1e3 * height)))
    return out


class KeepoutSurface(object):
    """Sperrzone im Kollisionsmodell -- Werkzeugspitze, nur innerhalb der Zone."""

    margin = 0.0

    def __init__(self, zone, plane):
        self.zone = zone
        self.plane = plane
        self.name = "Sperrzone %s" % zone.name

    def clearance_for(self, frame, position, radius):
        if frame != "tool" or not self.zone.contains_xy(position):
            return None
        height = clearance(self.plane, position) if self.plane is not None else float(position[2])
        return height - self.zone.min_height


class TableSurface(object):
    """Tischebene im Kollisionsmodell -- nur fuer die Werkzeugspitze.

    Die Huellkugel des Werkzeugs (``collision.DEFAULT_ARM_POINTS``, 6 cm)
    plus Sicherheitsabstand (3 cm) passt nicht zu einem Griff wenige mm ueber
    dem Tisch: jede Bahn waere verworfen. Hier zaehlt deshalb nur der TCP
    mit eigenem Mindestabstand; Handgelenk und Ellbogen prueft weiter der
    Tischquader darunter (``collision.default_workspace``).
    """

    def __init__(self, plane, min_clearance=config.TABLE_MIN_CLEARANCE_M,
                 frames=("tool",), name="Tischebene"):
        self.plane = plane
        self.margin = float(min_clearance)
        self.frames = tuple(frames)
        self.name = name

    def clearance_for(self, frame, position, radius):
        """Freiraum (m); None = dieser Stuetzpunkt wird hier nicht geprueft."""
        if frame not in self.frames:
            return None
        return clearance(self.plane, position)

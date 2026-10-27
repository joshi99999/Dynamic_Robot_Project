"""Automatische Objektlagen: der Roboter legt das Objekt selbst um (AP 2.2/2.4).

Wunsch Anwender 2026-10-26: Nach den Fahrten einer Lage legt der Arm das
Objekt nicht an PICK zurueck, sondern an eine NEUE Lage -- neue Position im
Bereich der Tischpunkte, neue Drehung --, und die naechsten Fahrten greifen
dort. Die Lage ist damit exakt bekannt (der Roboter hat sie selbst
hergestellt) und landet als Zahl in den Metadaten. Von Hand geteachte Lagen
bleiben daneben moeglich, vor allem am Rand.

Wie gezogen wird:
* **Bereich**: konvexe Huelle der Tischpunkte (ID0-ID3), um
  ``config.AUTO_EDGE_MARGIN_M`` nach innen geschrumpft.
* **Sperrzonen** (bc/table.Keepout): die neue Greifposition liegt ausserhalb.
* **Abstand**: mindestens ``config.AUTO_MIN_DIST_M`` zu allen bisherigen Lagen
  des Aufrufs -- gleichmaessig ueber die Flaeche statt Haufen.
* **Drehung**: gleichverteilt in +-``yaw_range_deg`` um die geteachte.
* **Hoehe und Neigung**: wie "Ausrichten" (bc/table.align_pose) -- senkrecht
  zur Tischebene, Greifhoehe darueber. So wird an jeder Stelle des schiefen
  Tisches gleich hoch gegriffen und abgelegt.

Erreichbarkeit, Gelenktempo und Kollisionsmodell prueft der Aufrufer wie
bei jeder Lage (apps/record.py); scheitert das, wird neu gezogen.
"""

import numpy as np

from . import config, geometry
from .table import align_pose


class OverridePoints(object):
    """Punktquelle mit ersetzten Punkten -- der Rest kommt aus ``source``.

    So laeuft eine gezogene Lage durch dieselbe Aufloesung wie eine geteachte
    (bc/sequence.resolve_sequence): PRE_PICK/ANFLUG werden daraus berechnet.
    """

    def __init__(self, source, overrides):
        self.source = source
        self.overrides = dict(overrides)

    def point_names(self):
        return list(dict.fromkeys(list(self.source.point_names()) + list(self.overrides)))

    def get_point(self, name):
        if name in self.overrides:
            joints, pose = self.overrides[name]
            return np.asarray(joints, dtype=float), np.asarray(pose, dtype=float)
        return self.source.get_point(name)

    def ik(self, pose, reference):
        return self.source.ik(pose, reference)


def convex_hull(points):
    """Konvexe Huelle (x, y) gegen den Uhrzeigersinn (Andrew)."""
    pts = sorted({(float(x), float(y)) for x, y in points})
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def inside_with_margin(hull, xy, margin):
    """Liegt ``xy`` in der Huelle und mindestens ``margin`` von jeder Kante weg?"""
    if len(hull) < 3:
        return False
    x, y = float(xy[0]), float(xy[1])
    for k in range(len(hull)):
        (x1, y1), (x2, y2) = hull[k], hull[(k + 1) % len(hull)]
        edge = np.array([x2 - x1, y2 - y1])
        length = float(np.linalg.norm(edge))
        if length < 1e-9:
            continue
        # Abstand nach innen (Huelle gegen den Uhrzeigersinn -> links = innen)
        inner = (edge[0] * (y - y1) - edge[1] * (x - x1)) / length
        if inner < margin:
            return False
    return True


def rotated_about(pose, axis, angle_rad):
    """Orientierung von ``pose`` um ``axis`` (Basis-KS) gedreht, Position bleibt."""
    pose = np.asarray(pose, dtype=float).copy()
    turn = geometry.quat_from_axis_angle(np.asarray(axis, dtype=float), float(angle_rad))
    pose[3:] = geometry.quat_canonical(geometry.quat_multiply(turn, pose[3:]), pose[3:])
    return pose


def sample_pose(rng, base_pose, plane, region_points, keepouts=(), previous_xy=(),
                yaw_range_deg=config.AUTO_YAW_RANGE_DEG, pick_height=config.PICK_HEIGHT_DEFAULT_M,
                margin=config.AUTO_EDGE_MARGIN_M, min_dist=config.AUTO_MIN_DIST_M,
                tries=config.AUTO_SAMPLE_TRIES):
    """Eine neue Greifpose -> (Pose, Drehung gegen die geteachte in Grad).

    ``base_pose``: geteachte PICK-Pose (gibt Orientierung vor), ``plane``:
    Tischebene (Pflicht), ``region_points``: Tischpunkte (Posen).
    Wirft ``ValueError``, wenn in ``tries`` Zuegen nichts passt.
    """
    if plane is None:
        raise ValueError("Automatische Lagen brauchen die Tischpunkte (Tischebene)")
    hull = convex_hull([p[:2] for p in region_points])
    if len(hull) < 3:
        raise ValueError("Automatische Lagen brauchen mindestens drei Tischpunkte als Bereich")
    xs, ys = [p[0] for p in hull], [p[1] for p in hull]
    upright = align_pose(base_pose, plane, pick_height)
    for _ in range(int(tries)):
        xy = np.array([rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))])
        if not inside_with_margin(hull, xy, margin):
            continue
        if any(zone.contains_xy(xy) for zone in keepouts):
            continue
        if any(np.linalg.norm(xy - np.asarray(p)) < min_dist for p in previous_xy):
            continue
        turn = float(rng.uniform(-yaw_range_deg, yaw_range_deg))
        pose = rotated_about(upright, plane.normal, np.radians(turn))
        pose[0], pose[1] = xy
        pose[2] = plane.height_at(xy[0], xy[1]) + float(pick_height)
        return pose, turn
    raise ValueError("Keine freie Lage gefunden (%d Versuche) -- Bereich zu klein, "
                     "Mindestabstand zu gross oder Sperrzonen zu breit?" % tries)

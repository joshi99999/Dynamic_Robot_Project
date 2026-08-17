"""Minimaler URDF-Parser fuer die Offline-Kinematik des SimRobot (AP 0.4).

Liest eine serielle Kette aus revoluten Gelenken aus einer URDF-Datei und
stellt Vorwaertskinematik als homogene Transformationen bereit. Bewusst
klein gehalten: keine Meshes, keine Dynamik, keine Baeume mit mehreren
aktiven Zweigen -- nur das, was FK/IK und Kollisionsstuetzpunkte brauchen.

WICHTIG (AP 0.4): Das mitgelieferte Modell ``data/lara5_candidate.urdf``
ist als Strukturquelle brauchbar, numerisch aber UNBESTAETIGT -- die
Gegenprobe mit dem realen Wertepaar aus tools/log.txt ergab ~83 mm
Restfehler. Der SimRobot prueft deshalb LOGIK, nicht Geometrie.
"""

import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_matrix(r, p, y):
    """URDF-rpy (fixed-axis XYZ, entspricht ZYX-Eulerwinkeln) -> 3x3."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_rotation(axis, angle):
    """Rodrigues-Formel: Drehung um eine Achse."""
    v = np.asarray(axis, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.eye(3)
    v = v / n
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


class UrdfJoint(object):
    __slots__ = ("name", "type", "parent", "child", "xyz", "rpy", "axis", "limit")

    def __init__(self, name, jtype, parent, child, xyz, rpy, axis, limit):
        self.name = name
        self.type = jtype
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.rpy = rpy
        self.axis = axis
        self.limit = limit  # (lower, upper) oder None


class KinematicChain(object):
    """Serielle Kette: Basis -> Endeffektor.

    ``joints`` enthaelt fixe UND revolute Gelenke in Kettenreihenfolge;
    ``n_joints`` zaehlt nur die revoluten (= Freiheitsgrade).
    """

    def __init__(self, joints):
        self.joints = joints
        self.revolute = [j for j in joints if j.type == "revolute"]
        self.n_joints = len(self.revolute)

    @property
    def joint_limits(self):
        """((lower, upper), ...) je revolutem Gelenk; None falls unbekannt."""
        lims = []
        for j in self.revolute:
            lims.append(j.limit if j.limit is not None else (None, None))
        return tuple(lims)

    @classmethod
    def from_urdf(cls, path):
        root = ET.parse(str(path)).getroot()
        joints = []
        for el in root.findall("joint"):
            origin = el.find("origin")
            axis = el.find("axis")
            limit = el.find("limit")
            parent = el.find("parent").get("link")
            child = el.find("child").get("link")
            xyz = np.array(
                [float(v) for v in (origin.get("xyz", "0 0 0").split())]
                if origin is not None
                else [0, 0, 0],
                dtype=float,
            )
            rpy = np.array(
                [float(v) for v in (origin.get("rpy", "0 0 0").split())]
                if origin is not None
                else [0, 0, 0],
                dtype=float,
            )
            ax = (
                np.array([float(v) for v in axis.get("xyz").split()], dtype=float)
                if axis is not None
                else None
            )
            lim = None
            if limit is not None and limit.get("lower") is not None:
                lim = (float(limit.get("lower")), float(limit.get("upper")))
            joints.append(
                UrdfJoint(el.get("name"), el.get("type"), parent, child, xyz, rpy, ax, lim)
            )

        return cls(cls._order_chain(joints))

    @staticmethod
    def _order_chain(joints):
        """Ordnet die Gelenke als Kette von der Basis zum Endeffektor.

        Bei Verzweigungen wird der Zweig mit den meisten verbleibenden
        revoluten Gelenken verfolgt (Dummy-/Anbau-Zweige fallen weg).
        """
        by_parent = {}
        children = set()
        for j in joints:
            by_parent.setdefault(j.parent, []).append(j)
            children.add(j.child)
        roots = [j.parent for j in joints if j.parent not in children]
        if not roots:
            raise ValueError("URDF enthaelt keine eindeutige Wurzel")

        def count_revolute(link):
            total = 0
            for j in by_parent.get(link, []):
                total += (1 if j.type == "revolute" else 0) + count_revolute(j.child)
            return total

        chain = []
        link = roots[0]
        while True:
            options = by_parent.get(link, [])
            if not options:
                break
            best = max(options, key=lambda j: (count_revolute(j.child) + (1 if j.type == "revolute" else 0)))
            chain.append(best)
            link = best.child
        return chain

    # -- Vorwaertskinematik ------------------------------------------------

    def fk_frames(self, q):
        """Alle Zwischen-Frames der Kette.

        Rueckgabe: Liste (gelenkname, 4x4-Transformation Basis->Frame) --
        ein Eintrag NACH jedem Gelenk (fix wie revolut), zuletzt der
        Endeffektor.
        """
        q = np.asarray(q, dtype=float)
        if len(q) != self.n_joints:
            raise ValueError(
                "Erwarte %d Gelenkwinkel, bekam %d" % (self.n_joints, len(q))
            )
        T = np.eye(4)
        out = []
        qi = 0
        for j in self.joints:
            A = np.eye(4)
            A[:3, :3] = _rpy_matrix(*j.rpy)
            A[:3, 3] = j.xyz
            T = T @ A
            if j.type == "revolute":
                B = np.eye(4)
                B[:3, :3] = _axis_rotation(j.axis, q[qi])
                qi += 1
                T = T @ B
            out.append((j.name, T.copy()))
        return out

    def fk(self, q):
        """Endeffektor-Transformation (4x4) fuer die Gelenkwinkel ``q``."""
        return self.fk_frames(q)[-1][1]

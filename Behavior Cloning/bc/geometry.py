"""Posen- und Quaternion-Mathematik.

Hintergrund (siehe AP 2.4): Die realen Arbeitsposen liegen mit Roll = Yaw =
-3.1416 rad exakt am +/-pi-Umschlagpunkt der RPY-Darstellung. Lineare
Interpolation, Delta-Bildung und das Daempfen des Rauschens brechen dort,
weil zwei physikalisch identische Orientierungen numerisch um 2*pi
auseinanderliegen koennen.

Deshalb wird der Rotationsanteil projektweit in **Quaternionen** gerechnet
und nur an der Schnittstelle zu NeuraPy bei Bedarf konvertiert.

Posen-Konventionen (wie NeuraPy):
    RPY-Pose  : [X, Y, Z, R, P, Y]        -- 6 Werte, Winkel in rad
    Quat-Pose : [X, Y, Z, QW, QX, QY, QZ] -- 7 Werte (NeuraPy: XYZQWQ1Q2Q3)
"""

import math

import numpy as np

__all__ = [
    "quat_normalize",
    "quat_multiply",
    "quat_conjugate",
    "quat_from_rpy",
    "quat_to_rpy",
    "quat_from_axis_angle",
    "quat_to_matrix",
    "matrix_to_quat",
    "quat_rotate",
    "quat_angle_between",
    "quat_canonical",
    "slerp",
    "pose_rpy_to_quat",
    "pose_quat_to_rpy",
    "pose_interpolate",
    "pose_path",
]


def quat_normalize(q):
    """Normiert ein Quaternion [w, x, y, z]."""
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("Quaternion mit Norm 0 kann nicht normiert werden")
    return q / n


def quat_multiply(q1, q2):
    """Hamilton-Produkt zweier Quaternionen [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def quat_from_rpy(roll, pitch, yaw):
    """RPY (ZYX-Konvention) -> Quaternion [w, x, y, z]."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def quat_to_rpy(q):
    """Quaternion [w, x, y, z] -> (roll, pitch, yaw) in ZYX-Konvention."""
    w, x, y, z = quat_normalize(q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quat_from_axis_angle(axis, angle_rad):
    """Drehachse (3er-Vektor) + Winkel -> Quaternion."""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    s = math.sin(angle_rad * 0.5)
    return np.array([math.cos(angle_rad * 0.5), axis[0] * s, axis[1] * s, axis[2] * s])


def quat_to_matrix(q):
    """Quaternion [w, x, y, z] -> 3x3-Rotationsmatrix."""
    w, x, y, z = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quat(R):
    """3x3-Rotationsmatrix -> Quaternion [w, x, y, z] (Shepperd-Methode)."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array(
            [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s]
        )
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = np.array(
            [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s,
             (R[0, 2] + R[2, 0]) / s]
        )
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = np.array(
            [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s,
             (R[1, 2] + R[2, 1]) / s]
        )
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = np.array(
            [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
             (R[1, 2] + R[2, 1]) / s, 0.25 * s]
        )
    return quat_normalize(q)


def quat_rotate(q, v):
    """Dreht den Vektor ``v`` mit dem Quaternion ``q``."""
    return quat_to_matrix(q) @ np.asarray(v, dtype=float)


def quat_angle_between(q1, q2):
    """Kleinster Drehwinkel zwischen zwei Orientierungen in rad.

    Beruecksichtigt die Doppeldeutigkeit q und -q (gleiche Orientierung).
    """
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = abs(float(np.dot(q1, q2)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def quat_canonical(q, reference):
    """Waehlt von q und -q (gleiche Orientierung) das zur Referenz naehere.

    Noetig, wo Quaternionen als ZAHLEN weiterverwendet werden -- im
    State-Vektor lernt die Policy sonst aus q und -q zwei scheinbar
    verschiedene Zustaende (AP 1.5.1).

    Bewusst NICHT "w >= 0": bei Greifer-nach-unten-Posen ist w ~ 0 (Home:
    exakt 0, PICK: -0.025, gemessen an den VM-Punkten 2026-09-14), dort
    wuerde schon das Rotationsrauschen das Vorzeichen staendig kippen. Die
    Referenz liegt dagegen mitten im Arbeitsbereich; ein Umschlag passiert
    erst bei ~180 Grad Abweichung von ihr.
    """
    q = np.asarray(q, dtype=float)
    return -q if float(np.dot(q, np.asarray(reference, dtype=float))) < 0.0 else q


def slerp(q1, q2, t):
    """Sphaerische Interpolation zwischen zwei Quaternionen.

    Waehlt automatisch den kuerzeren Weg (Vorzeichenflip bei negativem
    Skalarprodukt) -- genau das, was lineare RPY-Interpolation am
    +/-pi-Umschlagpunkt falsch macht.
    """
    q1 = quat_normalize(q1)
    q2 = quat_normalize(q2)
    dot = float(np.dot(q1, q2))
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    if dot > 0.9995:
        # Nahezu identisch -> lineare Interpolation ist numerisch stabiler
        return quat_normalize(q1 + t * (q2 - q1))
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    a = math.sin((1.0 - t) * theta) / sin_theta
    b = math.sin(t * theta) / sin_theta
    return quat_normalize(a * q1 + b * q2)


def pose_rpy_to_quat(pose):
    """[X,Y,Z,R,P,Y] -> [X,Y,Z,QW,QX,QY,QZ]."""
    pose = list(pose)
    if len(pose) != 6:
        raise ValueError("RPY-Pose braucht 6 Werte, bekam %d" % len(pose))
    q = quat_from_rpy(pose[3], pose[4], pose[5])
    return np.array([pose[0], pose[1], pose[2], q[0], q[1], q[2], q[3]])


def pose_quat_to_rpy(pose):
    """[X,Y,Z,QW,QX,QY,QZ] -> [X,Y,Z,R,P,Y]."""
    pose = list(pose)
    if len(pose) != 7:
        raise ValueError("Quaternion-Pose braucht 7 Werte, bekam %d" % len(pose))
    roll, pitch, yaw = quat_to_rpy(pose[3:7])
    return np.array([pose[0], pose[1], pose[2], roll, pitch, yaw])


def pose_interpolate(pose_a, pose_b, t):
    """Interpoliert zwei Quaternion-Posen: linear in XYZ, SLERP in Rotation."""
    pose_a = np.asarray(pose_a, dtype=float)
    pose_b = np.asarray(pose_b, dtype=float)
    pos = pose_a[:3] + t * (pose_b[:3] - pose_a[:3])
    rot = slerp(pose_a[3:7], pose_b[3:7], t)
    return np.concatenate([pos, rot])


def pose_path(waypoints_quat, steps_per_segment):
    """Erzeugt eine dichte Posenbahn durch die Wegpunkte.

    Bewusst simpel (segmentweise linear + SLERP) -- die weiche Spline-Bahn
    aus AP 2.4 gehoert in den Trajektorienplaner (Phase 3). Hier geht es
    nur darum, eine pruefbare Bahn fuer Kinematik und Kollision zu haben.

    Rueckgabe: Array (N, 7).
    """
    waypoints_quat = [np.asarray(w, dtype=float) for w in waypoints_quat]
    if len(waypoints_quat) < 2:
        raise ValueError("Mindestens zwei Wegpunkte noetig")

    out = [waypoints_quat[0]]
    for a, b in zip(waypoints_quat[:-1], waypoints_quat[1:]):
        for i in range(1, steps_per_segment + 1):
            out.append(pose_interpolate(a, b, i / steps_per_segment))
    return np.array(out)

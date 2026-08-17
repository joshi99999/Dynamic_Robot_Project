"""Trajektorienplaner: Wegpunkte -> zeitparametrierte Soll-Bahn (AP 2.2/2.4).

Erzeugt aus den geteachten Wegpunkten eine dichte, mit konstanter
Geschwindigkeit abgetastete Posenbahn bei CONTROL_RATE_HZ. Die
Geschwindigkeit steckt damit implizit in der Schrittweite zwischen zwei
aufeinanderfolgenden Posen -- exakt so lernt die Policy sie spaeter mit
(AP 2.6: die Zielgeschwindigkeit ist VOR der Datenaufzeichnung festzulegen).

Greiferbefehle werden als Dwell abgebildet: an Wegpunkten mit
Greiferwechsel haelt die Bahn GRIPPER_DWELL_STEPS Schritte die Position,
waehrend der Greifer schliesst/oeffnet (AP 2.1, Totzeit ohne
Ist-Rueckmeldung). Diese Schritte sind im ``dwell_mask`` markiert -- dort
wird auch kein Rauschen aufgepraegt.

Interpolation: linear in der Position, SLERP in der Rotation
(geometry.py -- nie RPY-Interpolation, +/-pi-Umschlagpunkt).
"""

from dataclasses import dataclass, field

import numpy as np

from . import config, geometry


@dataclass
class Waypoint:
    """Ein geteachter Wegpunkt.

    ``gripper_closed`` ist der Soll-Zustand des Greifers AB diesem Punkt;
    weicht er vom Vorgaenger ab, fuegt der Planer dort den Dwell ein.
    ``approach=True`` senkt die Geschwindigkeit des ANFAHRT-Segments auf
    APPROACH_SPEED_MS (Endanflug ans Objekt bzw. Absetzen).
    """

    pose_quat: np.ndarray
    gripper_closed: bool = False
    approach: bool = False
    name: str = ""

    def __post_init__(self):
        self.pose_quat = np.asarray(self.pose_quat, dtype=float)
        if self.pose_quat.shape != (7,):
            raise ValueError("Waypoint erwartet eine Quaternion-Pose (7 Werte)")


@dataclass
class IdealTrajectory:
    """Die ungestoerte Soll-Bahn T_soll -- Quelle der Action-Labels (AP 2.4)."""

    poses_quat: np.ndarray  # (N, 7)
    gripper: np.ndarray  # (N,) float, GRIPPER_OPEN/GRIPPER_CLOSED
    dwell_mask: np.ndarray  # (N,) bool -- Halte-Schritte fuer den Greifer
    dist_to_grasp: np.ndarray  # (N,) m -- Distanz zum naechsten Greiferwechsel
    rate_hz: float = config.CONTROL_RATE_HZ

    def __len__(self):
        return len(self.poses_quat)


def build_ideal_trajectory(
    waypoints,
    rate_hz=config.CONTROL_RATE_HZ,
    transit_speed=config.TRANSIT_SPEED_MS,
    approach_speed=config.APPROACH_SPEED_MS,
    dwell_steps=config.GRIPPER_DWELL_STEPS,
):
    """Wegpunkte -> dichte Soll-Bahn mit Greifer-Dwell.

    Jedes Segment wird mit konstanter kartesischer Geschwindigkeit
    abgetastet: Schrittzahl = ceil(Distanz / (v * dt)). Damit ist das Tempo
    ueber alle Episoden und Datenquellen konsistent (AP 2.6).
    """
    if len(waypoints) < 2:
        raise ValueError("Mindestens zwei Wegpunkte noetig")
    dt = 1.0 / rate_hz

    poses = [waypoints[0].pose_quat]
    gripper = [_g(waypoints[0].gripper_closed)]
    dwell = [False]

    for prev, wp in zip(waypoints[:-1], waypoints[1:]):
        speed = approach_speed if wp.approach else transit_speed
        dist = float(np.linalg.norm(wp.pose_quat[:3] - prev.pose_quat[:3]))
        steps = max(1, int(np.ceil(dist / (speed * dt))))
        for i in range(1, steps + 1):
            poses.append(
                geometry.pose_interpolate(prev.pose_quat, wp.pose_quat, i / steps)
            )
            # Waehrend der Fahrt gilt noch der Greiferzustand des Vorgaengers
            gripper.append(_g(prev.gripper_closed))
            dwell.append(False)

        if wp.gripper_closed != prev.gripper_closed:
            # Greiferwechsel: Position halten, Befehl wirkt ab jetzt,
            # die Backen brauchen die Totzeit (AP 2.1).
            for _ in range(dwell_steps):
                poses.append(wp.pose_quat.copy())
                gripper.append(_g(wp.gripper_closed))
                dwell.append(True)
        else:
            gripper[-1] = _g(wp.gripper_closed)

    poses = np.asarray(poses)
    gripper = np.asarray(gripper, dtype=float)
    dwell = np.asarray(dwell, dtype=bool)

    return IdealTrajectory(
        poses_quat=poses,
        gripper=gripper,
        dwell_mask=dwell,
        dist_to_grasp=_dist_to_next_gripper_change(poses, gripper),
        rate_hz=rate_hz,
    )


def _g(closed):
    return config.GRIPPER_CLOSED if closed else config.GRIPPER_OPEN


def _dist_to_next_gripper_change(poses, gripper):
    """Distanz jedes Schritts zur Position des naechsten Greiferwechsels.

    Grundlage der Trichter-Daempfung (AP 2.4): je naeher am Greifpunkt,
    desto staerker wird das Rauschen gegen 0 gedaempft. Nach dem letzten
    Greiferwechsel ist die Distanz unendlich (volles Transit-Rauschen).
    """
    n = len(poses)
    dist = np.full(n, np.inf)
    next_change_pos = None
    for i in range(n - 1, -1, -1):
        if i < n - 1 and gripper[i] != gripper[i + 1]:
            next_change_pos = poses[i + 1, :3]
        if next_change_pos is not None:
            dist[i] = float(np.linalg.norm(poses[i, :3] - next_change_pos))
    return dist

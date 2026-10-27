"""Probefahrt nach dem Teachen (AP 2.2/4.2, Wunsch Anwender 2026-10-23).

Vor der ersten Episode faehrt der Arm die IDEALE Bahn langsam ab -- ohne
Rauschen, mit offenem Greifer, ohne Aufzeichnung -- und danach zurueck an
den Start. So faellt ein falsch geteachter Punkt auf, bevor mit Tempo und
geschlossenem Greifer gefahren wird (Anlage 2026-10-23: PICK und PRE_GRASP
vertauscht, leichter Crash in der ersten Episode).

Langsamer wird die Bahn ZEITLICH gestreckt, nicht ueber den Override:
Dieselben Stuetzpunkte im Gelenkraum, dazwischen linear -- genau das, was
servo_j bei der Aufnahme zwischen den 15-Hz-Schritten auch faehrt. Die
Geometrie ist also die der Aufnahme, nur die Rampen sind sanfter
(Beschleunigung mit dem Quadrat des Faktors). Der Override bliebe dagegen
nicht ohne Folgen: Er bremst den servo_j-Strom zeitlich, bei 0.2 laeuft der
Arm in der VM bis 2,3 s hinterher (CLAUDE.md, Offen 1).

Zurueck geht es auf der Bahn des Zuruecklegens (bc/put_back.py), sonst auf
der Vorwaertsbahn rueckwaerts -- in beiden Faellen Geometrie, die schon
geplant und kollisionsgeprueft ist.
"""

import numpy as np

from . import config
from .noise import TrajectoryPlan


def slowed_plan(plan, factor=config.PROBE_SPEED_FACTOR):
    """Ideale Bahn von ``plan``, zeitlich gestreckt, Greifer durchgehend offen.

    ``factor`` < 1 = langsamer (0.4 -> 2,5-fache Dauer). Soll = Ist: Die
    Probefahrt hat kein Rauschen.
    """
    if not 0.0 < factor <= 1.0:
        raise ValueError("Tempofaktor der Probefahrt muss in (0, 1] liegen, nicht %r" % factor)
    n = len(plan)
    m = max(n, int(np.ceil((n - 1) / float(factor))) + 1)
    s = np.linspace(0.0, n - 1, m)  # Position in alten Schritten
    k = np.arange(n)
    joints = np.column_stack(
        [np.interp(s, k, plan.joints_ideal[:, j]) for j in range(plan.joints_ideal.shape[1])])
    poses = np.column_stack(
        [np.interp(s, k, plan.poses_ideal[:, j]) for j in range(7)])
    poses[:, 3:] /= np.linalg.norm(poses[:, 3:], axis=1, keepdims=True)
    nearest = np.rint(s).astype(int)
    return TrajectoryPlan(
        joints_ideal=joints,
        joints_noisy=joints.copy(),
        poses_ideal=poses,
        poses_noisy=poses.copy(),
        gripper=np.full(m, config.GRIPPER_OPEN),
        dwell_mask=np.asarray(plan.dwell_mask)[nearest],
        rate_hz=plan.rate_hz,
    )


def reversed_plan(plan):
    """Dieselbe Bahn rueckwaerts (Rueckweg der Probefahrt ohne Zuruecklegen)."""
    return TrajectoryPlan(
        joints_ideal=plan.joints_ideal[::-1].copy(),
        joints_noisy=plan.joints_noisy[::-1].copy(),
        poses_ideal=plan.poses_ideal[::-1].copy(),
        poses_noisy=plan.poses_noisy[::-1].copy(),
        gripper=np.asarray(plan.gripper)[::-1].copy(),
        dwell_mask=np.asarray(plan.dwell_mask)[::-1].copy(),
        rate_hz=plan.rate_hz,
    )


def probe_plans(forward, back=None, factor=config.PROBE_SPEED_FACTOR):
    """(hin, zurueck) der Probefahrt.

    ``forward``: ein Plan der Aufnahme (benutzt wird nur seine IDEALE Bahn);
    ``back``: der Plan des Zuruecklegens, oder None -> Vorwaertsbahn
    rueckwaerts.
    """
    hin = slowed_plan(forward, factor)
    if back is None:
        return hin, reversed_plan(hin)
    return hin, slowed_plan(back, factor)

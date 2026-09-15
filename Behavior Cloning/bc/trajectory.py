"""Trajektorienplaner: Wegpunkte -> zeitparametrierte Soll-Bahn (AP 2.2/2.4).

Erzeugt aus den geteachten Wegpunkten eine dichte, bei CONTROL_RATE_HZ
abgetastete Bahn. Die Geschwindigkeit steckt damit implizit in der
Schrittweite zwischen zwei aufeinanderfolgenden Posen -- exakt so lernt die
Policy sie spaeter mit (AP 2.6: die Zielgeschwindigkeit ist VOR der
Datenaufzeichnung festzulegen).

Geschwindigkeitsprofil je Segment (:func:`segment_progress`): Anfahren aus
dem Stand ueber SEGMENT_RAMP_S, konstante Fahrt, Bremsen bis zum Stillstand
am Wegpunkt. Frueher sprang die Bahn an Start, Greifer-Dwell und Wechsel
LIN <-> PTP direkt auf volle Geschwindigkeit; der servo_j-Regler schwang
danach bis 0.1 rad ueber (VM 2026-09-14). Die Rampen sind fest, also in
jeder Episode gleich -- das gelernte Tempo bleibt konsistent.

Bewegungsarten je Segment (``Waypoint.motion``, gilt fuer das Segment, das
ZU diesem Punkt fuehrt -- wie am Teach-Pendant):

* ``"lin"``: Gerade im Raum -- linear in der Position, SLERP in der
  Rotation (geometry.py, nie RPY-Interpolation am +/-pi-Umschlagpunkt).
* ``"ptp"``: linear im GELENKRAUM zwischen den geteachten Stellungen; die
  Posen entstehen per FK. Schrittzahl nach dem strengeren Limit aus
  PTP_JOINT_SPEED_RADS und TCP-Bahngeschwindigkeit.

Greiferbefehle werden als Dwell abgebildet: an Wegpunkten mit
Greiferwechsel haelt die Bahn GRIPPER_DWELL_STEPS Schritte die Position,
waehrend der Greifer schliesst/oeffnet (AP 2.1, Totzeit ohne
Ist-Rueckmeldung). Diese Schritte sind im ``dwell_mask`` markiert.

Ankerpunkte fuer die Rausch-Daempfung (noise.py) sind Episodenstart, jeder
Greiferwechsel und Episodenende: dort ist das Rauschen 0, dazwischen steigt
und faellt es ueber die Bahnlaenge weich an. Frueher zaehlte nur der Abstand
zum NAECHSTEN Greiferwechsel -- direkt nach dem Greifen sprang das Rauschen
dadurch in einem Schritt von 0 auf volle Amplitude, waehrend die Backen
noch am Objekt waren.
"""

from dataclasses import dataclass

import numpy as np

from . import config, geometry

MOTION_LIN = "lin"
MOTION_PTP = "ptp"
MOTIONS = (MOTION_LIN, MOTION_PTP)

#: Stuetzstellen je PTP-Segment, um die TCP-Bahnlaenge abzuschaetzen (die
#: Gerade zwischen Start- und Zielpose unterschaetzt sie).
_PTP_LENGTH_PROBES = 20


@dataclass
class Waypoint:
    """Ein geteachter Wegpunkt.

    ``gripper_closed`` ist der Soll-Zustand des Greifers AB diesem Punkt;
    weicht er vom Vorgaenger ab, fuegt der Planer dort den Dwell ein.
    ``approach=True`` senkt die Geschwindigkeit des ANFAHRT-Segments auf
    APPROACH_SPEED_MS (Endanflug ans Objekt bzw. Absetzen).
    ``motion`` ist die Bewegungsart des Segments ZU diesem Punkt.
    ``joints`` ist die geteachte Gelenkstellung -- Pflicht an beiden Enden
    eines PTP-Segments.
    """

    pose_quat: np.ndarray
    gripper_closed: bool = False
    approach: bool = False
    name: str = ""
    motion: str = MOTION_LIN
    joints: np.ndarray = None

    def __post_init__(self):
        self.pose_quat = np.asarray(self.pose_quat, dtype=float)
        if self.pose_quat.shape != (7,):
            raise ValueError("Waypoint erwartet eine Quaternion-Pose (7 Werte)")
        if self.motion not in MOTIONS:
            raise ValueError(
                "Waypoint '%s': unbekannte Bewegungsart %r (erlaubt: %s)"
                % (self.name, self.motion, ", ".join(MOTIONS))
            )
        if self.joints is not None:
            self.joints = np.asarray(self.joints, dtype=float)


@dataclass
class IdealTrajectory:
    """Die ungestoerte Soll-Bahn T_soll -- Quelle der Action-Labels (AP 2.4)."""

    poses_quat: np.ndarray  # (N, 7)
    gripper: np.ndarray  # (N,) float, GRIPPER_OPEN/GRIPPER_CLOSED
    dwell_mask: np.ndarray  # (N,) bool -- Halte-Schritte fuer den Greifer
    dist_to_anchor: np.ndarray  # (N,) m Bahnlaenge zum naechstgelegenen Anker
    #: (N, dof) bekannte Soll-Gelenkstellungen (PTP-Segmente), sonst NaN;
    #: None, wenn kein Wegpunkt eine Gelenkstellung traegt.
    joints: np.ndarray = None
    rate_hz: float = config.CONTROL_RATE_HZ

    def __len__(self):
        return len(self.poses_quat)


def build_ideal_trajectory(
    waypoints,
    rate_hz=config.CONTROL_RATE_HZ,
    transit_speed=config.TRANSIT_SPEED_MS,
    approach_speed=config.APPROACH_SPEED_MS,
    dwell_steps=config.GRIPPER_DWELL_STEPS,
    fk=None,
    ptp_joint_speed=config.PTP_JOINT_SPEED_RADS,
):
    """Wegpunkte -> dichte Soll-Bahn mit Greifer-Dwell.

    LIN-Segmente fahren mit kartesischer Hoechstgeschwindigkeit v, Rampen
    an beiden Enden (:func:`segment_progress`). PTP-Segmente
    brauchen ``fk`` (Gelenkwinkel -> Quaternion-Pose, z. B.
    ``robot.fk``). Damit ist das Tempo ueber alle Episoden und
    Datenquellen konsistent (AP 2.6).
    """
    if len(waypoints) < 2:
        raise ValueError("Mindestens zwei Wegpunkte noetig")
    dt = 1.0 / rate_hz

    dof = next((len(wp.joints) for wp in waypoints if wp.joints is not None), None)
    nan_row = None if dof is None else np.full(dof, np.nan)

    first = waypoints[0]
    poses = [first.pose_quat]
    joints = [_joint_row(first.joints, nan_row)]
    gripper = [_g(first.gripper_closed)]
    dwell = [False]

    for prev, wp in zip(waypoints[:-1], waypoints[1:]):
        speed = approach_speed if wp.approach else transit_speed
        if wp.motion == MOTION_PTP:
            seg_poses, seg_joints = _ptp_segment(
                prev, wp, fk, speed, ptp_joint_speed, dt
            )
        else:
            seg_poses = _lin_segment(prev, wp, speed, dt)
            seg_joints = [nan_row] * len(seg_poses)

        for pose, row in zip(seg_poses, seg_joints):
            poses.append(pose)
            joints.append(_joint_row(row, nan_row))
            # Waehrend der Fahrt gilt noch der Greiferzustand des Vorgaengers
            gripper.append(_g(prev.gripper_closed))
            dwell.append(False)

        if wp.gripper_closed != prev.gripper_closed:
            # Greiferwechsel: Position halten, Befehl wirkt ab jetzt,
            # die Backen brauchen die Totzeit (AP 2.1).
            for _ in range(dwell_steps):
                poses.append(poses[-1].copy())
                joints.append(_joint_row(joints[-1], nan_row))
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
        dist_to_anchor=_dist_to_anchor(poses, gripper),
        joints=None if dof is None else np.asarray(joints, dtype=float),
        rate_hz=rate_hz,
    )


def joint_derivatives(joints, rate_hz=config.CONTROL_RATE_HZ):
    """Geschwindigkeit und Beschleunigung einer abgetasteten Gelenkbahn.

    Zentrale Differenzen; vor dem ersten und nach dem letzten Schritt wird
    Stillstand angenommen (Randwerte wiederholt). Dwell-Phasen ergeben
    automatisch 0. Rueckgabe ``(velocity, acceleration)`` je (N, dof) in
    rad/s bzw. rad/s^2 -- das, was ``servo_j`` neben der Position braucht.
    """
    q = np.asarray(joints, dtype=float)
    if q.ndim != 2 or len(q) == 0:
        raise ValueError("joint_derivatives erwartet ein Array (N, dof)")
    dt = 1.0 / rate_hz
    padded = np.pad(q, ((1, 1), (0, 0)), mode="edge")
    velocity = (padded[2:] - padded[:-2]) / (2.0 * dt)
    acceleration = (padded[2:] - 2.0 * padded[1:-1] + padded[:-2]) / (dt * dt)
    return velocity, acceleration


def segment_progress(nominal_duration, dt, ramp=config.SEGMENT_RAMP_S):
    """Bahnparameter s in (0, 1] je Takt fuer EIN Segment, mit Rampen.

    ``nominal_duration``: Dauer bei konstanter Hoechstgeschwindigkeit
    (Laenge / Geschwindigkeit). Das Segment startet und endet im
    Stillstand; Anfahren und Bremsen dauern je ``ramp`` Sekunden mit
    Sinus-Geschwindigkeitsprofil v(t) = v_max * (1 - cos(pi t / ramp)) / 2 --
    die Beschleunigung ist stetig und an den Segmentgrenzen 0. Die
    Gesamtdauer waechst dadurch um genau ``ramp``.

    Ist das Segment zu kurz fuer v_max (nominal_duration < ramp), wird die
    Rampe auf sqrt(nominal_duration * ramp) verkuerzt: gleiche
    Spitzenbeschleunigung, niedrigere Spitzengeschwindigkeit.

    Rueckgabe: Array der Laenge ceil(Dauer / dt), letzter Wert exakt 1.0.
    Die Takte werden gleichmaessig ueber die Dauer verteilt (hoechstens
    ein Bruchteil eines Takts langsamer als geplant, wie zuvor bei der
    konstanten Geschwindigkeit).
    """
    t0 = float(nominal_duration)
    if t0 <= 0.0:
        return np.ones(1)
    if ramp <= 0.0:
        r, v_peak, cruise = 0.0, 1.0 / t0, t0
    elif t0 >= ramp:
        r, v_peak, cruise = ramp, 1.0 / t0, t0 - ramp
    else:
        r = float(np.sqrt(t0 * ramp))
        v_peak, cruise = 1.0 / r, 0.0
    total = 2.0 * r + cruise
    steps = max(1, int(np.ceil(total / dt - 1e-9)))
    t = np.arange(1, steps + 1) * (total / steps)

    def ramp_up(tau):
        if r == 0.0:
            return np.zeros_like(tau)
        return v_peak * (tau / 2.0 - r / (2.0 * np.pi) * np.sin(np.pi * tau / r))

    s = np.where(
        t <= r,
        ramp_up(np.minimum(t, r)),
        np.where(
            t <= r + cruise,
            v_peak * (r / 2.0 + (t - r)),
            1.0 - ramp_up(np.clip(total - t, 0.0, r)),
        ),
    )
    s[-1] = 1.0
    return np.clip(s, 0.0, 1.0)


def _lin_segment(prev, wp, speed, dt):
    dist = float(np.linalg.norm(wp.pose_quat[:3] - prev.pose_quat[:3]))
    return [
        geometry.pose_interpolate(prev.pose_quat, wp.pose_quat, float(s))
        for s in segment_progress(dist / speed, dt)
    ]


def _ptp_segment(prev, wp, fk, speed, joint_speed, dt):
    if prev.joints is None or wp.joints is None:
        raise ValueError(
            "PTP-Segment '%s' -> '%s' braucht die Gelenkstellung beider Punkte"
            % (prev.name, wp.name)
        )
    if fk is None:
        raise ValueError(
            "PTP-Segment '%s' -> '%s': build_ideal_trajectory braucht fk"
            % (prev.name, wp.name)
        )
    delta = wp.joints - prev.joints
    probe_pos = np.array(
        [
            np.asarray(fk(prev.joints + t * delta), dtype=float)[:3]
            for t in np.linspace(0.0, 1.0, _PTP_LENGTH_PROBES + 1)
        ]
    )
    tcp_length = float(np.sum(np.linalg.norm(np.diff(probe_pos, axis=0), axis=1)))
    # Das strengere Limit bestimmt die Dauer (Gelenk- oder TCP-Tempo)
    nominal = max(float(np.max(np.abs(delta))) / joint_speed, tcp_length / speed)
    seg_joints = [prev.joints + float(s) * delta for s in segment_progress(nominal, dt)]
    seg_poses = [np.asarray(fk(q), dtype=float) for q in seg_joints]
    # Die Zielpose exakt aus dem Wegpunkt uebernehmen (FK derselben Stellung)
    seg_poses[-1] = wp.pose_quat.copy()
    return seg_poses, seg_joints


def _joint_row(row, nan_row):
    if nan_row is None:
        return None
    return nan_row.copy() if row is None else np.asarray(row, dtype=float).copy()


def _g(closed):
    return config.GRIPPER_CLOSED if closed else config.GRIPPER_OPEN


def _dist_to_anchor(poses, gripper):
    """Bahnlaenge jedes Schritts zum naechstgelegenen Ankerpunkt.

    Anker: Schritt 0, der erste Dwell-Schritt jedes Greiferwechsels und der
    letzte Schritt. Gemessen wird entlang der TCP-Bahn (kumulierte
    Positionsdifferenzen) zum vorherigen und zum naechsten Anker, das
    Minimum zaehlt -- so ist die Daempfung beidseitig jedes Ankers
    symmetrisch und unabhaengig davon, ob die Bahn raeumlich wieder nahe an
    einem frueheren Punkt vorbeifuehrt.
    """
    n = len(poses)
    steps = np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1)
    arc = np.concatenate([[0.0], np.cumsum(steps)])

    anchors = {0, n - 1}
    for i in range(n - 1):
        if gripper[i] != gripper[i + 1]:
            anchors.add(i + 1)
    anchors = np.array(sorted(anchors))

    idx = np.arange(n)
    nxt = anchors[np.searchsorted(anchors, idx, side="left")]
    prv = anchors[np.searchsorted(anchors, idx, side="right") - 1]
    return np.minimum(arc[idx] - arc[prv], arc[nxt] - arc[idx])

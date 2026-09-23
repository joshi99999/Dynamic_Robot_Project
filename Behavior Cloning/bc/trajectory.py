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

Ueberschleifen (``Waypoint.blend_m``, Ablaufdatei ``"blend"``): statt am
Wegpunkt anzuhalten, beginnt das Folgesegment schon waehrend der Bremsrampe.
Die Bahn ist in diesem Fenster die Ueberlagerung beider Segmente -- Brems-
und Anfahrrampe addieren sich, die Geschwindigkeit bleibt stetig und die
Ecke wird verrundet. Der Radius ist die Bahnlaenge vor und nach dem Punkt,
innerhalb der ueberschliffen wird; ausserhalb davon faehrt die Bahn exakt
wie ohne Ueberschleifen. Groessere Radien als die normale Rampenstrecke
(v * SEGMENT_RAMP_S / 2, im Transit 3.75 cm) verlaengern die beteiligten
Rampen entsprechend -- die Ecke wird weiter, die Beschleunigung kleiner.
Zwei PTP-Segmente werden im Gelenkraum ueberlagert (Posen per FK), jede
Kombination mit LIN kartesisch (Position additiv, Rotation als
Produkt der Teildrehungen; die Gelenkwinkel loest dort die IK).
Nie ueberschliffen wird am Start, am Ende und an Greiferwechseln -- dort
muss der Punkt exakt erreicht werden.

Bewegungsarten je Segment (``Waypoint.motion``, gilt fuer das Segment, das
ZU diesem Punkt fuehrt -- wie am Teach-Pendant):

* ``"lin"``: Gerade im Raum -- linear in der Position, SLERP in der
  Rotation (geometry.py, nie RPY-Interpolation am +/-pi-Umschlagpunkt).
* ``"ptp"``: linear im GELENKRAUM zwischen den geteachten Stellungen; die
  Posen entstehen per FK. Dauer nach dem strengeren Limit aus
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
    ``blend_m``: Ueberschleif-Radius an DIESEM Punkt in m (0 = anhalten).
    """

    pose_quat: np.ndarray
    gripper_closed: bool = False
    approach: bool = False
    name: str = ""
    motion: str = MOTION_LIN
    joints: np.ndarray = None
    blend_m: float = 0.0

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
        self.blend_m = float(self.blend_m or 0.0)
        if self.blend_m < 0.0:
            raise ValueError("Waypoint '%s': blend_m darf nicht negativ sein" % self.name)


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
    #: Tatsaechlich verwendeter Ueberschleif-Radius je Wegpunkt (m) -- nach
    #: Begrenzung auf die halbe Segmentlaenge und 0 an Start/Ende/Greifer.
    blend_applied: tuple = ()

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
    ramp_s=config.SEGMENT_RAMP_S,
):
    """Wegpunkte -> dichte Soll-Bahn mit Greifer-Dwell und Ueberschleifen.

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

    segments = [
        _Segment(prev, wp, approach_speed if wp.approach else transit_speed,
                 fk, ptp_joint_speed)
        for prev, wp in zip(waypoints[:-1], waypoints[1:])
    ]
    n_wp = len(waypoints)
    change = [False] + [
        waypoints[k].gripper_closed != waypoints[k - 1].gripper_closed
        for k in range(1, n_wp)
    ]

    # -- Ueberschleif-Radien (Wegpunkt k liegt zwischen Segment k-1 und k) --
    radii = [0.0] * n_wp
    for k in range(1, n_wp - 1):
        if change[k] or waypoints[k].blend_m <= 0.0:
            continue
        seg_in, seg_out = segments[k - 1], segments[k]
        radii[k] = min(waypoints[k].blend_m, 0.5 * seg_in.length, 0.5 * seg_out.length)

    # -- Profile: Rampen, die an ueberschliffenen Punkten verlaengert werden --
    for k, seg in enumerate(segments):
        seg.profile = _Profile(
            seg.nominal,
            _blend_ramp(ramp_s, radii[k], seg.tcp_speed),
            _blend_ramp(ramp_s, radii[k + 1], seg.tcp_speed),
        )

    # -- Zeitplan (kontinuierlich). Haltepunkte liegen exakt auf dem Raster:
    # die Dauer eines Segments wird dafuer um < 1 Takt gestreckt. Nur ein
    # ueberschliffener Uebergang darf zwischen zwei Rasterpunkten beginnen --
    # sonst waere die Ueberlappung auf ganze Takte gerundet und die
    # Geschwindigkeit saeckte an der Ecke ab.
    starts, ends = [], []
    t = 0.0
    for k, seg in enumerate(segments):
        end = max(1, int(np.ceil((t + seg.profile.total) / dt - 1e-9))) * dt
        seg.profile.stretch(end - t)
        starts.append(t)
        ends.append(end)
        wp_index = k + 1
        t = end
        if change[wp_index]:
            t += dwell_steps * dt
        elif wp_index < n_wp - 1 and radii[wp_index] > 0.0:
            t -= _overlap_time(seg, segments[k + 1], radii[wp_index])
    arrival_steps = [0] + [int(round(e / dt)) for e in ends]
    total = arrival_steps[-1] + (dwell_steps if change[-1] else 0) + 1

    poses = np.empty((total, 7))
    joints = None if dof is None else np.empty((total, dof))
    gripper = np.empty(total)
    dwell = np.zeros(total, dtype=bool)
    eps = 1e-9

    for i in range(total):
        ti = i * dt
        active = [
            k for k in range(len(segments)) if starts[k] + eps < ti < ends[k] - eps
        ]
        if len(active) > 2:
            raise AssertionError("Mehr als zwei Segmente gleichzeitig aktiv (Schritt %d)" % i)
        if not active:
            b = max(k for k in range(n_wp) if arrival_steps[k] <= i)
            poses[i] = waypoints[b].pose_quat
            row = waypoints[b].joints if (b == 0 or segments[b - 1].kind == MOTION_PTP) else None
        elif len(active) == 1:
            k = active[0]
            s = segments[k].profile.at(ti - starts[k])
            poses[i] = segments[k].pose(s)
            row = segments[k].joints_at(s)
        else:
            a, b = active
            sa = segments[a].profile.at(ti - starts[a])
            sb = segments[b].profile.at(ti - starts[b])
            poses[i], row = _superpose(segments[a], sa, segments[b], sb)
        if joints is not None:
            joints[i] = nan_row if row is None else row

        m = max([k for k in range(n_wp) if arrival_steps[k] < i] or [0])
        gripper[i] = _g(waypoints[m].gripper_closed)
        dwell[i] = change[m] and m > 0 and i <= arrival_steps[m] + dwell_steps

    return IdealTrajectory(
        poses_quat=poses,
        gripper=gripper,
        dwell_mask=dwell,
        dist_to_anchor=_dist_to_anchor(poses, gripper),
        joints=joints,
        rate_hz=rate_hz,
        blend_applied=tuple(radii),
    )


def joint_derivatives(joints, rate_hz=config.CONTROL_RATE_HZ):
    """Geschwindigkeit und Beschleunigung einer abgetasteten Gelenkbahn.

    Zentrale Differenzen; vor dem ersten und nach dem letzten Schritt wird
    Stillstand angenommen (Randwerte wiederholt). Dwell-Phasen ergeben
    automatisch 0. Rueckgabe ``(velocity, acceleration)`` je (N, dof) in
    rad/s bzw. rad/s^2.
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
    ein Bruchteil eines Takts langsamer als geplant).
    """
    profile = _Profile(nominal_duration, ramp, ramp)
    if profile.total <= 0.0:
        return np.ones(1)
    steps = max(1, int(np.ceil(profile.total / dt - 1e-9)))
    profile.stretch(steps * dt)
    s = np.array([profile.at(j * dt) for j in range(1, steps + 1)])
    s[-1] = 1.0
    return s


# ---------------------------------------------------------------------------
# Intern
# ---------------------------------------------------------------------------


class _Segment(object):
    """Ein Segment mit Bahnfunktion s -> Pose (und Gelenke bei PTP)."""

    def __init__(self, prev, wp, speed, fk, joint_speed):
        self.prev, self.wp, self.kind = prev, wp, wp.motion
        self.fk = fk
        if self.kind == MOTION_PTP:
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
            self.delta = wp.joints - prev.joints
            probe_pos = np.array(
                [
                    np.asarray(fk(prev.joints + t * self.delta), dtype=float)[:3]
                    for t in np.linspace(0.0, 1.0, _PTP_LENGTH_PROBES + 1)
                ]
            )
            self.length = float(np.sum(np.linalg.norm(np.diff(probe_pos, axis=0), axis=1)))
            # Das strengere Limit bestimmt die Dauer (Gelenk- oder TCP-Tempo)
            self.nominal = max(
                float(np.max(np.abs(self.delta))) / joint_speed, self.length / speed
            )
        else:
            self.length = float(np.linalg.norm(wp.pose_quat[:3] - prev.pose_quat[:3]))
            self.nominal = self.length / speed
        self.tcp_speed = self.length / self.nominal if self.nominal > 0 else 0.0
        self.profile = None

    def pose(self, s):
        if s >= 1.0:
            return self.wp.pose_quat.copy()
        if self.kind == MOTION_PTP:
            if s <= 0.0:
                return self.prev.pose_quat.copy()
            return np.asarray(self.fk(self.prev.joints + s * self.delta), dtype=float)
        return geometry.pose_interpolate(self.prev.pose_quat, self.wp.pose_quat, float(s))

    def joints_at(self, s):
        if self.kind != MOTION_PTP:
            return None
        return self.prev.joints + float(s) * self.delta


def _superpose(seg_a, sa, seg_b, sb):
    """Ueberschleif-Fenster: Segment a bremst, Segment b faehrt schon an.

    Zwei PTP-Segmente: Gelenkwinkel addieren (bleibt PTP-Charakter), Pose
    per FK. Sonst kartesisch: Position additiv, Rotation als Produkt der
    Teildrehung von b (Weltrahmen) mit der aktuellen Orientierung von a.
    An den Fenstergrenzen (sb = 0 bzw. sa = 1) stimmt das exakt mit dem
    jeweils allein aktiven Segment ueberein.
    """
    if seg_a.kind == MOTION_PTP and seg_b.kind == MOTION_PTP:
        q = seg_a.joints_at(sa) + float(sb) * seg_b.delta
        return np.asarray(seg_a.fk(q), dtype=float), q
    pa = seg_a.pose(sa)
    pb = seg_b.pose(sb)
    pb0 = seg_b.pose(0.0)
    pose = np.empty(7)
    pose[:3] = pa[:3] + (pb[:3] - pb0[:3])
    turn = geometry.quat_multiply(pb[3:7], geometry.quat_conjugate(pb0[3:7]))
    pose[3:7] = geometry.quat_normalize(geometry.quat_multiply(turn, pa[3:7]))
    return pose, None


def _blend_ramp(ramp_s, radius, tcp_speed):
    """Rampendauer an einem Segmentende: normal, oder so lang, dass die
    Rampe genau die Ueberschleif-Strecke ``radius`` abdeckt."""
    if radius <= 0.0 or tcp_speed <= 0.0:
        return ramp_s
    return max(ramp_s, 2.0 * radius / tcp_speed)


class _Profile(object):
    """Bahnparameter s(t) eines Segments: Sinus-Rampe, Fahrt, Sinus-Rampe.

    Rampen mit v(t) = v_peak * (1 - cos(pi t / r)) / 2. Ist das Segment zu
    kurz fuer die volle Geschwindigkeit, werden Rampen und Spitze gleich
    stark gestaucht -- die Spitzenbeschleunigung (v_peak / r) bleibt.
    """

    def __init__(self, nominal, ramp_in, ramp_out):
        t0 = float(nominal)
        r_in, r_out = float(ramp_in), float(ramp_out)
        if t0 <= 0.0:
            self.r_in = self.r_out = self.cruise = self.total = 0.0
            self.peak = 0.0
            return
        if t0 >= 0.5 * (r_in + r_out):
            peak, cruise = 1.0 / t0, t0 - 0.5 * (r_in + r_out)
        else:
            f = np.sqrt(2.0 * t0 / (r_in + r_out))
            r_in, r_out, peak, cruise = f * r_in, f * r_out, f / t0, 0.0
        self.r_in, self.r_out, self.peak, self.cruise = r_in, r_out, peak, cruise
        self.total = r_in + cruise + r_out

    def stretch(self, new_total):
        """Zeitlich strecken (gleiche Form, etwas langsamer)."""
        if self.total <= 0.0 or new_total <= 0.0:
            self.total = max(self.total, new_total)
            return
        f = new_total / self.total
        self.r_in *= f
        self.r_out *= f
        self.cruise *= f
        self.peak /= f
        self.total = new_total

    def _ramp_dist(self, tau, r):
        if r <= 0.0:
            return 0.0
        return self.peak * (tau / 2.0 - r / (2.0 * np.pi) * np.sin(np.pi * tau / r))

    def at(self, t):
        if self.peak <= 0.0 or t >= self.total:
            return 1.0
        if t <= 0.0:
            return 0.0
        if t <= self.r_in:
            s = self._ramp_dist(t, self.r_in)
        elif t <= self.r_in + self.cruise:
            s = self.peak * (self.r_in / 2.0 + (t - self.r_in))
        else:
            s = 1.0 - self._ramp_dist(min(self.total - t, self.r_out), self.r_out)
        return float(min(1.0, max(0.0, s)))


def _overlap_time(seg_in, seg_out, radius):
    """Um wie viele Sekunden das Folgesegment frueher startet.

    Hoechstens so lang, dass (a) nur Bremsrampe von ``seg_in`` und
    Anfahrrampe von ``seg_out`` ueberlappen und (b) beide Segmente im
    Fenster hoechstens ``radius`` Bahnlaenge zuruecklegen. Bei verlaengerten
    Rampen (Radius > normale Rampenstrecke) ist das die volle Rampe.
    """
    p_in, p_out = seg_in.profile, seg_out.profile

    def longest(limit, dist):
        lo, hi = 0.0, limit
        if dist(hi) <= radius + 1e-9:
            return hi
        for _ in range(50):
            mid = 0.5 * (lo + hi)
            if dist(mid) <= radius:
                lo = mid
            else:
                hi = mid
        return lo

    tau_in = longest(p_in.r_out, lambda tau: seg_in.length * (1.0 - p_in.at(p_in.total - tau)))
    tau_out = longest(p_out.r_in, lambda tau: seg_out.length * p_out.at(tau))
    return min(tau_in, tau_out)


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

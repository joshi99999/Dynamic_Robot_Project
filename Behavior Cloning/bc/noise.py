"""Action Noise Injection: Rauschprozess, Trichter, Rejection Sampling (AP 2.4).

Setzt die asymmetrische Rauscheinspielung um:

* **Ornstein-Uhlenbeck-Prozess** fuer weiche, realistische Abweichungen
  statt unruhigem Zittern -- zusaetzlich geglaettet, damit auch
  Geschwindigkeit und Beschleunigung der Befehlsbahn stetig bleiben
  (servo_j, Befund VM 2026-09-14, siehe :class:`SmoothedOUProcess`).
* **Trichter-Daempfung**: das Rauschen ist an jedem Ankerpunkt 0 --
  Episodenstart, jeder Greiferwechsel, Episodenende -- und steigt bzw.
  faellt dazwischen ueber die Bahnlaenge weich an (praeziser Griff trotz
  Schlingerns im Transit, kein Ruck direkt nach dem Greifen, Start und
  Uebergabe exakt an den geteachten Punkten).
* **Asymmetrische Begrenzung**: kartesische Box-Grenzen je Achse, z. B.
  kein/kaum Rauschen Richtung Tisch (Sicherheitsanforderung AP 2.4).
* **Rejection Sampling**: jede verrauschte Bahn wird per IK geloest
  (inkl. der vier Absicherungen aus kinematics.py) und gegen das
  Kollisionsmodell geprueft; bei Verletzung wird neu gesampelt.

Das Rauschen wird KARTESISCH aufgepraegt und per IK in Gelenkwinkel
umgerechnet -- nie direkt im Gelenkraum (Begruendung in AP 2.4: nur
kartesisch lassen sich Richtungsgrenzen wie "nicht Richtung Tisch"
garantieren). Die IK laeuft offline bei der Generierung, nicht in der
Echtzeitschleife.
"""

from dataclasses import dataclass, field

import numpy as np

from . import config, geometry
from .kinematics import IKFailure


class OUProcess(object):
    """Ornstein-Uhlenbeck-Prozess in d Dimensionen.

    Stationaere Standardabweichung = ``sigma``, Zeitkonstante ``tau``.
    Grosse tau = traege Schlingerbewegung, kleine tau = nervoeses Zittern.
    """

    def __init__(self, sigma, tau, dt, dims=3, rng=None):
        self.sigma = float(sigma)
        self.tau = float(tau)
        self.dt = float(dt)
        self.dims = dims
        self.rng = rng if rng is not None else np.random.default_rng()
        self.state = np.zeros(dims)

    def reset(self):
        self.state = np.zeros(self.dims)
        return self.state

    def step(self):
        # Exakte Diskretisierung des OU-Prozesses (stabil fuer jedes dt)
        alpha = np.exp(-self.dt / self.tau)
        noise_std = self.sigma * np.sqrt(1.0 - alpha * alpha)
        self.state = alpha * self.state + noise_std * self.rng.standard_normal(self.dims)
        return self.state.copy()


class SmoothedOUProcess(object):
    """OU-Prozess mit nachgeschalteter Glaettung -- das Rauschen fuer servo_j.

    Beim reinen OU-Prozess ist die Position stetig, die GESCHWINDIGKEIT aber
    weisses Rauschen: jeder Takt bekommt einen unabhaengigen Stoss. Gemessen
    VM 2026-09-14: 0.09 m/s Streuung je Achse (60 % der Transitgeschwindig-
    keit), Befehlsbahn bis 25 rad/s^2 an den Handgelenken, der Arm lief bis
    0.57 rad hinterher. Der SimRobot springt einfach mit, deshalb fiel das
    vorher nicht auf.

    Abhilfe: zwei gleiche Tiefpaesse 1. Ordnung (Zeitkonstante
    ``tau_smooth``) hinter dem OU-Prozess -- ein kritisch gedaempftes Glied
    2. Ordnung. Der Charakter bleibt (traege Drift mit ``tau``), aber
    Geschwindigkeit und Beschleunigung sind endlich. Bei tau_smooth=0.25 s
    und 15 Hz: Geschwindigkeit 0.022 statt 0.09 m/s, Beschleunigung
    0.11 statt 1.95 m/s^2 (je Achse, Std).

    * **Amplitude bleibt ``sigma``:** Die Glaettung senkt die Varianz; der
      Verstaerkungsfaktor dafuer wird exakt aus der stationaeren Kovarianz
      des diskreten Systems berechnet (Lyapunov-Gleichung).
    * **Grenzen ohne Knick:** ``lo``/``hi`` werden auf den OU-Zustand VOR der
      Glaettung angewendet. Die Tiefpaesse mitteln nur (Impulsantwort >= 0,
      Summe 1), der Ausgang bleibt also garantiert in den Grenzen -- ein
      hartes Klemmen des Ausgangs dagegen braechte wieder einen
      Geschwindigkeitssprung.
    * **Stationaerer Start:** Der Anfangszustand wird aus der stationaeren
      Verteilung gezogen, damit die Amplitude nicht erst ueber einige
      Sekunden einschwingt. Am Anker ist das Rauschen trotzdem 0 (Trichter).
    """

    def __init__(self, sigma, tau, tau_smooth, dt, dims=3, rng=None, lo=None, hi=None):
        self.sigma = float(sigma)
        self.dims = dims
        self.rng = rng if rng is not None else np.random.default_rng()
        self.lo = None if lo is None else np.asarray(lo, dtype=float)
        self.hi = None if hi is None else np.asarray(hi, dtype=float)

        self._alpha = np.exp(-dt / float(tau))
        self._beta = np.exp(-dt / float(tau_smooth)) if tau_smooth > 0 else 0.0
        # Zustand je Achse s = [ou, tiefpass1, tiefpass2]. Tiefpaesse in der
        # Form y_neu = beta*y + (1-beta)*u_neu; als lineares System
        # s_neu = A s + b w fuer die stationaere Kovarianz (ohne Grenzen).
        a, beta = self._alpha, self._beta
        g = 1.0 - beta
        A = np.array(
            [
                [a, 0.0, 0.0],
                [g * a, beta, 0.0],
                [g * g * a, g * beta, beta],
            ]
        )
        b = np.array([1.0, g, g * g]) * np.sqrt(1.0 - a * a)
        cov = _stationary_covariance(A, b)
        # Verstaerkung so, dass der AUSGANG die Std sigma hat
        self._gain = self.sigma / np.sqrt(cov[2, 2])
        self._noise_std = self._gain * np.sqrt(1.0 - a * a)
        self._chol = np.linalg.cholesky(cov + 1e-12 * np.eye(3)) * self._gain
        self.state = np.zeros((dims, 3))
        self.reset()

    def reset(self):
        """Neuer Startzustand aus der stationaeren Verteilung."""
        s = self.rng.standard_normal((self.dims, 3)) @ self._chol.T
        # In den Grenzen starten (Tiefpaesse sind Mittelwerte des OU-Zustands)
        self.state = np.stack([self._clip(s[:, k]) for k in range(3)], axis=1)
        return self.state[:, 2].copy()

    def step(self):
        beta = self._beta
        ou = self._alpha * self.state[:, 0]
        ou = self._clip(ou + self._noise_std * self.rng.standard_normal(self.dims))
        y1 = beta * self.state[:, 1] + (1.0 - beta) * ou
        y2 = beta * self.state[:, 2] + (1.0 - beta) * y1
        self.state = np.stack([ou, y1, y2], axis=1)
        return y2.copy()

    def _clip(self, values):
        if self.lo is None and self.hi is None:
            return values
        return np.clip(values, self.lo, self.hi)


def _stationary_covariance(A, b):
    """Loest P = A P A^T + b b^T (diskrete Lyapunov-Gleichung, n klein)."""
    n = A.shape[0]
    rhs = np.outer(b, b).reshape(-1)
    vec = np.linalg.solve(np.eye(n * n) - np.kron(A, A), rhs)
    P = vec.reshape(n, n)
    return 0.5 * (P + P.T)


def funnel_scale(
    dist_to_anchor,
    start=config.FUNNEL_START_DIST_M,
    zero=config.FUNNEL_ZERO_DIST_M,
):
    """Trichter-Daempfung: 1.0 im Transit, 0.0 am Ankerpunkt.

    Smoothstep zwischen ``zero`` und ``start`` -- stetig differenzierbar,
    damit die Daempfung selbst keine Spruenge in die Bahn traegt.
    """
    d = np.asarray(dist_to_anchor, dtype=float)
    t = np.clip((d - zero) / (start - zero), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


@dataclass
class NoiseLimits:
    """Asymmetrische kartesische Grenzen des Rauschens (Weltkoordinaten).

    ``lo``/``hi`` sind je Achse die minimal/maximal zulaessige Auslenkung.
    Default: seitlich und nach oben symmetrisch, nach unten (Richtung
    Tisch) stark reduziert -- die Kernforderung aus AP 2.4.
    """

    lo: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                -3.0 * config.NOISE_TRANS_AMPLITUDE_M,
                -3.0 * config.NOISE_TRANS_AMPLITUDE_M,
                -0.002,  # kaum Auslenkung Richtung Tisch
            ]
        )
    )
    hi: np.ndarray = field(
        default_factory=lambda: np.array(
            [
                3.0 * config.NOISE_TRANS_AMPLITUDE_M,
                3.0 * config.NOISE_TRANS_AMPLITUDE_M,
                3.0 * config.NOISE_TRANS_AMPLITUDE_M,
            ]
        )
    )

    def clip(self, offset):
        return np.clip(offset, self.lo, self.hi)


def sample_noisy_poses(
    ideal,
    rng,
    sigma_pos=config.NOISE_TRANS_AMPLITUDE_M,
    sigma_rot=config.NOISE_ROT_AMPLITUDE_RAD,
    tau=config.NOISE_OU_TAU_S,
    limits=None,
    tau_smooth=config.NOISE_SMOOTH_TAU_S,
):
    """Erzeugt EINE verrauschte Posenbahn T_ist = T_soll + N_t.

    Rauschen: geglaetteter OU-Prozess (:class:`SmoothedOUProcess`), die
    asymmetrischen Grenzen wirken innerhalb des Prozesses.
    Daempfung: Trichter (Naehe zu einem Ankerpunkt) UND Dwell-Schritte
    (Greifer schliesst -- dort wird still gehalten, Rauschen = 0).
    """
    if limits is None:
        limits = NoiseLimits()
    dt = 1.0 / ideal.rate_hz
    ou_pos = SmoothedOUProcess(
        sigma_pos, tau, tau_smooth, dt, dims=3, rng=rng, lo=limits.lo, hi=limits.hi
    )
    ou_rot = SmoothedOUProcess(sigma_rot, tau, tau_smooth, dt, dims=3, rng=rng)

    scale = funnel_scale(ideal.dist_to_anchor)
    scale = np.where(ideal.dwell_mask, 0.0, scale)

    noisy = ideal.poses_quat.copy()
    for i in range(len(noisy)):
        n_pos = ou_pos.step() * scale[i]
        n_rot = ou_rot.step() * scale[i]
        noisy[i, :3] = ideal.poses_quat[i, :3] + n_pos
        angle = float(np.linalg.norm(n_rot))
        if angle > 1e-12:
            dq = geometry.quat_from_axis_angle(n_rot / angle, angle)
            noisy[i, 3:7] = geometry.quat_normalize(
                geometry.quat_multiply(dq, ideal.poses_quat[i, 3:7])
            )
    return noisy


@dataclass
class TrajectoryPlan:
    """Fertige, geprueft ausfuehrbare Episode (Ergebnis der Generierung).

    Enthaelt bewusst BEIDE Bahnen (AP 0.9 Punkt 2): die verrauschte
    Ist-Bahn wird gefahren und ist die Observation, die ideale Soll-Bahn
    liefert die Action-Labels (t+1). Beide werden auch aufgezeichnet,
    damit die Label-Logik spaeter ueberpruef- und umlabelbar bleibt.
    """

    joints_ideal: np.ndarray  # (N, dof)
    joints_noisy: np.ndarray  # (N, dof)
    poses_ideal: np.ndarray  # (N, 7)
    poses_noisy: np.ndarray  # (N, 7)
    gripper: np.ndarray  # (N,)
    dwell_mask: np.ndarray  # (N,)
    rate_hz: float
    rejects: int = 0  # verworfene Samples bis zur gueltigen Bahn

    def __len__(self):
        return len(self.joints_ideal)


class PlanRejected(RuntimeError):
    """Auch nach NOISE_MAX_REJECTS Versuchen keine gueltige Bahn gefunden."""

    def __init__(self, message, reasons):
        super().__init__(message)
        self.reasons = reasons


def generate_plan(
    kin,
    collision_model,
    ideal,
    seed_joints,
    rng=None,
    max_rejects=config.NOISE_MAX_REJECTS,
    limits=None,
    collision_stride=1,
    noise_scale=1.0,
):
    """Vollstaendige Episodengenerierung mit Rejection Sampling (AP 2.4).

    1. Ideale Bahn einmalig per IK loesen (Warm-Start, vier Absicherungen)
       und gegen das Kollisionsmodell pruefen -- schlaegt DAS fehl, sind
       die Wegpunkte selbst unbrauchbar (kein Resampling sinnvoll).
    2. Verrauschte Bahn samplen, loesen, pruefen; bei IKFailure oder
       Kollisionsverletzung neu samplen (max. ``max_rejects`` Versuche).

    ``kin``: :class:`bc.kinematics.Kinematics`;
    ``collision_model``: :class:`bc.collision.CollisionModel`.
    ``noise_scale``: Faktor auf beide Rauschamplituden -- 0 ergibt eine
    Referenzfahrt exakt auf der idealen Bahn (Diagnose, z. B. Folgeverhalten
    des Controllers ohne Rauscheinfluss).
    """
    rng = rng if rng is not None else np.random.default_rng()

    try:
        joints_ideal = kin.solve_path(
            ideal.poses_quat, seed_joints, fixed_joints=ideal.joints
        )
    except IKFailure as exc:
        raise PlanRejected(
            "Bereits die IDEALE Bahn ist nicht loesbar (Schritt %s: %s) -- "
            "Wegpunkte pruefen." % (exc.index, exc),
            reasons=[("ideal_ik", exc)],
        ) from exc
    ideal_violations = collision_model.check_path(
        kin.robot, joints_ideal, stride=collision_stride
    )
    if ideal_violations:
        raise PlanRejected(
            "Bereits die IDEALE Bahn verletzt das Kollisionsmodell -- "
            "Wegpunkte pruefen: %s" % ideal_violations[0],
            reasons=[("ideal_collision", ideal_violations)],
        )

    reasons = []
    for attempt in range(max_rejects):
        poses_noisy = sample_noisy_poses(
            ideal,
            rng,
            sigma_pos=config.NOISE_TRANS_AMPLITUDE_M * noise_scale,
            sigma_rot=config.NOISE_ROT_AMPLITUDE_RAD * noise_scale,
            limits=limits,
        )
        try:
            joints_noisy = kin.solve_path(poses_noisy, seed_joints)
        except IKFailure as exc:
            reasons.append(("ik", exc))
            continue

        violations = collision_model.check_path(
            kin.robot, joints_noisy, stride=collision_stride
        )
        if violations:
            reasons.append(("collision", violations[0]))
            continue

        return TrajectoryPlan(
            joints_ideal=joints_ideal,
            joints_noisy=joints_noisy,
            poses_ideal=ideal.poses_quat,
            poses_noisy=poses_noisy,
            gripper=ideal.gripper,
            dwell_mask=ideal.dwell_mask,
            rate_hz=ideal.rate_hz,
            rejects=attempt,
        )

    raise PlanRejected(
        "Keine gueltige verrauschte Bahn nach %d Versuchen. Letzte Gruende: %s"
        % (max_rejects, [r[0] for r in reasons[-3:]]),
        reasons=reasons,
    )

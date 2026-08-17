"""Action Noise Injection: Rauschprozess, Trichter, Rejection Sampling (AP 2.4).

Setzt die asymmetrische Rauscheinspielung um:

* **Ornstein-Uhlenbeck-Prozess** fuer weiche, realistische Abweichungen
  statt unruhigem Zittern.
* **Trichter-Daempfung**: das Rauschen klingt Richtung Greifpunkt gegen 0
  ab (praeziser Griff trotz Schlingerns im Transit).
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


def funnel_scale(
    dist_to_grasp,
    start=config.FUNNEL_START_DIST_M,
    zero=config.FUNNEL_ZERO_DIST_M,
):
    """Trichter-Daempfung: 1.0 im Transit, 0.0 am Greifpunkt.

    Smoothstep zwischen ``zero`` und ``start`` -- stetig differenzierbar,
    damit die Daempfung selbst keine Spruenge in die Bahn traegt.
    """
    d = np.asarray(dist_to_grasp, dtype=float)
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
):
    """Erzeugt EINE verrauschte Posenbahn T_ist = T_soll + N_t.

    Daempfung: Trichter (Naehe zum Greifpunkt) UND Dwell-Schritte
    (Greifer schliesst -- dort wird still gehalten, Rauschen = 0).
    """
    if limits is None:
        limits = NoiseLimits()
    dt = 1.0 / ideal.rate_hz
    ou_pos = OUProcess(sigma_pos, tau, dt, dims=3, rng=rng)
    ou_rot = OUProcess(sigma_rot, tau, dt, dims=3, rng=rng)

    scale = funnel_scale(ideal.dist_to_grasp)
    scale = np.where(ideal.dwell_mask, 0.0, scale)

    noisy = ideal.poses_quat.copy()
    for i in range(len(noisy)):
        n_pos = limits.clip(ou_pos.step()) * scale[i]
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
):
    """Vollstaendige Episodengenerierung mit Rejection Sampling (AP 2.4).

    1. Ideale Bahn einmalig per IK loesen (Warm-Start, vier Absicherungen)
       und gegen das Kollisionsmodell pruefen -- schlaegt DAS fehl, sind
       die Wegpunkte selbst unbrauchbar (kein Resampling sinnvoll).
    2. Verrauschte Bahn samplen, loesen, pruefen; bei IKFailure oder
       Kollisionsverletzung neu samplen (max. ``max_rejects`` Versuche).

    ``kin``: :class:`bc.kinematics.Kinematics`;
    ``collision_model``: :class:`bc.collision.CollisionModel`.
    """
    rng = rng if rng is not None else np.random.default_rng()

    try:
        joints_ideal = kin.solve_path(ideal.poses_quat, seed_joints)
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
        poses_noisy = sample_noisy_poses(ideal, rng, limits=limits)
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

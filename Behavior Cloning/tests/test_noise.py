"""Rauscheinspielung: OU-Prozess, Trichter, Grenzen, Rejection (AP 2.4)."""

import _paths  # noqa: F401

import numpy as np

import _fixtures
from bc import config, geometry
from bc.adapters.sim_robot import FaultProfile
from bc.collision import Box, CollisionModel, default_workspace
from bc.kinematics import Kinematics
from bc.noise import (
    NoiseLimits,
    OUProcess,
    PlanRejected,
    SmoothedOUProcess,
    funnel_scale,
    generate_plan,
    sample_noisy_poses,
)
from bc.trajectory import Waypoint, build_ideal_trajectory


def test_ou_process_statistics():
    ou = OUProcess(sigma=0.01, tau=0.8, dt=1 / 15.0, rng=np.random.default_rng(7))
    samples = np.array([ou.step() for _ in range(8000)])
    # Stationaere Standardabweichung ~ sigma, Mittel ~ 0
    assert abs(samples.std() - 0.01) < 0.002
    assert abs(samples.mean()) < 0.002
    # Weich: aufeinanderfolgende Werte sind stark korreliert (kein Zittern)
    corr = np.corrcoef(samples[:-1, 0], samples[1:, 0])[0, 1]
    assert corr > 0.85


def test_smoothed_ou_keeps_amplitude_but_is_smooth():
    # Befund VM 2026-09-14: der reine OU-Prozess hat weisse Geschwindigkeit
    # (0.09 m/s Std bei sigma 1.5 cm), servo_j lief bis 0.57 rad hinterher.
    dt = 1 / 15.0
    raw = OUProcess(sigma=0.015, tau=0.8, dt=dt, rng=np.random.default_rng(1))
    smooth = SmoothedOUProcess(
        sigma=0.015, tau=0.8, tau_smooth=0.25, dt=dt, rng=np.random.default_rng(1)
    )
    x_raw = np.array([raw.step() for _ in range(20000)])
    x = np.array([smooth.step() for _ in range(20000)])
    # Amplitude unveraendert (Verstaerkung aus der stationaeren Kovarianz)
    assert abs(x.std() - 0.015) < 0.0015
    v_raw = np.diff(x_raw, axis=0).std() / dt
    v = np.diff(x, axis=0).std() / dt
    a = np.diff(x, 2, axis=0).std() / dt**2
    assert v < 0.3 * v_raw
    assert a < 0.2
    # Geschwindigkeit von Takt zu Takt korreliert -- keine Stoesse mehr
    vel = np.diff(x[:, 0])
    assert np.corrcoef(vel[:-1], vel[1:])[0, 1] > 0.9


def test_smoothed_ou_stays_within_limits_without_clipping_output():
    lo = np.array([-0.045, -0.045, -0.002])
    hi = np.array([0.045, 0.045, 0.045])
    ou = SmoothedOUProcess(
        sigma=0.015, tau=0.8, tau_smooth=0.25, dt=1 / 15.0,
        rng=np.random.default_rng(4), lo=lo, hi=hi,
    )
    x = np.array([ou.step() for _ in range(5000)])
    assert np.all(x >= lo - 1e-12) and np.all(x <= hi + 1e-12)
    # Auch Richtung Tisch bleibt die Bahn glatt (kein Knick an der Grenze)
    acc_z = np.abs(np.diff(x[:, 2], 2)) * 15.0**2
    assert acc_z.max() < 1.0


def test_funnel_scale_shape():
    assert funnel_scale(config.FUNNEL_ZERO_DIST_M) == 0.0
    assert funnel_scale(config.FUNNEL_START_DIST_M) == 1.0
    assert funnel_scale(1.0) == 1.0
    mid = funnel_scale(
        (config.FUNNEL_START_DIST_M + config.FUNNEL_ZERO_DIST_M) / 2.0
    )
    assert 0.0 < mid < 1.0
    # monoton
    d = np.linspace(0, 0.3, 100)
    s = funnel_scale(d)
    assert np.all(np.diff(s) >= -1e-12)


def _demo_traj():
    def wp(x, z, gripper=False, approach=False):
        return Waypoint(
            geometry.pose_rpy_to_quat([x, 0.0, z, 0, 0, 0]),
            gripper_closed=gripper,
            approach=approach,
        )

    return build_ideal_trajectory(
        [wp(0.0, 0.50), wp(0.25, 0.50), wp(0.25, 0.42, gripper=True, approach=True)]
    )


def test_noise_zero_at_anchors_and_in_dwell():
    traj = _demo_traj()
    noisy = sample_noisy_poses(traj, np.random.default_rng(3))
    offsets = np.linalg.norm(noisy[:, :3] - traj.poses_quat[:, :3], axis=1)
    # Dwell-Schritte: exakt rauschfrei (Greifer schliesst, stillhalten)
    assert np.all(offsets[traj.dwell_mask] < 1e-12)
    # Start und Ende exakt an den geteachten Punkten
    assert offsets[0] < 1e-12 and offsets[-1] < 1e-12
    # Im Transit weit von allen Ankern: Rauschen sichtbar vorhanden
    far = traj.dist_to_anchor > config.FUNNEL_START_DIST_M
    assert far.any()
    assert offsets[far].max() > 0.003


def test_no_noise_jump_after_gripper_change():
    # Befund 2026-09-14: nach dem Greifen sprang das Rauschen in einem
    # Schritt von 0 auf volle Amplitude, waehrend die Backen noch am Objekt
    # waren. Jetzt muss es auch NACH dem Anker weich anlaufen.
    def wp(x, z, gripper=False, approach=False):
        return Waypoint(
            geometry.pose_rpy_to_quat([x, 0.0, z, 0, 0, 0]),
            gripper_closed=gripper,
            approach=approach,
        )

    traj = build_ideal_trajectory(
        [
            wp(0.0, 0.50),
            wp(0.25, 0.50),
            wp(0.25, 0.40, gripper=True, approach=True),
            wp(0.25, 0.60, gripper=True),
            wp(0.0, 0.60, gripper=True),
        ]
    )
    last_dwell = np.where(traj.dwell_mask)[0][-1]
    worst = 0.0
    for seed in range(10):
        noisy = sample_noisy_poses(traj, np.random.default_rng(seed))
        offsets = np.linalg.norm(noisy[:, :3] - traj.poses_quat[:, :3], axis=1)
        # erster Schritt nach dem Dwell: praktisch noch am Greifpunkt (vorher
        # hier volle OU-Amplitude, ~sigma*sqrt(3) = 2.6 cm)
        worst = max(worst, float(offsets[last_dwell + 1]))
    assert worst < 1e-3


def test_noise_respects_asymmetric_limits():
    traj = _demo_traj()
    limits = NoiseLimits()
    noisy = sample_noisy_poses(traj, np.random.default_rng(11), limits=limits)
    delta = noisy[:, :3] - traj.poses_quat[:, :3]
    # Richtung Tisch (negatives z) praktisch keine Auslenkung (AP 2.4)
    assert delta[:, 2].min() >= limits.lo[2] - 1e-12
    assert delta[:, 0].max() <= limits.hi[0] + 1e-12


def test_generate_plan_end_to_end():
    robot = _fixtures.make_robot("sim")
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)

    def wp(dx, dz, gripper=False, approach=False):
        pose = np.asarray(start, dtype=float).copy()
        pose[0] += dx
        pose[2] += dz
        return Waypoint(pose, gripper_closed=gripper, approach=approach)

    ideal = build_ideal_trajectory([wp(0, 0), wp(0.06, -0.02), wp(0.06, -0.05, True, True)])
    workspace = default_workspace(table_height_m=float(start[2]) - 0.30)

    plan = generate_plan(kin, workspace, ideal, home, rng=np.random.default_rng(5))
    n = len(ideal)
    assert plan.joints_ideal.shape == (n, 6)
    assert plan.joints_noisy.shape == (n, 6)
    assert plan.gripper.shape == (n,)
    # Die verrauschte Bahn unterscheidet sich von der idealen ...
    assert not np.allclose(plan.joints_ideal, plan.joints_noisy)
    # ... aber am Ende (Anker, Trichter) sind beide praktisch gleich
    assert np.allclose(plan.joints_ideal[-1], plan.joints_noisy[-1], atol=1e-3)


def test_generate_plan_rejects_on_ideal_collision():
    robot = _fixtures.make_robot("sim")
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)

    ideal = build_ideal_trajectory(
        [
            Waypoint(start),
            Waypoint(np.concatenate([start[:2], [start[2] - 0.04], start[3:7]])),
        ]
    )
    # Tisch schneidet die ideale Bahn -> Wegpunkte selbst unbrauchbar
    workspace = default_workspace(table_height_m=float(start[2]) + 0.05)
    try:
        generate_plan(kin, workspace, ideal, home)
        assert False, "PlanRejected erwartet"
    except PlanRejected as exc:
        assert exc.reasons[0][0] == "ideal_collision"


def test_generate_plan_rejected_on_ik_outage():
    # Fehlerinjektion (AP 0.5): Totalausfall der IK muss als PlanRejected
    # gemeldet werden, nicht als unbehandelte Exception.
    robot = _fixtures.make_robot(
        "sim", faults=FaultProfile(ik_fail_rate=1.0), seed=1
    )
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)  # fk funktioniert, nur ik faellt aus
    ideal = build_ideal_trajectory(
        [
            Waypoint(start),
            Waypoint(np.concatenate([[start[0] + 0.05], start[1:7]])),
        ]
    )
    workspace = default_workspace(table_height_m=float(start[2]) - 0.30)
    try:
        generate_plan(kin, workspace, ideal, home, max_rejects=3)
        assert False, "PlanRejected erwartet"
    except PlanRejected as exc:
        assert exc.reasons[0][0] == "ideal_ik"


def test_generate_plan_rejection_sampling_on_tight_corridor():
    # Ein Hindernis dicht neben der Bahn: die ideale Bahn ist frei, aber
    # verrauschte Bahnen treffen es -- Rejection Sampling muss neu ziehen
    # (oder bei max_rejects sauber mit "collision" ablehnen).
    robot = _fixtures.make_robot("sim")
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)

    ideal = build_ideal_trajectory(
        [
            Waypoint(start),
            Waypoint(np.concatenate([[start[0] + 0.15], start[1:7]])),
        ]
    )
    # Wand seitlich in +y, knapp ausserhalb von Huellkugeln+Marge der
    # IDEALEN Bahn (~5 mm Luft), aber mitten im Rauschkorridor (OU-Sigma
    # 1.5 cm). Der Abstand wird aus den realen Stuetzpunkten berechnet,
    # damit nicht versehentlich schon die ideale Bahn kollidiert. Der
    # Ellbogen bleibt aussen vor -- er folgt dem TCP-Rauschen kaum und
    # wuerde die Wand nur unnoetig weit wegschieben.
    from bc.collision import ArmPoint

    arm_points = [ArmPoint("tool", 0.06), ArmPoint("wrist", 0.08)]
    radii = {p.frame: p.radius for p in arm_points}
    positions = robot.link_positions(home)
    margin = 0.03
    y_reach = max(positions[frame][1] + radii[frame] for frame in radii)
    tool_pos = np.asarray(start[:3])
    wall = Box.from_bounds(
        "Wand",
        lo=(tool_pos[0] - 0.3, y_reach + margin + 0.005, tool_pos[2] - 0.4),
        hi=(tool_pos[0] + 0.3, y_reach + margin + 0.055, tool_pos[2] + 0.4),
    )
    model = CollisionModel(boxes=[wall], arm_points=arm_points, margin=margin)

    outcomes = []
    for seed in range(6):
        try:
            plan = generate_plan(
                kin, model, ideal, home,
                rng=np.random.default_rng(seed), max_rejects=8,
            )
            outcomes.append(plan.rejects)
        except PlanRejected as exc:
            assert all(kind == "collision" for kind, _ in exc.reasons)
            outcomes.append("rejected")

    # Der enge Korridor muss sich bemerkbar machen: mindestens ein Lauf
    # mit Resampling oder Ablehnung.
    assert any(o == "rejected" or (isinstance(o, int) and o > 0) for o in outcomes)

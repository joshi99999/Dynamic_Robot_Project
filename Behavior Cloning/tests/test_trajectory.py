"""Trajektorienplaner: Schrittweite, Dwell, Trichter-Distanz (AP 2.2/2.4/2.6)."""

import _paths  # noqa: F401

import numpy as np

from bc import config, geometry
from bc.trajectory import Waypoint, build_ideal_trajectory


def _wp(x, y, z, gripper=False, approach=False):
    pose = geometry.pose_rpy_to_quat([x, y, z, 0, 0, 0])
    return Waypoint(pose, gripper_closed=gripper, approach=approach)


def test_step_size_encodes_speed():
    # AP 2.6: Die Geschwindigkeit steckt in der Schrittweite bei fester Rate.
    traj = build_ideal_trajectory([_wp(0, 0, 0.5), _wp(0.30, 0, 0.5)])
    steps = np.linalg.norm(np.diff(traj.poses_quat[:, :3], axis=0), axis=1)
    expected = config.TRANSIT_SPEED_MS / config.CONTROL_RATE_HZ
    assert np.all(steps <= expected + 1e-9)
    # In der Mitte (nach der Rampe) faehrt die Bahn mit voller Geschwindigkeit
    mid = steps[len(steps) // 2 - 3 : len(steps) // 2 + 3]
    assert np.all(np.abs(mid - expected) / expected < 0.05)
    # Dauer = konstante Fahrt + eine Rampendauer (zwei halbe Rampen)
    duration = (len(traj) - 1) / config.CONTROL_RATE_HZ
    nominal = 0.30 / config.TRANSIT_SPEED_MS + config.SEGMENT_RAMP_S
    assert abs(duration - nominal) <= 1.0 / config.CONTROL_RATE_HZ + 1e-9


def test_segments_start_and_stop_smoothly():
    # Befund VM 2026-09-14: Geschwindigkeitsspruenge an Start, Dwell und
    # LIN->PTP liessen den Controller ueberschwingen. Jetzt: Anfahren aus
    # dem Stand, Stillstand an jedem Wegpunkt, begrenzte Beschleunigung.
    traj = build_ideal_trajectory(
        [_wp(0, 0, 0.5), _wp(0.20, 0, 0.5), _wp(0.20, 0.15, 0.5, gripper=True)]
    )
    dt = 1.0 / config.CONTROL_RATE_HZ
    v = np.linalg.norm(np.diff(traj.poses_quat[:, :3], axis=0), axis=1) / dt
    v_max = config.TRANSIT_SPEED_MS
    # erster und letzter Takt deutlich unter der Hoechstgeschwindigkeit
    assert v[0] < 0.1 * v_max and v[-1] < 1e-12
    # am Zwischenpunkt (Richtungswechsel) wird angehalten
    corner = int(np.argmin(np.linalg.norm(traj.poses_quat[:, :3] - [0.20, 0, 0.5], axis=1)))
    assert v[corner - 1] < 0.1 * v_max and v[corner] < 0.1 * v_max
    # Beschleunigung nirgends ueber dem Sinus-Profil (+ Abtastreserve)
    acc = np.abs(np.diff(v)) / dt
    a_peak = v_max * np.pi / (2.0 * config.SEGMENT_RAMP_S)
    assert acc.max() <= 1.15 * a_peak


def test_short_segment_keeps_peak_acceleration():
    from bc.trajectory import segment_progress

    dt = 1.0 / config.CONTROL_RATE_HZ
    ramp = config.SEGMENT_RAMP_S
    s = segment_progress(0.1, dt, ramp)  # viel kuerzer als die Rampe
    assert s[-1] == 1.0 and np.all(np.diff(s) >= 0.0)
    expected_duration = 2.0 * np.sqrt(0.1 * ramp)
    assert abs(len(s) * dt - expected_duration) <= dt + 1e-9
    assert np.allclose(segment_progress(0.0, dt), [1.0])


def test_approach_segment_is_slower():
    traj = build_ideal_trajectory(
        [_wp(0, 0, 0.5), _wp(0.10, 0, 0.5), _wp(0.10, 0, 0.44, approach=True)]
    )
    steps = np.linalg.norm(np.diff(traj.poses_quat[:, :3], axis=0), axis=1)
    approach_expected = config.APPROACH_SPEED_MS / config.CONTROL_RATE_HZ
    # Die letzten Schritte (Approach-Segment) muessen deutlich kleiner sein
    assert steps[-2] <= approach_expected + 1e-9


def test_gripper_dwell_inserted():
    traj = build_ideal_trajectory(
        [_wp(0, 0, 0.5), _wp(0.05, 0, 0.5, gripper=True)]
    )
    assert int(traj.dwell_mask.sum()) == config.GRIPPER_DWELL_STEPS
    dwell_idx = np.where(traj.dwell_mask)[0]
    # Waehrend des Dwell: Position haelt, Greifer bereits "zu"
    for i in dwell_idx:
        assert np.allclose(traj.poses_quat[i], traj.poses_quat[dwell_idx[0]])
        assert traj.gripper[i] == config.GRIPPER_CLOSED
    # Vor dem Dwell ist der Greifer offen
    assert traj.gripper[dwell_idx[0] - 1] == config.GRIPPER_OPEN


def test_anchor_distance_is_zero_at_start_gripper_change_and_end():
    traj = build_ideal_trajectory(
        [_wp(0, 0, 0.5), _wp(0.20, 0, 0.5, gripper=True), _wp(0.20, 0, 0.7, gripper=True)]
    )
    d = traj.dist_to_anchor
    dwell_idx = np.where(traj.dwell_mask)[0]
    assert d[0] == 0.0  # Episodenstart
    assert d[dwell_idx[0]] < 1e-9  # Greiferwechsel
    assert d[-1] < 1e-9  # Episodenende / Uebergabe
    # Vor dem Greifen faellt, nach dem Greifen steigt der Abstand wieder --
    # beidseitig des Ankers, nicht nur davor (Befund 2026-09-14).
    after = d[dwell_idx[-1] :]
    k = int(np.argmax(after))
    assert after[k] > 0.05
    assert np.all(np.diff(after[: k + 1]) >= -1e-9)
    assert np.all(np.diff(after[k:]) <= 1e-9)


def test_ptp_segment_interpolates_in_joint_space():
    import _fixtures

    robot = _fixtures.make_robot("sim")
    q_a = robot.read_state().joints
    q_b = q_a + np.array([0.4, -0.1, 0.1, 0.0, 0.2, 0.3])
    wp_a = Waypoint(robot.fk(q_a), name="A", joints=q_a)
    wp_b = Waypoint(robot.fk(q_b), name="B", motion="ptp", joints=q_b)

    traj = build_ideal_trajectory([wp_a, wp_b], fk=robot.fk)
    assert traj.joints.shape == (len(traj), 6)
    assert np.allclose(traj.joints[0], q_a)
    assert np.allclose(traj.joints[-1], q_b)
    # Linear im Gelenkraum: alle Gelenkschritte parallel zu q_b - q_a
    # (Betrag folgt dem Rampenprofil)
    dq = np.diff(traj.joints, axis=0)
    direction = (q_b - q_a) / np.linalg.norm(q_b - q_a)
    along = dq @ direction
    assert np.all(along >= -1e-12)
    assert np.allclose(dq, np.outer(along, direction), atol=1e-9)
    # Keine Gelenkgeschwindigkeit ueber dem PTP-Limit
    max_step = config.PTP_JOINT_SPEED_RADS / config.CONTROL_RATE_HZ
    assert np.max(np.abs(dq)) <= max_step + 1e-9
    # Posen sind die FK der Gelenkstellungen (nicht die Gerade im Raum)
    mid = len(traj) // 2
    assert np.allclose(traj.poses_quat[mid][:3], robot.fk(traj.joints[mid])[:3])


def test_ptp_requires_joints_and_fk():
    wp_a = _wp(0, 0, 0.5)
    wp_b = Waypoint(_wp(0.1, 0, 0.5).pose_quat, motion="ptp", joints=np.zeros(6))
    try:
        build_ideal_trajectory([wp_a, wp_b], fk=lambda q: None)
        assert False, "ValueError erwartet (Start ohne Gelenkstellung)"
    except ValueError:
        pass


def test_joint_derivatives_match_motion():
    from bc.trajectory import joint_derivatives

    rate = config.CONTROL_RATE_HZ
    t = np.arange(40) / rate
    q = np.stack([0.3 * t, 0.1 * t**2, np.zeros_like(t)], axis=1)
    vel, acc = joint_derivatives(q, rate)
    # Innen exakt (zentrale Differenzen sind fuer Polynome 2. Grades exakt)
    assert np.allclose(vel[1:-1, 0], 0.3)
    assert np.allclose(vel[1:-1, 1], 0.2 * t[1:-1])
    assert np.allclose(acc[1:-1, 1], 0.2)
    # Stillstand bleibt Stillstand -- Dwell ergibt 0
    assert np.allclose(vel[:, 2], 0.0) and np.allclose(acc[:, 2], 0.0)


def test_requires_two_waypoints():
    try:
        build_ideal_trajectory([_wp(0, 0, 0.5)])
        assert False, "ValueError erwartet"
    except ValueError:
        pass

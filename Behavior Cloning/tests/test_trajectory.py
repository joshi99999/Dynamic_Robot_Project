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
    assert abs(steps.mean() - expected) / expected < 0.1


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


def test_dist_to_grasp_decreases_to_zero():
    traj = build_ideal_trajectory(
        [_wp(0, 0, 0.5), _wp(0.20, 0, 0.5, gripper=True), _wp(0.20, 0, 0.6)]
    )
    n_pre = np.where(traj.dwell_mask)[0][0]
    d = traj.dist_to_grasp[:n_pre]
    # Distanz zum Greifpunkt faellt monoton auf ~0
    assert d[0] > d[-1]
    assert d[-1] < 1e-9
    assert np.all(np.diff(d) <= 1e-9)


def test_requires_two_waypoints():
    try:
        build_ideal_trajectory([_wp(0, 0, 0.5)])
        assert False, "ValueError erwartet"
    except ValueError:
        pass

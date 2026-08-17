"""URDF-Kette und SimRobot-Kinematik (AP 0.4, Gleis A).

Prueft die LOGIK der Offline-Kinematik: Konsistenz FK<->IK, Seed-Semantik,
Fehlerverhalten. Die GEOMETRIE ist bewusst nicht Gegenstand -- das Modell
ist gegen die reale LARA 5 unbestaetigt (Kalibrierung: Gleis B, AP 0.6).
"""

import _paths  # noqa: F401

import numpy as np

import _fixtures
from bc import config, geometry
from bc.kinematics import IKFailure, Kinematics
from bc.ports import IKError
from bc.urdf import KinematicChain


def test_chain_structure():
    chain = KinematicChain.from_urdf(config.URDF_PATH)
    assert chain.n_joints == 6
    lims = chain.joint_limits
    assert len(lims) == 6
    # Die URDF-Grenzen muessen zu den (provisorischen) config-Werten passen
    for (lo, hi), (clo, chi) in zip(lims, config.JOINT_LIMITS_RAD):
        assert abs(lo - clo) < 1e-6 and abs(hi - chi) < 1e-6


def test_fk_deterministic_and_valid():
    robot = _fixtures.make_robot("sim")
    q = robot.read_state().joints
    p1 = robot.fk(q)
    p2 = robot.fk(q)
    assert np.allclose(p1, p2)
    assert p1.shape == (7,)
    assert abs(np.linalg.norm(p1[3:7]) - 1.0) < 1e-9  # Einheitsquaternion


def test_ik_reaches_fk_pose():
    robot = _fixtures.make_robot("sim")
    home = robot.read_state().joints
    target_q = home + np.array([0.05, -0.04, 0.06, 0.03, -0.05, 0.04])
    target_pose = robot.fk(target_q)

    sol = robot.ik(target_pose, home)
    reached = robot.fk(sol)
    assert np.linalg.norm(reached[:3] - target_pose[:3]) < 1e-4
    assert geometry.quat_angle_between(reached[3:7], target_pose[3:7]) < 1e-3


def test_ik_seed_semantics():
    # Seed-Semantik (AP 2.4): kleine Posenaenderung => Loesung nahe am Seed
    robot = _fixtures.make_robot("sim")
    home = robot.read_state().joints
    pose = robot.fk(home)
    pose[0] += 0.002
    sol = robot.ik(pose, home)
    assert float(np.max(np.abs(sol - home))) < 0.05


def test_ik_unreachable_raises():
    robot = _fixtures.make_robot("sim")
    home = robot.read_state().joints
    pose = robot.fk(home)
    pose[:3] = [5.0, 5.0, 5.0]  # weit ausserhalb jeder Reichweite
    try:
        robot.ik(pose, home)
        assert False, "IKError erwartet"
    except IKError:
        pass


def test_solve_path_warm_start_and_guards():
    robot = _fixtures.make_robot("sim")
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)
    end = start.copy()
    end[0] += 0.05
    end[2] -= 0.03
    poses = geometry.pose_path([start, end], steps_per_segment=20)

    joints = kin.solve_path(poses, home)
    assert joints.shape == (21, 6)
    # Warm-Start: aufeinanderfolgende Loesungen liegen dicht beieinander
    deltas = np.max(np.abs(np.diff(joints, axis=0)), axis=1)
    assert float(deltas.max()) < config.IK_MAX_DELTA_Q_RAD


def test_solve_path_reports_failing_index():
    robot = _fixtures.make_robot("sim")
    kin = Kinematics(robot)
    home = robot.read_state().joints
    good = robot.fk(home)
    bad = good.copy()
    bad[:3] = [4.0, 4.0, 4.0]
    try:
        kin.solve_path(np.stack([good, bad]), home)
        assert False, "IKFailure erwartet"
    except IKFailure as exc:
        assert exc.index == 1


def test_conditioning_finite_at_home():
    robot = _fixtures.make_robot("sim")
    kin = Kinematics(robot)
    cond = kin.conditioning(robot.read_state().joints)
    assert set(cond) == {"+X", "-X", "+Y", "-Y", "+Z", "-Z"}
    assert all(np.isfinite(v) for v in cond.values())

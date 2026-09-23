"""Contract-Tests des RobotPort (AP 0.5) -- die Abnahmeliste der Adapter.

Dieselbe Suite laeuft gegen JEDE Implementierung:

    pytest tests/contract -q                  # sim (hardwarefrei, heute)
    pytest tests/contract -q --robot=neura    # Anlage (Hardwaretag, AP 0.6)

Geprueft werden die ZUSAGEN des Ports (Einheiten, Formen, Seed-Semantik,
Fehlerverhalten, Lebenszyklus) -- nicht die Geometrie des Modells.

Hinweis Anlage: Die Tests lesen nur und aktivieren kurz das
Servo-Interface, ohne Bewegungsbefehle abzusetzen; servo_j und
move_to_joints werden nur mit der AKTUELLEN Ist-Stellung aufgerufen
(Halten: Geschwindigkeit und Beschleunigung bewusst 0). Trotzdem: Not-Halt
in Reichweite. Gegen den Neura-Adapter laufen die Tests nur, wenn der
Controller is_robot_in_simulation() == True meldet (Adapter-Sperre).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import _paths  # noqa: F401,E402

import numpy as np  # noqa: E402

from bc import config, geometry  # noqa: E402
from bc.ports import IKError, RobotError, RobotState, ServoLimitError  # noqa: E402


def test_dof(robot):
    assert robot.dof == 6


def test_read_state_contract(robot):
    state = robot.read_state()
    assert isinstance(state, RobotState)
    assert len(state.joints) == robot.dof
    tcp = np.asarray(state.tcp_quat, dtype=float)
    assert tcp.shape == (7,)
    assert abs(np.linalg.norm(tcp[3:7]) - 1.0) < 1e-6  # Einheitsquaternion
    assert np.linalg.norm(tcp[:3]) < 2.0  # plausible Reichweite (Meter!)
    assert state.t > 0 and state.t_joints > 0 and state.t_pose > 0
    assert isinstance(state.gripper_closed, bool)


def test_fk_matches_reported_pose(robot):
    # fk(gemeldete Winkel) muss die gemeldete TCP-Pose reproduzieren --
    # DIE Konsistenzzusage zwischen State-Abgriff und Kinematik (AP 1.5.1)
    state = robot.read_state()
    pose = robot.fk(state.joints, frame="tool")
    assert np.linalg.norm(pose[:3] - np.asarray(state.tcp_quat)[:3]) < 1e-3
    assert (
        geometry.quat_angle_between(pose[3:7], np.asarray(state.tcp_quat)[3:7])
        < 1e-2
    )


def test_fk_supports_collision_frames(robot):
    joints = robot.read_state().joints
    for frame in config.COLLISION_CHECK_FRAMES:
        pose = robot.fk(joints, frame=frame)
        assert np.asarray(pose).shape == (7,)


def test_link_positions_contract(robot):
    joints = robot.read_state().joints
    positions = robot.link_positions(joints)
    for frame in config.COLLISION_CHECK_FRAMES:
        assert frame in positions
        assert np.asarray(positions[frame]).shape == (3,)
    # Stuetzpunkte muessen mit den fk-Frames uebereinstimmen
    for frame in config.COLLISION_CHECK_FRAMES:
        fk_pos = robot.fk(joints, frame=frame)[:3]
        assert np.linalg.norm(positions[frame] - fk_pos) < 1e-6


def test_ik_roundtrip_and_seed(robot):
    state = robot.read_state()
    pose = robot.fk(state.joints)

    # Roundtrip an der aktuellen Pose
    sol = robot.ik(pose, state.joints)
    assert np.asarray(sol).shape == (robot.dof,)
    back = robot.fk(sol)
    assert np.linalg.norm(back[:3] - pose[:3]) < 1e-3

    # Seed-Semantik: 2-mm-Verschiebung => kleine Gelenkaenderung
    near = np.asarray(pose, dtype=float).copy()
    near[0] += 0.002
    sol_near = robot.ik(near, state.joints)
    assert float(np.max(np.abs(sol_near - np.asarray(state.joints)))) < 0.1


def test_ik_unreachable_raises_ikerror(robot):
    state = robot.read_state()
    impossible = np.array([10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0])
    try:
        robot.ik(impossible, state.joints)
        assert False, "IKError erwartet"
    except IKError:
        pass


def test_servo_lifecycle(robot):
    joints = robot.read_state().joints
    hold = [0.0] * robot.dof  # Halten: Geschwindigkeit/Beschleunigung 0
    # Vor Aktivierung verweigert servo_j
    try:
        robot.servo_j(joints, hold, hold)
        assert False, "RobotError erwartet (Servo nicht aktiv)"
    except RobotError:
        pass

    robot.activate_servo("position")
    robot.servo_j(joints, hold, hold)  # Halten der Ist-Stellung: bewegungsfrei
    robot.deactivate_servo()
    try:
        robot.servo_j(joints, hold, hold)
        assert False, "RobotError erwartet (Servo deaktiviert)"
    except RobotError:
        pass


def test_servo_guard_refuses_jump(robot):
    # Sprung- und Geschwindigkeitsfilter im Adapter (servo.ServoGuard): ein
    # 0.3-rad-Sprung wird VOR dem Senden abgelehnt und loest den
    # Software-Stopp aus. Bewegungsfrei -- es wird nichts gesendet.
    joints = np.asarray(robot.read_state().joints, dtype=float)
    hold = [0.0] * robot.dof
    robot.activate_servo("position")
    try:
        robot.servo_j(joints + 0.3, hold, hold)
        assert False, "ServoLimitError erwartet"
    except ServoLimitError:
        pass
    finally:
        robot.deactivate_servo()
    assert robot.stop_requested
    after = np.asarray(robot.read_state().joints, dtype=float)
    assert float(np.max(np.abs(after - joints))) <= config.START_POSE_TOL_RAD
    robot.clear_stop()


def test_move_to_current_joints_is_noop(robot):
    joints = np.asarray(robot.read_state().joints, dtype=float)
    robot.move_to_joints(joints)
    after = np.asarray(robot.read_state().joints, dtype=float)
    assert float(np.max(np.abs(after - joints))) <= config.START_POSE_TOL_RAD


def test_gripper_command_is_nonblocking_and_tracked(robot):
    before = robot.gripper_closed
    robot.gripper_command(not before)
    assert robot.gripper_closed == (not before)
    robot.gripper_command(before)
    assert robot.gripper_closed == before


def test_emergency_stop_contract(robot):
    robot.activate_servo("position")
    robot.emergency_stop()
    assert robot.stop_requested
    # Nach Not-Halt: servo_j muss verweigern
    try:
        hold = [0.0] * robot.dof
        robot.servo_j(robot.read_state().joints, hold, hold)
        assert False, "RobotError erwartet (nach Not-Halt)"
    except RobotError:
        pass
    robot.clear_stop()
    assert not robot.stop_requested

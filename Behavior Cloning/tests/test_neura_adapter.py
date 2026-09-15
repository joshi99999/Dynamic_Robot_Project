"""Neura-Adapter ohne Controller: Sicherheitssperre, servo_j, Zeitstempel,
Greifer, Punkte (AP 1.5 / Befunde VM 2026-09-09 und 2026-09-14).

Statt der VM wird ein Fake-Client injiziert, der die dort gemessenen
Eigenheiten nachbildet (Zeitstempel 0.0, kein ``gripper_name``, servo_j mit
drei Listen). Die Abnahme gegen die echte VM bleibt Aufgabe der
Contract-Suite (``run_all.py --robot=neura``).
"""

import _paths  # noqa: F401

import numpy as np

from bc import config, geometry
from bc.adapters.neura import (
    GRIPPER_HARDWARE,
    GRIPPER_LOGGED,
    GRIPPER_UNAVAILABLE,
    NeuraRobot,
)
from bc.clock import host_time
from bc.ports import MotionRefused, RobotError


class FakeNeuraClient(object):
    """Bildet ``neurapy.robot.Robot`` so nach, wie ihn die VM bedient."""

    dof = 6

    def __init__(self, in_simulation=True, gripper=False, sim_raises=False):
        self._in_simulation = in_simulation
        self._sim_raises = sim_raises
        if gripper:
            self.gripper_name = "RobotiQ"
        self.calls = []
        self.joints = [0.0, -0.2, 1.6, 0.0, 1.1, 0.0]
        self.points = {
            "PICK": [-0.00675369, 0.287347023, 1.925959684, 0.005852666, 0.87913435, -0.011515414]
        }

    def _log(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def is_robot_in_simulation(self):
        self._log("is_robot_in_simulation")
        if self._sim_raises:
            raise RuntimeError("keine Antwort")
        return self._in_simulation

    def init_program(self):
        self._log("init_program")

    def power_on(self):
        self._log("power_on")

    def set_override(self, value):
        self._log("set_override", value)

    def stop(self):
        self._log("stop")

    def get_selected_tool_name(self):
        return "NoTool"

    def get_current_joint_angles(self):
        return list(self.joints)

    def get_current_joint_angles_with_timestamp(self):
        return [list(self.joints), 0.0]  # VM: Zeitstempel konstant 0.0

    def compute_forward_kinematics(self, joint_angles, target_frame, representation):
        # Einfaches, aber eindeutiges Modell: Position aus den ersten drei
        # Winkeln, Orientierung "Greifer unten" mit Pitch aus Gelenk 5.
        q = joint_angles
        return [0.4 + 0.1 * q[0], 0.1 * q[1], 0.2 + 0.1 * q[2], np.pi, q[4] - 0.8, np.pi]

    def get_point_names(self):
        return list(self.points)

    def get_point(self, name, representation="Joint"):
        joints = self.points[name]
        if representation == "Joint":
            return list(joints)
        return self.compute_forward_kinematics(joints, "tool", "rpy")

    def activate_servo_interface(self, mode):
        self._log("activate_servo_interface", mode)

    def deactivate_servo_interface(self):
        self._log("deactivate_servo_interface")

    def servo_j(self, position, velocity, acceleration):
        self._log("servo_j", position, velocity, acceleration)
        return 0

    def move_joint(self, target_joint, speed, acceleration, current_joint_angles):
        self._log("move_joint", target_joint=target_joint)
        self.joints = list(target_joint[0])

    def grasp(self):
        self._log("grasp")

    def release(self):
        self._log("release")


def _names(client):
    return [c[0] for c in client.calls]


def test_simulation_check_runs_first_and_allows_motion():
    client = FakeNeuraClient(in_simulation=True)
    robot = NeuraRobot(robot=client).connect(power_on=True)
    assert _names(client)[0] == "is_robot_in_simulation"
    assert robot.in_simulation is True and robot.motion_allowed
    robot.activate_servo()
    hold = [0.0] * 6
    robot.servo_j(client.joints, hold, hold)


def test_real_robot_refused_without_explicit_release():
    # Befund 2026-09-14: VM und Anlage teilen eine IP. Wer vergisst, die
    # Anlage freizugeben, bekommt einen Abbruch -- nie eine Bewegung.
    for client in (
        FakeNeuraClient(in_simulation=False),
        FakeNeuraClient(sim_raises=True),  # Abfrage scheitert -> keine Sim
    ):
        try:
            NeuraRobot(robot=client).connect(power_on=True)
            assert False, "MotionRefused erwartet (power_on ohne Freigabe)"
        except MotionRefused:
            pass
        assert "power_on" not in _names(client)

        robot = NeuraRobot(robot=client).connect()  # nur lesen: erlaubt
        assert not robot.motion_allowed
        robot.read_state()
        for action in (
            lambda: robot.activate_servo(),
            lambda: robot.move_to_joints(client.joints),
        ):
            try:
                action()
                assert False, "MotionRefused erwartet"
            except MotionRefused:
                pass
        assert "activate_servo_interface" not in _names(client)
        assert "move_joint" not in _names(client)


def test_real_robot_with_explicit_release():
    client = FakeNeuraClient(in_simulation=False, gripper=True)
    robot = NeuraRobot(robot=client, allow_real=True).connect(power_on=True)
    assert robot.motion_allowed and robot.in_simulation is False
    robot.activate_servo()


def test_servo_j_sends_three_lists_and_requires_derivatives():
    client = FakeNeuraClient()
    robot = NeuraRobot(robot=client).connect()
    robot.activate_servo()
    try:
        robot.servo_j(client.joints)
        assert False, "ValueError erwartet (ohne Geschwindigkeit/Beschleunigung)"
    except ValueError:
        pass
    vel = [0.1, 0, 0, 0, 0, 0]
    acc = [0.5, 0, 0, 0, 0, 0]
    robot.servo_j(np.asarray(client.joints), vel, acc)
    name, args, _ = client.calls[-1]
    assert name == "servo_j" and len(args) == 3
    assert all(isinstance(a, list) and len(a) == 6 for a in args)
    assert args[1] == vel and args[2] == acc


def test_servo_error_code_raises():
    client = FakeNeuraClient()
    client.servo_j = lambda p, v, a: 3
    robot = NeuraRobot(robot=client).connect()
    robot.activate_servo()
    hold = [0.0] * 6
    try:
        robot.servo_j(client.joints, hold, hold)
        assert False, "RobotError erwartet (Fehlercode 3)"
    except RobotError:
        pass


def test_zero_controller_timestamp_falls_back_to_host_time():
    # Befund VM: *_with_timestamp liefert 0.0 -> vorher als 1970 gewertet,
    # jeder Frame waere eine Sync-Verletzung gewesen.
    robot = NeuraRobot(robot=FakeNeuraClient()).connect()
    before = host_time()
    state = robot.read_state()
    after = host_time()
    assert before <= state.t_joints <= after
    assert state.t_pose == state.t_joints
    assert robot.last_controller_timestamp is None


def test_gripper_logged_in_simulation_without_gripper():
    client = FakeNeuraClient(in_simulation=True, gripper=False)
    robot = NeuraRobot(robot=client).connect()
    assert robot.gripper_mode == GRIPPER_LOGGED
    robot.gripper_command(True)
    robot.gripper_command(False)
    assert robot.gripper_closed is False
    assert [closed for _, closed in robot.gripper_log] == [True, False]
    assert "grasp" not in _names(client) and "release" not in _names(client)


def test_gripper_hardware_and_unavailable_modes():
    client = FakeNeuraClient(in_simulation=True, gripper=True)
    robot = NeuraRobot(robot=client).connect()
    assert robot.gripper_mode == GRIPPER_HARDWARE
    robot.gripper_command(True)
    assert _names(client)[-1] == "grasp"

    real = NeuraRobot(robot=FakeNeuraClient(in_simulation=False)).connect()
    assert real.gripper_mode == GRIPPER_UNAVAILABLE
    try:
        real.gripper_command(True)
        assert False, "RobotError erwartet (Anlage ohne Greifer)"
    except RobotError:
        pass


def test_get_point_uses_joints_and_crosschecks_cartesian():
    client = FakeNeuraClient()
    robot = NeuraRobot(robot=client).connect()
    joints, pose = robot.get_point("PICK")
    assert np.allclose(joints, client.points["PICK"])
    assert np.allclose(pose, robot.fk(joints))

    try:
        robot.get_point("GIBT_ES_NICHT")
        assert False, "KeyError erwartet"
    except KeyError:
        pass

    # Gespeicherte Cartesian-Pose passt nicht zur Gelenkstellung (anderes Tool)
    original = client.get_point

    def shifted(name, representation="Joint"):
        value = original(name, representation)
        if representation == "Cartesian":
            value = list(value)
            value[2] += 0.21  # Tool-Offset RobotiQ laut get_tools()
        return value

    client.get_point = shifted
    try:
        robot.get_point("PICK")
        assert False, "RobotError erwartet (Gegenprobe)"
    except RobotError as exc:
        assert "Tool" in str(exc)


def test_move_to_joints_uses_move_joint_and_checks_target():
    client = FakeNeuraClient()
    robot = NeuraRobot(robot=client).connect(power_on=True)
    target = [0.1, -0.1, 1.5, 0.0, 1.0, 0.0]
    robot.move_to_joints(target)
    assert client.joints == target

    client.move_joint = lambda **kwargs: None  # faehrt nicht
    try:
        robot.move_to_joints([0.3, -0.1, 1.5, 0.0, 1.0, 0.0])
        assert False, "RobotError erwartet (Ziel nicht erreicht)"
    except RobotError:
        pass

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


class UrdfMatchingClient(FakeNeuraClient):
    """Fake-Steuerung, deren FK exakt die URDF-Kette des Projekts ist.

    Damit laesst sich der Vortest aus :meth:`NeuraRobot._check_local_fk`
    hardwarefrei pruefen: stimmt das Modell, soll read_state() lokal
    rechnen; weicht es ab, muss die Steuerung massgeblich bleiben.
    """

    def __init__(self, offset_m=0.0, **kwargs):
        super().__init__(**kwargs)
        from bc.urdf import KinematicChain

        self._chain = KinematicChain.from_urdf(config.URDF_PATH)
        self._offset = float(offset_m)

    def _pose_quat(self, joint_angles):
        T = self._chain.fk(np.asarray(joint_angles, dtype=float))
        quat = geometry.matrix_to_quat(T[:3, :3])
        pos = T[:3, 3].copy()
        pos[2] += self._offset  # kuenstliche Modellabweichung
        return np.concatenate([pos, quat])

    def compute_forward_kinematics(self, joint_angles, target_frame, representation):
        self._log("compute_forward_kinematics")
        return list(geometry.pose_quat_to_rpy(self._pose_quat(joint_angles)))

    def get_flange_pose(self):
        return list(geometry.pose_quat_to_rpy(self._pose_quat(self.joints)))

    def get_tcp_pose_quaternion(self):
        return list(self._pose_quat(self.joints))


def test_local_fk_used_when_urdf_matches_controller():
    bot = NeuraRobot(robot=UrdfMatchingClient()).connect()

    assert bot.local_fk_active
    assert bot.local_fk_check["pos_err_m"] <= config.LOCAL_FK_TOL_POS_M
    # Mehrere Stellungen geprueft, nicht nur die aktuelle -- bei lauter
    # Nullen waere ein invertiertes Achsvorzeichen unsichtbar.
    assert bot.local_fk_check["stellungen"] >= 4

    # read_state() rechnet die Pose jetzt lokal: kein FK-Aufruf mehr
    bot.robot.calls.clear()
    state = bot.read_state()
    assert not any(name == "compute_forward_kinematics" for name, _, _ in bot.robot.calls)
    # ... und liefert trotzdem dieselbe Pose wie die Steuerung
    erwartet = bot.fk(state.joints, frame="tool")
    assert np.allclose(state.tcp_quat, erwartet, atol=1e-9)


def test_local_fk_refused_when_urdf_deviates():
    bot = NeuraRobot(robot=UrdfMatchingClient(offset_m=0.02)).connect()

    assert not bot.local_fk_active
    assert bot.local_fk_check["pos_err_m"] > config.LOCAL_FK_TOL_POS_M
    assert "weicht von der Steuerung ab" in bot.local_fk_check["grund"]

    # Ohne bestaetigtes Modell bleibt die Steuerung massgeblich
    bot.robot.calls.clear()
    bot.read_state()
    assert any(name == "compute_forward_kinematics" for name, _, _ in bot.robot.calls)


def test_local_fk_can_be_switched_off():
    bot = NeuraRobot(robot=UrdfMatchingClient(), local_fk=False).connect()

    assert not bot.local_fk_active
    assert bot.local_fk_check["grund"] == "per Konfiguration aus"


class ModeClient(FakeNeuraClient):
    """Fake mit Betriebsmodus und Diagnose -- fuer init_program-Faelle.

    ``teach``: steht im Teach-Modus (init_program scheitert dann).
    ``critical``: Fehlerzustand nach RCSC_10x (init_program scheitert dann).
    """

    def __init__(self, teach=False, critical=False, **kwargs):
        super().__init__(**kwargs)
        self.teach = teach
        self.critical = critical

    def is_robot_in_teach_mode(self):
        self._log("is_robot_in_teach_mode")
        return self.teach

    def switch_to_automatic_mode(self):
        self._log("switch_to_automatic_mode")
        self.teach = False

    def get_diagnostics(self):
        if self.critical:
            return {"critical": True, "issues": {"other_errors": ["undefined error"]}}
        return {"critical": False, "issues": {}}

    def program_status(self):
        return "NOT_RUNNING"

    def init_program(self):
        self._log("init_program")
        if self.teach or self.critical:
            raise Exception("Unable to switch to play mode. Check if robot in automatic mode")


def _schnell(fn):
    """init_program-Wiederholungen ohne Wartezeit (6 x 1 s im Ernstfall)."""
    from bc.adapters import neura as neura_module

    alt = neura_module.INIT_PROGRAM_RETRY_S
    neura_module.INIT_PROGRAM_RETRY_S = 0.0
    try:
        return fn()
    finally:
        neura_module.INIT_PROGRAM_RETRY_S = alt


def test_readonly_connect_does_not_init_program():
    """Lesend verbinden geht auch im Teach-Modus und im Fehlerzustand.

    Vorher scheiterte schon die GUI-Verbindung daran, und drei Werkzeuge
    mussten am Adapter vorbei arbeiten (Laptop-Inbetriebnahme-Befunde 3).
    """
    for client in (ModeClient(teach=True), ModeClient(critical=True)):
        robot = NeuraRobot(robot=client).connect()
        assert "init_program" not in _names(client)
        robot.read_state()  # lesen geht


def test_automatic_switch_happens_before_init_program():
    client = ModeClient(teach=True)
    NeuraRobot(robot=client).connect(power_on=True, ensure_automatic=True)
    names = _names(client)
    assert names.index("switch_to_automatic_mode") < names.index("init_program")


def test_first_motion_after_readonly_connect_inits_program_once():
    client = ModeClient()
    robot = NeuraRobot(robot=client).connect()
    assert "init_program" not in _names(client)
    robot.activate_servo()
    names = _names(client)
    assert names.count("init_program") == 1
    assert names.index("init_program") < names.index("activate_servo_interface")
    robot.deactivate_servo()
    robot.activate_servo()
    assert _names(client).count("init_program") == 1  # nicht bei jedem Mal


def test_stop_requires_a_new_init_program():
    client = ModeClient()
    robot = NeuraRobot(robot=client).connect(power_on=True)
    assert _names(client).count("init_program") == 1
    robot.emergency_stop()
    robot.clear_stop()
    assert _names(client).count("init_program") == 2


def test_init_failure_names_the_real_cause():
    """Die Meldung des Controllers nennt immer den Modus -- der Adapter fragt nach."""
    def scheitern(client):
        try:
            _schnell(lambda: NeuraRobot(robot=client).connect(power_on=True))
        except RobotError as exc:
            return str(exc)
        raise AssertionError("RobotError erwartet")

    text = scheitern(ModeClient(critical=True))
    assert "FEHLERZUSTAND" in text and "Reset Control" in text

    text = scheitern(ModeClient(teach=True))
    assert "TEACH-MODUS" in text

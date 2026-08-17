"""Recorder: asymmetrische Paarung, Dwell, Verwerf-Logik (AP 2.1/2.4/5.2).

Der End-to-End-Kern der hardwarefreien Absicherung: SimRobot + SimCameras
+ SimClock, komplette Episodengenerierung und -aufzeichnung.
"""

import _paths  # noqa: F401

import numpy as np

import _fixtures
from bc import config
from bc.adapters.cam_sim import CameraFaultProfile, SimCamera
from bc.adapters.sim_robot import SimRobot
from bc.capture import DirectCapture
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.kinematics import Kinematics
from bc.noise import generate_plan
from bc.recorder import EpisodeRecorder
from bc.trajectory import Waypoint, build_ideal_trajectory


def _setup(robot_cls=SimRobot, cam_faults=None, **robot_kwargs):
    clock = SimClock()
    robot = robot_cls(clock=clock, **robot_kwargs).connect()
    cams = [
        SimCamera(cfg, clock=clock, faults=cam_faults)
        for cfg in config.SIM_CAMERAS
    ]
    captures = [DirectCapture(c).start() for c in cams]
    return clock, robot, captures


def _plan(robot, gripper=True):
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)

    def wp(dx, dz, g=False, approach=False):
        pose = np.asarray(start, dtype=float).copy()
        pose[0] += dx
        pose[2] += dz
        return Waypoint(pose, gripper_closed=g, approach=approach)

    waypoints = [wp(0, 0), wp(0.05, -0.02)]
    if gripper:
        # Greifen + anschliessendes Heben: erst dadurch liegt zwischen
        # Greiferbefehl und Folgebewegung die volle Totzeit (AP 2.1).
        waypoints.append(wp(0.05, -0.04, g=True, approach=True))
        waypoints.append(wp(0.05, -0.01, g=True))
    ideal = build_ideal_trajectory(waypoints)
    workspace = default_workspace(table_height_m=float(start[2]) - 0.30)
    return generate_plan(
        kin, workspace, ideal, home, rng=np.random.default_rng(0)
    )


def test_record_full_episode():
    clock, robot, captures = _setup()
    plan = _plan(robot)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan, metadata={"mode": "test"})

    n = len(plan)
    assert len(episode) == n
    assert not episode.discarded, episode.discard_reason
    episode.validate()

    # Asymmetrische Paarung (AP 2.4): Action = IDEALE Winkel bei t+1
    for i in range(n):
        j = min(i + 1, n - 1)
        assert np.allclose(
            episode.arrays["action"][i, :6],
            plan.joints_ideal[j].astype(np.float32),
        )
        assert episode.arrays["action"][i, 6] == plan.gripper[j]

    # Observation = ECHTER (verrauschter) Zustand: der SimRobot folgt
    # servo_j exakt, also muss der State der verrauschten Bahn entsprechen.
    # Schritt 0 zeigt noch die Startpose (Zustand wird nach servo_j des
    # aktuellen Schritts gelesen) -> ab Schritt 1 vergleichen.
    for i in range(1, n):
        assert np.allclose(
            episode.arrays["observation.state"][i, :6],
            plan.joints_noisy[i].astype(np.float32),
            atol=1e-6,
        )

    # Sync: mit gemeinsamer SimClock ist alles im Budget
    assert episode.arrays["aux.sync_ok"].all()
    assert episode.metadata["recorded_steps"] == n


def test_gripper_dwell_respected():
    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=True)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)

    # Der Greifer-Befehl fiel waehrend der Episode
    assert robot.gripper_closed
    # Nach den Dwell-Schritten ist die Totzeit real verstrichen (die
    # SimClock lief mit dem Pacer weiter) -> Backen sind "angekommen".
    assert robot.gripper_settled()

    # Im Datensatz: Greiferkanal wechselt genau am Dwell-Beginn
    g = episode.arrays["observation.state"][:, 13]
    changes = np.where(np.diff(g) != 0)[0]
    assert len(changes) == 1
    dwell_start = np.where(plan.dwell_mask)[0][0]
    assert changes[0] == dwell_start - 1


def test_discard_on_stale_camera():
    # Fehlerinjektion (AP 0.5): eine Kamera haengt 50 ms hinterher --
    # ueber dem 30-ms-Budget -> Episode muss verworfen werden (AP 5.2).
    clock, robot, captures = _setup(
        cam_faults=CameraFaultProfile(stale_s=0.05)
    )
    plan = _plan(robot, gripper=False)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)
    assert episode.discarded
    assert "latenzbudget" in episode.discard_reason
    assert not episode.arrays["aux.sync_ok"].any()


class TrippingRobot(SimRobot):
    """Loest nach N servo-Befehlen den Schutzstopp aus (Testhelfer)."""

    TRIP_AFTER = 5

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._servo_count = 0

    def servo_j(self, joint_angles):
        super().servo_j(joint_angles)
        self._servo_count += 1
        if self._servo_count == self.TRIP_AFTER:
            self.emergency_stop()


def test_discard_on_emergency_stop():
    clock, robot, captures = _setup(robot_cls=TrippingRobot)
    plan = _plan(robot, gripper=False)
    assert len(plan) > TrippingRobot.TRIP_AFTER + 1
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)

    assert episode.discarded
    assert episode.discard_reason == "schutzstopp"
    # Es wurde nur bis zum Stopp aufgezeichnet, nichts stillschweigend
    # aufgefuellt
    assert len(episode) == TrippingRobot.TRIP_AFTER
    # Nach dem Not-Halt ist das Servo-Interface deaktiviert
    assert not robot._servo_active

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
from bc.ports import RobotError
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

    # Gesendete Sollwinkel sind die verrauschte Bahn (Nachlauf-Auswertung)
    assert np.allclose(
        episode.arrays["aux.joints_command"], plan.joints_noisy.astype(np.float32)
    )
    # Uebergabepunkt: next.done genau im letzten Schritt
    done = episode.arrays["next.done"][:, 0]
    assert done[-1] and not done[:-1].any()


class SpyRobot(SimRobot):
    """Merkt sich alle servo_j-Aufrufe (Testhelfer)."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.servo_calls = []

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        super().servo_j(joint_angles, velocity, acceleration)
        self.servo_calls.append((joint_angles, velocity, acceleration))


def test_servo_substeps_interpolate_plan_with_velocity():
    # Befund VM 2026-09-15: bei 15 Hz faehrt der Controller jeden Sollwert
    # an und steht dann (Stop-and-go). Jetzt: Zwischenschritte mit
    # SERVO_RATE_HZ, das Ziel des Schritts i genau auf Takt i. servo_j bekommt
    # immer drei Listen (VM 2026-09-09), die Geschwindigkeit passt zur Fahrt.
    clock, robot, captures = _setup(robot_cls=SpyRobot)
    plan = _plan(robot)
    t0 = clock.now()
    EpisodeRecorder(robot, captures, clock).record(plan)

    sub = int(round(config.SERVO_RATE_HZ / config.CONTROL_RATE_HZ))
    assert sub > 1
    calls = robot.servo_calls
    assert len(calls) == 1 + (len(plan) - 1) * sub
    q_all = np.array([c[0] for c in calls])
    # Auf jedem Beobachtungstakt exakt der geplante Sollwert
    assert np.allclose(q_all[::sub], plan.joints_noisy)
    # Dazwischen linear
    i = len(plan) // 2
    mid = q_all[(i - 1) * sub + sub // 2]
    assert np.allclose(
        mid, plan.joints_noisy[i - 1] + (sub // 2) / sub * (plan.joints_noisy[i] - plan.joints_noisy[i - 1])
    )
    # Geschwindigkeit = Zieldifferenz je Takt, nie pauschal None
    period = 1.0 / config.CONTROL_RATE_HZ
    for k in range(1, sub + 1):
        q, qd, qdd = calls[(i - 1) * sub + k]
        assert qd is not None and qdd is not None
        assert np.allclose(qd, (plan.joints_noisy[i] - plan.joints_noisy[i - 1]) / period)
    assert max(np.abs(c[1]).max() for c in calls) > 0.01
    # Senderate stimmt: Episode dauert (N-1) Takte
    assert abs((clock.now() - t0) - (len(plan) - 1) * period) < 2 * period


def test_refuses_to_start_away_from_start_pose():
    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=False)
    robot._joints = robot._joints + 0.1  # Arm steht woanders
    try:
        EpisodeRecorder(robot, captures, clock).record(plan)
        assert False, "RobotError erwartet (nicht an der Startstellung)"
    except RobotError as exc:
        assert "Startstellung" in str(exc)
    assert not robot._servo_active  # Servo wurde gar nicht erst aktiviert


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


_SUBSTEPS = int(round(config.SERVO_RATE_HZ / config.CONTROL_RATE_HZ))


class TrippingRobot(SimRobot):
    """Loest nach N servo-Befehlen den Schutzstopp aus (Testhelfer)."""

    #: Halte-Befehl + 4 volle Fenster, Stopp mitten im fuenften
    TRIP_AFTER = 1 + 4 * _SUBSTEPS + 1

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._servo_count = 0

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        super().servo_j(joint_angles, velocity, acceleration)
        self._servo_count += 1
        if self._servo_count == self.TRIP_AFTER:
            self.emergency_stop()


def test_discard_on_emergency_stop():
    clock, robot, captures = _setup(robot_cls=TrippingRobot)
    plan = _plan(robot, gripper=False)
    assert len(plan) > 6
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)

    assert episode.discarded
    assert episode.discard_reason == "schutzstopp"
    # Es wurde nur bis zum Stopp aufgezeichnet (Takte 0-4), das angebrochene
    # Fenster nicht mehr, nichts stillschweigend aufgefuellt
    assert len(episode) == 5
    # Nach dem Stopp kein weiterer Sollwert
    assert robot._servo_count == TrippingRobot.TRIP_AFTER
    # Nach dem Not-Halt ist das Servo-Interface deaktiviert
    assert not robot._servo_active
    # Abgebrochene Episode: kein Uebergabepunkt markiert
    assert not episode.arrays["next.done"].any()

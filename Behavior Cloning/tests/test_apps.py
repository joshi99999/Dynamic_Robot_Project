"""Apps: Teach-Schleife mit ScriptedTeleop (AP 2.2, hardwarefrei)."""

import sys
from pathlib import Path

import _paths  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))

import numpy as np

import _fixtures
from bc import config
from bc.adapters.teleop_script import ScriptedTeleop, teach_script
from bc.ports import TeleopEvent


def test_teach_loop_saves_waypoints():
    from teach import teach_loop

    robot = _fixtures.make_robot("sim")
    jog = np.array([0.01, 0.0, 0.0])
    teleop = teach_script([[jog], [jog, jog], []])

    waypoints = teach_loop(robot, teleop, clock=robot._clock)
    assert len(waypoints) == 3
    # Jogs verschieben die gespeicherten Posen in +x
    x0 = waypoints[0]["pose_quat"][0]
    x1 = waypoints[1]["pose_quat"][0]
    assert x1 > x0 + 0.015  # zwei weitere 1-cm-Jogs (IK-Toleranz)


def test_camera_modes_and_static_placeholders():
    from bc import config
    from bc.adapters import camera_configs, resolve_camera_mode, start_cameras
    from bc.clock import SimClock

    assert resolve_camera_mode("auto", robot_is_sim=True) == "sim"
    assert resolve_camera_mode("auto", robot_is_sim=False) == "real"
    mixed = {c.name: c.backend for c in camera_configs("wrist-real")}
    assert mixed == {"wrist": "daheng", "scene": "sim"}
    # Webcam als Wrist-Ersatz: gleiche Namen/Reihenfolge, Index ueberschreibbar
    webcam = camera_configs("wrist-uvc", uvc_device=1)
    assert [(c.name, c.backend) for c in webcam] == [("wrist", "uvc"), ("scene", "sim")]
    assert webcam[0].device == 1
    assert camera_configs("wrist-uvc")[0].device == config.WRIST_CAMERA_UVC_STANDIN.device

    captures, cfgs = start_cameras("sim", SimClock())
    try:
        first = [cap.latest().image.copy() for cap in captures]
        later = [cap.latest().image for cap in captures]
        # Platzhalter ohne Zaehler: keine Uhr im Bild (Befund 2026-09-17)
        assert all(np.array_equal(a, b) for a, b in zip(first, later))
    finally:
        for cap in captures:
            cap.stop()
    for mode in ("wrist-real", "wrist-uvc"):
        try:
            start_cameras(mode, SimClock())
            assert False, "ValueError erwartet (echte Kamera braucht Host-Uhr)"
        except ValueError:
            pass


def _infer_setup():
    from bc import capture, config
    from bc.adapters.cam_sim import SimCamera
    from bc.clock import SimClock
    from bc.collision import default_workspace

    clock = SimClock()
    robot = _fixtures.make_robot("sim", clock=clock)
    captures = capture.start_all(
        [SimCamera(cfg, clock=clock) for cfg in config.SIM_CAMERAS], threaded=False
    )
    workspace = default_workspace(table_height_m=float(robot.read_state().tcp_quat[2]) - 0.3)
    return robot, captures, clock, workspace


def test_infer_episode_ends_at_end_pose():
    from infer import END_HOLD_STEPS, run_episode

    from bc.policy import ChunkEnsembler, HoldPolicy

    robot, captures, clock, workspace = _infer_setup()
    end = robot.read_state().joints.copy()
    log = run_episode(robot, captures, ChunkEnsembler(HoldPolicy(), replan_steps=2), clock,
                      max_steps=50, workspace=workspace, end_joints=end,
                      threaded_watchdog=False, progress=None)
    assert log["stop_reason"] == "endstellung"
    assert len(log["tcp"]) == END_HOLD_STEPS
    assert log["pacer_overruns"] == 0

    # Gleiche Stellung, aber Greifer anders als am Ende der Aufzeichnung ->
    # kein Ende (Befund 2026-09-17: Zwischenpunkt == Endpunkt der Sim-Demo)
    log = run_episode(robot, captures, ChunkEnsembler(HoldPolicy(), replan_steps=2), clock,
                      max_steps=12, workspace=workspace, end_joints=end, end_gripper=1.0,
                      threaded_watchdog=False, progress=None)
    assert log["stop_reason"] == "max_steps"
    # ... und nicht vor min_steps
    log = run_episode(robot, captures, ChunkEnsembler(HoldPolicy(), replan_steps=2), clock,
                      max_steps=50, workspace=workspace, end_joints=end, min_steps=10,
                      threaded_watchdog=False, progress=None)
    assert log["stop_reason"] == "endstellung" and len(log["tcp"]) == 10 + END_HOLD_STEPS


def test_infer_refuses_far_target():
    from infer import run_episode

    from bc.policy import ChunkEnsembler, HoldPolicy

    class FarPolicy(HoldPolicy):
        def predict(self, observation):
            chunk = super().predict(observation)
            # Ziel deutlich ueber der Grenze neben der Ist-Stellung
            chunk[:, 1] += config.SERVO_MAX_TARGET_GAP_RAD + 0.15
            return chunk

    robot, captures, clock, workspace = _infer_setup()
    start = robot.read_state().joints.copy()
    log = run_episode(robot, captures, ChunkEnsembler(FarPolicy(), replan_steps=2), clock,
                      max_steps=20, workspace=workspace, threaded_watchdog=False, progress=None)
    assert log["stop_reason"].startswith("abgelehnt")
    assert robot.stop_requested
    assert np.allclose(robot.read_state().joints, start)


def test_infer_hang_delays_but_does_not_jump():
    """Ein haengender servo_j-Aufruf (Anlage: ~210 ms, einmal 1014 ms,
    0,1-0,2 % der Aufrufe) verschiebt nur den Rest der Fahrt: danach geht der
    NAECHSTE Zwischenschritt raus, kein Sprung um die verlorene Zeit
    (Uebergabe 19.10., Abschnitt 7.3). Die Schleife zaehlt Schritte, nicht die Uhr."""
    from infer import run_episode

    from bc.policy import ChunkEnsembler, HoldPolicy

    schritt = 0.01  # rad je 15-Hz-Takt auf Gelenk 2 (0,15 rad/s)

    class FahrPolicy(HoldPolicy):
        def predict(self, observation):
            chunk = super().predict(observation)
            chunk[:, 1] += schritt * np.arange(1, len(chunk) + 1)
            return chunk

    robot, captures, clock, workspace = _infer_setup()
    gesendet = []
    original = robot.servo_j

    def servo_j(q, v=None, a=None):
        gesendet.append(np.asarray(q, dtype=float).copy())
        if len(gesendet) == 40:
            clock.advance(1.014)   # der laengste gemessene Haenger
        return original(q, v, a)

    robot.servo_j = servo_j
    log = run_episode(robot, captures, ChunkEnsembler(FahrPolicy(), replan_steps=1), clock,
                      max_steps=30, workspace=workspace, threaded_watchdog=False, progress=None)
    assert log["stop_reason"] == "max_steps", log["stop_reason"]
    assert log["pacer_overruns"] > 0          # die Uhr hat den Haenger gesehen
    sprung = np.max(np.abs(np.diff(np.asarray(gesendet)[:, 1])))
    # ein 60-Hz-Zwischenschritt (float32 im State: 1e-6 Toleranz)
    assert sprung <= schritt / (config.SERVO_RATE_HZ / config.CONTROL_RATE_HZ) + 1e-6, sprung


def _greif_fahrt(reach_tol_mm, max_steps=25):
    """Inferenz online: Policy faehrt ein Ziel in 0.01-rad-Schritten bis 0.06
    rad weiter (sanft genug fuer den ServoGuard) und greift dort ab der 8.
    Vorhersage; der Arm laeuft deutlich hinterher."""
    from infer import run_episode

    from bc import execution
    from bc.policy import ChunkEnsembler, HoldPolicy

    robot, captures, clock, workspace = _infer_setup()
    robot.ONLINE_GAIN = 0.1
    start = robot.read_state().joints.copy()
    ziel = start.copy()
    ziel[1] += 0.06

    class GreifPolicy(HoldPolicy):
        calls = 0

        def predict(self, observation):
            GreifPolicy.calls += 1
            chunk = super().predict(observation)
            chunk[:, :6] = start + min(1.0, GreifPolicy.calls / 6.0) * (ziel - start)
            chunk[:, 6] = 1.0 if GreifPolicy.calls >= 8 else 0.0
            return chunk

    greifer = []
    original = robot.gripper_command

    def gripper_command(close):
        greifer.append((bool(close), robot.read_state().tcp_quat[:3].copy()))
        return original(close)

    robot.gripper_command = gripper_command
    ex = execution.describe("online", reach_tol_mm=reach_tol_mm)
    log = run_episode(robot, captures, ChunkEnsembler(GreifPolicy(), replan_steps=1), clock,
                      max_steps=max_steps, workspace=workspace, threaded_watchdog=False,
                      progress=None, execution=ex)
    return log, greifer, robot, ex


def test_infer_online_waits_at_the_gripper_change_like_the_recorder():
    from bc import config

    log, greifer, robot, ex = _greif_fahrt(config.ONLINE_REACH_TOL_MM)
    assert log["stop_reason"] == "max_steps", log["stop_reason"]
    assert len(log["reach_waits"]) == 1
    halt = log["reach_waits"][0]
    assert halt["reached"] and halt["start_mm"] > ex["reach_tol_mm"]
    assert abs(log["reach_wait_s"] - halt["wait_s"]) < 1e-12
    # die Wartezeit steht im Protokoll am Takt des Greiferwechsels
    assert np.count_nonzero(log["halt_s"]) == 1
    assert log["halt_s"][halt["step"]] == halt["wait_s"]
    # Greifer schliesst erst am Ziel der Action dieses Takts (wie im Recorder
    # am Haltepunkt des Plans; hier die gemittelte Action der Policy)
    halte_tcp = robot.fk(log["action"][halt["step"]][:6])[:3]
    zu = [p for close, p in greifer if close]
    assert len(zu) == 1
    assert 1e3 * np.linalg.norm(zu[0] - halte_tcp) <= ex["reach_tol_mm"]
    assert log["pacer_overruns"] == 0


def test_infer_online_without_reach_rule_grips_while_still_moving():
    # Aeltere Aufnahmen ohne reach_*: wie bisher, kein Warten
    log, greifer, robot, ex = _greif_fahrt(None)
    assert log["reach_waits"] == [] and log["reach_wait_s"] == 0.0
    assert not np.any(log["halt_s"])
    zu = [p for close, p in greifer if close]
    assert len(zu) == 1
    ziel_tcp = robot.fk(log["action"][-1][:6])[:3]
    assert 1e3 * np.linalg.norm(zu[0] - ziel_tcp) > 1.0


def test_teach_loop_gripper_toggle():
    from teach import teach_loop

    robot = _fixtures.make_robot("sim")
    teleop = ScriptedTeleop(
        [
            TeleopEvent("gripper", True),
            TeleopEvent("save"),
            TeleopEvent("gripper", False),
            TeleopEvent("save"),
            TeleopEvent("quit"),
        ]
    )
    waypoints = teach_loop(robot, teleop, clock=robot._clock)
    assert waypoints[0]["gripper_closed"] is True
    assert waypoints[1]["gripper_closed"] is False

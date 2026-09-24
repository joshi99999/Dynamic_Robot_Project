"""Apps: Teach-Schleife mit ScriptedTeleop (AP 2.2, hardwarefrei)."""

import sys
from pathlib import Path

import _paths  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))

import numpy as np

import _fixtures
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
            chunk[:, 1] += 0.5  # Ziel 0.5 rad neben der Ist-Stellung
            return chunk

    robot, captures, clock, workspace = _infer_setup()
    start = robot.read_state().joints.copy()
    log = run_episode(robot, captures, ChunkEnsembler(FarPolicy(), replan_steps=2), clock,
                      max_steps=20, workspace=workspace, threaded_watchdog=False, progress=None)
    assert log["stop_reason"].startswith("abgelehnt")
    assert robot.stop_requested
    assert np.allclose(robot.read_state().joints, start)


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

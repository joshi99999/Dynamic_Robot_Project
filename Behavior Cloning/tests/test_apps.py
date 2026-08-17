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

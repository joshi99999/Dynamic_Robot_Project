"""Touch-up aus der Bedienoberflaeche (bc/touchup.py, AP 2.2) -- hardwarefrei.

Wunsch Anwender 2026-10-21: Punkt per Klick auf die aktuelle Stellung
setzen. Geprueft werden die Sperren (Bewegung, Werkzeug), die Sicherung vor
dem Schreiben und die Ruecknahme.
"""

import json
import tempfile
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

from bc import config, touchup
from bc.adapters.sim_robot import SimRobot
from bc.clock import SimClock
from bc.ports import RobotError


def _robot():
    clock = SimClock()
    robot = SimRobot(clock=clock).connect()
    return robot, clock


def test_touch_up_sets_point_to_current_joints_and_backs_up():
    robot, clock = _robot()
    alt = robot.read_state().joints + np.array([0.0, 0.0, 0.01, 0.0, 0.0, 0.0])
    robot.store_point("PICK", alt)
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, "NoTool")
    assert not plan["problems"] and plan["exists"]
    assert 1.0 < plan["move_mm"] < config.TOUCHUP_WARN_MM
    with tempfile.TemporaryDirectory() as tmp:
        backup, _record = touchup.apply_touch_up(robot, plan, tmp)
        assert np.allclose(robot.get_point("PICK")[0], robot.read_state().joints)
        data = json.loads(Path(backup).read_text(encoding="utf-8"))
        assert data["existed"] and np.allclose(data["record"]["jointAngles"], alt)
        assert "zurueckgesetzt" in touchup.undo_touch_up(robot, backup)
        assert np.allclose(robot.get_point("PICK")[0], alt)


def test_touch_up_of_missing_point_creates_it_and_undo_deletes_it():
    robot, clock = _robot()
    plan = touchup.plan_touch_up(robot, "APPROACH_01", clock, True, None)
    assert not plan["exists"] and any("neu angelegt" in w for w in plan["warnings"])
    with tempfile.TemporaryDirectory() as tmp:
        backup, _record = touchup.apply_touch_up(robot, plan, tmp)
        assert "APPROACH_01" in robot.point_names()
        touchup.undo_touch_up(robot, backup)
        assert "APPROACH_01" not in robot.point_names()


def test_touch_up_refuses_while_the_arm_moves():
    robot, clock = _robot()
    original = robot.read_state
    calls = []

    def moving():
        state = original()
        calls.append(1)
        state.joints = state.joints + 0.01 * len(calls)
        return state

    robot.read_state = moving
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, None)
    assert any("bewegt sich" in p for p in plan["problems"])
    try:
        touchup.apply_touch_up(robot, plan, ".")
        raise AssertionError("RobotError erwartet")
    except RobotError:
        pass
    assert "PICK" not in robot.point_names()


def test_touch_up_at_the_plant_needs_the_recording_tool():
    robot, clock = _robot()
    for tool in (None, "NoTool", "AnderesTool"):
        plan = touchup.plan_touch_up(robot, "PICK", clock, False, tool)
        assert any("Werkzeug" in p for p in plan["problems"]), tool
    plan = touchup.plan_touch_up(robot, "PICK", clock, False, config.TOUCHUP_TOOL_NAME)
    assert not plan["problems"]
    assert "REALE ANLAGE" in touchup.describe(plan)
    # In der Simulation nur ein Hinweis
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, "NoTool")
    assert not plan["problems"] and any("Simulation" in w for w in plan["warnings"])


def test_touch_up_warns_on_large_change_and_rejects_bad_names():
    robot, clock = _robot()
    robot.store_point("PICK", robot.read_state().joints + np.array([0.4, 0, 0, 0, 0, 0]))
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, None)
    assert any("Grosse Aenderung" in w for w in plan["warnings"])
    assert "BLOCKIERT" not in touchup.describe(plan)
    for name in ("_BC_TOUCHUP_HILFE", "  "):
        assert touchup.plan_touch_up(robot, name, clock, True, None)["problems"]


def test_two_touch_ups_in_the_same_second_keep_both_backups():
    from datetime import datetime

    robot, clock = _robot()
    jetzt = datetime(2026, 10, 21, 9, 41, 31)
    with tempfile.TemporaryDirectory() as tmp:
        plan = touchup.plan_touch_up(robot, "PICK", clock, True, None)
        erste, _ = touchup.apply_touch_up(robot, plan, tmp, now=jetzt)
        plan = touchup.plan_touch_up(robot, "PICK", clock, True, None)
        zweite, _ = touchup.apply_touch_up(robot, plan, tmp, now=jetzt)
        assert erste != zweite
        touchup.undo_touch_up(robot, zweite)
        touchup.undo_touch_up(robot, erste)
        assert "PICK" not in robot.point_names()   # erste hatte ihn angelegt


def test_undo_refuses_when_the_point_was_changed_again():
    robot, clock = _robot()
    robot.store_point("PICK", robot.read_state().joints + 0.01)
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, None)
    with tempfile.TemporaryDirectory() as tmp:
        backup, _record = touchup.apply_touch_up(robot, plan, tmp)
        robot.store_point("PICK", robot.read_state().joints + 0.02)   # am Pendant nachgeteacht
        try:
            touchup.undo_touch_up(robot, backup)
            raise AssertionError("RobotError erwartet")
        except RobotError as exc:
            assert "erneut geaendert" in str(exc)
        touchup.undo_touch_up(robot, backup, force=True)
        assert np.allclose(robot.get_point("PICK")[0], robot.read_state().joints + 0.01)

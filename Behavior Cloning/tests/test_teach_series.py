"""Teach-Serie mit den Knoepfen am Arm (bc/teach_series.py, AP 2.2) -- hardwarefrei.

Wunsch Anwender 2026-10-21: POINT am Arm setzt den Punkt der markierten
Zeile und springt weiter. Geprueft werden das Erkennen des vom Knopf
angelegten Punkts, der Touch-up mit dessen Gelenken und das Aufraeumen.
"""

import tempfile

import _paths  # noqa: F401

import numpy as np

from bc import teach_series, touchup
from bc.adapters.sim_robot import SimRobot
from bc.clock import SimClock


def _robot():
    clock = SimClock()
    robot = SimRobot(clock=clock).connect()
    return robot, clock


def test_watcher_reports_only_points_created_after_the_start():
    robot, _clock = _robot()
    robot.store_point("ALT", robot.read_state().joints)
    watcher = teach_series.NewPointWatcher(robot, pattern=None)
    assert watcher.poll() == []
    robot.store_point("Point_7", robot.read_state().joints)
    assert watcher.poll() == ["Point_7"]
    assert watcher.poll() == []
    assert watcher.pending == ["Point_7"]


def test_watcher_ignores_helpers_and_the_points_of_the_sequence():
    robot, _clock = _robot()
    watcher = teach_series.NewPointWatcher(robot, pattern=None)
    for name in ("_BC_TOUCHUP_HILFE", "APPROACH_01", "Point_1"):
        robot.store_point(name, robot.read_state().joints)
    assert watcher.poll(protected=["APPROACH_01", "PICK"]) == ["Point_1"]


def test_watcher_pattern_keeps_hand_made_points_out():
    robot, _clock = _robot()
    watcher = teach_series.NewPointWatcher(robot, pattern=r"Point_\d+")
    robot.store_point("MEIN_PUNKT", robot.read_state().joints)
    robot.store_point("Point_3", robot.read_state().joints)
    assert watcher.poll() == ["Point_3"]


def test_a_consumed_name_given_out_again_is_seen_again():
    robot, _clock = _robot()
    watcher = teach_series.NewPointWatcher(robot, pattern=None)
    robot.store_point("Point_1", robot.read_state().joints)
    assert watcher.poll() == ["Point_1"]
    assert teach_series.consume_point(robot, "Point_1")
    assert watcher.poll() == []
    robot.store_point("Point_1", robot.read_state().joints)
    assert watcher.poll() == ["Point_1"]
    # ... auch ohne poll zwischen Loeschen und naechstem Druck
    assert teach_series.consume_point(robot, "Point_1")
    watcher.forget("Point_1")
    robot.store_point("Point_1", robot.read_state().joints)
    assert watcher.poll() == ["Point_1"]


def test_button_point_becomes_the_row_point_without_reading_the_arm():
    """Die Gelenke kommen aus dem Knopf-Punkt -- der Arm darf sich schon
    wieder bewegen, die Stillstandspruefung entfaellt."""
    robot, clock = _robot()
    pressed = robot.read_state().joints + np.array([0.0, 0.02, 0.0, 0.0, 0.0, 0.0])
    robot.store_point("Point_1", pressed)

    def moving():
        raise AssertionError("read_state darf mit joints=... nicht gelesen werden")

    robot.read_state = moving
    plan = touchup.plan_touch_up(robot, "APPROACH_01", clock, True, None,
                                 joints=teach_series.joints_of(robot, "Point_1"))
    assert not plan["problems"] and not teach_series.needs_confirm(plan)
    with tempfile.TemporaryDirectory() as tmp:
        touchup.apply_touch_up(robot, plan, tmp)
    assert teach_series.consume_point(robot, "Point_1")
    assert "Point_1" not in robot.point_names()
    assert np.allclose(robot.get_point("APPROACH_01")[0], pressed)
    assert teach_series.summary(plan) == "APPROACH_01 neu angelegt"


def test_large_change_stops_the_series_for_a_question():
    robot, clock = _robot()
    here = robot.read_state().joints
    robot.store_point("PICK", here + np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, None, joints=here)
    assert plan["large_change"] and teach_series.needs_confirm(plan)


def test_small_correction_runs_through():
    """Zweiter PRE_GRASP nach "Z hoch" ueber PICK: wenige Zentimeter."""
    robot, clock = _robot()
    here = robot.read_state().joints
    robot.store_point("PRE_GRASP", here + np.array([0.0, 0.0, 0.01, 0.0, 0.0, 0.0]))
    plan = touchup.plan_touch_up(robot, "PRE_GRASP", clock, True, None, joints=here)
    assert plan["exists"] and not teach_series.needs_confirm(plan)


def test_next_row_ends_after_the_last():
    assert teach_series.next_row(0, 6) == 1
    assert teach_series.next_row(5, 6) == -1


def test_default_pattern_takes_only_the_point_button_names():
    """Anlage 21.10.: "Save Point" legte GP35 an."""
    robot, _clock = _robot()
    watcher = teach_series.NewPointWatcher(robot)
    for name in ("GP35", "GP_ALT", "MEIN_PUNKT"):
        robot.store_point(name, robot.read_state().joints)
    assert watcher.poll() == ["GP35"]


def test_undo_all_brings_a_point_set_twice_back_to_before_the_series():
    """PRE_GRASP steht zweimal im Ablauf -- rueckwaerts zuruecknehmen."""
    robot, clock = _robot()
    here = robot.read_state().joints
    before = here + np.array([0.0, 0.0, 0.02, 0.0, 0.0, 0.0])
    robot.store_point("PRE_GRASP", before)
    entries = []
    with tempfile.TemporaryDirectory() as tmp:
        for name, dz in (("PRE_GRASP", 0.005), ("PICK", 0.0), ("PRE_GRASP", 0.01)):
            q = here + np.array([0.0, 0.0, dz, 0.0, 0.0, 0.0])
            plan = touchup.plan_touch_up(robot, name, clock, True, None, joints=q)
            backup, _record = touchup.apply_touch_up(robot, plan, tmp)
            entries.append(teach_series.entry(plan, backup))
        assert "3 Punkt(e)" in teach_series.review_text(entries)
        done, error = teach_series.undo_all(robot, entries)
    assert error is None and len(done) == 3
    assert np.allclose(robot.get_point("PRE_GRASP")[0], before)
    assert "PICK" not in robot.point_names()


def test_large_change_is_flagged_in_the_review():
    robot, clock = _robot()
    here = robot.read_state().joints
    robot.store_point("PICK", here + np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0]))
    plan = touchup.plan_touch_up(robot, "PICK", clock, True, None, joints=here)
    assert "⚠ PICK" in teach_series.review_text([teach_series.entry(plan, "x.json")])

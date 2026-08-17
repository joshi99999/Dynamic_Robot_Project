"""Watchdog und Geofencing (AP 4.2)."""

import _paths  # noqa: F401

import _fixtures
from bc.collision import default_workspace
from bc.safety import Watchdog, geofence_check


def test_watchdog_passes_when_free():
    robot = _fixtures.make_robot("sim")
    joints = robot.read_state().joints
    tool_z = float(robot.link_positions(joints)["tool"][2])
    workspace = default_workspace(table_height_m=tool_z - 0.5)

    dog = Watchdog(robot, [geofence_check(robot, workspace)], robot._clock)
    assert dog.run_once() is None
    assert not robot.stop_requested


def test_watchdog_trips_on_violation():
    robot = _fixtures.make_robot("sim")
    joints = robot.read_state().joints
    tool_z = float(robot.link_positions(joints)["tool"][2])
    # Tisch auf Werkzeughoehe -> sofortige Verletzung
    workspace = default_workspace(table_height_m=tool_z + 0.01)

    dog = Watchdog(robot, [geofence_check(robot, workspace)], robot._clock)
    violation = dog.run_once()
    assert violation
    assert robot.stop_requested
    assert dog.tripped == violation
    # Not-Halt hat das Servo-Interface deaktiviert
    assert not robot._servo_active


def test_broken_check_counts_as_violation():
    # Ein kaputter Waechter darf NIE "gruen" bedeuten
    robot = _fixtures.make_robot("sim")

    def broken():
        raise RuntimeError("Sensor weg")

    dog = Watchdog(robot, [broken], robot._clock)
    violation = dog.run_once()
    assert violation and "Sensor weg" in str(violation)
    assert robot.stop_requested

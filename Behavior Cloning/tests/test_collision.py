"""Kollisionsmodell: Quader, Geofence, Pfadpruefung (AP 2.4 / 4.2)."""

import _paths  # noqa: F401

import numpy as np

import _fixtures
from bc import collision


def test_box_distances():
    box = collision.Box.from_bounds("Tisch", lo=(-1, -1, -0.5), hi=(1, 1, 0.0))
    assert abs(box.distance_to_point([0, 0, 0.30]) - 0.30) < 1e-9
    assert box.distance_to_point([0, 0, -0.25]) < 0
    assert box.clearance([0, 0, 0.03], radius=0.06) < 0
    assert box.clearance([0, 0, 0.30], radius=0.06) > 0


def test_geofence_inverted_logic():
    fence = collision.Box.from_bounds(
        "Arbeitsraum", lo=(-0.5, -0.5, 0.0), hi=(0.5, 0.5, 0.8), solid=False
    )
    assert fence.clearance([0.0, 0.0, 0.4], radius=0.05) > 0
    assert fence.clearance([0.9, 0.0, 0.4], radius=0.05) < 0
    assert fence.clearance([0.48, 0.0, 0.4], radius=0.05) < 0  # Huellkugel


def test_check_positions_named_points():
    box = collision.Box.from_bounds("Tisch", lo=(-1, -1, -0.5), hi=(1, 1, 0.0))
    model = collision.CollisionModel(
        boxes=[box], arm_points=[collision.ArmPoint("tool", 0.05)], margin=0.02
    )
    assert model.check_positions({"tool": np.array([0, 0, 0.5])}) == []
    violations = model.check_positions({"tool": np.array([0, 0, 0.01])})
    assert len(violations) == 1 and violations[0].box == "Tisch"

    # Fehlender Stuetzpunkt darf NIE stillschweigend als "frei" gelten
    try:
        model.check_positions({"wrist": np.array([0, 0, 0.5])})
        assert False, "KeyError erwartet"
    except KeyError:
        pass


def test_check_joints_with_sim_robot():
    robot = _fixtures.make_robot("sim")
    joints = robot.read_state().joints
    tool_z = float(robot.link_positions(joints)["tool"][2])

    # Tisch weit unterhalb des Werkzeugs -> frei
    free_model = collision.default_workspace(table_height_m=tool_z - 0.5)
    assert free_model.check_joints(robot, joints) == []

    # Tisch direkt auf Werkzeughoehe -> Verletzung
    hit_model = collision.default_workspace(table_height_m=tool_z + 0.01)
    assert hit_model.check_joints(robot, joints)


def test_check_path_stops_at_first():
    robot = _fixtures.make_robot("sim")
    joints = robot.read_state().joints
    tool_z = float(robot.link_positions(joints)["tool"][2])
    model = collision.default_workspace(table_height_m=tool_z + 0.01)

    path = np.tile(joints, (5, 1))
    found = model.check_path(robot, path, stop_at_first=True)
    assert found and found[0].index == 0

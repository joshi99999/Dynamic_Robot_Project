"""Ablaufdatei: Validierung, Aufloesung gegen Punkte, Ende-zu-Ende (AP 2.2)."""

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.cam_sim import SimCamera
from bc.adapters.sim_robot import SimRobot
from bc.capture import DirectCapture
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.kinematics import Kinematics
from bc.noise import generate_plan
from bc.recorder import EpisodeRecorder
from bc.sequence import SequenceError, parse_sequence, resolve_sequence
from bc.trajectory import build_ideal_trajectory

#: Ablauf wie mit dem Anwender festgelegt (2026-09-14): BC endet an
#: PRE_PLACE, das Ablegen uebernimmt das Hauptprogramm.
PICK_SEQUENCE = {
    "name": "pick_to_station",
    "sequence": [
        {"point": "CLEAR_FOV", "motion": "ptp"},
        {"point": "APPROACH_01", "motion": "ptp"},
        {"point": "APPROACH_02", "motion": "ptp", "optional": True},
        {"point": "PRE_GRASP", "motion": "ptp"},
        {"point": "PICK", "motion": "lin", "approach": True, "gripper": "close"},
        {"point": "PRE_GRASP", "motion": "lin"},
        {"point": "PRE_PLACE", "motion": "ptp"},
    ],
}


def _sim_points(robot):
    """Punkte-Datenbank fuer den SimRobot: kleine, sichere Variationen um
    dessen Home-Stellung (die URDF-Geometrie ist nicht die der Anlage)."""
    home = robot.read_state().joints
    kin = Kinematics(robot)
    home_pose = robot.fk(home)

    def at(dx, dy, dz):
        pose = home_pose.copy()
        pose[:3] += (dx, dy, dz)
        return kin.ik(pose, home)

    return {
        "CLEAR_FOV": home,
        "APPROACH_01": at(0.06, 0.03, 0.0),
        "PRE_GRASP": at(0.08, 0.05, -0.02),
        "PICK": at(0.08, 0.05, -0.07),
        "PRE_PLACE": at(-0.05, -0.04, 0.0),
    }


def _robot(clock=None):
    clock = clock or SimClock()
    probe = SimRobot(clock=clock).connect()
    return SimRobot(clock=clock, points=_sim_points(probe)).connect()


def test_parse_rejects_invalid_files():
    bad_cases = [
        {},
        {"sequence": [{"point": "A"}]},  # nur ein Schritt
        {"sequence": [{"point": "A"}, {"point": "B", "motion": "circ"}]},
        {"sequence": [{"point": "A"}, {"point": "B", "gripper": "halb"}]},
        {"sequence": [{"point": "A"}, {"point": "B", "speed": 1}]},  # Tippfehler-Schutz
        {"sequence": [{"point": "A", "optional": True}, {"point": "B"}]},
        {"sequence": [{"point": "A"}, {"point": "B", "optional": True}]},
    ]
    for data in bad_cases:
        try:
            parse_sequence(data)
            assert False, "SequenceError erwartet fuer %r" % (data,)
        except SequenceError:
            pass


def test_resolve_skips_optional_and_propagates_gripper():
    robot = _robot()
    resolved = resolve_sequence(parse_sequence(PICK_SEQUENCE), robot)

    names = [wp.name for wp in resolved.waypoints]
    assert names == ["CLEAR_FOV", "APPROACH_01", "PRE_GRASP", "PICK", "PRE_GRASP", "PRE_PLACE"]
    assert resolved.skipped == ["APPROACH_02"]
    # Greifer zu ab PICK und bis zum Ende (Uebergabe mit Objekt)
    assert [wp.gripper_closed for wp in resolved.waypoints] == [
        False, False, False, True, True, True,
    ]
    assert [wp.motion for wp in resolved.waypoints][1:] == [
        "ptp", "ptp", "lin", "lin", "ptp",
    ]
    assert np.allclose(resolved.start_joints, robot.get_point("CLEAR_FOV")[0])
    assert set(resolved.points) == {"CLEAR_FOV", "APPROACH_01", "PRE_GRASP", "PICK", "PRE_PLACE"}


def test_resolve_reports_all_missing_required_points():
    robot = SimRobot(clock=SimClock(), points={"CLEAR_FOV": np.zeros(6)}).connect()
    try:
        resolve_sequence(parse_sequence(PICK_SEQUENCE), robot)
        assert False, "SequenceError erwartet"
    except SequenceError as exc:
        for name in ("APPROACH_01", "PRE_GRASP", "PICK", "PRE_PLACE"):
            assert name in str(exc)
        assert "APPROACH_02" not in str(exc).split("(vorhanden")[0]


def test_touch_up_takes_effect_in_next_resolution():
    # Koordinaten werden je Episode frisch abgefragt -- ein Touch-up am
    # Pendant wirkt ohne Export (Anforderung 2026-09-14).
    robot = _robot()
    seq = parse_sequence(PICK_SEQUENCE)
    before = resolve_sequence(seq, robot)
    robot._points["PICK"] = robot._points["PICK"] + np.array([0.02, 0, 0, 0, 0, 0])
    after = resolve_sequence(seq, robot)
    assert not np.allclose(before.points["PICK"]["joints"], after.points["PICK"]["joints"])


def test_sequence_end_to_end_with_ptp_and_handoff():
    clock = SimClock()
    robot = _robot(clock)
    resolved = resolve_sequence(parse_sequence(PICK_SEQUENCE), robot)
    ideal = build_ideal_trajectory(resolved.waypoints, fk=robot.fk)

    # Vorher woanders stehend -> an den Start fahren (ausserhalb der Aufnahme)
    robot.move_to_joints(resolved.start_joints + 0.05)
    robot.move_to_joints(resolved.start_joints)

    kin = Kinematics(robot)
    table_z = min(wp.pose_quat[2] for wp in resolved.waypoints) - 0.15
    plan = generate_plan(
        kin,
        default_workspace(table_height_m=table_z),
        ideal,
        resolved.start_joints,
        rng=np.random.default_rng(1),
    )
    # Start und Uebergabe exakt an den geteachten Stellungen (Rauschen 0)
    assert np.allclose(plan.joints_noisy[0], resolved.start_joints, atol=1e-3)
    assert np.allclose(plan.joints_noisy[-1], robot.get_point("PRE_PLACE")[0], atol=1e-3)

    cams = [DirectCapture(SimCamera(cfg, clock=clock)).start() for cfg in config.SIM_CAMERAS]
    episode = EpisodeRecorder(robot, cams, clock).record(plan)
    assert not episode.discarded, episode.discard_reason
    episode.validate()
    assert episode.arrays["next.done"][-1, 0]
    # Greifer im State ab dem Greifen geschlossen, bis zur Uebergabe
    assert episode.arrays["observation.state"][-1, 13] == config.GRIPPER_CLOSED

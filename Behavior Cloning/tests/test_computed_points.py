"""Berechnete Punkte und Wechsel in den Teach-Modus (AP 2.2, Anwender 2026-10-26).

Je Objektlage wird nur PICK geteacht; PRE_GRASP und der Anflugpunkt liegen
fest darueber. Nach dem Lauf kann die Steuerung in Teach schalten, damit
der naechste PICK ohne Anmeldung am Pendant geteacht werden kann.
"""

import contextlib
import io
import json
import sys
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.neura import NeuraRobot
from bc.adapters.sim_robot import SimRobot
from bc.clock import SimClock
from bc.gui import recording
from bc.ports import MotionRefused
from bc.sequence import (
    SequenceError,
    approach_problems,
    load_sequence,
    parse_sequence,
    resolve_sequence,
)
from bc.teach_series import next_row

from test_neura_adapter import FakeNeuraClient
from test_put_back import _sim_robot_keeping_last_state
from test_sequence import _sim_points

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "apps"))

COMPUTED = {
    "name": "berechnet",
    "sequence": [
        {"point": "CLEAR_FOV", "motion": "ptp"},
        {"point": "ANFLUG", "motion": "ptp", "above": "PRE_GRASP", "dz": 0.10, "blend": 0.05},
        {"point": "PRE_GRASP", "motion": "lin", "above": "PICK", "dz": 0.05, "blend": 0.01},
        {"point": "PICK", "motion": "lin", "approach": True, "gripper": "close"},
        {"point": "PRE_GRASP", "motion": "lin", "blend": 0.04},
        {"point": "PRE_PLACE", "motion": "ptp"},
    ],
}


def _only_taught_points():
    points = _sim_points(SimRobot(clock=SimClock()).connect())
    del points["PRE_GRASP"], points["APPROACH_01"]
    return points


def test_only_pick_needs_teaching():
    seq = parse_sequence(COMPUTED)
    assert seq.computed == {"ANFLUG": ("PRE_GRASP", 0.10), "PRE_GRASP": ("PICK", 0.05)}
    assert seq.taught_names == ["CLEAR_FOV", "PICK", "PRE_PLACE"]
    # die mitgelieferte Ablaufdatei ist genau so gebaut
    shipped = load_sequence(ROOT / "sequences" / "pick_to_station.json")
    assert set(shipped.taught_names) == {"CLEAR_FOV", "PICK", "PRE_PLACE"}
    assert shipped.computed == {"ANFLUG": ("PRE_PICK", 0.10), "PRE_PICK": ("PICK", 0.05)}


def test_computed_points_sit_straight_above():
    robot = SimRobot(clock=SimClock(), points=_only_taught_points()).connect()
    resolved = resolve_sequence(parse_sequence(COMPUTED), robot)
    pose = {wp.name: wp.pose_quat for wp in resolved.waypoints}
    assert np.allclose(pose["PRE_GRASP"][:2], pose["PICK"][:2])
    assert np.isclose(pose["PRE_GRASP"][2] - pose["PICK"][2], 0.05)
    assert np.isclose(pose["ANFLUG"][2] - pose["PRE_GRASP"][2], 0.10)
    assert np.allclose(pose["ANFLUG"][3:], pose["PICK"][3:])  # gleiche Orientierung
    # Gelenke per IK: reproduzieren die Pose
    for wp in resolved.waypoints:
        assert np.allclose(robot.fk(wp.joints)[:3], wp.pose_quat[:3], atol=1e-3), wp.name
    assert approach_problems(resolved.waypoints) == []
    assert resolved.points["PRE_GRASP"]["computed"] == {"above": "PICK", "dz": 0.05}
    assert "computed" not in resolved.points["PICK"]


def test_computed_wins_over_a_stale_database_point():
    points = _sim_points(SimRobot(clock=SimClock()).connect())  # PRE_GRASP noch geteacht
    robot = SimRobot(clock=SimClock(), points=points).connect()
    resolved = resolve_sequence(parse_sequence(COMPUTED), robot)
    pick = [wp for wp in resolved.waypoints if wp.name == "PICK"][0]
    pre = [wp for wp in resolved.waypoints if wp.name == "PRE_GRASP"][0]
    assert np.isclose(pre.pose_quat[2] - pick.pose_quat[2], 0.05)


def test_invalid_computed_points_are_rejected():
    def steps(*middle):
        return {"sequence": [{"point": "A"}] + list(middle) + [{"point": "Z"}]}

    for data in (
            steps({"point": "B", "above": "A"}),                       # dz fehlt
            steps({"point": "B", "above": "A", "dz": 0}),              # kein Abstand
            steps({"point": "B", "above": "A", "dz": 2.0}),            # zu weit
            steps({"point": "B", "above": "B", "dz": 0.1}),            # ueber sich selbst
            steps({"point": "B", "above": "A", "dz": 0.1, "optional": True}),
            steps({"point": "B", "above": "A", "dz": 0.1},
                  {"point": "B", "above": "A", "dz": 0.2}),            # widerspruechlich
            steps({"point": "B", "above": "C", "dz": 0.1},
                  {"point": "C", "above": "B", "dz": 0.1})):           # im Kreis
        try:
            parse_sequence(data)
            assert False, "SequenceError erwartet fuer %r" % (data,)
        except SequenceError:
            pass
    # fehlt der Bezugspunkt in der Datenbank, heisst die Meldung ihn
    robot = SimRobot(clock=SimClock(), points={"A": np.zeros(6), "Z": np.zeros(6)}).connect()
    try:
        resolve_sequence(parse_sequence(steps({"point": "B", "above": "PICK", "dz": 0.1})), robot)
        assert False, "SequenceError erwartet"
    except SequenceError as exc:
        assert "PICK" in str(exc)


def test_unreachable_computed_point_is_named():
    robot = SimRobot(clock=SimClock(), points=_only_taught_points()).connect()
    original = robot.ik

    def failing(pose, ref):
        if pose[2] > 0.6:
            raise RuntimeError("ausserhalb")
        return original(pose, ref)

    robot.ik = failing
    data = json.loads(json.dumps(COMPUTED))
    data["sequence"][1]["dz"] = 0.9
    try:
        resolve_sequence(parse_sequence(data), robot)
        assert False, "SequenceError erwartet"
    except SequenceError as exc:
        assert "ANFLUG" in str(exc) and "nicht erreichbar" in str(exc)


def test_gui_round_trip_keeps_computed_points(tmp_path):
    path = tmp_path / "ablauf.json"
    path.write_text(json.dumps(COMPUTED), encoding="utf-8")
    name, start, steps = recording.read_steps(path)
    assert steps[1]["above"] == "PRE_GRASP" and steps[1]["dz"] == 0.10
    assert steps[0]["above"] is None
    recording.write_steps(path, name, start, steps)
    again = json.loads(path.read_text(encoding="utf-8"))
    assert again["sequence"][2] == COMPUTED["sequence"][2]
    assert "above" not in again["sequence"][0]
    described = recording.describe_sequence(path)
    assert "PRE_GRASP = PICK + 50 mm" in described["computed"]
    assert "berechnet" in recording.sequence_summary(described)


def test_teach_series_skips_computed_rows():
    # Zeilen: CLEAR_FOV, ANFLUG*, PRE_GRASP*, PICK, PRE_GRASP*, PRE_PLACE
    computed = {1, 2, 4}
    assert next_row(0, 6, computed) == 3
    assert next_row(3, 6, computed) == 5
    assert next_row(5, 6, computed) == -1
    assert next_row(0, 6) == 1  # ohne Angabe wie bisher


def test_record_with_computed_points_end_to_end(tmp_path):
    import record

    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(COMPUTED), encoding="utf-8")
    points = _only_taught_points()
    opened = []
    saved = sys.argv, record.open_robot
    sys.argv = ["record.py", "--sim", "--sequence", str(seq), "--episodes", "2",
                "--seed", "3", "--out", str(tmp_path / "data")]
    record.open_robot = lambda kind, clock=None, **kw: _sim_robot_keeping_last_state(
        clock, points, opened)
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log):
            record.main()
    finally:
        sys.argv, record.open_robot = saved
    lines = [z for z in log.getvalue().splitlines() if z.startswith("Episode ") and "-> ep_" in z]
    assert len(lines) == 2 and all(z.endswith("ok") for z in lines), log.getvalue()[-1500:]
    meta = json.loads((tmp_path / "data" / "ep_00000" / "meta.json").read_text())
    assert meta["metadata"]["points"]["ANFLUG"]["computed"]["dz"] == 0.10


# -- Teach-Modus nach dem Lauf ------------------------------------------------

class _TeachClient(FakeNeuraClient):
    def __init__(self, teach=False, **kw):
        super().__init__(**kw)
        self.teach = teach

    def is_robot_in_teach_mode(self):
        return self.teach

    def switch_to_teach_mode(self):
        self._log("switch_to_teach_mode")
        self.teach = True
        return True

    def switch_to_automatic_mode(self):
        self._log("switch_to_automatic_mode")
        self.teach = False
        return True


def test_adapter_switches_to_teach_and_back():
    client = _TeachClient(teach=True)
    robot = NeuraRobot(robot=client).connect(power_on=True, ensure_automatic=True)
    assert client.teach is False  # Aufnahme-Start: Automatik
    assert robot.switch_to_teach() is True
    names = [c[0] for c in client.calls]
    assert names.index("stop") < names.index("switch_to_teach_mode")  # erst anhalten


def test_teach_switch_needs_motion_release():
    client = _TeachClient(in_simulation=False)
    robot = NeuraRobot(robot=client).connect()
    try:
        robot.switch_to_teach()
        assert False, "MotionRefused erwartet"
    except MotionRefused:
        pass
    assert "switch_to_teach_mode" not in [c[0] for c in client.calls]


def test_record_helper_reports_without_crashing():
    import record

    class Refusing(object):
        def switch_to_teach(self):
            raise RuntimeError("Controller bietet die Funktion nicht an")

    for robot, expected in ((Refusing(), "am Pendant umschalten"),
                            (type("T", (), {"switch_to_teach": lambda self: True})(),
                             "Teach-Modus."),
                            (object(), "")):  # SimRobot: nichts zu tun
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            record.switch_to_teach(robot)
        assert expected in log.getvalue()


def test_gui_teach_flag_only_for_a_controller():
    assert "--nach-ende-teach" in recording.record_argv(
        "data_vm/x", robot_kind="neura", teach_after=True)
    assert "--nach-ende-teach" not in recording.record_argv(
        "data_vm/x", robot_kind="neura", teach_after=False)
    assert "--nach-ende-teach" not in recording.record_argv(
        "data_sim/x", robot_kind="sim", teach_after=True)
    assert config.COMPUTED_POINT_MAX_JUMP_RAD > 0

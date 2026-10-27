"""Tischreferenz aus geteachten Punkten (bc/table.py, Anwender 2026-10-26)."""

import contextlib
import io
import json
import sys
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

from bc import config, geometry
from bc.adapters.sim_robot import SimRobot
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.gui import points as points_module
from bc.gui import recording
from bc.kinematics import Kinematics
from bc.sequence import SequenceError, parse_sequence, resolve_sequence
from bc.table import (
    TableSurface,
    align_pose,
    clearance,
    fit_table,
    table_problems,
    tilt_deg,
)
from bc.trajectory import Waypoint

from test_put_back import _sim_robot_keeping_last_state
from test_sequence import _sim_points

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "apps"))
sys.path.insert(0, str(ROOT / "tools"))

DOWN = geometry.quat_from_axis_angle([1.0, 0.0, 0.0], np.pi)  # Werkzeug-z nach unten


def _pose(x, y, z, quat=DOWN):
    return np.r_[x, y, z, quat]


def test_plane_from_points_one_mm_above():
    points = [_pose(0.3, -0.1, 0.011), _pose(0.5, -0.1, 0.011),
              _pose(0.3, 0.1, 0.011), _pose(0.5, 0.1, 0.011)]
    plane = fit_table(points)
    assert np.isclose(plane.height_at(0.4, 0.0), 0.010)   # 1 mm unter den Punkten
    assert plane.tilt_deg < 1e-6 and plane.residual_m < 1e-9 and plane.count == 4
    # schraeger Tisch: 2 mm auf 20 cm
    tilted = fit_table([_pose(0.3, 0, 0.011), _pose(0.5, 0, 0.013), _pose(0.3, 0.2, 0.011)])
    assert np.isclose(tilted.height_at(0.5, 0.2), 0.012)
    assert 0.5 < tilted.tilt_deg < 0.6
    # ein Punkt: waagerecht; keiner: unbekannt
    assert fit_table([_pose(0.3, 0, 0.02)]).height_at(1, 1) == 0.019
    assert fit_table([]) is None


def test_points_too_close_to_the_table_are_named():
    plane = fit_table([_pose(0.3, 0, 0.011), _pose(0.5, 0, 0.011), _pose(0.4, 0.2, 0.011)])
    wps = [Waypoint(_pose(0.4, 0.0, 0.018), name="PICK"),        # 8 mm drueber
           Waypoint(_pose(0.4, 0.0, 0.011), name="TIEF"),        # 1 mm
           Waypoint(_pose(0.4, 0.0, 0.005), name="DRUNTER")]
    problems = table_problems(wps, plane)
    assert len(problems) == 2
    assert "TIEF liegt 1.0 mm ueber" in problems[0] and "UNTER" in problems[1]
    # verbogene Ebene: ein Punkt 5 mm daneben
    bad = fit_table([_pose(0.3, 0, 0.011), _pose(0.5, 0, 0.011), _pose(0.3, 0.2, 0.011),
                     _pose(0.5, 0.2, 0.031)])
    assert any("nicht in einer Ebene" in p for p in table_problems([], bad))
    assert table_problems(wps, None) == []


def test_align_keeps_yaw_and_sets_height():
    plane = fit_table([_pose(0.3, 0, 0.011), _pose(0.5, 0, 0.011), _pose(0.4, 0.2, 0.011)])
    yaw = geometry.quat_from_axis_angle([0.0, 0.0, 1.0], np.radians(115))
    tilt = geometry.quat_from_axis_angle([1.0, 0.0, 0.0], np.radians(1.2))
    taught = _pose(0.42, 0.05, 0.020, geometry.quat_multiply(yaw, geometry.quat_multiply(DOWN, tilt)))
    assert 1.1 < tilt_deg(taught, plane) < 1.3
    aligned = align_pose(taught, plane, 0.008)
    assert tilt_deg(aligned, plane) < 1e-6
    assert np.isclose(clearance(plane, aligned), 0.008)
    assert np.allclose(aligned[:2], taught[:2])
    # Drehung um die Hochachse bleibt (bis auf die Neigung)
    x_old = geometry.quat_to_matrix(taught[3:])[:, 0]
    x_new = geometry.quat_to_matrix(aligned[3:])[:, 0]
    assert np.degrees(np.arccos(np.clip(np.dot(x_old[:2] / np.linalg.norm(x_old[:2]),
                                               x_new[:2] / np.linalg.norm(x_new[:2])), -1, 1))) < 0.1
    # ohne Tisch: nur senkrecht, Hoehe bleibt
    level = align_pose(taught)
    assert tilt_deg(level) < 1e-6 and np.isclose(level[2], taught[2])


def test_table_surface_checks_only_the_tool_tip():
    plane = fit_table([_pose(0.3, 0, 0.011)])
    surface = TableSurface(plane)
    assert surface.clearance_for("wrist", [0.4, 0, 0.0], 0.08) is None
    assert np.isclose(surface.clearance_for("tool", [0.4, 0, 0.018], 0.06), 0.008)
    model = default_workspace(table_height_m=-0.5)
    model.add_box(surface)
    ok = model.check_positions({"tool": [0.4, 0, 0.016], "wrist": [0.4, 0, 0.2],
                                "elbow": [0.2, 0, 0.4]})
    low = model.check_positions({"tool": [0.4, 0, 0.012], "wrist": [0.4, 0, 0.2],
                                 "elbow": [0.2, 0, 0.4]})
    assert ok == [] and len(low) == 1 and low[0].box == "Tischebene"


def _sim_table():
    """SimRobot-Punkte plus vier Tischpunkte 1 mm ueber einem Tisch 8 mm unter PICK."""
    probe = SimRobot(clock=SimClock()).connect()
    points = _sim_points(probe)
    kin = Kinematics(probe)
    pick = probe.fk(points["PICK"])
    for k, (dx, dy) in enumerate(((-0.03, -0.03), (0.03, -0.03), (-0.03, 0.03), (0.03, 0.03))):
        pose = pick.copy()
        pose[0] += dx
        pose[1] += dy
        pose[2] -= 0.007          # Tisch 8 mm unter PICK, Punkt 1 mm darueber
        points["ID%d" % k] = kin.ik(pose, points["PICK"])
    return points


SEQ = {
    "name": "mit_tisch",
    "table": {"points": ["ID0", "ID1", "ID2", "ID3"], "pick_height": 0.008},
    "sequence": [
        {"point": "CLEAR_FOV", "motion": "ptp"},
        {"point": "PRE_PICK", "motion": "ptp", "above": "PICK", "dz": 0.05, "blend": 0.01},
        {"point": "PICK", "motion": "lin", "approach": True, "gripper": "close"},
        {"point": "PRE_PICK", "motion": "lin"},
        {"point": "PRE_PLACE", "motion": "ptp"},
    ],
}


def test_sequence_reads_and_resolves_the_table():
    seq = parse_sequence(SEQ)
    assert seq.table_points == ["ID0", "ID1", "ID2", "ID3"]
    assert seq.pick_height == 0.008
    robot = SimRobot(clock=SimClock(), points=_sim_table()).connect()
    resolved = resolve_sequence(seq, robot)
    assert resolved.table is not None and resolved.table.count == 4
    pick = resolved.points["PICK"]["pose_quat"]
    assert abs(1e3 * clearance(resolved.table, pick) - 8.0) < 0.5
    # fehlende Tischpunkte: genannt, nicht verlangt
    points = _sim_table()
    del points["ID3"]
    partial = resolve_sequence(seq, SimRobot(clock=SimClock(), points=points).connect())
    assert partial.table_missing == ["ID3"] and partial.table.count == 3
    for bad in ({"points": "ID0"}, {"points": ["X"], "pick_height": 0.5},
                {"punkte": []}, {"points": ["PICK"]}):
        try:
            parse_sequence(dict(SEQ, table=bad))
            assert False, "SequenceError erwartet fuer %r" % (bad,)
        except SequenceError:
            pass


def test_gui_keeps_the_table_when_saving(tmp_path):
    path = tmp_path / "ablauf.json"
    path.write_text(json.dumps(SEQ), encoding="utf-8")
    name, start, steps = recording.read_steps(path)
    table = recording.read_table(path)
    recording.write_steps(path, name, start, steps, table)
    assert json.loads(path.read_text(encoding="utf-8"))["table"] == SEQ["table"]
    rows = points_module.point_rows(steps, table_points=table["points"])
    assert [r["kind"] for r in rows[-4:]] == [points_module.TABLE] * 4
    # die mitgelieferte Ablaufdatei hat den Abschnitt
    shipped = json.loads((ROOT / "sequences" / "pick_to_station.json").read_text(encoding="utf-8"))
    assert shipped["table"]["points"] == ["ID0", "ID1", "ID2", "ID3"]


def _record(tmp_path, points):
    import record

    tmp_path.mkdir(parents=True, exist_ok=True)
    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(SEQ), encoding="utf-8")
    opened = []
    saved = sys.argv, record.open_robot
    sys.argv = ["record.py", "--sim", "--sequence", str(seq), "--episodes", "1",
                "--seed", "3", "--out", str(tmp_path / "data")]
    record.open_robot = lambda kind, clock=None, **kw: _sim_robot_keeping_last_state(
        clock, points, opened)
    log = io.StringIO()
    error = None
    try:
        with contextlib.redirect_stdout(log):
            record.main()
    except SystemExit as exc:
        error = exc
    finally:
        sys.argv, record.open_robot = saved
    return error, log.getvalue()


def test_record_uses_the_table_and_refuses_points_below_it(tmp_path):
    error, log = _record(tmp_path, _sim_table())
    assert error is None, log[-1500:]
    assert "Tisch aus 4 Punkt(en)" in log and "PICK: 8." in log
    meta = json.loads((tmp_path / "data" / "ep_00000" / "meta.json").read_text())["metadata"]
    assert abs(meta["pick_over_table_m"] - 0.008) < 5e-4
    assert meta["table"]["count"] == 4 and len(meta["table_points"]) == 4
    # Objektlage automatisch (Anwender 2026-10-26)
    assert np.allclose(meta["object_xy_m"], meta["points"]["PICK"]["pose_quat"][:2])
    assert 0.0 <= meta["object_yaw_deg"] < 180.0
    # Tisch 2 mm UEBER PICK geteacht -> Abbruch vor jeder Bewegung
    points = _sim_table()
    probe = SimRobot(clock=SimClock()).connect()
    kin = Kinematics(probe)
    for name in ("ID0", "ID1", "ID2", "ID3"):
        pose = probe.fk(points[name])
        pose[2] += 0.009
        points[name] = kin.ik(pose, points[name])
    error, log = _record(tmp_path / "zwei", points)
    assert isinstance(error, SystemExit) and "PICK liegt" in str(error)
    assert config.TABLE_MIN_CLEARANCE_M < config.PICK_HEIGHT_DEFAULT_M


def test_keepout_zone_around_ignore():
    from bc.table import Keepout, KeepoutSurface, keepout_problems

    plane = fit_table([_pose(0.2, -0.3, 0.001), _pose(0.6, -0.3, 0.001), _pose(0.2, 0.0, 0.001)])
    zone = Keepout("IGNORE", (0.232, -0.091))
    assert zone.half_size == config.KEEPOUT_HALF_SIZE_M == 0.05
    inside = Waypoint(_pose(0.25, -0.10, 0.008), name="PICK")
    outside = Waypoint(_pose(0.285, -0.207, 0.006), name="PICK")      # Anlage 26.10.
    high = Waypoint(_pose(0.25, -0.10, 0.10), name="ANFLUG")            # darueber erlaubt
    assert "Sperrzone um IGNORE" in keepout_problems([inside], plane, [zone])[0]
    assert keepout_problems([outside, high], plane, [zone]) == []
    surface = KeepoutSurface(zone, plane)
    assert surface.clearance_for("wrist", [0.25, -0.1, 0.0], 0.08) is None
    assert surface.clearance_for("tool", [0.40, -0.1, 0.0], 0.0) is None   # ausserhalb
    assert surface.clearance_for("tool", [0.25, -0.1, 0.02], 0.0) < 0     # zu tief darin
    assert surface.clearance_for("tool", [0.25, -0.1, 0.06], 0.0) > 0


def test_sequence_with_keepout_resolves_zone():
    points = _sim_table()
    probe = SimRobot(clock=SimClock()).connect()
    pick = probe.fk(points["PICK"])
    ignore = pick.copy()
    ignore[0] += 0.12                       # weit genug weg
    points["IGNORE"] = Kinematics(probe).ik(ignore, points["PICK"])
    data = dict(SEQ, table=dict(SEQ["table"], keepout=["IGNORE"]))
    resolved = resolve_sequence(parse_sequence(data), SimRobot(clock=SimClock(), points=points).connect())
    assert [z.name for z in resolved.keepouts] == ["IGNORE"]
    assert "IGNORE" in resolved.keepout_points
    # Sperrpunkt darf nicht Teil der Bahn sein
    try:
        parse_sequence(dict(SEQ, table=dict(SEQ["table"], keepout=["PICK"])))
        assert False, "SequenceError erwartet"
    except SequenceError:
        pass
    import export_points  # tools/

    assert "IGNORE" in export_points.names_from_sequence(ROOT / "sequences" / "pick_to_station.json")


def test_block_counts_up_and_yaw_is_an_axis():
    from bc.table import yaw_deg

    assert recording.next_block("B07") == "B08"
    assert recording.next_block("B09") == "B10"
    assert recording.next_block("Lage 9") == "Lage 10"
    assert recording.next_block("Erste Daten") == "Erste Daten 2"
    assert recording.next_block("") == "B01"
    yaw = geometry.quat_from_axis_angle([0.0, 0.0, 1.0], np.radians(115))
    pose = _pose(0.3, 0.0, 0.01, geometry.quat_multiply(yaw, DOWN))
    assert abs(yaw_deg(pose) - 115.0) < 1e-6
    turned = geometry.quat_from_axis_angle([0.0, 0.0, 1.0], np.radians(295))
    assert abs(yaw_deg(_pose(0.3, 0, 0.01, geometry.quat_multiply(turned, DOWN))) - 115.0) < 1e-6

"""Automatische Objektlagen (bc/auto_layout.py, apps/record.py --lagen)."""

import contextlib
import io
import json
import sys
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.sim_robot import SimRobot
from bc.auto_layout import convex_hull, inside_with_margin, sample_pose
from bc.clock import SimClock
from bc.kinematics import Kinematics
from bc.table import Keepout, align_pose, clearance, fit_table, tilt_deg, yaw_deg

from test_put_back import _sim_robot_keeping_last_state
from test_sequence import _sim_points

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "apps"))

SEQ = {
    "name": "auto",
    "table": {"points": ["ID0", "ID1", "ID2", "ID3"], "pick_height": 0.008},
    "sequence": [
        {"point": "CLEAR_FOV", "motion": "ptp"},
        {"point": "PRE_PICK", "motion": "ptp", "above": "PICK", "dz": 0.05, "blend": 0.01},
        {"point": "PICK", "motion": "lin", "approach": True, "gripper": "close"},
        {"point": "PRE_PICK", "motion": "lin"},
        {"point": "PRE_PLACE", "motion": "ptp"},
    ],
}


def _points(size=0.10):
    """SimRobot-Punkte, Tischpunkte im Quadrat +-size um PICK, 8 mm darunter."""
    probe = SimRobot(clock=SimClock()).connect()
    points = _sim_points(probe)
    kin = Kinematics(probe)
    pick = probe.fk(points["PICK"])
    for k, (dx, dy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        pose = pick.copy()
        pose[0] += dx * size
        pose[1] += dy * size
        pose[2] -= 0.007
        points["ID%d" % k] = kin.ik(pose, points["PICK"])
    return points, probe


def test_region_hull_and_margin():
    hull = convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5)])
    assert len(hull) == 4
    assert inside_with_margin(hull, (0.5, 0.5), 0.4)
    assert not inside_with_margin(hull, (0.05, 0.5), 0.1)
    assert not inside_with_margin(hull, (1.5, 0.5), 0.0)


def test_sampled_poses_respect_region_keepout_distance_and_height():
    points, probe = _points()
    region = [probe.fk(points["ID%d" % k]) for k in range(4)]
    plane = fit_table(region)
    base = probe.fk(points["PICK"])
    zone = Keepout("IGNORE", tuple(base[:2] + [0.05, 0.0]), half_size=0.02)
    rng = np.random.default_rng(1)
    used = [tuple(base[:2])]
    hull = convex_hull([p[:2] for p in region])
    base_yaw = yaw_deg(align_pose(base, plane), plane)
    for _ in range(10):
        pose, turn = sample_pose(rng, base, plane, region, [zone], used,
                                 yaw_range_deg=30.0, pick_height=0.008)
        assert inside_with_margin(hull, pose[:2], config.AUTO_EDGE_MARGIN_M - 1e-9)
        assert not zone.contains_xy(pose[:2])
        assert min(np.linalg.norm(pose[:2] - np.asarray(u)) for u in used) >= config.AUTO_MIN_DIST_M
        assert abs(clearance(plane, pose) - 0.008) < 1e-9 and tilt_deg(pose, plane) < 1e-6
        delta = (yaw_deg(pose, plane) - base_yaw + 90) % 180 - 90
        assert abs(delta - turn) < 1e-6 and abs(turn) <= 30.0
        used.append(tuple(pose[:2]))
    try:
        sample_pose(rng, base, None, region)
        assert False, "ValueError erwartet"
    except ValueError:
        pass


def _record(tmp_path, extra, points):
    import record

    tmp_path.mkdir(parents=True, exist_ok=True)
    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(SEQ), encoding="utf-8")
    opened = []
    saved = sys.argv, record.open_robot
    sys.argv = ["record.py", "--sim", "--sequence", str(seq), "--seed", "4",
                "--block", "B01", "--out", str(tmp_path / "data")] + extra
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
    return error, log.getvalue(), opened


def test_record_places_the_object_at_new_poses_and_back(tmp_path):
    points, probe = _points()
    error, log, opened = _record(tmp_path, ["--episodes", "1", "--lagen", "3"], points)
    assert error is None, log[-2500:]
    metas = [json.loads((tmp_path / "data" / ("ep_%05d" % k) / "meta.json").read_text())["metadata"]
             for k in range(3)]
    assert [m["object_source"] for m in metas] == ["geteacht", "automatisch", "automatisch"]
    assert [m["block"] for m in metas] == ["B01", "B02", "B03"]
    assert len({m["repeat_group"] for m in metas}) == 3
    xy = [tuple(np.round(m["object_xy_m"], 4)) for m in metas]
    assert len(set(xy)) == 3
    # automatische Lagen: senkrecht und 8 mm ueber dem Tisch
    for m in metas[1:]:
        assert abs(m["pick_over_table_m"] - 0.008) < 5e-4
    assert "Umlegen: Objekt an Lage 2" in log and "Umlegen: Objekt an Lage 3" in log
    assert "Umlegen: Objekt an den geteachten PICK" in log
    assert "Naechster freier Block: B04" in log
    # zum Schluss: Arm offen am Start, Objekt wieder am geteachten PICK
    final = opened[-1].final_state
    assert np.allclose(final.joints, points["CLEAR_FOV"], atol=config.START_POSE_TOL_RAD)


def test_stop_file_ends_after_the_current_pose(tmp_path):
    points, _ = _points()
    stop = tmp_path / "stopp.flag"
    stop.parent.mkdir(parents=True, exist_ok=True)
    stop.write_text("x")
    error, log, _ = _record(tmp_path, ["--episodes", "1", "--lagen", "5",
                                       "--stopp-datei", str(stop)], points)
    assert error is None, log[-2000:]
    assert "Beenden nach dieser Lage angefordert." in log
    assert len(list((tmp_path / "data").glob("ep_*"))) == 1
    assert "Naechster freier Block: B02" in log


def test_auto_poses_need_the_table(tmp_path):
    points, _ = _points()
    for k in range(4):
        del points["ID%d" % k]
    error, log, _ = _record(tmp_path, ["--episodes", "1", "--lagen", "2"], points)
    assert isinstance(error, SystemExit) and "Tischpunkte" in str(error)


def _closes(tmp_path, extra):
    """Wie oft schliesst der Greifer in einem Lauf mit 2 Lagen x 2 Fahrten?"""
    import record

    points, _ = _points()
    closes = []
    original = record.open_robot

    def counting(kind, clock=None, **kw):
        robot = _sim_robot_keeping_last_state(clock, points, [])
        send = robot.gripper_command

        def logged(close):
            if close:
                closes.append(1)
            return send(close)

        robot.gripper_command = logged
        return robot

    tmp_path.mkdir(parents=True, exist_ok=True)
    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(SEQ), encoding="utf-8")
    saved = sys.argv
    sys.argv = ["record.py", "--sim", "--sequence", str(seq), "--seed", "4", "--episodes", "2",
                "--lagen", "2", "--out", str(tmp_path / "data")] + extra
    record.open_robot = counting
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log):
            record.main()
    finally:
        sys.argv, record.open_robot = saved, original
    return len(closes), log.getvalue()


def test_regrip_only_when_the_pose_changes(tmp_path):
    # 4 Rueckfahrten; Nachgreifen nur beim Umlegen an Lage 2 und bei der
    # Rueckkehr an PICK (2 von 4), "immer" nach jeder (4 von 4)
    default, log = _closes(tmp_path / "a", [])
    never, _ = _closes(tmp_path / "b", ["--nachgreifen", "nie"])
    always, _ = _closes(tmp_path / "c", ["--nachgreifen", "immer"])
    assert default - never == 2, log[-1500:]
    assert always - never == 4
    assert "Geschaetzte Dauer: ca." in log and "2 Fahrt(en) x 2 Lage(n)" in log


def test_duration_estimates_for_the_gui():
    from bc.gui import recording

    one = recording.estimate_duration_s(3, 1, probe=True)
    ten = recording.estimate_duration_s(3, 10, probe=True)
    assert 2 * 60 < one < 5 * 60 and 12 * 60 < ten < 18 * 60
    assert recording.format_duration(30) == "30 s"
    assert recording.format_duration(8 * 60) == "8 min"
    assert recording.format_duration(85 * 60) == "1 h 25 min"
    rows = recording.duration_table(3)
    assert rows[1].startswith("10 Lagen: ca.") and len(rows) == 5
    assert recording.duration_estimate("Geschaetzte Dauer: ca. 8 min (3 ...)") == "ca. 8 min (3 ...)"
    argv = recording.record_argv("x", sequence="s.json", lagen=3, regrip="nie")
    assert argv[argv.index("--nachgreifen") + 1] == "nie"
    assert "--nachgreifen" not in recording.record_argv("x", sequence="s.json",
                                                        regrip="lagenwechsel")


def test_edge_margin_is_adjustable():
    from bc.gui import recording

    argv = recording.record_argv("x", sequence="s.json", lagen=3, margin=0.02)
    assert argv[argv.index("--randabstand") + 1] == "0.02"
    assert "--randabstand" not in recording.record_argv(
        "x", sequence="s.json", lagen=3, margin=config.AUTO_EDGE_MARGIN_M)
    points, probe = _points(size=0.05)
    region = [probe.fk(points["ID%d" % k]) for k in range(4)]
    plane = fit_table(region)
    base = probe.fk(points["PICK"])
    hull = convex_hull([p[:2] for p in region])
    rng = np.random.default_rng(2)
    # 10 cm breiter Bereich: mit 4 cm Rand bleibt fast nichts, mit 1 cm genug
    for _ in range(5):
        pose, _turn = sample_pose(rng, base, plane, region, margin=0.01, min_dist=0.0)
        assert inside_with_margin(hull, pose[:2], 0.01 - 1e-9)
    try:
        sample_pose(rng, base, plane, region, margin=0.06, tries=50)
        assert False, "ValueError erwartet"
    except ValueError:
        pass

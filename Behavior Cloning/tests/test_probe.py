"""Probefahrt, Plausibilitaet der Punkte, Planung vorab (AP 2.2/2.4/4.2).

Anlage 2026-10-23: PICK und PRE_GRASP vertauscht geteacht, leichter Crash;
zwischen den Episoden lange Planungspausen. Daraus: Plausibilitaetspruefung
vor jeder Bewegung, langsame Probefahrt, Planung einmal je Aufruf.
"""

import contextlib
import io
import json
import sys
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.gui import recording
from bc.kinematics import Kinematics
from bc.noise import generate_plan
from bc.probe import probe_plans, reversed_plan, slowed_plan
from bc.put_back import plan_put_back
from bc.recorder import EpisodeRecorder
from bc.sequence import approach_problems, same_points
from bc.trajectory import Waypoint, build_ideal_trajectory

from test_put_back import _resolved, _sim_robot_keeping_last_state
from test_sequence import PICK_SEQUENCE, _sim_points

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))

QUAT = [0.0084, -0.5542, -0.8323, 0.0074]


def _approach(pre_grasp_xyz, pick_xyz):
    return [
        Waypoint(np.r_[pre_grasp_xyz, QUAT], name="PRE_GRASP"),
        Waypoint(np.r_[pick_xyz, QUAT], name="PICK", approach=True, motion="lin",
                 gripper_closed=True),
    ]


def test_swapped_pick_is_caught_before_motion():
    # Punkte der Anlage 2026-10-23, ep_00003 (vertauscht) und ep_00004 (richtig)
    swapped = approach_problems(_approach([0.514, -0.009, 0.015], [0.513, -0.027, 0.064]))
    # 49 mm darunter -- und dort auch 18 mm seitlich versetzt: beides gemeldet
    assert len(swapped) == 2 and "UNTER" in swapped[0] and "vertauscht" in swapped[0]
    assert "senkrecht" in swapped[1]
    assert approach_problems(_approach([0.521, 0.002, 0.079], [0.521, 0.002, 0.017])) == []
    assert approach_problems(_approach([0.605, 0.026, 0.088], [0.605, 0.026, 0.048])) == []
    # knapp darueber, aber weniger als die Mindesthoehe
    assert approach_problems(_approach([0.5, 0.0, 0.025], [0.5, 0.0, 0.020]))
    # schraeg statt senkrecht
    slanted = approach_problems(_approach([0.53, 0.0, 0.08], [0.5, 0.0, 0.02]))
    assert len(slanted) == 1 and "senkrecht" in slanted[0]
    # Ohne approach-Flag wird nichts geprueft (Transit darf hinunter)
    wps = _approach([0.5, 0.0, 0.0], [0.5, 0.0, 0.1])
    wps[1].approach = False
    assert approach_problems(wps) == []


def test_same_points_compares_names_and_joints():
    a = {"A": {"joints": [0.0, 1.0], "pose_quat": [0] * 7}}
    assert same_points(a, {"A": {"joints": [0.0, 1.0 + 1e-9], "pose_quat": [1] * 7}})
    assert not same_points(a, {"A": {"joints": [0.0, 1.001], "pose_quat": [0] * 7}})
    assert not same_points(a, {"B": {"joints": [0.0, 1.0], "pose_quat": [0] * 7}})


def _plans():
    clock, robot, wps = _resolved()
    kin = Kinematics(robot)
    workspace = default_workspace(table_height_m=min(wp.pose_quat[2] for wp in wps) - 0.15)
    ideal = build_ideal_trajectory(wps, fk=robot.fk)
    plan = generate_plan(kin, workspace, ideal, wps[0].joints, rng=np.random.default_rng(1))
    back = plan_put_back(kin, workspace, wps, fk=robot.fk)
    return clock, robot, wps, plan, back


def test_slowed_plan_is_the_ideal_path_stretched_in_time():
    _, _, _, plan, _ = _plans()
    slow = slowed_plan(plan, 0.4)
    n, m = len(plan), len(slow)
    assert m == int(np.ceil((n - 1) / 0.4)) + 1
    # dieselbe Bahn: Anfang/Ende und jeder Stuetzpunkt der idealen Bahn
    assert np.allclose(slow.joints_ideal[0], plan.joints_ideal[0])
    assert np.allclose(slow.joints_ideal[-1], plan.joints_ideal[-1])
    s = np.linspace(0, n - 1, m)
    for k in range(m):
        lo, hi = int(np.floor(s[k])), int(np.ceil(s[k]))
        a, b = plan.joints_ideal[lo], plan.joints_ideal[hi]
        between = (np.minimum(a, b) - 1e-12 <= slow.joints_ideal[k]) & (
            slow.joints_ideal[k] <= np.maximum(a, b) + 1e-12)
        assert between.all(), k
    # ohne Rauschen, Greifer immer offen
    assert np.array_equal(slow.joints_noisy, slow.joints_ideal)
    assert np.all(slow.gripper == config.GRIPPER_OPEN)
    # langsamer: hoechste Schrittweite je Takt mit dem Faktor kleiner
    v_plan = np.abs(np.diff(plan.joints_ideal, axis=0)).max()
    v_slow = np.abs(np.diff(slow.joints_ideal, axis=0)).max()
    assert v_slow <= 0.41 * v_plan
    assert np.allclose(np.linalg.norm(slow.poses_ideal[:, 3:], axis=1), 1.0)
    for bad in (0.0, 1.5):
        try:
            slowed_plan(plan, bad)
            assert False, "ValueError erwartet"
        except ValueError:
            pass


def test_probe_returns_on_put_back_path_or_reversed():
    _, _, wps, plan, back = _plans()
    hin, zurueck = probe_plans(plan, back, 0.5)
    assert np.allclose(zurueck.joints_ideal[0], hin.joints_ideal[-1], atol=1e-6)
    assert np.allclose(zurueck.joints_ideal[-1], wps[0].joints, atol=1e-6)
    assert np.all(zurueck.gripper == config.GRIPPER_OPEN)
    hin, zurueck = probe_plans(plan, None, 0.5)
    assert np.array_equal(zurueck.joints_ideal, hin.joints_ideal[::-1])
    assert np.array_equal(reversed_plan(zurueck).joints_ideal, hin.joints_ideal)


def _run_probe(robot, clock, plan, back, answer):
    import record

    commands = []
    send = robot.gripper_command
    robot.gripper_command = lambda close: (commands.append(bool(close)), send(close))[1]
    saved = record.ask
    record.ask = lambda prompt, default="": answer
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log):
            record.run_probe([None, plan], back, robot, EpisodeRecorder(robot, [], clock), 0.5)
        return None, commands, log.getvalue()
    except SystemExit as exc:
        return exc, commands, log.getvalue()
    finally:
        record.ask = saved


def test_run_probe_drives_there_and_back_with_open_gripper():
    clock, robot, wps, plan, back = _plans()
    exc, commands, log = _run_probe(robot, clock, plan, back, "j")
    assert exc is None, exc
    assert commands and not any(commands)  # Greifer nie geschlossen
    assert np.allclose(robot.read_state().joints, wps[0].joints, atol=config.START_POSE_TOL_RAD)
    assert recording.probe_question([z for z in log.splitlines() if z.startswith("PROBEFAHRT")][0])
    assert "freigegeben" in log


def test_run_probe_without_release_stops_the_run():
    clock, robot, _, plan, back = _plans()
    for answer in ("n", ""):  # Nein, oder keine Eingabe (stdin zu)
        exc, _, _ = _run_probe(robot, clock, plan, back, answer)
        assert isinstance(exc, SystemExit) and "abgebrochen" in str(exc)


def _record_main(tmp_path, extra, points=None, patch=None):
    import record
    from bc.adapters.sim_robot import SimRobot

    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(PICK_SEQUENCE), encoding="utf-8")
    points = points or _sim_points(SimRobot(clock=SimClock()).connect())
    opened = []

    def fake_open_robot(kind, clock=None, **kw):
        return _sim_robot_keeping_last_state(clock, points, opened)

    out = tmp_path / "data"
    argv = ["record.py", "--sim", "--sequence", str(seq), "--seed", "5",
            "--out", str(out)] + extra
    saved = sys.argv, record.open_robot, dict(patch or {})
    originals = {name: getattr(record, name) for name in (patch or {})}
    sys.argv, record.open_robot = argv, fake_open_robot
    for name, value in (patch or {}).items():
        setattr(record, name, value)
    log = io.StringIO()
    error = None
    try:
        with contextlib.redirect_stdout(log):
            record.main()
    except SystemExit as exc:
        error = exc
    finally:
        sys.argv, record.open_robot = saved[0], saved[1]
        for name, value in originals.items():
            setattr(record, name, value)
    return error, log.getvalue(), out, opened, points


def test_record_plans_ideal_path_and_put_back_once(tmp_path):
    import record

    calls = {"ideal": 0, "back": 0}

    def counting(name, fn):
        def wrapped(*a, **kw):
            calls[name] += 1
            return fn(*a, **kw)
        return wrapped

    error, log, out, _, _ = _record_main(
        tmp_path, ["--episodes", "3"],
        patch={"solve_ideal": counting("ideal", record.solve_ideal),
               "plan_put_back": counting("back", record.plan_put_back)})
    assert error is None, log[-2000:]
    assert calls == {"ideal": 1, "back": 1}
    assert len([z for z in log.splitlines() if z.startswith("Planung: 3 Episode(n) + Rueckfahrt")]) == 1
    # gegen den SimRobot keine Probefahrt
    assert "PROBEFAHRT" not in log
    assert len([z for z in log.splitlines() if z.startswith("Episode ") and "-> ep_" in z]) == 3


def test_record_refuses_swapped_points_before_any_motion(tmp_path):
    from bc.adapters.sim_robot import SimRobot

    points = _sim_points(SimRobot(clock=SimClock()).connect())
    points["PICK"], points["PRE_GRASP"] = points["PRE_GRASP"], points["PICK"]
    error, log, out, opened, _ = _record_main(tmp_path, ["--episodes", "1"], points=points)
    assert isinstance(error, SystemExit) and "unplausibel" in str(error)
    assert "vertauscht" in str(error)
    # nichts bewegt, nichts aufgezeichnet
    assert np.allclose(opened[-1].final_state.joints, points["CLEAR_FOV"], atol=1e-9)
    assert not list(out.glob("ep_*"))


def test_record_stops_when_points_change_during_the_run(tmp_path):
    import record

    resolve = record.resolve_points
    calls = []

    def touched_up(sequence, robot, use_sim):
        calls.append(1)
        resolved = resolve(sequence, robot, use_sim)
        if len(calls) > 1:  # Touch-up nach dem Start
            resolved.points["PICK"]["joints"] = list(
                np.asarray(resolved.points["PICK"]["joints"]) + 0.01)
        return resolved

    error, log, out, _, _ = _record_main(
        tmp_path, ["--episodes", "3"], patch={"resolve_points": touched_up})
    assert error is None, log[-2000:]
    assert "Punkte in der Steuerung haben sich seit dem Start geaendert" in log
    assert len(list(out.glob("ep_*"))) == 1


def test_gui_probe_flag_and_question():
    assert "--ohne-probefahrt" not in recording.record_argv("data_vm/x", robot_kind="neura")
    assert "--ohne-probefahrt" in recording.record_argv("data_vm/x", probe=False)
    import record

    assert recording.PROBE_PROMPT == record.PROBE_PROMPT
    assert recording.PROBE_ANSWERS["weiter"] == "j"
    assert recording.probe_question("PROBEFAHRT beendet -- Aufnahme starten? [j]a / [n]ein")
    assert not recording.probe_question("Probefahrt: ideale Bahn mit 40 % Tempo")
    assert not recording.probe_question("")


def test_record_stops_cleanly_when_the_servo_guard_refuses(tmp_path):
    # Anlage 2026-10-26: der ServoGuard-Abbruch endete mit einem Traceback
    import record
    from bc.ports import ServoLimitError

    class Refusing(record.EpisodeRecorder):
        def record(self, plan, metadata=None):
            raise ServoLimitError("servo_j abgelehnt: Sprung 0.0245 rad an Gelenk 1")

    error, log, out, _, _ = _record_main(
        tmp_path, ["--episodes", "2"], patch={"EpisodeRecorder": Refusing})
    assert error is None, log[-1500:]
    assert "Episode 0 abgebrochen: servo_j abgelehnt" in log
    assert "Gelenktempo: ideal max" in log
    assert not list(out.glob("ep_*"))

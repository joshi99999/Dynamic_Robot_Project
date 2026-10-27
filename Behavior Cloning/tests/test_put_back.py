"""Zuruecklegen zwischen Wiederholungen derselben Objektlage (AP 2.2/2.4)."""

import sys
from pathlib import Path
from types import SimpleNamespace

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.cam_sim import SimCamera
from bc.capture import DirectCapture
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.gui import recording
from bc.kinematics import Kinematics
from bc.noise import generate_plan
from bc.put_back import (
    PutBackError,
    grasp_index,
    plan_put_back,
    put_back_waypoints,
    stratified_scales,
)
from bc.recorder import EpisodeRecorder
from bc.sequence import parse_sequence, resolve_sequence
from bc.trajectory import Waypoint, build_ideal_trajectory

from test_sequence import PICK_SEQUENCE, _robot, _sim_points

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))


def _resolved():
    clock = SimClock()
    robot = _robot(clock)
    return clock, robot, resolve_sequence(parse_sequence(PICK_SEQUENCE), robot).waypoints


def _sim_robot_keeping_last_state(clock, points, opened):
    """SimRobot fuer record.main(); haelt beim close() den letzten Zustand fest."""
    from bc.adapters.sim_robot import SimRobot

    robot = SimRobot(clock=clock, points=points)
    close = robot.close

    def closing():
        robot.final_state = robot.read_state()
        return close()

    robot.close = closing
    opened.append(robot)
    return robot


def test_put_back_path_is_the_forward_path_reversed():
    _, _, wps = _resolved()
    back = put_back_waypoints(wps)
    assert [wp.name for wp in back] == [
        "PRE_PLACE", "PRE_GRASP", "PICK", "PRE_GRASP", "APPROACH_01", "CLEAR_FOV"]
    # Objekt bis zum Greifpunkt halten, dort oeffnen, danach offen
    assert [wp.gripper_closed for wp in back] == [True, True, False, False, False, False]
    # Segment B->A faehrt wie A->B: PRE_PLACE->PRE_GRASP ptp, Abstieg lin, ...
    assert [wp.motion for wp in back][1:] == ["ptp", "lin", "lin", "ptp", "ptp"]
    # Absetzen langsam, auch wenn das Segment vorwaerts das Anheben war
    assert back[2].approach
    assert all(wp.blend_m == 0.0 for wp in back)
    assert np.allclose(back[-1].joints, wps[0].joints)


def test_put_back_needs_exactly_one_close():
    pose = np.r_[0.3, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]
    j = np.zeros(6)
    W = lambda closed: Waypoint(pose, gripper_closed=closed, joints=j)  # noqa: E731
    for case in ([W(False), W(False)],            # nie gegriffen
                 [W(True), W(True)],              # schon zu am Start
                 [W(False), W(True), W(False)]):  # oeffnet wieder
        try:
            grasp_index(case)
            assert False, "PutBackError erwartet"
        except PutBackError:
            pass
    assert grasp_index([W(False), W(True), W(True)]) == 1


def test_put_back_returns_object_to_pick_and_arm_to_start():
    clock, robot, wps = _resolved()
    kin = Kinematics(robot)
    workspace = default_workspace(table_height_m=min(wp.pose_quat[2] for wp in wps) - 0.15)
    ideal = build_ideal_trajectory(wps, fk=robot.fk)
    plan = generate_plan(kin, workspace, ideal, wps[0].joints, rng=np.random.default_rng(3))
    cams = [DirectCapture(SimCamera(cfg, clock=clock)).start() for cfg in config.SIM_CAMERAS]
    episode = EpisodeRecorder(robot, cams, clock).record(plan)
    assert not episode.discarded, episode.discard_reason

    back = plan_put_back(kin, workspace, wps, fk=robot.fk)
    # unverrauscht: Soll = Ist
    assert np.array_equal(back.joints_ideal, back.joints_noisy)
    # Greiferbefehle mit der Stellung mitschreiben, an der sie fallen
    commands = []
    send = robot.gripper_command

    def logged(close):
        commands.append((bool(close), robot.read_state().tcp_quat[:3].copy()))
        return send(close)

    robot.gripper_command = logged
    result = EpisodeRecorder(robot, [], clock).record(back)
    assert not result.discarded, result.discard_reason
    # Arm steht am Start, die naechste Episode braucht keine Hinfahrt
    assert np.allclose(robot.read_state().joints, wps[0].joints, atol=config.START_POSE_TOL_RAD)
    assert not robot.read_state().gripper_closed
    # Am Greifpunkt: auf, zu, auf (Nachgreifen, config.PUT_BACK_REGRIP) --
    # alle drei an derselben Stelle, auf der Greifpose (Anwender 2026-10-26)
    at_pick = [(close, pos) for close, pos in commands[1:]]
    assert [close for close, _ in at_pick] == [False, True, False]
    for _close, pos in at_pick:
        assert np.linalg.norm(pos - robot.fk(wps[3].joints)[:3]) < 2e-3


def test_record_put_back_stops_after_failed_or_incomplete_episode():
    from record import put_back

    clock, robot, wps = _resolved()
    kin = Kinematics(robot)
    workspace = default_workspace(table_height_m=min(wp.pose_quat[2] for wp in wps) - 0.15)
    back = plan_put_back(kin, workspace, wps, fk=robot.fk)
    rec = EpisodeRecorder(robot, [], clock)
    incomplete = SimpleNamespace(metadata={"recorded_steps": 40, "planned_steps": 200},
                                 success=None, discard_reason="schutzstopp")
    failed = SimpleNamespace(metadata={"recorded_steps": 200, "planned_steps": 200},
                             success=False, discard_reason="")
    # Bewegt nichts: der Arm steht am Start, nicht an PRE_PLACE
    before = robot.read_state().joints.copy()
    assert put_back(incomplete, back, rec) is False
    assert put_back(failed, back, rec) is False
    assert np.allclose(robot.read_state().joints, before)


def test_stratified_scales_cover_the_range():
    rng = np.random.default_rng(0)
    for n in (1, 3, 4):
        scales = stratified_scales(n, rng, 0.0, 1.0)
        assert len(scales) == n
        # je Teilintervall genau einer
        assert sorted(int(s * n) for s in scales) == list(range(n))
    # Ueber viele Lagen bleibt die Verteilung U(0,1)
    many = np.concatenate([stratified_scales(3, rng) for _ in range(2000)])
    assert abs(many.mean() - 0.5) < 0.02 and abs(many.std() - 12 ** -0.5) < 0.02


def test_validation_split_keeps_repeats_together():
    from train import split_episodes

    # Ohne Gruppen: dasselbe Ergebnis wie bisher
    plain = split_episodes(10, 0.2, seed=1)
    assert split_episodes(10, 0.2, seed=1, groups=[None] * 10) == plain
    groups = ["L%d" % (k // 3) for k in range(12)]  # 4 Lagen x 3 Fahrten
    for seed in range(20):
        train, val = split_episodes(12, 0.25, seed, groups=groups)
        assert sorted(train + val) == list(range(12))
        assert {groups[k] for k in train}.isdisjoint({groups[k] for k in val})
        assert len(val) == 3


def test_gui_puts_back_by_default_and_advises_repeats():
    # Zuruecklegen ist Standard -- nur das Abschalten steht in der Kommandozeile
    argv = recording.record_argv("data_vm/x", robot_kind="neura", episodes=3,
                                 sequence="s.json")
    assert "--ohne-zuruecklegen" not in argv
    assert "--ohne-zuruecklegen" in recording.record_argv(
        "data_vm/x", sequence="s.json", put_back=False)
    # ohne Ablaufdatei gibt es nichts zurueckzulegen
    assert "--ohne-zuruecklegen" not in recording.record_argv("data_vm/x", put_back=False)
    # Mit Zuruecklegen sind eine oder mehrere Fahrten an der Anlage in Ordnung
    assert recording.episode_advice(True, 1) is None
    assert recording.episode_advice(True, 3) is None
    assert recording.episode_advice(True, 8)
    assert "Zurücklegen" in recording.episode_advice(True, 3, put_back=False)


def test_record_main_puts_back_after_every_episode(tmp_path):
    import contextlib
    import io
    import json

    import record
    from bc import dataset
    from bc.adapters.sim_robot import SimRobot

    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(PICK_SEQUENCE), encoding="utf-8")
    probe = SimRobot(clock=SimClock()).connect()
    points = _sim_points(probe)
    opened = []

    def fake_open_robot(kind, clock=None, **kw):
        return _sim_robot_keeping_last_state(clock, points, opened)

    out = tmp_path / "data"
    argv = ["record.py", "--sim", "--sequence", str(seq), "--episodes", "3",
            "--block", "B01", "--seed", "5", "--out", str(out)]
    saved = sys.argv, record.open_robot
    sys.argv, record.open_robot = argv, fake_open_robot
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log):
            record.main()
    finally:
        sys.argv, record.open_robot = saved
    # Standard: nach JEDER Fahrt abgelegt, auch nach der letzten
    put_backs = [z for z in log.getvalue().splitlines() if z.startswith("Zuruecklegen: ") and "Schritte" in z]
    assert len(put_backs) == 3 and "nicht aufgezeichnet" in put_backs[0], log.getvalue()[-2000:]

    episodes = [dataset.load_episode(out, k) for k in range(3)]
    assert all(not ep.discarded for ep in episodes), [ep.discard_reason for ep in episodes]
    metas = [ep.metadata for ep in episodes]
    assert len({m["repeat_group"] for m in metas}) == 1
    assert [m["repeat_index"] for m in metas] == [0, 1, 2]
    assert all(m["repeats"] == 3 and m["noise_scale_mode"] == "gestaffelt je Lage" for m in metas)
    # je Drittel von [0, 1] genau ein Rauschfaktor
    assert sorted(int(m["noise_scale"] * 3) for m in metas) == [0, 1, 2]
    # Am Ende steht der Arm offen am Start, das Objekt liegt am Greifpunkt
    final = opened[-1].final_state
    assert np.allclose(final.joints, points["CLEAR_FOV"], atol=config.START_POSE_TOL_RAD)
    assert not final.gripper_closed
    # Jede Episode startet offen und endet geschlossen an der Uebergabe
    for ep in episodes:
        assert ep.arrays["observation.state"][0, 13] == config.GRIPPER_OPEN
        assert ep.arrays["observation.state"][-1, 13] == config.GRIPPER_CLOSED


def test_record_main_without_put_back_keeps_the_object(tmp_path):
    import contextlib
    import io
    import json

    import record
    from bc.adapters.sim_robot import SimRobot

    seq = tmp_path / "ablauf.json"
    seq.write_text(json.dumps(PICK_SEQUENCE), encoding="utf-8")
    points = _sim_points(SimRobot(clock=SimClock()).connect())
    opened = []

    def fake_open_robot(kind, clock=None, **kw):
        return _sim_robot_keeping_last_state(clock, points, opened)

    argv = ["record.py", "--sim", "--sequence", str(seq), "--episodes", "1",
            "--ohne-zuruecklegen", "--seed", "5", "--out", str(tmp_path / "data")]
    saved = sys.argv, record.open_robot
    sys.argv, record.open_robot = argv, fake_open_robot
    log = io.StringIO()
    try:
        with contextlib.redirect_stdout(log):
            record.main()
    finally:
        sys.argv, record.open_robot = saved
    assert "Zuruecklegen AUS" in log.getvalue()
    # Arm haelt das Objekt an der Uebergabe (PRE_PLACE)
    final = opened[-1].final_state
    assert np.allclose(final.joints, points["PRE_PLACE"], atol=1e-2)
    assert final.gripper_closed


def test_put_back_places_slowly_and_regrips_on_the_spot():
    from bc.noise import joint_speed
    from bc.put_back import with_regrip

    clock, robot, wps = _resolved()
    kin = Kinematics(robot)
    workspace = default_workspace(table_height_m=min(wp.pose_quat[2] for wp in wps) - 0.15)
    plain = plan_put_back(kin, workspace, wps, fk=robot.fk, regrip=False)
    regrip = plan_put_back(kin, workspace, wps, fk=robot.fk)
    assert len(regrip) == len(plain) + 2 * config.GRIPPER_DWELL_STEPS
    # zu und auf, waehrend der Arm still auf der Greifpose steht
    changes = np.flatnonzero(np.diff(regrip.gripper) != 0) + 1
    assert len(changes) == 3
    still = regrip.joints_ideal[changes[0]:changes[2] + 1]
    assert np.allclose(still, still[0])
    assert regrip.dwell_mask[changes[0]:changes[2]].all()
    # ohne Oeffnen im Plan: unveraendert
    assert with_regrip(plain) is not plain
    no_open = with_regrip(type(plain)(**dict(vars(plain), gripper=np.zeros(len(plain)))))
    assert len(no_open) == len(plain)
    # Abstieg zum Ablegen langsamer als mit Aufnahme-Tempo
    fast = plan_put_back(kin, workspace, wps, fk=robot.fk, regrip=False,
                         approach_speed=config.APPROACH_SPEED_MS)
    assert len(fast) == len(plain)  # Tempo wird auf PUT_BACK_APPROACH_SPEED_MS gedeckelt
    assert config.PUT_BACK_APPROACH_SPEED_MS < config.APPROACH_SPEED_MS
    assert joint_speed(regrip.joints_ideal, regrip.rate_hz)[0] <= config.PLAN_MAX_JOINT_SPEED_RADS

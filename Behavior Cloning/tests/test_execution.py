"""Ausfuehrungsart servo_j / online (bc/execution.py).

Begruendung und Messungen: Dokumentation/Zittern-an-der-Anlage.md.
"""

import math

import _paths  # noqa: F401

from bc import config, execution


def test_auto_is_online_only_at_the_real_plant():
    assert execution.resolve_mode("auto", "neura", False) == "online"
    # VM (alpha.102) faehrt movelinear_online nicht, SimRobot bildet nur nach
    assert execution.resolve_mode("auto", "neura", True) == "servo_j"
    assert execution.resolve_mode("auto", "neura", None) == "servo_j"
    assert execution.resolve_mode("auto", "sim", True) == "servo_j"
    # ausdruecklich gewaehlt gilt ueberall
    assert execution.resolve_mode("online", "sim", True) == "online"
    assert execution.resolve_mode("servo_j", "neura", False) == "servo_j"


def test_online_limits_follow_the_planner_peaks():
    vel, acc = execution.online_limits(0.15, 0.5, speed_factor=1.3, accel_factor=1.5, ramp_s=0.5)
    assert vel[:3] == [0.15 * 1.3] * 3 and vel[3:] == [0.5 * 1.3] * 3
    # Spitzenbeschleunigung der Sinus-Rampe: v * pi / (2 T)
    assert abs(acc[0] - 1.5 * 0.15 * math.pi / 1.0) < 1e-12
    assert abs(acc[3] - 1.5 * 0.5 * math.pi / 1.0) < 1e-12


def test_describe_carries_everything_inference_must_repeat():
    d = execution.describe("online")
    assert d["mode"] == "online"
    assert d["lead_steps"] == config.ONLINE_LEAD_STEPS
    assert d["speed_factor"] == config.ONLINE_SPEED_FACTOR
    assert len(d["vel"]) == 6 and len(d["acc"]) == 6
    assert execution.describe("servo_j") == {"mode": "servo_j"}
    for bad in (dict(mode="quatsch"), dict(mode="online", lead_steps=9)):
        try:
            execution.describe(**bad)
        except ValueError:
            continue
        raise AssertionError("abgelehnt erwartet: %r" % bad)


def test_old_recordings_count_as_servo_j():
    assert execution.mode_of({}) == "servo_j"
    assert execution.mode_of({"execution": {"mode": "online"}}) == "online"


def test_lead_index_holds_the_last_target():
    assert execution.lead_index(0, 2, 10) == 2
    assert execution.lead_index(8, 2, 10) == 9
    assert execution.lead_index(9, 2, 10) == 9


def test_servo_j_pos_only_when_chosen():
    # An der Anlage ungeprueft (VM 2026-10-14) -- auto waehlt es nie.
    for kind, sim in (("neura", False), ("neura", True), ("sim", True)):
        assert execution.resolve_mode("auto", kind, sim) != "servo_j_pos"
    assert execution.resolve_mode("servo_j_pos", "neura", False) == "servo_j_pos"
    assert execution.describe("servo_j_pos") == {"mode": "servo_j_pos"}
    assert execution.servo_profile({"mode": "servo_j_pos"}) == "pos"
    assert execution.servo_profile({"mode": "servo_j"}) == "linear"
    assert execution.servo_profile({}) == "linear"          # aeltere Aufnahmen
    assert execution.servo_profile({"mode": "online"}) is None


# -- "Erst weiter, wenn erreicht" (Anlage 2026-10-16) ------------------------


def _sim_online():
    import numpy as np

    from bc.adapters.sim_robot import SimRobot
    from bc.clock import SimClock

    clock = SimClock()
    robot = SimRobot(clock=clock).connect()
    robot.activate_servo("position")
    start = robot.read_state().joints.copy()
    # klein genug fuer den ServoGuard (erster Sollwert ohne Vorgeschichte)
    target = start + np.array([0.0, 0.012, -0.012, 0.0, 0.0, 0.0])
    return clock, robot, start, target


def test_describe_carries_the_reach_rule_for_online_only():
    d = execution.describe("online")
    assert d["reach_tol_mm"] == config.ONLINE_REACH_TOL_MM
    assert d["reach_settle_steps"] == config.ONLINE_REACH_SETTLE_STEPS
    assert d["reach_timeout_s"] == config.ONLINE_REACH_TIMEOUT_S
    assert d["hold_dwell_steps"] == config.GRIPPER_DWELL_STEPS
    assert execution.reach_rule(d)
    # abschaltbar -- so fahren aeltere online-Aufnahmen ohne die Schluessel
    alt = execution.describe("online", reach_tol_mm=None)
    assert "reach_tol_mm" not in alt and not execution.reach_rule(alt)
    assert not execution.reach_rule({"mode": "online", "lead_steps": 2})
    assert not execution.reach_rule(execution.describe("servo_j"))
    for bad in (dict(reach_tol_mm=0.0), dict(reach_timeout_s=0.0), dict(reach_settle_steps=0)):
        try:
            execution.describe("online", **bad)
        except ValueError:
            continue
        raise AssertionError("abgelehnt erwartet: %r" % bad)


def test_lead_index_does_not_look_past_the_end_of_a_hold():
    # Haltepunkt mit Dwell-Ende 12: bis dahin hoechstens Ziel 12, danach normal
    assert execution.lead_index(9, 2, 30, hold_end=12) == 11
    assert execution.lead_index(11, 2, 30, hold_end=12) == 12
    assert execution.lead_index(12, 2, 30, hold_end=12) == 12
    assert execution.lead_index(13, 2, 30, hold_end=12) == 15
    assert execution.lead_index(13, 2, 30, hold_end=None) == 15


def test_hold_blocks_start_at_the_gripper_change_and_end_with_the_dwell():
    gripper = [0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0]
    dwell = [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0]
    assert execution.hold_blocks(gripper, dwell) == [(3, 5), (8, 9)]
    assert execution.hold_blocks([0, 0, 0], [0, 0, 0]) == []
    blocks = [(3, 5), (8, 9)]
    assert execution.next_hold_end(0, blocks) == 5
    assert execution.next_hold_end(5, blocks) == 5
    assert execution.next_hold_end(6, blocks) == 9
    assert execution.next_hold_end(10, blocks) is None


def test_wait_until_reached_holds_the_target_until_the_arm_is_there():
    import numpy as np

    clock, robot, start, target = _sim_online()
    ex = execution.describe("online")
    gesendet = []
    original = robot.online_j

    def mitschreiben(q, vel, acc):
        gesendet.append(np.asarray(q, dtype=float).copy())
        return original(q, vel, acc)

    robot.online_j = mitschreiben
    t0 = clock.now()
    r = execution.wait_until_reached(robot, target, ex, clock)
    assert r["reached"] and r["reason"] == ""
    assert r["start_mm"] > 2.0 and r["dist_mm"] <= ex["reach_tol_mm"]
    # immer dasselbe Ziel, ein Aufruf je 15-Hz-Takt
    assert gesendet and all(np.allclose(q, target) for q in gesendet)
    assert r["ticks"] == len(gesendet)
    assert abs(r["wait_s"] - (clock.now() - t0)) < 1e-9
    assert abs(r["wait_s"] - r["ticks"] / config.CONTROL_RATE_HZ) < 1e-6
    dist = 1e3 * np.linalg.norm(robot.read_state().tcp_quat[:3] - robot.fk(target)[:3])
    assert dist <= ex["reach_tol_mm"]


def test_wait_until_reached_needs_settle_steps_in_a_row():
    clock, robot, start, target = _sim_online()
    # Arm steht schon dort: trotzdem reach_settle_steps Messungen
    robot._joints = target.copy()
    ex = execution.describe("online", reach_settle_steps=3)
    r = execution.wait_until_reached(robot, target, ex, clock)
    assert r["reached"] and r["ticks"] == 2 and r["start_mm"] < 1e-6


def test_wait_until_reached_gives_up_after_the_timeout_without_estop():
    clock, robot, start, target = _sim_online()
    robot.online_j = lambda q, vel, acc: None          # Arm folgt nicht
    ex = execution.describe("online", reach_timeout_s=1.0)
    r = execution.wait_until_reached(robot, target, ex, clock)
    assert not r["reached"] and r["reason"] == "zeit"
    assert 1.0 <= r["wait_s"] <= 1.0 + 1.5 / config.CONTROL_RATE_HZ
    assert not robot.stop_requested                     # geordnet, kein Not-Halt
    text = execution.reach_failure_text(r, 42)
    assert "haltepunkt nicht erreicht" in text and "Takt 42" in text
    assert "NICHT ERREICHT" in execution.reach_line(dict(r, step=42))


def test_wait_until_reached_stops_on_a_stop_request():
    clock, robot, start, target = _sim_online()
    original = robot.online_j
    zaehler = {"n": 0}

    def stoppt(q, vel, acc):
        zaehler["n"] += 1
        if zaehler["n"] == 2:
            robot.emergency_stop()
            return None
        return original(q, vel, acc)

    robot.online_j = stoppt
    r = execution.wait_until_reached(robot, target, execution.describe("online"), clock)
    assert not r["reached"] and r["reason"] == "schutzstopp"
    assert execution.reach_failure_text(r) == "schutzstopp"

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

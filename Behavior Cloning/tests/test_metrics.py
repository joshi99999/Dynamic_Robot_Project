"""Bewertung einzelner Aufnahmen (bc/metrics.episode_quality).

Die drei Faelle aus der Auswertung vom 2026-10-03, synthetisch nachgestellt:
gesund, Arm laeuft hinterher (kleiner Override), Stop-and-go (verspaetete
Befehle).
"""

import _paths  # noqa: F401

import numpy as np

from bc import config, metrics

N = 150


def _command():
    t = np.linspace(0.0, 1.0, N)
    smooth = 0.5 * (1 - np.cos(np.pi * t))          # glatter Hub 0 -> 1 rad
    return np.tile(smooth[:, None], (1, 6)) * np.linspace(0.2, 1.0, 6)


def _arrays(state_joints, command):
    state = np.zeros((N, 14), dtype=np.float32)
    state[:, :6] = state_joints
    return {"observation.state": state, "aux.joints_command": command.astype(np.float32)}


def _meta(**extra):
    meta = {"rate_hz": 15.0, "servo_rate_hz": 60.0, "override": 1.0,
            "duration_s": 10.0, "planned_duration_s": 10.0,
            "pacer_overruns": 2, "servo_j_ms": {"median": 2.0, "n": 4 * N},
            "read_state_ms": {"median": 3.0}, "bad_frame_ratio": 0.0}
    meta.update(extra)
    return meta


def _levels(verdicts):
    return [level for level, _text in verdicts]


def test_healthy_episode_has_no_warnings():
    cmd = _command()
    figures, verdicts = metrics.episode_quality(_arrays(cmd, cmd), _meta())
    assert "warnung" not in _levels(verdicts) and "fehler" not in _levels(verdicts)
    assert figures["follow_median_rad"] < 1e-6
    assert abs(figures["accel_ratio_p95"] - 1.0) < 1e-6


def test_lagging_arm_is_reported_with_the_override_hint():
    cmd = _command()
    lagging = np.vstack([np.repeat(cmd[:1], 12, axis=0), cmd[:-12]])  # 0.8 s hinterher
    _figures, verdicts = metrics.episode_quality(_arrays(lagging, cmd), _meta(override=0.2))
    texts = [t for level, t in verdicts if level == "warnung"]
    assert any("hinterher" in t and "Planer" in t for t in texts), texts


def test_stop_and_go_is_reported_as_jerk():
    cmd = _command()
    # Arm haelt jeden zweiten Takt an und holt dann auf (verspaetete Befehle)
    jerky = cmd.copy()
    jerky[1::2] = cmd[0:-1:2]
    figures, verdicts = metrics.episode_quality(
        _arrays(jerky, cmd), _meta(pacer_overruns=300))
    assert figures["accel_ratio_p95"] > metrics.QUALITY_ACCEL_RATIO_WARN
    texts = [t for level, t in verdicts if level == "warnung"]
    assert any("Ruckeln" in t for t in texts)
    assert any("Takt nicht gehalten" in t for t in texts)


def test_discarded_episode_is_an_error():
    cmd = _command()
    _figures, verdicts = metrics.episode_quality(
        _arrays(cmd, cmd), _meta(), discarded=True, discard_reason="latenzbudget")
    assert ("fehler", "verworfen: latenzbudget") in verdicts


def test_slow_run_adds_one_warning_and_explanations():
    cmd = _command()
    _figures, verdicts = metrics.episode_quality(
        _arrays(cmd, cmd), _meta(duration_s=15.0, servo_j_ms={"median": 14.0, "n": 4 * N}))
    slow = [lv for lv, t in verdicts if "langsamer als geplant" in t]
    assert slow == ["warnung"]
    assert "info" in _levels(verdicts)
    assert config.RECORDER_SLOW_FACTOR_WARN < 1.5

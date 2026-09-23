"""Sollwert-Interpolation fuer servo_j (AP 1.3/4.1, Befund VM 2026-09-15)."""

import _paths  # noqa: F401

import numpy as np

import _fixtures
from bc.clock import SimClock
from bc.ports import RobotError, ServoLimitError
from bc.servo import ServoGuard, ServoInterpolator, TargetLimiter


def _raises(fn, exc=ServoLimitError):
    try:
        fn()
    except exc as err:
        return err
    assert False, "%s erwartet" % exc.__name__


def test_guard_accepts_interpolated_motion_and_rejects_jumps():
    guard = ServoGuard(max_speed_rads=1.0, max_gap_rad=0.3).reset(np.zeros(6), t=0.0)
    interp = ServoInterpolator(15.0, 60.0).reset(np.zeros(6))
    t = 0.0
    # 0.06 rad in einem 15-Hz-Takt = 0.9 rad/s -> zulaessig
    for q, _, _ in interp.window(np.full(6, 0.06)):
        t += 1.0 / 60.0
        guard.check(q, t)
    # derselbe Weg als EIN Sollwert im 60-Hz-Raster -> 3.6 rad/s
    err = _raises(lambda: guard.check(np.full(6, 0.12), t + 1.0 / 60.0))
    assert "rad/s" in str(err)
    assert guard.rejections == 1


def test_target_limiter_caps_step_keeps_direction_and_stays_below_guard():
    limiter = TargetLimiter(max_speed_rads=0.75, control_rate_hz=15.0).reset(np.zeros(6))
    target = np.array([0.2, -0.1, 0.0, 0.0, 0.0, 0.0])
    q = limiter.limit(target)
    assert abs(np.max(np.abs(q)) - 0.05) < 1e-12        # 0.75 rad/s * 1/15 s
    assert np.allclose(q / np.linalg.norm(q), target / np.linalg.norm(target))
    assert limiter.clamped == 1
    assert np.allclose(limiter.limit(q + 0.01), q + 0.01) and limiter.clamped == 1
    # Die gekappten Ziele passieren den harten Filter, interpoliert mit 60 Hz
    guard = ServoGuard(max_speed_rads=1.0).reset(np.zeros(6), t=0.0)
    limiter.reset(np.zeros(6))
    interp = ServoInterpolator(15.0, 60.0).reset(np.zeros(6))
    t = 0.0
    for _ in range(10):
        for q, _, _ in interp.window(limiter.limit(np.full(6, 3.0))):
            t += 1.0 / 60.0
            guard.check(q, t)


def test_guard_dt_is_clamped():
    # Stocken der Schleife (1 s) erlaubt keinen grossen Sprung: dt <= 1/15 s
    guard = ServoGuard(max_speed_rads=1.0).reset(np.zeros(6), t=0.0)
    _raises(lambda: guard.check(np.full(6, 0.2), 1.0))
    guard.check(np.full(6, 0.06), 1.0)
    # Zu frueher Aufruf wird nicht faelschlich abgelehnt: dt >= 1/60 s
    guard.check(np.full(6, 0.075), 1.0)


def test_guard_rejects_nan_limits_and_gap():
    guard = ServoGuard(max_gap_rad=0.3).reset(np.zeros(6), t=0.0)
    q = np.zeros(6)
    q[2] = np.nan
    _raises(lambda: guard.check(q, 0.1))
    far = ServoGuard(joint_limits=((-0.1, 0.1),) * 6).reset(np.full(6, 0.095), t=0.0)
    _raises(lambda: far.check(np.full(6, 0.105), 0.1))
    guard.check_gap(np.full(6, 0.2), np.zeros(6))
    _raises(lambda: guard.check_gap(np.full(6, 0.4), np.zeros(6)))


def test_sim_robot_refuses_jump_and_stops():
    robot = _fixtures.make_robot("sim", clock=SimClock())
    joints = robot.read_state().joints
    zeros = [0.0] * robot.dof
    robot.activate_servo("position")
    _raises(lambda: robot.servo_j(joints + 0.3, zeros, zeros))
    assert robot.stop_requested
    assert np.allclose(robot.read_state().joints, joints)  # nichts gesendet
    _raises(lambda: robot.servo_j(joints, zeros, zeros), RobotError)


def test_window_reaches_target_linearly():
    interp = ServoInterpolator(control_rate_hz=15.0, servo_rate_hz=60.0).reset(np.zeros(3))
    target = np.array([0.4, -0.2, 0.0])
    window = interp.window(target)
    assert len(window) == 4
    qs = np.array([w[0] for w in window])
    assert np.allclose(qs[-1], target)
    assert np.allclose(qs, np.outer([0.25, 0.5, 0.75, 1.0], target))
    # Geschwindigkeit: Zieldifferenz je 15-Hz-Takt, im Fenster konstant
    for _, v, a in window:
        assert np.allclose(v, target * 15.0)
        assert np.allclose(a, target * 15.0 * 15.0)  # aus dem Stand


def test_constant_velocity_has_zero_acceleration():
    interp = ServoInterpolator(15.0, 60.0).reset(np.zeros(1))
    interp.window([0.01])
    _, v, a = interp.window([0.02])[0]
    assert np.allclose(v, 0.15) and np.allclose(a, 0.0)


def test_hold_and_rate_validation():
    interp = ServoInterpolator(15.0, 15.0).reset([1.0, 2.0])
    q, v, a = interp.hold()
    assert np.allclose(q, [1.0, 2.0]) and not v.any() and not a.any()
    assert len(interp.window([1.1, 2.0])) == 1
    try:
        ServoInterpolator(15.0, 50.0)
        assert False, "ValueError erwartet (kein ganzzahliges Vielfaches)"
    except ValueError:
        pass

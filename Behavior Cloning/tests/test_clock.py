"""Host-Uhr: gemeinsame, fein aufgeloeste Zeitbasis (AP 1.3)."""

import time

import _paths  # noqa: F401

from bc.clock import RealClock, host_time


def test_host_time_is_close_to_wall_clock():
    assert abs(host_time() - time.time()) < 1.0


def test_host_time_resolves_below_sync_budget():
    # time.time() ist unter Windows/Python 3.12 nur auf 15.6 ms zugesichert
    # -- die gemeinsame Zeitbasis muss deutlich feiner sein (Befund
    # 2026-09-14).
    steps = []
    last = host_time()
    while len(steps) < 50:
        now = host_time()
        assert now >= last  # monoton
        if now != last:
            steps.append(now - last)
        last = now
    assert min(steps) < 0.001


def test_real_clock_uses_host_time():
    clock = RealClock()
    a = host_time()
    b = clock.now()
    c = host_time()
    assert a <= b <= c

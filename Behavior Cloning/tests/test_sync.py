"""Synchronisation: Taktgeber und Latenzbudget (AP 1.3)."""

import _paths  # noqa: F401

from bc import config
from bc.clock import SimClock
from bc.sync import Pacer, evaluate


def test_pacer_holds_rate():
    clock = SimClock()
    pacer = Pacer(clock, rate_hz=15.0).start()
    ticks = [pacer.tick() for _ in range(30)]
    diffs = [b - a for a, b in zip(ticks[:-1], ticks[1:])]
    assert all(abs(d - 1 / 15.0) < 1e-9 for d in diffs)
    assert pacer.overruns == 0


def test_pacer_skips_after_stall():
    clock = SimClock()
    start = clock.now()
    pacer = Pacer(clock, rate_hz=15.0).start()
    pacer.tick()
    clock.advance(0.5)  # Schleife hing eine halbe Sekunde
    t = pacer.tick()
    # Kein Aufhol-Burst: es wird der aktuelle Rasterpunkt gewaehlt (max.
    # eine Periode in der Vergangenheit), nicht alle verpassten nachgeholt
    assert t >= start + 0.5 - 1 / 15.0 - 1e-9
    assert pacer.overruns > 0
    # ... und der Folge-Tick liegt wieder in der Zukunft
    assert pacer.tick() >= start + 0.5 - 1e-9


def test_evaluate_within_budget():
    report = evaluate(1.0, {"robot": 0.995, "wrist": 0.99, "scene": 0.998})
    assert report.ok
    assert abs(report.spread - 0.008) < 1e-12


def test_evaluate_over_budget():
    report = evaluate(1.0, {"robot": 1.0, "wrist": 1.0 - config.SYNC_MAX_SKEW_S - 0.01})
    assert not report.ok
    assert report.worst_source == "wrist"


def test_evaluate_missing_source_is_not_ok():
    report = evaluate(1.0, {"robot": 1.0, "wrist": None})
    assert not report.ok

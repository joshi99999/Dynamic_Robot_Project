"""tools/report_zittern.py (Handy-Auswertung): Abschnitte nach phyphox-Pausen.

Hardwarefrei. Befund Labortag 2026-10-16: phyphox zaehlt die Experimentzeit
ueber PAUSE/START ohne Luecke weiter; die Uhrzeiten der Fahrten nach einer
Pause waren deshalb falsch, und Filter liefen ueber die Pausengrenze.
"""

import sys
from datetime import datetime
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import report_zittern as rz  # noqa: E402

META = [
    ["event", "experiment time", "system time", "system time text"],
    ["START", 0.0, 1000.0, ""],
    ["PAUSE", 30.0, 1030.0, ""],
    ["START", 30.0, 1600.0, ""],      # 9:30 min Pause
    ["PAUSE", 60.0, 1630.0, ""],
]


def test_zeitabschnitte_paart_start_und_pause():
    ab = rz.zeitabschnitte(META, 60.0)
    assert ab == [(0.0, 30.0, 1000.0), (30.0, 60.0, 1600.0)]
    assert rz.systemzeit(10.0, ab) == 1010.0
    assert rz.systemzeit(45.0, ab) == 1615.0
    # laufende Messung ohne abschliessende PAUSE: bis zum letzten Wert
    assert rz.zeitabschnitte(META[:4], 50.0)[-1] == (30.0, 50.0, 1600.0)
    # ohne Metadaten: ein Abschnitt ohne Uhrzeit
    assert rz.zeitabschnitte(None, 12.0) == [(0.0, 12.0, None)]


def _messung(fs=200, seed=0):
    """Zwei Abschnitte a 30 s, je eine Fahrt (Lagewechsel + 8-Hz-Zittern)."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, 60.0, 1.0 / fs)
    a = np.tile([0.0, 0.0, 9.81], (len(t), 1)) + rng.normal(0, 0.02, (len(t), 3))
    for t0 in (10.0, 40.0):                    # 10 s in Abschnitt 0, 10 s in Abschnitt 1
        m = (t >= t0) & (t < t0 + 3.0)
        phase = (t[m] - t0) / 3.0
        a[m, 0] += 3.0 * np.sin(np.pi * phase)              # langsame Lageaenderung
        a[m, 1] += 0.5 * np.sin(2 * np.pi * 8.0 * t[m])     # Zittern
    return t, a


def test_fahrten_bekommen_die_uhrzeit_ihres_abschnitts():
    t, a = _messung()
    h = rz.handy_auswerten_daten(t, a, rz.zeitabschnitte(META, t[-1]))
    assert len(h["fahrten"]) == 2
    f0, f1 = h["fahrten"]
    assert f0["abschnitt"] == 0 and f1["abschnitt"] == 1
    # Uhrzeit = START des Abschnitts + Abstand in der Experimentzeit
    assert abs(f0["uhr0"] - (1000.0 + f0["t0"])) < 1e-9
    assert abs(f1["uhr0"] - (1600.0 + f1["t0"] - 30.0)) < 1e-9
    assert abs(f1["uhr0"] - 1610.0) < 1.0          # vorher: 1040 (erster START)
    assert f1["beginn"] == datetime.fromtimestamp(f1["uhr0"]).strftime("%H:%M:%S")
    for f in (f0, f1):
        assert 7.0 <= f["spitze_hz"] <= 9.0
        assert f["rms"] > 5 * h["boden_rms"]


def test_keine_fahrt_ueber_die_pausengrenze():
    # Bewegung genau an der Grenze: zwei getrennte Stuecke, nie eines ueber die Pause
    t, a = _messung()
    m = (t >= 28.0) & (t < 32.0)
    a[m, 0] += 3.0
    h = rz.handy_auswerten_daten(t, a, rz.zeitabschnitte(META, t[-1]))
    for f in h["fahrten"]:
        e0, e1, _ = h["abschnitte"][f["abschnitt"]]
        assert e0 <= f["t0"] and f["t1"] <= e1

"""tools/check_trajectory_buffer.py: Testbahn, Bahnpruefung und Auswertung.

Hardwarefrei. Die Auswertung muss die beiden Faelle sicher trennen, um die
es am 2026-10-05 ging: glatte Fahrt gegen Stop-and-go im servo_j-Takt.
"""

import sys
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import check_trajectory_buffer as tool  # noqa: E402

Q0 = np.array([0.1, -0.3, 1.5, 0.0, 0.8, 0.0])


def _bahn(amplitude=0.05, dauer=4.0):
    return tool.Testbahn(Q0, 0, amplitude, dauer)


def _mitschrieb(lage, dauer=5.0, hz=250.0, jitter_s=0.0, seed=0):
    """Gelenkwinkel wie aus dem Lese-Thread: unregelmaessige Zeitstempel."""
    rng = np.random.default_rng(seed)
    t = np.arange(-0.3, dauer, 1.0 / hz)
    t = np.sort(t + rng.uniform(-jitter_s, jitter_s, size=t.size))
    q = np.tile(Q0, (t.size, 1))
    q[:, 0] = lage(t)
    return t, q


def test_testbahn_starts_and_ends_at_rest_in_q0():
    bahn = _bahn()
    assert abs(float(bahn.qj(0.0)) - Q0[0]) < 1e-12
    assert abs(float(bahn.qj(bahn.dauer)) - Q0[0]) < 1e-12
    assert abs(float(bahn.vj(0.0))) < 1e-12
    assert abs(float(bahn.vj(bahn.dauer))) < 1e-9
    assert abs(float(bahn.qj(bahn.dauer / 2)) - (Q0[0] + 0.05)) < 1e-12
    assert abs(bahn.v_max - 0.05 * np.pi / 4.0) < 1e-12


def test_chunks_reproduce_the_whole_path_without_gaps():
    bahn = _bahn()
    ganz_rel, ganz = bahn.knoten(0.0, bahn.dauer, 100.0)
    teile = tool.abschnitte(bahn.dauer, 4 / 15.0)
    assert abs(teile[0][0]) < 1e-12 and abs(teile[-1][1] - bahn.dauer) < 1e-12
    for (a_von, a_bis), (b_von, _b_bis) in zip(teile, teile[1:]):
        assert abs(a_bis - b_von) < 1e-12
    zeiten, punkte = [], []
    for von, bis in teile:
        rel, p = bahn.knoten(von, bis, 100.0)
        assert rel[0] > 0 and abs(rel[-1] - (bis - von)) < 1e-12
        zeiten += list(von + rel)
        punkte += p
    assert abs(zeiten[-1] - bahn.dauer) < 1e-9
    assert abs(punkte[-1][0] - ganz[-1][0]) < 1e-12
    assert np.all(np.diff(zeiten) > 0)


def test_path_check_accepts_the_test_path():
    bahn = _bahn()
    rel, punkte = bahn.knoten(0.0, bahn.dauer, 100.0)
    tool.pruefe_bahn(Q0, rel, punkte, Q0, bahn.amplitude)


def _wirft(fn):
    try:
        fn()
    except ValueError as exc:
        return str(exc)
    raise AssertionError("keine Ablehnung")


def test_path_check_rejects_leaving_the_window():
    bahn = _bahn()
    rel, punkte = bahn.knoten(0.0, bahn.dauer, 100.0)
    punkte[10] = punkte[10] + np.array([0, 0, 0.2, 0, 0, 0])
    assert "Startstellung" in _wirft(lambda: tool.pruefe_bahn(Q0, rel, punkte, Q0, 0.05))


def test_path_check_rejects_a_jump_from_the_current_pose():
    bahn = _bahn()
    rel, punkte = bahn.knoten(0.0, bahn.dauer, 100.0)
    weit = Q0 + np.array([0.04, 0, 0, 0, 0, 0])   # im Fenster, aber 4 rad/s
    assert "rad/s" in _wirft(lambda: tool.pruefe_bahn(weit, rel, punkte, Q0, 0.05))


def test_path_check_rejects_unsorted_timestamps():
    bahn = _bahn()
    rel, punkte = bahn.knoten(0.0, bahn.dauer, 100.0)
    rel = rel.copy()
    rel[5] = rel[4]
    assert "steigend" in _wirft(lambda: tool.pruefe_bahn(Q0, rel, punkte, Q0, 0.05))


def test_smooth_motion_has_low_ripple_and_the_right_lag():
    bahn = _bahn()
    t, q = _mitschrieb(lambda t: bahn.qj(t - 0.030), jitter_s=0.0005)
    k = tool.auswerten(t, q, 0.0, bahn)
    assert abs(k["verzug_ms"] - 30.0) <= 4.0
    assert k["welligkeit_prozent"] < 5.0
    assert k["stillstand_prozent"] < 1.0


def test_stop_and_go_is_detected_as_ripple_and_standstill():
    # servo_j mit 60 Hz, die Steuerung faehrt jeden Sollwert in 4 ms an und
    # wartet dann -- das Bild aus der VM-Messung vom 2026-09-15.
    bahn = _bahn()

    def treppe(t):
        k = np.floor(t * 60.0)
        anfang = bahn.qj(k / 60.0)
        ziel = bahn.qj((k + 1) / 60.0)
        anteil = np.clip((t - k / 60.0) / 0.004, 0.0, 1.0)
        return anfang + anteil * (ziel - anfang)

    t, q = _mitschrieb(treppe, jitter_s=0.0005)
    k = tool.auswerten(t, q, 0.0, bahn)
    glatt = tool.auswerten(*_mitschrieb(lambda t: bahn.qj(t), jitter_s=0.0005), t_start=0.0, bahn=bahn)
    assert k["welligkeit_prozent"] > 5 * glatt["welligkeit_prozent"]
    assert k["stillstand_prozent"] > 30.0
    assert 50.0 <= k["spitze_hz"] <= 70.0


def test_standstill_survives_inaccurate_timestamps():
    # Der Zeitstempel ist nur die Mitte eines ~3,5-ms-Aufrufs: +-2 ms Fehler.
    # Die Welligkeit bekommt davon einen Rauschboden, der Stillstand nicht --
    # deshalb ist er das Hauptkriterium im Fazit.
    bahn = _bahn()
    rng = np.random.default_rng(1)

    def treppe(t):
        k = np.floor(t * 60.0)
        anfang, ziel = bahn.qj(k / 60.0), bahn.qj((k + 1) / 60.0)
        return anfang + np.clip((t - k / 60.0) / 0.004, 0.0, 1.0) * (ziel - anfang)

    ergebnis = {}
    for name, lage in (("glatt", lambda t: bahn.qj(t)), ("treppe", treppe)):
        t, q = _mitschrieb(lage)
        t_meld = t + rng.uniform(-0.002, 0.002, size=t.size)
        o = np.argsort(t_meld)
        ergebnis[name] = tool.auswerten(t_meld[o], q[o], 0.0, bahn)
    assert ergebnis["glatt"]["stillstand_prozent"] < 1.0
    assert ergebnis["treppe"]["stillstand_prozent"] > 30.0
    assert ergebnis["glatt"]["welligkeit_prozent"] < 20.0
    assert not ergebnis["glatt"]["spitze_aussagekraeftig"]
    assert ergebnis["treppe"]["spitze_aussagekraeftig"]


def test_standstill_at_a_chunk_boundary_is_reported():
    bahn = _bahn()
    grenze = 1.0   # in voller Fahrt (Spitze bei T/4)

    def mit_halt(t):
        # 150 ms Halt an der Grenze, danach dieselbe Bahn verspaetet
        return np.where(t < grenze, bahn.qj(t),
                        np.where(t < grenze + 0.15, bahn.qj(grenze), bahn.qj(t - 0.15)))

    t, q = _mitschrieb(mit_halt, dauer=5.5)
    k = tool.auswerten(t, q, 0.0, bahn, grenzen=[grenze])
    assert k["anschluss_min_verhaeltnis"] < 0.3

    t, q = _mitschrieb(lambda t: bahn.qj(t))
    k = tool.auswerten(t, q, 0.0, bahn, grenzen=[grenze])
    assert k["anschluss_min_verhaeltnis"] > 0.9


def test_stretched_run_and_slow_log_are_not_interpreted():
    # VM 2026-10-08: Sollwerte kamen zu spaet (4-s-Bahn in ~4,9 s), der
    # Mitschrieb lief nur mit 53 Hz. Beides muss als "nicht deutbar"
    # auffallen, statt 600 ms Verzug und 26 % Stillstand zu melden.
    bahn = _bahn()
    t, q = _mitschrieb(lambda t: bahn.qj(t / 1.25), dauer=6.0, hz=53.0)
    k = tool.auswerten(t, q, 0.0, bahn)
    assert 1.2 <= k["dauer_faktor"] <= 1.3
    gruende = tool.aussagekraft(k)
    assert any("langsamer" in g for g in gruende)
    assert any("Mitschrieb" in g for g in gruende)

    t, q = _mitschrieb(lambda t: bahn.qj(t - 0.03))
    k = tool.auswerten(t, q, 0.0, bahn)
    assert abs(k["dauer_faktor"] - 1.0) < 0.02
    assert tool.aussagekraft(k) == []


def test_slow_controller_state_is_not_read_as_standstill():
    # VM 2026-10-12: 573 Hz gelesen, die Steuerung liefert aber nur alle
    # 20 ms neue Gelenkwerte. Der Arm faehrt glatt -- die "91 % Stillstand"
    # sind Wiederholungen und duerfen nicht als Stop-and-go gelten.
    bahn = _bahn()
    t, q = _mitschrieb(lambda t: bahn.qj(np.floor(t * 50.0) / 50.0), hz=573.0)
    k = tool.auswerten(t, q, 0.0, bahn)
    assert k["stillstand_prozent"] > 80.0
    assert 45.0 <= k["neue_werte_hz"] <= 55.0
    gruende = tool.aussagekraft(k, servo_hz=60.0)
    assert any("neue Gelenkwerte" in g for g in gruende)


def test_real_stop_and_go_stays_interpretable():
    # Gegenprobe: echte 60-Hz-Treppe, Steuerung liefert jeden Wert. Dann
    # gibt es mindestens einen neuen Wert je Stufe -- deutbar.
    bahn = _bahn()

    def treppe(t):
        k = np.floor(t * 60.0)
        anfang, ziel = bahn.qj(k / 60.0), bahn.qj((k + 1) / 60.0)
        return anfang + np.clip((t - k / 60.0) / 0.004, 0.0, 1.0) * (ziel - anfang)

    k = tool.auswerten(*_mitschrieb(treppe, jitter_s=0.0005), t_start=0.0, bahn=bahn)
    assert k["stillstand_prozent"] > 30.0
    assert k["neue_werte_hz"] >= 0.9 * 60.0
    assert tool.aussagekraft(k, servo_hz=60.0) == []


def test_new_value_metrics_see_smooth_motion_behind_a_slow_source():
    # Dieselbe VM-Lage: glatt, aber nur 50 neue Werte je Sekunde. Auf die
    # neuen Werte reduziert muss die Fahrt glatt aussehen.
    bahn = _bahn()
    t, q = _mitschrieb(lambda t: bahn.qj(np.floor(t * 50.0) / 50.0), hz=573.0)
    k = tool.auswerten(t, q, 0.0, bahn)
    assert k["einbruch_neu_prozent"] < 10.0
    assert k["tempo_p5_anteil"] > 0.7


def test_new_value_metrics_see_a_15hz_pulse_behind_a_slow_source():
    # movelinear_online, das jedes 15-Hz-Ziel zu frueh erreicht und 30 ms
    # steht -- durch dieselbe 50-Hz-Quelle betrachtet. Muss auffallen.
    bahn = _bahn()
    takt = 1.0 / 15.0

    def puls(t):
        k = np.floor(t / takt)
        anteil = np.clip((t - k * takt) / (takt - 0.030), 0.0, 1.0)
        return bahn.qj(k * takt) + anteil * (bahn.qj((k + 1) * takt) - bahn.qj(k * takt))

    t, q = _mitschrieb(lambda t: puls(np.floor(t * 50.0) / 50.0), hz=573.0)
    k = tool.auswerten(t, q, 0.0, bahn)
    glatt = tool.auswerten(*_mitschrieb(lambda t: bahn.qj(np.floor(t * 50.0) / 50.0),
                                        hz=573.0), t_start=0.0, bahn=bahn)
    assert k["einbruch_neu_prozent"] > 5.0
    assert k["tempo_p5_anteil"] < 0.5
    assert k["welligkeit_neu_prozent"] > 3 * glatt["welligkeit_neu_prozent"]
    assert tool.aussagekraft(k, servo_hz=15.0) == []


def test_gap_in_the_log_is_visible():
    bahn = _bahn()
    t, q = _mitschrieb(lambda t: bahn.qj(t))
    behalten = (t < 1.0) | (t > 1.5)
    k = tool.auswerten(t[behalten], q[behalten], 0.0, bahn)
    assert k["groesste_luecke_ms"] > 450


def test_too_few_samples_is_a_finding_not_a_crash():
    bahn = _bahn()
    k = tool.auswerten(np.arange(5) * 0.01, np.tile(Q0, (5, 1)), 0.0, bahn)
    assert "ungueltig" in k

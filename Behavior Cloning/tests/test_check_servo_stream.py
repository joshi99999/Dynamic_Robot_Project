"""tools/check_servo_stream.py: Sollwert-Profile und Auswertung.

Hardwarefrei. Die Auswertung muss glatte Fahrt, Stop-and-go laengs der Bahn
und Zacken quer zur Bahn auseinanderhalten -- darum geht es beim Vergleich
der Stromarten (Befund VM 2026-10-14).
"""

import sys
from pathlib import Path

import _paths  # noqa: F401

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import check_servo_stream as tool  # noqa: E402
from check_trajectory_buffer import sendetakt  # noqa: E402

from bc.servo import ServoInterpolator  # noqa: E402


def _knoten(n=40, seed=0):
    rng = np.random.default_rng(seed)
    q = np.cumsum(rng.normal(0.0, 0.01, size=(n, 6)), axis=0)
    q[:3] = q[3]
    q[-3:] = q[-4]
    return q


def test_linear_entspricht_dem_recorder():
    q = _knoten()
    prof = tool.Profil(q, "linear")
    interp = ServoInterpolator(15.0, 60.0).reset(q[0])
    k = 0
    q0, v0, a0 = prof.at(0.0)
    assert np.allclose(q0, interp.hold()[0])
    for i in range(1, len(q)):
        for qc, vc, ac in interp.window(q[i]):
            k += 1
            qp, vp, ap = prof.at(k / 60.0)
            if k / 60.0 >= prof.dauer - 1e-12:
                assert np.allclose(qp, qc)
                continue
            assert np.allclose(qp, qc) and np.allclose(vp, vc) and np.allclose(ap, ac)


def test_a0_und_pos():
    q = _knoten()
    _, v, a = tool.Profil(q, "a0").at(0.5)
    assert np.any(v != 0) and np.all(a == 0)
    _, v, a = tool.Profil(q, "pos").at(0.5)
    assert np.all(v == 0) and np.all(a == 0)


def test_spline_trifft_ziele_und_ist_stetig():
    q = _knoten()
    prof = tool.Profil(q, "spline")
    T = prof.T
    for i in range(1, len(q) - 1):
        assert np.allclose(prof.at(i * T)[0], q[i])
        links = prof.at(i * T - 1e-7)[1]
        rechts = prof.at(i * T + 1e-7)[1]
        assert np.allclose(links, rechts, atol=1e-4)
    # v ist die Ableitung von q
    t = 0.71
    dq = (prof.at(t + 1e-6)[0] - prof.at(t - 1e-6)[0]) / 2e-6
    assert np.allclose(dq, prof.at(t)[1], atol=1e-5)
    dv = (prof.at(t + 1e-6)[1] - prof.at(t - 1e-6)[1]) / 2e-6
    assert np.allclose(dv, prof.at(t)[2], atol=1e-3)


def test_sendezeiten_enden_am_letzten_ziel():
    prof = tool.Profil(_knoten(n=31), "linear")   # 2 s
    for rate in (30.0, 50.0, 60.0, 90.0, 120.0, 47.0):
        ts = prof.sendezeiten(rate)
        assert ts[0] == 0.0 and abs(ts[-1] - prof.dauer) < 1e-9
        assert np.all(np.diff(ts) > 0)


def test_parse_lauf():
    l = tool.parse_lauf("spline-120-r0.5-o0.25")
    assert l == {"name": "spline-120-r0.5-o0.25", "profil": "spline", "rate": 120.0,
                 "rauschen": 0.5, "override": 0.25}
    for falsch in ("spline-120-r0.5", "kubisch-60-r0-o1", "linear-60-r2-o1", "linear-1000-r0-o1"):
        try:
            tool.parse_lauf(falsch)
        except ValueError:
            continue
        raise AssertionError("nicht abgelehnt: %s" % falsch)


def test_standardmatrix_ist_gueltig():
    class A:
        laeufe = ""
        matrix = "standard"
    laeufe = tool.laeufe_aus(A)
    assert len(laeufe) == sum(len(v) for v in tool.MATRIX.values())
    assert len({l["name"] for l in laeufe}) == len(laeufe)  # Wiederholung umbenannt
    assert laeufe[-1]["name"] == "linear-60-r0-o1#2"


def test_raster_zeiten_entfernt_lesejitter():
    rng = np.random.default_rng(1)
    wahr = 5.0 + 0.02 * np.arange(300)
    gelesen = wahr + rng.uniform(0.0, 0.004, size=300)   # erst nach bis zu 4 ms gesehen
    t, info = tool.raster_zeiten(gelesen)
    assert info["raster"]
    assert np.max(np.abs(np.diff(t) - 0.02)) < 2e-5


def test_raster_zeiten_lehnt_ab_ohne_raster():
    rng = np.random.default_rng(2)
    t_in = np.cumsum(rng.uniform(0.005, 0.035, size=200))
    t, info = tool.raster_zeiten(t_in)
    assert not info["raster"] and np.array_equal(t, t_in)


def _fahrt(stop_and_go=False, zacken_mm=0.0, hz=50.0, dauer=4.0):
    """Gerade Fahrt in x mit 0,1 m/s, optional mit 60-Hz-Halten bzw. Querzacken."""
    t = np.arange(0.0, dauer, 1.0 / hz)
    x = 0.1 * t
    if stop_and_go:
        # Arm faehrt jeden 60-Hz-Schritt in 4 ms an und steht dann
        stufe = np.floor(t * 60.0) / 60.0
        x = 0.1 * (stufe + np.minimum(t - stufe, 0.004) / 0.004 / 60.0)
    y = zacken_mm * 1e-3 * np.where(np.sin(2 * np.pi * 7.0 * t + 0.1) >= 0, 1.0, -1.0)
    return t, np.stack([x, y, np.zeros_like(t)], 1)


def test_tempo_kennzahlen_trennen_glatt_und_stop_and_go():
    t, p = _fahrt()
    glatt = tool.tempo_kennzahlen(t, p)
    t2, p2 = _fahrt(stop_and_go=True, hz=250.0)
    treppe = tool.tempo_kennzahlen(t2, p2)
    assert glatt["tempo_welligkeit_prozent"] < 1.0
    assert treppe["tempo_welligkeit_prozent"] > 50.0
    assert treppe["stillstand_prozent"] > 30.0


def test_querabweichung_sieht_zacken_aber_nicht_langsfehler():
    t, p = _fahrt(zacken_mm=0.5)
    ref = np.stack([np.linspace(-0.01, 0.5, 2000), np.zeros(2000), np.zeros(2000)], 1)
    dev = tool.querabweichung(p, ref)
    assert np.allclose(np.abs(dev[:, 1]), 0.5e-3, atol=1e-9)
    assert np.max(np.abs(dev[:, 0])) < 1e-6   # Laengsfehler zaehlt nicht
    # Stop-and-go laengs ist quer unsichtbar
    _, p2 = _fahrt(stop_and_go=True, hz=250.0)
    assert np.max(np.linalg.norm(tool.querabweichung(p2, ref), axis=1)) < 1e-6


def test_auswerten_lauf_findet_verzug():
    q = _knoten(n=46)
    prof = tool.Profil(q, "linear")
    t = np.arange(-0.3, prof.dauer + 0.5, 0.02)
    verzug = 0.06
    ist = prof.positionen(t - verzug)

    def fk_pos(qs):
        return np.asarray(qs)[:, :3]

    send = np.stack([np.arange(0, prof.dauer, 1 / 60.0)] * 3, 1)
    k = tool.auswerten_lauf(send, t, ist, prof, 60.0, fk_pos, 0.0)
    assert abs(k["verzug_ms"] - 60.0) <= 4.0
    assert k["quer"]["rms_mm"] < 0.05
    assert k["sendetakt"]["abstand_std_ms"] < 1e-6


def _sendungen_mit_haenger(prof, haenger_s=0.2, bei=60):
    """60-Hz-Sendungen (t_soll, Beginn, Ende), Aufruf ``bei`` haengt ``haenger_s``.

    Wie check_servo_stream.fahre_lauf: danach geht das naechste
    Profilstueck raus, der Rest der Bahn also ``haenger_s`` spaeter.
    """
    k = np.arange(len(prof.sendezeiten(60.0)))
    beginn = k / 60.0 + haenger_s * (k > bei)
    ende = k / 60.0 + haenger_s * (k >= bei)
    return np.stack([beginn, beginn, ende], 1)


def test_haenger_zaehlt_nicht_als_verzug():
    """Befund PC 2026-10-19: gegen die Uhr gerechnet erschien jeder Haenger
    als bleibender Nachlauf des Arms (Override 0,5: scheinbar bis 0,9 s)."""
    q = _knoten(n=46)
    prof = tool.Profil(q, "linear")
    send = _sendungen_mit_haenger(prof)
    t = np.arange(-0.3, prof.dauer + 0.8, 0.02)
    # Profilzeit des gesendeten Stroms: bis Sendung 59 = Uhr, waehrend des
    # Haengers fast stehend, danach Uhr - 0,2 s. Arm folgt mit 60 ms.
    t59 = 59 / 60.0
    tp = np.where(t < t59, t, np.where(t < 1.2, t59 + (t - t59) * (1 / 60.0) / (1.2 - t59), t - 0.2))
    ist = prof.positionen(tp - 0.06)

    def fk_pos(qs):
        return np.asarray(qs)[:, :3]

    k = tool.auswerten_lauf(send, t, ist, prof, 60.0, fk_pos, 0.0)
    assert abs(k["verzug_ms"] - 60.0) <= 4.0
    assert np.allclose(tool.profilzeit(t, send, prof, 60.0), tp, atol=1e-9)


def test_hostzeit_ist_die_umkehrung_von_profilzeit():
    prof = tool.Profil(_knoten(n=46), "linear")
    send = _sendungen_mit_haenger(prof)
    tp = np.arange(0, 46) / 15.0
    t = tool.hostzeit(tp, send, prof, 60.0)
    assert np.allclose(tool.profilzeit(t, send, prof, 60.0), tp, atol=1e-9)
    assert np.allclose(t[tp < 59 / 60.0], tp[tp < 59 / 60.0])
    assert np.allclose(t[tp >= 1.0], tp[tp >= 1.0] + 0.2)


def test_sendetakt_sieht_jitter_ohne_ueberlauf():
    rng = np.random.default_rng(3)
    soll = np.arange(600) / 60.0
    beginn = soll + rng.uniform(0.0, 0.006, size=600)
    k = sendetakt(np.stack([soll, beginn, beginn + 0.003], 1), 60.0, soll[-1])
    assert 1.0 < k["abstand_std_ms"] < 4.0
    assert abs(k["aufruf_median_ms"] - 3.0) < 1e-6
    assert abs(k["dauer_faktor"] - 1.0) < 0.01


def test_online_nur_mit_einem_ziel_je_takt():
    l = tool.parse_lauf("online-15-r1-o1")
    assert l["profil"] == "online" and l["rate"] == 15.0
    try:
        tool.parse_lauf("online-60-r0-o1")
    except ValueError:
        return
    raise AssertionError("online mit 60 Hz nicht abgelehnt")


def test_onlinehalt_ist_eine_online_art():
    l = tool.parse_lauf("onlinehalt-15-r0-o1")
    assert l["profil"] == "onlinehalt"
    try:
        tool.parse_lauf("onlinehalt-60-r0-o1")
    except ValueError:
        return
    raise AssertionError("onlinehalt mit 60 Hz nicht abgelehnt")


# -- Haltepunkte (Anlage 2026-10-16: online kehrte 27 mm ueber PICK um) -------


def _fk_identitaet(qs):
    """Testkette: die ersten drei "Gelenke" sind die TCP-Position in m."""
    return np.atleast_2d(np.asarray(qs, dtype=float))[:, :3]


def _abstieg(tiefster_mm, stand_s, takt_s=1.0 / 15.0, lese_hz=100.0):
    """Lesungen: aus 50 mm Hoehe auf ``tiefster_mm`` ueber PICK, dort
    ``stand_s`` stehen, wieder hoch. Haltepunkt im Plan bei Takt 30-37."""
    pick = np.array([0.4, 0.0, 0.1])
    t = np.arange(0.0, 6.0, 1.0 / lese_hz)
    z = np.empty_like(t)
    t_ab, t_auf = 2.0, 2.0 + stand_s
    for k, tk in enumerate(t):
        if tk < t_ab:
            z[k] = 0.05 - (0.05 - 1e-3 * tiefster_mm) * min(1.0, tk / t_ab)
        elif tk <= t_auf:
            z[k] = 1e-3 * tiefster_mm
        else:
            z[k] = 1e-3 * tiefster_mm + 0.02 * (tk - t_auf)
    q = np.zeros((len(t), 6))
    q[:, :3] = pick
    q[:, 2] += z
    ziele = np.zeros((60, 6))
    ziele[:, :3] = pick
    return t, q, ziele, [(30, 37)], takt_s


def test_halt_kennzahlen_sehen_das_umkehren_ueber_pick():
    t, q, ziele, halte, takt_s = _abstieg(27.0, 0.0)
    e, = tool.halt_kennzahlen(t, q, ziele, halte, _fk_identitaet, takt_s=takt_s)
    assert abs(e["tief_mm"] - 27.0) < 0.1
    assert abs(e["min_mm"] - 27.0) < 0.1 and e["quer_tief_mm"] < 1e-6
    assert e["verweil_s"] == 0.0
    assert e["unten_s"] < 0.1                  # kein Halt, nur Umkehr


def test_halt_kennzahlen_messen_die_verweilzeit_am_haltepunkt():
    t, q, ziele, halte, takt_s = _abstieg(0.2, 0.5)
    e, = tool.halt_kennzahlen(t, q, ziele, halte, _fk_identitaet, takt_s=takt_s)
    assert abs(e["tief_mm"] - 0.2) < 0.01
    assert abs(e["verweil_s"] - 0.5) < 0.1     # + Ein-/Auslauf innerhalb 1 mm
    assert abs(e["unten_s"] - 0.5) < 0.05
    zeile, = tool.halt_zeilen("onlinehalt-15-r0-o1", [e])
    assert "+0.2 mm" in zeile and "Takt 30" in zeile


def test_halt_kennzahlen_verschieben_das_fenster_um_die_wartezeit():
    # Ein Warten von 10 s an einem frueheren Haltepunkt schiebt alles spaeter
    t, q, ziele, halte, takt_s = _abstieg(0.2, 0.5)
    warte = [{"step": 5, "wait_s": 10.0, "reached": True, "start_mm": 3.0,
              "dist_mm": 0.9, "min_mm": 0.9, "reason": ""}]
    e = tool.halt_kennzahlen(t + 10.0, q, ziele, [(5, 6)] + halte, _fk_identitaet,
                             warte=warte, takt_s=takt_s)
    assert e[0]["warten"]["wait_s"] == 10.0
    assert abs(e[1]["tief_mm"] - 0.2) < 0.01
    assert "gewartet 10.00 s" in tool.halt_zeilen("x", e)[0]


def test_halte_aus_zielen_findet_die_haltepunkte_des_planers():
    """Fuer Berichte vor 2026-10-16 (ohne Greiferspur im Mitschrieb)."""
    from bc import execution
    from bc.adapters.sim_robot import SimRobot
    from bc.clock import SimClock
    from bc.collision import default_workspace
    from bc.kinematics import Kinematics
    from bc.noise import generate_plan
    from bc.trajectory import Waypoint, build_ideal_trajectory

    robot = SimRobot(clock=SimClock()).connect()
    home = robot.read_state().joints
    start = robot.fk(home)

    def wp(dx, dz, g=False, approach=False):
        pose = np.asarray(start, dtype=float).copy()
        pose[0] += dx
        pose[2] += dz
        return Waypoint(pose, gripper_closed=g, approach=approach)

    ideal = build_ideal_trajectory([wp(0, 0), wp(0.05, -0.02), wp(0.05, -0.04, g=True, approach=True),
                                    wp(0.05, -0.01, g=True), wp(0.0, -0.01, g=False)])
    plan = generate_plan(Kinematics(robot), default_workspace(table_height_m=float(start[2]) - 0.3),
                         ideal, home, rng=np.random.default_rng(0))
    soll = execution.hold_blocks(plan.gripper, plan.dwell_mask)
    assert len(soll) == 2
    assert tool.halte_aus_zielen(plan.joints_noisy) == soll


def test_halte_aus_zielen_mit_laengerem_dwell_als_heute():
    """Test 3 am 2026-10-16 lief mit 1,5 s Dwell (23 Takte) -- der Haltepunkt
    beginnt trotzdem im ersten Takt nach der Ankunft, nicht 8 Takte vor
    dem Ende des Stillstands."""
    from bc import config

    ziele = np.cumsum(np.full((60, 6), 0.01), axis=0)
    ankunft = 20
    ziele[ankunft:ankunft + 24] = ziele[ankunft]      # Ankunft + 23 Dwell-Takte
    ziele[ankunft + 24:] += 0.0
    halte = tool.halte_aus_zielen(ziele)
    assert config.GRIPPER_DWELL_STEPS < 23
    assert halte == [(ankunft + 1, ankunft + 23)]

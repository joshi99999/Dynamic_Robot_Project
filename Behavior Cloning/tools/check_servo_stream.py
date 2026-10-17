"""Wie setzt die Steuerung unseren servo_j-Strom um? Ganze Aufnahmebahn (AP 1.3 / 4.1)

Anlass: Zittern an der Anlage (Dokumentation/Zittern-an-der-Anlage.md) und
die Auswertung vom 2026-10-14: Auch in der VM am PC -- ohne Mechanik und mit
sauberem Takt -- liegt die gefahrene Bahn zackig neben der befohlenen
(Override 1, Rauschen 0: quer RMS 0,47 mm), bei Override 0,5 nicht. Die
Anregung entsteht also schon in der Umsetzung des Sollwertstroms. Dieses
Werkzeug faehrt die ECHTE Aufnahmebahn (Ablaufdatei, Planer, Rauschen wie
``apps/record.py``) mit verschiedenen Arten, den Strom zu senden, und misst,
welche die Steuerung am glattesten umsetzt.

Je Lauf einstellbar:

* Profil zwischen den 15-Hz-Zielen (``PROFILE``):
  ``linear``  wie heute im Recorder (bc/servo.py): Position linear,
              v konstant je Takt, a = Sprung von v, ueber den ganzen Takt
              wiederholt (unstimmig: die Positionen sagen a = 0);
  ``a0``      wie linear, aber a = 0;
  ``pos``     nur Position, v = a = 0;
  ``spline``  kubische Hermite-Kurve (Catmull-Rom) durch die Ziele --
              q, v, a passen zueinander, v ist stetig. Braucht das
              NAECHSTE Ziel (in der Inferenz aus dem Chunk verfuegbar).
  ``online``  zum Vergleich: ``movelinear_online`` wie im Recorder (ein Ziel
              je 15-Hz-Takt, Vorlauf und Hoechstwerte aus bc/execution.py);
              Rate muss 15 sein. Nur an der Anlage -- die VM faehrt es nicht.
              OHNE die Halteregel (wie vor dem 2026-10-16).
  ``onlinehalt`` wie ``online``, mit "erst weiter, wenn erreicht" wie im
              Recorder: am Greiferwechsel bleibt das Ziel stehen, bis der
              TCP innerhalb ONLINE_REACH_TOL_MM steht, dann der volle Dwell
              (bc/execution.py). Der Greifer schaltet trotzdem nicht.
* Senderate in Hz -- frei, auch 50 Hz (kein Vielfaches von 15): der
  Sollwert wird zur Sendezeit aus dem Profil gerechnet.
* Rauschfaktor (fest, gleicher Seed: gleiche Rauschform, nur skaliert).
* Override.

Gemessen (Mitschrieb in eigenem Thread, nur NEUE Werte der Steuerung):

* Sendetakt: Streuung des Abstands zwischen zwei Aufrufen, Verspaetung
  gegen den Pacer, Aufrufdauer, Streckung (``check_trajectory_buffer.sendetakt``).
* Quer: Abstand der gefahrenen TCP-Bahn zur befohlenen Bahn (naechster
  Punkt, zeitunabhaengig), und deren schneller Anteil (< 0,33 s) =
  "Welligkeit quer". Das ist das Zackige in der VM-Spur.
* Tempo: TCP-Tempo zwischen zwei neuen Werten gegen das ueber 0,25 s
  geglaettete Tempo -- "Tempo-Welligkeit", "Einbruch" (< 50 %),
  "Stillstand". Das ist das Stop-and-go laengs der Bahn. Zeitunabhaengig
  vom Plan, also auch bei Override < 1 vergleichbar.
* Spektrum dieser Tempo-Abweichung: staerkste Frequenz, Anteil bei 15 Hz
  (Zieltakt) und 10 Hz (60-Hz-Strom gegen 50-Hz-Zustand der Steuerung).
* Verzug und Folgefehler im Gelenkraum (bei Override < 1 nur grob: dort
  waechst der Verzug unterwegs; bei ``onlinehalt`` verfaelscht die
  Wartezeit Verzug und Fahrzeit-Faktor).
* Haltepunkte (``online``/``onlinehalt``, je Greiferwechsel der Bahn, z. B.
  PICK): tiefster Punkt gegenueber dem Haltepunkt in mm (+ = darueber),
  kleinster Abstand, Verweilzeit innerhalb ONLINE_REACH_TOL_MM und am
  tiefsten Punkt (+-HALT_UNTEN_MM), bei ``onlinehalt`` die Wartezeit.
  Anlass: 2026-10-16 kehrte der Arm mit ``online`` 27 mm ueber PICK um.

Grenzen: Die Steuerung liefert ~50 neue Gelenkwerte je Sekunde, aufloesbar
ist bis ~25 Hz. Die Zeitpunkte der neuen Werte werden auf das Raster der
Steuerung gelegt, wenn es eines gibt (``raster_zeiten``) -- sonst ist das
Tempo um die Lesedauer verrauscht. FK lokal aus der URDF-Kette (Flansch;
in der VM ohne Tool = TCP). In der VM: keine Mechanik -- gemessen wird,
wie die Steuerung den Strom UMSETZT, nicht, wie der Arm schwingt.

BEWEGUNG: nur mit ``--move``. In der VM (``is_robot_in_simulation()`` True)
ohne Rueckfrage; an der Anlage nur mit ``--real-robot``, Freigabewort und
ausdruecklicher ``--laeufe``-Liste (die Profile ``pos`` und ``a0`` sind dort
ungeprueft und koennen MEHR anregen). Der Greifer schaltet nicht -- es geht
nur um die Bahn. Jeder Sollwert laeuft durch den ServoGuard des Adapters.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_servo_stream.py --move                       # VM, Standardmatrix
    python tools/check_servo_stream.py --move --matrix kurz
    python tools/check_servo_stream.py --move --laeufe linear-60-r0-o1,spline-60-r0-o1
    python tools/check_servo_stream.py --move --real-robot --laeufe online-15-r0-o1,onlinehalt-15-r0-o1
    python tools/check_servo_stream.py --auswerten Berichte/<stempel>_Servo-Strom   # nur neu rechnen
"""

import argparse
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config, execution
from bc.clock import RealClock, host_time
from bc.sync import Pacer
from check_trajectory_buffer import Mitschrieb, sendetakt

BERICHTE = Path(__file__).resolve().parent.parent / "Berichte"

PROFILE = ("linear", "a0", "pos", "spline")
#: movelinear_online ohne und mit Halteregel (bc/execution.py).
ONLINE_ARTEN = ("online", "onlinehalt")
#: Lauf-Arten: die servo_j-Profile und movelinear_online zum Vergleich.
ARTEN = PROFILE + ONLINE_ARTEN

#: Haltepunkt-Auswertung: nur Werte naeher als HALT_NAEHE_MM am Haltepunkt,
#: im Zeitfenster HALT_VOR_S vor bis HALT_NACH_S nach dem geplanten Dwell
#: (plus Wartezeit). "Am tiefsten Punkt" = hoechstens HALT_UNTEN_MM ueber ihm.
HALT_NAEHE_MM = 60.0
HALT_VOR_S = 1.0
HALT_NACH_S = 3.0
HALT_UNTEN_MM = 0.5

#: Kleinste und groesste Senderate -- darueber haelt kein Rechner den Takt.
MIN_RATE_HZ = 5.0
MAX_RATE_HZ = 250.0

#: Override-Bereich des Werkzeugs.
MIN_OVERRIDE = 0.1

#: Zeitfenster der Glaettung fuer das Bezugstempo (s) und des Hochpasses
#: der Querabweichung (s).
TEMPO_FENSTER_S = 0.25
QUER_FENSTER_S = 0.33

#: Ab diesem geglaetteten TCP-Tempo (m/s) zaehlt ein Abschnitt als "in Fahrt".
FAHRT_MS = 0.02

#: Unter diesem Weg zwischen zwei neuen Werten (m) gilt der Arm als stehend.
STILLSTAND_M = 1e-4

#: Arm in Ruhe, wenn sich kein Gelenk ueber RUHE_S um mehr als RUHE_RAD bewegt.
RUHE_RAD = 2e-4
RUHE_S = 0.3
#: Hoechstens so lange nach dem letzten Sollwert auf Ruhe warten (Override < 1).
NACHLAUF_MAX_S = 60.0

#: Standardmatrix (VM). Kuerzel: profil-rate-r<rauschen>-o<override>.
MATRIX = {
    # Heute (linear 60 Hz) gegen Override und Rauschen
    "override": [
        "linear-60-r%s-o%s" % (r, o)
        for o in ("1", "0.5", "0.25") for r in ("0", "0.5", "1")
    ],
    # Art des Stroms bei Override 1, ohne und mit vollem Rauschen
    "strom": [
        "%s-r%s-o1" % (pr, r)
        for r in ("0", "1")
        for pr in ("linear-30", "linear-50", "linear-90", "linear-120",
                   "a0-60", "pos-60", "spline-30", "spline-60", "spline-120")
    ],
    # Wiederholung des Bezugs am Ende: hat sich die VM unterwegs veraendert?
    "wiederholung": ["linear-60-r0-o1"],
}
MATRIX_KURZ = ["linear-60-r0-o1", "linear-60-r0-o0.5", "linear-60-r1-o1",
               "a0-60-r0-o1", "pos-60-r0-o1", "spline-60-r0-o1", "linear-50-r0-o1"]


def section(titel):
    print("\n" + "=" * 70)
    print(titel)
    print("=" * 70)


def show(label, value):
    print("  %-40s %s" % (label, value))


# -- Lauf-Kuerzel ----------------------------------------------------------


def parse_lauf(text):
    """``"spline-60-r0.5-o1"`` -> dict(profil, rate, rauschen, override)."""
    teile = text.strip().split("-")
    if len(teile) != 4 or not teile[2].startswith("r") or not teile[3].startswith("o"):
        raise ValueError("Lauf %r: Form profil-rate-r<rauschen>-o<override>, "
                         "z. B. linear-60-r0-o1" % text)
    profil = teile[0]
    if profil not in ARTEN:
        raise ValueError("Lauf %r: Profil %r unbekannt (%s)" % (text, profil, ", ".join(ARTEN)))
    rate = float(teile[1])
    if profil in ONLINE_ARTEN and rate != config.CONTROL_RATE_HZ:
        raise ValueError("Lauf %r: online sendet ein Ziel je Takt -- Rate %g"
                         % (text, config.CONTROL_RATE_HZ))
    rauschen = float(teile[2][1:])
    override = float(teile[3][1:])
    if not MIN_RATE_HZ <= rate <= MAX_RATE_HZ:
        raise ValueError("Lauf %r: Rate %g Hz ausserhalb %g..%g" % (text, rate, MIN_RATE_HZ, MAX_RATE_HZ))
    if not 0.0 <= rauschen <= 1.0:
        raise ValueError("Lauf %r: Rauschen %g ausserhalb 0..1" % (text, rauschen))
    if not MIN_OVERRIDE <= override <= 1.0:
        raise ValueError("Lauf %r: Override %g ausserhalb %g..1" % (text, override, MIN_OVERRIDE))
    return {"name": text.strip(), "profil": profil, "rate": rate,
            "rauschen": rauschen, "override": override}


def laeufe_aus(args):
    if args.laeufe:
        laeufe = [parse_lauf(t) for t in args.laeufe.split(",") if t.strip()]
    elif args.matrix == "kurz":
        laeufe = [parse_lauf(t) for t in MATRIX_KURZ]
    else:
        laeufe = [parse_lauf(t) for block in ("override", "strom", "wiederholung")
                  for t in MATRIX[block]]
    # Wiederholungen eindeutig benennen -- sonst ueberschreibt der zweite
    # Lauf Bericht und Rohdaten des ersten (Matrix 2026-10-14).
    gesehen = {}
    for l in laeufe:
        n = gesehen.get(l["name"], 0) + 1
        gesehen[l["name"]] = n
        if n > 1:
            l["name"] = "%s#%d" % (l["name"], n)
    return laeufe


# -- Sollwert-Profil (hardwarefrei, getestet in tests/test_check_servo_stream.py)


class Profil(object):
    """Sollwerte zwischen den 15-Hz-Zielen ``knoten`` zu beliebiger Zeit.

    Zeitachse wie im Recorder: t = 0 haelt am ersten Ziel, Ziel i ist zur
    Zeit i * T erreicht (T = 1 / rate_knoten). ``linear`` liefert zu
    t = k / 60 genau die Sollwerte von bc/servo.ServoInterpolator.
    """

    def __init__(self, knoten, art="linear", rate_knoten=config.CONTROL_RATE_HZ):
        if art not in PROFILE:
            raise ValueError("Profil %r unbekannt" % art)
        self.q = np.asarray(knoten, dtype=float)
        if self.q.ndim != 2 or len(self.q) < 2:
            raise ValueError("mindestens zwei Ziele noetig")
        self.art = art
        self.T = 1.0 / float(rate_knoten)
        self.n = len(self.q)
        self.dauer = (self.n - 1) * self.T
        # Segment-Geschwindigkeiten (Segment i endet an Ziel i), v[0] = 0
        self.v_seg = np.zeros_like(self.q)
        self.v_seg[1:] = np.diff(self.q, axis=0) / self.T
        # Knoten-Tangenten fuer den Spline: zentrale Differenz, Enden 0
        self.m = np.zeros_like(self.q)
        self.m[1:-1] = (self.q[2:] - self.q[:-2]) / (2.0 * self.T)

    def at(self, t):
        """(q, v, a) zur Zeit t (s)."""
        null = np.zeros(self.q.shape[1])
        if t <= 0.0:
            return self.q[0].copy(), null, null.copy()
        if t >= self.dauer - 1e-12:
            return self.q[-1].copy(), null, null.copy()
        i = int(math.ceil(t / self.T - 1e-9))
        i = min(max(i, 1), self.n - 1)
        u = (t - (i - 1) * self.T) / self.T
        p0, p1 = self.q[i - 1], self.q[i]
        if self.art == "spline":
            T = self.T
            m0, m1 = self.m[i - 1], self.m[i]
            u2, u3 = u * u, u * u * u
            q = ((2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * T * m0
                 + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * T * m1)
            v = ((6 * u2 - 6 * u) * p0 + (3 * u2 - 4 * u + 1) * T * m0
                 + (-6 * u2 + 6 * u) * p1 + (3 * u2 - 2 * u) * T * m1) / T
            a = ((12 * u - 6) * p0 + (6 * u - 4) * T * m0
                 + (-12 * u + 6) * p1 + (6 * u - 2) * T * m1) / (T * T)
            return q, v, a
        q = p0 + u * (p1 - p0)
        if self.art == "pos":
            return q, null, null.copy()
        v = self.v_seg[i].copy()
        if self.art == "a0":
            return q, v, null
        return q, v, (self.v_seg[i] - self.v_seg[i - 1]) / self.T

    def positionen(self, ts):
        return np.asarray([self.at(float(t))[0] for t in ts])

    def sendezeiten(self, rate):
        """Sendezeitpunkte 0, 1/rate, ... -- das letzte Ziel immer dabei."""
        k = int(math.floor(self.dauer * rate + 1e-9))
        ts = np.arange(k + 1) / float(rate)
        if ts[-1] < self.dauer - 1e-9:
            ts = np.append(ts, self.dauer)
        return ts


# -- Auswertung (hardwarefrei) ---------------------------------------------


def neue_werte(t, q, tol=1e-7):
    """Nur die Lesungen, bei denen sich etwas geaendert hat (erste Sichtung)."""
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    if len(t) < 2:
        return t, q
    neu = np.any(np.abs(np.diff(q, axis=0)) > tol, axis=1)
    idx = np.concatenate([[0], np.flatnonzero(neu) + 1])
    return t[idx], q[idx]


def raster_zeiten(tn, max_rest_anteil=0.25):
    """Legt die Zeitpunkte neuer Werte auf das Raster der Steuerung.

    Liefert die Steuerung neue Werte in festem Takt (VM: ~20 ms), ist der
    gelesene Zeitpunkt nur um die Lesedauer verrauscht -- bei 20 ms Abstand
    und ~3 ms Lesedauer +-15 % im Tempo. Passt ein Raster t = a + b * k
    (k ganzzahlig) mit kleinem Rest, werden die Rasterzeiten verwendet.
    Rueckgabe: (Zeiten, Info).
    """
    tn = np.asarray(tn, dtype=float)
    if len(tn) < 20:
        return tn, {"raster": False, "grund": "zu wenige Werte"}
    d = np.diff(tn)
    periode = float(np.median(d))
    if periode <= 0:
        return tn, {"raster": False, "grund": "Periode 0"}
    k = np.round((tn - tn[0]) / periode)
    if np.any(np.diff(k) < 1):
        return tn, {"raster": False, "grund": "zwei Werte in einem Rasterschritt",
                    "periode_ms": 1e3 * periode}
    b, a = np.polyfit(k, tn, 1)
    rest = tn - (a + b * k)
    info = {"periode_ms": 1e3 * float(b), "rest_std_ms": 1e3 * float(np.std(rest)),
            "luecken": int(np.sum(np.diff(k) > 1))}
    if np.std(rest) > max_rest_anteil * b:
        info.update(raster=False, grund="Rest zu gross")
        return tn, info
    info["raster"] = True
    return a + b * k, info


def _gleiten_zeit(t, x, fenster):
    """Zeitgewichtetes gleitendes Mittel (Fenster in s) fuer unregelmaessige Zeiten."""
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    out = np.empty_like(x)
    lo = np.searchsorted(t, t - 0.5 * fenster, side="left")
    hi = np.searchsorted(t, t + 0.5 * fenster, side="right")
    for i in range(len(t)):
        out[i] = x[lo[i]:hi[i]].mean(axis=0)
    return out


def querabweichung(p, ref):
    """Abweichungsvektoren der Punkte ``p`` vom Polygonzug ``ref`` (je (N,3)).

    Naechster Punkt auf der Bahn, nicht der zeitgleiche -- unabhaengig von
    Verzug und Override.
    """
    p = np.asarray(p, dtype=float)
    ref = np.asarray(ref, dtype=float)
    out = np.empty_like(p)
    seg_a = ref[:-1]
    seg_d = np.diff(ref, axis=0)
    seg_l2 = np.maximum((seg_d ** 2).sum(1), 1e-18)
    for s in range(0, len(p), 200):
        block = p[s:s + 200]
        d2 = ((block[:, None, :] - ref[None, :, :]) ** 2).sum(2)
        naechst = np.argmin(d2, axis=1)
        for j, (x, k) in enumerate(zip(block, naechst)):
            best = ref[k]
            best_d = float(((x - best) ** 2).sum())
            for g in (k - 1, k):
                if 0 <= g < len(seg_a):
                    u = float(np.clip(np.dot(x - seg_a[g], seg_d[g]) / seg_l2[g], 0.0, 1.0))
                    c = seg_a[g] + u * seg_d[g]
                    dd = float(((x - c) ** 2).sum())
                    if dd < best_d:
                        best, best_d = c, dd
            out[s + j] = x - best
    return out


def tempo_kennzahlen(tn, pn):
    """Stop-and-go laengs der Bahn aus den neuen Werten (TCP-Positionen)."""
    tn = np.asarray(tn, dtype=float)
    pn = np.asarray(pn, dtype=float)
    if len(tn) < 20:
        return {}
    ds = np.linalg.norm(np.diff(pn, axis=0), axis=1)
    dt = np.maximum(np.diff(tn), 1e-6)
    s = np.concatenate([[0.0], np.cumsum(ds)])
    tm = 0.5 * (tn[1:] + tn[:-1])
    v = ds / dt
    h = 0.5 * TEMPO_FENSTER_S
    lo = np.clip(tm - h, tn[0], tn[-1])
    hi = np.clip(tm + h, tn[0], tn[-1])
    vg = (np.interp(hi, tn, s) - np.interp(lo, tn, s)) / np.maximum(hi - lo, 1e-6)
    fahrt = vg > FAHRT_MS
    if fahrt.sum() < 10:
        return {"fahrt_werte": int(fahrt.sum())}
    w = dt[fahrt]
    rel = (v[fahrt] - vg[fahrt]) / vg[fahrt]
    out = {
        "fahrt_s": float(w.sum()),
        "tempo_welligkeit_prozent": 100.0 * float(np.sqrt(np.sum(w * rel ** 2) / np.sum(w))),
        "einbruch_prozent": 100.0 * float(np.sum(w[v[fahrt] < 0.5 * vg[fahrt]]) / np.sum(w)),
        "stillstand_prozent": 100.0 * float(np.sum(w[ds[fahrt] < STILLSTAND_M]) / np.sum(w)),
        "tempo_max_ms": float(np.max(v[fahrt])),
        "tempo_glatt_max_ms": float(np.max(vg)),
    }
    # Spektrum der relativen Tempo-Abweichung auf gleichmaessigem Raster
    fs = 1.0 / float(np.median(dt))
    if fs >= 20.0:
        tg = np.arange(tm[0], tm[-1], 1.0 / fs)
        x = np.interp(tg, tm, np.where(fahrt, (v - vg) / np.maximum(vg, FAHRT_MS), 0.0))
        x = x - x.mean()
        spek = np.abs(np.fft.rfft(x * np.hanning(len(x)))) / max(len(x), 1)
        f = np.fft.rfftfreq(len(x), 1.0 / fs)
        band = (f >= 2.0) & (f <= 0.5 * fs - 0.5)
        if band.any():
            gesamt = float(np.sqrt(np.sum(spek[band] ** 2)))

            def anteil(f0):
                m = band & (np.abs(f - f0) <= 1.0)
                return float(np.sqrt(np.sum(spek[m] ** 2)) / gesamt) if gesamt > 0 and m.any() else 0.0

            out.update({
                "spitze_hz": float(f[band][np.argmax(spek[band])]),
                "anteil_15hz": anteil(15.0),
                "anteil_10hz": anteil(10.0),
                "abtastrate_hz": fs,
            })
    return out


def verzug_gelenke(tn, qn, profil, suche=(-0.3, 3.0), grob=0.02, fein=0.002):
    """Konstanter Zeitversatz, mit dem die Gelenke dem Profil am besten folgen.

    Erst grob, dann um das Minimum fein gesucht (jede Probe rechnet das
    Profil an allen Messpunkten neu).
    """
    tn = np.asarray(tn, dtype=float)
    qn = np.asarray(qn, dtype=float)

    def fehler(tau):
        m = (tn - tau >= 0.0) & (tn - tau <= profil.dauer)
        if m.sum() < 20:
            return float("inf")
        soll = profil.positionen(tn[m] - tau)
        return float(np.sqrt(np.mean((qn[m] - soll) ** 2)))

    best = min((fehler(t), float(t)) for t in np.arange(suche[0], suche[1] + 1e-12, grob))
    lo, hi = best[1] - grob, best[1] + grob
    best = min([best] + [(fehler(t), float(t)) for t in np.arange(lo, hi + 1e-12, fein)])
    return best[1], best[0]


def auswerten_lauf(sendungen, t, q, profil, rate, fk_pos, t_start):
    """Alle Kennzahlen eines Laufs. ``t_start``: Host-Zeit von Profil-t = 0.

    ``fk_pos``: Funktion Gelenkwinkel (N,6) -> TCP-Positionen (N,3).
    """
    k = {"sendetakt": sendetakt(sendungen, rate, profil.dauer)}
    t = np.asarray(t, dtype=float) - t_start
    tn, qn = neue_werte(t, q)
    if len(tn) < 20:
        k["fehler"] = "zu wenige neue Werte (%d)" % len(tn)
        return k
    # Nur das Fenster der Fahrt (+ Nachlauf) bewerten
    m = tn >= -0.2
    tn, qn = tn[m], qn[m]
    k["neue_werte_hz"] = float((len(tn) - 1) / max(tn[-1] - tn[0], 1e-6))
    tr, info = raster_zeiten(tn)
    k["raster"] = info
    pn = fk_pos(qn)
    k["tempo"] = tempo_kennzahlen(tr, pn)

    ref = fk_pos(profil.positionen(np.arange(0.0, profil.dauer, 0.002)))
    dev = querabweichung(pn, ref)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pn, axis=0), axis=1))])
    h = 0.5 * TEMPO_FENSTER_S
    vg = (np.interp(np.clip(tr + h, tr[0], tr[-1]), tr, s)
          - np.interp(np.clip(tr - h, tr[0], tr[-1]), tr, s)) / TEMPO_FENSTER_S
    fahrt = vg > FAHRT_MS
    if fahrt.sum() >= 10:
        betrag = np.linalg.norm(dev, axis=1)[fahrt] * 1e3
        schnell = np.linalg.norm(dev - _gleiten_zeit(tr, dev, QUER_FENSTER_S), axis=1)[fahrt] * 1e3
        k["quer"] = {
            "rms_mm": float(np.sqrt(np.mean(betrag ** 2))),
            "max_mm": float(np.max(betrag)),
            "welligkeit_rms_mm": float(np.sqrt(np.mean(schnell ** 2))),
            "welligkeit_max_mm": float(np.max(schnell)),
        }
    tau, fehler = verzug_gelenke(tr, qn, profil)
    k["verzug_ms"] = 1e3 * tau
    k["folgefehler_rms_mrad"] = 1e3 * fehler
    # Ende der Bewegung: letzter neuer Wert mit Aenderung > RUHE_RAD gegen das Ende
    ende = qn[-1]
    bewegt = np.flatnonzero(np.max(np.abs(qn - ende), axis=1) > RUHE_RAD)
    if len(bewegt):
        k["fahrzeit_faktor"] = float(tr[bewegt[-1]] / profil.dauer)
    return k


def _laengster_lauf(t, maske):
    """Dauer (s) des laengsten zusammenhaengenden Abschnitts mit ``maske``."""
    best, anfang = 0.0, None
    for ti, m in zip(t, maske):
        if m:
            anfang = ti if anfang is None else anfang
            best = max(best, float(ti - anfang))
        else:
            anfang = None
    return best


def halt_kennzahlen(t, q, ziele, halte, fk_pos, warte=None,
                    takt_s=1.0 / config.CONTROL_RATE_HZ, tol_mm=config.ONLINE_REACH_TOL_MM):
    """Wie nah und wie lange stand der Arm an jedem Haltepunkt?

    ``t``/``q``: ALLE Lesungen (nicht nur neue Werte -- ein stehender Arm
    liefert keine neuen), t = 0 beim ersten Sollwert. ``ziele``: Gelenkziele
    des Plans, ``halte``: Liste (start, ende) aus bc.execution.hold_blocks.
    ``warte``: Halte-Ergebnisse von ``onlinehalt`` (mit ``step``) -- sie
    verschieben die Zeitfenster der folgenden Haltepunkte.

    Je Haltepunkt: ``tief_mm`` tiefster Punkt gegenueber dem Haltepunkt
    (+ = darueber; 2026-10-16 ohne Halteregel +27 mm), ``quer_tief_mm``
    seitlicher Abstand dort, ``min_mm`` kleinster Abstand, ``verweil_s``
    laengste Zeit innerhalb ``tol_mm``, ``unten_s`` laengste Zeit
    hoechstens HALT_UNTEN_MM ueber dem tiefsten Punkt.
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    ziele = np.asarray(ziele, dtype=float)
    warte_je = {int(w["step"]): w for w in (warte or []) if w.get("step") is not None}
    out = []
    versatz = 0.0
    for start, ende in halte:
        w = warte_je.get(int(start))
        gewartet = float(w["wait_s"]) if w else 0.0
        e = {"takt": int(start), "ende": int(ende)}
        if w:
            e["warten"] = {k: w.get(k) for k in ("reached", "wait_s", "start_mm", "dist_mm",
                                                  "min_mm", "reason")}
        m = ((t >= start * takt_s - HALT_VOR_S + versatz)
             & (t <= ende * takt_s + HALT_NACH_S + versatz + gewartet))
        versatz += gewartet
        if m.sum() < 2:
            e["fehler"] = "keine Lesungen im Zeitfenster"
            out.append(e)
            continue
        tw = t[m]
        pw = fk_pos(q[m])
        ziel = fk_pos(ziele[start])[0]
        d = 1e3 * np.linalg.norm(pw - ziel, axis=1)
        e["min_mm"] = float(d.min())
        nah = d <= HALT_NAEHE_MM
        if not nah.any():
            e["fehler"] = "nie naeher als %g mm" % HALT_NAEHE_MM
            out.append(e)
            continue
        dz = 1e3 * (pw[:, 2] - ziel[2])
        k = int(np.flatnonzero(nah)[np.argmin(dz[nah])])
        e.update({
            "tief_mm": float(dz[k]),
            "quer_tief_mm": float(1e3 * np.linalg.norm(pw[k, :2] - ziel[:2])),
            "t_tief_s": float(tw[k]),
            "verweil_s": _laengster_lauf(tw, d <= tol_mm),
            "unten_s": _laengster_lauf(tw, nah & (dz <= dz[k] + HALT_UNTEN_MM)),
            "tol_mm": float(tol_mm),
        })
        out.append(e)
    return out


def halt_zeilen(name, halte):
    """Ausgabe der Haltepunkt-Kennzahlen eines Laufs."""
    zeilen = []
    for e in halte or []:
        if "fehler" in e:
            zeilen.append("  %-20s Halt Takt %d: %s" % (name, e["takt"], e["fehler"]))
            continue
        text = ("  %-20s Halt Takt %d: tiefster Punkt %+.1f mm (quer %.1f), min. Abstand "
                "%.1f mm, %.2f s <= %.1f mm, %.2f s unten"
                % (name, e["takt"], e["tief_mm"], e["quer_tief_mm"], e["min_mm"],
                   e["verweil_s"], e["tol_mm"], e["unten_s"]))
        w = e.get("warten")
        if w:
            text += ", gewartet %.2f s%s" % (w["wait_s"], "" if w["reached"] else
                                              " NICHT ERREICHT (%s)" % w["reason"])
        zeilen.append(text)
    return zeilen


# -- Ausfuehrung -----------------------------------------------------------


def fk_funktion():
    from bc.urdf import KinematicChain

    kette = KinematicChain.from_urdf(config.URDF_PATH)

    def fk_pos(qs):
        qs = np.atleast_2d(np.asarray(qs, dtype=float))
        return np.asarray([kette.fk(qq)[:3, 3] for qq in qs])

    return fk_pos


def standard_ablauf():
    root = Path(__file__).resolve().parent.parent
    arbeit = root / "sequences" / "arbeitskopien" / "pick_to_station.json"
    return arbeit if arbeit.exists() else root / "sequences" / "pick_to_station.json"


def plane(bot, ablauf_pfad, rauschen, seed):
    """Bahn wie apps/record.py (Standard-Planertempo), Rauschen fest."""
    from bc.collision import default_workspace
    from bc.kinematics import Kinematics
    from bc.noise import generate_plan
    from bc.sequence import load_sequence, resolve_sequence
    from bc.trajectory import build_ideal_trajectory

    sequenz = load_sequence(str(ablauf_pfad))
    aufgeloest = resolve_sequence(sequenz, bot)
    wp = aufgeloest.waypoints
    ideal = build_ideal_trajectory(wp, fk=bot.fk)
    z_min = min(w.pose_quat[2] for w in wp)
    arbeitsraum = default_workspace(table_height_m=z_min - 0.15)
    start = wp[0].joints if wp[0].joints is not None else bot.read_state().joints
    return generate_plan(Kinematics(bot), arbeitsraum, ideal, start,
                         rng=np.random.default_rng(seed), noise_scale=rauschen)


def warten_bis_ruhe(bot, max_s):
    t_end = time.time() + max_s
    letzte = np.asarray(bot.get_joint_angles(), dtype=float)
    seit = time.time()
    while time.time() < t_end:
        time.sleep(0.05)
        jetzt = np.asarray(bot.get_joint_angles(), dtype=float)
        if float(np.max(np.abs(jetzt - letzte))) > RUHE_RAD:
            letzte, seit = jetzt, time.time()
        elif time.time() - seit >= RUHE_S:
            return True
    return False


def fahre_lauf(bot, profil, rate, online=None, pause_s=0.0, halte=(), fk_pos=None):
    """Sendet das Profil per servo_j mit ``rate``; liefert Rohdaten.

    ``online``: Eintrag aus bc.execution.describe("online") -- dann statt
    servo_j je 15-Hz-Takt EIN movelinear_online-Ziel mit Vorlauf, wie der
    Recorder (Ausrollen, dann stop_online). Mit Halteregel (``reach_*``)
    wartet er an jedem Haltepunkt aus ``halte`` wie der Recorder; Abstand
    per ``fk_pos`` (lokale Kette) aus ``get_joint_angles``.
    """
    halt = online is not None and execution.reach_rule(online)
    halt_starts = {s for s, _ in halte} if halt else set()
    warte = []
    ist = np.asarray(bot.get_joint_angles(), dtype=float)
    if float(np.max(np.abs(ist - profil.q[0]))) > config.START_POSE_TOL_RAD:
        bot.move_to_joints(profil.q[0])
        time.sleep(0.3)
    if pause_s > 0:
        # Ruhe vor der Fahrt: trennt die Laeufe in der Handy-Aufzeichnung
        # (Rueckfahrt an den Start ist dort auch eine Bewegung).
        time.sleep(pause_s)
    print("  LOS  %s" % datetime.now().strftime("%H:%M:%S"))
    zeiten = profil.sendezeiten(rate)
    if online is not None:
        zeiten = np.arange(profil.n) * profil.T
    log = Mitschrieb(bot.get_joint_angles).start()
    time.sleep(0.3)  # Ruhe vor der Fahrt im Mitschrieb
    bot.activate_servo("position")
    pacer = Pacer(RealClock(), rate).start()
    sendungen = []
    t_start = None
    abbruch = ""
    try:
        for i, ts in enumerate(zeiten):
            if bot.stop_requested:
                abbruch = "Stopp angefordert"
                break
            t_soll = pacer.tick()
            if t_start is None:
                t_start = t_soll
            t0 = host_time()
            if online is not None:
                ziel = profil.q[execution.lead_index(
                    i, online["lead_steps"], profil.n,
                    execution.next_hold_end(i, halte) if halt else None)]
                bot.online_j(ziel, online["vel"], online["acc"])
                if i in halt_starts:
                    # wie der Recorder; Greifer schaltet hier nicht
                    t_beginn = host_time()
                    w = execution.wait_until_reached(
                        bot, profil.q[i], online, RealClock(), config.CONTROL_RATE_HZ,
                        target_tcp=fk_pos(profil.q[i])[0],
                        measure_tcp=lambda: fk_pos(bot.get_joint_angles())[0])
                    w.update(step=int(i), t_beginn_s=t_beginn - t_start)
                    warte.append(w)
                    print("  %s" % execution.reach_line(w))
                    if not w["reached"]:
                        abbruch = execution.reach_failure_text(w, i)
                        break
                    pacer.start().tick()   # Wartezeit ist kein Ueberlauf
            else:
                q, v, a = profil.at(float(ts))
                bot.servo_j(q, v, a)
            sendungen.append((t_soll, t0, host_time()))
        if online is not None and not bot.stop_requested and not abbruch:
            warten_bis_ruhe(bot, 5.0)   # Ausrollen bis zum letzten Ziel
    finally:
        if online is not None:
            bot.stop_online()
        bot.deactivate_servo()
    ruhe = warten_bis_ruhe(bot, NACHLAUF_MAX_S)
    time.sleep(0.2)
    t, q = log.stop()
    return {
        "t_start": t_start, "sendungen": np.asarray(sendungen), "t": t, "q": q,
        "ueberlaeufe": pacer.overruns, "lesedauer_ms": log.lesedauer_ms(),
        "lesefehler": log.fehler, "ruhe": ruhe, "abbruch": abbruch, "warte": warte,
    }


def zeile_kopf():
    return ("  %-20s %5s %6s %6s %7s %7s %6s %6s %6s %6s %6s %7s" % (
        "Lauf", "Ueb.", "Abst.", "Fakt.", "quer", "quer", "Tempo", "Einbr", "Still",
        "Spitze", "15Hz", "Verzug"))


def zeile_einheit():
    return ("  %-20s %5s %6s %6s %7s %7s %6s %6s %6s %6s %6s %7s" % (
        "", "", "std ms", "", "RMS mm", "well.mm", "well.%", "%", "%", "Hz", "Ant.", "ms"))


def zeile(name, e):
    k = e.get("kennzahlen", {})
    st = k.get("sendetakt", {})
    qu = k.get("quer", {})
    te = k.get("tempo", {})
    nan = float("nan")
    return ("  %-20s %5s %6.2f %6.2f %7.2f %7.2f %6.1f %6.1f %6.1f %6.1f %6.2f %7.0f" % (
        name, e.get("ueberlaeufe", "-"), st.get("abstand_std_ms", nan), st.get("dauer_faktor", nan),
        qu.get("rms_mm", nan), qu.get("welligkeit_rms_mm", nan),
        te.get("tempo_welligkeit_prozent", nan), te.get("einbruch_prozent", nan),
        te.get("stillstand_prozent", nan), te.get("spitze_hz", nan), te.get("anteil_15hz", nan),
        k.get("verzug_ms", nan)))


def tabelle(bericht):
    section("Ergebnis")
    print(zeile_kopf())
    print(zeile_einheit())
    for name, e in bericht["laeufe"].items():
        print(zeile(name, e))
    for name, e in bericht["laeufe"].items():
        for z in halt_zeilen(name, e.get("kennzahlen", {}).get("halte")):
            print(z)
    print("  (Ueb. = Takt-Ueberlaeufe; Abst. = Streuung des Sendeabstands; Fakt. = Sendedauer / Plan;\n"
          "   quer = Abstand zur befohlenen Bahn, well. = Anteil < 0,33 s; Tempo well. = RMS der\n"
          "   Tempo-Abweichung vom 0,25-s-Mittel; Einbr. = Fahrzeit unter 50 %; Still = Fahrzeit\n"
          "   ohne Bewegung; Spitze/15Hz = Spektrum der Tempo-Abweichung, aufloesbar bis ~25 Hz;\n"
          "   Halt = je Greiferwechsel: tiefster Punkt gegenueber dem Haltepunkt, + = darueber,\n"
          "   Flansch-FK der lokalen Kette)")


def auswerten_datei(stamm, fk_pos):
    """Kennzahlen aus gespeicherten Rohdaten neu rechnen (ohne Steuerung)."""
    bericht = json.loads(Path(str(stamm) + ".json").read_text(encoding="utf-8"))
    roh = np.load(str(stamm) + "_Mitschrieb.npz")
    plaene = {}
    for name, e in bericht["laeufe"].items():
        r = e["lauf"]["rauschen"]
        key = "plan_r%g" % r
        if key not in plaene:
            plaene[key] = roh[key]
        art = e["lauf"]["profil"]
        if name + "_send" not in roh.files:
            continue   # Lauf mit Fehler, keine Rohdaten
        profil = Profil(plaene[key], "linear" if art in ONLINE_ARTEN else art)
        e["kennzahlen"] = auswerten_lauf(
            roh[name + "_send"], roh[name + "_t"], roh[name + "_q"], profil,
            e["lauf"]["rate"], fk_pos, 0.0)
        if art in ONLINE_ARTEN:
            # Haltepunkte stehen erst seit 2026-10-16 im Mitschrieb; aeltere
            # Berichte: aus den Zielen rekonstruiert.
            if "halte_r%g" % r in roh.files:
                halte = [tuple(int(x) for x in b) for b in roh["halte_r%g" % r]]
            else:
                halte = halte_aus_zielen(plaene[key])
            e["kennzahlen"]["halte"] = halt_kennzahlen(
                roh[name + "_t"], roh[name + "_q"], plaene[key], halte, fk_pos,
                warte=e.get("warte"))
    return bericht


def halte_aus_zielen(ziele, dwell_steps=config.GRIPPER_DWELL_STEPS, tol=5e-6):
    """Haltepunkte aus den Gelenkzielen allein (fuer Berichte ohne Greiferspur).

    Der Planer haelt am Greiferwechsel nach der Ankunft exakt still
    (Rauschen dort 0); sonst steht die Bahn hoechstens einen Takt. Ein
    Stillstand von mindestens ``dwell_steps`` Zielen ist darum ein Dwell:
    (erster Takt nach der Ankunft, letzter Dwell-Takt) wie
    bc.execution.hold_blocks -- der Planer schaltet den Greifer im ersten
    Takt nach der Ankunft (bc/trajectory.py). Der Beginn wird NICHT aus
    ``dwell_steps`` zurueckgerechnet: der Dwell eines Berichts kann vom
    heutigen GRIPPER_DWELL_S abweichen (Test 3 am 2026-10-16: 1,5 s, der
    Haltepunkt landete sonst 15 Takte zu spaet).
    """
    ziele = np.asarray(ziele, dtype=float)
    gleich = np.max(np.abs(np.diff(ziele, axis=0)), axis=1) <= tol
    halte, i = [], 0
    while i < len(gleich):
        if not gleich[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(gleich) and gleich[j + 1]:
            j += 1
        ende = j + 1                     # letztes Ziel des Stillstands
        if ende - i >= dwell_steps and i > 0:
            halte.append((i + 1, ende))  # i = Ankunft, i + 1 = Greiferwechsel
        i = j + 1
    return halte


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)  # Fortschritt auch in Logdateien
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--move", action="store_true", help="Laeufe fahren (sonst nur Plan pruefen)")
    ap.add_argument("--matrix", choices=("standard", "kurz"), default="standard",
                    help="vordefinierte Laeufe (Standard: %d Laeufe)"
                         % sum(len(v) for v in MATRIX.values()))
    ap.add_argument("--laeufe", default="",
                    help="eigene Liste profil-rate-r<rauschen>-o<override>, Komma-getrennt "
                         "(an der Anlage Pflicht)")
    ap.add_argument("--ablauf", default=None, help="Ablaufdatei (Standard: Arbeitskopie pick_to_station)")
    ap.add_argument("--seed", type=int, default=7, help="Seed der Rauschform (Standard 7)")
    ap.add_argument("--real-robot", action="store_true", help="REALE Anlage freigeben (Freigabewort)")
    ap.add_argument("--pause", type=float, default=None,
                    help="Ruhe vor jeder Fahrt in s (Standard: Anlage 3, VM 0) -- trennt die "
                         "Laeufe in der Handy-Aufzeichnung")
    ap.add_argument("--auswerten", default=None,
                    help="nur auswerten: Berichtsstamm ohne Endung (Berichte/<stempel>_Servo-Strom)")
    args = ap.parse_args()

    fk_pos = fk_funktion()
    if args.auswerten:
        bericht = auswerten_datei(args.auswerten, fk_pos)
        Path(args.auswerten + ".json").write_text(
            json.dumps(bericht, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
        tabelle(bericht)
        return

    try:
        laeufe = laeufe_aus(args)
    except ValueError as exc:
        raise SystemExit(str(exc))
    ablauf = Path(args.ablauf) if args.ablauf else standard_ablauf()

    from bc.adapters.neura import NeuraRobot

    section("0. Verbindung")
    bot = NeuraRobot(override=1.0, allow_real=args.real_robot)
    bot.connect(power_on=False, ensure_automatic=False)
    in_sim = bot.in_simulation
    real = in_sim is not True
    show("is_robot_in_simulation()", in_sim)
    show("Server-Version", getattr(bot.robot, "version", None))
    show("Tool", bot.tool_name)
    show("Ablauf", ablauf)
    show("Laeufe", len(laeufe))
    move = args.move
    if move and real:
        if not args.real_robot:
            bot.close()
            raise SystemExit("is_robot_in_simulation() ist %r -- Fahrten nur mit --real-robot." % (in_sim,))
        if not args.laeufe:
            bot.close()
            raise SystemExit("An der Anlage nur mit ausdruecklicher --laeufe-Liste.")
        print("\n!! --real-robot: %d Laeufe der GANZEN Aufnahmebahn an der REALEN ANLAGE:" % len(laeufe))
        for l in laeufe:
            print("     %s" % l["name"])
        print("   Greifer schaltet nicht, kein Werkstueck. Arbeitsraum frei? Not-Aus in Reichweite?")
        try:
            antwort = input("   Zum Fortfahren 'ANLAGE' eintippen: ").strip()
        except EOFError:
            antwort = ""
        if antwort != "ANLAGE":
            bot.close()
            raise SystemExit("Abgebrochen -- nichts bewegt.")

    bericht = {
        "erzeugt": datetime.now().isoformat(timespec="seconds"),
        "geltungsbereich": "Zahlen gelten NUR fuer die hier gemessene Steuerung. VM: wie "
                           "die Steuerung den Strom umsetzt, keine Mechanik.",
        "in_simulation": in_sim,
        "server_version": getattr(bot.robot, "version", None),
        "ablauf": str(ablauf),
        "seed": args.seed,
        "argumente": vars(args),
        "laeufe": {},
    }
    roh = {}
    try:
        if move:
            bot.connect(power_on=True, ensure_automatic=True)
        plaene = {}
        for r in sorted({l["rauschen"] for l in laeufe}):
            plan = plane(bot, ablauf, r, args.seed)
            plaene[r] = plan
            roh["plan_r%g" % r] = np.asarray(plan.joints_noisy, dtype=float)
            halte_r = execution.hold_blocks(plan.gripper, plan.dwell_mask)
            roh["halte_r%g" % r] = np.asarray(halte_r, dtype=int).reshape(-1, 2)
            show("Plan Rauschen %g" % r, "%d Ziele, %.1f s, %d Rejects, Haltepunkte %s"
                 % (len(plan), (len(plan) - 1) / config.CONTROL_RATE_HZ, plan.rejects,
                    ", ".join("Takt %d-%d" % b for b in halte_r) or "keine"))
        if not move:
            print("\n(keine Fahrt -- --move)")
            return
        for nr, l in enumerate(laeufe, 1):
            section("Lauf %d/%d: %s  (%s %g Hz, Rauschen %g, Override %g)"
                    % (nr, len(laeufe), l["name"], l["profil"], l["rate"], l["rauschen"], l["override"]))
            bot._call("set_override", l["override"])
            ist_online = l["profil"] in ONLINE_ARTEN
            if ist_online and not real:
                print("  uebersprungen: die VM nimmt movelinear_online an, faehrt aber nicht")
                continue
            plan = plaene[l["rauschen"]]
            profil = Profil(plan.joints_noisy, "linear" if ist_online else l["profil"])
            # online = wie vor dem 2026-10-16 (ohne Halteregel), onlinehalt = Recorder heute
            online = (execution.describe(
                "online", reach_tol_mm=None if l["profil"] == "online" else config.ONLINE_REACH_TOL_MM)
                if ist_online else None)
            halte = execution.hold_blocks(plan.gripper, plan.dwell_mask)
            if ist_online:
                l["online"] = online
            start = datetime.now().strftime("%H:%M:%S")
            try:
                d = fahre_lauf(bot, profil, l["rate"], online=online,
                               pause_s=args.pause if args.pause is not None else (3.0 if real else 0.0),
                               halte=halte, fk_pos=fk_pos)
            except Exception as exc:
                print("  [FEHLER] %s: %s" % (type(exc).__name__, exc))
                bericht["laeufe"][l["name"]] = {"lauf": l, "start": start,
                                                "fehler": "%s: %s" % (type(exc).__name__, exc)}
                if bot.stop_requested:
                    bot.clear_stop()
                continue
            k = auswerten_lauf(d["sendungen"], d["t"], d["q"], profil, l["rate"],
                               fk_pos, d["t_start"])
            if ist_online:
                k["halte"] = halt_kennzahlen(np.asarray(d["t"]) - d["t_start"], d["q"],
                                             plan.joints_noisy, halte, fk_pos, warte=d["warte"])
            e = {"lauf": l, "start": start, "kennzahlen": k, "ueberlaeufe": d["ueberlaeufe"],
                 "lesedauer_ms": d["lesedauer_ms"], "lesefehler": d["lesefehler"],
                 "ruhe_erreicht": d["ruhe"], "abbruch": d["abbruch"], "warte": d["warte"]}
            bericht["laeufe"][l["name"]] = e
            roh[l["name"] + "_t"] = d["t"] - d["t_start"]
            roh[l["name"] + "_q"] = d["q"]
            roh[l["name"] + "_send"] = d["sendungen"] - d["t_start"]
            print(zeile_kopf())
            print(zeile(l["name"], e))
            for z in halt_zeilen(l["name"], k.get("halte")):
                print(z)
            if bot.stop_requested:
                print("  Stopp angefordert -- Serie beendet.")
                break
        bot._call("set_override", 1.0)
        tabelle(bericht)
    finally:
        bot.close()
        if move:
            # Auch nach einem Abbruch sichern: die gefahrenen Laeufe zaehlen.
            BERICHTE.mkdir(exist_ok=True)
            stamm = BERICHTE / ("%s_Servo-Strom" % datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
            Path(str(stamm) + ".json").write_text(
                json.dumps(bericht, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
            if roh:
                np.savez_compressed(str(stamm) + "_Mitschrieb.npz", **roh)
            print("\nBericht: %s.json" % stamm)
            print("Hinweis: %s" % ("ANLAGENMESSUNG" if real else
                                   "VIRTUELLE Steuerung -- Umsetzung des Stroms, keine Mechanik."))


if __name__ == "__main__":
    main()

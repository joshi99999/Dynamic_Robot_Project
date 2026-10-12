"""Kann die Steuerung eine Bahn im Voraus puffern? (AP 1.3 / 4.1)

Anlass: Labortag 2026-10-05. Der Arm zitterte an der Anlage, weil die
Steuerung mit ``servo_j`` nur alle 16,7 ms einen Sollwert bekommt (60 Hz
statt der 1 kHz, mit denen Neura den Befehl rechnet): Sie faehrt jeden
Sollwert an und wartet dann. ``servo_j`` ueber neurapy kostet ~4,7 ms je
Aufruf (eine TCP-Verbindung je Aufruf), mehr Sollwerte gehen nicht.

Idee "rollender Puffer": EIN Aufruf je 15-Hz-Takt traegt die naechsten
200-300 ms der Bahn als zeitgestempelte Punkte (``move_trajectory``, nur in
den LARA-5-Beispielen der neurapy-Doku, NICHT in der Funktionsreferenz).
Die Steuerung interpoliert im 1-ms-Takt und hat immer Vorrat. Ob das
traegt, haengt an Fragen, die nur die Steuerung beantwortet -- dieses
Werkzeug stellt sie der Reihe nach:

1. Gibt es ``move_trajectory`` und verwandte Pufferbefehle?   (nur lesen)
2. Faehrt eine ganze Bahn damit glatter als heute mit servo_j? (referenz/komplett)
3. Verbindet die Steuerung duenne Stuetzpunkte (15 Hz) glatt?  (knoten15)
4. Schliesst ein zweiter Aufruf NAHTLOS an den laufenden an?   (anschluss)
   -- die entscheidende Frage fuer die Inferenz.
5. Was passiert, wenn der Puffer leerlaeuft?                   (leerlauf)
6. Leert ``stop()`` den Puffer sofort?                         (stopp)
7. Faehrt ``movelinear_online`` mit EINEM Ziel je 15-Hz-Takt    (online)
   durchgehend, oder pulsiert es im Takt? Die Steuerung plant selbst
   (Ziel + Hoechstwerte fuer v/a, laut get_doc das Gegenstueck zu
   servo_j/servo_x, die fuer einen 1-kHz-Strom gedacht sind).

Ausweichweg, falls ``move_trajectory`` fehlt: Nach der Referenz faehrt
dieselbe Bahn per servo_j mit ``--vergleich-raten`` (Standard 90,120 Hz) --
kuerzere Stufen, gleicher Befehl. In der VM 2026-10-12 stabil (3/3
Episoden je Rate, kein Fehlerzustand); ob es glatter wird, zeigt nur die
Anlage (die VM liefert nur ~50 neue Gelenkwerte je Sekunde).

Weil keine externe Messung (Beschleunigungssensor) vorgesehen ist, liest ein
eigener Thread waehrend jeder Fahrt die Gelenkwinkel so schnell wie moeglich
(~250 Hz an der Anlage). Die 15-Hz-Aufnahmen koennen das nicht -- siehe
Auswertung vom 2026-10-05. Kennzahlen, nach Belastbarkeit:

* Stillstandsanteil: Anteil der Messabstaende IN FAHRT, in denen sich der
  Winkel praktisch nicht aendert -- das Stop-and-go im servo_j-Takt.
  Haengt nicht an der Genauigkeit der Zeitstempel und ist deshalb das
  Hauptkriterium (Synthese: glatt 0 %, 60-Hz-Treppe ~53 %).
* Welligkeit: RMS(v_ist - v_plan) / v_max. Hat einen Rauschboden von
  ~5-15 %, weil der Zeitstempel nur die Mitte eines ~3,5-ms-Aufrufs ist --
  nur im Vergleich zur Referenz lesen.
* Staerkste Frequenz der Abweichung: nur bei deutlichem Befund angezeigt
  (sonst ist es die Frequenz des Rauschens).
* ``*_neu``: dieselben Kennzahlen nur aus den NEUEN Werten der Steuerung
  (Wiederholungen entfernt, zeitgewichtet). Aufloesung = halbe Quellrate:
  in der VM (50 Hz) sieht man so ein 15-Hz-Pulsieren, keine 60-Hz-Stufen.

BEWEGUNG: ohne ``--move`` wird nur gelesen (Punkt 1), nichts bestromt. Mit
``--move`` faehrt EIN Gelenk eine Kosinus-Hin-und-Rueckbewegung um die
aktuelle Stellung (VM 0.15 rad, Anlage 0.05 rad). Jede Bahn wird VOR dem
Senden geprueft: Abstand zur Startstellung <= Auslenkung, Achsgrenzen,
Geschwindigkeit <= config.SERVO_MAX_JOINT_SPEED_RADS. Damit bleibt der Arm
auch dann im kleinen Fenster, wenn die Steuerung die Zeitstempel anders
deutet als erwartet. An der Anlage nur mit ``--real-robot`` und
Freigabewort; Arm vorher am Pendant in eine freie Pose fahren. Der
Software-Stopp ersetzt keinen Not-Aus (AP 4.2).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_trajectory_buffer.py                    # nur lesen
    python tools/check_trajectory_buffer.py --move             # VM, alle Tests
    python tools/check_trajectory_buffer.py --move --tests referenz,komplett
    python tools/check_trajectory_buffer.py --move --tests referenz,online
    python tools/check_trajectory_buffer.py --move --real-robot --joint 1 --amplitude 0.05
"""

import argparse
import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.neura import NeuraRobot
from bc.clock import RealClock, host_time
from bc.servo import ServoInterpolator, substeps_for
from bc.sync import Pacer

BERICHTE = Path(__file__).resolve().parent.parent / "Berichte"

#: Pufferfaehige Befehle laut neurapy-Doku v5.0.8 -- und zum Vergleich die
#: Online-Befehle, die schon in der Auswertung vom 2026-10-05 standen.
KANDIDATEN = (
    "move_trajectory",        # Zeitstempel + Gelenkziele (nur Doku-Beispiel)
    "plan_move_joint",        # geplante Bewegung, mit executor() ausgefuehrt
    "executor",
    "wait_motion_finished",   # "motion queued" -> es gibt eine Warteschlange
    "motion_status",
    "movelinear_online",      # Loesung 1: Interpolation in der Steuerung
    "stop_movelinear_online",
    "servo_x",
    "move_joint",
)

TESTS = ("referenz", "komplett", "knoten15", "anschluss", "leerlauf", "stopp", "online")

#: Tests, die move_trajectory brauchen (fehlt in der VM v5.0.0-alpha.102).
TESTS_PUFFER = ("komplett", "knoten15", "anschluss", "leerlauf", "stopp")

#: Obergrenze der Auslenkung -- das Werkzeug soll Fragen beantworten, nicht
#: fahren. An der Anlage enger als in der VM.
MAX_AMPLITUDE_SIM = 0.30
MAX_AMPLITUDE_REAL = 0.10

#: Raster der Auswertung (Gelenkwinkel werden darauf interpoliert).
AUSWERT_HZ = 250.0

#: Mindest-Leserate des Mitschriebs: 60-Hz-Stufen (16,7 ms) brauchen mehrere
#: Messpunkte je Stufe. In der VM kam der Mitschrieb am 2026-10-08 nur auf
#: 53 Hz (18,9 ms je Lesung) -- dort ist der Stillstandsanteil ein Artefakt
#: wiederholter Lesungen, keine Messung.
MIN_LESERATE_HZ = 150.0

#: Schnelles Lesen allein reicht nicht: Die STEUERUNG muss auch so oft neue
#: Werte liefern. Die VM aktualisiert die Gelenkwinkel nur alle ~20 ms
#: (2026-10-12: 573 Hz gelesen, 50 neue Werte je Sekunde) -- 91 % der
#: Lesungen waren Wiederholungen und wurden als "Stillstand" gezaehlt.
#: Kommen weniger neue Werte als servo_j-Stufen, ist eine Stufe nicht von
#: "noch kein neuer Wert" zu unterscheiden. Echtes Stop-and-go liefert
#: mindestens einen neuen Wert je Stufe; 10 % Spielraum fuer Takt-Jitter.
NEUE_WERTE_ANTEIL = 0.9

#: Ab welchem Faktor eine Fahrt als zeitlich gestreckt gilt (Rechner oder
#: Steuerung haelt den Takt nicht, VM 2026-10-08: 4-s-Bahn in ~4,9 s).
MAX_DAUER_FAKTOR = 1.10

#: Ab welchem Anteil der geplanten Spitzengeschwindigkeit ein Abschnitt als
#: "in Fahrt" zaehlt (Welligkeit und Stillstand werden nur dort bewertet).
FAHRT_ANTEIL = 0.3


def section(titel):
    print("\n" + "=" * 70)
    print(titel)
    print("=" * 70)


def show(label, value):
    print("  %-40s %s" % (label, value))


# -- Testbahn und Stuetzpunkte ---------------------------------------------


class Testbahn(object):
    """q(t) = q0 + A (1 - cos(2 pi t / T)) / 2 auf EINEM Gelenk.

    Beginnt und endet in q0 mit Geschwindigkeit 0; die Spitzengeschwindigkeit
    A pi / T liegt bei T/4 (hin) und 3T/4 (zurueck). Ausserhalb [0, T]
    steht sie in q0.
    """

    def __init__(self, q0, joint, amplitude, dauer):
        self.q0 = np.asarray(q0, dtype=float)
        self.joint = int(joint)
        self.amplitude = float(amplitude)
        self.dauer = float(dauer)

    @property
    def v_max(self):
        return abs(self.amplitude) * math.pi / self.dauer

    def qj(self, t):
        """Sollwinkel des Testgelenks, vektorisiert."""
        t = np.clip(np.asarray(t, dtype=float), 0.0, self.dauer)
        return self.q0[self.joint] + self.amplitude * (
            1.0 - np.cos(2.0 * math.pi * t / self.dauer)) / 2.0

    def vj(self, t):
        t = np.asarray(t, dtype=float)
        v = self.amplitude * math.pi / self.dauer * np.sin(2.0 * math.pi * t / self.dauer)
        return np.where((t >= 0.0) & (t <= self.dauer), v, 0.0)

    def q(self, t):
        """Alle Gelenke zur Zeit ``t`` (Skalar)."""
        out = self.q0.copy()
        out[self.joint] = float(self.qj(t))
        return out

    def knoten(self, t_von, t_bis, hz):
        """Stuetzpunkte fuer (t_von, t_bis] mit ``hz``.

        Rueckgabe ``(rel, punkte)``: Zeitstempel RELATIV zu ``t_von``
        (erster > 0, letzter genau ``t_bis - t_von``), wie im Doku-Beispiel
        von ``move_trajectory`` (0.01, 0.02, ...), und die Gelenkziele.
        Aneinandergereihte Abschnitte ergeben lueckenlos dieselbe Bahn.
        """
        laenge = float(t_bis) - float(t_von)
        if laenge <= 0:
            raise ValueError("leerer Abschnitt (%.3f..%.3f s)" % (t_von, t_bis))
        n = max(1, int(round(laenge * float(hz))))
        rel = laenge * np.arange(1, n + 1) / n
        punkte = [self.q(t_von + r) for r in rel]
        return rel, punkte


def abschnitte(dauer, chunk_s):
    """Zerlegt [0, dauer] in aufeinanderfolgende Abschnitte (von, bis)."""
    out = []
    t = 0.0
    while t < dauer - 1e-9:
        bis = min(dauer, t + chunk_s)
        out.append((t, bis))
        t = bis
    return out


def pruefe_bahn(q_start, rel, punkte, q0, amplitude,
                limits=config.JOINT_LIMITS_RAD,
                v_max=config.SERVO_MAX_JOINT_SPEED_RADS):
    """Wirft ValueError, wenn die Bahn das Testfenster verlaesst.

    Geprueft wird die Geometrie, nicht das Timing der Steuerung: Selbst wenn
    sie die Zeitstempel ignoriert, bleibt der Arm zwischen diesen Punkten.
    """
    q_start = np.asarray(q_start, dtype=float)
    q0 = np.asarray(q0, dtype=float)
    pts = np.asarray(punkte, dtype=float)
    rel = np.asarray(rel, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != q0.size or len(rel) != len(pts):
        raise ValueError("Bahn hat die Form %s, erwartet (n, %d)" % (pts.shape, q0.size))
    if not np.all(np.isfinite(pts)) or not np.all(np.isfinite(rel)):
        raise ValueError("Bahn enthaelt nicht endliche Werte")
    if np.any(np.diff(np.concatenate([[0.0], rel])) <= 0):
        raise ValueError("Zeitstempel nicht streng steigend")
    abstand = float(np.max(np.abs(pts - q0)))
    if abstand > abs(amplitude) + 1e-6:
        raise ValueError("Bahn entfernt sich %.4f rad von der Startstellung "
                         "(Grenze %.4f)" % (abstand, abs(amplitude)))
    if limits is not None:
        for i, (lo, hi) in enumerate(limits):
            if np.any(pts[:, i] < lo) or np.any(pts[:, i] > hi):
                raise ValueError("Gelenk %d verlaesst [%.2f, %.2f]" % (i + 1, lo, hi))
    kette = np.vstack([q_start, pts])
    dt = np.diff(np.concatenate([[0.0], rel]))
    tempo = np.max(np.abs(np.diff(kette, axis=0)), axis=1) / dt
    if float(np.max(tempo)) > v_max + 1e-9:
        k = int(np.argmax(tempo))
        raise ValueError("Schritt %d verlangt %.2f rad/s (Grenze %.2f)"
                         % (k, tempo[k], v_max))


# -- Mitschrieb der Gelenkwinkel --------------------------------------------


class Mitschrieb(object):
    """Liest in einem eigenen Thread fortlaufend die Gelenkwinkel.

    Zeitstempel = Mitte des Aufrufs (wie NeuraRobot.get_joint_angles_ts).
    Jeder Aufruf oeffnet eine eigene Verbindung -- blockiert die Steuerung
    waehrend einer Bewegung, sieht man das hier als Luecke (selbst ein
    Befund: dann kann der Recorder waehrend eines blockierenden Befehls
    keinen Zustand lesen).
    """

    def __init__(self, lesen):
        self._lesen = lesen
        self._lauf = threading.Event()
        self._thread = None
        self.t = []
        self.q = []
        self.dauern = []
        self.fehler = 0

    def start(self):
        self._lauf.set()
        self._thread = threading.Thread(target=self._schleife, name="mitschrieb", daemon=True)
        self._thread.start()
        return self

    def _schleife(self):
        while self._lauf.is_set():
            t0 = host_time()
            try:
                q = self._lesen()
            except Exception:
                self.fehler += 1
                time.sleep(0.005)
                continue
            t1 = host_time()
            self.t.append(0.5 * (t0 + t1))
            self.dauern.append(t1 - t0)
            self.q.append([float(v) for v in q])

    def lesedauer_ms(self):
        """Median der Aufrufdauer -- so unsicher ist jeder Zeitstempel."""
        return 1e3 * float(np.median(self.dauern)) if self.dauern else None

    def stop(self):
        self._lauf.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        return np.asarray(self.t, dtype=float), np.asarray(self.q, dtype=float)


# -- Auswertung (hardwarefrei, getestet in tests/test_check_trajectory_buffer.py)


def _gleitmittel(x, n):
    if n <= 1:
        return x
    kern = np.ones(n) / n
    pad = n // 2
    xp = np.concatenate([np.full(pad, x[0]), x, np.full(n - 1 - pad, x[-1])])
    return np.convolve(xp, kern, mode="valid")


def verzug(t_rel, qj, bahn, suche=(-0.1, 0.6), schritt=0.002):
    """Zeitversatz tau, mit dem q_ist(t) am besten q_plan(t - tau) folgt."""
    best = (float("inf"), 0.0)
    for tau in np.arange(suche[0], suche[1] + 1e-12, schritt):
        maske = (t_rel - tau >= 0.0) & (t_rel - tau <= bahn.dauer)
        if maske.sum() < 10:
            continue
        fehler = float(np.sqrt(np.mean((qj[maske] - bahn.qj(t_rel[maske] - tau)) ** 2)))
        if fehler < best[0]:
            best = (fehler, float(tau))
    return best[1], best[0]


def _kennzahlen_neu(rel, qj, neu, bahn, tau):
    """Welligkeit und Stillstand nur aus den NEUEN Werten der Steuerung.

    Wiederholte Lesungen werden verworfen, jeder neue Wert zaehlt ab dem
    Zeitpunkt, zu dem er zuerst gelesen wurde; gewichtet wird mit der
    Zeit bis zum naechsten neuen Wert. Ein kurzer Halt des Arms erscheint
    damit nicht als Stillstand (die Wiederholungen sind ja entfernt),
    sondern als Tempo-EINBRUCH im Abstand, der ihn ueberspannt -- deshalb
    "Anteil der Fahrzeit unter 50 % des Plantempos" statt Stillstand.
    Aufloesbar ist nur, was langsamer als die halbe Quellrate ist.
    """
    idx = np.concatenate([[0], np.flatnonzero(neu) + 1])
    if len(idx) < 10:
        return {}
    tn, qn = rel[idx], qj[idx]
    dtn = np.diff(tn)
    vn = np.diff(qn) / np.maximum(dtn, 1e-6)
    vpn = bahn.vj(0.5 * (tn[1:] + tn[:-1]) - tau)
    fahrt = np.abs(vpn) > FAHRT_ANTEIL * bahn.v_max
    if not fahrt.any():
        return {}
    w = dtn[fahrt]
    return {
        "welligkeit_neu_prozent": float(100.0 * math.sqrt(
            float(np.sum(w * (vn[fahrt] - vpn[fahrt]) ** 2)) / float(np.sum(w))) / bahn.v_max),
        "einbruch_neu_prozent": float(100.0 * float(np.sum(
            w[np.abs(vn[fahrt]) < 0.5 * np.abs(vpn[fahrt])])) / float(np.sum(w))),
        "tempo_p5_anteil": float(np.percentile(vn[fahrt] / vpn[fahrt], 5)),
    }


def auswerten(t, q, t_start, bahn, grenzen=()):
    """Kennzahlen einer Fahrt des Testgelenks.

    ``t``/``q``: Mitschrieb (Host-Zeit, alle Gelenke). ``t_start``: Host-Zeit,
    zu der die Bahn bei t = 0 beginnt. ``grenzen``: Zeitpunkte (relativ zur
    Bahn), an denen ein neuer Aufruf anschliesst -- dort wird die
    Ist-Geschwindigkeit gegen die geplante gesetzt.
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    out = {"proben": int(len(t))}
    if len(t) < 20:
        out["ungueltig"] = "zu wenige Messpunkte (%d)" % len(t)
        return out
    j = bahn.joint
    rel = t - t_start
    qj = q[:, j]
    abstaende = np.diff(t)
    out["leserate_hz"] = float(1.0 / np.median(abstaende))
    out["groesste_luecke_ms"] = float(1e3 * np.max(abstaende))

    suche = (-0.1, 0.6)
    tau, rms = verzug(rel, qj, bahn, suche=suche)
    out["verzug_ms"] = 1e3 * tau
    out["folgefehler_rms_mrad"] = 1e3 * rms
    # Liegt der beste Versatz am Rand des Suchbereichs, hat die Ist-Bahn
    # eine andere ZEITACHSE als die Planung (gestreckt) -- dann sind
    # Verzug, Folgefehler und Welligkeit nicht deutbar.
    out["verzug_am_rand"] = bool(tau <= suche[0] + 0.004 or tau >= suche[1] - 0.004)
    out["auslenkung_rad"] = float(np.max(np.abs(qj - bahn.q0[j])))
    # Zeitstreckung: wie lange war der Arm ausgelenkt, gegen die Planung?
    schwelle = 0.02 * abs(bahn.amplitude)
    ist_aus = np.flatnonzero(np.abs(qj - bahn.q0[j]) > schwelle)
    fein = np.linspace(0.0, bahn.dauer, 4001)
    plan_aus = fein[np.abs(bahn.qj(fein) - bahn.q0[j]) > schwelle]
    if len(ist_aus) > 1 and len(plan_aus) > 1:
        out["dauer_faktor"] = float((rel[ist_aus[-1]] - rel[ist_aus[0]])
                                    / (plan_aus[-1] - plan_aus[0]))

    # Geschwindigkeit auf einem 4-ms-Raster, zentrale Differenz (Basis 8 ms)
    # und bewusst NICHT weiter geglaettet: jedes Fenster ueber ~8 ms buegelt
    # genau die Stufen von 16,7 ms (servo_j mit 60 Hz) weg, die hier
    # gesucht werden (erste Fassung mit 20 ms sah die Treppe nicht).
    dt = 1.0 / AUSWERT_HZ
    raster = np.arange(rel[0], rel[-1], dt)
    qg = np.interp(raster, rel, qj)
    v_ist = np.gradient(qg, dt)
    v_plan = bahn.vj(raster - tau)
    in_fahrt = np.abs(v_plan) > FAHRT_ANTEIL * bahn.v_max
    if in_fahrt.sum() < 10:
        out["ungueltig"] = "keine Fahrt im Mitschrieb"
        return out
    abw = v_ist - v_plan
    out["welligkeit_prozent"] = float(100.0 * np.sqrt(np.mean(abw[in_fahrt] ** 2)) / bahn.v_max)
    # Stillstand direkt aus den einzelnen Messabstaenden: Steht der Arm,
    # aendert sich der Winkel nicht -- unabhaengig davon, wie ungenau der
    # Zeitstempel ist. Das Raster wuerde Halte- und Fahrphase vermischen.
    mitte = 0.5 * (rel[1:] + rel[:-1])
    v_roh = np.diff(qj) / np.maximum(np.diff(rel), 1e-6)
    v_plan_roh = bahn.vj(mitte - tau)
    fahrt_roh = np.abs(v_plan_roh) > FAHRT_ANTEIL * bahn.v_max
    out["stillstand_prozent"] = float(100.0 * np.mean(
        np.abs(v_roh[fahrt_roh]) < 0.1 * np.abs(v_plan_roh[fahrt_roh]))) if fahrt_roh.any() else 0.0
    # Neue Werte je Sekunde in Fahrt: eine Lesung zaehlt als neu, wenn sich
    # IRGENDEIN Gelenk geaendert hat. Eine wiederholte Lesung ist bitgleich
    # -- "Steuerung hat noch nichts Neues" oder "Arm steht exakt", aus den
    # Lesungen allein nicht zu trennen (siehe NEUE_WERTE_ANTEIL).
    if fahrt_roh.any():
        neu = np.any(np.diff(q, axis=0) != 0.0, axis=1)
        dauer_fahrt = float(np.sum(np.diff(rel)[fahrt_roh]))
        out["neue_werte_hz"] = float(np.sum(neu[fahrt_roh]) / max(dauer_fahrt, 1e-9))
        out["wiederholt_prozent"] = float(100.0 * np.mean(~neu[fahrt_roh]))
        out.update(_kennzahlen_neu(rel, qj, neu, bahn, tau))
    out["v_ist_max"] = float(np.max(np.abs(v_ist)))
    out["v_plan_max"] = float(bahn.v_max)

    # Staerkste Frequenz der Abweichung in der Fahrt (5 Hz .. Nyquist):
    # 15 Hz = Taktknicke, 60 Hz = servo_j-Treppe.
    idx = np.flatnonzero(in_fahrt)
    stueck = abw[idx[0]:idx[-1] + 1]
    if len(stueck) >= 32:
        stueck = (stueck - stueck.mean()) * np.hanning(len(stueck))
        f = np.fft.rfftfreq(len(stueck), dt)
        p = np.abs(np.fft.rfft(stueck)) ** 2
        band = f >= 5.0
        if band.any() and p[band].sum() > 0:
            k = np.flatnonzero(band)[int(np.argmax(p[band]))]
            out["spitze_hz"] = float(f[k])
            out["spitze_aussagekraeftig"] = bool(
                out["stillstand_prozent"] > 10.0 or out["welligkeit_prozent"] > 20.0)
            out["anteil_ueber_5hz_prozent"] = float(100.0 * p[band].sum() / max(p[1:].sum(), 1e-30))

    # Ende der Bewegung: letzter Zeitpunkt deutlich in Fahrt.
    bewegt = np.flatnonzero(np.abs(v_ist) > 0.05 * bahn.v_max)
    if len(bewegt):
        out["ende_s"] = float(raster[bewegt[-1]])

    if len(grenzen):
        verh = []
        for g in grenzen:
            fenster = np.abs(raster - (g + tau)) <= 0.02
            if not fenster.any():
                continue
            soll = abs(float(bahn.vj(g)))
            if soll < FAHRT_ANTEIL * bahn.v_max:
                continue
            # Mittel statt Minimum: im Anschluss soll ein HALT auffallen,
            # nicht eine einzelne langsame Probe.
            verh.append(float(np.mean(np.abs(v_ist[fenster]))) / soll)
        if verh:
            out["anschluss_min_verhaeltnis"] = float(min(verh))
            out["anschluss_verhaeltnisse"] = verh
    return out


def aussagekraft(k, servo_hz=config.SERVO_RATE_HZ):
    """Gruende, warum die Kennzahlen einer Fahrt NICHT deutbar sind (Liste).

    ``servo_hz``: Rate der servo_j-Stufen, die aufgeloest werden sollen.
    """
    gruende = []
    if k.get("leserate_hz", 0.0) < MIN_LESERATE_HZ:
        gruende.append("Mitschrieb nur %.0f Hz (mindestens %.0f) -- 60-Hz-Stufen sind "
                       "nicht aufloesbar, der Stillstand zaehlt wiederholte Lesungen"
                       % (k.get("leserate_hz", 0.0), MIN_LESERATE_HZ))
    elif k.get("neue_werte_hz") is not None and k["neue_werte_hz"] < NEUE_WERTE_ANTEIL * servo_hz:
        gruende.append("Steuerung liefert nur %.0f neue Gelenkwerte je Sekunde, weniger als "
                       "servo_j-Stufen (%g Hz) -- %.0f %% der Lesungen sind Wiederholungen, der "
                       "Stillstand zaehlt sie mit; Stufen und Glaette sind hier nicht messbar"
                       % (k["neue_werte_hz"], servo_hz, k.get("wiederholt_prozent", 0.0)))
    if k.get("dauer_faktor", 1.0) > MAX_DAUER_FAKTOR:
        gruende.append("Bahn lief %.2f-fach langsamer als geplant -- der Sollwertstrom "
                       "kam zu spaet (Rechner/Steuerung haelt den Takt nicht)"
                       % k["dauer_faktor"])
    if k.get("verzug_am_rand"):
        gruende.append("Verzug am Rand des Suchbereichs (%.0f ms) -- Zeitachse passt "
                       "nicht zur Planung" % k.get("verzug_ms", 0.0))
    return gruende


def zeile(name, k):
    """Eine Ergebniszeile fuer die Vergleichstabelle."""
    if "ungueltig" in k:
        return "  %-12s %s" % (name, k["ungueltig"])
    return "  %-12s %7.1f %% %7.1f %% %8s %7.0f ms %8.2f %8.0f Hz %7.0f Hz %7.1f ms" % (
        name, k["welligkeit_prozent"], k["stillstand_prozent"],
        ("%.0f Hz" % k["spitze_hz"]) if k.get("spitze_aussagekraeftig") else "-",
        k["verzug_ms"], k["folgefehler_rms_mrad"], k["leserate_hz"],
        k.get("neue_werte_hz", float("nan")), k["groesste_luecke_ms"]) + (
        "   | neu: Welligk. %5.1f %%  Einbruch %5.1f %%  Tempo p5 %4.2f" % (
            k["welligkeit_neu_prozent"], k["einbruch_neu_prozent"], k["tempo_p5_anteil"])
        if "welligkeit_neu_prozent" in k else "")


# -- Laufzeit gegen die Steuerung -------------------------------------------


class Lauf(object):
    """Haelt Verbindung, Bericht und Mitschriebe eines Werkzeuglaufs."""

    def __init__(self, bot, bahn, args):
        self.bot = bot
        self.bahn = bahn
        self.args = args
        self.bericht = {"tests": {}, "befunde": []}
        self.roh = {}
        self.nonblocking = None   # unbekannt, bis 'anschluss' es zeigt
        self._log = None

    def befund(self, stufe, text):
        self.bericht["befunde"].append([stufe, text])
        print("  [%s] %s" % (stufe.upper(), text))

    def an_den_start(self):
        """Vor jedem Test an q0 -- ausserhalb der Messung, langsam (PTP)."""
        ist = np.asarray(self.bot.get_joint_angles(), dtype=float)
        if float(np.max(np.abs(ist - self.bahn.q0))) > config.START_POSE_TOL_RAD:
            print("  fahre zurueck an die Startstellung (%.4f rad entfernt)"
                  % float(np.max(np.abs(ist - self.bahn.q0))))
            self.bot.move_to_joints(self.bahn.q0)
        time.sleep(0.3)

    def mitschreiben(self):
        self._log = Mitschrieb(self.bot.get_joint_angles).start()
        return self._log

    def sichern(self, name, t, q, t_start, kennzahlen, extra=None):
        eintrag = {"kennzahlen": kennzahlen,
                   "lesedauer_ms": self._log.lesedauer_ms() if self._log else None,
                   "lesefehler": self._log.fehler if self._log else None}
        eintrag.update(extra or {})
        self.bericht["tests"][name] = eintrag
        self.roh[name + "_t"] = t - t_start
        self.roh[name + "_q"] = q

    def trajektorie(self, rel, punkte, q_start, blocking=None):
        """Ein ``move_trajectory``-Aufruf nach Pruefung der Bahn.

        ``blocking=None`` laesst den Parameter weg (Doku-Beispiel). Liefert
        (Rueckgabe, Aufrufdauer s, Fehlertext oder None).
        """
        pruefe_bahn(q_start, rel, punkte, self.bahn.q0, self.bahn.amplitude)
        kwargs = {
            "current_joint_angles": [float(v) for v in q_start],
            "timestamps": [float(v) for v in rel],
            "target_joint": [[float(v) for v in p] for p in punkte],
        }
        if blocking is not None:
            kwargs["blocking"] = bool(blocking)
        t0 = host_time()
        try:
            res = self.bot._call("move_trajectory", **kwargs)
            return res, host_time() - t0, None
        except Exception as exc:
            return None, host_time() - t0, "%s: %s" % (type(exc).__name__, exc)

    def online(self):
        """movelinear_online: ein kartesisches Ziel je 15-Hz-Takt.

        Ziel = FK(q_plan(t + Vorlauf)). Die Steuerung plant selbst mit den
        mitgegebenen HOECHSTWERTEN -- zu knapp, und der Arm haengt nach;
        zu grosszuegig, und er erreicht jedes Ziel vor dem naechsten und
        bremst (Pulsieren im Takt). Beides zeigt die Auswertung.

        Sicherheit: Die Gelenkbahn wird wie alle anderen vorher mit
        pruefe_bahn geprueft. Zwischen zwei Zielen faehrt die Steuerung
        eine Gerade im Raum; bei 66 ms Abstand weicht sie von der Gelenkbahn
        um Bruchteile eines Millimeters ab.
        """
        args = self.args
        section("2c. online: movelinear_online, 1 Ziel je Takt, Vorlauf %d Takt(e)"
                % args.online_vorlauf)
        self.an_den_start()
        bahn = self.bahn
        dt = 1.0 / config.CONTROL_RATE_HZ
        vorlauf = args.online_vorlauf * dt
        n = int(round(bahn.dauer * config.CONTROL_RATE_HZ))
        zeiten = np.arange(1, n + 1 + int(config.CONTROL_RATE_HZ // 2)) * dt
        ziele_q = [bahn.q(min(t + vorlauf, bahn.dauer)) for t in zeiten]
        pruefe_bahn(bahn.q0, zeiten, ziele_q, bahn.q0, bahn.amplitude)
        ziele = [np.asarray(self.bot.fk(qz), dtype=float) for qz in ziele_q]
        # Hoechstwerte aus der Bahn selbst: Tempo des TCP und der Drehung.
        fein = np.linspace(0.0, bahn.dauer, 201)
        posen = np.asarray([self.bot.fk(bahn.q(t)) for t in fein])
        v_tcp = float(np.max(np.linalg.norm(np.diff(posen[:, :3], axis=0), axis=1))
                      / (fein[1] - fein[0]))
        v_rot = bahn.v_max   # J1 dreht das Werkzeug mit derselben Rate
        a_tcp = v_tcp * 2.0 * math.pi / bahn.dauer
        a_rot = v_rot * 2.0 * math.pi / bahn.dauer
        vel = [args.online_tempo * v_tcp] * 3 + [args.online_tempo * v_rot] * 3
        acc = [args.online_beschl * a_tcp] * 3 + [args.online_beschl * a_rot] * 3
        show("TCP-Spitze geplant", "%.4f m/s, Drehung %.3f rad/s" % (v_tcp, v_rot))
        show("Hoechstwerte v / a", "x%.2f / x%.1f der Bahn" % (args.online_tempo, args.online_beschl))
        codes, fehler = [], None
        log = self.mitschreiben()
        self.bot.activate_servo("position")
        pacer = Pacer(RealClock(), config.CONTROL_RATE_HZ).start()
        t_start = None
        try:
            for ziel in ziele:
                t_soll = pacer.tick()
                if t_start is None:
                    t_start = t_soll
                try:
                    codes.append(self.bot._call("movelinear_online",
                                                [float(v) for v in ziel], vel, acc))
                except Exception as exc:
                    fehler = "%s: %s" % (type(exc).__name__, exc)
                    break
            if fehler is None:
                self.warten_bis_ruhe(0.5, 3.0)
        finally:
            self.bot._call_safe("stop_movelinear_online")
            self.bot.deactivate_servo()
        time.sleep(0.3)
        t, q = log.stop()
        k = auswerten(t, q, t_start, bahn)
        # VM v5.0.0-alpha.102 (2026-10-12): Aufruf angenommen, Rueckgabe
        # None, der Arm bewegt sich nicht -- auch nicht mit einem Einzelziel
        # und den Werten aus dem Doku-Beispiel. Das ist kein "nicht deutbar",
        # sondern ein klares Nein fuer diese Steuerung.
        bewegt = k.get("auslenkung_rad", 0.0) >= 0.1 * abs(bahn.amplitude)
        werte = sorted(set(repr(c) for c in codes))
        self.sichern("online", t, q, t_start, k, {
            "taktueberlaeufe": pacer.overruns, "servo_hz": config.CONTROL_RATE_HZ,
            "codes": werte[:10], "aufrufe": len(codes), "fehler": fehler,
            "vorlauf_takte": args.online_vorlauf, "tempo_faktor": args.online_tempo,
            "beschl_faktor": args.online_beschl, "v_tcp_ms": v_tcp, "bewegt": bewegt,
        })
        show("Aufrufe / Rueckgabewerte", "%d / %s" % (len(codes), ", ".join(werte[:5])))
        show("Taktueberlaeufe", pacer.overruns)
        if fehler:
            self.befund("fehler", "online: movelinear_online abgelehnt: %s" % fehler)
        elif not bewegt:
            self.befund("fehler", "online: movelinear_online angenommen (Rueckgabe %s), aber "
                        "der Arm bewegt sich nicht (%.4f rad statt %.3f) -- diese Steuerung "
                        "fuehrt den Befehl nicht aus." % (", ".join(werte[:3]) or "-",
                                                         k.get("auslenkung_rad", 0.0),
                                                         abs(bahn.amplitude)))
        return k

    def warten_bis_ruhe(self, mindestens_s, max_s=None):
        """Schlaeft mindestens ``mindestens_s``; danach bis der Arm steht."""
        time.sleep(max(0.0, mindestens_s))
        ende = host_time() + (max_s if max_s is not None else 3.0)
        letzte = np.asarray(self.bot.get_joint_angles(), dtype=float)
        while host_time() < ende:
            time.sleep(0.1)
            jetzt = np.asarray(self.bot.get_joint_angles(), dtype=float)
            if float(np.max(np.abs(jetzt - letzte))) < 2e-4:
                return
            letzte = jetzt

    # -- die Tests -------------------------------------------------------

    def referenz(self, rate=None, name="referenz"):
        """Heute: 15-Hz-Ziele, linear mit ``rate`` (Standard --servo-rate) ueber servo_j.

        Mit einer hoeheren ``rate`` ist es der billigste Ausweichweg, falls
        move_trajectory fehlt: kuerzere Stufen, gleicher Befehl (Rechnung
        in Dokumentation/Taktzeit-und-RPC-Latenz.md, Abschnitt 11).
        """
        rate = float(rate if rate is not None else self.args.servo_rate)
        section("2a. %s: servo_j wie im Recorder (%g Hz, linear je Takt)" % (name, rate))
        self.an_den_start()
        bahn = self.bahn
        interp = ServoInterpolator(config.CONTROL_RATE_HZ, rate).reset(bahn.q0)
        n = int(round(bahn.dauer * config.CONTROL_RATE_HZ))
        log = self.mitschreiben()
        self.bot.activate_servo("position")
        pacer = Pacer(RealClock(), rate).start()
        t_start = None
        try:
            for k in range(1, n + 1 + int(config.CONTROL_RATE_HZ // 2)):
                ziel = bahn.q(k / config.CONTROL_RATE_HZ)
                for q_cmd, v_cmd, a_cmd in interp.window(ziel):
                    t_soll = pacer.tick()
                    if t_start is None:
                        t_start = t_soll - 1.0 / rate
                    self.bot.servo_j(q_cmd, v_cmd, a_cmd)
        finally:
            self.bot.deactivate_servo()
        time.sleep(0.3)
        t, q = log.stop()
        k = auswerten(t, q, t_start, bahn)
        self.sichern(name, t, q, t_start, k, {"taktueberlaeufe": pacer.overruns,
                                              "servo_hz": rate})
        show("Taktueberlaeufe", pacer.overruns)
        return k

    def komplett(self, name, knoten_hz):
        """Die ganze Bahn in EINEM move_trajectory-Aufruf."""
        section("2b. %s: ganze Bahn in einem Aufruf, Stuetzpunkte %g Hz" % (name, knoten_hz))
        self.an_den_start()
        bahn = self.bahn
        rel, punkte = bahn.knoten(0.0, bahn.dauer, knoten_hz)
        show("Stuetzpunkte", len(punkte))
        log = self.mitschreiben()
        t_start = host_time()
        res, dauer, fehler = self.trajektorie(rel, punkte, bahn.q0)
        show("Rueckgabe", res if fehler is None else fehler)
        show("Aufruf dauerte", "%.2f s (Bahn %.2f s)" % (dauer, bahn.dauer))
        blockiert = dauer > 0.5 * bahn.dauer
        if fehler is None:
            self.warten_bis_ruhe(0.5 if blockiert else bahn.dauer - dauer + 0.5)
        t, q = log.stop()
        k = auswerten(t, q, t_start, bahn)
        self.sichern(name, t, q, t_start, k, {
            "fehler": fehler, "rueckgabe": repr(res), "aufruf_s": dauer,
            "blockierend": blockiert, "stuetzpunkte": len(punkte), "knoten_hz": knoten_hz})
        if fehler:
            self.befund("fehler", "move_trajectory abgelehnt (%s): %s" % (name, fehler))
        else:
            self.befund("info", "move_trajectory %s -- der Aufruf kehrt %s zurueck (%.2f s)."
                        % (name, "erst nach der Fahrt" if blockiert else "sofort", dauer))
            if k.get("groesste_luecke_ms", 0) > 200 and blockiert:
                self.befund("warnung",
                            "Waehrend des blockierenden Aufrufs kam %.0f ms lang kein "
                            "Messwert -- die Steuerung bedient parallel keine Abfragen. "
                            "Fuer den rollenden Puffer muss der Befehl nicht blockierend "
                            "laufen." % k["groesste_luecke_ms"])
        return k

    def anschluss(self):
        """Rollender Puffer: Abschnitte nacheinander, jeder VOR dem Ende des vorigen."""
        bahn = self.bahn
        chunk = self.args.chunk_takte / config.CONTROL_RATE_HZ
        vorlauf = self.args.vorlauf_takte / config.CONTROL_RATE_HZ
        section("3. Anschluss: Abschnitte zu %.0f ms, jeder %.0f ms vor Ende des vorigen"
                % (1e3 * chunk, 1e3 * vorlauf))
        self.an_den_start()
        teile = abschnitte(bahn.dauer, chunk)
        log = self.mitschreiben()
        t_start = host_time()
        aufrufe = []
        blocking = False
        for i, (von, bis) in enumerate(teile):
            senden = t_start + max(0.0, von - vorlauf)
            warte = senden - host_time()
            if warte > 0:
                time.sleep(warte)
            rel, punkte = bahn.knoten(von, bis, self.args.knoten_hz)
            q_von = bahn.q(von)
            res, dauer, fehler = self.trajektorie(rel, punkte, q_von, blocking=blocking)
            if i == 0 and fehler and "blocking" in fehler.lower():
                # Parameter unbekannt: ohne ihn (blockierend) weiter.
                blocking = None
                res, dauer, fehler = self.trajektorie(rel, punkte, q_von, blocking=None)
            aufrufe.append({"abschnitt": [von, bis], "gesendet_s": senden - t_start,
                            "aufruf_s": dauer, "fehler": fehler, "rueckgabe": repr(res)})
            print("  Abschnitt %d (%.2f-%.2f s): %s, Aufruf %.0f ms"
                  % (i + 1, von, bis, "Fehler: " + fehler if fehler else "angenommen", 1e3 * dauer))
            if fehler:
                break
        nonblocking = blocking is False and all(a["aufruf_s"] < 0.5 * chunk for a in aufrufe
                                                 if not a["fehler"])
        self.nonblocking = nonblocking
        rest = bahn.dauer - (host_time() - t_start)
        self.warten_bis_ruhe(rest + 0.5)
        t, q = log.stop()
        grenzen = [von for von, _ in teile[1:]]
        k = auswerten(t, q, t_start, bahn, grenzen=grenzen)
        self.sichern("anschluss", t, q, t_start, k, {
            "aufrufe": aufrufe, "nicht_blockierend": nonblocking,
            "chunk_s": chunk, "vorlauf_s": vorlauf})

        fehler = [a for a in aufrufe if a["fehler"]]
        if fehler:
            self.befund("fehler", "Abschnitt %d wurde abgelehnt (%s) -- die Steuerung "
                        "nimmt waehrend einer Fahrt keinen weiteren move_trajectory an."
                        % (len(aufrufe), fehler[0]["fehler"]))
            return k
        if not nonblocking:
            self.befund("warnung", "move_trajectory laeuft nur blockierend -- ein neuer "
                        "Abschnitt kann erst nach dem Ende des vorigen gesendet werden.")
        verh = k.get("anschluss_min_verhaeltnis")
        ende = k.get("ende_s")
        if verh is not None:
            show("Ist/Soll-Tempo an den Anschluessen (min)", "%.0f %%" % (100 * verh))
        if ende is not None:
            show("Bewegung endet bei", "%.2f s (Bahn %.2f s + Verzug %.0f ms)"
                 % (ende, bahn.dauer, k.get("verzug_ms", 0)))
        if verh is not None and verh >= 0.7 and nonblocking:
            self.befund("ok", "Abschnitte schliessen nahtlos an (Tempo an den "
                        "Anschluessen min. %.0f %% der Planung)." % (100 * verh))
        elif verh is not None:
            self.befund("warnung", "An den Anschluessen bricht das Tempo auf %.0f %% "
                        "ein -- die Steuerung haelt zwischen zwei Aufrufen an oder "
                        "verwirft den Rest." % (100 * verh))
        return k

    def leerlauf(self):
        """Nur die erste Viertelbahn senden: Puffer endet bei voller Fahrt."""
        bahn = self.bahn
        section("4. Puffer laeuft leer: Bahn endet bei T/4 mit voller Geschwindigkeit")
        self.an_den_start()
        ende_s = bahn.dauer / 4.0
        rel, punkte = bahn.knoten(0.0, ende_s, self.args.knoten_hz)
        log = self.mitschreiben()
        t_start = host_time()
        res, dauer, fehler = self.trajektorie(rel, punkte, bahn.q0,
                                              blocking=False if self.nonblocking else None)
        self.warten_bis_ruhe(max(0.0, ende_s - dauer) + 1.0)
        t, q = log.stop()
        j = bahn.joint
        letzter = float(punkte[-1][j])
        qj = q[:, j] if len(q) else np.zeros(0)
        richtung = math.copysign(1.0, bahn.amplitude)
        ueber = float(np.max(richtung * (qj - letzter))) if len(qj) else float("nan")
        k = {"proben": int(len(t)), "ueberschwingen_mrad": 1e3 * max(0.0, ueber),
             "endstellung_abweichung_mrad": 1e3 * abs(float(qj[-1]) - letzter) if len(qj) else None}
        if len(t) > 20:
            rel_t = t - t_start
            raster = np.arange(rel_t[0], rel_t[-1], 1.0 / AUSWERT_HZ)
            qg = np.interp(raster, rel_t, qj)
            v = _gleitmittel(np.gradient(qg, 1.0 / AUSWERT_HZ), 5)
            a = np.gradient(v, 1.0 / AUSWERT_HZ)
            nach = raster >= ende_s - 0.05
            k["bremsverzoegerung_max"] = float(np.max(np.abs(a[nach]))) if nach.any() else None
            bewegt = np.flatnonzero(np.abs(v) > 0.05 * bahn.v_max)
            k["stillstand_bei_s"] = float(raster[bewegt[-1]]) if len(bewegt) else None
        self.sichern("leerlauf", t, q, t_start, k, {"fehler": fehler, "puffer_ende_s": ende_s})
        if fehler:
            self.befund("fehler", "Leerlauf-Test: move_trajectory abgelehnt: %s" % fehler)
        else:
            show("Ueberschwingen ueber den letzten Punkt", "%.2f mrad" % k["ueberschwingen_mrad"])
            if k.get("stillstand_bei_s") is not None:
                show("Arm steht ab", "%.2f s (Puffer endete bei %.2f s)"
                     % (k["stillstand_bei_s"], ende_s))
            if k.get("bremsverzoegerung_max") is not None:
                show("groesste Bremsverzoegerung", "%.2f rad/s^2" % k["bremsverzoegerung_max"])
            self.befund("info", "Leerer Puffer: Arm bleibt %.2f mrad hinter/ueber dem letzten "
                        "Punkt stehen, Bremsung bis %.2f rad/s^2 -- so endet eine Fahrt, "
                        "wenn Python ausfaellt." % (
                            k["endstellung_abweichung_mrad"] or 0.0,
                            k.get("bremsverzoegerung_max") or 0.0))
        return k

    def stopp(self):
        """Ganze Bahn senden, bei T/4 Software-Stopp: wird der Puffer geleert?"""
        bahn = self.bahn
        section("5. Stopp mitten in der Fahrt: leert stop() den Puffer?")
        self.an_den_start()
        rel, punkte = bahn.knoten(0.0, bahn.dauer, self.args.knoten_hz)
        log = self.mitschreiben()
        t_start = host_time()
        ergebnis = {}

        def senden():
            ergebnis["res"], ergebnis["dauer"], ergebnis["fehler"] = self.trajektorie(
                rel, punkte, bahn.q0, blocking=False if self.nonblocking else None)

        # In einem Thread, damit der Stopp auch bei blockierendem Befehl kommt.
        sender = threading.Thread(target=senden, daemon=True)
        sender.start()
        stopp_bei = bahn.dauer / 4.0
        time.sleep(max(0.0, t_start + stopp_bei - host_time()))
        t_stopp = host_time()
        self.bot.emergency_stop()
        time.sleep(1.5)
        sender.join(timeout=bahn.dauer + 2.0)
        t, q = log.stop()
        j = bahn.joint
        k = {"proben": int(len(t))}
        if len(t) > 20:
            nach = t >= t_stopp + 0.5
            if nach.any():
                k["bewegung_nach_500ms_mrad"] = 1e3 * float(np.ptp(q[nach, j]))
            rel_t = t - t_stopp
            raster = np.arange(max(rel_t[0], -0.2), rel_t[-1], 1.0 / AUSWERT_HZ)
            qg = np.interp(raster, rel_t, q[:, j])
            v = _gleitmittel(np.gradient(qg, 1.0 / AUSWERT_HZ), 5)
            bewegt = np.flatnonzero((raster > 0) & (np.abs(v) > 0.05 * bahn.v_max))
            k["stillstand_nach_ms"] = 1e3 * float(raster[bewegt[-1]]) if len(bewegt) else 0.0
            k["weg_nach_stopp_mrad"] = 1e3 * float(np.ptp(q[t >= t_stopp, j])) if (t >= t_stopp).any() else None
        self.sichern("stopp", t, q, t_start, k, {
            "fehler": ergebnis.get("fehler"), "stopp_bei_s": stopp_bei})
        try:
            self.bot.clear_stop()
        except Exception as exc:
            self.befund("fehler", "Nach dem Stopp laesst sich nicht neu initialisieren: %s" % exc)
            return k
        if ergebnis.get("fehler") and "stop" not in ergebnis["fehler"].lower():
            self.befund("warnung", "Stopp-Test: Befehl schon vor dem Stopp abgelehnt: %s"
                        % ergebnis["fehler"])
        nachlauf = k.get("bewegung_nach_500ms_mrad")
        if nachlauf is None:
            self.befund("warnung", "Stopp-Test: zu wenige Messwerte nach dem Stopp.")
        elif nachlauf < 1.0 and k.get("stillstand_nach_ms", 1e9) < 500:
            self.befund("ok", "stop() haelt den Arm in %.0f ms an, danach keine Bewegung "
                        "mehr (%.2f mrad) -- der Puffer wird geleert."
                        % (k["stillstand_nach_ms"], nachlauf))
        else:
            self.befund("fehler", "Nach stop() bewegt sich der Arm weiter (%.2f mrad ab "
                        "500 ms, Stillstand nach %.0f ms) -- der Puffer wird NICHT "
                        "geleert. So ist der rollende Puffer nicht sicher."
                        % (nachlauf, k.get("stillstand_nach_ms", float("nan"))))
        return k


def funktionen_pruefen(bot, doku):
    """Punkt 1: welche Pufferbefehle bietet die Steuerung an? Nur lesen."""
    section("1. Pufferfaehige Befehle in der Steuerung (nur lesen)")
    try:
        vorhanden = set(bot._call("get_functions"))
    except Exception as exc:
        print("  [FEHLER] get_functions() nicht abfragbar: %s" % exc)
        return {}
    show("Funktionen insgesamt", len(vorhanden))
    stand = {}
    for name in KANDIDATEN:
        stand[name] = name in vorhanden
        show(name, "vorhanden" if stand[name] else "FEHLT")
    weitere = sorted(n for n in vorhanden
                     if any(w in n for w in ("traj", "buffer", "queue", "spline", "path"))
                     and n not in KANDIDATEN)
    if weitere:
        show("weitere mit traj/buffer/path", ", ".join(weitere))
    stand["_weitere"] = weitere
    for name in (KANDIDATEN if doku else ("move_trajectory",)):
        if not stand.get(name):
            continue
        text = bot._call_safe("get_doc", name)
        if text:
            print("\n  --- get_doc('%s') ---" % name)
            for line in str(text).strip().splitlines():
                print("  | " + line)
            stand["doc_" + name] = str(text)
    if stand.get("move_trajectory"):
        print("  [OK] move_trajectory ist vorhanden -- der rollende Puffer ist pruefbar.")
    else:
        print("  [WARNUNG] move_trajectory fehlt in dieser Steuerung -- der rollende "
              "Puffer geht so nicht. Bleiben movelinear_online (Loesung 1) und DDS (B).")
    return stand


def fazit(lauf, stand):
    section("Fazit: traegt der rollende Puffer?")
    tests = lauf.bericht["tests"]
    ref = tests.get("referenz", {}).get("kennzahlen", {})
    komp = tests.get("komplett", {}).get("kennzahlen", {})
    print("  %-12s %9s %9s %8s %10s %8s %11s %10s %10s" % (
        "Test", "Welligk.", "Stillst.", "Spitze", "Verzug", "RMS mrad", "Leserate",
        "neue Werte", "Luecke"))
    # In Laufreihenfolge -- referenz_<Hz> stehen damit direkt unter referenz.
    gefahren = [n for n in tests if "welligkeit_prozent" in tests[n]["kennzahlen"]]
    for name in gefahren:
        print(zeile(name, tests[name]["kennzahlen"]))
    nicht_deutbar = set()
    for name in gefahren:
        if tests[name].get("bewegt") is False:
            continue   # Arm stand -- da gibt es nichts zu deuten
        gruende = aussagekraft(tests[name]["kennzahlen"],
                               servo_hz=tests[name].get("servo_hz", lauf.args.servo_rate))
        tests[name]["nicht_deutbar"] = gruende
        if gruende:
            nicht_deutbar.add(name)
            lauf.befund("warnung", "%s nicht deutbar: %s." % (name, "; ".join(gruende)))
    if "stillstand_prozent" in ref and "referenz" not in nicht_deutbar:
        if ref["stillstand_prozent"] >= 15.0:
            lauf.befund("info", "Referenz servo_j: Arm steht in %.0f %% der Fahrt kurz still "
                        "-- das Stop-and-go vom 2026-10-05 ist damit gemessen."
                        % ref["stillstand_prozent"])
        else:
            lauf.befund("info", "Referenz servo_j: kaum Stillstand (%.0f %%) -- hier faehrt "
                        "auch servo_j ohne Stop-and-go." % ref["stillstand_prozent"])
    for name in gefahren:
        if not name.startswith("referenz_"):
            continue
        k = tests[name]["kennzahlen"]
        hz = tests[name].get("servo_hz")
        ueber = tests[name].get("taktueberlaeufe", 0)
        if name in nicht_deutbar or "referenz" in nicht_deutbar or "stillstand_prozent" not in ref:
            # Ueberlaeufe sind auch ohne deutbaren Mitschrieb eine Aussage.
            lauf.befund("info", "servo_j %g Hz: %d Taktueberlaeufe; Glaette hier nicht "
                        "vergleichbar." % (hz, ueber))
            continue
        besser = (k["stillstand_prozent"] <= 0.5 * ref["stillstand_prozent"]
                  or k["welligkeit_prozent"] <= 0.5 * ref["welligkeit_prozent"])
        lauf.befund("ok" if besser and ueber == 0 else "info",
                    "servo_j %g Hz statt %g Hz: Stillstand %.0f %% statt %.0f %%, Welligkeit "
                    "%.1f %% statt %.1f %%, %d Taktueberlaeufe%s."
                    % (hz, tests["referenz"].get("servo_hz", lauf.args.servo_rate),
                       k["stillstand_prozent"], ref["stillstand_prozent"],
                       k["welligkeit_prozent"], ref["welligkeit_prozent"], ueber,
                       " -- deutlich ruhiger, Ausweichweg ohne Puffer" if besser else ""))
    if ("welligkeit_prozent" in ref and "welligkeit_prozent" in komp
            and not nicht_deutbar & {"referenz", "komplett"}):
        glatter = (komp["stillstand_prozent"] < 5.0
                   and (ref["stillstand_prozent"] >= 15.0
                        or komp["welligkeit_prozent"] <= 0.5 * ref["welligkeit_prozent"]))
        text = ("(Stillstand %.0f %% statt %.0f %%, Welligkeit %.1f %% statt %.1f %%)"
                % (komp["stillstand_prozent"], ref["stillstand_prozent"],
                   komp["welligkeit_prozent"], ref["welligkeit_prozent"]))
        if glatter:
            lauf.befund("ok", "move_trajectory faehrt deutlich glatter als servo_j " + text + ".")
        else:
            lauf.befund("warnung", "move_trajectory ist nicht deutlich glatter als servo_j "
                        + text + ".")
    on = tests.get("online")
    if on is not None and on.get("bewegt") is False:
        pass   # Befund steht schon aus dem Test (faehrt nicht)
    elif on is not None and "welligkeit_neu_prozent" in on["kennzahlen"]:
        k = on["kennzahlen"]
        vergleich = ""
        if "welligkeit_neu_prozent" in ref:
            vergleich = (" (servo_j: Welligkeit %.1f %%, Einbruch %.0f %%)"
                         % (ref["welligkeit_neu_prozent"], ref["einbruch_neu_prozent"]))
        text = ("movelinear_online, 15-Hz-Ziele: Welligkeit %.1f %%, Tempo-Einbruch %.0f %% "
                "der Fahrzeit, Tempo p5 %.2f der Bahn, Verzug %.0f ms, Folgefehler %.2f mrad%s"
                % (k["welligkeit_neu_prozent"], k["einbruch_neu_prozent"],
                   k["tempo_p5_anteil"], k["verzug_ms"], k["folgefehler_rms_mrad"], vergleich))
        if "online" in nicht_deutbar:
            lauf.befund("warnung", text + " -- nicht deutbar, siehe oben.")
        # Kriterien aus der Synthese (tests/test_check_trajectory_buffer.py):
        # glatt hinter 50-Hz-Quelle Welligkeit ~4 %, Tempo p5 ~0,94; 15-Hz-
        # Puls mit 30 ms Halt 39 % und 0,31. Der Einbruch ist schwaecher.
        elif k["welligkeit_neu_prozent"] < 15.0 and k["tempo_p5_anteil"] > 0.6:
            lauf.befund("ok", text + " -- faehrt durch, kein Pulsieren im Takt (bis ~%.0f Hz "
                        "aufloesbar)." % (0.5 * k.get("neue_werte_hz", 0.0)))
        else:
            lauf.befund("warnung", text + " -- pulsiert im Takt; Vorlauf/Hoechstwerte "
                        "anpassen (--online-vorlauf, --online-tempo).")
    elif on is not None and on.get("fehler"):
        lauf.befund("fehler", "movelinear_online nicht nutzbar: %s" % on["fehler"])
    k15 = tests.get("knoten15", {}).get("kennzahlen", {})
    if "welligkeit_prozent" in k15 and "welligkeit_prozent" in komp:
        if (k15["stillstand_prozent"] < 5.0
                and k15["welligkeit_prozent"] <= 1.5 * komp["welligkeit_prozent"] + 2.0):
            lauf.befund("info", "Auch 15-Hz-Stuetzpunkte werden glatt verbunden -- die "
                        "Steuerung interpoliert selbst.")
        else:
            lauf.befund("info", "15-Hz-Stuetzpunkte fahren unruhiger (%.1f %% gegen %.1f %%) "
                        "-- Punkte vorher fein ausrechnen (%g Hz)."
                        % (k15["welligkeit_prozent"], komp["welligkeit_prozent"],
                           lauf.args.knoten_hz))
    gruende = []
    if not stand.get("move_trajectory"):
        gruende.append("move_trajectory fehlt")
    an = tests.get("anschluss")
    if an is None:
        gruende.append("Anschluss nicht getestet")
    else:
        if not an.get("nicht_blockierend"):
            gruende.append("nur blockierend")
        if any(a["fehler"] for a in an.get("aufrufe", [])):
            gruende.append("Folgeaufruf abgelehnt")
        verh = an["kennzahlen"].get("anschluss_min_verhaeltnis")
        if verh is None or verh < 0.7:
            gruende.append("Tempoeinbruch an den Anschluessen")
    st = tests.get("stopp")
    if st is None:
        gruende.append("Stopp nicht getestet")
    else:
        # Ausdruecklich auf None pruefen: 0.0 mrad ist das beste Ergebnis
        # und darf nicht als "fehlt" gelten (erste Fassung: "0.0 or 99").
        nachlauf = st["kennzahlen"].get("bewegung_nach_500ms_mrad")
        if nachlauf is None or nachlauf >= 1.0:
            gruende.append("stop() leert den Puffer nicht")
    if gruende:
        lauf.befund("warnung", "Rollender Puffer (noch) nicht freigegeben: %s."
                    % ", ".join(gruende))
    else:
        lauf.befund("ok", "Rollender Puffer moeglich: nicht blockierend, nahtloser "
                    "Anschluss, stop() leert den Puffer.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--move", action="store_true",
                    help="Fahrtests ausfuehren (ohne dies wird nur gelesen)")
    ap.add_argument("--tests", default=",".join(TESTS),
                    help="Komma-Liste aus %s (Standard: alle)" % ", ".join(TESTS))
    ap.add_argument("--joint", type=int, default=1, help="Testgelenk 1-6 (Standard 1)")
    ap.add_argument("--amplitude", type=float, default=None,
                    help="Auslenkung in rad (Standard VM 0.15, Anlage 0.05)")
    ap.add_argument("--dauer", type=float, default=4.0,
                    help="Dauer der Hin- und Rueckbewegung in s (Standard 4)")
    ap.add_argument("--knoten-hz", type=float, default=100.0,
                    help="Stuetzpunkte je Sekunde in move_trajectory (Standard 100)")
    ap.add_argument("--chunk-takte", type=int, default=4,
                    help="Abschnittslaenge im Anschlusstest in 15-Hz-Takten (Standard 4)")
    ap.add_argument("--vorlauf-takte", type=int, default=1,
                    help="so viele Takte vor Ende des vorigen Abschnitts senden (Standard 1)")
    ap.add_argument("--servo-rate", type=float, default=config.SERVO_RATE_HZ,
                    help="servo_j-Rate der Referenz (Standard config.SERVO_RATE_HZ)")
    ap.add_argument("--vergleich-raten", default="90,120",
                    help="nach der Referenz dieselbe Fahrt per servo_j mit diesen Raten "
                         "(Komma-Liste, Vielfache von 15; leer = keine). Ausweichweg, "
                         "falls move_trajectory fehlt (Standard 90,120)")
    ap.add_argument("--online-vorlauf", type=int, default=1,
                    help="online: Ziel so viele 15-Hz-Takte voraus (Standard 1)")
    ap.add_argument("--online-tempo", type=float, default=1.3,
                    help="online: Hoechstgeschwindigkeit als Vielfaches der Bahnspitze (Standard 1.3)")
    ap.add_argument("--online-beschl", type=float, default=3.0,
                    help="online: Hoechstbeschleunigung als Vielfaches der Bahnspitze (Standard 3)")
    ap.add_argument("--override", type=float, default=1.0,
                    help="Override fuer die Fahrten (Standard 1.0 -- 0.2 skaliert den Strom)")
    ap.add_argument("--doku", action="store_true",
                    help="get_doc() aller Kandidaten ausgeben, nicht nur move_trajectory")
    ap.add_argument("--real-robot", action="store_true",
                    help="REALE Anlage freigeben (mit Freigabewort)")
    args = ap.parse_args()

    gewaehlt = [n.strip() for n in args.tests.split(",") if n.strip()]
    unbekannt = [n for n in gewaehlt if n not in TESTS]
    if unbekannt:
        raise SystemExit("Unbekannte Tests: %s (moeglich: %s)" % (", ".join(unbekannt), ", ".join(TESTS)))
    if (not 0 <= args.online_vorlauf <= 5 or not 0.5 <= args.online_tempo <= 3.0
            or not 0.5 <= args.online_beschl <= 20.0):
        raise SystemExit("--online-vorlauf 0..5, --online-tempo 0.5..3, --online-beschl 0.5..20")
    if args.chunk_takte < 1 or not 0 <= args.vorlauf_takte < args.chunk_takte:
        raise SystemExit("--vorlauf-takte muss kleiner als --chunk-takte sein")
    if not 1 <= args.joint <= 6:
        raise SystemExit("--joint muss zwischen 1 und 6 liegen")
    try:
        vergleich = [float(r) for r in args.vergleich_raten.split(",") if r.strip()]
        for r in [args.servo_rate] + vergleich:
            substeps_for(r)
    except ValueError as exc:
        raise SystemExit("--vergleich-raten/--servo-rate: %s" % exc)
    vergleich = [r for r in vergleich if r != args.servo_rate]

    section("0. Verbindung")
    bot = NeuraRobot(override=args.override, allow_real=args.real_robot)
    bot.connect(power_on=False, ensure_automatic=False)
    in_sim = bot.in_simulation
    real = in_sim is not True
    show("is_robot_in_simulation()", in_sim)
    show("Server-Version", getattr(bot.robot, "version", None))
    show("Tool", bot.tool_name)

    bericht = {
        "erzeugt": datetime.now().isoformat(timespec="seconds"),
        "geltungsbereich": "Zahlen gelten NUR fuer die hier gemessene Steuerung. In der "
                           "VM Stellvertreter, kein Massstab -- an der Anlage wiederholen.",
        "in_simulation": in_sim,
        "argumente": vars(args),
    }
    stand = funktionen_pruefen(bot, args.doku)
    bericht["funktionen"] = stand

    move = args.move
    if move and real and not args.real_robot:
        print("  [FEHLER] is_robot_in_simulation() ist %r -- Fahrtests verweigert. "
              "An der Anlage nur mit --real-robot." % (in_sim,))
        move = False
    grenze = MAX_AMPLITUDE_REAL if real else MAX_AMPLITUDE_SIM
    amplitude = args.amplitude if args.amplitude is not None else (0.05 if real else 0.15)
    if abs(amplitude) > grenze:
        raise SystemExit("Auslenkung %.3f rad ueber der Grenze %.2f rad" % (amplitude, grenze))
    if move and not stand.get("move_trajectory"):
        ohne = [n for n in gewaehlt if n not in TESTS_PUFFER]
        if len(ohne) < len(gewaehlt):
            print("\n  move_trajectory fehlt -- uebersprungen: %s."
                  % ", ".join(n for n in gewaehlt if n in TESTS_PUFFER))
        gewaehlt = ohne
    if move and "online" in gewaehlt and not stand.get("movelinear_online"):
        print("\n  movelinear_online fehlt -- 'online' uebersprungen.")
        gewaehlt = [n for n in gewaehlt if n != "online"]
    if move and real:
        print("\n!! --real-robot: Fahrtests an der REALEN ANLAGE um die aktuelle Stellung,")
        print("   Gelenk %d, Auslenkung %.3f rad, Tests: %s." % (args.joint, amplitude, ", ".join(gewaehlt)))
        print("   Arbeitsraum frei? Not-Aus in Reichweite?")
        try:
            antwort = input("   Zum Fortfahren 'ANLAGE' eintippen: ").strip()
        except EOFError:
            antwort = ""
        if antwort != "ANLAGE":
            raise SystemExit("Abgebrochen -- nichts bewegt.")

    lauf = None
    if move and gewaehlt:
        bot.connect(power_on=True, ensure_automatic=True)
        q0 = np.asarray(bot.get_joint_angles(), dtype=float)
        bahn = Testbahn(q0, args.joint - 1, amplitude, args.dauer)
        show("Startstellung", np.round(q0, 4).tolist())
        show("Testbahn", "J%d, %.3f rad hin und zurueck in %.1f s, Spitze %.3f rad/s"
             % (args.joint, amplitude, args.dauer, bahn.v_max))
        show("Override", args.override)
        lauf = Lauf(bot, bahn, args)
        try:
            for name in gewaehlt:
                try:
                    if name == "referenz":
                        lauf.referenz()
                        for rate in vergleich:
                            lauf.referenz(rate, "referenz_%g" % rate)
                    elif name == "komplett":
                        lauf.komplett("komplett", args.knoten_hz)
                    elif name == "knoten15":
                        lauf.komplett("knoten15", config.CONTROL_RATE_HZ)
                    elif name == "anschluss":
                        lauf.anschluss()
                    elif name == "leerlauf":
                        lauf.leerlauf()
                    elif name == "stopp":
                        lauf.stopp()
                    elif name == "online":
                        lauf.online()
                except ValueError as exc:
                    # Bahnpruefung hat abgelehnt -- nichts wurde gesendet.
                    lauf.befund("fehler", "%s: Bahn vor dem Senden abgelehnt: %s" % (name, exc))
                if bot.stop_requested:
                    bot.clear_stop()
            lauf.an_den_start()
            fazit(lauf, stand)
        finally:
            bot.close()
        bericht.update(lauf.bericht)
    else:
        if not move:
            print("\n(Fahrtests uebersprungen -- --move)")
        bot.close()

    BERICHTE.mkdir(exist_ok=True)
    stempel = datetime.now().strftime("%Y-%m-%d_%H-%M")
    ziel = BERICHTE / ("%s_Trajektorien-Puffer.json" % stempel)
    ziel.write_text(json.dumps(bericht, indent=2, ensure_ascii=False, default=str),
                    encoding="utf-8")
    print("\nBericht: %s" % ziel)
    if lauf is not None and lauf.roh:
        roh = BERICHTE / ("%s_Trajektorien-Puffer_Mitschrieb.npz" % stempel)
        np.savez_compressed(roh, **lauf.roh)
        print("Mitschrieb: %s" % roh)
    if real:
        print("Hinweis: is_robot_in_simulation() == %r -- ANLAGENMESSUNG." % (in_sim,))
    else:
        print("Hinweis: VIRTUELLE Steuerung -- Zahlen sind kein Massstab fuer die Anlage.")


if __name__ == "__main__":
    main()

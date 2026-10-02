"""Was kostet ``servo_j``, und welche SERVO_RATE_HZ haelt den 15-Hz-Takt?

BEWEGT DEN ARM NICHT -- aber es wird BESTROMT und das Servo-Interface
aktiviert. Jeder gesendete Sollwert ist exakt die gemessene Ist-Stellung
mit v = a = 0 ("hier stehenbleiben"). Trotzdem gilt: Hand an der Freigabe,
Not-Halt in Reichweite. Das Werkzeug fragt vor dem Bestromen nach.

ACHTUNG, GELTUNGSBEREICH: Alle Zahlen gelten fuer die Steuerung, gegen die
gemessen wurde. In der virtuellen Steuerung sind sie ein Stellvertreter,
kein Massstab (siehe Dokumentation/Taktzeit-und-RPC-Latenz.md). An der
Anlage ist dieselbe Messung zu wiederholen, BEVOR Aufnahmeparameter
festgelegt werden.

Warum das die blockierende Zahl ist
-----------------------------------
``bc/recorder.py`` setzt je 15-Hz-Takt ab:

    SERVO_RATE_HZ / CONTROL_RATE_HZ  x  servo_j        (heute 60/15 = 4)
    1 x read_state()                                   (Winkel + FK)

Passt diese Summe nicht in 1/CONTROL_RATE_HZ, kommen die Sollwerte zu
spaet, der Arm bekommt Luecken im Strom, und der Watchdog der Steuerung
kappt die PC-Steuerung (RCSC_102). Aus ``servo_j`` laesst sich nicht
herausrechnen, wie teuer es ist -- es muss gemessen werden.

Was gemessen wird
-----------------
1. Kosten eines einzelnen ``servo_j`` (Haltebefehl), gegen die Kosten von
   ``read_state()`` als Vergleich.
2. Ein TROCKENER TAKT je Kandidatenrate: der vollstaendige Aufrufsatz
   eines Recorder-Takts, nur ohne Bewegung. Daraus die tatsaechlich
   erreichte Regelrate -- die Zahl, die ueber SERVO_RATE_HZ entscheidet.
3. Ob der PC-Steuerkanal ueberhaupt lebt: ``servo_j``-Rueckgabecode und
   ``get_diagnostics()`` vor und nach der Messung. Ein stehender Arm bei
   fehlerfreiem Code ist genau der Zustand, der am 2026-10-01 eine ganze
   Episode mit 218 Takten Datenmuell erzeugt hat.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_servo_timing.py
    python tools/check_servo_timing.py --raten 60,30,15 --takte 40
"""

import argparse
import json
import statistics
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.neura import NeuraRobot
from bc.clock import host_time

BERICHTE = Path(__file__).resolve().parent.parent / "Berichte"

#: Sollwerte, die sicher nichts bewegen: Ist-Stellung, v = 0, a = 0.
#: Genau die Form, die ServoInterpolator.hold() fuer den ersten Takt liefert.
HALTEN = "Ist-Stellung halten (v = a = 0)"


def section(titel):
    print("\n" + "=" * 70)
    print(titel)
    print("=" * 70)


def kennzahlen(werte_s):
    ms = sorted(1e3 * w for w in werte_s)
    p95 = ms[min(len(ms) - 1, int(round(0.95 * (len(ms) - 1))))]
    return {"min": ms[0], "median": statistics.median(ms), "p95": p95,
            "max": ms[-1], "n": len(ms)}


def kopf():
    print("  %-44s %7s %7s %7s %7s" % ("", "min", "med", "p95", "max"))


def zeile(label, k):
    print("  %-44s %7.1f %7.1f %7.1f %7.1f ms" % (
        label, k["min"], k["median"], k["p95"], k["max"]))


def frage(text):
    """Bestaetigung; ohne stdin (Unterprozess) ist die Antwort Nein."""
    try:
        return input(text).strip().lower() in ("j", "ja", "y", "yes")
    except EOFError:
        print("[keine Eingabe moeglich -> nein]")
        return False


def miss_servo(bot, winkel, n):
    """``n`` Haltebefehle, so schnell wie moeglich -- reine Aufrufkosten."""
    null = [0.0] * len(winkel)
    werte = []
    for _ in range(n):
        t0 = host_time()
        bot.servo_j(winkel, null, null)
        werte.append(host_time() - t0)
    return werte


def miss_read_state(bot, n):
    werte = []
    for _ in range(n):
        t0 = host_time()
        bot.read_state()
        werte.append(host_time() - t0)
    return werte


def trockener_takt(bot, winkel, servo_rate_hz, takte):
    """Ein kompletter Recorder-Takt ohne Bewegung, ``takte`` mal.

    Gesendet wird, was der Recorder auch senden wuerde -- nur sind alle
    Zwischenschritte derselbe Haltebefehl. Gemessen wird die Dauer des
    GANZEN Takts, also inklusive read_state().
    """
    substeps = int(round(servo_rate_hz / config.CONTROL_RATE_HZ))
    null = [0.0] * len(winkel)
    dauern = []
    for _ in range(takte):
        t0 = host_time()
        for _ in range(substeps):
            bot.servo_j(winkel, null, null)
        bot.read_state()
        dauern.append(host_time() - t0)
    return substeps, dauern


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raten", default="60,30,15",
                    help="SERVO_RATE_HZ-Kandidaten, kommagetrennt (Standard 60,30,15)")
    ap.add_argument("--takte", type=int, default=30,
                    help="Takte je Kandidatenrate (Standard 30)")
    ap.add_argument("--einzelaufrufe", type=int, default=30,
                    help="Aufrufe fuer die Einzelmessung (Standard 30)")
    ap.add_argument("--real-robot", action="store_true",
                    help="reale Anlage ausdruecklich freigeben (sonst nur Simulation)")
    args = ap.parse_args()

    raten = []
    for teil in args.raten.split(","):
        hz = float(teil)
        verhaeltnis = hz / config.CONTROL_RATE_HZ
        if abs(verhaeltnis - round(verhaeltnis)) > 1e-9 or round(verhaeltnis) < 1:
            raise SystemExit(
                "Rate %g Hz ist kein ganzzahliges Vielfaches von "
                "CONTROL_RATE_HZ (%g)" % (hz, config.CONTROL_RATE_HZ))
        raten.append(hz)

    bericht = {
        "erzeugt": datetime.now().isoformat(timespec="seconds"),
        "geltungsbereich": "Zahlen gelten NUR fuer die hier gemessene Steuerung. "
                           "In der VM Stellvertreter, kein Massstab -- an der "
                           "Anlage neu messen.",
        "control_rate_hz": config.CONTROL_RATE_HZ,
        "servo_rate_hz_konfiguriert": config.SERVO_RATE_HZ,
        "reihen": {},
        "takte": {},
        "befunde": [],
    }

    def befund(stufe, text):
        bericht["befunde"].append([stufe, text])
        print("  [%s] %s" % (stufe.upper(), text))

    section("0. Verbindung")
    bot = NeuraRobot(allow_real=args.real_robot)
    bot.connect(power_on=False, ensure_automatic=False)
    print("  is_robot_in_simulation(): %r" % (bot.in_simulation,))
    print("  Tool: %r   Greifer: %s" % (bot.tool_name, bot.gripper_mode))
    bericht["in_simulation"] = bot.in_simulation
    diag_vorher = bot._call_safe("get_diagnostics")
    print("  Diagnose vorher: %s" % (diag_vorher,))
    bericht["diagnose_vorher"] = diag_vorher

    print()
    print("  Dieses Werkzeug BESTROMT den Roboter und aktiviert das")
    print("  Servo-Interface. Gesendet wird ausschliesslich: %s." % HALTEN)
    print("  Der Arm soll sich dabei NICHT bewegen.")
    if not frage("  Fortfahren? [j/N] "):
        print("  Abgebrochen -- nichts bestromt.")
        return

    section("1. Bestromen und Servo-Interface aktivieren")
    bot.connect(power_on=True, ensure_automatic=True)
    winkel_start = [float(w) for w in bot.get_joint_angles()]
    print("  Ist-Stellung: %s" % [round(w, 4) for w in winkel_start])
    bot.activate_servo("position")
    print("  Servo-Interface aktiv.")

    try:
        section("2. Kosten eines einzelnen Aufrufs")
        kopf()
        k_servo = kennzahlen(miss_servo(bot, winkel_start, args.einzelaufrufe))
        bericht["reihen"]["servo_j (Haltebefehl)"] = k_servo
        zeile("servo_j (Haltebefehl)", k_servo)
        k_state = kennzahlen(miss_read_state(bot, args.einzelaufrufe))
        bericht["reihen"]["read_state() (Winkel + FK)"] = k_state
        zeile("read_state() (Winkel + FK)", k_state)
        print("  letzter servo_j-Code: %r" % (bot.last_servo_code,))

        section("3. Trockener Takt je Kandidatenrate")
        budget_ms = 1e3 / config.CONTROL_RATE_HZ
        print("  Budget je Takt bei %g Hz: %.1f ms" % (
            config.CONTROL_RATE_HZ, budget_ms))
        print()
        print("  %-10s %-9s %9s %9s %9s" % (
            "SERVO_HZ", "Teilschr.", "Takt med", "erreichbar", "Urteil"))
        beste = None
        for hz in raten:
            substeps, dauern = trockener_takt(bot, winkel_start, hz, args.takte)
            k = kennzahlen(dauern)
            erreichbar = 1e3 / k["median"]
            passt = k["median"] <= budget_ms
            bericht["takte"]["%g" % hz] = dict(
                k, substeps=substeps, erreichbare_rate_hz=erreichbar, passt=passt)
            print("  %-10g %-9d %7.1f ms %7.1f Hz   %s" % (
                hz, substeps, k["median"], erreichbar,
                "passt" if passt else "ZU LANGSAM"))
            if passt and beste is None:
                beste = hz
        print()
        print("  (p95 je Rate steht im JSON-Bericht -- der Median sagt, ob es")
        print("   im Mittel passt, der p95, ob es auch im schlechten Fall passt.)")

        if beste is None:
            befund("fehler",
                   "Keine der geprueften Raten haelt %g Hz. Entweder "
                   "CONTROL_RATE_HZ senken oder read_state() verbilligen "
                   "(FK lokal rechnen statt per RPC)." % config.CONTROL_RATE_HZ)
        elif beste != config.SERVO_RATE_HZ:
            befund("warnung",
                   "Konfiguriert ist SERVO_RATE_HZ = %g, gehalten wird erst "
                   "%g Hz. Bis zur Umstellung faehrt der Recorder die Bahn "
                   "langsamer als geplant." % (config.SERVO_RATE_HZ, beste))
        else:
            befund("ok", "Die konfigurierte SERVO_RATE_HZ = %g haelt den Takt."
                   % config.SERVO_RATE_HZ)

        section("4. Hat sich der Arm bewegt? (darf er nicht)")
        winkel_ende = np.asarray(bot.get_joint_angles(), dtype=float)
        drift = float(np.max(np.abs(winkel_ende - np.asarray(winkel_start))))
        bericht["drift_rad"] = drift
        print("  groesste Abweichung zur Startstellung: %.6f rad" % drift)
        if drift > 1e-3:
            befund("warnung",
                   "Der Arm hat sich um %.4f rad bewegt, obwohl nur "
                   "Haltebefehle gesendet wurden." % drift)
        else:
            befund("ok", "Der Arm stand still, wie erwartet.")
    finally:
        bot.deactivate_servo()
        print("\n  Servo-Interface deaktiviert.")

    section("5. Lebt der PC-Steuerkanal noch?")
    diag_nachher = bot._call_safe("get_diagnostics")
    bericht["diagnose_nachher"] = diag_nachher
    bericht["letzter_servo_code"] = bot.last_servo_code
    print("  Diagnose nachher: %s" % (diag_nachher,))
    print("  letzter servo_j-Code: %r" % (bot.last_servo_code,))
    kritisch = isinstance(diag_nachher, dict) and diag_nachher.get("critical")
    if kritisch:
        befund("fehler",
               "Die Steuerung meldet nach der Messung 'critical'. Vor dem "
               "naechsten Lauf reset_control() / reset_errors() -- siehe "
               "Laptop-Inbetriebnahme-Befunde.md 3.3.")
    else:
        befund("ok", "Keine kritische Meldung nach der Messung.")
    bot.close()

    BERICHTE.mkdir(exist_ok=True)
    ziel = BERICHTE / ("%s_Servo-Timing.json" % datetime.now().strftime("%Y-%m-%d_%H-%M"))
    ziel.write_text(json.dumps(bericht, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nBericht: %s" % ziel)
    print("Geltungsbereich: %s" % bericht["geltungsbereich"])


if __name__ == "__main__":
    main()

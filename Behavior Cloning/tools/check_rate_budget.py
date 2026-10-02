"""Welche Raten-Kombinationen passen in das Taktbudget? -- reine Rechnung.

KEIN ROBOTER, KEINE VERBINDUNG. Nimmt die Messwerte aus einem Bericht von
``tools/check_servo_timing.py`` (oder von der Kommandozeile) und rechnet
den Entwurfsraum durch.

Die Ungleichung (siehe Dokumentation/Taktzeit-und-RPC-Latenz.md, 3):

    k * t_servo_j  +  t_read_state  <=  1 / CONTROL_RATE_HZ
    mit k = SERVO_RATE_HZ / CONTROL_RATE_HZ, ganzzahlig >= 1

Was die Rechnung NICHT entscheidet
----------------------------------
Sie sagt, was zeitlich passt. Sie sagt nicht, was fachlich zulaessig ist:

* ``CONTROL_RATE_HZ`` ist in ``Requierments/requierments.md`` auf 15 Hz
  FESTGELEGT (Zeilen 19, 270, 350). Die Rate steckt implizit im Datensatz
  (Schrittweite zwischen zwei Actions = Geschwindigkeit) und muss bei
  Aufzeichnung und Inferenz gleich sein. Jede Abweichung entwertet
  vorhandene Aufnahmen.
* ``SERVO_RATE_HZ`` bestimmt die Laufruhe. Gemessen VM 2026-09-15 (J1 mit
  0,1 rad/s): Streuung der Ist-Geschwindigkeit innerhalb eines Takts
  15 Hz 60-100 %, 60 Hz 33 %, 120 Hz 27 %, 250 Hz 19 %. Massgeblich ist
  die ABSOLUTE Senderate, nicht die Zahl der Teilschritte.

Deshalb gibt dieses Werkzeug beides aus: was passt, und was es kostet.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_rate_budget.py
    python tools/check_rate_budget.py --bericht Berichte/2026-10-01_13-59_Servo-Timing.json
    python tools/check_rate_budget.py --servo-ms 2.5 --read-ms 4.0
"""

import argparse
import glob
import json
import os

import _bootstrap  # noqa: F401

from bc import config

#: Laufruhe nach Senderate, gemessen VM 2026-09-15 (bc/config.py,
#: SERVO_RATE_HZ): Streuung der Ist-Geschwindigkeit innerhalb eines Takts.
#: Zwischenwerte werden linear interpoliert, ausserhalb geklemmt.
LAUFRUHE_HZ = (15.0, 60.0, 120.0, 250.0)
LAUFRUHE_STREUUNG = (0.80, 0.33, 0.27, 0.19)

#: Referenzbahn fuer die Angabe "Takte je Episode": pick_to_station,
#: 218 Takte bei 15 Hz = 14,53 s Fahrzeit.
REFERENZ_SEKUNDEN = 218 / 15.0


def laufruhe(servo_hz):
    """Geschaetzte Geschwindigkeitsstreuung je Takt bei dieser Senderate."""
    if servo_hz <= LAUFRUHE_HZ[0]:
        return LAUFRUHE_STREUUNG[0]
    if servo_hz >= LAUFRUHE_HZ[-1]:
        return LAUFRUHE_STREUUNG[-1]
    for i in range(len(LAUFRUHE_HZ) - 1):
        a, b = LAUFRUHE_HZ[i], LAUFRUHE_HZ[i + 1]
        if a <= servo_hz <= b:
            t = (servo_hz - a) / (b - a)
            return LAUFRUHE_STREUUNG[i] + t * (LAUFRUHE_STREUUNG[i + 1] - LAUFRUHE_STREUUNG[i])
    return LAUFRUHE_STREUUNG[-1]


def letzter_bericht():
    treffer = sorted(glob.glob(os.path.join("Berichte", "*_Servo-Timing.json")))
    return treffer[-1] if treffer else None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bericht", default=None,
                    help="JSON von check_servo_timing.py (Standard: der neueste)")
    ap.add_argument("--servo-ms", type=float, default=None,
                    help="t_servo_j in ms, ueberschreibt den Bericht")
    ap.add_argument("--read-ms", type=float, default=None,
                    help="t_read_state in ms, ueberschreibt den Bericht")
    ap.add_argument("--raten", default="15,12.5,10,7.5,6,5",
                    help="CONTROL_RATE_HZ-Kandidaten (Standard 15,12.5,10,7.5,6,5)")
    ap.add_argument("--p95", action="store_true",
                    help="gegen den p95 rechnen statt gegen den Median "
                         "(strenger: haelt der Takt auch im schlechten Fall?)")
    args = ap.parse_args()

    kennzahl = "p95" if args.p95 else "median"
    quelle = "Kommandozeile"
    t_servo = args.servo_ms
    t_read = args.read_ms
    fk_lokal = config.NEURA_LOCAL_FK

    if t_servo is None or t_read is None:
        pfad = args.bericht or letzter_bericht()
        if pfad is None:
            raise SystemExit(
                "Kein Servo-Timing-Bericht gefunden. Erst "
                "tools/check_servo_timing.py laufen lassen, oder "
                "--servo-ms/--read-ms angeben.")
        with open(pfad, encoding="utf-8") as fh:
            b = json.load(fh)
        quelle = pfad
        reihen = b["reihen"]
        if t_servo is None:
            t_servo = reihen["servo_j (Haltebefehl)"][kennzahl]
        if t_read is None:
            t_read = reihen["read_state() (Winkel + FK)"][kennzahl]

    print("Quelle der Messwerte : %s  (%s)" % (quelle, kennzahl))
    print("t_servo_j            : %6.2f ms" % t_servo)
    print("t_read_state         : %6.2f ms" % t_read)
    print("lokale FK aktiv      : %s" % fk_lokal)
    print()
    print("GELTUNGSBEREICH: Die Messwerte gelten fuer die Steuerung, an der")
    print("gemessen wurde. An der Anlage neu messen, dann neu rechnen.")
    print()
    print("Soll laut Requierments: CONTROL_RATE_HZ = 15 (festgelegt),")
    print("                        SERVO_RATE_HZ   = %g (Auslegung)" % config.SERVO_RATE_HZ)
    print()

    kopf = ("%-9s %-3s %-9s %9s %9s %10s %9s %8s"
            % ("CTRL_HZ", "k", "SERVO_HZ", "Takt soll", "Takt ist",
               "erreicht", "Laufruhe", "Takte/Ep"))
    print(kopf)
    print("-" * len(kopf))

    passend = []
    for teil in args.raten.split(","):
        f = float(teil)
        budget = 1000.0 / f
        for k in range(1, 9):
            kosten = k * t_servo + t_read
            if kosten > budget and k > 1:
                break  # groessere k werden nur schlechter
            servo_hz = k * f
            passt = kosten <= budget
            takte = REFERENZ_SEKUNDEN * f
            print("%-9g %-3d %-9g %7.1f ms %7.1f ms %8.1f Hz %8.0f %% %8.0f  %s"
                  % (f, k, servo_hz, budget, kosten, min(f, 1000.0 / kosten),
                     100 * laufruhe(servo_hz), takte,
                     "" if passt else "ZU LANGSAM"))
            if passt:
                passend.append((f, k, servo_hz, kosten, budget, takte))
        print()

    print("=" * len(kopf))
    if not passend:
        print("KEINE Kombination passt. Die Aufrufkosten selbst muessen runter")
        print("(lokale FK, schnellere Steuerung) -- an den Raten ist nichts zu holen.")
        return

    print("Was passt, nach Laufruhe sortiert (beste zuerst):")
    print()
    for f, k, servo_hz, kosten, budget, takte in sorted(
            passend, key=lambda r: -r[2]):
        reserve = 100.0 * (budget - kosten) / budget
        einbusse_takte = 100.0 * (f - config.CONTROL_RATE_HZ) / config.CONTROL_RATE_HZ
        hinweise = []
        if f != config.CONTROL_RATE_HZ:
            hinweise.append(
                "Schema-Bruch: %+.0f %% Regelrate, %+.0f %% Takte je Episode, "
                "Reaktionszeit %.0f statt %.0f ms"
                % (einbusse_takte, einbusse_takte, 1000.0 / f,
                   1000.0 / config.CONTROL_RATE_HZ))
        if k == 1:
            hinweise.append("KEINE Interpolation mehr (bc/servo.py wirkungslos)")
        elif servo_hz < config.SERVO_RATE_HZ:
            hinweise.append("Laufruhe schlechter als heute (%g Hz)" % config.SERVO_RATE_HZ)
        print("  CONTROL %g Hz, SERVO %g Hz (k=%d) -- %.0f %% Reserve im Takt"
              % (f, servo_hz, k, reserve))
        for h in hinweise or ["ohne Abstriche"]:
            print("      %s" % h)
        print()


if __name__ == "__main__":
    main()

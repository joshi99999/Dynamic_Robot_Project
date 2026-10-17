"""Handy-Messung (phyphox "Beschleunigung mit g", xlsx) auswerten -- NUR LESEND.

Fuer beliebige Serien am Labortag (Labortag-Messplan-Zittern.md): findet
die Fahrten automatisch (Lageaenderung des Handys) und gibt je Fahrt
Uhrzeit, Zittern (RMS/p99 ueber 5 Hz) und Energieanteile in den Baendern
aus, um die es geht: 6-8 Hz (Eigenschwingung Aufbau), 9-11 Hz (servo_j
gegen den 50-Hz-Takt der Steuerung), 14-17 Hz (15-Hz-Zieltakt). Die
Uhrzeit vergleicht man mit den ``LOS``-Zeilen der Messwerkzeuge; auch die
Rueckfahrt an den Start erscheint als Fahrt.

Die Erkennung und Filterung ist dieselbe wie in tools/report_zittern.py
(Abschnitt 4.6 von Zittern-an-der-Anlage.md). Nur Laeufe derselben
Befestigung miteinander vergleichen. Nach PAUSE/START in phyphox wird jeder
Abschnitt fuer sich ausgewertet und ueber seinen START auf die Uhrzeit
abgebildet (Labortag 2026-10-16: vorher waren die Uhrzeiten nach einer
Pause falsch).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/report_handy.py Berichte/<datei>.xlsx
    python tools/report_handy.py Berichte/<datei>.xlsx --namen "linear,pos,online"
    python tools/report_handy.py <datei>.xlsx --fenster "linear@12:44:36+14.2,online@12:45:25+16"

``--fenster``: Name@Uhrzeit+Dauer(s) je Lauf, z. B. LOS-Zeit und Fahrzeit aus
dem Mitschrieb -- statt der automatischen Erkennung (sie haengt an der
Lageaenderung des Handys und uebersieht ruhige Fahrten wie ``online``).
"""

import argparse
from datetime import datetime

import _bootstrap  # noqa: F401

import numpy as np

import report_zittern as rz

BAENDER = ((5.0, 6.0), (6.0, 8.5), (8.5, 11.0), (11.0, 14.0), (14.0, 17.0), (17.0, 100.0))


def uhr(s):
    return "-" if s is None else datetime.fromtimestamp(s).strftime("%H:%M:%S")


def baender(fa):
    f, s = fa["f"], fa["spektrum"]
    m = f >= 5.0
    gesamt = float(np.sum(s[m] ** 2))
    out = []
    for lo, hi in BAENDER:
        b = (f >= lo) & (f < hi)
        out.append(float(np.sum(s[b] ** 2)) / gesamt if gesamt > 0 else float("nan"))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xlsx")
    ap.add_argument("--namen", default="", help="Komma-Liste: Namen der Fahrten in Reihenfolge (optional)")
    ap.add_argument("--fenster", default="",
                    help="Komma-Liste name@HH:MM:SS+dauer_s -- Laeufe ueber Uhrzeit-Fenster auswerten")
    args = ap.parse_args()
    h = rz.handy_auswerten(args.xlsx)
    if args.fenster:
        fahrten, namen = [], []
        tag = datetime.fromtimestamp(next(s for _, _, s in h["abschnitte"] if s is not None)).date()
        for teil in args.fenster.split(","):
            name, rest = teil.strip().split("@")
            zeit, dauer = rest.split("+")
            von = datetime.combine(tag, datetime.strptime(zeit, "%H:%M:%S").time()).timestamp()
            fa = rz.fenster_auswerten(h, von, von + float(dauer))
            if fa is None:
                print("%s: Fenster %s+%s s liegt nicht (ganz) in der Messung" % (name, zeit, dauer))
                continue
            fa["t1"] = fa["t0"] + float(dauer)
            fahrten.append(fa)
            namen.append(name)
        h["fahrten"] = fahrten
        args.namen = ",".join(namen)
    namen = [n.strip() for n in args.namen.split(",") if n.strip()]
    print("Rauschboden (Stillstand): %.3f m/s^2, Abtastrate %d Hz, %d Fahrten erkannt"
          % (h["boden_rms"], h["fs"], len(h["fahrten"])))
    for k, (e0, e1, s0) in enumerate(h["abschnitte"]):
        print("Abschnitt %d: Experimentzeit %7.1f-%7.1f s, Uhrzeit %s-%s" % (
            k, e0, e1, uhr(s0), uhr(None if s0 is None else s0 + e1 - e0)))
    kopf = "  ".join("%g-%g" % b if b[1] < 100 else ">%g" % b[0] for b in BAENDER)
    print("%-3s %-3s %-10s %-14s %6s %6s %6s %7s   Anteile [Hz]: %s"
          % ("Nr", "Ab", "Beginn", "Name", "Dauer", "RMS", "p99", "Spitze", kopf))
    for i, fa in enumerate(h["fahrten"]):
        name = namen[i] if i < len(namen) else ""
        print("%-3d %-3d %-10s %-14s %5.1fs %6.2f %6.2f %5.1fHz   %s" % (
            i + 1, fa["abschnitt"], fa["beginn"], name[:14], fa["t1"] - fa["t0"], fa["rms"], fa["p99"],
            fa["spitze_hz"], "  ".join("%5.2f" % a for a in baender(fa))))
    print("(RMS/p99 = Betrag der Beschleunigung ueber 5 Hz in m/s^2; Anteile = Energie je Band)")


if __name__ == "__main__":
    main()

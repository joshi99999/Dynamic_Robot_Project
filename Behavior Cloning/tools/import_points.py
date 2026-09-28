"""Gesicherte Punkte in die Punktdatenbank einer Steuerung zurueckspielen (AP 2.2).

Gegenstueck zu ``tools/export_points.py``. SCHREIBT in die Steuerung --
deshalb passiert ohne ``--write`` nichts, der Lauf zeigt dann nur, was er
tun wuerde. Der Roboter bewegt sich dabei nicht.

Die Aufrufform wurde am 2026-09-24 gegen die VM (``v5.0.0-alpha.102``)
bestimmt; NeuraPy reicht Aufrufe nur durch und kennt selbst keine
Signaturen:

    create_point(name, reference_frame, kartesische_pose)

* ``reference_frame`` ist einer von **World, Base, Tool** -- andere Namen
  werden mit "No reference frame found" abgelehnt.
* Position 3 ist eine **kartesische** Pose ``[X, Y, Z, RX, RY, RZ]``
  (Meter/Radiant, RPY wie in der Datenbank). Gelenkwinkel an dieser Stelle
  scheitern mit "Exception: Inverse Kinemat..." -- die Steuerung rechnet
  die Gelenkstellung selbst per IK aus.

**Daraus folgt der wichtigste Vorbehalt:** Die Steuerung waehlt den
IK-Loesungszweig neu. Die Gelenkstellung ist aber das, was den Zweig
festlegt und spaeter als IK-Seed dient (siehe Docstring von
``NeuraRobot.get_point``). Dieses Werkzeug liest deshalb jeden angelegten
Punkt zurueck und vergleicht die Gelenkwinkel mit der Sicherung. Weichen
sie ab, wird der Punkt als **ABWEICHEND** gemeldet -- er steht dann zwar
geometrisch richtig, faehrt aber moeglicherweise in einer anderen
Armkonfiguration an. In der Gegenprobe mit ``Home`` stimmten sie exakt
(0.000000 rad).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/import_points.py sequences/points_2026-09-24.json
    python tools/import_points.py punkte.json --write
    python tools/import_points.py punkte.json --write --overwrite
    python tools/import_points.py punkte.json --write --only PICK PRE_GRASP
"""

import argparse
import json
from pathlib import Path

import _bootstrap  # noqa: F401

from bc import config


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def show(label, value):
    print("  %-38s %s" % (label, value))


#: Von der Steuerung akzeptierte Referenzframes (2026-09-24 ermittelt).
FRAMES = ("World", "Base", "Tool")


def connect_raw():
    """Rohe ``Robot``-Instanz -- ohne ``init_program()``.

    Punkte anzulegen braucht weder Automatikmodus noch Bestromung; der
    Umweg ueber ``NeuraRobot.connect()`` wuerde beides voraussetzen.
    """
    from neurapy.robot import Robot, SOCKET_ADDRESS, SOCKET_PORT

    raw = Robot()
    return raw, "%s:%s" % (SOCKET_ADDRESS, SOCKET_PORT)


def joint_abweichung(soll, ist):
    if not soll or not ist or len(soll) != len(ist):
        return None
    return max(abs(float(a) - float(b)) for a, b in zip(soll, ist))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("datei", help="Sicherung aus tools/export_points.py")
    parser.add_argument("--write", action="store_true",
                        help="tatsaechlich schreiben (ohne: nur anzeigen)")
    parser.add_argument("--overwrite", action="store_true",
                        help="vorhandene Punkte gleichen Namens ersetzen")
    parser.add_argument("--only", nargs="*", default=None, help="nur diese Punkte")
    parser.add_argument("--frame", default="World", choices=FRAMES,
                        help="Referenzframe der kartesischen Posen (Default World)")
    parser.add_argument("--real-robot", action="store_true",
                        help="auch schreiben, wenn es NICHT die Simulation ist")
    args = parser.parse_args()

    daten = json.loads(Path(args.datei).read_text(encoding="utf-8"))
    punkte = daten.get("punkte", {})
    quelle = daten.get("herkunft", {})

    section("0. Sicherung und Ziel")
    show("Datei", args.datei)
    show("Punkte in der Sicherung", len(punkte))
    show("aufgenommen von", quelle.get("socket"))
    show("  Server-Version", quelle.get("server_version"))
    show("  in_simulation", quelle.get("in_simulation"))
    show("  Tool", quelle.get("tool"))

    raw, socket = connect_raw()
    ziel_sim = None
    try:
        ziel_sim = raw.is_robot_in_simulation()
    except Exception as exc:
        print("  [WARN] is_robot_in_simulation() nicht abfragbar: %s" % exc)
    show("Ziel", socket)
    show("  in_simulation", ziel_sim)
    show("  Tool", getattr(raw, "get_selected_tool_name", lambda: None)())
    show("Referenzframe", args.frame)

    if quelle.get("tool") != (raw.get_selected_tool_name() if hasattr(raw, "get_selected_tool_name") else None):
        print("\n  [WARN] Das aktive Tool weicht von dem der Sicherung ab. Punkte, die")
        print("         mit einem anderen Tool geteacht wurden, stehen woanders.")

    if ziel_sim is not True and not args.real_robot:
        print("\n  [FAIL] Ziel meldet is_robot_in_simulation() != True. VM und reale")
        print("         Control-Box teilen sich dieselbe Adresse -- ohne --real-robot")
        print("         wird nichts geschrieben.")
        return 1

    vorhanden = set(str(n) for n in raw.get_point_names())
    gewuenscht = [n for n in (args.only if args.only else punkte.keys()) if n in punkte]
    fehlt_in_datei = [n for n in (args.only or []) if n not in punkte]

    section("1. Abgleich")
    for name in fehlt_in_datei:
        show(name, "NICHT IN DER SICHERUNG")
    plan = []
    for name in gewuenscht:
        cart = punkte[name].get("cartesian")
        if not cart:
            show(name, "uebersprungen -- keine kartesische Pose gesichert")
            continue
        if name in vorhanden and not args.overwrite:
            show(name, "existiert bereits -- uebersprungen (--overwrite erzwingt)")
            continue
        plan.append(name)
        show(name, "%s%s" % ("ERSETZEN" if name in vorhanden else "ANLEGEN",
                             "" if args.write else "  (Probelauf)"))

    if not args.write:
        section("2. Probelauf -- nichts geschrieben")
        show("wuerde schreiben", "%d Punkte" % len(plan))
        print("\n  Zum Schreiben denselben Aufruf mit --write wiederholen.")
        return 0

    section("2. Schreiben")
    angelegt, abweichend, fehler = [], [], []
    for name in plan:
        cart = [float(v) for v in punkte[name]["cartesian"]]
        if name in vorhanden:
            try:
                raw.delete_pose_in_DB(name)
            except Exception as exc:
                fehler.append(name)
                show(name, "FEHLER beim Loeschen: %s" % exc)
                continue
        try:
            raw.create_point(name, args.frame, cart)
        except Exception as exc:
            fehler.append(name)
            show(name, "FEHLER: %s" % exc)
            continue

        # Rueckprobe: hat die Steuerung denselben IK-Zweig gewaehlt?
        try:
            ist = [float(v) for v in raw.get_point(name, representation="Joint")]
        except Exception as exc:
            fehler.append(name)
            show(name, "angelegt, aber nicht rueckleshar: %s" % exc)
            continue
        soll = punkte[name].get("joint")
        delta = joint_abweichung(soll, ist)
        if delta is None:
            angelegt.append(name)
            show(name, "angelegt (keine Gelenkwerte in der Sicherung -- nicht geprueft)")
        elif delta <= config.START_POSE_TOL_RAD:
            angelegt.append(name)
            show(name, "angelegt, Gelenke stimmen (max %.6f rad)" % delta)
        else:
            abweichend.append(name)
            show(name, "ABWEICHEND: Gelenke weichen um %.4f rad ab" % delta)

    section("3. Ergebnis")
    show("angelegt", "%d" % len(angelegt))
    show("abweichender IK-Zweig", "%d (%s)" % (len(abweichend), ", ".join(abweichend) or "-"))
    show("Fehler", "%d (%s)" % (len(fehler), ", ".join(fehler) or "-"))
    print("\n  Abnahme:")
    print("    [%s] alle angeforderten Punkte angelegt" % ("OK" if not fehler else "  "))
    print("    [%s] IK-Loesungszweig ueberall erhalten" % ("OK" if not abweichend else "  "))
    if abweichend:
        print("\n  Abweichende Punkte vor der Aufzeichnung pruefen: die Steuerung")
        print("  faehrt sie geometrisch richtig an, moeglicherweise aber in einer")
        print("  anderen Armkonfiguration als beim Teachen.")
    return 1 if (fehler or abweichend) else 0


if __name__ == "__main__":
    raise SystemExit(main())

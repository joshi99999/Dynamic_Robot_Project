"""Geteachte Punkte aus der Steuerung in eine Datei sichern (AP 2.2).

NUR LESEN -- der Roboter bewegt sich nicht, es wird nichts bestromt und
nichts in der Steuerung geaendert (Vorbild tools/check_ik.py).

Warum es das gibt: ``sequences/*.json`` nennt nur NAMEN; die Koordinaten
holt ``bc/sequence.py`` zur Laufzeit aus der Punktdatenbank der Steuerung
(``bc/adapters/neura.py:get_point``). Die Punkte leben damit ausschliesslich
in der Control-Box bzw. in der Festplatte der VM -- nicht im Repo. Wird die
VM neu aus dem OVA importiert oder die Steuerung getauscht, sind sie weg,
und aufgezeichnete Episoden lassen sich nicht mehr zuordnen. Genau das ist
am 2026-09-24 beim Wechsel auf den Laptop passiert (siehe
``Dokumentation/Laptop-Inbetriebnahme-Befunde.md``).

Die Datei ist daher zweierlei:
  * Sicherung, die sich mit ``tools/import_points.py`` zurueckspielen laesst,
  * Beleg, gegen welche Punkte ein Datensatz aufgezeichnet wurde.

Gesichert werden BEIDE Darstellungen der Datenbank:
  * ``Joint`` -- massgeblich, legt den IK-Loesungszweig fest (siehe Docstring
    von ``NeuraRobot.get_point``),
  * ``Cartesian`` -- die Gegenprobe. Weichen beide beim Ruecklesen
    voneinander ab, wurde mit einem anderen Tool/Frame geteacht.

Mitgeschrieben wird ausserdem, WOHER die Werte stammen (Socket,
Server-Version, ``is_robot_in_simulation()``, aktives Tool). Ohne das
laesst sich spaeter nicht mehr sagen, ob eine Sicherung aus der VM oder von
der Anlage kommt -- die beiden teilen sich dieselbe Adresse (AP 0.6).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/export_points.py                       # alle Punkte
    python tools/export_points.py --sequence sequences/pick_to_station.json
    python tools/export_points.py --points PICK PRE_GRASP
    python tools/export_points.py --out sequences/points_anlage.json
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def show(label, value):
    print("  %-38s %s" % (label, value))


def connect_readonly():
    """Rohe ``Robot``-Instanz plus Herkunftsangaben.

    Bewusst NICHT ueber ``NeuraRobot.connect()``: das ruft ``init_program()``
    auf und setzt damit den Automatikmodus voraus, obwohl hier nur gelesen
    wird (Befund 2026-09-24).
    """
    from neurapy.robot import Robot, SOCKET_ADDRESS, SOCKET_PORT

    raw = Robot()
    herkunft = {
        "socket": "%s:%s" % (SOCKET_ADDRESS, SOCKET_PORT),
        "server_version": getattr(raw, "version", None),
        "exportiert_am": datetime.now().isoformat(timespec="seconds"),
    }
    try:
        from neurapy.robot import VERSION as client_version
    except ImportError:
        client_version = None
    herkunft["client_version"] = client_version

    for label, fn in (("in_simulation", "is_robot_in_simulation"),
                      ("tool", "get_selected_tool_name")):
        try:
            herkunft[label] = getattr(raw, fn)()
        except Exception as exc:
            herkunft[label] = None
            print("  [WARN] %s() nicht abfragbar: %s" % (fn, exc))
    return raw, herkunft


def names_from_sequence(path):
    """Punktnamen aus einer Ablaufdatei -- Reihenfolge erhalten, ohne Dubletten."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    out = []
    for step in data.get("sequence", []):
        name = step.get("point")
        if name and name not in out:
            out.append(name)
    return out


def read_point(raw, name):
    """Beide Darstellungen eines Punktes lesen. Fehlt eine, bleibt sie ``None``."""
    entry = {}
    for key, representation in (("joint", "Joint"), ("cartesian", "Cartesian")):
        try:
            entry[key] = [float(v) for v in raw.get_point(name, representation=representation)]
        except Exception as exc:
            entry[key] = None
            entry.setdefault("fehler", {})[key] = str(exc)
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None, help="Zieldatei (JSON)")
    parser.add_argument("--points", nargs="*", default=None, help="nur diese Punkte")
    parser.add_argument("--sequence", default=None, help="Namen aus einer Ablaufdatei")
    args = parser.parse_args()

    section("0. Verbindung und Herkunft")
    raw, herkunft = connect_readonly()
    for key in ("socket", "server_version", "client_version", "in_simulation", "tool"):
        show(key, herkunft.get(key))
    if herkunft.get("in_simulation") is not True:
        print("\n  [WARN] is_robot_in_simulation() ist nicht True -- diese Sicherung")
        print("         stammt moeglicherweise von der REALEN Anlage. Dateinamen")
        print("         entsprechend waehlen, damit sie nicht mit VM-Punkten mischt.")

    section("1. Punkte in der Steuerung")
    vorhanden = [str(n) for n in raw.get_point_names()]
    show("Punkte insgesamt", len(vorhanden))
    for name in sorted(vorhanden):
        print("      %s" % name)

    if args.sequence:
        gewuenscht = names_from_sequence(args.sequence)
    elif args.points:
        gewuenscht = list(args.points)
    else:
        gewuenscht = sorted(vorhanden)

    fehlend = [n for n in gewuenscht if n not in set(vorhanden)]
    gewuenscht = [n for n in gewuenscht if n in set(vorhanden)]

    section("2. Auslesen")
    punkte = {}
    for name in gewuenscht:
        punkte[name] = read_point(raw, name)
        joint = punkte[name].get("joint")
        show(name, "J=%s" % ([round(v, 6) for v in joint] if joint else "FEHLER"))
    for name in fehlend:
        show(name, "NICHT IN DER STEUERUNG")

    out = Path(args.out) if args.out else Path("sequences") / (
        "points_%s.json" % datetime.now().strftime("%Y-%m-%d_%H-%M"))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps({"herkunft": herkunft, "punkte": punkte}, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    section("3. Ergebnis")
    show("gesichert", "%d Punkte" % len(punkte))
    show("nicht gefunden", "%d (%s)" % (len(fehlend), ", ".join(fehlend) or "-"))
    show("Datei", out)
    print("\n  Zurueckspielen auf eine andere Steuerung:")
    print("      python tools/import_points.py %s" % out)
    if fehlend:
        print("\n  [FAIL] %d angeforderte Punkte fehlen in der Steuerung." % len(fehlend))
    return 1 if fehlend else 0


if __name__ == "__main__":
    raise SystemExit(main())

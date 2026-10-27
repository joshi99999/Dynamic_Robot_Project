"""Was loesen die Knoepfe FREE und POINT am Arm aus? (Teach-Serie, AP 2.2)

NUR LESEN -- der Roboter bewegt sich nicht, es wird nichts bestromt und
nichts in der Steuerung geaendert (Vorbild tools/check_ik.py). Die Punkte,
die POINT anlegt, bleiben stehen; aufraeumen am Pendant.

Hintergrund: Ab Werk stehen beide Knoepfe auf "noAction" (Sicherung
``hmis.json``: ``freeButton``/``pointButton``) -> Meldung "no option is
selected for this". Die Teach-Serie der Oberflaeche (bc/teach_series.py)
setzt voraus:

* FREE  = Zero-G am Pendant. Frage: wirkt es nur beim Halten oder schaltet
  es um? Sichtbar an ``is_free_drive_mode_enabled``/Teach-Modus.
* POINT = "Save Point" am Pendant. Frage: legt das einen Punkt an, den
  ``get_point_names`` sieht -- und wie heisst er? Den Namen (bzw. ein
  Muster) in ``config.TEACH_SERIES_POINT_PATTERN`` eintragen.
* Zusaetzlich: Kommt ein Knopfdruck als Signal in ``get_io_status`` an?
  Dann koennte POINT auf "noAction" bleiben und die Oberflaeche direkt
  darauf reagieren.

Vorgehen an der Anlage: Knoepfe am Pendant belegen, Skript starten, dann
nacheinander FREE kurz, FREE halten, POINT einmal, POINT zweimal schnell
druecken. Das Skript schreibt jede Aenderung mit Uhrzeit mit.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/probe_arm_buttons.py                    # bis Strg+C
    python tools/probe_arm_buttons.py --seconds 120 --log Berichte/knoepfe_anlage.txt
"""

import argparse
import json
import time
from datetime import datetime

import _bootstrap  # noqa: F401

#: Abfragen je Durchlauf: (Anzeigename, Funktion). Was der Server nicht
#: anbietet, wird einmal gemeldet und dann ausgelassen.
STATE_CALLS = (
    ("teach_mode", "is_robot_in_teach_mode"),
    ("free_drive", "is_free_drive_mode_enabled"),
    ("io_status", "get_io_status"),
)

#: Stichworte, nach denen die Funktionsliste des Servers durchsucht wird.
KEYWORDS = ("button", "hmi", "zero", "free", "drive", "io", "teach", "point", "jog")


class Output(object):
    def __init__(self, path):
        self.file = open(path, "a", encoding="utf-8") if path else None

    def __call__(self, text):
        line = "%s  %s" % (datetime.now().strftime("%H:%M:%S.%f")[:-3], text)
        print(line)
        if self.file:
            self.file.write(line + "\n")
            self.file.flush()

    def close(self):
        if self.file:
            self.file.close()


def connect_readonly():
    """Rohe ``Robot``-Instanz -- bewusst NICHT ueber ``NeuraRobot.connect()``,
    das ``init_program()`` aufruft und den Automatikmodus voraussetzt
    (Befund 2026-09-24, wie tools/export_points.py)."""
    from neurapy.robot import Robot, SOCKET_ADDRESS, SOCKET_PORT

    raw = Robot()
    return raw, "%s:%s" % (SOCKET_ADDRESS, SOCKET_PORT)


def compact(value):
    """Wert einzeilig fuer das Protokoll."""
    try:
        return json.dumps(value, ensure_ascii=False, default=str, sort_keys=True)
    except Exception:
        return repr(value)


def point_info(raw, name):
    """Datensatz eines neuen Punkts: Typ, Zeitstempel, Gelenke."""
    try:
        record = raw.get_pose_in_reference_frame_from_DB(raw.get_pose_id_from_name(name))
        record = dict(record or {})
        joints = record.get("jointAngles")
        return "type=%s createdAt=%s joints=%s Felder=%s" % (
            record.get("type"), record.get("createdAt"),
            None if joints is None else "[%s]" % ", ".join("%.4f" % float(v) for v in joints),
            sorted(record))
    except Exception as exc:
        return "Datensatz nicht lesbar: %s" % exc


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=None, help="Laufzeit (Standard: bis Strg+C)")
    parser.add_argument("--interval", type=float, default=0.2, help="Abfrageabstand in s")
    parser.add_argument("--log", default=None, help="zusaetzlich in diese Datei schreiben")
    args = parser.parse_args()

    out = Output(args.log)
    raw, socket = connect_readonly()
    out("Verbunden mit %s, Server %s" % (socket, getattr(raw, "version", None)))
    for label, fn in (("is_robot_in_simulation", "is_robot_in_simulation"),
                      ("Werkzeug", "get_selected_tool_name")):
        try:
            out("%s: %r" % (label, getattr(raw, fn)()))
        except Exception as exc:
            out("%s nicht abfragbar: %s" % (label, exc))

    try:
        functions = [str(f) for f in raw.get_functions()]
        hits = sorted(f for f in functions if any(k in f.lower() for k in KEYWORDS))
        out("Server-Funktionen mit %s:" % "/".join(KEYWORDS))
        for name in hits:
            out("    %s" % name)
    except Exception as exc:
        functions = None
        out("Funktionsliste nicht lesbar: %s" % exc)

    calls = []
    for label, fn in STATE_CALLS:
        if functions is not None and fn not in functions:
            out("%s: vom Server nicht angeboten -- ausgelassen" % fn)
            continue
        calls.append((label, fn))

    known = [str(n) for n in raw.get_point_names()]
    out("%d Punkte vorhanden. Jetzt die Knöpfe drücken (Strg+C beendet)." % len(known))

    last = {}
    end = None if args.seconds is None else time.monotonic() + args.seconds
    try:
        while end is None or time.monotonic() < end:
            for label, fn in list(calls):
                try:
                    value = compact(getattr(raw, fn)())
                except Exception as exc:
                    value = "FEHLER %s" % exc
                if last.get(label) != value:
                    out("%-10s %s" % (label, value))
                    last[label] = value
            names = [str(n) for n in raw.get_point_names()]
            for name in names:
                if name not in known:
                    out("NEUER PUNKT %r: %s" % (name, point_info(raw, name)))
            for name in known:
                if name not in names:
                    out("Punkt entfernt: %r" % name)
            known = names
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    out("Ende.")
    out.close()


if __name__ == "__main__":
    main()

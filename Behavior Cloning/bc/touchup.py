"""Touch-up eines geteachten Punkts aus der Bedienoberflaeche (AP 2.2) -- Qt-frei.

Wunsch Anwender 2026-10-21: In der Ablauftabelle neben jedem Punkt per
Klick die aktuelle Stellung des Arms (samt Werkzeug) uebernehmen -- statt
am Pendant den Punkt zu suchen und dort nachzuteachen. In der Reihenfolge
der Tabelle angeklickt, legt das die Punkte eines neuen Ablaufs auch an.

Ablauf in drei Schritten, damit die Oberflaeche dazwischen fragen kann:

1. :func:`plan_touch_up` liest, prueft und rechnet aus, was sich aendern
   wuerde -- schreibt nichts.
2. :func:`apply_touch_up` sichert den alten Datensatz als Datei und schreibt
   dann (``PointStorePort.store_point``: Gelenkwinkel, gleiche ID).
3. :func:`undo_touch_up` spielt eine Sicherung zurueck.

Geprueft wird vor dem Schreiben:

* **Stillstand** -- zwei Lesungen im Abstand ``TOUCHUP_STILL_WAIT_S``.
  Sonst landet eine Zwischenstellung in der Datenbank.
* **Werkzeug** -- ein Punkt speichert kein Tool, die Steuerung rechnet die
  Pose mit dem aktiven. An der realen Anlage muss deshalb
  ``config.TOUCHUP_TOOL_NAME`` gewaehlt sein. In der Simulation nur ein
  Hinweis (die VM kennt nur "NoTool").
* **Groesse der Aenderung** -- ab ``TOUCHUP_WARN_MM``/``_DEG`` warnt die
  Rueckfrage: das ist eher die falsche Zeile als ein Nachteachen.

Bewegt nichts. Schreibt nur in die Punkte-Datenbank -- an der Anlage sind
das die echten Punkte; die Rueckfrage nennt deshalb, woran sie schreibt.
"""

import json
from datetime import datetime
from pathlib import Path

import numpy as np

from . import config, geometry
from .ports import RobotError


def _deltas(robot, old_joints, new_joints):
    """(Verschiebung mm, Drehung Grad) der Toolspitze zwischen zwei Stellungen."""
    a = np.asarray(robot.fk(old_joints, frame="tool"), dtype=float)
    b = np.asarray(robot.fk(new_joints, frame="tool"), dtype=float)
    move_mm = 1e3 * float(np.linalg.norm(b[:3] - a[:3]))
    turn_deg = float(np.degrees(geometry.quat_angle_between(a[3:7], b[3:7])))
    return move_mm, turn_deg


def plan_touch_up(robot, name, clock, in_simulation, tool_name,
                  required_tool=config.TOUCHUP_TOOL_NAME,
                  still_wait_s=config.TOUCHUP_STILL_WAIT_S,
                  still_tol_rad=config.TOUCHUP_STILL_TOL_RAD):
    """Was wuerde ein Touch-up von ``name`` aendern? Schreibt nichts.

    ``in_simulation``/``tool_name``: was die Steuerung beim Verbinden
    gemeldet hat (``NeuraRobot.in_simulation``/``tool_name``; SimRobot:
    True/None). Rueckgabe: dict mit ``problems`` (blockierend) und
    ``warnings`` (Rueckfrage), dazu Stellung und Aenderung.
    """
    name = (name or "").strip()
    plan = {"name": name, "exists": False, "joints": None, "old_joints": None,
            "move_mm": None, "turn_deg": None, "tool": tool_name,
            "in_simulation": in_simulation, "problems": [], "warnings": []}
    if not name:
        plan["problems"].append("Die Zeile hat keinen Punktnamen.")
        return plan
    if name.startswith("_"):
        plan["problems"].append("Namen mit '_' am Anfang sind Hilfspunkten vorbehalten.")
        return plan

    first = np.asarray(robot.read_state().joints, dtype=float)
    clock.sleep(still_wait_s)
    joints = np.asarray(robot.read_state().joints, dtype=float)
    plan["joints"] = [float(v) for v in joints]
    moving = float(np.max(np.abs(joints - first)))
    if moving > still_tol_rad:
        plan["problems"].append(
            "Der Arm bewegt sich noch (%.4f rad in %.1f s) -- erst anhalten."
            % (moving, still_wait_s))

    if in_simulation is not True:
        if not tool_name or tool_name == "NoTool" or (required_tool and tool_name != required_tool):
            plan["problems"].append(
                "Aktives Werkzeug ist %r, fuer die Aufnahme gilt %r. Ein Punkt "
                "speichert kein Werkzeug -- die Pose wuerde mit dem falschen "
                "berechnet. Am Pendant das Werkzeug waehlen und neu verbinden."
                % (tool_name, required_tool))
    elif required_tool and tool_name != required_tool:
        plan["warnings"].append(
            "Werkzeug %r statt %r -- in der Simulation zugelassen." % (tool_name, required_tool))

    if name in set(robot.point_names()):
        plan["exists"] = True
        try:
            old_joints, _pose = robot.get_point(name)
        except RobotError as exc:
            # Gegenprobe gescheitert: mit einem anderen Werkzeug geteacht.
            # Der Touch-up rechnet ihn mit dem aktiven neu -- das ist dann
            # gerade die Korrektur, aber es soll bewusst geschehen.
            old_joints = robot.point_record(name)["jointAngles"]
            plan["warnings"].append(
                "Der bisherige Punkt passt nicht zum aktiven Werkzeug (%s). Der "
                "Touch-up speichert ihn mit %r neu." % (exc, tool_name))
        plan["old_joints"] = [float(v) for v in old_joints]
        plan["move_mm"], plan["turn_deg"] = _deltas(robot, old_joints, joints)
        if plan["move_mm"] > config.TOUCHUP_WARN_MM or plan["turn_deg"] > config.TOUCHUP_WARN_DEG:
            plan["warnings"].append(
                "Grosse Aenderung: %.0f mm / %.0f Grad -- richtige Zeile?"
                % (plan["move_mm"], plan["turn_deg"]))
    else:
        plan["warnings"].append("Punkt gibt es noch nicht -- er wird neu angelegt.")
    return plan


def describe(plan):
    """Text fuer die Rueckfrage."""
    if plan["in_simulation"] is True:
        ziel = "Simulation"
    elif plan["in_simulation"] is False:
        ziel = "REALE ANLAGE -- echte Punkte-Datenbank"
    else:
        ziel = "Steuerung unbekannt"
    lines = ["Punkt: %s  (%s)" % (plan["name"], "aendern" if plan["exists"] else "neu anlegen"),
             "Ziel: %s" % ziel,
             "Werkzeug: %s" % (plan["tool"] or "-")]
    if plan["joints"] is not None:
        lines.append("Neue Gelenkstellung (rad): %s"
                     % ", ".join("%.4f" % v for v in plan["joints"]))
    if plan["move_mm"] is not None:
        lines.append("Aenderung der Werkzeugspitze: %.1f mm, %.1f Grad"
                     % (plan["move_mm"], plan["turn_deg"]))
    for text in plan["problems"]:
        lines.append("BLOCKIERT: " + text)
    for text in plan["warnings"]:
        lines.append("Hinweis: " + text)
    return "\n".join(lines)


def apply_touch_up(robot, plan, backup_dir, now=None):
    """Alten Datensatz sichern, dann schreiben. Rueckgabe: (Sicherung, neuer Datensatz).

    Die Sicherung wird VOR dem Schreiben angelegt -- scheitert das
    Schreiben halb, ist der alte Stand trotzdem da.
    """
    if plan["problems"]:
        raise RobotError("Touch-up '%s' nicht freigegeben: %s"
                         % (plan["name"], "; ".join(plan["problems"])))
    name = plan["name"]
    old = robot.point_record(name) if name in set(robot.point_names()) else None
    now = now or datetime.now()
    backup_dir = Path(backup_dir)
    backup_dir.mkdir(parents=True, exist_ok=True)
    # Eindeutig, auch bei zwei Touch-ups in derselben Sekunde -- sonst
    # ueberschriebe die zweite Sicherung die erste (Befund GUI-Test 21.10.).
    stem = "%s_%s" % (now.strftime("%Y-%m-%d_%H-%M-%S"), name)
    backup = backup_dir / (stem + ".json")
    nr = 2
    while backup.exists():
        backup = backup_dir / ("%s_%d.json" % (stem, nr))
        nr += 1
    backup.write_text(json.dumps({
        "name": name,
        "saved_at": now.isoformat(timespec="seconds"),
        "existed": old is not None,
        "record": old,
        "new_joints": plan["joints"],
        "tool": plan["tool"],
        "in_simulation": plan["in_simulation"],
    }, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    record = robot.store_point(name, plan["joints"])
    return backup, record


def undo_touch_up(robot, backup, force=False):
    """Sicherung zurueckspielen. Rueckgabe: Text fuer die Anzeige.

    Steht der Punkt inzwischen woanders als der Touch-up ihn gesetzt hat
    (am Pendant erneut geteacht), wird ohne ``force`` nichts ueberschrieben.
    """
    data = json.loads(Path(backup).read_text(encoding="utf-8"))
    name = data["name"]
    if name in set(robot.point_names()) and data.get("new_joints") is not None and not force:
        now = np.asarray(robot.point_record(name)["jointAngles"], dtype=float)
        if float(np.max(np.abs(now - np.asarray(data["new_joints"], dtype=float)))) > 1e-6:
            raise RobotError("Punkt '%s' wurde nach dem Touch-up erneut geaendert -- "
                             "Ruecknahme nicht ausgefuehrt." % name)
    if data["existed"]:
        robot.restore_point(data["record"])
        return "Punkt %s auf den Stand vom %s zurueckgesetzt." % (name, data["saved_at"])
    if name in set(robot.point_names()):
        robot.delete_point(name)
    return "Neu angelegten Punkt %s wieder geloescht." % name

"""Teach-Serie: Punkte der Ablauftabelle mit den Knoepfen am Arm teachen -- Qt-frei.

Wunsch Anwender 2026-10-21: Zeile markieren (z. B. APPROACH_01), dann nur
noch am Arm arbeiten -- FREE gedrueckt halten und den Arm fuehren, POINT
druecken: der Punkt der markierten Zeile wird per Touch-up gesetzt, die
Markierung springt auf die naechste Zeile.

Wie die Knoepfe wirken, legt die Steuerung fest (Pendant, Einstellungen
der Arm-Knoepfe; Sicherung ``hmis.json``: ``freeButton``/``pointButton``,
ab Werk "noAction" -> Meldung "no option is selected for this"):

* FREE -> Zero-G. Laeuft ganz in der Steuerung, hier ist nichts zu tun.
* POINT -> "Save Point". Die Steuerung legt einen Punkt mit eigenem Namen
  an. :class:`NewPointWatcher` sieht ihn in der Punktliste, die
  Oberflaeche uebernimmt seine Gelenke mit :func:`touchup.plan_touch_up`
  (``joints=...``) und loescht ihn danach (:func:`consume_point`).

Ob "Save Point" wirklich einen Punkt anlegt, den die API sieht, und wie er
heisst, ist an der Anlage mit ``tools/probe_arm_buttons.py`` zu pruefen
(Stand 21.10.: nicht geprueft). Ohne Knopf geht dasselbe ueber die Taste
F8 in der Oberflaeche -- dann mit der aktuellen Stellung des Arms.

Reihenfolge PICK / PRE_GRASP braucht keine Sonderregel: PRE_GRASP steht im
Ablauf vor UND nach PICK. Der erste wird grob gesetzt (oder mit "Zeile
ueberspringen" ausgelassen), der zweite nach "Z hoch" ueber PICK
ueberschreibt ihn.

Bewegt nichts. Schreibt in die Punkte-Datenbank (Touch-up) und loescht den
vom Knopf angelegten Punkt.
"""

import re

from . import config


class NewPointWatcher(object):
    """Meldet Punkte, die seit dem Start neu sind.

    Neue Namen landen in :attr:`pending` -- in der Reihenfolge, in der die
    Steuerung sie liefert. Nicht gemeldet werden Hilfspunkte (``_...``,
    z. B. der des Touch-ups), Namen aus ``protected`` (die Punkte der
    Ablauftabelle -- wer sie am Pendant anlegt, meint sie so, und ein
    Touch-up legt fehlende selbst an) und, wenn ein Muster gesetzt ist,
    alles, was nicht darauf passt.
    """

    def __init__(self, robot, pattern=config.TEACH_SERIES_POINT_PATTERN):
        self.robot = robot
        self.pattern = re.compile(pattern) if pattern else None
        self.pending = []
        self._known = set(str(n) for n in robot.point_names())

    def accepts(self, name, protected=()):
        if name.startswith("_") or name in set(protected):
            return False
        return self.pattern is None or self.pattern.fullmatch(name) is not None

    def poll(self, protected=()):
        """Punktliste lesen; neue, passende Namen an :attr:`pending` haengen.
        Rueckgabe: die in diesem Aufruf neu gefundenen."""
        names = [str(n) for n in self.robot.point_names()]
        found = [n for n in names if n not in self._known and self.accepts(n, protected)]
        # Ganz neu setzen statt ergaenzen: ein verbrauchter (geloeschter)
        # Name faellt heraus und wird erkannt, falls die Steuerung ihn beim
        # naechsten Druck wieder vergibt.
        self._known = set(names)
        self.pending.extend(found)
        return found

    def forget(self, name):
        """Nach :func:`consume_point`: vergibt die Steuerung denselben Namen
        beim naechsten Druck wieder, soll er erneut als neu gelten -- auch
        wenn dazwischen kein :meth:`poll` lag."""
        self._known.discard(name)


def joints_of(robot, name):
    """Gelenkstellung eines Punkts aus seinem Datensatz (rad)."""
    return [float(v) for v in robot.point_record(name)["jointAngles"]]


def consume_point(robot, name):
    """Den vom Knopf angelegten Punkt loeschen. Nur echte Punkte -- ein
    anderer Datensatztyp (z. B. Frame) bleibt stehen. Rueckgabe: geloescht?"""
    if name not in set(robot.point_names()):
        return False
    if robot.point_record(name).get("type", "point") != "point":
        return False
    robot.delete_point(name)
    return True


def needs_confirm(plan):
    """Auffaellig -- deutet auf die falsche Zeile? (grosse Aenderung, alter
    Punkt passt nicht zum Werkzeug; "neu angelegt" allein nicht). Die Serie
    haelt dafuer NICHT an (Anlage 21.10.: Rueckfrage je Punkt war laestig),
    sondern zeigt es gross an und markiert es in der Schlussabfrage."""
    return bool(plan["large_change"] or plan["old_tool_mismatch"])


def summary(plan):
    """Eine Zeile fuers Protokoll."""
    if plan["exists"]:
        return "%s geändert (%.1f mm, %.1f°)" % (plan["name"], plan["move_mm"], plan["turn_deg"])
    return "%s neu angelegt" % plan["name"]


def entry(plan, backup):
    """Ein gesetzter Punkt der Serie -- fuer Schlussabfrage und Ruecknahme."""
    return {"name": plan["name"], "backup": backup, "summary": summary(plan),
            "flagged": needs_confirm(plan)}


def review_text(entries):
    """Text der Schlussabfrage: was die Serie geschrieben hat."""
    lines = ["Die Serie hat %d Punkt(e) gesetzt:" % len(entries), ""]
    for item in entries:
        lines.append(("⚠ " if item["flagged"] else "   ") + item["summary"])
    if any(item["flagged"] for item in entries):
        lines += ["", "⚠ = große Änderung oder alter Punkt mit anderem Werkzeug geteacht."]
    return "\n".join(lines)


def undo_all(robot, entries):
    """Alle Touch-ups der Serie zuruecknehmen -- rueckwaerts, damit ein
    zweimal gesetzter Punkt (PRE_GRASP) wieder auf dem Stand VOR der Serie
    landet. Haelt beim ersten Fehler an (z. B. am Pendant inzwischen
    geaendert). Rueckgabe: (Meldungen, Fehler oder None)."""
    from .touchup import undo_touch_up

    done = []
    for item in reversed(entries):
        try:
            done.append(undo_touch_up(robot, item["backup"]))
        except Exception as exc:
            return done, "%s: %s" % (item["name"], exc)
    return done, None


def next_row(row, row_count, skip=()):
    """Zeile nach ``row`` -- oder -1, wenn die Serie am Ende ist.

    ``skip``: Zeilen, die nicht geteacht werden (berechnete Punkte,
    bc/sequence.py) -- die Serie springt darueber hinweg.
    """
    nxt = row + 1
    while 0 <= nxt < row_count and nxt in skip:
        nxt += 1
    return nxt if 0 <= nxt < row_count else -1

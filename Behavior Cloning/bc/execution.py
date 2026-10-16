"""Ausfuehrungsart der Sollwerte: servo_j oder movelinear_online (AP 1.3 / 4.1).

Hardwarefrei. Legt fest, WIE ein Gelenkziel je 15-Hz-Takt an die Steuerung
geht -- in Aufzeichnung und Inferenz identisch, weil das Folgeverhalten des
Arms mitgelernt wird. Begruendung, Messungen und verworfene Wege:
``Dokumentation/Zittern-an-der-Anlage.md``.

* ``servo_j``: 60 Hz linear interpoliert (bc/servo.py). Faehrt an der
  Anlage als Treppe und regt eine Eigenschwingung an.
* ``servo_j_pos``: dieselben Sollpositionen, aber v = a = 0 (Profil
  ``pos`` in bc/servo.py). VM 2026-10-14 (tools/check_servo_stream.py):
  die Steuerung glaettet dann selbst -- Welligkeit quer 0,09 statt
  0,46 mm, dafuer ~70 ms Verzug und ~1 mm Abkuerzen in Ecken, also wie
  Override 0,5. An der Anlage UNGEPRUEFT: ohne Glaettung waere v = 0
  "an jedem Sollwert anhalten" und damit mehr Stop-and-go.
* ``online``: ``movelinear_online`` mit EINEM kartesischen Ziel je Takt
  (FK des Gelenkziels), ``lead_steps`` Takte voraus; die Steuerung plant
  selbst innerhalb der Hoechstwerte. Die Action bleibt im Gelenkraum --
  nur die Ausfuehrung ist kartesisch.

Die Hoechstwerte werden aus dem PLANER-Tempo abgeleitet, nicht aus einer
konkreten Bahn: so stehen in Aufzeichnung und Inferenz dieselben Zahlen,
auch wenn die Policy die Bahn erst waehrend der Fahrt vorgibt.

"Erst weiter, wenn erreicht" (nur ``online``, Anlage 2026-10-16): der Arm
laeuft ~1,3 s hinter dem Ziel her; mit 0,5 s Greifer-Dwell kehrte er 27 mm
ueber PICK um, ohne zu halten. An jedem Greiferwechsel (Haltepunkt) bleibt
das Ziel deshalb stehen, bis der TCP innerhalb ``reach_tol_mm`` steht
(:func:`wait_until_reached`); erst dann Greiferbefehl und der volle Dwell --
der Vorlauf schaut bis zum Ende des Dwells nicht ueber den Haltepunkt
hinaus (:func:`lead_index` mit ``hold_end``). Die Wartezeit liegt AUSSERHALB
der 15-Hz-Schritte: Aufnahme und Inferenz warten an derselben Stelle, die
Schrittfolge der Daten bleibt die des Plans. Die Regel steht mit ihren
Werten im Eintrag ``execution`` (``reach_*``); aeltere Aufnahmen ohne diese
Schluessel fahren wie bisher ohne Warten.
"""

import math

import numpy as np

from . import config
from .sync import Pacer

SERVO_J = "servo_j"
SERVO_J_POS = "servo_j_pos"
ONLINE = "online"
AUTO = "auto"
MODES = (SERVO_J, SERVO_J_POS, ONLINE)
#: Ausfuehrungsarten ueber servo_j (Interpolator, SERVO_RATE_HZ) -> Profil
SERVO_PROFILES = {SERVO_J: "linear", SERVO_J_POS: "pos"}


def resolve_mode(requested, robot_kind, in_simulation):
    """``auto`` -> online nur an der realen Anlage, sonst servo_j.

    Die VM (v5.0.0-alpha.102) nimmt movelinear_online an, faehrt aber nicht
    (2026-10-12); der SimRobot bildet es nur nach. Ausdruecklich gewaehlt
    gilt die Wahl ueberall.
    """
    if requested in MODES:
        return requested
    if requested not in (None, AUTO):
        raise ValueError("Ausfuehrung '%s' unbekannt (%s, %s)" % (requested, AUTO, ", ".join(MODES)))
    return ONLINE if robot_kind == "neura" and in_simulation is False else SERVO_J


def online_limits(transit_speed, ptp_speed, speed_factor=config.ONLINE_SPEED_FACTOR,
                  accel_factor=config.ONLINE_ACCEL_FACTOR, ramp_s=config.SEGMENT_RAMP_S):
    """Hoechstwerte fuer movelinear_online: ([v x3, w x3], [a x3, alpha x3]).

    Bezug ist die Spitze des Planers: TCP-Tempo im Transit, Drehung so
    schnell wie das schnellste PTP-Gelenk; Spitzenbeschleunigung der
    Sinus-Rampe v * pi / (2 T). Die Faktoren entsprechen denen des
    Messwerkzeugs (dort bezogen auf die Testbahn).
    """
    if ramp_s <= 0:
        raise ValueError("ramp_s muss > 0 sein")
    v_lin = speed_factor * float(transit_speed)
    v_rot = speed_factor * float(ptp_speed)
    a_lin = accel_factor * float(transit_speed) * math.pi / (2.0 * ramp_s)
    a_rot = accel_factor * float(ptp_speed) * math.pi / (2.0 * ramp_s)
    return [v_lin] * 3 + [v_rot] * 3, [a_lin] * 3 + [a_rot] * 3


def describe(mode, transit_speed=config.TRANSIT_SPEED_MS, ptp_speed=config.PTP_JOINT_SPEED_RADS,
             lead_steps=config.ONLINE_LEAD_STEPS, speed_factor=config.ONLINE_SPEED_FACTOR,
             accel_factor=config.ONLINE_ACCEL_FACTOR, ramp_s=config.SEGMENT_RAMP_S,
             reach_tol_mm=config.ONLINE_REACH_TOL_MM,
             reach_settle_steps=config.ONLINE_REACH_SETTLE_STEPS,
             reach_timeout_s=config.ONLINE_REACH_TIMEOUT_S,
             dwell_steps=config.GRIPPER_DWELL_STEPS):
    """Eintrag ``execution`` fuer die Metadaten -- alles, was die Inferenz
    gleich machen muss. ``servo_j`` braucht nur den Modus (die Rate steht
    seit jeher in ``servo_rate_hz``).

    ``reach_tol_mm=None`` schaltet "erst weiter, wenn erreicht" ab (so
    fahren aeltere Aufnahmen ohne die Schluessel). ``hold_dwell_steps``
    braucht nur die Inferenz: sie kennt das Ende des Dwells nicht aus einem
    Plan.
    """
    if mode in SERVO_PROFILES:
        return {"mode": mode}
    if mode != ONLINE:
        raise ValueError("Ausfuehrung '%s' unbekannt" % mode)
    if not 0 <= int(lead_steps) <= 5:
        raise ValueError("Vorlauf %r ausserhalb 0..5 Takte" % lead_steps)
    vel, acc = online_limits(transit_speed, ptp_speed, speed_factor, accel_factor, ramp_s)
    out = {"mode": ONLINE, "lead_steps": int(lead_steps),
           "speed_factor": float(speed_factor), "accel_factor": float(accel_factor),
           "vel": vel, "acc": acc}
    if reach_tol_mm is not None:
        if not 0.0 < float(reach_tol_mm) <= 20.0:
            raise ValueError("reach_tol_mm %r ausserhalb (0, 20] mm" % reach_tol_mm)
        if int(reach_settle_steps) < 1:
            raise ValueError("reach_settle_steps muss >= 1 sein")
        if not 0.0 < float(reach_timeout_s) <= 30.0:
            raise ValueError("reach_timeout_s %r ausserhalb (0, 30] s" % reach_timeout_s)
        out.update({"reach_tol_mm": float(reach_tol_mm),
                    "reach_settle_steps": int(reach_settle_steps),
                    "reach_timeout_s": float(reach_timeout_s),
                    "hold_dwell_steps": int(dwell_steps)})
    return out


def reach_rule(execution):
    """True, wenn die Ausfuehrung an Greiferwechseln auf das Erreichen wartet."""
    execution = execution or {}
    return execution.get("mode") == ONLINE and execution.get("reach_tol_mm") is not None


def mode_of(metadata):
    """Ausfuehrungsart einer Episode; aeltere Aufnahmen ohne Eintrag: servo_j."""
    return ((metadata or {}).get("execution") or {}).get("mode", SERVO_J)


def servo_profile(execution):
    """Profil fuer bc.servo.ServoInterpolator; None bei ``online``."""
    return SERVO_PROFILES.get((execution or {}).get("mode", SERVO_J))


def lead_index(i, lead_steps, n, hold_end=None):
    """Index des Ziels, das im Takt ``i`` gesendet wird (am Ende festgehalten).

    ``hold_end``: letzter Dwell-Takt des naechsten Haltepunkts (siehe
    :func:`hold_blocks`). Bis dahin schaut der Vorlauf nicht darueber hinaus
    -- sonst liefe das Abfahrtsziel ``lead_steps`` Takte vor Ende des Dwells
    los, und der Greifer haette weniger als GRIPPER_DWELL_S.
    """
    j = min(int(i) + int(lead_steps), int(n) - 1)
    if hold_end is not None and int(i) <= int(hold_end):
        j = min(j, int(hold_end))
    return j


def hold_blocks(gripper, dwell_mask):
    """Haltepunkte eines Plans: Liste ``(start, ende)`` je Greiferwechsel.

    ``start`` ist der erste Takt mit dem neuen Greiferwert (dort setzt der
    Recorder den Befehl ab), ``ende`` der letzte Dwell-Takt danach. Der
    Planer legt beides zusammen (bc/trajectory.py: der Dwell beginnt mit
    dem Wechsel).
    """
    gripper = np.asarray(gripper, dtype=float)
    dwell = np.asarray(dwell_mask, dtype=bool)
    blocks = []
    for start in range(1, len(gripper)):
        if gripper[start] == gripper[start - 1]:
            continue
        end = start
        while end + 1 < len(dwell) and dwell[end + 1]:
            end += 1
        blocks.append((start, end))
    return blocks


def next_hold_end(i, blocks):
    """Ende des naechsten Haltepunkts ab Takt ``i`` (oder None)."""
    ends = [end for _, end in blocks if end >= int(i)]
    return min(ends) if ends else None


def wait_until_reached(robot, target, execution, clock, rate_hz=config.CONTROL_RATE_HZ,
                       target_tcp=None, measure_tcp=None):
    """Haelt ``target`` (Gelenkziel), bis der TCP innerhalb ``reach_tol_mm`` steht.

    Je 15-Hz-Takt: Abstand messen, sonst dasselbe Ziel erneut per
    ``online_j`` senden -- der Sollwertstrom bleibt am Haltepunkt stehen.
    Erreicht = ``reach_settle_steps`` Messungen in Folge innerhalb der
    Toleranz. Kein Not-Halt bei Zeitueberschreitung: der Arm faehrt auf ein
    ruhendes Ziel zu, der Aufrufer beendet die Fahrt geordnet.

    Abstand im TCP-Raum, beide Seiten aus derselben Kette: Ziel einmal per
    ``robot.fk``, Messung aus ``read_state().tcp_quat`` (an der Anlage lokal
    aus der beim Verbinden gegen die Steuerung geprueften Kette).
    Werkzeuge koennen ``target_tcp`` und ``measure_tcp`` (-> Position in m)
    selbst vorgeben.

    Rueckgabe: dict ``reached``, ``wait_s``, ``ticks`` (erneut gesendete
    Ziele), ``start_mm``, ``dist_mm`` (zuletzt), ``min_mm``, ``tol_mm``,
    ``reason`` ("", "schutzstopp", "zeit").
    """
    tol = float(execution["reach_tol_mm"])
    settle = int(execution.get("reach_settle_steps", 1))
    timeout = float(execution["reach_timeout_s"])
    target = np.asarray(target, dtype=float)
    if target_tcp is None:
        target_tcp = np.asarray(robot.fk(target), dtype=float)[:3]
    target_tcp = np.asarray(target_tcp, dtype=float)
    if measure_tcp is None:
        def measure_tcp():
            return np.asarray(robot.read_state().tcp_quat, dtype=float)[:3]
    # Den laufenden Takt verbrauchen: das erste erneute Senden kommt eine
    # Periode nach dem Ziel des Aufrufers, nicht direkt danach.
    pacer = Pacer(clock, rate_hz).start()
    pacer.tick()
    t0 = clock.now()
    result = {"reached": False, "wait_s": 0.0, "ticks": 0, "start_mm": None,
              "dist_mm": None, "min_mm": None, "tol_mm": tol, "reason": ""}
    in_tol = 0
    while True:
        if robot.stop_requested:
            result["reason"] = "schutzstopp"
            break
        dist = 1e3 * float(np.linalg.norm(measure_tcp() - target_tcp))
        if result["start_mm"] is None:
            result["start_mm"] = dist
        result["dist_mm"] = dist
        result["min_mm"] = dist if result["min_mm"] is None else min(result["min_mm"], dist)
        in_tol = in_tol + 1 if dist <= tol else 0
        if in_tol >= settle:
            result["reached"] = True
            break
        if clock.now() - t0 >= timeout:
            result["reason"] = "zeit"
            break
        pacer.tick()
        if robot.stop_requested:
            result["reason"] = "schutzstopp"
            break
        robot.online_j(target, execution["vel"], execution["acc"])
        result["ticks"] += 1
    result["wait_s"] = clock.now() - t0
    return result


def reach_failure_text(result, step=None):
    """Klartext fuer einen gescheiterten Halt (Abbruchgrund)."""
    if result["reason"] == "schutzstopp":
        return "schutzstopp"
    where = "" if step is None else " (Takt %d)" % step
    return ("haltepunkt nicht erreicht: noch %.1f mm (Toleranz %.1f mm) nach %.1f s%s"
            % (result["dist_mm"], result["tol_mm"], result["wait_s"], where))


def reach_line(halt):
    """Eine Ausgabezeile je Halt (Aufnahme und Inferenz)."""
    def mm(v):
        return "-" if v is None else "%.1f" % v
    return ("Halt Takt %s: %.2f s gewartet, %s -> %s mm%s"
            % (halt.get("step", "-"), halt["wait_s"], mm(halt["start_mm"]), mm(halt["dist_mm"]),
               "" if halt["reached"] else " NICHT ERREICHT (%s)" % halt["reason"]))


def reach_summary(waits):
    """Kennzahlen der Halte einer Fahrt fuer Metadaten/Protokoll."""
    waits = list(waits or [])
    return {
        "reach_waits": waits,
        "reach_wait_s": float(sum(w["wait_s"] for w in waits)),
        "reach_wait_max_s": float(max([w["wait_s"] for w in waits] or [0.0])),
    }

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
"""

import math

from . import config

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
             accel_factor=config.ONLINE_ACCEL_FACTOR, ramp_s=config.SEGMENT_RAMP_S):
    """Eintrag ``execution`` fuer die Metadaten -- alles, was die Inferenz
    gleich machen muss. ``servo_j`` braucht nur den Modus (die Rate steht
    seit jeher in ``servo_rate_hz``)."""
    if mode in SERVO_PROFILES:
        return {"mode": mode}
    if mode != ONLINE:
        raise ValueError("Ausfuehrung '%s' unbekannt" % mode)
    if not 0 <= int(lead_steps) <= 5:
        raise ValueError("Vorlauf %r ausserhalb 0..5 Takte" % lead_steps)
    vel, acc = online_limits(transit_speed, ptp_speed, speed_factor, accel_factor, ramp_s)
    return {"mode": ONLINE, "lead_steps": int(lead_steps),
            "speed_factor": float(speed_factor), "accel_factor": float(accel_factor),
            "vel": vel, "acc": acc}


def mode_of(metadata):
    """Ausfuehrungsart einer Episode; aeltere Aufnahmen ohne Eintrag: servo_j."""
    return ((metadata or {}).get("execution") or {}).get("mode", SERVO_J)


def servo_profile(execution):
    """Profil fuer bc.servo.ServoInterpolator; None bei ``online``."""
    return SERVO_PROFILES.get((execution or {}).get("mode", SERVO_J))


def lead_index(i, lead_steps, n):
    """Index des Ziels, das im Takt ``i`` gesendet wird (am Ende festgehalten)."""
    return min(int(i) + int(lead_steps), int(n) - 1)

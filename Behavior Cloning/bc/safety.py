"""Watchdog und Geofencing (AP 4.2).

NeuraPy bietet keine 3D-Schutzraeume -- die Ueberwachung laeuft vollstaendig
hier: ein Watchdog prueft zyklisch eine Liste von Checks und loest bei der
ersten Verletzung den Software-Not-Halt aus (``robot.emergency_stop()``).

Der Watchdog laeuft in einem EIGENEN Thread, damit eine blockierende oder
verzoegerte Policy-Inferenz den Stopp nicht verzoegern kann. Fuer
deterministische Tests existiert ``run_once()`` ohne Thread.

WICHTIG: Software-Stopp ersetzt KEINEN zertifizierten Hardware-Not-Aus.
Ein physischer Not-Halt muss waehrend aller Laeufe in Reichweite sein.
"""

import threading
import traceback

from . import config


class Watchdog(object):
    """Prueft zyklisch Checks und loest den Not-Halt aus.

    ``checks``: Liste von Callables ohne Argumente. Rueckgabe None/leer =
    in Ordnung; alles andere gilt als Verletzung (und wird protokolliert).
    Wirft ein Check selbst eine Exception, gilt das ebenfalls als
    Verletzung -- ein kaputter Waechter darf nie "gruen" bedeuten.
    """

    def __init__(self, robot, checks, clock, period_s=config.SAFETY_CHECK_PERIOD_S):
        self.robot = robot
        self.checks = list(checks)
        self.clock = clock
        self.period_s = period_s
        self.tripped = None  # erste Verletzung (fuer Diagnose/Episode)
        self._thread = None
        self._running = threading.Event()

    # -- Kernpruefung ------------------------------------------------------

    def run_once(self):
        """Fuehrt alle Checks einmal aus. Rueckgabe: Verletzung oder None."""
        for check in self.checks:
            try:
                result = check()
            except Exception as exc:  # kaputter Check == Verletzung
                result = "check-exception: %s\n%s" % (exc, traceback.format_exc())
            if result:
                self.tripped = result
                self.robot.emergency_stop()
                return result
        return None

    # -- Thread-Betrieb ----------------------------------------------------

    def start(self):
        self._running.set()
        self._thread = threading.Thread(
            target=self._loop, name="safety-watchdog", daemon=True
        )
        self._thread.start()
        return self

    def _loop(self):
        while self._running.is_set():
            if self.run_once() is not None:
                return  # Not-Halt ausgeloest, Watchdog-Aufgabe erledigt
            self.clock.sleep(self.period_s)

    def stop(self):
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


def geofence_check(robot, collision_model):
    """Baut einen Watchdog-Check aus dem Kollisionsmodell (AP 4.2).

    Dieselben Quader wie bei der Offline-Vorabpruefung (AP 2.4) -- online
    wiederverwendet: aktuelle Gelenkstellung lesen, Stuetzpunkte gegen die
    Quader pruefen.
    """

    def check():
        state = robot.read_state()
        violations = collision_model.check_positions(
            robot.link_positions(state.joints)
        )
        return violations or None

    return check

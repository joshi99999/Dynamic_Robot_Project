"""Tippbetrieb "halten = fahren" fuer die Bedienoberflaeche -- Qt-frei.

Wunsch Anwender 2026-10-21: im Aufnahme-Reiter ein Knopf, der den Arm
senkrecht nach oben faehrt, solange er gedrueckt ist -- im Teach-Modus, um
PRE_GRASP genau ueber PICK zu teachen.

Der Reiter ruft :meth:`Jogger.tick` aus einem Zeitgeber (config.JOG_TICK_HZ)
und nur, solange der Knopf unten ist. Die Steuerung faehrt nur, solange
``jog`` kommt (ports.JogPort) -- haengt die Oberflaeche, bleibt der Arm von
selbst stehen. Zusaetzlich haelt der Jogger nach ``config.JOG_MAX_S`` an
und bei jedem Fehler eines Aufrufs. Bewegt wird nur ueber den Adapter,
dessen Sperren (Simulation/Freigabe, Teach-Modus) gelten.

Ein Software-Tippbetrieb ersetzt keinen Zustimmtaster und keinen Not-Aus
(AP 4.2).
"""

from .. import config


def z_up_velocity(fraction=config.JOG_Z_FRACTION):
    """Anteile fuer "senkrecht nach oben" (Basis-z)."""
    return [0.0, 0.0, float(fraction), 0.0, 0.0, 0.0]


class Jogger(object):
    """Ein Tippvorgang: start -> tick ... -> stop."""

    def __init__(self, robot, velocity, clock, max_s=config.JOG_MAX_S):
        self.robot = robot
        self.velocity = list(velocity)
        self.clock = clock
        self.max_s = float(max_s)
        self.active = False
        self.ticks = 0
        #: Warum zuletzt angehalten wurde ("" = losgelassen).
        self.reason = ""
        self._t0 = None

    def start(self):
        """Schaltet den Tippbetrieb ein. Wirft, wenn der Adapter verweigert
        (keine Freigabe, nicht im Teach-Modus) -- dann faehrt nichts."""
        if not hasattr(self.robot, "jog_start"):
            raise RuntimeError("Dieser Roboter kann keinen Tippbetrieb.")
        self.robot.jog_start(self.velocity)
        self.active = True
        self.ticks = 0
        self.reason = ""
        self._t0 = self.clock.now()

    def tick(self):
        """Ein Schritt. Liefert False, wenn (jetzt) angehalten ist."""
        if not self.active:
            return False
        if self.clock.now() - self._t0 > self.max_s:
            self.stop("Zeitlimit %.0f s erreicht -- neu drücken, um weiterzufahren" % self.max_s)
            return False
        try:
            self.robot.jog_tick()
        except Exception as exc:
            self.stop("Fehler: %s" % exc)
            return False
        self.ticks += 1
        return True

    def stop(self, reason=""):
        """Anhalten. Mehrfach aufrufbar, wirft nie."""
        if self.active:
            self.active = False
            if reason:
                self.reason = reason
            try:
                self.robot.jog_stop()
            except Exception:
                pass

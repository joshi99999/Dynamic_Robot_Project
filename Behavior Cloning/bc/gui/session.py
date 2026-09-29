"""Gemeinsamer Zustand der Oberflaeche: Backend, Roboter, Kameras -- Qt-frei.

Qt-frei, damit sich alles ohne Bildschirm testen laesst und damit dieselbe
Schicht unter der eigenstaendigen GUI und unter dem Plugin-Reiter liegt.

DIE WICHTIGSTE ENTSCHEIDUNG HIER: Die Anzeige sagt nicht, was ausgewaehlt
wurde, sondern was der Controller auf ``is_robot_in_simulation()`` geantwortet
hat. VM und reale Control-Box hoeren auf dieselbe Adresse -- eine Anzeige,
die der Auswahl folgt, wuerde am Labortag genau dann luegen, wenn es darauf
ankommt. Solange nicht verbunden ist, steht die Anzeige auf "unbekannt".

Das ist eine Software-Anzeige und ersetzt keinen zertifizierten
Hardware-Not-Aus (AP 4.2).
"""

import logging

from .. import config

log = logging.getLogger(__name__)

#: Backends zur Auswahl.
#:   SIM    SimRobot im Prozess, kein Controller, keine Anlage in der Naehe
#:   NEURA  NeuraPy-Controller -- ob das die VM oder die Anlage ist, sagt
#:          erst die Abfrage beim Verbinden
BACKEND_SIM = "sim"
BACKEND_NEURA = "neura"

BACKEND_LABELS = {
    BACKEND_SIM: "Simulation (ohne Controller)",
    BACKEND_NEURA: "Neura-Steuerung",
}

#: Zustaende der Backend-Anzeige.
OFFLINE = "offline"                 # nicht verbunden
SIM_LOCAL = "sim_local"             # SimRobot im Prozess
SIM_CONTROLLER = "sim_controller"   # Controller meldet Simulation
REAL = "real"                       # Controller meldet KEINE Simulation, Bewegung freigegeben
BLOCKED = "blocked"                 # keine Simulation bestaetigt, Bewegung gesperrt
UNKNOWN = "unknown"                 # Abfrage gescheitert

#: Klartext und Farbe je Zustand. Rot ist ausschliesslich fuer "reale Anlage"
#: reserviert, damit die Farbe am Labortag eine Bedeutung behaelt.
STATUS_TEXT = {
    OFFLINE: "nicht verbunden",
    SIM_LOCAL: "Simulation im Prozess -- keine Steuerung verbunden",
    SIM_CONTROLLER: "virtuelle Steuerung -- Simulation bestaetigt",
    REAL: "REALE ANLAGE -- Bewegung freigegeben",
    BLOCKED: "Simulation nicht bestaetigt -- Bewegung gesperrt",
    UNKNOWN: "Simulationsabfrage gescheitert -- Bewegung gesperrt",
}

STATUS_COLOUR = {
    OFFLINE: "#616161",
    SIM_LOCAL: "#1565c0",
    SIM_CONTROLLER: "#2e7d32",
    REAL: "#c62828",
    BLOCKED: "#e65100",
    UNKNOWN: "#e65100",
}


class BackendStatus(object):
    """Was gerade wirklich dranhaengt -- Grundlage der Anzeige."""

    def __init__(self, state, backend, detail="", motion_allowed=False,
                 tool_name=None, gripper_mode=None):
        self.state = state
        self.backend = backend
        self.detail = detail
        self.motion_allowed = motion_allowed
        self.tool_name = tool_name
        self.gripper_mode = gripper_mode

    @property
    def text(self):
        return STATUS_TEXT[self.state]

    @property
    def colour(self):
        return STATUS_COLOUR[self.state]

    @property
    def is_real_plant(self):
        return self.state == REAL

    def __repr__(self):
        return "<BackendStatus %s motion=%s>" % (self.state, self.motion_allowed)


class Session(object):
    """Haelt Backendwahl und Roboterverbindung fuer alle Modi gemeinsam."""

    def __init__(self, backend=BACKEND_SIM, override=config.DEFAULT_OVERRIDE,
                 allow_real=False):
        self.backend = backend
        self.override = override
        #: Nur wahr, wenn der Bedienende die reale Anlage ausdruecklich
        #: freigegeben hat. Wird nie automatisch gesetzt.
        self.allow_real = bool(allow_real)
        self.robot = None
        self._connect_error = None
        self.listeners = []          # aufgerufen bei jeder Statusaenderung

    # -- Beobachter ---------------------------------------------------------

    def add_listener(self, callback):
        self.listeners.append(callback)

    def _notify(self):
        status = self.status()
        for callback in list(self.listeners):
            try:
                callback(status)
            except Exception:
                log.exception("Status-Listener gescheitert")

    # -- Auswahl ------------------------------------------------------------

    def set_backend(self, backend):
        if backend not in BACKEND_LABELS:
            raise ValueError("Unbekanntes Backend '%s'" % backend)
        if backend == self.backend:
            return
        self.disconnect()
        self.backend = backend
        self._notify()

    def set_allow_real(self, allowed):
        """Reale Anlage freigeben. Wirkt erst beim naechsten Verbinden."""
        allowed = bool(allowed)
        if allowed == self.allow_real:
            return
        self.disconnect()
        self.allow_real = allowed
        self._notify()

    def set_override(self, value):
        self.override = float(value)

    # -- Verbindung ---------------------------------------------------------

    def connect(self):
        """Verbindet das gewaehlte Backend. Bewegt nichts.

        ``power_on`` und ``ensure_automatic`` bleiben bewusst aus: Verbinden
        ist keine Bewegungsvorbereitung. Wer fahren will, tut das im
        jeweiligen Modus und bestaetigt dort.
        """
        from ..adapters import open_robot

        self.disconnect()
        self._connect_error = None
        try:
            if self.backend == BACKEND_SIM:
                robot = open_robot("sim")
            else:
                robot = open_robot("neura", override=self.override,
                                   allow_real=self.allow_real)
            robot.connect()
        except Exception as exc:
            self._connect_error = "%s: %s" % (type(exc).__name__, exc)
            self._notify()
            raise
        self.robot = robot
        status = self.status()
        log.info("Verbunden: %s (%s)", BACKEND_LABELS[self.backend], status.text)
        self._notify()
        return robot

    def disconnect(self):
        if self.robot is not None:
            try:
                self.robot.close()
            except Exception:
                log.exception("Trennen gescheitert")
            self.robot = None
            self._notify()

    @property
    def connected(self):
        return self.robot is not None

    # -- Anzeige ------------------------------------------------------------

    def status(self):
        """Der tatsaechliche Zustand -- Grundlage des Backend-Abzeichens."""
        if self.robot is None:
            return BackendStatus(OFFLINE, self.backend, self._connect_error or "")
        if self.backend == BACKEND_SIM:
            return BackendStatus(SIM_LOCAL, self.backend,
                                 "SimRobot -- nicht die LARA-5-Kinematik",
                                 motion_allowed=True)

        # Neura: die Antwort des Controllers entscheidet, nicht die Auswahl.
        in_simulation = getattr(self.robot, "in_simulation", None)
        motion_allowed = bool(getattr(self.robot, "motion_allowed", False))
        tool_name = getattr(self.robot, "tool_name", None)
        gripper_mode = getattr(self.robot, "gripper_mode", None)
        if in_simulation is True:
            state = SIM_CONTROLLER
        elif in_simulation is False:
            state = REAL if motion_allowed else BLOCKED
        else:
            state = UNKNOWN
        detail = "is_robot_in_simulation() = %r" % (in_simulation,)
        return BackendStatus(state, self.backend, detail,
                             motion_allowed=motion_allowed,
                             tool_name=tool_name, gripper_mode=gripper_mode)

    def close(self):
        self.disconnect()

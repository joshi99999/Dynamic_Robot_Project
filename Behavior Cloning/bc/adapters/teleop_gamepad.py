"""Gamepad-Teleoperation zum Teachen der Wegpunkte (AP 2.2).

WICHTIG: Das Gamepad dient AUSSCHLIESSLICH dem Teachen -- die Aufzeichnung
erfolgt automatisiert mit Rauscheinspielung (AP 2.4). Deshalb ist die
Schnittstelle bewusst schmal: joggen, Wegpunkt speichern, Greifer, beenden.

Backend: ``pygame`` (optionale Abhaengigkeit, siehe pyproject.toml Extra
``gamepad``). Der Import erfolgt erst beim Oeffnen, damit die Pipeline ohne
pygame importierbar bleibt. Belegung und Skalierung sind am realen Geraet
zu pruefen (Abnahmeliste AP 0.6).
"""

import numpy as np

from ..ports import TeleopEvent, TeleopPort

#: Verfahrschritt in Metern pro Poll bei Vollausschlag des Sticks.
JOG_STEP_M = 0.005

#: Totzone der Analogsticks.
DEADZONE = 0.15


class GamepadTeleop(TeleopPort):
    """Teach-Eingabe ueber ein Standard-Gamepad (pygame).

    Belegung (Vorschlag, am Geraet verifizieren):
        linker Stick   -- X/Y joggen
        rechter Stick  -- Z joggen (vertikal)
        Taste A (0)    -- Wegpunkt speichern
        Taste B (1)    -- Greifer togglen
        Taste Start(7) -- beenden
    """

    def __init__(self, index=0):
        self._index = index
        self._pygame = None
        self._joystick = None
        self._gripper_closed = False

    def open(self):
        try:
            import pygame
        except ImportError as exc:
            raise RuntimeError(
                "pygame ist nicht installiert. Installation: "
                "pip install .[gamepad] (siehe pyproject.toml). "
                "Fuer hardwarefreie Tests ScriptedTeleop verwenden."
            ) from exc
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= self._index:
            raise RuntimeError(
                "Kein Gamepad an Index %d gefunden (%d Geraete erkannt)."
                % (self._index, pygame.joystick.get_count())
            )
        self._pygame = pygame
        self._joystick = pygame.joystick.Joystick(self._index)
        self._joystick.init()
        return self

    def poll(self):
        if self._pygame is None:
            self.open()
        pg = self._pygame
        for event in pg.event.get():
            if event.type == pg.JOYBUTTONDOWN:
                if event.button == 0:
                    return TeleopEvent("save")
                if event.button == 1:
                    self._gripper_closed = not self._gripper_closed
                    return TeleopEvent("gripper", self._gripper_closed)
                if event.button == 7:
                    return TeleopEvent("quit")

        ax = _deadzone(self._joystick.get_axis(0))
        ay = _deadzone(-self._joystick.get_axis(1))
        az = _deadzone(-self._joystick.get_axis(3))
        if ax or ay or az:
            return TeleopEvent(
                "jog", np.array([ax, ay, az], dtype=float) * JOG_STEP_M
            )
        return None

    def close(self):
        if self._joystick is not None:
            self._joystick.quit()
            self._joystick = None
        if self._pygame is not None:
            self._pygame.joystick.quit()
            self._pygame = None


def _deadzone(value):
    return 0.0 if abs(value) < DEADZONE else float(value)

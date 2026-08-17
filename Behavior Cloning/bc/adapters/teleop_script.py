"""Skriptgesteuerte Teleoperation fuer Tests und Simulation (AP 0.5).

Spielt eine vorgegebene Ereignisfolge ab -- damit laesst sich der komplette
Teach-Ablauf (apps/teach.py) deterministisch und ohne Gamepad testen.
"""

from collections import deque

from ..ports import TeleopEvent, TeleopPort


class ScriptedTeleop(TeleopPort):
    """Gibt vorbereitete :class:`TeleopEvent` in Reihenfolge zurueck."""

    def __init__(self, events=()):
        self._events = deque(events)

    def push(self, event):
        self._events.append(event)
        return self

    def poll(self):
        if self._events:
            return self._events.popleft()
        return None

    def close(self):
        self._events.clear()


def teach_script(waypoints_jogs):
    """Baut eine typische Teach-Sequenz.

    ``waypoints_jogs`` ist eine Liste von Listen: je Wegpunkt die
    Jog-Vektoren, die vor dem Speichern gefahren werden.
    """
    events = []
    for jogs in waypoints_jogs:
        for jog in jogs:
            events.append(TeleopEvent("jog", jog))
        events.append(TeleopEvent("save"))
    events.append(TeleopEvent("quit"))
    return ScriptedTeleop(events)

"""Zeitquellen: echte Uhr und simulierte Uhr (AP 0.5).

Pipeline-Code bekommt die Uhr injiziert (Default: :class:`RealClock`).
Tests verwenden :class:`SimClock` -- damit laufen sie deterministisch und
schneller als Echtzeit, z. B. wird der 500-ms-Greifer-Dwell im Test in
Mikrosekunden "gewartet".
"""

import time

from .ports import ClockPort


class RealClock(ClockPort):
    """Host-Uhr. ``now()`` ist ``time.time()`` -- dieselbe Zeitbasis, mit
    der Frames und Roboterzustaende gestempelt werden (AP 1.3)."""

    def now(self):
        return time.time()

    def sleep(self, seconds):
        if seconds > 0:
            time.sleep(seconds)


class SimClock(ClockPort):
    """Manuell bzw. durch ``sleep()`` fortschreitende Uhr fuer Tests.

    ``sleep()`` springt sofort vorwaerts statt zu warten. ``advance()``
    erlaubt Tests, Zeit gezielt vergehen zu lassen (z. B. um veraltete
    Frames oder Latenzbudget-Verletzungen zu konstruieren).
    """

    def __init__(self, start=1000.0):
        # Start bewusst > 0: die Port-Zusage "Zeitstempel > 0" (ports.py)
        # gilt auch in der Simulation, und relative Rechnungen bleiben
        # davon unberuehrt.
        self._t = float(start)

    def now(self):
        return self._t

    def sleep(self, seconds):
        if seconds > 0:
            self._t += seconds

    def advance(self, seconds):
        self._t += float(seconds)
        return self._t

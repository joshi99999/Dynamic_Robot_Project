"""Zeitquellen: echte Uhr und simulierte Uhr (AP 0.5 / AP 1.3).

Pipeline-Code bekommt die Uhr injiziert (Default: :class:`RealClock`).
Tests verwenden :class:`SimClock` -- damit laufen sie deterministisch und
schneller als Echtzeit, z. B. wird der 500-ms-Greifer-Dwell im Test in
Mikrosekunden "gewartet".

Warum nicht ``time.time()``: Unter Windows ist das bis Python 3.12
``GetSystemTimeAsFileTime`` mit einer zugesicherten Aufloesung von nur
15,6 ms (gemessen 2026-09-14 in bc_env: ``resolution=0.015625``). Feinere
Schritte gibt es nur, solange irgendein anderer Prozess die Timer-Aufloesung
hochsetzt -- darauf darf ein 30-ms-Latenzbudget (AP 1.3) nicht bauen. Zudem
ist die Systemuhr verstellbar und springt bei der Zeitsynchronisation.

Deshalb laufen alle Host-Zeitstempel ueber :func:`host_time`: der monotone
``perf_counter`` (0,1 us Aufloesung), einmal pro Prozess an die Weltzeit
gekoppelt. Kameras, Roboter und Takt teilen damit dieselbe Zeitbasis.
"""

import time

from .ports import ClockPort

#: Kopplung perf_counter -> Weltzeit, einmal beim Import festgelegt. Der
#: absolute Versatz (Aufloesung von time.time()) ist fuer die Sync egal,
#: weil ALLE Quellen denselben Versatz tragen -- entscheidend ist, dass
#: die Differenzen fein aufgeloest und sprungfrei sind.
_EPOCH_OFFSET = time.time() - time.perf_counter()


def host_time():
    """Host-Zeitstempel in Sekunden: monoton, fein aufgeloest, weltzeitnah.

    Verbindliche Zeitquelle fuer Frame- und Roboterzeitstempel
    (ports.Frame, ports.RobotState).
    """
    return time.perf_counter() + _EPOCH_OFFSET


class RealClock(ClockPort):
    """Host-Uhr. ``now()`` ist :func:`host_time` -- dieselbe Zeitbasis, mit
    der Frames und Roboterzustaende gestempelt werden (AP 1.3)."""

    def now(self):
        return host_time()

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

"""Zeit-Synchronisation der Datenquellen (AP 1.3).

Die Quellen (zwei Kameras, Roboterzustand) laufen asynchron mit
unterschiedlichen Raten. Zusammengefuehrt wird auf das 15-Hz-Raster:
der Recorder greift zu jedem Zieltakt den juengsten Wert jeder Quelle ab
(ThreadedCapture/DirectCapture) und bewertet die Zeitstempel gegen das
Latenzbudget. Frames ausserhalb des Budgets werden markiert; Episoden mit
zu vielen markierten Frames werden verworfen (AP 5.2).
"""

from dataclasses import dataclass

from . import config


class Pacer(object):
    """Taktgeber fuer die Zielrate -- driftfrei ueber absolute Deadlines.

    ``tick()`` wartet bis zum naechsten Rasterpunkt und gibt dessen
    Sollzeit zurueck. Faellt die Schleife hinter den Takt zurueck, wird
    nicht "aufgeholt" (kein Burst), sondern der naechste Rasterpunkt in
    der Zukunft gewaehlt und der Rueckstand gezaehlt.
    """

    def __init__(self, clock, rate_hz=config.CONTROL_RATE_HZ):
        self.clock = clock
        self.period = 1.0 / rate_hz
        self._next = None
        self.overruns = 0

    def start(self):
        self._next = self.clock.now()
        return self

    def tick(self):
        if self._next is None:
            self.start()
        now = self.clock.now()
        if now > self._next + self.period:
            # Rueckstand: Rasterpunkte ueberspringen statt Burst zu senden
            missed = int((now - self._next) / self.period)
            self._next += missed * self.period
            self.overruns += missed
        wait = self._next - now
        if wait > 0:
            self.clock.sleep(wait)
        t = self._next
        self._next += self.period
        return t


@dataclass
class SyncReport:
    """Bewertung eines Sample-Satzes gegen das Latenzbudget."""

    t_target: float
    timestamps: dict  # Quelle -> Zeitstempel
    spread: float  # max - min der Zeitstempel
    max_age: float  # aeltester Wert relativ zum Zieltakt
    ok: bool

    @property
    def worst_source(self):
        if not self.timestamps:
            return None
        return min(self.timestamps, key=self.timestamps.get)


def evaluate(t_target, timestamps, max_skew=config.SYNC_MAX_SKEW_S):
    """Bewertet die Zeitstempel aller Quellen eines Zieltakts.

    ``timestamps``: dict Quelle -> Zeitstempel (None = Quelle hat noch
    nie geliefert -> automatisch nicht ok).

    Kriterium (AP 1.3): die maximale Differenz zwischen den Quellen darf
    ``max_skew`` nicht ueberschreiten. Zusaetzlich wird das Alter relativ
    zum Zieltakt gemeldet (Diagnose, z. B. haengende Kamera).
    """
    if not timestamps or any(v is None for v in timestamps.values()):
        return SyncReport(
            t_target=t_target,
            timestamps=dict(timestamps),
            spread=float("inf"),
            max_age=float("inf"),
            ok=False,
        )
    values = list(timestamps.values())
    spread = max(values) - min(values)
    max_age = t_target - min(values)
    return SyncReport(
        t_target=t_target,
        timestamps=dict(timestamps),
        spread=spread,
        max_age=max_age,
        ok=spread <= max_skew,
    )

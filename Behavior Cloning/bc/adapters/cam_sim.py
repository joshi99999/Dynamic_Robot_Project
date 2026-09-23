"""Simulierte Kamera: Platzhalter-Frames mit Fehlerinjektion (AP 0.5/0.10).

Liefert deterministische, unterscheidbare Testbilder (Framezaehler als
Muster einkodiert) und kann gezielt gestoert werden: Latenz, Jitter,
Frame-Drops und veraltete Frames. Damit werden Synchronisation (AP 1.3)
und Episoden-Verwerfregeln (AP 5.2) hardwarefrei geprueft.

Bewusste Grenze (AP 0.10): Die Bilder haben KEINEN Bezug zur Roboterpose --
ArUco-Rektifizierung und die inhaltliche Trainingskette sind damit nicht
abgedeckt und stehen auf der Hardware-Abnahmeliste (AP 0.6).
"""

from dataclasses import dataclass

import numpy as np

from ..clock import RealClock
from ..ports import CameraError, CameraPort, Frame


@dataclass
class CameraFaultProfile:
    """Konfigurierbare Stoerungen je ``read()``-Aufruf."""

    #: Feste Latenz pro Abgriff (Sekunden, ueber die injizierte Uhr).
    latency_s: float = 0.0
    #: Zusaetzlicher gleichverteilter Jitter [0, jitter_s].
    jitter_s: float = 0.0
    #: Wahrscheinlichkeit, dass ein Abgriff fehlschlaegt (CameraError).
    drop_rate: float = 0.0
    #: Zeitstempel haengt um diesen Betrag hinter der Uhr (veralteter Frame).
    stale_s: float = 0.0


#: Bildmuster: "counter" kodiert den Framezaehler (Tests), "static" ist in
#: jedem Frame gleich (Aufzeichnung/Inferenz mit Platzhalterbildern).
PATTERNS = ("counter", "static")


class SimCamera(CameraPort):
    """Platzhalter-Kamera fuer hardwarefreie Tests.

    ``pattern="static"`` fuer alles, woraus eine Policy lernt: der
    Framezaehler im Muster "counter" ist eine UHR im Bild. Befund
    Durchstich 2026-09-17: die Policy las ihn als Zeitsignal ab (AP 2.6,
    Anti-Pattern) -- Fahrt 0 gelang, ab Fahrt 1 (Zaehler weitergelaufen)
    fuhr sie direkt ans Bahnende.
    """

    def __init__(self, cfg, clock=None, faults=None, seed=0, pattern="counter"):
        super().__init__(cfg)
        if pattern not in PATTERNS:
            raise ValueError("pattern muss eines von %s sein" % (PATTERNS,))
        self.pattern = pattern
        self._clock = clock if clock is not None else RealClock()
        self._faults = faults if faults is not None else CameraFaultProfile()
        self._rng = np.random.default_rng(seed)

    def open(self):
        self._open = True
        return self

    def read(self):
        if not self._open:
            raise CameraError("Kamera '%s' ist nicht geoeffnet" % self.name)

        f = self._faults
        if f.latency_s > 0 or f.jitter_s > 0:
            delay = f.latency_s + (self._rng.uniform(0, f.jitter_s) if f.jitter_s > 0 else 0.0)
            self._clock.sleep(delay)
        if f.drop_rate > 0 and self._rng.random() < f.drop_rate:
            raise CameraError(
                "Kamera '%s' lieferte kein Bild (simulierter Frame-Drop)" % self.name
            )

        index = self._next_index()
        image = self._render(index)
        t = self._clock.now() - f.stale_s
        return Frame(image, t, index, self.name)

    def _render(self, index):
        """Deterministisches Testbild: Gradient + Framezaehler-Streifen.

        Der Zaehler ist in den ersten Pixelzeilen binaer einkodiert, damit
        Tests einzelne Frames eindeutig identifizieren koennen.
        """
        h, w = self.cfg.height, self.cfg.width
        image = np.zeros((h, w, 3), dtype=np.uint8)
        image[:, :, 0] = np.linspace(0, 255, w, dtype=np.uint8)[None, :]
        image[:, :, 1] = np.linspace(0, 255, h, dtype=np.uint8)[:, None]
        if self.pattern == "static":
            return image
        for bit in range(16):
            if (index >> bit) & 1:
                x0 = bit * (w // 16)
                image[0:8, x0 : x0 + (w // 16), 2] = 255
        return image

    @staticmethod
    def decode_index(image):
        """Liest den einkodierten Framezaehler wieder aus (fuer Tests)."""
        h, w, _ = image.shape
        index = 0
        for bit in range(16):
            x0 = bit * (w // 16)
            if image[0:8, x0 : x0 + (w // 16), 2].mean() > 127:
                index |= 1 << bit
        return index

    def close(self):
        self._open = False

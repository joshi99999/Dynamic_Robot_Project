"""Livebild waehrend einer laufenden Aufzeichnung (AP 1.2).

Die Bedienoberflaeche startet ``apps/record.py`` als Unterprozess. Damit
gehoeren die Kameras waehrend der Aufnahme dem Unterprozess -- eine Kamera
laesst sich nicht zweimal oeffnen, die GUI kann also nicht nebenher
mitschauen. Der uebliche Weg ist deshalb: Vorschau beim EINRICHTEN in der
GUI, waehrend der Aufnahme nur Zahlen.

Wer das Bild trotzdem sehen will (Wunsch Anwender, 2026-09-29: "bei Bedarf
einschaltbar, sonst aus"), schaltet dieses Modul dazu. Es legt in grossen
Abstaenden ein zusammengesetztes JPEG ab, das die GUI anzeigt.

AUS GUTEM GRUND STANDARDMAESSIG AUS: Die Aufzeichnungsschleife hat ein
Budget von 1/15 s je Takt, und jede zusaetzliche Arbeit darin gefaehrdet
genau die Datenqualitaet, um die es geht. Deshalb:

* Voreinstellung 5 Hz statt 15 -- nur jeder dritte Takt kostet ueberhaupt
  etwas.
* Verkleinern vor dem Kodieren, niedrige JPEG-Qualitaet.
* **Ein Fehler beim Schreiben bricht die Aufnahme nie ab.** Ein Livebild
  ist eine Bequemlichkeit; die Episode ist die Arbeit.
* Geschrieben wird ueber eine Nachbardatei und ``os.replace`` -- so sieht
  der Leser immer ein vollstaendiges Bild und nie ein halb geschriebenes.
"""

import logging
import os
from pathlib import Path

log = logging.getLogger(__name__)

#: Voreinstellung: wie oft das Bild hoechstens neu geschrieben wird.
DEFAULT_RATE_HZ = 5.0
#: Hoehe des zusammengesetzten Bildes in Pixeln.
DEFAULT_HEIGHT = 240
#: JPEG-Qualitaet. Es geht ums Hinsehen, nicht um Archivierung.
DEFAULT_QUALITY = 70


class PreviewWriter(object):
    """Schreibt das zuletzt aufgenommene Bild in Abstaenden als JPEG.

    ``clock`` ist ein ClockPort (injizierbar wie ueberall sonst), damit sich
    die Ratenbegrenzung ohne Echtzeit testen laesst.
    """

    def __init__(self, path, clock, rate_hz=DEFAULT_RATE_HZ,
                 height=DEFAULT_HEIGHT, quality=DEFAULT_QUALITY):
        self.path = Path(path)
        self.clock = clock
        self.period_s = 1.0 / float(rate_hz) if rate_hz else 0.0
        self.height = int(height)
        self.quality = int(quality)
        self._next_at = None
        self.written = 0
        self.failures = 0

    def due(self):
        """Ist wieder ein Bild faellig? Billig genug fuer jeden Takt."""
        if self.period_s <= 0:
            return False
        now = self.clock.now()
        if self._next_at is None or now >= self._next_at:
            self._next_at = now + self.period_s
            return True
        return False

    def offer(self, frames, overlay=None):
        """Bilder anbieten. Schreibt nur, wenn faellig; schluckt jeden Fehler.

        ``frames``: ``{Name: Frame oder None}`` wie im Recorder.
        ``overlay``: Zeile, die ins Bild geschrieben wird (Takt, Episode).
        """
        if not self.due():
            return False
        try:
            image = compose(frames, self.height, overlay)
            if image is None:
                return False
            self._write(image)
            self.written += 1
            return True
        except Exception as exc:
            # Eine Aufnahme darf nie an der Vorschau scheitern.
            self.failures += 1
            if self.failures <= 3:
                log.warning("Vorschau nicht geschrieben: %s: %s",
                            type(exc).__name__, exc)
            return False

    def _write(self, image):
        import cv2

        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".part")
        ok, buffer = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            raise RuntimeError("cv2.imencode hat kein JPEG geliefert")
        temporary.write_bytes(buffer.tobytes())
        # Atomar ersetzen: der Leser sieht nie ein halb geschriebenes Bild.
        os.replace(str(temporary), str(self.path))

    def close(self):
        try:
            self.path.unlink()
        except OSError:
            pass


def compose(frames, height=DEFAULT_HEIGHT, overlay=None):
    """Bilder auf gleiche Hoehe skalieren und nebeneinanderlegen.

    Reihenfolge ist die des uebergebenen Mappings -- also die der Kameras
    im Datensatz. Fehlt ein Bild, kommt ein schwarzes Feld mit dem Namen
    hinein: Dass eine Kamera gerade nichts liefert, ist die wichtigste
    Information, die eine Vorschau geben kann.
    """
    import cv2
    import numpy as np

    tiles = []
    for name, frame in frames.items():
        if frame is None or getattr(frame, "image", None) is None:
            tile = np.zeros((height, int(height * 4 / 3), 3), dtype=np.uint8)
            cv2.putText(tile, "%s: kein Bild" % name, (8, height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 220), 1, cv2.LINE_AA)
        else:
            image = frame.image
            if image.ndim == 2:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            scale = float(height) / image.shape[0]
            tile = cv2.resize(image, (max(1, int(image.shape[1] * scale)), height),
                              interpolation=cv2.INTER_AREA)
            cv2.putText(tile, name, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    if not tiles:
        return None
    board = np.hstack(tiles)
    if overlay:
        # Dunkler Balken darunter: gruene Schrift auf einem gruenen
        # Platzhalterbild liest sich sonst nicht.
        cv2.rectangle(board, (0, board.shape[0] - 24),
                      (board.shape[1], board.shape[0]), (0, 0, 0), -1)
        cv2.putText(board, str(overlay), (8, board.shape[0] - 7),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return board

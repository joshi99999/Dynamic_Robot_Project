"""Frame-Bereitstellung fuer den Recorder (AP 1.3).

Zwei austauschbare Strategien hinter derselben ``latest()``-Schnittstelle:

* :class:`ThreadedCapture` -- echter Betrieb: Capture laeuft frei in einem
  Hintergrund-Thread und puffert stets den juengsten Frame. Genau die
  Entkopplung, die AP 1.3 fordert -- der Recorder greift zur Zielrate den
  aktuellsten Wert ab, statt sich von der Kamerarate takten zu lassen.
* :class:`DirectCapture` -- Tests/Simulation: liest synchron beim Abruf.
  Deterministisch und mit SimClock beliebig schnell.
"""

import threading
import time

from .ports import CameraError


class DirectCapture(object):
    """Synchron: jeder ``latest()``-Aufruf liest ein frisches Bild."""

    def __init__(self, camera):
        self.camera = camera
        self._latest = None
        self._error = None
        self._frames_captured = 0

    @property
    def name(self):
        return self.camera.name

    def start(self):
        if not self.camera.is_open:
            self.camera.open()
        return self

    def latest(self):
        try:
            self._latest = self.camera.read()
            self._frames_captured += 1
            self._error = None
        except CameraError as exc:
            # Frame-Drop: letzter gueltiger Frame bleibt stehen (wird ueber
            # seinen alten Zeitstempel vom Latenzbudget erkannt, AP 1.3).
            self._error = exc
        return self._latest

    @property
    def error(self):
        return self._error

    @property
    def frames_captured(self):
        return self._frames_captured

    def stop(self):
        self.camera.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False


class ThreadedCapture(object):
    """Haelt im Hintergrund immer den juengsten Frame bereit."""

    def __init__(self, camera):
        self.camera = camera
        self._latest = None
        self._lock = threading.Lock()
        self._thread = None
        self._running = threading.Event()
        self._error = None
        self._frames_captured = 0

    @property
    def name(self):
        return self.camera.name

    def start(self):
        if not self.camera.is_open:
            self.camera.open()
        self._running.set()
        self._thread = threading.Thread(
            target=self._loop, name="cam-%s" % self.camera.name, daemon=True
        )
        self._thread.start()
        return self

    def _loop(self):
        while self._running.is_set():
            try:
                frame = self.camera.read()
            except Exception as exc:
                self._error = exc
                time.sleep(0.05)
                continue
            with self._lock:
                self._latest = frame
                self._frames_captured += 1

    def latest(self):
        """Juengster Frame oder None, solange noch keiner eingetroffen ist."""
        with self._lock:
            return self._latest

    def wait_for_frame(self, timeout=5.0):
        """Blockiert, bis der erste Frame da ist. Wirft bei Timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self.latest()
            if frame is not None:
                return frame
            if self._error is not None:
                raise CameraError(
                    "Kamera '%s' meldet Fehler: %s" % (self.name, self._error)
                )
            time.sleep(0.01)
        raise CameraError(
            "Kamera '%s' lieferte innerhalb %.1f s keinen Frame"
            % (self.name, timeout)
        )

    @property
    def error(self):
        return self._error

    @property
    def frames_captured(self):
        with self._lock:
            return self._frames_captured

    def stop(self):
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.camera.close()

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False


def start_all(cameras, threaded=True):
    """Startet Captures fuer alle Kameras.

    Schlaegt eine fehl, werden die bereits gestarteten wieder gestoppt --
    sonst bleiben Geraete belegt und der naechste Start scheitert.
    """
    wrapper = ThreadedCapture if threaded else DirectCapture
    started = []
    try:
        for cam in cameras:
            cap = wrapper(cam)
            cap.start()
            started.append(cap)
        return started
    except Exception:
        for cap in started:
            try:
                cap.stop()
            except Exception:
                pass
        raise

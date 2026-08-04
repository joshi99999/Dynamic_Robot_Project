"""Kamera-Abstraktionsschicht (AP 1.1).

Die beiden Kameras werden voellig unterschiedlich angebunden:

* **Wrist (Daheng VEN-161-61U3C)** ist eine USB3-Vision-/GenICam-Kamera und
  meldet sich NICHT als UVC-Geraet. ``cv2.VideoCapture`` kann sie nicht
  oeffnen. Zugriff ueber das Daheng Galaxy SDK mit ``gxipy``.
* **Szene (USB-Webcam)** ist eine normale UVC-Kamera ueber OpenCV.

Beide liegen hinter derselben Schnittstelle ``read() -> Frame``, damit
Recorder und Inferenzschleife den Kameratyp nicht kennen muessen. Ab dem
Frame-Abgriff ist alles ein NumPy-Array wie gewohnt.

HINWEIS: Der Daheng-Pfad ist gegen die gxipy-API geschrieben, aber noch
nicht an der realen Kamera getestet.
"""

import threading
import time

import numpy as np

from . import config


class CameraError(RuntimeError):
    pass


class Frame(object):
    """Ein Bild mit Zeitstempel.

    ``timestamp`` ist bewusst die Host-Zeit unmittelbar nach dem Abgriff
    (``time.time()``), damit alle Quellen dieselbe Zeitbasis haben wie der
    Roboterzustand. Die geraeteinterne Kamerazeit waere praeziser, muesste
    aber erst auf die Host-Uhr abgebildet werden.
    """

    __slots__ = ("image", "timestamp", "index", "source")

    def __init__(self, image, timestamp, index, source):
        self.image = image
        self.timestamp = timestamp
        self.index = index
        self.source = source

    @property
    def shape(self):
        return self.image.shape

    def __repr__(self):
        return "Frame(%s, #%d, %s, t=%.3f)" % (
            self.source,
            self.index,
            self.image.shape,
            self.timestamp,
        )


class BaseCamera(object):
    """Gemeinsame Schnittstelle beider Kameratypen."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.name = cfg.name
        self._index = 0
        self._open = False

    def open(self):
        raise NotImplementedError

    def read(self):
        """Liefert den naechsten :class:`Frame` (RGB, uint8)."""
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

    @property
    def is_open(self):
        return self._open

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def _next_index(self):
        i = self._index
        self._index += 1
        return i


class UvcCamera(BaseCamera):
    """Standard-USB-Webcam ueber OpenCV."""

    def __init__(self, cfg):
        super().__init__(cfg)
        self._cap = None

    def open(self):
        import cv2

        device = self.cfg.device if self.cfg.device is not None else 0
        # CAP_DSHOW vermeidet unter Windows die langsame MSMF-Initialisierung
        cap = cv2.VideoCapture(device, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            raise CameraError(
                "UVC-Kamera '%s' (Index %s) laesst sich nicht oeffnen. "
                "Index pruefen bzw. ob die Kamera von einer anderen "
                "Anwendung belegt ist." % (self.name, device)
            )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
        if self.cfg.fps:
            cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
        # Kleiner Puffer -> weniger veraltete Frames (AP 1.3)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        self._cap = cap
        self._open = True
        return self

    def read(self):
        import cv2

        if not self._open:
            raise CameraError("Kamera '%s' ist nicht geoeffnet" % self.name)
        ok, bgr = self._cap.read()
        t = time.time()
        if not ok or bgr is None:
            raise CameraError("Kamera '%s' lieferte kein Bild" % self.name)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return Frame(rgb, t, self._next_index(), self.name)

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._open = False


class DahengCamera(BaseCamera):
    """Daheng VEN-161-61U3C ueber das Galaxy SDK (``gxipy``).

    Aufloesung wird kameraseitig gesetzt (ROI), statt volle 1440x1080 zu
    uebertragen und spaeter zu skalieren -- das spart USB-Bandbreite und
    CPU-Last (AP 1.1).
    """

    def __init__(self, cfg):
        super().__init__(cfg)
        self._device_manager = None
        self._cam = None
        self._gx = None

    def open(self):
        try:
            import gxipy as gx
        except ImportError as exc:
            raise CameraError(
                "gxipy ist nicht installiert. Die Daheng-Kamera ist eine "
                "USB3-Vision-/GenICam-Kamera und NICHT ueber cv2.VideoCapture "
                "erreichbar. Daheng Galaxy SDK installieren und daraus das "
                "Python-Paket gxipy einrichten."
            ) from exc

        self._gx = gx
        self._device_manager = gx.DeviceManager()
        dev_count, dev_info = self._device_manager.update_device_list()
        if dev_count == 0:
            raise CameraError(
                "Keine Daheng-Kamera gefunden. USB3-Verbindung und Treiber "
                "pruefen (Galaxy Viewer als Gegentest)."
            )

        if self.cfg.device:
            self._cam = self._device_manager.open_device_by_sn(str(self.cfg.device))
        else:
            self._cam = self._device_manager.open_device_by_index(1)

        self._configure(gx)
        self._cam.stream_on()
        self._open = True
        return self

    def _configure(self, gx):
        cam = self._cam

        # Freilauf statt Trigger -- getriggerte Synchronisation waere nur
        # sinnvoll, wenn auch die Szenenkamera triggerfaehig ist (AP 1.1).
        _try(lambda: cam.TriggerMode.set(gx.GxSwitchEntry.OFF))

        # ROI kameraseitig setzen. Reihenfolge: erst Offsets auf 0, dann
        # Groesse, sonst kollidieren die Wertebereiche.
        _try(lambda: cam.OffsetX.set(0))
        _try(lambda: cam.OffsetY.set(0))
        _try(lambda: cam.Width.set(int(self.cfg.width)))
        _try(lambda: cam.Height.set(int(self.cfg.height)))

        if self.cfg.exposure_us is None:
            _try(lambda: cam.ExposureAuto.set(gx.GxAutoEntry.CONTINUOUS))
        else:
            _try(lambda: cam.ExposureAuto.set(gx.GxAutoEntry.OFF))
            _try(lambda: cam.ExposureTime.set(float(self.cfg.exposure_us)))

        if self.cfg.gain_db is None:
            _try(lambda: cam.GainAuto.set(gx.GxAutoEntry.CONTINUOUS))
        else:
            _try(lambda: cam.GainAuto.set(gx.GxAutoEntry.OFF))
            _try(lambda: cam.Gain.set(float(self.cfg.gain_db)))

        # Farbkamera: Weissabgleich einmalig automatisch bestimmen lassen.
        _try(lambda: cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS))

    def read(self):
        if not self._open:
            raise CameraError("Kamera '%s' ist nicht geoeffnet" % self.name)

        raw = self._cam.data_stream[0].get_image()
        t = time.time()
        if raw is None:
            raise CameraError("Daheng-Kamera lieferte kein Bild (Timeout)")
        if raw.get_status() != self._gx.GxFrameStatusList.SUCCESS:
            raise CameraError(
                "Daheng-Frame unvollstaendig (Status %s)" % raw.get_status()
            )

        # Bayer RG8/RG10 -> RGB. Das SDK uebernimmt das Debayering; bei
        # Mono-Kameras gibt convert("RGB") None zurueck.
        rgb_image = raw.convert("RGB")
        if rgb_image is None:
            arr = raw.get_numpy_array()
            if arr is None:
                raise CameraError("Konnte Daheng-Frame nicht konvertieren")
            arr = np.stack([arr] * 3, axis=-1)  # Mono -> pseudo-RGB
        else:
            arr = rgb_image.get_numpy_array()
        if arr is None:
            raise CameraError("Daheng-Frame ohne NumPy-Daten")

        return Frame(np.ascontiguousarray(arr), t, self._next_index(), self.name)

    def close(self):
        if self._cam is not None:
            _try(self._cam.stream_off)
            _try(self._cam.close_device)
            self._cam = None
        self._open = False


def _try(fn):
    """Fuehrt eine optionale SDK-Einstellung aus, ohne bei Nichtunterstuetzung
    das Oeffnen der Kamera zu verhindern."""
    try:
        fn()
        return True
    except Exception as exc:  # gxipy wirft je nach Feature unterschiedlich
        print("[cameras] Hinweis: Einstellung nicht anwendbar (%s)" % exc)
        return False


class ThreadedCamera(object):
    """Haelt im Hintergrund immer den juengsten Frame bereit.

    Genau die Entkopplung, die AP 1.3 fordert: Capture laeuft frei, der
    Recorder greift zur Zielrate den jeweils aktuellsten Wert ab, statt sich
    von der Kamerarate takten zu lassen.
    """

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


_BACKENDS = {"daheng": DahengCamera, "uvc": UvcCamera}


def open_camera(cfg, threaded=True):
    """Fabrikfunktion: erzeugt die passende Kamera zur Konfiguration."""
    backend = _BACKENDS.get(cfg.backend)
    if backend is None:
        raise CameraError(
            "Unbekanntes Kamera-Backend '%s' (bekannt: %s)"
            % (cfg.backend, ", ".join(sorted(_BACKENDS)))
        )
    cam = backend(cfg)
    if threaded:
        return ThreadedCamera(cam)
    cam.open()
    return cam


def open_all(configs=config.CAMERAS, threaded=True):
    """Oeffnet alle konfigurierten Kameras.

    Schlaegt eine fehl, werden die bereits geoeffneten wieder geschlossen --
    sonst bleiben Geraete belegt und der naechste Start scheitert.
    """
    opened = []
    try:
        for cfg in configs:
            cam = open_camera(cfg, threaded=threaded)
            if threaded:
                cam.start()
            opened.append(cam)
        return opened
    except Exception:
        for cam in opened:
            try:
                cam.stop() if threaded else cam.close()
            except Exception:
                pass
        raise

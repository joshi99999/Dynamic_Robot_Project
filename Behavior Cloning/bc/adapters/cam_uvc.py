"""Szenen-/Top-View-Kamera: Standard-USB-Webcam ueber OpenCV (AP 1.1)."""

import time

from ..ports import CameraError, CameraPort, Frame


class UvcCamera(CameraPort):
    """Standard-USB-Webcam ueber ``cv2.VideoCapture``."""

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

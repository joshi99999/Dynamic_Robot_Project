"""Wrist-Kamera: Daheng VEN-161-61U3C ueber das Galaxy SDK (AP 1.1).

Die Kamera ist eine USB3-Vision-/GenICam-Kamera und meldet sich NICHT als
UVC-Geraet -- ``cv2.VideoCapture`` kann sie nicht oeffnen. Zugriff ueber
``gxipy`` aus dem Daheng Galaxy SDK.

HINWEIS: Gegen die gxipy-API geschrieben, aber noch nicht an der realen
Kamera getestet -- Abnahme ueber die Contract-Tests am Geraet
(``pytest --camera=daheng``, siehe AP 0.6 Punkt 7).
"""

import time

import numpy as np

from ..ports import CameraError, CameraPort, Frame


class DahengCamera(CameraPort):
    """Daheng VEN-161-61U3C ueber ``gxipy``.

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

        # Datenreduktion kameraseitig: Binning statt ROI-Crop.
        # WICHTIG beim Fisheye (AP 1.1): Width/Height sind bei GenICam ein
        # ROI-AUSSCHNITT, keine Skalierung -- ein kleiner ROI verkleinert
        # also das Sichtfeld. Genau das Sichtfeld ist beim Fisheye aber der
        # Grund fuer die Objektivwahl. Binning halbiert die Datenmenge,
        # OHNE Sichtfeld zu verlieren (und verbessert den Rauschabstand).
        # Muss VOR dem ROI gesetzt werden, weil es dessen Grenzen aendert.
        if self.cfg.binning and self.cfg.binning > 1:
            _try(lambda: cam.BinningHorizontal.set(int(self.cfg.binning)))
            _try(lambda: cam.BinningVertical.set(int(self.cfg.binning)))

        self._configure_roi()

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

        # Farbkamera: Weissabgleich automatisch nachfuehren lassen.
        _try(lambda: cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS))

    def _configure_roi(self):
        """Setzt den ROI -- und zwar ZENTRIERT.

        ``width``/``height`` = None bedeutet "voller Sensor" (nach Binning)
        und ist beim Fisheye der Normalfall. Wird doch ein kleinerer ROI
        gewuenscht, muss er mittig liegen: bei OffsetX/Y = 0 schneidet man
        die linke obere Sensorecke aus und damit beim Fisheye das optische
        Zentrum weg.
        """
        cam = self._cam
        if not self.cfg.width or not self.cfg.height:
            return  # voller Sensor -- nichts zu tun

        # Offsets zuerst auf 0, sonst kollidieren die Wertebereiche
        _try(lambda: cam.OffsetX.set(0))
        _try(lambda: cam.OffsetY.set(0))
        if not _try(lambda: cam.Width.set(int(self.cfg.width))):
            return
        if not _try(lambda: cam.Height.set(int(self.cfg.height))):
            return

        # Nach dem Setzen der Groesse ist OffsetX.max = Sensor - Width,
        # die Haelfte davon zentriert den Ausschnitt.
        try:
            range_x = cam.OffsetX.get_range()
            range_y = cam.OffsetY.get_range()
            step_x = int(range_x.get("inc", 1)) or 1
            step_y = int(range_y.get("inc", 1)) or 1
            offset_x = (int(range_x["max"]) // 2 // step_x) * step_x
            offset_y = (int(range_y["max"]) // 2 // step_y) * step_y
        except Exception as exc:
            print(
                "[cam_daheng] WARNUNG: ROI laesst sich nicht zentrieren (%s) -- "
                "Bild zeigt die Sensorecke, nicht die Mitte!" % exc
            )
            return
        _try(lambda: cam.OffsetX.set(offset_x))
        _try(lambda: cam.OffsetY.set(offset_y))

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
        print("[cam_daheng] Hinweis: Einstellung nicht anwendbar (%s)" % exc)
        return False

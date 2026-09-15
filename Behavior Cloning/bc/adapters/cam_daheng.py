"""Wrist-Kamera: Daheng VEN-161-61U3C ueber das Galaxy SDK (AP 1.1).

Die Kamera ist eine USB3-Vision-/GenICam-Kamera und meldet sich NICHT als
UVC-Geraet -- ``cv2.VideoCapture`` kann sie nicht oeffnen. Zugriff ueber
``gxipy`` aus dem Daheng Galaxy SDK.

HINWEIS: Gegen die gxipy-API geschrieben, aber noch nicht an der realen
Kamera getestet -- Abnahme ueber die Contract-Tests am Geraet
(``pytest --camera=daheng``, siehe AP 0.6 Punkt 7).
"""

import os
import sys

import numpy as np

from ..clock import host_time
from ..ports import CameraError, CameraPort, Frame

#: Standardpfade, unter denen das Galaxy SDK sein Python-Paket ablegt.
#: Daheng liefert ``gxipy`` NICHT als installierbares Paket aus (kein
#: setup.py, kein PyPI) -- laut SDK-README soll man den Ordner neben das
#: eigene Skript kopieren. Statt eine Kopie ins Repo zu legen (Vendor-Code,
#: veraltet still bei SDK-Updates), wird der SDK-Pfad hier zur Laufzeit
#: gesucht. Eigener Pfad ueberschreibbar via Umgebungsvariable
#: GALAXY_SDK_PYTHON.
_SDK_PYTHON_HINTS = (
    r"C:\Program Files\Daheng Imaging\GalaxySDK\Development\Samples\Python",
    r"C:\Program Files (x86)\Daheng Imaging\GalaxySDK\Development\Samples\Python",
)


def _inject_machine_environment():
    """Holt die vom Galaxy-Installer gesetzten Systemvariablen nach.

    Windows vererbt Machine-Umgebungsvariablen nur an NEU gestartete
    Prozesse. Wer seine Shell offen hatte, als das SDK installiert wurde,
    laeuft sonst in ``KeyError: 'GALAXY_GENICAM_ROOT'`` bzw. ``Cannot find
    GxIAPI.dll`` -- obwohl alles korrekt installiert ist. Das ist reine
    Umgebungsvererbung, kein Installationsfehler, kostet aber erfahrungs-
    gemaess viel Suchzeit. Deshalb wird die Machine-Umgebung hier direkt
    aus der Registry nachgezogen.
    """
    if os.name != "nt":
        return
    try:
        import winreg
    except ImportError:
        return

    key_path = r"SYSTEM\CurrentControlSet\Control\Session Manager\Environment"
    machine_path = ""
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_path) as key:
            index = 0
            while True:
                try:
                    name, value, _kind = winreg.EnumValue(key, index)
                except OSError:
                    break
                index += 1
                if name.upper() == "PATH":
                    machine_path = str(value)
                elif name.upper().startswith(("GALAXY", "GENICAM")):
                    os.environ.setdefault(name, str(value))
    except OSError:
        return

    # Verzeichnisse mit GxIAPI.dll und den GenICam-Bibliotheken nachtragen
    for entry in machine_path.split(os.pathsep):
        entry = os.path.expandvars(entry.strip())
        if not entry or not os.path.isdir(entry):
            continue
        if "Daheng" not in entry and "Galaxy" not in entry:
            continue
        if entry not in os.environ.get("PATH", ""):
            os.environ["PATH"] = entry + os.pathsep + os.environ.get("PATH", "")
        try:
            os.add_dll_directory(entry)
        except (AttributeError, OSError):
            pass


def _purge_gxipy_modules():
    """Entfernt halb importierte gxipy-Module aus sys.modules.

    Noetig, weil ein fehlgeschlagener Import Teilmodule zuruecklaesst und
    ein erneuter Versuch sonst die kaputte Version wiederverwendet.
    """
    for name in [n for n in sys.modules if n == "gxipy" or n.startswith("gxipy.")]:
        del sys.modules[name]


def import_gxipy():
    """Importiert ``gxipy``, notfalls ueber den SDK-Installationspfad.

    Wirft :class:`CameraError` mit konkreter Handlungsanweisung, wenn es
    nicht klappt.
    """
    if os.name == "nt" and "GALAXY_GENICAM_ROOT" not in os.environ:
        _inject_machine_environment()

    #: None = bereits im sys.path (regulaer installiert)
    candidates = [None]
    env_path = os.environ.get("GALAXY_SDK_PYTHON")
    if env_path:
        candidates.append(env_path)
    candidates.extend(_SDK_PYTHON_HINTS)

    errors = []
    for path in candidates:
        if path is not None:
            if not os.path.isdir(os.path.join(path, "gxipy")):
                continue
            if path not in sys.path:
                sys.path.append(path)
        _purge_gxipy_modules()
        try:
            import gxipy

            return gxipy
        except Exception as exc:
            errors.append("  %s\n    -> %s: %s" % (path or "sys.path", type(exc).__name__, exc))

    _purge_gxipy_modules()
    raise CameraError(
        "gxipy laesst sich nicht importieren. Die Daheng-Kamera ist eine "
        "USB3-Vision-/GenICam-Kamera und NICHT ueber cv2.VideoCapture "
        "erreichbar.\nVersuche:\n%s\n\n"
        "Haeufige Ursachen:\n"
        "  * Galaxy SDK nicht installiert -> installieren.\n"
        "  * SDK an ungewoehnlichem Ort -> Ordner, der 'gxipy' enthaelt, in "
        "der Umgebungsvariable GALAXY_SDK_PYTHON setzen.\n"
        "  * 'GALAXY_GENICAM_ROOT' / 'GxIAPI.dll' fehlt -> Shell nach der "
        "SDK-Installation neu starten (Windows vererbt Systemvariablen nur "
        "an neue Prozesse)." % "\n".join(errors)
    )


class DahengCamera(CameraPort):
    """Daheng-Kamera (VEN-161-61U3C) ueber ``gxipy``.

    Geraeteauswahl ueber ``cfg.device`` = Seriennummer. Bei mehreren
    angeschlossenen Daheng-Kameras ist die Seriennummer PFLICHT: die
    Reihenfolge der Geraeteindizes ist nicht stabil (haengt an
    Enumerationsreihenfolge, USB-Port und Einschaltzeitpunkt). Waeren die
    beiden Kameras vertauscht, wuerden Wrist- und Szenenbild im Datensatz
    vertauscht -- ein Fehler, der im Training nicht auffaellt.
    """

    def __init__(self, cfg):
        super().__init__(cfg)
        self._device_manager = None
        self._cam = None
        self._gx = None

    def open(self):
        gx = import_gxipy()
        self._gx = gx
        self._device_manager = gx.DeviceManager()
        dev_count, dev_info = self._device_manager.update_device_list()
        if dev_count == 0:
            raise CameraError(
                "Keine Daheng-Kamera gefunden. USB3-Verbindung und Treiber "
                "pruefen (Galaxy Viewer als Gegentest)."
            )

        if self.cfg.device:
            serial = str(self.cfg.device)
            known = [str(i.get("sn", "")) for i in dev_info]
            if serial not in known:
                raise CameraError(
                    "Daheng-Kamera mit Seriennummer '%s' (konfiguriert fuer "
                    "'%s') nicht gefunden. Angeschlossen: %s"
                    % (serial, self.name, ", ".join(known) or "keine")
                )
            self._cam = self._device_manager.open_device_by_sn(serial)
        elif dev_count > 1:
            raise CameraError(
                "%d Daheng-Kameras angeschlossen, aber fuer '%s' ist keine "
                "Seriennummer konfiguriert. Bei mehreren Geraeten ist die "
                "Index-Reihenfolge nicht stabil -- Wrist- und Szenenbild "
                "koennten vertauscht werden.\nAngeschlossen: %s\nAbhilfe: "
                "'device' in der jeweiligen CameraConfig auf die Seriennummer "
                "setzen (ermitteln mit 'python tools/check_cameras.py --list')."
                % (
                    dev_count,
                    self.name,
                    ", ".join(
                        "%s (%s)" % (i.get("sn", "?"), i.get("model_name", "?"))
                        for i in dev_info
                    ),
                )
            )
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

        # Weissabgleich: einmalig bestimmen, NICHT laufend nachfuehren.
        # Die VEN-161 kann ohnehin nur Off/Once. Fuer den Datensatz ist das
        # auch die richtige Wahl: ein laufend nachgeregelter Weissabgleich
        # aendert die Farbstatistik, waehrend der Arm faehrt und sich der
        # Bildinhalt aendert -- die Policy saehe dieselbe Szene je nach
        # Blickrichtung unterschiedlich eingefaerbt.
        # ACHTUNG (AP 0.6): "Once" haengt davon ab, was beim Start im Bild
        # war, und schwankt damit zwischen Sessions. Vor der echten
        # Datenaufzeichnung sind feste Ratios zu messen und in der Config
        # zu pinnen (siehe white_balance_ratios).
        if self.cfg.white_balance_ratios:
            _try(lambda: cam.BalanceWhiteAuto.set(gx.GxAutoEntry.OFF))
            for channel, ratio in zip(
                ("Red", "Green", "Blue"), self.cfg.white_balance_ratios
            ):
                _try(lambda c=channel: cam.BalanceRatioSelector.set(c))
                _try(lambda r=ratio: cam.BalanceRatio.set(float(r)))
        else:
            _try(lambda: cam.BalanceWhiteAuto.set(gx.GxAutoEntry.ONCE))

        # Bildrate begrenzen, falls gewuenscht: die Kamera laeuft sonst mit
        # 61 fps frei durch, obwohl nur mit 15 Hz abgetastet wird. Native
        # Rate = frischere Frames (kleineres Alter im Latenzbudget, AP 1.3),
        # gedrosselte Rate = weniger CPU-Last durchs Debayering.
        if self.cfg.fps:
            _try(lambda: cam.AcquisitionFrameRateMode.set(gx.GxSwitchEntry.ON))
            _try(lambda: cam.AcquisitionFrameRate.set(float(self.cfg.fps)))

    def _configure_roi(self):
        """Setzt den ROI -- und zwar ZENTRIERT.

        ``width``/``height`` = None bedeutet "voller Sensor" (nach Binning)
        und ist beim Fisheye der Normalfall. Wird doch ein kleinerer ROI
        gewuenscht, muss er mittig liegen: bei OffsetX/Y = 0 schneidet man
        die linke obere Sensorecke aus und damit beim Fisheye das optische
        Zentrum weg.
        """
        cam = self._cam

        # Offsets zuerst auf 0, sonst kollidieren die Wertebereiche
        _try(lambda: cam.OffsetX.set(0))
        _try(lambda: cam.OffsetY.set(0))

        # "Voller Sensor" muss AKTIV gesetzt werden. Die Kamera speichert
        # ihren ROI persistent -- wer hier nichts setzt, bekommt still den
        # zuletzt konfigurierten Ausschnitt (beobachtet: 640x480 aus einem
        # 1440x1080-Sensor, also ein Eckausschnitt ohne optisches Zentrum).
        width = self.cfg.width
        height = self.cfg.height
        if not width or not height:
            try:
                width = int(cam.WidthMax.get())
                height = int(cam.HeightMax.get())
            except Exception as exc:
                print(
                    "[cam_daheng] WARNUNG: Sensorgroesse nicht lesbar (%s) -- "
                    "ROI bleibt auf dem gespeicherten Wert der Kamera!" % exc
                )
                return

        if not _try(lambda: cam.Width.set(int(width))):
            return
        if not _try(lambda: cam.Height.set(int(height))):
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
        t = host_time()
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

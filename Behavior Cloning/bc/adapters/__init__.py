"""Adapter: konkrete Implementierungen der Ports (AP 0.3).

NUR hier duerfen ``neurapy`` und ``gxipy`` importiert werden -- und auch
hier erst verzoegert beim Verbinden, damit sich alles ohne installierte
SDKs importieren laesst.

Kamera-Backends werden ueber :func:`open_camera` anhand von
``CameraConfig.backend`` aufgeloest ("daheng" | "uvc" | "sim").

Welches GERAET an welchem Platz haengt, ist davon getrennt: :func:`list_devices`
sagt, was angeschlossen ist, und :func:`camera_config_for` baut daraus die
Konfiguration fuer einen Platz ("wrist"/"scene"). Damit ist ein Kamerawechsel
ein Zuordnungswechsel und keine Code-Aenderung -- gebraucht wird das, sobald
die Szenenkamera feststeht (AP 1.1, noch offen) und fuer Tests mit einer
beliebigen Webcam statt der Daheng.
"""

from dataclasses import replace

from .. import config
from ..ports import CameraError


def open_camera(cfg):
    """Fabrikfunktion: erzeugt die passende (ungeoeffnete) Kamera."""
    if cfg.backend == "daheng":
        from .cam_daheng import DahengCamera

        return DahengCamera(cfg)
    if cfg.backend == "uvc":
        from .cam_uvc import UvcCamera

        return UvcCamera(cfg)
    if cfg.backend == "sim":
        from .cam_sim import SimCamera

        return SimCamera(cfg)
    raise CameraError(
        "Unbekanntes Kamera-Backend '%s' (bekannt: daheng, uvc, sim)" % cfg.backend
    )


# --------------------------------------------------------------------------
# Welche Geraete sind angeschlossen, und was haengt wo?
# --------------------------------------------------------------------------

#: Backends, die ein Geraet haben koennen. "sim" steht immer zur Verfuegung.
CAMERA_BACKENDS = ("sim", "uvc", "daheng")

#: Wie viele OpenCV-Indizes probeweise geoeffnet werden. Fuer UVC gibt es
#: keine saubere Enumeration -- man muss aufmachen, um zu wissen, ob da was
#: ist. Jeder Versuch kostet Zeit, deshalb eine kleine Obergrenze.
UVC_PROBE_RANGE = 6


class CameraDevice(object):
    """Ein gefundenes (oder immer verfuegbares) Kameraziel.

    ``key`` ist die Kurzform fuer die Kommandozeile: "sim", "uvc:1",
    "daheng:EBK24100633" -- dieselbe Schreibweise, die ``--camera`` erwartet.
    """

    def __init__(self, backend, device=None, label=None, detail="", available=True):
        self.backend = backend
        self.device = device
        self.label = label or backend
        self.detail = detail
        self.available = available

    @property
    def key(self):
        return self.backend if self.device is None else "%s:%s" % (self.backend, self.device)

    def __repr__(self):
        return "<CameraDevice %s>" % self.key

    def __eq__(self, other):
        return (isinstance(other, CameraDevice)
                and (self.backend, str(self.device)) == (other.backend, str(other.device)))

    def __hash__(self):
        return hash((self.backend, str(self.device)))


def sim_device():
    """Der Platzhalter -- immer da, auch ohne jede Hardware.

    Steht bewusst an erster Stelle jeder Liste: Ohne Kamera muss sich die
    ganze Kette trotzdem durchspielen lassen (AP 0.5), und das Schema des
    Datensatzes ist mit Platzhalterbildern identisch.
    """
    return CameraDevice("sim", None, "Simulation (Platzhalterbild)",
                        "statisches Muster, keine Hardware")


def list_uvc_devices(probe_range=UVC_PROBE_RANGE):
    """UVC-/Webcam-Indizes durchprobieren.

    OpenCV kennt keine Enumeration; ein Index gilt als vorhanden, wenn er
    sich oeffnen laesst. Liefert auch Geraete, die sich oeffnen, aber kein
    Bild liefern -- das ist ein Befund und keine Abwesenheit.
    """
    try:
        import cv2
    except Exception as exc:
        return [], "OpenCV fehlt (%s)" % type(exc).__name__

    # Ein nicht belegter Index ist der Normalfall, kein Befund. OpenCV
    # schreibt dafuer je Versuch eine WARN-Zeile auf stderr -- in einer
    # Oberflaeche ist das nur Rauschen im Terminal.
    logging_api = getattr(getattr(cv2, "utils", None), "logging", None)
    previous = None
    if logging_api is not None:
        previous = logging_api.getLogLevel()
        logging_api.setLogLevel(logging_api.LOG_LEVEL_ERROR)
    try:
        found = _probe_uvc(cv2, probe_range)
    finally:
        if logging_api is not None:
            logging_api.setLogLevel(previous)
    return found, None


def _probe_uvc(cv2, probe_range):
    found = []
    for index in range(probe_range):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        try:
            if not cap.isOpened():
                continue
            ok, frame = cap.read()
            fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
            if ok and frame is not None:
                detail = "%dx%d, gemeldet %.0f fps" % (
                    frame.shape[1], frame.shape[0], fps)
                available = True
            else:
                detail = "geoeffnet, aber kein Bild"
                available = False
            found.append(CameraDevice("uvc", index, "Webcam %d" % index,
                                      detail, available))
        finally:
            cap.release()
    return found


def list_daheng_devices():
    """Daheng-Geraete ueber das Galaxy SDK auflisten.

    Nur hier darf ``gxipy`` beruehrt werden (Schichtenregel AP 0.3), und
    auch hier erst beim Aufruf. Ist das SDK nicht installiert, ist das ein
    Befund mit Klartext -- kein Fehler.
    """
    from .cam_daheng import import_gxipy

    try:
        gx = import_gxipy()
    except CameraError as exc:
        return [], str(exc)
    try:
        manager = gx.DeviceManager()
        count, info_list = manager.update_device_list()
    except Exception as exc:
        return [], "Geraeteliste nicht lesbar: %s: %s" % (type(exc).__name__, exc)
    if not count:
        return [], ("keine Daheng-Kamera gefunden (USB3-Kabel und Treiber "
                    "pruefen, Gegenprobe mit dem Galaxy Viewer)")
    found = []
    for info in info_list:
        serial = info.get("sn") or None
        model = info.get("model_name", "?")
        found.append(CameraDevice(
            "daheng", serial, "Daheng %s" % model,
            "SN %s, %s" % (serial or "?", info.get("vendor_name", "?"))))
    return found, None


def list_devices(uvc=True, daheng=True, probe_range=UVC_PROBE_RANGE):
    """Alles, was sich einem Kameraplatz zuweisen laesst.

    Rueckgabe: ``{"devices": [CameraDevice, ...], "notes": {backend: Text}}``.
    ``notes`` sagt, warum ein Backend nichts beigesteuert hat -- die
    Unterscheidung "kein Geraet" und "kein SDK" ist beim Einrichten der
    entscheidende Unterschied.

    Das Oeffnen der UVC-Indizes dauert; Aufrufer mit Oberflaeche rufen das
    in einem Arbeitsthread auf.
    """
    devices = [sim_device()]
    notes = {}
    if uvc:
        found, note = list_uvc_devices(probe_range)
        devices.extend(found)
        if note:
            notes["uvc"] = note
        elif not found:
            notes["uvc"] = "keine Webcam gefunden (Indizes 0-%d geprueft)" % (probe_range - 1)
    if daheng:
        found, note = list_daheng_devices()
        devices.extend(found)
        if note:
            notes["daheng"] = note
    return {"devices": devices, "notes": notes}


def parse_device(backend, text):
    """Geraeteangabe der Kommandozeile in den Typ bringen, den das Backend braucht.

    UVC adressiert ueber einen ganzzahligen OpenCV-Index, Daheng ueber die
    Seriennummer (Text), "sim" hat kein Geraet.
    """
    if backend == "sim":
        if text:
            raise ValueError("Backend 'sim' hat kein Geraet (angegeben: %r)" % text)
        return None
    if text in (None, ""):
        return None
    if backend == "uvc":
        try:
            return int(text)
        except (TypeError, ValueError):
            raise ValueError("Webcam-Geraet muss ein OpenCV-Index sein, nicht %r" % text)
    return str(text)


def parse_camera_spec(text):
    """``"wrist=uvc:1"`` -> ``("wrist", "uvc", 1)``.

    Schreibweise der Option ``--camera`` in record.py und infer.py, und
    zugleich die, die die Oberflaeche anzeigt -- eine Zuordnung, die man
    liest, soll sich auch eintippen lassen.
    """
    if "=" not in text:
        raise ValueError(
            "Kamerazuordnung '%s' erwartet NAME=BACKEND[:GERAET], z. B. "
            "wrist=uvc:1 oder scene=sim" % text)
    name, _, source = text.partition("=")
    name = name.strip()
    backend, _, device = source.strip().partition(":")
    known = [c.name for c in config.CAMERAS]
    if name not in known:
        raise ValueError("Unbekannter Kameraplatz '%s' (bekannt: %s)"
                         % (name, ", ".join(known)))
    if backend not in CAMERA_BACKENDS:
        raise ValueError("Unbekanntes Backend '%s' (bekannt: %s)"
                         % (backend, ", ".join(CAMERA_BACKENDS)))
    return name, backend, parse_device(backend, device)


def camera_config_for(name, backend, device=None):
    """Konfiguration fuer einen Platz mit einem bestimmten Backend.

    Ausgangspunkt ist die hinterlegte Konfiguration des Platzes, wenn das
    Backend dazu passt -- dann bleiben Belichtung, ROI und Rate erhalten und
    es wird nur das Geraet getauscht. Passt es nicht, wird die vorbereitete
    Konfiguration desselben Backends genommen (z. B. die zweite Daheng fuer
    die Szene) und sonst ein schlichter Standard aufgebaut.
    """
    known = {c.name: c for c in config.CAMERAS}
    if name not in known:
        raise ValueError("Unbekannter Kameraplatz '%s'" % name)

    if backend == "sim":
        for cfg in config.SIM_CAMERAS:
            if cfg.name == name:
                return cfg
        return config.CameraConfig(name=name, backend="sim", fps=30.0)

    base = known[name]
    if base.backend == backend:
        return base if device is None else replace(base, device=device)

    prepared = {
        ("scene", "daheng"): config.SCENE_CAMERA_DAHENG,
        ("wrist", "uvc"): config.WRIST_CAMERA_UVC_STANDIN,
    }.get((name, backend))
    if prepared is not None:
        return prepared if device is None else replace(prepared, device=device)

    if backend == "daheng":
        # Wie die Wrist-Kamera: voller Sensor, feste Belichtung. Rate
        # begrenzt, weil zwei USB3-Kameras sonst die Bandbreite ausreizen.
        return replace(config.WRIST_CAMERA, name=name, device=device, fps=30.0)
    return config.CameraConfig(name=name, backend="uvc", device=device or 0,
                               width=640, height=480, fps=30.0)


def apply_camera_specs(cfgs, specs):
    """Zuordnungen auf eine Konfigurationsliste anwenden.

    ``specs`` sind Texte in der Schreibweise von :func:`parse_camera_spec`
    oder fertige Tripel. Was nicht genannt ist, bleibt wie es war -- eine
    Zuordnung ersetzt einen Platz, nicht die ganze Auswahl.
    """
    if not specs:
        return list(cfgs)
    overrides = {}
    for spec in specs:
        name, backend, device = (parse_camera_spec(spec)
                                 if isinstance(spec, str) else spec)
        overrides[name] = camera_config_for(name, backend, device)
    return [overrides.pop(c.name, c) for c in cfgs] + list(overrides.values())


#: Kamera-Auswahl der Apps (record/infer):
#:   sim         Platzhalterbilder (statisch) fuer beide Kameras
#:   real        config.CAMERAS (Wrist Daheng + Szenenkamera)
#:   wrist-real  echte Wrist-Kamera, Szenenkamera als Platzhalter -- fuer
#:               den Labortest "VM/Sim mit echter Wrist-Kamera", solange die
#:               Szenenkamera nicht entschieden ist
#:   wrist-uvc   USB-Webcam statt Wrist-Kamera, Szene als Platzhalter -- NUR
#:               Test der Kamerakette ohne Daheng (config.WRIST_CAMERA_UVC_STANDIN)
#:   auto        sim beim SimRobot, real am Neura
CAMERA_MODES = ("auto", "sim", "real", "wrist-real", "wrist-uvc")


def resolve_camera_mode(mode, robot_is_sim):
    if mode not in CAMERA_MODES:
        raise ValueError("Kamera-Modus '%s' unbekannt (%s)" % (mode, ", ".join(CAMERA_MODES)))
    if mode == "auto":
        return "sim" if robot_is_sim else "real"
    return mode


def camera_configs(mode, uvc_device=None, specs=None):
    """Konfigurationen je Modus (ohne etwas zu oeffnen).

    ``uvc_device``: OpenCV-Index der Webcam im Modus "wrist-uvc"
    (None = config.WRIST_CAMERA_UVC_STANDIN.device).
    ``specs``: einzelne Plaetze abweichend besetzen ("scene=uvc:1"), siehe
    :func:`apply_camera_specs`. Der Modus ist dann nur noch der Ausgangspunkt.
    """
    if mode == "sim":
        cfgs = list(config.SIM_CAMERAS)
    elif mode == "real":
        cfgs = list(config.CAMERAS)
    elif mode in ("wrist-real", "wrist-uvc"):
        if mode == "wrist-real":
            wrist = config.WRIST_CAMERA
        else:
            wrist = config.WRIST_CAMERA_UVC_STANDIN
            if uvc_device is not None:
                wrist = replace(wrist, device=uvc_device)
        sim = {c.name: c for c in config.SIM_CAMERAS}
        cfgs = [wrist if c.name == "wrist" else sim[c.name] for c in config.CAMERAS]
    else:
        raise ValueError(mode)
    return apply_camera_specs(cfgs, specs)


def start_cameras(mode, clock, seed=0, uvc_device=None, specs=None):
    """Oeffnet die Kameras eines (aufgeloesten) Modus und startet die Captures.

    Echte Kameras laufen im Hintergrund-Thread (ThreadedCapture), Platzhalter
    synchron (DirectCapture) -- ein Thread um eine sofort liefernde
    Sim-Kamera wuerde nur leer drehen. Platzhalter sind STATISCH: ein
    Framezaehler im Bild waere eine Uhr, aus der die Policy die Zeit abliest
    (Befund 2026-09-17, siehe cam_sim.SimCamera).

    Werden echte und Sim-Kameras gemischt, muss ``clock`` die Host-Uhr sein,
    sonst sind die Zeitstempel nicht vergleichbar (AP 1.3).
    """
    from .. import capture
    from ..clock import RealClock
    from .cam_sim import SimCamera

    cfgs = camera_configs(mode, uvc_device=uvc_device, specs=specs)
    if any(c.backend != "sim" for c in cfgs) and not isinstance(clock, RealClock):
        raise ValueError("Echte Kameras brauchen die Host-Uhr (RealClock), nicht %s"
                         % type(clock).__name__)
    started = []
    try:
        for cfg in cfgs:
            if cfg.backend == "sim":
                cap = capture.DirectCapture(SimCamera(cfg, clock=clock, seed=seed, pattern="static"))
            else:
                cap = capture.ThreadedCapture(open_camera(cfg))
            cap.start()
            started.append(cap)
        for cap in started:
            if isinstance(cap, capture.ThreadedCapture):
                cap.wait_for_frame(timeout=10.0)
    except Exception:
        for cap in started:
            try:
                cap.stop()
            except Exception:
                pass
        raise
    return started, cfgs


def open_robot(kind="neura", **kwargs):
    """Fabrikfunktion fuer den Roboter ("neura" | "sim")."""
    if kind == "neura":
        from .neura import NeuraRobot

        return NeuraRobot(**kwargs)
    if kind == "sim":
        from .sim_robot import SimRobot

        return SimRobot(**kwargs)
    raise ValueError("Unbekannter Roboter-Typ '%s' (bekannt: neura, sim)" % kind)

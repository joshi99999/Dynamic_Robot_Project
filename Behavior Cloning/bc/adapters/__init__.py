"""Adapter: konkrete Implementierungen der Ports (AP 0.3).

NUR hier duerfen ``neurapy`` und ``gxipy`` importiert werden -- und auch
hier erst verzoegert beim Verbinden, damit sich alles ohne installierte
SDKs importieren laesst.

Kamera-Backends werden ueber :func:`open_camera` anhand von
``CameraConfig.backend`` aufgeloest ("daheng" | "uvc" | "sim").
"""

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


#: Kamera-Auswahl der Apps (record/infer):
#:   sim         Platzhalterbilder (statisch) fuer beide Kameras
#:   real        config.CAMERAS (Wrist Daheng + Szenenkamera)
#:   wrist-real  echte Wrist-Kamera, Szenenkamera als Platzhalter -- fuer
#:               den Labortest "VM/Sim mit echter Wrist-Kamera", solange die
#:               Szenenkamera nicht entschieden ist
#:   auto        sim beim SimRobot, real am Neura
CAMERA_MODES = ("auto", "sim", "real", "wrist-real")


def resolve_camera_mode(mode, robot_is_sim):
    if mode not in CAMERA_MODES:
        raise ValueError("Kamera-Modus '%s' unbekannt (%s)" % (mode, ", ".join(CAMERA_MODES)))
    if mode == "auto":
        return "sim" if robot_is_sim else "real"
    return mode


def camera_configs(mode):
    """Konfigurationen je Modus (ohne etwas zu oeffnen)."""
    if mode == "sim":
        return list(config.SIM_CAMERAS)
    if mode == "real":
        return list(config.CAMERAS)
    if mode == "wrist-real":
        sim = {c.name: c for c in config.SIM_CAMERAS}
        return [config.WRIST_CAMERA if c.name == "wrist" else sim[c.name] for c in config.CAMERAS]
    raise ValueError(mode)


def start_cameras(mode, clock, seed=0):
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

    cfgs = camera_configs(mode)
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

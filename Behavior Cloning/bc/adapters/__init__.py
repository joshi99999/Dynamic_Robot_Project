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


def open_robot(kind="neura", **kwargs):
    """Fabrikfunktion fuer den Roboter ("neura" | "sim")."""
    if kind == "neura":
        from .neura import NeuraRobot

        return NeuraRobot(**kwargs)
    if kind == "sim":
        from .sim_robot import SimRobot

        return SimRobot(**kwargs)
    raise ValueError("Unbekannter Roboter-Typ '%s' (bekannt: neura, sim)" % kind)

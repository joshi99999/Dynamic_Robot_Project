"""Gemeinsame Fabriken fuer Tests und Contract-Suite (AP 0.5).

Die Contract-Tests laufen gegen JEDE Implementierung eines Ports --
``make_robot("sim")`` heute, ``make_robot("neura")`` am Hardwaretag
(pytest --robot=neura). Diese Fabriken sind die einzige Stelle, die
entscheidet, welche Implementierung geprueft wird.
"""

from dataclasses import replace

import _paths  # noqa: F401

from bc import config
from bc.clock import RealClock, SimClock


def make_clock(kind="sim"):
    return SimClock() if kind == "sim" else RealClock()


#: Freigabe der REALEN Anlage fuer die Contract-Suite -- nur ueber
#: ``run_all.py --robot=neura --real-robot`` (mit Bestaetigung) gesetzt.
ALLOW_REAL = {"robot": False}

#: OpenCV-Index der UVC-Kamera fuer die Contract-Suite, gesetzt ueber
#: ``run_all.py --uvc-device=N`` bzw. ``pytest --uvc-device=N``.
#: ``None`` = Index aus ``config.SCENE_CAMERA`` (0).
#:
#: Noetig, weil der Index NICHT stabil ist: er haengt am Rechner und am
#: OpenCV-Backend. Ohne die Option testet die Suite auf einem Laptop die
#: eingebaute Webcam statt der angeschlossenen Kamera -- und zwar
#: unbemerkt, weil auch die eingebaute Bilder liefert.
UVC_DEVICE = {"device": None}


def make_robot(kind="sim", clock=None, **kwargs):
    if kind == "sim":
        from bc.adapters.sim_robot import SimRobot

        return SimRobot(clock=clock or SimClock(), **kwargs).connect()
    if kind == "neura":
        from bc.adapters.neura import NeuraRobot

        # power_on: servo_j/move_to_joints brauchen einen bestromten Arm.
        # Die Adapter-Sperre verweigert das, solange der Controller nicht
        # is_robot_in_simulation() == True meldet (reale Anlage: allow_real).
        kwargs.setdefault("allow_real", ALLOW_REAL["robot"])
        return NeuraRobot(**kwargs).connect(power_on=True)
    raise ValueError(kind)


def make_camera(kind="sim", name="wrist", clock=None, **kwargs):
    if kind == "sim":
        from bc.adapters.cam_sim import SimCamera

        cfg = config.CameraConfig(name=name, backend="sim")
        return SimCamera(cfg, clock=clock or SimClock(), **kwargs)
    if kind == "uvc":
        from bc.adapters.cam_uvc import UvcCamera

        cfg = config.SCENE_CAMERA
        if UVC_DEVICE["device"] is not None:
            cfg = replace(cfg, device=UVC_DEVICE["device"])
        return UvcCamera(cfg)
    if kind == "daheng":
        from bc.adapters.cam_daheng import DahengCamera

        return DahengCamera(config.WRIST_CAMERA)
    raise ValueError(kind)

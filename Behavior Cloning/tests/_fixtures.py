"""Gemeinsame Fabriken fuer Tests und Contract-Suite (AP 0.5).

Die Contract-Tests laufen gegen JEDE Implementierung eines Ports --
``make_robot("sim")`` heute, ``make_robot("neura")`` am Hardwaretag
(pytest --robot=neura). Diese Fabriken sind die einzige Stelle, die
entscheidet, welche Implementierung geprueft wird.
"""

import _paths  # noqa: F401

from bc import config
from bc.clock import RealClock, SimClock


def make_clock(kind="sim"):
    return SimClock() if kind == "sim" else RealClock()


def make_robot(kind="sim", clock=None, **kwargs):
    if kind == "sim":
        from bc.adapters.sim_robot import SimRobot

        return SimRobot(clock=clock or SimClock(), **kwargs).connect()
    if kind == "neura":
        from bc.adapters.neura import NeuraRobot

        # power_on: servo_j/move_to_joints brauchen einen bestromten Arm.
        # Die Adapter-Sperre verweigert das, solange der Controller nicht
        # is_robot_in_simulation() == True meldet (reale Anlage: allow_real).
        return NeuraRobot(**kwargs).connect(power_on=True)
    raise ValueError(kind)


def make_camera(kind="sim", name="wrist", clock=None, **kwargs):
    if kind == "sim":
        from bc.adapters.cam_sim import SimCamera

        cfg = config.CameraConfig(name=name, backend="sim")
        return SimCamera(cfg, clock=clock or SimClock(), **kwargs)
    if kind == "uvc":
        from bc.adapters.cam_uvc import UvcCamera

        return UvcCamera(config.SCENE_CAMERA)
    if kind == "daheng":
        from bc.adapters.cam_daheng import DahengCamera

        return DahengCamera(config.WRIST_CAMERA)
    raise ValueError(kind)

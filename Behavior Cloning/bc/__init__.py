"""Behavior-Cloning-Pipeline fuer den Neura LARA 5.

Phase 0 (Fundament):
    config          -- zentrale Konstanten, an denen Aufzeichnung und
                       Inferenz gemeinsam haengen
    geometry        -- Posen-/Quaternion-Mathematik (vermeidet den
                       +/-pi-Umschlagpunkt der RPY-Darstellung)
    robot_adapter   -- NeuraPy-Anbindung, State-Abgriff, Greifer, Not-Halt
    kinematics      -- IK/FK mit den vier Absicherungen aus AP 2.4
    collision       -- Quader-Kollisionsmodell (offline + Geofencing)
    cameras         -- Daheng (gxipy) und UVC (OpenCV) hinter einer
                       gemeinsamen Schnittstelle

Der Import von ``neurapy`` und ``gxipy`` erfolgt bewusst erst beim
tatsaechlichen Verbinden, damit sich Geometrie, Kollisionsmodell und
Kamerateile auch ohne Roboter- bzw. SDK-Installation testen lassen.
"""

from . import config, geometry  # noqa: F401

__all__ = [
    "config",
    "geometry",
    "robot_adapter",
    "kinematics",
    "collision",
    "cameras",
]

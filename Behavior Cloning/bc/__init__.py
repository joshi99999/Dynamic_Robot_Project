"""Behavior-Cloning-Pipeline fuer den Neura LARA 5.

Schichtenmodell (AP 0.3 in Requierments/requierments.md):

    Pipeline (hardwarefrei):
        config      -- zentrale Konstanten (Schema, Raten, Grenzen)
        geometry    -- Posen-/Quaternion-Mathematik (+/-pi-Problem)
        ports       -- abstrakte Schnittstellen (Robot/Camera/Teleop/Clock)
        clock       -- RealClock / SimClock (injizierbare Zeit)
        urdf        -- URDF-Parser fuer die Offline-Kinematik
        kinematics  -- IK/FK mit den vier Absicherungen aus AP 2.4
        collision   -- Quader-Kollisionsmodell (offline + Geofencing)
        trajectory  -- Wegpunkte -> zeitparametrierte Soll-Bahn
        noise       -- OU-Rauschen, Trichter, Rejection Sampling
        sync        -- 15-Hz-Taktung und Latenzbudget (AP 1.3)
        capture     -- Threaded/Direct-Frame-Bereitstellung
        recorder    -- Episodenaufzeichnung mit asymmetrischer Paarung
        dataset     -- Datensatz-Schema und Ablage
        rectify     -- ArUco-Homographie + Posen-Ueberwachung (AP 1.4)
        safety      -- Watchdog, Geofence, Not-Halt (AP 4.2)
        policy      -- Policy-Schnittstelle, Action Chunking (Geruest)
        metrics     -- Datensatz-/Evaluationsauswertung

    Adapter (bc.adapters -- NUR hier neurapy/gxipy):
        neura, sim_robot, cam_daheng, cam_uvc, cam_sim,
        teleop_gamepad, teleop_script

Verbindliche Regel (abgesichert durch tests/test_layering.py): Die
Pipeline-Schicht importiert weder ``neurapy`` noch ``gxipy``.
"""

from . import config, geometry  # noqa: F401

__all__ = [
    "adapters",
    "capture",
    "clock",
    "collision",
    "config",
    "dataset",
    "geometry",
    "kinematics",
    "metrics",
    "noise",
    "policy",
    "ports",
    "recorder",
    "rectify",
    "safety",
    "sync",
    "trajectory",
    "urdf",
]

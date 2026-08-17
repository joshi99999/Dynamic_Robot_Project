"""NeuraPy-Adapter fuer den LARA 5 (AP 1.5) -- Implementierung des RobotPort.

Kapselt Verbindung, Lebenszyklus, State-Abgriff, Kinematik-Aufrufe und
Greifer hinter der Port-Schnittstelle. Wird sowohl bei der Datenaufzeichnung
als auch bei der Live-Inferenz benutzt und ist damit die zentrale Bruecke
zwischen KI-Framework und Hardware.

Dieser Adapter ist gegen die NeuraPy-API geschrieben, aber ohne Anlage
nicht abnehmbar -- die Abnahme erfolgt ueber die Contract-Tests
(``pytest --robot=neura``, siehe AP 0.5/0.6).

Import von ``neurapy`` erfolgt bewusst erst beim Verbinden, damit sich das
Modul ohne installierte SDKs importieren laesst.
"""

import threading
import time

import numpy as np

from .. import config, geometry
from ..ports import IKError, NotConnectedError, RobotError, RobotPort, RobotState


def _unwrap_neurapy_error(exc, function_name):
    """Uebersetzt den bekannten NeuraPy-Bug in eine verstaendliche Meldung.

    ``receive_json`` in neurapy/robot.py gibt implizit None zurueck, wenn der
    Server die Verbindung ohne Daten schliesst; ``wrapped_function`` greift
    dann ungeprueft auf ``response["error"]`` zu. Das aeussert sich als
    ``TypeError: 'NoneType' object is not subscriptable`` und bedeutet
    IMMER "Server hat nicht geantwortet", nie "falsche Parameter".
    """
    if isinstance(exc, TypeError) and "not subscriptable" in str(exc):
        return RobotError(
            "Control-Box hat auf '%s' nicht geantwortet (leere Socket-Antwort). "
            "Moegliche Ursachen: Funktion in dieser Controller-Version nicht "
            "verfuegbar, Verbindung abgebrochen, oder serverseitiger Fehler. "
            "Mit adapter.list_methods() pruefen, was der Controller anbietet."
            % function_name
        )
    return None


class NeuraRobot(RobotPort):
    """Schmale, robuste Huelle um ``neurapy.robot.Robot``.

    Thread-Sicherheit: NeuraPy oeffnet pro Aufruf einen eigenen Socket
    (siehe ``generate_function`` in neurapy/robot.py), daher sind Aufrufe aus
    mehreren Threads unproblematisch. ``emergency_stop()`` nimmt bewusst
    keine Locks, damit ein Watchdog nie blockiert.
    """

    def __init__(self, robot=None, override=config.DEFAULT_OVERRIDE):
        self._robot = robot
        self._override = override
        self._gripper_closed = False
        self._servo_active = False
        self._stop_requested = threading.Event()

    # -- Eigenschaften -----------------------------------------------------

    @property
    def dof(self):
        return int(getattr(self.robot, "dof", 6))

    # -- Lebenszyklus ------------------------------------------------------

    def connect(self, power_on=False, ensure_automatic=False):
        """Verbindet mit der Control-Box und initialisiert das Programm."""
        if self._robot is None:
            from neurapy.robot import Robot  # lokal, damit Import ohne HW geht

            self._robot = Robot()

        self._call("init_program")
        if power_on:
            self._call("power_on")
        if ensure_automatic and self._call_safe("is_robot_in_teach_mode"):
            self._call("switch_to_automatic_mode")
            time.sleep(1.0)
        if self._override is not None:
            self._call("set_override", self._override)
        return self

    def close(self):
        """Faehrt sauber herunter: Servo-Interface aus, dann stop()."""
        if self._robot is None:
            return
        if self._servo_active:
            self._call_safe("deactivate_servo_interface")
            self._servo_active = False
        self._call_safe("stop")

    # -- Aufruf-Helfer -----------------------------------------------------

    @property
    def robot(self):
        if self._robot is None:
            raise NotConnectedError("connect() wurde noch nicht aufgerufen")
        return self._robot

    def _call(self, name, *args, **kwargs):
        """Ruft eine NeuraPy-Funktion auf und uebersetzt bekannte Fehler."""
        fn = getattr(self.robot, name, None)
        if fn is None:
            raise RobotError(
                "Controller bietet die Funktion '%s' nicht an. Verfuegbare "
                "Funktionen mit list_methods() pruefen." % name
            )
        try:
            return fn(*args, **kwargs)
        except Exception as exc:
            translated = _unwrap_neurapy_error(exc, name)
            if translated is not None:
                raise translated from exc
            raise

    def _call_safe(self, name, *args, **kwargs):
        """Wie ``_call``, gibt bei Fehlern aber None zurueck statt zu werfen."""
        try:
            return self._call(name, *args, **kwargs)
        except Exception:
            return None

    def list_methods(self):
        """Alle vom Controller angebotenen Funktionen (Diagnose)."""
        return self.robot.list_methods()

    # -- Zustand -----------------------------------------------------------

    def get_joint_angles(self):
        return list(self._call("get_current_joint_angles"))

    def get_joint_angles_ts(self):
        """(joints, timestamp). Faellt auf Host-Zeit zurueck, falls noetig.

        ACHTUNG (Abnahmeliste AP 0.6): Die Zeitbasis der
        ``*_with_timestamp``-Funktionen (UTC der Control-Box vs. Host-Uhr)
        ist am Geraet zu verifizieren, bevor die Synchronisation darauf
        vertraut.
        """
        try:
            result = self._call("get_current_joint_angles_with_timestamp")
            return _split_timestamped(result)
        except Exception:
            return self.get_joint_angles(), time.time()

    def get_flange_pose(self):
        return list(self._call("get_flange_pose"))

    def get_tcp_pose_quaternion(self):
        return list(self._call("get_tcp_pose_quaternion"))

    def read_state(self):
        """Vollstaendiger, zeitgestempelter Zustand fuer den Recorder."""
        joints, t_joints = self.get_joint_angles_ts()
        t_pose = time.time()
        tcp_quat = self.fk(joints, frame="tool")
        return RobotState(
            t=time.time(),
            joints=np.asarray(joints, dtype=float),
            tcp_quat=tcp_quat,
            gripper_closed=self._gripper_closed,
            t_joints=t_joints,
            t_pose=t_pose,
        )

    def has_tool_offset(self, tol=1e-4):
        """True, wenn im Controller ein Greifer-Tool hinterlegt ist.

        Ohne Tool-Eintrag beziehen sich alle Posen auf den Flansch statt auf
        die Greiferspitze (AP 2.4, Vortest Punkt 0).
        """
        flange = self.get_flange_pose()
        tcp = self.get_tcp_pose_quaternion()
        dist = sum((flange[i] - tcp[i]) ** 2 for i in range(3)) ** 0.5
        return dist > tol

    def pose_is_plausible(self, pose):
        """Erkennt Platzhalterwerte (Pose ausserhalb der Reichweite)."""
        dist = sum(v * v for v in list(pose)[:3]) ** 0.5
        return dist <= config.MAX_REACH_M

    # -- Kinematik ---------------------------------------------------------

    def fk(self, joints, frame="tool"):
        """Gelenkwinkel -> Pose [X,Y,Z,QW,QX,QY,QZ].

        Bevorzugt ``compute_forward_kinematics``, weil das exakt dieselbe
        Winkelkonvention wie ``compute_inverse_kinematics`` verwendet
        (siehe tools/check_ik.py). Rueckgabe projektweit als Quaternion,
        da die Arbeitsposen am +/-pi-Umschlagpunkt der RPY-Darstellung
        liegen (geometry.py).
        """
        pose = self._call(
            "compute_forward_kinematics",
            joint_angles=list(joints),
            target_frame=frame,
            representation="rpy",
        )
        if not pose:
            raise RobotError("compute_forward_kinematics lieferte keine Pose")
        return geometry.pose_rpy_to_quat(list(pose))

    def ik(self, pose_quat, reference_joint):
        """Pose [X,Y,Z,QW,QX,QY,QZ] -> Gelenkwinkel, geseedet.

        Uebergibt die Pose in Quaternion-Darstellung an den Controller
        (``representation='quaternion'``) -- nie RPY, siehe AP 2.4.
        """
        pose_quat = np.asarray(pose_quat, dtype=float)
        if pose_quat.shape[-1] != 7:
            raise ValueError("ik() erwartet eine Quaternion-Pose mit 7 Werten")
        try:
            sol = self._call(
                "compute_inverse_kinematics",
                target_pose=list(pose_quat),
                reference_joint=list(reference_joint),
                representation="quaternion",
            )
        except Exception as exc:
            raise IKError(
                "IK ohne Loesung fuer Pose %s (%s)"
                % (np.round(pose_quat, 4).tolist(), exc),
                reason="ik_not_found",
            ) from exc
        if not sol:
            raise IKError("IK lieferte eine leere Antwort", reason="empty")
        return np.asarray(sol, dtype=float)

    def link_positions(self, joints):
        """Stuetzpunkte via ``compute_forward_kinematics(target_frame=...)``.

        Kostet einen TCP-Roundtrip (~2 ms) je Frame, siehe tools/log.txt.
        """
        out = {}
        for frame in config.COLLISION_CHECK_FRAMES:
            out[frame] = np.asarray(self.fk(joints, frame=frame)[:3], dtype=float)
        return out

    # -- Bewegung ----------------------------------------------------------

    def activate_servo(self, mode="position"):
        self._call("activate_servo_interface", mode)
        self._servo_active = True

    def deactivate_servo(self):
        if self._servo_active:
            self._call("deactivate_servo_interface")
            self._servo_active = False

    def servo_j(self, joint_angles):
        if not self._servo_active:
            raise RobotError("Servo-Interface ist nicht aktiv")
        return self._call("servo_j", list(joint_angles))

    def set_speed(self, joint_percent=None, linear_ms=None):
        """Setzt die globalen Geschwindigkeitsregler (AP 2.6)."""
        if joint_percent is not None:
            self._call("set_joint_speed", joint_percent)
        if linear_ms is not None:
            self._call("set_linear_speed", linear_ms)

    # -- Greifer -----------------------------------------------------------

    @property
    def gripper_closed(self):
        return self._gripper_closed

    def gripper_command(self, close):
        """Setzt den Greiferbefehl ab, OHNE zu warten (Dwell macht der
        Planer als Halte-Schritte in der Trajektorie, AP 2.1)."""
        if close:
            self._call("grasp")
        else:
            self._call("release")
        self._gripper_closed = bool(close)

    # -- Sicherheit --------------------------------------------------------

    def emergency_stop(self):
        """Sofortiger Stopp. Aus jedem Thread aufrufbar, nimmt keine Locks.

        WICHTIG: Das ist ein Software-Stopp und ersetzt keinen zertifizierten
        Hardware-Not-Aus (AP 4.2).
        """
        self._stop_requested.set()
        self._call_safe("stop")
        self._call_safe("stop_movelinear_online")
        self._call_safe("deactivate_servo_interface")
        self._servo_active = False

    @property
    def stop_requested(self):
        return self._stop_requested.is_set()

    def clear_stop(self):
        self._stop_requested.clear()


def _split_timestamped(result):
    """Zerlegt die Antwort einer ``*_with_timestamp``-Funktion.

    Das genaue Format ist versionsabhaengig, daher defensiv: akzeptiert
    (werte, zeit), {"data":..., "timestamp":...} und flache Listen, bei
    denen der letzte Eintrag der Zeitstempel ist.
    """
    if isinstance(result, dict):
        for key in ("data", "value", "values", "result"):
            if key in result:
                values = result[key]
                break
        else:
            raise ValueError("Unerwartetes Timestamp-Format: %r" % (result,))
        for key in ("timestamp", "time", "utc"):
            if key in result:
                return list(values), float(result[key])
        return list(values), time.time()

    if isinstance(result, (list, tuple)) and len(result) == 2:
        values, ts = result
        if isinstance(values, (list, tuple)):
            return list(values), float(ts)

    if isinstance(result, (list, tuple)) and len(result) >= 2:
        return list(result[:-1]), float(result[-1])

    raise ValueError("Unerwartetes Timestamp-Format: %r" % (result,))

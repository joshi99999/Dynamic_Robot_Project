"""NeuraPy-Adapter fuer den LARA 5 (AP 1.5).

Kapselt Verbindung, Lebenszyklus, State-Abgriff und Greifer hinter einer
schmalen Schnittstelle. Wird sowohl bei der Datenaufzeichnung als auch bei
der Live-Inferenz benutzt und ist damit die zentrale Bruecke zwischen
KI-Framework und Hardware.

Bewusst NICHT enthalten: Bewegungsplanung und Rauscheinspielung (Phase 3)
sowie das Servo-Streaming der Inferenz (Phase 6) -- hier steht nur, was
Aufzeichnung und Sicherheit brauchen.
"""

import threading
import time

from . import config


class RobotError(RuntimeError):
    """Fehler in der Kommunikation mit der Control-Box."""


class NotConnectedError(RobotError):
    pass


class RobotState(object):
    """Ein zeitgestempelter Roboterzustand.

    ``gripper_closed`` ist der KOMMANDIERTE Zustand -- NeuraPy liefert keine
    Ist-Rueckmeldung der Greiferweite (AP 2.1).
    """

    __slots__ = ("t", "joints", "tcp_rpy", "gripper_closed", "t_joints", "t_pose")

    def __init__(self, t, joints, tcp_rpy, gripper_closed, t_joints, t_pose):
        self.t = t
        self.joints = joints
        self.tcp_rpy = tcp_rpy
        self.gripper_closed = gripper_closed
        self.t_joints = t_joints
        self.t_pose = t_pose

    @property
    def skew(self):
        """Zeitversatz zwischen Gelenk- und Posenmessung in Sekunden."""
        return abs(self.t_joints - self.t_pose)

    def __repr__(self):
        return "RobotState(t=%.3f, joints=%s, skew=%.4fs, gripper=%s)" % (
            self.t,
            ["%.4f" % j for j in self.joints],
            self.skew,
            "zu" if self.gripper_closed else "auf",
        )


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


class RobotAdapter(object):
    """Schmale, robuste Huelle um ``neurapy.robot.Robot``.

    Nutzung als Context-Manager stellt sicher, dass das Servo-Interface
    deaktiviert und ``stop()`` aufgerufen wird -- auch bei Exceptions::

        with RobotAdapter() as bot:
            print(bot.read_state())

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
        self._warned_no_joint_limits = False

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

    def __enter__(self):
        if self._robot is None:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

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

    @property
    def dof(self):
        return int(getattr(self.robot, "dof", 6))

    # -- Zustand -----------------------------------------------------------

    def get_joint_angles(self):
        return list(self._call("get_current_joint_angles"))

    def get_joint_angles_ts(self):
        """(joints, timestamp). Faellt auf Host-Zeit zurueck, falls noetig."""
        try:
            result = self._call("get_current_joint_angles_with_timestamp")
            return _split_timestamped(result)
        except Exception:
            return self.get_joint_angles(), time.time()

    def get_tcp_pose_rpy(self):
        """TCP-Pose als [X, Y, Z, R, P, Y].

        Bevorzugt ``compute_forward_kinematics``, weil das exakt dieselbe
        Winkelkonvention wie ``compute_inverse_kinematics`` liefert. Erst
        danach die direkten Getter (siehe tools/check_ik.py).
        """
        joints = self.get_joint_angles()
        pose = self._call_safe(
            "compute_forward_kinematics", joint_angles=joints, representation="rpy"
        )
        if pose:
            return list(pose)
        pose = self._call_safe("get_tcp_pose")
        if pose and len(pose) == 6:
            return list(pose)
        raise RobotError(
            "Konnte keine RPY-TCP-Pose ermitteln (weder "
            "compute_forward_kinematics noch get_tcp_pose lieferten Daten)"
        )

    def get_tcp_pose_quaternion(self):
        return list(self._call("get_tcp_pose_quaternion"))

    def get_flange_pose(self):
        return list(self._call("get_flange_pose"))

    def read_state(self):
        """Vollstaendiger, zeitgestempelter Zustand fuer den Recorder."""
        joints, t_joints = self.get_joint_angles_ts()
        t_pose = time.time()
        tcp_rpy = self._call_safe(
            "compute_forward_kinematics", joint_angles=joints, representation="rpy"
        )
        if not tcp_rpy:
            tcp_rpy = self.get_tcp_pose_rpy()
        return RobotState(
            t=time.time(),
            joints=list(joints),
            tcp_rpy=list(tcp_rpy),
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

    def pose_is_plausible(self, pose_rpy):
        """Erkennt Platzhalterwerte (Pose ausserhalb der Reichweite)."""
        dist = sum(v * v for v in pose_rpy[:3]) ** 0.5
        return dist <= config.MAX_REACH_M

    # -- Greifer -----------------------------------------------------------

    @property
    def gripper_closed(self):
        """Kommandierter Greiferzustand (keine Ist-Rueckmeldung verfuegbar)."""
        return self._gripper_closed

    def gripper_close(self, wait=True):
        """Schliesst den Greifer und wartet die Totzeit ab.

        Das Warten ist zwingend: ohne Dwell setzt die Folgebewegung ein,
        bevor die Backen wirklich geschlossen sind (AP 2.1).
        """
        self._call("grasp")
        self._gripper_closed = True
        if wait:
            time.sleep(config.GRIPPER_DWELL_S)

    def gripper_open(self, wait=True):
        self._call("release")
        self._gripper_closed = False
        if wait:
            time.sleep(config.GRIPPER_DWELL_S)

    # -- Servo-Interface ---------------------------------------------------

    def activate_servo(self, mode="position"):
        self._call("activate_servo_interface", mode)
        self._servo_active = True

    def deactivate_servo(self):
        if self._servo_active:
            self._call("deactivate_servo_interface")
            self._servo_active = False

    def servo_j(self, joint_angles):
        """Sendet Zielgelenkwinkel (rad) an das Servo-Interface."""
        if not self._servo_active:
            raise RobotError("Servo-Interface ist nicht aktiv")
        return self._call("servo_j", joint_angles)

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

    def set_speed(self, joint_percent=None, linear_ms=None):
        """Setzt die globalen Geschwindigkeitsregler (AP 2.6)."""
        if joint_percent is not None:
            self._call("set_joint_speed", joint_percent)
        if linear_ms is not None:
            self._call("set_linear_speed", linear_ms)


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

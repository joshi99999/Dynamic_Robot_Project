"""NeuraPy-Adapter fuer den LARA 5 (AP 1.5) -- Implementierung des RobotPort.

Kapselt Verbindung, Lebenszyklus, State-Abgriff, Kinematik-Aufrufe und
Greifer hinter der Port-Schnittstelle. Wird sowohl bei der Datenaufzeichnung
als auch bei der Live-Inferenz benutzt und ist damit die zentrale Bruecke
zwischen KI-Framework und Hardware.

Abnahme ueber die Contract-Tests (``run_all.py --robot=neura``). Gegen die
virtuelle Steuerung (VM "Lara5_V5") geprueft am 2026-09-09 und 2026-09-14;
die drei damaligen Befunde sind hier behoben:

1. ``servo_j`` verlangt drei Listen (Position, Geschwindigkeit,
   Beschleunigung) -- siehe :meth:`NeuraRobot.servo_j`.
2. Die ``*_with_timestamp``-Funktionen liefern in der VM konstant 0.0 --
   Zeitstempel kommen aus der Host-Uhr, siehe :meth:`get_joint_angles_ts`.
3. Kein Greifer konfiguriert -- in der Simulation wird der Greiferbefehl
   nur protokolliert, siehe :meth:`gripper_command`.

SICHERHEIT: VM und reale Control-Box hoeren auf DIESELBE Adresse. Jede
Bewegung setzt deshalb voraus, dass ``is_robot_in_simulation()`` beim
Verbinden ``True`` lieferte -- es sei denn, der Aufrufer erlaubt die reale
Anlage ausdruecklich (``allow_real=True``, in den Apps ``--real-robot`` mit
Bestaetigung). Wer etwas vergisst, bekommt :class:`MotionRefused`, nie
eine Bewegung. Das ist eine Software-Sperre und ersetzt keinen
zertifizierten Hardware-Not-Aus (AP 4.2).

Import von ``neurapy`` erfolgt bewusst erst beim Verbinden, damit sich das
Modul ohne installierte SDKs importieren laesst.
"""

import threading
import time

import numpy as np

from .. import config, geometry
from ..clock import host_time
from ..ports import (
    IKError,
    MotionRefused,
    NotConnectedError,
    PointSourcePort,
    RobotError,
    RobotPort,
    RobotState,
)

#: ``servo_j`` liefert einen Warn-/Fehlercode. Im Doku-Beispiel
#: (``r.get_doc('servo_j')``) laeuft der Strom weiter, solange der Code < 3
#: ist, sonst gilt das Servo als im Fehler.
SERVO_ERROR_CODE_MIN = 3

#: Greifer-Betriebsarten, ermittelt beim Verbinden.
GRIPPER_HARDWARE = "hardware"  # Greifer konfiguriert -> grasp()/release()
GRIPPER_LOGGED = "logged"  # Simulation ohne Greifer -> nur protokolliert
GRIPPER_UNAVAILABLE = "unavailable"  # reale Anlage ohne Greifer -> Fehler


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


class NeuraRobot(RobotPort, PointSourcePort):
    """Schmale, robuste Huelle um ``neurapy.robot.Robot``.

    Thread-Sicherheit: NeuraPy oeffnet pro Aufruf einen eigenen Socket
    (siehe ``generate_function`` in neurapy/robot.py), daher sind Aufrufe aus
    mehreren Threads unproblematisch. ``emergency_stop()`` nimmt bewusst
    keine Locks, damit ein Watchdog nie blockiert.
    """

    def __init__(self, robot=None, override=config.DEFAULT_OVERRIDE, allow_real=False):
        self._robot = robot
        self._override = override
        self._allow_real = bool(allow_real)
        self._in_simulation = None
        self._motion_allowed = False
        self._gripper_mode = None
        self._gripper_closed = False
        self._servo_active = False
        self._stop_requested = threading.Event()
        #: Protokoll der Greiferbefehle im Modus GRIPPER_LOGGED:
        #: Liste von (host_time, geschlossen).
        self.gripper_log = []
        #: Name des im Controller gewaehlten Tools (Diagnose/Metadaten).
        self.tool_name = None
        #: Letzter Rueckgabewert von servo_j (Diagnose).
        self.last_servo_code = None
        #: Letzter vom Controller gemeldeter Zeitstempel (nur Diagnose, s.
        #: get_joint_angles_ts) -- None, wenn keiner geliefert wurde.
        self.last_controller_timestamp = None

    # -- Eigenschaften -----------------------------------------------------

    @property
    def dof(self):
        return int(getattr(self.robot, "dof", 6))

    @property
    def in_simulation(self):
        """Ergebnis von ``is_robot_in_simulation()`` beim Verbinden.

        ``True``/``False``, oder ``None``, wenn die Abfrage scheiterte.
        """
        return self._in_simulation

    @property
    def motion_allowed(self):
        """Duerfen Bewegungsbefehle abgesetzt werden? (siehe Moduldoku)"""
        return self._motion_allowed

    @property
    def gripper_mode(self):
        """GRIPPER_HARDWARE, GRIPPER_LOGGED oder GRIPPER_UNAVAILABLE."""
        return self._gripper_mode

    # -- Lebenszyklus ------------------------------------------------------

    def connect(self, power_on=False, ensure_automatic=False):
        """Verbindet, prueft Simulation/Freigabe und initialisiert.

        Die Simulationspruefung laeuft als ALLERERSTES, vor jedem anderen
        Aufruf. ``power_on``/``ensure_automatic`` gelten als Bewegungsvorbereitung
        und werden ohne Freigabe verweigert.
        """
        if self._robot is None:
            from neurapy.robot import Robot  # lokal, damit Import ohne HW geht

            self._robot = Robot()

        self._in_simulation = self._query_simulation()
        self._motion_allowed = self._in_simulation is True or self._allow_real
        if (power_on or ensure_automatic) and not self._motion_allowed:
            raise MotionRefused(self._refusal_message("power_on"))

        self._call("init_program")
        if power_on:
            self._call("power_on")
        if ensure_automatic and self._call_safe("is_robot_in_teach_mode"):
            self._call("switch_to_automatic_mode")
            time.sleep(1.0)
        if self._override is not None:
            self._call("set_override", self._override)

        self.tool_name = self._call_safe("get_selected_tool_name")
        self._gripper_mode = self._detect_gripper_mode()
        return self

    def close(self):
        """Faehrt sauber herunter: Servo-Interface aus, dann stop()."""
        if self._robot is None:
            return
        if self._servo_active:
            self._call_safe("deactivate_servo_interface")
            self._servo_active = False
        self._call_safe("stop")

    def _query_simulation(self):
        try:
            result = self._call("is_robot_in_simulation")
        except Exception:
            return None
        return result is True

    def _refusal_message(self, action):
        return (
            "Bewegung '%s' verweigert: is_robot_in_simulation() war beim "
            "Verbinden %r. VM und reale Control-Box teilen sich dieselbe "
            "Adresse -- ohne bestaetigte Simulation wird nichts bewegt. Die "
            "reale Anlage nur bewusst freigeben (NeuraRobot(allow_real=True), "
            "in den Apps --real-robot)." % (action, self._in_simulation)
        )

    def _require_motion(self, action):
        if not self._motion_allowed:
            raise MotionRefused(self._refusal_message(action))

    def _detect_gripper_mode(self):
        """Greifer vorhanden? Sonst in der Simulation nur protokollieren.

        Kriterium ist das Attribut ``gripper_name`` des NeuraPy-Clients --
        genau dessen Fehlen laesst ``grasp()`` scheitern ("'Robot' object has
        no attribute 'gripper_name'", VM 2026-09-09; Tool dort "NoTool").
        """
        if getattr(self.robot, "gripper_name", None):
            return GRIPPER_HARDWARE
        if self._in_simulation is True:
            return GRIPPER_LOGGED
        return GRIPPER_UNAVAILABLE

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
        """(joints, timestamp) -- Zeitstempel IMMER aus der Host-Uhr.

        Zeitstempel = Mitte zwischen Absenden und Empfang der Anfrage; der
        Fehler ist damit hoechstens die halbe RPC-Dauer (~1 ms bei ~2 ms
        Roundtrip, gemessen VM 2026-09-09). Kameras stempeln mit derselben
        Uhr (clock.host_time), die Quellen sind also direkt vergleichbar.

        Der Zeitstempel des Controllers wird bewusst NICHT fuer die
        Synchronisation verwendet: in der VM ist er konstant 0.0 (alle vier
        ``*_with_timestamp``-Funktionen, 2026-09-14), und an der Anlage ist
        der Bezug seiner UTC-Zeit zur Host-Uhr unverifiziert (Abnahmeliste
        AP 0.6 Punkt 2). Er bleibt als ``last_controller_timestamp``
        abrufbar, um genau das spaeter zu pruefen.
        """
        t_send = host_time()
        try:
            result = self._call("get_current_joint_angles_with_timestamp")
            joints, t_controller = _split_timestamped(result)
        except Exception:
            t_send = host_time()
            joints, t_controller = self.get_joint_angles(), None
        t_recv = host_time()
        self.last_controller_timestamp = t_controller
        return joints, 0.5 * (t_send + t_recv)

    def get_flange_pose(self):
        return list(self._call("get_flange_pose"))

    def get_tcp_pose_quaternion(self):
        return list(self._call("get_tcp_pose_quaternion"))

    def read_state(self):
        """Vollstaendiger, zeitgestempelter Zustand fuer den Recorder.

        Die TCP-Pose wird aus DERSELBEN Gelenkmessung per FK berechnet --
        ``t_pose`` ist deshalb gleich ``t_joints``.
        """
        joints, t_joints = self.get_joint_angles_ts()
        tcp_quat = self.fk(joints, frame="tool")
        return RobotState(
            t=host_time(),
            joints=np.asarray(joints, dtype=float),
            tcp_quat=tcp_quat,
            gripper_closed=self._gripper_closed,
            t_joints=t_joints,
            t_pose=t_joints,
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
            joint_angles=list(map(float, joints)),
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
                target_pose=list(map(float, pose_quat)),
                reference_joint=list(map(float, reference_joint)),
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

    # -- Geteachte Punkte (PointSourcePort) --------------------------------

    def point_names(self):
        return [str(n) for n in self._call("get_point_names")]

    def get_point(self, name):
        """Punkt aus der Datenbank der Control-Box -> (joints, pose_quat).

        Massgeblich ist die GELENK-Darstellung: sie legt den Loesungszweig
        fest, in dem geteacht wurde, und dient spaeter als IK-Seed. Die
        Pose wird daraus mit derselben FK berechnet, die auch Planer und
        IK verwenden. Die Cartesian-Darstellung der Datenbank dient nur
        der Gegenprobe -- weicht sie ab, wurde der Punkt vermutlich mit
        einem anderen Tool/Frame geteacht, und der Plan wuerde woanders
        hinfahren als das Programm am Pendant.
        """
        if name not in set(self.point_names()):
            raise KeyError(name)
        joints = np.asarray(
            self._call("get_point", name, representation="Joint"), dtype=float
        )
        pose = self.fk(joints, frame="tool")
        cart = self._call("get_point", name, representation="Cartesian")
        stored = geometry.pose_rpy_to_quat(list(cart))
        pos_err = float(np.linalg.norm(pose[:3] - stored[:3]))
        rot_err = geometry.quat_angle_between(pose[3:7], stored[3:7])
        if (
            pos_err > config.POINT_CROSSCHECK_TOL_POS_M
            or rot_err > config.POINT_CROSSCHECK_TOL_ROT_RAD
        ):
            raise RobotError(
                "Punkt '%s': Pose aus der Gelenkstellung weicht von der "
                "gespeicherten Cartesian-Pose ab (%.4f m, %.4f rad). Wurde er "
                "mit einem anderen Tool/Frame geteacht als dem aktuellen "
                "(%r)?" % (name, pos_err, rot_err, self.tool_name)
            )
        return joints, pose

    # -- Bewegung ----------------------------------------------------------

    def activate_servo(self, mode="position"):
        self._require_motion("activate_servo")
        self._call("activate_servo_interface", mode)
        self._servo_active = True

    def deactivate_servo(self):
        if self._servo_active:
            self._call("deactivate_servo_interface")
            self._servo_active = False

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        """Ein Servo-Sollwert: Position (rad), Geschwindigkeit (rad/s),
        Beschleunigung (rad/s^2), je dof Werte.

        ``velocity``/``acceleration`` sind hier Pflicht -- ein stiller
        Default von 0 wuerde dem Controller in jedem Takt "hier anhalten"
        melden (Begruendung in ports.RobotPort.servo_j).
        """
        self._require_motion("servo_j")
        if not self._servo_active:
            raise RobotError("Servo-Interface ist nicht aktiv")
        if self._stop_requested.is_set():
            raise RobotError("Stopp angefordert -- servo_j verweigert")
        if velocity is None or acceleration is None:
            raise ValueError(
                "servo_j am Neura braucht Position, Geschwindigkeit UND "
                "Beschleunigung (trajectory.joint_derivatives)."
            )
        lists = []
        for label, values in (
            ("Position", joint_angles),
            ("Geschwindigkeit", velocity),
            ("Beschleunigung", acceleration),
        ):
            values = [float(v) for v in values]
            if len(values) != self.dof:
                raise ValueError(
                    "servo_j: %s hat %d Werte, erwartet %d"
                    % (label, len(values), self.dof)
                )
            lists.append(values)

        code = self._call("servo_j", *lists)
        self.last_servo_code = code
        if isinstance(code, (int, float)) and code >= SERVO_ERROR_CODE_MIN:
            raise RobotError("servo_j meldet Fehlercode %r" % (code,))
        return code

    def move_to_joints(self, joints):
        """Blockierende PTP-Fahrt (``move_joint``) mit Zielkontrolle."""
        self._require_motion("move_to_joints")
        if self._servo_active:
            raise RobotError("move_to_joints bei aktivem Servo-Interface")
        target = [float(v) for v in joints]
        if len(target) != self.dof:
            raise ValueError("move_to_joints erwartet %d Winkel" % self.dof)
        self._call(
            "move_joint",
            target_joint=[target],
            speed=config.RESET_JOINT_SPEED,
            acceleration=config.RESET_JOINT_ACCELERATION,
            current_joint_angles=[float(v) for v in self.get_joint_angles()],
        )
        reached = np.asarray(self.get_joint_angles(), dtype=float)
        err = float(np.max(np.abs(reached - np.asarray(target))))
        if err > config.START_POSE_TOL_RAD:
            raise RobotError(
                "move_to_joints: Ziel nicht erreicht (Restfehler %.4f rad)" % err
            )

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
        Planer als Halte-Schritte in der Trajektorie, AP 2.1).

        Ohne konfigurierten Greifer wird der Befehl in der Simulation nur
        in ``gripper_log`` protokolliert -- fuer die Datenpipeline zaehlt
        der KOMMANDIERTE Zustand, der ohnehin im State steht. An der realen
        Anlage ist ein fehlender Greifer dagegen ein Fehler.
        """
        close = bool(close)
        mode = self._gripper_mode
        if mode == GRIPPER_HARDWARE:
            self._require_motion("gripper_command")
            self._call("grasp" if close else "release")
        elif mode == GRIPPER_LOGGED:
            self.gripper_log.append((host_time(), close))
        elif mode == GRIPPER_UNAVAILABLE:
            raise RobotError(
                "Kein Greifer konfiguriert (Tool %r) und keine Simulation -- "
                "Greiferbefehl nicht ausfuehrbar." % (self.tool_name,)
            )
        else:
            raise NotConnectedError("connect() wurde noch nicht aufgerufen")
        self._gripper_closed = close

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

    Rueckgabe ``(werte, zeitstempel)``; ``zeitstempel`` ist None, wenn die
    Antwort keinen brauchbaren enthaelt (fehlend, <= 0 oder nicht endlich --
    die VM liefert konstant 0.0). Das Format ist versionsabhaengig, daher
    defensiv: akzeptiert (werte, zeit), {"data":..., "timestamp":...} und
    flache Listen, bei denen der letzte Eintrag der Zeitstempel ist.
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
                return list(values), _valid_timestamp(result[key])
        return list(values), None

    if isinstance(result, (list, tuple)) and len(result) == 2:
        values, ts = result
        if isinstance(values, (list, tuple)):
            return list(values), _valid_timestamp(ts)

    if isinstance(result, (list, tuple)) and len(result) >= 2:
        return list(result[:-1]), _valid_timestamp(result[-1])

    raise ValueError("Unerwartetes Timestamp-Format: %r" % (result,))


def _valid_timestamp(value):
    try:
        ts = float(value)
    except (TypeError, ValueError):
        return None
    return ts if np.isfinite(ts) and ts > 0.0 else None

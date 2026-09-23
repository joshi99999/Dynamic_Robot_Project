"""Ports: die abstrakten Schnittstellen zwischen Pipeline und Hardware (AP 0.3).

Die gesamte Pipeline (Recorder, Planer, Sicherheit, Inferenz) kennt NUR
diese Schnittstellen. Konkrete Hardware (NeuraPy, gxipy, OpenCV) und die
Simulationen leben in ``bc/adapters/`` und implementieren diese Ports.

Verbindliche Regel (durch tests/test_layering.py abgesichert): Kein Modul
ausserhalb von ``bc/adapters/`` importiert ``neurapy`` oder ``gxipy``.

Einheiten- und Formatzusagen (Contract, geprueft in tests/contract/):
    * Gelenkwinkel: rad, Reihenfolge Basis -> Flansch, Laenge = dof.
    * Posen: [X, Y, Z, QW, QX, QY, QZ] in Metern / Einheitsquaternion,
      Basis-Koordinatensystem des Roboters.
    * Zeitstempel: Sekunden der Host-Uhr (``bc.clock.host_time`` bzw.
      ``ClockPort.now()``) -- fuer Kameras UND Roboter dieselbe Zeitbasis.
    * Bilder: RGB, uint8, (H, W, 3).
"""

import abc


class RobotError(RuntimeError):
    """Fehler in der Kommunikation mit dem Roboter (real oder simuliert)."""


class NotConnectedError(RobotError):
    pass


class MotionRefused(RobotError):
    """Bewegung verweigert, weil nicht sichergestellt ist, dass es die
    Simulation ist (Sicherheitsregel 1: VM und Anlage teilen eine IP)."""


class ServoLimitError(RobotError):
    """servo_j-Sollwert abgelehnt: Sprung, Geschwindigkeit, Abstand zur
    Ist-Stellung oder Achsgrenze verletzt (servo.ServoGuard). Der Adapter
    hat dann bereits den Software-Stopp ausgeloest."""


class CameraError(RuntimeError):
    pass


class RobotState(object):
    """Ein zeitgestempelter Roboterzustand.

    ``gripper_closed`` ist der KOMMANDIERTE Zustand -- NeuraPy liefert keine
    Ist-Rueckmeldung der Greiferweite (AP 2.1).

    ``tcp_quat`` ist die TCP-Pose als [X,Y,Z,QW,QX,QY,QZ]. Projektweit wird
    in Quaternionen gerechnet, weil die realen Arbeitsposen am
    +/-pi-Umschlagpunkt der RPY-Darstellung liegen (siehe geometry.py).
    """

    __slots__ = ("t", "joints", "tcp_quat", "gripper_closed", "t_joints", "t_pose")

    def __init__(self, t, joints, tcp_quat, gripper_closed, t_joints, t_pose):
        self.t = t
        self.joints = joints
        self.tcp_quat = tcp_quat
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


class Frame(object):
    """Ein Kamerabild mit Zeitstempel.

    ``timestamp`` ist verbindlich die Host-Zeit unmittelbar NACH dem Abgriff
    (enthaelt also die Uebertragungslatenz). Darauf beruht die gesamte
    Synchronisation (AP 1.3) -- Konvention nicht stillschweigend aendern.
    """

    __slots__ = ("image", "timestamp", "index", "source")

    def __init__(self, image, timestamp, index, source):
        self.image = image
        self.timestamp = timestamp
        self.index = index
        self.source = source

    @property
    def shape(self):
        return self.image.shape

    def __repr__(self):
        return "Frame(%s, #%d, %s, t=%.3f)" % (
            self.source,
            self.index,
            self.image.shape,
            self.timestamp,
        )


class ClockPort(abc.ABC):
    """Injizierbare Zeitquelle.

    Pipeline-Code nimmt Zeit NIE direkt ueber ``time.time()``/``time.sleep``,
    sondern ueber diesen Port -- nur so sind Tests deterministisch und
    schneller als Echtzeit (AP 0.5).
    """

    @abc.abstractmethod
    def now(self):
        """Aktuelle Zeit in Sekunden (float)."""

    @abc.abstractmethod
    def sleep(self, seconds):
        """Wartet die angegebene Dauer."""


class RobotPort(abc.ABC):
    """Semantische Roboterschnittstelle (AP 1.5).

    Implementierungen: ``adapters.neura.NeuraRobot`` (Hardware) und
    ``adapters.sim_robot.SimRobot`` (URDF-Kinematik, hardwarefrei).
    """

    # -- Eigenschaften -----------------------------------------------------

    @property
    @abc.abstractmethod
    def dof(self):
        """Anzahl der Gelenke (LARA 5: 6)."""

    # -- Lebenszyklus ------------------------------------------------------

    @abc.abstractmethod
    def connect(self):
        """Stellt die Verbindung her. Gibt self zurueck."""

    @abc.abstractmethod
    def close(self):
        """Faehrt sauber herunter (Servo aus, Stopp)."""

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    # -- Zustand -----------------------------------------------------------

    @abc.abstractmethod
    def read_state(self):
        """Vollstaendiger, zeitgestempelter Zustand -> :class:`RobotState`."""

    # -- Kinematik (AP 0.4) ------------------------------------------------

    @abc.abstractmethod
    def fk(self, joints, frame="tool"):
        """Gelenkwinkel -> Pose [X,Y,Z,QW,QX,QY,QZ] des Frames.

        Unterstuetzte Frames mindestens: die Namen aus
        ``config.COLLISION_CHECK_FRAMES``.
        """

    @abc.abstractmethod
    def ik(self, pose_quat, reference_joint):
        """Pose -> Gelenkwinkel, geseedet mit ``reference_joint``.

        Der Seed ist verbindliche Semantik: die Loesung ist die zur
        Referenzkonfiguration naechstgelegene (keine Zweigspruenge).
        Wirft :class:`IKError`, wenn keine Loesung existiert.
        """

    @abc.abstractmethod
    def link_positions(self, joints):
        """Stuetzpunkte entlang des Arms fuer die Kollisionspruefung.

        Rueckgabe: dict Name -> ndarray(3,) in Weltkoordinaten. Enthaelt
        mindestens die Namen aus ``config.COLLISION_CHECK_FRAMES``.
        """

    # -- Bewegung ----------------------------------------------------------

    @abc.abstractmethod
    def activate_servo(self, mode="position"):
        """Aktiviert das Echtzeit-Servo-Interface."""

    @abc.abstractmethod
    def deactivate_servo(self):
        pass

    @abc.abstractmethod
    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        """Sendet einen Sollwert an das aktive Servo-Interface.

        ``joint_angles`` in rad, ``velocity`` in rad/s, ``acceleration`` in
        rad/s^2 -- je ``dof`` Werte. Der NeuraPy-Controller verlangt alle
        drei Listen (``r.get_doc('servo_j')``); ``velocity``/``acceleration``
        beschreiben, mit welcher Geschwindigkeit der Arm den Sollwert
        erreichen soll. Sie werden aus der geplanten Bahn abgeleitet
        (``trajectory.joint_derivatives``), nie pauschal zu 0 gesetzt: 0
        heisst "am Sollwert anhalten" und erzeugt bei 15 Hz Stop-and-go.
        Bewusst 0 ist nur beim Halten der Ist-Stellung korrekt.

        ``None`` ist nur fuer Implementierungen zulaessig, die die Werte
        ohnehin ignorieren (SimRobot); der Neura-Adapter verweigert es.
        """

    @abc.abstractmethod
    def move_to_joints(self, joints):
        """Blockierende PTP-Fahrt auf eine Gelenkstellung (rad).

        Fuer Fahrten AUSSERHALB der Aufzeichnung, z. B. zurueck an die
        Startpose zwischen zwei Episoden. Kehrt erst nach Erreichen zurueck;
        wirft :class:`RobotError`, wenn das Ziel nicht erreicht wurde.
        """

    # -- Greifer -----------------------------------------------------------

    @property
    @abc.abstractmethod
    def gripper_closed(self):
        """Kommandierter Greiferzustand (keine Ist-Rueckmeldung verfuegbar)."""

    @abc.abstractmethod
    def gripper_command(self, close):
        """Setzt den Greiferbefehl ab und kehrt SOFORT zurueck.

        Die Totzeit (GRIPPER_DWELL_S) wird NICHT hier abgewartet, sondern
        vom Planer als Dwell-Schritte in der Trajektorie abgebildet
        (AP 2.1) -- ein blockierendes sleep() im Adapter wuerde bei 15 Hz
        die Aufzeichnung anhalten.
        """

    # -- Sicherheit --------------------------------------------------------

    @abc.abstractmethod
    def emergency_stop(self):
        """Sofortiger Software-Stopp. Aus jedem Thread aufrufbar, darf
        niemals blockieren (keine Locks). Ersetzt KEINEN zertifizierten
        Hardware-Not-Aus (AP 4.2)."""

    @property
    @abc.abstractmethod
    def stop_requested(self):
        pass

    @abc.abstractmethod
    def clear_stop(self):
        pass


class PointSourcePort(abc.ABC):
    """Quelle geteachter Punkte, adressiert ueber ihren Namen (AP 2.2).

    Am Neura ist das die Punkte-Datenbank der Control-Box: Punkte werden am
    Teach-Pendant angelegt und per Touch-up nachgeteacht. Die Pipeline
    haelt nur die REIHENFOLGE der Namen (bc.sequence) und fragt die
    Koordinaten vor jeder Episode frisch ab -- so wirkt ein Touch-up ohne
    Export in die naechste Episode.
    """

    @abc.abstractmethod
    def point_names(self):
        """Namen aller verfuegbaren Punkte (Liste von str)."""

    @abc.abstractmethod
    def get_point(self, name):
        """Punkt -> ``(joints, pose_quat)``.

        ``joints``: geteachte Gelenkstellung in rad (ndarray, dof).
        ``pose_quat``: TCP-Pose [X,Y,Z,QW,QX,QY,QZ] dieser Stellung.
        Wirft ``KeyError``, wenn der Punkt nicht existiert.
        """


class IKError(RobotError):
    """Keine IK-Loesung gefunden (entspricht NeuraPy ``IKNotFound``)."""

    def __init__(self, message, reason=None):
        super().__init__(message)
        self.reason = reason


class CameraPort(abc.ABC):
    """Gemeinsame Schnittstelle aller Kameratypen (AP 1.1)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.name = cfg.name
        self._index = 0
        self._open = False

    @abc.abstractmethod
    def open(self):
        pass

    @abc.abstractmethod
    def read(self):
        """Liefert den naechsten :class:`Frame` (RGB, uint8). Blockiert bis
        ein Bild vorliegt; wirft :class:`CameraError` bei Fehlern."""

    @abc.abstractmethod
    def close(self):
        pass

    @property
    def is_open(self):
        return self._open

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def _next_index(self):
        i = self._index
        self._index += 1
        return i


class TeleopEvent(object):
    """Ein Ereignis der Teleoperation (AP 2.2: nur Teachen, kein Aufzeichnen).

    ``kind`` ist einer der Werte:
        "jog"      -- kartesischer Verfahrwunsch, ``value`` = ndarray(3,) m
        "save"     -- aktuelle Pose als Wegpunkt speichern
        "gripper"  -- Greifer togglen, ``value`` = True (zu) / False (auf)
        "quit"     -- Teachen beenden
    """

    __slots__ = ("kind", "value")

    def __init__(self, kind, value=None):
        self.kind = kind
        self.value = value

    def __repr__(self):
        return "TeleopEvent(%s, %r)" % (self.kind, self.value)


class TeleopPort(abc.ABC):
    """Eingabegeraet fuers Teachen der Wegpunkte."""

    @abc.abstractmethod
    def poll(self):
        """Liefert das naechste :class:`TeleopEvent` oder None."""

    @abc.abstractmethod
    def close(self):
        pass

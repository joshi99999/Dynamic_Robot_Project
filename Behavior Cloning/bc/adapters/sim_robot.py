"""Hardwarefreier Roboter: URDF-Kinematik plus Fehlerinjektion (AP 0.4/0.5).

Implementiert :class:`bc.ports.RobotPort` vollstaendig offline:

* FK analytisch aus der URDF-Kette (``bc/data/lara5_candidate.urdf``).
* IK numerisch als gedaempfte Pseudoinverse (Damped Least Squares) mit
  derselben Seed-Semantik wie NeuraPys ``reference_joint``: gestartet wird
  an der Referenzkonfiguration, geliefert wird die naechstgelegene Loesung.
* Fehlerinjektion (:class:`FaultProfile`), damit Tests nicht nur den
  Schoenwetterfall pruefen: IK-Ausfaelle, leere Socket-Antworten,
  Antwortlatenz, veraltete Zeitstempel.

ZWECK (AP 0.4, Gleis A): Pruefung der LOGIK von Planer, Rauschen,
Kollisionspruefung und Recorder. Geometrische Zahlenwerte (Erreichbarkeit,
Freiraeume, dq/dx) sind NICHT auf die reale Anlage uebertragbar, solange
die Kalibrierung aus Gleis B aussteht.
"""

import threading
from dataclasses import dataclass

import numpy as np

from .. import config, geometry
from ..clock import RealClock
from ..ports import (
    IKError,
    NotConnectedError,
    PointSourcePort,
    RobotError,
    RobotPort,
    RobotState,
)
from ..urdf import KinematicChain


@dataclass
class FaultProfile:
    """Konfigurierbare Stoerungen fuer Tests (AP 0.5, Fehlerinjektion).

    Alle Raten sind Wahrscheinlichkeiten je Aufruf in [0, 1].
    """

    #: IK wirft IKError (entspricht NeuraPy ``IKNotFound``).
    ik_fail_rate: float = 0.0
    #: Aufrufe schlagen mit RobotError fehl (leere Socket-Antwort, der in
    #: adapters/neura.py uebersetzte NeuraPy-Bug).
    comm_fail_rate: float = 0.0
    #: Kuenstliche Antwortlatenz je Aufruf (ueber die injizierte Uhr).
    latency_s: float = 0.0
    #: Zeitstempel des Roboterzustands haengen um diesen Betrag hinterher
    #: (konstruiert Verletzungen des Latenzbudgets aus AP 1.3).
    stale_state_s: float = 0.0
    #: Gleichverteiltes Rauschen auf den per servo_j erreichten Winkeln.
    servo_noise_rad: float = 0.0


#: Abbildung der NeuraPy-Framenamen auf die revoluten Gelenke der Kette
#: (1-basiert): nach welchem Gelenk der jeweilige Frame abgegriffen wird.
_FRAME_AFTER_JOINT = {
    "elbow": 3,
    "link5": 5,
    "wrist": 5,
    "link6": 6,
    "flange": None,  # Kettenende
    "tool": None,  # Kettenende (kein Tool-Offset hinterlegt, vgl. log.txt)
}


class SimRobot(RobotPort, PointSourcePort):
    """Simulierter LARA 5 auf Basis der URDF-Kette.

    ``points``: optionale "Punkte-Datenbank" als dict Name -> Gelenkstellung
    (rad), analog zur Datenbank der Control-Box -- damit laesst sich ein
    Ablauf aus bc.sequence hardwarefrei durchspielen.
    """

    def __init__(
        self,
        clock=None,
        urdf_path=config.URDF_PATH,
        joint_limits=config.JOINT_LIMITS_RAD,
        faults=None,
        seed=0,
        home=None,
        points=None,
    ):
        self._clock = clock if clock is not None else RealClock()
        self._chain = KinematicChain.from_urdf(urdf_path)
        self._limits = joint_limits
        self._faults = faults if faults is not None else FaultProfile()
        self._rng = np.random.default_rng(seed)
        self._connected = False
        self._servo_active = False
        self._gripper_closed = False
        self._gripper_cmd_time = None
        self._stop_requested = threading.Event()
        #: (Position, Geschwindigkeit, Beschleunigung) des letzten servo_j.
        self.last_servo_command = None
        self._points = {
            str(k): np.asarray(v, dtype=float) for k, v in (points or {}).items()
        }
        self._joints = (
            np.asarray(home, dtype=float)
            if home is not None
            else np.array([-0.2, 0.1, 1.5, 0.0, 1.57, -0.2])
        )
        if len(self._joints) != self._chain.n_joints:
            raise ValueError(
                "home hat %d Werte, Kette hat %d Gelenke"
                % (len(self._joints), self._chain.n_joints)
            )

    # -- Fehlerinjektion ---------------------------------------------------

    def _fault_gate(self):
        if self._faults.latency_s > 0:
            self._clock.sleep(self._faults.latency_s)
        if self._faults.comm_fail_rate > 0 and self._rng.random() < self._faults.comm_fail_rate:
            raise RobotError(
                "Control-Box hat nicht geantwortet (simulierte leere "
                "Socket-Antwort)"
            )

    # -- Eigenschaften -----------------------------------------------------

    @property
    def dof(self):
        return self._chain.n_joints

    @property
    def chain(self):
        """Zugriff auf die Kette fuer Diagnose und Tests."""
        return self._chain

    # -- Lebenszyklus ------------------------------------------------------

    def connect(self):
        self._connected = True
        return self

    def close(self):
        self._servo_active = False
        self._connected = False

    def _require_connected(self):
        if not self._connected:
            raise NotConnectedError("connect() wurde noch nicht aufgerufen")

    # -- Zustand -----------------------------------------------------------

    def read_state(self):
        self._require_connected()
        self._fault_gate()
        t = self._clock.now()
        t_meas = t - self._faults.stale_state_s
        return RobotState(
            t=t,
            joints=self._joints.copy(),
            tcp_quat=self.fk(self._joints, frame="tool"),
            gripper_closed=self._gripper_closed,
            t_joints=t_meas,
            t_pose=t_meas,
        )

    # -- Kinematik ---------------------------------------------------------

    def _frame_transform(self, joints, frame):
        frames = self._chain.fk_frames(joints)
        if frame not in _FRAME_AFTER_JOINT:
            raise RobotError(
                "Unbekannter Frame '%s' (bekannt: %s)"
                % (frame, ", ".join(sorted(_FRAME_AFTER_JOINT)))
            )
        after = _FRAME_AFTER_JOINT[frame]
        if after is None:
            return frames[-1][1]
        count = 0
        for (name, T), joint in zip(frames, self._chain.joints):
            if joint.type == "revolute":
                count += 1
                if count == after:
                    return T
        raise RobotError("Kette hat weniger als %d revolute Gelenke" % after)

    def fk(self, joints, frame="tool"):
        self._fault_gate()
        T = self._frame_transform(np.asarray(joints, dtype=float), frame)
        quat = geometry.matrix_to_quat(T[:3, :3])
        return np.concatenate([T[:3, 3], quat])

    def ik(self, pose_quat, reference_joint, max_iter=200, tol_pos=1e-5, tol_rot=1e-4):
        """Gedaempfte Pseudoinverse (DLS), Warm-Start am Seed.

        Konvergiert die Iteration nicht oder verletzt die Loesung die
        Achsgrenzen, wird :class:`IKError` geworfen -- das ist das
        Sim-Aequivalent von NeuraPys ``IKNotFound`` und der Ausloeser fuer
        das Rejection Sampling in AP 2.4.
        """
        self._fault_gate()
        if self._faults.ik_fail_rate > 0 and self._rng.random() < self._faults.ik_fail_rate:
            raise IKError("IK ohne Loesung (simulierter Ausfall)", reason="injected")

        target = np.asarray(pose_quat, dtype=float)
        if target.shape != (7,):
            raise ValueError("ik() erwartet eine Quaternion-Pose mit 7 Werten")
        target_pos = target[:3]
        target_rot = geometry.quat_to_matrix(target[3:7])

        q = np.asarray(reference_joint, dtype=float).copy()
        damping = 0.05
        for _ in range(max_iter):
            T = self._frame_transform(q, "tool")
            err_pos = target_pos - T[:3, 3]
            R_err = target_rot @ T[:3, :3].T
            err_rot = _rotation_vector(R_err)
            if np.linalg.norm(err_pos) < tol_pos and np.linalg.norm(err_rot) < tol_rot:
                bad = self._limit_violations(q)
                if bad:
                    raise IKError(
                        "IK-Loesung verletzt Achsgrenzen an Gelenk(en) %s" % bad,
                        reason="joint_limits",
                    )
                return q
            err = np.concatenate([err_pos, err_rot])
            J = self._jacobian(q)
            JJt = J @ J.T + (damping**2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)
            step = float(np.max(np.abs(dq)))
            if step > 0.2:  # Schrittweite begrenzen, haelt die Linearisierung gueltig
                dq *= 0.2 / step
            q = q + dq

        raise IKError(
            "IK nicht konvergiert fuer Pose %s" % np.round(target, 4).tolist(),
            reason="no_convergence",
        )

    def _jacobian(self, q, eps=1e-6):
        """Numerische 6xN-Jacobi-Matrix (Position + Rotationsvektor)."""
        T0 = self._frame_transform(q, "tool")
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]
        J = np.zeros((6, len(q)))
        for i in range(len(q)):
            qi = q.copy()
            qi[i] += eps
            Ti = self._frame_transform(qi, "tool")
            J[:3, i] = (Ti[:3, 3] - p0) / eps
            J[3:, i] = _rotation_vector(Ti[:3, :3] @ R0.T) / eps
        return J

    def _limit_violations(self, q):
        if self._limits is None:
            return []
        bad = []
        for i, val in enumerate(q):
            lo, hi = self._limits[i]
            if lo is not None and (val < lo or val > hi):
                bad.append(i)
        return bad

    def link_positions(self, joints):
        self._fault_gate()
        joints = np.asarray(joints, dtype=float)
        return {
            name: self._frame_transform(joints, name)[:3, 3]
            for name in config.COLLISION_CHECK_FRAMES
        }

    # -- Bewegung ----------------------------------------------------------

    def activate_servo(self, mode="position"):
        self._require_connected()
        self._fault_gate()
        self._servo_active = True

    def deactivate_servo(self):
        self._servo_active = False

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        """Folgt dem Positionssollwert sofort und exakt.

        ``velocity``/``acceleration`` beeinflussen die Simulation nicht,
        werden aber (falls angegeben) auf ihre Laenge geprueft und fuer
        Tests in ``last_servo_command`` abgelegt.
        """
        self._require_connected()
        if not self._servo_active:
            raise RobotError("Servo-Interface ist nicht aktiv")
        if self._stop_requested.is_set():
            raise RobotError("Stopp angefordert -- servo_j verweigert")
        self._fault_gate()
        target = np.asarray(joint_angles, dtype=float)
        if len(target) != self.dof:
            raise ValueError("servo_j erwartet %d Winkel" % self.dof)
        for label, values in (("velocity", velocity), ("acceleration", acceleration)):
            if values is not None and len(values) != self.dof:
                raise ValueError("servo_j: %s braucht %d Werte" % (label, self.dof))
        self.last_servo_command = (target.copy(), velocity, acceleration)
        if self._faults.servo_noise_rad > 0:
            target = target + self._rng.uniform(
                -self._faults.servo_noise_rad, self._faults.servo_noise_rad, self.dof
            )
        self._joints = target

    def move_to_joints(self, joints):
        """PTP-Fahrt: in der Simulation ein Sprung ans Ziel."""
        self._require_connected()
        if self._servo_active:
            raise RobotError("move_to_joints bei aktivem Servo-Interface")
        if self._stop_requested.is_set():
            raise RobotError("Stopp angefordert -- move_to_joints verweigert")
        self._fault_gate()
        target = np.asarray(joints, dtype=float)
        if len(target) != self.dof:
            raise ValueError("move_to_joints erwartet %d Winkel" % self.dof)
        self._joints = target.copy()

    # -- Geteachte Punkte (PointSourcePort) --------------------------------

    def point_names(self):
        return list(self._points)

    def get_point(self, name):
        if name not in self._points:
            raise KeyError(name)
        joints = self._points[name].copy()
        return joints, self.fk(joints, frame="tool")

    # -- Greifer -----------------------------------------------------------

    @property
    def gripper_closed(self):
        return self._gripper_closed

    def gripper_command(self, close):
        self._require_connected()
        self._fault_gate()
        self._gripper_closed = bool(close)
        self._gripper_cmd_time = self._clock.now()

    def gripper_settled(self):
        """Nur Simulation/Tests: sind die Backen physisch angekommen?

        Die reale Anlage liefert diese Information NICHT (AP 2.1) -- die
        Pipeline darf sich also nie darauf stuetzen. Tests benutzen sie,
        um die Dwell-Logik des Planers zu verifizieren.
        """
        if self._gripper_cmd_time is None:
            return True
        return self._clock.now() - self._gripper_cmd_time >= config.GRIPPER_DWELL_S

    # -- Sicherheit --------------------------------------------------------

    def emergency_stop(self):
        self._stop_requested.set()
        self._servo_active = False

    @property
    def stop_requested(self):
        return self._stop_requested.is_set()

    def clear_stop(self):
        self._stop_requested.clear()


def _rotation_vector(R):
    """Rotationsmatrix -> Rotationsvektor (Achse * Winkel)."""
    q = geometry.matrix_to_quat(R)
    w = max(-1.0, min(1.0, float(q[0])))
    angle = 2.0 * np.arccos(w)
    if angle < 1e-12:
        return np.zeros(3)
    axis = q[1:4] / np.sin(angle / 2.0)
    if angle > np.pi:
        angle -= 2.0 * np.pi
    return axis * angle

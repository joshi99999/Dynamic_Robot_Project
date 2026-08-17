"""Kinematik mit Absicherungen fuer die Rauscheinspielung (AP 2.4).

Arbeitet ausschliesslich gegen :class:`bc.ports.RobotPort` -- laeuft also
identisch gegen den NeuraPy-Adapter (Controller-IK, siehe Vortest in
tools/log.txt) und gegen den SimRobot (URDF-Kinematik, hardwarefrei).

Ergaenzt die Port-IK um die vier im Requirements-Dokument geforderten
Absicherungen:

1. IKError abfangen         -> zugleich Erreichbarkeitspruefung
2. |Delta q|-Schranke       -> erkennt Konfigurationsspruenge/Singularitaeten
3. FK-Rueckprobe            -> IK-Loesung muss die Sollpose reproduzieren
4. Achsgrenzen              -> gemaess config.JOINT_LIMITS_RAD

Zentrale Designentscheidung: Zielposen werden intern als **Quaternion**
gefuehrt. Grund siehe geometry.py -- die realen Arbeitsposen liegen am
+/-pi-Umschlagpunkt der RPY-Darstellung.
"""

import numpy as np

from . import config, geometry
from .ports import IKError, RobotError


class IKFailure(RobotError):
    """Eine IK-Loesung fehlt oder hat eine Pruefung nicht bestanden."""

    def __init__(self, message, index=None, reason=None):
        super().__init__(message)
        self.index = index
        self.reason = reason


class Kinematics(object):
    """Kinematik-Fassade auf Basis eines :class:`RobotPort`."""

    def __init__(self, robot, joint_limits=config.JOINT_LIMITS_RAD):
        self.robot = robot
        self.joint_limits = joint_limits
        self._warned_no_limits = False

    # -- Vorwaertskinematik ------------------------------------------------

    def fk_quat(self, joints, frame="tool"):
        """Gelenkwinkel -> [X,Y,Z,QW,QX,QY,QZ] fuer den gewuenschten Frame."""
        return np.asarray(self.robot.fk(joints, frame=frame), dtype=float)

    def fk_position(self, joints, frame="tool"):
        """Nur die kartesische Position."""
        return self.fk_quat(joints, frame=frame)[:3]

    # -- Inverskinematik ---------------------------------------------------

    def ik(self, pose_quat, reference_joint):
        """Eine einzelne IK-Loesung, geseedet mit ``reference_joint``.

        Der Seed ist der Kern der Robustheit: die IK liefert die zur
        Referenzkonfiguration naechstgelegene Loesung, wodurch Spruenge
        zwischen Gelenkkonfigurationen konstruktiv ausgeschlossen sind.
        """
        pose_quat = np.asarray(pose_quat, dtype=float)
        if pose_quat.shape[-1] != 7:
            raise ValueError("ik() erwartet eine Quaternion-Pose mit 7 Werten")
        try:
            sol = self.robot.ik(pose_quat, reference_joint)
        except IKError as exc:
            raise IKFailure(
                "IK ohne Loesung fuer Pose %s (%s)"
                % (np.round(pose_quat, 4).tolist(), exc),
                reason=exc.reason or "ik_not_found",
            ) from exc
        return np.asarray(sol, dtype=float)

    # -- Pruefungen --------------------------------------------------------

    def check_joint_limits(self, joints):
        """Prueft Achsgrenzen. Gibt Liste verletzter Gelenkindizes zurueck."""
        if self.joint_limits is None:
            if not self._warned_no_limits:
                print(
                    "[kinematics] WARNUNG: Achsgrenzen sind nicht gesetzt -- "
                    "Grenzpruefung wird uebersprungen."
                )
                self._warned_no_limits = True
            return []
        bad = []
        for i, q in enumerate(joints):
            lo, hi = self.joint_limits[i]
            if lo is not None and (q < lo or q > hi):
                bad.append(i)
        return bad

    def check_fk_roundtrip(self, joints, target_quat):
        """Rechnet die IK-Loesung zurueck und vergleicht mit der Sollpose.

        Rueckgabe: (positions_fehler_m, rotations_fehler_rad).
        """
        actual = self.fk_quat(joints)
        pos_err = float(np.linalg.norm(actual[:3] - np.asarray(target_quat)[:3]))
        rot_err = geometry.quat_angle_between(actual[3:7], np.asarray(target_quat)[3:7])
        return pos_err, rot_err

    # -- Bahnloesung -------------------------------------------------------

    def solve_path(
        self,
        poses_quat,
        seed_joints,
        max_delta_q=config.IK_MAX_DELTA_Q_RAD,
        fk_tol_pos=config.IK_FK_TOL_POS_M,
        fk_tol_rot=config.IK_FK_TOL_ROT_RAD,
        verify_fk=True,
    ):
        """Loest eine ganze Posenbahn im Gelenkraum -- mit Warm-Start.

        Jeder Schritt benutzt die Loesung des Vorschritts als Seed. Alle vier
        Absicherungen laufen mit; bei Verletzung wird :class:`IKFailure` mit
        dem betroffenen Index geworfen, sodass der Aufrufer die Trajektorie
        verwerfen und neu sampeln kann (Rejection Sampling, AP 2.4).

        Rueckgabe: Array (N, dof) mit Gelenkwinkeln in rad.
        """
        poses_quat = np.asarray(poses_quat, dtype=float)
        if poses_quat.ndim != 2 or poses_quat.shape[1] != 7:
            raise ValueError("solve_path() erwartet ein Array (N, 7)")

        reference = np.asarray(seed_joints, dtype=float)
        solutions = np.empty((len(poses_quat), len(reference)), dtype=float)

        for i, pose in enumerate(poses_quat):
            try:
                sol = self.ik(pose, reference)
            except IKFailure as exc:
                exc.index = i
                raise

            delta = float(np.max(np.abs(sol - reference)))
            if i > 0 and delta > max_delta_q:
                raise IKFailure(
                    "Konfigurationssprung bei Schritt %d: |Delta q| = %.4f rad "
                    "> %.4f rad. Verdacht auf Singularitaetsdurchgang oder "
                    "Zweigwechsel." % (i, delta, max_delta_q),
                    index=i,
                    reason="delta_q",
                )

            bad = self.check_joint_limits(sol)
            if bad:
                raise IKFailure(
                    "Achsgrenze verletzt bei Schritt %d, Gelenk(e) %s"
                    % (i, bad),
                    index=i,
                    reason="joint_limits",
                )

            if verify_fk:
                pos_err, rot_err = self.check_fk_roundtrip(sol, pose)
                if pos_err > fk_tol_pos or rot_err > fk_tol_rot:
                    raise IKFailure(
                        "FK-Rueckprobe fehlgeschlagen bei Schritt %d: "
                        "Positionsfehler %.5f m (max %.5f), Rotationsfehler "
                        "%.5f rad (max %.5f)"
                        % (i, pos_err, fk_tol_pos, rot_err, fk_tol_rot),
                        index=i,
                        reason="fk_roundtrip",
                    )

            solutions[i] = sol
            reference = sol

        return solutions

    # -- Diagnose ----------------------------------------------------------

    def conditioning(self, joints, eps=0.002):
        """Misst dq/dx in sechs Richtungen -- Indikator fuer Singularitaeten.

        Kleine Werte (Groessenordnung 1/Hebelarm, im Vortest 1.7-3.6 rad/m)
        bedeuten gute Konditionierung. Stark erhoehte Werte weisen auf
        Singularitaetsnaehe hin, wo kleines kartesisches Rauschen grosse
        Gelenkausschlaege erzeugt.

        Rueckgabe: dict Richtung -> dq/dx in rad/m.
        """
        base_quat = self.fk_quat(joints)
        out = {}
        for axis, name in enumerate(("X", "Y", "Z")):
            for sign in (1.0, -1.0):
                target = base_quat.copy()
                target[axis] += sign * eps
                try:
                    sol = self.ik(target, joints)
                except IKFailure:
                    out["%s%s" % ("+" if sign > 0 else "-", name)] = float("inf")
                    continue
                dq = float(np.max(np.abs(sol - np.asarray(joints, dtype=float))))
                out["%s%s" % ("+" if sign > 0 else "-", name)] = dq / eps
        return out

"""Episoden-Recorder: fuehrt eine geplante Bahn aus und zeichnet auf (AP 2.x).

Ablauf pro Zeitschritt i (bei CONTROL_RATE_HZ):

0. (einmalig) Pruefen, dass der Roboter an der Startstellung steht --
   sonst waere der erste Sollwert ein Sprung. Hinfahren ist Sache des
   Aufrufers (apps/record.py, ausserhalb der Aufzeichnung).
1. Takt abwarten (:class:`bc.sync.Pacer`).
2. Verrauschte Sollwinkel senden, mit Geschwindigkeit und Beschleunigung
   aus derselben Bahn (``servo_j(q, qd, qdd)``, trajectory.joint_derivatives).
3. Greiferbefehl absetzen, wenn die Bahn an dieser Stelle wechselt --
   die Totzeit ist als Dwell-Schritte bereits IN der Bahn (AP 2.1).
4. Zustand und juengste Frames abgreifen, Zeitstempel gegen das
   Latenzbudget bewerten (AP 1.3).
5. Asymmetrisch paaren (AP 2.4): Observation = echter (verrauschter)
   Zustand + Bilder; Action = IDEALE Sollwinkel des Schritts i+1.

Verwerf-Logik (AP 5.2): ausgeloester Stopp bricht sofort ab und markiert
die Episode als verworfen; zu viele Latenzbudget-Verletzungen ebenso.

``next.done`` ist nur im letzten Schritt einer VOLLSTAENDIGEN Episode True
-- das Ende der Bahn ist der Uebergabepunkt ans Hauptprogramm.
"""

import numpy as np

from . import config, dataset
from .ports import RobotError
from .sync import Pacer, evaluate
from .trajectory import joint_derivatives


class EpisodeRecorder(object):
    """Zeichnet Episoden entlang eines :class:`TrajectoryPlan` auf.

    ``captures``: Liste von Objekten mit ``.name`` und ``.latest()``
    (ThreadedCapture im Betrieb, DirectCapture in Tests).
    """

    def __init__(
        self,
        robot,
        captures,
        clock,
        rate_hz=config.CONTROL_RATE_HZ,
        max_skew=config.SYNC_MAX_SKEW_S,
        max_bad_ratio=config.SYNC_MAX_BAD_FRAME_RATIO,
    ):
        self.robot = robot
        self.captures = list(captures)
        self.clock = clock
        self.rate_hz = rate_hz
        self.max_skew = max_skew
        self.max_bad_ratio = max_bad_ratio

    def record(self, plan, metadata=None):
        """Fuehrt den Plan aus und liefert eine :class:`dataset.Episode`.

        Bricht bei ``robot.stop_requested`` sofort ab (Schutzstopp,
        AP 2.4/4.2) -- die Episode wird verworfen, nie stillschweigend
        gekuerzt.
        """
        n = len(plan)
        camera_names = [c.name for c in self.captures]

        start_error = float(
            np.max(np.abs(self.robot.read_state().joints - plan.joints_noisy[0]))
        )
        if start_error > config.START_POSE_TOL_RAD:
            raise RobotError(
                "Roboter steht nicht an der Startstellung der Bahn (Abweichung "
                "%.4f rad > %.4f rad) -- vorher move_to_joints ausfuehren."
                % (start_error, config.START_POSE_TOL_RAD)
            )
        velocity, acceleration = joint_derivatives(plan.joints_noisy, plan.rate_hz)

        steps = {
            "observation.state": [],
            "action": [],
            "aux.joints_ideal": [],
            "aux.joints_command": [],
            "aux.pose_ideal": [],
            "aux.pose_noisy": [],
            "aux.sync_ok": [],
            "next.done": [],
        }
        for name in camera_names:
            steps["observation.images.%s" % name] = []

        bad_frames = 0
        discard_reason = ""
        pacer = Pacer(self.clock, self.rate_hz).start()
        self.robot.activate_servo("position")
        try:
            prev_gripper = None
            for i in range(n):
                if self.robot.stop_requested:
                    discard_reason = "schutzstopp"
                    break

                t_target = pacer.tick()
                self.robot.servo_j(plan.joints_noisy[i], velocity[i], acceleration[i])

                g = plan.gripper[i]
                if prev_gripper is None or g != prev_gripper:
                    self.robot.gripper_command(g >= config.GRIPPER_THRESHOLD)
                prev_gripper = g

                state = self.robot.read_state()
                frames = {}
                timestamps = {"robot": state.t_joints}
                for cap in self.captures:
                    frame = cap.latest()
                    frames[cap.name] = frame
                    timestamps[cap.name] = None if frame is None else frame.timestamp

                report = evaluate(t_target, timestamps, self.max_skew)
                if not report.ok:
                    bad_frames += 1

                # Observation: der ECHTE (verrauschte) Zustand
                steps["observation.state"].append(
                    dataset.build_state(state.joints, state.tcp_quat, g)
                )
                for name in camera_names:
                    frame = frames[name]
                    if frame is None:
                        image = np.zeros(
                            (config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3), dtype=np.uint8
                        )
                    else:
                        image = _to_schema_size(frame.image)
                    steps["observation.images.%s" % name].append(image)

                # Action: die IDEALE Soll-Bahn bei t+1 (AP 2.4). Am letzten
                # Schritt wird auf den Endzustand geklemmt.
                j = min(i + 1, n - 1)
                steps["action"].append(
                    dataset.build_action(plan.joints_ideal[j], plan.gripper[j])
                )

                steps["aux.joints_ideal"].append(
                    np.asarray(plan.joints_ideal[i], dtype=np.float32)
                )
                steps["aux.joints_command"].append(
                    np.asarray(plan.joints_noisy[i], dtype=np.float32)
                )
                steps["aux.pose_ideal"].append(
                    dataset.canonical_pose(plan.poses_ideal[i]).astype(np.float32)
                )
                steps["aux.pose_noisy"].append(
                    dataset.canonical_pose(plan.poses_noisy[i]).astype(np.float32)
                )
                steps["aux.sync_ok"].append(np.array([report.ok], dtype=bool))
                steps["next.done"].append(np.array([False], dtype=bool))
        finally:
            self.robot.deactivate_servo()

        recorded = len(steps["observation.state"])
        if recorded == 0:
            raise RuntimeError(
                "Keine Schritte aufgezeichnet (Stopp vor dem ersten Takt?)"
            )

        bad_ratio = bad_frames / float(recorded)
        discarded = bool(discard_reason)
        if not discarded and bad_ratio > self.max_bad_ratio:
            discarded = True
            discard_reason = (
                "latenzbudget: %.0f%% der Frames ueber %d ms"
                % (100.0 * bad_ratio, int(self.max_skew * 1000))
            )
        if not discarded and recorded < n:
            discarded = True
            discard_reason = discard_reason or "unvollstaendig"

        arrays = {k: np.stack(v) for k, v in steps.items()}
        if recorded == n:
            arrays["next.done"][-1, 0] = True
        meta = dict(metadata or {})
        meta.update(
            {
                "planned_steps": n,
                "recorded_steps": recorded,
                "bad_frame_ratio": bad_ratio,
                "pacer_overruns": pacer.overruns,
                "noise_rejects": plan.rejects,
                "rate_hz": self.rate_hz,
            }
        )
        return dataset.Episode(
            arrays=arrays,
            metadata=meta,
            discarded=discarded,
            discard_reason=discard_reason,
        )


def _to_schema_size(image):
    """Bringt ein Bild auf die Schema-Groesse (AP 0.10: 240x320).

    Identische Vorverarbeitung fuer Aufzeichnung und Inferenz -- wer hier
    etwas aendert, aendert das Schema (config.SCHEMA_VERSION!).
    """
    h, w = config.IMAGE_HEIGHT, config.IMAGE_WIDTH
    if image.shape[:2] == (h, w):
        return np.ascontiguousarray(image, dtype=np.uint8)
    import cv2

    return np.ascontiguousarray(
        cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA), dtype=np.uint8
    )

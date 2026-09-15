"""Automatisierte Datenaufzeichnung mit Action Noise Injection (AP 2.2/2.4).

Ablauf je Episode:
    1. Wegpunkte bestimmen -- bei ``--sequence`` die Koordinaten FRISCH aus
       der Punkte-Datenbank (Touch-ups wirken sofort, bc/sequence.py).
    2. Ideale Soll-Bahn bauen (trajectory.py, LIN- und PTP-Segmente).
    3. Verrauschte Bahn samplen + per IK loesen + Kollision pruefen
       (noise.py, Rejection Sampling).
    4. Ausserhalb der Aufzeichnung an die Startstellung fahren, falls der Arm
       woanders steht (z. B. am Uebergabepunkt der vorigen Episode).
    5. Abfahren und aufzeichnen (recorder.py): Observation = echter,
       verrauschter Zustand; Action = ideale Sollwinkel bei t+1.
    6. Episode validieren und ablegen (dataset.py).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/record.py --sim --episodes 3 --out data_sim
    python apps/record.py --robot neura --sequence sequences/pick_to_station.json \\
        --episodes 3 --out data_vm

SICHERHEIT: Gegen den Neura-Adapter wird nur bewegt, wenn der Controller
``is_robot_in_simulation() == True`` meldet (VM und Anlage teilen eine IP).
Die reale Anlage braucht ``--real-robot`` UND eine Bestaetigung im
Terminal. Ohne Kameras-Hardware: ``--cameras sim`` (Platzhalterbilder, das
Datensatz-Schema bleibt identisch).
"""

import argparse
import json
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import capture, config, metrics
from bc.adapters import open_camera, open_robot
from bc.clock import RealClock, SimClock
from bc.collision import default_workspace
from bc.dataset import DatasetWriter
from bc.kinematics import Kinematics
from bc.noise import PlanRejected, generate_plan
from bc.ports import MotionRefused
from bc.recorder import EpisodeRecorder
from bc.sequence import SequenceError, load_sequence, resolve_sequence
from bc.trajectory import Waypoint, build_ideal_trajectory

#: Wortlaut, der fuer --real-robot eingetippt werden muss.
REAL_ROBOT_CONFIRMATION = "ANLAGE"


def demo_waypoints_sim(robot):
    """Demo-Griff um die Home-Pose des SimRobot (nur --sim ohne Ablauf).

    Anfahrt -> Absenken (approach) -> Greifen -> Heben -> Ablegen ->
    Oeffnen. Offsets klein genug, um sicher im Arbeitsraum des
    URDF-Modells zu bleiben.
    """
    home = robot.read_state().tcp_quat
    p = np.asarray(home, dtype=float)

    def at(dx, dy, dz, gripper, approach=False, name=""):
        pose = p.copy()
        pose[0] += dx
        pose[1] += dy
        pose[2] += dz
        return Waypoint(pose, gripper_closed=gripper, approach=approach, name=name)

    return [
        at(0.00, 0.00, 0.00, False, name="home"),
        at(0.06, 0.04, -0.02, False, name="anfahrt"),
        at(0.06, 0.04, -0.06, True, approach=True, name="greifen"),
        at(0.06, 0.04, 0.02, True, name="heben"),
        at(-0.04, -0.04, 0.02, True, name="transport"),
        at(-0.04, -0.04, -0.04, False, approach=True, name="ablegen"),
        at(-0.04, -0.04, 0.02, False, name="rueckzug"),
    ]


def waypoints_from_file(path):
    """Statische Wegpunkte aus apps/teach.py (nur LIN, ohne Gelenkstellung)."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return [
        Waypoint(
            np.asarray(wp["pose_quat"], dtype=float),
            gripper_closed=wp["gripper_closed"],
            approach=wp.get("approach", False),
            name=wp.get("name", ""),
        )
        for wp in data
    ]


def confirm_real_robot():
    print("\n!! --real-robot: Die Freigabe gilt der REALEN ANLAGE.")
    print("   Not-Aus in Reichweite? Arbeitsraum frei? Override geprueft?")
    answer = input("   Zum Fortfahren '%s' eintippen: " % REAL_ROBOT_CONFIRMATION)
    if answer.strip() != REAL_ROBOT_CONFIRMATION:
        raise SystemExit("Abgebrochen -- reale Anlage nicht freigegeben.")


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--sim", action="store_true", help="hardwarefreier Lauf (= --robot sim)")
    parser.add_argument("--robot", choices=("sim", "neura"), default=None)
    parser.add_argument(
        "--sequence", default=None,
        help="Ablaufdatei (bc/sequence.py): Punktnamen, Koordinaten live je Episode",
    )
    parser.add_argument("--waypoints", default=None, help="waypoints.json aus teach.py")
    parser.add_argument(
        "--cameras", choices=("auto", "sim", "real"), default="auto",
        help="auto: Sim-Kameras beim SimRobot, echte Kameras am Neura",
    )
    parser.add_argument(
        "--real-robot", action="store_true",
        help="reale Anlage freigeben (sonst nur Simulation), mit Bestaetigung",
    )
    parser.add_argument(
        "--override", type=float, default=config.DEFAULT_OVERRIDE,
        help="Geschwindigkeits-Override am Neura (0-1)",
    )
    parser.add_argument(
        "--noise-scale", type=float, default=1.0,
        help="Faktor auf die Rauschamplituden (0 = Referenzfahrt ohne Rauschen)",
    )
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--out", default="data_out")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--table-z", type=float, default=None,
        help="Tischhoehe in m fuer das Kollisionsmodell (Basis-KS). "
             "Default: 15 cm unter dem tiefsten Wegpunkt.",
    )
    args = parser.parse_args()
    if args.sequence and args.waypoints:
        parser.error("--sequence und --waypoints schliessen sich aus")
    return args


def main():
    args = parse_args()
    robot_kind = args.robot or ("sim" if args.sim else "neura")
    use_sim_robot = robot_kind == "sim"
    use_sim_cameras = args.cameras == "sim" or (args.cameras == "auto" and use_sim_robot)

    sequence = load_sequence(args.sequence) if args.sequence else None

    cam_cfgs = config.SIM_CAMERAS if use_sim_cameras else config.CAMERAS
    if not use_sim_cameras:
        # VOR jeder Hardware-Aktion: eine falsch konfigurierte Kamera faellt
        # im fertigen Datensatz nicht auf, deshalb hier hart nachfragen.
        warnings = config.check_cameras_configured(cam_cfgs)
        if warnings:
            print("\n!! KAMERA-KONFIGURATION UNBESTAETIGT:")
            for warning in warnings:
                print("   - %s" % warning)
            if input("Trotzdem aufzeichnen? [j/N] ").strip().lower() != "j":
                raise SystemExit("Abgebrochen.")

    if args.real_robot:
        if use_sim_robot:
            raise SystemExit("--real-robot ergibt mit dem SimRobot keinen Sinn.")
        confirm_real_robot()

    clock = SimClock() if use_sim_robot else RealClock()
    if use_sim_robot:
        robot = open_robot("sim", clock=clock, seed=args.seed).connect()
        robot_info = {"robot": "sim"}
    else:
        robot = open_robot("neura", allow_real=args.real_robot, override=args.override)
        try:
            robot.connect(power_on=True, ensure_automatic=True)
        except MotionRefused as exc:
            raise SystemExit("Abbruch vor jeder Bewegung: %s" % exc)
        robot_info = {
            "robot": "neura",
            "in_simulation": robot.in_simulation,
            "tool": robot.tool_name,
            "gripper_mode": robot.gripper_mode,
            "override": args.override,
            "timestamp_source": "host",
        }
        print(
            "Neura: is_robot_in_simulation()=%r, Tool=%r, Greifer=%s, Override=%.2f"
            % (robot.in_simulation, robot.tool_name, robot.gripper_mode, args.override)
        )
        if args.override < 1.0:
            # Gemessen VM 2026-09-14 (tools/check_sim_robot.py): Override 0.2
            # bremst den servo_j-Strom -- Nachlauf ~1 s statt 67 ms bei 1.0.
            # Die Observation liefe dann der gesendeten Bahn weit hinterher.
            # Das Tempo begrenzt der Planer (TRANSIT_SPEED_MS), nicht der
            # Override.
            print(
                "   WARNUNG: Override %.2f < 1.0 -- der Arm folgt servo_j stark "
                "verzoegert (VM: ~1 s Nachlauf bei 0.2)." % args.override
            )
        if robot.gripper_mode == "logged":
            print("   Kein Greifer konfiguriert -- Greiferbefehle werden nur protokolliert.")
        elif robot.gripper_mode == "unavailable":
            robot.close()
            raise SystemExit("Abbruch: reale Anlage ohne konfigurierten Greifer.")

    if sequence is None and args.waypoints is None and not use_sim_robot:
        robot.close()
        raise SystemExit("Am Neura sind --sequence oder --waypoints Pflicht.")

    if use_sim_cameras:
        from bc.adapters.cam_sim import SimCamera

        cams = [SimCamera(cfg, clock=clock, seed=args.seed) for cfg in cam_cfgs]
    else:
        cams = [open_camera(cfg) for cfg in cam_cfgs]
    # Sim-Kameras synchron abgreifen: sie liefern sofort, ein Capture-Thread
    # wuerde nur leer drehen -- und der Zeitstempel ist dann exakt der Abgriff.
    captures = capture.start_all(cams, threaded=not use_sim_cameras)

    kin = Kinematics(robot)
    writer = DatasetWriter(args.out, camera_names=[c.name for c in cams])
    recorder = EpisodeRecorder(robot, captures, clock)
    rng = np.random.default_rng(args.seed)
    # Ohne Ablaufdatei: feste Wegpunkte, EINMAL bestimmt (die Demo haengt an
    # der Startpose -- je Episode neu berechnet wuerde sie mitwandern).
    if args.waypoints:
        fixed_waypoints = waypoints_from_file(args.waypoints)
    elif sequence is None:
        fixed_waypoints = demo_waypoints_sim(robot)
    else:
        fixed_waypoints = None

    recorded = 0
    try:
        for ep in range(args.episodes):
            meta = dict(
                robot_info,
                mode="noise_injection" if args.noise_scale > 0 else "reference",
                noise_scale=args.noise_scale,
                episode_index=ep,
                seed=args.seed,
            )

            # 1. Wegpunkte -- beim Ablauf je Episode frisch abgefragt
            if sequence is not None:
                try:
                    resolved = resolve_sequence(sequence, robot)
                except (SequenceError, KeyError) as exc:
                    raise SystemExit("Ablauf nicht aufloesbar: %s" % exc)
                waypoints = resolved.waypoints
                meta.update(
                    sequence=sequence.name,
                    points=resolved.points,
                    skipped_points=resolved.skipped,
                )
                if resolved.skipped:
                    print("Episode %d: optionale Punkte fehlen: %s" % (ep, resolved.skipped))
            else:
                waypoints = fixed_waypoints

            # 2. Ideale Bahn und Kollisionsmodell
            ideal = build_ideal_trajectory(waypoints, fk=robot.fk)
            z_min = min(wp.pose_quat[2] for wp in waypoints)
            table_z = args.table_z if args.table_z is not None else z_min - 0.15
            workspace = default_workspace(table_height_m=table_z)
            if ep == 0:
                print("Kollisionsmodell: Tischhoehe z = %.3f m" % table_z)
                print(
                    "Soll-Bahn: %d Schritte (%.1f s bei %.0f Hz), %d Dwell-Schritte"
                    % (len(ideal), len(ideal) / ideal.rate_hz, ideal.rate_hz,
                       int(ideal.dwell_mask.sum()))
                )

            # 3. Verrauschte Bahn -- geseedet mit der geteachten Startstellung
            seed_joints = waypoints[0].joints
            if seed_joints is None:
                seed_joints = robot.read_state().joints
            try:
                plan = generate_plan(
                    kin, workspace, ideal, seed_joints, rng=rng, noise_scale=args.noise_scale
                )
            except PlanRejected as exc:
                print("Episode %d: Planung verworfen (%s)" % (ep, exc))
                continue

            # 4. Ausserhalb der Aufzeichnung an den Start
            start = plan.joints_noisy[0]
            away = float(np.max(np.abs(robot.read_state().joints - start)))
            if away > config.START_POSE_TOL_RAD:
                print("Episode %d: fahre an die Startstellung (%.3f rad entfernt)" % (ep, away))
                robot.move_to_joints(start)

            # 5./6. Aufzeichnen und ablegen
            log_start = len(getattr(robot, "gripper_log", []))
            episode = recorder.record(plan, metadata=meta)
            if getattr(robot, "gripper_log", None) is not None:
                episode.metadata["gripper_log"] = [
                    [t, closed] for t, closed in robot.gripper_log[log_start:]
                ]
            if use_sim_robot and not episode.discarded:
                episode.success = True  # Sim: Bahn vollstaendig == Erfolg
            idx = writer.append(episode)
            recorded += 1
            print(
                "Episode %d -> ep_%05d: %d Schritte, %s%s"
                % (
                    ep,
                    idx,
                    len(episode),
                    "VERWORFEN (%s)" % episode.discard_reason
                    if episode.discarded
                    else "ok",
                    ", %d Rausch-Rejects" % plan.rejects if plan.rejects else "",
                )
            )
    finally:
        for cap in captures:
            cap.stop()
        robot.close()

    print()
    if recorded:
        print(metrics.format_summary(metrics.summarize(args.out)))
    else:
        print("Keine Episode aufgezeichnet.")


if __name__ == "__main__":
    main()

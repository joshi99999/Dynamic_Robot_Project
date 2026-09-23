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
import sys
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config, metrics
from bc.adapters import (
    CAMERA_MODES,
    camera_configs,
    open_robot,
    resolve_camera_mode,
    start_cameras,
)
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
        "--cameras", choices=CAMERA_MODES, default="auto",
        help="auto: Sim-Kameras beim SimRobot, echte am Neura; wrist-real: echte "
             "Wrist-Kamera + Platzhalter-Szene (Labortest)",
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
        help="Hoechster Faktor auf die Rauschamplituden (0 = Referenzfahrt ohne Rauschen); "
             "je Episode wird daraus zufaellig skaliert (config.NOISE_EPISODE_SCALE_RANGE)",
    )
    parser.add_argument(
        "--noise-fixed", action="store_true",
        help="Rauschfaktor NICHT je Episode ziehen, sondern immer --noise-scale (Vergleichsfahrten)",
    )
    parser.add_argument(
        "--servo-rate", type=float, default=config.SERVO_RATE_HZ,
        help="servo_j-Senderate in Hz, Vielfaches von %.0f (Vergleichsfahrten; "
             "fuer Datensaetze den Default lassen)" % config.CONTROL_RATE_HZ,
    )
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--out", default="data_out")
    parser.add_argument(
        "--seed", type=int, default=None,
        help="Zufalls-Seed fuer das Rauschen. Default: neu gezogen und in den Metadaten "
             "abgelegt -- ein fester Seed wiederholt in jedem Aufruf DIESELBEN Bahnen "
             "(Befund 2026-09-16: zwei Laeufe mit Seed 0 hatten identisches Rauschen).",
    )
    parser.add_argument(
        "--table-z", type=float, default=None,
        help="Tischhoehe in m fuer das Kollisionsmodell (Basis-KS). "
             "Default: 15 cm unter dem tiefsten Wegpunkt.",
    )
    # Metadaten je Aufruf (AP 5.2) -- ein Aufruf = ein Block = eine
    # Objektlage (Objekt hinlegen, PRE_GRASP/PICK per Touch-up, aufzeichnen).
    meta = parser.add_argument_group("Metadaten (AP 5.2), gelten fuer alle Episoden des Aufrufs")
    meta.add_argument(
        "--block", default=None,
        help="Block-ID, z. B. B07 -- eine Objektlage. An der Anlage Pflicht.",
    )
    meta.add_argument(
        "--session", default=None,
        help="Session-ID (Default: Datum, z. B. 2026-09-17)",
    )
    meta.add_argument("--light", default=None, help="Beleuchtung, z. B. 'Decke an, Rollo zu'")
    meta.add_argument(
        "--camera-pose", default=None,
        help="Kamerapose-Variante der Szenenkamera, z. B. 'nominal' oder '+2cm x'",
    )
    meta.add_argument("--object", dest="object_note", default=None,
                      help="Objekt und Lage in Worten, z. B. 'Teil A, 30 Grad gedreht, links'")
    meta.add_argument(
        "--object-points", default="PICK,PRE_GRASP",
        help="Punkte, deren geteachte Lage als Objektlage abgelegt wird (Komma-Liste)",
    )
    args = parser.parse_args()
    if args.sequence and args.waypoints:
        parser.error("--sequence und --waypoints schliessen sich aus")
    if args.seed is None:
        args.seed = int(np.random.SeedSequence().entropy % (2**31))
    return args


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)  # Fortschritt auch in Logdateien
    args = parse_args()
    robot_kind = args.robot or ("sim" if args.sim else "neura")
    use_sim_robot = robot_kind == "sim"
    camera_mode = resolve_camera_mode(args.cameras, use_sim_robot)
    use_sim_cameras = camera_mode == "sim"

    sequence = load_sequence(args.sequence) if args.sequence else None

    cam_cfgs = camera_configs(camera_mode)
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
        if not args.block:
            raise SystemExit(
                "--block fehlt: an der Anlage gehoert jede Aufzeichnung zu einem Block "
                "(Objektlage), sonst sind die Ablationen nicht auswertbar (AP 5.2)."
            )
        confirm_real_robot()

    # Echte Kameras stempeln mit der Host-Uhr -- dann auch der SimRobot.
    clock = SimClock() if use_sim_robot and use_sim_cameras else RealClock()
    if use_sim_robot:
        robot = open_robot("sim", clock=clock, seed=args.seed).connect()
        robot_info = {"robot": "sim", "in_simulation": True}
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

    try:
        captures, cam_cfgs = start_cameras(camera_mode, clock, seed=args.seed)
    except Exception:
        robot.close()
        raise

    kin = Kinematics(robot)
    writer = DatasetWriter(args.out, camera_names=[c.name for c in cam_cfgs])
    recorder = EpisodeRecorder(robot, captures, clock, servo_rate_hz=args.servo_rate)
    rng = np.random.default_rng(args.seed)
    print(
        "Lauf-Einstellungen: servo_j %.0f Hz, Rauschen %s x%.2f, Seed %d, Ablauf %s"
        % (args.servo_rate, "fest" if args.noise_fixed else "je Episode zufaellig bis",
           args.noise_scale, args.seed, args.sequence or args.waypoints or "Demo")
    )
    # Ohne Ablaufdatei: feste Wegpunkte, EINMAL bestimmt (die Demo haengt an
    # der Startpose -- je Episode neu berechnet wuerde sie mitwandern).
    if args.waypoints:
        fixed_waypoints = waypoints_from_file(args.waypoints)
    elif sequence is None:
        fixed_waypoints = demo_waypoints_sim(robot)
    else:
        fixed_waypoints = None

    planner_meta = {
        "transit_speed_ms": config.TRANSIT_SPEED_MS,
        "approach_speed_ms": config.APPROACH_SPEED_MS,
        "ptp_joint_speed_rads": config.PTP_JOINT_SPEED_RADS,
        "segment_ramp_s": config.SEGMENT_RAMP_S,
        "gripper_dwell_s": config.GRIPPER_DWELL_S,
    }
    object_points = [p.strip() for p in args.object_points.split(",") if p.strip()]
    if not use_sim_robot and not args.block:
        print("Hinweis: ohne --block (Objektlage) -- an der Anlage Pflicht (AP 5.2).")

    recorded = 0
    try:
        for ep in range(args.episodes):
            # Rauschstaerke je Episode (AP 2.4, Auswertung 2026-09-16): gezogen
            # aus NOISE_EPISODE_SCALE_RANGE -- mal fast ideal, mal volle
            # Auslenkung. --noise-fixed nimmt immer genau --noise-scale.
            if args.noise_fixed:
                episode_scale = args.noise_scale
            else:
                lo, hi = config.NOISE_EPISODE_SCALE_RANGE
                episode_scale = args.noise_scale * float(rng.uniform(lo, hi))
            meta = dict(
                robot_info,
                mode="noise_injection" if args.noise_scale > 0 else "reference",
                noise_scale=episode_scale,
                noise_scale_max=args.noise_scale,
                noise_scale_mode="fest" if args.noise_fixed else "je Episode zufaellig",
                noise_amplitude_m=config.NOISE_TRANS_AMPLITUDE_M,
                noise_rot_amplitude_rad=config.NOISE_ROT_AMPLITUDE_RAD,
                episode_index=ep,
                seed=args.seed,
                recorded_at=datetime.now().isoformat(timespec="seconds"),
                sequence_file=args.sequence,
                session=args.session or datetime.now().strftime("%Y-%m-%d"),
                block=args.block,
                light=args.light,
                camera_pose=args.camera_pose,
                object_note=args.object_note,
                cameras={c.name: c.backend for c in cam_cfgs},
                # Mitgelerntes Tempo (AP 2.6): der Export verlangt, dass diese
                # Werte ueber alle Episoden eines Datensatzes gleich sind.
                planner=planner_meta,
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
                    # Objektlage = die per Touch-up geteachten Greifpunkte
                    object_pose={
                        name: resolved.points[name]["pose_quat"]
                        for name in object_points
                        if name in resolved.points
                    },
                )
                if resolved.skipped:
                    print("Episode %d: optionale Punkte fehlen: %s" % (ep, resolved.skipped))
            else:
                waypoints = fixed_waypoints

            # 2. Ideale Bahn und Kollisionsmodell
            ideal = build_ideal_trajectory(waypoints, fk=robot.fk)
            meta["blend_m"] = {
                wp.name or str(k): r
                for k, (wp, r) in enumerate(zip(waypoints, ideal.blend_applied))
                if r > 0
            }
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
                    kin, workspace, ideal, seed_joints, rng=rng, noise_scale=episode_scale
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

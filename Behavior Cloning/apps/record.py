"""Automatisierte Datenaufzeichnung mit Action Noise Injection (AP 2.2/2.4).

Ablauf je Episode:
    1. Ideale Soll-Bahn aus den Wegpunkten bauen (trajectory.py).
    2. Verrauschte Bahn samplen + per IK loesen + Kollision pruefen
       (noise.py, Rejection Sampling).
    3. Abfahren und aufzeichnen (recorder.py): Observation = echter,
       verrauschter Zustand; Action = ideale Sollwinkel bei t+1.
    4. Episode validieren und ablegen (dataset.py).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/record.py --sim --episodes 3 --out data_sim
    python apps/record.py --robot neura --waypoints wp.json --out data_run1

Der --sim-Lauf ist der hardwarefreie End-to-End-Nachweis der Kette
(AP 0.5 Stufe 3, im Rahmen der Platzhalter-Kameras aus AP 0.10).
"""

import argparse

import _bootstrap  # noqa: F401

import numpy as np

from bc import capture, config, geometry, metrics
from bc.adapters import open_camera, open_robot
from bc.clock import RealClock, SimClock
from bc.collision import default_workspace
from bc.dataset import DatasetWriter
from bc.kinematics import Kinematics
from bc.noise import PlanRejected, generate_plan
from bc.recorder import EpisodeRecorder
from bc.trajectory import Waypoint, build_ideal_trajectory


def demo_waypoints_sim(robot):
    """Demo-Griff um die Home-Pose des SimRobot (nur --sim).

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
    import json
    from pathlib import Path

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true", help="hardwarefreier Lauf")
    parser.add_argument("--robot", choices=("sim", "neura"), default=None)
    parser.add_argument("--waypoints", default=None, help="waypoints.json aus teach.py")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--out", default="data_out")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--table-z", type=float, default=None,
        help="Tischhoehe in m fuer das Kollisionsmodell (Basis-KS). "
             "Default: 15 cm unter dem tiefsten Wegpunkt.",
    )
    args = parser.parse_args()

    robot_kind = args.robot or ("sim" if args.sim else "neura")
    use_sim = robot_kind == "sim"

    clock = SimClock() if use_sim else RealClock()
    robot = open_robot("sim", clock=clock, seed=args.seed) if use_sim else open_robot("neura")
    robot.connect()

    if args.waypoints:
        waypoints = waypoints_from_file(args.waypoints)
    elif use_sim:
        waypoints = demo_waypoints_sim(robot)
    else:
        raise SystemExit("An der Anlage sind --waypoints aus apps/teach.py Pflicht.")

    # Kollisionsmodell: Tisch unterhalb des tiefsten Wegpunkts (AP 2.4).
    z_values = [wp.pose_quat[2] for wp in waypoints]
    table_z = args.table_z if args.table_z is not None else min(z_values) - 0.15
    workspace = default_workspace(table_height_m=table_z)
    print("Kollisionsmodell: Tischhoehe z = %.3f m" % table_z)

    cam_cfgs = config.SIM_CAMERAS if use_sim else config.CAMERAS
    if use_sim:
        from bc.adapters.cam_sim import SimCamera

        cams = [SimCamera(cfg, clock=clock, seed=args.seed) for cfg in cam_cfgs]
    else:
        cams = [open_camera(cfg) for cfg in cam_cfgs]
    captures = capture.start_all(cams, threaded=not use_sim)

    kin = Kinematics(robot)
    writer = DatasetWriter(args.out, camera_names=[c.name for c in cams])
    recorder = EpisodeRecorder(robot, captures, clock)
    rng = np.random.default_rng(args.seed)

    ideal = build_ideal_trajectory(waypoints)
    print(
        "Soll-Bahn: %d Schritte (%.1f s bei %.0f Hz), %d Dwell-Schritte"
        % (
            len(ideal),
            len(ideal) / ideal.rate_hz,
            ideal.rate_hz,
            int(ideal.dwell_mask.sum()),
        )
    )

    recorded = 0
    try:
        for ep in range(args.episodes):
            seed_joints = robot.read_state().joints
            try:
                plan = generate_plan(kin, workspace, ideal, seed_joints, rng=rng)
            except PlanRejected as exc:
                print("Episode %d: Planung verworfen (%s)" % (ep, exc))
                continue

            episode = recorder.record(
                plan,
                metadata={
                    "mode": "noise_injection",
                    "robot": robot_kind,
                    "episode_index": ep,
                    "seed": args.seed,
                },
            )
            if use_sim and not episode.discarded:
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
    print(metrics.format_summary(metrics.summarize(args.out)))


if __name__ == "__main__":
    main()

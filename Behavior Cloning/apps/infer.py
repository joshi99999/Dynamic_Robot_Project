"""Live-Inferenz (AP 4) -- GERUEST mit lauffaehiger Verdrahtung.

Die Schleife selbst steht und laesst sich mit der HoldPolicy hardwarefrei
gegen den SimRobot durchspielen (Schema, Timing, Watchdog, Not-Halt).
Die echte Diffusion Policy folgt mit AP 3 (siehe apps/train.py).

Aufbau der Schleife (AP 4.1/4.2):
    * 15 Hz (identisch zur Aufzeichnung -- NIE aendern, AP 1.3/2.6)
    * Action Chunking ueber policy.ChunkExecutor
    * Watchdog-Thread mit Geofence prueft unabhaengig von der Inferenz
    * Greifer binaer ueber die Schema-Schwelle (dataset.gripper_from_action)

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/infer.py --sim --steps 45
"""

import argparse

import _bootstrap  # noqa: F401

import numpy as np

from bc import capture, config, dataset
from bc.adapters import open_robot
from bc.clock import RealClock, SimClock
from bc.collision import default_workspace
from bc.policy import ChunkExecutor, HoldPolicy
from bc.recorder import _to_schema_size
from bc.safety import Watchdog, geofence_check
from bc.sync import Pacer


def build_observation(robot, captures):
    """Baut die Beobachtung EXAKT wie der Recorder (AP 1.5.1)."""
    state = robot.read_state()
    obs = {
        "observation.state": dataset.build_state(
            state.joints,
            state.tcp_quat,
            config.GRIPPER_CLOSED if state.gripper_closed else config.GRIPPER_OPEN,
        )
    }
    for cap in captures:
        frame = cap.latest()
        obs["observation.images.%s" % cap.name] = (
            np.zeros((config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3), dtype=np.uint8)
            if frame is None
            else _to_schema_size(frame.image)
        )
    return obs


def inference_loop(robot, captures, policy, clock, steps, table_z):
    executor = ChunkExecutor(policy)
    executor.reset()

    workspace = default_workspace(table_height_m=table_z)
    watchdog = Watchdog(robot, [geofence_check(robot, workspace)], clock)
    watchdog.start()

    pacer = Pacer(clock).start()
    robot.activate_servo("position")
    executed = 0
    try:
        for _ in range(steps):
            if robot.stop_requested:
                print("Stopp angefordert -- Inferenz beendet (%s)" % watchdog.tripped)
                break
            pacer.tick()
            obs = build_observation(robot, captures)
            action = executor.next_action(obs)
            # Nur SimRobot (siehe main): der Neura-Adapter verlangt zusaetzlich
            # Geschwindigkeit/Beschleunigung -- mit AP 4 aus dem Action-Chunk
            # ableiten (trajectory.joint_derivatives).
            robot.servo_j(action[:6])
            robot.gripper_command(dataset.gripper_from_action(action))
            executed += 1
    finally:
        watchdog.stop()
        robot.deactivate_servo()
    return executed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sim", action="store_true")
    parser.add_argument("--steps", type=int, default=45)
    parser.add_argument("--checkpoint", default=None, help="(spaeter) Policy-Gewichte")
    parser.add_argument("--table-z", type=float, default=None)
    args = parser.parse_args()

    if not args.sim:
        raise SystemExit(
            "Nur --sim ist in dieser Ausbaustufe lauffaehig (HoldPolicy). "
            "Echte Inferenz folgt mit AP 3/4."
        )

    clock = SimClock()
    robot = open_robot("sim", clock=clock).connect()
    from bc.adapters.cam_sim import SimCamera

    captures = capture.start_all(
        [SimCamera(cfg, clock=clock) for cfg in config.SIM_CAMERAS], threaded=False
    )

    table_z = (
        args.table_z
        if args.table_z is not None
        else float(robot.read_state().tcp_quat[2]) - 0.3
    )
    policy = HoldPolicy()
    try:
        executed = inference_loop(robot, captures, policy, clock, args.steps, table_z)
    finally:
        for cap in captures:
            cap.stop()
        robot.close()
    print("Inferenzschleife: %d/%d Schritte ausgefuehrt." % (executed, args.steps))


if __name__ == "__main__":
    main()

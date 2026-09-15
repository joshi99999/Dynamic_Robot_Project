"""Wegpunkte teachen (AP 2.2): Gamepad-Jog -> waypoints.json.

Das Gamepad dient AUSSCHLIESSLICH dem Teachen -- aufgezeichnet wird spaeter
automatisiert mit Rauscheinspielung (apps/record.py). Gespeichert werden
Pose (Quaternion), Greiferzustand und Name je Wegpunkt.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/teach.py --robot sim                 # hardwarefrei testen
    python apps/teach.py --robot neura --out wp.json # an der Anlage

Hinweis Anlage: Der Jog laeuft ueber das Servo-Interface (IK auf die
verschobene Zielpose). Vorher Not-Halt in Reichweite (AP 4.2).
"""

import argparse
import json
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config
from bc.adapters import open_robot
from bc.kinematics import IKFailure, Kinematics


def save_waypoints(path, waypoints):
    data = [
        {
            "name": wp["name"],
            "pose_quat": [float(v) for v in wp["pose_quat"]],
            "gripper_closed": bool(wp["gripper_closed"]),
            "approach": bool(wp.get("approach", False)),
        }
        for wp in waypoints
    ]
    Path(path).write_text(
        json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8"
    )


def load_waypoints(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def teach_loop(robot, teleop, poll_sleep=0.02, clock=None):
    """Kernschleife, testbar mit ScriptedTeleop + SimRobot.

    Rueckgabe: Liste gespeicherter Wegpunkte (dicts).
    """
    kin = Kinematics(robot)
    waypoints = []
    gripper_closed = robot.gripper_closed

    state = robot.read_state()
    target_pose = np.asarray(state.tcp_quat, dtype=float).copy()
    robot.activate_servo("position")
    try:
        while True:
            event = teleop.poll()
            if event is None:
                if clock is not None:
                    clock.sleep(poll_sleep)
                continue

            if event.kind == "quit":
                break

            if event.kind == "jog":
                candidate = target_pose.copy()
                candidate[:3] += np.asarray(event.value, dtype=float)
                try:
                    joints = kin.ik(candidate, robot.read_state().joints)
                except IKFailure as exc:
                    print("Jog nicht erreichbar: %s" % exc)
                    continue
                # Jog = Sprung auf ein neues Ziel, an dem der Arm stehen
                # bleibt: Geschwindigkeit/Beschleunigung 0 sind hier korrekt.
                hold = [0.0] * robot.dof
                robot.servo_j(joints, hold, hold)
                target_pose = candidate

            elif event.kind == "gripper":
                gripper_closed = bool(event.value)
                robot.gripper_command(gripper_closed)
                print("Greifer: %s" % ("zu" if gripper_closed else "auf"))

            elif event.kind == "save":
                state = robot.read_state()
                wp = {
                    "name": "wp%02d" % len(waypoints),
                    "pose_quat": np.asarray(state.tcp_quat, dtype=float),
                    "gripper_closed": gripper_closed,
                }
                waypoints.append(wp)
                print(
                    "Gespeichert %s: %s (Greifer %s)"
                    % (
                        wp["name"],
                        np.round(wp["pose_quat"][:3], 4).tolist(),
                        "zu" if gripper_closed else "auf",
                    )
                )
    finally:
        robot.deactivate_servo()
    return waypoints


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", choices=("sim", "neura"), default="sim")
    parser.add_argument("--out", default="waypoints.json")
    parser.add_argument(
        "--real-robot", action="store_true",
        help="reale Anlage freigeben (sonst verweigert der Adapter jede Bewegung)",
    )
    args = parser.parse_args()

    if args.robot == "neura":
        # Sicherheitssperre im Adapter: ohne is_robot_in_simulation() == True
        # nur mit ausdruecklicher Freigabe (VM und Anlage teilen eine IP).
        robot = open_robot("neura", allow_real=args.real_robot)
        robot.connect(power_on=True)
    else:
        robot = open_robot("sim")
        robot.connect()

    if args.robot == "sim":
        print("SIM-Modus: Ereignisse kommen aus einem Demo-Skript.")
        from bc.adapters.teleop_script import teach_script

        jog = np.array([0.01, 0.0, 0.0])
        teleop = teach_script([[jog], [jog], []])
    else:
        from bc.adapters.teleop_gamepad import GamepadTeleop

        teleop = GamepadTeleop().open()
        print("Gamepad bereit: A=speichern, B=Greifer, Start=beenden.")

    try:
        waypoints = teach_loop(robot, teleop)
    finally:
        teleop.close()
        robot.close()

    if waypoints:
        save_waypoints(args.out, waypoints)
        print("%d Wegpunkte -> %s" % (len(waypoints), args.out))
    else:
        print("Keine Wegpunkte gespeichert.")


if __name__ == "__main__":
    main()

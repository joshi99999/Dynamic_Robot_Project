"""Live-Inferenz (AP 4): die trainierte Policy faehrt selbst.

Aufbau der Schleife (AP 4.1/4.2), in jedem 15-Hz-Takt:
    1. Beobachtung EXAKT wie der Recorder bauen (dataset.build_state,
       Bildgroesse 240x320) -- AP 1.5.1.
    2. Policy: jede Beobachtung in die Historie, alle ``--replan`` Takte neu
       vorhersagen, ueberlappende Chunks mitteln (policy.ChunkEnsembler).
    3. Pruefen: Ziel nicht weiter als SERVO_MAX_TARGET_GAP_RAD von der
       Ist-Stellung (policy.check_action).
    4. Ziel fuer t+1 mit SERVO_RATE_HZ interpoliert anfahren (servo.py, wie
       im Recorder). Jeder Zwischenschritt laeuft im Adapter durch den
       Sprung- und Geschwindigkeitsfilter (servo.ServoGuard).
    5. Greiferbefehl bei Wechsel (Schwelle 0.5).
    Geofence-Watchdog parallel (echte Uhr: eigener Thread; SimClock: je Takt).

Vor jeder Fahrt geht es ausserhalb der Policy an die Startstellung der
Aufzeichnung (bc_policy.json), wie beim Reset zwischen Episoden (AP 5.2).
Die Fahrt endet am Uebergabepunkt (Endstellung erreicht und ruhig), nach
``--max-steps`` oder bei Stopp.

Gleichheit mit der Aufzeichnung wird erzwungen, nicht nur empfohlen:
Schema, Rate, Kameras, servo_j-Rate und Override muessen zu bc_policy.json
passen (AP 2.6 -- das Tempo und das Folgeverhalten sind mitgelernt).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/infer.py --sim --checkpoint checkpoints/sim/policy --episodes 5
    python apps/infer.py --robot neura --cameras sim --checkpoint checkpoints/vm/policy
    python apps/infer.py --sim --hold --steps 45        # Verdrahtung ohne Modell

SICHERHEIT: Am Neura nur mit is_robot_in_simulation() == True, die reale
Anlage braucht --real-robot und eine Bestaetigung. Software-Stopp ersetzt
keinen Hardware-Not-Aus (AP 4.2).
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config, dataset, metrics
from bc.adapters import (
    CAMERA_MODES,
    camera_configs,
    open_robot,
    resolve_camera_mode,
    start_cameras,
)
from bc.clock import RealClock, SimClock
from bc.collision import default_workspace
from bc.policy import ChunkEnsembler, ChunkExecutor, HoldPolicy, PredictionTooSlow, check_action
from bc.ports import MotionRefused, RobotError
from bc.recorder import _to_schema_size
from bc.safety import Watchdog, geofence_check
from bc.servo import ServoGuard, ServoInterpolator, TargetLimiter
from bc.sync import Pacer

#: Wortlaut fuer --real-robot (wie apps/record.py).
REAL_ROBOT_CONFIRMATION = "ANLAGE"

#: Endstellung gilt als erreicht, wenn alle Gelenke naeher als dies sind ...
END_TOL_RAD = 0.02
#: ... und das so viele Takte in Folge bleibt.
END_HOLD_STEPS = 5


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
    return obs, state


def run_episode(robot, captures, executor, clock, max_steps, workspace,
                end_joints=None, end_gripper=None, min_steps=0, threaded_watchdog=True,
                progress=print):
    """Eine Policy-Fahrt. Rueckgabe: Log-dict (Arrays je Takt + Zusammenfassung).

    Ende = Endstellung +-END_TOL_RAD UND (falls bekannt) Greiferzustand wie am
    Ende der Aufzeichnung, END_HOLD_STEPS Takte lang, fruehestens ab
    ``min_steps``. Stellung allein ist nicht eindeutig: dieselbe Stellung kann
    in einer Bahn mehrfach vorkommen (Befund 2026-09-17, Sim-Demo).
    """
    executor.reset()
    guard = getattr(robot, "servo_guard", None) or ServoGuard()
    watchdog = Watchdog(robot, [geofence_check(robot, workspace)], clock)
    if threaded_watchdog:
        watchdog.start()

    log = {k: [] for k in ("t", "joints", "tcp", "action", "gripper_cmd", "predict_s",
                           "delay_steps", "ensemble_spread", "ensemble_count")}
    # Takt 0 VOR dem Aktivieren des Servo-Interface: die erste Vorhersage
    # (bei asynchroner Vorhersage wird auf sie gewartet) haelt so keinen
    # laufenden Servostrom an. Der Arm steht, die Beobachtung bleibt gueltig.
    obs, state = build_observation(robot, captures)
    first = (obs, state, executor.next_action(obs))
    interp = ServoInterpolator().reset(state.joints)
    limiter = TargetLimiter().reset(state.joints)
    robot.activate_servo("position")
    pacer = Pacer(clock, config.SERVO_RATE_HZ).start()  # nach der Aktivierung
    stop_reason, at_end, prev_gripper = None, 0, None
    try:
        q, v, a = interp.hold()
        pacer.tick()
        robot.servo_j(q, v, a)
        for step in range(max_steps):
            if not threaded_watchdog and watchdog.run_once() is not None:
                stop_reason = "geofence: %s" % watchdog.tripped
                break
            if robot.stop_requested:
                stop_reason = "stopp: %s" % (watchdog.tripped or "angefordert")
                break
            before = executor.predictions
            if first is not None:
                (obs, state, action), first = first, None
                before = 0
            else:
                obs, state = build_observation(robot, captures)
                action = executor.next_action(obs)
            predicted = executor.predictions > before
            check_action(action, state.joints, guard)

            # Weiche Stufe: Zielaenderung je Takt begrenzen (servo.TargetLimiter);
            # der ServoGuard im Adapter bleibt die harte Stufe.
            for q, v, a in interp.window(limiter.limit(action[:6])):
                if robot.stop_requested:
                    break
                pacer.tick()
                robot.servo_j(q, v, a)
            if robot.stop_requested:
                stop_reason = "stopp: %s" % (watchdog.tripped or "angefordert")
                break
            gripper = dataset.gripper_from_action(action)
            if gripper != prev_gripper:
                robot.gripper_command(gripper)
                prev_gripper = gripper

            log["t"].append(clock.now())
            log["joints"].append(state.joints)
            log["tcp"].append(np.asarray(state.tcp_quat, dtype=float))
            log["action"].append(action)
            log["gripper_cmd"].append(float(gripper))
            predict_s = getattr(executor, "last_predict_s", None) if predicted else None
            log["predict_s"].append(np.nan if predict_s is None else predict_s)
            log["delay_steps"].append(getattr(executor, "last_delay_steps", 0) if predicted else np.nan)
            log["ensemble_spread"].append(getattr(executor, "last_spread", 0.0))
            log["ensemble_count"].append(getattr(executor, "last_count", 1))

            if end_joints is not None:
                near = float(np.max(np.abs(np.asarray(state.joints) - end_joints))) < END_TOL_RAD
                if end_gripper is not None:
                    near = near and bool(state.gripper_closed) == (end_gripper > config.GRIPPER_THRESHOLD)
                near = near and step >= min_steps
                at_end = at_end + 1 if near else 0
                if at_end >= END_HOLD_STEPS:
                    stop_reason = "endstellung"
                    break
            if progress and step % 15 == 0:
                progress("  Takt %4d  Spread %.4f rad  Vorhersage %s"
                         % (step, log["ensemble_spread"][-1],
                            "-" if predict_s is None else "%.1f ms" % (1e3 * predict_s)))
        else:
            stop_reason = "max_steps"
    except RobotError as exc:
        stop_reason = "abgelehnt: %s" % exc
        robot.emergency_stop()
    except PredictionTooSlow as exc:
        stop_reason = "vorhersage zu langsam: %s" % exc
        robot.emergency_stop()
    finally:
        watchdog.stop()
        robot.deactivate_servo()

    # Letzten Zustand nachtragen (Ziel des letzten Takts ist dort erreicht)
    final = robot.read_state()
    log["final_tcp"] = np.asarray(final.tcp_quat, dtype=float)
    log["final_joints"] = np.asarray(final.joints, dtype=float)
    arrays = {k: np.asarray(v, dtype=float) for k, v in log.items()}
    arrays["pacer_overruns"] = pacer.overruns
    arrays["stop_reason"] = stop_reason
    arrays["reached_end"] = stop_reason == "endstellung"
    arrays["guard_rejections"] = guard.rejections
    arrays["limiter_clamped"] = limiter.clamped
    return arrays


def confirm_real_robot():
    print("\n!! --real-robot: Die POLICY faehrt die REALE ANLAGE.")
    print("   Not-Aus in Reichweite? Arbeitsraum frei? Override geprueft?")
    answer = input("   Zum Fortfahren '%s' eintippen: " % REAL_ROBOT_CONFIRMATION)
    if answer.strip() != REAL_ROBOT_CONFIRMATION:
        raise SystemExit("Abgebrochen -- reale Anlage nicht freigegeben.")


def check_against_recording(info, args, robot_kind, cam_cfgs):
    """Erzwingt Gleichheit mit der Aufzeichnung (AP 1.5.1 / 2.6)."""
    problems = []
    camera_names = [c.name for c in cam_cfgs]
    backends = (info.get("recording") or {}).get("camera_backends")
    if backends:
        now = {c.name: c.backend for c in cam_cfgs if c.name in info["cameras"]}
        differ = {k: (backends.get(k), v) for k, v in now.items() if backends.get(k) != v}
        if differ:
            problems.append("Kamera-Backends Aufzeichnung/jetzt: %s (--cameras pruefen)" % differ)
    if info["schema_version"] != config.SCHEMA_VERSION:
        problems.append("Schema v%s, Code v%s" % (info["schema_version"], config.SCHEMA_VERSION))
    if float(info["rate_hz"]) != config.CONTROL_RATE_HZ:
        problems.append("Rate %.1f Hz, Code %.1f Hz" % (info["rate_hz"], config.CONTROL_RATE_HZ))
    if list(info["image_size_hw"]) != [config.IMAGE_HEIGHT, config.IMAGE_WIDTH]:
        problems.append("Bildgroesse %s" % info["image_size_hw"])
    missing = [c for c in info["cameras"] if c not in camera_names]
    if missing:
        problems.append("Kameras %s fehlen (vorhanden %s)" % (missing, camera_names))
    rec = info.get("recording", {})
    if rec.get("servo_rate_hz") is not None and float(rec["servo_rate_hz"]) != config.SERVO_RATE_HZ:
        problems.append("servo_j-Rate Aufzeichnung %.0f Hz, jetzt %.0f Hz"
                        % (rec["servo_rate_hz"], config.SERVO_RATE_HZ))
    if robot_kind == "neura" and rec.get("override") is not None and float(rec["override"]) != args.override:
        problems.append("Override Aufzeichnung %.2f, jetzt %.2f" % (rec["override"], args.override))
    if rec.get("robot") and rec["robot"] != robot_kind:
        print("Hinweis: aufgezeichnet mit Roboter '%s', jetzt '%s'." % (rec["robot"], robot_kind))
    return problems


def resolve_checkpoint(path):
    path = Path(path)
    if (path / "policy" / "bc_policy.json").is_file():
        return path / "policy"
    return path


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--checkpoint", help="Policy-Ordner (apps/train.py: <out>/policy)")
    parser.add_argument("--hold", action="store_true",
                        help="HoldPolicy statt Modell (Verdrahtungstest)")
    parser.add_argument("--sim", action="store_true", help="= --robot sim")
    parser.add_argument("--robot", choices=("sim", "neura"), default=None)
    parser.add_argument("--cameras", choices=CAMERA_MODES, default="auto",
                        help="wie apps/record.py -- muss zur Aufzeichnung passen")
    parser.add_argument("--real-robot", action="store_true")
    parser.add_argument("--override", type=float, default=None,
                        help="Neura-Override; Default und Pflichtwert: der der Aufzeichnung")
    parser.add_argument("--realtime", action="store_true",
                        help="SimRobot mit echter Uhr (Latenz der Vorhersage wirkt wie an der Anlage)")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--steps", "--max-steps", dest="max_steps", type=int, default=None,
                        help="Hoechstzahl Takte je Fahrt (Default 1.5 x laengste Episode)")
    parser.add_argument("--replan", type=int, default=None,
                        help="alle N Takte neu vorhersagen (Default aus bc_policy.json)")
    parser.add_argument("--decay", type=float, default=None,
                        help="Gewichtung beim Mitteln, exp(-k*Alter) (Default aus bc_policy.json)")
    parser.add_argument("--no-ensemble", action="store_true",
                        help="Chunks nicht mitteln, sondern je --replan Takte abarbeiten (Vergleich)")
    parser.add_argument("--inference-steps", type=int, default=None, help="DDIM-Schritte")
    parser.add_argument("--sync-predict", action="store_true",
                        help="Vorhersage im Takt statt im eigenen Thread (Default nur bei SimClock)")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--table-z", type=float, default=None)
    parser.add_argument("--no-start-move", action="store_true",
                        help="nicht vorher an die Startstellung fahren")
    parser.add_argument("--out", default=None,
                        help="Ordner fuer Fahrtprotokolle (Default: <checkpoint>/rollouts/<Zeit>)")
    args = parser.parse_args()
    if not args.hold and not args.checkpoint:
        parser.error("--checkpoint fehlt (oder --hold fuer den Verdrahtungstest)")
    return args


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)  # Fortschritt auch in Logdateien
    args = parse_args()
    robot_kind = args.robot or ("sim" if args.sim else "neura")
    use_sim_robot = robot_kind == "sim"
    camera_mode = resolve_camera_mode(args.cameras, use_sim_robot)
    use_sim_cameras = camera_mode == "sim"
    cam_cfgs = camera_configs(camera_mode)

    info, policy = None, None
    if args.hold:
        policy = HoldPolicy(horizon=config.POLICY_N_ACTION_STEPS)
        info = {"start_joints": None, "end_joints": None, "episode_length_max": 45,
                "inference_defaults": {}, "reference": {}, "ideal_z_min": None,
                "recording": {}}
    else:
        from bc.diffusion import DiffusionPolicyRunner, read_policy_info

        checkpoint = resolve_checkpoint(args.checkpoint)
        info = read_policy_info(checkpoint)
        rec_override = info.get("recording", {}).get("override")
        if args.override is None:
            args.override = float(rec_override) if rec_override is not None else config.DEFAULT_OVERRIDE
        problems = check_against_recording(info, args, robot_kind, cam_cfgs)
        if problems:
            raise SystemExit("Policy passt nicht zu dieser Ausfuehrung:\n  - " + "\n  - ".join(problems))
    if args.override is None:
        args.override = config.DEFAULT_OVERRIDE

    if args.real_robot:
        if use_sim_robot:
            raise SystemExit("--real-robot ergibt mit dem SimRobot keinen Sinn.")
        confirm_real_robot()
    if not use_sim_cameras:
        warnings = config.check_cameras_configured(cam_cfgs)
        if warnings:
            print("\n!! KAMERA-KONFIGURATION UNBESTAETIGT:")
            for warning in warnings:
                print("   - %s" % warning)
            if input("Trotzdem fahren? [j/N] ").strip().lower() != "j":
                raise SystemExit("Abgebrochen.")

    if policy is None:
        print("Lade Policy %s ..." % checkpoint)
        policy = DiffusionPolicyRunner(checkpoint, device=args.device,
                                       inference_steps=args.inference_steps, seed=args.seed)
        print("  Geraet %s, trainiert %s Schritte auf %s, Kameras %s"
              % (policy.device, info["training"]["step"],
                 info["training"]["hardware"].get("gpu", info["training"]["hardware"]["device"]),
                 info["cameras"]))

    defaults = info.get("inference_defaults", {})
    replan = args.replan or defaults.get("replan_steps", config.POLICY_REPLAN_STEPS)
    decay = args.decay if args.decay is not None else defaults.get("ensemble_decay",
                                                                   config.POLICY_ENSEMBLE_DECAY)
    max_steps = args.max_steps or int(round(1.5 * info["episode_length_max"]))

    clock = SimClock() if use_sim_robot and use_sim_cameras and not args.realtime else RealClock()
    # Asynchrone Vorhersage bei echter Uhr (Befund 2026-09-17: ~77 ms je
    # Vorhersage > 66 ms Takt). Mit SimClock gibt es keine echte Latenz --
    # dort synchron und damit reproduzierbar.
    asynchronous = isinstance(clock, RealClock) and not args.sync_predict
    if args.no_ensemble:
        if asynchronous:
            raise SystemExit("--no-ensemble nur mit synchroner Vorhersage (--sync-predict)")
        executor = ChunkExecutor(policy, n_execute=replan)
    else:
        executor = ChunkEnsembler(policy, replan_steps=replan, decay=decay,
                                  asynchronous=asynchronous)
    if use_sim_robot:
        robot = open_robot("sim", clock=clock, seed=args.seed).connect()
        robot_info = {"robot": "sim"}
    else:
        robot = open_robot("neura", allow_real=args.real_robot, override=args.override)
        try:
            robot.connect(power_on=True, ensure_automatic=True)
        except MotionRefused as exc:
            raise SystemExit("Abbruch vor jeder Bewegung: %s" % exc)
        robot_info = {"robot": "neura", "in_simulation": robot.in_simulation,
                      "override": args.override, "gripper_mode": robot.gripper_mode}
        print("Neura: is_robot_in_simulation()=%r, Override=%.2f, Greifer=%s"
              % (robot.in_simulation, args.override, robot.gripper_mode))

    try:
        captures, cam_cfgs = start_cameras(camera_mode, clock, seed=args.seed)
    except Exception:
        robot.close()
        raise

    if args.table_z is not None:
        table_z = args.table_z
    elif info.get("ideal_z_min") is not None:
        table_z = float(info["ideal_z_min"]) - 0.15  # Regel wie apps/record.py
    else:
        table_z = float(robot.read_state().tcp_quat[2]) - 0.3
    workspace = default_workspace(table_height_m=table_z)
    start = np.asarray(info["start_joints"]) if info.get("start_joints") else None
    end = np.asarray(info["end_joints"]) if info.get("end_joints") else None
    start_gripper, end_gripper = info.get("start_gripper"), info.get("end_gripper")
    if end is not None and end_gripper is None:
        print("WARNUNG: bc_policy.json ohne end_gripper (aelterer Export) -- Ende nur "
              "ueber die Stellung erkennbar, kann zu frueh ausloesen.")
    min_steps = int(0.5 * info.get("episode_length_median", 0))

    out = Path(args.out) if args.out else (
        (Path(checkpoint) if not args.hold else Path("rollouts"))
        / "rollouts" / datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    )
    out.mkdir(parents=True, exist_ok=True)
    if hasattr(policy, "warmup"):
        warm = policy.warmup(build_observation(robot, captures)[0])
        print("  Vorhersage nach Aufwaermen: %.1f ms (%d DDIM-Schritte)"
              % (1e3 * warm, policy.policy.diffusion.num_inference_steps))
    print("Fahrt: bis %d Takte, Tischhoehe %.3f m, %s, Vorhersage %s, Protokoll %s"
          % (max_steps, table_z,
             "ohne Mitteln, neu alle %d Takte" % replan if args.no_ensemble
             else "Chunks gemittelt, neu alle %d Takte, decay %.2f" % (replan, decay),
             "asynchron" if asynchronous else "synchron", out))

    results = []
    try:
        for ep in range(args.episodes):
            if robot.stop_requested and getattr(robot, "in_simulation", True) is not True:
                # An der Anlage nach einem Stopp nie automatisch weiterfahren.
                answer = input("Fahrt %d wurde gestoppt. Arbeitsraum pruefen -- naechste Fahrt? [j/N] "
                               % (ep - 1))
                if answer.strip().lower() != "j":
                    print("Beendet nach Stopp.")
                    break
            robot.clear_stop()
            # Reset wie in der Aufzeichnung (AP 5.2): Greifer auf den Startzustand,
            # dann an die Startstellung -- sonst beginnt die Policy z. B. mit
            # geschlossenem Greifer und "glaubt", das Objekt schon zu halten.
            if start_gripper is not None:
                robot.gripper_command(start_gripper > config.GRIPPER_THRESHOLD)
            if start is not None and not args.no_start_move:
                away = float(np.max(np.abs(robot.read_state().joints - start)))
                if away > config.START_POSE_TOL_RAD:
                    print("Fahrt %d: an die Startstellung (%.3f rad entfernt)" % (ep, away))
                    robot.move_to_joints(start)
            t0 = time.perf_counter()
            log = run_episode(robot, captures, executor, clock, max_steps, workspace,
                              end_joints=end, end_gripper=end_gripper, min_steps=min_steps,
                              threaded_watchdog=not isinstance(clock, SimClock))
            wall = time.perf_counter() - t0
            rollout = {"tcp": np.vstack([log["tcp"], log["final_tcp"][None]]) if len(log["tcp"]) else log["final_tcp"][None],
                       "gripper_cmd": np.append(log["gripper_cmd"], log["gripper_cmd"][-1] if len(log["gripper_cmd"]) else 0.0),
                       "reached_end": log["reached_end"]}
            summary = metrics.evaluate_rollout(rollout, info.get("reference") or {})
            predict = log["predict_s"][np.isfinite(log["predict_s"])] if len(log["predict_s"]) else np.array([])
            delays = log["delay_steps"][np.isfinite(log["delay_steps"])] if len(log["delay_steps"]) else np.array([])
            summary.update(
                episode=ep,
                stop_reason=log["stop_reason"],
                pacer_overruns=int(log["pacer_overruns"]),
                guard_rejections=int(log["guard_rejections"]),
                limiter_clamped_steps=int(log["limiter_clamped"]),
                predictions=int(len(predict)),
                asynchronous=asynchronous,
                chunk_delay_steps_max=float(delays.max()) if len(delays) else None,
                predictions_skipped_busy=int(getattr(executor, "skipped", 0)),
                predict_ms_median=float(np.median(predict) * 1e3) if len(predict) else None,
                predict_ms_p95=float(np.percentile(predict, 95) * 1e3) if len(predict) else None,
                ensemble_spread_p95_rad=float(np.percentile(log["ensemble_spread"], 95)) if len(log["ensemble_spread"]) else None,
                wall_s=wall,
                **robot_info,
            )
            results.append(summary)
            np.savez_compressed(out / ("rollout_%03d.npz" % ep),
                                **{k: v for k, v in log.items() if isinstance(v, np.ndarray)})
            print("Fahrt %d: %s -- %d Takte, Greifpunkt %s, Ende %s, Bahn p95 %s, %s, Vorhersage %s ms, "
                  "Overruns %d, gekappt %d"
                  % (ep, "ERFOLG" if summary["success"] else "kein Erfolg", summary["steps"],
                     _mm(summary.get("grasp_error_m")), _mm(summary.get("end_error_m")),
                     _mm(summary.get("path_dev_p95_m")), log["stop_reason"],
                     "%.1f" % summary["predict_ms_median"] if summary["predict_ms_median"] else "-",
                     summary["pacer_overruns"], summary["limiter_clamped_steps"]))
    finally:
        if hasattr(executor, "close"):
            executor.close()
        for cap in captures:
            cap.stop()
        robot.close()

    report = {
        "checkpoint": None if args.hold else str(checkpoint),
        "created": datetime.now().isoformat(timespec="seconds"),
        "args": vars(args),
        "robot": robot_info,
        "replan_steps": replan,
        "ensemble_decay": None if args.no_ensemble else decay,
        "episodes": results,
        "success_rate": float(np.mean([r["success"] for r in results])) if results else None,
    }
    (out / "summary.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    if results:
        print("\nErfolg %d/%d (Referenz: Greifpunkt und Uebergabepunkt je <= 10 mm) -> %s"
              % (sum(r["success"] for r in results), len(results), out / "summary.json"))


def _mm(value):
    return "-" if value is None else "%.1f mm" % (1e3 * value)


if __name__ == "__main__":
    main()

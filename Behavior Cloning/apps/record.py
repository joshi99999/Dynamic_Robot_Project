"""Automatisierte Datenaufzeichnung mit Action Noise Injection (AP 2.2/2.4).

Vorab, EINMAL je Aufruf (Anwender 2026-10-23, vorher je Episode):
    1. Wegpunkte bestimmen -- bei ``--sequence`` die Koordinaten aus der
       Punkte-Datenbank (bc/sequence.py) -- und auf Plausibilitaet pruefen
       (Endanflug von oben, ``approach_problems``).
    2. Ideale Soll-Bahn bauen (trajectory.py, LIN- und PTP-Segmente) und
       per IK loesen, dazu die Rueckfahrt des Zuruecklegens.
    3. Verrauschte Bahnen ALLER Episoden samplen + per IK loesen + Kollision
       pruefen (noise.py, Rejection Sampling).
    Probefahrt (Standard ausser gegen den SimRobot, ``--ohne-probefahrt``):
       ideale Bahn langsam, ohne Rauschen, Greifer offen, nicht
       aufgezeichnet, und zurueck -- dann Freigabe mit "j" (bc/probe.py).

Je Episode (vorher: haben sich die Punkte in der Steuerung geaendert, endet
der Lauf -- neu starten plant neu):
    4. Ausserhalb der Aufzeichnung an die Startstellung fahren, falls der Arm
       woanders steht (z. B. am Uebergabepunkt der vorigen Episode).
    5. Abfahren und aufzeichnen (recorder.py): Observation = echter,
       verrauschter Zustand; Action = ideale Sollwinkel bei t+1.
    6. Episode validieren und ablegen (dataset.py).
    7. Objekt am Greifpunkt ablegen und an den Start fahren -- ohne
       Aufzeichnung, ohne Rauschen (bc/put_back.py). Standard mit
       ``--sequence``, auch nach der letzten Episode: Bei der Aufnahme wird
       das Objekt immer wieder gebraucht, die Uebergabe an PRE_PLACE gibt es
       nur bei der Inferenz (Anwender 2026-10-23). Die naechste Episode
       faehrt dieselbe Lage mit neuem Rauschen. Abschaltbar mit
       ``--ohne-zuruecklegen``.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/record.py --sim --episodes 3 --out data_sim
    python apps/record.py --robot neura --sequence sequences/pick_to_station.json \\
        --episodes 3 --out data_vm

SICHERHEIT: Gegen den Neura-Adapter wird nur bewegt, wenn der Controller
``is_robot_in_simulation() == True`` meldet (VM und Anlage teilen eine IP).
Die reale Anlage braucht ``--real-robot`` UND eine Bestaetigung im
Terminal. Ohne Kameras-Hardware: ``--cameras sim`` (Platzhalterbilder, das
Datensatz-Schema bleibt identisch). Kamerakette mit einer USB-Webcam statt
der Daheng testen (auch ohne VM, der SimRobot laeuft dann in Echtzeit):
``--sim --cameras wrist-uvc --uvc-device 1``.
"""

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config, metrics, preview
from bc import execution as execution_module
from bc.servo import substeps_for
from bc.adapters import (
    CAMERA_MODES,
    camera_configs,
    describe_settings,
    fps_label,
    open_robot,
    parse_camera_spec,
    resolve_camera_mode,
    start_cameras,
)
from bc.clock import RealClock, SimClock
from bc.collision import default_workspace
from bc.dataset import DatasetWriter
from bc.kinematics import Kinematics
from bc.noise import PlanRejected, generate_plan, joint_speed, solve_ideal
from bc.ports import MotionRefused, ServoLimitError
from bc.auto_layout import OverridePoints, sample_pose
from bc.gui.recording import next_block
from bc.probe import probe_plans
from bc.put_back import (
    PutBackError,
    grasp_index,
    plan_put_back,
    stratified_scales,
    with_regrip,
)
from bc.recorder import EpisodeRecorder, timing_findings
from bc.table import (
    KeepoutSurface,
    TableSurface,
    clearance as table_clearance,
    keepout_problems,
    table_problems,
    tilt_deg,
    yaw_deg,
)
from bc.sequence import (
    SequenceError,
    approach_problems,
    load_sequence,
    resolve_sequence,
    same_points,
)
from bc.trajectory import Waypoint, build_ideal_trajectory, planner_speed_problems

#: Wortlaut, der fuer --real-robot eingetippt werden muss.
REAL_ROBOT_CONFIRMATION = "ANLAGE"


#: Zeile, mit der record.py nach der Bewertung einer Episode fragt. Die
#: Bedienoberflaeche erkennt sie daran und stellt die Rueckfrage selbst
#: (gui/recording.LABEL_PROMPT). Im Terminal ist es eine gewoehnliche Frage.
LABEL_PROMPT = "BEWERTUNG"
#: Erlaubte Antworten auf stdin -> (success, verworfen)
LABEL_ANSWERS = {
    "e": (True, False),    # erfolgreich
    "f": (False, False),   # fehlgeschlagen, aber behalten (markiert)
    "v": (None, True),     # verwerfen
}

#: Zeile, mit der record.py nach der Probefahrt um Freigabe bittet (Antwort
#: "j" auf stdin). Die Bedienoberflaeche erkennt sie wie LABEL_PROMPT
#: (gui/recording.PROBE_PROMPT).
PROBE_PROMPT = "PROBEFAHRT"


def ask_label(episode_index, steps):
    """Erfolgs-Label und Verwerfen abfragen (AP 5.2, AP 1.2).

    Rueckgabe ``(success, discard)``. Ohne Eingabe (geschlossenes stdin)
    bleibt die Episode UNBEWERTET und wird behalten -- nichts wird
    stillschweigend weggeworfen, und ``export.py --require-success`` laesst
    sie dann spaeter bewusst aussen vor.
    """
    # Die Frage als VOLLSTAENDIGE Zeile ausgeben, nicht als input()-Prompt:
    # Die Bedienoberflaeche liest die Ausgabe zeilenweise mit, und ein
    # Prompt ohne Zeilenumbruch kaeme dort nie an.
    print("%s Episode %d (%d Schritte) -- [e]rfolgreich, [f]ehlgeschlagen, "
          "[v]erwerfen" % (LABEL_PROMPT, episode_index, steps))
    # Ohne Prompt-Text: Ein Prompt ohne Zeilenumbruch bleibt im Puffer
    # haengen und klebt an der NAECHSTEN Ausgabezeile -- die Oberflaeche
    # las dann "> Episode 0 -> ep_00000: ..." und erkannte die Episode
    # nicht mehr. Die Frage steht ohnehin schon in der Zeile darueber.
    answer = ask("", "").strip().lower()[:1]
    if answer not in LABEL_ANSWERS:
        if answer:
            print("   '%s' nicht verstanden -- Episode bleibt unbewertet." % answer)
        return None, False
    return LABEL_ANSWERS[answer]


def ask(prompt, default=""):
    """Rueckfrage im Terminal; ohne stdin die sichere Antwort.

    Aus der Bedienoberflaeche laufen diese Skripte als Unterprozess mit
    geschlossenem stdin. ``input()`` wirft dort EOFError -- und zwar unter
    Umstaenden mitten in einem Lauf, der den Roboter bewegt. Eine
    unbeantwortete Sicherheitsfrage ist ein Nein, kein Absturz.
    """
    try:
        return input(prompt)
    except EOFError:
        # input() hat den Prompt schon geschrieben -- hier nur noch, was
        # daraus wird.
        print("[keine Eingabe moeglich -> '%s']" % default)
        return default


class EpisodeProgress(object):
    """Fortschritt je Takt -- Zeilen fuer das Terminal, optional ein Livebild.

    Wird dem Recorder als ``monitor`` uebergeben. Die Zeilen sind das, was
    die Bedienoberflaeche mitliest; ohne ``--preview`` faellt hier nur eine
    Zeile je Sekunde an.
    """

    #: Abstand der Fortschrittszeilen in Takten (15 Hz -> einmal je Sekunde).
    EVERY = 15

    def __init__(self, preview_writer=None):
        self.preview = preview_writer
        self.episode = 0
        self.bad = 0

    def start_episode(self, episode):
        self.episode = episode
        self.bad = 0

    def __call__(self, i, n, frames, report):
        if not report.ok:
            self.bad += 1
        if i % self.EVERY == 0 or i == n - 1:
            print("  Takt %4d von %d  Versatz %3.0f ms  ausserhalb Budget %d"
                  % (i, n, 1e3 * report.spread, self.bad))
        if self.preview is not None:
            self.preview.offer(
                frames, "Episode %d  Takt %d/%d" % (self.episode, i, n))


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


def point_problems(waypoints, resolved):
    """Alle Pruefungen vor jeder Bewegung: Endanflug, Tisch, Sperrzonen."""
    problems = approach_problems(waypoints)
    if resolved is not None:
        problems += table_problems(waypoints, resolved.table)
        problems += keepout_problems(waypoints, resolved.table, resolved.keepouts)
    return problems


def plan_lage(args, waypoints, resolved, robot, kin, scales, rng, index, block, source,
              yaw_deg_turn=None):
    """Eine Lage planen: Bahnen aller Fahrten plus Rueckfahrt (= Ablegen dort)."""
    ideal, workspace, plans, back_plan = plan_all(
        args, waypoints, robot, kin, scales, rng,
        table=resolved.table if resolved is not None else None,
        keepouts=resolved.keepouts if resolved is not None else ())
    xy = None
    if resolved is not None and "PICK" in resolved.points:
        xy = [float(v) for v in resolved.points["PICK"]["pose_quat"][:2]]
    return {
        "index": index, "block": block, "source": source, "resolved": resolved,
        "waypoints": waypoints, "plans": plans, "back_plan": back_plan, "scales": scales,
        "xy": xy, "yaw_deg": yaw_deg_turn,
        # Ablegen HIER als neue Lage (Umlegen, Rueckkehr): mit Nachgreifen
        "regrip_plan": (with_regrip(back_plan)
                        if back_plan is not None and args.nachgreifen == "lagenwechsel"
                        else back_plan),
        "blend_m": {wp.name or str(k): r
                    for k, (wp, r) in enumerate(zip(waypoints, ideal.blend_applied)) if r > 0},
    }


def next_auto_lage(args, sequence, robot, kin, rng, start, used_xy, scales, index, block):
    """Neue Lage ziehen und planen (bc/auto_layout.py); None, wenn keine passt.

    Gezogen wird im Bereich der Tischpunkte, ausserhalb der Sperrzonen, mit
    Abstand zu den bisherigen Lagen. Dieselben Pruefungen wie fuer eine
    geteachte Lage -- scheitert eine, wird neu gezogen.
    """
    base = start["resolved"]
    pick = base.points["PICK"]
    print("Naechste Lage wird gesucht ...")
    for attempt in range(config.AUTO_PLAN_TRIES):
        try:
            pose, turn = sample_pose(
                rng, pick["pose_quat"], base.table,
                [p["pose_quat"] for p in base.table_points.values()], base.keepouts,
                used_xy, yaw_range_deg=args.drehung, pick_height=sequence.pick_height,
                margin=args.randabstand)
            joints = kin.ik(pose, pick["joints"])
            pos_err, rot_err = kin.check_fk_roundtrip(joints, pose)
            if pos_err > config.IK_FK_TOL_POS_M or rot_err > config.IK_FK_TOL_ROT_RAD:
                raise ValueError("IK trifft die Pose nicht (%.1f mm)" % (1e3 * pos_err))
            resolved = resolve_sequence(
                sequence, OverridePoints(robot, {"PICK": (joints, pose)}))
            problems = point_problems(resolved.waypoints, resolved)
            if problems:
                raise ValueError(problems[0])
            lage = plan_lage(args, resolved.waypoints, resolved, robot, kin, scales, rng,
                             index=index, block=next_block(block) if block else None,
                             source="automatisch", yaw_deg_turn=yaw_deg(pose, base.table))
            if not any(p is not None for p in lage["plans"]):
                raise ValueError("keine gueltige verrauschte Bahn")
            print("Lage %d: x %.0f mm, y %.0f mm, Drehung %+.0f Grad gegen die geteachte"
                  % (index + 1, 1e3 * pose[0], 1e3 * pose[1], turn))
            return lage
        except (ValueError, SequenceError, RuntimeError, SystemExit) as exc:
            # SystemExit: plan_all bricht bei unbrauchbarer idealer Bahn oder
            # Rueckfahrt so ab -- hier heisst das nur "andere Lage ziehen"
            print("   Lage verworfen (Versuch %d): %s" % (attempt + 1, exc))
    print("Keine weitere Lage gefunden -- zurueck an den geteachten PICK.")
    return None


def duration_line(args, lage, plan_s):
    """"Geschaetzte Dauer: ca. ..." aus den geplanten Bahnen der ersten Lage.

    Je Fahrt: Episode + Zuruecklegen (geplante Laenge) + Nebenzeit; je
    weitere Lage: die gemessene Planungszeit plus Umlegen. Die Oberflaeche
    zeigt die Zeile an (gui/recording.duration_estimate). Ohne Wartezeit
    fuer Bewertungen.
    """
    plans = [p for p in lage["plans"] if p is not None]
    rate = plans[0].rate_hz if plans else config.CONTROL_RATE_HZ
    episode_s = (sum(len(p) for p in plans) / len(plans) / rate) if plans else 0.0
    back_s = len(lage["back_plan"]) / rate if lage["back_plan"] is not None else 0.0
    cycle = episode_s + back_s + config.ESTIMATE_OVERHEAD_S
    total = args.episodes * args.lagen * cycle
    total += (args.lagen - 1) * (plan_s + config.ESTIMATE_TRANSFER_S)
    if args.probefahrt:
        total += config.ESTIMATE_PROBE_S
    return ("Geschaetzte Dauer: ca. %s (%d Fahrt(en) x %d Lage(n), je Fahrt ~%.0f s%s)"
            % (format_duration(total), args.episodes, args.lagen, cycle,
               ", ohne Wartezeit fuer Bewertungen" if args.ask_label else ""))


def format_duration(seconds):
    """"8 min", "1 h 25 min" -- grob, fuer Meldungen."""
    minutes = int(round(seconds / 60.0))
    if minutes < 1:
        return "%d s" % int(round(seconds))
    if minutes < 60:
        return "%d min" % minutes
    return "%d h %02d min" % (minutes // 60, minutes % 60)


def stop_requested(args):
    """Hat die Oberflaeche "Nach dieser Lage beenden" gedrueckt?"""
    return bool(args.stopp_datei) and Path(args.stopp_datei).exists()


def resolve_points(sequence, robot, use_sim_robot):
    """Koordinaten des Ablaufs aus der Punkte-Datenbank (bc/sequence.py)."""
    try:
        return resolve_sequence(sequence, robot)
    except (SequenceError, KeyError) as exc:
        # Haeufigster Fall, und aus "vorhanden: " allein nicht zu erraten:
        # Die Punkte stehen in der Datenbank der Control-Box und sind nur
        # ueber den Neura-Adapter lesbar. Der SimRobot haelt eine eigene,
        # leere Liste.
        hint = ""
        if use_sim_robot and not robot.point_names():
            hint = ("\nDer SimRobot hat keine Punkte-Datenbank -- die "
                    "Punkte liegen in der Control-Box. Entweder "
                    "--robot neura verwenden oder ohne --sequence "
                    "aufzeichnen (Demo-Wegpunkte um die Home-Pose).")
        raise SystemExit("Ablauf nicht aufloesbar: %s%s" % (exc, hint))


def plan_all(args, waypoints, robot, kin, scales, rng, table=None, keepouts=()):
    """Alle Bahnen des Aufrufs EINMAL vorab planen (Schritte 2-3, AP 2.4).

    Wunsch Anwender 2026-10-23: Zwischen zwei Episoden stand der Arm lange,
    weil je Episode neu geplant wurde -- ideale Bahn, verrauschte Bahn und
    Rueckfahrt. An der Anlage laeuft jeder IK/FK-Schritt als Aufruf ueber
    die Steuerung, grob 2000 je Episode. Ideale Bahn und Rueckfahrt sind bei
    Wiederholungen derselben Lage aber jedes Mal dieselben; neu ist nur das
    Rauschen. Jetzt: beide einmal, dazu die verrauschten Bahnen aller
    Episoden, alles bevor sich der Arm bewegt.

    Nicht parallel zur Fahrt: Die Planung liefe ueber dieselbe Verbindung
    wie der 60-Hz-servo_j-Strom, der an der Anlage ohnehin schon 3-7
    verspaetete Aufrufe je Episode hat.

    Rueckgabe ``(ideal, workspace, plans, back_plan)``; ``plans[k]`` ist
    None, wenn fuer Episode k keine gueltige verrauschte Bahn gefunden
    wurde, ``back_plan`` None ohne Zuruecklegen. Eine unbrauchbare ideale
    Bahn oder Rueckfahrt bricht ab -- noch hat sich nichts bewegt.
    """
    t0 = time.perf_counter()
    speeds = dict(transit_speed=args.transit_speed,
                  approach_speed=args.approach_speed,
                  ptp_joint_speed=args.ptp_speed)
    ideal = build_ideal_trajectory(waypoints, fk=robot.fk, **speeds)
    z_min = min(wp.pose_quat[2] for wp in waypoints)
    table_z = args.table_z if args.table_z is not None else z_min - 0.15
    workspace = default_workspace(table_height_m=table_z)
    if table is not None:
        # Werkzeugspitze gegen die geteachte Tischebene (bc/table.py); der
        # Quader darunter bleibt fuer Handgelenk und Ellbogen
        workspace.add_box(TableSurface(table))
        print("Kollisionsmodell: Werkzeugspitze mindestens %.1f mm ueber der Tischebene"
              % (1e3 * config.TABLE_MIN_CLEARANCE_M))
    for zone in keepouts:
        workspace.add_box(KeepoutSurface(zone, table))
        print("Kollisionsmodell: Sperrzone %s +-%.0f mm, darin mindestens %.0f mm ueber dem Tisch"
              % (zone.name, 1e3 * zone.half_size, 1e3 * zone.min_height))
    print("Kollisionsmodell: Tischhoehe z = %.3f m" % table_z)
    print(
        "Soll-Bahn: %d Schritte (%.1f s bei %.0f Hz), %d Dwell-Schritte"
        % (len(ideal), len(ideal) / ideal.rate_hz, ideal.rate_hz,
           int(ideal.dwell_mask.sum()))
    )
    # Geseedet mit der geteachten Startstellung
    seed_joints = waypoints[0].joints
    if seed_joints is None:
        seed_joints = robot.read_state().joints
    try:
        joints_ideal = solve_ideal(kin, workspace, ideal, seed_joints)
    except PlanRejected as exc:
        raise SystemExit("Abbruch vor jeder Bewegung: %s" % exc)

    plans = []
    for ep, scale in enumerate(scales):
        try:
            plans.append(generate_plan(kin, workspace, ideal, seed_joints, rng=rng,
                                       noise_scale=scale, joints_ideal=joints_ideal))
        except PlanRejected as exc:
            print("Episode %d: Planung verworfen (%s)" % (ep, exc))
            plans.append(None)

    back_plan = None
    if args.zuruecklegen:
        try:
            back_plan = plan_put_back(kin, workspace, waypoints, fk=robot.fk,
                                      regrip=args.nachgreifen == "immer", **speeds)
        except PutBackError as exc:
            raise SystemExit(
                "Abbruch vor jeder Bewegung: Zuruecklegen nicht planbar (%s). "
                "Punkte pruefen oder --ohne-zuruecklegen." % exc)
    print("Planung: %d Episode(n)%s in %.1f s -- einmal fuer den ganzen Lauf"
          % (len(scales), " + Rueckfahrt" if back_plan is not None else "",
             time.perf_counter() - t0))
    # Gelenktempo der Bahnen gegen den ServoGuard (Anlage 2026-10-26: eine
    # Bahn mit 1,47 rad/s wurde erst beim Fahren gestoppt)
    ideal_speed = joint_speed(joints_ideal, ideal.rate_hz)
    noisy = [joint_speed(p.joints_noisy, p.rate_hz) for p in plans if p is not None]
    worst = max(noisy) if noisy else (0.0, 0, 0)
    print("Gelenktempo: ideal max %.2f rad/s (Gelenk %d), verrauscht max %.2f rad/s "
          "(Gelenk %d) -- Planergrenze %.2f, ServoGuard %.2f"
          % (ideal_speed[0], ideal_speed[1] + 1, worst[0], worst[1] + 1,
             config.PLAN_MAX_JOINT_SPEED_RADS, config.SERVO_MAX_JOINT_SPEED_RADS))
    for ep, plan in enumerate(plans):
        if plan is not None and plan.rejects:
            print("   Episode %d: %d Rauschsample(s) verworfen und neu gezogen"
                  % (ep, plan.rejects))
    return ideal, workspace, plans, back_plan


def run_probe(plans, back_plan, robot, recorder, factor):
    """Langsame Probefahrt vor der ersten Episode, danach Freigabe (bc/probe.py).

    Ideale Bahn ohne Rauschen, Greifer offen, nicht aufgezeichnet; zurueck
    auf der Bahn des Zuruecklegens (sonst rueckwaerts). Bricht die Fahrt ab
    oder wird sie nicht freigegeben, endet der Aufruf (SystemExit) -- bevor
    eine Episode mit Tempo und Greifer faehrt.
    """
    forward = next(p for p in plans if p is not None)
    hin, zurueck = probe_plans(forward, back_plan, factor)
    print("Probefahrt: ideale Bahn mit %.0f %% Tempo, ohne Rauschen, Greifer offen, "
          "nicht aufgezeichnet (hin %.1f s, zurueck %.1f s)"
          % (100 * factor, len(hin) / hin.rate_hz, len(zurueck) / zurueck.rate_hz))
    start = hin.joints_noisy[0]
    if float(np.max(np.abs(robot.read_state().joints - start))) > config.START_POSE_TOL_RAD:
        robot.move_to_joints(start)
    for name, plan in (("hin", hin), ("zurueck", zurueck)):
        try:
            result = recorder.record(plan, metadata={"probe": True})
        except RuntimeError as exc:  # RobotError, oder Stopp vor dem ersten Takt
            raise SystemExit("Probefahrt (%s) abgebrochen: %s" % (name, exc))
        if result.discarded:
            raise SystemExit("Probefahrt (%s) abgebrochen (%s) -- Punkte und "
                             "Arbeitsraum pruefen." % (name, result.discard_reason))
    # Als vollstaendige Zeile, wie bei der Bewertung (ask_label): die
    # Oberflaeche erkennt sie und fragt selbst.
    print("%s beendet -- Aufnahme starten? [j]a / [n]ein" % PROBE_PROMPT)
    if ask("", "").strip().lower()[:1] != "j":
        raise SystemExit("Nach der Probefahrt abgebrochen -- keine Episode aufgezeichnet.")
    print("Probefahrt freigegeben.")


def save_reference_images(captures, cam_cfgs, out, block=None):
    """Ein Bild je echter Kamera in VOLLER Aufloesung, einmal je Aufruf (AP 1.4).

    Die ArUco-Rektifizierung der Szenenkamera (bc/rectify.py) ist noch nicht
    Teil der Aufnahme; gespeichert werden die Bilder unrektifiziert in
    240 x 320. Rektifiziert wird spaeter auf diesen Rohdaten (Export) -- die
    Homographie ist ein reines Bild-zu-Bild-Mapping, Labels und Zustand
    aendern sich nicht. Bei 240 x 320 findet der Detektor die Marker aber
    nur grenzwertig (OpenCV 4.13: 2 von 4 bei Kippung). Mit diesem Vollbild
    laesst sich die Homographie je Block genau bestimmen und auf 240 x 320
    umrechnen -- und es belegt die Kamerapose des Blocks.

    Rueckgabe ``{Kamera: Pfad relativ zu out}``; ohne OpenCV oder ohne Bild
    leer -- eine fehlende Referenz bricht keine Aufnahme ab.
    """
    real = {c.name for c in cam_cfgs if c.backend != "sim"}
    if not real:
        return {}
    try:
        import cv2
    except ImportError:
        print("   Referenzbilder: OpenCV fehlt -- keine gespeichert.")
        return {}
    folder = Path(out) / "referenzbilder"
    folder.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    tag = "_%s" % "".join(c if c.isalnum() or c in "-_" else "-" for c in block) if block else ""
    saved = {}
    for cap in captures:
        frame = cap.latest() if cap.name in real else None
        if frame is None:
            continue
        path = folder / ("%s%s_%s.png" % (stamp, tag, cap.name))
        image = frame.image
        if image.ndim == 3:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if cv2.imwrite(str(path), image):
            saved[cap.name] = path.relative_to(Path(out)).as_posix()
    if saved:
        print("   Referenzbilder (Vollbild): %s" % ", ".join(sorted(saved.values())))
    return saved


def object_layout(resolved):
    """``object_xy_m`` und ``object_yaw_deg`` (0..180, um die Tischnormale)
    aus PICK -- automatisch statt von Hand ins Feld "Objekt"."""
    if resolved is None or "PICK" not in resolved.points:
        return {}
    pose = resolved.points["PICK"]["pose_quat"]
    return {"object_xy_m": [float(pose[0]), float(pose[1])],
            "object_yaw_deg": round(yaw_deg(pose, resolved.table), 1)}


def switch_to_teach(robot):
    """Nach dem Lauf in den Teach-Modus (``--nach-ende-teach``).

    Auch nach einem Abbruch -- dann wird meist ohnehin neu geteacht. Ein
    Fehlschlag ist nur eine Meldung: der Lauf ist vorbei, am Pendant geht
    es wie bisher. Gegen den SimRobot gibt es keinen Teach-Modus.
    """
    if not hasattr(robot, "switch_to_teach"):
        return
    try:
        state = robot.switch_to_teach()
    except Exception as exc:
        print("Teach-Modus nicht geschaltet (%s) -- am Pendant umschalten." % exc)
        return
    if state is True:
        print("Steuerung steht jetzt im Teach-Modus.")
    else:
        print("Teach-Modus angefordert, die Steuerung meldet is_robot_in_teach_mode() "
              "= %r -- am Pendant pruefen." % (state,))


def put_back(episode, plan, put_back_recorder):
    """Objekt am Greifpunkt ablegen und an den Start fahren (Schritt 7).

    ``plan``: die vorab geplante Rueckfahrt (:func:`plan_all`).

    Rueckgabe False = Wiederholungen hier beenden. Nur nach einer
    VOLLSTAENDIG gefahrenen Episode, die nicht als fehlgeschlagen bewertet
    wurde: Nach einem Abbruch steht der Arm irgendwo auf der Bahn, nach
    einem Fehlgriff liegt das Objekt nicht mehr dort, wohin die geteachte
    Bahn greift -- die naechste Episode haette dann falsche Labels.
    """
    md = episode.metadata
    if md.get("recorded_steps") != md.get("planned_steps"):
        print("Zuruecklegen entfaellt: Episode nicht vollstaendig gefahren (%s) -- "
              "Wiederholungen beendet, Objekt von Hand pruefen." % episode.discard_reason)
        return False
    if episode.success is False:
        print("Zuruecklegen entfaellt: Episode als fehlgeschlagen bewertet -- "
              "Wiederholungen beendet, Objekt neu hinlegen.")
        return False
    print("Zuruecklegen: %d Schritte (%.1f s), nicht aufgezeichnet"
          % (len(plan), len(plan) / plan.rate_hz))
    try:
        result = put_back_recorder.record(plan, metadata={"put_back": True})
    except RuntimeError as exc:  # RobotError, oder Stopp vor dem ersten Takt
        print("Zuruecklegen abgebrochen: %s -- Wiederholungen beendet." % exc)
        return False
    # Ohne Kameras gibt es kein Latenzbudget -- verworfen heisst hier immer:
    # Stopp, Abbruch oder der Arm ist nicht gefolgt.
    if result.discarded:
        print("Zuruecklegen abgebrochen (%s) -- Wiederholungen beendet, "
              "Objekt und Arm von Hand pruefen." % result.discard_reason)
        return False
    return True


def confirm_real_robot():
    print("\n!! --real-robot: Die Freigabe gilt der REALEN ANLAGE.")
    print("   Not-Aus in Reichweite? Arbeitsraum frei? Override geprueft?")
    answer = ask("   Zum Fortfahren '%s' eintippen: " % REAL_ROBOT_CONFIRMATION)
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
             "Wrist-Kamera + Platzhalter-Szene (Labortest); wrist-uvc: USB-Webcam "
             "statt Wrist-Kamera (nur Test der Kamerakette)",
    )
    parser.add_argument(
        "--uvc-device", type=int, default=None,
        help="OpenCV-Index der Webcam bei --cameras wrist-uvc "
             "(Default config.WRIST_CAMERA_UVC_STANDIN; tools/check_cameras.py --list)",
    )
    parser.add_argument(
        "--ask-label", action="store_true",
        help="nach jeder Episode nach Erfolg/Verwerfen fragen (AP 5.2). Ohne "
             "das bleibt die Episode unbewertet (ausser gegen den SimRobot)",
    )
    parser.add_argument(
        "--preview", default=None, metavar="DATEI",
        help="waehrend der Aufnahme das zuletzt aufgenommene Bild als JPEG hierhin "
             "schreiben (fuer die Bedienoberflaeche). STANDARD AUS: kostet Zeit in "
             "der 15-Hz-Schleife",
    )
    parser.add_argument(
        "--preview-hz", type=float, default=None,
        help="wie oft das Vorschaubild hoechstens geschrieben wird (Default %g)"
             % preview.DEFAULT_RATE_HZ,
    )
    parser.add_argument(
        "--camera", action="append", default=None, metavar="NAME=BACKEND[:GERAET][@FPS]",
        help="einzelnen Kameraplatz abweichend besetzen, z. B. 'scene=uvc:1' oder 'wrist=sim', mit Bildrate 'wrist=daheng@30' bzw. '@nativ'; mehrfach angebbar. Ergaenzt --cameras, das den Ausgangspunkt setzt (Geraete auflisten: tools/check_cameras.py --list)",
    )
    parser.add_argument(
        "--real-robot", action="store_true",
        help="reale Anlage freigeben (sonst nur Simulation), mit Bestaetigung",
    )
    parser.add_argument(
        "--no-gripper", action="store_true",
        help="TESTFAHRT ohne konfigurierten Greifer: die Bahn wird vollstaendig "
             "gefahren, Greiferbefehle werden nur protokolliert. Jede Episode "
             "wird dabei VERWORFEN und ist nicht exportierbar",
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
    # Planer-Tempo (AP 2.6): wird mitgelernt und muss innerhalb eines
    # Datensatzes einheitlich sein -- apps/export.py erzwingt das ueber
    # die Metadaten. Je Lauf einstellbar, damit es ohne Codeaenderung
    # festgelegt werden kann (Labortag) und im Protokoll steht.
    planner = parser.add_argument_group(
        "Planer-Tempo (AP 2.6) -- mitgelernt, fuer einen Datensatz einheitlich")
    planner.add_argument(
        "--transit-speed", type=float, default=config.TRANSIT_SPEED_MS,
        help="TCP-Geschwindigkeit im Transit in m/s (Default %(default)g)",
    )
    planner.add_argument(
        "--approach-speed", type=float, default=config.APPROACH_SPEED_MS,
        help="TCP-Geschwindigkeit im Endanflug (Greifen/Absetzen) in m/s (Default %(default)g)",
    )
    planner.add_argument(
        "--ptp-speed", type=float, default=config.PTP_JOINT_SPEED_RADS,
        help="hoechste Gelenkgeschwindigkeit in PTP-Segmenten in rad/s (Default %(default)g)",
    )
    # Ausfuehrungsart (bc/execution.py, Dokumentation/Zittern-an-der-Anlage.md):
    # wie die Sollwerte an die Steuerung gehen. Steht in den Metadaten und
    # wird von der Inferenz uebernommen.
    run = parser.add_argument_group(
        "Ausfuehrung -- servo_j (60 Hz), servo_j_pos (60 Hz ohne v/a) oder "
        "online (movelinear_online), fuer einen Datensatz einheitlich")
    run.add_argument(
        "--ausfuehrung", choices=(execution_module.AUTO,) + execution_module.MODES,
        default=config.EXECUTION_MODE,
        help="servo_j = Arbeitswert seit 2026-10-19 (mit Override 0.5); "
             "auto = online an der realen Anlage, servo_j in VM und SimRobot; "
             "servo_j_pos = servo_j ohne Geschwindigkeit/Beschleunigung, die "
             "Steuerung glaettet selbst (VM 2026-10-14, an der Anlage ungeprueft) "
             "(Default %(default)s)")
    run.add_argument("--online-vorlauf", type=int, default=config.ONLINE_LEAD_STEPS,
                     help="online: Ziel so viele 15-Hz-Takte voraus (Default %(default)s)")
    run.add_argument("--online-tempo", type=float, default=config.ONLINE_SPEED_FACTOR,
                     help="online: Tempogrenze als Vielfaches der Planer-Spitze "
                          "(Default %(default)s)")
    run.add_argument("--online-beschl", type=float, default=config.ONLINE_ACCEL_FACTOR,
                     help="online: Beschleunigungsgrenze als Vielfaches der Planer-Spitze "
                          "(Default %(default)s)")
    run.add_argument("--online-halt-mm", type=float, default=config.ONLINE_REACH_TOL_MM,
                     help="online: am Greiferwechsel warten, bis der TCP naeher als so "
                          "viele mm am Haltepunkt steht; 0 = nicht warten "
                          "(Default %(default)s, bc/execution.py)")
    run.add_argument("--online-halt-s", type=float, default=config.ONLINE_REACH_TIMEOUT_S,
                     help="online: hoechstens so lange warten, sonst Episode verwerfen "
                          "(Default %(default)s)")
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument(
        "--ohne-zuruecklegen", dest="ohne_zuruecklegen", action="store_true",
        help="Objekt NICHT zuruecklegen. Standard mit --sequence: nach jeder Episode "
             "(auch der letzten) legt der Arm das Objekt am Greifpunkt ab und faehrt "
             "an den Start -- nicht aufgezeichnet, ohne Rauschen. --episodes = Fahrten "
             "derselben Lage (empfohlen 3), Rauschfaktor gestaffelt (bc/put_back.py)",
    )
    parser.add_argument(
        "--lagen", type=int, default=1,
        help="so viele verschiedene Objektlagen je Aufruf (Default 1 = nur die geteachte). "
             "Ab 2 legt der Arm das Objekt nach den Fahrten einer Lage selbst an eine "
             "neue, zufaellige Stelle im Bereich der Tischpunkte (bc/auto_layout.py); "
             "nach der letzten Lage zurueck an den geteachten PICK",
    )
    parser.add_argument(
        "--drehung", type=float, default=config.AUTO_YAW_RANGE_DEG,
        help="automatische Lagen: Drehung bis zu +- so viel Grad gegen die geteachte "
             "(Default %(default)g)",
    )
    parser.add_argument(
        "--randabstand", type=float, default=config.AUTO_EDGE_MARGIN_M,
        help="automatische Lagen: so viel Abstand (m) zum Rand des Bereichs der "
             "Tischpunkte (Default %(default)g)",
    )
    parser.add_argument(
        "--stopp-datei", dest="stopp_datei", default=None, metavar="DATEI",
        help="existiert diese Datei, endet der Lauf nach der laufenden Lage (Objekt "
             "zurueck an den geteachten PICK) -- fuer 'Nach dieser Lage beenden'",
    )
    parser.add_argument(
        "--nachgreifen", choices=config.PUT_BACK_REGRIP_MODES,
        default=config.PUT_BACK_REGRIP_DEFAULT,
        help="nach dem Ablegen einmal zu/auf, damit der Greifer das Objekt zentriert: "
             "nur beim Ablegen an einer neuen Lage (Standard), immer oder nie "
             "(bc/put_back.py)",
    )
    parser.add_argument(
        "--ohne-probefahrt", dest="ohne_probefahrt", action="store_true",
        help="KEINE Probefahrt. Standard (nicht gegen den SimRobot): vor der ersten "
             "Episode die ideale Bahn langsam abfahren -- ohne Rauschen, Greifer "
             "offen, nicht aufgezeichnet -- und zurueck; danach Freigabe mit 'j' "
             "(bc/probe.py)",
    )
    parser.add_argument(
        "--probefahrt-tempo", type=float, default=config.PROBE_SPEED_FACTOR,
        help="Tempo der Probefahrt als Anteil des Planer-Tempos (Default %(default)g)",
    )
    parser.add_argument(
        "--nach-ende-teach", dest="nach_ende_teach", action="store_true",
        help="nach dem Lauf (auch nach Abbruch) die Steuerung in den Teach-Modus "
             "schalten -- zum Teachen des naechsten PICK ohne Anmeldung am Pendant. "
             "Zurueck in Automatik schaltet der naechste Start selbst",
    )
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
        "--object-points", default="PICK,PRE_PICK",
        help="Punkte, deren geteachte Lage als Objektlage abgelegt wird (Komma-Liste)",
    )
    args = parser.parse_args()
    if args.sequence and args.waypoints:
        parser.error("--sequence und --waypoints schliessen sich aus")
    # Zuruecklegen braucht die geteachten Gelenkstellungen -- also eine
    # Ablaufdatei. Ohne (Demo, --waypoints) bleibt es wie bisher aus.
    args.zuruecklegen = bool(args.sequence) and not args.ohne_zuruecklegen
    # Vor allem anderen pruefen: eine unpassende Senderate soll hier
    # scheitern, nicht nach dem Bestromen mitten im Aufbau.
    try:
        substeps_for(args.servo_rate)
    except ValueError as exc:
        parser.error(str(exc))
    problems = planner_speed_problems(
        args.transit_speed, args.approach_speed, args.ptp_speed)
    if problems:
        parser.error("; ".join(problems))
    if (not 0 <= args.online_vorlauf <= 5 or not 0.5 <= args.online_tempo <= 3.0
            or not 0.5 <= args.online_beschl <= 10.0):
        parser.error("--online-vorlauf 0..5, --online-tempo 0.5..3, --online-beschl 0.5..10")
    if not 0 <= args.online_halt_mm <= 20 or not 0 < args.online_halt_s <= 30:
        parser.error("--online-halt-mm 0..20 (0 = aus), --online-halt-s 0..30")
    if not 1 <= args.lagen <= 500 or not 0 <= args.drehung <= 90 \
            or not 0 <= args.randabstand <= 0.2:
        parser.error("--lagen 1..500, --drehung 0..90, --randabstand 0..0.2")
    if not 0.1 <= args.probefahrt_tempo <= 1.0:
        parser.error("--probefahrt-tempo 0.1..1")
    if args.seed is None:
        args.seed = int(np.random.SeedSequence().entropy % (2**31))
    return args


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)  # Fortschritt auch in Logdateien
    args = parse_args()
    robot_kind = args.robot or ("sim" if args.sim else "neura")
    use_sim_robot = robot_kind == "sim"
    # Gegen den SimRobot gibt es nichts, woran der Arm stossen koennte.
    args.probefahrt = not args.ohne_probefahrt and not use_sim_robot
    camera_mode = resolve_camera_mode(args.cameras, use_sim_robot)
    use_sim_cameras = camera_mode == "sim"

    sequence = load_sequence(args.sequence) if args.sequence else None

    try:
        cam_cfgs = camera_configs(camera_mode, uvc_device=args.uvc_device,
                                  specs=args.camera)
    except ValueError as exc:
        raise SystemExit("--camera: %s" % exc)
    # Der Modus sagt nur, WOMIT begonnen wird -- nach den Zuordnungen kann
    # trotzdem ueberall ein Platzhalter stehen (oder eben nicht mehr).
    use_sim_cameras = all(c.backend == "sim" for c in cam_cfgs)
    if not use_sim_cameras:
        # VOR jeder Hardware-Aktion: eine falsch konfigurierte Kamera faellt
        # im fertigen Datensatz nicht auf, deshalb hier hart nachfragen.
        warnings = config.check_cameras_configured(cam_cfgs)
        if warnings:
            print("\n!! KAMERA-KONFIGURATION UNBESTAETIGT:")
            for warning in warnings:
                print("   - %s" % warning)
            if ask("Trotzdem aufzeichnen? [j/N] ", "n").strip().lower() != "j":
                raise SystemExit("Abgebrochen.")

    if args.real_robot:
        if use_sim_robot:
            raise SystemExit("--real-robot ergibt mit dem SimRobot keinen Sinn.")
        webcams = [c.name for c in cam_cfgs if c.backend == "uvc"]
        if camera_mode == "wrist-uvc" or "wrist" in webcams:
            raise SystemExit(
                "Eine Webcam als Wrist-Kamera ist nur ein Test der Kamerakette "
                "-- an der Anlage 'wrist-real', 'real' oder 'sim' verwenden."
            )
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
        robot = open_robot("neura", allow_real=args.real_robot, override=args.override,
                           allow_dry_gripper=args.no_gripper)
        try:
            robot.connect(power_on=True, ensure_automatic=True)
        except MotionRefused as exc:
            raise SystemExit("Abbruch vor jeder Bewegung: %s" % exc)
        robot_info = {
            "robot": "neura",
            "in_simulation": robot.in_simulation,
            "tool": robot.tool_name,
            "gripper_mode": robot.gripper_mode,
            # Digitalausgaenge statt grasp()/release() (config.
            # NEURA_GRIPPER_DO_CLOSE/_OPEN) -- andere Schaltzeit moeglich.
            "gripper_via_outputs": robot.gripper_via_outputs,
            "override": args.override,
            "timestamp_source": "host",
            # Welcher FK-Weg lief? Lokal ist ein RPC-Roundtrip weniger je
            # Takt und kein Versatz zwischen Roboter- und Bildzeitstempel --
            # beim Vergleich zweier Aufnahmen gehoert das in die Metadaten.
            "local_fk": robot.local_fk_active,
            "local_fk_check": robot.local_fk_check,
        }
        print(
            "Neura: is_robot_in_simulation()=%r, Tool=%r, Greifer=%s, Override=%.2f"
            % (robot.in_simulation, robot.tool_name, robot.gripper_mode, args.override)
        )
        if robot.local_fk_active:
            print("   FK lokal aus der URDF-Kette (ein RPC-Aufruf je Takt weniger).")
        else:
            print("   FK ueber die Steuerung: %s"
                  % (robot.local_fk_check or {}).get("grund", "unbekannt"))
        if args.override < 0.5:
            # Gemessen VM 2026-09-14 (tools/check_sim_robot.py): Override 0.2
            # bremst den servo_j-Strom -- Nachlauf ~1 s statt 67 ms bei 1.0.
            # Die Observation liefe dann der gesendeten Bahn weit hinterher.
            # Anlage 2026-10-19: 0.5 ist der Arbeitswert (config.
            # DEFAULT_OVERRIDE) -- ~0,09 s Nachlauf, Bahn auf ~1 mm, und
            # erst dadurch glatt. Das Tempo begrenzt der Planer
            # (TRANSIT_SPEED_MS), nicht der Override.
            print(
                "   WARNUNG: Override %.2f < 0.5 -- der Arm folgt servo_j stark "
                "verzoegert (VM: ~1 s Nachlauf bei 0.2; Arbeitswert 0.5)." % args.override
            )
        elif args.override > 0.5 and args.ausfuehrung == execution_module.SERVO_J:
            # Anlage 2026-10-19: bei 1.0 faehrt der Arm die 60-Hz-Stufen als
            # Treppe -- H1 1,6-1,7 m/s^2 statt 0,19-0,20 bei 0.5.
            print(
                "   WARNUNG: Override %.2f > 0.5 mit servo_j -- an der Anlage "
                "zittert der Arm dann deutlich (Arbeitswert 0.5)." % args.override
            )
        if robot.gripper_mode == "logged":
            print("   Kein Greifer konfiguriert -- Greiferbefehle werden nur protokolliert.")
        elif robot.gripper_mode == "dry_run":
            # Der Arm faehrt gleich die volle Bahn, nur der Greifer schaltet
            # nicht. Das ist als Bahntest gewollt -- aber die Episode enthaelt
            # dann Greiferbefehle im Action-Vektor, die nie stattgefunden
            # haben. Sie wird deshalb unten hart verworfen.
            print("")
            print("   " + "!" * 66)
            print("   !! TROCKENLAUF: --no-gripper ist gesetzt.")
            print("   !! Der Arm faehrt die VOLLSTAENDIGE Bahn an der realen Anlage,")
            print("   !! der Greifer schaltet dabei NICHT. Kein Werkstueck einlegen.")
            print("   !! Jede Episode wird verworfen und ist nicht exportierbar.")
            print("   " + "!" * 66)
            print("")
        elif robot.gripper_mode == "unavailable":
            robot.close()
            raise SystemExit(
                "Abbruch: reale Anlage ohne konfigurierten Greifer. Schaltet er "
                "ueber Digitalausgaenge der Control-Box, die Pins in config."
                "NEURA_GRIPPER_DO_CLOSE/_OPEN eintragen. Fuer eine reine "
                "Bahnpruefung --no-gripper setzen "
                "(Episoden werden dann verworfen und sind nicht exportierbar)."
            )

    if sequence is None and args.waypoints is None and not use_sim_robot:
        robot.close()
        raise SystemExit("Am Neura sind --sequence oder --waypoints Pflicht.")

    try:
        captures, cam_cfgs = start_cameras(camera_mode, clock, seed=args.seed,
                                           uvc_device=args.uvc_device,
                                           specs=args.camera)
    except Exception:
        robot.close()
        raise
    # Was die Kameras WIRKLICH tun (Belichtung, Bildrate) -- gesetzt wird
    # nur mit Hinweis bei Fehlschlag, Befund 2026-10-26 (bc/adapters/cam_daheng.py).
    camera_settings = {}
    for cap in captures:
        read_back = cap.camera.settings()
        if read_back:
            camera_settings[cap.name] = read_back
            print("   Kamera %s" % describe_settings(cap.name, read_back))
    reference_images = save_reference_images(captures, cam_cfgs, args.out, args.block)

    kin = Kinematics(robot)
    writer = DatasetWriter(args.out, camera_names=[c.name for c in cam_cfgs])
    # Livebild nur auf ausdruecklichen Wunsch (--preview): es kostet Zeit
    # in der 15-Hz-Schleife, und die Datenqualitaet geht vor (bc/preview.py).
    preview_writer = None
    if args.preview:
        preview_writer = preview.PreviewWriter(
            args.preview, clock,
            rate_hz=args.preview_hz or preview.DEFAULT_RATE_HZ)
        print("Livebild nach %s (%g Hz)"
              % (args.preview, args.preview_hz or preview.DEFAULT_RATE_HZ))

    mode = execution_module.resolve_mode(args.ausfuehrung, robot_kind,
                                         robot_info.get("in_simulation"))
    execution = execution_module.describe(
        mode, transit_speed=args.transit_speed, ptp_speed=args.ptp_speed,
        lead_steps=args.online_vorlauf, speed_factor=args.online_tempo,
        accel_factor=args.online_beschl,
        reach_tol_mm=args.online_halt_mm or None, reach_timeout_s=args.online_halt_s)
    if mode == execution_module.ONLINE:
        print("Ausfuehrung: online (movelinear_online), Vorlauf %d Takte, Tempo x%.2f, "
              "Beschl. x%.2f -- Grenzen %.3f m/s / %.2f rad/s"
              % (execution["lead_steps"], execution["speed_factor"],
                 execution["accel_factor"], execution["vel"][0], execution["vel"][3]))
        if execution_module.reach_rule(execution):
            print("   Greiferwechsel: warten bis %.1f mm am Haltepunkt (hoechstens %.1f s)"
                  % (execution["reach_tol_mm"], execution["reach_timeout_s"]))
        else:
            print("   Greiferwechsel: OHNE Warten (--online-halt-mm 0)")
        if robot_kind == "neura" and robot_info.get("in_simulation"):
            print("   WARNUNG: Die VM (v5.0.0-alpha.102) nimmt movelinear_online an, "
                  "faehrt aber nicht -- der Recorder bricht dann mit 'arm folgt "
                  "nicht' ab. In der VM --ausfuehrung servo_j.")
    else:
        print("Ausfuehrung: servo_j %.0f Hz%s" % (
            args.servo_rate,
            " ohne Geschwindigkeit (servo_j_pos)" if mode == execution_module.SERVO_J_POS else ""))

    progress = EpisodeProgress(preview_writer)
    recorder = EpisodeRecorder(robot, captures, clock,
                               servo_rate_hz=args.servo_rate,
                               monitor=progress,
                               execution=execution)
    # Rueckfahrt beim Zuruecklegen: dieselbe Ausfuehrung und dieselben
    # Pruefungen (Schleppfehler, Stopp), aber ohne Kameras -- nichts davon
    # wird gespeichert.
    put_back_recorder = EpisodeRecorder(robot, [], clock,
                                        servo_rate_hz=args.servo_rate,
                                        execution=execution)
    rng = np.random.default_rng(args.seed)
    staged_scales = None
    if args.noise_fixed:
        noise_mode = "fest"
    elif args.zuruecklegen:
        # Gestaffelt je Lage (bc/put_back.stratified_scales): bei 3 Fahrten
        # je eine schwache, mittlere und starke -- ueber den Datensatz
        # weiterhin U(NOISE_EPISODE_SCALE_RANGE).
        noise_mode = "gestaffelt je Lage"
        staged_scales = stratified_scales(args.episodes, rng, *config.NOISE_EPISODE_SCALE_RANGE)
    else:
        noise_mode = "je Episode zufaellig"
    session = args.session or datetime.now().strftime("%Y-%m-%d")
    # Kennung der Lage fuer die Validierungsaufteilung (apps/train.py):
    # Wiederholungen derselben Lage duerfen nie auf beide Seiten fallen.
    repeat_group = "%s/%s/%d" % (session, args.block or "-", args.seed) if args.zuruecklegen else None
    print(
        "Lauf-Einstellungen: servo_j %.0f Hz, Rauschen %s x%.2f, Seed %d, Ablauf %s"
        % (args.servo_rate, noise_mode if noise_mode == "fest" else noise_mode + " bis",
           args.noise_scale, args.seed, args.sequence or args.waypoints or "Demo")
    )
    if args.zuruecklegen:
        print("Zuruecklegen: %d Fahrt(en) derselben Lage, nach jeder Ablegen am "
              "Greifpunkt und Rueckfahrt zum Start (nicht aufgezeichnet)" % args.episodes)
    elif args.sequence:
        print("Zuruecklegen AUS (--ohne-zuruecklegen): der Arm haelt das Objekt am "
              "Ende an der Uebergabe.")
    if args.probefahrt:
        print("Probefahrt: vor der ersten Episode mit %.0f %% Tempo, danach Freigabe"
              % (100 * args.probefahrt_tempo))
    print(
        "Planer-Tempo: Transit %.3f m/s, Endanflug %.3f m/s, PTP %.2f rad/s%s"
        % (args.transit_speed, args.approach_speed, args.ptp_speed,
           "" if (args.transit_speed, args.approach_speed, args.ptp_speed) == (
               config.TRANSIT_SPEED_MS, config.APPROACH_SPEED_MS,
               config.PTP_JOINT_SPEED_RADS) else "  (abweichend vom Standard)")
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
        "transit_speed_ms": args.transit_speed,
        "approach_speed_ms": args.approach_speed,
        "ptp_joint_speed_rads": args.ptp_speed,
        "segment_ramp_s": config.SEGMENT_RAMP_S,
        "gripper_dwell_s": config.GRIPPER_DWELL_S,
    }
    object_points = [p.strip() for p in args.object_points.split(",") if p.strip()]
    if not use_sim_robot and not args.block:
        print("Hinweis: ohne --block (Objektlage) -- an der Anlage Pflicht (AP 5.2).")

    # Rauschstaerke je Episode (AP 2.4, Auswertung 2026-09-16): gezogen
    # aus NOISE_EPISODE_SCALE_RANGE -- mal fast ideal, mal volle
    # Auslenkung. --noise-fixed nimmt immer genau --noise-scale. Je Lage
    # neu gestaffelt (bei automatischen Lagen mehrmals je Aufruf).
    def draw_scales(first):
        if args.noise_fixed:
            return [args.noise_scale] * args.episodes
        if args.zuruecklegen:
            staged = staged_scales if first else stratified_scales(
                args.episodes, rng, *config.NOISE_EPISODE_SCALE_RANGE)
            return [args.noise_scale * s for s in staged]
        lo, hi = config.NOISE_EPISODE_SCALE_RANGE
        return [args.noise_scale * float(rng.uniform(lo, hi)) for _ in range(args.episodes)]

    recorded = 0
    aborted = False
    used_xy = []
    try:
        # 1.-3. EINMAL je Lage (plan_all): Punkte, Plausibilitaet, ideale
        # Bahn, verrauschte Bahnen, Rueckfahrt.
        if sequence is not None:
            resolved = resolve_points(sequence, robot, use_sim_robot)
            if resolved.skipped:
                print("Optionale Punkte fehlen: %s" % resolved.skipped)
            if resolved.table is not None:
                print(resolved.table.describe())
                for wp in resolved.waypoints:
                    if wp.name == "PICK":
                        print("   PICK: %.1f mm ueber dem Tisch, Neigung %.2f Grad"
                              % (1e3 * table_clearance(resolved.table, wp.pose_quat),
                                 tilt_deg(wp.pose_quat, resolved.table)))
                        break
            if resolved.table_missing:
                print("Tischpunkte fehlen in der Steuerung: %s%s"
                      % (", ".join(resolved.table_missing),
                         "" if resolved.table is not None else
                         " -- ohne Tischreferenz (Untergrenze wie bisher)"))
            waypoints = resolved.waypoints
        else:
            resolved = None
            waypoints = fixed_waypoints
        problems = point_problems(waypoints, resolved)
        if problems:
            raise SystemExit(
                "Abbruch vor jeder Bewegung -- Punkte unplausibel:\n   - %s"
                % "\n   - ".join(problems))
        if args.zuruecklegen:
            # Passt der Ablauf ueberhaupt? Zuruecklegen ist Standard -- ein
            # Ablauf ohne Greifen (reine Bahnpruefung) laeuft dann eben ohne.
            try:
                grasp_index(waypoints)
            except PutBackError as exc:
                print("Zuruecklegen AUS: %s" % exc)
                args.zuruecklegen = False
        if args.lagen > 1 and (not args.zuruecklegen or resolved is None
                               or resolved.table is None):
            raise SystemExit(
                "Automatische Lagen (--lagen %d) brauchen Ablaufdatei, Zuruecklegen und "
                "die Tischpunkte -- dort wird abgelegt." % args.lagen)
        t_plan = time.perf_counter()
        start = plan_lage(args, waypoints, resolved, robot, kin, draw_scales(True), rng,
                          index=0, block=args.block, source="geteacht")
        resolved_start = resolved
        print(duration_line(args, start, time.perf_counter() - t_plan))

        # Probefahrt: langsam, ohne Rauschen, Greifer offen (bc/probe.py)
        if args.probefahrt and any(p is not None for p in start["plans"]):
            run_probe(start["plans"], start["back_plan"], robot, put_back_recorder,
                      args.probefahrt_tempo)

        lage = start
        used_xy[:] = [lage["xy"]] if lage["xy"] is not None else []
        while lage is not None and not aborted:
            plans = lage["plans"]
            resolved = lage["resolved"]
            valid = [i for i, p in enumerate(plans) if p is not None]
            if args.lagen > 1:
                print("Lage %d von %d: Block %s, %s%s"
                      % (lage["index"] + 1, args.lagen, lage["block"] or "-", lage["source"],
                         "" if lage["yaw_deg"] is None else
                         ", Drehung %.0f Grad" % lage["yaw_deg"]))
            next_lage = None
            for ep in range(args.episodes):
                episode_scale = lage["scales"][ep]
                meta = dict(
                    robot_info,
                    mode="noise_injection" if args.noise_scale > 0 else "reference",
                    noise_scale=episode_scale,
                    noise_scale_max=args.noise_scale,
                    noise_scale_mode=noise_mode,
                    noise_amplitude_m=config.NOISE_TRANS_AMPLITUDE_M,
                    noise_rot_amplitude_rad=config.NOISE_ROT_AMPLITUDE_RAD,
                    episode_index=ep,
                    seed=args.seed,
                    recorded_at=datetime.now().isoformat(timespec="seconds"),
                    sequence_file=args.sequence,
                    session=session,
                    block=lage["block"],
                    # Wiederholungen derselben Lage (Zuruecklegen), sonst None
                    repeat_group=(None if repeat_group is None
                                  else repeat_group if lage["index"] == 0
                                  else "%s/L%d" % (repeat_group, lage["index"])),
                    repeat_index=ep if repeat_group else None,
                    repeats=args.episodes if repeat_group else None,
                    # Herkunft der Lage (bc/auto_layout.py): geteacht oder vom
                    # Roboter selbst hingelegt
                    object_source=lage["source"],
                    layout_index=lage["index"],
                    light=args.light,
                    camera_pose=args.camera_pose,
                    object_note=args.object_note,
                    cameras={c.name: c.backend for c in cam_cfgs},
                    # Angeforderte Bildrate und zurueckgelesene Einstellungen:
                    # die Rate entscheidet ueber das Bildalter im Latenzbudget.
                    camera_fps={c.name: fps_label(c) for c in cam_cfgs if c.backend != "sim"},
                    camera_settings=camera_settings,
                    # Vollbild je Kamera beim Start des Aufrufs (Rektifizierung
                    # spaeter, AP 1.4) -- relativ zum Zielordner
                    reference_images=reference_images,
                    # Mitgelerntes Tempo (AP 2.6): der Export verlangt, dass diese
                    # Werte ueber alle Episoden eines Datensatzes gleich sind.
                    planner=planner_meta,
                )

                # 1. Wegpunkte: vorab geplant. Hat sich in der Steuerung seitdem
                # etwas geaendert (Touch-up am Pendant), passt der Plan nicht
                # mehr -- und bei Wiederholungen waere es nicht mehr dieselbe
                # Lage. Dann aufhoeren statt mit alten Punkten weiterzufahren.
                if sequence is not None:
                    if (ep > 0 or lage["index"] > 0) and not same_points(
                            resolve_points(sequence, robot, use_sim_robot).points,
                            resolved_start.points):
                        print("Punkte in der Steuerung haben sich seit dem Start geaendert "
                              "(Touch-up?) -- Lauf beendet. Neu starten, dann wird mit den "
                              "neuen Punkten geplant.")
                        aborted = True
                        break
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
                        # Tischreferenz (bc/table.py): Ebene, Punkte, Greifhoehe
                        table=(resolved.table.as_dict() if resolved.table is not None else None),
                        table_points=resolved.table_points,
                        keepout=[zone.as_dict() for zone in resolved.keepouts],
                        pick_over_table_m=(
                            table_clearance(resolved.table, resolved.points["PICK"]["pose_quat"])
                            if resolved.table is not None and "PICK" in resolved.points
                            else None),
                        # Objektlage als Zahlen (Anwender 2026-10-26): Position und
                        # Drehung des Greifers an PICK -- fuer Auswertung nach Lage
                        **object_layout(resolved),
                    )
                meta["blend_m"] = lage["blend_m"]

                # 2./3. Ideale und verrauschte Bahn: vorab geplant
                plan = plans[ep]
                if plan is None:
                    continue

                # 4. Ausserhalb der Aufzeichnung an den Start
                start_joints = plan.joints_noisy[0]
                away = float(np.max(np.abs(robot.read_state().joints - start_joints)))
                if away > config.START_POSE_TOL_RAD:
                    print("Episode %d: fahre an die Startstellung (%.3f rad entfernt)" % (ep, away))
                    robot.move_to_joints(start_joints)

                # 5./6. Aufzeichnen und ablegen
                log_start = len(getattr(robot, "gripper_log", []))
                progress.start_episode(ep)
                try:
                    episode = recorder.record(plan, metadata=meta)
                except ServoLimitError as exc:
                    # Der Schutz im Adapter hat einen Sollwert abgelehnt -- der
                    # Arm steht irgendwo am Anfang der Bahn, das Objekt liegt
                    # (noch) am Greifpunkt. Nichts ablegen, Lauf beenden.
                    print("Episode %d abgebrochen: %s\n   Der Planer haette das abfangen "
                          "muessen -- Ausgabe 'Gelenktempo' oben pruefen. Arm und Objekt "
                          "von Hand pruefen." % (ep, exc))
                    aborted = True
                    break
                if getattr(robot, "gripper_log", None) is not None:
                    episode.metadata["gripper_log"] = [
                        [t, closed] for t, closed in robot.gripper_log[log_start:]
                    ]
                if use_sim_robot and not episode.discarded:
                    episode.success = True  # Sim: Bahn vollstaendig == Erfolg
                if args.ask_label and not episode.discarded:
                    # VOR dem Ablegen fragen: Verwerfen muss sich noch auf die
                    # Episode auswirken koennen, nicht erst auf die Datei.
                    success, discard = ask_label(ep, len(episode))
                    if discard:
                        episode.discarded = True
                        episode.discard_reason = "vom Bedienenden verworfen"
                        # Kein Erfolgs-Label auf einer verworfenen Episode --
                        # sonst taucht sie in der Bilanz als "ok" auf (der
                        # SimRobot setzt es weiter oben automatisch).
                        episode.success = None
                    else:
                        episode.success = success
                if getattr(robot, "gripper_mode", None) == "dry_run":
                    # ZULETZT und ohne Ausnahme: Der Greifer hat nie geschaltet,
                    # im Action-Vektor steht er trotzdem. Eine solche Episode darf
                    # nicht in einen Datensatz -- auch nicht, wenn das Label oben
                    # "erfolgreich" gesagt hat. bc/lerobot_io.scan_sources weist
                    # sie beim Export zusaetzlich ueber die Metadaten ab.
                    episode.discarded = True
                    episode.discard_reason = "Trockenlauf ohne Greifer (--no-gripper)"
                    episode.success = None
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
                # Laufzeit: eine zu langsam gefahrene Bahn sieht sonst gesund aus
                # (0 % schlechte Frames, kein Abbruch) und wird trotzdem mitgelernt.
                for zeile in timing_findings(episode.metadata):
                    print("   HINWEIS: %s" % zeile)
                for halt in episode.metadata.get("reach_waits") or []:
                    print("   %s" % execution_module.reach_line(halt))

                # 7. Zuruecklegen (bc/put_back.py) -- auch nach der letzten
                # Episode: das Objekt liegt dann fuer den naechsten Aufruf am
                # Greifpunkt, der Arm steht offen am Start. Nach der letzten
                # Fahrt einer Lage: an die NAECHSTE Lage (automatische Lagen),
                # nach der letzten Lage zurueck an den geteachten PICK.
                if args.zuruecklegen:
                    back = lage["back_plan"]
                    if ep == valid[-1]:
                        if lage["index"] + 1 < args.lagen and not stop_requested(args):
                            next_lage = next_auto_lage(args, sequence, robot, kin, rng, start,
                                                       used_xy, draw_scales(False),
                                                       lage["index"] + 1, lage["block"])
                        if stop_requested(args):
                            print("Beenden nach dieser Lage angefordert.")
                        target = next_lage or start
                        back = (target["back_plan"] if target is lage
                                else target["regrip_plan"])
                        if target is not lage:
                            print("Umlegen: Objekt an %s"
                                  % ("Lage %d" % (next_lage["index"] + 1) if next_lage
                                     else "den geteachten PICK (Ausgangslage)"))
                    if not put_back(episode, back, put_back_recorder):
                        aborted = True
                        next_lage = None
                        break
            if next_lage is not None:
                used_xy.append(next_lage["xy"])
            lage = next_lage
    finally:
        for cap in captures:
            cap.stop()
        robot.close()
        if args.nach_ende_teach:
            switch_to_teach(robot)

    if args.block:
        # Fuer die Oberflaeche: welcher Block als naechster frei ist
        # (automatische Lagen verbrauchen mehrere).
        last = args.block
        for _ in range(max(0, len(used_xy) - 1)):
            last = next_block(last)
        print("Naechster freier Block: %s" % next_block(last))

    print()
    if recorded:
        print(metrics.format_summary(metrics.summarize(args.out)))
    else:
        print("Keine Episode aufgezeichnet.")


if __name__ == "__main__":
    main()

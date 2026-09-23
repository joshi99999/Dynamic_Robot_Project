"""
Abnahme der virtuellen Steuerung (Neurapy-OfflineSim) -- AP 0.4 / 0.6.

Prueft, was die als OVA gelieferte virtuelle Control-Box tatsaechlich
bedient, und beantwortet damit die in AP 0.4 offen gelassenen Fragen:
Host/Port, NeuraPy-Version, Servo-Interface und Greiferbefehle. Zusaetzlich
wird der Adapter ``bc.adapters.neura.NeuraRobot`` mitgeprueft -- er ist das
eigentliche Abnahmeobjekt (AP 0.6).

WICHTIG (Falle aus AP 0.4): Ergebnisse aus der virtuellen Steuerung sind
KEINE Anlagenmessung. Das Skript fuehrt ``is_robot_in_simulation()`` in
jeder Ausgabe mit und verweigert Bewegungsbefehle, wenn der Controller
NICHT im Simulationsmodus ist.

AN DER ANLAGE (Nachlauf/Override neu messen, AP 0.6 Punkt 3) nur mit
``--real-robot`` und Bestaetigung. Dann gilt:
    * KEINE Fahrt in die feste Startpose -- der Servotest laeuft um die
      aktuelle Stellung. Arm vorher am Pendant in eine freie Pose fahren.
    * Kleine Auslenkung (``--amplitude``, Default an der Anlage 0.05 rad)
      auf EINEM Gelenk (``--joint``), move_linear-Test entfaellt.
    * Greifertest nur mit ``--gripper``.

Ohne ``--move`` wird nur gelesen und gerechnet, der Roboter bewegt sich
nicht (wie tools/check_ik.py). Erst ``--move`` fuehrt Bewegungen aus.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_sim_robot.py                 # nur lesen
    python tools/check_sim_robot.py --move          # inkl. Bewegungstests
    python tools/check_sim_robot.py --move --rate 60 --overrides 1.0,0.5
    python tools/check_sim_robot.py --move --servo-derivs zero --override 1.0
    python tools/check_sim_robot.py --move --real-robot --rate 60 --overrides 1.0,0.5 --joint 1

``--servo-derivs`` vergleicht, was der Controller aus Geschwindigkeit und
Beschleunigung macht: ``plan`` sendet die analytischen Ableitungen der
Testbahn, ``zero`` sendet Nullen ("am Sollwert anhalten"). Gemessen werden
Nachlauf (Zeitversatz Soll -> Ist), Folgefehler und groesste Auslenkung.
"""

import argparse
import math
import time

import _bootstrap  # noqa: F401

import numpy as np

from bc import config
from bc.adapters.neura import NeuraRobot
from bc.ports import RobotError

#: Ruhige Startpose fuer die Bewegungstests. Bewusst nicht die Nullstellung:
#: die ist gestreckt und liegt nahe an einer Singularitaet.
START_JOINTS = [0.0, -0.26, 1.57, 0.0, 0.78, 0.0]

#: Von der Pipeline benoetigte NeuraPy-Funktionen (AP 1.5.1). Fehlt eine
#: davon, ist der betroffene Teil der Pipeline gegen diesen Controller
#: nicht lauffaehig.
REQUIRED_FUNCTIONS = (
    "init_program",
    "power_on",
    "set_override",
    "get_current_joint_angles",
    "compute_forward_kinematics",
    "compute_inverse_kinematics",
    "activate_servo_interface",
    "servo_j",
    "deactivate_servo_interface",
    "stop",
)

#: Nuetzlich, aber die Pipeline hat dokumentierte Rueckfallpfade (siehe
#: NeuraRobot.get_joint_angles_ts bzw. den Dwell-Automaten in AP 2.1).
OPTIONAL_FUNCTIONS = (
    "get_current_joint_angles_with_timestamp",
    "get_flange_pose",
    "get_tcp_pose_quaternion",
    "grasp",
    "release",
    "gripper",
    "move_joint",
    "move_linear",
    "movelinear_online",
    "stop_movelinear_online",
    "set_joint_speed",
    "set_linear_speed",
)


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def show(label, value):
    print("  %-38s %s" % (label, value))


class Report(object):
    """Sammelt Befunde, damit am Ende eine Abnahmeliste steht."""

    def __init__(self):
        self.ok = []
        self.warn = []
        self.fail = []

    def add(self, level, text):
        getattr(self, level).append(text)
        print("  [%s] %s" % (level.upper(), text))

    def summary(self):
        section("Zusammenfassung")
        for level, items in (("FAIL", self.fail), ("WARN", self.warn)):
            for item in items:
                print("  %-6s %s" % (level, item))
        print(
            "\n  %d ok, %d Warnungen, %d Fehler"
            % (len(self.ok), len(self.warn), len(self.fail))
        )
        return not self.fail


# -- 0. Verbindung ---------------------------------------------------------


def connect(report):
    from neurapy.robot import Robot, SOCKET_ADDRESS, SOCKET_PORT

    section("0. Verbindung und Identitaet")
    show("Socket", "%s:%s" % (SOCKET_ADDRESS, SOCKET_PORT))
    raw = Robot()

    server = getattr(raw, "version", None)
    show("Server-Version", server)
    try:
        from neurapy.robot import VERSION as client_version
    except ImportError:
        client_version = None
    show("Client-Version", client_version)
    if client_version and server and client_version != server:
        report.add(
            "warn",
            "Versionen weichen ab (Client %s, Server %s) -- einzelne "
            "Funktionen der Doku fehlen oder verhalten sich anders."
            % (client_version, server),
        )

    in_sim = None
    try:
        in_sim = raw.is_robot_in_simulation()
    except Exception as exc:
        report.add("warn", "is_robot_in_simulation() nicht abfragbar: %s" % exc)
    show("is_robot_in_simulation()", in_sim)

    try:
        show("get_diagnostics()", raw.get_diagnostics())
    except Exception as exc:
        show("get_diagnostics()", "nicht abfragbar: %s" % exc)

    return raw, in_sim


# -- 1. Funktionsumfang ----------------------------------------------------


def check_functions(raw, report):
    section("1. Funktionsumfang gegen den Bedarf der Pipeline (AP 1.5.1)")
    try:
        available = set(raw.get_functions())
    except Exception as exc:
        report.add("fail", "get_functions() nicht abfragbar: %s" % exc)
        return set()

    show("Funktionen insgesamt", len(available))

    missing = [n for n in REQUIRED_FUNCTIONS if n not in available]
    if missing:
        report.add("fail", "Pflichtfunktionen fehlen: %s" % ", ".join(missing))
    else:
        report.add("ok", "Alle Pflichtfunktionen vorhanden.")

    absent = [n for n in OPTIONAL_FUNCTIONS if n not in available]
    if absent:
        report.add("warn", "Optionale Funktionen fehlen: %s" % ", ".join(absent))

    return available


# -- 2. Zustandsabgriff ----------------------------------------------------


def check_timestamps(bot, report):
    """Punkt 2 der Abnahmeliste AP 0.6: Zeitbasis der *_with_timestamp.

    Unterscheidet drei Faelle, weil sie voellig verschiedene Konsequenzen
    haben: brauchbarer Zeitstempel, verschobene Zeitbasis (z. B. UTC vs.
    Host, korrigierbar), oder gar kein Zeitstempel (Sync nach AP 1.3 ist
    dann gegen diesen Controller nicht pruefbar).
    """
    now = time.time()
    try:
        raw = bot._call("get_current_joint_angles_with_timestamp")
    except Exception as exc:
        report.add(
            "warn",
            "get_current_joint_angles_with_timestamp() nicht nutzbar (%s) -- "
            "die Synchronisation faellt auf die Hostzeit zurueck." % exc,
        )
        return

    bot.get_joint_angles_ts()
    t_joints = bot.last_controller_timestamp
    show("Zeitstempel roh", raw[-1] if isinstance(raw, (list, tuple)) else raw)

    # Kein plausibler Absolutzeitstempel: 0, negativ oder fehlend. Seit
    # 2026-09-14 stempelt der Adapter ohnehin mit der Host-Uhr -- das hier
    # ist die Diagnose, ob der Controller-Zeitstempel spaeter nutzbar waere.
    if t_joints is None:
        report.add(
            "warn",
            "Controller liefert keinen brauchbaren Zeitstempel -- der Adapter "
            "stempelt mit der Host-Uhr (Mitte der RPC-Anfrage). Pruefpunkt "
            "Abnahmeliste AP 0.6 Punkt 2 bleibt an der Anlage offen.",
        )
        return
    skew = now - t_joints

    show("Zeitstempel-Abstand zur Hostzeit", "%.3f s" % skew)
    if abs(skew) > config.SYNC_MAX_SKEW_S:
        report.add(
            "warn",
            "Zeitbasis weicht um %.3f s von der Hostzeit ab (Budget %.3f s) "
            "-- vermutlich UTC vs. Hostzeit, vor der Aufzeichnung klaeren."
            % (skew, config.SYNC_MAX_SKEW_S),
        )
    else:
        report.add("ok", "Zeitstempel liegt im Sync-Budget (%.3f s)." % skew)


def check_state(bot, report):
    section("2. Zustandsabgriff und Tool-Geometrie")
    joints = bot.get_joint_angles()
    show("get_current_joint_angles()", np.round(joints, 4).tolist())
    if len(joints) != bot.dof:
        report.add("fail", "Gelenkzahl %d passt nicht zu dof=%d" % (len(joints), bot.dof))

    check_timestamps(bot, report)

    try:
        has_tool = bot.has_tool_offset()
        show("Tool-Offset hinterlegt", has_tool)
        if not has_tool:
            report.add(
                "warn",
                "Kein Greifer-Tool im Controller -- alle Posen beziehen sich "
                "auf den Flansch, nicht auf die Greiferspitze (AP 2.4, "
                "Vortest Punkt 0).",
            )
    except Exception as exc:
        report.add("warn", "Tool-Offset nicht pruefbar: %s" % exc)

    return np.asarray(joints, dtype=float)


# -- 3. Kinematik ----------------------------------------------------------


def check_kinematics(bot, joints, report):
    section("3. Kinematik ueber den Adapter (FK/IK-Rueckprobe)")
    pose = bot.fk(joints, frame="tool")
    show("fk(q) -> Pose [XYZ|QWXYZ]", np.round(pose, 4).tolist())

    if not bot.pose_is_plausible(pose):
        report.add(
            "fail",
            "TCP liegt %.3f m vom Ursprung -- ausserhalb der Reichweite "
            "(%.1f m). Controller liefert vermutlich Platzhalterwerte."
            % (float(np.linalg.norm(pose[:3])), config.MAX_REACH_M),
        )
        return

    try:
        sol = bot.ik(pose, joints)
    except Exception as exc:
        report.add("fail", "ik() an der aktuellen Pose fehlgeschlagen: %s" % exc)
        return
    show("ik(fk(q)) -> q", np.round(sol, 4).tolist())

    d_q = float(np.max(np.abs(sol - joints)))
    show("max |q_ik - q|", "%.5f rad" % d_q)
    if d_q > config.IK_MAX_DELTA_Q_RAD:
        report.add(
            "fail",
            "IK springt auf einen anderen Loesungszweig (%.4f rad > %.2f) -- "
            "die Seed-Semantik aus AP 2.4 traegt hier nicht."
            % (d_q, config.IK_MAX_DELTA_Q_RAD),
        )
    else:
        report.add("ok", "IK-Rueckprobe seedtreu (%.5f rad)." % d_q)

    back = bot.fk(sol, frame="tool")
    d_pos = float(np.linalg.norm(back[:3] - pose[:3]))
    show("FK-Rueckprobe Positionsfehler", "%.6f m" % d_pos)
    if d_pos > config.IK_FK_TOL_POS_M:
        report.add(
            "fail",
            "FK-Rueckprobe verfehlt die Sollpose um %.4f m (Toleranz %.4f m)."
            % (d_pos, config.IK_FK_TOL_POS_M),
        )
    else:
        report.add("ok", "FK-Rueckprobe innerhalb der Toleranz.")

    t0 = time.time()
    for _ in range(20):
        bot.fk(joints, frame="tool")
    show("Laufzeit fk() inkl. TCP-Roundtrip", "%.1f ms" % ((time.time() - t0) / 20 * 1e3))


# -- 4. Bewegung -----------------------------------------------------------


def check_motion(bot, report):
    section("4. Bewegungsbefehle (move_joint / move_linear)")
    start = list(map(float, bot.get_joint_angles()))
    ok = bot._call(
        "move_joint",
        target_joint=[START_JOINTS],
        speed=25.0,
        acceleration=20.0,
        current_joint_angles=start,
    )
    reached = np.asarray(bot.get_joint_angles(), dtype=float)
    err = float(np.max(np.abs(reached - np.asarray(START_JOINTS))))
    show("move_joint -> Startpose", "%r, Restfehler %.5f rad" % (ok, err))
    if err > 1e-3:
        report.add("fail", "move_joint erreicht die Zielpose nicht (%.4f rad)." % err)
    else:
        report.add("ok", "move_joint erreicht die Zielpose.")

    pose_rpy = bot._call(
        "compute_forward_kinematics",
        joint_angles=list(map(float, reached)),
        target_frame="tool",
        representation="rpy",
    )
    target = list(map(float, pose_rpy))
    lift = 0.05
    target[2] += lift
    try:
        ok = bot._call(
            "move_linear",
            target_pose=[target],
            speed=config.APPROACH_SPEED_MS,
            acceleration=0.05,
            current_joint_angles=list(map(float, bot.get_joint_angles())),
        )
    except RobotError as exc:
        report.add("warn", "move_linear nicht nutzbar: %s" % exc)
        return
    after = bot._call(
        "compute_forward_kinematics",
        joint_angles=list(map(float, bot.get_joint_angles())),
        target_frame="tool",
        representation="rpy",
    )
    got = float(after[2]) - float(pose_rpy[2])
    show("move_linear +%.0f mm in Z" % (lift * 1e3), "%r, erreicht %.4f m" % (ok, got))
    if abs(got - lift) > config.IK_FK_TOL_POS_M:
        report.add("fail", "move_linear verfehlt den Z-Hub um %.4f m." % abs(got - lift))
    else:
        report.add("ok", "move_linear haelt den Z-Hub ein.")


# -- 5. Servo-Interface ----------------------------------------------------


def check_servo(bot, report, rate_hz, derivs="plan", seconds=4.0, amplitude=0.15, joint=0,
                label=""):
    """Sollwertstrom auf EINEM Gelenk -- das Interface, auf dem AP 4 spielt.

    Testbahn: q(t) = q0 + A * (1 - cos(2 pi f t)) / 2 mit analytischer
    Geschwindigkeit und Beschleunigung (``derivs="plan"``) oder Nullen
    (``derivs="zero"``).
    """
    section(
        "5. Servo-Interface bei %.0f Hz, Ableitungen: %s, Gelenk %d%s (AP 0.6 Punkt 3)"
        % (rate_hz, derivs, joint + 1, label)
    )
    q0 = np.asarray(bot.get_joint_angles(), dtype=float)
    period = 1.0 / rate_hz
    steps = int(seconds / period)
    freq = 0.5  # eine volle Hin- und Rueckbewegung in 2 s
    omega = 2 * math.pi * freq

    bot.activate_servo("position")
    late = 0
    commanded, measured = [], []
    t0 = time.time()
    try:
        for i in range(steps):
            t = i * period
            q = q0.copy()
            qd = np.zeros_like(q0)
            qdd = np.zeros_like(q0)
            q[joint] = q0[joint] + amplitude * (1.0 - math.cos(omega * t)) / 2.0
            if derivs == "plan":
                qd[joint] = amplitude * omega * math.sin(omega * t) / 2.0
                qdd[joint] = amplitude * omega * omega * math.cos(omega * t) / 2.0
            bot.servo_j(q, qd, qdd)
            commanded.append(q[joint] - q0[joint])
            measured.append(float(np.asarray(bot.get_joint_angles())[joint]) - q0[joint])
            rest = t0 + (i + 1) * period - time.time()
            if rest > 0:
                time.sleep(rest)
            else:
                late += 1
    except Exception as exc:
        report.add(
            "fail",
            "servo_j fehlgeschlagen: %s: %s -- Signatur/Interface gegen den "
            "Controller pruefen (r.get_doc('servo_j'))." % (type(exc).__name__, exc),
        )
        return
    finally:
        bot.deactivate_servo()

    duration = time.time() - t0
    commanded = np.asarray(commanded)
    measured = np.asarray(measured)
    peak = float(np.max(np.abs(measured))) if len(measured) else 0.0
    lag_steps = _estimate_lag(commanded, measured, max_lag=int(rate_hz))
    show("Zyklen", "%d in %.2f s (Soll %.2f s)" % (steps, duration, steps * period))
    show("Zyklen ueber Budget", "%d (%.0f %%)" % (late, 100.0 * late / max(1, steps)))
    show("groesste erreichte Auslenkung J%d" % (joint + 1), "%.4f rad (Soll %.4f)" % (peak, amplitude))
    show("Nachlauf Soll -> Ist", "%d Takte = %.0f ms" % (lag_steps, 1000.0 * lag_steps * period))
    report.add("ok", "Nachlauf%s: %.0f ms bei %.0f Hz" % (label, 1000.0 * lag_steps * period, rate_hz))
    show(
        "Folgefehler J%d (RMS / max)" % (joint + 1),
        "%.4f / %.4f rad"
        % (
            float(np.sqrt(np.mean((commanded - measured) ** 2))),
            float(np.max(np.abs(commanded - measured))),
        ),
    )
    show("letzter servo_j-Code", bot.last_servo_code)

    if late:
        report.add(
            "warn",
            "%d von %d Zyklen haben die Periode gerissen -- bei %.0f Hz ist "
            "das Latenzbudget aus AP 1.3 nicht sicher einzuhalten."
            % (late, steps, rate_hz),
        )
    if peak < amplitude * 0.5:
        report.add(
            "warn",
            "Der Arm folgt dem Sollwertstrom nur zu %.0f %% -- die Ausfuehrung "
            "haengt merklich hinterher (Override/Trajektorien-Skalierung "
            "pruefen, siehe AP 2.6)." % (100.0 * peak / amplitude),
        )
    else:
        report.add("ok", "Servo-Interface folgt dem Sollwertstrom bei %.0f Hz." % rate_hz)

    try:
        show("Trajektorien-Skalierung", bot._call("get_servo_trajectory_scaling_factor"))
    except Exception:
        pass


def _estimate_lag(commanded, measured, max_lag):
    """Zeitversatz in Takten, bei dem die Ist-Kurve der Soll-Kurve am
    besten folgt (kleinster mittlerer quadratischer Fehler)."""
    best, best_err = 0, float("inf")
    for lag in range(0, min(max_lag, len(commanded) - 1) + 1):
        a = commanded[: len(commanded) - lag]
        b = measured[lag:]
        err = float(np.mean((a - b) ** 2))
        if err < best_err:
            best, best_err = lag, err
    return best


# -- 6. Greifer ------------------------------------------------------------


def check_gripper(bot, report):
    section("6. Greifer (binaer, AP 2.1)")
    show("Greifer-Modus", bot.gripper_mode)
    try:
        bot.gripper_command(False)
        bot.gripper_command(True)
        bot.gripper_command(False)
    except Exception as exc:
        report.add(
            "warn",
            "Greiferbefehle nicht bedient (%s) -- Greifer-Totzeit (AP 0.6 "
            "Punkt 5) bleibt an der Anlage zu messen." % exc,
        )
        return
    if bot.gripper_mode == "logged":
        report.add(
            "warn",
            "Kein Greifer konfiguriert -- Befehle wurden nur protokolliert "
            "(%d Eintraege). Fuer die Datenpipeline ausreichend." % len(bot.gripper_log),
        )
    else:
        report.add("ok", "grasp()/release() werden bedient.")


# -- Ablauf ----------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--move",
        action="store_true",
        help="Bewegungs- und Servotests ausfuehren (ohne dies wird nur gelesen)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=config.CONTROL_RATE_HZ,
        help="Servo-Rate in Hz (Vorgabe: config.CONTROL_RATE_HZ)",
    )
    parser.add_argument(
        "--servo-derivs",
        choices=("plan", "zero"),
        default="plan",
        help="Geschwindigkeit/Beschleunigung fuer servo_j: analytisch oder Nullen",
    )
    parser.add_argument(
        "--override",
        type=float,
        default=config.DEFAULT_OVERRIDE,
        help="Geschwindigkeits-Override beim Verbinden (0-1)",
    )
    parser.add_argument(
        "--overrides", default=None,
        help="Servotest nacheinander mit diesen Overrides, z. B. 1.0,0.5 (Nachlauf je Override)",
    )
    parser.add_argument("--joint", type=int, default=1, help="Gelenk fuer den Servotest (1-6)")
    parser.add_argument("--amplitude", type=float, default=None,
                        help="Auslenkung in rad (Default: VM 0.15, Anlage 0.05)")
    parser.add_argument("--real-robot", action="store_true",
                        help="REALE Anlage freigeben (mit Bestaetigung, siehe Docstring)")
    parser.add_argument("--gripper", action="store_true",
                        help="an der Anlage auch den Greifer schalten")
    args = parser.parse_args()

    report = Report()
    raw, in_sim = connect(report)
    real = in_sim is not True

    move = args.move
    if move and real and not args.real_robot:
        report.add(
            "fail",
            "is_robot_in_simulation() ist %r -- Bewegungstests werden "
            "verweigert. An der Anlage nur mit --real-robot." % (in_sim,),
        )
        move = False
    if move and real:
        print("\n!! --real-robot: Servotest an der REALEN ANLAGE um die aktuelle Stellung,")
        print("   Gelenk %d, Auslenkung %.3f rad. Arbeitsraum frei? Not-Aus in Reichweite?"
              % (args.joint, args.amplitude or 0.05))
        if input("   Zum Fortfahren 'ANLAGE' eintippen: ").strip() != "ANLAGE":
            raise SystemExit("Abgebrochen.")
    amplitude = args.amplitude if args.amplitude is not None else (0.05 if real else 0.15)
    overrides = (
        [float(v) for v in args.overrides.split(",")] if args.overrides else [args.override]
    )

    check_functions(raw, report)

    # Der Adapter prueft is_robot_in_simulation() beim Verbinden selbst noch
    # einmal und verweigert Bewegungen ohne bestaetigte Simulation.
    bot = NeuraRobot(robot=raw, override=overrides[0], allow_real=args.real_robot)
    bot.connect(power_on=move, ensure_automatic=move)
    show("Override", overrides[0])
    try:
        joints = check_state(bot, report)
        check_kinematics(bot, joints, report)
        if move:
            if not real:
                check_motion(bot, report)
            for override in overrides:
                bot._call("set_override", override)
                check_servo(bot, report, args.rate, derivs=args.servo_derivs,
                            amplitude=amplitude, joint=args.joint - 1,
                            label=", Override %.2f" % override)
            if not real or args.gripper:
                check_gripper(bot, report)
        else:
            print("\n(Bewegungs-, Servo- und Greifertests uebersprungen -- --move)")
    finally:
        bot.close()

    passed = report.summary()
    if real:
        print("\nHinweis: is_robot_in_simulation() == %r -- ANLAGENMESSUNG." % (in_sim,))
    else:
        print(
            "\nHinweis: Alle Ergebnisse stammen aus der VIRTUELLEN Steuerung "
            "(is_robot_in_simulation() == %r).\nSie sind keine Anlagenmessung -- "
            "Raten, Jitter, Latenzen, Greifer-Totzeit und\nKollisionsmodell "
            "bleiben Punkte der Abnahmeliste AP 0.6." % (in_sim,)
        )
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()

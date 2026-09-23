"""Kinematik und Achsgrenzen gegen den Controller pruefen (AP 0.4 Gleis B, AP 0.6 Punkt 4).

NUR LESEN UND RECHNEN -- der Roboter bewegt sich nicht, es wird nichts
bestromt (Vorbild tools/check_ik.py). Laeuft gegen VM und Anlage gleich.

Der Planer, das Rauschen und die Kollisionspruefung wurden bisher gegen die
virtuelle Steuerung entwickelt. Traegt das an der Anlage? Die Kinematik
steckt im Controller (compute_forward/inverse_kinematics) -- sie laesst sich
also OHNE Bewegung vergleichen:

1. Controller-FK fuer die 50 Gelenkstellungen der VM-Referenz
   (tests/data/golden_fk_lara5_sim.json) -- gleiches Modell wie in der VM?
   Verglichen werden ``wrist`` und ``elbow`` (unabhaengig vom Tool) und
   ``tool``, solange kein Tool-Offset hinterlegt ist.
2. IK-Rueckprobe an denselben Posen (Seed = Referenzstellung).
3. Gemeldete TCP-Pose (get_tcp_pose_quaternion) gegen FK der aktuellen
   Gelenkstellung -- die Annahme, auf der read_state() beruht.
4. Achsgrenzen: config.JOINT_LIMITS_RAD (aus der URDF, UNVERIFIZIERT) und
   alle Controller-Funktionen mit "limit" im Namen; lesende ``get_*``-
   Funktionen ohne Argumente werden abgefragt.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_kinematics.py
"""

import json
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config, geometry
from bc.adapters.neura import NeuraRobot

GOLDEN = Path(__file__).resolve().parent.parent / "tests" / "data" / "golden_fk_lara5_sim.json"
TOL_POS_M = 1e-3
TOL_ROT_RAD = 1e-2


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def show(label, value):
    print("  %-40s %s" % (label, value))


def main():
    report = {"erzeugt": datetime.now().isoformat(timespec="seconds"), "befunde": []}

    def finding(level, text):
        report["befunde"].append([level, text])
        print("  [%s] %s" % (level.upper(), text))

    section("0. Verbindung (ohne Bestromung)")
    bot = NeuraRobot()  # allow_real egal: es wird nichts bewegt
    bot.connect(power_on=False, ensure_automatic=False)
    raw = bot.robot
    report["in_simulation"] = bot.in_simulation
    report["tool"] = bot.tool_name
    show("is_robot_in_simulation()", bot.in_simulation)
    show("Tool", bot.tool_name)
    show("Server-Version", getattr(raw, "version", None))
    try:
        has_tool = bot.has_tool_offset()
    except Exception as exc:
        has_tool = None
        finding("warn", "Tool-Offset nicht pruefbar: %s" % exc)
    show("Tool-Offset hinterlegt", has_tool)

    golden = json.loads(GOLDEN.read_text(encoding="utf-8"))
    frames = ["wrist", "elbow"] + (["tool"] if has_tool is False else [])
    try:
        section("1. Controller-FK gegen die VM-Referenz (%d Stellungen, Frames %s)"
                % (len(golden["samples"]), ", ".join(frames)))
        pos_err = {f: [] for f in frames}
        rot_err = {f: [] for f in frames}
        for sample in golden["samples"]:
            for frame in frames:
                pose = bot.fk(sample["q"], frame=frame)
                ref = np.asarray(sample["poses"][frame])
                pos_err[frame].append(float(np.linalg.norm(pose[:3] - ref[:3])))
                rot_err[frame].append(geometry.quat_angle_between(pose[3:7], ref[3:7]))
        report["fk_vs_vm"] = {}
        for frame in frames:
            p, r = max(pos_err[frame]), max(rot_err[frame])
            report["fk_vs_vm"][frame] = {"max_pos_m": p, "max_rot_rad": r}
            show("%s: max Positions-/Winkelfehler" % frame, "%.2f mm / %.4f rad" % (1e3 * p, r))
            if p <= TOL_POS_M and r <= TOL_ROT_RAD:
                finding("ok", "Frame %s: Controller-FK wie in der VM" % frame)
            else:
                finding("fail", "Frame %s weicht von der VM ab (%.1f mm) -- Planer, Kollisions"
                        "modell und VM-Ergebnisse nicht ungeprueft uebertragen" % (frame, 1e3 * p))
        if has_tool:
            finding("warn", "Tool-Offset hinterlegt -- Frame 'tool' nicht mit der VM (NoTool) "
                    "vergleichbar; Tool-Geometrie gegen das Greifer-Datenblatt pruefen (AP 0.6 Punkt 6)")

        section("2. IK-Rueckprobe an den Referenzposen")
        bad, errors = 0, []
        for sample in golden["samples"]:
            pose = bot.fk(sample["q"], frame="tool")
            try:
                sol = bot.ik(pose, sample["q"])
            except Exception:
                bad += 1
                continue
            errors.append(float(np.max(np.abs(np.asarray(sol) - np.asarray(sample["q"])))))
        report["ik_roundtrip"] = {"failed": bad, "max_dq_rad": max(errors) if errors else None}
        show("ohne Loesung", "%d von %d" % (bad, len(golden["samples"])))
        if errors:
            show("max |q_ik - q_ref|", "%.5f rad" % max(errors))
            level = "ok" if max(errors) < config.IK_MAX_DELTA_Q_RAD and not bad else "warn"
            finding(level, "IK-Rueckprobe: %d ohne Loesung, max %.4f rad" % (bad, max(errors)))

        section("3. Gemeldete TCP-Pose gegen FK der aktuellen Stellung")
        joints = bot.get_joint_angles()
        fk = bot.fk(joints, frame="tool")
        reported = np.asarray(bot.get_tcp_pose_quaternion(), dtype=float)
        d_pos = float(np.linalg.norm(fk[:3] - reported[:3]))
        d_rot = geometry.quat_angle_between(fk[3:7], reported[3:7])
        report["tcp_vs_fk"] = {"pos_m": d_pos, "rot_rad": d_rot, "joints": list(joints)}
        show("aktuelle Stellung", np.round(joints, 4).tolist())
        show("Abweichung", "%.2f mm / %.4f rad" % (1e3 * d_pos, d_rot))
        finding("ok" if d_pos <= TOL_POS_M and d_rot <= TOL_ROT_RAD else "fail",
                "get_tcp_pose_quaternion vs. FK: %.2f mm" % (1e3 * d_pos))

        section("4. Achsgrenzen")
        report["joint_limits_config"] = config.JOINT_LIMITS_RAD
        for i, (lo, hi) in enumerate(config.JOINT_LIMITS_RAD):
            show("J%d config (URDF, unverifiziert)" % (i + 1), "[%.3f, %.3f] rad" % (lo, hi))
        try:
            names = sorted(n for n in raw.get_functions() if "limit" in n.lower())
        except Exception as exc:
            names = []
            finding("warn", "get_functions() nicht abfragbar: %s" % exc)
        report["limit_functions"] = {}
        show("Controller-Funktionen mit 'limit'", ", ".join(names) or "-")
        for name in names:
            if not name.startswith("get_"):
                continue
            try:
                value = getattr(raw, name)()
            except Exception as exc:
                value = "nicht ohne Argumente abfragbar (%s)" % type(exc).__name__
            report["limit_functions"][name] = value if isinstance(value, (str, int, float, list, dict)) else repr(value)
            show(name, value)
        finding("warn", "Achsgrenzen mit Datenblatt/Pendant abgleichen und dann "
                "config.JOINT_LIMITS_VERIFIED = True setzen (der ServoGuard nutzt sie)")
    finally:
        bot.close()

    out = Path("Berichte") / ("%s_Kinematik-%s.json" % (
        datetime.now().strftime("%Y-%m-%d_%H-%M"), "VM" if bot.in_simulation else "Anlage"))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False, default=str), encoding="utf-8")
    section("Zusammenfassung")
    for level, text in report["befunde"]:
        if level != "ok":
            print("  %-5s %s" % (level.upper(), text))
    print("  Protokoll: %s" % out)


if __name__ == "__main__":
    main()

"""
IK-Verhaltenstest fuer die Noise-Injection-Umrechnung (siehe AP 2.4 in
Requierments/requierments.md).

Bewegt den Roboter NICHT - compute_inverse_kinematics/compute_forward_kinematics
sind reine Berechnungsfunktionen ohne Bewegungsausfuehrung. Gefahrlos ausfuehrbar,
sobald `from neurapy.robot import Robot; r = Robot()` moeglich ist.

Voraussetzung fuer aussagekraeftige Ergebnisse: Der Greifer muss mechanisch
montiert UND seine Geometrie im Tool-GUI/`create_tool` hinterlegt sein - die
elektrische/pneumatische Anbindung ist dafuer NICHT noetig. Ohne Tool-Eintrag
rechnet die API relativ zum Flansch statt zur Greiferspitze (siehe Test 0).

Ausfuehren:
    python tools/check_ik.py
"""
 
import math
import time
import random

import numpy as np
from neurapy.robot import Robot

# Ungefaehre Reichweite der LARA 5 in Metern - nur zur Plausibilitaetspruefung
# der vom Controller gemeldeten Pose (siehe check_pose_plausible).
MAX_REACH_M = 0.9


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def quat_pose_to_rpy(pose):
    """[X,Y,Z,W,EX,EY,EZ] -> [X,Y,Z,R,P,Y], lokal berechnet.

    Wird nur als letzte Rueckfallebene benutzt. Bevorzugt wird die RPY-Pose
    direkt vom Controller geholt (compute_forward_kinematics), damit exakt
    dieselbe Winkelkonvention wie in compute_inverse_kinematics gilt.
    """
    x, y, z, qw, qx, qy, qz = pose
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n == 0:
        raise ValueError("Quaternion mit Norm 0")
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return [x, y, z, roll, pitch, yaw]


def get_rpy_pose(r, base_joint):
    """XYZRPY-Pose in der Konvention des Controllers besorgen.

    Reihenfolge bewusst so gewaehlt:
    1. compute_forward_kinematics - garantiert dieselbe Konvention wie die IK
       und braucht keine separate Konvertierungsfunktion.
    2. get_tcp_pose - liefert laut Doku bereits XYZRPY.
    3. get_tcp_pose_quaternion + lokale Umrechnung (Konvention ungeprueft).
    """
    try:
        pose = r.compute_forward_kinematics(
            joint_angles=base_joint, representation="rpy"
        )
        if pose:
            print("Pose-Quelle: compute_forward_kinematics(representation='rpy')")
            return list(pose)
    except Exception as exc:
        print("compute_forward_kinematics nicht nutzbar:", exc)

    try:
        pose = r.get_tcp_pose()
        if pose and len(pose) == 6:
            print("Pose-Quelle: get_tcp_pose()")
            return list(pose)
    except Exception as exc:
        print("get_tcp_pose nicht nutzbar:", exc)

    pose = r.get_tcp_pose_quaternion()
    print("Pose-Quelle: get_tcp_pose_quaternion() + lokale Umrechnung")
    print("  ACHTUNG: RPY-Konvention lokal angenommen (ZYX) und nicht gegen")
    print("  den Controller verifiziert - Ergebnisse mit Vorsicht bewerten.")
    return quat_pose_to_rpy(list(pose))


def check_pose_plausible(r, pose, joint):
    """Erkennt Platzhalter-/Dummy-Werte, bevor sinnlose Tests laufen."""
    section("Vorpruefung: Liefert der Controller echte Werte?")
    dist = math.sqrt(pose[0] ** 2 + pose[1] ** 2 + pose[2] ** 2)
    print("Abstand TCP vom Roboterursprung: %.3f m" % dist)

    try:
        in_sim = r.is_robot_in_simulation()
        print("is_robot_in_simulation():", in_sim)
    except Exception as exc:
        print("is_robot_in_simulation() nicht abfragbar:", exc)

    problems = []
    if dist > MAX_REACH_M:
        problems.append(
            "TCP-Abstand %.3f m liegt ausserhalb der LARA-5-Reichweite "
            "(~%.1f m)." % (dist, MAX_REACH_M)
        )
    if all(abs(q) < 1e-9 for q in joint):
        problems.append("Alle Gelenkwinkel exakt 0 - untypisch fuer eine reale Pose.")

    if problems:
        print("\nWARNUNG - die gemeldeten Werte wirken wie Platzhalter:")
        for p in problems:
            print("  -", p)
        print(
            "\nMoegliche Ursachen: Controller im Simulations-/Nicht-"
            "initialisierten\nZustand, kein Programm aktiv (init_program()), "
            "oder der Roboter ist\nnicht betriebsbereit. Die nachfolgenden "
            "IK-Tests waeren auf solchen\nWerten wertlos, weil jedes Ziel "
            "ausserhalb des Arbeitsraums laege.\n"
        )
        return False

    print("OK: Werte wirken plausibel.")
    return True


def test_tool_offset(r):
    section("0. Tool-/TCP-Kalibrierung (Voraussetzung)")
    flange = r.get_flange_pose()
    tcp = r.get_tcp_pose_quaternion()
    print("Flansch-Pose :", np.round(flange, 4))
    print("TCP-Pose     :", np.round(tcp, 4))
    offset = np.linalg.norm(np.array(flange[:3]) - np.array(tcp[:3]))
    print("Translations-Offset Flansch->TCP: %.4f m" % offset)
    if offset < 1e-4:
        print(
            "\nWARNUNG: Offset ~0 -> vermutlich ist kein Tool/Greifer-Offset\n"
            "         im Controller hinterlegt (Tool-DB zeigt evtl. 'NoTool').\n"
            "         Alle nachfolgenden IK-Ziele wuerden sich auf den Flansch,\n"
            "         nicht auf die Greiferspitze beziehen. Vor dem Fortfahren\n"
            "         die Greifer-Geometrie im Tool-GUI eintragen."
        )
    else:
        print("OK: Es ist ein Tool-Offset konfiguriert.")


def sample_offsets(n, max_trans=0.02, max_rot_rad=0.05, rng=None):
    rng = rng or random.Random(0)
    offsets = []
    for _ in range(n):
        d = np.array([rng.uniform(-max_trans, max_trans) for _ in range(3)])
        rot = np.array([rng.uniform(-max_rot_rad, max_rot_rad) for _ in range(3)])
        offsets.append((d, rot))
    return offsets


def test_reachability(r, base_pose, base_joint, n=100, max_trans=0.02):
    section("1. Erreichbarkeit im Rausch-Korridor (+-%.0f mm)" % (max_trans * 1000))
    ok, fail = 0, 0
    fail_examples = []
    ref = base_joint
    for d, rot in sample_offsets(n, max_trans=max_trans):
        target = list(base_pose)
        target[0] += d[0]
        target[1] += d[1]
        target[2] += d[2]
        target[3] += rot[0]
        target[4] += rot[1]
        target[5] += rot[2]
        try:
            sol = r.compute_inverse_kinematics(
                target_pose=target, reference_joint=ref, representation="rpy"
            )
            ref = sol  # warm start fuer naechsten Sample
            ok += 1
        except Exception as exc:  # IKNotFound o.ae.
            fail += 1
            if len(fail_examples) < 3:
                fail_examples.append((target, str(exc)))
    print("Erfolgreich: %d / %d (%.1f%%)" % (ok, n, 100 * ok / n))
    if fail:
        print("Fehlgeschlagen: %d - Beispiele:" % fail)
        for target, err in fail_examples:
            print("  Ziel:", np.round(target, 4), "->", err)


def test_seed_consistency(r, base_pose, base_joint, n=50, step=0.001,
                           jump_threshold_rad=0.2):
    section("2. Seed-Konsistenz entlang eines dichten Pfads")
    ref = base_joint
    max_delta = 0.0
    jumps = []
    pose = list(base_pose)
    for i in range(n):
        pose[0] += step  # kleine Schritte in eine Richtung, z.B. X
        try:
            sol = r.compute_inverse_kinematics(
                target_pose=pose, reference_joint=ref, representation="rpy"
            )
        except Exception as exc:
            print("  Schritt %d: IK fehlgeschlagen (%s)" % (i, exc))
            break
        delta = np.max(np.abs(np.array(sol) - np.array(ref)))
        if delta > jump_threshold_rad:
            jumps.append((i, delta))
        max_delta = max(max_delta, delta)
        ref = sol
    print("Max. |Delta q| pro Schritt: %.4f rad" % max_delta)
    if jumps:
        print(
            "WARNUNG: %d Sprung/Spruenge > %.2f rad erkannt trotz Warm-Start "
            "(vermutlich Konfigurations-/Zweigwechsel):" % (len(jumps), jump_threshold_rad)
        )
        for i, d in jumps:
            print("  Schritt %d: |Delta q| = %.4f rad" % (i, d))
    else:
        print("OK: keine auffaelligen Spruenge.")


def test_singularity_sensitivity(r, base_pose, base_joint, eps=0.002):
    section("3. Singularitaetsnaehe an der aktuellen (Greif-)Pose")
    directions = [
        ("+X", [eps, 0, 0, 0, 0, 0]),
        ("-X", [-eps, 0, 0, 0, 0, 0]),
        ("+Y", [0, eps, 0, 0, 0, 0]),
        ("-Y", [0, -eps, 0, 0, 0, 0]),
        ("+Z", [0, 0, eps, 0, 0, 0]),
        ("-Z", [0, 0, -eps, 0, 0, 0]),
    ]
    print("Test-Auslenkung: %.1f mm" % (eps * 1000))
    ratios = []
    for name, d in directions:
        target = [base_pose[i] + d[i] for i in range(6)]
        try:
            sol = r.compute_inverse_kinematics(
                target_pose=target, reference_joint=base_joint, representation="rpy"
            )
            dq = np.max(np.abs(np.array(sol) - np.array(base_joint)))
            ratio = dq / eps
            ratios.append(ratio)
            print("  Richtung %-3s: |Delta q|max = %.4f rad  (Verhaeltnis dq/dx = %.1f)"
                  % (name, dq, ratio))
        except Exception as exc:
            print("  Richtung %-3s: IK fehlgeschlagen (%s)" % (name, exc))
    if ratios and max(ratios) > 20:
        print(
            "\nWARNUNG: Hohes dq/dx-Verhaeltnis an dieser Pose -> moegliche "
            "Naehe zu einer Singularitaet (z.B. Handgelenk bei senkrechtem "
            "Greifer). Diesen Wegpunkt ggf. anders orientieren oder das "
            "Rauschen dort staerker daempfen."
        )


def test_representation(r, base_pose, base_joint):
    section("4. Repraesentations-Vergleich (RPY vs. Quaternion)")
    try:
        sol_rpy = r.compute_inverse_kinematics(
            target_pose=base_pose, reference_joint=base_joint, representation="rpy"
        )
        quat_pose = r.convert_euler_to_quaternion_pose(base_pose)
        sol_quat = r.compute_inverse_kinematics(
            target_pose=quat_pose, reference_joint=base_joint, representation="quaternion"
        )
        delta = np.max(np.abs(np.array(sol_rpy) - np.array(sol_quat)))
        print("Max. Differenz RPY- vs. Quaternion-Loesung: %.5f rad" % delta)
        if delta > 1e-3:
            print(
                "WARNUNG: Loesungen weichen spuerbar ab - moeglicher "
                "Gimbal-Lock-Effekt in der RPY-Repraesentation an dieser "
                "Pose. Fuer senkrechte Greif-Orientierungen 'quaternion' "
                "bevorzugen."
            )
        else:
            print("OK: beide Repraesentationen konsistent.")
    except Exception as exc:
        print("Test uebersprungen/fehlgeschlagen:", exc)


def test_timing(r, base_pose, base_joint, n=20):
    section("5. Laufzeit pro IK-/FK-Aufruf (TCP-Roundtrip inklusive)")
    t0 = time.perf_counter()
    for _ in range(n):
        r.compute_inverse_kinematics(
            target_pose=base_pose, reference_joint=base_joint, representation="rpy"
        )
    dt_ik = (time.perf_counter() - t0) / n
    t0 = time.perf_counter()
    for _ in range(n):
        r.compute_forward_kinematics(joint_angles=base_joint)
    dt_fk = (time.perf_counter() - t0) / n
    print("compute_inverse_kinematics: %.1f ms/Aufruf" % (dt_ik * 1000))
    print("compute_forward_kinematics: %.1f ms/Aufruf" % (dt_fk * 1000))
    print(
        "\nHinweis: Fuer eine Trajektorie mit z.B. 150 Zeitschritten und "
        "etwas Rejection Sampling ergibt sich daraus die zu erwartende "
        "Generierungsdauer (unkritisch, da offline, aber relevant fuers "
        "Iterationstempo beim Parameter-Tuning)."
    )


def main():
    r = Robot()

    base_joint = r.get_current_joint_angles()
    base_pose = get_rpy_pose(r, base_joint)

    print("\nAktuelle Pose (XYZRPY):", np.round(base_pose, 4))
    print("Aktuelle Gelenkwinkel :", np.round(base_joint, 4))

    if not check_pose_plausible(r, base_pose, base_joint):
        print(
            "Abbruch: Erst die Verbindung/den Roboterzustand klaeren, dann\n"
            "das Skript erneut ausfuehren."
        )
        return

    test_tool_offset(r)
    test_reachability(r, base_pose, base_joint)
    test_seed_consistency(r, base_pose, base_joint)
    test_singularity_sensitivity(r, base_pose, base_joint)
    test_representation(r, base_pose, base_joint)
    test_timing(r, base_pose, base_joint)

    section("Fertig")
    print(
        "Wichtig: Diese Tests an der AKTUELLEN Pose sagen nur etwas ueber\n"
        "diese eine Konfiguration aus. Fuer aussagekraeftige Ergebnisse den\n"
        "Roboter (per Gamepad/Freidrehen) nacheinander in die tatsaechlich\n"
        "geteachten Wegpunkte fahren (Anfahrt, Greifpose, Hebepose, Ziel)\n"
        "und das Skript an jedem davon erneut ausfuehren - insbesondere an\n"
        "der Greifpose (Test 3, Singularitaetsnaehe) ist das entscheidend."
    )


if __name__ == "__main__":
    main()

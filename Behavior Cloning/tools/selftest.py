"""Selbsttest der hardwarefreien Module (Phase 0).

Prueft Geometrie und Kollisionsmodell ohne Roboter, ohne Kameras und ohne
SDKs -- laeuft also auf jedem Rechner. Sinnvoll als schnelle Kontrolle nach
Aenderungen, bevor man an die echte Anlage geht.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/selftest.py
"""

import math

import _bootstrap  # noqa: F401

import numpy as np

from bc import collision, geometry

FAILURES = []


def check(name, condition, detail=""):
    if condition:
        print("  OK   %s" % name)
    else:
        print("  FAIL %s %s" % (name, detail))
        FAILURES.append(name)


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def test_geometry():
    section("Geometrie: Quaternionen und Posen")

    # Hin- und Rueckkonvertierung an einer harmlosen Orientierung
    rpy = (0.3, -0.7, 1.2)
    q = geometry.quat_from_rpy(*rpy)
    back = geometry.quat_to_rpy(q)
    check(
        "RPY -> Quat -> RPY reproduziert die Winkel",
        np.allclose(rpy, back, atol=1e-9),
        "%s vs %s" % (rpy, back),
    )

    # Der eigentliche Grund fuer dieses Modul: die reale Arbeitsorientierung
    # liegt exakt am +/-pi-Umschlagpunkt (siehe tools/log.txt).
    pose_a = np.array([0.4366, -0.0885, 0.4345, -math.pi + 1e-4, 0.0, -math.pi + 1e-4])
    pose_b = np.array([0.4366, -0.0885, 0.4345, math.pi - 1e-4, 0.0, math.pi - 1e-4])
    qa = geometry.pose_rpy_to_quat(pose_a)
    qb = geometry.pose_rpy_to_quat(pose_b)
    angle = geometry.quat_angle_between(qa[3:7], qb[3:7])
    check(
        "Orientierungen bei -pi und +pi gelten als (fast) gleich",
        angle < 1e-3,
        "Winkel = %.6f rad" % angle,
    )

    # Und genau hier scheitert naive RPY-Arithmetik -- das ist der Grund,
    # warum der Planer in Quaternionen rechnen muss.
    naive = float(np.max(np.abs(pose_b[3:] - pose_a[3:])))
    check(
        "naive RPY-Differenz zeigt den 2*pi-Sprung (erwartetes Fehlverhalten)",
        naive > 6.0,
        "Differenz = %.4f rad" % naive,
    )

    mid = geometry.slerp(qa[3:7], qb[3:7], 0.5)
    check(
        "SLERP bleibt am Umschlagpunkt bei der Ausgangsorientierung",
        geometry.quat_angle_between(mid, qa[3:7]) < 1e-3,
        "Abweichung = %.6f rad" % geometry.quat_angle_between(mid, qa[3:7]),
    )

    # Bahnerzeugung
    path = geometry.pose_path([qa, qb], steps_per_segment=10)
    check("pose_path liefert N+1 Posen", path.shape == (11, 7), str(path.shape))
    check(
        "alle Quaternionen der Bahn sind normiert",
        np.allclose(np.linalg.norm(path[:, 3:7], axis=1), 1.0, atol=1e-9),
    )

    # Interpolation der Position
    p0 = geometry.pose_rpy_to_quat([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    p1 = geometry.pose_rpy_to_quat([1.0, 2.0, 3.0, 0.0, 0.0, 0.0])
    half = geometry.pose_interpolate(p0, p1, 0.5)
    check(
        "Position wird linear interpoliert",
        np.allclose(half[:3], [0.5, 1.0, 1.5]),
        str(half[:3]),
    )


def test_collision():
    section("Kollisionsmodell: Quader und Freiraum")

    box = collision.Box.from_bounds("Tisch", lo=(-1, -1, -0.5), hi=(1, 1, 0.0))

    check(
        "Punkt deutlich ueber dem Quader hat positiven Abstand",
        abs(box.distance_to_point([0, 0, 0.30]) - 0.30) < 1e-9,
        str(box.distance_to_point([0, 0, 0.30])),
    )
    check(
        "Punkt im Quader hat negativen Abstand",
        box.distance_to_point([0, 0, -0.25]) < 0,
        str(box.distance_to_point([0, 0, -0.25])),
    )
    check(
        "Huellkugel knapp ueber der Oberflaeche kollidiert",
        box.clearance([0, 0, 0.03], radius=0.06) < 0,
        str(box.clearance([0, 0, 0.03], radius=0.06)),
    )
    check(
        "Huellkugel mit Abstand kollidiert nicht",
        box.clearance([0, 0, 0.30], radius=0.06) > 0,
        str(box.clearance([0, 0, 0.30], radius=0.06)),
    )

    # Geofence: invertierte Logik -- der Arm muss drin bleiben
    fence = collision.Box.from_bounds(
        "Arbeitsraum", lo=(-0.5, -0.5, 0.0), hi=(0.5, 0.5, 0.8), solid=False
    )
    check(
        "Punkt mitten im Geofence ist frei",
        fence.clearance([0.0, 0.0, 0.4], radius=0.05) > 0,
        str(fence.clearance([0.0, 0.0, 0.4], radius=0.05)),
    )
    check(
        "Punkt ausserhalb des Geofence verletzt die Grenze",
        fence.clearance([0.9, 0.0, 0.4], radius=0.05) < 0,
        str(fence.clearance([0.9, 0.0, 0.4], radius=0.05)),
    )
    check(
        "Punkt knapp innen an der Wand verletzt wegen Huellkugel",
        fence.clearance([0.48, 0.0, 0.4], radius=0.05) < 0,
        str(fence.clearance([0.48, 0.0, 0.4], radius=0.05)),
    )

    model = collision.default_workspace(
        table_height_m=0.0, geofence=((-0.8, -0.8, -0.05), (0.8, 0.8, 0.9))
    )
    check("default_workspace enthaelt Tisch und Arbeitsraum", len(model.boxes) == 2)

    # Kollisionspruefung ohne echten Roboter: FK durch eine Attrappe ersetzen
    class FakeKinematics(object):
        """Gibt eine feste Position zurueck -- nur fuer diesen Test."""

        def __init__(self, position):
            self.position = np.asarray(position, dtype=float)

        def fk_position(self, joints, frame="tool"):
            return self.position

    free = collision.CollisionModel(
        boxes=[box], arm_points=[collision.ArmPoint("tool", 0.05)], margin=0.02
    )
    check(
        "check_joints meldet frei, wenn der Punkt hoch genug liegt",
        free.check_joints(FakeKinematics([0, 0, 0.5]), [0] * 6) == [],
    )
    violations = free.check_joints(FakeKinematics([0, 0, 0.01]), [0] * 6)
    check(
        "check_joints meldet Verletzung dicht ueber dem Tisch",
        len(violations) == 1 and violations[0].box == "Tisch",
        str(violations),
    )

    path = np.zeros((5, 6))
    kin_low = FakeKinematics([0, 0, 0.01])
    found = free.check_path(kin_low, path, stop_at_first=True)
    check("check_path bricht bei der ersten Verletzung ab", len(found) == 1, str(found))


def main():
    print("Selbsttest der hardwarefreien Phase-0-Module")
    test_geometry()
    test_collision()

    section("Ergebnis")
    if FAILURES:
        print("%d Pruefung(en) fehlgeschlagen:" % len(FAILURES))
        for name in FAILURES:
            print("  - %s" % name)
        raise SystemExit(1)
    print("Alle Pruefungen bestanden.")


if __name__ == "__main__":
    main()

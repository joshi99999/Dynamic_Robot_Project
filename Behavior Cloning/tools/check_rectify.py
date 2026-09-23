"""ArUco-Marker am realen Aufbau pruefen (AP 1.4, offener Befund test_rectify).

NUR LESEN -- kein Roboter, nur ein Kamerabild (live oder aus einer Datei).

Beantwortet am Labortermin, was vor der Integration der Rektifizierung in
Aufzeichnung und Inferenz feststehen muss:

1. Welches ArUco-Woerterbuch und welche IDs liegen auf dem Tisch?
   (``--dictionary auto`` probiert die gaengigen.)
2. Reicht die Policy-Aufloesung 240 x 320 zur Erkennung? Befund 11.09.2026:
   synthetisch nur grenzwertig (test_rectify). Hier: Erkennung im vollen
   Bild UND im auf Schemagroesse verkleinerten Bild, Markergroesse in Pixeln.
   Faellt die kleine Aufloesung ab, wird die Homographie im vollen Bild
   gerechnet und erst das entzerrte Bild verkleinert.
3. Mit ``--layout`` (Markerpositionen des CV-Teams): Homographie,
   Reprojektionsfehler der Ecken und das entzerrte Bild zur Sichtpruefung.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_rectify.py --camera scene
    python tools/check_rectify.py --image snapshot_000.png --dictionary DICT_4X4_50
    python tools/check_rectify.py --camera scene --layout marker_layout.json --x-range 0.2,0.6 --y-range -0.2,0.2

``marker_layout.json``: {"dictionary": "DICT_4X4_50", "marker_size_m": 0.05,
"markers": {"0": [0.25, 0.15], "1": [0.55, 0.15], ...}} -- Markerzentren in
Metern, Tischebene, Bezugssystem wie beim CV-Team.
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config

DICTIONARIES = ("DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_100", "DICT_6X6_250",
                "DICT_7X7_1000", "DICT_ARUCO_ORIGINAL", "DICT_APRILTAG_36h11")


def detect(image, dictionary):
    import cv2

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)),
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.flatten())}


def side_px(corners):
    return float(np.mean([np.linalg.norm(corners[k] - corners[(k + 1) % 4]) for k in range(4)]))


def grab(camera_name):
    from bc import capture
    from bc.adapters import open_camera

    cfg = {c.name: c for c in config.CAMERAS}[camera_name]
    cap = capture.ThreadedCapture(open_camera(cfg)).start()
    try:
        return cap.wait_for_frame(timeout=10.0).image
    finally:
        cap.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--camera", choices=("scene", "wrist"))
    src.add_argument("--image", help="Bilddatei (RGB/BGR egal fuer die Erkennung)")
    parser.add_argument("--dictionary", default="auto")
    parser.add_argument("--layout", default=None, help="Markerpositionen (JSON, siehe Docstring)")
    parser.add_argument("--x-range", default=None, help="Workspace x_min,x_max in m (mit --layout)")
    parser.add_argument("--y-range", default=None, help="Workspace y_min,y_max in m (mit --layout)")
    parser.add_argument("--out", default=None, help="Ordner fuer Bilder und Protokoll")
    args = parser.parse_args()

    import cv2

    if args.image:
        image = cv2.cvtColor(cv2.imread(args.image), cv2.COLOR_BGR2RGB)
    else:
        image = grab(args.camera)
    small = cv2.resize(image, (config.IMAGE_WIDTH, config.IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
    out = Path(args.out) if args.out else Path("Berichte") / (
        "%s_ArUco" % datetime.now().strftime("%Y-%m-%d_%H-%M"))
    out.mkdir(parents=True, exist_ok=True)
    print("Bild %dx%d, Schemagroesse %dx%d" % (image.shape[1], image.shape[0],
                                               config.IMAGE_WIDTH, config.IMAGE_HEIGHT))

    layout = json.loads(Path(args.layout).read_text(encoding="utf-8")) if args.layout else None
    dicts = [layout["dictionary"]] if layout else (
        DICTIONARIES if args.dictionary == "auto" else [args.dictionary])
    best = max(dicts, key=lambda d: len(detect(image, d)))
    full = detect(image, best)
    reduced = detect(small, best)
    report = {"dictionary": best, "image_size": list(image.shape[:2]),
              "full": {str(k): side_px(v) for k, v in full.items()},
              "schema": {str(k): side_px(v) for k, v in reduced.items()}}
    print("\nWoerterbuch: %s" % best)
    print("  volles Bild : %d Marker %s, Kantenlaenge %s px"
          % (len(full), sorted(full), ", ".join("%.0f" % side_px(v) for v in full.values())))
    print("  240 x 320   : %d Marker %s, Kantenlaenge %s px"
          % (len(reduced), sorted(reduced), ", ".join("%.0f" % side_px(v) for v in reduced.values())))
    lost = sorted(set(full) - set(reduced))
    if lost:
        print("  !! In Schemagroesse verloren: %s -- Homographie im VOLLEN Bild rechnen, "
              "danach verkleinern (oder groessere Marker)." % lost)
    elif full:
        print("  Alle Marker auch in Schemagroesse erkannt.")

    annotated = cv2.cvtColor(image.copy(), cv2.COLOR_RGB2BGR)
    for marker_id, corners in full.items():
        cv2.polylines(annotated, [corners.astype(np.int32)], True, (0, 255, 0), 2)
        cv2.putText(annotated, str(marker_id), tuple(corners[0].astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.imwrite(str(out / "marker_erkannt.png"), annotated)

    if layout:
        from bc.rectify import MarkerLayout, Rectifier, WorkspaceView

        if not (args.x_range and args.y_range):
            raise SystemExit("--layout braucht --x-range und --y-range")
        marker_layout = MarkerLayout(
            markers={int(k): tuple(v) for k, v in layout["markers"].items()},
            marker_size_m=float(layout["marker_size_m"]), dictionary=best)
        view = WorkspaceView(tuple(float(v) for v in args.x_range.split(",")),
                             tuple(float(v) for v in args.y_range.split(",")))
        rect = Rectifier(marker_layout, view)
        H, n = rect.compute_homography(image)
        if H is None:
            raise SystemExit("Homographie nicht bestimmbar: %d bekannte Marker sichtbar" % n)
        errors = []
        for marker_id, corners in detect(image, best).items():
            if marker_id not in marker_layout.markers:
                continue
            world = marker_layout.corners_world(marker_id)
            expected = np.array([view.world_to_pixel(c) for c in world])
            pts = np.hstack([corners, np.ones((4, 1))]) @ H.T
            errors.extend(np.linalg.norm(pts[:, :2] / pts[:, 2:3] - expected, axis=1).tolist())
        report["homography"] = {"markers": n, "reprojection_px_max": max(errors),
                                "reprojection_px_mean": float(np.mean(errors))}
        print("\nHomographie aus %d Markern: Reprojektion max %.2f px, Mittel %.2f px (Zielbild %dx%d)"
              % (n, max(errors), np.mean(errors), view.width_px, view.height_px))
        cv2.imwrite(str(out / "entzerrt.png"), cv2.cvtColor(rect.rectify(image).image, cv2.COLOR_RGB2BGR))

    (out / "protokoll.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("\nBilder und Protokoll: %s" % out)


if __name__ == "__main__":
    main()

"""Live-Vorschau beider Kameras -- Debug-Fenster fuer Phase 0.4.

Bewusst ein schlankes OpenCV-Fenster statt eines GUI-Frameworks: Es geht
nur darum zu sehen, was die Kameras liefern, und die tatsaechlich erreichte
Bildrate zu pruefen. Das richtige Dashboard (NiceGUI) kommt in Phase 2,
wenn es einen Recorder zu steuern gibt.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_cameras.py
    python tools/check_cameras.py --only scene     # nur eine Kamera
    python tools/check_cameras.py --no-display     # nur Ratenmessung

Tasten im Fenster: q oder ESC beendet, s speichert einen Schnappschuss.
"""

import argparse
import time

import _bootstrap  # noqa: F401

import numpy as np

from bc import cameras, config


def build_configs(only):
    if only is None:
        return list(config.CAMERAS)
    selected = [c for c in config.CAMERAS if c.name == only]
    if not selected:
        raise SystemExit(
            "Unbekannte Kamera '%s'. Bekannt: %s"
            % (only, ", ".join(c.name for c in config.CAMERAS))
        )
    return selected


def side_by_side(frames, height=480):
    """Skaliert alle Bilder auf gleiche Hoehe und legt sie nebeneinander."""
    import cv2

    tiles = []
    for frame in frames:
        img = frame.image
        scale = height / img.shape[0]
        width = max(1, int(img.shape[1] * scale))
        tile = cv2.resize(img, (width, height))
        tiles.append(tile)
    return np.hstack(tiles) if tiles else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--only", default=None, help="nur diese Kamera oeffnen")
    parser.add_argument(
        "--no-display", action="store_true", help="kein Fenster, nur Raten messen"
    )
    parser.add_argument(
        "--seconds", type=float, default=0.0,
        help="nach N Sekunden automatisch beenden (0 = unbegrenzt)",
    )
    args = parser.parse_args()

    configs = build_configs(args.only)
    print("Oeffne Kameras: %s" % ", ".join(c.name for c in configs))
    for cfg in configs:
        print("  %-6s backend=%-7s %dx%d device=%s"
              % (cfg.name, cfg.backend, cfg.width, cfg.height, cfg.device))

    cams = cameras.open_all(configs, threaded=True)
    for cam in cams:
        cam.wait_for_frame(timeout=10.0)
    print("Alle Kameras liefern Bilder.\n")

    if not args.no_display:
        import cv2

    last_report = time.time()
    last_counts = {cam.name: 0 for cam in cams}
    start = time.time()
    snapshot_no = 0

    try:
        while True:
            frames = [cam.latest() for cam in cams]
            frames = [f for f in frames if f is not None]

            now = time.time()
            if now - last_report >= 1.0:
                parts = []
                for cam in cams:
                    count = cam.frames_captured
                    fps = (count - last_counts[cam.name]) / (now - last_report)
                    last_counts[cam.name] = count
                    frame = cam.latest()
                    age_ms = (now - frame.timestamp) * 1000.0 if frame else float("nan")
                    parts.append("%s: %.1f fps (Alter %.0f ms)" % (cam.name, fps, age_ms))
                if len(frames) == 2:
                    skew_ms = abs(frames[0].timestamp - frames[1].timestamp) * 1000.0
                    budget = config.SYNC_MAX_SKEW_S * 1000.0
                    flag = "OK" if skew_ms <= budget else "UEBER BUDGET"
                    parts.append("Versatz %.0f ms (max %.0f, %s)" % (skew_ms, budget, flag))
                print(" | ".join(parts))
                last_report = now

            if not args.no_display and frames:
                canvas = side_by_side(frames)
                bgr = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
                cv2.imshow("Kamera-Vorschau (q beendet, s speichert)", bgr)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    name = "snapshot_%03d.png" % snapshot_no
                    cv2.imwrite(name, bgr)
                    print("Gespeichert: %s" % name)
                    snapshot_no += 1
            else:
                time.sleep(0.01)

            if args.seconds and (time.time() - start) >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer.")
    finally:
        for cam in cams:
            cam.stop()
        if not args.no_display:
            import cv2

            cv2.destroyAllWindows()
        print("Kameras geschlossen.")


if __name__ == "__main__":
    main()

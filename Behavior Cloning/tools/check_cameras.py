"""Live-Vorschau beider Kameras -- Debug-Fenster fuer Phase 0.4.

Bewusst ein schlankes OpenCV-Fenster statt eines GUI-Frameworks: Es geht
nur darum zu sehen, was die Kameras liefern, und die tatsaechlich erreichte
Bildrate zu pruefen. Das richtige Dashboard (NiceGUI) kommt in Phase 2,
wenn es einen Recorder zu steuern gibt.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_cameras.py --list           # welche Geraete sind da?
    python tools/check_cameras.py --backend sim    # Werkzeug ohne Hardware
    python tools/check_cameras.py --backend uvc --device 1  # Webcam als Wrist-Ersatz
    python tools/check_cameras.py                  # beide Kameras
    python tools/check_cameras.py --only scene     # nur eine Kamera
    python tools/check_cameras.py --only scene --device 1   # anderer Index
    python tools/check_cameras.py --no-display     # nur Ratenmessung

Tasten im Fenster: q oder ESC beendet, s speichert einen Schnappschuss.

Reihenfolge bei der Inbetriebnahme (AP 0.6 Punkt 7/8):
    1. --list       -> wird die Kamera ueberhaupt gefunden?
    2. --only <cam> -> liefert sie Bilder, in welcher Aufloesung?
    3. ohne --only  -> beide zusammen: Rate und Zeitversatz gegen das
                       30-ms-Budget (AP 1.3).
"""

import argparse
import time
from dataclasses import replace

import _bootstrap  # noqa: F401

import numpy as np

from bc import capture, config
from bc.adapters import open_camera


def build_configs(only, backend=None, device=None):
    if backend == "sim":
        base = list(config.SIM_CAMERAS)
    elif backend == "uvc":
        # Webcam als Wrist-Ersatz (config.WRIST_CAMERA_UVC_STANDIN) -- dieselbe
        # Konfiguration wie record.py --cameras wrist-uvc
        if only not in (None, "wrist"):
            raise SystemExit("--backend uvc ersetzt nur die Wrist-Kamera (--only wrist)")
        base = [config.WRIST_CAMERA_UVC_STANDIN]
    else:
        base = list(config.CAMERAS)

    if only is not None:
        base = [c for c in base if c.name == only]
        if not base:
            raise SystemExit(
                "Unbekannte Kamera '%s'. Bekannt: %s"
                % (only, ", ".join(c.name for c in config.CAMERAS))
            )

    if device is not None:
        if len(base) != 1:
            raise SystemExit("--device nur zusammen mit --only sinnvoll")
        base = [replace(base[0], device=device)]
    return base


def list_devices():
    """Sucht nach angeschlossenen Kameras -- erster Schritt der Inbetriebnahme.

    UVC: es gibt keine saubere Enumeration in OpenCV, daher werden die
    Indizes 0..5 probeweise geoeffnet. Daheng: ueber das Galaxy SDK.
    """
    print("== UVC / Webcams (OpenCV) ==")
    try:
        import cv2
    except ImportError:
        print("  opencv-python nicht installiert")
        return

    found_uvc = False
    for index in range(6):
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if cap.isOpened():
            ok, frame = cap.read()
            shape = frame.shape if ok and frame is not None else None
            fps = cap.get(cv2.CAP_PROP_FPS)
            print(
                "  Index %d: %s, gemeldete FPS %.1f"
                % (index, shape if shape else "geoeffnet, aber kein Bild", fps)
            )
            found_uvc = True
        cap.release()
    if not found_uvc:
        print("  keine UVC-Kamera gefunden (Indizes 0-5 geprueft)")

    print("\n== Daheng / USB3-Vision (gxipy) ==")
    from bc.adapters.cam_daheng import import_gxipy
    from bc.ports import CameraError

    try:
        gx = import_gxipy()
    except CameraError as exc:
        print("  %s" % exc)
        return
    print("  gxipy: %s" % gx.__file__)

    manager = gx.DeviceManager()
    count, info_list = manager.update_device_list()
    if count == 0:
        print("  keine Daheng-Kamera gefunden (USB3-Kabel/Treiber pruefen,")
        print("  Gegenprobe mit dem Galaxy Viewer)")
        return
    for info in info_list:
        print(
            "  %s  SN=%s  Vendor=%s"
            % (
                info.get("model_name", "?"),
                info.get("sn", "?"),
                info.get("vendor_name", "?"),
            )
        )


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
    parser.add_argument(
        "--list", action="store_true",
        help="angeschlossene Kameras suchen und beenden",
    )
    parser.add_argument(
        "--backend", default=None, choices=("sim", "uvc"),
        help="'sim' testet das Werkzeug ohne Hardware; 'uvc' oeffnet eine USB-Webcam "
             "als Wrist-Ersatz (Index mit --device)",
    )
    parser.add_argument(
        "--device", default=None,
        help="Geraet ueberschreiben (UVC: Index, Daheng: Seriennummer)",
    )
    args = parser.parse_args()

    if args.list:
        list_devices()
        return

    device = args.device
    if device is not None and device.isdigit():
        device = int(device)

    configs = build_configs(args.only, backend=args.backend, device=device)
    print("Oeffne Kameras: %s" % ", ".join(c.name for c in configs))
    for cfg in configs:
        size = (
            "%sx%s" % (cfg.width, cfg.height)
            if cfg.width and cfg.height
            else "voller Sensor"
        )
        binning = " binning=%dx%d" % (cfg.binning, cfg.binning) if cfg.binning > 1 else ""
        print("  %-6s backend=%-7s %-13s device=%s%s"
              % (cfg.name, cfg.backend, size, cfg.device, binning))

    try:
        cams = capture.start_all([open_camera(cfg) for cfg in configs], threaded=True)
    except Exception as exc:
        raise SystemExit(
            "\nKamera liess sich nicht oeffnen:\n  %s\n\n"
            "Naechster Schritt: 'python tools/check_cameras.py --list'" % exc
        )
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

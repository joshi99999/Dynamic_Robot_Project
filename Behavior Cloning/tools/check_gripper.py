"""Greifer abnehmen und Totzeit messen (AP 0.6 Punkt 5, AP 2.1).

NeuraPy meldet den Greiferzustand nicht zurueck -- die Totzeit steckt als
fester Dwell (config.GRIPPER_DWELL_S = 0.5 s, UNVERIFIZIERT) in jeder
aufgezeichneten Bahn und damit im mitgelernten Verhalten. Gemessen wird sie
hier optisch mit der Wrist-Kamera, die die Backen sieht:

    Befehl absetzen (Host-Zeit) -> Bildaenderung beginnt (Backen bewegen
    sich) -> Bild wieder ruhig (Backen stehen).

Zusaetzlich die Dauer des RPC-Aufrufs selbst: blockiert ``grasp()`` bis
die Backen stehen, haelt es in Aufzeichnung und Inferenz den Servostrom an
-- das waere vor der ersten Aufzeichnung zu loesen.

Der ARM bewegt sich nicht, nur der Greifer. Trotzdem: Greiferbereich frei,
Not-Aus in Reichweite.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_gripper.py --real-robot                    # 5 Zyklen
    python tools/check_gripper.py --real-robot --roi 500,700,440,380 --save-frames gripper_frames
    python tools/check_gripper.py --camera sim                    # Werkzeug ohne Hardware pruefen

``--roi x,y,w,h`` (Pixel im Kamerabild) auf die Backen legen -- sonst
zaehlt jede Bewegung im Bild. Mit ``--save-frames`` werden Bilder rund um
jeden Befehl abgelegt, um ROI und Schwelle zu pruefen.
"""

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config
from bc.clock import host_time


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def to_gray_small(image, roi):
    img = image
    if roi:
        x, y, w, h = roi
        img = img[y:y + h, x:x + w]
    gray = img.mean(axis=2) if img.ndim == 3 else img
    step = max(1, int(max(gray.shape) // 160))
    return gray[::step, ::step].astype(np.float32)


def record_window(cap, seconds, roi):
    """Frames fuer ``seconds`` sammeln: Liste (Zeitstempel, kleines Graubild)."""
    frames, last_index = [], None
    end = host_time() + seconds
    while host_time() < end:
        frame = cap.latest()
        if frame is not None and frame.index != last_index:
            frames.append((frame.timestamp, to_gray_small(frame.image, roi), frame.image))
            last_index = frame.index
        time.sleep(0.002)
    return frames


def analyse(frames, t_cmd, noise_sigmas=4.0, min_level=2.0):
    """Beginn und Ende der Bewegung relativ zum Befehl (s) aus Bilddifferenzen."""
    if len(frames) < 10:
        return {"error": "zu wenige Frames (%d)" % len(frames)}
    ts = np.array([f[0] for f in frames])
    diffs = np.array([0.0] + [float(np.abs(frames[i][1] - frames[i - 1][1]).mean())
                              for i in range(1, len(frames))])
    before = diffs[(ts < t_cmd) & (ts > ts[0])]
    floor = float(np.median(before)) if len(before) else 0.0
    spread = float(np.std(before)) if len(before) > 2 else 0.0
    threshold = max(min_level, floor + noise_sigmas * spread)
    moving = np.flatnonzero((diffs > threshold) & (ts >= t_cmd))
    if not len(moving):
        return {"threshold": threshold, "start_s": None, "end_s": None,
                "fps": float(len(ts) / (ts[-1] - ts[0]))}
    return {
        "threshold": threshold,
        "noise_floor": floor,
        "start_s": float(ts[moving[0]] - t_cmd),
        "end_s": float(ts[moving[-1]] - t_cmd),
        "peak_diff": float(diffs.max()),
        "fps": float(len(ts) / (ts[-1] - ts[0])),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--real-robot", action="store_true", help="REALE Anlage (Bestaetigung)")
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--pre", type=float, default=0.5, help="Sekunden Ruhebild vor dem Befehl")
    parser.add_argument("--post", type=float, default=2.0, help="Sekunden nach dem Befehl")
    parser.add_argument("--roi", default=None, help="x,y,w,h im Kamerabild (Backen)")
    parser.add_argument("--camera", choices=("wrist", "sim"), default="wrist")
    parser.add_argument("--save-frames", default=None, help="Ordner fuer Bilder um die Befehle")
    parser.add_argument("--out", default=None, help="JSON-Protokoll (Default: Berichte/Daten)")
    args = parser.parse_args()
    roi = [int(v) for v in args.roi.split(",")] if args.roi else None

    from bc import capture
    from bc.adapters import open_camera, open_robot
    from bc.adapters.cam_sim import SimCamera

    if args.camera == "sim":
        cam = SimCamera(config.SIM_CAMERAS[0], pattern="static")
        robot = open_robot("sim").connect()
        print("SIM: Werkzeugtest ohne Hardware (keine Bildaenderung zu erwarten).")
    else:
        robot = open_robot("neura", allow_real=args.real_robot)
        robot.connect()
        print("Neura: is_robot_in_simulation()=%r, Tool=%r, Greifer=%s"
              % (robot.in_simulation, robot.tool_name, robot.gripper_mode))
        if robot.gripper_mode != "hardware":
            robot.close()
            raise SystemExit("Kein Greifer konfiguriert (Modus %s) -- nichts zu messen. "
                             "Tool/Greifer am Pendant waehlen." % robot.gripper_mode)
        if args.real_robot:
            print("\n!! Greifer schaltet %d x zu/auf. Greiferbereich frei? Not-Aus in Reichweite?"
                  % args.cycles)
            if input("   Zum Fortfahren 'ANLAGE' eintippen: ").strip() != "ANLAGE":
                robot.close()
                raise SystemExit("Abgebrochen.")
        cam = open_camera(config.WRIST_CAMERA)
    cap = capture.ThreadedCapture(cam).start()
    try:
        cap.wait_for_frame(timeout=10.0)
        results = []
        save_dir = Path(args.save_frames) if args.save_frames else None
        for cycle in range(args.cycles):
            for close in (True, False):
                label = "zu" if close else "auf"
                section("Zyklus %d: Greifer %s" % (cycle + 1, label))
                pre = record_window(cap, args.pre, roi)
                t_cmd = host_time()
                robot.gripper_command(close)
                t_ret = host_time()
                post = record_window(cap, args.post, roi)
                result = analyse(pre + post, t_cmd)
                result.update(cycle=cycle, command=label, rpc_s=t_ret - t_cmd)
                results.append(result)
                print("  RPC-Dauer %.0f ms | Bewegung ab %s bis %s | %.0f fps, Schwelle %.2f"
                      % (1e3 * result["rpc_s"],
                         "-" if result.get("start_s") is None else "%.0f ms" % (1e3 * result["start_s"]),
                         "-" if result.get("end_s") is None else "%.0f ms" % (1e3 * result["end_s"]),
                         result.get("fps", 0.0), result.get("threshold", 0.0)))
                if save_dir is not None:
                    import cv2

                    folder = save_dir / ("c%02d_%s" % (cycle, label))
                    folder.mkdir(parents=True, exist_ok=True)
                    for ts, _, image in (pre + post)[:: max(1, len(pre + post) // 40)]:
                        cv2.imwrite(str(folder / ("%+06.0fms.png" % (1e3 * (ts - t_cmd)))),
                                    cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    finally:
        cap.stop()
        robot.close()

    section("Zusammenfassung (AP 0.6 Punkt 5)")
    summary = {}
    for label in ("zu", "auf"):
        rows = [r for r in results if r["command"] == label]
        ends = [r["end_s"] for r in rows if r.get("end_s") is not None]
        rpc = [r["rpc_s"] for r in rows]
        summary[label] = {
            "rpc_ms_median": 1e3 * float(np.median(rpc)),
            "rpc_ms_max": 1e3 * float(np.max(rpc)),
            "end_ms_median": 1e3 * float(np.median(ends)) if ends else None,
            "end_ms_max": 1e3 * float(np.max(ends)) if ends else None,
            "detected": "%d/%d" % (len(ends), len(rows)),
        }
        s = summary[label]
        print("  %-3s RPC %.0f ms (max %.0f) | Backen stehen nach %s (max %s), erkannt %s"
              % (label, s["rpc_ms_median"], s["rpc_ms_max"],
                 "-" if s["end_ms_median"] is None else "%.0f ms" % s["end_ms_median"],
                 "-" if s["end_ms_max"] is None else "%.0f ms" % s["end_ms_max"], s["detected"]))
    dwell_ms = 1e3 * config.GRIPPER_DWELL_S
    worst = max([s["end_ms_max"] or 0.0 for s in summary.values()] or [0.0])
    blocking = max(s["rpc_ms_max"] for s in summary.values())
    print("\n  Arbeitswert GRIPPER_DWELL_S = %.0f ms" % dwell_ms)
    if worst:
        print("  -> %s" % ("reicht (groesste gemessene Schliesszeit %.0f ms)" % worst
                           if worst <= dwell_ms else
                           "ZU KURZ: groesste Schliesszeit %.0f ms -- Dwell vor der Aufzeichnung erhoehen"
                           % worst))
    if blocking > 1e3 / config.SERVO_RATE_HZ:
        print("  !! grasp()/release() blockiert bis %.0f ms -- laenger als ein servo_j-Intervall "
              "(%.1f ms). Greiferbefehl muss vor der Aufzeichnung aus dem Takt-Thread heraus."
              % (blocking, 1e3 / config.SERVO_RATE_HZ))

    out = Path(args.out) if args.out else Path("Berichte") / (
        "%s_Greifer-Totzeit.json" % datetime.now().strftime("%Y-%m-%d_%H-%M"))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"roi": roi, "summary": summary, "cycles": results,
                               "dwell_ms_config": dwell_ms,
                               "in_simulation": getattr(robot, "in_simulation", None)},
                              indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nProtokoll: %s" % out)


if __name__ == "__main__":
    main()

"""Aufgezeichnete Episoden bewerten: Takt, Schleppfehler, Ruckeln (AP 2.4 / 5.2).

NUR LESEN -- kein Roboter, keine Kamera. Liest ``meta.json`` und
``arrays.npz`` der Episoden, die ``apps/record.py`` abgelegt hat.

Beantwortet nach einer Probefahrt die Fragen aus der Auswertung vom
2026-10-03 (Override 1.0 ruckelte, 0.2 sah schoener aus):

* Hat der Rechner den Takt gehalten? (verspaetete servo_j, Laufzeit)
* Folgt der Arm der Bahn -- oder laeuft er hinterher (kleiner Override)?
* Faehrt er glatt oder im Stop-and-go? (Ist- gegen Soll-Beschleunigung)

Die Bewertung selbst steht in ``bc/metrics.episode_quality`` (getestet),
hier wird nur gesucht und ausgegeben. Urteile im gemeinsamen Format
``[OK]/[WARNUNG]/[FEHLER]/[INFO] Text`` fuer die Systemcheck-Oberflaeche.
Fuer die Bilder dazu: ``tools/plot_episode.py``.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_episodes.py                       # die 3 neuesten Episoden
    python tools/check_episodes.py --last 5
    python tools/check_episodes.py "data_vm/2026-10-03"  # bestimmter Ordner
"""

import argparse
import json
from pathlib import Path

import _bootstrap  # noqa: F401

from bc import metrics
from bc.dataset import load_episode

#: Wo ohne Angabe gesucht wird (Ablage laut CLAUDE.md / Anlagentag-Checkliste).
DEFAULT_ROOTS = ("data_vm", "data_anlage", "data_sim")


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def find_datasets(patterns):
    """Alle Ordner mit index.json unter den Mustern (rekursiv)."""
    found = []
    for pattern in patterns:
        # PowerShell expandiert keine Platzhalter -- hier selbst aufloesen
        roots = sorted(Path().glob(pattern)) if any(c in pattern for c in "*?[") \
            else [Path(pattern)]
        for root in roots:
            # rglob findet auch root/index.json selbst -- und die Datums-
            # Unterordner, die unter einem alten Sammel-Index liegen koennen
            if root.is_dir():
                found.extend(sorted(p.parent for p in root.rglob("index.json")))
    return list(dict.fromkeys(found))


def list_episodes(datasets):
    """(Zeitpunkt, Ordner, Index, Eintrag) fuer jede Episode, aelteste zuerst."""
    rows = []
    for root in datasets:
        index = json.loads((root / "index.json").read_text(encoding="utf-8"))
        for i, entry in enumerate(index["episodes"]):
            meta_file = root / entry["dir"] / "meta.json"
            try:
                meta = json.loads(meta_file.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                continue
            stamp = (meta.get("metadata") or {}).get("recorded_at") or ""
            rows.append((stamp, root, i, meta))
    rows.sort(key=lambda r: r[0])
    return rows


def fmt(value, pattern, missing="-"):
    return missing if value is None else pattern % value


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("datasets", nargs="*",
                        help="Datensatz-Ordner oder Muster (Default: %s)" % ", ".join(DEFAULT_ROOTS))
    parser.add_argument("--last", type=int, default=3, help="die N neuesten Episoden (Default 3)")
    args = parser.parse_args()

    patterns = args.datasets or [r for r in DEFAULT_ROOTS if Path(r).is_dir()]
    rows = list_episodes(find_datasets(patterns))
    if not rows:
        print("  [WARNUNG] Keine Episoden gefunden in: %s" % ", ".join(patterns or DEFAULT_ROOTS))
        return
    rows = rows[-max(1, args.last):]

    section("%d Episode(n), aelteste zuerst" % len(rows))
    print("  %-34s %8s %6s %7s %8s %9s %8s %7s"
          % ("Episode", "Override", "servo", "Dauer", "verspaet", "Schlepp", "Schlepp", "Beschl."))
    print("  %-34s %8s %6s %7s %8s %9s %8s %7s"
          % ("", "", "Hz", "Ist/Soll", "servo_j", "Median", "max", "Ist/Soll"))
    results = []
    for stamp, root, i, meta in rows:
        ep = load_episode(root, i)
        figures, verdicts = metrics.episode_quality(
            ep.arrays, ep.metadata, ep.discarded, ep.discard_reason)
        label = "%s/%s" % (root.as_posix(), json.loads(
            (root / "index.json").read_text(encoding="utf-8"))["episodes"][i]["dir"])
        results.append((label, stamp, ep.metadata, verdicts))
        print("  %-34s %8s %6s %7s %8s %9s %8s %7s" % (
            label[-34:],
            fmt(figures["override"], "%.2f"),
            fmt(figures["servo_rate_hz"], "%g"),
            fmt(figures["duration_factor"], "%.2fx"),
            "-" if figures["overrun_ratio"] is None
            else "%.0f %%" % (100 * figures["overrun_ratio"]),
            "%.3f" % figures["follow_median_rad"],
            "%.3f" % figures["follow_max_rad"],
            fmt(figures["accel_ratio_p95"], "%.1fx")))

    for label, stamp, meta, verdicts in results:
        planner = meta.get("planner") or {}
        section("%s  (%s)" % (label, stamp.replace("T", " ")))
        print("  Tempo: Transit %s m/s, Endanflug %s m/s, PTP %s rad/s, Rauschen x%s"
              % (planner.get("transit_speed_ms", "-"), planner.get("approach_speed_ms", "-"),
                 planner.get("ptp_joint_speed_rads", "-"),
                 fmt(meta.get("noise_scale"), "%.2f")))
        print("  in_simulation: %s" % meta.get("in_simulation"))
        # Episode im Urteil nennen: die Oberflaeche sortiert die Urteile
        # aller Episoden nach Schwere, ohne Kopfzeile waeren sie nicht zuzuordnen.
        short = "/".join(label.split("/")[-2:])
        for level, text in verdicts:
            print("  [%s] %s: %s" % (level.upper(), short, text))

    print("\nBilder dazu: python tools/plot_episode.py <Ordner>")


if __name__ == "__main__":
    main()

"""Aufzeichnungen -> LeRobotDataset (AP 2.3 / 0.9 Punkt 6).

Fasst eine oder mehrere Aufzeichnungen (typisch: ein Ordner je Block bzw.
Objektlage) zu einem Datensatz zusammen, der auf JEDEM Trainingsrechner mit
lerobot 0.6.1 laeuft -- den Ordner einfach kopieren. Pruefungen (Schema,
einheitliches Tempo, Episodenauswahl) siehe bc/lerobot_io.py.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python apps/export.py --data "data_vm/2026-09-17/*" --out datasets/vm_run1
    python apps/export.py --data data_sim --out datasets/sim --check   # nur pruefen

Unter Windows kodiert lerobot die Videos in Unterprozessen -- deshalb
laeuft alles hinter ``if __name__ == "__main__"``.
"""

import argparse
import glob
from pathlib import Path

import _bootstrap  # noqa: F401

from bc import lerobot_io


def expand_sources(patterns):
    """Glob-Muster -> Aufzeichnungsordner (mit index.json), sortiert."""
    roots = []
    for pattern in patterns:
        matches = sorted(glob.glob(pattern)) or [pattern]
        for match in matches:
            path = Path(match)
            if (path / "index.json").is_file():
                roots.append(path)
            elif path.is_dir():
                roots.extend(sorted(p.parent for p in path.glob("*/index.json")))
    unique = list(dict.fromkeys(roots))
    if not unique:
        raise SystemExit("Keine Aufzeichnung gefunden unter: %s" % ", ".join(patterns))
    return unique


def add_export_arguments(parser):
    parser.add_argument(
        "--require-success", action="store_true",
        help="nur Episoden mit Erfolgs-Label True (sonst auch unbewertete)",
    )
    parser.add_argument(
        "--allow-mixed", action="store_true",
        help="uneinheitliches Tempo/Override/Roboter zulassen (nur fuer Vergleiche!)",
    )
    parser.add_argument(
        "--images", action="store_true",
        help="Bilder als PNG statt Video ablegen (gross, verlustfrei)",
    )
    parser.add_argument(
        "--crf", type=int, default=None,
        help="Videoqualitaet (AV1-CRF, kleiner = besser; lerobot-Default 30)",
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--data", nargs="+", required=True,
                        help="Aufzeichnungsordner oder Glob-Muster")
    parser.add_argument("--out", required=False, help="Zielordner des LeRobotDataset")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--check", action="store_true",
                        help="nur Quellen pruefen und Auswahl anzeigen, nichts schreiben")
    add_export_arguments(parser)
    args = parser.parse_args()

    sources = expand_sources(args.data)
    print("Quellen (%d):" % len(sources))
    for src in sources:
        print("  %s" % src)
    if args.check:
        scan = lerobot_io.scan_sources(sources, require_success=args.require_success)
        print(lerobot_io.format_scan(scan))
        if scan["conflicts"]:
            raise SystemExit("Uneinheitlich: %s" % sorted(scan["conflicts"]))
        return
    if not args.out:
        parser.error("--out fehlt (oder --check fuer eine reine Pruefung)")

    try:
        info = lerobot_io.export(
            sources,
            args.out,
            require_success=args.require_success,
            allow_mixed=args.allow_mixed,
            use_video=not args.images,
            crf=args.crf,
            overwrite=args.overwrite,
        )
    except lerobot_io.ExportError as exc:
        raise SystemExit("Export abgebrochen: %s" % exc)
    print(
        "\nLeRobotDataset %s: %d Episoden, lerobot %s, Format %s, Schema v%d"
        % (args.out, len(info["episodes"]), info["lerobot_version"],
           info["codebase_version"], info["schema_version"])
    )


if __name__ == "__main__":
    main()

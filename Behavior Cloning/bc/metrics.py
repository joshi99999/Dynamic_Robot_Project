"""Auswertung von Datensaetzen und Evaluationslaeufen (AP 5.1/5.2).

Erste Ausbaustufe (AP 0.10): Bestandsauswertung eines aufgezeichneten
Datensatzes -- Episodenzahlen, Verwerf-Gruende, Sync-Qualitaet. Die
Benchmark-Metriken (Erfolgsrate je Stoergroesse, Ablationen) kommen mit
AP 5.
"""

import json
from pathlib import Path


def summarize(root):
    """Fasst einen Datensatz zusammen (liest nur index/meta, keine Arrays).

    Rueckgabe: dict mit Zaehlern -- geeignet fuer Dashboard und CLI.
    """
    root = Path(root)
    index = json.loads((root / "index.json").read_text(encoding="utf-8"))

    total = len(index["episodes"])
    discarded = 0
    reasons = {}
    success = {"true": 0, "false": 0, "unlabeled": 0}
    steps_total = 0
    bad_frame_ratios = []

    for entry in index["episodes"]:
        meta = json.loads(
            (root / entry["dir"] / "meta.json").read_text(encoding="utf-8")
        )
        steps_total += meta["length"]
        if meta["discarded"]:
            discarded += 1
            reason = meta["discard_reason"] or "unbekannt"
            reasons[reason] = reasons.get(reason, 0) + 1
        if meta["success"] is True:
            success["true"] += 1
        elif meta["success"] is False:
            success["false"] += 1
        else:
            success["unlabeled"] += 1
        ratio = meta["metadata"].get("bad_frame_ratio")
        if ratio is not None:
            bad_frame_ratios.append(ratio)

    return {
        "schema_version": index["schema_version"],
        "rate_hz": index["rate_hz"],
        "episodes": total,
        "episodes_usable": total - discarded,
        "episodes_discarded": discarded,
        "discard_reasons": reasons,
        "success_labels": success,
        "steps_total": steps_total,
        "mean_bad_frame_ratio": (
            sum(bad_frame_ratios) / len(bad_frame_ratios) if bad_frame_ratios else None
        ),
    }


def format_summary(summary):
    """Menschenlesbare Ausgabe von :func:`summarize`."""
    lines = [
        "Datensatz (Schema v%s, %.0f Hz)" % (summary["schema_version"], summary["rate_hz"]),
        "  Episoden gesamt   : %d" % summary["episodes"],
        "  davon verwendbar  : %d" % summary["episodes_usable"],
        "  davon verworfen   : %d" % summary["episodes_discarded"],
    ]
    for reason, count in sorted(summary["discard_reasons"].items()):
        lines.append("    - %s: %d" % (reason, count))
    lines.append("  Schritte gesamt   : %d" % summary["steps_total"])
    if summary["mean_bad_frame_ratio"] is not None:
        lines.append(
            "  Sync-Verletzungen : %.1f%% der Frames (Mittel)"
            % (100.0 * summary["mean_bad_frame_ratio"])
        )
    s = summary["success_labels"]
    lines.append(
        "  Erfolgs-Labels    : %d ok, %d fehlgeschlagen, %d unbewertet"
        % (s["true"], s["false"], s["unlabeled"])
    )
    return "\n".join(lines)

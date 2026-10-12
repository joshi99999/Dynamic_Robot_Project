"""Auswertung von Datensaetzen und Evaluationslaeufen (AP 5.1/5.2).

Erste Ausbaustufe (AP 0.10): Bestandsauswertung eines aufgezeichneten
Datensatzes -- Episodenzahlen, Verwerf-Gruende, Sync-Qualitaet. Die
Benchmark-Metriken (Erfolgsrate je Stoergroesse, Ablationen) kommen mit
AP 5.
"""

import json
from pathlib import Path

import numpy as np


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


def distance_to_path(points, path):
    """Kleinster Abstand jedes Punkts zur Polylinie ``path`` (beide (N, 3))."""
    points = np.asarray(points, dtype=float)
    path = np.asarray(path, dtype=float)
    if len(path) == 1:
        return np.linalg.norm(points - path[0], axis=1)
    a, b = path[:-1], path[1:]
    ab = b - a
    denom = np.maximum((ab * ab).sum(axis=1), 1e-12)
    ap = points[:, None, :] - a[None, :, :]
    t = np.clip((ap * ab[None]).sum(axis=2) / denom[None], 0.0, 1.0)
    closest = a[None] + t[..., None] * ab[None]
    return np.linalg.norm(points[:, None, :] - closest, axis=2).min(axis=1)


def evaluate_rollout(rollout, reference, tol_m=0.01):
    """Bewertet eine Policy-Fahrt gegen die Idealbahn der Aufzeichnung (AP 5.1).

    ``rollout``: dict mit ``tcp`` (N, 7) gemessen je Takt, ``gripper_cmd``
    (N,) kommandiert je Takt, ``reached_end`` (bool).
    ``reference``: ``reference`` aus bc_policy.json (``path``, ``grasp_tcp``,
    ``end_tcp`` je Trainings-Episode).

    Aussagekraeftig nur bei FESTER Objektlage (Simulation, VM): gegriffen
    wird dort, wo in der Aufzeichnung gegriffen wurde. Bei variierter
    Objektlage bewertet erst der reale Griff (AP 5.1).

    Erfolg = Greifer schliesst innerhalb ``tol_m`` am Referenz-Greifpunkt
    UND die Fahrt endet innerhalb ``tol_m`` am Uebergabepunkt.
    """
    tcp = np.asarray(rollout["tcp"], dtype=float)[:, :3]
    gripper = np.asarray(rollout["gripper_cmd"], dtype=float) > 0.5
    out = {"steps": int(len(tcp)), "reached_end": bool(rollout.get("reached_end", False))}

    grasps = [g for g in reference.get("grasp_tcp") or [] if g is not None]
    closing = np.flatnonzero(gripper[1:] & ~gripper[:-1])
    if not closing.size and len(gripper) and gripper[0]:
        closing = np.array([-1])  # schon im ersten Takt geschlossen
    if closing.size:
        # Befehl im Takt c; gemessen wird im Takt c+1 -- dort hat der Arm
        # das Ziel erreicht, mit dem der Befehl gesendet wurde.
        reached = min(int(closing[0]) + 2, len(tcp) - 1)
        at = tcp[reached]
        out["grasp_step"] = reached
        out["grasp_error_m"] = (
            float(np.min(np.linalg.norm(np.asarray(grasps) - at, axis=1))) if grasps else None
        )
    else:
        out["grasp_step"] = None
        out["grasp_error_m"] = None

    ends = [e for e in reference.get("end_tcp") or [] if e is not None]
    out["end_error_m"] = (
        float(np.min(np.linalg.norm(np.asarray(ends) - tcp[-1], axis=1))) if ends else None
    )
    path = reference.get("path")
    if path:
        dev = distance_to_path(tcp, path)
        out["path_dev_p95_m"] = float(np.percentile(dev, 95))
        out["path_dev_max_m"] = float(dev.max())
    out["success"] = bool(
        out["grasp_error_m"] is not None
        and out["grasp_error_m"] <= tol_m
        and out["end_error_m"] is not None
        and out["end_error_m"] <= tol_m
        and out["reached_end"]
    )
    return out


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


# -- Qualitaet einer einzelnen Aufnahme (Systemcheck, tools/check_episodes.py) --

#: Anteil verspaeteter servo_j-Aufrufe (pacer_overruns / Aufrufe), ab dem der
#: Takt als nicht gehalten gilt. Gesunde VM-Laeufe am PC: 0-3 Ueberlaeufe je
#: Episode (< 1 %); Laptop 2026-10-02/03: 13-45 %.
QUALITY_OVERRUN_WARN = 0.05
#: Median-Schleppfehler (rad, groesstes Gelenk) zwischen gesendetem Sollwert
#: und gemessener Stellung im selben Takt. Override 1.0: 0.002 rad;
#: Override 0.2: 0.078 rad -- der Arm lief der Bahn 4-10 Grad hinterher
#: (Auswertung 2026-10-03).
QUALITY_FOLLOW_MEDIAN_WARN_RAD = 0.03
#: Ist- gegen Soll-Beschleunigung (p95, groesstes Gelenk). Glatte Befehle,
#: aber ein Vielfaches an Ist-Beschleunigung = Stop-and-go ("Ruckeln").
#: VM am PC nach der 60-Hz-Interpolation: ~2.5x; Laptop mit verspaeteten
#: Befehlen 4.8-5.8x. Mit fester Taktbreite gerechnet -- bei unregelmaessigen
#: Takten ueberzeichnet das etwas, die Richtung stimmt.
QUALITY_ACCEL_RATIO_WARN = 3.0


def episode_quality(arrays, metadata, discarded=False, discard_reason=""):
    """Kennzahlen und Urteile zu EINER aufgezeichneten Episode.

    Beantwortet die Fragen aus der Auswertung vom 2026-10-03: Hat der Rechner
    den Takt gehalten? Folgt der Arm der Bahn (oder laeuft er hinterher, z. B.
    wegen eines kleinen Overrides)? Faehrt er glatt oder im Stop-and-go?

    ``arrays`` wie in dataset.load_episode (Schluessel mit Punkt). Rueckgabe:
    ``(kennzahlen, urteile)`` mit urteile = Liste von (Stufe, Text), Stufe
    "ok" | "warnung" | "fehler" | "info".
    """
    from . import config
    from .recorder import timing_findings

    rate = float(metadata.get("rate_hz") or config.CONTROL_RATE_HZ)
    dt = 1.0 / rate
    state = np.asarray(arrays["observation.state"][:, :6], dtype=float)
    command = np.asarray(arrays.get("aux.joints_command", state), dtype=float)

    follow = np.abs(state - command).max(axis=1)
    figures = {
        "steps": int(len(state)),
        "override": metadata.get("override"),
        "servo_rate_hz": metadata.get("servo_rate_hz"),
        "follow_median_rad": float(np.median(follow)),
        "follow_max_rad": float(follow.max()),
        "bad_frame_ratio": metadata.get("bad_frame_ratio"),
        "duration_factor": None,
        "overrun_ratio": None,
        "accel_ratio_p95": None,
    }
    if metadata.get("duration_s") and metadata.get("planned_duration_s"):
        figures["duration_factor"] = metadata["duration_s"] / metadata["planned_duration_s"]
    calls = (metadata.get("servo_j_ms") or {}).get("n")
    if calls and metadata.get("pacer_overruns") is not None:
        figures["overrun_ratio"] = metadata["pacer_overruns"] / float(calls)
    if len(state) > 4:
        acc_ist = np.abs(np.diff(state, n=2, axis=0)).max(axis=1) / dt ** 2
        acc_soll = np.abs(np.diff(command, n=2, axis=0)).max(axis=1) / dt ** 2
        soll = float(np.percentile(acc_soll, 95))
        if soll > 1e-6:
            figures["accel_ratio_p95"] = float(np.percentile(acc_ist, 95)) / soll

    verdicts = []
    if discarded:
        verdicts.append(("fehler", "verworfen: %s" % (discard_reason or "ohne Grund")))

    overrun = figures["overrun_ratio"]
    if overrun is not None:
        if overrun > QUALITY_OVERRUN_WARN:
            verdicts.append((
                "warnung",
                "Takt nicht gehalten: %.0f %% der servo_j-Aufrufe kamen zu spaet "
                "(%d von %d). Die Bahn wird dann im Stop-and-go gefahren."
                % (100 * overrun, metadata["pacer_overruns"], calls)))
        else:
            verdicts.append(("ok", "Takt gehalten (%.1f %% verspaetete servo_j)."
                             % (100 * overrun)))
    # Erste Zeile ist der Befund (zu langsam), die weiteren erklaeren ihn.
    for k, line in enumerate(timing_findings(metadata)):
        verdicts.append(("warnung" if k == 0 else "info", line))

    override = metadata.get("override")
    if figures["follow_median_rad"] > QUALITY_FOLLOW_MEDIAN_WARN_RAD:
        reason = (" Override %.2f begrenzt die Geschwindigkeit -- das Tempo im "
                  "Planer senken, nicht den Override." % override
                  if override is not None and override < 1.0 else "")
        verdicts.append((
            "warnung",
            "Arm laeuft der Bahn hinterher: Schleppfehler Median %.3f rad, max "
            "%.3f rad. Die Daten zeigen dann eine Bahn, die so nicht gefahren "
            "wurde.%s" % (figures["follow_median_rad"], figures["follow_max_rad"], reason)))
    else:
        verdicts.append(("ok", "Arm folgt der Bahn (Schleppfehler Median %.3f rad, max %.3f rad)."
                         % (figures["follow_median_rad"], figures["follow_max_rad"])))
    limit = config.RECORDER_MAX_FOLLOW_ERROR_RAD
    if figures["follow_max_rad"] > 0.5 * limit:
        verdicts.append((
            "warnung",
            "Schleppfehler max %.3f rad liegt bei %.0f %% der Abbruchgrenze (%.2f rad) "
            "-- mit mehr Rauschen wuerden Episoden verworfen."
            % (figures["follow_max_rad"], 100 * figures["follow_max_rad"] / limit, limit)))

    ratio = figures["accel_ratio_p95"]
    if ratio is not None:
        if ratio > QUALITY_ACCEL_RATIO_WARN:
            verdicts.append((
                "warnung",
                "Ruckeln: Ist-Beschleunigung %.1f-fach ueber dem Sollwert (p95). Bei "
                "glatten Befehlen heisst das Stop-and-go -- meist verspaetete "
                "Befehle (Rechner), siehe Takt." % ratio))
        else:
            verdicts.append(("ok", "Bahn glatt (Ist-Beschleunigung %.1f-fach Soll, p95)." % ratio))

    bad = figures["bad_frame_ratio"]
    if bad:
        verdicts.append(("info", "%.1f %% der Takte ausserhalb des Latenzbudgets." % (100 * bad)))
    return figures, verdicts

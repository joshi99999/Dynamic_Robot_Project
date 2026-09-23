"""Bericht ueber Policy-Fahrten als PDF (AP 5.1) -- liest nur Protokolle.

Fasst ein oder mehrere Fahrtprotokolle von apps/infer.py
(``<checkpoint>/rollouts/<Lauf>/summary.json`` + ``rollout_*.npz``) und das
Trainingsprotokoll des Checkpoints zusammen:

* Uebersicht je Lauf: Erfolg, Greif-/Endfehler, Bahnabweichung,
  Vorhersagezeit, Chunk-Verzug, Pacer-Ueberlaeufe, gekappte Takte,
  Filter-Ablehnungen, Abbruchgruende.
* Training je Checkpoint: Validierung (Gelenkfehler, Chunk-Glaette gegen
  das Label).
* Bahnen je Lauf: Draufsicht und Hoehe ueber der Zeit gegen die Referenz.

Bewertet wird gegen die Referenz in bc_policy.json -- aussagekraeftig nur bei
fester Objektlage (Simulation, VM).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/report_rollouts.py --run "Sim, echtzeit=checkpoints/sim/rollouts/2026-09-17_12-00-00" \\
        --title "Durchstich" --notes notes.txt --out Berichte/2026-09-17_Durchstich.pdf
"""

import argparse
import csv
import html
import json
import sys
import tempfile
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from plot_episode import PRINT_TEMPLATE, _legend, _nice_ticks, _num, html_to_pdf  # noqa: E402

REF_COLOR = "#2a78d6"
RUN_COLOR = "#1baf7a"
FAIL_COLOR = "#b8322a"


def load_run(label, folder):
    folder = Path(folder)
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    ckpt = Path(summary["checkpoint"]) if summary.get("checkpoint") else None
    if ckpt is not None and not (ckpt / "bc_policy.json").is_file():
        # Checkpoint-Ordner verschoben/umbenannt: Protokolle liegen unter
        # <checkpoint>/rollouts/<Lauf> bzw. <checkpoint>/policy/rollouts/<Lauf>
        for candidate in (folder.parent.parent / "policy", folder.parent.parent):
            if (candidate / "bc_policy.json").is_file():
                ckpt = candidate
                break
    info = json.loads((ckpt / "bc_policy.json").read_text(encoding="utf-8")) if ckpt else {}
    rollouts = []
    for path in sorted(folder.glob("rollout_*.npz")):
        rollout = dict(np.load(path))
        # Fahrt ohne einen protokollierten Takt: leeres 1-D-Array
        rollout["tcp"] = np.asarray(rollout["tcp"], dtype=float).reshape(-1, 7)
        rollouts.append(rollout)
    return {"label": label, "folder": folder, "summary": summary, "info": info,
            "checkpoint": ckpt, "rollouts": rollouts}


def _median(values):
    values = [v for v in values if v is not None]
    return float(np.median(values)) if values else None


def overview_rows(runs):
    rows = []
    for run in runs:
        eps = run["summary"]["episodes"]
        ok = sum(e["success"] for e in eps)
        reasons = {}
        for e in eps:
            key = (e["stop_reason"] or "-").split(":")[0]
            reasons[key] = reasons.get(key, 0) + 1
        rows.append(
            "<tr><td class='l'>%s</td><td class='l'>%s</td><td>%s</td><td><b>%d/%d</b></td>"
            "<td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%d</td><td>%d</td><td>%d</td>"
            "<td class='l'>%s</td></tr>"
            % (html.escape(run["label"]),
               html.escape("%s%s" % (eps[0].get("robot", "?"),
                                     " (Sim)" if eps[0].get("in_simulation", True) else "")),
               "asynchron" if eps[0].get("asynchronous") else "synchron",
               ok, len(eps),
               _num(_mm(_median([e.get("grasp_error_m") for e in eps])), 1, "mm"),
               _num(_mm(_median([e.get("end_error_m") for e in eps])), 1, "mm"),
               _num(_mm(_median([e.get("path_dev_p95_m") for e in eps])), 1, "mm"),
               _num(_median([e.get("predict_ms_median") for e in eps]), 0, "ms"),
               _num(max([e.get("chunk_delay_steps_max") or 0 for e in eps]), 0),
               sum(e.get("pacer_overruns", 0) for e in eps),
               sum(e.get("limiter_clamped_steps", 0) for e in eps),
               sum(e.get("guard_rejections", 0) for e in eps),
               html.escape(", ".join("%s %d" % kv for kv in sorted(reasons.items())))))
    return "".join(rows)


def _mm(v):
    return None if v is None else 1e3 * v


def svg_steps(series, height=150, width=540, ylabel=""):
    """Linien ueber Trainingsschritten: [(Farbe, [(step, wert)])]."""
    m_l, m_r, m_t, m_b = 52, 10, 8, 24
    pts = [p for _, s in series for p in s if p[1] is not None]
    if not pts:
        return "<p class='note'>keine Validierungswerte</p>"
    x_hi = max(p[0] for p in pts)
    lo, hi = 0.0, max(p[1] for p in pts) * 1.08
    ticks = _nice_ticks(lo, hi, 4)
    hi = max(hi, ticks[-1])
    X = lambda s: m_l + (width - m_l - m_r) * s / max(1, x_hi)  # noqa: E731
    Y = lambda v: m_t + (height - m_t - m_b) * (1 - (v - lo) / (hi - lo))  # noqa: E731
    out = ['<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg">' % (width, height)]
    for v in ticks:
        out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#e1e0d9"/>' % (m_l, width - m_r, Y(v), Y(v)))
        out.append('<text x="%d" y="%.1f" text-anchor="end">%g</text>' % (m_l - 5, Y(v) + 3.5, v))
    for s in _nice_ticks(0, x_hi, 5):
        out.append('<text x="%.1f" y="%d" text-anchor="middle">%d</text>' % (X(s), height - 7, s))
    if ylabel:
        out.append('<text x="%d" y="%d" class="lbl">%s</text>' % (m_l + 4, m_t + 10, html.escape(ylabel)))
    for color, s in series:
        s = [p for p in s if p[1] is not None]
        d = "".join("%s%.1f %.1f" % ("L" if i else "M", X(a), Y(b)) for i, (a, b) in enumerate(s))
        out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.6"/>' % (d, color))
        for a, b in s:
            out.append('<circle cx="%.1f" cy="%.1f" r="2.2" fill="%s"/>' % (X(a), Y(b), color))
    out.append("</svg>")
    return "".join(out)


def training_section(run):
    log = run["checkpoint"].parent / "train_log.csv" if run["checkpoint"] else None
    if log is None or not log.is_file():
        return ""
    rows = [r for r in csv.DictReader(log.open(encoding="utf-8")) if r.get("val_joint_mae_rad")]
    f = lambda r, k: float(r[k]) if r.get(k) not in (None, "") else None  # noqa: E731
    mae = [(int(r["step"]), f(r, "val_joint_mae_rad")) for r in rows]
    jump = [(int(r["step"]), f(r, "val_chunk_jump_p95_rad")) for r in rows]
    label = [(int(r["step"]), f(r, "val_label_jump_p95_rad")) for r in rows]
    tr = run["info"].get("training", {})
    pol = run["info"].get("policy", {})
    ds = run["info"].get("dataset", {})
    chips = [
        ("Datensatz", "%s Episoden, %s Frames" % (ds.get("episodes"), ds.get("frames"))),
        ("Schritte", tr.get("step")), ("Batch", tr.get("effective_batch_size")),
        ("Vorhersage", pol.get("prediction_type", "epsilon")),
        ("DDIM", pol.get("inference_steps")), ("GPU", tr.get("hardware", {}).get("gpu")),
    ]
    return """
<h2>Training: %s</h2>
<div class="chips">%s</div>
<div class="two">
  <div>%s%s</div>
  <div>%s%s</div>
</div>""" % (
        html.escape(str(run["checkpoint"])),
        "".join("<span>%s <b>%s</b></span>" % (k, html.escape(str(v))) for k, v in chips),
        _legend([("Gelenkfehler Validierung (MAE)", RUN_COLOR, False)]),
        svg_steps([(RUN_COLOR, mae)], ylabel="rad"),
        _legend([("Groesster Sprung im Chunk (p95)", FAIL_COLOR, False), ("im Label", REF_COLOR, False)]),
        svg_steps([(FAIL_COLOR, jump), (REF_COLOR, label)], ylabel="rad"),
    )


def svg_topview(reference, rollouts, successes, width=420, height=320):
    m = 30
    ref = np.asarray(reference) if reference else np.zeros((0, 3))
    allp = np.vstack([ref[:, :2]] + [r["tcp"][:, :2] for r in rollouts if len(r["tcp"])]) * 1e3
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    span = max(hi - lo) * 1.1 + 1.0
    c = (hi + lo) / 2
    s = (min(width, height) - 2 * m) / span
    X = lambda v: width / 2 + (v - c[0]) * s  # noqa: E731
    Y = lambda v: height / 2 - (v - c[1]) * s  # noqa: E731
    out = ['<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg">' % (width, height)]
    for v in _nice_ticks(c[0] - span / 2, c[0] + span / 2, 5):
        out.append('<line x1="%.1f" x2="%.1f" y1="%d" y2="%d" stroke="#e1e0d9"/>' % (X(v), X(v), m, height - m))
        out.append('<text x="%.1f" y="%d" text-anchor="middle">%g</text>' % (X(v), height - 12, v))
    for v in _nice_ticks(c[1] - span / 2, c[1] + span / 2, 5):
        out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#e1e0d9"/>' % (m, width - m, Y(v), Y(v)))
        out.append('<text x="%d" y="%.1f" text-anchor="end">%g</text>' % (m - 2, Y(v) + 3, v))
    out.append('<text x="%d" y="%d" text-anchor="end" class="lbl">x [mm]</text>' % (width - m, height - 24))
    out.append('<text x="%d" y="%d" class="lbl">y [mm]</text>' % (m + 4, m + 10))
    for r, ok in zip(rollouts, successes):
        p = r["tcp"][:, :2] * 1e3
        d = "".join("%s%.1f %.1f" % ("L" if i else "M", X(a), Y(b)) for i, (a, b) in enumerate(p))
        out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.1" opacity="0.7"/>'
                   % (d, RUN_COLOR if ok else FAIL_COLOR))
    if len(ref):
        p = ref[:, :2] * 1e3
        d = "".join("%s%.1f %.1f" % ("L" if i else "M", X(a), Y(b)) for i, (a, b) in enumerate(p))
        out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.8" stroke-dasharray="5 3"/>' % (d, REF_COLOR))
    out.append("</svg>")
    return "".join(out)


def svg_height(reference, rollouts, successes, rate=15.0, width=620, height=320):
    m_l, m_r, m_t, m_b = 46, 10, 10, 24
    series = [np.asarray(reference)[:, 2] * 1e3] if reference else []
    series += [r["tcp"][:, 2] * 1e3 for r in rollouts if len(r["tcp"])]
    lo = min(s.min() for s in series)
    hi = max(s.max() for s in series)
    pad = (hi - lo) * 0.08 + 1.0
    lo, hi = lo - pad, hi + pad
    n = max(len(s) for s in series)
    X = lambda i: m_l + (width - m_l - m_r) * i / max(1, n - 1)  # noqa: E731
    Y = lambda v: m_t + (height - m_t - m_b) * (1 - (v - lo) / (hi - lo))  # noqa: E731
    out = ['<svg viewBox="0 0 %d %d" xmlns="http://www.w3.org/2000/svg">' % (width, height)]
    for v in _nice_ticks(lo, hi, 5):
        out.append('<line x1="%d" x2="%d" y1="%.1f" y2="%.1f" stroke="#e1e0d9"/>' % (m_l, width - m_r, Y(v), Y(v)))
        out.append('<text x="%d" y="%.1f" text-anchor="end">%g</text>' % (m_l - 4, Y(v) + 3, v))
    for t in _nice_ticks(0, (n - 1) / rate, 8):
        out.append('<text x="%.1f" y="%d" text-anchor="middle">%g s</text>' % (X(t * rate), height - 7, t))
    out.append('<text x="%d" y="%d" class="lbl">z [mm]</text>' % (m_l + 4, m_t + 10))
    for r, ok in zip(rollouts, successes):
        z = r["tcp"][:, 2] * 1e3
        d = "".join("%s%.1f %.1f" % ("L" if i else "M", X(i), Y(v)) for i, v in enumerate(z))
        out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.1" opacity="0.7"/>'
                   % (d, RUN_COLOR if ok else FAIL_COLOR))
    if reference:
        z = np.asarray(reference)[:, 2] * 1e3
        d = "".join("%s%.1f %.1f" % ("L" if i else "M", X(i), Y(v)) for i, v in enumerate(z))
        out.append('<path d="%s" fill="none" stroke="%s" stroke-width="1.8" stroke-dasharray="5 3"/>' % (d, REF_COLOR))
    out.append("</svg>")
    return "".join(out)


def run_section(run):
    eps = run["summary"]["episodes"]
    ref = (run["info"].get("reference") or {}).get("path")
    oks = [e["success"] for e in eps]
    rows = "".join(
        "<tr><td>%d</td><td>%s</td><td>%d</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td>"
        "<td>%s</td><td>%d</td><td>%d</td><td class='l'>%s</td></tr>"
        % (e["episode"], "ja" if e["success"] else "nein", e["steps"],
           _num(_mm(e.get("grasp_error_m")), 1), _num(_mm(e.get("end_error_m")), 1),
           _num(_mm(e.get("path_dev_p95_m")), 1), _num(e.get("predict_ms_median"), 0),
           _num(e.get("chunk_delay_steps_max"), 0), e.get("pacer_overruns", 0),
           e.get("limiter_clamped_steps", 0), html.escape(str(e["stop_reason"])[:90]))
        for e in eps)
    return """
<section class="page">
  <h1>%s</h1>
  <p class="sub">%s</p>
  %s
  <div class="two">
    <div><h2>Draufsicht TCP</h2>%s</div>
    <div><h2>Hoehe TCP ueber der Zeit</h2>%s</div>
  </div>
  <table class="ov"><tr><th>Fahrt</th><th>Erfolg</th><th>Takte</th><th>Greifpunkt mm</th><th>Ende mm</th>
  <th>Bahn p95 mm</th><th>Vorhersage ms</th><th>Chunk-Verzug</th><th>Overruns</th><th>gekappt</th>
  <th class="l">Abbruch</th></tr>%s</table>
</section>""" % (
        html.escape(run["label"]), html.escape(str(run["folder"])),
        _legend([("Referenz (Idealbahn Aufzeichnung)", REF_COLOR, False), ("Fahrt mit Erfolg", RUN_COLOR, False),
                 ("Fahrt ohne Erfolg", FAIL_COLOR, False)]),
        svg_topview(ref, run["rollouts"], oks), svg_height(ref, run["rollouts"], oks), rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", action="append", required=True, help="LABEL=Ordner (mehrfach)")
    parser.add_argument("--title", default="Policy-Fahrten")
    parser.add_argument("--notes", default=None, help="Textdatei, eine Zeile je Befund")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    runs = [load_run(*spec.split("=", 1)) for spec in args.run]
    notes = []
    if args.notes:
        notes = [l.strip() for l in Path(args.notes).read_text(encoding="utf-8").splitlines() if l.strip()]
    seen, trainings = set(), []
    for run in runs:
        if run["checkpoint"] and str(run["checkpoint"]) not in seen:
            seen.add(str(run["checkpoint"]))
            trainings.append(training_section(run))
    body = """
<section class="page">
  <h1>%s</h1>
  <p class="sub">%s</p>
  <table class="ov"><tr><th class="l">Lauf</th><th class="l">Roboter</th><th>Vorhersage</th><th>Erfolg</th>
  <th>Greifpunkt</th><th>Ende</th><th>Bahn p95</th><th>Vorhersage</th><th>Chunk-Verzug max</th>
  <th>Overruns</th><th>gekappt</th><th>Filter</th><th class="l">Abbruchgruende</th></tr>%s</table>
  <p class="note">Werte je Lauf als Median ueber die Fahrten; Erfolg = Greifer schliesst &le; 10 mm am
  Referenz-Greifpunkt und Fahrt endet &le; 10 mm am Uebergabepunkt (Stellung und Greifer).</p>
  %s
</section>
<section class="page">%s</section>
%s""" % (
        html.escape(args.title),
        html.escape("Erstellt aus %d Laeufen" % len(runs)),
        overview_rows(runs),
        ("<h2>Befunde</h2><ul class='note'>%s</ul>" % "".join("<li>%s</li>" % html.escape(n) for n in notes))
        if notes else "",
        "".join(trainings),
        "".join(run_section(r) for r in runs),
    )
    page = PRINT_TEMPLATE.replace("__TITLE__", html.escape(args.title)).replace("__BODY__", body)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / "bericht.html"
        src.write_text(page, encoding="utf-8")
        html_to_pdf(src, out, Path(tmp))
    print("PDF: %s" % out.resolve())


if __name__ == "__main__":
    main()

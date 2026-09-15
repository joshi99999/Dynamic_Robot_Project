"""
Episoden-Auswertung als HTML-Bericht (AP 2.4 / AP 5.1) -- NUR LESEND.

Zeigt je Episode, ob die Datenaufzeichnung das tut, was sie soll:

* TCP-Bahn in Drauf- und Seitenansicht: ideal, gesendeter (verrauschter)
  Befehl, tatsaechlich gefahrene Bahn.
* Abweichung vom Ideal ueber die Zeit: Rauschen (Befehl - Ideal) und
  Ist - Ideal, mit Greiferphase -- hier sieht man die Trichter-Daempfung an
  Start, Greifpunkt und Uebergabe.
* Folgefehler Befehl(t) -> Ist(t+1) je Takt: wie gut der Controller folgt.
* Alle sechs Gelenke: ideal / Befehl / Ist.

Braucht nur numpy (kein matplotlib): der Bericht ist eine einzelne
HTML-Datei mit eingebettetem SVG und Hover-Anzeige. Mehrere Datensaetze in
einem Bericht erlauben den direkten Vergleich (z. B. mit und ohne Rauschen).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/plot_episode.py data_vm
    python tools/plot_episode.py data_vm data_vm_ref --out vergleich.html
"""

import argparse
import html
import json
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import dataset


def section(title):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)


def _r(values, digits):
    return [None if not np.isfinite(v) else round(float(v), digits) for v in values]


def episode_payload(root, index):
    ep = dataset.load_episode(root, index)
    a = ep.arrays
    n = len(ep)
    rate = float(ep.metadata.get("rate_hz", 15.0))
    state = a["observation.state"].astype(float)
    q_act = state[:, :6]
    tcp_act = state[:, 6:9]
    q_ideal = a["aux.joints_ideal"].astype(float)
    p_ideal = a["aux.pose_ideal"][:, :3].astype(float)
    p_cmd = a["aux.pose_noisy"][:, :3].astype(float)
    q_cmd = a["aux.joints_command"].astype(float) if "aux.joints_command" in a else q_ideal

    noise_mm = np.linalg.norm(p_cmd - p_ideal, axis=1) * 1000.0
    dev_mm = np.linalg.norm(tcp_act - p_ideal, axis=1) * 1000.0
    # Folgefehler: Befehl im Takt t gegen den Zustand, der im Takt t+1
    # gelesen wurde (der Zustand wird direkt nach dem Senden abgegriffen).
    track = np.full(n, np.nan)
    track[:-1] = np.max(np.abs(q_cmd[:-1] - q_act[1:]), axis=1)
    tcp_track_mm = np.full(n, np.nan)
    tcp_track_mm[:-1] = np.linalg.norm(p_cmd[:-1] - tcp_act[1:], axis=1) * 1000.0

    meta = ep.metadata
    valid = np.isfinite(track)
    stats = {
        "steps": n,
        "duration_s": round(n / rate, 1),
        "track_rms": round(float(np.sqrt(np.mean(track[valid] ** 2))), 4) if valid.any() else None,
        "track_max": round(float(np.max(track[valid])), 4) if valid.any() else None,
        "tcp_track_mean_mm": round(float(np.nanmean(tcp_track_mm)), 1),
        "tcp_track_max_mm": round(float(np.nanmax(tcp_track_mm)), 1),
        "noise_max_mm": round(float(noise_mm.max()), 1),
        "sync_ok_pct": round(100.0 * float(a["aux.sync_ok"].mean()), 1),
        "done_last": bool(a["next.done"][-1, 0]) if "next.done" in a else None,
        "discarded": ep.discarded,
        "discard_reason": ep.discard_reason,
    }
    info = {
        "robot": meta.get("robot"),
        "in_simulation": meta.get("in_simulation"),
        "sequence": meta.get("sequence"),
        "noise_scale": meta.get("noise_scale", 1.0),
        "override": meta.get("override"),
        "gripper_mode": meta.get("gripper_mode"),
        "pacer_overruns": meta.get("pacer_overruns"),
        "noise_rejects": meta.get("noise_rejects"),
        "skipped_points": meta.get("skipped_points"),
        "points": list((meta.get("points") or {}).keys()),
    }
    return {
        "label": "%s / ep_%05d" % (Path(root).name, index),
        "rate": rate,
        "stats": stats,
        "info": info,
        "gripper": [int(v > 0.5) for v in state[:, 13]],
        "done": [bool(v) for v in a["next.done"][:, 0]] if "next.done" in a else [],
        "path": {
            key: [_r(arr[:, k] * 1000.0, 1) for k in range(3)]
            for key, arr in (("ideal", p_ideal), ("cmd", p_cmd), ("act", tcp_act))
        },
        "joints": {
            key: [_r(arr[:, k], 4) for k in range(6)]
            for key, arr in (("ideal", q_ideal), ("cmd", q_cmd), ("act", q_act))
        },
        "noise_mm": _r(noise_mm, 1),
        "dev_mm": _r(dev_mm, 1),
        "track": _r(track, 4),
    }


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("datasets", nargs="+", help="Datensatz-Verzeichnisse (apps/record.py --out)")
    parser.add_argument("--out", default=None, help="Ziel-HTML (Default: <erster Datensatz>/bericht.html)")
    args = parser.parse_args()

    section("Episoden laden")
    episodes = []
    for root in args.datasets:
        index = json.loads((Path(root) / "index.json").read_text(encoding="utf-8"))
        for i in range(len(index["episodes"])):
            episodes.append(episode_payload(root, i))
            s = episodes[-1]["stats"]
            print(
                "  %-28s %3d Schritte, Folgefehler max %.3f rad, TCP-Abweichung max %.0f mm"
                % (episodes[-1]["label"], s["steps"], s["track_max"] or 0, s["tcp_track_max_mm"])
            )
    if not episodes:
        raise SystemExit("Keine Episoden gefunden.")

    out = Path(args.out) if args.out else Path(args.datasets[0]) / "bericht.html"
    title = "Episoden-Auswertung: " + ", ".join(Path(d).name for d in args.datasets)
    page = TEMPLATE.replace("__TITLE__", html.escape(title)).replace(
        "__DATA__", json.dumps(episodes, separators=(",", ":")).replace("</", "<\\/")
    )
    out.write_text(page, encoding="utf-8")
    section("Ergebnis")
    print("  Bericht: %s" % out.resolve())


TEMPLATE = r"""<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e;
  --muted: #898781; --grid: #e1e0d9; --axis: #c3c2b7; --ring: rgba(11,11,11,0.10);
  --band: rgba(137,135,129,0.14);
  --s-ideal: #2a78d6; --s-cmd: #eb6834; --s-act: #1baf7a; --s-track: #4a3aa7;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7;
    --muted: #898781; --grid: #2c2c2a; --axis: #383835; --ring: rgba(255,255,255,0.10);
    --band: rgba(137,135,129,0.20);
    --s-ideal: #3987e5; --s-cmd: #d95926; --s-act: #199e70; --s-track: #9085e9;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --ink: #ffffff; --ink-2: #c3c2b7;
  --muted: #898781; --grid: #2c2c2a; --axis: #383835; --ring: rgba(255,255,255,0.10);
  --band: rgba(137,135,129,0.20);
  --s-ideal: #3987e5; --s-cmd: #d95926; --s-act: #199e70; --s-track: #9085e9;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--ink);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
main { max-width: 1180px; margin: 0 auto; padding: 24px 20px 60px; }
h1 { font-size: 20px; margin: 0 0 4px; font-weight: 600; }
h2 { font-size: 15px; margin: 0 0 2px; font-weight: 600; }
.sub { color: var(--ink-2); margin: 0 0 16px; }
.note { color: var(--ink-2); font-size: 13px; margin: 0 0 10px; }
.bar { display: flex; flex-wrap: wrap; gap: 12px; align-items: center; margin: 12px 0 16px; }
select { font: inherit; padding: 6px 8px; border-radius: 6px; border: 1px solid var(--ring);
  background: var(--surface); color: var(--ink); }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(min(150px, 100%), 1fr)); gap: 10px; margin-bottom: 16px; }
.tile { background: var(--surface); border: 1px solid var(--ring); border-radius: 10px; padding: 10px 12px; }
.tile .k { color: var(--ink-2); font-size: 12px; }
.tile .v { font-size: 20px; font-weight: 600; font-variant-numeric: tabular-nums; }
.tile .d { color: var(--muted); font-size: 12px; }
.card { min-width: 0; background: var(--surface); border: 1px solid var(--ring); border-radius: 12px; padding: 14px 14px 8px; margin-bottom: 14px; position: relative; }
.grid2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(min(340px, 100%), 1fr)); gap: 14px; }
.grid3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(min(300px, 100%), 1fr)); gap: 6px 14px; }
.legend { display: flex; flex-wrap: wrap; gap: 14px; color: var(--ink-2); font-size: 12px; margin: 4px 0 6px; }
.legend span { display: inline-flex; align-items: center; gap: 6px; }
.legend i { display: inline-block; width: 16px; height: 2px; border-radius: 1px; }
.legend i.band { height: 10px; background: var(--band); }
svg { display: block; width: 100%; height: auto; overflow: visible; }
svg text { fill: var(--muted); font-size: 11px; font-variant-numeric: tabular-nums; }
svg .lbl { fill: var(--ink-2); font-size: 11px; }
.tip { position: absolute; pointer-events: none; background: var(--surface); color: var(--ink);
  border: 1px solid var(--ring); border-radius: 8px; padding: 6px 8px; font-size: 12px;
  box-shadow: 0 4px 14px rgba(0,0,0,0.12); white-space: nowrap; z-index: 5; }
.tip b { font-weight: 600; }
.tip .row { display: flex; align-items: center; gap: 6px; font-variant-numeric: tabular-nums; }
.tip i { display: inline-block; width: 10px; height: 2px; }
details { margin-top: 8px; }
summary { cursor: pointer; color: var(--ink-2); }
.tablewrap { overflow-x: auto; max-height: 360px; margin-top: 8px; }
table { border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; width: 100%; }
th, td { text-align: right; padding: 3px 8px; border-bottom: 1px solid var(--grid); }
th { position: sticky; top: 0; background: var(--surface); color: var(--ink-2); font-weight: 500; }
.warn { color: var(--ink); }
</style>
</head>
<body>
<main>
  <h1>__TITLE__</h1>
  <p class="sub">Ideal = geplante Bahn &middot; Befehl = verrauschte, per servo_j gesendete Bahn &middot; Ist = gemessene Stellung (FK). VM-Ergebnisse sind keine Anlagenmessung.</p>
  <div class="bar">
    <label for="ep">Episode</label> <select id="ep"></select>
    <span id="info" class="note" style="margin:0"></span>
  </div>
  <div class="tiles" id="tiles"></div>

  <div class="grid2">
    <div class="card"><h2>TCP-Bahn, Draufsicht</h2><p class="note">X/Y in mm, Basis-Koordinatensystem</p><div class="legend" data-legend="path"></div><div id="c-xy"></div></div>
    <div class="card"><h2>TCP-Bahn, Seitenansicht</h2><p class="note">X/Z in mm</p><div class="legend" data-legend="path"></div><div id="c-xz"></div></div>
  </div>

  <div class="card"><h2>Abweichung vom Ideal</h2>
    <p class="note">Befehl &minus; Ideal zeigt das aufgepraegte Rauschen (0 an Start, Greifpunkt und Uebergabe). Ist &minus; Ideal zeigt, was davon am Arm ankommt. Grau: Greifer geschlossen.</p>
    <div class="legend" data-legend="dev"></div><div id="c-dev"></div></div>

  <div class="card"><h2>Folgefehler Befehl &rarr; Ist</h2>
    <p class="note">Groesste Gelenkabweichung zwischen dem Befehl im Takt t und der Stellung im Takt t+1, in rad.</p>
    <div id="c-track"></div></div>

  <div class="card"><h2>Gelenke</h2><p class="note">rad</p><div class="legend" data-legend="path"></div><div class="grid3" id="c-joints"></div>
    <details><summary>Tabelle aller Schritte</summary><div class="tablewrap"><table id="tbl"></table></div></details>
  </div>
</main>
<script>
const EPISODES = __DATA__;
const NS = "http://www.w3.org/2000/svg";
const SERIES = [
  { key: "ideal", name: "Ideal", color: "var(--s-ideal)" },
  { key: "cmd", name: "Befehl (verrauscht)", color: "var(--s-cmd)" },
  { key: "act", name: "Ist", color: "var(--s-act)" },
];

function el(tag, attrs, parent) {
  const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}
function niceTicks(lo, hi, count) {
  if (!(hi > lo)) { hi = lo + 1; }
  const raw = (hi - lo) / count, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 2.5, 5, 10].map(m => m * mag).find(s => s >= raw);
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(6));
  return out;
}
function fmt(v, d) { return v === null || v === undefined ? "\u2013" : Number(v).toFixed(d); }
function legend(kind, items) {
  document.querySelectorAll('[data-legend="' + kind + '"]').forEach(box => {
    box.textContent = "";
    items.forEach(it => {
      const s = document.createElement("span"), i = document.createElement("i");
      if (it.band) i.className = "band"; else i.style.background = it.color;
      s.appendChild(i); s.appendChild(document.createTextNode(it.name)); box.appendChild(s);
    });
  });
}
function tooltip(card) {
  let t = card.querySelector(".tip");
  if (!t) { t = document.createElement("div"); t.className = "tip"; t.hidden = true; card.appendChild(t); }
  return t;
}
function fillTip(tip, title, rows) {
  tip.textContent = "";
  const h = document.createElement("b"); h.textContent = title; tip.appendChild(h);
  rows.forEach(r => {
    const row = document.createElement("div"); row.className = "row";
    const i = document.createElement("i"); i.style.background = r.color || "transparent";
    row.appendChild(i); row.appendChild(document.createTextNode(r.text)); tip.appendChild(row);
  });
}
function placeTip(tip, card, evt) {
  const cr = card.getBoundingClientRect();
  tip.hidden = false;
  let x = evt.clientX - cr.left + 14, y = evt.clientY - cr.top + 14;
  if (x + tip.offsetWidth > cr.width) x = evt.clientX - cr.left - tip.offsetWidth - 14;
  tip.style.left = x + "px"; tip.style.top = y + "px";
}

// Zeitreihe mit Fadenkreuz: series = [{name,color,values}], x in Sekunden
function lineChart(host, opts) {
  host.textContent = "";
  const W = 640, H = opts.height || 220, m = { l: 48, r: opts.directLabels ? 118 : 12, t: 10, b: 26 };
  const n = opts.series[0].values.length, rate = opts.rate;
  let lo = Infinity, hi = -Infinity;
  opts.series.forEach(s => s.values.forEach(v => { if (v !== null) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }));
  if (opts.zero) lo = Math.min(0, lo);
  if (!(hi > lo)) { hi = lo + 1; }
  const pad = (hi - lo) * 0.06; hi += pad; if (!opts.zero) lo -= pad;
  const yt = niceTicks(lo, hi, 4); lo = Math.min(lo, yt[0]); hi = Math.max(hi, yt[yt.length - 1]);
  const X = i => m.l + (W - m.l - m.r) * i / Math.max(1, n - 1);
  const Y = v => m.t + (H - m.t - m.b) * (1 - (v - lo) / (hi - lo));
  const svg = el("svg", { viewBox: "0 0 " + W + " " + H, role: "img", "aria-label": opts.title || "Zeitreihe" }, host);
  if (opts.bands) {
    let start = null;
    for (let i = 0; i <= n; i++) {
      const on = i < n && opts.bands[i];
      if (on && start === null) start = i;
      if (!on && start !== null) { el("rect", { x: X(start), y: m.t, width: Math.max(1, X(i - 1) - X(start)), height: H - m.t - m.b, fill: "var(--band)" }, svg); start = null; }
    }
  }
  yt.forEach(v => {
    el("line", { x1: m.l, x2: W - m.r, y1: Y(v), y2: Y(v), stroke: "var(--grid)", "stroke-width": 1 }, svg);
    el("text", { x: m.l - 6, y: Y(v) + 4, "text-anchor": "end" }, svg).textContent = opts.yFmt ? opts.yFmt(v) : v;
  });
  el("line", { x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, stroke: "var(--axis)", "stroke-width": 1 }, svg);
  niceTicks(0, (n - 1) / rate, 6).forEach(s => {
    const x = X(s * rate); if (x > W - m.r + 1) return;
    el("text", { x: x, y: H - 8, "text-anchor": "middle" }, svg).textContent = s + " s";
  });
  opts.series.forEach(s => {
    let d = "", pen = false;
    s.values.forEach((v, i) => { if (v === null) { pen = false; return; } d += (pen ? "L" : "M") + X(i).toFixed(1) + " " + Y(v).toFixed(1); pen = true; });
    el("path", { d: d, fill: "none", stroke: s.color, "stroke-width": 2, "stroke-linejoin": "round", "stroke-linecap": "round" }, svg);
  });
  if (opts.directLabels) {
    const ends = opts.series.map(s => { let i = s.values.length - 1; while (i > 0 && s.values[i] === null) i--; return { s: s, y: Y(s.values[i]) }; })
      .sort((a, b) => a.y - b.y);
    for (let k = 1; k < ends.length; k++) if (ends[k].y - ends[k - 1].y < 13) ends[k].y = ends[k - 1].y + 13;
    const spill = ends[ends.length - 1].y - (H - m.b - 4);  // nicht unter die x-Achse
    if (spill > 0) ends.forEach(e => { e.y -= spill; });
    ends.forEach(e => { el("text", { x: W - m.r + 6, y: e.y + 4, class: "lbl" }, svg).textContent = e.s.name; });
  }
  const cross = el("line", { y1: m.t, y2: H - m.b, stroke: "var(--axis)", "stroke-width": 1, visibility: "hidden" }, svg);
  const dots = opts.series.map(s => el("circle", { r: 4, fill: s.color, stroke: "var(--surface)", "stroke-width": 2, visibility: "hidden" }, svg));
  const hit = el("rect", { x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b, fill: "transparent" }, svg);
  const card = host.closest(".card"), tip = tooltip(card);
  hit.addEventListener("pointermove", evt => {
    const pt = svg.createSVGPoint(); pt.x = evt.clientX; pt.y = evt.clientY;
    const p = pt.matrixTransform(svg.getScreenCTM().inverse());
    const i = Math.max(0, Math.min(n - 1, Math.round((p.x - m.l) / (W - m.l - m.r) * (n - 1))));
    cross.setAttribute("x1", X(i)); cross.setAttribute("x2", X(i)); cross.setAttribute("visibility", "visible");
    opts.series.forEach((s, k) => {
      const v = s.values[i];
      if (v === null) { dots[k].setAttribute("visibility", "hidden"); return; }
      dots[k].setAttribute("cx", X(i)); dots[k].setAttribute("cy", Y(v)); dots[k].setAttribute("visibility", "visible");
    });
    const rows = opts.series.map(s => ({ color: s.color, text: s.name + ": " + fmt(s.values[i], opts.digits) + (opts.unit ? " " + opts.unit : "") }));
    if (opts.extra) rows.push({ text: opts.extra(i) });
    fillTip(tip, "Schritt " + i + " \u00b7 " + (i / rate).toFixed(2) + " s", rows);
    placeTip(tip, card, evt);
  });
  hit.addEventListener("pointerleave", () => { tip.hidden = true; cross.setAttribute("visibility", "hidden"); dots.forEach(d => d.setAttribute("visibility", "hidden")); });
}

// Bahn in der Ebene, gleicher Massstab auf beiden Achsen
function pathChart(host, ep, ax, ay, xName, yName) {
  host.textContent = "";
  const W = 520, H = 380, m = { l: 52, r: 12, t: 10, b: 30 };
  let xlo = Infinity, xhi = -Infinity, ylo = Infinity, yhi = -Infinity;
  SERIES.forEach(s => ep.path[s.key][ax].forEach((v, i) => {
    const w = ep.path[s.key][ay][i];
    xlo = Math.min(xlo, v); xhi = Math.max(xhi, v); ylo = Math.min(ylo, w); yhi = Math.max(yhi, w);
  }));
  const span = Math.max(xhi - xlo, yhi - ylo, 1) * 1.08, cx = (xlo + xhi) / 2, cy = (ylo + yhi) / 2;
  const pw = W - m.l - m.r, ph = H - m.t - m.b, scale = Math.min(pw, ph) / span;
  const X = v => m.l + pw / 2 + (v - cx) * scale, Y = v => m.t + ph / 2 - (v - cy) * scale;
  const svg = el("svg", { viewBox: "0 0 " + W + " " + H, role: "img", "aria-label": "TCP-Bahn " + xName + "/" + yName }, host);
  niceTicks(cy - ph / 2 / scale, cy + ph / 2 / scale, 5).forEach(v => {
    el("line", { x1: m.l, x2: W - m.r, y1: Y(v), y2: Y(v), stroke: "var(--grid)" }, svg);
    el("text", { x: m.l - 6, y: Y(v) + 4, "text-anchor": "end" }, svg).textContent = v;
  });
  niceTicks(cx - pw / 2 / scale, cx + pw / 2 / scale, 6).forEach(v => {
    el("line", { x1: X(v), x2: X(v), y1: m.t, y2: H - m.b, stroke: "var(--grid)" }, svg);
    el("text", { x: X(v), y: H - 10, "text-anchor": "middle" }, svg).textContent = v;
  });
  el("text", { x: W - m.r, y: H - 10 - 12, "text-anchor": "end", class: "lbl" }, svg).textContent = xName + " [mm]";
  el("text", { x: m.l + 4, y: m.t + 12, class: "lbl" }, svg).textContent = yName + " [mm]";
  SERIES.forEach(s => {
    const xs = ep.path[s.key][ax], ys = ep.path[s.key][ay];
    el("path", { d: xs.map((v, i) => (i ? "L" : "M") + X(v).toFixed(1) + " " + Y(ys[i]).toFixed(1)).join(""), fill: "none", stroke: s.color, "stroke-width": 2, "stroke-linejoin": "round" }, svg);
  });
  const ix = ep.path.ideal[ax], iy = ep.path.ideal[ay], last = ix.length - 1;
  [[0, "Start"], [last, "Uebergabe"]].forEach(([i, name]) => {
    el("circle", { cx: X(ix[i]), cy: Y(iy[i]), r: 5, fill: "var(--surface)", stroke: "var(--ink-2)", "stroke-width": 2 }, svg);
    el("text", { x: X(ix[i]) + 8, y: Y(iy[i]) - 8, class: "lbl" }, svg).textContent = name;
  });
  const dots = SERIES.map(s => el("circle", { r: 4, fill: s.color, stroke: "var(--surface)", "stroke-width": 2, visibility: "hidden" }, svg));
  const hit = el("rect", { x: m.l, y: m.t, width: pw, height: ph, fill: "transparent" }, svg);
  const card = host.closest(".card"), tip = tooltip(card);
  hit.addEventListener("pointermove", evt => {
    const pt = svg.createSVGPoint(); pt.x = evt.clientX; pt.y = evt.clientY;
    const p = pt.matrixTransform(svg.getScreenCTM().inverse());
    let best = 0, bd = Infinity;
    SERIES.forEach(s => ep.path[s.key][ax].forEach((v, i) => {
      const d = Math.hypot(X(v) - p.x, Y(ep.path[s.key][ay][i]) - p.y); if (d < bd) { bd = d; best = i; }
    }));
    if (bd > 30) { tip.hidden = true; dots.forEach(d => d.setAttribute("visibility", "hidden")); return; }
    SERIES.forEach((s, k) => { dots[k].setAttribute("cx", X(ep.path[s.key][ax][best])); dots[k].setAttribute("cy", Y(ep.path[s.key][ay][best])); dots[k].setAttribute("visibility", "visible"); });
    fillTip(tip, "Schritt " + best + " \u00b7 " + (best / ep.rate).toFixed(2) + " s", SERIES.map(s => ({
      color: s.color, text: s.name + ": " + xName + " " + fmt(ep.path[s.key][ax][best], 1) + ", " + yName + " " + fmt(ep.path[s.key][ay][best], 1) + " mm" })));
    placeTip(tip, card, evt);
  });
  hit.addEventListener("pointerleave", () => { tip.hidden = true; dots.forEach(d => d.setAttribute("visibility", "hidden")); });
}

function tile(k, v, d) {
  const t = document.createElement("div"); t.className = "tile";
  [["k", k], ["v", v], ["d", d || ""]].forEach(([c, txt]) => { const e = document.createElement("div"); e.className = c; e.textContent = txt; t.appendChild(e); });
  return t;
}

function render(ep) {
  const s = ep.stats, inf = ep.info;
  const tiles = document.getElementById("tiles"); tiles.textContent = "";
  tiles.appendChild(tile("Schritte", s.steps, s.duration_s + " s"));
  tiles.appendChild(tile("Folgefehler max", fmt(s.track_max, 3) + " rad", "RMS der Takt-Maxima " + fmt(s.track_rms, 3) + " rad"));
  tiles.appendChild(tile("TCP Befehl \u2192 Ist", fmt(s.tcp_track_max_mm, 0) + " mm", "Mittel " + fmt(s.tcp_track_mean_mm, 1) + " mm"));
  tiles.appendChild(tile("Rauschen max", fmt(s.noise_max_mm, 0) + " mm", "Faktor " + inf.noise_scale));
  tiles.appendChild(tile("Sync im Budget", fmt(s.sync_ok_pct, 1) + " %", s.discarded ? "verworfen: " + s.discard_reason : "nicht verworfen"));
  tiles.appendChild(tile("\u00dcbergabe markiert", s.done_last ? "ja" : "nein", "next.done im letzten Schritt"));

  const parts = [];
  if (inf.robot) parts.push("Roboter " + inf.robot + (inf.in_simulation === true ? " (Simulation)" : ""));
  if (inf.sequence) parts.push("Ablauf " + inf.sequence);
  if (inf.override !== null && inf.override !== undefined) parts.push("Override " + inf.override);
  if (inf.skipped_points && inf.skipped_points.length) parts.push("\u00fcbersprungen: " + inf.skipped_points.join(", "));
  document.getElementById("info").textContent = parts.join(" \u00b7 ");

  legend("path", SERIES);
  legend("dev", [{ name: "Befehl \u2212 Ideal (Rauschen)", color: "var(--s-cmd)" }, { name: "Ist \u2212 Ideal", color: "var(--s-act)" }, { name: "Greifer zu", band: true }]);
  pathChart(document.getElementById("c-xy"), ep, 0, 1, "X", "Y");
  pathChart(document.getElementById("c-xz"), ep, 0, 2, "X", "Z");
  lineChart(document.getElementById("c-dev"), {
    rate: ep.rate, zero: true, unit: "mm", digits: 1, height: 230, directLabels: true, bands: ep.gripper.map(g => g === 1),
    series: [{ name: "Befehl \u2212 Ideal", color: "var(--s-cmd)", values: ep.noise_mm }, { name: "Ist \u2212 Ideal", color: "var(--s-act)", values: ep.dev_mm }],
    extra: i => "Greifer " + (ep.gripper[i] ? "zu" : "auf"),
  });
  lineChart(document.getElementById("c-track"), {
    rate: ep.rate, zero: true, unit: "rad", digits: 4, height: 180, bands: ep.gripper.map(g => g === 1),
    series: [{ name: "Folgefehler", color: "var(--s-track)", values: ep.track }],
  });
  const jhost = document.getElementById("c-joints"); jhost.textContent = "";
  for (let j = 0; j < 6; j++) {
    const box = document.createElement("div"), h = document.createElement("div");
    h.className = "note"; h.style.margin = "6px 0 0"; h.textContent = "Gelenk " + (j + 1);
    const c = document.createElement("div"); box.appendChild(h); box.appendChild(c); jhost.appendChild(box);
    lineChart(c, { rate: ep.rate, unit: "rad", digits: 4, height: 150,
      series: SERIES.map(s => ({ name: s.name, color: s.color, values: ep.joints[s.key][j] })) });
  }
  const tbl = document.getElementById("tbl"); tbl.textContent = "";
  const head = tbl.insertRow();
  ["Schritt", "t [s]", "Rauschen [mm]", "Ist \u2212 Ideal [mm]", "Folgefehler [rad]", "Greifer", "done"].forEach(t => { const th = document.createElement("th"); th.textContent = t; head.appendChild(th); });
  ep.noise_mm.forEach((v, i) => {
    const r = tbl.insertRow();
    [i, (i / ep.rate).toFixed(2), fmt(v, 1), fmt(ep.dev_mm[i], 1), fmt(ep.track[i], 4), ep.gripper[i] ? "zu" : "auf", ep.done[i] ? "ja" : ""].forEach(t => { r.insertCell().textContent = t; });
  });
}

const sel = document.getElementById("ep");
EPISODES.forEach((ep, i) => { const o = document.createElement("option"); o.value = i; o.textContent = ep.label; sel.appendChild(o); });
sel.addEventListener("change", () => render(EPISODES[+sel.value]));
render(EPISODES[0]);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()

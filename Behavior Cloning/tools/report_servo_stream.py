"""Grafiken zur Umsetzung des servo_j-Stroms (Zittern-an-der-Anlage.md, Abschnitt 9) -- NUR LESEND.

Liest einen Bericht von ``tools/check_servo_stream.py`` (JSON + Mitschrieb)
und optional die Handy-Messung vom 13.10. und legt die Diagramme 08-12 in
``Dokumentation/Bilder/Zittern/`` ab. Jede Zahl des Abschnitts ist damit aus
den Rohdaten nachrechenbar. Braucht matplotlib (nur dieses Werkzeug).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/report_servo_stream.py
    python tools/report_servo_stream.py --bericht Berichte/2026-10-14_10-59-58_Servo-Strom
"""

import argparse
import json
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

import check_servo_stream as strom

BC = Path(__file__).resolve().parent.parent
BILDER = BC / "Dokumentation" / "Bilder" / "Zittern"
STANDARD = BC / "Berichte" / "2026-10-14_10-59-58_Servo-Strom"
HANDY = BC / "Berichte" / "2026-10-13_17-00_Handy-Beschleunigung_Gelenk5.xlsx"

#: Farben (kategorisch, feste Reihenfolge) und Schrift -- hell, fuer Markdown.
BLAU, ORANGE, TUERKIS, GELB = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
GRAU = "#9a9893"
TEXT, TEXT2 = "#0b0b0b", "#52514e"
#: Rauschen 0 / 0,5 / 1 -- eine Farbe, hell nach dunkel (Groesse)
RAUSCH = ("#9cc3ee", "#4f93e0", "#1d5aa6")


def stil(plt):
    plt.rcParams.update({
        "figure.dpi": 110, "font.size": 9, "axes.edgecolor": GRAU, "axes.labelcolor": TEXT2,
        "xtick.color": TEXT2, "ytick.color": TEXT2, "text.color": TEXT,
        "axes.grid": True, "grid.color": "#e4e2dd", "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "axes.axisbelow": True,
        "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb", "savefig.facecolor": "#fcfcfb",
        "legend.frameon": False,
    })


def laden(stamm):
    bericht = json.loads(Path(str(stamm) + ".json").read_text(encoding="utf-8"))
    roh = np.load(str(stamm) + "_Mitschrieb.npz")
    return bericht, roh


def kenn(bericht, name, *pfad):
    x = bericht["laeufe"].get(name, {}).get("kennzahlen", {})
    for p in pfad:
        x = (x or {}).get(p)
    return float("nan") if x is None else float(x)


def tempo_verlauf(roh, name, fk):
    tn, qn = strom.neue_werte(roh[name + "_t"], roh[name + "_q"])
    tr, _ = strom.raster_zeiten(tn)
    p = fk(qn)
    return 0.5 * (tr[1:] + tr[:-1]), np.linalg.norm(np.diff(p, axis=0), axis=1) / np.diff(tr)


def bild_tempo(plt, roh, fk):
    """08: TCP-Tempo befohlen gegen gefahren, vier Arten."""
    plan = strom.Profil(roh["plan_r0"], "linear")
    ts = np.arange(0.0, plan.dauer, 0.002)
    vp = np.linalg.norm(np.diff(fk(plan.positionen(ts)), axis=0), axis=1) / 0.002
    laeufe = [("linear-60-r0-o1", "heute: servo_j linear 60 Hz, Override 1"),
              ("spline-120-r0-o1", "Spline 120 Hz, Override 1"),
              ("pos-60-r0-o1", "servo_j ohne Geschwindigkeit (v = a = 0) 60 Hz, Override 1"),
              ("linear-60-r0-o0.5", "servo_j linear 60 Hz, Override 0,5")]
    fig, axs = plt.subplots(len(laeufe), 1, figsize=(10, 8.2), sharex=True)
    for ax, (n, titel) in zip(axs, laeufe):
        t, v = tempo_verlauf(roh, n, fk)
        ax.plot(ts[1:], vp, color=GRAU, lw=1.2, label="befohlen")
        ax.plot(t, v, color=BLAU, lw=1.0, label="gefahren (neue Werte, 50 Hz)")
        ax.set_title("%s  --  Tempo-Welligkeit %.0f %%, Verzug %.0f ms"
                     % (titel, kenn_glob[n]["tempo"], kenn_glob[n]["verzug"]),
                     fontsize=9, loc="left")
        ax.set_ylabel("TCP m/s")
        ax.set_ylim(0, 0.21)
    axs[0].legend(loc="upper right", fontsize=8, ncol=2)
    axs[-1].set_xlabel("Zeit seit Bahnbeginn (s)")
    axs[-1].set_xlim(0.4, 5.4)
    fig.suptitle("VM am PC, Rauschen 0 -- TCP-Tempo im Abschnitt CLEAR_FOV -> APPROACH_01 -> PRE_GRASP",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(BILDER / "08_vm_tempo_vergleich.png")
    plt.close(fig)


def bild_rate_profil(plt, bericht):
    """09: Welligkeit quer und Verzug je Art (Rauschen 0, Override 1)."""
    zeilen = [("linear-30-r0-o1", "linear 30 Hz"), ("linear-50-r0-o1", "linear 50 Hz"),
              ("linear-60-r0-o1", "linear 60 Hz (heute)"), ("linear-90-r0-o1", "linear 90 Hz"),
              ("linear-120-r0-o1", "linear 120 Hz"), ("a0-60-r0-o1", "a = 0, 60 Hz"),
              ("spline-30-r0-o1", "Spline 30 Hz"), ("spline-60-r0-o1", "Spline 60 Hz"),
              ("spline-120-r0-o1", "Spline 120 Hz"), ("pos-60-r0-o1", "v = a = 0, 60 Hz"),
              ("linear-60-r0-o0.5", "linear 60 Hz, Override 0,5")]
    namen = [z[1] for z in zeilen][::-1]
    well = [kenn(bericht, z[0], "quer", "welligkeit_rms_mm") for z in zeilen][::-1]
    verz = [kenn(bericht, z[0], "verzug_ms") for z in zeilen][::-1]
    farbe = [ORANGE if "heute" in n else (TUERKIS if ("v = a" in n or "Override" in n) else BLAU)
             for n in namen]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4.6), sharey=True,
                                 gridspec_kw={"width_ratios": [3, 2]})
    y = np.arange(len(namen))
    a1.barh(y, well, color=farbe, height=0.62)
    for yi, w in zip(y, well):
        a1.text(w + 0.015, yi, "%.2f" % w, va="center", fontsize=8, color=TEXT2)
    a1.set_yticks(y)
    a1.set_yticklabels(namen)
    a1.set_xlabel("Welligkeit quer zur befohlenen Bahn, RMS (mm)")
    a2.barh(y, verz, color=farbe, height=0.62)
    for yi, w in zip(y, verz):
        a2.text(w + 1.0, yi, "%.0f" % w, va="center", fontsize=8, color=TEXT2)
    a2.set_xlabel("Verzug (ms)")
    for ax in (a1, a2):
        ax.grid(axis="y", visible=False)
    fig.suptitle("VM am PC, ganze Aufnahmebahn, Rauschen 0, Override 1 (ausser letzte Zeile) -- "
                 "orange = heute, tuerkis = Steuerung glaettet selbst", fontsize=9.5)
    fig.tight_layout()
    fig.savefig(BILDER / "09_vm_art_und_rate.png")
    plt.close(fig)


def bild_override_rauschen(plt, bericht):
    """10: Override x Rauschen (servo_j linear 60 Hz)."""
    overrides = ("1", "0.5", "0.25")
    rauschen = ("0", "0.5", "1")
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.8))
    x = np.arange(len(overrides))
    b = 0.26
    for j, r in enumerate(rauschen):
        w = [kenn(bericht, "linear-60-r%s-o%s" % (r, o), "quer", "welligkeit_rms_mm") for o in overrides]
        q = [kenn(bericht, "linear-60-r%s-o%s" % (r, o), "quer", "rms_mm") for o in overrides]
        a1.bar(x + (j - 1) * b, w, b - 0.02, color=RAUSCH[j], label="Rauschen %s" % r.replace(".", ","))
        a2.bar(x + (j - 1) * b, q, b - 0.02, color=RAUSCH[j], label="Rauschen %s" % r.replace(".", ","))
    for ax, titel in ((a1, "Welligkeit quer, RMS (mm) -- das Zackige"),
                      (a2, "Abstand zur befohlenen Bahn gesamt, RMS (mm) -- inkl. Abkuerzen")):
        ax.set_xticks(x)
        ax.set_xticklabels(["Override %s" % o.replace(".", ",") for o in overrides])
        ax.set_title(titel, fontsize=9, loc="left")
        ax.grid(axis="x", visible=False)
    a1.legend(fontsize=8)
    fig.suptitle("VM am PC, servo_j linear 60 Hz -- Override glaettet, kuerzt aber ab "
                 "(0,25: Verzug ~400 ms)", fontsize=9.5)
    fig.tight_layout()
    fig.savefig(BILDER / "10_vm_override_rauschen.png")
    plt.close(fig)


def spektrum(roh, name, fk):
    tn, qn = strom.neue_werte(roh[name + "_t"], roh[name + "_q"])
    tr, _ = strom.raster_zeiten(tn)
    p = fk(qn)
    ds = np.linalg.norm(np.diff(p, axis=0), axis=1)
    dt = np.diff(tr)
    tm = 0.5 * (tr[1:] + tr[:-1])
    v = ds / dt
    s = np.concatenate([[0.0], np.cumsum(ds)])
    h = 0.5 * strom.TEMPO_FENSTER_S
    lo, hi = np.clip(tm - h, tr[0], tr[-1]), np.clip(tm + h, tr[0], tr[-1])
    vg = (np.interp(hi, tr, s) - np.interp(lo, tr, s)) / np.maximum(hi - lo, 1e-6)
    fahrt = vg > strom.FAHRT_MS
    fs = 1.0 / float(np.median(dt))
    tg = np.arange(tm[0], tm[-1], 1.0 / fs)
    x = np.interp(tg, tm, np.where(fahrt, (v - vg) / np.maximum(vg, strom.FAHRT_MS), 0.0))
    x = x - x.mean()
    spek = np.abs(np.fft.rfft(x * np.hanning(len(x)))) / len(x) * 2
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    k = np.ones(5) / 5
    return f, np.convolve(spek, k, mode="same")


def bild_spektrum(plt, roh, fk):
    """11: Spektrum der Tempo-Abweichung (VM)."""
    fig, ax = plt.subplots(figsize=(10, 3.6))
    for name, label, farbe in (("linear-60-r0-o1", "linear 60 Hz (heute)", ORANGE),
                               ("spline-120-r0-o1", "Spline 120 Hz", BLAU),
                               ("pos-60-r0-o1", "v = a = 0, 60 Hz", TUERKIS)):
        f, s = spektrum(roh, name, fk)
        m = (f >= 1.0) & (f <= 24.5)
        ax.plot(f[m], 100 * s[m], color=farbe, lw=2, label=label)
    for f0, txt in ((10.0, "10 Hz"), (15.0, "15 Hz Zieltakt")):
        ax.axvline(f0, color=GRAU, ls=":", lw=1)
        ax.text(f0 + 0.2, ax.get_ylim()[1] * 0.92, txt, fontsize=8, color=TEXT2)
    ax.set_xlabel("Frequenz (Hz) -- aufloesbar bis 25 Hz (Steuerung liefert 50 Werte/s)")
    ax.set_ylabel("Tempo-Abweichung (% je Linie)")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("VM am PC, Rauschen 0 -- die Schwankung liegt bei 6-12 Hz, unabhaengig von der Senderate",
                 fontsize=9.5, loc="left")
    fig.tight_layout()
    fig.savefig(BILDER / "11_vm_spektrum.png")
    plt.close(fig)


def bild_handy(plt):
    """12: Handy 13.10.: servo_j gegen online Lauf 1, Baender um 8 und 10 Hz."""
    import report_zittern as rz

    h = rz.handy_auswerten(str(HANDY))
    fahrten = h["fahrten"]
    if len(fahrten) < 6:
        print("Handy: weniger als 6 Fahrten gefunden -- Bild 12 uebersprungen")
        return
    fig, ax = plt.subplots(figsize=(10, 3.6))
    for i, label, farbe in ((5, "servo_j 60 Hz (Lauf 6)", ORANGE),
                            (0, "online T1.3 B1.5 (Lauf 1)", BLAU)):
        f, s = fahrten[i]["f"], fahrten[i]["spektrum"]
        m = (f >= 4.0) & (f <= 25.0)
        k = np.ones(3) / 3
        ax.plot(f[m], np.convolve(s, k, mode="same")[m], color=farbe, lw=2, label=label)
    ax.axvspan(6.0, 8.5, color=GELB, alpha=0.15, lw=0)
    ax.text(6.1, ax.get_ylim()[1] * 0.9, "Eigenschwingung\nAufbau 6-8 Hz", fontsize=8, color=TEXT2)
    for f0, txt in ((10.0, "10 Hz = 60 - 50"), (15.0, "15 Hz Zieltakt")):
        ax.axvline(f0, color=GRAU, ls=":", lw=1)
        ax.text(f0 + 0.2, ax.get_ylim()[1] * 0.92, txt, fontsize=8, color=TEXT2)
    ax.set_xlabel("Frequenz (Hz)")
    ax.set_ylabel("Amplitude (m/s^2)")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Anlage 13.10., Handy an Gelenk 5 -- servo_j regt 6-8 Hz an und hat eine eigene "
                 "Spitze bei 10 Hz", fontsize=9.5, loc="left")
    fig.tight_layout()
    fig.savefig(BILDER / "12_handy_10hz.png")
    plt.close(fig)


kenn_glob = {}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bericht", default=str(STANDARD), help="Berichtsstamm ohne Endung")
    ap.add_argument("--ohne-handy", action="store_true", help="Bild 12 weglassen")
    args = ap.parse_args()
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    stil(plt)
    BILDER.mkdir(parents=True, exist_ok=True)
    bericht, roh = laden(args.bericht)
    fk = strom.fk_funktion()
    for name in bericht["laeufe"]:
        kenn_glob[name] = {"tempo": kenn(bericht, name, "tempo", "tempo_welligkeit_prozent"),
                           "verzug": kenn(bericht, name, "verzug_ms")}
    bild_tempo(plt, roh, fk)
    bild_rate_profil(plt, bericht)
    bild_override_rauschen(plt, bericht)
    bild_spektrum(plt, roh, fk)
    if not args.ohne_handy and HANDY.exists():
        bild_handy(plt)
    print("Bilder: %s (08-12)" % BILDER)


if __name__ == "__main__":
    main()

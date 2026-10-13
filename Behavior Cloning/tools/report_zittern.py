"""Grafiken und Anhang fuer Dokumentation/Zittern-an-der-Anlage.md -- NUR LESEND.

Liest die Berichte von ``tools/check_trajectory_buffer.py`` (JSON + Mitschrieb)
und die Handy-Messung (phyphox-Export "Beschleunigung mit g", xlsx) und
erzeugt daraus die Diagramme des Dokuments sowie die Tabelle aller
Anlagenlaeufe (Markdown, auf stdout). So bleibt jede Zahl im Dokument aus
den Rohdaten nachrechenbar.

Braucht matplotlib (nur dieses Werkzeug; die Pipeline nicht). Die xlsx wird
ohne openpyxl gelesen (ZIP + XML), damit es auch auf dem Laptop laeuft.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/report_zittern.py
    python tools/report_zittern.py --datum 2026-10-13 --handy Berichte/2026-10-13_17-00_Handy-Beschleunigung_Gelenk5.xlsx
"""

import argparse
import glob
import json
import re
import zipfile
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

BC = Path(__file__).resolve().parent.parent
BERICHTE = BC / "Berichte"
BILDER = BC / "Dokumentation" / "Bilder" / "Zittern"

#: Serie 17:00 -- dieselbe Reihenfolge wie SERIE_STUFEN im Messwerkzeug.
SERIE_NAMEN = ["1: online T1.3 B1.5", "2: online T2 B1.5", "3: online T2 B2",
               "4: online T2 B3", "5: online T2 B3 V1", "6: servo_j 60 Hz"]
FARBEN = ["#2b6cb0", "#3182ce", "#4fa3d9", "#80c4e8", "#a9d8f0", "#c53030"]

_NS = {"m": "http://schemas.openxmlformats.org/spreadsheetml/2006/main"}


# -- Handy-Messung ------------------------------------------------------------


def lies_xlsx(pfad):
    """phyphox-xlsx -> {Blattname: [Zeilen]} (Inline-Strings, keine Formeln)."""
    z = zipfile.ZipFile(pfad)
    wb = ET.fromstring(z.read("xl/workbook.xml"))
    namen = [s.get("name") for s in wb.find("m:sheets", _NS)]
    blaetter = {}
    for i, name in enumerate(namen, 1):
        root = ET.fromstring(z.read("xl/worksheets/sheet%d.xml" % i))
        zeilen = []
        for row in root.iter("{%s}row" % _NS["m"]):
            werte = []
            for c in row.findall("m:c", _NS):
                if c.get("t") == "inlineStr":
                    werte.append("".join(x.text or "" for x in c.iter("{%s}t" % _NS["m"])))
                else:
                    v = c.find("m:v", _NS)
                    try:
                        werte.append(float(v.text))
                    except (AttributeError, TypeError, ValueError):
                        werte.append(None if v is None else v.text)
            zeilen.append(werte)
        blaetter[name] = zeilen
    return blaetter


def _gleit(x, n):
    n = max(1, int(n))
    k = np.ones(n) / n
    if x.ndim == 1:
        return np.convolve(x, k, mode="same")
    return np.column_stack([np.convolve(x[:, i], k, mode="same") for i in range(x.shape[1])])


def handy_auswerten(pfad):
    """Fahrten finden (langsame Lageaenderung) und Zittern je Fahrt (> 5 Hz)."""
    b = lies_xlsx(pfad)
    roh = np.array([r[:4] for r in b["Raw Data"][1:] if r and r[0] is not None], dtype=float)
    start = float(b["Metadata Time"][1][2])
    t, a = roh[:, 0], roh[:, 1:4]
    fs = round(1.0 / float(np.median(np.diff(t))))
    tg = np.arange(t[0], t[-1], 1.0 / fs)
    ag = np.column_stack([np.interp(tg, t, a[:, i]) for i in range(3)])
    langsam = _gleit(ag, 0.25 * fs)
    d = _gleit(np.linalg.norm(np.gradient(langsam, axis=0), axis=1) * fs, 0.3 * fs)
    an = d > 0.25 * np.percentile(d, 99)
    kanten = np.flatnonzero(np.diff(an.astype(int)))
    fahrten, i = [], 0
    if an[0]:
        kanten = np.concatenate([[-1], kanten])
    while i + 1 < len(kanten):
        s0, s1 = kanten[i] + 1, kanten[i + 1] + 1
        if fahrten and (s0 - fahrten[-1][1]) / fs < 1.0:
            fahrten[-1][1] = s1
        else:
            fahrten.append([s0, s1])
        i += 2
    fahrten = [f for f in fahrten if (f[1] - f[0]) / fs > 1.0]
    hp = ag - _gleit(ag, fs / 5.0)              # Hochpass ~5 Hz
    betrag = np.sqrt(np.sum(hp ** 2, axis=1))
    # Die ersten/letzten 0,5 s sind Randeffekte des Filters, keine Messung.
    rand = fs // 2
    betrag[:rand] = np.nan
    betrag[-rand:] = np.nan
    ruhe = np.isfinite(betrag)
    for s0, s1 in fahrten:
        ruhe[max(0, s0 - fs):s1 + fs] = False
    ruhe &= np.isfinite(betrag)
    out = {"fs": fs, "t": tg, "betrag": betrag, "start": start, "fahrten": [],
           "boden_rms": float(np.sqrt(np.mean(betrag[ruhe] ** 2)))}
    for s0, s1 in fahrten:
        x = hp[s0:s1]
        F = np.fft.rfft(x * np.hanning(len(x))[:, None], axis=0)
        P = np.sum(np.abs(F) ** 2, axis=1)
        f = np.fft.rfftfreq(len(x), 1.0 / fs)

        def band(f0, bw=1.5):
            m = (f > f0 - bw) & (f < f0 + bw)
            return float(np.sqrt(np.sum(P[m]))) / len(x) * 2

        m = betrag[s0:s1]
        out["fahrten"].append({
            "t0": tg[s0], "t1": tg[s1 - 1],
            "beginn": datetime.fromtimestamp(start + tg[s0]).strftime("%H:%M:%S"),
            "rms": float(np.sqrt(np.mean(m ** 2))), "p99": float(np.percentile(m, 99)),
            "b15": band(15), "b30": band(30), "b60": band(60),
            "f": f, "spektrum": np.sqrt(P) / len(x) * 2,
            "spitze_hz": float(f[f > 5][np.argmax(P[f > 5])]),
        })
    return out


# -- Berichte des Messwerkzeugs -------------------------------------------------


def berichte(datum):
    out = []
    for f in sorted(glob.glob(str(BERICHTE / ("%s_*_Trajektorien-Puffer.json" % datum)))):
        b = json.loads(Path(f).read_text(encoding="utf-8"))
        npz = Path(f.replace(".json", "_Mitschrieb.npz"))
        out.append({"datei": Path(f).name, "stamp": re.search(r"_(\d\d-\d\d(?:-\d\d)?)_", f).group(1),
                    "b": b, "npz": np.load(npz) if npz.exists() else None})
    return out


def lauf_zeilen(alle):
    """Eine Zeile je Fahrt an der Anlage -- Grundlage fuer Anhang und Grafiken."""
    zeilen = []
    for r in alle:
        b = r["b"]
        if b.get("in_simulation") is not False:
            continue
        a = b.get("argumente", {})
        for name, e in (b.get("tests") or {}).items():
            k = e.get("kennzahlen", {})
            m = e.get("mehrachs", {})
            st = e.get("stopp", {})
            art = "servo_j %g Hz" % e.get("servo_hz", 60) if name.startswith("referenz") or (
                name.startswith("serie_") and "tempo_faktor" not in e) else "online"
            einst = ""
            if art == "online":
                einst = "T%.1f B%.1f V%d%s" % (
                    e.get("tempo_faktor", a.get("online_tempo")), e.get("beschl_faktor", a.get("online_beschl")),
                    e.get("vorlauf_takte", a.get("online_vorlauf")),
                    " %g Hz" % e["online_hz"] if e.get("online_hz", 15) != 15 else "")
            zeilen.append({
                "zeit": r["stamp"].replace("-", ":"), "datei": r["datei"], "test": name, "art": art,
                "einst": einst, "bahn": "6 Achsen" if (m or name.startswith("serie")) else "J1",
                "amp": a.get("amplitude") or 0.05, "dauer": a.get("dauer"),
                "welligkeit": k.get("welligkeit_neu_prozent"), "einbruch": k.get("einbruch_neu_prozent"),
                "p5": k.get("tempo_p5_anteil"), "verzug": k.get("verzug_ms"),
                "rms": k.get("folgefehler_rms_mrad"), "maxabw": m.get("max_mrad"),
                "gelenk": (m.get("gelenk", -1) + 1) if m else None,
                "ueberlauf": e.get("taktueberlaeufe"), "neue_hz": k.get("neue_werte_hz"),
                "stopp_ms": st.get("stillstand_nach_ms"), "stopp_weg": st.get("weg_nach_stopp_mrad"),
                "e": e, "npz": r["npz"],
            })
    return zeilen


def _f(v, fmt):
    return "–" if v is None else fmt % v


def anhang_tabelle(zeilen):
    print("| Zeit | Test | Bahn | Verfahren | Einstellung | Welligkeit | Einbruch | Tempo p5 | "
          "Verzug | Folgefehler RMS | max. Gelenkabw. | Takt-Überl. | Bericht |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for z in zeilen:
        print("| %s | `%s` | %s, %.2f rad/%gs | %s | %s | %s | %s | %s | %s | %s | %s | %s | `%s` |" % (
            z["zeit"], z["test"], z["bahn"], z["amp"], z["dauer"], z["art"], z["einst"] or "–",
            _f(z["welligkeit"], "%.1f %%"), _f(z["einbruch"], "%.1f %%"), _f(z["p5"], "%.2f"),
            _f(z["verzug"], "%.0f ms"), _f(z["rms"], "%.1f mrad"),
            _f(z["maxabw"], "%.1f mrad") + (" (J%d)" % z["gelenk"] if z["gelenk"] else ""),
            _f(z["ueberlauf"], "%d"), z["datei"].replace("_Trajektorien-Puffer.json", "")))


# -- Grafiken -----------------------------------------------------------------------


def grafiken(handy, zeilen, plt):
    BILDER.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"figure.dpi": 110, "font.size": 9, "axes.grid": True,
                         "grid.alpha": 0.3})
    fahrten = handy["fahrten"]

    # 1. Zeitverlauf der ganzen Serie
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot(handy["t"], handy["betrag"], lw=0.4, color="#444")
    for i, fa in enumerate(fahrten):
        ax.axvspan(fa["t0"], fa["t1"], color=FARBEN[i], alpha=0.25)
        ax.text(0.5 * (fa["t0"] + fa["t1"]), ax.get_ylim()[1] * 0.92, SERIE_NAMEN[i].split(":")[0],
                ha="center", fontsize=10, weight="bold")
    ax.set_xlabel("Zeit seit Messstart [s]")
    ax.set_ylabel("|a| > 5 Hz [m/s²]")
    ax.set_title("Handy am Gelenk 5, Serie 13.10. 17:00 – Schwingung je Fahrt (1–5 online, 6 servo_j)")
    fig.tight_layout()
    fig.savefig(BILDER / "01_handy_zeitverlauf.png")
    plt.close(fig)

    # 2. RMS / p99 je Fahrt
    fig, ax = plt.subplots(figsize=(7, 3.4))
    x = np.arange(len(fahrten))
    ax.bar(x - 0.2, [f["rms"] for f in fahrten], 0.4, color=FARBEN[:len(fahrten)], label="RMS")
    ax.bar(x + 0.2, [f["p99"] for f in fahrten], 0.4, color=FARBEN[:len(fahrten)], alpha=0.45,
           label="p99")
    ax.axhline(handy["boden_rms"], color="#888", ls="--", lw=1, label="Stillstand (Rauschboden)")
    ax.set_xticks(x, [n.replace(": ", "\n") for n in SERIE_NAMEN[:len(fahrten)]], fontsize=8)
    ax.set_ylabel("Schwingung > 5 Hz [m/s²]")
    ax.set_title("Zittern je Einstellung (Handy, Gelenk 5)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(BILDER / "02_handy_rms.png")
    plt.close(fig)

    # 3. Spektren
    fig, ax = plt.subplots(figsize=(8, 3.6))
    for i, fa in enumerate(fahrten):
        m = (fa["f"] > 3) & (fa["f"] < 70)
        glatt = _gleit(fa["spektrum"][m], 3)
        ax.plot(fa["f"][m], glatt, color=FARBEN[i], lw=1.6 if i in (0, 5) else 1.0,
                label=SERIE_NAMEN[i])
    for f0 in (15, 30, 45, 60):
        ax.axvline(f0, color="#999", ls=":", lw=0.8)
    ax.set_xlabel("Frequenz [Hz]  (gepunktet: 15-Hz-Zieltakt und Vielfache)")
    ax.set_ylabel("Amplitude [m/s²]")
    ax.set_title("Spektrum je Fahrt – online pulsiert bei 15 Hz, servo_j regt 7–8 Hz an")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(BILDER / "03_handy_spektrum.png")
    plt.close(fig)

    # 4. Zielkonflikt: Zittern gegen Bahntreue (Serie, Mittel der 3 Serien)
    serie = [z for z in zeilen if z["test"].startswith("serie_")]
    if serie and fahrten:
        fig, ax = plt.subplots(figsize=(6.4, 4.2))
        for i in range(len(fahrten)):
            werte = [z for z in serie if z["test"] == "serie_%d" % (i + 1)]
            mx = np.mean([z["maxabw"] for z in werte])
            rms = np.mean([z["rms"] for z in werte])
            ax.scatter(fahrten[i]["rms"], mx, s=80, color=FARBEN[i], zorder=3)
            ax.annotate(SERIE_NAMEN[i], (fahrten[i]["rms"], mx), textcoords="offset points",
                        xytext=(6, -12) if i == 4 else (6, 4), fontsize=8)
            ax.scatter(fahrten[i]["rms"], rms, s=30, color=FARBEN[i], marker="s", alpha=0.6)
        ax.scatter([], [], color="#666", s=60, label="max. Gelenkabweichung")
        ax.scatter([], [], color="#666", s=25, marker="s", label="Folgefehler RMS (J1)")
        ax.set_xlabel("Zittern RMS am Gelenk 5 [m/s²]  (weniger = ruhiger)")
        ax.set_ylabel("Abweichung von der Gelenkbahn [mrad]")
        ax.set_title("Zielkonflikt Laufruhe gegen Bahntreue (Mehrachsbahn)")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(BILDER / "04_zielkonflikt.png")
        plt.close(fig)

        # 5. Reproduzierbarkeit ueber die drei Serien
        fig, axs = plt.subplots(1, 3, figsize=(11, 3.2))
        zeiten = sorted(set(z["zeit"] for z in serie))
        for ax, (key, titel, einheit) in zip(axs, (
                ("maxabw", "max. Gelenkabweichung", "mrad"),
                ("rms", "Folgefehler RMS (J1)", "mrad"),
                ("einbruch", "Tempo-Einbruch (neue Werte)", "%"))):
            for s, zeit in enumerate(zeiten):
                werte = [next(z[key] for z in serie if z["zeit"] == zeit and z["test"] == "serie_%d" % (i + 1))
                         for i in range(6)]
                ax.bar(np.arange(6) + (s - 1) * 0.27, werte, 0.27, color=FARBEN, alpha=0.5 + 0.25 * s,
                       label="Serie %s" % zeit)
            ax.set_xticks(np.arange(6), [str(i + 1) for i in range(6)])
            ax.set_xlabel("Lauf")
            ax.set_title("%s [%s]" % (titel, einheit))
        axs[0].legend(fontsize=7)
        fig.suptitle("Drei Serien hintereinander – die Werte wiederholen sich (Lauf 6 = servo_j)")
        fig.tight_layout()
        fig.savefig(BILDER / "05_serien_reproduzierbar.png")
        plt.close(fig)

    # 6. J1 allein: servo_j gegen online (Tempo aus neuen Werten)
    paar = [(z, "servo_j 60 Hz") for z in zeilen if z["zeit"] == "16:10" and z["test"] == "referenz"]
    paar += [(z, "online T1.3 B3 V2") for z in zeilen if z["zeit"] == "16:11" and z["test"] == "online"]
    if len(paar) == 2:
        fig, ax = plt.subplots(figsize=(8, 3.4))
        for (z, label), farbe in zip(paar, ("#c53030", "#2b6cb0")):
            npz = z["npz"]
            t, q = npz[z["test"] + "_t"], npz[z["test"] + "_q"][:, 0]
            neu = np.concatenate([[True], np.abs(np.diff(q)) > 1e-7])
            tn, qn = t[neu], q[neu]
            v = np.diff(qn) / np.diff(tn)
            ax.step(tn[1:], v, where="pre", color=farbe, lw=1, label=label)
        tt = np.linspace(0, 2, 400)
        ax.plot(tt, 0.1 * np.pi / 2 * np.sin(2 * np.pi * tt / 2), color="#333", ls="--", lw=1,
                label="Plan")
        ax.set_xlim(-0.1, 2.5)
        ax.set_xlabel("Zeit [s]")
        ax.set_ylabel("Tempo J1 [rad/s]")
        ax.set_title("J1 allein, 0,1 rad in 2 s: Tempo aus den neuen Gelenkwerten (~50/s)")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(BILDER / "06_j1_servo_gegen_online.png")
        plt.close(fig)

    # 7. Software-Stopp (einziger erhaltener Stopp-Mitschrieb)
    stopp = [z for z in zeilen if z["test"] == "online_notstopp"]
    if stopp:
        z = stopp[0]
        t, q = z["npz"][z["test"] + "_t"], z["npz"][z["test"] + "_q"][:, 0]
        fig, ax = plt.subplots(figsize=(7, 3.2))
        ax.plot(t, 1e3 * (q - q[0]), color="#2b6cb0", lw=1.2, label="J1 Ist")
        tt = np.linspace(0, 2, 400)
        ax.plot(tt, 1e3 * 0.1 * (1 - np.cos(2 * np.pi * tt / 2)) / 2, color="#333", ls="--", lw=1,
                label="Plan ohne Stopp")
        ax.axvline(0.5, color="#c53030", lw=1, label="Software-Stopp (T/4)")
        ax.set_xlim(-0.1, 1.6)
        ax.set_ylim(-2, 60)
        ax.set_xlabel("Zeit [s]")
        ax.set_ylabel("Auslenkung J1 [mrad]")
        ax.set_title("Software-Stopp bei voller Fahrt: steht nach %.0f ms, %.1f mrad Restweg"
                     % (z["stopp_ms"], z["stopp_weg"]))
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(BILDER / "07_softwarestopp.png")
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--datum", default="2026-10-13")
    ap.add_argument("--handy", default=str(BERICHTE / "2026-10-13_17-00_Handy-Beschleunigung_Gelenk5.xlsx"))
    args = ap.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    handy = handy_auswerten(args.handy)
    zeilen = lauf_zeilen(berichte(args.datum))
    grafiken(handy, zeilen, plt)
    print("Handy: %d Hz, %d Fahrten, Rauschboden %.3f m/s^2" % (
        handy["fs"], len(handy["fahrten"]), handy["boden_rms"]))
    print("| Lauf | Beginn (Handy) | Zittern RMS | p99 | 15 Hz | 30 Hz | 60 Hz | staerkste Frequenz |")
    print("|---|---|---|---|---|---|---|---|")
    for i, f in enumerate(handy["fahrten"]):
        print("| %s | %s | %.2f | %.2f | %.2f | %.2f | %.3f | %.1f Hz |" % (
            SERIE_NAMEN[i], f["beginn"], f["rms"], f["p99"], f["b15"], f["b30"], f["b60"], f["spitze_hz"]))
    print()
    anhang_tabelle(zeilen)
    print("\nGrafiken: %s" % BILDER)


if __name__ == "__main__":
    main()

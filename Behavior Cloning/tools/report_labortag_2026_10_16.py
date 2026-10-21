"""Auswertung Labortag 2026-10-16 (Zittern-an-der-Anlage.md, Abschnitt 11) -- NUR LESEND.

Rechnet aus den Berichten/Mitschrieben des Tages und den Handy-Dateien alle
Zahlen des Abschnitts nach und legt die Bilder 13-15 ab:

1. Handy H1 je Lauf ueber Zeitfenster (LOS-Zeit + Bewegungsdauer aus dem
   Mitschrieb) -- die automatische Erkennung uebersieht ruhige Fahrten.
2. Schleppfehler so, wie ihn der Recorder prueft (Gelenkziel des Takts i
   gegen die Messung direkt nach dessen Senden), mit den Abbruchkriterien
   a und b (config.RECORDER_*) und der Ziel-Ist-Grenze der Inferenz
   (config.SERVO_MAX_TARGET_GAP_RAD). Mit ``--mitschriebe`` nur dieser
   Teil, fuer beliebige Mitschriebe (z. B. Labortag 19.10.).
3. Bahntreue zeitunabhaengig: naechster Punkt der Planbahn (Gelenkraum und
   Pose), je Abschnitt der Bahn.
4. Effektives Tempo bei ``online`` gegen die uebergebenen Grenzen.
5. Haenger > 150 ms in servo_j/online_j (Sendezeitpunkte) gegen Luecken im
   parallelen Lese-Thread.

FK lokal aus der URDF-Kette (Flansch; das Werkzeug ZimmerLWR50 ist nicht
enthalten -- Abstaende gelten am Flansch).

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/report_labortag_2026_10_16.py --handy-ordner "D:/.../Zittern"
    python tools/report_labortag_2026_10_16.py --ohne-bilder
    python tools/report_labortag_2026_10_16.py --mitschriebe "Berichte/2026-10-19_*_Mitschrieb.npz"
"""

import argparse
import glob
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

import check_servo_stream as strom
import report_zittern as rz
from bc import config
from bc.urdf import KinematicChain

BC = Path(__file__).resolve().parent.parent
BERICHTE = BC / "Berichte"
BILDER = BC / "Dokumentation" / "Bilder" / "Zittern"
T = 1.0 / config.CONTROL_RATE_HZ

#: Laeufe mit LOS-Zeit (Berichte/2026-10-16_Labortag-Uebersicht.md) und Handy-Datei.
LAEUFE = [
    # (Kurzname, Handy-Exportzeit, LOS, Bericht-Stempel, Laufname, Fenster s ohne Mitschrieb)
    ("Neura-Standard", "13-00-57", "12:42:59", None, None, 14.0),
    ("D1 linear", "13-00-57", "12:44:36", "12-47-00", "linear-60-r0-o1", None),
    ("D1 pos", "13-00-57", "12:45:01", "12-47-00", "pos-60-r0-o1", None),
    ("D1 online", "13-00-57", "12:45:25", "12-47-00", "online-15-r0-o1", None),
    ("D1 linear o0,5", "13-00-57", "12:45:55", "12-47-00", "linear-60-r0-o0.5", None),
    ("D1 pos r1", "13-00-57", "12:46:18", "12-47-00", "pos-60-r1-o1", None),
    ("D1 online r1", "13-00-57", "12:46:42", "12-47-00", "online-15-r1-o1", None),
    ("D2 linear", "13-00-57", "12:58:22", "13-00-44", "linear-60-r0-o1", None),
    ("D2 pos", "13-00-57", "12:58:46", "13-00-44", "pos-60-r0-o1", None),
    ("D2 online", "13-00-57", "12:59:10", "13-00-44", "online-15-r0-o1", None),
    ("D2 linear o0,5", "13-00-57", "12:59:39", "13-00-44", "linear-60-r0-o0.5", None),
    ("D2 pos r1", "13-00-57", "13:00:03", "13-00-44", "pos-60-r1-o1", None),
    ("D2 online r1", "13-00-57", "13:00:26", "13-00-44", "online-15-r1-o1", None),
    ("T1 pos", "14-05-57", "14:01:18", "14-01-59", "pos-60-r0-o1", None),
    ("T1 online", "14-05-57", "14:01:41", "14-01-59", "online-15-r0-o1", None),
    ("W online", "14-05-57", "14:02:52", "14-03-10", "online-15-r0-o1", None),
    ("W2 online", "14-05-57", "14:04:25", "14-05-06", "online-15-r0-o1", None),
    ("W2 pos", "14-05-57", "14:04:50", "14-05-06", "pos-60-r0-o1", None),
    ("T2 online x2/x3", "14-19-07", "14:15:05", "14-15-21", "online-15-r0-o1", None),
    ("T3 online Dwell 1,5 s", "14-19-07", "14:18:40", "14-18-59", "online-15-r0-o1", None),
]
HANDY_NAME = "Beschleunigung mit g 2026-10-16 %s.xlsx"

#: Abschnitte der Aufnahmebahn (Plan-Takte, Dwell 0,5 s; ab PICK +15 bei 1,5 s)
ABSCHNITTE = [(0, 50, "CLEAR_FOV -> APPROACH_01"), (50, 125, "APPROACH_01 -> PRE_GRASP"),
              (125, 147, "PRE_GRASP -> PICK"), (147, 175, "PICK -> PRE_GRASP"),
              (175, 10 ** 6, "-> PRE_PLACE")]


def mitschrieb(stempel):
    return np.load(str(BERICHTE / ("2026-10-16_%s_Servo-Strom_Mitschrieb.npz" % stempel)))


def plan_von(z, name):
    return z["plan_r1"] if "-r1-" in name else z["plan_r0"]


def bewegung(z, name):
    t, q = z[name + "_t"], z[name + "_q"]
    bew = np.flatnonzero(np.max(np.abs(np.diff(q, axis=0)), axis=1) > 2e-4)
    return float(t[bew[0]]), float(t[bew[-1] + 1])


def handy_tabelle(ordner):
    cache, zeilen = {}, []
    for kurz, datei, los, stempel, name, dauer in LAEUFE:
        if datei not in cache:
            cache[datei] = rz.handy_auswerten(str(Path(ordner) / (HANDY_NAME % datei)))
        h = cache[datei]
        tag = datetime.fromtimestamp(next(s for _, _, s in h["abschnitte"] if s is not None)).date()
        von = datetime.combine(tag, datetime.strptime(los, "%H:%M:%S").time()).timestamp()
        if stempel:
            a, e = bewegung(mitschrieb(stempel), name)
            # LOS -> 0,3 s Ruhe im Mitschrieb -> activate_servo -> t = 0
            v0, v1 = von + 0.3 + max(a, 0.0), von + 0.3 + e
        else:
            v0, v1 = von, von + dauer
        fa = rz.fenster_auswerten(h, v0, v1)
        zeilen.append((kurz, fa, v1 - v0))
    return zeilen


def schleppfehler(z, name):
    """Recorder-Pruefung nachgestellt: Fehler je Takt, Abbruch a/b (erster Takt).

    Messung direkt nach dem Senden des Ziels i (``strom.hostzeit``), wie
    der Recorder liest -- NICHT zur Uhrzeit i/15 s: nach jedem Haenger
    sendet check_servo_stream den Rest der Bahn spaeter, gegen die Uhr
    erschien das als wachsender Schleppfehler (bis 0,46 statt 0,15 rad,
    Kriterium a/b schlugen scheinbar an; korrigiert PC 2026-10-19).
    """
    q = plan_von(z, name)
    tt, qq = z[name + "_t"], z[name + "_q"]
    lauf = strom.parse_lauf(name.split("#")[0])
    t_ist = strom.hostzeit(np.arange(len(q)) * T, z[name + "_send"],
                           strom.Profil(q, "linear"), lauf["rate"])
    mess = np.array([qq[max(np.searchsorted(tt, t, side="right") - 1, 0)] for t in t_ist])
    fehler = np.max(np.abs(mess - q), axis=1)
    lim, n = config.RECORDER_MAX_FOLLOW_ERROR_RAD, config.RECORDER_FOLLOW_ERROR_STEPS
    a_at, schlecht = None, 0
    for i, e in enumerate(fehler):
        schlecht = schlecht + 1 if e > lim else 0
        if schlecht >= n and a_at is None:
            a_at = i
    b_at = None
    for i in range(n, len(q)):
        d_cmd = np.max(np.abs(q[i] - q[i - n]))
        d_mess = np.max(np.abs(mess[i] - mess[i - n]))
        if d_cmd > config.START_POSE_TOL_RAD and d_mess < config.RECORDER_MIN_FOLLOW_RATIO * d_cmd:
            b_at = i
            break
    return fehler, a_at, b_at


def schleppfehler_tabelle(dateien):
    """Teil 2 fuer eine Liste von Mitschrieben; liefert {(stempel, lauf): fehler}."""
    print("\n== 2. Schleppfehler wie im Recorder (a: > %.2f rad ueber %d Takte; b: Arm < %.0f %% "
          "des Sollwegs) und Inferenz-Grenze %.2f rad"
          % (config.RECORDER_MAX_FOLLOW_ERROR_RAD, config.RECORDER_FOLLOW_ERROR_STEPS,
             100 * config.RECORDER_MIN_FOLLOW_RATIO, config.SERVO_MAX_TARGET_GAP_RAD))
    kurven = {}
    for datei in dateien:
        z = np.load(str(datei))
        tag, stempel = Path(datei).name[:10], Path(datei).name[11:19]
        for name in [k[:-2] for k in z.files if k.endswith("_t")]:
            try:
                plan = "plan_r%g" % strom.parse_lauf(name.split("#")[0])["rauschen"]
            except ValueError:
                continue   # kein Lauf von check_servo_stream (andere Werkzeuge)
            if name + "_send" not in z.files or plan not in z.files:
                continue
            fehler, a_at, b_at = schleppfehler(z, name)
            kurven[(stempel, name)] = fehler
            print("  %s %s %-20s max %.3f rad (Takt %3d) | Abbruch a: %-5s b: %-5s | "
                  "Inferenz-Grenze ueberschritten: %s"
                  % (tag, stempel, name, fehler.max(), fehler.argmax(), a_at, b_at,
                     "ja" if fehler.max() > config.SERVO_MAX_TARGET_GAP_RAD else "nein"))
    return kurven


def bahntreue(z, name, kette):
    """Zeitunabhaengig: naechster Punkt der Planbahn je Messwert, nach Abschnitt."""
    q = plan_von(z, name)
    prof = strom.Profil(q, "linear")
    ts = np.arange(0.0, prof.dauer, 0.005)
    qd = prof.positionen(ts)
    Pd = np.array([kette.fk(x) for x in qd])
    tn, qn = strom.neue_werte(z[name + "_t"], z[name + "_q"])
    m = tn > 0
    qn = qn[m]
    P = np.array([kette.fk(x) for x in qn])
    out = []
    for Pi, qi in zip(P, qn):
        dp = np.linalg.norm(Pd[:, :3, 3] - Pi[:3, 3], axis=1) * 1e3
        tr = np.einsum("nij,ij->n", Pd[:, :3, :3], Pi[:3, :3])
        dr = np.degrees(np.arccos(np.clip((tr - 1) / 2, -1, 1)))
        k = int(np.argmin(dp + dr))          # 1 Grad ~ 1 mm
        dj = float(np.min(np.max(np.abs(qd - qi), axis=1)))
        out.append((ts[k] / T, dp[k], dr[k], dj))
    return np.array(out), Pd, P


def effektives_tempo(z, name, kette):
    tn, qn = strom.neue_werte(z[name + "_t"], z[name + "_q"])
    tr, _ = strom.raster_zeiten(tn)
    P = np.array([kette.fk(x) for x in qn])
    dt = np.diff(tr)
    v = np.linalg.norm(np.diff(P[:, :3, 3], axis=0), axis=1) / dt
    tr_ = np.einsum("nij,nij->n", P[:-1, :3, :3], P[1:, :3, :3])
    w = np.arccos(np.clip((tr_ - 1) / 2, -1, 1)) / dt
    tm = 0.5 * (tr[1:] + tr[:-1])
    m = (tm > 0.5) & (tm < 3.5)               # erster Dreh-Abschnitt (PTP)
    return float(np.median(v[m])), float(np.median(w[m]))


def haenger():
    zeilen, summe = [], {"servo_j": [0, 0], "online": [0, 0]}
    for f in sorted(glob.glob(str(BERICHTE / "2026-10-16_*_Mitschrieb.npz"))):
        z = np.load(f)
        for k in z.files:
            if not k.endswith("_send") or z[k].ndim != 2 or len(z[k]) < 5:
                continue
            s = z[k]
            dauer = (s[:, 2] - s[:, 1]) * 1e3
            lang = dauer > 150
            art = "online" if "online" in k else "servo_j"
            summe[art][0] += len(s)
            summe[art][1] += int(lang.sum())
            tt = z[k[:-5] + "_t"]
            luecke = float(np.max(np.diff(tt)) * 1e3) if len(tt) > 2 else float("nan")
            if lang.any():
                zeilen.append((Path(f).name[11:30], k[:-5], len(s), dauer[lang], luecke))
    return zeilen, summe


def bilder(handy, fehler_kurven, bahnen):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import report_servo_stream as rs

    rs.stil(plt)
    BILDER.mkdir(parents=True, exist_ok=True)

    # 13: H1 je Art (Mittel der Wiederholungen) -- nur mit Handy-Daten, sonst
    # bliebe das vorhandene Bild nicht erhalten (leer ueberschrieben)
    if handy:
        gruppen = [("Neura-Standard", ["Neura-Standard"]),
                   ("online x1,3/x1,5", ["D1 online", "D2 online", "T1 online", "W online", "W2 online"]),
                   ("linear 60 Hz, Override 0,5", ["D1 linear o0,5", "D2 linear o0,5"]),
                   ("online x1,3/x1,5, Dwell 1,5 s (T3)", ["T3 online Dwell 1,5 s"]),
                   ("online x2/x3 (T2)", ["T2 online x2/x3"]),
                   ("pos 60 Hz (v = a = 0)", ["D1 pos", "D2 pos", "T1 pos", "W2 pos"]),
                   ("linear 60 Hz (bisher)", ["D1 linear", "D2 linear"])]
        werte = {k: fa["rms"] for k, fa, _ in handy if fa}
        namen, mittel, spann = [], [], []
        for titel, glieder in gruppen:
            v = [werte[g] for g in glieder if g in werte]
            if v:
                namen.append(titel)
                mittel.append(float(np.mean(v)))
                spann.append((min(v), max(v)))
        fig, ax = plt.subplots(figsize=(10, 4.2))
        y = np.arange(len(namen))[::-1]
        farben = [rs.GRAU if "Neura" in n else (rs.ORANGE if "bisher" in n else rs.BLAU) for n in namen]
        ax.barh(y, mittel, color=farben, height=0.6)
        for yi, m, (lo, hi) in zip(y, mittel, spann):
            ax.text(m + 0.03, yi, "%.2f" % m + ("" if lo == hi else "  (%.2f-%.2f)" % (lo, hi)),
                    va="center", fontsize=8, color=rs.TEXT2)
        ax.set_yticks(y)
        ax.set_yticklabels(namen)
        ax.set_xlabel("Handy H1 (Unterarm, Achse 5): Beschleunigung ueber 5 Hz, RMS (m/s^2)")
        ax.grid(axis="y", visible=False)
        ax.set_title("Anlage 16.10., ganze Aufnahmebahn, Rauschen 0 -- grau = Pendant-Programm von Neura",
                     fontsize=9.5, loc="left")
        fig.tight_layout()
        fig.savefig(BILDER / "13_labortag16_h1.png")
        plt.close(fig)

    # 14: Schleppfehler wie im Recorder
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for (label, fehler), farbe in zip(fehler_kurven, (rs.BLAU, rs.TUERKIS, rs.ORANGE)):
        ax.plot(np.arange(len(fehler)) / config.CONTROL_RATE_HZ, fehler, color=farbe, lw=1.6, label=label)
    for wert, txt in ((config.RECORDER_MAX_FOLLOW_ERROR_RAD, "Recorder bricht ab (4 Takte ueber %.2f rad)"
                       % config.RECORDER_MAX_FOLLOW_ERROR_RAD),
                      (config.SERVO_MAX_TARGET_GAP_RAD, "Inferenz bricht ab (Ziel-Ist > %.2f rad)"
                       % config.SERVO_MAX_TARGET_GAP_RAD)):
        ax.axhline(wert, color=rs.GRAU, ls=":", lw=1)
        ax.text(0.1, wert + 0.012, txt, fontsize=8, color=rs.TEXT2)
    ax.set_xlabel("Zeit seit Bahnbeginn (s)")
    ax.set_ylabel("max. Gelenk: Ist - Ziel des Takts (rad)")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Anlage 16.10. (D1, Rauschen 0) -- Schleppfehler, wie Recorder und Inferenz ihn pruefen",
                 fontsize=9.5, loc="left")
    fig.tight_layout()
    fig.savefig(BILDER / "14_labortag16_schleppfehler.png")
    plt.close(fig)

    # 15: Bahn von der Seite und von oben (Flansch)
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.4))
    Pd = bahnen[0][1]
    for ax, (i, j, titel) in zip(axs, ((0, 2, "Seitenansicht (x-z)"), (0, 1, "Draufsicht (x-y)"))):
        ax.plot(Pd[:, i, 3] * 1e3, Pd[:, j, 3] * 1e3, color=rs.GRAU, lw=3, label="Planbahn")
        for (label, _, P), farbe in zip(bahnen, (rs.BLAU, rs.TUERKIS)):
            ax.plot(P[:, i, 3] * 1e3, P[:, j, 3] * 1e3, color=farbe, lw=1.2, label=label)
        ax.set_xlabel("%s (mm)" % "xyz"[i])
        ax.set_ylabel("%s (mm)" % "xyz"[j])
        ax.set_title(titel, fontsize=9, loc="left")
        ax.set_aspect("equal", adjustable="datalim")
    axs[0].legend(fontsize=8)
    fig.suptitle("Anlage 16.10. (D1, Rauschen 0) -- online kuerzt die Dreh-Abschnitte ab, "
                 "servo_j Override 0,5 bleibt auf der Bahn (Flansch, lokale FK)", fontsize=9.5)
    fig.tight_layout()
    fig.savefig(BILDER / "15_labortag16_bahn.png")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--handy-ordner", default=None,
                    help="Ordner mit den phyphox-Exporten des Tages (ohne: Handy-Teil entfaellt)")
    ap.add_argument("--ohne-bilder", action="store_true")
    ap.add_argument("--mitschriebe", default=None,
                    help="Glob auf *_Mitschrieb.npz: nur Teil 2 (Schleppfehler) fuer diese Dateien")
    args = ap.parse_args()
    if args.mitschriebe:
        schleppfehler_tabelle(sorted(glob.glob(args.mitschriebe)))
        return
    kette = KinematicChain.from_urdf(config.URDF_PATH)

    handy = []
    if args.handy_ordner:
        print("== 1. Handy H1 je Lauf (Fenster LOS + Bewegung laut Mitschrieb)")
        handy = handy_tabelle(args.handy_ordner)
        for kurz, fa, dauer in handy:
            if fa is None:
                print("  %-24s nicht in der Messung" % kurz)
                continue
            f, s = fa["f"], fa["spektrum"]
            g = np.sum(s[f >= 5] ** 2)
            ant = [np.sum(s[(f >= lo) & (f < hi)] ** 2) / g for lo, hi in ((5, 8.5), (8.5, 11), (14, 17))]
            print("  %-24s %s %5.1f s  RMS %.2f  p99 %.2f  Spitze %5.1f Hz  Anteil 5-8,5/8,5-11/14-17 Hz "
                  "%.2f/%.2f/%.2f" % (kurz, fa["beginn"], dauer, fa["rms"], fa["p99"], fa["spitze_hz"], *ant))

    kurven = schleppfehler_tabelle(
        [BERICHTE / ("2026-10-16_%s_Servo-Strom_Mitschrieb.npz" % s)
         for s in ("12-47-00", "13-00-44", "14-15-21", "14-18-59")])

    print("\n== 3. Bahntreue zeitunabhaengig (Flansch; max je Abschnitt: Position mm / Orientierung Grad / Gelenk rad)")
    bahnen = []
    for stempel, name in (("12-47-00", "online-15-r0-o1"), ("14-15-21", "online-15-r0-o1"),
                          ("12-47-00", "linear-60-r0-o0.5"), ("12-47-00", "pos-60-r0-o1")):
        res, Pd, P = bahntreue(mitschrieb(stempel), name, kette)
        if stempel == "12-47-00" and name in ("online-15-r0-o1", "linear-60-r0-o0.5"):
            bahnen.append(("online x1,3/x1,5" if "online" in name else "servo_j linear, Override 0,5", Pd, P))
        teile = []
        for a, e, txt in ABSCHNITTE:
            m = (res[:, 0] >= a) & (res[:, 0] < e)
            if m.any():
                teile.append("%s %.0f/%.0f/%.2f" % (txt.split(" -> ")[-1], res[m, 1].max(), res[m, 2].max(), res[m, 3].max()))
        print("  %s %-18s %s" % (stempel, name, " | ".join(teile)))

    print("\n== 4. Effektives Tempo im ersten Dreh-Abschnitt (0,5-3,5 s) gegen die Grenzen")
    for stempel, name, grenzen in (("12-47-00", "online-15-r0-o1", (0.195, 0.65)),
                                   ("14-15-21", "online-15-r0-o1", (0.30, 1.0)),
                                   ("12-47-00", "linear-60-r0-o0.5", None)):
        v, w = effektives_tempo(mitschrieb(stempel), name, kette)
        print("  %s %-18s TCP %.3f m/s, Drehung %.3f rad/s%s"
              % (stempel, name, v, w, "" if grenzen is None else
                 "  (Grenzen %.3f / %.2f -> Drehung %.0f %% der Grenze)" % (grenzen[0], grenzen[1], 100 * w / grenzen[1])))

    print("\n== 5. Haenger > 150 ms in servo_j/online_j")
    zeilen, summe = haenger()
    for datei, name, n, lang, luecke in zeilen:
        print("  %s %-18s %4d Aufrufe, %d Haenger (%s ms), groesste Luecke im Lese-Thread %.0f ms"
              % (datei, name, n, len(lang), ", ".join("%.0f" % x for x in lang), luecke))
    for art, (n, l) in summe.items():
        print("  SUMME %-8s %d Aufrufe, %d Haenger (%.2f %%)" % (art, n, l, 100.0 * l / max(n, 1)))

    if not args.ohne_bilder:
        kurven_bild = [("online x1,3/x1,5", kurven[("12-47-00", "online-15-r0-o1")]),
                       ("servo_j linear, Override 0,5", kurven[("12-47-00", "linear-60-r0-o0.5")]),
                       ("servo_j pos", kurven[("12-47-00", "pos-60-r0-o1")])]
        bilder(handy, kurven_bild, bahnen)
        print("\nBilder: %s (13-15%s)" % (BILDER, "" if handy else "; 13 ohne Handy unveraendert"))


if __name__ == "__main__":
    main()

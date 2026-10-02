"""Kosten eines NeuraPy-Aufrufs messen -- die Groesse, an der die 15-Hz-Schleife haengt.

NUR LESEN -- der Roboter bewegt sich nicht, es wird nichts bestromt
(Vorbild tools/check_kinematics.py). ``servo_j`` wird NICHT gesendet.

ACHTUNG, GELTUNGSBEREICH: Alle Zahlen gelten fuer die Steuerung, gegen die
gemessen wurde. In der virtuellen Steuerung sind sie ein Stellvertreter,
kein Massstab -- die Sim "taugt fuer Formate, Semantik und Kinematik, nicht
fuer Timing" (Sim-Inbetriebnahme-Befunde.md). An der Anlage ist dieselbe
Messung zu wiederholen, BEVOR daraus Parameter abgeleitet werden. Die
Einordnung steht in Dokumentation/Taktzeit-und-RPC-Latenz.md.

Hintergrund (Befund 2026-10-01): Die Aufzeichnung lief in der VM mit dem
2,4- bis 3,8-fachen der geplanten Zeit, der Arm blieb unterwegs stehen
(RCSC_102), und die Episoden wurden wegen "Latenzbudget" verworfen --
obwohl beide Kameras simuliert waren. Beides haengt an derselben Groesse:
wie lange ein einzelner RPC-Aufruf dauert.

Je 15-Hz-Takt (66,7 ms) setzt bc/recorder.py SECHS Aufrufe ab:

    4 x servo_j            (SERVO_RATE_HZ / CONTROL_RATE_HZ = 60/15)
    1 x get_current_joint_angles_with_timestamp   \\  zusammen read_state()
    1 x compute_forward_kinematics                /

Damit bleiben pro Aufruf 11,1 ms. NeuraPy oeffnet fuer JEDEN Aufruf einen
neuen TCP-Socket (``generate_function`` in neurapy/robot.py), setzt unter
Windows je Aufruf einen Console-Handler und formatiert eine Logzeile mit
allen Argumenten. Dieses Skript trennt die Anteile:

    roh-TCP      Verbindungsaufbau + Abbau allein
    roh-RPC      eigenes JSON ueber einen eigenen Socket (= Serverzeit)
    neurapy      derselbe Aufruf ueber den Client (= plus Client-Overhead)

Ausserdem wird der Versatz nachgestellt, den bc/sync.evaluate() als
"Frame ueber dem Zeitbudget" meldet: read_state() stempelt die Gelenke in
der MITTE des Winkel-Aufrufs, der Recorder greift die Kamerabilder aber
erst NACH der anschliessenden FK ab. Die Differenz ist reine
Controller-Latenz und hat mit den Kameras nichts zu tun.

Abschnitt 4 und 5 gehen zwei Fragen nach, die sich aus der ersten Messung
(2026-10-01) ergeben haben:

* Nimmt die Steuerung MEHRERE Aufrufe ueber eine Verbindung an? Wenn ja,
  lohnt eine stehende Verbindung im Adapter -- wenn nein, ist das Thema
  erledigt.
* Die 1000-ms-Haenger: 2 von 30 reinen TCP-Verbindungen brauchten 1003 ms.
  Das ist die Signatur eines verworfenen SYN. Offen ist, ob das nur im
  dichten Messburst auftritt oder auch bei normal getaktetem Verkehr --
  im zweiten Fall reisst ein einziger Haenger jeden Servo-Strom.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/check_rpc_latency.py
    python tools/check_rpc_latency.py --wiederholungen 50
    python tools/check_rpc_latency.py --dauerlauf 15
"""

import argparse
import json
import socket
import statistics
import time
from datetime import datetime
from pathlib import Path

import _bootstrap  # noqa: F401

from bc import config
from bc.clock import host_time

BERICHTE = Path(__file__).resolve().parent.parent / "Berichte"

#: Aufrufe, die bc/recorder.py je Takt absetzt. servo_j fehlt bewusst --
#: es bewegt den Arm und wird hier nicht gemessen, sondern aus den uebrigen
#: Werten hochgerechnet.
TAKT_AUFRUFE = 6


def section(titel):
    print("\n" + "=" * 68)
    print(titel)
    print("=" * 68)


def kennzahlen(werte_s):
    """min / median / p95 / max in Millisekunden."""
    ms = sorted(1e3 * w for w in werte_s)
    p95 = ms[min(len(ms) - 1, int(round(0.95 * (len(ms) - 1))))]
    return {
        "min": ms[0],
        "median": statistics.median(ms),
        "p95": p95,
        "max": ms[-1],
        "n": len(ms),
    }


def zeile(label, k):
    print("  %-46s %6.1f %6.1f %6.1f %6.1f ms" % (
        label, k["min"], k["median"], k["p95"], k["max"]))


def kopf():
    print("  %-46s %6s %6s %6s %6s" % ("", "min", "med", "p95", "max"))


# -- Messungen ------------------------------------------------------------

def miss_tcp(adresse, n):
    """Nur Verbindungsaufbau und -abbau, ohne ein einziges Byte Nutzlast."""
    werte = []
    for _ in range(n):
        t0 = host_time()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.connect(adresse)
        finally:
            s.close()
        werte.append(host_time() - t0)
    return werte


def roh_rpc(adresse, funktion, args=(), kwargs=None):
    """Ein Aufruf ueber einen eigenen Socket -- dasselbe Protokoll wie NeuraPy,
    aber ohne dessen Client-Overhead. Misst die reine Serverzeit."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(adresse)
    try:
        nachricht = {"function": funktion, "args": list(args), "kwargs": kwargs or {}}
        s.sendall(json.dumps(nachricht).encode("utf-8"))
        daten = ""
        for _ in range(6):
            stueck = s.recv(8192).decode("utf-8")
            if not stueck:
                break
            daten += stueck
            try:
                return json.loads(daten)
            except json.JSONDecodeError:
                continue
        raise RuntimeError("keine vollstaendige Antwort auf '%s'" % funktion)
    finally:
        s.close()


def miss_roh(adresse, funktion, n, args=(), kwargs=None):
    werte = []
    for _ in range(n):
        t0 = host_time()
        roh_rpc(adresse, funktion, args, kwargs)
        werte.append(host_time() - t0)
    return werte


def miss_neurapy(aufruf, n):
    werte = []
    for _ in range(n):
        t0 = host_time()
        aufruf()
        werte.append(host_time() - t0)
    return werte


def mehrere_je_verbindung(adresse, funktion, anzahl=3, timeout=3.0):
    """Nimmt die Steuerung mehrere Aufrufe ueber EINE Verbindung an?

    Sendet ``anzahl`` Anfragen nacheinander auf demselben Socket und misst
    jede einzeln. Antwortet die zweite nicht, schliesst der Server nach dem
    ersten Aufruf -- dann ist eine stehende Verbindung im Adapter nicht
    moeglich und die Frage erledigt.

    Rueckgabe: (beantwortet, dauern_s, hinweis).
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    dauern = []
    try:
        s.connect(adresse)
        for _ in range(anzahl):
            t0 = host_time()
            nachricht = {"function": funktion, "args": [], "kwargs": {}}
            s.sendall(json.dumps(nachricht).encode("utf-8"))
            daten = ""
            antwort = None
            for _ in range(6):
                stueck = s.recv(8192).decode("utf-8")
                if not stueck:
                    break
                daten += stueck
                try:
                    antwort = json.loads(daten)
                    break
                except json.JSONDecodeError:
                    continue
            if antwort is None:
                return len(dauern), dauern, "Server schliesst nach dem Aufruf"
            dauern.append(host_time() - t0)
        return len(dauern), dauern, ""
    except socket.timeout:
        return len(dauern), dauern, "Zeitlimit -- keine weitere Antwort"
    except OSError as exc:
        return len(dauern), dauern, "Socket-Fehler: %s" % exc
    finally:
        s.close()


def dauerlauf(aufruf, sekunden, rate_hz):
    """Getakteter Verkehr statt Burst -- sucht die Sekunden-Haenger.

    Ruft mit ``rate_hz`` auf, so wie der Recorder es tut, und gibt alle
    Einzeldauern zurueck. Der Burst in Abschnitt 1 beantwortet die Frage
    nicht: 30 Verbindungen so schnell wie moeglich koennen die
    Accept-Warteschlange selbst ueberlaufen lassen.
    """
    periode = 1.0 / rate_hz
    ende = host_time() + sekunden
    naechster = host_time()
    werte = []
    while host_time() < ende:
        warten = naechster - host_time()
        if warten > 0:
            time.sleep(warten)
        t0 = host_time()
        aufruf()
        werte.append(host_time() - t0)
        naechster += periode
        if naechster < host_time():
            naechster = host_time()
    return werte


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wiederholungen", type=int, default=30,
                    help="Aufrufe je Messreihe (Standard 30)")
    ap.add_argument("--dauerlauf", type=float, default=10.0,
                    help="Sekunden getakteter Verkehr fuer die Haenger-Suche "
                         "(Standard 10, 0 schaltet ab)")
    args = ap.parse_args()
    n = max(5, args.wiederholungen)

    from neurapy import robot as neurapy_robot

    adresse = (neurapy_robot.SOCKET_ADDRESS, neurapy_robot.SOCKET_PORT)
    bericht = {
        "erzeugt": datetime.now().isoformat(timespec="seconds"),
        "geltungsbereich": "Zahlen gelten NUR fuer die hier gemessene Steuerung. "
                           "In der VM Stellvertreter, kein Massstab -- an der "
                           "Anlage neu messen "
                           "(Dokumentation/Taktzeit-und-RPC-Latenz.md).",
        "adresse": "%s:%d" % adresse,
        "wiederholungen": n,
        "reihen": {},
        "befunde": [],
    }

    def befund(stufe, text):
        bericht["befunde"].append([stufe, text])
        print("  [%s] %s" % (stufe.upper(), text))

    section("0. Verbindung (nichts wird bewegt, nichts bestromt)")
    print("  Steuerung: %s:%d" % adresse)
    bot = neurapy_robot.Robot()
    version = getattr(bot, "version", None)
    print("  Server meldet: %s" % (version if version else "(keine Version gelesen)"))
    winkel = [float(w) for w in bot.get_current_joint_angles()]
    print("  Aktuelle Gelenkstellung: %s" % [round(w, 4) for w in winkel])

    section("1. Anteile eines einzelnen Aufrufs")
    kopf()
    reihen = [
        ("TCP-Verbindung allein (ohne Nutzlast)",
         lambda: miss_tcp(adresse, n)),
        ("roh-RPC get_current_joint_angles",
         lambda: miss_roh(adresse, "get_current_joint_angles", n)),
        ("neurapy get_current_joint_angles",
         lambda: miss_neurapy(bot.get_current_joint_angles, n)),
        ("neurapy ..._with_timestamp",
         lambda: miss_neurapy(bot.get_current_joint_angles_with_timestamp, n)),
        ("neurapy compute_forward_kinematics",
         lambda: miss_neurapy(lambda: bot.compute_forward_kinematics(
             joint_angles=winkel, target_frame="tool", representation="rpy"), n)),
    ]
    for label, messung in reihen:
        k = kennzahlen(messung())
        bericht["reihen"][label] = k
        zeile(label, k)

    tcp = bericht["reihen"]["TCP-Verbindung allein (ohne Nutzlast)"]["median"]
    roh = bericht["reihen"]["roh-RPC get_current_joint_angles"]["median"]
    client = bericht["reihen"]["neurapy get_current_joint_angles"]["median"]
    print()
    print("  Aufschluesselung (Median):")
    print("    Socket-Aufbau            %6.1f ms" % tcp)
    print("    Serverzeit darueber      %6.1f ms" % (roh - tcp))
    print("    NeuraPy-Client darueber  %6.1f ms" % (client - roh))

    section("2. Hochrechnung auf den 15-Hz-Takt des Recorders")
    budget_ms = 1e3 / config.CONTROL_RATE_HZ
    je_aufruf = budget_ms / TAKT_AUFRUFE
    takt_ms = (
        4 * client
        + bericht["reihen"]["neurapy ..._with_timestamp"]["median"]
        + bericht["reihen"]["neurapy compute_forward_kinematics"]["median"]
    )
    bericht["takt_ms_geschaetzt"] = takt_ms
    bericht["takt_ms_budget"] = budget_ms
    print("  Budget je Takt (%g Hz)                        %6.1f ms" % (
        config.CONTROL_RATE_HZ, budget_ms))
    print("  davon je Aufruf bei %d Aufrufen je Takt        %6.1f ms" % (
        TAKT_AUFRUFE, je_aufruf))
    print("  Gemessen je Takt (servo_j mit den Kosten von")
    print("  get_current_joint_angles angesetzt)           %6.1f ms" % takt_ms)
    faktor = takt_ms / budget_ms
    print("  -> Schleife laeuft %.1f-fach zu langsam (%.1f Hz statt %g Hz)" % (
        faktor, 1e3 / takt_ms, config.CONTROL_RATE_HZ))
    if faktor > 1.0:
        befund("fehler" if faktor > 1.5 else "warnung",
               "Ein Takt braucht %.0f ms, erlaubt sind %.0f ms. Der Arm bekommt "
               "seine Sollwerte zu spaet -- das ist die Ursache fuer "
               "abbrechende Bahnen (RCSC_102) UND fuer die als "
               "'Latenzbudget' verworfenen Episoden." % (takt_ms, budget_ms))
    else:
        befund("ok", "Die Aufrufkosten passen in den Takt.")

    section("3. Versatz, den der Recorder als 'Frame ueber Budget' meldet")
    print("  read_state() stempelt die Gelenke in der Mitte des Winkel-Aufrufs;")
    print("  die Kamerabilder werden erst NACH der FK abgegriffen. Die Differenz")
    print("  ist reine Controller-Latenz -- unabhaengig von der Kamera.")
    versaetze = []
    for _ in range(n):
        t_send = host_time()
        bot.get_current_joint_angles_with_timestamp()
        t_recv = host_time()
        t_joints = 0.5 * (t_send + t_recv)
        bot.compute_forward_kinematics(
            joint_angles=winkel, target_frame="tool", representation="rpy")
        t_kamera = host_time()  # so frisch, wie ein Sim-Frame nur sein kann
        versaetze.append(t_kamera - t_joints)
    k = kennzahlen(versaetze)
    bericht["reihen"]["Versatz Roboter <-> Kamera (Sim)"] = k
    kopf()
    zeile("Versatz Roboter <-> frischester Frame", k)
    grenze_ms = 1e3 * config.SYNC_MAX_SKEW_S
    ueber = sum(1 for v in versaetze if v > config.SYNC_MAX_SKEW_S)
    print("  Grenze SYNC_MAX_SKEW_S: %.0f ms -- darueber: %d von %d (%.0f %%)" % (
        grenze_ms, ueber, len(versaetze), 100.0 * ueber / len(versaetze)))
    if ueber:
        befund("fehler",
               "%.0f %% der Takte reissen das Budget allein durch die "
               "Controller-Latenz. Eine perfekte Kamera aendert daran nichts."
               % (100.0 * ueber / len(versaetze)))
    else:
        befund("ok", "Der Versatz bleibt unter der Grenze.")

    section("4. Nimmt die Steuerung mehrere Aufrufe je Verbindung an?")
    print("  Entscheidet, ob eine stehende Verbindung im Adapter ueberhaupt")
    print("  moeglich ist. Drei Aufrufe nacheinander auf EINEM Socket:")
    beantwortet, dauern, hinweis = mehrere_je_verbindung(
        adresse, "get_current_joint_angles", anzahl=3)
    bericht["mehrere_je_verbindung"] = {
        "beantwortet": beantwortet,
        "dauern_ms": [round(1e3 * d, 2) for d in dauern],
        "hinweis": hinweis,
    }
    for i, d in enumerate(dauern, 1):
        print("    Aufruf %d: %6.1f ms" % (i, 1e3 * d))
    if hinweis:
        print("    %s" % hinweis)
    if beantwortet >= 2:
        befund("ok",
               "Die Steuerung beantwortet mehrere Aufrufe je Verbindung. Eine "
               "stehende Verbindung im Adapter spart den Socket-Aufbau und "
               "umgeht die Haenger aus Abschnitt 5 -- lohnt sich.")
    else:
        befund("info",
               "Die Steuerung beantwortet nur EINEN Aufruf je Verbindung. Eine "
               "stehende Verbindung ist damit nicht moeglich; der Socket-Aufbau "
               "je Aufruf ist nicht vermeidbar.")

    if args.dauerlauf > 0:
        section("5. Haenger bei getaktetem Verkehr (nicht im Burst)")
        print("  %.0f s lang mit %g Hz aufrufen -- so, wie der Recorder es tut."
              % (args.dauerlauf, config.SERVO_RATE_HZ))
        werte = dauerlauf(bot.get_current_joint_angles,
                          args.dauerlauf, config.SERVO_RATE_HZ)
        k = kennzahlen(werte)
        bericht["reihen"]["Dauerlauf get_current_joint_angles"] = k
        kopf()
        zeile("Dauerlauf, %d Aufrufe" % len(werte), k)
        schwellen = (0.2, 0.5, 1.0)
        zaehler = {}
        for schwelle in schwellen:
            treffer = sum(1 for w in werte if w > schwelle)
            zaehler["%.1fs" % schwelle] = treffer
            print("    ueber %4.0f ms: %d von %d (%.1f %%)" % (
                1e3 * schwelle, treffer, len(werte),
                100.0 * treffer / max(1, len(werte))))
        bericht["dauerlauf_haenger"] = zaehler
        if zaehler["1.0s"]:
            befund("fehler",
                   "%d Aufrufe brauchten ueber 1 s, obwohl der Verkehr "
                   "getaktet war. Das reisst jeden Servo-Strom, unabhaengig "
                   "von der mittleren Latenz -- stehende Verbindung bzw. "
                   "Zeitlimit um _call() werden damit vorrangig."
                   % zaehler["1.0s"])
        elif zaehler["0.2s"]:
            befund("warnung",
                   "%d Aufrufe ueber 200 ms. Keine Sekunden-Haenger, aber "
                   "Ausreisser, die einzelne Takte reissen." % zaehler["0.2s"])
        else:
            befund("ok",
                   "Keine Haenger bei getaktetem Verkehr -- die 1000-ms-Werte "
                   "aus Abschnitt 1 sind ein Artefakt des Messbursts.")

    BERICHTE.mkdir(exist_ok=True)
    ziel = BERICHTE / ("%s_RPC-Latenz.json" % datetime.now().strftime("%Y-%m-%d_%H-%M"))
    ziel.write_text(json.dumps(bericht, indent=2, ensure_ascii=False), encoding="utf-8")
    print("\nBericht: %s" % ziel)
    print("Geltungsbereich: %s" % bericht["geltungsbereich"])


if __name__ == "__main__":
    main()

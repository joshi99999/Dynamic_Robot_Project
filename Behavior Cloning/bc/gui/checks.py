"""Katalog der Systemcheck-Eintraege und Auswertung ihrer Ausgabe -- Qt-frei.

Der Modus "Systemcheck" startet die vorhandenen Skripte aus ``tests/`` und
``tools/`` als Unterprozesse, statt sie zu importieren. Gruende:

* Es bleibt EIN Weg, Pruefungen auszufuehren -- Terminal und GUI rufen
  dieselbe Zeile auf, und die Abnahmeliste bleibt damit gueltig.
* Ein haengender Controller-Aufruf blockiert die GUI nicht.
* Die Ausgabe ist ohnehin schon fuer Menschen gebaut (``section()``).

Die Eintraege sind nach Eingriffstiefe gestaffelt, weil genau das am
Labortag die Frage ist: laeuft hier etwas, das den Roboter bewegt?

    FREE     hardwarefrei, kein Controller, keine Kamera
    CONNECT  liest am Controller oder an einer Kamera, bewegt NICHT
    MOTION   bewegt den Roboter, bestromt ihn oder schaltet den Greifer

MOTION-Eintraege werden in der GUI nur nach ausdruecklicher Bestaetigung
gestartet. Das ist eine Bedienerfuehrung und ersetzt keinen zertifizierten
Hardware-Not-Aus (AP 4.2).

**Bewertung (2026-10-04, Wunsch Anwender: "Rueckmeldung, ob es passt").**
Die Werkzeuge schreiben ihr Urteil in einer gemeinsamen Form, die hier
ausgelesen wird::

    [OK] Text        [WARNUNG] Text  (auch WARN)
    [FEHLER] Text    (auch FAIL)     [INFO] Text

Das Urteil faellt im Werkzeug, nicht hier -- dort steht das Wissen, was
"zu langsam" heisst, und im Terminal gilt es genauso.
"""

import re
from pathlib import Path

#: Eingriffstiefe (siehe Moduldoku).
FREE = "free"
CONNECT = "connect"
MOTION = "motion"

LEVEL_LABELS = {
    FREE: "hardwarefrei",
    CONNECT: "Hardware, ohne Bewegung",
    MOTION: "mit Bewegung",
}

#: Ordner "Behavior Cloning" -- Arbeitsverzeichnis der Unterprozesse.
WORKDIR = Path(__file__).resolve().parents[2]

#: Wo Aufnahmen liegen (Ablage laut Anlagentag-Checkliste).
DATA_ROOTS = ("data_anlage", "data_vm", "data_sim")


class Param(object):
    """Ein einstellbarer Parameter eines Pruefeintrags.

    ``kind``: "text" | "int" | "float" | "choice" | "bool".
    ``flag``: Kommandozeilenschalter; None = positionales Argument. Endet er
    auf "=", wird der Wert angehaengt (``--camera=daheng``).
    ``default`` darf eine Funktion sein (z. B. neuester Aufnahmeordner).
    Leerer Text / nicht angehakt = weggelassen -- dann gilt der Default des
    Werkzeugs.
    """

    def __init__(self, key, label, default="", kind="text", flag=None,
                 choices=(), help="", minimum=None, maximum=None):
        self.key = key
        self.label = label
        self._default = default
        self.kind = kind
        self.flag = flag
        self.choices = tuple(choices)
        self.help = help
        self.minimum = minimum
        self.maximum = maximum

    @property
    def default(self):
        return self._default() if callable(self._default) else self._default

    def to_argv(self, value):
        if self.kind == "bool":
            return [self.flag] if value else []
        if value is None or str(value).strip() == "":
            return []
        text = ("%g" % value) if isinstance(value, float) else str(value).strip()
        if self.flag is None:
            return [text]
        if self.flag.endswith("="):
            return [self.flag + text]
        return [self.flag, text]


class Check(object):
    """Ein Eintrag der Pruefliste.

    ``argv`` ist die Kommandozeile OHNE das Python-Programm; das setzt
    ProcessRunner ein, damit die GUI dieselbe Umgebung benutzt, aus der
    sie gestartet wurde.
    """

    def __init__(self, key, label, level, argv, description,
                 needs=(), confirm=None, stdin=None, params=(),
                 real_flag=False, real_stdin=None):
        self.key = key
        self.label = label
        self.level = level
        self.argv = list(argv)
        self.description = description
        #: Schluessel aus requirements.py, ohne die der Eintrag nicht laeuft.
        self.needs = tuple(needs)
        #: Text, der vor dem Start bestaetigt werden muss (None = ohne Rueckfrage).
        self.confirm = confirm
        #: Was dem Unterprozess auf stdin geschickt wird (z. B. "j" auf eine Rueckfrage).
        self.stdin = stdin
        self.params = tuple(params)
        #: Kennt das Werkzeug ``--real-robot``? Dann haengt die GUI den
        #: Schalter an, wenn "reale Anlage" angehakt ist.
        self.real_flag = real_flag
        #: stdin an der Anlage (None = Freigabewort ANLAGE).
        self.real_stdin = real_stdin

    def defaults(self):
        return dict((p.key, p.default) for p in self.params)

    def build_argv(self, values=None, real=False):
        """Kommandozeile mit eingestellten Parametern."""
        values = dict(self.defaults(), **(values or {}))
        argv = list(self.argv)
        for param in self.params:
            argv += param.to_argv(values.get(param.key))
        if real and self.real_flag:
            argv.append("--real-robot")
        return argv

    def stdin_for(self, real=False):
        if real and self.real_flag:
            return self.real_stdin if self.real_stdin is not None else REAL_ROBOT_KEYWORD
        return self.stdin

    def __repr__(self):
        return "<Check %s %s>" % (self.key, self.level)


#: Freigabewort, das die Werkzeuge bei --real-robot auf stdin erwarten.
REAL_ROBOT_KEYWORD = "ANLAGE\n"

#: Bestaetigungstext fuer alles, was den realen Roboter freigibt.
REAL_ROBOT_CONFIRM = (
    "Dieser Punkt gibt die REALE Anlage frei -- nicht die Simulation.\n\n"
    "Arbeitsraum frei? Not-Aus in Reichweite? Greifer frei?"
)

MOTION_CONFIRM = (
    "Dieser Punkt BEWEGT den Roboter.\n\n"
    "Arbeitsraum frei? Not-Aus in Reichweite?"
)

POWER_CONFIRM = (
    "Dieser Punkt BESTROMT den Roboter und aktiviert das Servo-Interface. "
    "Gesendet wird nur 'hier stehenbleiben' -- der Arm soll sich nicht "
    "bewegen.\n\nHand an der Freigabe, Not-Aus in Reichweite?"
)

GRIPPER_CONFIRM = (
    "Dieser Punkt SCHALTET DEN GREIFER (der Arm bewegt sich nicht).\n\n"
    "Greiferbereich frei?"
)


def latest_recording():
    """Neuester Aufnahmeordner (``data_*/<Datum>``), relativ zum Arbeitsordner.

    Vorbelegung fuer die Auswertungspunkte: nach einer Probefahrt will man
    genau die ansehen.
    """
    candidates = []
    for root in DATA_ROOTS:
        base = WORKDIR / root
        if base.is_dir():
            candidates += [p.parent for p in base.rglob("index.json")]
    if not candidates:
        return ""
    newest = max(candidates, key=lambda p: (p / "index.json").stat().st_mtime)
    try:
        return newest.relative_to(WORKDIR).as_posix()
    except ValueError:
        return str(newest)


def catalogue():
    """Alle Pruefeintraege in Anzeigereihenfolge.

    Neue Pruefungen hier eintragen -- die Oberflaeche baut sich daraus auf.
    Reihenfolge innerhalb einer Stufe = empfohlene Reihenfolge am Labortag
    (Dokumentation/Anlagentag-Checkliste.md).
    """
    return [
        # -- hardwarefrei --------------------------------------------------
        Check("unittests", "Unittests (komplett)", FREE,
              ["tests/run_all.py"],
              "Alle Testmodule ohne Hardware. Das ist der Punkt, der vor "
              "jeder Sitzung laufen sollte. Bekannt: test_rectify scheitert "
              "mit OpenCV 4.13 (ArUco bei 240x320)."),
        Check("layering", "Schichtentrennung", FREE,
              ["tests/run_all.py", "test_layering"],
              "Prueft, dass die Pipeline weder neurapy noch gxipy importiert."),
        Check("cameras_sim", "Kamerakette (Platzhalterbilder)", FREE,
              ["tools/check_cameras.py", "--backend", "sim", "--no-display",
               "--seconds", "5"],
              "Prueft die Kamerakette ohne Hardware: Oeffnen, Abgriff, Rate.",
              needs=("opencv",)),
        Check("episodes", "Aufnahmen bewerten", FREE,
              ["tools/check_episodes.py"],
              "Bewertet die letzten Aufnahmen: Hat der Rechner den Takt "
              "gehalten? Folgt der Arm der Bahn oder laeuft er hinterher "
              "(kleiner Override)? Faehrt er glatt oder im Stop-and-go? "
              "Nach jeder Probefahrt der erste Blick.",
              params=(
                  Param("datasets", "Ordner", latest_recording, flag=None,
                        help="Aufnahmeordner oder Muster; leer = alle data_*-Ordner"),
                  Param("last", "Anzahl", 3, kind="int", flag="--last",
                        minimum=1, maximum=50,
                        help="die N neuesten Episoden"),
              )),
        Check("plot", "Episodenbericht (PDF)", FREE,
              ["tools/plot_episode.py"],
              "TCP-Bahn, Geschwindigkeit, Beschleunigung, Folgefehler und alle "
              "Gelenke je Episode als PDF in Berichte/ (Edge headless). Das "
              "Bild zu 'Aufnahmen bewerten'.",
              params=(
                  Param("dataset", "Ordner", latest_recording, flag=None,
                        help="Aufnahmeordner (data_vm/<Datum>)"),
                  Param("html", "zusaetzlich HTML", False, kind="bool", flag="--html"),
              )),
        Check("rate_budget", "Taktbudget rechnen", FREE,
              ["tools/check_rate_budget.py"],
              "Rechnet aus einer servo_j-Messung (Punkt 'servo_j-Timing'), "
              "welche servo_j-Rate in den 15-Hz-Takt passt. Ohne Angabe gilt "
              "der neueste Bericht in Berichte/.",
              params=(
                  Param("bericht", "Bericht", "", flag="--bericht",
                        help="Servo-Timing-JSON; leer = neuester"),
                  Param("p95", "gegen p95 rechnen", False, kind="bool", flag="--p95",
                        help="strenger: haelt der Takt auch im schlechten Fall?"),
              )),
        Check("gpu", "GPU / CUDA", FREE,
              ["tools/check_gpu.py"],
              "Treiber, PyTorch-Build und ein kurzer Durchsatztest. "
              "Auf dem Aufnahme-Laptop erwartungsgemaess ohne CUDA.",
              needs=("torch",)),
        Check("daheng_api", "Daheng-API (ohne Kamera)", FREE,
              ["tools/check_daheng_api.py"],
              "Vergleicht die im Adapter benutzten gxipy-Namen mit der "
              "installierten gxipy. Oeffnet keine Kamera."),

        # -- Hardware, ohne Bewegung ---------------------------------------
        Check("controller_read", "Steuerung pruefen (nur lesen)", CONNECT,
              ["tools/check_sim_robot.py"],
              "Host, Port, NeuraPy-Version, is_robot_in_simulation(), "
              "Servo-Interface und Greifermodus. Ohne Bewegung wird nur "
              "gelesen und gerechnet.",
              needs=("neurapy",)),
        Check("kinematics", "Kinematik und Achsgrenzen", CONNECT,
              ["tools/check_kinematics.py"],
              "Controller-FK gegen die VM-Referenz (50 Stellungen), "
              "IK-Rueckprobe, TCP-Pose gegen FK, Achsgrenzen. Rechnet nur "
              "in der Steuerung, bewegt nichts.",
              needs=("neurapy",)),
        Check("ik", "IK-Absicherungen", CONNECT,
              ["tools/check_ik.py"],
              "Die Absicherungen aus AP 2.4 (Tool-Offset, Spruenge, "
              "Repraesentationen) -- Berechnungen in der Steuerung, keine "
              "Bewegung.",
              needs=("neurapy",)),
        Check("rpc_latency", "RPC-Latenz messen", CONNECT,
              ["tools/check_rpc_latency.py"],
              "Was kostet ein NeuraPy-Aufruf? Trennt TCP-Aufbau, Serverzeit "
              "und Client-Overhead und sucht Haenger im Dauerlauf. Sendet "
              "kein servo_j. Bericht nach Berichte/.",
              needs=("neurapy",),
              params=(
                  Param("wiederholungen", "Aufrufe je Reihe", 30, kind="int",
                        flag="--wiederholungen", minimum=5, maximum=1000),
                  Param("dauerlauf", "Dauerlauf (s)", 10.0, kind="float",
                        flag="--dauerlauf", minimum=0.0, maximum=300.0),
              )),
        Check("trajectory_read", "Pufferbefehle pruefen (nur lesen)", CONNECT,
              ["tools/check_trajectory_buffer.py"],
              "Bietet die Steuerung move_trajectory und die verwandten "
              "Pufferbefehle an, und was sagt get_doc() dazu? Erste Frage fuer "
              "den rollenden Puffer gegen das Ruckeln (Befund 2026-10-05). "
              "Bewegt und bestromt nichts.",
              needs=("neurapy",),
              params=(
                  Param("doku", "Doku aller Kandidaten", False, kind="bool", flag="--doku",
                        help="get_doc() auch fuer movelinear_online, executor usw."),
              )),
        Check("export_points", "Punkte sichern", CONNECT,
              ["tools/export_points.py"],
              "Liest die geteachten Punkte aus der Steuerung und sichert sie "
              "nach sequences/ -- Beleg, gegen welche Punkte aufgezeichnet "
              "wurde. Aendert nichts in der Steuerung.",
              needs=("neurapy",),
              params=(
                  Param("sequence", "Ablaufdatei", "sequences/pick_to_station.json",
                        flag="--sequence", help="nur die Punkte dieser Ablaufdatei; leer = alle"),
              )),
        Check("robot_contract", "Roboter-Contract", CONNECT,
              ["tests/run_all.py", "--robot=neura", "contract.test_robot_contract"],
              "Die Abnahmesuite des Adapters (12 Tests). Sendet servo_j nur "
              "mit der Ist-Stellung, faehrt also nicht -- der Greifer "
              "schaltet aber.",
              needs=("neurapy",), confirm=GRIPPER_CONFIRM, real_flag=True),
        Check("cameras_list", "Kameras suchen", CONNECT,
              ["tools/check_cameras.py", "--list"],
              "Welche Geraete sind angeschlossen? Oeffnet nichts.",
              needs=("opencv",)),
        Check("daheng_open", "Daheng am Geraet pruefen", CONNECT,
              ["tools/check_daheng_api.py", "--open"],
              "Oeffnet die Daheng und prueft jedes Feature: vorhanden, "
              "schreibbar, aktueller Wert. Nur lesen. Bekannt: Binning ist "
              "an der VEN-161 nicht schreibbar.",
              needs=("gxipy",),
              params=(
                  Param("serial", "Seriennummer", "", flag="--serial",
                        help="leer = erste gefundene Kamera"),
              )),
        Check("cameras_rate", "Kameras: Rate und Versatz", CONNECT,
              ["tools/check_cameras.py", "--no-display"],
              "Oeffnet die Kameras und misst Bildrate, Bildalter und den "
              "Versatz zwischen beiden -- gegen den 15-Hz-Takt und das "
              "30-ms-Budget. Ohne Auswahl die Kameras aus config.py.",
              needs=("opencv",),
              params=(
                  Param("only", "nur Kamera", "", kind="choice", flag="--only",
                        choices=("", "wrist", "scene")),
                  Param("backend", "Backend", "", kind="choice", flag="--backend",
                        choices=("", "uvc", "sim"),
                        help="leer = wie in config.py; uvc = Webcam als Wrist-Ersatz"),
                  Param("device", "Geraet", "", flag="--device",
                        help="UVC-Index oder Daheng-Seriennummer (nur mit 'nur Kamera')"),
                  Param("seconds", "Messdauer (s)", 10.0, kind="float", flag="--seconds",
                        minimum=3.0, maximum=300.0),
              )),
        Check("camera_contract", "Kamera-Contract", CONNECT,
              ["tests/run_all.py", "contract.test_camera_contract"],
              "Die Abnahmesuite des Kamera-Adapters gegen das echte Geraet.",
              needs=("opencv",),
              params=(
                  Param("camera", "Kamera", "daheng", kind="choice", flag="--camera=",
                        choices=("daheng", "uvc")),
                  Param("uvc_device", "UVC-Index", "", flag="--uvc-device=",
                        help="nur bei uvc; leer = config.SCENE_CAMERA"),
              )),
        Check("rectify", "ArUco / Rektifizierung", CONNECT,
              ["tools/check_rectify.py"],
              "Tischmarker im Kamerabild: Woerterbuch und IDs, Erkennung im "
              "vollen Bild und in 240x320, mit Layout-Datei die Homographie. "
              "Bilder und Protokoll nach Berichte/.",
              needs=("opencv",),
              params=(
                  Param("camera", "Kamera", "scene", kind="choice", flag="--camera",
                        choices=("scene", "wrist")),
                  Param("dictionary", "Woerterbuch", "auto", flag="--dictionary"),
                  Param("layout", "Layout-Datei", "", flag="--layout",
                        help="Markerpositionen (JSON, siehe tools/check_rectify.py)"),
                  Param("x_range", "x-Bereich", "", flag="--x-range=",
                        help="Workspace x_min,x_max in m (mit Layout)"),
                  Param("y_range", "y-Bereich", "", flag="--y-range=",
                        help="Workspace y_min,y_max in m (mit Layout)"),
              )),

        # -- mit Bewegung ---------------------------------------------------
        Check("servo_timing", "servo_j-Timing messen", MOTION,
              ["tools/check_servo_timing.py"],
              "Was kostet servo_j, und welche Senderate haelt den 15-Hz-Takt? "
              "BESTROMT den Roboter, sendet aber nur 'hier stehenbleiben'. "
              "Ergebnis danach mit 'Taktbudget rechnen' auswerten. An der "
              "Anlage die wichtigste Messung vor der ersten Aufnahme.",
              needs=("neurapy",), confirm=POWER_CONFIRM, stdin="j\n",
              real_flag=True, real_stdin="j\n",
              params=(
                  Param("raten", "Raten (Hz)", "60,30,15", flag="--raten"),
                  Param("takte", "Takte je Rate", 30, kind="int", flag="--takte",
                        minimum=5, maximum=1000),
              )),
        Check("controller_move", "Nachlauf und Override", MOTION,
              ["tools/check_sim_robot.py", "--move"],
              "Servotest um die aktuelle Stellung je Override: Nachlauf in "
              "ms, Folgefehler, Takte ueber Budget. Grundlage fuer die "
              "Override-Festlegung. In der VM zusaetzlich move_linear.",
              needs=("neurapy",), confirm=MOTION_CONFIRM, real_flag=True,
              params=(
                  Param("rate", "servo_j-Rate (Hz)", 60.0, kind="float", flag="--rate",
                        minimum=15.0, maximum=250.0),
                  Param("overrides", "Overrides", "1.0,0.5", flag="--overrides",
                        help="Komma-Liste, je Override ein Servotest"),
                  Param("joint", "Gelenk", 1, kind="int", flag="--joint",
                        minimum=1, maximum=6),
                  Param("amplitude", "Auslenkung (rad)", "", flag="--amplitude",
                        help="leer = VM 0.15, Anlage 0.05"),
                  Param("gripper", "Greifer mittesten", False, kind="bool", flag="--gripper",
                        help="an der Anlage auch den Greifer schalten"),
              )),
        Check("trajectory_buffer", "Bahnpuffer testen", MOTION,
              ["tools/check_trajectory_buffer.py", "--move"],
              "Faehrt EIN Gelenk hin und zurueck: erst wie heute per servo_j, "
              "dann per move_trajectory (ganze Bahn, Abschnitte nacheinander, "
              "leerer Puffer, Stopp mitten in der Fahrt). Ein Lese-Thread "
              "schreibt die Gelenkwinkel mit ~250 Hz mit und misst Stop-and-go "
              "und Welligkeit. Fazit: traegt der rollende Puffer gegen das "
              "Ruckeln? Bericht nach Berichte/.",
              needs=("neurapy",), confirm=MOTION_CONFIRM, real_flag=True,
              params=(
                  Param("tests", "Tests", "referenz,komplett,knoten15,anschluss,leerlauf,stopp,online",
                        flag="--tests", help="Komma-Liste; Reihenfolge wie angegeben"),
                  Param("joint", "Gelenk", 1, kind="int", flag="--joint",
                        minimum=1, maximum=6),
                  Param("amplitude", "Auslenkung (rad)", "", flag="--amplitude",
                        help="leer = VM 0.15, Anlage 0.05 (Grenze 0.10 an der Anlage)"),
                  Param("dauer", "Dauer hin+zurueck (s)", 4.0, kind="float", flag="--dauer",
                        minimum=1.0, maximum=30.0),
                  Param("knoten_hz", "Stuetzpunkte (Hz)", 100.0, kind="float",
                        flag="--knoten-hz", minimum=5.0, maximum=1000.0),
                  Param("chunk_takte", "Abschnitt (Takte)", 4, kind="int",
                        flag="--chunk-takte", minimum=1, maximum=30,
                        help="Laenge eines Abschnitts im Anschlusstest, 1 Takt = 66,7 ms"),
                  Param("vorlauf_takte", "Vorlauf (Takte)", 1, kind="int",
                        flag="--vorlauf-takte", minimum=0, maximum=29,
                        help="so viele Takte vor Ende des vorigen Abschnitts senden"),
                  Param("vergleich_raten", "Vergleichsraten servo_j (Hz)", "90,120",
                        flag="--vergleich-raten",
                        help="nach der Referenz dieselbe Fahrt mit diesen Raten; leer = keine"),
                  Param("override", "Override", 1.0, kind="float", flag="--override",
                        minimum=0.05, maximum=1.0),
              )),
        Check("gripper", "Greifer-Totzeit messen", MOTION,
              ["tools/check_gripper.py"],
              "Schaltet den Greifer mehrfach und misst ueber die Wrist-Kamera, "
              "wann die Backen stehen -- gegen GRIPPER_DWELL_S (500 ms, "
              "unverifiziert). Erst ohne ROI mit gespeicherten Bildern, dann "
              "mit ROI auf die Backen. 'sim' prueft nur das Werkzeug.",
              needs=("neurapy", "opencv"), confirm=GRIPPER_CONFIRM, real_flag=True,
              params=(
                  Param("camera", "Kamera", "wrist", kind="choice", flag="--camera",
                        choices=("wrist", "sim")),
                  Param("cycles", "Zyklen", 5, kind="int", flag="--cycles",
                        minimum=1, maximum=50),
                  Param("roi", "ROI x,y,w,h", "", flag="--roi",
                        help="Pixel im Kamerabild, auf die Backen legen"),
                  Param("save_frames", "Bilder nach", "Berichte/greifer_frames",
                        flag="--save-frames", help="leer = keine Bilder speichern"),
              )),
    ]


def by_level():
    """Katalog nach Eingriffstiefe gruppiert, Reihenfolge FREE, CONNECT, MOTION."""
    groups = {FREE: [], CONNECT: [], MOTION: []}
    for check in catalogue():
        groups[check.level].append(check)
    return [(level, groups[level]) for level in (FREE, CONNECT, MOTION)]


def find(key):
    for check in catalogue():
        if check.key == key:
            return check
    raise KeyError("Unbekannte Pruefung '%s'" % key)


# -- Auswertung der Ausgabe ------------------------------------------------

#: Schlusszeile von tests/run_all.py, z. B. "123 Tests bestanden, 0 fehlgeschlagen".
_SUMMARY = re.compile(r"(\d+)\s+Tests bestanden,\s+(\d+)\s+fehlgeschlagen")
#: Einzelne Fehlschlaege, z. B. "  FAIL test_noise::test_ou_is_stationary".
_FAIL = re.compile(r"^\s*FAIL\s+(\S+)", re.MULTILINE)
#: Zeilen, die auf einen Befund hindeuten, wenn es keine Schlusszeile gibt.
_PROBLEM = re.compile(r"(?im)^\s*(?:FEHLER|FAIL|ERROR|Traceback)\b")
#: Urteilszeilen der Werkzeuge (Moduldoku): "[OK] Text" und, aelter,
#: "OK: Text" / "FEHLER: Text" (check_ik, check_gpu).
_VERDICT = re.compile(
    r"^\s*(?:\[(OK|WARNUNG|WARN|FEHLER|FAIL|INFO|HINWEIS)\]|(OK|FEHLER|WARNUNG):)\s+(.+?)\s*$",
    re.MULTILINE | re.IGNORECASE)
_LEVEL = {"ok": "ok", "warnung": "warnung", "warn": "warnung", "fehler": "fehler",
          "fail": "fehler", "info": "info", "hinweis": "info"}

#: Reihenfolge der Schwere fuer Zusammenfassung und Farbe.
SEVERITY = ("ok", "info", "warnung", "fehler")
VERDICT_LABELS = {"ok": "ok", "info": "Hinweis", "warnung": "Warnung", "fehler": "Fehler"}
_PLURAL = {"ok": "ok", "info": "Hinweise", "warnung": "Warnungen", "fehler": "Fehler"}


def parse_verdicts(text):
    """Alle Urteilszeilen als Liste (Stufe, Text) in Ausgabereihenfolge."""
    out = []
    for match in _VERDICT.finditer(text):
        word = (match.group(1) or match.group(2)).lower()
        out.append((_LEVEL[word], match.group(3)))
    return out


class Outcome(object):
    """Ausgewertetes Ergebnis eines Pruefeintrags."""

    def __init__(self, check, returncode, text):
        self.check = check
        self.returncode = returncode
        self.text = text
        self.passed, self.failed = _counts(text)
        self.failures = _FAIL.findall(text)
        self.verdicts = parse_verdicts(text)

    @property
    def ok(self):
        return self.returncode == 0

    def counts(self):
        out = dict((level, 0) for level in SEVERITY)
        for level, _text in self.verdicts:
            out[level] += 1
        return out

    @property
    def state(self):
        """'bestanden' | 'Befunde' | 'fehlgeschlagen' | 'abgebrochen' --
        fuer Farbe und Gesamturteil. Ein [FEHLER] zaehlt auch bei
        Rueckgabewert 0: manche Werkzeuge melden ihr Urteil nur als Text."""
        if self.returncode is None:
            return "abgebrochen"
        counts = self.counts()
        if not self.ok or counts["fehler"]:
            return "fehlgeschlagen"
        if counts["warnung"]:
            return "Befunde"
        return "bestanden"

    def summary(self):
        """Eine Zeile fuer die Liste -- das, was man im Vorbeigehen liest."""
        if self.passed is not None:
            state = "bestanden" if self.ok else "fehlgeschlagen"
            return "%s -- %d Tests bestanden, %d fehlgeschlagen" % (
                state, self.passed, self.failed)
        if self.returncode is None:
            return "abgebrochen"
        counts = self.counts()
        if any(counts.values()):
            parts = ["%d %s" % (counts[level], (VERDICT_LABELS if counts[level] == 1
                                                else _PLURAL)[level])
                     for level in ("fehler", "warnung", "ok") if counts[level]]
            head = "durchgelaufen" if self.ok else (
                "fehlgeschlagen (Rueckgabewert %d)" % self.returncode)
            return "%s -- %s" % (head, ", ".join(parts))
        if self.ok:
            hint = " (Befunde in der Ausgabe)" if _PROBLEM.search(self.text) else ""
            return "durchgelaufen" + hint
        return "fehlgeschlagen (Rueckgabewert %d)" % self.returncode


def _counts(text):
    match = None
    for match in _SUMMARY.finditer(text):       # letzte Schlusszeile zaehlt
        pass
    if match is None:
        return None, 0
    return int(match.group(1)), int(match.group(2))


def protocol_text(outcomes, log_text, stamp):
    """Protokoll einer Pruefsitzung als Text -- fuer Berichte/.

    ``outcomes``: Outcome-Objekte in Ausfuehrungsreihenfolge.
    """
    lines = ["Systemcheck %s" % stamp, "=" * 60, ""]
    for outcome in outcomes:
        lines.append("%s: %s" % (outcome.check.label, outcome.summary()))
        for level, text in outcome.verdicts:
            if level != "ok":
                lines.append("    [%s] %s" % (level.upper(), text))
    lines += ["", "=" * 60, "Vollstaendige Ausgabe", "=" * 60, "", log_text]
    return "\n".join(lines)

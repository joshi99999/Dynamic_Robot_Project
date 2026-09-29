"""Katalog der Systemcheck-Eintraege und Auswertung ihrer Ausgabe -- Qt-frei.

Der Modus "Systemcheck" startet die vorhandenen Skripte aus ``tests/`` und
``tools/`` als Unterprozesse, statt sie zu importieren. Gruende:

* Es bleibt EIN Weg, Pruefungen auszufuehren -- Terminal und GUI rufen
  dieselbe Zeile auf, und die Abnahmeliste bleibt damit gueltig.
* Ein haengender Controller-Aufruf blockiert die GUI nicht.
* Die Ausgabe ist ohnehin schon fuer Menschen gebaut (``section()``).

Die Eintraege sind nach Eingriffstiefe gestaffelt, weil genau das am
Labortag die Frage ist: laeuft hier etwas, das den Roboter bewegt?

    FREE     hardwarefrei, kein Controller
    CONNECT  liest am Controller, bewegt NICHT
    MOTION   bewegt den Roboter oder schaltet den Greifer

MOTION-Eintraege werden in der GUI nur nach ausdruecklicher Bestaetigung
gestartet. Das ist eine Bedienerfuehrung und ersetzt keinen zertifizierten
Hardware-Not-Aus (AP 4.2).
"""

import re

#: Eingriffstiefe (siehe Moduldoku).
FREE = "free"
CONNECT = "connect"
MOTION = "motion"

LEVEL_LABELS = {
    FREE: "hardwarefrei",
    CONNECT: "am Controller, ohne Bewegung",
    MOTION: "mit Bewegung",
}


class Check(object):
    """Ein Eintrag der Pruefliste.

    ``argv`` ist die Kommandozeile OHNE das Python-Programm; das setzt
    ProcessRunner ein, damit die GUI dieselbe Umgebung benutzt, aus der
    sie gestartet wurde.
    """

    def __init__(self, key, label, level, argv, description,
                 needs=(), confirm=None, stdin=None):
        self.key = key
        self.label = label
        self.level = level
        self.argv = list(argv)
        self.description = description
        #: Schluessel aus requirements.py, ohne die der Eintrag nicht laeuft.
        self.needs = tuple(needs)
        #: Text, der vor dem Start bestaetigt werden muss (None = ohne Rueckfrage).
        self.confirm = confirm
        #: Was dem Unterprozess auf stdin geschickt wird (z. B. Freigabewort).
        self.stdin = stdin

    def __repr__(self):
        return "<Check %s %s>" % (self.key, self.level)


#: Bestaetigungstext fuer alles, was den realen Roboter freigibt.
REAL_ROBOT_CONFIRM = (
    "Dieser Punkt gibt die REALE Anlage frei -- nicht die Simulation.\n\n"
    "Arbeitsraum frei? Not-Aus in Reichweite? Greifer frei?"
)

MOTION_CONFIRM = (
    "Dieser Punkt BEWEGT den Roboter.\n\n"
    "Arbeitsraum frei? Not-Aus in Reichweite?"
)


def catalogue():
    """Alle Pruefeintraege in Anzeigereihenfolge.

    Neue Pruefungen hier eintragen -- die Oberflaeche baut sich daraus auf.
    """
    return [
        # -- hardwarefrei --------------------------------------------------
        Check("unittests", "Unittests (komplett)", FREE,
              ["tests/run_all.py"],
              "Alle Testmodule ohne Hardware. Das ist der Punkt, der vor "
              "jeder Sitzung laufen sollte."),
        Check("layering", "Schichtentrennung", FREE,
              ["tests/run_all.py", "test_layering"],
              "Prueft, dass die Pipeline weder neurapy noch gxipy importiert."),
        Check("kinematics", "Kinematik gegen Referenz", FREE,
              ["tools/check_kinematics.py"],
              "IK/FK gegen die abgelegten Referenzwerte. Bewegt nichts."),
        Check("ik", "IK-Absicherungen", FREE,
              ["tools/check_ik.py"],
              "Die vier Absicherungen aus AP 2.4, rein rechnerisch."),
        Check("selftest", "Selbsttest", FREE,
              ["tools/selftest.py"],
              "Kurzer Durchlauf der Pipeline ohne Geraete."),
        Check("gpu", "GPU / CUDA", FREE,
              ["tools/check_gpu.py"],
              "Treiber, PyTorch-Build und ein kurzer Durchsatztest. "
              "Auf dem Aufnahme-Laptop erwartungsgemaess ohne CUDA.",
              needs=("torch",)),
        Check("daheng_api", "Daheng-API (ohne Kamera)", FREE,
              ["tools/check_daheng_api.py"],
              "Vergleicht die im Adapter benutzten gxipy-Namen mit der "
              "installierten gxipy. Oeffnet keine Kamera."),

        # -- am Controller, ohne Bewegung ----------------------------------
        Check("robot_contract", "Roboter-Contract (Simulation)", CONNECT,
              ["tests/run_all.py", "--robot=neura", "contract.test_robot_contract"],
              "Die Abnahmesuite des Adapters gegen die virtuelle Steuerung. "
              "Sendet servo_j nur mit der Ist-Stellung, faehrt also nicht -- "
              "der Greifer schaltet aber.",
              needs=("neurapy",)),
        Check("controller_read", "Steuerung pruefen (nur lesen)", CONNECT,
              ["tools/check_sim_robot.py"],
              "Host, Port, NeuraPy-Version, Servo-Interface und Greifermodus. "
              "Ohne --move wird nur gelesen und gerechnet.",
              needs=("neurapy",)),
        Check("cameras_list", "Kameras auflisten", CONNECT,
              ["tools/check_cameras.py", "--list"],
              "Welche Geraete sind angeschlossen? Oeffnet nichts.",
              needs=("opencv",)),
        Check("cameras_sim", "Kamerakette (Platzhalterbilder)", CONNECT,
              ["tools/check_cameras.py", "--backend", "sim"],
              "Prueft die Kamerakette ohne Hardware.",
              needs=("opencv",)),

        # -- mit Bewegung ---------------------------------------------------
        Check("controller_move", "Steuerung pruefen (mit Bewegung)", MOTION,
              ["tools/check_sim_robot.py", "--move"],
              "Servotest und move_linear an der virtuellen Steuerung.",
              needs=("neurapy",), confirm=MOTION_CONFIRM),
        Check("gripper", "Greifer vermessen", MOTION,
              ["tools/check_gripper.py"],
              "Misst die Greifer-Totzeit ueber das Kamerabild. Der Greifer "
              "schaltet dabei mehrfach.",
              needs=("neurapy", "opencv"), confirm=MOTION_CONFIRM),
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


class Outcome(object):
    """Ausgewertetes Ergebnis eines Pruefeintrags."""

    def __init__(self, check, returncode, text):
        self.check = check
        self.returncode = returncode
        self.text = text
        self.passed, self.failed = _counts(text)
        self.failures = _FAIL.findall(text)

    @property
    def ok(self):
        return self.returncode == 0

    def summary(self):
        """Eine Zeile fuer die Liste -- das, was man im Vorbeigehen liest."""
        if self.passed is not None:
            state = "bestanden" if self.ok else "fehlgeschlagen"
            return "%s -- %d Tests bestanden, %d fehlgeschlagen" % (
                state, self.passed, self.failed)
        if self.returncode is None:
            return "abgebrochen"
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

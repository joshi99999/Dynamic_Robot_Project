"""Voraussetzungen je Modus pruefen -- Qt-frei, damit testbar (GUI, AP 5.x).

Festlegung des Anwenders (2026-09-29): Fehlt etwas, verschwindet die
Schaltflaeche NICHT. Sie bleibt sichtbar und klickbar, und der Bedienende
bekommt eine konkrete Meldung, was fehlt und was trotzdem geht. Der
Aufnahme-Laptop hat z. B. kein CUDA -- Aufnahme und Export gehen dort,
die Policy-Fahrt nicht (CPU 384 ms je Vorhersage, gemessen 2026-09-23).

Jede Pruefung ist ein Requirement mit einer Sonde, die (erfuellt, Detail)
liefert. Sonden importieren erst beim Pruefen, damit diese Datei ohne
installierte SDKs importierbar bleibt, und fangen jeden Fehler ab -- eine
fehlende Abhaengigkeit ist ein Befund, kein Absturz.
"""

import importlib
import os
import shutil
import socket
import sys
from pathlib import Path

#: Schweregrad einer Voraussetzung.
#:   REQUIRED  ohne das geht der Modus gar nicht
#:   OPTIONAL  ohne das geht ein Teil des Modus nicht
REQUIRED = "required"
OPTIONAL = "optional"


class Requirement(object):
    """Eine pruefbare Voraussetzung mit Klartext-Begruendung."""

    def __init__(self, key, label, severity, probe, missing_hint):
        self.key = key
        self.label = label
        self.severity = severity
        self._probe = probe
        #: Was zu tun ist, wenn es fehlt -- erscheint in der Meldung beim Klick.
        self.missing_hint = missing_hint

    def check(self):
        try:
            ok, detail = self._probe()
        except Exception as exc:                      # Befund, kein Absturz
            return Result(self, False, "%s: %s" % (type(exc).__name__, exc))
        return Result(self, bool(ok), detail)


class Result(object):
    def __init__(self, requirement, ok, detail=""):
        self.requirement = requirement
        self.ok = ok
        self.detail = detail

    @property
    def key(self):
        return self.requirement.key

    @property
    def label(self):
        return self.requirement.label

    @property
    def severity(self):
        return self.requirement.severity

    def __repr__(self):
        return "<Result %s ok=%s %r>" % (self.key, self.ok, self.detail)


class Report(object):
    """Ergebnis aller Voraussetzungen eines Modus."""

    def __init__(self, results):
        self.results = list(results)

    def __iter__(self):
        return iter(self.results)

    @property
    def missing_required(self):
        return [r for r in self.results if not r.ok and r.severity == REQUIRED]

    @property
    def missing_optional(self):
        return [r for r in self.results if not r.ok and r.severity == OPTIONAL]

    @property
    def ok(self):
        return not self.missing_required

    def message(self, action, also_possible=None):
        """Meldungstext fuer einen Klick, bei dem etwas fehlt.

        ``action`` ist die angeklickte Handlung ("Training starten"),
        ``also_possible`` nennt, was trotz des fehlenden Teils geht.
        Liefert None, wenn nichts fehlt -- dann einfach ausfuehren.
        """
        missing = self.missing_required + self.missing_optional
        if not missing:
            return None
        lines = ["%s braucht:" % action, ""]
        for result in missing:
            lines.append("  - " + result.requirement.missing_hint)
            if result.detail:
                lines.append("    (%s)" % result.detail)
        if also_possible:
            lines.append("")
            lines.append(also_possible)
        return "\n".join(lines)


# -- Sonden ---------------------------------------------------------------

def module_probe(name, version_attr="__version__"):
    """Sonde: Modul importierbar? Liefert die Version als Detailtext."""

    def probe():
        try:
            module = importlib.import_module(name)
        except Exception as exc:
            return False, "%s: %s" % (type(exc).__name__, exc)
        return True, "%s %s" % (name, getattr(module, version_attr, "?"))

    return probe


def cuda_probe():
    def probe():
        try:
            import torch
        except Exception as exc:
            return False, "torch fehlt (%s)" % type(exc).__name__
        if not torch.cuda.is_available():
            return False, "torch %s ohne CUDA (CPU-Betrieb)" % torch.__version__
        return True, "%s (torch %s)" % (torch.cuda.get_device_name(0), torch.__version__)

    return probe


def interpreter_tool_dirs():
    """Ordner neben dem Interpreter, in denen mitgelieferte Programme liegen.

    In der Projektumgebung (``E:\\Enviroments\\bc_env``) liegt ``ffmpeg.exe``
    unter ``Library\\bin`` -- im PATH steht es aber nur, wenn die Umgebung
    aktiviert wurde. Wird die GUI direkt ueber ``python.exe -m bc.gui``
    gestartet, ist sie das nicht. ``shutil.which`` allein wuerde dann
    "fehlt" melden, obwohl das Programm daliegt; die Unterprozesse haetten
    dasselbe Problem (siehe runner.ProcessRunner).
    """
    base = Path(sys.executable).resolve().parent
    return [d for d in (base, base / "Library" / "bin", base / "Scripts",
                        base / "bin") if d.is_dir()]


def find_tool(name):
    """Programm im PATH oder neben dem Interpreter suchen. Liefert den Pfad oder None."""
    found = shutil.which(name)
    if found:
        return found
    extra = os.pathsep.join(str(d) for d in interpreter_tool_dirs())
    return shutil.which(name, path=extra) if extra else None


def ffmpeg_probe():
    def probe():
        path = find_tool("ffmpeg")
        if not path:
            return False, "weder im PATH noch neben dem Interpreter"
        if not shutil.which("ffmpeg"):
            # Da, aber nur weil wir daneben nachgesehen haben. Die
            # Unterprozesse bekommen den Ordner ueber ProcessRunner dazu.
            return True, "%s (nicht im PATH -- wird den Unterprozessen ergaenzt)" % path
        return True, path

    return probe


def controller_probe(host, port=65432, timeout=1.0):
    """Sonde: liegt auf host:port etwas? Verbindet NICHT per NeuraPy, bewegt nichts."""

    def probe():
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            sock.connect((host, port))
        except OSError as exc:
            return False, "%s:%d nicht erreichbar (%s)" % (host, port, exc.strerror or exc)
        finally:
            sock.close()
        return True, "%s:%d antwortet" % (host, port)

    return probe


# -- Zusammenstellungen je Modus ------------------------------------------

_NEURAPY_HINT = ("neurapy aus dem Ordner \"Robot Controller\" -- unter Windows "
                 "braucht es zusaetzlich pywin32 und prettytable")


def _req(key, label, severity, probe, hint):
    return Requirement(key, label, severity, probe, hint)


def systemcheck_requirements():
    return [
        _req("numpy", "numpy", REQUIRED, module_probe("numpy"),
             "numpy -- Teil der Grundinstallation"),
        _req("opencv", "OpenCV", OPTIONAL, module_probe("cv2"),
             "OpenCV (cv2) fuer die Kamera- und Rektifizierungspruefungen"),
        _req("neurapy", "neurapy", OPTIONAL, module_probe("neurapy"),
             _NEURAPY_HINT + " -- ohne das laufen nur die hardwarefreien Pruefungen"),
        # torch gehoert hierher, weil der Punkt "GPU / CUDA" im Katalog es
        # braucht. Jeder Schluessel aus checks.Check.needs muss in diesem
        # Modus stehen, sonst wird der Punkt stumm uebersprungen
        # (abgesichert durch tests/test_gui.py).
        _req("torch", "PyTorch", OPTIONAL, module_probe("torch"),
             "PyTorch -- nur fuer den Punkt \"GPU / CUDA\"; die uebrigen "
             "Pruefungen laufen ohne"),
    ]


def recording_requirements():
    """Alles, was IRGENDEINE Aufnahme brauchen koennte -- nichts davon immer.

    Bis 2026-09-29 waren ``opencv`` und ``neurapy`` hier REQUIRED. Das war
    falsch: Eine Aufnahme gegen den SimRobot mit Platzhalterbildern braucht
    weder das eine noch das andere, bekam aber ein rotes Kreuz und eine
    Sperre. Was ein konkreter Lauf wirklich braucht, haengt am Backend und
    an der Kamerazuordnung -- der Reiter fragt die Schluessel deshalb erst
    beim Start ab (``gui.cameras.required_backend_keys``, ModeTab.
    ensure_requirements). Diese Liste ist nur noch die Anzeige: was ist da,
    was nicht.
    """
    return [
        _req("opencv", "OpenCV", OPTIONAL, module_probe("cv2"),
             "OpenCV (cv2) -- fuer jede echte Kamera; mit Platzhalterbildern "
             "nicht noetig"),
        _req("neurapy", "neurapy", OPTIONAL, module_probe("neurapy"),
             _NEURAPY_HINT + " -- nur fuer die Neura-Steuerung; gegen den "
             "SimRobot nicht noetig"),
        _req("gxipy", "Daheng SDK (gxipy)", OPTIONAL, module_probe("gxipy"),
             "gxipy (Galaxy SDK) -- nur, wenn einem Platz eine Daheng-Kamera "
             "zugewiesen ist; Platzhalter und Webcam gehen ohne"),
    ]


def training_requirements():
    return [
        _req("torch", "PyTorch", REQUIRED, module_probe("torch"),
             "PyTorch -- ohne das laesst sich kein Modell trainieren"),
        _req("cuda", "CUDA-Geraet", OPTIONAL, cuda_probe(),
             "eine CUDA-GPU -- auf der CPU dauert ein Training unverhaeltnismaessig lang"),
        _req("lerobot", "lerobot", REQUIRED, module_probe("lerobot"),
             "lerobot 0.6.1 (Datensatzformat v3.0) fuer Export und Training"),
        _req("ffmpeg", "FFmpeg", OPTIONAL, ffmpeg_probe(),
             "FFmpeg im PATH -- lerobot kodiert die Videos beim Export damit"),
    ]


def operation_requirements():
    """Ohne torch und lerobot gibt es keine Policy -- alles andere haengt am Lauf.

    ``cuda`` ist seit 2026-09-29 OPTIONAL statt REQUIRED: Auf der CPU
    braucht eine Vorhersage 384 ms und reisst den 15-Hz-Takt (gemessen
    2026-09-23) -- eine Fahrt in der Simulation zum Nachsehen, was die
    Policy tut, ist damit aber immer noch moeglich. Der Reiter fragt vor dem
    Start nach, statt es zu verbieten (Festlegung Anwender: benennen statt
    verstecken).
    """
    return [
        _req("torch", "PyTorch", REQUIRED, module_probe("torch"),
             "PyTorch zum Laden des Modells"),
        _req("cuda", "CUDA-Geraet", OPTIONAL, cuda_probe(),
             "eine CUDA-GPU -- auf der CPU braucht eine Vorhersage 384 ms und "
             "reisst den 15-Hz-Takt (gemessen 2026-09-23). Ohne GPU nur zum "
             "Nachsehen in der Simulation, nicht an der Anlage"),
        _req("lerobot", "lerobot", REQUIRED, module_probe("lerobot"),
             "lerobot 0.6.1 -- das Modell wird in diesem Format geladen"),
        _req("neurapy", "neurapy", OPTIONAL, module_probe("neurapy"),
             _NEURAPY_HINT + " -- nur fuer die Neura-Steuerung; gegen den "
             "SimRobot nicht noetig"),
        _req("opencv", "OpenCV", OPTIONAL, module_probe("cv2"),
             "OpenCV (cv2) -- fuer jede echte Kamera; mit Platzhalterbildern "
             "nicht noetig"),
        _req("gxipy", "Daheng SDK (gxipy)", OPTIONAL, module_probe("gxipy"),
             "gxipy (Galaxy SDK) -- nur, wenn einem Platz eine Daheng-Kamera "
             "zugewiesen ist"),
    ]


#: Modusname -> Zusammenstellung. Neue Modi hier eintragen.
BY_MODE = {
    "systemcheck": systemcheck_requirements,
    "recording": recording_requirements,
    "training": training_requirements,
    "operation": operation_requirements,
}


def report_for(mode):
    """Prueft alle Voraussetzungen eines Modus und liefert einen Report."""
    if mode not in BY_MODE:
        raise KeyError("Unbekannter Modus '%s' (%s)" % (mode, ", ".join(sorted(BY_MODE))))
    return Report([r.check() for r in BY_MODE[mode]()])

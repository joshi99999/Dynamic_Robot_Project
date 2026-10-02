"""Nebenlaeufigkeit der Oberflaeche: Arbeitsthreads und Unterprozesse.

Zwei Wege, bewusst getrennt:

* :class:`TaskRunner` -- kurze Funktionen in Arbeitsthreads, Ergebnis
  zurueck im GUI-Thread. Getrennte Pools, damit ein laufender Bildabgriff
  den Roboter nicht ausbremst und ein Stopp nie hinter einer Bewegung
  wartet. Zuschnitt wie in der GUI der anderen Gruppe, damit ein Blick in
  beide Oberflaechen dasselbe Muster zeigt.

* :class:`ProcessRunner` -- die Skripte aus ``tests/`` und ``tools/`` als
  Unterprozess, mit laufender Ausgabe. Ueber QProcess statt threading,
  weil Qt die Ausgabe dann selbst im GUI-Thread zustellt und ein
  haengender Controller-Aufruf hier nichts blockiert.
"""

import logging
import os
import sys
from concurrent.futures import ThreadPoolExecutor

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, Signal, Slot

from . import _bootstrap
from . import requirements as requirements_module
from ..adapters import Cancelled

log = logging.getLogger(__name__)


class TaskRunner(QObject):
    """Fuehrt Funktionen in Arbeitsthreads aus und liefert im GUI-Thread ab.

    Pools:
        "io"     Datei- und Pruefarbeit (2 Threads)
        "camera" Kameras suchen und oeffnen -- streng nacheinander
        "robot"  alles, was mit dem Roboter spricht -- streng nacheinander
        "stop"   Nothalt, wird nie von einer laufenden Bewegung blockiert

    "camera" ist eigen, weil ein Kameratreiber haengen kann (Labortag
    2026-10-01: Geraetesuche und Vorschau liessen sich nicht abbrechen,
    die Oberflaeche wirkte eingefroren). Ein haengender Treiberaufruf
    laesst sich aus Python nicht unterbrechen; er soll dann wenigstens nur
    die Kameras blockieren und nicht die Datei- und Pruefarbeit im
    "io"-Pool. Ein Thread, weil dasselbe Geraet nicht zweimal gleichzeitig
    geoeffnet werden darf.
    """

    _deliver = Signal(object, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._deliver.connect(self._on_deliver)
        self.pools = {
            "io": ThreadPoolExecutor(2, "bcgui-io"),
            "camera": ThreadPoolExecutor(1, "bcgui-camera"),
            "robot": ThreadPoolExecutor(1, "bcgui-robot"),
            "stop": ThreadPoolExecutor(1, "bcgui-stop"),
        }
        self.pending = dict((name, 0) for name in self.pools)

    def submit(self, fn, on_done=None, on_error=None, pool="io"):
        self.pending[pool] += 1

        def job():
            try:
                value = fn()
                self._deliver.emit(on_done, value)
            except Cancelled as exc:
                # Vom Bedienenden gewollt -- kein Fehler, kein Traceback im Log.
                log.info("Abgebrochen.")
                self._deliver.emit(on_error, exc)
            except Exception as exc:               # Meldung im GUI-Thread
                log.error("%s", exc, exc_info=not isinstance(exc, (RuntimeError, ValueError)))
                self._deliver.emit(on_error, exc)
            finally:
                self._deliver.emit(lambda _: self._finished(pool), None)

        return self.pools[pool].submit(job)

    def _finished(self, pool):
        self.pending[pool] -= 1

    def busy(self, pool):
        return self.pending[pool] > 0

    @Slot(object, object)
    def _on_deliver(self, callback, value):
        if callback is not None:
            callback(value)

    def shutdown(self):
        for pool in self.pools.values():
            pool.shutdown(wait=False, cancel_futures=True)


class ProcessRunner(QObject):
    """Startet ein Python-Skript des Projekts und meldet Ausgabe und Ende.

    Signale:
        line(str)             eine Ausgabezeile, sobald sie kommt
        finished(int, str)    Rueckgabewert und die vollstaendige Ausgabe
        started(list)         die tatsaechlich gestartete Kommandozeile
    """

    line = Signal(str)
    finished = Signal(int, str)
    started = Signal(list)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._process = None
        self._buffer = []
        self._tail = ""

    @property
    def running(self):
        return self._process is not None

    def start(self, argv, stdin_text=None, keep_stdin=False):
        """Startet ``argv`` (ohne Python-Programm) im Ordner "Behavior Cloning".

        ``-u`` erzwingt ungepufferte Ausgabe, sonst kommt bei einem langen
        Testlauf minutenlang nichts an. ``stdin_text`` bedient Skripte, die
        eine Eingabe erwarten (z. B. das Freigabewort von run_all.py) --
        die GUI hat da ihre eigene Rueckfrage schon gestellt.

        ``keep_stdin`` laesst den Eingabekanal OFFEN, damit spaeter noch
        geantwortet werden kann (:meth:`send`). Gebraucht wird das fuer die
        Bewertung je Episode: ``apps/record.py --ask-label`` haelt nach
        jeder Episode an und wartet auf eine Zeile. Sonst wird der Kanal
        sofort geschlossen -- ein Skript, das dann doch fragt, bekommt EOF
        und nimmt seinen sicheren Zweig (apps/record.py: ``ask()``).
        """
        if self.running:
            raise RuntimeError("Es laeuft bereits eine Pruefung.")
        self._buffer = []
        self._tail = ""

        command = [sys.executable, "-u"] + list(argv)
        process = QProcess(self)
        process.setProgram(command[0])
        process.setArguments(command[1:])
        process.setWorkingDirectory(str(_bootstrap.WORKDIR))
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.readyReadStandardOutput.connect(self._on_output)
        process.finished.connect(self._on_finished)
        process.errorOccurred.connect(self._on_error)

        # ACHTUNG: QProcess.processEnvironment() ist LEER, solange nichts
        # gesetzt wurde -- wer da hineinschreibt, startet den Unterprozess
        # mit genau diesen paar Variablen und ohne alles andere. lerobot
        # scheitert dann schon beim Import ("Could not determine home
        # directory", USERPROFILE fehlt). Deshalb von der Systemumgebung
        # ausgehen und nur ergaenzen.
        environment = QProcessEnvironment.systemEnvironment()
        # Umlaute und Sonderzeichen der Skriptausgabe nicht an der
        # Windows-Codepage scheitern lassen.
        environment.insert("PYTHONIOENCODING", "utf-8")
        environment.insert("PYTHONUTF8", "1")
        environment.insert("PYTHONPATH", os.pathsep.join(
            [str(_bootstrap.WORKDIR), str(_bootstrap.REPO_ROOT)]))
        # Programme, die neben dem Interpreter liegen (ffmpeg in
        # Library\bin), sind nur im PATH, wenn die Umgebung aktiviert
        # wurde. Beim Start ueber python.exe -m bc.gui ist sie das nicht --
        # lerobot wuerde beim Export dann kein ffmpeg finden.
        tool_dirs = [str(d) for d in requirements_module.interpreter_tool_dirs()]
        environment.insert("PATH", os.pathsep.join(
            tool_dirs + [environment.value("PATH", os.environ.get("PATH", ""))]))
        process.setProcessEnvironment(environment)

        self._process = process
        process.start()
        self.started.emit(command)
        if stdin_text is not None:
            process.write(stdin_text.encode("utf-8"))
        if not keep_stdin:
            process.closeWriteChannel()

    def send(self, text):
        """Eine Zeile an den laufenden Unterprozess schicken.

        Nur sinnvoll, wenn mit ``keep_stdin=True`` gestartet wurde.
        Liefert True, wenn geschrieben werden konnte.
        """
        if self._process is None:
            return False
        if not text.endswith("\n"):
            text += "\n"
        written = self._process.write(text.encode("utf-8"))
        return written > 0

    def close_stdin(self):
        """Eingabekanal schliessen -- danach bekommt das Skript EOF."""
        if self._process is not None:
            self._process.closeWriteChannel()

    def stop(self):
        """Bricht den laufenden Unterprozess ab."""
        if self._process is None:
            return
        self._process.kill()

    def _decode(self, data):
        return bytes(data).decode("utf-8", errors="replace").replace("\r\n", "\n")

    @Slot()
    def _on_output(self):
        if self._process is None:
            return
        text = self._tail + self._decode(self._process.readAllStandardOutput())
        parts = text.split("\n")
        self._tail = parts.pop()                    # angefangene Zeile aufheben
        for part in parts:
            self._buffer.append(part)
            self.line.emit(part)

    @Slot(QProcess.ProcessError)
    def _on_error(self, _error):
        if self._process is None:
            return
        message = "Prozessfehler: %s" % self._process.errorString()
        self._buffer.append(message)
        self.line.emit(message)

    @Slot(int, QProcess.ExitStatus)
    def _on_finished(self, code, status):
        if self._tail:
            self._buffer.append(self._tail)
            self.line.emit(self._tail)
            self._tail = ""
        if status == QProcess.ExitStatus.CrashExit:
            code = code or -1
        text = "\n".join(self._buffer)
        self._process.deleteLater()
        self._process = None
        self.finished.emit(int(code), text)

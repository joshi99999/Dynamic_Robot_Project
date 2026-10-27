"""Code geaendert -- muss die Oberflaeche neu starten? (AP 1.2)

Wunsch Anwender 2026-10-26: nach einer Aenderung nicht von Hand neu
starten muessen. Ein echtes Neuladen im laufenden Prozess
(``importlib.reload``) wird bewusst NICHT gemacht: Es tauscht Module, aber
keine schon gebauten Reiter, keine laufenden Objekte und keine Klassen,
von denen schon Exemplare leben -- danach liefe eine Mischung aus altem
und neuem Code, und das ausgerechnet in dem Prozess, der den Roboter
bedient. Stattdessen:

* ``apps/`` und ``tools/`` laufen als eigener Prozess (Aufnahme, Training,
  Pruefungen) und lesen ihren Code bei jedem Start neu ein -- Aenderungen
  dort wirken **ohne** Neustart.
* ``bc/`` laedt die Oberflaeche selbst (Kommandozeilen, Pruefungen,
  Anzeigen). Die Unterprozesse saehen den neuen Stand zwar auch sofort,
  die Oberflaeche aber nicht -- beide liefen auseinander. Deshalb dort:
  **Neustart**, als Knopf im Systemcheck (``MainWindow.restart_gui``).

Qt-frei, damit testbar ohne PySide6.
"""

import sys
from pathlib import Path

#: Ordner, deren Python-Dateien beobachtet werden (relativ zum Arbeitsordner).
WATCHED = ("bc", "apps", "tools")
#: Davon laufen diese nur als eigener Prozess -- wirken ohne Neustart.
SUBPROCESS_ONLY = ("apps", "tools")


def snapshot(workdir, watched=WATCHED):
    """``{relativer Pfad: mtime}`` aller .py-Dateien der beobachteten Ordner."""
    workdir = Path(workdir)
    out = {}
    for folder in watched:
        base = workdir / folder
        if not base.is_dir():
            continue
        for path in base.rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            try:
                out[path.relative_to(workdir).as_posix()] = path.stat().st_mtime
            except OSError:
                pass  # waehrend des Speicherns kurz weg
    return out


def changed(before, after):
    """Geaenderte, neue und geloeschte Dateien zwischen zwei Schnappschuessen."""
    names = set(before) | set(after)
    return sorted(n for n in names if before.get(n) != after.get(n))


def classify(paths):
    """-> (Neustart noetig, wirkt sofort)."""
    restart, immediate = [], []
    for path in paths:
        top = path.split("/", 1)[0]
        (immediate if top in SUBPROCESS_ONLY else restart).append(path)
    return restart, immediate


class CodeWatch(object):
    """Merkt sich den Code-Stand beim Start der Oberflaeche."""

    def __init__(self, workdir):
        self.workdir = Path(workdir)
        self.start = snapshot(self.workdir)

    def status(self):
        """-> (Neustart noetig, wirkt sofort) seit dem Start."""
        return classify(changed(self.start, snapshot(self.workdir)))


def describe(restart, immediate, limit=6):
    """Zeilen fuer die Anzeige im Systemcheck."""
    def names(paths):
        shown = ", ".join(paths[:limit])
        return shown + (" … (+%d)" % (len(paths) - limit) if len(paths) > limit else "")

    if not restart and not immediate:
        return "Code unverändert seit dem Start der Oberfläche."
    lines = []
    if restart:
        lines.append("Neustart nötig — geändert in bc/: %s" % names(restart))
    if immediate:
        lines.append("Wirkt ohne Neustart (eigener Prozess): %s" % names(immediate))
    return "\n".join(lines)


def restart_command(mode):
    """Programm und Argumente fuer den neuen Prozess (wie BC-Oberflaeche.bat)."""
    return sys.executable, ["-m", "bc.gui", "--mode", mode]


def busy_reasons(tabs):
    """Was einen Neustart gerade verbietet -- je Reiter ``busy_reason()``."""
    out = []
    for tab in tabs:
        try:
            reason = tab.busy_reason()
        except Exception as exc:  # lieber blockieren als raten
            reason = "Zustand nicht lesbar (%s)" % exc
        if reason:
            out.append("%s: %s" % (tab.title, reason))
    return out

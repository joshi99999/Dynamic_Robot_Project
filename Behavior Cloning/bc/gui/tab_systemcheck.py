"""Modus "Systemcheck": Unittests und Geraetepruefungen ausfuehren.

Die Liste kommt aus checks.catalogue() und ist nach Eingriffstiefe
gestaffelt, weil das am Labortag die eigentliche Frage ist: laeuft hier
gerade etwas, das den Roboter bewegt? Ausgefuehrt wird als Unterprozess --
dieselbe Kommandozeile, die auch im Terminal gilt (Begruendung in
checks.py).

Mehrere angehakte Punkte laufen nacheinander, nicht parallel: sie greifen
auf denselben Controller und dieselben Kameras zu.
"""

import logging

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (QAbstractItemView, QGroupBox, QHBoxLayout, QLabel,
                               QMessageBox, QPushButton, QSplitter, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout)

from . import checks as checks_module
from . import session as session_module
from .runner import ProcessRunner
from .tab_base import ModeTab
from .widgets import BAD, GOOD, LogView, NEUTRAL, WARN

log = logging.getLogger(__name__)

#: Freigabewort, das tests/run_all.py bei --real-robot auf stdin erwartet.
REAL_ROBOT_KEYWORD = "ANLAGE\n"

_STATE_COLOURS = {"laeuft": NEUTRAL, "bestanden": GOOD, "fehlgeschlagen": BAD,
                  "abgebrochen": WARN}


class SystemcheckTab(ModeTab):
    mode = "systemcheck"
    title = "Systemcheck"

    def __init__(self, session, runner, parent=None):
        super().__init__(session, runner, parent)
        self.process = ProcessRunner(self)
        self.process.line.connect(self._on_line)
        self.process.finished.connect(self._on_finished)
        self.process.started.connect(self._on_started)
        self._queue = []
        self._current = None
        self._items = {}
        self._outcomes = {}

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Prüfung", "Ergebnis"])
        self.tree.setColumnWidth(0, 330)
        self.tree.setRootIsDecorated(True)
        self.tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.tree.currentItemChanged.connect(self._on_current_changed)
        self._build_tree()

        self.description = QLabel("Einen Punkt auswählen, um zu sehen, was er tut.")
        self.description.setWordWrap(True)
        self.description.setMinimumHeight(56)
        self.description.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.btn_selected = QPushButton("Angehakte ausführen")
        self.btn_free = QPushButton("Alle hardwarefreien ausführen")
        self.btn_cancel = QPushButton("Abbrechen")
        self.btn_cancel.setEnabled(False)
        self.btn_selected.clicked.connect(self.run_checked)
        self.btn_free.clicked.connect(self.run_all_free)
        self.btn_cancel.clicked.connect(self.cancel)

        buttons = QHBoxLayout()
        buttons.addWidget(self.btn_free)
        buttons.addWidget(self.btn_selected)
        buttons.addWidget(self.btn_cancel)
        buttons.addStretch(1)

        left_box = QGroupBox("Prüfungen")
        left = QVBoxLayout(left_box)
        left.addWidget(self.tree, 1)
        left.addWidget(self.description)
        left.addLayout(buttons)

        self.output = LogView()
        right_box = QGroupBox("Ausgabe")
        right = QVBoxLayout(right_box)
        self.summary = QLabel("noch nichts ausgeführt")
        right.addWidget(self.summary)
        right.addWidget(self.output, 1)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_box)
        splitter.addWidget(right_box)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([520, 720])

        layout = QVBoxLayout(self)
        layout.addWidget(self.requirement_view)
        layout.addWidget(splitter, 1)

    # -- Aufbau -------------------------------------------------------------

    def _build_tree(self):
        self.tree.clear()
        self._items = {}
        for level, items in checks_module.by_level():
            group = QTreeWidgetItem(self.tree, [checks_module.LEVEL_LABELS[level], ""])
            group.setFlags(Qt.ItemFlag.ItemIsEnabled)
            font = group.font(0)
            font.setBold(True)
            group.setFont(0, font)
            if level == checks_module.MOTION:
                group.setForeground(0, Qt.GlobalColor.darkRed)
            for check in items:
                item = QTreeWidgetItem(group, [check.label, ""])
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                state = Qt.CheckState.Checked if level == checks_module.FREE \
                    else Qt.CheckState.Unchecked
                item.setCheckState(0, state)
                item.setData(0, Qt.ItemDataRole.UserRole, check.key)
                item.setToolTip(0, check.description)
                self._items[check.key] = item
            group.setExpanded(True)

    # -- Auswahl ------------------------------------------------------------

    def _checked_keys(self):
        keys = []
        for check in checks_module.catalogue():
            item = self._items[check.key]
            if item.checkState(0) == Qt.CheckState.Checked:
                keys.append(check.key)
        return keys

    def _on_current_changed(self, item, _previous):
        if item is None:
            return
        key = item.data(0, Qt.ItemDataRole.UserRole)
        if key is None:
            self.description.setText("")
            return
        check = checks_module.find(key)
        text = check.description
        outcome = self._outcomes.get(key)
        if outcome is not None and outcome.failures:
            text += "\n\nFehlgeschlagen: " + ", ".join(outcome.failures[:6])
        self.description.setText(text)

    # -- Starten ------------------------------------------------------------

    def run_all_free(self):
        keys = [c.key for c in checks_module.catalogue() if c.level == checks_module.FREE]
        self._start(keys)

    def run_checked(self):
        keys = self._checked_keys()
        if not keys:
            QMessageBox.information(self, "Nichts ausgewählt",
                                    "Keine Prüfung angehakt.")
            return
        self._start(keys)

    def _start(self, keys):
        if self.process.running:
            QMessageBox.information(self, "Läuft bereits",
                                    "Es läuft schon eine Prüfung. Erst abbrechen "
                                    "oder abwarten.")
            return
        keys = self._filter_by_requirements(keys)
        if not keys:
            return
        if not self._confirm_motion(keys):
            return
        self._queue = list(keys)
        for key in self._queue:
            self._set_state(key, "")
            self._outcomes.pop(key, None)
        self.output.clear()
        self.summary.setText("%d Prüfungen in der Warteschlange" % len(self._queue))
        self._run_next()

    def _filter_by_requirements(self, keys):
        """Punkte aussortieren, deren Voraussetzungen fehlen -- mit Meldung."""
        report = self.requirement_view.refresh()
        available = dict((r.key, r.ok) for r in report)
        runnable, blocked = [], []
        for key in keys:
            check = checks_module.find(key)
            missing = [n for n in check.needs if not available.get(n, False)]
            if missing:
                blocked.append((check, missing))
            else:
                runnable.append(key)
        if blocked:
            lines = ["Diese Punkte werden übersprungen, weil etwas fehlt:", ""]
            for check, missing in blocked:
                hints = []
                for result in report:
                    if result.key in missing:
                        hints.append(result.requirement.missing_hint)
                lines.append("  %s" % check.label)
                for hint in hints:
                    lines.append("      braucht %s" % hint)
            if runnable:
                lines += ["", "Die übrigen %d Punkte laufen trotzdem." % len(runnable)]
            QMessageBox.warning(self, "Voraussetzung fehlt", "\n".join(lines))
        return runnable

    def _confirm_motion(self, keys):
        """Rueckfrage, bevor irgendetwas den Roboter bewegt."""
        motion = [checks_module.find(k) for k in keys
                  if checks_module.find(k).level == checks_module.MOTION]
        if not motion:
            return True
        status = self.session.status()
        lines = ["Diese Punkte bewegen den Roboter:", ""]
        lines += ["  - " + check.label for check in motion]
        lines += ["", "Backend laut Steuerung: %s" % status.text]
        if status.state == session_module.REAL:
            lines += ["", "ACHTUNG: Das ist die REALE ANLAGE, nicht die Simulation."]
        elif status.state == session_module.OFFLINE:
            lines += ["", "Es ist nichts verbunden -- die Prüfungen verbinden selbst. "
                          "Welche Steuerung dabei antwortet, steht erst dann fest."]
        lines += ["", "Arbeitsraum frei? Not-Aus in Reichweite?"]
        return self.confirm("Roboter bewegt sich", "\n".join(lines))

    def _run_next(self):
        if not self._queue:
            self._current = None
            self._set_buttons_running(False)
            self._show_total()
            return
        key = self._queue.pop(0)
        check = checks_module.find(key)
        self._current = check
        self._set_state(key, "läuft")
        self._set_buttons_running(True)
        stdin = REAL_ROBOT_KEYWORD if "--real-robot" in check.argv else check.stdin
        try:
            self.process.start(check.argv, stdin_text=stdin)
        except Exception as exc:
            log.error("Start gescheitert: %s", exc)
            self._set_state(key, "fehlgeschlagen")
            self._run_next()

    def cancel(self):
        self._queue = []
        if self.process.running:
            self.process.stop()

    # -- Ergebnis -----------------------------------------------------------

    def _on_started(self, command):
        self.output.append_record("$ " + " ".join(command[1:]), logging.INFO)

    def _on_line(self, text):
        self.output.append_line(text)

    def _on_finished(self, code, text):
        check = self._current
        self._current = None
        if check is None:
            return
        outcome = checks_module.Outcome(check, code, text)
        self._outcomes[check.key] = outcome
        state = "bestanden" if outcome.ok else "fehlgeschlagen"
        self._set_state(check.key, state, outcome.summary())
        self.output.append_record("-> %s: %s" % (check.label, outcome.summary()),
                                  logging.INFO if outcome.ok else logging.ERROR)
        self.output.append_record("", logging.INFO)
        self._run_next()

    def _set_state(self, key, state, detail=None):
        item = self._items.get(key)
        if item is None:
            return
        item.setText(1, detail or state)
        colour = _STATE_COLOURS.get(state)
        if colour:
            item.setForeground(1, QBrush(QColor(colour)))

    def _set_buttons_running(self, running):
        self.btn_selected.setEnabled(not running)
        self.btn_free.setEnabled(not running)
        self.btn_cancel.setEnabled(running)

    def _show_total(self):
        done = [o for o in self._outcomes.values()]
        if not done:
            self.summary.setText("noch nichts ausgeführt")
            return
        failed = [o for o in done if not o.ok]
        if failed:
            self.summary.setText(
                "%d von %d Prüfungen fehlgeschlagen: %s"
                % (len(failed), len(done), ", ".join(o.check.label for o in failed)))
        else:
            self.summary.setText("alle %d Prüfungen bestanden" % len(done))

    # -- Lebenszyklus -------------------------------------------------------

    def shutdown(self):
        self.cancel()

"""Modus "Systemcheck": Unittests, Messungen und Geraetepruefungen ausfuehren.

Die Liste kommt aus checks.catalogue() und ist nach Eingriffstiefe
gestaffelt, weil das am Labortag die eigentliche Frage ist: laeuft hier
gerade etwas, das den Roboter bewegt? Ausgefuehrt wird als Unterprozess --
dieselbe Kommandozeile, die auch im Terminal gilt (Begruendung in
checks.py). Die Kommandozeile steht unter den Einstellungen, damit man sie
ins Protokoll oder ins Terminal uebernehmen kann.

Mehrere angehakte Punkte laufen nacheinander, nicht parallel: sie greifen
auf denselben Controller und dieselben Kameras zu.

Nach jedem Punkt erscheinen seine Urteile ([OK]/[WARNUNG]/[FEHLER]) in der
Liste "Bewertung"; "Protokoll speichern" legt Bewertung und Ausgabe in
Berichte/ ab (Wunsch Anwender 2026-10-04).

"Reale Anlage" ist KEIN eigener Schalter hier: es gilt der Haken "Reale
Anlage freigeben" in der Backend-Leiste. Ist er gesetzt, laufen Werkzeuge,
die das kennen, mit ``--real-robot`` und bekommen das Freigabewort.
"""

import logging
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox,
                               QDoubleSpinBox, QFormLayout, QGroupBox,
                               QHBoxLayout, QLabel, QLineEdit, QMessageBox,
                               QPushButton, QSpinBox, QSplitter, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout)

from . import _bootstrap
from . import checks as checks_module
from . import reload as reload_module
from . import session as session_module
from . import training as training_module
from .runner import ProcessRunner
from .tab_base import ModeTab
from .widgets import BAD, GOOD, LogView, NEUTRAL, WARN, block_wheel

log = logging.getLogger(__name__)

_STATE_COLOURS = {"laeuft": NEUTRAL, "bestanden": GOOD, "Befunde": WARN,
                  "fehlgeschlagen": BAD, "abgebrochen": WARN}
_VERDICT_COLOURS = {"ok": GOOD, "info": NEUTRAL, "warnung": WARN, "fehler": BAD}


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
        self._current_real = False
        self._items = {}
        self._outcomes = {}
        self._order = []                 # Ausfuehrungsreihenfolge fuers Protokoll
        self._values = {}                # Parameter je Pruefung
        self._selected = None

        # -- links: Liste, Beschreibung, Einstellungen ----------------------
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Prüfung", "Ergebnis"])
        self.tree.setColumnWidth(0, 300)
        self.tree.setRootIsDecorated(True)
        self.tree.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.tree.currentItemChanged.connect(self._on_current_changed)
        self._build_tree()

        self.description = QLabel("Einen Punkt auswählen, um zu sehen, was er tut.")
        self.description.setWordWrap(True)
        self.description.setMinimumHeight(56)
        self.description.setAlignment(Qt.AlignmentFlag.AlignTop)

        self.param_box = QGroupBox("Einstellungen")
        self.param_form = QFormLayout(self.param_box)
        self.param_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self.param_box.setVisible(False)

        self.command = QLabel("")
        self.command.setWordWrap(True)
        self.command.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.command.setFont(QFont("Consolas", 9))

        self.real_hint = QLabel("")
        self.real_hint.setWordWrap(True)

        self.btn_one = QPushButton("Diesen Punkt ausführen")
        self.btn_selected = QPushButton("Angehakte ausführen")
        self.btn_free = QPushButton("Alle hardwarefreien")
        self.btn_cancel = QPushButton("Abbrechen")
        self.btn_cancel.setEnabled(False)
        self.btn_one.setEnabled(False)
        self.btn_one.clicked.connect(self.run_selected)
        self.btn_selected.clicked.connect(self.run_checked)
        self.btn_free.clicked.connect(self.run_all_free)
        self.btn_cancel.clicked.connect(self.cancel)

        buttons = QHBoxLayout()
        buttons.addWidget(self.btn_one)
        buttons.addWidget(self.btn_selected)
        buttons.addWidget(self.btn_free)
        buttons.addWidget(self.btn_cancel)
        buttons.addStretch(1)

        left_box = QGroupBox("Prüfungen")
        left = QVBoxLayout(left_box)
        left.addWidget(self.tree, 1)
        left.addWidget(self.description)
        left.addWidget(self.param_box)
        left.addWidget(self.command)
        left.addWidget(self.real_hint)
        left.addLayout(buttons)

        # -- rechts: Bewertung oben, Ausgabe unten --------------------------
        self.summary = QLabel("noch nichts ausgeführt")
        self.summary.setWordWrap(True)
        self.verdicts = QTreeWidget()
        self.verdicts.setHeaderLabels(["Bewertung", ""])
        self.verdicts.setColumnWidth(0, 260)
        self.verdicts.setRootIsDecorated(True)
        self.verdicts.setWordWrap(True)
        self.verdicts.setUniformRowHeights(False)
        verdict_box = QGroupBox("Bewertung")
        verdict_layout = QVBoxLayout(verdict_box)
        verdict_layout.addWidget(self.summary)
        verdict_layout.addWidget(self.verdicts, 1)

        self.output = LogView()
        self.btn_save = QPushButton("Protokoll speichern")
        self.btn_save.setToolTip("Bewertung und vollständige Ausgabe nach Berichte/")
        self.btn_save.clicked.connect(self.save_protocol)
        self.btn_clear = QPushButton("Ausgabe leeren")
        self.btn_clear.clicked.connect(self.clear_results)
        out_buttons = QHBoxLayout()
        out_buttons.addWidget(self.btn_save)
        out_buttons.addWidget(self.btn_clear)
        out_buttons.addStretch(1)
        output_box = QGroupBox("Ausgabe")
        output_layout = QVBoxLayout(output_box)
        output_layout.addWidget(self.output, 1)
        output_layout.addLayout(out_buttons)

        right = QSplitter(Qt.Orientation.Vertical)
        right.addWidget(verdict_box)
        right.addWidget(output_box)
        right.setSizes([260, 480])

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left_box)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([560, 720])

        layout = QVBoxLayout(self)
        layout.addWidget(self.requirement_view)
        layout.addWidget(self._build_code_box())
        layout.addWidget(splitter, 1)

        session.add_listener(self._on_session_status)
        self._update_real_hint()

    # -- Code-Stand und Neustart (bc/gui/reload.py) --------------------------

    def _build_code_box(self):
        """Wunsch Anwender 2026-10-26: nach Aenderungen nicht von Hand neu
        starten. Kein Neuladen im laufenden Prozess -- Begruendung in
        bc/gui/reload.py -- sondern Anzeige plus Neustart-Knopf."""
        self._code_watch = reload_module.CodeWatch(_bootstrap.WORKDIR)
        #: Wird vom eigenen Fenster gesetzt (MainWindow.restart_gui). Als
        #: Reiter in der Gruppen-GUI bleibt es None: ein Neustart beendete
        #: deren Programm mit.
        self._restart_handler = None
        box = QGroupBox("Code-Stand")
        row = QHBoxLayout(box)
        self.code_status = QLabel("")
        self.code_status.setWordWrap(True)
        row.addWidget(self.code_status, 1)
        self.btn_code_check = QPushButton("Auf Änderungen prüfen")
        self.btn_code_check.setToolTip(
            "Vergleicht bc/, apps/ und tools/ mit dem Stand beim Start. Änderungen in "
            "apps/ und tools/ wirken ohne Neustart (eigener Prozess), in bc/ braucht "
            "die Oberfläche einen Neustart.")
        self.btn_code_check.clicked.connect(self.refresh_code_status)
        row.addWidget(self.btn_code_check)
        self.btn_restart = QPushButton("Oberfläche neu starten")
        self.btn_restart.setToolTip(
            "Schließt die Oberfläche sauber (Verbindung, Kameras) und startet sie mit "
            "demselben Python neu. Gesperrt, solange eine Aufnahme, Vorschau, "
            "Teach-Serie oder Prüfung läuft.")
        self.btn_restart.clicked.connect(self._restart_clicked)
        self.btn_restart.setEnabled(False)
        row.addWidget(self.btn_restart)
        self.refresh_code_status()
        return box

    def set_restart_handler(self, handler):
        self._restart_handler = handler
        self.btn_restart.setEnabled(handler is not None)
        self.refresh_code_status()

    def refresh_code_status(self):
        restart, immediate = self._code_watch.status()
        text = reload_module.describe(restart, immediate)
        if restart and self._restart_handler is None:
            text += ("\nAls Reiter in der Gruppen-GUI: deren Programm neu starten.")
        self.code_status.setText(text)
        self.code_status.setStyleSheet("color:%s" % (WARN if restart else NEUTRAL))

    def _restart_clicked(self):
        if self._restart_handler is not None:
            self._restart_handler()

    def refresh(self):
        super().refresh()
        self.refresh_code_status()

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
                # Vorbelegt nur, was ohne Geraet und ohne Eingabe sinnvoll
                # ist: die Auswertungen brauchen erst eine Aufnahme.
                state = Qt.CheckState.Checked if check.key in (
                    "unittests", "layering", "cameras_sim") else Qt.CheckState.Unchecked
                item.setCheckState(0, state)
                item.setData(0, Qt.ItemDataRole.UserRole, check.key)
                item.setToolTip(0, check.description)
                self._items[check.key] = item
            group.setExpanded(True)

    # -- Auswahl und Einstellungen -------------------------------------------

    def _checked_keys(self):
        keys = []
        for check in checks_module.catalogue():
            item = self._items[check.key]
            if item.checkState(0) == Qt.CheckState.Checked:
                keys.append(check.key)
        return keys

    def values_for(self, key):
        """Eingestellte Parameter einer Pruefung (mit Defaults beim ersten Mal).

        Defaults, die von der Platte abhaengen (neuester Aufnahmeordner),
        werden beim ersten Zugriff gezogen und dann gehalten -- sonst
        sprangen sie unter den Haenden des Bedienenden um.
        """
        if key not in self._values:
            self._values[key] = checks_module.find(key).defaults()
        return self._values[key]

    def _real(self):
        return bool(self.session.allow_real)

    def _on_current_changed(self, item, _previous):
        key = None if item is None else item.data(0, Qt.ItemDataRole.UserRole)
        self._selected = key
        self.btn_one.setEnabled(key is not None and not self.process.running)
        self._rebuild_params(key)
        if key is None:
            self.description.setText("")
            self.command.setText("")
            return
        check = checks_module.find(key)
        text = check.description
        outcome = self._outcomes.get(key)
        if outcome is not None and outcome.failures:
            text += "\n\nFehlgeschlagen: " + ", ".join(outcome.failures[:6])
        self.description.setText(text)
        self._update_command()

    def _rebuild_params(self, key):
        while self.param_form.rowCount():
            self.param_form.removeRow(0)
        check = None if key is None else checks_module.find(key)
        if check is None or not check.params:
            self.param_box.setVisible(False)
            return
        values = self.values_for(key)
        for param in check.params:
            widget = self._param_widget(key, param, values.get(param.key))
            if param.help:
                widget.setToolTip(param.help)
            self.param_form.addRow(param.label, widget)
        self.param_box.setVisible(True)

    def _param_widget(self, key, param, value):
        def store(new):
            self.values_for(key)[param.key] = new
            self._update_command()

        if param.kind == "bool":
            widget = QCheckBox()
            widget.setChecked(bool(value))
            widget.toggled.connect(store)
        elif param.kind == "int":
            widget = block_wheel(QSpinBox())
            widget.setRange(param.minimum if param.minimum is not None else 0,
                            param.maximum if param.maximum is not None else 100000)
            widget.setValue(int(value))
            widget.valueChanged.connect(store)
        elif param.kind == "float":
            widget = block_wheel(QDoubleSpinBox())
            widget.setRange(param.minimum if param.minimum is not None else 0.0,
                            param.maximum if param.maximum is not None else 1e6)
            widget.setDecimals(2)
            widget.setValue(float(value))
            widget.valueChanged.connect(store)
        elif param.kind == "choice":
            widget = block_wheel(QComboBox())
            for choice in param.choices:
                widget.addItem(choice or "(Standard)", choice)
            index = widget.findData(value)
            widget.setCurrentIndex(max(0, index))
            widget.currentIndexChanged.connect(lambda _i: store(widget.currentData()))
        else:
            widget = QLineEdit(str(value or ""))
            widget.textChanged.connect(store)
        return widget

    def _update_command(self):
        if self._selected is None:
            self.command.setText("")
            return
        check = checks_module.find(self._selected)
        argv = check.build_argv(self.values_for(self._selected), real=self._real())
        self.command.setText("$ " + training_module.command_line(argv))

    def _on_session_status(self, _status=None):
        self._update_real_hint()
        self._update_command()

    def _update_real_hint(self):
        if self._real():
            self.real_hint.setText(
                "REALE ANLAGE freigegeben (Haken oben in der Backend-Leiste): "
                "Werkzeuge, die das kennen, laufen mit --real-robot.")
            self.real_hint.setStyleSheet("color:%s; font-weight:bold" % BAD)
        else:
            self.real_hint.setText(
                "Simulation: Bewegungen nur, wenn die Steuerung als Simulation "
                "antwortet. Für die Anlage oben „Reale Anlage freigeben“ setzen.")
            self.real_hint.setStyleSheet("color:%s" % NEUTRAL)

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

    def run_selected(self):
        if self._selected is not None:
            self._start([self._selected])

    def _start(self, keys):
        if self.process.running:
            QMessageBox.information(self, "Läuft bereits",
                                    "Es läuft schon eine Prüfung. Erst abbrechen "
                                    "oder abwarten.")
            return
        keys = self._filter_by_requirements(keys)
        if not keys:
            return
        real = self._real()
        if not self._confirm(keys, real):
            return
        self._queue = list(keys)
        self._current_real = real
        for key in self._queue:
            self._set_state(key, "")
        self.summary.setText("%d Prüfung%s in der Warteschlange"
                             % (len(self._queue), "" if len(self._queue) == 1 else "en"))
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

    def _confirm(self, keys, real):
        """Rueckfrage, bevor irgendetwas bewegt, bestromt oder schaltet."""
        risky = [checks_module.find(k) for k in keys
                 if checks_module.find(k).level == checks_module.MOTION
                 or checks_module.find(k).confirm]
        if not risky:
            return True
        status = self.session.status()
        lines = ["Diese Punkte greifen in die Anlage ein:", ""]
        for check in risky:
            what = {checks_module.POWER_CONFIRM: "bestromt",
                    checks_module.GRIPPER_CONFIRM: "schaltet den Greifer",
                    checks_module.MOTION_CONFIRM: "bewegt"}.get(check.confirm, "bewegt")
            lines.append("  - %s (%s)" % (check.label, what))
        lines += ["", "Backend laut Steuerung: %s" % status.text]
        if real:
            lines += ["", "ACHTUNG: REALE ANLAGE freigegeben — die Werkzeuge laufen "
                          "mit --real-robot, nicht gegen die Simulation."]
        elif status.state == session_module.OFFLINE:
            lines += ["", "Es ist nichts verbunden -- die Prüfungen verbinden selbst. "
                          "Welche Steuerung dabei antwortet, steht erst dann fest."]
        lines += ["", "Arbeitsraum frei? Not-Aus in Reichweite?"]
        return self.confirm("Eingriff in die Anlage", "\n".join(lines))

    def _run_next(self):
        if not self._queue:
            self._current = None
            self._set_buttons_running(False)
            self._show_total()
            return
        key = self._queue.pop(0)
        check = checks_module.find(key)
        self._current = check
        self._set_state(key, "laeuft", "läuft")
        self._set_buttons_running(True)
        argv = check.build_argv(self.values_for(key), real=self._current_real)
        try:
            self.process.start(argv, stdin_text=check.stdin_for(self._current_real))
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
        if check.key in self._order:
            self._order.remove(check.key)
        self._order.append(check.key)
        self._set_state(check.key, outcome.state, outcome.summary())
        self._show_verdicts(outcome)
        self.output.append_record("-> %s: %s" % (check.label, outcome.summary()),
                                  logging.INFO if outcome.state == "bestanden"
                                  else logging.WARNING if outcome.state == "Befunde"
                                  else logging.ERROR)
        self.output.append_record("", logging.INFO)
        self._run_next()

    def _show_verdicts(self, outcome):
        """Urteile eines Punktes in die Bewertungsliste -- schlimmste zuerst."""
        for index in range(self.verdicts.topLevelItemCount()):
            item = self.verdicts.topLevelItem(index)
            if item.data(0, Qt.ItemDataRole.UserRole) == outcome.check.key:
                self.verdicts.takeTopLevelItem(index)
                break
        head = QTreeWidgetItem([outcome.check.label, outcome.summary()])
        head.setData(0, Qt.ItemDataRole.UserRole, outcome.check.key)
        font = head.font(0)
        font.setBold(True)
        head.setFont(0, font)
        head.setForeground(1, QBrush(QColor(_STATE_COLOURS.get(outcome.state, NEUTRAL))))
        order = dict((level, i) for i, level in enumerate(reversed(checks_module.SEVERITY)))
        verdicts = sorted(outcome.verdicts, key=lambda v: order[v[0]])
        if not verdicts and outcome.failures:
            verdicts = [("fehler", "FAIL " + name) for name in outcome.failures]
        for level, text in verdicts:
            child = QTreeWidgetItem(head, [checks_module.VERDICT_LABELS[level], text])
            child.setForeground(0, QBrush(QColor(_VERDICT_COLOURS[level])))
            child.setToolTip(1, text)
        self.verdicts.addTopLevelItem(head)
        head.setExpanded(outcome.state != "bestanden")
        self.verdicts.resizeColumnToContents(1)

    def _set_state(self, key, state, detail=None):
        item = self._items.get(key)
        if item is None:
            return
        item.setText(1, detail if detail is not None else state)
        colour = _STATE_COLOURS.get(state)
        item.setForeground(1, QBrush(QColor(colour or "#333")))

    def _set_buttons_running(self, running):
        self.btn_selected.setEnabled(not running)
        self.btn_free.setEnabled(not running)
        self.btn_one.setEnabled(not running and self._selected is not None)
        self.btn_cancel.setEnabled(running)

    def _show_total(self):
        done = [self._outcomes[k] for k in self._order]
        if not done:
            self.summary.setText("noch nichts ausgeführt")
            self.summary.setStyleSheet("")
            return
        failed = [o for o in done if o.state in ("fehlgeschlagen", "abgebrochen")]
        findings = [o for o in done if o.state == "Befunde"]
        if failed:
            text = "%d von %d Prüfungen fehlgeschlagen: %s" % (
                len(failed), len(done), ", ".join(o.check.label for o in failed))
            colour = BAD
        elif findings:
            text = "alle %d durchgelaufen, mit Warnungen bei: %s" % (
                len(done), ", ".join(o.check.label for o in findings))
            colour = WARN
        else:
            text = "alle %d Prüfungen bestanden" % len(done)
            colour = GOOD
        self.summary.setText(text)
        self.summary.setStyleSheet("color:%s; font-weight:bold" % colour)

    def save_protocol(self):
        if not self._order:
            QMessageBox.information(self, "Nichts zu speichern",
                                    "Es wurde noch nichts ausgeführt.")
            return
        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
        target = Path(checks_module.WORKDIR) / "Berichte" / ("%s_Systemcheck.txt" % stamp)
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text(checks_module.protocol_text(
                [self._outcomes[k] for k in self._order],
                self.output.toPlainText(), stamp), encoding="utf-8")
        except OSError as exc:
            QMessageBox.warning(self, "Nicht gespeichert", str(exc))
            return
        self.output.append_record("Protokoll gespeichert: %s" % target, logging.INFO)

    def clear_results(self):
        if self.process.running:
            return
        self.output.clear()
        self.verdicts.clear()
        self._outcomes = {}
        self._order = []
        for key in self._items:
            self._set_state(key, "")
        self._show_total()

    # -- Lebenszyklus -------------------------------------------------------

    def shutdown(self):
        self.cancel()

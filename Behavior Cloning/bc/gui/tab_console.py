"""Modus "Konsole": Befehle aus Doku und Absprachen einfuegen und starten.

Wunsch Anwender 2026-10-21. Die Pruefung, was erlaubt ist, steht Qt-frei in
``console.py``; hier nur Bedienung: Befehl (mit Verlauf), Beispiele,
Starten/Stoppen, Ausgabe und eine Eingabezeile fuer Rueckfragen des
laufenden Programms (Freigabewort ``ANLAGE``, Bewertung je Episode).

Laeuft als Unterprozess wie Systemcheck und Aufnahme -- Stoppen beendet den
Prozess. Das ist ein SOFTWARE-Stopp und ersetzt keinen Not-Aus (AP 4.2).
"""

import logging

from PySide6.QtWidgets import (QComboBox, QGroupBox, QHBoxLayout, QLabel,
                               QLineEdit, QPushButton, QVBoxLayout)

from . import console as console_module
from .runner import ProcessRunner
from .tab_base import ModeTab
from .training import command_line
from .widgets import BAD, GOOD, LogView, NEUTRAL, block_wheel

log = logging.getLogger(__name__)

#: Wie viele Befehle der Verlauf behaelt.
HISTORY = 30


class ConsoleTab(ModeTab):
    mode = "console"
    title = "Konsole"

    def __init__(self, session, runner, parent=None):
        super().__init__(session, runner, parent)
        self.process = ProcessRunner(self)
        self.process.line.connect(self._on_line)
        self.process.finished.connect(self._on_finished)
        self.process.started.connect(self._on_started)

        box = QGroupBox("Befehl")
        box_layout = QVBoxLayout(box)
        hint = QLabel(
            "Eine Zeile <b>python …</b> wie in der Doku einfügen. Läuft im Ordner "
            "„Behavior Cloning“ mit dem Python der Oberfläche. Keine Shell: "
            "<code>|</code>, <code>&gt;</code>, <code>&amp;&amp;</code>, <code>cd</code> gehen nicht.")
        hint.setWordWrap(True)
        box_layout.addWidget(hint)

        row = QHBoxLayout()
        self.command = QComboBox()
        self.command.setEditable(True)
        self.command.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        self.command.lineEdit().setPlaceholderText("python tools/check_sim_robot.py")
        self.command.lineEdit().returnPressed.connect(self.run)
        row.addWidget(self.command, 1)
        self.btn_run = QPushButton("Ausführen")
        self.btn_run.clicked.connect(self.run)
        row.addWidget(self.btn_run)
        self.btn_stop = QPushButton("Stopp")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet("font-weight:bold; color:%s" % BAD)
        self.btn_stop.clicked.connect(self.stop)
        row.addWidget(self.btn_stop)
        box_layout.addLayout(row)

        examples = QHBoxLayout()
        examples.addWidget(QLabel("Beispiele"))
        self.examples = block_wheel(QComboBox())
        self.examples.addItem("— einfügen —", None)
        for label, text in console_module.EXAMPLES:
            self.examples.addItem(label, text)
        self.examples.activated.connect(self._insert_example)
        examples.addWidget(self.examples, 1)
        box_layout.addLayout(examples)

        answer = QHBoxLayout()
        answer.addWidget(QLabel("Eingabe an das laufende Programm"))
        self.answer = QLineEdit()
        self.answer.setPlaceholderText("z. B. ANLAGE (Freigabewort) oder j / n")
        self.answer.returnPressed.connect(self.send_answer)
        answer.addWidget(self.answer, 1)
        self.btn_send = QPushButton("Senden")
        self.btn_send.clicked.connect(self.send_answer)
        answer.addWidget(self.btn_send)
        box_layout.addLayout(answer)

        self.headline = QLabel("Bereit")
        self.headline.setStyleSheet("color:%s" % NEUTRAL)
        self.output = LogView()

        layout = QVBoxLayout(self)
        layout.addWidget(self.requirement_view)
        layout.addWidget(box)
        layout.addWidget(self.headline)
        layout.addWidget(self.output, 1)
        self._set_running(False)

    # -- Bedienung ----------------------------------------------------------

    def _insert_example(self, index):
        text = self.examples.itemData(index)
        if text:
            self.command.setEditText(text)
            self.command.lineEdit().setFocus()
        self.examples.setCurrentIndex(0)

    def run(self):
        if self.process.running:
            return
        text = self.command.currentText()
        parsed = console_module.parse_command(text)
        if not parsed.ok:
            self.headline.setText("✗ " + " ".join(parsed.problems))
            self.headline.setStyleSheet("color:%s" % BAD)
            return
        if parsed.warnings:
            question = "\n".join(parsed.warnings)
            if parsed.real_robot:
                question += ("\n\nDie Werkzeuge fragen an der Anlage zusätzlich nach dem "
                             "Freigabewort — es wird über die Eingabezeile geschickt.")
            if not self.confirm("Befehl starten?", "%s\n\n%s" % (text.strip(), question)):
                return
        self._remember(text.strip())
        self.output.clear()
        self.process.start(parsed.argv, keep_stdin=True)

    def stop(self):
        if self.process.running:
            self.process.stop()
            self.output.append_record("-> abgebrochen (Prozess beendet)", logging.WARNING)

    def send_answer(self):
        text = self.answer.text()
        if not self.process.running:
            return
        if self.process.send(text):
            self.output.append_record("<- %s" % text)
            self.answer.clear()

    def _remember(self, text):
        index = self.command.findText(text)
        if index >= 0:
            self.command.removeItem(index)
        self.command.insertItem(0, text)
        while self.command.count() > HISTORY:
            self.command.removeItem(self.command.count() - 1)
        self.command.setCurrentIndex(0)

    # -- Prozess ------------------------------------------------------------

    def _on_started(self, command):
        self._set_running(True)
        self.headline.setText("Läuft: %s" % command_line(command[2:]))
        self.headline.setStyleSheet("color:%s; font-weight:bold" % NEUTRAL)
        self.output.append_record("> %s" % command_line(command[2:]))

    def _on_line(self, line):
        self.output.append_record(line)

    def _on_finished(self, code, _text):
        self._set_running(False)
        ok = code == 0
        self.headline.setText("Fertig (Rückgabewert 0)" if ok
                              else "Beendet mit Rückgabewert %d" % code)
        self.headline.setStyleSheet("color:%s; font-weight:bold" % (GOOD if ok else BAD))

    def _set_running(self, running):
        self.btn_run.setEnabled(not running)
        self.command.setEnabled(not running)
        self.btn_stop.setEnabled(running)
        self.answer.setEnabled(running)
        self.btn_send.setEnabled(running)

    def shutdown(self):
        if self.process.running:
            self.process.stop()

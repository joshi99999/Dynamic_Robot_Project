"""Modus "Betrieb": Modell laden und die Policy fahren lassen.

Aufbau des Reiters:

    1. Modell    Checkpoint waehlen; darunter steht, woraus er stammt und
                 unter welchen Randbedingungen er trainiert wurde.
    2. Kameras   Welches Geraet auf welchem Platz. Muss zur Aufzeichnung
                 passen -- die Pruefung darunter sagt das VORHER.
    3. Fahrt     Fahrten, Takte, Neuvorhersage, Geraet, Echtzeit.

Die Ampel oben im Fenster gilt auch hier: Rot heisst reale Anlage. Der
Stoppknopf beendet den Unterprozess -- ein SOFTWARE-Stopp, der keinen
zertifizierten Not-Aus ersetzt (AP 4.2).

Bewusst NICHT gebaut: die Uebergabe an die andere Gruppe bei PRE_PLACE. Die
braucht eine zweite Schnittstelle (Uebergabe der Roboterhoheit statt eines
Rueckgabewerts) und damit eine Absprache -- siehe unten "Offen".
"""

import logging
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
                               QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                               QMessageBox, QProgressBar, QPushButton,
                               QScrollArea, QSpinBox, QSplitter, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout, QWidget)

from . import _bootstrap
from . import cameras as cameras_module
from . import operation as operation_module
from . import session as session_module
from . import training as training_module
from .runner import ProcessRunner
from .tab_base import ModeTab
from .widgets import (BAD, GOOD, LogView, NEUTRAL, WARN,
                      CameraAssignmentView, block_wheel, find_data)

log = logging.getLogger(__name__)


class OperationTab(ModeTab):
    mode = "operation"
    title = "Betrieb"

    def __init__(self, session, runner, parent=None):
        super().__init__(session, runner, parent)
        self.workdir = Path(_bootstrap.WORKDIR)
        self.process = ProcessRunner(self)
        self.process.line.connect(self._on_line)
        self.process.finished.connect(self._on_finished)
        self.process.started.connect(self._on_started)
        self._progress = None
        self._described = None
        self._problems = []
        self._out_dir = None

        left = self._build_controls()
        right = self._build_output()

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([560, 780])

        layout = QVBoxLayout(self)
        layout.addWidget(self.requirement_view)
        layout.addWidget(splitter, 1)

        session.add_listener(self._on_session_status)
        self.refresh()

    # -- Aufbau: Bedienseite ------------------------------------------------

    def _build_controls(self):
        page = QWidget()
        column = QVBoxLayout(page)
        column.setContentsMargins(0, 0, 0, 0)
        column.addWidget(self._build_model_box())
        column.addWidget(self._build_camera_box())
        column.addWidget(self._build_run_box())
        column.addStretch(1)

        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(page)

        panel = QWidget()
        panel.setMinimumWidth(500)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(area, 1)
        layout.addWidget(self._build_action_bar(), 0)
        return panel

    def _build_model_box(self):
        box = QGroupBox("1. Modell")
        layout = QVBoxLayout(box)

        row = QHBoxLayout()
        self.checkpoint = block_wheel(QComboBox())
        self.checkpoint.currentIndexChanged.connect(self._on_checkpoint_changed)
        row.addWidget(self.checkpoint, 1)
        self.btn_rescan = QPushButton("Neu suchen")
        self.btn_rescan.clicked.connect(self.refresh)
        row.addWidget(self.btn_rescan)
        layout.addLayout(row)

        self.model_info = _hint("")
        layout.addWidget(self.model_info)

        self.hold = QCheckBox("Ohne Modell fahren (Verdrahtungstest)")
        self.hold.setToolTip(
            "HoldPolicy statt Policy: die Kette aus Takt, Servo-Interpolation "
            "und Kameras läuft, der Arm hält die Stellung. Damit prüft man die "
            "Verdrahtung, bevor ein Modell drankommt.")
        self.hold.toggled.connect(self._on_hold_toggled)
        layout.addWidget(self.hold)
        return box

    def _build_camera_box(self):
        box = QGroupBox("2. Kameras")
        layout = QVBoxLayout(box)
        self.cameras = CameraAssignmentView(self.runner)
        self.cameras.changed.connect(self._on_cameras_changed)
        layout.addWidget(self.cameras)
        return box

    def _build_run_box(self):
        box = QGroupBox("3. Fahrt")
        layout = QVBoxLayout(box)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self.episodes = block_wheel(QSpinBox())
        self.episodes.setRange(1, 200)
        self.episodes.setValue(1)
        self.episodes.valueChanged.connect(self._update_command)
        form.addRow("Fahrten", self.episodes)

        self.max_steps = block_wheel(QSpinBox())
        self.max_steps.setRange(0, 100000)
        self.max_steps.setSingleStep(10)
        self.max_steps.setSpecialValueText("aus dem Modell")
        self.max_steps.setToolTip(
            "Höchstzahl Takte je Fahrt. 0 = 1.5 × längste Episode der "
            "Aufzeichnung (Default von apps/infer.py).")
        self.max_steps.valueChanged.connect(self._update_command)
        form.addRow("Takte je Fahrt", self.max_steps)

        self.replan = block_wheel(QSpinBox())
        self.replan.setRange(0, 64)
        self.replan.setSpecialValueText("aus dem Modell")
        self.replan.setToolTip(
            "Alle N Takte neu vorhersagen. Kleiner = reaktiver, aber mehr "
            "GPU-Last; die Vorhersage dauert ~77 ms und damit länger als ein "
            "Takt (67 ms).")
        self.replan.valueChanged.connect(self._update_command)
        form.addRow("Neu vorhersagen", self.replan)

        self.device = block_wheel(QComboBox())
        self.device.currentIndexChanged.connect(self._update_command)
        form.addRow("Gerät", self.device)

        # Derselbe Wert wie oben in der Backend-Leiste. Wird ein Modell
        # gewaehlt, setzt der Reiter ihn auf den der Aufzeichnung: ein
        # anderer Wert laesst apps/infer.py ohnehin abbrechen (AP 2.6).
        self.override = block_wheel(QDoubleSpinBox())
        self.override.setRange(0.05, 1.0)
        self.override.setSingleStep(0.05)
        self.override.setDecimals(2)
        self.override.setValue(float(self.session.override))
        self.override.setToolTip(
            "Geschwindigkeits-Override am Neura — dieselbe Zahl wie oben in "
            "der Backend-Leiste. Muss dem der Aufzeichnung entsprechen, das "
            "Tempo ist mitgelernt (AP 2.6); apps/infer.py erzwingt das.")
        self.override.valueChanged.connect(self.session.set_override)
        form.addRow("Override", self.override)

        layout.addLayout(form)

        self.realtime = QCheckBox("SimRobot mit echter Uhr (Echtzeit)")
        self.realtime.setToolTip(
            "Ohne Haken läuft der SimRobot mit der Simulationsuhr, die "
            "Vorhersagedauer fällt dann nicht ins Gewicht. Mit Haken wirkt "
            "die Latenz wie an der Anlage.")
        self.realtime.toggled.connect(self._update_command)
        layout.addWidget(self.realtime)

        self.no_ensemble = QCheckBox("Chunks nicht mitteln (Vergleichslauf)")
        self.no_ensemble.toggled.connect(self._update_command)
        layout.addWidget(self.no_ensemble)

        row = QHBoxLayout()
        row.addWidget(QLabel("Protokoll"))
        self.out = QLineEdit()
        self.out.setPlaceholderText("Default: <Modell>/rollouts/<Zeit>")
        self.out.textChanged.connect(self._update_command)
        row.addWidget(self.out, 1)
        layout.addLayout(row)

        return box

    def _build_action_bar(self):
        """Pruefung und Knoepfe ausserhalb des Bildlaufs.

        Ob Modell, Kameras und Tempo zusammenpassen, ist die Frage, die
        unmittelbar vor dem Klick zaehlt -- sie darf nicht unter der
        Bildschirmkante stehen. Der Stoppknopf erst recht nicht.
        """
        bar = QWidget()
        outer = QVBoxLayout(bar)
        outer.setContentsMargins(0, 4, 0, 0)
        self.check_label = QLabel("")
        self.check_label.setWordWrap(True)
        outer.addWidget(self.check_label)
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        self.btn_start = QPushButton("Inferenz starten")
        self.btn_stop = QPushButton("STOPP")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setToolTip(
            "Beendet den laufenden Prozess. Software-Stopp — ersetzt keinen "
            "zertifizierten Not-Aus (AP 4.2).")
        self.btn_stop.setStyleSheet(
            "QPushButton:enabled{background:%s; color:white; font-weight:bold}" % BAD)
        self.btn_start.clicked.connect(self.start_inference)
        self.btn_stop.clicked.connect(self.stop)
        layout.addWidget(self.btn_start)
        layout.addWidget(self.btn_stop)
        layout.addStretch(1)
        outer.addWidget(row)
        return bar

    # -- Aufbau: Ausgabeseite ----------------------------------------------

    def _build_output(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        command_box = QGroupBox("Kommandozeile")
        command_layout = QVBoxLayout(command_box)
        self.command = QLabel("")
        self.command.setWordWrap(True)
        self.command.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        self.command.setStyleSheet("font-family:Consolas; color:#333")
        command_layout.addWidget(self.command)
        layout.addWidget(command_box)

        progress_box = QGroupBox("Fahrt")
        progress_layout = QVBoxLayout(progress_box)
        self.headline = QLabel("noch nichts gestartet")
        headline_font = QFont()
        headline_font.setBold(True)
        self.headline.setFont(headline_font)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.detail_line = _hint("")
        self.totals_line = _hint("")
        progress_layout.addWidget(self.headline)
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.detail_line)
        progress_layout.addWidget(self.totals_line)

        self.rollouts = QTreeWidget()
        self.rollouts.setHeaderLabels(
            ["Fahrt", "Ergebnis", "Takte", "Greifpunkt", "Ende", "Bahn p95",
             "Vorhersage", "Überläufe", "Abbruchgrund"])
        self.rollouts.setRootIsDecorated(False)
        self.rollouts.setMaximumHeight(170)
        for column, width in enumerate((55, 90, 60, 85, 70, 80, 85, 75, 130)):
            self.rollouts.setColumnWidth(column, width)
        progress_layout.addWidget(self.rollouts)
        layout.addWidget(progress_box)

        output_box = QGroupBox("Ausgabe")
        output_layout = QVBoxLayout(output_box)
        self.output = LogView()
        output_layout.addWidget(self.output)
        layout.addWidget(output_box, 1)
        return page

    # -- Aktualisieren ------------------------------------------------------

    def refresh(self):
        super().refresh()
        self._reload_checkpoints()
        self._reload_devices()
        self.cameras.refresh(
            backend_is_sim_robot=self.session.backend == session_module.BACKEND_SIM)
        self._update_command()

    def _reload_checkpoints(self):
        current = self.checkpoint.currentData()
        self.checkpoint.blockSignals(True)
        self.checkpoint.clear()
        self.checkpoint.addItem("— keines —", None)
        for path in operation_module.find_policies(self.workdir):
            self.checkpoint.addItem(str(path), str(path))
        index = find_data(self.checkpoint, current)
        self.checkpoint.setCurrentIndex(index if index >= 0 else 0)
        self.checkpoint.blockSignals(False)
        self._on_checkpoint_changed()

    def _reload_devices(self):
        current = self.device.currentData()
        self.device.blockSignals(True)
        self.device.clear()
        for device in training_module.devices():
            self.device.addItem(device.label, device.key)
        index = find_data(self.device, current)
        self.device.setCurrentIndex(index if index >= 0 else 0)
        self.device.blockSignals(False)

    def _on_session_status(self, _status):
        """Ampel oder Override haben sich geaendert -- nachziehen."""
        if abs(self.override.value() - self.session.override) > 1e-12:
            self.override.blockSignals(True)
            self.override.setValue(float(self.session.override))
            self.override.blockSignals(False)
        self._update_command()

    def _on_checkpoint_changed(self):
        path = self.checkpoint.currentData()
        if not path:
            self._described = None
            self.model_info.setText("")
            self._update_command()
            return
        self._described = operation_module.describe_policy(path)
        self.model_info.setText(operation_module.policy_summary(self._described))
        # Vorbelegung aus dem Modell -- die Werte der Aufzeichnung sind die,
        # die passen muessen. Wer abweichen will, tut das bewusst; die
        # Vorabpruefung sagt es dann.
        if self._described["override"] is not None:
            self.session.set_override(float(self._described["override"]))
        self._update_command()

    def _on_hold_toggled(self, checked):
        self.checkpoint.setEnabled(not checked)
        self._update_command()

    def _on_cameras_changed(self):
        self._update_command()

    # -- Pruefung und Kommandozeile ----------------------------------------

    def _robot_kind(self):
        return ("sim" if self.session.backend == session_module.BACKEND_SIM
                else "neura")

    def _assignment(self):
        return self.cameras.assignment()

    def _update_command(self):
        status = self.session.status()
        assignment = self._assignment()
        try:
            argv = operation_module.infer_argv(
                self.checkpoint.currentData(),
                robot_kind=self._robot_kind(),
                episodes=self.episodes.value(),
                camera_specs=cameras_module.to_specs(assignment),
                override=(self.session.override
                          if self._robot_kind() == "neura" else None),
                max_steps=self.max_steps.value() or None,
                replan=self.replan.value() or None,
                device=self.device.currentData(),
                realtime=self.realtime.isChecked(),
                hold=self.hold.isChecked(),
                no_ensemble=self.no_ensemble.isChecked(),
                out=self.out.text().strip() or None,
                real_robot=status.is_real_plant)
            self.command.setText(training_module.command_line(argv))
        except ValueError as exc:
            self.command.setText("Betrieb: %s" % exc)

        self._problems = []
        lines = []
        if self._described is not None and not self.hold.isChecked():
            self._problems = operation_module.compatibility(
                self._described, assignment, self._robot_kind(),
                self.session.override if self._robot_kind() == "neura" else None,
                is_real_plant=status.is_real_plant)
        for problem in self._problems:
            lines.append(("✗ " if problem.blocking else "! ") + problem.text)
        for warning in cameras_module.assignment_warnings(
                assignment, self.cameras.inventory, status.is_real_plant):
            lines.append("! " + warning)
        if not lines:
            self.check_label.setText(
                "Modell, Kameras und Tempo passen zusammen."
                if self._described is not None or self.hold.isChecked()
                else "Kein Modell gewählt.")
            self.check_label.setStyleSheet("color:%s" % NEUTRAL)
        else:
            self.check_label.setText("\n".join(lines))
            self.check_label.setStyleSheet(
                "color:%s" % (BAD if operation_module.blocking(self._problems) else WARN))

    # -- Starten ------------------------------------------------------------

    def start_inference(self):
        if self.process.running:
            QMessageBox.information(self, "Läuft bereits",
                                    "Es läuft schon eine Fahrt.")
            return
        if not self.hold.isChecked() and not self.checkpoint.currentData():
            QMessageBox.information(
                self, "Kein Modell",
                "Ein Modell wählen — oder \"Ohne Modell fahren\" für den "
                "Verdrahtungstest ankreuzen.")
            return

        assignment = self._assignment()
        keys = ["torch", "lerobot"] if not self.hold.isChecked() else []
        keys += cameras_module.required_backend_keys(assignment)
        if self._robot_kind() == "neura":
            keys.append("neurapy")
        if not self.hold.isChecked():
            keys.append("cuda")
        if not self.ensure_requirements(
                "Eine Policy-Fahrt", keys,
                "Ohne GPU braucht eine Vorhersage 384 ms und reißt den "
                "15-Hz-Takt — in der Simulation zum Nachsehen trotzdem "
                "brauchbar, an der Anlage nicht."):
            return

        blocking = operation_module.blocking(self._problems)
        if blocking and not self.confirm(
                "Modell passt nicht zur Fahrt",
                "apps/infer.py bricht dabei ab:\n\n"
                + "\n".join("  - " + p.text for p in blocking)
                + "\n\nTrotzdem starten?"):
            return

        if not self._confirm_motion():
            return

        self._progress = operation_module.RolloutProgress(
            total_episodes=self.episodes.value())
        self.rollouts.clear()
        self.output.clear()
        self.totals_line.setText("")
        self.detail_line.setText("")
        self.progress_bar.setValue(0)
        self.headline.setText("startet …")
        self.headline.setStyleSheet("font-weight:bold")

        status = self.session.status()
        argv = operation_module.infer_argv(
            self.checkpoint.currentData(),
            robot_kind=self._robot_kind(),
            episodes=self.episodes.value(),
            camera_specs=cameras_module.to_specs(assignment),
            override=(self.session.override
                      if self._robot_kind() == "neura" else None),
            max_steps=self.max_steps.value() or None,
            replan=self.replan.value() or None,
            device=self.device.currentData(),
            realtime=self.realtime.isChecked(),
            hold=self.hold.isChecked(),
            no_ensemble=self.no_ensemble.isChecked(),
            out=self.out.text().strip() or None,
            real_robot=status.is_real_plant)
        try:
            # apps/infer.py fragt bei --real-robot nach dem Freigabewort und
            # nach einem Stopp vor jeder weiteren Fahrt. Die Rueckfragen hat
            # die GUI schon gestellt; das Wort geht deshalb mit, der Rest
            # laeuft auf EOF in den sicheren Zweig (Fahrt nicht fortsetzen).
            self.process.start(argv, stdin_text="ANLAGE\n"
                               if status.is_real_plant else None)
        except Exception as exc:
            QMessageBox.warning(self, "Start gescheitert", str(exc))
            return
        self._set_running(True)

    def _confirm_motion(self):
        """Rueckfrage vor jeder Bewegung -- die Antwort der Steuerung zaehlt."""
        status = self.session.status()
        lines = ["Die Policy bewegt den Roboter.", ""]
        lines.append("Backend laut Steuerung: %s" % status.text)
        lines.append("Kameras: %s" % cameras_module.describe(self._assignment()))
        lines.append("Fahrten: %d" % self.episodes.value())
        if status.state == session_module.OFFLINE:
            lines += ["", "Es ist nichts verbunden — apps/infer.py verbindet "
                          "selbst. Welche Steuerung antwortet, steht erst dann fest."]
        if status.is_real_plant:
            lines += ["", "ACHTUNG: Das ist die REALE ANLAGE, nicht die Simulation.",
                      "Die Fahrt wird mit --real-robot gestartet."]
        lines += ["", "Arbeitsraum frei? Not-Aus in Reichweite?"]
        return self.confirm("Policy fährt", "\n".join(lines))

    def stop(self):
        if not self.process.running:
            return
        self.process.stop()
        self.output.append_record(
            "STOPP ausgelöst — Prozess beendet. Das ist ein Software-Stopp "
            "und ersetzt keinen Not-Aus (AP 4.2).", logging.WARNING)

    # -- Ausgabe ------------------------------------------------------------

    def _on_started(self, command):
        self.output.append_record("$ " + " ".join(command[1:]), logging.INFO)

    def _on_line(self, text):
        self.output.append_line(text)
        if self._progress is None or not self._progress.feed(text):
            return
        before = self.rollouts.topLevelItemCount()
        for rollout in self._progress.rollouts[before:]:
            item = QTreeWidgetItem(self.rollouts, rollout.row())
            item.setForeground(1, QBrush(QColor(GOOD if rollout.success else WARN)))
            self.rollouts.addTopLevelItem(item)
            self.rollouts.scrollToItem(item)
        self.headline.setText(self._progress.headline())
        self.detail_line.setText(self._progress.detail_line())
        self.totals_line.setText(self._progress.totals_line())
        percent = self._progress.percent
        if percent is not None:
            self.progress_bar.setValue(percent)

    def _on_finished(self, code, text):
        self._set_running(False)
        ok = code == 0
        if ok and self._progress is not None and self._progress.total is not None:
            succeeded = self._progress.succeeded
            total = self._progress.total
            self.headline.setText("Erfolg %d von %d Fahrten" % (succeeded, total))
            self.headline.setStyleSheet(
                "color:%s; font-weight:bold" % (GOOD if succeeded == total else WARN))
        elif ok:
            self.headline.setText("Durchgelaufen")
            self.headline.setStyleSheet("color:%s; font-weight:bold" % GOOD)
        else:
            self.headline.setText("Abgebrochen (Rückgabewert %d)" % code)
            self.headline.setStyleSheet("color:%s; font-weight:bold" % BAD)
        self.output.append_record(
            "-> Fahrt beendet, Rückgabewert %d" % code,
            logging.INFO if ok else logging.ERROR)

    def _set_running(self, running):
        for widget in (self.btn_start, self.btn_rescan, self.checkpoint,
                       self.hold, self.episodes, self.cameras):
            widget.setEnabled(not running)
        self.btn_stop.setEnabled(running)

    # -- Lebenszyklus -------------------------------------------------------

    def shutdown(self):
        if self.process.running:
            self.process.stop()


def _hint(text):
    label = QLabel(text)
    label.setWordWrap(True)
    label.setStyleSheet("color:%s" % NEUTRAL)
    return label

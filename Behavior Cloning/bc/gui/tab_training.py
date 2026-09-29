"""Modus "Training": Datensatz exportieren und Policy trainieren.

Der Reiter fuehrt durch die Kette, die in der Dokumentation steht:

    1. Datensatz    Aufnahmen auswaehlen, pruefen, exportieren -- oder einen
                    fertigen LeRobotDataset-Ordner nehmen (auch einen von
                    einem anderen Rechner; der Ordner ist portabel).
    2. Hardware     Geraet waehlen. Was da ist, sagt torch zur Laufzeit.
    3. Einstellung  Hyperparameter; Abweichungen von bc/config.py und den
                    Vorgaben aus apps/train.py werden benannt, nicht versteckt.
    4. Lauf         Training mit Fortschritt, Validierungsfehler in rad,
                    jederzeit abbrechbar.

Ausgefuehrt wird ueber Unterprozesse -- dieselbe Kommandozeile, die auch im
Terminal gilt. Sie steht rechts oben zum Mitlesen und Kopieren; Begruendung
in training.py.

Der Aufnahme-Laptop hat kein CUDA. Der Reiter bleibt dort trotzdem
vollstaendig bedienbar: Export geht, Training wird mit Ansage verweigert
bzw. auf die CPU-Warnung hingewiesen (Festlegung Anwender, 2026-09-29).
"""

import logging
import shutil
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QBrush, QColor, QFont
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox,
                               QDoubleSpinBox, QFileDialog, QFormLayout,
                               QGroupBox, QHBoxLayout, QLabel, QLineEdit,
                               QListWidget, QListWidgetItem, QMessageBox,
                               QProgressBar, QPushButton, QScrollArea,
                               QSpinBox, QSplitter, QTreeWidget, QTreeWidgetItem,
                               QVBoxLayout, QWidget)

from . import _bootstrap
from . import training as training_module
from .runner import ProcessRunner
from .tab_base import ModeTab
from .widgets import BAD, GOOD, LogView, NEUTRAL, WARN, block_wheel, find_data

log = logging.getLogger(__name__)

#: Was gerade laeuft -- bestimmt, was die Ausgabe bedeutet.
IDLE = "idle"
CHECKING = "checking"
EXPORTING = "exporting"
TRAINING = "training"


class TrainingTab(ModeTab):
    mode = "training"
    title = "Training"

    def __init__(self, session, runner, parent=None):
        super().__init__(session, runner, parent)
        self.workdir = Path(_bootstrap.WORKDIR)
        self.process = ProcessRunner(self)
        self.process.line.connect(self._on_line)
        self.process.finished.connect(self._on_finished)
        self.process.started.connect(self._on_started)
        self._activity = IDLE
        self._progress = None
        self._params = training_module.parameters()
        self._param_widgets = {}
        self._devices = []
        #: Datensatzordner, der nicht unter datasets/ liegt (von woanders).
        self._external_dataset = None

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

        self.refresh()

    # -- Aufbau: Bedienseite ------------------------------------------------

    def _build_controls(self):
        """Bedienseite: scrollbare Abschnitte, darunter eine feste Knopfleiste.

        Die Knoepfe, die etwas ausloesen, stehen ABSICHTLICH ausserhalb des
        Bildlaufs. Auf einem kleinen Laptopbildschirm waere "Training
        starten" sonst unter der Kante -- und "Abbrechen" auch, was beim
        Abbrechen der schlechtere Ort ist.
        """
        page = QWidget()
        column = QVBoxLayout(page)
        column.setContentsMargins(0, 0, 0, 0)
        column.addWidget(self._build_data_box())
        column.addWidget(self._build_hardware_box())
        column.addWidget(self._build_parameter_box())
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

    def _build_action_bar(self):
        bar = QWidget()
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(0, 4, 0, 0)
        self.btn_check = QPushButton("Nur prüfen")
        self.btn_check.setToolTip(
            "Konsistenzprüfung der markierten Aufnahmen: Schema, Rate, "
            "Override, Planer-Tempo, Kameras. Schreibt nichts.")
        self.btn_export = QPushButton("Exportieren")
        self.btn_train = QPushButton("Training starten")
        self.btn_cancel = QPushButton("Abbrechen")
        self.btn_cancel.setEnabled(False)
        self.btn_check.clicked.connect(self.check_sources)
        self.btn_export.clicked.connect(self.export_dataset)
        self.btn_train.clicked.connect(self.start_training)
        self.btn_cancel.clicked.connect(self.cancel)
        for button in (self.btn_check, self.btn_export, self.btn_train,
                       self.btn_cancel):
            layout.addWidget(button)
        layout.addStretch(1)
        return bar

    def _build_data_box(self):
        box = QGroupBox("1. Datensatz")
        layout = QVBoxLayout(box)

        layout.addWidget(_hint("Aufnahmen auswählen und exportieren — oder unten "
                               "einen fertigen Datensatz nehmen."))
        self.recordings = QListWidget()
        self.recordings.setSelectionMode(
            QAbstractItemView.SelectionMode.ExtendedSelection)
        self.recordings.setMinimumHeight(130)
        self.recordings.itemSelectionChanged.connect(self._on_sources_changed)
        layout.addWidget(self.recordings)

        self.require_success = QCheckBox("nur Episoden mit Erfolgs-Label")
        self.require_success.setToolTip(
            "Ohne Haken kommen auch unbewertete Episoden in den Datensatz "
            "(AP 5.2).")
        self.require_success.toggled.connect(self._update_commands)
        layout.addWidget(self.require_success)

        target = QHBoxLayout()
        target.addWidget(QLabel("Ziel"))
        self.export_target = QLineEdit()
        self.export_target.textChanged.connect(self._update_commands)
        target.addWidget(self.export_target, 1)
        layout.addLayout(target)

        layout.addSpacing(6)
        row = QHBoxLayout()
        row.addWidget(QLabel("Datensatz"))
        self.dataset = block_wheel(QComboBox())
        self.dataset.currentIndexChanged.connect(self._on_dataset_changed)
        row.addWidget(self.dataset, 1)
        self.btn_import = QPushButton("Ordner…")
        self.btn_import.setToolTip(
            "Einen Datensatz von einem anderen Rechner einbinden. Der "
            "Ordner wird nicht kopiert, sondern direkt benutzt (AP 3.1).")
        self.btn_import.clicked.connect(self.import_dataset)
        row.addWidget(self.btn_import)
        layout.addLayout(row)

        self.dataset_info = _hint("")
        layout.addWidget(self.dataset_info)
        return box

    def _build_hardware_box(self):
        box = QGroupBox("2. Hardware")
        layout = QVBoxLayout(box)
        row = QHBoxLayout()
        row.addWidget(QLabel("Gerät"))
        self.device = block_wheel(QComboBox())
        self.device.currentIndexChanged.connect(self._on_device_changed)
        row.addWidget(self.device, 1)
        layout.addLayout(row)
        self.device_note = _hint("")
        layout.addWidget(self.device_note)
        return box

    def _build_parameter_box(self):
        """Zwei Spalten: links, was man zwischen Laeufen dreht, rechts die
        Festlegungen aus bc/config.py.

        Zweispaltig, damit der Startknopf darunter ohne Scrollen erreichbar
        bleibt -- am Labortag soll niemand erst suchen muessen. Die Trennung
        ist zugleich die Aussage: rechts steht nichts, was man nebenbei
        aendert.
        """
        box = QGroupBox("3. Einstellungen")
        layout = QVBoxLayout(box)

        columns = QHBoxLayout()
        free_form = QFormLayout()
        settled_form = QFormLayout()
        for form in (free_form, settled_form):
            form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        for param in self._params:
            widget = self._parameter_widget(param)
            self._param_widgets[param.flag] = widget
            label = QLabel(param.label)
            if param.help_text:
                widget.setToolTip(param.help_text)
            if param.is_locked_down:
                label.setStyleSheet("color:%s" % NEUTRAL)
                label.setToolTip("Festlegung aus bc/config.py — nicht "
                                 "stillschweigend ändern (Arbeitsanweisung 5).")
                settled_form.addRow(label, widget)
            else:
                free_form.addRow(label, widget)
        left = QVBoxLayout()
        left.addLayout(free_form)
        left.addStretch(1)
        right = QVBoxLayout()
        right.addWidget(_hint("Festlegungen aus bc/config.py"))
        right.addLayout(settled_form)
        right.addStretch(1)
        columns.addLayout(left, 1)
        columns.addLayout(right, 1)
        layout.addLayout(columns)

        self.deviation_label = QLabel("")
        self.deviation_label.setWordWrap(True)
        layout.addWidget(self.deviation_label)

        self.btn_defaults = QPushButton("Vorgaben wiederherstellen")
        self.btn_defaults.clicked.connect(self.reset_parameters)
        layout.addWidget(self.btn_defaults, 0, Qt.AlignmentFlag.AlignLeft)
        return box

    def _parameter_widget(self, param):
        if param.kind == "choice":
            widget = block_wheel(QComboBox())
            for choice in param.choices:
                widget.addItem(choice)
            widget.setCurrentText(str(param.default))
            widget.currentIndexChanged.connect(self._update_commands)
            return widget
        if param.kind == "float":
            widget = block_wheel(QDoubleSpinBox())
            widget.setDecimals(6)
            widget.setRange(float(param.minimum), float(param.maximum))
            widget.setSingleStep(float(param.step))
            widget.setValue(float(param.default))
            widget.valueChanged.connect(self._update_commands)
            return widget
        widget = block_wheel(QSpinBox())
        widget.setRange(int(param.minimum), int(param.maximum))
        widget.setSingleStep(int(param.step))
        widget.setValue(int(param.default))
        widget.valueChanged.connect(self._update_commands)
        return widget

    def _build_run_box(self):
        box = QGroupBox("4. Lauf")
        layout = QVBoxLayout(box)

        row = QHBoxLayout()
        row.addWidget(QLabel("Ziel"))
        self.train_target = QLineEdit()
        self.train_target.textChanged.connect(self._update_commands)
        row.addWidget(self.train_target, 1)
        layout.addLayout(row)

        options = QHBoxLayout()
        self.resume = QCheckBox("fortsetzen")
        self.resume.setToolTip(
            "Am letzten Checkpoint mit training_state.pt weitermachen.")
        self.resume.toggled.connect(self._update_commands)
        options.addWidget(self.resume)
        options.addWidget(QLabel("Zeitbudget (h)"))
        self.max_hours = block_wheel(QDoubleSpinBox())
        self.max_hours.setRange(0.0, 240.0)
        self.max_hours.setSingleStep(0.5)
        self.max_hours.setDecimals(1)
        self.max_hours.setSpecialValueText("kein")
        self.max_hours.setToolTip(
            "Nach dieser Zeit wird regulär gespeichert und beendet. 0 = kein Budget.")
        self.max_hours.valueChanged.connect(self._update_commands)
        options.addWidget(self.max_hours)
        options.addStretch(1)
        layout.addLayout(options)

        layout.addSpacing(6)
        row = QHBoxLayout()
        row.addWidget(QLabel("Lauf"))
        self.checkpoint = block_wheel(QComboBox())
        self.checkpoint.currentIndexChanged.connect(self._on_checkpoint_changed)
        row.addWidget(self.checkpoint, 1)
        self.btn_copy_run = QPushButton("Kopieren…")
        self.btn_copy_run.setToolTip(
            "Den Lauf auf einen Stick oder einen anderen Rechner kopieren "
            "(Checkpoints sind geräteneutral, AP 3.1).")
        self.btn_copy_run.clicked.connect(self.copy_run)
        row.addWidget(self.btn_copy_run)
        layout.addLayout(row)

        self.checkpoint_info = _hint("")
        layout.addWidget(self.checkpoint_info)
        return box

    # -- Aufbau: Ausgabeseite ----------------------------------------------

    def _build_output(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        command_box = QGroupBox("Kommandozeile")
        command_layout = QVBoxLayout(command_box)
        self.command_export = _command_label()
        self.command_train = _command_label()
        command_layout.addWidget(self.command_export)
        command_layout.addWidget(self.command_train)
        layout.addWidget(command_box)

        progress_box = QGroupBox("Fortschritt")
        progress_layout = QVBoxLayout(progress_box)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.headline = QLabel("noch nichts gestartet")
        headline_font = QFont()
        headline_font.setBold(True)
        self.headline.setFont(headline_font)
        self.dataset_line = _hint("")
        self.hardware_line = _hint("")
        progress_layout.addWidget(self.headline)
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.dataset_line)
        progress_layout.addWidget(self.hardware_line)

        self.validation = QTreeWidget()
        self.validation.setHeaderLabels(
            ["Schritt", "Gelenk-MAE (rad)", "p95 (rad)", "Greifer", "Loss"])
        self.validation.setRootIsDecorated(False)
        self.validation.setMaximumHeight(150)
        for column in range(5):
            self.validation.setColumnWidth(column, 120)
        progress_layout.addWidget(QLabel("Validierung"))
        progress_layout.addWidget(self.validation)
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
        self._reload_recordings()
        self._reload_datasets()
        self._reload_devices()
        self._reload_checkpoints()
        self._update_commands()

    def _reload_recordings(self):
        selected = set(self._selected_sources())
        self.recordings.clear()
        for path in training_module.find_recordings(self.workdir):
            described = training_module.describe_recording(path)
            item = QListWidgetItem(training_module.recording_label(described))
            item.setData(Qt.ItemDataRole.UserRole, str(path))
            if described.get("error"):
                item.setForeground(QBrush(QColor(BAD)))
            elif described["schema_version"] != training_module.config.SCHEMA_VERSION:
                item.setForeground(QBrush(QColor(WARN)))
            self.recordings.addItem(item)
            if str(path) in selected:
                item.setSelected(True)

    def _reload_datasets(self):
        current = self.dataset.currentData()
        self.dataset.blockSignals(True)
        self.dataset.clear()
        self.dataset.addItem("— keiner —", None)
        for path in training_module.find_datasets(self.workdir):
            self.dataset.addItem(str(path), str(path))
        if self._external_dataset:
            self.dataset.addItem(str(self._external_dataset), str(self._external_dataset))
        index = find_data(self.dataset, current)
        self.dataset.setCurrentIndex(index if index >= 0 else 0)
        self.dataset.blockSignals(False)
        self._on_dataset_changed()

    def _reload_devices(self):
        current = self.device.currentData()
        self._devices = training_module.devices()
        self.device.blockSignals(True)
        self.device.clear()
        for device in self._devices:
            self.device.addItem(device.label, device.key)
        index = find_data(self.device, current)
        self.device.setCurrentIndex(index if index >= 0 else 0)
        self.device.blockSignals(False)
        self._on_device_changed()

    def _reload_checkpoints(self):
        current = self.checkpoint.currentData()
        self.checkpoint.blockSignals(True)
        self.checkpoint.clear()
        self.checkpoint.addItem("— keiner —", None)
        for path in training_module.find_checkpoints(self.workdir):
            self.checkpoint.addItem(str(path), str(path))
        index = find_data(self.checkpoint, current)
        self.checkpoint.setCurrentIndex(index if index >= 0 else 0)
        self.checkpoint.blockSignals(False)
        self._on_checkpoint_changed()

    # -- Auswahl ------------------------------------------------------------

    def _selected_sources(self):
        return [item.data(Qt.ItemDataRole.UserRole)
                for item in self.recordings.selectedItems()]

    def _on_sources_changed(self):
        sources = self._selected_sources()
        if sources and not self.export_target.text().strip():
            self.export_target.setText(
                str(training_module.suggest_dataset_name(self.workdir, sources)))
        self._update_commands()

    def _on_dataset_changed(self):
        path = self.dataset.currentData()
        if not path:
            self.dataset_info.setText("")
            self._update_commands()
            return
        described = training_module.describe_dataset(path)
        bits = []
        if described["episodes"] is not None:
            bits.append("%s Episoden" % described["episodes"])
        if described["frames"] is not None:
            bits.append("%s Frames" % described["frames"])
        if described["cameras"]:
            bits.append("Kameras: %s" % ", ".join(described["cameras"]))
        if described["fps"] is not None:
            bits.append("%g Hz" % float(described["fps"]))
        if described["lerobot_version"]:
            bits.append("lerobot %s" % described["lerobot_version"])
        text = "  ·  ".join(bits)
        if described["warnings"]:
            text += "\n⚠ " + "\n⚠ ".join(described["warnings"])
            self.dataset_info.setStyleSheet("color:%s" % WARN)
        else:
            self.dataset_info.setStyleSheet("color:%s" % NEUTRAL)
        self.dataset_info.setText(text)
        if not self.train_target.text().strip():
            self.train_target.setText(
                str(training_module.suggest_output(self.workdir, path)))
        self._update_commands()

    def _on_device_changed(self):
        key = self.device.currentData()
        for device in self._devices:
            if device.key == key:
                self.device_note.setText(device.note)
                break
        else:
            self.device_note.setText("")
        hint = training_module.hardware_hint(self._devices)
        if not any(d.key.startswith("cuda") or d.key == "mps" for d in self._devices):
            self.device_note.setText(hint)
            self.device_note.setStyleSheet("color:%s" % WARN)
        else:
            self.device_note.setStyleSheet("color:%s" % NEUTRAL)
        self._update_commands()

    def _on_checkpoint_changed(self):
        path = self.checkpoint.currentData()
        if not path:
            self.checkpoint_info.setText("")
            return
        described = training_module.describe_checkpoint(path)
        bits = []
        if described["dataset"]:
            bits.append("Datensatz: %s" % described["dataset"])
        if described["checkpoints"]:
            bits.append("Checkpoints: %s" % ", ".join(described["checkpoints"]))
        bits.append("Policy vorhanden" if described["has_policy"] else "keine policy/")
        if described["resumable"]:
            bits.append("fortsetzbar ab %s" % described["resumable"])
        else:
            bits.append("nicht fortsetzbar (kein training_state.pt)")
        self.checkpoint_info.setText("  ·  ".join(bits))

    def parameter_values(self):
        """Die eingestellten Werte als {Flag: Wert}."""
        values = {}
        for param in self._params:
            widget = self._param_widgets[param.flag]
            if param.kind == "choice":
                values[param.flag] = widget.currentText()
            elif param.kind == "float":
                values[param.flag] = float(widget.value())
            else:
                values[param.flag] = int(widget.value())
        return values

    def reset_parameters(self):
        for param in self._params:
            widget = self._param_widgets[param.flag]
            if param.kind == "choice":
                widget.setCurrentText(str(param.default))
            elif param.kind == "float":
                widget.setValue(float(param.default))
            else:
                widget.setValue(int(param.default))
        self._update_commands()

    # -- Kommandozeilen anzeigen -------------------------------------------

    def _update_commands(self):
        sources = self._selected_sources()
        try:
            argv = training_module.export_argv(
                sources, out=self.export_target.text().strip() or None,
                require_success=self.require_success.isChecked())
            self.command_export.setText(training_module.command_line(argv))
        except ValueError as exc:
            self.command_export.setText("Export: %s" % exc)

        values = self.parameter_values()
        try:
            argv = training_module.train_argv(
                self.dataset.currentData(),
                self.train_target.text().strip() or None,
                device=self.device.currentData(),
                values=values,
                resume=self.resume.isChecked(),
                max_hours=self.max_hours.value() or None,
                params=self._params)
            self.command_train.setText(training_module.command_line(argv))
        except ValueError as exc:
            self.command_train.setText("Training: %s" % exc)

        self._show_deviations(values)
        if self._activity == IDLE:
            self.progress_bar.setMaximum(100)
            self.progress_bar.setValue(0)

    def _show_deviations(self, values):
        deviations = training_module.deviations(values, self._params)
        if not deviations:
            self.deviation_label.setText("Alle Werte auf Vorgabe.")
            self.deviation_label.setStyleSheet("color:%s" % NEUTRAL)
            return
        locked = [d for d in deviations if d[0].is_locked_down]
        lines = ["%d Abweichung(en) von der Vorgabe:" % len(deviations)]
        for param, value in deviations:
            lines.append("  %s: %s statt %s%s" % (
                param.label, _text(value), _text(param.default),
                "   ← Festlegung aus bc/config.py" if param.is_locked_down else ""))
        if locked:
            lines.append("Festlegungen aus bc/config.py werden nicht "
                         "stillschweigend geändert — Schema-Auswirkung prüfen.")
        self.deviation_label.setText("\n".join(lines))
        self.deviation_label.setStyleSheet(
            "color:%s" % (BAD if locked else WARN))

    # -- Starten ------------------------------------------------------------

    def check_sources(self):
        sources = self._selected_sources()
        if not sources:
            QMessageBox.information(self, "Nichts ausgewählt",
                                    "Keine Aufzeichnung markiert.")
            return
        self._run(training_module.export_argv(
            sources, check=True,
            require_success=self.require_success.isChecked()), CHECKING)

    def export_dataset(self):
        sources = self._selected_sources()
        if not sources:
            QMessageBox.information(self, "Nichts ausgewählt",
                                    "Keine Aufzeichnung markiert.")
            return
        target = self.export_target.text().strip()
        if not target:
            QMessageBox.information(self, "Kein Ziel",
                                    "Zielordner für den Datensatz angeben.")
            return
        if not self.ensure_requirements(
                "Der Export", ("lerobot", "ffmpeg"),
                "Die Konsistenzprüfung (\"Nur prüfen\") läuft auch ohne."):
            return
        overwrite = False
        if Path(target).exists():
            if not self.confirm(
                    "Ordner überschreiben",
                    "%s gibt es schon.\n\nInhalt ersetzen?" % target):
                return
            overwrite = True
        self._run(training_module.export_argv(
            sources, out=target,
            require_success=self.require_success.isChecked(),
            overwrite=overwrite), EXPORTING)

    def start_training(self):
        dataset = self.dataset.currentData()
        target = self.train_target.text().strip()
        if not dataset:
            QMessageBox.information(
                self, "Kein Datensatz",
                "Erst einen Datensatz wählen oder Aufnahmen exportieren.")
            return
        if not target:
            QMessageBox.information(self, "Kein Ziel",
                                    "Zielordner für den Lauf angeben.")
            return
        if not self.ensure_requirements(
                "Ein Training", ("torch", "lerobot", "cuda"),
                "Ohne GPU dauert ein Training unverhältnismäßig lang. Der "
                "Datensatz ist portabel — exportieren und auf einem "
                "Trainingsrechner trainieren."):
            return
        values = self.parameter_values()
        locked = [d for d in training_module.deviations(values, self._params)
                  if d[0].is_locked_down]
        if locked and not self.confirm(
                "Festlegung geändert",
                "Diese Werte sind Festlegungen aus bc/config.py:\n\n"
                + "\n".join("  %s: %s statt %s" % (p.label, _text(v), _text(p.default))
                            for p, v in locked)
                + "\n\nEin so trainiertes Modell passt nicht mehr zu den "
                  "Vorgaben der übrigen Kette (Rate, Horizont, Chunk).\n\n"
                  "Trotzdem starten?"):
            return
        if Path(target).exists() and not self.resume.isChecked():
            if not self.confirm(
                    "Ordner existiert",
                    "%s gibt es schon.\n\napps/train.py schreibt hinein. "
                    "Fortsetzen wäre \"fortsetzen\" ankreuzen.\n\nTrotzdem starten?"
                    % target):
                return
        argv = training_module.train_argv(
            dataset, target, device=self.device.currentData(), values=values,
            resume=self.resume.isChecked(),
            max_hours=self.max_hours.value() or None, params=self._params)
        self._progress = training_module.TrainingProgress(
            total_steps=values.get("--steps"))
        self.validation.clear()
        self._run(argv, TRAINING)

    def _run(self, argv, activity):
        if self.process.running:
            QMessageBox.information(
                self, "Läuft bereits",
                "Es läuft schon etwas in diesem Reiter. Erst abbrechen oder "
                "abwarten.")
            return
        self.output.clear()
        self._activity = activity
        if activity != TRAINING:
            self._progress = None
            self.progress_bar.setRange(0, 0)       # unbestimmt
        else:
            self.progress_bar.setRange(0, 100)
            self.progress_bar.setValue(0)
        self.headline.setText({
            CHECKING: "Konsistenzprüfung läuft",
            EXPORTING: "Export läuft — Videokodierung dauert",
            TRAINING: "Training gestartet",
        }[activity])
        self.dataset_line.setText("")
        self.hardware_line.setText("")
        try:
            self.process.start(argv)
        except Exception as exc:
            self._activity = IDLE
            self.progress_bar.setRange(0, 100)
            QMessageBox.warning(self, "Start gescheitert", str(exc))
            return
        self._set_buttons_running(True)

    def cancel(self):
        if not self.process.running:
            return
        if self._activity == TRAINING and not self.confirm(
                "Training abbrechen",
                "Der Lauf wird beendet. Der zuletzt geschriebene Checkpoint "
                "bleibt erhalten, die Schritte seitdem sind verloren.\n\n"
                "Wirklich abbrechen?"):
            return
        self.process.stop()

    # -- Ausgabe ------------------------------------------------------------

    def _on_started(self, command):
        self.output.append_record("$ " + " ".join(command[1:]), logging.INFO)

    def _on_line(self, text):
        self.output.append_line(text)
        if self._progress is None or not self._progress.feed(text):
            return
        self.headline.setText(self._progress.headline())
        self.dataset_line.setText(self._progress.dataset_line())
        self.hardware_line.setText(self._progress.hardware_line())
        percent = self._progress.percent
        if percent is not None:
            self.progress_bar.setValue(percent)
        if "val loss" in text:
            self._append_validation()

    def _append_validation(self):
        progress = self._progress
        item = QTreeWidgetItem(self.validation, [
            str(progress.step),
            "%.4f" % progress.val_mae_rad if progress.val_mae_rad is not None else "",
            "%.4f" % progress.val_p95_rad if progress.val_p95_rad is not None else "",
            "%.1f %%" % progress.val_gripper_pct
            if progress.val_gripper_pct is not None else "",
            "%.4f" % progress.val_loss if progress.val_loss is not None else "",
        ])
        self.validation.addTopLevelItem(item)
        self.validation.scrollToItem(item)

    def _on_finished(self, code, text):
        activity = self._activity
        self._activity = IDLE
        self._set_buttons_running(False)
        self.progress_bar.setRange(0, 100)
        ok = code == 0
        name = {CHECKING: "Konsistenzprüfung", EXPORTING: "Export",
                TRAINING: "Training"}.get(activity, "Lauf")
        if ok:
            self.headline.setText("%s abgeschlossen" % name)
            self.progress_bar.setValue(100)
        else:
            self.headline.setText("%s fehlgeschlagen (Rückgabewert %d)" % (name, code))
            self.progress_bar.setValue(0)
        self.headline.setStyleSheet("color:%s; font-weight:bold" % (GOOD if ok else BAD))
        self.output.append_record("-> %s: %s" % (
            name, "durchgelaufen" if ok else "Rückgabewert %d" % code),
            logging.INFO if ok else logging.ERROR)

        if activity == EXPORTING and ok:
            self._reload_datasets()
            index = find_data(self.dataset, self.export_target.text().strip())
            if index >= 0:
                self.dataset.setCurrentIndex(index)
        if activity == TRAINING:
            self._reload_checkpoints()
            index = find_data(self.checkpoint, self.train_target.text().strip())
            if index >= 0:
                self.checkpoint.setCurrentIndex(index)

    def _set_buttons_running(self, running):
        for button in (self.btn_check, self.btn_export, self.btn_train,
                       self.btn_import, self.btn_copy_run):
            button.setEnabled(not running)
        self.btn_cancel.setEnabled(running)

    # -- Ordner ein- und ausbinden -----------------------------------------

    def import_dataset(self):
        """Einen Datensatz von woanders einbinden -- ohne zu kopieren.

        Der Ordner ist portabel (AP 3.1); apps/train.py bekommt einfach
        dessen Pfad. Kopiert wird nichts, sonst laegen zwei Staende
        desselben Datensatzes herum.
        """
        folder = QFileDialog.getExistingDirectory(
            self, "LeRobotDataset-Ordner wählen", str(self.workdir / "datasets"))
        if not folder:
            return
        path = Path(folder)
        if not (path / "meta" / "info.json").is_file():
            QMessageBox.warning(
                self, "Kein Datensatz",
                "%s enthält keine meta/info.json — das ist kein "
                "LeRobotDataset." % path)
            return
        described = training_module.describe_dataset(path)
        if described["warnings"] and not self.confirm(
                "Datensatz mit Befunden",
                "\n".join(described["warnings"]) + "\n\nTrotzdem einbinden?"):
            return
        self._external_dataset = path
        self._reload_datasets()
        index = find_data(self.dataset, str(path))
        if index >= 0:
            self.dataset.setCurrentIndex(index)

    def copy_run(self):
        """Einen Trainingslauf auf einen anderen Datentraeger kopieren.

        Laeuft im "io"-Pool des TaskRunner -- ein Lauf hat mehrere hundert
        Megabyte, im GUI-Thread waere das Fenster solange eingefroren.
        """
        source = self.checkpoint.currentData()
        if not source:
            QMessageBox.information(self, "Kein Lauf",
                                    "Erst einen Lauf auswählen.")
            return
        folder = QFileDialog.getExistingDirectory(self, "Zielordner wählen")
        if not folder:
            return
        target = Path(folder) / Path(source).name
        if target.exists():
            QMessageBox.warning(self, "Ziel existiert",
                                "%s gibt es schon." % target)
            return
        self.output.append_record("Kopiere %s -> %s" % (source, target), logging.INFO)
        self._set_buttons_running(True)
        self.progress_bar.setRange(0, 0)
        self.headline.setText("Kopiere Lauf …")
        self.runner.submit(
            lambda: shutil.copytree(source, target),
            on_done=lambda _: self._copy_done(target, None),
            on_error=lambda exc: self._copy_done(target, exc),
            pool="io")

    def _copy_done(self, target, error):
        self._set_buttons_running(False)
        self.progress_bar.setRange(0, 100)
        if error is None:
            self.headline.setText("Lauf kopiert")
            self.headline.setStyleSheet("color:%s; font-weight:bold" % GOOD)
            self.output.append_record("-> %s" % target, logging.INFO)
        else:
            self.headline.setText("Kopieren fehlgeschlagen")
            self.headline.setStyleSheet("color:%s; font-weight:bold" % BAD)
            self.output.append_record("Kopieren gescheitert: %s" % error, logging.ERROR)

    # -- Lebenszyklus -------------------------------------------------------

    def shutdown(self):
        if self.process.running:
            self.process.stop()


def _hint(text):
    label = QLabel(text)
    label.setWordWrap(True)
    label.setStyleSheet("color:%s" % NEUTRAL)
    return label


def _command_label():
    label = QLabel("")
    label.setWordWrap(True)
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    label.setStyleSheet("font-family:Consolas; color:#333")
    return label


def _text(value):
    if isinstance(value, float):
        return "%g" % value
    return str(value)

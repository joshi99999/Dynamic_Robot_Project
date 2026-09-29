"""Modus "Aufnahme": Kameras einrichten, Punkte teachen, Episoden aufzeichnen.

Zwei Zustaende, und der Unterschied ist keine Geschmacksfrage: Eine Kamera
laesst sich nicht zweimal oeffnen.

    VORSCHAU    Die Oberflaeche oeffnet die Kameras selbst und zeigt Bild,
                Bildrate und Zeitversatz gegen das 30-ms-Budget. Das ist
                der Zustand zum Einrichten -- Zuordnung pruefen, Kamera
                ausrichten, Rate messen.
    AUFNAHME    Die Vorschau wird geschlossen, ``apps/record.py`` uebernimmt
                die Kameras. Angezeigt werden Takt, Episode, Versatz.

Wer waehrend der Aufnahme trotzdem ein Bild sehen will, schaltet
"Livebild auch waehrend der Aufnahme" dazu (Wunsch Anwender, 2026-09-29:
bei Bedarf einschaltbar, sonst aus). Der Unterprozess legt dann in grossen
Abstaenden ein JPEG ab (bc/preview.py) -- das kostet Zeit in der
15-Hz-Schleife, deshalb ist es nicht die Voreinstellung.

Offen und bewusst nicht gebaut: der passive Modus (Aufzeichnung laeuft mit,
waehrend das klassische Team greift). Der braucht zuerst die
Abschnittserkennung -- es sollen nur die richtigen Abschnitte in den
Datensatz -- und die ist dieselbe Grenze wie die Uebergabe bei PRE_PLACE.
Beides erst nach Absprache mit der anderen Gruppe.
"""

import logging
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox,
                               QDoubleSpinBox, QFormLayout, QGroupBox,
                               QHBoxLayout, QHeaderView, QLabel, QLineEdit,
                               QMessageBox, QProgressBar, QPushButton,
                               QScrollArea, QSpinBox, QSplitter, QTableWidget,
                               QTreeWidget, QTreeWidgetItem, QVBoxLayout,
                               QWidget)

from . import _bootstrap
from . import cameras as cameras_module
from . import recording as recording_module
from . import session as session_module
from . import training as training_module
from .. import config
from .runner import ProcessRunner
from .tab_base import ModeTab
from .widgets import (BAD, GOOD, ImageView, LogView, NEUTRAL, WARN,
                      CameraAssignmentView, block_wheel, find_data)

log = logging.getLogger(__name__)

#: Wie oft die Vorschau das Bild aus den Kameras holt (ms).
PREVIEW_INTERVAL_MS = 100
#: Wie oft das Livebild einer laufenden Aufnahme von der Platte gelesen wird.
LIVE_INTERVAL_MS = 250


class RecordingTab(ModeTab):
    mode = "recording"
    title = "Aufnahme"

    def __init__(self, session, runner, parent=None):
        super().__init__(session, runner, parent)
        self.workdir = Path(_bootstrap.WORKDIR)
        self.process = ProcessRunner(self)
        self.process.line.connect(self._on_line)
        self.process.finished.connect(self._on_finished)
        self.process.started.connect(self._on_started)
        self._progress = None
        self._captures = None
        self._live_path = None
        self._rates = RateMeter()
        self._sequence_source = None
        self._sequence_working = None
        self._sequence_name = ""
        self._gripper_start = "open"
        self._steps_dirty = False
        self._loading_steps = False

        self.preview_timer = QTimer(self)
        self.preview_timer.setInterval(PREVIEW_INTERVAL_MS)
        self.preview_timer.timeout.connect(self._tick_preview)
        self.live_timer = QTimer(self)
        self.live_timer.setInterval(LIVE_INTERVAL_MS)
        self.live_timer.timeout.connect(self._tick_live)

        left = self._build_controls()
        right = self._build_output()

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([540, 800])

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
        column.addWidget(self._build_camera_box())
        column.addWidget(self._build_sequence_box())
        column.addWidget(self._build_run_box())
        column.addWidget(self._build_metadata_box())
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

    def _build_camera_box(self):
        box = QGroupBox("1. Kameras")
        layout = QVBoxLayout(box)
        self.cameras = CameraAssignmentView(self.runner)
        self.cameras.changed.connect(self._on_cameras_changed)
        layout.addWidget(self.cameras)

        row = QHBoxLayout()
        self.btn_preview = QPushButton("Vorschau starten")
        self.btn_preview.setToolTip(
            "Öffnet die zugeordneten Kameras in dieser Oberfläche: Bild, "
            "Bildrate und Zeitversatz. Wird beim Start einer Aufnahme "
            "geschlossen — apps/record.py braucht die Kameras dann selbst.")
        self.btn_preview.clicked.connect(self.toggle_preview)
        row.addWidget(self.btn_preview)
        row.addStretch(1)
        layout.addLayout(row)

        self.live_during_recording = QCheckBox("Livebild auch während der Aufnahme")
        self.live_during_recording.setToolTip(
            "Standardmäßig aus. apps/record.py schreibt das Bild dann in "
            "großen Abständen als JPEG — das kostet Zeit in der 15-Hz-Schleife, "
            "und die Datenqualität geht vor.")
        self.live_during_recording.toggled.connect(self._update_command)
        layout.addWidget(self.live_during_recording)
        return box

    def _build_sequence_box(self):
        """Ablauf waehlen und bearbeiten.

        Bearbeitet wird eine ARBEITSKOPIE, nie das Original (Festlegung
        Anwender, 2026-09-29): Die Datei im Projekt bleibt unveraendert und
        ist damit zugleich die Sicherungskopie. Aufgezeichnet wird mit der
        Arbeitskopie -- sie steht auch in der Kommandozeile. Sie bleibt
        liegen, beim naechsten Start ist der letzte Stand wieder da.
        """
        box = QGroupBox("2. Ablauf")
        layout = QVBoxLayout(box)

        row = QHBoxLayout()
        self.sequence = block_wheel(QComboBox())
        self.sequence.currentIndexChanged.connect(self._on_sequence_changed)
        row.addWidget(self.sequence, 1)
        self.btn_teach = QPushButton("Teachen …")
        self.btn_teach.setToolTip(
            "Startet apps/teach.py: Punkte mit dem Gamepad anfahren und "
            "ablegen. Bewegt den Roboter.")
        self.btn_teach.clicked.connect(self.start_teaching)
        row.addWidget(self.btn_teach)
        layout.addLayout(row)

        self.steps = QTableWidget(0, 6)
        self.steps.setHorizontalHeaderLabels(
            ["Punkt", "Bewegung", "Überschleifen (mm)", "optional", "Anflug", "Greifer"])
        self.steps.verticalHeader().setVisible(False)
        self.steps.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self.steps.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection)
        self.steps.setMinimumHeight(170)
        header = self.steps.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.steps.itemChanged.connect(self._on_steps_edited)
        layout.addWidget(self.steps)

        buttons = QHBoxLayout()
        for text, slot, tip in (
                ("+", self.add_step, "Schritt unter der Auswahl einfügen"),
                ("−", self.remove_step, "Ausgewählten Schritt löschen"),
                ("▲", lambda: self.move_step(-1), "Schritt nach oben"),
                ("▼", lambda: self.move_step(1), "Schritt nach unten")):
            button = QPushButton(text)
            button.setFixedWidth(34)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            buttons.addWidget(button)
        buttons.addSpacing(12)
        self.btn_save_sequence = QPushButton("Speichern")
        self.btn_save_sequence.setToolTip(
            "Schreibt die Arbeitskopie. Das Original im Projekt bleibt "
            "unberührt.")
        self.btn_save_sequence.clicked.connect(self.save_sequence)
        buttons.addWidget(self.btn_save_sequence)
        self.btn_reset_sequence = QPushButton("Aus Original zurücksetzen")
        self.btn_reset_sequence.clicked.connect(self.reset_sequence)
        buttons.addWidget(self.btn_reset_sequence)
        buttons.addStretch(1)
        layout.addLayout(buttons)

        self.sequence_info = _hint("")
        layout.addWidget(self.sequence_info)
        return box

    # -- Ablauf bearbeiten --------------------------------------------------

    def _known_points(self):
        """Punktnamen aus der Steuerung, wenn verbunden -- sonst leer.

        Nur eine Eingabehilfe: Ein Name, den die Datenbank nicht kennt, ist
        hier erlaubt (er kann spaeter geteacht werden). Ob er da ist, sagt
        record.py beim Aufloesen -- und die Vorabpruefung im Reiter.
        """
        robot = self.session.robot
        if robot is None:
            return []
        try:
            return sorted(robot.point_names())
        except Exception:
            log.exception("Punktnamen nicht lesbar")
            return []

    def _fill_steps(self, steps):
        known = self._known_points()
        self._loading_steps = True
        try:
            self.steps.setRowCount(len(steps))
            for row, step in enumerate(steps):
                point = block_wheel(QComboBox())
                point.setEditable(True)
                for name in known:
                    point.addItem(name)
                point.setCurrentText(step["point"])
                point.currentTextChanged.connect(self._on_steps_edited)
                self.steps.setCellWidget(row, 0, point)

                motion = block_wheel(QComboBox())
                motion.addItems(recording_module.MOTIONS)
                motion.setCurrentText(step["motion"])
                motion.currentTextChanged.connect(self._on_steps_edited)
                self.steps.setCellWidget(row, 1, motion)

                blend = block_wheel(QDoubleSpinBox())
                blend.setRange(0.0, 500.0)
                blend.setDecimals(1)
                blend.setSingleStep(5.0)
                blend.setValue(1e3 * float(step["blend"]))
                blend.setToolTip(
                    "Radius, mit dem die Ecke überschliffen wird. 0 = anhalten.")
                blend.valueChanged.connect(self._on_steps_edited)
                self.steps.setCellWidget(row, 2, blend)

                for column, key, tip in (
                        (3, "optional", "Fehlt der Punkt in der Datenbank, "
                                        "wird der Schritt übersprungen."),
                        (4, "approach", "Anflug: der Punkt wird auf gerader "
                                        "Linie angefahren.")):
                    check = QCheckBox()
                    check.setChecked(bool(step[key]))
                    check.setToolTip(tip)
                    check.toggled.connect(self._on_steps_edited)
                    holder = QWidget()
                    holder_layout = QHBoxLayout(holder)
                    holder_layout.setContentsMargins(0, 0, 0, 0)
                    holder_layout.addWidget(check)
                    holder_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
                    self.steps.setCellWidget(row, column, holder)

                gripper = block_wheel(QComboBox())
                gripper.addItem("—", None)
                gripper.addItem("auf", "open")
                gripper.addItem("zu", "close")
                index = find_data(gripper, step["gripper"])
                gripper.setCurrentIndex(index if index >= 0 else 0)
                gripper.currentIndexChanged.connect(self._on_steps_edited)
                self.steps.setCellWidget(row, 5, gripper)
            self.steps.resizeColumnsToContents()
            self.steps.horizontalHeader().setSectionResizeMode(
                0, QHeaderView.ResizeMode.Stretch)
        finally:
            self._loading_steps = False
        self._refresh_sequence_info()

    def current_steps(self):
        """Tabelle -> Schritte als dicts."""
        steps = []
        for row in range(self.steps.rowCount()):
            checks = []
            for column in (3, 4):
                holder = self.steps.cellWidget(row, column)
                checks.append(holder.findChild(QCheckBox).isChecked()
                              if holder else False)
            steps.append({
                "point": self.steps.cellWidget(row, 0).currentText().strip(),
                "motion": self.steps.cellWidget(row, 1).currentText(),
                "blend": self.steps.cellWidget(row, 2).value() / 1e3,
                "optional": checks[0],
                "approach": checks[1],
                "gripper": self.steps.cellWidget(row, 5).currentData(),
            })
        return steps

    def _on_steps_edited(self, *_args):
        """Signal aus der Tabelle: ab jetzt weicht sie von der Datei ab."""
        if self._loading_steps:
            return
        self._steps_dirty = True
        self._refresh_sequence_info()

    def _refresh_sequence_info(self):
        """Zusammenfassung und Pruefung unter der Tabelle neu aufbauen.

        Geprueft wird mit ``bc.sequence.parse_sequence`` -- genau dem, was
        auch record.py beim Laden anwendet. Ein Ablauf, der hier durchgeht,
        scheitert dort nicht mehr an der Form.
        """
        if not self._sequence_working:
            return
        steps = self.current_steps()
        problems = recording_module.validate_steps(
            self._sequence_name, self._gripper_start, steps)
        self.btn_save_sequence.setEnabled(bool(problems) is False and self._steps_dirty)
        if problems:
            self.sequence_info.setText("✗ " + "\n✗ ".join(problems))
            self.sequence_info.setStyleSheet("color:%s" % BAD)
            return
        summary = recording_module.sequence_summary({
            "error": None, "name": self._sequence_name,
            "points": [s["point"] for s in steps],
            "optional": [s["point"] for s in steps if s["optional"]],
            "blends": ["%s %.0f mm" % (s["point"], 1e3 * s["blend"])
                       for s in steps if s["blend"] > 0],
            "gripper": ["%s: %s" % (s["point"], s["gripper"])
                        for s in steps if s["gripper"]],
            "gripper_start": self._gripper_start,
        })
        if self._steps_dirty:
            summary += "\n\nGeändert — noch nicht gespeichert. Aufgezeichnet "
            summary += "wird der gespeicherte Stand."
        self.sequence_info.setText(summary)
        self.sequence_info.setStyleSheet(
            "color:%s" % (WARN if self._steps_dirty else NEUTRAL))

    def add_step(self):
        steps = self.current_steps()
        row = self.steps.currentRow()
        at = len(steps) if row < 0 else row + 1
        steps.insert(at, {"point": "", "motion": "ptp", "blend": 0.0,
                          "optional": False, "approach": False, "gripper": None})
        self._fill_steps(steps)
        self.steps.setCurrentCell(at, 0)
        self._steps_dirty = True
        self._refresh_sequence_info()

    def remove_step(self):
        row = self.steps.currentRow()
        if row < 0:
            return
        steps = self.current_steps()
        del steps[row]
        self._fill_steps(steps)
        self._steps_dirty = True
        self._refresh_sequence_info()

    def move_step(self, delta):
        row = self.steps.currentRow()
        target = row + delta
        if row < 0 or not 0 <= target < self.steps.rowCount():
            return
        steps = self.current_steps()
        steps[row], steps[target] = steps[target], steps[row]
        self._fill_steps(steps)
        self.steps.setCurrentCell(target, 0)
        self._steps_dirty = True
        self._refresh_sequence_info()

    def save_sequence(self):
        path = self._sequence_working
        if not path:
            return
        try:
            recording_module.write_steps(
                self.workdir / path, self._sequence_name, self._gripper_start,
                self.current_steps())
        except ValueError as exc:
            QMessageBox.warning(self, "Ablauf ungültig", str(exc))
            return
        self._steps_dirty = False
        self._refresh_sequence_info()
        log.info("Arbeitskopie gespeichert: %s", path)

    def reset_sequence(self):
        source = self.sequence.currentData()
        if not source or not self.confirm(
                "Aus Original zurücksetzen",
                "Die Arbeitskopie wird verworfen und neu aus\n%s\nangelegt.\n\n"
                "Fortfahren?" % self._sequence_source):
            return
        recording_module.reset_working_copy(self.workdir, self._sequence_source)
        self._on_sequence_changed()

    def _build_run_box(self):
        box = QGroupBox("3. Aufnahme")
        layout = QVBoxLayout(box)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self.episodes = block_wheel(QSpinBox())
        self.episodes.setRange(1, 500)
        self.episodes.setValue(2)
        self.episodes.valueChanged.connect(self._update_command)
        form.addRow("Episoden", self.episodes)

        self.noise_scale = block_wheel(QDoubleSpinBox())
        self.noise_scale.setRange(0.0, 3.0)
        self.noise_scale.setSingleStep(0.1)
        self.noise_scale.setDecimals(2)
        self.noise_scale.setValue(1.0)
        self.noise_scale.setToolTip(
            "Höchster Faktor auf die Rauschamplituden. Je Episode wird daraus "
            "zufällig skaliert (Festlegung Rauschstudie 2026-09-16). "
            "0 = Referenzfahrt ohne Rauschen.")
        self.noise_scale.valueChanged.connect(self._update_command)
        form.addRow("Rauschen bis", self.noise_scale)

        self.noise_fixed = QCheckBox("Faktor nicht je Episode ziehen")
        self.noise_fixed.setToolTip("Nur für Vergleichsfahrten.")
        self.noise_fixed.toggled.connect(self._update_command)
        form.addRow("", self.noise_fixed)

        # Derselbe Wert wie oben in der Backend-Leiste: der Override
        # gehoert zum Lauf, nicht zum Reiter (siehe session.set_override).
        self.override = block_wheel(QDoubleSpinBox())
        self.override.setRange(0.05, 1.0)
        self.override.setSingleStep(0.05)
        self.override.setDecimals(2)
        self.override.setValue(float(self.session.override))
        self.override.setToolTip(
            "Geschwindigkeits-Override am Neura — dieselbe Zahl wie oben in "
            "der Backend-Leiste. Muss innerhalb eines Datensatzes einheitlich "
            "sein (der Export erzwingt das) und ist mitgelernt (AP 2.6).")
        self.override.valueChanged.connect(self.session.set_override)
        form.addRow("Override", self.override)

        layout.addLayout(form)

        self.ask_label = QCheckBox("Nach jeder Episode bewerten")
        self.ask_label.setChecked(True)
        self.ask_label.setToolTip(
            "Der Lauf hält nach jeder Episode an und fragt: erfolgreich, "
            "fehlgeschlagen oder verwerfen (AP 5.2). Ohne Haken bleibt die "
            "Episode unbewertet — apps/export.py --require-success lässt sie "
            "dann später aus.")
        self.ask_label.toggled.connect(self._update_command)
        layout.addWidget(self.ask_label)

        row = QHBoxLayout()
        row.addWidget(QLabel("Ziel"))
        self.out = QLineEdit()
        self.out.textChanged.connect(self._update_command)
        row.addWidget(self.out, 1)
        layout.addLayout(row)

        self.episode_hint = _hint("")
        layout.addWidget(self.episode_hint)
        return box

    def _build_metadata_box(self):
        """Metadaten je Aufruf (AP 5.2).

        Alle vier sind FREITEXT und technisch optional -- bis auf den Block
        an der realen Anlage, wo ``record.py`` ohne ihn abbricht. Sie gehen
        unveraendert in ``meta.json`` jeder Episode und sind die Grundlage
        der Ablationen aus AP 5.1: Ohne sie laesst sich hinterher nicht mehr
        sagen, welche Episoden zu welcher Objektlage, Beleuchtung oder
        Kamerapose gehoeren.
        """
        box = QGroupBox("4. Metadaten (AP 5.2)")
        layout = QVBoxLayout(box)
        layout.addWidget(_hint(
            "Freitext, gilt für alle Episoden dieses Aufrufs und landet in "
            "meta.json jeder Episode. Ohne diese Angaben lassen sich die "
            "Ablationen (AP 5.1) hinterher nicht auswerten — man weiß dann "
            "nicht mehr, welche Episode zu welcher Objektlage gehört.\n"
            "Nur der Block ist an der realen Anlage Pflicht; die Session "
            "füllt record.py sonst mit dem Datum."))
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self.block = QLineEdit()
        self.block.setPlaceholderText("z. B. B07 — Pflicht an der Anlage")
        self.block.setToolTip(
            "Eine Objektlage = ein Block. Ändern, sobald das Objekt neu "
            "hingelegt und PRE_GRASP/PICK neu geteacht wurden.")
        self.object_note = QLineEdit()
        self.object_note.setPlaceholderText("z. B. Teil A, 30° gedreht, links")
        self.object_note.setToolTip(
            "Objekt und Lage in Worten. Die geteachten Greifpunkte werden "
            "ohnehin als object_pose abgelegt — das hier ist die Lesehilfe "
            "dazu.")
        self.light = QLineEdit()
        self.light.setPlaceholderText("z. B. Decke an, Rollo zu")
        self.light.setToolTip(
            "Beleuchtungssituation. Ändert die Bildstatistik, auf die "
            "trainiert wird.")
        self.camera_pose = QLineEdit()
        self.camera_pose.setPlaceholderText("z. B. nominal oder +2cm x")
        self.camera_pose.setToolTip(
            "Variante der Szenenkamera-Pose. Nur ändern, wenn die Kamera "
            "tatsächlich versetzt wurde (AP 1.4).")
        for widget, label in ((self.block, "Block"),
                              (self.object_note, "Objekt"),
                              (self.light, "Beleuchtung"),
                              (self.camera_pose, "Kamerapose")):
            widget.textChanged.connect(self._update_command)
            form.addRow(label, widget)
        layout.addLayout(form)
        return box

    def _build_action_bar(self):
        bar = QWidget()
        outer = QVBoxLayout(bar)
        outer.setContentsMargins(0, 4, 0, 0)
        self.check_label = QLabel("")
        self.check_label.setWordWrap(True)
        outer.addWidget(self.check_label)
        row = QWidget()
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        self.btn_record = QPushButton("Aufnahme starten")
        self.btn_stop = QPushButton("STOPP")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(
            "QPushButton:enabled{background:%s; color:white; font-weight:bold}" % BAD)
        self.btn_record.clicked.connect(self.start_recording)
        self.btn_stop.clicked.connect(self.stop)
        layout.addWidget(self.btn_record)
        layout.addWidget(self.btn_stop)
        layout.addStretch(1)
        outer.addWidget(row)
        return bar

    # -- Aufbau: Ausgabeseite ----------------------------------------------

    def _build_output(self):
        page = QWidget()
        layout = QVBoxLayout(page)

        # Bild und Zeile darunter als EIN Kasten: sonst nimmt sich die
        # Bildflaeche (Expanding) den Platz der Zeile, und beide
        # ueberlagern sich, sobald das Fenster knapp wird.
        image_box = QGroupBox("Bild")
        image_layout = QVBoxLayout(image_box)
        self.image = ImageView("Vorschau aus — \"Vorschau starten\" öffnet die "
                               "zugeordneten Kameras")
        # Klein genug, dass die ganze Spalte auch auf einem Laptopbildschirm
        # passt: Reicht der Platz nicht, verteilt Qt ihn nicht, sondern legt
        # die Widgets uebereinander. Mehr Platz nimmt sich die Bildflaeche
        # ueber die Expanding-Groessenpolitik von selbst.
        self.image.setMinimumHeight(120)
        image_layout.addWidget(self.image, 1)
        self.sources_line = _hint("")
        self.sources_line.setMinimumHeight(18)
        image_layout.addWidget(self.sources_line, 0)
        layout.addWidget(image_box, 1)

        command_box = QGroupBox("Kommandozeile")
        command_layout = QVBoxLayout(command_box)
        self.command = QLabel("")
        self.command.setWordWrap(True)
        self.command.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        self.command.setStyleSheet("font-family:Consolas; color:#333")
        command_layout.addWidget(self.command)
        layout.addWidget(command_box)

        progress_box = QGroupBox("Fortschritt")
        progress_layout = QVBoxLayout(progress_box)
        self.headline = QLabel("noch nichts gestartet")
        headline_font = QFont()
        headline_font.setBold(True)
        self.headline.setFont(headline_font)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.detail_line = _hint("")
        progress_layout.addWidget(self.headline)
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.detail_line)

        self.episode_list = QTreeWidget()
        self.episode_list.setHeaderLabels(["Episode", "Ordner", "Schritte", "Ergebnis"])
        self.episode_list.setRootIsDecorated(False)
        self.episode_list.setMaximumHeight(110)
        for column, width in enumerate((70, 130, 80, 240)):
            self.episode_list.setColumnWidth(column, width)
        progress_layout.addWidget(self.episode_list)
        layout.addWidget(progress_box)

        output_box = QGroupBox("Ausgabe")
        output_layout = QVBoxLayout(output_box)
        self.output = LogView()
        self.output.setMaximumHeight(110)
        output_layout.addWidget(self.output)
        layout.addWidget(output_box)

        # Auch die Ausgabeseite in einen Bildlauf: Reicht die Hoehe nicht,
        # verteilt Qt den Platz nicht, sondern legt Widgets uebereinander.
        # Auf einem Laptopbildschirm lagen so Bildflaeche und die Zeile
        # darunter ineinander.
        area = QScrollArea()
        area.setWidgetResizable(True)
        area.setWidget(page)
        area.setFrameShape(QScrollArea.Shape.NoFrame)
        return area

    # -- Aktualisieren ------------------------------------------------------

    def refresh(self):
        super().refresh()
        self._reload_sequences()
        self.cameras.refresh(
            backend_is_sim_robot=self.session.backend == session_module.BACKEND_SIM)
        self._suggest_target()
        self._update_command()

    def _suggest_target(self):
        """Zielordner vorschlagen -- ohne einen eigenen Eintrag zu ueberschreiben.

        Der Ordner haengt am Roboter: VM-Daten nach ``data_vm/<Datum>/``,
        Sim-Daten nach ``data_sim/`` (Festlegung 2026-09-16). Beim Wechsel
        des Ziels muss der Vorschlag also mitgehen -- sonst landen VM-Laeufe
        unter data_sim und die Berichte zeigen ins Leere. Was der Bedienende
        selbst eingetippt hat, bleibt stehen.
        """
        suggestion = str(recording_module.suggest_out(
            self.workdir, self._robot_kind()))
        current = self.out.text().strip()
        if current and current != getattr(self, "_suggested", None):
            return
        self._suggested = suggestion
        self.out.setText(suggestion)

    def _reload_sequences(self):
        current = self.sequence.currentData()
        self.sequence.blockSignals(True)
        self.sequence.clear()
        self.sequence.addItem("— keine (Demo-Wegpunkte) —", None)
        for path in recording_module.find_sequences(self.workdir):
            self.sequence.addItem(str(path), str(path))
        # Beim ersten Aufbau die erste Ablaufdatei vorbelegen: Ohne
        # Ablaufdatei faehrt record.py Demo-Wegpunkte um die Home-Pose, und
        # das ist fast nie gemeint.
        index = find_data(self.sequence, current) if current else -1
        if index < 0 and self.sequence.count() > 1:
            index = 1
        self.sequence.setCurrentIndex(max(index, 0))
        self.sequence.blockSignals(False)
        self._on_sequence_changed()

    def _on_sequence_changed(self):
        """Ablauf gewechselt: Arbeitskopie bereitstellen und einlesen.

        Ausgewaehlt wird das ORIGINAL, aufgezeichnet wird die Arbeitskopie
        daneben. Existiert sie schon, bleibt sie, wie sie ist -- das ist der
        letzte Stand.
        """
        source = self.sequence.currentData()
        self._sequence_source = source
        if not source:
            self._sequence_working = None
            self._sequence_name = ""
            self._gripper_start = "open"
            self._steps_dirty = False
            self.steps.setRowCount(0)
            self.sequence_info.setText(
                "Ohne Ablaufdatei fährt record.py Demo-Wegpunkte um die "
                "Home-Pose — nur für den SimRobot sinnvoll.")
            self.sequence_info.setStyleSheet("color:%s" % NEUTRAL)
            self._set_editor_enabled(False)
            self._update_command()
            return

        try:
            working = recording_module.ensure_working_copy(self.workdir, source)
            name, gripper_start, steps = recording_module.read_steps(
                self.workdir / working)
        except Exception as exc:
            self._sequence_working = None
            self.steps.setRowCount(0)
            self.sequence_info.setText(
                "Ablauf nicht lesbar: %s: %s" % (type(exc).__name__, exc))
            self.sequence_info.setStyleSheet("color:%s" % BAD)
            self._set_editor_enabled(False)
            self._update_command()
            return

        self._sequence_working = working
        self._sequence_name = name
        self._gripper_start = gripper_start
        self._steps_dirty = False
        self._set_editor_enabled(True)
        self._fill_steps(steps)
        self._update_command()

    def _set_editor_enabled(self, enabled):
        for widget in (self.steps, self.btn_save_sequence,
                       self.btn_reset_sequence):
            widget.setEnabled(enabled)

    def _on_cameras_changed(self):
        if self.preview_timer.isActive():
            # Zuordnung geaendert: die offene Vorschau zeigt sonst noch die
            # alten Geraete.
            self.stop_preview()
        self._update_command()

    def _on_session_status(self, _status):
        # Das Ziel haengt am Roboter -- beim Backendwechsel mitziehen.
        self._suggest_target()
        if abs(self.override.value() - self.session.override) > 1e-12:
            self.override.blockSignals(True)
            self.override.setValue(float(self.session.override))
            self.override.blockSignals(False)
        self._update_command()

    def _robot_kind(self):
        return ("sim" if self.session.backend == session_module.BACKEND_SIM
                else "neura")

    # -- Kommandozeile und Pruefung ----------------------------------------

    def _live_file(self):
        """Ablageort des Livebildes waehrend einer Aufnahme.

        Im Temp-Verzeichnis, nicht im Projekt: Es ist eine fluechtige Datei
        zwischen zwei Prozessen und hat im Repository nichts verloren.
        """
        import tempfile

        return Path(tempfile.gettempdir()) / "bc_gui_live_preview.jpg"

    def _argv(self):
        status = self.session.status()
        return recording_module.record_argv(
            self.out.text().strip() or None,
            robot_kind=self._robot_kind(),
            episodes=self.episodes.value(),
            # Aufgezeichnet wird die Arbeitskopie, nicht das Original.
            sequence=self._sequence_working,
            camera_specs=cameras_module.to_specs(self.cameras.assignment()),
            override=(self.session.override if self._robot_kind() == "neura"
                      else None),
            noise_scale=self.noise_scale.value(),
            noise_fixed=self.noise_fixed.isChecked(),
            block=self.block.text().strip() or None,
            light=self.light.text().strip() or None,
            camera_pose=self.camera_pose.text().strip() or None,
            object_note=self.object_note.text().strip() or None,
            ask_label=self.ask_label.isChecked(),
            preview=(str(self._live_file())
                     if self.live_during_recording.isChecked() else None),
            real_robot=status.is_real_plant)

    def _update_command(self):
        try:
            self.command.setText(training_module.command_line(self._argv()))
        except ValueError as exc:
            self.command.setText("Aufnahme: %s" % exc)

        status = self.session.status()
        assignment = self.cameras.assignment()
        advice = recording_module.episode_advice(
            status.is_real_plant, self.episodes.value())
        self.episode_hint.setText(advice or "")
        self.episode_hint.setStyleSheet("color:%s" % (WARN if advice else NEUTRAL))

        lines = []
        for problem in recording_module.setup_problems(
                self._robot_kind(), self.sequence.currentData()):
            lines.append("✗ " + problem)
        if status.is_real_plant:
            for missing in recording_module.plant_requirements(
                    self.block.text().strip(), self.sequence.currentData()):
                lines.append("✗ " + missing)
        for warning in cameras_module.assignment_warnings(
                assignment, self.cameras.inventory, status.is_real_plant):
            lines.append("! " + warning)
        if not lines:
            self.check_label.setText("Bereit.")
            self.check_label.setStyleSheet("color:%s" % NEUTRAL)
        else:
            self.check_label.setText("\n".join(lines))
            self.check_label.setStyleSheet(
                "color:%s" % (BAD if any(l.startswith("✗") for l in lines) else WARN))

    # -- Vorschau -----------------------------------------------------------

    def toggle_preview(self):
        if self.preview_timer.isActive() or self._captures:
            self.stop_preview()
        else:
            self.start_preview()

    def start_preview(self):
        """Oeffnet die zugeordneten Kameras IN DIESEM PROZESS.

        Nur hier -- waehrend einer Aufnahme gehoeren sie dem Unterprozess.
        """
        if self.process.running:
            QMessageBox.information(
                self, "Aufnahme läuft",
                "Während der Aufnahme gehören die Kameras apps/record.py. "
                "Eine Kamera lässt sich nicht zweimal öffnen.\n\n"
                "Für ein Bild während der Aufnahme \"Livebild auch während "
                "der Aufnahme\" ankreuzen und neu starten.")
            return
        assignment = self.cameras.assignment()
        if not self.ensure_requirements(
                "Die Vorschau", cameras_module.required_backend_keys(assignment),
                "Mit Platzhalterbildern geht sie ohne alles."):
            return
        self.btn_preview.setEnabled(False)
        self.btn_preview.setText("öffne …")
        configs = cameras_module.to_configs(assignment)
        self.runner.submit(lambda: _open_captures(configs),
                           on_done=self._preview_started,
                           on_error=self._preview_failed,
                           pool="io")

    def _preview_started(self, captures):
        self._captures = captures
        self._rates = RateMeter()
        self.btn_preview.setEnabled(True)
        self.btn_preview.setText("Vorschau beenden")
        self.preview_timer.start()
        log.info("Vorschau offen: %s",
                 ", ".join(c.name for c in captures))

    def _preview_failed(self, error):
        self._captures = None
        self.btn_preview.setEnabled(True)
        self.btn_preview.setText("Vorschau starten")
        self.image.set_image(None)
        self.sources_line.setText("Vorschau gescheitert: %s" % error)
        self.sources_line.setStyleSheet("color:%s" % BAD)
        log.error("Vorschau gescheitert: %s", error)

    def stop_preview(self):
        self.preview_timer.stop()
        captures, self._captures = self._captures, None
        if captures:
            for capture in captures:
                try:
                    capture.stop()
                except Exception:
                    log.exception("Kamera schliessen gescheitert")
        self.btn_preview.setEnabled(True)
        self.btn_preview.setText("Vorschau starten")
        self.image.set_image(None)
        self.sources_line.setText("")
        self.sources_line.setStyleSheet("color:%s" % NEUTRAL)

    def _tick_preview(self):
        if not self._captures:
            return
        from .. import preview as preview_module

        frames = {}
        bits = []
        timestamps = []
        for capture in self._captures:
            try:
                frame = capture.latest()
            except Exception as exc:
                frame = None
                bits.append("%s: %s" % (capture.name, type(exc).__name__))
            frames[capture.name] = frame
            if frame is None:
                bits.append("%s: kein Bild" % capture.name)
            else:
                timestamps.append(frame.timestamp)
                fps = self._rates.update(capture.name, frame)
                bits.append("%s: %s" % (
                    capture.name, "—" if fps is None else "%.1f fps" % fps))
        board = preview_module.compose(frames, height=360)
        self.image.set_image(board)
        if len(timestamps) > 1:
            skew_ms = 1e3 * (max(timestamps) - min(timestamps))
            budget = 1e3 * config.SYNC_MAX_SKEW_S
            bits.append("Versatz %.0f ms %s Budget %.0f ms"
                        % (skew_ms, "über" if skew_ms > budget else "unter", budget))
            self.sources_line.setStyleSheet(
                "color:%s" % (WARN if skew_ms > budget else NEUTRAL))
        self.sources_line.setText("  ·  ".join(bits))

    def _tick_live(self):
        """Livebild einer laufenden Aufnahme von der Platte lesen."""
        if self._live_path is None:
            return
        try:
            import cv2

            image = cv2.imread(str(self._live_path))
        except Exception:
            return
        if image is not None:
            self.image.set_image(image)

    # -- Teachen ------------------------------------------------------------

    def start_teaching(self):
        if self.process.running:
            QMessageBox.information(self, "Läuft bereits",
                                    "Erst die laufende Aufnahme beenden.")
            return
        if not self.confirm(
                "Teachen startet",
                "apps/teach.py bewegt den Roboter über das Gamepad.\n\n"
                "Backend laut Steuerung: %s\n\n"
                "Die Bedienung läuft im Terminal-Fenster dieser Anwendung, "
                "die Ausgabe erscheint unten.\n\n"
                "Arbeitsraum frei? Not-Aus in Reichweite?"
                % self.session.status().text):
            return
        self.stop_preview()
        self.output.clear()
        argv = ["apps/teach.py"]
        if self._robot_kind() == "sim":
            argv.append("--sim")
        try:
            self.process.start(argv)
        except Exception as exc:
            QMessageBox.warning(self, "Start gescheitert", str(exc))
            return
        self._progress = None
        self.headline.setText("Teachen läuft")
        self._set_running(True)

    # -- Aufnehmen ----------------------------------------------------------

    def start_recording(self):
        if self.process.running:
            QMessageBox.information(self, "Läuft bereits",
                                    "Es läuft schon etwas in diesem Reiter.")
            return
        if not self.out.text().strip():
            QMessageBox.information(self, "Kein Ziel",
                                    "Zielordner für die Aufzeichnung angeben.")
            return

        assignment = self.cameras.assignment()
        keys = list(cameras_module.required_backend_keys(assignment))
        if self._robot_kind() == "neura":
            keys.append("neurapy")
        if not self.ensure_requirements(
                "Eine Aufnahme", keys,
                "Mit Platzhalterbildern und dem SimRobot geht sie ohne alles."):
            return

        problems = recording_module.setup_problems(
            self._robot_kind(), self.sequence.currentData())
        if problems:
            QMessageBox.warning(
                self, "So kann es nicht laufen",
                "apps/record.py bricht damit ab:\n\n"
                + "\n\n".join("  - " + p for p in problems))
            return

        status = self.session.status()
        if status.is_real_plant:
            missing = recording_module.plant_requirements(
                self.block.text().strip(), self.sequence.currentData())
            if missing:
                QMessageBox.warning(
                    self, "An der Anlage nicht erlaubt",
                    "apps/record.py bricht damit ab:\n\n"
                    + "\n".join("  - " + m for m in missing))
                return

        if not self._confirm_motion():
            return

        # Die Vorschau muss weg: apps/record.py oeffnet dieselben Kameras.
        self.stop_preview()

        self._progress = recording_module.RecordProgress(
            total_episodes=self.episodes.value())
        self.episode_list.clear()
        self.output.clear()
        self.progress_bar.setValue(0)
        self.headline.setText("startet …")
        self.headline.setStyleSheet("font-weight:bold")
        self.detail_line.setText("")

        if self.live_during_recording.isChecked():
            self._live_path = self._live_file()
            try:
                self._live_path.unlink()
            except OSError:
                pass
            self.live_timer.start()
            self.image.setText("warte auf das erste Livebild …")
        else:
            self._live_path = None
            self.image.set_image(None)
            self.image.setText("Livebild aus — während der Aufnahme gehören "
                               "die Kameras apps/record.py")

        try:
            # stdin bleibt offen, solange nach jeder Episode gefragt wird
            # -- sonst bekommt record.py EOF und laesst alles unbewertet.
            self.process.start(
                self._argv(),
                stdin_text="ANLAGE\n" if status.is_real_plant else None,
                keep_stdin=self.ask_label.isChecked())
        except Exception as exc:
            self.live_timer.stop()
            QMessageBox.warning(self, "Start gescheitert", str(exc))
            return
        self._set_running(True)

    def _confirm_motion(self):
        status = self.session.status()
        lines = ["Die Aufzeichnung fährt die Bahn ab und bewegt den Roboter.", ""]
        lines.append("Backend laut Steuerung: %s" % status.text)
        lines.append("Kameras: %s" % cameras_module.describe(self.cameras.assignment()))
        lines.append("Episoden: %d  ·  Ablauf: %s"
                     % (self.episodes.value(),
                        self.sequence.currentData() or "Demo-Wegpunkte"))
        lines.append("Ziel: %s" % self.out.text().strip())
        if status.state == session_module.OFFLINE:
            lines += ["", "Es ist nichts verbunden — apps/record.py verbindet "
                          "selbst. Welche Steuerung antwortet, steht erst dann fest."]
        if status.is_real_plant:
            lines += ["", "ACHTUNG: Das ist die REALE ANLAGE, nicht die Simulation."]
        lines += ["", "Arbeitsraum frei? Not-Aus in Reichweite?"]
        return self.confirm("Roboter bewegt sich", "\n".join(lines))

    def stop(self):
        if not self.process.running:
            return
        if not self.confirm(
                "Abbrechen",
                "Der laufende Prozess wird beendet. Die angefangene Episode "
                "ist verloren, bereits abgelegte bleiben.\n\n"
                "Das ist ein Software-Stopp und ersetzt keinen Not-Aus "
                "(AP 4.2).\n\nWirklich abbrechen?"):
            return
        self.process.stop()

    # -- Ausgabe ------------------------------------------------------------

    def _on_started(self, command):
        self.output.append_record("$ " + " ".join(command[1:]), logging.INFO)

    def _on_line(self, text):
        self.output.append_line(text)

        question = recording_module.label_question(text)
        if question is not None:
            # Der Lauf steht jetzt und wartet auf eine Zeile auf stdin.
            self._ask_label(*question)
            return

        if self._progress is None or not self._progress.feed(text):
            return
        before = self.episode_list.topLevelItemCount()
        for episode in self._progress.episodes[before:]:
            item = QTreeWidgetItem(self.episode_list, episode.row())
            if episode.discarded:
                for column in range(4):
                    item.setForeground(column, _brush(WARN))
            self.episode_list.addTopLevelItem(item)
            self.episode_list.scrollToItem(item)
        self.headline.setText(self._progress.headline())
        self.detail_line.setText(self._progress.detail_line())
        percent = self._progress.percent
        if percent is not None:
            self.progress_bar.setValue(percent)

    def _ask_label(self, episode, steps):
        """Erfolgs-Label abfragen und an den Unterprozess schicken (AP 5.2).

        Der Lauf steht so lange. Das ist gewollt: An der Anlage wird
        zwischen zwei Episoden ohnehin zurueckgesetzt, und bewertet wird,
        solange man es noch gesehen hat. Wird das Fenster geschlossen oder
        der Lauf abgebrochen, bekommt record.py EOF und laesst die Episode
        unbewertet -- nichts wird stillschweigend weggeworfen.
        """
        box = QMessageBox(self)
        box.setWindowTitle("Episode %d bewerten" % episode)
        box.setIcon(QMessageBox.Icon.Question)
        box.setText("Episode %d ist aufgezeichnet (%d Schritte)." % (episode, steps))
        box.setInformativeText(
            "War der Griff erfolgreich?\n\n"
            "Der Lauf wartet auf die Antwort. „Verwerfen\" markiert die "
            "Episode als verworfen — sie bleibt auf der Platte, kommt aber "
            "nicht in den Datensatz.")
        good = box.addButton("Erfolgreich", QMessageBox.ButtonRole.AcceptRole)
        bad = box.addButton("Fehlgeschlagen", QMessageBox.ButtonRole.DestructiveRole)
        drop = box.addButton("Verwerfen", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(good)
        box.exec()
        clicked = box.clickedButton()
        answer = {good: "erfolg", bad: "fehler", drop: "verwerfen"}.get(clicked, "fehler")
        if not self.process.send(recording_module.LABEL_ANSWERS[answer]):
            log.warning("Bewertung konnte nicht gesendet werden -- laeuft der Prozess noch?")

    def _on_finished(self, code, text):
        self._set_running(False)
        self.live_timer.stop()
        if self._live_path is not None:
            try:
                self._live_path.unlink()
            except OSError:
                pass
            self._live_path = None
        ok = code == 0
        if self._progress is None:
            self.headline.setText("Teachen beendet" if ok else
                                  "Beendet (Rückgabewert %d)" % code)
        elif ok:
            self.headline.setText(self._progress.headline())
            self.progress_bar.setValue(100)
        else:
            self.headline.setText("Abgebrochen (Rückgabewert %d)" % code)
        self.headline.setStyleSheet("color:%s; font-weight:bold" % (GOOD if ok else BAD))
        self.output.append_record("-> beendet, Rückgabewert %d" % code,
                                  logging.INFO if ok else logging.ERROR)
        self.image.setText("Vorschau aus — \"Vorschau starten\" öffnet die "
                           "zugeordneten Kameras")

    def _set_running(self, running):
        for widget in (self.btn_record, self.btn_teach, self.btn_preview,
                       self.cameras, self.sequence, self.episodes):
            widget.setEnabled(not running)
        self.btn_stop.setEnabled(running)

    # -- Lebenszyklus -------------------------------------------------------

    def shutdown(self):
        self.preview_timer.stop()
        self.live_timer.stop()
        self.stop_preview()
        if self.process.running:
            self.process.stop()


class RateMeter(object):
    """Tatsaechlich erreichte Bildrate je Quelle (AP 1.2).

    Gemessen wird aus ``Frame.index`` und ``Frame.timestamp`` der Kamera --
    also die Rate, die WIRKLICH ankommt, nicht die angeforderte. Genau
    darum geht es: Die Webcam am Entwicklungsrechner meldet 30 fps und
    liefert 9-11 (Befund 2026-09-23), und der Recorder verwirft dann
    korrekt wegen des Latenzbudgets. Das soll man vor der Aufnahme sehen.

    Gemittelt ueber ein Fenster, weil die Momentanrate zwischen zwei
    Frames nur rauscht.
    """

    #: Laenge des Fensters in Sekunden.
    WINDOW_S = 2.0

    def __init__(self):
        self._first = {}

    def update(self, name, frame):
        """Liefert die Rate in fps, oder None, solange zu wenig vorliegt."""
        index, timestamp = frame.index, frame.timestamp
        start = self._first.get(name)
        if start is None or timestamp < start[1] or index < start[0]:
            self._first[name] = (index, timestamp)
            return None
        frames = index - start[0]
        seconds = timestamp - start[1]
        if seconds >= self.WINDOW_S:
            # Fenster weiterschieben, damit ein Einbruch sichtbar wird und
            # nicht im Mittel seit dem Start untergeht.
            self._first[name] = (index, timestamp)
        if seconds <= 0 or frames <= 0:
            return None
        return frames / seconds

    def reset(self):
        self._first.clear()


def _open_captures(configs):
    """Kameras oeffnen und starten -- laeuft im Arbeitsthread.

    Geht ueber ``bc.capture`` und ``bc.adapters`` wie der Recorder, damit
    die Vorschau dieselbe Kette prueft, die spaeter aufzeichnet.
    """
    from .. import capture as capture_module
    from ..adapters import open_camera

    started = []
    try:
        for cfg in configs:
            if cfg.backend == "sim":
                from ..adapters.cam_sim import SimCamera
                from ..clock import RealClock

                cap = capture_module.DirectCapture(
                    SimCamera(cfg, clock=RealClock(), pattern="static"))
            else:
                cap = capture_module.ThreadedCapture(open_camera(cfg))
            cap.start()
            started.append(cap)
        for cap in started:
            if isinstance(cap, capture_module.ThreadedCapture):
                cap.wait_for_frame(timeout=10.0)
    except Exception:
        for cap in started:
            try:
                cap.stop()
            except Exception:
                pass
        raise
    return started


def _hint(text):
    label = QLabel(text)
    label.setWordWrap(True)
    label.setStyleSheet("color:%s" % NEUTRAL)
    return label


def _brush(colour):
    from PySide6.QtGui import QBrush, QColor

    return QBrush(QColor(colour))

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
import time
from pathlib import Path

import numpy as np
from PySide6.QtCore import QEvent, Qt, QTimer
from PySide6.QtGui import QFont, QKeySequence, QShortcut
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QCheckBox,
                               QComboBox, QDoubleSpinBox, QFormLayout,
                               QGroupBox, QHBoxLayout, QHeaderView, QLabel,
                               QLineEdit, QMenu, QMessageBox, QProgressBar,
                               QPushButton, QScrollArea, QSpinBox, QSplitter,
                               QDialog, QTableWidget, QTableWidgetItem,
                               QToolButton, QTreeWidget, QTreeWidgetItem,
                               QVBoxLayout, QWidget)

from . import _bootstrap
from . import cameras as cameras_module
from . import jogging as jogging_module
from . import points as points_module
from . import vocabulary as vocabulary_module
from . import recording as recording_module
from . import session as session_module
from . import training as training_module
from .. import config
from .. import teach_series as series_module
from .. import touchup as touchup_module
from .. import trajectory as trajectory_module
from ..adapters import describe_settings, fps_label
from ..clock import RealClock
from ..sync import camera_rate_findings
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
        #: Ausgabeordner des letzten Laufs (fuer _episode_menu)
        self._run_out = None
        self._captures = None
        #: Abbruchsignal eines laufenden Oeffnens der Vorschau, sonst None
        #: (Muster wie widgets.CameraAssignment.search).
        self._preview_cancel = None
        self._live_path = None
        self._rates = RateMeter()
        #: Zurueckgelesene Kameraeinstellungen und Konfiguration der
        #: laufenden Vorschau (Bildratenpruefung, Anlage 2026-10-26).
        self._camera_settings = {}
        self._preview_configs = {}
        self._sequence_source = None
        self._sequence_working = None
        self._sequence_name = ""
        self._gripper_start = "open"
        self._steps_dirty = False
        self._loading_steps = False
        #: Sicherungsdatei des letzten Touch-ups (bc/touchup.py), fuer
        #: "Touch-up rueckgaengig".
        self._last_touchup = None
        #: Zeile des letzten Touch-ups -- die Ruecknahme springt dorthin zurueck.
        self._last_touchup_row = -1
        #: Punkte-Liste (bc/gui/points.py): Namen in der Steuerung (beim
        #: Fuellen der Tabelle gelesen), Sperre gegen Rueckkopplung beim
        #: Uebertragen in die Ablauftabelle, laufendes "Anfahren".
        self._known_cache = []
        self._points_editing = False
        self._moving = False
        #: "Teachen" eines einzelnen Punkts: Name, solange die Serie nur auf
        #: ihn wartet, sonst None.
        self._series_single = None
        #: "table" der Ablaufdatei (Tischpunkte, Greifhoehe), bc/table.py
        self._table_cfg = None
        #: Block schon fuer eine Aufnahme benutzt? Dann zaehlt die naechste
        #: Aenderung von PICK (neue Lage) ihn hoch (Anwender 2026-10-26).
        self._block_used = False
        #: "Nicht erneut anzeigen" im Bewertungsdialog: restliche Episoden
        #: dieses Laufs bleiben unbewertet, der Lauf wartet nicht mehr.
        self._labels_off = False
        #: Naechster freier Block laut record.py (automatische Lagen
        #: verbrauchen mehrere) -- gilt fuer die naechste neue Lage.
        self._next_free_block = None
        #: Hiermit sagt die Oberflaeche record.py "nach dieser Lage beenden"
        import tempfile
        self._stop_file = Path(tempfile.gettempdir()) / "bc_gui_nach_lage_beenden.flag"
        #: Grossansicht einer Kamera (Doppelklick): (Dialog, Kameraname)
        self._zoom = None
        self._live_names = []

        #: Tippbetrieb "Z hoch": laufender Vorgang und sein Zeitgeber.
        #: ``_jog_source``: "button" (Maus) oder "key" (F7 gehalten).
        self._jogger = None
        self._jog_source = "button"
        self._jog_key_down = False
        self.jog_timer = QTimer(self)
        self.jog_timer.setInterval(int(round(1000.0 / config.JOG_TICK_HZ)))
        self.jog_timer.timeout.connect(self._jog_tick)

        #: Teach-Serie (bc/teach_series.py): Beobachter der Punktliste, solange
        #: sie laeuft, sonst None. ``_series_busy`` sperrt den Zeitgeber,
        #: waehrend eine Rueckfrage offen ist -- deren Ereignisschleife liefe
        #: sonst in den naechsten Tick hinein.
        self._series_watcher = None
        self._series_busy = False
        self.series_timer = QTimer(self)
        self.series_timer.setInterval(config.TEACH_SERIES_POLL_MS)
        self.series_timer.timeout.connect(self._series_tick)

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
        # Links so breit, dass die Punkte-Liste samt Knoepfen ganz sichtbar
        # ist (Anwender 2026-10-26: musste immer von Hand aufgezogen werden)
        splitter.setSizes([780, 620])

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
        panel.setMinimumWidth(740)
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
        self.sequence.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.sequence.setMinimumContentsLength(24)
        self.sequence.currentIndexChanged.connect(self._on_sequence_changed)
        row.addWidget(self.sequence, 1)
        self.btn_teach = QPushButton("Teachen …")
        self.btn_teach.setToolTip(
            "Startet apps/teach.py: Punkte mit dem Gamepad anfahren und "
            "ablegen. Bewegt den Roboter.")
        self.btn_teach.clicked.connect(self.start_teaching)
        row.addWidget(self.btn_teach)
        # Tippbetrieb: faehrt nur, solange gedrueckt (bc/gui/jogging.py)
        self.btn_jog_up = QPushButton("▲ Z hoch (halten, F7)")
        self.btn_jog_up.setToolTip(
            "Fährt den Arm senkrecht nach oben, solange der Knopf (oder F7) gedrückt "
            "ist — nur im Teach-Modus und mit Bewegungsfreigabe. Gedacht für PRE_GRASP "
            "genau über PICK: PICK per Touch-up setzen, hochfahren, PRE_GRASP per "
            "Touch-up setzen. Software-Tippbetrieb, kein Zustimmtaster.")
        self.btn_jog_up.pressed.connect(self._jog_pressed)
        self.btn_jog_up.released.connect(self._jog_released)
        row.addWidget(self.btn_jog_up)
        layout.addLayout(row)

        layout.addWidget(self._build_points_box())

        # Ablauf im Detail (Schritte, Teach-Serie): eingeklappt -- fuer den
        # Alltag reicht die Punkte-Liste darueber (Anwender 2026-10-26).
        self.btn_detail = QToolButton()
        self.btn_detail.setText("Ablauf im Detail (Schritte, Überschleifen, Teach-Serie)")
        self.btn_detail.setCheckable(True)
        self.btn_detail.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        self.btn_detail.setArrowType(Qt.ArrowType.RightArrow)
        self.btn_detail.toggled.connect(self._toggle_detail)
        layout.addWidget(self.btn_detail)
        self.detail = QWidget()
        detail_layout = QVBoxLayout(self.detail)
        detail_layout.setContentsMargins(0, 0, 0, 0)
        self.detail.setVisible(False)

        # Teach-Serie: POINT am Arm (bzw. F8) setzt die markierte Zeile und
        # springt weiter (bc/teach_series.py)
        row = QHBoxLayout()
        self.btn_series = QPushButton("Teach-Serie (POINT am Arm)")
        self.btn_series.setCheckable(True)
        self.btn_series.setToolTip(
            "Ab der markierten Zeile: am Arm FREE halten und führen, POINT drücken — "
            "der Punkt der markierten Zeile wird per Touch-up gesetzt, danach springt "
            "die Markierung weiter. Keine Rückfrage je Punkt: am Ende der Serie "
            "einmal behalten oder alle zurücknehmen. Voraussetzung am Pendant: "
            "POINT = \"Save Point\", FREE = Zero-G. Tasten: F6 Serie an/aus, "
            "F8 Touch-up + weiter, Bild↓/Bild↑ Zeile weiter/zurück, F7 halten = Z hoch.")
        self.btn_series.toggled.connect(self._series_toggled)
        row.addWidget(self.btn_series)
        self.series_info = _hint("Tasten: F8 Touch-up + weiter · Bild↓/↑ Zeile · "
                                 "F7 halten Z hoch · F6 Serie")
        row.addWidget(self.series_info, 1)
        detail_layout.addLayout(row)
        # Grosse Anzeige fuer den Blick vom Arm aus: was zuletzt gesetzt
        # wurde, was als Naechstes kommt, Zero-G an/aus.
        self.series_banner = QLabel("")
        self.series_banner.setWordWrap(True)
        self.series_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        banner_font = QFont()
        banner_font.setPointSize(18)
        banner_font.setBold(True)
        self.series_banner.setFont(banner_font)
        self.series_banner.setVisible(False)
        layout.addWidget(self.series_banner)
        self._banner_last = ("", NEUTRAL)
        #: Gesetzte Punkte der laufenden Serie (teach_series.entry) -- fuer
        #: die Schlussabfrage "behalten / alle zuruecknehmen".
        self._series_entries = []
        #: Zero-G laut Steuerung (True/False/None = nicht abfragbar).
        self._free_drive = None
        for key, slot in ((Qt.Key.Key_F6, self.btn_series.toggle),
                          (Qt.Key.Key_F8, self._hotkey_touch_up),
                          (Qt.Key.Key_PageDown, lambda: self._step_row(1)),
                          (Qt.Key.Key_PageUp, lambda: self._step_row(-1))):
            shortcut = QShortcut(QKeySequence(key), self)
            shortcut.setContext(Qt.ShortcutContext.WindowShortcut)
            shortcut.activated.connect(slot)

        self.steps = QTableWidget(0, 9)
        self.steps.setHorizontalHeaderLabels(
            ["Punkt", "Bewegung", "Überschleifen (mm)", "optional", "Anflug", "Greifer",
             "berechnet über", "Abstand (mm)", "Touch-up"])
        self.steps.verticalHeader().setVisible(False)
        self.steps.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self.steps.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection)
        # Alle Zeilen zeigen statt eines zweiten Bildlaufs im Bildlauf
        # (Wunsch Anwender 2026-10-21) -- Hoehe folgt der Zeilenzahl.
        self.steps.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        header = self.steps.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        self.steps.itemChanged.connect(self._on_steps_edited)
        self.steps.currentCellChanged.connect(lambda *_args: self._update_series_info())
        detail_layout.addWidget(self.steps)

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
        buttons.addSpacing(12)
        self.btn_undo_touchup = QPushButton("Touch-up rückgängig")
        self.btn_undo_touchup.setToolTip(
            "Setzt den zuletzt per Touch-up geänderten Punkt auf den gesicherten "
            "Stand zurück (bzw. löscht ihn, wenn er neu angelegt wurde).")
        self.btn_undo_touchup.setEnabled(False)
        self.btn_undo_touchup.clicked.connect(self.undo_touch_up)
        buttons.addWidget(self.btn_undo_touchup)
        buttons.addStretch(1)
        detail_layout.addLayout(buttons)
        layout.addWidget(self.detail)

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
        self._known_cache = known
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

                # Berechneter Punkt (bc/sequence.py, Festlegung 2026-10-26:
                # nur PICK teachen): leer = geteacht, sonst Bezugspunkt
                above = block_wheel(QComboBox())
                above.setEditable(True)
                above.addItem("")
                for name in dict.fromkeys(s["point"] for s in steps if s["point"]):
                    above.addItem(name)
                above.setCurrentText(step.get("above") or "")
                above.setToolTip(
                    "Leer = Punkt wird geteacht. Sonst wird er berechnet: so viele mm "
                    "senkrecht über diesem Punkt, gleiche Orientierung, Gelenke per IK. "
                    "Ein Name braucht die Angabe nur in einer Zeile.")
                above.currentTextChanged.connect(self._on_steps_edited)
                self.steps.setCellWidget(row, 6, above)

                dz = block_wheel(QDoubleSpinBox())
                dz.setRange(0.0, 500.0)
                dz.setDecimals(0)
                dz.setSingleStep(5.0)
                dz.setValue(1e3 * float(step.get("dz") or 0.0))
                dz.setToolTip("Abstand senkrecht über dem Bezugspunkt in mm "
                              "(nur bei berechneten Punkten).")
                dz.valueChanged.connect(self._on_steps_edited)
                self.steps.setCellWidget(row, 7, dz)

                touch = QPushButton("Touch-up")
                touch.setToolTip(
                    "Aktuelle Stellung des Arms (mit aktivem Werkzeug) als diesen "
                    "Punkt speichern -- legt ihn an, wenn er fehlt. Bewegt nichts, "
                    "fragt vorher nach.")
                touch.clicked.connect(
                    lambda _checked=False, button=touch: self.touch_up_row(self._row_of(button, 8)))
                self.steps.setCellWidget(row, 8, touch)
            self.steps.resizeColumnsToContents()
            self.steps.horizontalHeader().setSectionResizeMode(
                0, QHeaderView.ResizeMode.Stretch)
        finally:
            self._loading_steps = False
        self._fit_steps_height()
        self._refresh_sequence_info()
        self._fill_points()

    def _fit_steps_height(self):
        """Tabelle so hoch wie ihr Inhalt -- gescrollt wird nur der Reiter."""
        table = self.steps
        height = table.horizontalHeader().height() + 2 * table.frameWidth()
        for row in range(table.rowCount()):
            height += table.rowHeight(row)
        if table.horizontalScrollBar().isVisible():
            height += table.horizontalScrollBar().height()
        # Leere Tabelle: Platz fuer eine Zeile, damit sie nicht verschwindet
        if table.rowCount() == 0:
            height += table.verticalHeader().defaultSectionSize()
        table.setFixedHeight(height)

    def current_steps(self):
        """Tabelle -> Schritte als dicts."""
        steps = []
        for row in range(self.steps.rowCount()):
            checks = []
            for column in (3, 4):
                holder = self.steps.cellWidget(row, column)
                checks.append(holder.findChild(QCheckBox).isChecked()
                              if holder else False)
            above = self.steps.cellWidget(row, 6).currentText().strip()
            steps.append({
                "point": self.steps.cellWidget(row, 0).currentText().strip(),
                "motion": self.steps.cellWidget(row, 1).currentText(),
                "blend": self.steps.cellWidget(row, 2).value() / 1e3,
                "optional": checks[0],
                "approach": checks[1],
                "gripper": self.steps.cellWidget(row, 5).currentData(),
                "above": above or None,
                "dz": self.steps.cellWidget(row, 7).value() / 1e3 if above else 0.0,
            })
        return steps

    def _computed_names(self):
        """Namen berechneter Punkte in der Tabelle -- die werden nicht geteacht."""
        return {s["point"] for s in self.current_steps() if s["above"]}

    def _on_steps_edited(self, *_args):
        """Signal aus der Tabelle: ab jetzt weicht sie von der Datei ab."""
        if self._loading_steps:
            return
        self._steps_dirty = True
        self._refresh_sequence_info()
        if not self._points_editing:
            self._fill_points()

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
            self._sequence_name, self._gripper_start, steps, self._table_cfg)
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
            "computed": ["%s = %s + %.0f mm" % (s["point"], s["above"], 1e3 * s["dz"])
                         for s in steps if s["above"]],
            "gripper_start": self._gripper_start,
        })
        if self._steps_dirty:
            summary += "\n\nGeändert — noch nicht gespeichert. Aufgezeichnet "
            summary += "wird der gespeicherte Stand."
        self.sequence_info.setText(summary)
        self.sequence_info.setStyleSheet(
            "color:%s" % (WARN if self._steps_dirty else NEUTRAL))

    def _row_of(self, widget, column):
        """Zeile, in der ``widget`` steht -- Zeilen wandern beim Verschieben."""
        for row in range(self.steps.rowCount()):
            if self.steps.cellWidget(row, column) is widget:
                return row
        return -1

    # -- Punkte-Liste (bc/gui/points.py) ------------------------------------

    def _build_points_box(self):
        """Eine Zeile je Punkt statt je Schritt (Wunsch Anwender 2026-10-26):
        PICK je Lage teachen, CLEAR_FOV/PRE_PLACE einmal, berechnete Punkte
        nur in der Hoehe. Aenderungen gehen in die Ablauftabelle und werden
        sofort in der Arbeitskopie gespeichert."""
        box = QGroupBox("Punkte")
        layout = QVBoxLayout(box)
        row = QHBoxLayout()
        self.btn_teach_pick = QPushButton("PICK teachen")
        font = QFont()
        font.setBold(True)
        self.btn_teach_pick.setFont(font)
        self.btn_teach_pick.setToolTip(
            "Schaltet die Steuerung in den Teach-Modus und wartet auf POINT am Arm: "
            "FREE halten, Greifer an das Objekt führen, POINT drücken. Danach ist "
            "PICK gesetzt; PRE_GRASP und ANFLUG rechnen sich daraus.")
        self.btn_teach_pick.clicked.connect(lambda: self.teach_point("PICK"))
        row.addWidget(self.btn_teach_pick)
        self.align_after_teach = QCheckBox("danach ausrichten")
        self.align_after_teach.setChecked(True)
        self.align_after_teach.setToolTip(
            "Nach „PICK teachen“: Greifer senkrecht zur Tischebene stellen und auf die "
            "Greifhöhe über dem Tisch setzen; die Drehung um die Hochachse bleibt. "
            "Ohne Tischpunkte nur senkrecht, Höhe bleibt.")
        row.addWidget(self.align_after_teach)
        self.btn_add_keepout = QPushButton("+ Sperrzone")
        self.btn_add_keepout.setToolTip(
            "Neue Sperrzone anlegen: Namen vergeben, dann in ihrer Zeile „Teachen“ "
            "(Mitte der Zone, nahe am Tisch) und die Größe ±mm einstellen. Dort wird "
            "weder gegriffen noch abgelegt.")
        self.btn_add_keepout.clicked.connect(self._add_keepout)
        row.addWidget(self.btn_add_keepout)
        row.addWidget(_hint("Je Lage nur PICK — danach Probefahrt. „Anfahren“ kommt "
                            "immer senkrecht von oben."), 1)
        layout.addLayout(row)
        self.point_table = QTableWidget(0, 9)
        self.point_table.setHorizontalHeaderLabels(
            ["Punkt", "Art", "Höhe mm", "Greifer", "Status", "", "", "", ""])
        self.point_table.verticalHeader().setVisible(False)
        self.point_table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.point_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.point_table.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        layout.addWidget(self.point_table)
        return box

    def _toggle_detail(self, on):
        self.detail.setVisible(on)
        self.btn_detail.setArrowType(Qt.ArrowType.DownArrow if on else Qt.ArrowType.RightArrow)

    def _fill_points(self):
        if not hasattr(self, "point_table"):
            return
        cfg = self._table_cfg or {}
        rows = (points_module.point_rows(
            self.current_steps(), table_points=list(cfg.get("points") or []),
            keepout_points=[n for n, _s in self._keepout_entries()])
            if self._sequence_working else [])
        table = self.point_table
        table.setRowCount(len(rows))
        known = set(self._known_cache)
        connected = self.session.robot is not None
        for index, row in enumerate(rows):
            name = row["name"]
            item = QTableWidgetItem(name)
            if row["scope"] == "je Lage":
                bold = QFont()
                bold.setBold(True)
                item.setFont(bold)
            table.setItem(index, 0, item)
            if row["kind"] == points_module.COMPUTED:
                kind, tip = "über %s" % row["above"], "berechnet, nicht teachen"
            elif row["kind"] == points_module.TABLE:
                kind, tip = "Tisch", ("1 mm über dem Tisch teachen, mittig über einem "
                                      "ArUco-Marker -- Tischebene und Untergrenze")
            elif row["kind"] == points_module.KEEPOUT:
                kind, tip = "Sperre", (
                    "Mitte einer Sperrzone (±mm in der Spalte daneben): dort wird nicht "
                    "gegriffen und nicht abgelegt, das Werkzeug bleibt %.0f mm über dem Tisch"
                    % (1e3 * config.KEEPOUT_MIN_HEIGHT_M))
            else:
                kind = row["scope"]
                tip = ("je Objektlage neu teachen" if kind == "je Lage"
                       else "einmal für den ganzen Datensatz teachen")
            kind_item = QTableWidgetItem(kind)
            kind_item.setToolTip(tip)
            table.setItem(index, 1, kind_item)

            if row["kind"] == points_module.COMPUTED:
                height = block_wheel(QDoubleSpinBox())
                height.setRange(5.0, 500.0)
                height.setDecimals(0)
                height.setSingleStep(5.0)
                height.setValue(1e3 * row["dz"])
                height.setToolTip("Abstand senkrecht über %s in mm" % row["above"])
                height.valueChanged.connect(
                    lambda value, n=name: self._point_height_changed(n, value))
                table.setCellWidget(index, 2, height)
            elif row["kind"] == points_module.KEEPOUT:
                size = block_wheel(QDoubleSpinBox())
                size.setRange(5.0, 300.0)
                size.setDecimals(0)
                size.setSingleStep(5.0)
                size.setPrefix("±")
                size.setValue(1e3 * self._keepout_size(name))
                size.setToolTip("Halbe Kantenlänge der Sperrzone in mm (Quadrat um den Punkt)")
                size.valueChanged.connect(
                    lambda value, n=name: self._keepout_size_changed(n, value))
                table.setCellWidget(index, 2, size)
            elif name == "PICK" and self._table_cfg is not None:
                # Greifhoehe ueber dem Tisch -- gilt fuer "Ausrichten"
                height = block_wheel(QDoubleSpinBox())
                height.setRange(2.0, 100.0)
                height.setDecimals(1)
                height.setSingleStep(0.5)
                height.setValue(1e3 * float(self._table_cfg.get(
                    "pick_height", config.PICK_HEIGHT_DEFAULT_M)))
                height.setToolTip("Greifhöhe über dem Tisch in mm (für „Ausrichten“)")
                height.valueChanged.connect(self._pick_height_changed)
                table.setCellWidget(index, 2, height)
            else:
                table.setItem(index, 2, QTableWidgetItem(""))

            if row["kind"] in (points_module.TABLE, points_module.KEEPOUT):
                table.setItem(index, 3, QTableWidgetItem("—"))
            elif row["gripper"] == "mehrfach":
                many = QTableWidgetItem("mehrfach")
                many.setToolTip("Punkt kommt mehrmals vor -- Greifer im Detail einstellen")
                table.setItem(index, 3, many)
            else:
                gripper = block_wheel(QComboBox())
                gripper.addItem("—", None)
                gripper.addItem("auf", "open")
                gripper.addItem("zu", "close")
                found = find_data(gripper, row["gripper"])
                gripper.setCurrentIndex(found if found >= 0 else 0)
                gripper.setToolTip("Greiferbefehl, wenn dieser Punkt erreicht ist")
                gripper.currentIndexChanged.connect(
                    lambda _i, n=name, box=gripper: self._point_gripper_changed(n, box.currentData()))
                table.setCellWidget(index, 3, gripper)

            if row["kind"] == points_module.COMPUTED:
                state = "—"
            elif not connected:
                state = "offline"
            else:
                state = "✓" if name in known else "✗ fehlt"
            table.setItem(index, 4, QTableWidgetItem(state))

            for column, text, slot, taught_only in (
                    (5, "Teachen", self.teach_point, True),
                    (6, "Touch-up", self._touch_up_point, True),
                    (7, "Anfahren", self.move_to_point, False),
                    (8, "Ausrichten", self.align_point, True)):
                if column == 8 and row["kind"] == points_module.KEEPOUT:
                    remove = QPushButton("Entfernen")
                    remove.clicked.connect(lambda _c=False, n=name: self._remove_keepout(n))
                    table.setCellWidget(index, column, remove)
                    continue
                if column == 8 and row["scope"] != "je Lage":
                    table.removeCellWidget(index, column)
                    table.setItem(index, column, QTableWidgetItem(""))
                    continue
                if taught_only and row["kind"] == points_module.COMPUTED:
                    table.removeCellWidget(index, column)
                    table.setItem(index, column, QTableWidgetItem(""))
                    continue
                button = QPushButton(text)
                button.clicked.connect(lambda _checked=False, n=name, f=slot: f(n))
                table.setCellWidget(index, column, button)
        table.resizeColumnsToContents()
        # Feste Breiten fuer die Bedienelemente, damit die ganze Liste in die
        # linke Spalte passt (Anwender 2026-10-26)
        for column, width in ((2, 70), (3, 72), (5, 72), (6, 72), (7, 76), (8, 80)):
            table.setColumnWidth(column, width)
        height = table.horizontalHeader().height() + 2 * table.frameWidth()
        for index in range(table.rowCount()):
            height += table.rowHeight(index)
        table.setFixedHeight(max(height, table.horizontalHeader().height() + 30))

    def _rows_of(self, name):
        return [r for r in range(self.steps.rowCount())
                if self.steps.cellWidget(r, 0).currentText().strip() == name]

    def _point_height_changed(self, name, millimetres):
        self._points_editing = True
        try:
            for r in self._rows_of(name):
                if self.steps.cellWidget(r, 6).currentText().strip():
                    self.steps.cellWidget(r, 7).setValue(millimetres)
        finally:
            self._points_editing = False
        self._autosave_points()

    def _point_gripper_changed(self, name, value):
        rows = self._rows_of(name)
        if len(rows) != 1:
            return
        self._points_editing = True
        try:
            box = self.steps.cellWidget(rows[0], 5)
            found = find_data(box, value)
            box.setCurrentIndex(found if found >= 0 else 0)
        finally:
            self._points_editing = False
        self._autosave_points()

    def _autosave_points(self):
        """Aenderungen aus der Punkte-Liste sofort in die Arbeitskopie --
        aufgezeichnet wird der gespeicherte Stand."""
        problems = recording_module.validate_steps(
            self._sequence_name, self._gripper_start, self.current_steps(),
            self._table_cfg)
        if problems:
            self._refresh_sequence_info()
            return
        self.save_sequence()

    def _pick_height_changed(self, millimetres):
        if self._table_cfg is None:
            return
        self._table_cfg = dict(self._table_cfg, pick_height=round(millimetres / 1e3, 4))
        self._autosave_points()

    # -- Sperrzonen ("table": {"keepout": [...]}, bc/table.Keepout) -----------

    def _keepout_entries(self):
        """[(Name, halbe Kantenlaenge oder None)] aus der Ablaufdatei."""
        out = []
        for entry in (self._table_cfg or {}).get("keepout") or []:
            if isinstance(entry, dict):
                out.append((entry.get("point"), entry.get("size")))
            else:
                out.append((entry, None))
        return out

    def _set_keepout_entries(self, entries):
        cfg = dict(self._table_cfg or {"points": []})
        cfg["keepout"] = [name if size is None else {"point": name, "size": round(size, 4)}
                          for name, size in entries]
        self._table_cfg = cfg
        self._autosave_points()
        self._fill_points()

    def _keepout_size(self, name):
        default = float((self._table_cfg or {}).get("keepout_size", config.KEEPOUT_HALF_SIZE_M))
        for entry, size in self._keepout_entries():
            if entry == name and size is not None:
                return float(size)
        return default

    def _keepout_size_changed(self, name, millimetres):
        entries = [(n, millimetres / 1e3 if n == name else s) for n, s in self._keepout_entries()]
        cfg = dict(self._table_cfg or {})
        cfg["keepout"] = [n if s is None else {"point": n, "size": round(s, 4)} for n, s in entries]
        self._table_cfg = cfg
        self._autosave_points()   # Tabelle bleibt -- das Feld behaelt den Fokus

    def _add_keepout(self):
        from PySide6.QtWidgets import QInputDialog

        if not self._sequence_working:
            return
        existing = {n for n, _s in self._keepout_entries()}
        k = 1
        while "SPERRE_%d" % k in existing:
            k += 1
        name, ok = QInputDialog.getText(self, "Sperrzone", "Name des Punkts in der Steuerung:",
                                        text="SPERRE_%d" % k)
        name = (name or "").strip()
        if not ok or not name:
            return
        if name in existing or name in self._table_names() or self._rows_of(name):
            QMessageBox.information(self, "Sperrzone", "%s gibt es schon." % name)
            return
        self._set_keepout_entries(self._keepout_entries() + [(name, None)])
        self.output.append_record("Sperrzone %s angelegt -- jetzt in ihrer Zeile „Teachen“." % name)

    def _remove_keepout(self, name):
        if not self.confirm("Sperrzone entfernen",
                            "Sperrzone %s aus dem Ablauf nehmen? Der Punkt bleibt in der "
                            "Steuerung." % name):
            return
        self._set_keepout_entries([(n, s) for n, s in self._keepout_entries() if n != name])

    def _table_names(self):
        """Tisch- und Sperrpunkte: geteacht, aber nicht Teil der Bahn."""
        cfg = self._table_cfg or {}
        return list(cfg.get("points") or []) + [n for n, _s in self._keepout_entries()]

    def _touch_up_point(self, name):
        rows = self._rows_of(name)
        if rows:
            self.touch_up_row(rows[0])
        elif name in self._table_names():
            self.touch_up_row(-1, name=name)
        self._fill_points()

    def teach_point(self, name):
        """Teach-Modus und auf POINT am Arm warten -- fuer genau diesen Punkt."""
        robot = self.session.robot
        if robot is None:
            QMessageBox.information(self, "Teachen", "Erst mit der Steuerung verbinden (Leiste oben).")
            return
        if self.process.running or self._moving:
            QMessageBox.information(self, "Teachen", "Erst die laufende Aufnahme bzw. Fahrt beenden.")
            return
        rows = self._rows_of(name)
        if (not rows and name not in self._table_names()) or name in self._computed_names():
            QMessageBox.information(self, "Teachen",
                                    "%s steht nicht als geteachter Punkt im Ablauf." % name)
            return
        if hasattr(robot, "switch_to_teach") and robot.in_teach_mode() is not True:
            try:
                state = robot.switch_to_teach()
            except Exception as exc:
                state = exc
            if state is not True:
                self.output.append_record(
                    "Teach-Modus nicht geschaltet (%s) -- am Pendant umschalten, dann POINT."
                    % state, logging.WARNING)
            else:
                self.output.append_record("Steuerung steht im Teach-Modus.")
        if self._series_watcher is not None:
            self._stop_series("neuer Punkt gewählt", ask=False)
        self.steps.setCurrentCell(rows[0] if rows else 0, 0)
        self.btn_series.setChecked(True)
        if self._series_watcher is None:
            return  # Serie liess sich nicht starten, Meldung kam schon
        self._series_single = name
        self.output.append_record(
            "%s teachen: am Arm FREE halten und führen, dann POINT drücken (oder F8)." % name)
        self._update_series_info()

    def move_to_point(self, name):
        """Gespeicherten Punkt langsam anfahren (PTP), an PICK von oben."""
        from ..sequence import parse_sequence, resolve_sequence

        robot = self.session.robot
        if robot is None:
            QMessageBox.information(self, "Anfahren", "Erst mit der Steuerung verbinden (Leiste oben).")
            return
        reason = self.busy_reason()
        if reason or self._series_watcher is not None:
            QMessageBox.information(self, "Anfahren", "Erst beenden: %s" % (
                reason or "Teach-Serie läuft"))
            return
        status = self.session.status()
        if not status.motion_allowed:
            QMessageBox.information(
                self, "Anfahren", "Bewegung nicht freigegeben -- an der Anlage in der "
                "Leiste oben \"Reale Anlage freigeben\" setzen und neu verbinden.")
            return
        try:
            sequence = parse_sequence(recording_module.steps_to_data(
                self._sequence_name, self._gripper_start, self.current_steps(),
                self._table_cfg))
            resolved = resolve_sequence(sequence, robot)
            current = robot.read_state().joints
        except Exception as exc:  # SequenceError, IK, Verbindung
            QMessageBox.warning(self, "Anfahren", "Punkte nicht auflösbar:\n%s" % exc)
            return
        try:
            targets = self._approach_targets(name, robot, resolved, sequence, current)
        except Exception as exc:
            QMessageBox.warning(self, "Anfahren", "Weg nicht berechenbar:\n%s" % exc)
            return
        path = [label for label, _joints in targets]
        if not path:
            self.output.append_record("Arm steht schon an %s." % name)
            return
        lines = ["Fährt per PTP, langsam: %s" % " → ".join(path),
                 "Override %.2f · Backend: %s" % (self.session.override, status.text)]
        if hasattr(robot, "prepare_motion"):
            lines.append("Steht die Steuerung in Teach, wird auf Automatik geschaltet "
                         "(Zero-G muss aus sein).")
        if status.is_real_plant:
            lines.append("ACHTUNG: REALE ANLAGE.")
        lines += ["", "Zwischen den Punkten wird nichts geprüft außer dem Ziel. "
                      "Arbeitsraum frei? Not-Aus in Reichweite? STOPP bricht ab."]
        if not self.confirm("%s anfahren" % name, "\n".join(lines)):
            return

        def job():
            if hasattr(robot, "prepare_motion"):
                robot.prepare_motion()
            elif robot.stop_requested:
                robot.clear_stop()
            for point, joints in targets:
                if robot.stop_requested:
                    raise RuntimeError("gestoppt vor %s" % point)
                robot.move_to_joints(joints)
            return name

        self._set_moving(True)
        self.output.append_record("Anfahren: %s" % " → ".join(path))
        self.runner.submit(job, on_done=self._move_done, on_error=self._move_failed,
                           pool="robot")

    def _approach_targets(self, name, robot, resolved, sequence, current, tol=0.02):
        """(Bezeichnung, Gelenke) fuer "Anfahren": Bahnpunkte ueber
        points.approach_path; Tischpunkte immer senkrecht von oben und, wenn
        der Arm an einem steht, erst senkrecht hinauf -- 1 mm ueber dem Tisch
        seitlich loszufahren kratzte ueber Tisch und Marker."""
        table_points = dict(resolved.table_points, **resolved.keepout_points)

        def above(point):
            data = table_points[point]
            pose = np.asarray(data["pose_quat"], dtype=float).copy()
            pose[2] += config.TABLE_APPROACH_ABOVE_M
            return ("über %s" % point, robot.ik(pose, data["joints"]))

        current = np.asarray(current, dtype=float)
        prefix = []
        for point, data in table_points.items():
            if np.max(np.abs(np.asarray(data["joints"]) - current)) <= tol:
                if point == name:
                    return []
                prefix = [above(point)]
                current = np.asarray(prefix[0][1], dtype=float)
                break
        if name in table_points:
            return prefix + [above(name), (name, table_points[name]["joints"])]
        path = points_module.approach_path(name, resolved.points, sequence.computed, current)
        return prefix + [(n, resolved.points[n]["joints"]) for n in path]

    def align_point(self, name="PICK", ask=True):
        """PICK ausrichten (bc/table.align_pose): senkrecht zur Tischebene,
        Greifhoehe darueber, Drehung um die Hochachse bleibt. Schreibt per
        Touch-up (mit Sicherung) -- bewegt nichts."""
        from ..sequence import parse_sequence, resolve_sequence
        from .. import table as table_module

        robot = self.session.robot
        if robot is None:
            QMessageBox.information(self, "Ausrichten", "Erst mit der Steuerung verbinden.")
            return False
        try:
            sequence = parse_sequence(recording_module.steps_to_data(
                self._sequence_name, self._gripper_start, self.current_steps(),
                self._table_cfg))
            resolved = resolve_sequence(sequence, robot)
            point = resolved.points[name]
            plane = resolved.table
            aligned = table_module.align_pose(
                point["pose_quat"], plane, sequence.pick_height if plane is not None else None)
            joints = np.asarray(robot.ik(aligned, point["joints"]), dtype=float)
        except Exception as exc:  # SequenceError, IK, Verbindung
            QMessageBox.warning(self, "Ausrichten", "%s nicht ausrichtbar:\n%s" % (name, exc))
            return False
        jump = float(np.max(np.abs(joints - np.asarray(point["joints"]))))
        if jump > config.COMPUTED_POINT_MAX_JUMP_RAD:
            QMessageBox.warning(self, "Ausrichten", "IK springt %.2f rad -- anderer "
                                "Gelenkzweig? %s von Hand nachteachen." % (jump, name))
            return False
        before = (table_module.tilt_deg(point["pose_quat"], plane),
                  1e3 * (table_module.clearance(plane, point["pose_quat"])
                         if plane is not None else point["pose_quat"][2]))
        note = ("%s: Neigung %.2f° → 0°, %s %.1f → %.1f mm"
                % (name, before[0], "über Tisch" if plane is not None else "z",
                   before[1], 1e3 * (sequence.pick_height if plane is not None
                                     else point["pose_quat"][2])))
        if plane is None:
            note += " (ohne Tischpunkte: nur senkrecht, Höhe bleibt)"
        self.output.append_record("Ausrichten " + note)
        ok = self.touch_up_row(-1, joints=joints, series=not ask, name=name)
        if ok:
            self.output.append_record("%s ausgerichtet." % name)
        self._fill_points()
        return ok

    def _set_moving(self, moving):
        self._moving = moving
        for widget in (self.btn_record, self.btn_teach, self.point_table,
                       self.btn_teach_pick, self.btn_series, self.btn_jog_up):
            widget.setEnabled(not moving)
        self.btn_stop.setEnabled(moving or self.process.running)

    def _move_done(self, name):
        self._set_moving(False)
        self.output.append_record("%s erreicht." % name)

    def _move_failed(self, error):
        self._set_moving(False)
        self.output.append_record("Anfahren abgebrochen: %s" % error, logging.WARNING)

    # -- Grossansicht (Doppelklick aufs Bild) -------------------------------

    def _camera_names_shown(self):
        if self._captures:
            return [c.name for c in self._captures]
        return list(self._live_names)

    def _zoom_at(self, x):
        names = self._camera_names_shown()
        if not names:
            return
        name = names[min(int(x * len(names)), len(names) - 1)]
        if self._zoom is not None:
            self._zoom[0].close()
        dialog = QDialog(self)
        dialog.setWindowTitle("%s — groß (Doppelklick schließt)" % name)
        dialog.resize(1000, 780)
        view = ImageView("wartet auf das nächste Bild …")
        view.double_clicked.connect(lambda _x: dialog.close())
        box = QVBoxLayout(dialog)
        box.setContentsMargins(0, 0, 0, 0)
        box.addWidget(view)
        dialog.view = view
        dialog.finished.connect(lambda _r: setattr(self, "_zoom", None))
        self._zoom = (dialog, name)
        dialog.show()

    def _feed_zoom(self, frames=None, board=None):
        if self._zoom is None:
            return
        from .. import preview as preview_module

        dialog, name = self._zoom
        if frames is not None:
            frame = frames.get(name)
            if frame is None or getattr(frame, "image", None) is None:
                return
            dialog.view.set_image(preview_module.compose(
                {name: frame}, height=frame.image.shape[0]))
        elif board is not None:
            names = self._camera_names_shown()
            if name not in names:
                return
            width = board.shape[1] // len(names)
            index = names.index(name)
            dialog.view.set_image(board[:, index * width:(index + 1) * width].copy())

    # -- Metadaten-Auswahl (bc/gui/vocabulary.py) ---------------------------

    def _vocabulary_box(self, entries):
        box = block_wheel(QComboBox())
        box.setEditable(True)
        box.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        box.addItem("")
        for entry in entries:
            box.addItem(entry)
        box.setCurrentIndex(0)
        return box

    def _on_new_pose(self):
        """PICK neu gesetzt = neue Objektlage: Block hochzaehlen (einmal je
        Lage -- "Ausrichten" direkt nach dem Teachen zaehlt nicht doppelt)
        und Lage/Drehung ins Protokoll. In die Metadaten schreibt record.py
        Position und Drehung selbst (object_xy_m, object_yaw_deg)."""
        if self._block_used:
            old = self.block.text().strip()
            self.block.setText(self._next_free_block or recording_module.next_block(old))
            self._next_free_block = None
            self._block_used = False
            self.output.append_record("Neue Lage: Block %s → %s" % (old or "—", self.block.text()))
        robot = self.session.robot
        try:
            from .. import table as table_module

            pose = robot.get_point("PICK")[1]
            self.output.append_record(
                "PICK: x %.0f mm, y %.0f mm, Drehung %.0f°"
                % (1e3 * pose[0], 1e3 * pose[1], table_module.yaw_deg(pose)))
        except Exception:
            log.debug("PICK nicht lesbar", exc_info=True)

    def _remember_metadata(self):
        """Werte in bekannter Schreibweise uebernehmen und neue merken."""
        fields = {"light": self.light, "camera_pose": self.camera_pose,
                  "object_note": self.object_note}
        try:
            values = vocabulary_module.remember(
                self.workdir, {k: box.currentText() for k, box in fields.items()})
        except OSError as exc:
            log.warning("Metadaten-Auswahl nicht gespeichert: %s", exc)
            return
        for key, box in fields.items():
            value = values.get(key) or ""
            if value and box.findText(value) < 0:
                box.addItem(value)
            box.setCurrentText(value)

    # -- Touch-up (bc/touchup.py) -------------------------------------------

    def touch_up_row(self, row, joints=None, series=False, name=None):
        """Punkt dieser Zeile auf die aktuelle Stellung setzen (Wunsch
        Anwender 2026-10-21). Prueft, fragt, sichert, schreibt -- bewegt nichts.

        ``joints``: Stellung von aussen (Teach-Serie: der Punkt des
        POINT-Knopfs) statt der aktuellen. ``series``: Regeln der Serie --
        keine Rueckfrage je Punkt (Anlage 21.10.: zu laestig), stattdessen
        grosse Anzeige und eine Schlussabfrage am Ende der Serie; Sperren
        und Fehler gehen in Anzeige und Protokoll statt in eine
        Meldungsbox, der Bediener steht am Arm.
        ``name``: Punkt ohne eigene Zeile (Tischpunkt) bzw. das Ziel von
        "Teachen"/"Ausrichten" -- dann zaehlt ``row`` nicht.
        Rueckgabe: True, wenn geschrieben wurde.
        """
        if name is None:
            if row < 0:
                return False
            name = self.steps.cellWidget(row, 0).currentText().strip()
        if name in self._computed_names():
            text = ("%s wird berechnet (Spalte \"berechnet über\") und nicht "
                    "geteacht -- Touch-up hier wirkungslos." % name)
            if series:
                self.output.append_record(text, logging.WARNING)
            else:
                QMessageBox.information(self, "Touch-up", text)
            return False
        robot = self.session.robot
        if robot is None:
            QMessageBox.information(
                self, "Touch-up", "Erst mit der Steuerung verbinden (Leiste oben).")
            return False
        if self.process.running:
            QMessageBox.information(
                self, "Touch-up", "Während einer Aufnahme nicht möglich.")
            return False
        status = self.session.status()
        in_simulation = (True if status.state == session_module.SIM_LOCAL
                         else getattr(robot, "in_simulation", None))
        try:
            plan = touchup_module.plan_touch_up(
                robot, name, RealClock(), in_simulation, status.tool_name, joints=joints)
        except Exception as exc:
            log.exception("Touch-up: Stellung nicht lesbar")
            QMessageBox.warning(self, "Touch-up", "Stellung nicht lesbar:\n%s" % exc)
            return False
        text = touchup_module.describe(plan)
        if plan["problems"]:
            if series:
                self._series_failed(name, "; ".join(plan["problems"]))
            else:
                QMessageBox.warning(self, "Touch-up nicht möglich", text)
            return False
        if not series and not self.confirm(
                "Touch-up %s" % name, text + "\n\nStellung als Punkt %s speichern?" % name):
            return False
        try:
            backup, _record = touchup_module.apply_touch_up(
                robot, plan, self.workdir / config.TOUCHUP_BACKUP_ROOT)
        except Exception as exc:
            log.exception("Touch-up gescheitert")
            if series:
                self._series_failed(name, str(exc))
            else:
                QMessageBox.warning(self, "Touch-up gescheitert", str(exc))
            return False
        self._last_touchup = backup
        self._last_touchup_row = row
        if name == "PICK":
            self._on_new_pose()
        self.btn_undo_touchup.setEnabled(True)
        self.output.append_record(
            "Touch-up %s, Sicherung %s" % (series_module.summary(plan), backup))
        if series:
            self._series_entries.append(series_module.entry(plan, backup))
            flagged = series_module.needs_confirm(plan)
            if flagged:
                QApplication.beep()
            self._banner_last = (
                ("⚠ " if flagged else "✔ ") + series_module.summary(plan)
                + ("\n" + " ".join(plan["warnings"]) if flagged else ""),
                WARN if flagged else GOOD)
        self._reload_point_lists()
        return True

    def _series_failed(self, name, why):
        QApplication.beep()
        self.output.append_record(
            "Touch-up %s nicht möglich: %s" % (name or "(leer)", why), logging.WARNING)
        self._banner_last = ("✘ %s NICHT gesetzt\n%s" % (name or "(leer)", why), BAD)
        self._update_series_info()

    def undo_touch_up(self):
        robot = self.session.robot
        if self._last_touchup is None or robot is None:
            return
        if not self.confirm("Touch-up rückgängig",
                            "Sicherung zurückspielen?\n%s" % self._last_touchup):
            return
        force = False
        while True:
            try:
                text = touchup_module.undo_touch_up(robot, self._last_touchup, force=force)
                break
            except Exception as exc:
                # Am Pendant nach dem Touch-up erneut geaendert: nur auf
                # ausdrueckliche Rueckfrage ueberschreiben.
                if not force and "erneut geaendert" in str(exc):
                    if not self.confirm("Touch-up rückgängig",
                                        "%s\n\nTrotzdem zurücksetzen?" % exc):
                        return
                    force = True
                    continue
                log.exception("Ruecknahme gescheitert")
                QMessageBox.warning(self, "Rücknahme gescheitert", str(exc))
                return
        self.output.append_record(text)
        self._last_touchup = None
        self.btn_undo_touchup.setEnabled(False)
        self._reload_point_lists()
        # Zurueck auf die Zeile -- in der Serie ist der naechste Druck dann
        # wieder fuer sie.
        if 0 <= self._last_touchup_row < self.steps.rowCount():
            self.steps.setCurrentCell(self._last_touchup_row, 0)
            self._update_series_info()

    # -- Teach-Serie und Tasten (bc/teach_series.py) ------------------------

    def _series_toggled(self, on):
        if on:
            self._start_series()
        else:
            self._stop_series()

    def _start_series(self):
        robot = self.session.robot
        problem = None
        if robot is None:
            problem = "Erst mit der Steuerung verbinden (Leiste oben)."
        elif self.process.running:
            problem = "Während einer Aufnahme nicht möglich."
        elif self.steps.rowCount() == 0:
            problem = "Der Ablauf hat keine Zeilen."
        if problem is None:
            try:
                self._series_watcher = series_module.NewPointWatcher(robot)
            except Exception as exc:
                log.exception("Teach-Serie: Punktliste nicht lesbar")
                problem = "Punktliste nicht lesbar: %s" % exc
        if problem is not None:
            self._series_watcher = None
            self.btn_series.blockSignals(True)
            self.btn_series.setChecked(False)
            self.btn_series.blockSignals(False)
            QMessageBox.information(self, "Teach-Serie", problem)
            return
        if self.steps.currentRow() < 0:
            self.steps.setCurrentCell(0, 0)
        self._series_entries = []
        self._banner_last = ("", NEUTRAL)
        self._free_drive = None
        self.series_timer.start()
        self.output.append_record(
            "Teach-Serie an: am Arm FREE drücken und führen, POINT drücken (oder F8). "
            "Bestätigt wird einmal am Ende.")
        self._update_series_info()

    def _stop_series(self, reason="", ask=True):
        """Serie beenden. ``ask``: Schlussabfrage "behalten / alle
        zuruecknehmen" -- nur, wenn der Bediener die Serie selbst beendet
        oder sie am Ende ankommt. Bei Reiterwechsel, Aufnahmestart oder
        Verbindungswechsel bleiben die Punkte ohne Frage stehen (Protokoll
        nennt sie, "Touch-up rueckgaengig" nimmt den letzten zurueck)."""
        was_active = self._series_watcher is not None
        robot = self._series_watcher.robot if was_active else None
        self._series_single = None
        self.series_timer.stop()
        self._series_watcher = None
        if self.btn_series.isChecked():
            self.btn_series.blockSignals(True)
            self.btn_series.setChecked(False)
            self.btn_series.blockSignals(False)
        if was_active:
            self.output.append_record(
                "Teach-Serie aus" + (": %s" % reason if reason else ""))
        entries, self._series_entries = self._series_entries, []
        self._update_series_info()
        if not was_active or not entries:
            return
        if not ask or robot is not self.session.robot:
            self.output.append_record(
                "%d Punkt(e) der Serie behalten (ohne Schlussabfrage): %s"
                % (len(entries), ", ".join(e["name"] for e in entries)), logging.WARNING)
            return
        self._review_series(robot, entries)

    def _review_series(self, robot, entries):
        """Schlussabfrage: alles behalten oder alle Touch-ups der Serie zuruecknehmen."""
        box = QMessageBox(self)
        box.setWindowTitle("Teach-Serie übernehmen?")
        box.setIcon(QMessageBox.Icon.Question)
        box.setText(series_module.review_text(entries))
        keep = box.addButton("Behalten", QMessageBox.ButtonRole.AcceptRole)
        undo = box.addButton("Alle zurücknehmen", QMessageBox.ButtonRole.DestructiveRole)
        box.setDefaultButton(keep)
        box.exec()
        if box.clickedButton() is not undo:
            self.output.append_record("Serie übernommen: %d Punkt(e)." % len(entries))
            return
        done, error = series_module.undo_all(robot, entries)
        for text in done:
            self.output.append_record(text)
        self._last_touchup = None
        self.btn_undo_touchup.setEnabled(False)
        self._reload_point_lists()
        if error:
            self.output.append_record("Rücknahme angehalten bei %s" % error, logging.ERROR)
            QMessageBox.warning(
                self, "Rücknahme unvollständig",
                "%d von %d zurückgenommen, angehalten bei\n%s\n\nDie Sicherungen liegen in %s."
                % (len(done), len(entries), error, config.TOUCHUP_BACKUP_ROOT))
        else:
            self.output.append_record("Serie zurückgenommen: %d Punkt(e)." % len(done))

    def _update_series_info(self):
        if not hasattr(self, "series_info"):
            return
        if self._series_watcher is None:
            self.series_info.setText("Tasten: F8 Touch-up + weiter · Bild↓/↑ Zeile · "
                                     "F7 halten Z hoch · F6 Serie")
            self.series_info.setStyleSheet("color:%s" % NEUTRAL)
            self.series_banner.setVisible(False)
            return
        row = self.steps.currentRow()
        name = self.steps.cellWidget(row, 0).currentText().strip() if row >= 0 else "-"
        self.series_info.setText("Serie läuft — %d gesetzt, Bestätigung am Ende (F6)"
                                 % len(self._series_entries))
        self.series_info.setStyleSheet("color:%s; font-weight:bold" % GOOD)

        last, colour = self._banner_last
        lines = []
        if last:
            lines.append('<span style="color:%s">%s</span>'
                         % (colour, last.replace("\n", "<br><small>") + ("</small>" if "\n" in last else "")))
        lines.append("Nächster POINT → <b>%s</b> (Zeile %d/%d)"
                     % (name or "(leer)", row + 1, self.steps.rowCount()))
        if self._free_drive is False:
            lines.append('<span style="color:%s">Zero-G AUS — erst FREE drücken, '
                         'dann den Arm führen</span>' % BAD)
        elif self._free_drive is True:
            lines.append('<span style="color:%s"><small>Zero-G an — vor Automatik am Pendant '
                         'ausschalten</small></span>' % NEUTRAL)
        self.series_banner.setText("<br>".join(lines))
        self.series_banner.setVisible(True)

    def _read_free_drive(self, robot):
        """Zero-G laut Steuerung -- None, wenn der Roboter es nicht kennt."""
        probe = getattr(robot, "free_drive_enabled", None)
        if probe is None:
            return None
        try:
            return probe()
        except Exception:
            return None

    def _table_point_names(self):
        return [s["point"] for s in self.current_steps() if s["point"]]

    def _series_tick(self):
        watcher = self._series_watcher
        if watcher is None or self._series_busy:
            return
        if self.process.running or self.session.robot is not watcher.robot:
            self._stop_series("Aufnahme läuft oder Verbindung gewechselt", ask=False)
            return
        # Nicht waehrend "Z hoch": dessen 50-Hz-Takt geht vor
        if self._jogger is not None:
            return
        self._series_busy = True
        try:
            # Anlage 21.10.: nach "Z hoch" war Zero-G aus, das Fuehren loeste
            # einen Not-Halt aus -- deshalb gross anzeigen.
            free_drive = self._read_free_drive(watcher.robot)
            if free_drive != self._free_drive:
                self._free_drive = free_drive
                if free_drive is False:
                    QApplication.beep()
                self._update_series_info()
            try:
                watcher.poll(protected=self._table_point_names())
            except Exception as exc:
                log.exception("Teach-Serie: Punktliste nicht lesbar")
                self._stop_series("Punktliste nicht lesbar: %s" % exc, ask=False)
                return
            if watcher.pending:
                self._series_take(watcher.pending.pop(0))
        finally:
            self._series_busy = False

    def _series_take(self, pressed):
        """Ein POINT-Druck: Gelenke des neuen Punkts in die markierte Zeile,
        neuen Punkt loeschen, weiter. Scheitert der Touch-up, ist der Druck
        verworfen -- einfach noch einmal druecken."""
        robot = self.session.robot
        row = self.steps.currentRow()
        try:
            joints = series_module.joints_of(robot, pressed)
        except Exception as exc:
            log.exception("Teach-Serie: Punkt %s nicht lesbar", pressed)
            self.output.append_record(
                "POINT-Punkt %s nicht lesbar: %s" % (pressed, exc), logging.WARNING)
            return
        self.output.append_record("POINT gedrückt (Steuerung legte %s an)" % pressed)
        ok = self.touch_up_row(row, joints=joints, series=True, name=self._series_single)
        try:
            if series_module.consume_point(robot, pressed):
                if self._series_watcher is not None:
                    self._series_watcher.forget(pressed)
                self._reload_point_lists()
        except Exception as exc:
            log.exception("Teach-Serie: Punkt %s nicht loeschbar", pressed)
            self.output.append_record(
                "Punkt %s ließ sich nicht löschen: %s" % (pressed, exc), logging.WARNING)
        if ok:
            self._advance_series(row)
        else:
            self.output.append_record("Druck verworfen — Zeile bleibt markiert, "
                                      "POINT noch einmal drücken.", logging.WARNING)
            self._update_series_info()

    def _computed_rows(self):
        computed = self._computed_names()
        return {row for row, step in enumerate(self.current_steps())
                if step["point"] in computed}

    def _advance_series(self, row):
        if self._series_single:
            # "Teachen" eines Punkts: nach dem einen Druck fertig
            name = self._series_single
            self._series_entries = []
            self._stop_series("%s gesetzt" % name, ask=False)
            if name == "PICK" and self.align_after_teach.isChecked():
                self.align_point(name, ask=False)
            self.output.append_record(
                "%s gesetzt. Prüfen: \"Anfahren\" oder die Probefahrt vor der Aufnahme."
                % name)
            return
        nxt = series_module.next_row(row, self.steps.rowCount(), self._computed_rows())
        if nxt < 0:
            self._stop_series("letzte Zeile gesetzt")
            return
        self.steps.setCurrentCell(nxt, 0)
        self._update_series_info()

    def _keys_allowed(self):
        """Tasten nur im sichtbaren Reiter, ohne laufende Aufnahme."""
        return self.isVisible() and not self.process.running and self.steps.isEnabled()

    def _hotkey_touch_up(self):
        """F8: wie POINT am Arm, aber mit der aktuellen Stellung."""
        if not self._keys_allowed() or self._series_busy:
            return
        row = self.steps.currentRow()
        if row < 0:
            return
        self._series_busy = True
        try:
            ok = self.touch_up_row(row, series=self._series_watcher is not None)
        finally:
            self._series_busy = False
        if ok:
            if self._series_watcher is not None:
                self._advance_series(row)
            else:
                nxt = series_module.next_row(row, self.steps.rowCount(),
                                             self._computed_rows())
                if nxt >= 0:
                    self.steps.setCurrentCell(nxt, 0)

    def _step_row(self, delta):
        """Bild↓/Bild↑: Zeile weiter/zurueck -- z. B. den ersten PRE_GRASP
        ueberspringen."""
        if not self._keys_allowed() or self.steps.rowCount() == 0:
            return
        row = max(0, min(self.steps.rowCount() - 1, self.steps.currentRow() + delta))
        self.steps.setCurrentCell(row, 0)
        self._update_series_info()

    # -- Tippbetrieb "Z hoch" (bc/gui/jogging.py) ---------------------------

    def _jog_pressed(self, source="button"):
        if self._jogger is not None:
            return
        self._jog_source = source
        robot = self.session.robot
        if robot is None:
            self.output.append_record("Z hoch: erst mit der Steuerung verbinden.", logging.WARNING)
            return
        if self.process.running:
            self.output.append_record("Z hoch: nicht während einer Aufnahme.", logging.WARNING)
            return
        jogger = jogging_module.Jogger(robot, jogging_module.z_up_velocity(), RealClock())
        try:
            jogger.start()
        except Exception as exc:
            # Keine Meldungsbox: sie erschiene, waehrend die Maus noch
            # gedrueckt ist, und schluckte das Loslassen.
            self.output.append_record("Z hoch verweigert: %s" % exc, logging.WARNING)
            return
        self._jogger = jogger
        self.jog_timer.start()

    def _jog_tick(self):
        jogger = self._jogger
        if jogger is None:
            self.jog_timer.stop()
            return
        # Loslassen ausserhalb des Knopfs oder Fokuswechsel: nicht weiterfahren
        held = (self._jog_key_down if self._jog_source == "key"
                else self.btn_jog_up.isDown())
        if not held:
            self._jog_released()
            return
        if not jogger.tick():
            self._jog_released()

    def _jog_released(self):
        self.jog_timer.stop()
        jogger, self._jogger = self._jogger, None
        if jogger is None:
            return
        jogger.stop()
        if jogger.reason:
            self.output.append_record("Z hoch angehalten: %s" % jogger.reason, logging.WARNING)
        else:
            self.output.append_record(
                "Z hoch: %d Schritte (%.1f s)" % (jogger.ticks, jogger.ticks / config.JOG_TICK_HZ))

    def eventFilter(self, watched, event):
        """F7 halten = "Z hoch". Ein QShortcut kennt kein Loslassen, daher
        ein Filter auf der Anwendung -- eingehaengt nur, solange der Reiter
        sichtbar ist. Die automatische Tastenwiederholung wird verschluckt,
        sonst kaeme bei jedem Wiederholen ein Loslassen. Verliert das
        Fenster den Fokus, haelt der Arm an."""
        kind = event.type()
        if kind in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) \
                and event.key() == Qt.Key.Key_F7:
            if not self.window().isActiveWindow():
                return False
            if event.isAutoRepeat():
                return True
            if kind == QEvent.Type.KeyPress:
                self._jog_key_down = True
                if self._keys_allowed() and self.btn_jog_up.isEnabled():
                    self._jog_pressed(source="key")
            else:
                self._jog_key_down = False
                if self._jog_source == "key":
                    self._jog_released()
            return True
        if kind == QEvent.Type.ApplicationDeactivate:
            self._jog_key_down = False
            if self._jog_source == "key":
                self._jog_released()
        return False

    def showEvent(self, event):
        QApplication.instance().installEventFilter(self)
        super().showEvent(event)

    def hideEvent(self, event):
        # Reiterwechsel/Fenster versteckt: Tippbetrieb und Serie sicher beenden
        QApplication.instance().removeEventFilter(self)
        self._jog_key_down = False
        self._jog_released()
        self._stop_series("Reiter verlassen", ask=False)
        super().hideEvent(event)

    def _reload_point_lists(self):
        """Punktauswahl neu fuellen (ein Touch-up kann einen Punkt anlegen),
        ohne den Bearbeitungsstand der Tabelle zu verlieren."""
        dirty, row = self._steps_dirty, self.steps.currentRow()
        self._fill_steps(self.current_steps())
        self._steps_dirty = dirty
        if row >= 0:
            self.steps.setCurrentCell(row, 0)
        self._refresh_sequence_info()

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
                self.current_steps(), self._table_cfg)
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

    def _speed_box(self, value_range, default, step, decimals, suffix, tip):
        box = block_wheel(QDoubleSpinBox())
        box.setRange(*value_range)
        box.setSingleStep(step)
        box.setDecimals(decimals)
        box.setSuffix(suffix)
        box.setValue(default)
        box.setToolTip(tip + "\n\nStandard: %g%s" % (default, suffix))
        box.valueChanged.connect(self._update_command)
        return box

    def _build_run_box(self):
        box = QGroupBox("3. Aufnahme")
        layout = QVBoxLayout(box)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self.episodes = block_wheel(QSpinBox())
        self.episodes.setRange(1, 500)
        # Standard 3 Fahrten derselben Lage: Mit Zuruecklegen (Standard,
        # bc/put_back.py) bringen 3 den Grossteil der zusaetzlichen
        # Abdeckung. Ohne Zuruecklegen gehoert an der Anlage zu einem Aufruf
        # genau eine Episode (recording.episode_advice, Labortag 2026-10-01).
        self.episodes.setValue(3)
        self.episodes.valueChanged.connect(self._update_command)
        self.episodes.setToolTip("Fahrten derselben Lage, je mit neuem Rauschen")
        form.addRow("Fahrten je Lage", self.episodes)

        # Automatische Lagen (bc/auto_layout.py, Anwender 2026-10-26)
        self.lagen = block_wheel(QSpinBox())
        self.lagen.setRange(1, 200)
        self.lagen.setValue(1)
        self.lagen.setToolTip(
            "Verschiedene Objektlagen in diesem Lauf. 1 = nur die geteachte. Ab 2 legt "
            "der Arm das Objekt nach den Fahrten einer Lage selbst an eine neue, "
            "zufällige Stelle im Bereich der Tischpunkte (außerhalb der Sperrzonen, "
            "senkrecht, Greifhöhe über dem Tisch) und greift dort weiter. Zum Schluss "
            "zurück an den geteachten PICK. Braucht Zurücklegen und die Tischpunkte.")
        self.lagen.valueChanged.connect(self._update_command)
        form.addRow("Lagen", self.lagen)
        self.drehung = block_wheel(QDoubleSpinBox())
        self.drehung.setRange(0.0, 90.0)
        self.drehung.setDecimals(0)
        self.drehung.setSingleStep(5.0)
        self.drehung.setSuffix(" °")
        self.drehung.setValue(config.AUTO_YAW_RANGE_DEG)
        self.drehung.setToolTip("Automatische Lagen: Drehung bis zu ± so viel gegen die "
                                "geteachte Lage")
        self.drehung.valueChanged.connect(self._update_command)
        form.addRow("Drehung ±", self.drehung)
        self.margin = block_wheel(QDoubleSpinBox())
        self.margin.setRange(0.0, 200.0)
        self.margin.setDecimals(0)
        self.margin.setSingleStep(5.0)
        self.margin.setSuffix(" mm")
        self.margin.setValue(1e3 * config.AUTO_EDGE_MARGIN_M)
        self.margin.setToolTip(
            "Automatische Lagen: so viel Abstand hält die Greifposition zum Rand des "
            "Bereichs der Tischpunkte (ID0–ID3) -- Platz für Objekt und Backen.")
        self.margin.valueChanged.connect(self._update_command)
        form.addRow("Abstand zum Rand", self.margin)
        self.regrip = QCheckBox("Nachgreifen beim Lagenwechsel")
        self.regrip.setChecked(True)
        self.regrip.setToolTip(
            "Nach dem Ablegen an einer NEUEN Lage einmal zu und auf: der Greifer schiebt "
            "das Objekt mittig zwischen die Backen. Bei Wiederholungen derselben Lage "
            "nicht nötig. Aus = nie nachgreifen.")
        self.regrip.toggled.connect(self._update_command)
        form.addRow("", self.regrip)
        self.duration_hint = _hint("")
        form.addRow("Dauer", self.duration_hint)

        # Zuruecklegen nach jeder Episode (record.py, bc/put_back.py) --
        # Standard bei der Aufnahme. Im Greifer bleibt das Objekt nur bei
        # der Inferenz.
        self.put_back = QCheckBox("Objekt nach jeder Episode zurücklegen")
        self.put_back.setChecked(True)
        self.put_back.setToolTip(
            "Nach jeder Episode, auch der letzten, legt der Arm das Objekt am "
            "Greifpunkt ab und fährt an den Start — nicht aufgezeichnet, ohne "
            "Rauschen. Mehrere Episoden fahren so dieselbe Lage mit neuem "
            "Rauschen; der Rauschfaktor wird über die Fahrten gestaffelt. "
            "Nur mit Ablaufdatei. Aus = der Arm hält das Objekt am Ende an "
            "der Übergabe.")
        self.put_back.toggled.connect(self._put_back_toggled)
        form.addRow("", self.put_back)

        # Probefahrt vor der ersten Episode (record.py, bc/probe.py) --
        # Standard, Wunsch Anwender 2026-10-23 nach einem Crash mit
        # vertauscht geteachten Punkten.
        self.probe = QCheckBox("Probefahrt vor der ersten Episode")
        self.probe.setChecked(True)
        self.probe.setToolTip(
            "Vor der ersten Episode fährt der Arm die Bahn mit %.0f %% Tempo ab "
            "und wieder zurück — ohne Rauschen, Greifer bleibt offen, nichts "
            "wird aufgezeichnet. Danach fragt die Oberfläche, ob die Aufnahme "
            "starten soll.\n\nDie Plausibilitätsprüfung der Punkte (Endanflug "
            "von oben) läuft immer, auch ohne Haken. Gegen den SimRobot "
            "entfällt die Probefahrt." % (100 * config.PROBE_SPEED_FACTOR))
        self.probe.toggled.connect(self._update_command)
        form.addRow("", self.probe)

        # Betriebsart (Wunsch Anwender 2026-10-26): am Pendant umzuschalten
        # heisst jedes Mal anmelden. Automatik schaltet record.py beim
        # Start selbst (connect(ensure_automatic=True)); zurueck in Teach
        # nur auf Wunsch -- fuer Lichtvarianten derselben Lage bleibt man
        # in Automatik.
        self.teach_after = QCheckBox("Nach dem Lauf in Teach schalten")
        self.teach_after.setChecked(True)
        self.teach_after.setToolTip(
            "Nach dem Lauf (auch nach einem Abbruch) schaltet die Steuerung in den "
            "Teach-Modus — zum Teachen des nächsten PICK ohne Anmeldung am Pendant. "
            "Beim nächsten Start schaltet die Aufnahme selbst zurück auf Automatik "
            "(Zero-G muss dafür aus sein).\n\nAus: z. B. dieselbe Lage mehrmals "
            "mit anderem Licht aufnehmen. Wirkt nur an einer Steuerung, nicht "
            "gegen den SimRobot. Ein Abbrechen über den Knopf beendet den Prozess "
            "sofort — dann wird nicht umgeschaltet.")
        self.teach_after.toggled.connect(self._update_command)
        form.addRow("", self.teach_after)

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

        # servo_j-Senderate: in der VM muss sie herunter, an der Anlage
        # vermutlich nicht (Dokumentation/Taktzeit-und-RPC-Latenz.md, 8).
        # Deshalb je Lauf einstellbar statt als globale Konstante -- so
        # steht der Wert im Protokoll und in den Metadaten der Episode.
        self.servo_rate = block_wheel(QComboBox())
        for k in range(1, 9):
            hz = k * config.CONTROL_RATE_HZ
            self.servo_rate.addItem(
                "%g Hz  (%d Teilschritt%s)" % (hz, k, "" if k == 1 else "e"), hz)
        self.servo_rate.setCurrentIndex(
            int(round(config.SERVO_RATE_HZ / config.CONTROL_RATE_HZ)) - 1)
        self.servo_rate.setToolTip(
            "Wie fein die Sollwerte zwischen zwei Regeltakten gesendet werden. "
            "Höher ist ruhiger, kostet aber Zeit im Takt: jeder Teilschritt ist "
            "ein eigener Aufruf an die Steuerung. Hält der Takt nicht, wird die "
            "Bahn langsamer abgefahren — und diese Geschwindigkeit wird "
            "mitgelernt (AP 2.6).\n\n"
            "MUSS bei Aufzeichnung und Inferenz gleich sein (AP 1.5.1): eine "
            "Policy aus 30-Hz-Daten lässt sich nur mit 30 Hz fahren. "
            "apps/infer.py bricht bei Abweichung ab.\n\n"
            "Messen mit tools/check_servo_timing.py, rechnen mit "
            "tools/check_rate_budget.py.")
        self.servo_rate.currentIndexChanged.connect(self._update_command)
        form.addRow("servo_j-Rate", self.servo_rate)
        self.servo_rate_hint = QLabel("")
        self.servo_rate_hint.setWordWrap(True)
        form.addRow("", self.servo_rate_hint)

        # Planer-Tempo (AP 2.6). Langsamer heisst auch ruhiger, wenn der
        # Rechner den Takt nicht haelt: ein verspaeteter Sollwert erzeugt
        # einen Sprung proportional zur Geschwindigkeit (Auswertung
        # 2026-10-03). Anders als der Override faehrt der Arm die langsamere
        # Bahn genau ab -- deshalb wird das Tempo HIER gesenkt, nicht dort.
        tempo_tip = (
            "Wird mitgelernt (AP 2.6) und muss für alle Episoden eines "
            "Datensatzes gleich sein — apps/export.py bricht bei gemischten "
            "Werten ab. Langsamer = ruhiger, auch wenn der Rechner den Takt "
            "nicht ganz hält; der Arm fährt die Bahn dann trotzdem genau ab. "
            "Den Override dafür NICHT senken: dann läuft der Arm hinterher "
            "und die Daten zeigen eine Bahn, die nicht gefahren wurde.")
        self.transit_speed = self._speed_box(
            config.PLANNER_TRANSIT_SPEED_RANGE_MS, config.TRANSIT_SPEED_MS,
            0.01, 3, " m/s", tempo_tip)
        form.addRow("Tempo Transit", self.transit_speed)
        self.approach_speed = self._speed_box(
            config.PLANNER_APPROACH_SPEED_RANGE_MS, config.APPROACH_SPEED_MS,
            0.005, 3, " m/s", tempo_tip)
        form.addRow("Tempo Endanflug", self.approach_speed)
        self.ptp_speed = self._speed_box(
            config.PLANNER_PTP_SPEED_RANGE_RADS, config.PTP_JOINT_SPEED_RADS,
            0.05, 2, " rad/s", tempo_tip)
        form.addRow("Tempo PTP (Gelenke)", self.ptp_speed)
        self.tempo_hint = QLabel("")
        self.tempo_hint.setWordWrap(True)
        form.addRow("", self.tempo_hint)

        # Ausfuehrungsart (bc/execution.py, Dokumentation/Zittern-an-der-
        # Anlage.md): wie die Sollwerte an die Steuerung gehen. "auto" ist
        # an der Anlage online, in VM und SimRobot servo_j.
        self.execution = block_wheel(QComboBox())
        for key, text in (("auto", "auto (Anlage: online, VM: servo_j)"),
                          ("servo_j", "servo_j 60 Hz (bisher)"),
                          ("servo_j_pos", "servo_j 60 Hz ohne Geschwindigkeit"),
                          ("online", "online (movelinear_online)")):
            self.execution.addItem(text, key)
        self.execution.setCurrentIndex(max(0, self.execution.findData(config.EXECUTION_MODE)))
        self.execution.setToolTip(
            "servo_j: 60 Sollwerte je Sekunde, linear interpoliert -- an der "
            "Anlage zittert der Arm (Treppe, Eigenschwingung 7-8 Hz).\n"
            "servo_j ohne Geschwindigkeit: dieselben Sollwerte, v = a = 0 -- "
            "die Steuerung glaettet selbst (VM: glatt, ~70 ms Verzug). An der "
            "Anlage noch ungeprueft.\n"
            "online: ein Ziel je Takt per movelinear_online, die Steuerung "
            "interpoliert selbst -- gemessen 76 %% weniger Zittern, dafuer "
            "etwas weniger bahntreu (Vorlauf %d, Tempo x%.1f, Beschl. x%.1f).\n\n"
            "Steht in den Metadaten, muss in einem Datensatz einheitlich sein "
            "und wird von der Inferenz uebernommen. In der VM faehrt online "
            "nicht (alpha.102)." % (config.ONLINE_LEAD_STEPS, config.ONLINE_SPEED_FACTOR,
                                     config.ONLINE_ACCEL_FACTOR))
        self.execution.currentIndexChanged.connect(self._update_command)
        form.addRow("Ausführung", self.execution)

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

        # Trockenlauf: nur fuer Bahntests, solange am Greifer noch die
        # Pinbelegung fehlt. Die Episode wird dabei zwingend verworfen --
        # der Greifer schaltet nicht, steht im Action-Vektor aber drin.
        self.no_gripper = QCheckBox("Trockenlauf ohne Greifer (Episoden werden verworfen)")
        self.no_gripper.setToolTip(
            "Für Bahntests, wenn in der Steuerung noch kein Greifer "
            "konfiguriert ist. Der Arm fährt die VOLLSTÄNDIGE Bahn, der "
            "Greifer schaltet nicht — kein Werkstück einlegen.\n\n"
            "Jede so aufgezeichnete Episode wird verworfen und vom Export "
            "abgewiesen: die Greiferbefehle stehen im Action-Vektor, haben "
            "aber nie stattgefunden.")
        self.no_gripper.toggled.connect(self._update_command)
        layout.addWidget(self.no_gripper)

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
            "Auswahl oder eigener Text (wird gemerkt, einstellungen/metadaten_auswahl.json "
            "— einheitliche Begriffe für die Auswertung). Gilt für alle Episoden dieses "
            "Aufrufs und landet in "
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
            "Eine Objektlage = ein Block. Zählt von selbst hoch, sobald nach einer "
            "Aufnahme PICK neu gesetzt wird (Teachen, Touch-up, Ausrichten). "
            "Position und Drehung des Objekts schreibt die Aufnahme selbst in die "
            "Metadaten (object_xy_m, object_yaw_deg).")
        vocabulary = vocabulary_module.load(self.workdir)
        self.object_note = self._vocabulary_box(vocabulary["object_note"])
        self.object_note.lineEdit().setPlaceholderText("z. B. Teil A, 30° gedreht, links")
        self.object_note.setToolTip(
            "Objekt und Lage in Worten. Die geteachten Greifpunkte werden "
            "ohnehin als object_pose abgelegt — das hier ist die Lesehilfe "
            "dazu.")
        self.light = self._vocabulary_box(vocabulary["light"])
        self.light.lineEdit().setPlaceholderText("z. B. Decke an, Rollo zu")
        self.light.setToolTip(
            "Beleuchtungssituation. Ändert die Bildstatistik, auf die "
            "trainiert wird.")
        self.camera_pose = self._vocabulary_box(vocabulary["camera_pose"])
        self.camera_pose.lineEdit().setPlaceholderText("z. B. nominal oder +2cm x")
        self.camera_pose.setToolTip(
            "Variante der Szenenkamera-Pose. Nur ändern, wenn die Kamera "
            "tatsächlich versetzt wurde (AP 1.4).")
        self.block.textChanged.connect(self._update_command)
        form.addRow("Block", self.block)
        for widget, label in ((self.object_note, "Objekt"),
                              (self.light, "Beleuchtung"),
                              (self.camera_pose, "Kamerapose")):
            widget.editTextChanged.connect(self._update_command)
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
        self.btn_stop_after = QPushButton("Nach dieser Lage beenden")
        self.btn_stop_after.setEnabled(False)
        self.btn_stop_after.setToolTip(
            "Die laufende Lage wird zu Ende gefahren, dann legt der Arm das Objekt "
            "zurück an den geteachten PICK und der Lauf endet.")
        self.btn_stop_after.clicked.connect(self._request_stop_after)
        layout.addWidget(self.btn_stop_after)
        layout.addStretch(1)
        outer.addWidget(row)
        return bar

    def _request_stop_after(self):
        try:
            self._stop_file.write_text("nach dieser Lage beenden", encoding="utf-8")
        except OSError as exc:
            QMessageBox.warning(self, "Beenden", "Nicht möglich: %s" % exc)
            return
        self.btn_stop_after.setEnabled(False)
        self.output.append_record("Beenden nach dieser Lage angefordert.")

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
        self.image.setToolTip("Doppelklick auf ein Kamerabild: groß anzeigen")
        self.image.double_clicked.connect(self._zoom_at)
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
        self.estimate_line = _hint("")
        progress_layout.addWidget(self.estimate_line)
        progress_layout.addWidget(self.headline)
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.detail_line)

        self.episode_list = QTreeWidget()
        self.episode_list.setHeaderLabels(["Episode", "Ordner", "Schritte", "Ergebnis"])
        self.episode_list.setRootIsDecorated(False)
        self.episode_list.setMaximumHeight(110)
        for column, width in enumerate((70, 130, 80, 240)):
            self.episode_list.setColumnWidth(column, width)
        self.episode_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.episode_list.customContextMenuRequested.connect(self._episode_menu)
        self.episode_list.setToolTip(
            "Episoden dieses Laufs. Rechtsklick: nachträglich als fehlgeschlagen "
            "markieren, verwerfen oder wieder als erfolgreich werten — auch "
            "während der Lauf weiterfährt.")
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
        try:
            self._table_cfg = recording_module.read_table(self.workdir / working)
        except Exception:
            self._table_cfg = None
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
            camera_specs=cameras_module.to_specs(self.cameras.assignment(),
                                                 self.cameras.fps()),
            override=(self.session.override if self._robot_kind() == "neura"
                      else None),
            noise_scale=self.noise_scale.value(),
            noise_fixed=self.noise_fixed.isChecked(),
            servo_rate=self.servo_rate.currentData(),
            transit_speed=self.transit_speed.value(),
            approach_speed=self.approach_speed.value(),
            ptp_speed=self.ptp_speed.value(),
            execution=self.execution.currentData(),
            block=self.block.text().strip() or None,
            light=vocabulary_module.normalize(self.light.currentText()),
            camera_pose=vocabulary_module.normalize(self.camera_pose.currentText()),
            object_note=vocabulary_module.normalize(self.object_note.currentText()),
            ask_label=self.ask_label.isChecked(),
            preview=(str(self._live_file())
                     if self.live_during_recording.isChecked() else None),
            real_robot=status.is_real_plant,
            no_gripper=self.no_gripper.isChecked(),
            put_back=self.put_back.isChecked(),
            probe=self.probe.isChecked(),
            teach_after=self.teach_after.isChecked(),
            lagen=self.lagen.value(),
            drehung=self.drehung.value(),
            stop_file=self._stop_file,
            regrip="lagenwechsel" if self.regrip.isChecked() else "nie",
            margin=self.margin.value() / 1e3)

    def _put_back_toggled(self, checked):
        # Ohne Zuruecklegen ist an der Anlage nur EINE Episode je Aufruf
        # moeglich -- gleich darauf stellen; wieder an -> die Empfehlung 3.
        if self.session.status().is_real_plant:
            self.episodes.setValue(3 if checked else 1)
        self._update_command()

    def _update_command(self):
        try:
            self.command.setText(training_module.command_line(self._argv()))
        except ValueError as exc:
            self.command.setText("Aufnahme: %s" % exc)
        self._update_duration()

    def _update_duration(self):
        """Grobe Dauer fuer die aktuellen Einstellungen (Wunsch Anwender 2026-10-27)."""
        if not all(hasattr(self, n) for n in ("duration_hint", "probe", "ask_label", "lagen")):
            return
        probe = self.probe.isChecked() and self._robot_kind() != "sim"
        seconds = recording_module.estimate_duration_s(
            self.episodes.value(), self.lagen.value(), probe)
        self.duration_hint.setText(
            "ca. %s%s" % (recording_module.format_duration(seconds),
                          " + Bewertungen" if self.ask_label.isChecked() else ""))
        self.lagen.setToolTip(
            self.lagen.toolTip().split("\n\nGrob")[0]
            + "\n\nGrob bei %d Fahrt(en) je Lage:\n" % self.episodes.value()
            + "\n".join(recording_module.duration_table(self.episodes.value(), probe)))

        status = self.session.status()
        assignment = self.cameras.assignment()
        hz = self.servo_rate.currentData()
        if hz is not None and float(hz) != config.SERVO_RATE_HZ:
            self.servo_rate_hint.setText(
                "Abweichend vom Standard (%g Hz). Die Rate landet in den "
                "Metadaten und muss bei der Inferenz dieselbe sein — eine "
                "Policy aus diesen Daten lässt sich nur mit %g Hz fahren."
                % (config.SERVO_RATE_HZ, float(hz)))
            self.servo_rate_hint.setStyleSheet("color:%s" % WARN)
        else:
            self.servo_rate_hint.setText("")
            self.servo_rate_hint.setStyleSheet("color:%s" % NEUTRAL)

        speeds = (self.transit_speed.value(), self.approach_speed.value(),
                  self.ptp_speed.value())
        problems = trajectory_module.planner_speed_problems(*speeds)
        if problems:
            self.tempo_hint.setText("✗ " + "; ".join(problems))
            self.tempo_hint.setStyleSheet("color:%s" % BAD)
        elif any(abs(v - d) > 1e-9 for v, d in zip(speeds, (
                config.TRANSIT_SPEED_MS, config.APPROACH_SPEED_MS,
                config.PTP_JOINT_SPEED_RADS))):
            self.tempo_hint.setText(
                "Abweichend vom Standard. Das Tempo landet in den Metadaten; "
                "alle Episoden eines Datensatzes brauchen dasselbe.")
            self.tempo_hint.setStyleSheet("color:%s" % WARN)
        else:
            self.tempo_hint.setText("")
            self.tempo_hint.setStyleSheet("color:%s" % NEUTRAL)

        advice = recording_module.episode_advice(
            status.is_real_plant, self.episodes.value(),
            put_back=self.put_back.isChecked())
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
        if self._preview_cancel is not None:
            self.cancel_preview_open()
        elif self.preview_timer.isActive() or self._captures:
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
        import threading

        cancel = threading.Event()
        self._preview_cancel = cancel
        self.btn_preview.setText("Abbrechen")
        self.sources_line.setStyleSheet("color:%s" % NEUTRAL)
        self.sources_line.setText(
            "Wartet auf einen vorherigen Kamerazugriff, der noch nicht "
            "zurückgekehrt ist …" if self.runner.busy("camera")
            else "Kameras werden geöffnet …")
        configs = cameras_module.to_configs(assignment, self.cameras.fps())
        self._preview_configs = {c.name: c for c in configs}
        self.runner.submit(lambda: _open_captures(configs, cancel),
                           on_done=lambda caps: self._preview_started(caps, cancel),
                           on_error=lambda err: self._preview_failed(err, cancel),
                           pool="camera")

    def cancel_preview_open(self):
        """Oeffnen abbrechen -- sofort, ohne auf den Treiber zu warten.

        Kehrt das Oeffnen spaeter doch noch zurueck, werden die Kameras in
        _preview_started gleich wieder geschlossen: sie sollen nicht
        unbemerkt belegt bleiben, wenn danach eine Aufnahme sie braucht.
        """
        if self._preview_cancel is None:
            return
        self._preview_cancel.set()
        self._preview_cancel = None
        self.btn_preview.setEnabled(True)
        self.btn_preview.setText("Vorschau starten")
        self.sources_line.setText("Öffnen abgebrochen.")
        self.sources_line.setStyleSheet("color:%s" % NEUTRAL)
        log.info("Vorschau: Oeffnen abgebrochen")

    def _preview_started(self, captures, cancel):
        if cancel is not self._preview_cancel:
            # Abgebrochen, waehrend der Treiber noch oeffnete: Geraete
            # sofort wieder freigeben.
            for capture in captures:
                try:
                    capture.stop()
                except Exception:
                    log.exception("Kamera schliessen gescheitert")
            return
        self._preview_cancel = None
        self._captures = captures
        self._rates = RateMeter()
        self._camera_settings = {}
        for capture in captures:
            try:
                self._camera_settings[capture.name] = capture.camera.settings()
            except Exception:
                log.exception("Kameraeinstellungen nicht lesbar")
            if self._camera_settings.get(capture.name):
                log.info("Kamera %s", describe_settings(
                    capture.name, self._camera_settings[capture.name]))
        self.btn_preview.setEnabled(True)
        self.btn_preview.setText("Vorschau beenden")
        self.preview_timer.start()
        log.info("Vorschau offen: %s",
                 ", ".join(c.name for c in captures))

    def _preview_failed(self, error, cancel):
        if cancel is not self._preview_cancel:
            return  # Abbruch oder veraltetes Oeffnen
        self._preview_cancel = None
        self._captures = None
        self.btn_preview.setEnabled(True)
        self.btn_preview.setText("Vorschau starten")
        self.image.set_image(None)
        self.sources_line.setText("Vorschau gescheitert: %s" % error)
        self.sources_line.setStyleSheet("color:%s" % BAD)
        log.error("Vorschau gescheitert: %s", error)

    def stop_preview(self):
        if self._preview_cancel is not None:
            self.cancel_preview_open()
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
        self._feed_zoom(frames=frames)
        # Schafft jede Kamera ihre Rate? (Anlage 2026-10-26: am Hub 23 statt
        # 61 fps -- und 20 % der Frames ueber Budget, ohne dass es auffiel)
        warnings = []
        for capture in self._captures:
            fps = self._rates.last(capture.name)
            cfg = getattr(self, "_preview_configs", {}).get(capture.name)
            if fps is None or cfg is None or cfg.backend == "sim":
                continue
            settings = self._camera_settings.get(capture.name) or {}
            warnings += [line.replace("[WARNUNG] ", "") for line in camera_rate_findings(
                capture.name, fps, fps_label(cfg), settings.get("rate_possible_fps"))]
        if len(timestamps) > 1:
            skew_ms = 1e3 * (max(timestamps) - min(timestamps))
            budget = 1e3 * config.SYNC_MAX_SKEW_S
            bits.append("Versatz %.0f ms %s Budget %.0f ms"
                        % (skew_ms, "über" if skew_ms > budget else "unter", budget))
            self.sources_line.setStyleSheet(
                "color:%s" % (WARN if skew_ms > budget else NEUTRAL))
        if warnings:
            bits.append("⚠ " + " ⚠ ".join(warnings))
            self.sources_line.setStyleSheet("color:%s" % WARN)
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
            self._feed_zoom(board=image)

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
        problems += trajectory_module.planner_speed_problems(
            self.transit_speed.value(), self.approach_speed.value(),
            self.ptp_speed.value())
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
        if self.runner.busy("camera"):
            # Ein abgebrochenes Oeffnen haengt noch im Treiber und haelt
            # womoeglich ein Geraet -- record.py wuerde daran scheitern,
            # mit einer Meldung, die nicht auf die Ursache zeigt.
            QMessageBox.information(
                self, "Kamera noch belegt",
                "Ein Kamerazugriff der Oberfläche (Vorschau oder Gerätesuche) "
                "ist noch nicht zurückgekehrt. Die Aufnahme würde dieselbe "
                "Kamera öffnen wollen.\n\nIn ein paar Sekunden erneut starten.")
            return

        self._remember_metadata()
        self._block_used = True
        self._labels_off = False
        self.estimate_line.setText("")
        self._live_names = [s for s in cameras_module.SLOTS if s in assignment]
        self._progress = recording_module.RecordProgress(
            total_episodes=self.episodes.value() * max(1, self.lagen.value()))
        try:
            self._stop_file.unlink()   # alte Anforderung weg
        except OSError:
            pass
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

        argv = self._argv()
        # Fuer das nachtraegliche Bewerten (_episode_menu): wohin dieser Lauf
        # schreibt. Relativ zum Arbeitsordner, wie record.py es aufloest.
        self._run_out = self.workdir / argv[argv.index("--out") + 1]
        try:
            # stdin bleibt offen, solange nach jeder Episode oder nach der
            # Probefahrt gefragt wird -- sonst bekommt record.py EOF, laesst
            # alles unbewertet bzw. bricht nach der Probefahrt ab.
            self.process.start(
                argv,
                stdin_text="ANLAGE\n" if status.is_real_plant else None,
                keep_stdin=self.ask_label.isChecked() or self.probe.isChecked())
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
        lines.append("Ausführung: %s" % self.execution.currentText())
        lines.append("Tempo: Transit %.3f m/s · Endanflug %.3f m/s · PTP %.2f rad/s"
                     % (self.transit_speed.value(), self.approach_speed.value(),
                        self.ptp_speed.value()))
        if status.state == session_module.OFFLINE:
            lines += ["", "Es ist nichts verbunden — apps/record.py verbindet "
                          "selbst. Welche Steuerung antwortet, steht erst dann fest."]
        if status.is_real_plant:
            lines += ["", "ACHTUNG: Das ist die REALE ANLAGE, nicht die Simulation."]
        if self._robot_kind() != "sim":
            lines += ["", "Steht die Steuerung im Teach-Modus, wird sie auf Automatik "
                          "geschaltet%s." % (" — nach dem Lauf zurück in Teach"
                                             if self.teach_after.isChecked() else "")]
        if self.probe.isChecked() and self._robot_kind() != "sim":
            lines += ["", "Zuerst Probefahrt mit %.0f %% Tempo, Greifer offen — "
                          "danach wird nach der Freigabe gefragt."
                      % (100 * config.PROBE_SPEED_FACTOR)]
        if self.no_gripper.isChecked():
            lines += ["", "TROCKENLAUF: Der Greifer schaltet NICHT, die Bahn wird "
                          "trotzdem vollständig gefahren. Kein Werkstück einlegen.",
                      "Jede Episode wird verworfen und ist nicht exportierbar."]
        lines += ["", "Arbeitsraum frei? Not-Aus in Reichweite?"]
        return self.confirm("Roboter bewegt sich", "\n".join(lines))

    def stop(self):
        if self._moving:
            # Software-Stopp der Fahrt "Anfahren" -- kein Not-Aus (AP 4.2)
            robot = self.session.robot
            if robot is not None:
                robot.emergency_stop()
            self.output.append_record("Anfahren gestoppt.", logging.WARNING)
            return
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

        estimate = recording_module.duration_estimate(text)
        if estimate:
            self.estimate_line.setText("Geschätzte Dauer: " + estimate)

        free = recording_module.next_free_block(text)
        if free:
            # Automatische Lagen haben Bloecke verbraucht -- die naechste
            # neue Lage bekommt den ersten freien
            if self.lagen.value() > 1:
                self._next_free_block = free

        if recording_module.probe_question(text):
            # Probefahrt fertig, record.py wartet auf die Freigabe.
            self._ask_probe()
            return

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

    def _ask_probe(self):
        """Freigabe nach der Probefahrt (bc/probe.py) an record.py schicken.

        Standardknopf ist "Abbrechen": Weiter geht es nur auf ausdrueckliche
        Freigabe -- danach faehrt der Arm mit Tempo und Greifer.
        """
        box = QMessageBox(self)
        box.setWindowTitle("Probefahrt beendet")
        box.setIcon(QMessageBox.Icon.Question)
        box.setText("Die Probefahrt ist durch, der Arm steht wieder am Start.")
        box.setInformativeText(
            "Ist der Arm ohne Kollision gefahren, und sitzt der Greifer am "
            "Objekt richtig?\n\n„Aufnahme starten\" fährt die Episoden mit "
            "vollem Tempo, Rauschen und Greifer.")
        go = box.addButton("Aufnahme starten", QMessageBox.ButtonRole.AcceptRole)
        cancel = box.addButton("Abbrechen", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(cancel)
        box.exec()
        answer = "weiter" if box.clickedButton() is go else "abbrechen"
        if not self.process.send(recording_module.PROBE_ANSWERS[answer]):
            log.warning("Freigabe konnte nicht gesendet werden -- laeuft der Prozess noch?")

    def _ask_label(self, episode, steps):
        """Erfolgs-Label abfragen und an den Unterprozess schicken (AP 5.2).

        Der Lauf steht so lange. Das ist gewollt: An der Anlage wird
        zwischen zwei Episoden ohnehin zurueckgesetzt, und bewertet wird,
        solange man es noch gesehen hat. Wird das Fenster geschlossen oder
        der Lauf abgebrochen, bekommt record.py EOF und laesst die Episode
        unbewertet -- nichts wird stillschweigend weggeworfen.
        """
        if self._labels_off:
            # Leere Antwort = erfolgreich (record.ask_label), ohne zu warten.
            # Fehlschlaege setzt man dann in der Episodenliste (Rechtsklick).
            self.process.send("")
            return
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
        silence = QCheckBox("nicht erneut fragen — restliche Episoden dieses Laufs "
                            "als erfolgreich werten")
        silence.setToolTip(
            "Der Lauf hält dann nicht mehr an. Einen Fehlschlag markierst du "
            "danach in der Episodenliste unter dem Fortschritt (Rechtsklick).")
        box.setCheckBox(silence)
        box.exec()
        clicked = box.clickedButton()
        answer = {good: "erfolg", bad: "fehler", drop: "verwerfen"}.get(clicked, "fehler")
        if silence.isChecked():
            self._labels_off = True
            self.output.append_record("Keine Rückfrage mehr -- restliche Episoden dieses "
                                      "Laufs gelten als erfolgreich. Fehlschläge per "
                                      "Rechtsklick in der Episodenliste markieren.")
        if not self.process.send(recording_module.LABEL_ANSWERS[answer]):
            log.warning("Bewertung konnte nicht gesendet werden -- laeuft der Prozess noch?")

    def _episode_menu(self, position):
        """Rechtsklick in der Episodenliste: Bewertung nachtraeglich aendern.

        Gegenstueck zu "Standard erfolgreich": Der Lauf muss fuer einen
        Fehlschlag nicht anhalten, man markiert ihn hier, sobald man ihn
        gesehen hat. Geaendert wird die meta.json (dataset.relabel).
        """
        item = self.episode_list.itemAt(position)
        if item is None or self._run_out is None:
            return
        from .. import dataset as dataset_module

        menu = QMenu(self)
        actions = {
            menu.addAction("Als fehlgeschlagen markieren"): "fehler",
            menu.addAction("Verwerfen (nicht in den Datensatz)"): "verwerfen",
            menu.addAction("Als erfolgreich werten"): "erfolg",
        }
        chosen = menu.exec(self.episode_list.viewport().mapToGlobal(position))
        label = actions.get(chosen)
        if label is None:
            return
        folder = item.text(1)
        try:
            dataset_module.relabel(self._run_out, folder, label,
                                   reason="vom Bedienenden nachträglich verworfen")
        except Exception as exc:
            QMessageBox.warning(self, "Bewertung nicht geändert",
                                "%s/%s: %s" % (self._run_out, folder, exc))
            return
        text = {"erfolg": "erfolgreich (nachträglich)",
                "fehler": "FEHLGESCHLAGEN (nachträglich)",
                "verwerfen": "VERWORFEN (nachträglich)"}[label]
        item.setText(3, text)
        for column in range(4):
            item.setForeground(column, _brush(GOOD if label == "erfolg" else WARN))
        self.output.append_record("%s -> %s" % (folder, text))

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
        if running:
            self._jog_released()
            self._stop_series("Aufnahme gestartet", ask=False)
        for widget in (self.btn_record, self.btn_teach, self.btn_preview,
                       self.steps, self.btn_jog_up, self.btn_series,
                       self.cameras, self.sequence, self.episodes, self.put_back,
                       self.probe, self.teach_after, self.point_table,
                       self.lagen, self.drehung, self.btn_add_keepout, self.regrip,
                       self.margin,
                       self.btn_teach_pick, self.btn_detail,
                       self.transit_speed, self.approach_speed, self.ptp_speed,
                       self.execution):
            widget.setEnabled(not running)
        self.btn_undo_touchup.setEnabled(not running and self._last_touchup is not None)
        self.btn_stop.setEnabled(running)
        self.btn_stop_after.setEnabled(running and self.lagen.value() > 1)

    # -- Lebenszyklus -------------------------------------------------------

    def busy_reason(self):
        if self.process.running:
            return "Aufnahme bzw. Teachen läuft"
        if self._moving:
            return "„Anfahren“ läuft"
        if self._captures or self._preview_cancel is not None:
            return "Kameravorschau ist offen"
        if self._series_watcher is not None:
            return "Teach-Serie läuft"
        if self._jogger is not None:
            return "„Z hoch“ ist aktiv"
        return None

    def shutdown(self):
        self._jog_released()
        self._stop_series("Oberfläche beendet", ask=False)
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
        self._last = {}

    def last(self, name):
        """Zuletzt berechnete Rate dieser Quelle, oder None."""
        return self._last.get(name)

    def update(self, name, frame):
        """Liefert die Rate in fps, oder None, solange zu wenig vorliegt."""
        rate = self._update(name, frame)
        if rate is not None:
            self._last[name] = rate
        return rate

    def _update(self, name, frame):
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


def _open_captures(configs, cancel=None):
    """Kameras oeffnen und starten -- laeuft im Arbeitsthread.

    Geht ueber ``bc.capture`` und ``bc.adapters`` wie der Recorder, damit
    die Vorschau dieselbe Kette prueft, die spaeter aufzeichnet.

    ``cancel`` (threading.Event) wird zwischen den Kameras und waehrend
    des Wartens auf das erste Bild geprueft. Das Oeffnen selbst ist ein
    Treiberaufruf und laesst sich nicht unterbrechen.
    """
    from .. import capture as capture_module
    from ..adapters import Cancelled, open_camera
    from ..ports import CameraError

    def pruefen():
        if cancel is not None and cancel.is_set():
            raise Cancelled("abgebrochen")

    started = []
    try:
        for cfg in configs:
            pruefen()
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
                # In kurzen Stuecken warten, damit ein Abbruch durchkommt --
                # wait_for_frame(10 s) am Stueck hielte ihn bis zu 10 s auf.
                frist = time.monotonic() + 10.0
                while True:
                    pruefen()
                    try:
                        cap.wait_for_frame(timeout=0.25)
                        break
                    except CameraError:
                        if cap.error is not None:
                            raise  # echter Kamerafehler, nicht nur "noch nichts"
                        if time.monotonic() > frist:
                            raise CameraError(
                                "Kamera '%s' lieferte innerhalb 10 s keinen "
                                "Frame" % cap.name)
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

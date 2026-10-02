"""Wiederverwendbare Qt-Bausteine der Oberflaeche.

Bewusst eigene Bausteine statt der Widgets aus der GUI der anderen Gruppe:
Die eigenstaendige Nutzung ist der Fall, der am Labortag gebraucht wird, und
die darf nicht an fremdem Code haengen, der sich jederzeit aendern kann
(Festlegung Anwender, 2026-09-29). Zuschnitt und Benennung sind trotzdem
absichtlich dieselben, damit die Einbindung als Reiter nicht auffaellt.
"""

import logging

from PySide6.QtCore import QEvent, QObject, Qt, Signal, Slot
from PySide6.QtGui import QFont, QImage, QPixmap
from PySide6.QtWidgets import (QComboBox, QFormLayout, QFrame, QGroupBox,
                               QHBoxLayout, QLabel, QPlainTextEdit, QPushButton,
                               QSizePolicy, QVBoxLayout)

from . import cameras as cameras_module
from . import requirements as requirements_module

log = logging.getLogger(__name__)

GOOD = "#2e7d32"
BAD = "#c62828"
WARN = "#e65100"
NEUTRAL = "#616161"


class BackendBanner(QFrame):
    """Dauerhafte Anzeige, WAS wirklich dranhaengt.

    Der Text kommt aus session.Session.status(), also aus der Antwort des
    Controllers auf ``is_robot_in_simulation()`` -- nicht aus der Auswahl im
    Bedienfeld. VM und reale Control-Box hoeren auf dieselbe Adresse; eine
    Anzeige, die der Auswahl folgt, wuerde genau dann luegen, wenn es darauf
    ankommt.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.Shape.StyledPanel)
        self.headline = QLabel("nicht verbunden")
        font = QFont()
        font.setPointSize(13)
        font.setBold(True)
        self.headline.setFont(font)
        self.detail = QLabel("")
        self.detail.setWordWrap(True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(2)
        layout.addWidget(self.headline)
        layout.addWidget(self.detail)
        self.show_status(None)

    @Slot(object)
    def show_status(self, status):
        if status is None:
            self.headline.setText("nicht verbunden")
            self.detail.setText("Backend im Bedienfeld wählen und verbinden.")
            colour = NEUTRAL
        else:
            prefix = "ACHTUNG  " if status.is_real_plant else ""
            self.headline.setText(prefix + status.text)
            bits = [status.detail] if status.detail else []
            if status.tool_name:
                bits.append("Werkzeug: %s" % status.tool_name)
            if status.gripper_mode:
                bits.append("Greifer: %s" % status.gripper_mode)
            bits.append("Bewegung " + ("freigegeben" if status.motion_allowed else "gesperrt"))
            self.detail.setText("  ·  ".join(bits))
            colour = status.colour
        self.setStyleSheet(
            "QFrame{background:%s;border-radius:6px}"
            "QLabel{color:white;background:transparent}" % colour)


class RequirementView(QGroupBox):
    """Feste Zeile je Modus: was da ist, was fehlt, und was das bedeutet.

    Nichts wird ausgeblendet -- fehlende Voraussetzungen werden benannt,
    die Schaltflaechen bleiben (Festlegung Anwender, 2026-09-29).
    """

    def __init__(self, mode, parent=None):
        super().__init__("Voraussetzungen", parent)
        self.mode = mode
        self.report = None
        self._rows = QHBoxLayout()
        self._rows.setSpacing(14)
        outer = QVBoxLayout(self)
        outer.addLayout(self._rows)
        self.refresh()

    def refresh(self):
        """Prueft neu und baut die Zeile auf."""
        while self._rows.count():
            item = self._rows.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
        self.report = requirements_module.report_for(self.mode)
        for result in self.report:
            self._rows.addWidget(self._chip(result))
        self._rows.addStretch(1)
        return self.report

    def _chip(self, result):
        if result.ok:
            mark, colour = "✓", GOOD
        elif result.severity == requirements_module.REQUIRED:
            mark, colour = "✗", BAD
        else:
            mark, colour = "!", WARN
        label = QLabel("%s %s" % (mark, result.label))
        label.setToolTip(result.detail or result.requirement.missing_hint)
        label.setStyleSheet("color:%s; font-weight:bold" % colour)
        return label

    def has(self, key):
        """Ist die Voraussetzung mit diesem Schluessel erfuellt?"""
        for result in self.report or []:
            if result.key == key:
                return result.ok
        return False

    def missing_among(self, keys):
        """Ergebnisse der genannten Schluessel, die fehlen."""
        keys = set(keys)
        return [r for r in (self.report or []) if r.key in keys and not r.ok]


class ImageView(QLabel):
    """Zeigt ein BGR-Bild, auf die Widgetgroesse skaliert, Seitenverhaeltnis erhalten."""

    def __init__(self, placeholder="kein Bild", parent=None):
        super().__init__(placeholder, parent)
        self._pixmap = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(240, 180)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setStyleSheet("background:#202020; color:#aaa; border:1px solid #444;")

    def set_image(self, bgr):
        if bgr is None:
            self._pixmap = None
            self.clear()
            return
        import cv2
        import numpy as np

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if bgr.ndim == 3 else \
            cv2.cvtColor(bgr, cv2.COLOR_GRAY2RGB)
        rgb = np.ascontiguousarray(rgb)
        height, width = rgb.shape[:2]
        image = QImage(rgb.data, width, height, 3 * width, QImage.Format.Format_RGB888)
        self._pixmap = QPixmap.fromImage(image.copy())
        self._rescale()

    def _rescale(self):
        if self._pixmap is not None:
            self.setPixmap(self._pixmap.scaled(
                self.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):                    # noqa: N802 (Qt-API)
        super().resizeEvent(event)
        self._rescale()


class LogView(QPlainTextEdit):
    """Laufende Ausgabe -- Logsaetze der GUI und Zeilen der Unterprozesse."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(5000)
        self.setFont(QFont("Consolas", 9))
        self.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)

    @Slot(str, int)
    def append_record(self, text, level=logging.INFO):
        colour = BAD if level >= logging.ERROR else WARN if level >= logging.WARNING else "#333"
        escaped = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.appendHtml('<span style="color:%s">%s</span>' % (colour, escaped or "&nbsp;"))

    @Slot(str)
    def append_line(self, text):
        """Zeile eines Unterprozesses -- Schweregrad aus dem Text geraten."""
        upper = text.upper()
        if "FAIL" in upper or "FEHLER" in upper or "TRACEBACK" in upper:
            level = logging.ERROR
        elif upper.startswith("!!") or "WARN" in upper:
            level = logging.WARNING
        else:
            level = logging.INFO
        self.append_record(text, level)


class _LogEmitter(QObject):
    message = Signal(str, int)


class QtLogHandler(logging.Handler):
    """Leitet Logsaetze aus beliebigen Threads in eine LogView."""

    def __init__(self):
        super().__init__()
        self.emitter = _LogEmitter()
        self.setFormatter(logging.Formatter(
            "%(asctime)s %(levelname)-7s %(name)s: %(message)s", "%H:%M:%S"))

    def emit(self, record):
        try:
            self.emitter.message.emit(self.format(record).split("\n")[0], record.levelno)
        except RuntimeError:
            pass                                     # Fenster schon zu


class CameraAssignmentView(QGroupBox):
    """Welches Geraet liegt auf welchem Platz -- gemeinsam fuer Aufnahme und Betrieb.

    Die Frage am Labortag ist nicht "welcher Kameramodus?", sondern "was
    haengt auf *wrist*, was auf *scene*?". Genau so steht es hier. Der
    Platzhalter ("Simulation") steht immer zur Auswahl, auch wenn Hardware
    da ist -- zum Testen der Kette ohne Kamera ist das der kuerzeste Weg
    (AP 0.5, Wunsch Anwender 2026-09-29).

    Die Suche oeffnet UVC-Indizes probeweise und dauert; sie laeuft deshalb
    im "io"-Pool des TaskRunner und nicht im GUI-Thread.
    """

    changed = Signal()

    def __init__(self, runner, parent=None):
        super().__init__("Zuordnung", parent)
        self.runner = runner
        self.inventory = cameras_module.Inventory()
        self._boxes = {}
        #: Abbruchsignal der laufenden Suche (threading.Event) oder None.
        #: Zugleich Kennung: ein Ergebnis, das zu einem anderen Signal
        #: gehoert, stammt aus einer abgebrochenen Suche und wird verworfen.
        self._search_cancel = None

        layout = QVBoxLayout(self)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        for slot in cameras_module.SLOTS:
            box = QComboBox()
            box.setToolTip(cameras_module.SLOT_HINTS.get(slot, ""))
            box.currentIndexChanged.connect(self._on_changed)
            self._boxes[slot] = box
            form.addRow(cameras_module.SLOT_LABELS.get(slot, slot), box)
        layout.addLayout(form)

        row = QHBoxLayout()
        self.button = QPushButton("Geräte suchen")
        self.button.setToolTip(
            "Sucht Webcams (OpenCV-Indizes) und Daheng-Kameras (Galaxy SDK). "
            "Dauert einen Moment, weil jeder Index probeweise geöffnet wird. "
            "Während der Suche wird der Knopf zu \"Abbrechen\".")
        self.button.clicked.connect(self._button_clicked)
        row.addWidget(self.button)
        row.addStretch(1)
        layout.addLayout(row)

        self.notes = QLabel("")
        self.notes.setWordWrap(True)
        self.notes.setStyleSheet("color:%s" % NEUTRAL)
        layout.addWidget(self.notes)

        self._fill(cameras_module.default_assignment(self.inventory))

    # -- Suche --------------------------------------------------------------

    def refresh(self, backend_is_sim_robot=True):
        """Beim Reiterwechsel: nichts neu suchen, nur die Vorbelegung setzen.

        Neu gesucht wird nur auf Knopfdruck -- ein Reiterwechsel darf nicht
        jedes Mal sechs Kameraindizes aufmachen.
        """
        if not any(box.count() for box in self._boxes.values()):
            self._fill(cameras_module.default_assignment(
                self.inventory, backend_is_sim_robot))

    @property
    def searching(self):
        return self._search_cancel is not None

    def _button_clicked(self):
        if self.searching:
            self.cancel_search()
        else:
            self.search()

    def search(self):
        if self.searching:
            return
        import threading

        cancel = threading.Event()
        self._search_cancel = cancel
        self.button.setText("Abbrechen")
        if self.runner.busy("camera"):
            # Ein frueherer, abgebrochener Kamerazugriff haengt noch im
            # Treiber. Diese Suche wartet dahinter -- sagen, warum nichts
            # passiert, statt still zu stehen.
            self.notes.setText("Wartet auf einen vorherigen Kamerazugriff, "
                               "der noch nicht zurückgekehrt ist …")
        else:
            self.notes.setText("Suche läuft …")
        self.runner.submit(
            lambda: cameras_module.discover(cancel=cancel),
            on_done=lambda inventory: self._on_found(inventory, cancel),
            on_error=lambda error: self._on_search_failed(error, cancel),
            pool="camera")

    def cancel_search(self):
        """Sofort zurueck in den Ruhezustand; das Ergebnis wird verworfen.

        Die Suche selbst haelt erst zwischen zwei Geraeten an -- ein
        einzelnes Oeffnen ist ein Treiberaufruf. Die Oberflaeche wartet
        darauf nicht.
        """
        if self._search_cancel is None:
            return
        self._search_cancel.set()
        self._search_cancel = None
        self.button.setText("Geräte suchen")
        self.notes.setText("Suche abgebrochen.")
        log.info("Geraetesuche abgebrochen")

    def _on_found(self, inventory, cancel):
        if cancel is not self._search_cancel:
            return  # abgebrochen -- Ergebnis gehoert zu keiner Suche mehr
        self._search_cancel = None
        self.button.setText("Geräte suchen")
        self.inventory = inventory
        keep = self.assignment()
        self._fill(keep)
        lines = inventory.note_lines()
        self.notes.setText("\n".join(lines) if lines else
                           "%d Quelle(n) gefunden." % len(inventory.devices))

    def _on_search_failed(self, error, cancel):
        if cancel is not self._search_cancel:
            return  # Abbruch oder veraltete Suche: nichts zu melden
        self._search_cancel = None
        self.button.setText("Geräte suchen")
        self.notes.setText("Suche gescheitert: %s" % error)
        log.error("Geraetesuche gescheitert: %s", error)

    # -- Zuordnung ----------------------------------------------------------

    def _fill(self, assignment):
        for slot, box in self._boxes.items():
            wanted = assignment.get(slot)
            box.blockSignals(True)
            box.clear()
            for device in self.inventory.for_slot(slot):
                label = device.label
                if device.detail:
                    label += "  —  " + device.detail
                if not device.available:
                    label += "   [liefert kein Bild]"
                box.addItem(label, device.key)
            if wanted is not None and box.findData(wanted) < 0:
                # Zugewiesen, aber nicht gefunden: nicht stillschweigend
                # wegwerfen -- am Labortag ist das die Information, dass die
                # Kamera nicht (mehr) da ist.
                box.addItem("%s  —  nicht gefunden" % wanted, wanted)
            index = box.findData(wanted)
            box.setCurrentIndex(index if index >= 0 else 0)
            box.blockSignals(False)
        self._on_changed()

    def assignment(self):
        """Aktuelle Zuordnung als ``{Platz: Geraeteschluessel}``."""
        return dict((slot, box.currentData())
                    for slot, box in self._boxes.items()
                    if box.currentData() is not None)

    def set_assignment(self, assignment):
        self._fill(assignment)

    def _on_changed(self):
        self.changed.emit()


def find_data(combo, value):
    """Index des Eintrags mit diesem Nutzdatenwert, oder -1.

    ``QComboBox.findData`` vergleicht wertgleiche Strings richtig, aber
    stumpf -- und fast alle Werte hier sind PFADE. ``datasets/vm`` und
    ``datasets\vm`` sind derselbe Ordner, aber nicht derselbe String:
    Die Liste kommt aus ``Path`` (unter Windows mit Backslash), der
    gesuchte Wert oft aus einem Eingabefeld, in das jemand Schraegstriche
    getippt hat. Aufgefallen beim Auswaehlen eines gerade exportierten
    Datensatzes -- der Eintrag war da, wurde aber nicht markiert.

    ``None`` trifft den Eintrag "keiner", falls es ihn gibt.
    """
    if value is None:
        return 0 if combo.count() and combo.itemData(0) is None else -1
    wanted = str(value)
    wanted_path = _as_path(wanted)
    for index in range(combo.count()):
        data = combo.itemData(index)
        if data is None:
            continue
        text = str(data)
        if text == wanted:
            return index
        if wanted_path is not None and _as_path(text) == wanted_path:
            return index
    return -1


def _as_path(text):
    """Vergleichbare Pfadform, oder None, wenn es kein Pfad ist."""
    from pathlib import Path

    try:
        return Path(text).as_posix().rstrip("/").lower()
    except (TypeError, ValueError):
        return None


def select_data(combo, value):
    """Eintrag mit diesem Wert auswaehlen. Liefert True, wenn es ihn gab."""
    index = find_data(combo, value)
    if index >= 0:
        combo.setCurrentIndex(index)
        return True
    return False


# -- Eingabefelder, die beim Scrollen nichts verstellen --------------------

def block_wheel(widget):
    """Mausrad-Ereignisse an das Elternwidget durchreichen.

    Qt aendert den Wert eines Drehfelds oder Auswahlfelds, sobald das
    Mausrad darueber steht -- auch wenn man eigentlich nur die Seite
    scrollen wollte. In einem Bedienfeld mit Bildlauf ist das eine
    Fehlerquelle: Man scrollt vorbei und hat unbemerkt die Episodenzahl
    oder den Override verstellt (Anwender, 2026-09-29).

    Das Rad wirkt hier nur noch, wenn das Feld den Tastaturfokus hat --
    dann ist es eine bewusste Eingabe. Sonst geht das Ereignis an den
    Bildlauf darueber.
    """
    widget.installEventFilter(_WHEEL_GUARD)
    # Ohne StrongFocus bekaeme das Feld den Fokus schon beim Ueberfahren.
    widget.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    return widget


class _WheelGuard(QObject):
    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.Wheel and not watched.hasFocus():
            event.ignore()
            return True                      # nicht an das Feld zustellen
        return False


#: Ein Filter fuer alle Felder -- er haelt keinen Zustand.
_WHEEL_GUARD = _WheelGuard()

"""Wiederverwendbare Qt-Bausteine der Oberflaeche.

Bewusst eigene Bausteine statt der Widgets aus der GUI der anderen Gruppe:
Die eigenstaendige Nutzung ist der Fall, der am Labortag gebraucht wird, und
die darf nicht an fremdem Code haengen, der sich jederzeit aendern kann
(Festlegung Anwender, 2026-09-29). Zuschnitt und Benennung sind trotzdem
absichtlich dieselben, damit die Einbindung als Reiter nicht auffaellt.
"""

import logging

from PySide6.QtCore import QObject, Qt, Signal, Slot
from PySide6.QtGui import QFont, QImage, QPixmap
from PySide6.QtWidgets import (QFrame, QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit,
                               QSizePolicy, QVBoxLayout)

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

"""Eigenstaendiges Fenster der BC-Oberflaeche.

    python -m bc.gui                    # alle Modi
    python -m bc.gui --mode systemcheck # direkt in einem Modus starten

Der Aufbau ist absichtlich derselbe wie in der GUI der anderen Gruppe
(Reiter oben, Log unten), damit der Reiter im Plugin-Fall nicht auffaellt.
Gemeinsam genutzt wird nichts von dort -- siehe widgets.py.
"""

import argparse
import logging
import sys

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDockWidget,
                               QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
                               QMainWindow, QMessageBox, QPushButton, QTabWidget,
                               QVBoxLayout, QWidget)

from . import MODES, _bootstrap  # noqa: F401  (_bootstrap setzt sys.path)
from . import session as session_module
from .runner import TaskRunner
from .tab_operation import OperationTab
from .tab_recording import RecordingTab
from .tab_systemcheck import SystemcheckTab
from .tab_training import TrainingTab
from .widgets import BackendBanner, LogView, QtLogHandler

log = logging.getLogger(__name__)

#: Modusname -> Reiterklasse. Neuer Modus: hier und in MODES eintragen.
TAB_CLASSES = {
    "systemcheck": SystemcheckTab,
    "recording": RecordingTab,
    "training": TrainingTab,
    "operation": OperationTab,
}


def make_tabs(session, runner, modes=MODES):
    """Baut die Modus-Reiter. Wird auch vom Plugin benutzt."""
    return [TAB_CLASSES[mode](session, runner) for mode in modes]


class BackendBar(QWidget):
    """Backendwahl und die Anzeige dessen, was wirklich dranhaengt.

    Die Auswahl links sagt, womit VERBUNDEN werden soll. Das Abzeichen
    rechts sagt, was der Controller geantwortet hat. Das ist nicht dasselbe,
    und genau deshalb stehen beide nebeneinander.
    """

    def __init__(self, session, parent=None):
        super().__init__(parent)
        self.session = session

        self.backend = QComboBox()
        for key in (session_module.BACKEND_SIM, session_module.BACKEND_NEURA):
            self.backend.addItem(session_module.BACKEND_LABELS[key], key)
        self.backend.setCurrentIndex(self.backend.findData(session.backend))
        self.backend.currentIndexChanged.connect(self._backend_changed)

        self.allow_real = QCheckBox("Reale Anlage freigeben")
        self.allow_real.setToolTip(
            "Ohne Haken wird jede Bewegung verweigert, solange die Steuerung "
            "nicht als Simulation antwortet. Software-Sperre, kein Not-Aus.")
        self.allow_real.toggled.connect(self._allow_real_toggled)

        self.override = QDoubleSpinBox(minimum=0.05, maximum=1.0, singleStep=0.05, decimals=2)
        self.override.setValue(float(session.override))
        self.override.setToolTip("Globaler Geschwindigkeits-Override 0..1.")
        self.override.valueChanged.connect(session.set_override)

        self.btn_connect = QPushButton("Verbinden")
        self.btn_disconnect = QPushButton("Trennen")
        self.btn_connect.clicked.connect(self.connect_backend)
        self.btn_disconnect.clicked.connect(self.disconnect_backend)

        controls = QGroupBox("Backend")
        row = QHBoxLayout(controls)
        row.addWidget(QLabel("Ziel"))
        row.addWidget(self.backend)
        row.addWidget(QLabel("Override"))
        row.addWidget(self.override)
        row.addWidget(self.allow_real)
        row.addWidget(self.btn_connect)
        row.addWidget(self.btn_disconnect)

        self.banner = BackendBanner()
        session.add_listener(self.banner.show_status)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(controls, 0)
        layout.addWidget(self.banner, 1)
        self._sync()

    def _backend_changed(self):
        self.session.set_backend(self.backend.currentData())
        self._sync()

    def _allow_real_toggled(self, checked):
        if checked and not self._confirm_real():
            self.allow_real.setChecked(False)
            return
        self.session.set_allow_real(checked)
        self._sync()

    def _confirm_real(self):
        return QMessageBox.question(
            self, "Reale Anlage freigeben",
            "Damit werden Bewegungen auch dann ausgeführt, wenn die Steuerung "
            "NICHT als Simulation antwortet.\n\n"
            "Die virtuelle Steuerung und die echte Control-Box hören auf "
            "dieselbe Adresse — ohne diesen Haken verweigert der Adapter jede "
            "Bewegung, solange die Simulation nicht bestätigt ist.\n\n"
            "Das ist eine Software-Sperre und ersetzt keinen Not-Aus.\n\n"
            "Wirklich freigeben?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes

    def connect_backend(self):
        try:
            self.session.connect()
        except Exception as exc:
            QMessageBox.warning(self, "Verbinden gescheitert",
                                "%s: %s" % (type(exc).__name__, exc))
        self._sync()

    def disconnect_backend(self):
        self.session.disconnect()
        self._sync()

    def _sync(self):
        connected = self.session.connected
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.backend.setEnabled(not connected)
        is_neura = self.session.backend == session_module.BACKEND_NEURA
        self.allow_real.setEnabled(is_neura and not connected)
        self.override.setEnabled(is_neura)
        self.banner.show_status(self.session.status() if connected else None)


class MainWindow(QMainWindow):
    def __init__(self, session=None, start_mode="systemcheck"):
        super().__init__()
        self.session = session or session_module.Session()
        self.runner = TaskRunner(self)
        self.setWindowTitle("Behavior Cloning — Bedienoberfläche")
        self.resize(1400, 900)

        self.bar = BackendBar(self.session)
        self.tabs = QTabWidget()
        self.mode_tabs = make_tabs(self.session, self.runner)
        for tab in self.mode_tabs:
            self.tabs.addTab(tab, tab.title)
        self.tabs.currentChanged.connect(self._tab_changed)
        if start_mode in MODES:
            self.tabs.setCurrentIndex(MODES.index(start_mode))

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addWidget(self.bar)
        layout.addWidget(self.tabs, 1)
        self.setCentralWidget(central)

        self.log_view = LogView()
        self.log_handler = QtLogHandler()
        self.log_handler.emitter.message.connect(self.log_view.append_record)
        logging.getLogger().addHandler(self.log_handler)
        dock = QDockWidget("Log", self)
        dock.setWidget(self.log_view)
        dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetMovable
                         | QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)
        self.resizeDocks([dock], [150], Qt.Orientation.Vertical)

        self.statusBar().showMessage(
            "Arbeitsverzeichnis: %s" % _bootstrap.WORKDIR)

    def _tab_changed(self, index):
        widget = self.tabs.widget(index)
        if hasattr(widget, "refresh"):
            widget.refresh()

    def closeEvent(self, event):                     # noqa: N802 (Qt-API)
        for tab in self.mode_tabs:
            try:
                tab.shutdown()
            except Exception:
                log.exception("Herunterfahren von %s gescheitert", tab.title)
        logging.getLogger().removeHandler(self.log_handler)
        self.runner.shutdown()
        self.session.close()
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Bedienoberfläche für den Behavior-Cloning-Teil")
    parser.add_argument("--mode", choices=MODES, default="systemcheck",
                        help="Modus, in dem gestartet wird")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow(start_mode=args.mode)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

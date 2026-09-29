"""Gemeinsamer Unterbau aller Modus-Reiter.

Jeder Modus ist ein eigenstaendiges QWidget, das Sitzung und TaskRunner
mitbekommt. Es gibt bewusst keine Vererbungstiefe darueber hinaus -- ein
neuer Modus ist eine neue Datei, ein Eintrag in ``MODES`` und einer in
``requirements.BY_MODE``.

``refresh()`` heisst so, weil die GUI der anderen Gruppe beim Reiterwechsel
``w.refresh()`` aufruft, wenn das Widget die Methode hat. Mit diesem Namen
greift ihre Konvention im Plugin-Fall von allein -- ohne Eingriff in
ihren Code.
"""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGroupBox, QLabel, QMessageBox, QVBoxLayout, QWidget

from . import requirements as requirements_module
from .widgets import RequirementView


class ModeTab(QWidget):
    """Basis eines Modus-Reiters: Sitzung, Arbeitsthreads, Voraussetzungen."""

    #: Modusname, muss in requirements.BY_MODE stehen.
    mode = None
    #: Ueberschrift des Reiters.
    title = None

    def __init__(self, session, runner, parent=None):
        super().__init__(parent)
        self.session = session
        self.runner = runner
        self.requirement_view = RequirementView(self.mode)

    # -- Konvention der anderen GUI ---------------------------------------

    def refresh(self):
        """Wird beim Reiterwechsel aufgerufen (eigene GUI und Plugin)."""
        self.requirement_view.refresh()

    def shutdown(self):
        """Wird beim Schliessen aufgerufen -- Threads, Kameras, Prozesse beenden.

        Im Plugin-Fall raeumt deren ``closeEvent`` nur die eigenen Reiter auf
        (hart aufgezaehlt, nicht in einer Schleife). Deshalb haengt sich
        plugin.py zusaetzlich an ``QApplication.aboutToQuit``, sonst liefe
        hier eine Kamera oder eine Servoschleife weiter.
        """

    # -- Meldungen statt versteckter Schaltflaechen ------------------------

    def ensure_requirements(self, action, keys, also_possible=None):
        """Prueft die genannten Voraussetzungen und meldet, was fehlt.

        Liefert True, wenn es losgehen kann. Fehlt etwas Erforderliches,
        erscheint eine Meldung, die benennt was fehlt und was trotzdem geht
        -- die Schaltflaeche bleibt aber da und klickbar (Festlegung
        Anwender, 2026-09-29).
        """
        missing = self.requirement_view.missing_among(keys)
        if not missing:
            return True
        lines = ["%s braucht:" % action, ""]
        blocking = False
        for result in missing:
            lines.append("  - " + result.requirement.missing_hint)
            if result.detail:
                lines.append("    (%s)" % result.detail)
            blocking = blocking or result.severity == requirements_module.REQUIRED
        if also_possible:
            lines.append("")
            lines.append(also_possible)
        text = "\n".join(lines)
        if blocking:
            QMessageBox.warning(self, "Voraussetzung fehlt", text)
            return False
        return QMessageBox.question(
            self, "Eingeschränkt möglich", text + "\n\nTrotzdem starten?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes

    def confirm(self, title, text):
        return QMessageBox.question(
            self, title, text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes


class PlaceholderTab(ModeTab):
    """Geruest eines noch nicht gebauten Modus.

    Zeigt Voraussetzungen und den geplanten Aufbau, statt einen leeren
    Reiter zu sein -- am Labortag soll erkennbar sein, was hier hinkommt
    und was davon schon geht.
    """

    #: Stichpunkte, was der Modus koennen soll.
    planned = ()
    #: Welche Kommandozeile diesen Modus heute schon abdeckt.
    today = ()

    def __init__(self, session, runner, parent=None):
        super().__init__(session, runner, parent)
        layout = QVBoxLayout(self)
        layout.addWidget(self.requirement_view)

        planned_box = QGroupBox("Geplant")
        planned_layout = QVBoxLayout(planned_box)
        for item in self.planned:
            label = QLabel("•  " + item)
            label.setWordWrap(True)
            planned_layout.addWidget(label)
        layout.addWidget(planned_box)

        if self.today:
            today_box = QGroupBox("Bis dahin über die Kommandozeile")
            today_layout = QVBoxLayout(today_box)
            for item in self.today:
                label = QLabel(item)
                label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
                label.setStyleSheet("font-family:Consolas; color:#333")
                today_layout.addWidget(label)
            layout.addWidget(today_box)

        layout.addStretch(1)

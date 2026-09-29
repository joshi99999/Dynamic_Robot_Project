"""Einbindung als zusaetzlicher Reiter in die GUI der anderen Gruppe.

Harte Vorgabe des Anwenders (2026-09-29): **an deren Code wird nichts
geaendert.** Das geht auf, weil ihr Fenster den Reiterbehaelter als
gewoehnliches Attribut ``tabs`` haelt -- der Reiter laesst sich von aussen
anhaengen:

    from gui.app import MainWindow          # deren Fenster
    from bc.gui.plugin import attach_to

    window = MainWindow()
    attach_to(window)
    window.show()

Zwei Eigenheiten ihrer GUI, die hier abgefangen werden:

1. ``MainWindow._tab_changed`` ruft ``w.refresh()``, wenn das Widget die
   Methode hat. Unsere Reiter heissen deshalb genau so -- damit greift ihre
   Konvention von allein.
2. Ihr ``closeEvent`` raeumt die Reiter HART AUFGEZAEHLT auf
   (tab_auto/m1/m2/m3), nicht in einer Schleife. Ein fremder Reiter wird
   dort also nie heruntergefahren. Deshalb haengt sich diese Datei
   zusaetzlich an ``QApplication.aboutToQuit`` -- sonst liefen Kamera und
   Servoschleife nach dem Schliessen weiter.

Wenn die andere Gruppe spaeter doch eine Zeile spendieren will (ein
``tab_m4`` in ihrer Aufzaehlung), aendert das hier nichts: ``attach_to``
bleibt derselbe Aufruf.
"""

import logging

from PySide6.QtWidgets import QApplication, QTabWidget

from . import MODES
from . import session as session_module
from .app import make_tabs
from .runner import TaskRunner

log = logging.getLogger(__name__)

#: Beschriftung des Reiters in der fremden GUI.
TAB_TITLE = "Behavior Cloning"


class BcPlugin(object):
    """Haelt die eingehaengten Reiter und raeumt sie wieder ab."""

    def __init__(self, session, runner, tabs, container):
        self.session = session
        self.runner = runner
        self.tabs = tabs
        self.container = container
        self._closed = False

    def shutdown(self):
        if self._closed:
            return
        self._closed = True
        for tab in self.tabs:
            try:
                tab.shutdown()
            except Exception:
                log.exception("Herunterfahren von %s gescheitert", tab.title)
        self.runner.shutdown()
        self.session.close()

    def detach(self):
        """Reiter wieder entfernen -- fuer Tests und zum Nachladen."""
        for tab in self.tabs:
            index = self.container.indexOf(tab)
            if index >= 0:
                self.container.removeTab(index)
            tab.setParent(None)
        self.shutdown()


def find_tab_container(window):
    """Sucht den Reiterbehaelter des fremden Fensters.

    Erst das Attribut ``tabs`` (so heisst er in ihrer app.py), sonst der
    erste QTabWidget im Fenster. Beides ohne Eingriff in ihren Code.
    """
    container = getattr(window, "tabs", None)
    if isinstance(container, QTabWidget):
        return container
    found = window.findChild(QTabWidget)
    if found is None:
        raise RuntimeError(
            "Kein QTabWidget im Fenster %s gefunden -- der Reiter kann nicht "
            "eingehängt werden." % type(window).__name__)
    return found


def attach_to(window, session=None, modes=MODES, single_tab=True, title=TAB_TITLE):
    """Haengt die BC-Oberflaeche als Reiter an ein fremdes Fenster.

    ``single_tab=True`` legt EINEN Reiter "Behavior Cloning" mit den vier
    Modi darin an -- so bleibt deren Reiterleiste uebersichtlich.
    ``single_tab=False`` haengt die vier Modi einzeln daneben.

    Gibt ein :class:`BcPlugin` zurueck; aufheben muss man es nur, wenn man
    spaeter ``detach()`` aufrufen will -- das Herunterfahren beim Beenden
    haengt bereits an ``aboutToQuit``.
    """
    container = find_tab_container(window)
    session = session or session_module.Session()
    runner = TaskRunner(window)
    tabs = make_tabs(session, runner, modes)

    if single_tab:
        inner = QTabWidget()
        for tab in tabs:
            inner.addTab(tab, tab.title)
        inner.currentChanged.connect(
            lambda index: _refresh(inner.widget(index)))
        # Damit ihr _tab_changed() auch den inneren Reiter aktualisiert.
        inner.refresh = lambda: _refresh(inner.currentWidget())
        container.addTab(inner, title)
        attached = [inner]
    else:
        for tab in tabs:
            container.addTab(tab, "%s: %s" % (title, tab.title))
        attached = list(tabs)

    plugin = BcPlugin(session, runner, tabs, container)
    plugin.attached = attached

    app = QApplication.instance()
    if app is not None:
        app.aboutToQuit.connect(plugin.shutdown)
    else:
        log.warning("Keine QApplication -- Herunterfahren nicht angemeldet.")
    log.info("Behavior-Cloning-Reiter eingehängt (%d Modi)", len(tabs))
    return plugin


def detach(plugin):
    plugin.detach()


def _refresh(widget):
    if widget is not None and hasattr(widget, "refresh"):
        widget.refresh()

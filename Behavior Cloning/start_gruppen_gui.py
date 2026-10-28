"""Startet die GUI der anderen Gruppe mit dem Reiter "Behavior Cloning".

Fuer das Repo ``ProjektMaster_DynamicPicking``, in dem dieser Ordner als
``behavior_cloning/`` neben ``code/`` liegt (INTEGRATION.md). Deren Code
wird dabei NICHT veraendert: ihr Fenster wird importiert, erzeugt und der
Reiter von aussen angehaengt (bc/gui/plugin.py).

Laeuft mit DEREN Python (``code/.venv``). Training, Export und Aufnahme
startet der Reiter als Unterprozesse mit dem Python der BC-Umgebung --
dafuer muss ``BC_PYTHON`` gesetzt sein; das erledigt
``BC-in-Gruppen-GUI.bat``.

    python start_gruppen_gui.py [--tab auto|m1|m2|m3|settings]
"""

import argparse
import logging
import os
import sys
from pathlib import Path

BC_DIR = Path(__file__).resolve().parent
CODE_DIR = BC_DIR.parent / "code"


def main():
    if not (CODE_DIR / "gui" / "app.py").is_file():
        sys.exit("GUI der Gruppe nicht gefunden: %s\n"
                 "Dieser Ordner muss neben code/ liegen (INTEGRATION.md)."
                 % (CODE_DIR / "gui" / "app.py"))

    # Deren Pakete (gui, common, m1_... ) und unseres (bc). Deren
    # start_gui.bat startet im Ordner code/ -- dasselbe hier.
    for path in (BC_DIR, CODE_DIR):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    os.chdir(CODE_DIR)

    from bc.gui import _bootstrap  # noqa: F401  (neurapy-Pfad)
    from bc.gui.plugin import attach_to
    from gui.app import TABS, MainWindow
    from PySide6.QtWidgets import QApplication

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--tab", choices=TABS, default="auto")
    args = parser.parse_args()

    # Wie deren gui.app.main(), nur mit eingehaengtem Reiter.
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow(start_tab=args.tab)
    attach_to(window)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

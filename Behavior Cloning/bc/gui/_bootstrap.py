"""Pfad-Setup, damit sich die GUI auch direkt starten laesst (wie apps/_bootstrap.py).

Am Anfang der Einstiegspunkte::

    from . import _bootstrap  # noqa: F401

Wird die GUI als Modul importiert (``from bc.gui import ...``), ist der Pfad
ohnehin gesetzt; das Modul ist dann wirkungslos. Gebraucht wird es beim
Start ueber ``python bc/gui/__main__.py`` und fuer ``neurapy`` aus dem
Nachbarordner "Robot Controller".
"""

import sys
from pathlib import Path

_GUI_DIR = Path(__file__).resolve().parent
_BC_DIR = _GUI_DIR.parent.parent      # "Behavior Cloning" -> enthaelt bc/
_REPO_ROOT = _BC_DIR.parent           # Repo-Wurzel

#: ``neurapy`` liegt im Nachbarordner "Robot Controller" (Fremdcode von
#: Neura). Dieselbe Suche wie in apps/_bootstrap.py und tools/_bootstrap.py.
_NEURAPY_DIRS = [
    p
    for p in (_REPO_ROOT / "Robot Controller", _REPO_ROOT)
    if (p / "neurapy" / "__init__.py").is_file()
]

for _path in (_BC_DIR, _REPO_ROOT, *_NEURAPY_DIRS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

# Kein __pycache__ in fremden Ordnern ("Robot Controller") anlegen.
sys.dont_write_bytecode = True

#: Arbeitsverzeichnis fuer Unterprozesse (tests/, tools/, apps/ erwarten
#: den Ordner "Behavior Cloning" als CWD).
WORKDIR = _BC_DIR
REPO_ROOT = _REPO_ROOT

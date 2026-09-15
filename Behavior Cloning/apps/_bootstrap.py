"""Pfad-Setup fuer die Skripte in diesem Ordner (wie tools/_bootstrap.py).

Am Anfang jedes App-Skripts::

    import _bootstrap  # noqa: F401
"""

import sys
from pathlib import Path

_APPS_DIR = Path(__file__).resolve().parent
_BC_DIR = _APPS_DIR.parent           # "Behavior Cloning" -> enthaelt bc/
_REPO_ROOT = _BC_DIR.parent          # Repo-Wurzel

#: ``neurapy`` liegt im Nachbarordner "Robot Controller" (Fremdcode von
#: Neura). Ohne diesen Pfad scheitert ``--robot neura`` schon beim Import.
#: Dieselbe Suche wie in tools/_bootstrap.py.
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

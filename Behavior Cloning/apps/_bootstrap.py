"""Pfad-Setup fuer die Skripte in diesem Ordner (wie tools/_bootstrap.py).

Am Anfang jedes App-Skripts::

    import _bootstrap  # noqa: F401
"""

import sys
from pathlib import Path

_APPS_DIR = Path(__file__).resolve().parent
_BC_DIR = _APPS_DIR.parent           # "Behavior Cloning" -> enthaelt bc/
_REPO_ROOT = _BC_DIR.parent          # Repo-Wurzel        -> enthaelt neurapy/

for _path in (_BC_DIR, _REPO_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

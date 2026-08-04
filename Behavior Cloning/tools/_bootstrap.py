"""Pfad-Setup fuer die Skripte in diesem Ordner.

Noetig, weil der Projektordner ein Leerzeichen im Namen hat ("Behavior
Cloning") und damit selbst kein importierbares Paket sein kann, und weil
``neurapy`` in der Repo-Wurzel liegt.

Am Anfang jedes Tool-Skripts::

    import _bootstrap  # noqa: F401
"""

import sys
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent
_BC_DIR = _TOOLS_DIR.parent          # "Behavior Cloning" -> enthaelt bc/
_REPO_ROOT = _BC_DIR.parent          # Repo-Wurzel        -> enthaelt neurapy/

for _path in (_BC_DIR, _REPO_ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

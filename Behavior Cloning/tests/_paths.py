"""Pfad-Setup fuer die Tests (Ordnername enthaelt ein Leerzeichen,
deshalb kein regulaeres Paket -- wie tools/_bootstrap.py)."""

import sys
from pathlib import Path

_TESTS_DIR = Path(__file__).resolve().parent
_BC_DIR = _TESTS_DIR.parent
_REPO_ROOT = _BC_DIR.parent

#: ``neurapy`` liegt im Nachbarordner "Robot Controller" (Fremdcode). Ohne
#: diesen Pfad ist ``--robot=neura`` nicht ausfuehrbar. Siehe
#: tools/_bootstrap.py, das dieselbe Suche macht.
_NEURAPY_DIRS = [
    p
    for p in (_REPO_ROOT / "Robot Controller", _REPO_ROOT)
    if (p / "neurapy" / "__init__.py").is_file()
]

for _path in (_BC_DIR, _REPO_ROOT, _TESTS_DIR, *_NEURAPY_DIRS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

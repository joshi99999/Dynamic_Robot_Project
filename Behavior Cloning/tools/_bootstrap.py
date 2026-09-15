"""Pfad-Setup fuer die Skripte in diesem Ordner.

Noetig, weil der Projektordner ein Leerzeichen im Namen hat ("Behavior
Cloning") und damit selbst kein importierbares Paket sein kann, und weil
``neurapy`` als Fremdcode ausserhalb dieses Ordners liegt.

Am Anfang jedes Tool-Skripts::

    import _bootstrap  # noqa: F401
"""

import sys
from pathlib import Path

_TOOLS_DIR = Path(__file__).resolve().parent
_BC_DIR = _TOOLS_DIR.parent          # "Behavior Cloning" -> enthaelt bc/
_REPO_ROOT = _BC_DIR.parent          # Repo-Wurzel

#: Kandidaten fuer den Ordner, der das ``neurapy``-Paket enthaelt. Es liegt
#: im Nachbarordner "Robot Controller" (Fremdcode von Neura, wird von hier
#: aus nicht veraendert) -- frueher lag es in der Repo-Wurzel, deshalb
#: bleibt sie als Kandidat stehen. Eingehaengt wird nur, was existiert; ist
#: neurapy regulaer installiert, greift ohnehin sys.path.
_NEURAPY_CANDIDATES = (_REPO_ROOT / "Robot Controller", _REPO_ROOT)


def _neurapy_dirs():
    return [p for p in _NEURAPY_CANDIDATES if (p / "neurapy" / "__init__.py").is_file()]


for _path in (_BC_DIR, _REPO_ROOT, *_neurapy_dirs()):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

# Kein __pycache__ in fremden Ordnern anlegen: der Import von ``neurapy``
# wuerde sonst Bytecode in "Robot Controller" schreiben, der hier nicht
# hingehoert. Kostet beim Start einiger weniger Module nichts Messbares.
sys.dont_write_bytecode = True

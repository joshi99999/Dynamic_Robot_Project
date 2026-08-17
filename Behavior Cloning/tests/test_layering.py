"""Schichtentrennung (AP 0.3): Pipeline importiert nie neurapy/gxipy.

Die verbindliche Regel aus dem Requirements-Dokument, als Test statt als
Disziplin: kein Modul ausserhalb von bc/adapters/ darf ``neurapy`` oder
``gxipy`` referenzieren -- auch nicht verzoegert in einer Funktion.
"""

import re
from pathlib import Path

import _paths  # noqa: F401

import bc

BC_DIR = Path(bc.__file__).resolve().parent
FORBIDDEN = re.compile(
    r"^\s*(import\s+(neurapy|gxipy)|from\s+(neurapy|gxipy)[.\s])", re.MULTILINE
)


def test_pipeline_layer_has_no_hardware_imports():
    offenders = []
    for path in BC_DIR.glob("*.py"):
        text = path.read_text(encoding="utf-8")
        if FORBIDDEN.search(text):
            offenders.append(path.name)
    assert not offenders, (
        "Hardware-Import ausserhalb von bc/adapters/: %s" % offenders
    )


def test_adapters_import_lazily():
    # Auch die Adapter selbst muessen ohne installierte SDKs importierbar
    # sein (Import erst beim Verbinden/Oeffnen).
    import importlib

    for module in ("bc.adapters.neura", "bc.adapters.cam_daheng"):
        importlib.import_module(module)

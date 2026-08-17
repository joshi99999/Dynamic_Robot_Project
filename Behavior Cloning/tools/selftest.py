"""Selbsttest der hardwarefreien Module.

Delegiert an die Testsuite in tests/ -- laeuft ohne pytest, ohne Roboter,
ohne Kameras und ohne SDKs auf jedem Rechner mit numpy/opencv/scipy.

Ausfuehren (aus dem Ordner "Behavior Cloning"):
    python tools/selftest.py
"""

import sys
from pathlib import Path

import _bootstrap  # noqa: F401

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests"))

import run_all  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(run_all.main())

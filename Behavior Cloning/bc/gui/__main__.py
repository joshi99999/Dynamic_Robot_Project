"""Einstieg: python -m bc.gui"""

from . import _bootstrap  # noqa: F401  (setzt sys.path, auch bei Direktstart)
from .app import main

if __name__ == "__main__":
    raise SystemExit(main())

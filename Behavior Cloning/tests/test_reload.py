"""Code-Stand und Neustart der Oberflaeche; Referenzbilder je Aufruf.

Wunsch Anwender 2026-10-26: nach Aenderungen nicht von Hand neu starten.
Qt-frei -- geprueft wird die Logik in bc/gui/reload.py, nicht der Knopf.
"""

import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.gui import reload as reload_module
from bc.ports import Frame

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "apps"))


def _touch(path, text="x = 1\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def test_changes_are_split_into_restart_and_immediate(tmp_path):
    _touch(tmp_path / "bc" / "a.py")
    _touch(tmp_path / "apps" / "record.py")
    _touch(tmp_path / "bc" / "__pycache__" / "a.cpython-311.py")
    watch = reload_module.CodeWatch(tmp_path)
    assert "bc/__pycache__/a.cpython-311.py" not in watch.start
    assert watch.status() == ([], [])
    assert "unverändert" in reload_module.describe([], [])

    later = time.time() + 5
    os.utime(tmp_path / "bc" / "a.py", (later, later))
    os.utime(tmp_path / "apps" / "record.py", (later, later))
    _touch(tmp_path / "tools" / "neu.py")                 # neu
    restart, immediate = watch.status()
    assert restart == ["bc/a.py"]
    assert immediate == ["apps/record.py", "tools/neu.py"]
    text = reload_module.describe(restart, immediate)
    assert "Neustart nötig" in text and "ohne Neustart" in text

    (tmp_path / "bc" / "a.py").unlink()                   # geloescht zaehlt auch
    assert watch.status()[0] == ["bc/a.py"]


def test_restart_uses_the_same_python_and_mode():
    program, args = reload_module.restart_command("recording")
    assert program == sys.executable
    assert args == ["-m", "bc.gui", "--mode", "recording"]


def test_busy_tabs_block_the_restart():
    free = SimpleNamespace(title="Training", busy_reason=lambda: None)
    busy = SimpleNamespace(title="Aufnahme", busy_reason=lambda: "Kameravorschau ist offen")

    def broken():
        raise RuntimeError("kaputt")

    odd = SimpleNamespace(title="Betrieb", busy_reason=broken)
    assert reload_module.busy_reasons([free]) == []
    reasons = reload_module.busy_reasons([free, busy, odd])
    assert reasons[0] == "Aufnahme: Kameravorschau ist offen"
    assert reasons[1].startswith("Betrieb: Zustand nicht lesbar")  # im Zweifel blockieren


def test_reference_images_only_for_real_cameras(tmp_path):
    import record

    image = np.zeros((1080, 1440, 3), dtype=np.uint8)
    image[:, :, 0] = 200
    caps = [SimpleNamespace(name="wrist", latest=lambda: Frame(image, 0.0, 0, "wrist")),
            SimpleNamespace(name="scene", latest=lambda: None)]
    cfgs = [config.CameraConfig(name="wrist", backend="daheng"),
            config.CameraConfig(name="scene", backend="daheng")]
    saved = record.save_reference_images(caps, cfgs, tmp_path, block="B 07/x")
    assert list(saved) == ["wrist"]  # Szene ohne Bild: still uebersprungen
    path = tmp_path / saved["wrist"]
    assert path.is_file() and path.parent.name == "referenzbilder"
    assert "B-07-x" in path.name
    import cv2

    back = cv2.imread(str(path))
    assert back.shape == (1080, 1440, 3) and back[0, 0, 2] == 200  # RGB richtig herum
    # nur Platzhalter: nichts gespeichert, kein Ordner
    sim = [config.CameraConfig(name="wrist", backend="sim")]
    assert record.save_reference_images(caps[:1], sim, tmp_path / "leer") == {}
    assert not (tmp_path / "leer").exists()

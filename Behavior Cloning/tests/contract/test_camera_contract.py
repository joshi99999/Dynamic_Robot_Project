"""Contract-Tests des CameraPort (AP 0.5).

    pytest tests/contract -q                    # sim (heute)
    pytest tests/contract -q --camera=uvc       # Szenenkamera am Geraet
    pytest tests/contract -q --camera=daheng    # Wrist-Kamera am Geraet
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import _paths  # noqa: F401,E402

import numpy as np  # noqa: E402

from bc.ports import CameraError, Frame  # noqa: E402


def test_open_read_close(camera):
    camera.open()
    assert camera.is_open
    frame = camera.read()
    assert isinstance(frame, Frame)
    camera.close()
    assert not camera.is_open


def test_frame_contract(camera):
    camera.open()
    frame = camera.read()
    # RGB, uint8, (H, W, 3) -- die verbindliche Bildzusage (ports.py)
    assert frame.image.dtype == np.uint8
    assert frame.image.ndim == 3 and frame.image.shape[2] == 3
    assert frame.image.shape[0] > 0 and frame.image.shape[1] > 0
    assert frame.source == camera.name
    assert frame.timestamp > 0
    camera.close()


def test_timestamps_and_indices_increase(camera):
    camera.open()
    frames = [camera.read() for _ in range(5)]
    for a, b in zip(frames[:-1], frames[1:]):
        assert b.index == a.index + 1
        assert b.timestamp >= a.timestamp
    camera.close()


def test_read_after_close_raises(camera):
    camera.open()
    camera.read()
    camera.close()
    try:
        camera.read()
        assert False, "CameraError erwartet"
    except CameraError:
        pass


def test_close_is_idempotent(camera):
    camera.open()
    camera.close()
    camera.close()  # zweites close darf nicht werfen

"""ArUco-Rektifizierung und Posen-Ueberwachung (AP 1.4).

Nutzt die synthetische Marker-Szene aus rectify.draw_synthetic_scene:
echte cv2.aruco-Marker, perspektivisch verzerrt wie von einer (ggf.
verschobenen) Kamera gesehen -- damit wird die komplette Kette
Erkennung -> Homographie -> Entzerrung -> Ueberwachung ohne Kamera geprueft.
"""

import _paths  # noqa: F401

import numpy as np

from bc.rectify import (
    MarkerLayout,
    Rectifier,
    RectifyResult,
    WorkspaceView,
    _apply_h,
    draw_synthetic_scene,
)


def _layout_view():
    # 4 Marker an den Ecken eines 40x30-cm-Workspace
    layout = MarkerLayout(
        markers={
            0: (0.05, 0.05),
            1: (0.35, 0.05),
            2: (0.35, 0.25),
            3: (0.05, 0.25),
        },
        marker_size_m=0.04,
    )
    view = WorkspaceView(x_range_m=(0.0, 0.40), y_range_m=(0.0, 0.30))
    return layout, view


def _camera_h(dx=0.0, dy=0.0, tilt=0.0):
    """Homographie kanonische Ansicht -> Kamerabild (simulierte Kamera).

    Basis-Skalierung 0.9 haelt alle Marker trotz Verschiebung/Kippung im
    Bild -- sonst testet man Marker-Ausfall statt Rektifizierung.
    """
    H = np.array(
        [
            [0.9 + tilt, tilt * 0.5, 10.0 + dx],
            [0.0, 0.9 + tilt * 0.7, 8.0 + dy],
            [tilt * 1e-4, tilt * 5e-5, 1.0],
        ]
    )
    return H


def test_homography_recovers_marker_positions():
    layout, view = _layout_view()
    image = draw_synthetic_scene(layout, view, _camera_h(dx=15, dy=-8, tilt=0.05))
    rect = Rectifier(layout, view, min_markers=2)

    H, n = rect.compute_homography(image)
    assert H is not None
    assert n >= 3  # mindestens 3 der 4 Marker erkannt

    # Die Homographie muss die Marker-Ecken auf ihre kanonischen
    # Pixelpositionen zurueckbilden
    found = rect._detect(image)
    for marker_id, corners_px in found.items():
        world = layout.corners_world(marker_id)
        expected = np.array([view.world_to_pixel(c) for c in world])
        mapped = _apply_h(H, corners_px)
        err = np.linalg.norm(mapped - expected, axis=1).max()
        assert err < 3.0, "Marker %d: %0.2f px Fehler" % (marker_id, err)


def test_rectified_image_is_canonical():
    layout, view = _layout_view()
    rect = Rectifier(layout, view)

    # Zwei verschieden verzerrte Aufnahmen derselben Szene ...
    img_a = draw_synthetic_scene(layout, view, _camera_h(dx=10, dy=5, tilt=0.03))
    img_b = draw_synthetic_scene(layout, view, _camera_h(dx=-12, dy=8, tilt=-0.02))
    out_a = rect.rectify(img_a)
    out_b = rect.rectify(img_b)
    assert isinstance(out_a, RectifyResult)

    # ... muessen nach der Rektifizierung (nahezu) identisch sein
    diff = np.abs(
        out_a.image.astype(np.int16) - out_b.image.astype(np.int16)
    ).mean()
    assert diff < 12.0, "mittlere Bilddifferenz %.1f" % diff


def test_shift_monitoring_detects_camera_move():
    layout, view = _layout_view()
    rect = Rectifier(layout, view, shift_warn_px=8.0)

    reference = draw_synthetic_scene(layout, view, _camera_h())
    rect.set_reference(reference)

    # Unverschobene Kamera: ok
    ok_result = rect.rectify(draw_synthetic_scene(layout, view, _camera_h()))
    assert ok_result.ok
    assert ok_result.shift_px < 3.0

    # Kamera um ~20 px verschoben: Warnung (AP 1.4 -- aus einem stillen
    # Fehler wird ein sichtbarer)
    moved = draw_synthetic_scene(layout, view, _camera_h(dx=20, dy=-15))
    moved_result = rect.rectify(moved)
    assert moved_result.shift_px > 8.0
    assert not moved_result.ok


def test_fallback_when_markers_hidden():
    layout, view = _layout_view()
    rect = Rectifier(layout, view)

    visible = draw_synthetic_scene(layout, view, _camera_h(dx=5))
    rect.rectify(visible)  # legt die letzte gueltige Homographie an

    # Alle Marker verdeckt (leeres Bild) -> letzte gueltige verwenden + melden
    blank = np.full_like(visible, 128)
    result = rect.rectify(blank)
    assert result.used_fallback
    assert result.n_markers == 0
    assert not result.ok

    # Ganz ohne vorherige Homographie: harter Fehler statt stillem Weiter
    fresh = Rectifier(layout, view)
    try:
        fresh.rectify(blank)
        assert False, "RuntimeError erwartet"
    except RuntimeError:
        pass

"""ArUco-Homographie-Rektifizierung der Top-View-Kamera (AP 1.4).

Zwei Aufgaben mit denselben Markern:

* **(b) Rektifizierung (entschieden -- wird umgesetzt):** Aus den sichtbaren
  Marker-Ecken wird eine Homographie auf eine kanonische Draufsicht
  berechnet und jedes Bild dorthin entzerrt + auf den festen
  Workspace-Ausschnitt gecroppt. Muss bei Aufzeichnung und Inferenz
  IDENTISCH angewandt werden.
* **(a) Ueberwachung:** Die aktuelle Homographie wird gegen eine Referenz
  verglichen; ueberschreitet die Abweichung eine Schwelle, wird gewarnt
  (Dashboard AP 1.2) bzw. die Episode verworfen / der Inferenzstart
  blockiert -- aus einem stillen Fehler wird ein sichtbarer.

Grenzen (AP 1.4): exakt nur in der Markerebene; Objekte mit Hoehe
unterliegen Parallaxe. Verdeckte Marker: es wird mit den sichtbaren
gerechnet; bei zu wenigen faellt die Rektifizierung auf die letzte gueltige
Homographie zurueck und meldet das.

HINWEIS Abnahme (AP 0.6): Marker-Orientierung auf dem realen Tisch muss
der hier angenommenen Konvention entsprechen (Marker-x parallel Welt-x);
am Aufbau verifizieren.
"""

from dataclasses import dataclass, field

import numpy as np

from . import config


@dataclass
class MarkerLayout:
    """Physisches Marker-Layout auf dem Tisch.

    ``markers``: dict ArUco-ID -> (x, y) des Markerzentrums in Metern
    (Tischebene, gemeinsames Bezugssystem mit dem CV-Team).
    ``marker_size_m``: Kantenlaenge der gedruckten Marker.
    ``dictionary``: Name des ArUco-Woerterbuchs (cv2.aruco.DICT_*).
    """

    markers: dict
    marker_size_m: float = 0.05
    dictionary: str = "DICT_4X4_50"

    def corners_world(self, marker_id):
        """Welt-Koordinaten der 4 Ecken in ArUco-Reihenfolge.

        ArUco liefert [oben-links, oben-rechts, unten-rechts, unten-links]
        bezogen auf das gedruckte Muster; angenommen wird ein axial zum
        Welt-KS ausgerichteter Marker (x rechts, y "oben" auf dem Tisch).
        """
        cx, cy = self.markers[marker_id]
        h = self.marker_size_m / 2.0
        return np.array(
            [
                [cx - h, cy + h],
                [cx + h, cy + h],
                [cx + h, cy - h],
                [cx - h, cy - h],
            ]
        )


@dataclass
class WorkspaceView:
    """Kanonischer Bildausschnitt: welcher Tischbereich wird auf welches
    Pixelraster abgebildet (fester Workspace-Crop, AP 1.4)."""

    x_range_m: tuple  # (x_min, x_max) auf dem Tisch
    y_range_m: tuple
    width_px: int = config.IMAGE_WIDTH
    height_px: int = config.IMAGE_HEIGHT

    def world_to_pixel(self, xy):
        """(x, y) in Metern -> (u, v) in Pixeln der kanonischen Ansicht.

        v waechst nach unten (Bildkonvention); y waechst im Bild nach oben.
        """
        x, y = float(xy[0]), float(xy[1])
        u = (x - self.x_range_m[0]) / (self.x_range_m[1] - self.x_range_m[0])
        v = (self.y_range_m[1] - y) / (self.y_range_m[1] - self.y_range_m[0])
        return np.array([u * self.width_px, v * self.height_px])


@dataclass
class RectifyResult:
    image: np.ndarray  # kanonische Ansicht (H, W, 3)
    n_markers: int  # Anzahl verwendeter Marker
    used_fallback: bool  # letzte gueltige Homographie verwendet?
    shift_px: float  # Abweichung zur Referenz (Ueberwachung); nan ohne Referenz
    ok: bool


class Rectifier(object):
    """Berechnet und ueberwacht die Homographie Bild -> kanonische Ansicht."""

    def __init__(
        self,
        layout,
        view,
        min_markers=2,
        shift_warn_px=8.0,
    ):
        self.layout = layout
        self.view = view
        self.min_markers = min_markers
        self.shift_warn_px = shift_warn_px
        self._last_h = None
        self._reference_h = None
        self._detector = None

    # -- Erkennung ---------------------------------------------------------

    def _detect(self, image):
        """ArUco-Erkennung; kapselt die API-Unterschiede von OpenCV."""
        import cv2

        if self._detector is None:
            dictionary = cv2.aruco.getPredefinedDictionary(
                getattr(cv2.aruco, self.layout.dictionary)
            )
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _rejected = self._detector.detectMarkers(image)
        if ids is None:
            return {}
        found = {}
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            if int(marker_id) in self.layout.markers:
                found[int(marker_id)] = marker_corners.reshape(4, 2)
        return found

    def compute_homography(self, image):
        """Homographie aus allen sichtbaren, bekannten Markern.

        Rueckgabe: (H, n_markers) oder (None, n) bei zu wenigen Markern.
        """
        import cv2

        found = self._detect(image)
        if len(found) < self.min_markers:
            return None, len(found)

        src = []
        dst = []
        for marker_id, corners_px in found.items():
            world = self.layout.corners_world(marker_id)
            for corner_px, corner_world in zip(corners_px, world):
                src.append(corner_px)
                dst.append(self.view.world_to_pixel(corner_world))
        H, _mask = cv2.findHomography(np.asarray(src), np.asarray(dst), cv2.RANSAC)
        return H, len(found)

    # -- Anwendung ---------------------------------------------------------

    def set_reference(self, image):
        """Hinterlegt die Referenz-Homographie (bei der Aufzeichnung der
        ersten Episode bzw. beim Einrichten, AP 1.4 Ueberwachung)."""
        H, n = self.compute_homography(image)
        if H is None:
            raise RuntimeError(
                "Referenz nicht bestimmbar: nur %d Marker sichtbar (min. %d)"
                % (n, self.min_markers)
            )
        self._reference_h = H
        self._last_h = H
        return H

    def rectify(self, image):
        """Entzerrt ein Bild in die kanonische Ansicht -> :class:`RectifyResult`.

        Bei zu wenigen sichtbaren Markern wird die letzte gueltige
        Homographie verwendet und ``used_fallback`` gesetzt (AP 1.4).
        """
        import cv2

        H, n = self.compute_homography(image)
        used_fallback = False
        if H is None:
            if self._last_h is None:
                raise RuntimeError(
                    "Keine Homographie verfuegbar: nur %d Marker sichtbar und "
                    "keine letzte gueltige Homographie vorhanden" % n
                )
            H = self._last_h
            used_fallback = True
        else:
            self._last_h = H

        warped = cv2.warpPerspective(
            image, H, (self.view.width_px, self.view.height_px)
        )
        shift = self._shift_px(H)
        ok = not used_fallback and (
            np.isnan(shift) or shift <= self.shift_warn_px
        )
        return RectifyResult(
            image=warped,
            n_markers=n,
            used_fallback=used_fallback,
            shift_px=shift,
            ok=ok,
        )

    # -- Ueberwachung ------------------------------------------------------

    def _shift_px(self, H):
        """Abweichung der aktuellen zur Referenz-Homographie.

        Gemessen als maximale Verschiebung der vier Workspace-Ecken, wenn
        man sie mit Referenz- und aktueller Homographie zurueckprojiziert.
        """
        if self._reference_h is None:
            return float("nan")
        corners = np.array(
            [
                [0.0, 0.0],
                [self.view.width_px, 0.0],
                [self.view.width_px, self.view.height_px],
                [0.0, self.view.height_px],
            ]
        )
        ref_inv = np.linalg.inv(self._reference_h)
        cur_inv = np.linalg.inv(H)
        pts_ref = _apply_h(ref_inv, corners)
        pts_cur = _apply_h(cur_inv, corners)
        # Rueckprojektion in Kamerapixel: wie weit ist die Kamera "gewandert"
        return float(np.max(np.linalg.norm(pts_ref - pts_cur, axis=1)))


def _apply_h(H, points):
    """Wendet eine Homographie auf (N, 2)-Punkte an."""
    pts = np.hstack([points, np.ones((len(points), 1))])
    out = (H @ pts.T).T
    return out[:, :2] / out[:, 2:3]


def draw_synthetic_scene(layout, view, camera_h, background=64):
    """Rendert ein synthetisches Kamerabild mit echten ArUco-Markern.

    Fuer Tests: die Marker werden an ihren Welt-Positionen in die kanonische
    Ansicht gezeichnet und dann mit ``camera_h`` (kanonisch -> Kamerabild)
    perspektivisch verzerrt -- simuliert also eine (ggf. verschobene)
    Kamera. Damit laesst sich die Rektifizierung end-to-end gegen
    ``cv2.aruco`` pruefen, ohne echte Kamera.
    """
    import cv2

    canvas = np.full(
        (view.height_px, view.width_px, 3), background, dtype=np.uint8
    )
    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, layout.dictionary)
    )
    for marker_id in layout.markers:
        world = layout.corners_world(marker_id)
        px = np.array([view.world_to_pixel(c) for c in world])
        x0, y0 = px.min(axis=0).astype(int)
        x1, y1 = px.max(axis=0).astype(int)
        side = max(8, min(x1 - x0, y1 - y0))
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
        marker_rgb = np.stack([marker] * 3, axis=-1)
        # Weisser Rand ("quiet zone"), noetig fuer zuverlaessige Erkennung
        pad = max(2, side // 8)
        y0p, x0p = max(0, y0 - pad), max(0, x0 - pad)
        y1p, x1p = min(view.height_px, y0 + side + pad), min(view.width_px, x0 + side + pad)
        canvas[y0p:y1p, x0p:x1p] = 255
        canvas[y0 : y0 + side, x0 : x0 + side] = marker_rgb

    return cv2.warpPerspective(
        canvas,
        camera_h,
        (view.width_px, view.height_px),
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(background, background, background),
    )

"""Zentrale Konstanten und Tuning-Parameter der Behavior-Cloning-Pipeline.

Alle Werte, die an mehreren Stellen gebraucht werden, stehen hier -- damit
Aufzeichnung und Inferenz garantiert dieselben Annahmen verwenden. Ein
Auseinanderlaufen dieser Werte zwischen Training und Inferenz macht die
Policy unbrauchbar (siehe AP 1.5.1 in Requierments/requierments.md).
"""

from dataclasses import dataclass, field

# --------------------------------------------------------------------------
# Zeitverhalten (AP 1.3 / 2.6)
# --------------------------------------------------------------------------

#: Gemeinsame Zielrate fuer Aufzeichnung UND Inferenz. Verbindlich -- die
#: Geschwindigkeit der Policy ist ueber die Schrittweite bei dieser Rate
#: implizit im Datensatz codiert.
CONTROL_RATE_HZ = 15.0

#: Abgeleiteter Frame-Abstand.
CONTROL_PERIOD_S = 1.0 / CONTROL_RATE_HZ

#: Maximal zulaessige Zeitdifferenz zwischen den Quellen eines Frames.
#: Deutlich kleiner als ein halber Frame-Abstand (AP 1.3).
SYNC_MAX_SKEW_S = 0.030

# --------------------------------------------------------------------------
# Greifer (AP 2.1)
# --------------------------------------------------------------------------

#: Totzeit zwischen abgesetztem Greiferbefehl und tatsaechlich geschlossenen
#: Backen. Es gibt keine Ist-Rueckmeldung ueber NeuraPy, daher fest verankert.
#: ACHTUNG: Vorlaeufiger Arbeitswert -- noch per Video zu verifizieren.
GRIPPER_DWELL_S = 0.5
GRIPPER_DWELL_VERIFIED = False

# --------------------------------------------------------------------------
# Kinematik-Absicherungen (AP 2.4)
# --------------------------------------------------------------------------

#: Maximale Gelenkwinkelaenderung zwischen zwei Trajektorienschritten.
#: Erkennt Konfigurationsspruenge und Singularitaetsdurchgaenge auch dann,
#: wenn die IK formal eine Loesung liefert. Referenz aus dem Vortest
#: (tools/log.txt): reale Schritte lagen bei ~0.004 rad, der Wert ist also
#: bewusst grosszuegig als reiner Sprung-Detektor gewaehlt.
IK_MAX_DELTA_Q_RAD = 0.20

#: Toleranz der FK-Rueckprobe (IK-Loesung zurueckgerechnet vs. Sollpose).
IK_FK_TOL_POS_M = 1.0e-3
IK_FK_TOL_ROT_RAD = 1.0e-2

#: Achsgrenzen in rad als [(min, max), ...] je Gelenk.
#: TODO: Aus dem LARA-5-Datenblatt eintragen. Solange None, wird die
#: Grenzpruefung uebersprungen (mit einmaliger Warnung) -- IKNotFound und
#: die Delta-q-Schranke bleiben als Absicherung aktiv.
JOINT_LIMITS_RAD = None

# --------------------------------------------------------------------------
# Rauscheinspielung (AP 2.4)
# --------------------------------------------------------------------------

#: Amplitude des kartesischen Rauschens in der Transitphase.
NOISE_TRANS_AMPLITUDE_M = 0.02

#: Rotationsanteil des Rauschens.
NOISE_ROT_AMPLITUDE_RAD = 0.05

#: Sicherheitsabstand, den jede verrauschte Trajektorie zu allen
#: Kollisionsgeometrien einhalten muss.
COLLISION_MARGIN_M = 0.03

#: Frames entlang des Arms, die gegen das Kollisionsmodell geprueft werden.
#: Weniger Frames = schnellere Pruefung (jeder Frame kostet einen
#: FK-Aufruf ~2 ms, siehe tools/log.txt).
COLLISION_CHECK_FRAMES = ("tool", "wrist", "elbow")

# --------------------------------------------------------------------------
# Kameras (AP 1.1)
# --------------------------------------------------------------------------


@dataclass
class CameraConfig:
    """Konfiguration einer einzelnen Kamera.

    ``backend`` ist entweder ``"daheng"`` (USB3-Vision ueber gxipy) oder
    ``"uvc"`` (Standard-Webcam ueber OpenCV).
    """

    name: str
    backend: str
    #: Bei "uvc" der OpenCV-Geraeteindex, bei "daheng" optional die
    #: Seriennummer (None = erstes gefundenes Geraet).
    device: object = None
    width: int = 640
    height: int = 480
    #: Nur "daheng": Belichtungszeit in Mikrosekunden. None = Automatik.
    exposure_us: float = None
    #: Nur "daheng": Gain in dB. None = Automatik.
    gain_db: float = None
    #: Nur "uvc": angeforderte Kamera-FPS.
    fps: float = 30.0


#: Wrist-Kamera: Daheng VEN-161-61U3C. Aufloesung bewusst kameraseitig
#: reduziert statt spaeter in Python zu skalieren (spart USB-Bandbreite).
WRIST_CAMERA = CameraConfig(
    name="wrist",
    backend="daheng",
    width=640,
    height=480,
    exposure_us=8000.0,
)

#: Szenen-/Top-View-Kamera. Modell noch offen -- Index ggf. anpassen.
SCENE_CAMERA = CameraConfig(
    name="scene",
    backend="uvc",
    device=0,
    width=640,
    height=480,
    fps=30.0,
)

CAMERAS = (WRIST_CAMERA, SCENE_CAMERA)


# --------------------------------------------------------------------------
# Roboter
# --------------------------------------------------------------------------

#: Globaler Geschwindigkeits-Override beim Verbinden (0.0 - 1.0).
#: Bewusst gedrosselt; fuer die finale Aufzeichnung anzupassen (AP 2.6).
DEFAULT_OVERRIDE = 0.2

#: Ungefaehre Reichweite der LARA 5 -- nur zur Plausibilitaetspruefung
#: gemeldeter Posen (Erkennung von Platzhalterwerten).
MAX_REACH_M = 0.9

"""Zentrale Konstanten und Tuning-Parameter der Behavior-Cloning-Pipeline.

Alle Werte, die an mehreren Stellen gebraucht werden, stehen hier -- damit
Aufzeichnung und Inferenz garantiert dieselben Annahmen verwenden. Ein
Auseinanderlaufen dieser Werte zwischen Training und Inferenz macht die
Policy unbrauchbar (siehe AP 1.5.1 in Requierments/requierments.md).

Werte, die ohne Hardware nicht verifizierbar sind, tragen ein
``*_VERIFIED = False``-Flag bzw. einen PROVISORISCH-Hinweis (siehe AP 0.4
und die Abnahmeliste AP 0.6).
"""

import math
from dataclasses import dataclass
from pathlib import Path

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

#: Anteil der Frames einer Episode, der das Latenzbudget reissen darf,
#: bevor die ganze Episode verworfen wird (AP 5.2).
SYNC_MAX_BAD_FRAME_RATIO = 0.05

# --------------------------------------------------------------------------
# Greifer (AP 2.1)
# --------------------------------------------------------------------------

#: Totzeit zwischen abgesetztem Greiferbefehl und tatsaechlich geschlossenen
#: Backen. Es gibt keine Ist-Rueckmeldung ueber NeuraPy, daher fest verankert.
#: ACHTUNG: Vorlaeufiger Arbeitswert -- noch per Video zu verifizieren.
GRIPPER_DWELL_S = 0.5
GRIPPER_DWELL_VERIFIED = False

#: Dwell in ganzen Trajektorienschritten bei CONTROL_RATE_HZ.
GRIPPER_DWELL_STEPS = int(math.ceil(GRIPPER_DWELL_S * CONTROL_RATE_HZ))

#: Kodierung des binaeren Greifers in State- und Action-Vektor (AP 0.10).
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.0
#: Schwelle bei der Inferenz: Werte darueber gelten als "geschlossen".
GRIPPER_THRESHOLD = 0.5

# --------------------------------------------------------------------------
# Datensatz-Schema (AP 0.9 Punkt 1/2 -- Festlegung)
# --------------------------------------------------------------------------

#: observation.state = 6 Gelenkwinkel + TCP-Pose [x,y,z,qw,qx,qy,qz]
#: + 1 Greifer (kommandiert) = 14 Werte, float32.
STATE_DIM = 14

#: action = 6 Gelenkwinkel (IDEALE Solltrajektorie, t+1) + 1 Greifer = 7.
#: Absolute Zielwinkel, keine Deltas (AP 1.5.2 / 2.4).
ACTION_DIM = 7

#: Bildgroesse beider Kamerastreams nach der Vorverarbeitung, identisch
#: fuer Aufzeichnung und Inferenz (Hoehe, Breite).
IMAGE_HEIGHT = 240
IMAGE_WIDTH = 320

#: Version dieses Schemas. Bei JEDER Aenderung an Dimensionen, Reihenfolge
#: oder Einheiten hochzaehlen -- Datensaetze verschiedener Versionen duerfen
#: nicht gemischt werden.
SCHEMA_VERSION = 1

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

#: Achsgrenzen in rad als ((min, max), ...) je Gelenk.
#: PROVISORISCH: Werte stammen aus bc/data/lara5_candidate.urdf (elfin5 aus
#: dem LARA-5.0.8-Deployment). Die Zuordnung dieses Modells zur realen
#: LARA 5 ist NICHT bestaetigt (AP 0.4) -- vor der ersten physischen
#: Ausfuehrung gegen Datenblatt/Controller verifizieren.
JOINT_LIMITS_RAD = (
    (-3.14, 3.14),
    (-2.35, 2.35),
    (-2.61, 2.61),
    (-3.14, 3.14),
    (-2.56, 2.56),
    (-3.14, 3.14),
)
JOINT_LIMITS_VERIFIED = False

#: URDF-Modell fuer den SimRobot (Gleis A aus AP 0.4). Dient der Pruefung
#: der LOGIK, nicht der Geometrie -- Zahlenwerte aus der Simulation sind
#: nicht auf die Anlage uebertragbar.
URDF_PATH = Path(__file__).resolve().parent / "data" / "lara5_candidate.urdf"

# --------------------------------------------------------------------------
# Trajektorie & Geschwindigkeit (AP 2.2 / 2.6)
# --------------------------------------------------------------------------

#: Zielgeschwindigkeit des Pfadplaners in der Transitphase.
#: PROVISORISCH (AP 0.9 Punkt 5): Startwert, Festlegung nach dem ersten
#: realen Lauf. Legt die spaetere Ausfuehrungsgeschwindigkeit der Policy
#: unveraenderlich fest!
TRANSIT_SPEED_MS = 0.15

#: Reduzierte Geschwindigkeit im Endanflug (Greifen/Absetzen).
APPROACH_SPEED_MS = 0.05

# --------------------------------------------------------------------------
# Rauscheinspielung (AP 2.4)
# --------------------------------------------------------------------------

#: Amplitude des kartesischen Rauschens in der Transitphase (Zielwert der
#: OU-Standardabweichung; +-1-2 cm laut Anforderung).
NOISE_TRANS_AMPLITUDE_M = 0.015

#: Rotationsanteil des Rauschens.
NOISE_ROT_AMPLITUDE_RAD = 0.05

#: Zeitkonstante des Ornstein-Uhlenbeck-Prozesses. Grosse Werte = traege,
#: weiche Schlingerbewegung statt Zittern.
NOISE_OU_TAU_S = 0.8

#: Abstand zum Greifpunkt, ab dem die Trichter-Daempfung einsetzt.
FUNNEL_START_DIST_M = 0.15

#: Abstand, ab dem das Rauschen vollstaendig auf 0 gedaempft ist.
FUNNEL_ZERO_DIST_M = 0.02

#: Maximale Zahl an Rejection-Sampling-Versuchen, bevor die Generierung
#: einer verrauschten Trajektorie als fehlgeschlagen gilt.
NOISE_MAX_REJECTS = 20

# --------------------------------------------------------------------------
# Kollision & Geofencing (AP 2.4 / 4.2)
# --------------------------------------------------------------------------

#: Sicherheitsabstand, den jede verrauschte Trajektorie zu allen
#: Kollisionsgeometrien einhalten muss.
COLLISION_MARGIN_M = 0.03

#: Namen der Arm-Stuetzpunkte, die gegen das Kollisionsmodell geprueft
#: werden. Muessen von RobotPort.link_positions() geliefert werden.
COLLISION_CHECK_FRAMES = ("tool", "wrist", "elbow")

#: Pruefintervall des Geofence-Waechters (AP 4.2).
SAFETY_CHECK_PERIOD_S = 0.05

# --------------------------------------------------------------------------
# Kameras (AP 1.1)
# --------------------------------------------------------------------------


@dataclass
class CameraConfig:
    """Konfiguration einer einzelnen Kamera.

    ``backend`` ist einer der in ``bc.adapters`` registrierten Namen:
    ``"daheng"`` (USB3-Vision ueber gxipy), ``"uvc"`` (Webcam ueber
    OpenCV) oder ``"sim"`` (Platzhalter-Frames fuer hardwarefreie Tests).
    """

    name: str
    backend: str
    #: Bei "uvc" der OpenCV-Geraeteindex, bei "daheng" optional die
    #: Seriennummer (None = erstes gefundenes Geraet).
    device: object = None
    #: Gewuenschte Bildgroesse. Bei "daheng" ist das ein ROI-AUSSCHNITT
    #: (kein Skalieren!) und wird zentriert gesetzt; None = voller Sensor.
    width: int = IMAGE_WIDTH
    height: int = IMAGE_HEIGHT
    #: Nur "daheng": kameraseitiges Binning (1 = aus, 2 = 2x2). Reduziert
    #: die Datenmenge OHNE Sichtfeldverlust -- beim Fisheye der richtige
    #: Weg, im Gegensatz zum ROI-Crop.
    binning: int = 1
    #: Nur "daheng": Belichtungszeit in Mikrosekunden. None = Automatik.
    exposure_us: float = None
    #: Nur "daheng": Gain in dB. None = Automatik.
    gain_db: float = None
    #: Nur "uvc"/"sim": angeforderte Kamera-FPS.
    fps: float = 30.0


#: Wrist-Kamera: Daheng VEN-161-61U3C (IMX296, 1/2.9", 1440x1080).
#: Objektiv: Fisheye 1.85 mm, Bildkreis fuer 1/1.8" -- der kleinere Sensor
#: nutzt also nur den zentralen Teil des Bildkreises.
#:
#: VOLLER SENSOR (width/height = None): Beim Fisheye ist das Sichtfeld der
#: Grund fuer die Objektivwahl -- ein ROI-Crop wuerde genau das wegschneiden.
#: Die Datenreduktion laeuft stattdessen ueber 2x2-Binning (720x540, halbe
#: Datenrate, besserer Rauschabstand), die Skalierung auf die Schemagroesse
#: 240x320 dann in Software. Seitenverhaeltnis bleibt durchgehend 4:3.
WRIST_CAMERA = CameraConfig(
    name="wrist",
    backend="daheng",
    width=None,
    height=None,
    binning=2,
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

#: Hardwarefreie Variante derselben Konfiguration (AP 0.5).
SIM_CAMERAS = (
    CameraConfig(name="wrist", backend="sim", fps=30.0),
    CameraConfig(name="scene", backend="sim", fps=30.0),
)


# --------------------------------------------------------------------------
# Roboter
# --------------------------------------------------------------------------

#: Globaler Geschwindigkeits-Override beim Verbinden (0.0 - 1.0).
#: Bewusst gedrosselt; fuer die finale Aufzeichnung anzupassen (AP 2.6).
DEFAULT_OVERRIDE = 0.2

#: Ungefaehre Reichweite der LARA 5 -- nur zur Plausibilitaetspruefung
#: gemeldeter Posen (Erkennung von Platzhalterwerten).
MAX_REACH_M = 0.9

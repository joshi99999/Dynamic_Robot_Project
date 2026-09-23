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

#: Senderate der servo_j-Sollwerte -- ganzzahliges Vielfaches von
#: CONTROL_RATE_HZ. Zwischen zwei 15-Hz-Zielen wird linear interpoliert
#: (bc/servo.py), in Aufzeichnung UND Inferenz identisch. Die Policy-Rate
#: bleibt 15 Hz; nur der Controller bekommt feinere Zwischenschritte.
#: Gemessen VM 2026-09-15 (J1-Fahrt 0.1 rad/s, Ist-Stellung mit ~50 Hz
#: gelesen): bei 15 Hz Stop-and-go innerhalb jedes Takts (Geschwindigkeit
#: je Update 0.004-0.6 rad/s, Streuung 60-100 % des Mittels), bei 60 Hz
#: 33 %, 120 Hz 27 %, 250 Hz 19 % (Rest ist Messrauschen der Zeitstempel).
#: servo_j dauert ~2.5 ms je Aufruf -- 60 Hz laesst Luft fuer Zustand und
#: Kameras im Beobachtungstakt. Neuras eigenes Beispiel sendet mit 1 kHz.
SERVO_RATE_HZ = 60.0

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
#:   1 -- Grundschema (AP 0.10)
#:   2 -- TCP-Quaternion kanonisch zu QUAT_HEMISPHERE_REF; neue Features
#:        ``next.done`` (Uebergabepunkt ans Hauptprogramm) und
#:        ``aux.joints_command`` (tatsaechlich per servo_j gesendete Winkel)
SCHEMA_VERSION = 2

#: Referenz-Orientierung fuer die Vorzeichenwahl der Quaternionen im
#: Datensatz (geometry.quat_canonical): Greifer zeigt senkrecht nach unten,
#: [QW, QX, QY, QZ] = 180 Grad um Y. Alle bisher geteachten Posen liegen
#: nahe daran (|<q, ref>| >= 0.91 an den VM-Punkten, 2026-09-14).
QUAT_HEMISPHERE_REF = (0.0, 0.0, 1.0, 0.0)

# --------------------------------------------------------------------------
# Kinematik-Absicherungen (AP 2.4)
# --------------------------------------------------------------------------

#: Sprung- und Geschwindigkeitsfilter fuer JEDEN servo_j-Sollwert
#: (servo.ServoGuard, in beiden Robot-Adaptern). Letzte Verteidigungslinie
#: gegen eine Policy oder einen Planer, der Unsinn kommandiert -- greift in
#: Aufzeichnung UND Inferenz, weil er im Adapter sitzt.
#: Hoechste zulaessige Gelenkgeschwindigkeit zwischen zwei Sollwerten.
#: Gemessen VM 2026-09-16: Idealbahn max 0.39 rad/s, gesendete verrauschte
#: Befehle max 0.60 rad/s (Rauschfaktor 1). 1.0 laesst Luft fuer
#: Policy-Korrekturen und haelt einen Sprung pro Aufruf trotzdem klein
#: (60 Hz: 0.017 rad). PROVISORISCH wie das Planer-Tempo (AP 2.6).
SERVO_MAX_JOINT_SPEED_RADS = 1.0

#: Weiche Stufe davor, nur in der Inferenz (servo.TargetLimiter): die
#: Aenderung des Policy-Ziels je 15-Hz-Takt wird auf diese Geschwindigkeit
#: BEGRENZT statt abgelehnt. Befund Durchstich 2026-09-17: die Labels selbst
#: verlangen Korrekturen bis ~1 rad/s (Aktion - Zustand p99 0.07 rad/Takt),
#: und beim Eintreffen eines verspaeteten Chunks springt das Mittel -- eine
#: Fahrt wurde bei 1.03 rad/s vom ServoGuard gestoppt. Unter
#: SERVO_MAX_JOINT_SPEED_RADS, damit der harte Filter nur Fehler faengt.
POLICY_MAX_JOINT_SPEED_RADS = 0.8

#: Groesster zulaessiger Abstand (rad, je Gelenk) zwischen einem neuen
#: Sollwert und der GEMESSENEN Stellung. Gemessen VM 2026-09-16: Action vs.
#: Ist-Stellung max 0.20 rad (Rauschen + Nachlauf). Groesser = Ziel liegt
#: nicht mehr "vor dem Arm", sondern woanders -- Abbruch.
SERVO_MAX_TARGET_GAP_RAD = 0.35

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

#: Hoechste Gelenkgeschwindigkeit in PTP-Segmenten (Interpolation im
#: Gelenkraum). Die Schrittzahl eines PTP-Segments richtet sich nach dem
#: strengeren von beiden Limits: diesem UND TRANSIT_SPEED_MS fuer den TCP --
#: so faehrt ein PTP-Segment nie schneller als ein LIN-Segment.
#: PROVISORISCH wie TRANSIT_SPEED_MS (AP 0.9 Punkt 5 / AP 2.6).
PTP_JOINT_SPEED_RADS = 0.5

#: Dauer der Beschleunigungs- bzw. Bremsrampe jedes Segments (Sinus-Profil,
#: Beschleunigung an beiden Enden 0). Jedes Segment startet und endet im
#: Stillstand -- Start, Greifer-Dwell und Wechsel LIN <-> PTP sind damit
#: keine Geschwindigkeitsspruenge mehr (Befund VM 2026-09-14: Controller
#: schwang dort bis 0.1 rad ueber). Spitzenbeschleunigung = v*pi/(2*T):
#: 0.47 m/s^2 im Transit, 1.6 rad/s^2 bei PTP_JOINT_SPEED_RADS. Segmente, die
#: zu kurz fuer volle Geschwindigkeit sind, bekommen dieselbe
#: Spitzenbeschleunigung und eine kuerzere Rampe.
#: Gehoert wie TRANSIT_SPEED_MS zum mitgelernten Tempo (AP 2.6): fuer alle
#: Episoden eines Datensatzes gleich lassen. PROVISORISCH.
SEGMENT_RAMP_S = 0.5

# --------------------------------------------------------------------------
# Ablauf zwischen den Episoden (AP 2.2)
# --------------------------------------------------------------------------

#: Groesste zulaessige Abweichung (rad, je Gelenk) zwischen Ist-Stellung und
#: geplanter Startstellung, bevor die Aufzeichnung startet. Darueber wird
#: zuerst per move_to_joints an den Start gefahren -- sonst waere der erste
#: servo_j-Sollwert ein Sprung.
START_POSE_TOL_RAD = 0.01

#: Parameter der blockierenden PTP-Fahrt (NeuraPy move_joint) fuer die
#: Rueckfahrt an den Start. Einheit laut Doku "% of maximum" bei einem
#: Default von 0.25 -- widerspruechlich. Diese Werte wurden am 2026-09-09
#: mit tools/check_sim_robot.py in der VM verifiziert (Ziel erreicht).
RESET_JOINT_SPEED = 25.0
RESET_JOINT_ACCELERATION = 20.0

#: Toleranz fuer die Plausibilisierung geteachter Punkte: die Pose aus der
#: Gelenkstellung (FK) muss die Cartesian-Darstellung der Punkte-Datenbank
#: reproduzieren. Weicht sie ab, sind Punkte vermutlich in einem anderen
#: Tool/Frame geteacht worden -- dann stimmen Plan und Programm nicht.
POINT_CROSSCHECK_TOL_POS_M = 2.0e-3
POINT_CROSSCHECK_TOL_ROT_RAD = 2.0e-2

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

#: Rauschstaerke JE EPISODE: Faktor gleichverteilt in diesem Bereich, mal
#: den Amplituden oben (apps/record.py, abschaltbar mit --noise-fixed).
#: Begruendung (Auswertung 2026-09-16, Berichte/2026-09-16_Rauschstudie.pdf):
#:   * VM-Laeufe mit festem Faktor 1: Ist-Zustand im Transit praktisch nie
#:     auf der Idealbahn (nur 33 % der Schritte < 5 mm, fast alle davon an
#:     Start/Greifpunkt) -- die Policy saehe den Zustand, in dem sie spaeter
#:     meist sein soll, kaum.
#:   * Stellvertreter-Studie (MLP-Policy, SimRobot, 5 Trainings-Seeds): ohne
#:     Rauschen 12-40 % Erfolg und Bahnabweichung p95 34 mm; fester Faktor 1
#:     macht die gelernten Befehle 5x unruhiger als die Idealbahn, kleines
#:     festes Rauschen korrigiert grosse Stoesse ruckartig. Die Mischung 0-1
#:     war ungestoert so ruhig wie kleines Rauschen und korrigierte 30-mm-
#:     Stoesse am weichsten.
NOISE_EPISODE_SCALE_RANGE = (0.0, 1.0)

#: Zeitkonstante der zwei Tiefpaesse hinter dem OU-Prozess
#: (noise.SmoothedOUProcess). Der reine OU-Prozess hat weisse
#: Geschwindigkeit -- fuer servo_j zu ruckig (VM 2026-09-14: bis 25 rad/s^2
#: in der Befehlsbahn). Bei 0.25 s und 15 Hz je Achse: 0.022 m/s und
#: 0.11 m/s^2 Streuung statt 0.09 m/s und 1.95 m/s^2; die Amplitude
#: (NOISE_TRANS_AMPLITUDE_M) bleibt unveraendert. 0 = ungeglaettet.
NOISE_SMOOTH_TAU_S = 0.25

#: Bahnlaenge zum naechsten bzw. vom letzten Ankerpunkt, ab der die
#: Trichter-Daempfung einsetzt. Ankerpunkte sind Episodenstart, jeder
#: Greiferwechsel und Episodenende (trajectory.py) -- dort ist das Rauschen
#: exakt 0 und steigt/faellt beidseitig weich an.
FUNNEL_START_DIST_M = 0.15

#: Bahnlaenge, unterhalb der das Rauschen vollstaendig auf 0 gedaempft ist.
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
    #: Weg, im Gegensatz zum ROI-Crop. ACHTUNG: die VEN-161-61U3C
    #: unterstuetzt KEIN Binning (Feature nicht schreibbar).
    binning: int = 1
    #: Nur "daheng": Belichtungszeit in Mikrosekunden. None = Automatik.
    exposure_us: float = None
    #: Nur "daheng": Gain in dB. None = Automatik.
    gain_db: float = None
    #: Nur "daheng": feste Weissabgleich-Ratios (Rot, Gruen, Blau).
    #: None = einmaliger Auto-Abgleich beim Oeffnen. Vor der echten
    #: Datenaufzeichnung messen und pinnen, sonst driftet die Farbstatistik
    #: zwischen Sessions (AP 0.6).
    white_balance_ratios: tuple = None
    #: Angeforderte Kamerarate. Bei "daheng" None = native Rate (frischere
    #: Frames, mehr CPU-Last durchs Debayering).
    fps: float = 30.0


#: Wrist-Kamera: Daheng VEN-161-61U3C (IMX296, 1/2.9", 1440x1080).
#: Objektiv: Fisheye 1.85 mm, Bildkreis fuer 1/1.8" -- der kleinere Sensor
#: nutzt also nur den zentralen Teil des Bildkreises.
#:
#: VOLLER SENSOR (width/height = None -> 1440x1080): Beim Fisheye ist das
#: Sichtfeld der Grund fuer die Objektivwahl -- ein ROI-Crop wuerde genau
#: das wegschneiden. Binning waere die elegante Datenreduktion, wird von
#: diesem Modell aber NICHT unterstuetzt (Feature nicht schreibbar), also
#: bleibt es beim vollen Frame: 1440x1080 Bayer8 @ 15 Hz ~ 23 MB/s, fuer
#: USB3 unkritisch. Skalierung auf die Schemagroesse 240x320 in Software,
#: Seitenverhaeltnis durchgehend 4:3.
#: fps=None -> native Rate (~61 fps): frischere Frames im Latenzbudget
#: (gemessen: 4-9 ms Alter), dafuer mehr CPU-Last durchs Debayering.
WRIST_CAMERA = CameraConfig(
    name="wrist",
    backend="daheng",
    device="EBK24100633",  # Seriennummer -- bei zwei Daheng zwingend
    width=None,
    height=None,
    exposure_us=8000.0,
    fps=None,
)

#: Szenen-/Top-View-Kamera -- MODELL NOCH NICHT ENTSCHIEDEN.
#:
#: ACHTUNG: Diese Konfiguration ist ein PLATZHALTER und zeigt auf
#: OpenCV-Index 0. Auf einem Laptop ist das die eingebaute Webcam --
#: damit aufgezeichnete Daten waeren wertlos, faellt aber im Datensatz
#: nicht auf. Vor der ersten Aufzeichnung durch die reale Kamera
#: ersetzen (Konsistenzpruefung: bc.config.check_cameras_configured()).
SCENE_CAMERA = CameraConfig(
    name="scene",
    backend="uvc",
    device=0,
    width=640,
    height=480,
    fps=30.0,
)
SCENE_CAMERA_CONFIRMED = False

#: Vorbereitete Alternative: zweite Daheng VEN-161 mit dem zweiten
#: Objektiv (rektilinear). Vorteile gegenueber einer UVC-Webcam:
#:   * gleiche Bildkette, gleiches Debayering, gleiche Steuerbarkeit
#:     (Belichtung/Gain/Weissabgleich fest pinnbar -- bei UVC oft nicht)
#:   * Global Shutter auch fuer die Top-View
#:   * beide Kameras sind hardware-triggerfaehig -> echte getriggerte
#:     Synchronisation statt zeitstempelbasiertem Sampling moeglich
#:     (siehe AP 1.1/1.3)
#: Seriennummer eintragen, sobald das Geraet vorliegt.
SCENE_CAMERA_DAHENG = CameraConfig(
    name="scene",
    backend="daheng",
    device=None,  # TODO: Seriennummer der zweiten Kamera
    width=None,
    height=None,
    exposure_us=8000.0,
    #: Bei zwei USB3-Kameras an einem Controller die Rate begrenzen, statt
    #: beide mit 61 fps frei laufen zu lassen (2 x 95 MB/s waeren knapp).
    #: 30 fps reichen fuer 15-Hz-Sampling mit Reserve im Latenzbudget.
    fps=30.0,
)

CAMERAS = (WRIST_CAMERA, SCENE_CAMERA)


def check_cameras_configured(cameras=CAMERAS):
    """Warnt vor Aufzeichnungen mit noch nicht bestaetigter Kamera.

    Rueckgabe: Liste von Warnungen (leer = alles bestaetigt).
    """
    warnings = []
    for cfg in cameras:
        if cfg.name == "scene" and cfg.backend != "sim" and not SCENE_CAMERA_CONFIRMED:
            warnings.append(
                "Szenenkamera ist noch ein Platzhalter (backend=%s, device=%s). "
                "Auf einem Laptop ist OpenCV-Index 0 die eingebaute Webcam!"
                % (cfg.backend, cfg.device)
            )
        if cfg.backend == "daheng" and not cfg.device:
            warnings.append(
                "Kamera '%s': keine Seriennummer konfiguriert -- bei mehreren "
                "Daheng-Geraeten ist die Zuordnung dann nicht eindeutig."
                % cfg.name
            )
    return warnings

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

# --------------------------------------------------------------------------
# Policy, Training und Inferenz (AP 3 / AP 4)
# --------------------------------------------------------------------------

#: Gepinnte LeRobot-Version (AP 0.9 Punkt 6). Der Export schreibt sie in den
#: Datensatz, Training und Inferenz pruefen dagegen. Das Datensatzformat
#: selbst (codebase_version "v3.0") legt lerobot fest.
LEROBOT_VERSION = "0.6.1"

#: Beobachtungen je Vorhersage (aktueller Takt + Vorgaenger). Der Wert
#: der Diffusion-Policy-Referenz; zwei Takte geben der Policy die
#: Bewegungsrichtung, ohne sie zur Zeitreihe zu machen.
POLICY_N_OBS_STEPS = 2

#: Laenge des vorhergesagten Aktions-Horizonts (AP 4.1: 8-16). Muss durch
#: 2 ** len(POLICY_DOWN_DIMS) teilbar sein (U-Net-Downsampling).
POLICY_HORIZON = 16

#: Davon nutzbare Schritte ab dem aktuellen Takt (horizon - n_obs + 1 ist
#: das Maximum). Aus diesen Schritten mittelt die Inferenz ueberlappende
#: Chunks (policy.ChunkEnsembler).
POLICY_N_ACTION_STEPS = 8

#: Nach wie vielen Takten neu vorhergesagt wird (AP 0.9 Punkt 7). Kleiner =
#: mehr ueberlappende Chunks zum Mitteln und schnellere Reaktion auf
#: Stoerungen, dafuer mehr GPU-Last. 1 <= Wert <= POLICY_N_ACTION_STEPS.
POLICY_REPLAN_STEPS = 2

#: Gewichtung beim Mitteln ueberlappender Chunks: w = exp(-k * Alter in
#: Takten). 0 = gleichgewichtet. k > 0 bevorzugt NEUERE Vorhersagen
#: (reaktiver), k < 0 aeltere (ruhiger, ACT-Stil).
POLICY_ENSEMBLE_DECAY = 0.0

#: U-Net-Kanaele. (256, 512, 1024) ist die Groesse der Diffusion-Policy-
#: Referenz fuer Realroboter (~65 M Parameter) statt lerobots Default
#: (512, 1024, 2048, ~260 M) -- passt auf 16 GB und bleibt auf den
#: Inferenz-Laptops (Hardware noch unbekannt, AP 3.1) rechenbar.
POLICY_DOWN_DIMS = (256, 512, 1024)

#: Bildausschnitt fuer die Augmentierung: zufaelliger Crop mit diesem
#: Anteil im Training, zentrierter Crop bei der Inferenz (lerobot).
POLICY_CROP_RATIO = 0.9

#: Denoising-Schritte bei der Inferenz (DDIM). Training laeuft mit 100.
#: Vorlaeufig; auf der Zielhardware messen (AP 4.1 Punkt a).
POLICY_INFERENCE_STEPS = 10

#: Was das Netz vorhersagt: "epsilon" (Rauschen, lerobot-Default) oder
#: "sample" (direkt die Aktionsfolge). Befund Durchstich 2026-09-17 (Sim,
#: 50 Episoden, 8000 Schritte, epsilon): mit 10 DDIM-Schritten gezackte
#: Chunks -- groesster Sprung im Chunk 0.10-0.12 rad statt 0.015 wie im
#: Label, Streuung zwischen Samples 0.05-0.07 rad; erst mit 100 Schritten
#: glatt (0.019), dann aber 600 ms je Vorhersage. Die Policy loeste so schon
#: im ersten Takt den ServoGuard aus.
POLICY_PREDICTION_TYPE = "sample"

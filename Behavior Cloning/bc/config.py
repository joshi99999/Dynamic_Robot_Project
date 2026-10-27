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

#: Ausfuehrungsart der Sollwerte (bc/execution.py), in Aufzeichnung UND
#: Inferenz identisch. Begruendung und Messungen:
#: Dokumentation/Zittern-an-der-Anlage.md.
#:   servo_j  -- 60 Hz linear interpoliert (bisher). An der Anlage zittert
#:               der Arm: die Steuerung erwartet einen 1-kHz-Strom und faehrt
#:               die 60-Hz-Stufen als Treppe (2026-10-13: 1,80 m/s^2 am
#:               Gelenk 5, Eigenschwingung 7-8 Hz).
#:   servo_j_pos -- wie servo_j, aber v = a = 0: die Steuerung glaettet
#:               selbst (VM 2026-10-14: Welligkeit quer 0,09 statt 0,46 mm,
#:               ~70 ms Verzug, tools/check_servo_stream.py). An der Anlage
#:               UNGEPRUEFT -- nur ausdruecklich waehlbar, nie per auto.
#:   online   -- ``movelinear_online``: ein kartesisches Ziel je 15-Hz-Takt,
#:               die Steuerung interpoliert selbst (0,43 m/s^2, -76 %).
#:   auto     -- online an der realen Anlage, sonst servo_j: die VM
#:               (v5.0.0-alpha.102) nimmt movelinear_online an, faehrt aber
#:               nicht (2026-10-12).
#:
#: Entscheidung 2026-10-19 (Anlage, drei Serien, Berichte/2026-10-19_*):
#: ``servo_j`` mit Override 0,5 (DEFAULT_OVERRIDE). Am Arm so ruhig wie
#: online (H1 0,19-0,20 m/s^2, Neura-Standard 0,16-0,18), aber bahntreu
#: (quer max 1,2 mm statt 27-49 mm), PICK exakt, kein 15-Hz-Pulsieren, und
#: es laeuft auch in der VM. Das Zittern von servo_j kam vom Override 1,0
#: (1,6-1,7 m/s^2), nicht von servo_j selbst. online bleibt waehlbar.
EXECUTION_MODE = "servo_j"

#: Startwerte fuer ``online`` (Anlage 2026-10-13, Serie mit Handy-Messung,
#: Freigabe Anwender 13.10.): die ruhigste gemessene Einstellung. Vorlauf in
#: 15-Hz-Takten; Tempo/Beschleunigung als Vielfaches der Planer-Spitzen
#: (bc.execution.online_limits). Groessere Faktoren fahren bahntreuer, aber
#: hoerbar im 15-Hz-Takt. PROVISORISCH bis zur Bestaetigung an echten
#: Aufnahmebahnen.
ONLINE_LEAD_STEPS = 2
ONLINE_SPEED_FACTOR = 1.3
ONLINE_ACCEL_FACTOR = 1.5
ONLINE_VERIFIED = False

#: Wie lange die Aufzeichnung am Bahnende hoechstens wartet, bis der Arm
#: das letzte Ziel erreicht hat, bevor die Online-Fahrt beendet und das
#: Servo-Interface abgebaut wird (Abbau bei Restbewegung steht im Verdacht,
#: den Not-Halt am Bahnende ausgeloest zu haben, Taktzeit-Doku 6.5).
ONLINE_SETTLE_S = 1.0

#: "Erst weiter, wenn erreicht" an Greiferwechseln (nur ``online``,
#: bc/execution.py). Anlage 2026-10-16: der Arm laeuft ~1,3 s hinter dem
#: Ziel her; mit dem 0,5-s-Dwell kehrte er 27 mm ueber PICK um, ohne zu
#: halten. Jetzt bleibt das Ziel am Haltepunkt stehen, bis der TCP naeher
#: als ONLINE_REACH_TOL_MM am Ziel ist (ONLINE_REACH_SETTLE_STEPS Takte in
#: Folge); erst dann Greiferbefehl und der normale GRIPPER_DWELL. Nach
#: ONLINE_REACH_TIMEOUT_S ohne Erfolg: Abbruch (Aufnahme verworfen,
#: Inferenzfahrt beendet). Aufnahme und Inferenz nutzen dieselben Werte --
#: sie stehen in den Metadaten (execution/reach_*). PROVISORISCH bis zur
#: Messung an der Anlage (tools/check_servo_stream.py, Lauf onlinehalt-15-r0-o1).
ONLINE_REACH_TOL_MM = 1.0
ONLINE_REACH_SETTLE_STEPS = 2
ONLINE_REACH_TIMEOUT_S = 3.0

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

#: Ablegen beim Zuruecklegen (bc/put_back.py, Anwender 2026-10-26): Das
#: Objekt muss so liegen bleiben, wie es gegriffen wurde -- nicht mit dem
#: Objekt in den Tisch druecken, nicht zu hoch loslassen. Abgelegt wird
#: deshalb exakt auf der Greifpose (PICK), der letzte Abstieg langsamer als
#: der Endanflug der Aufnahme (wird nicht aufgezeichnet, Tempo frei) ...
PUT_BACK_APPROACH_SPEED_MS = 0.02
#: ... und nach dem Oeffnen einmal "zu, auf" (Nachgreifen): der
#: Parallelgreifer schiebt das Objekt dabei mittig zwischen die Backen --
#: die Lage ist danach durch den Greifer bestimmt, nicht durch ein Rutschen
#: beim Loslassen.
PUT_BACK_REGRIP = True
#: Wann nachgegriffen wird (``record.py --nachgreifen``, Anwender 2026-10-27):
#: "lagenwechsel" = nur beim Ablegen an einer NEUEN Lage (automatische Lagen,
#: Rueckkehr an den geteachten PICK) -- bei Wiederholungen derselben Lage
#: liegt das Objekt schon zentriert; "immer"; "nie".
PUT_BACK_REGRIP_MODES = ("lagenwechsel", "immer", "nie")
PUT_BACK_REGRIP_DEFAULT = "lagenwechsel"

#: Grobe Dauer eines Laufs (Anzeige in der Oberflaeche, gui/recording.
#: estimate_duration_s). Gemessen Anlage 2026-10-26: eine Fahrt samt
#: Zuruecklegen und Nebenzeiten ~25 s (Episode 8,5 s, Abstand 23-24 s),
#: Planung je Lage ~12 s (3 Fahrten + Rueckfahrt), Probefahrt ~50 s, Start
#: (Verbinden, Kameras, Referenzbilder) ~20 s, Umlegen an eine neue Lage
#: einige s mehr (Nachgreifen, laengerer Weg). Wartezeit fuer Bewertungen
#: ist NICHT enthalten.
ESTIMATE_CYCLE_S = 25.0
ESTIMATE_PLAN_LAGE_S = 12.0
ESTIMATE_PROBE_S = 50.0
ESTIMATE_START_S = 20.0
ESTIMATE_TRANSFER_S = 5.0
#: record.py nach der Planung: Nebenzeit je Fahrt zusaetzlich zu den
#: geplanten Bahnen (Servo an/aus, Anfahren an den Start, Ablegen der Datei)
ESTIMATE_OVERHEAD_S = 4.0

#: Kodierung des binaeren Greifers in State- und Action-Vektor (AP 0.10).
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.0
#: Schwelle bei der Inferenz: Werte darueber gelten als "geschlossen".
GRIPPER_THRESHOLD = 0.5

#: Greifer an der realen Anlage ueber Digitalausgaenge der Control-Box
#: schalten (NeuraPy ``io("set", io_name=..., target_value=...)``) statt
#: ueber grasp()/release(). Namen wie in der Steuerung ("DO_0" ... "DO_7",
#: ab 0 gezaehlt -- Sicherung caboutputdigitals.json); ``None`` = aus.
#: Ueber den Namen statt set_digital_output(int), damit keine Frage nach
#: 0- oder 1-basierter Nummer offen bleibt.
#:
#: Grund (Anlage 2026-10-23): Pinbelegung steht, der Zimmer-Greifer laesst
#: sich ueber die Ausgaenge schalten und ist als Tool "ZimmerLWR50"
#: hinterlegt -- am Pendant aber nicht als Greifer spezifizierbar.
#: grasp()/release() laufen auf der Steuerung und brauchen dort den
#: Greifereintrag; ohne ihn werfen sie "'Robot' object has no attribute
#: 'gripper_name'" (VM 2026-09-09, Anlage 2026-10-23, erster Greiferbefehl
#: der Aufnahme).
#:
#: Belegung Zimmer LWR50 (Anlage 2026-10-23): DO_0 auf, DO_1 zu, DO_2
#: Reset, DO_3 Enable (muss dauerhaft 1 sein). Vor dem ersten Befehl je
#: Verbindung geht Enable an; Reset wird nie geschaltet.
#:
#: Schliessen: AUF-Pin aus, dann ZU-Pin an; Oeffnen umgekehrt. Nie beide
#: gleichzeitig an. Gilt NUR an der realen Anlage: In der Simulation bleibt
#: es beim Protokollieren, und --no-gripper (Trockenlauf) geht vor. Vor der
#: ersten Aufnahme tools/check_gripper.py --real-robot laufen lassen.
NEURA_GRIPPER_DO_OPEN = "DO_0"
NEURA_GRIPPER_DO_CLOSE = "DO_1"
NEURA_GRIPPER_DO_ENABLE = "DO_3"

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

#: Hoechste Gelenkgeschwindigkeit, die der PLANER einer Bahn erlaubt
#: (ideal und verrauscht, ``noise.joint_speed``) -- mit Abstand unter
#: SERVO_MAX_JOINT_SPEED_RADS, weil der Takt um einige ms schwankt und der
#: ServoGuard mit der gemessenen Zeit rechnet. Anlage 2026-10-26: eine
#: verrauschte Bahn mit 1,47 rad/s an Gelenk 1 kam durch die Planung (dort
#: galt nur IK_MAX_DELTA_Q_RAD, 0,2 rad je 15-Hz-Schritt = 3 rad/s) und wurde
#: erst beim Fahren vom ServoGuard gestoppt -- Episode verloren, Lauf zu Ende.
#: Jetzt wird so ein Rauschsample verworfen und neu gezogen.
PLAN_MAX_JOINT_SPEED_RADS = 0.85

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
#:
#: Geprueft 2026-10-19 (Anlage, servo_j Override 0,5, 6 Recorder-Episoden
#: mit Rauschen 0-1, data_vm/2026-10-19): Ziel-Ist max 0,10 rad -- 0,35
#: laesst gut dreifache Reserve. Eine kurzzeitige Lockerung auf 0,60 (nach
#: einer Mitschrieb-Rechnung, die den Nachlauf ueberschaetzte) wurde
#: zurueckgenommen: strenger schuetzt in der Inferenz vor Fehlvorhersagen.
#: Ursache der Ueberschaetzung (PC 2026-10-19): die Rechnung nahm die Uhr
#: statt der Sendezeit, jeder ~210-ms-Haenger verschob so den Rest der
#: Bahn. Gegen die Sendezeit alle servo_j-Laeufe vom 16./19.10. 0,03-0,15
#: rad (tools/report_labortag_2026_10_16.py --mitschriebe).
SERVO_MAX_TARGET_GAP_RAD = 0.35

#: Schleppfehler-Abbruch des Recorders (AP 2.4/4.2): groesster zulaessiger
#: Abstand (rad, je Gelenk) zwischen dem Sollwert eines Takts und der dort
#: GEMESSENEN Stellung, und wie viele Takte in Folge er anliegen darf.
#:
#: Befund 2026-10-01 (Labortag): Die Steuerung kappte waehrend der Fahrt
#: die PC-Steuerung (RCSC_102). ``servo_j`` meldete weiter einen Code < 3,
#: der Recorder zeichnete 218 von 218 Takten auf -- und der Arm hatte sich
#: in einer Episode KEINEN EINZIGEN Gelenkwinkel bewegt. Die Episode kam
#: als "vollstaendig, verworfen wegen Latenzbudget" auf die Platte, also
#: mit einer Begruendung, die auf die Kameras zeigt. Genau das verhindert
#: diese Pruefung: der Arm folgt oder die Episode bricht ab.
#:
#: Werte: gute VM-Laeufe lagen bei einem Schleppfehler von max 0,12 rad
#: (ep_00000/ep_00003), der tote Kanal bei konstant 1,1 rad ab Takt 6.
#: 0,30 rad liegt deutlich ueber dem Nachlauf und weit unter dem Ausfall.
#: Vier Takte in Folge, damit ein einzelner Ausreisser nichts abbricht.
#: VORLAEUFIG -- an der Anlage ist der echte Nachlauf neu zu messen
#: (schnellere Fahrt = groesserer Nachlauf), siehe
#: Dokumentation/Taktzeit-und-RPC-Latenz.md.
#:
#: Anlage 2026-10-19 (servo_j Override 0,5, 6 Recorder-Episoden mit
#: Rauschen 0-1): Schleppfehler max 0,10 rad -- 0,30 bleibt mit Reserve.
RECORDER_MAX_FOLLOW_ERROR_RAD = 0.30
RECORDER_FOLLOW_ERROR_STEPS = 4

#: Zweites, MASSSTABSFREIES Kriterium fuer dieselbe Pruefung: Anteil der
#: kommandierten Bewegung, den der Arm ueber das Fenster mindestens
#: zuruecklegen muss.
#:
#: Noetig, weil die absolute Schwelle oben an kurzen oder langsamen Bahnen
#: nicht greift: bewegt sich die ganze Bahn nur um 0,12 rad, erreicht auch
#: ein voellig stehender Arm nie 0,30 rad Schleppfehler. Dieses Kriterium
#: fragt stattdessen "der Sollwert ist gewandert -- ist der Arm mit?" und
#: ist damit unabhaengig von Bahnlaenge und Geschwindigkeit. Waehrend des
#: Greifer-Dwells wandert der Sollwert nicht, es loest dort also nicht aus.
RECORDER_MIN_FOLLOW_RATIO = 0.10

#: Wie lange Kriterium (b) ununterbrochen anliegen muss, bevor es abbricht.
#:
#: Anlass 2026-10-16/19: Aus den Mitschrieben von tools/check_servo_stream.py
#: schien der Arm nach Stillstaenden 0,3-1,8 s stehen zu bleiben, waehrend
#: die Sollwerte weiterliefen, und (b) haette dort angeschlagen. Am PC
#: 2026-10-19 als Rechenfehler erkannt: die Nachrechnung nahm die Uhr statt
#: der Sendezeit, und jeder Haenger eines servo_j-Aufrufs (~210 ms, einmal
#: 1014 ms) verschob den Rest der Bahn. Gegen die Sendezeit schlaegt (b) in
#: keinem servo_j-Lauf an, auch die 6 Recorder-Episoden vom 19.10. nicht.
#: Echt ist nur der Haenger selbst: so lange kommt kein Sollwert, der Arm
#: steht -- und der Recorder misst erst nach dem Aufruf, sieht ihn also nicht.
#:
#: Der Wert bleibt vorerst als Absicherung (Entscheidung Anwender
#: 2026-10-19), ist aber durch keinen beobachteten Fehlalarm begruendet. Der
#: tote Steuerkanal (Arm bewegt sich gar nicht) wird weiterhin erkannt, nur
#: so viel spaeter -- der Arm steht in dem Fall ohnehin. Kriterium (a)
#: bleibt davon unberuehrt.
RECORDER_STALL_GRACE_S = 2.5

#: Ab welchem Verhaeltnis Ist- zu Soll-Dauer der Recorder nach einer
#: Episode warnt (bc.recorder.timing_findings). Gesunde Laeufe lagen am
#: 2026-10-02 bei 1,01-1,04; die zu schnellen Senderaten (90/120 Hz) bei
#: 1,62-1,70 -- ohne dass sonst etwas auffiel.
RECORDER_SLOW_FACTOR_WARN = 1.10

#: Auslegungsannahme fuer einen servo_j-Aufruf (gemessen am PC, VM
#: 2026-09-15, siehe SERVO_RATE_HZ) und die Schwelle, ab der ein Hinweis
#: auf einen ausgelasteten Rechner erscheint (Vierfaches). Auf dem Laptop
#: im Akkubetrieb lag servo_j bei 22,4 ms (2026-10-01).
SERVO_J_DESIGN_MS = 2.5
SERVO_J_HOST_HINT_MS = 4 * SERVO_J_DESIGN_MS

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

#: Zulaessiger Bereich fuer das je Lauf einstellbare Planer-Tempo
#: (``record.py --transit-speed/--approach-speed/--ptp-speed``, GUI).
#: Nach unten: darunter wird eine Episode unpraktisch lang. Nach oben: die
#: verrauschten Befehle liegen ueber der Idealbahn (VM 2026-09-16: ideal
#: 0.39, verrauscht 0.60 rad/s bei PTP 0.5) und muessen unter
#: SERVO_MAX_JOINT_SPEED_RADS bleiben, die Policy spaeter unter
#: POLICY_MAX_JOINT_SPEED_RADS -- deshalb PTP hoechstens 0.6 rad/s.
PLANNER_TRANSIT_SPEED_RANGE_MS = (0.02, 0.25)
PLANNER_APPROACH_SPEED_RANGE_MS = (0.01, 0.10)
PLANNER_PTP_SPEED_RANGE_RADS = (0.05, 0.6)

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

#: Plausibilitaet des Endanflugs (``bc/sequence.approach_problems``), vor
#: jeder Bewegung geprueft. Anlage 2026-10-23: PICK und PRE_GRASP vertauscht
#: geteacht (PRE_GRASP 49 mm UNTER PICK) -- der Arm kam von unten und
#: stiess an; das Kollisionsmodell (Tisch 15 cm unter dem tiefsten Punkt)
#: sah nichts. Der Punkt vor einem ``approach``-Punkt muss mindestens so
#: weit darueber liegen ...
APPROACH_MIN_DROP_M = 0.010
#: ... und darf seitlich hoechstens so weit versetzt sein (PRE_GRASP wird
#: senkrecht ueber PICK geteacht, "Z hoch" laeuft 1-3 mm nach).
APPROACH_MAX_LATERAL_M = 0.015

#: Tischreferenz (bc/table.py, Anwender 2026-10-26): Punkte werden so weit
#: UEBER dem Tisch geteacht (mittig ueber den ArUco-Markern) ...
TABLE_POINT_OFFSET_M = 0.001
#: ... kein Wegpunkt und keine Werkzeugspitze darf naeher an die Ebene
#: (PICK liegt ueblich 6-10 mm darueber) ...
TABLE_MIN_CLEARANCE_M = 0.003
#: ... die Punkte duerfen hoechstens so weit von einer Ebene abweichen
#: (sonst ist einer falsch geteacht) ...
TABLE_MAX_RESIDUAL_M = 0.003
#: Sperrzone um einen geteachten Punkt (``"table": {"keepout": [...]}``,
#: Anwender 2026-10-26: Schraube im Tisch an Punkt IGNORE): Quadrat mit
#: dieser halben Kantenlaenge um den Punkt (+-5 cm) ...
KEEPOUT_HALF_SIZE_M = 0.05
#: ... darin muss die Werkzeugspitze mindestens so hoch ueber dem Tisch
#: bleiben -- kein Greifen dort, und keine verrauschte Bahn taucht hinein.
KEEPOUT_MIN_HEIGHT_M = 0.05
#: ... "Anfahren" eines Tischpunkts kommt von so weit senkrecht darueber.
TABLE_APPROACH_ABOVE_M = 0.05
#: Greifhoehe ueber dem Tisch fuer "PICK ausrichten", solange die
#: Ablaufdatei keine eigene nennt (``"table": {"pick_height": ...}``).
PICK_HEIGHT_DEFAULT_M = 0.008

#: Automatische Lagen (bc/auto_layout.py, Anwender 2026-10-26): gezogen wird
#: so weit innerhalb der Tischpunkte (Objekt + Backen) ...
AUTO_EDGE_MARGIN_M = 0.04
#: ... mit so viel Abstand zu allen Lagen davor ...
AUTO_MIN_DIST_M = 0.03
#: ... gedreht um bis zu so viel gegen die geteachte Lage (Grad) ...
AUTO_YAW_RANGE_DEG = 45.0
#: ... und hoechstens so oft gezogen (Bereich) bzw. geplant (Erreichbarkeit,
#: Tempo, Kollision), bevor der Lauf aufhoert.
AUTO_SAMPLE_TRIES = 500
AUTO_PLAN_TRIES = 8

#: Berechnete Punkte (``"above"``/``"dz"`` in der Ablaufdatei, Festlegung
#: Anwender 2026-10-26: je Lage nur PICK teachen): die IK fuer den Punkt
#: ueber dem Bezugspunkt darf sich je Gelenk hoechstens so weit von dessen
#: Gelenken entfernen. 10 cm senkrecht sind an der LARA 5 wenige Zehntel
#: rad; ein groesserer Sprung heisst anderer Gelenkzweig.
COMPUTED_POINT_MAX_JUMP_RAD = 0.8

#: Probefahrt nach dem Teachen (``record.py``, Wunsch Anwender 2026-10-23):
#: die ideale Bahn ohne Rauschen, Greifer bleibt offen, nicht aufgezeichnet
#: -- so viel langsamer als geplant (0.4 = 40 % Tempo). Bewusst NICHT ueber
#: den Override: der bremst servo_j zeitlich (VM 0.2: Nachlauf bis 2,3 s,
#: dann greift "arm folgt nicht"), die Bahn saehe anders aus als bei der
#: Aufnahme. Hier wird dieselbe Bahn nur zeitlich gestreckt.
PROBE_SPEED_FACTOR = 0.4

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

#: Touch-up aus der Bedienoberflaeche (bc/touchup.py, Wunsch Anwender
#: 2026-10-21). Ein Punkt speichert KEIN Tool -- die Pose rechnet die
#: Steuerung mit dem gerade aktiven. An der realen Anlage muss deshalb das
#: Tool der Aufnahme gewaehlt sein (Anlage 2026-10-16/19: ZimmerLWR50).
#: In der Simulation wird der Name nicht verlangt (VM: "NoTool").
#: PROVISORISCH -- Tool-Daten am Pendant noch zu pruefen (Uebergabe Abschnitt 7.6).
TOUCHUP_TOOL_NAME = "ZimmerLWR50"

#: Sicherungen der Punkte vor jedem Touch-up (gitignored wie die
#: Arbeitskopien: sie gehoeren zum Arbeitsplatz, nicht ins Repository).
TOUCHUP_BACKUP_ROOT = "sequences/arbeitskopien/punkt_sicherungen"

#: Der Arm muss beim Touch-up stillstehen: zwei Lesungen im Abstand
#: TOUCHUP_STILL_WAIT_S duerfen sich je Gelenk hoechstens so weit
#: unterscheiden. Sonst wuerde eine Zwischenstellung gespeichert.
TOUCHUP_STILL_TOL_RAD = 1.0e-3
TOUCHUP_STILL_WAIT_S = 0.2

#: Ab welcher Verschiebung die Rueckfrage ausdruecklich warnt -- ein
#: Touch-up korrigiert Millimeter, eine grosse Strecke deutet auf die
#: falsche Zeile oder einen anderen Punkt.
TOUCHUP_WARN_MM = 50.0
TOUCHUP_WARN_DEG = 15.0

#: Teach-Serie (bc/teach_series.py, Wunsch Anwender 2026-10-21): der
#: POINT-Knopf am Arm legt -- am Pendant auf "Save Point" gestellt -- einen
#: Punkt an; die Oberflaeche uebernimmt dessen Gelenke per Touch-up in die
#: markierte Zeile, loescht ihn und springt weiter. So oft wird nach neuen
#: Punkten gesehen (ms):
TEACH_SERIES_POLL_MS = 300
#: Nur Punkte, deren Name auf dieses Muster passt (re.fullmatch), gelten
#: als Knopfdruck -- None = jeder neue Name ausser Hilfspunkten ("_...")
#: und Namen aus der Ablauftabelle. Anlage 2026-10-21: POINT ("Save Point")
#: legte "GP35" an -- die Steuerung zaehlt "GP<n>" hoch. Ein am Pendant
#: von Hand angelegter Punkt mit anderem Namen wird so nie verbraucht.
TEACH_SERIES_POINT_PATTERN = r"GP\d+"

#: Tippbetrieb "Z hoch" in der Bedienoberflaeche (ports.JogPort, Wunsch
#: Anwender 2026-10-21: PRE_GRASP senkrecht ueber PICK teachen). Die
#: Steuerung faehrt je ``jog``-Aufruf ein Stueck; das Tempo haengt an
#: Anteil, Aufrufrate UND Override. Gemessen VM 2026-10-21, Override 0,5:
#: Anteil 0,3 bei 50 Hz = 14 mm/s (Override 1: 27 mm/s), Auslauf nach dem
#: letzten Aufruf 1-3 mm, Bewegung rein in z (Basis). An der Anlage
#: begrenzt der Teach-Modus zusaetzlich.
JOG_Z_FRACTION = 0.3
JOG_TICK_HZ = 50.0
#: Laengstes Tippen am Stueck, dann haelt die Oberflaeche von selbst an
#: (Absicherung, falls ein Loslassen verloren geht).
JOG_MAX_S = 10.0

#: Soll ``NeuraRobot.read_state()`` die TCP-Pose LOKAL aus der URDF-Kette
#: rechnen statt per ``compute_forward_kinematics``?
#:
#: Grund (Befund 2026-10-01, Labortag): read_state() kostete zwei
#: RPC-Roundtrips. Der zweite -- die FK -- liegt ZWISCHEN dem Zeitstempel
#: der Gelenkmessung und dem Abgriff der Kamerabilder und erzeugt damit
#: genau den Versatz, den bc/sync.py als "Frame ueber dem Latenzbudget"
#: meldet. Lokal gerechnet entfaellt beides: ein Aufruf weniger je Takt,
#: und Roboter- und Bildzeitstempel liegen unmittelbar nebeneinander.
#:
#: SICHERHEIT: Der Adapter prueft beim Verbinden, ob die lokale Kette die
#: Steuerung reproduziert (NeuraRobot._check_local_fk). Weicht sie ab,
#: wird NICHT lokal gerechnet -- die Steuerung bleibt massgeblich. Die
#: Uebereinstimmung ist bisher nur gegen die virtuelle Steuerung belegt
#: (0,00 mm ueber 50 Stuetzstellen, Sim-Inbetriebnahme-Befunde.md
#: Abschnitt 5); an der Anlage entscheidet der Vortest.
NEURA_LOCAL_FK = True

#: Toleranz dieses Vortests. Enger als POINT_CROSSCHECK_*, weil es hier
#: nicht um geteachte Punkte geht, sondern um dieselbe Rechnung auf beiden
#: Seiten -- eine echte Abweichung ist ein Modellfehler, kein Teach-Fehler.
LOCAL_FK_TOL_POS_M = 1.0e-4
LOCAL_FK_TOL_ROT_RAD = 1.0e-3

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
#: 60 fps (Anlage 2026-10-26: beide Daheng direkt am Laptop schaffen 60,
#: am USB-Hub nur 23). Das neueste Bild ist dann hoechstens ~17 ms alt --
#: Reserve im 30-ms-Latenzbudget; 30 fps lagen mit bis 33 ms schon darueber.
#: Fest statt nativ (61,3), damit beide Kameras dieselbe Rate haben.
WRIST_CAMERA = CameraConfig(
    name="wrist",
    backend="daheng",
    device="EBK24100633",  # Seriennummer -- bei zwei Daheng zwingend
    width=None,
    height=None,
    exposure_us=8000.0,
    fps=60.0,
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
    #: 60 fps wie die Wrist-Kamera (Anlage 2026-10-26, beide direkt am
    #: Laptop). 30 fps reichten NICHT: Bild bis 33 ms alt, ueber dem Budget.
    #: Zusammen ~190 MB/s -- nur an getrennten USB-3-Ports, nicht am Hub.
    fps=60.0,
)

CAMERAS = (WRIST_CAMERA, SCENE_CAMERA)

#: NUR ZUM TESTEN: beliebige USB-Webcam als Ersatz fuer die Wrist-Kamera
#: (Kameramodus "wrist-uvc", AP 1.1/1.3). Prueft die Kamera-KETTE mit
#: echtem Geraet -- Rate, Frame-Alter, Zeitstempel, Sync-Budget, Last im
#: Recorder -- bevor die Daheng am Labortag zur Verfuegung steht. Nicht die
#: Daheng-Einstellungen (Belichtung, Weissabgleich, Binning). Index 0 ist auf
#: einem Laptop meist die eingebaute Kamera; USB-Webcam oft Index 1
#: (``tools/check_cameras.py --list``, Aufruf mit ``--uvc-device``).
#: Aufnahmen damit sind Testdaten: record.py verweigert den Modus an der
#: realen Anlage, der Export bricht bei gemischten Backends ab.
WRIST_CAMERA_UVC_STANDIN = CameraConfig(
    name="wrist",
    backend="uvc",
    device=0,
    width=640,
    height=480,
    fps=30.0,
)


def check_cameras_configured(cameras=CAMERAS):
    """Warnt vor Aufzeichnungen mit noch nicht bestaetigter Kamera.

    Unbestaetigt ist nur der UVC-Platzhalter (OpenCV-Index); eine Daheng
    mit Seriennummer ist eine bewusste Zuordnung -- gleiche Regel wie in
    der Bedienoberflaeche (bc/gui/cameras.py).

    Rueckgabe: Liste von Warnungen (leer = alles bestaetigt).
    """
    warnings = []
    for cfg in cameras:
        if cfg.name == "scene" and cfg.backend == "uvc" and not SCENE_CAMERA_CONFIRMED:
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
#:
#: 0,5 seit 2026-10-19 (Anlage, mit EXECUTION_MODE = "servo_j"): der
#: Override deckelt das Nachfuehren der Steuerung, nicht die Bahnzeit
#: (Fahrt 11,6 s wie bei 1,0). Bei 1,0 faehrt der Arm die 60-Hz-Stufen als
#: Treppe (H1 1,6-1,7 m/s^2), bei 0,5 glatt (0,19-0,20) mit konstant
#: ~0,09 s Nachlauf (1,0: ~0,02 s; gegen die Sendezeit, PC 2026-10-19 --
#: die frueher genannten 0,15-0,43 s enthielten Haenger der Aufrufe) und
#: Bahn auf ~1 mm. Das Tempo begrenzt weiterhin der Planer
#: (TRANSIT_SPEED_MS, PTP), nicht der Override. Bisher 0,2: in der VM
#: ~1 s Nachlauf (2026-09-14) -- fuer Messungen in der VM ggf. ausdruecklich
#: setzen. Die Inferenz uebernimmt den Override der Aufzeichnung.
DEFAULT_OVERRIDE = 0.5

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

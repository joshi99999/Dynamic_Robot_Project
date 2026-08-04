# Systemarchitektur & Requirements Engineering – KI-basierte Robotersteuerung (LeRobot & Diffusion Policy)

Dieses Dokument dient der präzisen Erfassung aller technischen, infrastrukturellen und betrieblichen Anforderungen für die Arbeitspakete (AP 1 bis AP 5). Das System wird als vollständig **lokale, autarke Lösung** konzipiert, um Unabhängigkeit von Cloud-Infrastrukturen großer Tech-Unternehmen zu gewährleisten.

---

## 📋 Status: Hardware-Aufbau und Inbetriebnahme

### Bereits geklärt
* **Robotermodell:** Bestätigt — Neura Robotics **LARA 5** (ausgelieferte Software LARA-5.0.8; Datenblätter und Python-API unter `10_Neura`).
* **Greifer-Modell:** Bestätigt — Zimmer **LWR50L-03** Backengreifer (HRC match-Ecosystem; Datenblatt unter `20_Dokumentation`).
* **Greifer-Ansteuerung:** Geklärt — die NeuraPy-API stellt nur **binäre** Greiferbefehle bereit (`gripper('on'/'off')`, `grasp()`/`release()`); Öffnungsweite/Kraft/Geschwindigkeit sind lediglich als vorkonfigurierte Tool-Parameter hinterlegbar. Eine kontinuierliche Positionsregelung oder ein Auslesen der Ist-Greiferweite ist über NeuraPy **nicht** möglich (siehe AP 2.1).
* **Software-Not-Aus / Sicherheits-Schnittstelle:** Geklärt — über NeuraPy verfügbar (`stop`, `pause`, `power_off`) sowie ein separates Terminate-Skript. Dies ersetzt **keinen** zertifizierten Hardware-Not-Aus (siehe AP 4.2).
* **Tiefendaten (Kameras):** Geklärt — es liegen **keine** Tiefendaten vor; die Policy arbeitet ausschließlich auf RGB-Bildern (siehe AP 1.1).
* **Wrist-Kamera:** Bestätigt — Daheng Imaging **VEN-161-61U3C** (Farbvariante, Sony IMX296 Global Shutter, 1440 × 1080 @ 61,2 fps, USB3.0). Datenblatt: `../20_Dokumentation/WristKamera-VEN-161-61U3MC-Datasheet.pdf`.
* **Kamera-SDK:** Geklärt — die VEN-161-61U3C ist eine **USB3-Vision-/GenICam**-Kamera und **nicht** über `cv2.VideoCapture` ansprechbar. Der Zugriff erfolgt über das **Daheng Galaxy SDK mit der Python-API `gxipy`** (siehe AP 1.1). Intel RealSense bzw. `pyrealsense2` wird im Projekt **nicht** verwendet.
* **Aufzeichnungsrate:** Festgelegt — **15 Hz** als gemeinsame Zielrate für alle Datenquellen (siehe AP 1.3).
* **Trainings-Hardware:** Bestätigt — **NVIDIA RTX 5070 Ti (16 GB VRAM)** für das Training; zusätzlich stehen Laptops mit NVIDIA-GPU bereit, die für die Inferenz vorgesehen sind (siehe AP 3.1).
* **Aufnahmemodus der Datenerfassung:** **Endgültig festgelegt** — automatisierte Trajektorien mit **Action Noise Injection** (siehe AP 2.2 / 2.4). Ablauf: Per Gamepad werden für den Griff einige wenige Wegpunkte (Posen) definiert; diese werden anschließend **automatisiert und mit Rauschen** abgefahren, während synchron aufgezeichnet wird. Die Gamepad-Teleoperation dient damit **nur** dem Teachen der Wegpunkte, nicht der Aufzeichnung selbst. Zusätzliche Demonstrationsdaten entstehen langfristig aus der Übernahme des klassischen CV-Datensatzes (siehe AP 5.1). Mit dieser Entscheidung entfällt das Geschwindigkeits-/Stalling-Risiko aus AP 2.6 weitgehend, da die Ausführungsgeschwindigkeit über den Pfadplaner definiert und konstant ist statt vom menschlichen Tempo abzuhängen.

### Noch offen (bei physischem Aufbau zwingend zu ergänzen)
1. **Zweite Kamera (Szenen-/Top-View):** Exaktes Modell, native Auflösung, maximale FPS und Shutter-Typ ermitteln. Erwartet wird eine UVC-Kamera, die über `cv2.VideoCapture` abgegriffen werden kann.
2. **Montage-Stabilität der Top-View-Kamera:** Bewertung und Absicherung der Frage, wie stark sich die Szenenkamera im Betrieb verschieben kann (siehe AP 1.4) — inklusive der Entscheidung Wrist+Top vs. Wrist-only.
3. **Hardware-Ressourcen der Inferenz-Laptops:** GPU-Modell/VRAM, RAM und CPU der bereitgestellten Laptops dokumentieren, sobald bekannt; Inferenz-Durchsatz messen (siehe AP 3.1 / AP 4.1).
4. **Massenspeicher:** Prüfen, ob der Steuerungs-Laptop eine NVMe-SSD mit ausreichend freiem Speicher besitzt (siehe AP 2.3).

### Zwingend vor Beginn der Datenaufzeichnung zu entscheiden
*(Diese Punkte lassen sich nachträglich nicht oder nur mit Neuaufnahme korrigieren.)*
1. ~~**Aufnahmemodus:** Automatisierte Trajektorien mit Noise Injection oder Gamepad-Teleoperation~~ — **entschieden:** Noise Injection auf Basis per Gamepad geteachter Wegpunkte (siehe oben, AP 2.2 / 2.4).
2. **Ausführungsgeschwindigkeit:** Zielgeschwindigkeit der Planer-Trajektorie festlegen, einheitlich über alle Datenquellen inkl. der später übernommenen CV-Demos (siehe AP 2.6) — legt die spätere Ausführungsgeschwindigkeit der Policy unveränderlich fest. *Mechanismus bereits identifiziert:* NeuraPy stellt globale Regler bereit (`set_joint_speed` 0–100 %, `set_linear_speed` 0–1.0 m/s, `set_linear_acceleration`, `set_joint_acceleration`), die von `move_joint`/`move_linear` genutzt werden — der konkrete Wert ist aber noch zu wählen.
3. ~~**Bildvorverarbeitung:** ArUco-Homographie-Rektifizierung ja/nein~~ — **entschieden: ja.** Die Top-View wird per ArUco-Homographie auf eine kanonische Draufsicht rektifiziert (siehe AP 1.4). Muss bei Aufzeichnung und Inferenz identisch implementiert werden.
4. ~~**Kamerakonfiguration:** Wrist + Top vs. Wrist-only~~ — **entschieden: Wrist + Top.** Da feststeht, dass sich die Top-Kamera in der Praxis bewegen kann, ist die in AP 1.4 beschriebene **bewusste Posenvariation im Datensatz zwingend** (nicht optional) einzuplanen — die Homographie-Rektifizierung (Punkt 3) reduziert den nötigen Umfang dieser Variation, ersetzt sie aber nicht (siehe Grenzen der Rektifizierung, insbesondere Parallaxe bei Kiste/Objekt).

---

## AP 1: KI-Infrastruktur & Daten-Streaming

### 1.1 Kameraschnittstellen & Hardware
* **Welche genauen Kameramodelle werden verwendet?**
  Als Wrist-Kamera wird eine **Daheng Imaging VEN-161-61U3C** direkt am Roboterflansch montiert. Die Modellvariante ist bestätigt die **C-Variante (Farbe, Bayer-Sensor)** — nicht die Mono-Variante `VEN-161-61U3M`; das ist Voraussetzung für die RGB-basierte Policy. Als Szenenkamera (Top-View) dient eine zweite USB-Kamera, deren Modell noch zu dokumentieren ist.
* **Über welche Schnittstelle/SDK werden die Kameras nativ angesprochen?**
  Die gesamte Pipeline wird nativ in Python implementiert, die beiden Kameras werden aber **unterschiedlich angebunden**:
  * **Wrist-Kamera (Daheng):** Die VEN-161-61U3C ist eine **USB3-Vision-/GenICam**-Kamera und meldet sich **nicht** als UVC-Gerät. `cv2.VideoCapture` kann sie daher **nicht** öffnen. Der Zugriff erfolgt über das **Daheng Galaxy SDK** mit der offiziellen Python-API **`gxipy`** (Alternative: GenTL-Producer über `harvesters`).
  * **Szenenkamera (USB/UVC):** Abgriff wie gewohnt über OpenCV (`cv2.VideoCapture`).
  * *Auswirkung auf die Architektur — bewusst gering gehalten:* `gxipy` liefert die Frames als **NumPy-Array**, d. h. ab dem Frame-Abgriff ist die Verarbeitung identisch zu OpenCV (Resize, Farbraum, LeRobot, PyTorch). Die Kamera liefert **Bayer RG8/RG10**, das Debayering erfolgt entweder über `raw.convert("RGB")` des SDK oder über `cv2.cvtColor(..., cv2.COLOR_BAYER_RG2RGB)`. Beide Kameras werden hinter einer **gemeinsamen Kamera-Schnittstelle** (`read() -> (frame_rgb, timestamp)`) gekapselt, sodass Recorder und Inferenzschleife den Kameratyp nicht kennen müssen.
* **Welche Auflösung und Bildwiederholrate (FPS) liefern die Kameras nativ?**
  Wrist-Kamera: **1440 (H) × 1080 (V) bei 61,2 fps** (Sony IMX296, **Global Shutter**, 1/2.9", 8/10 bit). Datenblatt: `../20_Dokumentation/WristKamera-VEN-161-61U3MC-Datasheet.pdf`.
  Da nur mit 15 Hz aufgezeichnet wird (siehe 1.3), wird die Auflösung bereits **kameraseitig per ROI/Skalierung reduziert**, statt volle Frames zu übertragen und später in Python zu verkleinern — das spart USB-Bandbreite und CPU-Last.
  Die native Auflösung/FPS der Szenenkamera ist noch zu ermitteln.
* **Wie sind die Kameras physikalisch montiert?**
  Die Kameras werden fest verbaut: Die Szenenkamera an der Decke (Top) und die Wrist-Kamera starr am Effektor/Flansch des Roboterarms. Zur Stabilität der Kamerapose und den Konsequenzen einer Verschiebung siehe **AP 1.4**.
* **Shutter-Typ und Konsequenz für die Datenqualität**
  Die Wrist-Kamera besitzt einen **Global Shutter** — ideal, da sie sich mit dem Arm bewegt und bei der Rauscheinspielung (AP 2.4) beschleunigt wird. Handelsübliche USB-Webcams haben dagegen meist einen **Rolling Shutter**. Sollte die Szenenkamera ein Rolling-Shutter-Modell sein, ist das für eine ortsfeste Top-View unkritisch (statische Szene), erzeugt aber Verzerrungen, sobald sich Objekte oder eine Hand schnell durchs Bild bewegen (Testszenario in AP 5.1). Der Shutter-Typ der Szenenkamera ist daher zu dokumentieren.
* **Hardware-Trigger**
  Die Daheng-Kamera unterstützt Hardware- und Software-Trigger (opto-isolierter Eingang Line0 + programmierbarer GPIO). Eine echte getriggerte Synchronisation beider Kameras ist damit **nur möglich, wenn auch die Szenenkamera triggerfähig ist** — bei einer UVC-Webcam ist das nicht der Fall. Es bleibt daher beim zeitstempelbasierten Sampling (siehe 1.3); der Hardware-Trigger wird als optionale Rückfallebene vermerkt.
* **Werden Tiefendaten (Depth) genutzt?**
  Nein. Es liegen **keine** Tiefendaten vor; die Policy verarbeitet ausschließlich **RGB-Bilder**. Intel RealSense bzw. `pyrealsense2` wird im Projekt **nicht** eingesetzt.

### 1.2 Dashboard & UI
* **Welches UI-Framework wird präferiert?**
  NiceGUI wird aufgrund der modernen Optik und asynchronen Architektur bevorzugt, sofern der Implementierungsaufwand die Kern-Pipeline nicht beeinträchtigt. Alternativ wird auf schlanke OpenCV-Overlays zurückgegriffen.
* **Soll das Dashboard auf demselben Rechner laufen wie die Robotersteuerung oder remote im selben Netzwerk abrufbar sein?**
  Diese Entscheidung ist aktuell noch offen; die Software-Architektur sollte idealerweise beide Optionen (lokal und Netzwerk-Streaming) unterstützen.
  Nach aktueller Absprache ist geplant, alles auf einem Laptop laufen zu lassen, der mit dem LARA-Roboter verbunden ist und diesen steuert.
  *Korrektur/Präzisierung:* Die NeuraPy-Python-API kommuniziert **nicht** direkt über EtherCAT, sondern als Client über einen **TCP/IP-Socket zur Control-Box** (Default `192.168.2.13:65432`, Standard-Ethernet). EtherCAT ist controller-intern bzw. nur über ein separates Interface (`activate_ethercat_interface`) relevant. Der Laptop muss also per Ethernet im selben Netz wie die Control-Box liegen.
* **Welche Steuerungselemente werden im Dashboard benötigt?**
  Neben den Statusanzeigen soll das Dashboard vor allem den Prozess der Datenerfassung steuern (z. B. Start/Stopp der Aufzeichnung). Ergänzend benötigt:
  * Live-Vorschau beider Kamerabilder sowie Anzeige der tatsächlich erreichten Rate je Quelle (Erkennung von Frame-Drops, siehe 1.3).
  * Warnung bei erkannter Verschiebung der Szenenkamera (Marker-Abgleich, siehe 1.4).
  * Markieren einer Episode als erfolgreich/fehlgeschlagen sowie Verwerfen der letzten Episode (siehe 5.2).
  * Not-Halt-/Stopp-Button für die laufende Aufzeichnung bzw. Inferenz (siehe 4.2).

### 1.3 Zeit-Synchronisation der Datenquellen
Das `LeRobotDataset` erwartet pro Zeitschritt einen konsistenten Frame aus mehreren **asynchronen** Quellen (Wrist-Kamera, Szenen-Kamera, Roboterzustand, Greiferzustand). Diese Quellen liefern mit unterschiedlichen und teils schwankenden Raten. Es muss daher **aktiv eine Synchronisation implementiert werden**, damit Beobachtung (Observation) und Aktion zeitlich korrekt gepaart sind. Andernfalls lernt die Policy auf zeitversetzten Daten, was das Training instabil macht und die Live-Regelung verschlechtert.

**Konzept:**
* **Gemeinsame Zielrate:** **Festgelegt auf 15 Hz.** Alle Quellen werden auf dieses Raster zusammengeführt. Diese Rate gilt verbindlich für Aufzeichnung **und** Inferenz — der zeitliche Abstand zwischen zwei Aktionen ist im Datensatz implizit codiert, eine abweichende Rate bei der Inferenz führt zu systematisch zu schnellen bzw. zu langsamen Bewegungen.
* **Zeitstempel-basiertes Sampling:** Kamerabilder und Roboterzustand mit Zeitstempeln versehen. NeuraPy liefert hierfür die `*_with_timestamp`-Funktionen (UTC-Zeit); pro Zielframe wird jeweils die zeitlich nächstgelegene Messung gewählt (Nearest-Neighbor, ggf. leichte Interpolation der Pose).
* **Entkopplung per Threads/Queues:** Kamera-Capture und Roboter-Polling laufen in eigenen Threads und puffern jeweils den jüngsten Wert; der Recorder greift zur Zielrate den aktuellsten synchronisierten Satz ab.
* **Latenz-Budget:** Eine maximal zulässige Zeitdifferenz zwischen Kamerabild und Roboterzustand festlegen; Frames außerhalb dieses Fensters werden verworfen oder markiert. Richtwert: deutlich kleiner als ein halber Frame-Abstand, also **< 30 ms** bei 15 Hz.

### 1.4 Stabilität der Kamera-Extrinsik (Top-View)
Eine Behavior-Cloning-Policy lernt implizit die **feste Abbildung zwischen Kamerapose und Szene**. Verschiebt sich die ortsfeste Szenenkamera zwischen Aufzeichnung und Inferenz — durch Anstoßen, Vibration, Nachjustieren oder Neumontage —, entsteht ein Distribution Shift, den das Modell nicht als Fehler erkennt: Es greift systematisch versetzt oder scheitert still, ohne dass ein Sensorfehler gemeldet wird. In der Praxis ist das eine der häufigsten Ursachen für „das Modell lief gestern noch". Es ist daher als **reale, dauerhaft zu erwartende Störgröße** zu behandeln und muss durch den Datensatz abgedeckt sein.

Die **Wrist-Kamera ist von diesem Problem strukturell nicht betroffen**, solange sie starr am Flansch sitzt: Ihre Pose ist über die Roboterkinematik definiert und bewegt sich mit dem Arm mit. Ein Verrutschen der Wrist-Kamera **am Flansch** hätte allerdings denselben Effekt und ist mechanisch entsprechend zu sichern.

**Maßnahmen (mehrstufig):**
1. **Mechanisch:** Steife, verstiftete oder zumindest markierte Montage der Deckenkamera; jede Demontage/Neujustage wird protokolliert und zieht eine Neubewertung des Datensatzes nach sich.
2. **Datenseitig (Kernmaßnahme):** Die Kamerapose wird **bewusst variiert**. Die Aufzeichnung erfolgt in mehreren Blöcken mit leicht verschobener bzw. verdrehter Top-Kamera (Richtwert ± 2–5 cm Translation, ± 5° Rotation), damit die Policy Invarianz gegenüber der Kamerapose lernt, statt eine einzige Pose auswendig zu lernen. Diese Variation ist im Datensatz zu dokumentieren (Block-/Session-ID), damit ihr Effekt in AP 5 auswertbar ist.
3. **Augmentierung:** Random (Resized) Crop sowie Helligkeits-/Farbjitter beim Training — bei Diffusion Policy ohnehin Standard. Das deckt **kleine** Translationen ab, ersetzt aber keine echte Posenvariation bei größeren Abweichungen.
4. **Detektion statt stiller Fehler:** Ein statischer Marker (z. B. ArUco) im Sichtfeld der Top-Kamera erlaubt einen automatischen Abgleich der Kamerapose **vor jeder Episode** (Aufzeichnung) und **vor jedem Inferenzlauf**. Überschreitet die Abweichung eine Schwelle, wird im Dashboard (AP 1.2) gewarnt, die Episode verworfen bzw. der Start der Inferenz blockiert.

#### Nutzung der vorhandenen ArUco-Marker
Das klassische CV-Team setzt bereits ArUco-Marker auf dem Tisch ein. Diese lassen sich mitverwenden — allerdings **nicht** in der gleichen Weise wie dort, weshalb sie das Problem **nicht automatisch lösen**:

* **Warum die Marker beim klassischen Ansatz genügen:** Dort wird aus dem Marker die Kamerapose analytisch bestimmt und die Objektpose ins Roboter-Koordinatensystem umgerechnet. Verschiebt sich die Kamera, ändert sich die geschätzte Extrinsik mit — das Ergebnis bleibt korrekt. Die Kameraverschiebung ist dort strukturell unkritisch.
* **Warum das bei Behavior Cloning nicht gilt:** Die Policy verarbeitet **rohe Pixel** und führt keine Koordinatentransformation durch. Sie lernt „Objekt an dieser **Bild**position ⇒ diese Gelenkbewegung". Ein im Bild sichtbarer Marker wird vom Netz nicht als Referenz genutzt. Die bloße Anwesenheit der Marker ändert daher **nichts** an der Empfindlichkeit gegenüber Kameraverschiebung.

**Die Marker sind trotzdem der zentrale Hebel — sie müssen nur aktiv ausgewertet werden:**
* **(a) Überwachung (verbindlich, sehr geringer Aufwand):** Kamerapose je Episode/Lauf aus den Markern schätzen und gegen die im Datensatz hinterlegte Referenzpose vergleichen (siehe Punkt 4 oben). Da die Marker ohnehin vorhanden sind, ist das nahezu kostenlos und macht aus einem stillen Fehler einen sichtbaren.
* **(b) Homographie-Rektifizierung (entschieden — wird umgesetzt):** Aus den Marker-Ecken wird eine Homographie auf eine **kanonische Draufsicht** berechnet und jedes Bild dorthin entzerrt. Da die Deckenkamera auf einen **ebenen** Tisch blickt, wird die Tischebene dabei exakt rektifiziert — für alles, was auf dem Tisch liegt, wird die Kamerapose damit weitgehend irrelevant. Zusätzlich lässt sich über die Marker ein **fester Workspace-Ausschnitt** croppen, sodass das Bild immer denselben physischen Tischbereich zeigt.

**Grenzen der Rektifizierung (bewusst zu berücksichtigen):**
* Exakt ist die Entzerrung **nur in der Markerebene**. Objekte mit Höhe über dem Tisch — Zielobjekt, **Kistenwände**, Roboterarm — unterliegen **Parallaxe** und werden fehlerhaft verzerrt. Der Fehler wächst mit Objekthöhe × Kameraversatz und ist gerade im Kisten-Szenario relevant.
* Eine reine **Rotation** der Kamera um ihr optisches Zentrum ist vollständig korrigierbar, eine **Translation** dagegen nur für die Tischebene. Vibration/leichtes Verkippen ⇒ gut korrigierbar; ein mehrere Zentimeter versetzter Kameraaufbau ⇒ nur eingeschränkt.
* Die Rektifizierung muss bei **Aufzeichnung und Inferenz identisch** angewandt werden und ist damit **vor** Beginn der Datenaufnahme festzulegen — nachträglich ist sie auf bestehende Datensätze nur eingeschränkt anwendbar.
* Marker werden durch Arm und Kiste zeitweise **verdeckt**. Daher mehrere Marker über den Tisch verteilen und jeweils mit den sichtbaren rechnen; bei zu wenigen sichtbaren Markern auf die letzte gültige Homographie zurückfallen und warnen.
* Nebeneffekt (positiv): Die Marker liefern ein **gemeinsames Bezugssystem mit dem CV-Team** und erleichtern damit die spätere Übernahme von dessen Demonstrationsdaten (siehe AP 5.1).

**Schlussfolgerung:** Marker-basierte Überwachung und Rektifizierung decken kleine bis mittlere Verschiebungen weitgehend ab und sind wegen des planaren Top-View-Aufbaus hier die passende Maßnahme. Sie **ersetzen die bewusste Posenvariation im Datensatz (Punkt 2) nicht**, sondern reduzieren deren notwendigen Umfang. Die verbleibende Toleranz wird über den Störgrößen-Test in AP 5.1 quantifiziert.

**Designentscheidung: Wrist + Top-View (entschieden)**
* *Wrist-only* würde das Extrinsik-Problem vollständig eliminieren, Datenrate und Rechenlast halbieren und die Synchronisation vereinfachen. Nachteil: kein globaler Kontext. Das funktioniert nur, wenn das Objekt aus der Startpose bereits im Sichtfeld der Wrist-Kamera liegt, und ist beim Greifen **aus der Kiste** kritisch — kurz vor dem Griff verdecken Greiferbacken und Kistenwand einen Großteil des Bildes („letzte Zentimeter blind").
* *Top-View* liefert genau den globalen Kontext, der für die AP-5-Szenarien (verschobenes/rotiertes Objekt, Hand greift dazwischen) gebraucht wird.
* **Entscheidung: beide Kameras werden aufgezeichnet.** Da feststeht, dass die Top-Kamera sich in der Praxis bewegen kann, ist die **bewusste Posenvariation der Top-Kamera im Datensatz zwingender Bestandteil der Aufzeichnung** — nicht nur eine Option (siehe Maßnahme 2 oben). Die ArUco-Homographie-Rektifizierung (siehe unten) wird ergänzend eingesetzt, um den nötigen Umfang dieser Variation zu reduzieren, deckt aber wegen der Parallaxe bei Objekt/Kiste nicht alles ab.
* Eine **Wrist-only-Ablation** bleibt trotzdem sinnvoll und günstig, da der Datensatz beide Ströme enthält: Ein Top-View-Stream lässt sich nachträglich nicht ergänzen, ein vorhandener aber jederzeit für einen Vergleichslauf weglassen (siehe AP 5.1).

---

## AP 1.5: LeRobot ↔ NeuraPy Integrationsschicht (Robot- & Teleop-Adapter)
*Querschnitts-Arbeitspaket – bildet die Grundlage sowohl für AP 2 (Datenerfassung) als auch für AP 4 (Live-Inferenz).*

LeRobot bringt **keine** native Unterstützung für Neura-Roboter mit. Es muss daher eine eigene Adapter-Schicht implementiert werden, die den LARA 5 über NeuraPy in die LeRobot-Abstraktionen (`Robot` / `Teleoperator`) einbindet. Diese Schicht wird bei der Datenaufzeichnung **und** bei der Live-Inferenz genutzt und ist damit die zentrale Brücke zwischen KI-Framework und Hardware. Ihr Fehlen wurde bisher nicht als eigenes Arbeitspaket erfasst, ist aber Voraussetzung für AP 2 und AP 4.

### 1.5.1 Anforderungen an den Adapter
* **Observation-Abgriff (State):** Auslesen von Gelenkwinkeln (`get_current_joint_angles`) bzw. TCP-Pose (`get_tcp_pose_quaternion`) sowie des Greiferzustands. Für die Synchronisation (siehe 1.3) werden bevorzugt die `*_with_timestamp`-Varianten verwendet.
* **Action-Ausgabe (Command):** Umsetzung der vom Modell prädizierten Aktion auf ein echtzeitfähiges Servo-Interface:
  * Gelenkraum-Policy → `activate_servo_interface('position')` + `servo_j` (Zielwinkel in rad).
  * Kartesische Policy → `movelinear_online` (Ziel als `[X, Y, Z, qw, qx, qy, qz]`).
  * Greifer → binärer Befehl `gripper('on'/'off')` bzw. `grasp()` / `release()`.
* **Konsistenz Training ↔ Inferenz:** Der State-/Action-Raum des aufgezeichneten Datensatzes muss **exakt** dem bei der Inferenz bespielten Servo-Interface entsprechen (gleiche Größen, Einheiten, Reihenfolge, Bezugsframe). Ein Bruch hier führt zu unbrauchbaren Policies.
* **Lebenszyklus/Sicherheit:** Sauberes Aktivieren/Deaktivieren des Servo-Interface (`deactivate_servo_interface`), definierte Stopp-Pfade (`stop`, `stop_movelinear_online`) und Einbindung des Not-Aus (siehe AP 4.2).

### 1.5.2 Offene Designentscheidung: Action-Repräsentation
Noch abzuklären (mit direktem Einfluss auf Adapter und Datensatz-Struktur):
* **Gelenkraum vs. kartesisch:** Aktuell wird der Gelenkraum bevorzugt (→ `servo_j`).
  *Antwort auf die Frage „können beide erfasst werden?" — hier ist zwischen State und Action zu unterscheiden:*
  * **Observation/State: ja, beide erfassen.** Gelenkwinkel **und** TCP-Pose werden parallel aufgezeichnet. Der Mehraufwand ist vernachlässigbar (wenige Float-Werte pro Frame), beim Training lässt sich jederzeit auswählen oder kombinieren, und die kartesische Pose wird ohnehin für den Vergleich mit dem klassischen CV-Ablauf (AP 5) sowie für Geofencing/Kollisionsprüfung (AP 2.4, AP 4.2) benötigt. **Entscheidung: beide Größen werden immer mitgeschrieben.**
  * **Action: nein — genau ein Raum.** Der Action-Vektor muss 1:1 dem bei der Inferenz bespielten Servo-Interface entsprechen. Zwei parallele Action-Räume wären redundante und (durch IK-Mehrdeutigkeit und Rundung) widersprüchliche Labels, die das Training verschlechtern. **Entscheidung: Action = Gelenkraum (`servo_j`); die TCP-Pose ist reiner State-Kanal.**
* **Absolute vs. relative (Delta-)Aktionen:** ob das Modell absolute Zielposen oder inkrementelle Deltas ausgibt. Die in AP 2.4 beschriebene Rausch-Methodik legt absolute Soll-Posen (`T_soll(t+1)`) als Label nahe.

---

## AP 2: Datenerfassung & Teleoperation (LeRobot)

### 2.1 Roboter-Schnittstellen & State-Action-Space
* **Welches konkrete Robotermodell von Neura Robotics wird eingesetzt?**
  LARA 5. Datenblätter und Infos liegen unter 10_Neura
* **Welche physikalische Schnittstelle/API stellt Neura bereit?**
  Es wird die native Python-Schnittstelle von Neura Robotics verwendet. Auf ROS / ROS2 wird in diesem Projekt explizit verzichtet, um das System schlank und Python-nativ zu halten.
* **Wie sieht der exakte Zustandsraum (State Space) aus?**
  * *Roboterpose:* Die **Action** wird im Gelenkraum (Joint Positions) geführt. Als **State** werden Gelenkwinkel **und** TCP-Pose gemeinsam aufgezeichnet (Begründung siehe 1.5.2).
  * *Greiferzustand:* Ursprünglich wurde die kontinuierliche Erfassung der realen Greifer-Position (Weite/Öffnung) bevorzugt. **Korrektur nach Prüfung der NeuraPy-API:** Diese stellt nur **binäre** Greiferbefehle (`gripper('on'/'off')`, `grasp()`/`release()`) bereit und liefert **keine** Rückmeldung der Ist-Öffnungsweite. Der Greiferzustand wird daher als **binärer Zustand (Offen/Geschlossen)** in State und Action geführt. Eine kontinuierliche Regelung wäre nur über einen direkten Zugriff auf den Zimmer-Greifer (z. B. Modbus/IO) möglich und ist aktuell nicht vorgesehen.
  * *Konsequenz — Totzeit des Greifers:* Der Befehl (`gripper('on')`/`grasp()`) wird sofort ausgelöst, die Backen benötigen aber eine reale physische Zeit, bis sie tatsächlich geschlossen sind (Pneumatik/Motor). Da keine Ist-Rückmeldung existiert, würde ein naiver Recorder den Zustand „geschlossen" bereits **mit dem Befehl** labeln, obwohl die Backen in den folgenden Frames im Kamerabild noch sichtbar offen sind — ein systematischer Bild-Label-Fehler. Zusätzlich darf das automatisierte Skript (AP 2.2/2.4) nach dem Greifbefehl nicht sofort mit der Abfahrbewegung fortfahren, sonst hebt der Arm an, bevor die Backen wirklich geschlossen sind. Die Totzeit wird sowohl im Aufzeichnungsskript als fester **Dwell** vor der Folgebewegung als auch im Recorder-Label konsistent berücksichtigt.
    **Vorläufiger Arbeitswert: 500 ms.** ⚠️ *Noch offen — vor der finalen Datenaufzeichnung zu verifizieren* (Messung z. B. per Video: Zeit vom abgesetzten Befehl bis zum sichtbar vollständig geschlossenen Greifer). Der Wert ist bei Abweichung anzupassen, da er sowohl die Zykluszeit als auch die Bild-Label-Zuordnung beeinflusst.
* **Welche Frequenz wird für die Teleoperations-Aufzeichnung angestrebt?**
  **Festgelegt: 15 Hz**, einheitlich für alle Datenquellen und identisch bei der Inferenz (siehe AP 1.3). Der Wert liegt im von LeRobot üblichen Bereich (10–30 Hz) und ist mit den Kameras (61,2 fps nativ) und der NeuraPy-Abfragerate problemlos erreichbar.

### 2.2 Teleoperation & Hardware-Führung
* **Wie erfolgt das manuelle Führen konkret?**
  **Endgültig festgelegt:** Der Roboter wird per Gamepad (alternativ via Gravity Compensation / Lead-Through) **ausschließlich zum Teachen der Wegpunkte** an die relevanten Zielpositionen verfahren; diese Wegpunkte werden abgespeichert. Die eigentliche **Aufzeichnung** erfolgt **nicht** per Gamepad, sondern automatisiert: Ein Skript verfährt den Roboter zwischen den geteachten Punkten (PTP zur Anfahrt, Linear zum Greifen, Linear zur Abfahrt, PTP zur Zielpose) und prägt dabei gezielt Rauschen auf (siehe AP 2.4), um starre Fahrtplanung und daraus resultierende Fehler beim Lernen zu vermeiden.
  Weitere Demonstrationsdaten entstehen langfristig durch Übernahme des klassischen, CV-basierten Ansatzes (siehe AP 5.1).
  *Konsequenz:* Die Ausführungsgeschwindigkeit der Policy wird über die **Geschwindigkeit des Pfadplaners** bestimmt (konstant, vom Skript vorgegeben) und hängt **nicht** vom Tempo des menschlichen Teachens ab. Das in AP 2.6 beschriebene Stalling-/Tempo-Risiko der reinen Gamepad-Aufzeichnung entfällt damit weitgehend; die dortigen Anforderungen an eine **einheitliche, konstante Zielgeschwindigkeit** gelten aber unverändert für den Pfadplaner und für die Abstimmung mit den übernommenen CV-Demos. Bei automatisierten Trajektorien liefert die Rauscheinspielung die Korrekturdaten; bei Gamepad-Teleoperation liefert der Mensch die Korrekturen implizit, und die asymmetrische Label-Logik aus 2.4 entfällt. Da beide Varianten unterschiedliche Datensatzstrukturen erzeugen, ist dies die **wichtigste noch zu treffende Entscheidung** vor Beginn der Datenaufzeichnung.

* **Greifer-Details:**
  Es ist ein Backengreifer des Herstellers Zimmer montiert. siehe https://www.zimmer-group.com/de/produkte/komponenten/robotertechnik/match-end-of-arm-ecosystem/match-greifer/lwr-hrc-03/produkte/lwr50l-03-00001-a
  Datenblatt liegt unter 20_Dokumentation/ZimmerGreiferDatenblatt.pdf

### 2.3 Speicher- & Datenmanagement
* **Wo sollen die Daten lokal abgelegt werden?**
  Lokal auf einer SSD im Steuerungsrechner. **Präzisierung:** `LeRobotDataset` encodiert die Episoden nach der Aufnahme als **MP4-Video**; unkomprimierte Bildarrays fallen nur während der Aufzeichnung im Puffer an. Der dauerhafte Speicherbedarf ist dadurch deutlich geringer als die Rohdatenrate. Kritisch ist damit weniger die Kapazität als die **Schreib- und Encoding-Last während bzw. unmittelbar nach der Aufnahme**.
  Größenordnung roh: zwei Streams à 640 × 480 RGB bei 15 Hz ≈ **28 MB/s**; bei voller Wrist-Auflösung (1440 × 1080) entsprechend mehr — ein weiterer Grund für die kameraseitige Auflösungsreduktion (siehe 1.1).
  Eine **NVMe-SSD** ist dringend empfohlen. Ob der vorgesehene Laptop eine NVMe-SSD mit ausreichend freiem Speicher besitzt, ist **noch zu prüfen**.
* **Gibt es Vorgaben zur Ordnerstruktur oder Anbindung an Hugging Face Communities?**
  Die Daten werden lokal im standardisierten `LeRobotDataset`-Format strukturiert. Eine Cloud-Anbindung an Hugging Face entfällt, da das Projekt vollständig lokal und autark betrieben wird.

### 2.4 Methodik der Datenerfassung: Action Noise Injection
*(Verbindlich — siehe endgültige Festlegung in 2.2: Gamepad nur zum Teachen der Wegpunkte, Aufzeichnung erfolgt automatisiert mit Rauscheinspielung.)*
Um trotz automatisierter Trajektorien (PTP/Linear/Splines zwischen geteachten Punkten) ein robustes Imitation-Learning-Modell zu erhalten, wird eine fortgeschrittene Datengenerierungs-Methodik implementiert. Ein rein starr-ideales Abfahren von einprogrammierten Bahnen führt beim Behavior Cloning unweigerlich zu dem mathematischen Problem der Divergenz (Kompensation von Abweichungen wird mangels fehlerhafter Beispiele im Datensatz nie gelernt).


```

Verteilung der Trainingsdaten (Eindimensionale Ideallinie):
[Ziel] <=================== Idealer Pfad =================== [Start]
x (Sollte die Inferenz hier abweichen, bricht das System ab)

Erzeugtes Vektorfeld durch Noise Injection (Trichter-Effekt):
[Ziel] <=================== Idealer Pfad =================== [Start]
↖    ↖    ↖    ↙    ↙    ↙    ↖    ↖    ↖    ↙    ↙
[Korrektur-Aktionen ziehen den Arm aktiv zurück]

```

#### Das technische Prinzip der "Asymmetrischen Rauscheinspielung":
1. **Der Planer berechnet die ideale Soll-Trajektorie ($T_{soll}$):** Ein Python-Skript generiert eine weiche Bahn (z. B. kubische Splines) zwischen den für das Objekt geteachten Punkten (Anfahrt, Greifen, Heben, Ziel).
2. **Aufprägung von physischem Rauschen ($N_t$):** Während der Roboter die Bewegung physisch ausführt, beaufschlagt der Pfadplaner die Gelenkwinkel oder TCP-Koordinaten in jedem Zeitschritt $t$ mit einem kontinuierlichen Rauschen (z. B. über einen Ornstein-Uhlenbeck-Prozess, um weiche, realistische Abweichungen statt unruhigem Zittern zu erzeugen):
   $$T_{ist}(t) = T_{soll}(t) + N_t$$
   Der Roboter schlingert somit auf dem Weg zum Objekt gezielt und sichtbar im Zentimeterbereich ($\pm 1-2\text{ cm}$) hin und her.
   *Begründung der Amplitude:* $\pm 1-2\text{ cm}$ ist bewusst deutlich über normalem Kamera-/Sensorrauschen und alltäglichem Vibrations-Jitter gewählt. Die Policy soll aus dem Kamerabild ein eindeutiges Korrektursignal lernen („Objekt im Bild verschoben ⇒ aktiv gegensteuern") — eine Auslenkung im Millimeterbereich wäre im Bild kaum vom Bildrauschen zu unterscheiden und würde kein verlässlich lernbares Signal erzeugen. Die Amplitude gilt für die **Transitphase**; im kritischen Endanflug ist sie durch die Trichter-Dämpfung (Punkt 3 unten) längst gegen 0 abgeklungen, wodurch die absolute Größe während der Annäherung ans Objekt unkritisch bleibt.
   *Entscheidung: Rauschen wird kartesisch aufgeprägt und per inverser Kinematik in Gelenkwinkel umgerechnet — nicht direkt im Gelenkraum.* Grund: Die Amplitude sowie die asymmetrische Begrenzung „kein/reduziertes Rauschen Richtung Tisch bzw. Kistenwand" (siehe Sicherheitsanforderung unten) sind **kartesisch/richtungsbezogen** definiert. Im Gelenkraum lässt sich eine feste Weltrichtung wie „Richtung Tisch" nicht fix einem Gelenk zuordnen — die Beziehung zwischen Gelenkwinkeländerung und resultierender kartesischer Auslenkung hängt über die Jacobi-Matrix von der aktuellen Armkonfiguration ab (ein Rauschen auf einem schulternahen Gelenk verschiebt den TCP deutlich stärker als dasselbe Rauschen auf dem Handgelenk). Ein fester Gelenkraum-Rauschwert könnte die geforderte Sicherheitsgrenze daher **nicht zuverlässig garantieren** und wäre in bestimmten Posen kollisionsträchtiger. Der kartesische Ansatz + IK ist damit die sicherere, aber implementierungsaufwändigere Wahl.
   *Umsetzung der IK — NeuraPy liefert die benötigten Funktionen mit:*
   Ein eigenes Kinematikmodell (DH-Parameter) ist **nicht** erforderlich. Die API stellt bereit:
   * `compute_inverse_kinematics(target_pose, reference_joint, representation='rpy'|'quaternion')` → Gelenkwinkel in rad; wirft `IKNotFound`, falls keine Lösung existiert.
   * `compute_forward_kinematics(joint_angles, target_frame='tool'|'flange'|'wrist'|'elbow'|'link6'|'link5', representation=...)` → TCP-Pose im XYZRPY-Format.

   **Robustheitsstrategie (Lösungszweig-Konsistenz):** Der Parameter `reference_joint` ist genau der benötigte Seed. Es wird bei jedem Zeitschritt die **Lösung des vorherigen Zeitschritts als `reference_joint`** übergeben. Die IK liefert dadurch die zur Referenzkonfiguration nächstgelegene Lösung, wodurch Sprünge zwischen Gelenkkonfigurationen **konstruktiv ausgeschlossen** werden — eine separate Zweigauswahl entfällt. Für den ersten Zeitschritt dient die aktuelle Ist-Gelenkstellung als Referenz.

   **Zeitpunkt der Berechnung:** Die IK läuft **offline bei der Trajektoriengenerierung**, nicht in der Echtzeit-Servoschleife. Das ist ohnehin durch die Kollisions-Vorabprüfung (siehe Sicherheitsanforderung unten) vorgegeben und vermeidet zugleich, dass der TCP-Roundtrip zur Control-Box pro Zeitschritt in die Laufzeit eingeht. Ergebnis ist eine fertige, geprüfte Gelenkraum-Trajektorie, die anschließend per `servo_j` abgefahren wird.

   **Absicherungen (jeweils Trajektorie neu sampeln bei Verletzung):**
   * `IKNotFound` abfangen → Rauschsample verwerfen und neu ziehen (dient zugleich als Erreichbarkeitsprüfung).
   * Maximale Gelenkwinkeländerung $|\Delta q|$ zwischen zwei Zeitschritten begrenzen — erkennt Singularitätsdurchgänge und Konfigurationssprünge auch dann, wenn die IK formal eine Lösung liefert.
   * Rückprobe per `compute_forward_kinematics`: Die aus der IK-Lösung zurückgerechnete Pose muss innerhalb einer Toleranz mit der kartesischen Sollpose übereinstimmen.
   * Gelenkwinkel gegen Achsgrenzen prüfen.
   * *Singularitätshinweis:* In Singularitätsnähe erzeugt eine kleine kartesische Auslenkung sehr große Gelenkwinkeländerungen. Die $|\Delta q|$-Schranke fängt das ab; zusätzlich sollten die geteachten Wegpunkte so gewählt werden, dass die Bahn singularitätsferne Konfigurationen nutzt.

   *Alternative (nur falls die eingebaute IK sich als unzureichend erweist):* Differentielle IK über die Jacobi-Matrix mit gedämpfter Pseudoinverser (Damped Least Squares, $\Delta q = J^T(JJ^T+\lambda^2 I)^{-1}\Delta x$). Da das Rauschen eine **kleine** Störung um eine bekannte Sollpose ist, wäre die Linearisierung zulässig und Kontinuität automatisch gegeben. NeuraPy exportiert jedoch keine Jacobi-Matrix — sie müsste per finiter Differenzen aus `compute_forward_kinematics` approximiert werden. Angesichts der vorhandenen, seedbaren IK ist dieser Weg voraussichtlich nicht nötig.

   **Vortest am realen Roboter (vor der Implementierung, Skript: `tools/check_ik.py`; bewegt den Roboter nicht):**
   0. **Voraussetzung — Tool-/TCP-Kalibrierung:** `get_flange_pose()` gegen `get_tcp_pose_quaternion()` vergleichen. Sind beide (nahezu) identisch, ist im Controller kein Greifer-Tool hinterlegt (vgl. Status „NoTool" im Controller-Backup) — alle IK-Zielposen würden sich dann auf den Flansch statt die Greiferspitze beziehen. Dies erfordert nur die **mechanische** Montage des Greifers plus Eintrag der Geometrie im Tool-GUI/`create_tool`, **nicht** die elektrische/pneumatische Anbindung.
   1. Erreichbarkeit im späteren Rausch-Korridor um jeden geteachten Wegpunkt (Erfolgsquote von `compute_inverse_kinematics`).
   2. Seed-Konsistenz entlang eines dicht abgetasteten Pfads (Warm-Start-Sprünge trotz `reference_joint` erkennen).
   3. **Singularitätsnähe genau an der Greifpose** (wichtigster Punkt): Ein senkrecht von oben greifender Endeffektor ist bei vielen 6-Achs-Robotern strukturell nah an der Handgelenk-Singularität — dort kann eine kleine kartesische Auslenkung eine unverhältnismäßig große Gelenkbewegung erzeugen, genau an der Stelle, wo die Trichter-Dämpfung eigentlich zur Ruhe kommen soll.
   4. RPY- vs. Quaternion-Repräsentation vergleichen (Gimbal-Lock-Risiko bei Pitch ≈ ±90°, typisch bei senkrechter Greif-Orientierung).
   5. Laufzeit pro IK-/FK-Aufruf (unkritisch für Echtzeit, relevant fürs Iterationstempo bei der Trajektoriengenerierung).
3. **Dynamische Dämpfung (Trichter-Verengung):** Je näher sich der Greifer dem eigentlichen Greifpunkt am Objekt nähert, desto stärker wird das Rauschen $N_t$ softwareseitig gegen $0$ gedämpft, um eine präzise Kollision mit dem Objekt und einen sauberen Griff zu gewährleisten.
4. **Die asymmetrische Datenaufzeichnung im `LeRobotDataset`:**
   Um dem neuronalen Netz Korrekturverhalten beizubringen, werden die Daten asymmetrisch im Datensatz gepaart:
   * **Eingabe (Observation):** Das **echte, fehlerhafte** Kamerabild (Wrist und Szene zeigen die verrauschte, verschobene Perspektive) gekoppelt mit der **echten, verrauschten** Roboterpose ($T_{ist}(t)$).
   * **Ausgabe (Action / Label):** Der **ungestörte, ideale** Bewegungsschritt der Soll-Trajektorie ($T_{soll}(t+1)$), welcher den Roboter direkt zurück auf die Ideallinie bzw. zum Ziel zieht.

**Resultierender Lerneffekt:** Die Diffusion Policy lernt durch diese Methodik im Training ein stabiles Kraftfeld (Vektorfeld) um das Objekt herum. Sie lernt: *"Wenn das Objekt im Kamerabild nach links versetzt zu sehen ist (weil der Roboter real nach rechts abgedriftet ist), lautet die korrekte Aktion: Steuere aktiv nach links."* Erst dieses Verfahren ermöglicht die in AP 5 geforderte, dynamische Anpassung an verschobene Objekte in Echtzeit.

#### Sicherheitsanforderung: Kollisionsausschluss während der Rauscheinspielung
Das gezielte Aufprägen von Rauschen (±1–2 cm) in Tischnähe und beim Greifen aus einer Kiste birgt ein reales Kollisionsrisiko (Tisch, Kistenwände, Objekt, Vorrichtung). **Kollisionen müssen während der gesamten Datenaufzeichnung zwingend ausgeschlossen werden** — sowohl zum Schutz der Hardware als auch, weil ein Kollisions-Stopp die Trajektorie verfälscht und damit den Datensatz unbrauchbar macht.

**Konzept (mehrstufig, defense-in-depth):**
1. **Geometrische Begrenzung des Rauschens:** Das Rauschen wird nicht isotrop aufgeprägt, sondern auf einen sicheren Korridor um die Soll-Trajektorie beschränkt. Insbesondere wird die Auslenkung Richtung Tisch bzw. Kistenwände asymmetrisch begrenzt (z. B. kein oder stark reduziertes Rauschen nach unten / zur Wand).
2. **Vorab-Prüfung (Offline):** Jede verrauschte Soll-Trajektorie wird **vor** der physischen Ausführung gegen ein Kollisionsmodell der Umgebung (Tisch, Kiste, bekannte Hindernisse) geprüft. Trajektorien, die den Sicherheitsabstand unterschreiten, werden neu gesampelt (Rejection Sampling) oder beschnitten.
   *Aufwandshinweis:* NeuraPy bringt **kein** Umgebungs-Kollisionsmodell mit; dieses muss selbst erstellt werden. Die Umgebung ist überschaubar (Tisch, Kiste, ggf. Vorrichtung), sodass ein Satz einfacher **Quader-Geometrien** ausreicht.
   Für die Roboterseite ist **kein eigenes Kinematikmodell nötig**: `compute_forward_kinematics` akzeptiert über `target_frame` auch Zwischenglieder (`'flange'`, `'wrist'`, `'elbow'`, `'link5'`, `'link6'`). Damit lassen sich pro Zeitschritt mehrere Stützpunkte entlang des Arms bestimmen und gegen die Quader prüfen — zusammen mit einer groben Hüllgeometrie (Kugeln/Kapseln um diese Punkte) für Arm und Greifer. Der Aufwand ist damit überschaubar, bleibt aber ein eigener Implementierungsschritt.
3. **Trichter-Dämpfung (vgl. Punkt 3 oben):** Die ohnehin vorgesehene Dämpfung des Rauschens gegen 0 nahe des Greifpunkts dient gleichzeitig der Kollisionsvermeidung im kritischen Endanflug.
4. **Online-Absicherung (Laufzeit):** Während der Ausführung bleiben die controllerseitigen Schutzmechanismen aktiv (`enable_collision_detection`, Reflex) und die Workspace-Grenzen (Geofencing, siehe AP 4.2) werden im Python-Skript überwacht. Ein ausgelöster Schutzstopp führt zum **Verwerfen** der betroffenen Episode (fehlerhafte Daten dürfen nicht in den Datensatz).

### 2.5 Umfang des Datensatzes (grobe Richtgröße)
Die konkret benötigte **Anzahl an Demonstrationen** hängt stark von der finalen Umsetzung ab (Komplexität der Greifaufgabe, Qualität und Menge der Rausch-Augmentierung, Anteil der zusätzlich übernommenen Klassik-Demonstrationen aus dem CV-Ablauf). Sie kann daher nicht vorab fixiert, sondern nur **iterativ** anhand der Trainings-/Validierungsleistung bestimmt werden. Als **grober Startrahmen** für Diffusion-Policy-Greifaufgaben dieser Art wird eine Größenordnung von **einigen Dutzend bis wenigen Hundert Demonstrationen** pro Objekt/Szenario angesetzt und bei Bedarf nachjustiert.

### 2.6 Bewegungsgeschwindigkeit & zeitliche Konsistenz der Demonstrationen
*Hinweis: Mit der Festlegung in 2.2 (Gamepad nur zum Teachen, Aufzeichnung per automatisiertem Pfadplaner) betrifft das Stalling-Risiko aus Punkt 1 unten primär die künftig übernommenen CV-Demonstrationen sowie die Wahl der Planer-Geschwindigkeit — die Grundaussage und die Anforderung an einheitliches Tempo bleiben aber vollständig gültig.*

**Kernaussage: Die Ausführungsgeschwindigkeit wird mitgelernt und ist bei der Inferenz kein freier Parameter.** Werden die Demonstrationen langsam aufgezeichnet, führt der Roboter die Aufgabe später **genauso langsam** aus.

**Begründung:** Die Action ist eine absolute Zielpose pro Zeitschritt bei fester Rate (15 Hz, siehe 1.3). Die Geschwindigkeit steckt implizit in der **Schrittweite zwischen aufeinanderfolgenden Actions**. Langsames Teachen erzeugt sehr kleine Schrittweiten; die Policy lernt genau diese und reproduziert sie.

**Nachträgliche Korrektur ist nicht sauber möglich.** Schnellere Ausführung erforderte entweder eine höhere Taktung der Regelschleife (bricht die Zeitkonsistenz, auf die trainiert wurde) oder eine Skalierung der Actions (erzeugt Aktionen außerhalb der Trainingsverteilung). Beides verletzt die Konsistenzforderung aus 1.5.1. ⇒ **Die Zielgeschwindigkeit ist vor Beginn der Datenaufzeichnung festzulegen.**

**Konkrete Risiken bei Gamepad-Teleoperation:**
1. **Stalling/Deadlock durch Zögern (kritischstes Risiko):** Überlegen, Nachjustieren oder Stop-and-Go erzeugt viele Frames mit nahezu identischer Beobachtung und Action ≈ „stehenbleiben". Bei der Inferenz kann die Policy in genau diesen Zustand laufen: Sie gibt „nicht bewegen" aus, die Beobachtung ändert sich dadurch nicht, also gibt sie erneut „nicht bewegen" aus — der Roboter bleibt stehen und findet nicht mehr heraus. Diffusion Policy ist hiergegen robuster als eine einfache Regression (sie mittelt nicht über Modi), und Action Chunking (siehe 4.1) entschärft es zusätzlich, beseitigt es aber nicht.
2. **Inkonsistentes Tempo über Demonstrationen hinweg:** Wird dieselbe Phase der Aufgabe mal schnell, mal langsam demonstriert, wird die Action-Verteilung in der Schrittweite multimodal. Die Policy sampelt dann pro Durchlauf einen Modus, das Timing wird unvorhersagbar. **Gleichmäßigkeit ist wichtiger als absolute Geschwindigkeit.**
3. **Vermischung der Datenquellen:** Gamepad-Demos (langsam), Noise-Injection-Demos (automatisiert, gleichmäßig) und übernommene CV-Demos (AP 5.1) haben von Natur aus unterschiedliches Tempo. Werden sie gemischt, entsteht genau die Multimodalität aus Punkt 2. **Anforderung: vergleichbares Tempo über alle Datenquellen hinweg.**
4. **Datenvolumen ohne Informationsgewinn:** Langsame Demos erzeugen viele nahezu identische Frames — Speicherbedarf und Trainingszeit steigen, die Anzahl wirklich verschiedener Situationen jedoch nicht.
5. **Quasi-statische Dynamik:** Sehr langsam aufgezeichnete Bewegungen enthalten keine Trägheits-/Nachlaufeffekte. Eine später gewünschte schnellere Ausführung träfe auf Dynamik, die im Datensatz nie vorkam.

**Empfohlenes Vorgehen:**
* **Bevorzugt:** Das Gamepad nur zum **Anfahren und Speichern der Wegpunkte** nutzen; die eigentliche Aufzeichnung fährt ein Skript mit **definierter, gleichmäßiger Geschwindigkeit** ab (Variante aus 2.2). Damit wird „der Mensch ist langsam" von „der Roboter ist langsam" entkoppelt, das Tempo ist automatisch konsistent, und nur so ist die Noise Injection aus 2.4 überhaupt möglich.
* **Falls direkt per Gamepad aufgezeichnet wird:** Bewegungsablauf vorher einüben; Zögerphasen nicht aufzeichnen bzw. Episode verwerfen (siehe 5.2); Leerlauf-Frames unterhalb einer Mindest-Schrittweite herausfiltern. **Ausnahme:** bewusste Pausen wie die Greifer-Totzeit (siehe 2.1) müssen als **konstanter** Dwell erhalten bleiben und dürfen nicht vom Menschen variabel eingebracht werden.
* **Anti-Pattern:** Kein normiertes Zeit-/Phasensignal als Modell-Eingang zur Kompensation. Das macht die Policy quasi open-loop und zerstört genau das reaktive Korrekturverhalten, das in AP 5 gefordert ist.

---

## AP 3: Modell-Training (Diffusion Policy)

### 3.1 Lokale Hardware-Ressourcen
* **Welche Grafikkarte (GPU) steht für das lokale Training zur Verfügung?**
  **Festgelegt:** Das Training erfolgt auf einer **NVIDIA RTX 5070 Ti (16 GB VRAM)**. Zusätzlich werden Laptops mit NVIDIA-GPU bereitgestellt, deren genaue Ausstattung noch nicht bekannt ist; diese sind für die **Inferenz** vorgesehen und dafür voraussichtlich ausreichend (Messung siehe AP 4.1). Die zuvor erwogene Option Jetson Thor entfällt.
  * **Randbedingung Blackwell-Architektur:** Die RTX 5070 Ti besitzt Compute Capability **sm_120** und benötigt **CUDA 12.8 oder neuer** mit einem entsprechend gebauten PyTorch (cu128+). Ältere PyTorch-Wheels laufen auf dieser Karte nicht. Die Toolchain (Treiber, PyTorch-Build) ist **vor Trainingsbeginn** zu verifizieren — das ist der wahrscheinlichste Stolperstein beim Setup.
  * **Konkretes Prüfvorgehen** (Skript: `tools/check_gpu.py`, auf dem Zielrechner ausführen):
    1. **Treiber:** `nvidia-smi` muss laufen und im Kopf CUDA 12.8+ melden (das ist die vom Treiber *maximal unterstützte* Version, nicht die installierte).
    2. **PyTorch installieren:** `pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128`. Ein separates CUDA-Toolkit (`nvcc`) ist **nicht** nötig — die Wheels bringen die Runtime mit.
    3. **Entscheidender Test:** `torch.cuda.get_arch_list()` muss **`sm_120`** enthalten. `torch.cuda.is_available() == True` allein genügt **nicht** — bei fehlendem sm_120 schlägt erst der eigentliche Kernel-Aufruf mit *„no kernel image is available for execution on the device"* fehl.
    4. **Smoke-Test:** eine echte GPU-Rechnung (Matmul) ausführen, nicht nur Verfügbarkeit abfragen.
    5. **Durchsatztest:** ResNet18 Forward+Backward mit realistischer Batch-Größe messen (Schrittzeit, Bilder/s, Peak-VRAM) und auf zwei Kamerastreams plus Denoising-Kopf hochrechnen (Faktor ~2–3).
  * **Zu prüfen: Reicht die 5070 Ti für das Training?** Die 16 GB VRAM sind für eine CNN-Diffusion-Policy mit zwei ResNet18-Backbones bei moderater Bildauflösung (z. B. 224 × 224 bzw. 240 × 320) **unkritisch** — das ursprünglich angesetzte Limit von $\le 12\text{ GB}$ wird damit eingehalten. Limitierend sind eher **Trainingsdauer und Datendurchsatz** (Video-Decoding im DataLoader) als der Speicher. Stellschrauben, falls es zu langsam wird: Batch-Größe, Mixed Precision (bf16/AMP), Bildauflösung, Anzahl der Kamerastreams (siehe 1.4). Ein realistischer Durchsatztest ist früh mit einem kleinen Pilot-Datensatz durchzuführen.
* **Welches Betriebssystem wird genutzt?**
  **Windows**, da der Steuerungs-Laptop unter Windows läuft.
  * **Risiko:** LeRobot wird primär unter Linux entwickelt und getestet. Erfahrungsgemäß sind Video-Encoding/-Decoding (ffmpeg, `torchcodec`/PyAV) und das DataLoader-Multiprocessing unter Windows die Reibungspunkte.
  * **Rückfallebene:** Training unter **WSL2/Linux** auf demselben Datensatz. Die **Kameraaufzeichnung muss dabei nativ unter Windows laufen**, da USB3-Kameras in WSL2 nur über USB/IP-Passthrough und mit Bandbreiten-/Latenzverlust erreichbar sind. Daraus ergäbe sich eine saubere Trennung: **Aufzeichnung und Live-Inferenz nativ unter Windows, Training wahlweise Windows oder WSL2.** Die Windows-Lauffähigkeit der Aufzeichnungs- und Inferenzkette ist damit zwingend, die des Trainings nur optional.

### 3.2 Modell-Architektur & Hyperparameter
* **Welcher Typ der Diffusion Policy soll primär evaluiert werden?**
  Fokus liegt auf einer CNN-basierten Diffusion Policy mit kompakten Bildnetzwerken (z. B. ResNet18 pro Kamera) zur Minimierung der Hardwareanforderungen.
* **Sollen vortrainierte Gewichte für die Bildnetzwerke (Backbones) genutzt werden?**
  Ja, standardmäßig werden auf ImageNet vortrainierte Gewichte verwendet, um die Trainingszeit auf lokaler Hardware zu verkürzen.
* **Wie hoch ist das maximale Zeitbudget für einen Trainingslauf?**
  Ziel ist ein effizientes, lokales Training, das innerhalb weniger Stunden auf einer Single-GPU konvergiert.

---

## AP 4: Live-Inferenz & Neura-Anbindung

### 4.1 Latenz- & Performance-Ziele
* **Welche maximale End-to-End-Latenz ist zulässig?**
  Zielgröße ist eine geschlossene Regelschleife von **15 Hz**, identisch zur Aufzeichnungsrate (siehe 1.3). Da eine Diffusion Policy pro Vorhersage **mehrere Denoising-Schritte** benötigt, wird die Anforderung **nicht** als reine Einzel-Inferenz-Latenz formuliert, sondern über **Action Chunking**: Das Modell prädiziert einen Aktions-Horizont (z. B. 8–16 Schritte), von dem nur die ersten $n$ Schritte offen ausgeführt werden, bevor neu prädiziert wird. Der Kompromiss ist dabei explizit: ein größeres $n$ entlastet die GPU, verzögert aber die Reaktion auf Störungen (relevant für das Hand-Szenario in AP 5.1).
  Zu spezifizieren und **zu messen** sind: (a) Dauer eines vollständigen Denoising-Durchlaufs auf der Ziel-Hardware, (b) Chunk-Länge und ausgeführter Anteil $n$, (c) der **Roundtrip der NeuraPy-TCP-Verbindung zur Control-Box**, der additiv hinzukommt und nicht vernachlässigt werden darf. Richtwert für die Gesamtreaktionszeit weiterhin $< 100\text{ ms}$.
* **Wird TensorRT oder ONNX Runtime zur Inferenz-Beschleunigung eingeplant?**
  Falls die reine PyTorch-Inferenz auf der lokalen GPU zu langsam ist, wird eine Konvertierung nach ONNX oder TensorRT als Optimierungsschritt eingeplant.

### 4.2 Sicherheit & Fail-Safe-Mechanismen
* **Wie wird ein Software-Not-Aus realisiert?**
  **Geklärt:** NeuraPy stellt `stop`, `pause` und `power_off` bereit; zusätzlich existiert ein separates Terminate-Skript (socketio). Diese Pfade werden direkt in die Python-Inferenzschleife eingebunden und erzwingen bei unvorhergesehenen Trajektorien sofort den Stopp. Der Auslöser läuft in einem **eigenen Watchdog-Thread**, damit eine blockierende oder verzögerte Policy-Inferenz den Stopp nicht verzögern kann. Beim Beenden ist zudem das Servo-Interface sauber zu deaktivieren (`deactivate_servo_interface`, `stop_movelinear_online`, siehe 1.5.1).
  **Wichtig:** Dies ist ein reiner **Software-Stopp** und ersetzt **keinen zertifizierten Hardware-Not-Aus**. Ein physischer Not-Halt muss während aller Aufzeichnungs- und Inferenzläufe unabhängig davon jederzeit in Reichweite und wirksam sein.
* **Gibt es virtuelle Schutzräume (Workspace Boundaries / Geofencing)?**
  **Geklärt:** NeuraPy bietet **keine** Funktion zur Definition von 3D-Schutzräumen. Der vollständige Funktionsindex der API enthält kein `workspace_boundary`/`geofence` o. ä.; verfügbar sind nur `set_safety(bool)` (globaler Ein/Aus-Schalter, keine Geometrie), `lock_cartesian_axes`/`lock_joint_axes` (sperrt ganze Achsen, keine Zonen) sowie die generische `enable_collision_detection`/Reflex-Überwachung. Schutzräume gegen Tisch/Kiste müssen also **vollständig im Python-Skript** abgefangen werden.
  **Synergie mit AP 2.4:** Es handelt sich um dieselbe Aufgabe wie die Kollisions-Vorabprüfung bei der Rauscheinspielung — das dort ohnehin zu bauende Quader-Kollisionsmodell (Tisch/Kiste + Stützpunkte entlang des Arms via `compute_forward_kinematics`) wird für die Inferenz als **Online-Geofencing-Wächter** wiederverwendet: ein separater Thread prüft laufend die aktuelle TCP-/Gelenkpose gegen die Quader und löst bei Grenzverletzung denselben Not-Halt-Pfad aus wie in AP 4.2 beschrieben.

---

## AP 5: KI-Evaluation & Benchmark

### 5.1 Benchmark-Szenarien & Metriken
* **Wie genau ist der "klassische, starre Ablauf" implementiert?**
  Das Vergleichsteam nutzt eine klassische Computer-Vision-Pipeline zur Lage- und Orientierungserkennung des Objekts. Basierend darauf wird das Koordinatensystem (KS) rechnerisch verschoben und gedreht. Die Anfahrt und der Greifprozess selbst erfolgen starr über vordefinierte, eingeteachte Posen.
* **Welche Störgrößen sollen systematisch getestet werden?**
  * *Lichtverhältnisse:* Ja, künstliche Schatten und wechselnde Tageslichteinflüsse werden getestet.
  * *Objektvarianz:* In der ersten Phase werden ausschließlich die Positionsverschiebung und die Rotation desselben Objekts getestet. Eine Varianz in Geometrie oder Textur folgt ggf. in späteren Phasen.
  * *Dynamische Hindernisse / Umgebung:* Das Dazwischengreifen einer Hand während der Ausführung ist ein wichtiges Testszenario für die dynamische Anpassungsfähigkeit. Zudem wird das Zielobjekt in einer Kiste platziert, um das Greifen unter eingeschränkten Platzverhältnissen und potenziellen Verdeckungen zu benchmarken.
  * *Verschiebung der Szenenkamera:* Die Top-View-Kamera wird gezielt um definierte Beträge verschoben/verdreht (z. B. ± 2 cm / ± 5 cm, ± 5° / ± 10°), um die Robustheit gegenüber Extrinsik-Änderungen zu quantifizieren (siehe AP 1.4). Ergebnis dieses Tests ist die Toleranzgrenze, bis zu der die Policy ohne Neuaufnahme betrieben werden darf — und damit die Antwort auf die Frage, wie steif die Montage in der Praxis sein muss.
* **Welche Ablationen werden gefahren?**
  * *Wrist+Top vs. Wrist-only:* Beide Varianten werden auf **demselben** Datensatz trainiert und verglichen (Erfolgsrate, Robustheit gegenüber Objektverschiebung, Verhalten beim Greifen aus der Kiste). Damit wird entschieden, ob die Top-View-Kamera — und mit ihr das Extrinsik-Risiko aus AP 1.4 — im Produktivbetrieb überhaupt benötigt wird.
  * *Mit/ohne Rausch-Augmentierung:* Vergleich einer Policy auf rein idealen Trajektorien gegen eine mit Noise Injection (AP 2.4), um den Nutzen der Methodik quantitativ zu belegen.
* **Wie viele Test-Trials pro Szenario werden für die quantitative Evaluation vereinbart?**
  Aktuell noch nicht exakt geplant, muss statistisch sinnvoll bemessen werden (z. B. 20–30 Durchläufe).
* **Vergleichbarkeit der Zykluszeit (Einschränkung):**
  Die Ausführungsgeschwindigkeit der Policy ist durch die Aufzeichnungsgeschwindigkeit festgelegt (siehe 2.6). Läuft der klassische Ablauf in Produktivgeschwindigkeit und die Policy im Teach-Tempo, ist ein Zykluszeit-Vergleich **nicht aussagekräftig**. Entweder werden beide Verfahren auf vergleichbare Geschwindigkeit eingestellt, oder die Zykluszeit wird ausdrücklich **nicht** als Vergleichsmetrik geführt und die Bewertung auf **Erfolgsrate und Robustheit gegenüber Störgrößen** beschränkt. Diese Festlegung ist vor der Datenaufzeichnung zu treffen, da sie die Aufzeichnungsgeschwindigkeit bestimmt.
* **Langfristige Datennutzung:**
  Es ist geplant, die stabilen Daten und Trajektorien aus dem klassischen, CV-basierten Ablauf von Felix und Joshi langfristig als zusätzliche Demonstrationsdaten (Demo-Daten) für das Training der KI zu verwenden.
  **Voraussetzung:** Damit die Daten ohne Nacharbeit nutzbar sind, müssen sie in **identischem Format** vorliegen — gleicher State-/Action-Raum (Gelenkwinkel als Action, Gelenkwinkel + TCP-Pose als State, binärer Greifer), gleiche Aufzeichnungsrate (15 Hz), gleiche Kameraperspektiven inkl. identischer Homographie-Rektifizierung sowie gleiche Zeitstempel-Konvention.
  * **Geschwindigkeit: unkritisch.** Der klassische Ablauf verfährt den Roboter mit derselben Geschwindigkeit, die auch bei der Noise Injection verwendet wird. Die in AP 2.6 beschriebene Tempo-Inkonsistenz zwischen Datenquellen entfällt damit.
  * **Verbleibendes Risiko: die Aufzeichnung selbst, nicht die Bewegung.** Sicherzustellen ist, dass die Demonstrationen mit **demselben Recorder** (gleiche Synchronisation, gleiche Kamerakonfiguration, gleiche Rektifizierung, gleiche Label-Konvention inkl. Greifer-Dwell) mitgeschnitten werden. Der einfachste und sicherste Weg ist, das Aufzeichnungsmodul aus AP 1.3/2.2 unverändert im klassischen Ablauf mitlaufen zu lassen, statt ein zweites Aufzeichnungsformat zu pflegen.
  * **Inhaltlicher Unterschied (bewusst):** Der klassische Ablauf liefert **saubere Trajektorien ohne Rauschen**, also keine Korrekturbeispiele. Er ergänzt den Noise-Injection-Datensatz um zusätzliche Objektpositionen und Varianz, ersetzt dessen Korrektursignal aber nicht.

### 5.2 Episoden-Handling & Datenqualität
Für Aufzeichnung und Evaluation gleichermaßen zu definieren:
* **Reset-Prozedur:** Definierter Ablauf zwischen zwei Episoden (Rückfahrt in eine feste Home-Pose, Neuplatzierung des Objekts, Zurücksetzen des Greifers). Die Home-Pose muss reproduzierbar sein, da die Policy den Episodenstart implizit lernt.
* **Erfolgs-/Misserfolgs-Label:** Jede Episode wird als erfolgreich/fehlgeschlagen markiert (manuell über das Dashboard, AP 1.2). Fehlgeschlagene Demonstrationen dürfen nicht unkommentiert ins Training einfließen.
* **Verwerfen fehlerhafter Episoden:** Episoden mit ausgelöstem Schutzstopp (AP 2.4), überschrittenem Synchronisations-Latenzbudget (AP 1.3) oder erkannter Kameraverschiebung (AP 1.4) werden verworfen bzw. markiert.
* **Metadaten pro Episode:** Session-/Block-ID, Kamerapose-Variante, Objektposition, Beleuchtungssituation und Aufnahmemodus (automatisiert mit Rauschen / Gamepad / klassischer CV-Ablauf). Ohne diese Metadaten sind die Ablationen in 5.1 nachträglich nicht auswertbar.

# Laptop — Inbetriebnahme und Befunde

Stand: 2026-10-02 (Abschnitt 7 = Labortag 2026-10-01; 6 = Pipeline-Abnahme
2026-09-28; 1–5 vom 2026-09-24)
· Aufnahme-Laptop `LAPPI` (Windows 11 Home 26200, HP, Intel i7-1255U, kein CUDA)
· Gegenstück zu `../../70_BehaviorCloning/Sim-Inbetriebnahme-Befunde.md` (PC).
**Alle Zahlen stammen vom Laptop und aus der VM, nicht von der Anlage.**

> **Was davon heute noch gilt (2026-10-02).** Die Abschnitte 1–6 sind ein
> Protokoll und bleiben als solches stehen. Überholt ist:
>
> | Abschnitt | damals | heute |
> |---|---|---|
> | 1 | Testsuite 137, Arbeitsbaum sauber | **271** Tests; Interpreter `C:\Enviroments\bc_env` (Python 3.11.9) mit PySide6, gxipy, pygame — `torch`, `lerobot`, `datasets`, `av` fehlen weiterhin (Export/Training nicht auf diesem Rechner) |
> | 2 | Punkte fehlen in der VM | **erledigt** — Punktdatenbank vollständig (6.1) |
> | 3 | lesende Werkzeuge scheitern im Teach-Modus | **erledigt 2026-10-02** — `connect()` ruft `init_program()` nur noch bei Bewegungsvorbereitung (Punkt 1 in 5) |
> | 3.3 | `reset_control()` als Ausweg | gilt weiter; der Adapter **nennt jetzt selbst** die Ursache eines abgelehnten `init_program()` (Fehlerzustand / Teach-Modus / laufendes Programm) |
> | 4.2 | Daheng nicht enumeriert | **erledigt** — Kabel/Port; am 2026-10-01 lieferte sie echte Bilder |
> | 6.4 | 15-Hz-Takt wird nicht gehalten | **Ursache gefunden: Host-Leistung** — siehe 7.1 |
>
> Weiter offen sind die Punkte 4–6 der Liste in Abschnitt 5 und die
> Szenenkamera (6.5).

---

## 1. Was auf diesem Rechner funktioniert

| | Stand |
|---|---|
| Repo | Branch `BehaviorCloning`, HEAD `082ca3b` "Kameranbindung und Tests", Arbeitsbaum sauber |
| Hardwarefreie Testsuite | **137/137 grün** (`tests/run_all.py`) |
| VM `Lara5_V5` | läuft, erreichbar, `192.168.2.13:65432` und `:8080` offen |
| `is_robot_in_simulation()` | **antwortet hier mit `True`** (am PC lief sie in den Timeout) |
| Punktdatenbank der VM | nur `Home` und `Parking` — **alle geteachten Punkte fehlen** |
| USB-Kamera (UVC) | erkannt und liefert Bilder, aber **~10 fps über den Adapter** |
| Daheng VEN-161 | **nicht enumeriert** — siehe 4 |
| Gamepad | als USB-Verbundgerät erkannt (`VID_248A&PID_8713`), `pygame` fehlt |

Python ist hier der System-Interpreter
`C:\Users\jerem\AppData\Local\Programs\Python\Python311\python.exe` (3.11.9).
**Das in `CLAUDE.md` genannte `E:\Enviroments\bc_env\python.exe` existiert
nicht** — der Laptop hat kein Laufwerk E:.

Vorhanden: `numpy 1.26.4`, `opencv-python 4.11.0.86`, `scipy`, `pywin32 306`,
`prettytable`. **Fehlt: `torch`, `lerobot`, `datasets`, `av`, `pygame`.**

Daraus folgt für die Apps:

| App | hier lauffähig |
|---|---|
| `apps/record.py` | **ja** (nur numpy/cv2) |
| `apps/teach.py` | nein — `pygame` fehlt (Gamepad ist angeschlossen) |
| `apps/export.py` | nein — `lerobot` fehlt (Import erfolgt verzögert in `bc/lerobot_io.py`) |
| `apps/train.py` | nein — `torch` + `lerobot` fehlen |
| `apps/infer.py` | nur `--hold`; echte Policy braucht `torch` |

Das deckt sich mit der Festlegung in `CLAUDE.md` (Aufnahme-Laptop ohne CUDA:
Aufnahme/Export ja, Policy-Fahrt nein) — **für den Export fehlt lerobot aber
trotzdem noch.**

---

## 2. Die Punkte fehlen in dieser VM (Hauptbefund)

`sequences/pick_to_station.json` holt seine Punkte **live aus der
Datenbank der Steuerung** (`bc/adapters/neura.py:get_point`), nicht aus einer
Datei im Repo. Abfrage gegen die laufende VM:

```
Punkte in der Steuerung (2):  Home, Parking

CLEAR_FOV      FEHLT
APPROACH_01    FEHLT
APPROACH_02    FEHLT (optional)
PRE_GRASP      FEHLT
PICK           FEHLT
PRE_PLACE      FEHLT
```

Die VM auf diesem Laptop ist offenbar ein **frischer OVA-Import**; die Punkte
wurden am PC im Teach-Pendant geteacht und liegen damit **nur in der
Festplatte der dortigen VM**. Im Repo stehen sie nirgends: `data_vm/` ist
gitignored und hier nicht vorhanden, und `apps/teach.py` schreibt in eine
lokale `waypoints.json` (ebenfalls gitignored) — ein **anderer** Speicherort
als die Punktdatenbank der Steuerung.

**Es gibt keinen Weg, die Punkte ohne die PC-VM zu rekonstruieren.**

### 2.1 Werkzeuge gebaut — mit einem wichtigen Vorbehalt

`tools/export_points.py` (nur lesen) und `tools/import_points.py` (schreibt
nur mit `--write`) sind da und gegen die VM erprobt.

Die Aufrufform von `create_point` war nirgends dokumentiert — `neurapy`
reicht Aufrufe nur durch und kennt keine Signaturen. Am 2026-09-24 gegen die
VM bestimmt:

```
create_point(name, reference_frame, kartesische_pose)
    reference_frame :  World | Base | Tool   (andere -> "No reference frame found")
    Position 3      :  [X, Y, Z, RX, RY, RZ] in Meter/Radiant (RPY)
```

Gelenkwinkel an Position 3 scheitern mit „Exception: Inverse Kinemat..." —
**die Steuerung rechnet die Gelenkstellung selbst per IK aus.**

**Und genau das ist das Problem.** Die Gelenkstellung legt den
IK-Lösungszweig fest und dient später als Seed (siehe Docstring von
`NeuraRobot.get_point`). Beim Import wird sie neu bestimmt. `import_points.py`
liest deshalb jeden Punkt zurück und vergleicht — im Test mit den beiden
vorhandenen Punkten:

| Punkt | Abweichung der Gelenkwinkel |
|---|---|
| `Home` | 0.000000 rad — Zweig erhalten |
| `Parking` | **4.9916 rad — anderer Zweig** |

Einer von zwei Punkten landete in einer **anderen Armkonfiguration**.
Geometrisch steht der TCP richtig, angefahren wird er anders. Für
Wegpunkte einer aufgezeichneten Bahn ist das nicht hinnehmbar.

### Empfohlene Wege (in dieser Reihenfolge)

1. **VM-Abbild vom PC kopieren.** Nach dem Befund oben der **einzige Weg,
   der die Punkte unverändert überträgt** — inklusive Lösungszweig, Tool und
   Einstellungen. Am PC `Lara5_V5` sauber herunterfahren, in VirtualBox
   *Datei → Appliance exportieren* (OVA), auf dem Laptop importieren, das
   Host-only-Netz wieder auf `192.168.2.13` setzen.
2. **Export/Import per Werkzeug** — als Sicherung und als Beleg, gegen
   welche Punkte aufgezeichnet wurde, uneingeschränkt sinnvoll. Zum
   Übertragen nur mit Kontrolle der Meldung „ABWEICHEND"; betroffene Punkte
   müssen nachgeteacht werden.
3. **Neu teachen.** Langfristig ohnehin der Normalfall, weil am echten
   Roboter geteacht wird. Die Zahlen aus `Berichte/2026-09-1x_*.pdf` sind
   danach nicht mehr vergleichbar.

---

## 3. Die VM steht im Teach-Modus — und das blockiert auch lesende Werkzeuge

`tools/check_sim_robot.py` und `tools/check_kinematics.py` brechen ab mit:

```
Exception: Unable to switch to play mode. Check if robot in automatic mode
```

Ursache ist nicht der Modus allein, sondern die Reihenfolge in
`bc/adapters/neura.py:connect()` (Zeile 159 ff.):

```python
self._call("init_program")                      # braucht Automatikmodus
if power_on: ...
if ensure_automatic and ...: switch_to_automatic_mode()   # kommt zu spät
```

`init_program()` wird **unbedingt** aufgerufen, auch bei
`power_on=False, ensure_automatic=False`. Damit scheitern ausgerechnet die
Werkzeuge, die laut ihrem eigenen Docstring **nichts bewegen und nur lesen**
(`check_kinematics.py`: „NUR LESEN UND RECHNEN"). Zusätzlich steht der
Automatik-Wechsel *hinter* dem Aufruf, der ihn voraussetzt — mit
`ensure_automatic=True` aus dem Teach-Modus heraus greift er also nie.

**Zwei Anpassungen, unabhängig voneinander:**

* `init_program()` nur aufrufen, wenn tatsächlich Bewegung vorbereitet wird
  (`power_on or ensure_automatic`) — dann laufen die lesenden Werkzeuge in
  jedem Modus.
* Den Automatik-Wechsel **vor** `init_program()` ziehen.

Bis dahin: im Teach-Pendant (`http://192.168.2.13:8080`) auf **Automatik**
umschalten — und zwar **dort**, nicht über RPC (siehe 3.1).

### 3.1 Der eigentliche Blocker war der RCSC-Fehlerzustand

Zwischenzeitlich sah es so aus, als sei `switch_to_automatic_mode()` über
RPC wirkungslos (`is_robot_in_teach_mode()` meldete `False`, `init_program()`
scheiterte trotzdem weiter mit „Check if robot in automatic mode"). **Das
war ein Trugschluss.** Nach `reset_control()` (siehe 3.3) lief dieselbe
Folge sauber durch:

```
switch_to_automatic_mode        156 ms  -> True
is_robot_in_teach_mode          135 ms  -> False
reset_errors                    148 ms  -> None
init_program                    367 ms  -> True
power_on                        191 ms  -> True
set_override                    262 ms  -> True
move_joint (Achse 1 +0.05 rad)  2508 ms -> True     -> Roboter BEWEGT sich
move_joint (zurueck)            1120 ms -> True
```

Blockiert hat also nicht der Betriebsmodus, sondern der latente
Fehlerzustand der Steuerung. Solange er anlag, war die Fehlermeldung von
`init_program()` **irreführend** — sie nennt den Modus, gemeint ist der
Fehlerzustand. Das kostet Zeit, wenn man ihr glaubt.

**Trotzdem offen für den Adapter:** `connect()` kennt `reset_errors()`
nicht. Liegt ein Fehler an, scheitert es mit einer Meldung, die auf den
Modus zeigt — und die Ursache steht woanders. Ein `reset_errors()` vor
`init_program()` (oder wenigstens ein Hinweis im Fehlertext) würde das
abkürzen.

### 3.2 Der RPC-Dispatcher kann dauerhaft blockieren

Beim ersten Lauf der Contract-Suite gegen die VM kam ein Aufruf nicht
zurück. Danach war die Steuerung **komplett blockiert**: TCP 65432 nahm
weiter an, der Verbindungs-Handshake lieferte noch die Serverversion, aber
**jeder** Funktionsaufruf lief ins Leere — auch rein lesende. Auch der
Roboter liess sich im Pendant nicht mehr verfahren. Ein Reset in der
Steuerung half nicht; erst ein **Neustart der VM** stellte sie wieder her
(danach alle Aufrufe wieder 8–470 ms).

Welcher Aufruf es war, ist **nicht geklärt**. `power_on` war der
naheliegende Verdacht, ist aber entlastet: es liefert reproduzierbar in
~150 ms `True`. Eine Reproduktion wurde nicht versucht, weil sie die
Steuerung erneut lahmlegt.

**Konsequenz für die Werkzeuge:** NeuraPy kennt kein Socket-Timeout. Ein
hängender Aufruf blockiert den Aufrufer unbegrenzt und ist von „dauert
lange" nicht unterscheidbar. Für den Anlagentag wäre ein Zeitlimit um
`_call()` sinnvoll — dort ist ein stiller Dauerhänger deutlich teurer als
in der VM.

### 3.3 `reset_control()` ist der Ausweg — nicht der weisse Taster

Die Steuerung meldet im Pendant unter anderem:

* `RCSC_105 connection interrupted, please do reset control from the PC
  option in GUI`
* `please press white reset button and try again switching to simulation mode`

Der **weisse Reset-Taster ist Hardware der realen Control-Box** — in der VM
gibt es ihn nicht. Diese Meldung führt also in die Irre und ist hier nicht
befolgbar. Gemeint und wirksam ist der andere Weg, und der ist auch über
NeuraPy erreichbar:

```
reset_control()     22 ms  -> True
reset_errors()      89 ms  -> None
reset_collision()  276 ms  -> True
get_diagnostics()          -> {'critical': False, 'issues': {}}
```

Damit verschwindet der dauerhafte `critical: True /
other_errors: ['undefined error']` **zum ersten Mal überhaupt** — er lag
über alle bisherigen Messungen an, auch in
`../../70_BehaviorCloning/Sim-Inbetriebnahme-Befunde.md`.

**`reset_control()` startet die Robotersoftware in der VM neu.** Unmittelbar
danach antworten Aufrufe mit „Cross-check whether the robot is reachable and
that the robot software is up and running", kurz darauf verweigert Port
65432 die Verbindung ganz, bis der Dienst wieder oben ist. Das ist normal
und kein Fehler — Werkzeuge müssen diese Phase aber aushalten (heute
scheitert schon `Robot()` im Konstruktor mit `ConnectionError`).

**Das ist vermutlich auch das Mittel gegen 3.2:** Der blockierte Dispatcher
liess sich bisher nur per VM-Neustart lösen. `reset_control()` ist der
kleinere Eingriff und sollte beim nächsten Hänger zuerst versucht werden —
sofern der Aufruf selbst noch durchkommt.

**Die OVA selbst ist unauffällig.** Nach dem Neustart antwortet die
Steuerung zügig, `power_on` greift, die Kinematik stimmte in den früheren
Messungen exakt (`check_kinematics.py`, 0,00 mm). Die Auffälligkeiten sind
die eines Alpha-Stands (`v5.0.0-alpha.102` gegen Client `v5.0.8`), nicht
die eines beschädigten Abbilds.

### Sonstiges aus der VM (gelesen, ohne Bewegung)

```
Server v5.0.0-alpha.102   Client v5.0.8   (bekannte Warnung)
is_robot_in_simulation()  True            <- funktioniert hier
is_robot_in_teach_mode()  True
get_selected_tool_name()  'NoTool'
get_override()            1.0
get_diagnostics()         critical=True, powered_off=False,
                          other_errors=['undefined error']
Pflichtfunktionen         alle vorhanden (207 Funktionen)
```

`powered_off` ist hier **False** (am PC war es `True`) und
`is_robot_in_simulation()` antwortet, statt in den Timeout zu laufen. Der
offene Punkt 5 aus `Sim-Inbetriebnahme-Befunde.md` („Ersatz für
`is_robot_in_simulation()` festlegen") ist damit auf diesem Stand **nicht
mehr akut** — die Sicherheitssperre im Adapter greift wie vorgesehen.

---

## 4. Kameras

### 4.1 Angeschlossen und nutzbar: eine USB-Webcam, aber am falschen Backend

Am Rechner hängen drei Kameras. Eindeutig zugeordnet über ihre maximale
Auflösung und Schnappschüsse:

| OpenCV-Index (DSHOW) | Gerät | max. Auflösung |
|---|---|---|
| 0 | HP Wide Vision HD Camera (eingebaut) | 1280 × 720 |
| 1 | Sony Camera (Imaging Edge) — **virtuell**, keine echte Kamera | 1024 × 576 |
| 2 | **angeschlossene USB-Webcam** (`VID_046D&PID_09A4`, Logitech) | 640 × 480 |

**Achtung: die Indizes sind je Backend verschieden.** Unter `CAP_MSMF` ist
die Sony-Virtuelle die 0 und die eingebaute Kamera die 1. Der in `CLAUDE.md`
genannte Aufruf `--uvc-device 1` trifft auf diesem Laptop **die falsche
Kamera** (die virtuelle Sony). Richtig ist hier `--uvc-device 2`.

**Treiber sind in Ordnung** — die Kamera läuft am generischen
Windows-UVC-Treiber, Status `OK`, kein Windows-11-Problem. Das vermutete
Treiberproblem liegt woanders (4.2).

**Die Bildrate ist das Problem, und sie ist hausgemacht.** `bc/adapters/cam_uvc.py`
öffnet mit `cv2.CAP_DSHOW` (Kommentar: „vermeidet die langsame
MSMF-Initialisierung"). Dieselbe Kamera, dieselbe Auflösung 320 × 240,
8 s Dauermessung:

| Backend | Rate | Bildabstand median / p95 |
|---|---|---|
| `CAP_DSHOW` (heute im Adapter) | **10.1 fps** | 96 ms / 112 ms |
| `CAP_MSMF` | **27.6 fps** | 2 ms / 105 ms |
| Default (Backend-Wahl durch OpenCV) | **29.6 fps** | 2 ms / 123 ms |

Über den Projekt-Adapter gemessen (`check_cameras.py --backend uvc
--device 2`): **8–10 fps, Bildalter bis 108 ms** — der Recorder verwirft das
zu Recht gegen `SYNC_MAX_SKEW_S = 30 ms`. Das ist genau der in `CLAUDE.md`
notierte Befund „Webcam am Entwicklungsrechner 9-11 fps"; die Ursache ist
**nicht die Webcam, sondern die Backend-Wahl**.

**Anpassung:** Backend in `CameraConfig` konfigurierbar machen (oder beim
Öffnen DSHOW gegen MSMF messen und das schnellere nehmen). Der Median von
2 ms bei MSMF zeigt allerdings gepufferte Auslieferung in Schüben —
`CAP_PROP_BUFFERSIZE` greift dort nicht. Nach der Umstellung ist deshalb
**das Bildalter** neu zu messen, nicht nur die Rate.

### 4.2 Daheng VEN-161: nicht enumeriert — vermutlich Kabel/Port, nicht Windows 11

* Galaxy SDK **ist hier installiert**: `C:\Program Files\Daheng Imaging\GalaxySDK`,
  Version 2.6.2608.9131 (am PC laut `CLAUDE.md` nicht installiert).
* Der USB-Treiber liegt im Treiberspeicher: `oem311.inf` ← `gxusbbase.inf`,
  Geräteklasse `USBCamera`.
* **Die Geräteklasse `USBCamera` ist leer** — es ist keine Daheng-Kamera
  enumeriert. `gxipy` meldet folgerichtig „keine Daheng-Kamera gefunden",
  `tools/check_daheng_api.py --open` ist damit nicht ausführbar.
* Es gibt aber **ein USB-Gerät im Fehlerzustand**:

```
Unbekanntes USB-Gerät (Fehler beim Zurücksetzen des Ports.)
InstanceId : USB\VID_0000&PID_0001\5&C38CF22&0&14
Problem    : CM_PROB_FAILED_POST_START
Ort        : Port_#0014.Hub_#0002  (Root-Hub USB 3.0)
```

`VID_0000&PID_0001` bedeutet, dass Windows **die Gerätedeskriptoren nie
lesen konnte** — das Gerät hat die USB-Enumeration nicht abgeschlossen. Ein
Treiber kann deshalb gar nicht greifen, unabhängig davon, welcher installiert
ist. **Das ist kein Windows-11-Treiberproblem**, sondern der klassische Befund
bei USB3-Vision-Kameras für Kabel, Port oder Stromversorgung.

Ob dieses Gerät die Daheng ist, **konnte ich nicht beweisen** — es meldet
keine Kennung. Es ist aber das einzige nicht enumerierte USB-Gerät, und alle
anderen angeschlossenen Geräte (beide Webcams, Gamepad, Bluetooth) sind `OK`.

Zu prüfen, in dieser Reihenfolge:

1. Kamera **ab- und wieder anstecken**, möglichst an einen anderen
   USB-3-Port (blau / SS-Logo), direkt am Laptop, **ohne Hub**.
2. Das **mitgelieferte USB3-Kabel** verwenden — ein USB-2-Kabel oder eine
   Verlängerung reicht für die VEN-161 nicht.
3. Gegenprobe außerhalb von Python: Galaxy Viewer bzw. `GxUpgradeTool` aus
   `C:\Program Files\Daheng Imaging\GalaxySDK\...`. Sieht der die Kamera
   nicht, ist es sicher kein Python-/`gxipy`-Problem.
4. Erst danach: `python tools/check_daheng_api.py --open` (nur lesend).

---

## 4a. Testergebnisse 2026-09-24

| Suite | Ergebnis |
|---|---|
| `run_all.py` (hardwarefrei) | **137/137** |
| `run_all.py --camera=uvc --uvc-device=2` gegen die Logitech | **5/5** |
| `run_all.py --robot=neura contract.test_robot_contract` gegen die VM | **11/12** |

Der eine Ausfall, `test_fk_matches_reported_pose`, ist **kein
Kinematikfehler**. Er scheiterte in der *Fixture*:

```
_fixtures.make_robot -> NeuraRobot().connect(power_on=True)
  -> neura.py:159  self._call("init_program")
     Exception: Unable to switch to play mode. Check if robot in automatic mode
```

Betroffen war der Test, der **unmittelbar auf
`test_emergency_stop_contract` folgt** — der nächste Test danach lief wieder
durch. Die Steuerung ist also nach einem Not-Halt kurzzeitig nicht
initialisierbar.

Ein gezielter Nachbau (Not-Halt → `clear_stop()` → `init_program()` →
zweites `connect()`) lief **fehlerfrei durch**, ist also nicht
deterministisch reproduzierbar. Es sieht nach einem **Zeitverhalten** aus:
`stop()` wirkt asynchron, `clear_stop()` setzt direkt danach
`init_program()` ab, und in der Suite folgt das nächste `connect()`
Sekundenbruchteile später.

Erschwerend: `clear_stop()` ruft `init_program()` über `_call_safe()` auf —
**ein Fehler dort wird verschluckt**. Die Software hält den Stopp für
quittiert, der Controller nimmt aber keine Bewegung an. An der Anlage ist
das der Ablauf „Bediener quittiert Not-Halt, nächster Fahrbefehl geht ins
Leere". Das gehört geprüft, bevor dort Not-Halt-Pfade gefahren werden
(AP 4.2, Abnahmeliste Punkt 10).

## 5. Was als Nächstes zu tun ist

**Durch den Anwender:**

1. Entscheiden, wie die Punkte hierher kommen (Abschnitt 2 — nach 2.1
   spricht alles für das VM-Abbild).
2. Daheng-Kamera nach 4.2 prüfen (Kabel/Port), sobald sie wieder verfügbar
   ist. Bis dahin bleibt die Logitech der Ersatz.

**Erledigt am 2026-09-24:**

| Was | Wo |
|---|---|
| `--uvc-device=N` für die Contract-Suite (Index ist je Rechner und Backend verschieden) | `tests/run_all.py`, `tests/conftest.py`, `tests/_fixtures.py` |
| Punkte sichern und zurückspielen, mit Rückprobe des IK-Zweigs | `tools/export_points.py`, `tools/import_points.py` |

**Im Code anzupassen (noch nicht gemacht):**

| Nr. | Was | Wo |
|---|---|---|
| 1 | ~~`init_program()` nur bei Bewegungsvorbereitung~~ — **erledigt 2026-10-02**: lesend verbunden kein `init_program()`, beim ersten Bewegungsbefehl nachgeholt; Automatik-Wechsel steht jetzt davor. Damit verbindet auch die GUI im Teach-Modus | `bc/adapters/neura.py` |
| 2 | ~~`reset_errors()` vor `init_program()`~~ — **erledigt 2026-09-28** durch `_init_program()` mit Wiederholung, siehe 6.3 | `bc/adapters/neura.py` |
| 3 | ~~`clear_stop()` verschluckt das Scheitern von `init_program()`~~ — **erledigt 2026-09-28**, siehe 6.3 | `bc/adapters/neura.py` |
| 4 | Zeitlimit um `_call()`: NeuraPy hat kein Socket-Timeout, ein Hänger ist von „dauert lange" nicht unterscheidbar | `bc/adapters/neura.py:231` |
| 5 | Kamera-Backend konfigurierbar (DSHOW → MSMF), danach Bildalter neu messen | `bc/adapters/cam_uvc.py:19`, `bc/config.py` |
| 6 | `--no-display` ohne `--seconds` läuft endlos (kein Fenster zum Beenden) | `tools/check_cameras.py:143` |
| 7 | ~~Interpreterpfad und `--uvc-device 1` stimmen auf diesem Rechner nicht~~ — **kein Fehler**: `CLAUDE.md` beschreibt den PC (Laufwerk `E:`), der Laptop-Pfad steht in `Environment-Laptop.md` | `CLAUDE.md` Abschnitt 2 |

**Umgebung (Stand 2026-10-02):** `C:\Enviroments\bc_env` ist eingerichtet
(Python 3.11.9, PySide6, gxipy, pygame, OpenCV 4.13). Es fehlen weiterhin
`torch`, `lerobot`, `datasets`, `av` — Aufnahme und Teach gehen, Export und
Training nicht. `lerobot` verlangt Python 3.12 (`Environment-Laptop.md`).
OpenCV 4.13 lässt `test_rectify::test_homography_recovers_marker_positions`
scheitern (ArUco bei 240×320) — bekannt, nicht durch Pinnen zudecken.

---

## 6. Pipeline-Abnahme für die Datenaufnahme (2026-09-28)

Ausgangslage: VM läuft, Punkte sind angelegt, Roboter in Automatik. Geprüft
wurde die Frage „steht die Kette, um Daten aufzunehmen?" — **kein Training,
kein Export.** Die vorhandenen Punkte wurden behandelt, als seien sie gerade
am realen Roboter geteacht worden.

### 6.1 Ergebnis: ja, die Kette steht

| Prüfung | Ergebnis |
|---|---|
| `run_all.py` (hardwarefrei) | **137/137** |
| Kamera-Contract, Logitech (Index 2) | **5/5** |
| Kamera-Contract, eingebaute Kamera (Index 0) | **5/5** |
| Roboter-Contract gegen die VM | **12/12** (vorher 11/12) |
| `check_daheng_api.py` gegen gxipy 2.6 | **34 in Ordnung, 0 Fehler** |
| Aufzeichnung mit Platzhalter-Kameras, 2 Episoden | **2 verwendbar, 0 verworfen** |
| Aufzeichnung mit der Logitech, 3 Episoden | 0 verwendbar — **nur** Latenzbudget |

Die Punktdatenbank enthält `CLEAR_FOV`, `APPROACH_01`, `PRE_GRASP`, `PICK`,
`PRE_PLACE` (+ `Home`, `Parking`). `APPROACH_02` fehlt, ist in
`pick_to_station.json` aber als `optional` markiert und wird sauber
übersprungen.

### 6.2 Die Rauscheinspielung tut, was sie soll

Gemessen an den aufgezeichneten Arrays (`aux.pose_ideal` gegen
`aux.pose_noisy`, TCP-Abstand):

| | ep_00000 | ep_00001 |
|---|---|---|
| Rauschen Mittel / Max | 7,5 mm / 22,0 mm | 10,1 mm / 27,9 mm |
| Abstand an Start und Ende | 0,00 mm | 0,00 mm |
| `max‖action[t] − joints_ideal[t+1]‖` | **0.000000 rad** | **0.000000 rad** |
| `max‖action[t] − state_joints[t+1]‖` | 0,083 rad | 0,159 rad |

Damit sind die drei Kernzusagen aus `CLAUDE.md` Abschnitt 5 am Datensatz
belegt: Amplitude im Korridor ±1–2 cm, **Trichter-Dämpfung an den Ankern**
(exakt 0 mm an Start und Ende), und **Label = idealer Schritt t+1**, während
die Observation den verrauschten Ist-Zustand trägt. Der Rauschfaktor wird je
Episode gezogen (0,595 bzw. ein anderer Wert) — die beiden Bahnen
unterscheiden sich um im Mittel 4,3 mm, sind also nicht identisch.

Schema wie festgelegt: `observation.state` (218, **14**), `action`
(218, **7**), Bilder (218, **240, 320, 3**), Schema v2, 15 Hz, dazu
`aux.joints_ideal/joints_command/pose_ideal/pose_noisy/sync_ok` und
`next.done`. Metadaten nach AP 5.2 vollständig: `block`, `session`,
`object_note`, `object_pose` (PICK + PRE_GRASP), alle geteachten Punkte mit
Gelenkwinkeln und Pose, Planer- und Rauschparameter, Seed, Greiferprotokoll.

### 6.3 Behoben: sporadischer Abbruch beim Verbinden

`init_program()` wurde sporadisch abgelehnt — mit der Meldung „Unable to
switch to play mode. Check if robot in automatic mode", obwohl der Modus
nachweislich Automatik war (`is_robot_in_teach_mode() == False`,
`get_diagnostics()` ohne Fehler, `program_status == 'RUNNING'`).

**Ursache:** `stop()` wirkt verzögert. Wer direkt danach `init_program()`
ruft, wird abgewiesen; Sekunden später greift derselbe Aufruf. Genau das
passiert in der Contract-Suite (jeder Test schließt mit `stop()`, der
nächste verbindet sofort) und zwischen zwei Aufzeichnungsläufen.

Die Meldung ist **irreführend** — sie nennt den Betriebsmodus, gemeint ist
das noch nicht beendete Programm. Das hat die Diagnose zweimal in die
falsche Richtung geschickt.

**Wirkung vorher:** In der Contract-Suite fiel je Lauf genau ein
*beliebiger* Test aus (2026-09-24 `test_fk_matches_reported_pose`,
2026-09-28 `test_gripper_command_is_nonblocking_and_tracked`) — deshalb sah
es zunächst nach einem Zusammenhang mit dem Not-Halt-Test aus, was falsch
war. Und `apps/record.py` brach **vor der ersten Episode** ab.

**Behoben** in `bc/adapters/neura.py`: `_init_program()` wiederholt den
Aufruf (6 x 1 s) und wirft danach mit einer Meldung, die auf
`program_status()`/`get_diagnostics()` und auf `reset_errors()` bzw.
`reset_control()` verweist. `connect()` und `clear_stop()` benutzen es.
`clear_stop()` rief `init_program()` vorher über `_call_safe()` auf und
**verschluckte den Fehler** — damit galt ein Not-Halt als quittiert,
während der Controller weiter jede Bewegung ablehnte (Punkt 3 der Liste in
Abschnitt 5, damit ebenfalls erledigt).

### 6.4 Was noch nicht stimmt — Takt

> **Nachtrag 2026-10-02:** Ursache gefunden — die Leistung des Laptops
> (Akkubetrieb, Energiesparmodus, VM-Laufzeit). Mit „Beste Leistung" und
> frisch gestarteter VM hält er den Takt bei 30 und 60 Hz (Zeitfaktor
> 1,01–1,04, 0 % schlechte Frames). Siehe 7.1. Die Vermutung unten, der
> Platzhalter-Pfad sei schuld, war nicht die Erklärung.

Der 15-Hz-Takt wird auf diesem Laptop **nicht gehalten**:

| Lauf | Kameras | Takt-Überläufe | schlechte Frames |
|---|---|---|---|
| `data_vm` ep0 | uvc + Platzhalter | 52/218 (24 %) | 33,5 % |
| `data_vm` ep1 | uvc + Platzhalter | 100/218 (46 %) | 45,0 % |
| `data_vm_sim` ep0 | beide Platzhalter | 169/218 (78 %) | 2,8 % |
| `data_vm_sim` ep1 | beide Platzhalter | 214/218 (98 %) | 3,2 % |

Kontraintuitiv, aber erklärbar: Platzhalter-Kameras laufen als
`DirectCapture` **synchron in der Taktschleife** (echte Kameras als
`ThreadedCapture` daneben). Die hohe Zahl im Platzhalter-Lauf ist damit ein
Artefakt des Testpfads, kein Maß für den Normalbetrieb.

Ein Überlauf verfälscht die Daten **nicht** — der Pacer holt nicht auf,
sondern nimmt den nächsten Rasterpunkt; jeder Schritt trägt seinen echten
Zeitstempel. Die Bahn wird dadurch aber langsamer abgefahren als geplant,
und die Ausführungsgeschwindigkeit wird mitgelernt (AP 2.6). Für die VM ist
das laut `Sim-Inbetriebnahme-Befunde.md` ohnehin kein Maßstab („die Sim
taugt für Formate, Semantik und Kinematik, nicht für Timing") — **an der
Anlage ist es neu zu messen** (Abnahmeliste 0.6, Punkte 1 und 3).

### 6.5 Was damit wirklich noch offen ist

1. **Daheng:** Kamera anstecken, `check_daheng_api.py --open`, dann
   `record.py --cameras wrist-real`. Die Adapter-API ist gegen die
   installierte gxipy 2.6 vollständig geprüft — offen ist nur das Öffnen
   des Geräts und der Frame-Abgriff.
2. **Punkte teachen und aufnehmen** an der Anlage.
3. Szenenkamera ist weiterhin unentschieden; in allen Läufen hier war sie
   ein Platzhalter (`SCENE_CAMERA_CONFIRMED = False`). Vor der ersten
   echten Aufzeichnung festlegen.

Alles andere in Abschnitt 5 bleibt stehen, ist für die Datenaufnahme aber
nicht blockierend.

Testdaten dieser Abnahme liegen in `data_vm/`, `data_vm_sim/` und
`data_vm_uvc2/` (gitignored) — sie sind **keine** Nutzdaten.

---

## 7. Labortag 2026-10-01 und Nachbereitung 2026-10-02

Erster Tag im Labor: Daheng am Laptop, Roboter weiterhin die VM.
Ausführlich in `Taktzeit-und-RPC-Latenz.md` und
`../../20_Dokumentation/Known-Issues-Neura-Sim.md` Punkt 4; hier das
Ergebnis.

### 7.1 Bahnabbrüche, RCSC_102/105 und „Frames über Budget" — Host-Leistung

**Symptom:** Der Arm blieb mitten in der Bahn stehen, das Pendant meldete
RCSC_102 bzw. RCSC_105, alle Episoden wurden wegen des Latenzbudgets
verworfen — auch mit Platzhalter-Kameras.

**Ursache:** der Laptop selbst. Derselbe Code-Stand lief am PC fehlerfrei
und sichtbar schneller. Der Laptop ist ein 15-W-Hybridchip (i7-1255U,
2 Performance- + 8 Effizienzkerne) mit 15,7 GB RAM, die VM belegt davon
10,9 GB und 6 vCPU; er lief zeitweise im Akkubetrieb. `servo_j` kostete
dort 22,4 ms gegen ~2,5 ms Auslegung — die Taktschleife lief 2–4× zu
langsam, und die Echtzeitschleife der Robotersoftware in der VM verfehlte
ihre Deadlines.

**Bestätigt am 2026-10-02:** VM neu gestartet, Energiemodus „Beste
Leistung" — alle Läufe erfolgreich:

| servo_j-Rate | Zeitfaktor | Überläufe je Tick | schlechte Frames |
|---|---|---|---|
| 30 Hz | 1,01 | 0,00 | 0 % |
| 60 Hz | 1,01–1,04 | 0,01–0,03 | 0 % |
| 90 Hz | **1,62** | 0,47 | 0 % |
| 120 Hz | **1,70** | 0,67 | 0 % |

Zwei Änderungen gleichzeitig (Neustart und Energiemodus) — ihr Anteil ist
einzeln nicht getrennt. Bekannt ist, dass ein VM-Neustart die Latenz
schon am Vortag halbiert hatte.

**Achtung bei 90 und 120 Hz:** „erfolgreich", aber 1,6–1,7× zu langsam
gefahren. Diese Läufe sehen gesund aus (0 % schlechte Frames, kein
Abbruch), die Ausführungsgeschwindigkeit wird aber mitgelernt (AP 2.6).
Seit 2026-10-02 warnt `apps/record.py` nach jeder Episode davor (7.3).

**Für die Anlage:** Dort hostet der Laptop keine VM — die Ursache entfällt.
Am Netzteil betreiben.

### 7.2 Zwei weitere Ausfallarten, die gleich aussahen

| | Kennzeichen | Ursache | Abhilfe |
|---|---|---|---|
| Override zu niedrig | Schleppfehler **wächst allmählich** | Override 0.2 deckelt die Gelenkgeschwindigkeit unter die Sollgeschwindigkeit | Override 1.0 |
| Steuerkanal gekappt | Schleppfehler **springt**, Arm steht exakt still | RCSC_102/105 | Reset Control im Pendant |
| Fehlerzustand bleibt | Folgeläufe scheitern still oder sofort | Zustand löst sich nicht von selbst, kippt erst ~1 min nach dem Fehllauf | Reset Control, dann Automatik |

### 7.3 Was im Code geändert wurde

| Was | Wo |
|---|---|
| URDF-Korrektur aus `Sim-Inbetriebnahme-Befunde.md` 2.2 erneut eingepflegt (war in diesem Branch verloren) | `bc/data/lara5_candidate.urdf` |
| FK lokal statt per RPC, mit Vortest gegen die Steuerung an mehreren Stellungen | `bc/adapters/neura.py` |
| Schleppfehler-Prüfung je Takt (absolut und maßstabsfrei), Meldung nennt beide Ursachen | `bc/recorder.py` |
| Laufzeit je Episode in den Metadaten, Hinweis bei zu langsamer Bahn | `bc/recorder.py`, `apps/record.py` |
| `init_program()` nur bei Bewegungsvorbereitung, Automatik davor, Ursache bei Ablehnung | `bc/adapters/neura.py` |
| servo_j-Rate je Lauf wählbar, Aufzeichnung und Inferenz gegeneinander geprüft | `apps/record.py`, `apps/infer.py`, GUI |
| Messwerkzeuge | `tools/check_rpc_latency.py`, `tools/check_servo_timing.py`, `tools/check_rate_budget.py` |

### 7.4 Deine Punkte vom Labortag

| Punkt | Status |
|---|---|
| **Blaustich in der Kameravorschau**, im Galaxy Viewer nicht | **behoben.** Die Vermutung lag nah dran: kein doppelter Filter, aber eine **doppelte Farbumrechnung**. Die Kamerabilder sind RGB, die Anzeige erwartet BGR und dreht selbst — Rot und Blau wurden einmal zu oft getauscht. Betraf Vorschau und Livebild während der Aufnahme, **nicht die aufgezeichneten Daten**. Behoben in `bc/preview.py` (`compose`), mit Test |
| **Episodenanzahl standardmäßig 2** | **behoben** — Standard ist jetzt 1 (`bc/gui/tab_recording.py`) |
| **Gerätesuche und Vorschau lassen sich nicht abbrechen** (Freeze?) | **behoben.** Während des Vorgangs heißt der Knopf „Abbrechen"; ein Klick setzt die Oberfläche sofort zurück, ein später eintreffendes Ergebnis wird verworfen und geöffnete Kameras werden wieder freigegeben. Kamerazugriffe laufen in einem eigenen Arbeitsthread, damit ein hängender Treiber nicht die übrige Oberfläche blockiert — das dürfte der „Freeze" gewesen sein. **Grenze:** ein einzelner hängender Treiberaufruf lässt sich aus Python nicht unterbrechen; ein neuer Kamerazugriff wartet dann dahinter, und die Oberfläche sagt das. Eine Aufnahme startet nicht, solange ein solcher Zugriff noch läuft |
bei aufnahme starten ändert sich die % anzeige nicht bei bahn planen erst beim ausführen des taktes, ich weiß nicht ob das gewollt ist aber es steht nur dran "planung läuft" oder so gefolgt von bahn geplant, dann wartet der roboter und dann bewegt er sich und führt den takt aus dort ändert sich die % anzeige. für mich wäre hier qol änderung wenn man etwas mehr nachvollziehen kann was gerade passiert und wann das programm tatsächlich hängt oder nur etwas lädt/verbindung aufbaut etc.
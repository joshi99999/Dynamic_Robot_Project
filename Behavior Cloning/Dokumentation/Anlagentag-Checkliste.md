# Anlagentag — Checkliste bis zur ersten echten Aufzeichnung

Stand 23.09.2026. Reihenfolge so gewählt, dass jeder Schritt nur bestätigte
Voraussetzungen braucht und bewegungsfreie Prüfungen vor den bewegten kommen.

**Zwei Termine** (Festlegung Anwender, 23.09.2026):

* **Termin 1 — Anbindung:** Steht die Kameraanbindung mit der echten Kamera,
  lassen sich Punkte teachen und abfahren (Override egal)? Schritte **0, 1, 2,
  4, 5, 7** und aus 8 **Teachen + Referenzfahrt**. Danach wird alles final
  festgelegt und nachgezogen.
* **Termin 2 — finale Aufnahme:** Override, Rektifizierung, Szenenkamera,
  Blockaufzeichnung (Schritte 3, 6, Rest von 8).

Der Laptop braucht für Termin 1 **kein CUDA**: `record.py`, `teach.py` und
alle `check_*`-Werkzeuge importieren kein torch. CUDA braucht erst die
Policy-Fahrt (CPU gemessen: 384 ms je Vorhersage bei 66 ms Takt).

Alle Befehle aus `Behavior Cloning`, Python `E:\Enviroments\bc_env\python.exe`
(bzw. das Environment des Labor-Laptops). **Not-Aus während aller bewegten
Schritte in Reichweite.** Die Software-Sperre verweigert jede Bewegung, solange
der Controller nicht `is_robot_in_simulation() == True` meldet. An der Anlage
braucht jeder bewegte Schritt `--real-robot` und das Eintippen von `ANLAGE`.

Protokolle landen in `Berichte/` (JSON, Bilder), Aufzeichnungen in
`data_anlage/<Datum>/<Block>`.

---

## 0. Vorab (vor dem Termin, ohne Anlage)

- [ ] `python tests/run_all.py` — alles grün außer dem bekannten
      `test_rectify` (siehe Schritt 6).
- [ ] Sim-Durchstich auf dem Labor-Laptop: `python apps/infer.py --sim --checkpoint checkpoints/sim_durchstich --episodes 3`
      — bestätigt Environment, GPU und die gemessene Vorhersagezeit
      (`predict_ms_median` in `summary.json`, AP 4.1 a).
- [ ] ResNet18-Gewichte im Cache, falls der Laptop offline ist
      (`Training-und-Inferenz.md`, 3.1).
- [ ] **Galaxy SDK installieren und den Adapter gegen gxipy prüfen** — ohne
      Kamera: `python tools/check_daheng_api.py`. Der Adapter ist gegen die
      gxipy-Doku geschrieben; fehlt ein Feature-Name, läuft die Kamera still
      mit falschen Einstellungen weiter (nur eine Hinweiszeile). Erwartet:
      keine `FAIL`. `WARN` bei Features heißt „erst am Gerät prüfbar“ → am
      Termin `--open`.
- [ ] **Kamerakette mit einer USB-Webcam** (Daheng-Ersatz, ohne VM, SimRobot in
      Echtzeit):
      ```powershell
      python tools/check_cameras.py --list
      python tools/check_cameras.py --backend uvc --device 1
      python apps/record.py --sim --cameras wrist-uvc --uvc-device 1 --episodes 2 --out data_sim/<Datum>_webcam
      ```
      Bewertet wird: Bilder kommen an, keine Pacer-Überläufe, Episode wird
      geschrieben. **Nicht** das Latenzbudget: Eine Webcam mit ≤ 30 fps
      liefert Frames, die oft älter als 30 ms sind, der Recorder verwirft die
      Episode dann korrekt als `latenzbudget` (am Entwicklungsrechner
      23.09.: Webcam 9–11 fps, 71 % über Budget). Die Daheng mit nativer Rate
      lag bei 4–9 ms. Webcam-Aufnahmen sind Testdaten — an der Anlage
      verweigert `record.py` den Modus.

## 1. Verbindung, Kinematik, Achsgrenzen — bewegungsfrei

```powershell
python tools/check_sim_robot.py
python tools/check_kinematics.py
```

| Prüfung | bestanden, wenn | wenn nicht |
|---|---|---|
| Funktionsumfang, Versionen | alle Pflichtfunktionen da | Versionsabweichung notieren |
| Controller-FK vs. VM (50 Stellungen, `wrist`/`elbow`) | < 1 mm / 0,01 rad | Planer/Kollisionsmodell nicht übertragbar, zuerst klären |
| IK-Rückprobe | alle lösbar, seedtreu | — |
| TCP-Pose vs. FK der Ist-Stellung | < 1 mm | `read_state()` falsch — nicht aufzeichnen |
| Tool-Offset | Greifer-Tool hinterlegt, TCP an der Backenspitze | Tool am Pendant eintragen (AP 0.6 Punkt 6) |
| Achsgrenzen | mit Datenblatt/Pendant abgeglichen | `config.JOINT_LIMITS_RAD` korrigieren, `JOINT_LIMITS_VERIFIED = True` |

## 2. Adapter-Contract-Suite — dieselbe wie gegen die VM

```powershell
python tests/run_all.py --robot=neura --real-robot contract.test_robot_contract
```

Bewegt den Arm nicht (`servo_j` nur mit der Ist-Stellung, Sprungtest wird vor dem
Senden abgelehnt), **schaltet aber den Greifer**. Erwartet: 12/12 wie in der VM.

## 3. Nachlauf und Override

Arm am Pendant in eine freie Pose fahren, dann:

```powershell
python tools/check_sim_robot.py --move --real-robot --rate 60 --overrides 1.0,0.5 --joint 1
```

Ergebnis je Override: Nachlauf in ms, Folgefehler, Zyklen über Budget. VM-Werte
zum Vergleich: 60 Hz, Override 1,0 → 20–35 ms; 0,5 → 50–120 ms.

- [ ] **Override festlegen** (Sicherheitsdiskussion). Er gilt dann für **alle**
      Aufzeichnungen und die Inferenz — `export.py` bricht bei gemischten Werten
      ab, `infer.py` erzwingt den Wert der Aufzeichnung.
- [ ] Planer-Tempo (`TRANSIT_SPEED_MS`/`APPROACH_SPEED_MS`) festlegen, ebenso
      einheitlich; danach nicht mehr ändern.

## 4. Greifer: Ansteuerung und Totzeit

```powershell
python tools/check_gripper.py --real-robot --cycles 5 --save-frames Berichte/greifer_frames
```

Erst ohne `--roi` laufen lassen, in den gespeicherten Bildern die Backen
suchen, dann mit `--roi x,y,w,h` wiederholen.

- [ ] `grasp()`/`release()` schalten den Greifer (Modus `hardware`).
- [ ] **RPC-Dauer** < 17 ms. Blockiert der Aufruf länger, hält er Aufzeichnung
      und Inferenz im Takt an — vor der Aufzeichnung lösen.
- [ ] Schließzeit ≤ `GRIPPER_DWELL_S` (500 ms). Sonst Dwell erhöhen, danach
      `GRIPPER_DWELL_VERIFIED = True`. **So kurz wie möglich festlegen** —
      jeder Dwell-Takt ist ein Stillstand, den die Policy nicht zählen kann
      (siehe unten, Stillstand).
- [ ] **Gibt der Greifer eine Rückmeldung?** Backenposition, Kraft, ein
      digitaler Eingang „geschlossen“ oder eine NeuraPy-Funktion, die den
      Ist-Zustand liest. Notieren, wie sie heißt und wie schnell sie kommt.
      Hintergrund: Im State steht heute nur der **kommandierte** Zustand
      (`NeuraRobot.gripper_command` setzt ihn sofort beim Befehl). Damit
      unterscheidet die Policy „muss greifen“ von „hat gegriffen“, aber nicht
      „schließt noch“ von „ist zu, weiterfahren“. Eine gemessene Rückmeldung
      würde genau das liefern — ist eine **Schema-Änderung** und muss vor
      Termin 2 entschieden sein.

## 5. Kameras

```powershell
python tools/check_cameras.py --list
python tools/check_daheng_api.py --open
python tools/check_cameras.py --only wrist
python tools/check_cameras.py
```

- [ ] `check_daheng_api.py --open`: jedes Feature vorhanden und schreibbar
      (bekannt: Binning ist an der VEN-161 nicht schreibbar, Config setzt es
      nicht). Sonst den Namen im Adapter korrigieren, bevor aufgezeichnet wird.
- [ ] Wrist: Rate, Frame-Alter < 30 ms, Seriennummer passt.
- [ ] Belichtung fest (`exposure_us`), Weißabgleich einmal automatisch, dann die
      Ratios in `config.WRIST_CAMERA.white_balance_ratios` pinnen.
- [ ] **Szenenkamera entscheiden** (Modell, Montage). Danach `SCENE_CAMERA`
      eintragen und `SCENE_CAMERA_CONFIRMED = True`.
- [ ] Beide zusammen: Versatz unter 30 ms.

## 6. ArUco / Rektifizierung (Szenenkamera)

```powershell
python tools/check_rectify.py --camera scene
python tools/check_rectify.py --camera scene --layout marker_layout.json --x-range 0.2,0.6 --y-range=-0.2,0.2
```

- [ ] Wörterbuch und IDs der Tischmarker (mit dem CV-Team), Layout-Datei anlegen.
- [ ] Erkennung im vollen Bild **und** in 240 × 320 vergleichen. Befund bisher
      (synthetisch): in 240 × 320 gehen Marker verloren → Homographie im vollen
      Bild rechnen, erst das entzerrte Bild verkleinern. Damit klärt sich
      `test_rectify`.
- [ ] Offen danach (Code): Rektifizierung in Recorder und Inferenz einbauen,
      identisch für beide (AP 1.4). Das ist eine **Schema-Änderung** und kommt
      vor die erste echte Aufzeichnung.

## 7. Sim-Kette mit echter Wrist-Kamera (VM oder SimRobot)

Wie besprochen: in der Simulation fahren, aber mit dem echten Wrist-Bild.

```powershell
python apps/record.py --robot neura --cameras wrist-real --override 1.0 --sequence sequences/pick_to_station.json --episodes 3 --out data_vm/<Datum>/wrist_real --block VM-WR
python tools/plot_episode.py "data_vm/<Datum>/wrist_real"
```

- [ ] Sync-Verletzungen 0 %, keine Pacer-Überläufe durch die Kamera-Last.

## 8. Erste bewegte Fahrt der Aufzeichnung an der Anlage

- [ ] Punkte der Ablaufdatei am Pendant teachen (`CLEAR_FOV`, `APPROACH_01`,
      `PRE_GRASP`, `PICK`, `PRE_PLACE`, optional `APPROACH_02`).
      `PRE_GRASP` **senkrecht über** `PICK`, mindestens 2 cm, besser 5–10 cm
      (VM: 14,5 cm) — sonst kappt der Planer das 10-mm-Überschleifen auf die
      halbe Abstiegsstrecke.
- [ ] Referenzfahrt ohne Rauschen, eine Episode, Override wie festgelegt:
      `--noise-scale 0 --episodes 1`, dann `plot_episode.py` ansehen.
- [ ] Dann mit Rauschen (Faktor je Episode 0–1), einige Episoden.
- [ ] Nach Schritt 6 (Rektifizierung) und 5 (Szenenkamera): **Blockweise
      aufzeichnen** — Objekt hinlegen, `PRE_GRASP`/`PICK` per Touch-up,
      `record.py --block Bxx --object "..." --light "..." --camera-pose nominal`.

## Vor der ersten echten Aufzeichnung festzuhalten (nachträglich nicht korrigierbar)

| Punkt | Wo | Status |
|---|---|---|
| Override | `record.py --override`, einheitlich | Schritt 3 |
| Planer-Tempo, Rampe, Dwell | `config.py` | Schritt 3/4 |
| Szenenkamera + Rektifizierung im Schema | `config.SCENE_CAMERA`, Recorder | Schritt 5/6 |
| Achsgrenzen/Filtergrenzen | `JOINT_LIMITS_RAD`, `SERVO_MAX_*` | Schritt 1/3 |
| Zykluszeit als Vergleichsmetrik ja/nein | AP 5.1 | Team |
| **Stillstand an Zwischenpunkten** (Deadlock, AP 2.6) | Ablaufdatei (Überschleifen), Greifer-Dwell, ggf. Schema | **PRE_GRASP gelöst** (23.09.): 10 mm Überschleifen, Anfahrt bleibt senkrecht (Ecke um 0,4 mm verfehlt, ab 1,6 mm darunter exakt auf der Achse, VM-Punkte nachgerechnet). **Offen: PICK** — einziger verbleibender Halt ist der Greifer-Dwell. Kandidaten: gemessene Greiferrückmeldung als State (Schritt 4), Dwell kurz und konstant, Policy am Greifpunkt aufteilen (aus denselben Aufnahmen möglich). Vor Termin 2 in der VM gegenfahren. |
| Greiferrückmeldung als State ja/nein | `dataset.py`, Adapter | Schritt 4 |

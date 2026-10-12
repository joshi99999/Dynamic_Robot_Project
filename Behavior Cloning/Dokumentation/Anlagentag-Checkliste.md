# Anlagentag — Checkliste bis zur ersten echten Aufzeichnung

Stand 04.10.2026 (vorher 23.09.2026). Die Reihenfolge folgt dem Reiter
**Systemcheck** der Oberfläche: erst hardwarefrei, dann Hardware ohne
Bewegung, dann mit Bewegung — jeder Schritt braucht nur bestätigte
Voraussetzungen. Die Spalte **GUI** nennt den Punkt im Reiter; die
Kommandozeile steht dort unter den Einstellungen und gilt im Terminal
genauso.

Jeder Punkt gibt eine **Bewertung** aus (`[OK]` / `[WARNUNG]` / `[FEHLER]`).
Nach einer Sitzung „Protokoll speichern“ → `Berichte/<Datum>_Systemcheck.txt`.
Messberichte (JSON, Bilder) legen die Werkzeuge selbst in `Berichte/` ab,
Aufzeichnungen landen in `data_anlage/<Datum>/<Block>`.

**An der Anlage:** oben in der Backend-Leiste „Reale Anlage freigeben“
setzen. Dann laufen die Werkzeuge, die das kennen, mit `--real-robot`, und
die GUI schickt das Freigabewort `ANLAGE`. Ohne den Haken verweigert der
Adapter jede Bewegung, solange die Steuerung nicht als Simulation antwortet
(VM und Control-Box teilen sich die Adresse). **Not-Aus während aller
bewegten Schritte in Reichweite** — die Rückfragen der GUI sind eine
Bedienerführung, kein Not-Aus.

---

## Was seit dem 23.09. erledigt ist

| | Stand |
|---|---|
| Daheng VEN-161 | liefert am Laptop echte Bilder (Labortag 01.10.); Adapter gegen gxipy 2.6 geprüft |
| Ursache für Bahnabbrüche, RCSC_102 und „Frames über Budget“ in der VM | **Host-Leistung** (Laptop gedrosselt, VM darauf) — `Laptop-Inbetriebnahme-Befunde.md` 7.1 |
| Lesende Werkzeuge im Teach-Modus/Fehlerzustand | gehen; der Adapter nennt bei Ablehnung die Ursache |
| URDF-Korrektur, lokale FK, Schleppfehler-Abbruch im Recorder | im Code, mit Vortest gegen die Steuerung |
| servo_j-Rate | je Lauf wählbar (Standard 60 Hz) |
| **Planer-Tempo** | je Lauf wählbar — Aufnahme-Reiter „Tempo Transit/Endanflug/PTP“, `record.py --transit-speed/--approach-speed/--ptp-speed` |
| Blaustich in der Vorschau, Episodenzahl, Abbrechen in der Gerätesuche | behoben |
| Override 1,0 gegen 0,2 (VM, 03.10.) | 0,2 sieht ruhiger aus, weil der Arm **4–10° hinterherläuft** — die Daten zeigen dann eine Bahn, die nicht gefahren wurde. Das Ruckeln bei 1,0 kam vom gedrosselten Laptop (verspätete `servo_j`). **Tempo im Planer senken, nicht den Override.** |

---

## 0. Vorab — ohne Anlage (Laptop, VM)

| ✓ | Schritt | GUI | bestanden, wenn |
|---|---|---|---|
| [ ] | Unittests | hardwarefrei → *Unittests* | alles grün außer dem bekannten `test_rectify` (OpenCV 4.13) |
| [ ] | Laptop vorbereiten | — | **Netzteil dran, Energiemodus „Beste Leistung“.** Daran hing der Takt in allen Läufen bis 03.10. An der Anlage läuft keine VM — die größte Last fällt dort weg |
| [ ] | Probefahrt in der VM, dann bewerten | Aufnahme-Reiter, danach hardwarefrei → *Aufnahmen bewerten* | „Takt gehalten“ (< 5 % verspätete `servo_j`), kein „Ruckeln“, Schleppfehler-Median < 0,03 rad. Hält der Laptop den Takt nicht: Tempo senken (z. B. Transit 0,08 / Endanflug 0,03 / PTP 0,3) und wiederholen |
| [ ] | Daheng-API ohne Kamera | hardwarefrei → *Daheng-API* | keine `FAIL`; `WARN` bei Features heißt „erst am Gerät prüfbar“ |
| [ ] | Punkte der VM sichern | Hardware → *Punkte sichern* | Datei in `sequences/` — Beleg, gegen welche Punkte aufgezeichnet wurde |

## 1. An der Anlage — ohne Bewegung

| ✓ | Schritt | GUI | bestanden, wenn | wenn nicht |
|---|---|---|---|---|
| [ ] | Steuerung lesen | *Steuerung prüfen (nur lesen)* | `is_robot_in_simulation() = False`, Greifermodus `hardware`, Tool gesetzt | Versionsabweichung notieren; ohne Greifer kein Greifertest |
| [ ] | Kinematik, Achsgrenzen | *Kinematik und Achsgrenzen* | Controller-FK gegen VM (50 Stellungen, `wrist`/`elbow`) < 1 mm / 0,01 rad; TCP-Pose gegen FK < 1 mm | Planer/Kollisionsmodell nicht übertragbar — zuerst klären |
| [ ] | Tool-Offset | *IK-Absicherungen* | Greifer-Tool hinterlegt, TCP an der Backenspitze | Tool am Pendant eintragen |
| [ ] | Achsgrenzen abgleichen | (Ausgabe von *Kinematik*) | mit Datenblatt/Pendant gleich | `config.JOINT_LIMITS_RAD` korrigieren, `JOINT_LIMITS_VERIFIED = True` |
| [ ] | RPC-Latenz | *RPC-Latenz messen* | kein Hänger im Dauerlauf; Aufrufkosten notieren | Grundlage für Schritt 2.1 |
| [ ] | Kameras finden | *Kameras suchen* | Daheng (Seriennummer) und Szenenkamera gelistet | Kabel/Port, `Laptop-Inbetriebnahme-Befunde.md` 4.2 |
| [ ] | Daheng am Gerät | *Daheng am Gerät prüfen* | jedes Feature vorhanden und schreibbar (bekannt: Binning nicht) | Feature-Namen im Adapter korrigieren, **bevor** aufgezeichnet wird |
| [ ] | Kameras: Rate, Alter, Versatz | *Kameras: Rate und Versatz* | je Kamera ≥ 15 fps, Bildalter p95 < 30 ms, Versatz p95 < 30 ms | Belichtung/Last prüfen |
| [ ] | Belichtung, Weißabgleich | (Aufnahme-Reiter, Vorschau) | Belichtung fest; Weißabgleich einmal automatisch, dann Ratios in `config.WRIST_CAMERA.white_balance_ratios` pinnen | — |
| [ ] | Kamera-Contract | *Kamera-Contract* (daheng) | alle grün | — |
| [ ] | **Szenenkamera entscheiden** | — | Modell, Montage festgelegt → `config.SCENE_CAMERA`, `SCENE_CAMERA_CONFIRMED = True` | Szenenkamera bleibt Platzhalter — keine echten Aufnahmen |
| [ ] | ArUco / Rektifizierung | *ArUco / Rektifizierung* | Wörterbuch und IDs bekannt (mit dem CV-Team); Erkennung im vollen Bild **und** in 240 × 320 | in 240 × 320 verloren → Homographie im vollen Bild rechnen, erst danach verkleinern |
| [ ] | Roboter-Contract | *Roboter-Contract* (schaltet den Greifer) | 12/12 wie in der VM | — |
| [ ] | Punkte teachen und sichern | Pendant, dann *Punkte sichern* | `CLEAR_FOV`, `APPROACH_01`, `PRE_GRASP`, `PICK`, `PRE_PLACE` (optional `APPROACH_02`); `PRE_GRASP` **senkrecht über** `PICK`, besser 5–10 cm | sonst kappt der Planer das 10-mm-Überschleifen |

## 2. An der Anlage — mit Bewegung

Arm vorher am Pendant in eine freie Pose fahren.

| ✓ | Schritt | GUI | bestanden, wenn | daraus festlegen |
|---|---|---|---|---|
| [ ] | **2.1 servo_j-Timing** (bestromt, bewegt nicht) | mit Bewegung → *servo_j-Timing messen*, danach hardwarefrei → *Taktbudget rechnen* | „15 Hz / servo_j 60 Hz passt“ mit Reserve | **servo_j-Rate.** Die wichtigste Messung vor der ersten Aufnahme — in der VM stimmten die Zahlen nur bedingt |
| [ ] | 2.2 Nachlauf und Override | *Nachlauf und Override* (Overrides `1.0,0.5`, Gelenk 1) | Nachlauf je Override (VM: 1,0 → 20–35 ms; 0,5 → 50–120 ms) | **Override** (Sicherheitsdiskussion). Empfehlung aus den VM-Läufen: 1,0, das Tempo über den Planer begrenzen |
| [ ] | 2.3 Greifer-Totzeit | *Greifer-Totzeit messen*: erst ohne ROI mit gespeicherten Bildern, Backen im Bild suchen, dann mit ROI | „Dwell reicht“, „Greiferbefehl blockiert höchstens … ms“ (< 17 ms) | **`GRIPPER_DWELL_S`** so kurz wie möglich, dann `GRIPPER_DWELL_VERIFIED = True` |
| [ ] | 2.4 Greifer-Rückmeldung klären | — | Gibt es Backenposition, Kraft, einen Eingang „geschlossen“ oder eine NeuraPy-Funktion? Name und Latenz notieren | ob der gemessene Greiferzustand in den State kommt (**Schema-Änderung**) |
| [ ] | 2.5 Referenzfahrt ohne Rauschen | Aufnahme-Reiter: Rauschen 0, 1 Episode, Block `REF`; danach *Aufnahmen bewerten* und *Episodenbericht (PDF)* | Takt gehalten, kein Ruckeln, Arm folgt der Bahn | **Planer-Tempo** (Transit, Endanflug, PTP) |
| [ ] | 2.6 Mit Rauschen | Aufnahme-Reiter: Rauschen bis 1,0, einige Episoden einzeln | wie 2.5, keine verworfenen Episoden | — |
| [ ] | 2.7 Blockweise aufzeichnen | Aufnahme-Reiter mit Block, Objekt, Licht, Kamerapose | erst nach Szenenkamera und Rektifizierung | — |

### 2a. Zittern gegen Bahnpuffer (vorbereitet 12.10., für den nächsten Labortag)

Anlass: Am 05.10. zitterte der Arm mit `servo_j` 60 Hz. In der VM ist das
**nicht messbar**: Die VM liefert nur ~50 neue Gelenkwerte je Sekunde, und
`move_trajectory` sowie `movelinear_online` tun dort nichts (der eine fehlt,
der andere wird angenommen und nicht ausgeführt). Die Anlage meldete am
05.10. `v5.0.8` mit 210 statt 202 Funktionen. Alles Weitere entscheidet
sich also hier. Reihenfolge, je Schritt ein Lauf:

| ✓ | Schritt | GUI | Ergebnis lesen | daraus folgt |
|---|---|---|---|---|
| [ ] | P1 Pufferbefehle (nur lesen) | *Pufferbefehle prüfen* | `move_trajectory` vorhanden? | ja → P3 mit allen Tests, nein → P3 ohne die Puffertests |
| [ ] | P2 Quellrate | in P3 enthalten, Spalte **neue Werte** | ≥ 0,9 × servo-Rate? | sonst ist die Glätte auch an der Anlage nur grob messbar (Kennzahlen `neu:`) |
| [ ] | P3 Bahnpuffer testen | *Bahnpuffer testen*, Tests `referenz,komplett,knoten15,anschluss,leerlauf,stopp,online`, Vergleichsraten `90,120`, Gelenk 1, Auslenkung 0,05 | Fazit-Zeilen: `referenz_90/120` (Stillstand gegen 60 Hz, Taktüberläufe), `online` (fährt / pulsiert / bewegt sich nicht), Puffer freigegeben ja/nein | siehe unten |
| [ ] | P4 Gegenprobe Aufnahme | Aufnahme-Reiter, Rauschen 0, 1 Episode, mit der gewählten servo-Rate (`--servo-rate`) | *Aufnahmen bewerten*: Takt gehalten, verspätete `servo_j` < 5 % | Rate bestätigen |

**Entscheidung nach P3**, in dieser Reihenfolge der Kosten:

1. **90 oder 120 Hz glätten genug** (Stillstand deutlich kleiner, 0 Überläufe) → nur
   `config.SERVO_RATE_HZ` ändern, Recorder und `infer.py` nutzen dieselbe Schicht.
   Rechnung mit den Aufrufkosten vom 05.10.: 90 Hz p95 ~47 ms, 120 Hz p95 ~60 ms von
   66,7 ms je Takt — 120 Hz nur, wenn 2.1 deutlich schnellere Aufrufe zeigt.
2. **`online` fährt ohne Pulsieren** → `movelinear_online` als Ausführungsschicht
   bauen (kartesisch über die FK, Action bleibt im Gelenkraum).
3. **Rollender Puffer freigegeben** (`move_trajectory`, nahtloser Anschluss, `stop()`
   leert) → Puffer im Adapter bauen.

Kein Schritt ändert das Datenschema. Die Ausführungsschicht muss nur in Aufnahme
und Inferenz dieselbe sein.

---

## Vor der ersten echten Aufzeichnung festzulegen (nachträglich nicht korrigierbar)

Alles hier gilt für **alle** Episoden eines Datensatzes und die Inferenz —
`apps/export.py` bricht bei gemischten Werten ab, `apps/infer.py` erzwingt
Override und servo_j-Rate der Aufzeichnung.

| Punkt | Wo | aus Schritt | Status |
|---|---|---|---|
| Override | Backend-Leiste / `--override` | 2.2 | offen (Sicherheitsdiskussion); Empfehlung 1,0 |
| servo_j-Rate | Aufnahme-Reiter / `--servo-rate` | 2.1 | offen; Standard 60 Hz |
| Planer-Tempo | Aufnahme-Reiter / `--transit-speed`, `--approach-speed`, `--ptp-speed` | 2.5 | offen; Standard 0,15 / 0,05 m/s, 0,5 rad/s (provisorisch) |
| Rampe, Dwell | `config.SEGMENT_RAMP_S`, `config.GRIPPER_DWELL_S` | 2.3 | Dwell unverifiziert |
| Achs- und Filtergrenzen | `JOINT_LIMITS_RAD`, `SERVO_MAX_*` | 1, 2.2 | unverifiziert |
| Szenenkamera + Rektifizierung im Schema | `config.SCENE_CAMERA`, Recorder **und** Inferenz | 1 | offen — **Schema-Änderung, Code fehlt noch** |
| Greiferrückmeldung als State | `dataset.py`, Adapter | 2.4 | offen — Schema-Änderung |
| Stillstand an Zwischenpunkten | Ablaufdatei, Dwell, ggf. Schema | — | `PRE_GRASP` gelöst (10 mm Überschleifen). **Offen: `PICK`** — einziger Halt ist der Greifer-Dwell. Kandidaten: Greiferrückmeldung als State, Dwell kurz und konstant, Policy am Greifpunkt aufteilen |
| Zykluszeit als Vergleichsmetrik ja/nein | AP 5.1 | — | Team |

## Offen im Code — vor dem Labortag nicht blockierend

| Was | Wo | Anmerkung |
|---|---|---|
| Rektifizierung in Recorder und Inferenz | `bc/rectify.py`, Recorder, `apps/infer.py` | **wird blockierend**, sobald die Szenenkamera feststeht |
| Zeitlimit um `_call()` | `bc/adapters/neura.py` | NeuraPy hat kein Socket-Timeout; ein Hänger ist von „dauert lange“ nicht unterscheidbar |
| Not-Halt am Bahnende | Recorder | Hypothese: Servo-Interface wird bei Restbewegung abgebaut (`Taktzeit-und-RPC-Latenz.md` 6.5); prüfbar durch einige Takte Halten vor dem Deaktivieren |
| Vorabtest Sollgeschwindigkeit gegen Override × Achsgrenze | Planer | hängt an den unverifizierten Grenzen (`Taktzeit-und-RPC-Latenz.md` 9.4) |
| Fortschrittsanzeige beim Bahnplanen | Aufnahme-Reiter | Anwender-Notiz 01.10.: bis zur ersten Bewegung steht nur „Planung läuft“ — nicht erkennbar, ob es hängt oder lädt |
| Verkleinern im Kamera-Thread, Daheng mit 30 fps | Capture, `config.WRIST_CAMERA` | erst, wenn *servo_j-Timing* an der Anlage einen engen Takt zeigt |
| Export/Training auf dem Laptop | Environment | `torch`, `lerobot` fehlen — Aufnahmen zum PC bringen |
| UVC-Backend DSHOW → MSMF | `bc/adapters/cam_uvc.py` | betrifft nur die Webcam als Ersatz |

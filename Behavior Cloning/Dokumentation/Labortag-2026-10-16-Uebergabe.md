# Übergabe Labortag 16.10.2026 → Weiterarbeit am PC

Arbeitsanweisung für die nächste Sitzung am PC. Von oben nach unten
abarbeiten. Hintergrund: `Labortag-Messplan-Zittern.md` (Plan des Tages),
`Zittern-an-der-Anlage.md` (§ 4–9 bisherige Messungen, **§ 10 Halteregel**).
Alle Dateien des Tages mit Zuordnung: `Berichte/2026-10-16_Labortag-Uebersicht.md`.

**Regeln:** Nichts committen — der Anwender prüft und committet selbst.
Am PC das Environment `E:\Enviroments\bc_env` nutzen (Laptop: `C:\Enviroments\bc_env`).
`test_rectify` scheitert bekannt an OpenCV 4.13 — nicht durch Pinnen zudecken.

---

## 1. Ergebnis in Kürze

| Frage | Antwort |
|---|---|
| F1 Laptop-Takt | Laptop plant pünktlich (Verspätung median 0,4 ms), egal ob Programme offen sind. Engpass ist der `servo_j`-RPC: ~10 ms median, p95 ~21 ms, dazu 1–2 Hänger à ~210 ms je Lauf. Ping < 1 ms, Netz unauffällig (USB-C-LAN-Adapter). 60 Hz knapp, 90/120 Hz nicht haltbar. |
| F2 Eigenfrequenz | 5,6–6,2 Hz (Hand und Gummihammer gleich). Tisch heute **ausgeklappt** (13.10. eingeklappt) → steifer, nur eingeschränkt mit 13.10. vergleichbar. |
| F3 Ausführungsart | **`online`** ist am Arm so ruhig wie der Neura-Standardlauf. `servo_j_pos` halbiert das Zittern gegenüber `linear`, ist aber 4× unruhiger als `online` → verworfen (nur VM). |
| Neues Problem | `online` (×1,3/×1,5) läuft ~1,3 s hinterher und **dreht 27 mm über PICK um**, ohne Halt (Neura-Licht bleibt grün, bei `servo_j` kurz blau). |

Zittern an H1 (Handy am Unterarm, Achse 5; RMS über 5 Hz, m/s²):

| Neura-Standard | online | linear o0,5 | pos | linear |
|---|---|---|---|---|
| **0,18** | **0,20** | 0,21 | 0,83–0,89 | ~2,0 |

PICK-Höhe (Vorwärtskinematik aus dem Mitschrieb):

| Lauf | tiefster Punkt ggü. PICK | Zeit unten | H1 RMS | Eindruck |
|---|---|---|---|---|
| pos | +0,1 mm | 0,91 s | 0,83 | hält (blau), unruhig |
| online ×1,3/×1,5, Dwell 0,5 s | **+27,3 mm** | 0,18 s | 0,20 | sehr glatt, dreht oben um |
| Test 2: online ×2,0/×3,0 | +0,1 mm | 0,66 s | **0,67** (15-Hz-Spitze) | tiefer, an Ecken ruckiger |
| Test 3: online ×1,3/×1,5, Dwell 1,5 s | +0,2 mm | 0,33 s | **0,26** | deutlich tiefer, sehr glatt |

Vorgabe des Anwenders: Der Roboter soll sich **möglichst wie mit dem
Neura-Standardprogramm** bewegen. Kamerastange/Aufbau sind Hardware-Themen,
nicht Teil dieser Arbeit (deshalb kein Handy H2).

## 2. Die Idee: „am PICK warten, bis erreicht“

Test 3 zeigt: Bekommt `online` genug Zeit am Haltepunkt, erreicht es PICK
und bleibt ruhig. Statt pauschal länger zu warten (kostet in jeder Episode
Zeit) oder schneller zu fahren (Test 2: 3× mehr Zittern), wartet die neue
Regel genau so lange, wie der Arm braucht:

1. Am Greiferwechsel (PICK) bleibt das Ziel stehen, bis der TCP ≤ 1 mm am
   Ziel ist (2 Messungen in Folge).
2. Dann Greiferbefehl und der normale Dwell (0,5 s), danach weiter.
3. Zeitlimit 3 s → Episode verworfen bzw. Inferenzfahrt beendet.
4. Aufnahme und Inferenz nutzen dieselbe Regel (steht in den Metadaten).

**Stand:** Am 16.10. von einer Nebensitzung eingebaut, **nicht committet,
nicht an der Anlage gemessen**. Beschreibung und Abnahmekriterien:
`Zittern-an-der-Anlage.md` § 10.2–10.5.
Geänderte Dateien (12): `bc/config.py`, `bc/execution.py`, `bc/recorder.py`,
`bc/lerobot_io.py`, `apps/record.py`, `apps/infer.py`,
`tools/check_servo_stream.py`, `tests/test_execution.py`,
`tests/test_recorder.py`, `tests/test_apps.py`,
`tests/test_check_servo_stream.py`, `Dokumentation/Zittern-an-der-Anlage.md`.
Testlauf am Laptop: 354 bestanden, nur `test_rectify` rot.

## 3. Daten

* **Berichte/Mitschriebe:** `Berichte/2026-10-16_*` (JSON + `_Mitschrieb.npz`).
* **Handy:** `C:\Users\jerem\OneDrive\Projekt\20_Dokumentation\Zittern\Beschleunigung mit g 2026-10-16 *.xlsx`.
  Dateiname = Exportzeit. Duplikate: `12-22-31` (in `12-42-37`), `12-49-01` (in `13-00-57`).
* **Video:** noch vom Video-Handy holen und ablegen (Ort in der Übersicht nachtragen).

Wichtigste Läufe:

| Bericht | Inhalt |
|---|---|
| `2026-10-16_11-51/-52/-55_Trajektorien-Puffer.json` | A (Laptop-Takt) |
| `2026-10-16_12-18 / 12-18-43 / 12-24 / 12-24-27_Trajektorien-Puffer.json` | C (J1-Bahn linear/pos, zweimal) |
| `2026-10-16_12-47-00_Servo-Strom.json` | D1 (6 Läufe) |
| `2026-10-16_13-00-44_Servo-Strom.json` | D2 (Wiederholung) |
| `2026-10-16_14-01-59_Servo-Strom.json` | Test 1 (pos + online) |
| `2026-10-16_14-03-10 / 14-05-06_Servo-Strom.json` | Wiederholungen (Config versehentlich nicht umgestellt) |
| `2026-10-16_14-15-21_Servo-Strom.json` | Test 2 (×2,0/×3,0) |
| `2026-10-16_14-18-59_Servo-Strom.json` | Test 3 (Dwell 1,5 s) |

Vollständige Zuordnung mit LOS-Zeiten und Handy-Abschnitten:
`Berichte/2026-10-16_Labortag-Uebersicht.md`.

## 4. Aufgaben am PC

> **Stand 16.10. abends (PC):** abgearbeitet bis auf die Videos. Ergebnisse in
> `Zittern-an-der-Anlage.md` **§ 11**, Arbeitsblatt `Labortag-Messplan-Zittern.md`
> ausgefüllt. **Wichtigster Befund:** `online` ×1,3/×1,5 würde von Recorder
> und Inferenz abgebrochen (Schleppfehler bis 0,79 rad) und fährt bis
> 49 mm / 31° neben der Planbahn — Entscheidung § 11.8 offen (Empfehlung:
> `servo_j` linear, Override 0,5 an der Anlage bestätigen). Abschnitt 5
> unten gilt deshalb erst nach dieser Entscheidung.

### 4.1 Code der Halteregel prüfen
- [x] `git diff` der 12 Dateien durchsehen (Anwender + Claude). Besonders:
      `bc/recorder.py` (Warten vor dem Greiferbefehl, Pacer-Neustart),
      `apps/infer.py` (gleiche Regel beim Greiferwechsel der Policy),
      `bc/execution.py` (`wait_until_reached`, `lead_index(..., hold_end)`).
- [x] `python tests/run_all.py` am PC. (354/355 vor den Änderungen dieser Sitzung, danach 358/359 — nur `test_rectify`)
- [x] In der VM/SimRobot einmal fahren (VM fährt `online` nicht → SimRobot über `apps/record.py --sim --ausfuehrung online`: 2 Halte à 0,07 s):
      `python tools/check_servo_stream.py --move --laeufe online-15-r0-o1,onlinehalt-15-r0-o1`
      (VM fährt `movelinear_online` evtl. nicht, siehe `bc/execution.resolve_mode` — dann SimRobot).
- [x] Offene Punkte aus § 10.5 bewerten (§ 11.2) (Toleranz/Zeitlimit, nur Position statt Orientierung,
      Greiferbefehl in der Inferenz einen Takt früher).

### 4.2 Auswertung der Messdaten nachholen
- [x] **`tools/report_handy.py` reparieren:** Uhrzeiten nach phyphox-Pause falsch
      (`report_zittern.handy_auswerten` nutzt nur den ersten START). Abschnitte über
      „Metadata Time“ (START/PAUSE-Paare) auf die Systemzeit abbilden; keine
      Interpolation über Pausen. Test dazu.
- [x] Danach alle Handy-Dateien mit dem Werkzeug auswerten und mit den Zahlen in § 1 vergleichen
      (die wurden am Labortag mit einem Hilfsskript gerechnet).
- [x] Alte Berichte nachrechnen, damit die Haltepunkt-Kennzahlen drinstehen:
      `python tools/check_servo_stream.py --auswerten Berichte/2026-10-16_<stempel>_Servo-Strom`
      für `12-47-00`, `13-00-44`, `14-01-59`, `14-15-21`, `14-18-59`.
- [x] **Wo `online` sonst noch abweicht:** quer max 27 mm sitzt am PICK; prüfen, wie groß die
      Abweichung an den übrigen Punkten (APPROACH_01, PRE_GRASP, PRE_PLACE) ist und ob Orientierungswechsel
      zu früh beginnen. Ggf. Haltepunkte auch dort nötig?
- [x] **Test 3 genauer:** 4 Takt-Überläufe und 14 ms Sendeabstand-Streuung bei `online` (sonst 0) —
      vermutlich ein ~210-ms-Hänger; im Mitschrieb `14-18-59` prüfen.
- [x] **210-ms-Hänger** im `servo_j`/`online_j`-Aufruf: fest ~210 ms, zufällige Stelle — Verdacht
      Delayed-ACK/Nagle bzw. USB-LAN-Adapter (Energiesparen). Nur Vermutung; im Mitschrieb aller Läufe
      zählen und mit Aufrufzeiten belegen. Für `online` relevant, weil es künftig die Ausführungsart ist.
- [ ] Videos: Neura-Leuchte bei Test 3 (blau oder grün am PICK?) und Höhe am PICK sichten.

### 4.3 Dokumentation
- [x] `Labortag-Messplan-Zittern.md` ausfüllen (Tabellen A–E, *Neue Infos*: Tisch ausgeklappt,
      kein H2, Punkte mit falschem Tool gespeichert → am Pendant mit `ZimmerLWR50` neu gespeichert).
- [x] `Zittern-an-der-Anlage.md`: Messungen 16.10. ergänzen (Zahlen aus § 1, Bilder aus den Handy-Plots).
- [x] `Environment-Laptop.md`: Tippfehler `C:\Enviromentsc_env` → `C:\Enviroments\bc_env`.

## 5. Nächster Anlagentag (vorbereiten)

> **Überholt durch § 6** (Labortag 19.10. erledigt, Entscheidung für `servo_j`
> linear Override 0,5). Stehen gelassen zur Nachvollziehbarkeit.

Kurzer Block, ~20 min, Handy an H1, Video auf PICK:

```bat
python tools/check_servo_stream.py --move --real-robot --laeufe online-15-r0-o1,onlinehalt-15-r0-o1
```

danach eine Aufnahme mit `apps/record.py ... --ausfuehrung online` (mit Greifer, sobald
angebunden; sonst `--no-gripper`) und `reach_waits` in `meta.json` prüfen.
Abnahme laut § 10.4: tiefster Punkt ≤ 1 mm über PICK, Verweilzeit ≥ 0,5 s,
Wartezeit deutlich unter 3 s, H1 ≈ 0,20 m/s², Leuchte zeigt den Halt wie
beim Standardlauf. Danach `config.EXECUTION_MODE`/`ONLINE_VERIFIED` entscheiden.

---

## 6. Labortag 19.10. und Entscheidung

Dreimal dieselbe Serie (`linear o0.5`, `linear o1`, `online`, `onlinehalt`,
jeweils mit und ohne Rauschen), Aufbau variiert (Fuß lose / festgestellt /
Plattform gehalten). Dateien, Tabellen und nachgestellte Abbruchprüfungen:
`Berichte/2026-10-19_Labortag-Uebersicht.md`.

| Lauf | H1 RMS (m/s²) | Bahn quer max | PICK | Fahrt |
|---|---|---|---|---|
| **linear o0.5** | **0,19–0,20** | **1,2 mm** | exakt | **11,6 s** |
| linear o1 | 1,55–1,74 | ~2 mm | exakt | 11,9 s |
| online | 0,20 | 27 mm | +27 mm | 13,4 s |
| onlinehalt | 0,21–0,22 | 27 mm (Ecken) | +0,2 mm (~1,6 s gewartet) | 14,1 s |
| linear r1 o0.5 | 0,24–0,26 | 5 mm | – | 12,3 s |
| onlinehalt r1 | 0,27–0,29 | 39 mm | +0,2 mm | 15,4 s |

* Override 0,5 halbiert das Tempo **nicht** (Fahrzeit wie Override 1).
* Fuß festgestellt / Plattform gehalten: kein nennenswerter Einfluss.
* `onlinehalt` funktioniert (PICK reproduzierbar), kürzt aber Ecken und zeigt 15-Hz-Pulsieren.

**Entscheidung Anwender (19.10.): `servo_j` linear, Override 0,5** — für
Aufnahme, Inferenz und VM. `online`/`onlinehalt` bleiben im Code, aber nicht
als Standard.

## 7. Umsetzung `servo_j` linear Override 0,5 (Aufgaben)

> **Überholt (PC 19.10., § 7.7):** Die folgenden Befunde stammten aus einer
> Nachrechnung mit falscher Zeitbasis. Gegen die Sendezeit liegt Ziel–Ist bei
> 0,06–0,15 rad, und weder Kriterium a noch b schlägt an. Stehen gelassen zur
> Nachvollziehbarkeit.

Befunde aus den nachgestellten Recorder-Prüfungen (19.10., siehe Übersicht):
ohne Rauschen läuft Kriterium a nie an (max Ziel–Ist 0,10–0,20 rad), aber
**Kriterium b schlägt in allen drei Serien an** (Anlauf nach Stillstand,
nach PICK-Dwell). **Mit Rauschen** erreicht Ziel–Ist 0,23–0,46 rad
(Gelenk 6, Plan bis 0,69 rad/s, Arm folgt mit ~0,3 s Nachlauf) →
**Kriterium a (0,30 rad) in 2 von 3 Serien** und die Inferenz-Grenze
`SERVO_MAX_TARGET_GAP_RAD = 0,35` überschritten. Ohne Anpassung würde also
fast jede Aufnahme verworfen.

### 7.1 Konfiguration
- [x] `config.EXECUTION_MODE = "servo_j"` (heute `"auto"` → an der realen Anlage `online`!).
- [x] Override 0,5 als Standard für Aufnahme: `config.DEFAULT_OVERRIDE` (heute 0,2) bzw.
      eigener Wert für die Aufnahme; GUI-Vorgabe prüfen (`bc/gui/app.py` übernimmt `session.override`).
      Die Inferenz übernimmt den Override aus der Aufzeichnung und verweigert Abweichungen (`apps/infer.py`) — passt.
- [x] Warnung in `apps/record.py` („Override < 1.0 … VM: ~1 s Nachlauf bei 0.2“) an den Anlagenbefund anpassen
      (0,5: Nachlauf ~0,09 s — die zuerst genannten 0,15–0,43 s waren der Rechenfehler aus § 7.7, Bahn auf ~1 mm).
- [ ] `ONLINE_VERIFIED` bleibt `False`; Kommentare in `config.py`/`bc/execution.py` auf den neuen Stand.

### 7.2 Abbruchprüfungen (Verzug) — Recorder und Inferenz
- [x] **Kriterium b** (`bc/recorder.py`, „Sollwert wandert, Arm nicht“): Fehlalarm nach Stillstand.
      Der Arm bleibt nach dem Dwell 0,3–1,8 s stehen (§ 11.6). Vorschlag: b erst auslösen, wenn der Arm
      über ein längeres Fenster (≥ 2 s) nicht folgt, oder nach Stillstand/Dwell eine Anlaufzeit erlauben.
      Der tote Steuerkanal (Befund 01.10.: Arm bewegt sich gar nicht) wird so weiterhin erkannt, nur später.
- [x] ~~**Kriterium a**~~ entfällt (§ 7.7: höchstens 0,15 rad, Grenzen bleiben) — (`RECORDER_MAX_FOLLOW_ERROR_RAD = 0,30`) und **Inferenz** (`SERVO_MAX_TARGET_GAP_RAD = 0,35`):
      mit Rauschen bis 0,46 rad durch reinen Nachlauf. Optionen abwägen und entscheiden:
      (1) Vergleich gegen den um den gemessenen Nachlauf verzögerten Sollwert statt gegen den aktuellen,
      (2) Grenzen anheben (toter Kanal lag bei 1,1 rad; z. B. 0,5–0,6 rad),
      (3) Rauschen so begrenzen, dass Gelenk 6 langsamer wird (Plan heute bis 0,69 rad/s).
      Empfehlung: (1), sonst (2) mit Begründung im Kommentar.
- [x] Mit den Mitschrieben vom 16.10. und 19.10. nachweisen, dass die neuen Regeln keinen gesunden
      `linear o0.5`-Lauf mehr abbrechen und den toten Kanal (Daten 01.10. bzw. Test mit stehendem Arm im SimRobot)
      weiterhin erkennen. Werkzeug: `tools/report_labortag_2026_10_16.schleppfehler` als Vorlage.
- [x] Tests (`tests/test_recorder.py`, `tests/test_apps.py`) für beide Fälle.

### 7.3 Sicherheit
- [ ] **Override absprechen** (Betreuer/Sicherheitsverantwortliche): Override 0,5 ist **kleiner** als der
      Override 1,0 aller Messläufe, aber größer als der bisherige Code-Standard 0,2. Er begrenzt die
      Gelenkgeschwindigkeit der Steuerung; das Planertempo (`TRANSIT_SPEED_MS`, PTP ≤ 0,6 rad/s) bleibt die eigentliche Grenze.
- [ ] Bestehende Schutzschichten unverändert lassen: `SERVO_MAX_JOINT_SPEED_RADS = 1,0` (harter Filter),
      `POLICY_MAX_JOINT_SPEED_RADS = 0,8`, Kollisionsprüfung, Freigabewort `ANLAGE`, Not-Aus.
      Prüfen, dass eine Lockerung in 7.2 keine dieser Grenzen berührt.
- [ ] Die Hänger von ~210 ms (und einmal 1014 ms) bleiben bestehen (0,1–0,2 % der Aufrufe). Klären, ob der
      Servo-Filter bei einem Hänger einen Sprung erzeugen kann (nach dem Hänger springt der Sollwert um
      ~0,2 s Bahn) — bei 0,69 rad/s ≈ 0,14 rad, unter dem Filter, aber dokumentieren.
      *PC 19.10.:* Im Recorder springt nichts — nach dem Hänger geht der NÄCHSTE Zwischenschritt raus,
      der Rest der Bahn kommt nur später (§ 7.7). In `apps/infer.py` noch zu prüfen.

### 7.4 VM / offline
- [ ] `servo_j` läuft in der VM (anders als `online`). Einmal in der VM mit Override 0,5 messen
      (`python tools/check_servo_stream.py --move --laeufe linear-60-r0-o0.5,linear-60-r1-o0.5`):
      Nachlauf und Abbruchprüfungen. Achtung: in der VM glättet die Steuerung anders (§ 9), und Override 0,2
      gab dort ~1 s Nachlauf — VM-Daten sind kein Ersatz für Anlagendaten, aber die Pipeline ist dieselbe.

### 7.5 Nächster Anlagentag
- [ ] Nach 7.1–7.3: eine Aufnahme `apps/record.py --robot neura --real-robot --ausfuehrung servo_j --override 0.5 …`
      mit Rauschen (ohne Greifer `--no-gripper`), Handy H1 → Episode darf nicht verworfen werden,
      H1 ≈ 0,20–0,26 m/s².
- [ ] Danach eine Inferenz-Probefahrt mit einem kleinen Testmodell (oder Replay), Override aus der Aufzeichnung.
- [x] `Zittern-an-der-Anlage.md`: neuer Abschnitt 19.10. + Entscheidung (§ 11.8 geschlossen, § 12 neu).

### 7.6 Stand 19.10. nachmittags (an der Anlage umgesetzt, nicht committet)

**Umgesetzt** (`bc/config.py`, `bc/recorder.py`, `apps/record.py`, `tests/test_recorder.py`, `tests/test_apps.py`):
- `EXECUTION_MODE = "servo_j"`, `DEFAULT_OVERRIDE = 0.5` (Kommentare mit Begründung).
- Kriterium b mit Anlaufzeit `RECORDER_STALL_GRACE_S = 2.5` (bricht erst ab, wenn es 2,5 s in Folge anliegt)
  plus **Gesamtprüfung am Episodenende** (Arm hat sich insgesamt kaum bewegt, obwohl der Sollwert wanderte
  → verworfen; nötig, weil Bahnen < 2,5 s sonst nie geprüft würden). 3 neue Tests.
- Warnung in `apps/record.py` bei Override < 0,5 bzw. > 0,5 mit `servo_j`.
- Grenzen a / Inferenz **unverändert 0,30 / 0,35 rad** (kurz auf 0,55/0,60 gelockert, nach den echten
  Aufnahmen zurückgenommen, Entscheidung Anwender).
- Tests am Laptop: 361 bestanden, nur `test_rectify` rot.

**Echte Recorder-Episoden** (`data_vm/2026-10-19/ep_00000`–`ep_00005`, `--no-gripper`, Kameras sim,
Block `test`): alle 206 Schritte gefahren, Dauer 13,69–13,73 s (= Plan), **Ziel–Ist max 0,057–0,097 rad**,
mit den alten Regeln hätte keine Prüfung angeschlagen.
Handy H1 (Fenster 13,7 s ab Ende der Anfahrt): r0 **0,18**, Rauschen 0,92–1,0 **0,26–0,30** m/s².
Handy-Dateien: `… 2026-10-19 13-08-00.xlsx` (ep1), `… 13-21-28.xlsx` (ep2–ep5); ep0 nicht aufgenommen.

**Wichtig — Korrektur:** Die Nachrechnung aus den `check_servo_stream`-Mitschrieben (§ 7, Übersicht 19.10.:
Ziel–Ist bis 0,46 rad, Kriterium a/b schlagen an) **überschätzt den Nachlauf** um Faktor 3–6 gegenüber dem
Recorder. Vermutlich zeitlicher Versatz zwischen Plan und Messung in `tools/report_labortag_2026_10_16.schleppfehler`.
Am PC prüfen, bevor diese Zahlen weiter verwendet werden (auch § 11.3/11.6 in `Zittern-an-der-Anlage.md`).
→ **Geklärt am PC, § 7.7.**

**Offen für den PC:**
- [x] Zeitbasis in `report_labortag_2026_10_16.schleppfehler` prüfen (s. o.), Doku § 11 ggf. korrigieren. → § 7.7
- [x] Ist Kriterium b mit Anlaufzeit noch nötig? → bleibt vorerst als Absicherung (Anwender 19.10.), begründet ist sie nicht mehr.
- [x] 3b (nachlaufbereinigte Prüfung) entfällt vorerst — kein Bedarf bei 0,10 rad.
- [ ] VM-Messung mit Override 0,5 (§ 7.4).
- [ ] TCP/Tool-Daten `ZimmerLWR50` am Pendant prüfen (Orientierung im Tool-KS ändern, Fingerspitze muss stehen bleiben);
      Punkte sind noch Testpunkte → finales Teachen.
- [ ] Override 0,5 mit dem Betreuer abgestimmt (Anwender übernimmt) — Ergebnis hier vermerken.
- [ ] Hinweis aus `apps/record.py`: „FK über die Steuerung: Tool hinterlegt“ → ein RPC je Takt mehr; Takt wurde gehalten.

### 7.7 PC 19.10. abends: Zeitbasis geklärt

**Ursache der Überschätzung:** `check_servo_stream.fahre_lauf` sendet nach einem Hänger (~210 ms) das
nächste Profilstück. Der Rest der Bahn geht also später raus. `schleppfehler` und der Verzug in
`auswerten_lauf` verglichen mit der Uhrzeit (Ziel *i* ↔ Messung bei *i*/15 s), und jeder Hänger
zählte so als bleibender Nachlauf (je Lauf 0,1–1,2 s aufsummiert). Der Recorder misst direkt
nach dem Senden und war nie betroffen.

**Korrigiert** (`Zittern-an-der-Anlage.md` § 12): `check_servo_stream.profilzeit`/`hostzeit` (gegen das
Ende jedes Aufrufs), beide Werkzeuge stellen darauf um. `report_labortag_2026_10_16.py --mitschriebe <glob>`
rechnet Teil 2 für beliebige Tage. 2 neue Tests. Berichte 16./19.10. mit `--auswerten` neu gerechnet
(nur Verzug/Folgefehler geändert), Bild 14 neu. Kommentare in `bc/config.py`, `bc/recorder.py`,
`apps/record.py` und die Übersichten 16./19.10. sind angepasst.

| `servo_j` 16./19.10. | Uhrzeit (alt) | Sendezeit (neu) |
|---|---|---|
| Ziel–Ist max o0.5 r0 / r1 | 0,09–0,20 / 0,23–0,46 rad | **0,06–0,10 / 0,09–0,15 rad** |
| Kriterium a / b | a 3×, b fast immer | **nie** |
| Verzug o0.5 / o1 | 150–430 / 80–250 ms | **82–100 / 20–24 ms** |

Das deckt sich mit den Recorder-Episoden (0,057–0,097 rad). Die „Pause nach Stillstand“ (§ 11.6) ist
widerrufen. `RECORDER_STALL_GRACE_S = 2,5` bleibt als Absicherung.

**Nicht am PC:** `data_vm/2026-10-19/` (6 Episoden) und die Handy-Dateien vom 19.10. liegen nur auf dem
Laptop (gitignored bzw. OneDrive nicht synchron). Für einen Episodenbericht
(`tools/plot_episode.py`) und die H1-Nachrechnung herüberkopieren.

**Noch offen:**
- [x] VM-Messung mit Override 0,5 (§ 7.4) → § 7.8.
- [x] `apps/infer.py`: Verhalten nach einem Hänger (springt das Ziel?) → § 7.8, kein Sprung.
- [x] Anhang C in `Zittern-an-der-Anlage.md` (13.10., `check_trajectory_buffer`): `referenz_90/120` (88/246
      Überläufe, Verzug 396/600 ms) gleich betroffen → § 7.8.
- [ ] Tool-Daten `ZimmerLWR50`, finales Teachen, Override-Absprache (s. o.).

### 7.8 PC 21.10.: VM-Messung, Inferenz nach Hängern, 13.10.-Werte, Punkte über die API

**VM mit Override 0,5** (`is_robot_in_simulation() = True`, v5.0.0-alpha.102, `NoTool`, Ablauf-Arbeitskopie
des PCs mit 172 Zielen — am Laptop 206):

| Lauf | Bericht | Verzug | Ziel–Ist max | Kriterium a / b | Überläufe |
|---|---|---|---|---|---|
| `check_servo_stream` linear o0.5 r0 / r1 | `2026-10-21_09-28-53_Servo-Strom` | 64 / 70 ms | 0,036 / 0,053 rad | nie | 0 |
| `check_servo_stream` linear o1 r0 | dto. | 10 ms | 0,013 rad | nie | 0 |
| `apps/record.py`, Standardwerte (servo_j, 0,5), 3 Episoden, Rauschen 0,45–0,99 | `data_vm/2026-10-21/1_servo_j_o05`, PDF `2026-10-21_VM_servo_j_o05_Episoden.pdf` | – | 0,040–0,045 rad | nie, alle verwendbar | 0 |

Die VM läuft also etwas schneller nach als die Anlage (Override 0,5: 64–70 statt 82–100 ms), die Pipeline
mit den neuen Standardwerten ist in sich stimmig.

**Inferenz nach einem Hänger:** `apps/infer.py` zählt Takte, nicht die Uhr. Nach einem hängenden Aufruf geht
der nächste Zwischenschritt raus, wie im Recorder. Das Ziel springt nicht, der Rest der Fahrt kommt nur später.
Neuer Test `test_apps.test_infer_hang_delays_but_does_not_jump` (Hänger 1014 ms im SimRobot). Dafür braucht es
weder VM noch Anlage.

**13.10. (`check_trajectory_buffer`, Anhang C):** Das Werkzeug hat dasselbe Muster (Ziel *k* nach Takt
gezählt, Verzug gegen die Uhr). Die Mitschriebe vom 13.10. enthalten noch keine Sendezeiten, exakt
korrigieren lässt es sich also nicht. Rechnet man die gemessene Streckung (Dauer-Faktor 1,25 / 1,51) heraus,
fällt der Folgefehler von 6,1 / 14,6 auf 1,8 / 0,4 mrad. Die 396 / 600 ms für `referenz_90/120` sind also
Streckung durch die Überläufe, kein Nachlauf des Arms. An der damaligen Aussage („90/120 Hz schafft der
Laptop nicht“) ändert das nichts.

**Punkte über die API (für eine GUI-Funktion „Touch-up“), in der VM geprüft:**
- `create_point(name, target_joint_angles=q)` legt einen Punkt aus Gelenkwinkeln an — Rückprobe exakt
  (0,0 rad). Am 24.09. war die Gelenkliste positional als *Pose* übergeben worden; daher der IK-Fehler.
  Optional `tool_name=…`, sonst das aktive Tool.
- `update_pose_in_DB(datensatz)` ändert einen Punkt **in place** (gleiche ID, `createdAt` bleibt). Es müssen
  `jointAngles`, `position`, `euler` und `quaternion` zusammen gesetzt werden. Nur `jointAngles` zu ändern,
  wird stillschweigend ignoriert.
- Jeder Punkt trägt `createdAt`/`updatedAt` → „zuletzt gespeicherte Punkte in Reihenfolge“ ist lesbar.
- Der Punkt speichert **kein Tool**. Touch-up nur mit dem Aufnahme-Tool (`get_selected_tool_name`).
- Freedrive/Teach-Modus und IOs sind abfragbar (`is_robot_in_teach_mode`, `is_free_drive_mode_enabled`,
  `get_io_status`). In der VM gibt es keine IOs. Ob der Speichern-Knopf am Arm ein IO-Signal liefert oder
  nur in der Neura-Oberfläche einen Punkt anlegt, ist an der Anlage zu prüfen.

### 7.9 Touch-up-Knopf in der GUI (gebaut 21.10.)

**Bedienung:** Reiter „Aufnahme“ → Ablauf-Tabelle, neue Spalte **Touch-up** je Zeile. Mit der Steuerung
verbunden: Arm (z. B. per Freedrive am Pendant) an den Punkt bringen → **Touch-up** → Rückfrage zeigt Punkt,
Ziel (Simulation / **REALE ANLAGE**), Werkzeug, neue Gelenkstellung und die Änderung in mm/Grad → **Ja**.
Fehlt der Punkt in der Datenbank, wird er angelegt. Die Zeilen der Reihe nach angeklickt, entstehen so auch
die Punkte eines neuen Ablaufs. **Touch-up rückgängig** spielt die letzte Sicherung zurück; ein neu
angelegter Punkt wird dabei wieder gelöscht.

**Sperren** (`bc/touchup.py`, Werte in `bc/config.py`):
- Arm muss stillstehen (zwei Lesungen im Abstand von 0,2 s, ≤ 1 mrad).
- An der realen Anlage muss das Werkzeug `TOUCHUP_TOOL_NAME = "ZimmerLWR50"` aktiv sein (der Punkt speichert
  kein Tool). In der Simulation ist das nur ein Hinweis.
- Ab 50 mm / 15° Änderung fragt die Rückfrage, ob die Zeile stimmt.
- Während einer Aufnahme ist die Tabelle gesperrt. Namen mit `_` am Anfang sind reserviert (Hilfspunkt).

**Technik:** `NeuraRobot.store_point` ändert einen bestehenden Punkt **in place** (gleiche ID, gleicher Frame,
`createdAt` bleibt erhalten), über einen Hilfspunkt `_BC_TOUCHUP_HILFE`, der sofort wieder gelöscht wird.
Vor jedem Schreiben sichert die GUI den alten Datensatz nach `sequences/arbeitskopien/punkt_sicherungen/`
(gitignored).

**In der VM geprüft (21.10.):**
- GUI offscreen gegen die VM: anlegen, ändern (ID und `createdAt` bleiben), beide Rücknahmen, kein Hilfspunkt
  bleibt übrig.
- Arm um 0,05 rad verfahren und Touch-up: Datensatz um 22,0 mm verschoben (Plan 22,0 mm), Gelenke exakt.
  Die Rücknahme ist exakt.
- **`PICK` hängt am Frame `Base`, nicht an `World`.** Ein Testpunkt in `Base` behält beim Touch-up seinen Frame,
  und die Gegenprobe in `get_point` (Cartesian ↔ FK) bleibt bei 0,00 mm.
- Ein Befund beim Testen: Zwei Touch-ups in derselben Sekunde überschrieben dieselbe Sicherung. Behoben,
  mit Test.

**Noch an der Anlage:** Werkzeug `ZimmerLWR50` wählen, einmal Touch-up an einem Testpunkt mit Rücknahme. Danach
klären, was der Speichern-Knopf am Arm liefert (IO über `get_io_status`, oder nur ein Punkt in der Neura-Oberfläche).
Davon hängt ab, ob sich „Knopf am Arm → nächste Zeile“ bauen lässt.

### 7.10 GUI 21.10.: Punkte-Tabelle, „Z hoch“, Konsole

**Punkte-Tabelle** (Aufnahme): Sie hat keinen eigenen Bildlauf mehr, ihre Höhe folgt der Zeilenzahl. Gescrollt
wird nur noch der Reiter.

**▲ Z hoch (halten)** (Aufnahme, neben „Teachen …“): Der Arm fährt senkrecht nach oben (Basis-z), solange der
Knopf gedrückt ist. Gedacht für PRE_GRASP genau über PICK: PICK per Touch-up setzen, hochfahren, PRE_GRASP per
Touch-up setzen.
- API: `turn_on_jog(jog_velocity, jog_type="Cartesian")` → `jog(set_jogging_external_flag=1)` je Schritt
  (50 Hz) → `turn_off_jog()`. Die Steuerung fährt nur, solange `jog` kommt.
- Gemessen in der VM (21.10.): direkt am Adapter mit Anteil 0,3 und 50 Hz 14 mm/s bei Override 0,5 (27 mm/s bei 1,0),
  rein in z. Über die GUI 6,3 mm in 1 s plus 1,9 mm Auslauf. Hängt die Oberfläche, steht der Arm nach 1–3 mm.
- Sperren: nur im **Teach-Modus** (`is_robot_in_teach_mode`), nur mit Bewegungsfreigabe (VM bestätigt oder
  „Reale Anlage freigeben“). Hält an bei Loslassen, Maus außerhalb des Knopfs, Reiterwechsel, Fehler eines
  Aufrufs und nach 10 s am Stück. Während einer Aufnahme gesperrt. Tempo: `config.JOG_Z_FRACTION`.
- Software-Tippbetrieb, kein Zustimmtaster, kein Not-Aus.

**Konsole** (neuer Reiter): Eine Zeile `python …` aus Doku oder Chat einfügen und **Ausführen**. Sie läuft im
Ordner „Behavior Cloning“ mit dem Python der Oberfläche (`bc_env`). Dazu gibt es einen Verlauf und eine
Beispielliste. Über **Eingabe an das laufende Programm** gehen Antworten an das Programm, z. B. das
Freigabewort `ANLAGE`. **Stopp** beendet den Prozess.
- Nur Python-Befehle, eine Zeile je Start, keine Shell (`|`, `>`, `&&`, `cd` werden abgelehnt). Platzhalter
  `<…>` müssen vorher ersetzt werden.
- `--move`, Apps, die fahren, und `--real-robot` lösen eine Rückfrage aus. Die Sperren der Werkzeuge selbst
  bleiben bestehen.

Geprüft: Tests (381, nur ArUco rot), Offscreen-GUI gegen SimRobot und VM.

**Start per Doppelklick:** `BC-Oberflaeche.bat` im Ordner „Behavior Cloning“. Sie sucht das Python in dieser
Reihenfolge: `BC_PYTHON` → `E:\Enviroments\bc_env` (PC) → `C:\Enviroments\bc_env312` → `C:\Enviroments\bc_env`
(Laptop). Das Log-Fenster startet minimiert und bleibt bei einem Fehler offen. Argumente werden durchgereicht
(`--mode recording`). Für den Desktop: Rechtsklick → Senden an → Desktop (Verknüpfung erstellen).
Eine `.exe` (PyInstaller) wäre wegen torch/lerobot mehrere GB groß und müsste nach jeder Codeänderung neu
gebaut werden — die `.bat` nimmt immer den aktuellen Code.

**Offen an der Anlage:** „Z hoch“ im Teach-Modus testen. Dabei klären, ob die Steuerung für externes Tippen den
Zustimmtaster am Pendant verlangt und wie schnell es dort mit Anteil 0,3 fährt.

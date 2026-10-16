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

### 4.1 Code der Halteregel prüfen
- [ ] `git diff` der 12 Dateien durchsehen (Anwender + Claude). Besonders:
      `bc/recorder.py` (Warten vor dem Greiferbefehl, Pacer-Neustart),
      `apps/infer.py` (gleiche Regel beim Greiferwechsel der Policy),
      `bc/execution.py` (`wait_until_reached`, `lead_index(..., hold_end)`).
- [ ] `python tests/run_all.py` am PC.
- [ ] In der VM/SimRobot einmal fahren:
      `python tools/check_servo_stream.py --move --laeufe online-15-r0-o1,onlinehalt-15-r0-o1`
      (VM fährt `movelinear_online` evtl. nicht, siehe `bc/execution.resolve_mode` — dann SimRobot).
- [ ] Offene Punkte aus § 10.5 bewerten (Toleranz/Zeitlimit, nur Position statt Orientierung,
      Greiferbefehl in der Inferenz einen Takt früher).

### 4.2 Auswertung der Messdaten nachholen
- [ ] **`tools/report_handy.py` reparieren:** Uhrzeiten nach phyphox-Pause falsch
      (`report_zittern.handy_auswerten` nutzt nur den ersten START). Abschnitte über
      „Metadata Time“ (START/PAUSE-Paare) auf die Systemzeit abbilden; keine
      Interpolation über Pausen. Test dazu.
- [ ] Danach alle Handy-Dateien mit dem Werkzeug auswerten und mit den Zahlen in § 1 vergleichen
      (die wurden am Labortag mit einem Hilfsskript gerechnet).
- [ ] Alte Berichte nachrechnen, damit die Haltepunkt-Kennzahlen drinstehen:
      `python tools/check_servo_stream.py --auswerten Berichte/2026-10-16_<stempel>_Servo-Strom`
      für `12-47-00`, `13-00-44`, `14-01-59`, `14-15-21`, `14-18-59`.
- [ ] **Wo `online` sonst noch abweicht:** quer max 27 mm sitzt am PICK; prüfen, wie groß die
      Abweichung an den übrigen Punkten (APPROACH_01, PRE_GRASP, PRE_PLACE) ist und ob Orientierungswechsel
      zu früh beginnen. Ggf. Haltepunkte auch dort nötig?
- [ ] **Test 3 genauer:** 4 Takt-Überläufe und 14 ms Sendeabstand-Streuung bei `online` (sonst 0) —
      vermutlich ein ~210-ms-Hänger; im Mitschrieb `14-18-59` prüfen.
- [ ] **210-ms-Hänger** im `servo_j`/`online_j`-Aufruf: fest ~210 ms, zufällige Stelle — Verdacht
      Delayed-ACK/Nagle bzw. USB-LAN-Adapter (Energiesparen). Nur Vermutung; im Mitschrieb aller Läufe
      zählen und mit Aufrufzeiten belegen. Für `online` relevant, weil es künftig die Ausführungsart ist.
- [ ] Videos: Neura-Leuchte bei Test 3 (blau oder grün am PICK?) und Höhe am PICK sichten.

### 4.3 Dokumentation
- [ ] `Labortag-Messplan-Zittern.md` ausfüllen (Tabellen A–E, *Neue Infos*: Tisch ausgeklappt,
      kein H2, Punkte mit falschem Tool gespeichert → am Pendant mit `ZimmerLWR50` neu gespeichert).
- [ ] `Zittern-an-der-Anlage.md`: Messungen 16.10. ergänzen (Zahlen aus § 1, Bilder aus den Handy-Plots).
- [ ] `Environment-Laptop.md`: Tippfehler `C:\Enviromentsc_env` → `C:\Enviroments\bc_env`.

## 5. Nächster Anlagentag (vorbereiten)

Kurzer Block, ~20 min, Handy an H1, Video auf PICK:

```bat
python tools/check_servo_stream.py --move --real-robot --laeufe online-15-r0-o1,onlinehalt-15-r0-o1
```

danach eine Aufnahme mit `apps/record.py ... --ausfuehrung online` (mit Greifer, sobald
angebunden; sonst `--no-gripper`) und `reach_waits` in `meta.json` prüfen.
Abnahme laut § 10.4: tiefster Punkt ≤ 1 mm über PICK, Verweilzeit ≥ 0,5 s,
Wartezeit deutlich unter 3 s, H1 ≈ 0,20 m/s², Leuchte zeigt den Halt wie
beim Standardlauf. Danach `config.EXECUTION_MODE`/`ONLINE_VERIFIED` entscheiden.

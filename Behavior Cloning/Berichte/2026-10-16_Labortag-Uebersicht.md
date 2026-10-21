# Labortag 16.10.2026 — Übersicht aller Dateien

Zuordnung aller Berichte, Mitschriebe und Handy-Dateien des Tages, damit beim
Nachtragen in `Dokumentation/Labortag-Messplan-Zittern.md` nichts vertauscht
wird. Uhrzeiten = Laptop-Uhr bzw. phyphox-Systemzeit (aus „Metadata Time“).

* **Berichte/Mitschriebe:** dieser Ordner (`Berichte/2026-10-16_*`).
  Der Dateiname ist der Zeitpunkt, an dem der Bericht geschrieben wurde
  (Ende des Laufs), nicht der Start.
* **Handy (phyphox, H1 = Unterarm an Achse 5):**
  `C:\Users\jerem\OneDrive\Projekt\20_Dokumentation\Zittern\Beschleunigung mit g 2026-10-16 <hh-mm-ss>.xlsx`.
  Der Dateiname ist die **Exportzeit**. Nach Pause/Fortsetzen hängt phyphox an
  dieselbe Messung an, deshalb enthalten spätere Exporte die früheren.
  `tools/report_handy.py` rechnet Uhrzeiten nach einer Pause falsch (offener Punkt).
* **Video:** separat (drittes Handy), Uhrzeiten über die `LOS`-Zeiten unten zuordnen.
* **Kein H2** (Kamerastange): bewusst weggelassen, Fokus auf der Roboterbewegung.
* **Aufbau:** Tisch ausgeklappt (am 13.10. eingeklappt) — steifer.
* **Steuerung:** v5.0.8, `is_robot_in_simulation() = False`, Tool `ZimmerLWR50`,
  Laptop über USB-C-LAN-Adapter, Control-Box `192.168.2.15` (Ping < 1 ms).

---

## Vormittag: Systemcheck (GUI)

| Datei | Inhalt |
|---|---|
| `2026-10-16_11-42_Daheng-API.json` | Kamera-API-Check |
| `2026-10-16_11-44_RPC-Latenz.json` | RPC-Latenz |
| `2026-10-16_11-44_Trajektorien-Puffer.json` | nur Lesen, ohne Fahrt |
| `2026-10-16_11-45_Servo-Timing.json` | Servo-Timing |
| `2026-10-16_11-45_Trajektorien-Puffer.json` + `_Mitschrieb.npz` | Fahrtest real: referenz 60/90/120 Hz + online |
| `2026-10-16_11-45_Systemcheck.txt` | Gesamtbericht (336 Tests ok, 1 fehlgeschlagen) |
| `bericht_2026-10-16_1142.pdf` | PDF-Bericht dazu |

Handy: keins.

## A. Laptop-Takt (F1) — `check_trajectory_buffer.py --tests referenz --vergleich-raten 90,120`

| Bericht (+ `_Mitschrieb.npz`) | Bemerkung | 60 Hz Überl. / Streuung | `servo_j`-Aufruf Median / p95 |
|---|---|---|---|
| `2026-10-16_11-51_Trajektorien-Puffer.json` | GUI u. a. offen | 14 / 11,9 ms | 9,9 / 20,1 ms |
| `2026-10-16_11-52_Trajektorien-Puffer.json` | das meiste geschlossen | 16 / 12,3 ms | 10,6 / 21,8 ms |
| `2026-10-16_11-55_Trajektorien-Puffer.json` | nur cmd | 26 / 17,1 ms | 9,4 / 21,6 ms |

Handy: keins. Je Lauf 1–2 Hänger à ~210 ms im `servo_j`-Aufruf.

## B. Eigenfrequenz (F2) — Anstoßen, keine Fahrt

Reihenfolge je 3 Schläge: Achse 4 seitlich, Achse 4 von oben, Achse 2, Plattform.

| Handy-Datei | Zeit | Anregung | Spitze |
|---|---|---|---|
| `… 12-10-51.xlsx` | 12:09:02–12:10:23 | Hand | 5,6–6,2 Hz (Plattform teils 14,9 Hz) |
| `… 12-16-54.xlsx` | 12:14:36–12:16:51 | Gummihammer | 5,6–6,2 Hz |

## C. Kleine J1-Bahn linear gegen pos — `check_trajectory_buffer.py --vergleich-raten "" --servo-profil …`

| Bericht (+ `_Mitschrieb.npz`) | Lauf | Handy-Zeit (Bewegung) | H1 RMS |
|---|---|---|---|
| `2026-10-16_12-18_Trajektorien-Puffer.json` | C1 linear | ~12:18:07 | 0,65 |
| `2026-10-16_12-18-43_Trajektorien-Puffer.json` | C1 pos | ~12:18:37 | 0,43 |
| `2026-10-16_12-24_Trajektorien-Puffer.json` | C2 linear | ~12:24:08 | 0,63 |
| `2026-10-16_12-24-27_Trajektorien-Puffer.json` | C2 pos | ~12:24:22 | 0,47 |

Handy: `… 12-42-37.xlsx` (Abschnitte 12:17:54–12:19:40 und 12:24:04–12:24:32).
`… 12-22-31.xlsx` = nur C1, **Duplikat** (steckt komplett in `12-42-37`).

## Abgebrochene D-Versuche (Punkte mit falschem Tool gespeichert, keine Fahrt)

| Bericht | Fehler |
|---|---|
| `2026-10-16_12-19-11_Servo-Strom.json` | `CLEAR_FOV`: Cartesian-Pose 0,1975 m daneben |
| `2026-10-16_12-20-50_Servo-Strom.json` | dito |
| `2026-10-16_12-21-26_Servo-Strom.json` | dito |
| `2026-10-16_12-32-51_Servo-Strom.json` | `APPROACH_01`: dito |

Handy: `… 12-42-37.xlsx`, Abschnitt 12:32:36–12:32:54 (Ruhe, keine Bewegung).
Danach Punkte am Pendant mit Tool `ZimmerLWR50` neu gespeichert.

## Neura-Standardlauf (Pendant-Programm, echte Punkte) — Referenz

| Handy-Datei | Zeit | H1 RMS / p99 |
|---|---|---|
| `… 13-00-57.xlsx`, Abschnitt 1 | 12:42:44–12:43:16 (Bewegung ~12:42:59–12:43:13) | **0,18 / 0,50** |

Kein Bericht (nicht über die Werkzeuge gefahren).

## D. Ganze Aufnahmebahn — `check_servo_stream.py --laeufe linear-60-r0-o1,pos-60-r0-o1,online-15-r0-o1,linear-60-r0-o0.5,pos-60-r1-o1,online-15-r1-o1`

Config: online 1.3 / 1.5, `GRIPPER_DWELL_S` 0.5.

| Bericht (+ `_Mitschrieb.npz`) | Reihe | LOS-Zeiten (in Lauf-Reihenfolge) |
|---|---|---|
| `2026-10-16_12-47-00_Servo-Strom.json` | D1 | 12:44:36 · 12:45:01 · 12:45:25 · 12:45:55 · 12:46:18 · 12:46:42 |
| `2026-10-16_13-00-44_Servo-Strom.json` | D2 | 12:58:22 · 12:58:46 · 12:59:10 · 12:59:39 · 13:00:03 · 13:00:26 |

Handy: `… 13-00-57.xlsx` (Abschnitte 12:44:13–12:47:09 = D1, 12:58:06–13:00:54 = D2).
`… 12-49-01.xlsx` = Neura-Lauf + D1, **Duplikat** (steckt komplett in `13-00-57`).

| Lauf | H1 RMS D1 / D2 | quer RMS / max (mm) | Verzug (ms)¹ |
|---|---|---|---|
| linear-60-r0-o1 | 1,98 / 2,05 | 0,4 / 1,2–2,1 | ~20 |
| pos-60-r0-o1 | 0,89 / 0,84 | 0,5 / 1,4–1,5 | ~90 |
| online-15-r0-o1 | 0,20 / 0,20 | 11,6 / 27 | ~1300 |
| linear-60-r0-o0.5 | 0,22 / 0,21 | 0,5 / 1,3–1,7 | ~85 |
| pos-60-r1-o1 | 0,86 / 0,85 | 0,5 / 1,4–1,7 | ~95 |
| online-15-r1-o1 | 0,20 / 0,20 | 13,7 / 39 | 1200–1240 |

¹ Gegen die Sendezeit, neu gerechnet am PC 19.10. (`Zittern-an-der-Anlage.md` § 12).
Vorher gegen die Uhrzeit: linear o1 ~240, pos 140–340, linear o0.5 150–250 ms. Die
Hänger der Aufrufe zählten dabei als Nachlauf.

## Nachmittag: PICK-Höhe bei online (Tests 1–3)

| Bericht (+ `_Mitschrieb.npz`) | Test | Config (Tempo / Beschl. / Halt) | LOS | tiefster Punkt ggü. PICK | H1 RMS |
|---|---|---|---|---|---|
| `2026-10-16_14-01-59_Servo-Strom.json` | Test 1: pos + online | 1.3 / 1.5 / 0.5 s | pos 14:01:18 · online 14:01:41 | pos +0,1 mm · online **+27,3 mm** | pos 0,82 · online 0,21 |
| `2026-10-16_14-03-10_Servo-Strom.json` | Wiederholung online (gedacht als Test 2, Config nicht umgestellt) | 1.3 / 1.5 / 0.5 s | 14:02:52 | +27,3 mm | 0,20 |
| `2026-10-16_14-05-06_Servo-Strom.json` | Wiederholung online + pos (gedacht als Test 3, Config nicht umgestellt) | 1.3 / 1.5 / 0.5 s | online 14:04:25 · pos 14:04:50 | online +27,3 mm | online 0,20 · pos 0,83 |
| `2026-10-16_14-15-21_Servo-Strom.json` | **Test 2: schnelleres online** | **2.0 / 3.0** / 0.5 s | 14:15:05 | **+0,1 mm**, 0,66 s unten | **0,67** (15-Hz-Spitze) |
| `2026-10-16_14-18-59_Servo-Strom.json` | **Test 3: längerer Halt** | 1.3 / 1.5 / **1.5 s** | 14:18:40 | **+0,2 mm**, 0,33 s unten | **0,26** |

Handy:
* `… 14-05-57.xlsx`: 14:00:45–14:05:55 (Test 1 und die beiden Wiederholungen)
* `… 14-19-07.xlsx`: 14:14:46–14:15:29 (Test 2) und 14:18:20–14:19:05 (Test 3)

Beobachtung Anwender:
* Test 2: kam tiefer, kein hörbares Brummen, an den Ecken etwas ruckiger.
* Test 3: kam deutlich weiter runter, sehr glatt, Licht vermutlich grün (im Video prüfen).

`config.py` danach wieder auf dem Ausgangsstand (1.3 / 1.5 / 0.5), git-sauber.

---

## Handy-Dateien auf einen Blick

| Datei | brauchen? | Inhalt |
|---|---|---|
| `… 12-10-51.xlsx` | ja | B Hand |
| `… 12-16-54.xlsx` | ja | B Gummihammer |
| `… 12-22-31.xlsx` | nein (Duplikat) | C1 |
| `… 12-42-37.xlsx` | ja | C1, C2, abgebrochener D-Versuch |
| `… 12-49-01.xlsx` | nein (Duplikat) | Neura-Lauf, D1 |
| `… 13-00-57.xlsx` | ja | Neura-Lauf, D1, D2 |
| `… 14-05-57.xlsx` | ja | Test 1 + Wiederholungen |
| `… 14-19-07.xlsx` | ja | Test 2, Test 3 |

# Labortag 19.10.2026 — Übersicht und Ergebnis

Vergleich `servo_j` linear Override 0,5 gegen `online` / `onlinehalt`
(Halteregel aus `Zittern-an-der-Anlage.md` § 10) an der Anlage, dreimal
dieselbe Serie. Vorgeschichte: `2026-10-16_Labortag-Uebersicht.md`,
`Zittern-an-der-Anlage.md` § 11.

* **Steuerung:** v5.0.8, `is_robot_in_simulation() = False`, Tool `ZimmerLWR50`.
* **Config:** Stand nach § 10 (online ×1,3/×1,5, Dwell 0,5 s, Halteregel 1 mm / 3 s), unverändert.
* **Handy H1:** Unterarm an Gelenk 5, Befestigung wie am 16.10.
  Dateien: `C:\Users\jerem\OneDrive\Projekt\20_Dokumentation\Zittern\Beschleunigung mit g 2026-10-19 <hh-mm-ss>.xlsx`
  (Dateiname = Exportzeit; Originale noch in Downloads).
* **Video:** separat, über die LOS-Zeiten zuordnen.

Befehl (alle drei Serien gleich):

```bat
python tools/check_servo_stream.py --move --real-robot --laeufe linear-60-r0-o0.5,linear-60-r0-o1,online-15-r0-o1,onlinehalt-15-r0-o1,linear-60-r1-o0.5,onlinehalt-15-r1-o1
```

## Serien und Dateien

| Serie | Aufbau | Bericht (+ `_Mitschrieb.npz`) | Handy | LOS-Zeiten (Lauf 1–6) |
|---|---|---|---|---|
| S1 | wie 16.10. (hinterer Fuß lose) | `2026-10-19_12-22-07_Servo-Strom.json` | `… 12-23-09.xlsx` (Abschnitte ab 12:19:02, 12:22:08) | 12:19:39 · 12:20:03 · 12:20:27 · 12:20:52 · 12:21:23 · 12:21:47 |
| S2 | Fuß festgestellt **und** Plattform von Hand gehalten | `2026-10-19_12-26-46_Servo-Strom.json` | `… 12-31-07.xlsx` (ab 12:23:40, 12:27:00) | 12:24:18 · 12:24:42 · 12:25:06 · 12:25:31 · 12:26:02 · 12:26:26 |
| S3 | Fuß festgestellt, nicht gehalten | `2026-10-19_12-34-37_Servo-Strom.json` | `… 12-34-53.xlsx` (ab 12:31:39, 12:34:47) | 12:32:09 · 12:32:33 · 12:32:57 · 12:33:23 · 12:33:54 · 12:34:17 |

Lauf-Reihenfolge: `linear-60-r0-o0.5`, `linear-60-r0-o1`, `online-15-r0-o1`,
`onlinehalt-15-r0-o1`, `linear-60-r1-o0.5`, `onlinehalt-15-r1-o1`.

## Ergebnis

H1 = RMS über 5 Hz in m/s², Fenster LOS bis nächste LOS − 2 s (letzter Lauf LOS + 18 s).
Neura-Standard (16.10.): 0,16–0,18.

| Lauf | H1 S1 / S2 / S3 | p99 | Bahn quer max | Folgefehler RMS | PICK | Fahrt |
|---|---|---|---|---|---|---|
| **linear o0.5** | **0,19 / 0,19 / 0,20** | 0,46 | **1,2 mm** | 2–4 mrad | exakt | **11,6 s** |
| linear o1 | 1,74 / 1,55 / 1,66 | ~4,2 | 1,8–2,3 mm | 2–3 mrad | exakt | 11,9 s |
| online | 0,20 / 0,20 / 0,20 | 0,60 | 27 mm | ~106 mrad | **+27 mm** | 13,4 s |
| onlinehalt | 0,21 / 0,21 / 0,22 | 0,82 | 27 mm (Ecken) | ~107 mrad | **+0,2 mm**, 1,56–1,62 s gewartet | 14,1 s |
| linear r1 o0.5 | 0,24 / 0,25 / 0,26 | 0,59 | 5 mm | 4–9 mrad | – | 12,3 s |
| onlinehalt r1 | 0,27 / 0,27 / 0,29 | 1,05 | 39 mm | ~116 mrad | +0,2 mm | 15,4 s |

* **Override 0,5 halbiert das Tempo nicht** (Fahrt 11,6 s wie Override 1). Er wirkt
  auf das Nachführen in der Steuerung, nicht auf die Bahnzeit.
* **`onlinehalt`** erreicht PICK reproduzierbar (+0,2 mm, ~1,6 s Warten), kürzt aber
  weiter Ecken/Drehungen ab und zeigt 50–60 % Energie bei 15 Hz (Anwender: „abtauchen
  bzw. schneller werden bei neuen Befehlen“).
* **Aufbau:** Fuß festgestellt / Plattform gehalten → praktisch kein Einfluss
  (nur linear o1 ~10 % weniger). Der Aufbau ist nicht der Engpass.
* **Beobachtung Anwender:** linear-Läufe sahen gut aus; mit Rauschen viel (gewollte)
  Bewegung, aber kein Zittern im Verhältnis zur Bewegung. online mit Rauschen glatt,
  bekanntes Abtauchen.

## Nachgestellte Abbruchprüfungen (Recorder-Logik, `tools/report_labortag_2026_10_16.schleppfehler`)

> **Korrigiert am PC 19.10.** (`Zittern-an-der-Anlage.md` § 12): Die erste Fassung
> verglich gegen die Uhrzeit statt gegen die Sendezeit. Nach jedem ~210-ms-Hänger
> lag der Rest der Bahn dann scheinbar zurück. Sie meldete deshalb 0,10–0,46 rad,
> Kriterium a bei `linear r1 o0.5` (S1, S2) und Kriterium b in allen
> `linear`-Läufen. Folgefehler RMS oben ist ebenfalls neu gerechnet.

| Lauf | max Ziel–Ist (rad) | Kriterium a (> 0,30 rad, 4 Takte) | Kriterium b (Soll wandert, Arm nicht) |
|---|---|---|---|
| linear o0.5 | **0,06–0,10** | nie | nie |
| linear o1 | 0,03 | nie | nie |
| linear r1 o0.5 | **0,09–0,15** | nie | nie |
| online / onlinehalt | 0,78–0,83 | Takt 22–25 | nie (einmal Takt 4, S2 onlinehalt) |

* Das passt zu den echten Recorder-Episoden des Nachmittags (Ziel–Ist max
  0,057–0,097 rad, Übergabe § 7.6). Verzug gegen die Sendezeit: Override 0,5
  82–100 ms, Override 1 20–24 ms, konstant über die Fahrt.
* Neu rechnen: `python tools/report_labortag_2026_10_16.py --mitschriebe "Berichte/2026-10-19_*_Mitschrieb.npz"`.

## Entscheidung

**Anwender 19.10.: `servo_j` linear mit Override 0,5** für Aufnahme und Inferenz.
Offene Anpassungen: siehe `Dokumentation/Labortag-2026-10-16-Uebergabe.md`, Abschnitt „Umsetzung linear o0.5“.

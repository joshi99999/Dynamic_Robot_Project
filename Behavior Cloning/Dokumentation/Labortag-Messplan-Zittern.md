# Labortag — Messplan Zittern (Ausführungsart festlegen)

Angelegt **14.10.2026**. Arbeitsblatt für den nächsten Anlagentermin:
von oben nach unten abarbeiten, Ergebnisse **direkt hier** eintragen
(Felder `…`), neue Beobachtungen unten unter *Neue Infos* anhängen.
Hintergrund und Messungen bis 14.10.: `Zittern-an-der-Anlage.md`
(§ 4–6 Anlage 13.10., § 9 VM 14.10.). Allgemeiner Anlagenablauf:
`Anlagentag-Checkliste.md`.

**Ziel des Tages:** entscheiden, mit welcher Ausführungsart aufgenommen
wird — `online` (heute `auto`), `servo_j_pos` (neu, VM-Favorit) oder
`servo_j` mit höherer Rate — und die drei Fragen klären:

| # | Frage | beantwortet in |
|---|---|---|
| F1 | Ist der Laptop an der Anlage ein Problem (Streuung im Takt)? | A |
| F2 | Wo liegt die Eigenfrequenz des Aufbaus (Arm, Plattform, Kamerastange)? | B |
| F3 | Glättet die Anlagen-Steuerung (v5.0.8) `servo_j` ohne Geschwindigkeit wie die VM — und ist das ruhiger als `online`? | C, D |

**Datum:** 16.10.2026 **Beteiligte:** Anwender **Steuerung (Version):** v5.0.8, Tool `ZimmerLWR50`, Control-Box `192.168.2.15`
**Laptop am Netzteil, „Beste Leistung“:** ja, USB-C-LAN-Adapter (Ping < 1 ms) **Override:** 1,0 (D: ein Lauf je Reihe mit 0,5)

> **Ausgefüllt am PC (16.10. abends)** aus `Berichte/2026-10-16_*`, den
> Handy-Dateien und `Berichte/2026-10-16_Labortag-Uebersicht.md`. Die
> Auswertung am PC (`tools/report_labortag_2026_10_16.py`) steht in
> `Zittern-an-der-Anlage.md` § 11 — sie ändert die Bewertung von `online`.

---

## 0. Vorbereitung (vor dem Termin, am PC/Laptop)

| ✓ | Schritt | Wie | bestanden, wenn |
|---|---|---|---|
| [ ] | Code-Stand auf dem Laptop | Stand mit `tools/check_servo_stream.py`, `tools/report_handy.py`, Ausführung `servo_j_pos` | `python tests/run_all.py`: alles grün außer `test_rectify` |
| [ ] | VM-Gegenprobe am **PC** (schon erledigt 14.10.) | `python tools/check_servo_stream.py --move --matrix kurz` | `pos-60-r0-o1` Welligkeit quer ≈ 0,09 mm, `linear-60-r0-o1` ≈ 0,46 mm |
| [ ] | **Keine VM auf dem Laptop starten** | — | an der Anlage rechnet die Control-Box; die VM auf dem Laptop war am 14.10. der Grund für 14–64 ms je `servo_j` |
| [ ] | Zwei Handys mit phyphox („Beschleunigung mit g“), geladen, Klebeband | — | Abtastrate ≥ 400 Hz in phyphox |
| [ ] | Handy-Uhrzeit = Laptop-Uhrzeit (Netzzeit) | beide Uhren vergleichen | Abweichung < 1 s (Zuordnung über die `LOS`-Zeiten) |
| [ ] | Video (drittes Handy/Kamera) | Stativ, Arm und Kamerastange im Bild | — |

Handy-Positionen für **alle** Messungen gleich lassen (sonst sind RMS-Werte
nicht vergleichbar):

| Handy | Ort | Befestigung (Foto machen) |
|---|---|---|
| H1 | Unterarm an Gelenk 5 (wie 13.10.) | Klebeband, wie 13.10. |
| H2 | Kamerastange, Höhe der Kamera | **nicht verwendet** — Kamerastange/Aufbau sind Hardware-Themen, nicht Teil dieser Arbeit |

---

## 1. Sicherheit und Ausgangslage (ohne Bewegung)

| ✓ | Schritt | Wie | Ergebnis |
|---|---|---|---|
| [x] | Steuerung lesen | GUI *Steuerung prüfen (nur lesen)* | `is_robot_in_simulation()` = `False`, Version v5.0.8, Tool `ZimmerLWR50` (Systemcheck `2026-10-16_11-45`) |
| [ ] | Arbeitsraum frei, Not-Aus in Reichweite | — | — |
| [x] | Punkte der Aufnahmebahn vorhanden | GUI *Punkte sichern* | vorhanden (`sequences/points_2026-10-16_11-44.json`) — **aber mit falschem Tool gespeichert**, D brach viermal ab (`CLEAR_FOV`/`APPROACH_01` 0,1975 m daneben); am Pendant mit `ZimmerLWR50` neu gespeichert |
| [ ] | Arm in freie Pose (für A–C) | Pendant | Pose notiert: `…` |

---

## A. Laptop-Takt an der Anlage (F1) — ~5 min

Kleine J1-Bahn, servo_j wie heute, 60/90/120 Hz. Neu im Bericht: die
**Sendezeitpunkte** (Zeile „Sendeabstand“ in der Ausgabe, im JSON
`sendetakt`).

```bat
python tools/check_trajectory_buffer.py --move --real-robot --tests referenz --joint 1 --amplitude 0.05 --dauer 4 --vergleich-raten 90,120
```

| Rate | Takt-Überläufe | Sendeabstand Streuung (ms) | p95-Abweichung (ms) | größter (ms) | `servo_j` Median (ms) |
|---|---|---|---|---|---|
| 60 Hz | 14 / 16 / 26 | 11,9 / 12,3 / 17,1 | 4,5 / 6,6 / 8,3 | 208 / 209 / 214 | 9,9 / 10,6 / 9,4 (p95 20–22) |
| 90 Hz | 59 / 70 / 71 | 5,3 / 5,6 / 11,6 | 13,0 / 10,8 / 9,9 | 40 / 52 / 211 | 9,6 / 11,2 / 9,6 |
| 120 Hz | 237 / 209 / 189 | 13,2 / 5,5 / 4,9 | 14,6 / 13,9 / 12,3 | 212 / 47 / 36 | 9,4 / 9,9 / 9,8 |

Bericht: `Berichte/2026-10-16_11-51` (GUI offen) / `11-52` (das meiste zu) / `11-55` (nur cmd) `_Trajektorien-Puffer.json`.
Fahrzeit 60 Hz 1,06–1,11 × Plan, 90 Hz 1,16–1,20, 120 Hz 1,36–1,41.

**Lesen:** Am PC (VM) lag die Streuung bei 0,2–0,4 ms. Liegt sie an der
Anlage bei 60 Hz **unter ~1 ms** und gibt es fast keine Überläufe → der
Laptop ist **nicht** die Ursache (F1 erledigt). Deutlich mehr → mit
geschlossener GUI und ohne andere Programme wiederholen; bleibt es, gehört
der Laptop-Takt in die Bewertung jeder servo_j-Variante.

**Ergebnis F1:** Der **Laptop plant pünktlich** (Verspätung gegen den Pacer
median ~0,4 ms, egal ob Programme offen sind). Engpass ist der
`servo_j`-RPC der Steuerung: ~10 ms median, p95 ~21 ms, dazu 1–2 Hänger à
~210 ms je Lauf (sie machen die große Streuung). 60 Hz knapp haltbar,
90/120 Hz nicht. Die Hänger treffen nur Befehlsaufrufe, nie den parallelen
Lese-Thread → Netz/Adapter unwahrscheinlich (`Zittern-an-der-Anlage.md` § 11.7).

---

## B. Eigenfrequenz (F2) — ~5 min, keine Bewegung der Steuerung

H1 und H2 aufnehmen lassen. Arm steht (bestromt, Bremsen wie im Betrieb).
Je dreimal mit der flachen Hand kurz anstoßen, jeweils 5 s ausschwingen
lassen:

1. Unterarm seitlich
2. Unterarm von oben
3. Kamerastange seitlich
4. Plattform / Gestell

Datei: `Berichte/<Datum>_Handy-Anstoss_<H1|H2>.xlsx`

```bat
python tools/report_handy.py Berichte/<Datum>_Handy-Anstoss_H1.xlsx
```

| Anstoß (je 3×, Hand 12:09–12:10, Gummihammer 12:14–12:16) | H1 Spitze (Hz) | H2 Spitze (Hz) | Ausschwingen sichtbar? |
|---|---|---|---|
| Achse 4 seitlich | 5,9–6,2 | – | ja |
| Achse 4 von oben | 5,6–5,8 (einmal 15,0) | – | ja |
| Achse 2 | 6,3 | – | ja |
| Plattform | 5,8 (teils 14,9) | – | ja |

Handy: `Beschleunigung mit g 2026-10-16 12-10-51.xlsx` (Hand),
`… 12-16-54.xlsx` (Gummihammer, gleiche Frequenzen). Tisch heute
**ausgeklappt** (13.10. eingeklappt) → steifer, nur eingeschränkt
vergleichbar.

**Lesen:** Schwingt es bei **6–8 Hz** aus, ist das die Eigenfrequenz aus
dem 13.10. (Bild 3/12). Die VM zeigte, dass die Steuerung den
servo_j-Strom mit einer Schwankung bei **6–12 Hz** umsetzt (§ 9.4) — liegt
die Eigenfrequenz dort, verstärkt der Aufbau genau das.
Hinweis: die Anstoß-Spitzen erscheinen in `report_handy.py` als „Fahrten“,
falls die Lage sich ändert; sonst Spektrum im Zeitfenster ansehen.

**Ergebnis F2:** Eigenfrequenz **5,6–6,2 Hz** (Hand und Gummihammer gleich).
Liegt im Band, in dem die Steuerung den `servo_j`-Strom schwanken lässt
(6–12 Hz, § 9.4) — `servo_j` linear regt genau dort an (H1-Spitze 8,3 Hz).

---

## C. Erste Probe `servo_j_pos` auf der kleinen Bahn (F3, Sicherheit) — ~5 min

**Warum vorsichtig:** In der VM glättet die Steuerung ohne Geschwindigkeit.
Tut die Anlagen-Steuerung das **nicht**, heißt v = 0 „an jedem Sollwert
anhalten“ — dann wird es **lauter**, nicht leiser. Deshalb zuerst die
kleine J1-Bahn (0,05 rad), H1 aufnehmen, Hand an der Freigabe.

```bat
python tools/check_trajectory_buffer.py --move --real-robot --tests referenz --joint 1 --amplitude 0.05 --dauer 4 --vergleich-raten "" --servo-profil linear
python tools/check_trajectory_buffer.py --move --real-robot --tests referenz --joint 1 --amplitude 0.05 --dauer 4 --vergleich-raten "" --servo-profil pos
```

| Lauf | LOS / Uhrzeit | Gehör / Sicht | H1 RMS (m/s²) | Verzug (ms) | neu: Welligkeit (%) |
|---|---|---|---|---|---|
| linear (C1 / C2) | 12:18:06 / 12:24:08 | unruhig | 0,70 / 0,71 (Spitze 10,7 / 10,2 Hz) | 214 / 122 | 52 / 43 |
| pos (C1 / C2) | 12:18:38 / 12:24:22 | ruhiger | 0,46 / 0,55 | 120 / 152 | 33 / 36 |

H1 aus der automatischen Erkennung von `report_handy.py` (Datei
`… 12-42-37.xlsx`; die Übersicht nennt mit anderer Abgrenzung 0,65/0,63
und 0,43/0,47). Berichte `2026-10-16_12-18`, `12-18-43`, `12-24`,
`12-24-27`.

**Abbruchregel:** `pos` hörbar lauter oder ruckartiger als `linear` → D ohne
`pos` fahren, Ergebnis unten eintragen (dann ist die VM hier kein guter
Stellvertreter).

**Ergebnis C:** `pos` ist an der Anlage **nicht** lauter (Abbruchregel
greift nicht) — rund ein Drittel weniger Zittern als `linear`, weiter mit D.

---

## D. Kernmessung: ganze Aufnahmebahn (F3) — ~15 min

Die echte Aufnahmebahn (Arbeitskopie `pick_to_station`), **ohne Objekt,
Greifer schaltet nicht**, Rauschen 0 und 1, jeder Lauf mit 3 s Ruhe davor
(`LOS hh:mm:ss` in der Ausgabe). H1 **und** H2 aufnehmen, Video laufen
lassen. Das Werkzeug fragt nach dem Freigabewort `ANLAGE`.

```bat
python tools/check_servo_stream.py --move --real-robot --laeufe linear-60-r0-o1,pos-60-r0-o1,online-15-r0-o1,linear-60-r0-o0.5,pos-60-r1-o1,online-15-r1-o1
```

Optional, wenn A bei 120 Hz **0 Überläufe** zeigte:

```bat
python tools/check_servo_stream.py --move --real-robot --laeufe spline-120-r0-o1,linear-120-r0-o1
```

`pos` nur, wenn C nicht abgebrochen hat — sonst aus der Liste nehmen.

Auswertung direkt danach (Werkzeug-Tabelle steht am Ende der Ausgabe;
Handy-Dateien nach `Berichte/` kopieren):

```bat
python tools/report_handy.py Berichte/<Datum>_Handy-Ganzbahn_H1.xlsx
python tools/report_handy.py Berichte/<Datum>_Handy-Ganzbahn_H2.xlsx
```

| Lauf | LOS | H1 RMS | H1 p99 | H1 Anteil 6–8,5 / 8,5–11 / 14–17 Hz | H2 RMS | Welligkeit quer (mm) | Abstand quer (mm) | Verzug (ms) | Gehör |
|---|---|---|---|---|---|---|---|---|---|
| linear-60-r0-o1 | 12:44:36 / 12:58:22 | 1,95 / 2,05 | 4,9 / 4,7 | 0,69/0,19/0,03 | – | 0,38 / 0,42 | 0,37 / 0,42 | 238 / 246 | laut |
| pos-60-r0-o1 | 12:45:01 / 12:58:46 | 0,90 / 0,85 | 2,7 / 2,8 | 0,55/0,19/0,13 | – | 0,20 / 0,21 | 0,50 / 0,48 | 330 / 138 | unruhig |
| online-15-r0-o1 | 12:45:25 / 12:59:10 | 0,20 / 0,20 | 0,63 / 0,63 | 0,46/0,07/0,22 | – | 1,26 / 1,26 | **11,6 / 11,6** | **1308 / 1310** | sehr glatt |
| linear-60-r0-o0.5 | 12:45:55 / 12:59:39 | 0,22 / 0,20 | 0,50 / 0,54 | 0,74/0,04/0,04 | – | 0,17 / 0,11 | 0,55 / 0,50 | 252 / 148 | ruhig |
| pos-60-r1-o1 | 12:46:18 / 13:00:03 | 0,85 / 0,87 | 2,6 / 2,3 | 0,55/0,15/0,13 | – | 0,27 / 0,25 | 0,49 / 0,47 | 198 / 340 | unruhig |
| online-15-r1-o1 | 12:46:42 / 13:00:26 | 0,19 / 0,20 | 0,61 / 0,63 | 0,07/0,03/0,55 | – | 2,27 / 2,22 | **13,6 / 13,7** | 1208 / 1252 | glatt |

Werte D1 / D2. H1 über Zeitfenster (LOS + Bewegungsdauer laut Mitschrieb,
`tools/report_labortag_2026_10_16.py`); Anteile hier 5–8,5 / 8,5–11 /
14–17 Hz, D1. Neura-Standardlauf (Pendant, 12:42:59–12:43:13): H1 **0,16**,
p99 0,46.
Bericht: `Berichte/2026-10-16_12-47-00_Servo-Strom.json` (D1),
`…_13-00-44_…` (D2) · Handy: `… 13-00-57.xlsx` · Video: noch vom
Video-Handy holen.

**Lesen:**

* **Zittern** zählt am Handy (H1 RMS), **Bild/Objekt** an H2 — beides vor
  der Bahnwelligkeit.
* `online` vom 13.10. (Testbahn): H1 0,43 m/s², `servo_j` 1,80 m/s².
  Kommt `pos` auf ≲ `online` und hat keine Spitze bei 8,5–11 Hz → `pos`
  ist der Kandidat.
* Bahntreue: Abstand quer im Anflug (`PRE_GRASP → PICK`) — Grenze vorher
  festlegen: **< 5 mm**. VM: `pos` ~1,2 mm, Verzug ~70 ms.
* Die Werkzeug-Kennzahlen (Welligkeit, Spektrum) sind an der Anlage
  verrauschter als in der VM (Lesedauer ~7 ms statt ~2 ms) — nur Lauf
  gegen Lauf lesen.

**Ergebnis F3:** Die Anlagen-Steuerung glättet `servo_j` ohne
Geschwindigkeit nur teilweise: `pos` halbiert das Zittern gegenüber
`linear`, bleibt aber ~4× über `online` → `servo_j_pos` verworfen (nur VM).
`online` und `servo_j` linear mit **Override 0,5** sind so ruhig wie der
Neura-Standard (0,20–0,22 gegen 0,16). **Nachrechnung am PC:** `online`
fährt dabei bis 49 mm / 31° neben der Planbahn und würde von Recorder und
Inferenz abgebrochen (Schleppfehler bis 0,79 rad); Override 0,5 bleibt auf
≤ 2 mm (`Zittern-an-der-Anlage.md` § 11.3–11.5).

---

## E. Entscheidung

| Wenn … | dann … |
|---|---|
| `pos` so ruhig wie `online` (H1, H2, Gehör) und Abstand quer < 5 mm | **`servo_j_pos`** für Aufnahme und Inferenz: `config.EXECUTION_MODE = "servo_j_pos"` (oder `--ausfuehrung servo_j_pos`), Freigabe hier vermerken |
| `pos` ruhiger als `linear`, aber lauter als `online` | `online` bleibt (`auto`); `pos` für VM-Arbeit, Begründung hier |
| `pos` nicht ruhiger / lauter (C oder D) | `online` bleibt; VM ist für diese Frage kein Stellvertreter — in `Zittern-an-der-Anlage.md` vermerken |
| 120 Hz hält den Takt und ist ruhig wie `online` | Ausweichweg `servo_j` 120 Hz (Spline wäre neuer Code) — Aufwand gegen `pos` abwägen |

**Entscheidung:** am 16.10. vorläufig `online` + Halteregel (§ 10);
nach der Nachrechnung **offen** zwischen `servo_j` linear Override 0,5 und
`online` (`Zittern-an-der-Anlage.md` § 11.8) **Freigabe durch:** `…` **Datum:** `…`

Nachgefahren am 16.10. nachmittags (PICK-Höhe, `online`):

| Bericht | Test | Tempo / Beschl. / Dwell | tiefster Punkt ggü. PICK | H1 RMS |
|---|---|---|---|---|
| `14-01-59`, `14-03-10`, `14-05-06` | Test 1 + Wiederholungen | 1,3 / 1,5 / 0,5 s | +27,3 mm | 0,20–0,21 (pos 0,83–0,84) |
| `14-15-21` | Test 2 | 2,0 / 3,0 / 0,5 s | +0,1 mm | 0,66 (Spitze 14,6 Hz) |
| `14-18-59` | Test 3 | 1,3 / 1,5 / 1,5 s | +0,2 mm | 0,26 |

Danach (am PC): Entscheidung und Zahlen als § 10 in
`Zittern-an-der-Anlage.md` übernehmen; `config.EXECUTION_MODE` und ggf.
`ONLINE_VERIFIED` setzen; Tests laufen lassen.

---

## F. Wenn Zeit bleibt: Referenzaufnahme mit der gewählten Art

```bat
python apps/record.py --robot neura --real-robot --cameras wrist-real --override 1.0 --sequence sequences/arbeitskopien/pick_to_station.json --episodes 1 --noise-scale 0 --noise-fixed --ausfuehrung <gewählt> --block REF --ask-label --out data_anlage/<Datum>/REF
```

(oder Aufnahme-Reiter: Rauschen 0, Ausführung wählen, Block `REF`).
Mit Objekt und Greifer wie bei der echten Aufnahme; ohne konfigurierten Greifer
nur mit `--no-gripper` (Trockenlauf, Episode wird verworfen).
H1 und H2 wie in D. Danach GUI *Aufnahmen bewerten* und *Episodenbericht (PDF)*.

| ✓ | Episode | Takt gehalten | Schleppfehler max (rad) | H1 RMS | H2 RMS | Bild ruhig? |
|---|---|---|---|---|---|---|
| [ ] | `…` | `…` | `…` | `…` | `…` | `…` |

---

## G. Nachbereitung (am PC)

| ✓ | Schritt |
|---|---|
| [ ] | Alle `Berichte/<Datum>_*` (JSON, npz, xlsx), Video-Verweis und Fotos der Handy-Befestigung sichern |
| [ ] | Tabellen oben vollständig, Entscheidung in E eingetragen |
| [ ] | `Zittern-an-der-Anlage.md` § 10 (Anlage `<Datum>`): Zahlen, Bilder, Entscheidung |
| [ ] | Kennzahlen neu rechnen, falls nötig: `python tools/check_servo_stream.py --auswerten Berichte/<stamm>_Servo-Strom` |
| [ ] | Offene Punkte aus `Zittern-an-der-Anlage.md` § 7 / § 9.8 abhaken oder übertragen |

---

## Neue Infos

*Hier anhängen, was am Tag auffällt — mit Uhrzeit, nicht überschreiben.*

| Zeit | Beobachtung | Folge |
|---|---|---|
| vormittags | Punkte mit falschem Tool gespeichert, D brach viermal ab (`12-19-11` … `12-32-51`) | am Pendant mit `ZimmerLWR50` neu gespeichert |
| ganztägig | Tisch **ausgeklappt** (13.10. eingeklappt) | Eigenfrequenz und Zittern nur eingeschränkt mit 13.10. vergleichbar |
| ganztägig | kein H2 (Kamerastange) | bewusst: Aufbau ist Hardware-Thema |
| 12:45 | `online` dreht ~27 mm über PICK um, Leuchte bleibt grün (bei `servo_j` kurz blau) | Halteregel § 10; Nachrechnung § 11 |
| 14:00–14:19 | Test 2/3 (Config versehentlich zweimal nicht umgestellt: `14-03-10`, `14-05-06`) | `config.py` danach wieder 1,3 / 1,5 / 0,5 s |
| PC 16.10. abends | `report_handy.py`: Uhrzeiten nach phyphox-Pause falsch, ruhige Fahrten nicht erkannt | behoben, `--fenster` (§ 11.1) |
| PC 16.10. abends | `online` erreicht nur ~30 % der Drehgrenze → Schleppfehler bis 0,79 rad, Bahn bis 49 mm daneben | Entscheidung § 11.8 |
| PC 16.10. abends | Nach dem PICK-Dwell steht der Arm bei `servo_j` 0,3–1,8 s (ein Aufruf 1014 ms) | Recorder-Kriterium b vor einer Aufnahme anpassen (§ 11.6) |

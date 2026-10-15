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

**Datum:** `…` **Beteiligte:** `…` **Steuerung (Version):** `…`
**Laptop am Netzteil, „Beste Leistung“:** `…` **Override:** 1,0

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
| H1 | Unterarm an Gelenk 5 (wie 13.10.) | `…` |
| H2 | Kamerastange, Höhe der Kamera | `…` |

---

## 1. Sicherheit und Ausgangslage (ohne Bewegung)

| ✓ | Schritt | Wie | Ergebnis |
|---|---|---|---|
| [ ] | Steuerung lesen | GUI *Steuerung prüfen (nur lesen)* | `is_robot_in_simulation()` = `…` (muss `False`), Version `…`, Tool `…` |
| [ ] | Arbeitsraum frei, Not-Aus in Reichweite | — | — |
| [ ] | Punkte der Aufnahmebahn vorhanden | GUI *Punkte sichern* | `CLEAR_FOV, APPROACH_01, PRE_GRASP, PICK, PRE_PLACE` vorhanden: `…` |
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
| 60 Hz | `…` | `…` | `…` | `…` | `…` |
| 90 Hz | `…` | `…` | `…` | `…` | `…` |
| 120 Hz | `…` | `…` | `…` | `…` | `…` |

Bericht: `Berichte/…_Trajektorien-Puffer.json`

**Lesen:** Am PC (VM) lag die Streuung bei 0,2–0,4 ms. Liegt sie an der
Anlage bei 60 Hz **unter ~1 ms** und gibt es fast keine Überläufe → der
Laptop ist **nicht** die Ursache (F1 erledigt). Deutlich mehr → mit
geschlossener GUI und ohne andere Programme wiederholen; bleibt es, gehört
der Laptop-Takt in die Bewertung jeder servo_j-Variante.

**Ergebnis F1:** `…`

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

| Anstoß | H1 Spitze (Hz) | H2 Spitze (Hz) | Ausschwingen sichtbar? |
|---|---|---|---|
| Unterarm seitlich | `…` | `…` | `…` |
| Unterarm oben | `…` | `…` | `…` |
| Kamerastange | `…` | `…` | `…` |
| Plattform | `…` | `…` | `…` |

**Lesen:** Schwingt es bei **6–8 Hz** aus, ist das die Eigenfrequenz aus
dem 13.10. (Bild 3/12). Die VM zeigte, dass die Steuerung den
servo_j-Strom mit einer Schwankung bei **6–12 Hz** umsetzt (§ 9.4) — liegt
die Eigenfrequenz dort, verstärkt der Aufbau genau das.
Hinweis: die Anstoß-Spitzen erscheinen in `report_handy.py` als „Fahrten“,
falls die Lage sich ändert; sonst Spektrum im Zeitfenster ansehen.

**Ergebnis F2:** `…`

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
| linear | `…` | `…` | `…` | `…` | `…` |
| pos | `…` | `…` | `…` | `…` | `…` |

**Abbruchregel:** `pos` hörbar lauter oder ruckartiger als `linear` → D ohne
`pos` fahren, Ergebnis unten eintragen (dann ist die VM hier kein guter
Stellvertreter).

**Ergebnis C:** `…`

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
| linear-60-r0-o1 | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` |
| pos-60-r0-o1 | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` |
| online-15-r0-o1 | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` |
| linear-60-r0-o0.5 | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` |
| pos-60-r1-o1 | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` |
| online-15-r1-o1 | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` | `…` |

Bericht: `Berichte/…_Servo-Strom.json` · Handy: `…` · Video: `…`

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

**Ergebnis F3:** `…`

---

## E. Entscheidung

| Wenn … | dann … |
|---|---|
| `pos` so ruhig wie `online` (H1, H2, Gehör) und Abstand quer < 5 mm | **`servo_j_pos`** für Aufnahme und Inferenz: `config.EXECUTION_MODE = "servo_j_pos"` (oder `--ausfuehrung servo_j_pos`), Freigabe hier vermerken |
| `pos` ruhiger als `linear`, aber lauter als `online` | `online` bleibt (`auto`); `pos` für VM-Arbeit, Begründung hier |
| `pos` nicht ruhiger / lauter (C oder D) | `online` bleibt; VM ist für diese Frage kein Stellvertreter — in `Zittern-an-der-Anlage.md` vermerken |
| 120 Hz hält den Takt und ist ruhig wie `online` | Ausweichweg `servo_j` 120 Hz (Spline wäre neuer Code) — Aufwand gegen `pos` abwägen |

**Entscheidung:** `…` **Freigabe durch:** `…` **Datum:** `…`

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
| `…` | `…` | `…` |

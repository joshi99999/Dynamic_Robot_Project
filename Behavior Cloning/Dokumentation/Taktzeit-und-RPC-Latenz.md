# Taktzeit, RPC-Latenz und abbrechende Bahnen

Stand: 2026-10-02 · Entstanden am ersten Labortag
· **Alle Zahlen hier stammen aus der virtuellen Steuerung (VM `Lara5_V5`
auf dem Aufnahme-Laptop), nicht von der Anlage.**

> **Ergebnis vorweg (2026-10-02).** Die Ursache war der **Rechner**, nicht
> der Code: derselbe Stand lief am PC fehlerfrei, und auf dem Laptop mit
> Energiemodus „Beste Leistung" und frischer VM ebenfalls (30 und 60 Hz:
> Zeitfaktor 1,01–1,04, 0 % schlechte Frames). Die Messwerte in den
> Abschnitten 4.1, 4.2 und 8 stammen von einem **gedrosselten** Laptop —
> sie zeigen, wie sich ein zu schwacher Host äußert, nicht, was die
> Steuerung kostet. Insbesondere ist die Empfehlung „`SERVO_RATE_HZ = 30`
> für die VM" (8.5) überholt: 60 Hz hält, wenn der Rechner nicht drosselt.
>
> Was bleibt: die Methode (Abschnitt 3, die Werkzeuge in 4), die lokale FK
> (spart einen Aufruf je Takt und beseitigt den Zeitstempelversatz), die
> Schleppfehler-Prüfung und die Laufzeitwarnung nach jeder Episode.

> **Wofür dieses Dokument da ist.** Es beschreibt eine **Fehlerklasse**,
> nicht einen Einzelfall: „Der Roboter fährt die Bahn nicht zu Ende, die
> Steuerung meldet einen Verbindungsabbruch, und die Episoden werden wegen
> des Latenzbudgets verworfen." Tritt an der Anlage etwas Ähnliches auf,
> ist hier der Ablauf dokumentiert, mit dem es in der VM eingegrenzt wurde
> — Werkzeuge, Messgrößen und Reihenfolge.
>
> **Die Zahlen selbst sind nicht übertragbar.** Weder die Latenzen noch die
> daraus abgeleiteten Raten. Sie gelten für eine Alpha-Steuerung
> (`v5.0.0-alpha.102`) in einer VirtualBox-VM auf einem Laptop. Die Anlage
> hat andere Hardware, andere Software und ein anderes Netz. Was überträgt,
> ist die **Methode** und die **Form der Rechnung** in Abschnitt 3.

---

## 1. Das Krankheitsbild

Am 2026-10-01 im Labor, Aufnahme gegen die VM:

* Der Arm fuhr die Bahn nicht zu Ende und blieb unterwegs stehen.
* Das Teach-Pendant meldete wiederholt
  `RCSC_102 connection interrupted. Please do reset control from the PC
  option in the GUI.`
* Alle Episoden wurden verworfen, Grund: „Latenzbudget: x % der Frames über
  30 ms" — **auch die, in denen gar keine echte Kamera lief.**

Die Meldung zeigt auf die Kameras. Die Ursache lag woanders. Das ist der
teure Teil dieser Fehlerklasse: **die Diagnose, die die Software anbietet,
ist die falsche.**

## 2. Die Befunde aus den Aufzeichnungen

Fünf Episoden, `data_vm/2026-10-01/`. Entscheidend ist nicht das
Latenzbudget, sondern der Vergleich von Soll- und Ist-Gelenkwinkeln
(`aux.joints_command` gegen `observation.state`):

| Episode | Override | Kameras | Soll-Dauer | Ist-Dauer | Schleppfehler max | Ist-Weg / Soll-Weg | verworfen wegen |
|---|---|---|---|---|---|---|---|
| ep_00000 | 0.2 | sim/sim | 8,3 s | 19,7 s (2,4×) | 0,12 rad | 1,24 / 1,24 rad | Latenz 29 % |
| ep_00001 | 1.0 | daheng/sim | 8,3 s | 31,6 s (3,8×) | 1,09 rad | **0,02** / 1,15 rad | Latenz 90 % |
| ep_00002 | 1.0 | sim/sim | 8,3 s | 28,4 s (3,4×) | 1,22 rad | **0,15** / 1,23 rad | Latenz 44 % |
| ep_00003 | 0.2 | sim/sim | 8,3 s | 13,8 s (1,7×) | 0,12 rad | 1,13 / 1,14 rad | Latenz 7 % |
| ep_00004 | 0.2 | sim/sim | 8,3 s | 21,8 s (2,6×) | 1,10 rad | **0,00** / 1,19 rad | Latenz 40 % |

Daraus die drei Kernbefunde:

### 2.1 Der Recorder merkte nicht, dass der Arm stand

In ep_00004 änderte sich **kein einziger Gelenkwinkel** über alle 218
Takte. Trotzdem stand in den Metadaten `recorded_steps: 218 /
planned_steps: 218` — also „vollständig" — und als Verwerfgrund das
Latenzbudget. 218 Takte Bilder und Zustände eines stehenden Arms, abgelegt
mit einer Begründung, die auf die Kameras zeigt.

Möglich war das, weil `servo_j` weiter einen Rückgabecode < 3 lieferte und
niemand Soll gegen Ist verglich. **Behoben** durch die Schleppfehler-Prüfung
in `bc/recorder.py` (siehe Abschnitt 5).

### 2.2 Nicht die Steuerung war tot, sondern der PC-Steuerkanal

ep_00004 lief **nach** dem Not-Halt am Ende von ep_00003. Vor der Aufnahme
fährt `apps/record.py` mit `move_to_joints()` — einem gewöhnlichen
PTP-Befehl — an die Startstellung, und `recorder.record()` bricht ab, wenn
der Arm dort nicht steht. Beides lief durch.

> **`move_joint` funktionierte, `servo_j` tat nichts.** Genau das besagt
> RCSC_102 und der Hinweis „reset control from the PC option": gekappt wird
> der externe Steuerkanal, nicht die Steuerung.

Das Gegenmittel ist `reset_control()`, nicht der weiße Taster — siehe
`Laptop-Inbetriebnahme-Befunde.md` 3.3.

### 2.3 Das „Latenzbudget" maß die Controller-Latenz, nicht die Kameras

In `bc/recorder.py` wurde der Roboterzustand vor den Kamerabildern
abgegriffen:

```
state = self.robot.read_state()   # t_joints = Mitte des Winkel-Aufrufs ...
                                  # ... danach kostete die FK noch einen RPC
frame = cap.latest()              # Zeitstempel = jetzt
```

`bc/sync.evaluate()` bewertet `max - min` der Zeitstempel gegen
`SYNC_MAX_SKEW_S` (30 ms). Der Abstand war damit **½ × Winkel-RPC + 1 ×
FK-RPC** — reine Controller-Latenz. Eine perfekt frische Platzhalter-Kamera
riss das Budget genauso wie eine echte. **Behoben** durch die lokale FK
(Abschnitt 5).

## 3. Die Rechnung, die überträgt

Das ist der Teil, der an der Anlage genauso gilt — nur mit anderen Zahlen.

`bc/recorder.py` setzt je Takt von `CONTROL_RATE_HZ` ab:

```
SERVO_RATE_HZ / CONTROL_RATE_HZ   x  servo_j
1                                 x  read_state()
```

`read_state()` kostet einen RPC-Aufruf, wenn die FK lokal gerechnet wird,
sonst zwei. Die Bedingung lautet:

```
    (SERVO_RATE_HZ / CONTROL_RATE_HZ) * t_servo_j
  + t_winkel  [+ t_fk, falls die FK über die Steuerung läuft]
  <=  1 / CONTROL_RATE_HZ
```

Ist sie verletzt, folgt **zwangsläufig** die ganze Kette: die Sollwerte
kommen zu spät, der Arm bekommt Lücken im Strom, der Watchdog kappt die
PC-Steuerung, die Bahn bricht ab — und die mitgemessenen Zeitstempel reißen
nebenbei das Synchronisationsbudget.

**An der Anlage ist diese Ungleichung vor der ersten Aufnahme zu prüfen**
(Abnahmeliste 0.6, Punkte 1 und 3). Die Messwerkzeuge dafür stehen in
Abschnitt 4.

## 4. Werkzeuge

| Werkzeug | Was es misst | Bewegt? |
|---|---|---|
| `tools/check_rpc_latency.py` | Kosten je RPC-Aufruf, aufgeschlüsselt in Socket / Serverzeit / NeuraPy-Client; den Versatz, den der Recorder als „Frame über Budget" meldet; ob die Steuerung mehrere Aufrufe je Verbindung annimmt; Hänger bei getaktetem Verkehr | **nein**, nur lesend |
| `tools/check_servo_timing.py` | `t_servo_j`, `read_state()`, und je Kandidatenrate einen trockenen Takt → welche `SERVO_RATE_HZ` den Takt hält | nein, aber **bestromt** und aktiviert das Servo-Interface (Haltebefehle, v = a = 0) |
| `tools/check_rate_budget.py` | rechnet aus diesen Messwerten den gesamten Entwurfsraum durch: welche Kombination aus `CONTROL_RATE_HZ` und `SERVO_RATE_HZ` passt, mit welcher Reserve und zu welchem Preis | **nein**, reine Rechnung ohne Verbindung |

Reihenfolge an der Anlage: erst `check_rpc_latency.py` (gefahrlos), dann
`check_servo_timing.py` mit Hand an der Freigabe.

### 4.1 Messwerte VM, 2026-10-01 — als Beispiel, nicht als Vorgabe

Zwei Läufe am selben Tag, derselbe Rechner, dieselbe VM:

| Aufruf (Median) | 12:44, VM lief seit Stunden | 13:22, direkt nach VM-Neustart |
|---|---|---|
| TCP-Verbindung allein | 1,3 ms | 0,9 ms |
| `get_current_joint_angles` | 14,7 ms | 8,7 ms |
| `..._with_timestamp` | 18,5 ms | 8,2 ms |
| `compute_forward_kinematics` | 6,8 ms | 4,7 ms |
| Versatz Roboter ↔ Sim-Kamera | 17,7 ms (10 % über 30 ms) | 11,3 ms (0 % über 30 ms) |

**Die Latenz halbierte sich durch einen Neustart der VM.** Das ist der
wichtigste Vorbehalt gegen jede Zahl in diesem Dokument: sie hängt am
Laufzeitzustand der VM, nicht an der Software des Projekts. Während der
Aufnahme lag sie noch einmal deutlich höher — aus ep_00003 zurückgerechnet
rund 21 ms je `servo_j`.

Aufschlüsselung eines Aufrufs (12:44): Socket-Aufbau 1,3 ms, **Serverzeit
11,2 ms**, NeuraPy-Client 2,2 ms. Der Aufruf ist also langsam, weil die
*Steuerung* ihn langsam beantwortet — nicht wegen des Verbindungsaufbaus.
Eine frühere Vermutung in die andere Richtung hat sich damit erledigt.

### 4.2 `servo_j` gemessen, 2026-10-01 13:58 — VM

`tools/check_servo_timing.py`, Servo-Interface aktiv, nur Haltebefehle.
Der Arm bewegte sich um **0,000 rad**, die Diagnose blieb vorher wie
nachher `critical: false`.

| | min | median | p95 | max |
|---|---|---|---|---|
| `servo_j` (Haltebefehl) | 12,2 | **22,4** | 39,0 | 80,5 ms |
| `read_state()` (Winkel + FK, 2 Aufrufe) | 18,1 | **30,7** | 51,0 | 51,6 ms |

Trockener Takt je Kandidatenrate, Budget 66,7 ms:

| `SERVO_RATE_HZ` | Teilschritte | Takt median | Takt p95 | erreichbare Regelrate | Urteil |
|---|---|---|---|---|---|
| 60 (konfiguriert) | 4 | 123,7 ms | 161,4 ms | **8,1 Hz** | weit darüber |
| 30 | 2 | 67,2 ms | 98,3 ms | 14,9 Hz | Median knapp daneben |
| 15 | 1 | 58,1 ms | 73,0 ms | 17,2 Hz | Median passt, p95 nicht |

**Drei Dinge, die daraus folgen:**

1. Die Rückrechnung aus ep_00003 (≈ 21 ms je `servo_j`) war richtig —
   gemessen 22,4 ms. Der Zusammenhang aus Abschnitt 3 ist damit belegt und
   nicht nur plausibel.
2. **Mit aktivem Servo-Interface wird jeder Aufruf teurer.** Im Leerlauf
   (13:22) kostete ein Lesen 8,7 ms, `read_state()` rechnerisch 12,9 ms —
   unter Servo-Last 30,7 ms, also rund das 2,4-fache. Eine Messung ohne
   aktives Servo-Interface **unterschätzt die Taktkosten systematisch**.
   An der Anlage deshalb immer `check_servo_timing.py` heranziehen, nicht
   nur `check_rpc_latency.py`.
3. **In dieser VM hält selbst der Minimalfall den Takt nicht sicher.** Bei
   `SERVO_RATE_HZ = 15` — ein `servo_j` je Takt, also gar keine
   Interpolation mehr — passt der Median, der p95 mit 73,0 ms aber nicht.

Mit lokaler FK entfiele ein Aufruf aus `read_state()`. Aus dem
Leerlauf-Verhältnis (Winkel 8,2 : FK 4,7 ms) geschätzt rund **11 ms je
Takt** — damit läge 15 Hz auch im p95 im Budget und 30 Hz im Median
komfortabel. **Geschätzt, nicht gemessen**, und erst nach der
URDF-Korrektur aus Abschnitt 7 überhaupt wirksam.

### 4.3 `servo_j` liefert in dieser Version gar keinen Code

`letzter_servo_code: null`. Die Prüfung in `bc/adapters/neura.py`
(`code >= SERVO_ERROR_CODE_MIN`) kann in dieser Controller-Version also
**nie** auslösen — der Adapter hat über den Rückgabewert keinerlei
Information darüber, ob der Sollwert angekommen ist. Das ist der zweite
Grund, warum die Schleppfehler-Prüfung aus Abschnitt 5.2 nötig war und
nicht durch eine Auswertung des Codes zu ersetzen ist.

### 4.4 Zwei beantwortete Nebenfragen

**Eine stehende Verbindung ist nicht möglich.** Die Steuerung schließt den
Socket nach genau einem Aufruf (`check_rpc_latency.py`, Abschnitt 4). Der
Socket-Aufbau je Aufruf ist damit nicht vermeidbar — er kostet aber nur
rund 1 ms und ist nicht das Problem.

**Die 1000-ms-Hänger sind ein Artefakt der Messung.** Im Burst brauchten
2 von 30 reinen TCP-Verbindungen 1003 ms — die Signatur eines verworfenen
SYN bei voller Accept-Warteschlange. Bei getaktetem Verkehr (560 Aufrufe
mit 60 Hz über 10 s) trat **kein einziger** Hänger über 200 ms auf. Für
den Normalbetrieb also unkritisch; die Messung sollte an der Anlage
trotzdem wiederholt werden, weil ein einzelner Sekundenhänger jeden
Servo-Strom reißen würde.

## 5. Was im Code geändert wurde

| Was | Wo | Wirkung |
|---|---|---|
| FK lokal aus der URDF-Kette statt per RPC, mit Vortest gegen die Steuerung | `bc/adapters/neura.py` (`read_state`, `_check_local_fk`), `config.NEURA_LOCAL_FK` | ein RPC-Aufruf je Takt weniger; Roboter- und Bildzeitstempel liegen unmittelbar nebeneinander, der Scheinbefund aus 2.3 entfällt |
| Schleppfehler-Abbruch je Takt, zwei Kriterien | `bc/recorder.py`, `config.RECORDER_*` | eine Episode mit stehendem Arm bricht ab und nennt den Grund, statt als „vollständig" abgelegt zu werden |
| FK-Weg in den Metadaten | `apps/record.py` | beim Vergleich zweier Aufnahmen ist erkennbar, welcher Weg lief |

### 5.1 Der Vortest der lokalen FK — und warum er mehrere Stellungen prüft

Lokal gerechnet wird **nur**, wenn die URDF-Kette die Steuerung
reproduziert. Der Vortest läuft beim Verbinden, bewegt nichts
(`compute_forward_kinematics` rechnet für beliebige übergebene Winkel) und
schaltet bei Abweichung ab — dann bleibt die Steuerung maßgeblich.

Die erste Fassung prüfte nur die **aktuelle** Stellung. Das war zu wenig:
Die VM stand auf `[0, 0, 0, 0, 0, 0]`, und dort ist ein invertiertes
Achsvorzeichen unsichtbar (sin 0 = 0, cos 0 = 1). Der Vortest meldete
14,5 mm, während dieselbe Kette über die 50 Golden-Stützstellen bis zu
**1850 mm und 161°** danebenlag. Geprüft wird deshalb gegen
`neura.PROBE_JOINTS` — vier Stellungen, in denen jede Achse ungleich null
ist und die Vorzeichen wechseln.

### 5.2 Die beiden Schleppfehler-Kriterien

```
(a) absolut         |q_mess - q_soll| > RECORDER_MAX_FOLLOW_ERROR_RAD
                    über RECORDER_FOLLOW_ERROR_STEPS Takte in Folge
(b) maßstabsfrei    der Sollwert ist über das Fenster gewandert,
                    der Arm aber um weniger als RECORDER_MIN_FOLLOW_RATIO davon
```

(b) ist nicht redundant. Eine Bahn mit nur 0,12 rad Gelenkhub erreicht auch
bei völlig stehendem Arm nie die absolute Schwelle von 0,30 rad — genau
dieser Fall trat im Test auf. (b) fragt stattdessen „der Sollwert ist
gewandert, ist der Arm mit?" und ist damit unabhängig von Bahnlänge und
Geschwindigkeit. Während des Greifer-Dwells wandert der Sollwert nicht, es
löst dort also nicht aus.

**Kein Not-Halt an dieser Stelle.** Im beobachteten Fall steht der Arm
bereits; ein Stopp ließe nur die nächste Episode mit einer irreführenden
Meldung scheitern. Der Abbruch beendet den Sollwertstrom, das `finally`
deaktiviert das Servo-Interface.

**`RECORDER_MAX_FOLLOW_ERROR_RAD = 0.30` ist vorläufig.** Der Wert liegt
zwischen dem gemessenen Nachlauf guter VM-Läufe (0,12 rad) und dem Ausfall
(1,1 rad). An der Anlage wird schneller gefahren, der Nachlauf ist also
größer — **der Wert ist dort neu zu bestimmen**, sonst bricht die Prüfung
gesunde Episoden ab.

## 6. Offen

1. ~~`t_servo_j` ist nicht gemessen~~ — **erledigt 2026-10-01**, siehe 4.2:
   22,4 ms Median unter Servo-Last.
2. **`SERVO_RATE_HZ` ist unverändert 60.** Die Entwurfsrechnung steht in
   Abschnitt 8; empfohlen ist dort 30 für Läufe in der VM. Nicht gesetzt,
   weil es eine Festlegung wäre, die an der Anlage vermutlich unnötig ist.
3. **Die lokale FK ist auf diesem Stand inaktiv.** Die URDF im Repo ist
   die unkorrigierte (siehe Abschnitt 7) — der Vortest schaltet korrekt ab.
   Fix 2 wirkt erst nach der URDF-Korrektur.
4. **Zeitlimit um `_call()`** fehlt weiterhin (Punkt 4 der Liste in
   `Laptop-Inbetriebnahme-Befunde.md` Abschnitt 5). NeuraPy kennt kein
   Socket-Timeout; ein hängender Aufruf ist von „dauert lange" nicht
   unterscheidbar.
5. **Not-Halt am Bahnende** (ep_00003): Der Arm hatte beim letzten Takt
   noch 0,0155 rad Restfehler, war also in Bewegung, als `finally:
   deactivate_servo()` lief. Abbau des Servo-Interface bei laufender
   Restbewegung ist der naheliegendste Auslöser — **Hypothese, nicht
   belegt.** Prüfbar, indem am Bahnende einige Takte auf dem letzten
   Sollwert gehalten werden, bevor deaktiviert wird.

## 7. Nebenbefund: die URDF im Repo ist die unkorrigierte

Der Vortest der lokalen FK hat es aufgedeckt. `bc/data/lara5_candidate.urdf`
steht auf dem Stand des Commits „BehaviorCloning Grundgerüst Phase 0":

| Stelle | im Repo | laut `70_BehaviorCloning/Sim-Inbetriebnahme-Befunde.md` 2.2 |
|---|---|---|
| `elfin_joint2` | `<axis xyz="-1 0 0"/>` | `<axis xyz="1 0 0"/>` |
| `elfin_end_joint` | `xyz="0 -0.0735 0"` | `xyz="0 -0.059 0"` |

Gemessen gegen die 50 Golden-Stützstellen (`tests/data/golden_fk_lara5_sim.json`):
Position **min 14,5 mm, Median 1296 mm, max 1850 mm**; Rotation bis **161°**.
Die 14,5 mm des Minimums sind exakt der dokumentierte Flanschfehler — das
Minimum tritt bei der einen Stützstelle mit `q2 = 0` auf, wo der
Achsfehler verschwindet.

Die Korrektur gilt in `Sim-Inbetriebnahme-Befunde.md` Abschnitt 5 als
„erledigt (2026-08-27)". In diesem Branch (`BehaviorCloning`) ist sie
nicht vorhanden; vermutlich liegt sie nur im Arbeitsbaum am PC.

**Betroffen ist der Sim-Pfad**, nicht die Aufnahme gegen die VM: gegen
`NeuraRobot` kommen FK und IK aus der Steuerung. Der `SimRobot` und damit
alles, was hardwarefrei geplant, kollisionsgeprüft und getestet wird,
rechnet dagegen mit dieser Kette.

Die Korrektur ist **nicht** eingepflegt — sie ändert das geometrische
Modell des gesamten Sim-Pfads und gehört bewusst entschieden, nicht
nebenbei.

---

## 8. Entwurfsraum der Raten (Stand nach der URDF-Korrektur)

Gerechnet mit `tools/check_rate_budget.py` aus den Messwerten vom
2026-10-01 13:58 (`t_servo_j` 22,4 ms median / 39,1 p95).
`t_read_state` ist seit der URDF-Korrektur **ein** Aufruf statt zwei;
angesetzt mit 19,5 ms median / 32,4 p95 — aus dem Leerlaufverhältnis
(Winkel 8,2 : FK 4,7 ms) geschätzt, **nicht gemessen**. Ein erneuter Lauf
von `check_servo_timing.py` liefert den echten Wert.

### 8.1 Was die URDF-Korrektur allein schon bringt

| | Takt bei `SERVO_RATE_HZ` = 60 | erreichte Regelrate |
|---|---|---|
| vorher (FK über die Steuerung) | 123,7 ms | 8,1 Hz |
| nachher (FK lokal), `SERVO_RATE_HZ` = 30 | **64,4 ms** | **15,0 Hz** |

**Die 15 Hz aus den Requierments sind damit im Median wieder haltbar, ohne
dass eine Festlegung fällt.** Der Preis ist die halbierte Senderate: zwei
Teilschritte statt vier.

### 8.2 Die Kandidaten im Median

| `CONTROL_RATE_HZ` | k | `SERVO_RATE_HZ` | Takt ist / soll | Reserve | Laufruhe¹ | Takte je Episode² |
|---|---|---|---|---|---|---|
| **15** | **2** | **30** | 64,4 / 66,7 ms | 3 % | ~64 % | 218 |
| 15 | 1 | 15 | 41,9 / 66,7 ms | 37 % | ~80 % | 218 |
| 12,5 | 2 | 25 | 64,4 / 80,0 ms | 20 % | ~70 % | 182 |
| 10 | 3 | 30 | 86,8 / 100,0 ms | 13 % | ~64 % | 145 |
| 10 | 2 | 20 | 64,4 / 100,0 ms | 36 % | ~75 % | 145 |

¹ Geschätzte Streuung der Ist-Geschwindigkeit innerhalb eines Takts,
interpoliert aus der Messung VM 2026-09-15 (15 Hz → 60–100 %, 60 Hz → 33 %,
120 Hz → 27 %, 250 Hz → 19 %). **Kleiner ist besser**, heute 33 %.
² Referenzbahn `pick_to_station`, 14,5 s Fahrzeit. Weniger Takte heißt
weniger Trainingsbeispiele je Episode.

### 8.3 Im p95 hält bei 15 Hz nichts

| `CONTROL_RATE_HZ` | k | `SERVO_RATE_HZ` | Takt ist / soll | |
|---|---|---|---|---|
| 15 | 1 | 15 | 71,4 / 66,7 ms | reißt |
| 12,5 | 1 | 12,5 | 71,4 / 80,0 ms | 11 % Reserve |
| 10 | 1 | 10 | 71,4 / 100,0 ms | 29 % Reserve |
| 7,5 | 2 | 15 | 110,5 / 133,3 ms | 17 % Reserve |

Auch der Minimalfall bei 15 Hz — ein `servo_j` je Takt, gar keine
Interpolation — reißt im p95. **Echte Reserve im schlechten Fall gibt es
nur unterhalb von 15 Hz.**

Ein Überlauf ist allerdings kein Datenfehler: der Pacer holt nicht auf,
sondern nimmt den nächsten Rasterpunkt, und jeder Schritt trägt seinen
echten Zeitstempel. Die Bahn wird langsamer abgefahren — und *diese*
Geschwindigkeit wird mitgelernt (AP 2.6). Für Daten, aus denen eine Policy
entstehen soll, ist das der eigentliche Schaden.

### 8.4 Der Preis einer kleineren `CONTROL_RATE_HZ`

Nicht empfohlen, aber vollständigkeitshalber beziffert. Bei 10 statt 15 Hz:

* **Schema-Bruch.** Die Rate ist in `Requierments/requierments.md` auf
  15 Hz festgelegt (Zeilen 19, 270, 350) und muss bei Aufzeichnung und
  Inferenz gleich sein. Alle vorhandenen Aufnahmen werden unvergleichbar
  (ebd. Zeile 553).
* **−33 % Trainingsbeispiele je Episode** (145 statt 218).
* **Reaktionszeit 100 statt 67 ms.** Relevant für das Störungsszenario
  AP 5.1 und für die Chunk-Auslegung (ebd. Zeile 519).
* Dafür: 13 % Reserve im Median bei `SERVO_RATE_HZ` = 30 und 29 % im p95
  bei k = 1.

### 8.5 Die Gegenprobe — und warum nichts festgelegt wird

Mit der ursprünglichen Auslegungsannahme (`t_servo_j` = 2,5 ms, siehe
`bc/config.py`) gerechnet:

| `CONTROL_RATE_HZ` | k | `SERVO_RATE_HZ` | Takt ist / soll |
|---|---|---|---|
| 15 | 4 | 60 (heutige Konfiguration) | 14,0 / 66,7 ms |
| 15 | 8 | 120 | 24,0 / 66,7 ms |

Bei 2,5 ms je Aufruf passt die heutige Konfiguration mit **79 % Reserve**,
und selbst 120 Hz Senderate wäre möglich. Die Auslegung war richtig; nur
ihre Eingangsgröße stimmt auf diesem Rechner nicht.

**Deshalb wird hier nichts umgestellt.** Für Läufe *in der VM* ist
`SERVO_RATE_HZ = 30` der Behelf der Wahl — er hält die festgelegten 15 Hz
im Median und kostet nur Laufruhe. An der Anlage ist zuerst `t_servo_j`
neu zu messen; liegt es wieder im Bereich der Auslegungsannahme, bleibt
alles, wie es ist.

---

## 9. Zweiter Befund 2026-10-01: der Override deckelt die Bahn

Nach der URDF-Korrektur brachen zwei Läufe mit der neuen
Schleppfehler-Prüfung ab (`ep_00005`, 60 Hz, Takt 145; `ep_00006`, 30 Hz,
Takt 21). Beide mit **Override 0.2**. Die Meldung zeigte auf RCSC_102 —
**das war falsch.** Im Log der Steuerung stand kein RCSC_102, sondern nur
`servo Interface connected/disconnected successfully`, also der eigene
Auf- und Abbau.

### 9.1 Es war ein echter Schleppfehler, kein toter Kanal

| | ep_00005 | ep_00006 |
|---|---|---|
| Ist-Weg / Soll-Weg bis zum Abbruch | 1,15 / 1,145 rad | 0,195 / 0,388 rad |
| Schleppfehler median | 0,011 rad | 0,078 rad |
| Schleppfehler, letzte 10 Takte | 0,002 → **0,138** | 0,091 → **0,193** |

Der Fehler wächst **monoton**, er springt nicht. Bei einem gekappten
Steuerkanal steht der Arm sofort und vollständig (so wie in `ep_00004`,
Abschnitt 2.1). Hier folgte er bis zuletzt, nur zunehmend schlechter.

### 9.2 Die Ursache: die Sollgeschwindigkeit übersteigt den Override

Der Override deckelt die Gelenkgeschwindigkeit. Gegen die
URDF-Grenzwerte (1,57 rad/s je Achse) gerechnet erlaubt Override 0.2
**0,314 rad/s**:

| | kritisches Gelenk | max. Sollgeschwindigkeit | Anteil der Takte darüber |
|---|---|---|---|
| ep_00005 | 3 | 0,457 rad/s = **146 %** | 47 von 144 |
| ep_00006 | 2 | 0,369 rad/s = **117 %** | 14 von 20 |

Und der Abbruch fällt genau auf das Überschreiten. `ep_00005`, Gelenk 3
über die letzten acht Takte:

```
0.106  0.158  0.208  0.259  0.295  0.313  0.328  0.337   rad/s
                                    ^^^^^ Grenze 0.314 ^^^^^^^^
```

Der Controller sättigt, der Sollwert läuft weiter, der Abstand wächst
unbegrenzt. Genau das meldet die Prüfung — korrekt, nur mit der falschen
Vermutung in der Begründung.

**Vorbehalt:** Die 1,57 rad/s stammen aus `bc/data/lara5_candidate.urdf`
und sind gegen den Controller **nicht verifiziert**
(`config.JOINT_LIMITS_VERIFIED = False`). Die Rechnung passt quantitativ
sehr gut, ist aber eine Herleitung, keine Messung.

### 9.3 Konsequenz

* **Mit Override 1.0 aufnehmen.** Das Tempo soll der Planer bestimmen
  (`TRANSIT_SPEED_MS`, `APPROACH_SPEED_MS`), nicht der Override — so steht
  es auch in der Warnung in `apps/record.py`. Bei 0.2 ist der Override der
  Begrenzer, und die Bahn wird nicht mehr so gefahren, wie sie geplant ist.
* Die früheren „erfolgreichen" Läufe bei Override 0.2 (`ep_00000`,
  `ep_00003`) sind damit **keine guten Daten**: sie liefen 1,7–2,4-fach zu
  langsam, und der Override hat überall dort mitgeschnitten, wo die Bahn
  schneller gewollt hätte. Die Ausführungsgeschwindigkeit wird mitgelernt
  (AP 2.6).
* Die Abbruchmeldung nennt jetzt die Sollgeschwindigkeit und das
  kritische Gelenk und führt beide Ursachen auf, statt eine zu raten.

### 9.4 Offen

Ein Vorabtest wäre möglich und würde den Fehllauf ganz vermeiden: die
Sollgeschwindigkeiten des fertigen Plans stehen vor der Fahrt fest und
lassen sich gegen `Override × Achsgrenze` halten. Bewusst noch nicht
gebaut — er stünde und fiele mit den unverifizierten Grenzwerten aus 9.2.

---

## 10. Dritter Befund: der Fehlerzustand bleibt und wirkt in die nächsten Läufe

Nach dem 120-Hz-Versuch (`ep_00010`, 14:53, RCSC_105) scheiterten alle
folgenden Läufe — auch in einer Konfiguration, die zehn Minuten vorher
218 von 218 Takten sauber gefahren war.

### 10.1 Es lag nicht am Takt

| | `ep_00007` 14:50 (gelungen) | `ep_00011` 15:00 (Abbruch Takt 93) |
|---|---|---|
| Einstellung | 30 Hz, Override 1.0 | 30 Hz, Override 1.0 |
| Überläufe je Servo-Tick | 0,28 | **0,20** |
| schlechte Frames | 5,0 % | **4,3 %** |
| Schleppfehler bis zum Abriss | 0,004 rad median | 0,0005–0,005 rad |

Der gescheiterte Lauf lief **sauberer** als der gelungene. Die Taktzeit
scheidet als Ursache aus.

### 10.2 Es war das Sprungmuster, nicht die Sättigung

`ep_00011`, Ist-Bewegung je Takt am Ende:

```
0.0190  0.0099  0.0019  0.0045  0.0000  0.0000  0.0000   rad
Schleppfehler: 0.0036  0.0035  0.0048  0.0085  0.0162  0.0238  0.0314
```

Der Arm folgte bis Takt 89 praktisch fehlerfrei und stand dann **sofort
und vollständig**. Das ist das Muster aus Abschnitt 2.1 (gekappter
Steuerkanal), nicht das aus 9.1 (Override-Sättigung).

### 10.3 Der Fehlerzustand bleibt stehen

Gemessen um 15:06, also sechs Minuten nach dem letzten Lauf, rein lesend:

```
get_diagnostics         {'critical': True, 'issues': {'collision_detected': False,
                         'other_errors': ['undefined error'], 'powered_off': False}}
is_robot_in_teach_mode  False          <- Automatik, der Modus ist es NICHT
program_status          NOT_RUNNING
init_program            FEHLER: Unable to switch to play mode.
                                Check if robot in automatic mode
```

Der Roboter steht in Automatik, und `init_program()` beklagt trotzdem den
Modus. Genau die irreführende Meldung aus
`Laptop-Inbetriebnahme-Befunde.md` 3.1 — gemeint ist `critical: True`.

**Daraus folgt: nach einem RCSC_10x ist die Steuerung bis zum
`reset_control()` unbrauchbar, und zwar still.** Ein Folgelauf kann
scheinbar normal anlaufen und erst nach Dutzenden Takten stehenbleiben.
Wer das nicht weiß, sucht den Fehler im zuletzt geänderten Parameter —
hier wäre das fälschlich die Servorate gewesen.

### 10.4 Erkennung im Adapter

Ein eigenes Reset-Werkzeug gab es vorübergehend; es ist am 2026-10-02
entfallen. Stattdessen fragt `NeuraRobot`, wenn `init_program()`
endgültig abgelehnt wird, `get_diagnostics()` und
`is_robot_in_teach_mode()` ab und nennt die tatsächliche Ursache:
Fehlerzustand (→ Reset Control im Pendant), Teach-Modus (→ Automatik)
oder noch laufendes Programm (→ warten). Lesend verbunden wird
`init_program()` gar nicht mehr gerufen, sodass sich eine Steuerung im
Fehlerzustand weiter untersuchen lässt.

### 10.5 Offen

Ob der 120-Hz-Lauf die Steuerung in diesen Zustand gebracht hat und
`ep_00011` schon darauf lief, ist **plausibel, aber nicht bewiesen**.
Unterscheidbar durch genau einen Versuch: zurücksetzen, dann 30 Hz /
Override 1.0 wiederholen.

* Läuft er durch → der Fehlerzustand war die Ursache, 30 Hz ist der
  Betriebspunkt, und 120 Hz gehört nicht mehr probiert.
* Bricht er wieder bei ~90 Takten ab → der Kanal fällt unabhängig davon
  aus, und die Suche geht weiter.

Zu klären wäre dann außerdem, warum `connect()` diesen Zustand nicht
erkennt: `get_diagnostics()` meldet ihn eindeutig, der Adapter fragt
aber nicht danach (Punkt 1 der Liste in
`Laptop-Inbetriebnahme-Befunde.md` Abschnitt 5 berührt das nur am Rand).

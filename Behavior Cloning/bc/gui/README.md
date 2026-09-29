# Bedienoberfläche (`bc/gui`)

PySide6-Oberfläche für den Behavior-Cloning-Teil. Vier Modi:
**Systemcheck · Aufnahme · Training · Betrieb**.

## Starten

```bash
python -m bc.gui                     # alle Modi
python -m bc.gui --mode systemcheck  # direkt in einem Modus
```

Braucht `PySide6>=6.6` zusätzlich zur bisherigen Umgebung (in
`pyproject.toml` als Extra `gui` deklariert):

```bash
E:\Enviroments\bc_env\python.exe -m pip install "PySide6>=6.6"
```

## Zwei Ebenen

**Eigenständig** — der Fall, der am Labortag zählt. Läuft genauso gegen die
Simulation (`SimRobot`) und gegen die virtuelle Steuerung.

**Als Reiter in der GUI der anderen Gruppe** — ohne eine einzige Änderung an
deren Code:

```python
from gui.app import MainWindow          # deren Fenster
from bc.gui.plugin import attach_to

window = MainWindow()
attach_to(window)                       # hängt "Behavior Cloning" an
window.show()
```

`attach_to` findet deren `QTabWidget` selbst, hängt den Reiter von außen an
und meldet das Herunterfahren bei `QApplication.aboutToQuit` an — ihr
`closeEvent` räumt nur die eigenen Reiter auf, hart aufgezählt.

## Was wirklich dranhängt

Die Anzeige oben zeigt **nicht die Auswahl**, sondern die Antwort des
Controllers auf `is_robot_in_simulation()`:

| Zustand | Bedeutung |
|---|---|
| grau | nicht verbunden |
| blau | `SimRobot` im Prozess, keine Steuerung |
| grün | Controller meldet Simulation |
| orange | Simulation nicht bestätigt → Bewegung gesperrt |
| **rot** | **reale Anlage, Bewegung freigegeben** |

Rot ist ausschließlich der Anlage vorbehalten. Virtuelle Steuerung und echte
Control-Box hören auf dieselbe Adresse — eine Anzeige, die der Auswahl folgt,
würde genau dann lügen, wenn es darauf ankommt. Das ist eine Software-Anzeige
und ersetzt keinen zertifizierten Not-Aus (AP 4.2).

## Kameras zuordnen

Die Oberfläche fragt nicht „welcher Kameramodus?", sondern **was hängt auf
welchem Platz**:

| Platz | was dort sitzt |
|---|---|
| `wrist` | am Greifer montiert, trägt den Hauptteil der Information |
| `scene` | Top-View; Modell noch nicht entschieden (AP 1.1) |

„Geräte suchen" listet Webcams (OpenCV-Indizes) und Daheng-Kameras
(Galaxy SDK). **Der Platzhalter „Simulation" steht immer zur Auswahl** —
auch wenn Hardware da ist. Damit lässt sich die ganze Kette ohne jede
Kamera durchspielen, und das Datensatz-Schema bleibt identisch (AP 0.5).

Daraus folgt: **Voraussetzungen hängen an der Zuordnung, nicht am Modus.**
Das Galaxy SDK wird nur verlangt, wenn tatsächlich eine Daheng zugewiesen
ist; mit Platzhaltern und dem SimRobot braucht eine Aufnahme gar nichts.

Dieselbe Zuordnung gibt es auf der Kommandozeile, und zwar in derselben
Schreibweise, die im Reiter steht:

```bash
python tools/check_cameras.py --list                       # was ist da?
python apps/record.py --sim --camera wrist=sim --camera scene=uvc:1
python apps/infer.py --robot neura --camera wrist=daheng:EBK24100633 --camera scene=uvc:1
```

`--camera NAME=BACKEND[:GERÄT]` ergänzt das bisherige `--cameras <modus>`,
das nur noch den Ausgangspunkt setzt. Ein Kamerawechsel ist damit ein
Zuordnungswechsel und keine Code-Änderung — solange es einen Adapter gibt,
lässt sich jedes Gerät auf jeden Platz legen.

## Aufbau

| Datei | Qt? | Inhalt |
|---|---|---|
| `session.py` | nein | Backendwahl, Verbindung, Statusermittlung |
| `requirements.py` | nein | Voraussetzungen je Modus (CUDA, lerobot, neurapy, …) |
| `checks.py` | nein | Katalog der Systemcheck-Punkte, Auswertung der Ausgabe |
| `cameras.py` | nein | Geräte suchen, Kameraplätze besetzen, Befunde dazu |
| `training.py` | nein | Datensätze/Läufe finden, Kommandozeilen bauen, Trainingsausgabe lesen |
| `recording.py` | nein | Ablaufdateien lesen und schreiben, Arbeitskopien, Aufnahme-Kommandozeile, Aufnahmeausgabe lesen |
| `operation.py` | nein | Modelle finden, Vorabprüfung gegen die Aufzeichnung, Fahrtausgabe lesen |
| `runner.py` | ja | Arbeitsthreads (`TaskRunner`), Unterprozesse (`ProcessRunner`) |
| `widgets.py` | ja | Abzeichen, Voraussetzungszeile, Bildansicht, Log |
| `tab_base.py` | ja | Unterbau der Modi, Platzhalter-Reiter |
| `tab_*.py` | ja | die vier Modi |
| `app.py` | ja | eigenständiges Fenster |
| `plugin.py` | ja | Einbindung in die fremde GUI |

Die Qt-freien Teile sind durch `tests/test_gui.py` abgedeckt und laufen in
`tests/run_all.py` mit. `bc/gui` importiert weder `neurapy` noch `gxipy`
direkt, sondern geht über `bc.adapters` — dieselbe Schichtenregel wie im
Rest der Pipeline.

## Stand

**Fertig:** Grundgerüst, Backendwahl mit echter Simulationsanzeige,
Voraussetzungsmeldungen.

*Systemcheck* — führt `tests/run_all.py` und die `tools/check_*.py` als
Unterprozesse aus, gestaffelt nach hardwarefrei / lesend / mit Bewegung,
Rückfrage vor jeder Bewegung.

*Training* (2026-09-29) — die ganze Kette aus dem Reiter heraus:

1. **Datensatz** — Aufnahmen aus `data_vm/` und `data_sim/` auswählen,
   Konsistenz prüfen (`--check`, schreibt nichts), exportieren. Oder einen
   fertigen `LeRobotDataset`-Ordner nehmen, auch einen von einem anderen
   Rechner — der Ordner wird eingebunden, nicht kopiert.
2. **Hardware** — Geräteliste zur Laufzeit aus torch. Ohne GPU sagt der
   Reiter das und verweist auf den portablen Datensatz.
3. **Einstellungen** — links, was man zwischen Läufen dreht; rechts die
   Festlegungen aus `bc/config.py`. Abweichungen werden benannt, bei den
   Festlegungen zusätzlich mit Rückfrage vor dem Start.
4. **Lauf** — Fortschritt, Validierungsfehler in rad je Validierung,
   jederzeit abbrechbar; Läufe auf einen Stick kopieren.

Die Kommandozeile steht daneben zum Mitlesen und Kopieren — es ist genau
die, die auch im Terminal gilt. Nur Werte, die von der Vorgabe abweichen,
stehen darin.

*Aufnahme* (2026-09-29):

1. **Kameras** — Zuordnung (siehe oben) und **Vorschau**: Die Oberfläche
   öffnet die zugeordneten Kameras selbst und zeigt Bild, tatsächlich
   erreichte Bildrate je Quelle und den Zeitversatz gegen das 30-ms-Budget.
   Das ist der Zustand zum Einrichten.
2. **Ablauf** — Ablaufdatei wählen und **bearbeiten** (siehe unten);
   `apps/teach.py` startet aus dem Reiter.
3. **Aufnahme** — Episoden, Rauschfaktor, Zielordner nach der vereinbarten
   Ablage (`data_vm/<Datum>/` gegen die Steuerung, `data_sim/` gegen den
   SimRobot), und die **Bewertung nach jeder Episode**.
4. **Metadaten** (AP 5.2) — Block, Objekt, Beleuchtung, Kamerapose. Alles
   Freitext und optional; nur der Block ist an der Anlage Pflicht.

### Ablauf bearbeiten: Arbeitskopie statt Original

Ausgewählt wird die Ablaufdatei im Projekt, bearbeitet und **aufgezeichnet
wird eine Arbeitskopie** unter `sequences/arbeitskopien/` (gitignored). Die
Datei im Projekt bleibt damit unverändert und ist zugleich die
Sicherungskopie; „Aus Original zurücksetzen" stellt den Ausgangszustand
wieder her. Die Arbeitskopie bleibt liegen — beim nächsten Start steht der
letzte Stand wieder da.

In der Tabelle lassen sich Punkte hinzufügen, löschen, verschieben und
umbenennen, dazu Bewegungsart (ptp/lin), Überschleifradius in mm, `optional`
und `approach` sowie der Greiferzustand. Ist die Steuerung verbunden, füllt
sich die Punktauswahl aus ihrer Datenbank; ein noch nicht geteachter Name
ist trotzdem erlaubt.

Geprüft wird mit `bc.sequence.parse_sequence` — genau dem, was auch
`record.py` beim Laden anwendet. Gespeichert wird nur, was durchgeht: Ein
ungültiger Ablauf fiele sonst erst beim Start auf, und dann steht jemand an
der Anlage davor.

### Episoden je Aufruf

An der realen Anlage gehört zu einem Aufruf **eine** Episode. Grund: Die
Bahn endet am Übergabepunkt, der Arm hält dort das Objekt — und zu Beginn
jeder Episode sendet der Recorder den Greifer-Startzustand („auf"). Episode 2
ließe das Objekt also an `PRE_PLACE` fallen und zeichnete danach einen Griff
ins Leere auf. Der Reiter sagt das, sobald die Ampel rot ist.

Ohne physisches Objekt (SimRobot, virtuelle Steuerung) sind mehrere Episoden
je Aufruf sinnvoll und der Normalfall — dort variiert nur das Rauschen.

### Bewertung je Episode (AP 5.2)

Mit „Nach jeder Episode bewerten" hält `apps/record.py --ask-label` nach
jeder Episode an; die Oberfläche fragt **erfolgreich / fehlgeschlagen /
verwerfen** und schickt die Antwort auf `stdin`. Verworfene Episoden bleiben
auf der Platte, werden aber markiert und kommen nicht in den Datensatz.

Ohne den Haken bleibt eine Episode **unbewertet** — `apps/export.py
--require-success` lässt sie dann später aus. Gegen den SimRobot setzt
`record.py` das Label weiterhin automatisch auf „erfolgreich".

*Betrieb* (2026-09-29):

1. **Modell** — Checkpoint wählen, auch Zwischenstände (`step_*`). Darunter
   steht, woraus er stammt: Datensatz, Schritte, Rate, servo_j-Rate,
   Override, Kameras der Aufzeichnung.
2. **Kameras** — dieselbe Zuordnung wie in der Aufnahme.
3. **Fahrt** — Fahrten, Takte, Neuvorhersage, Gerät, Override, Echtzeit;
   „Ohne Modell fahren" ist der Verdrahtungstest (HoldPolicy).

Über den Knöpfen steht die **Vorabprüfung**: `apps/infer.py` erzwingt
Gleichheit mit der Aufzeichnung (Schema, Rate, Kameras, Override) und
bricht sonst ab. Dieselben Prüfungen laufen hier vorher und ohne etwas zu
öffnen — am Labortag soll man sehen, dass die Kameras nicht passen, bevor
der Arm sich bewegt.

### Livebild und Aufnahme

Eine Kamera lässt sich nicht zweimal öffnen. Während einer Aufnahme
gehören die Kameras `apps/record.py`, die Oberfläche kann also nicht
nebenher mitschauen. Deshalb zwei Zustände: **Vorschau** beim Einrichten,
**Zahlen** während der Aufnahme.

Wer das Bild trotzdem sehen will, kreuzt „Livebild auch während der
Aufnahme" an. `apps/record.py` legt dann in großen Abständen ein
zusammengesetztes JPEG ab (`--preview`, `bc/preview.py`), das die
Oberfläche anzeigt. **Standardmäßig aus**: Es kostet Zeit in der
15-Hz-Schleife, und die Datenqualität geht vor. Ein Fehler dabei bricht
eine Aufnahme nie ab.

**Offen:** Abschnittserkennung für den passiven Aufnahmemodus und die
Schnittstelle zur Übergabe bei `PRE_PLACE` — beides erst nach Absprache mit
der anderen Gruppe. Deren M4-Slot erwartet `ObjectDetector.detect() ->
ObjectPose`, also eine Greifpose als Rückgabewert; hier geht es aber um die
Übergabe der Roboterhoheit. Dafür braucht es eine zweite Schnittstelle.

### Ein Override für alles

Der Geschwindigkeits-Override steht in der Backend-Leiste **und** in den
Reitern — es ist aber dieselbe Zahl (`session.set_override`), und alle
Felder ziehen mit. Zwei verschiedene Werte wären immer ein Fehler: Der
Override ist mitgelernt (AP 2.6), muss über alle Episoden eines Datensatzes
gleich sein (der Export erzwingt das) und bei der Inferenz derselbe wie in
der Aufzeichnung (`apps/infer.py` erzwingt das). Wird im Betrieb ein Modell
gewählt, setzt der Reiter den Wert auf den der Aufzeichnung.

### Mausrad

Drehfelder und Auswahlfelder reagieren **nur mit Tastaturfokus** auf das
Mausrad (`widgets.block_wheel`). Sonst verstellt man beim Scrollen durch das
Bedienfeld unbemerkt die Episodenzahl oder den Override.

## Befunde aus dem ersten Lauf (2026-09-29)

Bis dahin war die Oberfläche nie ausgeführt worden (PySide6 war nicht
installiert). Zwei Fehler, die nur beim Ausführen auffallen:

* `QProcess.processEnvironment()` liefert eine **leere** Umgebung, solange
  nichts gesetzt wurde. Wer da hineinschreibt, startet den Unterprozess mit
  genau diesen Variablen und ohne alles andere — lerobot scheitert dann
  schon beim Import („Could not determine home directory", `USERPROFILE`
  fehlt). Jetzt von `QProcessEnvironment.systemEnvironment()` ausgehend.
* `ffmpeg` liegt in dieser Umgebung unter `Library\bin` und steht nur im
  `PATH`, wenn die Umgebung aktiviert wurde. Beim Start über `python.exe -m
  bc.gui` ist sie das nicht. Die Voraussetzungszeile sieht jetzt auch neben
  dem Interpreter nach, und `ProcessRunner` ergänzt den Ordner im `PATH`
  der Unterprozesse.

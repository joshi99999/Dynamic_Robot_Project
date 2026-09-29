# Bedienoberfläche (`bc/gui`)

PySide6-Oberfläche für den Behavior-Cloning-Teil. Vier Modi:
**Systemcheck · Aufnahme · Training · Betrieb**.

## Starten

```bash
python -m bc.gui                     # alle Modi
python -m bc.gui --mode systemcheck  # direkt in einem Modus
```

Braucht `PySide6>=6.6` zusätzlich zur bisherigen Umgebung.

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

## Aufbau

| Datei | Qt? | Inhalt |
|---|---|---|
| `session.py` | nein | Backendwahl, Verbindung, Statusermittlung |
| `requirements.py` | nein | Voraussetzungen je Modus (CUDA, lerobot, neurapy, …) |
| `checks.py` | nein | Katalog der Systemcheck-Punkte, Auswertung der Ausgabe |
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
Voraussetzungsmeldungen, Modus *Systemcheck* (führt `tests/run_all.py` und
die `tools/check_*.py` als Unterprozesse aus, gestaffelt nach
hardwarefrei / lesend / mit Bewegung, Rückfrage vor jeder Bewegung).

**Gerüst:** *Aufnahme*, *Training*, *Betrieb* — zeigen Voraussetzungen und
den geplanten Aufbau, plus die Kommandozeile, die das heute schon kann.

**Offen:** Abschnittserkennung für den passiven Aufnahmemodus und die
Schnittstelle zur Übergabe bei `PRE_PLACE` — beides erst nach Absprache mit
der anderen Gruppe.

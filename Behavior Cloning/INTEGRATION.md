# Integration in `ProjektMaster_DynamicPicking`

Stand 28.10.2026. Anleitung, wie dieser Ordner (Behavior Cloning, kurz **BC**)
in das Repo `Miicrop/ProjektMaster_DynamicPicking` übernommen und als Reiter
in dessen GUI eingehängt wird. Geschrieben so, dass sie Schritt für Schritt
abgearbeitet werden kann — auch von Claude.

Quelle: Repo `joshi99999/Dynamic_Robot_Project`, Branch `main`, Ordner
`Behavior Cloning/`.

---

## 0. Regeln (bitte nicht abweichen)

1. **Am bestehenden Code der Gruppe ändert sich nichts.** Kein Edit in
   `code/`, keine neue Zeile in `code/gui/app.py`, keine Änderung an
   `code/requirements.txt` oder `code/.venv`. Der BC-Reiter wird von außen
   angehängt (`bc/gui/plugin.py`).
2. **Zwei getrennte Python-Umgebungen.**
   | Umgebung | Ort | Python | Inhalt |
   |---|---|---|---|
   | Gruppe | `code/.venv` | 3.11 (wie bisher) | GUI, M1–M4, easyocr … |
   | BC | `behavior_cloning/.venv` | **3.12** | torch (cu128 oder CPU), lerobot 0.6.1, opencv-python, PySide6 |

   **Nie** torch, lerobot oder `opencv-python` in `code/.venv` installieren.
   Grund: easyocr bringt `opencv-python-headless` und ein CPU-torch mit;
   beide beißen sich mit den BC-Versionen, wenn sie in einer Umgebung liegen.
   lerobot 0.6.1 verlangt außerdem Python 3.12.
3. Der Ordner wird **als Ganzes** übernommen und darin nichts umsortiert.
   Alle Pfade im BC-Code sind relativ zum Ordner; der Ordnername selbst ist
   egal.
4. `neurapy` wird **nicht** kopiert. BC findet das vorhandene
   `code/m2_robot_control/vendor/neurapy` von allein (gleiche Version
   v5.0.8 wie bei BC, verglichen 28.10.2026).

---

## 1. Zielstruktur

```
ProjektMaster_DynamicPicking/
├── code/                       unverändert
│   ├── .venv/                  Umgebung der Gruppe
│   ├── gui/  common/  m1_…  m2_robot_control/vendor/neurapy  …
│   └── start_gui.bat           startet die GUI wie bisher, OHNE BC
├── behavior_cloning/           NEU — Inhalt von "Behavior Cloning/"
│   ├── .venv/                  BC-Umgebung (nicht eingecheckt)
│   ├── apps/  bc/  tests/  tools/  sequences/  einstellungen/
│   ├── Dokumentation/  Berichte/  Requierments/
│   ├── BC-Oberflaeche.bat      BC allein (eigenes Fenster)
│   ├── BC-in-Gruppen-GUI.bat   Gruppen-GUI MIT Reiter "Behavior Cloning"
│   ├── start_gruppen_gui.py    dazu der Starter
│   ├── pyproject.toml  .gitignore  README.md  INTEGRATION.md
├── plan/  docs/  thesis/  …    unverändert
```

`behavior_cloning/` liegt bewusst **neben** `code/`, nicht darin: so greift
weder deren `pytest`-Konfiguration (`code/pyproject.toml`) noch deren venv
auf BC zu.

---

## 2. Ordner übernehmen

Empfohlen ist **`git subtree`**: die Historie von BC bleibt erhalten, und
spätere Änderungen lassen sich mit einem Befehl nachziehen (Abschnitt 7).

```bash
# a) Im Klon von Dynamic_Robot_Project (aktueller main):
cd Dynamic_Robot_Project
git checkout main && git pull
git subtree split --prefix="Behavior Cloning" -b bc-export

# b) Im Klon von ProjektMaster_DynamicPicking, auf einem neuen Branch:
cd ../ProjektMaster_DynamicPicking
git checkout -b behavior-cloning
git subtree add --prefix=behavior_cloning ../Dynamic_Robot_Project bc-export
```

`git subtree add` legt selbst einen Commit an. Pfad `../Dynamic_Robot_Project`
anpassen, falls die Klone anders liegen.

*Alternative ohne Historie:* den Inhalt von `Behavior Cloning/` (nur die
eingecheckten Dateien, z. B. über `git archive`) nach `behavior_cloning/`
kopieren und normal committen. Dann geht Abschnitt 7 nur durch erneutes
Kopieren.

Nach dem Übernehmen prüfen:

* `behavior_cloning/bc/__init__.py` und `behavior_cloning/apps/record.py`
  existieren.
* Keine Datenordner mitgekommen (`data_*`, `datasets/`, `checkpoints/`) — die
  schließt `behavior_cloning/.gitignore` aus.
* `git status` zeigt **keine** Änderung unter `code/`.

---

## 3. BC-Umgebung einrichten

Im Ordner `behavior_cloning/`, mit **Python 3.12** (z. B. `py -3.12`):

```bat
cd behavior_cloning
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install --upgrade pip
```

**Rechner mit NVIDIA-Karte** (Training, Inferenz in Echtzeit):

```bat
.venv\Scripts\python.exe -m pip install -r Dokumentation\environment-lock.txt --extra-index-url https://download.pytorch.org/whl/cu128
.venv\Scripts\python.exe -m pip install "PySide6>=6.6,<7" "prettytable>=3.0" "pywin32>=306"
.venv\Scripts\python.exe tools\check_gpu.py
```

Das installierte CUDA-Toolkit ist egal (auch CUDA 13): das torch-Paket
`+cu128` bringt seine Laufzeit selbst mit, es zählt nur ein aktueller
NVIDIA-Treiber. `check_gpu.py` muss die Karte und ihre Architektur
(RTX 50xx: `sm_120`) melden.

**Rechner ohne NVIDIA-Karte** (Laptop: Aufnahme, Export, Tests): wie oben,
aber torch als CPU-Build — Vorgehen in
`Dokumentation/Environment-Laptop.md`. Training wird dort von der GUI mit
Hinweis verweigert; das ist Absicht.

**Daheng-Kameras** (nur für Aufnahme an der Anlage): Galaxy-SDK mit Treiber
einmal mit Adminrechten installieren, `gxipy` daraus — siehe
`Dokumentation/Kamera-Inbetriebnahme.md`.

`.venv/` ist durch die `.gitignore` der Gruppe bereits ausgeschlossen.

> Eine automatische Installation (ein Skript, das all das erledigt, mit
> CPU/CUDA-Weiche) ist geplant und kommt als eigener Branch nach. Bis dahin
> gilt dieser Abschnitt.

---

## 4. Starten

| Was | Wie |
|---|---|
| Gruppen-GUI wie bisher, ohne BC | `code\start_gui.bat` (unverändert) |
| Gruppen-GUI **mit** Reiter „Behavior Cloning“ | `behavior_cloning\BC-in-Gruppen-GUI.bat` |
| BC allein im eigenen Fenster | `behavior_cloning\BC-Oberflaeche.bat` (mit `BC_PYTHON` auf `behavior_cloning\.venv\Scripts\python.exe`) |

`BC-in-Gruppen-GUI.bat` macht Folgendes:

1. startet `start_gruppen_gui.py` mit **`code\.venv\Scripts\python.exe`**,
2. setzt vorher `BC_PYTHON` auf das BC-Python (Suchreihenfolge: gesetzte
   Variable `BC_PYTHON`, dann `behavior_cloning\.venv`, dann die
   Entwicklungsrechner),
3. `start_gruppen_gui.py` wechselt nach `code/`, importiert
   `gui.app.MainWindow`, erzeugt das Fenster genau wie deren `main()` und
   hängt den Reiter mit `attach_to(window)` an.

Im Gruppenprozess braucht der Reiter selbst nur numpy, OpenCV (headless
genügt) und PySide6 — die sind dort schon installiert. Alles mit torch und
lerobot (Export, Training, Inferenz) läuft als Unterprozess mit `BC_PYTHON`;
die Voraussetzungsanzeige der BC-Reiter prüft torch, lerobot und CUDA
ebenfalls dort.

### Umgebungsvariablen

| Variable | Wirkung |
|---|---|
| `BC_PYTHON` | Python für die BC-Unterprozesse. Ohne sie nimmt BC das Python, in dem die GUI läuft. |
| `BC_NEURAPY_DIR` | Ordner, der `neurapy/` enthält, falls es nicht unter `code/m2_robot_control/vendor` liegt. |
| `BC_GUI_ECHO=1` | Die `.bat`-Dateien zeigen nur an, was sie starten würden. |

---

## 5. Abnahme (nach dem Einrichten einmal durchgehen)

1. `behavior_cloning\.venv\Scripts\python.exe tests\run_all.py` (im Ordner
   `behavior_cloning`) — alle hardwarefreien Tests grün. Bekannte Ausnahme:
   `test_rectify::test_homography_recovers_marker_positions` scheitert mit
   OpenCV 4.13 (ArUco-Erkennung), nicht durch Herunterstufen zudecken.
2. `set BC_GUI_ECHO=1` und `BC-in-Gruppen-GUI.bat` — beide Pythons werden
   gefunden und angezeigt.
3. Ohne `BC_GUI_ECHO` starten: die Reiterleiste zeigt die fünf Reiter der
   Gruppe und als sechsten **„Behavior Cloning“** mit den vier Modi
   Systemcheck · Aufnahme · Training · Betrieb.
4. Im BC-Reiter *Training*: die Voraussetzungszeile zeigt torch und lerobot
   als vorhanden und auf dem Rechner mit Grafikkarte CUDA mit Kartennamen.
   Steht dort „fehlt (in …\code\.venv…)“, zeigt `BC_PYTHON` nicht auf die
   BC-Umgebung.
5. `code\start_gui.bat` startet weiter wie gewohnt, ohne BC-Reiter.
6. `git status` zeigt keine Änderung unter `code/`.

Schritte 1–4 wurden am 28.10.2026 an einer Kopie des Repos (Stand
`bfed128`) nachgestellt: Reiter eingehängt, `neurapy` aus `vendor` gefunden,
Sonde im getrennten Python.

---

## 6. Bekannte Punkte

* **Roboter nie aus zwei Reitern gleichzeitig steuern.** M2 und BC verbinden
  sich jeweils selbst mit der Steuerung. Vor einer BC-Aufnahme oder -Inferenz
  die Verbindung in M2 trennen bzw. dort nichts laufen lassen.
* **Beenden:** deren `closeEvent` räumt nur die eigenen Reiter auf. BC hängt
  sich deshalb an `QApplication.aboutToQuit` und fährt Kamera und
  Servo-Schleife dort selbst herunter. Nichts zu tun — nur nicht wundern.
* **Aktualisieren der Reiter:** deren `_tab_changed` ruft `refresh()`; die
  BC-Reiter heißen genau so, es greift also automatisch.
* **Datenablage:** Aufnahmen, Datensätze und Checkpoints landen in
  `behavior_cloning/` (`data_*`, `datasets/`, `checkpoints/`) und sind nicht
  eingecheckt. Für große Datenmengen auf einer anderen Platte ist eine
  Ordnerwahl in der GUI geplant.

---

## 7. Spätere Änderungen von BC übernehmen

Solange BC in `Dynamic_Robot_Project` weiterentwickelt wird:

```bash
# im Klon von Dynamic_Robot_Project
git checkout main && git pull
git subtree split --prefix="Behavior Cloning" -b bc-export

# im Klon von ProjektMaster_DynamicPicking
git subtree pull --prefix=behavior_cloning ../Dynamic_Robot_Project bc-export
```

Wird `ProjektMaster_DynamicPicking` das Hauptrepo, entfällt das: dann wird
direkt in `behavior_cloning/` weitergearbeitet.

---

## 8. Hinweise für Claude (beim Abarbeiten dieser Anleitung)

* Nur in `behavior_cloning/` schreiben. Muss in `code/` doch etwas geändert
  werden, **vorher den Anwender fragen** — die Vorgabe ist, dass es nicht
  nötig ist.
* Keine Pakete in `code/.venv` installieren oder aktualisieren.
* Nicht committen oder pushen, ohne dass der Anwender es verlangt.
* Bei Fehlern in Abschnitt 5 erst `BC_PYTHON` und den `neurapy`-Pfad prüfen,
  bevor Code geändert wird.

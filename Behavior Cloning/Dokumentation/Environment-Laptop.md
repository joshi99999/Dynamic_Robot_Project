# Environment für den Aufnahme-Laptop — was die GUI braucht

Stand: 2026-10-01 · Alle Angaben auf dem Laptop selbst geprüft; Paketdaten
am 2026-10-01 gegen `pypi.org` verifiziert (nur Metadaten gelesen, nichts
installiert).

Gegenstück zu [Environment-Einrichtung.md](Environment-Einrichtung.md) (PC,
`E:\Enviroments\bc_env`, RTX 5070 Ti). Befunde zum Rechner selbst stehen in
[Laptop-Inbetriebnahme-Befunde.md](Laptop-Inbetriebnahme-Befunde.md).

> **Stand 2026-10-01: `C:\Enviromentsc_env` ist angelegt und abgenommen**
> (Python 3.11, ohne torch/lerobot). Siehe Abschnitt 4. Eingefrorener
> Ist-Stand: [environment-lock-laptop.txt](environment-lock-laptop.txt).

---

## 1. Ausgangslage

| | |
|---|---|
| Python | **nur 3.11.9**, System-Interpreter `C:\Users\jerem\AppData\Local\Programs\Python\Python311` |
| `py --list` | `-V:3.11 *` — keine weitere Version installiert |
| Laufwerk `E:` | existiert nicht, `E:\Enviroments\bc_env` also auch nicht |
| venvs | keine |
| vorhanden | `numpy 1.26.4`, `opencv-python 4.11.0.86`, `scipy 1.13.1`, `pywin32 306`, `prettytable 3.17.0` |
| **fehlt** | **`PySide6`**, `torch`, `lerobot`, `pygame`, `gxipy`, `ffmpeg` |
| Testsuite | `tests/run_all.py` → **251/251 grün** (auf 3.11) |
| Galaxy SDK | installiert unter `C:\Program Files\Daheng Imaging\GalaxySDK` |

Der System-Interpreter ist **kein reines Projekt-Environment**: darin liegen
Jupyter, dash, open3d, matplotlib und scikit-learn aus anderer Arbeit.

---

## 2. Die zwei Randbedingungen, die alles bestimmen

### 2.1 Ein Interpreter muss alles tragen

`bc/gui/runner.py` startet jede Prüfung, jede Aufnahme und jedes Training als
Unterprozess mit **`sys.executable`** — also mit genau dem Interpreter, der
die GUI ausführt. Die Abhängigkeiten lassen sich deshalb **nicht** auf zwei
Umgebungen verteilen: was ein Modus braucht, muss dort liegen, wo die GUI
läuft.

### 2.2 `lerobot` erzwingt Python 3.12

`lerobot==0.6.1` deklariert `requires-python >=3.12` (am 2026-10-01 geprüft;
0.6.1 ist weiterhin die neueste Version). Auf Python 3.11 ist es **nicht
installierbar**.

Zusammen mit 2.1 heißt das: **Export, Training und Betrieb sind auf dem
Python 3.11 dieses Laptops dauerhaft nicht erreichbar** — auch nicht, wenn
lerobot irgendwo sonst läge.

---

## 3. Was welcher Modus braucht

PySide6 steht in keiner Zeile, weil es **Voraussetzung zum Starten** ist und
keine Voraussetzung eines Modus: ohne Qt öffnet sich kein Fenster.

| Modus | erforderlich | optional |
|---|---|---|
| **Systemcheck** | numpy, OpenCV | neurapy *(Controller-Punkte)*, torch *(nur „GPU / CUDA")* |
| **Aufnahme** | OpenCV, neurapy | gxipy *(nur wenn eine Daheng zugeordnet ist)*, pygame *(nur `apps/teach.py`)* |
| **Training** | **torch, lerobot** | CUDA, FFmpeg |
| **Betrieb** | **torch, lerobot**, neurapy, OpenCV | CUDA, gxipy |

Was die GUI auf diesem Rechner heute selbst meldet
(`bc.gui.requirements.report_for`):

```
systemcheck   erfüllt    numpy OK · OpenCV OK · neurapy OK · torch fehlt (opt)
recording     erfüllt    OpenCV OK · neurapy OK · gxipy fehlt (opt)
training      NEIN       torch FEHLT · lerobot FEHLT · CUDA fehlt · FFmpeg fehlt
operation     NEIN       torch FEHLT · lerobot FEHLT · neurapy OK · OpenCV OK
```

Systemcheck und Aufnahme sind also **jetzt schon vollständig bedienbar** —
es fehlt nur Qt zum Starten.

---

## 4. Was eingerichtet ist

`C:\Enviromentsc_env` — **Python 3.11**, bewusst ohne torch und lerobot.
Damit entfällt die Installation von Python 3.12 vollständig: lerobot verlangt
torch hart, ohne torch gibt es also auch keinen Grund für 3.12.

```bash
py -3.11 -m venv C:\Enviromentsc_env
C:\Enviromentsc_env\Scripts\python.exe -m pip install --upgrade pip
C:\Enviromentsc_env\Scripts\python.exe -m pip install "numpy>=1.26,<2.3" "scipy>=1.13,<2" "opencv-python>=4.9,<4.14" "PySide6-Essentials>=6.6,<7" "pywin32>=306" "prettytable>=3.17" "pygame>=2.5,<3"
```

Starten:

```bash
C:\Enviromentsc_env\Scripts\python.exe -m bc.gui
```

### 4.1 Gemessener Aufwand

| | Pakete | Download | belegt |
|---|---|---|---|
| **dieses venv** (ohne torch) | 9 | **186 MB** | **595 MB** |
| mit torch + lerobot (bräuchte 3.12) | 73 | 448 MB | ~1,5 GB |

Die größten Posten im venv: PySide6 207 MB, scipy 119 MB, cv2 109 MB,
numpy 59 MB, pygame 27 MB. Installationsdauer lag bei wenigen Minuten,
überwiegend Entpacken.

`PySide6-Essentials` statt `PySide6`, weil `bc/gui` nur `QtCore`, `QtGui` und
`QtWidgets` benutzt. Das vollständige Paket zieht zusätzlich
`PySide6-Addons` (168 MB) für Qt3D, Multimedia und WebEngine — hier
ungenutzt. `pyproject.toml` deklariert `PySide6`; das ist eine bewusste
Abweichung zugunsten der kleineren Installation.

`scipy` ist mit drin, weil `pyproject.toml` es deklariert — benutzt wird es
im Code nirgends (bekannter offener Punkt). Ohne scipy wären es 119 MB
weniger.

### 4.2 Wenn später doch Export und Training auf dem Laptop gebraucht werden

Dann ist ein **zweites** Environment mit Python 3.12 nötig, nicht eine
Erweiterung dieses einen — `lerobot==0.6.1` ist auf 3.11 nicht
installierbar. Reihenfolge:

```bash
py -3.12 -m venv C:\Enviromentsc_env312
… wie oben, aber numpy auf ">=2.0,<2.3"
… "torch==2.11.0" "torchvision==0.26.0"          # CPU-Build, KEIN cu128-Index
… "lerobot==0.6.1" "diffusers==0.39.0" "torchcodec==0.11.1" "av==15.1.0" "datasets==4.8.5"
… "opencv-python>=4.9,<4.14" --upgrade --force-reinstall   # ZULETZT, siehe unten
```

`lerobot` verlangt hart `opencv-python-headless`. Das installiert ein zweites
Paket mit derselben `cv2`-Bibliothek; welches gewinnt, entscheidet die
Reihenfolge. Die Werkzeuge brauchen die **nicht-headless** Variante
(`cv2.imshow` in `tools/check_cameras.py` und `tools/check_rectify.py`),
deshalb kommt sie zum Schluss. pip warnt danach über den „überschriebenen"
Konflikt — das ist in Ordnung, `opencv-python` ist headless plus
GUI-Fenster. Genau so sieht auch [environment-lock.txt](environment-lock.txt)
vom PC aus.

Der cu128-Index aus [Environment-Einrichtung.md](Environment-Einrichtung.md)
gilt **nur für den Trainingsrechner**. Dieser Laptop hat kein CUDA; der
PyPI-Standardbuild ist hier richtig und ~115 MB statt ~2,5 GB.

## 5. Was pip nicht installiert

### 5.1 `neurapy` — liegt im Repo

Kommt aus `Dynamic_Robot_Project/Robot Controller/neurapy` und wird von
`bc/gui/_bootstrap.py` zur Laufzeit in `sys.path` gehängt. Nichts zu
installieren — aber es **braucht `pywin32` und `prettytable`** im
Environment, sonst scheitert schon der Import (in Schritt 3 enthalten).

Der Ordner `Dynamic_Robot_Project/neurapy` enthält nur ein verwaistes
`__pycache__` und ist **nicht** das Paket.

### 5.2 `gxipy` — aus dem Galaxy SDK, per Pfad

Das SDK bringt kein pip-Paket, sondern ein nacktes Paketverzeichnis:

```
C:\Program Files\Daheng Imaging\GalaxySDK\Development\Samples\Python\gxipy
```

Geprüft: über diesen Pfad importierbar, Version `2.0.2512.9261`. Im venv
bekanntmachen, ohne etwas zu kopieren — eine `.pth`-Datei in `site-packages`:

```bash
echo C:\Program Files\Daheng Imaging\GalaxySDK\Development\Samples\Python> C:\Enviroments\bc_env\Lib\site-packages\gxipy.pth
```

Nur nötig, wenn eine Daheng-Kamera zugeordnet wird. Ohne sie bleiben
Platzhalter und die Logitech als Ersatz.

### 5.3 FFmpeg — für Export und Videodekodierung

`lerobot` kodiert die Episodenvideos damit, `torchcodec` liest sie. Auf dem PC
liegen die DLLs in `Library\bin` neben dem Interpreter.

`bc/gui/requirements.py` sucht `ffmpeg` im `PATH` **und neben dem
Interpreter**; findet es es nur daneben, ergänzt `ProcessRunner` den Ordner
im `PATH` der Unterprozesse. Es reicht also, `ffmpeg.exe` nach
`C:\Enviroments\bc_env\Scripts\` zu legen.

---

## 6. Abnahme — Ergebnis vom 2026-10-01

```bash
C:\Enviromentsc_env\Scripts\python.exe tests/run_all.py
```

**250 bestanden, 1 fehlgeschlagen** —
`test_rectify::test_homography_recovers_marker_positions`.

Voraussetzungsbericht der GUI aus diesem venv:

```
systemcheck   erfüllt    numpy 2.2.6 · cv2 4.13.0 · neurapy · torch fehlt (opt)
recording     erfüllt    cv2 4.13.0 · neurapy · gxipy 2.0.2512.9261
training      NEIN       torch FEHLT · lerobot FEHLT · CUDA · FFmpeg
operation     NEIN       torch FEHLT · lerobot FEHLT · neurapy OK · cv2 OK · gxipy OK
```

Systemcheck und Aufnahme sind damit **vollständig bedienbar**, Aufnahme
einschließlich Daheng-Zuordnung.

### 6.1 Der eine Ausfall: OpenCV 4.13 erkennt weniger ArUco-Marker

Das ist **nicht neu kaputt**, sondern der schon in `CLAUDE.md` notierte
offene Punkt („ArUco bei 240x320 grenzwertig", AP 1.4) — jetzt mit Ursache.
Dieselbe synthetische Szene, dieselbe Bildgröße, nur die OpenCV-Version
unterscheidet sich:

| Szene (240 × 320) | OpenCV 4.11.0 | OpenCV 4.13.0 |
|---|---|---|
| ohne Kippung | 4 von 4 Markern | 4 von 4 |
| `dx=5, dy=-3, tilt=0.02` | 4 von 4 | 4 von 4 |
| **`dx=15, dy=-8, tilt=0.05`** (der Test) | **4 von 4** | **2 von 4** |

Der ArUco-Detektor von 4.13 verliert bei Kippung auf dieser kleinen
Bildgröße zwei Marker; der Test verlangt mindestens 3. Der System-Python des
Laptops hat 4.11.0.86 und läuft deshalb grün — der PC hat laut
[environment-lock.txt](environment-lock.txt) **4.13.0.92** und müsste
denselben Ausfall zeigen.

**Bewusst nicht auf 4.11 gepinnt.** Grün wäre es damit sofort, aber die
Abweichung zum PC wäre dann im Environment versteckt statt dokumentiert —
und die Rektifizierung läuft auf genau diesen 240 × 320 (`config.IMAGE_*`).
Zu klären, sobald die Szenenkamera entschieden ist (AP 1.4):

1. Ist die Toleranz des Tests zu eng, oder ist 4.13 an dieser Bildgröße
   wirklich schlechter? Mit echten Markerbildern nachmessen, nicht
   synthetisch.
2. Reicht die Rektifizierung bei 240 × 320 überhaupt, oder muss sie auf dem
   vollen Kamerabild laufen und erst danach skaliert werden?

Wer vorher Grün braucht: `opencv-python==4.11.0.86` installieren. Dann aber
beide Rechner umstellen, nicht nur einen.

## 7. Was danach noch nicht geht — und warum das in Ordnung ist

| | Grund |
|---|---|
| Policy-Fahrt in Echtzeit | kein CUDA; 384 ms je Vorhersage gegen 67 ms Takt |
| Training in brauchbarer Zeit | dito — der Datensatz ist portabel und gehört auf den Trainingsrechner |
| Daheng-Aufnahme | Kamera ist nicht enumeriert (Kabel/Port, Befunde 4.2) |
| 15-Hz-Takt sauber halten | auf diesem Laptop nicht gehalten (Befunde 6.4); an der Anlage neu zu messen |

Keiner dieser Punkte hindert Systemcheck, Aufnahme oder Export — und das ist
das, was der Laptop am Labortag leisten muss.

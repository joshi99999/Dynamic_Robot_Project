# Environment — Einrichtung (Stand 11.09.2026)

Handlungsanleitung zum Aufsetzen des Projekt-Environments für die
BC-Pipeline auf dem Windows-Trainingsrechner (RTX 5070 Ti).
Begründungen und Anforderungen: [../Requierments/requierments.md](../Requierments/requierments.md)
(AP 0.9 Punkt 6/9, AP 0.10, AP 3.1).

**Status: am 11.09.2026 so durchgeführt.** Das Environment steht unter
`E:\Enviroments\bc_env`; das Abnahmeprotokoll steht in Abschnitt 5, der
eingefrorene Ist-Stand in [environment-lock.txt](environment-lock.txt).
`E:\Enviroments\ML_env` bleibt unangetastet (Fremd-Environment,
Python 3.12.11, kein OpenCV).

---

## 1. Was die Recherche ergeben hat — vier Korrekturen zur Vorplanung

Alle Angaben am 11.09.2026 gegen `pypi.org` und `download.pytorch.org`
geprüft (nur Metadaten gelesen, nichts installiert).

### 1.1 Python 3.11 geht nicht — es muss 3.12 sein

`lerobot` fordert seit 0.5.0 durchgängig `requires-python >=3.12`
(geprüft für 0.5.0, 0.5.1, 0.6.0, 0.6.1). Damit ist die Python-Version
durch LeRobot bestimmt, nicht frei wählbar. `pyproject.toml` verlangt
`>=3.11` — 3.12 erfüllt das, die Datei muss dafür nicht geändert werden.

Gewählt: **Python 3.12**. Passt zum Bestand (die vorhandenen
`__pycache__`-Dateien sind `cpython-312`). 3.13 wäre denkbar, bringt aber
ohne Not Wheel-Risiko bei den Randpaketen.

### 1.2 „torch neueste Version" geht nicht — 2.11.0 ist die Obergrenze

`lerobot==0.6.1` pinnt seine Umgebung eng:

| Paket | LeRobot-Bereich | aktuell auf PyPI | brauchbar |
|---|---|---|---|
| `torch` | `>=2.7,<2.12` | 2.14.0 | **2.11.0** |
| `torchvision` | `>=0.22,<0.27` | 0.29.0 | **0.26.0** |
| `numpy` | `>=2.0,<2.3` | 2.5.3 | 2.2.x |
| `opencv-python-headless` | `>=4.9,<4.14` | — | 4.13.x |
| `torchcodec` (win32) | `>=0.7,<0.12` | 0.16.0 | **0.11.1** |

Die passenden cu128-Wheels existieren und sind geprüft:
`torch-2.11.0+cu128-cp312-cp312-win_amd64.whl` (hochgeladen 27.04.2026) und
`torchvision-0.26.0+cu128-cp312-cp312-win_amd64.whl`.

`torchcodec` ist versionsgekoppelt an torch: 0.11.0 erschien am 24.03.2026,
torch 2.11.0 am 23.03.2026; 0.12.0 gehört bereits zu torch 2.12. Die
LeRobot-Obergrenze `<0.12` ist also genau die torch-2.11-Zeile — die Pins
sind untereinander konsistent.

### 1.3 ffmpeg ist **nicht** zwingend

Nachgesehen im Wheel von `lerobot 0.6.1`:

* **Encoding** (`lerobot/datasets/video_utils.py`, `encode_video_frames`,
  `concatenate_video_files`) läuft vollständig über **PyAV** (`import av`).
  PyAV bringt FFmpeg in seinen Wheels mit — für das Schreiben der MP4s ist
  kein System-FFmpeg nötig.
* **Decoding** beim Training nutzt bevorzugt `torchcodec`, und *das* braucht
  FFmpeg-Bibliotheken im PATH (torchcodec liefert keine mit).
* Fehlt FFmpeg, fängt LeRobot das ab: `lerobot/utils/import_utils.py:72`
  (`get_safe_default_video_backend`) fällt bei `ImportError/OSError/
  RuntimeError` automatisch auf `pyav` zurück und schreibt eine Warnung.

Konsequenz: FFmpeg ist ein **Performance-Thema beim Training**, kein
Blocker. Schritt 9 ist optional und kann nachgeholt werden.

### 1.4 OpenCV: voll oder headless — beides gleichzeitig geht nicht sauber

`lerobot[dataset]` zieht `opencv-python-headless`. Dieses Paket und
`opencv-python` installieren **dieselben Dateien** (`cv2/`); pip erkennt den
Konflikt nicht, das zuletzt installierte gewinnt.

Im Projekt braucht genau eine Stelle das volle Paket:
`tools/check_cameras.py:218` (`cv2.imshow` für die Live-Vorschau). Mit
`--no-display` läuft das Werkzeug auch headless. `cv2.aruco` (Rektifizierung
AP 1.4) ist seit OpenCV 4.7 im Hauptmodul und damit in **beiden** Varianten
enthalten — `opencv-contrib-python` wird nicht gebraucht.

Empfehlung: volles `opencv-python` **zuletzt** installieren (Schritt 6).

### 1.5 Unverändert richtig

* `gxipy` gibt es nicht per pip — kommt aus dem Daheng Galaxy SDK
  (Schritt 10). `bc/adapters/cam_daheng.py` sucht den Pfad zur Laufzeit.
* `neurapy` ist Fremdcode in `Robot Controller/`, braucht aber `pywin32`
  und `prettytable` aus pip, sonst scheitert schon der Import.
* `scipy` wird im gesamten Code nirgends importiert → weglassen. (Nur
  einzelne LeRobot-Extras wie `pi`, `wallx`, `phone` würden es nachziehen —
  die brauchen wir nicht.)

---

## 2. Zielzustand

| | |
|---|---|
| Ort | `E:\Enviroments\bc_env` (neben den bestehenden Environments) |
| Anlage | conda, **nur für Python** — alles Weitere per pip |
| Python | 3.12 (installiert: 3.12.14 aus `pkgs/main`, pip 26.2.1) |
| torch / torchvision | 2.11.0+cu128 / 0.26.0+cu128 |
| lerobot | **0.6.1**, Extras `dataset,diffusion` |
| Datensatzformat | `CODEBASE_VERSION = "v3.0"` (aus `lerobot/datasets/dataset_metadata.py:60`) |
| OpenCV | `opencv-python==4.13.0.92` (voll, statt headless) |
| Windows-Zubehör | `pywin32`, `prettytable` (für neurapy) |
| Teachen | `pygame` |
| Tests | `pytest>=8,<9` |
| FFmpeg | **n8.1 (LGPL, shared)**, als DLLs in `bc_env\Library\bin` — kein PATH-Eintrag, siehe Schritt 9 |

Warum conda nur für Python: die cu128-Wheels kommen aus einem eigenen
Index, und gemischte conda/pip-Installationen derselben Bibliothek (numpy,
Pillow, FFmpeg-Libs) sind die häufigste Ursache für DLL-Konflikte unter
Windows.

---

## 3. Voraussetzungen (bereits geprüft)

* NVIDIA-Treiber 610.88, meldet CUDA UMD 13.3 → cu128-Wheels laufen
  (Treiber ist abwärtskompatibel). `nvidia-smi` ist im PATH.
* conda 26.1.1 vorhanden (`C:\Users\jerem\miniconda3`).
* Kein `ffmpeg` im PATH, kein Galaxy SDK installiert — beides ist
  Gegenstand der optionalen Schritte 9 und 10.
* `python` liegt nicht im PATH; im Folgenden immer erst das Environment
  aktivieren.

---

## 4. Installation Schritt für Schritt

Alle Befehle in **PowerShell**, ausgeführt aus
`E:\Projekt\Dynamic_Robot_Project\Behavior Cloning`.

### Schritt 1 — Environment anlegen (nur Python)

```powershell
conda create -p E:\Enviroments\bc_env python=3.12 -y
conda activate E:\Enviroments\bc_env
python -V
python -m pip install --upgrade pip
```

Erwartet: `Python 3.12.x`.

### Schritt 2 — Constraints-Datei anlegen

Sie verhindert, dass eine spätere Installation den cu128-Build von torch
durch den CPU-/Default-Build von PyPI ersetzt. Das ist der Fallstrick, an
dem die Einrichtung sonst still scheitert.

```powershell
@"
torch==2.11.0+cu128
torchvision==0.26.0+cu128
"@ | Out-File -Encoding utf8 E:\Enviroments\bc_env\constraints.txt
```

### Schritt 3 — torch und torchvision aus dem cu128-Index (zuerst!)

```powershell
pip install --index-url https://download.pytorch.org/whl/cu128 --extra-index-url https://pypi.org/simple torch==2.11.0+cu128 torchvision==0.26.0+cu128
```

Der zusätzliche PyPI-Index ist ungefährlich: das lokale Versions-Suffix
`+cu128` gibt es dort nicht, die Auflösung bleibt eindeutig.

### Schritt 4 — GPU verifizieren, bevor es weitergeht

```powershell
python tools\check_gpu.py
```

Abnahme: `get_arch_list()` muss **`sm_120`** enthalten. `is_available() ==
True` allein genügt nicht — ohne sm_120 fällt der Fehler erst beim ersten
Kernel-Aufruf („no kernel image is available for execution on the device").

Schlägt das fehl: Schritt 3 wiederholen, nicht weitermachen.

### Schritt 5 — LeRobot mit gepinnter Version

```powershell
pip install -c E:\Enviroments\bc_env\constraints.txt "lerobot[dataset,diffusion]==0.6.1"
```

* `dataset` → `datasets`, `pyarrow`, `pandas`, `av`, `torchcodec`,
  `jsonlines`; das ist alles, was `LeRobotDataset` zum Schreiben und Lesen
  der Episoden braucht.
* `diffusion` → `diffusers` für die Diffusion Policy.
* `training` (wandb + accelerate) ist **absichtlich nicht dabei**: die
  Pipeline soll lokal und autark laufen. Bei Bedarf nachinstallierbar.

Direkt danach kontrollieren, dass torch nicht ersetzt wurde:

```powershell
python -c "import torch, torchvision; print(torch.__version__, torchvision.__version__)"
```

Erwartet: `2.11.0+cu128 0.26.0+cu128`. Steht dort `2.11.0` ohne `+cu128`,
wurde der CPU-Build gezogen → Schritt 3 wiederholen.

### Schritt 6 — OpenCV auf die volle Variante umstellen

```powershell
pip uninstall -y opencv-python-headless
```

```powershell
pip install opencv-python==4.13.0.92
```

Die Version liegt bewusst im von LeRobot geforderten Fenster
(`>=4.9,<4.14`). `pip check` meldet danach, dass `lerobot` das
headless-Paket vermisst — das ist erwartet und folgenlos, beide liefern
dasselbe `cv2`-Modul. Wer die Meldung nicht will, lässt Schritt 6 weg und
ruft `tools\check_cameras.py` immer mit `--no-display` auf.

### Schritt 7 — Abhängigkeiten für neurapy

```powershell
pip install pywin32 prettytable
```

Falls `import win32api` danach mit einem DLL-Fehler scheitert (kommt in
conda-Environments gelegentlich vor):

```powershell
python E:\Enviroments\bc_env\Scripts\pywin32_postinstall.py -install
```

### Schritt 8 — Teachen und Tests

```powershell
pip install "pygame>=2.5,<3" "pytest>=8,<9"
```

`pygame` ist nur am Steuerungsrechner nötig (AP 2.2). `pytest` ist optional —
`tests/run_all.py` läuft auch ohne.

### Schritt 9 — FFmpeg (nur für den torchcodec-Decoder)

Nur nötig, damit `torchcodec` statt `pyav` decodiert. Zwei Dinge sind dabei
entscheidend, beide nicht offensichtlich:

**Ein PATH-Eintrag hilft nicht.** torchcodec lädt seine Bibliotheken über
`torch.ops.load_library(<absoluter Pfad>)`, das landet bei `ctypes.CDLL`,
und CPython setzt dort seit 3.8 die Flags `LOAD_LIBRARY_SEARCH_DEFAULT_DIRS
| LOAD_LIBRARY_SEARCH_DLL_LOAD_DIR` (`Lib/ctypes/__init__.py:368`). Diese
Suchreihenfolge **schließt den PATH aus**; durchsucht werden nur das
Verzeichnis der DLL selbst, `System32`, das Verzeichnis der `python.exe`
und alles, was per `os.add_dll_directory` angemeldet wurde.

**Deshalb gehören die DLLs nach `<env>\Library\bin`.** Genau dieses
Verzeichnis meldet `torch/__init__.py:172` als `sys.exec_prefix\Library\bin`
per `os.add_dll_directory` an, und torch ist beim Laden von torchcodec
immer schon importiert. Das funktioniert damit auch ohne `conda activate`,
braucht keinen PATH-Eintrag und hält alles im Environment-Ordner.

**Die FFmpeg-Version muss zu torchcodec passen.** Unterstützt sind die
Major-Versionen 4 bis 8; torchcodec probiert `libtorchcodec_core8` …
`core4` der Reihe nach. Der `master`-Build von BtbN ist bereits
FFmpeg 9 (`avutil-61`, `avcodec-63`) und lädt **nicht** — es muss ein
getaggter `n8.x`-Build sein (`avutil-60`, `avcodec-62`, `avformat-62`).

So durchgeführt:

1. `ffmpeg-n8.1-latest-win64-lgpl-shared-8.1.zip` von
   [github.com/BtbN/FFmpeg-Builds](https://github.com/BtbN/FFmpeg-Builds/releases/tag/latest)
   herunterladen (73 MB). LGPL statt GPL genügt: encodiert wird über PyAV,
   FFmpeg wird hier nur zum Decodieren gebraucht.
2. Aus dem `bin`-Ordner des Archivs nach
   `E:\Enviroments\bc_env\Library\bin` kopieren:
   `avutil-60.dll`, `avcodec-62.dll`, `avformat-62.dll`, `avfilter-11.dll`,
   `avdevice-62.dll`, `swscale-9.dll`, `swresample-6.dll` sowie
   `ffmpeg.exe` und `ffprobe.exe`. `ffplay.exe` wird nicht gebraucht.
   Vorher auf Namenskollisionen prüfen — es gab keine.

Prüfen:

```powershell
python -c "from torchcodec.decoders import VideoDecoder; from lerobot.utils.import_utils import get_safe_default_video_backend as b; print('Backend:', b())"
```

Erwartet: `Backend: torchcodec`. Schlägt der Import fehl, ist das kein
Defekt: LeRobot fällt automatisch auf PyAV zurück (siehe 1.3), das Training
wird nur langsamer.

**Hinweis zur Deinstallation:** Diese neun Dateien sind conda unbekannt und
tauchen weder in `conda list` noch in `pip freeze` auf. Sie liegen
vollständig innerhalb von `E:\Enviroments\bc_env`; ein Löschen des
Environment-Ordners entfernt sie mit.

### Schritt 10 — Daheng Galaxy SDK (kein pip, für die Wrist-Kamera)

Galaxy SDK für Windows installieren (Daheng Imaging, „Galaxy Windows SDK");
es legt `gxipy` unter
`C:\Program Files\Daheng Imaging\GalaxySDK\Development\Samples\Python` ab.
`bc/adapters/cam_daheng.py` findet den Pfad selbst und zieht die DLL-Pfade
aus der Registry nach; ein abweichender Ort ist über die Umgebungsvariable
`GALAXY_SDK_PYTHON` setzbar. Details und Kamera-Einstellungen:
[Kamera-Inbetriebnahme.md](Kamera-Inbetriebnahme.md).

Nach der Installation **neue Shell öffnen** (gleicher PATH-Grund wie oben).

---

## 5. Abnahme des Environments

Der Reihe nach, alles im aktivierten Environment:

```powershell
python tools\check_gpu.py
```

```powershell
python tests\run_all.py
```

```powershell
python tools\check_sim_robot.py
```

```powershell
python apps\record.py --sim --episodes 3 --out data_sim
```

### Ergebnis der Abnahme am 11.09.2026

| Prüfung | Ergebnis |
|---|---|
| `check_gpu.py` | **OK** — `torch 2.11.0+cu128`, `torch.version.cuda 12.8`, Compute Capability `sm_120`, kompilierte Architekturen `sm_75, sm_80, sm_86, sm_90, sm_100, sm_120`. ResNet18 (Batch 64 @ 224²): 26,5 ms/Schritt, 2417 Bilder/s, 0,94 GB Peak-VRAM |
| Import-Test | **OK** — `numpy 2.2.6`, `cv2 4.13.0` (mit `aruco` **und** `imshow`), `lerobot 0.6.1`, `av 15.1.0`, `diffusers 0.39.0`, `pygame 2.6.1`, `win32api`, `prettytable 3.18.0` |
| `CODEBASE_VERSION` | **v3.0** — wie erwartet |
| Video-Backend | **`torchcodec`** — nach Schritt 9 (FFmpeg n8.1 in `Library\bin`). Roundtrip geprüft: PyAV encodiert 30 Frames nach MP4 (SVT-AV1), torchcodec liest sie zurück als `(3, 240, 320) float32`; über alle 30 Frames liefern torchcodec und PyAV **identische** Bilder |
| `tests/run_all.py` | **74 von 75 grün**; einzige Ausnahme `test_rectify::test_homography_recovers_marker_positions` — Befund siehe Abschnitt 7 |
| `check_sim_robot.py` | **nicht durchgeführt** — `TimeoutError [WinError 10060]` auf `192.168.2.13:65432`, die VM `Lara5_V5` lief nicht. Der `neurapy`-Import selbst lief durch, `pywin32`/`prettytable` sind damit verifiziert |
| `record.py --sim` | **OK** — 2 Episoden à 83 Schritte bei 15 Hz, 0 verworfen, 0,0 % Sync-Verletzungen |
| `pip check` | meldet `lerobot 0.6.1 requires opencv-python-headless, which is not installed` — **erwartet**, Folge von Schritt 6 |

Ist-Stand eingefroren (77 Pakete):

```powershell
pip freeze > Dokumentation\environment-lock.txt
```

---

## 6. Reihenfolge — die drei Stellen, an denen es schiefgeht

1. **torch vor lerobot.** Umgekehrt zieht die Abhängigkeitsauflösung ein
   torch vom PyPI-Default und überschreibt den cu128-Build. Die
   Constraints-Datei aus Schritt 2 ist die Absicherung dagegen.
2. **Die LeRobot-Version bestimmt die Python-Version**, nicht umgekehrt
   (1.1). Deshalb wird die Version *vor* dem Anlegen des Environments
   festgelegt.
3. **Volles OpenCV zuletzt**, sonst überschreibt das headless-Paket von
   LeRobot das `cv2`-Modul und die Kamera-Vorschau stirbt mit
   „The function is not implemented".

---

## 7. Offene Punkte

### 7.1 Befund aus der Abnahme: ArUco-Erkennung ist bei 240 × 320 grenzwertig

`test_rectify::test_homography_recovers_marker_positions` fällt mit
OpenCV 4.13: bei `tilt=0.05` erkennt der Detektor nur **2 der 4** Marker,
der Test verlangt aber mindestens 3. Nachgemessen über die
Kipp-Reihe derselben Szene:

| `tilt` | erkannte Marker |
|---|---|
| 0,00 | 0, 1, 2, 3 |
| 0,02 | 1, 2, 3 |
| 0,03 | 0, 1, 2, 3 |
| 0,04 | 0, 1, 2, 3 |
| 0,05 | **0, 3** |

Das Verhalten ist nicht monoton — typisch für eine Erkennung am Rand ihrer
Auflösungsgrenze, nicht für einen Installationsfehler. Bei
`config.IMAGE_*` = 240 × 320 und einem 4-cm-Marker in einem 40 × 30-cm-
Workspace ist ein Marker nur rund 25–30 px groß; perspektivische
Verkleinerung plus Interpolationsunschärfe kippen ihn dann unter die
Erkennungsschwelle.

Das ist ein **Sachbefund für AP 1.4**, nicht nur ein Testproblem: an der
realen Anlage muss entweder der Marker größer, der Workspace-Ausschnitt
enger oder die Rektifizierung auf einem höher aufgelösten Bild als dem
Policy-Bild gerechnet werden. Vor einer Entscheidung darüber sollte die
Testschwelle **nicht** abgesenkt werden — sie zeigt genau dieses Risiko an.

### 7.2 Noch offen aus der Einrichtung

* **Schritt 10 (Galaxy SDK) steht aus** — ohne SDK kein `gxipy` und damit
  keine Wrist-Kamera.
* **Decoder-Toleranz nicht zu groß wählen.** Ist `tolerance_s` größer als
  ein Frame-Abstand (bei 15 Hz: 66,7 ms), dürfen torchcodec und PyAV
  verschiedene Nachbarframes zum selben Zeitstempel liefern — beim Test mit
  `tolerance_s=0.1` traten so Unterschiede bis 0,80 (auf 0–1 normiert) auf,
  mit `0.02` waren beide deckungsgleich. `config.SYNC_MAX_SKEW_S` = 30 ms
  liegt sicher darunter; relevant wird es, wenn beim Training eine eigene
  Toleranz gesetzt wird.
* `check_sim_robot.py` ist gegen die laufende VM zu wiederholen.

### 7.3 Offene Punkte, die dieses Dokument erzeugt

* `pyproject.toml` ist noch nicht nachgezogen: `lerobot==0.6.1` steht dort
  weiterhin auskommentiert (AP 0.9 Punkt 6), `scipy` ist deklariert, wird
  aber nicht gebraucht, und `numpy<3` ist faktisch durch LeRobot auf
  `<2.3` verengt. Nach erfolgreicher Einrichtung angleichen.
* Das Datensatzformat ist ab der ersten echten Aufzeichnung an
  **LeRobot 0.6.1 / Format v3.0** gebunden. Ein späterer Versionswechsel
  entwertet vorhandene Aufnahmen — Pin nicht stillschweigend anheben.
* `bc/dataset.py:255` (`to_lerobot`) und `bc/policy.py` sind gegen die
  gepinnte Version noch zu implementieren (AP 0.9 Punkt 9).

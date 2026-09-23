# Export, Training und Inferenz (AP 3 / AP 4)

Stand 17.09.2026. Handlungsanleitung für die Kette **Aufzeichnung → LeRobotDataset
→ Diffusion Policy → Policy fährt selbst**. Begründungen stehen in den
Docstrings von `bc/lerobot_io.py`, `bc/diffusion.py`, `bc/policy.py`,
`apps/train.py` und `apps/infer.py`; die Anforderungen in
[../Requierments/requierments.md](../Requierments/requierments.md).

```
apps/record.py  ->  data_*/<Block>/            neutrales Format (npz + json), je Block eine Objektlage
apps/export.py  ->  datasets/<Name>/           LeRobotDataset v3.0 (lerobot 0.6.1), portabel
apps/train.py   ->  checkpoints/<Name>/policy  Diffusion Policy + bc_policy.json
apps/infer.py   ->  <policy>/rollouts/<Zeit>   Fahrtprotokolle + summary.json
```

---

## 1. Aufzeichnen: ein Aufruf = ein Block = eine Objektlage

Ablauf an der Anlage (vom Anwender festgelegt, 17.09.2026):

1. Objekt von Hand an eine neue, möglichst unterschiedliche Lage legen.
2. Die Greifpunkte (`PRE_GRASP`, `PICK`) am Pendant per **Touch-up** nachteachen.
3. Aufzeichnen — `record.py` liest die Punkte vor jeder Episode frisch aus der
   Controller-Datenbank, die neue Lage wirkt sofort.
4. Zurück zu 1.

```powershell
python apps/record.py --robot neura --real-robot --override 1.0 `
    --sequence sequences/pick_to_station.json --episodes 10 `
    --out data_anlage/2026-10-01/B07 --block B07 `
    --object "Teil A, ca. 30 Grad gedreht, links hinten" `
    --light "Decke an, Rollo zu" --camera-pose nominal
```

In jeder Episode abgelegt (AP 5.2): `block`, `session` (Datum), `light`,
`camera_pose`, `object_note`, **`object_pose`** (die geteachten Posen von
`PICK`/`PRE_GRASP`, änderbar mit `--object-points`), alle Punkte, Rauschfaktor
und Seed sowie die **Tempo-Werte** (`planner`: Transit-/Anfluggeschwindigkeit,
PTP-Gelenkgeschwindigkeit, Rampe, Greifer-Dwell), `servo_rate_hz` und `override`.
Mit `--real-robot` ist `--block` Pflicht.

## 2. Exportieren

```powershell
python apps/export.py --data "data_anlage/2026-10-01/*" --check      # nur prüfen
python apps/export.py --data "data_anlage/*/*" --out datasets/anlage_v1
```

Der Export **bricht ab**, wenn

* Schema-Version, Rate oder Kameras zwischen den Quellen abweichen,
* Override, `servo_j`-Rate, Robotertyp oder ein Planer-Tempo nicht in allen
  Episoden gleich sind — das Tempo wird mitgelernt (AP 2.6). Nur für bewusste
  Vergleiche: `--allow-mixed`,
* lerobot nicht die gepinnte Version 0.6.1 ist.

Verworfene Episoden und `success == False` fließen nie ein, unbewertete nur
ohne `--require-success` (sie werden gezählt). `meta/bc_export.json` hält
lerobot- und Schema-Version, alle Quellen und je Episode die Herkunft mit
sämtlichen Metadaten.

Dauer (gemessen, Streaming-Kodierung AV1): etwa 1–2 s je Episode mit zwei
Kameras. Der Ordner ist **eigenständig** — auf einen anderen Rechner kopieren
und dort trainieren.

## 3. Trainieren — auf beliebiger Hardware

```powershell
python apps/train.py --dataset datasets/anlage_v1 --out checkpoints/anlage_v1
```

### 3.1 Anderer Rechner (z. B. RTX 5090, Linux)

1. Environment wie in [Environment-Einrichtung.md](Environment-Einrichtung.md):
   Python 3.12, `torch==2.11.0+cu128`, `lerobot[dataset,diffusion]==0.6.1`
   (Liste: [environment-lock.txt](environment-lock.txt)). Die RTX 5090 ist
   ebenfalls Blackwell (sm_120) — derselbe cu128-Build passt.
   *Linux:* torchcodec braucht die FFmpeg-Bibliotheken des Systems
   (`apt install ffmpeg`); ohne sie fällt lerobot auf PyAV zurück (langsamer).
2. `python tools/check_gpu.py` — `sm_120` (bzw. die Architektur der Karte)
   muss in `get_arch_list()` stehen. `train.py` prüft das auch selbst.
3. Nur `bc/`, `apps/` und den Datensatzordner kopieren; Hardware-SDKs
   (neurapy, gxipy) braucht das Training nicht.
4. **Offline-Rechner:** die ImageNet-Gewichte von ResNet18 lädt torchvision
   beim ersten Training herunter (45 MB, nach
   `~/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth`). Die Datei vorher
   dorthin kopieren — oder `--no-pretrained` (schlechter, nicht empfohlen).

### 3.2 Stellschrauben, die nur die Hardware betreffen

| Parameter | Wirkung | Hinweis |
|---|---|---|
| `--device auto` | cuda → mps → cpu | `cuda:1` für die zweite Karte |
| `--amp auto` | bf16, wo verfügbar | `off` zur Fehlersuche |
| `--batch-size` / `--grad-accum` | effektive Batch = Produkt | für vergleichbare Läufe das **Produkt** gleich lassen (Default 64 × 1) |
| `--num-workers auto` | Windows ≤ 4, Linux ≤ 8 | Datenladen war nie der Engpass (0,02 s von 0,21 s) |
| `--compile` | torch.compile des U-Nets | nur Linux sinnvoll |
| `--max-hours` | Zeitbudget, dann regulär speichern | |
| `--resume` | am letzten Checkpoint fortsetzen | auch auf anderem Rechner |

Gemessen auf der RTX 5070 Ti (16 GB), zwei Kameras 240 × 320, 89,5 M Parameter,
bf16: Batch 32 → 0,13 s/Schritt, 3,6 GB; **Batch 64 → 0,21 s/Schritt, 5,6 GB**.
Der Default von 60 000 Schritten dauert dort rund 3,5 h. Eine 5090 schafft
Batch 64 schneller, am Ergebnis ändert sich bei gleicher effektiver Batch nichts.

**Windows-Stolperstein (gemessen 17.09.2026):** Zwei Trainings gleichzeitig
(je 4 Worker) neben der VirtualBox-VM brachen mit `Couldn't open shared file
mapping … error code 1455` bzw. einem CUDA-OOM trotz freier GPU ab. 1455 ist
das Commit-Limit (Auslagerungsdatei): jeder Worker lädt torch in einen eigenen
Prozess. Abhilfe: Läufe nacheinander, `--num-workers 2` oder die
Auslagerungsdatei vergrößern.

### 3.3 Modell (Defaults in `bc/config.py`, `POLICY_*`)

CNN-Diffusion-Policy aus lerobot: ResNet18 je Kamera (ImageNet), Spatial Softmax,
1D-U-Net `(256, 512, 1024)`, **2 Beobachtungen**, **Horizont 16**, davon
**8 nutzbar**, DDIM mit 100 Trainings- und **10 Inferenzschritten**, das Netz sagt
**die Aktionsfolge direkt** vorher (`prediction_type="sample"`, Begründung in 5.),
zufälliger
Crop 90 % im Training (Mitte bei der Inferenz), EMA 0,999 (gespeichert werden die
EMA-Gewichte), AdamW 1e-4 mit Cosine-Schedule. Ablation Wrist-only (AP 5.1):
`--cameras wrist`.

### 3.4 Ausgabe

* `train_log.csv` — Loss, LR, Gradientennorm, Zeiten, VRAM; alle `--eval-every`
  Schritte die Validierung auf zurückgehaltenen **Episoden**: Diffusion-Loss,
  **Gelenk-MAE der gesampelten Chunks in rad** (anschaulicher als der Loss) und
  Greifer-Trefferquote.
* `policy/` — für `apps/infer.py`: `model.safetensors`, `config.json`,
  Prä-/Postprozessor mit den Normierungsstatistiken und **`bc_policy.json`**:
  Schema, Rate, Kameras, lerobot-Version, Override/`servo_j`-Rate/Planer-Tempo der
  Aufzeichnung, Start- und Endstellung, Referenzbahn und Greifpunkte zur Bewertung,
  Trainingshardware und Metriken.

## 4. Inferenz

```powershell
python apps/infer.py --sim --checkpoint checkpoints/sim_durchstich --episodes 5
python apps/infer.py --robot neura --cameras sim --checkpoint checkpoints/vm_durchstich --episodes 3
python apps/infer.py --robot neura --real-robot --checkpoint checkpoints/anlage_v1
```

**Reset vor jeder Fahrt** (AP 5.2): Greifer auf den Startzustand der Aufzeichnung,
dann `move_to_joints` an die Startstellung. **Ende:** Endstellung ±0,02 rad **und**
Greiferzustand wie am Ende der Aufzeichnung, fünf Takte lang, frühestens nach der
halben typischen Episodenlänge; sonst nach 1,5 × der längsten Episode. Die
Stellung allein reicht nicht: In der Sim-Demo ist der Zwischenpunkt „transport"
identisch mit dem Endpunkt, nur der Greifer unterscheidet sie. Ohne diese Regel
wurde der Durchstich mitten in der Bahn als „fertig" gewertet.

**Erzwungen gleich wie bei der Aufzeichnung** (sonst Abbruch vor jeder Bewegung):
Schema, Rate 15 Hz, Bildgröße, Kameras, `servo_j`-Rate 60 Hz und am Neura der
**Override** (Default ist der Wert der Aufzeichnung).

Je Takt: Beobachtung wie im Recorder → Policy → Mitteln → Prüfen → 4 interpolierte
`servo_j`-Sollwerte → Greifer bei Wechsel.

* **Überlappende Chunks mitteln** (`bc/policy.py`, `ChunkEnsembler`): alle
  `--replan` Takte (Default 2) neu vorhersagen; für jeden Takt liegen bis zu
  4 Vorhersagen vor, ausgeführt wird ihr Mittel (`--decay` gewichtet nach Alter,
  Default gleich). `--no-ensemble` zum Vergleich.
* **Weicher Geschwindigkeitsfilter** (`servo.TargetLimiter`, nur Inferenz): Die
  Zieländerung pro Takt wird auf **0,8 rad/s** begrenzt, die Richtung bleibt
  erhalten. `gekappt` im Protokoll zählt die betroffenen Takte. Grund: Die Labels
  verlangen Korrekturen bis ~1 rad/s, und beim Eintreffen eines verspäteten Chunks
  springt das Mittel. Ohne diese Stufe hat der harte Filter eine sonst gute Fahrt
  bei 1,03 rad/s gestoppt.
* **Sprung- und Geschwindigkeitsfilter im Adapter** (`bc/servo.py`, `ServoGuard`,
  in `NeuraRobot` und `SimRobot`): jeder `servo_j`-Sollwert wird vor dem Senden
  geprüft — endlich, innerhalb der Achsgrenzen, höchstens **1,0 rad/s** gegenüber
  dem letzten Sollwert (Zeitbasis auf 1/60 … 1/15 s begrenzt, ein Stocken erlaubt
  also keinen großen Sprung). Die Schleife prüft zusätzlich je Takt, dass das Ziel
  höchstens **0,35 rad** von der gemessenen Stellung liegt. Verletzung ⇒ nichts
  senden, Software-Stopp, Fahrt beendet. Der Filter gilt auch für Aufzeichnung und
  Teachen. Grenzwerte (`config.SERVO_MAX_*`) sind aus den VM-Daten abgeleitet
  (Befehle max. 0,60 rad/s, Ziel–Ist max. 0,20 rad) und an der Anlage zu bestätigen.

Protokoll je Fahrt (`rollout_XXX.npz`) und `summary.json`: Takte, Abbruchgrund,
Greifpunkt- und Endfehler gegen die Referenz (nur bei **fester** Objektlage
aussagekräftig), Bahnabweichung, Vorhersagezeit (Median/p95), Chunk-Verzug in
Takten, Uneinigkeit der Chunks, Pacer-Overruns, gekappte Takte, Filter-Ablehnungen.
Als PDF:

```powershell
python tools/report_rollouts.py --run "VM=checkpoints/vm/policy/rollouts/<Zeit>" --out Berichte/<Datum>_Policy.pdf
```

## 5. Grenzen des Standes

* **Platzhalterbilder** (`--cameras sim`) haben keinen Bezug zur Szene. Die
  Policy lernt damit nur aus dem Zustand — der Durchstich prüft die Kette, nicht
  die spätere Qualität.
* **Vorhersagezeit** (RTX 5070 Ti, Windows): ~77 ms bei 10 DDIM-Schritten —
  länger als ein Takt. Ein U-Net-Durchlauf braucht 5,5 ms und ist durch den
  Kernel-Start-Overhead begrenzt, nicht durch die Rechenleistung. Deshalb läuft
  die Vorhersage bei echter Uhr **asynchron** in einem eigenen Thread
  (`policy.AsyncPredictor`): Der 60-Hz-Servostrom läuft ungestört weiter, jeder
  Chunk wird dem Takt seiner Beobachtung zugeordnet, bei Ankunft schon vergangene
  Einträge verfallen. Reicht kein Chunk mehr bis zum aktuellen Takt, stoppt die
  Fahrt (`vorhersage zu langsam`). Auf den Inferenz-Laptops messen (AP 4.1 a).
* **Wenige DDIM-Schritte brauchen `prediction_type="sample"`.** Befund
  Durchstich (epsilon-Vorhersage, 8000 Schritte): mit 10 Schritten gezackte
  Chunks, Sprung im Chunk 0,10–0,12 rad statt 0,015 wie im Label — der Filter
  hat die Fahrt schon in den ersten Takten gestoppt. Erst mit 100 Schritten war
  der Chunk glatt, dann aber 600 ms. Die Validierung protokolliert deshalb auch
  `val_chunk_jump_p95_rad` neben dem Label-Wert.
* **Keine Uhr im Bild:** Die frühere Sim-Kamera kodierte einen Bildzähler ins
  Bild. Die Policy las ihn als Zeit ab: Fahrt 0 gelang, danach fuhr sie direkt
  ans Bahnende. Platzhalter sind jetzt statisch (AP 2.6).
* Die Erfolgsbewertung in `summary.json` vergleicht mit der aufgezeichneten
  Greifposition. Bei variierter Objektlage zählt an der Anlage der reale Griff.
* **Offen, vor der echten Aufzeichnung (Datendesign): Stillstand an
  Zwischenpunkten.** VM-Durchstich: Die Policy folgt der Bahn bis `PRE_GRASP`
  (p95 5 mm) und bleibt dort in 5/5 Fahrten stehen. Das PTP-Segment endet im
  Stillstand, die LIN-Anfahrt beginnt aus dem Stillstand, und im Zustand ist
  „angekommen" nicht von „losfahren" zu unterscheiden (Deadlock aus AP 2.6).
  Echte Bilder lösen das an einem Wartepunkt nicht. Seltener neu vorhersagen
  (`--replan 4/8`) kommt darüber hinweg, war mit 30 Episoden aber instabil.
  Kandidaten: an Durchfahrpunkten nicht anhalten (Überschleifen/Rampen), längere
  Chunks offen ausführen, mehr Daten. Beim Greifer-Dwell besteht dieselbe
  Mehrdeutigkeit (Sim: 2/10 Fahrten schlossen nicht, Wartezeiten schwanken).

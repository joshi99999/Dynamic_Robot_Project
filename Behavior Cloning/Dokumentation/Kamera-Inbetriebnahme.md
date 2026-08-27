# Kameras — Inbetriebnahme, Hardware und Einstellungen

Praxisdokumentation zur Kamerakette der BC-Pipeline: was verbaut ist, wie
man es zum Laufen bringt, welche Einstellungen bewusst so gewählt sind und
was mechanisch gesichert werden muss.

Begründungen und Anforderungen stehen in
[../Requierments/requierments.md](../Requierments/requierments.md) (AP 1.1,
AP 1.4, AP 0.6). Dieses Dokument ist die Handlungsanleitung dazu.

---

## 1. Verbaute Hardware

### Wrist-Kamera (bestätigt, am Gerät verifiziert)

| | |
|---|---|
| Modell | Daheng Imaging **VEN-161-61U3C-M05** |
| Seriennummer | `EBK24100633` |
| Sensor | Sony IMX296, 1/2.9", **Global Shutter**, 1440 × 1080 |
| Pixelgröße | 3,45 µm |
| Pixelformat | BayerRG8 (Farbvariante — Voraussetzung für die RGB-Policy) |
| Native Rate | 61,3 fps |
| Schnittstelle | USB3.0, **USB3-Vision/GenICam** |
| Datenblatt | `../../20_Dokumentation/WristKamera-VEN-161-61U3MC-Datasheet.pdf` |

**Wichtig:** Die Kamera meldet sich **nicht** als UVC-Gerät.
`cv2.VideoCapture` kann sie nicht öffnen, unabhängig vom Index.

### Objektiv Wrist-Kamera (entschieden)

| | |
|---|---|
| Typ | **Fisheye** |
| Brennweite | 1,85 mm |
| Bildkreis | für 1/1.8" ausgelegt |
| Auflösungsreserve | 12 MP auf 1/1.8" (≈ 1,85 µm) |

Zur Wahl stand ein rektilineares Objektiv mit festem Fokus, das den
Arbeitsraum nicht durchgehend scharf abdeckt. **Entscheidung: Fisheye**, weil
Unschärfe unwiederbringlicher Informationsverlust ist, eine feste Verzeichnung
aber vom Netz mitgelernt wird. Ausführliche Begründung in AP 1.1.

Zwei Konsequenzen für die Praxis:

* **Das reale Sichtfeld ist deutlich enger als der Datenblattwert.** Die
  Angabe gilt für 1/1.8", der Sensor ist 1/2.9" — genutzt wird nur der
  zentrale Teil des Bildkreises (Diagonale ca. 70 %). Vorteil: weniger
  Randverzeichnung. Nachteil: das FOV **muss am Aufbau gemessen** werden,
  die Datenblattzahl ist unbrauchbar.
* **Beurteilung immer am 240 × 320-Bild**, nicht am Vollbild. Das ist die
  Auflösung, welche die Policy sieht; am Vollbild sieht alles gut aus.

### Szenen-/Top-View-Kamera (noch nicht entschieden)

Aktuell **nicht angeschlossen**. In Klärung: zweite Daheng VEN-161 mit dem
rektilinearen Objektiv statt einer UVC-Webcam. Bewertung in AP 1.1 —
kurz: gleiche Bildkette, feste Belichtung/Gain/Weißabgleich, Global
Shutter, Hardware-Trigger möglich; der konstante Arbeitsabstand der
Deckenkamera macht das Festfokus-Problem irrelevant.

> ⚠️ **`config.SCENE_CAMERA` ist bis dahin ein Platzhalter auf
> OpenCV-Index 0.** Auf einem Laptop ist das die **eingebaute Webcam**.
> Damit aufgezeichnete Daten wären wertlos, fallen im Datensatz aber nicht
> auf. `apps/record.py` fragt deshalb vor jeder Aufzeichnung nach, solange
> `config.SCENE_CAMERA_CONFIRMED = False` steht. Nach der Entscheidung:
> Konfiguration eintragen und das Flag setzen.

---

## 2. SDK-Installation (Windows)

1. **Daheng Galaxy SDK installieren** (Galaxy Viewer + USB3-Vision-Treiber).
   Installiert nach `C:\Program Files\Daheng Imaging\GalaxySDK`.
2. **Gegenprobe mit dem Galaxy Viewer, bevor Python ins Spiel kommt.**
   Zeigt der Viewer kein Bild, ist es ein Treiber- oder Kabelproblem und
   kein Code-Problem — das spart die meiste Suchzeit.
3. **Galaxy Viewer wieder schließen.** USB3-Vision-Kameras sind exklusiv
   belegt; solange der Viewer offen ist, bekommt Python kein Gerät. Das ist
   die häufigste Ursache für „im SDK geht's, in Python nicht".
4. **Kein `pip install gxipy`.** Siehe nächster Abschnitt.

### `gxipy` ist nicht installierbar — und muss es auch nicht sein

Daheng liefert die Python-API nur als Ordner mit, ohne `setup.py` und ohne
PyPI-Paket:

```
C:\Program Files\Daheng Imaging\GalaxySDK\Development\Samples\Python\gxipy
```

Das SDK-README empfiehlt, diesen Ordner neben das eigene Skript zu kopieren.
**Das machen wir bewusst nicht** — eine Kopie im Repo wäre Vendor-Code, der
bei SDK-Updates still veraltet und nicht zur installierten DLL-Version passt.

Stattdessen sucht `bc/adapters/cam_daheng.py` den Pfad zur Laufzeit
(`import_gxipy()`). Liegt das SDK an einem ungewöhnlichen Ort, kann der
Ordner, der `gxipy` enthält, über die Umgebungsvariable
`GALAXY_SDK_PYTHON` gesetzt werden.

### Stolperstein: vererbte Umgebungsvariablen

Der Installer setzt `GALAXY_GENICAM_ROOT`, `GENICAM_GENTL64_PATH` und die
DLL-Pfade nur auf **Machine**-Ebene. Windows vererbt diese ausschließlich
an **neu gestartete** Prozesse. Wer seine Shell offen hatte, als das SDK
installiert wurde, sieht:

```
KeyError: 'GALAXY_GENICAM_ROOT'
...
Cannot find GxIAPI.dll.
```

Das sieht nach einer kaputten Installation aus, ist aber reine
Umgebungsvererbung. **Einfachste Abhilfe: Shell neu starten.** Der Adapter
zieht die Werte zusätzlich selbst aus der Registry nach, damit es niemandem
mehr passiert.

### Verifikation

```bash
# 1. Wird die Kamera gefunden? (zeigt Modell + Seriennummer)
python tools/check_cameras.py --list

# 2. Liefert sie Bilder? (Live-Vorschau; q beendet, s speichert)
python tools/check_cameras.py --only wrist

# 3. Adapter-Abnahme: dieselben Contract-Tests wie in der Simulation
python tests/run_all.py --camera=daheng contract.test_camera_contract
```

Erwartetes Ergebnis (verifiziert): 5/5 Tests bestanden, Frame 1080 × 1440 × 3
uint8, Frame-Alter 7–13 ms bei 61 fps.

---

## 3. Kameraeinstellungen und warum sie so sind

Konfiguriert in [`bc/config.py`](../bc/config.py), angewandt in
[`bc/adapters/cam_daheng.py`](../bc/adapters/cam_daheng.py).

### Voller Sensor statt ROI — und warum das aktiv gesetzt werden muss

> ⚠️ **Der ROI wird in der Kamera persistent gespeichert.** Wird er nicht
> aktiv gesetzt, liefert die Kamera stumm den zuletzt konfigurierten
> Ausschnitt. Beobachtet: 640 × 480 aus einem 1440 × 1080-Sensor bei
> `OffsetX/Y = 0`, also ein **Eckausschnitt ohne optisches Zentrum** — beim
> Fisheye der Verlust des größten Teils des Sichtfelds, und zwar lautlos.

Bei GenICam sind `Width`/`Height` ein **Ausschnitt**, keine Skalierung. Der
Adapter setzt deshalb immer explizit `WidthMax`/`HeightMax` und Offsets auf 0.
Wird doch ein kleinerer ROI gebraucht, wird er **zentriert** gesetzt.

Die Skalierung auf die Schemagröße **240 × 320** passiert in Software
(`bc/recorder.py`). Seitenverhältnis 4:3 durchgehend.

### Binning: von diesem Modell nicht unterstützt

`BinningHorizontal`/`BinningVertical` sind bei der VEN-161 nicht schreibbar.
Die geplante Datenreduktion entfällt; es bleibt beim vollen Frame. Unkritisch:
1440 × 1080 Bayer8 bei 15 Hz ≈ **23 MB/s**.

### Bildrate

`fps = None` → native Rate (~61 fps). Frischere Frames und damit mehr Reserve
im 30-ms-Latenzbudget (AP 1.3), dafür mehr CPU-Last durchs Debayering.

Bei **zwei** Daheng an einem USB3-Controller auf ~30 fps begrenzen: zwei
Kameras mit 61 fps und vollem Sensor wären zusammen rund 190 MB/s. Nominell
machbar, aber knapp. Alternativ `DeviceLinkThroughputLimit` setzen; nach
Möglichkeit auf getrennte Controller verteilen.

### ⚠️ Offen: Auto-Regelungen festnageln

Beides verändert die Bildstatistik, auf die trainiert wird, und muss **vor
der ersten echten Datenaufzeichnung** fixiert werden:

* **Weißabgleich.** Die VEN-161 kann nur `Off` und `Once` (kein
  `Continuous`). `Once` hängt davon ab, was beim Programmstart im Bild war —
  die Farbstatistik schwankt damit **zwischen Sessions**. Feste Ratios
  messen und in `config.WRIST_CAMERA.white_balance_ratios` eintragen.
* **Gain.** Läuft aktuell automatisch (gemessen: 16,6 dB). Festen Wert
  ermitteln und in `config.WRIST_CAMERA.gain_db` setzen.
* **Belichtung** ist bereits fest (8000 µs).

### Geräteauswahl per Seriennummer

Sobald **zwei** Daheng angeschlossen sind, ist die Index-Reihenfolge nicht
stabil (Enumerationsreihenfolge, USB-Port, Einschaltzeitpunkt). Vertauschte
Wrist- und Szenenbilder würden im Training **nicht auffallen**. Der Adapter
verweigert daher den Start, wenn mehrere Geräte vorhanden sind und keine
Seriennummer konfiguriert ist.

Seriennummern ermitteln: `python tools/check_cameras.py --list`

---

## 4. Mechanik

### Objektiv fixieren

Die Verzeichnung ist Teil des Gelernten. Verdreht sich das Objektiv oder
ändert sich der Fokus, ist der Datensatz entwertet. Beim Fisheye wirkt sich
das stärker aus als bei rektilinearer Optik, weil die Verzeichnung zentriert
ist.

**Vorgesehen: Distanzring als reproduzierbarer Anschlag.** Richtige
Reihenfolge — der Ring kann nicht vorab dimensioniert werden:

1. Objektiv von Hand auf den Arbeitsabstand scharfstellen.
2. Entstandenen Spalt ausmessen.
3. Ring auf **genau dieses Maß** fertigen.
4. Objektiv bis Anschlag einschrauben.

Hintergrund: M12-Gewinde haben 0,5 mm Steigung, die sensorseitige
Schärfentiefe liegt im Bereich einiger 10 µm. Eine berechnete Dicke trifft
das nicht.

**Der Ring allein genügt nicht.** Ein auf einen Ring auflaufendes Gewinde
kann verkanten, und „bis Anschlag" hängt vom Drehmoment ab. Zusätzlich:

* **Konterring** (liegt M12-Objektiven bei),
* **Madenschraube** im Halter,
* **Körnerstrich/Lackmarkierung** über Objektiv und Halter — macht ein
  Verdrehen sofort sichtbar,
* bei verstellbarer Blende: **Blendenring ebenfalls sichern** (verändert
  Schärfentiefe *und* Helligkeit).

### Referenzbild — Veränderung erkennbar machen

Nach dem Fixieren ein Bild eines festen Testtargets aufnehmen und zum
Datensatz legen. Damit lässt sich später **prüfen**, ob Fokus oder
Verzeichnung sich verändert haben, statt es zu hoffen. Dasselbe Prinzip wie
die ArUco-Überwachung der Deckenkamera in AP 1.4: aus einem stillen Fehler
einen sichtbaren machen.

Ebenfalls einmalig aufnehmen: **Kalibrierdatensatz** (Schachbrett oder
ChArUco, einige Dutzend Bilder). Wird vermutlich nie gebraucht, ist aber
nachträglich nicht zu beschaffen — die Kamera ist dann längst umgebaut.

### Kamerapose am Greifer

**Entschieden: Greiferbacken sichtbar, aber nur als Anschnitt.**

Begründung: NeuraPy liefert **keine** Rückmeldung über den Greiferzustand
(nur binäres `grasp()`/`release()`, AP 2.1). Die Kamera ist damit der
**einzige Kanal**, über den überhaupt beobachtbar ist, ob die Backen
wirklich geschlossen sind — besonders während der 500 ms Totzeit, in der
der kommandierte Zustand im State-Vektor falsch ist. Zusätzlich ist Greifen
eine Relativaufgabe: Backen und Objekt im selben Bild machen den Fehler
direkt ablesbar.

Umsetzung:

* Backenspitzen am **unteren Bildrand**, grob 15–25 % der Bildhöhe.
* Kamera leicht **nach vorn/schräg** blickend, nicht senkrecht zwischen die
  Backen.
* **Prüfkriterium:** An der Greifpose muss das **Objekt** noch sichtbar
  sein, nicht nur die Backen. An den real geteachten Wegpunkten prüfen —
  am 240 × 320-Bild.

### Halterung und Kabel

* **Verstiften, nicht nur verschrauben.** Ein Verrutschen der Wrist-Kamera
  am Flansch hat denselben Effekt wie eine verschobene Deckenkamera
  (AP 1.4) — nur ohne Marker-Überwachung, die es aufdecken würde.
* **USB3-Kabel:** Zugentlastung und Serviceschleife einplanen. Prüfen, dass
  Achse 6 nicht eingeschränkt wird. Ein Kabel, das bei bestimmten Posen
  zieht, verstellt die Kamerapose über Wochen langsam — ein stiller
  Datensatz-Drift, den nichts meldet.
* **Masse und Tool-Geometrie:** Kamera am Flansch verändert Last und
  Schwerpunkt. Der Tool-Eintrag im Controller steht ohnehin noch aus
  (Vortest Punkt 0, `NoTool`) — Kamera und Greifer zusammen eintragen und
  den dq/dx-Test aus AP 2.4 danach **wiederholen**, weil sich der Hebelarm
  ändert.

---

## 5. Checkliste

### Erledigt

- [x] Galaxy SDK installiert, Bild über Galaxy Viewer
- [x] Python-Zugriff über `gxipy` (Pfadsuche im Adapter, Registry-Fallback)
- [x] Kamera per Seriennummer angesprochen (`EBK24100633`)
- [x] Voller Sensor 1440 × 1080, Offsets 0, Skalierung auf 240 × 320
- [x] Adapter-Contract-Tests gegen die reale Kamera: 5/5
- [x] Objektiv entschieden (Fisheye 1,85 mm)
- [x] Kamerapose entschieden (Backen als Anschnitt sichtbar)

### Offen

- [ ] **Szenenkamera:** Modell entscheiden (zweite Daheng vs. UVC), dann
      `config.SCENE_CAMERA` eintragen und `SCENE_CAMERA_CONFIRMED = True`
- [ ] **Weißabgleich-Ratios** messen und pinnen
- [ ] **Gain** messen und pinnen
- [ ] **Reales Sichtfeld** am Aufbau messen (Datenblattwert unbrauchbar)
- [ ] **Objektiv mechanisch fixieren** (Distanzring + Konterring +
      Madenschraube + Markierung)
- [ ] **Referenzbild** und **Kalibrierdatensatz** aufnehmen
- [ ] **Kamera montieren**, Backen-Anschnitt an den geteachten Wegpunkten
      prüfen (am 240 × 320-Bild)
- [ ] **Zeitversatz beider Kameras** gegen das 30-ms-Budget messen
      (`python tools/check_cameras.py`) — *bisher kein belastbarer Wert:
      die erste Messung lief gegen die Laptop-Webcam*
- [ ] **Bei zwei Daheng:** Hardware-Trigger-Synchronisation prüfen
      (siehe AP 1.1) und Rate/Bandbreite begrenzen

"""Auswahllisten fuer die Metadaten (AP 5.2, Wunsch Anwender 2026-10-26).

Freitext fuehrt zu "Licht aus, Fenster" in der einen und "Aus, Fensterlicht"
in der naechsten Aufnahme -- fuer die Auswertung nach Beleuchtung
(AP 5.1) zwei verschiedene Werte. Deshalb: feste Vorschlaege plus alles,
was schon einmal benutzt wurde, als Auswahl; eigene Eintraege bleiben
moeglich und werden gemerkt. Schreibt man einen Wert, der sich nur in
Gross-/Kleinschreibung oder Leerzeichen von einem bekannten unterscheidet,
wird die bekannte Schreibweise genommen.

Die Datei liegt in ``einstellungen/`` und ist eingecheckt: dieselben
Begriffe auf allen Rechnern der Gruppe. NICHT in ``sequences/`` -- dort hielt
die Oberflaeche sie fuer eine Ablaufdatei und waehlte sie als erste aus
(2026-10-26: Punkte-Liste leer). Qt-frei.
"""

import json
import re
from pathlib import Path

#: Ablage der Auswahl, relativ zum Arbeitsordner "Behavior Cloning".
VOCABULARY_FILE = "einstellungen/metadaten_auswahl.json"

#: Felder mit Auswahlliste (Schluessel = Name in record_argv).
FIELDS = ("light", "camera_pose", "object_note")

#: Vorschlaege, solange die Datei sie nicht ueberschreibt.
DEFAULTS = {
    "light": ["Decke an", "Decke aus", "Decke an + Fensterlicht",
              "Lampe weiß", "Lampe warmweiß", "Lampe rot", "Lampe grün", "Lampe blau"],
    "camera_pose": ["nominal", "+2 cm x", "-2 cm x", "+2 cm y", "-2 cm y", "+5° gedreht"],
    "object_note": [],
}


def normalize(text):
    """Leerzeichen vereinheitlichen; leer -> None."""
    text = re.sub(r"\s+", " ", str(text or "")).strip()
    return text or None


def canonical(text, known):
    """Bekannte Schreibweise, wenn sich ``text`` nur in Gross-/Kleinschreibung
    oder Leerzeichen unterscheidet -- sonst ``text`` selbst (normalisiert)."""
    text = normalize(text)
    if text is None:
        return None
    for entry in known:
        if normalize(entry).casefold() == text.casefold():
            return entry
    return text


def load(workdir, path=VOCABULARY_FILE):
    """``{Feld: [Eintraege]}`` -- Vorschlaege plus Gemerktes, ohne Dubletten."""
    data = {}
    file = Path(workdir) / path
    if file.is_file():
        try:
            data = json.loads(file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            data = {}
    out = {}
    for field in FIELDS:
        entries = []
        for entry in list(DEFAULTS.get(field, [])) + list(data.get(field) or []):
            entry = normalize(entry)
            if entry and canonical(entry, entries) not in entries:
                entries.append(entry)
        out[field] = entries
    return out


def remember(workdir, values, path=VOCABULARY_FILE):
    """Neue Werte merken (beim Start einer Aufnahme). Liefert die Werte in
    bekannter Schreibweise zurueck -- die gehen dann in die Kommandozeile."""
    vocabulary = load(workdir, path)
    result, changed = {}, False
    for field, value in values.items():
        if field not in FIELDS:
            result[field] = value
            continue
        value = canonical(value, vocabulary[field])
        result[field] = value
        if value and value not in vocabulary[field]:
            vocabulary[field].append(value)
            changed = True
    if changed:
        file = Path(workdir) / path
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(json.dumps(vocabulary, indent=2, ensure_ascii=False) + "\n",
                        encoding="utf-8")
    return result

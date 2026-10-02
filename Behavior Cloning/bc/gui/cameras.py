"""Kamerazuordnung fuer die Oberflaeche -- Qt-frei (AP 1.1 / 1.2).

Die Oberflaeche fragt nicht "welcher Kameramodus?", sondern "was haengt auf
Platz *wrist*, was auf Platz *scene*?". Das ist die Sicht, die am Labortag
gebraucht wird, und sie haelt die Zusage aus AP 0.3 ein: Ein Kamerawechsel
ist ein Zuordnungswechsel, keine Code-Aenderung. Solange es einen Adapter
gibt, laesst sich jedes Geraet auf jeden Platz legen.

Der Platzhalter ("Simulation") steht ueberall zur Auswahl, auch wenn
Hardware da ist -- zum Testen der Kette ohne Kamera ist das der kuerzeste
Weg, und das Datensatz-Schema bleibt identisch (AP 0.5). An der realen
Anlage ergibt er keinen Sinn; :func:`assignment_warnings` sagt das, statt
die Auswahl zu verbieten (Festlegung Anwender, 2026-09-29: benennen statt
verstecken).

Die Suche nach Geraeten oeffnet UVC-Indizes probeweise und dauert -- die
Oberflaeche ruft :func:`discover` deshalb in einem Arbeitsthread auf und
haelt das Ergebnis in :class:`Inventory`.
"""

from .. import config
from ..adapters import (
    CAMERA_BACKENDS,
    camera_config_for,
    list_devices,
    parse_camera_spec,
    sim_device,
)

#: Reihenfolge der Plaetze -- dieselbe wie im Datensatz.
SLOTS = tuple(c.name for c in config.CAMERAS)

SLOT_LABELS = {"wrist": "Wrist-Kamera", "scene": "Szenenkamera"}

#: Was jeder Platz ist, in einem Satz. Steht als Hinweis am Auswahlfeld.
SLOT_HINTS = {
    "wrist": "Am Greifer montiert. Trägt den Hauptteil der Information, "
             "aus der die Policy lernt.",
    "scene": "Top-View auf den Arbeitsbereich. Modell noch nicht entschieden "
             "(AP 1.1) — bis dahin Platzhalter oder Webcam.",
}


class Inventory(object):
    """Was gefunden wurde, plus die Begruendung, warum etwas fehlt.

    Die Unterscheidung "kein Geraet angeschlossen" und "kein SDK
    installiert" ist beim Einrichten der entscheidende Unterschied --
    deshalb werden die Hinweise mitgefuehrt statt weggeworfen.
    """

    def __init__(self, devices=None, notes=None):
        self.devices = list(devices or [sim_device()])
        self.notes = dict(notes or {})

    def for_slot(self, slot):
        """Auswahlliste eines Platzes.

        Alles, was gefunden wurde, steht jedem Platz zur Verfuegung -- welche
        Kamera wo sitzt, entscheidet der Aufbau und nicht die Software.
        """
        return list(self.devices)

    def find(self, key):
        for device in self.devices:
            if device.key == key:
                return device
        return None

    def backends_present(self):
        return set(d.backend for d in self.devices)

    def note_lines(self):
        """Hinweise als Zeilen -- was fehlt und warum."""
        lines = []
        for backend in ("uvc", "daheng"):
            note = self.notes.get(backend)
            if note:
                lines.append("%s: %s" % (backend, note.splitlines()[0]))
        return lines


def discover(uvc=True, daheng=True, cancel=None):
    """Geraete suchen. Dauert (UVC-Indizes werden geoeffnet) -- Arbeitsthread.

    ``cancel`` (threading.Event) bricht zwischen zwei Geraeten ab und wirft
    dann ``bc.adapters.Cancelled``.
    """
    found = list_devices(uvc=uvc, daheng=daheng, cancel=cancel)
    return Inventory(found["devices"], found["notes"])


def default_assignment(inventory, backend_is_sim_robot=True):
    """Vorbelegung beim Oeffnen des Reiters.

    Ohne Controller (SimRobot) ueberall der Platzhalter -- das ist der Fall,
    in dem man die Kette durchspielt. Mit Controller wird genommen, was zur
    hinterlegten Konfiguration passt, und sonst wieder der Platzhalter: eine
    Vorbelegung, die stillschweigend auf OpenCV-Index 0 zeigt, waere auf
    einem Laptop die eingebaute Webcam (config.SCENE_CAMERA_CONFIRMED).
    """
    assignment = {}
    for slot in SLOTS:
        chosen = None
        if not backend_is_sim_robot:
            configured = {c.name: c for c in config.CAMERAS}[slot]
            for device in inventory.devices:
                if device.backend != configured.backend or not device.available:
                    continue
                if configured.device in (None, "") or str(device.device) == str(configured.device):
                    chosen = device
                    break
        assignment[slot] = (chosen or sim_device()).key
    return assignment


def to_specs(assignment):
    """Zuordnung -> ``["wrist=sim", "scene=uvc:1"]`` fuer die Kommandozeile.

    Immer BEIDE Plaetze ausschreiben, auch wenn einer dem Modus entspricht:
    Eine Kommandozeile, die im Bericht landet, soll ohne Kenntnis der
    Vorgaben lesbar sein.
    """
    return ["%s=%s" % (slot, assignment[slot]) for slot in SLOTS if slot in assignment]


def to_configs(assignment):
    """Zuordnung -> CameraConfig je Platz, in der Reihenfolge des Datensatzes."""
    configs = []
    for slot in SLOTS:
        if slot not in assignment:
            continue
        name, backend, device = parse_camera_spec("%s=%s" % (slot, assignment[slot]))
        configs.append(camera_config_for(name, backend, device))
    return configs


def uses_backend(assignment, backend):
    """Ist ``backend`` in dieser Zuordnung ueberhaupt im Spiel?

    Damit haengt die Voraussetzungspruefung an der Zuordnung statt am Modus:
    Das Galaxy SDK wird nur verlangt, wenn tatsaechlich eine Daheng zugewiesen
    ist (Festlegung Anwender, 2026-09-29).
    """
    return any(str(key).split(":")[0] == backend for key in assignment.values())


def all_simulated(assignment):
    return bool(assignment) and all(
        str(key).split(":")[0] == "sim" for key in assignment.values())


def required_backend_keys(assignment):
    """Voraussetzungs-Schluessel, die diese Zuordnung wirklich braucht.

    Nicht mehr und nicht weniger: Platzhalter brauchen gar nichts, eine
    Webcam OpenCV, eine Daheng zusaetzlich das Galaxy SDK.
    """
    keys = []
    if uses_backend(assignment, "uvc") or uses_backend(assignment, "daheng"):
        keys.append("opencv")
    if uses_backend(assignment, "daheng"):
        keys.append("gxipy")
    return keys


def assignment_warnings(assignment, inventory=None, is_real_plant=False):
    """Was an dieser Zuordnung auffaellt -- benannt, nicht verboten.

    Reihenfolge nach Tragweite: Was einen Datensatz unbrauchbar macht,
    steht oben.
    """
    warnings = []
    if not assignment:
        return warnings

    if is_real_plant:
        simulated = [SLOT_LABELS.get(s, s) for s in SLOTS
                     if str(assignment.get(s, "")).startswith("sim")]
        if simulated:
            warnings.append(
                "An der realen Anlage mit Platzhalterbildern: %s. Solche "
                "Aufnahmen sind Testdaten, keine Trainingsdaten."
                % ", ".join(simulated))
        if str(assignment.get("wrist", "")).startswith("uvc"):
            warnings.append(
                "Webcam als Wrist-Kamera — apps/record.py verweigert das an "
                "der Anlage (nur Test der Kamerakette).")

    for slot in SLOTS:
        key = assignment.get(slot)
        if key is None:
            continue
        backend = str(key).split(":")[0]
        if backend == "uvc" and slot == "scene" and not config.SCENE_CAMERA_CONFIRMED:
            warnings.append(
                "Szenenkamera ist noch nicht entschieden (AP 1.1). OpenCV-Index 0 "
                "ist auf einem Laptop die eingebaute Kamera — Index prüfen.")
        if backend == "daheng" and ":" not in str(key):
            warnings.append(
                "%s ohne Seriennummer: bei mehreren Daheng-Geräten ist die "
                "Zuordnung nicht eindeutig." % SLOT_LABELS.get(slot, slot))
        if inventory is not None and inventory.find(str(key)) is None:
            warnings.append(
                "%s: %s wurde bei der letzten Suche nicht gefunden."
                % (SLOT_LABELS.get(slot, slot), key))
        elif inventory is not None and not inventory.find(str(key)).available:
            warnings.append(
                "%s: %s liefert kein Bild."
                % (SLOT_LABELS.get(slot, slot), key))

    mixed = set(str(k).split(":")[0] for k in assignment.values())
    if len(mixed) > 1 and "sim" in mixed:
        warnings.append(
            "Platzhalter und echte Kamera gemischt — der Export bricht bei "
            "gemischten Backends ab (bc/lerobot_io.py).")
    return warnings


def describe(assignment):
    """Eine Zeile je Platz fuer die Anzeige."""
    lines = []
    for slot in SLOTS:
        key = assignment.get(slot)
        lines.append("%s: %s" % (SLOT_LABELS.get(slot, slot), key or "—"))
    return "  ·  ".join(lines)


__all__ = ["SLOTS", "SLOT_LABELS", "SLOT_HINTS", "CAMERA_BACKENDS", "Inventory",
           "discover", "default_assignment", "to_specs", "to_configs",
           "uses_backend", "all_simulated", "required_backend_keys",
           "assignment_warnings", "describe"]

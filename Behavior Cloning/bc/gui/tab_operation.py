"""Modus "Betrieb": Modell laden und Inferenz aktivieren -- Geruest.

Uebergabe an die andere Gruppe: Behavior Cloning fuehrt das Greifen selbst
aus, bis PRE_PLACE, und gibt dort zurueck (Anwender, 2026-09-29). Die
vorhandene Ablaufdatei sequences/pick_to_station.json endet bereits genau
dort. Ist die Inferenz nicht aktiv, bekommt die andere Seite eine Meldung
statt stillschweigend nichts.

Offen, weil mit der anderen Gruppe zu besprechen: deren M4-Slot erwartet
``ObjectDetector.detect() -> ObjectPose``, also eine Greifpose als
Rueckgabewert. Hier geht es aber um die Uebergabe der Roboterhoheit, nicht
um einen Rueckgabewert -- dafuer braucht es eine zweite Schnittstelle.
"""

from .tab_base import PlaceholderTab


class OperationTab(PlaceholderTab):
    mode = "operation"
    title = "Betrieb"

    planned = (
        "Modell (Checkpoint) wählen, Metadaten zeigen: Schema, Rate, Override, "
        "Datensatz, auf dem es trainiert wurde.",
        "Inferenz aktivieren und deaktivieren — mit einem Schalter, der sagt, "
        "was gerade gilt.",
        "Laufende Übersicht: Takt, Vorhersagedauer, Folgefehler, Greiferzustand, "
        "Pacer-Überläufe.",
        "Übergabe an die andere Gruppe bei PRE_PLACE; ist die Inferenz aus, geht "
        "eine Meldung zurück statt stillschweigend nichts.",
        "Abbruch jederzeit — Software-Stopp, kein Ersatz für den Not-Aus.",
    )

    today = (
        "python apps/infer.py --sim --checkpoint checkpoints/sim_durchstich --episodes 3",
        "python apps/infer.py --sim --hold --steps 45",
        "python apps/eval.py",
    )

"""Modus "Training": Datensatz exportieren und Policy trainieren -- Geruest.

Auf dem Aufnahme-Laptop fehlt CUDA. Der Reiter bleibt trotzdem da und sagt,
was fehlt und was ohne GPU geht (Export ja, Training praktisch nein) --
Festlegung Anwender, 2026-09-29.
"""

from .tab_base import PlaceholderTab


class TrainingTab(PlaceholderTab):
    mode = "training"
    title = "Training"

    planned = (
        "Datensatz wählen, Aufnahmen zu einem LeRobotDataset exportieren, "
        "Konsistenz prüfen (Schema, Rate, Override).",
        "Hyperparameter einstellen — Vorgaben aus bc/config.py als Ausgangswert, "
        "Abweichungen sichtbar machen.",
        "Hardware anzeigen und wählen: verfügbare CUDA-Geräte, sonst CPU mit "
        "deutlichem Hinweis.",
        "Datensatz und Modell importieren und exportieren (anderer "
        "Trainingsrechner, offline).",
        "Trainingslauf mit Fortschritt, Validierungsfehler in rad und "
        "abbrechbar; Checkpoints ablegen.",
    )

    today = (
        "python apps/export.py --data \"data_vm/2026-09-17/*\" --out datasets/vm",
        "python apps/train.py --dataset datasets/vm --out checkpoints/vm",
        "python tools/check_gpu.py",
    )

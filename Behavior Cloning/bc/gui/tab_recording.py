"""Modus "Aufnahme": Punkte teachen und Episoden aufzeichnen -- Geruest.

Geplanter Aufbau (Anwender, 2026-09-29): in der Mitte gross die
Wrist-Kamera, rechts kleiner die Szenenkamera, darunter Bildrate, Zustaende,
Takt und Status; links die Schaltflaechen. Aufnahme muss abbrechbar sein und
am Ende verworfen werden koennen.

Der passive Modus (Aufzeichnung laeuft mit, waehrend das klassische Team
greift) braucht zuerst die Abschnittserkennung -- es sollen nur die
richtigen Abschnitte in den Datensatz, nicht alles. Dieselbe
Abschnittsgrenze ist die Uebergabe an die andere Gruppe bei PRE_PLACE.
"""

from .tab_base import PlaceholderTab


class RecordingTab(PlaceholderTab):
    mode = "recording"
    title = "Aufnahme"

    planned = (
        "Wrist-Kamera gross, Szenenkamera kleiner daneben, darunter Bildrate, "
        "Gelenk- und TCP-Zustand, Takt und Sync-Budget.",
        "Punkte über das Touchpad des Neura einteachen und in die Ablaufdatei "
        "übernehmen (sequences/pick_to_station.json).",
        "Aufnahme starten: der Roboter fährt die Bahn ab und zeichnet dabei auf. "
        "Stoppen jederzeit möglich.",
        "Am Ende einer Episode die Rückfrage speichern oder verwerfen.",
        "Passiver Modus: Aufzeichnung läuft im Hintergrund, während das "
        "klassische Team greift — offen, bis die Abschnittserkennung steht.",
        "Übergabe an die andere Gruppe bei PRE_PLACE.",
    )

    today = (
        "python apps/teach.py",
        "python apps/record.py --sim --episodes 3 --out data_sim",
        "python apps/record.py --robot neura --cameras sim --override 1.0 "
        "--sequence sequences/pick_to_station.json --episodes 2 --out data_vm",
    )

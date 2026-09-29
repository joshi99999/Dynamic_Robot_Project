"""Bedienoberflaeche fuer den Behavior-Cloning-Teil (PySide6).

Vier Modi, benannt am 2026-09-29:

    Systemcheck   Unittests und Geraetepruefungen, gestaffelt nach
                  Eingriffstiefe (hardwarefrei / lesend / mit Bewegung)
    Aufnahme      Teachen und Aufzeichnen von Episoden
    Training      Datensatz exportieren, Policy trainieren
    Betrieb       Modell laden, Inferenz aktivieren

Zwei Ebenen, wie mit dem Anwender festgelegt:

1. **Eigenstaendig** -- ``python -m bc.gui``. Das ist der Fall, der am
   Labortag gebraucht wird, und laeuft genauso gegen die Simulation und die
   virtuelle Steuerung.
2. **Als Reiter in der GUI der anderen Gruppe** -- ``bc.gui.plugin``. Die
   Einbindung haengt sich von aussen an deren Fenster; deren Code muss
   dafuer nicht angefasst werden (siehe plugin.py).

Schichtung (dieselbe Regel wie im Rest der Pipeline): Dieses Paket
importiert weder ``neurapy`` noch ``gxipy`` direkt, sondern geht ueber
``bc.adapters``. Qt-frei und damit ohne Bildschirm testbar sind
``session``, ``requirements`` und ``checks``; alles mit Qt liegt in
``app``, ``widgets``, ``runner`` und den ``tab_*``-Modulen.

Sprache: Oberflaechentexte deutsch mit Umlauten (sie stehen am Labortag auf
dem Bildschirm), Bezeichner englisch, Docstrings deutsch in ASCII wie im
uebrigen Bestand.
"""

#: Reihenfolge der Reiter. Neue Modi hier und in requirements.BY_MODE eintragen.
MODES = ("systemcheck", "recording", "training", "operation")

MODE_LABELS = {
    "systemcheck": "Systemcheck",
    "recording": "Aufnahme",
    "training": "Training",
    "operation": "Betrieb",
}

__all__ = ["MODES", "MODE_LABELS", "checks", "requirements", "session"]


def __getattr__(name):
    """Qt-Teile erst bei Bedarf laden.

    So bleibt ``import bc.gui`` ohne installiertes PySide6 moeglich -- die
    Qt-freien Module (session, requirements, checks) sind damit auch auf
    einem Rechner ohne Qt nutzbar und testbar.
    """
    if name in ("main", "MainWindow"):
        from . import app
        return getattr(app, name)
    if name in ("attach_to", "detach"):
        from . import plugin
        return getattr(plugin, name)
    raise AttributeError("module %r has no attribute %r" % (__name__, name))

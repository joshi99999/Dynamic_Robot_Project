"""Was der Modus "Aufnahme" ausfuehrt und wie seine Ausgabe gelesen wird -- Qt-frei.

Wie in den anderen Modi laeuft ``apps/record.py`` als UNTERPROZESS: Ein
Abbruch ist damit ein Prozessende und keine Hoffnung auf ein kooperatives
Flag, und die Kommandozeile bleibt die aus der Dokumentation.

Daraus folgt die Sache mit dem LIVEBILD. Eine Kamera laesst sich nicht
zweimal oeffnen -- waehrend der Aufnahme gehoeren die Kameras dem
Unterprozess. Deshalb zwei getrennte Zustaende (Festlegung Anwender,
2026-09-29):

    Vorschau    Die Oberflaeche oeffnet die Kameras SELBST, zeigt Bild,
                Rate und Zeitversatz. Das ist der Zustand zum Einrichten:
                Kameras zuordnen, ausrichten, Budget pruefen.
    Aufnahme    Die Vorschau wird geschlossen, apps/record.py uebernimmt.
                Angezeigt werden Takt, Episode, Versatz und Befunde.

Auf Wunsch kann die Aufnahme zusaetzlich ein Livebild schreiben
(``--preview``, bc/preview.py) -- standardmaessig AUS, weil es Zeit in der
15-Hz-Schleife kostet und die Datenqualitaet vorgeht.
"""

import json
import re
from datetime import date
from pathlib import Path

from .. import config
from .training import relative

SEQUENCE_ROOT = "sequences"

#: Wohin aufgezeichnet wird, je Roboter. data_vm/<Datum> ist die Ablage,
#: auf die sich die Berichte beziehen (Festlegung 2026-09-16).
DATA_ROOTS = {"sim": "data_sim", "neura": "data_vm"}


# -- Ablaufdateien ---------------------------------------------------------

def find_sequences(workdir, root=SEQUENCE_ROOT):
    """Ablaufdateien in ``sequences/`` -- nur, was wirklich ein Ablauf ist.

    Dort liegen auch Punkt-Sicherungen (``points_*.json``); als Ablauf
    gewaehlt ergaben sie eine leere Punkte-Liste.
    """
    base = Path(workdir) / root
    if not base.is_dir():
        return []
    found = []
    for path in sorted(base.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if isinstance(data, dict) and isinstance(data.get("sequence"), list):
            found.append(relative(path, workdir))
    return found


def describe_sequence(path):
    """Punkte und Ueberschleifradien einer Ablaufdatei -- ohne den Controller.

    Die Koordinaten stehen NICHT in der Datei; sie werden je Episode frisch
    aus der Punkte-Datenbank des Controllers geholt (bc/sequence.py), damit
    ein Touch-up sofort wirkt. Hier steht deshalb nur, welche Punkte
    gebraucht werden -- das genuegt, um vor dem Labortag zu sehen, was
    geteacht sein muss.
    """
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception as exc:
        return {"path": Path(path), "error": "%s: %s" % (type(exc).__name__, exc)}
    steps = data.get("sequence") or []
    points = []
    blends = []
    optional = []
    gripper = []
    computed = []
    for step in steps:
        if not isinstance(step, dict):
            continue
        name = str(step.get("point") or step.get("name") or "?")
        points.append(name)
        if step.get("blend"):
            blends.append("%s %.0f mm" % (name, 1e3 * float(step["blend"])))
        if step.get("optional"):
            optional.append(name)
        if step.get("gripper"):
            gripper.append("%s: %s" % (name, step["gripper"]))
        if step.get("above") and step.get("dz"):
            computed.append("%s = %s + %.0f mm" % (name, step["above"], 1e3 * float(step["dz"])))
    return {
        "path": Path(path),
        "name": data.get("name") or Path(path).stem,
        "points": points,
        "blends": blends,
        "optional": optional,
        "gripper": gripper,
        "computed": computed,
        "gripper_start": data.get("gripper_start"),
        "error": None,
    }


def sequence_summary(described):
    if described.get("error"):
        return "unlesbar (%s)" % described["error"]
    bits = ["%d Punkte: %s" % (len(described["points"]),
                               " → ".join(described["points"]))]
    if described["optional"]:
        bits.append("optional (wird übersprungen, wenn nicht geteacht): "
                    + ", ".join(described["optional"]))
    if described["blends"]:
        bits.append("überschliffen: " + ", ".join(described["blends"]))
    if described["gripper"]:
        bits.append("Greifer — Start %s, %s"
                    % (described["gripper_start"] or "?",
                       ", ".join(described["gripper"])))
    if described.get("computed"):
        bits.append("berechnet (nicht teachen): " + ", ".join(described["computed"]))
    bits.append("Koordinaten kommen beim Start aus der Steuerung, geplant wird "
                "einmal für alle Episoden — ein Touch-up während des Laufs "
                "beendet ihn.")
    return "\n".join(bits)


def suggest_out(workdir, robot_kind="sim", label=None, roots=None):
    """Zielordner nach der vereinbarten Ablage: ``data_vm/<Datum>/<Lauf>``."""
    roots = roots or DATA_ROOTS
    root = roots.get(robot_kind, "data_vm")
    folder = Path(workdir) / root / date.today().isoformat()
    if label:
        folder = folder / label
    return relative(folder, workdir)


# -- Kommandozeile ---------------------------------------------------------

def record_argv(out, robot_kind="sim", episodes=1, sequence=None,
                camera_specs=None, override=None, noise_scale=None,
                noise_fixed=False, seed=None, block=None, session=None,
                light=None, camera_pose=None, object_note=None,
                ask_label=False, preview=None, preview_hz=None,
                real_robot=False, servo_rate=None, transit_speed=None,
                approach_speed=None, ptp_speed=None, no_gripper=False,
                execution=None, put_back=True, probe=True, teach_after=False,
                lagen=1, drehung=None, stop_file=None, regrip=None, margin=None):
    """Kommandozeile fuer apps/record.py (ohne das Python-Programm)."""
    if not out:
        raise ValueError("Kein Zielordner angegeben.")
    argv = ["apps/record.py", "--robot", robot_kind]
    if sequence:
        argv += ["--sequence", str(sequence)]
    for spec in camera_specs or []:
        argv += ["--camera", spec]
    if override is not None:
        argv += ["--override", "%g" % float(override)]
    if noise_scale is not None:
        argv += ["--noise-scale", "%g" % float(noise_scale)]
    if noise_fixed:
        argv.append("--noise-fixed")
    # Nur anhaengen, wenn abweichend: der Normalfall bleibt eine kurze,
    # lesbare Kommandozeile, und der Default steht an einer Stelle.
    if servo_rate is not None and float(servo_rate) != config.SERVO_RATE_HZ:
        argv += ["--servo-rate", "%g" % float(servo_rate)]
    for flag, value, default in (
            ("--transit-speed", transit_speed, config.TRANSIT_SPEED_MS),
            ("--approach-speed", approach_speed, config.APPROACH_SPEED_MS),
            ("--ptp-speed", ptp_speed, config.PTP_JOINT_SPEED_RADS)):
        if value is not None and abs(float(value) - default) > 1e-9:
            argv += [flag, "%g" % float(value)]
    # Ausfuehrungsart (bc/execution.py): nur wenn abweichend vom Standard.
    if execution and execution != config.EXECUTION_MODE:
        argv += ["--ausfuehrung", str(execution)]
    argv += ["--episodes", str(int(episodes)), "--out", str(out)]
    # Zuruecklegen ist in record.py Standard (mit Ablaufdatei) -- nur das
    # Abschalten gehoert in die Kommandozeile.
    if sequence and not put_back:
        argv.append("--ohne-zuruecklegen")
    # Probefahrt ebenso Standard (record.py, bc/probe.py) -- gegen den
    # SimRobot laesst record.py sie von selbst aus.
    if not probe:
        argv.append("--ohne-probefahrt")
    # Nach dem Lauf in Teach (Wunsch Anwender 2026-10-26) -- nur an einer
    # Steuerung; der SimRobot kennt keinen Teach-Modus.
    if teach_after and robot_kind == "neura":
        argv.append("--nach-ende-teach")
    if sequence and put_back and regrip and regrip != config.PUT_BACK_REGRIP_DEFAULT:
        argv += ["--nachgreifen", str(regrip)]
    # Automatische Lagen (bc/auto_layout.py) -- nur mit Zuruecklegen
    if sequence and put_back and int(lagen) > 1:
        argv += ["--lagen", str(int(lagen))]
        if drehung is not None and abs(float(drehung) - config.AUTO_YAW_RANGE_DEG) > 1e-9:
            argv += ["--drehung", "%g" % float(drehung)]
        if margin is not None and abs(float(margin) - config.AUTO_EDGE_MARGIN_M) > 1e-9:
            argv += ["--randabstand", "%g" % float(margin)]
        if stop_file:
            argv += ["--stopp-datei", str(stop_file)]
    if seed is not None:
        argv += ["--seed", str(int(seed))]
    # Metadaten AP 5.2 -- an der Anlage ist --block Pflicht.
    for flag, value in (("--block", block), ("--session", session),
                        ("--light", light), ("--camera-pose", camera_pose),
                        ("--object", object_note)):
        if value:
            argv += [flag, str(value)]
    if ask_label:
        argv.append("--ask-label")
    if preview:
        argv += ["--preview", str(preview)]
        if preview_hz:
            argv += ["--preview-hz", "%g" % float(preview_hz)]
    if real_robot:
        argv.append("--real-robot")
    if no_gripper:
        argv.append("--no-gripper")
    return argv


#: "Naechster freier Block: B05" -- record.py am Ende (automatische Lagen
#: verbrauchen mehrere Bloecke)
_NEXT_BLOCK = re.compile(r"^Naechster freier Block:\s*(?P<block>.+?)\s*$")


def estimate_duration_s(episodes, lagen=1, probe=True):
    """Grobe Dauer eines Laufs in s (config.ESTIMATE_*, gemessen 26.10.)."""
    lagen = max(1, int(lagen))
    total = config.ESTIMATE_START_S + lagen * config.ESTIMATE_PLAN_LAGE_S
    total += int(episodes) * lagen * config.ESTIMATE_CYCLE_S
    total += (lagen - 1) * config.ESTIMATE_TRANSFER_S
    if probe:
        total += config.ESTIMATE_PROBE_S
    return total


def format_duration(seconds):
    """"8 min", "1 h 25 min"."""
    minutes = int(round(seconds / 60.0))
    if minutes < 1:
        return "%d s" % int(round(seconds))
    if minutes < 60:
        return "%d min" % minutes
    return "%d h %02d min" % (minutes // 60, minutes % 60)


def duration_table(episodes, probe=True, counts=(1, 10, 50, 100, 150)):
    """Zeilen "10 Lagen: ca. 15 min" fuer den Tooltip des Feldes "Lagen"."""
    return ["%d Lage%s: ca. %s" % (n, "" if n == 1 else "n",
                                   format_duration(estimate_duration_s(episodes, n, probe)))
            for n in counts]


#: "Geschaetzte Dauer: ca. 8 min (...)" -- record.py nach der Planung
_DURATION = re.compile(r"^Geschaetzte Dauer:\s*(?P<text>.+)$")


def duration_estimate(line):
    """Text aus der Dauer-Zeile von record.py, sonst None."""
    match = _DURATION.match((line or "").strip())
    return match.group("text") if match else None


def next_free_block(line):
    """Block aus der Schlusszeile von record.py, sonst None."""
    match = _NEXT_BLOCK.match((line or "").strip())
    return match.group("block") if match else None


def next_block(text):
    """Naechste Block-ID: Zahl am Ende hochzaehlen, Stellen bleiben.

    "B07" -> "B08", "Lage 9" -> "Lage 10", "Erste Daten" -> "Erste Daten 2",
    leer -> "B01". Ein Block = eine Objektlage (AP 5.2).
    """
    text = (text or "").strip()
    if not text:
        return "B01"
    match = re.search(r"(\d+)$", text)
    if not match:
        return text + " 2"
    digits = match.group(1)
    return text[:match.start()] + str(int(digits) + 1).zfill(len(digits))


def episode_advice(is_real_plant, episodes, put_back=True):
    """Wie viele Episoden je Aufruf sinnvoll sind -- und warum.

    Standard ist Zuruecklegen (``record.py``, Anwender 2026-10-23): Der Arm
    legt das Objekt nach JEDER Episode am Greifpunkt ab und faehrt an den
    Start. Mehrere Fahrten derselben Lage sind dann gewollt, empfohlen 2-4
    (Nutzen danach gering, bc/put_back.py); auch eine einzelne ist in
    Ordnung.

    Abgeschaltet (``--ohne-zuruecklegen``) ist die Antwort an der realen
    Anlage EINE (Anwender,
    2026-09-29), und der Grund steckt im Ablauf: Die Bahn endet am
    Uebergabepunkt, der Arm haelt dort das Objekt. Zu Beginn JEDER Episode
    sendet der Recorder den Greifer-Startzustand ("auf", ``bc/recorder.py``)
    -- Episode 2 wuerde das Objekt also an PRE_PLACE fallen lassen und
    danach einen Griff ins Leere aufzeichnen. Mehrere Episoden je Aufruf
    ergeben dann nur dort Sinn, wo kein physisches Objekt im Spiel ist
    (SimRobot, virtuelle Steuerung).
    """
    if put_back:
        if episodes > 4:
            return ("Mehr als 4 Fahrten derselben Lage bringen kaum noch neue "
                    "Abdeckung — besser eine neue Objektlage.")
        return None
    if not is_real_plant:
        return None
    if episodes > 1:
        return ("An der realen Anlage gehört zu einem Aufruf genau EINE "
                "Episode: Die Bahn endet am Übergabepunkt, und zu Beginn "
                "der nächsten Episode öffnet der Greifer dort — das Objekt "
                "läge dann nicht mehr am Greifpunkt. Objekt neu hinlegen, "
                "Punkte neu teachen, erneut aufnehmen — oder „Zurücklegen“ "
                "wieder einschalten, dann legt der Arm das Objekt nach jeder "
                "Episode selbst am Greifpunkt ab.")
    return None


def setup_problems(robot_kind, sequence):
    """Was an dieser Kombination nicht aufgehen kann -- vor dem Start.

    Der Fall, der ohne diese Pruefung teuer ist: Ablaufdatei gewaehlt, aber
    gegen den SimRobot gefahren. Die Punkte stehen in der Datenbank der
    Control-Box und werden je Episode frisch von dort geholt (bc/sequence.py,
    damit Touch-ups sofort wirken). Der SimRobot haelt eine eigene, leere
    Liste -- ``record.py`` bricht dann mit "Pflichtpunkte fehlen ...
    (vorhanden: )" ab, und aus der leeren Klammer ist die Ursache nicht zu
    erraten.
    """
    problems = []
    if sequence and robot_kind == "sim":
        problems.append(
            "Die Ablaufdatei holt ihre Punkte aus der Datenbank der Steuerung. "
            "Der SimRobot hat keine — oben bei \"Ziel\" die Neura-Steuerung "
            "wählen, oder den Ablauf auf „keine\" stellen (Demo-Wegpunkte).")
    return problems


def plant_requirements(block, sequence):
    """Was apps/record.py an der realen Anlage verlangt -- vorher gesagt.

    Dort bricht es mit einer Meldung ab; hier steht es, bevor jemand
    startet.
    """
    missing = []
    if not block:
        missing.append(
            "Block-ID (--block): an der Anlage gehört jede Aufzeichnung zu "
            "einer Objektlage, sonst sind die Ablationen nicht auswertbar "
            "(AP 5.2).")
    if not sequence:
        missing.append(
            "Ablaufdatei (--sequence): ohne sie fährt record.py die "
            "Demo-Wegpunkte um die Home-Pose.")
    return missing


# -- Ausgabe von apps/record.py lesen --------------------------------------

#: "BEWERTUNG Episode 0 (129 Schritte) -- [e]rfolgreich, ..." -- die Zeile,
#: mit der apps/record.py --ask-label auf eine Antwort wartet (AP 5.2).
#: Ohne Antwort steht der Lauf; die Oberflaeche stellt die Frage selbst.
LABEL_PROMPT = "BEWERTUNG"
_LABEL = re.compile(r"^BEWERTUNG\s+Episode\s+(?P<episode>\d+)\s*"
                    r"\((?P<steps>\d+)\s+Schritte\)")

#: Antwort -> Zeichen fuer stdin (muss zu record.LABEL_ANSWERS passen)
LABEL_ANSWERS = {"erfolg": "e", "fehler": "f", "verwerfen": "v"}


def label_question(line):
    """Fragt diese Zeile nach einer Bewertung? Liefert (Episode, Schritte)."""
    match = _LABEL.match((line or "").strip())
    if not match:
        return None
    return int(match.group("episode")), int(match.group("steps"))


#: "PROBEFAHRT beendet -- Aufnahme starten? [j]a / [n]ein" -- nach der
#: Probefahrt wartet apps/record.py auf die Freigabe (bc/probe.py).
PROBE_PROMPT = "PROBEFAHRT"
_PROBE = re.compile(r"^PROBEFAHRT\s+beendet")

#: Antwort -> Zeichen fuer stdin (muss zu record.run_probe passen)
PROBE_ANSWERS = {"weiter": "j", "abbrechen": "n"}


def probe_question(line):
    """Bittet diese Zeile nach der Probefahrt um Freigabe?"""
    return bool(_PROBE.match((line or "").strip()))


#: "  Takt   90 von 129  Versatz   0 ms  ausserhalb Budget 0"
_TICK = re.compile(r"^\s*Takt\s+(?P<step>\d+)\s+von\s+(?P<total>\d+)\s+"
                   r"Versatz\s+(?P<skew>[\d.]+)\s*ms\s+"
                   r"ausserhalb Budget\s+(?P<bad>\d+)")
#: "Episode 0 -> ep_00000: 129 Schritte, ok" / "... VERWORFEN (schutzstopp)"
_EPISODE = re.compile(r"^Episode\s+(?P<episode>\d+)\s*->\s*(?P<dir>\S+):\s*"
                      r"(?P<steps>\d+)\s+Schritte,\s*(?P<outcome>.+)$")
#: "Soll-Bahn: 129 Schritte (8.6 s bei 15 Hz), 16 Dwell-Schritte"
_PLAN = re.compile(r"^Soll-Bahn:\s+(?P<steps>\d+)\s+Schritte\s*"
                   r"\((?P<seconds>[\d.]+)\s*s")
#: "Lauf-Einstellungen: servo_j 60 Hz, Rauschen je Episode zufaellig bis x1.00, Seed 123, ..."
_SETTINGS = re.compile(r"^Lauf-Einstellungen:.*Seed\s+(?P<seed>\d+)")
#: "  Episoden gesamt   : 3"
_TOTAL = re.compile(r"^\s*Episoden gesamt\s*:\s*(?P<total>\d+)")
_USABLE = re.compile(r"^\s*davon verwendbar\s*:\s*(?P<usable>\d+)")
_DISCARDED = re.compile(r"^\s*davon verworfen\s*:\s*(?P<discarded>\d+)")


class Episode(object):
    def __init__(self, index, folder, steps, outcome):
        self.index = index
        self.folder = folder
        self.steps = steps
        self.outcome = outcome

    @property
    def discarded(self):
        return "VERWORFEN" in self.outcome.upper()

    def row(self):
        return [str(self.index), self.folder, str(self.steps), self.outcome]


class RecordProgress(object):
    """Liest die Ausgabe von apps/record.py mit.

    Tolerant wie in den anderen Modi: Verbindlich ist der Datensatz auf der
    Platte, das hier ist eine Lesehilfe.
    """

    def __init__(self, total_episodes=None):
        self.total_episodes = total_episodes
        self.episodes = []
        self.step = None
        self.steps_per_episode = None
        self.seconds_per_episode = None
        self.skew_ms = None
        self.bad_frames = 0
        self.seed = None
        self.total = None
        self.usable = None
        self.discarded = None
        self.last_event = ""

    def feed(self, line):
        line = (line or "").rstrip()
        if not line:
            return False

        match = _SETTINGS.match(line)
        if match:
            self.seed = int(match.group("seed"))
            self.last_event = "Lauf vorbereitet"
            return True

        match = _PLAN.match(line)
        if match:
            self.steps_per_episode = int(match.group("steps"))
            self.seconds_per_episode = float(match.group("seconds"))
            self.last_event = "Bahn geplant"
            return True

        match = _TICK.match(line)
        if match:
            self.step = int(match.group("step"))
            self.steps_per_episode = int(match.group("total"))
            self.skew_ms = float(match.group("skew"))
            self.bad_frames = int(match.group("bad"))
            self.last_event = "zeichnet auf"
            return True

        match = _EPISODE.match(line)
        if match:
            self.episodes.append(Episode(
                int(match.group("episode")), match.group("dir"),
                int(match.group("steps")), match.group("outcome").strip()))
            self.step = None
            self.last_event = "Episode abgelegt"
            return True

        for pattern, field in ((_TOTAL, "total"), (_USABLE, "usable"),
                               (_DISCARDED, "discarded")):
            match = pattern.match(line)
            if match:
                setattr(self, field, int(match.group(field)))
                self.last_event = "fertig"
                return True
        return False

    @property
    def percent(self):
        if not self.total_episodes:
            return None
        done = len(self.episodes)
        within = 0.0
        if self.step is not None and self.steps_per_episode:
            within = min(1.0, float(self.step) / float(self.steps_per_episode))
        return max(0, min(100, int(100.0 * (done + within) / self.total_episodes)))

    def headline(self):
        if self.total is not None:
            return "%d Episoden, %s verwendbar, %s verworfen" % (
                self.total, self.usable, self.discarded)
        if not self.episodes and self.step is None:
            return self.last_event or "wartet auf die erste Episode"
        done = len(self.episodes)
        bits = ["Episode %d%s" % (done + 1 if self.step is not None else done,
                                  " von %d" % self.total_episodes
                                  if self.total_episodes else "")]
        if self.step is not None and self.steps_per_episode:
            bits.append("Takt %d von %d" % (self.step, self.steps_per_episode))
        kept = sum(1 for e in self.episodes if not e.discarded)
        if self.episodes:
            bits.append("%d von %d behalten" % (kept, done))
        return "  ·  ".join(bits)

    def detail_line(self):
        bits = []
        if self.steps_per_episode and self.seconds_per_episode:
            bits.append("%d Schritte, %.1f s je Episode bei %g Hz"
                        % (self.steps_per_episode, self.seconds_per_episode,
                           config.CONTROL_RATE_HZ))
        if self.skew_ms is not None:
            budget = 1e3 * config.SYNC_MAX_SKEW_S
            bits.append("Versatz %.0f ms %s Budget %.0f ms"
                        % (self.skew_ms,
                           "über" if self.skew_ms > budget else "unter", budget))
        if self.bad_frames:
            bits.append("%d Takte außerhalb des Budgets" % self.bad_frames)
        if self.seed is not None:
            bits.append("Seed %d" % self.seed)
        return "  ·  ".join(bits)


# -- Ablauf bearbeiten: Arbeitskopie statt Original -----------------------
#
# Festlegung Anwender (2026-09-29): Die Ablaufdatei im Projekt bleibt
# UNVERAENDERT und dient damit zugleich als Sicherungskopie. Bearbeitet wird
# eine Arbeitskopie daneben, und genau die wird aufgezeichnet. Sie bleibt
# liegen -- beim naechsten Start steht der letzte Stand wieder da, statt dass
# man dieselben Aenderungen erneut eintippt.

#: Wo die Arbeitskopien liegen (gitignored -- sie gehoeren zum Arbeitsplatz,
#: nicht ins Repository).
WORKING_ROOT = "sequences/arbeitskopien"

#: Bewegungsarten und Greiferzustaende, wie bc/sequence.py sie zulaesst.
MOTIONS = ("ptp", "lin")
GRIPPER_VALUES = (None, "open", "close")


def working_copy_path(workdir, source, root=WORKING_ROOT):
    """Pfad der Arbeitskopie zu einer Ablaufdatei."""
    return relative(Path(workdir) / root / Path(source).name, workdir)


def ensure_working_copy(workdir, source, root=WORKING_ROOT):
    """Arbeitskopie anlegen, falls sie fehlt. Liefert ihren Pfad.

    Angelegt wird sie als 1:1-Kopie des Originals. Existiert sie schon,
    bleibt sie unberuehrt -- das ist der letzte Stand, und den will man beim
    naechsten Start wiederhaben.
    """
    target = Path(workdir) / working_copy_path(workdir, source, root)
    if target.is_file():
        return relative(target, workdir)
    source = Path(workdir) / source if not Path(source).is_absolute() else Path(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(Path(source).read_text(encoding="utf-8"), encoding="utf-8")
    return relative(target, workdir)


def reset_working_copy(workdir, source, root=WORKING_ROOT):
    """Arbeitskopie verwerfen und aus dem Original neu anlegen."""
    target = Path(workdir) / working_copy_path(workdir, source, root)
    if target.is_file():
        target.unlink()
    return ensure_working_copy(workdir, source, root)


def read_steps(path):
    """Ablaufdatei -> (Name, gripper_start, Schritte als dicts).

    Die dicts sind vollstaendig (alle Felder gesetzt), damit die Tabelle im
    Reiter nicht zwischen "fehlt" und "Vorgabewert" unterscheiden muss.
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    steps = []
    for raw in data.get("sequence") or []:
        steps.append({
            "point": str(raw.get("point", "")),
            "motion": str(raw.get("motion", "ptp")),
            "blend": float(raw.get("blend", 0.0) or 0.0),
            "optional": bool(raw.get("optional", False)),
            "approach": bool(raw.get("approach", False)),
            "gripper": raw.get("gripper"),
            # Berechneter Punkt (bc/sequence.py): Bezugspunkt und Abstand in m
            "above": raw.get("above") or None,
            "dz": float(raw.get("dz", 0.0) or 0.0),
        })
    return data.get("name") or Path(path).stem, data.get("gripper_start", "open"), steps


def read_table(path):
    """``"table"``-Angabe einer Ablaufdatei (Tischpunkte, Greifhoehe) oder None."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    table = data.get("table")
    return dict(table) if isinstance(table, dict) else None


def steps_to_data(name, gripper_start, steps, table=None):
    """Schritte -> dict fuer die JSON, ohne Vorgabewerte auszuschreiben.

    Nur was vom Standard abweicht, landet in der Datei -- so bleibt sie so
    lesbar wie die von Hand geschriebene Vorlage.
    """
    out = []
    for step in steps:
        entry = {"point": str(step["point"]).strip(),
                 "motion": str(step.get("motion") or "ptp")}
        if step.get("approach"):
            entry["approach"] = True
        if step.get("optional"):
            entry["optional"] = True
        if step.get("gripper"):
            entry["gripper"] = step["gripper"]
        if float(step.get("blend") or 0.0) > 0:
            entry["blend"] = round(float(step["blend"]), 4)
        if step.get("above"):
            entry["above"] = str(step["above"]).strip()
            entry["dz"] = round(float(step.get("dz") or 0.0), 4)
        out.append(entry)
    data = {"name": name, "gripper_start": gripper_start}
    if table:
        # Tischpunkte und Greifhoehe bleiben beim Speichern erhalten
        data["table"] = table
    data["sequence"] = out
    return data


def validate_steps(name, gripper_start, steps, table=None):
    """Pruefen, OHNE zu schreiben. Liefert eine Liste von Klartext-Fehlern.

    Geprueft wird mit ``bc.sequence.parse_sequence`` -- also genau dem, was
    auch ``record.py`` beim Laden anwendet. Eine zweite, eigene Pruefung
    waere eine zweite Wahrheit.
    """
    from ..sequence import SequenceError, parse_sequence

    problems = []
    for i, step in enumerate(steps):
        if not str(step.get("point") or "").strip():
            problems.append("Schritt %d: kein Punktname" % i)
    if problems:
        return problems
    try:
        parse_sequence(steps_to_data(name, gripper_start, steps, table))
    except SequenceError as exc:
        problems.append(str(exc))
    return problems


def write_steps(path, name, gripper_start, steps, table=None):
    """Arbeitskopie schreiben -- nur, wenn sie gueltig ist.

    Eine ungueltige Ablaufdatei wuerde erst beim Start auffallen, und dann
    steht jemand an der Anlage davor. Deshalb hier dieselbe Pruefung wie in
    record.py, bevor etwas auf die Platte geht.
    """
    problems = validate_steps(name, gripper_start, steps, table)
    if problems:
        raise ValueError("\n".join(problems))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(steps_to_data(name, gripper_start, steps, table),
                   indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    return path

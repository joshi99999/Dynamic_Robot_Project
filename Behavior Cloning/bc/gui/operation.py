"""Was der Modus "Betrieb" ausfuehrt und wie seine Ausgabe gelesen wird -- Qt-frei.

Wie in den anderen Modi laeuft ``apps/infer.py`` als UNTERPROZESS. Hier
wiegt der Grund am schwersten: Eine Policy-Fahrt bewegt den Roboter. Ein
Abbruch muss auch dann greifen, wenn im GUI-Prozess gerade eine Vorhersage
auf der GPU rechnet oder ein Controller-Aufruf haengt -- als eigener Prozess
ist das ein Prozessende und keine Hoffnung auf ein kooperatives Flag. Das
bleibt ein SOFTWARE-Stopp und ersetzt keinen zertifizierten Not-Aus
(AP 4.2).

Der zweite Teil dieses Moduls ist die **Vorabpruefung**. ``apps/infer.py``
erzwingt Gleichheit mit der Aufzeichnung (Schema, Rate, Kameras, servo_j-Rate,
Override) und bricht sonst ab -- zu Recht, denn Tempo und Folgeverhalten sind
mitgelernt (AP 2.6). Dieselben Pruefungen stehen hier noch einmal, aber
vorher und ohne etwas zu oeffnen: Am Labortag soll man sehen, dass die
Kameras nicht passen, bevor der Arm sich bewegt -- nicht als Abbruchmeldung
danach.
"""

import json
import re
from pathlib import Path

from .. import config

CHECKPOINT_ROOT = "checkpoints"

#: Dateiname, an dem ein Policy-Ordner erkannt wird (apps/train.py legt ihn an).
POLICY_MARKER = "bc_policy.json"


# -- Welche Modelle liegen da ---------------------------------------------

def policy_dir(path):
    """Der Ordner, den ``apps/infer.py --checkpoint`` tatsaechlich laedt.

    Dieselbe Aufloesung wie ``infer.resolve_checkpoint``: Ein Lauf hat die
    fertige Policy unter ``<lauf>/policy``; zeigt man direkt darauf, bleibt
    es dabei.
    """
    path = Path(path)
    if (path / "policy" / POLICY_MARKER).is_file():
        return path / "policy"
    return path


def has_policy(path):
    return (policy_dir(path) / POLICY_MARKER).is_file()


def find_policies(workdir, root=CHECKPOINT_ROOT):
    """Alle Laeufe unter ``checkpoints/`` mit einer ladbaren Policy.

    Zwischenstaende (``checkpoints/step_*``) werden mitgenommen: Nach dem
    Durchstich ist genau der Vergleich "Schritt 4000 gegen 8000" die Frage,
    die man am Rechner stellt.
    """
    from .training import relative

    base = Path(workdir) / root
    if not base.is_dir():
        return []
    found = []
    for run in sorted(base.glob("*")):
        if not run.is_dir():
            continue
        if has_policy(run):
            found.append(relative(run, workdir))
        for step in sorted((run / "checkpoints").glob("step_*")):
            if has_policy(step):
                found.append(relative(step, workdir))
    return found


def describe_policy(path):
    """Alles, was vor einer Fahrt ueber das Modell bekannt sein muss."""
    info = _read_json(policy_dir(path) / POLICY_MARKER)
    recording = info.get("recording") or {}
    training = info.get("training") or {}
    dataset = info.get("dataset") or {}
    defaults = info.get("inference_defaults") or {}
    return {
        "path": Path(path),
        "policy_dir": policy_dir(path),
        "found": bool(info),
        "schema_version": info.get("schema_version"),
        "rate_hz": info.get("rate_hz"),
        "cameras": list(info.get("cameras") or []),
        "camera_backends": dict(recording.get("camera_backends") or {}),
        "robot": recording.get("robot"),
        "in_simulation": recording.get("in_simulation"),
        "override": recording.get("override"),
        "servo_rate_hz": recording.get("servo_rate_hz"),
        "dataset": dataset.get("path"),
        "episodes": dataset.get("episodes"),
        "frames": dataset.get("frames"),
        "step": training.get("step"),
        "steps_planned": training.get("steps_planned"),
        "replan_steps": defaults.get("replan_steps"),
        "ensemble_decay": defaults.get("ensemble_decay"),
        "episode_length_max": info.get("episode_length_max"),
        "lerobot_version": info.get("lerobot_version"),
        "created": info.get("created"),
    }


def policy_summary(described):
    """Zwei Zeilen Klartext ueber das Modell -- Herkunft und Randbedingungen."""
    if not described["found"]:
        return "Kein bc_policy.json — das ist kein Policy-Ordner."
    first = []
    if described["step"]:
        first.append("Schritt %s von %s" % (described["step"], described["steps_planned"]))
    if described["dataset"]:
        first.append("Datensatz %s" % described["dataset"])
    if described["episodes"]:
        first.append("%s Episoden" % described["episodes"])
    if described["created"]:
        first.append(described["created"].replace("T", " "))

    second = []
    if described["rate_hz"] is not None:
        second.append("%g Hz" % described["rate_hz"])
    if described["servo_rate_hz"] is not None:
        second.append("servo_j %g Hz" % described["servo_rate_hz"])
    if described["override"] is not None:
        second.append("Override %.2f" % described["override"])
    if described["cameras"]:
        backends = described["camera_backends"]
        if backends:
            second.append("Kameras " + ", ".join(
                "%s=%s" % (c, backends.get(c, "?")) for c in described["cameras"]))
        else:
            # Aeltere Exporte haben camera_backends=null. Das ist kein
            # "unbekanntes Backend", sondern eine fehlende Angabe -- und
            # damit faellt auch die Pruefung von apps/infer.py aus.
            second.append("Kameras %s (Backends nicht vermerkt)"
                          % ", ".join(described["cameras"]))
    if described["robot"]:
        second.append("aufgezeichnet mit '%s'%s" % (
            described["robot"],
            "" if described["in_simulation"] is None
            else (" (Simulation)" if described["in_simulation"] else " (ANLAGE)")))
    return "\n".join(x for x in ("  ·  ".join(first), "  ·  ".join(second)) if x)


# -- Vorabpruefung ---------------------------------------------------------

class Problem(object):
    """Ein Befund vor der Fahrt.

    ``blocking`` heisst: ``apps/infer.py`` bricht damit ab. Das ist keine
    Meinung der Oberflaeche, sondern das, was dort erzwungen wird -- deshalb
    wird es hier vorher gesagt und nicht als Ueberraschung nach dem Start.
    """

    def __init__(self, text, blocking=True):
        self.text = text
        self.blocking = blocking

    def __repr__(self):
        return "<Problem %s %r>" % ("blockend" if self.blocking else "Hinweis", self.text)


def compatibility(described, assignment=None, robot_kind="sim", override=None,
                  is_real_plant=False):
    """Passt diese Fahrt zu diesem Modell? Liefert Befunde, blockende zuerst.

    Gespiegelt aus ``apps/infer.py`` (Schema, Rate, Kameras, Override) --
    absichtlich doppelt: dort ist es der Schutz, hier die Ansage vorher.
    """
    problems = []
    if not described["found"]:
        return [Problem("Kein bc_policy.json im gewählten Ordner — "
                        "apps/infer.py kann daraus keine Policy laden.")]

    if described["schema_version"] != config.SCHEMA_VERSION:
        problems.append(Problem(
            "Modell hat Schema v%s, der Code erwartet v%d — Datensätze und "
            "Modelle verschiedener Schemata nie mischen."
            % (described["schema_version"], config.SCHEMA_VERSION)))
    if described["rate_hz"] is not None and float(described["rate_hz"]) != config.CONTROL_RATE_HZ:
        problems.append(Problem(
            "Modell wurde mit %g Hz aufgezeichnet, Festlegung ist %g Hz — der "
            "zeitliche Abstand zweier Aktionen steckt im Datensatz (AP 1.3)."
            % (float(described["rate_hz"]), config.CONTROL_RATE_HZ)))
    if (described["lerobot_version"]
            and described["lerobot_version"] != config.LEROBOT_VERSION):
        problems.append(Problem(
            "Modell mit lerobot %s, hier gepinnt ist %s."
            % (described["lerobot_version"], config.LEROBOT_VERSION)))

    if assignment:
        now = {}
        for slot, key in assignment.items():
            now[slot] = str(key).split(":")[0]
        recorded = described["camera_backends"]
        missing = [c for c in described["cameras"] if c not in now]
        if missing:
            problems.append(Problem(
                "Das Modell erwartet die Kamera(s) %s — sie sind nicht besetzt."
                % ", ".join(missing)))
        differ = ["%s: aufgezeichnet %s, jetzt %s" % (c, recorded.get(c), now[c])
                  for c in described["cameras"]
                  if c in now and recorded.get(c) and recorded[c] != now[c]]
        if differ:
            problems.append(Problem(
                "Kamera-Backends weichen von der Aufzeichnung ab — %s. "
                "apps/infer.py bricht deswegen ab." % "; ".join(differ)))

    if override is not None and described["override"] is not None:
        if abs(float(override) - float(described["override"])) > 1e-9:
            problems.append(Problem(
                "Override %.2f, aufgezeichnet wurde mit %.2f — das Tempo ist "
                "mitgelernt (AP 2.6). apps/infer.py erzwingt denselben Wert."
                % (float(override), float(described["override"]))))

    # -- Hinweise, die nichts blockieren -----------------------------------
    if described["robot"] and described["robot"] != robot_kind:
        problems.append(Problem(
            "Aufgezeichnet mit Roboter '%s', jetzt '%s'."
            % (described["robot"], robot_kind), blocking=False))
    if is_real_plant and described["in_simulation"] is True:
        problems.append(Problem(
            "Das Modell stammt aus der Simulation und soll die REALE ANLAGE "
            "fahren. Kinematik, Latenzen und Greifer-Totzeit sind dort andere.",
            blocking=False))
    if described["camera_backends"] and all(
            b == "sim" for b in described["camera_backends"].values()):
        problems.append(Problem(
            "Das Modell wurde mit Platzhalterbildern trainiert — es kann aus "
            "dem Bild nichts gelernt haben (Durchstich 2026-09-17).",
            blocking=False))
    elif described["cameras"] and not described["camera_backends"]:
        problems.append(Problem(
            "Die Aufzeichnung hat die Kamera-Backends nicht vermerkt (älterer "
            "Export) — apps/infer.py kann sie nicht gegenprüfen. Ob die Bilder "
            "zu denen des Trainings passen, muss hier von Hand stimmen.",
            blocking=False))
    return sorted(problems, key=lambda p: not p.blocking)


def blocking(problems):
    return [p for p in problems if p.blocking]


# -- Kommandozeile ---------------------------------------------------------

def infer_argv(checkpoint, robot_kind="sim", episodes=1, camera_specs=None,
               override=None, max_steps=None, replan=None, device="auto",
               realtime=False, hold=False, no_ensemble=False, out=None,
               real_robot=False, seed=None):
    """Kommandozeile fuer apps/infer.py (ohne das Python-Programm).

    ``hold`` faehrt ohne Modell (HoldPolicy) -- der Verdrahtungstest, mit dem
    man Takt, Servo-Kette und Kameras prueft, bevor eine Policy dranhaengt.
    """
    argv = ["apps/infer.py"]
    if hold:
        argv.append("--hold")
    else:
        if not checkpoint:
            raise ValueError("Kein Modell ausgewaehlt.")
        argv += ["--checkpoint", str(checkpoint)]
    argv += ["--robot", robot_kind]
    for spec in camera_specs or []:
        argv += ["--camera", spec]
    if override is not None:
        argv += ["--override", "%g" % float(override)]
    argv += ["--episodes", str(int(episodes))]
    if max_steps:
        argv += ["--steps", str(int(max_steps))]
    if replan:
        argv += ["--replan", str(int(replan))]
    if no_ensemble:
        argv.append("--no-ensemble")
    if device and device != "auto":
        argv += ["--device", str(device)]
    if realtime:
        argv.append("--realtime")
    if seed is not None:
        argv += ["--seed", str(int(seed))]
    if out:
        argv += ["--out", str(out)]
    if real_robot:
        argv.append("--real-robot")
    return argv


# -- Ausgabe von apps/infer.py lesen ---------------------------------------

#: "Fahrt 0: ERFOLG -- 176 Takte, Greifpunkt 3.2 mm, Ende 5.1 mm, Bahn p95 8.0 mm,
#:  end_reached, Vorhersage 77.1 ms, Overruns 0, gekappt 3"
_EPISODE = re.compile(
    r"^Fahrt\s+(?P<episode>\d+):\s+(?P<outcome>ERFOLG|kein Erfolg)\s*--\s*"
    r"(?P<steps>\d+)\s+Takte"
    r",\s*Greifpunkt\s+(?P<grasp>[-\d.]+|-)\s*(?:mm)?"
    r",\s*Ende\s+(?P<end>[-\d.]+|-)\s*(?:mm)?"
    r",\s*Bahn p95\s+(?P<path>[-\d.]+|-)\s*(?:mm)?"
    r",\s*(?P<stop>[^,]+)"
    r",\s*Vorhersage\s+(?P<predict>[\d.]+|-)\s*ms"
    r",\s*Overruns\s+(?P<overruns>\d+)"
    r",\s*gekappt\s+(?P<clamped>\d+)")
#: "  Vorhersage nach Aufwaermen: 77.1 ms (10 DDIM-Schritte)"
_WARMUP = re.compile(r"Vorhersage nach Aufwaermen:\s+(?P<ms>[\d.]+)\s*ms"
                     r"(?:\s*\((?P<ddim>\d+)\s*DDIM)?")
#: "  Geraet cuda, trainiert 8000 Schritte auf datasets\vm, Kameras ['wrist', 'scene']"
_LOADED = re.compile(r"^\s*Geraet\s+(?P<device>\S+?),\s+trainiert\s+(?P<step>\d+)\s+Schritte")
#: "Neura: is_robot_in_simulation()=True, Override=1.00, Greifer=..."
_NEURA = re.compile(r"^Neura: is_robot_in_simulation\(\)=(?P<sim>\w+), "
                    r"Override=(?P<override>[\d.]+)")
#: "Erfolg 3/5 (Referenz: ...)"
_TOTAL = re.compile(r"^Erfolg\s+(?P<ok>\d+)/(?P<total>\d+)")
#: "Fahrt 2: an die Startstellung (0.412 rad entfernt)"
_START_MOVE = re.compile(r"^Fahrt\s+(?P<episode>\d+): an die Startstellung")
#: "  Takt   90  Spread 0.0072 rad  Vorhersage 71.4 ms" -- Fortschritt INNERHALB
#: einer Fahrt. Ohne diese Zeile stuende der Balken waehrend einer ganzen
#: Episode still, und die ist bei 15 Hz gut zehn Sekunden lang.
_TICK = re.compile(r"^\s*Takt\s+(?P<step>\d+)\s+Spread\s+(?P<spread>[\d.]+)\s*rad"
                   r"(?:\s+Vorhersage\s+(?P<ms>[\d.]+)\s*ms)?")
#: "Fahrt: bis 194 Takte, ..." -- Obergrenze je Fahrt, Nenner des Balkens.
_LIMITS = re.compile(r"^Fahrt: bis\s+(?P<max_steps>\d+)\s+Takte")


class Rollout(object):
    """Eine gefahrene Episode, aus der Zusammenfassungszeile gelesen."""

    def __init__(self, episode, success, steps, grasp_mm, end_mm, path_mm,
                 stop_reason, predict_ms, overruns, clamped):
        self.episode = episode
        self.success = success
        self.steps = steps
        self.grasp_mm = grasp_mm
        self.end_mm = end_mm
        self.path_mm = path_mm
        self.stop_reason = stop_reason
        self.predict_ms = predict_ms
        self.overruns = overruns
        self.clamped = clamped

    def row(self):
        """Spalten fuer die Tabelle im Reiter."""
        return [
            str(self.episode),
            "Erfolg" if self.success else "kein Erfolg",
            str(self.steps),
            _mm(self.grasp_mm), _mm(self.end_mm), _mm(self.path_mm),
            "-" if self.predict_ms is None else "%.1f ms" % self.predict_ms,
            str(self.overruns),
            self.stop_reason,
        ]


class RolloutProgress(object):
    """Liest die Ausgabe von apps/infer.py mit.

    Tolerant wie im Training: Aendert sich eine Zeile, faellt ein Feld aus,
    aber nichts bricht. Verbindlich ist ``summary.json`` im Protokollordner.
    """

    def __init__(self, total_episodes=None):
        self.total_episodes = total_episodes
        self.rollouts = []
        self.current_episode = None
        self.device = None
        self.trained_steps = None
        self.warmup_ms = None
        self.ddim_steps = None
        self.in_simulation = None
        self.override = None
        self.succeeded = None
        self.total = None
        self.last_event = ""
        self.step = None
        self.max_steps = None
        self.spread_rad = None
        self.predict_ms = None

    def feed(self, line):
        """Eine Ausgabezeile einlesen. Liefert True, wenn sich etwas geaendert hat."""
        line = (line or "").rstrip()
        if not line:
            return False

        match = _EPISODE.match(line)
        if match:
            self.rollouts.append(Rollout(
                episode=int(match.group("episode")),
                success=match.group("outcome") == "ERFOLG",
                steps=int(match.group("steps")),
                grasp_mm=_maybe_float(match.group("grasp")),
                end_mm=_maybe_float(match.group("end")),
                path_mm=_maybe_float(match.group("path")),
                stop_reason=match.group("stop").strip(),
                predict_ms=_maybe_float(match.group("predict")),
                overruns=int(match.group("overruns")),
                clamped=int(match.group("clamped"))))
            self.current_episode = int(match.group("episode"))
            self.step = None
            self.last_event = "Fahrt beendet"
            return True

        match = _LOADED.match(line)
        if match:
            self.device = match.group("device")
            self.trained_steps = int(match.group("step"))
            self.last_event = "Policy geladen"
            return True

        match = _WARMUP.search(line)
        if match:
            self.warmup_ms = float(match.group("ms"))
            if match.group("ddim"):
                self.ddim_steps = int(match.group("ddim"))
            self.last_event = "aufgewärmt"
            return True

        match = _NEURA.match(line)
        if match:
            self.in_simulation = match.group("sim") == "True"
            self.override = float(match.group("override"))
            self.last_event = "mit der Steuerung verbunden"
            return True

        match = _START_MOVE.match(line)
        if match:
            self.current_episode = int(match.group("episode"))
            self.step = None
            self.last_event = "fährt an die Startstellung"
            return True

        match = _LIMITS.match(line)
        if match:
            self.max_steps = int(match.group("max_steps"))
            return True

        match = _TICK.match(line)
        if match:
            self.step = int(match.group("step"))
            self.spread_rad = _maybe_float(match.group("spread"))
            if match.group("ms"):
                self.predict_ms = float(match.group("ms"))
            self.last_event = "fährt"
            return True

        match = _TOTAL.match(line)
        if match:
            self.succeeded = int(match.group("ok"))
            self.total = int(match.group("total"))
            self.last_event = "fertig"
            return True
        return False

    @property
    def percent(self):
        """Fortschritt ueber alle Fahrten, mit dem Takt innerhalb der laufenden.

        Abgerundet -- der Balken behauptet nie mehr, als gefahren ist.
        """
        if not self.total_episodes:
            return None
        done = len(self.rollouts)
        within = 0.0
        if self.step is not None and self.max_steps:
            within = min(1.0, float(self.step) / float(self.max_steps))
        return max(0, min(100, int(100.0 * (done + within) / self.total_episodes)))

    def headline(self):
        if self.total is not None:
            return "Erfolg %d von %d Fahrten" % (self.succeeded, self.total)
        if not self.rollouts:
            return self.last_event or "wartet auf die erste Fahrt"
        done = len(self.rollouts)
        ok = sum(1 for r in self.rollouts if r.success)
        bits = ["Fahrt %d%s" % (done + 1 if self.step is not None else done,
                                " von %d" % self.total_episodes
                                if self.total_episodes else "")]
        if self.step is not None:
            bits.append("Takt %d%s" % (self.step,
                                       " von %d" % self.max_steps if self.max_steps else ""))
        bits.append("Erfolg %d von %d" % (ok, done))
        if self.predict_ms is not None:
            bits.append("Vorhersage %.1f ms" % self.predict_ms)
        return "  ·  ".join(bits)

    def detail_line(self):
        bits = []
        if self.device:
            bits.append("Gerät %s" % self.device)
        if self.trained_steps:
            bits.append("%d Trainingsschritte" % self.trained_steps)
        if self.warmup_ms is not None:
            budget = 1e3 / config.CONTROL_RATE_HZ
            bits.append("Vorhersage %.0f ms %s Takt %.0f ms"
                        % (self.warmup_ms,
                           "über" if self.warmup_ms > budget else "unter", budget))
        if self.in_simulation is not None:
            bits.append("Steuerung meldet %s"
                        % ("Simulation" if self.in_simulation else "KEINE Simulation"))
        if self.override is not None:
            bits.append("Override %.2f" % self.override)
        return "  ·  ".join(bits)

    def totals_line(self):
        if not self.rollouts:
            return ""
        overruns = sum(r.overruns for r in self.rollouts)
        clamped = sum(r.clamped for r in self.rollouts)
        bits = ["%d Fahrten" % len(self.rollouts)]
        if overruns:
            bits.append("%d Takt-Überläufe" % overruns)
        if clamped:
            bits.append("%d mal gekappt (TargetLimiter)" % clamped)
        reasons = {}
        for rollout in self.rollouts:
            reasons[rollout.stop_reason] = reasons.get(rollout.stop_reason, 0) + 1
        bits.append("Ende: " + ", ".join("%s (%dx)" % (k, v)
                                         for k, v in sorted(reasons.items())))
        return "  ·  ".join(bits)


def read_summary(path):
    """``summary.json`` einer Fahrtenreihe -- die verbindlichen Zahlen."""
    data = _read_json(Path(path) / "summary.json")
    if not data:
        return None
    return data


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return {}


def _maybe_float(text):
    if text in (None, "", "-"):
        return None
    try:
        return float(text)
    except ValueError:
        return None


def _mm(value):
    return "-" if value is None else "%.1f mm" % value

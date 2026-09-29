"""Was der Modus "Training" ausfuehrt und wie seine Ausgabe gelesen wird -- Qt-frei.

Wie im Systemcheck laufen ``apps/export.py`` und ``apps/train.py`` als
UNTERPROZESS, nicht als Import. Dieselben Gruende wie in checks.py, plus
zwei, die hier schwerer wiegen:

* Ein Training laeuft Stunden. Im GUI-Prozess wuerde ein Abbruch bedeuten,
  einen torch-Trainingsschritt mitten im CUDA-Kernel zu unterbrechen --
  ueber einen Unterprozess ist Abbrechen einfach "Prozess beenden", und der
  letzte Checkpoint liegt auf der Platte.
* Unter Windows kodiert lerobot die Videos beim Export in eigenen
  Unterprozessen (deshalb ``if __name__ == "__main__"`` in apps/export.py).
  Aus einer laufenden Qt-Anwendung heraus ist das eine Fehlerquelle, die
  man sich mit QProcess spart.

Damit bleibt eine Zusage aus AP 3.1 bestehen: Die Kommandozeile, die hier
zusammengebaut wird, ist genau die, die auch im Terminal gilt und in der
Dokumentation steht. Die Oberflaeche ist eine Eingabehilfe, kein zweiter
Weg -- ``export_argv`` und ``train_argv`` sind deshalb auch das, was der
Reiter dem Bedienenden zum Mitlesen anzeigt.

Hardwareunabhaengig (AP 3.1, Anwender 2026-09-17): Es wird nichts auf die
RTX 5070 Ti verdrahtet. Die Geraeteliste kommt zur Laufzeit aus torch,
Abweichungen von ``bc/config.py`` werden benannt statt versteckt.
"""

import json
import re
from pathlib import Path

from .. import config

#: Ordner, in denen nach Aufzeichnungen gesucht wird (relativ zum Arbeitsordner).
RECORDING_ROOTS = ("data_vm", "data_sim")
#: Ordner der exportierten Datensaetze bzw. der Trainingslaeufe.
DATASET_ROOT = "datasets"
CHECKPOINT_ROOT = "checkpoints"


def relative(path, workdir):
    """Pfad relativ zum Arbeitsordner -- so steht er auch in der Dokumentation.

    Die Unterprozesse laufen ohnehin im Ordner "Behavior Cloning"
    (ProcessRunner setzt das Arbeitsverzeichnis). Absolute Pfade wuerden
    die Kommandozeile unlesbar machen und waeren nicht mehr die Zeile, die
    man ins Terminal kopieren kann. Liegt der Ordner woanders -- ein
    Datensatz von einem anderen Rechner, ein Stick --, bleibt der Pfad
    absolut, sonst zeigte er ins Leere.
    """
    try:
        return Path(path).resolve().relative_to(Path(workdir).resolve())
    except ValueError:
        return Path(path)


# -- Was liegt auf der Platte ---------------------------------------------

def find_recordings(workdir, roots=RECORDING_ROOTS):
    """Aufzeichnungsordner (mit ``index.json``) unter data_vm/ und data_sim/.

    Gesucht wird zwei Ebenen tief, weil die Ablage ``data_vm/<Datum>/<Lauf>``
    ist (Festlegung 2026-09-16), Sim-Daten aber direkt unter ``data_sim/<Lauf>``
    liegen. Sortiert, damit die Liste zwischen zwei Aufrufen gleich aussieht.
    """
    workdir = Path(workdir)
    found = []
    for root in roots:
        base = workdir / root
        if not base.is_dir():
            continue
        for candidate in sorted(base.glob("*")) + sorted(base.glob("*/*")):
            if (candidate / "index.json").is_file():
                found.append(candidate)
    return [relative(p, workdir) for p in dict.fromkeys(found)]


def describe_recording(path):
    """Kurzbeschreibung einer Aufzeichnung aus ihrer ``index.json``.

    Liest nur die Indexdatei, nicht die Episoden -- die Liste im Reiter soll
    sich auch bei 50 Aufzeichnungen sofort aufbauen.
    """
    path = Path(path)
    try:
        index = json.loads((path / "index.json").read_text(encoding="utf-8"))
    except Exception as exc:
        return {"path": path, "error": "%s: %s" % (type(exc).__name__, exc)}
    episodes = index.get("episodes") or []
    discarded = sum(1 for e in episodes if e.get("discarded"))
    return {
        "path": path,
        "episodes": len(episodes),
        "discarded": discarded,
        # Nur, was ohne Oeffnen der Episoden zu haben ist. Erfolgs-Label,
        # Roboter, Override und Tempo stehen je Episode in meta.json --
        # die liest die Konsistenzpruefung (apps/export.py --check), und
        # die ist hier auch die verbindliche Aussage.
        "frames": sum(int(e.get("length") or 0) for e in episodes),
        "cameras": list(index.get("cameras") or []),
        "rate_hz": index.get("rate_hz"),
        "schema_version": index.get("schema_version"),
        "error": None,
    }


def recording_label(described):
    """Eine Zeile je Aufzeichnung fuer die Auswahlliste."""
    if described.get("error"):
        return "%s  --  unlesbar (%s)" % (described["path"], described["error"])
    bits = ["%d Episoden" % described["episodes"]]
    if described["discarded"]:
        bits.append("%d verworfen" % described["discarded"])
    if described["frames"]:
        bits.append("%d Frames" % described["frames"])
    if described["rate_hz"] is not None:
        bits.append("%.0f Hz" % float(described["rate_hz"]))
    if described["schema_version"] != config.SCHEMA_VERSION:
        bits.append("Schema v%s (Code: v%d) -- nicht mischen"
                    % (described["schema_version"], config.SCHEMA_VERSION))
    return "%s  --  %s" % (described["path"], ", ".join(bits))


def find_datasets(workdir, root=DATASET_ROOT):
    """Exportierte LeRobotDatasets (Ordner mit ``meta/info.json``)."""
    base = Path(workdir) / root
    if not base.is_dir():
        return []
    return [relative(p, workdir) for p in sorted(base.glob("*"))
            if (p / "meta" / "info.json").is_file()]


def describe_dataset(path):
    """Was in einem Datensatz steht -- Herkunft, Umfang, Version.

    ``meta/bc_export.json`` schreibt apps/export.py selbst; fehlt sie, ist
    der Ordner von woanders hergekommen (anderer Trainingsrechner, AP 3.1).
    Das ist kein Fehler, muss aber sichtbar sein.
    """
    path = Path(path)
    info = _read_json(path / "meta" / "info.json")
    export = _read_json(path / "meta" / "bc_export.json")
    out = {
        "path": path,
        "episodes": info.get("total_episodes"),
        "frames": info.get("total_frames"),
        "fps": info.get("fps"),
        "codebase_version": info.get("codebase_version"),
        "lerobot_version": export.get("lerobot_version"),
        "schema_version": export.get("schema_version"),
        "cameras": list(export.get("cameras") or []),
        "sources": list(export.get("sources") or []),
        "foreign": not export,
        "warnings": [],
    }
    if export.get("lerobot_version") and export["lerobot_version"] != config.LEROBOT_VERSION:
        out["warnings"].append(
            "mit lerobot %s exportiert, hier gepinnt ist %s -- Format haengt an "
            "der Version (AP 0.9 Punkt 6)"
            % (export["lerobot_version"], config.LEROBOT_VERSION))
    if export.get("schema_version") not in (None, config.SCHEMA_VERSION):
        out["warnings"].append(
            "Schema v%s, Code erwartet v%s -- Datensaetze verschiedener "
            "Versionen nie mischen"
            % (export["schema_version"], config.SCHEMA_VERSION))
    if out["fps"] is not None and float(out["fps"]) != config.CONTROL_RATE_HZ:
        out["warnings"].append(
            "%.1f Hz statt der festgelegten %.1f Hz -- der Zeitabstand zweier "
            "Aktionen steckt im Datensatz (AP 1.3)"
            % (float(out["fps"]), config.CONTROL_RATE_HZ))
    if out["foreign"]:
        out["warnings"].append(
            "keine meta/bc_export.json -- Ordner stammt nicht aus apps/export.py, "
            "Herkunft unbekannt")
    return out


def find_checkpoints(workdir, root=CHECKPOINT_ROOT):
    """Trainingslaeufe (Ordner mit ``train_config.json``)."""
    base = Path(workdir) / root
    if not base.is_dir():
        return []
    return [relative(p, workdir) for p in sorted(base.glob("*"))
            if (p / "train_config.json").is_file()]


def describe_checkpoint(path):
    """Stand eines Trainingslaufs: Datensatz, Schritte, Fortsetzbarkeit."""
    path = Path(path)
    cfg = _read_json(path / "train_config.json")
    args = cfg.get("args") or cfg
    steps = sorted((path / "checkpoints").glob("step_*"))
    resumable = [s for s in steps if (s / "training_state.pt").is_file()]
    return {
        "path": path,
        "dataset": args.get("dataset") or args.get("data"),
        "steps_planned": args.get("steps"),
        "checkpoints": [s.name for s in steps],
        # Ohne training_state.pt laesst sich ein Lauf nicht fortsetzen --
        # die Dateien wurden am 2026-09-22 bewusst geloescht (Testlaeufe).
        "resumable": resumable[-1].name if resumable else None,
        "has_policy": (path / "policy" / "model.safetensors").is_file(),
        "log": (path / "train_log.csv") if (path / "train_log.csv").is_file() else None,
    }


def _read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return {}


# -- Hardware --------------------------------------------------------------

class Device(object):
    """Ein waehlbares Rechengeraet mit dem, was der Bedienende wissen muss."""

    def __init__(self, key, label, usable=True, note=""):
        self.key = key            # was an --device uebergeben wird
        self.label = label        # was im Auswahlfeld steht
        self.usable = usable
        self.note = note

    def __repr__(self):
        return "<Device %s usable=%s>" % (self.key, self.usable)


def devices():
    """Verfuegbare Geraete -- zur Laufzeit erfragt, nichts fest verdrahtet.

    "auto" steht bewusst oben: apps/train.py waehlt damit selbst und prueft
    zusaetzlich, ob der torch-Build die GPU-Architektur kennt (Blackwell
    braucht sm_120 aus einem cu128-Build). Wer dem nicht traut, waehlt das
    Geraet darunter ausdruecklich.
    """
    found = [Device("auto", "automatisch (empfohlen)")]
    try:
        import torch
    except Exception as exc:
        found.append(Device("cpu", "CPU", usable=False,
                            note="torch fehlt (%s)" % type(exc).__name__))
        return found

    if torch.cuda.is_available():
        for i in range(torch.cuda.device_count()):
            props = torch.cuda.get_device_properties(i)
            found.append(Device(
                "cuda:%d" % i,
                "cuda:%d -- %s" % (i, props.name),
                note="%.1f GB VRAM, sm_%d%d" % (
                    props.total_memory / 1e9, props.major, props.minor)))
    if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
        found.append(Device("mps", "mps -- Apple GPU"))
    found.append(Device(
        "cpu", "CPU", usable=True,
        note="ohne GPU dauert ein Training unverhaeltnismaessig lang"))
    return found


def hardware_hint(device_list):
    """Ein Satz ueber die Rechenlage -- wird im Reiter dauerhaft angezeigt."""
    gpus = [d for d in device_list if d.key.startswith("cuda") or d.key == "mps"]
    if gpus:
        return "GPU vorhanden: " + ", ".join(d.label for d in gpus)
    return ("Keine GPU gefunden -- Training laeuft nur auf der CPU und dauert "
            "unverhaeltnismaessig lang. Datensatz hier exportieren und auf "
            "einem Trainingsrechner trainieren (der Ordner ist portabel).")


# -- Hyperparameter --------------------------------------------------------

class Parameter(object):
    """Ein einstellbarer Wert mit Vorgabe aus bc/config.py bzw. apps/train.py.

    ``default`` ist das, was ohne Zutun gilt. Der Reiter zeigt Abweichungen
    ausdruecklich an, statt sie stillschweigend mitzuschicken -- ein
    Trainingslauf, dessen Einstellung man spaeter nicht mehr kennt, ist
    fuer die Berichte wertlos.
    """

    def __init__(self, flag, label, default, kind="int", minimum=None,
                 maximum=None, step=None, choices=None, help_text="",
                 source="apps/train.py"):
        self.flag = flag
        self.label = label
        self.default = default
        self.kind = kind
        self.minimum = minimum
        self.maximum = maximum
        self.step = step
        self.choices = choices
        self.help_text = help_text
        #: Woher die Vorgabe stammt -- "bc/config.py" ist eine Festlegung
        #: aus Abschnitt 5 der Arbeitsanweisung, kein beliebiger Default.
        self.source = source

    @property
    def is_locked_down(self):
        return self.source == "bc/config.py"

    def __repr__(self):
        return "<Parameter %s default=%r>" % (self.flag, self.default)


def parameters():
    """Die Werte, die der Reiter anbietet -- in der Reihenfolge der Anzeige.

    Bewusst NICHT alles, was apps/train.py kennt: Was hier steht, ist das,
    was man zwischen zwei Laeufen tatsaechlich dreht. Alles Uebrige bleibt
    auf dem Default und ist ueber die Kommandozeile erreichbar.
    """
    return [
        Parameter("--steps", "Schritte", 60000, "int", 100, 1000000, 1000,
                  "Optimierer-Schritte. Der Durchstich-Lauf hatte 8000."),
        Parameter("--batch-size", "Batchgroesse", 64, "int", 1, 512, 8,
                  "Gemessen auf einer 5070 Ti (2 Kameras, bf16): 32 -> 0.13 s/Schritt "
                  "bei 3.6 GB, 64 -> 0.21 s/Schritt bei 5.6 GB."),
        Parameter("--grad-accum", "Gradienten sammeln", 1, "int", 1, 32, 1,
                  "Effektive Batch = dieser Wert x Batchgroesse. Auf einer "
                  "kleineren Karte Batch halbieren und diesen Wert verdoppeln -- "
                  "Lernrate und Ergebnis bleiben vergleichbar."),
        Parameter("--lr", "Lernrate", 1e-4, "float", 1e-6, 1e-2, 1e-5),
        Parameter("--val-fraction", "Validierungsanteil", 0.1, "float", 0.0, 0.5, 0.05,
                  "Episodenweise geteilt -- nie Frames einer Episode auf beiden Seiten."),
        Parameter("--eval-every", "Validierung alle", 2000, "int", 100, 100000, 500,
                  "Schritte zwischen zwei Validierungen (Gelenkfehler in rad)."),
        Parameter("--save-every", "Checkpoint alle", 10000, "int", 500, 100000, 500,
                  "Schritte zwischen zwei Checkpoints."),
        Parameter("--seed", "Seed", 0, "int", 0, 999999, 1,
                  "Erfolgsquoten zwischen moderaten Einstellungen sind zu einem "
                  "guten Teil Seed-Rauschen (Rauschstudie 2026-09-16)."),
        # -- ab hier Festlegungen aus bc/config.py --------------------------
        Parameter("--horizon", "Horizont", config.POLICY_HORIZON, "int", 4, 64, 2,
                  "Festlegung: Diffusion, 2 Obs, Horizont 16, 8 nutzbar.",
                  source="bc/config.py"),
        Parameter("--n-action-steps", "nutzbare Schritte",
                  config.POLICY_N_ACTION_STEPS, "int", 1, 64, 1,
                  source="bc/config.py"),
        Parameter("--n-obs-steps", "Beobachtungsschritte",
                  config.POLICY_N_OBS_STEPS, "int", 1, 8, 1,
                  source="bc/config.py"),
        Parameter("--inference-steps", "DDIM-Schritte",
                  config.POLICY_INFERENCE_STEPS, "int", 1, 100, 1,
                  source="bc/config.py"),
        Parameter("--prediction-type", "Vorhersageart",
                  config.POLICY_PREDICTION_TYPE, "choice",
                  choices=("sample", "epsilon"),
                  help_text="epsilon mit 10 DDIM-Schritten ergab gezackte Bahnen "
                            "(Durchstich 2026-09-17) -- deshalb sample.",
                  source="bc/config.py"),
    ]


def deviations(values, params=None):
    """Welche Werte weichen von der Vorgabe ab? Liste von (Parameter, Wert).

    Grundlage der Anzeige "Abweichungen sichtbar machen": nicht die
    Einstellung verbieten, sondern benennen.
    """
    params = params or parameters()
    out = []
    for param in params:
        if param.flag not in values:
            continue
        value = values[param.flag]
        if _differs(value, param.default):
            out.append((param, value))
    return out


def _differs(value, default):
    if isinstance(default, float) or isinstance(value, float):
        try:
            return abs(float(value) - float(default)) > 1e-12
        except (TypeError, ValueError):
            return value != default
    return value != default


# -- Kommandozeilen --------------------------------------------------------

def export_argv(sources, out=None, require_success=False, allow_mixed=False,
                images=False, crf=None, check=False, overwrite=False):
    """Kommandozeile fuer apps/export.py (ohne das Python-Programm).

    ``check=True`` prueft nur und schreibt nichts -- der Reiter bietet das
    als eigenen Knopf an, weil die Konsistenzpruefung (Schema, Rate,
    Override, Tempo) vor jedem Export die eigentliche Frage beantwortet.
    """
    sources = [str(s) for s in sources]
    if not sources:
        raise ValueError("Keine Aufzeichnung ausgewaehlt.")
    argv = ["apps/export.py", "--data"] + sources
    if check:
        return argv + ["--check"] + (["--require-success"] if require_success else [])
    if not out:
        raise ValueError("Kein Zielordner angegeben.")
    argv += ["--out", str(out)]
    if require_success:
        argv.append("--require-success")
    if allow_mixed:
        argv.append("--allow-mixed")
    if images:
        argv.append("--images")
    if crf is not None:
        argv += ["--crf", str(int(crf))]
    if overwrite:
        argv.append("--overwrite")
    return argv


def train_argv(dataset, out, device="auto", values=None, resume=False,
               cameras=None, max_hours=None, params=None):
    """Kommandozeile fuer apps/train.py (ohne das Python-Programm).

    Nur Werte, die von der Vorgabe abweichen, landen als Flag in der Zeile.
    So bleibt sie kurz und lesbar, und man sieht auf einen Blick, was an
    diesem Lauf anders ist -- genau das, was spaeter in den Bericht muss.
    """
    if not dataset:
        raise ValueError("Kein Datensatz ausgewaehlt.")
    if not out:
        raise ValueError("Kein Zielordner fuer den Lauf angegeben.")
    argv = ["apps/train.py", "--dataset", str(dataset), "--out", str(out)]
    if device and device != "auto":
        argv += ["--device", str(device)]
    for param, value in deviations(values or {}, params):
        argv += [param.flag, _format_value(value)]
    if cameras:
        argv += ["--cameras", ",".join(cameras)]
    if max_hours:
        argv += ["--max-hours", "%g" % float(max_hours)]
    if resume:
        argv.append("--resume")
    return argv


def _format_value(value):
    if isinstance(value, float):
        return repr(float(value)) if value < 1e-3 else "%g" % value
    return str(value)


def command_line(argv, program="python"):
    """Die Zeile zum Mitlesen und Kopieren -- so steht sie in der Doku."""
    parts = [program]
    for item in argv:
        text = str(item)
        parts.append('"%s"' % text if " " in text or "*" in text else text)
    return " ".join(parts)


# -- Ausgabe von apps/train.py lesen ---------------------------------------

#: "step    200  loss 0.1011  lr 4.0e-05  grad 1.71 ... 1.3 min (Rest ~49)"
_STEP = re.compile(
    r"^step\s+(?P<step>\d+)\s+loss\s+(?P<loss>[\d.eE+-]+)"
    r"(?:.*?\bmem\s+(?P<mem>[\d.]+)\s*GB)?"
    r"(?:.*?\b(?P<elapsed>[\d.]+)\s*min)?"
    r"(?:.*?\(Rest\s*~(?P<eta>[\d.]+)\))?")
#: "| val loss 0.0194  Gelenk-MAE 0.0251 rad (p95 0.0922)  Greifer 98.6 %"
_VAL = re.compile(
    r"val loss\s+(?P<val_loss>[\d.eE+-]+)"
    r"(?:\s+Gelenk-MAE\s+(?P<mae>[\d.eE+-]+)\s*rad"
    r"(?:\s*\(p95\s+(?P<p95>[\d.eE+-]+)\))?)?"
    r"(?:\s+Greifer\s+(?P<gripper>[\d.]+)\s*%)?")
#: "Datensatz datasets\vm: 30 Episoden (27 Training, 3 Validierung), 4752 Frames"
_DATASET = re.compile(
    r"^Datensatz\s+(?P<name>.+?):\s+(?P<episodes>\d+)\s+Episoden"
    r"(?:\s*\((?P<split>[^)]*)\))?,\s*(?P<frames>\d+)\s+Frames")
#: "Parameter: 89.5 M, effektive Batch 64, Worker 4"
_PARAMS = re.compile(r"^Parameter:\s+(?P<params>[\d.]+)\s*M,\s*effektive Batch\s+(?P<batch>\d+)")
#: "Hardware: {...}" -- apps/train.py schreibt die Zeile als JSON.
_HARDWARE = re.compile(r"^Hardware:\s*(?P<json>\{.*\})\s*$")


class TrainingProgress(object):
    """Liest die Ausgabe von apps/train.py mit und haelt den Stand.

    Bewusst tolerant: Aendert sich eine Zeile in apps/train.py, faellt hier
    ein Feld aus, aber nichts bricht -- die Rohausgabe steht ohnehin
    daneben im Log. Der Fortschritt ist eine Lesehilfe, keine zweite
    Wahrheit; die verbindlichen Zahlen stehen in ``train_log.csv``.
    """

    def __init__(self, total_steps=None):
        self.total_steps = total_steps
        self.step = 0
        self.loss = None
        self.gpu_mem_gb = None
        self.elapsed_min = None
        self.eta_min = None
        self.val_loss = None
        self.val_mae_rad = None
        self.val_p95_rad = None
        self.val_gripper_pct = None
        self.episodes = None
        self.frames = None
        self.split = None
        self.dataset_name = None
        self.effective_batch = None
        self.parameters_m = None
        self.hardware = {}

    def feed(self, line):
        """Eine Ausgabezeile einlesen. Liefert True, wenn sich etwas geaendert hat."""
        line = (line or "").strip()
        if not line:
            return False
        changed = False

        match = _HARDWARE.match(line)
        if match:
            try:
                self.hardware = json.loads(match.group("json"))
                changed = True
            except ValueError:
                pass

        match = _DATASET.match(line)
        if match:
            self.dataset_name = match.group("name")
            self.episodes = int(match.group("episodes"))
            self.frames = int(match.group("frames"))
            self.split = match.group("split")
            changed = True

        match = _PARAMS.match(line)
        if match:
            self.parameters_m = float(match.group("params"))
            self.effective_batch = int(match.group("batch"))
            changed = True

        match = _STEP.match(line)
        if match:
            self.step = int(match.group("step"))
            self.loss = float(match.group("loss"))
            self.gpu_mem_gb = _maybe_float(match.group("mem"))
            self.elapsed_min = _maybe_float(match.group("elapsed"))
            self.eta_min = _maybe_float(match.group("eta"))
            changed = True

        match = _VAL.search(line)
        if match:
            self.val_loss = _maybe_float(match.group("val_loss"))
            self.val_mae_rad = _maybe_float(match.group("mae"))
            self.val_p95_rad = _maybe_float(match.group("p95"))
            self.val_gripper_pct = _maybe_float(match.group("gripper"))
            changed = True

        return changed

    @property
    def percent(self):
        """Fortschritt in Prozent, ABGERUNDET.

        Abgerundet, damit der Balken nie mehr behauptet, als gerechnet ist
        -- und damit der Wert nicht von der Rundungsregel abhaengt
        (``round(12.5)`` ist in Python 12, nicht 13).
        """
        if not self.total_steps:
            return None
        return max(0, min(100, int(100.0 * self.step / self.total_steps)))

    def headline(self):
        """Eine Zeile Fortschritt -- das, was man im Vorbeigehen liest."""
        if not self.step:
            return "wartet auf den ersten Schritt"
        bits = ["Schritt %d" % self.step]
        if self.total_steps:
            bits[0] += " von %d" % self.total_steps
        if self.loss is not None:
            bits.append("Loss %.4f" % self.loss)
        if self.val_mae_rad is not None:
            bits.append("Validierung %.4f rad" % self.val_mae_rad)
        if self.eta_min is not None:
            bits.append("Rest ~%d min" % int(round(self.eta_min)))
        return "  ·  ".join(bits)

    def dataset_line(self):
        if self.episodes is None:
            return ""
        bits = ["%d Episoden" % self.episodes]
        if self.split:
            bits[0] += " (%s)" % self.split
        if self.frames is not None:
            bits.append("%d Frames" % self.frames)
        if self.effective_batch:
            bits.append("effektive Batch %d" % self.effective_batch)
        if self.parameters_m:
            bits.append("%.1f M Parameter" % self.parameters_m)
        return "  ·  ".join(bits)

    def hardware_line(self):
        if not self.hardware:
            return ""
        bits = [str(self.hardware.get("gpu") or self.hardware.get("device") or "?")]
        if self.hardware.get("vram_gb"):
            bits.append("%.1f GB" % float(self.hardware["vram_gb"]))
        if self.hardware.get("torch"):
            bits.append("torch %s" % self.hardware["torch"])
        if self.gpu_mem_gb is not None:
            bits.append("belegt %.1f GB" % self.gpu_mem_gb)
        return "  ·  ".join(bits)


def _maybe_float(text):
    if text is None:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def read_train_log(path, limit=None):
    """``train_log.csv`` einlesen -- die verbindlichen Zahlen eines Laufs.

    Liefert (Spaltennamen, Zeilen als dict). Fehlt oder haengt die Datei
    gerade mitten im Schreiben, ist das ein leeres Ergebnis und kein Fehler:
    Der Reiter liest sie waehrend des Laufs wiederholt.
    """
    import csv

    path = Path(path)
    if not path.is_file():
        return [], []
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            rows = [row for row in reader]
            fields = list(reader.fieldnames or [])
    except OSError:
        return [], []
    if limit:
        rows = rows[-int(limit):]
    return fields, rows


def val_series(rows, key="val_joint_mae_rad"):
    """(Schritt, Wert)-Paare einer Spalte -- fuer den Verlauf im Reiter.

    Leere Zellen werden uebersprungen: Validierungswerte stehen nur in den
    Zeilen, in denen auch validiert wurde.
    """
    series = []
    for row in rows:
        value = (row.get(key) or "").strip()
        if not value:
            continue
        try:
            series.append((int(float(row["step"])), float(value)))
        except (KeyError, TypeError, ValueError):
            continue
    return series


def suggest_output(workdir, dataset, root=CHECKPOINT_ROOT):
    """Vorschlag fuer den Zielordner eines Laufs: ``checkpoints/<Datensatz>``.

    Existiert er schon, wird durchnummeriert -- ein vorhandener Lauf wird
    nie stillschweigend ueberschrieben.
    """
    name = Path(dataset).name if dataset else "lauf"
    return _free_name(Path(workdir), root, name)


def suggest_dataset_name(workdir, sources, root=DATASET_ROOT):
    """Vorschlag fuer den Zielordner eines Exports, aus den Quellnamen."""
    if not sources:
        return Path(workdir) / root / "datensatz"
    first = Path(sources[0])
    name = first.name if len(sources) == 1 else "%s_und_%d_weitere" % (
        first.name, len(sources) - 1)
    return _free_name(Path(workdir), root, name)


def _free_name(workdir, root, name):
    """Freier Ordnername unter ``root`` -- ein vorhandener Lauf wird nie
    stillschweigend ueberschrieben. Rueckgabe relativ zum Arbeitsordner."""
    candidate = workdir / root / name
    counter = 2
    while candidate.exists():
        candidate = workdir / root / ("%s_%d" % (name, counter))
        counter += 1
    return relative(candidate, workdir)

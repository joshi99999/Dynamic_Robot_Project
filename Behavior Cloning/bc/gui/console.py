"""Modus "Konsole": Kommandozeilen aus Doku und Chat direkt ausfuehren -- Qt-frei.

Wunsch Anwender 2026-10-21: Die Befehle aus Doku und Absprachen (``python
tools/check_servo_stream.py --move ...``) sollen sich in der Oberflaeche
einfuegen und starten lassen, statt ein Terminal mit aktivierter Umgebung zu
oeffnen.

Bewusst eng gefasst:

* **Nur Python-Befehle.** Das fuehrende ``python`` wird durch den
  Interpreter der Oberflaeche ersetzt (``bc_env``) -- genau der, mit dem die
  Werkzeuge getestet sind. Andere Programme und Shell-Syntax (``|``, ``>``,
  ``&&``, ``cd``) gehen nicht: Es gibt keine Shell dazwischen, und ein
  stilles Ignorieren waere schlimmer als eine Meldung.
* **Eine Zeile je Start.** Arbeitsverzeichnis ist immer "Behavior Cloning"
  (wie in der Doku vorausgesetzt).
* **Bewegung wird benannt**: ``--move``, ``--real-robot`` und die Apps, die
  fahren, loesen eine Rueckfrage aus. Die Sicherheitssperren der Werkzeuge
  selbst (Simulationsabfrage, Freigabewort) bleiben unberuehrt -- das
  Freigabewort wird ueber die Eingabezeile geschickt.
"""

import re

#: Erste Woerter, die "den Python-Interpreter" meinen.
_PYTHON_WORDS = ("python", "python.exe", "python3", "py", "py.exe")

#: Shell-Syntax, die ohne Shell nichts tut (Platzhalter <...> vorher entfernt).
_SHELL = re.compile(r"(\|\||&&|[|<>;]|^\s*cd\b|^\s*set\b|^\s*export\b)")
#: Platzhalter aus der Doku, z. B. ``Berichte/<stempel>_Servo-Strom``.
_PLACEHOLDER = re.compile(r"<[^<>\s]+>")

#: Skripte/Optionen, die den Roboter bewegen (fuer die Rueckfrage).
_MOVES_SCRIPTS = ("apps/record.py", "apps/infer.py", "apps/teach.py", "apps/eval.py")


#: Beispiele zum Einfuegen -- dieselben Zeilen wie in CLAUDE.md und den
#: Uebergaben. (Beschriftung, Befehl)
EXAMPLES = (
    ("Tests (hardwarefrei)", "python tests/run_all.py"),
    ("Contract-Tests gegen die VM", "python tests/run_all.py --robot=neura contract.test_robot_contract"),
    ("VM pruefen, ohne Bewegung", "python tools/check_sim_robot.py"),
    ("Servo-Strom messen (VM, Override 0,5)",
     "python tools/check_servo_stream.py --move --laeufe linear-60-r0-o0.5,linear-60-r1-o0.5"),
    ("Servo-Strom neu auswerten", "python tools/check_servo_stream.py --auswerten Berichte/<stempel>_Servo-Strom"),
    ("Schleppfehler nachrechnen",
     'python tools/report_labortag_2026_10_16.py --mitschriebe "Berichte/2026-10-1*_Mitschrieb.npz"'),
    ("Episoden als PDF", 'python tools/plot_episode.py "data_vm/<datum>/<ordner>"'),
    ("Punkte sichern (nur lesen)", "python tools/export_points.py"),
    ("Kameras suchen", "python tools/check_cameras.py --list"),
)


class ParsedCommand(object):
    """Ergebnis von :func:`parse_command`."""

    def __init__(self, argv, problems, warnings, moves, real_robot):
        #: Argumente OHNE Interpreter (fuer ProcessRunner.start).
        self.argv = argv
        self.problems = problems
        self.warnings = warnings
        #: Bewegt der Befehl voraussichtlich einen Roboter?
        self.moves = moves
        #: Steht ``--real-robot`` darin?
        self.real_robot = real_robot

    @property
    def ok(self):
        return not self.problems


def _tokens(line):
    """Zerlegt wie die Windows-Eingabe: Anfuehrungszeichen gruppieren,
    Backslashes bleiben (Windows-Pfade)."""
    tokens, current, quoted, in_token = [], [], None, False
    for char in line:
        if quoted:
            if char == quoted:
                quoted = None
            else:
                current.append(char)
        elif char in "\"'":
            quoted, in_token = char, True
        elif char.isspace():
            if in_token:
                tokens.append("".join(current))
                current, in_token = [], False
        else:
            current.append(char)
            in_token = True
    if quoted:
        raise ValueError("Anfuehrungszeichen %s nicht geschlossen" % quoted)
    if in_token:
        tokens.append("".join(current))
    return tokens


def parse_command(text):
    """Eingefuegter Text -> :class:`ParsedCommand`.

    Leere Zeilen, Kommentare (``#``) und ein Prompt-Zeichen am Anfang
    (``$ ``, ``> ``) werden uebergangen.
    """
    lines = []
    for raw in (text or "").splitlines():
        line = raw.strip()
        for prompt in ("$ ", "> ", "PS> "):
            if line.startswith(prompt):
                line = line[len(prompt):].strip()
        if line and not line.startswith("#"):
            lines.append(line)
    if not lines:
        return ParsedCommand([], ["Kein Befehl eingegeben."], [], False, False)
    if len(lines) > 1:
        return ParsedCommand([], ["Mehrere Zeilen -- bitte einen Befehl je Start "
                                  "(%d gefunden)." % len(lines)], [], False, False)
    line = lines[0]
    placeholders = _PLACEHOLDER.findall(line)
    if placeholders:
        return ParsedCommand([], ["Platzhalter im Befehl: %s -- vorher durch den echten "
                                  "Wert ersetzen." % ", ".join(placeholders)], [], False, False)
    if _SHELL.search(line):
        return ParsedCommand([], ["Shell-Syntax (| > < ; && cd set) geht hier nicht -- "
                                  "es gibt keine Shell, Arbeitsordner ist immer "
                                  "\"Behavior Cloning\"."], [], False, False)
    try:
        tokens = _tokens(line)
    except ValueError as exc:
        return ParsedCommand([], [str(exc)], [], False, False)
    first = tokens[0].lower().replace("\\", "/").rsplit("/", 1)[-1]
    if first not in _PYTHON_WORDS:
        return ParsedCommand([], ["Nur Python-Befehle (\"python ...\") -- %r ist keiner."
                                  % tokens[0]], [], False, False)
    argv = tokens[1:]
    if not argv:
        return ParsedCommand([], ["Hinter \"python\" fehlt das Skript."], [], False, False)
    warnings = []
    if argv[0] == "-u":
        argv = argv[1:]   # setzt der Starter ohnehin
    real_robot = "--real-robot" in argv
    script = argv[0].replace("\\", "/") if argv else ""
    moves = "--move" in argv or any(script.endswith(s) for s in _MOVES_SCRIPTS)
    if real_robot:
        warnings.append("--real-robot: Der Befehl darf die REALE ANLAGE bewegen.")
    elif moves:
        warnings.append("Der Befehl bewegt den Roboter (VM oder, mit Freigabe, Anlage).")
    return ParsedCommand(argv, [], warnings, moves, real_robot)

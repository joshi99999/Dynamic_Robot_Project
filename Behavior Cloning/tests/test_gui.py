"""Qt-freie Schichten der Bedienoberflaeche (bc/gui).

Geprueft wird nur, was ohne Bildschirm und ohne PySide6 laeuft: die
Voraussetzungspruefung, der Pruefkatalog samt Auswertung und die
Backend-Anzeige der Sitzung. Genau dafuer ist der Schnitt so gelegt --
die Aussage "was haengt wirklich dran" darf nicht erst am Labortag zum
ersten Mal ausgefuehrt werden.

Die Qt-Teile (app, widgets, runner, tab_*) sind hier bewusst NICHT
importiert; sie brauchen PySide6 und einen Bildschirm.
"""

import _paths  # noqa: F401

from bc.gui import MODES, checks, requirements, session


# -- Voraussetzungen ------------------------------------------------------

def test_every_mode_has_requirements():
    for mode in MODES:
        assert mode in requirements.BY_MODE, mode
        report = requirements.report_for(mode)
        assert list(report), "Modus %s ohne Voraussetzungen" % mode


def test_missing_requirement_is_a_finding_not_a_crash():
    probe = requirements.module_probe("modul_das_es_nicht_gibt")
    requirement = requirements.Requirement(
        "x", "Test", requirements.REQUIRED, probe, "irgendwas installieren")
    result = requirement.check()
    assert result.ok is False
    assert "ModuleNotFoundError" in result.detail


def test_probe_that_raises_is_caught():
    def explode():
        raise RuntimeError("kaputt")

    requirement = requirements.Requirement(
        "x", "Test", requirements.REQUIRED, explode, "reparieren")
    result = requirement.check()
    assert result.ok is False
    assert "kaputt" in result.detail


def test_message_names_what_is_missing_and_what_still_works():
    missing = requirements.Requirement(
        "x", "Test", requirements.REQUIRED,
        lambda: (False, "nicht da"), "das Fehlende installieren")
    report = requirements.Report([missing.check()])
    text = report.message("Training starten", "Export geht trotzdem.")
    assert "Training starten" in text
    assert "das Fehlende installieren" in text
    assert "Export geht trotzdem." in text
    assert report.ok is False


def test_message_is_none_when_nothing_missing():
    present = requirements.Requirement(
        "x", "Test", requirements.REQUIRED, lambda: (True, "da"), "egal")
    report = requirements.Report([present.check()])
    assert report.ok is True
    assert report.message("Irgendwas") is None


# -- Pruefkatalog ---------------------------------------------------------

def test_catalogue_keys_are_unique():
    keys = [c.key for c in checks.catalogue()]
    assert len(keys) == len(set(keys))


def test_motion_checks_ask_before_they_move():
    for check in checks.catalogue():
        if check.level == checks.MOTION:
            assert check.confirm, "%s bewegt, fragt aber nicht" % check.key


def test_free_checks_never_touch_the_controller():
    # Ein hardwarefreier Punkt darf den Roboteradapter nicht anfordern.
    for check in checks.catalogue():
        if check.level == checks.FREE:
            assert "--robot=neura" not in check.argv, check.key
            assert "--real-robot" not in check.argv, check.key


def test_every_needed_key_is_declared_in_the_systemcheck_requirements():
    # Sonst wird ein Punkt stumm uebersprungen, weil die Voraussetzung, auf
    # die er sich beruft, in diesem Modus gar nicht geprueft wird -- und die
    # Meldung dazu bliebe leer.
    declared = set(r.key for r in requirements.systemcheck_requirements())
    for check in checks.catalogue():
        for key in check.needs:
            assert key in declared, "%s braucht '%s', das im Modus fehlt" % (check.key, key)


def test_by_level_covers_the_whole_catalogue():
    grouped = sum((items for _level, items in checks.by_level()), [])
    assert len(grouped) == len(checks.catalogue())


def test_outcome_reads_the_run_all_summary():
    text = ("== test_noise ==\n"
            "123 Tests bestanden, 2 fehlgeschlagen\n"
            "  FAIL test_noise::test_a\n"
            "  FAIL test_rectify::test_b\n")
    outcome = checks.Outcome(checks.find("unittests"), 1, text)
    assert outcome.passed == 123
    assert outcome.failed == 2
    assert outcome.failures == ["test_noise::test_a", "test_rectify::test_b"]
    assert outcome.ok is False
    assert "fehlgeschlagen" in outcome.summary()


def test_outcome_without_summary_line_uses_the_returncode():
    ok = checks.Outcome(checks.find("kinematics"), 0, "alles in Ordnung\n")
    assert ok.ok is True
    assert ok.passed is None

    hinted = checks.Outcome(checks.find("kinematics"), 0, "Traceback (most recent call last):\n")
    assert "Befunde" in hinted.summary()

    failed = checks.Outcome(checks.find("kinematics"), 2, "\n")
    assert failed.ok is False


# -- Backend-Anzeige ------------------------------------------------------

class _FakeRobot(object):
    """Stellvertreter fuer NeuraRobot -- nur die Felder der Anzeige."""

    def __init__(self, in_simulation, motion_allowed):
        self.in_simulation = in_simulation
        self.motion_allowed = motion_allowed
        self.tool_name = "NoTool"
        self.gripper_mode = "GRIPPER_LOGGED"
        self.closed = False

    def close(self):
        self.closed = True


def _neura_session(in_simulation, motion_allowed):
    s = session.Session(backend=session.BACKEND_NEURA)
    s.robot = _FakeRobot(in_simulation, motion_allowed)
    return s


def test_status_is_offline_before_connecting():
    status = session.Session().status()
    assert status.state == session.OFFLINE
    assert status.is_real_plant is False


def test_confirmed_simulation_is_not_flagged_as_the_plant():
    status = _neura_session(True, True).status()
    assert status.state == session.SIM_CONTROLLER
    assert status.is_real_plant is False


def test_controller_denying_simulation_is_the_plant():
    # Bewegung freigegeben und KEINE Simulation -> das ist die Anlage.
    status = _neura_session(False, True).status()
    assert status.state == session.REAL
    assert status.is_real_plant is True


def test_unconfirmed_simulation_blocks_instead_of_guessing():
    status = _neura_session(False, False).status()
    assert status.state == session.BLOCKED
    assert status.motion_allowed is False


def test_failed_simulation_query_is_not_treated_as_simulation():
    status = _neura_session(None, False).status()
    assert status.state == session.UNKNOWN
    assert status.is_real_plant is False


def test_every_status_has_text_and_colour():
    for state in (session.OFFLINE, session.SIM_LOCAL, session.SIM_CONTROLLER,
                  session.REAL, session.BLOCKED, session.UNKNOWN):
        assert session.STATUS_TEXT[state]
        assert session.STATUS_COLOUR[state].startswith("#")


def test_only_the_plant_is_red():
    # Rot bleibt der Anlage vorbehalten, sonst verliert die Farbe am
    # Labortag ihre Bedeutung.
    red = [s for s, c in session.STATUS_COLOUR.items() if c == "#c62828"]
    assert red == [session.REAL]


def test_switching_backend_disconnects():
    s = _neura_session(True, True)
    robot = s.robot
    s.set_backend(session.BACKEND_SIM)
    assert robot.closed is True
    assert s.robot is None


def test_allowing_the_plant_forces_a_reconnect():
    # Die Freigabe wirkt erst beim naechsten Verbinden -- sonst waere eine
    # bestehende Verbindung ploetzlich mehr wert als beim Aufbau geprueft.
    s = _neura_session(False, False)
    robot = s.robot
    s.set_allow_real(True)
    assert robot.closed is True
    assert s.robot is None
    assert s.allow_real is True

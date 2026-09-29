"""Qt-freie Schichten der Bedienoberflaeche (bc/gui).

Geprueft wird nur, was ohne Bildschirm und ohne PySide6 laeuft: die
Voraussetzungspruefung, der Pruefkatalog samt Auswertung und die
Backend-Anzeige der Sitzung. Genau dafuer ist der Schnitt so gelegt --
die Aussage "was haengt wirklich dran" darf nicht erst am Labortag zum
ersten Mal ausgefuehrt werden.

Die Qt-Teile (app, widgets, runner, tab_*) sind hier bewusst NICHT
importiert; sie brauchen PySide6 und einen Bildschirm.
"""

import json

import _paths  # noqa: F401

from bc import config
from bc import preview as bc_preview
from bc.gui import (MODES, cameras, checks, operation, recording, requirements,
                    session, training)


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


# -- Modus "Training" -----------------------------------------------------

def _recording(tmp, name, episodes=2, rate=15.0, schema=None, lengths=None):
    """Legt eine Aufzeichnung mit index.json an (nur so viel, wie gelesen wird)."""
    root = tmp / name
    root.mkdir(parents=True)
    lengths = lengths or [10] * episodes
    (root / "index.json").write_text(json.dumps({
        "schema_version": config.SCHEMA_VERSION if schema is None else schema,
        "rate_hz": rate,
        "cameras": ["wrist", "scene"],
        "episodes": [{"dir": "ep_%05d" % i, "length": lengths[i], "discarded": False}
                     for i in range(episodes)],
    }), encoding="utf-8")
    return root


def test_recordings_are_found_one_and_two_levels_deep(tmp_path):
    # data_vm/<Datum>/<Lauf>, aber data_sim/<Lauf> -- beide muessen auftauchen.
    _recording(tmp_path / "data_vm" / "2026-09-17", "1_lauf")
    _recording(tmp_path / "data_sim", "durchstich")
    found = [str(p) for p in training.find_recordings(tmp_path)]
    assert len(found) == 2
    assert any("2026-09-17" in p for p in found)
    assert any("durchstich" in p for p in found)


def test_found_paths_are_relative_to_the_workdir(tmp_path):
    # Die Kommandozeile soll die sein, die auch im Terminal gilt -- der
    # Unterprozess laeuft im Ordner "Behavior Cloning".
    _recording(tmp_path / "data_sim", "lauf")
    found = training.find_recordings(tmp_path)
    assert not found[0].is_absolute()
    assert str(found[0]).startswith("data_sim")


def test_recording_label_names_a_foreign_schema(tmp_path):
    root = _recording(tmp_path / "data_sim", "alt", schema=config.SCHEMA_VERSION - 1)
    label = training.recording_label(training.describe_recording(root))
    assert "nicht mischen" in label


def test_unreadable_recording_is_a_finding_not_a_crash(tmp_path):
    root = tmp_path / "data_sim" / "kaputt"
    root.mkdir(parents=True)
    (root / "index.json").write_text("{kein json", encoding="utf-8")
    described = training.describe_recording(root)
    assert described["error"]
    assert "unlesbar" in training.recording_label(described)


def _dataset(tmp, name, info, export=None):
    root = tmp / "datasets" / name
    (root / "meta").mkdir(parents=True)
    (root / "meta" / "info.json").write_text(json.dumps(info), encoding="utf-8")
    if export is not None:
        (root / "meta" / "bc_export.json").write_text(json.dumps(export), encoding="utf-8")
    return root


def test_dataset_with_another_lerobot_version_is_flagged(tmp_path):
    root = _dataset(
        tmp_path, "fremd",
        {"total_episodes": 3, "total_frames": 30, "fps": config.CONTROL_RATE_HZ},
        {"lerobot_version": "0.5.0", "schema_version": config.SCHEMA_VERSION,
         "cameras": ["wrist"]})
    assert any("lerobot" in w for w in training.describe_dataset(root)["warnings"])


def test_dataset_without_export_metadata_says_the_origin_is_unknown(tmp_path):
    root = _dataset(tmp_path, "vonwoanders",
                    {"total_episodes": 1, "total_frames": 5,
                     "fps": config.CONTROL_RATE_HZ})
    described = training.describe_dataset(root)
    assert described["foreign"] is True
    assert any("Herkunft" in w for w in described["warnings"])


def test_wrong_rate_is_flagged(tmp_path):
    root = _dataset(
        tmp_path, "schnell",
        {"total_episodes": 1, "total_frames": 5, "fps": 30.0},
        {"lerobot_version": config.LEROBOT_VERSION,
         "schema_version": config.SCHEMA_VERSION})
    assert any("Hz" in w for w in training.describe_dataset(root)["warnings"])


def test_only_deviations_end_up_in_the_command_line():
    values = dict((p.flag, p.default) for p in training.parameters())
    argv = training.train_argv("datasets/x", "checkpoints/y", values=values)
    assert argv == ["apps/train.py", "--dataset", "datasets/x", "--out", "checkpoints/y"]

    values["--steps"] = 8000
    argv = training.train_argv("datasets/x", "checkpoints/y", values=values)
    assert argv[-2:] == ["--steps", "8000"]


def test_device_auto_is_not_written_out():
    # apps/train.py prueft bei "auto" zusaetzlich die GPU-Architektur --
    # ein ausgeschriebenes "--device auto" waere nur Rauschen in der Zeile.
    argv = training.train_argv("d", "o", device="auto", values={})
    assert "--device" not in argv
    argv = training.train_argv("d", "o", device="cuda:1", values={})
    assert argv[argv.index("--device") + 1] == "cuda:1"


def test_config_defaults_are_marked_as_settled():
    settled = dict((p.flag, p) for p in training.parameters() if p.is_locked_down)
    assert settled, "Die Festlegungen aus bc/config.py fehlen"
    assert settled["--horizon"].default == config.POLICY_HORIZON
    assert settled["--n-obs-steps"].default == config.POLICY_N_OBS_STEPS
    assert settled["--prediction-type"].default == config.POLICY_PREDICTION_TYPE


def test_export_needs_a_source_and_a_target():
    for call in (lambda: training.export_argv([], out="datasets/x"),
                 lambda: training.export_argv(["data_sim/x"])):
        try:
            call()
        except ValueError:
            continue
        raise AssertionError("fehlende Angabe haette auffallen muessen")
    # Nur pruefen braucht kein Ziel -- es wird ja nichts geschrieben.
    assert "--check" in training.export_argv(["data_sim/x"], check=True)


def test_progress_reads_the_real_training_output():
    progress = training.TrainingProgress(total_steps=8000)
    progress.feed('Hardware: {"device": "cuda", "torch": "2.11.0+cu128", '
                  '"gpu": "NVIDIA GeForce RTX 5070 Ti", "vram_gb": 15.9}')
    progress.feed("Datensatz datasets\\vm: 30 Episoden (27 Training, 3 Validierung), "
                  "4752 Frames, Kameras ['wrist', 'scene']")
    progress.feed("Parameter: 89.5 M, effektive Batch 64, Worker 4")
    progress.feed("step   1000  loss 0.0033  lr 9.9e-05  grad 0.19  data 0.068s  "
                  "upd 0.166s  mem 5.6 GB  4.7 min (Rest ~33)  | val loss 0.0194  "
                  "Gelenk-MAE 0.0251 rad (p95 0.0922)  Greifer 98.6 %  "
                  "Chunk-Sprung p95 0.1640 rad (Label 0.0257)")
    assert progress.step == 1000
    assert progress.percent == 12          # abgerundet: 1000 von 8000
    assert abs(progress.loss - 0.0033) < 1e-9
    assert abs(progress.val_mae_rad - 0.0251) < 1e-9
    assert abs(progress.val_p95_rad - 0.0922) < 1e-9
    assert abs(progress.val_gripper_pct - 98.6) < 1e-9
    assert progress.eta_min == 33.0
    assert progress.episodes == 30 and progress.frames == 4752
    assert progress.effective_batch == 64
    assert "5070" in progress.hardware_line()
    assert "0.0251 rad" in progress.headline()


def test_progress_survives_lines_it_does_not_know():
    progress = training.TrainingProgress(total_steps=100)
    assert progress.feed("irgendein Hinweis von lerobot") is False
    assert progress.feed("") is False
    assert progress.step == 0
    assert progress.headline()          # kein Absturz ohne Zahlen


def test_validation_series_skips_rows_without_a_value(tmp_path):
    log = tmp_path / "train_log.csv"
    log.write_text("step,loss,val_joint_mae_rad\n"
                   "100,0.5,\n"
                   "200,0.4,0.02\n"
                   "300,0.3,\n", encoding="utf-8")
    fields, rows = training.read_train_log(log)
    assert "val_joint_mae_rad" in fields and len(rows) == 3
    assert training.val_series(rows) == [(200, 0.02)]


def test_missing_train_log_is_empty_not_an_error(tmp_path):
    assert training.read_train_log(tmp_path / "gibtsnicht.csv") == ([], [])


def test_suggested_targets_never_overwrite_an_existing_run(tmp_path):
    (tmp_path / "checkpoints" / "vm").mkdir(parents=True)
    assert training.suggest_output(tmp_path, "datasets/vm").name == "vm_2"


def test_checkpoint_without_training_state_is_not_resumable(tmp_path):
    root = tmp_path / "checkpoints" / "lauf"
    (root / "checkpoints" / "step_004000").mkdir(parents=True)
    (root / "train_config.json").write_text(
        json.dumps({"args": {"dataset": "datasets/vm", "steps": 8000}}),
        encoding="utf-8")
    described = training.describe_checkpoint(root)
    assert described["checkpoints"] == ["step_004000"]
    assert described["resumable"] is None


def test_ffmpeg_next_to_the_interpreter_counts_as_present():
    # Der Aufnahme-Laptop startet die GUI ueber python.exe, ohne die
    # Umgebung zu aktivieren -- dann steht Library\bin nicht im PATH,
    # ffmpeg liegt aber daneben. ProcessRunner ergaenzt den Ordner, und
    # die Voraussetzungszeile darf deshalb nicht "fehlt" melden.
    dirs = requirements.interpreter_tool_dirs()
    assert dirs, "kein Ordner neben dem Interpreter gefunden"
    assert all(d.is_dir() for d in dirs)


# -- Kamerazuordnung in der Oberflaeche -----------------------------------

def test_placeholder_is_offered_for_every_slot():
    # Wunsch des Anwenders: unter "Simulation" ohne Daheng-SDK arbeiten
    # koennen, und Bilder simulieren duerfen.
    inventory = cameras.Inventory()
    for slot in cameras.SLOTS:
        keys = [d.key for d in inventory.for_slot(slot)]
        assert "sim" in keys


def test_default_assignment_is_placeholders_without_a_controller():
    assignment = cameras.default_assignment(cameras.Inventory(),
                                            backend_is_sim_robot=True)
    assert set(assignment) == set(cameras.SLOTS)
    assert cameras.all_simulated(assignment)


def test_default_assignment_never_silently_picks_index_zero():
    # Auf einem Laptop waere OpenCV-Index 0 die eingebaute Webcam; eine
    # stillschweigende Vorbelegung darauf faellt im Datensatz nicht auf.
    inventory = cameras.Inventory()
    assignment = cameras.default_assignment(inventory, backend_is_sim_robot=False)
    assert cameras.all_simulated(assignment)


def test_specs_always_name_both_slots():
    specs = cameras.to_specs({"wrist": "uvc:1", "scene": "sim"})
    assert specs == ["wrist=uvc:1", "scene=sim"]


def test_assignment_becomes_camera_configs_in_dataset_order():
    configs = cameras.to_configs({"wrist": "sim", "scene": "uvc:2"})
    assert [c.name for c in configs] == list(cameras.SLOTS)
    assert configs[1].backend == "uvc" and configs[1].device == 2


def test_placeholders_need_no_sdk_at_all():
    # Der Kern des Wunsches: keine Voraussetzung, solange nichts Echtes dranhaengt.
    assert cameras.required_backend_keys({"wrist": "sim", "scene": "sim"}) == []


def test_a_webcam_needs_opencv_but_not_the_galaxy_sdk():
    keys = cameras.required_backend_keys({"wrist": "sim", "scene": "uvc:1"})
    assert keys == ["opencv"]


def test_a_daheng_needs_the_galaxy_sdk():
    keys = cameras.required_backend_keys({"wrist": "daheng:SN", "scene": "sim"})
    assert "gxipy" in keys and "opencv" in keys


def test_placeholders_at_the_real_plant_are_named():
    warnings = cameras.assignment_warnings(
        {"wrist": "sim", "scene": "sim"}, is_real_plant=True)
    assert any("Testdaten" in w for w in warnings)


def test_placeholders_in_simulation_are_not_complained_about():
    warnings = cameras.assignment_warnings(
        {"wrist": "sim", "scene": "sim"}, is_real_plant=False)
    assert warnings == []


def test_mixing_placeholder_and_real_camera_is_flagged():
    warnings = cameras.assignment_warnings({"wrist": "uvc:1", "scene": "sim"})
    assert any("gemischten Backends" in w for w in warnings)


def test_a_device_that_vanished_is_named_not_swallowed():
    inventory = cameras.Inventory()          # enthaelt nur den Platzhalter
    warnings = cameras.assignment_warnings(
        {"wrist": "daheng:SN", "scene": "sim"}, inventory)
    assert any("nicht gefunden" in w for w in warnings)


# -- Modus "Betrieb" -------------------------------------------------------

def _policy(tmp, **overrides):
    info = {
        "schema_version": config.SCHEMA_VERSION,
        "rate_hz": config.CONTROL_RATE_HZ,
        "cameras": ["wrist", "scene"],
        "lerobot_version": config.LEROBOT_VERSION,
        "recording": {"robot": "neura", "in_simulation": True, "override": 1.0,
                      "servo_rate_hz": 60.0,
                      "camera_backends": {"wrist": "daheng", "scene": "uvc"}},
        "training": {"step": 8000, "steps_planned": 8000},
        "dataset": {"path": "datasets/vm", "episodes": 30, "frames": 5280},
        "inference_defaults": {"replan_steps": 2, "ensemble_decay": 0.0},
    }
    info.update(overrides)
    root = tmp / "checkpoints" / "lauf" / "policy"
    root.mkdir(parents=True)
    (root / "bc_policy.json").write_text(json.dumps(info), encoding="utf-8")
    return tmp / "checkpoints" / "lauf"


def test_a_run_and_its_intermediate_steps_are_both_offered(tmp_path):
    run = _policy(tmp_path)
    step = run / "checkpoints" / "step_004000"
    step.mkdir(parents=True)
    (step / "bc_policy.json").write_text("{}", encoding="utf-8")
    found = [str(p) for p in operation.find_policies(tmp_path)]
    assert any(p.endswith("lauf") for p in found)
    assert any("step_004000" in p for p in found)


def test_matching_setup_has_nothing_to_complain_about(tmp_path):
    described = operation.describe_policy(_policy(tmp_path))
    problems = operation.compatibility(
        described, {"wrist": "daheng:SN", "scene": "uvc:0"}, "neura", 1.0)
    assert operation.blocking(problems) == []


def test_different_camera_backend_is_blocking(tmp_path):
    described = operation.describe_policy(_policy(tmp_path))
    problems = operation.compatibility(
        described, {"wrist": "sim", "scene": "sim"}, "neura", 1.0)
    assert any("Kamera-Backends" in p.text for p in operation.blocking(problems))


def test_different_override_is_blocking_because_the_tempo_is_learned(tmp_path):
    described = operation.describe_policy(_policy(tmp_path))
    problems = operation.compatibility(
        described, {"wrist": "daheng:SN", "scene": "uvc:0"}, "neura", 0.2)
    assert any("Override" in p.text for p in operation.blocking(problems))


def test_a_missing_camera_slot_is_blocking(tmp_path):
    described = operation.describe_policy(_policy(tmp_path))
    problems = operation.compatibility(
        described, {"wrist": "daheng:SN"}, "neura", 1.0)
    assert any("erwartet die Kamera" in p.text for p in operation.blocking(problems))


def test_a_simulation_model_at_the_plant_is_a_hint_not_a_block(tmp_path):
    described = operation.describe_policy(_policy(tmp_path))
    problems = operation.compatibility(
        described, {"wrist": "daheng:SN", "scene": "uvc:0"}, "neura", 1.0,
        is_real_plant=True)
    assert operation.blocking(problems) == []
    assert any("REALE ANLAGE" in p.text for p in problems)


def test_a_folder_without_bc_policy_is_named_as_such(tmp_path):
    (tmp_path / "leer").mkdir()
    described = operation.describe_policy(tmp_path / "leer")
    assert described["found"] is False
    problems = operation.compatibility(described)
    assert len(operation.blocking(problems)) == 1


def test_hold_mode_needs_no_checkpoint():
    argv = operation.infer_argv(None, "sim", 1, hold=True)
    assert "--hold" in argv and "--checkpoint" not in argv


def test_inference_without_a_model_is_refused():
    try:
        operation.infer_argv(None, "sim", 1)
    except ValueError:
        return
    raise AssertionError("fehlendes Modell haette auffallen muessen")


def test_camera_assignment_reaches_the_command_line():
    argv = operation.infer_argv("checkpoints/x", "neura", 2,
                                camera_specs=["wrist=sim", "scene=uvc:1"])
    assert argv.count("--camera") == 2
    assert "scene=uvc:1" in argv


def test_rollout_line_is_read_completely():
    progress = operation.RolloutProgress(total_episodes=2)
    progress.feed("  Geraet cuda:0, trainiert 8000 Schritte auf NVIDIA GeForce RTX 5070 Ti, "
                  "Kameras ['wrist', 'scene']")
    progress.feed("  Vorhersage nach Aufwaermen: 75.8 ms (10 DDIM-Schritte)")
    progress.feed("Fahrt: bis 194 Takte, Tischhoehe 0.282 m, Chunks gemittelt, "
                  "neu alle 2 Takte, decay 0.00, Vorhersage synchron, Protokoll x")
    progress.feed("Fahrt 0: ERFOLG -- 149 Takte, Greifpunkt 2.2 mm, Ende 6.2 mm, "
                  "Bahn p95 3.8 mm, endstellung, Vorhersage 76.3 ms, Overruns 0, gekappt 0")
    assert len(progress.rollouts) == 1
    rollout = progress.rollouts[0]
    assert rollout.success is True and rollout.steps == 149
    assert abs(rollout.grasp_mm - 2.2) < 1e-9
    assert rollout.stop_reason == "endstellung"
    assert progress.warmup_ms == 75.8 and progress.ddim_steps == 10
    assert progress.percent == 50


def test_ticks_move_the_bar_inside_an_episode():
    # Eine Fahrt dauert bei 15 Hz gut zehn Sekunden -- ohne die Takt-Zeilen
    # stuende der Balken solange still.
    progress = operation.RolloutProgress(total_episodes=1)
    progress.feed("Fahrt: bis 200 Takte, Tischhoehe 0.282 m, x, y, z")
    assert progress.percent == 0
    progress.feed("  Takt  100  Spread 0.0072 rad  Vorhersage 71.4 ms")
    assert progress.percent == 50
    assert progress.predict_ms == 71.4


def test_a_failed_run_is_read_as_such():
    progress = operation.RolloutProgress(total_episodes=1)
    progress.feed("Fahrt 0: kein Erfolg -- 194 Takte, Greifpunkt -, Ende -, "
                  "Bahn p95 12.0 mm, max_steps, Vorhersage - ms, Overruns 4, gekappt 7")
    rollout = progress.rollouts[0]
    assert rollout.success is False
    assert rollout.grasp_mm is None and rollout.predict_ms is None
    assert rollout.overruns == 4 and rollout.clamped == 7
    assert "4 Takt-Überläufe" in progress.totals_line()


def test_progress_survives_unknown_lines():
    progress = operation.RolloutProgress(total_episodes=1)
    assert progress.feed("Loading weights from local directory") is False
    assert progress.headline()


def test_slow_prediction_is_named_against_the_cycle_time():
    progress = operation.RolloutProgress()
    progress.feed("  Vorhersage nach Aufwaermen: 384.0 ms (10 DDIM-Schritte)")
    assert "über" in progress.detail_line()
    progress = operation.RolloutProgress()
    progress.feed("  Vorhersage nach Aufwaermen: 20.0 ms (10 DDIM-Schritte)")
    assert "unter" in progress.detail_line()


# -- Modus "Aufnahme" ------------------------------------------------------

def _sequence(tmp, name="pick.json"):
    root = tmp / "sequences"
    root.mkdir(parents=True, exist_ok=True)
    path = root / name
    path.write_text(json.dumps({
        "name": "pick_to_station",
        "gripper_start": "open",
        "sequence": [
            {"point": "CLEAR_FOV", "motion": "ptp"},
            {"point": "APPROACH_02", "motion": "ptp", "optional": True, "blend": 0.05},
            {"point": "PRE_GRASP", "motion": "ptp", "blend": 0.01},
            {"point": "PICK", "motion": "lin", "approach": True, "gripper": "close"},
        ],
    }), encoding="utf-8")
    return path


def test_sequence_points_and_blends_are_read(tmp_path):
    described = recording.describe_sequence(_sequence(tmp_path))
    assert described["points"] == ["CLEAR_FOV", "APPROACH_02", "PRE_GRASP", "PICK"]
    assert described["optional"] == ["APPROACH_02"]
    assert any("10 mm" in b for b in described["blends"])
    assert described["gripper_start"] == "open"


def test_sequence_summary_says_the_points_come_from_the_controller(tmp_path):
    summary = recording.sequence_summary(
        recording.describe_sequence(_sequence(tmp_path)))
    assert "Touch-up" in summary


def test_unreadable_sequence_is_a_finding(tmp_path):
    path = tmp_path / "kaputt.json"
    path.write_text("{kein json", encoding="utf-8")
    assert recording.describe_sequence(path)["error"]


def test_the_plant_needs_a_block_and_a_sequence():
    missing = recording.plant_requirements(None, None)
    assert len(missing) == 2
    assert any("Block" in m for m in missing)
    assert recording.plant_requirements("B07", "sequences/x.json") == []


def test_camera_assignment_reaches_the_record_command():
    argv = recording.record_argv("data_sim/x", "sim", 2,
                                 camera_specs=["wrist=sim", "scene=uvc:1"])
    assert argv.count("--camera") == 2
    assert "scene=uvc:1" in argv


def test_live_preview_is_off_unless_asked_for():
    argv = recording.record_argv("data_sim/x", "sim", 1)
    assert "--preview" not in argv
    argv = recording.record_argv("data_sim/x", "sim", 1, preview="live.jpg",
                                 preview_hz=5)
    assert argv[argv.index("--preview") + 1] == "live.jpg"
    assert argv[argv.index("--preview-hz") + 1] == "5"


def test_recording_needs_a_target():
    try:
        recording.record_argv(None, "sim", 1)
    except ValueError:
        return
    raise AssertionError("fehlendes Ziel haette auffallen muessen")


def test_metadata_reaches_the_command_line():
    argv = recording.record_argv("data_vm/x", "neura", 3, block="B07",
                                 light="Decke an", camera_pose="nominal")
    assert argv[argv.index("--block") + 1] == "B07"
    assert argv[argv.index("--light") + 1] == "Decke an"
    assert argv[argv.index("--camera-pose") + 1] == "nominal"


def test_target_follows_the_agreed_layout(tmp_path):
    # data_vm/<Datum>/<Lauf> fuer die VM, data_sim/... fuer den SimRobot.
    assert str(recording.suggest_out(tmp_path, "neura")).startswith("data_vm")
    assert str(recording.suggest_out(tmp_path, "sim")).startswith("data_sim")


def test_record_output_is_read_completely():
    progress = recording.RecordProgress(total_episodes=2)
    progress.feed("Lauf-Einstellungen: servo_j 60 Hz, Rauschen je Episode "
                  "zufaellig bis x1.00, Seed 4711, Ablauf Demo")
    progress.feed("Soll-Bahn: 129 Schritte (8.6 s bei 15 Hz), 16 Dwell-Schritte")
    progress.feed("  Takt   60 von 129  Versatz   2 ms  ausserhalb Budget 0")
    assert progress.seed == 4711
    assert progress.steps_per_episode == 129
    assert progress.percent == 23
    progress.feed("Episode 0 -> ep_00000: 129 Schritte, ok")
    assert len(progress.episodes) == 1
    assert progress.episodes[0].discarded is False
    assert progress.percent == 50


def test_a_discarded_episode_is_marked():
    progress = recording.RecordProgress(total_episodes=1)
    progress.feed("Episode 0 -> ep_00000: 41 Schritte, VERWORFEN (schutzstopp)")
    assert progress.episodes[0].discarded is True
    assert "VERWORFEN" in progress.episodes[0].row()[3]


def test_skew_above_the_budget_is_named():
    progress = recording.RecordProgress()
    progress.feed("  Takt   10 von 100  Versatz  45 ms  ausserhalb Budget 3")
    assert "über" in progress.detail_line()
    assert "3 Takte" in progress.detail_line()


def test_final_counts_are_read():
    progress = recording.RecordProgress(total_episodes=3)
    for line in ("  Episoden gesamt   : 3",
                 "  davon verwendbar  : 2",
                 "  davon verworfen   : 1"):
        progress.feed(line)
    assert progress.headline() == "3 Episoden, 2 verwendbar, 1 verworfen"


def test_progress_survives_unknown_lines():
    progress = recording.RecordProgress(total_episodes=1)
    assert progress.feed("Kollisionsmodell: Tischhoehe z = 0.282 m") is False
    assert progress.headline()


# -- Livebild (bc/preview.py) ---------------------------------------------

class _Clock(object):
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t


def test_preview_writes_at_most_at_the_configured_rate(tmp_path):
    # Der Sinn der Begrenzung: die Aufnahmeschleife hat 1/15 s je Takt.
    clock = _Clock()
    writer = bc_preview.PreviewWriter(tmp_path / "live.jpg", clock, rate_hz=5.0)
    due = 0
    for tick in range(15):                      # eine Sekunde bei 15 Hz
        clock.t = tick / 15.0
        if writer.due():
            due += 1
    assert due == 5


def test_preview_never_breaks_the_recording(tmp_path):
    # Ein Livebild ist eine Bequemlichkeit, die Episode ist die Arbeit:
    # Ein Schreibfehler darf nie nach oben durchschlagen.
    blocked = tmp_path / "live.jpg"
    blocked.mkdir()                       # Ordner statt Datei -> Schreiben scheitert
    writer = bc_preview.PreviewWriter(blocked, _Clock(), rate_hz=5.0)
    assert writer.offer({"wrist": None}) is False
    assert writer.failures == 1
    assert writer.written == 0

    # Ein Objekt ohne .image ist kein Fehler, sondern ein leeres Feld.
    good = bc_preview.PreviewWriter(tmp_path / "ok.jpg", _Clock(), rate_hz=5.0)
    assert good.offer({"wrist": object()}) is True
    assert good.failures == 0 and (tmp_path / "ok.jpg").is_file()


def test_preview_off_means_never_due(tmp_path):
    writer = bc_preview.PreviewWriter(tmp_path / "live.jpg", _Clock(), rate_hz=0)
    assert writer.due() is False
    assert writer.offer({"wrist": None}) is False


def test_missing_frame_is_drawn_not_dropped():
    # Dass eine Kamera nichts liefert, ist die wichtigste Information, die
    # eine Vorschau geben kann.
    board = bc_preview.compose({"wrist": None, "scene": None}, height=60)
    assert board is not None
    assert board.shape[0] == 60
    assert board.shape[1] > 60          # zwei Felder nebeneinander


def test_compose_without_cameras_is_none():
    assert bc_preview.compose({}) is None


def test_a_sequence_against_the_simrobot_is_refused_before_the_start():
    # Der Fall aus dem Betrieb: Ziel stand auf "Simulation (ohne Controller)",
    # die Ablaufdatei war gewaehlt -- record.py brach mit "Pflichtpunkte
    # fehlen ... (vorhanden: )" ab, und aus der leeren Klammer war die
    # Ursache nicht zu erraten. Die Punkte stehen in der Control-Box.
    problems = recording.setup_problems("sim", "sequences/pick_to_station.json")
    assert len(problems) == 1
    assert "Steuerung" in problems[0]


def test_the_same_sequence_against_the_controller_is_fine():
    assert recording.setup_problems("neura", "sequences/pick_to_station.json") == []


def test_the_simrobot_without_a_sequence_is_fine():
    # Demo-Wegpunkte um die Home-Pose -- das ist der hardwarefreie Durchlauf.
    assert recording.setup_problems("sim", None) == []


def test_the_target_folder_follows_the_robot(tmp_path):
    # VM-Daten nach data_vm/<Datum>/, Sim-Daten nach data_sim/
    # (Festlegung 2026-09-16) -- sonst zeigen die Berichte ins Leere.
    assert str(recording.suggest_out(tmp_path, "neura")).startswith("data_vm")
    assert str(recording.suggest_out(tmp_path, "sim")).startswith("data_sim")
    assert str(recording.suggest_out(tmp_path, "neura")) != \
        str(recording.suggest_out(tmp_path, "sim"))


# -- Episoden je Aufruf ----------------------------------------------------

def test_more_than_one_episode_at_the_plant_is_named():
    # Grund: Die Bahn endet am Uebergabepunkt, und zu Beginn der naechsten
    # Episode sendet der Recorder den Greifer-Startzustand ("auf") -- das
    # Objekt laege dann nicht mehr am Greifpunkt.
    advice = recording.episode_advice(True, 3)
    assert advice and "EINE" in advice


def test_one_episode_at_the_plant_needs_no_hint():
    assert recording.episode_advice(True, 1) is None


def test_several_episodes_are_fine_without_a_physical_object():
    # SimRobot und virtuelle Steuerung: kein Objekt im Spiel.
    assert recording.episode_advice(False, 20) is None


# -- Bewertung je Episode (AP 5.2) ----------------------------------------

def test_the_label_question_is_recognised():
    assert recording.label_question(
        "BEWERTUNG Episode 3 (129 Schritte) -- [e]rfolgreich, [f]ehlgeschlagen, "
        "[v]erwerfen") == (3, 129)


def test_an_ordinary_line_is_not_a_question():
    assert recording.label_question("Episode 0 -> ep_00000: 129 Schritte, ok") is None
    assert recording.label_question("") is None


def test_every_answer_has_a_character_for_stdin():
    # Die Oberflaeche schickt ein Zeichen auf stdin -- record.py muss genau
    # diese Zeichen verstehen, sonst bleibt der Lauf stehen.
    import sys as _sys
    from pathlib import Path as _Path

    apps = str(_Path(__file__).resolve().parent.parent / "apps")
    if apps not in _sys.path:
        _sys.path.insert(0, apps)
    import record

    for answer, char in recording.LABEL_ANSWERS.items():
        assert char in record.LABEL_ANSWERS, answer
    assert record.LABEL_PROMPT == recording.LABEL_PROMPT
    # Und die Zeile, die record.py schreibt, muss die GUI wiedererkennen.
    assert recording.label_question(
        "%s Episode 0 (12 Schritte) -- [e]rfolgreich, [f]ehlgeschlagen, "
        "[v]erwerfen" % record.LABEL_PROMPT) == (0, 12)


def test_asking_is_a_flag_not_the_default():
    assert "--ask-label" not in recording.record_argv("data_sim/x", "sim", 1)
    assert "--ask-label" in recording.record_argv("data_sim/x", "sim", 1,
                                                  ask_label=True)


# -- Ablauf bearbeiten: Arbeitskopie --------------------------------------

def _sequence_file(tmp, name="pick.json"):
    root = tmp / "sequences"
    root.mkdir(parents=True, exist_ok=True)
    path = root / name
    path.write_text(json.dumps({
        "name": "pick_to_station",
        "gripper_start": "open",
        "sequence": [
            {"point": "CLEAR_FOV", "motion": "ptp"},
            {"point": "PRE_GRASP", "motion": "ptp", "blend": 0.01},
            {"point": "PICK", "motion": "lin", "approach": True, "gripper": "close"},
            {"point": "PRE_PLACE", "motion": "ptp"},
        ],
    }), encoding="utf-8")
    return path


def test_the_original_is_never_touched(tmp_path):
    # Festlegung Anwender: Die Datei im Projekt bleibt unveraendert und ist
    # damit zugleich die Sicherungskopie.
    source = _sequence_file(tmp_path)
    before = source.read_text(encoding="utf-8")
    working = recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    name, gripper_start, steps = recording.read_steps(tmp_path / working)
    steps[1]["blend"] = 0.025
    recording.write_steps(tmp_path / working, name, gripper_start, steps)
    assert source.read_text(encoding="utf-8") == before
    assert recording.read_steps(tmp_path / working)[2][1]["blend"] == 0.025


def test_the_working_copy_survives_and_is_not_overwritten(tmp_path):
    # Der letzte Stand soll beim naechsten Start wieder dastehen.
    _sequence_file(tmp_path)
    working = recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    name, gripper_start, steps = recording.read_steps(tmp_path / working)
    steps[1]["blend"] = 0.033
    recording.write_steps(tmp_path / working, name, gripper_start, steps)
    again = recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    assert str(again) == str(working)
    assert recording.read_steps(tmp_path / working)[2][1]["blend"] == 0.033


def test_reset_brings_the_original_back(tmp_path):
    _sequence_file(tmp_path)
    working = recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    name, gripper_start, steps = recording.read_steps(tmp_path / working)
    del steps[1]
    recording.write_steps(tmp_path / working, name, gripper_start, steps)
    assert len(recording.read_steps(tmp_path / working)[2]) == 3
    recording.reset_working_copy(tmp_path, "sequences/pick.json")
    assert len(recording.read_steps(tmp_path / working)[2]) == 4


def test_an_invalid_sequence_is_refused_before_it_is_written(tmp_path):
    # Sonst faellt es erst beim Start auf -- und dann steht jemand an der
    # Anlage davor.
    _sequence_file(tmp_path)
    working = tmp_path / recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    name, gripper_start, steps = recording.read_steps(working)
    before = working.read_text(encoding="utf-8")
    steps[0]["blend"] = 0.05          # erster Schritt darf nicht ueberschliffen
    try:
        recording.write_steps(working, name, gripper_start, steps)
    except ValueError as exc:
        assert "erste" in str(exc)
    else:
        raise AssertionError("ungueltiger Ablauf haette auffallen muessen")
    assert working.read_text(encoding="utf-8") == before


def test_validation_uses_the_same_rules_as_record_py(tmp_path):
    _sequence_file(tmp_path)
    working = tmp_path / recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    name, gripper_start, steps = recording.read_steps(working)
    assert recording.validate_steps(name, gripper_start, steps) == []
    # blend und gripper schliessen sich aus (bc/sequence.py)
    steps[2]["blend"] = 0.01
    assert recording.validate_steps(name, gripper_start, steps)


def test_a_step_without_a_point_is_named(tmp_path):
    _sequence_file(tmp_path)
    working = tmp_path / recording.ensure_working_copy(tmp_path, "sequences/pick.json")
    name, gripper_start, steps = recording.read_steps(working)
    steps[1]["point"] = "   "
    problems = recording.validate_steps(name, gripper_start, steps)
    assert problems and "kein Punktname" in problems[0]


def test_defaults_are_not_written_out(tmp_path):
    # Die Arbeitskopie soll so lesbar bleiben wie die Vorlage von Hand.
    data = recording.steps_to_data("x", "open", [
        {"point": "A", "motion": "ptp", "blend": 0.0, "optional": False,
         "approach": False, "gripper": None},
        {"point": "B", "motion": "lin", "blend": 0.02, "optional": True,
         "approach": True, "gripper": None},
    ])
    assert data["sequence"][0] == {"point": "A", "motion": "ptp"}
    assert set(data["sequence"][1]) == {"point", "motion", "blend", "optional",
                                        "approach"}

"""Recorder: asymmetrische Paarung, Dwell, Verwerf-Logik (AP 2.1/2.4/5.2).

Der End-to-End-Kern der hardwarefreien Absicherung: SimRobot + SimCameras
+ SimClock, komplette Episodengenerierung und -aufzeichnung.
"""

import _paths  # noqa: F401

import numpy as np

import _fixtures
from bc import config
from bc.adapters.cam_sim import CameraFaultProfile, SimCamera
from bc.adapters.sim_robot import SimRobot
from bc.capture import DirectCapture
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.kinematics import Kinematics
from bc.noise import generate_plan
from bc.ports import RobotError
from bc.recorder import EpisodeRecorder
from bc.trajectory import Waypoint, build_ideal_trajectory


def _setup(robot_cls=SimRobot, cam_faults=None, **robot_kwargs):
    clock = SimClock()
    robot = robot_cls(clock=clock, **robot_kwargs).connect()
    cams = [
        SimCamera(cfg, clock=clock, faults=cam_faults)
        for cfg in config.SIM_CAMERAS
    ]
    captures = [DirectCapture(c).start() for c in cams]
    return clock, robot, captures


def _plan(robot, gripper=True):
    kin = Kinematics(robot)
    home = robot.read_state().joints
    start = robot.fk(home)

    def wp(dx, dz, g=False, approach=False):
        pose = np.asarray(start, dtype=float).copy()
        pose[0] += dx
        pose[2] += dz
        return Waypoint(pose, gripper_closed=g, approach=approach)

    waypoints = [wp(0, 0), wp(0.05, -0.02)]
    if gripper:
        # Greifen + anschliessendes Heben: erst dadurch liegt zwischen
        # Greiferbefehl und Folgebewegung die volle Totzeit (AP 2.1).
        waypoints.append(wp(0.05, -0.04, g=True, approach=True))
        waypoints.append(wp(0.05, -0.01, g=True))
    ideal = build_ideal_trajectory(waypoints)
    workspace = default_workspace(table_height_m=float(start[2]) - 0.30)
    return generate_plan(
        kin, workspace, ideal, home, rng=np.random.default_rng(0)
    )


def test_record_full_episode():
    clock, robot, captures = _setup()
    plan = _plan(robot)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan, metadata={"mode": "test"})

    n = len(plan)
    assert len(episode) == n
    assert not episode.discarded, episode.discard_reason
    episode.validate()

    # Asymmetrische Paarung (AP 2.4): Action = IDEALE Winkel bei t+1
    for i in range(n):
        j = min(i + 1, n - 1)
        assert np.allclose(
            episode.arrays["action"][i, :6],
            plan.joints_ideal[j].astype(np.float32),
        )
        assert episode.arrays["action"][i, 6] == plan.gripper[j]

    # Observation = ECHTER (verrauschter) Zustand: der SimRobot folgt
    # servo_j exakt, also muss der State der verrauschten Bahn entsprechen.
    # Schritt 0 zeigt noch die Startpose (Zustand wird nach servo_j des
    # aktuellen Schritts gelesen) -> ab Schritt 1 vergleichen.
    for i in range(1, n):
        assert np.allclose(
            episode.arrays["observation.state"][i, :6],
            plan.joints_noisy[i].astype(np.float32),
            atol=1e-6,
        )

    # Sync: mit gemeinsamer SimClock ist alles im Budget
    assert episode.arrays["aux.sync_ok"].all()
    assert episode.metadata["recorded_steps"] == n

    # Gesendete Sollwinkel sind die verrauschte Bahn (Nachlauf-Auswertung)
    assert np.allclose(
        episode.arrays["aux.joints_command"], plan.joints_noisy.astype(np.float32)
    )
    # Uebergabepunkt: next.done genau im letzten Schritt
    done = episode.arrays["next.done"][:, 0]
    assert done[-1] and not done[:-1].any()


class SpyRobot(SimRobot):
    """Merkt sich alle servo_j-Aufrufe (Testhelfer)."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.servo_calls = []

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        super().servo_j(joint_angles, velocity, acceleration)
        self.servo_calls.append((joint_angles, velocity, acceleration))


def test_servo_substeps_interpolate_plan_with_velocity():
    # Befund VM 2026-09-15: bei 15 Hz faehrt der Controller jeden Sollwert
    # an und steht dann (Stop-and-go). Jetzt: Zwischenschritte mit
    # SERVO_RATE_HZ, das Ziel des Schritts i genau auf Takt i. servo_j bekommt
    # immer drei Listen (VM 2026-09-09), die Geschwindigkeit passt zur Fahrt.
    clock, robot, captures = _setup(robot_cls=SpyRobot)
    plan = _plan(robot)
    t0 = clock.now()
    EpisodeRecorder(robot, captures, clock).record(plan)

    sub = int(round(config.SERVO_RATE_HZ / config.CONTROL_RATE_HZ))
    assert sub > 1
    calls = robot.servo_calls
    assert len(calls) == 1 + (len(plan) - 1) * sub
    q_all = np.array([c[0] for c in calls])
    # Auf jedem Beobachtungstakt exakt der geplante Sollwert
    assert np.allclose(q_all[::sub], plan.joints_noisy)
    # Dazwischen linear
    i = len(plan) // 2
    mid = q_all[(i - 1) * sub + sub // 2]
    assert np.allclose(
        mid, plan.joints_noisy[i - 1] + (sub // 2) / sub * (plan.joints_noisy[i] - plan.joints_noisy[i - 1])
    )
    # Geschwindigkeit = Zieldifferenz je Takt, nie pauschal None
    period = 1.0 / config.CONTROL_RATE_HZ
    for k in range(1, sub + 1):
        q, qd, qdd = calls[(i - 1) * sub + k]
        assert qd is not None and qdd is not None
        assert np.allclose(qd, (plan.joints_noisy[i] - plan.joints_noisy[i - 1]) / period)
    assert max(np.abs(c[1]).max() for c in calls) > 0.01
    # Senderate stimmt: Episode dauert (N-1) Takte
    assert abs((clock.now() - t0) - (len(plan) - 1) * period) < 2 * period


def test_refuses_to_start_away_from_start_pose():
    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=False)
    robot._joints = robot._joints + 0.1  # Arm steht woanders
    try:
        EpisodeRecorder(robot, captures, clock).record(plan)
        assert False, "RobotError erwartet (nicht an der Startstellung)"
    except RobotError as exc:
        assert "Startstellung" in str(exc)
    assert not robot._servo_active  # Servo wurde gar nicht erst aktiviert


def test_gripper_dwell_respected():
    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=True)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)

    # Der Greifer-Befehl fiel waehrend der Episode
    assert robot.gripper_closed
    # Nach den Dwell-Schritten ist die Totzeit real verstrichen (die
    # SimClock lief mit dem Pacer weiter) -> Backen sind "angekommen".
    assert robot.gripper_settled()

    # Im Datensatz: Greiferkanal wechselt genau am Dwell-Beginn
    g = episode.arrays["observation.state"][:, 13]
    changes = np.where(np.diff(g) != 0)[0]
    assert len(changes) == 1
    dwell_start = np.where(plan.dwell_mask)[0][0]
    assert changes[0] == dwell_start - 1


def test_discard_on_stale_camera():
    # Fehlerinjektion (AP 0.5): eine Kamera haengt 50 ms hinterher --
    # ueber dem 30-ms-Budget -> Episode muss verworfen werden (AP 5.2).
    clock, robot, captures = _setup(
        cam_faults=CameraFaultProfile(stale_s=0.05)
    )
    plan = _plan(robot, gripper=False)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)
    assert episode.discarded
    assert "latenzbudget" in episode.discard_reason
    assert not episode.arrays["aux.sync_ok"].any()


_SUBSTEPS = int(round(config.SERVO_RATE_HZ / config.CONTROL_RATE_HZ))


class TrippingRobot(SimRobot):
    """Loest nach N servo-Befehlen den Schutzstopp aus (Testhelfer)."""

    #: Halte-Befehl + 4 volle Fenster, Stopp mitten im fuenften
    TRIP_AFTER = 1 + 4 * _SUBSTEPS + 1

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._servo_count = 0

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        super().servo_j(joint_angles, velocity, acceleration)
        self._servo_count += 1
        if self._servo_count == self.TRIP_AFTER:
            self.emergency_stop()


def test_discard_on_emergency_stop():
    clock, robot, captures = _setup(robot_cls=TrippingRobot)
    plan = _plan(robot, gripper=False)
    assert len(plan) > 6
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)

    assert episode.discarded
    assert episode.discard_reason == "schutzstopp"
    # Es wurde nur bis zum Stopp aufgezeichnet (Takte 0-4), das angebrochene
    # Fenster nicht mehr, nichts stillschweigend aufgefuellt
    assert len(episode) == 5
    # Nach dem Stopp kein weiterer Sollwert
    assert robot._servo_count == TrippingRobot.TRIP_AFTER
    # Nach dem Not-Halt ist das Servo-Interface deaktiviert
    assert not robot._servo_active
    # Abgebrochene Episode: kein Uebergabepunkt markiert
    assert not episode.arrays["next.done"].any()


class DeafRobot(SimRobot):
    """Nimmt Sollwerte an, bewegt sich aber nicht (Testhelfer).

    Bildet den Befund vom 2026-10-01 nach: die Steuerung kappte die
    PC-Steuerung (RCSC_102), ``servo_j`` meldete weiter "in Ordnung", und
    der Arm stand. Ohne Schleppfehler-Pruefung lief der Recorder die
    gesamte Bahn durch und legte 218 Takte Datenmuell als "vollstaendig" ab.
    """

    def servo_j(self, joint_angles, velocity=None, acceleration=None):
        gehalten = self._joints.copy()
        super().servo_j(joint_angles, velocity, acceleration)
        self._joints = gehalten  # Sollwert quittiert, nichts bewegt


def test_discard_when_arm_does_not_follow():
    """Kriterium (b): der Sollwert wandert, der Arm nicht.

    Massstabsfrei -- greift auch hier, wo die ganze Bahn nur rund 0,12 rad
    Gelenkhub hat und die absolute Schwelle (0,30 rad) nie erreicht wird.
    Genau diese Bahnlaenge ist der Grund, warum es (a) allein nicht tut.
    """
    clock, robot, captures = _setup(robot_cls=DeafRobot)
    plan = _plan(robot, gripper=False)
    recorder = EpisodeRecorder(robot, captures, clock)
    episode = recorder.record(plan)

    assert episode.discarded
    assert episode.discard_reason.startswith("arm folgt nicht")
    assert "Sollwert wanderte" in episode.discard_reason
    # Nicht die ganze Bahn, und nichts stillschweigend aufgefuellt
    assert len(episode) < len(plan)
    assert len(episode.arrays["observation.state"]) == len(episode)
    # Kein Uebergabepunkt auf einer abgebrochenen Episode
    assert not episode.arrays["next.done"].any()
    assert not robot._servo_active
    # Der Schleppfehler steht als Zahl in den Metadaten, nicht nur im Text
    assert episode.metadata["max_follow_error_rad"] > 0.0


def test_discard_on_absolute_follow_error():
    """Kriterium (a): der Arm bewegt sich, haengt aber zu weit zurueck."""
    clock, robot, captures = _setup(robot_cls=DeafRobot)
    plan = _plan(robot, gripper=False)
    # Schwelle unter den Gelenkhub der Bahn legen, Kriterium (b) abschalten:
    # so wird ausschliesslich (a) geprueft.
    recorder = EpisodeRecorder(
        robot, captures, clock, max_follow_error_rad=0.02,
        follow_error_steps=2, min_follow_ratio=0.0,
    )
    episode = recorder.record(plan)

    assert episode.discarded
    assert episode.discard_reason.startswith("arm folgt nicht")
    assert "Schleppfehler" in episode.discard_reason
    assert episode.metadata["max_follow_error_rad"] > 0.02


def test_following_arm_is_not_discarded_for_follow_error():
    """Gegenprobe: der normale Lauf darf von keinem der Kriterien getroffen werden."""
    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=True)
    episode = EpisodeRecorder(robot, captures, clock).record(plan)

    assert not episode.discarded
    assert len(episode) == len(plan)
    assert episode.metadata["max_follow_error_rad"] <= config.RECORDER_MAX_FOLLOW_ERROR_RAD


# -- Laufzeit-Hinweise (timing_findings) --------------------------------------

def _meta(dauer, soll, servo_ms, read_ms, servo_rate=60.0):
    return {
        "duration_s": dauer,
        "planned_duration_s": soll,
        "rate_hz": 15.0,
        "servo_rate_hz": servo_rate,
        "servo_j_ms": {"median": servo_ms},
        "read_state_ms": {"median": read_ms},
    }


def test_timing_findings_silent_for_a_healthy_run():
    """Gesunde Laeufe (2026-10-02: Faktor 1,01-1,04) bleiben ohne Hinweis."""
    from bc.recorder import timing_findings

    assert timing_findings(_meta(14.7, 14.5, 3.0, 5.0)) == []


def test_timing_findings_name_slowdown_budget_and_suspected_host():
    """Der Laptop im Akkubetrieb (2026-10-01): servo_j 22,4 ms bei 60 Hz."""
    from bc.recorder import timing_findings

    lines = timing_findings(_meta(33.0, 14.5, 22.4, 19.5))
    text = "\n".join(lines)
    assert "2.28-fach langsamer" in text
    assert "4 x servo_j" in text
    assert "kleinere servo_j-Rate" in text
    assert "ausgelasteten Rechner" in text and "Moeglich ist" in text


def test_timing_findings_blame_the_rate_not_the_host_when_calls_are_cheap():
    """120 Hz auf einem gesunden Rechner: zu viele Aufrufe, nicht zu teure."""
    from bc.recorder import timing_findings

    lines = timing_findings(_meta(24.0, 14.5, 9.0, 8.0, servo_rate=120.0))
    text = "\n".join(lines)
    assert "kleinere servo_j-Rate" in text
    assert "ausgelasteten Rechner" not in text


def test_recorder_measures_duration_and_call_costs():
    """Durchgehend: langsame Steuerung -> Metadaten und Hinweis stimmen."""
    from bc.adapters.sim_robot import FaultProfile
    from bc.recorder import timing_findings

    clock, robot, captures = _setup(faults=FaultProfile(latency_s=0.03))
    plan = _plan(robot, gripper=False)
    episode = EpisodeRecorder(robot, captures, clock).record(plan)
    meta = episode.metadata

    assert meta["servo_j_ms"]["median"] > 29.9  # SimClock-Arithmetik: 29,9999...
    assert meta["read_state_ms"]["median"] > 29.9
    assert meta["duration_s"] > 1.1 * meta["planned_duration_s"]
    assert timing_findings(meta), "zu langsame Episode ohne Hinweis"


def test_monitor_gets_schema_size_frames_and_cannot_touch_the_data():
    """Livebild aus den schon verkleinerten Bildern (Sparvorschlag 1).

    Volle Kameraaufloesung rein, der Monitor sieht 240x320 -- und das
    Beschriften im Livebild (compose) veraendert die Episode nicht.
    """
    import dataclasses

    from bc import preview

    clock = SimClock()
    robot = SimRobot(clock=clock).connect()
    cams = [
        SimCamera(dataclasses.replace(cfg, width=1440, height=1080), clock=clock)
        for cfg in config.SIM_CAMERAS
    ]
    captures = [DirectCapture(c).start() for c in cams]
    plan = _plan(robot, gripper=False)

    seen = []
    composed = []

    def monitor(i, n, frames, report):
        seen.append({k: None if f is None else f.image.shape
                     for k, f in frames.items()})
        if i < 3:
            snapshot = {k: f.image.copy() for k, f in frames.items()}
            composed.append(preview.compose(frames, overlay="Takt %d" % i))
            for k, f in frames.items():
                assert np.array_equal(f.image, snapshot[k]), "compose schreibt in die Daten"

    episode = EpisodeRecorder(robot, captures, clock, monitor=monitor).record(plan)

    schema = (config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3)
    assert len(seen) == len(plan)
    assert all(shape == schema for step in seen for shape in step.values())
    assert composed[0].shape == (config.IMAGE_HEIGHT, 2 * config.IMAGE_WIDTH, 3)
    for cfg in config.SIM_CAMERAS:
        assert episode.arrays["observation.images.%s" % cfg.name].shape[1:] == schema


def test_online_execution_sends_one_target_per_tick_with_lead():
    """Ausfuehrung online (bc/execution.py): ein Ziel je Takt, Vorlauf."""
    from bc import execution

    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=False)
    ex = execution.describe("online", lead_steps=2)
    gesendet = []
    original = robot.online_j

    def mitschreiben(q, vel, acc):
        gesendet.append(np.asarray(q, dtype=float).copy())
        return original(q, vel, acc)

    robot.online_j = mitschreiben
    servo_aufrufe = []
    robot.servo_j = lambda *a, **k: servo_aufrufe.append(a)
    episode = EpisodeRecorder(robot, captures, clock, execution=ex).record(plan)

    n = len(plan)
    assert not episode.discarded, episode.discard_reason
    assert not servo_aufrufe, "online darf kein servo_j senden"
    assert len(gesendet) == n
    # Takt i sendet das Ziel i+2, am Ende festgehalten
    assert np.allclose(gesendet[0], plan.joints_noisy[2])
    assert np.allclose(gesendet[-1], plan.joints_noisy[n - 1])
    assert episode.metadata["execution"]["mode"] == "online"
    # Ein Aufruf je Regeltakt -> timing_findings rechnet mit k = 1
    assert episode.metadata["servo_j_ms"]["n"] == n


def test_online_stop_discards_the_episode():
    from bc import execution

    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=False)
    zaehler = {"n": 0}
    original = robot.online_j

    def stoppt(q, vel, acc):
        zaehler["n"] += 1
        if zaehler["n"] == 5:
            robot.emergency_stop()
            return None
        return original(q, vel, acc)

    robot.online_j = stoppt
    episode = EpisodeRecorder(robot, captures, clock,
                              execution=execution.describe("online")).record(plan)
    assert episode.discarded and episode.discard_reason == "schutzstopp"


def test_timing_findings_count_one_call_per_tick_for_online():
    from bc.recorder import timing_findings

    meta = _meta(20.0, 14.5, 9.0, 8.0)
    meta["execution"] = {"mode": "online"}
    text = "\n".join(timing_findings(meta))
    assert "1 x online_j" in text


# -- "Erst weiter, wenn erreicht" (online, Anlage 2026-10-16) ----------------


def _online_protokoll(robot):
    """Schreibt online_j-Ziele und Greiferbefehle in Reihenfolge mit."""
    ereignisse = []
    online, greifer = robot.online_j, robot.gripper_command

    def online_j(q, vel, acc):
        ereignisse.append(("online", np.asarray(q, dtype=float).copy()))
        return online(q, vel, acc)

    def gripper_command(close):
        ereignisse.append(("greifer", bool(close), robot.read_state().tcp_quat[:3].copy()))
        return greifer(close)

    robot.online_j = online_j
    robot.gripper_command = gripper_command
    return ereignisse


def test_online_waits_at_the_gripper_change_until_the_arm_is_there():
    from bc import execution

    clock, robot, captures = _setup()
    robot.ONLINE_GAIN = 0.15        # Arm laeuft hinterher wie an der Anlage
    plan = _plan(robot, gripper=True)
    ex = execution.describe("online", lead_steps=2)
    holds = execution.hold_blocks(plan.gripper, plan.dwell_mask)
    assert len(holds) == 1
    h, e = holds[0]
    pick = robot.fk(plan.joints_noisy[h])[:3]
    ereignisse = _online_protokoll(robot)

    episode = EpisodeRecorder(robot, captures, clock, execution=ex).record(plan)

    n = len(plan)
    assert not episode.discarded, episode.discard_reason
    assert len(episode) == n                    # Wartezeit ist kein Schritt
    meta = episode.metadata
    assert len(meta["reach_waits"]) == 1
    halt = meta["reach_waits"][0]
    assert halt["step"] == h and halt["reached"] and halt["ticks"] > 0
    assert halt["start_mm"] > ex["reach_tol_mm"]       # ohne Warten: zu hoch
    assert abs(meta["reach_wait_s"] - halt["wait_s"]) < 1e-12 and halt["wait_s"] > 0
    assert meta["pacer_overruns"] == 0

    # Greifer schliesst erst, wenn der Arm am Haltepunkt steht ...
    zu = [k for k, ev in enumerate(ereignisse) if ev[0] == "greifer" and ev[1]]
    assert len(zu) == 1
    assert 1e3 * np.linalg.norm(ereignisse[zu[0]][2] - pick) <= ex["reach_tol_mm"]
    # ... und die Beobachtung dieses Takts zeigt ihn dort
    obs_tcp = episode.arrays["observation.state"][h, 6:9]
    assert 1e3 * np.linalg.norm(obs_tcp - pick) <= ex["reach_tol_mm"] + 1e-3

    # Danach der volle Dwell: bis zu dessen Ende kein Abfahrtsziel, dann
    # weiter mit Vorlauf.
    nachher = [ev[1] for ev in ereignisse[zu[0] + 1:] if ev[0] == "online"]
    assert len(nachher) == n - 1 - h
    for k in range(e - h):                      # Takte h+1 .. e
        assert np.allclose(nachher[k], plan.joints_noisy[e])
    assert np.allclose(nachher[e - h], plan.joints_noisy[min(e + 1 + 2, n - 1)])
    # vor dem Halt: Vorlauf schaut nicht ueber das Dwell-Ende hinaus
    vorher = [ev[1] for ev in ereignisse[:zu[0]] if ev[0] == "online"]
    assert np.allclose(vorher[0], plan.joints_noisy[2])


def test_online_hold_timeout_discards_the_episode_cleanly():
    from bc import execution

    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=True)
    (h, _), = execution.hold_blocks(plan.gripper, plan.dwell_mask)
    ex = execution.describe("online", reach_timeout_s=0.5)
    zaehler = {"n": 0}
    original = robot.online_j

    def bleibt_stehen(q, vel, acc):
        # Ab einige Takte vor dem Haltepunkt nimmt die Steuerung nichts mehr an
        zaehler["n"] += 1
        if zaehler["n"] > h - 8:
            return None
        return original(q, vel, acc)

    robot.online_j = bleibt_stehen
    episode = EpisodeRecorder(robot, captures, clock, execution=ex,
                              max_follow_error_rad=None).record(plan)
    assert episode.discarded
    assert episode.discard_reason.startswith("haltepunkt nicht erreicht")
    assert "Takt %d" % h in episode.discard_reason
    assert len(episode) == h                    # bis vor den Haltepunkt
    halt, = episode.metadata["reach_waits"]
    assert not halt["reached"] and halt["reason"] == "zeit"
    assert not robot.stop_requested             # geordnet, kein Not-Halt
    assert not robot.gripper_closed             # nie gegriffen


def test_online_without_reach_rule_behaves_as_before():
    """Aeltere online-Aufnahmen (ohne reach_*) fahren ohne Warten."""
    from bc import execution

    clock, robot, captures = _setup()
    plan = _plan(robot, gripper=True)
    ex = execution.describe("online", lead_steps=2, reach_tol_mm=None)
    ereignisse = _online_protokoll(robot)
    episode = EpisodeRecorder(robot, captures, clock, execution=ex).record(plan)
    assert not episode.discarded, episode.discard_reason
    assert "reach_waits" not in episode.metadata
    ziele = [ev[1] for ev in ereignisse if ev[0] == "online"]
    n = len(plan)
    assert len(ziele) == n
    for i, q in enumerate(ziele):
        assert np.allclose(q, plan.joints_noisy[min(i + 2, n - 1)])


def test_timing_findings_ignore_the_hold_wait():
    from bc.recorder import timing_findings

    meta = _meta(20.0, 14.5, 9.0, 8.0)
    meta["execution"] = {"mode": "online"}
    assert timing_findings(meta)
    meta["reach_wait_s"] = 5.5
    assert timing_findings(meta) == []


def test_servo_j_pos_sends_same_positions_without_velocity():
    # Ausfuehrung servo_j_pos (bc/execution.py, VM 2026-10-14): dieselben
    # Sollpositionen wie servo_j, aber v = a = 0 -- die Steuerung glaettet
    # dann selbst. Steht in den Metadaten, damit die Inferenz gleich faehrt.
    from bc import execution

    clock, robot, captures = _setup(robot_cls=SpyRobot)
    plan = _plan(robot)
    EpisodeRecorder(robot, captures, clock).record(plan)
    linear = [np.asarray(c[0]) for c in robot.servo_calls]

    clock, robot, captures = _setup(robot_cls=SpyRobot)
    episode = EpisodeRecorder(robot, captures, clock,
                              execution=execution.describe("servo_j_pos")).record(plan)
    calls = robot.servo_calls
    assert len(calls) == len(linear)
    assert np.allclose(np.array([c[0] for c in calls]), np.array(linear))
    for _, v, a in calls:
        assert v is not None and a is not None
        assert not np.any(v) and not np.any(a)
    assert episode.metadata["execution"] == {"mode": "servo_j_pos"}
    assert not episode.discarded, episode.discard_reason

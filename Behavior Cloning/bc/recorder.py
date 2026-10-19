"""Episoden-Recorder: fuehrt eine geplante Bahn aus und zeichnet auf (AP 2.x).

Ablauf pro Zeitschritt i (bei CONTROL_RATE_HZ):

0. (einmalig) Pruefen, dass der Roboter an der Startstellung steht --
   sonst waere der erste Sollwert ein Sprung. Hinfahren ist Sache des
   Aufrufers (apps/record.py, ausserhalb der Aufzeichnung).
1./2. Verrauschte Sollwinkel des Schritts i ueber den vorangehenden Takt
   linear anfahren: Zwischenschritte mit SERVO_RATE_HZ (bc/servo.py), der
   letzte faellt genau auf Takt i (:class:`bc.sync.Pacer`). Dieselbe
   Interpolation nutzt die Inferenz -- sonst folgt der Arm dort anders als
   in den Daten. Ausfuehrung ``servo_j_pos``: dieselben Positionen ohne
   Geschwindigkeit/Beschleunigung (Profil ``pos``).
   Ausfuehrung ``online`` (bc/execution.py): statt dessen EIN Ziel je Takt
   per ``robot.online_j`` -- der Sollwinkel des Schritts i+Vorlauf, die
   Steuerung interpoliert selbst (gegen das Zittern an der Anlage,
   Dokumentation/Zittern-an-der-Anlage.md). ``aux.joints_command`` bleibt
   der Sollwinkel des Schritts i -- die Stellung, die der Arm in diesem
   Takt haben soll.
3. Greiferbefehl absetzen, wenn die Bahn an dieser Stelle wechselt --
   die Totzeit ist als Dwell-Schritte bereits IN der Bahn (AP 2.1).
   Ausfuehrung ``online`` mit Halteregel (``reach_*``, bc/execution.py):
   VOR dem Befehl am Haltepunkt warten, bis der Arm dort steht; der
   Vorlauf schaut bis zum Ende des Dwells nicht darueber hinaus. Die
   Wartezeit ist kein Schritt der Episode, sie steht in den Metadaten
   (``reach_waits``); die Beobachtung dieses Takts wird danach gelesen.
4. Zustand und juengste Frames abgreifen, Zeitstempel gegen das
   Latenzbudget bewerten (AP 1.3).
5. Asymmetrisch paaren (AP 2.4): Observation = echter (verrauschter)
   Zustand + Bilder; Action = IDEALE Sollwinkel des Schritts i+1.

Verwerf-Logik (AP 5.2): ausgeloester Stopp bricht sofort ab und markiert
die Episode als verworfen; zu viele Latenzbudget-Verletzungen ebenso.

``next.done`` ist nur im letzten Schritt einer VOLLSTAENDIGEN Episode True
-- das Ende der Bahn ist der Uebergabepunkt ans Hauptprogramm.
"""

import logging

import numpy as np

from . import config, dataset
from . import execution as execution_module
from .ports import Frame, RobotError
from .servo import ServoInterpolator
from .sync import Pacer, evaluate

log = logging.getLogger(__name__)


class EpisodeRecorder(object):
    """Zeichnet Episoden entlang eines :class:`TrajectoryPlan` auf.

    ``captures``: Liste von Objekten mit ``.name`` und ``.latest()``
    (ThreadedCapture im Betrieb, DirectCapture in Tests).
    """

    def __init__(
        self,
        robot,
        captures,
        clock,
        rate_hz=config.CONTROL_RATE_HZ,
        max_skew=config.SYNC_MAX_SKEW_S,
        max_bad_ratio=config.SYNC_MAX_BAD_FRAME_RATIO,
        servo_rate_hz=config.SERVO_RATE_HZ,
        monitor=None,
        max_follow_error_rad=config.RECORDER_MAX_FOLLOW_ERROR_RAD,
        follow_error_steps=config.RECORDER_FOLLOW_ERROR_STEPS,
        min_follow_ratio=config.RECORDER_MIN_FOLLOW_RATIO,
        stall_grace_s=config.RECORDER_STALL_GRACE_S,
        execution=None,
    ):
        self.robot = robot
        #: Eintrag aus bc.execution.describe(); None = servo_j wie bisher.
        self.execution = execution or execution_module.describe(execution_module.SERVO_J)
        self.captures = list(captures)
        self.clock = clock
        self.rate_hz = rate_hz
        self.servo_rate_hz = servo_rate_hz
        self.max_skew = max_skew
        self.max_bad_ratio = max_bad_ratio
        #: Schleppfehler-Abbruch (config.RECORDER_*). ``None`` schaltet ab --
        #: nur fuer Werkzeuge, die bewusst ohne Arm laufen.
        self.max_follow_error_rad = max_follow_error_rad
        self.follow_error_steps = int(follow_error_steps)
        self.min_follow_ratio = float(min_follow_ratio)
        #: Kriterium (b) erst nach so vielen Takten in Folge (mindestens 1):
        #: der Arm pausiert an der Anlage nach Stillstaenden bis 1,8 s.
        self.stall_grace_steps = max(1, int(round(float(stall_grace_s) * rate_hz)))
        #: Optionaler Rueckruf je Takt: ``monitor(i, n, frames, report)``,
        #: ``frames`` bereits auf Schemagroesse (wie gespeichert).
        #: Gedacht fuer Fortschritt und Livebild in der Bedienoberflaeche
        #: (bc/preview.py). STANDARD IST None -- die Aufzeichnungsschleife
        #: hat 1/15 s je Takt, und jede zusaetzliche Arbeit darin gefaehrdet
        #: genau die Datenqualitaet, um die es geht. Ein Fehler im Rueckruf
        #: bricht die Aufnahme nie ab.
        self.monitor = monitor

    def record(self, plan, metadata=None):
        """Fuehrt den Plan aus und liefert eine :class:`dataset.Episode`.

        Bricht bei ``robot.stop_requested`` sofort ab (Schutzstopp,
        AP 2.4/4.2) -- die Episode wird verworfen, nie stillschweigend
        gekuerzt.
        """
        n = len(plan)
        camera_names = [c.name for c in self.captures]

        start_error = float(
            np.max(np.abs(self.robot.read_state().joints - plan.joints_noisy[0]))
        )
        if start_error > config.START_POSE_TOL_RAD:
            raise RobotError(
                "Roboter steht nicht an der Startstellung der Bahn (Abweichung "
                "%.4f rad > %.4f rad) -- vorher move_to_joints ausfuehren."
                % (start_error, config.START_POSE_TOL_RAD)
            )
        # Profil der Zwischenschritte: servo_j linear, servo_j_pos ohne v/a
        # (bc/execution.py); bei online ungenutzt.
        interp = ServoInterpolator(
            self.rate_hz, self.servo_rate_hz,
            profile=execution_module.servo_profile(self.execution) or "linear",
        ).reset(plan.joints_noisy[0])

        steps = {
            "observation.state": [],
            "action": [],
            "aux.joints_ideal": [],
            "aux.joints_command": [],
            "aux.pose_ideal": [],
            "aux.pose_noisy": [],
            "aux.sync_ok": [],
            "next.done": [],
        }
        for name in camera_names:
            steps["observation.images.%s" % name] = []

        bad_frames = 0
        discard_reason = ""
        follow_bad = 0  # Takte in Folge mit zu grossem Schleppfehler
        stall_steps = 0  # Takte in Folge, in denen (b) anliegt
        cmd_path = None   # Weglaenge je Gelenk: Sollwert ...
        meas_path = None  # ... und Messung (Gesamtpruefung am Ende)
        prev_cmd = prev_meas = None
        worst_follow = 0.0
        cmd_hist = []   # Sollwerte der letzten Takte (Kriterium b)
        meas_hist = []  # zugehoerige Messungen
        online = self.execution["mode"] == execution_module.ONLINE
        # "Erst weiter, wenn erreicht" (nur online): Haltepunkte des Plans.
        halt = execution_module.reach_rule(self.execution)
        holds = execution_module.hold_blocks(plan.gripper, plan.dwell_mask) if halt else []
        hold_starts = {start for start, _ in holds}
        reach_waits = []
        self.robot.activate_servo("position")
        # Takt erst NACH dem Aktivieren starten: die Aktivierung dauert an
        # der VM einige 10 ms -- bei 60 Hz zaehlte das bisher als 3
        # Ueberlaeufe je Episode, obwohl kein Sollwert zu spaet kam.
        # online: ein Aufruf je Regeltakt, keine Zwischenschritte.
        pacer = Pacer(self.clock, self.rate_hz if online else self.servo_rate_hz).start()
        completed = False
        # Laufzeitmessung fuer timing_findings(): Kosten je Aufruf und
        # Dauer der Episode. Nur Metadaten, kein Teil des Schemas.
        t_start = self.clock.now()
        servo_s = []
        read_s = []
        try:
            prev_gripper = None
            for i in range(n):
                # Takt 0: an der Startstellung halten. Danach das Ziel des
                # Schritts i ueber den vorangehenden Takt interpoliert
                # anfahren -- der letzte Zwischenschritt faellt auf Takt i.
                if online:
                    if not self.robot.stop_requested:
                        t_target = pacer.tick()
                        ziel = plan.joints_noisy[execution_module.lead_index(
                            i, self.execution["lead_steps"], n,
                            execution_module.next_hold_end(i, holds))]
                        t0 = self.clock.now()
                        self.robot.online_j(ziel, self.execution["vel"],
                                            self.execution["acc"])
                        servo_s.append(self.clock.now() - t0)
                else:
                    commands = [interp.hold()] if i == 0 else interp.window(plan.joints_noisy[i])
                    for q_cmd, v_cmd, a_cmd in commands:
                        if self.robot.stop_requested:
                            break
                        t_target = pacer.tick()
                        t0 = self.clock.now()
                        self.robot.servo_j(q_cmd, v_cmd, a_cmd)
                        servo_s.append(self.clock.now() - t0)
                if self.robot.stop_requested:
                    discard_reason = "schutzstopp"
                    break

                if i in hold_starts:
                    # Haltepunkt: Ziel stehen lassen, bis der Arm dort ist
                    # (Anlage 2026-10-16: ohne das kehrte er 27 mm ueber
                    # PICK um). Erst danach Greiferbefehl und Dwell.
                    halt_ergebnis = execution_module.wait_until_reached(
                        self.robot, plan.joints_noisy[i], self.execution,
                        self.clock, self.rate_hz)
                    halt_ergebnis["step"] = i
                    reach_waits.append(halt_ergebnis)
                    if not halt_ergebnis["reached"]:
                        discard_reason = execution_module.reach_failure_text(
                            halt_ergebnis, i)
                        break
                    # Neuer Takt ab hier -- die Wartezeit ist kein
                    # Rueckstand (sonst zaehlte der Pacer sie als Overruns);
                    # der naechste Sollwert kommt eine Periode spaeter.
                    pacer.start().tick()
                    t_target = self.clock.now()

                g = plan.gripper[i]
                if prev_gripper is None or g != prev_gripper:
                    self.robot.gripper_command(g >= config.GRIPPER_THRESHOLD)
                prev_gripper = g

                t0 = self.clock.now()
                state = self.robot.read_state()
                read_s.append(self.clock.now() - t0)

                # Folgt der Arm ueberhaupt? Der Sollwert des Takts i ist mit
                # dem letzten Zwischenschritt gerade gesendet worden -- die
                # Messung muss jetzt in seiner Naehe liegen. Tut sie das ueber
                # mehrere Takte nicht, nimmt die Steuerung die Sollwerte nicht
                # mehr an (Befund 2026-10-01: RCSC_102 kappt die PC-Steuerung,
                # servo_j meldet trotzdem weiter "in Ordnung"). Ohne diese
                # Pruefung landet eine Episode mit stehendem Arm als
                # "vollstaendig" auf der Platte.
                #
                # Kein Not-Halt an dieser Stelle: im beobachteten Fall steht
                # der Arm bereits, ein Stopp wuerde nur die naechste Episode
                # mit einer irrefuehrenden Meldung scheitern lassen. Der
                # Abbruch beendet den Sollwertstrom, das finally deaktiviert
                # das Servo-Interface -- mehr gehoert hier nicht hin (AP 4.2).
                if self.max_follow_error_rad is not None:
                    cmd_now = np.asarray(plan.joints_noisy[i], dtype=float)
                    meas_now = np.asarray(state.joints, dtype=float)
                    follow_err = float(np.max(np.abs(meas_now - cmd_now)))
                    worst_follow = max(worst_follow, follow_err)
                    cmd_hist.append(cmd_now)
                    meas_hist.append(meas_now)
                    if prev_cmd is None:
                        cmd_path = np.zeros_like(cmd_now)
                        meas_path = np.zeros_like(meas_now)
                    else:
                        cmd_path += np.abs(cmd_now - prev_cmd)
                        meas_path += np.abs(meas_now - prev_meas)
                    prev_cmd, prev_meas = cmd_now, meas_now
                    if len(cmd_hist) > self.follow_error_steps + 1:
                        cmd_hist.pop(0)
                        meas_hist.pop(0)

                    grund = ""
                    # (a) Absolut: der Arm haengt zu weit hinter dem Sollwert.
                    if follow_err > self.max_follow_error_rad:
                        follow_bad += 1
                        if follow_bad >= self.follow_error_steps:
                            grund = (
                                "%.3f rad Schleppfehler ueber %d Takte "
                                "(Grenze %.2f rad)"
                                % (follow_err, follow_bad,
                                   self.max_follow_error_rad)
                            )
                    else:
                        follow_bad = 0
                    # (b) Massstabsfrei: der Sollwert ist gewandert, der Arm
                    # nicht. Faengt den toten Steuerkanal auch auf kurzen oder
                    # langsamen Bahnen, wo (a) nie anschlaegt.
                    if not grund and len(cmd_hist) > self.follow_error_steps:
                        schritt = np.abs(cmd_hist[-1] - cmd_hist[0])
                        gelenk = int(np.argmax(schritt))
                        d_cmd = float(schritt[gelenk])
                        d_meas = float(np.max(np.abs(meas_hist[-1] - meas_hist[0])))
                        if (d_cmd > config.START_POSE_TOL_RAD
                                and d_meas < self.min_follow_ratio * d_cmd):
                            stall_steps += 1
                        else:
                            stall_steps = 0
                        # Erst nach stall_grace_steps in Folge: nach einem
                        # Stillstand pausiert der Arm an der Anlage bis
                        # 1,8 s und faehrt dann weiter (config).
                        if stall_steps >= self.stall_grace_steps:
                            # Sollgeschwindigkeit mitnennen: sie unterscheidet
                            # die beiden Ursachen, die hier zusammenlaufen
                            # (siehe unten). Ohne sie raet der Bedienende.
                            tempo = d_cmd / (self.follow_error_steps / self.rate_hz)
                            grund = (
                                "Sollwert wanderte %.3f rad, der Arm nur "
                                "%.3f rad ueber %d Takte (Gelenk %d, "
                                "Sollgeschwindigkeit %.2f rad/s), seit %.1f s"
                                % (d_cmd, d_meas, self.follow_error_steps,
                                   gelenk + 1, tempo, stall_steps / self.rate_hz)
                            )
                    if grund:
                        # Zwei Ursachen fuehren hierher, und sie sehen gleich
                        # aus. Beide nennen, statt eine zu raten (Befund
                        # 2026-10-01: zweimal war es die erste, nicht die
                        # zweite -- die Meldung zeigte trotzdem auf die
                        # zweite und haette die Suche fehlgeleitet):
                        #
                        # 1. Der Controller DARF nicht so schnell. Der
                        #    Override deckelt die Gelenkgeschwindigkeit;
                        #    liegt die Sollgeschwindigkeit darueber,
                        #    saettigt er und der Abstand waechst monoton.
                        # 2. Der PC-Steuerkanal ist weg (RCSC_102). Dann
                        #    steht der Arm ganz, und zwar sofort.
                        #
                        # Unterscheidbar am Verlauf: waechst der Schleppfehler
                        # allmaehlich, ist es (1); springt er, ist es (2).
                        discard_reason = (
                            "arm folgt nicht: %s (Takt %d von %d). "
                            "Pruefen: reicht der Override fuer diese "
                            "Sollgeschwindigkeit, oder ist der PC-Steuerkanal "
                            "weg (RCSC_102)? Der Verlauf von "
                            "aux.joints_command gegen observation.state "
                            "unterscheidet beides." % (grund, i, n)
                        )
                        break

                frames = {}
                timestamps = {"robot": state.t_joints}
                for cap in self.captures:
                    frame = cap.latest()
                    frames[cap.name] = frame
                    timestamps[cap.name] = None if frame is None else frame.timestamp

                report = evaluate(t_target, timestamps, self.max_skew)
                if not report.ok:
                    bad_frames += 1

                # Observation: der ECHTE (verrauschte) Zustand
                steps["observation.state"].append(
                    dataset.build_state(state.joints, state.tcp_quat, g)
                )
                schema_frames = {}
                for name in camera_names:
                    frame = frames[name]
                    if frame is None:
                        image = np.zeros(
                            (config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3), dtype=np.uint8
                        )
                        schema_frames[name] = None
                    else:
                        image = _to_schema_size(frame.image)
                        schema_frames[name] = Frame(
                            image, frame.timestamp, frame.index, frame.source)
                    steps["observation.images.%s" % name].append(image)

                # Der Monitor bekommt die schon verkleinerten Bilder: das
                # Livebild zeigt so genau das, was gespeichert wird, und
                # verkleinert die vollen Kamerabilder nicht ein zweites Mal
                # (gemessen ~3 ms je Kamera auf jedem dritten Takt, 2026-10-02).
                if self.monitor is not None:
                    try:
                        self.monitor(i, n, schema_frames, report)
                    except Exception:
                        log.exception("Monitor-Rueckruf gescheitert -- "
                                      "Aufnahme laeuft weiter")
                        self.monitor = None

                # Action: die IDEALE Soll-Bahn bei t+1 (AP 2.4). Am letzten
                # Schritt wird auf den Endzustand geklemmt.
                j = min(i + 1, n - 1)
                steps["action"].append(
                    dataset.build_action(plan.joints_ideal[j], plan.gripper[j])
                )

                steps["aux.joints_ideal"].append(
                    np.asarray(plan.joints_ideal[i], dtype=np.float32)
                )
                steps["aux.joints_command"].append(
                    np.asarray(plan.joints_noisy[i], dtype=np.float32)
                )
                steps["aux.pose_ideal"].append(
                    dataset.canonical_pose(plan.poses_ideal[i]).astype(np.float32)
                )
                steps["aux.pose_noisy"].append(
                    dataset.canonical_pose(plan.poses_noisy[i]).astype(np.float32)
                )
                steps["aux.sync_ok"].append(np.array([report.ok], dtype=bool))
                steps["next.done"].append(np.array([False], dtype=bool))
            else:
                completed = True
                # Gesamtpruefung zu (b): Bahnen kuerzer als die Anlaufzeit
                # (RECORDER_STALL_GRACE_S) erreicht (b) nie. Hat der Arm sich
                # ueber die ganze Episode kaum bewegt, obwohl der Sollwert
                # wanderte, ist der Steuerkanal tot (Befund 2026-10-01).
                if (cmd_path is not None and self.min_follow_ratio > 0
                        and not discard_reason):
                    gelenk = int(np.argmax(cmd_path))
                    d_cmd = float(cmd_path[gelenk])
                    d_meas = float(np.max(meas_path))
                    if (d_cmd > config.START_POSE_TOL_RAD
                            and d_meas < self.min_follow_ratio * d_cmd):
                        discard_reason = (
                            "arm folgt nicht: Sollwert wanderte insgesamt "
                            "%.3f rad (Gelenk %d), der Arm nur %.3f rad. "
                            "Ist der PC-Steuerkanal weg (RCSC_102)?"
                            % (d_cmd, gelenk + 1, d_meas)
                        )
        finally:
            if online and not self.robot.stop_requested:
                # Normales Ende: der Arm laeuft dem Vorlauf hinterher -- erst
                # ausrollen lassen, dann beenden. stop_online() bremst nicht
                # (Anlage 2026-10-13); Abbrueche laufen ueber emergency_stop().
                if completed:
                    self._ausrollen(plan.joints_noisy[n - 1])
                self.robot.stop_online()
            self.robot.deactivate_servo()
        duration = self.clock.now() - t_start

        recorded = len(steps["observation.state"])
        if recorded == 0:
            raise RuntimeError(
                "Keine Schritte aufgezeichnet (Stopp vor dem ersten Takt?)"
            )

        bad_ratio = bad_frames / float(recorded)
        discarded = bool(discard_reason)
        if not discarded and bad_ratio > self.max_bad_ratio:
            discarded = True
            discard_reason = (
                "latenzbudget: %.0f%% der Frames ueber %d ms"
                % (100.0 * bad_ratio, int(self.max_skew * 1000))
            )
        if not discarded and recorded < n:
            discarded = True
            discard_reason = discard_reason or "unvollstaendig"

        arrays = {k: np.stack(v) for k, v in steps.items()}
        if recorded == n:
            arrays["next.done"][-1, 0] = True
        meta = dict(metadata or {})
        meta.update(
            {
                "planned_steps": n,
                "recorded_steps": recorded,
                "bad_frame_ratio": bad_ratio,
                "max_follow_error_rad": worst_follow,
                "pacer_overruns": pacer.overruns,
                "duration_s": duration,
                # Soll-Dauer der GEFAHRENEN Takte -- bei einem Abbruch
                # zaehlt nur, was tatsaechlich gefahren wurde.
                "planned_duration_s": recorded / float(self.rate_hz),
                "servo_j_ms": _ms_stats(servo_s),
                "read_state_ms": _ms_stats(read_s),
                "noise_rejects": plan.rejects,
                "rate_hz": self.rate_hz,
                "servo_rate_hz": self.servo_rate_hz,
                "execution": self.execution,
            }
        )
        if halt:
            # Wartezeiten an den Haltepunkten (Takt, Dauer, Abstand vorher/
            # nachher) -- in duration_s enthalten, nicht in den Schritten.
            meta.update(execution_module.reach_summary(reach_waits))
        return dataset.Episode(
            arrays=arrays,
            metadata=meta,
            discarded=discarded,
            discard_reason=discard_reason,
        )


    def _ausrollen(self, ziel):
        """Wartet (hoechstens ONLINE_SETTLE_S), bis der Arm am letzten Ziel steht."""
        ziel = np.asarray(ziel, dtype=float)
        pacer = Pacer(self.clock, self.rate_hz).start()
        for _ in range(max(1, int(config.ONLINE_SETTLE_S * self.rate_hz))):
            pacer.tick()
            ist = np.asarray(self.robot.read_state().joints, dtype=float)
            if float(np.max(np.abs(ist - ziel))) < config.START_POSE_TOL_RAD:
                return True
        return False


def _to_schema_size(image):
    """Bringt ein Bild auf die Schema-Groesse (AP 0.10: 240x320).

    Identische Vorverarbeitung fuer Aufzeichnung und Inferenz -- wer hier
    etwas aendert, aendert das Schema (config.SCHEMA_VERSION!).
    """
    h, w = config.IMAGE_HEIGHT, config.IMAGE_WIDTH
    if image.shape[:2] == (h, w):
        return np.ascontiguousarray(image, dtype=np.uint8)
    import cv2

    return np.ascontiguousarray(
        cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA), dtype=np.uint8
    )


def _ms_stats(werte_s):
    """median / p95 / max in Millisekunden, oder None ohne Messwerte."""
    if not werte_s:
        return None
    ms = np.sort(np.asarray(werte_s, dtype=float) * 1e3)
    return {
        "median": float(np.median(ms)),
        "p95": float(ms[min(len(ms) - 1, int(round(0.95 * (len(ms) - 1))))]),
        "max": float(ms[-1]),
        "n": int(len(ms)),
    }


def timing_findings(metadata):
    """Hinweise zur Laufzeit einer Episode -- fuer die Ausgabe nach dem Lauf.

    Befund 2026-10-01/02: Laeuft die Taktschleife zu langsam, wird die Bahn
    langsamer abgefahren als geplant -- und die Episode sieht trotzdem
    gesund aus (0 % schlechte Frames, kein Abbruch). Bei 90 und 120 Hz
    Senderate lief sie 1,6-1,7-fach zu langsam, ohne dass etwas warnte. Die
    Ausfuehrungsgeschwindigkeit wird aber mitgelernt (AP 2.6).

    Ursache und Abhilfe werden nur VERMUTET und als solche benannt: hohe
    Aufrufkosten passen zu einem ausgelasteten Rechner (VM auf dem Laptop,
    Akkubetrieb, Energiesparmodus), koennen aber auch von der Steuerung
    kommen. Rueckgabe: Liste von Textzeilen, leer wenn alles passt.
    """
    out = []
    dauer = metadata.get("duration_s")
    soll = metadata.get("planned_duration_s")
    if not dauer or not soll:
        return out
    # Wartezeit an Haltepunkten (online) ist gewollt, kein langsamer Takt.
    dauer -= float(metadata.get("reach_wait_s") or 0.0)
    faktor = dauer / soll
    if faktor <= config.RECORDER_SLOW_FACTOR_WARN:
        return out

    out.append(
        "Bahn lief %.2f-fach langsamer als geplant (%.1f s statt %.1f s). Die "
        "Ausfuehrungsgeschwindigkeit wird mitgelernt (AP 2.6) -- Episode fuer "
        "den Datensatz pruefen." % (faktor, dauer, soll))

    servo = metadata.get("servo_j_ms") or {}
    read = metadata.get("read_state_ms") or {}
    rate = float(metadata.get("rate_hz") or config.CONTROL_RATE_HZ)
    servo_rate = float(metadata.get("servo_rate_hz") or config.SERVO_RATE_HZ)
    online = execution_module.mode_of(metadata) == execution_module.ONLINE
    # "servo_j_ms" haelt bei online die Kosten des online_j-Aufrufs.
    k = 1 if online else int(round(servo_rate / rate))
    if servo.get("median") is not None and read.get("median") is not None:
        takt = k * servo["median"] + read["median"]
        budget = 1e3 / rate
        out.append(
            "Aufrufkosten je Takt: %d x %s (%.1f ms) + read_state "
            "(%.1f ms) = %.0f ms bei %.0f ms Budget."
            % (k, "online_j" if online else "servo_j", servo["median"], read["median"],
               takt, budget))
        if takt > budget and k > 1:
            out.append(
                "Mit %g Hz Senderate passt das nicht in den Takt -- eine "
                "kleinere servo_j-Rate senkt die Last (in der Aufnahme-"
                "Oberflaeche einstellbar)." % servo_rate)
        if servo["median"] > config.SERVO_J_HOST_HINT_MS:
            out.append(
                "servo_j ist mit %.1f ms deutlich teurer als ausgelegt "
                "(~%.1f ms). Das passt zu einem ausgelasteten Rechner -- bei "
                "der VM: Netzteil, Energiemodus 'Beste Leistung', VM neu "
                "starten (Known-Issues-Neura-Sim.md, Punkt 4). Moeglich ist "
                "auch eine langsame Steuerung."
                % (servo["median"], config.SERVO_J_DESIGN_MS))
    return out

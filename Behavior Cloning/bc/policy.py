"""Policy-Schnittstelle, Action Chunking und Mitteln ueberlappender Chunks (AP 4.1).

Die Schnittstelle ist so geschnitten, dass die Inferenzschleife exakt
dieselben Schema-Funktionen benutzt wie der Recorder (dataset.build_state,
dataset.gripper_from_action) -- der Konsistenz-Anker aus AP 1.5.1.

Zeitbezug eines Chunks (verbindlich, passt zum Datensatz): wird im Takt t
mit der Beobachtung von t vorhergesagt, ist ``chunk[0]`` das Ziel fuer
t+1 -- genau wie ``action`` im Datensatz (ideale Sollwinkel bei t+1). Im
Takt t+k gilt ``chunk[k]``.

Die eigentliche Diffusion Policy (torch/lerobot) liegt in bc/diffusion.py,
damit dieses Modul ohne torch importierbar und testbar bleibt.
"""

import abc
import threading
import time

import numpy as np

from . import config


class PolicyPort(abc.ABC):
    """Eine Policy: Beobachtung -> Aktions-Chunk.

    ``observation``: dict mit den Schluesseln aus dataset.features()
    (observation.state (14,), observation.images.* (H,W,3) uint8).
    Rueckgabe von predict: ndarray (horizon, ACTION_DIM) absoluter
    Zielwinkel + Greifer, ``[0]`` fuer den naechsten Takt.
    """

    @abc.abstractmethod
    def reset(self):
        """Vor jedem Episodenstart aufrufen."""

    def observe(self, observation):
        """Jede Beobachtung im 15-Hz-Takt, auch ohne Vorhersage.

        Policies mit Beobachtungshistorie (n_obs_steps > 1) brauchen JEDEN
        Takt, sonst saehe die Vorhersage Luecken, die es im Training nie
        gab. Default: nichts zu tun.
        """

    @abc.abstractmethod
    def predict(self, observation):
        pass

    def snapshot(self, observation):
        """Unveraenderlicher Stand der Beobachtungshistorie fuer eine
        Vorhersage in einem anderen Thread (:class:`AsyncPredictor`).
        Default: die Beobachtung selbst."""
        return observation

    def predict_snapshot(self, snapshot):
        """Vorhersage aus :meth:`snapshot` -- darf in einem anderen Thread
        laufen, waehrend ``observe`` weiterlaeuft."""
        return self.predict(snapshot)


class PredictionTooSlow(RuntimeError):
    """Fuer den aktuellen Takt liegt keine Vorhersage mehr vor -- die
    Vorhersage dauert laenger, als die Chunks reichen (AP 4.1)."""


class AsyncPredictor(object):
    """Vorhersage in einem eigenen Thread (AP 4.1).

    Befund 2026-09-17 (RTX 5070 Ti, Windows): eine Vorhersage mit 10
    DDIM-Schritten dauert ~77 ms, laenger als ein 15-Hz-Takt. Synchron im
    Takt wuerde sie den 60-Hz-Servostrom jedes Mal anhalten -- der Arm folgte
    dann zeitlich anders als in der Aufzeichnung. Asynchron laeuft der
    Servostrom ungestoert weiter; der fertige Chunk wird ueber den Takt
    seiner BEOBACHTUNG eingeordnet, die bei Ankunft schon vergangenen
    Eintraege verfallen einfach.

    Genau ein Auftrag gleichzeitig -- ein neuer wird erst angenommen, wenn
    der vorige fertig ist (keine Warteschlange veralteter Beobachtungen).
    """

    def __init__(self, policy):
        self.policy = policy
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._done = threading.Event()
        self._job = None
        self._results = []
        self._error = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="policy-predict", daemon=True)
        self._thread.start()

    @property
    def busy(self):
        with self._lock:
            return self._job is not None

    def submit(self, step, snapshot):
        with self._lock:
            if self._job is not None:
                return False
            self._job = (step, snapshot)
            self._done.clear()
        self._wake.set()
        return True

    def poll(self):
        """Fertige Vorhersagen: Liste (Takt, Chunk, Dauer s). Wirft Fehler
        aus dem Vorhersage-Thread hier, im Takt-Thread."""
        with self._lock:
            if self._error is not None:
                error, self._error = self._error, None
                raise error
            results, self._results = self._results, []
        return results

    def wait(self, timeout=5.0):
        return self._done.wait(timeout)

    def close(self):
        self._running = False
        self._wake.set()
        self._thread.join(timeout=2.0)

    def _loop(self):
        while True:
            self._wake.wait()
            self._wake.clear()
            if not self._running:
                return
            with self._lock:
                job = self._job
            if job is None:
                continue
            step, snapshot = job
            t0 = time.perf_counter()
            try:
                chunk = self.policy.predict_snapshot(snapshot)
                result = (step, chunk, time.perf_counter() - t0)
                with self._lock:
                    self._results.append(result)
                    self._job = None
            except Exception as exc:  # im Takt-Thread weiterwerfen
                with self._lock:
                    self._error = exc
                    self._job = None
            self._done.set()


class HoldPolicy(PolicyPort):
    """Triviale Platzhalter-Policy: haelt die aktuelle Gelenkstellung.

    Dient dem Verdrahtungstest der Inferenzschleife (Schema, Timing,
    Sicherheit), ohne ein Modell zu brauchen.
    """

    def __init__(self, horizon=8):
        self.horizon = horizon

    def reset(self):
        pass

    def predict(self, observation):
        state = np.asarray(observation["observation.state"], dtype=np.float32)
        joints = state[:6]
        gripper = state[13:14]
        action = np.concatenate([joints, gripper])
        return np.tile(action, (self.horizon, 1))


def _validate_chunk(chunk):
    chunk = np.asarray(chunk, dtype=np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != config.ACTION_DIM or len(chunk) == 0:
        raise ValueError(
            "Policy lieferte Form %s, erwartet (horizon, %d)"
            % (chunk.shape, config.ACTION_DIM)
        )
    if not np.all(np.isfinite(chunk)):
        raise ValueError("Policy lieferte nicht endliche Werte")
    return chunk


class ChunkExecutor(object):
    """Action Chunking ohne Mitteln: praedizieren, n Schritte ausfuehren, neu.

    Kompromiss explizit: grosses ``n_execute`` entlastet die GPU, verzoegert
    aber die Reaktion auf Stoerungen (Hand-Szenario AP 5.1). Fuer die
    Inferenz ist :class:`ChunkEnsembler` der Standard; dieser Executor
    bleibt als Vergleich (``--no-ensemble``).
    """

    def __init__(self, policy, n_execute=4):
        self.policy = policy
        self.n_execute = n_execute
        self._chunk = None
        self._cursor = 0
        self.predictions = 0

    def reset(self):
        self.policy.reset()
        self._chunk = None
        self._cursor = 0
        self.predictions = 0

    def next_action(self, observation):
        """Liefert die naechste Einzel-Action (ACTION_DIM,)."""
        self.policy.observe(observation)
        if self._chunk is None or self._cursor >= min(self.n_execute, len(self._chunk)):
            self._chunk = _validate_chunk(self.policy.predict(observation))
            self.predictions += 1
            self._cursor = 0
        action = self._chunk[self._cursor]
        self._cursor += 1
        return action


class ChunkEnsembler(object):
    """Mittelt ueberlappende Chunks (AP 4.1, Befund Rauschstudie 2026-09-16).

    Alle ``replan_steps`` Takte wird neu vorhergesagt. Fuer den aktuellen
    Takt liegen dann bis zu ``len(chunk) / replan_steps`` Vorhersagen vor
    (aus verschiedenen Beobachtungen); ausgefuehrt wird ihr gewichtetes
    Mittel mit ``w = exp(-decay * Alter)``. Effekt: der Wechsel von einem
    Chunk zum naechsten ist kein Sprung mehr, und ein einzelner
    Ausreisser-Sample der Diffusion wird gedaempft.

    Der Greiferkanal wird ebenfalls gemittelt und danach wie immer ueber
    config.GRIPPER_THRESHOLD binarisiert -- er schaltet also, wenn die
    Mehrheit (gewichtet) umgeschaltet hat.

    Diagnose: ``last_spread`` = groesste Abweichung (rad) zwischen den
    gemittelten Vorhersagen im letzten Takt. Grosse Werte heissen: die
    Policy ist sich uneinig (z. B. zwei Modi) -- das Mittel liegt dann
    zwischen den Modi.

    ``asynchronous=True``: Vorhersagen laufen im :class:`AsyncPredictor`.
    Faellig ist eine neue Vorhersage alle ``replan_steps`` Takte; laeuft
    noch eine, wird gewartet, bis sie fertig ist (``skipped``). Reicht
    kein Chunk mehr bis zum aktuellen Takt, wird auf die laufende
    Vorhersage gewartet (nur beim Start normal), sonst
    :class:`PredictionTooSlow`. ``last_delay_steps``: um wie viele Takte
    der zuletzt eingetroffene Chunk hinter seiner Beobachtung lag.
    """

    def __init__(
        self,
        policy,
        replan_steps=config.POLICY_REPLAN_STEPS,
        decay=config.POLICY_ENSEMBLE_DECAY,
        asynchronous=False,
    ):
        if replan_steps < 1:
            raise ValueError("replan_steps muss >= 1 sein")
        self.policy = policy
        self.replan_steps = int(replan_steps)
        self.decay = float(decay)
        self.asynchronous = bool(asynchronous)
        self._predictor = AsyncPredictor(policy) if self.asynchronous else None
        self.reset_state()

    def reset_state(self):
        self._chunks = []  # Liste (Takt der Beobachtung, chunk)
        self._step = 0
        self._last_request = None
        self.predictions = 0
        self.skipped = 0
        self.last_spread = 0.0
        self.last_count = 0
        self.last_predict_s = None
        self.last_delay_steps = 0

    def reset(self):
        if self._predictor is not None and self._predictor.busy:
            self._predictor.wait()
            self._predictor.poll()
        self.policy.reset()
        self.reset_state()

    def close(self):
        if self._predictor is not None:
            self._predictor.close()

    def _add(self, step, chunk, seconds):
        chunk = _validate_chunk(chunk)
        if len(chunk) < self.replan_steps:
            raise ValueError(
                "Chunk (%d Schritte) kuerzer als replan_steps (%d) -- Luecke"
                % (len(chunk), self.replan_steps)
            )
        self._chunks.append((step, chunk))
        self.predictions += 1
        self.last_predict_s = seconds
        self.last_delay_steps = self._step - step

    def _collect(self):
        for s, chunk, seconds in self._predictor.poll():
            self._add(s, chunk, seconds)

    def _covered(self, step):
        return any(s <= step < s + len(c) for s, c in self._chunks)

    def next_action(self, observation):
        step = self._step
        self.policy.observe(observation)
        if self.asynchronous:
            self._collect()
        self._chunks = [(s, c) for s, c in self._chunks if step < s + len(c)]

        due = self._last_request is None or step - self._last_request >= self.replan_steps
        if due:
            if not self.asynchronous:
                t0 = time.perf_counter()
                chunk = self.policy.predict_snapshot(self.policy.snapshot(observation))
                self._add(step, chunk, time.perf_counter() - t0)
                self._last_request = step
            elif self._predictor.submit(step, self.policy.snapshot(observation)):
                self._last_request = step
            else:
                self.skipped += 1

        if not self._covered(step) and self.asynchronous:
            if self._predictor.busy:
                # Takt 0 (vor dem Servostrom) darf lange warten -- die erste
                # Vorhersage in einem neuen Thread ist langsamer.
                self._predictor.wait(timeout=60.0 if step == 0 else 5.0)
            self._collect()
        if not self._covered(step):
            raise PredictionTooSlow(
                "Takt %d: keine gueltige Vorhersage (letzte Dauer %s s, Chunks %s)"
                % (step, self.last_predict_s, [(s, len(c)) for s, c in self._chunks])
            )

        self._chunks = [(s, c) for s, c in self._chunks if s <= step < s + len(c)]
        actions = np.array([c[step - s] for s, c in self._chunks])
        ages = np.array([step - s for s, _ in self._chunks], dtype=float)
        weights = np.exp(-self.decay * ages)
        action = (weights[:, None] * actions).sum(axis=0) / weights.sum()
        self.last_count = len(actions)
        self.last_spread = float(np.max(np.ptp(actions[:, :6], axis=0))) if len(actions) > 1 else 0.0
        self._step += 1
        return action


def check_action(action, measured_joints, guard):
    """Plausibilitaet einer Action gegen die gemessene Stellung (AP 4.2).

    ``guard``: servo.ServoGuard (dieselben Grenzen wie im Adapter). Der
    Adapter prueft jeden Zwischenschritt auf Sprung/Geschwindigkeit; hier
    kommt pro Takt der Abstand Ziel <-> Ist-Stellung dazu, den der Adapter
    ohne frische Messung nicht kennt. Wirft ServoLimitError.
    """
    action = np.asarray(action, dtype=float)
    if action.shape != (config.ACTION_DIM,) or not np.all(np.isfinite(action)):
        from .ports import ServoLimitError

        raise ServoLimitError("Action ungueltig: %s" % np.round(action, 4).tolist())
    guard.check_gap(action[:6], measured_joints)

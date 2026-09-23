"""Policy-Schnittstelle, Action Chunking und Mitteln der Chunks (AP 4.1)."""

import _paths  # noqa: F401

import numpy as np

from bc import config, metrics
from bc.policy import ChunkEnsembler, ChunkExecutor, HoldPolicy, PolicyPort, check_action
from bc.ports import ServoLimitError
from bc.servo import ServoGuard


class RampPolicy(PolicyPort):
    """Chunk = Zeitindex der Vorhersage + Offset je Schritt (pruefbar)."""

    def __init__(self, length=8):
        self.length = length
        self.calls = 0
        self.observed = 0

    def reset(self):
        self.calls = 0
        self.observed = 0

    def observe(self, observation):
        self.observed += 1

    def predict(self, observation):
        t = self.observed - 1  # Takt der Vorhersage
        self.calls += 1
        chunk = np.zeros((self.length, config.ACTION_DIM))
        for k in range(self.length):
            chunk[k, :6] = t + k + 1 + 0.1 * self.calls  # Ziel fuer Takt t+k+1 (+ Kennung)
        return chunk


def test_ensembler_time_alignment_and_average():
    policy = RampPolicy(length=8)
    ens = ChunkEnsembler(policy, replan_steps=2, decay=0.0)
    ens.reset()
    actions = [ens.next_action(_obs())[0] for _ in range(6)]
    # Jede Vorhersage zielt auf t+1 -- ohne die Kennung waere action == t + 1
    assert policy.observed == 6 and policy.calls == 3
    assert abs(actions[0] - (1 + 0.1)) < 1e-9
    assert abs(actions[1] - (2 + 0.1)) < 1e-9
    # Takt 2: Chunk 1 (Kennung 0.1) und Chunk 2 (0.2) ueberlappen -> Mittel
    assert abs(actions[2] - (3 + 0.15)) < 1e-9
    # Takt 4: drei Chunks (0.1, 0.2, 0.3)
    assert abs(actions[4] - (5 + 0.2)) < 1e-9
    assert ens.last_count == 3


def test_ensembler_decay_prefers_newer_and_reports_spread():
    policy = RampPolicy(length=8)
    ens = ChunkEnsembler(policy, replan_steps=2, decay=5.0)
    ens.reset()
    for _ in range(3):
        action = ens.next_action(_obs())
    # starkes decay -> fast nur der neueste Chunk (Kennung 0.2)
    assert abs(action[0] - (3 + 0.2)) < 0.01
    assert abs(ens.last_spread - 0.1) < 1e-9


def test_ensembler_rejects_short_chunks_and_nan():
    ens = ChunkEnsembler(RampPolicy(length=1), replan_steps=2)
    ens.reset()
    try:
        ens.next_action(_obs())
        assert False, "ValueError erwartet (Luecke)"
    except ValueError:
        pass

    class NanPolicy(HoldPolicy):
        def predict(self, observation):
            chunk = super().predict(observation)
            chunk[0, 0] = np.nan
            return chunk

    ens = ChunkEnsembler(NanPolicy(), replan_steps=2)
    try:
        ens.next_action(_obs())
        assert False, "ValueError erwartet (NaN)"
    except ValueError:
        pass


def test_async_ensembler_keeps_time_alignment_despite_latency():
    import time

    class SlowAbsolutePolicy(PolicyPort):
        """Ziel = absoluter Takt + 1 -- jede korrekt eingeordnete Vorhersage
        liefert fuer denselben Takt denselben Wert, egal wie spaet sie kommt."""

        def __init__(self):
            self.observed = 0

        def reset(self):
            self.observed = 0

        def observe(self, observation):
            self.observed += 1

        def predict(self, observation):
            raise AssertionError("asynchron muss predict_snapshot benutzt werden")

        def snapshot(self, observation):
            return self.observed - 1  # Takt der Beobachtung

        def predict_snapshot(self, step):
            time.sleep(0.03)  # laenger als ein "Takt" unten
            chunk = np.zeros((8, config.ACTION_DIM))
            chunk[:, :6] = (step + 1 + np.arange(8))[:, None]
            return chunk

    ens = ChunkEnsembler(SlowAbsolutePolicy(), replan_steps=2, asynchronous=True)
    try:
        ens.reset()
        for step in range(12):
            action = ens.next_action(_obs())
            assert abs(action[0] - (step + 1)) < 1e-9, (step, action[0])
            time.sleep(0.02)
        assert ens.predictions >= 3
        assert ens.last_delay_steps >= 1  # Chunks kamen verspaetet an
    finally:
        ens.close()


def test_check_action_gap():
    guard = ServoGuard(max_gap_rad=0.3)
    action = np.zeros(config.ACTION_DIM)
    check_action(action, np.full(6, 0.2), guard)
    try:
        check_action(action, np.full(6, 0.5), guard)
        assert False, "ServoLimitError erwartet"
    except ServoLimitError:
        pass


def test_evaluate_rollout_against_reference():
    path = [[0.0, 0.0, 0.3], [0.1, 0.0, 0.3], [0.1, 0.0, 0.2], [0.2, 0.0, 0.3]]
    reference = {"path": path, "grasp_tcp": [[0.1, 0.0, 0.2]], "end_tcp": [[0.2, 0.0, 0.3]]}
    tcp = np.array([[0.0, 0.0, 0.3], [0.1, 0.003, 0.3], [0.1, 0.0, 0.205], [0.2, 0.0, 0.3]])
    tcp = np.hstack([tcp, np.tile([0, 0, 1, 0], (4, 1))])
    gripper = np.array([0.0, 1.0, 1.0, 1.0])  # Befehl in Takt 1, erreicht in Takt 2
    result = metrics.evaluate_rollout(
        {"tcp": tcp, "gripper_cmd": gripper, "reached_end": True}, reference
    )
    assert result["grasp_step"] == 2
    assert abs(result["grasp_error_m"] - 0.005) < 1e-9
    assert result["end_error_m"] < 1e-9
    assert abs(result["path_dev_max_m"] - 0.003) < 1e-9
    assert result["success"]
    missed = metrics.evaluate_rollout(
        {"tcp": tcp, "gripper_cmd": np.zeros(4), "reached_end": True}, reference
    )
    assert not missed["success"] and missed["grasp_step"] is None


def _obs(joints=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6), gripper=0.0):
    state = np.zeros(config.STATE_DIM, dtype=np.float32)
    state[:6] = joints
    state[13] = gripper
    return {"observation.state": state}


def test_hold_policy_holds():
    policy = HoldPolicy(horizon=8)
    chunk = policy.predict(_obs())
    assert chunk.shape == (8, config.ACTION_DIM)
    assert np.allclose(chunk[0, :6], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    assert chunk[0, 6] == 0.0


def test_chunk_executor_repredicts_after_n():
    calls = []

    class CountingPolicy(HoldPolicy):
        def predict(self, observation):
            calls.append(1)
            return super().predict(observation)

    executor = ChunkExecutor(CountingPolicy(horizon=8), n_execute=4)
    executor.reset()
    for _ in range(10):
        action = executor.next_action(_obs())
        assert action.shape == (config.ACTION_DIM,)
    # 10 Schritte bei n_execute=4 -> 3 Praediktionen (4+4+2)
    assert sum(calls) == 3


def test_chunk_executor_validates_shape():
    class BadPolicy(HoldPolicy):
        def predict(self, observation):
            return np.zeros((8, 3))  # falsche Action-Dimension

    executor = ChunkExecutor(BadPolicy())
    try:
        executor.next_action(_obs())
        assert False, "ValueError erwartet"
    except ValueError:
        pass

"""Policy-Schnittstelle und Action Chunking (AP 4.1, Geruest)."""

import _paths  # noqa: F401

import numpy as np

from bc import config
from bc.policy import ChunkExecutor, HoldPolicy


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

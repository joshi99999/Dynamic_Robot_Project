"""Policy-Schnittstelle und Action Chunking (AP 4.1) -- GERUEST.

Umfangsfestlegung AP 0.10: AP 3-5 werden zunaechst nur als Geruest mit
festen Schnittstellen angelegt. Die Diffusion-Policy-Implementierung folgt,
sobald torch/lerobot gepinnt und installiert sind (AP 0.9 Punkt 6/9).

Die Schnittstelle ist so geschnitten, dass die Inferenzschleife exakt
dieselben Schema-Funktionen benutzt wie der Recorder (dataset.build_state,
dataset.gripper_from_action) -- der Konsistenz-Anker aus AP 1.5.1.
"""

import abc

import numpy as np

from . import config


class PolicyPort(abc.ABC):
    """Eine Policy: Beobachtung -> Aktions-Chunk.

    ``observation``: dict mit den Schluesseln aus dataset.features()
    (observation.state (14,), observation.images.* (H,W,3) uint8).
    Rueckgabe: ndarray (horizon, ACTION_DIM) absoluter Zielwinkel+Greifer.
    """

    @abc.abstractmethod
    def reset(self):
        """Vor jedem Episodenstart aufrufen."""

    @abc.abstractmethod
    def predict(self, observation):
        pass


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


class ChunkExecutor(object):
    """Action Chunking (AP 4.1): prädizieren, n Schritte ausfuehren, neu.

    Kompromiss explizit: grosses ``n_execute`` entlastet die GPU, verzoegert
    aber die Reaktion auf Stoerungen (Hand-Szenario AP 5.1). Wert ist eine
    offene Entscheidung (AP 0.9 Punkt 7).
    """

    def __init__(self, policy, n_execute=4):
        self.policy = policy
        self.n_execute = n_execute
        self._chunk = None
        self._cursor = 0

    def reset(self):
        self.policy.reset()
        self._chunk = None
        self._cursor = 0

    def next_action(self, observation):
        """Liefert die naechste Einzel-Action (ACTION_DIM,)."""
        if self._chunk is None or self._cursor >= min(
            self.n_execute, len(self._chunk)
        ):
            self._chunk = np.asarray(self.policy.predict(observation))
            if self._chunk.ndim != 2 or self._chunk.shape[1] != config.ACTION_DIM:
                raise ValueError(
                    "Policy lieferte Form %s, erwartet (horizon, %d)"
                    % (self._chunk.shape, config.ACTION_DIM)
                )
            self._cursor = 0
        action = self._chunk[self._cursor]
        self._cursor += 1
        return action


class DiffusionPolicy(PolicyPort):
    """GERUEST: CNN-basierte Diffusion Policy (AP 3.2).

    Implementierung folgt nach dem Pinnen von torch (cu128, sm_120 --
    siehe tools/check_gpu.py) und lerobot. Geplant: ResNet18-Backbones je
    Kamera (ImageNet-vortrainiert), Denoising-Kopf, Action-Horizon 8-16.
    """

    def __init__(self, checkpoint_path):
        raise NotImplementedError(
            "Diffusion Policy folgt in der naechsten Ausbaustufe: erst "
            "torch/lerobot pinnen und installieren (AP 0.9 Punkt 6/9), "
            "dann gegen tools/check_gpu.py verifizieren."
        )

    def reset(self):
        raise NotImplementedError

    def predict(self, observation):
        raise NotImplementedError

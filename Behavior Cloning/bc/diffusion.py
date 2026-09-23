"""Diffusion Policy auf Basis von lerobot (AP 3.2 / AP 4.1).

Baut die CNN-Diffusion-Policy aus lerobot 0.6.1 (ResNet18 je Kamera,
ImageNet-vortrainiert, 1D-U-Net) fuer UNSER Schema und stellt drei Dinge
bereit, die apps/train.py und apps/infer.py teilen:

* :func:`build_config` -- Policy-Konfiguration aus dem exportierten
  Datensatz (Features, Bildgroesse, Horizont; Werte in config.POLICY_*).
* :func:`save_policy` / :func:`load_policy` -- Checkpoint = lerobot-Format
  (``model.safetensors``, ``config.json``, Pre-/Postprozessor mit den
  Normierungsstatistiken) plus ``bc_policy.json`` mit allem, was die
  Inferenz gegen die Aufzeichnung pruefen muss (Schema, Rate, Kameras,
  Override, servo_j-Rate, Start-/Endstellung, lerobot-Version).
* :class:`DiffusionPolicyRunner` -- :class:`bc.policy.PolicyPort` fuer die
  Inferenzschleife: dieselbe Bildvorverarbeitung wie der Datensatz
  (uint8 HWC -> float CHW in [0, 1]), Beobachtungshistorie je Takt.

HARDWAREUNABHAENGIG (AP 3.1, Training ggf. auf 5090 oder anderem Rechner):
nichts hier setzt eine bestimmte GPU voraus. Geraet und Mixed Precision
werden zur Laufzeit gewaehlt (:func:`select_device`, :func:`select_amp`),
der Checkpoint ist geraeteneutral und laedt auf CUDA, MPS oder CPU. Einzige
harte Pruefung: die CUDA-Architektur der Karte muss im torch-Build stecken
(Blackwell braucht sm_120 -- cu128 oder neuer), sonst scheitert es erst
beim ersten Kernel mit einer unverstaendlichen Meldung.

torch und lerobot werden hier auf Modulebene importiert -- dieses Modul
ist nur fuer Training und Inferenz gedacht; bc.policy bleibt torch-frei.
"""

import json
import platform
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

from . import config

#: Checkpoint-Datei mit den Bezuegen zur Aufzeichnung (neben lerobots Dateien).
POLICY_INFO = "bc_policy.json"


# --------------------------------------------------------------------------
# Geraet und Genauigkeit
# --------------------------------------------------------------------------


def select_device(requested="auto"):
    """``auto`` -> cuda, sonst mps, sonst cpu. Prueft die CUDA-Architektur."""
    if requested == "auto":
        if torch.cuda.is_available():
            requested = "cuda"
        elif getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            requested = "mps"
        else:
            requested = "cpu"
    device = torch.device(requested)
    if device.type == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA angefordert, aber torch.cuda.is_available() ist False")
        major, minor = torch.cuda.get_device_capability(device)
        arch = "sm_%d%d" % (major, minor)
        if arch not in torch.cuda.get_arch_list():
            raise RuntimeError(
                "GPU %s braucht %s, der torch-Build %s kennt nur %s. Passenden Build "
                "installieren (Blackwell/RTX 50xx: cu128 oder neuer), siehe "
                "tools/check_gpu.py." % (torch.cuda.get_device_name(device), arch,
                                         torch.__version__, torch.cuda.get_arch_list())
            )
    return device


def select_amp(device, requested="auto"):
    """Mixed Precision: ``bf16`` wo die Hardware es kann, sonst aus.

    bf16 braucht keinen GradScaler und ist numerisch unkritisch (Ampere und
    neuer: 30xx/40xx/50xx). fp16 wird bewusst nicht angeboten.
    """
    if requested == "off":
        return None
    supported = device.type == "cuda" and torch.cuda.is_bf16_supported()
    if requested == "bf16" and not supported:
        raise RuntimeError("bf16 auf %s nicht verfuegbar" % device)
    return torch.bfloat16 if supported else None


def hardware_info(device):
    info = {
        "device": str(device),
        "torch": torch.__version__,
        "cuda_build": torch.version.cuda,
        "platform": platform.platform(),
        "python": platform.python_version(),
    }
    if device.type == "cuda":
        props = torch.cuda.get_device_properties(device)
        info.update(gpu=props.name, vram_gb=round(props.total_memory / 1024**3, 1))
    return info


# --------------------------------------------------------------------------
# Konfiguration
# --------------------------------------------------------------------------


def image_keys(cameras):
    return ["observation.images.%s" % name for name in cameras]


def build_config(
    ds_meta,
    cameras,
    device,
    n_obs_steps=config.POLICY_N_OBS_STEPS,
    horizon=config.POLICY_HORIZON,
    n_action_steps=config.POLICY_N_ACTION_STEPS,
    down_dims=config.POLICY_DOWN_DIMS,
    crop_ratio=config.POLICY_CROP_RATIO,
    inference_steps=config.POLICY_INFERENCE_STEPS,
    pretrained_backbone=True,
    use_amp=False,
    prediction_type=config.POLICY_PREDICTION_TYPE,
):
    """DiffusionConfig fuer das bc-Schema aus den Metadaten des Datensatzes.

    Eingaenge sind genau ``observation.state`` und die Kamerabilder --
    ``aux.*`` und ``next.done`` sind Diagnose, nie Policy-Eingang.
    DDIM als Scheduler: gleicher Vorwaertsprozess wie DDPM im Training,
    aber mit wenigen Schritten bei der Inferenz (AP 4.1).
    """
    from lerobot.policies.diffusion.configuration_diffusion import DiffusionConfig
    from lerobot.utils.feature_utils import dataset_to_policy_features

    feats = dataset_to_policy_features(ds_meta.features)
    wanted = ["observation.state"] + image_keys(cameras)
    missing = [k for k in wanted + ["action"] if k not in feats]
    if missing:
        raise ValueError("Datensatz ohne Features %s" % missing)
    if n_action_steps > horizon - n_obs_steps + 1:
        raise ValueError(
            "n_action_steps (%d) > horizon - n_obs_steps + 1 (%d)"
            % (n_action_steps, horizon - n_obs_steps + 1)
        )
    return DiffusionConfig(
        n_obs_steps=n_obs_steps,
        horizon=horizon,
        n_action_steps=n_action_steps,
        drop_n_last_frames=horizon - n_action_steps - n_obs_steps + 1,
        input_features={k: feats[k] for k in wanted},
        output_features={"action": feats["action"]},
        resize_shape=(config.IMAGE_HEIGHT, config.IMAGE_WIDTH),
        crop_ratio=crop_ratio,
        pretrained_backbone_weights="ResNet18_Weights.IMAGENET1K_V1" if pretrained_backbone else None,
        use_group_norm=not pretrained_backbone,
        down_dims=tuple(down_dims),
        noise_scheduler_type="DDIM",
        num_inference_steps=inference_steps,
        prediction_type=prediction_type,
        device=device.type,
        use_amp=use_amp,
        push_to_hub=False,
    )


def delta_timestamps(policy_cfg, cameras, fps=config.CONTROL_RATE_HZ):
    """Zeitversaetze fuer LeRobotDataset: Beobachtungshistorie + Aktionshorizont."""
    obs = [i / fps for i in policy_cfg.observation_delta_indices]
    out = {key: obs for key in ["observation.state"] + image_keys(cameras)}
    out["action"] = [i / fps for i in policy_cfg.action_delta_indices]
    return out


# --------------------------------------------------------------------------
# Checkpoints
# --------------------------------------------------------------------------


def save_policy(directory, policy, preprocessor, postprocessor, info):
    """Speichert Policy + Prozessoren (lerobot) + bc_policy.json."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    policy.save_pretrained(directory)
    preprocessor.save_pretrained(directory)
    postprocessor.save_pretrained(directory)
    (directory / POLICY_INFO).write_text(
        json.dumps(info, indent=2, ensure_ascii=False, default=_json_default), encoding="utf-8"
    )


def read_policy_info(directory):
    path = Path(directory) / POLICY_INFO
    if not path.is_file():
        raise FileNotFoundError(
            "%s fehlt -- kein mit apps/train.py erzeugter Checkpoint" % path
        )
    return json.loads(path.read_text(encoding="utf-8"))


def load_policy(directory, device="auto", inference_steps=None):
    """Laedt einen Checkpoint auf beliebiger Hardware.

    Rueckgabe: (policy, preprocessor, postprocessor, info). Das Geraet im
    gespeicherten config.json wird ueberschrieben -- trainiert auf CUDA,
    lauffaehig auf einem Laptop ohne GPU (dann langsam, siehe AP 4.1).
    """
    from lerobot.configs import PreTrainedConfig
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

    directory = Path(directory)
    info = read_policy_info(directory)
    dev = select_device(device)
    cfg = PreTrainedConfig.from_pretrained(directory)
    cfg.device = dev.type
    # Beim Laden keine ImageNet-Gewichte nachladen -- sie stecken schon im
    # Checkpoint, und der Inferenzrechner ist evtl. offline.
    cfg.pretrained_backbone_weights = None
    if inference_steps is not None:
        cfg.num_inference_steps = int(inference_steps)
    policy = DiffusionPolicy.from_pretrained(directory, config=cfg)
    policy.to(dev)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=str(directory),
        preprocessor_overrides={"device_processor": {"device": dev.type}},
    )
    return policy, preprocessor, postprocessor, info


def _json_default(value):
    if isinstance(value, (np.ndarray, torch.Tensor)):
        return np.asarray(value).tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError("nicht serialisierbar: %r" % type(value))


def timestamp():
    return datetime.now().isoformat(timespec="seconds")


# --------------------------------------------------------------------------
# Inferenz
# --------------------------------------------------------------------------


def observation_to_tensors(observation, cameras):
    """Beobachtung (Schema) -> Tensoren wie aus dem LeRobotDataset.

    Bilder: uint8 (H, W, 3) -> float32 (3, H, W) in [0, 1] -- exakt die
    Darstellung, die lerobot beim Dekodieren der Trainingsvideos liefert.
    """
    out = {
        "observation.state": torch.from_numpy(
            np.asarray(observation["observation.state"], dtype=np.float32)
        )
    }
    for key in image_keys(cameras):
        image = np.asarray(observation[key])
        if image.shape != (config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3) or image.dtype != np.uint8:
            raise ValueError("%s: Form %s/%s statt Schema-Bild" % (key, image.shape, image.dtype))
        out[key] = torch.from_numpy(image).permute(2, 0, 1).contiguous().float() / 255.0
    return out


class DiffusionPolicyRunner(object):
    """PolicyPort-Implementierung (bc.policy) fuer einen Checkpoint.

    ``observe`` muss jeden Takt laufen (Historie ``n_obs_steps``),
    ``predict`` liefert (n_action_steps, 7) in physikalischen Einheiten.
    Die Vorhersage nutzt die Warteschlangen der lerobot-Policy genauso wie
    deren ``select_action`` -- nur ohne deren eigenes Chunk-Abarbeiten, das
    uebernimmt bc.policy.ChunkEnsembler.
    """

    def __init__(self, directory, device="auto", inference_steps=None, seed=None):
        from lerobot.policies.utils import populate_queues

        self._populate_queues = populate_queues
        self.policy, self.pre, self.post, self.info = load_policy(
            directory, device=device, inference_steps=inference_steps
        )
        self.cameras = list(self.info["cameras"])
        self.device = next(self.policy.parameters()).device
        if self.device.type == "cuda":
            # AUS, nicht an: cuDNN legt je Thread eigene Handles an, und mit
            # benchmark=True sucht die erste Vorhersage im Vorhersage-Thread
            # alle Faltungsalgorithmen neu -- gemessen 2026-09-17: 28.8 s statt
            # 138 ms, danach kein Geschwindigkeitsvorteil (~100 ms beide).
            torch.backends.cudnn.benchmark = False
        self._last_batch = None
        if seed is not None:
            torch.manual_seed(int(seed))
        self.last_predict_s = None

    # PolicyPort ----------------------------------------------------------

    def reset(self):
        self.policy.reset()
        self._last_batch = None

    @torch.no_grad()
    def observe(self, observation):
        batch = self.pre(observation_to_tensors(observation, self.cameras))
        # Der Praeprozessor liefert u. a. "action": None mit -- das darf nicht
        # in die Warteschlangen der Policy (sonst None in der Aktions-Queue).
        batch = {k: v for k, v in batch.items()
                 if k.startswith("observation.") and isinstance(v, torch.Tensor)}
        batch["observation.images"] = torch.stack(
            [batch[k] for k in image_keys(self.cameras)], dim=-4
        )
        self.policy._queues = self._populate_queues(self.policy._queues, batch)
        self._last_batch = batch

    def snapshot(self, observation=None):
        """Historie als neue Tensoren (B=1, n_obs, ...) -- unabhaengig von
        weiteren ``observe``-Aufrufen, also in einem anderen Thread nutzbar."""
        if self._last_batch is None:
            raise RuntimeError("observe() vor predict() aufrufen (Beobachtungshistorie)")
        queues = self.policy._queues
        return {k: torch.stack(list(queues[k]), dim=1)
                for k in self._last_batch if k in queues}

    @torch.no_grad()
    def predict_snapshot(self, snapshot):
        import time

        t0 = time.perf_counter()
        # wie DiffusionPolicy.predict_action_chunk mit gefuellten Queues
        actions = self.policy.diffusion.generate_actions(snapshot)  # (1, n, 7) normiert
        actions = self.post(actions)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        self.last_predict_s = time.perf_counter() - t0
        return actions[0].float().cpu().numpy()

    def predict(self, observation):
        return self.predict_snapshot(self.snapshot(observation))

    def warmup(self, observation, repeats=3):
        """Erste Vorhersagen sind langsam (cuDNN-Auswahl, Speicher) -- vor
        dem Aktivieren des Servo-Interface abarbeiten."""
        for _ in range(repeats):
            self.observe(observation)
            self.predict(observation)
        self.reset()
        return self.last_predict_s

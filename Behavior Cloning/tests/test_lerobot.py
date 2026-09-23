"""Export ins LeRobotDataset und Diffusion-Policy-Kette (AP 3 / 0.9 Punkt 6).

Die Auswahl- und Konsistenzpruefung laeuft ohne lerobot. Export, Training
und Laden nur, wenn lerobot/torch installiert sind (sonst uebersprungen und
gemeldet) -- auf CPU mit einem Mini-Modell, damit der Test hardwarefrei
und in Sekunden bleibt.
"""

import json

import _paths  # noqa: F401

import numpy as np

from bc import config, dataset, lerobot_io


def _episode(n=24, override=1.0, success=True, cameras=("wrist", "scene"), seed=0):
    rng = np.random.default_rng(seed)
    arrays = {
        "observation.state": np.zeros((n, config.STATE_DIM), dtype=np.float32),
        "action": np.zeros((n, config.ACTION_DIM), dtype=np.float32),
        "aux.joints_ideal": np.zeros((n, 6), dtype=np.float32),
        "aux.joints_command": np.zeros((n, 6), dtype=np.float32),
        "aux.pose_ideal": np.zeros((n, 7), dtype=np.float32),
        "aux.pose_noisy": np.zeros((n, 7), dtype=np.float32),
        "aux.sync_ok": np.ones((n, 1), dtype=bool),
        "next.done": np.zeros((n, 1), dtype=bool),
    }
    ramp = np.linspace(0.0, 0.5, n, dtype=np.float32)
    arrays["observation.state"][:, :6] = ramp[:, None]
    arrays["observation.state"][n // 2:, 13] = 1.0
    arrays["action"][:, :6] = ramp[:, None] + 0.02
    arrays["action"][n // 2 - 1:, 6] = 1.0
    arrays["aux.joints_ideal"][:] = ramp[:, None]
    arrays["aux.pose_ideal"][:, 0] = 0.4 + ramp * 0.2
    arrays["aux.pose_ideal"][:, 2] = 0.3 - ramp * 0.1
    arrays["aux.pose_ideal"][:, 5] = 1.0
    arrays["next.done"][-1, 0] = True
    for name in cameras:
        arrays["observation.images.%s" % name] = rng.integers(
            0, 255, (n, config.IMAGE_HEIGHT, config.IMAGE_WIDTH, 3), dtype=np.uint8
        )
    meta = {
        "robot": "neura", "in_simulation": True, "override": override,
        "servo_rate_hz": config.SERVO_RATE_HZ, "rate_hz": config.CONTROL_RATE_HZ,
        "planner": {"transit_speed_ms": config.TRANSIT_SPEED_MS},
        "sequence": "test_seq", "block": "B01",
    }
    return dataset.Episode(arrays=arrays, metadata=meta, success=success)


def _record(root, episodes):
    writer = dataset.DatasetWriter(root)
    for ep in episodes:
        writer.append(ep)
    return root


def _have(*modules):
    import importlib

    for name in modules:
        try:
            importlib.import_module(name)
        except Exception:
            print("  (uebersprungen: %s nicht installiert)" % name)
            return False
    return True


def test_scan_selects_episodes_and_detects_tempo_conflicts(tmp_path):
    a = _record(tmp_path / "a", [_episode(), _episode(success=False), _episode(success=None)])
    discarded = _episode()
    discarded.discarded, discarded.discard_reason = True, "schutzstopp"
    b = _record(tmp_path / "b", [discarded, _episode(override=0.5)])

    scan = lerobot_io.scan_sources([a])
    assert len(scan["episodes"]) == 2 and scan["unlabeled"] == 1
    assert scan["skipped"] == {"fehlgeschlagen": 1}
    assert not scan["conflicts"]
    assert len(lerobot_io.scan_sources([a], require_success=True)["episodes"]) == 1

    both = lerobot_io.scan_sources([a, b])
    assert "override" in both["conflicts"]
    assert both["skipped"]["verworfen (schutzstopp)"] == 1


def test_scan_refuses_other_schema(tmp_path):
    root = _record(tmp_path / "a", [_episode()])
    index = json.loads((root / "index.json").read_text(encoding="utf-8"))
    index["schema_version"] = 1
    (root / "index.json").write_text(json.dumps(index), encoding="utf-8")
    try:
        lerobot_io.scan_sources([root])
        assert False, "ExportError erwartet"
    except lerobot_io.ExportError:
        pass


def test_export_train_save_load_predict(tmp_path):
    if not _have("lerobot", "torch", "diffusers"):
        return
    import torch

    from bc import diffusion
    from lerobot.datasets import LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

    src = _record(tmp_path / "rec", [_episode(seed=1), _episode(seed=2)])
    out = tmp_path / "lr"
    info = lerobot_io.export([src], out, use_video=False, progress=lambda *_: None)
    assert info["lerobot_version"] == config.LEROBOT_VERSION
    assert info["codebase_version"].startswith("v3")
    assert len(info["episodes"]) == 2 and info["episodes"][0]["grasp_tcp"] is not None
    assert lerobot_io.read_export_info(out)["schema_version"] == config.SCHEMA_VERSION

    # Mini-Policy auf der CPU: Features, Zeitversaetze, ein Lernschritt
    device = torch.device("cpu")
    meta = LeRobotDatasetMetadata("local/bc_lara5", root=out)
    cfg = diffusion.build_config(meta, ["wrist", "scene"], device, horizon=8, n_action_steps=4,
                                 down_dims=(16, 32), inference_steps=2, pretrained_backbone=False)
    assert set(cfg.input_features) == {"observation.state", "observation.images.wrist",
                                       "observation.images.scene"}
    ds = LeRobotDataset("local/bc_lara5", root=out,
                        delta_timestamps=diffusion.delta_timestamps(cfg, ["wrist", "scene"]))
    item = ds[3]
    assert item["observation.state"].shape == (2, config.STATE_DIM)
    assert item["action"].shape == (8, config.ACTION_DIM)
    assert item["observation.images.wrist"].shape == (2, 3, config.IMAGE_HEIGHT, config.IMAGE_WIDTH)

    policy = DiffusionPolicy(cfg)
    pre, post = make_pre_post_processors(cfg, dataset_stats=meta.stats)
    batch = pre(next(iter(torch.utils.data.DataLoader(ds, batch_size=2))))
    loss, _ = policy.forward(batch)
    loss.backward()
    assert torch.isfinite(loss)

    ckpt = tmp_path / "policy"
    diffusion.save_policy(ckpt, policy, pre, post, {
        "schema_version": config.SCHEMA_VERSION, "rate_hz": config.CONTROL_RATE_HZ,
        "cameras": ["wrist", "scene"], "image_size_hw": [config.IMAGE_HEIGHT, config.IMAGE_WIDTH],
    })
    runner = diffusion.DiffusionPolicyRunner(ckpt, device="cpu")
    ep = dataset.load_episode(src, 0)
    obs = {k: ep.arrays[k][5] for k in ("observation.state", "observation.images.wrist",
                                        "observation.images.scene")}
    runner.reset()
    runner.observe(obs)
    chunk = runner.predict(obs)
    assert chunk.shape == (4, config.ACTION_DIM)
    assert np.all(np.isfinite(chunk))
    # unnormiert: Gelenkwerte im Bereich der Trainingsdaten (Min/Max-Norm, clip +-1)
    assert chunk[:, :6].min() >= -0.1 and chunk[:, :6].max() <= 0.7

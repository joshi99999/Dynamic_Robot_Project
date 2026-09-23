"""Training der Diffusion Policy (AP 3).

Eingabe ist ein mit apps/export.py erzeugtes LeRobotDataset (``--dataset``)
oder direkt Aufzeichnungsordner (``--data``, werden nach ``<out>/dataset``
exportiert). Modell: CNN-Diffusion-Policy aus lerobot 0.6.1, ResNet18 je
Kamera (ImageNet), 240x320, Horizont/Chunk aus config.POLICY_*.

HARDWAREUNABHAENGIG (AP 3.1): Das Training kann auf der RTX 5070 Ti, einer
5090 oder einem anderen Rechner laufen. Nichts ist fest verdrahtet:

* ``--device auto`` waehlt cuda/mps/cpu und prueft, ob der torch-Build die
  GPU-Architektur kennt (Blackwell: sm_120, cu128+).
* ``--amp auto`` nutzt bf16, wo die Karte es kann.
* ``--batch-size`` x ``--grad-accum`` = effektive Batchgroesse. Wer auf einer
  kleineren Karte dieselbe effektive Batch will, halbiert die eine und
  verdoppelt die andere -- Lernrate und Ergebnis bleiben vergleichbar.
* ``--num-workers auto`` richtet sich nach CPU-Kernen und Betriebssystem.
* Checkpoints sind geraeteneutral; ``--resume`` setzt auf jedem Rechner fort.
* Der Datensatzordner ist portabel: exportieren, kopieren, dort trainieren.

Ausgabe in ``--out``::

    train_config.json        Argumente, Hardware, Datensatz
    train_log.csv            Schritt, Loss, LR, Zeiten, Validierung
    checkpoints/step_XXXXXX  Policy + Prozessoren + bc_policy.json + Trainingsstand
    policy/                  finale Policy (EMA-Gewichte) fuer apps/infer.py

Beispiele (aus dem Ordner "Behavior Cloning"):
    python apps/train.py --dataset datasets/vm_run1 --out checkpoints/vm_run1
    python apps/train.py --data "data_sim/*" --out checkpoints/sim --steps 3000
    python apps/train.py --dataset D --out C --batch-size 16 --grad-accum 2   # kleinere GPU
    python apps/train.py --dataset D --out C --resume                        # fortsetzen
"""

import argparse
import csv
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np

from bc import config, lerobot_io


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    src = parser.add_argument_group("Daten")
    src.add_argument("--dataset", help="exportiertes LeRobotDataset (apps/export.py)")
    src.add_argument("--data", nargs="+", help="Aufzeichnungen, werden nach <out>/dataset exportiert")
    src.add_argument("--require-success", action="store_true",
                     help="beim Export nur Episoden mit Erfolgs-Label")
    src.add_argument("--val-fraction", type=float, default=0.1,
                     help="Anteil Episoden fuer die Validierung (0 = keine)")
    parser.add_argument("--out", required=True)

    hw = parser.add_argument_group("Hardware")
    hw.add_argument("--device", default="auto", help="auto | cuda | cuda:1 | mps | cpu")
    hw.add_argument("--amp", default="auto", choices=("auto", "bf16", "off"))
    # Gemessen 2026-09-17, RTX 5070 Ti, 2 Kameras 240x320, bf16: Batch 32 ->
    # 0.13 s/Schritt, 3.6 GB; Batch 64 -> 0.21 s/Schritt, 5.6 GB.
    hw.add_argument("--batch-size", type=int, default=64)
    hw.add_argument("--grad-accum", type=int, default=1,
                    help="Gradienten ueber N Batches sammeln (effektive Batch = N x batch-size)")
    hw.add_argument("--num-workers", default="auto")
    hw.add_argument("--compile", action="store_true", help="torch.compile fuer das U-Net (Linux)")

    tr = parser.add_argument_group("Training")
    tr.add_argument("--steps", type=int, default=60000, help="Optimierer-Schritte")
    tr.add_argument("--max-hours", type=float, default=None,
                    help="Zeitbudget; danach wird regulaer gespeichert und beendet")
    tr.add_argument("--lr", type=float, default=1e-4)
    tr.add_argument("--warmup", type=int, default=500)
    tr.add_argument("--weight-decay", type=float, default=1e-6)
    tr.add_argument("--grad-clip", type=float, default=10.0)
    tr.add_argument("--ema", type=float, default=0.999,
                    help="EMA-Abklingrate der Gewichte (0 = aus); die EMA-Gewichte werden gespeichert")
    tr.add_argument("--seed", type=int, default=0)
    tr.add_argument("--log-every", type=int, default=100)
    tr.add_argument("--eval-every", type=int, default=2000)
    tr.add_argument("--save-every", type=int, default=10000)
    tr.add_argument("--resume", action="store_true", help="am letzten Checkpoint in --out fortsetzen")

    pol = parser.add_argument_group("Policy (Defaults: config.POLICY_*)")
    pol.add_argument("--horizon", type=int, default=config.POLICY_HORIZON)
    pol.add_argument("--n-action-steps", type=int, default=config.POLICY_N_ACTION_STEPS)
    pol.add_argument("--n-obs-steps", type=int, default=config.POLICY_N_OBS_STEPS)
    pol.add_argument("--down-dims", default=",".join(map(str, config.POLICY_DOWN_DIMS)))
    pol.add_argument("--crop-ratio", type=float, default=config.POLICY_CROP_RATIO)
    pol.add_argument("--inference-steps", type=int, default=config.POLICY_INFERENCE_STEPS)
    pol.add_argument("--prediction-type", choices=("sample", "epsilon"),
                     default=config.POLICY_PREDICTION_TYPE)
    pol.add_argument("--no-pretrained", action="store_true",
                     help="ResNet18 ohne ImageNet-Gewichte (offline; dann GroupNorm)")
    pol.add_argument("--cameras", default=None,
                     help="Komma-Liste, z. B. 'wrist' fuer die Ablation Wrist-only (AP 5.1)")
    args = parser.parse_args(argv)
    if bool(args.dataset) == bool(args.data):
        parser.error("genau eines von --dataset oder --data angeben")
    return args


def auto_workers(requested):
    if requested != "auto":
        return int(requested)
    cpus = os.cpu_count() or 2
    # Windows startet Worker per spawn (langsamer Start, mehr RAM je Worker)
    return max(0, min(8 if os.name != "nt" else 4, cpus - 2))


def latest_checkpoint(out):
    ckpts = sorted((Path(out) / "checkpoints").glob("step_*"))
    ckpts = [c for c in ckpts if (c / "training_state.pt").is_file()]
    return ckpts[-1] if ckpts else None


def split_episodes(n_episodes, fraction, seed):
    """Episodenweise Aufteilung (nie Frames einer Episode auf beide Seiten)."""
    rng = np.random.default_rng(seed)
    order = rng.permutation(n_episodes).tolist()
    n_val = int(round(n_episodes * fraction)) if fraction > 0 else 0
    if fraction > 0 and n_episodes >= 5:
        n_val = max(1, n_val)
    n_val = min(n_val, n_episodes - 1)
    return sorted(order[n_val:]), sorted(order[:n_val])


def single_value(consistency, key):
    values = consistency.get(key) or {}
    if len(values) == 1:
        return json.loads(next(iter(values)))
    return None


def main(argv=None):
    # Zeilenweise ausgeben, auch wenn die Ausgabe in eine Datei umgeleitet ist
    # (lange Laeufe im Hintergrund / auf einem anderen Rechner).
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(line_buffering=True)
    args = parse_args(argv)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Export zuerst (ohne torch) -- scheitert er, ist nichts geladen.
    if args.data:
        from export import expand_sources

        dataset_root = out / "dataset"
        if not (dataset_root / "meta" / lerobot_io.EXPORT_INFO).is_file():
            lerobot_io.export(
                expand_sources(args.data), dataset_root,
                require_success=args.require_success, overwrite=True,
            )
    else:
        dataset_root = Path(args.dataset)
    export_info = lerobot_io.read_export_info(dataset_root)
    if export_info["schema_version"] != config.SCHEMA_VERSION:
        raise SystemExit(
            "Datensatz Schema v%s, Code v%s -- neu exportieren bzw. neu aufzeichnen"
            % (export_info["schema_version"], config.SCHEMA_VERSION)
        )

    import torch
    from torch.optim.swa_utils import AveragedModel, get_ema_multi_avg_fn

    import lerobot
    from lerobot.datasets import EpisodeAwareSampler, LeRobotDataset, LeRobotDatasetMetadata
    from lerobot.policies import make_pre_post_processors
    from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy

    from bc import diffusion

    if lerobot.__version__ != export_info["lerobot_version"]:
        print("WARNUNG: exportiert mit lerobot %s, installiert ist %s"
              % (export_info["lerobot_version"], lerobot.__version__))

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = diffusion.select_device(args.device)
    amp_dtype = diffusion.select_amp(device, args.amp)
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
    hw = diffusion.hardware_info(device)
    print("Hardware: %s" % json.dumps(hw, ensure_ascii=False))
    print("Mixed Precision: %s" % ("bf16" if amp_dtype else "aus"))

    cameras = export_info["cameras"]
    if args.cameras:
        chosen = [c.strip() for c in args.cameras.split(",") if c.strip()]
        unknown = [c for c in chosen if c not in cameras]
        if unknown:
            raise SystemExit("Kameras %s nicht im Datensatz (%s)" % (unknown, cameras))
        cameras = chosen

    repo_id = "local/bc_lara5"
    meta = LeRobotDatasetMetadata(repo_id, root=dataset_root)
    policy_cfg = diffusion.build_config(
        meta, cameras, device,
        n_obs_steps=args.n_obs_steps,
        horizon=args.horizon,
        n_action_steps=args.n_action_steps,
        down_dims=[int(v) for v in args.down_dims.split(",")],
        crop_ratio=args.crop_ratio,
        inference_steps=args.inference_steps,
        pretrained_backbone=not args.no_pretrained,
        use_amp=amp_dtype is not None,
        prediction_type=args.prediction_type,
    )
    train_eps, val_eps = split_episodes(meta.total_episodes, args.val_fraction, args.seed)
    deltas = diffusion.delta_timestamps(policy_cfg, cameras)
    train_ds = LeRobotDataset(repo_id, root=dataset_root, episodes=train_eps, delta_timestamps=deltas)
    val_ds = (
        LeRobotDataset(repo_id, root=dataset_root, episodes=val_eps, delta_timestamps=deltas)
        if val_eps else None
    )
    print("Datensatz %s: %d Episoden (%d Training, %d Validierung), %d Frames, Kameras %s"
          % (dataset_root, meta.total_episodes, len(train_eps), len(val_eps),
             train_ds.num_frames, cameras))

    resume_from = latest_checkpoint(out) if args.resume else None
    if args.resume and resume_from is None:
        raise SystemExit("--resume: kein Checkpoint unter %s/checkpoints" % out)

    policy = DiffusionPolicy(policy_cfg)
    preprocessor, postprocessor = make_pre_post_processors(policy_cfg, dataset_stats=meta.stats)
    policy.to(device)
    if args.compile:
        policy.diffusion.unet = torch.compile(policy.diffusion.unet)

    optimizer = torch.optim.AdamW(
        policy.get_optim_params(), lr=args.lr, betas=(0.95, 0.999), eps=1e-8,
        weight_decay=args.weight_decay,
    )

    def lr_lambda(step):
        if step < args.warmup:
            return (step + 1) / float(args.warmup)
        progress = (step - args.warmup) / float(max(1, args.steps - args.warmup))
        return 0.5 * (1.0 + math.cos(math.pi * min(1.0, progress)))

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)
    ema = (
        AveragedModel(policy, multi_avg_fn=get_ema_multi_avg_fn(args.ema), use_buffers=True)
        if args.ema > 0 else None
    )

    step = 0
    if resume_from is not None:
        state = torch.load(resume_from / "training_state.pt", map_location=device, weights_only=False)
        policy.load_state_dict(state["policy"])
        optimizer.load_state_dict(state["optimizer"])
        scheduler.load_state_dict(state["scheduler"])
        if ema is not None and state.get("ema") is not None:
            ema.load_state_dict(state["ema"])
        step = int(state["step"])
        print("Fortgesetzt bei Schritt %d aus %s" % (step, resume_from))

    workers = auto_workers(args.num_workers)
    sampler = EpisodeAwareSampler(
        train_ds.meta.episodes["dataset_from_index"],
        train_ds.meta.episodes["dataset_to_index"],
        episode_indices_to_use=train_eps,
        drop_n_last_frames=policy_cfg.drop_n_last_frames,
        shuffle=True,
        seed=args.seed + step,
        absolute_to_relative_idx=train_ds.absolute_to_relative_idx,
    )
    loader = torch.utils.data.DataLoader(
        train_ds, batch_size=args.batch_size, sampler=sampler, num_workers=workers,
        pin_memory=device.type == "cuda", drop_last=True,
        persistent_workers=workers > 0, prefetch_factor=2 if workers > 0 else None,
    )
    if len(loader) == 0:
        raise SystemExit("Zu wenige Frames fuer eine Batch (%d) -- --batch-size verkleinern"
                         % args.batch_size)
    val_batches = _fixed_val_batches(val_ds, args.batch_size, workers=0) if val_ds else []

    run_info = {
        "started": diffusion.timestamp(),
        "argv": sys.argv,
        "args": vars(args),
        "hardware": hw,
        "amp": "bf16" if amp_dtype else None,
        "num_workers": workers,
        "effective_batch_size": args.batch_size * args.grad_accum,
        "dataset": str(dataset_root),
        "train_episodes": train_eps,
        "val_episodes": val_eps,
        "cameras": cameras,
        "parameters": sum(p.numel() for p in policy.parameters()),
    }
    (out / "train_config.json").write_text(
        json.dumps(run_info, indent=2, ensure_ascii=False, default=str), encoding="utf-8"
    )
    print("Parameter: %.1f M, effektive Batch %d, Worker %d"
          % (run_info["parameters"] / 1e6, run_info["effective_batch_size"], workers))

    log_path = out / "train_log.csv"
    new_log = not log_path.exists() or resume_from is None
    log_file = open(log_path, "w" if new_log else "a", newline="", encoding="utf-8")
    log = csv.writer(log_file)
    if new_log:
        log.writerow(["step", "loss", "lr", "grad_norm", "data_s", "update_s", "gpu_mem_gb",
                      "val_loss", "val_joint_mae_rad", "val_joint_p95_rad", "val_gripper_acc",
                      "elapsed_min", "val_chunk_jump_p95_rad", "val_label_jump_p95_rad"])

    def make_info(final_step, last_metrics):
        return build_policy_info(export_info, dataset_root, meta, policy_cfg, cameras, args,
                                 hw, amp_dtype, final_step, last_metrics, run_info)

    def save(directory, final_step, last_metrics, with_state=True):
        model = ema.module if ema is not None else policy
        diffusion.save_policy(directory, model, preprocessor, postprocessor,
                              make_info(final_step, last_metrics))
        if with_state:
            torch.save(
                {
                    "step": final_step,
                    "policy": policy.state_dict(),
                    "optimizer": optimizer.state_dict(),
                    "scheduler": scheduler.state_dict(),
                    "ema": ema.state_dict() if ema is not None else None,
                    "args": vars(args),
                },
                directory / "training_state.pt",
            )

    t_start = time.perf_counter()
    deadline = t_start + args.max_hours * 3600 if args.max_hours else None
    data_iter = _cycle(loader, sampler)
    window = {"loss": [], "grad": [], "data": [], "update": []}
    last_metrics = {}
    policy.train()
    try:
        while step < args.steps:
            t0 = time.perf_counter()
            optimizer.zero_grad(set_to_none=True)
            loss_sum, data_s = 0.0, 0.0
            for _ in range(args.grad_accum):
                td = time.perf_counter()
                batch = next(data_iter)
                batch = preprocessor(batch)
                data_s += time.perf_counter() - td
                with torch.autocast(device.type, dtype=amp_dtype, enabled=amp_dtype is not None):
                    loss, _ = policy.forward(batch)
                (loss / args.grad_accum).backward()
                loss_sum += float(loss.detach())
            grad_norm = torch.nn.utils.clip_grad_norm_(policy.parameters(), args.grad_clip)
            optimizer.step()
            scheduler.step()
            if ema is not None:
                ema.update_parameters(policy)
            step += 1
            if not math.isfinite(loss_sum):
                raise SystemExit("Loss nicht endlich bei Schritt %d -- Abbruch" % step)

            window["loss"].append(loss_sum / args.grad_accum)
            window["grad"].append(float(grad_norm))
            window["data"].append(data_s)
            window["update"].append(time.perf_counter() - t0 - data_s)

            is_eval = val_batches and (step % args.eval_every == 0 or step == args.steps)
            if step % args.log_every == 0 or is_eval or step == args.steps:
                val = {}
                if is_eval:
                    model = ema.module if ema is not None else policy
                    val = evaluate(model, preprocessor, postprocessor, val_batches, policy_cfg,
                                   device, amp_dtype)
                    policy.train()
                mem = torch.cuda.max_memory_allocated(device) / 1024**3 if device.type == "cuda" else 0.0
                elapsed = (time.perf_counter() - t_start) / 60.0
                row = [step, np.mean(window["loss"]), scheduler.get_last_lr()[0],
                       np.mean(window["grad"]), np.mean(window["data"]), np.mean(window["update"]),
                       mem, val.get("loss"), val.get("joint_mae"), val.get("joint_p95"),
                       val.get("gripper_acc"), elapsed, val.get("chunk_jump_p95"),
                       val.get("label_jump_p95")]
                log.writerow(row)
                log_file.flush()
                eta = elapsed / step * (args.steps - step) if step else 0.0
                print("step %6d  loss %.4f  lr %.1e  grad %.2f  data %.3fs  upd %.3fs  mem %.1f GB  "
                      "%.1f min (Rest ~%.0f)%s"
                      % (step, row[1], row[2], row[3], row[4], row[5], mem, elapsed, eta,
                         "  | val loss %.4f  Gelenk-MAE %.4f rad (p95 %.4f)  Greifer %.1f %%  "
                         "Chunk-Sprung p95 %.4f rad (Label %.4f)"
                         % (val["loss"], val["joint_mae"], val["joint_p95"],
                            100 * val["gripper_acc"], val["chunk_jump_p95"],
                            val["label_jump_p95"]) if val else ""))
                last_metrics = {"step": step, "train_loss": float(row[1]), **val}
                for key in window:
                    window[key].clear()

            if step % args.save_every == 0:
                save(out / "checkpoints" / ("step_%06d" % step), step, last_metrics)
            if deadline and time.perf_counter() > deadline:
                print("Zeitbudget --max-hours erreicht bei Schritt %d" % step)
                break
    except KeyboardInterrupt:
        print("\nAbgebrochen bei Schritt %d -- speichere Checkpoint" % step)
    finally:
        log_file.close()

    final_dir = out / "checkpoints" / ("step_%06d" % step)
    if not (final_dir / "training_state.pt").is_file():
        save(final_dir, step, last_metrics)
    policy_dir = out / "policy"
    if policy_dir.exists():
        shutil.rmtree(policy_dir)
    save(policy_dir, step, last_metrics, with_state=False)
    print("Policy -> %s (Schritt %d)" % (policy_dir, step))


def _cycle(loader, sampler):
    # EpisodeAwareSampler schaltet die Epoche (und damit die Permutation)
    # bei jedem __iter__ selbst weiter -- hier nicht zusaetzlich set_epoch.
    while True:
        for batch in loader:
            yield batch


def _fixed_val_batches(val_ds, batch_size, workers, max_batches=4):
    """Feste Validierungs-Batches (gleichmaessig ueber die Episoden verteilt)."""
    import torch

    n = len(val_ds)
    count = min(n, batch_size * max_batches)
    indices = np.linspace(0, n - 1, count).round().astype(int).tolist()
    subset = torch.utils.data.Subset(val_ds, indices)
    loader = torch.utils.data.DataLoader(subset, batch_size=batch_size, shuffle=False,
                                         num_workers=workers)
    return list(loader)


def evaluate(model, preprocessor, postprocessor, batches, policy_cfg, device, amp_dtype):
    """Validierung: Diffusion-Loss UND Abweichung der gesampelten Chunks.

    Der Loss allein sagt wenig ueber die Qualitaet (Rauschvorhersage). Die
    gesampelte Aktionsfolge gegen das Label (ideale Bahn) in rad ist die
    anschaulichere Groesse: so weit liegt die Policy offline daneben.
    """
    import torch

    model.eval()
    losses, errors, grip_ok, grip_n = [], [], 0, 0
    jumps, label_jumps = [], []
    start = policy_cfg.n_obs_steps - 1
    end = start + policy_cfg.n_action_steps
    with torch.no_grad():
        for raw in batches:
            gt = raw["action"][:, start:end]
            pad = raw["action_is_pad"][:, start:end]
            batch = preprocessor(dict(raw))
            with torch.autocast(device.type, dtype=amp_dtype, enabled=amp_dtype is not None):
                loss, _ = model.forward(batch)
                model.reset()
                pred = model.predict_action_chunk(batch)
            losses.append(float(loss))
            pred = postprocessor(pred.float()).cpu()
            valid = ~pad
            err = (pred[..., :6] - gt[..., :6]).abs()[valid]
            errors.append(err.reshape(-1).numpy())
            # Glaette: groesster Gelenksprung zwischen Nachbarschritten im Chunk.
            # Befund 2026-09-17: der Mittelwertfehler war klein, einzelne
            # Spruenge aber 7x groesser als im Label -- das loest den ServoGuard aus.
            pair_valid = (valid[:, 1:] & valid[:, :-1]).all(dim=0)
            if pair_valid.any():
                d_pred = (pred[:, 1:, :6] - pred[:, :-1, :6]).abs().amax(dim=2)[:, pair_valid]
                d_gt = (gt[:, 1:, :6] - gt[:, :-1, :6]).abs().amax(dim=2)[:, pair_valid]
                jumps.append(d_pred.amax(dim=1).numpy())
                label_jumps.append(d_gt.amax(dim=1).numpy())
            g_pred = pred[..., 6] > config.GRIPPER_THRESHOLD
            g_true = gt[..., 6] > config.GRIPPER_THRESHOLD
            grip_ok += int((g_pred == g_true)[valid].sum())
            grip_n += int(valid.sum())
    model.reset()
    errors = np.concatenate(errors) if errors else np.zeros(1)
    jumps = np.concatenate(jumps) if jumps else np.zeros(1)
    label_jumps = np.concatenate(label_jumps) if label_jumps else np.zeros(1)
    return {
        "loss": float(np.mean(losses)),
        "joint_mae": float(np.mean(errors)),
        "joint_p95": float(np.percentile(errors, 95)),
        "gripper_acc": grip_ok / float(max(1, grip_n)),
        "chunk_jump_p95": float(np.percentile(jumps, 95)),
        "label_jump_p95": float(np.percentile(label_jumps, 95)),
    }


def build_policy_info(export_info, dataset_root, meta, policy_cfg, cameras, args, hw, amp_dtype,
                      step, metrics, run_info):
    """bc_policy.json: alles, was die Inferenz gegen die Aufzeichnung prueft."""
    import lerobot

    from bc import diffusion

    consistency = export_info.get("consistency", {})
    lengths = [ep["length"] for ep in export_info["episodes"]]
    return {
        "created": diffusion.timestamp(),
        "schema_version": export_info["schema_version"],
        "rate_hz": export_info["rate_hz"],
        "image_size_hw": export_info["image_size_hw"],
        "cameras": cameras,
        "lerobot_version": lerobot.__version__,
        "dataset": {
            "path": str(dataset_root),
            "codebase_version": export_info["codebase_version"],
            "episodes": meta.total_episodes,
            "frames": meta.total_frames,
            "train_episodes": run_info["train_episodes"],
            "val_episodes": run_info["val_episodes"],
            "sources": export_info["sources"],
            "unlabeled_episodes": export_info.get("unlabeled_episodes"),
        },
        # Muessen bei der Inferenz gleich sein (AP 2.6 / 1.3): die Policy hat
        # genau dieses Tempo und dieses Folgeverhalten des Arms gelernt.
        "recording": {
            "robot": single_value(consistency, "robot"),
            "camera_backends": single_value(consistency, "cameras"),
            "in_simulation": single_value(consistency, "in_simulation"),
            "override": single_value(consistency, "override"),
            "servo_rate_hz": single_value(consistency, "servo_rate_hz"),
            "planner": {
                key.split("/", 1)[1]: single_value(consistency, key)
                for key in consistency if key.startswith("planner/")
            },
        },
        "start_joints": export_info["start_joints"],
        "end_joints": export_info["end_joints"],
        "start_gripper": export_info.get("start_gripper"),
        "end_gripper": export_info.get("end_gripper"),
        "ideal_z_min": export_info.get("ideal_z_min"),
        # Referenz fuer die Bewertung einer Fahrt ohne Datensatz (bc.metrics)
        "reference": {
            "path": export_info.get("reference_path"),
            "grasp_tcp": [ep.get("grasp_tcp") for ep in export_info["episodes"]],
            "end_tcp": [ep.get("end_tcp") for ep in export_info["episodes"]],
        },
        "episode_length_max": max(lengths),
        "episode_length_median": float(np.median(lengths)),
        "policy": {
            "type": "diffusion",
            "n_obs_steps": policy_cfg.n_obs_steps,
            "horizon": policy_cfg.horizon,
            "n_action_steps": policy_cfg.n_action_steps,
            "down_dims": list(policy_cfg.down_dims),
            "crop_ratio": policy_cfg.crop_ratio,
            "inference_steps": policy_cfg.num_inference_steps,
            "prediction_type": policy_cfg.prediction_type,
            "pretrained_backbone": not args.no_pretrained,
        },
        "inference_defaults": {
            "replan_steps": config.POLICY_REPLAN_STEPS,
            "ensemble_decay": config.POLICY_ENSEMBLE_DECAY,
        },
        "training": {
            "step": step,
            "steps_planned": args.steps,
            "ema": args.ema,
            "effective_batch_size": args.batch_size * args.grad_accum,
            "lr": args.lr,
            "amp": "bf16" if amp_dtype else None,
            "hardware": hw,
            "metrics": metrics,
        },
    }


if __name__ == "__main__":
    main()

"""Stellvertreter-Studie: welche Rauschstaerke hilft einer gelernten Policy? (AP 2.4/5)

NUR LESEND -- bewegt keinen Roboter. Laeuft komplett gegen den SimRobot und
trainiert kleine MLP-Policies auf der GPU (torch). Ergebnis der Laeufe vom
2026-09-16: Berichte/2026-09-16_Rauschstudie.pdf.

Aufbau:
* Aufgabe wie sequences/pick_to_station.json, Posen (kartesisch) aus einer
  VM-Aufnahme, Objekt je Episode um +-3 cm verschoben (PRE_GRASP/PICK wandern mit).
* Echte Pipeline: build_ideal_trajectory (Rampen, Ueberschleifen), Rauschen per
  SmoothedOUProcess + Trichter + IK + Kollisionspruefung.
* Arm-Modell: 60-Hz-Interpolation + Tiefpass tau=30 ms (VM, Override 1.0: 20-35 ms).
* Policy: MLP, Beobachtung = Gelenke t und t-1, TCP, Greifer, Greifer-Timer,
  Objekt-xy (Ersatz fuer die Kamera). Label = Chunk der idealen Gelenke t+1..t+8,
  ausgefuehrt werden 4 Schritte (wie policy.ChunkExecutor).
* Bewertung im geschlossenen Regelkreis: Greif- und Uebergabefehler,
  Bahnabweichung, Befehlsbeschleunigung -- ungestoert und mit Stoessen.

GRENZEN: kein Bild, keine Diffusion Policy, URDF-Kinematik statt LARA 5. Die
Studie zeigt Tendenzen (braucht es Rauschen, wie unruhig wird die Policy), keine
auf die Anlage uebertragbaren Erfolgsquoten. Erfolgsquoten zwischen
moderaten Einstellungen streuen staerker mit dem Trainings-Seed als mit dem
Rauschen -- verbindlich ist erst der Vergleich mit der echten Policy (AP 5).

Ausfuehren (aus dem Ordner "Behavior Cloning"; Umgebungsvariablen optional):
    python tools/study_noise_proxy.py                 # alle Einstellungen
    N_SEEDS=5 N_TEST=24 PUSH_MM=30 python tools/study_noise_proxy.py F_ M_
"""
import json, sys, time
from pathlib import Path

import _bootstrap  # noqa: F401

import numpy as np
import torch
from bc import config, dataset, geometry
from bc.adapters.sim_robot import SimRobot
from bc.clock import SimClock
from bc.collision import default_workspace
from bc.kinematics import Kinematics, IKFailure
from bc.noise import SmoothedOUProcess, funnel_scale, NoiseLimits
from bc.trajectory import Waypoint, build_ideal_trajectory

_BC = Path(__file__).resolve().parent.parent
OUT = str(_BC / "Berichte" / "rauschstudie.json")
DT = 1 / 15.0
SUB = 4
TAU_LAG = 0.03
HORIZON, EXECUTE = 8, 4  # wie policy.ChunkExecutor / HoldPolicy
LIM = np.array(config.JOINT_LIMITS_RAD)
MAX_STEP = 1.0 * DT  # 1 rad/s je Gelenk als Sicherungsgrenze
PUSH = float(__import__("os").environ.get("PUSH_MM", "15")) / 1000.0
ALPHA = 1 - np.exp(-(DT / SUB) / TAU_LAG)
DWELL = config.GRIPPER_DWELL_STEPS
DEV = torch.device("cuda")

vm = dataset.load_episode(str(_BC / "data_vm" / "2026-09-16" / "2_60Hz_Rauschen_Ueberschleifen"), 0).metadata["points"]
robot = SimRobot(clock=SimClock()).connect()
kin = Kinematics(robot)
home = robot.read_state().joints


def find_offset():
    # VM-Posen in den Arbeitsraum des URDF-Modells schieben (nur Lage, Form bleibt)
    for dz in (0.0, 0.1, -0.1, 0.2, -0.2):
        for dx in (0.0, 0.1, -0.1):
            try:
                chain(np.array([dx, 0, dz]), np.zeros(2))
                chain(np.array([dx, 0, dz]), np.array([0.03, 0.03]))
                chain(np.array([dx, 0, dz]), np.array([-0.03, -0.03]))
                return np.array([dx, 0, dz])
            except IKFailure:
                continue
    raise SystemExit("keine erreichbare Verschiebung")


def chain(offset, obj):
    poses = {}
    for name in ("CLEAR_FOV", "APPROACH_01", "PRE_GRASP", "PICK", "PRE_PLACE"):
        p = np.array(vm[name]["pose_quat"], dtype=float)
        p[:3] += offset
        if name in ("PRE_GRASP", "PICK"):
            p[:2] += obj
        poses[name] = p
    joints, seed = {}, home
    for name in ("CLEAR_FOV", "APPROACH_01", "PRE_GRASP", "PICK", "PRE_PLACE"):
        seed = kin.ik(poses[name], seed)
        joints[name] = seed
    return poses, joints


def task(offset, obj):
    poses, joints = chain(offset, obj)
    W = lambda n, **kw: Waypoint(robot.fk(joints[n]), name=n, joints=joints[n], **kw)  # noqa: E731
    wps = [W("CLEAR_FOV"), W("APPROACH_01", motion="ptp", blend_m=0.05), W("PRE_GRASP", motion="ptp"),
           W("PICK", motion="lin", approach=True, gripper_closed=True),
           W("PRE_GRASP", motion="lin", gripper_closed=True, blend_m=0.04),
           W("PRE_PLACE", motion="ptp", gripper_closed=True)]
    return wps, build_ideal_trajectory(wps, fk=robot.fk)


def noisy_plan(ideal, rng, s_pos, s_rot, tau_smooth=config.NOISE_SMOOTH_TAU_S, workspace=None, tries=20):
    q_ideal = kin.solve_path(ideal.poses_quat, ideal.joints[0], fixed_joints=ideal.joints)
    if s_pos == 0 and s_rot == 0:
        return q_ideal, q_ideal
    limits = NoiseLimits()
    scale = np.where(ideal.dwell_mask, 0.0, funnel_scale(ideal.dist_to_anchor))
    for _ in range(tries):
        ou_p = SmoothedOUProcess(s_pos, config.NOISE_OU_TAU_S, tau_smooth, DT, 3, rng,
                                 lo=limits.lo * s_pos / config.NOISE_TRANS_AMPLITUDE_M,
                                 hi=limits.hi * s_pos / config.NOISE_TRANS_AMPLITUDE_M)
        ou_r = SmoothedOUProcess(max(s_rot, 1e-9), config.NOISE_OU_TAU_S, tau_smooth, DT, 3, rng)
        noisy = ideal.poses_quat.copy()
        for i in range(len(noisy)):
            noisy[i, :3] += ou_p.step() * scale[i]
            r = ou_r.step() * scale[i]
            ang = float(np.linalg.norm(r))
            if ang > 1e-12:
                noisy[i, 3:7] = geometry.quat_normalize(geometry.quat_multiply(geometry.quat_from_axis_angle(r / ang, ang), noisy[i, 3:7]))
        try:
            q_noisy = kin.solve_path(noisy, ideal.joints[0])
        except IKFailure:
            continue
        if workspace.check_path(robot, q_noisy, stride=3):
            continue
        return q_ideal, q_noisy
    return None


def execute(q_cmd_seq, q0):
    """Arm-Modell: 60-Hz-Zwischenschritte + Tiefpass. Liefert Zustand je Takt."""
    q = q0.copy(); prev = q0.copy(); out = [q.copy()]
    for target in q_cmd_seq[1:]:
        for k in range(1, SUB + 1):
            sp = prev + k / SUB * (target - prev)
            q = q + ALPHA * (sp - q)
        prev = target
        out.append(q.copy())
    return np.array(out)


def features(q, q_prev, grip, timer, obj):
    tcp = robot.fk(q)[:3]
    return np.concatenate([q, q_prev, tcp, [grip, timer], obj * 10.0])


def build_dataset(episodes):
    X, Y = [], []
    for ep in episodes:
        q_state, q_ideal, grip, obj = ep
        n = len(q_state)
        change = 0
        for i in range(n):
            if i > 0 and grip[i] != grip[i - 1]:
                change = i
            timer = min(i - change, DWELL) / DWELL if change else 1.0
            X.append(features(q_state[i], q_state[max(i - 1, 0)], grip[i], timer, obj))
            chunk = []
            for h in range(1, HORIZON + 1):
                j = min(i + h, n - 1)
                chunk.append(np.r_[q_ideal[j], grip[j]])
            Y.append(np.concatenate(chunk))
    return np.array(X, np.float32), np.array(Y, np.float32)


class MLP(torch.nn.Module):
    def __init__(self, d_in, d_out):
        super().__init__()
        self.net = torch.nn.Sequential(torch.nn.Linear(d_in, 512), torch.nn.GELU(), torch.nn.Linear(512, 512), torch.nn.GELU(),
                                       torch.nn.Linear(512, 512), torch.nn.GELU(), torch.nn.Linear(512, d_out))

    def forward(self, x):
        return self.net(x)


def train(X, Y, seed, steps=6000):
    torch.manual_seed(seed)
    xm, xs = X.mean(0), X.std(0) + 1e-6
    ym, ys = Y.mean(0), Y.std(0) + 1e-6
    Xt = torch.tensor((X - xm) / xs, device=DEV); Yt = torch.tensor((Y - ym) / ys, device=DEV)
    model = MLP(X.shape[1], Y.shape[1]).to(DEV)
    opt = torch.optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, steps)
    g = torch.Generator(device=DEV).manual_seed(seed)
    for _ in range(steps):
        idx = torch.randint(0, len(Xt), (1024,), device=DEV, generator=g)
        loss = torch.nn.functional.mse_loss(model(Xt[idx]), Yt[idx])
        opt.zero_grad(); loss.backward(); opt.step(); sched.step()
    model.eval()

    def policy(x):
        with torch.no_grad():
            y = model(torch.tensor(((x - xm) / xs)[None], dtype=torch.float32, device=DEV)).cpu().numpy()[0]
        return y * ys + ym
    return policy


def rollout(policy, wps, ideal, q_ideal, obj, pushes=(), rng=None):
    n_max = len(ideal) + 30
    q = wps[0].joints.copy(); q_prev = q.copy(); cmd_prev = q.copy()
    grip, change, grasp_err = 0.0, 0, None
    pick = wps[3].pose_quat[:3]; place = wps[-1].pose_quat[:3]
    tcp_log, cmds = [robot.fk(q)[:3]], [q.copy()]
    chunk, cursor = None, EXECUTE
    for t in range(n_max):
        timer = min(t - change, DWELL) / DWELL if change else 1.0
        if cursor >= EXECUTE:
            chunk = policy(features(q + rng.normal(0, 3e-4, 6), q_prev, grip, timer, obj)).reshape(HORIZON, 7)
            cursor = 0
        a = chunk[cursor]; cursor += 1
        # Sicherung wie am echten System: Achsgrenzen, Sprungbegrenzung je Takt
        target = np.clip(a[:6], LIM[:, 0], LIM[:, 1])
        target = cmd_prev + np.clip(target - cmd_prev, -MAX_STEP, MAX_STEP)
        g_new = float(a[6] > 0.5)
        if g_new != grip:
            if g_new == 1.0 and grasp_err is None:
                grasp_err = float(np.linalg.norm(robot.fk(q)[:3] - pick))
            grip, change = g_new, t + 1
        q_prev = q.copy()
        for k in range(1, SUB + 1):
            sp = cmd_prev + k / SUB * (target - cmd_prev)
            q = q + ALPHA * (sp - q)
        cmd_prev = target
        for (tp, off) in pushes:
            if t == tp:
                try:
                    p = robot.fk(q); p[:3] += off
                    q = kin.ik(p, q)
                except IKFailure:
                    pass
        cmds.append(target.copy()); tcp_log.append(robot.fk(q)[:3])
    tcp_log = np.array(tcp_log); cmds = np.array(cmds)
    ideal_tcp = ideal.poses_quat[:, :3]
    d = np.array([np.min(np.linalg.norm(ideal_tcp - p, axis=1)) for p in tcp_log])
    acc = np.abs(np.diff(cmds, 2, axis=0)).max(axis=1) / DT**2
    return dict(grasp_err=grasp_err, end_err=float(np.linalg.norm(tcp_log[-1] - place)), closed_end=grip == 1.0,
                dev_p95=float(np.percentile(d, 95)), dev_max=float(d.max()), acc_p95=float(np.percentile(acc, 95)),
                acc_max=float(acc.max()))


CONFIGS = {
    "A_ohne":            dict(mode="fix", s_pos=0.0, s_rot=0.0),
    "B_aktuell_15mm_3deg": dict(mode="fix", s_pos=0.015, s_rot=0.05),
    "C_7.5mm_1.4deg":    dict(mode="fix", s_pos=0.0075, s_rot=0.025),
    "D_3.75mm_0.7deg":   dict(mode="fix", s_pos=0.00375, s_rot=0.0125),
    "E_7.5mm_0.6deg":    dict(mode="fix", s_pos=0.0075, s_rot=0.01),
    "F_mix_0-15mm":      dict(mode="mix", s_pos=0.015, s_rot=0.05),
    "G_mix_0-7.5mm_0.6deg": dict(mode="mix", s_pos=0.0075, s_rot=0.01),
    "H_stark_22mm_4deg": dict(mode="fix", s_pos=0.0225, s_rot=0.075),
    "I_15mm_0.6deg":     dict(mode="fix", s_pos=0.015, s_rot=0.01),
    "J_5mm_0.6deg":      dict(mode="fix", s_pos=0.005, s_rot=0.01),
    "K_mix_0-15mm_0.6deg": dict(mode="mix", s_pos=0.015, s_rot=0.01),
    "L_mix_0-10mm_1.4deg": dict(mode="mix", s_pos=0.010, s_rot=0.025),
    "M_mix_0-15mm_1.4deg": dict(mode="mix", s_pos=0.015, s_rot=0.025),
}
N_TRAIN = 60
N_TEST = int(__import__("os").environ.get("N_TEST", "16"))

if __name__ == "__main__":
    t0 = time.time()
    offset = find_offset()
    ws_z = None
    rng_obj = np.random.default_rng(123)
    train_objs = rng_obj.uniform(-0.03, 0.03, (N_TRAIN, 2))
    test_objs = rng_obj.uniform(-0.025, 0.025, (N_TEST, 2))
    tasks_train = [task(offset, o) for o in train_objs]
    tasks_test = [task(offset, o) for o in test_objs]
    z = min(wp.pose_quat[2] for wp in tasks_train[0][0]) - 0.15
    workspace = default_workspace(table_height_m=z)
    print("Verschiebung", offset, "Schritte", len(tasks_train[0][1]), "Aufbau %.0f s" % (time.time() - t0))
    ideal_test = [(wps, ideal, kin.solve_path(ideal.poses_quat, ideal.joints[0], fixed_joints=ideal.joints)) for wps, ideal in tasks_test]
    ref_acc = [np.percentile(np.abs(np.diff(q, 2, axis=0)).max(axis=1) / DT**2, 95) for _, _, q in ideal_test]
    print("Referenz: ideale Bahn selbst, Befehl-Beschl. p95 median %.2f rad/s^2" % np.median(ref_acc))

    results = {}
    only = sys.argv[1:]
    for name, cfg in CONFIGS.items():
        if only and not any(name.startswith(o) for o in only):
            continue
        rng = np.random.default_rng(7)
        eps, noise_mm = [], []
        for (wps, ideal), obj in zip(tasks_train, train_objs):
            s = rng.uniform(0, 1) if cfg["mode"] == "mix" else 1.0
            plan = noisy_plan(ideal, rng, cfg["s_pos"] * s, cfg["s_rot"] * s, workspace=workspace)
            if plan is None:
                continue
            q_ideal, q_noisy = plan
            q_state = execute(q_noisy, q_noisy[0])
            tcp_state = np.array([robot.fk(q)[:3] for q in q_state])
            noise_mm.append(np.linalg.norm(tcp_state - ideal.poses_quat[:, :3], axis=1) * 1000)
            eps.append((q_state, q_ideal, ideal.gripper, obj))
        X, Y = build_dataset(eps)
        nm = np.concatenate(noise_mm)
        res = dict(episodes=len(eps), dev_train_median=float(np.median(nm)), dev_train_p95=float(np.percentile(nm, 95)), runs=[])
        for seed in range(int(__import__("os").environ.get("N_SEEDS", "3"))):
            pol = train(X, Y, seed)
            for kind, pushes_fn in (("ungestoert", lambda L: ()),
                                    ("stoesse", lambda L: ((int(0.2 * L), np.array([0.0, PUSH, 0.0])),
                                                           (int(0.8 * L), np.array([PUSH, 0.0, 0.0]))))):
                ev_rng = np.random.default_rng(99)
                for (wps, ideal, q_id), obj in zip(ideal_test, test_objs):
                    r = rollout(pol, wps, ideal, q_id, obj, pushes=pushes_fn(len(ideal)), rng=ev_rng)
                    r.update(seed=seed, kind=kind)
                    res["runs"].append(r)
        results[name] = res
        def summ(kind):
            rr = [r for r in res["runs"] if r["kind"] == kind]
            ge = np.array([r["grasp_err"] if r["grasp_err"] is not None else 0.999 for r in rr]) * 1000
            ee = np.array([r["end_err"] for r in rr]) * 1000
            ok = np.mean((ge < 5) & (ee < 5) & np.array([r["closed_end"] for r in rr]))
            return "%s: Erfolg %3.0f%% | Greiffehler median %5.1f p90 %5.1f mm | Uebergabe median %5.1f mm | Bahn p95 %5.1f max %5.1f mm | Befehl-Beschl. p95 %5.2f max %6.2f" % (
                kind, 100 * ok, np.median(ge), np.percentile(ge, 90), np.median(ee),
                1000 * np.median([r["dev_p95"] for r in rr]), 1000 * np.median([r["dev_max"] for r in rr]),
                np.median([r["acc_p95"] for r in rr]), np.median([r["acc_max"] for r in rr]))
        print("\n%s  (%d Ep., Trainingsabweichung median %.1f p95 %.1f mm)  [%.0f s]" % (name, len(eps), res["dev_train_median"], res["dev_train_p95"], time.time() - t0))
        print("   " + summ("ungestoert"))
        print("   " + summ("stoesse"))
        json.dump(results, open(OUT.replace(".json", "_%s_push%s.json" % ("_".join(only or ["alle"]), __import__("os").environ.get("PUSH_MM", "15"))), "w"))

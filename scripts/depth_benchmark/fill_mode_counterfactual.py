#!/usr/bin/env python3
"""Counterfactual: how much does ``depth_fill_mode`` change the policy input/output?

Rebuilds the 64x40 policy input from the logger's 16-bit raw-depth PNGs two ways:

* ``min``     -- legacy behaviour: invalid pixels clamped to ``min_depth``
* ``nearest`` -- deployed default: invalid pixels take the nearest valid depth

then runs the deployed VAE encoder and policy (TorchScript exports; same weights
as the deployed ONNX) on both, holding the 16-dim state identical and keeping a
separate LSTM state per branch.  The observed ``cmd_vel`` (low-pass filtered by
the controller) is reconstructed by applying the controller's filter to each
branch, so per-branch replay-vs-log agreement is meaningful.

Usage::

    python3 scripts/depth_benchmark/fill_mode_counterfactual.py \
        --session door_diagnostics/session_20261110_145214_realsense \
        --session door_diagnostics/session_20261110_145750_realsense \
        --vae /tmp/nav_models/vae.pt --policy /tmp/nav_models/policy.pt \
        --out /tmp/fill_cf
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import sys

import numpy as np
import cv2

POLICY_SCALE = np.array([1.5, 1.0, 1.0], dtype=np.float64)
LATERAL_SCALE = 0.6
LOW_PASS = np.array([0.9, 0.5, 0.5], dtype=np.float64)  # vx, vy, wz
MIN_DEPTH = 0.15
MAX_DEPTH = 10.0
ROWS, COLS = 40, 64


def load_trajectory(session):
    with open(os.path.join(session, "trajectory.csv"), newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        cols = list(reader.fieldnames or [])
        rows = list(reader)

    def num(v):
        s = str(v).strip()
        if s == "":
            return np.nan
        low = s.lower()
        if low == "true":
            return 1.0
        if low == "false":
            return 0.0
        try:
            return float(s)
        except ValueError:
            return np.nan

    return cols, rows, {c: np.array([num(r.get(c)) for r in rows]) for c in cols}


def masks(depth, min_depth=MIN_DEPTH, max_depth=MAX_DEPTH):
    nan = np.isnan(depth)
    far = np.isposinf(depth) | (depth > max_depth)
    close = np.isneginf(depth) | ((depth < min_depth) & ~nan & ~far)
    fill = close | nan
    valid = ~(fill | far)
    return fill, far, valid


def fill_depth(depth, mode, min_depth=MIN_DEPTH):
    d = depth.copy()
    fillm, farm, valid = masks(d, min_depth)
    d[farm] = 0.0
    if mode == "min":
        d[fillm] = min_depth
        return d
    if fillm.any() and not valid.any():
        d[fillm] = min_depth
        return d
    if fillm.any():
        invalid = (~valid).astype(np.uint8)
        _, labels = cv2.distanceTransformWithLabels(
            invalid, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
        lut = np.zeros(int(labels.max()) + 1, dtype=np.float32)
        lut[labels[valid]] = d[valid]
        nearest = lut[labels]
        d[fillm] = nearest[fillm]
    return d


def grid(depth, mode):
    return cv2.resize(fill_depth(depth, mode), (COLS, ROWS), interpolation=cv2.INTER_LINEAR)


def rot_inv(yaw):
    c, s = np.cos(-yaw), np.sin(-yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def projected_gravity(roll, pitch):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    rxt = np.array([[1, 0, 0], [0, cr, sr], [0, -sr, cr]])
    ryt = np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
    return ryt @ (rxt @ np.array([0.0, 0.0, -1.0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", action="append", required=True)
    ap.add_argument("--vae", required=True)
    ap.add_argument("--policy", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--stateless", action="store_true",
                    help="reset the LSTM every frame: gives the exact instantaneous "
                         "action f(state, depth) of each fill mode (no memory drift)")
    args = ap.parse_args()

    import torch

    vae = torch.jit.load(args.vae, map_location="cpu").eval()
    net = {}
    for mode in ("min", "nearest"):
        m = torch.jit.load(args.policy, map_location="cpu").eval()
        m.hidden_state.zero_()
        m.cell_state.zero_()
        net[mode] = m

    os.makedirs(args.out, exist_ok=True)
    summary = {}

    for sess in args.session:
        sess = os.path.expanduser(sess.rstrip("/"))
        name = os.path.basename(sess)
        cols, rows, d = load_trajectory(sess)
        cmd = np.stack([d["cmd_vx"], d["cmd_vy"], d["cmd_wz"]], axis=1)
        nz = np.where(np.linalg.norm(cmd, axis=1) > 1e-6)[0]
        lo, hi = int(nz.min()), int(nz.max())

        raw_files = sorted(glob.glob(os.path.join(sess, "depth_raw", "*_raw_depth.png")))
        n = min(len(raw_files), len(rows))

        last_action = {"min": np.zeros(3), "nearest": np.zeros(3)}
        prev_filt = {"min": np.zeros(3), "nearest": np.zeros(3)}
        out = {k: [] for k in ("idx", "a_min", "a_near", "cmd_min", "cmd_near",
                               "log", "e_min", "e_near", "g_min", "g_near")}

        first = True
        for i in range(lo, min(hi + 1, n), args.stride):
            if not (np.isfinite(d["robot_x"][i]) and np.isfinite(d["robot_y"][i])
                    and np.isfinite(d["carrot_x"][i])):
                continue
            img = cv2.imread(raw_files[i], cv2.IMREAD_UNCHANGED)
            if img is None:
                continue
            depth = img.astype(np.float32) / 1000.0
            g = {"min": grid(depth, "min"), "nearest": grid(depth, "nearest")}
            emb = {}
            with torch.no_grad():
                for mode in ("min", "nearest"):
                    emb[mode] = vae(torch.from_numpy(g[mode])[None, None]).reshape(-1).numpy()

            yaw = np.deg2rad(d["robot_yaw_deg"][i])
            v_b = rot_inv(yaw) @ np.array([d["odom_vx"][i], d["odom_vy"][i], d["odom_vz"][i]])
            w_b = rot_inv(yaw) @ np.array([0.0, 0.0, d["odom_wz"][i]])
            grav = projected_gravity(np.deg2rad(d["robot_roll_deg"][i]),
                                     np.deg2rad(d["robot_pitch_deg"][i]))
            tgt = np.array([d["carrot_x"][i], d["carrot_y"][i], d["carrot_z"][i]])
            rob = np.array([d["robot_x"][i], d["robot_y"][i], d["robot_z"][i]])
            tv = rot_inv(yaw) @ (tgt - rob)
            dist = np.linalg.norm(tv) + 1e-6
            tgt_log = np.concatenate([tv / dist, [np.log(dist + 1.0)]])

            acts = {}
            cmds = {}
            with torch.no_grad():
                for mode in ("min", "nearest"):
                    state = np.concatenate([v_b, w_b, grav, last_action[mode], tgt_log]).astype(np.float32)
                    obs = torch.from_numpy(np.concatenate([state, emb[mode]])[None])
                    a = net[mode](obs, bool(args.stateless or first)).reshape(-1).numpy()
                    acts[mode] = a
                    last_action[mode] = a
                    c = np.tanh(a) * POLICY_SCALE
                    c[1] *= LATERAL_SCALE
                    filt = LOW_PASS * c + (1.0 - LOW_PASS) * prev_filt[mode]
                    prev_filt[mode] = filt
                    cmds[mode] = filt
            first = False

            out["idx"].append(i)
            out["a_min"].append(acts["min"]);  out["a_near"].append(acts["nearest"])
            out["cmd_min"].append(cmds["min"]); out["cmd_near"].append(cmds["nearest"])
            out["log"].append([d["cmd_vx"][i], d["cmd_vy"][i], d["cmd_wz"][i]])
            out["e_min"].append(emb["min"]);   out["e_near"].append(emb["nearest"])
            out["g_min"].append(g["min"]);     out["g_near"].append(g["nearest"])

        A0 = np.array(out["a_min"]); A2 = np.array(out["a_near"])
        C0 = np.array(out["cmd_min"]); C2 = np.array(out["cmd_near"])
        L = np.array(out["log"])
        E0 = np.array(out["e_min"]); E2 = np.array(out["e_near"])
        G0 = np.array(out["g_min"]); G2 = np.array(out["g_near"])

        def corr(a, b):
            if np.std(a) < 1e-9 or np.std(b) < 1e-9:
                return float("nan")
            return float(np.corrcoef(a, b)[0, 1])

        el2 = np.linalg.norm(E0 - E2, axis=1)
        ecos = np.sum(E0 * E2, 1) / (np.linalg.norm(E0, axis=1) * np.linalg.norm(E2, axis=1) + 1e-9)
        res = {
            "session": name,
            "frames": int(len(L)),
            "grid_diff": {
                "mean_abs_m": float(np.mean(np.abs(G0 - G2))),
                "p95_abs_m": float(np.percentile(np.abs(G0 - G2), 95)),
                "near03_v0": float(np.mean((G0 > 0) & (G0 < 0.3))),
                "near03_v2": float(np.mean((G2 > 0) & (G2 < 0.3))),
            },
            "embedding": {
                "l2_mean": float(el2.mean()), "l2_p95": float(np.percentile(el2, 95)),
                "cos_mean": float(ecos.mean()), "cos_min": float(ecos.min()),
                "rel_l2_mean": float(np.mean(el2 / (np.linalg.norm(E0, axis=1) + 1e-9))),
            },
            "action_v0_vs_v2": {
                "d_vx_mean": float(np.mean(np.abs(A0[:, 0] - A2[:, 0]))),
                "d_vx_p95": float(np.percentile(np.abs(A0[:, 0] - A2[:, 0]), 95)),
                "d_wz_mean": float(np.mean(np.abs(A0[:, 2] - A2[:, 2]))),
                "d_wz_p95": float(np.percentile(np.abs(A0[:, 2] - A2[:, 2]), 95)),
                "d_cmd_vx_mean": float(np.mean(np.abs(C0[:, 0] - C2[:, 0]))),
                "sign_agree_vx": float(np.mean(np.sign(A0[:, 0]) == np.sign(A2[:, 0]))),
                "sign_agree_wz": float(np.mean(np.sign(A0[:, 2]) == np.sign(A2[:, 2]))),
            },
            "temporal_jitter": {
                "cmd_vx_step_v0": float(np.mean(np.abs(np.diff(C0[:, 0])))),
                "cmd_vx_step_v2": float(np.mean(np.abs(np.diff(C2[:, 0])))),
                "cmd_wz_step_v0": float(np.mean(np.abs(np.diff(C0[:, 2])))),
                "cmd_wz_step_v2": float(np.mean(np.abs(np.diff(C2[:, 2])))),
            },
            "replay_vs_log": {
                "corr_vx_v0": corr(C0[:, 0], L[:, 0]), "corr_vx_v2": corr(C2[:, 0], L[:, 0]),
                "corr_wz_v0": corr(C0[:, 2], L[:, 2]), "corr_wz_v2": corr(C2[:, 2], L[:, 2]),
                "mae_vx_v0": float(np.mean(np.abs(C0[:, 0] - L[:, 0]))),
                "mae_vx_v2": float(np.mean(np.abs(C2[:, 0] - L[:, 0]))),
                "mae_wz_v0": float(np.mean(np.abs(C0[:, 2] - L[:, 2]))),
                "mae_wz_v2": float(np.mean(np.abs(C2[:, 2] - L[:, 2]))),
                "log_vx_std": float(np.std(L[:, 0])), "log_wz_std": float(np.std(L[:, 2])),
            },
        }
        summary[name] = res
        np.savez(os.path.join(args.out, f"{name}_cf.npz"), **{k: np.array(v) for k, v in out.items()})

    with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()

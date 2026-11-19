#!/usr/bin/env python3
"""Motion-smoothness comparison for door-diagnostic sessions (A/B tool).

Reads one or more ``door_diagnostic_logger`` sessions, infers the autonomous
mission window from ``/cmd_vel`` (non-zero => policy in control; the operator's
teleop tail publishes on a different topic and leaves ``cmd_*`` at zero), and
reports the metrics that separate "smooth" from "stuttering" runs:

* mission / teleop-tail timing and distance
* stop->start cycles, share of time below 0.10 / 0.20 m/s, hard dips
* acceleration magnitude, command step size
* flat-ground (no stair flag) speed ripple after detrending, yaw-command
  percentiles and saturation share, heading error to the carrot
* depth-input health (near / hole / validity, center min) for context

Usage::

    python3 scripts/analyze_motion_stutter.py \
        --session door_diagnostics/session_..._realsense \
        --session door_diagnostics/session_..._realsense \
        --out door_diagnostics/analysis_motion_20261110
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from datetime import datetime

import numpy as np


def _num(v):
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


def load_session(path):
    path = os.path.expanduser(path.rstrip("/"))
    with open(os.path.join(path, "trajectory.csv"), newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        cols = list(reader.fieldnames or [])
        rows = list(reader)
    data = {c: np.array([_num(r.get(c)) for r in rows], dtype=float) for c in cols}
    return os.path.basename(path), cols, rows, data


def iso(epoch):
    return datetime.fromtimestamp(epoch).strftime("%H:%M:%S.%f")[:-3]


def detrend(sig, win=12):
    if len(sig) < win + 4:
        return sig - sig.mean()
    trend = np.convolve(sig, np.ones(win) / win, mode="same")
    edge = win // 2
    res = sig - trend
    return res[edge:-edge]


def analyze(path):
    name, cols, rows, d = load_session(path)
    cmd = np.stack([d["cmd_vx"], d["cmd_vy"], d["cmd_wz"]], axis=1)
    mag = np.linalg.norm(cmd, axis=1)
    nz = np.where(mag > 1e-6)[0]
    if not len(nz):
        raise SystemExit(f"{name}: no non-zero cmd_vel, cannot infer mission window")
    lo, hi = int(nz.min()), int(nz.max())

    t = d["epoch_sec"]
    sp = d["speed_mps"]
    dt = np.gradient(t)
    dist = np.cumsum(np.nan_to_num(sp) * dt)
    dist -= dist[lo]

    mission = slice(lo, hi + 1)
    tail = slice(hi + 1, len(rows))
    dur = t[hi] - t[lo]
    nframes = hi - lo + 1

    flat = np.array([i for i in range(lo, hi + 1)
                     if d["is_descending_stairs"][i] < 0.5 and d["is_ascending_stairs"][i] < 0.5])
    stair = np.array([i for i in range(lo, hi + 1)
                      if d["is_descending_stairs"][i] > 0.5 or d["is_ascending_stairs"][i] > 0.5])

    s = sp[mission]
    below10 = s < 0.10
    cycles = int(np.sum((~below10[1:]) & below10[:-1]))
    hard = int(sum(1 for i in range(lo + 2, hi - 1) if sp[i] < 0.12 and sp[i - 2] > 0.30))
    acc = np.gradient(np.nan_to_num(sp)) / np.maximum(dt, 1e-6)
    acc_m = acc[mission]

    flat_res = detrend(np.nan_to_num(sp[flat]))
    wz_flat = np.abs(d["cmd_wz"][flat])
    ta_flat = np.abs(d["target_angle_deg"][flat])

    out = {
        "session": name,
        "note": (json.load(open(os.path.join(os.path.expanduser(path), "session_args.json")))
                 .get("note") if os.path.exists(os.path.join(os.path.expanduser(path), "session_args.json")) else None),
        "depth_fill_mode": (json.load(open(os.path.join(os.path.expanduser(path), "session_args.json")))
                            .get("depth_fill_mode")
                            if os.path.exists(os.path.join(os.path.expanduser(path), "session_args.json")) else None),
        "mission": {
            "start": iso(t[lo]), "end": iso(t[hi]), "duration_s": round(float(dur), 1),
            "distance_m": round(float(dist[hi]), 1),
            "mean_speed": round(float(np.nanmean(s)), 3),
            "mean_cmd_vx": round(float(np.nanmean(d["cmd_vx"][mission])), 3),
        },
        "teleop_tail": {
            "start": iso(t[hi + 1]) if hi + 1 < len(rows) else None,
            "end": iso(t[-1]), "duration_s": round(float(t[-1] - t[hi + 1]), 1),
            "distance_m": round(float(dist[-1] - dist[hi + 1]), 1),
            "cmd_all_zero": bool(np.all(mag[tail] < 1e-6)),
        },
        "smoothness": {
            "stop_start_cycles": cycles,
            "stop_start_per_min": round(cycles / (dur / 60.0), 2),
            "speed_below_010_share": round(float(np.mean(s < 0.10)), 3),
            "speed_below_020_share": round(float(np.mean(s < 0.20)), 3),
            "hard_dips": hard,
            "accel_abs_mean": round(float(np.nanmean(np.abs(acc_m))), 3),
            "accel_abs_p95": round(float(np.nanpercentile(np.abs(acc_m), 95)), 3),
            "cmd_vx_abs_step_mean": round(float(np.mean(np.abs(np.diff(d["cmd_vx"][mission])))), 3),
            "cmd_vx_range": [round(float(np.nanmin(d["cmd_vx"][mission])), 2),
                             round(float(np.nanmax(d["cmd_vx"][mission])), 2)],
        },
        "flat_ground": {
            "frames": int(len(flat)),
            "share_of_mission": round(float(len(flat) / nframes), 2),
            "mean_speed": round(float(np.nanmean(sp[flat])), 3),
            "speed_ripple_std": round(float(np.nanstd(flat_res)), 3),
            "speed_ripple_p95": round(float(np.nanpercentile(np.abs(flat_res), 95)), 3),
            "mean_abs_cmd_wz": round(float(np.nanmean(wz_flat)), 3),
            "cmd_wz_p50_p90_p99": [round(float(np.nanpercentile(wz_flat, q)), 2) for q in (50, 90, 99)],
            "cmd_wz_sat_share": round(float(np.mean(wz_flat > 0.7)), 3),
            "heading_err_p50_deg": round(float(np.nanpercentile(ta_flat, 50)), 1),
            "heading_err_p90_deg": round(float(np.nanpercentile(ta_flat, 90)), 1),
        },
        "stairs": {
            "frames": int(len(stair)),
            "window_s": ([round(float(t[stair[0]] - t[lo]), 1), round(float(t[stair[-1]] - t[lo]), 1)]
                         if len(stair) else None),
            "mean_speed": round(float(np.nanmean(sp[stair])), 3) if len(stair) else None,
            "mean_abs_cmd_wz": round(float(np.nanmean(np.abs(d["cmd_wz"][stair]))), 3) if len(stair) else None,
        },
        "depth_input": {
            "near_ratio_mean": round(float(np.nanmean(d["depth_near_ratio"][mission])), 4),
            "hole_ratio_mean": round(float(np.nanmean(d["depth_hole_ratio"][mission])), 4),
            "hole_ratio_max": round(float(np.nanmax(d["depth_hole_ratio"][mission])), 3),
            "valid_ratio_mean": round(float(np.nanmean(d["depth_valid_ratio_raw"][mission])), 3),
            "center_min_mean": round(float(np.nanmean(d["depth_center_min"][mission])), 3),
            "depth_age_ms_mean": round(float(np.nanmean(d["depth_age_ms"][mission])), 1),
        },
    }
    return out


def to_markdown(results):
    keys = [
        ("mission.duration_s", "任务时长 (s)"),
        ("mission.distance_m", "任务里程 (m)"),
        ("mission.mean_speed", "任务平均速度 (m/s)"),
        ("smoothness.stop_start_per_min", "停→走 次/分钟"),
        ("smoothness.speed_below_020_share", "速度<0.20 占比"),
        ("smoothness.hard_dips", "硬减速次数"),
        ("smoothness.accel_abs_p95", "加速度 abs p95"),
        ("smoothness.cmd_vx_abs_step_mean", "Δcmd_vx abs 均值"),
        ("flat_ground.speed_ripple_std", "平地速度纹波 std"),
        ("flat_ground.speed_ripple_p95", "平地速度纹波 p95"),
        ("flat_ground.mean_abs_cmd_wz", "平地 mean abs cmd_wz"),
        ("flat_ground.cmd_wz_sat_share", "平地 |cmd_wz|>0.7 占比".replace("|","｜")),
        ("flat_ground.heading_err_p50_deg", "平地航向误差 p50 (°)"),
        ("stairs.mean_speed", "楼梯平均速度"),
        ("depth_input.hole_ratio_mean", "原始深度空洞比例"),
        ("depth_input.hole_ratio_max", "空洞比例峰值"),
        ("depth_input.center_min_mean", "中心最小距离均值 (m)"),
    ]

    def get(res, path):
        cur = res
        for part in path.split("."):
            if not isinstance(cur, dict):
                return "—"
            cur = cur.get(part)
        return "—" if cur is None else cur

    lines = ["| 指标 | " + " | ".join(r["session"] for r in results) + " |",
             "|---|" + "---|" * len(results)]
    for path, label in keys:
        lines.append(f"| {label} | " + " | ".join(str(get(r, path)) for r in results) + " |")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", action="append", required=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    results = [analyze(p) for p in args.session]
    print(json.dumps(results, indent=2, ensure_ascii=False))
    print()
    print(to_markdown(results))

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        with open(os.path.join(args.out, "motion_metrics.json"), "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        with open(os.path.join(args.out, "motion_metrics.md"), "w", encoding="utf-8") as f:
            f.write(to_markdown(results) + "\n")
        print(f"\nwritten: {args.out}/motion_metrics.{{json,md}}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Stage 0: offline lookahead replay on recorded door/stairs sessions.

Replays the deployment's real `AdaptivePathSegmentTracker` against the poses,
stair-gate flags and 40x64 depth frames recorded by `door_diagnostic_logger`,
recomputing for every logged frame:

    path progress s(t) -> lookahead L(t) -> carrot point -> body-frame target -> d_theta

so that lookahead parameters can be screened **without driving the robot**.

What this is and is not
-----------------------
* It is a faithful replay of the *tracker*: the tracker is imported from the
  deployment repo (same source the robot runs), fed the recorded pose history
  and the recorded stair-gate booleans, so `L(t)` and the carrot for a given
  parameter set are exactly what the controller would have produced on the
  same trajectory.
* It is **not** a closed-loop simulation: the recorded trajectory is an input,
  not a consequence. So it cannot predict contact, only what the policy was
  asked to do (target demand) under each parameter set.

Pose latency
------------
The recorded `/vis/lookahead_goal` carrot is consistently ~0.3-0.5 m *behind*
what this replay produces from the pose logged in the same frame. Fitting a
pure pose delay gives tau ~= 1.0 s in both sessions (0.7 s at 0.8 m/s,
1.2 s at 0.35 m/s), and on the stairs it fully cancels the intended 0.4 m
lookahead (observed arc lookahead 0.04-0.06 m). Use `--pose-lag-sec` to model
it; `auto` fits it against the recorded carrot; `--pose-lag-sec 0` shows the
same trajectory without the deficit.

Usage
-----
    python3 scripts/stage0_lookahead_replay.py \
        --session door_diagnostics/session_20261110_144216_realsense \
        --session door_diagnostics/session_20261110_145109_realsense \
        --pose-lag-sec auto --out door_diagnostics/analysis_stage0

    # parameter sweep (1-D, one parameter at a time, everything else baseline)
    python3 scripts/stage0_lookahead_replay.py ... --sweep all

Outputs (in --out): stage0_metrics.json, stage0_report.md,
figures/l_vs_s.svg, figures/dtheta_vs_s.svg, figures/sweep_*.svg
"""

import argparse
import csv
import glob
import importlib.util
import json
import math
import os
import sys

import numpy as np

# --------------------------------------------------------------------------
# Deployment tracker import
# --------------------------------------------------------------------------

DEFAULT_TRACKER = (
    "/home/yichangfeng/sru-robot-deployment/go2w_nav_ros2_ws/src/"
    "rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py"
)


def load_tracker_class(path):
    spec = importlib.util.spec_from_file_location("deployed_adaptive_path_tracker", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import tracker from {path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.AdaptivePathSegmentTracker


# Baseline = what start_nav.sh launches today (mode curvature_only, stair L 0.40)
BASELINE = dict(
    adaptive_mode="curvature_only",
    lookahead_min=0.50,
    lookahead_max=2.00,
    window_dist=1.60,
    turn_angle_low_deg=20.0,
    turn_angle_high_deg=50.0,
    slew_rate_decrease=2.5,
    slew_rate_increase=0.8,
    stair_lookahead=0.40,
    stair_up_lookahead=0.45,
)

# 1-D sweep grid (Stage 0 feasibility screening)
SWEEP = {
    "lookahead_max": [1.5, 1.75, 2.0, 2.25, 2.5],
    "lookahead_min": [0.35, 0.5, 0.6, 0.7, 0.8, 0.9],
    "window_dist": [1.0, 1.3, 1.6, 2.0, 2.4],
    "turn_angle_high_deg": [40.0, 50.0, 60.0, 70.0],
    "turn_angle_low_deg": [10.0, 20.0, 30.0],
    "slew_rate_increase": [0.4, 0.8, 1.2, 2.0],
    "slew_rate_decrease": [1.5, 2.5, 4.0],
    "stair_lookahead": [0.25, 0.40, 0.55, 0.70],
}

# Landmarks used only to make reports readable (map frame, from the recordings)
LANDMARKS = [
    ("实验室出门", -3.2, 2.6),
    ("门外走廊", -0.5, 6.3),
    ("门上沿转弯", -0.1, 9.3),
    ("楼梯前窄门", 1.5, 9.5),
    ("楼梯入口", 2.5, 6.7),
    ("第一段楼梯", 5.5, 6.15),
    ("平台掉头", 8.05, 7.10),
    ("第二段楼梯", 5.0, 7.65),
    ("楼梯底部窄门", 2.5, 8.10),
    ("底部走廊", 1.0, 9.5),
]


# --------------------------------------------------------------------------
# Session loading
# --------------------------------------------------------------------------


def _f(row, key, default=float("nan")):
    v = row.get(key, "")
    if v in ("", None):
        return default
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


class Session:
    def __init__(self, path):
        self.dir = os.path.abspath(path)
        self.name = os.path.basename(self.dir.rstrip("/"))
        rows = list(csv.DictReader(open(os.path.join(self.dir, "trajectory.csv"))))
        if not rows:
            raise RuntimeError(f"empty trajectory in {self.dir}")
        self.rows = rows

        self.t = np.array([_f(r, "epoch_sec") for r in rows])
        self.t = self.t - self.t[0]
        self.x = np.array([_f(r, "robot_x") for r in rows])
        self.y = np.array([_f(r, "robot_y") for r in rows])
        self.z = np.array([_f(r, "robot_z") for r in rows])
        self.yaw = np.radians(np.array([_f(r, "robot_yaw_deg", 0.0) for r in rows]))
        self.cmd_vx = np.nan_to_num(np.array([_f(r, "cmd_vx", 0.0) for r in rows]))
        self.cmd_wz = np.nan_to_num(np.array([_f(r, "cmd_wz", 0.0) for r in rows]))
        self.speed = np.nan_to_num(np.array([_f(r, "speed_mps", 0.0) for r in rows]))
        self.stair_down = np.nan_to_num(np.array([_f(r, "is_descending_stairs", 0.0) for r in rows])) > 0.5
        self.stair_up = np.nan_to_num(np.array([_f(r, "is_ascending_stairs", 0.0) for r in rows])) > 0.5
        self.carrot = np.array([[_f(r, "carrot_x"), _f(r, "carrot_y")] for r in rows])
        self.carrot_src = [r.get("carrot_source", "") for r in rows]
        self.frame_idx = np.array([int(float(r["frame_idx"])) for r in rows])

        pj = glob.glob(os.path.join(self.dir, "paths", "*.json"))
        if not pj:
            raise RuntimeError(f"no path snapshot in {self.dir}")
        self.path_json = json.load(open(sorted(pj)[0]))
        self.path_pts = self.path_json["points"]
        self.P = np.array([[p[0], p[1]] for p in self.path_pts])
        self.cum = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(self.P, axis=0), axis=1))]
        )
        self.path_len = float(self.cum[-1])

        # Task window: first .. last non-zero command (afterwards = manual teleop return)
        nz = np.where(np.abs(self.cmd_vx) > 1e-6)[0]
        if len(nz) == 0:
            raise RuntimeError(f"no non-zero cmd in {self.dir}")
        self.i0, self.i1 = int(nz[0]), int(nz[-1])

        self.depth_cache = {}

    def depth(self, i):
        fi = int(self.frame_idx[i])
        if fi not in self.depth_cache:
            p = os.path.join(self.dir, "depth", f"frame_{fi:06d}_depth_40x64.npy")
            self.depth_cache[fi] = np.load(p) if os.path.exists(p) else None
        return self.depth_cache[fi]

    def pose_at(self, tq):
        """Logged pose linearly interpolated at absolute session time `tq`."""
        return np.interp(tq, self.t, self.x), np.interp(tq, self.t, self.y)

    def project(self, xy):
        """Nearest projection onto the recorded path: (dist_m, arc_s)."""
        best_d, best_s = float("inf"), 0.0
        for i in range(len(self.P) - 1):
            v = self.P[i + 1] - self.P[i]
            l2 = float(v @ v)
            tt = 0.0 if l2 < 1e-12 else float(np.clip((xy - self.P[i]) @ v / l2, 0.0, 1.0))
            proj = self.P[i] + tt * v
            d = float(np.sum((xy - proj) ** 2))
            if d < best_d:
                best_d = d
                best_s = self.cum[i] + tt * (self.cum[i + 1] - self.cum[i])
        return math.sqrt(best_d), best_s

    def point_at(self, s):
        s = float(np.clip(s, 0.0, self.path_len))
        i = int(np.clip(np.searchsorted(self.cum, s) - 1, 0, len(self.P) - 2))
        seg = self.cum[i + 1] - self.cum[i]
        r = 0.0 if seg < 1e-9 else (s - self.cum[i]) / seg
        return self.P[i] + r * (self.P[i + 1] - self.P[i])


# --------------------------------------------------------------------------
# Path geometry: corners
# --------------------------------------------------------------------------


def resample_path(P, step=0.05):
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    s = np.arange(0.0, cum[-1] + 1e-9, step)
    out = np.stack([np.interp(s, cum, P[:, 0]), np.interp(s, cum, P[:, 1])], axis=1)
    return s, out


def heading_profile(pts, half_window_m, step):
    n = int(round(half_window_m / step))
    n = max(1, n)
    lo = np.clip(np.arange(len(pts)) - n, 0, len(pts) - 1)
    hi = np.clip(np.arange(len(pts)) + n, 0, len(pts) - 1)
    v = pts[hi] - pts[lo]
    return np.arctan2(v[:, 1], v[:, 0])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def detect_corners(session, half_window_m=0.6, step=0.05, thresh_deg=45.0, merge_m=2.0):
    """Corners = where the path heading changes >= thresh within +/- half_window."""
    s, pts = resample_path(session.P, step)
    h = heading_profile(pts, half_window_m, step)
    # signed turn over the 1.2 m window ahead: heading(s+1.2) - heading(s)
    n = int(round(2.0 * half_window_m / step))
    fwd = np.clip(np.arange(len(pts)) + n, 0, len(pts) - 1)
    turn = np.degrees(wrap(h[fwd] - h))
    mag = np.abs(turn)
    cand = np.where(mag >= thresh_deg)[0]
    corners = []
    if len(cand) == 0:
        return corners
    groups = [[cand[0]]]
    for i in cand[1:]:
        if s[i] - s[groups[-1][-1]] <= merge_m:
            groups[-1].append(i)
        else:
            groups.append([i])
    for g in groups:
        apex = g[int(np.argmax(mag[g]))]
        corners.append(
            dict(
                s=float(s[apex]),
                x=float(pts[apex, 0]),
                y=float(pts[apex, 1]),
                turn_deg=float(turn[apex]),
                s_start=float(s[g[0]]),
                s_end=float(s[g[-1]]),
            )
        )
    for c in corners:
        best = min(LANDMARKS, key=lambda L: math.hypot(L[1] - c["x"], L[2] - c["y"]))
        c["label"] = best[0] if math.hypot(best[1] - c["x"], best[2] - c["y"]) < 1.6 else f"({c['x']:.1f},{c['y']:.1f})"
    return corners


# --------------------------------------------------------------------------
# Replay
# --------------------------------------------------------------------------


def fit_pose_lag(session, Tracker, params, taus=np.arange(0.0, 2.01, 0.1)):
    """Pick the pose delay that best reproduces the recorded carrot (task window)."""
    best = (0.0, float("inf"), 0)
    for tau in taus:
        res = replay(session, Tracker, params, pose_lag=float(tau), collect=False)
        errs = []
        for k, i in enumerate(res["idx"]):
            cp = res["carrot"][k]
            if cp is None:
                continue
            c = session.carrot[i]
            if not np.isfinite(c).all() or session.carrot_src[i] != "/vis/lookahead_goal":
                continue
            errs.append(float(np.linalg.norm(c - cp)))
        if errs and np.median(errs) < best[1]:
            best = (float(tau), float(np.median(errs)), len(errs))
    return best


def replay(session, Tracker, params, pose_lag=0.0, collect=True):
    """Run the deployment tracker over one session's recorded trajectory."""
    tr = Tracker(
        lookahead_min=params["lookahead_min"],
        lookahead_max=params["lookahead_max"],
        window_dist=params["window_dist"],
        turn_angle_low_deg=params["turn_angle_low_deg"],
        turn_angle_high_deg=params["turn_angle_high_deg"],
        slew_rate_decrease=params["slew_rate_decrease"],
        slew_rate_increase=params["slew_rate_increase"],
        adaptive_mode=params["adaptive_mode"],
    )
    tr.set_path(session.path_pts)

    n = len(session.rows)
    out = dict(
        idx=[],
        t=[], s=[], x=[], y=[], yaw=[], branch=[], L=[], l_curv=[], turn=[], carrot=[],
        dtheta=[], carrot_step=[], cmd_wz=[], cmd_vx=[], speed=[], depth_age=None,
    )
    prev_carrot = None
    for i in range(session.i0, session.i1 + 1):
        tq = session.t[i] - float(pose_lag)
        x, y = session.pose_at(tq)
        yaw = session.yaw[i]
        tr.update_progress([x, y])

        if session.stair_down[i]:
            cp, _cy = tr.get_lookahead_point(params["stair_lookahead"])
            branch, L, l_curv, turn = "stair_down", params["stair_lookahead"], np.nan, np.nan
        elif session.stair_up[i]:
            cp, _cy = tr.get_lookahead_point(params["stair_up_lookahead"])
            branch, L, l_curv, turn = "stair_up", params["stair_up_lookahead"], np.nan, np.nan
        else:
            depth = session.depth(i) if params["adaptive_mode"] in ("both", "clearance_only") else None
            cp, _cy, dbg = tr.get_adaptive_lookahead_point(current_time=session.t[i], depth_image=depth)
            branch = "adaptive"
            L = float(dbg.get("lookahead", np.nan))
            l_curv = float(dbg.get("l_curv", np.nan))
            turn = float(dbg.get("turn_deg", np.nan))

        if cp is None:
            cp2 = None
            dtheta = np.nan
        else:
            cp2 = np.array([cp[0], cp[1]], dtype=float)
            v = cp2 - np.array([x, y])
            ang = math.atan2(v[1], v[0]) - yaw
            dtheta = math.degrees(wrap(ang))

        step = np.nan
        if cp2 is not None and prev_carrot is not None:
            step = float(np.linalg.norm(cp2 - prev_carrot))
        prev_carrot = cp2

        out["idx"].append(i)
        out["t"].append(float(session.t[i]))
        out["s"].append(float(tr.current_progress))
        out["x"].append(float(x))
        out["y"].append(float(y))
        out["yaw"].append(float(yaw))
        out["branch"].append(branch)
        out["L"].append(float(L))
        out["l_curv"].append(l_curv)
        out["turn"].append(turn)
        out["carrot"].append(cp2)
        out["dtheta"].append(dtheta)
        out["carrot_step"].append(step)
        out["cmd_vx"].append(float(session.cmd_vx[i]))
        out["cmd_wz"].append(float(session.cmd_wz[i]))
        out["speed"].append(float(session.speed[i]))

    if not collect:
        return out

    for k in ("t", "s", "x", "y", "yaw", "L", "l_curv", "turn", "dtheta", "carrot_step", "cmd_vx", "cmd_wz", "speed"):
        out[k] = np.array(out[k], dtype=float)
    out["adtheta"] = np.abs(out["dtheta"])
    # demand rate: per-frame change of the body-frame bearing, scaled to per-second
    dt = np.gradient(out["t"]) if len(out["t"]) > 2 else np.ones(len(out["t"]))
    out["dtheta_rate"] = np.abs(np.gradient(np.nan_to_num(out["dtheta"]))) / np.maximum(dt, 1e-3)
    return out


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------


def metrics(session, res, params, corners):
    idx = res["idx"]
    moving = (np.abs(res["cmd_vx"]) > 0.02) | (np.abs(res["cmd_wz"]) > 0.02)
    # ignore the paused stretch: it is not a tracking decision
    m = moving & np.isfinite(res["L"])
    L = res["L"][m]
    ad = res["adtheta"][m]
    dr = res["dtheta_rate"][m]

    def pct(a, q):
        return float(np.nanpercentile(a, q)) if len(a) else float("nan")

    out = {
        "n_frames": int(m.sum()),
        "task_start_s": float(res["t"][0]),
        "task_end_s": float(res["t"][-1]),
        "dist_traveled_m": float(np.sum(np.abs(np.diff(res["s"]))) if len(res["s"]) else 0.0),
        "L_p5": pct(L, 5),
        "L_p50": pct(L, 50),
        "L_p95": pct(L, 95),
        "L_at_min_frac": float(np.mean(L <= params["lookahead_min"] * 1.02)) if len(L) else float("nan"),
        "L_at_max_frac": float(np.mean(L >= params["lookahead_max"] * 0.98)) if len(L) else float("nan"),
        "L_short1m_frac": float(np.mean(L < 1.0)) if len(L) else float("nan"),
        "dtheta_p50": pct(ad, 50),
        "dtheta_p95": pct(ad, 95),
        "dtheta_gt60_frac": float(np.mean(ad > 60.0)) if len(ad) else float("nan"),
        "dtheta_gt90_frac": float(np.mean(ad > 90.0)) if len(ad) else float("nan"),
        "dtheta_rate_p95": pct(dr, 95),
        "carrot_step_p95": pct(res["carrot_step"][m], 95),
        "carrot_step_max": pct(res["carrot_step"][m], 100),
    }

    # per-corner demand, sampled on the approach
    per_corner = []
    for c in corners:
        win = (res["s"] >= c["s"] - 1.2) & (res["s"] <= c["s"] + 0.4) & np.isfinite(res["L"])
        if not win.any():
            continue
        Lc = res["L"][win]
        adc = res["adtheta"][win]
        # lookahead just before the apex (0.6-0.2 m before)
        pre = (res["s"] >= c["s"] - 0.8) & (res["s"] <= c["s"] - 0.2) & np.isfinite(res["L"])
        per_corner.append(
            dict(
                label=c["label"],
                s=c["s"],
                turn_deg=c["turn_deg"],
                L_pre=float(np.median(res["L"][pre])) if pre.any() else float("nan"),
                L_min=float(np.min(Lc)),
                dtheta_max=float(np.max(adc)),
                dtheta_p50=float(np.median(adc)),
            )
        )
    out["corners"] = per_corner
    out["dtheta_max_corner"] = float(np.max([c["dtheta_max"] for c in per_corner])) if per_corner else float("nan")

    # stair-landing window: recorded stair-gate dropouts on the platform
    sd = session.stair_down
    drops = []
    inside = False
    for k in range(session.i0, session.i1):
        if sd[k] and not sd[k + 1]:
            start = k + 1
            inside = True
        elif inside and (sd[k + 1] or k + 1 == session.i1):
            drops.append((start, k))
            inside = False
    land = []
    for a, b in drops:
        pos = np.array([(a + b) // 2])
        i_mid = int(pos[0])
        z = session.z[i_mid]
        # a landing dropout = flat segment between two descending flights
        if b - a < 3 or not (sd[max(0, a - 3):a].any() and sd[b:b + 4].any()):
            continue
        win = np.isin(res["idx"], list(range(a, b + 1)))
        if not win.any():
            continue
        land.append(
            dict(
                frames=[int(a), int(b)],
                t=[float(session.t[a]), float(session.t[b])],
                z=float(z),
                x=float(session.x[i_mid]),
                y=float(session.y[i_mid]),
                L_min=float(np.nanmin(res["L"][win])),
                L_max=float(np.nanmax(res["L"][win])),
                dtheta_max=float(np.nanmax(res["adtheta"][win])),
                yaw_span_deg=float(np.degrees(abs(wrap(session.yaw[b] - session.yaw[a])))),
            )
        )
    out["stair_landings"] = land
    out["landing_dtheta_max"] = float(np.max([q["dtheta_max"] for q in land])) if land else float("nan")
    return out


def score(m):
    """Single ranking number: how hard the tracker asks the policy to turn.

    Weights: corner demand (worst corner), general demand, demand jerk, and the
    share of frames where the lookahead is so short the policy is effectively
    driving blind (L < 1 m).
    """
    parts = [
        (m.get("dtheta_max_corner", np.nan), 0.35),
        (m.get("dtheta_p95", np.nan), 0.25),
        (m.get("dtheta_rate_p95", np.nan) * 0.5, 0.20),  # deg/s -> deg-ish
        (m.get("L_short1m_frac", np.nan) * 90.0, 0.20),  # 0..1 -> 0..90
    ]
    vals = [(v, w) for v, w in parts if np.isfinite(v)]
    if not vals:
        return float("nan")
    tw = sum(w for _, w in vals)
    return float(sum(v * w for v, w in vals) / tw)


# --------------------------------------------------------------------------
# SVG figures (no third-party deps)
# --------------------------------------------------------------------------


def svg_line_plot(path, series, xlabel, ylabel, title, width=980, height=420,
                  y_range=None, hlines=()):
    pad_l, pad_r, pad_t, pad_b = 70, 150, 40, 52
    pw, ph = width - pad_l - pad_r, height - pad_t - pad_b
    allx = np.concatenate([s["x"] for s in series if len(s["x"])])
    ally = np.concatenate([s["y"] for s in series if len(s["y"])])
    ally = ally[np.isfinite(ally)]
    if y_range:
        y0, y1 = y_range
    else:
        y0, y1 = float(np.min(ally)), float(np.max(ally))
        if y1 - y0 < 1e-6:
            y1 = y0 + 1.0
    x0, x1 = float(np.min(allx)), float(np.max(allx))
    if x1 - x0 < 1e-6:
        x1 = x0 + 1.0

    def X(v):
        return pad_l + (v - x0) / (x1 - x0) * pw

    def Y(v):
        return pad_t + ph - (v - y0) / (y1 - y0) * ph

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
           f'style="background:#fff;font-family:-apple-system,Segoe UI,Helvetica,Arial">']
    out.append(f'<text x="{pad_l}" y="24" font-size="15" fill="#111">{title}</text>')
    # grid
    for k in range(6):
        yv = y0 + (y1 - y0) * k / 5
        yy = Y(yv)
        out.append(f'<line x1="{pad_l}" y1="{yy:.1f}" x2="{pad_l+pw}" y2="{yy:.1f}" stroke="#eee"/>')
        out.append(f'<text x="{pad_l-8}" y="{yy+4:.1f}" font-size="11" fill="#666" text-anchor="end">{yv:.2f}</text>')
    for k in range(6):
        xv = x0 + (x1 - x0) * k / 5
        xx = X(xv)
        out.append(f'<text x="{xx:.1f}" y="{pad_t+ph+18}" font-size="11" fill="#666" text-anchor="middle">{xv:.1f}</text>')
    for hv, txt in hlines:
        yy = Y(hv)
        out.append(f'<line x1="{pad_l}" y1="{yy:.1f}" x2="{pad_l+pw}" y2="{yy:.1f}" stroke="#bbb" stroke-dasharray="4 4"/>')
        out.append(f'<text x="{pad_l+pw-4}" y="{yy-5:.1f}" font-size="10" fill="#888" text-anchor="end">{txt}</text>')
    out.append(f'<line x1="{pad_l}" y1="{pad_t+ph}" x2="{pad_l+pw}" y2="{pad_t+ph}" stroke="#999"/>')
    out.append(f'<line x1="{pad_l}" y1="{pad_t}" x2="{pad_l}" y2="{pad_t+ph}" stroke="#999"/>')
    for s in series:
        x, y = s["x"], s["y"]
        ok = np.isfinite(y)
        if ok.sum() < 2:
            continue
        pts = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in zip(x[ok], y[ok]))
        out.append(f'<polyline points="{pts}" fill="none" stroke="{s["color"]}" '
                   f'stroke-width="{s.get("w",1.5)}" opacity="{s.get("op",0.95)}"/>')
    # legend
    ly = pad_t + 6
    for s in series:
        out.append(f'<line x1="{pad_l+pw+12}" y1="{ly}" x2="{pad_l+pw+32}" y2="{ly}" stroke="{s["color"]}" stroke-width="{s.get("w",1.5)}"/>')
        out.append(f'<text x="{pad_l+pw+38}" y="{ly+4}" font-size="11" fill="#333">{s["label"]}</text>')
        ly += 18
    out.append(f'<text x="{pad_l+pw/2}" y="{height-10}" font-size="12" fill="#333" text-anchor="middle">{xlabel}</text>')
    out.append(f'<text x="16" y="{pad_t+ph/2}" font-size="12" fill="#333" '
               f'transform="rotate(-90 16 {pad_t+ph/2})" text-anchor="middle">{ylabel}</text>')
    out.append("</svg>")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w").write("\n".join(out))


def svg_path_overlay(path, session, res, corners, title, width=760, height=700):
    pad = 46
    P = session.P
    xs, ys = P[:, 0], P[:, 1]
    x0, x1, y0, y1 = xs.min() - 0.8, xs.max() + 0.8, ys.min() - 0.8, ys.max() + 0.8
    scale = min((width - 2 * pad) / (x1 - x0), (height - 2 * pad) / (y1 - y0))

    def X(v):
        return pad + (v - x0) * scale

    def Y(v):
        return height - pad - (v - y0) * scale

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
           f'style="background:#fff;font-family:-apple-system,Segoe UI,Helvetica,Arial">']
    out.append(f'<text x="{pad}" y="24" font-size="15" fill="#111">{title}</text>')
    pl = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in P)
    out.append(f'<polyline points="{pl}" fill="none" stroke="#1f77b4" stroke-width="2.5" opacity="0.75"/>')
    tl = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in zip(res["x"], res["y"]))
    out.append(f'<polyline points="{tl}" fill="none" stroke="#2ca02c" stroke-width="1.2" opacity="0.9"/>')
    # carrot polylines only where they differ noticeably from the trajectory
    for k, c in enumerate(res["carrot"]):
        if c is None:
            continue
        if k % 4 == 0:
            out.append(f'<circle cx="{X(c[0]):.1f}" cy="{Y(c[1]):.1f}" r="1.3" fill="#ff7f0e" opacity="0.55"/>')
    for c in corners:
        out.append(f'<circle cx="{X(c["x"]):.1f}" cy="{Y(c["y"]):.1f}" r="4" fill="none" stroke="#d62728" stroke-width="1.6"/>')
        out.append(f'<text x="{X(c["x"])+7:.1f}" y="{Y(c["y"])-5:.1f}" font-size="10" fill="#d62728">'
                   f'{c["label"]} {c["turn_deg"]:+.0f}°</text>')
    out.append(f'<rect x="{pad}" y="{pad}" width="12" height="4" fill="#1f77b4"/>')
    out.append(f'<text x="{pad+18}" y="{pad+5}" font-size="11" fill="#333">global path</text>')
    out.append(f'<rect x="{pad}" y="{pad+18}" width="12" height="4" fill="#2ca02c"/>')
    out.append(f'<text x="{pad+18}" y="{pad+23}" font-size="11" fill="#333">robot (recorded)</text>')
    out.append(f'<circle cx="{pad+6}" cy="{pad+36}" r="3" fill="#ff7f0e"/>')
    out.append(f'<text x="{pad+18}" y="{pad+40}" font-size="11" fill="#333">carrot (replayed, every 4th)</text>')
    out.append(f'<circle cx="{pad+6}" cy="{pad+54}" r="4" fill="none" stroke="#d62728"/>')
    out.append(f'<text x="{pad+18}" y="{pad+58}" font-size="11" fill="#333">path corner &gt;45° / 1.2m</text>')
    out.append("</svg>")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w").write("\n".join(out))


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------


def fmt(v, nd=2, unit=""):
    if v is None or (isinstance(v, float) and not np.isfinite(v)):
        return "—"
    return f"{v:.{nd}f}{unit}"


def run_variant(variant_name, params, sessions, Tracker, pose_lag, corners_by_session):
    rows = []
    for ss in sessions:
        res = replay(ss, Tracker, params, pose_lag=pose_lag)
        m = metrics(ss, res, params, corners_by_session[ss.name])
        m["score"] = score(m)
        rows.append(dict(session=ss.name, res=res, metrics=m))
    agg = {}
    for key in ("L_p50", "L_short1m_frac", "dtheta_p50", "dtheta_p95", "dtheta_gt60_frac",
                "dtheta_rate_p95", "dtheta_max_corner", "landing_dtheta_max", "score",
                "carrot_step_p95", "L_at_min_frac", "L_at_max_frac"):
        vals = [r["metrics"][key] for r in rows if np.isfinite(r["metrics"].get(key, np.nan))]
        agg[key] = float(np.mean(vals)) if vals else float("nan")
    return dict(name=variant_name, params=params, per_session=rows, agg=agg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", action="append", required=True, help="session dir (repeatable)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tracker", default=DEFAULT_TRACKER, help="path to adaptive_path_tracker.py")
    ap.add_argument("--pose-lag-sec", default="auto",
                    help="pose delay used for path progress: seconds, or 'auto' to fit, or 0")
    ap.add_argument("--sweep", default="none", choices=["none", "all"],
                    help="'all' runs the 1-D parameter sweep")
    ap.add_argument("--mode", default=None,
                    help="override adaptive_mode for the baseline (curvature_only|clearance_only|both|none)")
    args = ap.parse_args()

    Tracker = load_tracker_class(args.tracker)
    sessions = [Session(s) for s in args.session]
    baseline = dict(BASELINE)
    if args.mode:
        baseline["adaptive_mode"] = args.mode

    # pose lag
    if str(args.pose_lag_sec).lower() == "auto":
        fits = {}
        for ss in sessions:
            tau, err, n = fit_pose_lag(ss, Tracker, baseline)
            fits[ss.name] = dict(tau=tau, median_err_m=err, n=n)
        pose_lag = float(np.median([f["tau"] for f in fits.values()]))
    else:
        pose_lag = float(args.pose_lag_sec)
        fits = {ss.name: dict(tau=pose_lag, median_err_m=None, n=None) for ss in sessions}

    corners = {ss.name: detect_corners(ss) for ss in sessions}

    results = {}
    results["baseline"] = run_variant("baseline", baseline, sessions, Tracker, pose_lag, corners)
    if pose_lag > 1e-6:
        results["baseline_lag0"] = run_variant("baseline_lag0", baseline, sessions, Tracker, 0.0, corners)

    if args.sweep == "all":
        for key, vals in SWEEP.items():
            for v in vals:
                p = dict(baseline)
                p[key] = v
                name = f"{key}={v:g}"
                results[name] = run_variant(name, p, sessions, Tracker, pose_lag, corners)
        # a couple of 2-D probes that Stage 1 would otherwise have to spend runs on
        for lmin, wh in [(0.5, 60.0), (0.5, 70.0), (0.35, 60.0), (0.7, 60.0)]:
            p = dict(baseline)
            p["lookahead_min"] = lmin
            p["turn_angle_high_deg"] = wh
            results[f"lmin={lmin:g},turn_high={wh:g}"] = run_variant(
                f"lmin={lmin:g},turn_high={wh:g}", p, sessions, Tracker, pose_lag, corners)

    # ---- outputs -------------------------------------------------------
    os.makedirs(args.out, exist_ok=True)
    figdir = os.path.join(args.out, "figures")

    payload = {
        "generated_by": "scripts/stage0_lookahead_replay.py",
        "tracker_module": args.tracker,
        "pose_lag_sec_used": pose_lag,
        "pose_lag_fit": fits,
        "baseline_params": baseline,
        "sessions": {ss.name: dict(path=ss.dir, task_window=[ss.i0, ss.i1],
                                   path_len_2d_m=ss.path_len,
                                   path_points=len(ss.path_pts)) for ss in sessions},
        "corners": corners,
        "variants": {k: dict(name=v["name"], params=v["params"], agg=v["agg"],
                             per_session={r["session"]: r["metrics"] for r in v["per_session"]})
                     for k, v in results.items()},
    }
    json.dump(payload, open(os.path.join(args.out, "stage0_metrics.json"), "w"), indent=2, default=float)

    # figures: L(s) and dtheta(s) per session, baseline vs lag-free vs two sweep extremes
    pick = ["baseline"]
    if "baseline_lag0" in results:
        pick.append("baseline_lag0")
    for cand in ("lookahead_max=2.5", "lookahead_min=0.9", "window_dist=1.0", "turn_angle_high_deg=70"):
        if cand in results:
            pick.append(cand)
    colors = ["#d62728", "#2ca02c", "#1f77b4", "#9467bd", "#ff7f0e", "#8c564b", "#17becf"]
    for ss in sessions:
        series = []
        for k, name in enumerate(pick):
            v = results.get(name)
            if not v:
                continue
            r = next((r for r in v["per_session"] if r["session"] == ss.name), None)
            if r is None:
                continue
            series.append(dict(x=r["res"]["s"], y=r["res"]["L"], label=name,
                               color=colors[k % len(colors)], w=1.9 if k == 0 else 1.1,
                               op=1.0 if k == 0 else 0.75))
        svg_line_plot(os.path.join(figdir, f"l_vs_s_{ss.name}.svg"), series,
                      "path progress s [m]", "lookahead L [m]",
                      f"Lookahead vs path progress — {ss.name} (pose lag {pose_lag:.2f}s)",
                      y_range=(0, 2.6))
        series = []
        for k, name in enumerate(pick):
            v = results.get(name)
            if not v:
                continue
            r = next((r for r in v["per_session"] if r["session"] == ss.name), None)
            if r is None:
                continue
            series.append(dict(x=r["res"]["s"], y=r["res"]["adtheta"], label=name,
                               color=colors[k % len(colors)], w=1.9 if k == 0 else 1.1,
                               op=1.0 if k == 0 else 0.7))
        svg_line_plot(os.path.join(figdir, f"dtheta_vs_s_{ss.name}.svg"), series,
                      "path progress s [m]", "|bear to carrot| [deg]",
                      f"Target demand vs path progress — {ss.name}",
                      y_range=(0, 180), hlines=[(60, "60°"), (90, "90°")])
        base_res = next(r for r in results["baseline"]["per_session"] if r["session"] == ss.name)["res"]
        svg_path_overlay(os.path.join(figdir, f"carrot_overlay_{ss.name}.svg"),
                         ss, base_res, corners[ss.name], f"Carrot vs path — {ss.name}")

    # sweep figures
    if args.sweep == "all":
        corner_labels = [c["label"] for c in corners[sessions[0].name]]
        for key in SWEEP:
            xs, ys, y2 = [], [], []
            per_corner = [[] for _ in corner_labels]
            for v in SWEEP[key]:
                r = results.get(f"{key}={v:g}")
                if not r:
                    continue
                xs.append(v)
                ys.append(r["agg"]["dtheta_max_corner"])
                y2.append(r["agg"]["dtheta_p95"])
                for ci in range(len(corner_labels)):
                    vals = [m["metrics"]["corners"][ci]["dtheta_max"] for m in r["per_session"]
                            if len(m["metrics"]["corners"]) > ci]
                    per_corner[ci].append(float(np.nanmean(vals)) if vals else np.nan)
            if len(xs) < 2:
                continue
            series = [dict(x=np.array(xs), y=np.array(per_corner[ci]), label=corner_labels[ci],
                           color=colors[(ci + 2) % len(colors)], w=1.8)
                      for ci in range(len(corner_labels))]
            series.append(dict(x=np.array(xs), y=np.array(y2), label="dΘ p95 (all frames)",
                               color="#111111", w=1.4, op=0.6))
            svg_line_plot(os.path.join(figdir, f"sweep_{key}.svg"), series,
                          key, "peak |ΔΘ| at corner [deg]",
                          f"Sweep: {key} (mean of {len(sessions)} sessions)")

    # ---- markdown ------------------------------------------------------
    md = []
    md.append("# Stage 0 前瞻参数离线回放（自动生成）\n")
    md.append(f"- tracker: `{args.tracker}`")
    md.append(f"- pose lag used: **{pose_lag:.2f} s**  (fit: "
              + ", ".join(f"{k.split('_')[-2]}: τ={v['tau']:.2f}s/err={fmt(v['median_err_m'],3)}m"
                          for k, v in fits.items()) + ")")
    md.append(f"- baseline params: `{json.dumps(baseline, ensure_ascii=False)}`\n")
    md.append("## 会话与拐角\n")
    md.append("| session | task window | 2D path | corners |")
    md.append("|---|---|---:|---:|")
    for ss in sessions:
        md.append(f"| {ss.name} | {ss.i0}–{ss.i1} | {ss.path_len:.2f} m | {len(corners[ss.name])} |")
    md.append("")
    for ss in sessions:
        md.append(f"**{ss.name} corners**\n")
        md.append("| label | at | turn | L@pre(0.8–0.2m) | dtheta max |")
        md.append("|---|---|---:|---:|---:|")
        base_res = next(r for r in results["baseline"]["per_session"] if r["session"] == ss.name)
        for c in base_res["metrics"]["corners"]:
            md.append(f"| {c['label']} | ({c['s']:.2f} m) | {c['turn_deg']:+.0f}° | "
                      f"{fmt(c['L_pre'])} | {fmt(c['dtheta_max'],0)}° |")
        md.append("")
    md.append("## 变体汇总（两会话均值）\n")
    corner_labels = [c["label"] for c in corners[sessions[0].name]]
    hdr = "| variant | L p50 | L<1m frac | L@min frac | dΘ p50 | dΘ p95 | dΘ>60° | dΘ/s p95 | 拐角 dΘmax | 平台 dΘmax |"
    hdr += "".join(f" {lb} |" for lb in corner_labels)
    hdr += " score |"
    md.append(hdr)
    md.append("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|" + "---:|" * len(corner_labels) + "---:|")
    order = ["baseline"] + (["baseline_lag0"] if "baseline_lag0" in results else []) + \
            [k for k in results if k not in ("baseline", "baseline_lag0")]
    for k in order:
        v = results[k]
        a = v["agg"]
        row = "| {n} | {a} | {b} | {c} | {d} | {e} | {f} | {g} | {h} | {i} |".format(
            n=k,
            a=fmt(a["L_p50"]), b=fmt(a["L_short1m_frac"], 2), c=fmt(a["L_at_min_frac"], 2),
            d=fmt(a["dtheta_p50"], 0), e=fmt(a["dtheta_p95"], 0), f=fmt(a["dtheta_gt60_frac"], 2),
            g=fmt(a["dtheta_rate_p95"], 0), h=fmt(a["dtheta_max_corner"], 0),
            i=fmt(a["landing_dtheta_max"], 0))
        for ci in range(len(corner_labels)):
            vals = [m["metrics"]["corners"][ci]["dtheta_max"] for m in v["per_session"]
                    if len(m["metrics"]["corners"]) > ci]
            row += f" {fmt(float(np.nanmean(vals)) if vals else np.nan, 0)} |"
        row += f" {fmt(a['score'], 1)} |"
        md.append(row)
    md.append("")
    md.append("## 各会话明细（baseline）\n")
    for r in results["baseline"]["per_session"]:
        m = r["metrics"]
        md.append(f"### {r['session']}\n")
        md.append("| 指标 | 值 |")
        md.append("|---|---:|")
        for key in ("n_frames", "L_p5", "L_p50", "L_p95", "L_at_min_frac", "L_at_max_frac",
                    "L_short1m_frac", "dtheta_p50", "dtheta_p95", "dtheta_gt60_frac",
                    "dtheta_gt90_frac", "dtheta_rate_p95", "carrot_step_p95", "carrot_step_max",
                    "dtheta_max_corner", "landing_dtheta_max", "score"):
            md.append(f"| {key} | {fmt(m.get(key), 3)} |")
        if m.get("stair_landings"):
            md.append("\nstair landings:")
            for q in m["stair_landings"]:
                md.append(f"- frames {q['frames']} t={q['t'][0]:.1f}–{q['t'][1]:.1f}s at "
                          f"({q['x']:.2f},{q['y']:.2f}) z={q['z']:.2f}: L {q['L_min']:.2f}–{q['L_max']:.2f} m, "
                          f"dΘmax {q['dtheta_max']:.0f}°, yaw swept {q['yaw_span_deg']:.0f}°")
        md.append("")
    open(os.path.join(args.out, "stage0_report.md"), "w").write("\n".join(md))

    print(f"[stage0] pose lag = {pose_lag:.2f} s")
    for ss in sessions:
        c = corners[ss.name]
        print(f"[stage0] {ss.name}: task {ss.i0}-{ss.i1}, {len(c)} corners, path {ss.path_len:.2f} m")
    print(f"[stage0] wrote {args.out}/stage0_metrics.json, stage0_report.md, figures/")


if __name__ == "__main__":
    sys.exit(main())

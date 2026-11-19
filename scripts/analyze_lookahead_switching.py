#!/usr/bin/env python3
"""Offline audit of *how the lookahead switches* between long (flat, obstacle
avoidance) and short (corner / stairs / door), and evaluation of candidate
switching rules.

Why this exists
---------------
On the robot the lookahead has two jobs:
  * flat open ground -> long lookahead, so the policy can see obstacles early;
  * corners, stairs, narrow doors -> short lookahead (0.40 m already works).
The switch between them must be **accurate** (don't shrink on straights, don't
stay short on open ground) and **sensitive** (recover to long right after a
corner, because that is exactly when obstacle avoidance matters again).

This script measures the deployed curvature rule on recorded sessions and
compares it with rule candidates, using a *distance-to-corner* function

    d_corner(s) = distance along the path until the path heading deviates from
                  the heading at s by >= THRESH degrees

which is the quantity a "switch at the right distance" rule should use.

Rules
-----
  curv      the deployed AdaptivePathSegmentTracker (curvature accumulation)
  curv_fast same, but slew_rate_increase raised (recovery only)
  corner    L = clamp(d_corner - margin, Lmin, Lmax)
  hybrid    L = min(curv_target, clamp(d_corner - margin, Lmin, Lmax))

Metrics
-------
  flat_at_max   frac of frames with d_corner>=2.0 m and L >= 0.98*Lmax
  flat_short    frac of frames with d_corner>=2.0 m and L < 1.0 m
  lead_m        distance to corner when L first drops below 1.0 m (per corner)
  recover_m     distance past a corner apex until L returns >= 1.5 m
  corner_dth    max |body-frame bearing to carrot| in [apex-1.2, apex+0.4] m
  dth_p95       p95 of |bearing| over all adaptive frames

Usage
-----
    python3 scripts/analyze_lookahead_switching.py \
        --session door_diagnostics/session_20261110_144216_realsense \
        --session door_diagnostics/session_20261118_102759_realsense \
        --pose-lag-sec auto --out door_diagnostics/analysis_switching
"""

import argparse
import importlib.util
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

DEFAULT_TRACKER = (
    "/home/yichangfeng/sru-robot-deployment/go2w_nav_ros2_ws/src/"
    "rl_nav_controller/rl_nav_controller/adaptive_path_tracker.py"
)

# --- tuning constants shared by the candidate rules -------------------------
STEP = 0.05            # path resample step [m]
HEAD_HALF = 0.30       # heading smoothing half-window [m]
DCORNER_THRESH = 25.0  # heading deviation that defines "a corner" [deg]
DCORNER_MAX = 6.0      # cap for the distance-to-corner search [m]
MARGIN = 0.25          # keep this much distance before the corner [m]
LMIN = 0.40
LMAX = 2.00


def load_tracker_class(path):
    spec = importlib.util.spec_from_file_location("deployed_tracker", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.AdaptivePathSegmentTracker


def load_session(path):
    """Minimal session reader (path snapshot + trajectory.csv)."""
    import csv
    import glob
    d = os.path.abspath(path)
    rows = list(csv.DictReader(open(os.path.join(d, "trajectory.csv"))))
    pj = json.load(open(sorted(glob.glob(os.path.join(d, "paths", "*.json")))[0]))
    pts = pj["points"]

    def f(r, k, dflt=float("nan")):
        v = r.get(k, "")
        if v in ("", None):
            return dflt
        try:
            return float(v)
        except (TypeError, ValueError):
            return dflt

    t = np.array([f(r, "epoch_sec") for r in rows])
    t = t - t[0]
    return dict(
        name=os.path.basename(d),
        dir=d,
        t=t,
        x=np.array([f(r, "robot_x") for r in rows]),
        y=np.array([f(r, "robot_y") for r in rows]),
        yaw=np.radians(np.array([f(r, "robot_yaw_deg", 0.0) for r in rows])),
        cmd_vx=np.nan_to_num(np.array([f(r, "cmd_vx", 0.0) for r in rows])),
        cmd_wz=np.nan_to_num(np.array([f(r, "cmd_wz", 0.0) for r in rows])),
        stair_down=np.nan_to_num(np.array([f(r, "is_descending_stairs", 0.0) for r in rows])) > 0.5,
        stair_up=np.nan_to_num(np.array([f(r, "is_ascending_stairs", 0.0) for r in rows])) > 0.5,
        P=np.array([[p[0], p[1]] for p in pts]),
        points=pts,
    )


def path_geometry(P):
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    s = np.arange(0.0, cum[-1] + 1e-9, STEP)
    pts = np.stack([np.interp(s, cum, P[:, 0]), np.interp(s, cum, P[:, 1])], axis=1)
    n = max(1, int(round(HEAD_HALF / STEP)))
    lo = np.clip(np.arange(len(pts)) - n, 0, len(pts) - 1)
    hi = np.clip(np.arange(len(pts)) + n, 0, len(pts) - 1)
    v = pts[hi] - pts[lo]
    h = np.arctan2(v[:, 1], v[:, 0])
    return s, pts, h, cum[-1]


def d_corner_table(h, thresh_deg=DCORNER_THRESH, max_look=DCORNER_MAX):
    """For every resampled point: distance ahead until heading deviates >= thresh."""
    n = int(max_look / STEP)
    out = np.full(len(h), max_look, dtype=float)
    thr = math.radians(thresh_deg)
    for k in range(len(h)):
        lim = min(len(h), k + n)
        dev = np.abs((h[k:lim] - h[k] + math.pi) % (2 * math.pi) - math.pi)
        w = np.where(dev >= thr)[0]
        if len(w):
            out[k] = w[0] * STEP
    return out


def d_at(s_arr, s_grid, table):
    idx = np.clip(np.round(s_arr / STEP).astype(int), 0, len(table) - 1)
    return table[idx]


def project(P, xy, prev_seg=0):
    """Nearest projection onto the polyline -> (dist_m, arc_s). Simple O(N)."""
    best_d, best_s = float("inf"), 0.0
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    for i in range(len(P) - 1):
        v = P[i + 1] - P[i]
        l2 = float(v @ v)
        tt = 0.0 if l2 < 1e-12 else float(np.clip((xy - P[i]) @ v / l2, 0.0, 1.0))
        proj = P[i] + tt * v
        dd = float(np.sum((xy - proj) ** 2))
        if dd < best_d:
            best_d = dd
            best_s = cum[i] + tt * (cum[i + 1] - cum[i])
    return math.sqrt(best_d), best_s


def point_at(pts_grid, cum_pts, s):
    s = float(np.clip(s, 0.0, cum_pts[-1]))
    i = int(np.clip(np.searchsorted(cum_pts, s) - 1, 0, len(pts_grid) - 2))
    seg = cum_pts[i + 1] - cum_pts[i]
    r = 0.0 if seg < 1e-9 else (s - cum_pts[i]) / seg
    return pts_grid[i] + r * (pts_grid[i + 1] - pts_grid[i])


def clamp_l(x, lo, hi):
    return max(lo, min(hi, x))


def slew(prev, target, dt, dec, inc):
    if prev is None:
        return target
    if target < prev:
        return max(target, prev - dec * dt)
    return min(target, prev + inc * dt)


def replay_curv(session, tracker_cls, lag, slew_inc=0.8, turn_low=20.0):
    tr = tracker_cls(lookahead_min=LMIN, lookahead_max=LMAX, window_dist=1.6,
                     turn_angle_low_deg=turn_low, turn_angle_high_deg=50.0,
                     slew_rate_decrease=2.5, slew_rate_increase=slew_inc,
                     adaptive_mode="curvature_only")
    tr.set_path(session["points"])
    out = []
    for i in range(len(session["t"])):
        x, y = session["x"][i], session["y"][i]
        tq = session["t"][i] - lag
        if lag > 0:
            x = float(np.interp(tq, session["t"], session["x"]))
            y = float(np.interp(tq, session["t"], session["y"]))
        tr.update_progress([x, y])
        if session["stair_down"][i]:
            L, br = LMIN, "stair"
        elif session["stair_up"][i]:
            L, br = 0.45, "stair"
        else:
            cp, cy, db = tr.get_adaptive_lookahead_point(current_time=session["t"][i], depth_image=None)
            L, br = float(db.get("lookahead", LMIN)), "ad"
        out.append((tr.current_progress, L, br))
    return out


def replay_deployed(session, tracker_cls, lag, switch_mode, slew_inc,
                    thresh=DCORNER_THRESH, margin=MARGIN):
    """Replay with the *deployed* tracker's own switch-mode implementation."""
    tr = tracker_cls(lookahead_min=LMIN, lookahead_max=LMAX, window_dist=1.6,
                     turn_angle_low_deg=20.0, turn_angle_high_deg=50.0,
                     slew_rate_decrease=2.5, slew_rate_increase=slew_inc,
                     adaptive_mode="curvature_only",
                     switch_mode=switch_mode,
                     corner_switch_thresh_deg=thresh,
                     corner_switch_margin=margin)
    tr.set_path(session["points"])
    out = []
    for i in range(len(session["t"])):
        x = float(np.interp(session["t"][i] - lag, session["t"], session["x"]))
        y = float(np.interp(session["t"][i] - lag, session["t"], session["y"]))
        tr.update_progress([x, y])
        if session["stair_down"][i]:
            L, br = LMIN, "stair"
        elif session["stair_up"][i]:
            L, br = 0.45, "stair"
        else:
            _, _, db = tr.get_adaptive_lookahead_point(
                current_time=session["t"][i], depth_image=None)
            L, br = float(db.get("lookahead", LMIN)), "ad"
        out.append((float(tr.current_progress), L, br))
    return out


def replay_rule(session, rule, lag, slew_inc):
    """Rule-driven replay: corner / hybrid / curv_fast share this path."""
    pts = session["points"]
    P = session["P"]
    s_grid, pts_grid, h, total = path_geometry(P)
    cum_pts = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts_grid, axis=0), axis=1))])
    dct = d_corner_table(h)

    # curv baseline (only needed for 'hybrid' / 'curv_fast')
    curv = None
    if rule in ("hybrid", "curv_fast"):
        curv = replay_curv(session, load_tracker_class(DEFAULT_TRACKER), lag, slew_inc)

    out = []
    prev = None
    for i in range(len(session["t"])):
        s = float(session["s"][i]) if "s" in session else None
        if s is None:
            x = float(np.interp(session["t"][i] - lag, session["t"], session["x"]))
            y = float(np.interp(session["t"][i] - lag, session["t"], session["y"]))
            _, s = project(P, np.array([x, y]))
        if session["stair_down"][i] or session["stair_up"][i]:
            out.append((s, LMIN if session["stair_down"][i] else 0.45, "stair"))
            prev = None
            continue
        dc = d_at(np.array([s]), s_grid, dct)[0]
        target = clamp_l(dc - MARGIN, LMIN, LMAX)
        if rule == "hybrid":
            ct = curv[i][1]
            target = min(target, ct)
        if rule == "curv_fast":
            target = curv[i][1]
        dt = 0.05 if prev is None else max(0.001, min(0.5, 0.2))
        L = slew(prev, target, dt, 2.5, slew_inc)
        out.append((s, L, "ad"))
        prev = L
    return out


def metrics(session, series, corners):
    P = session["P"]
    s_grid, pts_grid, h, total = path_geometry(P)
    cum_pts = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts_grid, axis=0), axis=1))])
    dct = d_corner_table(h)

    L = np.array([r[1] for r in series])
    s = np.array([r[0] for r in series])
    br = np.array([r[2] for r in series], dtype=object)
    # 只统计任务段（cmd 非零的首末之间），与离线裁剪口径一致；
    # 返程手控时 progress 单调不动，会把夹角/恢复指标污染掉
    nz = np.where(np.abs(session["cmd_vx"]) > 1e-6)[0]
    task = np.zeros(len(L), dtype=bool)
    if len(nz):
        task[max(0, nz[0]):min(len(L), nz[-1] + 1)] = True
    moving = (np.abs(session["cmd_vx"]) > 0.01) | (np.abs(session["cmd_wz"]) > 0.01)
    # 暂停段（cmd=0 但仍在任务窗内）不是跟踪决策，拐角统计要排除
    ad = (br == "ad") & task & moving
    dc = d_at(s, s_grid, dct)
    flat = ad & (dc >= 2.0)
    short = ad & (dc < 1.0)

    m = {}
    m["n_ad"] = int(ad.sum())
    m["flat_n"] = int(flat.sum())
    m["flat_at_max"] = float(np.mean(L[flat] >= 0.98 * LMAX)) if flat.sum() else float("nan")
    m["flat_short_frac"] = float(np.mean(L[flat] < 1.0)) if flat.sum() else float("nan")
    m["flat_L_p50"] = float(np.median(L[flat])) if flat.sum() else float("nan")
    m["flat_L_p10"] = float(np.percentile(L[flat], 10)) if flat.sum() else float("nan")
    m["short_n"] = int(short.sum())

    # corner approach / recovery around the offline-detected corners
    leads, recovers, dths = [], [], []
    for c in corners:
        # 收缩提前量：只看进弯之前
        pre_win = ad & (s >= c["s"] - 1.5) & (s <= c["s"] - 0.1)
        pre = np.where(pre_win)[0]
        if len(pre):
            below = pre[L[pre] < 1.0]
            leads.append(float(c["s"] - s[below[0]]) if len(below) else float("nan"))
        # 恢复距离：出弯后 L 回到 >=1.5 m 所需的弧长（窗口放宽到 3 m）
        post = np.where(ad & (s >= c["s"]) & (s <= c["s"] + 3.0))[0]
        if len(post):
            up = post[L[post] >= 1.5]
            recovers.append(float(s[up[0]] - c["s"]) if len(up) else float("nan"))
        # 目标夹角只统计进弯段，避免终点钳位造成的 180 度
        dth = []
        for i in pre:
            cp = point_at(pts_grid, cum_pts, s[i] + L[i])
            v = cp - np.array([session["x"][i], session["y"][i]])
            dth.append(abs(math.degrees((math.atan2(v[1], v[0]) - session["yaw"][i] + math.pi) % (2 * math.pi) - math.pi)))
        dths.append(float(np.max(dth)) if dth else float("nan"))

    m["lead_m_p50"] = float(np.nanmedian(leads)) if leads else float("nan")
    m["recover_m_p50"] = float(np.nanmedian(recovers)) if recovers else float("nan")
    m["recover_m_p90"] = float(np.nanpercentile([r for r in recovers if np.isfinite(r)], 90)) if recovers else float("nan")
    m["corner_dth_max"] = float(np.nanmax(dths)) if dths else float("nan")

    # overall bearing p95
    all_dth = []
    for i in np.where(ad)[0]:
        cp = point_at(pts_grid, cum_pts, s[i] + L[i])
        v = cp - np.array([session["x"][i], session["y"][i]])
        all_dth.append(abs(math.degrees((math.atan2(v[1], v[0]) - session["yaw"][i] + math.pi) % (2 * math.pi) - math.pi)))
    m["dth_p95"] = float(np.percentile(all_dth, 95)) if all_dth else float("nan")
    return m


def detect_corners(session, half=0.6, thresh=45.0, merge=2.0):
    s_grid, pts_grid, h, total = path_geometry(session["P"])
    n = int(round(2 * half / STEP))
    fwd = np.clip(np.arange(len(h)) + n, 0, len(h) - 1)
    turn = np.degrees((h[fwd] - h + math.pi) % (2 * math.pi) - math.pi)
    mag = np.abs(turn)
    cand = np.where(mag >= thresh)[0]
    out = []
    if len(cand) == 0:
        return out
    groups = [[cand[0]]]
    for i in cand[1:]:
        if s_grid[i] - s_grid[groups[-1][-1]] <= merge:
            groups[-1].append(i)
        else:
            groups.append([i])
    for g in groups:
        a = g[int(np.argmax(mag[g]))]
        out.append(dict(s=float(s_grid[a]), x=float(pts_grid[a, 0]), y=float(pts_grid[a, 1]),
                        turn=float(turn[a])))
    return out


def fit_lag(session, tracker_cls):
    best = (0.0, float("inf"))
    for tau in np.arange(0.0, 2.01, 0.2):
        tr = tracker_cls(lookahead_min=LMIN, lookahead_max=LMAX, window_dist=1.6,
                         turn_angle_low_deg=20.0, turn_angle_high_deg=50.0,
                         slew_rate_decrease=2.5, slew_rate_increase=0.8,
                         adaptive_mode="curvature_only")
        tr.set_path(session["points"])
        errs = []
        for i in range(len(session["t"])):
            x = float(np.interp(session["t"][i] - tau, session["t"], session["x"]))
            y = float(np.interp(session["t"][i] - tau, session["t"], session["y"]))
            tr.update_progress([x, y])
            if session["stair_down"][i]:
                cp, _ = tr.get_lookahead_point(LMIN)
            else:
                cp, _, _ = tr.get_adaptive_lookahead_point(current_time=session["t"][i], depth_image=None)
            if cp is None:
                continue
            c = np.array([float(session["carrot"][i][0]), float(session["carrot"][i][1])]) \
                if session.get("carrot") is not None else None
            if c is None or not np.isfinite(c).all():
                continue
            errs.append(np.linalg.norm(c - np.array(cp[:2])))
        if errs:
            med = float(np.median(errs))
            if med < best[1]:
                best = (float(tau), med)
    return best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", action="append", required=True)
    ap.add_argument("--tracker", default=DEFAULT_TRACKER)
    ap.add_argument("--out", default=None)
    ap.add_argument("--pose-lag-sec", default="auto")
    args = ap.parse_args()

    tracker_cls = load_tracker_class(args.tracker)
    sessions = [load_session(p) for p in args.session]

    # pose lag fit needs the recorded carrot
    for ss in sessions:
        ss["carrot"] = np.array([[float(r.get("carrot_x") or float("nan")),
                                  float(r.get("carrot_y") or float("nan"))]
                                 for r in csv_rows(ss["dir"])])

    if args.pose_lag_sec == "auto":
        taus = [fit_lag(ss, tracker_cls) for ss in sessions]
        lag = float(np.median(taus))
    else:
        lag = float(args.pose_lag_sec)
        taus = [lag] * len(sessions)

    rules = [
        ("curv", dict(slew_inc=0.8)),
        ("curv_fast", dict(slew_inc=2.0)),
        ("curv_fast4", dict(slew_inc=4.0)),
        ("curv_low30", dict(slew_inc=2.0, turn_low=30.0)),
        ("curv_low40", dict(slew_inc=2.0, turn_low=40.0)),
        ("corner", dict(slew_inc=2.0)),
        ("corner_dep", dict(switch_mode="corner_dist", slew_inc=2.0)),
        ("hybrid_dep", dict(switch_mode="hybrid", slew_inc=2.0)),
    ]

    print(f"pose lag = {lag:.2f} s   (fits: {[round(t,2) for t in taus]})\n")
    rows = []
    for ss in sessions:
        corners = detect_corners(ss)
        print(f"### {ss['name']}  ({len(corners)} corners)")
        print(f"{'rule':11s} {'flat@max':>9s} {'flat<1m':>8s} {'L_p50':>6s} {'L_p10':>6s} {'lead_m':>7s} "
              f"{'recov_m':>8s} {'recov_p90':>10s} {'corner_dΘ':>9s} {'dΘp95':>6s}")
        for rname, kw in rules:
            if "switch_mode" in kw:
                series = replay_deployed(ss, tracker_cls, lag,
                                         kw["switch_mode"], kw["slew_inc"])
            elif rname in ("corner", "hybrid"):
                series = replay_rule(ss, rname, lag, kw["slew_inc"])
            else:
                series = replay_curv(ss, tracker_cls, lag, kw["slew_inc"],
                                     kw.get("turn_low", 20.0))
            m = metrics(ss, series, corners)
            rows.append(dict(session=ss["name"], rule=rname, **m))
            print(f"{rname:11s} {m['flat_at_max']*100:8.0f}% {m['flat_short_frac']*100:7.0f}% "
                  f"{m['flat_L_p50']:6.2f} {m['flat_L_p10']:6.2f} "
                  f"{m['lead_m_p50']:7.2f} {m['recover_m_p50']:8.2f} {m['recover_m_p90']:10.2f} "
                  f"{m['corner_dth_max']:9.0f} {m['dth_p95']:6.0f}")
        print()

    if args.out:
        os.makedirs(args.out, exist_ok=True)
        json.dump({"pose_lag_sec": lag, "lag_fits": taus, "rules": rows},
                  open(os.path.join(args.out, "switching_metrics.json"), "w"), indent=2)
        print(f"wrote {args.out}/switching_metrics.json")
    return 0


def csv_rows(d):
    import csv
    return list(csv.DictReader(open(os.path.join(d, "trajectory.csv"))))


if __name__ == "__main__":
    sys.exit(main())

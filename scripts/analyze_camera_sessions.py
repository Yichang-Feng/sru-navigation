#!/usr/bin/env python3
"""Door-diagnostic camera A/B analysis (RealSense vs ZED, or any two sessions).

Reads two ``door_diagnostic_logger`` sessions (schema 2.0), trims the manual
teleop tail (everything after the last non-zero ``/cmd_vel`` sample), and
produces:

* ``trimmed_<label>.csv``          -- mission-only trajectory (originals untouched)
* ``run_metrics.json``             -- per-run and paired metrics
* ``matched_pairs.csv``            -- 1:1 pose-matched frame pairs
* ``figures/*.svg``                -- trajectory, progress profiles, depth case
* ``report_auto.md``               -- tables generated from the numbers

The metric definitions follow ``docs/deployment/REALSENSE_DEPLOYMENT_EVAL_AND_PARAM_TUNING_PLAN.md``
(L0 sensor health / L1 matched-state comparison / L2 closed-loop behaviour).

Usage
-----
python3 scripts/analyze_camera_sessions.py \
  --a ~/sru-navigation-learning/door_diagnostics/session_..._realsense \
  --label-a RealSense \
  --b ~/sru-navigation-learning/door_diagnostics/session_..._zed \
  --label-b ZED \
  --out ~/sru-navigation-learning/door_diagnostics/analysis_20261110
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:  # raw-depth analysis is optional
    cv2 = None

CMD_COLUMNS = ("cmd_vx", "cmd_vy", "cmd_wz")


# ----------------------------------------------------------------------------
# Loading
# ----------------------------------------------------------------------------
def _to_float(value):
    if value is None:
        return np.nan
    text = str(value).strip()
    if text == "":
        return np.nan
    low = text.lower()
    if low == "true":
        return 1.0
    if low == "false":
        return 0.0
    try:
        return float(text)
    except ValueError:
        return np.nan


def load_session(path):
    path = os.path.expanduser(path)
    csv_path = os.path.join(path, "trajectory.csv")
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"trajectory.csv not found in {path}")

    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        raw_rows = list(reader)
        columns = reader.fieldnames or []

    data = {}
    for col in columns:
        data[col] = np.array([_to_float(row.get(col)) for row in raw_rows], dtype=float)

    meta = {}
    for name in ("session_summary.json", "session_meta.json", "camera_info.json"):
        candidate = os.path.join(path, name)
        if os.path.isfile(candidate):
            with open(candidate, encoding="utf-8") as f:
                meta.update(json.load(f))

    path_files = sorted(glob.glob(os.path.join(path, "paths", "*.json")))
    global_path = None
    if path_files:
        with open(path_files[0], encoding="utf-8") as f:
            pts = json.load(f).get("points") or []
        if pts:
            global_path = np.array(pts, dtype=float)

    return {
        "dir": path,
        "rows": raw_rows,
        "data": data,
        "meta": meta,
        "path": global_path,
    }


def trim_indices(session, min_cmd=1e-6):
    """First and last frame with a non-zero command (mission window)."""
    d = session["data"]
    moving = np.zeros(len(d["cmd_vx"]), dtype=bool)
    for col in CMD_COLUMNS:
        if col in d:
            moving |= np.abs(np.nan_to_num(d[col])) > min_cmd
    idx = np.where(moving)[0]
    if idx.size == 0:
        raise ValueError(f"no non-zero cmd samples in {session['dir']}")
    return int(idx[0]), int(idx[-1])


def slice_session(session, start, end):
    """Dict of arrays restricted to [start, end]."""
    return {k: v[start:end + 1].copy() for k, v in session["data"].items()}


def write_trimmed_csv(session, start, end, out_path):
    rows = session["rows"][start:end + 1]
    if not rows:
        return
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


# ----------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------
def fov_deg(camera_info):
    width = float(camera_info.get("width") or 0)
    height = float(camera_info.get("height") or 0)
    fx = float(camera_info.get("fx") or 0)
    fy = float(camera_info.get("fy") or 0)
    if width <= 0 or height <= 0 or fx <= 0 or fy <= 0:
        return None
    return {
        "h_deg": 2.0 * math.degrees(math.atan(width / (2.0 * fx))),
        "v_deg": 2.0 * math.degrees(math.atan(height / (2.0 * fy))),
    }


def pose_array(d):
    return np.stack([d["robot_x"], d["robot_y"], d["robot_z"]], axis=1)


def project_onto_path(poses, poly):
    """Progress in [0, 1] and distance to the 3-D polyline."""
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(cum[-1]) if cum.size else 0.0
    progress = np.zeros(len(poses))
    deviation = np.zeros(len(poses))
    for k, p in enumerate(poses):
        best_d, best_s = math.inf, 0.0
        for i in range(len(poly) - 1):
            a = poly[i]
            ab = poly[i + 1] - a
            l2 = float(ab @ ab)
            if l2 < 1e-12:
                continue
            t = float(np.clip((p - a) @ ab / l2, 0.0, 1.0))
            proj = a + t * ab
            dist = float(np.linalg.norm(p - proj))
            if dist < best_d:
                best_d, best_s = dist, cum[i] + t * seg[i]
        progress[k] = best_s / total if total > 0 else 0.0
        deviation[k] = best_d
    return progress, deviation


def matched_pairs(d_a, d_b, max_pos=0.25, max_yaw=8.0):
    """1:1 nearest-pose matching in 3-D position + yaw."""
    pa, pb = pose_array(d_a), pose_array(d_b)
    yaw_a, yaw_b = d_a["robot_yaw_deg"], d_b["robot_yaw_deg"]
    pairs = []
    used = set()
    for i in range(len(pa)):
        diff = pb - pa[i]
        dist = np.linalg.norm(diff, axis=1)
        dyaw = np.abs((yaw_b - yaw_a[i] + 180.0) % 360.0 - 180.0)
        cost = dist + dyaw * 0.01
        for j in np.argsort(cost):
            j = int(j)
            if j in used:
                continue
            if dist[j] <= max_pos and dyaw[j] <= max_yaw:
                used.add(j)
                pairs.append((i, j, float(dist[j]), float(dyaw[j])))
            break
    return pairs


def _finite(values):
    arr = np.asarray(values, dtype=float)
    return arr[np.isfinite(arr)]


def stats(values):
    arr = _finite(values)
    if arr.size == 0:
        return {}
    return {
        "n": int(arr.size),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p10": float(np.percentile(arr, 10)),
        "p90": float(np.percentile(arr, 90)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
    }


# ----------------------------------------------------------------------------
# Raw-depth analysis (perceived vs real nearness)
# ----------------------------------------------------------------------------
def _raw_depth(session, frame_idx):
    if cv2 is None:
        return None
    path = os.path.join(session["dir"], "depth_raw", f"frame_{int(frame_idx):06d}_raw_depth.png")
    if not os.path.isfile(path):
        return None
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    return img.astype(np.float32) / 1000.0


def _cell_grid(height, width, rows=40, cols=64):
    ys = np.linspace(0, height, rows + 1).astype(int)
    xs = np.linspace(0, width, cols + 1).astype(int)
    return ys, xs


def raw_cell_stats(depth_m, min_depth, max_depth, rows=40, cols=64):
    """Per-64x40-cell invalid fraction and minimum *valid* depth."""
    height, width = depth_m.shape
    valid = (depth_m >= min_depth) & (depth_m <= max_depth)
    ys, xs = _cell_grid(height, width, rows, cols)
    invalid_fraction = np.zeros((rows, cols))
    real_min = np.full((rows, cols), np.inf)
    for r in range(rows):
        for c in range(cols):
            cell = depth_m[ys[r]:ys[r + 1], xs[c]:xs[c + 1]]
            cell_valid = valid[ys[r]:ys[r + 1], xs[c]:xs[c + 1]]
            invalid_fraction[r, c] = 1.0 - float(cell_valid.mean())
            if cell_valid.any():
                real_min[r, c] = float(cell[cell_valid].min())
    return real_min, invalid_fraction


def policy_grid_angles(camera_info, rows=40, cols=64):
    """Bearing/elevation (deg) of each 64x40 cell centre."""
    width = float(camera_info["width"])
    height = float(camera_info["height"])
    fx, fy = float(camera_info["fx"]), float(camera_info["fy"])
    cx, cy = float(camera_info["cx"]), float(camera_info["cy"])
    u = (np.arange(cols) + 0.5) * width / cols
    v = (np.arange(rows) + 0.5) * height / rows
    bearing = np.degrees(np.arctan((u - cx) / fx))[None, :].repeat(rows, 0)
    elevation = np.degrees(np.arctan((v - cy) / fy))[:, None].repeat(cols, 1)
    return bearing, elevation


def depth_case(session, index, min_depth, max_depth):
    """Policy-input vs real-depth nearness for one frame."""
    d = session["data"]
    frame = int(d["frame_idx"][index])
    saved_path = os.path.join(session["dir"], "depth", f"frame_{frame:06d}_depth_40x64.npy")
    if not os.path.isfile(saved_path):
        return None
    saved = np.load(saved_path)
    bearing, elevation = policy_grid_angles(session["meta"], *saved.shape)
    center = (np.abs(bearing) < 20) & (np.abs(elevation) < 20)
    right = (bearing > 20) & (bearing < 40) & (np.abs(elevation) < 20)
    out = {
        "frame": frame,
        "saved": saved,
        "bearing": bearing,
        "elevation": elevation,
        "center_perceived_min": float(np.min(saved[center])),
        "right_perceived_min": float(np.min(saved[right])),
        "floor_avg": float(np.mean(saved[32:40, 20:44])),
    }
    depth_m = _raw_depth(session, frame)
    if depth_m is not None:
        real_min, invalid = raw_cell_stats(depth_m, min_depth, max_depth, *saved.shape)
        out.update({
            "center_real_min": float(np.min(real_min[center])),
            "right_real_min": float(np.min(real_min[right])),
            "invalid_fraction": float(np.mean(invalid)),
            "real_min": real_min,
            "invalid": invalid,
        })
    return out


# ----------------------------------------------------------------------------
# Run metrics
# ----------------------------------------------------------------------------
def run_metrics(label, session, start, end):
    d = slice_session(session, start, end)
    n = end - start + 1
    duration = float(np.nanmax(d["epoch_sec"]) - np.nanmin(d["epoch_sec"]))
    poses = pose_array(d)
    step = np.linalg.norm(np.diff(poses, axis=0), axis=1) if n > 1 else np.array([])
    distance = float(np.nansum(step))
    speed = _finite(d["speed_mps"])
    cmd_vx = _finite(d["cmd_vx"])

    result = {
        "label": label,
        "session_dir": session["dir"],
        "camera_id": session["meta"].get("camera_id"),
        "depth_topic": session["meta"].get("depth_topic"),
        "depth_encoding": session["meta"].get("depth_encoding"),
        "depth_size": session["meta"].get("depth_size"),
        "min_depth": session["meta"].get("min_depth"),
        "max_depth": session["meta"].get("max_depth"),
        "frames": int(n),
        "duration_sec": duration,
        "distance_m": distance,
        "mean_speed_mps": float(np.mean(speed)) if speed.size else None,
        "max_speed_mps": float(np.max(speed)) if speed.size else None,
        "mean_cmd_vx": float(np.mean(cmd_vx)) if cmd_vx.size else None,
        "max_cmd_vx": float(np.max(cmd_vx)) if cmd_vx.size else None,
        "depth_health": {
            "valid_ratio_raw": stats(d["depth_valid_ratio_raw"]),
            "hole_ratio_raw": stats(d["depth_hole_ratio"]),
            "nan_ratio_raw": stats(d["depth_nan_ratio"]),
            "zero_ratio_raw": stats(d["depth_zero_ratio"]),
            "far_ratio_raw": stats(d["depth_far_ratio"]),
            "valid_ratio_40x64": stats(d["depth_valid_ratio_40x64"]),
            "zero_ratio_40x64": stats(d["depth_zero_ratio_40x64"]),
            "latency_ms": stats(d["depth_latency_ms"]),
            "period_ms": stats(d["depth_period_ms"]),
        },
        "depth_roi": {
            "center_min": stats(d["depth_center_min"]),
            "center_avg": stats(d["depth_center_avg"]),
            "left_min": stats(d["depth_left_min"]),
            "right_min": stats(d["depth_right_min"]),
            "floor_near": stats(d["depth_floor_near"]),
            "center_void_ratio": stats(d["center_void_ratio"]),
        },
        "lidar_front": stats(d["lidar_min_dist_front"]),
        "stair_down_frames": int(np.nansum(d["is_descending_stairs"])),
        "stair_up_frames": int(np.nansum(d["is_ascending_stairs"])),
    }

    # Stair-gate entry/exit poses.
    gate = []
    flags = np.nan_to_num(d["is_descending_stairs"]) > 0
    idx = np.where(flags)[0]
    if idx.size:
        groups, start_i, prev = [], idx[0], idx[0]
        for i in idx[1:]:
            if i - prev > 3:
                groups.append((start_i, prev))
                start_i = i
            prev = i
        groups.append((start_i, prev))
        for a, b in groups:
            gate.append({
                "frames": int(b - a + 1),
                "entry": poses[a].tolist(),
                "exit": poses[b].tolist(),
                "entry_t": float(d["epoch_sec"][a] - d["epoch_sec"][0]),
                "exit_t": float(d["epoch_sec"][b] - d["epoch_sec"][0]),
            })
    result["stair_down_gates"] = gate

    # Near-miss episodes (LiDAR front corridor < 0.40 m).
    low = np.where(np.nan_to_num(d["lidar_min_dist_front"], nan=9.9) < 0.40)[0]
    episodes = []
    if low.size:
        start_i, prev = low[0], low[0]
        groups = []
        for i in low[1:]:
            if i - prev > 2:
                groups.append((start_i, prev))
                start_i = i
            prev = i
        groups.append((start_i, prev))
        for a, b in groups:
            k = a + int(np.argmin(d["lidar_min_dist_front"][a:b + 1]))
            episodes.append({
                "frames": int(b - a + 1),
                "min_lidar_m": float(d["lidar_min_dist_front"][k]),
                "pose": poses[k].tolist(),
                "t": float(d["epoch_sec"][k] - d["epoch_sec"][0]),
            })
    result["near_miss_episodes"] = episodes

    if session["path"] is not None:
        progress, deviation = project_onto_path(poses, session["path"])
        result["progress"] = progress.tolist()
        result["cross_track_m"] = stats(deviation)
        result["milestones_sec"] = {
            f"{th:.2f}": float(np.interp(th, progress, d["epoch_sec"] - d["epoch_sec"][0]))
            for th in (0.2, 0.4, 0.6, 0.8, 0.9)
        }
        # Path endpoint vs goal and the last segment geometry. A large +dz on the
        # last segment is what makes the stair-up gate fire on flat ground.
        goal = (session["meta"].get("goal") or {}).get("pos")
        if goal and len(goal) >= 3:
            result["path_end_minus_goal"] = [
                float(session["path"][-1][i] - goal[i]) for i in range(3)
            ]
        if len(session["path"]) >= 2:
            seg = session["path"][-1] - session["path"][-2]
            horiz = float(np.linalg.norm(seg[:2]))
            result["path_last_segment"] = {
                "dx": float(seg[0]), "dy": float(seg[1]), "dz": float(seg[2]),
                "horizontal_m": horiz,
                "slope": float(seg[2] / horiz) if horiz > 1e-6 else None,
            }
    return result


def paired_metrics(d_a, d_b, pairs):
    out = {"n_pairs": len(pairs)}
    if not pairs:
        return out
    ia = np.array([p[0] for p in pairs])
    ib = np.array([p[1] for p in pairs])
    keys = (
        "depth_latency_ms", "depth_floor_near", "depth_center_min", "depth_right_min",
        "depth_left_min", "cmd_vx", "cmd_wz", "speed_mps", "lidar_min_dist_front",
        "depth_valid_ratio_raw", "depth_hole_ratio",
    )
    for key in keys:
        if key not in d_a or key not in d_b:
            continue
        va = _finite(d_a[key][ia])
        vb = _finite(d_b[key][ib])
        out[key] = {
            "a_mean": float(np.mean(va)) if va.size else None,
            "b_mean": float(np.mean(vb)) if vb.size else None,
            "a_median": float(np.median(va)) if va.size else None,
            "b_median": float(np.median(vb)) if vb.size else None,
        }
    out["pose_error_m"] = stats([p[2] for p in pairs])
    out["yaw_error_deg"] = stats([p[3] for p in pairs])
    return out


# ----------------------------------------------------------------------------
# SVG figures (no matplotlib dependency)
# ----------------------------------------------------------------------------
def _svg_open(width, height):
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="DejaVu Sans, sans-serif">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
    ]


def _polyline(points, color, width=2.0, dash=""):
    pts = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<polyline points="{pts}" fill="none" stroke="{color}" '
            f'stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"{dash_attr}/>')


def _esc(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def _text(x, y, text, size=12, color="#222", anchor="start", weight="normal"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{color}" '
            f'text-anchor="{anchor}" font-weight="{weight}">{_esc(text)}</text>')


def trajectory_svg(run_a, run_b, path_pts, case_a, case_b, out_path):
    width, height = 760, 620
    pad = 46
    xs = [p[0] for p in path_pts] if path_pts is not None else []
    ys = [p[1] for p in path_pts] if path_pts is not None else []
    for run in (run_a, run_b):
        xs += [p[0] for p in run["poses"]]
        ys += [p[1] for p in run["poses"]]
    if not xs:
        return
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    scale = min((width - 2 * pad) / max(x1 - x0, 1e-3), (height - 2 * pad) / max(y1 - y0, 1e-3))

    def tx(p):
        return pad + (p[0] - x0) * scale, height - pad - (p[1] - y0) * scale

    svg = _svg_open(width, height)
    svg.append(_text(pad, 26, "Door / stairs route — trajectory overlay (map frame)", 15, weight="bold"))
    if path_pts is not None:
        svg.append(_polyline([tx(p) for p in path_pts], "#9aa0a6", 2.0, dash="6 5"))
    svg.append(_polyline([tx(p) for p in run_a["poses"]], "#1f6feb", 2.2))
    svg.append(_polyline([tx(p) for p in run_b["poses"]], "#e8710a", 2.2))
    for run, color, label in ((run_a, "#1f6feb", run_a["label"]), (run_b, "#e8710a", run_b["label"])):
        sx, sy = tx(run["poses"][0])
        ex, ey = tx(run["poses"][-1])
        svg.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="4.5" fill="{color}"/>')
        svg.append(f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="4.5" fill="none" stroke="{color}" stroke-width="2"/>')
        svg.append(_text(sx + 7, sy - 6, label, 11, color))
    if case_a is not None:
        cx, cy = tx(case_a)
        svg.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="7" fill="none" stroke="#d93025" stroke-width="2"/>')
        svg.append(_text(cx + 10, cy + 4, "case study", 11, "#d93025"))
    # legend
    lx, ly = width - 250, height - 40
    svg.append(_polyline([(lx, ly), (lx + 26, ly)], "#9aa0a6", 2.0, dash="6 5"))
    svg.append(_text(lx + 32, ly + 4, "global path", 11, "#5f6368"))
    svg.append(_polyline([(lx, ly + 18), (lx + 26, ly + 18)], "#1f6feb", 2.2))
    svg.append(_text(lx + 32, ly + 22, run_a["label"], 11, "#1f6feb"))
    svg.append(_polyline([(lx, ly + 36), (lx + 26, ly + 36)], "#e8710a", 2.2))
    svg.append(_text(lx + 32, ly + 40, run_b["label"], 11, "#e8710a"))
    svg.append("</svg>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


def progress_svg(run_a, run_b, out_path):
    width, height = 820, 640
    panels = [
        ("speed_mps", "Speed (m/s)", (0.0, 0.75)),
        ("depth_latency_ms", "Depth age at logger (ms)", (0.0, 1000.0)),
        ("depth_floor_near", "Near floor distance, bottom-centre ROI (m)", (0.4, 1.6)),
    ]
    svg = _svg_open(width, height)
    svg.append(_text(20, 26, "Progress-aligned profiles (progress = arc length along global path)", 15, weight="bold"))
    top, panel_h, gap = 50, 160, 46
    left, right = 78, width - 30
    for k, (key, title, (vmin, vmax)) in enumerate(panels):
        y0 = top + k * (panel_h + gap)
        y1 = y0 + panel_h
        svg.append(f'<rect x="{left}" y="{y0}" width="{right-left}" height="{panel_h}" fill="#fafafa" stroke="#dadce0"/>')
        svg.append(_text(left, y0 - 8, title, 12, "#3c4043", weight="bold"))
        # axes
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = left + frac * (right - left)
            svg.append(f'<line x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y1}" stroke="#eceff1"/>')
            svg.append(_text(x, y1 + 16, f"{frac:.2f}", 10, "#5f6368", anchor="middle"))
        for frac in (0.0, 0.5, 1.0):
            y = y1 - frac * panel_h
            svg.append(f'<line x1="{left}" y1="{y:.1f}" x2="{right}" y2="{y:.1f}" stroke="#eceff1"/>')
            value = vmin + frac * (vmax - vmin)
            svg.append(_text(left - 8, y + 4, f"{value:.2f}", 10, "#5f6368", anchor="end"))
        for run, color in ((run_a, "#1f6feb"), (run_b, "#e8710a")):
            progress = np.array(run["progress"])
            values = np.array(run["data"][key], dtype=float)
            if values.size != progress.size:
                continue
            bins = np.linspace(0, 1, 41)
            pts = []
            for i in range(len(bins) - 1):
                mask = (progress >= bins[i]) & (progress < bins[i + 1])
                if mask.sum() == 0:
                    continue
                v = float(np.nanmean(values[mask]))
                if not math.isfinite(v):
                    continue
                v = min(max(v, vmin), vmax)
                x = left + (bins[i] + bins[i + 1]) / 2 * (right - left)
                y = y1 - (v - vmin) / (vmax - vmin) * panel_h
                pts.append((x, y))
            svg.append(_polyline(pts, color, 2.0))
    svg.append(_text(left, height - 12, "blue = RealSense/A, orange = ZED/B", 11, "#5f6368"))
    svg.append("</svg>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


def _depth_color(value):
    if value is None or not math.isfinite(value):
        return "#ffffff"
    if value <= 0:
        return "#c8d6ff"  # open / beyond max range
    t = min(max(value / 4.0, 0.0), 1.0)
    # red (near) -> yellow -> green -> blue (far)
    stops = [(0.0, (165, 0, 38)), (0.15, (215, 48, 39)), (0.35, (253, 174, 97)),
             (0.55, (255, 255, 191)), (0.75, (171, 221, 164)), (1.0, (49, 54, 149))]
    for i in range(len(stops) - 1):
        a, ca = stops[i]
        b, cb = stops[i + 1]
        if a <= t <= b:
            f = (t - a) / (b - a)
            rgb = tuple(int(ca[j] + (cb[j] - ca[j]) * f) for j in range(3))
            return "#%02x%02x%02x" % rgb
    return "#313695"


def depth_case_svg(case_a, case_b, label_a, label_b, out_path):
    if case_a is None or case_b is None:
        return
    cell = 9
    rows, cols = case_a["saved"].shape
    width = cols * cell * 2 + 180
    height = rows * cell + 110
    svg = _svg_open(width, height)
    svg.append(_text(20, 26, "Policy input (64×40 depth) at the closest-clearance case", 14, weight="bold"))
    for k, (case, label) in enumerate(((case_a, label_a), (case_b, label_b))):
        ox = 20 + k * (cols * cell + 70)
        oy = 44
        svg.append(_text(ox, oy - 6, f"{label}  frame {case['frame']}", 12, weight="bold"))
        saved = case["saved"]
        for r in range(rows):
            for c in range(cols):
                svg.append(f'<rect x="{ox + c*cell}" y="{oy + r*cell}" width="{cell}" height="{cell}" '
                           f'fill="{_depth_color(float(saved[r, c]))}" stroke="none"/>')
        svg.append(f'<rect x="{ox}" y="{oy}" width="{cols*cell}" height="{rows*cell}" fill="none" stroke="#5f6368"/>')
        svg.append(_text(ox, oy + rows * cell + 18,
                         f"centre min {case['center_perceived_min']:.2f} m | right min {case['right_perceived_min']:.2f} m", 11))
        if "center_real_min" in case:
            svg.append(_text(ox, oy + rows * cell + 34,
                             f"real (LiDAR-scale) centre {case['center_real_min']:.2f} m | right {case['right_real_min']:.2f} m", 11, "#5f6368"))
    svg.append(_text(20, height - 16, "colour: dark red < 0.4 m, orange 0.8 m, yellow 1.5 m, green 2.5 m, blue > 3 m, pale blue = no return", 10, "#5f6368"))
    svg.append("</svg>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


# ----------------------------------------------------------------------------
# Report
# ----------------------------------------------------------------------------
def _fmt(value, digits=2, suffix=""):
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(value):
        return "—"
    return f"{value:.{digits}f}{suffix}"


def markdown_report(run_a, run_b, paired, pairs, fov_a, fov_b, case_a, case_b):
    la, lb = run_a["label"], run_b["label"]
    lines = []
    lines.append(f"# {la} vs {lb} — door / stairs camera A/B (auto-generated)\n")
    lines.append("Trimmed at the last non-zero `/cmd_vel` sample; manual teleop return excluded.\n")
    lines.append("## Run-level\n")
    lines.append(f"| metric | {la} | {lb} |")
    lines.append("|---|---:|---:|")
    rows = [
        ("camera_id", run_a["camera_id"], run_b["camera_id"]),
        ("depth topic", run_a["depth_topic"], run_b["depth_topic"]),
        ("depth encoding", run_a["depth_encoding"], run_b["depth_encoding"]),
        ("frames", run_a["frames"], run_b["frames"]),
        ("duration (s)", _fmt(run_a["duration_sec"], 1), _fmt(run_b["duration_sec"], 1)),
        ("distance (m)", _fmt(run_a["distance_m"], 1), _fmt(run_b["distance_m"], 1)),
        ("mean speed (m/s)", _fmt(run_a["mean_speed_mps"]), _fmt(run_b["mean_speed_mps"])),
        ("mean cmd vx", _fmt(run_a["mean_cmd_vx"]), _fmt(run_b["mean_cmd_vx"])),
        ("cross-track mean (m)", _fmt(run_a["cross_track_m"]["mean"]), _fmt(run_b["cross_track_m"]["mean"])),
        ("cross-track p90 (m)", _fmt(run_a["cross_track_m"]["p90"]), _fmt(run_b["cross_track_m"]["p90"])),
        ("cross-track max (m)", _fmt(run_a["cross_track_m"]["max"]), _fmt(run_b["cross_track_m"]["max"])),
        ("depth valid raw", _fmt(run_a["depth_health"]["valid_ratio_raw"]["mean"], 3), _fmt(run_b["depth_health"]["valid_ratio_raw"]["mean"], 3)),
        ("depth hole raw", _fmt(run_a["depth_health"]["hole_ratio_raw"]["mean"], 3), _fmt(run_b["depth_health"]["hole_ratio_raw"]["mean"], 3)),
        ("depth age median (ms)", _fmt(run_a["depth_health"]["latency_ms"]["median"], 0), _fmt(run_b["depth_health"]["latency_ms"]["median"], 0)),
        ("depth age p90 (ms)", _fmt(run_a["depth_health"]["latency_ms"]["p90"], 0), _fmt(run_b["depth_health"]["latency_ms"]["p90"], 0)),
        ("floor_near mean (m)", _fmt(run_a["depth_roi"]["floor_near"]["mean"]), _fmt(run_b["depth_roi"]["floor_near"]["mean"])),
        ("centre_min mean (m)", _fmt(run_a["depth_roi"]["center_min"]["mean"]), _fmt(run_b["depth_roi"]["center_min"]["mean"])),
        ("right_min mean (m)", _fmt(run_a["depth_roi"]["right_min"]["mean"]), _fmt(run_b["depth_roi"]["right_min"]["mean"])),
        ("stair-down frames", run_a["stair_down_frames"], run_b["stair_down_frames"]),
        ("stair-up frames", run_a["stair_up_frames"], run_b["stair_up_frames"]),
    ]
    for name, va, vb in rows:
        lines.append(f"| {name} | {va} | {vb} |")
    lines.append("")
    lines.append("## Path / goal consistency\n")
    lines.append(f"| metric | {la} | {lb} |")
    lines.append("|---|---:|---:|")
    for name, key in (("path end − goal (x, y, z)", "path_end_minus_goal"),
                      ("path last segment", "path_last_segment")):
        vals = []
        for run in (run_a, run_b):
            value = run.get(key)
            if key == "path_end_minus_goal":
                vals.append("(" + ", ".join(f"{v:+.2f}" for v in value) + ")" if value else "—")
            else:
                vals.append(f"dz {value['dz']:+.2f} m / horiz {value['horizontal_m']:.2f} m "
                            f"(slope {value['slope']:+.2f})" if value else "—")
        lines.append(f"| {name} | {vals[0]} | {vals[1]} |")
    lines.append("")
    lines.append("## Field of view (from CameraInfo)\n")
    lines.append(f"| camera | HFOV | VFOV | deg/col @64 | deg/row @40 |")
    lines.append("|---|---:|---:|---:|---:|")
    for label, fov in ((la, fov_a), (lb, fov_b)):
        if fov:
            lines.append(f"| {label} | {fov['h_deg']:.1f}° | {fov['v_deg']:.1f}° | {fov['h_deg']/64:.2f} | {fov['v_deg']/40:.2f} |")
    lines.append("| sim baseline | 105.8° | 79.1° | 1.65 | 1.98 |")
    lines.append("")
    if pairs:
        lines.append(f"## Matched-state comparison ({len(pairs)} pose-matched pairs)\n")
        lines.append(f"| metric | {la} | {lb} | Δ (B−A) |")
        lines.append("|---|---:|---:|---:|")
        names = {
            "depth_latency_ms": "depth age (ms)",
            "depth_floor_near": "floor_near (m)",
            "depth_center_min": "centre_min (m)",
            "depth_right_min": "right_min (m)",
            "depth_left_min": "left_min (m)",
            "cmd_vx": "cmd vx",
            "cmd_wz": "cmd wz",
            "speed_mps": "speed (m/s)",
            "lidar_min_dist_front": "LiDAR front min (m)",
        }
        for key, name in names.items():
            if key not in paired:
                continue
            a = paired[key]["a_mean"]
            b = paired[key]["b_mean"]
            delta = (b - a) if (a is not None and b is not None) else None
            lines.append(f"| {name} | {_fmt(a)} | {_fmt(b)} | {_fmt(delta)} |")
        lines.append("")
        lines.append(f"Pose-match error: mean {_fmt(paired['pose_error_m']['mean'])} m, "
                     f"max {_fmt(paired['pose_error_m']['max'])} m; yaw mean {_fmt(paired['yaw_error_deg']['mean'])}°.\n")
    if case_a and case_b:
        lines.append("## Case study (selected matched pose)\n")
        lines.append(f"| | {la} | {lb} |")
        lines.append("|---|---:|---:|")
        for key, name in (
            ("frame", "frame"),
            ("center_perceived_min", "policy-input centre min (m)"),
            ("center_real_min", "real centre min (m)"),
            ("right_perceived_min", "policy-input right min (m)"),
            ("right_real_min", "real right min (m)"),
            ("invalid_fraction", "invalid fraction in 64×40 cells"),
        ):
            va = case_a.get(key)
            vb = case_b.get(key)
            lines.append(f"| {name} | {_fmt(va, 0 if key == 'frame' else 2)} | {_fmt(vb, 0 if key == 'frame' else 2)} |")
        lines.append("")
    lines.append("## Near-miss episodes (LiDAR front < 0.40 m)\n")
    for run in (run_a, run_b):
        lines.append(f"### {run['label']}\n")
        if not run["near_miss_episodes"]:
            lines.append("none\n")
            continue
        lines.append("| t (s) | frames | min LiDAR (m) | pose (x, y, z) |")
        lines.append("|---:|---:|---:|---|")
        for ep in run["near_miss_episodes"]:
            x, y, z = ep["pose"]
            lines.append(f"| {ep['t']:.1f} | {ep['frames']} | {ep['min_lidar_m']:.2f} | ({x:.2f}, {y:.2f}, {z:.2f}) |")
        lines.append("")
    return "\n".join(lines)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--a", required=True, help="Session A directory (e.g. RealSense)")
    parser.add_argument("--b", required=True, help="Session B directory (e.g. ZED)")
    parser.add_argument("--label-a", default="A")
    parser.add_argument("--label-b", default="B")
    parser.add_argument("--out", default="", help="Output directory (default: <parent>/analysis)")
    parser.add_argument("--match-pos", type=float, default=0.25, help="Matched-pair position tolerance (m)")
    parser.add_argument("--match-yaw", type=float, default=8.0, help="Matched-pair yaw tolerance (deg)")
    parser.add_argument("--case-frame-a", type=int, default=None, help="Explicit case frame for A")
    parser.add_argument("--case-frame-b", type=int, default=None, help="Explicit case frame for B")
    parser.add_argument("--case-region", default="",
                        help="Restrict the auto case study to a box 'xmin,xmax,ymin,ymax,zmin,zmax' "
                             "(map frame); default = global minimum LiDAR clearance")
    args = parser.parse_args()

    sess_a = load_session(args.a)
    sess_b = load_session(args.b)
    a0, a1 = trim_indices(sess_a)
    b0, b1 = trim_indices(sess_b)
    print(f"[{args.label_a}] mission rows {a0}..{a1} ({a1-a0+1} frames)")
    print(f"[{args.label_b}] mission rows {b0}..{b1} ({b1-b0+1} frames)")

    out_dir = os.path.expanduser(args.out) if args.out else os.path.join(
        os.path.dirname(os.path.abspath(sess_a["dir"])), "analysis")
    os.makedirs(out_dir, exist_ok=True)
    fig_dir = os.path.join(out_dir, "figures")
    os.makedirs(fig_dir, exist_ok=True)

    write_trimmed_csv(sess_a, a0, a1, os.path.join(out_dir, f"trimmed_{args.label_a}.csv"))
    write_trimmed_csv(sess_b, b0, b1, os.path.join(out_dir, f"trimmed_{args.label_b}.csv"))

    run_a = run_metrics(args.label_a, sess_a, a0, a1)
    run_b = run_metrics(args.label_b, sess_b, b0, b1)

    d_a = slice_session(sess_a, a0, a1)
    d_b = slice_session(sess_b, b0, b1)
    pairs = matched_pairs(d_a, d_b, args.match_pos, args.match_yaw)
    paired = paired_metrics(d_a, d_b, pairs)
    print(f"matched pairs: {len(pairs)}")

    fov_a = fov_deg(sess_a["meta"])
    fov_b = fov_deg(sess_b["meta"])

    # Case study: the matched pair with the smallest LiDAR clearance (door / extinguisher).
    case_a = case_b = None
    case_i = case_j = None
    if args.case_frame_a is not None and args.case_frame_b is not None:
        case_i = a0 + int(np.argmin(np.abs(d_a["frame_idx"] - args.case_frame_a)))
        case_j = b0 + int(np.argmin(np.abs(d_b["frame_idx"] - args.case_frame_b)))
    elif pairs:
        candidates = pairs
        if args.case_region:
            box = [float(v) for v in args.case_region.split(",")]
            if len(box) != 6:
                raise ValueError("--case-region needs 6 comma-separated values")
            xmin, xmax, ymin, ymax, zmin, zmax = box

            def _inside(p):
                return (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax and zmin <= p[2] <= zmax)

            poses_a = pose_array(d_a)
            filtered = [p for p in pairs if _inside(poses_a[p[0]])]
            if filtered:
                candidates = filtered
        best = min(candidates, key=lambda p: min(np.nan_to_num(d_a["lidar_min_dist_front"][p[0]], nan=9.9),
                                                 np.nan_to_num(d_b["lidar_min_dist_front"][p[1]], nan=9.9)))
        case_i, case_j = a0 + best[0], b0 + best[1]
    if case_i is not None:
        case_a = depth_case(sess_a, case_i, sess_a["meta"].get("min_depth", 0.15), sess_a["meta"].get("max_depth", 10.0))
        case_b = depth_case(sess_b, case_j, sess_b["meta"].get("min_depth", 0.25), sess_b["meta"].get("max_depth", 10.0))

    run_a["poses"] = pose_array(d_a)
    run_b["poses"] = pose_array(d_b)
    run_a["data"] = d_a
    run_b["data"] = d_b

    path_pts = sess_a["path"] if sess_a["path"] is not None else sess_b["path"]
    trajectory_svg(run_a, run_b, path_pts,
                   run_a["poses"][case_i - a0] if case_i is not None else None,
                   run_b["poses"][case_j - b0] if case_j is not None else None,
                   os.path.join(fig_dir, "trajectory.svg"))
    progress_svg(run_a, run_b, os.path.join(fig_dir, "progress_profiles.svg"))
    depth_case_svg(case_a, case_b, args.label_a, args.label_b,
                   os.path.join(fig_dir, "depth_case.svg"))

    # JSON output (strip the big arrays for readability)
    serialisable_a = {k: v for k, v in run_a.items() if k not in ("poses", "data")}
    serialisable_b = {k: v for k, v in run_b.items() if k not in ("poses", "data")}
    payload = {
        "session_a": serialisable_a,
        "session_b": serialisable_b,
        "paired": paired,
        "fov_a": fov_a,
        "fov_b": fov_b,
        "case_a": {k: v for k, v in (case_a or {}).items() if k not in ("saved", "bearing", "elevation", "real_min", "invalid")},
        "case_b": {k: v for k, v in (case_b or {}).items() if k not in ("saved", "bearing", "elevation", "real_min", "invalid")},
    }
    with open(os.path.join(out_dir, "run_metrics.json"), "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    with open(os.path.join(out_dir, "matched_pairs.csv"), "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["row_a", "row_b", "pos_err_m", "yaw_err_deg"])
        for i, j, dist, dyaw in pairs:
            writer.writerow([i + a0, j + b0, f"{dist:.4f}", f"{dyaw:.2f}"])

    with open(os.path.join(out_dir, "report_auto.md"), "w", encoding="utf-8") as f:
        f.write(markdown_report(run_a, run_b, paired, pairs, fov_a, fov_b, case_a, case_b))

    print(f"wrote {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

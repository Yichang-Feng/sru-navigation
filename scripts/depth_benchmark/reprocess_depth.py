#!/usr/bin/env python3
"""Preprocessing ablation V0-V5 for door_diagnostic_logger sessions.

Rebuilds the policy's 64x40 depth input from the recorded raw depth PNGs under
several preprocessing variants, and quantifies what each variant does to:

* fill-induced "false near" obstacles (invalid pixels filled with min_depth),
* near-floor visibility (FOV / crop / letterbox),
* cross-camera divergence at pose-matched frames.

Variants
--------
V0_fill_min      current policy preprocessing: invalid -> min_depth, direct resize
V1_fill_zero     invalid -> 0.0 (open), direct resize
V2_nearest_valid invalid -> nearest valid depth (hole fill), direct resize
V3_center_crop   fill min, centre-crop to 64:40 aspect (1.6), resize
V4_letterbox     fill min, resize to 64x36 preserving aspect, pad to 64x40 with 0
V5_virtual_cam   fill min, remap to the sim virtual camera FOV (105.8x79.1, 64x40)

Usage
-----
python3 scripts/depth_benchmark/reprocess_depth.py \
  --rs ~/sru-navigation-learning/door_diagnostics/session_..._realsense \
  --zed ~/sru-navigation-learning/door_diagnostics/session_..._zed \
  --pairs ~/sru-navigation-learning/door_diagnostics/analysis_20261110/matched_pairs.csv \
  --out ~/sru-navigation-learning/door_diagnostics/analysis_20261110/ablation \
  --case-frame-rs 304 --case-frame-zed 118
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
except ImportError:  # pragma: no cover
    cv2 = None

# Sim virtual camera: 192x120, fx=fy=72.7025, cx=94.4457, cy=62.5424; the policy
# grid is the 3x downsample, so the 64x40 intrinsics are these / 3.
SIM_FX = 72.7025 / 3.0
SIM_FY = 72.7025 / 3.0
SIM_CX = 94.4457 / 3.0
SIM_CY = 62.5424 / 3.0

ROWS, COLS = 40, 64
VARIANTS = (
    "V0_fill_min",
    "V1_fill_zero",
    "V2_nearest_valid",
    "V3_center_crop",
    "V4_letterbox",
    "V5_virtual_cam",
)


# ----------------------------------------------------------------------------
# Session loading
# ----------------------------------------------------------------------------
def _to_float(value):
    text = str(value).strip() if value is not None else ""
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
    with open(os.path.join(path, "trajectory.csv"), newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        raw_rows = list(reader)
        columns = reader.fieldnames or []
    data = {c: np.array([_to_float(r.get(c)) for r in raw_rows], dtype=float) for c in columns}
    meta = {}
    for name in ("session_summary.json", "session_meta.json", "camera_info.json"):
        candidate = os.path.join(path, name)
        if os.path.isfile(candidate):
            with open(candidate, encoding="utf-8") as f:
                meta.update(json.load(f))
    return {"dir": path, "rows": raw_rows, "data": data, "meta": meta}


def load_raw_depth(session, frame_idx):
    path = os.path.join(session["dir"], "depth_raw", f"frame_{int(frame_idx):06d}_raw_depth.png")
    if not os.path.isfile(path):
        return None
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    return img.astype(np.float32) / 1000.0


def load_saved_grid(session, frame_idx):
    path = os.path.join(session["dir"], "depth", f"frame_{int(frame_idx):06d}_depth_40x64.npy")
    if not os.path.isfile(path):
        return None
    return np.load(path)


def scaled_intrinsics(meta, width, height):
    info = meta
    fx = float(info["fx"]) * width / float(info["width"])
    fy = float(info["fy"]) * height / float(info["height"])
    cx = float(info["cx"]) * width / float(info["width"])
    cy = float(info["cy"]) * height / float(info["height"])
    return fx, fy, cx, cy


# ----------------------------------------------------------------------------
# Preprocessing variants
# ----------------------------------------------------------------------------
def depth_masks(depth, min_depth, max_depth):
    close = (depth <= 0.0) | ((depth < min_depth) & (depth > 0.0))
    far = depth > max_depth
    valid = (~close) & (~far)
    return close, far, valid


def fill_depth(depth, close, far, valid, mode, min_depth):
    out = depth.copy()
    out[far] = 0.0
    if mode == "min":
        out[close] = min_depth
    elif mode == "zero":
        out[close] = 0.0
    elif mode == "nearest":
        # Nearest *valid* depth for every invalid pixel (close or far).
        invalid = (~valid).astype(np.uint8)
        _, labels = cv2.distanceTransformWithLabels(
            invalid, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL
        )
        lut = np.zeros(int(labels.max()) + 1, dtype=np.float32)
        lut[labels[valid]] = depth[valid]
        nearest = lut[labels]
        out[close] = nearest[close]
        out[far] = 0.0
    else:
        raise ValueError(mode)
    return out


def grid_angles(fx, fy, cx, cy, rows=ROWS, cols=COLS):
    u = (np.arange(cols) + 0.5 - cx) / fx
    v = (np.arange(rows) + 0.5 - cy) / fy
    bearing = np.degrees(np.arctan(u))[None, :].repeat(rows, 0)
    elevation = np.degrees(np.arctan(v))[:, None].repeat(cols, 1)
    return bearing, elevation


def build_variants(depth, min_depth, max_depth, camera_info):
    height, width = depth.shape
    close, far, valid = depth_masks(depth, min_depth, max_depth)
    fx_a, fy_a, cx_a, cy_a = scaled_intrinsics(camera_info, width, height)
    filled_min = fill_depth(depth, close, far, valid, "min", min_depth)
    filled_zero = fill_depth(depth, close, far, valid, "zero", min_depth)
    filled_nearest = fill_depth(depth, close, far, valid, "nearest", min_depth)

    out = {}

    # V0/V1/V2: direct resize.
    for name, filled in (
        ("V0_fill_min", filled_min),
        ("V1_fill_zero", filled_zero),
        ("V2_nearest_valid", filled_nearest),
    ):
        grid = cv2.resize(filled, (COLS, ROWS), interpolation=cv2.INTER_LINEAR)
        bearing, elevation = grid_angles(
            fx_a * COLS / width, fy_a * ROWS / height,
            cx_a * COLS / width, cy_a * ROWS / height,
        )
        out[name] = {"grid": grid, "bearing": bearing, "elevation": elevation, "coverage": 1.0}

    # V3: centre-crop to 1.6 aspect, then resize.
    target_w = int(round(height * (COLS / ROWS)))
    target_w = min(target_w, width)
    crop_left = (width - target_w) // 2
    cropped = filled_min[:, crop_left:crop_left + target_w]
    grid3 = cv2.resize(cropped, (COLS, ROWS), interpolation=cv2.INTER_LINEAR)
    bearing3, elevation3 = grid_angles(
        fx_a * COLS / target_w, fy_a * ROWS / height,
        (cx_a - crop_left) * COLS / target_w, cy_a * ROWS / height,
    )
    out["V3_center_crop"] = {"grid": grid3, "bearing": bearing3, "elevation": elevation3, "coverage": 1.0}

    # V4: letterbox: 64 x 36 image, pad 2 rows top/bottom with 0.
    pad = (ROWS - int(round(COLS / (width / height)))) // 2
    img_h = ROWS - 2 * pad
    img4 = cv2.resize(filled_min, (COLS, img_h), interpolation=cv2.INTER_LINEAR)
    grid4 = np.zeros((ROWS, COLS), dtype=np.float32)
    grid4[pad:pad + img_h, :] = img4
    bearing4, elevation4 = grid_angles(
        fx_a * COLS / width, fy_a * img_h / height,
        cx_a * COLS / width, cy_a * img_h / height + pad,
    )
    out["V4_letterbox"] = {"grid": grid4, "bearing": bearing4, "elevation": elevation4, "coverage": 1.0}

    # V5: rotation-only remap to the sim virtual camera FOV.
    u = np.arange(COLS)[None, :].repeat(ROWS, 0) + 0.5
    v = np.arange(ROWS)[:, None].repeat(COLS, 1) + 0.5
    map_x = (fx_a * (u - SIM_CX) / SIM_FX + cx_a).astype(np.float32)
    map_y = (fy_a * (v - SIM_CY) / SIM_FY + cy_a).astype(np.float32)
    grid5 = cv2.remap(filled_min, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderValue=0.0)
    coverage5 = cv2.remap(
        valid.astype(np.float32), map_x, map_y, interpolation=cv2.INTER_LINEAR, borderValue=0.0
    )
    bearing5, elevation5 = grid_angles(SIM_FX, SIM_FY, SIM_CX, SIM_CY)
    out["V5_virtual_cam"] = {
        "grid": grid5, "bearing": bearing5, "elevation": elevation5, "coverage": coverage5
    }
    return out, valid


def real_cell_min(depth, valid, rows=ROWS, cols=COLS):
    height, width = depth.shape
    ys = np.linspace(0, height, rows + 1).astype(int)
    xs = np.linspace(0, width, cols + 1).astype(int)
    real = np.full((rows, cols), np.inf, dtype=np.float32)
    for r in range(rows):
        for c in range(cols):
            cell = depth[ys[r]:ys[r + 1], xs[c]:xs[c + 1]]
            cell_valid = valid[ys[r]:ys[r + 1], xs[c]:xs[c + 1]]
            if cell_valid.any():
                real[r, c] = float(cell[cell_valid].min())
    return real


# ----------------------------------------------------------------------------
# Metrics
# ----------------------------------------------------------------------------
def sector_stats(grid, bearing, elevation, coverage=None):
    center = (np.abs(bearing) < 20) & (np.abs(elevation) < 20)
    right = (bearing > 20) & (bearing < 40) & (np.abs(elevation) < 20)
    floor = np.zeros_like(grid, dtype=bool)
    floor[32:40, 20:44] = True
    covered = np.ones_like(grid, dtype=bool)
    if coverage is not None and not np.isscalar(coverage):
        covered = coverage > 0.5
    data = covered & (grid > 0.0)  # 0 means "open / beyond max range", not near
    denom = float(covered.sum()) if covered.any() else 0.0

    def _min(mask):
        vals = grid[mask & data]
        return float(np.min(vals)) if vals.size else np.nan

    def _mean(mask):
        vals = grid[mask & data]
        return float(np.mean(vals)) if vals.size else np.nan

    out = {
        "valid_frac": float(data.sum()) / float(grid.size),
        "covered_frac": float(denom) / float(grid.size) if grid.size else np.nan,
        "near08": float(np.count_nonzero((grid < 0.8) & data)) / denom if denom else np.nan,
        "near15": float(np.count_nonzero((grid < 1.5) & data)) / denom if denom else np.nan,
        "center_min": _min(center),
        "right_min": _min(right),
        "floor_near": _mean(floor),
    }
    if coverage is not None and not np.isscalar(coverage):
        out["coverage"] = float(np.mean(covered))
    return out


def false_near_fraction(grid, real, bearing, elevation):
    """Cells that look near (<0.8 m) while the real valid depth is >1.5 m."""
    band = (np.abs(elevation) < 25) & (np.abs(bearing) < 40)
    mask = band & np.isfinite(real)
    if not mask.any():
        return np.nan
    return float(np.mean((grid[mask] > 0.0) & (grid[mask] < 0.8) & (real[mask] > 1.5)))


# ----------------------------------------------------------------------------
# SVG helpers
# ----------------------------------------------------------------------------
def _esc(text):
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _text(x, y, text, size=12, color="#222", anchor="start", weight="normal"):
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{color}" '
            f'text-anchor="{anchor}" font-weight="{weight}">{_esc(text)}</text>')


def depth_color(value):
    if value is None or not math.isfinite(value):
        return "#ffffff"
    if value <= 0:
        return "#c8d6ff"
    t = min(max(value / 4.0, 0.0), 1.0)
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


def variants_svg(rows_data, out_path):
    """rows_data: list of (label, frame, {variant: grid})."""
    cell = 6
    n_var = len(VARIANTS)
    width = n_var * (COLS * cell + 14) + 30
    height = len(rows_data) * (ROWS * cell + 42) + 70
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="DejaVu Sans, sans-serif">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        _text(20, 24, "Preprocessing ablation at the door / extinguisher frame (64×40 policy input)", 14, weight="bold"),
    ]
    for r, (label, frame, grids) in enumerate(rows_data):
        y0 = 42 + r * (ROWS * cell + 42)
        svg.append(_text(20, y0 + 12, f"{label}  frame {frame}", 12, weight="bold"))
        for c, name in enumerate(VARIANTS):
            x0 = 20 + c * (COLS * cell + 14)
            yy = y0 + 20
            svg.append(_text(x0, yy + 10, name, 10, "#3c4043"))
            grid = grids[name]
            for rr in range(ROWS):
                for cc in range(COLS):
                    svg.append(
                        f'<rect x="{x0 + cc*cell}" y="{yy + 16 + rr*cell}" width="{cell}" height="{cell}" '
                        f'fill="{depth_color(float(grid[rr, cc]))}" stroke="none"/>'
                    )
            svg.append(f'<rect x="{x0}" y="{yy + 16}" width="{COLS*cell}" height="{ROWS*cell}" '
                       f'fill="none" stroke="#5f6368"/>')
    svg.append(_text(20, height - 18,
                     "colour: dark red <0.4 m, orange 0.8 m, yellow 1.5 m, green 2.5 m, blue >3 m, pale blue = 0/open, white = no data",
                     10, "#5f6368"))
    svg.append("</svg>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


def bars_svg(summary, out_path):
    """Two-panel bar chart: near08 fraction and floor_near per variant."""
    cameras = list(summary.keys())
    variants = list(VARIANTS)
    width, height = 900, 520
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}" font-family="DejaVu Sans, sans-serif">',
        f'<rect width="{width}" height="{height}" fill="#ffffff"/>',
        _text(20, 24, "Ablation summary (mean over matched frames)", 14, weight="bold"),
    ]
    panels = [
        ("near08", "Fraction of covered grid < 0.8 m", 0.0, 0.7),
        ("floor_near", "Near-floor distance (m)", 0.0, 1.4),
    ]
    colors = {"RealSense": "#1f6feb", "ZED": "#e8710a"}
    for p, (key, title, vmin, vmax) in enumerate(panels):
        x0, y0, w, h = 70, 60 + p * 230, 780, 170
        svg.append(_text(x0, y0 - 10, title, 12, weight="bold"))
        svg.append(f'<rect x="{x0}" y="{y0}" width="{w}" height="{h}" fill="#fafafa" stroke="#dadce0"/>')
        group_w = w / len(variants)
        bar_w = group_w / (len(cameras) + 1)
        for i in range(len(variants) + 1):
            x = x0 + i * group_w
            svg.append(f'<line x1="{x:.1f}" y1="{y0}" x2="{x:.1f}" y2="{y0+h}" stroke="#eceff1"/>')
        for i, var in enumerate(variants):
            svg.append(_text(x0 + (i + 0.5) * group_w, y0 + h + 16, var.replace("_", " "), 9, "#5f6368", anchor="middle"))
            for j, cam in enumerate(cameras):
                value = summary[cam].get(var, {}).get(key)
                if value is None or not math.isfinite(value):
                    continue
                value = min(max(value, vmin), vmax)
                bh = (value - vmin) / (vmax - vmin) * h
                bx = x0 + i * group_w + (j + 1) * bar_w
                by = y0 + h - bh
                svg.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bar_w*0.8:.1f}" height="{bh:.1f}" '
                           f'fill="{colors.get(cam, "#888")}"/>')
        # y labels
        for frac in (0.0, 0.5, 1.0):
            y = y0 + h - frac * h
            svg.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0+w}" y2="{y:.1f}" stroke="#eceff1"/>')
            svg.append(_text(x0 - 8, y + 4, f"{vmin + frac*(vmax-vmin):.2f}", 10, "#5f6368", anchor="end"))
    lx, ly = 70, height - 40
    for j, cam in enumerate(cameras):
        svg.append(f'<rect x="{lx + j*120}" y="{ly-10}" width="14" height="14" fill="{colors.get(cam, "#888")}"/>')
        svg.append(_text(lx + j*120 + 20, ly + 2, cam, 11))
    svg.append("</svg>")
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(svg))


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rs", required=True, help="RealSense session directory")
    parser.add_argument("--zed", required=True, help="ZED session directory")
    parser.add_argument("--pairs", default="", help="matched_pairs.csv from analyze_camera_sessions.py")
    parser.add_argument("--out", default="", help="output directory")
    parser.add_argument("--stride", type=int, default=1, help="use every Nth matched pair")
    parser.add_argument("--case-frame-rs", type=int, default=304)
    parser.add_argument("--case-frame-zed", type=int, default=118)
    args = parser.parse_args()

    if cv2 is None:
        print("OpenCV (cv2) is required for the ablation.", file=sys.stderr)
        return 2

    rs = load_session(args.rs)
    zed = load_session(args.zed)
    out_dir = os.path.expanduser(args.out) if args.out else os.path.join(
        os.path.dirname(os.path.abspath(rs["dir"])), "analysis", "ablation")
    os.makedirs(out_dir, exist_ok=True)

    # Pairs: absolute CSV row indices (0-based).
    pairs = []
    if args.pairs and os.path.isfile(os.path.expanduser(args.pairs)):
        with open(os.path.expanduser(args.pairs), newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                pairs.append((int(row["row_a"]), int(row["row_b"])))
    else:
        # Fallback: match by 3-D pose.
        d_a, d_b = rs["data"], zed["data"]
        for i in range(len(d_a["robot_x"])):
            dx = d_b["robot_x"] - d_a["robot_x"][i]
            dy = d_b["robot_y"] - d_a["robot_y"][i]
            dz = d_b["robot_z"] - d_a["robot_z"][i]
            dyaw = np.abs((d_b["robot_yaw_deg"] - d_a["robot_yaw_deg"][i] + 180) % 360 - 180)
            dist = np.sqrt(dx*dx + dy*dy + dz*dz)
            j = int(np.argmin(dist + dyaw * 0.01))
            if dist[j] < 0.25 and dyaw[j] < 8:
                pairs.append((i, j))
    pairs = pairs[::max(1, args.stride)]
    print(f"pairs: {len(pairs)}")

    records = []
    for n, (ia, ib) in enumerate(pairs):
        for cam, session, idx in (("RealSense", rs, ia), ("ZED", zed, ib)):
            d = session["data"]
            frame = int(d["frame_idx"][idx])
            depth = load_raw_depth(session, frame)
            if depth is None:
                continue
            min_depth = float(session["meta"].get("min_depth", 0.15))
            max_depth = float(session["meta"].get("max_depth", 10.0))
            variants, valid = build_variants(depth, min_depth, max_depth, session["meta"])
            real = real_cell_min(depth, valid)
            for name in VARIANTS:
                v = variants[name]
                stats = sector_stats(v["grid"], v["bearing"], v["elevation"], v.get("coverage"))
                if name in ("V0_fill_min", "V1_fill_zero", "V2_nearest_valid"):
                    stats["false_near"] = false_near_fraction(
                        v["grid"], real, v["bearing"], v["elevation"])
                    # Fraction of cells whose *real* valid minimum is < 0.8 m.
                    stats["real_near08"] = float(np.mean(real < 0.8))
                else:
                    stats["false_near"] = np.nan
                    stats["real_near08"] = np.nan
                records.append({"pair": n, "row": idx, "frame": frame, "camera": cam,
                                "variant": name, **stats})
        if (n + 1) % 25 == 0:
            print(f"  processed {n + 1}/{len(pairs)} pairs")

    # Case frames.
    case_grids = {}
    v0_validation = {}
    for cam, session, frame_wanted in (("RealSense", rs, args.case_frame_rs), ("ZED", zed, args.case_frame_zed)):
        d = session["data"]
        idx = int(np.argmin(np.abs(d["frame_idx"] - frame_wanted)))
        frame = int(d["frame_idx"][idx])
        depth = load_raw_depth(session, frame)
        if depth is None:
            continue
        min_depth = float(session["meta"].get("min_depth", 0.15))
        max_depth = float(session["meta"].get("max_depth", 10.0))
        variants, _ = build_variants(depth, min_depth, max_depth, session["meta"])
        case_grids[cam] = (
            frame,
            {name: (variants[name]["grid"], variants[name]["bearing"],
                    variants[name]["elevation"], variants[name]["coverage"])
             for name in VARIANTS},
        )
        saved = load_saved_grid(session, frame)
        if saved is not None:
            diff = np.abs(variants["V0_fill_min"]["grid"] - saved)
            v0_validation[cam] = {
                "frame": frame,
                "mae": float(np.mean(diff)),
                "p95": float(np.percentile(diff, 95)),
            }

    # Aggregate.
    summary = {}
    for cam in ("RealSense", "ZED"):
        summary[cam] = {}
        for name in VARIANTS:
            rows = [r for r in records if r["camera"] == cam and r["variant"] == name]
            if not rows:
                continue
            summary[cam][name] = {
                key: float(np.nanmean([r[key] for r in rows]))
                for key in ("valid_frac", "covered_frac", "near08", "near15", "center_min",
                            "right_min", "floor_near", "false_near", "real_near08")
                if any(np.isfinite(r[key]) for r in rows)
            }
            if any("coverage" in r for r in rows):
                summary[cam][name]["coverage"] = float(np.nanmean([r.get("coverage", np.nan) for r in rows]))

    # Cross-camera divergence per pair and variant.
    by_pair = {}
    for r in records:
        by_pair.setdefault((r["pair"], r["variant"]), {})[r["camera"]] = r
    cross = {}
    for name in VARIANTS:
        diffs = []
        for (pair, var), cams in by_pair.items():
            if var != name or "RealSense" not in cams or "ZED" not in cams:
                continue
            a, b = cams["RealSense"], cams["ZED"]
            diffs.append({
                "d_center": abs(a["center_min"] - b["center_min"]),
                "d_right": abs(a["right_min"] - b["right_min"]),
                "d_floor": abs(a["floor_near"] - b["floor_near"]),
                "d_near08": abs(a["near08"] - b["near08"]),
            })
        if diffs:
            cross[name] = {k: float(np.nanmean([d[k] for d in diffs])) for k in diffs[0]}

    # V5 pixel-wise MAE over co-covered cells (same virtual FOV).
    v5_mae = []
    for (pair, var), cams in by_pair.items():
        if var != "V5_virtual_cam" or "RealSense" not in cams or "ZED" not in cams:
            continue
        ia, ib = pairs[pair]
        fa = int(rs["data"]["frame_idx"][ia])
        fb = int(zed["data"]["frame_idx"][ib])
        da = load_raw_depth(rs, fa)
        db = load_raw_depth(zed, fb)
        if da is None or db is None:
            continue
        va, _ = build_variants(da, float(rs["meta"].get("min_depth", 0.15)),
                               float(rs["meta"].get("max_depth", 10.0)), rs["meta"])
        vb, _ = build_variants(db, float(zed["meta"].get("min_depth", 0.25)),
                               float(zed["meta"].get("max_depth", 10.0)), zed["meta"])
        ga = va["V5_virtual_cam"]["grid"]
        gb = vb["V5_virtual_cam"]["grid"]
        both = (ga > 0) & (gb > 0)
        if both.any():
            v5_mae.append(float(np.mean(np.abs(ga[both] - gb[both]))))

    # Write outputs.
    with open(os.path.join(out_dir, "ablation_metrics.csv"), "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["pair", "row", "frame", "camera", "variant", "valid_frac", "covered_frac",
                         "near08", "near15", "center_min", "right_min", "floor_near", "false_near",
                         "real_near08", "coverage"])
        for r in records:
            writer.writerow([r["pair"], r["row"], r["frame"], r["camera"], r["variant"],
                             f"{r['valid_frac']:.4f}", f"{r.get('covered_frac', np.nan):.4f}",
                             f"{r['near08']:.4f}", f"{r['near15']:.4f}",
                             f"{r['center_min']:.3f}", f"{r['right_min']:.3f}", f"{r['floor_near']:.3f}",
                             "" if not np.isfinite(r["false_near"]) else f"{r['false_near']:.4f}",
                             "" if not np.isfinite(r["real_near08"]) else f"{r['real_near08']:.4f}",
                             f"{r.get('coverage', np.nan):.3f}"])

    payload = {
        "summary": summary,
        "cross_camera": cross,
        "v5_pixel_mae": {
            "mean": float(np.mean(v5_mae)) if v5_mae else None,
            "median": float(np.median(v5_mae)) if v5_mae else None,
            "n": len(v5_mae),
        },
        "v0_validation": v0_validation,
        "case_frames": {cam: case_grids[cam][0] for cam in case_grids},
    }
    with open(os.path.join(out_dir, "ablation_summary.json"), "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

    # Case grids as npy.
    for cam, (frame, grids) in case_grids.items():
        for name, (grid, _, _, _) in grids.items():
            np.save(os.path.join(out_dir, f"case_{cam}_{frame}_{name}.npy"), grid)

    # Figures.
    rows_data = []
    for cam in ("RealSense", "ZED"):
        if cam in case_grids:
            frame, grids = case_grids[cam]
            rows_data.append((cam, frame, {name: grids[name][0] for name in VARIANTS}))
    if rows_data:
        variants_svg(rows_data, os.path.join(out_dir, "variants_case.svg"))
    bars_svg(summary, os.path.join(out_dir, "variant_metrics.svg"))

    # Markdown report.
    def fmt(value, digits=3, dash="—"):
        if value is None or not isinstance(value, (int, float)) or not np.isfinite(value):
            return dash
        return f"{value:.{digits}f}"

    lines = ["# Preprocessing ablation V0–V5\n"]
    lines.append("Rebuilt from the recorded raw depth PNGs; means over matched frames.\n")
    lines.append("| variant | camera | valid_frac | near08 | near15 | real_near08 | center_min | right_min | floor_near | false_near | coverage |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for name in VARIANTS:
        for cam in ("RealSense", "ZED"):
            s = summary.get(cam, {}).get(name)
            if not s:
                continue
            lines.append(
                f"| {name} | {cam} | {fmt(s.get('valid_frac'))} | {fmt(s.get('near08'))} | "
                f"{fmt(s.get('near15'))} | {fmt(s.get('real_near08'))} | {fmt(s.get('center_min'), 2)} | "
                f"{fmt(s.get('right_min'), 2)} | {fmt(s.get('floor_near'), 2)} | "
                f"{fmt(s.get('false_near'))} | {fmt(s.get('coverage'))} |")
    lines.append("")
    if v0_validation:
        lines.append("V0 rebuild vs the saved 64×40 (raw PNG is downscaled to 320 px, so a small difference is expected):\n")
        for cam, v in v0_validation.items():
            lines.append(f"- {cam} frame {v['frame']}: MAE {v['mae']:.3f} m, P95 {v['p95']:.3f} m")
        lines.append("")
    lines.append("## Cross-camera divergence (mean |A−B| at matched poses)\n")
    lines.append("| variant | Δcenter | Δright | Δfloor | Δnear08 |")
    lines.append("|---|---:|---:|---:|---:|")
    for name in VARIANTS:
        c = cross.get(name)
        if not c:
            continue
        lines.append(f"| {name} | {c['d_center']:.2f} | {c['d_right']:.2f} | {c['d_floor']:.2f} | {c['d_near08']:.3f} |")
    lines.append("")
    if payload["v5_pixel_mae"]["mean"] is not None:
        lines.append(f"V5 pixel-wise MAE over co-covered cells: mean {payload['v5_pixel_mae']['mean']:.2f} m, "
                     f"median {payload['v5_pixel_mae']['median']:.2f} m, n={payload['v5_pixel_mae']['n']}.\n")
    lines.append("## Case frames\n")
    for cam, (frame, grids) in case_grids.items():
        lines.append(f"### {cam} frame {frame}\n")
        lines.append("| variant | center_min | right_min | floor_near | near08 |")
        lines.append("|---|---:|---:|---:|---:|")
        for name in VARIANTS:
            g, bearing, elevation, coverage = grids[name]
            s = sector_stats(g, bearing, elevation, coverage)
            lines.append(f"| {name} | {s['center_min']:.2f} | {s['right_min']:.2f} | {s['floor_near']:.2f} | {s['near08']:.3f} |")
        lines.append("")
    lines.append("![variants](variants_case.svg)\n")
    lines.append("![bars](variant_metrics.svg)\n")
    with open(os.path.join(out_dir, "report_ablation.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"wrote {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

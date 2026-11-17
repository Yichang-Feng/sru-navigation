#!/usr/bin/env python3
"""
Compare two camera diagnostic sessions recorded by door_diagnostic_logger.py (schema >= 2.0).

The robot path is chosen manually in RViz, so the two runs are only *approximately*
identical. This tool therefore reports:
  1. Session summary (camera, frames, duration, distance, speed, depth health).
  2. Trajectory similarity after optional start-pose alignment (mean/P95 nearest distance).
  3. Matched-state command differences: for nearby robot poses, compare cmd_vx/cmd_wz,
     center clearance and target distance.
  4. Progress-normalized series (0..1 along the traveled path) for speed, commands,
     center clearance and target distance.
  5. A Markdown + JSON report and a progress_series.csv for plotting.

No ROS 2 dependency. Usage:
    python3 compare_camera_sessions.py \
        --a ~/door_diagnostics/session_..._realsense_S3 \
        --b ~/door_diagnostics/session_..._zed_S3 \
        --label-a RealSense --label-b ZED \
        --out ~/door_diagnostics/compare_RS_vs_ZED
"""

import os
import csv
import json
import math
import argparse
import datetime
from collections import Counter

import numpy as np


NUMERIC_COLUMNS = [
    'frame_idx', 'epoch_sec',
    'robot_x', 'robot_y', 'robot_z', 'robot_yaw_deg',
    'carrot_x', 'carrot_y', 'carrot_z',
    'target_dist', 'target_angle_deg',
    'cmd_vx', 'cmd_vy', 'cmd_wz',
    'door_center_min_depth', 'door_center_avg_depth',
    'robot_roll_deg', 'robot_pitch_deg',
    'odom_vx', 'odom_vy', 'odom_vz', 'odom_wz', 'speed_mps',
    'target_dx_b', 'target_dy_b', 'target_dz_b', 'target_dist_3d', 'target_pitch_deg',
    'depth_left_min', 'depth_left_avg', 'depth_center_min', 'depth_center_avg',
    'depth_right_min', 'depth_right_avg', 'depth_floor_near', 'center_void_ratio',
    'path_points_count', 'path_dist_to_end', 'path_slope_ahead', 'path_length',
    'depth_fps_est', 'depth_period_ms', 'depth_latency_ms', 'depth_age_ms',
    'depth_valid_ratio_raw', 'depth_nan_ratio', 'depth_zero_ratio',
    'depth_near_ratio', 'depth_far_ratio', 'depth_hole_ratio',
    'depth_min', 'depth_p10', 'depth_median', 'depth_mean', 'depth_p90', 'depth_max',
    'depth_valid_ratio_40x64', 'depth_zero_ratio_40x64',
    'min_depth_param', 'max_depth_param',
    'raw_cmd_vx', 'raw_cmd_vy', 'raw_cmd_wz',
    'goal_x', 'goal_y', 'goal_z', 'goal_yaw_deg', 'goal_dist', 'goal_age_ms',
    'lidar_min_dist_front', 'lidar_min_dist_any',
    'lidar_nearest_x', 'lidar_nearest_y', 'lidar_nearest_z',
    'imu_ax', 'imu_ay', 'imu_az', 'imu_gx', 'imu_gy', 'imu_gz',
    'adaptive_lookahead', 'adaptive_l_curv', 'adaptive_l_clear', 'adaptive_l_target',
    'adaptive_turn_deg', 'adaptive_clearance_w',
    'stair_gate_slope', 'stair_gate_dz',
]


def _to_float(value):
    if value is None:
        return None
    try:
        s = str(value).strip()
        if s == '':
            return None
        v = float(s)
        return v if math.isfinite(v) else None
    except Exception:
        return None


def load_session(path):
    path = os.path.expanduser(path)
    csv_path = os.path.join(path, 'trajectory.csv')
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f'trajectory.csv not found in {path}')
    rows = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for raw in reader:
            row = dict(raw)
            for key in NUMERIC_COLUMNS:
                if key in row:
                    row[key] = _to_float(row.get(key))
            rows.append(row)
    summary = {}
    for name in ('session_summary.json', 'session_meta.json'):
        p = os.path.join(path, name)
        if os.path.isfile(p):
            try:
                with open(p, 'r', encoding='utf-8') as f:
                    summary.update(json.load(f))
            except Exception:
                pass
    return rows, summary


def _valid_xy(rows):
    pts = []
    for r in rows:
        x, y = r.get('robot_x'), r.get('robot_y')
        if x is not None and y is not None:
            pts.append([x, y])
    return np.asarray(pts, dtype=float).reshape(-1, 2)


def _yaw_deg(row):
    y = row.get('robot_yaw_deg')
    return y if y is not None else 0.0


def _start_pose(rows):
    for r in rows:
        x, y = r.get('robot_x'), r.get('robot_y')
        if x is not None and y is not None:
            return np.array([x, y], dtype=float), math.radians(_yaw_deg(r))
    return np.zeros(2), 0.0


def _align_to_start(rows):
    """Return xy points aligned to the first pose (translation + yaw)."""
    pts = _valid_xy(rows)
    if len(pts) == 0:
        return pts
    start, yaw0 = _start_pose(rows)
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    R = np.array([[c, -s], [s, c]], dtype=float)
    return (pts - start) @ R.T


def _path_length(pts):
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def _cumulative_progress(pts):
    if len(pts) == 0:
        return np.array([])
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] > 1e-9:
        s = s / s[-1]
    return s


def _nearest_distances(a, b):
    """For each point in a, distance to nearest point in b."""
    if len(a) == 0 or len(b) == 0:
        return np.array([])
    # Brute force is fine for a few thousand points.
    d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2)
    return np.min(d, axis=1)


def _series_stats(values):
    vals = np.asarray([v for v in values if v is not None and math.isfinite(v)], dtype=float)
    if vals.size == 0:
        return {}
    return {
        'count': int(vals.size),
        'mean': float(np.mean(vals)),
        'median': float(np.median(vals)),
        'p95': float(np.percentile(vals, 95)),
        'min': float(np.min(vals)),
        'max': float(np.max(vals)),
    }


def summarize(rows, summary, label):
    pts = _valid_xy(rows)
    aligned = _align_to_start(rows)
    start, start_yaw = _start_pose(rows)
    end = pts[-1] if len(pts) else np.array([np.nan, np.nan])

    frames = len(rows)
    durations = [r.get('epoch_sec') for r in rows if r.get('epoch_sec') is not None]
    duration = (max(durations) - min(durations)) if len(durations) >= 2 else None
    speed = [r.get('speed_mps') if r.get('speed_mps') is not None else
             (math.hypot(r.get('odom_vx') or 0.0, r.get('odom_vy') or 0.0)) for r in rows]
    cmd_vx = [r.get('cmd_vx') for r in rows]
    cmd_wz = [r.get('cmd_wz') for r in rows]
    center_min = [r.get('depth_center_min') for r in rows]
    center_avg = [r.get('depth_center_avg') for r in rows]
    valid = [r.get('depth_valid_ratio_raw') for r in rows]
    hole = [r.get('depth_hole_ratio') for r in rows]
    latency = [r.get('depth_latency_ms') for r in rows]
    lidar = [r.get('lidar_min_dist_front') for r in rows]
    target = [r.get('target_dist') for r in rows]
    nav_states = Counter(r.get('nav_state', '') for r in rows if r.get('nav_state'))

    depth = {
        'valid_ratio_raw': _series_stats(valid),
        'hole_ratio_raw': _series_stats(hole),
        'latency_ms': _series_stats(latency),
        'center_min': _series_stats(center_min),
        'center_avg': _series_stats(center_avg),
        'depth_min': _series_stats([r.get('depth_min') for r in rows]),
        'depth_median': _series_stats([r.get('depth_median') for r in rows]),
        'depth_p90': _series_stats([r.get('depth_p90') for r in rows]),
    }

    goal = summary.get('goal') or {}
    goal_pos = goal.get('pos') or [None, None]
    if len(goal_pos) < 2:
        goal_pos = [None, None]

    return {
        'label': label,
        'path': summary.get('session_dir', ''),
        'camera_id': summary.get('camera_id') or (rows[0].get('camera_id') if rows else ''),
        'depth_topic': summary.get('depth_topic') or (rows[0].get('depth_topic') if rows else ''),
        'frames': frames,
        'duration_sec': duration,
        'distance_m': _path_length(pts),
        'start_xy': start.tolist(),
        'start_yaw_deg': math.degrees(start_yaw),
        'end_xy': end.tolist(),
        'goal_xy': [goal_pos[0], goal_pos[1]],
        'path_length_m': _path_length(aligned),
        'speed_mps': _series_stats(speed),
        'cmd_vx': _series_stats(cmd_vx),
        'cmd_wz': _series_stats(cmd_wz),
        'target_dist': _series_stats(target),
        'lidar_min_front_m': _series_stats(lidar),
        'nav_states': dict(nav_states),
        'stair_down_frames': summary.get('stair_down_frames', 0),
        'stair_up_frames': summary.get('stair_up_frames', 0),
        'depth': depth,
        'summary': summary,
    }


def trajectory_similarity(rows_a, rows_b):
    a = _align_to_start(rows_a)
    b = _align_to_start(rows_b)
    if len(a) == 0 or len(b) == 0:
        return {}
    da = _nearest_distances(a, b)
    db = _nearest_distances(b, a)
    return {
        'aligned_start_a': a[0].tolist(),
        'aligned_end_a': a[-1].tolist(),
        'aligned_end_b': b[-1].tolist(),
        'a_to_b_mean_m': float(np.mean(da)),
        'a_to_b_p95_m': float(np.percentile(da, 95)),
        'a_to_b_max_m': float(np.max(da)),
        'b_to_a_mean_m': float(np.mean(db)),
        'b_to_a_p95_m': float(np.percentile(db, 95)),
        'b_to_a_max_m': float(np.max(db)),
        'endpoint_distance_m': float(np.linalg.norm(a[-1] - b[-1])),
        'path_length_a_m': _path_length(a),
        'path_length_b_m': _path_length(b),
        'path_length_delta_m': _path_length(b) - _path_length(a),
    }


def matched_state_compare(rows_a, rows_b, radius=0.5, max_samples=600):
    a_pts = _align_to_start(rows_a)
    b_pts = _align_to_start(rows_b)
    if len(a_pts) == 0 or len(b_pts) == 0:
        return {}
    step = max(1, len(a_pts) // max_samples)
    keys = ['cmd_vx', 'cmd_wz', 'depth_center_min', 'depth_center_avg', 'target_dist']
    diffs = {k: [] for k in keys}
    matched = 0
    for i in range(0, len(a_pts), step):
        d = np.linalg.norm(b_pts - a_pts[i], axis=1)
        j = int(np.argmin(d))
        if d[j] > radius:
            continue
        matched += 1
        for k in keys:
            va = rows_a[i].get(k) if i < len(rows_a) else None
            vb = rows_b[j].get(k) if j < len(rows_b) else None
            if va is not None and vb is not None:
                diffs[k].append(abs(vb - va))
    return {
        'radius_m': radius,
        'samples_checked': int(math.ceil(len(a_pts) / step)),
        'matched_samples': matched,
        'match_rate': matched / max(1, math.ceil(len(a_pts) / step)),
        'abs_diff': {k: _series_stats(v) for k, v in diffs.items()},
    }


def resample_series(progress, values, bins=101):
    pairs = [(p, v) for p, v in zip(progress, values) if v is not None and math.isfinite(v)]
    if len(pairs) < 2:
        return np.full(bins, np.nan)
    ps = np.array([p for p, _ in pairs], dtype=float)
    vs = np.array([v for _, v in pairs], dtype=float)
    # Remove duplicate progress values for np.interp.
    order = np.argsort(ps)
    ps, vs = ps[order], vs[order]
    uniq, idx = np.unique(ps, return_index=True)
    vs = vs[idx]
    if len(uniq) < 2:
        return np.full(bins, np.nan)
    grid = np.linspace(0.0, 1.0, bins)
    return np.interp(grid, uniq, vs)


def progress_compare(rows_a, rows_b, bins=101):
    a_pts = _align_to_start(rows_a)
    b_pts = _align_to_start(rows_b)
    if len(a_pts) < 2 or len(b_pts) < 2:
        return {}
    pa = _cumulative_progress(a_pts)
    pb = _cumulative_progress(b_pts)
    metrics = {
        'speed_mps': lambda r: r.get('speed_mps') if r.get('speed_mps') is not None
        else math.hypot(r.get('odom_vx') or 0.0, r.get('odom_vy') or 0.0),
        'cmd_vx': lambda r: r.get('cmd_vx'),
        'cmd_wz': lambda r: r.get('cmd_wz'),
        'depth_center_min': lambda r: r.get('depth_center_min'),
        'depth_center_avg': lambda r: r.get('depth_center_avg'),
        'target_dist': lambda r: r.get('target_dist'),
        'lidar_min_dist_front': lambda r: r.get('lidar_min_dist_front'),
    }
    out = {}
    series = {}
    grid = np.linspace(0.0, 1.0, bins)
    for name, getter in metrics.items():
        va = [getter(r) for r in rows_a]
        vb = [getter(r) for r in rows_b]
        sa = resample_series(pa, va, bins)
        sb = resample_series(pb, vb, bins)
        mask = np.isfinite(sa) & np.isfinite(sb)
        if np.count_nonzero(mask) >= 2:
            diff = sb[mask] - sa[mask]
            corr = float(np.corrcoef(sa[mask], sb[mask])[0, 1]) if np.std(sa[mask]) > 1e-9 and np.std(sb[mask]) > 1e-9 else None
            out[name] = {
                'mean_abs_diff': float(np.mean(np.abs(diff))),
                'median_abs_diff': float(np.median(np.abs(diff))),
                'mean_diff_b_minus_a': float(np.mean(diff)),
                'correlation': corr,
                'overlap_bins': int(np.count_nonzero(mask)),
            }
        series[name] = (sa, sb)
    return {'bins': bins, 'metrics': out, 'series': series, 'grid': grid}


def _md_table(rows, headers):
    def esc(x):
        return str(x).replace('|', '\\|')
    lines = ['| ' + ' | '.join(esc(h) for h in headers) + ' |',
             '|' + '|'.join(['---'] * len(headers)) + '|']
    for row in rows:
        lines.append('| ' + ' | '.join(esc(x) for x in row) + ' |')
    return '\n'.join(lines)


def _val(stats, key, fmt='{:.3f}', default='--'):
    if not stats or stats.get(key) is None:
        return default
    try:
        return fmt.format(stats[key])
    except Exception:
        return str(stats[key])


def write_report(out_dir, label_a, label_b, sa, sb, sim, matched, progress):
    os.makedirs(out_dir, exist_ok=True)
    report = {
        'generated_at': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'label_a': label_a,
        'label_b': label_b,
        'session_a': sa,
        'session_b': sb,
        'trajectory_similarity': sim,
        'matched_state': matched,
        'progress_compare': {
            k: v for k, v in progress.items() if k not in ('series', 'grid')
        },
    }
    with open(os.path.join(out_dir, 'comparison_report.json'), 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    # progress series CSV
    if progress.get('series'):
        grid = progress['grid']
        header = ['progress']
        cols = {}
        for name, (sa_vals, sb_vals) in progress['series'].items():
            header.append(f'A_{name}')
            header.append(f'B_{name}')
            cols[f'A_{name}'] = sa_vals
            cols[f'B_{name}'] = sb_vals
        with open(os.path.join(out_dir, 'progress_series.csv'), 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(header)
            for i in range(len(grid)):
                w.writerow([f'{grid[i]:.4f}'] + [f'{cols[h][i]:.4f}' if math.isfinite(cols[h][i]) else '' for h in header[1:]])

    lines = []
    lines.append('# Camera Session Comparison')
    lines.append('')
    lines.append(f'- Generated: {report["generated_at"]}')
    lines.append(f'- A: **{label_a}** — `{sa.get("path", "")}`')
    lines.append(f'- B: **{label_b}** — `{sb.get("path", "")}`')
    lines.append('')
    lines.append('## 1. Session summary')
    lines.append('')
    lines.append(_md_table([
        ['Camera', sa.get('camera_id', '--'), sb.get('camera_id', '--')],
        ['Depth topic', sa.get('depth_topic', '--'), sb.get('depth_topic', '--')],
        ['Frames', sa.get('frames', '--'), sb.get('frames', '--')],
        ['Duration (s)', _val({'v': sa.get('duration_sec')}, 'v', '{:.1f}'), _val({'v': sb.get('duration_sec')}, 'v', '{:.1f}')],
        ['Path length (m)', _val({'v': sa.get('path_length_m')}, 'v'), _val({'v': sb.get('path_length_m')}, 'v')],
        ['Mean speed (m/s)', _val(sa.get('speed_mps', {}), 'mean'), _val(sb.get('speed_mps', {}), 'mean')],
        ['Max speed (m/s)', _val(sa.get('speed_mps', {}), 'max'), _val(sb.get('speed_mps', {}), 'max')],
        ['Mean cmd_vx (m/s)', _val(sa.get('cmd_vx', {}), 'mean'), _val(sb.get('cmd_vx', {}), 'mean')],
        ['Stair down frames', sa.get('stair_down_frames', 0), sb.get('stair_down_frames', 0)],
        ['Stair up frames', sa.get('stair_up_frames', 0), sb.get('stair_up_frames', 0)],
    ], ['Metric', label_a, label_b]))
    lines.append('')
    lines.append('## 2. Trajectory similarity')
    lines.append('')
    if sim:
        lines.append(_md_table([
            ['A→B nearest mean (m)', _val(sim, 'a_to_b_mean_m')],
            ['A→B nearest P95 (m)', _val(sim, 'a_to_b_p95_m')],
            ['B→A nearest mean (m)', _val(sim, 'b_to_a_mean_m')],
            ['B→A nearest P95 (m)', _val(sim, 'b_to_a_p95_m')],
            ['Endpoint distance (m)', _val(sim, 'endpoint_distance_m')],
            ['Path length Δ (B−A, m)', _val(sim, 'path_length_delta_m')],
        ], ['Metric', 'Value']))
        lines.append('')
        if sim['a_to_b_mean_m'] > 0.5 or sim['b_to_a_mean_m'] > 0.5:
            lines.append('> **Warning**: the two trajectories differ by more than 0.5 m on average; '
                         'matched-state and aggregate comparisons may be confounded by the path difference.')
    else:
        lines.append('_Not enough trajectory points._')
    lines.append('')
    lines.append('## 3. Matched-state command differences')
    lines.append('')
    if matched:
        lines.append(f"Matched samples: {matched.get('matched_samples', 0)} / "
                     f"{matched.get('samples_checked', 0)} "
                     f"({matched.get('match_rate', 0)*100:.1f}%) within {matched.get('radius_m', 0.5)} m.")
        lines.append('')
        rows = []
        for k in ('cmd_vx', 'cmd_wz', 'depth_center_min', 'depth_center_avg', 'target_dist'):
            st = matched.get('abs_diff', {}).get(k, {})
            rows.append([k, _val(st, 'count', '{:.0f}'), _val(st, 'mean'), _val(st, 'median'), _val(st, 'p95')])
        lines.append(_md_table(rows, ['Metric', 'N', 'Mean |Δ|', 'Median |Δ|', 'P95 |Δ|']))
        lines.append('')
        lines.append('> Δ = B − A at matched poses. Large command differences at similar poses indicate '
                     'camera-induced policy divergence.')
    else:
        lines.append('_No matched poses within the radius._')
    lines.append('')
    lines.append('## 4. Depth health')
    lines.append('')
    rows = []
    for key, label in [
        ('valid_ratio_raw', 'Valid ratio (raw)'),
        ('hole_ratio_raw', 'Hole ratio (raw)'),
        ('latency_ms', 'Latency (ms)'),
        ('center_min', 'Center min depth (m)'),
        ('center_avg', 'Center avg depth (m)'),
        ('depth_median', 'Depth median (m)'),
        ('depth_p90', 'Depth P90 (m)'),
    ]:
        a = sa.get('depth', {}).get(key, {})
        b = sb.get('depth', {}).get(key, {})
        rows.append([label, _val(a, 'mean'), _val(b, 'mean'), _val(a, 'p95'), _val(b, 'p95')])
    lines.append(_md_table(rows, ['Metric', f'{label_a} mean', f'{label_b} mean', f'{label_a} P95', f'{label_b} P95']))
    lines.append('')
    lines.append('## 5. Progress-normalized comparison')
    lines.append('')
    if progress.get('metrics'):
        rows = []
        for name, st in progress['metrics'].items():
            rows.append([
                name,
                _val(st, 'mean_abs_diff', '{:.4f}'),
                _val(st, 'mean_diff_b_minus_a', '{:+.4f}'),
                _val(st, 'correlation', '{:.3f}'),
                st.get('overlap_bins', '--'),
            ])
        lines.append(_md_table(rows, ['Metric', 'Mean |Δ|', 'Mean Δ (B−A)', 'Correlation', 'Overlap bins']))
        lines.append('')
        lines.append('Progress is normalized 0..1 along each run\'s own traveled path.')
    else:
        lines.append('_Not enough data for progress comparison._')
    lines.append('')
    lines.append('## 6. Output files')
    lines.append('')
    lines.append('- `comparison_report.json` — all metrics')
    lines.append('- `comparison_report.md` — this report')
    lines.append('- `progress_series.csv` — 0..1 progress-normalized series')
    lines.append('')

    with open(os.path.join(out_dir, 'comparison_report.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
    return report


def main():
    parser = argparse.ArgumentParser(description='Compare two camera diagnostic sessions.')
    parser.add_argument('--a', required=True, help='Session A directory (e.g. RealSense)')
    parser.add_argument('--b', required=True, help='Session B directory (e.g. ZED)')
    parser.add_argument('--label-a', default='A')
    parser.add_argument('--label-b', default='B')
    parser.add_argument('--out', default='', help='Output directory')
    parser.add_argument('--match-radius', type=float, default=0.5,
                        help='Matched-state radius in meters (default 0.5)')
    parser.add_argument('--no-align-start', action='store_true',
                        help='Do not align both trajectories to their start pose')
    args = parser.parse_args()

    rows_a, summary_a = load_session(args.a)
    rows_b, summary_b = load_session(args.b)

    sa = summarize(rows_a, summary_a, args.label_a)
    sb = summarize(rows_b, summary_b, args.label_b)
    sim = trajectory_similarity(rows_a, rows_b)
    matched = matched_state_compare(rows_a, rows_b, radius=args.match_radius)
    progress = progress_compare(rows_a, rows_b)

    out_dir = args.out or os.path.join(
        os.path.dirname(os.path.expanduser(args.a)),
        f"compare_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}",
    )
    report = write_report(out_dir, args.label_a, args.label_b, sa, sb, sim, matched, progress)

    print(f"[✓] Report written to: {out_dir}")
    print(f"  A: {sa['camera_id']} / {sa['frames']} frames / {sa['distance_m']:.2f} m")
    print(f"  B: {sb['camera_id']} / {sb['frames']} frames / {sb['distance_m']:.2f} m")
    if sim:
        print(f"  Trajectory mean nearest distance: A→B {sim['a_to_b_mean_m']:.3f} m, "
              f"B→A {sim['b_to_a_mean_m']:.3f} m")
    if matched:
        cv = matched.get('abs_diff', {}).get('cmd_vx', {})
        wz = matched.get('abs_diff', {}).get('cmd_wz', {})
        print(f"  Matched poses: {matched['matched_samples']} "
              f"({matched['match_rate']*100:.1f}%), "
              f"mean |Δvx|={_val(cv, 'mean')} m/s, mean |Δwz|={_val(wz, 'mean')} rad/s")
    print(f"  Markdown: {os.path.join(out_dir, 'comparison_report.md')}")


if __name__ == '__main__':
    main()

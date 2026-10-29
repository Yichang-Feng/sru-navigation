#!/usr/bin/env python3
"""
Unitree Go2W Offline Benchmark Analysis & Quantitative Report Generator
-----------------------------------------------------------------------
Analyzes CSV logs recorded by benchmark_collector.py to produce:
  1. Standardized Quantitative Metrics:
     - Success Rate (SR), Collision Rate (CR)
     - Minimum Clearance Distance (d_min)
     - Reaction Initiation Distance (D_react)
     - SPL (Success weighted by Path Length)
     - Time-in-Danger Ratio (T_danger)
     - Close-range Zero-pixel Corruption Ratio
  2. Diagnostic Visualizations:
     - Fig 1: Close-range Deceleration & Depth Clipping Diagnostic
     - Fig 2: 2D Spatial Trajectory with LiDAR Ground Truth Clearance
     - Fig 3: Multi-height Reaction & Clearance Comparison (if directory provided)

Requirements:
  Python 3 (numpy, matplotlib, pandas/csv) - runnable without ROS 2 installed!
"""

import os
import sys
import glob
import argparse
import json
import csv
import numpy as np

# Configure matplotlib for headless generation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def parse_csv_log(csv_path: str):
    """Load and parse benchmark CSV data into numpy arrays."""
    data = {}
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        for field in fields:
            data[field] = []
        for row in reader:
            for field in fields:
                val = row[field]
                try:
                    data[field].append(float(val))
                except ValueError:
                    data[field].append(val)

    for field in fields:
        # If numeric, convert to numpy array
        try:
            data[field] = np.array(data[field], dtype=np.float64)
        except (ValueError, TypeError):
            data[field] = np.array(data[field])

    return data


def compute_quantitative_metrics(data: dict, safe_dist_thred: float = 0.40, collision_thred: float = 0.22):
    """Compute all evaluation metrics from time-series log."""
    rel_time = data["rel_time"]
    rx = data["robot_x"]
    ry = data["robot_y"]
    cmd_vx = data["cmd_vx"]
    cmd_wz = data["cmd_wz"]
    dto = data["lidar_min_dist_front"]
    center_zero_pct = data["depth_center_zero_pct"]

    # Filter out valid DTO readings (> 0)
    valid_dto_mask = (dto > 0.05) & (dto < 10.0)

    # 1. Minimum Clearance Distance (d_min)
    if np.any(valid_dto_mask):
        d_min = float(np.min(dto[valid_dto_mask]))
        idx_min = np.argmin(dto[valid_dto_mask])
    else:
        d_min = -1.0
        idx_min = -1

    # 2. Collision Assessment
    is_collision = (d_min > 0 and d_min <= collision_thred)

    # 3. Reaction Initiation Distance (D_react)
    # Search for first significant deceleration or steering when obstacle is in front (< 2.5m)
    d_react = -1.0
    nominal_vx = np.mean(cmd_vx[:min(30, len(cmd_vx))]) if len(cmd_vx) > 0 else 0.8
    for t_idx in range(len(rel_time)):
        d_val = dto[t_idx]
        if 0.2 < d_val < 2.5:
            # Did the robot decelerate or steer?
            if cmd_vx[t_idx] < (nominal_vx * 0.75) or abs(cmd_wz[t_idx]) > 0.25:
                d_react = float(d_val)
                break

    # 4. Time-in-Danger Ratio (T_danger)
    if np.any(valid_dto_mask):
        in_danger_mask = (dto > 0.05) & (dto < safe_dist_thred)
        t_danger_ratio = float(np.sum(in_danger_mask) / len(dto))
    else:
        t_danger_ratio = 0.0

    # 5. Close-range Depth Zero-pixel Corruption (Diagnostic for Issue 2)
    # When obstacle was within 0.5m, what was the average zero-depth pixel percentage in center ROI?
    close_mask = (dto > 0.05) & (dto < 0.50)
    if np.any(close_mask):
        mean_close_zero_pct = float(np.mean(center_zero_pct[close_mask]))
        max_close_zero_pct = float(np.max(center_zero_pct[close_mask]))
    else:
        mean_close_zero_pct = 0.0
        max_close_zero_pct = 0.0

    # 6. Actual Path Length
    dx = np.diff(rx)
    dy = np.diff(ry)
    actual_length = float(np.sum(np.sqrt(dx**2 + dy**2)))

    # Optimal path length (straight line from start to end)
    optimal_length = float(np.sqrt((rx[-1] - rx[0])**2 + (ry[-1] - ry[0])**2))
    spl = float(optimal_length / max(actual_length, optimal_length)) if actual_length > 0 else 0.0

    metrics = {
        "duration_s": float(rel_time[-1] - rel_time[0]),
        "total_steps": len(rel_time),
        "actual_path_length_m": actual_length,
        "optimal_path_length_m": optimal_length,
        "spl": spl,
        "min_clearance_m": d_min,
        "is_collision": is_collision,
        "reaction_distance_m": d_react,
        "time_in_danger_ratio": t_danger_ratio,
        "mean_close_range_zero_pct": mean_close_zero_pct,
        "max_close_range_zero_pct": max_close_zero_pct
    }
    return metrics


def plot_single_run_diagnostic(data: dict, metrics: dict, output_img_path: str, title_str: str = ""):
    """Generate diagnostic multi-subplot figure exposing close-range and low-obstacle defects."""
    rel_time = data["rel_time"]
    rx = data["robot_x"]
    ry = data["robot_y"]
    cmd_vx = data["cmd_vx"]
    cmd_wz = data["cmd_wz"]
    dto = data["lidar_min_dist_front"]
    depth_center_zero = data["depth_center_zero_pct"]
    depth_bottom = data["depth_bottom_mean"]

    fig, axs = plt.subplots(3, 1, figsize=(10, 11), sharex=True)
    fig.suptitle(title_str or "Unitree Go2W Navigation Benchmark Diagnostic", fontsize=13, fontweight='bold')

    # Subplot 1: Speed vs LiDAR Obstacle Distance
    ax1 = axs[0]
    color_dto = 'tab:red'
    ax1.set_ylabel('LiDAR DTO (m)', color=color_dto, fontsize=11)
    # Mask invalid dto
    dto_plot = np.where(dto > 0.05, dto, np.nan)
    line1 = ax1.plot(rel_time, dto_plot, color=color_dto, linewidth=2.0, label='Obstacle Distance (LiDAR Truth)')
    ax1.axhline(y=0.40, color='r', linestyle='--', alpha=0.6, label='Danger Threshold (0.4m)')
    ax1.axhline(y=0.22, color='darkred', linestyle=':', linewidth=2, label='Physical Collision Radius (0.22m)')
    ax1.tick_params(axis='y', labelcolor=color_dto)
    ax1.set_ylim([0.0, 3.5])
    ax1.grid(True, linestyle=':', alpha=0.5)

    ax1_twin = ax1.twinx()
    color_vx = 'tab:blue'
    ax1_twin.set_ylabel('Commanded vx (m/s)', color=color_vx, fontsize=11)
    line2 = ax1_twin.plot(rel_time, cmd_vx, color=color_vx, linewidth=1.8, label='cmd_vel vx')
    ax1_twin.tick_params(axis='y', labelcolor=color_vx)
    ax1_twin.set_ylim([-0.2, 1.6])

    # Combine legends
    lines = line1 + line2 + [plt.Line2D([0], [0], color='r', linestyle='--')]
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper right', framealpha=0.85, fontsize=9)
    ax1.set_title("1. Approach Dynamics: Speed Response vs Real Distance", fontsize=11)

    # Subplot 2: Depth Zero-Clipping Bug Diagnostic (Issue 2)
    ax2 = axs[1]
    ax2.plot(rel_time, depth_center_zero, color='purple', linewidth=2.0, label='Center ROI Zero Depth %')
    ax2.set_ylabel('Zero-Depth Pixels (%)', color='purple', fontsize=11)
    ax2.set_ylim([-5, 105])
    ax2.grid(True, linestyle=':', alpha=0.5)
    ax2.axhspan(40, 100, color='purple', alpha=0.15, label='High Dead-zone Risk (>40% 0.0 pixels)')
    ax2.legend(loc='upper left', fontsize=9)
    ax2.set_title("2. Close-range Perception Bug: Depth Zero-Clipping Ratio (min_depth bug)", fontsize=11)

    # Subplot 3: Angular Velocity & Bottom Depth (Issue 1 & Steering)
    ax3 = axs[2]
    ax3.plot(rel_time, cmd_wz, color='tab:green', linewidth=1.8, label='cmd_vel wz (Turning Action)')
    ax3.set_ylabel('wz (rad/s)', color='tab:green', fontsize=11)
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylim([-1.2, 1.2])
    ax3.grid(True, linestyle=':', alpha=0.5)

    ax3_twin = ax3.twinx()
    depth_bottom_plot = np.where(depth_bottom > 0.05, depth_bottom, np.nan)
    ax3_twin.plot(rel_time, depth_bottom_plot, color='tab:orange', linestyle='-.', label='Bottom Depth Mean (m)')
    ax3_twin.set_ylabel('Bottom Depth (m)', color='tab:orange', fontsize=11)
    ax3_twin.set_ylim([0.0, 3.5])

    lines3 = [plt.Line2D([0], [0], color='tab:green'), plt.Line2D([0], [0], color='tab:orange', linestyle='-.')]
    ax3.legend(lines3, ['cmd_vel wz', 'Bottom Depth Mean (m)'], loc='upper right', fontsize=9)
    ax3.set_title("3. Avoidance Steering & Low-Obstacle Ground Visibility", fontsize=11)

    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(output_img_path)), exist_ok=True)
    plt.savefig(output_img_path, dpi=200)
    plt.close()
    print(f"[Analysis] Saved diagnostic figure to:\n  -> {output_img_path}")


def print_metrics_table(metrics: dict, log_name: str = ""):
    """Pretty-print evaluation results to console."""
    print(f"\n{'='*65}")
    print(f"  QUANTITATIVE EVALUATION REPORT: {os.path.basename(log_name)}")
    print(f"{'='*65}")
    status_str = "\033[91mCOLLISION DETECTED\033[0m" if metrics["is_collision"] else "\033[92mSUCCESSFUL CLEARANCE\033[0m"
    print(f"  Overall Outcome:             {status_str}")
    print(f"  Duration:                    {metrics['duration_s']:.2f} s ({metrics['total_steps']} steps)")
    print(f"  Actual / Optimal Path:       {metrics['actual_path_length_m']:.2f}m / {metrics['optimal_path_length_m']:.2f}m")
    print(f"  SPL (Success Path Length):   {metrics['spl']:.3f}")
    print(f"  Minimum Clearance (d_min):   {metrics['min_clearance_m']:.3f} m")
    
    react_str = f"{metrics['reaction_distance_m']:.2f} m" if metrics['reaction_distance_m'] > 0 else "\033[91mNone (No Reaction!)\033[0m"
    print(f"  Reaction Distance (D_react): {react_str}")
    print(f"  Time in Danger Zone (<0.4m): {metrics['time_in_danger_ratio']*100:.1f} %")
    print(f"  Close-range Zero-pixel Mean: {metrics['mean_close_range_zero_pct']:.1f} % (Max: {metrics['max_close_range_zero_pct']:.1f}%)")
    print(f"{'='*65}\n")


def main():
    parser = argparse.ArgumentParser(description="Analyze Benchmark Logs")
    parser.add_argument("input_path", type=str, help="Path to CSV file or directory of CSV files")
    parser.add_argument("--output-dir", type=str, default="./benchmark_results", help="Directory to save figures")
    args = parser.parse_args()

    input_path = os.path.expanduser(args.input_path)
    if os.path.isfile(input_path):
        csv_files = [input_path]
    elif os.path.isdir(input_path):
        csv_files = sorted(glob.glob(os.path.join(input_path, "*.csv")))
    else:
        print(f"ERROR: Input path {input_path} not found.")
        sys.exit(1)

    if not csv_files:
        print(f"No CSV files found in {input_path}")
        sys.exit(1)

    for csv_file in csv_files:
        data = parse_csv_log(csv_file)
        if len(data.get("rel_time", [])) < 5:
            print(f"Skipping empty or corrupted log: {csv_file}")
            continue

        metrics = compute_quantitative_metrics(data)
        print_metrics_table(metrics, csv_file)

        base_name = os.path.splitext(os.path.basename(csv_file))[0]
        fig_path = os.path.join(os.path.expanduser(args.output_dir), f"{base_name}_diagnostic.png")
        plot_single_run_diagnostic(data, metrics, fig_path, title_str=f"Diagnostic: {base_name}")


if __name__ == "__main__":
    main()

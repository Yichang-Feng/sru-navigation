#!/usr/bin/env python3
"""Generate a realistic mock benchmark log to test analyze_benchmark.py."""
import os
import csv
import numpy as np

def generate_mock_log(csv_path: str):
    os.makedirs(os.path.dirname(os.path.abspath(csv_path)), exist_ok=True)
    t = np.linspace(0, 10, 200) # 20 Hz, 10s
    
    # Robot moving towards an obstacle from 3.0m at 1.0 m/s
    # Scenario: close range dead-zone bug occurs at t = 2.5s (d = 0.5m)
    # The obstacle depth turns to 0.0, policy fails to brake, collisions at t = 3.2s
    
    x = 0.8 * t
    y = np.zeros_like(t)
    yaw = np.zeros_like(t)
    
    # Distance to obstacle (starts at 3.0m, decreases)
    dto = np.maximum(0.18, 3.0 - 0.8 * t)
    
    # Commanded velocity: attempts to keep 0.8 m/s until close
    cmd_vx = np.where(t < 2.5, 0.8, 0.75) # fails to decelerate!
    cmd_wz = np.where(t > 2.8, 0.1, 0.0) # weak steering
    
    # Depth zero ratio: spikes when dto < 0.35m
    center_zero_pct = np.where(dto < 0.40, np.clip(100.0 * (0.40 - dto) / 0.20, 0, 100), 2.0)
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp", "rel_time", "robot_x", "robot_y", "robot_z", "robot_yaw",
            "odom_vx", "odom_vy", "odom_wz", "cmd_vx", "cmd_vy", "cmd_wz",
            "goal_x", "goal_y", "goal_dist", "lidar_min_dist_front",
            "lidar_nearest_x", "lidar_nearest_y", "lidar_nearest_z",
            "depth_nan_ratio", "depth_zero_ratio", "depth_center_mean",
            "depth_bottom_mean", "depth_center_zero_pct", "is_emergency_stop", "notes"
        ])
        for i in range(len(t)):
            writer.writerow([
                1700000000.0 + t[i], t[i], x[i], y[i], 0.38, yaw[i],
                cmd_vx[i], 0.0, cmd_wz[i], cmd_vx[i], 0.0, cmd_wz[i],
                5.0, 0.0, 5.0 - x[i], dto[i],
                dto[i], 0.0, 0.1,
                0.01, center_zero_pct[i]/100.0, dto[i],
                dto[i], center_zero_pct[i], 0, "mock_test"
            ])
    print(f"Generated sample log: {csv_path}")

if __name__ == "__main__":
    generate_mock_log("/tmp/sample_benchmark.csv")

#!/usr/bin/env python3
"""
Unitree Go2W Navigation Benchmark Data Collector (ROS 2 Humble / Foxy)
---------------------------------------------------------------------
Real-time logging node for quantitative, standardized evaluation of RL navigation policies.
Collects synchronized multi-modal data:
  1. Odometry & velocities (Point-LIO / Super-LIO / DLIO)
  2. Policy outputs & commanded velocities (/cmd_vel, /cmd_vel_direct)
  3. Depth camera health metrics (NaN ratio, 0.0 clipping ratio, bottom/center ROI depth)
  4. LiDAR ground truth clearance (Distance to Nearest Obstacle - DTO in front corridor)
  5. Active navigation goal & tracking error

Output:
  - Synchronized CSV time-series log (<log_dir>/benchmark_run_<timestamp>.csv)
  - Run metadata summary JSON (<log_dir>/benchmark_run_<timestamp>_meta.json)
"""

import os
import sys
import time
import math
import argparse
import json
import csv
from datetime import datetime

import numpy as np

# ROS 2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import Bool, String
    from cv_bridge import CvBridge
    try:
        from sensor_msgs_py import point_cloud2
    except ImportError:
        point_cloud2 = None
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def quaternion_to_yaw(qx, qy, qz, qw):
    """Compute yaw angle from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class BenchmarkCollectorNode(Node if ROS2_AVAILABLE else object):
    def __init__(self, args):
        super().__init__('benchmark_collector_node')
        self.args = args

        # Logging output setup
        log_dir = os.path.expanduser(args.output_dir)
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, f"benchmark_{args.test_name}_{timestamp}.csv")
        self.meta_path = os.path.join(log_dir, f"benchmark_{args.test_name}_{timestamp}_meta.json")

        self.bridge = CvBridge()
        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # Write CSV Header
        self.csv_writer.writerow([
            "timestamp",              # float (seconds)
            "rel_time",               # float (seconds relative to start)
            "robot_x", "robot_y", "robot_z", "robot_yaw",
            "odom_vx", "odom_vy", "odom_wz",
            "cmd_vx", "cmd_vy", "cmd_wz",
            "goal_x", "goal_y", "goal_dist",
            "lidar_min_dist_front",   # Ground truth minimum distance in front corridor
            "lidar_nearest_x", "lidar_nearest_y", "lidar_nearest_z",
            "depth_nan_ratio",        # Proportion of NaN in depth image
            "depth_zero_ratio",       # Proportion of exact 0.0 in depth image (clipping bug metric)
            "depth_center_mean",      # Mean depth in center 30% bounding box
            "depth_bottom_mean",      # Mean depth in bottom 25% rows (critical for low obstacles)
            "depth_center_zero_pct",  # Percentage of 0.0 in center (close-range blind zone metric)
            "is_emergency_stop",
            "notes"
        ])
        self.csv_file.flush()

        self.start_time = None
        self.latest_odom = None
        self.latest_cmd_vel = Twist()
        self.latest_goal = None
        self.latest_lidar_metrics = {
            "min_dist": 99.0, "x": 0.0, "y": 0.0, "z": 0.0
        }
        self.latest_depth_metrics = {
            "nan_ratio": 0.0, "zero_ratio": 0.0,
            "center_mean": 0.0, "bottom_mean": 0.0, "center_zero_pct": 0.0
        }
        self.is_emergency = False

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        standard_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, standard_qos
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist, args.cmd_vel_topic, self.cmd_vel_callback, standard_qos
        )
        self.sub_depth = self.create_subscription(
            Image, args.depth_topic, self.depth_callback, sensor_qos
        )
        if args.lidar_cloud_topic:
            self.sub_cloud = self.create_subscription(
                PointCloud2, args.lidar_cloud_topic, self.cloud_callback, sensor_qos
            )
        self.sub_goal = self.create_subscription(
            PoseStamped, args.goal_topic, self.goal_callback, standard_qos
        )

        # Periodic logging timer (e.g. 20 Hz)
        timer_period = 1.0 / args.log_freq
        self.log_timer = self.create_timer(timer_period, self.log_tick)

        self.sample_count = 0
        self.get_logger().info(
            f"\033[92m[BenchmarkCollector] Initialized. Logging to:\n  -> {self.csv_path}\033[0m"
        )

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    def goal_callback(self, msg: PoseStamped):
        self.latest_goal = msg

    def depth_callback(self, msg: Image):
        """Analyze depth image health, clipping artifacts, and low/close obstacle coverage."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_np = np.asarray(depth_img, dtype=np.float32)

            total_pixels = depth_np.size
            if total_pixels == 0:
                return

            # NaN & Zero metrics
            nan_mask = np.isnan(depth_np) | np.isinf(depth_np)
            nan_count = np.count_nonzero(nan_mask)
            zero_count = np.count_nonzero(depth_np <= 0.001)

            # Central 30% ROI
            h, w = depth_np.shape
            ch_start, ch_end = int(h * 0.35), int(h * 0.65)
            cw_start, cw_end = int(w * 0.35), int(w * 0.65)
            center_roi = depth_np[ch_start:ch_end, cw_start:cw_end]
            center_valid = center_roi[(center_roi > 0.001) & ~np.isnan(center_roi) & ~np.isinf(center_roi)]
            center_zero_count = np.count_nonzero(center_roi <= 0.001)
            center_zero_pct = (center_zero_count / center_roi.size) * 100.0 if center_roi.size > 0 else 0.0
            center_mean = float(np.mean(center_valid)) if center_valid.size > 0 else 0.0

            # Bottom 25% ROI (Where low obstacles appear or drop out)
            bottom_roi = depth_np[int(h * 0.75):, :]
            bottom_valid = bottom_roi[(bottom_roi > 0.001) & ~np.isnan(bottom_roi) & ~np.isinf(bottom_roi)]
            bottom_mean = float(np.mean(bottom_valid)) if bottom_valid.size > 0 else 0.0

            self.latest_depth_metrics = {
                "nan_ratio": float(nan_count / total_pixels),
                "zero_ratio": float(zero_count / total_pixels),
                "center_mean": center_mean,
                "bottom_mean": bottom_mean,
                "center_zero_pct": float(center_zero_pct)
            }
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def cloud_callback(self, msg: PointCloud2):
        """Extract ground-truth distance to nearest obstacle in front bounding corridor."""
        if point_cloud2 is None:
            return
        try:
            # Corridor definition in robot base frame:
            # X: 0.1m ~ 3.5m ahead
            # Y: -0.55m ~ 0.55m (robot body width clearance)
            # Z: -0.20m ~ 0.60m (above ground to robot upper body)
            min_dist = 99.0
            nearest_pt = (0.0, 0.0, 0.0)

            # Read x, y, z points
            pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            for p in pts:
                px, py, pz = p[0], p[1], p[2]
                if 0.1 <= px <= 3.5 and -0.55 <= py <= 0.55 and -0.20 <= pz <= 0.60:
                    dist = math.sqrt(px * px + py * py)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_pt = (px, py, pz)

            self.latest_lidar_metrics = {
                "min_dist": float(min_dist if min_dist < 90.0 else -1.0),
                "x": float(nearest_pt[0]),
                "y": float(nearest_pt[1]),
                "z": float(nearest_pt[2])
            }
        except Exception as e:
            self.get_logger().warn(f"Error parsing pointcloud: {e}")

    def log_tick(self):
        """Periodic synchronized record trigger."""
        if self.latest_odom is None:
            return

        now = time.time()
        if self.start_time is None:
            self.start_time = now
        rel_time = now - self.start_time

        # Extract robot state from odom
        p = self.latest_odom.pose.pose.position
        q = self.latest_odom.pose.pose.orientation
        v = self.latest_odom.twist.twist.linear
        w = self.latest_odom.twist.twist.angular
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # Target goal distance
        gx, gy, g_dist = 0.0, 0.0, -1.0
        if self.latest_goal is not None:
            gx = self.latest_goal.pose.position.x
            gy = self.latest_goal.pose.position.y
            g_dist = math.sqrt((gx - p.x)**2 + (gy - p.y)**2)

        # Write row
        row = [
            f"{now:.4f}", f"{rel_time:.4f}",
            f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}", f"{yaw:.4f}",
            f"{v.x:.4f}", f"{v.y:.4f}", f"{w.z:.4f}",
            f"{self.latest_cmd_vel.linear.x:.4f}",
            f"{self.latest_cmd_vel.linear.y:.4f}",
            f"{self.latest_cmd_vel.angular.z:.4f}",
            f"{gx:.4f}", f"{gy:.4f}", f"{g_dist:.4f}",
            f"{self.latest_lidar_metrics['min_dist']:.4f}",
            f"{self.latest_lidar_metrics['x']:.4f}",
            f"{self.latest_lidar_metrics['y']:.4f}",
            f"{self.latest_lidar_metrics['z']:.4f}",
            f"{self.latest_depth_metrics['nan_ratio']:.4f}",
            f"{self.latest_depth_metrics['zero_ratio']:.4f}",
            f"{self.latest_depth_metrics['center_mean']:.4f}",
            f"{self.latest_depth_metrics['bottom_mean']:.4f}",
            f"{self.latest_depth_metrics['center_zero_pct']:.2f}",
            int(self.is_emergency),
            self.args.notes
        ]
        self.csv_writer.writerow(row)
        self.sample_count += 1

        if self.sample_count % 40 == 0:
            self.csv_file.flush()
            # Status line in terminal
            d_obs = self.latest_lidar_metrics['min_dist']
            obs_str = f"{d_obs:.2f}m" if d_obs > 0 else "Clear"
            self.get_logger().info(
                f"[T={rel_time:5.1f}s] Pos:({p.x:.2f},{p.y:.2f}) | "
                f"Cmd:[vx={self.latest_cmd_vel.linear.x:.2f}, wz={self.latest_cmd_vel.angular.z:.2f}] | "
                f"DTO_Lidar:{obs_str} | ZeroDepth:{self.latest_depth_metrics['center_zero_pct']:.1f}%"
            )

    def close(self):
        """Save metadata and safely close files."""
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

        meta = {
            "test_name": self.args.test_name,
            "obstacle_height_m": self.args.obstacle_height,
            "obstacle_distance_m": self.args.obstacle_distance,
            "nominal_speed_m_s": self.args.nominal_speed,
            "total_samples": self.sample_count,
            "duration_s": (time.time() - self.start_time) if self.start_time else 0.0,
            "csv_file": self.csv_path,
            "notes": self.args.notes
        }
        with open(self.meta_path, 'w') as f:
            json.dump(meta, f, indent=4)
        print(f"\n[BenchmarkCollector] Finished. Saved {self.sample_count} records to:\n  -> {self.csv_path}\n  -> {self.meta_path}")


def main():
    parser = argparse.ArgumentParser(description="Go2W Real-Robot Benchmark Data Collector")
    parser.add_argument("--test-name", type=str, default="general_eval", help="Test identifier")
    parser.add_argument("--obstacle-height", type=float, default=0.20, help="Obstacle height in meters (e.g. 0.10, 0.20, 0.50)")
    parser.add_argument("--obstacle-distance", type=float, default=3.0, help="Initial distance to obstacle in meters")
    parser.add_argument("--nominal-speed", type=float, default=1.0, help="Nominal speed limit")
    parser.add_argument("--output-dir", type=str, default="~/benchmark_data", help="Output directory for logs")
    parser.add_argument("--log-freq", type=float, default=20.0, help="Logging frequency in Hz")
    parser.add_argument("--odom-topic", type=str, default="/lio/robo/odom", help="Odometry topic")
    parser.add_argument("--cmd-vel-topic", type=str, default="/cmd_vel", help="Command velocity topic")
    parser.add_argument("--depth-topic", type=str, default="/zed/zed_node/depth/depth_registered", help="Depth image topic")
    parser.add_argument("--lidar-cloud-topic", type=str, default="/livox/lidar", help="LiDAR point cloud topic for ground truth clearance")
    parser.add_argument("--goal-topic", type=str, default="/goal", help="Navigation goal topic")
    parser.add_argument("--notes", type=str, default="", help="Additional annotations")

    args = parser.parse_args()

    if not ROS2_AVAILABLE:
        print("ERROR: ROS 2 python libraries (rclpy) not found in this environment. Run with ROS 2 environment sourced.")
        sys.exit(1)

    rclpy.init()
    node = BenchmarkCollectorNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[BenchmarkCollector] Stopping...")
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

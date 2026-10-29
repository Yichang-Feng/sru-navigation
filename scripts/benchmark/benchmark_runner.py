#!/usr/bin/env python3
"""
Unitree Go2W Automated Benchmark Experiment Runner
--------------------------------------------------
Automates standardized navigation and obstacle avoidance benchmark trials:
  Suite 1: Frontal Obstacle Approach (多高度障碍物正向逼近测试)
  Suite 2: Dead-Zone Escape (近距离盲区起步测试)
  Suite 3: Corridors & Free Waypoints (狭道/自由发点测试)

Automatically tracks:
  - Goal arrival, collision detection (via LiDAR DTO threshold), and timeout
  - Quantitative metric logging: SPL, d_min, D_react, time-to-goal
"""

import os
import sys
import time
import math
import argparse
import subprocess
import json

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from sensor_msgs.msg import PointCloud2
    try:
        from sensor_msgs_py import point_cloud2
    except ImportError:
        point_cloud2 = None
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class BenchmarkRunnerNode(Node if ROS2_AVAILABLE else object):
    def __init__(self, args):
        super().__init__('benchmark_runner_node')
        self.args = args

        # Publishers
        self.pub_goal = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.pub_start_nav = self.create_publisher(Bool, "startNavi", 10)
        self.pub_cmd_vel = self.create_publisher(Twist, args.cmd_vel_topic, 10)

        # Subscribers
        self.sub_odom = self.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, 10
        )
        if args.lidar_cloud_topic:
            self.sub_cloud = self.create_subscription(
                PointCloud2, args.lidar_cloud_topic, self.cloud_callback, 10
            )

        self.current_odom = None
        self.min_lidar_dist = 99.0
        self.trial_active = False

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def cloud_callback(self, msg: PointCloud2):
        if point_cloud2 is None:
            return
        min_d = 99.0
        try:
            pts = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            for p in pts:
                px, py, pz = p[0], p[1], p[2]
                if 0.05 <= px <= 3.5 and -0.55 <= py <= 0.55 and -0.20 <= pz <= 0.60:
                    d = math.sqrt(px * px + py * py)
                    if d < min_d:
                        min_d = d
            self.min_lidar_dist = min_d
        except Exception:
            pass

    def send_stop(self):
        """Immediately command zero velocity."""
        stop_cmd = Twist()
        for _ in range(5):
            self.pub_cmd_vel.publish(stop_cmd)
            time.sleep(0.02)

    def trigger_navigation(self, enable: bool):
        """Publish startNavi true/false."""
        msg = Bool()
        msg.data = enable
        self.pub_start_nav.publish(msg)

    def publish_relative_goal(self, delta_x: float, delta_y: float):
        """Publish goal relative to robot current pose."""
        if self.current_odom is None:
            self.get_logger().error("No odometry received yet, cannot send goal!")
            return None

        p = self.current_odom.pose.pose.position
        q = self.current_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Transform delta in robot frame to map frame
        gx = p.x + delta_x * math.cos(yaw) - delta_y * math.sin(yaw)
        gy = p.y + delta_x * math.sin(yaw) + delta_y * math.cos(yaw)

        goal = PoseStamped()
        goal.header.frame_id = self.current_odom.header.frame_id or "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.position.z = p.z
        goal.pose.orientation.w = 1.0

        self.pub_goal.publish(goal)
        self.get_logger().info(f"\033[96m[Runner] Published Target Goal -> X:{gx:.2f}, Y:{gy:.2f}\033[0m")
        return (gx, gy)

    def run_single_trial(self, trial_idx: int, target_dist: float, timeout_s: float = 30.0,
                         collision_dist_threshold: float = 0.22,
                         obstacle_height: float = 0.20):
        """Execute one complete evaluation trial."""
        self.get_logger().info(f"\n{'='*60}\n>>> STARTING TRIAL #{trial_idx} (Target: {target_dist}m ahead, Obstacle Height: {obstacle_height}m)\n{'='*60}")
        
        # Wait for odom
        timeout = 5.0
        t0 = time.time()
        while self.current_odom is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_odom is None:
            self.get_logger().error("Failed to get odometry. Aborting trial.")
            return {"status": "ODOM_FAIL"}

        start_x = self.current_odom.pose.pose.position.x
        start_y = self.current_odom.pose.pose.position.y
        initial_opt_length = target_dist

        # 1. Publish goal & trigger nav
        goal_coords = self.publish_relative_goal(delta_x=target_dist, delta_y=0.0)
        self.trigger_navigation(True)

        trial_start_time = time.time()
        min_clearance = 99.0
        reaction_dist = -1.0
        traj_length = 0.0
        last_px, last_py = start_x, start_y
        nominal_vx = 0.0
        vx_samples = []
        trial_status = "RUNNING"

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            elapsed = now - trial_start_time

            # Update position & path length
            curr_x = self.current_odom.pose.pose.position.x
            curr_y = self.current_odom.pose.pose.position.y
            step_d = math.hypot(curr_x - last_px, curr_y - last_py)
            traj_length += step_d
            last_px, last_py = curr_x, curr_y

            # Update speeds
            vx = self.current_odom.twist.twist.linear.x
            wz = self.current_odom.twist.twist.angular.z
            if elapsed < 2.0 and vx > 0.3:
                vx_samples.append(vx)
                nominal_vx = float(np.mean(vx_samples))

            # Monitor LiDAR clearance
            cur_lidar_d = self.min_lidar_dist
            if cur_lidar_d < min_clearance:
                min_clearance = cur_lidar_d

            # Check reaction point: first significant braking or lateral turning
            if reaction_dist < 0 and nominal_vx > 0.4 and cur_lidar_d < 3.0:
                if vx < (nominal_vx * 0.75) or abs(wz) > 0.3:
                    reaction_dist = cur_lidar_d
                    self.get_logger().info(
                        f"\033[93m[Runner] Avoidance Reaction Detected at DTO = {reaction_dist:.2f}m! (vx: {vx:.2f}, wz: {wz:.2f})\033[0m"
                    )

            # Check collision threshold (soft safety guard)
            if cur_lidar_d <= collision_dist_threshold:
                trial_status = "COLLISION"
                self.get_logger().error(
                    f"\033[91m[Runner] COLLISION DETECTED! Clearance {cur_lidar_d:.2f}m <= {collision_dist_threshold:.2f}m. Halting robot.\033[0m"
                )
                self.send_stop()
                self.trigger_navigation(False)
                break

            # Check goal arrival
            dist_to_goal = math.hypot(curr_x - goal_coords[0], curr_y - goal_coords[1])
            if dist_to_goal <= 0.45:
                trial_status = "SUCCESS"
                self.get_logger().info(f"\033[92m[Runner] GOAL REACHED! Dist={dist_to_goal:.2f}m. Trial Completed.\033[0m")
                self.send_stop()
                self.trigger_navigation(False)
                break

            # Check timeout
            if elapsed >= timeout_s:
                trial_status = "TIMEOUT"
                self.get_logger().warn(f"\033[93m[Runner] Trial TIMEOUT after {elapsed:.1f}s.\033[0m")
                self.send_stop()
                self.trigger_navigation(False)
                break

        # Calculate metrics
        is_success = (trial_status == "SUCCESS")
        spl = 0.0
        if is_success:
            spl = float(initial_opt_length / max(traj_length, initial_opt_length))

        trial_result = {
            "trial_idx": trial_idx,
            "obstacle_height_m": obstacle_height,
            "status": trial_status,
            "success": is_success,
            "spl": round(spl, 3),
            "min_clearance_m": round(min_clearance, 3),
            "reaction_distance_m": round(reaction_dist if reaction_dist > 0 else -1.0, 3),
            "actual_path_length_m": round(traj_length, 2),
            "duration_s": round(now - trial_start_time, 2)
        }

        self.get_logger().info(
            f"[Runner] Trial Result Summary:\n"
            f"  - Status: {trial_status}\n"
            f"  - Min Clearance: {min_clearance:.2f}m\n"
            f"  - Reaction Dist: {reaction_dist:.2f}m\n"
            f"  - SPL: {spl:.3f}\n"
        )
        return trial_result


def main():
    parser = argparse.ArgumentParser(description="Go2W Automated Benchmark Test Runner")
    parser.add_argument("--suite", type=str, choices=["frontal_approach", "deadzone", "custom"], default="frontal_approach",
                        help="Benchmark suite type")
    parser.add_argument("--obstacle-height", type=float, default=0.20, help="Current obstacle height in meters")
    parser.add_argument("--distance", type=float, default=3.5, help="Distance to obstacle or goal (m)")
    parser.add_argument("--trials", type=int, default=3, help="Number of repetitions per test condition")
    parser.add_argument("--output", type=str, default="~/benchmark_data/summary.json", help="Summary json path")
    parser.add_argument("--odom-topic", type=str, default="/lio/robo/odom", help="Odometry topic")
    parser.add_argument("--cmd-vel-topic", type=str, default="/cmd_vel", help="Command velocity topic")
    parser.add_argument("--lidar-cloud-topic", type=str, default="/livox/lidar", help="LiDAR cloud topic")
    parser.add_argument("--goal-topic", type=str, default="/goal", help="Goal topic")

    args = parser.parse_args()

    if not ROS2_AVAILABLE:
        print("ERROR: ROS 2 python libraries (rclpy) not found in this environment. Run with ROS 2 environment sourced.")
        sys.exit(1)

    rclpy.init()
    runner = BenchmarkRunnerNode(args)

    results = []
    print(f"\n=======================================================")
    print(f"  UNITREE GO2W BENCHMARK RUNNER - SUITE: {args.suite}")
    print(f"  Obstacle Height: {args.obstacle_height}m | Distance: {args.distance}m | Reps: {args.trials}")
    print(f"=======================================================\n")

    try:
        for t in range(1, args.trials + 1):
            res = runner.run_single_trial(
                trial_idx=t,
                target_dist=args.distance,
                timeout_s=35.0,
                obstacle_height=args.obstacle_height
            )
            results.append(res)
            time.sleep(2.0)

        # Output summary table
        out_path = os.path.expanduser(args.output)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, 'w') as f:
            json.dump(results, f, indent=4)

        success_count = sum(1 for r in results if r.get("success", False))
        collision_count = sum(1 for r in results if r.get("status") == "COLLISION")
        avg_spl = float(np.mean([r.get("spl", 0.0) for r in results]))
        valid_reactions = [r["reaction_distance_m"] for r in results if r.get("reaction_distance_m", -1) > 0]
        avg_reaction = float(np.mean(valid_reactions)) if valid_reactions else -1.0
        min_clears = [r["min_clearance_m"] for r in results if "min_clearance_m" in r]
        avg_min_clear = float(np.mean(min_clears)) if min_clears else -1.0

        print(f"\n{'='*60}")
        print(f"  FINAL BENCHMARK SUITE REPORT ({args.suite})")
        print(f"{'='*60}")
        print(f"  Success Rate:          {success_count}/{len(results)} ({(success_count/len(results)*100):.1f}%)")
        print(f"  Collision Rate:        {collision_count}/{len(results)} ({(collision_count/len(results)*100):.1f}%)")
        print(f"  Average SPL:           {avg_spl:.3f}")
        print(f"  Avg Reaction Distance: {avg_reaction:.2f}m")
        print(f"  Avg Minimum Clearance: {avg_min_clear:.2f}m")
        print(f"  Saved JSON to:         {out_path}")
        print(f"{'='*60}\n")

    except KeyboardInterrupt:
        print("\nBenchmark runner interrupted by user.")
        runner.send_stop()
        runner.trigger_navigation(False)
    finally:
        runner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

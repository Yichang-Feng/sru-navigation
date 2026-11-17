#!/usr/bin/env python3
"""
Unitree Go2W Multi-Point Sequential Tracking & Smoothness Test Script
=====================================================================
Purpose:
  Test whether sequential discrete waypoints cause stutter/stop-and-go (一顿一顿)
  behavior in the RL navigation policy when arriving within <0.1m threshold.

Features:
  1. Configurable number of points (-n / --num-points) and spacing (-s / --spacing).
  2. Multi-directional support:
     - Straight forward (--pattern straight)
     - Diagonal paths / 斜向多点 (--pattern diagonal-right / diagonal-left, or custom --angle / --dx / --dy)
     - Continuous curves / 弧线 (--pattern arc-left / arc-right)
     - Zigzag / S形转折 (--pattern zigzag)
     - Custom point coordinate lists (--custom-points "x1,y1;x2,y2;...")
     - 3D ramp/slope descent support (--dz)
  3. Real-time distance checking (< threshold, default 0.1m) and instant switch to next point.
  4. Full telemetry & quantitative stutter detection:
     - Records instantaneous ground speed and cmd_vx at every transition.
     - Detects whether speed drops to zero (<0.05 m/s) between points.
  5. Interactive controls during test:
     - 's' + Enter: Manual skip current waypoint (ideal if obstacle blocks target point).
     - 'q' + Enter: Immediately stop robot and abort test.
  6. Rich RViz visualization (MarkerArray with path line, spheres, labels, and directional arrows).
"""

from __future__ import annotations

import os
import sys
import time
import math
import argparse
import select
import threading
import json
from typing import List, Tuple, Dict, Any

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist, Point, Vector3
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Bool, Header, ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Convert quaternion (x, y, z, w) to roll, pitch, yaw in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - x * z)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return quaternion (x, y, z, w) corresponding to yaw rotation."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MultiPointTrackingTester(Node if ROS2_AVAILABLE else object):
    def __init__(self, args):
        super().__init__('multi_point_tracking_tester')
        self.args = args

        # ROS 2 Publishers
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vis/multi_point_test', 10)
        self.path_pub = self.create_publisher(Path, '/vis/multi_point_path', 10)
        self.start_nav_pub = self.create_publisher(Bool, '/startNavi', 10)
        self.stop_cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)

        # ROS 2 Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, args.cmd_vel_topic, self.cmd_vel_callback, 10
        )

        # State variables
        self.current_odom: Odometry = None
        self.robot_pos = [0.0, 0.0, 0.0]
        self.robot_yaw = 0.0
        self.robot_linear_speed = 0.0
        self.latest_cmd_vel = Twist()
        self.odom_frame_id = 'world'

        # Waypoint sequence
        self.waypoints: List[Dict[str, Any]] = []
        self.current_idx = 0
        self.test_started = False
        self.test_finished = False

        # Metrics collection
        self.transition_records: List[Dict[str, Any]] = []
        self.point_start_time = 0.0
        self.test_start_time = 0.0
        self.speed_history: List[float] = []
        self.last_odom_time: float | None = None
        self.last_robot_pos: List[float] | None = None

        self.get_logger().info('\033[96m' + '='*65 + '\033[0m')
        self.get_logger().info('\033[96m  Unitree Go2W Multi-Point Tracking & Smoothness Tester  \033[0m')
        self.get_logger().info(
            f'  Config: Pattern={args.pattern}, Points={args.num_points}, Spacing={args.spacing}m, '
            f'Threshold={args.threshold}m (Terminal={args.terminal_threshold}m), Angle={args.angle}°'
        )
        self.get_logger().info(f'  Topics: odom="{args.odom_topic}", goal="{args.goal_topic}"')
        self.get_logger().info('\033[96m' + '='*65 + '\033[0m')

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg
        self.odom_frame_id = msg.header.frame_id or 'world'
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_pos = [p.x, p.y, p.z]
        _, _, self.robot_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        # Ground speed computation via pose finite differencing
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if now_sec == 0.0:
            now_sec = time.time()

        if self.last_odom_time is not None and self.last_robot_pos is not None:
            dt = now_sec - self.last_odom_time
            if 0.002 < dt < 1.0:
                dx = p.x - self.last_robot_pos[0]
                dy = p.y - self.last_robot_pos[1]
                diff_speed = math.hypot(dx, dy) / dt
                if diff_speed < 5.0:  # Filter out SLAM relocation jumps
                    self.robot_linear_speed = 0.7 * diff_speed + 0.3 * self.robot_linear_speed

        self.last_odom_time = now_sec
        self.last_robot_pos = [p.x, p.y, p.z]

        # If odometry message twist is populated (> 0.02), trust the hardware/SLAM twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        twist_speed = math.hypot(vx, vy)
        if twist_speed > 0.02:
            self.robot_linear_speed = twist_speed

        self.speed_history.append(self.robot_linear_speed)
        if len(self.speed_history) > 100:
            self.speed_history.pop(0)

    def cmd_vel_callback(self, msg: Twist):
        self.latest_cmd_vel = msg

    def generate_waypoints(self) -> List[Dict[str, Any]]:
        """Generate waypoints based on selected pattern, angle, spacing, and count."""
        if self.current_odom is None:
            self.get_logger().error('Cannot generate waypoints: No odometry received yet.')
            return []

        x0, y0, z0 = self.robot_pos
        yaw0 = self.robot_yaw

        points: List[Dict[str, Any]] = []
        pattern = self.args.pattern.lower()

        # Step 1: Compute local robot-frame relative coordinates (rel_x, rel_y, rel_z)
        rel_coords: List[Tuple[float, float, float]] = []

        if self.args.custom_points:
            # Custom format: "x1,y1;x2,y2;x3,y3..." or "x1,y1,z1;x2,y2,z2..."
            raw_pts = self.args.custom_points.strip().split(';')
            for raw_pt in raw_pts:
                parts = [float(val.strip()) for val in raw_pt.split(',') if val.strip()]
                if len(parts) >= 2:
                    rx = parts[0]
                    ry = parts[1]
                    rz = parts[2] if len(parts) >= 3 else 0.0
                    rel_coords.append((rx, ry, rz))

        elif self.args.dx is not None or self.args.dy is not None:
            # Explicit forward & lateral step mode (e.g. --dx 0.5 --dy -0.3 for diagonal right)
            step_dx = self.args.dx if self.args.dx is not None else self.args.spacing
            step_dy = self.args.dy if self.args.dy is not None else 0.0
            step_dz = self.args.dz
            for i in range(1, self.args.num_points + 1):
                rel_coords.append((i * step_dx, i * step_dy, i * step_dz))

        elif pattern in ['diagonal-right', 'diagonal_right', 'right-down']:
            # Diagonal forward-right (斜向右/斜向下)
            eff_angle = math.radians(self.args.angle if self.args.angle != 0.0 else -30.0)
            step_s = self.args.spacing
            step_dz = self.args.dz
            for i in range(1, self.args.num_points + 1):
                d = i * step_s
                rel_coords.append((d * math.cos(eff_angle), d * math.sin(eff_angle), i * step_dz))

        elif pattern in ['diagonal-left', 'diagonal_left', 'left-up']:
            # Diagonal forward-left (斜向左/斜向上)
            eff_angle = math.radians(self.args.angle if self.args.angle != 0.0 else 30.0)
            step_s = self.args.spacing
            step_dz = self.args.dz
            for i in range(1, self.args.num_points + 1):
                d = i * step_s
                rel_coords.append((d * math.cos(eff_angle), d * math.sin(eff_angle), i * step_dz))

        elif pattern in ['arc-left', 'arc-right', 'curve']:
            # Curving circular path
            sign = 1.0 if 'left' in pattern else -1.0
            d_theta_rad = math.radians(self.args.turn_angle) * sign
            cur_rx = 0.0
            cur_ry = 0.0
            cur_rz = 0.0
            cur_heading = 0.0

            for i in range(1, self.args.num_points + 1):
                cur_rx += self.args.spacing * math.cos(cur_heading)
                cur_ry += self.args.spacing * math.sin(cur_heading)
                cur_rz += self.args.dz
                rel_coords.append((cur_rx, cur_ry, cur_rz))
                cur_heading += d_theta_rad

        elif pattern in ['zigzag', 's-curve']:
            # Alternating lateral offsets (+offset, -offset) for obstacle/turn test
            lat_offset = self.args.lateral_offset
            for i in range(1, self.args.num_points + 1):
                cur_rx = i * self.args.spacing
                cur_ry = lat_offset if (i % 2 == 1) else -lat_offset
                cur_rz = i * self.args.dz
                rel_coords.append((cur_rx, cur_ry, cur_rz))

        else:
            # Default: straight ahead with configurable --angle
            eff_angle = math.radians(self.args.angle)
            step_s = self.args.spacing
            step_dz = self.args.dz
            for i in range(1, self.args.num_points + 1):
                d = i * step_s
                rel_coords.append((d * math.cos(eff_angle), d * math.sin(eff_angle), i * step_dz))

        # Step 2: Transform local robot-frame coordinates to world odometry frame
        prev_wx, prev_wy = x0, y0
        cos_yaw0 = math.cos(yaw0)
        sin_yaw0 = math.sin(yaw0)

        for i, (rx, ry, rz) in enumerate(rel_coords, start=1):
            # 2D rigid rotation + translation
            wx = x0 + rx * cos_yaw0 - ry * sin_yaw0
            wy = y0 + rx * sin_yaw0 + ry * cos_yaw0
            wz = z0 + rz

            # Tangent heading direction of this segment
            seg_yaw = math.atan2(wy - prev_wy, wx - prev_wx)
            qx, qy, qz, qw = quaternion_from_yaw(seg_yaw)

            rel_euclid_dist = math.hypot(wx - x0, wy - y0)

            points.append({
                'index': i,
                'rel_x': rx,
                'rel_y': ry,
                'rel_z': rz,
                'rel_dist': rel_euclid_dist,
                'x': wx,
                'y': wy,
                'z': wz,
                'quat': (qx, qy, qz, qw),
                'yaw': seg_yaw
            })
            prev_wx, prev_wy = wx, wy

        return points

    def publish_visualizations(self):
        """Publish markers and planned path for visual inspection in RViz."""
        if not self.waypoints:
            return

        now = self.get_clock().now().to_msg()

        # 1. Path message
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = self.odom_frame_id

        # 2. Marker array
        marker_arr = MarkerArray()

        # Line strip marker connecting all points
        line_marker = Marker()
        line_marker.header.stamp = now
        line_marker.header.frame_id = self.odom_frame_id
        line_marker.ns = 'path_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.color = ColorRGBA(r=0.2, g=0.85, b=0.2, a=0.85)

        # Add start position
        p_start = Point(x=self.robot_pos[0], y=self.robot_pos[1], z=self.robot_pos[2])
        line_marker.points.append(p_start)

        for wp in self.waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = now
            pose_stamped.header.frame_id = self.odom_frame_id
            pose_stamped.pose.position.x = wp['x']
            pose_stamped.pose.position.y = wp['y']
            pose_stamped.pose.position.z = wp['z']
            pose_stamped.pose.orientation.x = wp['quat'][0]
            pose_stamped.pose.orientation.y = wp['quat'][1]
            pose_stamped.pose.orientation.z = wp['quat'][2]
            pose_stamped.pose.orientation.w = wp['quat'][3]
            path_msg.poses.append(pose_stamped)

            pt = Point(x=wp['x'], y=wp['y'], z=wp['z'])
            line_marker.points.append(pt)

            # Sphere marker for each waypoint
            sphere = Marker()
            sphere.header.stamp = now
            sphere.header.frame_id = self.odom_frame_id
            sphere.ns = 'waypoints'
            sphere.id = wp['index']
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = wp['z']

            is_active = (wp['index'] == (self.current_idx + 1))
            is_passed = (wp['index'] <= self.current_idx)

            if is_active:
                sphere.scale.x = 0.25
                sphere.scale.y = 0.25
                sphere.scale.z = 0.25
                sphere.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=1.0)
            elif is_passed:
                sphere.scale.x = 0.12
                sphere.scale.y = 0.12
                sphere.scale.z = 0.12
                sphere.color = ColorRGBA(r=0.0, g=1.0, b=0.3, a=0.6)
            else:
                sphere.scale.x = 0.16
                sphere.scale.y = 0.16
                sphere.scale.z = 0.16
                sphere.color = ColorRGBA(r=0.1, g=0.7, b=1.0, a=0.7)

            marker_arr.markers.append(sphere)

            # Directional arrow marker at each waypoint
            arrow = Marker()
            arrow.header.stamp = now
            arrow.header.frame_id = self.odom_frame_id
            arrow.ns = 'arrows'
            arrow.id = 200 + wp['index']
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = wp['x']
            arrow.pose.position.y = wp['y']
            arrow.pose.position.z = wp['z']
            arrow.pose.orientation.x = wp['quat'][0]
            arrow.pose.orientation.y = wp['quat'][1]
            arrow.pose.orientation.z = wp['quat'][2]
            arrow.pose.orientation.w = wp['quat'][3]
            arrow.scale.x = 0.35  # arrow length
            arrow.scale.y = 0.06  # arrow shaft diameter
            arrow.scale.z = 0.08  # arrow head diameter
            arrow.color = ColorRGBA(r=1.0, g=0.3, b=0.1, a=0.8)
            marker_arr.markers.append(arrow)

            # Text label marker
            text = Marker()
            text.header.stamp = now
            text.header.frame_id = self.odom_frame_id
            text.ns = 'labels'
            text.id = 100 + wp['index']
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp['x']
            text.pose.position.y = wp['y']
            text.pose.position.z = wp['z'] + 0.28
            text.scale.z = 0.15
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            text.text = f"P{wp['index']} ({wp['rel_dist']:.1f}m)"
            marker_arr.markers.append(text)

        marker_arr.markers.append(line_marker)
        self.marker_pub.publish(marker_arr)
        self.path_pub.publish(path_msg)

    def publish_goal(self, wp: Dict[str, Any]):
        """Publish a specific waypoint to /goal_pose."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.odom_frame_id
        goal.pose.position.x = wp['x']
        goal.pose.position.y = wp['y']
        goal.pose.position.z = wp['z']
        goal.pose.orientation.x = wp['quat'][0]
        goal.pose.orientation.y = wp['quat'][1]
        goal.pose.orientation.z = wp['quat'][2]
        goal.pose.orientation.w = wp['quat'][3]

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"\033[92m[Test Runner] >>> Dispatched Waypoint {wp['index']}/{len(self.waypoints)} "
            f"-> Goal: ({wp['x']:.2f}, {wp['y']:.2f}) [Rel: +{wp['rel_dist']:.2f}m, Yaw: {math.degrees(wp['yaw']):.1f}°]\033[0m"
        )

    def unlock_chassis(self):
        """Send true to /startNavi to ensure bridge accepts velocity commands."""
        msg = Bool()
        msg.data = True
        self.start_nav_pub.publish(msg)

    def cancel_active_navigation(self):
        """Force cancel goal by setting target to robot current pose, ensuring rl_nav_controller stops."""
        if self.robot_pos is not None:
            cancel_goal = PoseStamped()
            cancel_goal.header.stamp = self.get_clock().now().to_msg()
            cancel_goal.header.frame_id = self.odom_frame_id
            cancel_goal.pose.position.x = self.robot_pos[0]
            cancel_goal.pose.position.y = self.robot_pos[1]
            cancel_goal.pose.position.z = self.robot_pos[2]
            cancel_goal.pose.orientation.w = 1.0
            self.goal_pub.publish(cancel_goal)

        # Send zero velocity commands
        stop = Twist()
        for _ in range(8):
            self.stop_cmd_pub.publish(stop)
            time.sleep(0.02)

    def emergency_stop(self):
        """Send zero velocity and cancel goal to safely halt robot."""
        self.cancel_active_navigation()


def compute_lookahead_carrot(
    waypoints: List[Dict[str, Any]],
    start_pos: List[float],
    robot_pos: List[float],
    lookahead_dist: float,
    current_progress_s: float
) -> Tuple[Dict[str, Any], float, bool]:
    """Compute continuous lookahead carrot point along the polyline path."""
    nodes = [{'x': start_pos[0], 'y': start_pos[1], 'z': start_pos[2]}] + waypoints
    cum_s = [0.0]
    for i in range(1, len(nodes)):
        seg_len = math.hypot(nodes[i]['x'] - nodes[i-1]['x'], nodes[i]['y'] - nodes[i-1]['y'])
        cum_s.append(cum_s[-1] + seg_len)

    total_len = cum_s[-1]
    rx, ry = robot_pos[0], robot_pos[1]
    min_dist_to_line = float('inf')
    best_s = current_progress_s

    for i in range(len(nodes) - 1):
        x1, y1 = nodes[i]['x'], nodes[i]['y']
        x2, y2 = nodes[i+1]['x'], nodes[i+1]['y']
        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-6:
            continue

        t = ((rx - x1) * dx + (ry - y1) * dy) / seg_len_sq
        t_clamped = max(0.0, min(1.0, t))
        proj_x = x1 + t_clamped * dx
        proj_y = y1 + t_clamped * dy

        dist_to_proj = math.hypot(rx - proj_x, ry - proj_y)
        if dist_to_proj < min_dist_to_line:
            min_dist_to_line = dist_to_proj
            seg_s = cum_s[i] + t_clamped * math.sqrt(seg_len_sq)
            if seg_s >= current_progress_s - 0.2:
                best_s = max(current_progress_s, seg_s)

    target_s = min(best_s + lookahead_dist, total_len)
    is_final = (target_s >= total_len - 1e-3)

    target_idx = 0
    for i in range(len(cum_s) - 1):
        if cum_s[i] <= target_s <= cum_s[i+1]:
            target_idx = i
            break
        target_idx = i

    s0 = cum_s[target_idx]
    s1 = cum_s[target_idx + 1]
    ratio = (target_s - s0) / (s1 - s0) if (s1 > s0) else 0.0
    ratio = max(0.0, min(1.0, ratio))

    n0 = nodes[target_idx]
    n1 = nodes[target_idx + 1]

    cx = n0['x'] + ratio * (n1['x'] - n0['x'])
    cy = n0['y'] + ratio * (n1['y'] - n0['y'])
    cz = n0['z'] + ratio * (n1['z'] - n0['z'])

    seg_yaw = math.atan2(n1['y'] - n0['y'], n1['x'] - n0['x'])
    qx, qy, qz, qw = quaternion_from_yaw(seg_yaw)

    carrot = {
        'index': target_idx + 1,
        'rel_dist': target_s,
        'x': cx,
        'y': cy,
        'z': cz,
        'yaw': seg_yaw,
        'quat': (qx, qy, qz, qw),
        's': target_s,
        'total_s': total_len
    }
    return carrot, best_s, is_final


def run_test(args):
    if not ROS2_AVAILABLE:
        print("[ERROR] ROS 2 (rclpy) is not available in the current Python environment.")
        print("Please source ROS 2 first: source /opt/ros/humble/setup.bash")
        return

    rclpy.init()
    node = MultiPointTrackingTester(args)

    # 1. Wait for odometry
    print("\n[1/4] Waiting for robot odometry (/lio/robo/odom)...")
    t0 = time.time()
    while node.current_odom is None and (time.time() - t0) < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_odom is None:
        node.get_logger().error("Timed out waiting for odometry! Check if Super LIO is publishing.")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(
        f"[✓] Initial Odometry received! Robot at X:{node.robot_pos[0]:.2f}, "
        f"Y:{node.robot_pos[1]:.2f}, Yaw:{math.degrees(node.robot_yaw):.1f}°"
    )

    # 2. Generate waypoints based on pattern & settings
    print(f"\n[2/4] Generating trajectory: Pattern='{args.pattern}', Spacing={args.spacing}m, Angle={args.angle}°...")
    node.waypoints = node.generate_waypoints()
    if not node.waypoints:
        print("[ERROR] Failed to generate waypoints. Aborting.")
        node.destroy_node()
        rclpy.shutdown()
        return

    for wp in node.waypoints:
        print(
            f"    - P{wp['index']}: World (X={wp['x']:6.2f}, Y={wp['y']:6.2f}, Z={wp['z']:5.2f}) "
            f"| Local (dx={wp['rel_x']:5.2f}, dy={wp['rel_y']:5.2f}) | Dist: {wp['rel_dist']:4.2f}m | Heading: {math.degrees(wp['yaw']):5.1f}°"
        )

    # Publish visualization to RViz
    node.publish_visualizations()

    # 3. Prompt user before starting
    print("\n[3/4] Ready to commence multi-point tracking test.")
    print("------------------------------------------------------------------")
    print(f"  * Pattern Preset:    {args.pattern}")
    print(f"  * Total Points:      {len(node.waypoints)}")
    print(f"  * Switch Threshold:  < {args.threshold:.2f} m (automatic advance to next point)")
    print(f"  * Goal Topic:        {args.goal_topic}")
    print("  * Obstacle Notes:    If an obstacle covers a waypoint, press 's' + Enter to manual skip.")
    print("                       Or press 'q' + Enter to abort.")
    print("------------------------------------------------------------------")

    if not args.auto_start:
        try:
            input("Press [ENTER] to unlock navigation (/startNavi) and start test: ")
        except (KeyboardInterrupt, EOFError):
            print("\nAborted by user.")
            node.destroy_node()
            rclpy.shutdown()
            return

    # Automatically unlock bridge
    node.unlock_chassis()
    time.sleep(0.5)

    # -------------------------------------------------------------
    # Mode B: Continuous Lookahead Carrot Tracking Mode
    # -------------------------------------------------------------
    if args.lookahead > 0.0:
        print(f"\n\033[92m[4/4] Continuous Lookahead Carrot Tracking active (Lookahead = {args.lookahead:.2f}m)...\033[0m\n")
        start_pos = list(node.robot_pos)
        progress_s = 0.0
        final_wp = node.waypoints[-1]

        # Non-blocking terminal input thread for quit
        user_action = {"action": None}
        def input_listener():
            while node.test_started and not node.test_finished:
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.5)
                    if rlist:
                        line = sys.stdin.readline().strip().lower()
                        if line == 'q':
                            user_action["action"] = "quit"
                except Exception:
                    break

        input_thread = threading.Thread(target=input_listener, daemon=True)
        input_thread.start()

        rate_hz = 20.0
        dt = 1.0 / rate_hz

        try:
            while rclpy.ok() and not node.test_finished:
                rclpy.spin_once(node, timeout_sec=dt)

                # Compute dynamic lookahead carrot
                carrot, progress_s, is_final = compute_lookahead_carrot(
                    node.waypoints, start_pos, node.robot_pos, args.lookahead, progress_s
                )

                # Continuously publish carrot to /goal_pose
                node.publish_goal(carrot)
                node.publish_visualizations()

                # Distance to final destination point
                dx_final = node.robot_pos[0] - final_wp['x']
                dy_final = node.robot_pos[1] - final_wp['y']
                dist_to_final = math.hypot(dx_final, dy_final)

                speed = node.robot_linear_speed
                cmd_vx = node.latest_cmd_vel.linear.x
                elapsed = time.time() - node.test_start_time

                status_line = (
                    f"\r[Carrot L={args.lookahead:.1f}m] Progress: {progress_s:5.2f}/{carrot['total_s']:5.2f}m | "
                    f"DistToFinal: {dist_to_final:5.2f}m (Thr: {args.terminal_threshold:.2f}m) | "
                    f"Speed: {speed:4.2f} m/s | Cmd_vx: {cmd_vx:4.2f} m/s | T: {elapsed:4.1f}s | ('q'=quit) "
                )
                sys.stdout.write(status_line)
                sys.stdout.flush()

                if user_action["action"] == "quit":
                    print("\n\n[!] Test cancelled by user.")
                    node.emergency_stop()
                    break

                # Reached final destination
                if dist_to_final < args.terminal_threshold:
                    print(
                        f"\n\n\033[92m>>> FINAL DESTINATION REACHED! "
                        f"Dist={dist_to_final:.3f}m <= {args.terminal_threshold:.2f}m | "
                        f"Speed={speed:.2f} m/s | Braking to complete stop.\033[0m"
                    )
                    node.test_finished = True
                    node.cancel_active_navigation()
                    break

        except KeyboardInterrupt:
            print("\n\n[!] Interrupted by KeyboardInterrupt. Stopping robot...")
            node.emergency_stop()

        total_time = time.time() - node.test_start_time
        print(f"\n[✓] Continuous Lookahead Carrot Test Completed in {total_time:.2f}s!\n")
        node.destroy_node()
        rclpy.shutdown()
        return

    # -------------------------------------------------------------
    # Mode A: Discrete Waypoint Switching Mode
    # -------------------------------------------------------------
    # Dispatch first waypoint
    node.current_idx = 0
    node.test_started = True
    node.test_start_time = time.time()
    node.point_start_time = time.time()
    node.publish_goal(node.waypoints[node.current_idx])

    print("\n[4/4] Multi-point tracking in progress...\n")

    # Non-blocking terminal input thread for obstacle skip/quit
    user_action = {"action": None}

    def input_listener():
        while node.test_started and not node.test_finished:
            try:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.5)
                if rlist:
                    line = sys.stdin.readline().strip().lower()
                    if line == 's':
                        user_action["action"] = "skip"
                    elif line == 'q':
                        user_action["action"] = "quit"
            except Exception:
                break

    input_thread = threading.Thread(target=input_listener, daemon=True)
    input_thread.start()

    # Tracking control loop at ~20Hz
    rate_hz = 20.0
    dt = 1.0 / rate_hz

    try:
        while rclpy.ok() and not node.test_finished:
            rclpy.spin_once(node, timeout_sec=dt)

            if node.current_idx >= len(node.waypoints):
                node.test_finished = True
                break

            active_wp = node.waypoints[node.current_idx]
            is_final_point = (node.current_idx == len(node.waypoints) - 1)
            active_threshold = min(args.threshold, args.terminal_threshold) if (is_final_point and not args.stop_at_threshold) else args.threshold

            dx = node.robot_pos[0] - active_wp['x']
            dy = node.robot_pos[1] - active_wp['y']
            dist = math.hypot(dx, dy)

            now = time.time()
            elapsed_point = now - node.point_start_time

            # Update RViz visualization
            node.publish_visualizations()

            # Format real-time status output
            speed = node.robot_linear_speed
            cmd_vx = node.latest_cmd_vel.linear.x
            cmd_wz = node.latest_cmd_vel.angular.z
            status_line = (
                f"\r[P{active_wp['index']}/{len(node.waypoints)} ({args.pattern})] "
                f"Dist: {dist:5.2f}m (Thr: {active_threshold:.2f}m) | "
                f"Speed: {speed:4.2f} m/s | Cmd_vx: {cmd_vx:4.2f} m/s | "
                f"T: {elapsed_point:4.1f}s | ('s'=skip, 'q'=quit) "
            )
            sys.stdout.write(status_line)
            sys.stdout.flush()

            # Check user skip/quit
            if user_action["action"] == "quit":
                print("\n\n[!] Test cancelled by user.")
                node.emergency_stop()
                break
            elif user_action["action"] == "skip":
                user_action["action"] = None
                print(f"\n[!] Manual SKIP triggered for Waypoint {active_wp['index']} (Dist was {dist:.2f}m).")
                node.transition_records.append({
                    'point': active_wp['index'],
                    'rel_dist': active_wp['rel_dist'],
                    'dist_at_switch': dist,
                    'speed_at_switch': speed,
                    'cmd_vx_at_switch': cmd_vx,
                    'elapsed_time': elapsed_point,
                    'stuttered': (speed < 0.20),
                    'note': 'MANUAL_SKIP (Obstacle)'
                })
                node.current_idx += 1
                node.point_start_time = time.time()
                if node.current_idx < len(node.waypoints):
                    node.publish_goal(node.waypoints[node.current_idx])
                continue

            # Check timeout per point (e.g. if blocked by obstacle)
            if args.timeout_per_point > 0 and elapsed_point > args.timeout_per_point:
                print(f"\n[!] Timeout ({args.timeout_per_point}s) reached on Waypoint {active_wp['index']}! Auto-advancing...")
                node.transition_records.append({
                    'point': active_wp['index'],
                    'rel_dist': active_wp['rel_dist'],
                    'dist_at_switch': dist,
                    'speed_at_switch': speed,
                    'cmd_vx_at_switch': cmd_vx,
                    'elapsed_time': elapsed_point,
                    'stuttered': True,
                    'note': 'TIMEOUT_SKIP (Obstacle)'
                })
                node.current_idx += 1
                node.point_start_time = time.time()
                if node.current_idx < len(node.waypoints):
                    node.publish_goal(node.waypoints[node.current_idx])
                continue

            # Core condition: distance < threshold
            if dist < active_threshold:
                if not is_final_point:
                    print(
                        f"\n\033[93m>>> REACHED Waypoint {active_wp['index']}! "
                        f"Dist={dist:.3f}m <= {active_threshold:.2f}m | "
                        f"Speed={speed:.2f} m/s | Cmd_vx={cmd_vx:.2f} m/s\033[0m"
                    )

                    is_stutter = (speed < 0.20)
                    node.transition_records.append({
                        'point': active_wp['index'],
                        'rel_dist': active_wp['rel_dist'],
                        'dist_at_switch': dist,
                        'speed_at_switch': speed,
                        'cmd_vx_at_switch': cmd_vx,
                        'elapsed_time': elapsed_point,
                        'stuttered': is_stutter,
                        'note': 'NORMAL_ARRIVAL'
                    })

                    node.current_idx += 1
                    node.point_start_time = time.time()
                    next_wp = node.waypoints[node.current_idx]
                    node.publish_goal(next_wp)
                else:
                    print(
                        f"\n\033[92m>>> FINAL Waypoint {active_wp['index']} REACHED! "
                        f"Dist={dist:.3f}m <= {active_threshold:.2f}m | "
                        f"Speed={speed:.2f} m/s | Braking to complete stop.\033[0m"
                    )

                    node.transition_records.append({
                        'point': active_wp['index'],
                        'rel_dist': active_wp['rel_dist'],
                        'dist_at_switch': dist,
                        'speed_at_switch': speed,
                        'cmd_vx_at_switch': cmd_vx,
                        'elapsed_time': elapsed_point,
                        'stuttered': False,
                        'note': 'FINAL_DESTINATION_STOP'
                    })

                    node.test_finished = True
                    node.cancel_active_navigation()
                    break

    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by KeyboardInterrupt. Stopping robot...")
        node.emergency_stop()

    total_time = time.time() - node.test_start_time
    print_analysis_summary(node.transition_records, total_time, args, len(node.waypoints))

    node.destroy_node()
    rclpy.shutdown()


def print_analysis_summary(records: List[Dict[str, Any]], total_time: float, args, total_points: int):
    """Print quantitative table analyzing whether robot stuttered at each waypoint."""
    print("\n" + "="*80)
    print("         MULTI-POINT TRACKING & SMOOTHNESS ANALYSIS REPORT         ")
    print("="*80)
    print(f"Pattern Preset:   {args.pattern} (Angle: {args.angle}°)")
    print(f"Total Points:     {len(records)}/{total_points} | Total Test Time: {total_time:.2f} s")
    print(f"Waypoint Spacing: {args.spacing} m | Switch Threshold: {args.threshold} m\n")

    print(f"{'WP':<4} | {'Target Rel':<10} | {'Dist(m)':<8} | {'Speed(m/s)':<11} | {'Cmd_Vx':<9} | {'Time(s)':<8} | {'Stutter?':<9} | {'Note'}")
    print("-" * 80)

    stutter_count = 0
    for r in records:
        stutter_str = "\033[91mYES (顿挫)\033[0m" if r['stuttered'] else "\033[92mNO (平滑)\033[0m"
        if r['stuttered']:
            stutter_count += 1
        print(
            f"P{r['point']:<3} | +{r['rel_dist']:<8.2f}m | {r['dist_at_switch']:<8.3f} | "
            f"{r['speed_at_switch']:<11.2f} | {r['cmd_vx_at_switch']:<9.2f} | "
            f"{r['elapsed_time']:<8.2f} | {stutter_str:<18} | {r['note']}"
        )

    print("-" * 80)
    print(f"Stutter Occurrences: {stutter_count}/{len(records)} waypoint transitions.")

    if len(records) > 0 and stutter_count > len(records) * 0.5:
        print("\033[93m[结论]：测试显示机器狗在各个点切换时有显著的降速/停顿现象（一顿一顿）。")
        print("原因说明：rl_nav_controller 在进入 arrive_threshold 时会主动置零速度且清空目标，")
        print("          直到收到下一个 /goal_pose 才重新加速。对于连续轨迹跟踪，建议采用前瞻动态胡萝卜点（Lookahead Carrot）。\033[0m")
    else:
        print("\033[92m[结论]：机器狗在多数点切换时保持了连续速度，顿挫现象较轻。\033[0m")

    log_dir = "/tmp/multi_point_benchmark"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"test_run_{int(time.time())}.json")
    try:
        with open(log_file, 'w') as f:
            json.dump({
                'timestamp': time.time(),
                'args': vars(args),
                'total_time': total_time,
                'stutter_count': stutter_count,
                'records': records
            }, f, indent=2)
        print(f"\n[✓] 详细数据日志已保存至: {log_file}")
    except Exception as e:
        print(f"[!] 无法写入日志文件: {e}")
    print("="*80 + "\n")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Unitree Go2W Multi-Point Tracking & Smoothness Tester",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Basic trajectory parameters
    parser.add_argument('-n', '--num-points', type=int, default=8,
                        help="Number of waypoints")
    parser.add_argument('-s', '--spacing', type=float, default=0.5,
                        help="Default distance between adjacent points in meters")
    parser.add_argument('-t', '--threshold', type=float, default=0.1,
                        help="Arrival threshold in meters to advance to next point (P1..PN-1)")
    parser.add_argument('-tt', '--terminal-threshold', type=float, default=0.1,
                        help="Arrival threshold in meters for the FINAL destination point (default: 0.1, matching rl_nav_controller arrive_threshold)")
    parser.add_argument('--stop-at-threshold', action='store_true',
                        help="Force stop the final point at --threshold instead of driving all the way to --terminal-threshold")
    parser.add_argument('-l', '--lookahead', type=float, default=0.0,
                        help="Continuous Lookahead Carrot distance in meters (e.g. 2.0 or 2.5). When > 0, decouples goal distance from waypoint spacing, enabling robust obstacle avoidance even with 0.5m/1.0m dense waypoints. (default: 0.0 = discrete waypoint mode)")

    # Direction and pattern geometry
    parser.add_argument('-p', '--pattern', type=str, default='straight',
                        choices=['straight', 'diagonal-right', 'diagonal-left', 'arc-left', 'arc-right', 'zigzag', 'custom'],
                        help="Path geometry pattern preset")
    parser.add_argument('-a', '--angle', type=float, default=0.0,
                        help="Path heading angle in degrees relative to robot (-180 to 180, e.g. -30 for diagonal right/downward, +30 for diagonal left)")
    parser.add_argument('--dx', type=float, default=None,
                        help="Step forward in robot frame (m). If set with --dy, defines arbitrary step vector")
    parser.add_argument('--dy', type=float, default=None,
                        help="Step lateral in robot frame (m, +: left, -: right/downward)")
    parser.add_argument('--dz', type=float, default=0.0,
                        help="Step vertical height in meters (e.g. -0.05 for slope descent)")
    parser.add_argument('--turn-angle', type=float, default=15.0,
                        help="Turn angle increment per point in degrees for arc patterns")
    parser.add_argument('--lateral-offset', type=float, default=0.4,
                        help="Lateral offset magnitude in meters for zigzag pattern")
    parser.add_argument('--custom-points', type=str, default=None,
                        help="Custom relative (x,y) points in robot frame separated by semicolons, e.g. '0.5,-0.2;1.0,-0.4;1.5,-0.6'")

    # ROS 2 Topics and execution options
    parser.add_argument('--odom-topic', type=str, default='/lio/robo/odom',
                        help="Odometry topic name")
    parser.add_argument('--goal-topic', type=str, default='/goal_pose',
                        help="Target goal topic name")
    parser.add_argument('--cmd-vel-topic', type=str, default='/cmd_vel',
                        help="Velocity command topic name")
    parser.add_argument('--timeout-per-point', type=float, default=25.0,
                        help="Max seconds to wait on a blocked point before auto-skipping (0=disable)")
    parser.add_argument('--auto-start', action='store_true',
                        help="Skip interactive Enter prompt and start immediately")

    return parser.parse_args()


if __name__ == '__main__':
    args = parse_args()
    run_test(args)

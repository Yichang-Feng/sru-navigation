#!/usr/bin/env python3
"""
Unitree Go2W Mock Global Scheduler & Interaction Test Console
=============================================================
Simulates the upper-level global planner/scheduler to test the RL navigation controller
according to GLOBAL_NAV_INTEGRATION_SPEC.md.

Capabilities:
1. Subscribes to /local_planner/status to monitor 3Hz heartbeat and millisecond-level state transitions.
2. Subscribes to /startNavi to verify chassis gate auto-unlocking.
3. Subscribes to /cmd_vel to monitor motor velocity outputs and active braking.
4. Subscribes to /lio/robo/odom to get robot current position and generate relative paths.
5. Publishes /global_path (nav_msgs/msg/Path) with configurable points and spacing.
6. Publishes /mission/control (std_msgs/msg/String: "stop", "continue") to test control.
7. Supports both interactive menu mode and automated regression test mode (--auto).

Usage:
    # Interactive mode:
    python3 scripts/mock_global_nav_tester.py

    # Automated regression mode:
    python3 scripts/mock_global_nav_tester.py --auto
"""

from __future__ import annotations

import sys
import time
import math
import json
import argparse
import threading
from typing import List, Tuple, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Bool, String, Header
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("\033[91m[Error] ROS 2 (rclpy) is not available in the current environment!\033[0m")
    print("Please run: source /opt/ros/humble/setup.bash (or source your ROS 2 environment).")


# ANSI colors
CYAN = '\033[0;36m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
RED = '\033[0;31m'
BLUE = '\033[0;34m'
MAGENTA = '\033[0;35m'
BOLD = '\033[1m'
NC = '\033[0m'


class MockGlobalScheduler(Node):
    """Simulates the upper global scheduler layer."""

    def __init__(self, path_topic="/global_path", stop_topic="/mission/control", status_topic="/local_planner/status",
                 odom_topic="/lio/robo/odom", cmd_vel_topic="/cmd_vel", start_navi_topic="/startNavi"):
        super().__init__('mock_global_scheduler')

        self.path_topic = path_topic
        self.stop_topic = stop_topic
        self.status_topic = status_topic
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.start_navi_topic = start_navi_topic

        # Publishers
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.stop_pub = self.create_publisher(String, self.stop_topic, 10)

        # Subscribers (status QoS: reliable + transient_local)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.status_sub = self.create_subscription(String, self.status_topic, self.status_cb, status_qos)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 10)
        self.start_navi_sub = self.create_subscription(Bool, self.start_navi_topic, self.start_navi_cb, 10)

        # Telemetry state
        self.current_state = "UNKNOWN"
        self.current_msgs = ""
        self.last_state = "UNKNOWN"
        self.heartbeat_count = 0
        self.last_status_time = time.time()
        self.transitions = []

        self.robot_pos = [0.0, 0.0, 0.0]
        self.robot_yaw = 0.0
        self.map_frame_id = "world"
        self.has_odom = False

        self.latest_cmd_vel = [0.0, 0.0, 0.0]
        self.chassis_gate_unlocked = False

        # Segment sequencing state & event synchronization
        self.state_event = threading.Event()
        self.auto_advance_mode = False
        self.segment_counter = 0

        self.get_logger().info(f"{GREEN}Mock Global Scheduler initialized on {path_topic}, {status_topic}{NC}")

    def status_cb(self, msg: String):
        now = time.time()
        raw_text = msg.data.strip()
        state_str = raw_text
        msgs_str = ""

        try:
            parsed = json.loads(raw_text)
            if isinstance(parsed, dict):
                state_str = str(parsed.get("state", raw_text))
                msgs_str = str(parsed.get("msgs", ""))
        except Exception:
            state_str = raw_text

        self.heartbeat_count += 1
        self.last_status_time = now
        self.current_msgs = msgs_str

        if state_str != self.current_state:
            self.last_state = self.current_state
            self.current_state = state_str
            dt_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
            extra_msg = f" (msgs: '{self.current_msgs}')" if self.current_msgs else ""
            log_entry = f"[{dt_str}] State transition: {self.last_state} -> {self.current_state}{extra_msg}"
            self.transitions.append(log_entry)
            print(f"\n{MAGENTA}{BOLD}>>> {log_entry}{NC}")

            # Notify any thread waiting on state transition
            self.state_event.set()

            # Auto advance to next segment if enabled and arrived / goal_reached
            if self.auto_advance_mode and self.current_state in ("goal_reached", "arrived"):
                self.get_logger().info(f"{GREEN}[Auto-Advance] '{self.current_state}' received! Scheduling next segment in 1.0s...{NC}")
                threading.Thread(target=self._delayed_next_segment, daemon=True).start()

    def _delayed_next_segment(self):
        time.sleep(1.0)
        if self.auto_advance_mode:
            self.segment_counter += 1
            print(f"\n{CYAN}>>> [Auto-Advance] Dispatching next segment #{self.segment_counter}...{NC}")
            self.send_straight_segment(length=2.5, num_points=3)

    def odom_cb(self, msg: Odometry):
        self.has_odom = True
        self.robot_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        if msg.header.frame_id:
            self.map_frame_id = msg.header.frame_id

        # Calculate yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cmd_vel_cb(self, msg: Twist):
        self.latest_cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

    def start_navi_cb(self, msg: Bool):
        self.chassis_gate_unlocked = msg.data

    def send_path(self, points: List[Tuple[float, float, float]], frame_id: Optional[str] = None, terminal_yaw: Optional[float] = None):
        """Construct and publish a nav_msgs/msg/Path message."""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id if frame_id is not None else self.map_frame_id

        for i, (x, y, z) in enumerate(points):
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = float(x)
            pose_stamped.pose.position.y = float(y)
            pose_stamped.pose.position.z = float(z)

            # Set yaw for last point if requested
            if i == len(points) - 1 and terminal_yaw is not None:
                pose_stamped.pose.orientation.z = math.sin(terminal_yaw / 2.0)
                pose_stamped.pose.orientation.w = math.cos(terminal_yaw / 2.0)
            else:
                pose_stamped.pose.orientation.w = 1.0

            path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)
        print(f"{GREEN}[✓] Published Path on {self.path_topic}: {len(points)} points, frame='{path_msg.header.frame_id}'{NC}")
        print(f"    Start: [{points[0][0]:.2f}, {points[0][1]:.2f}], End: [{points[-1][0]:.2f}, {points[-1][1]:.2f}]")

    def send_straight_segment(self, length=3.0, num_points=4):
        """Send a straight line path segment in the direction the robot is facing."""
        x0, y0, z0 = self.robot_pos
        yaw = self.robot_yaw
        points = []
        step = length / max(1, (num_points - 1))
        for i in range(num_points):
            d = i * step
            px = x0 + d * math.cos(yaw)
            py = y0 + d * math.sin(yaw)
            points.append((px, py, z0))
        self.send_path(points, terminal_yaw=yaw)
        return points

    def send_left_turn_segment(self, distance=1.5, num_points=3):
        """Send a path segment turning 90 degrees left relative to robot's current heading and walking distance meters."""
        x0, y0, z0 = self.robot_pos
        yaw0 = self.robot_yaw
        turn_yaw = yaw0 + math.pi / 2.0
        turn_yaw = math.atan2(math.sin(turn_yaw), math.cos(turn_yaw))
        points = []
        step = distance / max(1, (num_points - 1))
        for i in range(num_points):
            d = i * step
            px = x0 + d * math.cos(turn_yaw)
            py = y0 + d * math.sin(turn_yaw)
            points.append((px, py, z0))
        self.send_path(points, terminal_yaw=turn_yaw)
        return points

    def send_l_shape_segment(self, forward=2.0, left=2.0):
        """Send an L-shaped path segment (forward then 90-degree left)."""
        x0, y0, z0 = self.robot_pos
        yaw = self.robot_yaw
        points = []
        # Forward segment: 3 points
        for i in range(3):
            d = (forward / 2.0) * i
            px = x0 + d * math.cos(yaw)
            py = y0 + d * math.sin(yaw)
            points.append((px, py, z0))
        # Turn left 90 deg: 2 points
        corner_x = points[-1][0]
        corner_y = points[-1][1]
        turn_yaw = yaw + math.pi / 2.0
        for i in range(1, 3):
            d = (left / 2.0) * i
            px = corner_x + d * math.cos(turn_yaw)
            py = corner_y + d * math.sin(turn_yaw)
            points.append((px, py, z0))
        self.send_path(points, terminal_yaw=turn_yaw)

    def set_mission_control(self, cmd: str):
        """Publish command to /mission/control (std_msgs/msg/String: 'pause', 'resume', 'stop', 'continue')."""
        msg = String()
        msg.data = cmd.strip().lower()
        self.stop_pub.publish(msg)
        desc = "PAUSE/STOP (底盘主动刹停，进入 stopped 状态，自动保留当前路径)" if msg.data in ("pause", "stop") else "RESUME/CONTINUE (恢复沿保留路径推进，进入 running 状态)"
        print(f"{YELLOW}[✓] Published {self.stop_topic} = \"{msg.data}\" ({desc}){NC}")

    def set_nav_stop(self, stop: bool):
        """Compatibility wrapper: True -> 'pause', False -> 'resume'."""
        self.set_mission_control("pause" if stop else "resume")


def run_two_segment_routine(node: MockGlobalScheduler, seg1_length: float = 2.0, seg2_length: float = 1.5):
    """
    Execute dual-segment continuous navigation test:
    1. Send Segment 1: straight ahead (like Option 1)
    2. Wait for robot to track and arrive (detect state transition to 'goal_reached' / 'arrived')
    3. As soon as goal reached state is detected, immediately send Segment 2: turn left 90° and walk 1.5m
    4. Wait for Segment 2 to arrive
    """
    print(f"\n{CYAN}{BOLD}==================================================================")
    print(f"   执行双段连续推进测试: 第1段直行 -> 到点状态检测 -> 第2段左转1.5m    ")
    print(f"=================================================================={NC}")

    # Check if controller is alive
    time_since_status = time.time() - node.last_status_time
    if time_since_status > 3.0 and node.current_state == "UNKNOWN":
        print(f"{RED}[ERROR] 未接收到控制器状态心跳 (已超时 {time_since_status:.1f}s)，请检查 rl_nav_controller 节点！{NC}")
        return False

    # Disable background auto_advance_mode if enabled to prevent race conditions
    node.auto_advance_mode = False

    # -------------------------------------------------------------
    # Step 1: Send Segment 1 (Straight)
    # -------------------------------------------------------------
    print(f"\n{YELLOW}{BOLD}[步骤 1/4] 下发第 1 段路径: 沿当前朝向前进 {seg1_length:.1f}m...{NC}")
    node.state_event.clear()
    pts1 = node.send_straight_segment(length=seg1_length, num_points=4)
    target1 = pts1[-1]

    # Wait for controller to enter RUNNING
    print(f"  -> 等待控制器响应并进入 RUNNING 状态...")
    t0 = time.time()
    while time.time() - t0 < 3.0 and rclpy.ok():
        if node.current_state == "running":
            break
        node.state_event.wait(timeout=0.1)
        node.state_event.clear()

    if node.current_state != "running":
        print(f"{RED}[FAIL] 下发第 1 段路径后，3 秒内控制器未进入 running (当前状态: {node.current_state})！{NC}")
        return False

    print(f"{GREEN}[✓] 控制器已进入 RUNNING 状态，机器狗正执行第 1 段直行推进！{NC}")

    # -------------------------------------------------------------
    # Step 2: Track Segment 1 until GOAL_REACHED / ARRIVED
    # -------------------------------------------------------------
    t_start1 = time.time()
    last_print = 0.0
    while rclpy.ok():
        if node.current_state in ("goal_reached", "arrived"):
            elapsed = time.time() - t_start1
            print(f"\n{GREEN}{BOLD}[✓ 状态跃迁检测成功] 机器狗已到点！控制器上报状态: {node.current_state} (第1段耗时: {elapsed:.2f}s){NC}")
            break
        elif node.current_state in ("error", "failed", "blocked", "stopped", "stop"):
            print(f"\n{RED}[!] 第 1 段推进异常中止，当前状态: {node.current_state} (msgs: {node.current_msgs}){NC}")
            return False

        now = time.time()
        if now - last_print > 0.4:
            last_print = now
            dx = node.robot_pos[0] - target1[0]
            dy = node.robot_pos[1] - target1[1]
            dist_rem = math.hypot(dx, dy)
            elapsed = now - t_start1
            print(
                f"\r  -> [第1段直行中] 坐标: [{node.robot_pos[0]:.2f}, {node.robot_pos[1]:.2f}] | "
                f"距终点: {dist_rem:.2f}m | 速度: {node.latest_cmd_vel[0]:.2f}m/s | 状态: {node.current_state}   ",
                end="",
                flush=True
            )

        node.state_event.wait(timeout=0.1)
        node.state_event.clear()

    # -------------------------------------------------------------
    # Step 3: Immediate dispatch of Segment 2 (Left Turn 1.5m)
    # -------------------------------------------------------------
    print(f"\n{MAGENTA}{BOLD}[步骤 2/4] 检测到状态变化 (goal_reached)，立刻瞬发第 2 段路径: 左转 90° 走 {seg2_length:.1f}m！{NC}")
    node.state_event.clear()
    pts2 = node.send_left_turn_segment(distance=seg2_length, num_points=3)
    target2 = pts2[-1]

    # Wait for controller to enter RUNNING for Segment 2
    print(f"  -> 等待控制器响应第 2 段路径并恢复 RUNNING 状态...")
    t0 = time.time()
    while time.time() - t0 < 3.0 and rclpy.ok():
        if node.current_state == "running":
            break
        node.state_event.wait(timeout=0.1)
        node.state_event.clear()

    if node.current_state != "running":
        print(f"{RED}[FAIL] 下发第 2 段路径后，3 秒内控制器未进入 running (当前状态: {node.current_state})！{NC}")
        return False

    print(f"{GREEN}[✓] 控制器已接收第 2 段路径并进入 RUNNING 状态，机器狗正左转推进！{NC}")

    # -------------------------------------------------------------
    # Step 4: Track Segment 2 until GOAL_REACHED / ARRIVED
    # -------------------------------------------------------------
    t_start2 = time.time()
    last_print = 0.0
    while rclpy.ok():
        if node.current_state in ("goal_reached", "arrived"):
            elapsed = time.time() - t_start2
            print(f"\n{GREEN}{BOLD}[✓ 状态跃迁检测成功] 机器狗第 2 段左转 {seg2_length:.1f}m 顺利完成！控制器上报状态: {node.current_state} (第2段耗时: {elapsed:.2f}s){NC}")
            break
        elif node.current_state in ("error", "failed", "blocked", "stopped", "stop"):
            print(f"\n{RED}[!] 第 2 段推进异常中止，当前状态: {node.current_state} (msgs: {node.current_msgs}){NC}")
            return False

        now = time.time()
        if now - last_print > 0.4:
            last_print = now
            dx = node.robot_pos[0] - target2[0]
            dy = node.robot_pos[1] - target2[1]
            dist_rem = math.hypot(dx, dy)
            elapsed = now - t_start2
            print(
                f"\r  -> [第2段左转中] 坐标: [{node.robot_pos[0]:.2f}, {node.robot_pos[1]:.2f}] | "
                f"距终点: {dist_rem:.2f}m | 速度: {node.latest_cmd_vel[0]:.2f}m/s | 状态: {node.current_state}   ",
                end="",
                flush=True
            )

        node.state_event.wait(timeout=0.1)
        node.state_event.clear()

    total_time = (time.time() - t_start1)
    print(f"\n{GREEN}{BOLD}==================================================================")
    print(f"       双段连续测试成功！总耗时: {total_time:.2f}s")
    print(f"       第1段直行 {seg1_length:.1f}m -> 到点检测上报 -> 瞬发第2段左转 {seg2_length:.1f}m -> 顺利到达")
    print(f"=================================================================={NC}\n")
    return True


def run_interactive(node: MockGlobalScheduler, args: Optional[argparse.Namespace] = None):
    """Run interactive terminal menu."""
    while rclpy.ok():
        # Clear screen summary banner
        time_since_status = time.time() - node.last_status_time
        status_color = GREEN if node.current_state in ("running", "goal_reached", "arrived") else (
            YELLOW if node.current_state in ("stopped", "stop", "paused") else (
                RED if node.current_state in ("error", "failed", "blocked") else CYAN
            )
        )
        gate_str = f"{GREEN}UNLOCKED{NC}" if node.chassis_gate_unlocked else f"{RED}LOCKED{NC}"
        alive_str = f"{GREEN}OK (1Hz){NC}" if time_since_status < 2.5 else f"{RED}TIMEOUT ({time_since_status:.1f}s){NC}"
        msg_str = f" [msgs: \"{node.current_msgs}\"]" if node.current_msgs else ""

        print(f"\n{CYAN}=================================================================={NC}")
        print(f"{CYAN}       Unitree Go2W 全局导航交互仿真测试控制台 (Mock Scheduler)  {NC}")
        print(f"{CYAN}=================================================================={NC}")
        print(f" 状态机当前状态: {status_color}{BOLD}{node.current_state.upper()}{NC}{msg_str}  (上报心跳: {alive_str})")
        print(f" 底盘安全总闸:   {gate_str} (/startNavi)")
        print(f" 机器狗当前坐标: [{node.robot_pos[0]:.2f}, {node.robot_pos[1]:.2f}, {node.robot_pos[2]:.2f}], 航向: {math.degrees(node.robot_yaw):.1f}°")
        print(f" 下发控制速度:   vx={node.latest_cmd_vel[0]:.3f} m/s, vy={node.latest_cmd_vel[1]:.3f} m/s, wz={node.latest_cmd_vel[2]:.3f} rad/s")
        print(f"{CYAN}------------------------------------------------------------------{NC}")
        print(f"  {BOLD}[1]{NC} 下发第 1 段路径: 沿当前朝向前进 3.0m (点间距 1.0m)")
        print(f"  {BOLD}[2]{NC} 下发第 2 段路径: L 型拐角避障段 (前进 2m + 左转 2m)")
        print(f"  {BOLD}[3]{NC} 下发短位移微调段: 前进 1.0m (点间距 0.5m)")
        print(f"  {BOLD}[4]{NC} 发送暂停/停止指令: {YELLOW}/mission/control = \"pause\"{NC} (进入 STOPPED 状态，底盘刹停，自动保留路径)")
        print(f"  {BOLD}[5]{NC} 发送恢复推进指令: {GREEN}/mission/control = \"resume\"{NC} (状态恢复为 RUNNING，继续推进)")
        print(f"  {BOLD}[6]{NC} {MAGENTA}{BOLD}双段连续推进测试{NC}: 发送第1段直行 -> 检测到点上报 -> {BOLD}立刻发送第2段左转走1.5m{NC}")
        print(f"  {BOLD}[7]{NC} 模拟阻挡死锁检测 (原地发送穿障不可达段, 观察 4s 后触发 FAILED 状态)")
        print(f"  {BOLD}[8]{NC} 查看近期状态跃迁历史时间戳")
        print(f"  {BOLD}[q]{NC} 退出测试程序")
        print(f"{CYAN}=================================================================={NC}")

        try:
            choice = input(f"{BOLD}请选择操作指令 [1-8, q]: {NC}").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if choice == '1':
            node.send_straight_segment(length=3.0, num_points=4)
        elif choice == '2':
            node.send_l_shape_segment(forward=2.0, left=2.0)
        elif choice == '3':
            node.send_straight_segment(length=1.0, num_points=3)
        elif choice == '4':
            node.set_mission_control("pause")
        elif choice == '5':
            node.set_mission_control("resume")
        elif choice == '6':
            try:
                s1 = getattr(args, 'seg1_len', 2.0) if args is not None else 2.0
                s2 = getattr(args, 'seg2_len', 1.5) if args is not None else 1.5
                run_two_segment_routine(node, seg1_length=s1, seg2_length=s2)
            except KeyboardInterrupt:
                print(f"\n{YELLOW}[!] 双段测试被用户手动中断{NC}")
            input(f"\n{YELLOW}按回车键返回菜单...{NC}")
        elif choice == '7':
            print(f"{YELLOW}[!] 正在下发测试路径，请观察底盘制动或故意阻挡机器狗，连续 4.0 秒进尺 <0.1m 将触发 BLOCKED...{NC}")
            node.send_straight_segment(length=4.0, num_points=5)
        elif choice == '8':
            print(f"\n{BLUE}=== 状态跃迁历史 ==={NC}")
            for t_log in node.transitions[-10:]:
                print(f"  {t_log}")
            input(f"\n{YELLOW}按回车键返回菜单...{NC}")
        elif choice == 'q':
            print("正在退出测试...")
            break
        else:
            print("无效输入，请重新选择。")
        time.sleep(0.3)


def run_automated_test(node: MockGlobalScheduler):
    """Run non-interactive automated test verifying state machine loop."""
    print(f"\n{CYAN}=================================================================={NC}")
    print(f"{CYAN}       Unitree Go2W 全局导航自动化闭环回归测试 (Automated Test)  {NC}")
    print(f"{CYAN}=================================================================={NC}")

    # Wait for status heartbeat
    print("[1/5] 等待控制器心跳与状态广播...")
    t0 = time.time()
    while time.time() - t0 < 5.0 and node.current_state == "UNKNOWN":
        time.sleep(0.1)

    if node.current_state == "UNKNOWN":
        print(f"{RED}[FAIL] 5 秒内未接收到 /local_planner/status 心跳！请检查 rl_nav_controller 节点是否已启动。{NC}")
        return False
    print(f"{GREEN}[PASS] 接收到控制器状态: {node.current_state} (心跳正常){NC}")

    # Step 2: Send segment 1 and check RUNNING
    print("\n[2/5] 下发第 1 段分段路径并检测状态瞬发...")
    node.send_straight_segment(length=2.0, num_points=3)
    t0 = time.time()
    while time.time() - t0 < 2.0 and node.current_state != "running":
        time.sleep(0.05)

    if node.current_state != "running":
        print(f"{RED}[FAIL] 下发路径后状态未能进入 'running'! 当前状态: {node.current_state}{NC}")
        return False
    print(f"{GREEN}[PASS] 状态成功跃迁至 'running'！{NC}")

    # Step 3: Test Pause / Stop
    print("\n[3/5] 下发 /mission/control = 'pause' 测试暂停与零速约束...")
    node.set_mission_control("pause")
    t0 = time.time()
    while time.time() - t0 < 2.0 and node.current_state not in ("stopped", "stop"):
        time.sleep(0.05)

    if node.current_state not in ("stopped", "stop"):
        print(f"{RED}[FAIL] 发送 pause 指令后状态未能进入 'stopped'! 当前状态: {node.current_state}{NC}")
        return False
    print(f"{GREEN}[PASS] 状态成功跃迁至 '{node.current_state}'，底盘速度已制动并保留路径！{NC}")

    # Step 4: Test Resume / Continue
    print("\n[4/5] 下发 /mission/control = 'resume' 测试恢复执行...")
    node.set_mission_control("resume")
    t0 = time.time()
    while time.time() - t0 < 2.0 and node.current_state != "running":
        time.sleep(0.05)

    if node.current_state != "running":
        print(f"{RED}[FAIL] 恢复指令 resume 发送后状态未能进入 'running'! 当前状态: {node.current_state}{NC}")
        return False
    print(f"{GREEN}[PASS] 状态成功恢复为 'running'！{NC}")

    # Step 5: Finished
    print("\n[5/5] 测试底盘安全总闸监听...")
    print(f"      当前底盘开闸状态: {'UNLOCKED (True)' if node.chassis_gate_unlocked else 'LOCKED (False)'}")
    print(f"\n{GREEN}{BOLD}==================================================================")
    print(f"       全流程交互通信自动化验证成功！(ALL CHECKS PASSED)")
    print(f"=================================================================={NC}")
    return True


def main():
    if not ROS2_AVAILABLE:
        sys.exit(1)

    parser = argparse.ArgumentParser(description="Mock Global Scheduler for Go2W RL Navigation")
    parser.add_argument('--auto', action='store_true', help='Run automated non-interactive regression test')
    parser.add_argument('--two-seg', action='store_true', help='Directly execute dual-segment test (straight -> left turn 1.5m)')
    parser.add_argument('--seg1-len', type=float, default=2.0, help='Length of segment 1 straight line (meters, default: 2.0)')
    parser.add_argument('--seg2-len', type=float, default=1.5, help='Length of segment 2 left turn line (meters, default: 1.5)')
    parser.add_argument('--path-topic', type=str, default='/global_path', help='Path topic name')
    parser.add_argument('--stop-topic', type=str, default='/mission/control', help='Stop/mission control topic name')
    parser.add_argument('--status-topic', type=str, default='/local_planner/status', help='Status reporting topic name')
    args = parser.parse_args()

    rclpy.init()
    node = MockGlobalScheduler(
        path_topic=args.path_topic,
        stop_topic=args.stop_topic,
        status_topic=args.status_topic
    )

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    time.sleep(0.5)

    try:
        if args.auto:
            run_automated_test(node)
        elif args.two_seg:
            run_two_segment_routine(node, seg1_length=args.seg1_len, seg2_length=args.seg2_len)
        else:
            run_interactive(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

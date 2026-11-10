#!/usr/bin/env python3
"""
Door & Stairs Navigation Diagnostic Recorder & Analyzer (Comprehensive Edition)
专门用于采集和深度分析小门/窄道/复杂通道/下楼梯场景下 SRU 策略视觉与目标向量的真机综合诊断记录器。

核心功能与增强：
1. 【下楼梯感知门控诊断】：实时订阅 /nav/is_descending_stairs，记录坡度门控激活时序，验证下楼判定是否精准触发。
2. 【虚拟楼梯重映射深度留存】：实时订阅 /nav/remapped_depth，记录送入策略模型的真实重映射深度图及走廊统计。
3. 【全 3D 目标相对运动学】：计算并记录相对于机器人局部基坐标系的目标分量 (dx_b, dy_b, dz_b)、平面距离、3D 欧氏距离、偏航角、俯仰角与方位象限 (FRONT, BEHIND 等)，彻底排查“目标在身后导致掉头”现象。
4. 【多扇区空间净空划分 (40x64)】：细分左侧扇区、中央走廊、右侧扇区、近地地面高度以及虚空/深渊占比，直接反映扶手逼近与前方阶梯跌落。
5. 【机身姿态与实测速度对比】：提取里程计四元数解算 Roll/Pitch/Yaw，同时比对 cmd_vel 指令速度与里程计实际运动速度。
6. 【全局规划路径坡度诊断】：订阅 /global_path，计算当前前方 0.4~2.2m 规划路径的下倾坡度与剩余距离。
7. 【增强型直观热力图】：画面叠加三通道净空框、下楼状态指示、身后掉头警告横幅与双重速度显示。
8. 【100% 格式向后兼容】：原有 CSV 前 19 列保持完全一致，新增 20+ 诊断字段顺序追加，异步后台无阻塞写盘。
"""

import os
import sys
import time
import json
import math
import argparse
import threading
import queue
import datetime
import csv
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge


class DoorDiagnosticLogger(Node):
    def __init__(self, base_dir: str = '~/door_diagnostics', session_tag: str = '', 
                 record_rate: float = 5.0, verbose: bool = False,
                 record_remapped: bool = True, path_topic: str = '/global_path'):
        super().__init__('door_diagnostic_logger')

        self.bridge = CvBridge()
        self.record_rate = max(0.5, float(record_rate))
        self.verbose = verbose
        self.record_remapped = record_remapped
        self.path_topic = path_topic

        self.min_depth = 0.25
        self.max_depth = 10.0

        # 1. 初始化独立时间戳会话文件夹与子目录
        self.base_dir = os.path.expanduser(base_dir)
        now_str = datetime.datetime.now().strftime("session_%Y%m%d_%H%M%S")
        if session_tag:
            safe_tag = "".join(c for c in session_tag if c.isalnum() or c in ('_', '-'))
            self.session_name = f"{now_str}_{safe_tag}"
        else:
            self.session_name = now_str

        self.session_dir = os.path.join(self.base_dir, self.session_name)
        self.heatmap_dir = os.path.join(self.session_dir, 'heatmaps')
        self.depth_dir = os.path.join(self.session_dir, 'depth')
        self.meta_dir = os.path.join(self.session_dir, 'meta')
        self.remapped_depth_dir = os.path.join(self.session_dir, 'depth_remapped')
        self.remapped_heatmap_dir = os.path.join(self.session_dir, 'heatmaps_remapped')

        for d in [self.session_dir, self.heatmap_dir, self.depth_dir, self.meta_dir,
                 self.remapped_depth_dir, self.remapped_heatmap_dir]:
            os.makedirs(d, exist_ok=True)

        # 快捷软链接
        self.latest_link = os.path.join(self.base_dir, 'latest')
        try:
            if os.path.islink(self.latest_link) or os.path.exists(self.latest_link):
                os.unlink(self.latest_link)
            os.symlink(self.session_name, self.latest_link)
        except Exception:
            pass

        # 2. 初始化 CSV 轨迹汇总表 (46 列，前 19 列完全向后兼容)
        self.csv_path = os.path.join(self.session_dir, 'trajectory.csv')
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_headers = [
            # --- 原有字段 (0-18: 严格保持原有顺序以 100% 向后兼容) ---
            "frame_idx", "timestamp_iso", "epoch_sec", 
            "robot_x", "robot_y", "robot_z", "robot_yaw_deg",
            "carrot_x", "carrot_y", "carrot_z",
            "target_dist", "target_angle_deg",
            "cmd_vx", "cmd_vy", "cmd_wz",
            "door_center_min_depth", "door_center_avg_depth",
            "is_carrot_valid", "carrot_source",

            # --- 姿态角与实测速度 (19-24) ---
            "robot_roll_deg", "robot_pitch_deg",
            "odom_vx", "odom_vy", "odom_vz", "odom_wz",

            # --- 完整 3D 目标相对体坐标系分量与运动学 (25-30) ---
            "target_dx_b", "target_dy_b", "target_dz_b",
            "target_dist_3d", "target_pitch_deg", "target_quadrant",

            # --- 多区域空间净空与深渊占比 (31-38) ---
            "depth_left_min", "depth_left_avg",
            "depth_center_min", "depth_center_avg",
            "depth_right_min", "depth_right_avg",
            "depth_floor_near", "center_void_ratio",

            # --- 楼梯感知门控与重映射深度 (39-42) ---
            "is_descending_stairs", "remapped_active",
            "remapped_center_min", "remapped_center_avg",

            # --- 全局路径前瞻与坡度诊断 (43-45) ---
            "path_points_count", "path_dist_to_end", "path_slope_ahead"
        ]
        self.csv_writer.writerow(self.csv_headers)
        self.csv_file.flush()

        # 3. 运行状态变量
        self.robot_pos = None
        self.robot_quat = None
        self.robot_roll = 0.0
        self.robot_pitch = 0.0
        self.robot_yaw = 0.0
        self.robot_yaw_deg = 0.0
        self.last_odom_time = 0.0

        # 地图定位里程计 (/loc/robo/odom - 'map' 坐标系)
        self.robot_pos_map = None
        self.robot_quat_map = None
        self.robot_roll_map = 0.0
        self.robot_pitch_map = 0.0
        self.robot_yaw_map = 0.0
        self.robot_yaw_deg_map = 0.0
        self.last_loc_odom_time = 0.0

        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_vz = 0.0
        self.odom_wz = 0.0

        self.carrot_pos = None
        self.carrot_frame_id = "map"
        self.last_carrot_time = 0.0
        self.carrot_source = "none"

        self.last_cmd = [0.0, 0.0, 0.0]
        self.last_cmd_time = 0.0

        self.is_descending_stairs = False
        self.last_stair_time = 0.0

        self.latest_remapped_depth = None
        self.last_remapped_time = 0.0

        self.global_path_pts = None
        self.last_path_time = 0.0

        self.frame_idx = 0
        self.dropped_queue_frames = 0
        self.last_record_epoch = 0.0
        self.start_epoch = time.time()
        self.last_terminal_print = 0.0
        self.depth_msg_count = 0

        # 4. 异步写入队列与后台工作线程
        self.write_queue = queue.Queue(maxsize=300)
        self.stop_worker_event = threading.Event()
        self.worker_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.worker_thread.start()

        # 5. 订阅核心诊断 topic
        self.create_subscription(Odometry, '/lio/robo/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/loc/robo/odom', self.loc_odom_cb, 10)
        self.create_subscription(Marker, '/vis/lookahead_goal', self.carrot_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.depth_cb, 10)

        # 订阅新增楼梯与路径诊断 topic
        self.create_subscription(Bool, '/nav/is_descending_stairs', self.stair_state_cb, 10)
        self.create_subscription(Image, '/nav/remapped_depth', self.remapped_depth_cb, 10)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)

        # 6. 定时状态心跳 (1Hz)
        self.create_timer(1.0, self.status_heartbeat_cb)

        self.get_logger().info('\033[92m' + '='*75 + '\033[0m')
        self.get_logger().info(f'\033[92m[Door & Stairs Diagnostic Logger] 综合诊断记录器运行就绪！\033[0m')
        self.get_logger().info(f'  📁 会话存储目录: {self.session_dir}')
        self.get_logger().info(f'  🔗 快捷软链接:   {self.latest_link}')
        self.get_logger().info(f'  ⏱ 采集目标帧率: {self.record_rate:.1f} Hz (异步无阻写盘)')
        self.get_logger().info(f'  📋 轨迹汇总表:   {self.csv_path} (共 {len(self.csv_headers)} 列)')
        self.get_logger().info(f'  🪜 楼梯诊断通道: /nav/is_descending_stairs, /nav/remapped_depth')
        self.get_logger().info(f'  🗺 规划路径通道: {self.path_topic}')
        self.get_logger().info('\033[92m' + '='*75 + '\033[0m')

    def odom_cb(self, msg: Odometry):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.robot_pos = np.array([p.x, p.y, p.z], dtype=float)
            self.robot_quat = np.array([q.w, q.x, q.y, q.z], dtype=float)

            # 解析机器人三轴姿态角 (Roll, Pitch, Yaw)
            qw, qx, qy, qz = self.robot_quat

            # Roll (x-axis rotation)
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            self.robot_roll = math.degrees(roll) if math.isfinite(roll) else 0.0

            # Pitch (y-axis rotation)
            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))
            self.robot_pitch = math.degrees(pitch) if math.isfinite(pitch) else 0.0

            # Yaw (z-axis rotation)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_yaw = yaw if math.isfinite(yaw) else 0.0
            self.robot_yaw_deg = math.degrees(yaw) if math.isfinite(yaw) else 0.0

            # 提取里程计实际运动速度
            self.odom_vx = float(msg.twist.twist.linear.x)
            self.odom_vy = float(msg.twist.twist.linear.y)
            self.odom_vz = float(msg.twist.twist.linear.z)
            self.odom_wz = float(msg.twist.twist.angular.z)

            self.last_odom_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'里程计解析异常: {e}')

    def loc_odom_cb(self, msg: Odometry):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.robot_pos_map = np.array([p.x, p.y, p.z], dtype=float)
            self.robot_quat_map = np.array([q.w, q.x, q.y, q.z], dtype=float)

            qw, qx, qy, qz = self.robot_quat_map

            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll = math.atan2(sinr_cosp, cosr_cosp)
            self.robot_roll_map = math.degrees(roll) if math.isfinite(roll) else 0.0

            sinp = 2.0 * (qw * qy - qz * qx)
            pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))
            self.robot_pitch_map = math.degrees(pitch) if math.isfinite(pitch) else 0.0

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_yaw_map = yaw if math.isfinite(yaw) else 0.0
            self.robot_yaw_deg_map = math.degrees(yaw) if math.isfinite(yaw) else 0.0

            self.last_loc_odom_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'定位里程计解析异常: {e}')

    def carrot_cb(self, msg: Marker):
        try:
            p = msg.pose.position
            self.carrot_pos = np.array([p.x, p.y, p.z], dtype=float)
            self.carrot_frame_id = msg.header.frame_id if msg.header.frame_id else "map"
            self.last_carrot_time = time.time()
            self.carrot_source = "/vis/lookahead_goal"
        except Exception as e:
            self.get_logger().warn(f'前瞻点解析异常: {e}')

    def goal_pose_cb(self, msg: PoseStamped):
        # 辅助备选：如果当前没有路径跟踪的前瞻点（例如单目标点导航），采用 /goal_pose
        if self.carrot_pos is None or (time.time() - self.last_carrot_time > 3.0):
            try:
                p = msg.pose.position
                self.carrot_pos = np.array([p.x, p.y, p.z], dtype=float)
                self.carrot_frame_id = msg.header.frame_id if msg.header.frame_id else "map"
                self.last_carrot_time = time.time()
                self.carrot_source = "/goal_pose"
            except Exception:
                pass

    def cmd_cb(self, msg: Twist):
        self.last_cmd = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        self.last_cmd_time = time.time()

    def stair_state_cb(self, msg: Bool):
        self.is_descending_stairs = bool(msg.data)
        self.last_stair_time = time.time()

    def remapped_depth_cb(self, msg: Image):
        try:
            remap_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if remap_raw.shape != (40, 64):
                remap_raw = cv2.resize(remap_raw, (64, 40), interpolation=cv2.INTER_LINEAR)
            self.latest_remapped_depth = remap_raw.astype(np.float32)
            self.last_remapped_time = time.time()
        except Exception:
            pass

    def path_cb(self, msg: Path):
        try:
            if len(msg.poses) > 0:
                pts = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] 
                                for p in msg.poses], dtype=np.float32)
                self.global_path_pts = pts
                self.last_path_time = time.time()
        except Exception:
            pass

    def depth_cb(self, msg: Image):
        self.depth_msg_count += 1
        now = time.time()

        # 频率节流：按设定 rate (默认 5Hz) 采样
        min_interval = 1.0 / self.record_rate
        if (now - self.last_record_epoch) < min_interval:
            return

        # 必须等待里程计就绪，获取自身基准
        if (self.robot_pos is None and self.robot_pos_map is None) or \
           ((now - self.last_odom_time > 3.0) and (now - self.last_loc_odom_time > 3.0)):
            return

        # 动态坐标系对齐判断：
        # /vis/lookahead_goal 与规划路径基于 'map' 坐标系。
        # 只要 /loc/robo/odom 在 3 秒内有消息更新，基准位姿严格优先采用 map 坐标系；否则回退至 /lio/robo/odom
        # 彻底消除因 odom 与 map 坐标系相差数米导致的虚假"目标在身后(BEHIND!)"
        use_map_frame = (
            self.robot_pos_map is not None
            and (now - self.last_loc_odom_time <= 3.0)
        )

        if use_map_frame:
            curr_robot_pos = self.robot_pos_map
            curr_robot_yaw = self.robot_yaw_map
            curr_robot_yaw_deg = self.robot_yaw_deg_map
            curr_robot_roll = self.robot_roll_map
            curr_robot_pitch = self.robot_pitch_map
        else:
            curr_robot_pos = self.robot_pos
            curr_robot_yaw = self.robot_yaw
            curr_robot_yaw_deg = self.robot_yaw_deg
            curr_robot_roll = self.robot_roll
            curr_robot_pitch = self.robot_pitch

        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 深度图转换失败: {e}', throttle_duration_sec=2.0)
            return

        # 1. 严格复现 rl_nav_controller.py 策略使用的 40x64 深度矩阵预处理
        try:
            is_nan = np.isnan(depth_raw)
            is_posinf = np.isposinf(depth_raw)
            is_neginf = np.isneginf(depth_raw)

            far_mask = is_posinf | (depth_raw > self.max_depth)
            close_mask = is_neginf | ((depth_raw < self.min_depth) & ~is_nan & ~far_mask)
            nan_mask = is_nan

            depth_clamped = depth_raw.copy()
            depth_clamped[far_mask] = 0.0
            depth_clamped[close_mask] = self.min_depth
            depth_clamped[nan_mask] = self.min_depth

            # 下采样至 40x64 (模型实际输入)
            depth_40x64 = cv2.resize(depth_clamped, (64, 40), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().error(f'深度图预处理异常: {e}')
            return

        # 2. 计算相对机器人本体坐标系的完整 3D 目标向量与运动学
        is_carrot_valid = False
        target_dist_2d = None
        target_dist_3d = None
        target_yaw_deg = None
        target_pitch_deg = None
        target_quadrant = "NONE"
        dx_b = 0.0
        dy_b = 0.0
        dz_b = 0.0

        if self.carrot_pos is not None and (now - self.last_carrot_time <= 3.0):
            try:
                delta_w = self.carrot_pos - curr_robot_pos
                robot_yaw = curr_robot_yaw
                
                # 转换至机器人偏航基坐标系 (Forward=X, Left=Y, Up=Z)
                dx_b = math.cos(-robot_yaw) * delta_w[0] - math.sin(-robot_yaw) * delta_w[1]
                dy_b = math.sin(-robot_yaw) * delta_w[0] + math.cos(-robot_yaw) * delta_w[1]
                dz_b = delta_w[2]
                
                dist_2d = math.hypot(dx_b, dy_b)
                dist_3d = math.hypot(dist_2d, dz_b)
                angle_val = math.degrees(math.atan2(dy_b, dx_b))
                pitch_val = math.degrees(math.atan2(dz_b, dist_2d)) if dist_2d > 1e-4 else 0.0
                
                if math.isfinite(dist_2d) and math.isfinite(angle_val):
                    target_dist_2d = dist_2d
                    target_dist_3d = dist_3d
                    target_yaw_deg = angle_val
                    target_pitch_deg = pitch_val
                    is_carrot_valid = True

                    # 目标方位象限细分 (严格依据基坐标系前向分量 dx_b 正负判定身前/身后)
                    if dx_b >= 0.0:
                        if abs(angle_val) <= 30.0:
                            target_quadrant = "FRONT"
                        elif angle_val > 0.0:
                            target_quadrant = "FRONT_LEFT"
                        else:
                            target_quadrant = "FRONT_RIGHT"
                    else:
                        if abs(angle_val) >= 150.0:
                            target_quadrant = "BEHIND"
                        elif angle_val > 0.0:
                            target_quadrant = "BEHIND_LEFT"
                        else:
                            target_quadrant = "BEHIND_RIGHT"
            except Exception:
                is_carrot_valid = False

        # 3. 计算多区域空间净空与深渊占比 (40x64 矩阵细分统计)
        try:
            # 左侧扇区 (cols 0:18, rows 10:35)
            left_roi = depth_40x64[10:35, 0:18]
            valid_left = left_roi[left_roi > self.min_depth]
            depth_left_min = float(np.min(valid_left)) if len(valid_left) > 0 else float(np.min(left_roi))
            depth_left_avg = float(np.mean(left_roi))

            # 中央走廊 (cols 20:44, rows 10:35)
            center_roi = depth_40x64[10:35, 20:44]
            valid_center = center_roi[center_roi > self.min_depth]
            depth_center_min = float(np.min(valid_center)) if len(valid_center) > 0 else float(np.min(center_roi))
            depth_center_avg = float(np.mean(center_roi))

            # 右侧扇区 (cols 46:64, rows 10:35)
            right_roi = depth_40x64[10:35, 46:64]
            valid_right = right_roi[right_roi > self.min_depth]
            depth_right_min = float(np.min(valid_right)) if len(valid_right) > 0 else float(np.min(right_roi))
            depth_right_avg = float(np.mean(right_roi))

            # 近地地面 (cols 20:44, rows 32:40)
            floor_roi = depth_40x64[32:40, 20:44]
            valid_floor = floor_roi[floor_roi > self.min_depth]
            depth_floor_near = float(np.mean(valid_floor)) if len(valid_floor) > 0 else 0.0

            # 阶梯虚空/深渊占比 (cols 20:44, rows 15:38)
            void_window = depth_40x64[15:38, 20:44]
            center_void_ratio = float(np.mean((void_window == 0.0) | (void_window > 2.5)))

            # 原版中央门洞 ROI (cols 26:38, rows 15:25) - 保持 100% 兼容
            door_roi = depth_40x64[15:25, 26:38]
            valid_door = door_roi[door_roi > self.min_depth]
            door_center_min_depth = float(np.min(valid_door)) if len(valid_door) > 0 else float(np.min(door_roi))
            door_center_avg_depth = float(np.mean(door_roi))
        except Exception:
            depth_left_min = depth_left_avg = 0.0
            depth_center_min = depth_center_avg = 0.0
            depth_right_min = depth_right_avg = 0.0
            depth_floor_near = 0.0
            center_void_ratio = 0.0
            door_center_min_depth = door_center_avg_depth = 0.0

        # 4. 重映射深度图与楼梯感知状态
        remapped_active = False
        remapped_center_min = 0.0
        remapped_center_avg = 0.0
        remapped_40x64 = None
        remapped_heatmap = None

        if self.record_remapped and (self.latest_remapped_depth is not None) and (now - self.last_remapped_time <= 0.6):
            remapped_active = True
            remapped_40x64 = self.latest_remapped_depth.copy()
            try:
                r_roi = remapped_40x64[10:35, 20:44]
                valid_r = r_roi[r_roi > self.min_depth]
                remapped_center_min = float(np.min(valid_r)) if len(valid_r) > 0 else float(np.min(r_roi))
                remapped_center_avg = float(np.mean(r_roi))

                # 生成重映射热力图
                norm_r = np.clip(remapped_40x64 / 5.0, 0.0, 1.0)
                h_remap = cv2.applyColorMap((norm_r * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
                remapped_heatmap = cv2.resize(h_remap, (320, 200), interpolation=cv2.INTER_NEAREST)
                cv2.putText(remapped_heatmap, "REMAPPED VIRTUAL STAIRS", (10, 25), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(remapped_heatmap, f"Center: {remapped_center_min:.2f}m", (10, 185), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            except Exception:
                pass

        # 5. 全局路径前瞻与下楼坡度诊断
        path_points_count = 0
        path_dist_to_end = None
        path_slope_ahead = None

        if self.global_path_pts is not None and (now - self.last_path_time <= 5.0):
            try:
                path_points_count = len(self.global_path_pts)
                if curr_robot_pos is not None:
                    # 到终点直线距离
                    end_pt = self.global_path_pts[-1]
                    path_dist_to_end = float(np.linalg.norm(end_pt - curr_robot_pos))

                    # 提取身前 0.4m ~ 2.2m 窗口内的路径点，计算下倾坡度
                    dists_2d = np.linalg.norm(self.global_path_pts[:, :2] - curr_robot_pos[:2], axis=1)
                    win_mask = (dists_2d >= 0.4) & (dists_2d <= 2.2)
                    if np.any(win_mask):
                        win_pts = self.global_path_pts[win_mask]
                        win_dists = dists_2d[win_mask]
                        dz_win = win_pts[-1, 2] - curr_robot_pos[2]
                        dxy_win = win_dists[-1]
                        if dxy_win > 0.1:
                            path_slope_ahead = float(dz_win / dxy_win)
            except Exception:
                pass

        # 6. 生成直观诊断热力图（伪彩色 + 扇区标注 + 状态横幅）
        try:
            norm_img = np.clip(depth_40x64 / 5.0, 0.0, 1.0)
            heatmap = cv2.applyColorMap((norm_img * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            heatmap_large = cv2.resize(heatmap, (320, 200), interpolation=cv2.INTER_NEAREST)

            # 绘制多扇区几何框 (320x200 对应 40x64 放大 5 倍)
            # 左侧: cols 0:18 (x: 0~90) | 中央: cols 20:44 (x: 100~220) | 右侧: cols 46:64 (x: 230~320)
            # y: rows 10:35 (y: 50~175)
            cv2.rectangle(heatmap_large, (100, 50), (220, 175), (255, 255, 0), 1)  # 中央走廊
            cv2.line(heatmap_large, (100, 160), (220, 160), (0, 255, 255), 1)      # 近地分界线
            cv2.rectangle(heatmap_large, (130, 75), (190, 125), (200, 200, 200), 1) # 原门洞 ROI

            # 楼梯感知角标 (右上角)
            stair_active = self.is_descending_stairs and (now - self.last_stair_time <= 1.0)
            if stair_active:
                cv2.rectangle(heatmap_large, (185, 3), (317, 24), (200, 120, 0), -1)
                cv2.putText(heatmap_large, "[STAIRS ACTIVE]", (190, 18), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1)
            else:
                cv2.rectangle(heatmap_large, (215, 3), (317, 22), (40, 40, 40), -1)
                cv2.putText(heatmap_large, "[FLAT GROUND]", (220, 17), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)

            # 目标向量指示与身后掉头警示
            if is_carrot_valid:
                center_x = 160
                if abs(target_yaw_deg) <= 90.0:
                    # 目标在身前：正常绘制前向导引箭头
                    raw_arrow_x = center_x + (-dy_b * 40.0)
                    safe_arrow_x = int(np.clip(raw_arrow_x, 20.0, 300.0))
                    cv2.arrowedLine(heatmap_large, (160, 180), (safe_arrow_x, 48), (255, 255, 255), 2, tipLength=0.25)
                    tgt_text = f"Tgt: {target_dist_2d:.2f}m @ {target_yaw_deg:+.0f}d | dz:{dz_b:+.2f}m"
                    cv2.putText(heatmap_large, tgt_text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
                else:
                    # 目标在身后！醒目红黄背景横幅警告，避免误判
                    cv2.rectangle(heatmap_large, (4, 4), (180, 34), (0, 0, 180), -1)
                    cv2.putText(heatmap_large, f"BEHIND! {target_yaw_deg:+.0f}deg", (8, 18), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
                    cv2.putText(heatmap_large, f"[{target_quadrant}] {target_dist_2d:.1f}m", (8, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1)
                    # 绘制掉头旋转指引箭头
                    turn_dir_x = 40 if target_yaw_deg > 0 else 280
                    cv2.arrowedLine(heatmap_large, (160, 100), (turn_dir_x, 170), (0, 0, 255), 2, tipLength=0.3)
            else:
                cv2.putText(heatmap_large, "Tgt: WAITING /vis/lookahead_goal", (8, 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)

            # 底部叠加控制与实测对比，以及三区空间净空
            cv2.rectangle(heatmap_large, (0, 162), (320, 200), (10, 10, 10), -1)
            clearance_text = f"L:{depth_left_min:.1f}m | C:{depth_center_min:.1f}m | R:{depth_right_min:.1f}m | Void:{center_void_ratio*100:.0f}%"
            cv2.putText(heatmap_large, clearance_text, (6, 177), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 220, 100), 1)

            vel_text = f"Cmd:[{self.last_cmd[0]:.2f}, {self.last_cmd[2]:+.2f}] | Odom:[{self.odom_vx:.2f}, {self.odom_wz:+.2f}]"
            cv2.putText(heatmap_large, vel_text, (6, 194), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 120), 1)

        except Exception as e:
            self.get_logger().error(f'热力图生成异常: {e}')
            return

        # 7. 打包任务进入后台写盘队列
        self.frame_idx += 1
        self.last_record_epoch = now
        frame_name = f"frame_{self.frame_idx:06d}"
        timestamp_iso = datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        meta_dict = {
            "frame_idx": self.frame_idx,
            "timestamp_iso": timestamp_iso,
            "epoch_sec": now,
            "robot_pos": curr_robot_pos.tolist(),
            "robot_frame": "map" if use_map_frame else "odom",
            "robot_pos_odom": self.robot_pos.tolist() if self.robot_pos is not None else None,
            "robot_pos_map": self.robot_pos_map.tolist() if self.robot_pos_map is not None else None,
            "robot_attitude": {
                "roll_deg": curr_robot_roll,
                "pitch_deg": curr_robot_pitch,
                "yaw_deg": curr_robot_yaw_deg
            },
            "odom_velocity": {
                "vx": self.odom_vx,
                "vy": self.odom_vy,
                "vz": self.odom_vz,
                "wz": self.odom_wz
            },
            "carrot_pos": self.carrot_pos.tolist() if self.carrot_pos is not None else None,
            "carrot_source": self.carrot_source,
            "is_carrot_valid": is_carrot_valid,
            "target_3d": {
                "dx_b": dx_b,
                "dy_b": dy_b,
                "dz_b": dz_b,
                "dist_2d": target_dist_2d,
                "dist_3d": target_dist_3d,
                "yaw_deg": target_yaw_deg,
                "pitch_deg": target_pitch_deg,
                "quadrant": target_quadrant
            },
            "cmd_vel": self.last_cmd,
            "clearance_partitions": {
                "left_min": depth_left_min,
                "left_avg": depth_left_avg,
                "center_min": depth_center_min,
                "center_avg": depth_center_avg,
                "right_min": depth_right_min,
                "right_avg": depth_right_avg,
                "floor_near": depth_floor_near,
                "center_void_ratio": center_void_ratio,
                "door_center_min": door_center_min_depth,
                "door_center_avg": door_center_avg_depth
            },
            "stairs_diagnostics": {
                "is_descending_stairs": stair_active,
                "remapped_active": remapped_active,
                "remapped_center_min": remapped_center_min,
                "remapped_center_avg": remapped_center_avg
            },
            "path_diagnostics": {
                "points_count": path_points_count,
                "dist_to_end": path_dist_to_end,
                "slope_ahead": path_slope_ahead
            }
        }

        # 严格遵守 46 列 CSV 表结构 (前 19 列与旧版完全一致)
        csv_row = [
            # 0-18: 旧版字段
            self.frame_idx, timestamp_iso, f"{now:.3f}",
            f"{curr_robot_pos[0]:.3f}", f"{curr_robot_pos[1]:.3f}", f"{curr_robot_pos[2]:.3f}",
            f"{curr_robot_yaw_deg:.1f}",
            f"{self.carrot_pos[0]:.3f}" if self.carrot_pos is not None else "",
            f"{self.carrot_pos[1]:.3f}" if self.carrot_pos is not None else "",
            f"{self.carrot_pos[2]:.3f}" if self.carrot_pos is not None else "",
            f"{target_dist_2d:.3f}" if target_dist_2d is not None else "",
            f"{target_yaw_deg:.1f}" if target_yaw_deg is not None else "",
            f"{self.last_cmd[0]:.2f}", f"{self.last_cmd[1]:.2f}", f"{self.last_cmd[2]:.2f}",
            f"{door_center_min_depth:.3f}", f"{door_center_avg_depth:.3f}",
            "1" if is_carrot_valid else "0",
            self.carrot_source,

            # 19-24: 姿态角与实测速度
            f"{curr_robot_roll:.1f}", f"{curr_robot_pitch:.1f}",
            f"{self.odom_vx:.3f}", f"{self.odom_vy:.3f}", f"{self.odom_vz:.3f}", f"{self.odom_wz:.3f}",

            # 25-30: 3D 目标相对体坐标系分量与运动学
            f"{dx_b:.3f}", f"{dy_b:.3f}", f"{dz_b:.3f}",
            f"{target_dist_3d:.3f}" if target_dist_3d is not None else "",
            f"{target_pitch_deg:.1f}" if target_pitch_deg is not None else "",
            target_quadrant,

            # 31-38: 多区域空间净空与深渊占比
            f"{depth_left_min:.3f}", f"{depth_left_avg:.3f}",
            f"{depth_center_min:.3f}", f"{depth_center_avg:.3f}",
            f"{depth_right_min:.3f}", f"{depth_right_avg:.3f}",
            f"{depth_floor_near:.3f}", f"{center_void_ratio:.3f}",

            # 39-42: 楼梯感知门控与重映射深度
            "1" if stair_active else "0",
            "1" if remapped_active else "0",
            f"{remapped_center_min:.3f}", f"{remapped_center_avg:.3f}",

            # 43-45: 全局路径前瞻与坡度诊断
            path_points_count,
            f"{path_dist_to_end:.3f}" if path_dist_to_end is not None else "",
            f"{path_slope_ahead:.3f}" if path_slope_ahead is not None else ""
        ]

        task = (frame_name, depth_40x64, heatmap_large, remapped_40x64, remapped_heatmap, meta_dict, csv_row)
        try:
            self.write_queue.put_nowait(task)
        except queue.Full:
            self.dropped_queue_frames += 1
            self.get_logger().warn('写盘队列已满，为保证系统不卡顿丢弃该帧！', throttle_duration_sec=3.0)

        # 实时终端进度刷新 (每秒输出一次)
        if self.verbose or (now - self.last_terminal_print >= 1.0):
            self.last_terminal_print = now
            tgt_display = f"{target_dist_2d:.2f}m@{target_yaw_deg:+.0f}°" if is_carrot_valid else "等待目标中..."
            stair_badge = "\033[96m[STAIR:ON]\033[0m" if stair_active else "[FLAT]"
            behind_warn = " \033[91m[BEHIND!]\033[0m" if (is_carrot_valid and abs(target_yaw_deg) > 90.0) else ""

            sys.stdout.write(
                f"\r[REC #{self.frame_idx:04d}] {timestamp_iso[-12:]} | "
                f"目标: {tgt_display:<13}{behind_warn} | "
                f"{stair_badge} | "
                f"净空(L/C/R): {depth_left_min:.1f}/{depth_center_min:.1f}/{depth_right_min:.1f}m | "
                f"控制: [vx={self.last_cmd[0]:.2f}, wz={self.last_cmd[2]:+.2f}] | "
                f"实测: [vx={self.odom_vx:.2f}, wz={self.odom_wz:+.2f}] | "
                f"待写: {self.write_queue.qsize():<2}"
            )
            sys.stdout.flush()

    def status_heartbeat_cb(self):
        """定期输出传感器就绪状态"""
        now = time.time()
        if self.frame_idx == 0:
            odom_ok = (self.robot_pos is not None) and (now - self.last_odom_time < 3.0)
            depth_ok = (self.depth_msg_count > 0)
            carrot_ok = (self.carrot_pos is not None) and (now - self.last_carrot_time < 3.0)
            stair_topic_active = (now - self.last_stair_time < 5.0)

            odom_str = "\033[92mOK\033[0m" if odom_ok else "\033[91m等待中 (/lio/robo/odom)\033[0m"
            depth_str = "\033[92mOK\033[0m" if depth_ok else "\033[91m等待中 (/zed/.../depth_registered)\033[0m"
            carrot_str = "\033[92mOK\033[0m" if carrot_ok else "\033[93m等待中 (/vis/lookahead_goal)\033[0m"
            stair_str = "\033[92mOK\033[0m" if stair_topic_active else "\033[90m未激活 (/nav/is_descending_stairs)\033[0m"

            sys.stdout.write(
                f"\r[就绪检查] 里程计=[{odom_str}], 深度图=[{depth_str}], 目标点=[{carrot_str}], 楼梯通道=[{stair_str}]   "
            )
            sys.stdout.flush()

    def _writer_worker(self):
        """后台独立磁盘写入线程"""
        while not self.stop_worker_event.is_set() or not self.write_queue.empty():
            try:
                task = self.write_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            frame_name, depth_40x64, heatmap_large, remapped_40x64, remapped_heatmap, meta_dict, csv_row = task
            try:
                # 1. 保存原始 40x64 深度矩阵
                npy_path = os.path.join(self.depth_dir, f"{frame_name}_depth_40x64.npy")
                np.save(npy_path, depth_40x64)

                # 2. 保存直观伪彩色热力图
                png_path = os.path.join(self.heatmap_dir, f"{frame_name}_heatmap.png")
                cv2.imwrite(png_path, heatmap_large)

                # 3. 若有重映射深度图，同步保存
                if remapped_40x64 is not None:
                    remap_npy_path = os.path.join(self.remapped_depth_dir, f"{frame_name}_remapped_40x64.npy")
                    np.save(remap_npy_path, remapped_40x64)

                if remapped_heatmap is not None:
                    remap_png_path = os.path.join(self.remapped_heatmap_dir, f"{frame_name}_remapped_heatmap.png")
                    cv2.imwrite(remap_png_path, remapped_heatmap)

                # 4. 保存详细元数据 JSON
                json_path = os.path.join(self.meta_dir, f"{frame_name}_meta.json")
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(meta_dict, f, indent=2, ensure_ascii=False)

                # 5. 写入轨迹 CSV 汇总表
                self.csv_writer.writerow(csv_row)
                self.csv_file.flush()
            except Exception:
                pass
            finally:
                self.write_queue.task_done()

    def close(self):
        """安全停止录制并保存会话概况报告"""
        print("\n\n正在收尾保存数据，请稍候...")
        self.stop_worker_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=5.0)

        try:
            self.csv_file.close()
        except Exception:
            pass

        duration = max(0.1, time.time() - self.start_epoch)
        avg_fps = self.frame_idx / duration

        summary = {
            "session_name": self.session_name,
            "session_dir": self.session_dir,
            "start_time": datetime.datetime.fromtimestamp(self.start_epoch).strftime("%Y-%m-%d %H:%M:%S"),
            "end_time": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "duration_sec": round(duration, 2),
            "total_frames_recorded": self.frame_idx,
            "average_fps": round(avg_fps, 2),
            "dropped_queue_frames": self.dropped_queue_frames,
            "record_rate_setting": self.record_rate,
            "total_csv_columns": len(self.csv_headers),
            "latest_link": self.latest_link
        }

        summary_path = os.path.join(self.session_dir, 'session_summary.json')
        try:
            with open(summary_path, 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

        print('\033[92m' + '='*75 + '\033[0m')
        print(f"\033[92m[✓] 综合导航与楼梯诊断录制完成！\033[0m")
        print(f"  📁 本次会话目录: {self.session_dir}")
        print(f"  🔗 快捷软链接:   {self.latest_link}")
        print(f"  📊 累计有效录制: {self.frame_idx} 帧 (持续 {duration:.1f} 秒, 平均 {avg_fps:.1f} FPS)")
        if self.dropped_queue_frames > 0:
            print(f"  ⚠️ 队列丢帧:     {self.dropped_queue_frames} 帧 (磁盘偏慢)")
        else:
            print(f"  ✓ 队列状态:     完全零丢帧")
        print(f"  📋 结构化数据明细:")
        print(f"     - 原始热力图:   {self.heatmap_dir}/*.png")
        print(f"     - 原始深度矩阵: {self.depth_dir}/*.npy")
        if self.record_remapped:
            print(f"     - 重映射深度图: {self.remapped_depth_dir}/*.npy")
            print(f"     - 重映射热力图: {self.remapped_heatmap_dir}/*.png")
        print(f"     - 单帧综合元数据: {self.meta_dir}/*.json")
        print(f"     - 全程时序轨迹表: {self.csv_path} ({len(self.csv_headers)} 列指标)")
        print(f"     - 会话概况信息:   {summary_path}")
        print('\033[92m' + '='*75 + '\033[0m')


def main():
    parser = argparse.ArgumentParser(description="Comprehensive Navigation & Stairs Diagnostic Recorder")
    parser.add_argument('tag', nargs='?', default='', help='可选会话标签 (例如 stairs_run1 或 door_0.5m)')
    parser.add_argument('--rate', '-r', type=float, default=5.0, help='录制目标频率 (Hz，默认 5.0)')
    parser.add_argument('--dir', '-d', type=str, default='~/door_diagnostics', help='存储基础目录 (默认 ~/door_diagnostics)')
    parser.add_argument('--verbose', '-v', action='store_true', help='开启逐帧详细打印')
    parser.add_argument('--no-remapped', action='store_true', help='关闭重映射深度图保存')
    parser.add_argument('--path-topic', type=str, default='/global_path', help='全局规划路径 Topic (默认 /global_path)')

    parsed, unknown = parser.parse_known_args()

    rclpy.init()
    node = DoorDiagnosticLogger(
        base_dir=parsed.dir,
        session_tag=parsed.tag,
        record_rate=parsed.rate,
        verbose=parsed.verbose,
        record_remapped=not parsed.no_remapped,
        path_topic=parsed.path_topic
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

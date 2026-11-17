#!/usr/bin/env python3
"""
Camera Comparison & Navigation Diagnostic Recorder (schema 2.0)
================================================================
用于 ZED 2i / Intel RealSense D435 在同一路径、同一部署参数下的定量对比。

相比旧版（重映射楼梯诊断版）的变化：
1. 【移除】所有 `/nav/remapped_depth`、`depth_remapped/`、`heatmaps_remapped/` 采集。
2. 【新增】相机身份、深度话题、编码、原生分辨率、CameraInfo、TF 外参。
3. 【新增】深度健康度：有效率/空洞率/NaN/零值/近距/远距占比、分位数、延迟、帧率。
4. 【新增】原始深度留存（16-bit PNG，按最大宽度 320 等比缩放），可离线复算任意预处理。
5. 【新增】导航状态 `/local_planner/status`、`/motion_enabled`、`/startNavi`、
   `/nav/is_ascending_stairs`，以及可选的 `/nav/adaptive_debug`、`/nav/stairs_debug`。
6. 【新增】LiDAR 机体前向净空真值（PointCloud2 或 Livox CustomMsg，自动 TF 到 base_link）。
7. 【新增】目标点/路径快照：`goals.jsonl`、`paths/path_*.json`，用于复核两次路径是否一致。
8. 【新增】控制参数快照：启动后自动 `ros2 param dump` 控制器参数，保证"当前部署参数"可追溯。
9. 【新增】会话级 `session_meta.json` / `session_summary.json`，聚合相机、深度、速度、路径指标。

策略输入严格复现 `rl_nav_controller.py: depth_callback` 的预处理：
- 16UC1/mono16 -> 米；32FC1 -> 米；
- `> max_depth` / `+inf` -> 0.0；`< min_depth` / `NaN` / `-inf` -> min_depth；
- `cv2.resize(..., (64,40), INTER_LINEAR)`。

用法示例：
    # RealSense（当前默认部署参数：min=0.15, max=10.0）
    python3 door_diagnostic_logger.py --camera realsense --scene S3_door --note "RS run1"

    # ZED（当前默认部署参数：min=0.25, max=10.0）
    python3 door_diagnostic_logger.py --camera zed --scene S3_door --note "ZED run1"

    # 自动识别（同时监听两路，哪路有数据记哪路）
    python3 door_diagnostic_logger.py --scene S1_corridor

    # 显式覆盖量程/话题
    python3 door_diagnostic_logger.py --camera realsense --min-depth 0.15 --max-depth 6.0 \
        --depth-topic /camera/camera/depth/image_rect_raw --rate 5
"""

import os
import re
import sys
import csv
import json
import math
import time
import queue
import shutil
import socket
import getpass
import hashlib
import argparse
import datetime
import threading
import subprocess

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge

try:
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:  # pragma: no cover
    pc2 = None

try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:  # pragma: no cover
    CustomMsg = None

try:
    import tf2_ros
except ImportError:  # pragma: no cover
    tf2_ros = None


SCHEMA_VERSION = "2.0"

REALSENSE_DEPTH_TOPIC = '/camera/camera/depth/image_rect_raw'
REALSENSE_DEPTH_TOPIC_ALT = '/camera/depth/image_rect_raw'
ZED_DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'

REALSENSE_INFO_TOPIC = '/camera/camera/depth/camera_info'
ZED_INFO_TOPIC = '/zed/zed_node/depth/camera_info'

DEFAULT_STATUS_TOPIC = '/local_planner/status'
DEFAULT_PATH_TOPIC = '/global_path'
DEFAULT_GOAL_TOPIC = '/goal_pose'
DEFAULT_CMD_TOPIC = '/cmd_vel'
DEFAULT_RAW_CMD_TOPIC = '/path_manager/path_manager_ros/nav_vel'
DEFAULT_LIDAR_TOPIC = '/livox/points'      # PointCloud2（若驱动/中间件提供）
DEFAULT_LIVOX_TOPIC = '/livox/lidar'       # livox_ros_driver2/CustomMsg
DEFAULT_IMU_TOPIC = '/livox/imu'
DEFAULT_RGB_TOPIC = '/camera/camera/color/image_raw'

CAMERA_DEFAULTS = {
    'realsense': {
        'depth_topic': REALSENSE_DEPTH_TOPIC,
        'info_topic': REALSENSE_INFO_TOPIC,
        'camera_frame': 'camera_link',
        'min_depth': 0.15,
        'max_depth': 10.0,
    },
    'zed': {
        'depth_topic': ZED_DEPTH_TOPIC,
        'info_topic': ZED_INFO_TOPIC,
        'camera_frame': 'zed_camera_link',
        'min_depth': 0.25,
        'max_depth': 10.0,
    },
}


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def _safe_float(value, default=None):
    try:
        if value is None or value == '':
            return default
        v = float(value)
        return v if math.isfinite(v) else default
    except Exception:
        return default


def _yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw if math.isfinite(yaw) else 0.0


def _roll_pitch_yaw(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(float(np.clip(sinp, -1.0, 1.0)))
    yaw = _yaw_from_quat(qx, qy, qz, qw)
    return (
        math.degrees(roll) if math.isfinite(roll) else 0.0,
        math.degrees(pitch) if math.isfinite(pitch) else 0.0,
        math.degrees(yaw) if math.isfinite(yaw) else 0.0,
    )


def _quat_to_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ], dtype=np.float64)


def _camera_id_from_topic(topic):
    if not topic:
        return None
    t = topic.lower()
    if 'zed' in t:
        return 'zed'
    if 'camera' in t or 'realsense' in t:
        return 'realsense'
    return None


def _derive_info_topic(depth_topic):
    if not depth_topic:
        return ''
    if 'image_rect_raw' in depth_topic:
        return depth_topic.replace('/image_rect_raw', '/camera_info')
    if 'depth_registered' in depth_topic:
        return depth_topic.replace('/depth_registered', '/camera_info')
    if 'depth/image' in depth_topic:
        return depth_topic.replace('/image', '/camera_info')
    return ''


def _fmt(value, spec="{:.3f}"):
    if value is None:
        return ""
    try:
        v = float(value)
        if not math.isfinite(v):
            return ""
        return spec.format(v)
    except Exception:
        return ""


def _fmt_int(value):
    if value is None:
        return ""
    try:
        return str(int(value))
    except Exception:
        return ""


def _bool_str(value):
    if value is None:
        return ""
    return "1" if bool(value) else "0"


# ---------------------------------------------------------------------------
# Recorder
# ---------------------------------------------------------------------------

class DoorDiagnosticLogger(Node):
    def __init__(
        self,
        base_dir: str = '~/door_diagnostics',
        session_tag: str = '',
        record_rate: float = 5.0,
        verbose: bool = False,
        path_topic: str = DEFAULT_PATH_TOPIC,
        depth_topic: str = 'auto',
        camera: str = 'auto',
        camera_frame: str = 'auto',
        camera_info_topic: str = '',
        min_depth: float = None,
        max_depth: float = None,
        scene: str = '',
        note: str = '',
        operator: str = '',
        run_id: str = '',
        status_topic: str = DEFAULT_STATUS_TOPIC,
        goal_topic: str = DEFAULT_GOAL_TOPIC,
        cmd_topic: str = DEFAULT_CMD_TOPIC,
        raw_cmd_topic: str = DEFAULT_RAW_CMD_TOPIC,
        lidar_topic: str = DEFAULT_LIDAR_TOPIC,
        livox_topic: str = DEFAULT_LIVOX_TOPIC,
        enable_lidar: bool = True,
        imu_topic: str = DEFAULT_IMU_TOPIC,
        enable_imu: bool = True,
        param_node: str = 'auto',
        save_raw_depth: bool = True,
        raw_depth_width: int = 320,
        raw_depth_every: int = 1,
        record_rgb: bool = False,
        rgb_topic: str = DEFAULT_RGB_TOPIC,
    ):
        super().__init__('door_diagnostic_logger')

        self.bridge = CvBridge()
        self.record_rate = max(0.5, float(record_rate))
        self.verbose = verbose
        self.path_topic = path_topic
        self.status_topic = status_topic
        self.goal_topic = goal_topic
        self.cmd_topic = cmd_topic
        self.raw_cmd_topic = raw_cmd_topic
        self.lidar_topic = lidar_topic
        self.livox_topic = livox_topic
        self.enable_lidar = bool(enable_lidar)
        self.imu_topic = imu_topic
        self.enable_imu = bool(enable_imu)
        self.param_node = param_node
        self.save_raw_depth = bool(save_raw_depth)
        self.raw_depth_width = max(80, int(raw_depth_width))
        self.raw_depth_every = max(1, int(raw_depth_every))
        self.record_rgb = bool(record_rgb)
        self.rgb_topic = rgb_topic

        self.scene = scene.strip()
        self.note = note.strip()
        self.operator = operator.strip() or getpass.getuser()
        self.run_id = run_id.strip()

        self.camera_override = camera if camera in ('realsense', 'zed') else None
        self.camera_id = self.camera_override or 'unknown'
        self.depth_topic = depth_topic
        self.active_depth_topic = None
        self.camera_frame = camera_frame
        self.camera_info_topic = camera_info_topic
        self.camera_info = {}
        self.tf_snapshot = {}
        self.controller_params_node = None
        self.controller_params_text = None
        self.controller_min_depth = None
        self.controller_max_depth = None

        # Apply camera-specific defaults so the logger's 40x64 input matches the
        # deployment launch script even when --min-depth/--max-depth are omitted.
        cam_defaults = CAMERA_DEFAULTS.get(self.camera_override, {})
        if self.camera_override:
            if self.depth_topic in ('auto', 'default', ''):
                self.depth_topic = cam_defaults['depth_topic']
            if self.camera_frame in ('', 'auto'):
                self.camera_frame = cam_defaults['camera_frame']
            if not self.camera_info_topic:
                self.camera_info_topic = cam_defaults['info_topic']
        if min_depth is not None:
            self.min_depth = float(min_depth)
            self._min_depth_explicit = True
        else:
            self.min_depth = float(cam_defaults.get('min_depth', 0.25))
            self._min_depth_explicit = False
        if max_depth is not None:
            self.max_depth = float(max_depth)
            self._max_depth_explicit = True
        else:
            self.max_depth = float(cam_defaults.get('max_depth', 10.0))
            self._max_depth_explicit = False

        # 1) Session directory layout
        self.base_dir = os.path.expanduser(base_dir)
        now_str = datetime.datetime.now().strftime("session_%Y%m%d_%H%M%S")
        tag_parts = [p for p in [session_tag, self.scene, self.camera_override or ''] if p]
        safe_tag = "".join(c for c in "_".join(tag_parts) if c.isalnum() or c in ('_', '-'))
        self.session_name = f"{now_str}_{safe_tag}" if safe_tag else now_str

        self.session_dir = os.path.join(self.base_dir, self.session_name)
        self.heatmap_dir = os.path.join(self.session_dir, 'heatmaps')
        self.depth_dir = os.path.join(self.session_dir, 'depth')
        self.depth_raw_dir = os.path.join(self.session_dir, 'depth_raw')
        self.meta_dir = os.path.join(self.session_dir, 'meta')
        self.path_dir = os.path.join(self.session_dir, 'paths')
        self.rgb_dir = os.path.join(self.session_dir, 'rgb')

        dirs = [self.session_dir, self.heatmap_dir, self.depth_dir, self.meta_dir, self.path_dir]
        if self.save_raw_depth:
            dirs.append(self.depth_raw_dir)
        if self.record_rgb:
            dirs.append(self.rgb_dir)
        for d in dirs:
            os.makedirs(d, exist_ok=True)

        # latest symlink
        self.latest_link = os.path.join(self.base_dir, 'latest')
        try:
            if os.path.islink(self.latest_link) or os.path.exists(self.latest_link):
                os.unlink(self.latest_link)
            os.symlink(self.session_name, self.latest_link)
        except Exception:
            pass

        self.run_id = self.run_id or self.session_name
        self.start_epoch = time.time()
        self.start_iso = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # 2) CSV
        self.csv_path = os.path.join(self.session_dir, 'trajectory.csv')
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_headers = [
            # --- Legacy core (kept for continuity with old sessions) ---
            "frame_idx", "timestamp_iso", "epoch_sec",
            "robot_x", "robot_y", "robot_z", "robot_yaw_deg",
            "carrot_x", "carrot_y", "carrot_z",
            "target_dist", "target_angle_deg",
            "cmd_vx", "cmd_vy", "cmd_wz",
            "door_center_min_depth", "door_center_avg_depth",
            "is_carrot_valid", "carrot_source",

            # --- Attitude / velocity ---
            "robot_roll_deg", "robot_pitch_deg",
            "odom_vx", "odom_vy", "odom_vz", "odom_wz",
            "speed_mps",

            # --- Target vector in body frame ---
            "target_dx_b", "target_dy_b", "target_dz_b",
            "target_dist_3d", "target_pitch_deg", "target_quadrant",

            # --- Depth ROI (policy input 40x64) ---
            "depth_left_min", "depth_left_avg",
            "depth_center_min", "depth_center_avg",
            "depth_right_min", "depth_right_avg",
            "depth_floor_near", "center_void_ratio",

            # --- Stairs / path ---
            "is_descending_stairs", "is_ascending_stairs",
            "path_points_count", "path_dist_to_end", "path_slope_ahead",
            "path_length", "path_hash",

            # --- Camera identity / raw depth health ---
            "camera_id", "depth_topic", "depth_encoding",
            "depth_width", "depth_height",
            "depth_fps_est", "depth_period_ms", "depth_latency_ms", "depth_age_ms",
            "depth_valid_ratio_raw", "depth_nan_ratio", "depth_zero_ratio",
            "depth_near_ratio", "depth_far_ratio", "depth_hole_ratio",
            "depth_min", "depth_p10", "depth_median", "depth_mean", "depth_p90", "depth_max",
            "depth_valid_ratio_40x64", "depth_zero_ratio_40x64",
            "min_depth_param", "max_depth_param",

            # --- Navigation state / bridge ---
            "nav_state", "nav_state_msgs",
            "motion_enabled", "start_navi",
            "raw_cmd_vx", "raw_cmd_vy", "raw_cmd_wz",

            # --- Goal ---
            "goal_x", "goal_y", "goal_z", "goal_yaw_deg", "goal_dist", "goal_age_ms", "goal_source",

            # --- LiDAR ground truth ---
            "lidar_min_dist_front", "lidar_min_dist_any",
            "lidar_nearest_x", "lidar_nearest_y", "lidar_nearest_z",
            "lidar_valid", "lidar_frame", "lidar_tf_ok", "lidar_points",

            # --- IMU raw (collision / stair dynamics) ---
            "imu_ax", "imu_ay", "imu_az",
            "imu_gx", "imu_gy", "imu_gz", "imu_valid",

            # --- Adaptive / stairs debug (optional topics) ---
            "adaptive_lookahead", "adaptive_l_curv", "adaptive_l_clear", "adaptive_l_target",
            "adaptive_turn_deg", "adaptive_in_narrow", "adaptive_clearance_w",
            "stair_gate_slope", "stair_gate_dz", "stair_remap_active",

            # --- Misc ---
            "rgb_available",
        ]
        self.csv_writer.writerow(self.csv_headers)
        self.csv_file.flush()

        # 3) Runtime state
        self.robot_pos = None
        self.robot_quat = None
        self.robot_roll = 0.0
        self.robot_pitch = 0.0
        self.robot_yaw = 0.0
        self.robot_yaw_deg = 0.0
        self.last_odom_time = 0.0

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
        self.raw_cmd = [0.0, 0.0, 0.0]
        self.last_raw_cmd_time = 0.0

        self.goal_pos = None
        self.goal_yaw = None
        self.goal_frame = "map"
        self.goal_source = ""
        self.last_goal_time = 0.0

        self.nav_state = ""
        self.nav_state_msgs = ""
        self.nav_state_counts = {}
        self.last_status_time = 0.0

        self.motion_enabled = None
        self.last_motion_enabled_time = 0.0
        self.start_navi = None
        self.last_start_navi_time = 0.0

        self.is_descending_stairs = False
        self.last_stair_time = 0.0
        self.is_ascending_stairs = False
        self.last_stair_up_time = 0.0

        self.adaptive_debug = {}
        self.stairs_debug = {}
        self.last_adaptive_debug_time = 0.0
        self.last_stairs_debug_time = 0.0

        self.global_path_pts = None
        self.global_path_frame = "map"
        self.last_path_time = 0.0
        self.path_hash = ""
        self.path_length = None
        self.path_seq = 0

        self.latest_depth_msg_shape = (0, 0)
        self.depth_msg_count = 0
        self.last_depth_recv = 0.0
        self.depth_period_ms = None
        self.depth_fps_est = None
        self.depth_latency_ms = None
        self.depth_age_ms = None
        self.depth_encoding = ""
        self.depth_width = 0
        self.depth_height = 0
        self.latest_raw_depth_stats = {}
        self.latest_depth_40x64_stats = {}

        self.lidar_stats = {
            'min_dist_front': None,
            'min_dist_any': None,
            'nearest_x': None,
            'nearest_y': None,
            'nearest_z': None,
            'valid': False,
            'frame': '',
            'tf_ok': False,
            'points': 0,
        }
        self.lidar_msg_count = 0
        self.last_lidar_time = 0.0

        self.imu_stats = {
            'ax': None, 'ay': None, 'az': None,
            'gx': None, 'gy': None, 'gz': None,
        }
        self.imu_msg_count = 0
        self.last_imu_time = 0.0

        self.latest_rgb = None
        self.last_rgb_time = 0.0
        self.rgb_msg_count = 0

        self.frame_idx = 0
        self.dropped_queue_frames = 0
        self.last_record_epoch = 0.0
        self.last_terminal_print = 0.0

        self.agg = {
            'frames': 0,
            'valid_ratio_sum': 0.0,
            'hole_ratio_sum': 0.0,
            'nan_ratio_sum': 0.0,
            'zero_ratio_sum': 0.0,
            'latency_sum': 0.0,
            'latency_count': 0,
            'center_min_sum': 0.0,
            'center_avg_sum': 0.0,
            'speed_sum': 0.0,
            'cmd_vx_sum': 0.0,
            'max_speed': 0.0,
            'max_cmd_vx': 0.0,
            'distance': 0.0,
            'lidar_min_sum': 0.0,
            'lidar_count': 0,
            'stair_down_frames': 0,
            'stair_up_frames': 0,
        }
        self.last_record_pose = None
        self.last_record_time = None

        # 4) Async writer
        self.write_queue = queue.Queue(maxsize=600)
        self.stop_worker_event = threading.Event()
        self.worker_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.worker_thread.start()

        # 5) TF buffer (optional)
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None:
            try:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            except Exception as e:
                self.get_logger().warn(f'TF listener 初始化失败（不影响记录）: {e}')

        # 6) Subscriptions
        self._setup_subscriptions()

        # 7) Timers
        self.create_timer(1.0, self._status_heartbeat_cb)
        self.create_timer(2.0, self._tf_snapshot_cb)

        if self.enable_lidar and pc2 is None and CustomMsg is None:
            self.get_logger().warn(
                'LiDAR 订阅不可用：sensor_msgs_py 与 livox_ros_driver2 均未导入；'
                'lidar_* 字段将为空。可用 --no-lidar 关闭。'
            )

        # 8) Session metadata + param snapshot
        self._write_session_meta()
        self._write_args_snapshot()
        threading.Thread(target=self._param_snapshot_worker, daemon=True).start()

        self.get_logger().info('\033[92m' + '=' * 78 + '\033[0m')
        self.get_logger().info('\033[92m[Camera Comparison Diagnostic Logger] 运行就绪\033[0m')
        self.get_logger().info(f'  会话目录: {self.session_dir}')
        self.get_logger().info(f'  相机: {self.camera_id} | 深度话题: {self.depth_topic}')
        self.get_logger().info(
            f'  量程: [{self.min_depth:.2f}, {self.max_depth:.2f}] m | 采集: {self.record_rate:.1f} Hz'
        )
        self.get_logger().info(f'  CSV: {self.csv_path} ({len(self.csv_headers)} 列)')
        self.get_logger().info(
            f'  LiDAR: {self.lidar_topic} / {self.livox_topic} | 原始深度: {self.save_raw_depth}'
        )
        self.get_logger().info('\033[92m' + '=' * 78 + '\033[0m')

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _setup_subscriptions(self):
        self.create_subscription(Odometry, '/lio/robo/odom', self.odom_cb, 10)
        self.create_subscription(Odometry, '/loc/robo/odom', self.loc_odom_cb, 10)
        self.create_subscription(Marker, '/vis/lookahead_goal', self.carrot_cb, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_pose_cb, 10)
        self.create_subscription(PoseStamped, '/cur_goal', self.goal_pose_cb, 10)
        self.create_subscription(PoseStamped, '/goal_3d', self.goal_pose_cb, 10)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 10)
        self.create_subscription(Twist, self.raw_cmd_topic, self.raw_cmd_cb, 10)
        self.create_subscription(Bool, '/nav/is_descending_stairs', self.stair_state_cb, 10)
        self.create_subscription(Bool, '/nav/is_ascending_stairs', self.stair_up_state_cb, 10)
        motion_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, '/motion_enabled', self.motion_enabled_cb, motion_qos)
        self.create_subscription(Bool, '/startNavi', self.start_navi_cb, 10)
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)

        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, self.status_topic, self.status_cb, status_qos)
        self.create_subscription(String, '/nav/adaptive_debug', self.adaptive_debug_cb, 10)
        self.create_subscription(String, '/nav/stairs_debug', self.stairs_debug_cb, 10)

        # Depth camera: auto listen to both, or the explicitly requested topic.
        if self.camera_override == 'realsense':
            depth_topics = [CAMERA_DEFAULTS['realsense']['depth_topic']]
        elif self.camera_override == 'zed':
            depth_topics = [CAMERA_DEFAULTS['zed']['depth_topic']]
        elif self.depth_topic and self.depth_topic not in ('auto', 'default'):
            depth_topics = [self.depth_topic]
        else:
            depth_topics = [
                REALSENSE_DEPTH_TOPIC,
                REALSENSE_DEPTH_TOPIC_ALT,
                ZED_DEPTH_TOPIC,
            ]

        for topic in dict.fromkeys(depth_topics):
            self.create_subscription(
                Image, topic,
                lambda msg, t=topic: self.depth_cb(msg, t),
                qos_profile_sensor_data,
            )

        # CameraInfo: derive once the camera is known.
        info_topic = self.camera_info_topic
        if not info_topic:
            if self.camera_override:
                info_topic = CAMERA_DEFAULTS[self.camera_override]['info_topic']
            elif self.depth_topic not in ('auto', 'default', ''):
                info_topic = _derive_info_topic(self.depth_topic)
        if info_topic:
            self.camera_info_topic = info_topic
            self.create_subscription(
                CameraInfo, info_topic,
                lambda msg, t=info_topic: self.camera_info_cb(msg, t),
                qos_profile_sensor_data,
            )
        elif self.camera_override is None and self.depth_topic in ('auto', 'default', ''):
            # Auto mode: listen to both, the active camera fills the metadata.
            for t in (REALSENSE_INFO_TOPIC, ZED_INFO_TOPIC):
                self.create_subscription(
                    CameraInfo, t,
                    lambda msg, topic=t: self.camera_info_cb(msg, topic),
                    qos_profile_sensor_data,
                )

        # LiDAR: PointCloud2 topic + optional Livox CustomMsg topic.
        if self.enable_lidar:
            if pc2 is not None and self.lidar_topic:
                self.create_subscription(
                    PointCloud2, self.lidar_topic, self.lidar_cb, qos_profile_sensor_data
                )
            if CustomMsg is not None and self.livox_topic:
                self.create_subscription(
                    CustomMsg, self.livox_topic, self.livox_cb, qos_profile_sensor_data
                )

        if self.enable_imu and self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self.imu_cb, qos_profile_sensor_data)

        if self.record_rgb and self.rgb_topic:
            self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_profile_sensor_data)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def odom_cb(self, msg: Odometry):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.robot_pos = np.array([p.x, p.y, p.z], dtype=float)
            self.robot_quat = np.array([q.w, q.x, q.y, q.z], dtype=float)
            self.robot_roll, self.robot_pitch, self.robot_yaw_deg = _roll_pitch_yaw(
                q.x, q.y, q.z, q.w
            )
            self.robot_yaw = math.radians(self.robot_yaw_deg)
            self.odom_vx = float(msg.twist.twist.linear.x)
            self.odom_vy = float(msg.twist.twist.linear.y)
            self.odom_vz = float(msg.twist.twist.linear.z)
            self.odom_wz = float(msg.twist.twist.angular.z)
            self.last_odom_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'里程计解析异常: {e}', throttle_duration_sec=2.0)

    def loc_odom_cb(self, msg: Odometry):
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.robot_pos_map = np.array([p.x, p.y, p.z], dtype=float)
            self.robot_quat_map = np.array([q.w, q.x, q.y, q.z], dtype=float)
            self.robot_roll_map, self.robot_pitch_map, self.robot_yaw_deg_map = _roll_pitch_yaw(
                q.x, q.y, q.z, q.w
            )
            self.robot_yaw_map = math.radians(self.robot_yaw_deg_map)
            self.last_loc_odom_time = time.time()
        except Exception as e:
            self.get_logger().warn(f'定位里程计解析异常: {e}', throttle_duration_sec=2.0)

    def carrot_cb(self, msg: Marker):
        try:
            p = msg.pose.position
            self.carrot_pos = np.array([p.x, p.y, p.z], dtype=float)
            self.carrot_frame_id = msg.header.frame_id or "map"
            self.last_carrot_time = time.time()
            self.carrot_source = "/vis/lookahead_goal"
        except Exception as e:
            self.get_logger().warn(f'前瞻点解析异常: {e}', throttle_duration_sec=2.0)

    def goal_pose_cb(self, msg: PoseStamped):
        try:
            p = msg.pose.position
            q = msg.pose.orientation
            self.goal_pos = np.array([p.x, p.y, p.z], dtype=float)
            self.goal_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
            self.goal_frame = msg.header.frame_id or "map"
            self.goal_source = f"{msg.header.frame_id or 'map'}:goal"
            self.last_goal_time = time.time()
            # If no path lookahead is available (single-goal mode), use the goal as carrot.
            if self.carrot_pos is None or (time.time() - self.last_carrot_time > 3.0):
                self.carrot_pos = self.goal_pos.copy()
                self.carrot_frame_id = self.goal_frame
                self.last_carrot_time = time.time()
                self.carrot_source = self.goal_topic
            try:
                with open(os.path.join(self.session_dir, 'goals.jsonl'), 'a', encoding='utf-8') as f:
                    f.write(json.dumps({
                        'time_iso': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                        'frame_id': self.goal_frame,
                        'x': float(p.x), 'y': float(p.y), 'z': float(p.z),
                        'yaw_deg': math.degrees(self.goal_yaw),
                        'source_topic': self.goal_topic,
                    }, ensure_ascii=False) + '\n')
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f'目标点解析异常: {e}', throttle_duration_sec=2.0)

    def cmd_cb(self, msg: Twist):
        self.last_cmd = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        self.last_cmd_time = time.time()

    def raw_cmd_cb(self, msg: Twist):
        self.raw_cmd = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        self.last_raw_cmd_time = time.time()

    def stair_state_cb(self, msg: Bool):
        self.is_descending_stairs = bool(msg.data)
        self.last_stair_time = time.time()

    def stair_up_state_cb(self, msg: Bool):
        self.is_ascending_stairs = bool(msg.data)
        self.last_stair_up_time = time.time()

    def motion_enabled_cb(self, msg: Bool):
        self.motion_enabled = bool(msg.data)
        self.last_motion_enabled_time = time.time()

    def start_navi_cb(self, msg: Bool):
        self.start_navi = bool(msg.data)
        self.last_start_navi_time = time.time()

    def status_cb(self, msg: String):
        try:
            raw = msg.data or ''
            state = ''
            msgs = ''
            try:
                data = json.loads(raw)
                state = str(data.get('state', ''))
                msgs_val = data.get('msgs', [])
                if isinstance(msgs_val, (list, tuple)):
                    msgs = ';'.join(str(x) for x in msgs_val)
                else:
                    msgs = str(msgs_val)
            except Exception:
                state = raw.strip()
            self.nav_state = state
            self.nav_state_msgs = msgs
            self.nav_state_counts[state] = self.nav_state_counts.get(state, 0) + 1
            self.last_status_time = time.time()
        except Exception:
            pass

    def adaptive_debug_cb(self, msg: String):
        try:
            self.adaptive_debug = json.loads(msg.data)
            self.last_adaptive_debug_time = time.time()
        except Exception:
            pass

    def stairs_debug_cb(self, msg: String):
        try:
            self.stairs_debug = json.loads(msg.data)
            self.last_stairs_debug_time = time.time()
        except Exception:
            pass

    def camera_info_cb(self, msg: CameraInfo, topic: str = ''):
        try:
            if topic:
                self.camera_info_topic = topic
            k = list(msg.k)
            self.camera_info = {
                'width': int(msg.width),
                'height': int(msg.height),
                'fx': float(k[0]) if len(k) > 0 else None,
                'fy': float(k[4]) if len(k) > 4 else None,
                'cx': float(k[2]) if len(k) > 2 else None,
                'cy': float(k[5]) if len(k) > 5 else None,
                'distortion_model': msg.distortion_model,
                'd': list(msg.d),
                'frame_id': msg.header.frame_id,
            }
            self._write_camera_info()
        except Exception:
            pass

    def rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_rgb_time = time.time()
            self.rgb_msg_count += 1
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Path / LiDAR
    # ------------------------------------------------------------------
    def path_cb(self, msg: Path):
        try:
            if len(msg.poses) == 0:
                return
            pts = np.array(
                [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses],
                dtype=np.float32,
            )
            self.global_path_pts = pts
            self.global_path_frame = msg.header.frame_id or "map"
            self.last_path_time = time.time()
            self.path_length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
            digest = hashlib.md5(np.round(pts, 2).tobytes()).hexdigest()[:12]
            if digest != self.path_hash:
                self.path_hash = digest
                self.path_seq += 1
                path_file = os.path.join(self.path_dir, f"path_{self.path_seq:04d}_{digest}.json")
                with open(path_file, 'w', encoding='utf-8') as f:
                    json.dump({
                        'time_iso': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                        'frame_id': self.global_path_frame,
                        'point_count': int(len(pts)),
                        'length_m': self.path_length,
                        'points': pts.tolist(),
                    }, f, ensure_ascii=False)
        except Exception:
            pass

    def _transform_points_to_base(self, points, frame_id):
        """Return points in base_link if TF is available, else original points."""
        if not points.size:
            return points, False
        if self.tf_buffer is None or not frame_id:
            return points, False
        src = (frame_id or '').lstrip('/')
        dst = 'base_link'
        if src == dst:
            return points, True
        try:
            trans = self.tf_buffer.lookup_transform(
                dst, src, Time(), timeout=Duration(seconds=0.05)
            )
            t = trans.transform.translation
            q = trans.transform.rotation
            R = _quat_to_matrix(q.x, q.y, q.z, q.w)
            out = points @ R.T + np.array([t.x, t.y, t.z], dtype=np.float64)
            return out.astype(np.float32), True
        except Exception:
            return points, False

    def _update_lidar_stats(self, points, frame_id):
        try:
            pts, tf_ok = self._transform_points_to_base(points, frame_id)
            if pts is None or pts.size == 0:
                return
            # front walking corridor: 0.2-3.0 m forward, |y| <= 0.6 m, z in [-0.7, 1.5]
            front_mask = (
                (pts[:, 0] >= 0.2) & (pts[:, 0] <= 3.0) &
                (np.abs(pts[:, 1]) <= 0.6) &
                (pts[:, 2] >= -0.7) & (pts[:, 2] <= 1.5)
            )
            any_mask = (
                (pts[:, 0] >= 0.0) & (pts[:, 0] <= 3.0) &
                (np.abs(pts[:, 1]) <= 1.5) &
                (pts[:, 2] >= -1.0) & (pts[:, 2] <= 2.0)
            )
            front = pts[front_mask]
            any_pts = pts[any_mask]
            stats = {
                'min_dist_front': None,
                'min_dist_any': None,
                'nearest_x': None,
                'nearest_y': None,
                'nearest_z': None,
                'valid': False,
                'frame': frame_id,
                'tf_ok': bool(tf_ok),
                'points': int(len(pts)),
            }
            if len(front) > 0:
                d = np.hypot(front[:, 0], front[:, 1])
                i = int(np.argmin(d))
                stats.update({
                    'min_dist_front': float(d[i]),
                    'nearest_x': float(front[i, 0]),
                    'nearest_y': float(front[i, 1]),
                    'nearest_z': float(front[i, 2]),
                    'valid': True,
                })
            if len(any_pts) > 0:
                d = np.hypot(any_pts[:, 0], any_pts[:, 1])
                stats['min_dist_any'] = float(np.min(d))
            self.lidar_stats = stats
            self.lidar_msg_count += 1
            self.last_lidar_time = time.time()
        except Exception:
            pass

    def lidar_cb(self, msg: PointCloud2):
        if pc2 is None:
            return
        try:
            raw = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if not raw:
                return
            arr = np.asarray(raw, dtype=np.float32)
            if arr.ndim == 1:
                arr = arr.reshape(-1, 3)
            if arr.shape[1] > 3:
                arr = arr[:, :3]
            self._update_lidar_stats(arr, msg.header.frame_id)
        except Exception:
            pass

    def livox_cb(self, msg: CustomMsg):
        try:
            pts = np.array(
                [[p.x, p.y, p.z] for p in msg.points], dtype=np.float32
            )
            self._update_lidar_stats(pts, msg.header.frame_id)
        except Exception:
            pass

    def imu_cb(self, msg: Imu):
        try:
            a = msg.linear_acceleration
            g = msg.angular_velocity
            self.imu_stats = {
                'ax': float(a.x), 'ay': float(a.y), 'az': float(a.z),
                'gx': float(g.x), 'gy': float(g.y), 'gz': float(g.z),
            }
            self.imu_msg_count += 1
            self.last_imu_time = time.time()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Depth recording
    # ------------------------------------------------------------------
    @staticmethod
    def _compute_depth_stats(depth_m, min_depth, max_depth):
        total = int(depth_m.size)
        if total == 0:
            return {}
        finite = np.isfinite(depth_m)
        positive = finite & (depth_m > 1e-6)
        valid = positive & (depth_m <= max_depth) & (depth_m >= min_depth)
        nan_mask = ~finite
        zero_mask = depth_m == 0.0
        near_mask = positive & (depth_m < min_depth)
        far_mask = positive & (depth_m > max_depth)
        vals = depth_m[valid]
        if vals.size > 0:
            p10, p50, p90 = np.percentile(vals, [10, 50, 90])
            stats = {
                'min': float(np.min(vals)),
                'p10': float(p10),
                'median': float(p50),
                'mean': float(np.mean(vals)),
                'p90': float(p90),
                'max': float(np.max(vals)),
            }
        else:
            stats = {k: None for k in ('min', 'p10', 'median', 'mean', 'p90', 'max')}
        stats.update({
            'valid_ratio': float(np.count_nonzero(valid)) / total,
            'nan_ratio': float(np.count_nonzero(nan_mask)) / total,
            'zero_ratio': float(np.count_nonzero(zero_mask)) / total,
            'near_ratio': float(np.count_nonzero(near_mask)) / total,
            'far_ratio': float(np.count_nonzero(far_mask)) / total,
            'hole_ratio': float(np.count_nonzero(nan_mask | zero_mask)) / total,
        })
        return stats

    def depth_cb(self, msg: Image, topic_name: str = ''):
        self.depth_msg_count += 1
        if topic_name:
            self.active_depth_topic = topic_name
        if not self.camera_override:
            detected = _camera_id_from_topic(self.active_depth_topic)
            if detected:
                self.camera_id = detected
                if detected in CAMERA_DEFAULTS:
                    if not self._min_depth_explicit:
                        self.min_depth = CAMERA_DEFAULTS[detected]['min_depth']
                    if not self._max_depth_explicit:
                        self.max_depth = CAMERA_DEFAULTS[detected]['max_depth']
                    if self.camera_frame in ('', 'auto'):
                        self.camera_frame = CAMERA_DEFAULTS[detected]['camera_frame']
                    if not self.camera_info_topic:
                        self.camera_info_topic = CAMERA_DEFAULTS[detected]['info_topic']

        now = time.time()
        # Frame interval / latency (updated at camera rate, not record rate).
        if self.last_depth_recv > 0.0:
            self.depth_period_ms = (now - self.last_depth_recv) * 1000.0
            if self.depth_period_ms > 1e-3:
                inst_fps = 1000.0 / self.depth_period_ms
                self.depth_fps_est = (
                    inst_fps if self.depth_fps_est is None
                    else 0.9 * self.depth_fps_est + 0.1 * inst_fps
                )
        self.last_depth_recv = now

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp > 0.0:
            age = now - stamp
            if 0.0 <= age <= 5.0:
                self.depth_age_ms = age * 1000.0
                self.depth_latency_ms = self.depth_age_ms
            else:
                self.depth_age_ms = None
                self.depth_latency_ms = None
        else:
            self.depth_age_ms = None
            self.depth_latency_ms = None

        # Throttle.
        if (now - self.last_record_epoch) < (1.0 / self.record_rate):
            return
        self.last_record_epoch = now

        # Decode depth: RealSense 16UC1 (mm), ZED 32FC1 (m).
        try:
            depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_encoding = getattr(msg, 'encoding', '') or ''
            if self.depth_encoding in ('16UC1', 'mono16') or depth_array.dtype == np.uint16:
                depth_m = depth_array.astype(np.float32) / 1000.0
            else:
                depth_m = depth_array.astype(np.float32)
            self.depth_height, self.depth_width = depth_m.shape[:2]
            self.latest_depth_msg_shape = (self.depth_height, self.depth_width)
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 深度图转换失败: {e}', throttle_duration_sec=2.0)
            return

        raw_stats = self._compute_depth_stats(depth_m, self.min_depth, self.max_depth)
        self.latest_raw_depth_stats = raw_stats

        # Exact policy preprocessing.
        try:
            is_nan = np.isnan(depth_m)
            is_posinf = np.isposinf(depth_m)
            is_neginf = np.isneginf(depth_m)
            far_mask = is_posinf | (depth_m > self.max_depth)
            close_mask = is_neginf | ((depth_m < self.min_depth) & ~is_nan & ~far_mask)
            nan_mask = is_nan

            depth_clamped = depth_m.copy()
            depth_clamped[far_mask] = 0.0
            depth_clamped[close_mask] = self.min_depth
            depth_clamped[nan_mask] = self.min_depth

            depth_40x64 = cv2.resize(depth_clamped, (64, 40), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().error(f'深度图预处理异常: {e}')
            return

        valid_40 = depth_40x64 > 0.0
        depth_40_stats = {
            'valid_ratio': float(np.count_nonzero(valid_40)) / float(depth_40x64.size),
            'zero_ratio': float(np.count_nonzero(depth_40x64 == 0.0)) / float(depth_40x64.size),
        }
        self.latest_depth_40x64_stats = depth_40_stats

        # Robot pose: prefer map-frame localization.
        has_loc_odom = (self.robot_pos_map is not None) and (now - self.last_loc_odom_time <= 3.0)
        has_lio_odom = (self.robot_pos is not None) and (now - self.last_odom_time <= 3.0)
        use_map_frame = has_loc_odom
        if has_loc_odom:
            curr_robot_pos = self.robot_pos_map
            curr_robot_yaw = self.robot_yaw_map
            curr_robot_yaw_deg = self.robot_yaw_deg_map
            curr_robot_roll = self.robot_roll_map
            curr_robot_pitch = self.robot_pitch_map
        elif has_lio_odom:
            curr_robot_pos = self.robot_pos
            curr_robot_yaw = self.robot_yaw
            curr_robot_yaw_deg = self.robot_yaw_deg
            curr_robot_roll = self.robot_roll
            curr_robot_pitch = self.robot_pitch
        else:
            curr_robot_pos = np.zeros(3, dtype=float)
            curr_robot_yaw = 0.0
            curr_robot_yaw_deg = 0.0
            curr_robot_roll = 0.0
            curr_robot_pitch = 0.0

        # Target vector in body frame.
        is_carrot_valid = False
        target_dist_2d = None
        target_dist_3d = None
        target_yaw_deg = None
        target_pitch_deg = None
        target_quadrant = "NONE"
        dx_b = dy_b = dz_b = 0.0
        if self.carrot_pos is not None and (now - self.last_carrot_time <= 3.0):
            try:
                delta_w = self.carrot_pos - curr_robot_pos
                dx_b = math.cos(-curr_robot_yaw) * delta_w[0] - math.sin(-curr_robot_yaw) * delta_w[1]
                dy_b = math.sin(-curr_robot_yaw) * delta_w[0] + math.cos(-curr_robot_yaw) * delta_w[1]
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

        # ROI stats (policy input).
        try:
            left_roi = depth_40x64[10:35, 0:18]
            valid_left = left_roi[left_roi > self.min_depth]
            depth_left_min = float(np.min(valid_left)) if len(valid_left) > 0 else float(np.min(left_roi))
            depth_left_avg = float(np.mean(left_roi))

            center_roi = depth_40x64[10:35, 20:44]
            valid_center = center_roi[center_roi > self.min_depth]
            depth_center_min = float(np.min(valid_center)) if len(valid_center) > 0 else float(np.min(center_roi))
            depth_center_avg = float(np.mean(center_roi))

            right_roi = depth_40x64[10:35, 46:64]
            valid_right = right_roi[right_roi > self.min_depth]
            depth_right_min = float(np.min(valid_right)) if len(valid_right) > 0 else float(np.min(right_roi))
            depth_right_avg = float(np.mean(right_roi))

            floor_roi = depth_40x64[32:40, 20:44]
            valid_floor = floor_roi[floor_roi > self.min_depth]
            depth_floor_near = float(np.mean(valid_floor)) if len(valid_floor) > 0 else 0.0

            void_window = depth_40x64[15:38, 20:44]
            center_void_ratio = float(np.mean((void_window == 0.0) | (void_window > 2.5)))

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

        # Path diagnostics.
        path_points_count = 0
        path_dist_to_end = None
        path_slope_ahead = None
        if self.global_path_pts is not None and (now - self.last_path_time <= 5.0):
            try:
                path_points_count = len(self.global_path_pts)
                end_pt = self.global_path_pts[-1]
                path_dist_to_end = float(np.linalg.norm(end_pt - curr_robot_pos))
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

        # Goal diagnostics.
        goal_dist = None
        goal_age_ms = None
        if self.goal_pos is not None:
            goal_dist = float(np.linalg.norm(self.goal_pos[:2] - curr_robot_pos[:2]))
            goal_age_ms = max(0.0, (now - self.last_goal_time) * 1000.0) if self.last_goal_time > 0 else None

        # Stair flags (last message fresh).
        stair_down_active = self.is_descending_stairs and (now - self.last_stair_time <= 1.5)
        stair_up_active = self.is_ascending_stairs and (now - self.last_stair_up_time <= 1.5)

        # Adaptive / stairs debug (optional topics).
        adaptive = self.adaptive_debug if (now - self.last_adaptive_debug_time <= 2.0) else {}
        stairs_dbg = self.stairs_debug if (now - self.last_stairs_debug_time <= 2.0) else {}

        # RGB availability.
        rgb_available = (
            self.record_rgb and self.latest_rgb is not None and (now - self.last_rgb_time <= 0.5)
        )
        imu_valid = self.enable_imu and (now - self.last_imu_time <= 0.5)

        # Aggregates.
        dt = 0.0 if self.last_record_time is None else max(0.0, now - self.last_record_time)
        self.last_record_time = now
        speed = math.hypot(self.odom_vx, self.odom_vy)
        self.agg['frames'] += 1
        self.agg['valid_ratio_sum'] += raw_stats.get('valid_ratio', 0.0) or 0.0
        self.agg['hole_ratio_sum'] += raw_stats.get('hole_ratio', 0.0) or 0.0
        self.agg['nan_ratio_sum'] += raw_stats.get('nan_ratio', 0.0) or 0.0
        self.agg['zero_ratio_sum'] += raw_stats.get('zero_ratio', 0.0) or 0.0
        if self.depth_latency_ms is not None:
            self.agg['latency_sum'] += self.depth_latency_ms
            self.agg['latency_count'] += 1
        self.agg['center_min_sum'] += depth_center_min
        self.agg['center_avg_sum'] += depth_center_avg
        self.agg['speed_sum'] += speed
        self.agg['cmd_vx_sum'] += self.last_cmd[0]
        self.agg['max_speed'] = max(self.agg['max_speed'], speed)
        self.agg['max_cmd_vx'] = max(self.agg['max_cmd_vx'], self.last_cmd[0])
        self.agg['distance'] += speed * dt
        if self.lidar_stats.get('min_dist_front') is not None:
            self.agg['lidar_min_sum'] += self.lidar_stats['min_dist_front']
            self.agg['lidar_count'] += 1
        if stair_down_active:
            self.agg['stair_down_frames'] += 1
        if stair_up_active:
            self.agg['stair_up_frames'] += 1

        # Heatmap.
        heatmap_large = self._build_heatmap(
            depth_40x64, raw_stats, depth_left_min, depth_center_min, depth_right_min,
            center_void_ratio, is_carrot_valid, target_dist_2d, target_yaw_deg, dy_b, dz_b,
            target_quadrant, stair_down_active, stair_up_active, now
        )

        # Raw depth snapshot for offline preprocessing experiments.
        raw_depth_u16 = None
        if self.save_raw_depth and ((self.frame_idx + 1) % self.raw_depth_every == 0):
            raw_depth_u16 = self._prepare_raw_depth_u16(depth_m)

        rgb_bgr = self.latest_rgb.copy() if rgb_available else None

        self.frame_idx += 1
        frame_name = f"frame_{self.frame_idx:06d}"
        timestamp_iso = datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

        meta_dict = {
            "schema_version": SCHEMA_VERSION,
            "frame_idx": self.frame_idx,
            "timestamp_iso": timestamp_iso,
            "epoch_sec": now,
            "session_name": self.session_name,
            "run_id": self.run_id,
            "camera_id": self.camera_id,
            "depth_topic": self.active_depth_topic,
            "depth_encoding": self.depth_encoding,
            "depth_size": [self.depth_width, self.depth_height],
            "min_depth_param": self.min_depth,
            "max_depth_param": self.max_depth,
            "depth_timing": {
                "fps_est": self.depth_fps_est,
                "period_ms": self.depth_period_ms,
                "latency_ms": self.depth_latency_ms,
                "age_ms": self.depth_age_ms,
            },
            "depth_health_raw": raw_stats,
            "depth_health_40x64": depth_40_stats,
            "robot_pos": curr_robot_pos.tolist(),
            "robot_frame": "map" if use_map_frame else "odom",
            "robot_pos_odom": self.robot_pos.tolist() if self.robot_pos is not None else None,
            "robot_pos_map": self.robot_pos_map.tolist() if self.robot_pos_map is not None else None,
            "robot_attitude": {
                "roll_deg": curr_robot_roll,
                "pitch_deg": curr_robot_pitch,
                "yaw_deg": curr_robot_yaw_deg,
            },
            "odom_velocity": {
                "vx": self.odom_vx, "vy": self.odom_vy,
                "vz": self.odom_vz, "wz": self.odom_wz,
            },
            "speed_mps": speed,
            "cmd_vel": self.last_cmd,
            "raw_cmd_vel": self.raw_cmd,
            "carrot_pos": self.carrot_pos.tolist() if self.carrot_pos is not None else None,
            "carrot_source": self.carrot_source,
            "is_carrot_valid": is_carrot_valid,
            "target_3d": {
                "dx_b": dx_b, "dy_b": dy_b, "dz_b": dz_b,
                "dist_2d": target_dist_2d, "dist_3d": target_dist_3d,
                "yaw_deg": target_yaw_deg, "pitch_deg": target_pitch_deg,
                "quadrant": target_quadrant,
            },
            "goal": {
                "pos": self.goal_pos.tolist() if self.goal_pos is not None else None,
                "yaw_deg": math.degrees(self.goal_yaw) if self.goal_yaw is not None else None,
                "frame": self.goal_frame,
                "source": self.goal_source,
                "dist": goal_dist,
                "age_ms": goal_age_ms,
            },
            "nav": {
                "state": self.nav_state,
                "msgs": self.nav_state_msgs,
                "motion_enabled": self.motion_enabled,
                "start_navi": self.start_navi,
            },
            "clearance_partitions": {
                "left_min": depth_left_min, "left_avg": depth_left_avg,
                "center_min": depth_center_min, "center_avg": depth_center_avg,
                "right_min": depth_right_min, "right_avg": depth_right_avg,
                "floor_near": depth_floor_near, "center_void_ratio": center_void_ratio,
                "door_center_min": door_center_min_depth,
                "door_center_avg": door_center_avg_depth,
            },
            "stairs": {
                "is_descending_stairs": stair_down_active,
                "is_ascending_stairs": stair_up_active,
                "debug": stairs_dbg,
            },
            "adaptive": adaptive,
            "path": {
                "points_count": path_points_count,
                "dist_to_end": path_dist_to_end,
                "slope_ahead": path_slope_ahead,
                "length": self.path_length,
                "hash": self.path_hash,
                "frame": self.global_path_frame,
            },
            "lidar": self.lidar_stats,
            "imu": {**self.imu_stats, "valid": bool(imu_valid), "msg_count": self.imu_msg_count},
            "rgb_available": bool(rgb_available),
        }

        csv_row = [
            self.frame_idx, timestamp_iso, f"{now:.3f}",
            _fmt(curr_robot_pos[0]), _fmt(curr_robot_pos[1]), _fmt(curr_robot_pos[2]),
            _fmt(curr_robot_yaw_deg, "{:.1f}"),
            _fmt(self.carrot_pos[0]) if self.carrot_pos is not None else "",
            _fmt(self.carrot_pos[1]) if self.carrot_pos is not None else "",
            _fmt(self.carrot_pos[2]) if self.carrot_pos is not None else "",
            _fmt(target_dist_2d), _fmt(target_yaw_deg, "{:.1f}"),
            _fmt(self.last_cmd[0], "{:.3f}"), _fmt(self.last_cmd[1], "{:.3f}"), _fmt(self.last_cmd[2], "{:.3f}"),
            _fmt(door_center_min_depth), _fmt(door_center_avg_depth),
            "1" if is_carrot_valid else "0", self.carrot_source,

            _fmt(curr_robot_roll, "{:.1f}"), _fmt(curr_robot_pitch, "{:.1f}"),
            _fmt(self.odom_vx), _fmt(self.odom_vy), _fmt(self.odom_vz), _fmt(self.odom_wz),
            _fmt(speed),

            _fmt(dx_b), _fmt(dy_b), _fmt(dz_b),
            _fmt(target_dist_3d), _fmt(target_pitch_deg, "{:.1f}"), target_quadrant,

            _fmt(depth_left_min), _fmt(depth_left_avg),
            _fmt(depth_center_min), _fmt(depth_center_avg),
            _fmt(depth_right_min), _fmt(depth_right_avg),
            _fmt(depth_floor_near), _fmt(center_void_ratio),

            "1" if stair_down_active else "0", "1" if stair_up_active else "0",
            _fmt_int(path_points_count), _fmt(path_dist_to_end), _fmt(path_slope_ahead),
            _fmt(self.path_length), self.path_hash,

            self.camera_id, self.active_depth_topic or "", self.depth_encoding,
            _fmt_int(self.depth_width), _fmt_int(self.depth_height),
            _fmt(self.depth_fps_est, "{:.2f}"), _fmt(self.depth_period_ms, "{:.1f}"),
            _fmt(self.depth_latency_ms, "{:.1f}"), _fmt(self.depth_age_ms, "{:.1f}"),
            _fmt(raw_stats.get('valid_ratio'), "{:.4f}"),
            _fmt(raw_stats.get('nan_ratio'), "{:.4f}"),
            _fmt(raw_stats.get('zero_ratio'), "{:.4f}"),
            _fmt(raw_stats.get('near_ratio'), "{:.4f}"),
            _fmt(raw_stats.get('far_ratio'), "{:.4f}"),
            _fmt(raw_stats.get('hole_ratio'), "{:.4f}"),
            _fmt(raw_stats.get('min')), _fmt(raw_stats.get('p10')),
            _fmt(raw_stats.get('median')), _fmt(raw_stats.get('mean')),
            _fmt(raw_stats.get('p90')), _fmt(raw_stats.get('max')),
            _fmt(depth_40_stats.get('valid_ratio'), "{:.4f}"),
            _fmt(depth_40_stats.get('zero_ratio'), "{:.4f}"),
            _fmt(self.min_depth, "{:.2f}"), _fmt(self.max_depth, "{:.2f}"),

            self.nav_state, self.nav_state_msgs,
            _bool_str(self.motion_enabled), _bool_str(self.start_navi),
            _fmt(self.raw_cmd[0], "{:.3f}"), _fmt(self.raw_cmd[1], "{:.3f}"), _fmt(self.raw_cmd[2], "{:.3f}"),

            _fmt(self.goal_pos[0]) if self.goal_pos is not None else "",
            _fmt(self.goal_pos[1]) if self.goal_pos is not None else "",
            _fmt(self.goal_pos[2]) if self.goal_pos is not None else "",
            _fmt(math.degrees(self.goal_yaw), "{:.1f}") if self.goal_yaw is not None else "",
            _fmt(goal_dist), _fmt(goal_age_ms, "{:.0f}"), self.goal_source,

            _fmt(self.lidar_stats.get('min_dist_front')),
            _fmt(self.lidar_stats.get('min_dist_any')),
            _fmt(self.lidar_stats.get('nearest_x')),
            _fmt(self.lidar_stats.get('nearest_y')),
            _fmt(self.lidar_stats.get('nearest_z')),
            "1" if self.lidar_stats.get('valid') else "0",
            self.lidar_stats.get('frame', ''),
            "1" if self.lidar_stats.get('tf_ok') else "0",
            _fmt_int(self.lidar_stats.get('points')),

            _fmt(self.imu_stats.get('ax')), _fmt(self.imu_stats.get('ay')), _fmt(self.imu_stats.get('az')),
            _fmt(self.imu_stats.get('gx')), _fmt(self.imu_stats.get('gy')), _fmt(self.imu_stats.get('gz')),
            "1" if imu_valid else "0",

            _fmt(adaptive.get('lookahead')), _fmt(adaptive.get('l_curv')),
            _fmt(adaptive.get('l_clear')), _fmt(adaptive.get('l_target')),
            _fmt(adaptive.get('turn_deg'), "{:.1f}"),
            _bool_str(adaptive.get('in_narrow')),
            _fmt(adaptive.get('clearance_w')),
            _fmt(stairs_dbg.get('slope')), _fmt(stairs_dbg.get('dz')),
            _bool_str(stairs_dbg.get('remap_active')),

            "1" if rgb_available else "0",
        ]

        if len(csv_row) != len(self.csv_headers):
            self.get_logger().error(
                f'CSV 列数不匹配: row={len(csv_row)}, header={len(self.csv_headers)}；本帧跳过 CSV 写入'
            )
            csv_row = None

        task = (frame_name, depth_40x64, heatmap_large, raw_depth_u16, rgb_bgr, meta_dict, csv_row)
        try:
            self.write_queue.put_nowait(task)
        except queue.Full:
            self.dropped_queue_frames += 1
            self.get_logger().warn('写盘队列已满，丢弃该帧！', throttle_duration_sec=3.0)

        # Terminal progress.
        if self.verbose or (now - self.last_terminal_print >= 1.0):
            self.last_terminal_print = now
            tgt_display = (
                f"{target_dist_2d:.2f}m@{target_yaw_deg:+.0f}°" if is_carrot_valid else "等待目标..."
            )
            stair_badge = (
                "\033[96m[STAIR-DOWN]\033[0m" if stair_down_active
                else ("\033[96m[STAIR-UP]\033[0m" if stair_up_active else "[FLAT]")
            )
            behind_warn = (
                " \033[91m[BEHIND!]\033[0m"
                if (is_carrot_valid and abs(target_yaw_deg) > 90.0) else ""
            )
            lidar_str = (
                f"{self.lidar_stats['min_dist_front']:.2f}m"
                if self.lidar_stats.get('min_dist_front') is not None else "--"
            )
            sys.stdout.write(
                f"\r[REC #{self.frame_idx:04d}] {timestamp_iso[-12:]} | "
                f"{self.camera_id:<9} | tgt:{tgt_display:<13}{behind_warn} | {stair_badge} | "
                f"L/C/R:{depth_left_min:.1f}/{depth_center_min:.1f}/{depth_right_min:.1f}m | "
                f"lidar:{lidar_str} | valid:{(raw_stats.get('valid_ratio') or 0)*100:.0f}% "
                f"hole:{(raw_stats.get('hole_ratio') or 0)*100:.0f}% | "
                f"cmd:[{self.last_cmd[0]:.2f},{self.last_cmd[2]:+.2f}] | "
                f"q:{self.write_queue.qsize():<2}"
            )
            sys.stdout.flush()

    def _build_heatmap(
        self, depth_40x64, raw_stats, left_min, center_min, right_min,
        void_ratio, is_carrot_valid, target_dist, target_yaw, dy_b, dz_b,
        quadrant, stair_down, stair_up, now,
    ):
        try:
            norm_img = np.clip(np.nan_to_num(depth_40x64, nan=0.0) / 5.0, 0.0, 1.0)
            heatmap = cv2.applyColorMap((norm_img * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            heatmap_large = cv2.resize(heatmap, (320, 200), interpolation=cv2.INTER_NEAREST)

            cv2.rectangle(heatmap_large, (100, 50), (220, 175), (255, 255, 0), 1)
            cv2.line(heatmap_large, (100, 160), (220, 160), (0, 255, 255), 1)
            cv2.rectangle(heatmap_large, (130, 75), (190, 125), (200, 200, 200), 1)

            # Camera + depth health banner.
            cv2.rectangle(heatmap_large, (3, 3), (317, 24), (20, 20, 20), -1)
            valid_pct = (raw_stats.get('valid_ratio') or 0.0) * 100.0
            hole_pct = (raw_stats.get('hole_ratio') or 0.0) * 100.0
            lat = self.depth_latency_ms
            lat_txt = f"{lat:.0f}ms" if lat is not None else "--"
            cv2.putText(
                heatmap_large,
                f"{self.camera_id} v{valid_pct:.0f}% h{hole_pct:.0f}% lat{lat_txt}",
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1,
            )

            # Stair state.
            if stair_down:
                cv2.rectangle(heatmap_large, (215, 27), (317, 46), (200, 120, 0), -1)
                cv2.putText(heatmap_large, "[STAIR DOWN]", (220, 41),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1)
            elif stair_up:
                cv2.rectangle(heatmap_large, (215, 27), (317, 46), (0, 120, 200), -1)
                cv2.putText(heatmap_large, "[STAIR UP]", (220, 41),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (255, 255, 255), 1)
            else:
                cv2.putText(heatmap_large, "[FLAT]", (260, 41),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1)

            # Target.
            if is_carrot_valid:
                raw_arrow_x = 160 + (-dy_b * 40.0)
                safe_arrow_x = int(np.clip(np.nan_to_num(raw_arrow_x, nan=160.0), 20.0, 300.0))
                cv2.arrowedLine(heatmap_large, (160, 180), (safe_arrow_x, 50),
                                (255, 255, 255), 2, tipLength=0.25)
                cv2.putText(
                    heatmap_large,
                    f"Tgt {target_dist:.2f}m @ {target_yaw:+.0f}d dz{dz_b:+.2f}",
                    (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1,
                )
                if abs(target_yaw) > 90.0:
                    cv2.rectangle(heatmap_large, (4, 65), (180, 95), (0, 0, 180), -1)
                    cv2.putText(heatmap_large, f"BEHIND {target_yaw:+.0f}deg [{quadrant}]",
                                (8, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 255), 1)
            else:
                cv2.putText(heatmap_large, "Tgt: WAITING", (8, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)

            # Bottom status.
            cv2.rectangle(heatmap_large, (0, 162), (320, 200), (10, 10, 10), -1)
            clearance_text = (
                f"L:{left_min:.1f} C:{center_min:.1f} R:{right_min:.1f} "
                f"Void:{void_ratio*100:.0f}%"
            )
            cv2.putText(heatmap_large, clearance_text, (6, 177),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (255, 220, 100), 1)
            lidar_txt = (
                f"{self.lidar_stats['min_dist_front']:.2f}"
                if self.lidar_stats.get('min_dist_front') is not None else "--"
            )
            vel_text = (
                f"Cmd[{self.last_cmd[0]:.2f},{self.last_cmd[2]:+.2f}] "
                f"Odom[{self.odom_vx:.2f},{self.odom_wz:+.2f}] Lidar:{lidar_txt}"
            )
            cv2.putText(heatmap_large, vel_text, (6, 194),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (0, 255, 120), 1)
            return heatmap_large
        except Exception:
            return np.zeros((200, 320, 3), dtype=np.uint8)

    def _prepare_raw_depth_u16(self, depth_m):
        try:
            h, w = depth_m.shape[:2]
            scale = min(1.0, float(self.raw_depth_width) / float(max(1, w)))
            if scale < 1.0:
                small = cv2.resize(
                    depth_m, (max(1, int(round(w * scale))), max(1, int(round(h * scale)))),
                    interpolation=cv2.INTER_NEAREST,
                )
            else:
                small = depth_m
            mm = np.nan_to_num(small, nan=0.0, posinf=0.0, neginf=0.0)
            mm = np.clip(mm * 1000.0, 0.0, 65535.0).astype(np.uint16)
            return mm
        except Exception:
            return None

    # ------------------------------------------------------------------
    # Writer
    # ------------------------------------------------------------------
    def _writer_worker(self):
        while not self.stop_worker_event.is_set() or not self.write_queue.empty():
            try:
                task = self.write_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            frame_name, depth_40x64, heatmap_large, raw_depth_u16, rgb_bgr, meta_dict, csv_row = task
            try:
                np.save(os.path.join(self.depth_dir, f"{frame_name}_depth_40x64.npy"), depth_40x64)
                if heatmap_large is not None:
                    cv2.imwrite(os.path.join(self.heatmap_dir, f"{frame_name}_heatmap.png"), heatmap_large)
                if raw_depth_u16 is not None:
                    cv2.imwrite(os.path.join(self.depth_raw_dir, f"{frame_name}_raw_depth.png"), raw_depth_u16)
                if rgb_bgr is not None:
                    cv2.imwrite(os.path.join(self.rgb_dir, f"{frame_name}_rgb.jpg"), rgb_bgr,
                                [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                with open(os.path.join(self.meta_dir, f"{frame_name}_meta.json"), 'w', encoding='utf-8') as f:
                    json.dump(meta_dict, f, indent=2, ensure_ascii=False)
                if csv_row is not None:
                    self.csv_writer.writerow(csv_row)
                    self.csv_file.flush()
            except Exception as e:
                try:
                    self.get_logger().error(f'写盘失败 {frame_name}: {e}')
                except Exception:
                    pass
            finally:
                self.write_queue.task_done()

    # ------------------------------------------------------------------
    # Metadata / snapshots
    # ------------------------------------------------------------------
    def _write_args_snapshot(self):
        try:
            with open(os.path.join(self.session_dir, 'session_args.json'), 'w', encoding='utf-8') as f:
                json.dump({
                    'argv': sys.argv,
                    'camera': self.camera_id,
                    'depth_topic': self.depth_topic,
                    'min_depth': self.min_depth,
                    'max_depth': self.max_depth,
                    'record_rate': self.record_rate,
                    'scene': self.scene,
                    'note': self.note,
                    'operator': self.operator,
                    'run_id': self.run_id,
                }, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def _write_camera_info(self):
        if not self.camera_info:
            return
        try:
            with open(os.path.join(self.session_dir, 'camera_info.json'), 'w', encoding='utf-8') as f:
                json.dump(self.camera_info, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def _write_session_meta(self):
        try:
            meta = {
                'schema_version': SCHEMA_VERSION,
                'session_name': self.session_name,
                'run_id': self.run_id,
                'start_time': self.start_iso,
                'hostname': socket.gethostname(),
                'user': self.operator,
                'ros_distro': os.environ.get('ROS_DISTRO', ''),
                'camera_id': self.camera_id,
                'depth_topic': self.active_depth_topic or self.depth_topic,
                'camera_info_topic': self.camera_info_topic,
                'camera_frame': self.camera_frame,
                'depth_encoding': self.depth_encoding,
                'min_depth': self.min_depth,
                'max_depth': self.max_depth,
                'controller_min_depth': self.controller_min_depth,
                'controller_max_depth': self.controller_max_depth,
                'record_rate': self.record_rate,
                'scene': self.scene,
                'note': self.note,
                'operator': self.operator,
                'camera_info': self.camera_info,
                'tf': self.tf_snapshot,
                'controller_params_node': self.controller_params_node,
                'topics': {
                    'path': self.path_topic,
                    'status': self.status_topic,
                    'goal': self.goal_topic,
                    'cmd': self.cmd_topic,
                    'raw_cmd': self.raw_cmd_topic,
                    'lidar': self.lidar_topic,
                    'livox': self.livox_topic,
                    'imu': self.imu_topic,
                    'rgb': self.rgb_topic,
                },
            }
            with open(os.path.join(self.session_dir, 'session_meta.json'), 'w', encoding='utf-8') as f:
                json.dump(meta, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def _snapshot_controller_params(self):
        nodes = (
            [self.param_node] if self.param_node not in ('auto', '', None)
            else ['/rl_nav_controller', '/rl_nav_controller_adaptive', '/rl_nav_controller_stairs']
        )
        ros2_bin = shutil.which('ros2')
        if not ros2_bin:
            return None
        for node in nodes:
            try:
                out = subprocess.run(
                    [ros2_bin, 'param', 'dump', node],
                    capture_output=True, text=True, timeout=15,
                )
                if out.returncode == 0 and out.stdout.strip():
                    safe = node.strip('/').replace('/', '_')
                    path = os.path.join(self.session_dir, f'controller_params_{safe}.yaml')
                    with open(path, 'w', encoding='utf-8') as f:
                        f.write(out.stdout)
                    self.controller_params_node = node
                    self.controller_params_text = out.stdout
                    # Record the controller's own depth range for traceability.
                    parsed_min = None
                    parsed_max = None
                    m_min = re.search(r'^\s*min_depth:\s*([0-9.]+)', out.stdout, re.MULTILINE)
                    m_max = re.search(r'^\s*max_depth:\s*([0-9.]+)', out.stdout, re.MULTILINE)
                    if m_min:
                        parsed_min = float(m_min.group(1))
                        self.controller_min_depth = parsed_min
                    if m_max:
                        parsed_max = float(m_max.group(1))
                        self.controller_max_depth = parsed_max
                    # Only adopt the controller range before the first recorded frame;
                    # changing it mid-session would make the 40x64 inputs inconsistent.
                    if self.frame_idx == 0:
                        if parsed_min is not None and not self._min_depth_explicit:
                            self.min_depth = parsed_min
                        if parsed_max is not None and not self._max_depth_explicit:
                            self.max_depth = parsed_max
                    else:
                        if (parsed_min is not None and abs(parsed_min - self.min_depth) > 1e-6) or \
                           (parsed_max is not None and abs(parsed_max - self.max_depth) > 1e-6):
                            self.get_logger().warn(
                                f'控制器 depth 量程 ({parsed_min}, {parsed_max}) 与记录器 '
                                f'({self.min_depth}, {self.max_depth}) 不一致；记录已开始，本会话保持记录器值。'
                                f' 下次请显式传入 --min-depth/--max-depth。',
                                throttle_duration_sec=10.0,
                            )
                    return node
            except Exception:
                continue
        return None

    def _param_snapshot_worker(self):
        for _ in range(4):
            time.sleep(4.0)
            if self._snapshot_controller_params():
                self._write_session_meta()
                return

    def _tf_snapshot_cb(self):
        if self.tf_buffer is None or len(self.tf_snapshot) >= 2:
            return
        candidates = []
        if self.camera_frame and self.camera_frame not in ('', 'auto'):
            candidates.append(self.camera_frame)
        candidates.extend(['camera_link', 'zed_camera_link'])
        for frame in dict.fromkeys(candidates):
            frame = (frame or '').lstrip('/')
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', frame, Time(), timeout=Duration(seconds=0.05)
                )
                t = trans.transform.translation
                q = trans.transform.rotation
                self.tf_snapshot['base_link_to_camera'] = {
                    'child_frame': frame,
                    'translation': [t.x, t.y, t.z],
                    'rotation_xyzw': [q.x, q.y, q.z, q.w],
                }
                with open(os.path.join(self.session_dir, 'tf_base_to_camera.json'), 'w', encoding='utf-8') as f:
                    json.dump(self.tf_snapshot['base_link_to_camera'], f, indent=2)
                break
            except Exception:
                continue
        for frame in ('livox_frame', 'livox_lidar', 'base_link'):
            frame = (frame or '').lstrip('/')
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', frame, Time(), timeout=Duration(seconds=0.05)
                )
                t = trans.transform.translation
                q = trans.transform.rotation
                self.tf_snapshot['base_link_to_lidar'] = {
                    'child_frame': frame,
                    'translation': [t.x, t.y, t.z],
                    'rotation_xyzw': [q.x, q.y, q.z, q.w],
                }
                with open(os.path.join(self.session_dir, 'tf_base_to_lidar.json'), 'w', encoding='utf-8') as f:
                    json.dump(self.tf_snapshot['base_link_to_lidar'], f, indent=2)
                break
            except Exception:
                continue

    # ------------------------------------------------------------------
    # Status / close
    # ------------------------------------------------------------------
    def _status_heartbeat_cb(self):
        now = time.time()
        if self.frame_idx > 0:
            return
        odom_ok = (
            (self.robot_pos is not None and now - self.last_odom_time < 3.0)
            or (self.robot_pos_map is not None and now - self.last_loc_odom_time < 3.0)
        )
        depth_ok = self.depth_msg_count > 0
        carrot_ok = self.carrot_pos is not None and (now - self.last_carrot_time < 3.0)
        lidar_ok = self.lidar_stats.get('valid', False)
        motion_str = "?" if self.motion_enabled is None else ("ON" if self.motion_enabled else "OFF")
        sys.stdout.write(
            f"\r[就绪] depth={'OK' if depth_ok else '--'} "
            f"({self.active_depth_topic or self.depth_topic}) | "
            f"odom={'OK' if odom_ok else '--'} | tgt={'OK' if carrot_ok else '--'} | "
            f"lidar={'OK' if lidar_ok else '--'} | motion={motion_str}   "
        )
        sys.stdout.flush()

    def close(self):
        if getattr(self, '_closed', False):
            return
        self._closed = True
        print("\n\n正在收尾保存数据，请稍候...")
        self.stop_worker_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=8.0)
        try:
            self.csv_file.close()
        except Exception:
            pass

        duration = max(0.1, time.time() - self.start_epoch)
        avg_fps = self.frame_idx / duration
        n = max(1, self.agg['frames'])
        summary = {
            'schema_version': SCHEMA_VERSION,
            'session_name': self.session_name,
            'run_id': self.run_id,
            'session_dir': self.session_dir,
            'start_time': self.start_iso,
            'end_time': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'duration_sec': round(duration, 2),
            'camera_id': self.camera_id,
            'depth_topic': self.active_depth_topic or self.depth_topic,
            'depth_encoding': self.depth_encoding,
            'depth_size': [self.depth_width, self.depth_height],
            'min_depth_param': self.min_depth,
            'max_depth_param': self.max_depth,
            'controller_min_depth': self.controller_min_depth,
            'controller_max_depth': self.controller_max_depth,
            'camera_info': self.camera_info,
            'tf': self.tf_snapshot,
            'controller_params_node': self.controller_params_node,
            'total_frames_recorded': self.frame_idx,
            'average_fps': round(avg_fps, 2),
            'dropped_queue_frames': self.dropped_queue_frames,
            'record_rate_setting': self.record_rate,
            'total_csv_columns': len(self.csv_headers),
            'distance_traveled_m': round(self.agg['distance'], 3),
            'mean_speed_mps': round(self.agg['speed_sum'] / n, 3),
            'max_speed_mps': round(self.agg['max_speed'], 3),
            'mean_cmd_vx': round(self.agg['cmd_vx_sum'] / n, 3),
            'max_cmd_vx': round(self.agg['max_cmd_vx'], 3),
            'depth_health_mean': {
                'valid_ratio_raw': round(self.agg['valid_ratio_sum'] / n, 4),
                'hole_ratio_raw': round(self.agg['hole_ratio_sum'] / n, 4),
                'nan_ratio_raw': round(self.agg['nan_ratio_sum'] / n, 4),
                'zero_ratio_raw': round(self.agg['zero_ratio_sum'] / n, 4),
                'latency_ms': round(self.agg['latency_sum'] / max(1, self.agg['latency_count']), 1)
                if self.agg['latency_count'] > 0 else None,
                'depth_center_min': round(self.agg['center_min_sum'] / n, 3),
                'depth_center_avg': round(self.agg['center_avg_sum'] / n, 3),
            },
            'lidar_mean_front_m': round(self.agg['lidar_min_sum'] / max(1, self.agg['lidar_count']), 3)
            if self.agg['lidar_count'] > 0 else None,
            'imu_msg_count': self.imu_msg_count,
            'imu_topic': self.imu_topic,
            'nav_state_counts': self.nav_state_counts,
            'stair_down_frames': self.agg['stair_down_frames'],
            'stair_up_frames': self.agg['stair_up_frames'],
            'path': {
                'points_count': int(len(self.global_path_pts)) if self.global_path_pts is not None else 0,
                'length_m': self.path_length,
                'hash': self.path_hash,
                'frame': self.global_path_frame,
                'snapshots': self.path_seq,
            },
            'goal': {
                'pos': self.goal_pos.tolist() if self.goal_pos is not None else None,
                'yaw_deg': math.degrees(self.goal_yaw) if self.goal_yaw is not None else None,
                'frame': self.goal_frame,
                'source': self.goal_source,
            },
            'latest_link': self.latest_link,
        }

        try:
            with open(os.path.join(self.session_dir, 'session_summary.json'), 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
        except Exception:
            pass
        self._write_session_meta()

        print('\033[92m' + '=' * 78 + '\033[0m')
        print('\033[92m[✓] 相机对比诊断录制完成\033[0m')
        print(f"  会话目录: {self.session_dir}")
        print(f"  相机/话题: {self.camera_id} / {self.active_depth_topic or self.depth_topic}")
        if self.frame_idx == 0:
            print('\033[91m  ⚠ 未录制到任何帧：请检查深度话题、里程计、录制频率与权限。\033[0m')
        print(f"  录制帧数: {self.frame_idx} 帧 ({duration:.1f}s, {avg_fps:.1f} FPS)")
        print(f"  里程: {self.agg['distance']:.2f} m | 平均速度: {self.agg['speed_sum']/n:.2f} m/s")
        print(f"  深度健康: valid={(self.agg['valid_ratio_sum']/n)*100:.1f}% "
              f"hole={(self.agg['hole_ratio_sum']/n)*100:.1f}% "
              f"latency={summary['depth_health_mean']['latency_ms']}ms")
        if self.dropped_queue_frames:
            print(f"  ⚠ 队列丢帧: {self.dropped_queue_frames}")
        print("  文件: trajectory.csv / session_meta.json / session_summary.json / "
              "depth/ depth_raw/ heatmaps/ meta/ paths/")
        print('\033[92m' + '=' * 78 + '\033[0m')


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Camera comparison & navigation diagnostic recorder (schema 2.0)"
    )
    parser.add_argument('tag', nargs='?', default='', help='可选会话标签，例如 S3_door_run1')
    parser.add_argument('--rate', '-r', type=float, default=5.0, help='录制频率 Hz（默认 5.0）')
    parser.add_argument('--dir', '-d', type=str, default='~/door_diagnostics', help='存储根目录')
    parser.add_argument('--verbose', '-v', action='store_true', help='逐帧打印')
    parser.add_argument('--camera', choices=['auto', 'realsense', 'zed'], default='auto',
                        help='相机类型（默认 auto）')
    parser.add_argument('--realsense', '--d435', action='store_true', help='等价于 --camera realsense')
    parser.add_argument('--zed', action='store_true', help='等价于 --camera zed')
    parser.add_argument('--depth-topic', type=str, default='auto',
                        help='深度话题；auto 时按相机类型或自动监听')
    parser.add_argument('--camera-info-topic', type=str, default='',
                        help='CameraInfo 话题；空则自动推导')
    parser.add_argument('--camera-frame', type=str, default='auto',
                        help='相机 TF child frame；auto 时 RealSense=camera_link, ZED=zed_camera_link')
    parser.add_argument('--min-depth', type=float, default=None,
                        help='预处理近距阈值（默认 RealSense 0.15 / ZED 0.25）')
    parser.add_argument('--max-depth', type=float, default=None,
                        help='预处理远距阈值（默认 10.0；建议与控制器参数一致）')
    parser.add_argument('--scene', type=str, default='', help='场景 ID，例如 S1_corridor / S3_door')
    parser.add_argument('--note', type=str, default='', help='运行备注')
    parser.add_argument('--operator', type=str, default='', help='操作者')
    parser.add_argument('--run-id', type=str, default='', help='自定义 run ID')
    parser.add_argument('--path-topic', type=str, default=DEFAULT_PATH_TOPIC)
    parser.add_argument('--status-topic', type=str, default=DEFAULT_STATUS_TOPIC)
    parser.add_argument('--goal-topic', type=str, default=DEFAULT_GOAL_TOPIC)
    parser.add_argument('--cmd-topic', type=str, default=DEFAULT_CMD_TOPIC)
    parser.add_argument('--raw-cmd-topic', type=str, default=DEFAULT_RAW_CMD_TOPIC)
    parser.add_argument('--lidar-topic', type=str, default=DEFAULT_LIDAR_TOPIC,
                        help='PointCloud2 雷达话题；无则留空')
    parser.add_argument('--livox-topic', type=str, default=DEFAULT_LIVOX_TOPIC,
                        help='Livox CustomMsg 话题')
    parser.add_argument('--no-lidar', action='store_true', help='关闭 LiDAR 订阅')
    parser.add_argument('--imu-topic', type=str, default=DEFAULT_IMU_TOPIC,
                        help='IMU 话题（默认 /livox/imu）')
    parser.add_argument('--no-imu', action='store_true', help='关闭 IMU 订阅')
    parser.add_argument('--param-node', type=str, default='auto',
                        help='控制参数快照节点；auto 时尝试常见节点名')
    parser.add_argument('--no-raw-depth', action='store_true', help='不保存原始深度 PNG')
    parser.add_argument('--raw-depth-width', type=int, default=320,
                        help='原始深度 PNG 最大宽度（默认 320，等比缩放）')
    parser.add_argument('--raw-depth-every', type=int, default=1,
                        help='每 N 帧保存一次原始深度 PNG（默认 1；长时录制可设为 5/10）')
    parser.add_argument('--record-rgb', action='store_true', help='同时保存 RGB 图像（可选）')
    parser.add_argument('--rgb-topic', type=str, default=DEFAULT_RGB_TOPIC)

    parsed, unknown = parser.parse_known_args()
    if unknown:
        print(f"[warn] 忽略未知参数: {unknown}")

    camera = parsed.camera
    if parsed.realsense:
        camera = 'realsense'
    elif parsed.zed:
        camera = 'zed'

    rclpy.init()
    node = DoorDiagnosticLogger(
        base_dir=parsed.dir,
        session_tag=parsed.tag,
        record_rate=parsed.rate,
        verbose=parsed.verbose,
        path_topic=parsed.path_topic,
        depth_topic=parsed.depth_topic,
        camera=camera,
        camera_frame=parsed.camera_frame,
        camera_info_topic=parsed.camera_info_topic,
        min_depth=parsed.min_depth,
        max_depth=parsed.max_depth,
        scene=parsed.scene,
        note=parsed.note,
        operator=parsed.operator,
        run_id=parsed.run_id,
        status_topic=parsed.status_topic,
        goal_topic=parsed.goal_topic,
        cmd_topic=parsed.cmd_topic,
        raw_cmd_topic=parsed.raw_cmd_topic,
        lidar_topic=parsed.lidar_topic,
        livox_topic=parsed.livox_topic,
        enable_lidar=not parsed.no_lidar,
        imu_topic=parsed.imu_topic,
        enable_imu=not parsed.no_imu,
        param_node=parsed.param_node,
        save_raw_depth=not parsed.no_raw_depth,
        raw_depth_width=parsed.raw_depth_width,
        raw_depth_every=parsed.raw_depth_every,
        record_rgb=parsed.record_rgb,
        rgb_topic=parsed.rgb_topic,
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

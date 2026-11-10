"""RL Navigation Controller - Main module for autonomous robot navigation using ONNX Runtime."""

import os
import time
import json
import argparse

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Header, Bool, String
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

import cv2

from ament_index_python.packages import get_package_share_directory

from rl_nav_controller.utils import subtract_frame_transforms, transform_points, yaw_quat
from rl_nav_controller import constants
from rl_nav_controller.visualization import VisualizationManager
from rl_nav_controller.waypoint_manager import WaypointManager
from rl_nav_controller.path_tracker import NavState, PathSegmentTracker, BlockedDetector

try:
    import tf2_ros
    import tf2_geometry_msgs
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False


# --------------------------------------------------------------------------------
# Learning model - RL Navigation Controller (ONNX Runtime)
# --------------------------------------------------------------------------------

# Model architecture constants
STATE_DIM = 16  # linear_vel(3) + angular_vel(3) + gravity(3) + last_action(3) + target_pos_log(4)
DEPTH_EMBEDDING_DIM = 2560  # 64 * 5 * 8 (flattened VAE output)
POLICY_INPUT_DIM = STATE_DIM + DEPTH_EMBEDDING_DIM  # 2576
LSTM_HIDDEN_DIM = 512


class LearningModel:
    """Neural network model for RL-based navigation policy using ONNX Runtime.

    Model Architecture:
        VAE Encoder:
            - Input: depth_image [batch, 1, 40, 64]
            - Output: mu [batch, 64, 5, 8] -> flattened to 2560

        Navigation Policy (LSTM-based):
            - Inputs:
                - obs: [batch, 2576] (16 state features + 2560 depth embedding)
                - h_in: [1, batch, 512] (LSTM hidden state)
                - c_in: [1, batch, 512] (LSTM cell state)
            - Outputs:
                - actions: [batch, 3]
                - h_out: [1, batch, 512]
                - c_out: [1, batch, 512]
    """

    def __init__(self, preprocess_model_path, policy_model_path):
        """Initialize learning model with preprocessing and policy networks.

        Args:
            preprocess_model_path: Path to depth image preprocessing model (.onnx)
            policy_model_path: Path to navigation policy model (.onnx)
        """
        # Determine available execution providers
        available_providers = ort.get_available_providers()

        # Prefer CUDA if available, otherwise use CPU
        if 'CUDAExecutionProvider' in available_providers:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            print('\033[92m' + 'Using device: CUDA (ONNX Runtime)' + '\033[0m')
        else:
            providers = ['CPUExecutionProvider']
            print('\033[93m' + 'Using device: CPU (ONNX Runtime)' + '\033[0m')

        # Session options for optimization
        sess_options = ort.SessionOptions()
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        sess_options.intra_op_num_threads = 4

        # Load ONNX models
        self.preprocess_session = ort.InferenceSession(
            preprocess_model_path, sess_options=sess_options, providers=providers
        )
        self.policy_session = ort.InferenceSession(
            policy_model_path, sess_options=sess_options, providers=providers
        )

        # Get input/output names for the models
        self.preprocess_input_name = self.preprocess_session.get_inputs()[0].name
        self.preprocess_output_name = self.preprocess_session.get_outputs()[0].name

        # Policy model has 3 inputs: obs, h_in, c_in
        # and 3 outputs: actions, h_out, c_out
        self.policy_input_names = {inp.name: inp for inp in self.policy_session.get_inputs()}
        self.policy_output_names = [out.name for out in self.policy_session.get_outputs()]

        # Dynamically detect observation dimension and heading support
        obs_shape = self.policy_session.get_inputs()[0].shape
        self.policy_input_dim = obs_shape[1] if (len(obs_shape) > 1 and isinstance(obs_shape[1], int)) else POLICY_INPUT_DIM
        self.has_heading = (self.policy_input_dim == 2578)
        self.state_dim = 18 if self.has_heading else 16

        # Initialize LSTM hidden states
        self._h_state = np.zeros((1, 1, LSTM_HIDDEN_DIM), dtype=np.float32)
        self._c_state = np.zeros((1, 1, LSTM_HIDDEN_DIM), dtype=np.float32)

        # Policy output scaling
        self._policy_scale = np.array(constants.POLICY_SCALE, dtype=np.float32)

        # Warm up model with a fake run
        self.fake_run_once()

        print('\033[92m' + f'Learning model is ready (ONNX Runtime, state_dim={self.state_dim}, has_heading={self.has_heading}).' + '\033[0m')

    def _resize_bilinear(self, img: np.ndarray, target_size: tuple) -> np.ndarray:
        """Resize image using bilinear interpolation.

        Args:
            img: Input image of shape (H, W)
            target_size: Target size (H, W)

        Returns:
            Resized image of shape (target_H, target_W)
        """
        # cv2.resize expects (width, height), so we reverse target_size
        return cv2.resize(img, (target_size[1], target_size[0]), interpolation=cv2.INTER_LINEAR)

    def depth_preprocess(self, img: np.ndarray) -> np.ndarray:
        """Process depth image through the VAE encoder.

        Args:
            img: Depth image of shape (H, W)

        Returns:
            Flattened latent embedding of shape (2560,)
        """
        # Resize to (40, 64) using bilinear interpolation
        img_resized = self._resize_bilinear(img, (40, 64))

        # Add batch and channel dimensions: (H, W) -> (1, 1, H, W)
        img_tensor = img_resized.astype(np.float32)[np.newaxis, np.newaxis, :, :]

        # Run inference - output is [batch, 64, 5, 8]
        vae_output = self.preprocess_session.run(
            [self.preprocess_output_name],
            {self.preprocess_input_name: img_tensor}
        )[0]

        # Flatten the output: [1, 64, 5, 8] -> [2560]
        return vae_output.flatten()

    def normalize_target_position(self, target_pos_w, robot_pos_w, robot_orientation_w):
        """Normalize target position relative to robot frame.

        Args:
            target_pos_w: Target position in world frame [x, y, z]
            robot_pos_w: Robot position in world frame [x, y, z]
            robot_orientation_w: Robot orientation quaternion [w, x, y, z]

        Returns:
            Tuple of (normalized target position with log distance, target vector in base frame)
        """
        target_pos_w = np.array(target_pos_w, dtype=np.float32)[np.newaxis]
        robot_pos_w = np.array(robot_pos_w, dtype=np.float32)[np.newaxis]
        robot_orientation_w = np.array(robot_orientation_w, dtype=np.float32)[np.newaxis]

        inv_pos, inv_rot = subtract_frame_transforms(robot_pos_w, robot_orientation_w)
        target_vec_b = transform_points(target_pos_w, inv_pos, inv_rot)

        dist = np.linalg.norm(target_vec_b, axis=-1, keepdims=True) + 1e-6
        target_pos = target_vec_b / dist
        dist_log = np.log(dist + 1.0)

        target_pos = np.concatenate((target_pos, dist_log), axis=-1).reshape(-1)
        return target_pos, target_vec_b

    def reset_hidden_state(self):
        """Reset the LSTM hidden states to zeros."""
        self._h_state = np.zeros((1, 1, LSTM_HIDDEN_DIM), dtype=np.float32)
        self._c_state = np.zeros((1, 1, LSTM_HIDDEN_DIM), dtype=np.float32)

    def predict(
        self,
        linear_vel,
        angular_vel,
        gravity_vector,
        last_action,
        target_pos_w,
        robot_pos_w,
        robot_orientation_w,
        depth_image,
        target_yaw_w=None,
        is_reset=False
    ):
        """Run policy inference to get velocity command.

        Args:
            linear_vel: Linear velocity in base frame [vx, vy, vz]
            angular_vel: Angular velocity in base frame [wx, wy, wz]
            gravity_vector: Gravity vector in base frame [gx, gy, gz]
            last_action: Previous action [vx, vy, wz]
            target_pos_w: Target position in world frame
            robot_pos_w: Robot position in world frame
            robot_orientation_w: Robot orientation quaternion [w, x, y, z]
            depth_image: Depth image array
            target_yaw_w: Target yaw heading in world frame (radians)
            is_reset: Whether to reset hidden state

        Returns:
            Tuple of (velocity command, raw action, target vector in base frame)
        """
        # Reset hidden state if requested
        if is_reset:
            self.reset_hidden_state()

        # Preprocess depth image - returns flattened embedding of shape (2560,)
        depth_embedding = self.depth_preprocess(depth_image)

        # Normalize target position
        target_pos_log, target_vec_b = self.normalize_target_position(
            target_pos_w, robot_pos_w, robot_orientation_w
        )

        state_list = (
            list(linear_vel) + list(angular_vel) + list(gravity_vector) +
            list(last_action) + target_pos_log.tolist()
        )

        if self.has_heading:
            if target_yaw_w is not None and robot_orientation_w is not None:
                qw, qx, qy, qz = robot_orientation_w
                robot_yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
                yaw_err = (target_yaw_w - robot_yaw + np.pi) % (2.0 * np.pi) - np.pi
                target_heading = [float(np.cos(yaw_err)), float(np.sin(yaw_err))]
            else:
                target_heading = [1.0, 0.0]
            state_list.extend(target_heading)

        # Assemble state input tensor: 16 or 18 elements
        state_input = np.array(state_list, dtype=np.float32)

        # Combine state and depth embedding: [16/18] + [2560] = [2576/2578]
        obs = np.concatenate([state_input, depth_embedding])[np.newaxis].astype(np.float32)

        # Run policy inference with LSTM states
        outputs = self.policy_session.run(
            None,
            {
                'obs': obs,
                'h_in': self._h_state,
                'c_in': self._c_state
            }
        )

        raw_action, h_out, c_out = outputs

        # Update LSTM hidden states for next iteration
        self._h_state = h_out
        self._c_state = c_out

        # Apply tanh and scaling
        cmd_vel = np.tanh(raw_action) * self._policy_scale
        cmd_vel = cmd_vel.squeeze(0)
        raw_action = raw_action.squeeze(0)

        return cmd_vel, raw_action, target_vec_b

    def fake_run_once(self):
        """Warm up the model with a dummy inference."""
        linear_vel = [0.0, 0.0, 0.0]
        angular_vel = [0.0, 0.0, 0.0]
        gravity_vector = [0.0, 0.0, -1.0]
        target_pos_w = [1.0, 0.0, 0.0]
        robot_pos_w = [0.0, 0.0, 0.0]
        robot_orientation_w = [1.0, 0.0, 0.0, 0.0]
        last_action = [0.0, 0.0, 0.0]
        depth_image = np.random.rand(600, 960).astype(np.float32)

        cmd_vel, _, _ = self.predict(
            linear_vel,
            angular_vel,
            gravity_vector,
            last_action,
            target_pos_w,
            robot_pos_w,
            robot_orientation_w,
            depth_image,
            target_yaw_w=0.0,
            is_reset=True  # Reset hidden state for warmup
        )
        print(f'Predicted cmd_vel: linear_x={cmd_vel[0]:.4f}, '
              f'linear_y={cmd_vel[1]:.4f}, angular_z={cmd_vel[2]:.4f}')

        # Reset hidden state after warmup so it's clean for actual use
        self.reset_hidden_state()


# --------------------------------------------------------------------------------
# ROS2 Node
# --------------------------------------------------------------------------------
class NavigationPolicyNode(Node):
    def __init__(
        self,
        preprocess_model_path,
        policy_model_path,
        min_depth=constants.DEFAULT_MIN_DEPTH,
        max_depth=constants.DEFAULT_MAX_DEPTH,
        control_frequency=constants.DEFAULT_CONTROL_FREQUENCY,
        arrive_threshold=constants.ARRIVE_GOAL_THRESHOLD,
        use_sim=False
    ):
        """Initialize navigation policy node.

        Args:
            preprocess_model_path: Path to depth preprocessing model
            policy_model_path: Path to navigation policy model
            min_depth: Minimum valid depth value in meters
            max_depth: Maximum valid depth value in meters
            control_frequency: Control loop frequency in Hz
            arrive_threshold: Goal arrival distance threshold in meters
            use_sim: Whether running in simulation mode
        """
        super().__init__('navigation_policy_node')

        # Configuration parameters
        self.use_sim = use_sim
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.control_frequency = control_frequency
        self.odom_ready = False
        self.arrive_goal_threshold = arrive_threshold
        self.arrive_heading_threshold = getattr(constants, 'ARRIVE_HEADING_THRESHOLD', 0.1745)
        self.last_run_time = 0.0
        self.system_delay = constants.JOYSTICK_TIMEOUT

        # Parameters for Global Navigation Integration (GLOBAL_NAV_INTEGRATION_SPEC.md)
        self.declare_parameter('path_topic', getattr(constants, 'DEFAULT_PATH_TOPIC', '/global_path'))
        self.declare_parameter('stop_topic', getattr(constants, 'DEFAULT_STOP_TOPIC', '/mission/control'))
        self.declare_parameter('status_topic', getattr(constants, 'DEFAULT_STATUS_TOPIC', '/local_planner/status'))
        self.declare_parameter('status_msgs_format', getattr(constants, 'DEFAULT_STATUS_MSGS_FORMAT', 'list'))
        self.declare_parameter('start_navi_topic', getattr(constants, 'DEFAULT_START_NAVI_TOPIC', '/startNavi'))
        self.declare_parameter('motion_enable_topic', getattr(constants, 'DEFAULT_MOTION_ENABLE_TOPIC', '/motion_enabled'))
        self.declare_parameter('odom_topic', getattr(constants, 'DEFAULT_ODOM_TOPIC', '/lio/robo/odom'))
        self.declare_parameter('cmd_vel_topic', getattr(constants, 'DEFAULT_CMD_VEL_TOPIC', '/cmd_vel'))
        self.declare_parameter('auto_unlock_bridge', True)
        self.declare_parameter('lookahead_distance', getattr(constants, 'LOOKAHEAD_DISTANCE', 2.0))
        self.declare_parameter('segment_arrive_threshold', getattr(constants, 'SEGMENT_ARRIVE_THRESHOLD', 0.15))
        self.declare_parameter('blocked_timeout', getattr(constants, 'BLOCKED_TIMEOUT', 4.0))
        self.declare_parameter('blocked_dist_threshold', getattr(constants, 'BLOCKED_DIST_THRESHOLD', 0.1))
        self.declare_parameter('blocked_vel_threshold', getattr(constants, 'BLOCKED_VEL_THRESHOLD', 0.1))
        self.declare_parameter('path_start_max_dist', getattr(constants, 'PATH_START_MAX_DIST', 2.5))
        self.declare_parameter('status_heartbeat_frequency', getattr(constants, 'STATUS_HEARTBEAT_FREQUENCY', 1.0))
        self.declare_parameter('camera_timeout', getattr(constants, 'CAMERA_TIMEOUT', 2.0))
        self.declare_parameter('goal_reached_timeout', getattr(constants, 'DEFAULT_GOAL_REACHED_TIMEOUT', 1.0))
        self.declare_parameter('report_failure', getattr(constants, 'REPORT_FAILURE', True))
        self.declare_parameter('path_frame_id', '')

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.stop_topic = str(self.get_parameter('stop_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.status_msgs_format = str(self.get_parameter('status_msgs_format').value).strip().lower()
        self.start_navi_topic = str(self.get_parameter('start_navi_topic').value)
        self.motion_enable_topic = str(self.get_parameter('motion_enable_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.auto_unlock_bridge = bool(self.get_parameter('auto_unlock_bridge').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.report_failure = bool(self.get_parameter('report_failure').value)
        self.segment_arrive_threshold = float(self.get_parameter('segment_arrive_threshold').value)
        self.blocked_timeout = float(self.get_parameter('blocked_timeout').value)
        self.blocked_dist_threshold = float(self.get_parameter('blocked_dist_threshold').value)
        self.blocked_vel_threshold = float(self.get_parameter('blocked_vel_threshold').value)
        self.path_start_max_dist = float(self.get_parameter('path_start_max_dist').value)
        self.status_heartbeat_freq = float(self.get_parameter('status_heartbeat_frequency').value)
        self.camera_timeout = float(self.get_parameter('camera_timeout').value)
        self.goal_reached_timeout = float(self.get_parameter('goal_reached_timeout').value)
        self.configured_path_frame_id = str(self.get_parameter('path_frame_id').value).strip()

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/path_manager/path_manager_ros/nav_vel', 10
        )
        self.cmd_vel_direct_publisher = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )
        self.base_vel_publisher = self.create_publisher(
            Twist, '/walle_nav/robot_base_vel', 10
        )
        # Single publisher for all goal poses (moving goal, smart joystick, waypoints)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 1
        )

        # Global nav integration publishers (QoS: reliable + transient_local)
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_qos
        )
        self.start_navi_publisher = self.create_publisher(
            Bool, self.start_navi_topic, 10
        )
        motion_enable_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.motion_enable_publisher = self.create_publisher(
            Bool, self.motion_enable_topic, motion_enable_qos
        )
        self.lookahead_marker_publisher = self.create_publisher(
            Marker, '/vis/lookahead_goal', 10
        )

        # Visualization publishers
        self.twist_marker_publisher = self.create_publisher(
            Marker, 'vis/twist_cmd_marker', 10
        )
        self.goal_vector_marker_publisher = self.create_publisher(
            Marker, 'vis/goal_vector_marker', 10
        )
        self.moving_goal_marker_publisher = self.create_publisher(
            Marker, 'vis/moving_goal_marker', 10
        )
        self.recorded_waypoints_marker_pub = self.create_publisher(
            Marker, 'vis/recorded_waypoints_marker', 10
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        if self.odom_topic != '/loc/robo/odom':
            self.loc_odom_subscriber = self.create_subscription(
                Odometry, '/loc/robo/odom', self.loc_odom_callback, 10
            )
        else:
            self.loc_odom_subscriber = None
        self.depth_subscriber = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10
        )
        self.joy_subscriber = self.create_subscription(
            Joy, '/rsl_joy', self.joy_callback, 10
        )
        self.target_position_subscriber = self.create_subscription(
            PoseStamped, '/goal_pose', self.target_position_callback, 10
        )
        self.cur_goal_subscriber = self.create_subscription(
            PoseStamped, '/cur_goal', self.target_position_callback, 10
        )
        self.goal_3d_subscriber = self.create_subscription(
            PoseStamped, '/goal_3d', self.target_position_callback, 10
        )

        # Global path and mission control subscribers
        self.path_subscriber = self.create_subscription(
            Path, self.path_topic, self.global_path_callback, 10
        )
        self.stop_subscriber = self.create_subscription(
            String, self.stop_topic, self.mission_control_callback, 10
        )

        # Utilities
        self.bridge = CvBridge()
        self.model = LearningModel(
            preprocess_model_path=preprocess_model_path,
            policy_model_path=policy_model_path
        )
        self.waypoint_manager = WaypointManager(self.get_logger())
        self.visualization_manager = VisualizationManager(self)

        # Global navigation components
        self.path_tracker = PathSegmentTracker(self.get_logger())
        self.blocked_detector = BlockedDetector(
            timeout=self.blocked_timeout,
            dist_threshold=self.blocked_dist_threshold,
            vel_threshold=self.blocked_vel_threshold
        )
        self.state = NavState.IDLE
        self.error_msg = ""
        self.goal_reached_latched = False
        self.chassis_unlocked = False

        # TF2 buffer & listener for coordinate transforms
        self.has_tf2 = HAS_TF2
        if self.has_tf2:
            try:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            except Exception as e:
                self.get_logger().warn(f'Failed to initialize tf2_ros: {e}')
                self.has_tf2 = False
                self.tf_buffer = None
                self.tf_listener = None
        else:
            self.tf_buffer = None
            self.tf_listener = None

        # State variables - Robot state
        self.map_frame_id = None
        self.robot_frame_id = None
        self.robot_odom_time = 0.0
        self.robot_pos_w = None
        self.robot_orientation_w = None
        self.robot_pos_map = None
        self.robot_orientation_map = None
        self.linear_vel_w = None
        self.angular_vel_w = None
        self.linear_vel = None
        self.angular_vel = None
        self.gravity_vector = None
        self.depth_image = None
        self.last_depth_time = None
        self.last_state_change_time = time.time()

        # State variables - Navigation
        self.target_pos_w = None
        self.target_yaw_w = None
        self.last_target_pos = None
        self.is_reset_hidden_state = False
        self.last_action = self._reset_last_action()
        self.prev_cmd = np.zeros(3)  # For low-pass filtering
        self.is_abort_goal = False

        # Joystick state
        self.joy_linear_x = 0.0
        self.joy_linear_y = 0.0
        self.joy_angular_z = 0.0
        self.cmd_vel_ratio = 1.0
        self.joy_time = time.time()
        self.last_trigger_time = time.time()
        self.joy_active = False

        # Moving goal state
        self.moving_goal_delta = [0.0, 0.0, 0.0]

        # Smart joystick state
        self.smart_joystick_goal = [0.0, 0.0, 0.0]
        self.prev_smart_joystick_goal = [0.0, 0.0, 0.0]
        self.smart_joystick_mode_active = False
        self.smart_joystick_goal_aborted = False
        self.latest_joystick_axes = [0.0, 0.0, 0.0]

        # Timers
        self.create_timer(constants.WAYPOINT_PUBLISH_INTERVAL, self.publish_recorded_waypoints)
        self.create_timer(constants.TARGET_VECTOR_PUBLISH_INTERVAL, self.publish_target_vector)
        smart_joystick_interval = 1.0 / constants.SMART_JOYSTICK_UPDATE_FREQUENCY
        self.create_timer(smart_joystick_interval, self.update_smart_joystick_goal)

        # 1Hz status heartbeat timer (GLOBAL_NAV_INTEGRATION_SPEC.md)
        status_heartbeat_interval = 1.0 / max(0.1, self.status_heartbeat_freq)
        self.create_timer(status_heartbeat_interval, self.publish_status_heartbeat)

        # 5Hz sensor watchdog timer (monitors ZED camera depth frames)
        self.create_timer(0.2, self.check_sensor_health)

        # Initial status broadcast: IDLE
        self.publish_status_message()

        self.get_logger().info(
            '\033[92m' +
            f'Navigation policy node ready (Global Nav Segmented Mode active, state={self.state.value}).' +
            '\033[0m'
        )

    def odom_callback(self, odom_msg: Odometry):
        """Process odometry messages and update robot state.

        Args:
            odom_msg: Odometry message from the localization system
        """
        # 1) Update "time" from odom header
        self.robot_odom_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9

        # 2) Frame IDs (once)
        if self.map_frame_id is None:
            self.map_frame_id = odom_msg.header.frame_id
        if self.robot_frame_id is None:
            self.robot_frame_id = odom_msg.child_frame_id

        # 3) Store robot position & orientation
        self.robot_pos_w = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ]
        self.robot_orientation_w = [
            odom_msg.pose.pose.orientation.w,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z
        ]

        # 4) Extract raw linear & angular vel from odom (world‐frame)
        self.linear_vel_w = [
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z
        ]
        self.angular_vel_w = [
            odom_msg.twist.twist.angular.x,
            odom_msg.twist.twist.angular.y,
            odom_msg.twist.twist.angular.z
        ]

        # 5) Convert to base frame if not sim; else keep as‐is
        if self.use_sim:
            self.linear_vel = list(self.linear_vel_w)
            self.angular_vel = list(self.angular_vel_w)
        else:
            self.linear_vel = self.convert_vel_frame(self.linear_vel_w, self.robot_orientation_w)
            self.angular_vel = self.convert_vel_frame(self.angular_vel_w, self.robot_orientation_w)

        # 6) Publish "converted" base velocity
        self.publish_base_vel(self.linear_vel, self.angular_vel)

        # 7) Compute projected gravity vector in base frame
        self.gravity_vector = self.projected_gravity_vector(self.robot_orientation_w)

        # 8) Mark that odom is now available and auto unlock bridge
        if not self.odom_ready:
            self.odom_ready = True
            if self.auto_unlock_bridge and not self.chassis_unlocked:
                self.unlock_chassis()

        if odom_msg.header.frame_id == 'map':
            self.robot_pos_map = list(self.robot_pos_w)
            self.robot_orientation_map = list(self.robot_orientation_w)

    def loc_odom_callback(self, odom_msg: Odometry):
        """Process map-localized odometry from /loc/robo/odom in 'map' frame."""
        self.robot_pos_map = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ]
        self.robot_orientation_map = [
            odom_msg.pose.pose.orientation.w,
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z
        ]

    def get_robot_pose_in_frame(self, target_frame: str = 'map'):
        """Query the robot pose in the requested frame (default 'map').
        Prioritizes /loc/robo/odom when target_frame == 'map', then TF2 lookup, then fallback to odom."""
        if target_frame == 'map' and self.robot_pos_map is not None:
            return self.robot_pos_map, self.robot_orientation_map
        if self.map_frame_id == target_frame and self.robot_pos_w is not None:
            return self.robot_pos_w, self.robot_orientation_w
        if self.has_tf2 and self.tf_buffer is not None:
            try:
                base_frame = self.robot_frame_id if self.robot_frame_id else 'base'
                tf_stamped = self.tf_buffer.lookup_transform(
                    target_frame, base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05)
                )
                pos = [
                    tf_stamped.transform.translation.x,
                    tf_stamped.transform.translation.y,
                    tf_stamped.transform.translation.z
                ]
                ori = [
                    tf_stamped.transform.rotation.w,
                    tf_stamped.transform.rotation.x,
                    tf_stamped.transform.rotation.y,
                    tf_stamped.transform.rotation.z
                ]
                return pos, ori
            except Exception:
                pass
        return self.robot_pos_w, self.robot_orientation_w

    def unlock_chassis(self):
        """Auto unlock chassis gate by publishing True to /motion_enabled and /startNavi."""
        msg = Bool()
        msg.data = True
        if hasattr(self, 'start_navi_publisher') and self.start_navi_publisher is not None:
            self.start_navi_publisher.publish(msg)
        if hasattr(self, 'motion_enable_publisher') and self.motion_enable_publisher is not None:
            self.motion_enable_publisher.publish(msg)
        self.chassis_unlocked = True
        self.get_logger().info(
            '\033[92m' + f'[Auto-Unlock] Chassis gate unlocked: published {self.motion_enable_topic} & {self.start_navi_topic} = True' + '\033[0m'
        )

    def lock_chassis(self):
        """Lock chassis gate by publishing False to /motion_enabled and /startNavi on shutdown."""
        if self.auto_unlock_bridge and getattr(self, 'chassis_unlocked', False):
            self.chassis_unlocked = False
            try:
                msg = Bool()
                msg.data = False
                if rclpy.ok() and getattr(self, 'motion_enable_publisher', None) is not None:
                    self.motion_enable_publisher.publish(msg)
                if rclpy.ok() and getattr(self, 'start_navi_publisher', None) is not None:
                    self.start_navi_publisher.publish(msg)
                if rclpy.ok():
                    self.get_logger().info(
                        f'[Auto-Unlock] Chassis gate locked: published {self.motion_enable_topic} & {self.start_navi_topic} = False'
                    )
            except Exception:
                pass

    def set_state(self, new_state: NavState, reason: str = "", error_msg: str = ""):
        """Transition navigation state and trigger immediate event-driven status dispatch."""
        if self.state != new_state or (new_state == NavState.ERROR and error_msg != self.error_msg):
            old_state = self.state
            self.state = new_state
            self.last_state_change_time = time.time()
            if new_state == NavState.ERROR:
                self.error_msg = error_msg if error_msg else "unknown"
            else:
                self.error_msg = ""

            if new_state == NavState.GOAL_REACHED:
                self.goal_reached_latched = True
            else:
                self.goal_reached_latched = False

            self.get_logger().info(
                f'\033[96m[NavState] Transition: {old_state.value} -> {new_state.value}' +
                (f' ({reason})' if reason else '') +
                (f' [msgs="{self.error_msg}"]' if self.error_msg else '') + '\033[0m'
            )
            # Immediate millisecond-level event dispatch on state transition
            self.publish_status_message()

    def check_sensor_health(self):
        """Safety watchdog: monitor ZED depth camera frames and trigger error on timeout."""
        if self.state == NavState.RUNNING:
            now = time.time()
            if self.last_depth_time is None:
                elapsed = now - self.last_state_change_time
            else:
                elapsed = now - self.last_depth_time

            if elapsed > self.camera_timeout:
                if self.report_failure:
                    self._stop_robot()
                    self.set_state(
                        NavState.ERROR,
                        reason=f"ZED camera depth frame timeout ({elapsed:.1f}s > {self.camera_timeout:.1f}s)",
                        error_msg="zed_camera_lost"
                    )
                    self.get_logger().error(
                        '\033[91m' +
                        f'[Safety Watchdog] ZED camera depth frames lost ({elapsed:.1f}s > {self.camera_timeout:.1f}s)! Robot stopped, state -> ERROR (msgs="zed_camera_lost").' +
                        '\033[0m'
                    )
                else:
                    self.get_logger().warn(
                        f'[Safety Watchdog] ZED camera depth frame delayed ({elapsed:.1f}s > {self.camera_timeout:.1f}s), waiting for frame (report_failure=False)...',
                        throttle_duration_sec=3.0
                    )

    def format_status_json(self) -> str:
        """Format current state as standard JSON string per local_planner_status_diagram (1).html:
        {"state":"<state>", "msgs":[]} (where msgs is a real JSON array []).
        Compatible with multiple msgs styles via self.status_msgs_format:
          - 'list' / 'array' (default): non-error -> [], error -> [error_msg]
          - 'string_brackets': non-error -> "[]", error -> '["error_msg"]'
          - 'empty_string': non-error -> "", error -> error_msg
        """
        curr_state_val = self.state.value
        is_error = (self.state == NavState.ERROR and bool(self.error_msg))

        # If failure reporting is suppressed by user request, never report 'error' or 'failed'
        if not self.report_failure and self.state in (NavState.ERROR, NavState.FAILED):
            curr_state_val = NavState.RUNNING.value if self.path_tracker.is_active() else NavState.IDLE.value
            is_error = False

        if self.status_msgs_format in ('string_brackets', 'string_array'):
            msgs_val = json.dumps([self.error_msg]) if is_error else "[]"
        elif self.status_msgs_format in ('empty_string', 'string_empty', 'empty'):
            msgs_val = self.error_msg if is_error else ""
        elif self.status_msgs_format == 'brackets_all':
            msgs_val = f"[{self.error_msg}]" if is_error else "[]"
        else:
            # Default: 'list' (matches local_planner_status_diagram (1).html: msgs is real JSON array [])
            msgs_val = [self.error_msg] if is_error else []

        status_dict = {
            "state": curr_state_val,
            "msgs": msgs_val
        }
        return json.dumps(status_dict, separators=(',', ':'))

    def publish_status_message(self):
        """Publish current state as JSON to status topic (/local_planner/status)."""
        msg = String()
        msg.data = self.format_status_json()
        self.status_publisher.publish(msg)

    def publish_status_heartbeat(self):
        """Periodic 1Hz heartbeat to publish current state if unchanged.
        Per specification: running / idle / stopped / error repeat at 1Hz.
        """
        if self.state == NavState.GOAL_REACHED:
            self.set_state(NavState.IDLE, reason="Auto transition to IDLE")
            return
        self.publish_status_message()

    def _stop_robot(self):
        """Immediately brake and publish zero velocity to cmd_vel topics."""
        twist = Twist()
        twist.linear.x = 0.0 + self.joy_linear_x
        twist.linear.y = 0.0 + self.joy_linear_y
        twist.angular.z = 0.0 + self.joy_angular_z
        self.prev_cmd = np.zeros(3)
        self.cmd_vel_publisher.publish(twist)
        self.cmd_vel_direct_publisher.publish(twist)

    def mission_control_callback(self, msg: String):
        """Process incoming /mission/control commands ("pause", "resume", "stop", "continue").

        Args:
            msg: String message containing mission command.
        """
        raw = msg.data.strip()
        cmd = raw.lower()
        # Defensive: support both plain string ("pause"/"resume") and JSON (e.g. {"command":"pause"} or {"action":"pause"})
        if raw.startswith("{") and raw.endswith("}"):
            try:
                parsed = json.loads(raw)
                if isinstance(parsed, dict):
                    cmd = str(parsed.get("command") or parsed.get("action") or parsed.get("cmd") or parsed.get("state") or raw).strip().lower()
            except Exception:
                pass

        if cmd in ("pause", "stop"):
            # Emergency brake and enter STOPPED, automatically retaining current path
            self.set_state(NavState.STOPPED, reason=f"/mission/control received '{cmd}'")
            self._stop_robot()
            self.get_logger().warn(f'Navigation STOPPED by /mission/control ("{cmd}"). Current path automatically retained in memory.')
        elif cmd in ("resume", "continue"):
            # Resume if active path exists (from STOPPED, IDLE, or FAILED)
            if self.state in (NavState.STOPPED, NavState.IDLE, NavState.FAILED):
                if self.path_tracker.is_active():
                    self.goal_reached_latched = False
                    self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)
                    self.set_state(NavState.RUNNING, reason=f"/mission/control received '{cmd}' (resumed)")
                    self.get_logger().info('Navigation RESUMED, continuing retained path.')
                else:
                    self.set_state(NavState.IDLE, reason=f"/mission/control received '{cmd}' but no active path")
                    self.get_logger().info('Navigation remains in IDLE: no path retained.')
            elif self.state == NavState.ERROR:
                self.get_logger().error(
                    '\033[91m' +
                    f'Cannot resume: Controller is in ERROR state ({self.error_msg}). ' +
                    'Requires sensor recovery or terminal restart.' +
                    '\033[0m'
                )
        else:
            self.get_logger().warn(f"Unknown command on {self.stop_topic}: '{msg.data}'. Expected 'pause' or 'resume'.")

    # Alias for backward compatibility
    nav_stop_callback = mission_control_callback

    def global_path_callback(self, path_msg: Path):
        """Process incoming segmented global path.

        Args:
            path_msg: nav_msgs/msg/Path message.
        """
        # Reject path if in fatal unrecovered sensor error (e.g. camera lost)
        if self.report_failure and self.state == NavState.ERROR and (self.last_depth_time is None or time.time() - self.last_depth_time > self.camera_timeout):
            self._stop_robot()
            self.get_logger().error(
                '\033[91m' +
                f'[Path Rejected] Cannot accept new path: ZED camera is lost (state=ERROR, msgs="{self.error_msg}"). ' +
                'Robot cannot navigate without visual perception. Please restart terminal or recover camera.' +
                '\033[0m'
            )
            return

        if not path_msg.poses or len(path_msg.poses) == 0:
            self._stop_robot()
            self.path_tracker.clear()
            self.set_state(NavState.STOPPED, reason="Empty Path received (Task Cancelled)")
            self.get_logger().warn('Received empty Path segment: Task cancelled, State set to STOPPED.')
            return

        # 1) Coordinate frame check & TF2 transformation
        target_frame = self.map_frame_id if self.map_frame_id else 'world'
        frame_id = path_msg.header.frame_id.strip()

        # If configured path_frame_id parameter is set, prioritize it if header is empty
        if not frame_id and self.configured_path_frame_id:
            frame_id = self.configured_path_frame_id

        pts = []
        if frame_id and frame_id != target_frame and frame_id not in {'world', 'map', 'odom'}:
            if self.has_tf2 and self.tf_buffer is not None:
                try:
                    for pose_stamped in path_msg.poses:
                        transformed_pose = self.tf_buffer.transform(
                            pose_stamped, target_frame, timeout=Duration(seconds=0.2)
                        )
                        pts.append([
                            transformed_pose.pose.position.x,
                            transformed_pose.pose.position.y,
                            transformed_pose.pose.position.z
                        ])
                except Exception as e:
                    self.get_logger().error(
                        '\033[91m' +
                        f'[FATAL TF ERROR] Cannot transform path from frame "{frame_id}" to "{target_frame}": {e}. Rejecting path!' +
                        '\033[0m'
                    )
                    return
            else:
                self.get_logger().error(
                    '\033[91m' +
                    f'[FATAL ERROR] Path frame "{frame_id}" does not match "{target_frame}" and TF2 is unavailable! Rejecting path!' +
                    '\033[0m'
                )
                return
        else:
            if not frame_id:
                self.get_logger().warn(f'Path header.frame_id is empty, defaulting to onboard map frame: "{target_frame}".')
            for p in path_msg.poses:
                pts.append([p.pose.position.x, p.pose.position.y, p.pose.position.z])

        # 2) Extract terminal yaw if specified in last pose
        final_q = path_msg.poses[-1].pose.orientation
        norm_sq = final_q.x**2 + final_q.y**2 + final_q.z**2 + final_q.w**2
        terminal_yaw = None
        if norm_sq > 0.5:
            terminal_yaw = float(np.arctan2(
                2.0 * (final_q.w * final_q.z + final_q.x * final_q.y),
                1.0 - 2.0 * (final_q.y**2 + final_q.z**2)
            ))

        # 3) Safety sanity check: check distance from robot to first path point in the path frame
        robot_pos_in_frame, _ = self.get_robot_pose_in_frame(frame_id if frame_id else 'map')
        pos_check = robot_pos_in_frame if robot_pos_in_frame is not None else self.robot_pos_w
        if pos_check is not None:
            first_pt_dist = float(np.linalg.norm(np.array(pts[0][:2]) - np.array(pos_check[:2])))
            if first_pt_dist > self.path_start_max_dist:
                self.get_logger().warn(
                    '\033[93m' +
                    f'[SAFETY WARNING] Path start point is {first_pt_dist:.2f}m away from robot (> {self.path_start_max_dist}m). ' +
                    f'Path frame: "{frame_id}", Check localization alignment!' +
                    '\033[0m'
                )

        # 4) Load path into tracker and reset blocked detector
        self.goal_reached_latched = False
        self.path_tracker.set_path(pts, terminal_yaw=terminal_yaw)
        self.blocked_detector.reset(self.robot_odom_time, progress_s=0.0)

        # 5) State transition to RUNNING (unless currently STOPPED)
        if self.state == NavState.STOPPED:
            self._stop_robot()
            self.get_logger().info(
                '\033[93m' +
                f'Received new path segment ({len(pts)} points, {self.path_tracker.total_length:.2f}m) while STOPPED: ' +
                'retained path updated in memory. Robot remains STOPPED until "resume".' +
                '\033[0m'
            )
        else:
            self.set_state(
                NavState.RUNNING,
                reason=f"New path segment loaded ({len(pts)} points, {self.path_tracker.total_length:.2f}m)"
            )
            self.get_logger().info(
                '\033[92m' +
                f'Received new path segment: {len(pts)} points, total_length={self.path_tracker.total_length:.2f}m, ' +
                f'start=[{pts[0][0]:.2f}, {pts[0][1]:.2f}], terminal=[{pts[-1][0]:.2f}, {pts[-1][1]:.2f}]' +
                '\033[0m'
            )

    def depth_callback(self, depth_msg: Image):
        """Process depth image and trigger policy execution.

        Args:
            depth_msg: Depth image from the camera
        """
        # Update timestamp of last received depth frame
        self.last_depth_time = time.time()

        # If previously halted due to ZED camera loss, auto-recover if path is active
        if self.state == NavState.ERROR and self.error_msg == "zed_camera_lost":
            if self.path_tracker.is_active():
                self.get_logger().info(
                    '\033[92m' +
                    '[Safety Watchdog] ZED camera depth frames recovered! Resuming path tracking -> RUNNING.' +
                    '\033[0m'
                )
                self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)
                self.set_state(NavState.RUNNING, reason="ZED camera resumed")

        # 1) Don't proceed until odometry has come in at least once
        if not self.odom_ready:
            self.get_logger().warn('\033[93m' + 'Odometry not ready, skipping depth callback.' + '\033[0m')
            return

        # 2) Convert ROS Image → cv2 / NumPy, clamp NaNs / Infs / min/max
        try:
            depth_array = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

            # Fix near-wall vs far-distance 0.0 ambiguity:
            # 1. Distinguish between far distance (> max_depth or +inf) and close proximity / blindzone (< min_depth or nan/-inf)
            # Far/open space is mapped to 0.0 (aligned with VAE distant background representation).
            # Close obstacles, near-wall blindzone, and negative/close NaNs are clamped to min_depth (e.g. 0.25m),
            # preventing the robot from perceiving close walls/obstacles as open void!
            is_nan = np.isnan(depth_array)
            is_posinf = np.isposinf(depth_array)
            is_neginf = np.isneginf(depth_array)

            far_mask = is_posinf | (depth_array > self.max_depth)
            close_mask = is_neginf | ((depth_array < self.min_depth) & ~is_nan & ~far_mask)
            nan_mask = is_nan

            depth_array[far_mask] = 0.0
            depth_array[close_mask] = self.min_depth
            depth_array[nan_mask] = self.min_depth
            self.depth_image = depth_array
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return

        # 3) Respect control frequency (Hz)
        interval = 1.0 / self.control_frequency
        if (self.robot_odom_time - self.last_run_time) < interval:
            return

        self.last_run_time = self.robot_odom_time

        # 4) Everything is ready → run policy
        self.generate_cmd_vel()

    def generate_cmd_vel(self):
        """Generate and publish velocity command from policy network."""
        # Visualize moving goal marker
        self.visualization_manager.publish_moving_goal_marker(
            self.moving_goal_delta, self.robot_pos_w, self.robot_orientation_w,
            self.robot_odom_time, self.map_frame_id, self.moving_goal_marker_publisher
        )

        # Log smart joystick mode status
        if self.smart_joystick_mode_active:
            self.get_logger().info(
                f'Smart joystick mode active - Goal: [{self.smart_joystick_goal[0]:.2f}, '
                f'{self.smart_joystick_goal[1]:.2f}, {self.smart_joystick_goal[2]:.2f}]'
            )

        # 1) Check blocked / stopped / error states
        if self.state in (NavState.ERROR, NavState.FAILED, NavState.STOPPED):
            self._stop_robot()
            # If in STOPPED state and path is present, keep blocked_detector window refreshed
            # so pausing does not accumulate stale stationary time into stall timeout
            if self.state == NavState.STOPPED and hasattr(self, 'blocked_detector') and hasattr(self, 'path_tracker') and self.path_tracker.has_path():
                self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)
            return

        # Check aborted goal
        if self.is_abort_goal:
            self._stop_robot()
            if self.target_pos_w is not None:
                self.waypoint_manager.re_add_aborted_waypoint(self.target_pos_w)
            self.target_pos_w = None
            self.target_yaw_w = None
            self.path_tracker.clear()
            self.is_abort_goal = False
            self.last_action = self._reset_last_action()
            self.set_state(NavState.STOPPED, reason="Goal aborted")
            self.get_logger().info('Target aborted: State set to STOPPED.')
            return

        # 2) Segmented Path tracking logic (lookahead carrot point & blocked detection)
        pos_for_tracking = None
        ori_for_tracking = None
        if self.path_tracker.is_active():
            # Query robot pose in map frame (same frame as global path waypoints)
            pos_map, ori_map = self.get_robot_pose_in_frame('map')
            pos_for_tracking = pos_map if pos_map is not None else self.robot_pos_w
            ori_for_tracking = ori_map if ori_map is not None else self.robot_orientation_w

            if pos_for_tracking is None:
                self._stop_robot()
                return

            robot_xy = pos_for_tracking[:2]
            progress_s = self.path_tracker.update_progress(robot_xy)

            # Check if reached terminal goal of current segment
            if self.path_tracker.check_arrived(robot_xy, arrive_threshold=self.segment_arrive_threshold):
                self._stop_robot()
                self.path_tracker.clear()
                self.target_pos_w = None
                self.target_yaw_w = None
                self.last_action = self._reset_last_action()
                self.set_state(NavState.GOAL_REACHED, reason=f"Terminal goal reached (< {self.segment_arrive_threshold}m)")
                self.get_logger().info(
                    '\033[92m' +
                    f'[Path] Arrived at segment terminal goal! Actively braked (/cmd_vel=0), state -> GOAL_REACHED.' +
                    '\033[0m'
                )
                self.set_state(NavState.IDLE, reason="Goal reached reported, immediate transition to IDLE")
                return

            # Check if blocked / stalled against obstacles -> Transition to FAILED
            robot_vel_xy = self.linear_vel[:2] if self.linear_vel else [0.0, 0.0]
            if self.blocked_detector.update(self.robot_odom_time, progress_s, robot_vel_xy):
                if self.report_failure:
                    self._stop_robot()
                    self.set_state(
                        NavState.FAILED,
                        reason=f"Stalled: progress < {self.blocked_dist_threshold}m and vel < {self.blocked_vel_threshold}m/s for {self.blocked_timeout}s"
                    )
                    self.get_logger().warn(
                        '\033[91m' +
                        f'[Path] Robot is BLOCKED / STALLED! Stopped moving, state -> FAILED (awaiting upper layer replan or intervention).' +
                        '\033[0m'
                    )
                    return
                else:
                    self.get_logger().info(
                        f'[Path] Robot currently stalled/slow near obstacle ({self.blocked_timeout:.1f}s), continuing obstacle avoidance (report_failure=False)...',
                        throttle_duration_sec=3.0
                    )

            # Extract dynamic lookahead carrot point along the path
            carrot_pt, carrot_yaw = self.path_tracker.get_lookahead_point(lookahead_dist=self.lookahead_distance)
            self.target_pos_w = carrot_pt
            self.target_yaw_w = carrot_yaw

            # Publish lookahead marker for RViz (/vis/lookahead_goal)
            self.visualization_manager.publish_lookahead_goal_marker(
                self.target_pos_w, self.robot_odom_time, self.map_frame_id, self.lookahead_marker_publisher
            )

        elif self.target_pos_w is None:
            # No path and no target: stop
            self._stop_robot()
            if self.state in (NavState.RUNNING, NavState.GOAL_REACHED):
                self.set_state(NavState.IDLE, reason="No active target or path")
            return
        else:
            # Single goal tracking mode (e.g. from /goal_pose)
            is_arrived = self._check_goal_reached(
                self.target_pos_w, self.robot_pos_w, self.target_yaw_w, self.robot_orientation_w
            )
            if is_arrived:
                self._stop_robot()
                self.target_pos_w = None
                self.target_yaw_w = None
                self.last_action = self._reset_last_action()
                self.set_state(NavState.GOAL_REACHED, reason="Single goal reached")
                self.get_logger().info('Target reached: Target position reset, state -> GOAL_REACHED.')
                self.set_state(NavState.IDLE, reason="Single goal reached reported, immediate transition to IDLE")
                return

        # 3) Joystick timeout logic
        if self.joy_active and (self.robot_odom_time - self.joy_time > self.system_delay):
            self.cmd_vel_ratio = 0.0
            diff = self.robot_odom_time - self.joy_time
            self.get_logger().warn(
                '\033[93m' +
                f'Joystick timeout, stopping the robot. Time diff: {diff:.4f}s' +
                '\033[0m'
            )
        elif not self.joy_active:
            self.cmd_vel_ratio = 1.0

        self.get_logger().info(f'Joystick cmd_vel ratio: {self.cmd_vel_ratio:.4f}')

        # 4) Call the learning model
        curr_robot_pos = pos_for_tracking if (pos_for_tracking is not None) else self.robot_pos_w
        curr_robot_ori = ori_for_tracking if (ori_for_tracking is not None) else self.robot_orientation_w

        cmd, action, target_vec_b = self.model.predict(
            self.linear_vel,
            self.angular_vel,
            self.gravity_vector,
            self.last_action,
            self.target_pos_w,
            curr_robot_pos,
            curr_robot_ori,
            self.depth_image,
            target_yaw_w=self.target_yaw_w,
            is_reset=self.is_reset_hidden_state
        )

        if self.is_reset_hidden_state:
            self.get_logger().warn('\033[93m' + 'Resetting hidden state.' + '\033[0m')
            self.is_reset_hidden_state = False
            self.last_action = self._reset_last_action()

        self.last_action = action.tolist()

        # Apply low-pass filter and form Twist message
        twist = Twist()
        # Policy command (no joystick)
        model_cmd = np.array([
            cmd[0].item() * self.cmd_vel_ratio,
            cmd[1].item() * self.cmd_vel_ratio * constants.LATERAL_VELOCITY_SCALE,
            cmd[2].item() * self.cmd_vel_ratio
        ])
        filter_coef = np.array(constants.LOW_PASS_FILTER_COEF)
        # Low-pass filter on model command
        filt_model = filter_coef * model_cmd + (1 - filter_coef) * self.prev_cmd
        # Add joystick inputs without filtering
        twist.linear.x = float(filt_model[0]) + self.joy_linear_x
        twist.linear.y = float(filt_model[1]) + self.joy_linear_y
        twist.angular.z = float(filt_model[2]) + self.joy_angular_z
        # Update previous filtered command
        self.prev_cmd = filt_model
        self.cmd_vel_publisher.publish(twist)
        self.cmd_vel_direct_publisher.publish(twist)

        # Publish visualization markers
        self.visualization_manager.publish_twist_marker(
            twist, self.robot_odom_time, self.robot_frame_id, self.twist_marker_publisher
        )

        self.get_logger().info(
            f'Published cmd_vel: linear_x={twist.linear.x:.4f}, '
            f'linear_y={twist.linear.y:.4f}, angular_z={twist.angular.z:.4f}'
        )
        # Reset joystick deltas for next cycle
        self._reset_joystick()

    # ------------------------------
    # "Base‐frame" velocity publisher (called from odom_callback)
    # ------------------------------
    def publish_base_vel(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel[0]
        twist.linear.y = linear_vel[1]
        twist.angular.z = angular_vel[2]
        self.base_vel_publisher.publish(twist)

    # ------------------------------
    # Convert a vector in world frame → robot (base) frame
    # ------------------------------
    def convert_vel_frame(self, vel_vec, orientation_w):
        quat_xyzw = [orientation_w[1], orientation_w[2], orientation_w[3], orientation_w[0]]
        rotation = R.from_quat(quat_xyzw)
        inv_rot = rotation.inv()
        out = inv_rot.apply(np.array(vel_vec))
        return out.tolist()

    # ------------------------------
    # Compute projected gravity in robot frame
    # ------------------------------
    def projected_gravity_vector(self, robot_orientation_w):
        quat_xyzw = [robot_orientation_w[1], robot_orientation_w[2], robot_orientation_w[3], robot_orientation_w[0]]
        rotation = R.from_quat(quat_xyzw)
        inv_rot = rotation.inv()

        gravity = np.array([0.0, 0.0, -9.81])
        proj = inv_rot.apply(gravity)
        proj = proj / (np.linalg.norm(proj) + 1e-6)
        return proj.tolist()

    def joy_callback(self, joy_msg: Joy):
        """Process joystick input and update control state.

        Args:
            joy_msg: Joystick message with axes and button states
        """
        self.joy_active = True
        self.cmd_vel_ratio = (1.0 + joy_msg.axes[4]) * 1.0

        btn_pause = getattr(constants, 'BUTTON_PAUSE', 1)  # Button B (暂停/急停)
        btn_start = getattr(constants, 'BUTTON_START', 0)  # Button A (启动/恢复)
        btn_abort = getattr(constants, 'BUTTON_ABORT', 1)

        # 1. 按钮 B (Pause / Stop): 暂停导航并即时刹停 (保持当前路径不清除)
        if (len(joy_msg.buttons) > btn_pause and joy_msg.buttons[btn_pause] == 1) or \
           (len(joy_msg.buttons) > btn_abort and joy_msg.buttons[btn_abort] == 1):
            self.is_abort_goal = False
            self.set_state(NavState.STOPPED, reason="Joystick B button (Pause/Stop) pressed")
            self._stop_robot()
            if hasattr(self, 'blocked_detector') and hasattr(self, 'path_tracker'):
                self.blocked_detector.reset(
                    self.robot_odom_time,
                    self.path_tracker.current_progress if self.path_tracker.has_path() else 0.0
                )
            self.get_logger().warn(
                '\033[93m[Joystick] Button B pressed: Navigation PAUSED/STOPPED (Robot actively braked, cmd_vel=0, path retained).\033[0m'
            )

        dx, dy, dz = self._moving_xyz_with_buttons(joy_msg, scale=constants.MOVING_SCALE)
        self.moving_goal_delta[0] += dx
        self.moving_goal_delta[1] += dy
        self.moving_goal_delta[2] += dz

        current_time = time.time()
        cooldown = constants.TRIGGER_BUTTON_COOLDOWN

        # 2. 按钮 A (Start / Resume): 启动导航或从暂停状态恢复
        if (current_time - self.last_trigger_time) > cooldown and \
           len(joy_msg.buttons) > btn_start and joy_msg.buttons[btn_start] == 1:
            self.last_trigger_time = current_time
            self.is_abort_goal = False
            self.goal_reached_latched = False
            self.unlock_chassis()

            if self.path_tracker.has_path():
                # Realign path tracker to current robot pose before resuming
                pos_in_frame, _ = self.get_robot_pose_in_frame('map')
                pos_check = pos_in_frame if pos_in_frame is not None else self.robot_pos_w
                if pos_check is not None:
                    self.path_tracker.realign_to_robot(pos_check[:2])

                # Reset blocked detector so resuming has a clean 4.0s stall detection window
                if hasattr(self, 'blocked_detector'):
                    self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)

                self.set_state(NavState.RUNNING, reason="Joystick A button (Resume Path) pressed")
                self.get_logger().info(
                    '\033[92m[Joystick] Button A pressed: Resumed global path navigation with realigned progress!\033[0m'
                )
            elif self.target_pos_w is not None or self.last_target_pos is not None:
                if self.target_pos_w is None and self.last_target_pos is not None:
                    self.target_pos_w = list(self.last_target_pos)
                self.set_state(NavState.RUNNING, reason="Joystick A button (Start/Resume Goal) pressed")
                self.get_logger().info(
                    '\033[92m' +
                    f'[Joystick] Button A pressed: Started/Resumed goal navigation to [{self.target_pos_w[0]:.2f}, {self.target_pos_w[1]:.2f}, {self.target_pos_w[2]:.2f}]!' +
                    '\033[0m'
                )
            elif self.waypoint_manager.has_home_waypoints():
                self.get_logger().info('\033[92m[Joystick] Button A pressed: Starting home waypoint sequence!\033[0m')
                self.waypoint_manager.start_home_waypoint_sequence()
            elif self.waypoint_manager.has_inversed_waypoints():
                self.get_logger().info('\033[92m[Joystick] Button A pressed: Starting inversed waypoint sequence!\033[0m')
                self.waypoint_manager.start_inversed_waypoint_sequence()
            else:
                self.get_logger().info(
                    '\033[93m[Joystick] Button A pressed: Chassis unlocked, waiting for target goal or path.\033[0m'
                )

        if not self.is_abort_goal and (current_time - self.last_trigger_time) > cooldown and \
           len(joy_msg.buttons) > constants.BUTTON_SEND_GOAL and joy_msg.buttons[constants.BUTTON_SEND_GOAL] == 1:
            self.get_logger().warn('Trigger moving goal')
            self._publish_moving_goal()
            self.last_trigger_time = current_time

        if (current_time - self.last_trigger_time) > cooldown and \
           len(joy_msg.buttons) > constants.BUTTON_RECORD_WAYPOINT and joy_msg.buttons[constants.BUTTON_RECORD_WAYPOINT] == 1:
            self.waypoint_manager.record_waypoint(self.robot_pos_w)
            self.last_trigger_time = current_time

        if (current_time - self.last_trigger_time) > cooldown and \
           len(joy_msg.buttons) > constants.BUTTON_CLEAR_WAYPOINT and joy_msg.buttons[constants.BUTTON_CLEAR_WAYPOINT] == 1:
            self.waypoint_manager.remove_last_waypoint()
            self.last_trigger_time = current_time

        if joy_msg.axes[constants.JOYSTICK_AXIS_SMART] < -0.5:
            # Smart joystick mode enabled
            if not self.smart_joystick_mode_active:
                # Entering smart joystick mode for the first time
                self.smart_joystick_mode_active = True
                self.smart_joystick_goal_aborted = False

            # Abort current goal only once when entering smart joystick mode
            if not self.smart_joystick_goal_aborted and self.target_pos_w is not None:
                self.is_abort_goal = True
                self.smart_joystick_goal_aborted = True
                self.get_logger().info('Smart joystick mode: aborting current navigation goal')

            # Store the latest joystick axes values
            # Goal will be updated at controlled frequency in a timer callback
            self.latest_joystick_axes = [
                joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_X],
                joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_Y],
                joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_Z]
            ]

        else:
            # Smart joystick mode disabled - use direct joystick control
            if self.smart_joystick_mode_active:
                # Exiting smart joystick mode
                self.smart_joystick_mode_active = False
                self.is_abort_goal = True
                self.get_logger().info('Exiting smart joystick mode: aborting current navigation goal')

            self._reset_smart_joystick_goal()
            # Use the joystick axes to directly control the robot
            self.joy_linear_x = joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_X] * constants.LINEAR_SCALE * 1.5
            self.joy_linear_y = joy_msg.axes[constants.JOYSTICK_AXIS_LINEAR_Y] * constants.LINEAR_SCALE
            self.joy_angular_z = joy_msg.axes[constants.JOYSTICK_AXIS_ANGULAR_Z] * constants.ANGULAR_SCALE

        if joy_msg.buttons[constants.BUTTON_RESET_HIDDEN_STATE] == 1:
            self.is_reset_hidden_state = True
            self.get_logger().warn('Force Reset hidden state')

        self.joy_time = time.time()

    def _generate_waypoint_using_joystick(self, linear_x, linear_y, linear_z):
        """Generate a waypoint using joystick inputs relative to robot's current position.

        The goal is always calculated as an offset from the current robot position,
        preventing unbounded accumulation. Low-pass filtering is applied for smoothness.

        Args:
            linear_x: Joystick input for forward/backward
            linear_y: Joystick input for left/right
            linear_z: Joystick input for up/down
        """
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            return

        # Scale joystick inputs to goal offset in robot frame
        goal_offset_robot = np.array([
            linear_x * constants.SMART_JOYSTICK_SCALE,
            linear_y * constants.SMART_JOYSTICK_SCALE,
            linear_z * constants.SMART_JOYSTICK_SCALE * constants.SMART_JOYSTICK_Z_SCALE
        ], dtype=np.float32)

        # Transform the offset from robot frame to world frame
        robot_ori = np.array(self.robot_orientation_w, dtype=np.float32)
        robot_yaw_ori = yaw_quat(robot_ori)

        # Transform the offset (rotation only, no translation)
        goal_offset_world = transform_points(
            goal_offset_robot[np.newaxis],
            np.zeros(3, dtype=np.float32)[np.newaxis],  # Zero translation
            robot_yaw_ori[np.newaxis]
        )[0]

        # Calculate target goal as robot position + offset (always relative to current position)
        target_goal_world = [
            self.robot_pos_w[0] + goal_offset_world[0].item(),
            self.robot_pos_w[1] + goal_offset_world[1].item(),
            self.robot_pos_w[2] + goal_offset_world[2].item()
        ]

        # Apply low-pass filter to smooth goal changes
        # If this is the first update or goal was reset, initialize to target
        if self.prev_smart_joystick_goal == [0.0, 0.0, 0.0]:
            self.smart_joystick_goal = target_goal_world
        else:
            alpha = constants.SMART_JOYSTICK_FILTER_ALPHA
            self.smart_joystick_goal = [
                alpha * target_goal_world[0] + (1 - alpha) * self.prev_smart_joystick_goal[0],
                alpha * target_goal_world[1] + (1 - alpha) * self.prev_smart_joystick_goal[1],
                alpha * target_goal_world[2] + (1 - alpha) * self.prev_smart_joystick_goal[2]
            ]

        # Update previous goal for next iteration
        self.prev_smart_joystick_goal = list(self.smart_joystick_goal)

    def _publish_smart_joystick_goal(self):
        """Publish the smart joystick goal (already in world frame)."""
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            self.get_logger().warn('Cannot publish smart joystick goal: robot pose not available')
            return

        # Goal is already in world frame, just publish it directly
        self._publish_goal(self.smart_joystick_goal)

    def _reset_smart_joystick_goal(self):
        """Reset smart joystick goal state to origin."""
        self.smart_joystick_goal = [0.0, 0.0, 0.0]
        self.prev_smart_joystick_goal = [0.0, 0.0, 0.0]


    def _moving_xyz_with_buttons(self, joy_msg: Joy, scale=1.0):
        """Map discrete button presses to movement increments.

        Args:
            joy_msg: Joystick message
            scale: Scale factor for movement

        Returns:
            Tuple of (dx, dy, dz) increments
        """
        dx = scale * (bool(joy_msg.buttons[constants.BUTTON_FORWARD]) -
                      bool(joy_msg.buttons[constants.BUTTON_BACKWARD]))
        dy = scale * (bool(joy_msg.buttons[constants.BUTTON_LEFT]) -
                      bool(joy_msg.buttons[constants.BUTTON_RIGHT]))
        dz = (scale / 5.0) * (bool(joy_msg.buttons[constants.BUTTON_UP]) -
                               bool(joy_msg.buttons[constants.BUTTON_DOWN]))
        return dx, dy, dz

    def _publish_goal(self, goal_pos):
        """Publish a goal pose to the unified goal_pose topic.

        Args:
            goal_pos: Goal position [x, y, z] in world frame
        """
        goal_pose = PoseStamped()
        goal_pose.header = Header()
        goal_pose.header.stamp = Time(nanoseconds=int(self.robot_odom_time * 1e9)).to_msg()
        goal_pose.header.frame_id = self.map_frame_id
        goal_pose.pose.position.x = goal_pos[0]
        goal_pose.pose.position.y = goal_pos[1]
        goal_pose.pose.position.z = goal_pos[2]
        self.goal_pose_publisher.publish(goal_pose)

    def _publish_moving_goal(self):
        """Convert moving goal delta from robot frame to world frame and publish."""
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            self.get_logger().warn('Cannot publish moving goal: robot pose/orientation not yet set.')
            return

        robot_pos = np.array(self.robot_pos_w, dtype=np.float32)
        robot_ori = np.array(self.robot_orientation_w, dtype=np.float32)
        robot_yaw_ori = yaw_quat(robot_ori)

        moving_goal_pos = np.array(self.moving_goal_delta, dtype=np.float32)
        # Transform robot-frame delta to world-frame coordinate
        moving_goal_pos_w = transform_points(
            moving_goal_pos[np.newaxis],
            robot_pos[np.newaxis],
            robot_yaw_ori[np.newaxis]
        )

        # Publish the world-frame goal
        self._publish_goal(moving_goal_pos_w[0].tolist())

        # Reset the moving_goal_delta
        self._reset_moving_goal()

    def publish_recorded_waypoints(self):
        """Timer callback to publish recorded waypoints sequentially."""
        if self.robot_pos_w is None:
            self.get_logger().warn(
                'Waiting for robot position to initialize (waiting for odometry on "/lio/robo/odom")...',
                throttle_duration_sec=4.0
            )
            return

        # Visualize waypoints
        self.visualization_manager.publish_waypoints_marker(
            self.waypoint_manager.waypoints_visualization,
            self.map_frame_id,
            self.get_clock(),
            self.recorded_waypoints_marker_pub
        )

        # Publish home waypoints
        if self.waypoint_manager.is_home_sequence_active() and self.waypoint_manager.has_home_waypoints():
            if self.target_pos_w is None or self._check_near_goal(self.target_pos_w, self.robot_pos_w):
                next_wp = self.waypoint_manager.get_next_waypoint_home()
                self.get_logger().info(f'Publishing next waypoint: {next_wp}')
                self._publish_goal(next_wp)
            else:
                self.get_logger().info('Tracking the current waypoint ...')

            if not self.waypoint_manager.has_home_waypoints():
                self.waypoint_manager.stop_home_waypoint_sequence()

        # Publish inversed waypoints
        if self.waypoint_manager.is_inversed_sequence_active() and self.waypoint_manager.has_inversed_waypoints():
            if self.target_pos_w is None or self._check_near_goal(self.target_pos_w, self.robot_pos_w):
                next_wp = self.waypoint_manager.get_next_waypoint_inversed()
                self.get_logger().info(f'Publishing next inversed waypoint: {next_wp}')
                self._publish_goal(next_wp)
            else:
                self.get_logger().info('Tracking the current waypoint ...')

            if not self.waypoint_manager.has_inversed_waypoints():
                self.waypoint_manager.stop_inversed_waypoint_sequence()

        # Reset visualization if all waypoints published
        self.waypoint_manager.reset_visualization_if_complete()

    def _check_near_goal(self, target_pos_w, robot_pos_w):
        """Check if robot is near the goal (wider threshold than arrival).

        Args:
            target_pos_w: Target position in world frame
            robot_pos_w: Robot position in world frame

        Returns:
            True if near the goal, False otherwise
        """
        if target_pos_w is None or robot_pos_w is None:
            return False

        dist = np.linalg.norm(np.array(target_pos_w[:2]) - np.array(robot_pos_w[:2]))
        threshold = self.arrive_goal_threshold * constants.NEAR_GOAL_THRESHOLD_MULTIPLIER
        if dist > threshold:
            return False
        else:
            self.get_logger().info('Near the current goal position.')
            return True

    def _check_goal_reached(self, target_pos_w, robot_pos_w, target_yaw_w=None, robot_orientation_w=None):
        """Check if robot has arrived at the goal.

        Args:
            target_pos_w: Target position in world frame
            robot_pos_w: Robot position in world frame
            target_yaw_w: Target yaw heading in world frame (radians)
            robot_orientation_w: Robot orientation quaternion [w, x, y, z]

        Returns:
            True if arrived, False otherwise
        """
        if target_pos_w is None or robot_pos_w is None:
            return False

        dist = np.linalg.norm(np.array(target_pos_w[:2]) - np.array(robot_pos_w[:2]))
        if dist > self.arrive_goal_threshold:
            return False

        if getattr(self.model, 'has_heading', False) and target_yaw_w is not None and robot_orientation_w is not None:
            qw, qx, qy, qz = robot_orientation_w
            robot_yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            yaw_diff = abs((target_yaw_w - robot_yaw + np.pi) % (2.0 * np.pi) - np.pi)
            if yaw_diff > self.arrive_heading_threshold:
                self.get_logger().info(
                    f'At goal position ({dist:.2f}m <= {self.arrive_goal_threshold:.2f}m), '
                    f'aligning heading: {np.degrees(yaw_diff):.1f}° > {np.degrees(self.arrive_heading_threshold):.1f}°'
                )
                return False

        self.get_logger().info(f'Arrived at the goal position and heading. Distance: {dist:.3f}m <= {self.arrive_goal_threshold:.3f}m')
        return True

    def target_position_callback(self, msg: PoseStamped):
        """Process incoming goal pose messages.

        Args:
            msg: PoseStamped message with goal position
        """
        # Accept map, world, odom, empty string, or matching map_frame_id
        valid_frames = {self.map_frame_id, 'world', 'map', 'odom', ''}
        if self.map_frame_id and msg.header.frame_id not in valid_frames:
            self.get_logger().error(
                '\033[91m' +
                f'Target frame_id "{msg.header.frame_id}" does not match odometry frame_id "{self.map_frame_id}"' +
                '\033[0m'
            )
            return

        # If the received goal z is zero or very close to zero, use the robot's current z
        goal_z = msg.pose.position.z
        if abs(goal_z) < 1e-3:
            # Use robot's current z if available, otherwise fallback to received z
            if self.robot_pos_w is not None:
                goal_z = self.robot_pos_w[2]
            else:
                self.get_logger().warn('Robot position not available, using received z for target.')
        self.target_pos_w = [
            msg.pose.position.x,
            msg.pose.position.y,
            goal_z
        ]
        # Store as last target
        self.last_target_pos = list(self.target_pos_w)

        # Parse target orientation
        q = msg.pose.orientation
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq > 0.5:
            self.target_yaw_w = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        else:
            if self.robot_orientation_w is not None:
                qw, qx, qy, qz = self.robot_orientation_w
                self.target_yaw_w = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            else:
                self.target_yaw_w = 0.0

        dist = np.linalg.norm(np.array(self.target_pos_w[:2]) - np.array(self.robot_pos_w[:2])) if self.robot_pos_w is not None else -1.0
        robot_pos_str = f"[{self.robot_pos_w[0]:.2f}, {self.robot_pos_w[1]:.2f}, {self.robot_pos_w[2]:.2f}]" if self.robot_pos_w else "N/A"
        self.path_tracker.clear()
        self.blocked_detector.reset(self.robot_odom_time, progress_s=0.0)
        self.set_state(NavState.RUNNING, reason="Single goal received")
        self.get_logger().info(
            '\033[92m' +
            f'Received new target position: [{self.target_pos_w[0]:.2f}, {self.target_pos_w[1]:.2f}, {self.target_pos_w[2]:.2f}], ' +
            f'target_yaw: {self.target_yaw_w:.2f} rad ({np.degrees(self.target_yaw_w):.1f}°), ' +
            f'Robot: {robot_pos_str}, Distance: {dist:.2f}m' +
            '\033[0m'
        )

    def _reset_last_action(self):
        """Reset the last action vector and filter state.

        Returns:
            Zero action vector [0, 0, 0]
        """
        self.prev_cmd = np.zeros(3)
        return [0.0, 0.0, 0.0]

    def _reset_joystick(self):
        """Reset joystick control deltas to zero."""
        self.joy_linear_x = 0.0
        self.joy_linear_y = 0.0
        self.joy_angular_z = 0.0

    def _reset_moving_goal(self):
        """Reset moving goal delta to zero."""
        self.moving_goal_delta = [0.0, 0.0, 0.0]
        self.get_logger().info('Reset moving goal delta.')

    def publish_target_vector(self):
        """Timer callback to continuously publish target vector marker."""
        if self.robot_pos_w is None or self.robot_orientation_w is None:
            return

        # Use current or last target
        tgt = self.target_pos_w if self.target_pos_w is not None else self.last_target_pos
        if tgt is None:
            return

        # Transform target position to robot frame
        _, target_vec_b = self.model.normalize_target_position(
            tgt, self.robot_pos_w, self.robot_orientation_w
        )

        # Publish visualization marker
        self.visualization_manager.publish_target_vector_marker(
            target_vec_b, self.robot_odom_time, self.robot_frame_id,
            self.goal_vector_marker_publisher
        )

    def update_smart_joystick_goal(self):
        """Timer callback to update smart joystick goal at controlled frequency."""
        if not self.smart_joystick_mode_active:
            return

        # Generate and publish waypoint using the latest joystick axes
        self._generate_waypoint_using_joystick(
            self.latest_joystick_axes[0],
            self.latest_joystick_axes[1],
            self.latest_joystick_axes[2]
        )
        self._publish_smart_joystick_goal()


# --------------------------------------------------------------------------------
# Main: initialize and spin
# --------------------------------------------------------------------------------
def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='run in simulation mode')
    parser.add_argument('--policy-path', type=str, default=None, help='Path to custom nav_policy.onnx')
    parser.add_argument('--arrive-threshold', type=float, default=0.45, help='Goal arrival distance threshold in meters (default: 0.45)')
    parser.add_argument('--path-topic', type=str, default=None, help='Global path topic name (default: /global_path)')
    parser.add_argument('--stop-topic', type=str, default=None, help='Mission control topic name (default: /mission/control)')
    parser.add_argument('--control-topic', type=str, default=None, help='Mission control topic alias (default: /mission/control)')
    parser.add_argument('--status-topic', type=str, default=None, help='Local planner status topic name (default: /local_planner/status)')
    parser.add_argument('--status-msgs-format', type=str, default=None, choices=['string_brackets', 'list', 'empty_string', 'brackets_all'], help='Format for msgs in status JSON: string_brackets ("[]"), list ([]), empty_string ("")')
    parser.add_argument('--cmd-vel-topic', type=str, default=None, help='Velocity command topic name (default: /cmd_vel)')
    parser.add_argument('--motion-enable-topic', type=str, default=None, help='Chassis motion enable topic name (default: /motion_enabled)')
    parser.add_argument('--odom-topic', type=str, default=None, help='Robot odometry topic name (default: /lio/robo/odom)')
    parser.add_argument('--lookahead-distance', type=float, default=None, help='Dynamic lookahead distance in meters (default: 2.0)')
    parser.add_argument('--report-failure', dest='report_failure', action='store_true', default=None, help='Enable reporting FAILED/ERROR states (default: True)')
    parser.add_argument('--no-report-failure', dest='report_failure', action='store_false', help='Disable reporting FAILED/ERROR states')
    parser.add_argument('--segment-arrive-threshold', type=float, default=None, help='Segment arrival distance threshold in meters (default: 0.15)')
    parser.add_argument('--blocked-timeout', type=float, default=None, help='Stall detection timeout in seconds (default: 4.0)')
    known_args, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    use_sim = known_args.sim

    pkg_name = 'rl_nav_controller'
    pkg_path = get_package_share_directory(pkg_name)
    preprocess_model_path = os.path.join(pkg_path, 'deployment_policies', 'vae_encoder.onnx')
    
    if known_args.policy_path:
        policy_model_path = os.path.expanduser(known_args.policy_path)
    else:
        policy_model_path = os.path.join(pkg_path, 'deployment_policies', 'nav_policy.onnx')

    navigation_policy_node = NavigationPolicyNode(
        preprocess_model_path=preprocess_model_path,
        policy_model_path=policy_model_path,
        min_depth=0.25,
        max_depth=10.0,
        control_frequency=5.0,
        arrive_threshold=known_args.arrive_threshold,
        use_sim=use_sim
    )

    if known_args.path_topic:
        navigation_policy_node.path_topic = known_args.path_topic
    if known_args.control_topic:
        navigation_policy_node.stop_topic = known_args.control_topic
    elif known_args.stop_topic:
        navigation_policy_node.stop_topic = known_args.stop_topic
    if known_args.status_topic:
        navigation_policy_node.status_topic = known_args.status_topic
    if known_args.status_msgs_format:
        navigation_policy_node.status_msgs_format = known_args.status_msgs_format
    if known_args.cmd_vel_topic:
        navigation_policy_node.cmd_vel_topic = known_args.cmd_vel_topic
    if known_args.motion_enable_topic:
        navigation_policy_node.motion_enable_topic = known_args.motion_enable_topic
    if known_args.odom_topic:
        navigation_policy_node.odom_topic = known_args.odom_topic
    if known_args.lookahead_distance is not None:
        navigation_policy_node.lookahead_distance = known_args.lookahead_distance
    if known_args.report_failure is not None:
        navigation_policy_node.report_failure = known_args.report_failure
    if known_args.segment_arrive_threshold is not None:
        navigation_policy_node.segment_arrive_threshold = known_args.segment_arrive_threshold
    elif known_args.arrive_threshold is not None:
        navigation_policy_node.segment_arrive_threshold = known_args.arrive_threshold
    if known_args.blocked_timeout is not None:
        navigation_policy_node.blocked_timeout = known_args.blocked_timeout
        navigation_policy_node.blocked_detector.timeout = known_args.blocked_timeout

    navigation_policy_node.get_logger().info(
        f'RL Navigation launched in {"SIMULATION" if use_sim else "REAL-HARDWARE"} mode. '
        f'Segment arrive threshold: {navigation_policy_node.segment_arrive_threshold:.3f}m, '
        f'Lookahead: {navigation_policy_node.lookahead_distance:.2f}m, '
        f'Report failure: {navigation_policy_node.report_failure}'
    )
    navigation_policy_node.get_logger().info(
        '\033[92m' + f'Active navigation policy model: {policy_model_path}' + '\033[0m'
    )

    try:
        rclpy.spin(navigation_policy_node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            navigation_policy_node.lock_chassis()
        except Exception:
            pass
        try:
            navigation_policy_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

"""Unit tests for VLN base-frame goal handling in the SRU navigation node.

These tests exercise the goal-interpretation and stale-goal watchdog logic
without constructing the ROS node (and therefore without ONNX/camera/odom).

Run with the ROS 2 Humble Python, e.g.:

    source /opt/ros/humble/setup.bash
    python3 -m pytest test/test_vln_goal_logic.py -q
"""

import math
import os
import sys
import time
import types

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('geometry_msgs')

# The node module imports onnxruntime at top level; stub it before import so
# the logic can be tested on machines with the ROS runtime only.
if 'onnxruntime' not in sys.modules:
    ort = types.ModuleType('onnxruntime')
    ort.InferenceSession = object
    ort.get_available_providers = lambda: []
    ort.SessionOptions = object
    ort.GraphOptimizationLevel = types.SimpleNamespace(ORT_ENABLE_ALL=0)
    sys.modules['onnxruntime'] = ort

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from geometry_msgs.msg import PoseStamped  # noqa: E402
from rl_nav_controller.rl_nav_controller import NavigationPolicyNode, NavState  # noqa: E402


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kwargs):
        self.lines.append(('info', msg))

    def warn(self, msg, **kwargs):
        self.lines.append(('warn', msg))

    def error(self, msg, **kwargs):
        self.lines.append(('error', msg))


class _Tracker:
    def clear(self):
        pass


class _BlockedDetector:
    def reset(self, *args, **kwargs):
        pass


def make_node(yaw=math.pi / 2.0):
    """Bare node instance with only the attributes used by the goal path."""
    node = object.__new__(NavigationPolicyNode)
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.goal_frame_mode = 'auto'
    node.base_frame_ids = {'base', 'base_link', 'base_footprint', 'body', 'imu'}
    node.goal_stream_timeout = 0.0
    node.base_goal_max_distance = 5.0
    node.base_goal_arrive_threshold = 0.10
    node.arrive_goal_threshold = 0.45
    node.arrive_heading_threshold = 0.1745
    node.model = types.SimpleNamespace(has_heading=False)

    node.robot_pos_w = [2.0, 3.0, 0.3]
    node.robot_orientation_w = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    node.robot_odom_time = 10.0
    node.map_frame_id = 'world'

    node.target_pos_w = None
    node.target_yaw_w = None
    node.last_target_pos = None
    node.goal_is_relative = False
    node.last_goal_rx_time = None
    node.user_pause_latched = False

    node.path_tracker = _Tracker()
    node.blocked_detector = _BlockedDetector()
    node.is_aligning_heading = False
    node.heading_align_target_yaw = None
    node.heading_align_terminal_pos = None
    node.heading_align_start_time = None
    node.heading_align_stable_count = 0

    node.state = NavState.IDLE
    node.error_msg = ''
    node.prev_cmd = __import__('numpy').zeros(3)

    node.stop_calls = 0
    node._stop_robot = lambda: setattr(node, 'stop_calls', node.stop_calls + 1)
    node.publish_status_message = lambda: None
    return node


def pose_msg(frame_id, x, y, z, yaw=None):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    if yaw is not None:
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
    return msg


def test_base_frame_detection_modes():
    node = make_node()
    assert node._is_base_frame_goal('base_link') is True
    assert node._is_base_frame_goal('world') is False
    # Empty frame keeps the legacy world interpretation in auto mode.
    assert node._is_base_frame_goal('') is False

    node.goal_frame_mode = 'base'
    assert node._is_base_frame_goal('world') is True
    node.goal_frame_mode = 'world'
    assert node._is_base_frame_goal('base_link') is False


def test_base_goal_transforms_to_world():
    node = make_node()
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.goal_is_relative is True
    assert node.last_goal_rx_time is not None
    assert node.target_pos_w[0] == pytest.approx(2.0)
    assert node.target_pos_w[1] == pytest.approx(4.0)
    assert node.target_pos_w[2] == pytest.approx(0.3)
    assert node.state == NavState.RUNNING


def test_base_goal_heading_is_relative_to_robot_yaw():
    node = make_node()
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0, yaw=math.pi / 2))
    assert abs(node.target_yaw_w) == pytest.approx(math.pi, abs=1e-3)

    node = make_node()
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.target_yaw_w == pytest.approx(math.pi / 2, abs=1e-3)


def test_base_goal_distance_clamped():
    node = make_node()
    node._handle_base_frame_goal(pose_msg('base_link', 10.0, 0.0, 0.0))
    dist = math.hypot(node.target_pos_w[0] - 2.0, node.target_pos_w[1] - 3.0)
    assert dist == pytest.approx(5.0, abs=1e-3)


def test_base_goal_arrival_radius_is_10cm():
    node = make_node()
    node.robot_pos_w = [2.0, 3.0, 0.3]

    node.target_pos_w = [2.05, 3.0, 0.3]
    assert node._check_goal_reached(
        node.target_pos_w, node.robot_pos_w,
        arrive_threshold=node.base_goal_arrive_threshold) is True

    node.target_pos_w = [2.20, 3.0, 0.3]
    assert node._check_goal_reached(
        node.target_pos_w, node.robot_pos_w,
        arrive_threshold=node.base_goal_arrive_threshold) is False


def test_same_base_point_is_relatched_from_new_pose():
    # No deduplication: every message is transformed with the current pose,
    # which lets VLN refine a point near the previous one.
    node = make_node()
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    first_target = list(node.target_pos_w)

    node.robot_pos_w = [3.0, 3.0, 0.3]
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.target_pos_w[0] == pytest.approx(3.0)
    assert node.target_pos_w[1] == pytest.approx(4.0)
    assert node.target_pos_w != first_target


def test_base_goal_ignored_without_odometry():
    node = make_node()
    node.robot_pos_w = None
    node.robot_orientation_w = None
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.goal_is_relative is False
    assert node.target_pos_w is None


def test_stale_goal_stream_brakes_and_stops():
    node = make_node()
    node.goal_stream_timeout = 2.0
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    node.last_goal_rx_time = time.time() - 2.5
    node.check_goal_stream_timeout()
    assert node.stop_calls == 1
    assert node.state == NavState.STOPPED
    assert node.target_pos_w is None
    assert node.target_yaw_w is None
    assert node.goal_is_relative is False
    assert node.last_goal_rx_time is None


def test_fresh_stream_is_not_stopped():
    node = make_node()
    node.goal_stream_timeout = 2.0
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    node.check_goal_stream_timeout()
    assert node.stop_calls == 0
    assert node.state == NavState.RUNNING


def test_watchdog_disabled():
    node = make_node()
    node.goal_stream_timeout = 0.0
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    node.last_goal_rx_time = time.time() - 10.0
    node.check_goal_stream_timeout()
    assert node.stop_calls == 0


def test_absolute_goal_leaves_vln_mode():
    node = make_node()
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    node.target_position_callback(pose_msg('world', 7.0, 8.0, 0.0))
    assert node.goal_is_relative is False
    assert node.last_goal_rx_time is None
    assert node.target_pos_w[0] == pytest.approx(7.0)
    assert node.target_pos_w[1] == pytest.approx(8.0)
    assert node.state == NavState.RUNNING


def test_manual_pause_is_not_overridden_by_stream():
    """A mission pause stays STOPPED; a B emergency stop stays RUNNING."""
    node = make_node()
    node.user_pause_latched = True
    node.state = NavState.STOPPED
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.state == NavState.STOPPED
    assert node.target_pos_w is not None

    # B-style emergency stop: the gate is closed but the task stays RUNNING,
    # the stream only refreshes the target.
    node.state = NavState.RUNNING
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.state == NavState.RUNNING

    # A clears the latch; the following VLN goals keep the mission RUNNING.
    node.user_pause_latched = False
    node._handle_base_frame_goal(pose_msg('base_link', 1.0, 0.0, 0.0))
    assert node.state == NavState.RUNNING

"""Regression test for "B pause, then dispatch a new goal/path, don't press A".

Reported symptom: after pressing B (pause) the operator re-issues a goal; if A
is not pressed immediately the controller reported ``{"state":"failed"}`` a few
seconds later, even though the robot was only waiting for A.

Root cause: both the single-goal handler (``target_position_callback``, fed by
``/goal_3d`` / ``/goal_pose`` / ``/cur_goal``) and ``global_path_callback``
unconditionally resumed ``RUNNING``. The chassis gate is still closed until A,
so the robot could not move and the 4 s blocked detector tripped into FAILED.

These tests pin the rule: a latched manual pause (B / motion gate / mission
pause) must survive a newly dispatched goal or path, and only A / resume may
clear it.
"""

import os
import sys
import types

import pytest

pytest.importorskip('rclpy')

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
from nav_msgs.msg import Path  # noqa: E402

from rl_nav_controller.rl_nav_controller_adaptive import (  # noqa: E402
    NavigationPolicyNode, NavState, motion_enable_subscription_qos)
from rl_nav_controller.path_tracker import PathSegmentTracker, BlockedDetector  # noqa: E402


class _Logger:
    def info(self, msg, **kwargs):
        pass

    def warn(self, msg, **kwargs):
        pass

    def error(self, msg, **kwargs):
        pass


class _Node:
    """Object.__new__ shell exercising the real state machine bits."""

    def __init__(self, paused=False):
        node = object.__new__(NavigationPolicyNode)
        node.get_logger = lambda: _Logger()
        node.state = NavState.STOPPED if paused else NavState.IDLE
        node.error_msg = ''
        node.last_state_change_time = 0.0
        node.goal_reached_latched = False
        node.user_pause_latched = bool(paused)
        node.publish_status_message = lambda: None

        node.goal_frame_mode = 'auto'
        node.base_frame_ids = set()
        node.map_frame_id = 'map'
        node.configured_path_frame_id = ''
        node.path_start_max_dist = 5.0
        node.goal_is_relative = False
        node.last_goal_rx_time = None

        node.robot_odom_time = 1.0
        node.robot_pos_w = [0.0, 0.0, 0.0]
        node.robot_orientation_w = [1.0, 0.0, 0.0, 0.0]
        node.target_pos_w = None
        node.last_target_pos = None
        node.target_yaw_w = 0.0

        node.path_tracker = PathSegmentTracker()
        node.blocked_detector = BlockedDetector()
        node.is_aligning_heading = False
        node.heading_align_target_yaw = None
        node.heading_align_terminal_pos = None
        node.heading_align_start_time = None
        node.heading_align_stable_count = 0

        node.report_failure = True
        node.last_depth_time = None
        node.camera_timeout = 3.0
        node.stop_calls = 0

        node.get_robot_pose_in_frame = lambda frame='map': (
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        node._stop_robot = self._stop
        self.node = node

    def _stop(self):
        self.node.stop_calls += 1

    # --- helpers -------------------------------------------------------
    def goal(self, x=3.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def path(self, points=((0.0, 0.0, 0.0), (3.0, 0.0, 0.0))):
        msg = Path()
        msg.header.frame_id = 'map'
        for x, y, z in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def send_goal(self, **kwargs):
        NavigationPolicyNode.target_position_callback(self.node, self.goal(**kwargs))

    def send_path(self, **kwargs):
        NavigationPolicyNode.global_path_callback(self.node, self.path(**kwargs))


def test_goal_while_paused_stays_stopped():
    """B, then /goal_3d, without A -> STOPPED (never RUNNING -> no false FAILED)."""
    h = _Node(paused=True)
    h.send_goal()
    assert h.node.state == NavState.STOPPED
    assert h.node.user_pause_latched is True
    # The goal itself is retained, so A can start it.
    assert h.node.target_pos_w is not None
    assert h.node.last_target_pos is not None


def test_goal_while_running_starts_normally():
    h = _Node(paused=False)
    h.send_goal()
    assert h.node.state == NavState.RUNNING


def test_path_while_paused_stays_stopped_and_is_retained():
    """B, then a new /global_path, without A -> STOPPED with the path retained."""
    h = _Node(paused=True)
    h.send_path()
    assert h.node.state == NavState.STOPPED
    assert h.node.path_tracker.has_path() is True
    assert h.node.stop_calls >= 1


def test_path_while_failed_but_pause_latched_holds_stop():
    """Even if the node had tripped to FAILED, a latched pause wins over auto-run."""
    h = _Node(paused=True)
    h.node.state = NavState.FAILED
    h.send_path()
    assert h.node.state == NavState.STOPPED
    assert h.node.path_tracker.has_path() is True


def test_path_while_running_loads_and_runs():
    h = _Node(paused=False)
    h.node.state = NavState.RUNNING
    h.send_path()
    assert h.node.state == NavState.RUNNING
    assert h.node.path_tracker.has_path() is True


def test_resume_clears_latch_and_runs_retained_path():
    """A repeated after a goal+path arrived while paused must actually resume."""
    h = _Node(paused=True)
    h.send_goal()
    h.send_path()
    assert h.node.state == NavState.STOPPED

    from std_msgs.msg import Bool
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=True))
    assert h.node.user_pause_latched is False
    assert h.node.state == NavState.RUNNING


def test_motion_enable_subscription_is_volatile():
    """/motion_enabled must be subscribed VOLATILE to match either bridge.

    A TRANSIENT_LOCAL request is rejected when the chassis bridge publishes
    VOLATILE (unitree_bridge), which silently dropped every B/A event.
    """
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    qos = motion_enable_subscription_qos()
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.reliability == ReliabilityPolicy.RELIABLE

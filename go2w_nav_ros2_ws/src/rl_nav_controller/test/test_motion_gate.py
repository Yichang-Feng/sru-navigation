"""Regression test for the B-button / motion-gate pause path.

Pressing B publishes /motion_enabled=False. The controller must brake, latch
the pause and keep the node alive; pressing A must resume. This path used to
call PathSegmentTracker.has_path(), which was missing and raised AttributeError
inside the callback (the node then shut down).
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

from std_msgs.msg import Bool  # noqa: E402
from rl_nav_controller.rl_nav_controller import NavigationPolicyNode, NavState  # noqa: E402
from rl_nav_controller.path_tracker import PathSegmentTracker, BlockedDetector  # noqa: E402


class _Logger:
    def info(self, msg, **kwargs):
        pass

    def warn(self, msg, **kwargs):
        pass

    def error(self, msg, **kwargs):
        pass


class _Node:
    def __init__(self):
        node = object.__new__(NavigationPolicyNode)
        node.get_logger = lambda: _Logger()
        node.state = NavState.RUNNING
        node.user_pause_latched = False
        node.is_abort_goal = False
        node.goal_reached_latched = False
        node.robot_odom_time = 1.0
        node.path_tracker = PathSegmentTracker()
        node.blocked_detector = BlockedDetector()
        node.set_state = self._set_state
        node._stop_robot = lambda: None
        node.publish_status_message = lambda: None
        node.get_robot_pose_in_frame = lambda frame='map': ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        node.stop_calls = 0
        node._stop_robot = self._stop
        self.node = node

    def _set_state(self, new_state, reason='', error_msg=''):
        self.node.state = new_state

    def _stop(self):
        self.node.stop_calls += 1

    def disable_motion(self):
        NavigationPolicyNode.motion_enable_callback(self.node, Bool(data=False))

    def enable_motion(self):
        NavigationPolicyNode.motion_enable_callback(self.node, Bool(data=True))


def test_b_disable_brakes_and_keeps_node_alive_no_path():
    h = _Node()
    h.disable_motion()
    assert h.node.state == NavState.STOPPED
    assert h.node.user_pause_latched is True
    assert h.node.stop_calls >= 1

    h.enable_motion()
    assert h.node.state == NavState.IDLE
    assert h.node.user_pause_latched is False


def test_b_disable_then_a_resumes_with_active_path():
    h = _Node()
    h.node.path_tracker.set_path([[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]], frame_id='world')
    h.disable_motion()
    assert h.node.state == NavState.STOPPED
    assert h.node.path_tracker.has_path() is True

    h.enable_motion()
    assert h.node.state == NavState.RUNNING
    assert h.node.user_pause_latched is False
    assert h.node.path_tracker.current_progress >= 0.0

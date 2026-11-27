"""Regression test for the B-button / motion-gate emergency-stop path.

Pressing B publishes /motion_enabled=False. The controller must brake and
latch the pause, but the task state must stay RUNNING (the mission is still in
progress, the dog is only emergency-stopped) and it must never be reported as
FAILED. Pressing A clears the latch and re-arms the 4.0 s stall window, so a
robot that still cannot move after A is then correctly reported as FAILED.

This path used to call PathSegmentTracker.has_path(), which was missing and
raised AttributeError inside the callback (the node then shut down), and later
used to force STOPPED on every B press.
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
        node.chassis_motion_enabled = True
        node.motion_enable_topic = '/motion_enabled'
        node.is_abort_goal = False
        node.goal_reached_latched = False
        node.robot_odom_time = 1.0
        node.blocked_timeout = 4.0
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


def test_b_disable_keeps_running_with_active_path():
    """B is an emergency stop of the chassis, not a task abort: stay RUNNING."""
    h = _Node()
    h.node.path_tracker.set_path([[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]], frame_id='world')
    h.disable_motion()
    assert h.node.state == NavState.RUNNING
    assert h.node.user_pause_latched is True
    assert h.node.stop_calls >= 1
    assert h.node.path_tracker.has_path() is True

    h.enable_motion()
    assert h.node.state == NavState.RUNNING
    assert h.node.user_pause_latched is False
    assert h.node.path_tracker.current_progress >= 0.0


def test_b_disable_on_idle_stays_idle():
    """With no task in progress B must not fabricate a RUNNING state."""
    h = _Node()
    h.node.state = NavState.IDLE
    h.disable_motion()
    assert h.node.state == NavState.IDLE
    assert h.node.user_pause_latched is True

    h.enable_motion()
    assert h.node.state == NavState.IDLE
    assert h.node.user_pause_latched is False


def test_b_disable_from_failed_does_not_enter_stopped():
    """B never rewrites the state: an existing FAILED simply stays FAILED."""
    h = _Node()
    h.node.state = NavState.FAILED
    h.disable_motion()
    assert h.node.state == NavState.FAILED
    assert h.node.user_pause_latched is True

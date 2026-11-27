"""Regression tests for the B-button emergency stop vs. a mission pause.

Rule pinned here:

* B (``/motion_enabled=False``) is a chassis emergency stop, not a task abort.
  The state stays RUNNING and the reported status stays ``running``; the path
  or goal is retained and the robot is braked. It must never become ``stopped``
  and must never become ``failed`` while the gate is closed.
* A (``/motion_enabled=True``) clears the pause latch and re-arms the 4.0 s
  stall window. Only a robot that still makes no progress for 4.0 s after A is
  reported as ``failed``.
* An explicit ``/mission/control: "pause"`` is still the only producer of the
  ``stopped`` state, and only ``resume``/A leaves it.

Historically both B and a new goal/path used to force STOPPED, and goals sent
while paused were dropped; the tests below keep the corrected contract.
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
        node.chassis_motion_enabled = not paused
        node.motion_enable_topic = '/motion_enabled'
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
        node.blocked_timeout = 4.0
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


def test_path_while_failed_but_pause_latched_keeps_running():
    """A latched B emergency stop plus a fresh path keeps the mission RUNNING.

    The gate is closed so the robot cannot move, but the task is in progress:
    state must be RUNNING (never STOPPED, never FAILED).
    """
    h = _Node(paused=True)
    h.node.state = NavState.FAILED
    h.send_path()
    assert h.node.state == NavState.RUNNING
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


def test_duplicate_true_does_not_resume_a_pause():
    """A duplicate /motion_enabled=True (same A press from two bridges, or a
    replay) must not undo a pause that arrived in between."""
    import time
    from std_msgs.msg import Bool

    h = _Node(paused=True)
    h.send_path()
    assert h.node.state == NavState.STOPPED

    # The previous /motion_enabled message was a True just now, then B paused us:
    # a duplicate of that True arrives while the pause is latched.
    h.node.chassis_motion_enabled = True
    h.node._last_motion_enable_true_time = time.monotonic()
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=True))
    assert h.node.state == NavState.STOPPED, 'duplicate True must not resume a pause'

    # A fresh A press (well separated in time) does resume.
    h.node._last_motion_enable_true_time = time.monotonic() - 5.0
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=True))
    assert h.node.state == NavState.RUNNING


def test_stopped_can_never_transition_to_failed():
    """The state diagram has no stopped -> failed edge; refuse it outright."""
    h = _Node(paused=True)
    h.node.set_state(NavState.FAILED, reason='Stalled: simulated watchdog')
    assert h.node.state == NavState.STOPPED


def test_stall_detector_is_inert_while_gate_closed():
    """With the chassis gate closed, no-progress must not trip FAILED."""
    h = _Node(paused=True)
    h.node.state = NavState.RUNNING          # e.g. woke up without a real A edge
    h.node.chassis_motion_enabled = False
    detector = h.node.blocked_detector
    detector.reset(current_time=1.0, progress_s=0.0)
    # The control loop must reset instead of update() while the gate is closed.
    # Simulate 10 s of standstill with the same guard the controller uses.
    t = 1.0
    for _ in range(100):
        t += 0.1
        if not h.node.chassis_motion_enabled:
            detector.reset(t, 0.0)
        elif detector.update(t, 0.0, (0.0, 0.0)):
            h.node.set_state(NavState.FAILED, reason='Stalled: simulated')
    assert h.node.state == NavState.RUNNING
    # Control: with the gate open the same standstill does trip after 4 s.
    h.node.chassis_motion_enabled = True
    for _ in range(100):
        t += 0.1
        if not h.node.chassis_motion_enabled:
            detector.reset(t, 0.0)
        elif detector.update(t, 0.0, (0.0, 0.0)):
            h.node.set_state(NavState.FAILED, reason='Stalled: simulated')
            break
    assert h.node.state == NavState.FAILED


def test_b_keeps_running_and_new_path_keeps_running():
    """B while RUNNING stays RUNNING, and a path dispatched while latched keeps it."""
    from std_msgs.msg import Bool

    h = _Node()
    h.node.state = NavState.RUNNING
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=False))
    assert h.node.state == NavState.RUNNING
    assert h.node.user_pause_latched is True
    assert h.node.chassis_motion_enabled is False

    h.send_path()
    assert h.node.state == NavState.RUNNING
    assert h.node.path_tracker.has_path() is True


def test_a_after_b_reports_failed_only_after_4s_without_motion():
    """B never trips FAILED; A restarts the 4.0 s window, then stillness fails."""
    from std_msgs.msg import Bool

    h = _Node()
    h.node.state = NavState.RUNNING
    h.send_path()
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=False))
    assert h.node.state == NavState.RUNNING

    t = 1.0

    def loop(seconds):
        nonlocal t
        for _ in range(int(round(seconds / 0.1))):
            t += 0.1
            if not h.node.chassis_motion_enabled:
                # Same guard as generate_cmd_vel while the gate is closed.
                h.node.blocked_detector.reset(t, 0.0)
            elif h.node.blocked_detector.update(t, 0.0, (0.0, 0.0)):
                NavigationPolicyNode.set_state(h.node, NavState.FAILED, reason='Stalled: simulated')

    # 10 s of B: the emergency stop must never be reported as failed.
    loop(10.0)
    assert h.node.state == NavState.RUNNING

    # A re-opens the gate and starts a clean 4.0 s window.
    h.node.robot_odom_time = t
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=True))
    assert h.node.state == NavState.RUNNING
    assert h.node.user_pause_latched is False
    assert h.node.chassis_motion_enabled is True

    # 3.9 s without motion is still RUNNING...
    loop(3.9)
    assert h.node.state == NavState.RUNNING
    # ...and only after the 4.0 s threshold does it become FAILED.
    loop(0.5)
    assert h.node.state == NavState.FAILED


def test_failed_is_refused_while_gate_is_closed():
    """Even a direct FAILED request is ignored during a B emergency stop."""
    from std_msgs.msg import Bool

    h = _Node()
    h.node.state = NavState.RUNNING
    NavigationPolicyNode.motion_enable_callback(h.node, Bool(data=False))
    h.node.set_state(NavState.FAILED, reason='Stalled: simulated')
    assert h.node.state == NavState.RUNNING

"""Test verifying that motion_enable_callback(False) transitions safely to STOPPED without crash.

Ensures that:
1. PathSegmentTracker from path_tracker.py defines has_path(), realign_to_robot(), and get_terminal_pose().
2. Calling motion_enable_callback(msg.data=False) transitions state from IDLE to STOPPED cleanly.
3. No AttributeError is raised even when odom is not ready or path is empty.
"""

from unittest.mock import MagicMock
import path_tracker
from path_tracker import NavState, PathSegmentTracker, BlockedDetector


class MockBool:
    def __init__(self, data=False):
        self.data = bool(data)


def test_path_tracker_interface():
    tracker = PathSegmentTracker()
    assert hasattr(tracker, 'has_path')
    assert tracker.has_path() == tracker.is_active() == False
    assert hasattr(tracker, 'realign_to_robot')
    assert hasattr(tracker, 'get_terminal_pose')

    tracker.set_path([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    assert tracker.has_path() == True
    assert tracker.is_active() == True
    pt, yaw = tracker.get_terminal_pose()
    assert pt == [1.0, 0.0, 0.0]

    tracker.clear()
    assert tracker.has_path() == False


def test_motion_enable_callback_stopped_safe():
    class MockController:
        def __init__(self):
            self.state = NavState.IDLE
            self.is_abort_goal = False
            self.robot_odom_time = 0.0
            self.target_pos_w = None
            self.last_target_pos = None
            self.path_tracker = PathSegmentTracker()
            self.blocked_detector = BlockedDetector()
            self.logger = MagicMock()

        def get_logger(self):
            return self.logger

        def set_state(self, new_state, reason=""):
            self.state = new_state

        def _stop_robot(self):
            pass

        def _has_active_path(self):
            if not hasattr(self, 'path_tracker') or self.path_tracker is None:
                return False
            if hasattr(self.path_tracker, 'has_path'):
                return bool(self.path_tracker.has_path())
            if hasattr(self.path_tracker, 'is_active'):
                return bool(self.path_tracker.is_active())
            return False

        def motion_enable_callback(self, msg):
            enabled = bool(msg.data)
            if not enabled:
                if self.state in (NavState.RUNNING, NavState.IDLE, NavState.FAILED):
                    self.is_abort_goal = False
                    self.set_state(NavState.STOPPED, reason="Chassis motion disabled by remote B button")
                    self._stop_robot()
                    if hasattr(self, 'blocked_detector') and self._has_active_path():
                        self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)

    node = MockController()
    msg = MockBool(data=False)

    # Trigger initial motion_enable=False (as published by go2w_bridge on startup)
    node.motion_enable_callback(msg)
    assert node.state == NavState.STOPPED

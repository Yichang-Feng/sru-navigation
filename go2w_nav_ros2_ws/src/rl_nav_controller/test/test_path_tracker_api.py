"""API tests for the non-adaptive PathSegmentTracker used by rl_nav_controller.

The B-button / STOPPED paths of rl_nav_controller call has_path(),
realign_to_robot(), get_terminal_pose() and set_path(..., frame_id=...).
A missing method raises AttributeError inside a ROS callback and shuts the
node down, so these tests pin the API.
"""

import os
import sys

import pytest

PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from rl_nav_controller.path_tracker import PathSegmentTracker, BlockedDetector  # noqa: E402


def test_empty_tracker_api():
    tracker = PathSegmentTracker()
    assert tracker.has_path() is False
    assert tracker.is_active() is False
    assert tracker.get_terminal_pose() == (None, None)
    assert tracker.realign_to_robot([0.0, 0.0]) == 0.0


def test_set_path_and_has_path():
    tracker = PathSegmentTracker()
    ok = tracker.set_path(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        terminal_yaw=0.5,
        frame_id='world',
    )
    assert ok is True
    assert tracker.has_path() is True
    assert tracker.frame_id == 'world'
    terminal_pt, terminal_yaw = tracker.get_terminal_pose()
    assert terminal_pt == pytest.approx([2.0, 0.0, 0.0])
    assert terminal_yaw == pytest.approx(0.5)
    assert tracker.total_length == pytest.approx(2.0)


def test_realign_to_robot_updates_progress():
    tracker = PathSegmentTracker()
    tracker.set_path([[0.0, 0.0, 0.0], [4.0, 0.0, 0.0]], frame_id='map')
    progress = tracker.realign_to_robot([3.0, 0.5])
    assert progress == pytest.approx(3.0, abs=1e-6)
    assert tracker.current_progress == pytest.approx(3.0, abs=1e-6)


def test_clear_resets_path_state():
    tracker = PathSegmentTracker()
    tracker.set_path([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], frame_id='world')
    tracker.clear()
    assert tracker.has_path() is False
    assert tracker.frame_id == 'map'
    assert tracker.get_terminal_pose() == (None, None)


def test_blocked_detector_still_works():
    detector = BlockedDetector(timeout=1.0)
    detector.reset(current_time=0.0, progress_s=0.0)
    assert detector.update(0.5, 0.0, [0.0, 0.0]) is False
    assert detector.update(1.5, 0.0, [0.0, 0.0]) is True

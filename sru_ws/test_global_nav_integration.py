#!/usr/bin/env python3
"""Comprehensive test suite for Global Navigation Integration & Path Tracking."""

import sys
import os
import math
import numpy as np

# Add rl_nav_controller to sys.path
SRU_WS_PATH = "/home/yichangfeng/go2w_robot_ref/sru_ws/src/sru-robot-deployment/rl_nav_controller"
if SRU_WS_PATH not in sys.path:
    sys.path.insert(0, SRU_WS_PATH)

from rl_nav_controller.path_tracker import NavState, PathSegmentTracker, BlockedDetector


def test_nav_state_enum():
    """Verify NavState enumeration strictly matches GLOBAL_NAV_INTEGRATION_SPEC.md."""
    print("[TEST] 1. NavState Enumeration Alignment...")
    expected_states = {"idle", "running", "arrived", "paused", "blocked", "stop"}
    actual_states = {s.value for s in NavState}
    assert actual_states == expected_states, f"NavState mismatch! Expected {expected_states}, got {actual_states}"
    assert NavState.IDLE.value == "idle"
    assert NavState.RUNNING.value == "running"
    assert NavState.ARRIVED.value == "arrived"
    assert NavState.PAUSED.value == "paused"
    assert NavState.BLOCKED.value == "blocked"
    assert NavState.STOP.value == "stop"
    print("  -> Passed: NavState enumeration 100% aligned with specification.")


def test_path_tracker_straight_line():
    """Verify PathSegmentTracker with a straight-line segmented path (1m point spacing)."""
    print("[TEST] 2. PathSegmentTracker Straight Line & Lookahead...")
    tracker = PathSegmentTracker()

    # Path: (0,0) -> (1,0) -> (2,0) -> (3,0) -> (4,0)
    pts = [[0.0, 0.0, 0.2], [1.0, 0.0, 0.2], [2.0, 0.0, 0.2], [3.0, 0.0, 0.2], [4.0, 0.0, 0.2]]
    success = tracker.set_path(pts, terminal_yaw=0.0)
    assert success is True
    assert tracker.is_active() is True
    assert math.isclose(tracker.total_length, 4.0, abs_tol=1e-5)

    # 1. At start (0, 0), lookahead L=2.0m -> carrot at (2.0, 0.0)
    progress = tracker.update_progress([0.0, 0.0])
    assert math.isclose(progress, 0.0, abs_tol=1e-5)
    carrot_pt, carrot_yaw = tracker.get_lookahead_point(lookahead_dist=2.0)
    assert math.isclose(carrot_pt[0], 2.0, abs_tol=1e-4)
    assert math.isclose(carrot_pt[1], 0.0, abs_tol=1e-4)
    assert math.isclose(carrot_yaw, 0.0, abs_tol=1e-4)

    # 2. Robot moved to (1.5, 0.1) -> progress ~ 1.5, lookahead L=2.0m -> carrot at (3.5, 0.0)
    progress = tracker.update_progress([1.5, 0.1])
    assert math.isclose(progress, 1.5, abs_tol=1e-4)
    carrot_pt, carrot_yaw = tracker.get_lookahead_point(lookahead_dist=2.0)
    assert math.isclose(carrot_pt[0], 3.5, abs_tol=1e-4)
    assert math.isclose(carrot_pt[1], 0.0, abs_tol=1e-4)

    # 3. Robot moved to (2.5, 0.0) -> lookahead 2.5 + 2.0 = 4.5 >= 4.0 -> carrot clamped to terminal (4.0, 0.0)
    progress = tracker.update_progress([2.5, 0.0])
    assert math.isclose(progress, 2.5, abs_tol=1e-4)
    carrot_pt, carrot_yaw = tracker.get_lookahead_point(lookahead_dist=2.0)
    assert math.isclose(carrot_pt[0], 4.0, abs_tol=1e-4)
    assert math.isclose(carrot_pt[1], 0.0, abs_tol=1e-4)
    assert not tracker.check_arrived([2.5, 0.0], arrive_threshold=0.15)

    # 4. Robot approaching terminal goal
    # At (3.80, 0.0), distance to (4.0, 0.0) is 0.20m > 0.15m -> not arrived
    assert not tracker.check_arrived([3.80, 0.0], arrive_threshold=0.15)
    # At (3.88, 0.0), distance to (4.0, 0.0) is 0.12m <= 0.15m -> ARRIVED!
    assert tracker.check_arrived([3.88, 0.0], arrive_threshold=0.15)

    print("  -> Passed: Lookahead L=2.0m, polyline progression, clamping, and arrival check verified.")


def test_path_tracker_corner_path():
    """Verify PathSegmentTracker with an L-shaped corner path."""
    print("[TEST] 3. PathSegmentTracker Corner / Polyline Interpolation...")
    tracker = PathSegmentTracker()

    # Path: (0,0) -> (2,0) -> (2, 2)
    pts = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]]
    tracker.set_path(pts)
    assert math.isclose(tracker.total_length, 4.0, abs_tol=1e-5)

    # Robot at (1.0, 0.0), progress=1.0. Lookahead 2.0m -> target arc = 3.0m
    # 3.0m is on second segment: x=2.0, y=1.0
    tracker.update_progress([1.0, 0.0])
    carrot_pt, carrot_yaw = tracker.get_lookahead_point(lookahead_dist=2.0)
    assert math.isclose(carrot_pt[0], 2.0, abs_tol=1e-4)
    assert math.isclose(carrot_pt[1], 1.0, abs_tol=1e-4)
    # Tangent on second segment is +y (pi/2)
    assert math.isclose(carrot_yaw, math.pi / 2, abs_tol=1e-4)
    print("  -> Passed: Multi-segment corner interpolation and tangent yaw computation verified.")


def test_blocked_detector():
    """Verify BlockedDetector accurately triggers BLOCKED after continuous 4.0s stall."""
    print("[TEST] 4. BlockedDetector Stall Timeout...")
    detector = BlockedDetector(timeout=4.0, dist_threshold=0.1, vel_threshold=0.1)

    t0 = 100.0
    progress = 1.0
    detector.reset(current_time=t0, progress_s=progress)

    # 1. Normal progress: robot moves 0.15m every 0.5s -> never blocked
    current_time = t0
    for i in range(10):
        current_time += 0.5
        progress += 0.15
        is_blocked = detector.update(current_time, progress, robot_vel_xy=[0.3, 0.0])
        assert is_blocked is False, f"Unexpected blocked at step {i}"

    # 2. Obstacle blocks robot: progress and velocity drop to ~0
    stall_start_time = current_time
    stall_progress = progress

    # At 1.0s of stall -> not blocked
    is_blocked = detector.update(stall_start_time + 1.0, stall_progress + 0.02, robot_vel_xy=[0.02, 0.01])
    assert is_blocked is False

    # At 3.9s of stall (< 4.0s) -> not blocked
    is_blocked = detector.update(stall_start_time + 3.9, stall_progress + 0.05, robot_vel_xy=[0.01, 0.0])
    assert is_blocked is False

    # At 4.05s of stall (>= 4.0s) -> BLOCKED!
    is_blocked = detector.update(stall_start_time + 4.05, stall_progress + 0.05, robot_vel_xy=[0.0, 0.0])
    assert is_blocked is True, "Expected BLOCKED state to trigger after 4.0s timeout!"

    # 3. New path arrives -> reset detector
    detector.reset(current_time=stall_start_time + 5.0, progress_s=0.0)
    is_blocked = detector.update(stall_start_time + 5.1, 0.0, robot_vel_xy=[0.0, 0.0])
    assert is_blocked is False, "Expected reset to clear blocked state."

    print("  -> Passed: Stalled 4.0s threshold precisely triggers BLOCKED, reset verified.")


def test_edge_cases():
    """Verify edge case handling: empty path, single point path."""
    print("[TEST] 5. Edge Cases (Empty path, single point)...")
    tracker = PathSegmentTracker()

    assert tracker.set_path([]) is False
    assert tracker.is_active() is False

    # Single point path
    tracker.set_path([[5.0, 2.0, 0.3]], terminal_yaw=1.57)
    assert tracker.is_active() is True
    assert tracker.total_length == 0.0
    carrot_pt, carrot_yaw = tracker.get_lookahead_point(lookahead_dist=2.0)
    assert carrot_pt == [5.0, 2.0, 0.3]
    assert math.isclose(carrot_yaw, 1.57, abs_tol=1e-4)
    assert tracker.check_arrived([5.05, 2.0], arrive_threshold=0.15) is True

    print("  -> Passed: Edge cases handled safely.")


if __name__ == "__main__":
    print("==================================================================")
    print("       TESTING GLOBAL NAVIGATION INTEGRATION MODULES")
    print("==================================================================")
    test_nav_state_enum()
    test_path_tracker_straight_line()
    test_path_tracker_corner_path()
    test_blocked_detector()
    test_edge_cases()
    print("==================================================================")
    print("       ALL 5 INTEGRATION TEST SUITES PASSED (100% OK)")
    print("==================================================================")

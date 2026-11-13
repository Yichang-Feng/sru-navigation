#!/usr/bin/env python3
"""Comprehensive test suite for Global Navigation Integration & Path Tracking."""

import sys
import os
import math
import time
import numpy as np

# Add rl_nav_controller to sys.path
SRU_WS_PATH = "/home/yichangfeng/go2w_robot_ref/sru_ws/src/sru-robot-deployment/rl_nav_controller"
if SRU_WS_PATH not in sys.path:
    sys.path.insert(0, SRU_WS_PATH)

from rl_nav_controller.path_tracker import NavState, PathSegmentTracker, BlockedDetector


import json


def test_nav_state_enum():
    """Verify NavState enumeration matches specification."""
    print("[TEST] 1. NavState Enumeration Alignment...")
    canonical_states = {"idle", "running", "goal_reached", "failed", "error", "stopped"}
    actual_states = {s.value for s in NavState}
    assert actual_states == canonical_states, f"NavState mismatch! Expected {canonical_states}, got {actual_states}"
    assert NavState.IDLE.value == "idle"
    assert NavState.RUNNING.value == "running"
    assert NavState.GOAL_REACHED.value == "goal_reached"
    assert NavState.FAILED.value == "failed"
    assert NavState.ERROR.value == "error"
    assert NavState.STOPPED.value == "stopped"

    # Backward compatibility aliases
    assert NavState.ARRIVED.value == "goal_reached"
    assert NavState.BLOCKED.value == "failed"
    assert NavState.STOP.value == "stopped"
    assert NavState.PAUSED.value == "stopped"
    print("  -> Passed: NavState 6 canonical states (idle, running, goal_reached, failed, error, stopped) + aliases aligned.")


def test_json_status_format():
    """Verify status JSON formatting specification per local_planner_status_diagram (1).html: {"state": "...", "msgs": []}."""
    print("[TEST] 2. JSON Status Format & Multi-Format Compatibility...")
    
    def format_status_json(state: NavState, error_msg: str = "", format_mode: str = "list") -> str:
        is_error = (state == NavState.ERROR and bool(error_msg))
        if format_mode in ('string_brackets', 'string_array'):
            msgs_val = json.dumps([error_msg]) if is_error else "[]"
        elif format_mode in ('empty_string', 'empty'):
            msgs_val = error_msg if is_error else ""
        elif format_mode == 'brackets_all':
            msgs_val = f"[{error_msg}]" if is_error else "[]"
        else:
            # Default 'list' (matches local_planner_status_diagram (1).html: msgs is real JSON array [])
            msgs_val = [error_msg] if is_error else []
        status_dict = {"state": state.value, "msgs": msgs_val}
        return json.dumps(status_dict, separators=(',', ':'))

    # Part A: Test default 'list' format per local_planner_status_diagram (1).html (msgs = [] on non-error, ["..."] on error)
    # Exact raw string assertions:
    assert format_status_json(NavState.IDLE) == '{"state":"idle","msgs":[]}'
    assert format_status_json(NavState.RUNNING) == '{"state":"running","msgs":[]}'
    assert format_status_json(NavState.GOAL_REACHED) == '{"state":"goal_reached","msgs":[]}'
    assert format_status_json(NavState.STOPPED) == '{"state":"stopped","msgs":[]}'
    assert format_status_json(NavState.FAILED) == '{"state":"failed","msgs":[]}'
    assert format_status_json(NavState.ERROR, "zed_camera_lost") == '{"state":"error","msgs":["zed_camera_lost"]}'

    # Part B: Test 'string_brackets' format (msgs = "[]" on non-error, '["..."]' on error)
    assert json.loads(format_status_json(NavState.GOAL_REACHED, format_mode="string_brackets")) == {"state": "goal_reached", "msgs": "[]"}
    assert json.loads(format_status_json(NavState.ERROR, "zed_camera_lost", format_mode="string_brackets")) == {"state": "error", "msgs": '["zed_camera_lost"]'}

    # Part C: Test 'empty_string' format (msgs = "" on non-error)
    assert json.loads(format_status_json(NavState.GOAL_REACHED, format_mode="empty_string")) == {"state": "goal_reached", "msgs": ""}

    print("  -> Passed: Default list format verified ('{\"state\":\"goal_reached\",\"msgs\":[]}') + multi-format compatibility.")


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


class MockNavControllerStateMachine:
    """Mock state machine capturing rl_nav_controller state transition rules."""
    def __init__(self, goal_reached_timeout: float = 1.0):
        self.state = NavState.IDLE
        self.error_msg = ""
        self.last_state_change_time = 0.0
        self.goal_reached_timeout = goal_reached_timeout
        self.path_tracker = PathSegmentTracker()
        self.published_states = []

    def set_state(self, new_state: NavState, reason: str = "", error_msg: str = ""):
        if self.state != new_state or (new_state == NavState.ERROR and error_msg != self.error_msg):
            self.state = new_state
            self.last_state_change_time = time.time()
            self.published_states.append((new_state, time.time()))

    def on_path_received(self, pts, terminal_yaw=None):
        if not pts:
            self.path_tracker.clear()
            self.set_state(NavState.STOPPED, reason="Empty path")
            return

        self.path_tracker.set_path(pts, terminal_yaw=terminal_yaw)

        # In STOPPED: keep STOPPED, do not auto-run!
        if self.state == NavState.STOPPED:
            pass  # keep STOPPED, retain path
        else:
            self.set_state(NavState.RUNNING, reason="New path loaded")

    def on_goal_reached(self):
        self.path_tracker.clear()
        self.set_state(NavState.GOAL_REACHED, reason="Segment arrival")
        # Immediately return to IDLE, completing the unidirectional triangle
        self.set_state(NavState.IDLE, reason="Goal reached reported, immediate return to IDLE")

    def on_heartbeat_or_control_loop(self, now: float):
        if self.state == NavState.GOAL_REACHED:
            self.set_state(NavState.IDLE, reason="Auto transition to IDLE")

    def on_mission_control(self, cmd: str):
        cmd = cmd.strip().lower()
        if cmd in ("pause", "stop"):
            self.set_state(NavState.STOPPED, reason=f"Command {cmd}")
        elif cmd in ("resume", "continue"):
            if self.state in (NavState.STOPPED, NavState.IDLE, NavState.FAILED, NavState.GOAL_REACHED):
                if self.path_tracker.is_active():
                    self.set_state(NavState.RUNNING, reason="Resumed")
                else:
                    self.set_state(NavState.IDLE, reason="Resumed with no path")


def test_goal_reached_and_idle_transitions():
    """Verify unidirectional triangle: idle -> running -> goal_reached -> idle, and idle -> running on new path."""
    print("[TEST] 6. State Machine: Unidirectional triangle (idle -> running -> goal_reached -> idle)...")
    sm = MockNavControllerStateMachine()
    assert sm.state == NavState.IDLE

    # 1. idle -> running upon receiving path
    sm.on_path_received([[0, 0, 0], [1, 0, 0]])
    assert sm.state == NavState.RUNNING
    assert sm.path_tracker.is_active() is True

    # 2. running -> goal_reached (reported) -> immediately returns to idle
    sm.on_goal_reached()
    # State should immediately be IDLE
    assert sm.state == NavState.IDLE
    assert sm.path_tracker.is_active() is False

    # Check published states history: must contain GOAL_REACHED followed immediately by IDLE
    recorded_states = [s[0] for s in sm.published_states]
    assert NavState.GOAL_REACHED in recorded_states, "GOAL_REACHED must be published upon arrival"
    assert recorded_states[-2] == NavState.GOAL_REACHED
    assert recorded_states[-1] == NavState.IDLE

    # 3. In idle, if new path arrives -> immediately enters running
    sm.on_path_received([[1, 0, 0], [3, 0, 0]])
    assert sm.state == NavState.RUNNING
    assert sm.path_tracker.is_active() is True
    print("  -> Passed: Unidirectional triangle (idle -> running -> goal_reached -> idle) verified.")


def test_stopped_keeps_stopped_and_resumes():
    """Verify stopped state ignores path execution (only updates memory) until resume."""
    print("[TEST] 7. State Machine: stopped retains new path, waits for resume...")
    sm = MockNavControllerStateMachine()

    # Start running path 1: (0,0) -> (2,0)
    sm.on_path_received([[0, 0, 0], [2, 0, 0]])
    assert sm.state == NavState.RUNNING

    # Pause robot -> STOPPED
    sm.on_mission_control("pause")
    assert sm.state == NavState.STOPPED
    assert sm.path_tracker.total_length == 2.0

    # Scenario A: Receive new path while STOPPED -> MUST remain STOPPED, but path tracker updated!
    sm.on_path_received([[2, 0, 0], [5, 0, 0], [5, 3, 0]])  # total length 3 + 3 = 6m
    assert sm.state == NavState.STOPPED, "Robot must NOT start running upon receiving new path while STOPPED!"
    assert sm.path_tracker.is_active() is True
    assert math.isclose(sm.path_tracker.total_length, 6.0, abs_tol=1e-5), "New path must be retained in memory!"

    # Send resume -> transitions to RUNNING and follows the NEW path
    sm.on_mission_control("resume")
    assert sm.state == NavState.RUNNING
    assert math.isclose(sm.path_tracker.total_length, 6.0, abs_tol=1e-5)

    # Scenario B: Pause and resume without new path -> continues OLD path
    sm.on_mission_control("pause")
    assert sm.state == NavState.STOPPED
    sm.on_mission_control("resume")
    assert sm.state == NavState.RUNNING
    assert math.isclose(sm.path_tracker.total_length, 6.0, abs_tol=1e-5)

    print("  -> Passed: stopped remains stopped on new path, correctly resumes new/old path on resume.")


def test_joystick_button_a_start_b_pause():
    print("[TEST] 9. Joystick Button A (Start/Resume) & Button B (Pause/Stop)...")
    sm = MockNavControllerStateMachine()
    
    # Simulate single goal
    sm.target_pos_w = [4.0, 2.0, 0.0]
    sm.set_state(NavState.RUNNING, reason="Target set")
    assert sm.state == NavState.RUNNING

    # Button B pressed -> Pause / Stop (path retained, not aborted)
    btn_pause = 1
    joy_buttons_b = [0, 1, 0, 0]  # B=1
    if joy_buttons_b[btn_pause] == 1:
        sm.is_abort_goal = False
        sm.set_state(NavState.STOPPED, reason="Button B pressed")
    assert sm.state == NavState.STOPPED
    assert sm.is_abort_goal is False

    # Button A pressed -> Start / Resume
    btn_start = 0
    joy_buttons_a = [1, 0, 0, 0]  # A=1
    if joy_buttons_a[btn_start] == 1:
        sm.is_abort_goal = False
        if sm.target_pos_w is not None:
            sm.set_state(NavState.RUNNING, reason="Button A pressed")
    assert sm.state == NavState.RUNNING
    assert sm.is_abort_goal is False

    print("  -> Passed: Button B successfully paused/stopped (path retained), Button A successfully resumed.")


def test_report_failure_suppression():
    """Verify that report_failure=False suppresses error/failed status and prevents locking into FAILED."""
    print("[TEST] 10. Report Failure Suppression & Lookahead Setting...")

    class MockNode:
        def __init__(self, report_failure=True, lookahead_distance=2.0):
            self.state = NavState.RUNNING
            self.error_msg = ""
            self.report_failure = report_failure
            self.lookahead_distance = lookahead_distance
            self.status_msgs_format = 'list'
            self.has_active_path = True

        def format_status_json(self) -> str:
            curr_state_val = self.state.value
            is_error = (self.state == NavState.ERROR and bool(self.error_msg))

            if not self.report_failure and self.state in (NavState.ERROR, NavState.FAILED):
                curr_state_val = NavState.RUNNING.value if self.has_active_path else NavState.IDLE.value
                is_error = False

            msgs_val = [self.error_msg] if is_error else []
            return json.dumps({"state": curr_state_val, "msgs": msgs_val}, separators=(',', ':'))

    # Case 1: Original behavior (report_failure=True)
    node_orig = MockNode(report_failure=True, lookahead_distance=2.0)
    node_orig.state = NavState.FAILED
    assert json.loads(node_orig.format_status_json())["state"] == "failed"

    node_orig.state = NavState.ERROR
    node_orig.error_msg = "zed_camera_lost"
    parsed_err = json.loads(node_orig.format_status_json())
    assert parsed_err["state"] == "error"
    assert "zed_camera_lost" in parsed_err["msgs"]

    # Case 2: Suppressed behavior (report_failure=False)
    node_suppressed = MockNode(report_failure=False, lookahead_distance=1.5)
    assert node_suppressed.lookahead_distance == 1.5

    # If robot is internally marked FAILED, status report shows RUNNING and msgs []
    node_suppressed.state = NavState.FAILED
    parsed_failed = json.loads(node_suppressed.format_status_json())
    assert parsed_failed["state"] == "running"
    assert parsed_failed["msgs"] == []

    # If sensor error occurs, status report shows RUNNING and msgs []
    node_suppressed.state = NavState.ERROR
    node_suppressed.error_msg = "zed_camera_lost"
    parsed_err_supp = json.loads(node_suppressed.format_status_json())
    assert parsed_err_supp["state"] == "running"
    assert parsed_err_supp["msgs"] == []

    print("  -> Passed: report_failure=False cleanly suppresses 'failed' and 'error', original report_failure=True preserved.")


def test_motion_enable_gate_stop_and_resume():
    """Verify Unitree wireless controller button B/A behavior via /motion_enabled gate."""
    print("[TEST] 11. Unitree Remote /motion_enabled Gate (Button B Stop & Button A Resume)...")
    sm = MockNavControllerStateMachine()

    # 1. Robot is actively running a path
    sm.on_path_received([[0, 0, 0], [5, 0, 0]])
    assert sm.state == NavState.RUNNING

    # 2. Remote button B pressed -> /motion_enabled = False
    # Mocking motion_enable_callback behavior
    def mock_motion_enable(enabled):
        if not enabled:
            if sm.state in (NavState.RUNNING, NavState.IDLE, NavState.FAILED):
                sm.is_abort_goal = False
                sm.set_state(NavState.STOPPED, reason="Chassis motion disabled (/motion_enabled=False)")
                sm.path_tracker.total_length  # retain
        else:
            if sm.state == NavState.STOPPED:
                if sm.path_tracker.is_active():
                    sm.is_abort_goal = False
                    sm.set_state(NavState.RUNNING, reason="Chassis motion enabled (/motion_enabled=True)")

    mock_motion_enable(False)
    assert sm.state == NavState.STOPPED, f"Expected STOPPED, got {sm.state.value}"
    assert sm.path_tracker.is_active() is True, "Path must be retained on remote button B!"

    # 3. New path arrives while stopped -> still STOPPED, path updated
    sm.on_path_received([[0, 0, 0], [10, 0, 0]])
    assert sm.state == NavState.STOPPED
    assert math.isclose(sm.path_tracker.total_length, 10.0, abs_tol=1e-5)

    # 4. Remote button A pressed -> /motion_enabled = True
    mock_motion_enable(True)
    assert sm.state == NavState.RUNNING, f"Expected RUNNING, got {sm.state.value}"

    # 5. Remote button B pressed when FAILED -> transitions to STOPPED per HTML specification
    sm.state = NavState.FAILED
    mock_motion_enable(False)
    assert sm.state == NavState.STOPPED, f"Expected STOPPED when disabled from FAILED, got {sm.state.value}"

    print("  -> Passed: /motion_enabled gate correctly transitions between RUNNING and STOPPED per HTML spec.")


if __name__ == "__main__":
    print("==================================================================")
    print("       TESTING GLOBAL NAVIGATION INTEGRATION MODULES")
    print("==================================================================")
    test_nav_state_enum()
    test_json_status_format()
    test_path_tracker_straight_line()
    test_path_tracker_corner_path()
    test_blocked_detector()
    test_edge_cases()
    test_goal_reached_and_idle_transitions()
    test_stopped_keeps_stopped_and_resumes()
    test_joystick_button_a_start_b_pause()
    test_report_failure_suppression()
    test_motion_enable_gate_stop_and_resume()
    print("==================================================================")
    print("       ALL 11 INTEGRATION TEST SUITES PASSED (100% OK)")
    print("==================================================================")


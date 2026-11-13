#!/usr/bin/env python3
"""Unit tests verifying single goal arrival threshold alignment and Button B/A behavior.

Ensures that:
1. Single goal arrival threshold is aligned to 0.20m (consistent with segment_arrive_threshold).
2. Robot does not stop prematurely at 0.75m or 0.25m, but arrives when distance <= 0.20m.
3. In single goal tracking mode, receiving /motion_enabled=False (Button B) enters NavState.STOPPED,
   brakes the robot, and retains the single goal target in memory.
4. Receiving /motion_enabled=True (Button A) while in STOPPED resumes NavState.RUNNING towards the target.
"""

import sys
import os
import numpy as np
import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from adaptive_path_tracker import NavState, BlockedDetector
import scripts.constants as constants


class MockSingleGoalController:
    """Mock controller reproducing single-goal tracking, state transitions, and arrival checks."""

    def __init__(self, arrive_threshold=None, blocked_timeout=4.0):
        self.arrive_goal_threshold = (
            arrive_threshold if arrive_threshold is not None else getattr(constants, 'ARRIVE_GOAL_THRESHOLD', 0.20)
        )
        self.segment_arrive_threshold = getattr(constants, 'SEGMENT_ARRIVE_THRESHOLD', 0.20)
        self.state = NavState.IDLE
        self.target_pos_w = None
        self.last_target_pos = None
        self.target_yaw_w = None
        self.robot_pos_w = np.array([0.0, 0.0, 0.0])
        self.robot_orientation_w = [1.0, 0.0, 0.0, 0.0]  # [w, x, y, z]
        self.linear_vel = np.array([0.0, 0.0, 0.0])
        self.robot_odom_time = 0.0
        self.is_abort_goal = False
        self.goal_reached_latched = False
        self.robot_braked = False
        self.blocked_detector = BlockedDetector(timeout=blocked_timeout, dist_threshold=0.1, vel_threshold=0.1)

    def _stop_robot(self):
        self.robot_braked = True
        self.linear_vel = np.array([0.0, 0.0, 0.0])

    def set_state(self, new_state: NavState, reason: str = ""):
        self.state = new_state

    def send_goal(self, target_xyz):
        """Simulate receiving a 2D Nav Goal in RViz."""
        self.target_pos_w = list(target_xyz)
        self.last_target_pos = list(target_xyz)
        self.blocked_detector.reset(self.robot_odom_time, progress_s=0.0)
        self.set_state(NavState.RUNNING, reason="Single goal received")

    def _check_goal_reached(self, target_pos_w, robot_pos_w):
        """Replicate rl_nav_controller._check_goal_reached()."""
        if target_pos_w is None or robot_pos_w is None:
            return False
        dist = np.linalg.norm(np.array(target_pos_w[:2]) - np.array(robot_pos_w[:2]))
        return dist <= self.arrive_goal_threshold

    def on_motion_enable(self, enabled: bool):
        """Replicate rl_nav_controller.motion_enable_callback()."""
        if not enabled:
            if self.state in (NavState.RUNNING, NavState.IDLE, NavState.FAILED):
                self.is_abort_goal = False
                self.set_state(NavState.STOPPED, reason="Chassis motion disabled by remote B button")
                self._stop_robot()
        else:
            if self.state == NavState.STOPPED:
                if self.target_pos_w is not None or self.last_target_pos is not None:
                    if self.target_pos_w is None and self.last_target_pos is not None:
                        self.target_pos_w = list(self.last_target_pos)
                    self.is_abort_goal = False
                    self.goal_reached_latched = False
                    self.blocked_detector.reset(self.robot_odom_time, progress_s=0.0)
                    self.set_state(NavState.RUNNING, reason="Chassis motion resumed by remote A button")
                else:
                    self.set_state(NavState.IDLE, reason="Motion enabled with no active path or goal")

    def step(self, robot_xyz):
        """Simulate one cycle of control execution."""
        self.robot_pos_w = np.array(robot_xyz)
        self.robot_odom_time += 0.1

        if self.state == NavState.STOPPED:
            self._stop_robot()
            return

        if self.state == NavState.RUNNING and self.target_pos_w is not None:
            if self._check_goal_reached(self.target_pos_w, self.robot_pos_w):
                self._stop_robot()
                self.target_pos_w = None
                self.set_state(NavState.GOAL_REACHED, reason="Single goal reached")
                self.set_state(NavState.IDLE, reason="Goal reached transition to IDLE")
            else:
                # Simulating active motion towards goal
                dir_vec = np.array(self.target_pos_w[:2]) - self.robot_pos_w[:2]
                norm = np.linalg.norm(dir_vec)
                if norm > 1e-4:
                    self.linear_vel[:2] = (dir_vec / norm) * 0.5
                self.robot_braked = False


def test_constants_arrival_threshold_alignment():
    """Verify that ARRIVE_GOAL_THRESHOLD is aligned with SEGMENT_ARRIVE_THRESHOLD at 0.20m."""
    assert constants.ARRIVE_GOAL_THRESHOLD == 0.20, f"Expected 0.20, got {constants.ARRIVE_GOAL_THRESHOLD}"
    assert constants.SEGMENT_ARRIVE_THRESHOLD == 0.20, f"Expected 0.20, got {constants.SEGMENT_ARRIVE_THRESHOLD}"
    assert constants.ARRIVE_GOAL_THRESHOLD == constants.SEGMENT_ARRIVE_THRESHOLD


def test_single_goal_arrival_distance_precision():
    """Verify arrival check triggers precisely within 0.20m, and does not stop at 0.25m or 0.75m."""
    ctrl = MockSingleGoalController()
    target = [2.0, 0.0, 0.0]
    ctrl.send_goal(target)

    # 1. Robot at 0.70m from target: previously old 0.75m threshold would have arrived prematurely
    ctrl.step([1.30, 0.0, 0.0])  # dist = 0.70m
    assert ctrl.state == NavState.RUNNING, "Robot should still be RUNNING at 0.70m from goal"
    assert not ctrl.robot_braked

    # 2. Robot at 0.25m from target
    ctrl.step([1.75, 0.0, 0.0])  # dist = 0.25m
    assert ctrl.state == NavState.RUNNING, "Robot should still be RUNNING at 0.25m from goal"
    assert not ctrl.robot_braked

    # 3. Robot reaches 0.18m from target (within 0.20m)
    ctrl.step([1.82, 0.0, 0.0])  # dist = 0.18m <= 0.20m
    assert ctrl.robot_braked
    assert ctrl.target_pos_w is None
    assert ctrl.state == NavState.IDLE


def test_single_goal_b_stop_and_a_resume():
    """Verify Button B stops single goal motion, and Button A resumes toward the same goal."""
    ctrl = MockSingleGoalController()
    target = [5.0, 0.0, 0.0]
    ctrl.send_goal(target)

    # Step forward towards goal
    ctrl.step([1.0, 0.0, 0.0])
    assert ctrl.state == NavState.RUNNING
    assert not ctrl.robot_braked

    # User presses Button B -> /motion_enabled = False
    ctrl.on_motion_enable(False)
    assert ctrl.state == NavState.STOPPED
    assert ctrl.target_pos_w == target, "Target goal position must be preserved in memory"

    # In STOPPED state, robot must brake
    ctrl.step([1.0, 0.0, 0.0])
    assert ctrl.robot_braked
    assert ctrl.state == NavState.STOPPED

    # Extended pause (simulate 5 seconds while in STOPPED)
    for _ in range(50):
        ctrl.step([1.0, 0.0, 0.0])
    assert ctrl.state == NavState.STOPPED
    assert ctrl.target_pos_w == target

    # User presses Button A -> /motion_enabled = True
    ctrl.on_motion_enable(True)
    assert ctrl.state == NavState.RUNNING, "Button A must resume state to RUNNING"
    assert ctrl.target_pos_w == target

    # Robot continues moving and arrives at target
    ctrl.step([3.0, 0.0, 0.0])
    assert ctrl.state == NavState.RUNNING
    assert not ctrl.robot_braked

    # Reach 0.19m from target
    ctrl.step([4.81, 0.0, 0.0])
    assert ctrl.robot_braked
    assert ctrl.state == NavState.IDLE

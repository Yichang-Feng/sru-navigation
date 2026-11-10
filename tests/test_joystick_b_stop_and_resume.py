#!/usr/bin/env python3
"""Unit tests verifying Joystick Button B (Pause/Stop) and Button A (Start/Resume) behavior.

Ensures that:
1. Pressing Button B enters NavState.STOPPED, brakes the robot, and retains the path.
2. An extended pause duration (e.g. >10 seconds, exceeding blocked_timeout=4.0s) does NOT
   cause the controller to trip into NavState.FAILED / false.
3. Pressing Button A resumes navigation (NavState.RUNNING) without false stall triggers.
4. Genuine stalls while RUNNING (progress < 0.1m for >= 4.0s) correctly transition to
   NavState.FAILED per local_planner_status_diagram.html.
"""

import sys
import os
import time
import numpy as np

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from adaptive_path_tracker import NavState, AdaptivePathSegmentTracker, BlockedDetector


class MockRobotController:
    """Mock controller reproducing state machine and blocked detector logic."""

    def __init__(self, blocked_timeout=4.0, report_failure=True):
        self.state = NavState.IDLE
        self.error_msg = ""
        self.report_failure = report_failure
        self.path_tracker = AdaptivePathSegmentTracker()
        self.blocked_detector = BlockedDetector(timeout=blocked_timeout, dist_threshold=0.1, vel_threshold=0.1)
        self.robot_odom_time = 0.0
        self.robot_pos_w = np.array([0.0, 0.0, 0.0])
        self.linear_vel = np.array([0.0, 0.0, 0.0])
        self.is_abort_goal = False
        self.last_trigger_time = 0.0
        self.robot_braked = False

    def _stop_robot(self):
        self.robot_braked = True
        self.linear_vel = np.array([0.0, 0.0, 0.0])

    def set_state(self, new_state: NavState, reason: str = "", error_msg: str = ""):
        self.state = new_state
        if new_state == NavState.ERROR:
            self.error_msg = error_msg
        else:
            self.error_msg = ""

    def load_path(self, points):
        self.path_tracker.set_path(points)
        self.blocked_detector.reset(self.robot_odom_time, progress_s=0.0)
        self.set_state(NavState.RUNNING, reason="New path loaded")

    def press_button_b(self, current_time):
        """Simulate Joystick Button B press."""
        self.robot_odom_time = current_time
        self.is_abort_goal = False
        self.set_state(NavState.STOPPED, reason="Button B pressed")
        self._stop_robot()
        if self.path_tracker.has_path():
            self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)

    def press_button_a(self, current_time):
        """Simulate Joystick Button A press."""
        self.robot_odom_time = current_time
        self.is_abort_goal = False
        if self.path_tracker.has_path():
            self.path_tracker.realign_to_robot(self.robot_pos_w[:2])
            self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)
            self.set_state(NavState.RUNNING, reason="Button A pressed")

    def step_control_loop(self, dt=0.1):
        """Simulate one execution of generate_cmd_vel()."""
        self.robot_odom_time += dt

        # 1) Check blocked / stopped / error states
        if self.state in (NavState.ERROR, NavState.FAILED, NavState.STOPPED):
            self._stop_robot()
            if self.state == NavState.STOPPED and self.path_tracker.has_path():
                self.blocked_detector.reset(self.robot_odom_time, self.path_tracker.current_progress)
            return

        if self.is_abort_goal:
            self._stop_robot()
            self.path_tracker.clear()
            self.is_abort_goal = False
            self.set_state(NavState.STOPPED, reason="Goal aborted")
            return

        if self.path_tracker.is_active():
            progress_s = self.path_tracker.update_progress(self.robot_pos_w[:2])

            # Blocked / stall check
            robot_vel_xy = self.linear_vel[:2]
            if self.blocked_detector.update(self.robot_odom_time, progress_s, robot_vel_xy):
                if self.report_failure:
                    self._stop_robot()
                    self.set_state(
                        NavState.FAILED,
                        reason=f"Stalled: timeout reached"
                    )
                    return


def test_button_b_pause_and_resume():
    print("[TEST] 1. Joystick Button B Pause (Extended Duration) & Button A Resume...")
    controller = MockRobotController(blocked_timeout=4.0, report_failure=True)

    # 1. Load a straight line path
    pts = [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
    controller.load_path(pts)
    assert controller.state == NavState.RUNNING
    assert controller.path_tracker.has_path()

    # Move forward for 2.0s
    for _ in range(20):
        controller.robot_pos_w[0] += 0.05
        controller.linear_vel[0] = 0.5
        controller.step_control_loop(dt=0.1)
    assert controller.state == NavState.RUNNING
    assert controller.robot_pos_w[0] > 0.9

    # 2. User presses Joystick Button B (Pause/Stop)
    t_pause = controller.robot_odom_time
    controller.press_button_b(current_time=t_pause)
    assert controller.state == NavState.STOPPED
    assert controller.robot_braked is True
    assert controller.path_tracker.has_path() is True, "Path MUST be retained on Button B press!"
    assert controller.is_abort_goal is False

    # 3. Robot sits stationary in STOPPED for 20.0 seconds (5x the blocked_timeout of 4.0s!)
    for _ in range(200):
        controller.linear_vel = np.array([0.0, 0.0, 0.0])
        controller.step_control_loop(dt=0.1)
        assert controller.state == NavState.STOPPED, f"State must stay STOPPED, got {controller.state.value}"
        assert controller.path_tracker.has_path() is True

    print("  -> Passed 20.0s idle pause while STOPPED without entering FAILED / false.")

    # 4. User presses Joystick Button A (Resume)
    controller.press_button_a(current_time=controller.robot_odom_time)
    assert controller.state == NavState.RUNNING, f"State should resume to RUNNING, got {controller.state.value}"

    # 5. Execute 5 control loops immediately after resume (robot just accelerating from 0)
    for _ in range(5):
        controller.robot_pos_w[0] += 0.02
        controller.linear_vel[0] = 0.2
        controller.step_control_loop(dt=0.1)
        assert controller.state == NavState.RUNNING, f"Must NOT falsely trip into FAILED on resume! Got {controller.state.value}"

    print("  -> Passed: Resumed cleanly with Button A, no premature stall trip.")


def test_genuine_stall_triggers_failed():
    print("[TEST] 2. Genuine Stall (4.0s blocked while RUNNING) Triggers FAILED...")
    controller = MockRobotController(blocked_timeout=4.0, report_failure=True)
    pts = [[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]
    controller.load_path(pts)
    assert controller.state == NavState.RUNNING

    # Robot physically blocked by obstacle: vel=0, pos does not change
    for _ in range(39):  # 3.9 seconds
        controller.linear_vel = np.array([0.0, 0.0, 0.0])
        controller.step_control_loop(dt=0.1)
        assert controller.state == NavState.RUNNING

    # 4.0 seconds reached
    controller.step_control_loop(dt=0.1)
    assert controller.state == NavState.FAILED, f"Genuine stall must trigger FAILED, got {controller.state.value}"
    assert controller.robot_braked is True
    print("  -> Passed: Genuine stall correctly transitions to FAILED after 4.0s per standard spec.")


if __name__ == "__main__":
    print("==================================================================")
    print("   TESTING JOYSTICK B PAUSE / RESUME & STALL DETECTION FIX")
    print("==================================================================")
    test_button_b_pause_and_resume()
    test_genuine_stall_triggers_failed()
    print("==================================================================")
    print("   ALL TESTS PASSED SUCCESSFULLY (100% OK)")
    print("==================================================================")

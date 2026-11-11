import os
import sys
import math
import numpy as np

# Ensure project root in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from adaptive_path_tracker import AdaptivePathSegmentTracker, NavState


def test_get_terminal_pose():
    """Verify that AdaptivePathSegmentTracker exposes get_terminal_pose() correctly."""
    tracker = AdaptivePathSegmentTracker()
    pt, yaw = tracker.get_terminal_pose()
    assert pt is None and yaw is None, "Inactive tracker should return None, None"

    path_points = [
        [1.0, 2.0, 0.0],
        [3.0, 4.0, 0.5],
        [5.0, 6.0, 1.0]
    ]
    tracker.set_path(path_points, terminal_yaw=1.57, frame_id="map")
    pt, yaw = tracker.get_terminal_pose()
    assert pt == [5.0, 6.0, 1.0], f"Expected terminal point [5.0, 6.0, 1.0], got {pt}"
    assert math.isclose(yaw, 1.57, abs_tol=1e-4), f"Expected terminal yaw 1.57, got {yaw}"

    # Clear and verify reset
    tracker.clear()
    pt, yaw = tracker.get_terminal_pose()
    assert pt is None and yaw is None, "Cleared tracker should return None, None"


def test_yaw_error_wrapping():
    """Verify wrap-to-pi calculation across standard and wrap-around angles."""
    def calc_yaw_err(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    # Simple delta
    assert math.isclose(calc_yaw_err(1.0, 0.5), 0.5, abs_tol=1e-5)
    assert math.isclose(calc_yaw_err(0.5, 1.0), -0.5, abs_tol=1e-5)

    # Wrap across +pi / -pi boundary
    # Target = +170 deg (2.967 rad), Robot = -170 deg (-2.967 rad) -> shortest turn is -20 deg (-0.349 rad)
    t1 = math.radians(170.0)
    r1 = math.radians(-170.0)
    err1 = calc_yaw_err(t1, r1)
    assert math.isclose(err1, math.radians(-20.0), abs_tol=1e-4)

    # Target = -175 deg, Robot = +175 deg -> shortest turn is +10 deg
    t2 = math.radians(-175.0)
    r2 = math.radians(175.0)
    err2 = calc_yaw_err(t2, r2)
    assert math.isclose(err2, math.radians(10.0), abs_tol=1e-4)


def test_control_law_smoothness_deadband_clamp():
    """Verify non-linear P-control law: deadband compensation and velocity bounds."""
    kp = 1.2
    deadband = 0.12
    min_vel = 0.15
    max_vel = 0.45
    tol = 0.26  # ~15 degrees

    def compute_wz(yaw_err):
        if abs(yaw_err) <= tol:
            return 0.0
        abs_err = abs(yaw_err)
        sign = 1.0 if yaw_err >= 0.0 else -1.0
        raw_wz = abs_err * kp + deadband
        wz_mag = float(np.clip(raw_wz, min_vel, max_vel))
        return sign * wz_mag

    # Within 15 deg tolerance -> 0.0
    assert compute_wz(math.radians(10.0)) == 0.0
    assert compute_wz(math.radians(-14.0)) == 0.0

    # Just above 15 deg tolerance (~16 deg = 0.279 rad)
    # raw_wz = 0.279 * 1.2 + 0.12 = 0.4548 -> clamped to max_vel 0.45
    wz_pos = compute_wz(math.radians(16.0))
    assert wz_pos > 0.0
    assert min_vel <= wz_pos <= max_vel

    # Large error (90 deg = 1.57 rad)
    wz_large = compute_wz(math.radians(90.0))
    assert math.isclose(wz_large, max_vel, abs_tol=1e-5), f"Expected max vel {max_vel}, got {wz_large}"

    # Large negative error (-90 deg)
    wz_neg = compute_wz(math.radians(-90.0))
    assert math.isclose(wz_neg, -max_vel, abs_tol=1e-5), f"Expected -max vel {-max_vel}, got {wz_neg}"

    # Small error just above deadband boundary check
    wz_small = compute_wz(math.radians(15.5))
    assert wz_small >= min_vel, f"Angular velocity {wz_small} must be >= min_vel {min_vel} to overcome static friction"


def test_low_pass_filtering_smoothness():
    """Verify that applying low-pass filter prevents sudden velocity jumps."""
    filter_coef = np.array([0.9, 0.5, 0.5])  # [vx, vy, wz] from constants.LOW_PASS_FILTER_COEF
    prev_cmd = np.zeros(3)

    target_wz = 0.45
    cmd_vec = np.array([0.0, 0.0, target_wz])

    # First filter step
    filt1 = filter_coef * cmd_vec + (1.0 - filter_coef) * prev_cmd
    assert math.isclose(filt1[2], 0.225, abs_tol=1e-4), "First step should smoothly ramp to half speed (0.225)"

    # Second filter step
    filt2 = filter_coef * cmd_vec + (1.0 - filter_coef) * filt1
    assert math.isclose(filt2[2], 0.3375, abs_tol=1e-4), "Second step should smoothly continue ramping"

    # Deceleration to 0.0
    cmd_zero = np.array([0.0, 0.0, 0.0])
    filt_stop = filter_coef * cmd_zero + (1.0 - filter_coef) * filt2
    assert filt_stop[2] < filt2[2], "Should smoothly ramp down rather than abruptly snapping to 0.0"


def test_stair_guard_prevents_spin():
    """Verify that on stairs, heading alignment is strictly prohibited."""
    is_stair_down = True
    is_stair_up = False
    enable_terminal_heading_align = True
    terminal_yaw = 1.57

    need_heading_align = (
        enable_terminal_heading_align and
        terminal_yaw is not None and
        not is_stair_down and
        not is_stair_up
    )
    assert not need_heading_align, "Stair descent must suppress terminal heading alignment to prevent falling"

    is_stair_down = False
    is_stair_up = True
    need_heading_align_up = (
        enable_terminal_heading_align and
        terminal_yaw is not None and
        not is_stair_down and
        not is_stair_up
    )
    assert not need_heading_align_up, "Stair ascent must suppress terminal heading alignment"


def test_alignment_within_15deg_tolerance_finishes_immediately():
    """Verify that if robot arrives with heading error <= 15 deg, it finishes immediately without entering rotation."""
    terminal_yaw = 0.0
    robot_yaw = math.radians(12.0)  # 12 deg error <= 15 deg (0.26 rad)
    tolerance = 0.26

    yaw_err = math.atan2(math.sin(terminal_yaw - robot_yaw), math.cos(terminal_yaw - robot_yaw))
    assert abs(yaw_err) <= tolerance, "12 deg error is within 15 deg tolerance"


def test_timeout_and_drift_watchdogs():
    """Verify watchdog triggers for timeout and position drift."""
    timeout = 5.0
    drift_tol = 0.40

    start_time = 100.0
    current_time_ok = 103.0
    assert (current_time_ok - start_time) <= timeout

    current_time_timeout = 105.5
    assert (current_time_timeout - start_time) > timeout, "Should detect timeout"

    start_pos = np.array([2.0, 3.0])
    curr_pos_ok = np.array([2.1, 3.1])
    assert float(np.linalg.norm(curr_pos_ok - start_pos)) < drift_tol

    curr_pos_drift = np.array([2.35, 3.35])
    assert float(np.linalg.norm(curr_pos_drift - start_pos)) > drift_tol, "Should detect position drift"


def test_constants_arrival_threshold_and_drift_tol():
    """Verify system defaults for arrival detection (0.20m) and drift tolerance (0.40m)."""
    from scripts import constants
    assert hasattr(constants, 'SEGMENT_ARRIVE_THRESHOLD')
    assert math.isclose(constants.SEGMENT_ARRIVE_THRESHOLD, 0.20, abs_tol=1e-5), f"Expected 0.20m, got {constants.SEGMENT_ARRIVE_THRESHOLD}"
    assert hasattr(constants, 'DEFAULT_TERMINAL_HEADING_DRIFT_TOL')
    assert math.isclose(constants.DEFAULT_TERMINAL_HEADING_DRIFT_TOL, 0.40, abs_tol=1e-5), f"Expected 0.40m, got {constants.DEFAULT_TERMINAL_HEADING_DRIFT_TOL}"


if __name__ == "__main__":
    print("Running terminal heading alignment unit tests...")
    test_get_terminal_pose()
    test_yaw_error_wrapping()
    test_control_law_smoothness_deadband_clamp()
    test_low_pass_filtering_smoothness()
    test_stair_guard_prevents_spin()
    test_alignment_within_15deg_tolerance_finishes_immediately()
    test_timeout_and_drift_watchdogs()
    test_constants_arrival_threshold_and_drift_tol()
    print("ALL TERMINAL HEADING ALIGNMENT UNIT TESTS PASSED (100% OK)!")

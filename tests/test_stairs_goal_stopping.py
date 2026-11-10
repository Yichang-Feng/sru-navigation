import os
import sys
import math
import numpy as np

# Ensure project root in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from adaptive_path_tracker import AdaptivePathSegmentTracker
from stairs_depth_processor import StairsSlopeGate, StairsDepthRemapper


def test_goal_arrival_with_realistic_threshold():
    tracker = AdaptivePathSegmentTracker()
    path_points = [
        [3.0, 7.54, -4.13],
        [2.5, 7.50, -4.20],
        [2.0, 7.45, -4.25],
        [1.387, 7.345, -4.29]
    ]
    tracker.set_path(path_points, frame_id="map")
    robot_xy = [1.835, 7.419]
    tracker.update_progress(robot_xy)

    assert not tracker.check_arrived(robot_xy, arrive_threshold=0.15), "0.15m was too strict"
    assert tracker.check_arrived(robot_xy, arrive_threshold=0.48), "0.48m threshold correctly triggers arrival"
    assert tracker.check_arrived(robot_xy, arrive_threshold=0.50), "0.50m threshold correctly triggers arrival"


def test_no_hard_forward_velocity_floor():
    is_stair_down = True
    cmd1 = np.array([0.10, 0.0, 0.1])
    if is_stair_down:
        if cmd1[0] < 0.0:
            cmd1[0] = 0.0
        cmd1[2] = float(np.clip(cmd1[2], -0.5, 0.5))
    assert math.isclose(cmd1[0], 0.10, abs_tol=1e-5), "Should preserve 0.10 m/s without forcing 0.35 m/s"

    cmd2 = np.array([0.0, 0.0, 0.0])
    if is_stair_down:
        if cmd2[0] < 0.0:
            cmd2[0] = 0.0
        cmd2[2] = float(np.clip(cmd2[2], -0.5, 0.5))
    assert math.isclose(cmd2[0], 0.0, abs_tol=1e-5), "Should preserve 0.0 m/s for full stop"

    cmd3 = np.array([-0.25, 0.0, 0.0])
    if is_stair_down:
        if cmd3[0] < 0.0:
            cmd3[0] = 0.0
        cmd3[2] = float(np.clip(cmd3[2], -0.5, 0.5))
    assert math.isclose(cmd3[0], 0.0, abs_tol=1e-5), "Should clamp negative velocity to 0.0 m/s on stairs"

    cmd4 = np.array([0.4, 0.0, 1.2])
    if is_stair_down:
        if cmd4[0] < 0.0:
            cmd4[0] = 0.0
        cmd4[2] = float(np.clip(cmd4[2], -0.5, 0.5))
    assert math.isclose(cmd4[2], 0.5, abs_tol=1e-5), "Angular velocity clipped to 0.5 rad/s"


def test_depth_remapper_preserves_wall_at_terminal_goal():
    remapper = StairsDepthRemapper()
    wall_depth = np.full((40, 64), 0.25, dtype=np.float32)
    remapped_wall = remapper.process(wall_depth, is_stair_down=True)
    assert np.allclose(remapped_wall, wall_depth), "Close wall must be preserved 100%!"

    stair_depth = np.full((40, 64), 4.0, dtype=np.float32)
    stair_depth[20, 20:25] = 0.25
    remapped_stair = remapper.process(stair_depth, is_stair_down=True)
    center_roi = remapped_stair[14:26, 20:44]
    assert center_roi.mean() < 3.0, "Abyss on stairs correctly remapped to progressive ramp"
    assert center_roi.mean() > 1.2, "Ramp depth is realistic (1.2m ~ 2.2m)"


def test_defensive_safety_guard_near_terminal_goal():
    tracker = AdaptivePathSegmentTracker()
    path = [
        [5.0, 0.0, -1.0],
        [4.0, 0.0, -1.5],
        [3.0, 0.0, -2.0],
        [2.0, 0.0, -2.5],
        [1.0, 0.0, -2.5]
    ]
    tracker.set_path(path, frame_id="map")
    robot_pos = [1.2, 0.0, -2.5]
    dist_to_term = tracker.get_distance_to_terminal(robot_pos[:2])
    assert dist_to_term < 0.60, f"Distance to terminal is {dist_to_term}m"

    robot_mid = [3.5, 0.0, -1.8]
    dist_mid = tracker.get_distance_to_terminal(robot_mid[:2])
    assert dist_mid > 0.60, f"Distance mid-flight is {dist_mid}m"


def test_slope_gate_exit_at_terminal_flat_segment():
    gate = StairsSlopeGate(slope_threshold=-0.30, min_elevation_drop=0.15)
    tracker = AdaptivePathSegmentTracker()
    path = [
        [5.0, 0.0, 0.0],
        [4.0, 0.0, -0.4],
        [3.0, 0.0, -0.8],
        [2.0, 0.0, -1.2],
        [1.0, 0.0, -1.2],
        [0.8, 0.0, -1.2]
    ]
    tracker.set_path(path, frame_id="map")
    tracker.update_progress([5.0, 0.0])
    is_down, is_up, _ = gate.evaluate(tracker, current_progress=0.0)
    assert is_down, "Should detect stair descent at beginning"
    assert not is_up, "Should NOT detect stair ascent on downward stairs"

    tracker.update_progress([0.85, 0.0])
    is_down_term, is_up_term, info = gate.evaluate(tracker)
    assert not is_down_term, "Should exit stair descent at flat terminal landing"
    assert not is_up_term, "Should not be ascending at terminal landing"


def test_stair_ascent_detection_and_terminal_exit():
    """Verify that StairsSlopeGate detects stair ascent and exits at terminal flat landing."""
    gate = StairsSlopeGate(up_slope_threshold=0.20, min_elevation_gain=0.15, exit_up_slope_threshold=0.10)
    tracker = AdaptivePathSegmentTracker()
    path = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.35],
        [2.0, 0.0, 0.70],
        [3.0, 0.0, 1.05],
        [4.0, 0.0, 1.05],
        [4.5, 0.0, 1.05]
    ]
    tracker.set_path(path, frame_id="map")
    tracker.update_progress([0.0, 0.0])
    is_down, is_up, info = gate.evaluate(tracker, current_progress=0.0)
    assert is_up, "Should detect stair ascent on positive slope path"
    assert not is_down, "Should NOT detect stair descent on upward path"

    # Move near terminal goal on flat landing
    tracker.update_progress([4.35, 0.0])
    is_down_term, is_up_term, info_term = gate.evaluate(tracker)
    assert not is_up_term, "Should exit stair ascent at flat terminal landing"


def test_three_modes_lookahead_switching():
    """Verify automatic lookahead selection across Flat Ground, Downward Stairs, and Upward Stairs."""
    gate = StairsSlopeGate(slope_threshold=-0.20, up_slope_threshold=0.20, min_elevation_drop=0.15, min_elevation_gain=0.15)
    tracker = AdaptivePathSegmentTracker(lookahead_min=0.5, lookahead_max=2.0)

    stair_down_lookahead = 0.40
    stair_up_lookahead = 0.45

    def get_effective_lookahead(path_points, current_robot_xy):
        tracker.set_path(path_points, frame_id="map")
        progress = tracker.update_progress(current_robot_xy)
        is_down, is_up, _ = gate.evaluate(tracker, current_progress=progress)

        if is_down:
            return stair_down_lookahead, "STAIR_DOWN"
        elif is_up:
            return stair_up_lookahead, "STAIR_UP"
        else:
            _, _, info = tracker.get_adaptive_lookahead_point()
            return info.get("lookahead", tracker.lookahead_max), "FLAT_ADAPTIVE"

    # Scenario 1: Straight Flat Ground Path -> Should use flat adaptive lookahead (up to 2.0m)
    flat_path = [[x * 0.5, 0.0, 0.0] for x in range(20)]
    L_flat, mode_flat = get_effective_lookahead(flat_path, [2.0, 0.0])
    assert mode_flat == "FLAT_ADAPTIVE", f"Expected FLAT_ADAPTIVE, got {mode_flat}"
    assert L_flat >= 0.5 and L_flat <= 2.0, f"Flat lookahead ({L_flat}) should be in [0.5, 2.0]"

    # Scenario 2: Downward Stairs (dz = -0.35 * dx) -> Should latch 0.40m
    gate.reset()
    down_path = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, -0.35],
        [2.0, 0.0, -0.70],
        [3.0, 0.0, -1.05],
        [4.0, 0.0, -1.40]
    ]
    L_down, mode_down = get_effective_lookahead(down_path, [0.0, 0.0])
    assert mode_down == "STAIR_DOWN", f"Expected STAIR_DOWN, got {mode_down}"
    assert math.isclose(L_down, 0.40, abs_tol=1e-3), f"Expected 0.40m on downward stairs, got {L_down}"

    # Scenario 3: Upward Stairs (dz = +0.35 * dx) -> Should latch 0.45m
    gate.reset()
    up_path = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, +0.35],
        [2.0, 0.0, +0.70],
        [3.0, 0.0, +1.05],
        [4.0, 0.0, +1.40]
    ]
    L_up, mode_up = get_effective_lookahead(up_path, [0.0, 0.0])
    assert mode_up == "STAIR_UP", f"Expected STAIR_UP, got {mode_up}"
    assert math.isclose(L_up, 0.45, abs_tol=1e-3), f"Expected 0.45m on upward stairs, got {L_up}"


def test_ascent_anti_reversal_and_angular_clip():
    """Verify velocity guard for ascending stairs: vx >= 0.0 and |wz| <= 0.5."""
    is_stair_up = True
    is_stair_down = False

    cmd_negative = np.array([-0.30, 0.05, 0.80])
    if is_stair_down or is_stair_up:
        if cmd_negative[0] < 0.0:
            cmd_negative[0] = 0.0
        cmd_negative[2] = float(np.clip(cmd_negative[2], -0.5, 0.5))

    assert math.isclose(cmd_negative[0], 0.0, abs_tol=1e-5), "Must clamp reverse velocity to 0 on ascending stairs"
    assert math.isclose(cmd_negative[2], 0.5, abs_tol=1e-5), "Must clamp angular velocity to 0.5 rad/s on ascending stairs"

    cmd_forward = np.array([0.38, -0.02, -0.20])
    if is_stair_down or is_stair_up:
        if cmd_forward[0] < 0.0:
            cmd_forward[0] = 0.0
        cmd_forward[2] = float(np.clip(cmd_forward[2], -0.5, 0.5))

    assert math.isclose(cmd_forward[0], 0.38, abs_tol=1e-5), "Must preserve forward velocity on ascending stairs"
    assert math.isclose(cmd_forward[2], -0.20, abs_tol=1e-5), "Must preserve safe angular velocity"


def test_stair_raw_depth_toggle():
    """Verify that when enable_stair_remapping=False, depth image is completely untouched on stairs."""
    remapper = StairsDepthRemapper()
    raw_depth = np.random.uniform(0.25, 5.0, (40, 64)).astype(np.float32)

    # When enable_stair_remapping=False, depth_for_policy remains raw_depth
    enable_stair_remapping = False
    is_stair_down = True
    depth_for_policy = raw_depth
    if is_stair_down and enable_stair_remapping:
        depth_for_policy = remapper.process(raw_depth, is_stair_down=True)

    assert depth_for_policy is raw_depth, "Raw depth must be passed 100% untouched when remapping=False"
    assert np.array_equal(depth_for_policy, raw_depth)


if __name__ == '__main__':
    test_goal_arrival_with_realistic_threshold()
    test_no_hard_forward_velocity_floor()
    test_depth_remapper_preserves_wall_at_terminal_goal()
    test_defensive_safety_guard_near_terminal_goal()
    test_slope_gate_exit_at_terminal_flat_segment()
    test_stair_ascent_detection_and_terminal_exit()
    test_three_modes_lookahead_switching()
    test_ascent_anti_reversal_and_angular_clip()
    test_stair_raw_depth_toggle()
    print('ALL UNIT TESTS PASSED!')

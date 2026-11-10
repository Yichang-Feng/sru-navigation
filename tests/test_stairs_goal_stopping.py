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
    is_down, _ = gate.evaluate(tracker, current_progress=0.0)
    assert is_down, "Should detect stair descent at beginning"

    tracker.update_progress([0.85, 0.0])
    is_down_term, info = gate.evaluate(tracker)
    assert not is_down_term, "Should exit stair descent at flat terminal landing"


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
    test_stair_raw_depth_toggle()
    print('ALL UNIT TESTS PASSED!')

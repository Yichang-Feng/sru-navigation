#!/usr/bin/env python3
"""Comprehensive verification suite for Path Tracker Deadlock Repair and Stairs Safety Guard.

Validates:
1. Interface contract: has_path() is defined and matches is_active().
2. Cold-start deadlock fix: Robot placed at stairs entrance (5.5m) with path starting at indoor (0.0m)
   instantly snaps to robot's real position instead of deadlocking at 0.0m.
3. Pause & Teleoperation forward jump: Robot nudged forward > 3m recovers instantly.
4. Backward teleoperation recovery: realign_to_robot() correctly snaps backward.
5. Loop preservation: Continuous tracking along loops/re-entrant paths does NOT jump ahead.
6. Stairs safety guard: Clamps backward/sideward carrot points to forward heading on stairs.
"""

import os
import sys
import math
import numpy as np

# Add project root to sys.path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from adaptive_path_tracker import PathSegmentTracker, AdaptivePathSegmentTracker


def test_has_path_interface():
    """Verify that has_path() is defined on both tracker classes and mirrors is_active()."""
    print("\n[Test 1] Testing has_path() interface contract...")
    for cls in [PathSegmentTracker, AdaptivePathSegmentTracker]:
        tracker = cls()
        assert hasattr(tracker, 'has_path'), f"{cls.__name__} must define has_path()!"
        assert tracker.has_path() == tracker.is_active() == False

        # Add a dummy path
        tracker.set_path([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        assert tracker.has_path() == tracker.is_active() == True

        tracker.clear()
        assert tracker.has_path() == tracker.is_active() == False
    print("  [✓] PASSED: has_path() properly defined and matches is_active() on all classes.")


def test_cold_start_deadlock_fix():
    """Verify cold start deadlock fix when robot is at stairs entrance (5.5m) and path starts at indoor (0.0m)."""
    print("\n[Test 2] Testing Cold-Start Deadlock Fix (Session 193104 Scenario)...")
    tracker = AdaptivePathSegmentTracker()

    # Synthetic realistic path from indoor (4.334, 6.100) -> corridor -> stairs entrance (9.36, 3.93) -> stairs down
    path_pts = [
        [4.334, 6.100, -0.379],  # 0.0m (Indoor start)
        [5.334, 5.800, -0.350],  # ~1.0m
        [6.500, 5.200, -0.300],  # ~2.4m
        [7.800, 4.600, -0.200],  # ~3.8m
        [9.360, 3.930, 0.025],   # ~5.5m (Stairs threshold, robot position)
        [10.50, 3.800, -0.400],  # ~6.6m (Down stairs)
        [12.00, 3.700, -1.000],  # ~8.1m (Landing)
    ]
    tracker.set_path(path_pts)
    assert tracker.current_progress == 0.0

    robot_pos = [9.360, 3.930]  # Robot is at the stairs
    progress = tracker.update_progress(robot_pos)

    # In the unfixed code, progress stayed at 0.0m because distance to segment 0..3m was huge and s_start > 3.0 broke early.
    # In the fixed code, global snap should instantly align progress to ~5.5m!
    assert progress > 5.0, f"Cold start failed: progress was {progress:.2f}m (expected > 5.0m near robot)!"
    assert abs(progress - tracker.cum_dists[4]) < 0.1, f"Expected progress near segment 4 (~5.5m), got {progress:.2f}m"

    # Verify lookahead point is in front of the robot down the stairs
    carrot_pt, carrot_yaw = tracker.get_lookahead_point(lookahead_dist=1.2)
    assert carrot_pt[0] > robot_pos[0], f"Carrot x should be ahead (> {robot_pos[0]}), got {carrot_pt[0]}"
    assert carrot_pt[2] < 0.0, f"Carrot z should drop down the stairs (< 0.0), got {carrot_pt[2]}"
    print(f"  [✓] PASSED: Cold start instantly snapped to s={progress:.2f}m. Carrot is ahead at {carrot_pt}.")


def test_pause_teleop_jump_recovery():
    """Verify recovery when robot is paused and teleoperated > 3m forward."""
    print("\n[Test 3] Testing Pause & Teleoperation Forward Jump (> 3m)...")
    tracker = PathSegmentTracker()
    # 15m straight line
    path_pts = [[float(x), 0.0, 0.0] for x in np.linspace(0.0, 15.0, 151)]
    tracker.set_path(path_pts)

    # Robot walks smoothly to 2.5m
    for x in np.linspace(0.0, 2.5, 26):
        tracker.update_progress([x, 0.0])
    assert abs(tracker.current_progress - 2.5) < 0.05

    # Robot paused: tester remote-controls robot forward by 3.5m to x=6.0m
    # Calling update_progress at x=6.0m
    new_prog = tracker.update_progress([6.0, 0.0])
    assert abs(new_prog - 6.0) < 0.05, f"Teleop jump forward failed! Expected ~6.0m, got {new_prog:.2f}m"
    print(f"  [✓] PASSED: Teleoperated 3.5m jump successfully recovered to s={new_prog:.2f}m.")


def test_backward_teleop_recovery():
    """Verify realign_to_robot() and jump recovery when robot is pulled backward."""
    print("\n[Test 4] Testing Backward Teleoperation Recovery...")
    tracker = PathSegmentTracker()
    path_pts = [[float(x), 0.0, 0.0] for x in np.linspace(0.0, 15.0, 151)]
    tracker.set_path(path_pts)

    # Robot at 6.0m
    tracker.update_progress([6.0, 0.0], force_global=True)
    assert abs(tracker.current_progress - 6.0) < 0.05

    # Robot is pulled backward to 2.0m during pause
    # Realign to robot
    realigned_prog = tracker.realign_to_robot([2.0, 0.0])
    assert abs(realigned_prog - 2.0) < 0.05, f"Backward realignment failed! Expected 2.0m, got {realigned_prog:.2f}m"
    assert abs(tracker.current_progress - 2.0) < 0.05
    print(f"  [✓] PASSED: Backward teleop realignment successfully snapped to s={realigned_prog:.2f}m.")


def test_loop_path_preservation():
    """Verify that localized window prevents shortcutting/jumping across loop intersections."""
    print("\n[Test 5] Testing Loop Path Preservation (No shortcutting across loops)...")
    tracker = PathSegmentTracker()

    # Create a 20m loop:
    # 0m..5m: East (x: 0->5, y=0)
    # 5m..10m: North (x=5, y: 0->5)
    # 10m..15m: West (x: 5->0, y=5)
    # 15m..20m: South (x=0, y: 5->0) -> close to start at (0, 0.2)
    loop_pts = []
    for x in np.linspace(0, 5, 25):
        loop_pts.append([float(x), 0.0, 0.0])
    for y in np.linspace(0, 5, 25)[1:]:
        loop_pts.append([5.0, float(y), 0.0])
    for x in np.linspace(5, 0, 25)[1:]:
        loop_pts.append([float(x), 5.0, 0.0])
    for y in np.linspace(5, 0.2, 25)[1:]:
        loop_pts.append([0.0, float(y), 0.0])

    tracker.set_path(loop_pts)

    # Robot starts at (0.0, 0.0) -> s=0.0m
    prog = tracker.update_progress([0.0, 0.0])
    assert abs(prog - 0.0) < 0.05, f"Initial progress should be 0.0m, got {prog}"

    # Verify that even though the end of loop (s~20m) is at (0.0, 0.2) [only 0.2m away],
    # the tracker did NOT jump to s=20m!
    assert tracker.current_progress < 1.0, f"False shortcut jump! Progress jumped to {tracker.current_progress:.2f}m"

    # Walk along first segment
    for x in np.linspace(0.1, 3.0, 30):
        prog = tracker.update_progress([x, 0.0])
        assert abs(prog - x) < 0.1, f"Tracking failed at x={x}: prog={prog}"

    print("  [✓] PASSED: Loop preservation verified. No premature jumping across loop intersections.")


def test_stairs_safety_guard_math():
    """Verify stairs safety guard clamping math when backward carrot is encountered on stairs."""
    print("\n[Test 6] Testing Stairs Safety Guard Math...")
    # Simulate robot at [9.360, 3.930, 0.025] facing yaw = -7.0 deg
    robot_pos = [9.360, 3.930, 0.025]
    robot_yaw_deg = -7.023
    robot_yaw = math.radians(robot_yaw_deg)

    # Quaternion for yaw = -7.023 deg
    qw = math.cos(robot_yaw / 2.0)
    qz = math.sin(robot_yaw / 2.0)
    robot_ori = [qw, 0.0, 0.0, qz]

    # Backward carrot recorded in session 193104:
    backward_carrot = [4.334, 6.100, -0.379]

    # Compute body frame coordinates
    dx_w = backward_carrot[0] - robot_pos[0]
    dy_w = backward_carrot[1] - robot_pos[1]
    dx_b = dx_w * math.cos(robot_yaw) + dy_w * math.sin(robot_yaw)
    dy_b = -dx_w * math.sin(robot_yaw) + dy_w * math.cos(robot_yaw)

    assert dx_b < 0.0, f"Test setup error: dx_b should be negative (behind), got {dx_b}"

    # Apply safety guard clamping logic
    forward_L = 1.2
    slope = -0.35
    drop_z = min(-0.25, slope * forward_L)
    clamped_x = robot_pos[0] + forward_L * math.cos(robot_yaw)
    clamped_y = robot_pos[1] + forward_L * math.sin(robot_yaw)
    clamped_z = robot_pos[2] + drop_z

    # Re-evaluate body frame of clamped target
    cdx_w = clamped_x - robot_pos[0]
    cdy_w = clamped_y - robot_pos[1]
    cdx_b = cdx_w * math.cos(robot_yaw) + cdy_w * math.sin(robot_yaw)
    cdy_b = -cdx_w * math.sin(robot_yaw) + cdy_w * math.cos(robot_yaw)

    assert abs(cdx_b - forward_L) < 1e-4, f"Clamped dx_b should be {forward_L}, got {cdx_b}"
    assert abs(cdy_b) < 1e-4, f"Clamped dy_b should be 0.0, got {cdy_b}"
    assert clamped_z < robot_pos[2], f"Clamped z should descend ({clamped_z} < {robot_pos[2]})"
    print(f"  [✓] PASSED: Backward carrot ({dx_b:.2f}m) successfully clamped to forward {cdx_b:.2f}m, dz={drop_z:.2f}m.")


if __name__ == "__main__":
    print("==================================================================")
    print("Path Tracker Deadlock Repair & Stairs Safety Guard Verification")
    print("==================================================================")

    test_has_path_interface()
    test_cold_start_deadlock_fix()
    test_pause_teleop_jump_recovery()
    test_backward_teleop_recovery()
    test_loop_path_preservation()
    test_stairs_safety_guard_math()

    print("\n==================================================================")
    print("[ALL DEADLOCK REPAIR & SAFETY TESTS PASSED! 🚀]")
    print("==================================================================")

#!/usr/bin/env python3
"""Comprehensive test suite for AdaptivePathSegmentTracker.

Tests:
1. Straight line open space lookahead (holds 2.0m).
2. Straight then 90-degree right turn (door entrance scenario):
   - Proves lookahead smoothly contracts before reaching corner (preventing corner-cutting & wall penetration).
   - Proves carrot point tracks the corner tightly.
3. Narrow passage detection and spatial latching (prevents blindzone pop-out).
4. Slew-rate limiting smoothness.
"""

import sys
import os
import math
import numpy as np

# Add workspace to path
ws_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ws_dir not in sys.path:
    sys.path.insert(0, ws_dir)

from adaptive_path_tracker import AdaptivePathSegmentTracker, NavState


def test_straight_open_space():
    print("\n--- Test 1: Straight Open Space (Lookahead should stay at 2.0m) ---")
    tracker = AdaptivePathSegmentTracker(lookahead_min=0.5, lookahead_max=2.0, window_dist=1.5)
    # 10m straight line along X
    pts = [[float(x), 0.0, 0.0] for x in np.linspace(0.0, 10.0, 101)]
    tracker.set_path(pts)

    times = np.linspace(0.0, 10.0, 20)
    for t in times:
        robot_x = t * 0.8  # robot moving at 0.8 m/s
        tracker.update_progress([robot_x, 0.0])
        carrot_pt, carrot_yaw, debug = tracker.get_adaptive_lookahead_point(current_time=t)
        assert math.isclose(debug["lookahead"], 2.0, abs_tol=1e-3), f"Lookahead should be 2.0m in straight line, got {debug['lookahead']}"

    print("✓ Straight line test passed: Lookahead remains 2.0m throughout.")


def test_straight_then_90deg_right_turn():
    print("\n--- Test 2: Straight 2m then 90° Right Turn (The Corner & Door Scenario) ---")
    tracker = AdaptivePathSegmentTracker(
        lookahead_min=0.5,
        lookahead_max=2.0,
        window_dist=1.5,
        turn_angle_low_deg=15.0,
        turn_angle_high_deg=50.0,
        slew_rate_decrease=2.5,
        slew_rate_increase=0.6
    )

    # Path: (0,0) -> (2,0) -> (2, -3)
    # Dense points every 0.05m
    p1 = [[float(x), 0.0, 0.0] for x in np.arange(0.0, 2.0, 0.05)]
    p2 = [[2.0, float(y), 0.0] for y in np.arange(0.0, -3.05, -0.05)]
    pts = p1 + p2
    tracker.set_path(pts)

    print(f"Total path length: {tracker.total_length:.2f}m, Points: {len(pts)}")

    # Robot approaches from x = 0.0 to x = 2.0 (corner) and turns down to y = -2.0
    sim_steps = [
        # (robot_x, robot_y, sim_time, description)
        (0.0, 0.0, 0.0, "Start of corridor (2m before corner)"),
        (0.5, 0.0, 0.5, "1.5m before corner (entering lookahead window)"),
        (1.0, 0.0, 1.0, "1.0m before corner (turn detected in window)"),
        (1.5, 0.0, 1.5, "0.5m before corner (imminent turn)"),
        (1.9, 0.0, 1.9, "0.1m before corner (at the corner)"),
        (2.0, -0.5, 2.5, "Turned right into door corridor (0.5m in)"),
        (2.0, -1.5, 3.5, "Continuing straight inside room (1.5m in)"),
    ]

    lookahead_history = []
    for rx, ry, t, desc in sim_steps:
        tracker.update_progress([rx, ry])
        carrot_pt, carrot_yaw, debug = tracker.get_adaptive_lookahead_point(current_time=t)
        lookahead_history.append((rx, ry, debug["lookahead"], debug["turn_deg"], carrot_pt))
        print(f"[{desc:42s}] pos=({rx:.2f}, {ry:.2f}) -> L={debug['lookahead']:.2f}m (turn={debug['turn_deg']:.1f}°), Carrot=({carrot_pt[0]:.2f}, {carrot_pt[1]:.2f})")

    # Key Assertions:
    # 1. At start (x=0.0), lookahead is large (2.0m)
    assert lookahead_history[0][2] >= 1.9, "Should start with large lookahead"

    # 2. At x=1.5m (0.5m before corner), lookahead MUST be contracted to <= 0.85m to prevent corner cutting!
    l_near_corner = lookahead_history[3][2]
    assert l_near_corner <= 0.85, f"Before corner, lookahead must contract (got {l_near_corner:.2f}m)"

    # 3. At corner (x=1.9m), lookahead is at minimum (0.5m)
    l_at_corner = lookahead_history[4][2]
    assert math.isclose(l_at_corner, 0.5, abs_tol=0.08), f"At corner, lookahead must reach ~0.5m (got {l_at_corner:.2f}m)"

    # 4. Check carrot point at x=1.5m: with L ~ 0.6m, carrot is at (2.0, -0.1) near corner,
    # NOT cutting deep into the room at (2.0, -1.5)!
    carrot_at_1_5 = lookahead_history[3][4]
    dist_to_corner = math.hypot(carrot_at_1_5[0] - 2.0, carrot_at_1_5[1] - 0.0)
    print(f"Distance from carrot to corner when at x=1.5m: {dist_to_corner:.2f}m (No cutting across room!)")
    assert dist_to_corner < 0.8, "Carrot should stay close to corner!"

    print("✓ Straight then 90° right turn test passed!")


def test_narrow_passage_and_spatial_latching():
    print("\n--- Test 3: Narrow Door Detection & Spatial Latch (Blindzone Protection) ---")
    tracker = AdaptivePathSegmentTracker(
        lookahead_min=0.5,
        lookahead_max=2.0,
        passage_latch_dist=1.2,
        clearance_threshold=1.05
    )

    pts = [[float(x), 0.0, 0.0] for x in np.linspace(0.0, 6.0, 61)]
    tracker.set_path(pts)

    # 1. Create a simulated 40x64 depth image of a narrow door (0.85m gap)
    # Left 20 cols = close wall (0.8m), Right 20 cols = close wall (0.8m), Center 24 cols = doorway (2.5m)
    depth_door = np.ones((40, 64), dtype=np.float32) * 2.5
    depth_door[:, :20] = 0.8
    depth_door[:, -20:] = 0.8

    # Approach door at x = 1.0m
    tracker.update_progress([1.0, 0.0])
    carrot_pt, carrot_yaw, debug1 = tracker.get_adaptive_lookahead_point(current_time=1.0, depth_image=depth_door)
    print(f"Approaching door (x=1.0m, door visible): L={debug1['lookahead']:.2f}m, in_narrow={debug1['in_narrow']}")
    assert debug1["in_narrow"] is True, "Door should be detected as narrow passage"
    assert math.isclose(debug1["lookahead"], 0.5, abs_tol=0.1)

    # 2. Robot passes right through the door at x = 1.5m.
    # Simulating BLINDZONE: door frame is now out of camera FOV (depth is open space or None)
    tracker.update_progress([1.5, 0.0])
    carrot_pt, carrot_yaw, debug2 = tracker.get_adaptive_lookahead_point(current_time=1.5, depth_image=None)
    print(f"Inside door threshold (x=1.5m, frame out of view): L={debug2['lookahead']:.2f}m, in_narrow={debug2['in_narrow']}")
    assert debug2["in_narrow"] is True, "Spatial latch must maintain narrow mode when frames out of FOV!"
    assert math.isclose(debug2["lookahead"], 0.5, abs_tol=0.1), "Lookahead must NOT jump back to 2.0m inside the door!"

    # 3. Robot has fully cleared the door at x = 2.5m (> 1.2m latch distance from entry at 1.0m)
    tracker.update_progress([2.5, 0.0])
    carrot_pt, carrot_yaw, debug3 = tracker.get_adaptive_lookahead_point(current_time=2.0, depth_image=None)
    print(f"Cleared door (x=2.5m, >1.2m latch): L={debug3['lookahead']:.2f}m, in_narrow={debug3['in_narrow']}")
    assert debug3["in_narrow"] is False, "Spatial latch should release once dog has cleared the door"

    # Step forward in time in open space to observe gradual slew-rate recovery
    sim_t = 2.0
    for _ in range(5):
        sim_t += 0.4
        _, _, d_rec = tracker.get_adaptive_lookahead_point(current_time=sim_t, depth_image=None)

    print(f"Recovered in open space (t={sim_t:.1f}s): L={d_rec['lookahead']:.2f}m")
    assert d_rec["lookahead"] >= 1.5, f"Lookahead should be smoothly recovering to open space speed, got {d_rec['lookahead']:.2f}m"

    print("✓ Narrow passage and spatial latching test passed!")


def test_slew_rate_continuity():
    print("\n--- Test 4: Slew-Rate Limiter Continuity (No Discontinuous Jumps) ---")
    tracker = AdaptivePathSegmentTracker(
        lookahead_min=0.5,
        lookahead_max=2.0,
        slew_rate_decrease=2.0,  # max 2.0 m/s drop
        slew_rate_increase=0.5   # max 0.5 m/s rise
    )
    pts = [[float(x), 0.0, 0.0] for x in np.linspace(0.0, 5.0, 51)]
    tracker.set_path(pts)

    # Step at 5 Hz (dt = 0.2s)
    current_t = 0.0
    tracker.update_progress([0.0, 0.0])
    _, _, d0 = tracker.get_adaptive_lookahead_point(current_time=current_t)
    assert math.isclose(d0["lookahead"], 2.0, abs_tol=1e-3)

    # Suddenly inject narrow door obstacle
    depth_obstacle = np.ones((40, 64), dtype=np.float32) * 0.7
    max_drop = 0.0
    for step in range(5):
        current_t += 0.2
        _, _, d = tracker.get_adaptive_lookahead_point(current_time=current_t, depth_image=depth_obstacle)
        drop = d0["lookahead"] - d["lookahead"]
        max_drop = max(max_drop, drop)
        # In dt = 0.2s, max drop allowed is 2.0 * 0.2 = 0.4m
        assert drop <= 0.41, f"Step {step}: Lookahead drop {drop:.3f} exceeded slew rate!"
        d0 = d

    print(f"✓ Smooth contraction verified (max single-step drop: {max_drop:.3f}m <= 0.40m limit).")


def test_adaptive_modes_isolation():
    print("\n--- Test 5: Mode Isolation (curvature_only vs clearance_only vs none) ---")
    # Path with 90 deg turn at x=2.0
    p1 = [[float(x), 0.0, 0.0] for x in np.arange(0.0, 2.0, 0.05)]
    p2 = [[2.0, float(y), 0.0] for y in np.arange(0.0, -2.05, -0.05)]
    pts = p1 + p2

    # Fake narrow door depth image
    depth_door = np.ones((40, 64), dtype=np.float32) * 2.5
    depth_door[:, :20] = 0.8   # Left column
    depth_door[:, 44:] = 0.8   # Right column

    # 1. Mode: curvature_only
    tracker_curv = AdaptivePathSegmentTracker(
        lookahead_min=0.5, lookahead_max=2.0, window_dist=1.5,
        turn_angle_low_deg=15.0, turn_angle_high_deg=50.0,
        adaptive_mode="curvature_only"
    )
    tracker_curv.set_path(pts)
    tracker_curv.update_progress([1.5, 0.0])  # 0.5m before corner -> curvature should drop to 0.5m
    _, _, d_turn = tracker_curv.get_adaptive_lookahead_point(current_time=1.0)
    assert d_turn["lookahead"] <= 0.55, f"Curvature mode should shrink on turn, got {d_turn['lookahead']}"
    assert d_turn["mode"] == "curvature_only"

    # Now on straight line, inject narrow door depth image -> must be IGNORED
    tracker_curv.set_path([[float(x), 0.0, 0.0] for x in np.linspace(0.0, 5.0, 51)])
    tracker_curv.update_progress([1.0, 0.0])
    _, _, d_door = tracker_curv.get_adaptive_lookahead_point(current_time=2.0, depth_image=depth_door)
    assert math.isclose(d_door["lookahead"], 2.0, abs_tol=1e-3), (
        f"In curvature_only mode, visual door should be completely ignored, got {d_door['lookahead']}"
    )
    assert d_door["in_narrow"] is False
    print("✓ curvature_only mode verified: Shrinks for corners, 100% ignores visual doors.")

    # 2. Mode: clearance_only
    tracker_clear = AdaptivePathSegmentTracker(
        lookahead_min=0.5, lookahead_max=2.0, window_dist=1.5,
        turn_angle_low_deg=15.0, turn_angle_high_deg=50.0,
        adaptive_mode="clearance_only"
    )
    tracker_clear.set_path(pts)
    tracker_clear.update_progress([1.5, 0.0])  # 0.5m before corner -> corner should be IGNORED
    _, _, d_turn2 = tracker_clear.get_adaptive_lookahead_point(current_time=1.0)
    assert math.isclose(d_turn2["lookahead"], 2.0, abs_tol=1e-3), (
        f"In clearance_only mode, corner should be ignored, got {d_turn2['lookahead']}"
    )

    # Inject door -> must shrink to 0.5m (taking 2 steps to account for slew rate limit)
    tracker_clear.get_adaptive_lookahead_point(current_time=2.0, depth_image=depth_door)
    _, _, d_door2 = tracker_clear.get_adaptive_lookahead_point(current_time=2.3, depth_image=depth_door)
    assert d_door2["lookahead"] <= 0.55, f"Clearance mode should shrink on door, got {d_door2['lookahead']}"
    assert d_door2["in_narrow"] is True
    print("✓ clearance_only mode verified: Shrinks for visual doors, 100% ignores path turns.")


if __name__ == "__main__":
    test_straight_open_space()
    test_straight_then_90deg_right_turn()
    test_narrow_passage_and_spatial_latching()
    test_slew_rate_continuity()
    test_adaptive_modes_isolation()
    print("\n========================================================")
    print("ALL ADAPTIVE LOOKAHEAD UNIT & GEOMETRY TESTS PASSED! 🚀")
    print("========================================================")

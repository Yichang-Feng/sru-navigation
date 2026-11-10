"""Stairs Depth Processor & 3D Path Slope Gate.

Provides decoupled, non-invasive modules:
1. StairsSlopeGate: Analyzes 3D path elevation change to detect stair descent with spatial hysteresis.
2. StairsDepthRemapper: Remaps the deep stairwell abyss into a progressive virtual stair ramp
   without altering flat-ground data and preserving real obstacle detection.
"""

import math
import numpy as np
import cv2


class StairsSlopeGate:
    """Detects when the robot is entering, descending, and exiting stairs based on 3D path geometry."""

    def __init__(
        self,
        slope_threshold: float = -0.20,      # Slope threshold (dz / ds_2d) for stair descent (approx. >11.5° down, allows ~0.5m advance detection)
        lookahead_eval_dist: float = 1.2,    # Lookahead distance ahead to evaluate path slope (meters)
        exit_slope_threshold: float = -0.10, # Slope above which path is considered level/flat again
        exit_dist_threshold: float = 0.6,     # Distance robot must travel on flat surface before releasing stair latch
        min_elevation_drop: float = 0.15,    # Minimum total z-drop across eval window to confirm stairs (meters, single step drop)
    ):
        self.slope_threshold = float(slope_threshold)
        self.lookahead_eval_dist = float(lookahead_eval_dist)
        self.exit_slope_threshold = float(exit_slope_threshold)
        self.exit_dist_threshold = float(exit_dist_threshold)
        self.min_elevation_drop = float(min_elevation_drop)

        # Dynamic state
        self.is_descending_stairs = False
        self.stair_entry_progress = 0.0
        self.flat_ground_entry_progress = None
        self.consecutive_slope_detections = 0
        self.required_consecutive_detections = 1

    def reset(self):
        """Reset internal latching state."""
        self.is_descending_stairs = False
        self.stair_entry_progress = 0.0
        self.flat_ground_entry_progress = None
        self.consecutive_slope_detections = 0

    def evaluate(self, path_tracker, current_progress: float = None):
        """Evaluate 3D path slope ahead of the robot.

        Args:
            path_tracker: PathSegmentTracker or AdaptivePathSegmentTracker instance.
            current_progress: Optional arc-length progress along path. If None, uses tracker.current_progress.

        Returns:
            tuple: (is_descending_stairs: bool, debug_dict: dict)
        """
        if path_tracker is None or not hasattr(path_tracker, 'is_active') or not path_tracker.is_active():
            self.reset()
            return False, {"mode": "inactive", "slope": 0.0, "is_descending": False}

        if len(path_tracker.points) < 2:
            self.reset()
            return False, {"mode": "single_point", "slope": 0.0, "is_descending": False}

        s_curr = current_progress if current_progress is not None else getattr(path_tracker, 'current_progress', 0.0)
        s_total = getattr(path_tracker, 'total_length', 0.0)

        # Ensure we have enough path ahead to evaluate
        eval_window = min(self.lookahead_eval_dist, max(0.0, s_total - s_curr))
        if eval_window < 0.3:
            # Reached near terminal goal: check if remaining path elevation drop is flat
            p_curr, _ = path_tracker.get_lookahead_point(lookahead_dist=0.0)
            p_term = path_tracker.points[-1]
            dz_term = float(p_term[2] - p_curr[2]) if p_curr is not None else 0.0
            if dz_term > -self.min_elevation_drop or (s_total - s_curr < 0.2):
                # Terminal segment is level ground or robot is at end of path: exit stair descent
                self.is_descending_stairs = False
                self.flat_ground_entry_progress = None
            elif self.is_descending_stairs and self.flat_ground_entry_progress is None:
                self.flat_ground_entry_progress = s_curr
            if self.flat_ground_entry_progress is not None and (s_curr - self.flat_ground_entry_progress >= self.exit_dist_threshold):
                self.is_descending_stairs = False
                self.flat_ground_entry_progress = None
            return self.is_descending_stairs, {
                "mode": "near_terminal",
                "slope": 0.0,
                "dz": dz_term,
                "is_descending": self.is_descending_stairs
            }

        # Query 3D carrot points at current position and future evaluation point
        p_curr, _ = path_tracker.get_lookahead_point(lookahead_dist=0.0)
        p_future, _ = path_tracker.get_lookahead_point(lookahead_dist=eval_window)

        if p_curr is None or p_future is None:
            return self.is_descending_stairs, {"mode": "lookup_fail", "slope": 0.0, "is_descending": self.is_descending_stairs}

        dz = float(p_future[2] - p_curr[2])
        d_xy = float(math.hypot(p_future[0] - p_curr[0], p_future[1] - p_curr[1]))

        slope = dz / (d_xy + 1e-5)

        # State transition logic with hysteresis
        if not self.is_descending_stairs:
            # Entry condition: slope < threshold and sufficient z drop
            if slope < self.slope_threshold and dz < -self.min_elevation_drop:
                self.consecutive_slope_detections += 1
                if self.consecutive_slope_detections >= self.required_consecutive_detections:
                    self.is_descending_stairs = True
                    self.stair_entry_progress = s_curr
                    self.flat_ground_entry_progress = None
            else:
                self.consecutive_slope_detections = 0
        else:
            # Exit condition: future slope becomes level again
            if slope > self.exit_slope_threshold:
                if self.flat_ground_entry_progress is None:
                    self.flat_ground_entry_progress = s_curr
                elif (s_curr - self.flat_ground_entry_progress) >= self.exit_dist_threshold:
                    self.is_descending_stairs = False
                    self.flat_ground_entry_progress = None
                    self.consecutive_slope_detections = 0
            else:
                # Still on steep stairs, reset exit counter
                self.flat_ground_entry_progress = None

        debug_info = {
            "slope": slope,
            "dz": dz,
            "d_xy": d_xy,
            "eval_window": eval_window,
            "is_descending": self.is_descending_stairs,
            "consecutive": self.consecutive_slope_detections
        }
        return self.is_descending_stairs, debug_info


class StairsDepthRemapper:
    """Remaps depth images in stair descent regions to project a synthetic walkable ramp.

    Guarantees:
    - 100% bitwise identity on flat ground (Zero modification when is_stair_down=False).
    - Preserves real obstacles (< 2.2m) and terminal walls (< 0.45m) on stairs.
    - Eliminates false abyss-induced avoidance while maintaining close-wall blindzone safety.
    """

    def __init__(
        self,
        ramp_base: float = 1.0,         # Base depth at bottom row (meters)
        ramp_step: float = 0.046,       # Depth increment per row upwards (meters/row)
        abyss_threshold: float = 2.2,   # Depths greater than this in stair channel are considered abyss
        side_min_clamp: float = 0.65,   # Minimum depth for boundary railing columns to prevent hard spin
        left_col: int = 16,             # Left column boundary of central walking corridor
        right_col: int = 48,            # Right column boundary of central walking corridor
        min_row: int = 12,              # Top row index for virtual ramp projection
        max_row: int = 39,              # Bottom row index for virtual ramp projection
        blindzone_val: float = 0.25     # Value used for close blindzone clamping
    ):
        self.ramp_base = float(ramp_base)
        self.ramp_step = float(ramp_step)
        self.abyss_threshold = float(abyss_threshold)
        self.side_min_clamp = float(side_min_clamp)
        self.left_col = int(left_col)
        self.right_col = int(right_col)
        self.min_row = int(min_row)
        self.max_row = int(max_row)
        self.blindzone_val = float(blindzone_val)

    def process(self, depth_array: np.ndarray, is_stair_down: bool = False) -> np.ndarray:
        """Process depth image. If is_stair_down is False, returns exact unmodified array.

        Args:
            depth_array: NumPy array of shape (40, 64) with float32 depths.
            is_stair_down: Whether robot is currently in stair-descent mode.

        Returns:
            np.ndarray: Processed depth array (40, 64).
        """
        # ZERO IMPACT ON FLAT GROUND: If not descending stairs, bypass completely
        if not is_stair_down or depth_array is None or not isinstance(depth_array, np.ndarray):
            return depth_array

        h, w = depth_array.shape
        if h != 40 or w != 64:
            d_mapped = cv2.resize(depth_array, (64, 40), interpolation=cv2.INTER_LINEAR)
        else:
            d_mapped = depth_array.copy()

        # Front Wall / Close Obstacle Safety Check:
        # If the central corridor is blocked by a close wall/obstacle (mean < 0.45m or > 30% close pixels),
        # do NOT remap close pixels into a synthetic ramp! Preserve the real obstacle to allow braking.
        corridor_roi = d_mapped[self.min_row : self.max_row, self.left_col : self.right_col]
        valid_corridor = corridor_roi > 0.0
        corridor_mean = float(np.mean(corridor_roi[valid_corridor])) if np.any(valid_corridor) else 0.0
        corridor_close_ratio = float(np.mean((corridor_roi > 0.0) & (corridor_roi <= 0.35)))
        is_front_blocked = (corridor_close_ratio > 0.30) or (np.any(valid_corridor) and corridor_mean < 0.45)

        # 1. In central corridor, remap the stairwell abyss/void/steps into a continuous downward ramp
        # - Any pixel deeper than the virtual ramp (corridor > ramp_val) is a stair drop/abyss -> ramp_val
        # - Any pixel with 0.0 (far void/out-of-range from depth_callback) -> ramp_val
        # - Any isolated pixel <= blindzone_val on stair risers (only when front is NOT blocked) -> ramp_val
        # - Real obstacles closer than the ramp (blindzone_val < d <= ramp_val) are preserved 100%!
        for r in range(self.min_row, self.max_row):
            # Linearly interpolate ramp depth from row 38 (1.0m) to row 12 (2.2m)
            ramp_val = self.ramp_base + (38 - r) * self.ramp_step
            corridor = d_mapped[r, self.left_col : self.right_col]

            if is_front_blocked:
                # Front is blocked by real obstacle/wall: only remap far abyss or far void (0.0), NEVER close obstacles!
                stair_drop_mask = (corridor > ramp_val) | (corridor == 0.0)
            else:
                # Normal stair descent: remap deep abyss, void, and isolated stereo dropouts on step risers
                stair_drop_mask = (corridor > ramp_val) | (corridor == 0.0) | (corridor <= self.blindzone_val)

            d_mapped[r, self.left_col : self.right_col] = np.where(stair_drop_mask, ramp_val, corridor)

        # 2. Suppress excessive side railing repulsion without creating phantom walls:
        # Only clamp real close obstacles (0.0 < side < side_min_clamp) to side_min_clamp.
        # NEVER clamp 0.0 (open air/void) to side_min_clamp!
        # When front is blocked by a wall, do NOT clamp side columns so the entire wall is preserved.
        if not is_front_blocked:
            for col_slice in [slice(0, self.left_col), slice(self.right_col, w)]:
                side = d_mapped[:, col_slice]
                close_railing_mask = (side > 0.0) & (side < self.side_min_clamp)
                d_mapped[:, col_slice] = np.where(close_railing_mask, self.side_min_clamp, side)

        return d_mapped

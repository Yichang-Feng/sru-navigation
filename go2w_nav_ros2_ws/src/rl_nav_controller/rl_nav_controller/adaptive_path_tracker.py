"""Adaptive Path tracking, dynamic lookahead carrot point generation, and stall detection.

Combines:
1. Path curvature lookahead via forward sliding window analysis (handles sharp corners & prevents corner-cutting).
2. Local passage clearance lookahead with spatial latching (handles narrow doors without blindzone pop-out).
3. Slew-rate limiter to avoid abrupt lookahead distance jumps.
"""

from enum import Enum
import math
import numpy as np


class NavState(str, Enum):
    """Navigation states aligned with local_planner_status_diagram.html."""
    IDLE = "idle"
    RUNNING = "running"
    GOAL_REACHED = "goal_reached"
    FAILED = "failed"
    ERROR = "error"
    STOPPED = "stopped"

    # Backward compatibility aliases
    ARRIVED = "goal_reached"
    BLOCKED = "failed"
    STOP = "stopped"
    PAUSED = "stopped"


class PathSegmentTracker:
    """Manages tracking of a segmented 2D/3D path using lookahead carrot points."""

    def __init__(self, logger=None):
        self.logger = logger
        self.points = []  # list of np.ndarray [x, y, z]
        self.cum_dists = []  # cumulative 2D arc-length from start
        self.total_length = 0.0
        self.terminal_yaw = None
        self.current_progress = 0.0
        self.current_seg_idx = 0
        self.is_path_active = False
        self.frame_id = 'map'

    def set_path(self, points, terminal_yaw=None, frame_id='map', *args, **kwargs):
        """Set a new path segment."""
        if not points or len(points) == 0:
            self.clear()
            return False

        pts = []
        for p in points:
            if hasattr(p, "x") and hasattr(p, "y"):
                z = getattr(p, "z", 0.0)
                pts.append(np.array([float(p.x), float(p.y), float(z)], dtype=np.float64))
            else:
                z = float(p[2]) if len(p) > 2 else 0.0
                pts.append(np.array([float(p[0]), float(p[1]), z], dtype=np.float64))

        self.points = pts
        self.terminal_yaw = terminal_yaw
        self.frame_id = str(frame_id) if frame_id else 'map'

        # Compute cumulative 2D planar distances along the path
        self.cum_dists = [0.0]
        for i in range(1, len(self.points)):
            seg_dist = float(np.linalg.norm(self.points[i][:2] - self.points[i - 1][:2]))
            self.cum_dists.append(self.cum_dists[-1] + seg_dist)

        self.total_length = self.cum_dists[-1]
        self.current_progress = 0.0
        self.current_seg_idx = 0
        self.is_path_active = True
        return True

    def clear(self):
        """Clear the current path."""
        self.points = []
        self.cum_dists = []
        self.total_length = 0.0
        self.terminal_yaw = None
        self.current_progress = 0.0
        self.current_seg_idx = 0
        self.is_path_active = False
        self.frame_id = 'map'

    def is_active(self):
        """Check if path tracker is active with a valid path."""
        return self.is_path_active and len(self.points) > 0

    def has_path(self):
        """Check if path tracker is active with a valid path (alias for is_active)."""
        return self.is_active()

    def _project_onto_range(self, r_pos, start_idx, end_idx):
        """Project robot position onto polyline segments in [start_idx, end_idx).

        Returns:
            tuple: (best_dist_sq, best_progress, best_seg)
        """
        best_dist_sq = float("inf")
        best_progress = self.current_progress
        best_seg = self.current_seg_idx

        for i in range(start_idx, end_idx):
            p1 = self.points[i][:2]
            p2 = self.points[i + 1][:2]
            v = p2 - p1
            seg_len_sq = float(np.dot(v, v))

            if seg_len_sq < 1e-8:
                t = 0.0
                proj = p1
            else:
                t = float(np.dot(r_pos - p1, v) / seg_len_sq)
                t = max(0.0, min(1.0, t))
                proj = p1 + t * v

            d_sq = float(np.sum((r_pos - proj) ** 2))
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                s_start = self.cum_dists[i]
                s_end = self.cum_dists[i + 1]
                best_progress = s_start + t * (s_end - s_start)
                best_seg = i

        return best_dist_sq, best_progress, best_seg

    def realign_to_robot(self, robot_pos_xy):
        """Force a global nearest projection to realign path progress to robot pose."""
        return self.update_progress(robot_pos_xy, force_global=True)

    def update_progress(self, robot_pos_xy, force_global=False):
        """Project robot position onto the path polyline to update arc-length progress.

        Maintains a localized forward search window during continuous tracking to prevent
        skipping path loops, while adaptively executing a global nearest projection on cold-start,
        position discontinuities (e.g. manual joystick nudging / teleop), or explicit realign requests.
        """
        if not self.is_active():
            return 0.0

        if len(self.points) == 1:
            self.current_progress = 0.0
            return 0.0

        r_pos = np.array(robot_pos_xy[:2], dtype=np.float64)

        best_dist_sq = float("inf")
        best_progress = self.current_progress
        best_seg = self.current_seg_idx

        # 1. Primary search: Local forward progress window [progress - 0.5m, progress + 3.0m]
        # Strictly prevents skipping loops or jumping to adjacent future segments during continuous walking.
        start_idx = max(0, self.current_seg_idx - 1)
        hit_high_boundary = False
        best_t = 0.0
        last_evaluated_seg = start_idx

        if not force_global:
            low = max(0.0, self.current_progress - 0.50)
            high = self.current_progress + 3.0

            for i in range(start_idx, len(self.points) - 1):
                s_start = self.cum_dists[i]
                s_end = self.cum_dists[i + 1]

                if s_end < low:
                    continue
                if s_start > high:
                    # If best match was pressed up against the high cutoff while still approaching robot
                    if best_seg == last_evaluated_seg and best_t >= 0.95 and best_dist_sq > 0.04:
                        hit_high_boundary = True
                    break

                last_evaluated_seg = i

                p1 = self.points[i][:2]
                p2 = self.points[i + 1][:2]
                v = p2 - p1
                seg_len_sq = float(np.dot(v, v))

                if seg_len_sq < 1e-8:
                    t = 0.0
                    proj = p1
                else:
                    t = float(np.dot(r_pos - p1, v) / seg_len_sq)
                    t = max(0.0, min(1.0, t))
                    proj = p1 + t * v

                d_sq = float(np.sum((r_pos - proj) ** 2))
                if d_sq < best_dist_sq:
                    best_dist_sq = d_sq
                    best_progress = s_start + t * (s_end - s_start)
                    best_seg = i
                    best_t = t

        # 2. Desynchronization / Cold-start Deadlock Recovery:
        # Trigger global search across all polyline segments if:
        # a) force_global is True (e.g. Joy A resume or explicit relocalization)
        # b) cold start: current_progress == 0.0 and robot is > 0.5m away from start (best_dist_sq > 0.25)
        # c) position jump / local window deadlock: robot is > 1.0m away from local window (best_dist_sq > 1.0)
        # d) boundary cutoff: local search was stopped by high window while still projecting forward towards robot
        need_global = (
            force_global or
            (self.current_progress == 0.0 and best_dist_sq > 0.25) or
            hit_high_boundary or
            best_dist_sq > 1.0
        )

        if need_global:
            g_dist_sq, g_prog, g_seg = self._project_onto_range(r_pos, 0, len(self.points) - 1)
            # If global projection finds a segment closer than the local window (or local had no match)
            if g_dist_sq < best_dist_sq or force_global:
                best_dist_sq = g_dist_sq
                best_progress = g_prog
                best_seg = g_seg
                # When snapping globally due to discontinuity or cold start, allow progress to snap directly
                # (supporting both forward teleop and backward correction)
                self.current_seg_idx = best_seg
                self.current_progress = best_progress
                return self.current_progress

        self.current_seg_idx = best_seg
        self.current_progress = max(self.current_progress, best_progress)
        return self.current_progress

    def get_lookahead_point(self, lookahead_dist=2.0):
        """Compute the dynamic lookahead carrot point along the path."""
        if not self.is_active():
            return None, None

        if len(self.points) == 1:
            pt = self.points[0].tolist()
            return pt, self.terminal_yaw

        target_arc = self.current_progress + lookahead_dist

        # If lookahead reaches or exceeds terminal goal, clamp to terminal point
        if target_arc >= self.total_length:
            terminal_pt = self.points[-1].tolist()
            yaw = self.terminal_yaw
            if yaw is None:
                p_prev = self.points[-2]
                p_last = self.points[-1]
                yaw = float(np.arctan2(p_last[1] - p_prev[1], p_last[0] - p_prev[0]))
            return terminal_pt, yaw

        # Find the segment containing target_arc
        for i in range(len(self.points) - 1):
            s_start = self.cum_dists[i]
            s_end = self.cum_dists[i + 1]
            if s_start <= target_arc <= s_end:
                seg_len = s_end - s_start
                ratio = (target_arc - s_start) / seg_len if seg_len > 1e-6 else 0.0
                carrot_xy = self.points[i][:2] + ratio * (self.points[i + 1][:2] - self.points[i][:2])
                carrot_z = self.points[i][2] + ratio * (self.points[i + 1][2] - self.points[i][2])

                # Tangent yaw along segment
                seg_yaw = float(np.arctan2(
                    self.points[i + 1][1] - self.points[i][1],
                    self.points[i + 1][0] - self.points[i][0]
                ))
                return [float(carrot_xy[0]), float(carrot_xy[1]), float(carrot_z)], seg_yaw

        return self.points[-1].tolist(), self.terminal_yaw

    def check_arrived(self, robot_pos_xy, arrive_threshold=0.15):
        """Check if robot is within arrive_threshold of the terminal goal point.
        Includes path progression check to prevent premature triggers on loops or nearby segments.
        """
        if not self.is_active() or robot_pos_xy is None:
            return False

        r_pos = np.array(robot_pos_xy[:2], dtype=np.float64)
        term_pos = self.points[-1][:2]
        dist_to_end = float(np.linalg.norm(r_pos - term_pos))

        if dist_to_end > arrive_threshold:
            return False

        # Protect against false early triggers (loops, nearby start/waypoints):
        # Ensure robot is actually near the end of the path by checking remaining arc-length.
        if len(self.points) > 1 and self.total_length > 0.0:
            remaining_arc = self.total_length - self.current_progress
            if remaining_arc > max(0.40, arrive_threshold * 2.0):
                # Try evaluating progress at r_pos in case caller didn't call update_progress first
                test_progress = self.update_progress(robot_pos_xy)
                remaining_arc = self.total_length - test_progress
                if remaining_arc > max(0.40, arrive_threshold * 2.0):
                    return False

        return True

    def get_distance_to_terminal(self, robot_pos_xy):
        """Get distance from robot to terminal goal point in meters."""
        if not self.is_active() or robot_pos_xy is None:
            return float("inf")
        r_pos = np.array(robot_pos_xy[:2], dtype=np.float64)
        return float(np.linalg.norm(r_pos - self.points[-1][:2]))

    def get_terminal_pose(self):
        """Return (terminal_pt, terminal_yaw) if active, else (None, None)."""
        if not self.is_active() or len(self.points) == 0:
            return None, None
        return self.points[-1].tolist(), self.terminal_yaw


class AdaptivePathSegmentTracker(PathSegmentTracker):
    """Extended PathSegmentTracker with dual-factor adaptive lookahead:
    1. Forward sliding window curvature adaptation (handles sharp turns & corners)
    2. Local passage clearance detection with spatial latching (handles doors/narrowings)
    3. Slew-rate limiting for smooth lookahead transitions.
    """

    def __init__(
        self,
        logger=None,
        lookahead_min=0.5,
        lookahead_max=2.0,
        window_dist=1.6,
        turn_angle_low_deg=20.0,
        turn_angle_high_deg=50.0,
        slew_rate_decrease=2.5,
        slew_rate_increase=0.8,
        passage_latch_dist=1.2,
        clearance_threshold=1.05,
        adaptive_mode="both"
    ):
        super().__init__(logger=logger)
        self.lookahead_min = float(lookahead_min)
        self.lookahead_max = float(lookahead_max)
        self.window_dist = float(window_dist)
        self.turn_angle_low = math.radians(turn_angle_low_deg)
        self.turn_angle_high = math.radians(turn_angle_high_deg)
        self.slew_rate_decrease = float(slew_rate_decrease)  # m/s (quick contraction)
        self.slew_rate_increase = float(slew_rate_increase)  # m/s (gradual expansion)
        self.passage_latch_dist = float(passage_latch_dist)
        self.clearance_threshold = float(clearance_threshold)
        self.adaptive_mode = self._normalize_adaptive_mode(adaptive_mode)

        # Dynamic state
        self.current_lookahead = self.lookahead_max
        self.last_update_time = None
        self.is_in_narrow_passage = False
        self.narrow_passage_entry_s = -float("inf")

    @staticmethod
    def _normalize_adaptive_mode(mode):
        """Normalize mode string: 'both' | 'curvature_only' | 'clearance_only' | 'none'."""
        if mode is None:
            return "both"
        m = str(mode).strip().lower()
        if m in ("curvature_only", "curv_only", "curv", "curvature", "path_only"):
            return "curvature_only"
        elif m in ("clearance_only", "door_only", "clear_only", "clearance", "door"):
            return "clearance_only"
        elif m in ("none", "disabled", "off", "false", "fixed"):
            return "none"
        return "both"

    def set_adaptive_mode(self, mode):
        """Dynamically update adaptive mode."""
        self.adaptive_mode = self._normalize_adaptive_mode(mode)

    def reset_adaptive_state(self, initial_lookahead=None):
        """Reset internal adaptive lookahead state."""
        self.current_lookahead = initial_lookahead if initial_lookahead is not None else self.lookahead_max
        self.last_update_time = None
        self.is_in_narrow_passage = False
        self.narrow_passage_entry_s = -float("inf")

    def set_path(self, points, terminal_yaw=None, frame_id='map', *args, **kwargs):
        ret = super().set_path(points, terminal_yaw=terminal_yaw, frame_id=frame_id, *args, **kwargs)
        self.reset_adaptive_state()
        return ret

    def clear(self):
        super().clear()
        self.reset_adaptive_state()

    @staticmethod
    def _normalize_angle_diff(a1, a2):
        """Compute minimum angular difference in [-pi, pi]."""
        diff = (a1 - a2 + math.pi) % (2.0 * math.pi) - math.pi
        return diff

    def compute_path_curvature_lookahead(self, from_progress, window_dist=None):
        """Analyze accumulated direction changes along future path in [from_progress, from_progress + window_dist].

        Returns:
            tuple: (l_curv, acc_turn_deg, max_seg_turn_deg)
        """
        if not self.is_active() or len(self.points) < 2:
            return self.lookahead_max, 0.0, 0.0

        if window_dist is None:
            window_dist = self.window_dist

        target_arc_end = from_progress + window_dist
        if from_progress >= self.total_length:
            return self.lookahead_min, 0.0, 0.0

        # Collect segment heading vectors along the future window
        headings = []
        seg_weights = []

        for i in range(len(self.points) - 1):
            s_start = self.cum_dists[i]
            s_end = self.cum_dists[i + 1]

            overlap_start = max(s_start, from_progress)
            overlap_end = min(s_end, target_arc_end)

            if overlap_end > overlap_start:
                dx = self.points[i + 1][0] - self.points[i][0]
                dy = self.points[i + 1][1] - self.points[i][1]
                seg_yaw = math.atan2(dy, dx)
                headings.append(seg_yaw)
                seg_weights.append(overlap_end - overlap_start)

        if len(headings) <= 1:
            return self.lookahead_max, 0.0, 0.0

        # Net macro heading change between start and end of the evaluation window
        net_turn_rad = abs(self._normalize_angle_diff(headings[-1], headings[0]))

        # Filter out micro-discretization zigzag noise:
        # Only accumulate individual segment bends if they exceed 5 degrees
        acc_turn_rad = 0.0
        max_seg_turn_rad = 0.0
        for i in range(len(headings) - 1):
            d_theta = abs(self._normalize_angle_diff(headings[i + 1], headings[i]))
            max_seg_turn_rad = max(max_seg_turn_rad, d_theta)
            if d_theta > math.radians(5.0):
                acc_turn_rad += d_theta

        # Combine net turn and significant accumulated turns:
        # Prevents discrete segment jitter from artificially blowing up turn angle on straight corridors
        effective_turn = max(net_turn_rad, min(acc_turn_rad, net_turn_rad * 1.5))

        if effective_turn <= self.turn_angle_low:
            l_curv = self.lookahead_max
        elif effective_turn >= self.turn_angle_high:
            l_curv = self.lookahead_min
        else:
            ratio = (effective_turn - self.turn_angle_low) / (self.turn_angle_high - self.turn_angle_low)
            smooth_weight = 0.5 * (1.0 + math.cos(math.pi * ratio))
            l_curv = self.lookahead_min + (self.lookahead_max - self.lookahead_min) * smooth_weight

        return float(l_curv), math.degrees(effective_turn), math.degrees(max_seg_turn_rad)

    def evaluate_passage_clearance(self, depth_image=None, min_depth=0.25, max_depth=2.2):
        """Analyze local passage clearance width from depth image with spatial latching.

        Returns:
            tuple: (l_clear, clearance_w, is_narrow)
        """
        if depth_image is None or not isinstance(depth_image, np.ndarray):
            if self.is_in_narrow_passage:
                traveled = self.current_progress - self.narrow_passage_entry_s
                if traveled < self.passage_latch_dist:
                    return self.lookahead_min, 0.8, True
                else:
                    self.is_in_narrow_passage = False
            return self.lookahead_max, 2.5, False

        # If already latched in narrow passage, check progress
        if self.is_in_narrow_passage:
            traveled = self.current_progress - self.narrow_passage_entry_s
            if traveled < self.passage_latch_dist:
                return self.lookahead_min, 0.8, True
            else:
                self.is_in_narrow_passage = False

        h, w = depth_image.shape
        mid_row_start = int(h * 0.35)
        mid_row_end = int(h * 0.70)
        roi = depth_image[mid_row_start:mid_row_end, :]

        col_depths = np.nanmedian(roi, axis=0)
        valid_mask = (col_depths >= min_depth) & (col_depths <= max_depth) & np.isfinite(col_depths)

        left_valid = valid_mask[: w // 3]
        right_valid = valid_mask[2 * w // 3 :]
        center_valid = valid_mask[w // 3 : 2 * w // 3]

        left_near = np.any(left_valid) and (np.nanmin(col_depths[: w // 3]) < 1.4)
        right_near = np.any(right_valid) and (np.nanmin(col_depths[2 * w // 3 :]) < 1.4)

        center_depth = np.nanmedian(col_depths[w // 3 : 2 * w // 3]) if np.any(center_valid) else 0.0
        left_depth = np.nanmin(col_depths[: w // 3]) if np.any(left_valid) else 99.0
        right_depth = np.nanmin(col_depths[2 * w // 3 :]) if np.any(right_valid) else 99.0

        is_narrow_door = False
        est_clearance = 2.5

        if left_near and right_near and (center_depth > min(left_depth, right_depth) + 0.25 or center_depth == 0.0):
            est_clearance = 0.85
            is_narrow_door = True
        elif (left_near or right_near) and center_depth < 0.9 and center_depth > min_depth:
            est_clearance = 1.0
            is_narrow_door = True

        if is_narrow_door or (est_clearance < self.clearance_threshold):
            self.is_in_narrow_passage = True
            self.narrow_passage_entry_s = self.current_progress
            l_clear = self.lookahead_min
        else:
            l_clear = self.lookahead_max

        return float(l_clear), float(est_clearance), self.is_in_narrow_passage

    def get_adaptive_lookahead_point(self, current_time=None, depth_image=None):
        """Compute the dynamic lookahead carrot point combining curvature, clearance, and slew rate.

        Returns:
            tuple: (carrot_pt_3d, carrot_yaw, debug_info)
        """
        if not self.is_active():
            return None, None, {}

        # 1. Compute curvature-based factor (if enabled by mode)
        if self.adaptive_mode in ("both", "curvature_only"):
            l_curv, turn_deg, max_seg_turn_deg = self.compute_path_curvature_lookahead(
                from_progress=self.current_progress,
                window_dist=self.window_dist
            )
        else:
            l_curv, turn_deg, max_seg_turn_deg = self.lookahead_max, 0.0, 0.0

        # 2. Compute clearance/door-based factor (if enabled by mode)
        if self.adaptive_mode in ("both", "clearance_only"):
            l_clear, clearance_w, in_narrow = self.evaluate_passage_clearance(depth_image=depth_image)
        else:
            self.is_in_narrow_passage = False
            l_clear, clearance_w, in_narrow = self.lookahead_max, 2.5, False

        # 3. Target fusion based on active mode
        if self.adaptive_mode == "none":
            l_target = self.lookahead_max
        elif self.adaptive_mode == "curvature_only":
            l_target = l_curv
        elif self.adaptive_mode == "clearance_only":
            l_target = l_clear
        else:
            l_target = min(l_curv, l_clear)

        # 4. Slew-rate limiting
        if current_time is not None and self.last_update_time is not None:
            dt = max(0.001, min(0.5, current_time - self.last_update_time))
            if l_target < self.current_lookahead:
                max_step = self.slew_rate_decrease * dt
                self.current_lookahead = max(l_target, self.current_lookahead - max_step)
            else:
                max_step = self.slew_rate_increase * dt
                self.current_lookahead = min(l_target, self.current_lookahead + max_step)
        else:
            self.current_lookahead = l_target

        self.last_update_time = current_time

        # Clamping to valid range
        self.current_lookahead = max(self.lookahead_min, min(self.lookahead_max, self.current_lookahead))

        # 5. Extract point using smoothed lookahead
        carrot_pt, carrot_yaw = self.get_lookahead_point(lookahead_dist=self.current_lookahead)

        debug_info = {
            "mode": str(self.adaptive_mode),
            "lookahead": float(self.current_lookahead),
            "l_target": float(l_target),
            "l_curv": float(l_curv),
            "l_clear": float(l_clear),
            "turn_deg": float(turn_deg),
            "in_narrow": bool(in_narrow),
            "clearance_w": float(clearance_w),
            "progress_s": float(self.current_progress),
            "total_length": float(self.total_length),
        }

        return carrot_pt, carrot_yaw, debug_info


class BlockedDetector:
    """Detects when the robot is blocked / stalled against obstacles."""

    def __init__(self, timeout=4.0, dist_threshold=0.1, vel_threshold=0.1):
        self.timeout = float(timeout)
        self.dist_threshold = float(dist_threshold)
        self.vel_threshold = float(vel_threshold)

        self.window_start_time = None
        self.start_progress = 0.0

    def reset(self, current_time=None, progress_s=0.0):
        self.window_start_time = current_time
        self.start_progress = progress_s

    def update(self, current_time, progress_s, robot_vel_xy):
        if current_time is None:
            return False

        if self.window_start_time is None:
            self.window_start_time = current_time
            self.start_progress = progress_s
            return False

        v_mag = float(np.linalg.norm(robot_vel_xy[:2])) if robot_vel_xy is not None else 0.0
        delta_s = abs(progress_s - self.start_progress)

        if delta_s >= self.dist_threshold or v_mag >= self.vel_threshold:
            self.window_start_time = current_time
            self.start_progress = progress_s
            return False

        elapsed = current_time - self.window_start_time
        if elapsed >= self.timeout:
            return True

        return False

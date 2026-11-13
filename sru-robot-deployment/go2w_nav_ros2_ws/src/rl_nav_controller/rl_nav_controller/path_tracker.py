"""Path tracking, lookahead carrot point generation, and stall detection for global navigation integration."""

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
    """Manages tracking of a segmented 2D/3D path using dynamic lookahead carrot points."""

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
        """Set a new path segment.

        Args:
            points: List of [x, y, z] coordinates or Pose objects.
            terminal_yaw: Optional terminal heading in radians.
            frame_id: Coordinate frame for path tracking (default 'map').

        Returns:
            bool: True if path was successfully set.
        """
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

    def update_progress(self, robot_pos_xy):
        """Project robot position onto the path polyline to update arc-length progress.

        Args:
            robot_pos_xy: Robot [x, y] in the same frame as path.

        Returns:
            float: Current cumulative arc-length progress along path.
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

        # Search from max(0, current_seg_idx - 1) to end to enforce monotonic forward progress
        start_idx = max(0, self.current_seg_idx - 1)
        for i in range(start_idx, len(self.points) - 1):
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
                seg_len = self.cum_dists[i + 1] - self.cum_dists[i]
                best_progress = self.cum_dists[i] + t * seg_len
                best_seg = i

        self.current_seg_idx = best_seg
        self.current_progress = max(self.current_progress, best_progress)
        return self.current_progress

    def realign_to_robot(self, robot_pos_xy):
        """Realign progress to robot's current position without monotonic restriction."""
        if not self.is_active() or robot_pos_xy is None:
            return 0.0

        r_pos = np.array(robot_pos_xy[:2], dtype=np.float64)
        best_dist_sq = float("inf")
        best_progress = self.current_progress
        best_seg = self.current_seg_idx

        for i in range(len(self.points) - 1):
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
                seg_len = self.cum_dists[i + 1] - self.cum_dists[i]
                best_progress = self.cum_dists[i] + t * seg_len
                best_seg = i

        self.current_seg_idx = best_seg
        self.current_progress = best_progress
        return self.current_progress

    def get_lookahead_point(self, lookahead_dist=2.0):
        """Compute the dynamic lookahead carrot point along the path."""
        if not self.is_active():
            return None, None

        if len(self.points) == 1:
            pt = self.points[0].tolist()
            return pt, self.terminal_yaw

        target_arc = self.current_progress + lookahead_dist

        if target_arc >= self.total_length:
            terminal_pt = self.points[-1].tolist()
            yaw = self.terminal_yaw
            if yaw is None:
                p_prev = self.points[-2]
                p_last = self.points[-1]
                yaw = float(np.arctan2(p_last[1] - p_prev[1], p_last[0] - p_prev[0]))
            return terminal_pt, yaw

        for i in range(len(self.points) - 1):
            s_start = self.cum_dists[i]
            s_end = self.cum_dists[i + 1]
            if s_start <= target_arc <= s_end:
                seg_len = s_end - s_start
                ratio = (target_arc - s_start) / seg_len if seg_len > 1e-6 else 0.0
                carrot_xy = self.points[i][:2] + ratio * (self.points[i + 1][:2] - self.points[i][:2])
                carrot_z = self.points[i][2] + ratio * (self.points[i + 1][2] - self.points[i][2])
                seg_yaw = float(np.arctan2(
                    self.points[i + 1][1] - self.points[i][1],
                    self.points[i + 1][0] - self.points[i][0]
                ))
                return [float(carrot_xy[0]), float(carrot_xy[1]), float(carrot_z)], seg_yaw

        return self.points[-1].tolist(), self.terminal_yaw

    def get_terminal_pose(self):
        """Get terminal goal position and orientation yaw."""
        if not self.is_active() or len(self.points) == 0:
            return None, None
        return self.points[-1].tolist(), self.terminal_yaw

    def check_arrived(self, robot_pos_xy, arrive_threshold=0.20):
        """Check if robot is within arrive_threshold of the terminal goal point."""
        if not self.is_active() or robot_pos_xy is None:
            return False

        r_pos = np.array(robot_pos_xy[:2], dtype=np.float64)
        term_pos = self.points[-1][:2]
        dist_to_end = float(np.linalg.norm(r_pos - term_pos))
        return dist_to_end <= arrive_threshold

    def get_distance_to_terminal(self, robot_pos_xy):
        """Get distance from robot to terminal goal point in meters."""
        if not self.is_active() or robot_pos_xy is None:
            return float("inf")
        r_pos = np.array(robot_pos_xy[:2], dtype=np.float64)
        return float(np.linalg.norm(r_pos - self.points[-1][:2]))


class BlockedDetector:
    """Detects when the robot is blocked / stalled against obstacles."""

    def __init__(self, timeout=4.0, dist_threshold=0.1, vel_threshold=0.1):
        self.timeout = float(timeout)
        self.dist_threshold = float(dist_threshold)
        self.vel_threshold = float(vel_threshold)

        self.window_start_time = None
        self.start_progress = 0.0

    def reset(self, current_time=None, progress_s=0.0):
        """Reset stall timer and reference progress."""
        self.window_start_time = current_time
        self.start_progress = progress_s

    def update(self, current_time, progress_s, robot_vel_xy):
        """Check if robot has been stalled for more than timeout."""
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

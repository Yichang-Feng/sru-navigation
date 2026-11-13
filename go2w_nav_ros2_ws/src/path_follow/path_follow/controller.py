"""ROS-independent forward-only polyline tracking controller."""

from dataclasses import dataclass
import math


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class ControllerConfig:
    linear_vel: float = 0.30
    max_linear_vel: float = 0.50
    max_angular_vel: float = 1.0
    lookahead_distance: float = 0.50
    angular_kp: float = 1.5
    goal_tolerance: float = 0.20
    goal_height_tolerance: float = 0.40
    slow_down_angle: float = 0.60
    rotate_in_place_angle: float = 0.80
    max_linear_accel: float = 0.50
    max_angular_accel: float = 1.50
    max_tracking_error: float = 1.0
    progress_search_distance: float = 3.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)
        if self.linear_vel > self.max_linear_vel:
            raise ValueError('linear_vel must not exceed max_linear_vel')
        if not 0 < self.slow_down_angle < self.rotate_in_place_angle < math.pi:
            raise ValueError('require 0 < slow_down_angle < rotate_in_place_angle < pi')


class PathController:
    """Project onto segments and interpolate a lookahead target by path arc length."""

    def __init__(self, config):
        self.config = config
        self.points = []
        self.arcs = []
        self.progress = None
        self.velocity = (0.0, 0.0)

    def set_path(self, points):
        cleaned = []
        for point in points:
            if len(point) != 3 or not all(math.isfinite(v) for v in point):
                raise ValueError('invalid path position')
            if not cleaned or math.dist(cleaned[-1], point) > 1e-8:
                cleaned.append(tuple(point))
        self.points = cleaned
        self.arcs = [0.0] if cleaned else []
        for a, b in zip(cleaned, cleaned[1:]):
            arc = self.arcs[-1] + math.hypot(b[0]-a[0], b[1]-a[1])
            if not math.isfinite(arc):
                raise ValueError('path length overflow')
            self.arcs.append(arc)
        self.progress = None

    def stop(self):
        self.velocity = (0.0, 0.0)
        return 0.0, 0.0

    def reset_progress(self):
        self.progress = None

    def _sample(self, arc):
        for i in range(len(self.points)-1):
            if self.arcs[i+1] >= arc:
                length = self.arcs[i+1] - self.arcs[i]
                t = clamp((arc-self.arcs[i])/length, 0.0, 1.0) if length > 1e-8 else 0.0
                return tuple(a+t*(b-a) for a, b in zip(self.points[i], self.points[i+1]))
        return self.points[-1]

    def compute(self, robot, yaw, dt):
        c = self.config
        if not self.points or not all(math.isfinite(v) for v in (*robot, yaw, dt)) or dt <= 0:
            return (*self.stop(), 'invalid input')
        best = None
        low = max(0.0, self.progress-0.10) if self.progress is not None else 0.0
        high = self.progress+c.progress_search_distance if self.progress is not None else float('inf')
        if len(self.points) == 1:
            best = (math.dist(robot, self.points[0]), 0.0, self.points[0])
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            length = self.arcs[i+1]-self.arcs[i]
            if length < 1e-8 or self.arcs[i+1] < low or self.arcs[i] > high:
                continue
            t = ((robot[0]-a[0])*(b[0]-a[0])+(robot[1]-a[1])*(b[1]-a[1]))/(length*length)
            t = clamp(t, max(0.0, (low-self.arcs[i])/length), min(1.0, (high-self.arcs[i])/length))
            p = tuple(x+t*(y-x) for x, y in zip(a, b))
            candidate = (math.dist(robot, p), self.arcs[i]+t*length, p)
            if best is None or candidate[0] < best[0]-1e-8:
                best = candidate
        if best is None or best[0] > c.max_tracking_error:
            return (*self.stop(), 'tracking error')
        self.progress = max(self.progress or 0.0, best[1])
        remaining = max(0.0, self.arcs[-1]-self.progress)
        goal = self.points[-1]
        goal_distance = math.hypot(goal[0]-robot[0], goal[1]-robot[1])
        if (remaining <= c.goal_tolerance and goal_distance <= c.goal_tolerance and
                abs(goal[2]-robot[2]) <= c.goal_height_tolerance):
            return (*self.stop(), 'goal reached')
        if abs(best[2][2]-robot[2]) > c.goal_height_tolerance:
            return (*self.stop(), 'path height mismatch')
        target = self._sample(min(self.arcs[-1], self.progress+c.lookahead_distance))
        dx, dy = target[0]-robot[0], target[1]-robot[1]
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return (*self.stop(), 'no forward target')
        error = wrap_angle(math.atan2(dy, dx)-yaw)
        if abs(error) >= c.rotate_in_place_angle:
            v = 0.0
            w = clamp(c.angular_kp*error, -c.max_angular_vel, c.max_angular_vel)
        else:
            # Brake towards the endpoint. Pure pursuit curvature limits turn speed.
            speed_distance = max(remaining, goal_distance)
            v = min(c.linear_vel, math.sqrt(2*c.max_linear_accel*speed_distance))
            if abs(error) > c.slow_down_angle:
                v *= (c.rotate_in_place_angle-abs(error))/(c.rotate_in_place_angle-c.slow_down_angle)
            curvature = 2*math.sin(error)/distance
            if abs(curvature) > 1e-8:
                v = min(v, c.max_angular_vel/abs(curvature))
            w = clamp(v*curvature, -c.max_angular_vel, c.max_angular_vel)
        dt = min(dt, 0.10)
        old_v, old_w = self.velocity
        # Geometry/endpoint constraints may reduce speed immediately; only forward
        # acceleration is ramped. Recompute steering for the speed actually sent.
        rotating = abs(error) >= c.rotate_in_place_angle
        v = 0.0 if rotating else min(v, old_v+c.max_linear_accel*dt)
        if not rotating:
            w = clamp(v*curvature, -c.max_angular_vel, c.max_angular_vel)
        w = clamp(w, old_w-c.max_angular_accel*dt, old_w+c.max_angular_accel*dt)
        if not rotating and abs(curvature) > 1e-8:
            # If angular acceleration lags the requested curvature, slow forward
            # motion too, rather than cutting across the reference curve.
            v = min(v, abs(w/curvature)) if w*curvature > 0 else 0.0
        self.velocity = (clamp(v, 0.0, c.max_linear_vel), clamp(w, -c.max_angular_vel, c.max_angular_vel))
        return (*self.velocity, 'tracking')

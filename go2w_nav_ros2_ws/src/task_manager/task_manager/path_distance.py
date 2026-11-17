"""Remaining 3-D polyline length with a continuity-constrained projection."""

import bisect
import math


def valid_point(point):
    return len(point) == 3 and all(math.isfinite(v) for v in point)


class RemainingPath:
    def __init__(self):
        self.clear()

    def clear(self):
        self.points, self.arcs = [], []
        self.progress = self.robot = None

    def set_path(self, points):
        self.clear()
        for point in points:
            if not valid_point(point):
                self.clear()
                return False
            if not self.points or math.dist(point, self.points[-1]) > 1e-8:
                self.points.append(tuple(point))
        if not self.points:
            return False
        self.arcs = [0.0]
        for a, b in zip(self.points, self.points[1:]):
            self.arcs.append(self.arcs[-1] + math.dist(a, b))
        if not math.isfinite(self.arcs[-1]):
            self.clear()
            return False
        return True

    def remaining(self, robot):
        if not self.points or not valid_point(robot):
            return None
        if len(self.points) == 1:
            distance = math.dist(robot, self.points[0])
            return distance if math.isfinite(distance) else None
        # Search near the previous arc to avoid jumping to the far end of a
        # crossing/loop. Expand for distance actually travelled between updates.
        window = max(3.0, 2.0 * math.dist(robot, self.robot)) if self.robot else 3.0
        low = max(0.0, self.progress - window) if self.progress is not None else 0.0
        high = (min(self.arcs[-1], self.progress + window)
                if self.progress is not None else self.arcs[-1])
        best = None
        first = max(0, bisect.bisect_left(self.arcs, low) - 1)
        for index in range(first, len(self.points) - 1):
            if self.arcs[index] > high:
                break
            a, b = self.points[index:index + 2]
            length = self.arcs[index + 1] - self.arcs[index]
            lower = max(0.0, (low - self.arcs[index]) / length)
            upper = min(1.0, (high - self.arcs[index]) / length)
            fraction = sum(((p - x) / length) * ((y - x) / length)
                           for p, x, y in zip(robot, a, b))
            fraction = max(lower, min(upper, fraction))
            projected = tuple(x + fraction * (y - x) for x, y in zip(a, b))
            candidate = math.dist(robot, projected), self.arcs[index] + fraction * length
            if best is None or candidate[0] < best[0] - 1e-8:
                best = candidate
        if best is None or not all(math.isfinite(v) for v in best):
            return None
        self.robot, self.progress = tuple(robot), best[1]
        remaining = max(0.0, best[0] + (self.arcs[-1] - best[1]))
        return remaining if math.isfinite(remaining) else None

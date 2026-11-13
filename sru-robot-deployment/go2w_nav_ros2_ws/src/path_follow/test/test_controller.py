"""Behavioral tests without ROS or a robot."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from path_follow.controller import ControllerConfig, PathController  # noqa: E402


def controller(points, **kwargs):
    instance = PathController(ControllerConfig(**kwargs))
    instance.set_path(points)
    return instance


def test_sparse_path_projection_and_interpolation():
    c = controller([(0, 0, 0.5), (10, 0, 0.5)])
    v, w, status = c.compute((4, 0, 0.5), 0, 0.05)
    assert v > 0 and w == 0 and status == 'tracking'
    assert c.progress == pytest.approx(4)
    assert c._sample(4.5) == pytest.approx((4.5, 0, 0.5))


@pytest.mark.parametrize('offset,sign', [(0.3, -1), (-0.3, 1)])
def test_correct_turn_direction(offset, sign):
    c = controller([(0, 0, 0.5), (4, 0, 0.5)])
    _, w, _ = c.compute((0, offset, 0.5), 0, 0.05)
    assert w*sign > 0


def test_turn_in_place_and_wraparound():
    c = controller([(0, 0, 0.5), (4, 0, 0.5)])
    v, w, _ = c.compute((0, 0, 0.5), math.pi/2, 0.05)
    assert v == 0 and w < 0
    c = controller([(0, 0, 0.5), (-4, -0.01, 0.5)])
    v, w, _ = c.compute((0, 0, 0.5), math.pi-0.01, 0.05)
    assert v > 0 and 0 < w < 0.1


def test_goal_and_height_mismatch():
    c = controller([(0, 0, 0.5), (1, 0, 0.5)])
    assert c.compute((0.9, 0, 0.5), 0, 0.05) == (0, 0, 'goal reached')
    c = controller([(0, 0, 0.5), (4, 0, 0.5)])
    assert c.compute((0, 0, 1.0), 0, 0.05) == (0, 0, 'path height mismatch')


def test_closed_loop_does_not_finish_at_start():
    c = controller([(0, 0, 0.5), (3, 0, 0.5), (3, 3, 0.5), (0, 3, 0.5), (0, 0, 0.5)])
    v, _, status = c.compute((0, 0, 0.5), 0, 0.05)
    assert status == 'tracking' and v > 0
    assert c.progress == 0


def test_progress_does_not_jump_at_intersection():
    c = controller([(0, 0, 0.5), (4, 0, 0.5), (4, 4, 0.5),
                    (2, -2, 0.5), (0, 0, 0.5)])
    c.compute((0.1, 0, 0.5), 0, 0.05)
    c.compute((2, 0.01, 0.5), 0, 0.05)
    assert c.progress < 4


def test_speed_and_acceleration_limits_and_immediate_stop():
    c = controller([(0, 0, 0.5), (100, 0, 0.5)])
    last_v = last_w = 0
    for _ in range(100):
        v, w, _ = c.compute((0, 0.1, 0.5), 0, 0.05)
        assert 0 <= v <= c.config.max_linear_vel
        assert abs(w) <= c.config.max_angular_vel
        assert abs(v-last_v) <= c.config.max_linear_accel*0.05+1e-8
        assert abs(w-last_w) <= c.config.max_angular_accel*0.05+1e-8
        last_v, last_w = v, w
    assert c.stop() == (0, 0)
    assert c.velocity == (0, 0)


def test_endpoint_deceleration():
    c = controller([(0, 0, 0.5), (1, 0, 0.5)], goal_tolerance=0.01)
    c.velocity = (0.3, 0)
    v, _, _ = c.compute((0.98, 0, 0.5), 0, 0.1)
    assert v < 0.3


def test_empty_duplicate_single_point_and_invalid_values():
    c = controller([])
    assert c.compute((0, 0, 0), 0, 0.05)[:2] == (0, 0)
    c.set_path([(0, 0, 0.5), (0, 0, 0.5), (1, 0, 0.5)])
    assert len(c.points) == 2
    c.set_path([(0.5, 0, 0.5)])
    assert c.compute((0, 0, 0.5), 0, 0.05)[0] > 0
    assert c.compute((0, 0, 0.5), float('nan'), 0.05)[:2] == (0, 0)
    with pytest.raises(ValueError):
        c.set_path([(float('nan'), 0, 0)])
    with pytest.raises(ValueError):
        ControllerConfig(lookahead_distance=0)
    with pytest.raises(ValueError):
        ControllerConfig(rotate_in_place_angle=0.3)


def test_tracking_error_stops():
    c = controller([(0, 0, 0.5), (4, 0, 0.5)])
    assert c.compute((0, 2, 0.5), 0, 0.05) == (0, 0, 'tracking error')


def test_abrupt_curve_does_not_outpace_steering():
    c = controller([(0, 0, 0.5), (4, 0, 0.5)])
    c.velocity = (0.30, 0.0)
    v, w, _ = c.compute((0, 0.3, 0.5), 0, 0.05)
    curvature = 2*math.sin(math.atan2(-0.3, 0.5))/math.hypot(0.3, 0.5)
    assert w < 0
    assert v*abs(curvature) <= abs(w)+1e-8
    # Suppress forward motion while angular acceleration reverses a wrong turn.
    c.velocity = (0.30, 0.5)
    v, w, _ = c.compute((0, 0.3, 0.5), 0, 0.05)
    assert w > 0 and v == 0

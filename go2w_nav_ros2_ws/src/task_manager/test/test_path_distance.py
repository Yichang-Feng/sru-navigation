"""Geometric distance along a path, independent of ROS."""

import math

import pytest

from task_manager.path_distance import RemainingPath


def test_bend_uses_remaining_polyline_instead_of_endpoint_chord():
    path = RemainingPath()
    assert path.set_path([(0, 0, 0), (3, 0, 0), (3, 4, 0)])
    assert path.remaining((1, 0, 0)) == pytest.approx(6)
    assert path.remaining((3, 2, 0)) == pytest.approx(2)
    assert path.remaining((3, 4, 0)) == pytest.approx(0)


def test_offset_adds_connection_to_projected_segment():
    path = RemainingPath()
    path.set_path([(0, 0, 0), (10, 0, 0)])
    assert path.remaining((4, 2, 0)) == pytest.approx(8)


def test_three_dimensions_and_repeated_vertices():
    path = RemainingPath()
    path.set_path([(0, 0, 0), (0, 0, 0), (3, 0, 4)])
    assert path.remaining((1.5, 0, 2)) == pytest.approx(2.5)


def test_single_point_and_empty_path():
    path = RemainingPath()
    assert path.remaining((0, 0, 0)) is None
    assert not path.set_path([])
    path.set_path([(1, 2, 2), (1, 2, 2)])
    assert path.remaining((0, 0, 0)) == pytest.approx(3)


def test_invalid_path_clears_old_route_and_invalid_robot_is_unknown():
    path = RemainingPath()
    path.set_path([(0, 0, 0), (1, 0, 0)])
    assert not path.set_path([(math.nan, 0, 0)])
    assert path.remaining((0, 0, 0)) is None
    path.set_path([(0, 0, 0), (1, 0, 0)])
    assert path.remaining((math.inf, 0, 0)) is None


def test_crossing_retains_local_progress_and_new_path_resets_it():
    path = RemainingPath()
    points = [(-1, 0, 0), (10, 0, 0), (10, 10, 0), (0, 10, 0), (0, -1, 0)]
    path.set_path(points)
    assert path.remaining((-.5, 0, 0)) == pytest.approx(41.5)
    assert path.remaining((0, 0, 0)) == pytest.approx(41)
    path.set_path([(0, 0, 0), (0, 2, 0)])
    assert path.remaining((0, 1, 0)) == pytest.approx(1)

"""src/environment.py icin testler."""

import math
import random

import pytest

from src.environment import Environment, Obstacle

BOUNDS = (0.0, 0.0, 100.0, 60.0)


class TestObstacle:
    def test_fields_are_positional_in_order(self):
        obs = Obstacle(3.0, -4.0, 2.5)
        assert obs.x == 3.0
        assert obs.y == -4.0
        assert obs.radius == 2.5

    def test_is_frozen(self):
        obs = Obstacle(0.0, 0.0, 1.0)
        with pytest.raises(Exception):
            obs.radius = 5.0

    @pytest.mark.parametrize("bad_radius", [0.0, -1.0, -0.001])
    def test_nonpositive_radius_raises(self, bad_radius):
        with pytest.raises(ValueError):
            Obstacle(0.0, 0.0, bad_radius)

    def test_contains_center(self):
        assert Obstacle(5.0, 5.0, 2.0).contains((5.0, 5.0)) is True

    def test_contains_inside(self):
        assert Obstacle(0.0, 0.0, 2.0).contains((1.0, 1.0)) is True

    def test_contains_outside(self):
        assert Obstacle(0.0, 0.0, 2.0).contains((3.0, 0.0)) is False

    def test_boundary_counts_as_inside(self):
        # tam cember uzerinde: muhafazakar taraf carpisma sayar
        assert Obstacle(0.0, 0.0, 2.0).contains((2.0, 0.0)) is True

    def test_clearance_inflates(self):
        obs = Obstacle(0.0, 0.0, 2.0)
        assert obs.contains((3.0, 0.0)) is False
        assert obs.contains((3.0, 0.0), clearance=1.5) is True

    def test_accepts_pose_ignoring_yaw(self):
        # Pose verilirse ilk iki bileseni kullanilir
        obs = Obstacle(0.0, 0.0, 2.0)
        assert obs.contains((1.0, 1.0, 3.14)) is True


class TestEnvironmentValidation:
    def test_fields_are_positional_in_order(self):
        obs = (Obstacle(1.0, 2.0, 3.0),)
        env = Environment(BOUNDS, obs, 1.5)
        assert env.bounds == BOUNDS
        assert env.obstacles == obs
        assert env.clearance == 1.5

    def test_is_frozen(self):
        env = Environment(BOUNDS, (), 0.0)
        with pytest.raises(Exception):
            env.clearance = 5.0

    def test_empty_obstacle_list_is_allowed(self):
        env = Environment(BOUNDS, (), 0.0)
        assert env.obstacles == ()

    @pytest.mark.parametrize("bad_clearance", [-0.1, -5.0])
    def test_negative_clearance_raises(self, bad_clearance):
        with pytest.raises(ValueError):
            Environment(BOUNDS, (), bad_clearance)

    def test_zero_clearance_is_allowed(self):
        assert Environment(BOUNDS, (), 0.0).clearance == 0.0

    @pytest.mark.parametrize("bad_bounds", [
        (10.0, 0.0, 10.0, 60.0),    # xmin == xmax
        (10.0, 0.0, 5.0, 60.0),     # xmin > xmax
        (0.0, 10.0, 100.0, 10.0),   # ymin == ymax
        (0.0, 10.0, 100.0, 5.0),    # ymin > ymax
    ])
    def test_invalid_bounds_raise(self, bad_bounds):
        with pytest.raises(ValueError):
            Environment(bad_bounds, (), 0.0)


class TestBounds:
    def _env(self):
        return Environment(BOUNDS, (), 0.0)

    def test_point_inside(self):
        assert self._env().is_inside_bounds((50.0, 30.0)) is True

    @pytest.mark.parametrize("point", [
        (-1.0, 30.0),    # sol disi
        (101.0, 30.0),   # sag disi
        (50.0, -1.0),    # alt disi
        (50.0, 61.0),    # ust disi
    ])
    def test_point_outside(self, point):
        assert self._env().is_inside_bounds(point) is False

    @pytest.mark.parametrize("corner", [
        (0.0, 0.0), (100.0, 0.0), (0.0, 60.0), (100.0, 60.0),
    ])
    def test_corners_are_inside(self, corner):
        # sinir uzerindeki nokta haritanin ICINDE sayilir
        assert self._env().is_inside_bounds(corner) is True

    def test_accepts_pose_ignoring_yaw(self):
        assert self._env().is_inside_bounds((50.0, 30.0, 2.0)) is True

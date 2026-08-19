"""src/environment.py icin testler."""

import math
import random

import pytest

from src.environment import Obstacle


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

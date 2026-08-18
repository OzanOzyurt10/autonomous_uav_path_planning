"""src/dubins.py icin testler."""

import math

import pytest

from src.dubins import DubinsPath, _mod2pi

ANGLES = [-10.0, -math.pi, -0.1, 0.0, 0.1, math.pi, 7.0, 100.0]


class TestMod2Pi:
    def test_maps_into_zero_two_pi(self):
        for theta in ANGLES:
            assert 0.0 <= _mod2pi(theta) < 2 * math.pi

    def test_preserves_angle_identity(self):
        for theta in ANGLES:
            wrapped = _mod2pi(theta)
            assert math.isclose(math.sin(wrapped), math.sin(theta), abs_tol=1e-12)
            assert math.isclose(math.cos(wrapped), math.cos(theta), abs_tol=1e-12)

    def test_zero_stays_zero(self):
        assert _mod2pi(0.0) == 0.0

    def test_negative_small_wraps_near_two_pi(self):
        assert math.isclose(_mod2pi(-0.1), 2 * math.pi - 0.1, abs_tol=1e-12)


class TestDubinsPathBasics:
    def test_length_is_sum_of_segments(self):
        path = DubinsPath(start=(0.0, 0.0, 0.0), word="LSL",
                          lengths=(1.0, 2.0, 3.0), rho=5.0)
        assert math.isclose(path.length, 6.0)

    def test_is_frozen(self):
        path = DubinsPath(start=(0.0, 0.0, 0.0), word="LSL",
                          lengths=(1.0, 2.0, 3.0), rho=5.0)
        with pytest.raises(Exception):
            path.rho = 2.0

    def test_zero_length_path(self):
        path = DubinsPath(start=(1.0, 2.0, 0.5), word="LSL",
                          lengths=(0.0, 0.0, 0.0), rho=5.0)
        assert path.length == 0.0

    def test_fields_are_positional_in_order(self):
        path = DubinsPath((1.0, 2.0, 0.5), "RSR", (1.0, 2.0, 3.0), 4.0)
        assert path.start == (1.0, 2.0, 0.5)
        assert path.word == "RSR"
        assert path.lengths == (1.0, 2.0, 3.0)
        assert path.rho == 4.0

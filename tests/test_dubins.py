"""src/dubins.py icin testler."""

import math

import pytest

from src.dubins import DubinsPath, _mod2pi, _segment_end

ANGLES = [-10.0, -math.pi, -0.1, 0.0, 0.1, math.pi, 7.0, 100.0]
HALF_PI = math.pi / 2


def assert_pose_close(actual, expected, tol=1e-6):
    """Iki pozu karsilastirir; yaw farkini 2pi sarmasini dikkate alarak olcer."""
    assert math.isclose(actual[0], expected[0], abs_tol=tol), f"x: {actual} vs {expected}"
    assert math.isclose(actual[1], expected[1], abs_tol=tol), f"y: {actual} vs {expected}"
    dyaw = _mod2pi(actual[2] - expected[2])
    dyaw = min(dyaw, 2 * math.pi - dyaw)
    assert dyaw < tol, f"yaw: {actual} vs {expected}"


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


class TestSegmentGeometry:
    def test_straight_moves_along_heading(self):
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "S", 3.0, 1.0), (3.0, 0.0, 0.0))
        assert_pose_close(_segment_end((0.0, 0.0, HALF_PI), "S", 3.0, 1.0), (0.0, 3.0, HALF_PI))

    def test_left_quarter_turn(self):
        # rho=1, ceyrek tur sola (yay uzunlugu pi/2): (0,0,0) -> (1,1,pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "L", HALF_PI, 1.0), (1.0, 1.0, HALF_PI))

    def test_right_quarter_turn(self):
        # rho=1, ceyrek tur saga: (0,0,0) -> (1,-1,-pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "R", HALF_PI, 1.0), (1.0, -1.0, -HALF_PI))

    def test_turn_radius_is_respected(self):
        # rho=5 ile ceyrek tur sola: (0,0,0) -> (5,5,pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "L", HALF_PI * 5.0, 5.0), (5.0, 5.0, HALF_PI))

    def test_full_left_circle_returns_to_start(self):
        rho, start = 2.5, (3.0, -1.0, 0.7)
        assert_pose_close(_segment_end(start, "L", 2 * math.pi * rho, rho), start)

    def test_full_right_circle_returns_to_start(self):
        rho, start = 2.5, (3.0, -1.0, 0.7)
        assert_pose_close(_segment_end(start, "R", 2 * math.pi * rho, rho), start)

    def test_zero_length_is_identity(self):
        start = (3.0, -1.0, 0.7)
        for mode in "LSR":
            assert_pose_close(_segment_end(start, mode, 0.0, 2.0), start)

    def test_unknown_mode_raises(self):
        with pytest.raises(ValueError):
            _segment_end((0.0, 0.0, 0.0), "X", 1.0, 1.0)

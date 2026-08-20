"""src/dubins3d.py icin testler."""

import math
import random

import pytest

from src.dubins import DubinsPath, path_length, shortest_path
from src.dubins3d import DubinsPath3D, _helix

RHO = 5.0
GAMMA_MAX = math.radians(15.0)
TWO_PI_RHO = 2 * math.pi * RHO


def _flat(pose3):
    """3B pozdan 2B poz: (x, y, z, yaw) -> (x, y, yaw)."""
    return (pose3[0], pose3[1], pose3[3])


def _manual(start3, goal2, helix_turns, gamma):
    """airplane_path olmadan elle DubinsPath3D kurar (erken gorevler icin)."""
    return DubinsPath3D(start3, shortest_path(_flat(start3), goal2, RHO),
                        helix_turns, gamma)


def _as3(pose2, z=0.0):
    """2B pozu (x, y, yaw) 3B karsilastirma icin (x, y, z, yaw) yapar."""
    return (pose2[0], pose2[1], z, pose2[2])


def assert_pose3_close(actual, expected, tol=1e-6):
    for i, ad in enumerate(("x", "y", "z")):
        assert math.isclose(actual[i], expected[i], abs_tol=tol), \
            f"{ad}: {actual} vs {expected}"
    dyaw = (actual[3] - expected[3]) % (2 * math.pi)
    dyaw = min(dyaw, 2 * math.pi - dyaw)
    assert dyaw < tol, f"yaw: {actual} vs {expected}"


class TestDubinsPath3D:
    def test_fields_are_positional_in_order(self):
        h = shortest_path((0.0, 0.0, 0.0), (30.0, 0.0, 0.0), RHO)
        p = DubinsPath3D((0.0, 0.0, 100.0, 0.0), h, 2, 0.2122)
        assert p.start == (0.0, 0.0, 100.0, 0.0)
        assert p.horizontal is h
        assert p.helix_turns == 2
        assert math.isclose(p.gamma, 0.2122)

    def test_is_frozen(self):
        h = shortest_path((0.0, 0.0, 0.0), (30.0, 0.0, 0.0), RHO)
        p = DubinsPath3D((0.0, 0.0, 0.0, 0.0), h, 0, 0.0)
        with pytest.raises(Exception):
            p.gamma = 1.0

    def test_horizontal_is_a_two_d_path(self):
        h = shortest_path((0.0, 0.0, 0.0), (30.0, 0.0, 0.0), RHO)
        p = DubinsPath3D((0.0, 0.0, 0.0, 0.0), h, 0, 0.0)
        assert isinstance(p.horizontal, DubinsPath)
        assert len(p.horizontal.start) == 3      # 2B poz, z yok


class TestHelix:
    def test_returns_to_start_pose(self):
        start = (3.0, 7.0, math.radians(40))
        for k in (1, 2, 3):
            h = _helix(start, "L", k * TWO_PI_RHO, RHO)
            assert_pose3_close(_as3(h.end_pose()), _as3(start))

    def test_length_matches_requested(self):
        h = _helix((0.0, 0.0, 0.0), "L", 2 * TWO_PI_RHO, RHO)
        assert math.isclose(h.length, 2 * TWO_PI_RHO)

    def test_right_direction_also_returns(self):
        start = (1.0, 2.0, 0.5)
        h = _helix(start, "R", TWO_PI_RHO, RHO)
        assert_pose3_close(_as3(h.end_pose()), _as3(start))

    def test_turns_the_requested_way(self):
        """L saat yonunun tersine, R saat yonunde."""
        quarter = TWO_PI_RHO / 4
        left = _helix((0.0, 0.0, 0.0), "L", quarter, RHO)
        right = _helix((0.0, 0.0, 0.0), "R", quarter, RHO)
        assert left.interpolate(quarter)[1] > 0      # sola donunce y artar
        assert right.interpolate(quarter)[1] < 0     # saga donunce y azalir

    def test_zero_length_is_a_point(self):
        h = _helix((4.0, 5.0, 1.0), "L", 0.0, RHO)
        assert math.isclose(h.length, 0.0)
        assert h.end_pose() == (4.0, 5.0, 1.0)

    def test_word_is_valid(self):
        assert _helix((0.0, 0.0, 0.0), "L", 1.0, RHO).word == "LSL"
        assert _helix((0.0, 0.0, 0.0), "R", 1.0, RHO).word == "RSR"


class TestLengths:
    def test_no_helix_horizontal_equals_two_d(self):
        p = _manual((0.0, 0.0, 0.0, 0.0), (100.0, 0.0, 0.0), 0, 0.099669)
        assert math.isclose(p.horizontal_length, 100.0)

    def test_helix_adds_full_turns(self):
        p = _manual((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)
        assert math.isclose(p.horizontal_length, 30.0 + 2 * TWO_PI_RHO)

    def test_length_is_horizontal_over_cos_gamma(self):
        p = _manual((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)
        assert math.isclose(p.length, p.horizontal_length / math.cos(p.gamma))

    def test_length_matches_pythagoras(self):
        """length == sqrt(H^2 + dz^2), dz = H*tan(gamma)."""
        p = _manual((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)
        dz = p.horizontal_length * math.tan(p.gamma)
        assert math.isclose(p.length, math.hypot(p.horizontal_length, dz))

    def test_zero_gamma_length_equals_horizontal(self):
        p = _manual((0.0, 0.0, 0.0, 0.0), (40.0, 0.0, 0.0), 0, 0.0)
        assert math.isclose(p.length, 40.0)
        assert math.isclose(p.horizontal_length, 40.0)

    def test_descent_length_equals_climb_length(self):
        up = _manual((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)
        down = _manual((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 0.0), 2, -0.212200)
        assert math.isclose(up.length, down.length)

    def test_known_values(self):
        p = _manual((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)
        assert math.isclose(p.horizontal_length, 92.831853, abs_tol=1e-5)
        assert math.isclose(p.length, 94.961850, abs_tol=1e-5)


class TestInterpolate:
    def _climb(self):
        """30 m yatay yol, 2 helis, ~12.16 derece tirmanis."""
        return _manual((0.0, 0.0, 100.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)

    def test_start_is_returned_at_zero(self):
        p = self._climb()
        assert_pose3_close(p.interpolate(0.0), p.start)

    def test_end_pose_matches_interpolate_at_length(self):
        p = self._climb()
        assert_pose3_close(p.interpolate(p.length), p.end_pose())

    def test_end_pose_xy_yaw_come_from_horizontal(self):
        p = self._climb()
        hx, hy, hyaw = p.horizontal.end_pose()
        end = p.end_pose()
        assert math.isclose(end[0], hx, abs_tol=1e-9)
        assert math.isclose(end[1], hy, abs_tol=1e-9)
        assert math.isclose(end[3], hyaw, abs_tol=1e-9)

    def test_end_altitude_follows_gamma(self):
        p = self._climb()
        expected = p.start[2] + p.horizontal_length * math.tan(p.gamma)
        assert math.isclose(p.end_pose()[2], expected, abs_tol=1e-9)

    def test_altitude_is_monotone_while_climbing(self):
        p = self._climb()
        zs = [p.interpolate(i / 40 * p.length)[2] for i in range(41)]
        assert zs == sorted(zs)

    def test_altitude_is_monotone_while_descending(self):
        p = _manual((0.0, 0.0, 100.0, 0.0), (30.0, 0.0, 0.0), 2, -0.212200)
        zs = [p.interpolate(i / 40 * p.length)[2] for i in range(41)]
        assert zs == sorted(zs, reverse=True)

    def test_helix_region_stays_near_start_xy(self):
        """Helis bolgesinde ucak baslangicin cevresinde donuyor."""
        p = self._climb()
        h_end = p.helix_turns * TWO_PI_RHO           # helis biterken yatay mesafe
        s = (h_end / 2) / math.cos(p.gamma)          # helisin ortasi
        x, y, _, _ = p.interpolate(s)
        assert math.hypot(x - p.start[0], y - p.start[1]) <= 2 * RHO + 1e-9

    def test_after_helix_follows_horizontal(self):
        p = self._climb()
        h_after = p.helix_turns * TWO_PI_RHO + 10.0
        x, y, _, yaw = p.interpolate(h_after / math.cos(p.gamma))
        hx, hy, hyaw = p.horizontal.interpolate(10.0)
        assert math.isclose(x, hx, abs_tol=1e-9)
        assert math.isclose(y, hy, abs_tol=1e-9)
        assert math.isclose(yaw, hyaw, abs_tol=1e-9)

    def test_clamps_out_of_range(self):
        p = self._climb()
        assert_pose3_close(p.interpolate(-5.0), p.start)
        assert_pose3_close(p.interpolate(p.length + 5.0), p.end_pose())

    def test_no_helix_path_ignores_helix_branch(self):
        p = _manual((0.0, 0.0, 50.0, 0.0), (100.0, 0.0, 0.0), 0, 0.099669)
        x, y, z, yaw = p.interpolate(p.length / 2)
        hx, hy, hyaw = p.horizontal.interpolate(50.0)
        assert math.isclose(x, hx, abs_tol=1e-9)
        assert math.isclose(y, hy, abs_tol=1e-9)

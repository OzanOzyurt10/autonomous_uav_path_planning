"""src/dubins3d.py icin testler."""

import math
import random

import pytest

from src.dubins import DubinsPath, path_length, shortest_path
from src.dubins3d import (DubinsPath3D, _helix, airplane_length,
                          airplane_path)

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


class TestSample3D:
    def _climb(self):
        return _manual((0.0, 0.0, 100.0, 0.0), (30.0, 0.0, 0.0), 2, 0.212200)

    def test_first_and_last(self):
        p = self._climb()
        pts = p.sample(1.0)
        assert_pose3_close(pts[0], p.start)
        assert_pose3_close(pts[-1], p.end_pose())

    def test_spacing_never_exceeds_step(self):
        p = self._climb()
        pts = p.sample(2.0)
        for a, b in zip(pts, pts[1:]):
            d = math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
                          + (b[2] - a[2]) ** 2)
            assert d <= 2.0 + 1e-9

    def test_all_points_are_four_element(self):
        for pose in self._climb().sample(5.0):
            assert len(pose) == 4

    def test_zero_length_gives_single_point(self):
        p = _manual((1.0, 2.0, 3.0, 0.0), (1.0, 2.0, 0.0), 0, 0.0)
        assert p.sample(1.0) == [p.start]

    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_step_raises(self, bad):
        with pytest.raises(ValueError):
            self._climb().sample(bad)

    def test_smaller_step_gives_more_points(self):
        p = self._climb()
        assert len(p.sample(1.0)) > len(p.sample(5.0))


class TestAirplanePathValidation:
    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_rho_raises(self, bad):
        with pytest.raises(ValueError):
            airplane_path((0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 5.0, 0.0),
                          bad, GAMMA_MAX)

    @pytest.mark.parametrize("bad", [0.0, -0.1, math.pi / 2, 2.0])
    def test_bad_gamma_max_raises(self, bad):
        with pytest.raises(ValueError):
            airplane_path((0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 5.0, 0.0),
                          RHO, bad)


class TestAirplanePathCases:
    def test_reachable_altitude_needs_no_helix(self):
        p = airplane_path((0.0, 0.0, 0.0, 0.0), (100.0, 0.0, 10.0, 0.0),
                          RHO, GAMMA_MAX)
        assert p.helix_turns == 0
        assert math.isclose(p.horizontal_length, 100.0, abs_tol=1e-6)
        assert math.isclose(p.gamma, 0.099669, abs_tol=1e-5)
        assert math.isclose(p.length, 100.498756, abs_tol=1e-5)

    def test_unreachable_altitude_adds_helix(self):
        p = airplane_path((0.0, 0.0, 0.0, 0.0), (30.0, 0.0, 20.0, 0.0),
                          RHO, GAMMA_MAX)
        assert p.helix_turns == 2
        assert math.isclose(p.horizontal_length, 92.831853, abs_tol=1e-5)
        assert math.isclose(p.gamma, 0.212200, abs_tol=1e-5)
        assert math.isclose(p.length, 94.961850, abs_tol=1e-5)

    def test_descent_mirrors_climb(self):
        p = airplane_path((0.0, 0.0, 20.0, 0.0), (30.0, 0.0, 0.0, 0.0),
                          RHO, GAMMA_MAX)
        assert p.helix_turns == 2
        assert math.isclose(p.gamma, -0.212200, abs_tol=1e-5)

    def test_level_flight_reduces_to_two_d(self):
        p = airplane_path((0.0, 0.0, 50.0, 0.0), (40.0, 0.0, 50.0, 0.0),
                          RHO, GAMMA_MAX)
        assert p.helix_turns == 0
        assert p.gamma == 0.0
        assert math.isclose(p.length, path_length((0.0, 0.0, 0.0),
                                                  (40.0, 0.0, 0.0), RHO))
        assert all(math.isclose(q[2], 50.0) for q in p.sample(1.0))

    def test_horizontal_is_the_two_d_shortest_path(self):
        start = (2.0, 3.0, 10.0, 0.4)
        goal = (40.0, 25.0, 30.0, 1.2)
        p = airplane_path(start, goal, RHO, GAMMA_MAX)
        expected = shortest_path((2.0, 3.0, 0.4), (40.0, 25.0, 1.2), RHO)
        assert p.horizontal.word == expected.word
        assert math.isclose(p.horizontal.length, expected.length)


class TestAirplanePathGeneral:
    def _pairs(self, seed=5, n=40):
        rng = random.Random(seed)
        for _ in range(n):
            yield ((rng.uniform(-50, 50), rng.uniform(-50, 50),
                    rng.uniform(0, 100), rng.uniform(0, 2 * math.pi)),
                   (rng.uniform(-50, 50), rng.uniform(-50, 50),
                    rng.uniform(0, 100), rng.uniform(0, 2 * math.pi)))

    @pytest.mark.parametrize("rho", [2.0, 5.0, 12.0])
    @pytest.mark.parametrize("gamma_deg", [5.0, 15.0, 30.0])
    def test_end_pose_reaches_goal(self, rho, gamma_deg):
        """En guclu test: yolun sonu hedefe variyor mu, dort bilesende."""
        gamma_max = math.radians(gamma_deg)
        for start, goal in self._pairs():
            p = airplane_path(start, goal, rho, gamma_max)
            assert_pose3_close(p.end_pose(), goal)

    @pytest.mark.parametrize("gamma_deg", [5.0, 15.0, 30.0])
    def test_gamma_never_exceeds_limit(self, gamma_deg):
        gamma_max = math.radians(gamma_deg)
        for start, goal in self._pairs():
            p = airplane_path(start, goal, RHO, gamma_max)
            assert abs(p.gamma) <= gamma_max + 1e-12

    def test_helix_overshoot_is_less_than_one_turn(self):
        """Nicemleme en fazla bir tur fazla atmali."""
        for start, goal in self._pairs():
            p = airplane_path(start, goal, RHO, GAMMA_MAX)
            if p.helix_turns == 0:
                continue
            dz = goal[2] - start[2]
            required = abs(dz) / math.tan(GAMMA_MAX)
            assert p.horizontal_length - required < TWO_PI_RHO + 1e-9
            assert p.horizontal_length >= required - 1e-9

    def test_altitude_is_monotone(self):
        for start, goal in self._pairs(seed=11, n=15):
            p = airplane_path(start, goal, RHO, GAMMA_MAX)
            zs = [q[2] for q in p.sample(2.0)]
            assert zs == sorted(zs) or zs == sorted(zs, reverse=True)


class TestAirplaneLength:
    def test_matches_path_length(self):
        start = (1.0, 2.0, 10.0, 0.3)
        goal = (40.0, 20.0, 45.0, 1.1)
        p = airplane_path(start, goal, RHO, GAMMA_MAX)
        assert math.isclose(airplane_length(start, goal, RHO, GAMMA_MAX),
                            p.length)

    def test_returns_three_d_not_horizontal(self):
        start = (0.0, 0.0, 0.0, 0.0)
        goal = (30.0, 0.0, 20.0, 0.0)
        p = airplane_path(start, goal, RHO, GAMMA_MAX)
        value = airplane_length(start, goal, RHO, GAMMA_MAX)
        assert value > p.horizontal_length
        assert math.isclose(value, 94.961850, abs_tol=1e-5)

    def test_level_flight_equals_two_d_length(self):
        value = airplane_length((0.0, 0.0, 7.0, 0.0), (40.0, 0.0, 7.0, 0.0),
                                RHO, GAMMA_MAX)
        assert math.isclose(value, path_length((0.0, 0.0, 0.0),
                                               (40.0, 0.0, 0.0), RHO))

    def test_is_asymmetric(self):
        """Dubins asimetrisi 3B'de de gecerli."""
        a = (0.0, 0.0, 0.0, 0.0)
        b = (0.0, 3.0, 5.0, math.pi / 2)
        assert not math.isclose(airplane_length(a, b, 2.0, GAMMA_MAX),
                                airplane_length(b, a, 2.0, GAMMA_MAX))

    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_rho_raises(self, bad):
        with pytest.raises(ValueError):
            airplane_length((0.0, 0.0, 0.0, 0.0), (10.0, 0.0, 5.0, 0.0),
                            bad, GAMMA_MAX)

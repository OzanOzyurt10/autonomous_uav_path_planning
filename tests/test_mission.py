"""app/mission.py icin testler."""

import array
import math

import pytest

from app.mission import (Constraints, agl_profile, build_env, plan_mission,
                         sample_mission, waypoint_headings)
from src.environment3d import Cylinder
from src.terrain import Terrain

BOUNDS = (0.0, 0.0, 0.0, 4000.0, 4000.0, 1500.0)

# 5 x 5 post, 1000 m arali: 4000 x 4000 m duz zemin (200 m kot).
FLAT = Terrain(array.array("h", [200] * 25), cols=5, rows=5,
               spacing_x=1000.0, spacing_y=1000.0)


def _constraints(**kwargs):
    base = dict(speed=28.0, max_bank=math.radians(30.0),
                max_climb=math.radians(8.0), clearance=100.0,
                max_iterations=600, seed=1)
    base.update(kwargs)
    return Constraints(**base)


class TestConstraints:
    def test_turn_radius_comes_from_speed_and_bank(self):
        """rho = v^2 / (g tan(yatis)); 28 m/s ve 30 derecede ~138 m."""
        assert _constraints().rho == pytest.approx(
            28.0 ** 2 / (9.80665 * math.tan(math.radians(30.0))))

    def test_steeper_bank_turns_tighter(self):
        shallow = _constraints(max_bank=math.radians(20.0)).rho
        steep = _constraints(max_bank=math.radians(45.0)).rho
        assert steep < shallow

    def test_faster_needs_more_room(self):
        assert _constraints(speed=40.0).rho > _constraints(speed=20.0).rho

    @pytest.mark.parametrize("bad", [{"speed": 0.0}, {"speed": -1.0},
                                     {"max_bank": 0.0},
                                     {"max_bank": math.pi / 2},
                                     {"max_climb": 0.0},
                                     {"clearance": -1.0},
                                     {"max_iterations": 0}])
    def test_bad_values_raise(self, bad):
        with pytest.raises(ValueError):
            _constraints(**bad)


class TestWaypointHeadings:
    def test_two_waypoints_both_point_along_the_leg(self):
        headings = waypoint_headings([(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)])
        assert headings == pytest.approx([0.0, 0.0])

    def test_heading_follows_the_leg_direction(self):
        headings = waypoint_headings([(0.0, 0.0, 0.0), (0.0, 100.0, 0.0)])
        assert headings == pytest.approx([math.pi / 2, math.pi / 2])

    def test_middle_waypoint_takes_the_bisector(self):
        """Doguya gelip kuzeye donuyor: ortada 45 derece."""
        headings = waypoint_headings([(0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                                      (100.0, 100.0, 0.0)])
        assert headings[1] == pytest.approx(math.radians(45.0))

    def test_collinear_waypoints_share_one_heading(self):
        headings = waypoint_headings([(0.0, 0.0, 0.0), (50.0, 0.0, 0.0),
                                      (100.0, 0.0, 0.0)])
        assert headings == pytest.approx([0.0, 0.0, 0.0])

    def test_bisector_wraps_around_zero(self):
        """350 ile 10 derecenin ortasi 0, 180 degil - ortalama alinamaz."""
        def step(point, degrees):
            angle = math.radians(degrees)
            return (point[0] + 1000.0 * math.cos(angle),
                    point[1] + 1000.0 * math.sin(angle), 0.0)

        first = (0.0, 0.0, 0.0)
        second = step(first, -10.0)
        third = step(second, 10.0)
        headings = waypoint_headings([first, second, third])
        assert headings[0] == pytest.approx(math.radians(-10.0))
        assert headings[1] == pytest.approx(0.0, abs=1e-9)
        assert headings[2] == pytest.approx(math.radians(10.0))

    def test_reversal_does_not_crash(self):
        """Tam geri donuste aciortay tanimsiz; gelis yonu kullaniliyor."""
        headings = waypoint_headings([(0.0, 0.0, 0.0), (100.0, 0.0, 0.0),
                                      (0.0, 0.0, 0.0)])
        assert headings[1] == pytest.approx(0.0)

    def test_single_waypoint_raises(self):
        with pytest.raises(ValueError):
            waypoint_headings([(0.0, 0.0, 0.0)])

    def test_empty_raises(self):
        with pytest.raises(ValueError):
            waypoint_headings([])


class TestPlanMission:
    def _mission(self, waypoints, obstacles=(), **kwargs):
        constraints = _constraints(**kwargs)
        env = build_env(BOUNDS, FLAT, constraints, obstacles)
        return plan_mission(waypoints, env, constraints), env, constraints

    def test_two_waypoints_give_one_leg(self):
        mission, _, _ = self._mission([(500.0, 500.0, 700.0),
                                       (3500.0, 3500.0, 800.0)])
        assert len(mission.legs) == 1
        assert mission.found is True

    def test_three_waypoints_give_two_legs(self):
        mission, _, _ = self._mission([(500.0, 500.0, 700.0),
                                       (3500.0, 500.0, 700.0),
                                       (3500.0, 3500.0, 800.0)])
        assert len(mission.legs) == 2
        assert mission.found is True

    def test_legs_chain_end_to_end(self):
        mission, _, _ = self._mission([(500.0, 500.0, 700.0),
                                       (3500.0, 500.0, 700.0),
                                       (3500.0, 3500.0, 800.0)])
        for first, second in zip(mission.legs, mission.legs[1:]):
            assert first.goal == second.start

    def test_route_clears_the_terrain(self):
        mission, env, _ = self._mission([(500.0, 500.0, 700.0),
                                         (3500.0, 3500.0, 800.0)])
        for leg in mission.legs:
            for edge in leg.edges:
                assert env.is_path_free(edge.sample(50.0)) is True

    def test_cost_is_the_sum_of_legs(self):
        mission, _, _ = self._mission([(500.0, 500.0, 700.0),
                                       (3500.0, 500.0, 700.0),
                                       (3500.0, 3500.0, 800.0)])
        assert mission.cost == pytest.approx(
            sum(leg.cost for leg in mission.legs))

    def test_duration_is_length_over_speed(self):
        mission, _, constraints = self._mission([(500.0, 500.0, 700.0),
                                                 (3500.0, 3500.0, 800.0)])
        assert mission.duration == pytest.approx(
            mission.cost / constraints.speed)

    def test_waypoint_below_the_terrain_is_unreachable(self):
        """Zemin 200 m, emniyet payi 100 m: 250 m'lik waypoint yasak."""
        with pytest.raises(ValueError):
            self._mission([(500.0, 500.0, 250.0), (3500.0, 3500.0, 800.0)])

    def test_blocked_leg_reports_failure(self):
        wall = tuple(Cylinder(2000.0, float(y), 220.0, 0.0, 1500.0)
                     for y in range(-200, 4400, 400))
        mission, _, _ = self._mission([(500.0, 2000.0, 700.0),
                                       (3500.0, 2000.0, 700.0)],
                                      obstacles=wall)
        assert mission.found is False
        assert mission.cost == math.inf
        assert mission.duration == math.inf


class TestSampleMission:
    def _mission(self):
        constraints = _constraints()
        env = build_env(BOUNDS, FLAT, constraints)
        return plan_mission([(500.0, 500.0, 700.0),
                             (3500.0, 500.0, 700.0),
                             (3500.0, 3500.0, 800.0)], env, constraints)

    def test_starts_at_the_first_waypoint(self):
        poses = sample_mission(self._mission(), 50.0)
        assert poses[0][0] == pytest.approx(500.0)
        assert poses[0][1] == pytest.approx(500.0)

    def test_ends_at_the_last_waypoint(self):
        poses = sample_mission(self._mission(), 50.0)
        assert poses[-1][0] == pytest.approx(3500.0, abs=1.0)
        assert poses[-1][1] == pytest.approx(3500.0, abs=1.0)

    def test_spacing_never_exceeds_the_step(self):
        poses = sample_mission(self._mission(), 50.0)
        for first, second in zip(poses, poses[1:]):
            assert math.dist(first[:3], second[:3]) <= 50.0 + 1e-6

    def test_poses_have_four_elements(self):
        for pose in sample_mission(self._mission(), 200.0):
            assert len(pose) == 4

    def test_no_duplicate_points_at_edge_joins(self):
        poses = sample_mission(self._mission(), 50.0)
        for first, second in zip(poses, poses[1:]):
            assert math.dist(first[:3], second[:3]) > 1e-6

    def test_bad_step_raises(self):
        with pytest.raises(ValueError):
            sample_mission(self._mission(), 0.0)


class TestAglProfile:
    def test_flat_terrain_gives_altitude_minus_ground(self):
        poses = [(0.0, 0.0, 700.0, 0.0), (1000.0, 1000.0, 900.0, 0.0)]
        assert agl_profile(poses, FLAT) == pytest.approx([500.0, 700.0])

    def test_no_terrain_gives_empty(self):
        assert agl_profile([(0.0, 0.0, 700.0, 0.0)], None) == []

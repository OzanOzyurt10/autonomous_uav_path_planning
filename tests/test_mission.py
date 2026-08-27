"""app/mission.py icin testler."""

import array
import math

import pytest

from app.mission import (Constraints, FlatEdge, Zone, agl_profile,
                         build_env, build_env_2d, geo_window,
                         plan_mission, plan_mission_2d, sample_mission,
                         waypoint_headings)
from src.dubins import shortest_path
from src.environment import Obstacle
from src.environment3d import Cylinder
from src.geo import METRES_PER_DEGREE, Frame
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


# --- gorev penceresi: waypointlerin sinir kutusu arti pay ---------------
# Pencere artik elle secilmiyor. Bu testlerin ortak sartI: her waypoint
# uretilen pencerenin ICINDE kalmali - disarida kalirsa planlayici
# baslangic pozunu reddeder ve kullanici nedenini goremez.
def _inside(points, window):
    lat, lon, width, height = window
    frame = Frame.for_window(lat, lon, height)
    for point in points:
        x, y = frame.to_local(*point)
        if not (0.0 <= x <= width and 0.0 <= y <= height):
            return False
    return True


class TestGeoWindow:
    ALPS = [(46.20, 14.40), (46.28, 14.55)]

    def test_covers_every_waypoint(self):
        assert _inside(self.ALPS, geo_window(self.ALPS, 2000.0, 60000.0))

    @pytest.mark.parametrize("points", [
        [(-15.90, -47.95), (-15.75, -47.80)],      # guney yarikure, bati boylam
        [(69.60, 18.90), (69.70, 19.20)],          # yuksek enlem
        [(0.05, -0.05), (-0.05, 0.05)],            # ekvator ve sifir boylami
    ])
    def test_covers_waypoints_anywhere(self, points):
        assert _inside(points, geo_window(points, 2000.0, 60000.0))

    def test_margin_is_added_on_both_sides(self):
        points = [(46.0, 14.0), (46.0, 14.0)]      # tek noktaya cakisik
        _, _, width, height = geo_window(points, 3000.0, 60000.0,
                                         min_side=0.0)
        assert width == pytest.approx(6000.0)
        assert height == pytest.approx(6000.0)

    def test_tiny_mission_gets_minimum_window(self):
        points = [(46.20, 14.40), (46.2001, 14.4001)]
        _, _, width, height = geo_window(points, 0.0, 60000.0, min_side=4000.0)
        assert width == pytest.approx(4000.0)
        assert height == pytest.approx(4000.0)

    def test_window_is_centred_on_the_waypoints(self):
        lat, lon, width, height = geo_window(self.ALPS, 2000.0, 60000.0)
        frame = Frame.for_window(lat, lon, height)
        xs, ys = zip(*[frame.to_local(*p) for p in self.ALPS])
        assert (min(xs) + max(xs)) / 2 == pytest.approx(width / 2)
        assert (min(ys) + max(ys)) / 2 == pytest.approx(height / 2)

    def test_height_grows_with_the_span(self):
        near = geo_window([(46.0, 14.0), (46.1, 14.0)], 0.0, 60000.0)
        far = geo_window([(46.0, 14.0), (46.3, 14.0)], 0.0, 60000.0)
        assert far[3] > near[3]
        assert far[3] == pytest.approx(0.3 * METRES_PER_DEGREE)

    def test_too_large_raises_instead_of_clipping(self):
        # Kirpmak waypointi pencerenin disinda birakir ve planlama
        # anlasilmaz bicimde basarisiz olur.
        with pytest.raises(ValueError, match="cok buyuk"):
            geo_window([(46.0, 14.0), (47.0, 16.0)], 2000.0, 60000.0)

    def test_single_point_raises(self):
        with pytest.raises(ValueError):
            geo_window([(46.0, 14.0)], 2000.0, 60000.0)

    def test_antimeridian_raises(self):
        with pytest.raises(ValueError, match="180"):
            geo_window([(0.0, 179.5), (0.0, -179.5)], 2000.0, 60000.0)


# --- 2B planlama: arazi yok, yasak bolgeler var ------------------------
CON2D = Constraints(speed=28.0, max_bank=math.radians(30.0),
                    max_climb=math.radians(8.0), clearance=100.0,
                    max_iterations=2000, seed=1)
BOX = (0.0, 0.0, 20000.0, 20000.0)


class TestZone:
    def test_becomes_a_circle_in_2d(self):
        obstacle = Zone(100.0, 200.0, 50.0).as_obstacle()
        assert isinstance(obstacle, Obstacle)
        assert (obstacle.x, obstacle.y, obstacle.radius) == (100.0, 200.0, 50.0)

    def test_becomes_a_cylinder_in_3d(self):
        cylinder = Zone(100.0, 200.0, 50.0, 300.0, 900.0).as_cylinder()
        assert (cylinder.z_min, cylinder.z_max) == (300.0, 900.0)
        assert cylinder.radius == 50.0

    def test_default_column_spans_any_usable_altitude(self):
        # 2B'de irtifa yok sayilmali; varsayilan sutun her seyi kapsiyor.
        zone = Zone(0.0, 0.0, 100.0)
        assert zone.as_cylinder().contains((0.0, 0.0, 5000.0))


class TestFlatEdge:
    PATH = shortest_path((0.0, 0.0, 0.0), (600.0, 400.0, 1.0), 150.0)
    EDGE = FlatEdge(PATH, 1200.0)

    def test_length_is_the_horizontal_length(self):
        assert self.EDGE.length == pytest.approx(self.PATH.length)

    def test_start_and_end_carry_the_altitude(self):
        assert self.EDGE.start[2] == pytest.approx(1200.0)
        assert self.EDGE.end_pose()[2] == pytest.approx(1200.0)

    def test_start_keeps_yaw_as_the_fourth_element(self):
        # 3B poz duzeni (x, y, z, yaw); 2B'de yaw ucuncu sirada geliyordu.
        assert self.EDGE.start[3] == pytest.approx(self.PATH.start[2])

    def test_samples_are_four_element_poses_at_one_altitude(self):
        poses = self.EDGE.sample(50.0)
        assert all(len(pose) == 4 for pose in poses)
        assert {round(pose[2], 9) for pose in poses} == {1200.0}

    def test_samples_follow_the_horizontal_path(self):
        flat = self.EDGE.sample(50.0)
        plain = self.PATH.sample(50.0)
        assert len(flat) == len(plain)
        for lifted, original in zip(flat, plain):
            assert lifted[0] == pytest.approx(original[0])
            assert lifted[1] == pytest.approx(original[1])


class TestPlanMission2d:
    ZONES = (Zone(10000.0, 10000.0, 2500.0), Zone(6000.0, 14000.0, 1800.0))
    ENDS = [(2000.0, 2000.0), (18000.0, 18000.0)]

    def _mission(self, zones=ZONES, altitude=1200.0):
        env = build_env_2d(BOX, CON2D,
                           tuple(z.as_obstacle() for z in zones))
        return plan_mission_2d(self.ENDS, env, CON2D, altitude), env

    def test_finds_a_route_around_the_zones(self):
        mission, _ = self._mission()
        assert mission.found
        assert mission.cost > math.dist(*self.ENDS)      # dolasmak zorunda

    def test_route_stays_out_of_every_zone(self):
        mission, env = self._mission()
        for pose in sample_mission(mission, 25.0):
            for zone in self.ZONES:
                gap = math.hypot(pose[0] - zone.x, pose[1] - zone.y)
                assert gap >= zone.radius + env.clearance - 1e-6

    def test_altitude_is_constant(self):
        mission, _ = self._mission(altitude=850.0)
        assert {round(p[2], 6) for p in sample_mission(mission, 50.0)} == {850.0}

    def test_duration_follows_cost_and_speed(self):
        mission, _ = self._mission()
        assert mission.duration == pytest.approx(mission.cost / CON2D.speed)

    def test_open_map_route_is_near_the_straight_line(self):
        mission, _ = self._mission(zones=())
        assert mission.found
        assert mission.cost < math.dist(*self.ENDS) * 1.15

    def test_works_with_no_terrain_anywhere(self):
        # 2B'nin butun mesele bu: arazi verisi olmayan bolgede de calisiyor.
        env = build_env_2d(BOX, CON2D)
        mission = plan_mission_2d(self.ENDS, env, CON2D, 1200.0)
        assert mission.found
        assert agl_profile(sample_mission(mission, 100.0), None) == []

    def test_blocked_leg_is_reported_not_raised(self):
        wall = tuple(Zone(10000.0, float(y), 700.0)
                     for y in range(-1000, 22000, 1000))
        mission, _ = self._mission(zones=wall)
        assert not mission.found
        assert mission.cost == math.inf
        assert any(not leg.found for leg in mission.legs)

    def test_multi_leg_mission_chains_without_gaps(self):
        env = build_env_2d(BOX, CON2D)
        mission = plan_mission_2d([(2000.0, 2000.0), (10000.0, 4000.0),
                                   (17000.0, 15000.0)], env, CON2D, 900.0)
        assert mission.found and len(mission.legs) == 2
        poses = sample_mission(mission, 50.0)
        gaps = [math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:])]
        assert max(gaps) < 55.0

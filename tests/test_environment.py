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


class TestIsFree:
    def test_empty_map_everything_inside_is_free(self):
        env = Environment(BOUNDS, (), 0.0)
        for point in [(0.0, 0.0), (50.0, 30.0), (100.0, 60.0)]:
            assert env.is_free(point) is True

    def test_outside_bounds_is_not_free(self):
        env = Environment(BOUNDS, (), 0.0)
        assert env.is_free((150.0, 30.0)) is False

    def test_inside_obstacle_is_not_free(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((50.0, 30.0)) is False
        assert env.is_free((55.0, 30.0)) is False

    def test_outside_obstacle_is_free(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((70.0, 30.0)) is True

    def test_obstacle_boundary_is_not_free(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((60.0, 30.0)) is False

    def test_clearance_makes_previously_free_point_blocked(self):
        obstacles = (Obstacle(50.0, 30.0, 10.0),)
        assert Environment(BOUNDS, obstacles, 0.0).is_free((62.0, 30.0)) is True
        assert Environment(BOUNDS, obstacles, 5.0).is_free((62.0, 30.0)) is False

    def test_any_obstacle_blocks(self):
        env = Environment(BOUNDS, (
            Obstacle(20.0, 20.0, 5.0),
            Obstacle(50.0, 30.0, 5.0),
            Obstacle(80.0, 40.0, 5.0),
        ), 0.0)
        assert env.is_free((50.0, 30.0)) is False   # ikinciye giriyor
        assert env.is_free((80.0, 40.0)) is False   # ucuncuye giriyor
        assert env.is_free((5.0, 50.0)) is True     # hicbirine girmiyor

    def test_point_inside_obstacle_but_outside_bounds(self):
        # iki sebep birden: yine de False
        env = Environment(BOUNDS, (Obstacle(105.0, 30.0, 10.0),), 0.0)
        assert env.is_free((105.0, 30.0)) is False

    def test_accepts_pose_ignoring_yaw(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((50.0, 30.0, 1.2)) is False


class TestIsPathFree:
    def _env(self):
        return Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)

    def test_clear_path_is_free(self):
        points = [(float(x), 5.0) for x in range(0, 101, 5)]
        assert self._env().is_path_free(points) is True

    def test_path_through_obstacle_is_blocked(self):
        points = [(float(x), 30.0) for x in range(0, 101, 5)]
        assert self._env().is_path_free(points) is False

    def test_single_blocked_point_is_enough(self):
        points = [(5.0, 5.0), (50.0, 30.0), (95.0, 55.0)]
        assert self._env().is_path_free(points) is False

    def test_path_leaving_bounds_is_blocked(self):
        points = [(50.0, 5.0), (150.0, 5.0)]
        assert self._env().is_path_free(points) is False

    def test_empty_list_is_free(self):
        # bos kume: kontrol edilecek bir sey yok
        assert self._env().is_path_free([]) is True

    def test_accepts_poses(self):
        poses = [(5.0, 5.0, 0.0), (10.0, 5.0, 0.5)]
        assert self._env().is_path_free(poses) is True


class TestSuggestedStep:
    def test_half_of_smallest_inflated_radius(self):
        env = Environment(BOUNDS, (
            Obstacle(20.0, 20.0, 8.0),
            Obstacle(50.0, 30.0, 3.0),   # en kucuk
            Obstacle(80.0, 40.0, 5.0),
        ), 0.0)
        assert math.isclose(env.suggested_step(), 1.5)

    def test_clearance_counts_toward_radius(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 3.0),), 1.0)
        assert math.isclose(env.suggested_step(), 2.0)

    def test_no_obstacles_returns_positive_value(self):
        assert Environment(BOUNDS, (), 0.0).suggested_step() > 0.0

    def test_no_obstacles_scales_with_map(self):
        small = Environment((0.0, 0.0, 10.0, 10.0), (), 0.0).suggested_step()
        big = Environment((0.0, 0.0, 1000.0, 1000.0), (), 0.0).suggested_step()
        assert big > small

    def test_step_is_small_enough_to_catch_obstacle(self):
        # onerilen adimla orneklenen bir dogru, engeli atlamamali
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 4.0),), 0.0)
        step = env.suggested_step()
        n = int(100.0 / step) + 1
        points = [(i * step, 30.0) for i in range(n)]
        assert env.is_path_free(points) is False


class TestRandomFreePose:
    def _env(self):
        return Environment(BOUNDS, (
            Obstacle(30.0, 30.0, 12.0),
            Obstacle(70.0, 30.0, 12.0),
        ), 2.0)

    def test_returns_three_element_pose(self):
        pose = self._env().random_free_pose(random.Random(1))
        assert len(pose) == 3

    def test_result_is_always_free(self):
        env = self._env()
        rng = random.Random(7)
        for _ in range(200):
            assert env.is_free(env.random_free_pose(rng)) is True

    def test_result_is_inside_bounds(self):
        env = self._env()
        rng = random.Random(8)
        x_min, y_min, x_max, y_max = BOUNDS
        for _ in range(200):
            x, y, _yaw = env.random_free_pose(rng)
            assert x_min <= x <= x_max
            assert y_min <= y <= y_max

    def test_yaw_is_normalized(self):
        env = self._env()
        rng = random.Random(9)
        for _ in range(200):
            _x, _y, yaw = env.random_free_pose(rng)
            assert 0.0 <= yaw < 2 * math.pi

    def test_same_seed_gives_same_result(self):
        env = self._env()
        first = env.random_free_pose(random.Random(42))
        second = env.random_free_pose(random.Random(42))
        assert first == second

    def test_different_seeds_give_different_results(self):
        env = self._env()
        assert env.random_free_pose(random.Random(1)) !=             env.random_free_pose(random.Random(2))

    def test_full_map_raises_runtime_error(self):
        # tum haritayi kaplayan engel: serbest poz bulunamaz
        env = Environment((0.0, 0.0, 10.0, 10.0),
                          (Obstacle(5.0, 5.0, 100.0),), 0.0)
        with pytest.raises(RuntimeError):
            env.random_free_pose(random.Random(1))

    def test_max_attempts_is_respected(self):
        env = Environment((0.0, 0.0, 10.0, 10.0),
                          (Obstacle(5.0, 5.0, 100.0),), 0.0)
        with pytest.raises(RuntimeError):
            env.random_free_pose(random.Random(1), max_attempts=5)

"""src/environment3d.py icin testler."""

import array
import math
import random

import pytest

from src.environment3d import Cylinder, Environment3D
from src.terrain import Terrain

BOUNDS = (0.0, 0.0, 0.0, 100.0, 100.0, 80.0)
CLEARANCE = 2.0

# Ortada duran, tavana kadar cikmayan bir kule
TOWER = Cylinder(50.0, 50.0, 10.0, 0.0, 30.0)


class TestCylinderValidation:
    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_radius_raises(self, bad):
        with pytest.raises(ValueError):
            Cylinder(0.0, 0.0, bad, 0.0, 10.0)

    @pytest.mark.parametrize("z_min, z_max", [(10.0, 10.0), (10.0, 5.0)])
    def test_bad_height_raises(self, z_min, z_max):
        with pytest.raises(ValueError):
            Cylinder(0.0, 0.0, 5.0, z_min, z_max)

    def test_is_frozen(self):
        with pytest.raises(Exception):
            TOWER.radius = 1.0


class TestCylinderContains:
    def test_point_inside(self):
        assert TOWER.contains((50.0, 50.0, 15.0)) is True

    def test_point_above_the_top_is_free(self):
        """Sonlu yukseklik: kulenin ustunden ucabilirsin."""
        assert TOWER.contains((50.0, 50.0, 40.0)) is False

    def test_point_below_the_bottom_is_free(self):
        low = Cylinder(50.0, 50.0, 10.0, 20.0, 40.0)
        assert low.contains((50.0, 50.0, 5.0)) is False

    def test_point_beside_is_free(self):
        assert TOWER.contains((70.0, 50.0, 15.0)) is False

    def test_both_conditions_are_required(self):
        """Yatayda icinde ama irtifada disinda -> serbest, ve tersi."""
        assert TOWER.contains((50.0, 50.0, 100.0)) is False   # yatay icinde
        assert TOWER.contains((90.0, 50.0, 15.0)) is False    # irtifa icinde

    def test_boundary_is_inside(self):
        assert TOWER.contains((60.0, 50.0, 15.0)) is True     # tam yaricapta
        assert TOWER.contains((50.0, 50.0, 30.0)) is True     # tam tavanda
        assert TOWER.contains((50.0, 50.0, 0.0)) is True      # tam tabanda

    def test_clearance_expands_horizontally(self):
        p = (61.0, 50.0, 15.0)                                # 11 m uzakta
        assert TOWER.contains(p) is False
        assert TOWER.contains(p, clearance=2.0) is True

    def test_clearance_expands_vertically(self):
        p = (50.0, 50.0, 31.5)
        assert TOWER.contains(p) is False
        assert TOWER.contains(p, clearance=2.0) is True

    def test_accepts_four_element_pose(self):
        """Poz da nokta gibi kabul edilmeli; yaw yok sayilir."""
        assert TOWER.contains((50.0, 50.0, 15.0, 1.2)) is True


class TestEnvironmentBounds:
    def _env(self):
        return Environment3D(BOUNDS, (TOWER,), CLEARANCE)

    def test_inside(self):
        assert self._env().is_inside_bounds((10.0, 10.0, 10.0)) is True

    def test_corners_are_inside(self):
        env = self._env()
        assert env.is_inside_bounds((0.0, 0.0, 0.0)) is True
        assert env.is_inside_bounds((100.0, 100.0, 80.0)) is True

    @pytest.mark.parametrize("point", [
        (-1.0, 50.0, 10.0), (101.0, 50.0, 10.0),
        (50.0, -1.0, 10.0), (50.0, 101.0, 10.0),
        (50.0, 50.0, -1.0), (50.0, 50.0, 81.0),
    ])
    def test_outside(self, point):
        assert self._env().is_inside_bounds(point) is False

    def test_altitude_ceiling_is_enforced(self):
        """2B'de olmayan boyut: tavan."""
        assert self._env().is_inside_bounds((50.0, 50.0, 79.0)) is True
        assert self._env().is_inside_bounds((50.0, 50.0, 80.1)) is False


class TestEnvironmentIsFree:
    def _env(self):
        return Environment3D(BOUNDS, (TOWER,), CLEARANCE)

    def test_free_point(self):
        assert self._env().is_free((10.0, 10.0, 10.0)) is True

    def test_inside_obstacle(self):
        assert self._env().is_free((50.0, 50.0, 15.0)) is False

    def test_inside_clearance_band(self):
        assert self._env().is_free((61.0, 50.0, 15.0)) is False

    def test_above_obstacle_is_free(self):
        """Kulenin tepesi 30 m, pay 2 m; 40 m'de serbest olmali."""
        assert self._env().is_free((50.0, 50.0, 40.0)) is True

    def test_out_of_bounds_is_not_free(self):
        assert self._env().is_free((50.0, 50.0, 200.0)) is False

    def test_no_obstacles_means_only_bounds_matter(self):
        env = Environment3D(BOUNDS, (), CLEARANCE)
        assert env.is_free((50.0, 50.0, 15.0)) is True

    def test_accepts_pose(self):
        assert self._env().is_free((10.0, 10.0, 10.0, 0.7)) is True


class TestIsPathFree:
    def _env(self):
        return Environment3D(BOUNDS, (TOWER,), CLEARANCE)

    def test_all_free(self):
        pts = [(10.0, 10.0, 10.0), (20.0, 20.0, 15.0), (30.0, 30.0, 20.0)]
        assert self._env().is_path_free(pts) is True

    def test_one_blocked_point_fails_the_path(self):
        pts = [(10.0, 10.0, 10.0), (50.0, 50.0, 15.0), (30.0, 30.0, 20.0)]
        assert self._env().is_path_free(pts) is False

    def test_path_over_the_tower_is_free(self):
        """Kulenin uzerinden gecen yol serbest; 3B'nin kazandirdigi sey."""
        pts = [(30.0, 50.0, 40.0), (50.0, 50.0, 40.0), (70.0, 50.0, 40.0)]
        assert self._env().is_path_free(pts) is True

    def test_empty_path_is_free(self):
        assert self._env().is_path_free([]) is True

    def test_accepts_poses(self):
        pts = [(10.0, 10.0, 10.0, 0.0), (20.0, 20.0, 15.0, 1.0)]
        assert self._env().is_path_free(pts) is True


class TestSuggestedStep:
    def test_no_obstacles_uses_shortest_side(self):
        """Kenarlar 100, 100, 80 -> en kisasi 80, onda biri 8."""
        env = Environment3D(BOUNDS, (), CLEARANCE)
        assert math.isclose(env.suggested_step(), 8.0)

    def test_tall_thin_cylinder_uses_radius(self):
        """Yaricap 10, yukseklik 60: min(10+2, 30+2) = 12 -> adim 6."""
        tall = Cylinder(50.0, 50.0, 10.0, 0.0, 60.0)
        env = Environment3D(BOUNDS, (tall,), CLEARANCE)
        assert math.isclose(env.suggested_step(), 6.0)

    def test_low_wide_cylinder_uses_height(self):
        """Yaricap 20, yukseklik 4: min(20+2, 2+2) = 4 -> adim 2.

        2B kural sadece yaricapa baksaydi adim 11 cikardi ve bu ince
        engel dikeyde atlanabilirdi.
        """
        low = Cylinder(50.0, 50.0, 20.0, 10.0, 14.0)
        env = Environment3D(BOUNDS, (low,), CLEARANCE)
        assert math.isclose(env.suggested_step(), 2.0)

    def test_smallest_obstacle_wins(self):
        big = Cylinder(20.0, 20.0, 30.0, 0.0, 70.0)
        small = Cylinder(70.0, 70.0, 5.0, 0.0, 70.0)
        env = Environment3D(BOUNDS, (big, small), CLEARANCE)
        assert math.isclose(env.suggested_step(), 3.5)   # (5+2)/2

    def test_step_is_positive(self):
        env = Environment3D(BOUNDS, (TOWER,), CLEARANCE)
        assert env.suggested_step() > 0.0


class TestRandomFreePose:
    def _env(self):
        return Environment3D(BOUNDS, (TOWER,), CLEARANCE)

    def test_returns_four_element_pose(self):
        pose = self._env().random_free_pose(random.Random(1))
        assert len(pose) == 4

    def test_pose_is_free(self):
        env = self._env()
        rng = random.Random(2)
        for _ in range(50):
            assert env.is_free(env.random_free_pose(rng)) is True

    def test_pose_is_inside_bounds(self):
        env = self._env()
        rng = random.Random(3)
        for _ in range(50):
            x, y, z, yaw = env.random_free_pose(rng)
            assert 0.0 <= x <= 100.0
            assert 0.0 <= y <= 100.0
            assert 0.0 <= z <= 80.0
            assert 0.0 <= yaw < 2 * math.pi

    def test_same_seed_gives_same_pose(self):
        env = self._env()
        assert (env.random_free_pose(random.Random(9))
                == env.random_free_pose(random.Random(9)))

    def test_altitude_varies(self):
        """z sabit kalmamali; rastgele ornekleniyor olmali."""
        env = self._env()
        rng = random.Random(4)
        zs = {round(env.random_free_pose(rng)[2], 3) for _ in range(30)}
        assert len(zs) > 20

    def test_impossible_map_raises(self):
        """Butun hacmi kaplayan engel: serbest poz bulunamaz."""
        blocker = Cylinder(50.0, 50.0, 500.0, -100.0, 500.0)
        env = Environment3D(BOUNDS, (blocker,), CLEARANCE)
        with pytest.raises(RuntimeError):
            env.random_free_pose(random.Random(5), max_attempts=20)


# --- Arazi zemini ---

# 3x3 post, 50 m aralik -> 100x100 m, sinirlarla ayni ayak izi.
# Ortada 200 m'lik tek tepe; kenarlar deniz seviyesi.
#
#   y=100 |    0     0     0
#   y=50  |    0   200     0
#   y=0   |    0     0     0
#           x=0   x=50  x=100
HILL = Terrain(array.array("h", [0, 0, 0, 0, 200, 0, 0, 0, 0]),
               cols=3, rows=3, spacing_x=50.0, spacing_y=50.0)

TALL_BOUNDS = (0.0, 0.0, 0.0, 100.0, 100.0, 400.0)
GROUND_CLEARANCE = 10.0
HILL_ENV = Environment3D(TALL_BOUNDS, (), GROUND_CLEARANCE, HILL)


def _flat_terrain(spacing, level=0, cols=11, rows=11):
    return Terrain(array.array("h", [level] * (cols * rows)),
                   cols, rows, spacing, spacing)


class TestTerrainIsFree:
    def test_no_terrain_keeps_old_behaviour(self):
        """terrain None iken zemin diye bir sey yok, yer seviyesi serbest."""
        env = Environment3D(TALL_BOUNDS, (), 0.0)
        assert env.is_free((50.0, 50.0, 0.0)) is True

    @pytest.mark.parametrize("z, expected", [
        (100.0, False),     # tepenin icinde
        (205.0, False),     # tepenin ustunde ama emniyet payinda
        (210.0, True),      # tam payin sinirinda
        (215.0, True),
    ])
    def test_hill_top_column(self, z, expected):
        assert HILL_ENV.is_free((50.0, 50.0, z)) is expected

    @pytest.mark.parametrize("z, expected", [
        (5.0, False),
        (10.0, True),
        (15.0, True),
    ])
    def test_sea_level_corner(self, z, expected):
        assert HILL_ENV.is_free((0.0, 0.0, z)) is expected

    @pytest.mark.parametrize("z, expected", [(105.0, False), (115.0, True)])
    def test_interpolated_slope(self, z, expected):
        """(25, 50)'de arazi 100 m; ara deger de emniyet payina giriyor."""
        assert HILL_ENV.is_free((25.0, 50.0, z)) is expected

    def test_cylinder_still_applies_above_terrain(self):
        tower = Cylinder(80.0, 20.0, 10.0, 0.0, 400.0)
        env = Environment3D(TALL_BOUNDS, (tower,), GROUND_CLEARANCE, HILL)
        assert env.is_free((80.0, 20.0, 300.0)) is False
        assert env.is_free((20.0, 20.0, 300.0)) is True

    def test_bounds_still_apply(self):
        assert HILL_ENV.is_free((50.0, 50.0, 500.0)) is False

    def test_path_over_the_hill_is_blocked(self):
        """z = 150'de duz gecis tepeye (200 m) carpiyor."""
        line = [(x, 50.0, 150.0) for x in range(0, 101, 5)]
        assert HILL_ENV.is_path_free(line) is False

    def test_path_high_enough_is_free(self):
        line = [(float(x), 50.0, 250.0) for x in range(0, 101, 5)]
        assert HILL_ENV.is_path_free(line) is True


class TestTerrainSuggestedStep:
    def test_fine_terrain_tightens_the_step(self):
        """Post araligi 10 m -> adim 5 m'yi asmamali."""
        env = Environment3D(TALL_BOUNDS, (), 0.0, _flat_terrain(10.0))
        assert env.suggested_step() == pytest.approx(5.0)

    def test_coarse_terrain_does_not_loosen_the_step(self):
        """Post araligi 400 m; arazi kisiti gevsek, eski deger kalmali."""
        env = Environment3D(TALL_BOUNDS, (), 0.0, _flat_terrain(400.0))
        without = Environment3D(TALL_BOUNDS, (), 0.0).suggested_step()
        assert env.suggested_step() == pytest.approx(without)

    def test_obstacle_and_terrain_take_the_smaller(self):
        small = Cylinder(50.0, 50.0, 4.0, 0.0, 40.0)
        env = Environment3D(TALL_BOUNDS, (small,), 0.0, _flat_terrain(400.0))
        assert env.suggested_step() == pytest.approx(2.0)

    def test_no_terrain_is_unchanged(self):
        env = Environment3D(TALL_BOUNDS, (), 0.0)
        assert env.suggested_step() == pytest.approx(10.0)


class TestTerrainRandomFreePose:
    def test_poses_are_above_the_terrain(self):
        rng = random.Random(1)
        for _ in range(200):
            pose = HILL_ENV.random_free_pose(rng)
            assert HILL_ENV.is_free(pose) is True

    def test_returns_four_element_pose(self):
        pose = HILL_ENV.random_free_pose(random.Random(2))
        assert len(pose) == 4
        assert 0.0 <= pose[3] < 2 * math.pi

    def test_terrain_above_the_ceiling_leaves_nothing_free(self):
        buried = Environment3D(TALL_BOUNDS, (), GROUND_CLEARANCE,
                               _flat_terrain(50.0, level=500))
        with pytest.raises(RuntimeError):
            buried.random_free_pose(random.Random(3), max_attempts=50)

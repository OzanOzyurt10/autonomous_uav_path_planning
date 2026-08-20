"""src/environment3d.py icin testler."""

import math
import random

import pytest

from src.environment3d import Cylinder, Environment3D

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

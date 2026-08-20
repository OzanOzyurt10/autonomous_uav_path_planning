"""src/dubins3d.py icin testler."""

import math
import random

import pytest

from src.dubins import DubinsPath, path_length, shortest_path
from src.dubins3d import DubinsPath3D      # her gorevde yeni isim eklenir

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

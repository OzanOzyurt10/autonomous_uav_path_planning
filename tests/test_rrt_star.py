"""src/rrt_star.py icin testler."""

import math
import random

import pytest

from src.dubins import DubinsPath, path_length, shortest_path
from src.environment import Environment, Obstacle
from src.rrt_star import Node, RRTResult

BOUNDS = (0.0, 0.0, 100.0, 60.0)
RHO = 3.0
STEP = 0.5
START = (5.0, 30.0, 0.0)
GOAL = (95.0, 30.0, 0.0)

FREE_ENV = Environment(BOUNDS, (), 0.0)
# ortada duvar: dogrudan Dubins yolu (LSL, 90 m) engelden geciyor
WALL_ENV = Environment(BOUNDS, (Obstacle(50.0, 30.0, 12.0),), 2.0)

# hedefi cepheleyen kapali halka: disaridan ulasilamaz
_GC = (75.0, 30.0)
CAGE_GOAL = (_GC[0], _GC[1], 0.0)
CAGE_ENV = Environment(BOUNDS, tuple(
    Obstacle(_GC[0] + 14.0 * math.cos(2 * math.pi * k / 12),
             _GC[1] + 14.0 * math.sin(2 * math.pi * k / 12), 6.0)
    for k in range(12)), 1.0)


def assert_pose_close(actual, expected, tol=1e-6):
    assert math.isclose(actual[0], expected[0], abs_tol=tol), f"x: {actual} vs {expected}"
    assert math.isclose(actual[1], expected[1], abs_tol=tol), f"y: {actual} vs {expected}"
    dyaw = (actual[2] - expected[2]) % (2 * math.pi)
    dyaw = min(dyaw, 2 * math.pi - dyaw)
    assert dyaw < tol, f"yaw: {actual} vs {expected}"


class TestNode:
    def test_fields_are_positional_in_order(self):
        path = shortest_path(START, GOAL, RHO)
        node = Node((1.0, 2.0, 0.5), 3, 12.5, path)
        assert node.pose == (1.0, 2.0, 0.5)
        assert node.parent == 3
        assert node.cost == 12.5
        assert node.path_from_parent is path

    def test_root_has_no_parent(self):
        root = Node(START, None, 0.0, None)
        assert root.parent is None
        assert root.path_from_parent is None
        assert root.cost == 0.0

    def test_is_frozen(self):
        node = Node(START, None, 0.0, None)
        with pytest.raises(Exception):
            node.cost = 5.0


class TestRRTResult:
    def test_fields_are_positional_in_order(self):
        edges = [shortest_path(START, GOAL, RHO)]
        tree = [Node(START, None, 0.0, None)]
        result = RRTResult(True, edges, 90.0, 17, tree)
        assert result.found is True
        assert result.edges == edges
        assert result.cost == 90.0
        assert result.iterations == 17
        assert result.tree == tree

    def test_failure_shape(self):
        tree = [Node(START, None, 0.0, None)]
        result = RRTResult(False, [], math.inf, 5000, tree)
        assert result.found is False
        assert result.edges == []
        assert result.cost == math.inf
        assert result.tree == tree

    def test_is_frozen(self):
        result = RRTResult(False, [], math.inf, 0, [])
        with pytest.raises(Exception):
            result.found = True

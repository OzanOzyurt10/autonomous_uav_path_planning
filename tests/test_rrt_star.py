"""src/rrt_star.py icin testler."""

import math
import random

import pytest

from src.dubins import DubinsPath, path_length, shortest_path
from src.environment import Environment, Obstacle
from src.rrt_star import Node, RRTResult, _nearest, _try_connect

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


class TestTryConnect:
    def test_free_environment_returns_path(self):
        path = _try_connect(FREE_ENV, START, GOAL, RHO, STEP)
        assert path is not None
        assert_pose_close(path.end_pose(), GOAL)

    def test_returned_path_starts_at_from_pose(self):
        path = _try_connect(FREE_ENV, START, GOAL, RHO, STEP)
        assert path.start == START

    def test_length_matches_dubins(self):
        path = _try_connect(FREE_ENV, START, GOAL, RHO, STEP)
        assert math.isclose(path.length, path_length(START, GOAL, RHO))

    def test_blocked_returns_none(self):
        # ortadaki duvar dogrudan yolu kesiyor
        assert _try_connect(WALL_ENV, START, GOAL, RHO, STEP) is None

    def test_detour_around_wall_is_allowed(self):
        # duvarin ustunden dolasan iki parca temiz olmali
        mid = (50.0, 56.0, 0.0)
        assert _try_connect(WALL_ENV, START, mid, RHO, STEP) is not None
        assert _try_connect(WALL_ENV, mid, GOAL, RHO, STEP) is not None

    def test_returned_path_is_collision_free_at_fine_step(self):
        # planlayici STEP ile karar veriyor; burada cok daha siki bakiyoruz
        mid = (50.0, 56.0, 0.0)
        path = _try_connect(WALL_ENV, START, mid, RHO, STEP)
        assert path is not None
        assert WALL_ENV.is_path_free(path.sample(0.05)) is True

    def test_goal_outside_bounds_returns_none(self):
        outside = (150.0, 30.0, 0.0)
        assert _try_connect(FREE_ENV, START, outside, RHO, STEP) is None

    def test_same_pose_connects(self):
        path = _try_connect(FREE_ENV, START, START, RHO, STEP)
        assert path is not None
        assert math.isclose(path.length, 0.0, abs_tol=1e-9)


def _node(pose):
    return Node(pose, None, 0.0, None)


class TestNearest:
    def test_single_node_returns_zero(self):
        assert _nearest([_node(START)], GOAL, RHO) == 0

    def test_picks_minimum_dubins_distance(self):
        nodes = [_node((0.0, 0.0, 0.0)),
                 _node((50.0, 0.0, 0.0)),
                 _node((90.0, 0.0, 0.0))]
        target = (95.0, 0.0, 0.0)
        assert _nearest(nodes, target, RHO) == 2

    def test_dubins_not_euclidean(self):
        """Oklid'e gore yakin ama ters bakan dugum secilmemeli."""
        target = (10.0, 0.0, 0.0)
        near_but_backwards = _node((5.0, 0.0, math.pi))   # oklid 5, dubins ~13
        far_but_aligned = _node((0.0, 0.0, 0.0))          # oklid 10, dubins 10
        nodes = [near_but_backwards, far_but_aligned]
        assert _nearest(nodes, target, 2.0) == 1

    def test_direction_is_from_tree_to_target(self):
        """Mesafe gidis yonunde olculmeli; Dubins simetrik degildir."""
        # P=(0,0,0) -> Q=(0,3,pi/2) = 14.857 m,  Q -> P = 11.661 m
        p, q = (0.0, 0.0, 0.0), (0.0, 3.0, math.pi / 2)
        assert not math.isclose(path_length(p, q, 2.0), path_length(q, p, 2.0))

        # agacta P ve uzak bir dugum var; hedef Q.
        # dogru yon P->Q olculmeli
        far = _node((60.0, 60.0, 0.0))
        nodes = [_node(p), far]
        assert _nearest(nodes, q, 2.0) == 0

    def test_returns_index_not_node(self):
        result = _nearest([_node(START), _node(GOAL)], GOAL, RHO)
        assert isinstance(result, int)

    def test_ties_pick_first(self):
        nodes = [_node((0.0, 0.0, 0.0)), _node((0.0, 0.0, 0.0))]
        assert _nearest(nodes, (10.0, 0.0, 0.0), RHO) == 0

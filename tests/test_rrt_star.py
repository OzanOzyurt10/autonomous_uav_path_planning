"""src/rrt_star.py icin testler."""

import math
import random

import pytest

from src.dubins import DubinsPath, path_length, shortest_path
from src.environment import Environment, Obstacle
from src.rrt_star import (Node, RRTResult, _choose_parent, _extract_path,
                          _nearest, _neighbour_radius, _neighbours, _sample,
                          _try_connect, plan)

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


class TestSample:
    def test_full_bias_always_returns_goal(self):
        rng = random.Random(1)
        for _ in range(50):
            assert _sample(FREE_ENV, GOAL, rng, 1.0) == GOAL

    def test_zero_bias_never_returns_goal(self):
        rng = random.Random(2)
        for _ in range(50):
            assert _sample(FREE_ENV, GOAL, rng, 0.0) != GOAL

    def test_zero_bias_returns_free_pose(self):
        rng = random.Random(3)
        for _ in range(50):
            assert WALL_ENV.is_free(_sample(WALL_ENV, GOAL, rng, 0.0)) is True

    def test_partial_bias_mixes(self):
        rng = random.Random(4)
        samples = [_sample(FREE_ENV, GOAL, rng, 0.5) for _ in range(200)]
        goal_count = sum(1 for s in samples if s == GOAL)
        assert 60 < goal_count < 140       # 0.5 etrafinda genis bir bant

    def test_same_seed_gives_same_sequence(self):
        first = [_sample(FREE_ENV, GOAL, random.Random(9), 0.3) for _ in range(1)]
        second = [_sample(FREE_ENV, GOAL, random.Random(9), 0.3) for _ in range(1)]
        assert first == second

    def test_returns_three_element_pose(self):
        assert len(_sample(FREE_ENV, GOAL, random.Random(5), 0.0)) == 3


class TestExtractPath:
    def _chain(self):
        """Uc dugumluk zincir: A -> B -> C, ve C'den hedefe bir kenar."""
        a = (0.0, 0.0, 0.0)
        b = (20.0, 0.0, 0.0)
        c = (40.0, 0.0, 0.0)
        ab = shortest_path(a, b, RHO)
        bc = shortest_path(b, c, RHO)
        goal_edge = shortest_path(c, GOAL, RHO)
        nodes = [
            Node(a, None, 0.0, None),
            Node(b, 0, ab.length, ab),
            Node(c, 1, ab.length + bc.length, bc),
        ]
        return nodes, goal_edge, [ab, bc, goal_edge]

    def test_returns_edges_in_travel_order(self):
        nodes, goal_edge, expected = self._chain()
        assert _extract_path(nodes, 2, goal_edge) == expected

    def test_first_edge_starts_at_root(self):
        nodes, goal_edge, _ = self._chain()
        edges = _extract_path(nodes, 2, goal_edge)
        assert edges[0].start == nodes[0].pose

    def test_last_edge_is_goal_edge(self):
        nodes, goal_edge, _ = self._chain()
        assert _extract_path(nodes, 2, goal_edge)[-1] is goal_edge

    def test_chain_is_continuous(self):
        nodes, goal_edge, _ = self._chain()
        edges = _extract_path(nodes, 2, goal_edge)
        for first, second in zip(edges, edges[1:]):
            assert_pose_close(first.end_pose(), second.start)

    def test_from_root_gives_only_goal_edge(self):
        nodes, goal_edge, _ = self._chain()
        assert _extract_path(nodes, 0, goal_edge) == [goal_edge]

    def test_middle_node_skips_later_edges(self):
        nodes, goal_edge, expected = self._chain()
        assert _extract_path(nodes, 1, goal_edge) == [expected[0], goal_edge]


class TestPlanValidation:
    @pytest.mark.parametrize("bad_rho", [0.0, -1.0])
    def test_bad_rho_raises(self, bad_rho):
        with pytest.raises(ValueError):
            plan(START, GOAL, FREE_ENV, bad_rho, rng=random.Random(1))

    @pytest.mark.parametrize("bad_iters", [0, -5])
    def test_bad_max_iterations_raises(self, bad_iters):
        with pytest.raises(ValueError):
            plan(START, GOAL, FREE_ENV, RHO, max_iterations=bad_iters,
                 rng=random.Random(1))

    @pytest.mark.parametrize("bad_bias", [-0.1, 1.1])
    def test_bad_goal_bias_raises(self, bad_bias):
        with pytest.raises(ValueError):
            plan(START, GOAL, FREE_ENV, RHO, goal_bias=bad_bias,
                 rng=random.Random(1))

    @pytest.mark.parametrize("bad_step", [0.0, -1.0])
    def test_bad_step_raises(self, bad_step):
        with pytest.raises(ValueError):
            plan(START, GOAL, FREE_ENV, RHO, step=bad_step,
                 rng=random.Random(1))

    def test_blocked_start_raises(self):
        blocked = (50.0, 30.0, 0.0)      # duvarin tam ortasi
        with pytest.raises(ValueError):
            plan(blocked, GOAL, WALL_ENV, RHO, rng=random.Random(1))

    def test_blocked_goal_raises(self):
        blocked = (50.0, 30.0, 0.0)
        with pytest.raises(ValueError):
            plan(START, blocked, WALL_ENV, RHO, rng=random.Random(1))

    def test_out_of_bounds_start_raises(self):
        with pytest.raises(ValueError):
            plan((150.0, 30.0, 0.0), GOAL, FREE_ENV, RHO, rng=random.Random(1))


class TestPlanFreeSpace:
    def test_finds_route_in_first_iteration(self):
        # engelsiz ortamda her baglanti basarili; ilk iterasyonda biter
        result = plan(START, GOAL, FREE_ENV, RHO, step=STEP,
                      rng=random.Random(1))
        assert result.found is True
        assert result.iterations == 1

    def test_route_reaches_goal(self):
        result = plan(START, GOAL, FREE_ENV, RHO, step=STEP,
                      rng=random.Random(1))
        assert_pose_close(result.edges[0].start, START)
        assert_pose_close(result.edges[-1].end_pose(), GOAL)

    def test_goal_bias_one_gives_direct_route(self):
        """goal_bias=1.0 ile ilk ornek dogrudan hedeftir.

        O zaman kok -> hedef tek anlamli kenar olur; ardindan hedeften
        hedefe sifir uzunluklu bir kapanis kenari eklenir. Toplam maliyet
        dogrudan Dubins mesafesine esit cikar.
        """
        result = plan(START, GOAL, FREE_ENV, RHO, goal_bias=1.0, step=STEP,
                      rng=random.Random(1))
        assert result.found is True
        assert math.isclose(result.cost, path_length(START, GOAL, RHO),
                            abs_tol=1e-9)

    def test_cost_equals_sum_of_edges(self):
        result = plan(START, GOAL, FREE_ENV, RHO, step=STEP,
                      rng=random.Random(1))
        assert math.isclose(result.cost, sum(e.length for e in result.edges))

    def test_tree_contains_root(self):
        result = plan(START, GOAL, FREE_ENV, RHO, step=STEP,
                      rng=random.Random(1))
        assert result.tree[0].pose == START
        assert result.tree[0].parent is None
        assert result.tree[0].cost == 0.0


class TestPlanWithObstacle:
    def _result(self, seed=1):
        return plan(START, GOAL, WALL_ENV, RHO, max_iterations=3000,
                    step=STEP, rng=random.Random(seed))

    def test_finds_a_route(self):
        assert self._result().found is True

    def test_route_needs_more_than_one_edge(self):
        # dogrudan yol engelden geciyordu, yani dolasmak zorunda
        assert len(self._result().edges) >= 2

    def test_every_edge_is_collision_free_at_fine_step(self):
        """Kritik test: planlayici STEP ile karar verdi, biz 0.05 ile bakiyoruz."""
        for edge in self._result().edges:
            assert WALL_ENV.is_path_free(edge.sample(0.05)) is True

    def test_route_starts_at_start_and_ends_at_goal(self):
        edges = self._result().edges
        assert_pose_close(edges[0].start, START)
        assert_pose_close(edges[-1].end_pose(), GOAL)

    def test_route_is_continuous(self):
        edges = self._result().edges
        for first, second in zip(edges, edges[1:]):
            assert_pose_close(first.end_pose(), second.start)

    def test_cost_equals_sum_of_edges(self):
        result = self._result()
        assert math.isclose(result.cost, sum(e.length for e in result.edges))

    def test_cost_is_at_least_direct_dubins_distance(self):
        result = self._result()
        assert result.cost >= path_length(START, GOAL, RHO) - 1e-9

    def test_curvature_never_exceeds_limit(self):
        for edge in self._result().edges:
            points = edge.sample(0.05)
            for a, b in zip(points, points[1:]):
                ds = math.hypot(b[0] - a[0], b[1] - a[1])
                if ds < 1e-12:
                    continue
                dyaw = (b[2] - a[2]) % (2 * math.pi)
                dyaw = min(dyaw, 2 * math.pi - dyaw)
                assert dyaw / ds <= 1.0 / RHO + 1e-3

    def test_same_seed_gives_same_result(self):
        first = self._result(seed=7)
        second = self._result(seed=7)
        assert first.iterations == second.iterations
        assert math.isclose(first.cost, second.cost)
        assert len(first.edges) == len(second.edges)

    def test_tree_parents_are_valid_indices(self):
        tree = self._result().tree
        for i, node in enumerate(tree):
            if node.parent is None:
                assert i == 0
            else:
                assert 0 <= node.parent < i


class TestPlanUnsolvable:
    def _result(self):
        return plan(START, CAGE_GOAL, CAGE_ENV, RHO, max_iterations=300,
                    step=0.3, rng=random.Random(1))

    def test_reports_not_found(self):
        assert self._result().found is False

    def test_uses_full_budget(self):
        assert self._result().iterations == 300

    def test_edges_empty_and_cost_infinite(self):
        result = self._result()
        assert result.edges == []
        assert result.cost == math.inf

    def test_tree_is_returned_for_debugging(self):
        # agac bos donmemeli; nereye kadar yayildigini gorebilmeliyiz
        assert len(self._result().tree) >= 1


class TestNeighbourRadius:
    def test_small_tree_uses_cap(self):
        assert _neighbour_radius(0, 60.0, 30.0) == 30.0
        assert _neighbour_radius(1, 60.0, 30.0) == 30.0

    def test_shrinks_as_tree_grows(self):
        radii = [_neighbour_radius(n, 60.0, 30.0) for n in (50, 200, 1000)]
        assert radii == sorted(radii, reverse=True)

    def test_never_exceeds_cap(self):
        for n in range(2, 500):
            assert _neighbour_radius(n, 60.0, 30.0) <= 30.0

    def test_known_values(self):
        assert math.isclose(_neighbour_radius(50, 60.0, 30.0), 25.66, abs_tol=0.01)
        assert math.isclose(_neighbour_radius(200, 60.0, 30.0), 17.89, abs_tol=0.01)
        assert math.isclose(_neighbour_radius(1000, 60.0, 30.0), 11.43, abs_tol=0.01)

    def test_gamma_scales_radius(self):
        small = _neighbour_radius(500, 30.0, 1000.0)
        large = _neighbour_radius(500, 60.0, 1000.0)
        assert math.isclose(large, 2 * small)


class TestNeighbours:
    def _line(self):
        """x ekseninde 0, 10, 20, 30 metrede dort dugum."""
        return [_node((float(10 * k), 0.0, 0.0)) for k in range(4)]

    def test_returns_indices_within_radius(self):
        assert _neighbours(self._line(), (0.0, 0.0, 0.0), 15.0) == [0, 1]

    def test_boundary_node_is_included(self):
        # tam 10.0 metredeki dugum 10.0 yaricapta disarida kalmamali
        assert 1 in _neighbours(self._line(), (0.0, 0.0, 0.0), 10.0)

    def test_zero_radius_gives_only_coincident(self):
        assert _neighbours(self._line(), (0.0, 0.0, 0.0), 0.0) == [0]

    def test_large_radius_gives_all(self):
        assert _neighbours(self._line(), (0.0, 0.0, 0.0), 1000.0) == [0, 1, 2, 3]

    def test_ignores_yaw(self):
        """Oklid on eleme yalnizca konuma bakar; yaw farki elemez."""
        nodes = [_node((0.0, 0.0, 0.0)), _node((0.0, 0.0, math.pi))]
        assert _neighbours(nodes, (0.0, 0.0, 0.0), 1.0) == [0, 1]

    def test_returns_indices_not_nodes(self):
        result = _neighbours(self._line(), (0.0, 0.0, 0.0), 1000.0)
        assert all(isinstance(i, int) for i in result)

    def test_empty_when_all_far(self):
        assert _neighbours(self._line(), (500.0, 500.0, 0.0), 5.0) == []


# Yon testi icin dogrulanmis pozlar (rho=3.0):
#   d(A->P) = 11.37   d(B->P) = 22.49      ileri yon: A kazanir
#   d(P->A) = 20.79   d(P->B) = 15.42      ters yon:  B kazanirdi
DIR_A = (12.2, 16.6, 1.25 * math.pi)
DIR_B = (4.9, 6.5, 0.25 * math.pi)
DIR_P = (3.8, 11.4, 0.75 * math.pi)


class TestChooseParent:
    def _fallback(self, nodes, index, pose):
        edge = shortest_path(nodes[index].pose, pose, RHO)
        return (index, edge)

    def test_no_candidates_returns_fallback(self):
        nodes = [_node(START)]
        fb = self._fallback(nodes, 0, GOAL)
        assert _choose_parent(FREE_ENV, nodes, GOAL, [], RHO, STEP, fb) == fb

    def test_picks_cheapest_total_cost(self):
        """Maliyet dugumun kendi maliyeti + oraya giden kenar."""
        near = Node((40.0, 30.0, 0.0), None, 100.0, None)   # yakin ama pahali
        far = Node((10.0, 30.0, 0.0), None, 0.0, None)      # uzak ama ucuz
        nodes = [near, far]
        fb = self._fallback(nodes, 0, (60.0, 30.0, 0.0))
        parent, _ = _choose_parent(FREE_ENV, nodes, (60.0, 30.0, 0.0),
                                   [0, 1], RHO, STEP, fb)
        assert parent == 1

    def test_direction_is_candidate_to_new_pose(self):
        """Yanlis yon kullanilsaydi B secilirdi."""
        nodes = [_node(DIR_A), _node(DIR_B)]
        fb = self._fallback(nodes, 1, DIR_P)
        parent, _ = _choose_parent(FREE_ENV, nodes, DIR_P, [0, 1], RHO, STEP, fb)
        assert parent == 0

    def test_returned_edge_matches_returned_parent(self):
        nodes = [_node(DIR_A), _node(DIR_B)]
        fb = self._fallback(nodes, 1, DIR_P)
        parent, edge = _choose_parent(FREE_ENV, nodes, DIR_P, [0, 1], RHO, STEP, fb)
        assert edge.start == nodes[parent].pose
        assert_pose_close(edge.end_pose(), DIR_P)

    def test_blocked_candidate_is_skipped(self):
        """Duvarin ardindaki ucuz aday secilemez; fallback kalir."""
        blocked = Node((5.0, 30.0, 0.0), None, 0.0, None)
        ok = Node((80.0, 30.0, 0.0), None, 50.0, None)
        nodes = [blocked, ok]
        target = (95.0, 30.0, 0.0)
        fb = self._fallback(nodes, 1, target)
        parent, _ = _choose_parent(WALL_ENV, nodes, target, [0, 1], RHO, STEP, fb)
        assert parent == 1

    def test_fallback_wins_when_no_candidate_is_better(self):
        cheap_fallback = Node(START, None, 0.0, None)
        expensive = Node((50.0, 30.0, 0.0), None, 500.0, None)
        nodes = [cheap_fallback, expensive]
        fb = self._fallback(nodes, 0, GOAL)
        parent, edge = _choose_parent(FREE_ENV, nodes, GOAL, [1], RHO, STEP, fb)
        assert parent == 0
        assert edge is fb[1]

"""src/rrt_star3d.py icin testler."""

import functools
import math
import random

import pytest

from src.dubins3d import DubinsPath3D, airplane_length, airplane_path
from src.environment3d import Cylinder, Environment3D
from src.rrt_star3d import (Node3, RRTResult3, _extract_path, _nearest,
                            _choose_parent, _neighbour_radius, _neighbours,
                            _propagate_cost, _rewire, _sample, _steer,
                            _try_connect, plan3d, shortcut)

BOUNDS = (0.0, 0.0, 0.0, 100.0, 100.0, 80.0)
RHO = 5.0
GAMMA_MAX = math.radians(15.0)
STEP = 2.0
# Duvar y=50'de; baslangic ve hedef iki yaninda, duvarin icinde degil
START = (10.0, 10.0, 40.0, 0.0)
GOAL = (90.0, 90.0, 40.0, 0.0)

FREE_ENV = Environment3D(BOUNDS, (), 0.0)


def _wall(z_max):
    """y=50'de, x boyunca uzanan, z_max yuksekliginde dolu duvar."""
    return tuple(Cylinder(float(x), 50.0, 8.0, 0.0, z_max)
                 for x in range(-8, 109, 10))


# Tavana kadar cikan duvar: rota yok
TALL_ENV = Environment3D(BOUNDS, _wall(80.0), 1.0)
# Alcak duvar: uzerinden ucularak gecilir
SHORT_ENV = Environment3D(BOUNDS, _wall(20.0), 1.0)


def _node(pose):
    return Node3(pose, None, 0.0, None)


def assert_pose3_close(actual, expected, tol=1e-6):
    for i, ad in enumerate(("x", "y", "z")):
        assert math.isclose(actual[i], expected[i], abs_tol=tol), \
            f"{ad}: {actual} vs {expected}"
    dyaw = (actual[3] - expected[3]) % (2 * math.pi)
    dyaw = min(dyaw, 2 * math.pi - dyaw)
    assert dyaw < tol, f"yaw: {actual} vs {expected}"


class TestNode3:
    def test_fields_are_positional_in_order(self):
        path = airplane_path(START, GOAL, RHO, GAMMA_MAX)
        node = Node3((1.0, 2.0, 3.0, 0.5), 3, 12.5, path)
        assert node.pose == (1.0, 2.0, 3.0, 0.5)
        assert node.parent == 3
        assert node.cost == 12.5
        assert node.path_from_parent is path

    def test_root_has_no_parent(self):
        root = Node3(START, None, 0.0, None)
        assert root.parent is None
        assert root.path_from_parent is None
        assert root.cost == 0.0

    def test_is_frozen(self):
        with pytest.raises(Exception):
            Node3(START, None, 0.0, None).cost = 5.0


class TestRRTResult3:
    def test_fields_are_positional_in_order(self):
        edges = [airplane_path(START, GOAL, RHO, GAMMA_MAX)]
        tree = [Node3(START, None, 0.0, None)]
        result = RRTResult3(True, edges, 90.0, 17, tree)
        assert result.found is True
        assert result.edges == edges
        assert result.cost == 90.0
        assert result.iterations == 17
        assert result.tree == tree

    def test_failure_shape(self):
        tree = [Node3(START, None, 0.0, None)]
        result = RRTResult3(False, [], math.inf, 5000, tree)
        assert result.found is False
        assert result.edges == []
        assert result.cost == math.inf
        assert result.tree == tree

    def test_is_frozen(self):
        with pytest.raises(Exception):
            RRTResult3(False, [], math.inf, 0, []).found = True


class TestTryConnect3D:
    def test_free_environment_returns_path(self):
        path = _try_connect(FREE_ENV, START, GOAL, RHO, GAMMA_MAX, STEP)
        assert path is not None
        assert_pose3_close(path.end_pose(), GOAL)

    def test_tall_wall_blocks(self):
        assert _try_connect(TALL_ENV, START, GOAL, RHO, GAMMA_MAX, STEP) is None

    def test_short_wall_is_flown_over(self):
        """Duvar 20 m, ucus irtifasi 40 m: gecebilmeli."""
        path = _try_connect(SHORT_ENV, START, GOAL, RHO, GAMMA_MAX, STEP)
        assert path is not None

    def test_out_of_bounds_goal_returns_none(self):
        outside = (150.0, 50.0, 40.0, 0.0)
        assert _try_connect(FREE_ENV, START, outside, RHO, GAMMA_MAX,
                            STEP) is None

    def test_altitude_ceiling_blocks(self):
        """Tavanin ustunde bir hedef ulasilamaz."""
        too_high = (90.0, 50.0, 200.0, 0.0)
        assert _try_connect(FREE_ENV, START, too_high, RHO, GAMMA_MAX,
                            STEP) is None

    def test_same_pose_connects(self):
        path = _try_connect(FREE_ENV, START, START, RHO, GAMMA_MAX, STEP)
        assert path is not None
        assert math.isclose(path.length, 0.0, abs_tol=1e-9)


class TestNearest3D:
    def test_single_node_returns_zero(self):
        assert _nearest([_node(START)], GOAL, RHO, GAMMA_MAX) == 0

    def test_picks_minimum_airplane_distance(self):
        nodes = [_node((0.0, 50.0, 40.0, 0.0)),
                 _node((50.0, 50.0, 40.0, 0.0)),
                 _node((85.0, 50.0, 40.0, 0.0))]
        assert _nearest(nodes, GOAL, RHO, GAMMA_MAX) == 2

    def test_altitude_counts(self):
        """Yatayda ayni, irtifada uzak dugum secilmemeli."""
        near = _node((80.0, 50.0, 40.0, 0.0))
        far = _node((80.0, 50.0, 5.0, 0.0))
        assert _nearest([far, near], GOAL, RHO, GAMMA_MAX) == 1

    def test_direction_is_from_tree_to_target(self):
        a = (0.0, 0.0, 0.0, 0.0)
        b = (0.0, 3.0, 5.0, math.pi / 2)
        assert not math.isclose(airplane_length(a, b, 2.0, GAMMA_MAX),
                                airplane_length(b, a, 2.0, GAMMA_MAX))

    def test_returns_index_not_node(self):
        assert isinstance(_nearest([_node(START), _node(GOAL)], GOAL, RHO,
                                   GAMMA_MAX), int)

    def test_ties_pick_first(self):
        nodes = [_node(START), _node(START)]
        assert _nearest(nodes, GOAL, RHO, GAMMA_MAX) == 0


class TestSample3D:
    def test_full_bias_always_returns_goal(self):
        rng = random.Random(1)
        for _ in range(20):
            assert _sample(FREE_ENV, GOAL, rng, 1.0) == GOAL

    def test_zero_bias_never_returns_goal(self):
        rng = random.Random(2)
        for _ in range(20):
            assert _sample(FREE_ENV, GOAL, rng, 0.0) != GOAL

    def test_zero_bias_returns_free_pose(self):
        rng = random.Random(3)
        for _ in range(20):
            assert SHORT_ENV.is_free(_sample(SHORT_ENV, GOAL, rng, 0.0)) is True

    def test_returns_four_element_pose(self):
        assert len(_sample(FREE_ENV, GOAL, random.Random(5), 0.0)) == 4


class TestExtractPath3D:
    def _chain(self):
        a = (0.0, 50.0, 40.0, 0.0)
        b = (30.0, 50.0, 40.0, 0.0)
        c = (60.0, 50.0, 40.0, 0.0)
        ab = airplane_path(a, b, RHO, GAMMA_MAX)
        bc = airplane_path(b, c, RHO, GAMMA_MAX)
        goal_edge = airplane_path(c, GOAL, RHO, GAMMA_MAX)
        nodes = [
            Node3(a, None, 0.0, None),
            Node3(b, 0, ab.length, ab),
            Node3(c, 1, ab.length + bc.length, bc),
        ]
        return nodes, goal_edge, [ab, bc, goal_edge]

    def test_returns_edges_in_travel_order(self):
        nodes, goal_edge, expected = self._chain()
        assert _extract_path(nodes, 2, goal_edge) == expected

    def test_last_edge_is_goal_edge(self):
        nodes, goal_edge, _ = self._chain()
        assert _extract_path(nodes, 2, goal_edge)[-1] is goal_edge

    def test_chain_is_continuous(self):
        nodes, goal_edge, _ = self._chain()
        edges = _extract_path(nodes, 2, goal_edge)
        for first, second in zip(edges, edges[1:]):
            assert_pose3_close(first.end_pose(), second.start)

    def test_from_root_gives_only_goal_edge(self):
        nodes, goal_edge, _ = self._chain()
        assert _extract_path(nodes, 0, goal_edge) == [goal_edge]


class TestNeighbourRadius3D:
    def test_small_tree_uses_cap(self):
        assert _neighbour_radius(0, 80.0, 30.0) == 30.0
        assert _neighbour_radius(1, 80.0, 30.0) == 30.0

    def test_shrinks_as_tree_grows(self):
        radii = [_neighbour_radius(n, 80.0, 1000.0) for n in (100, 500, 2000)]
        assert radii == sorted(radii, reverse=True)

    def test_never_exceeds_cap(self):
        for n in range(2, 400):
            assert _neighbour_radius(n, 80.0, 30.0) <= 30.0

    def test_uses_fourth_root_not_cube_root(self):
        """Us 1/4: konfigurasyon uzayi dort boyutlu (x, y, z, yaw)."""
        n = 500
        expected = 80.0 * (math.log(n) / n) ** 0.25
        assert math.isclose(_neighbour_radius(n, 80.0, 1000.0), expected)

    def test_known_values(self):
        assert math.isclose(_neighbour_radius(300, 80.0, 1000.0), 29.7,
                            abs_tol=0.1)
        assert math.isclose(_neighbour_radius(2000, 80.0, 1000.0), 19.9,
                            abs_tol=0.1)


class TestNeighbours3D:
    def _line(self):
        """z ekseninde 0, 10, 20, 30 metrede dort dugum."""
        return [_node((50.0, 50.0, float(10 * k), 0.0)) for k in range(4)]

    def test_returns_indices_within_radius(self):
        assert _neighbours(self._line(), (50.0, 50.0, 0.0, 0.0), 15.0) == [0, 1]

    def test_altitude_counts_in_the_distance(self):
        """Oklid on eleme 3B: z farki da mesafeye giriyor."""
        nodes = [_node((50.0, 50.0, 0.0, 0.0))]
        assert _neighbours(nodes, (50.0, 50.0, 100.0, 0.0), 15.0) == []

    def test_diagonal_distance(self):
        """(3, 4, 12) -> uzaklik 13."""
        nodes = [_node((3.0, 4.0, 12.0, 0.0))]
        assert _neighbours(nodes, (0.0, 0.0, 0.0, 0.0), 13.0) == [0]
        assert _neighbours(nodes, (0.0, 0.0, 0.0, 0.0), 12.9) == []

    def test_ignores_yaw(self):
        nodes = [_node((0.0, 0.0, 0.0, 0.0)), _node((0.0, 0.0, 0.0, math.pi))]
        assert _neighbours(nodes, (0.0, 0.0, 0.0, 0.0), 1.0) == [0, 1]

    def test_large_radius_gives_all(self):
        assert _neighbours(self._line(), (50.0, 50.0, 0.0, 0.0),
                           1000.0) == [0, 1, 2, 3]

    def test_empty_when_all_far(self):
        assert _neighbours(self._line(), (0.0, 0.0, 500.0, 0.0), 5.0) == []


class TestChooseParent3D:
    def _fallback(self, nodes, index, pose):
        return (index, airplane_path(nodes[index].pose, pose, RHO, GAMMA_MAX))

    def test_no_candidates_returns_fallback(self):
        nodes = [_node(START)]
        fb = self._fallback(nodes, 0, GOAL)
        assert _choose_parent(FREE_ENV, nodes, GOAL, [], RHO, GAMMA_MAX,
                              STEP, fb) == fb

    def test_picks_cheapest_total_cost(self):
        near = Node3((60.0, 50.0, 40.0, 0.0), None, 100.0, None)
        far = Node3((20.0, 50.0, 40.0, 0.0), None, 0.0, None)
        nodes = [near, far]
        target = (80.0, 50.0, 40.0, 0.0)
        fb = self._fallback(nodes, 0, target)
        parent, _ = _choose_parent(FREE_ENV, nodes, target, [0, 1], RHO,
                                   GAMMA_MAX, STEP, fb)
        assert parent == 1

    def test_returned_edge_matches_returned_parent(self):
        nodes = [_node((20.0, 50.0, 40.0, 0.0)), _node((60.0, 50.0, 40.0, 0.0))]
        target = (80.0, 50.0, 40.0, 0.0)
        fb = self._fallback(nodes, 0, target)
        parent, edge = _choose_parent(FREE_ENV, nodes, target, [0, 1], RHO,
                                      GAMMA_MAX, STEP, fb)
        assert edge.start == nodes[parent].pose
        assert_pose3_close(edge.end_pose(), target)

    def test_blocked_candidate_is_skipped(self):
        """Duvarin ardindaki ucuz aday secilemez."""
        blocked = Node3((10.0, 50.0, 40.0, 0.0), None, 0.0, None)
        ok = Node3((80.0, 50.0, 40.0, 0.0), None, 300.0, None)
        nodes = [blocked, ok]
        target = (95.0, 50.0, 40.0, 0.0)
        fb = self._fallback(nodes, 1, target)
        parent, _ = _choose_parent(TALL_ENV, nodes, target, [0, 1], RHO,
                                   GAMMA_MAX, STEP, fb)
        assert parent == 1


class TestPropagateCost3D:
    def _chain(self):
        a = (0.0, 50.0, 40.0, 0.0)
        b = (30.0, 50.0, 40.0, 0.0)
        c = (60.0, 50.0, 40.0, 0.0)
        ab = airplane_path(a, b, RHO, GAMMA_MAX)
        bc = airplane_path(b, c, RHO, GAMMA_MAX)
        return [
            Node3(a, None, 0.0, None),
            Node3(b, 0, ab.length, ab),
            Node3(c, 1, ab.length + bc.length, bc),
        ]

    def test_no_change_when_costs_already_correct(self):
        nodes = self._chain()
        before = [n.cost for n in nodes]
        _propagate_cost(nodes, 0)
        assert [n.cost for n in nodes] == before

    def test_updates_whole_subtree(self):
        import dataclasses
        nodes = self._chain()
        nodes[1] = dataclasses.replace(nodes[1], cost=100.0)
        _propagate_cost(nodes, 1)
        assert math.isclose(nodes[2].cost,
                            100.0 + nodes[2].path_from_parent.length)

    def test_geometry_is_not_recomputed(self):
        import dataclasses
        nodes = self._chain()
        edges = [n.path_from_parent for n in nodes]
        nodes[1] = dataclasses.replace(nodes[1], cost=100.0)
        _propagate_cost(nodes, 1)
        assert [n.path_from_parent for n in nodes] == edges


class TestRewire3D:
    def test_rewires_when_cheaper(self):
        neighbour = Node3((60.0, 50.0, 40.0, 0.0), None, 500.0, None)
        new = Node3((20.0, 50.0, 40.0, 0.0), None, 0.0, None)
        nodes = [neighbour, new]
        _rewire(FREE_ENV, nodes, 1, [0], RHO, GAMMA_MAX, STEP)
        assert nodes[0].parent == 1
        assert nodes[0].cost < 500.0
        assert nodes[0].path_from_parent is not None

    def test_leaves_alone_when_not_cheaper(self):
        neighbour = Node3((60.0, 50.0, 40.0, 0.0), None, 1.0, None)
        new = Node3((20.0, 50.0, 40.0, 0.0), None, 0.0, None)
        nodes = [neighbour, new]
        _rewire(FREE_ENV, nodes, 1, [0], RHO, GAMMA_MAX, STEP)
        assert nodes[0].parent is None
        assert nodes[0].cost == 1.0

    def test_skips_the_new_node_itself(self):
        neighbour = Node3((60.0, 50.0, 40.0, 0.0), None, 500.0, None)
        new = Node3((20.0, 50.0, 40.0, 0.0), None, 0.0, None)
        nodes = [neighbour, new]
        _rewire(FREE_ENV, nodes, 1, [0, 1], RHO, GAMMA_MAX, STEP)
        assert nodes[1].parent is None
        assert nodes[1].cost == 0.0

    def test_blocked_edge_prevents_rewire(self):
        left = Node3((10.0, 50.0, 40.0, 0.0), None, 5000.0, None)
        right = Node3((90.0, 50.0, 40.0, 0.0), None, 0.0, None)
        nodes = [left, right]
        _rewire(TALL_ENV, nodes, 1, [0], RHO, GAMMA_MAX, STEP)
        assert nodes[0].parent is None

    def test_new_edge_starts_at_new_node(self):
        neighbour = Node3((60.0, 50.0, 40.0, 0.0), None, 500.0, None)
        new = Node3((20.0, 50.0, 40.0, 0.0), None, 0.0, None)
        nodes = [neighbour, new]
        _rewire(FREE_ENV, nodes, 1, [0], RHO, GAMMA_MAX, STEP)
        assert nodes[0].path_from_parent.start == new.pose


@functools.lru_cache(maxsize=None)
def _short_result(seed, iters):
    """Ayni (seed, iters) icin tek kosu; testler arasinda paylasilir.

    goal_bias varsayilandan yuksek (0.2): testlerin rota bulmasi garantiye
    yakin olsun ve kosu kisa sursun.
    """
    return plan3d(START, GOAL, SHORT_ENV, RHO, GAMMA_MAX,
                  max_iterations=iters, goal_bias=0.2, step=STEP,
                  rng=random.Random(seed))


class TestPlan3DValidation:
    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_rho_raises(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, bad, GAMMA_MAX, max_iterations=5,
                   rng=random.Random(1))

    @pytest.mark.parametrize("bad", [0.0, -0.1, math.pi / 2])
    def test_bad_gamma_max_raises(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, bad, max_iterations=5,
                   rng=random.Random(1))

    @pytest.mark.parametrize("bad", [0, -5])
    def test_bad_max_iterations_raises(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, GAMMA_MAX,
                   max_iterations=bad, rng=random.Random(1))

    @pytest.mark.parametrize("bad", [-0.1, 1.1])
    def test_bad_goal_bias_raises(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, GAMMA_MAX, goal_bias=bad,
                   max_iterations=5, rng=random.Random(1))

    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_step_raises(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, GAMMA_MAX, step=bad,
                   max_iterations=5, rng=random.Random(1))

    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_radius_params_raise(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, GAMMA_MAX, radius_gamma=bad,
                   max_iterations=5, rng=random.Random(1))
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, GAMMA_MAX, radius_cap=bad,
                   max_iterations=5, rng=random.Random(1))

    def test_blocked_start_raises(self):
        blocked = (50.0, 50.0, 10.0, 0.0)      # duvarin icinde
        with pytest.raises(ValueError):
            plan3d(blocked, GOAL, SHORT_ENV, RHO, GAMMA_MAX,
                   max_iterations=5, rng=random.Random(1))

    def test_out_of_bounds_goal_raises(self):
        with pytest.raises(ValueError):
            plan3d(START, (50.0, 50.0, 500.0, 0.0), FREE_ENV, RHO, GAMMA_MAX,
                   max_iterations=5, rng=random.Random(1))


class TestPlan3DAltitudeAdvantage:
    def test_tall_wall_blocks_the_route(self):
        """Tavana kadar cikan duvar: rota yok."""
        result = plan3d(START, GOAL, TALL_ENV, RHO, GAMMA_MAX,
                        max_iterations=120, goal_bias=1.0, step=STEP,
                        rng=random.Random(1))
        assert result.found is False
        assert result.edges == []
        assert result.cost == math.inf
        assert len(result.tree) >= 1

    def test_short_wall_is_flown_over(self):
        """Ayni duvar alcaltilinca ustunden gecilebiliyor.

        3B planlamanin kazandirdigi sey tam bu.
        """
        result = plan3d(START, GOAL, SHORT_ENV, RHO, GAMMA_MAX,
                        max_iterations=5, goal_bias=1.0, step=STEP,
                        rng=random.Random(1), stop_on_first_solution=True)
        assert result.found is True


class TestPlan3DInvariants:
    @pytest.mark.parametrize("seed", [1, 2, 3])
    def test_costs_are_consistent_with_edges(self, seed):
        tree = _short_result(seed, 120).tree
        for node in tree:
            if node.parent is None:
                continue
            expected = tree[node.parent].cost + node.path_from_parent.length
            assert math.isclose(node.cost, expected, abs_tol=1e-9)

    @pytest.mark.parametrize("seed", [1, 2, 3])
    def test_no_cycles(self, seed):
        tree = _short_result(seed, 120).tree
        for i in range(len(tree)):
            steps = 0
            j = i
            while tree[j].parent is not None:
                j = tree[j].parent
                steps += 1
                assert steps <= len(tree)
            assert j == 0

    def test_root_is_untouched(self):
        root = _short_result(1, 120).tree[0]
        assert root.parent is None
        assert root.cost == 0.0
        assert root.path_from_parent is None

    def test_route_is_collision_free_at_fine_step(self):
        result = _short_result(1, 120)
        assert result.found is True
        for edge in result.edges:
            assert SHORT_ENV.is_path_free(edge.sample(0.2)) is True

    def test_route_is_continuous(self):
        edges = _short_result(1, 120).edges
        for first, second in zip(edges, edges[1:]):
            assert_pose3_close(first.end_pose(), second.start)

    def test_route_starts_at_start_and_ends_at_goal(self):
        edges = _short_result(1, 120).edges
        assert_pose3_close(edges[0].start, START)
        assert_pose3_close(edges[-1].end_pose(), GOAL)

    def test_cost_equals_sum_of_edges(self):
        result = _short_result(1, 120)
        assert math.isclose(result.cost, sum(e.length for e in result.edges))

    def test_climb_angle_never_exceeds_limit(self):
        """Her kenarin gamma'si limit icinde olmali."""
        for edge in _short_result(1, 120).edges:
            assert abs(edge.gamma) <= GAMMA_MAX + 1e-12

    def test_same_seed_gives_same_result(self):
        first = _short_result(7, 120)
        second = _short_result(7, 120)
        assert math.isclose(first.cost, second.cost)
        assert len(first.edges) == len(second.edges)


# --- Oklid alt siniriyla budama: sonuc birebir ayni kalmali ---

GOLDEN = [
    # (tohum, dugum sayisi, kenar sayisi, maliyet) - budama oncesi olculdu
    (1, 94, 2, 113.96046635578995),
    (2, 107, 2, 114.75826421721456),
    (3, 104, 2, 146.41890804122727),
]


def _brute_nearest(nodes, target, rho, gamma_max):
    """Budamasiz referans: butun dugumler icin Dubins cozup en kucugu alir."""
    best_index = None
    best_dist = None
    for i in range(len(nodes)):
        dist = airplane_length(nodes[i].pose, target, rho, gamma_max)
        if best_index is None or dist < best_dist:
            best_index = i
            best_dist = dist
    return best_index


class TestPruningKeepsResults:
    @pytest.mark.parametrize("seed, nodes, edges, cost", GOLDEN)
    def test_plan_output_is_unchanged(self, seed, nodes, edges, cost):
        """Budama bir yaklasiklik degil; sayilar tam eslesmeli."""
        result = plan3d(START, GOAL, SHORT_ENV, RHO, GAMMA_MAX,
                        max_iterations=120, goal_bias=0.2, step=STEP,
                        rng=random.Random(seed))
        assert len(result.tree) == nodes
        assert len(result.edges) == edges
        assert result.cost == cost

    @pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
    def test_nearest_matches_brute_force(self, seed):
        """Budanmis _nearest, budamasiz referansla ayni indeksi vermeli."""
        rng = random.Random(seed)
        tree = [_node((rng.uniform(0, 100), rng.uniform(0, 100),
                       rng.uniform(0, 80), rng.uniform(0, 2 * math.pi)))
                for _ in range(60)]
        for _ in range(20):
            target = (rng.uniform(0, 100), rng.uniform(0, 100),
                      rng.uniform(0, 80), rng.uniform(0, 2 * math.pi))
            assert (_nearest(tree, target, RHO, GAMMA_MAX)
                    == _brute_nearest(tree, target, RHO, GAMMA_MAX))

    def test_nearest_handles_ties_like_brute_force(self):
        """Ayni pozdan iki dugum: ikisi de ilkini secmeli."""
        tree = [_node(START), _node(START), _node(GOAL)]
        assert (_nearest(tree, GOAL, RHO, GAMMA_MAX)
                == _brute_nearest(tree, GOAL, RHO, GAMMA_MAX))

    def test_nearest_single_node(self):
        tree = [_node(START)]
        assert _nearest(tree, GOAL, RHO, GAMMA_MAX) == 0


class TestSteer:
    """Adimli ilerleme: uzak ornege tam yol yerine ara poza gidilir."""

    def test_none_returns_target_unchanged(self):
        assert _steer(START, GOAL, RHO, GAMMA_MAX, None) is GOAL

    def test_short_path_returns_target_unchanged(self):
        near = (14.0, 12.0, 40.0, 0.0)
        assert _steer(START, near, RHO, GAMMA_MAX, 500.0) is near

    def test_long_path_is_truncated_to_a_pose_on_it(self):
        path = airplane_path(START, GOAL, RHO, GAMMA_MAX)
        assert path.length > 40.0
        result = _steer(START, GOAL, RHO, GAMMA_MAX, 40.0)
        assert_pose3_close(result, path.interpolate(40.0))

    def test_result_is_four_element_pose(self):
        assert len(_steer(START, GOAL, RHO, GAMMA_MAX, 30.0)) == 4

    def test_truncated_target_is_closer_than_the_original(self):
        result = _steer(START, GOAL, RHO, GAMMA_MAX, 30.0)
        assert (airplane_length(START, result, RHO, GAMMA_MAX)
                < airplane_length(START, GOAL, RHO, GAMMA_MAX))

    def test_exact_length_is_not_truncated(self):
        path = airplane_path(START, GOAL, RHO, GAMMA_MAX)
        assert _steer(START, GOAL, RHO, GAMMA_MAX, path.length) is GOAL


class TestPlan3DSteering:
    def _run(self, max_edge_length, seed=1, iters=120):
        return plan3d(START, GOAL, SHORT_ENV, RHO, GAMMA_MAX,
                      max_iterations=iters, goal_bias=0.2, step=STEP,
                      rng=random.Random(seed),
                      max_edge_length=max_edge_length)

    @pytest.mark.parametrize("bad", [0.0, -1.0])
    def test_bad_max_edge_length_raises(self, bad):
        with pytest.raises(ValueError):
            plan3d(START, GOAL, FREE_ENV, RHO, GAMMA_MAX, max_iterations=5,
                   rng=random.Random(1), max_edge_length=bad)

    def test_default_is_off(self):
        """Varsayilan None: davranis budama oncesiyle ayni kalmali."""
        assert self._run(None).cost == 113.96046635578995

    def test_steering_still_finds_a_route(self):
        assert self._run(35.0).found is True

    def test_steered_route_is_collision_free(self):
        result = self._run(35.0)
        for edge in result.edges:
            assert SHORT_ENV.is_path_free(edge.sample(0.2)) is True

    def test_steered_route_respects_climb_limit(self):
        for edge in self._run(35.0).edges:
            assert abs(edge.gamma) <= GAMMA_MAX + 1e-12

    def test_steered_tree_costs_stay_consistent(self):
        tree = self._run(35.0).tree
        for node in tree:
            if node.parent is None:
                continue
            expected = tree[node.parent].cost + node.path_from_parent.length
            assert math.isclose(node.cost, expected, abs_tol=1e-9)

    def test_steering_grows_more_nodes(self):
        """Kisa kenarlar daha az reddedilir, ayni butcede agac buyur."""
        assert len(self._run(35.0).tree) >= len(self._run(None).tree)



# --- Rota kisaltma (shortcut) ---

PILLAR_ENV = Environment3D(BOUNDS, (Cylinder(50.0, 50.0, 15.0, 0.0, 80.0),), 1.0)

# Kuzeye kacip geri donen apacik bir dolambac; dogrudan bag 80 m duz cizgi.
DETOUR_POSES = [(10.0, 50.0, 40.0, 0.0),
                (50.0, 90.0, 40.0, 0.0),
                (90.0, 50.0, 40.0, 0.0)]

# Dogrudan bag daha KISA (60.8 < 88.0) ama sutunun icinden geciyor.
PILLAR_POSES = [(20.0, 50.0, 40.0, -math.pi / 4),
                (50.0, 20.0, 40.0, math.pi / 4),
                (80.0, 50.0, 40.0, math.pi / 4)]

# Sutunun guneyinden dolasan 4 pozluk rota. Bas-son bagi engelli oldugu icin
# tek kenara cokemez; ara kestirmeler yine de kazanc verir.
AROUND_POSES = [(15.0, 50.0, 40.0, -math.pi / 4),
                (40.0, 22.0, 40.0, 0.0),
                (62.0, 22.0, 40.0, math.pi / 4),
                (85.0, 50.0, 40.0, math.pi / 4)]

# Aramayla bulundu: A->C dogrudan bagi helis turu atmak zorunda kaldigi icin
# A->B->C toplamindan 29.5 m UZUN. Carpisma yok, yine de kestirme gecersiz.
LONGER_POSES = [(18.049, 27.6453, 3.8886, 1.8069),
                (70.2343, 79.198, 35.7791, 3.6268),
                (28.8598, 32.4463, 60.8594, 4.3809)]


def _route(poses, env=FREE_ENV):
    """Poz dizisinden kenar listesi; her kenarin serbest oldugu dogrulanir."""
    edges = [airplane_path(a, b, RHO, GAMMA_MAX)
             for a, b in zip(poses, poses[1:])]
    for edge in edges:
        assert env.is_path_free(edge.sample(STEP)), "test rotasi zaten engelli"
    return edges


def _total(edges):
    return sum(edge.length for edge in edges)


class TestShortcut:
    def test_empty_route_stays_empty(self):
        assert shortcut([], FREE_ENV, RHO, GAMMA_MAX, STEP) == []

    def test_single_edge_is_unchanged(self):
        edges = _route(DETOUR_POSES[:2])
        out = shortcut(edges, FREE_ENV, RHO, GAMMA_MAX, STEP)
        assert len(out) == 1
        assert math.isclose(_total(out), _total(edges))

    def test_collapses_an_obvious_detour(self):
        edges = _route(DETOUR_POSES)
        out = shortcut(edges, FREE_ENV, RHO, GAMMA_MAX, STEP)
        assert len(out) == 1
        assert math.isclose(_total(out), 80.0, abs_tol=1e-6)

    def test_blocked_shortcut_is_rejected(self):
        edges = _route(PILLAR_POSES, PILLAR_ENV)
        out = shortcut(edges, PILLAR_ENV, RHO, GAMMA_MAX, STEP)
        assert len(out) == 2
        assert math.isclose(_total(out), _total(edges))

    def test_longer_direct_connection_is_rejected(self):
        """Carpisma yok ama kestirme daha uzun; uzunluk kiyasi sart."""
        edges = _route(LONGER_POSES)
        out = shortcut(edges, FREE_ENV, RHO, GAMMA_MAX, STEP)
        assert _total(out) <= _total(edges) + 1e-9

    def test_shortens_without_collapsing_to_one_edge(self):
        edges = _route(AROUND_POSES, PILLAR_ENV)
        out = shortcut(edges, PILLAR_ENV, RHO, GAMMA_MAX, STEP)
        assert len(out) == 2
        assert _total(out) < _total(edges) - 1.0

    def test_result_is_continuous(self):
        out = shortcut(_route(AROUND_POSES, PILLAR_ENV), PILLAR_ENV, RHO,
                       GAMMA_MAX, STEP)
        for first, second in zip(out, out[1:]):
            assert_pose3_close(first.end_pose(), second.start)

    def test_keeps_start_and_end_pose(self):
        edges = _route(AROUND_POSES, PILLAR_ENV)
        out = shortcut(edges, PILLAR_ENV, RHO, GAMMA_MAX, STEP)
        assert_pose3_close(out[0].start, edges[0].start)
        assert_pose3_close(out[-1].end_pose(), edges[-1].end_pose())

    def test_result_is_collision_free(self):
        out = shortcut(_route(AROUND_POSES, PILLAR_ENV), PILLAR_ENV, RHO,
                       GAMMA_MAX, STEP)
        for edge in out:
            assert PILLAR_ENV.is_path_free(edge.sample(0.2)) is True

    def test_does_not_mutate_input(self):
        edges = _route(DETOUR_POSES)
        before = list(edges)
        shortcut(edges, FREE_ENV, RHO, GAMMA_MAX, STEP)
        assert edges == before

    def test_second_pass_finds_nothing_more(self):
        once = shortcut(_route(AROUND_POSES, PILLAR_ENV), PILLAR_ENV, RHO,
                        GAMMA_MAX, STEP)
        twice = shortcut(once, PILLAR_ENV, RHO, GAMMA_MAX, STEP)
        assert math.isclose(_total(twice), _total(once))

    @pytest.mark.parametrize("seed", [1, 2, 3, 4, 5])
    def test_random_routes_keep_every_invariant(self, seed):
        rng = random.Random(seed)
        poses = [(rng.uniform(20, 80), rng.uniform(20, 80),
                  rng.uniform(20, 60), rng.uniform(0, 2 * math.pi))
                 for _ in range(5)]
        edges = _route(poses)
        out = shortcut(edges, FREE_ENV, RHO, GAMMA_MAX, STEP)
        assert _total(out) <= _total(edges) + 1e-9
        assert_pose3_close(out[0].start, edges[0].start)
        assert_pose3_close(out[-1].end_pose(), edges[-1].end_pose())
        for first, second in zip(out, out[1:]):
            assert_pose3_close(first.end_pose(), second.start)

    def test_real_route_stays_valid(self):
        edges = _short_result(1, 120).edges
        out = shortcut(edges, SHORT_ENV, RHO, GAMMA_MAX, STEP)
        assert _total(out) <= _total(edges) + 1e-9
        for edge in out:
            assert SHORT_ENV.is_path_free(edge.sample(0.2)) is True


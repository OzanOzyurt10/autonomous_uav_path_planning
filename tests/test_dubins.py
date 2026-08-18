"""src/dubins.py icin testler."""

import math

import pytest

from src.dubins import (DubinsPath, _mod2pi, _segment_end, _SOLVERS,
                        _to_canonical)

ANGLES = [-10.0, -math.pi, -0.1, 0.0, 0.1, math.pi, 7.0, 100.0]
HALF_PI = math.pi / 2


def assert_pose_close(actual, expected, tol=1e-6):
    """Iki pozu karsilastirir; yaw farkini 2pi sarmasini dikkate alarak olcer."""
    assert math.isclose(actual[0], expected[0], abs_tol=tol), f"x: {actual} vs {expected}"
    assert math.isclose(actual[1], expected[1], abs_tol=tol), f"y: {actual} vs {expected}"
    dyaw = _mod2pi(actual[2] - expected[2])
    dyaw = min(dyaw, 2 * math.pi - dyaw)
    assert dyaw < tol, f"yaw: {actual} vs {expected}"


class TestMod2Pi:
    def test_maps_into_zero_two_pi(self):
        for theta in ANGLES:
            assert 0.0 <= _mod2pi(theta) < 2 * math.pi

    def test_preserves_angle_identity(self):
        for theta in ANGLES:
            wrapped = _mod2pi(theta)
            assert math.isclose(math.sin(wrapped), math.sin(theta), abs_tol=1e-12)
            assert math.isclose(math.cos(wrapped), math.cos(theta), abs_tol=1e-12)

    def test_zero_stays_zero(self):
        assert _mod2pi(0.0) == 0.0

    def test_negative_small_wraps_near_two_pi(self):
        assert math.isclose(_mod2pi(-0.1), 2 * math.pi - 0.1, abs_tol=1e-12)


class TestDubinsPathBasics:
    def test_length_is_sum_of_segments(self):
        path = DubinsPath(start=(0.0, 0.0, 0.0), word="LSL",
                          lengths=(1.0, 2.0, 3.0), rho=5.0)
        assert math.isclose(path.length, 6.0)

    def test_is_frozen(self):
        path = DubinsPath(start=(0.0, 0.0, 0.0), word="LSL",
                          lengths=(1.0, 2.0, 3.0), rho=5.0)
        with pytest.raises(Exception):
            path.rho = 2.0

    def test_zero_length_path(self):
        path = DubinsPath(start=(1.0, 2.0, 0.5), word="LSL",
                          lengths=(0.0, 0.0, 0.0), rho=5.0)
        assert path.length == 0.0

    def test_fields_are_positional_in_order(self):
        path = DubinsPath((1.0, 2.0, 0.5), "RSR", (1.0, 2.0, 3.0), 4.0)
        assert path.start == (1.0, 2.0, 0.5)
        assert path.word == "RSR"
        assert path.lengths == (1.0, 2.0, 3.0)
        assert path.rho == 4.0


class TestSegmentGeometry:
    def test_straight_moves_along_heading(self):
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "S", 3.0, 1.0), (3.0, 0.0, 0.0))
        assert_pose_close(_segment_end((0.0, 0.0, HALF_PI), "S", 3.0, 1.0), (0.0, 3.0, HALF_PI))

    def test_left_quarter_turn(self):
        # rho=1, ceyrek tur sola (yay uzunlugu pi/2): (0,0,0) -> (1,1,pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "L", HALF_PI, 1.0), (1.0, 1.0, HALF_PI))

    def test_right_quarter_turn(self):
        # rho=1, ceyrek tur saga: (0,0,0) -> (1,-1,-pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "R", HALF_PI, 1.0), (1.0, -1.0, -HALF_PI))

    def test_turn_radius_is_respected(self):
        # rho=5 ile ceyrek tur sola: (0,0,0) -> (5,5,pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "L", HALF_PI * 5.0, 5.0), (5.0, 5.0, HALF_PI))

    def test_full_left_circle_returns_to_start(self):
        rho, start = 2.5, (3.0, -1.0, 0.7)
        assert_pose_close(_segment_end(start, "L", 2 * math.pi * rho, rho), start)

    def test_full_right_circle_returns_to_start(self):
        rho, start = 2.5, (3.0, -1.0, 0.7)
        assert_pose_close(_segment_end(start, "R", 2 * math.pi * rho, rho), start)

    def test_zero_length_is_identity(self):
        start = (3.0, -1.0, 0.7)
        for mode in "LSR":
            assert_pose_close(_segment_end(start, mode, 0.0, 2.0), start)

    def test_unknown_mode_raises(self):
        with pytest.raises(ValueError):
            _segment_end((0.0, 0.0, 0.0), "X", 1.0, 1.0)


class TestInterpolateAndSample:
    def _path(self):
        # rho=1: sola ceyrek tur, 2 birim duz, saga ceyrek tur
        return DubinsPath(start=(0.0, 0.0, 0.0), word="LSR",
                          lengths=(HALF_PI, 2.0, HALF_PI), rho=1.0)

    def test_interpolate_at_zero_is_start(self):
        p = self._path()
        assert_pose_close(p.interpolate(0.0), p.start)

    def test_known_waypoints(self):
        p = self._path()
        # birinci segmentin sonu: ceyrek tur sola
        assert_pose_close(p.interpolate(HALF_PI), (1.0, 1.0, HALF_PI))
        # ikinci segmentin sonu: 2 birim kuzeye duz
        assert_pose_close(p.interpolate(HALF_PI + 2.0), (1.0, 3.0, HALF_PI))
        # ucuncu segmentin sonu: ceyrek tur saga
        assert_pose_close(p.end_pose(), (2.0, 4.0, 0.0))

    def test_midpoint_of_first_segment(self):
        p = self._path()
        assert_pose_close(p.interpolate(HALF_PI / 2), (math.sin(HALF_PI / 2),
                                                       1 - math.cos(HALF_PI / 2),
                                                       HALF_PI / 2))

    def test_interpolate_at_length_is_end_pose(self):
        p = self._path()
        assert_pose_close(p.interpolate(p.length), p.end_pose())

    def test_interpolate_clamps_out_of_range(self):
        p = self._path()
        assert_pose_close(p.interpolate(-5.0), p.start)
        assert_pose_close(p.interpolate(p.length + 5.0), p.end_pose())

    def test_sample_endpoints(self):
        p = self._path()
        pts = p.sample(0.1)
        assert_pose_close(pts[0], p.start)
        assert_pose_close(pts[-1], p.end_pose())

    def test_sample_spacing_never_exceeds_step(self):
        p = self._path()
        step = 0.1
        pts = p.sample(step)
        assert len(pts) > 1
        for a, b in zip(pts, pts[1:]):
            assert math.hypot(b[0] - a[0], b[1] - a[1]) <= step + 1e-9

    def test_sample_is_monotonic_along_path(self):
        p = self._path()
        pts = p.sample(0.25)
        # ardisik noktalar hep ileri gitmeli, geri donmemeli
        total = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        assert total <= p.length + 1e-9

    def test_sample_rejects_nonpositive_step(self):
        p = self._path()
        with pytest.raises(ValueError):
            p.sample(0.0)
        with pytest.raises(ValueError):
            p.sample(-1.0)

    def test_sample_of_zero_length_path(self):
        p = DubinsPath(start=(1.0, 2.0, 0.5), word="LSL", lengths=(0.0, 0.0, 0.0), rho=1.0)
        pts = p.sample(0.1)
        assert len(pts) == 1
        assert_pose_close(pts[0], p.start)


class TestCanonicalTransform:
    def test_aligned_along_x_axis(self):
        d, alpha, beta = _to_canonical((0.0, 0.0, 0.0), (4.0, 0.0, 0.0), 2.0)
        assert math.isclose(d, 2.0)
        assert math.isclose(alpha, 0.0, abs_tol=1e-12)
        assert math.isclose(beta, 0.0, abs_tol=1e-12)

    def test_headings_measured_from_connecting_line(self):
        # baslangictan hedefe dogru kuzeye gidiliyor; ikisi de dogu'ya bakiyor
        # -> baglanti dogrultusu pi/2, iki aci da -pi/2 yani 3pi/2
        d, alpha, beta = _to_canonical((0.0, 0.0, 0.0), (0.0, 3.0, 0.0), 1.0)
        assert math.isclose(d, 3.0)
        assert math.isclose(alpha, 3 * math.pi / 2, abs_tol=1e-9)
        assert math.isclose(beta, 3 * math.pi / 2, abs_tol=1e-9)

    def test_rotation_invariance(self):
        rot = math.radians(40)

        def rotate(p):
            x, y, yaw = p
            return (x * math.cos(rot) - y * math.sin(rot),
                    x * math.sin(rot) + y * math.cos(rot),
                    yaw + rot)

        base = _to_canonical((0.0, 0.0, 0.3), (5.0, 2.0, 1.1), 1.5)
        turned = _to_canonical(rotate((0.0, 0.0, 0.3)), rotate((5.0, 2.0, 1.1)), 1.5)
        for a, b in zip(base, turned):
            diff = _mod2pi(a - b)
            assert min(diff, 2 * math.pi - diff) < 1e-9

    def test_translation_invariance(self):
        base = _to_canonical((0.0, 0.0, 0.3), (5.0, 2.0, 1.1), 1.5)
        moved = _to_canonical((10.0, -7.0, 0.3), (15.0, -5.0, 1.1), 1.5)
        for a, b in zip(base, moved):
            assert math.isclose(a, b, abs_tol=1e-12)

    def test_d_scales_inversely_with_rho(self):
        d1, _, _ = _to_canonical((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 1.0)
        d2, _, _ = _to_canonical((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 2.0)
        assert math.isclose(d1, 10.0)
        assert math.isclose(d2, 5.0)

    def test_angles_are_normalized(self):
        _, alpha, beta = _to_canonical((0.0, 0.0, -3.0), (1.0, 1.0, 9.0), 1.0)
        assert 0.0 <= alpha < 2 * math.pi
        assert 0.0 <= beta < 2 * math.pi

    def test_coincident_positions_give_zero_d(self):
        d, _, _ = _to_canonical((2.0, 2.0, 0.0), (2.0, 2.0, 1.0), 1.0)
        assert d == 0.0


CSC_WORDS = ["LSL", "RSR", "LSR", "RSL"]

POSE_PAIRS = [
    ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (4.0, 3.0, 1.2)),
    ((0.0, 0.0, 2.0), (-6.0, 5.0, -1.0)),
    ((1.0, -2.0, 0.5), (1.5, -1.5, 3.0)),
    ((0.0, 0.0, 0.0), (0.5, 0.0, math.pi)),
]


def build_path(word, start, goal, rho):
    """Kanonik cozucuyu cagirip DubinsPath kurar; kelime gecersizse None."""
    d, alpha, beta = _to_canonical(start, goal, rho)
    result = _SOLVERS[word](d, alpha, beta)
    if result is None:
        return None
    t, p, q = result
    return DubinsPath(start=start, word=word,
                      lengths=(t * rho, p * rho, q * rho), rho=rho)


class TestCSCWords:
    @pytest.mark.parametrize("word", CSC_WORDS)
    @pytest.mark.parametrize("start,goal", POSE_PAIRS)
    def test_reaches_goal_when_valid(self, word, start, goal):
        path = build_path(word, start, goal, 2.0)
        if path is None:
            pytest.skip(f"{word} bu poz cifti icin gecersiz")
        assert_pose_close(path.end_pose(), goal, tol=1e-6)

    @pytest.mark.parametrize("word", CSC_WORDS)
    @pytest.mark.parametrize("start,goal", POSE_PAIRS)
    def test_segment_lengths_nonnegative(self, word, start, goal):
        path = build_path(word, start, goal, 2.0)
        if path is None:
            pytest.skip(f"{word} bu poz cifti icin gecersiz")
        assert all(seg >= -1e-12 for seg in path.lengths)

    @pytest.mark.parametrize("word", ["LSL", "RSR"])
    def test_straight_line_is_exactly_distance(self, word):
        path = build_path(word, (0.0, 0.0, 0.0), (7.0, 0.0, 0.0), 1.0)
        assert path is not None
        assert math.isclose(path.length, 7.0, abs_tol=1e-9)

    @pytest.mark.parametrize("word", CSC_WORDS)
    def test_middle_segment_is_the_straight_one(self, word):
        # CSC'de ikinci segment duz; normalize p degeri mesafeyle uyumlu olmali
        result = _SOLVERS[word](5.0, 0.0, 0.0)
        if result is None:
            pytest.skip(f"{word} gecersiz")
        _, p, _ = result
        assert p >= 0.0

    def test_four_csc_solvers_registered(self):
        assert {"LSL", "RSR", "LSR", "RSL"} <= set(_SOLVERS)


CCC_WORDS = ["RLR", "LRL"]

CCC_POSE_PAIRS = [
    ((0.0, 0.0, 0.0), (1.0, 0.0, math.pi)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 2.0)),
    ((0.0, 0.0, 0.0), (2.0, 1.0, 3.0)),
    ((0.0, 0.0, 1.0), (-1.0, 0.5, -2.0)),
]


class TestCCCWords:
    @pytest.mark.parametrize("word", CCC_WORDS)
    @pytest.mark.parametrize("start,goal", CCC_POSE_PAIRS)
    def test_reaches_goal_when_valid(self, word, start, goal):
        path = build_path(word, start, goal, 1.0)
        if path is None:
            pytest.skip(f"{word} bu poz cifti icin gecersiz")
        assert_pose_close(path.end_pose(), goal, tol=1e-6)

    @pytest.mark.parametrize("word", CCC_WORDS)
    def test_invalid_when_far_apart(self, word):
        # CCC yalnizca d < 4 icin gecerlidir
        assert _SOLVERS[word](10.0, 0.0, 0.0) is None

    @pytest.mark.parametrize("word", CCC_WORDS)
    def test_middle_arc_is_major(self, word):
        # CCC'de orta yayin donus acisi pi'den buyuk olmalidir
        result = _SOLVERS[word](1.0, 0.0, math.pi)
        if result is None:
            pytest.skip(f"{word} gecersiz")
        _, p, _ = result
        assert p > math.pi - 1e-9

    @pytest.mark.parametrize("word", CCC_WORDS)
    def test_segment_lengths_nonnegative(self, word):
        for start, goal in CCC_POSE_PAIRS:
            path = build_path(word, start, goal, 1.0)
            if path is None:
                continue
            assert all(seg >= -1e-12 for seg in path.lengths)

    def test_all_six_solvers_registered(self):
        assert set(_SOLVERS) == {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"}

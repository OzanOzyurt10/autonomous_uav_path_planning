"""src/terrain.py icin testler."""

import array
import random

import pytest

from src.terrain import Terrain, synthetic_terrain

# 3 sutun x 2 satir. Kare DEGIL ve spacing_x != spacing_y: satir/sutun ve
# eksen karisikligini yakalamak icin bilerek boyle. Satirlar guneyden
# kuzeye, yani ilk uclu y = 0 hatti.
#
#   y=20 |  400   500   600
#   y=0  |    0   100   200
#        +------------------
#          x=0   x=10  x=20
GRID = Terrain(array.array("h", [0, 100, 200, 400, 500, 600]),
               cols=3, rows=2, spacing_x=10.0, spacing_y=20.0)


class TestTerrainValidation:
    def test_wrong_height_count_raises(self):
        with pytest.raises(ValueError):
            Terrain(array.array("h", [0, 1, 2]), cols=2, rows=2,
                    spacing_x=1.0, spacing_y=1.0)

    @pytest.mark.parametrize("cols, rows", [(1, 3), (3, 1), (1, 1)])
    def test_too_small_grid_raises(self, cols, rows):
        with pytest.raises(ValueError):
            Terrain(array.array("h", [0] * (cols * rows)), cols=cols,
                    rows=rows, spacing_x=1.0, spacing_y=1.0)

    @pytest.mark.parametrize("spacing_x, spacing_y",
                             [(0.0, 1.0), (1.0, 0.0), (-1.0, 1.0)])
    def test_bad_spacing_raises(self, spacing_x, spacing_y):
        with pytest.raises(ValueError):
            Terrain(array.array("h", [0] * 4), cols=2, rows=2,
                    spacing_x=spacing_x, spacing_y=spacing_y)


class TestTerrainShape:
    def test_extent_uses_gaps_not_posts(self):
        """3 post arasinda 2 aralik var; extent post sayisi degil."""
        assert GRID.extent_x == 20.0
        assert GRID.extent_y == 20.0

    def test_elevation_range(self):
        assert GRID.elevation_range() == (0.0, 600.0)


class TestElevationAt:
    @pytest.mark.parametrize("x, y, expected", [
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 100.0),
        (20.0, 0.0, 200.0),
        (0.0, 20.0, 400.0),
        (10.0, 20.0, 500.0),
        (20.0, 20.0, 600.0),
    ])
    def test_grid_nodes_return_stored_values(self, x, y, expected):
        assert GRID.elevation_at(x, y) == pytest.approx(expected)

    def test_midpoint_along_x(self):
        assert GRID.elevation_at(5.0, 0.0) == pytest.approx(50.0)

    def test_midpoint_along_y(self):
        assert GRID.elevation_at(0.0, 10.0) == pytest.approx(200.0)

    def test_cell_centre_averages_four_corners(self):
        """(0 + 100 + 400 + 500) / 4 - cift dogrusal enterpolasyon."""
        assert GRID.elevation_at(5.0, 10.0) == pytest.approx(250.0)

    def test_quarter_point(self):
        """x'te 1/4, y'de 3/4: agirliklar dogru mu."""
        expected = (0.0 * 0.75 * 0.25 + 100.0 * 0.25 * 0.25
                    + 400.0 * 0.75 * 0.75 + 500.0 * 0.25 * 0.75)
        assert GRID.elevation_at(2.5, 15.0) == pytest.approx(expected)

    @pytest.mark.parametrize("x, y, expected", [
        (-50.0, 0.0, 0.0),
        (999.0, 0.0, 200.0),
        (0.0, -50.0, 0.0),
        (0.0, 999.0, 400.0),
        (999.0, 999.0, 600.0),
        (-999.0, -999.0, 0.0),
    ])
    def test_outside_the_map_clamps(self, x, y, expected):
        assert GRID.elevation_at(x, y) == pytest.approx(expected)

    def test_far_edge_does_not_overflow(self):
        """Tam kenarda sutun indeksi cols-1 cikar, komsusu tasar."""
        assert GRID.elevation_at(GRID.extent_x, 0.0) == pytest.approx(200.0)
        assert GRID.elevation_at(0.0, GRID.extent_y) == pytest.approx(400.0)

    def test_returns_float(self):
        assert isinstance(GRID.elevation_at(5.0, 5.0), float)


def _synthetic(seed=1, cols=40, rows=30, spacing=90.0,
               base=200.0, relief=800.0, peaks=5):
    return synthetic_terrain(cols, rows, spacing, random.Random(seed),
                             base=base, relief=relief, peaks=peaks)


class TestSyntheticTerrain:
    def test_dimensions_match_the_request(self):
        terrain = _synthetic()
        assert terrain.cols == 40
        assert terrain.rows == 30
        assert len(terrain.heights) == 40 * 30

    def test_spacing_is_used_on_both_axes(self):
        terrain = _synthetic()
        assert terrain.spacing_x == 90.0
        assert terrain.spacing_y == 90.0
        assert terrain.extent_x == pytest.approx(39 * 90.0)
        assert terrain.extent_y == pytest.approx(29 * 90.0)

    def test_normalised_to_base_and_relief(self):
        """En alcak nokta base, en yuksek nokta base + relief olmali."""
        low, high = _synthetic(base=200.0, relief=800.0).elevation_range()
        assert low == pytest.approx(200.0, abs=1.0)
        assert high == pytest.approx(1000.0, abs=1.0)

    def test_same_seed_gives_the_same_terrain(self):
        assert _synthetic(seed=3).heights == _synthetic(seed=3).heights

    def test_different_seed_gives_different_terrain(self):
        assert _synthetic(seed=3).heights != _synthetic(seed=4).heights

    def test_field_is_smooth_not_noise(self):
        """Komsu postlar arasindaki sicrama sinirli: hucre basina rastgele
        sayi degil, yumusak bir alan uretilmeli."""
        terrain = _synthetic()
        limit = 800.0 / 3
        for row in range(terrain.rows):
            for col in range(terrain.cols - 1):
                here = terrain.heights[row * terrain.cols + col]
                right = terrain.heights[row * terrain.cols + col + 1]
                assert abs(right - here) < limit
        for row in range(terrain.rows - 1):
            for col in range(terrain.cols):
                here = terrain.heights[row * terrain.cols + col]
                above = terrain.heights[(row + 1) * terrain.cols + col]
                assert abs(above - here) < limit

    def test_elevation_at_works_on_generated_terrain(self):
        terrain = _synthetic()
        low, high = terrain.elevation_range()
        for _ in range(200):
            x = random.uniform(0.0, terrain.extent_x)
            y = random.uniform(0.0, terrain.extent_y)
            assert low - 1e-9 <= terrain.elevation_at(x, y) <= high + 1e-9

    @pytest.mark.parametrize("kwargs", [
        {"cols": 1},
        {"rows": 1},
        {"spacing": 0.0},
        {"spacing": -10.0},
        {"relief": -1.0},
        {"peaks": 0},
    ])
    def test_bad_arguments_raise(self, kwargs):
        with pytest.raises(ValueError):
            _synthetic(**kwargs)

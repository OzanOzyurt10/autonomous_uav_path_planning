"""SRTM .hgt ve viewfinderpanoramas .hgl okuyucusu icin testler."""

import array
import math
import os

import pytest

from src.terrain import load_terrain

# Gercek karolar 1201 veya 3601; okuyucu kare olmasini yeterli buluyor,
# testte 61 kullanmak 2.9 MB'lik fikstur yazmaktan kurtariyor.
SIZE = 61
STEP_DEG = 1.0 / (SIZE - 1)
SPACING_Y = STEP_DEG * 111320.0

# Yukseklik = kuzeyden satir * 10 + sutun. Satir ve sutunu ayirt edilebilir
# kiliyor: yon karisirsa deger tutmaz.
def _height(row_from_north, col):
    return row_from_north * 10 + col


def _tile_bytes(big_endian, voids=()):
    grid = array.array("h", [0]) * (SIZE * SIZE)
    for row in range(SIZE):
        for col in range(SIZE):
            grid[row * SIZE + col] = _height(row, col)
    for row, col in voids:
        grid[row * SIZE + col] = -32768
    if big_endian != (array.array("h", [1]).tobytes()[0] == 0):
        grid.byteswap()
    return grid.tobytes()


@pytest.fixture(scope="session")
def hgt_file(tmp_path_factory):
    """Standart SRTM: basliksiz, big-endian."""
    path = tmp_path_factory.mktemp("terrain") / "N46E014.hgt"
    path.write_bytes(_tile_bytes(big_endian=True))
    return str(path)


@pytest.fixture(scope="session")
def hgl_file(tmp_path_factory):
    """viewfinderpanoramas: 512 baytlik LXNSRTM basligi, little-endian."""
    path = tmp_path_factory.mktemp("terrain") / "N46E014.hgl"
    header = b"LXNSRTM" + bytes(512 - 7)
    path.write_bytes(header + _tile_bytes(big_endian=False))
    return str(path)


class TestFormatDetection:
    def test_both_formats_give_the_same_grid(self, hgt_file, hgl_file):
        """Tek fark baslik ve bayt sirasi; icerik ayni cikmali."""
        from_hgt = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        from_hgl = load_terrain(hgl_file, 46.0, 14.0, 5000.0, 5000.0)
        assert from_hgt.heights == from_hgl.heights
        assert from_hgt.cols == from_hgl.cols
        assert from_hgt.rows == from_hgl.rows

    def test_unknown_name_raises(self, tmp_path):
        bad = tmp_path / "arazi.hgt"
        bad.write_bytes(_tile_bytes(big_endian=True))
        with pytest.raises(ValueError):
            load_terrain(str(bad), 46.0, 14.0, 5000.0, 5000.0)

    def test_non_square_file_raises(self, tmp_path):
        bad = tmp_path / "N46E014.hgt"
        bad.write_bytes(bytes(1234))
        with pytest.raises(ValueError):
            load_terrain(str(bad), 46.0, 14.0, 5000.0, 5000.0)


class TestWindowGeometry:
    def test_window_size_in_posts(self, hgt_file):
        terrain = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        assert terrain.rows == int(5000.0 / SPACING_Y) + 1
        assert terrain.cols == int(5000.0 / terrain.spacing_x) + 1

    def test_longitude_posts_are_closer_than_latitude_posts(self, hgt_file):
        """46 kuzeyde bir derece boylam, bir derece enlemden kisa."""
        terrain = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        assert terrain.spacing_y == pytest.approx(SPACING_Y, rel=1e-6)
        assert terrain.spacing_x < terrain.spacing_y
        assert terrain.spacing_x == pytest.approx(
            SPACING_Y * math.cos(math.radians(46.0 + 2500.0 / 111320.0)),
            rel=1e-6)

    def test_window_outside_the_tile_raises(self, hgt_file):
        with pytest.raises(ValueError):
            load_terrain(hgt_file, 46.99, 14.99, 20000.0, 20000.0)

    def test_window_below_the_tile_raises(self, hgt_file):
        with pytest.raises(ValueError):
            load_terrain(hgt_file, 45.5, 14.0, 5000.0, 5000.0)


class TestOrientation:
    """Dosya kuzeyden guneye sakliyor, Terrain guneyden kuzeye.

    Cevirmeyi unutmak haritayi dikey aynalar ve kod yine calisir - bu
    yuzden yon testleri tam degerlerle yapiliyor.
    """

    def test_south_west_corner_is_the_files_last_row(self, hgt_file):
        terrain = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        assert terrain.at(0, 0) == _height(SIZE - 1, 0)

    def test_row_index_grows_northwards(self, hgt_file):
        terrain = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        assert terrain.at(0, 1) == _height(SIZE - 2, 0)
        assert terrain.at(0, 2) == _height(SIZE - 3, 0)

    def test_column_index_grows_eastwards(self, hgt_file):
        terrain = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        assert terrain.at(1, 0) == _height(SIZE - 1, 1)
        assert terrain.at(2, 0) == _height(SIZE - 1, 2)

    def test_offset_window_starts_at_the_right_post(self, hgt_file):
        """Guneybatidan 10 post dogu, 5 post kuzey."""
        terrain = load_terrain(hgt_file, 46.0 + 5 * STEP_DEG,
                               14.0 + 10 * STEP_DEG, 4000.0, 4000.0)
        assert terrain.at(0, 0) == _height(SIZE - 1 - 5, 10)


class TestVoidFilling:
    def test_voids_are_replaced_by_neighbours(self, tmp_path):
        """Ham -32768 birakilirsa arazi 32 km asagi iner."""
        path = tmp_path / "N46E014.hgt"
        path.write_bytes(_tile_bytes(big_endian=True,
                                     voids=((59, 1), (59, 2), (58, 1))))
        terrain = load_terrain(str(path), 46.0, 14.0, 5000.0, 5000.0)
        low, _ = terrain.elevation_range()
        assert low > -1000.0
        # (59, 1) bizim izgarada satir 1, sutun 1; komsulari 590..601 araligi
        assert 570 <= terrain.at(1, 1) <= 620

    def test_grid_without_voids_is_untouched(self, hgt_file):
        terrain = load_terrain(hgt_file, 46.0, 14.0, 5000.0, 5000.0)
        assert terrain.at(0, 0) == _height(SIZE - 1, 0)


REAL_TILE = "data/N46E014.hgl"


@pytest.mark.skipif(not os.path.exists(REAL_TILE),
                    reason=f"{REAL_TILE} yok")
class TestRealTile:
    def test_alpine_window_has_plausible_relief(self):
        terrain = load_terrain(REAL_TILE, 46.24, 14.5, 10000.0, 10000.0)
        low, high = terrain.elevation_range()
        assert 0.0 < low < high < 4000.0
        assert high - low > 500.0        # Alpler, duz ova degil

    def test_window_is_about_ten_kilometres(self):
        terrain = load_terrain(REAL_TILE, 46.24, 14.5, 10000.0, 10000.0)
        assert 9500.0 < terrain.extent_x < 10100.0
        assert 9500.0 < terrain.extent_y < 10100.0

    def test_interpolation_stays_within_the_grid_range(self):
        terrain = load_terrain(REAL_TILE, 46.24, 14.5, 10000.0, 10000.0)
        low, high = terrain.elevation_range()
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = fraction * terrain.extent_x
            y = (1.0 - fraction) * terrain.extent_y
            assert low <= terrain.elevation_at(x, y) <= high

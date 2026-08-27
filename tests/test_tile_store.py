"""Cok karolu pencere kesme icin testler.

Sentetik karolarin her postu kendi KURESEL konumunu kodluyor
(h = boylam_indeksi * 100 + enlem_indeksi). Boylece dikis yerinde bir
post kaymasi ya da karolarin yer degistirmesi tam degerle yakalaniyor -
bu hatalar aksi halde sessiz kalir, kod calisir ve harita bozulur.
"""

import array
import math
import os
import zipfile

import pytest

from src.terrain import (MissingTiles, TileStore, load_terrain, tile_name)

BASE_LAT, BASE_LON = 46, 14
SIZE = 61
SPAN = SIZE - 1
STEP = 1.0 / SPAN
SPACING_Y = STEP * 111320.0


def expected(lat_index, lon_index):
    return lon_index * 100 + lat_index


def tile_bytes(tile_lat, tile_lon):
    """Big-endian .hgt karosu; satirlar kuzeyden guneye."""
    grid = array.array("h", [0]) * (SIZE * SIZE)
    for row in range(SIZE):                     # row kuzeyden sayiyor
        lat_index = (tile_lat - BASE_LAT) * SPAN + (SPAN - row)
        for col in range(SIZE):
            lon_index = (tile_lon - BASE_LON) * SPAN + col
            grid[row * SIZE + col] = expected(lat_index, lon_index)
    if array.array("h", [1]).tobytes()[0] == 1:      # makine little-endian
        grid.byteswap()
    return grid.tobytes()


@pytest.fixture(scope="module")
def library(tmp_path_factory):
    """2x2 karo: 46-48 N, 14-16 D. Biri arsivde, ucu klasorde."""
    root = tmp_path_factory.mktemp("tiles")
    loose = root / "data"
    loose.mkdir()
    for lat, lon in ((46, 14), (46, 15), (47, 14)):
        (loose / f"{tile_name(lat, lon)}.hgt").write_bytes(
            tile_bytes(lat, lon))
    archive = root / "extra.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr(f"alps/{tile_name(47, 15)}.hgt", tile_bytes(47, 15))
    return str(loose), str(archive)


@pytest.fixture(scope="module")
def store(library):
    return TileStore(library)


def check_window(terrain, lat, lon):
    """Her postun degeri kuresel konumuyla tutuyor mu."""
    for row in range(terrain.rows):
        lat_index = round((lat + row * STEP - BASE_LAT) * SPAN)
        for col in range(terrain.cols):
            lon_index = round((lon + col * STEP - BASE_LON) * SPAN)
            assert terrain.at(col, row) == expected(lat_index, lon_index), \
                f"({col}, {row}) postu yanlis"


class TestCoverage:
    def test_finds_loose_and_archived_tiles(self, store):
        assert store.coverage() == [(46, 14), (46, 15), (47, 14), (47, 15)]

    def test_ignores_files_that_are_not_tiles(self, tmp_path):
        (tmp_path / "notlar.txt").write_text("selam")
        (tmp_path / "arazi.hgt").write_bytes(tile_bytes(46, 14))
        assert TileStore([str(tmp_path)]).coverage() == []

    def test_missing_source_is_skipped(self):
        assert TileStore(["boyle-bir-yer-yok"]).coverage() == []


class TestSingleTileWindow:
    def test_matches_the_single_tile_loader(self, store, library):
        """Tek karoya sigan pencere load_terrain ile birebir ayni olmali."""
        loose, _ = library
        path = os.path.join(loose, f"{tile_name(46, 14)}.hgt")
        direct = load_terrain(path, 46.2, 14.2, 4000.0, 4000.0)
        stitched = store.window(46.2, 14.2, 4000.0, 4000.0)
        assert stitched.cols == direct.cols
        assert stitched.rows == direct.rows
        assert stitched.heights == direct.heights
        assert stitched.spacing_x == pytest.approx(direct.spacing_x)

    def test_values_encode_global_position(self, store):
        terrain = store.window(46.2, 14.2, 4000.0, 4000.0)
        check_window(terrain, 46.2, 14.2)


class TestSeam:
    def test_window_across_the_longitude_seam(self, store):
        """Boylamda iki karoya yayilan pencere; dikis 15 derecede."""
        terrain = store.window(46.4, 14.9, 20000.0, 3000.0)
        assert 14.9 + (terrain.cols - 1) * STEP > 15.0, "pencere dikisi gecmiyor"
        check_window(terrain, 46.4, 14.9)

    def test_window_across_the_latitude_seam(self, store):
        terrain = store.window(46.9, 14.4, 3000.0, 20000.0)
        assert 46.9 + (terrain.rows - 1) * STEP > 47.0, "pencere dikisi gecmiyor"
        check_window(terrain, 46.9, 14.4)

    def test_window_across_all_four_tiles(self, store):
        terrain = store.window(46.92, 14.92, 20000.0, 20000.0)
        assert 14.92 + (terrain.cols - 1) * STEP > 15.0
        assert 46.92 + (terrain.rows - 1) * STEP > 47.0
        check_window(terrain, 46.92, 14.92)

    def test_shared_edge_post_is_not_duplicated(self, store):
        """Karolar kenarda bir post paylasiyor; pencere onu tek kez almali."""
        terrain = store.window(46.4, 14.9, 20000.0, 2000.0)
        seam = round((15.0 - 14.9) * SPAN)        # dikise dusen sutun
        left = terrain.at(seam - 1, 0)
        middle = terrain.at(seam, 0)
        right = terrain.at(seam + 1, 0)
        assert middle - left == 100               # bir boylam adimi
        assert right - middle == 100


class TestBoundaries:
    def test_top_edge_on_a_degree_line_needs_no_tile_above(self, tmp_path):
        """Ust kenar tam 47.0'a otururken 47N karosu istenmemeli."""
        (tmp_path / f"{tile_name(46, 14)}.hgt").write_bytes(
            tile_bytes(46, 14))
        lonely = TileStore([str(tmp_path)])
        rows = 20
        lat = 47.0 - (rows - 1) * STEP
        # Yukseklik tam kat verilirse int() kayan nokta yuzunden bir satir
        # eksik sayabiliyor; yarim adim fazlasi sayiyi kesinlestiriyor.
        terrain = lonely.window(lat, 14.2, 2000.0, (rows - 0.5) * SPACING_Y)
        assert terrain.rows == rows
        assert lat + (rows - 1) * STEP == pytest.approx(47.0)
        check_window(terrain, lat, 14.2)

    def test_missing_tile_names_the_gap(self, store):
        with pytest.raises(MissingTiles) as caught:
            store.window(46.4, 15.9, 20000.0, 3000.0)
        assert caught.value.names == ["N46E016"]

    def test_window_outside_coverage_raises(self, store):
        with pytest.raises(MissingTiles):
            store.window(10.0, 30.0, 4000.0, 4000.0)

    def test_mixed_resolution_raises(self, tmp_path):
        (tmp_path / f"{tile_name(46, 14)}.hgt").write_bytes(tile_bytes(46, 14))
        coarse = array.array("h", [7] * (31 * 31))
        if array.array("h", [1]).tobytes()[0] == 1:
            coarse.byteswap()
        (tmp_path / f"{tile_name(46, 15)}.hgt").write_bytes(coarse.tobytes())
        mixed = TileStore([str(tmp_path)])
        with pytest.raises(ValueError):
            mixed.window(46.4, 14.9, 20000.0, 3000.0)


class TestSpacing:
    def test_longitude_spacing_shrinks_with_latitude(self, store):
        terrain = store.window(46.2, 14.2, 4000.0, 4000.0)
        assert terrain.spacing_y == pytest.approx(SPACING_Y, rel=1e-6)
        assert terrain.spacing_x < terrain.spacing_y
        assert terrain.spacing_x == pytest.approx(
            SPACING_Y * math.cos(math.radians(46.2 + 2000.0 / 111320.0)),
            rel=1e-6)


class TestCache:
    def test_repeated_windows_stay_consistent(self, store):
        """Onbellek sinirini asan erisimlerden sonra sonuc degismemeli."""
        first = store.window(46.2, 14.2, 4000.0, 4000.0)
        for lat, lon in ((46.5, 14.5), (47.2, 14.2), (47.2, 15.2),
                         (46.2, 15.2), (46.9, 14.9)):
            store.window(lat, lon, 2000.0, 2000.0)
        again = store.window(46.2, 14.2, 4000.0, 4000.0)
        assert again.heights == first.heights

"""src/terrarium.py icin testler.

Ag hic kullanilmiyor: TerrariumSource'a sahte bir fetch veriliyor ve
karolar testin kendi urettigi PNG'ler. Boylece kot cozumu, mozaik dizilimi
ve yeniden ornekleme internetten bagimsiz sinaniyor.
"""

import io
import math
import re

import pytest

from src.terrarium import (MAX_TILES, TILE_SIZE, ElevationUnavailable,
                           TerrariumSource, decode_tile, pixel_of,
                           resolution, zoom_for)

PIL = pytest.importorskip("PIL.Image", reason="Pillow yok")

URL_PARTS = re.compile(r"/(\d+)/(\d+)/(\d+)\.png$")


def encode_png(heights):
    """Kot listesini (256*256, kuzeyden guneye) Terrarium PNG'sine cevirir."""
    pixels = bytearray()
    for metres in heights:
        value = int(round((metres + 32768) * 256))
        value = max(0, min(0xFFFFFF, value))
        red, rest = divmod(value, 65536)
        green, blue = divmod(rest, 256)
        pixels += bytes([red, green, blue])
    image = PIL.frombytes("RGB", (TILE_SIZE, TILE_SIZE), bytes(pixels))
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def tile_maker(height_fn):
    """URL'den z/x/y okuyup istenen karoyu ureten sahte bir fetch.

    height_fn KURESEL piksel konumunu aliyor; boylece komsu karolar
    kendiliginden tutarli oluyor ve dikis hatasi gorunur hale geliyor.
    """
    calls = []

    def fetch(url):
        match = URL_PARTS.search(url)
        if match is None:
            raise ValueError(f"adres cozulemedi: {url}")
        zoom, x, y = (int(part) for part in match.groups())
        calls.append((zoom, x, y))
        heights = [height_fn(x * TILE_SIZE + col, y * TILE_SIZE + row)
                   for row in range(TILE_SIZE) for col in range(TILE_SIZE)]
        return encode_png(heights)

    fetch.calls = calls
    return fetch


# Mercator piksel konumunun DOGRUSAL fonksiyonu. Cift dogrusal enterpolasyon
# dogrusal alanlari tam urettigi icin beklenen deger kesin biliniyor -
# mozaik dizilimi ya da eksen karisikligi aninda ortaya cikar.
def linear_field(px, py):
    return 500.0 + 0.05 * px - 0.03 * py


class TestTileGeometry:
    def test_resolution_halves_each_zoom(self):
        assert resolution(11, 0.0) == pytest.approx(resolution(10, 0.0) / 2)

    def test_resolution_shrinks_towards_the_pole(self):
        assert resolution(11, 60.0) == pytest.approx(resolution(11, 0.0) / 2,
                                                     rel=1e-3)

    def test_zoom_meets_the_requested_spacing(self):
        for spacing in (30.0, 60.0, 90.0, 200.0):
            for lat in (0.0, 46.0, 70.0):
                zoom = zoom_for(spacing, lat)
                # Karo cozunurlugu istenen araliktan ince olmali
                assert resolution(zoom, lat) <= spacing + 1e-9

    def test_zoom_is_the_smallest_that_works(self):
        zoom = zoom_for(60.0, 46.0)
        assert resolution(zoom - 1, 46.0) > 60.0

    def test_zoom_is_clamped(self):
        assert zoom_for(1e9, 0.0) == 6
        assert zoom_for(1e-9, 0.0) == 14

    @pytest.mark.parametrize("spacing", [0.0, -5.0])
    def test_bad_spacing_raises(self, spacing):
        with pytest.raises(ValueError):
            zoom_for(spacing, 0.0)

    def test_pixel_origin_is_the_north_west_corner(self):
        assert pixel_of(85.05112878, -180.0, 0)[0] == pytest.approx(0.0)
        assert pixel_of(85.05112878, -180.0, 0)[1] == pytest.approx(0.0,
                                                                    abs=1e-6)

    def test_equator_and_prime_meridian_are_the_centre(self):
        scale = TILE_SIZE
        assert pixel_of(0.0, 0.0, 0) == pytest.approx((scale / 2, scale / 2))

    def test_north_is_a_smaller_pixel_row(self):
        assert pixel_of(50.0, 14.0, 10)[1] < pixel_of(40.0, 14.0, 10)[1]

    def test_east_is_a_larger_pixel_column(self):
        assert pixel_of(46.0, 15.0, 10)[0] > pixel_of(46.0, 14.0, 10)[0]


class TestDecodeTile:
    def test_known_elevations(self):
        heights = [float(i % 3000) for i in range(TILE_SIZE * TILE_SIZE)]
        out = decode_tile(encode_png(heights))
        assert len(out) == TILE_SIZE * TILE_SIZE
        for expected, got in zip(heights[:500], out[:500]):
            assert got == pytest.approx(expected, abs=0.01)

    def test_negative_elevation_survives(self):
        # Terrarium batimetri de tasiyor; isaret hatasi denizi daga cevirir.
        heights = [-400.0] * (TILE_SIZE * TILE_SIZE)
        assert decode_tile(encode_png(heights))[0] == pytest.approx(-400.0,
                                                                    abs=0.01)

    def test_wrong_size_rejected(self):
        image = PIL.new("RGB", (64, 64))
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        with pytest.raises(ValueError, match="256"):
            decode_tile(buffer.getvalue())


class TestFetchingAndCache:
    def test_downloads_then_serves_from_cache(self, tmp_path):
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        source.window(46.0, 14.0, 3000.0, 3000.0, spacing_m=90.0)
        first = source.downloads
        assert first > 0

        # Ayni onbellek, yeni kaynak: tek karo bile inmemeli.
        again = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        again.window(46.0, 14.0, 3000.0, 3000.0, spacing_m=90.0)
        assert again.downloads == 0

    def test_failure_is_reported_not_swallowed(self, tmp_path):
        def broken(url):
            raise OSError("ag yok")

        source = TerrariumSource(cache_dir=str(tmp_path), fetch=broken)
        with pytest.raises(ElevationUnavailable, match="yukseklik karosu"):
            source.window(46.0, 14.0, 3000.0, 3000.0, spacing_m=90.0)

    def test_corrupt_tile_is_not_cached(self, tmp_path):
        def rubbish(url):
            return b"bu PNG degil"

        source = TerrariumSource(cache_dir=str(tmp_path), fetch=rubbish)
        with pytest.raises(ElevationUnavailable):
            source.tile(11, 1000, 700)
        # Bozuk veri diske yazilsaydi bir daha hic duzelmezdi.
        assert not list(tmp_path.rglob("*.png"))

    def test_cache_path_separates_zoom_levels(self, tmp_path):
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        source.tile(10, 5, 6)
        source.tile(11, 5, 6)
        assert (tmp_path / "terrarium" / "10" / "5" / "6.png").is_file()
        assert (tmp_path / "terrarium" / "11" / "5" / "6.png").is_file()


class TestWindow:
    SPACING = 90.0

    def _terrain(self, tmp_path, lat=46.0, lon=14.0, width=4000.0,
                 height=4000.0):
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        return source.window(lat, lon, width, height, spacing_m=self.SPACING)

    def test_grid_matches_the_local_tile_convention(self, tmp_path):
        terrain = self._terrain(tmp_path)
        # Post araligi derecede esit, metrede degil: dogu-bati daha dar.
        assert terrain.spacing_y == pytest.approx(self.SPACING)
        assert terrain.spacing_x < terrain.spacing_y
        assert terrain.spacing_x == pytest.approx(
            self.SPACING * math.cos(math.radians(46.0 + 2000.0 / 111320)),
            rel=1e-6)

    def test_heights_reproduce_the_source_field(self, tmp_path):
        # Alan dogrusal, enterpolasyon cift dogrusal: sonuc TAM olmali.
        # Sapma cikarsa mozaik dizilimi ya da eksen yonu bozuktur.
        terrain = self._terrain(tmp_path)
        zoom = zoom_for(self.SPACING, 46.0 + 2000.0 / 111320)
        step = self.SPACING / 111320
        worst = 0.0
        for row in range(0, terrain.rows, 7):
            for col in range(0, terrain.cols, 7):
                px, py = pixel_of(46.0 + row * step, 14.0 + col * step, zoom)
                worst = max(worst,
                            abs(terrain.at(col, row) - linear_field(px, py)))
        assert worst < 1.0

    def test_row_zero_is_south(self, tmp_path):
        # linear_field'de py katsayisi negatif; kuzeye gidince py kuculuyor,
        # yani kot ARTIYOR. Satir sirasi ters olsa isaret donerdi.
        terrain = self._terrain(tmp_path)
        assert terrain.at(0, terrain.rows - 1) > terrain.at(0, 0)

    def test_column_zero_is_west(self, tmp_path):
        terrain = self._terrain(tmp_path)
        assert terrain.at(terrain.cols - 1, 0) > terrain.at(0, 0)

    def test_spans_more_than_one_tile(self, tmp_path):
        # Tek karoya sigan pencere dikis hatasini gizlerdi; bu testin
        # anlamli olmasi icin birden fazla karo inmis olmali.
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        source.window(46.0, 14.0, 12000.0, 12000.0, spacing_m=self.SPACING)
        assert source.downloads > 1

    def test_southern_hemisphere(self, tmp_path):
        terrain = self._terrain(tmp_path, lat=-33.9, lon=18.4)
        assert terrain.cols > 1 and terrain.rows > 1
        assert terrain.at(0, terrain.rows - 1) > terrain.at(0, 0)

    def test_too_many_tiles_raises(self, tmp_path):
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        with pytest.raises(ValueError, match="karo gerektiriyor"):
            source.window(46.0, 14.0, 60000.0, 60000.0, spacing_m=5.0)
        assert source.downloads == 0        # sinir asilinca hic inmemeli

    @pytest.mark.parametrize("width, height", [(0.0, 1000.0), (-5.0, 1000.0)])
    def test_bad_window_raises(self, tmp_path, width, height):
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        with pytest.raises(ValueError):
            source.window(46.0, 14.0, width, height)

    def test_tiny_window_raises(self, tmp_path):
        fetch = tile_maker(linear_field)
        source = TerrariumSource(cache_dir=str(tmp_path), fetch=fetch)
        with pytest.raises(ValueError, match="cok kucuk"):
            source.window(46.0, 14.0, 10.0, 10.0, spacing_m=90.0)


class TestLimits:
    def test_max_tiles_is_a_real_bound(self):
        assert 0 < MAX_TILES <= 256

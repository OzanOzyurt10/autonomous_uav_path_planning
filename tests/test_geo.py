"""src/geo.py icin testler."""

import math

import pytest

import src.terrain
from src.geo import METRES_PER_DEGREE, Frame

# Alpler'den gercek bir pencere: 46.15 N, 14.35 D, 20 km yuksekliginde.
# Guney yarikure ve bati boylam ayri ayri test ediliyor; isaret hatalari
# yalnizca kuzeydogu ornekleriyle gizli kalir.
ALPS = Frame.for_window(46.15, 14.35, 20000.0)


class TestValidation:
    @pytest.mark.parametrize("lat", [-95.0, 91.0, 90.5])
    def test_impossible_latitude_raises(self, lat):
        with pytest.raises(ValueError):
            Frame(lat, 0.0, 0.0)

    @pytest.mark.parametrize("lon", [-181.0, 200.0])
    def test_impossible_longitude_raises(self, lon):
        with pytest.raises(ValueError):
            Frame(0.0, lon, 0.0)

    @pytest.mark.parametrize("mid_lat", [85.0, -85.0, 88.0])
    def test_polar_frame_raises(self, mid_lat):
        # Kutupta bir boylam derecesi sifira gidiyor; to_geo orada bolme
        # hatasi verir. Sessiz sacma sonuc yerine acik hata.
        with pytest.raises(ValueError):
            Frame(mid_lat, 0.0, mid_lat)

    def test_high_but_legal_latitude_is_accepted(self):
        Frame(84.0, 0.0, 84.0)


class TestForWindow:
    def test_corner_is_kept(self):
        assert ALPS.lat == pytest.approx(46.15)
        assert ALPS.lon == pytest.approx(14.35)

    def test_mid_lat_is_half_the_window_north(self):
        # terrain.py ile ayni sozlesme: olcek pencerenin ortasinda
        # donduruluyor, guney kenarinda degil.
        assert ALPS.mid_lat == pytest.approx(
            46.15 + 10000.0 / METRES_PER_DEGREE)

    def test_zero_height_window_keeps_south_edge(self):
        assert Frame.for_window(10.0, 20.0, 0.0).mid_lat == pytest.approx(10.0)


class TestScale:
    def test_longitude_degree_at_equator(self):
        assert Frame(0.0, 0.0, 0.0).lon_scale == pytest.approx(
            METRES_PER_DEGREE)

    def test_longitude_degree_shrinks_with_latitude(self):
        # 60 derecede cos tam 1/2; carpanin radyan/derece karisikligi
        # burada aninda yakalanir.
        assert Frame(60.0, 0.0, 60.0).lon_scale == pytest.approx(
            METRES_PER_DEGREE / 2)

    def test_scale_is_symmetric_across_equator(self):
        assert (Frame(-40.0, 0.0, -40.0).lon_scale
                == pytest.approx(Frame(40.0, 0.0, 40.0).lon_scale))

    def test_shares_terrain_constant(self):
        # Iki modul ayni sayiyi kullanmali; biri degisip digeri kalirsa
        # arazi izgarasi ile waypoint koordinatlari kayar.
        assert METRES_PER_DEGREE == src.terrain.METRES_PER_DEGREE


class TestToLocal:
    def test_corner_is_the_origin(self):
        assert ALPS.to_local(46.15, 14.35) == pytest.approx((0.0, 0.0))

    def test_one_degree_north(self):
        x, y = ALPS.to_local(47.15, 14.35)
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(METRES_PER_DEGREE)

    def test_one_degree_east(self):
        x, y = ALPS.to_local(46.15, 15.35)
        assert x == pytest.approx(METRES_PER_DEGREE
                                  * math.cos(math.radians(ALPS.mid_lat)))
        assert y == pytest.approx(0.0)

    def test_south_and_west_are_negative(self):
        x, y = ALPS.to_local(45.15, 13.35)
        assert x < 0.0 and y < 0.0

    def test_southern_hemisphere(self):
        frame = Frame.for_window(-33.9, 18.4, 20000.0)
        x, y = frame.to_local(-32.9, 19.4)
        assert y == pytest.approx(METRES_PER_DEGREE)
        assert x > 0.0

    def test_western_longitude(self):
        frame = Frame.for_window(37.6, -122.4, 20000.0)
        x, _ = frame.to_local(37.6, -121.4)
        assert x > 0.0


class TestRoundTrip:
    @pytest.mark.parametrize("lat, lon", [(46.15, 14.35), (46.4, 14.6),
                                          (45.9, 14.1), (46.15, 14.35)])
    def test_geo_to_local_and_back(self, lat, lon):
        x, y = ALPS.to_local(lat, lon)
        back_lat, back_lon = ALPS.to_geo(x, y)
        assert back_lat == pytest.approx(lat)
        assert back_lon == pytest.approx(lon)

    @pytest.mark.parametrize("x, y", [(0.0, 0.0), (20000.0, 20000.0),
                                      (-5000.0, 12345.0)])
    def test_local_to_geo_and_back(self, x, y):
        lat, lon = ALPS.to_geo(x, y)
        back_x, back_y = ALPS.to_local(lat, lon)
        assert back_x == pytest.approx(x)
        assert back_y == pytest.approx(y)

    def test_to_geo_uses_the_frame_scale_not_the_point(self):
        # to_geo, to_local'in tam tersi olmali. Olcegi noktanin kendi
        # enleminde yeniden hesaplarsan gidip gelmek ayni yere donmez.
        x, y = 30000.0, 40000.0
        lat, lon = ALPS.to_geo(x, y)
        assert lon == pytest.approx(ALPS.lon + x / ALPS.lon_scale)
        assert lat == pytest.approx(ALPS.lat + y / METRES_PER_DEGREE)


class TestWindowSize:
    def test_selected_rectangle_becomes_metres(self):
        # Tarayici dunya haritasinda enlem/boylam dikdortgeni seciyor;
        # sunucunun ihtiyaci olan sey onun metre karsiligi.
        frame = Frame.for_window(46.0, 14.0, 0.0)
        width, height = frame.to_local(46.25, 14.25)
        assert height == pytest.approx(0.25 * METRES_PER_DEGREE)
        assert width == pytest.approx(0.25 * METRES_PER_DEGREE
                                      * math.cos(math.radians(46.0)))
        assert width < height

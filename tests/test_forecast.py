"""Open-Meteo'dan ruzgar profili: saat secimi, eksen birligi, hata yolu.

Testler aga hic cikmiyor - fetch disaridan veriliyor ve sahte bir yuk
donuyor. Tutulan sozlesme uc sey: seviyeler MSL eksenine tasiniyor, yerin
altinda kalan basinc seviyeleri eleniyor, ve her ag hatasi
ForecastUnavailable oluyor ki cagiran elle girmeye donebilsin.
"""

import json
from datetime import datetime

import pytest

from src.forecast import (ForecastUnavailable, build_url, pick_hour,
                          wind_profile)
from src.wind import ProfileWind

NOON = datetime(2026, 9, 10, 1, 10)


def _payload(elevation=100.0):
    """Uc saatlik sahte tahmin; iki yuzey degeri, uc basinc seviyesi."""
    return {
        "elevation": elevation,
        "hourly": {
            "time": ["2026-09-10T00:00", "2026-09-10T01:00",
                     "2026-09-10T02:00"],
            "wind_speed_10m": [3.0, 4.0, 5.0],
            "wind_direction_10m": [270.0, 270.0, 270.0],
            "wind_speed_100m": [6.0, 7.0, 8.0],
            "wind_direction_100m": [275.0, 275.0, 275.0],
            # 50 m MSL: zemin 100 m'de oldugu icin YERIN ALTINDA.
            "wind_speed_1000hPa": [9.0, 9.0, 9.0],
            "wind_direction_1000hPa": [280.0, 280.0, 280.0],
            "geopotential_height_1000hPa": [50.0, 50.0, 50.0],
            "wind_speed_975hPa": [10.0, 11.0, 12.0],
            "wind_direction_975hPa": [285.0, 285.0, 285.0],
            "geopotential_height_975hPa": [400.0, 400.0, 400.0],
            # Bos deger: API zaman zaman null donuyor.
            "wind_speed_950hPa": [None, None, None],
            "wind_direction_950hPa": [None, None, None],
            "geopotential_height_950hPa": [600.0, 600.0, 600.0],
        },
    }


def _fetch(payload):
    return lambda url: json.dumps(payload).encode()


def _profile(payload=None, now=NOON):
    return wind_profile(41.2, 29.3, fetch=_fetch(payload or _payload()),
                        now=now)


class TestBuildUrl:
    def test_carries_the_coordinate(self):
        url = build_url(41.2, 29.3)
        assert "latitude=41.2" in url
        assert "longitude=29.3" in url

    def test_asks_for_metres_per_second(self):
        # Varsayilan km/h; birim istemezsek hizlar 3.6 kat buyuk gelir ve
        # hicbir sey hata vermez.
        assert "wind_speed_unit=ms" in build_url(0.0, 0.0)

    def test_asks_for_level_heights(self):
        # Basinc seviyesinin METRE karsiligi ayrica isteniyor; onsuz
        # seviyeleri bir eksene dizmek mumkun degil.
        assert "geopotential_height_1000hPa" in build_url(0.0, 0.0)


class TestPickHour:
    """Tahmin saatlik; gorev simdi planlaniyor, en yakin saat seciliyor."""

    TIMES = ["2026-09-10T00:00", "2026-09-10T01:00", "2026-09-10T02:00"]

    def test_picks_the_closest_hour(self):
        assert pick_hour(self.TIMES, datetime(2026, 9, 10, 1, 10)) == 1

    def test_rounds_to_the_later_hour_when_nearer(self):
        assert pick_hour(self.TIMES, datetime(2026, 9, 10, 1, 50)) == 2

    def test_before_the_range_takes_the_first(self):
        assert pick_hour(self.TIMES, datetime(2026, 9, 9, 20, 0)) == 0

    def test_after_the_range_takes_the_last(self):
        assert pick_hour(self.TIMES, datetime(2026, 9, 11, 8, 0)) == 2

    def test_empty_is_an_error(self):
        with pytest.raises(ForecastUnavailable):
            pick_hour([], NOON)


class TestLevels:
    """Seviyeler tek eksene tasiniyor: MSL."""

    def test_surface_levels_are_lifted_to_sea_level(self):
        # 10 m ve 100 m YERDEN, basinc seviyeleri DENIZDEN. Ikisini ayni
        # eksene tasimadan siralamak profili bozar.
        levels = _profile(_payload(elevation=100.0))["levels"]
        heights = sorted(level[0] for level in levels)
        assert 110.0 in heights
        assert 200.0 in heights

    def test_underground_levels_are_dropped(self):
        # Alp'te zemin 506 m iken 1000 hPa 155 m'de cikiyor: 350 m yerin
        # altinda. Birakilirsa profilin dibine anlamsiz bir deger oturur.
        levels = _profile(_payload(elevation=100.0))["levels"]
        assert all(level[0] > 100.0 for level in levels)
        assert all(level[1] != 9.0 for level in levels)   # 1000 hPa elendi

    def test_null_levels_are_skipped(self):
        levels = _profile()["levels"]
        assert all(level[0] != 600.0 for level in levels)

    def test_values_come_from_the_chosen_hour(self):
        levels = _profile()["levels"]
        surface = min(levels, key=lambda level: level[0])
        assert surface[1] == pytest.approx(4.0)          # 01:00 degeri

    def test_surface_reading_is_reported_for_the_inputs(self):
        got = _profile()
        assert got["speed"] == pytest.approx(4.0)
        assert got["from_deg"] == pytest.approx(270.0)

    def test_result_builds_a_field(self):
        field = ProfileWind(_profile()["levels"])
        assert field.strongest() > 0.0


class TestFailures:
    """Her hata ayni kapiya cikiyor: cagiran elle girmeye donebilmeli."""

    def test_network_error_is_wrapped(self):
        def boom(url):
            raise OSError("ag yok")

        with pytest.raises(ForecastUnavailable):
            wind_profile(41.2, 29.3, fetch=boom, now=NOON)

    def test_bad_json_is_wrapped(self):
        with pytest.raises(ForecastUnavailable):
            wind_profile(41.2, 29.3, fetch=lambda url: b"<html>", now=NOON)

    def test_missing_keys_are_wrapped(self):
        with pytest.raises(ForecastUnavailable):
            wind_profile(41.2, 29.3, fetch=_fetch({"elevation": 0.0}),
                         now=NOON)

    def test_no_usable_level_is_an_error(self):
        # Basinc seviyeleri yerin altinda VE yuzey degerleri bos: profil
        # kurulamaz. Sessizce bos donmek yerine sebebi soyluyoruz.
        payload = _payload(elevation=9000.0)
        for name in ("wind_speed_10m", "wind_direction_10m",
                     "wind_speed_100m", "wind_direction_100m"):
            payload["hourly"][name] = [None, None, None]
        with pytest.raises(ForecastUnavailable):
            _profile(payload)

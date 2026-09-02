"""Gorev dosyasi: seyreltme ve QGC WPL 110 bicimi.

Bicim hatalari SESSIZ: sutun kayarsa dosya yine yuklenir, yalnizca
irtifa yanlis olur. O yuzden testler satirlari ayristirip alan alan
kontrol ediyor, "iceriyor mu" diye bakmiyor.
"""

import math

import pytest

from src.wpl import (FRAME_GLOBAL, NAV_TAKEOFF, NAV_WAYPOINT, VERSION_LINE,
                     deviation, mission_text, simplify)


def _straight(count, step=100.0):
    return [(index * step, 0.0, 1000.0) for index in range(count)]


class TestSimplify:
    def test_straight_line_collapses_to_two_points(self):
        assert simplify(_straight(50), 1.0) == [0, 49]

    def test_endpoints_are_always_kept(self):
        points = [(0.0, 0.0, 0.0), (100.0, 400.0, 0.0), (200.0, 0.0, 0.0)]
        kept = simplify(points, 1.0)
        assert kept[0] == 0 and kept[-1] == len(points) - 1

    def test_corner_survives(self):
        # L donusu: kose noktasi atilirsa cizgi rotayi keser.
        points = ([(float(x), 0.0, 0.0) for x in range(0, 1000, 100)] +
                  [(900.0, float(y), 0.0) for y in range(100, 1000, 100)])
        kept = simplify(points, 10.0)
        assert 9 in kept                       # (900, 0) kose noktasi

    def test_indices_are_sorted_and_unique(self):
        points = [(float(x), math.sin(x / 200.0) * 300.0, 0.0)
                  for x in range(0, 4000, 25)]
        kept = simplify(points, 20.0)
        assert kept == sorted(set(kept))

    def test_tolerance_bound_is_respected(self):
        # simplify'in verdigi tek soz bu: birakilan cizgi atilan hicbir
        # noktadan tolerance'tan fazla sapmiyor.
        points = [(float(x), math.sin(x / 300.0) * 400.0,
                   1000.0 + math.cos(x / 500.0) * 200.0)
                  for x in range(0, 6000, 20)]
        for tolerance in (5.0, 25.0, 100.0):
            kept = simplify(points, tolerance)
            assert deviation(points, kept) <= tolerance + 1e-9

    def test_looser_tolerance_keeps_fewer_points(self):
        points = [(float(x), math.sin(x / 300.0) * 400.0, 1000.0)
                  for x in range(0, 6000, 20)]
        assert len(simplify(points, 100.0)) < len(simplify(points, 5.0))

    def test_short_inputs(self):
        assert simplify([], 1.0) == []
        assert simplify([(0.0, 0.0, 0.0)], 1.0) == [0]
        assert simplify([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], 1.0) == [0, 1]

    def test_negative_tolerance_is_refused(self):
        with pytest.raises(ValueError):
            simplify(_straight(5), -1.0)

    def test_altitude_change_is_not_ignored(self):
        # Yatayda duz ama irtifada kirilan rota: seyreltme kirilmayi
        # gormezse ucak tepeye girer. Uc boyutlu uzaklik sart.
        points = [(float(x), 0.0, 1000.0) for x in range(0, 600, 100)]
        points += [(float(x), 0.0, 1000.0 + (x - 500))
                   for x in range(600, 1200, 100)]
        assert len(simplify(points, 5.0)) > 2


def _rows(text):
    """Basliktan sonraki satirlari alan listesi olarak verir."""
    lines = text.strip("\n").split("\n")
    return [line.split("\t") for line in lines[1:]]


WAYPOINTS = [(46.10, 14.30, 900.0), (46.15, 14.35, 950.0),
             (46.20, 14.40, 880.0)]


class TestMissionText:
    def test_header_is_exact(self):
        assert mission_text(WAYPOINTS).split("\n")[0] == VERSION_LINE

    def test_every_row_has_twelve_tab_separated_fields(self):
        for row in _rows(mission_text(WAYPOINTS)):
            assert len(row) == 12

    def test_home_row_is_first_and_current(self):
        home = _rows(mission_text(WAYPOINTS))[0]
        assert home[0] == "0"                      # indeks
        assert home[1] == "1"                      # CURRENT WP
        assert int(home[2]) == FRAME_GLOBAL
        assert int(home[3]) == NAV_WAYPOINT

    def test_home_defaults_to_first_waypoint(self):
        home = _rows(mission_text(WAYPOINTS))[0]
        assert float(home[8]) == pytest.approx(46.10)
        assert float(home[9]) == pytest.approx(14.30)

    def test_explicit_home_is_used(self):
        # Dogrusu zemin kotunu gecirmek: home ucus kotu degil, kalkis yeri.
        home = _rows(mission_text(WAYPOINTS, home=(46.09, 14.29, 705.0)))[0]
        assert float(home[8]) == pytest.approx(46.09)
        assert float(home[10]) == pytest.approx(705.0)

    def test_takeoff_row_follows_home(self):
        row = _rows(mission_text(WAYPOINTS, takeoff=True))[1]
        assert int(row[3]) == NAV_TAKEOFF
        assert float(row[10]) == pytest.approx(900.0)   # ilk wp kotu

    def test_takeoff_can_be_left_out(self):
        rows = _rows(mission_text(WAYPOINTS, takeoff=False))
        assert all(int(row[3]) == NAV_WAYPOINT for row in rows)
        assert len(rows) == len(WAYPOINTS) + 1          # home + rota

    def test_indices_are_sequential(self):
        rows = _rows(mission_text(WAYPOINTS))
        assert [int(row[0]) for row in rows] == list(range(len(rows)))

    def test_only_home_is_current(self):
        rows = _rows(mission_text(WAYPOINTS))
        assert [row[1] for row in rows] == ["1"] + ["0"] * (len(rows) - 1)

    def test_waypoints_keep_their_coordinates(self):
        rows = _rows(mission_text(WAYPOINTS, takeoff=False))[1:]
        for (lat, lon, alt), row in zip(WAYPOINTS, rows):
            assert float(row[8]) == pytest.approx(lat)
            assert float(row[9]) == pytest.approx(lon)
            assert float(row[10]) == pytest.approx(alt)

    def test_every_row_is_absolute_altitude_frame(self):
        for row in _rows(mission_text(WAYPOINTS)):
            assert int(row[2]) == FRAME_GLOBAL

    def test_autocontinue_is_set_everywhere(self):
        assert all(row[11] == "1" for row in _rows(mission_text(WAYPOINTS)))

    def test_coordinate_precision_survives(self):
        # 8 ondalik ~1 mm. Az yazmak rotayi metrelerce kaydirirdi.
        rows = _rows(mission_text([(46.123456789, 14.987654321, 900.0)],
                                  takeoff=False))
        assert float(rows[1][8]) == pytest.approx(46.123456789, abs=1e-8)
        assert float(rows[1][9]) == pytest.approx(14.987654321, abs=1e-8)

    def test_file_ends_with_newline(self):
        assert mission_text(WAYPOINTS).endswith("\n")

    def test_empty_input_is_refused(self):
        with pytest.raises(ValueError):
            mission_text([])


class TestDeviation:
    def test_no_deviation_when_nothing_dropped(self):
        points = _straight(5)
        assert deviation(points, [0, 1, 2, 3, 4]) == pytest.approx(0.0)

    def test_measures_the_dropped_corner(self):
        points = [(0.0, 0.0, 0.0), (500.0, 300.0, 0.0), (1000.0, 0.0, 0.0)]
        # Kose atilinca sapma kosenin duz cizgiye uzakligi kadar.
        assert deviation(points, [0, 2]) == pytest.approx(300.0)

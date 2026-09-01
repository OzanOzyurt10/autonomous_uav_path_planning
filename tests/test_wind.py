"""Ruzgar ucgeni: yon donusumu, yer hizi ve ucus suresi.

Modul rotanin GEOMETRISINI degistirmiyor; planlanan iz veri kabul edilip
o izi tutmak icin gereken yer hizi ve sure hesaplaniyor. Testler bu
sozlesmeyi ve iki sessiz tuzagi tutuyor: yon donusumu ve yandan ruzgarda
yer hizinin AZALMASI.
"""

import math

import pytest

from src.wind import flight_time, ground_speed, ground_speeds, wind_vector

EAST = 0.0
NORTH = math.pi / 2
WEST = math.pi
SOUTH = -math.pi / 2


class TestWindVector:
    """Meteoroloji acisi ile yerel cerceve arasindaki donusum.

    Iki tersleme ust uste biniyor - GELDIGI yon yerine ESTIGI yon, ve
    kuzeyden saat yonu yerine dogudan saat tersi. Birini atlamak sessizce
    90 derece yanlis cevap verir; asagidaki dort yon onu yakaliyor.
    """

    def test_wind_from_north_blows_south(self):
        wx, wy = wind_vector(10.0, 0.0)
        assert wx == pytest.approx(0.0, abs=1e-9)
        assert wy == pytest.approx(-10.0)

    def test_wind_from_west_blows_east(self):
        wx, wy = wind_vector(10.0, 270.0)
        assert wx == pytest.approx(10.0)
        assert wy == pytest.approx(0.0, abs=1e-9)

    def test_wind_from_east_blows_west(self):
        wx, wy = wind_vector(10.0, 90.0)
        assert wx == pytest.approx(-10.0)
        assert wy == pytest.approx(0.0, abs=1e-9)

    def test_wind_from_south_blows_north(self):
        wx, wy = wind_vector(10.0, 180.0)
        assert wx == pytest.approx(0.0, abs=1e-9)
        assert wy == pytest.approx(10.0)

    def test_magnitude_is_preserved(self):
        for degrees in (0.0, 37.0, 123.5, 250.0, 359.9):
            wx, wy = wind_vector(7.5, degrees)
            assert math.hypot(wx, wy) == pytest.approx(7.5)

    def test_full_turn_is_the_same_wind(self):
        assert wind_vector(6.0, 45.0) == pytest.approx(wind_vector(6.0, 405.0))

    def test_calm_is_the_zero_vector(self):
        assert wind_vector(0.0, 123.0) == pytest.approx((0.0, 0.0))

    def test_negative_speed_is_refused(self):
        with pytest.raises(ValueError):
            wind_vector(-1.0, 0.0)


class TestGroundSpeed:
    AIRSPEED = 28.0

    def test_calm_gives_airspeed(self):
        assert ground_speed(self.AIRSPEED, EAST, (0.0, 0.0)) == \
            pytest.approx(self.AIRSPEED)

    def test_tailwind_adds(self):
        # Doguya giderken doguya esen ruzgar.
        assert ground_speed(self.AIRSPEED, EAST, (8.0, 0.0)) == \
            pytest.approx(36.0)

    def test_headwind_subtracts(self):
        assert ground_speed(self.AIRSPEED, EAST, (-8.0, 0.0)) == \
            pytest.approx(20.0)

    def test_crosswind_reduces_ground_speed(self):
        # Sezgiye aykiri olan bu: tam yandan ruzgarda yer hizi degismez
        # DEGIL, azalir. Ucak burnunu ruzgara kirip suruklenmeyi iptal
        # ederken hava hizinin bir kismini oraya harciyor.
        got = ground_speed(self.AIRSPEED, EAST, (0.0, 8.0))
        assert got == pytest.approx(math.sqrt(28.0 ** 2 - 8.0 ** 2))
        assert got < self.AIRSPEED

    def test_crosswind_sign_does_not_matter(self):
        assert ground_speed(self.AIRSPEED, EAST, (0.0, 8.0)) == \
            pytest.approx(ground_speed(self.AIRSPEED, EAST, (0.0, -8.0)))

    def test_same_wind_helps_one_way_and_hurts_the_other(self):
        wind = (8.0, 0.0)
        assert ground_speed(self.AIRSPEED, EAST, wind) == pytest.approx(36.0)
        assert ground_speed(self.AIRSPEED, WEST, wind) == pytest.approx(20.0)

    def test_course_is_used_not_ignored(self):
        # Kuzeye giderken kuzeye esen ruzgar da kuyruk ruzgari.
        assert ground_speed(self.AIRSPEED, NORTH, (0.0, 8.0)) == \
            pytest.approx(36.0)
        assert ground_speed(self.AIRSPEED, SOUTH, (0.0, 8.0)) == \
            pytest.approx(20.0)

    def test_components_reconstruct_the_wind(self):
        # Ayrisma bir DONDURME, o yuzden ruzgarin buyuklugunu korumali.
        # Eksene paralel ruzgarlarda yanlis bir ayrisma da dogru gorunur;
        # egik yonlerde gorunmez olur.
        wx, wy = 8.0, 6.0                      # buyuklugu tam 10
        for course in (0.3, 0.9, 2.4, -1.7):
            along = wx * math.cos(course) + wy * math.sin(course)
            cross = -wx * math.sin(course) + wy * math.cos(course)
            assert along ** 2 + cross ** 2 == pytest.approx(100.0)
            assert ground_speed(self.AIRSPEED, course, (wx, wy)) == \
                pytest.approx(
                    math.sqrt(self.AIRSPEED ** 2 - cross ** 2) + along)

    def test_diagonal_wind_on_a_diagonal_course(self):
        # Hem ruzgarin hem izin egik oldugu tek sayisal ornek. Eksene
        # paralel testlerin hepsi yanlis bir ayrismayla da gecebiliyor.
        got = ground_speed(28.0, math.pi / 4, (8.0, 6.0))
        assert got == pytest.approx(37.863, abs=1e-3)

    def test_climb_costs_horizontal_speed(self):
        got = ground_speed(self.AIRSPEED, EAST, (0.0, 0.0),
                           climb=math.radians(8.0))
        assert got == pytest.approx(
            self.AIRSPEED * math.cos(math.radians(8.0)))

    def test_crosswind_stronger_than_airspeed_is_refused(self):
        # Yan ruzgar hava hizindan buyukse o iz tutulamaz - ucak
        # surukleniyor. Sessizce sifir dondurmek yalan olurdu.
        with pytest.raises(ValueError):
            ground_speed(10.0, EAST, (0.0, 12.0))

    def test_crosswind_equal_to_airspeed_is_refused(self):
        with pytest.raises(ValueError):
            ground_speed(10.0, EAST, (0.0, 10.0))

    def test_headwind_stronger_than_airspeed_is_refused(self):
        with pytest.raises(ValueError):
            ground_speed(10.0, EAST, (-12.0, 0.0))

    def test_non_positive_airspeed_is_refused(self):
        with pytest.raises(ValueError):
            ground_speed(0.0, EAST, (0.0, 0.0))


def _leg(length, bearing=EAST, climb=0.0, step=100.0, altitude=1000.0):
    """Duz bir bacagi step metre araliklarla pozlara cevirir."""
    poses = []
    travelled = 0.0
    while travelled <= length + 1e-9:
        poses.append((travelled * math.cos(bearing),
                      travelled * math.sin(bearing),
                      altitude + travelled * math.tan(climb),
                      bearing))
        travelled += step
    return poses


class TestFlightTime:
    AIRSPEED = 28.0

    def test_calm_matches_distance_over_airspeed(self):
        poses = _leg(10000.0)
        assert flight_time(poses, self.AIRSPEED, (0.0, 0.0)) == \
            pytest.approx(10000.0 / self.AIRSPEED)

    def test_tailwind_is_faster_headwind_is_slower(self):
        poses = _leg(10000.0)
        tail = flight_time(poses, self.AIRSPEED, (8.0, 0.0))
        head = flight_time(poses, self.AIRSPEED, (-8.0, 0.0))
        assert tail == pytest.approx(10000.0 / 36.0)
        assert head == pytest.approx(10000.0 / 20.0)
        assert tail < head

    def test_out_and_back_costs_more_than_calm(self):
        # Ruzgarda gidip gelmek her zaman daha uzun surer: kuyruk
        # ruzgarinda kazanilan sure, karsi ruzgarda kaybedileni
        # kapatmiyor. Klasik sonuc, hesabin dogrulugunun iyi bir isareti.
        out = flight_time(_leg(10000.0, EAST), self.AIRSPEED, (8.0, 0.0))
        back = flight_time(_leg(10000.0, WEST), self.AIRSPEED, (8.0, 0.0))
        calm = 2 * 10000.0 / self.AIRSPEED
        assert out + back > calm

    def test_calm_climb_costs_three_d_length_over_airspeed(self):
        # Sakin havada tirmanan bir bacakta sure, UC BOYUTLU uzunlugun
        # hava hizina bolumu. Yatay hiz cos(climb) kadar dusuyor ama
        # yatay mesafe de o kadar kisa; ikisi sadelesiyor. Yatay hizi
        # uc boyutlu mesafeyle bolmek sureyi 1/cos(climb) kadar sisirir.
        climb = math.radians(8.0)
        poses = _leg(10000.0, climb=climb)
        length_3d = 10000.0 / math.cos(climb)
        assert flight_time(poses, self.AIRSPEED, (0.0, 0.0)) == \
            pytest.approx(length_3d / self.AIRSPEED)

    def test_climb_lengthens_the_flight(self):
        level = flight_time(_leg(10000.0), self.AIRSPEED, (0.0, 0.0))
        climbing = flight_time(_leg(10000.0, climb=math.radians(8.0)),
                               self.AIRSPEED, (0.0, 0.0))
        assert climbing > level

    def test_repeated_poses_are_skipped(self):
        # Bacak sinirlarinda pozlar ust uste biniyor; sifir uzunluklu
        # parca atan2(0, 0) ve sifira bolme demek.
        poses = _leg(1000.0)
        doubled = [pose for pose in poses for _ in range(2)]
        assert flight_time(doubled, self.AIRSPEED, (5.0, 0.0)) == \
            pytest.approx(flight_time(poses, self.AIRSPEED, (5.0, 0.0)))

    def test_short_input_takes_no_time(self):
        assert flight_time([], self.AIRSPEED, (0.0, 0.0)) == 0.0
        assert flight_time([(0.0, 0.0, 0.0, 0.0)], self.AIRSPEED,
                           (0.0, 0.0)) == 0.0

    def test_unflyable_leg_is_reported(self):
        with pytest.raises(ValueError):
            flight_time(_leg(1000.0), 10.0, (0.0, 12.0))


class TestGroundSpeeds:
    def test_one_speed_per_segment(self):
        poses = _leg(1000.0, step=100.0)          # 11 poz, 10 parca
        assert len(ground_speeds(poses, 28.0, (0.0, 0.0))) == len(poses) - 1

    def test_speeds_follow_the_turn(self):
        # Doguya sonra batiya giden rota: ayni ruzgar once kuyruk, sonra
        # karsi. Arayuzde "en yavas kesim" bunu gosterecek.
        poses = _leg(1000.0, EAST) + _leg(1000.0, WEST)
        speeds = ground_speeds(poses, 28.0, (8.0, 0.0))
        assert max(speeds) == pytest.approx(36.0)
        assert min(speeds) == pytest.approx(20.0)

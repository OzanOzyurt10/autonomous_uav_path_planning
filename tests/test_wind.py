"""Ruzgar ucgeni: yon donusumu, yer hizi ve ucus suresi.

Modul rotanin GEOMETRISINI degistirmiyor; planlanan iz veri kabul edilip
o izi tutmak icin gereken yer hizi ve sure hesaplaniyor. Testler bu
sozlesmeyi ve iki sessiz tuzagi tutuyor: yon donusumu ve yandan ruzgarda
yer hizinin AZALMASI.
"""

import math

import pytest

from src.wind import (ProfileWind, ShearWind, UniformWind, flight_time,
                      ground_speed, ground_speeds, wind_vector)

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


class TestUniformWind:
    """Alan arayuzunun tekduze hali: her yerde ayni vektor.

    Bu sinif davranis eklemiyor, mevcut davranisi alan sozlesmesine
    sokuyor - eski cagrilar bozulmadan ShearWind ile yer degistirebilsin.
    """

    def test_same_vector_everywhere(self):
        field = UniformWind(10.0, 270.0)
        assert field.at(0.0, 0.0, 0.0) == field.at(9000.0, 4000.0, 3000.0)

    def test_matches_wind_vector(self):
        field = UniformWind(10.0, 270.0)
        assert field.at(0.0, 0.0, 500.0) == pytest.approx(
            wind_vector(10.0, 270.0))

    def test_strongest_is_the_speed(self):
        assert UniformWind(7.5, 0.0).strongest() == pytest.approx(7.5)

    def test_negative_speed_rejected(self):
        with pytest.raises(ValueError):
            UniformWind(-1.0, 0.0)


class TestShearWind:
    """Guc yasasi: u(h) = u_ref * (h / h_ref) ** alpha, h YERDEN yukseklik.

    Iki sinir kritik. Tavansiz profil sinirsiz buyur ve planlayici arka
    ruzgarda her seferinde tavana tirmanir; taban olmadan da yerde ruzgar
    sifira duser ve karsi ruzgarda araziye yapismak bedava gorunur.
    """

    REF = 10.0
    TOP = 500.0

    def field(self, ground=None, speed=10.0, alpha=0.14):
        return ShearWind(speed, 270.0, ground=ground, ref_height=self.REF,
                         alpha=alpha, top=self.TOP)

    def test_reference_height_gives_reference_speed(self):
        speed = math.hypot(*self.field().at(0.0, 0.0, self.REF))
        assert speed == pytest.approx(10.0)

    def test_speed_grows_with_altitude(self):
        field = self.field()
        low = math.hypot(*field.at(0.0, 0.0, 50.0))
        high = math.hypot(*field.at(0.0, 0.0, 300.0))
        assert high > low

    def test_power_law_value(self):
        # 100 m'de: 10 * (100/10) ** 0.14
        speed = math.hypot(*self.field().at(0.0, 0.0, 100.0))
        assert speed == pytest.approx(10.0 * 10.0 ** 0.14)

    def test_capped_above_top(self):
        field = self.field()
        at_top = math.hypot(*field.at(0.0, 0.0, self.TOP))
        far_above = math.hypot(*field.at(0.0, 0.0, 5000.0))
        assert far_above == pytest.approx(at_top)

    def test_floor_below_reference_height(self):
        # Yerde sifira dusmemeli: duserse karsi ruzgarda araziye yapismak
        # sonsuz kazancli gorunur ve planlayici tabana yapisir.
        field = self.field()
        assert math.hypot(*field.at(0.0, 0.0, 0.0)) == pytest.approx(10.0)

    def test_direction_does_not_veer(self):
        field = self.field()
        low = field.at(0.0, 0.0, 20.0)
        high = field.at(0.0, 0.0, 400.0)
        assert math.atan2(*reversed(low)) == pytest.approx(
            math.atan2(*reversed(high)))

    def test_direction_matches_meteorology(self):
        # Batidan esen ruzgar doguya gider: +x bileseni pozitif olmali.
        assert self.field().at(0.0, 0.0, 100.0)[0] > 0.0

    def test_height_is_above_ground_not_sea_level(self):
        # Ayni MSL irtifasi, iki farkli zemin: daginin ustunde AGL kucuk,
        # dolayisiyla ruzgar zayif. MSL kullanilsaydi ikisi esit cikardi.
        valley = self.field(ground=lambda x, y: 0.0)
        ridge = self.field(ground=lambda x, y: 900.0)
        assert (math.hypot(*ridge.at(0.0, 0.0, 1000.0))
                < math.hypot(*valley.at(0.0, 0.0, 1000.0)))

    def test_ground_above_aircraft_does_not_crash(self):
        # Arazi verisi ile poz cakisabilir; negatif AGL taban degerine
        # kirpilmali, negatif sayinin kesirli kuvveti karmasik sayi verir.
        field = self.field(ground=lambda x, y: 2000.0)
        assert math.hypot(*field.at(0.0, 0.0, 500.0)) == pytest.approx(10.0)

    def test_strongest_is_the_capped_speed(self):
        field = self.field()
        assert field.strongest() == pytest.approx(
            math.hypot(*field.at(0.0, 0.0, self.TOP)))

    def test_zero_alpha_is_uniform(self):
        field = self.field(alpha=0.0)
        assert (math.hypot(*field.at(0.0, 0.0, 20.0))
                == pytest.approx(math.hypot(*field.at(0.0, 0.0, 480.0))))

    def test_negative_speed_rejected(self):
        with pytest.raises(ValueError):
            self.field(speed=-1.0)

    def test_negative_alpha_rejected(self):
        with pytest.raises(ValueError):
            self.field(alpha=-0.1)

    def test_non_positive_reference_height_rejected(self):
        with pytest.raises(ValueError):
            ShearWind(10.0, 270.0, ref_height=0.0)

    def test_top_below_reference_height_rejected(self):
        with pytest.raises(ValueError):
            ShearWind(10.0, 270.0, ref_height=100.0, top=50.0)


class TestProfileWind:
    """Olculmus seviyeler arasinda interpolasyon.

    Gercek profil guc yasasina benzemiyor: hiz irtifayla tek yonlu artmiyor
    (Riva'da 805 m'de tepe yapip dusuyor) ve yon doniyor (1.5 km'de 33
    derece). Bu sinif formul uydurmuyor, olculen seviyeleri veri kabul
    ediyor. Seviyeler MSL - basinc seviyesi verisi oyle geliyor.
    """

    LEVELS = [(100.0, 4.0, 270.0), (500.0, 8.0, 270.0),
              (1000.0, 6.0, 270.0)]

    def test_returns_the_level_value_at_that_height(self):
        field = ProfileWind(self.LEVELS)
        assert math.hypot(*field.at(0.0, 0.0, 500.0)) == pytest.approx(8.0)

    def test_interpolates_between_levels(self):
        field = ProfileWind(self.LEVELS)
        middle = math.hypot(*field.at(0.0, 0.0, 300.0))
        assert middle == pytest.approx(6.0)

    def test_speed_may_fall_with_altitude(self):
        # Guc yasasinin yapamadigi sey: 500 m'de tepe, ustunde dusus.
        field = ProfileWind(self.LEVELS)
        assert (math.hypot(*field.at(0.0, 0.0, 1000.0))
                < math.hypot(*field.at(0.0, 0.0, 500.0)))

    def test_clamped_below_lowest_level(self):
        field = ProfileWind(self.LEVELS)
        assert field.at(0.0, 0.0, 0.0) == field.at(0.0, 0.0, 100.0)

    def test_clamped_above_highest_level(self):
        field = ProfileWind(self.LEVELS)
        assert field.at(0.0, 0.0, 9000.0) == field.at(0.0, 0.0, 1000.0)

    def test_direction_interpolates_across_the_seam(self):
        # 350 ile 10 derecenin ortasi 0'dir, 180 degil. Hiz ve yonu ayri
        # ayri interpole etmek tam burada ters yone bakan bir ruzgar verir;
        # o yuzden VEKTOR interpole ediliyor.
        field = ProfileWind([(100.0, 10.0, 350.0), (200.0, 10.0, 10.0)])
        wx, wy = field.at(0.0, 0.0, 150.0)
        assert abs(wx) < 0.01
        assert wy < 0.0                  # kuzeyden esiyor: guneye gidiyor

    def test_single_level_is_uniform(self):
        field = ProfileWind([(300.0, 7.0, 90.0)])
        assert field.at(0.0, 0.0, 0.0) == field.at(0.0, 0.0, 5000.0)

    def test_unsorted_levels_are_ordered(self):
        field = ProfileWind(list(reversed(self.LEVELS)))
        assert math.hypot(*field.at(0.0, 0.0, 300.0)) == pytest.approx(6.0)

    def test_repeated_height_does_not_divide_by_zero(self):
        field = ProfileWind([(100.0, 4.0, 270.0), (100.0, 9.0, 270.0),
                             (200.0, 6.0, 270.0)])
        assert math.hypot(*field.at(0.0, 0.0, 150.0)) > 0.0

    def test_strongest_covers_the_whole_profile(self):
        # Alt sinir buna dayaniyor. Iki vektorun dogrusal karisiminin
        # buyuklugu ikisinin buyugunu asamaz, o yuzden seviyelerin
        # maksimumu butun profil icin gecerli bir ust sinir.
        field = ProfileWind(self.LEVELS)
        assert field.strongest() == pytest.approx(8.0)
        for height in range(0, 1200, 37):
            assert math.hypot(*field.at(0.0, 0.0, float(height))) <= 8.0 + 1e-9

    def test_empty_profile_rejected(self):
        with pytest.raises(ValueError):
            ProfileWind([])

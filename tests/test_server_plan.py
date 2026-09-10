"""plan_payload ve cevresi: istek yukunden rotaya kadar olan karar zinciri.

test_server.py yalnizca yan etkisiz kurallara bakiyor. Buradakiler asil
dalli fonksiyonu sinar: modun nasil secildigi, 3B takilinca ne oldugu,
harita tavaninin yasak bolgelerden ONCE kurulmasi, ve otopilota gidecek
sayilarin dogru kottan cikmasi.

Arazi sahte bir karo kutuphanesinden geliyor; testler aga hic cikmiyor.
Tohum sayisi 1'e cekiliyor - burada aranan sey rota KALITESI degil,
kararlarin dogrulugu.
"""

import array
import math

import pytest

import app.mission as mission
import app.server as server
from app.mission import Mission, indicated_airspeed
from src.forecast import ForecastUnavailable
from src.geo import Frame
from src.rrt_star import LENGTH
from src.terrain import MissingTiles, Terrain
from src.terrarium import ElevationUnavailable
from src.wind import REF_HEIGHT, ShearWind

BASE_LAT, BASE_LON = 40.0, 30.0
SPACING = 200.0
COLS, ROWS = 31, 61              # 6000 x 12000 m
GROUND = 200.0


def _terrain(build):
    """Sentetik arazi; build(x, y) metre cinsinden kot veriyor."""
    heights = array.array("h")
    for row in range(ROWS):
        for col in range(COLS):
            heights.append(int(round(build(col * SPACING, row * SPACING))))
    return Terrain(heights, COLS, ROWS, SPACING, SPACING)


def _flat():
    return _terrain(lambda x, y: GROUND)


class _Store:
    """STORE'un yerine gecen sahte kutuphane; hep ayni araziyi veriyor."""

    def __init__(self, terrain):
        self.terrain = terrain

    def window(self, lat, lon, width_m, height_m):
        if self.terrain is None:
            raise MissingTiles(["N40E030.hgt"])
        return self.terrain

    def coverage(self):
        return []


class _NoDownload:
    """Arazi indirmeyi kapatir: testler ag beklemeden kosmali."""

    def window(self, *args, **kwargs):
        raise ElevationUnavailable("test: indirme kapali")


@pytest.fixture
def plan(monkeypatch):
    """plan_payload'i sahte araziyle kosturan yardimci."""
    def run(terrain, body):
        monkeypatch.setattr(server, "STORE", _Store(terrain))
        monkeypatch.setattr(server, "TERRARIUM", _NoDownload())
        # Rota kalitesi burada aranmiyor; tek tohum testleri hizli tutuyor.
        monkeypatch.setattr(mission, "SEED_TRIES", 1)
        return server.plan_payload(body)
    return run


def _north_south(gap_deg=0.054, pins=None):
    """Kuzey-guney iki waypoint; arada ~6 km."""
    points = [[BASE_LAT, BASE_LON], [BASE_LAT + gap_deg, BASE_LON]]
    if pins is not None:
        for point, agl in zip(points, pins):
            if agl is not None:
                point.append(agl)
    return points


def _body(points, **extra):
    body = {"waypoints": points, "speed": 28.0, "bank_deg": 30.0,
            "climb_deg": 8.0, "clearance": 10.0, "iterations": 600,
            "seed": 1}
    body.update(extra)
    return body


class TestModeIsDerived:
    """Mod artik kullanicinin sectigi bir sey degil, arazinin sonucu.

    Uc yol var ve ucu de farkli sebeple 2B'ye cikabiliyor; arayuz hangisi
    oldugunu yazmak zorunda, o yuzden bayraklar da sinaniyor.
    """

    def test_terrain_gives_three_d(self, plan):
        out = plan(_flat(), _body(_north_south()))
        assert out["mode"] == "3d"
        assert out["has_terrain"] is True
        assert out["fell_back"] is False
        assert out["forced_2d"] is False

    def test_no_terrain_falls_to_two_d(self, plan):
        # Arazi yoksa 3B kurulamaz; hata degil, 2B'ye dusuyor. Uygulamanin
        # dunyanin her yerinde calisabilmesinin sarti bu.
        out = plan(None, _body(_north_south()))
        assert out["mode"] == "2d"
        assert out["has_terrain"] is False
        assert out["fell_back"] is False        # arazi yoklugu geri dusme degil
        assert out["agl"] is None               # zemin bilinmiyor

    def test_ignore_terrain_forces_two_d(self, plan):
        out = plan(_flat(), _body(_north_south(), ignore_terrain=True))
        assert out["mode"] == "2d"
        assert out["forced_2d"] is True
        assert out["fell_back"] is False
        # Arazi VAR, yalnizca planlamada kullanilmiyor: AGL profili yine
        # cikariliyor ki kullanici ne kadar alcaktan gectigini gorsun.
        assert out["agl"] is not None

    def test_unfound_three_d_falls_back(self, plan, monkeypatch):
        """3B bulunamayinca gorev cope atilmiyor, 2B'ye duruluyor.

        Arazi ile bunu kurmak mumkun degil: tavan hep en yuksek tepenin
        410 m ustunde ve o katman haritanin her yerinde serbest, ustelik
        Dubins airplane helis turuyla yerinde tirmanabiliyor. Gercek
        tetikleyici yineleme butcesinin tukenmesi; burada dogrudan o
        sonuc uretiliyor ki test planlayicinin gunune bagli olmasin.
        """
        def nothing_found(waypoints, env, constraints, headings=None,
                          model=None):
            return Mission([], False, math.inf, math.inf)

        monkeypatch.setattr(server, "plan_mission", nothing_found)
        out = plan(_flat(), _body(_north_south()))
        assert out["mode"] == "2d"
        assert out["fell_back"] is True
        assert out["forced_2d"] is False
        assert out["ok"] is True                # 2B bulabilmeli


class TestCeilingBeforeZones:
    """Harita tavani yasak bolgelerden ONCE kurulmali.

    Bolgelerin z_max'i tavandan geliyor. Sira bozulursa bolgeler alcak
    kalir, ucak ustlerinden gecer ve HICBIR SEY uyarmaz - ne hata, ne
    log. Sabitlenmis yuksek bir kot tam da bu sirayi zorluyor.
    """

    ZONE_AGL = 1500.0

    def _run(self, plan, pinned):
        points = _north_south(pins=[pinned, pinned])
        # Bolge tam iki waypointin ortasinda: rota dolasmak ZORUNDA.
        zones = [{"lat": BASE_LAT + 0.027, "lon": BASE_LON,
                  "radius_m": 900.0}]
        return plan(_flat(), _body(points, zones=zones))

    def test_route_avoids_the_zone_under_a_pinned_altitude(self, plan):
        out = self._run(plan, self.ZONE_AGL)
        assert out["ok"]
        zone = out["zones"][0]
        for pose in out["path"]:
            gap = math.hypot(pose[0] - zone["x"], pose[1] - zone["y"])
            assert gap >= zone["radius"]

    def test_pinned_altitude_actually_raised_the_ceiling(self, plan):
        # Testin anlamli oldugunun kaniti: sabitleme eski tavanin
        # (zemin + pay + 100 + 300) uzerine cikmali, yoksa sira hatasi
        # zaten ortaya cikmazdi.
        out = self._run(plan, self.ZONE_AGL)
        plain = GROUND + 10.0 + mission.ALTITUDE_MARGIN + server.CLIMB_ROOM
        assert out["ceiling"] > plain
        assert max(pose[2] for pose in out["path"]) > plain - server.CLIMB_ROOM


class TestMissionPayload:
    """Gorev dosyasiyla birlikte giden otopilot sayilari."""

    FRAME = Frame.for_window(BASE_LAT, BASE_LON, 10000.0)
    POSES = [(0.0, float(i) * 100.0, 500.0 + i * 100.0, 0.0) for i in range(11)]

    def _payload(self, airspeed=28.0):
        return server._mission_payload(self.POSES, self.FRAME, None, airspeed)

    def test_mean_altitude_is_the_average_of_the_poses(self):
        got = self._payload()["mean_altitude"]
        want = sum(p[2] for p in self.POSES) / len(self.POSES)
        assert got == pytest.approx(want)

    def test_cruise_comes_from_the_mean_altitude(self):
        payload = self._payload()
        assert payload["airspeed_cruise"] == pytest.approx(
            indicated_airspeed(28.0, payload["mean_altitude"]))

    def test_cruise_follows_the_planned_speed(self):
        slow = self._payload(20.0)["airspeed_cruise"]
        fast = self._payload(30.0)["airspeed_cruise"]
        assert fast > slow
        assert fast / slow == pytest.approx(30.0 / 20.0)

    def test_home_sits_below_the_lowest_point(self):
        # SITL zemini duz ve home irtifasinda; home rotanin ustunde
        # kalirsa ucak yerin altina inmeye calisir.
        payload = self._payload()
        lowest = min(p[2] for p in self.POSES)
        assert payload["home_alt"] == pytest.approx(lowest - server.HOME_MARGIN)
        assert payload["home_alt"] < lowest

    def test_empty_route_reports_nothing(self):
        payload = server._mission_payload([], self.FRAME, None, 28.0)
        assert payload["airspeed_cruise"] is None
        assert payload["home_alt"] is None
        assert payload["points"] == 0

    def test_numbers_reach_the_response(self, plan):
        out = plan(_flat(), _body(_north_south()))
        payload = out["mission_file"]
        assert payload["airspeed_cruise"] == pytest.approx(
            indicated_airspeed(28.0, payload["mean_altitude"]))


class TestConstraintsFrom:
    """Istek yukundeki sayilarin Constraints'e cevrilmesi."""

    def test_degrees_become_radians(self):
        got = server._constraints_from({"bank_deg": 45.0, "climb_deg": 10.0})
        assert got.max_bank == pytest.approx(math.radians(45.0))
        assert got.max_climb == pytest.approx(math.radians(10.0))

    def test_defaults_match_the_interface(self):
        got = server._constraints_from({})
        assert got.speed == 28.0
        assert got.clearance == 10.0
        assert got.seed == 1

    def test_turn_radius_is_derived(self):
        got = server._constraints_from({"speed": 28.0, "bank_deg": 30.0})
        assert got.rho == pytest.approx(
            28.0 ** 2 / (9.80665 * math.tan(math.radians(30.0))))


class TestZonesFrom:
    """Yasak bolgelerin yerel metreye cevrilmesi ve dogrulanmasi."""

    BOUNDS = (0.0, 0.0, 100.0, 6000.0, 12000.0, 2500.0)
    FRAME = Frame.for_window(BASE_LAT, BASE_LON, 12000.0)

    def _zones(self, raw):
        return server._zones_from({"zones": raw}, self.FRAME, self.BOUNDS)

    def test_height_spans_the_whole_map(self):
        # Varsayilan sutun tavandan tabana: kullanici irtifa girmediyse
        # bolge her kotta yasak olmali, ucak ustunden gecmemeli.
        zone = self._zones([{"lat": BASE_LAT, "lon": BASE_LON,
                             "radius_m": 500.0}])[0]
        assert zone.z_min == self.BOUNDS[2]
        assert zone.z_max == self.BOUNDS[5]

    def test_position_is_converted(self):
        zone = self._zones([{"lat": BASE_LAT, "lon": BASE_LON,
                             "radius_m": 500.0}])[0]
        assert (zone.x, zone.y) == pytest.approx(
            self.FRAME.to_local(BASE_LAT, BASE_LON))

    def test_zero_radius_is_refused(self):
        with pytest.raises(ValueError, match="pozitif"):
            self._zones([{"lat": BASE_LAT, "lon": BASE_LON, "radius_m": 0.0}])

    def test_too_many_zones_are_refused(self):
        many = [{"lat": BASE_LAT, "lon": BASE_LON, "radius_m": 100.0}
                for _ in range(server.MAX_ZONES + 1)]
        with pytest.raises(ValueError, match="en fazla"):
            self._zones(many)

    def test_no_zones_is_fine(self):
        assert self._zones([]) == []


class TestWindPayload:
    """Ruzgar rotayi degil, rotanin SURESINI degistiriyor."""

    POSES = [(float(i) * 100.0, 0.0, 500.0) for i in range(51)]   # 5 km, doguya

    def _wind(self, speed, from_deg):
        return server._wind_payload(self.POSES, 28.0, speed, from_deg)

    def test_calm_air_matches_the_airspeed(self):
        got = self._wind(0.0, 0.0)
        assert got["duration"] == pytest.approx(5000.0 / 28.0)
        assert got["message"] == ""

    def test_headwind_takes_longer_than_tailwind(self):
        head = self._wind(8.0, 90.0)      # doguya giderken doguDAN
        tail = self._wind(8.0, 270.0)
        assert head["duration"] > tail["duration"]

    def test_ground_speed_range_is_reported(self):
        got = self._wind(8.0, 270.0)
        assert got["min_speed"] == pytest.approx(36.0, abs=0.1)
        assert got["max_speed"] == pytest.approx(36.0, abs=0.1)

    def test_unflyable_crosswind_explains_itself(self):
        # Yan ruzgar hava hizini asiyor: rota gecerli ama UCULAMAZ.
        # Plani cope atmak yerine sebebi soyleyip rotayi geri veriyoruz.
        got = self._wind(40.0, 0.0)
        assert got["duration"] is None
        assert got["message"]

    def test_empty_route_is_quiet(self):
        got = server._wind_payload([], 28.0, 8.0, 0.0)
        assert got["duration"] is None
        assert got["message"] == ""


class TestInputLimits:
    """Girdi hatalari sessizce gecmemeli."""

    def test_one_waypoint_is_refused(self, plan):
        with pytest.raises(ValueError, match="en az iki"):
            plan(_flat(), _body([[BASE_LAT, BASE_LON]]))

    def test_too_many_waypoints_are_refused(self, plan):
        many = [[BASE_LAT + i * 0.01, BASE_LON]
                for i in range(server.MAX_WAYPOINTS + 1)]
        with pytest.raises(ValueError, match="en fazla"):
            plan(_flat(), _body(many))

    def test_pinned_altitude_below_clearance_is_refused(self, plan):
        # Environment3D bunu "harita disinda" diye bildirir ve kullanici
        # sebebi goremez; burada acik mesajla reddediliyor.
        points = _north_south(pins=[5.0, None])
        with pytest.raises(ValueError, match="emniyet"):
            plan(_flat(), _body(points, clearance=100.0))


class TestDisplayHelpers:
    """Yalnizca cizim icin olan yardimcilar; planlayici bunlari gormuyor."""

    def test_small_grid_is_sent_whole(self):
        assert server._display_stride(_flat()) == 1

    def test_large_grid_is_thinned(self):
        # 100 km karede yarim milyon post var, Plotly yuzeyi orada
        # kilitleniyor. Planlayici yine tam cozunurluk goruyor.
        big = Terrain(array.array("h", [0] * (600 * 600)), 600, 600, 90.0, 90.0)
        stride = server._display_stride(big)
        assert stride > 1
        assert (600 // stride) * (600 // stride) <= server.DISPLAY_POSTS

    def test_pose_is_rounded_for_the_payload(self):
        got = server._round_pose((1.23456, 2.34567, 3.45678, 0.123456))
        assert got == [1.23, 2.35, 3.46, 0.1235]

    def test_yaw_is_wrapped(self):
        # Yaw Dubins boyunca birikip 2*pi'yi asabiliyor; arayuz onu
        # dogrudan aci olarak kullaniyor.
        got = server._round_pose((0.0, 0.0, 0.0, 3 * math.pi))
        assert 0.0 <= got[3] < 2 * math.pi
        assert got[3] == pytest.approx(math.pi, abs=1e-3)

    def test_coverage_lists_the_library(self, monkeypatch):
        class _Two:
            def coverage(self):
                return [(46, 14), (41, 29)]
        monkeypatch.setattr(server, "STORE", _Two())
        got = server.coverage_payload()
        assert got["max_window"] == server.MAX_WINDOW
        assert [t["name"] for t in got["tiles"]] == ["N46E014", "N41E029"]


class TestLoiter:
    """Waypoint basina bekleme: sureye giriyor, gorev dosyasina komut oluyor.

    En kritik nokta seyreltme: gorev dosyasi ORNEKLENMIS rotadan yaziliyor
    ve Douglas-Peucker bekleme noktasini atabilir. Atarsa bekleme sessizce
    kaybolur - dosya gecerli gorunur, ucak beklemeden gecer.
    """

    def _plan(self, plan, holds):
        points = _north_south()
        for point, seconds in zip(points, holds):
            # Sira: kot (AGL), bas acisi, bekleme. Ikisini de None
            # birakmak "otomatik" demek; saniyeyi yanlis sutuna koymak
            # sessizce bas acisi olurdu.
            point.extend([None, None, seconds])
        return plan(_flat(), _body(points))

    def test_waiting_is_added_to_the_duration(self, plan):
        without = self._plan(plan, [0, 0])
        withhold = self._plan(plan, [0, 120])
        assert withhold["loiter_total"] == pytest.approx(120.0)
        assert withhold["duration"] == pytest.approx(without["duration"] + 120.0)

    def test_route_geometry_does_not_change(self, plan):
        # Bekleme ucagin ayni noktada donmesi; rota ayni kalmali.
        without = self._plan(plan, [0, 0])
        withhold = self._plan(plan, [0, 120])
        assert withhold["cost"] == pytest.approx(without["cost"])

    def test_wind_duration_includes_the_wait(self, plan):
        out = self._plan(plan, [0, 90])
        # Ruzgar verilmedi; ruzgarli sure de beklemeyi kapsamali.
        assert out["wind"]["duration"] == pytest.approx(
            out["duration"], rel=1e-6)

    def test_mission_file_carries_the_command(self, plan):
        out = self._plan(plan, [0, 60])
        assert out["mission_file"]["loiter_points"] == 1
        rows = [line.split("\t")
                for line in out["mission_file"]["text"].splitlines()[1:]]
        held = [row for row in rows if row[3] == "19"]
        assert len(held) == 1
        assert float(held[0][4]) == pytest.approx(60.0)

    def test_the_hold_survives_thinning(self, plan):
        # Asil korunan sey: seyreltme bekleme pozunu atamaz.
        out = self._plan(plan, [0, 60])
        rows = [line.split("\t")
                for line in out["mission_file"]["text"].splitlines()[1:]]
        held = [row for row in rows if row[3] == "19"][0]
        last = rows[-1]
        # Bekleme son waypointte: konumu son satirinkiyle ayni olmali.
        assert held[8:11] == last[8:11] or held is last

    def test_no_hold_leaves_the_file_untouched(self, plan):
        out = self._plan(plan, [0, 0])
        assert out["loiter_total"] == 0.0
        assert out["mission_file"]["loiter_points"] == 0
        assert "\t19\t" not in out["mission_file"]["text"]

    def test_negative_wait_is_refused(self, plan):
        with pytest.raises(ValueError, match="bekleme"):
            self._plan(plan, [0, -10])

    def test_absurd_wait_is_refused(self, plan):
        # Otopilot beklerken gorev ilerlemiyor; tek bir yanlis sayi
        # ucusu saatlerce kilitler.
        with pytest.raises(ValueError, match="bekleme"):
            self._plan(plan, [0, server.MAX_LOITER + 1])


class TestWindPlanning:
    """Rota hedefi: mesafe mi sure mi kucultuluyor.

    Tekduze ruzgar rotayi degistirmiyor (Zermelo), o yuzden anahtar acikken
    kurulan alan irtifayla degisen profil. Buradaki testler modelin gercekten
    planlayiciya gectigini ve surenin ayni alandan okundugunu tutuyor.
    """

    def _windy(self, **extra):
        return _body(_north_south(), wind_speed=10.0, wind_from=270.0,
                     **extra)

    def test_length_objective_by_default(self, plan):
        out = plan(_flat(), self._windy())
        assert out["wind"]["planned"] is False

    def test_time_objective_turns_it_on(self, plan):
        out = plan(_flat(), self._windy(objective="time"))
        assert out["wind"]["planned"] is True

    def test_zero_wind_stays_off(self, plan):
        # Sifir ruzgarda profil bir sey degistirmiyor; maliyet modeli
        # kurmak planlamayi bosuna yavaslatirdi.
        out = plan(_flat(), _body(_north_south(), wind_speed=0.0,
                                  objective="time"))
        assert out["wind"]["planned"] is False

    def test_route_sees_more_wind_than_entered(self, plan):
        # Girilen deger 10 m'deki ruzgar, rota ondan yuksekte uculuyor.
        # Ikisi esit cikarsa profil degil tekduze alan kurulmus demektir.
        out = plan(_flat(), self._windy(objective="time"))
        assert out["wind"]["mean_speed"] > out["wind"]["speed"]

    def test_model_reaches_the_three_d_planner(self, plan, monkeypatch):
        seen = []
        real = server.plan_mission

        def spy(waypoints, env, constraints, headings=None, model=LENGTH):
            seen.append(model)
            return real(waypoints, env, constraints, headings, model)

        monkeypatch.setattr(server, "plan_mission", spy)
        plan(_flat(), self._windy(objective="time"))
        assert seen and seen[0] is not LENGTH

    def test_model_reaches_the_two_d_planner(self, plan, monkeypatch):
        # 2B'de rota degismiyor ama model yine de gecmeli: yan ruzgarin
        # hava hizini astigi yonleri ancak o eliyor.
        seen = []
        real = server.plan_mission_2d

        def spy(waypoints, env, constraints, altitude, headings=None,
                model=LENGTH):
            seen.append(model)
            return real(waypoints, env, constraints, altitude, headings,
                        model)

        monkeypatch.setattr(server, "plan_mission_2d", spy)
        plan(_flat(), self._windy(objective="time", ignore_terrain=True))
        assert seen and seen[0] is not LENGTH

    def test_planner_stays_on_length_when_off(self, plan, monkeypatch):
        seen = []
        real = server.plan_mission

        def spy(waypoints, env, constraints, headings=None, model=LENGTH):
            seen.append(model)
            return real(waypoints, env, constraints, headings, model)

        monkeypatch.setattr(server, "plan_mission", spy)
        plan(_flat(), self._windy())
        assert seen and seen[0] is LENGTH

    def test_payload_reads_the_given_field(self):
        # Sure rotayi PLANLAYAN alandan okunmali; baska bir alandan
        # okunursa arayuz rotayla celisen bir sayi gosterir.
        field = ShearWind(10.0, 270.0)
        got = server._wind_payload(TestWindPayload.POSES, 28.0, 10.0, 270.0,
                                   field)
        assert got["planned"] is True
        assert got["ref_height"] == pytest.approx(REF_HEIGHT)
        assert got["mean_speed"] > 10.0

    def test_objective_is_echoed_back(self, plan):
        # Sure hedefi secilip ruzgar sifirsa alan kurulmuyor ve iki mod
        # ayni sonucu veriyor; arayuz bunu soyleyebilmek icin istenen
        # hedefi geri almak zorunda.
        out = plan(_flat(), _body(_north_south(), wind_speed=0.0,
                                  objective="time"))
        assert out["wind"]["objective"] == "time"
        assert out["wind"]["planned"] is False

    def test_unknown_objective_is_rejected(self, plan):
        with pytest.raises(ValueError):
            plan(_flat(), _body(_north_south(), objective="enerji"))


class TestFetchedProfile:
    """Cekilen tahmin profili: guc yasasinin yerine geciyor.

    Profil MSL seviyelerinden geliyor, arazi gerekmiyor. Cekilen veri
    varken formule dusmek sessiz bir gerileme olurdu, o yuzden hangi
    modelin kullanildigi yukte yaziyor.
    """

    LEVELS = [[300.0, 6.0, 270.0], [800.0, 12.0, 270.0],
              [1500.0, 9.0, 280.0]]

    def test_profile_is_used_when_given(self, plan):
        out = plan(_flat(), _body(_north_south(), wind_speed=5.0,
                                  wind_from=270.0, objective="time",
                                  wind_profile=self.LEVELS))
        assert out["wind"]["planned"] is True
        assert out["wind"]["source"] == server.PROFILE_FETCHED

    def test_formula_is_used_without_a_profile(self, plan):
        out = plan(_flat(), _body(_north_south(), wind_speed=5.0,
                                  wind_from=270.0, objective="time"))
        assert out["wind"]["source"] == server.PROFILE_FORMULA

    def test_profile_wins_even_with_zero_typed_speed(self, plan):
        # Kutudaki sayi profil varken yalnizca bir gosterge; alan
        # seviyelerden kuruluyor. Sifir hiz profili iptal etmemeli.
        out = plan(_flat(), _body(_north_south(), wind_speed=0.0,
                                  objective="time",
                                  wind_profile=self.LEVELS))
        assert out["wind"]["planned"] is True
        assert out["wind"]["mean_speed"] > 0.0

    def test_profile_ignored_on_the_length_objective(self, plan):
        out = plan(_flat(), _body(_north_south(), wind_speed=5.0,
                                  objective="length",
                                  wind_profile=self.LEVELS))
        assert out["wind"]["planned"] is False
        assert out["wind"]["source"] is None


class TestForecastEndpoint:
    """Tahmin ucu: ag hatasi plani cope atmamali."""

    def test_success_is_marked_ok(self, monkeypatch):
        monkeypatch.setattr(server, "wind_profile",
                            lambda lat, lon: {"levels": [[10.0, 3.0, 270.0]],
                                              "speed": 3.0,
                                              "from_deg": 270.0})
        got = server.forecast_payload(41.2, 29.3)
        assert got["ok"] is True
        assert got["speed"] == pytest.approx(3.0)

    def test_failure_explains_itself(self, monkeypatch):
        # Internet yokken uygulama calismaya devam etmeli: hata mesaji
        # doner, kullanici ruzgari elle girer.
        def boom(lat, lon):
            raise ForecastUnavailable("ag yok")

        monkeypatch.setattr(server, "wind_profile", boom)
        got = server.forecast_payload(41.2, 29.3)
        assert got["ok"] is False
        assert got["message"]

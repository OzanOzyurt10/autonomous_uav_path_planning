"""Uretilen rotanin ucagin kisitlarini sagladigini bastan sona dogrular.

Diger testler parcalari kontrol ediyor: Dubins kelimeleri, carpisma
kontrolu, kisaltma. Bu dosya NIHAI rotayi aliyor - planlama, kisaltma ve
ornekleme bittikten sonra - ve ucagin fiilen ucabilecegini denetliyor.

shortcut kenarlari yeniden birlestirdigi icin bir gerileme tam orada
saklanir; parca testleri onu gormez.

Toleranslar olculdu, tahmin degil: gercek rotalarda en kucuk donus
yaricapi TAM rho cikiyor ve tirmanma acisi sinirda 7.9996 derecede
duruyor. O yuzden pay dar tutuldu; gevsek pay gerilemeyi gizlerdi.
"""

import array
import math

import pytest

from app.mission import (Constraints, auto_altitude, build_env, build_env_2d,
                         plan_mission, plan_mission_2d, sample_mission)
from src.environment import Obstacle
from src.environment3d import Cylinder

# Olculen degerler tam sinirda durdugu icin pay yalnizca kayan nokta
# gurultusunu karsiliyor.
SLACK = 1e-4


def _turn_radius(a, b, c) -> float:
    """Uc pozun YATAY izdusumunden gecen cemberin yaricapi, metre.

    Dubins airplane'in yatay izdusumu rho yaricapli bir Dubins yolu; kisit
    orada tanimli. Ucgenin cevrel cember yaricabi R = abc / (4 * alan);
    noktalar bir dogru uzerindeyse alan sifir ve yaricap sonsuz - duz
    kesim, kisiti zorlamiyor.
    """
    ab = math.dist(a[:2], b[:2])
    bc = math.dist(b[:2], c[:2])
    ca = math.dist(c[:2], a[:2])
    twice_area = abs((b[0] - a[0]) * (c[1] - a[1])
                     - (b[1] - a[1]) * (c[0] - a[0]))
    if twice_area < 1e-9 or ab * bc * ca == 0.0:
        return math.inf
    return ab * bc * ca / (2 * twice_area)


def _climb_angles(poses):
    """Ardisik pozlar arasindaki ucus yolu acisi, radyan.

    Sifir yatay mesafeli parcalar atlaniyor: bacak sinirlarinda ayni poz
    iki kez geliyor ve orada aci tanimsiz.
    """
    angles = []
    for first, second in zip(poses, poses[1:]):
        flat = math.dist(first[:2], second[:2])
        if flat > 1e-9:
            angles.append(abs(math.atan2(second[2] - first[2], flat)))
    return angles


def _min_radius(poses) -> float:
    return min((_turn_radius(a, b, c)
                for a, b, c in zip(poses, poses[1:], poses[2:])),
               default=math.inf)


def _hilly(cols=120, rows=120, extent=12000.0):
    """Yumusak tepeli sentetik arazi; gercek karo beklemeden calisiyor."""
    from src.terrain import Terrain
    heights = array.array("f", [0.0] * (cols * rows))
    for row in range(rows):
        for col in range(cols):
            heights[row * cols + col] = (400.0
                                         + 250.0 * math.sin(col / 11.0)
                                         + 180.0 * math.cos(row / 14.0))
    return Terrain(heights, cols, rows, extent, extent)


def _flat(height=200.0, cols=60, rows=60, extent=6000.0):
    from src.terrain import Terrain
    return Terrain(array.array("f", [height] * (cols * rows)),
                   cols, rows, extent, extent)


CON = Constraints(speed=28.0, max_bank=math.radians(30.0),
                  max_climb=math.radians(8.0), clearance=100.0,
                  max_iterations=1500, seed=1)


class TestRoute3D:
    """Tepeli arazi ve yasak bolgelerle uretilen nihai 3B rota."""

    @staticmethod
    def _mission():
        terrain = _hilly()
        zones = (Cylinder(6000.0, 6000.0, 900.0, 0.0, 4000.0),)
        env = build_env((0.0, 0.0, 0.0, 12000.0, 12000.0, 2200.0), terrain,
                        CON, zones)
        flat = [(1500.0, 1500.0), (6000.0, 2500.0), (10000.0, 8000.0),
                (2500.0, 10000.0)]
        waypoints = [(x, y, auto_altitude(terrain.elevation_at(x, y),
                                          CON.clearance)) for x, y in flat]
        return plan_mission(waypoints, env, CON), env

    def test_route_is_found(self):
        mission, _ = self._mission()
        assert mission.found

    def test_every_pose_is_free(self):
        # Emniyet payi, arazi, yasak bolge ve harita siniri - hepsi
        # Environment3D.is_free icinde. Tek bir poz bile kacmamali.
        mission, env = self._mission()
        poses = sample_mission(mission, 25.0)
        assert len(poses) > 100
        bad = [p for p in poses if not env.is_free(p)]
        assert bad == []

    def test_turn_radius_never_below_rho(self):
        # Yatis acisi kisitinin geometrik karsiligi. shortcut kenarlari
        # yeniden kurdugu icin bu ozellik nihai rotada dogrulanmali.
        mission, _ = self._mission()
        poses = sample_mission(mission, 25.0)
        assert _min_radius(poses) >= CON.rho * (1.0 - SLACK)

    def test_climb_angle_never_above_limit(self):
        mission, _ = self._mission()
        poses = sample_mission(mission, 25.0)
        assert max(_climb_angles(poses)) <= CON.max_climb * (1.0 + SLACK)

    def test_route_stays_out_of_the_zone(self):
        mission, env = self._mission()
        zone = env.obstacles[0]
        for pose in sample_mission(mission, 25.0):
            gap = math.hypot(pose[0] - zone.x, pose[1] - zone.y)
            inside_band = zone.z_min <= pose[2] <= zone.z_max
            if inside_band:
                assert gap >= zone.radius + env.clearance - 1e-6

    def test_route_clears_the_terrain(self):
        mission, env = self._mission()
        for pose in sample_mission(mission, 25.0):
            ground = env.terrain.elevation_at(pose[0], pose[1])
            assert pose[2] >= ground + env.clearance - 1e-6

    def test_sampling_density_does_not_change_the_verdict(self):
        # Ince ornekleme daha cok firsat demek; kisit orada da tutmali.
        mission, env = self._mission()
        for step in (10.0, 50.0):
            poses = sample_mission(mission, step)
            assert _min_radius(poses) >= CON.rho * (1.0 - SLACK)
            assert max(_climb_angles(poses)) <= CON.max_climb * (1.0 + SLACK)
            assert all(env.is_free(p) for p in poses)


class TestClimbLimitSaturated:
    """Tirmanma sinirinin BAGLAYICI oldugu rota.

    Duz seyirde aci sifira yakin kaliyor ve kisit hic zorlanmiyor; boyle
    bir testte "tirmanma acisi sinirin altinda" bedava gecerdi. Burada
    kisa yatay mesafede buyuk irtifa farki isteniyor, aci sinira oturuyor.
    """

    @staticmethod
    def _mission():
        terrain = _flat()
        env = build_env((0.0, 0.0, 0.0, 6000.0, 6000.0, 3500.0), terrain,
                        CON, ())
        waypoints = [(1000.0, 3000.0, 500.0), (2800.0, 3000.0, 1900.0),
                     (4800.0, 3000.0, 700.0)]
        return plan_mission(waypoints, env, CON), env

    def test_route_is_found(self):
        mission, _ = self._mission()
        assert mission.found

    def test_limit_is_actually_reached(self):
        # Testin anlamli oldugunun kaniti: aci sinira degmiyorsa kisiti
        # dogrulamiyoruz demektir.
        mission, _ = self._mission()
        angles = _climb_angles(sample_mission(mission, 20.0))
        assert max(angles) > CON.max_climb * 0.99

    def test_limit_is_not_exceeded(self):
        mission, _ = self._mission()
        angles = _climb_angles(sample_mission(mission, 20.0))
        assert max(angles) <= CON.max_climb * (1.0 + SLACK)

    def test_turn_radius_holds_through_the_helix(self):
        # Buyuk irtifa farki helis turu getiriyor; yaricap orada da rho.
        mission, _ = self._mission()
        poses = sample_mission(mission, 20.0)
        assert _min_radius(poses) >= CON.rho * (1.0 - SLACK)


CON_2D = Constraints(speed=28.0, max_bank=math.radians(30.0),
                     max_climb=math.radians(8.0), clearance=100.0,
                     max_iterations=1500, seed=1)


class TestRoute2D:
    """2B rota: irtifa sabit, yasak bolgeler sonsuz sutun."""

    # Ikisi de dogrudan yolun uzerinde: rota dolasmak ZORUNDA. Kenarda
    # dursalardi asagidaki emniyet testleri bedava gecerdi.
    ZONES = (Obstacle(7000.0, 9000.0, 1000.0),
             Obstacle(12000.0, 9000.0, 1000.0))
    ALTITUDE = 900.0

    @staticmethod
    def _mission():
        env = build_env_2d((0.0, 0.0, 18000.0, 18000.0), CON_2D,
                           TestRoute2D.ZONES)
        waypoints = [(2000.0, 9000.0), (16000.0, 9000.0)]
        return plan_mission_2d(waypoints, env, CON_2D,
                               TestRoute2D.ALTITUDE), env

    def test_route_is_found(self):
        mission, _ = self._mission()
        assert mission.found

    def test_altitude_is_constant(self):
        mission, _ = self._mission()
        levels = {round(p[2], 6) for p in sample_mission(mission, 40.0)}
        assert levels == {self.ALTITUDE}

    def test_turn_radius_never_below_rho(self):
        mission, _ = self._mission()
        poses = sample_mission(mission, 25.0)
        assert _min_radius(poses) >= CON_2D.rho * (1.0 - SLACK)

    def test_route_keeps_clearance_from_every_zone(self):
        mission, env = self._mission()
        for pose in sample_mission(mission, 25.0):
            for zone in self.ZONES:
                gap = math.hypot(pose[0] - zone.x, pose[1] - zone.y)
                assert gap >= zone.radius + env.clearance - 1e-6

    def test_route_had_to_detour(self):
        # Bolgeler dogrudan yolu kesiyor; kesmeseydi yukaridaki testler
        # bedava gecerdi.
        mission, _ = self._mission()
        assert mission.cost > math.dist((2000.0, 9000.0), (16000.0, 9000.0))


class TestHelpers:
    """Olcut fonksiyonlarinin kendisi dogru mu."""

    def test_radius_of_a_known_circle(self):
        r = 500.0
        pts = [(r * math.cos(a), r * math.sin(a), 0.0, 0.0)
               for a in (0.0, 0.4, 0.8)]
        assert _turn_radius(*pts) == pytest.approx(r, rel=1e-6)

    def test_collinear_points_have_infinite_radius(self):
        pts = [(0.0, 0.0, 0.0, 0.0), (100.0, 0.0, 0.0, 0.0),
               (250.0, 0.0, 0.0, 0.0)]
        assert _turn_radius(*pts) == math.inf

    def test_climb_angle_of_a_known_slope(self):
        poses = [(0.0, 0.0, 0.0, 0.0), (1000.0, 0.0, 1000.0, 0.0)]
        assert _climb_angles(poses)[0] == pytest.approx(math.pi / 4)

    def test_repeated_poses_are_skipped(self):
        poses = [(0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 5.0, 0.0),
                 (100.0, 0.0, 0.0, 0.0)]
        assert len(_climb_angles(poses)) == 1

"""Cok bacakli gorev planlama: waypoint listesinden ucus rotasi.

Sunucunun test edilebilir cekirdegi. Iki isi var: kullanicinin verdigi
waypoint dizisini plan3d'nin anladigi poz ciftlerine cevirmek, ve bacaklari
sirayla planlayip tek bir rota olarak birlestirmek.

Kisitlar ucak kartindan geliyor (hiz, yatis, tirmanma); donus yaricapi
bunlardan turetiliyor, kullaniciya soyut bir rho sorulmuyor.
"""

import math
from dataclasses import dataclass, field

from src.dubins import DubinsPath
from src.dubins3d import DubinsPath3D
from src.environment import Environment, Obstacle
from src.environment3d import Cylinder, Environment3D
from src.geo import METRES_PER_DEGREE, Frame
from src.rrt_star import plan as plan2d
from src.rrt_star import shortcut as shortcut2d
from src.rrt_star3d import plan3d, shortcut
from src.terrain import Terrain
import random

GRAVITY = 9.80665

Point3 = tuple[float, float, float]
Pose3 = tuple[float, float, float, float]


@dataclass(frozen=True)
class Constraints:
    """Ucak kartindan doldurulan kisitlar; rho bunlardan turetiliyor."""

    speed: float                  # seyir hizi, m/s
    max_bank: float               # azami yatis acisi, radyan
    max_climb: float              # azami tirmanma/alcalma acisi, radyan
    clearance: float              # araziden ve engellerden emniyet payi, m
    max_iterations: int = 4000
    seed: int | None = None

    def __post_init__(self):
        if self.speed <= 0.0:
            raise ValueError(f"hiz pozitif olmali: {self.speed}")
        if not 0.0 < self.max_bank < math.pi / 2:
            raise ValueError(
                f"yatis acisi (0, 90) derece araliginda olmali: {self.max_bank}")
        if not 0.0 < self.max_climb < math.pi / 2:
            raise ValueError(
                f"tirmanma acisi (0, 90) derece araliginda olmali: "
                f"{self.max_climb}")
        if self.clearance < 0.0:
            raise ValueError(f"emniyet payi negatif olamaz: {self.clearance}")
        if self.max_iterations <= 0:
            raise ValueError(
                f"max_iterations pozitif olmali: {self.max_iterations}")

    @property
    def rho(self) -> float:
        """Duz donuste minimum yaricap: rho = v^2 / (g * tan(yatis)).

        Yatis acisi buyudukce yaricap kuculuyor; kullanici hiz ve yatis
        girip yaricabi kendiliginden aliyor.
        """
        return self.speed ** 2 / (GRAVITY * math.tan(self.max_bank))


@dataclass(frozen=True)
class Zone:
    """Dairesel yasak bolge; 2B'de Obstacle, 3B'de Cylinder oluyor.

    2B'de irtifa yok sayiliyor: bolge sonsuz bir sutun gibi davraniyor.
    Arazi olmayan bolgede planlayiciyi zorlayan tek sey bunlar.
    """

    x: float
    y: float
    radius: float
    z_min: float = 0.0
    z_max: float = 20000.0

    def as_obstacle(self) -> Obstacle:
        return Obstacle(self.x, self.y, self.radius)

    def as_cylinder(self) -> Cylinder:
        return Cylinder(self.x, self.y, self.radius, self.z_min, self.z_max)


@dataclass(frozen=True)
class FlatEdge:
    """2B Dubins kenarini sabit irtifada 3B poz ureten kenara cevirir.

    2B planlama sonucu boylece degismeden Mission'a giriyor: ornekleme,
    animasyon, harita ve 3B gorunum tek bir kenar arayuzu goruyor.
    """

    path: DubinsPath
    altitude: float

    @property
    def start(self) -> Pose3:
        x, y, yaw = self.path.start
        return (x, y, self.altitude, yaw)

    @property
    def length(self) -> float:
        return self.path.length

    def end_pose(self) -> Pose3:
        x, y, yaw = self.path.end_pose()
        return (x, y, self.altitude, yaw)

    def sample(self, step: float) -> list[Pose3]:
        return [(x, y, self.altitude, yaw)
                for x, y, yaw in self.path.sample(step)]


@dataclass(frozen=True)
class Leg:
    """Iki waypoint arasindaki bacak."""

    start: Pose3
    goal: Pose3
    edges: list                   # DubinsPath3D ya da FlatEdge
    found: bool
    cost: float


@dataclass(frozen=True)
class Mission:
    legs: list[Leg]
    found: bool                   # butun bacaklar bulundu mu
    cost: float                   # metre; bir bacak bile bulunamadiysa inf
    duration: float               # saniye, sabit hizda


def _bearing(origin: Point3, target: Point3) -> float:
    """Yatay duzlemde origin'den target'a kerteriz."""
    return math.atan2(target[1] - origin[1], target[0] - origin[0])


def waypoint_headings(waypoints) -> list[float]:
    """Her waypoint icin ucagin bakacagi yon.

    Kullanici haritaya tiklarken yon girmiyor, ama plan3d her iki uca da
    yaw istiyor. Ara noktalarda gelis ve gidis kerterizinin aciortasi
    aliniyor; boylece bacaklar arasinda bas acisi surekli kaliyor ve rota
    duraklarda kirilmiyor.
    """
    if len(waypoints) < 2:
        raise ValueError(f"en az iki waypoint gerekli: {len(waypoints)}")

    headings = []
    for index, point in enumerate(waypoints):
        if index == 0:
            headings.append(_bearing(point, waypoints[1]))
            continue
        if index == len(waypoints) - 1:
            headings.append(_bearing(waypoints[-2], point))
            continue

        arriving = _bearing(waypoints[index - 1], point)
        leaving = _bearing(point, waypoints[index + 1])
        # Aciortay birim vektorlerin toplamiyla aliniyor; ortalama almak
        # 350 ile 10 derecede yanlis tarafi verirdi.
        sin_sum = math.sin(arriving) + math.sin(leaving)
        cos_sum = math.cos(arriving) + math.cos(leaving)
        if math.hypot(sin_sum, cos_sum) < 1e-9:
            headings.append(arriving)     # tam geri donus; aciortay tanimsiz
        else:
            headings.append(math.atan2(sin_sum, cos_sum))
    return headings


def geo_window(points, margin_m: float, max_side: float,
               min_side: float = 4000.0):
    """Enlem/boylam noktalarini saracak pencereyi (lat, lon, en, boy) verir.

    Pencere artik elle secilmiyor: kullanici haritaya waypoint koyuyor,
    planlama alani onlarin sinir kutusu arti pay oluyor. Pay planlayiciya
    engelin etrafindan dolasacak yer birakmak icin.
    """
    if len(points) < 2:
        raise ValueError(f"en az iki nokta gerekli: {len(points)}")

    lats = [lat for lat, _ in points]
    lons = [lon for _, lon in points]
    if max(lons) - min(lons) > 180.0:
        raise ValueError(
            "gorev 180. boylami kesiyor; duzlem izdusumu orada anlamsiz")

    height = max((max(lats) - min(lats)) * METRES_PER_DEGREE + 2 * margin_m,
                 min_side)
    lat = (max(lats) + min(lats)) / 2 - height / 2 / METRES_PER_DEGREE
    # Olcek pencerenin ortasinda; for_window mid_lat'i tam oraya koyuyor.
    frame = Frame.for_window(lat, min(lons), height)
    width = max((max(lons) - min(lons)) * frame.lon_scale + 2 * margin_m,
                min_side)
    # Kirpmak waypointleri pencerenin disinda birakirdi; sessizce yanlis
    # planlamaktansa kullaniciya alani kucultmesini soylemek dogru.
    if width > max_side or height > max_side:
        raise ValueError(
            f"gorev alani cok buyuk: {width / 1000:.1f} x "
            f"{height / 1000:.1f} km, sinir {max_side / 1000:.0f} km. "
            f"Waypointleri birbirine yaklastir.")

    lon = (max(lons) + min(lons)) / 2 - width / 2 / frame.lon_scale
    return lat, lon, width, height


def build_env(bounds, terrain: Terrain | None, constraints: Constraints,
              obstacles: tuple[Cylinder, ...] = ()) -> Environment3D:
    """Kisitlardaki emniyet payiyla 3B ortam kurar."""
    return Environment3D(bounds, obstacles, constraints.clearance, terrain)


def build_env_2d(bounds, constraints: Constraints,
                 obstacles: tuple[Obstacle, ...] = ()) -> Environment:
    """2B ortam: arazi YOK, yalnizca engeller ve harita siniri.

    bounds (xmin, ymin, xmax, ymax). Arazi istemedigi icin dunyanin her
    yerinde kurulabiliyor - kapsama disinda calisabilmenin sarti bu.
    """
    return Environment(bounds, obstacles, constraints.clearance)


def plan_mission_2d(waypoints, env: Environment, constraints: Constraints,
                    altitude: float) -> Mission:
    """Yatay duzlemde bacak bacak planlar; irtifa sabit seyir irtifasi.

    Arazi hesaba KATILMIYOR - bu 2B planlama. Arazi verisi varsa rotanin
    AGL profili ayrica cikarilip kullaniciya gosteriliyor, ama planlayici
    ondan kacinmiyor; kacinma icin 3B mod var.
    """
    flat = [(point[0], point[1]) for point in waypoints]
    headings = waypoint_headings(flat)
    poses = [(point[0], point[1], heading)
             for point, heading in zip(flat, headings)]

    step = env.suggested_step()
    legs = []
    for start, goal in zip(poses, poses[1:]):
        result = plan2d(start, goal, env, constraints.rho,
                        max_iterations=constraints.max_iterations,
                        goal_bias=0.10,
                        max_edge_length=10 * constraints.rho,
                        rng=random.Random(constraints.seed))
        edges = [FlatEdge(path, altitude)
                 for path in shortcut2d(result.edges, env, constraints.rho,
                                        step)] if result.found else []
        cost = sum(edge.length for edge in edges) if result.found else math.inf
        legs.append(Leg((start[0], start[1], altitude, start[2]),
                        (goal[0], goal[1], altitude, goal[2]),
                        edges, result.found, cost))

    found = all(leg.found for leg in legs)
    cost = sum(leg.cost for leg in legs) if found else math.inf
    duration = cost / constraints.speed if found else math.inf
    return Mission(legs, found, cost, duration)


def plan_mission(waypoints, env: Environment3D,
                 constraints: Constraints) -> Mission:
    """Waypoint dizisini bacak bacak planlar ve tek rotaya birlestirir.

    env, constraints.clearance ile kurulmus olmali - build_env bunu yapiyor.
    Bir bacak bulunamazsa gorev found=False doner ama diger bacaklar yine
    hesaplanir; arayuzde hangi bacagin takildigini gostermek icin.
    """
    headings = waypoint_headings(waypoints)
    poses = [(point[0], point[1], point[2], heading)
             for point, heading in zip(waypoints, headings)]

    step = env.suggested_step()
    legs = []
    for start, goal in zip(poses, poses[1:]):
        result = plan3d(start, goal, env, constraints.rho,
                        constraints.max_climb,
                        max_iterations=constraints.max_iterations,
                        goal_bias=0.10,
                        max_edge_length=10 * constraints.rho,
                        refine_steps=8,
                        rng=random.Random(constraints.seed))
        edges = shortcut(result.edges, env, constraints.rho,
                         constraints.max_climb, step, 8) if result.found else []
        cost = sum(edge.length for edge in edges) if result.found else math.inf
        legs.append(Leg(start, goal, edges, result.found, cost))

    found = all(leg.found for leg in legs)
    cost = sum(leg.cost for leg in legs) if found else math.inf
    duration = cost / constraints.speed if found else math.inf
    return Mission(legs, found, cost, duration)


def sample_leg(leg: Leg, step: float) -> list[Pose3]:
    """Tek bacagi step metre araliklarla pozlara cevirir."""
    if step <= 0.0:
        raise ValueError(f"step pozitif olmali: {step}")

    poses = []
    for edge in leg.edges:
        for pose in edge.sample(step):
            if poses and _same_pose(poses[-1], pose):
                continue              # kenar sinirlari ust uste biniyor
            poses.append(pose)
    return poses


def sample_mission(mission: Mission, step: float) -> list[Pose3]:
    """Gorevin tamamini step metre araliklarla pozlara cevirir.

    Hiz sabit oldugu icin esit mesafe esit sure demek; tarayici bu diziyi
    dogrudan animasyon karesi olarak kullanabiliyor.
    """
    poses = []
    for leg in mission.legs:
        for pose in sample_leg(leg, step):
            if poses and _same_pose(poses[-1], pose):
                continue              # bacak sinirlari da ust uste biniyor
            poses.append(pose)
    return poses


def _same_pose(first: Pose3, second: Pose3, tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(first[:3], second[:3]))


def agl_profile(poses, terrain: Terrain | None):
    """Her poz icin yerden yukseklik; arazi yoksa bos liste."""
    if terrain is None:
        return []
    return [pose[2] - terrain.elevation_at(pose[0], pose[1]) for pose in poses]

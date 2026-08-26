"""Cok bacakli gorev planlama: waypoint listesinden ucus rotasi.

Sunucunun test edilebilir cekirdegi. Iki isi var: kullanicinin verdigi
waypoint dizisini plan3d'nin anladigi poz ciftlerine cevirmek, ve bacaklari
sirayla planlayip tek bir rota olarak birlestirmek.

Kisitlar ucak kartindan geliyor (hiz, yatis, tirmanma); donus yaricapi
bunlardan turetiliyor, kullaniciya soyut bir rho sorulmuyor.
"""

import math
from dataclasses import dataclass, field

from src.dubins3d import DubinsPath3D
from src.environment3d import Cylinder, Environment3D
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
class Leg:
    """Iki waypoint arasindaki bacak."""

    start: Pose3
    goal: Pose3
    edges: list[DubinsPath3D]
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


def build_env(bounds, terrain: Terrain | None, constraints: Constraints,
              obstacles: tuple[Cylinder, ...] = ()) -> Environment3D:
    """Kisitlardaki emniyet payiyla ortam kurar."""
    return Environment3D(bounds, obstacles, constraints.clearance, terrain)


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


def sample_mission(mission: Mission, step: float) -> list[Pose3]:
    """Gorevi step metre araliklarla pozlara cevirir.

    Hiz sabit oldugu icin esit mesafe esit sure demek; tarayici bu diziyi
    dogrudan animasyon karesi olarak kullanabiliyor.
    """
    if step <= 0.0:
        raise ValueError(f"step pozitif olmali: {step}")

    poses = []
    for leg in mission.legs:
        for edge in leg.edges:
            for pose in edge.sample(step):
                if poses and _same_pose(poses[-1], pose):
                    continue          # kenar sinirlari ust uste biniyor
                poses.append(pose)
    return poses


def _same_pose(first: Pose3, second: Pose3, tol: float = 1e-6) -> bool:
    return all(abs(a - b) <= tol for a, b in zip(first[:3], second[:3]))


def agl_profile(poses, terrain: Terrain | None):
    """Her poz icin yerden yukseklik; arazi yoksa bos liste."""
    if terrain is None:
        return []
    return [pose[2] - terrain.elevation_at(pose[0], pose[1]) for pose in poses]

"""3B ortam: silindir engeller ve irtifa sinirlari.

Nokta (x, y, z), poz (x, y, z, yaw); ikisi de kabul ediliyor
"""

import math
import random
from collections.abc import Iterable
from dataclasses import dataclass

Point3 = tuple[float, float, float]
Pose3 = tuple[float, float, float, float]
Bounds3 = tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class Cylinder:
    """Sonlu yukseklikte dairesel engel: dikey bir silindir.
    """

    x: float
    y: float
    radius: float
    z_min: float
    z_max: float
    
    def __post_init__(self):
        if self.radius <= 0:
            raise ValueError(f"yaricap pozitif olmali, verilen: {self.radius}")
        if self.z_max <= self.z_min:
            raise ValueError(f"z_max, z_min'den buyuk olmali: "
                             f"{self.z_max} <= {self.z_min}")

    def contains(self, point: Point3 | Pose3, clearance: float = 0.0) -> bool:
        """Nokta silindirin (veya emniyet payinin) icinde mi.

        Iki kosul birden gerekir: yatayda yaricap icinde ve irtifada
        [z_min, z_max] araliginda. Biri saglanmiyorsa nokta serbesttir.
        """
        horizontal_dist = math.hypot(point[0] - self.x, point[1] - self.y)
        return (horizontal_dist <= self.radius + clearance and self.z_min - clearance <= point[2] <= self.z_max + clearance)

@dataclass(frozen=True)
class Environment3D:
    """Dikdortgen prizma sinirlar, silindir engeller, ortak emniyet payi."""

    bounds: Bounds3
    obstacles: tuple[Cylinder, ...]
    clearance: float

    def is_inside_bounds(self, point: Point3 | Pose3) -> bool:
        """Nokta harita hacminin icinde mi; tavan ve taban dahil."""
        x_min, y_min, z_min, x_max, y_max, z_max = self.bounds
        return (x_min <= point[0] <= x_max
                and y_min <= point[1] <= y_max
                and z_min <= point[2] <= z_max)

    def is_free(self, point: Point3 | Pose3) -> bool:
        """Sinir icinde ve hicbir engelin emniyet payinda degil mi."""
        if not self.is_inside_bounds(point):
            return False
        return not any(obstacle.contains(point, self.clearance)
                       for obstacle in self.obstacles)

    def is_path_free(self, points: Iterable[Point3 | Pose3]) -> bool:
        """Nokta listesinin tamami serbest mi. Bos liste serbest sayilir."""
        return all(self.is_free(point) for point in points)

    def suggested_step(self) -> float:
        """Carpisma kontrolu icin onerilen ornekleme araligi, metre.

        Her silindirin sisirilmis en kucuk yari-boyutu min(yaricap,
        yukseklik/2) + clearance; bunlarin en kucugunun yarisi adim olur.
        Yukseklik de sayiliyor cunku alcak ve genis bir engel yalnizca
        yaricapa bakan bir adimla dikeyde atlanabilir.

        Sezgisel bir kural; ayriklastirma hatasini tamamen kapatmiyor,
        emniyet payi onu yutuyor.
        """
        x_min, y_min, z_min, x_max, y_max, z_max = self.bounds
        if not self.obstacles:
            return min(x_max - x_min, y_max - y_min, z_max - z_min) / 10

        half_sizes = [min(obstacle.radius,(obstacle.z_max - obstacle.z_min) / 2) + self.clearance for obstacle in self.obstacles]
        return min(half_sizes) / 2

    def random_free_pose(self, rng: random.Random,
                         max_attempts: int = 1000) -> Pose3:
        """Reddetme ornekleme ile serbest bir poz uretir.

        Bulunamamasi gecersiz girdi degil, cok dolu bir harita demek.
        """
        x_min, y_min, z_min, x_max, y_max, z_max = self.bounds
        for _ in range(max_attempts):
            pose = (rng.uniform(x_min, x_max),
                    rng.uniform(y_min, y_max),
                    rng.uniform(z_min, z_max),
                    rng.uniform(0, 2 * math.pi))
            if self.is_free(pose):
                return pose

        raise RuntimeError(
            f"{max_attempts} denemede serbest poz bulunamadi; "
            f"engeller cok buyuk veya clearance cok yuksek olabilir")

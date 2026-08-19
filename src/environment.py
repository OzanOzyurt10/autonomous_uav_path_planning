"""Planlama ortami: dairesel engeller, harita sinirlari ve emniyet payi.

"Bu nokta serbest mi", "bu yol serbest mi" sorularina cevap verir ve RRT*
icin serbest uzaydan rastgele poz uretir.

Koordinat sozlesmesi (dubins.py ile ayni):
    - ENU cercevesi: x dogu, y kuzey, metre
    - Aci gerekmedigi icin burada yalniz (x, y) ile ilgilenilir; uc elemanli
      bir poz verilirse ilk iki bileseni kullanilir

"""

import math
import random
from collections.abc import Iterable
from dataclasses import dataclass

Pose = tuple[float, float, float]   # (x, y, yaw)
Point = tuple[float, float]         # (x, y)


@dataclass(frozen=True)
class Obstacle:
    """Dairesel engel.

    Alanlar:
        x: Merkezin dogu koordinati, metre.
        y: Merkezin kuzey koordinati, metre.
        radius: Yaricap, metre. Pozitif olmali.
    """

    x: float
    y: float
    radius: float

    def __post_init__(self):
        if self.radius <= 0:
            raise ValueError(f"yaricap pozitif olmali, verilen: {self.radius}")

    def contains(self, point: Point | Pose, clearance: float = 0.0) -> bool:
        """Nokta bu engelin (sisirilmis) icinde mi.

        clearance emniyet payi: yaricaba eklenir. Verilmezse sifir.
        Sinir uzeri carpisma sayilir (<=, < degil).

        point uc elemanli da olabilir; yalnizca ilk iki bileseni okunur.
        """
        distance = math.hypot(point[0] - self.x, point[1] - self.y)
        return distance <= self.radius + clearance


@dataclass(frozen=True)
class Environment:
    """Planlama ortami: harita sinirlari, engeller ve emniyet payi.

    Alanlar:
        bounds: (xmin, ymin, xmax, ymax), metre. Planlamanin yapilacagi
            dikdortgen alan. RRT* rastgele pozu buradan ureteccek.
        obstacles: Dairesel engeller. Bos olabilir.
        clearance: Her engel yaricapina eklenen emniyet payi, metre. GPS
            hatasi, ruzgar suruklemesi ve kanat acikligi icin. Negatif olamaz.
    """

    bounds: tuple[float, float, float, float]
    obstacles: tuple[Obstacle, ...]
    clearance: float

    def __post_init__(self):
        if self.clearance < 0:
            raise ValueError(
                f"clearance negatif olamaz, verilen: {self.clearance}")

        x_min, y_min, x_max, y_max = self.bounds
        if x_min >= x_max or y_min >= y_max:
            raise ValueError(
                f"bounds (xmin, ymin, xmax, ymax) siralamasi bozuk: {self.bounds}")

    def is_inside_bounds(self, point: Point | Pose) -> bool:
        """Nokta harita dikdortgeninin icinde mi."""
        x_min, y_min, x_max, y_max = self.bounds
        return x_min <= point[0] <= x_max and y_min <= point[1] <= y_max

    def is_free(self, point: Point | Pose) -> bool:
        """Nokta serbest mi: harita icinde ve hicbir engele girmiyor.

        Engeller clearance ile sisirilmis halleriyle degerlendirilir.

        point uc elemanli da olabilir; yalnizca ilk iki bileseni okunur.
        """
        if not self.is_inside_bounds(point):
            return False

        for obstacle in self.obstacles:
            if obstacle.contains(point, self.clearance):
                return False

        return True

    def is_path_free(self, points: Iterable[Point | Pose]) -> bool:
        """Verilen noktalarin hepsi serbest mi. Bos liste icin True."""
        for point in points:
            if not self.is_free(point):
                return False
        return True

    def suggested_step(self) -> float:
        """Ornekleme icin onerilen adim boyu, metre.

        Engel varsa en kucuk sisirilmis yaricapin yarisi, yoksa haritanin
        kisa kenarinin onda biri."""
        x_min, y_min, x_max, y_max = self.bounds
        if not self.obstacles:
            step = min(x_max - x_min, y_max - y_min)
            return step / 10

        min_obs = None
        for obstacle in self.obstacles:
            if min_obs is None or min_obs.radius > obstacle.radius:
                min_obs = obstacle

        return (min_obs.radius + self.clearance) / 2

    def random_free_pose(self, rng: random.Random,max_attempts: int = 1000) -> Pose:
        x_min, y_min, x_max, y_max = self.bounds
        for _ in range(max_attempts):
            x = rng.uniform(x_min, x_max)
            y = rng.uniform(y_min, y_max)
            yaw = rng.uniform(0, 2 * math.pi)
            pose = (x, y, yaw)
            if self.is_free(pose):
                return pose

        raise RuntimeError(
            f"{max_attempts} denemede serbest poz bulunamadi; "
            f"engeller cok buyuk veya clearance cok yuksek olabilir")

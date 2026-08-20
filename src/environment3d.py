"""3B ortam: silindir engeller ve irtifa sinirlari.

dubins3d.py'yi import etmiyor. 2B'deki ayrimin aynisi: ortam ucak dinamigini
bilmiyor, yalnizca "su nokta serbest mi" sorusunu cevapliyor.

Nokta (x, y, z), poz (x, y, z, yaw); ikisi de kabul ediliyor, yaw yok sayilir.
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

    Yukseklik sonlu oldugu icin tepesinin ustunden ucmak serbest - 3B
    planlamanin kazandirdigi sey bu. Emniyet payi dikeyde de uygulanir.
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
        return (horizontal_dist <= self.radius + clearance
                and self.z_min - clearance <= point[2] <= self.z_max + clearance)

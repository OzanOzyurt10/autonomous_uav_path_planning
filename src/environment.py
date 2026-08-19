"""Planlama ortami: dairesel engeller, harita sinirlari ve emniyet payi.

"Bu nokta serbest mi", "bu yol serbest mi" sorularina cevap verir ve RRT*
icin serbest uzaydan rastgele poz uretir.

Koordinat sozlesmesi (dubins.py ile ayni):
    - ENU cercevesi: x dogu, y kuzey, metre
    - Aci gerekmedigi icin burada yalniz (x, y) ile ilgilenilir; uc elemanli
      bir poz verilirse ilk iki bileseni kullanilir

NEDEN dubins.py IMPORT EDILMIYOR:
    Bu modul bilerek Dubins'i tanimaz. Yol carpisma kontrolu bir DubinsPath
    degil, orneklenmis nokta listesi alir:

        env.is_path_free(path.sample(step))

    Boylece (1) iki modul birbirinden bagimsiz test edilebilir, (2) adim boyu
    secimi cagiranin sorumlulugunda kalir, (3) ileride Dubins yerine baska bir
    yol tipi denenirse bu dosya degismez. Pose takma adi bu yuzden burada
    yeniden tanimlanir; kolaylik olsun diye dubins'ten import etmek bu
    bagimsizligi bozar.
"""

import math
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

        clearance emniyet payi: yaricaba eklenir, yani engeli sisirir.
        Verilmezse sifir kullanilir.

        Sinir uzeri carpisma sayilir (<=, < degil) — muhafazakar taraf
        dogru taraftir.

        point uc elemanli da olabilir; yalnizca ilk iki bileseni okunur.
        """
        distance = math.hypot(point[0] - self.x, point[1] - self.y)
        return distance <= self.radius + clearance

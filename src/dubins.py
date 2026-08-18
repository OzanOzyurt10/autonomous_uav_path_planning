"""Fixed-wing IHA icin 2D Dubins yolu hesabi.

Dubins yolu, minimum donus yaricapi rho ile kisitlanmis bir aracin iki poz
arasindaki en kisa yoludur ve daima uc parcadan olusur: her parca ya rho
yaricapli bir yay (sola "L" / saga "R") ya da duz bir cizgidir ("S").

Koordinat sozlesmesi:
    Pose = (x, y, yaw)
    - ENU cercevesi: x dogu, y kuzey
    - yaw radyan, CCW pozitif, x ekseninden olculur
    - Havacilik heading'i (kuzeyden CW) bu modulun disinda donusturulur

Referans:
    Shkel, A. M., & Lumelsky, V. (2001). Classification of the Dubins set.
    Robotics and Autonomous Systems, 34(4), 179-202.
"""

import math
from dataclasses import dataclass

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class DubinsPath:
    """Cozulmus bir Dubins yolu.

    Alanlar:
        start: Yolun basladigi poz.
        word: Segment tipleri, uc harf. "LSL", "RSR", "LSR", "RSL",
            "RLR" veya "LRL".
        lengths: Uc segmentin yay uzunlugu, **metre**. i'inci eleman
            word'un i'inci harfine karsilik gelir. Kanonik cozuculer
            normalize deger dondurur; buraya konmadan once rho ile
            carpilmalidir.
        rho: Donus yaricapi, metre.

    Nesne degistirilemez (frozen): RRT* agacinda binlerce yol saklanacak
    ve birinin sessizce degismesi maliyet muhasebesini bozar.
    """

    start: Pose
    word: str
    lengths: tuple[float, float, float]  # Segmentlerin uzunluğu (metre)
    rho: float

    @property
    def length(self) -> float:
        """Yolun toplam uzunlugu (metre)."""
        return self.lengths[0] + self.lengths[1] + self.lengths[2]

    def interpolate(self, s: float) -> Pose:
        """Yolun basindan s metre ilerideki pozu dondurur.

        s aralik disindaysa kirpilir: negatif degerler baslangic pozunu,
        length'ten buyuk degerler son pozu verir.
        """
        s = max(0.0, min(s, self.length))
        pose = self.start
        remaining = s
        for mode, seg_len in zip(self.word, self.lengths):
            walk = min(seg_len, remaining)
            pose = _segment_end(pose, mode, walk, self.rho)
            remaining -= walk
        return pose

    def end_pose(self) -> Pose:
        """Yolun bittigi poz."""
        return self.interpolate(self.length)

    def sample(self, step: float) -> list[Pose]:
        """Yolu step araliklarla orneklenmis poz listesine cevirir.

        Ilk eleman daima start, son eleman daima son pozdur; ardisik
        noktalar arasindaki mesafe step'i asmaz. Sifir uzunluklu yol icin
        tek elemanli liste doner.

        Raises:
            ValueError: step pozitif degilse.
        """
        if step <= 0:
            raise ValueError(f"step pozitif olmali, verilen: {step}")

        total = self.length
        if total == 0.0:
            return [self.start]

        n = math.ceil(total / step)
        return [self.interpolate(i / n * total) for i in range(n + 1)]


def _mod2pi(theta: float) -> float:
    """Aciyi [0, 2*pi) araligina indirger.

    Python'un % operatoru bolenin isaretini aldigi icin negatif girdide de
    pozitif sonuc doner; math.fmod bunu yapmaz.
    """
    theta = theta % (2 * math.pi)
    return theta


def _segment_end(pose: Pose, mode: str, s: float, rho: float) -> Pose:
    """pose'dan baslayip mode tipinde s metre ilerledikten sonraki poz.

    mode: "S" duz, "L" sola yay, "R" saga yay. rho donus yaricapi.
    Donen yaw normalize edilmez; interpolate bunu zincirleme cagiriyor.

    Raises:
        ValueError: mode taninmiyorsa.
    """
    x, y, psi = pose
    if mode == "S":
        psi2 = psi
        x2 = x + s * math.cos(psi)
        y2 = y + s * math.sin(psi)

    elif mode == "L":
        psi2 = psi + s / rho
        x2 = x + rho * (math.sin(psi2) - math.sin(psi))
        y2 = y - rho * (math.cos(psi2) - math.cos(psi))

    elif mode == "R":
        psi2 = psi - s / rho
        x2 = x + rho * (math.sin(psi) - math.sin(psi2))
        y2 = y + rho * (math.cos(psi2) - math.cos(psi))

    else:
        raise ValueError(f"bilinmeyen segment tipi: {mode}")

    return (x2, y2, psi2)

"""Dubins airplane: 2B Dubins yolunun irtifa boyutuyla genisletilmesi.

Yatay bilesen olarak mevcut dubins.py kullaniliyor, o dosya degistirilmiyor.
Yol irtifayi kazanmaya yetmiyorsa baslangicta tam turlar (helis) eklenir.

Poz duzeni: (x, y, z, yaw); ENU, z yukari pozitif, yaw radyan.
"""

import math
from dataclasses import dataclass

from src.dubins import DubinsPath, shortest_path

Pose3 = tuple[float, float, float, float]


@dataclass(frozen=True)
class DubinsPath3D:
    """3B yol: yatay bir Dubins yolu ve uzerine irtifa katmani.

    horizontal mevcut 2B yol, z bilmiyor. helix_turns baslangicta atilan tam
    tur sayisi, gamma ise yatayla yapilan ucus yolu acisi (radyan).
    """

    start: Pose3
    horizontal: DubinsPath
    helix_turns: int
    gamma: float

    @property
    def horizontal_length(self) -> float:
        """Yatayda katedilen toplam mesafe: 2B yol + helis turlari."""
        turn = 2 * math.pi * self.horizontal.rho
        return self.horizontal.length + self.helix_turns * turn

    @property
    def length(self) -> float:
        """3B uzunluk; yatay mesafenin gamma acisiyla hipotenusu."""
        return self.horizontal_length / math.cos(self.gamma)

    def interpolate(self, s: float) -> Pose3:
        """Yolun basindan s metre ilerideki 3B pozu doner.

        s aralik disindaysa kirpilir. Yatayda alinan mesafe s*cos(gamma);
        irtifa buna bagli olarak s*sin(gamma) kadar degisir.
        """
        s = max(0.0, min(s, self.length))
        horizontal = s * math.cos(self.gamma)
        z = self.start[2] + s * math.sin(self.gamma)
        helix_length = self.helix_turns * 2 * math.pi * self.horizontal.rho

        if horizontal < helix_length:
            helix = _helix(self.horizontal.start, self.horizontal.word[0],
                           helix_length, self.horizontal.rho)
            x, y, yaw = helix.interpolate(horizontal)
        else:
            x, y, yaw = self.horizontal.interpolate(horizontal - helix_length)
        return (x, y, z, yaw)

    def end_pose(self) -> Pose3:
        """Yolun bitis pozu."""
        return self.interpolate(self.length)


def _helix(start2: tuple[float, float, float], direction: str,
           length: float, rho: float) -> DubinsPath:
    """Verilen yonde length metre donen bir yol; bitis pozunu degistirmez.

    Tam turlar (2*pi*rho katlari) baslangic pozuna birebir doner, yani yatay
    uzunluk eklerken hedefi kaydirmaz. Butun donus ilk parcada, digerleri sifir.
    """
    word = direction + "S" + direction
    lengths = (length, 0.0, 0.0)
    return DubinsPath(start2, word, lengths, rho)

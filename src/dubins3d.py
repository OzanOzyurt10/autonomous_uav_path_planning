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

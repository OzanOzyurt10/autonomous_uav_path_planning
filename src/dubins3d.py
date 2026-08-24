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

    def sample(self, step: float) -> list[Pose3]:
        """Yolu step araliklarla orneklenmis 3B poz listesine cevirir.

        Ilk eleman daima start, son eleman daima bitis pozu; ardisik noktalar
        arasindaki 3B mesafe step'i asmaz.
        """
        if step <= 0:
            raise ValueError(f"step pozitif olmali, verilen: {step}")

        total = self.length
        if total == 0.0:
            return [self.start]

        n = math.ceil(total / step)
        return [self.interpolate(i / n * total) for i in range(n + 1)]


def _helix(start2: tuple[float, float, float], direction: str,
           length: float, rho: float) -> DubinsPath:
    """Verilen yonde length metre donen bir yol; bitis pozunu degistirmez.

    Tam turlar (2*pi*rho katlari) baslangic pozuna birebir doner, yani yatay
    uzunluk eklerken hedefi kaydirmaz. Butun donus ilk parcada, digerleri sifir.
    """
    word = direction + "S" + direction
    lengths = (length, 0.0, 0.0)
    return DubinsPath(start2, word, lengths, rho)

def _widened_horizontal(start2: tuple[float, float, float],
                        goal2: tuple[float, float, float],
                        rho: float, required: float,
                        steps: int) -> DubinsPath | None:
    if steps <= 0 or required <=0 :
        return None
    hi = rho
    
    for _ in range (7) : 
        if shortest_path(start2,goal2,hi).length < required:
            hi *=2
        else:
            break

    
    if hi == rho * (128):
        return None
    lo = rho
    for _ in range (steps) :
        if shortest_path(start2,goal2,(hi+lo) / 2).length >= required:
            hi = (hi+lo) / 2
        else:
            lo = (hi+lo) / 2  
        

    return shortest_path(start2,goal2,hi)


def airplane_path(start: Pose3, goal: Pose3, rho: float,
                  gamma_max: float,refine_steps: int = 0) -> DubinsPath3D:
    """start'tan goal'a tirmanma acisi sinirina uyan bir 3B yol kurar.

    Yatay yol irtifayi kazanmaya yetmiyorsa baslangicta tam turlar eklenir.
    O durumda yol optimal degildir: ideal uzunlugun uzerine en fazla bir tur
    (2*pi*rho) binebilir. Tam tur yerine kismi uzatma literaturdeki "orta
    irtifa" cozumu, kapsam disi.
    """
    if rho <= 0:
        raise ValueError(f"rho pozitif olmali: {rho}")
    if gamma_max <= 0 or gamma_max >= math.pi / 2:
        raise ValueError(f"gamma_max (0, pi/2) araliginda olmali: {gamma_max}")
    if refine_steps < 0 :
        raise ValueError(f"adim pozitif olmali: {refine_steps}")

    pose_s = (start[0], start[1], start[3])
    pose_g = (goal[0], goal[1], goal[3])
    horizontal = shortest_path(pose_s, pose_g, rho)

    alt_diff = goal[2] - start[2]
    required = abs(alt_diff) / math.tan(gamma_max)

    turn = 2 * math.pi * rho
    helix_turns = 0
    if horizontal.length < required:
        helix_turns = math.ceil((required - horizontal.length) / turn)
    total_horizontal = horizontal.length + helix_turns * turn
    plain_length = math.hypot(total_horizontal, alt_diff)

    if helix_turns > 0 and refine_steps > 0:
        new_path = _widened_horizontal(pose_s, pose_g, rho, required,refine_steps)
        if new_path is not None:
            new_length = math.hypot(new_path.length, alt_diff)
            if new_length <= plain_length:
                horizontal = new_path
                helix_turns = 0
                total_horizontal = horizontal.length

    gamma = math.atan2(alt_diff, total_horizontal)
    return DubinsPath3D(start, horizontal, helix_turns, gamma)

def airplane_length(start: Pose3, goal: Pose3, rho: float,
                    gamma_max: float) -> float:
    """Yolun 3B uzunlugu (yatay degil); planlayicinin mesafe olcutu.

    refine_steps bilerek yok: _nearest yalnizca siralama yapiyor ve helis
    sismesi adaylarin cogunu benzer sekilde etkiledigi icin siralama korunuyor.
    Olculdu - refine'li olcut kazanc vermedi, 1.4-2.4 kat yavaslatti.
    """
    return airplane_path(start, goal, rho, gamma_max).length

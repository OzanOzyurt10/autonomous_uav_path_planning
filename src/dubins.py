#Shkel & Lumelsky Yaklaşımı Kullanılarak Üretilmiştir.
import math
from dataclasses import dataclass

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class DubinsPath:
    
    start: Pose
    word: str
    lengths: tuple[float, float, float]  # Segmentlerin uzunluğu (metre)
    rho: float

    @property
    def length(self) -> float:
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
        return self.interpolate(self.length)

    def sample(self, step: float) -> list[Pose]:
       
        if step <= 0:
            raise ValueError(f"step pozitif olmali, verilen: {step}")

        total = self.length
        if total == 0.0:
            return [self.start]

        n = math.ceil(total / step)
        return [self.interpolate(i / n * total) for i in range(n + 1)]


def _mod2pi(theta: float) -> float:
    theta = theta % (2 * math.pi)
    return theta


def _segment_end(pose: Pose, mode: str, s: float, rho: float) -> Pose:
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

def _to_canonical(start: Pose, goal: Pose, rho: float) -> tuple[float, float, float]:
    """ d: Dönüş Yariçapi Cinsinden iki poz arasi mesafe
        # alpha: Başlangiç heading'inin, bağlanti doğrultusuna göre açisi
        # beta: Hedef heading'inin, ayni doğrultuya göre açisi"""
    start_x, start_y, start_yaw = start
    goal_x, goal_y, goal_yaw = goal
    dx    = goal_x - start_x
    dy    = goal_y - start_y
    D     = math.hypot(dx, dy)
    d     = D / rho
    theta = _mod2pi(math.atan2(dy, dx))
    alpha = _mod2pi(start_yaw - theta)
    beta  = _mod2pi(goal_yaw  - theta)
    
    return (d,alpha,beta)
 
                          


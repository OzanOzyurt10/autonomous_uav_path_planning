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


def _mod2pi(theta: float) -> float:
    theta = theta % (2 * math.pi)
    return theta

def _segment_end(pose: Pose, mode: str, s: float, rho: float) -> Pose:
    x, y, psi = pose
    if mode == "S":
        psi2 = psi
        x2 = x + s*math.cos(psi)
        y2 = y + s*math.sin(psi)

    elif mode == "L":
        psi2 = psi + s/rho
        x2 = x + rho*(math.sin(psi2) - math.sin(psi))
        y2 = y - rho*(math.cos(psi2) - math.cos(psi))

    elif mode == "R":
        psi2 = psi - s/rho
        x2 =  x + rho*(math.sin(psi) - math.sin(psi2))
        y2 =  y + rho*(math.cos(psi2) - math.cos(psi))  

    else :
        raise ValueError(f"bilinmeyen segment tipi: {mode}")


    return (x2,y2,psi2)
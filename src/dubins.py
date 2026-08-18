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
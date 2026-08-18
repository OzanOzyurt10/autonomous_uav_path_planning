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

# Kayan nokta toleransi: bu buyuklukteki negatifler sifir kabul edilir.
_EPS = 1e-12


@dataclass(frozen=True) #obje oluştuktan sonra değerler değişemiyor
class DubinsPath:
    """Cozulmus bir Dubins yolu.

    Alanlar:
        start: Yolun basladigi poz.
        word: Segment tipleri, uc harf: "LSL", "RSR", "LSR", "RSL",
            "RLR" veya "LRL".
        lengths: Uc segmentin yay uzunlugu, **metre**. i'inci eleman
            word'un i'inci harfine karsilik gelir. Kanonik cozuculer
            normalize deger dondurur; buraya konmadan once rho ile
            carpilmalidir.
        rho: Donus yaricapi, metre.

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
        return self.interpolate(self.length)

    def sample(self, step: float) -> list[Pose]:
        """Yolu step araliklarla orneklenmis poz listesine cevirir.

        Ilk eleman daima start, son eleman daima son pozdur; ardisik
        noktalar arasindaki mesafe step'i asmaz. Sifir uzunluklu yol
        icin tek elemanli liste doner.

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


def _to_canonical(start: Pose, goal: Pose, rho: float) -> tuple[float, float, float]:
    """ d: Dönüş Yariçapi Cinsinden iki poz arasi mesafe
        alpha: Başlangiç heading'inin, bağlanti doğrultusuna göre açisi
        beta: Hedef heading'inin, ayni doğrultuya göre açisi"""
    start_x, start_y, start_yaw = start
    goal_x, goal_y, goal_yaw = goal
    dx    = goal_x - start_x
    dy    = goal_y - start_y
    D     = math.hypot(dx, dy)
    d     = D / rho
    theta = _mod2pi(math.atan2(dy, dx))
    alpha = _mod2pi(start_yaw - theta)
    beta  = _mod2pi(goal_yaw  - theta)

    return (d, alpha, beta)


def _safe_sqrt(value: float) -> float | None:
    if value < -_EPS:
        return None
    if value < 0.0:
        return 0.0
    return math.sqrt(value)


def _lsl(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None:
    """Sola don, duz git, sola don."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    c_ab = math.cos(alpha - beta)
    cb = math.cos(beta)

    p_sq = 2 + d*d - 2*c_ab + 2*d*(sa - sb)
    p  = _safe_sqrt(p_sq)
    if p is None:
        return None
    tmp = math.atan2(cb - ca, d + sa - sb)
    t   = _mod2pi(-alpha + tmp)
    q   = _mod2pi(beta - tmp)

    return (t, p, q)


def _rsr(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None:
    """Saga don, duz git, saga don."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    c_ab = math.cos(alpha - beta)
    cb = math.cos(beta)

    p_sq = 2 + d*d - 2*c_ab + 2*d*(sb - sa)
    p = _safe_sqrt(p_sq)
    if p is None:
        return None
    tmp = math.atan2(ca - cb, d - sa + sb)
    t   = _mod2pi(alpha - tmp)
    q   = _mod2pi(-_mod2pi(beta) + tmp)

    return (t, p, q)


def _lsr(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None:
    """Sola don, duz git, saga don."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    c_ab = math.cos(alpha - beta)
    cb = math.cos(beta)

    p_sq = -2 + d*d + 2*c_ab + 2*d*(sa + sb)
    p = _safe_sqrt(p_sq)
    if p is None: return None
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    t   = _mod2pi(-alpha + tmp)
    q   = _mod2pi(-_mod2pi(beta) + tmp)

    return (t, p, q)


def _rsl(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None:
    """Saga don, duz git, sola don. """
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    c_ab = math.cos(alpha - beta)
    cb = math.cos(beta)

    p_sq = d*d - 2 + 2*c_ab - 2*d*(sa + sb)
    p = _safe_sqrt(p_sq)
    if p is None:
        return None
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    t   = _mod2pi(alpha - tmp)
    q   = _mod2pi(beta - tmp)

    return (t, p, q)



def _rlr(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None:
    """Saga don, sola don, saga don."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    c_ab = math.cos(alpha - beta)
    cb = math.cos(beta)

    tmp = (6 - d*d + 2*c_ab + 2*d*(sa - sb)) / 8
    if abs(tmp) > 1.0:
        return None
    p = _mod2pi(2*math.pi - math.acos(tmp))
    t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p/2)
    q = _mod2pi(alpha - beta - t + p)

    return (t, p, q)


def _lrl(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None:
    """Sola don, saga don, sola don."""
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    c_ab = math.cos(alpha - beta)
    cb = math.cos(beta)

    tmp = (6 - d*d + 2*c_ab + 2*d*(-sa + sb)) / 8
    if abs(tmp) > 1.0:
        return None
    p = _mod2pi(2*math.pi - math.acos(tmp))
    t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p/2)
    q = _mod2pi(beta - alpha - t + p)

    return (t, p, q)


_SOLVERS = {
    "LSL": _lsl,
    "RSR": _rsr,
    "LSR": _lsr,
    "RSL": _rsl,
    "RLR": _rlr,
    "LRL": _lrl,
}


def all_paths(start: Pose, goal: Pose, rho: float) -> list[DubinsPath]:
    """start'tan goal'a giden tum gecerli Dubins yollarini dondurur.

    Alti kelimenin her biri denenir; geometrik olarak imkansiz olanlar
    listeye girmez. Liste en az bir eleman icerir.

    Raises:
        ValueError: rho pozitif degilse.
    """
    if rho <= 0:
        raise ValueError(f"rho pozitif olmali, verilen: {rho}")

    d, alpha, beta = _to_canonical(start, goal, rho)
    paths = []
    for word, solver in _SOLVERS.items():
        result = solver(d, alpha, beta)
        if result is None:
            continue
        # Cozuculer normalize deger dondurur; DubinsPath metre bekler.
        lengths = tuple(x * rho for x in result)
        paths.append(DubinsPath(start, word, lengths, rho))
    return paths


def shortest_path(start: Pose, goal: Pose, rho: float) -> DubinsPath:
    """start'tan goal'a giden en kisa Dubins yolu.

    Raises:
        ValueError: rho pozitif degilse.
        RuntimeError: hicbir kelime gecerli degilse. rho > 0 iken Dubins
            teoremi geregi olmamalidir.
    """
    paths = all_paths(start, goal, rho)
    if not paths:
        raise RuntimeError(
            f"hicbir Dubins kelimesi gecerli degil: "
            f"start={start}, goal={goal}, rho={rho}")
    return min(paths, key=lambda p: p.length)


def path_length(start: Pose, goal: Pose, rho: float) -> float:
    """En kisa Dubins yolunun uzunlugu (metre).

    Raises:
        ValueError: rho pozitif degilse.
        RuntimeError: hicbir kelime gecerli degilse.
    """
    return shortest_path(start, goal, rho).length

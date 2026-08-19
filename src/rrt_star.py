"""Dubins yollariyla calisan RRT rota planlayici.

dubins.py ile environment.py'yi birlestiren tek yer burasi. dubins.py
engelleri, environment.py de ucak dinamigini bilmiyor; ikisini haberdar
etmek yerine planlayici ortada durup "su Dubins yolu su ortamda serbest
mi" sorusunu soruyor.

Bu asamada duz RRT var: agac buyur, hedefe baglanabilirsen dur. RRT*'in
yeniden baglama (rewire) adimi ikinci asamada yine bu dosyaya eklenecek,
cunku ayni agac yapisi uzerinde calisiyor.

Koordinat duzeni dubins.py ile ayni: poz (x, y, yaw), ENU, yaw radyan.
"""

import math
import random
from dataclasses import dataclass

from src.dubins import DubinsPath, path_length, shortest_path
from src.environment import Environment

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class Node:
    """Agactaki tek bir poz ve oraya nereden gelindigi.

    Agac, ic ice nesne referanslari yerine bir liste olarak tutuluyor;
    parent o listedeki indeks, kok dugum icin None. Indeks tutmanin uc
    faydasi var: agaci oldugu gibi yazdirip inceleyebiliyorsun, dongusel
    referans olusma riski yok, ve ikinci asamada bir dugumu baska bir
    ebeveyne baglamak tek bir indeksi degistirmek demek.

    cost koktan bu poza kadar metre cinsinden toplam yol; kok icin 0.0.
    path_from_parent ebeveynden buraya gelen Dubins yolu, kok icin None.
    """

    pose: Pose
    parent: int | None
    cost: float
    path_from_parent: DubinsPath | None


@dataclass(frozen=True)
class RRTResult:
    """Bir planlama kosusunun sonucu.

    Rota bulunamamasi istisna degil sonuc: RRT rastgele ornekleme yapiyor,
    verilen yineleme butcesinde hedefe ulasamamak mesru bir cikti. O yuzden
    found=False donuyor, hata firlatilmiyor.

    Basarisiz kosuda bile tree dolu doner; agacin nereye kadar yayilip
    nerede tikandigini gorup sorunu teshis edebilesin diye. edges bos liste,
    cost ise math.inf olur.

    edges koktan hedefe siralı Dubins yollari, cost bunlarin toplam uzunlugu,
    iterations ise donguyu kac kez dondugumuz.
    """

    found: bool
    edges: list[DubinsPath]
    cost: float
    iterations: int
    tree: list[Node]


def _try_connect(env: Environment, from_pose: Pose, to_pose: Pose,
                 rho: float, step: float) -> DubinsPath | None:
    """Iki poz arasinda gecerli bir kenar kurmaya calisir.

    En kisa Dubins yolunu olusturur, step araligiyla ornekler ve carpisma
    kontrolunden gecirir. Temizse yolu, degilse None doner.

    Kenar gecerliliginin tek karar noktasi burasi: agaca giren her kenar
    buradan geciyor, dolayisiyla carpisma kuralini degistirmek istersen
    bakilacak tek yer bu fonksiyon.

    Sinir kontrolu ayrica yazilmiyor; harita disina cikan bir yolun
    orneklenen noktalarini env.is_free zaten eliyor.
    """
    path = shortest_path(from_pose, to_pose, rho)
    if env.is_path_free(path.sample(step)):
        return path
    return None


def _nearest(nodes: list[Node], target: Pose, rho: float) -> int:
    """Hedefe en yakin dugumun indeksini doner (dugumun kendisini degil)."""
    min_index = None
    min_dist = None
    for i in range(len(nodes)):
        dist = path_length(nodes[i].pose, target, rho)
        if min_index is None or dist < min_dist:
            min_index = i
            min_dist = dist
    return min_index


def _sample(env: Environment, goal: Pose, rng: random.Random,
            goal_bias: float) -> Pose:
    """goal_bias olasilikla hedefi, aksi halde rastgele serbest bir poz doner."""
    if rng.random() < goal_bias:
        return goal
    return env.random_free_pose(rng)


def _extract_path(nodes: list[Node], index: int,
                  goal_edge: DubinsPath) -> list[DubinsPath]:
    """nodes[index]'ten koke uzanan zinciri gidis sirasinda kenar listesine cevirir."""
    edges = []
    while nodes[index].parent is not None:
        edges.append(nodes[index].path_from_parent)
        index = nodes[index].parent
    edges.reverse()          # zincir yapraktan koke toplandi, gidis sirasina cevir
    edges.append(goal_edge)
    return edges


def plan(start: Pose, goal: Pose, env: Environment, rho: float,
         max_iterations: int = 5000, goal_bias: float = 0.05,
         step: float | None = None, rng: random.Random | None = None,
         stop_on_first_solution: bool = True) -> RRTResult:
    """start'tan goal'a carpismasiz bir Dubins rotasi arar.

    Duz RRT: her yinelemede bir poz orneklenir, agactaki en yakin dugumden
    oraya bir Dubins kenari kurulmaya calisilir, kenar temizse agac buyur.
    Ardindan yeni dugumden hedefe uzanilmaya calisilir; o kenar da temizse
    rota bulunmustur.

    Parametreler:
        start, goal    : baslangic ve hedef pozu, (x, y, yaw)
        env            : engel ve sinir bilgisini tutan ortam
        rho            : minimum donus yaricapi, metre
        max_iterations : yineleme butcesi
        goal_bias      : orneklemenin dogrudan hedefi secme olasiligi
        step           : carpisma kontrolu ornekleme araligi; None ise
                         env.suggested_step() kullanilir
        rng            : tohumlanmis random.Random; None ise tohumsuz uretilir
        stop_on_first_solution : ilk cozumde hemen donulsun mu

    Donus: RRTResult. found=False bir istisna degil, mesru bir sonuc -
    rastgele bir planlayici verilen butcede hedefe ulasamayabilir. O durumda
    edges bos, cost math.inf, tree ise doludur; agacin nereye kadar yayilip
    nerede tikandigini gorup sorunu teshis edebilesin diye.

    ValueError firlatir: rho, max_iterations veya step pozitif degilse;
    goal_bias 0.0-1.0 disindaysa; start veya goal engelliyse ya da harita
    disindaysa. Son ikisi olmasa planlayici butun butceyi harcayip
    "bulamadim" derdi ve hata algoritmada sanilirdi.

    stop_on_first_solution=False bu asamada rotayi iyilestirmez; duz RRT
    buldugu ilk cozumu degistirmez, yalnizca agac buyumeye devam eder.
    Parametre simdiden imzada duruyor ki ikinci asamada yeniden baglama
    (rewire) eklendiginde API degismesin.
    """
    if rho <= 0.0:
        raise ValueError(f"rho pozitif olmali: {rho}")
    if max_iterations <= 0:
        raise ValueError(f"max_iterations pozitif olmali: {max_iterations}")
    if goal_bias < 0.0 or goal_bias > 1.0:
        raise ValueError(f"goal_bias 0.0-1.0 araliginda olmali: {goal_bias}")
    if step is not None and step <= 0.0:
        raise ValueError(f"step pozitif olmali: {step}")
    if not env.is_free(start):
        raise ValueError(f"baslangic pozu engelli veya harita disinda: {start}")
    if not env.is_free(goal):
        raise ValueError(f"hedef pozu engelli veya harita disinda: {goal}")

    if step is None:
        step = env.suggested_step()
    if rng is None:
        rng = random.Random()

    nodes = [Node(start, None, 0.0, None)]
    best = None          # bulunan cozum: (edges, cost, iteration)

    for iteration in range(1, max_iterations + 1):
        target = _sample(env, goal, rng, goal_bias)
        i = _nearest(nodes, target, rho)
        edge = _try_connect(env, nodes[i].pose, target, rho, step)
        if edge is None:
            continue

        nodes.append(Node(target, i, nodes[i].cost + edge.length, edge))

        goal_edge = _try_connect(env, target, goal, rho, step)
        if goal_edge is None:
            continue
        if best is not None:
            continue

        edges = _extract_path(nodes, len(nodes)- 1 , goal_edge)
        best = (edges, nodes[-1].cost + goal_edge.length, iteration)

        if stop_on_first_solution:
            return RRTResult(True, edges, best[1], iteration, nodes)

    if best is not None:
        return RRTResult(True, best[0], best[1], max_iterations, nodes)
    return RRTResult(False, [], math.inf, max_iterations, nodes)

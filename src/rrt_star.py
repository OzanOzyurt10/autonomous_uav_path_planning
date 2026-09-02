"""Dubins yollariyla calisan RRT rota planlayici.

dubins.py ile environment.py'yi birlestiren tek yer. Bu asamada duz RRT;
RRT*'in rewire adimi ikinci asamada buraya eklenecek.

Poz duzeni dubins.py ile ayni: (x, y, yaw), ENU, yaw radyan.
"""
import collections
import dataclasses
import math
import random
from dataclasses import dataclass

from src.dubins import DubinsPath, path_length, shortest_path
from src.environment import Environment

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class CostModel:
    """Planlayicinin neyi kucultmeye calistigi.

    Planlayici artik "metre" bilmiyor, yalnizca maliyet biliyor. Ruzgarli
    planlama demek, buraya sureyi donduren bir model gecirmek demek;
    shortcut da AYNI modeli aliyor, boylece ikisinin farkli seyi optimize
    etmesi imkansiz.
    """

    of_edge: object                # kenar -> maliyet
    # Oklid mesafesini o mesafenin MUMKUN EN DUSUK maliyetine cevirir.
    # Budama buna dayaniyor: alt sinir bile eldekini gecmiyorsa aday kesin
    # kaybeder. Gercek maliyetten buyuk donerse iyi adaylar sessizce elenir.
    lower_bound: object


# Varsayilan: maliyet = kenar uzunlugu. Dubins yolu duz cizgiden kisa
# olamayacagi icin alt sinir mesafenin kendisi.
LENGTH = CostModel(of_edge=lambda edge: edge.length,
                   lower_bound=lambda distance: distance)


@dataclass(frozen=True)
class Node:
    """Agactaki bir poz ve oraya nereden gelindigi.

    Agac liste olarak tutuluyor, parent o listedeki indeks (kok icin None).
    cost koktan buraya toplam metre; path_from_parent ebeveynden gelen kenar.
    """

    pose: Pose
    parent: int | None
    cost: float
    path_from_parent: DubinsPath | None


@dataclass(frozen=True)
class RRTResult:
    """Bir planlama kosusunun sonucu.

    Rota bulunamamasi istisna degil sonuc: found=False, edges bos, cost inf.
    tree o durumda da dolu doner, tikanmanin nerede oldugunu gormek icin.
    """

    found: bool
    edges: list[DubinsPath]
    cost: float
    iterations: int
    tree: list[Node]


def _try_connect(env: Environment, from_pose: Pose, to_pose: Pose,
                 rho: float, step: float) -> DubinsPath | None:
    """Iki poz arasinda gecerli bir kenar kurar; carpisirsa None doner."""
    path = shortest_path(from_pose, to_pose, rho)
    if env.is_path_free(path.sample(step)):
        return path
    return None


def _euclid(a: Pose, b: Pose) -> float:
    """Iki poz arasindaki yatay Oklid mesafesi; yaw yok sayilir."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _steer(from_pose: Pose, to_pose: Pose, rho: float,
           max_edge_length: float | None) -> Pose:
    """OPTIMIZASYON - adimli ilerleme (steering).

    Uzak bir ornege tam yol kurmak cogu zaman carpisma yuzunden reddediliyor
    ve o yineleme bosa gidiyor. Yol max_edge_length'i asiyorsa hedef ayni
    yol uzerindeki ara poza kirpiliyor. None ise davranis eskisiyle ayni.
    """
    if max_edge_length is None:
        return to_pose

    path = shortest_path(from_pose, to_pose, rho)
    if not path.length > max_edge_length:
        return to_pose
    return path.interpolate(max_edge_length)


def _nearest(nodes: list[Node], target: Pose, rho: float) -> int:
    """Hedefe en yakin dugumun indeksini doner (dugumu degil).

    Oklid mesafesi Dubins mesafesinin alt siniri oldugu icin dugumler once
    ucuz hypot ile siralanip, eldeki en iyi Dubins mesafesini asan ilk
    dugumde donguden cikiliyor. Sonuc budamasiz halle birebir ayni; yalnizca
    kesin olarak elenebilecek Dubins cozumleri atlaniyor.
    """
    dist = [_euclid(node.pose, target) for node in nodes]
    order = sorted(range(len(dist)), key=lambda i: dist[i])

    best_index = None
    best_dist = None
    for i in order:
        if best_dist is not None and dist[i] >= best_dist:
            break
        dub = path_length(nodes[i].pose, target, rho)
        if best_dist is None or dub < best_dist:
            best_index = i
            best_dist = dub
    return best_index


def _sample(env: Environment, goal: Pose, rng: random.Random,
            goal_bias: float) -> Pose:
    """goal_bias olasilikla hedefi, aksi halde rastgele serbest bir poz doner."""
    if rng.random() < goal_bias:
        return goal
    return env.random_free_pose(rng)


def _extract_path(nodes: list[Node], index: int,
                  goal_edge: DubinsPath) -> list[DubinsPath]:
    """Zinciri koke kadar geri takip edip gidis sirasinda kenar listesi verir."""
    edges = []
    while nodes[index].parent is not None:
        edges.append(nodes[index].path_from_parent)
        index = nodes[index].parent
    edges.reverse()          # zincir yapraktan koke toplandi, gidis sirasina cevir
    edges.append(goal_edge)
    return edges


# 2B'de birim yuvarlagin alani pi; gerekce rrt_star3d._auto_radius'ta.
def _auto_radius(bounds, rho: float) -> tuple[float, float]:
    """Komsuluk yaricabini harita olceginden turetir; 3B ile ayni gerekce."""
    x_min, y_min, x_max, y_max = bounds
    area = (x_max - x_min) * (y_max - y_min)
    gamma = 2 * (3 / 2) ** 0.5 * (area / math.pi) ** 0.5
    return gamma, 2 * rho


def _neighbour_radius(n: int, gamma: float, cap: float) -> float:
    """RRT* komsuluk yaricapi; agac buyudukce kuculur."""
    if n <= 1:
        return cap

    return min(gamma * (math.log(n) / n) ** (1 / 3), cap)


def _neighbours(nodes: list[Node], pose: Pose, radius: float) -> list[int]:
    """Yaricap icindeki dugumlerin indekslerini doner; Oklid ile eler."""
    neighbour_list = []
    x, y, _ = pose
    for i in range(len(nodes)):
        x_n, y_n, _ = nodes[i].pose
        if math.hypot(x_n - x, y_n - y) <= radius:
            neighbour_list.append(i)
    return neighbour_list


def _choose_parent(env: Environment, nodes: list[Node], pose: Pose,
                   candidates: list[int], rho: float, step: float,
                   fallback: tuple[int, DubinsPath],
                   model: CostModel = LENGTH) -> tuple[int, DubinsPath]:
    """Adaylar arasindan poza en ucuza ulastirani secer, (indeks, kenar) doner."""
    best_index, best_edge = fallback
    best_cost = nodes[best_index].cost + model.of_edge(best_edge)
    for j in candidates:
        edge = _try_connect(env, nodes[j].pose, pose, rho, step)
        if edge is None:
            continue
        cost = nodes[j].cost + model.of_edge(edge)
        if cost < best_cost:
            best_index = j
            best_edge = edge
            best_cost = cost
    return best_index, best_edge

def _propagate_cost(nodes: list[Node], index: int,
                    model: CostModel = LENGTH) -> None:
    """index'in altindaki alt agacin maliyetlerini gunceller; listeyi yerinde degistirir.

    Node frozen oldugu icin dugumun yerine dataclasses.replace ile yenisi konur.
    Kenarlarin geometrisi degismez, yalnizca koke kadarki toplam maliyet.
    """
    queue = collections.deque([index])
    while queue:
        i = queue.popleft()
        for c in range(len(nodes)):
            if nodes[c].parent != i:
                continue
            nodes[c] = dataclasses.replace(
                nodes[c],
                cost=nodes[i].cost + model.of_edge(nodes[c].path_from_parent))
            queue.append(c)

def _rewire(env: Environment, nodes: list[Node], new_index: int,
            candidates: list[int], rho: float, step: float,
            model: CostModel = LENGTH) -> None:
    """Yeni dugum uzerinden gecmek ucuzlatiyorsa komsulari ona baglar.

    Yon _choose_parent'in tersi: kenar yeni dugumden adaya kurulur, cunku
    maliyet yeni.cost + d(yeni -> aday). Ayri bir dongu kontrolu gerekmez;
    aday yeni dugumun atasiysa iyilestirme kosulu zaten tutmaz.
    """
    for j in candidates:
        if j == new_index:
            continue
        new_edge = _try_connect(env, nodes[new_index].pose, nodes[j].pose, rho, step)
        if new_edge is None:
            continue
        new_cost = nodes[new_index].cost + model.of_edge(new_edge)
        if new_cost >= nodes[j].cost:
            continue
        nodes[j] = dataclasses.replace(nodes[j], parent=new_index,
                                       cost=new_cost,
                                       path_from_parent=new_edge)
        _propagate_cost(nodes, j, model)


def plan(start: Pose, goal: Pose, env: Environment, rho: float,
         max_iterations: int = 5000, goal_bias: float = 0.05,
         step: float | None = None, rng: random.Random | None = None,
         stop_on_first_solution: bool = False,
         max_edge_length: float | None = None,
         radius_gamma: float | None = None,
         radius_cap: float | None = None,
         model: CostModel = LENGTH) -> RRTResult:
    """start'tan goal'a carpismasiz bir Dubins rotasi arar (RRT*).

    Her yinelemede bir poz orneklenir, en yakin dugumden oraya kenar kurulur,
    komsular arasindan en ucuz ebeveyn secilir, sonra hedefe uzanilmaya
    calisilir. Hedefe ulasan tum baglantilar biriktirilir; sonunda en ucuzu
    secilir.
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
    if max_edge_length is not None and max_edge_length <= 0.0:
        raise ValueError(f"max_edge_length pozitif olmali: {max_edge_length}")
    if radius_gamma is not None and radius_gamma <= 0.0:
        raise ValueError(f"radius_gamma pozitif olmali: {radius_gamma}")
    if radius_cap is not None and radius_cap <= 0.0:
        raise ValueError(f"radius_cap pozitif olmali: {radius_cap}")

    auto_gamma, auto_cap = _auto_radius(env.bounds, rho)
    if radius_gamma is None:
        radius_gamma = auto_gamma
    if radius_cap is None:
        radius_cap = auto_cap

    if step is None:
        step = env.suggested_step()
    if rng is None:
        rng = random.Random()

    nodes = [Node(start, None, 0.0, None)]
    goal_links = []          # (dugum indeksi, hedefe giden kenar)

    for iteration in range(1, max_iterations + 1):
        target = _sample(env, goal, rng, goal_bias)
        i = _nearest(nodes, target, rho)
        target = _steer(nodes[i].pose, target, rho, max_edge_length)
        edge = _try_connect(env, nodes[i].pose, target, rho, step)
        if edge is None:
            continue

        # en iyi komsu araniyor
        radius = _neighbour_radius(len(nodes), radius_gamma, radius_cap)
        cands = _neighbours(nodes, target, radius)
        i, edge = _choose_parent(env, nodes, target, cands, rho, step,
                                 (i, edge), model)

        nodes.append(Node(target, i, nodes[i].cost + model.of_edge(edge), edge))
        _rewire(env, nodes, len(nodes) - 1, cands, rho, step, model)

        goal_edge = _try_connect(env, target, goal, rho, step)
        if goal_edge is None:
            continue

        goal_links.append((len(nodes) - 1, goal_edge))
        if stop_on_first_solution:
            edges = _extract_path(nodes, len(nodes) - 1, goal_edge)
            cost = nodes[-1].cost + model.of_edge(goal_edge)
            return RRTResult(True, edges, cost, iteration, nodes)

    if not goal_links:
        return RRTResult(False, [], math.inf, max_iterations, nodes)

    index, goal_edge = min(
        goal_links,
        key=lambda link: nodes[link[0]].cost + model.of_edge(link[1]))
    edges = _extract_path(nodes, index, goal_edge)
    cost = nodes[index].cost + model.of_edge(goal_edge)
    return RRTResult(True, edges, cost, max_iterations, nodes)




def shortcut_with(edges, connect, max_rounds: int = 10,
                  model: CostModel = LENGTH):
    """Rotadaki gereksiz duraklari atarak kisaltir; girdi listesi degismez.

    Ardisik olmayan iki pozu dogrudan baglamayi dener, bag gecerliyse ve
    aradaki kenarlarin toplamindan kisaysa kabul eder. Kazanc kalmayana ya
    da max_rounds dolana kadar tekrarlanir.

    connect(a, b) iki poz arasinda gecerli bir kenar dondurmeli, yoksa
    None. Boyutu bilmiyor: kenarin .start, .length ve .end_pose()'u olmasi
    yetiyor, o yuzden ayni algoritma 2B ve 3B ile calisiyor.

    Uzunluk kiyasi SART, carpisma kontrolu yetmez: 3B'de dogrudan bag
    helis turu atmak zorunda kalip daha UZUN olabiliyor.
    """
    if not edges:
        return []

    edges = list(edges)
    poses = [edge.start for edge in edges]
    poses.append(edges[-1].end_pose())

    for _ in range(max_rounds):
        changed = False
        i = 0
        # len(edges) her adimda yeniden okunuyor: kisayol kabul edilince
        # liste kisaliyor ve eski uzunlukla dolasmak indeks tasirir.
        while i + 2 <= len(edges):
            j = len(edges)
            while j >= i + 2:
                new_edge = connect(poses[i], poses[j])
                if (new_edge is not None
                        and model.of_edge(new_edge) < sum(
                            model.of_edge(e) for e in edges[i:j])):
                    edges[i:j] = [new_edge]
                    del poses[i + 1:j]
                    changed = True
                    break
                j -= 1
            i += 1
        if not changed:
            break
    return edges


def shortcut(edges: list[DubinsPath], env: Environment, rho: float,
             step: float, max_rounds: int = 10,
             model: CostModel = LENGTH) -> list[DubinsPath]:
    """shortcut_with'in 2B kolayligi; baglantiyi ortamdan kuruyor."""
    return shortcut_with(
        edges, lambda a, b: _try_connect(env, a, b, rho, step), max_rounds,
        model)

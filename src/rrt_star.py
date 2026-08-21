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


def _nearest(nodes: list[Node], target: Pose, rho: float) -> int:
    """Hedefe en yakin dugumun indeksini doner (dugumu degil)."""
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
    """Zinciri koke kadar geri takip edip gidis sirasinda kenar listesi verir."""
    edges = []
    while nodes[index].parent is not None:
        edges.append(nodes[index].path_from_parent)
        index = nodes[index].parent
    edges.reverse()          # zincir yapraktan koke toplandi, gidis sirasina cevir
    edges.append(goal_edge)
    return edges


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
                   fallback: tuple[int, DubinsPath]) -> tuple[int, DubinsPath]:
    """Adaylar arasindan poza en ucuza ulastirani secer, (indeks, kenar) doner."""
    best_index, best_edge = fallback
    best_cost = nodes[best_index].cost + best_edge.length
    for j in candidates:
        edge = _try_connect(env, nodes[j].pose, pose, rho, step)
        if edge is None:
            continue
        cost = nodes[j].cost + edge.length
        if cost < best_cost:
            best_index = j
            best_edge = edge
            best_cost = cost
    return best_index, best_edge

def _propagate_cost(nodes: list[Node], index: int) -> None:
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
            nodes[c] = dataclasses.replace(nodes[c], cost=nodes[i].cost + nodes[c].path_from_parent.length)
            queue.append(c)

def _rewire(env: Environment, nodes: list[Node], new_index: int,
            candidates: list[int], rho: float, step: float) -> None:
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
        new_cost = nodes[new_index].cost + new_edge.length
        if new_cost >= nodes[j].cost:
            continue
        nodes[j] = dataclasses.replace(nodes[j], parent=new_index, cost=new_cost, path_from_parent=new_edge)
        _propagate_cost(nodes, j)


def plan(start: Pose, goal: Pose, env: Environment, rho: float,
         max_iterations: int = 5000, goal_bias: float = 0.05,
         step: float | None = None, rng: random.Random | None = None,
         stop_on_first_solution: bool = False,
         radius_gamma: float = 60.0, radius_cap: float = 30.0) -> RRTResult:
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
    if radius_gamma <= 0.0:
        raise ValueError(f"radius_gamma pozitif olmali: {radius_gamma}")
    if radius_cap <= 0.0:
        raise ValueError(f"radius_cap pozitif olmali: {radius_cap}")

    if step is None:
        step = env.suggested_step()
    if rng is None:
        rng = random.Random()

    nodes = [Node(start, None, 0.0, None)]
    goal_links = []          # (dugum indeksi, hedefe giden kenar)

    for iteration in range(1, max_iterations + 1):
        target = _sample(env, goal, rng, goal_bias)
        i = _nearest(nodes, target, rho)
        edge = _try_connect(env, nodes[i].pose, target, rho, step)
        if edge is None:
            continue

        # en iyi komsu araniyor
        radius = _neighbour_radius(len(nodes), radius_gamma, radius_cap)
        cands = _neighbours(nodes, target, radius)
        i, edge = _choose_parent(env, nodes, target, cands, rho, step, (i, edge))

        nodes.append(Node(target, i, nodes[i].cost + edge.length, edge))
        _rewire(env, nodes, len(nodes) - 1, cands, rho, step)

        goal_edge = _try_connect(env, target, goal, rho, step)
        if goal_edge is None:
            continue

        goal_links.append((len(nodes) - 1, goal_edge))
        if stop_on_first_solution:
            edges = _extract_path(nodes, len(nodes) - 1, goal_edge)
            cost = nodes[-1].cost + goal_edge.length
            return RRTResult(True, edges, cost, iteration, nodes)

    if not goal_links:
        return RRTResult(False, [], math.inf, max_iterations, nodes)

    index, goal_edge = min(goal_links,
                           key=lambda link: nodes[link[0]].cost + link[1].length)
    edges = _extract_path(nodes, index, goal_edge)
    cost = nodes[index].cost + goal_edge.length
    return RRTResult(True, edges, cost, max_iterations, nodes)



"""Dubins yollariyla calisan RRT rota planlayici.

dubins.py ile environment.py'yi birlestiren tek yer. Bu asamada duz RRT;
RRT*'in rewire adimi ikinci asamada buraya eklenecek.

Poz duzeni dubins.py ile ayni: (x, y, yaw), ENU, yaw radyan.
"""
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
    """Iki poz arasinda gecerli bir kenar kurar; carpisirsa None doner.

    Kenar gecerliliginin tek karar noktasi. Sinir kontrolu ayrica yok,
    harita disina cikan orneklem noktalarini env.is_free zaten eliyor.
    """
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
    """Zinciri koke kadar geri takip edip gidis sirasinda kenar listesi verir.

    Zincir yalnizca yapraktan koke yurunebildigi icin kenarlar ters birikiyor;
    sonda cevriliyor. goal_edge cevirmeden sonra ekleniyor ki sonda kalsin.
    """
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

    Her yinelemede bir poz orneklenir, en yakin dugumden oraya kenar kurulur,
    temizse agac buyur; ardindan yeni dugumden hedefe uzanilmaya calisilir.

    step None ise env.suggested_step(), rng None ise tohumsuz uretilir.

    stop_on_first_solution=False bu asamada rotayi iyilestirmez; parametre
    ikinci asamadaki rewire icin simdiden imzada duruyor.
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


def _neighbour_radius(n: int, gamma: float, cap: float) -> float:
    """RRT* komsuluk yaricapi; agac buyudukce kuculur."""
    if n <= 1:
        return cap

    return min(gamma * (math.log(n) / n) ** (1 / 3), cap)

def _neighbours(nodes: list[Node], pose: Pose, radius: float) -> list[int]:
    """Yaricap icindeki dugumlerin indekslerini doner; Oklid ile eler.

    Eleme kayipsiz: bir Dubins yolu duz cizgiden kisa olamaz, yani Oklid
    mesafesi yaricapi asan dugumun Dubins mesafesi de asar. Ucuz hypot ile
    suzup pahali Dubins'i yalnizca kalanlara uyguluyoruz.
    """
    neighbour_list = []
    x, y, _ = pose
    for i in range(len(nodes)):
        x_n, y_n, _ = nodes[i].pose
        if math.hypot(x_n - x, y_n - y) <= radius:
            neighbour_list.append(i)
    return neighbour_list



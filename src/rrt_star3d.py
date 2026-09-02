import collections
import dataclasses
import math
import random
from dataclasses import dataclass

from src.dubins3d import DubinsPath3D, airplane_length, airplane_path
from src.environment3d import Bounds3, Environment3D
from src.rrt_star import LENGTH, CostModel, shortcut_with

Pose3 = tuple[float,float,float,float]

@dataclass(frozen=True)
class Node3:
    pose:Pose3
    parent:int | None
    cost:float
    path_from_parent:DubinsPath3D | None


@dataclass(frozen=True)
class RRTResult3:
    found:bool
    edges:list[DubinsPath3D]
    cost:float
    iterations:int
    tree:list[Node3]


def _try_connect(env: Environment3D, from_pose: Pose3, to_pose: Pose3,
                 rho: float, gamma_max: float, step: float,
                 refine_steps: int = 0) -> DubinsPath3D | None:
    path = airplane_path(from_pose, to_pose, rho, gamma_max, refine_steps)
    if env.is_path_free(path.sample(step)):
        return path
    return None

def _steer(from_pose: Pose3, to_pose: Pose3, rho: float, gamma_max: float,
           max_edge_length: float | None, refine_steps: int = 0) -> Pose3:
    """OPTIMIZASYON - adimli ilerleme (steering).

    Uzak bir ornege tam yol kurmak cogu zaman reddediliyor ve o yineleme
    bosa gidiyor. Yol max_edge_length'i asiyorsa hedef, ayni yol uzerindeki
    ara poza kirpiliyor; agac daha kisa ama daha sik kenarlarla buyuyor.
    None ise kirpma yok, davranis eskisiyle ayni.

    Ara poza giden en kisa yol kesilen on ekten farkli olabilir ve nadiren
    max_edge_length'i asabilir; kenar yine gecerli oldugu icin kabul ediliyor.
    """
    if max_edge_length is None:
        return to_pose

    path = airplane_path(from_pose, to_pose, rho, gamma_max, refine_steps)
    if not path.length > max_edge_length:
        return to_pose
    return path.interpolate(max_edge_length)
 

def _euclid(a: Pose3, b: Pose3) -> float:
    """Iki poz arasindaki 3B Oklid mesafesi; yaw yok sayilir."""
    return math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _nearest(nodes: list[Node3], target: Pose3, rho: float,
             gamma_max: float) -> int:
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
        dub = airplane_length(nodes[i].pose, target, rho, gamma_max)
        if best_dist is None or dub < best_dist:
            best_index = i
            best_dist = dub
    return best_index


def _sample(env: Environment3D, goal: Pose3, rng: random.Random,
            goal_bias: float) -> Pose3:
    if rng.random() < goal_bias:
        return goal
    return env.random_free_pose(rng)


def _extract_path(nodes: list[Node3], index: int,
                  goal_edge: DubinsPath3D) -> list[DubinsPath3D]:
    """Zinciri koke kadar geri takip edip gidis sirasinda kenar listesi verir."""
    edges = []
    while nodes[index].parent is not None:
        edges.append(nodes[index].path_from_parent)
        index = nodes[index].parent
    edges.reverse()          # zincir yapraktan koke toplandi, gidis sirasina cevir
    edges.append(goal_edge)
    return edges


# Karaman & Frazzoli RRT* yaricapi: gamma > 2 (1 + 1/d)^(1/d)
# (mu(X_free) / zeta_d)^(1/d). Serbest hacim yerine harita hacmi aliniyor;
# serbest hacim ondan kucuk oldugu icin yaricap guvenli tarafta kaliyor.
_UNIT_BALL_3 = 4 * math.pi / 3


def _auto_radius(bounds: Bounds3, rho: float) -> tuple[float, float]:
    """Komsuluk yaricabini harita olceginden turetir.

    Sabit metre varsayilanlari 100x60 m'lik demo haritasina gore
    ayarliydi; 20 km'lik haritada yaricap 18 m'ye dusuyor, komsu kumesi
    bosaliyor ve RRT* fiilen duz RRT'ye donuyor.

    Tavan bilerek kucuk: olculdu ki genis komsulugun ham agaca kazandirdigi
    %14'u shortcut zaten geri kazaniyor, ustelik 3.4 kat hizli.
    """
    x_min, y_min, z_min, x_max, y_max, z_max = bounds
    volume = (x_max - x_min) * (y_max - y_min) * (z_max - z_min)
    gamma = 2 * (4 / 3) ** (1 / 3) * (volume / _UNIT_BALL_3) ** (1 / 3)
    return gamma, 2 * rho


def _neighbour_radius(n: int, gamma: float, radius: float) -> float:
    """RRT* komsuluk yaricapi; agac buyudukce kuculur."""
    if n <= 1:
            return radius  
    return min(gamma * (math.log(n) / n) ** (1 / 4), radius)

def _neighbours(nodes: list[Node3], pose: Pose3, radius: float) -> list[int]:
    index = []
    x,y,z,_ = pose
    for i in range (len(nodes)):
        x_n,y_n,z_n,_=nodes[i].pose
        if math.hypot(x-x_n , y-y_n, z-z_n) <= radius:
            index.append(i)

    return index

def _choose_parent(env: Environment3D, nodes: list[Node3], pose: Pose3,
                   candidates: list[int], rho: float, gamma_max: float,
                   step: float,
                   fallback: tuple[int, DubinsPath3D],
                   refine_steps: int = 0,
                   model: CostModel = LENGTH) -> tuple[int, DubinsPath3D]:

    best_index , best_edge = fallback
    best_cost = nodes[best_index].cost + model.of_edge(best_edge)
    for j in candidates:
        # OPTIMIZASYON - Oklid alt siniri ile budama.
        # Gercek maliyet nodes[j].cost + d(j -> pose); Dubins mesafesi asla
        # duz cizgiden kisa olamayacagi icin bu ifade bir alt sinir. Alt sinir
        # bile best_cost'u gecmiyorsa aday kesin kaybeder; pahali _try_connect
        # (Dubins cozumu + ornekleme + carpisma kontrolu) hic cagrilmiyor.
        # Sonuc degismiyor, yalnizca gereksiz hesap atlaniyor.
        if not (nodes[j].cost
                + model.lower_bound(_euclid(nodes[j].pose, pose))
                < best_cost):
            continue
        edge = _try_connect(env, nodes[j].pose, pose, rho, gamma_max, step,
                            refine_steps)
        if edge is None:
            continue
        if model.of_edge(edge) + nodes[j].cost < best_cost:
            best_cost = model.of_edge(edge) + nodes[j].cost
            best_edge = edge
            best_index = j
    return (best_index,best_edge)


def _propagate_cost(nodes: list[Node3], index: int,
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

def _rewire(env: Environment3D, nodes: list[Node3], new_index: int,
            candidates: list[int], rho: float, gamma_max: float,
            step: float, refine_steps: int = 0,
            model: CostModel = LENGTH) -> None:
    for j in candidates:
        if j == new_index:
            continue
        # OPTIMIZASYON - Oklid alt siniri ile budama (yon _choose_parent'in
        # tersi: yeni dugumden adaya). Alt sinir bile adayin mevcut cost'unu
        # iyilestirmiyorsa yeniden baglama imkansiz; _try_connect atlaniyor.
        if not (nodes[new_index].cost
                + model.lower_bound(
                    _euclid(nodes[new_index].pose, nodes[j].pose))
                < nodes[j].cost):
            continue
        edge = _try_connect(env, nodes[new_index].pose, nodes[j].pose, rho,
                            gamma_max, step, refine_steps)
        if edge is None:
            continue
        cost = model.of_edge(edge) + nodes[new_index].cost
        if cost >= nodes[j].cost:
            continue
        nodes[j] = dataclasses.replace(nodes[j], parent=new_index, cost=cost,
                                       path_from_parent=edge)
        _propagate_cost(nodes, j, model)
    
    
def plan3d(start: Pose3, goal: Pose3, env: Environment3D, rho: float,
           gamma_max: float, max_iterations: int = 5000,
           goal_bias: float = 0.05, step: float | None = None,
           rng: random.Random | None = None,
           max_edge_length: float | None = None,
           stop_on_first_solution: bool = False,
           radius_gamma: float | None = None,
           radius_cap: float | None = None,
           refine_steps: int = 0,
           model: CostModel = LENGTH) -> RRTResult3:
    """start'tan goal'a carpismasiz bir Dubins airplane rotasi arar (3B RRT*).
    """
    if rho <= 0.0:
        raise ValueError(f"rho pozitif olmali: {rho}")
    if gamma_max <= 0.0 or gamma_max >= math.pi / 2:
        raise ValueError(f"gamma_max (0, pi/2) araliginda olmali: {gamma_max}")
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
    if radius_gamma is not None and radius_gamma <= 0.0:
        raise ValueError(f"radius_gamma pozitif olmali: {radius_gamma}")
    if radius_cap is not None and radius_cap <= 0.0:
        raise ValueError(f"radius_cap pozitif olmali: {radius_cap}")

    auto_gamma, auto_cap = _auto_radius(env.bounds, rho)
    if radius_gamma is None:
        radius_gamma = auto_gamma
    if radius_cap is None:
        radius_cap = auto_cap
    
    if max_edge_length is not None and max_edge_length <= 0.0:
        raise ValueError(f"max_edge_length pozitif olmali: {max_edge_length}")
    if refine_steps < 0:
        raise ValueError(f"refine_steps negatif olamaz: {refine_steps}")

    if step is None:
        step = env.suggested_step()
    if rng is None:
        rng = random.Random()

    nodes = [Node3(start, None, 0.0, None)]
    goal_links = []          # (dugum indeksi, hedefe giden kenar)

    for iteration in range(1, max_iterations + 1):
        target = _sample(env, goal, rng, goal_bias)
        i = _nearest(nodes, target, rho, gamma_max)
        # adimli ilerleme: uzak ornege tam yol yerine ara poza gidilir
        target = _steer(nodes[i].pose, target, rho, gamma_max,
                        max_edge_length, refine_steps)
        edge = _try_connect(env, nodes[i].pose, target, rho, gamma_max, step,
                            refine_steps)
        if edge is None:
            continue

        # komsular eklemeden once bulunuyor: indeksler gecerli kalsin ve
        # yeni dugum kendi komsusu olmasin
        radius = _neighbour_radius(len(nodes), radius_gamma, radius_cap)
        cands = _neighbours(nodes, target, radius)
        i, edge = _choose_parent(env, nodes, target, cands, rho, gamma_max,
                                 step, (i, edge), refine_steps, model)

        nodes.append(Node3(target, i, nodes[i].cost + model.of_edge(edge),
                           edge))
        _rewire(env, nodes, len(nodes) - 1, cands, rho, gamma_max, step,
                refine_steps, model)

        goal_edge = _try_connect(env, target, goal, rho, gamma_max, step,
                                 refine_steps)
        if goal_edge is None:
            continue

        goal_links.append((len(nodes) - 1, goal_edge))
        if stop_on_first_solution:
            edges = _extract_path(nodes, len(nodes) - 1, goal_edge)
            cost = nodes[-1].cost + model.of_edge(goal_edge)
            return RRTResult3(True, edges, cost, iteration, nodes)

    if not goal_links:
        return RRTResult3(False, [], math.inf, max_iterations, nodes)

    index, goal_edge = min(
        goal_links,
        key=lambda link: nodes[link[0]].cost + model.of_edge(link[1]))
    edges = _extract_path(nodes, index, goal_edge)
    cost = nodes[index].cost + model.of_edge(goal_edge)
    return RRTResult3(True, edges, cost, max_iterations, nodes)


def shortcut(edges: list[DubinsPath3D], env: Environment3D, rho: float,
             gamma_max: float, step: float, refine_steps: int = 0,
             max_rounds: int = 10,
             model: CostModel = LENGTH) -> list[DubinsPath3D]:
    """shortcut_with'in 3B kolayligi; baglantiyi ortamdan kuruyor.

    Algoritma 2B ile ortak: tek kopya olsun diye rrt_star.shortcut_with'te
    duruyor ve baglanti islevi disaridan veriliyor.
    """
    return shortcut_with(
        edges,
        lambda a, b: _try_connect(env, a, b, rho, gamma_max, step,
                                  refine_steps),
        max_rounds, model)

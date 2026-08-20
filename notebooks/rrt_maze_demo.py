"""RRT ve RRT* farkinin gorundugu yilankavi koridor demosu.

notebooks/rrt_demo.py'deki harita acik; koridorlar genis oldugu icin duz
RRT'nin ilk cozumu zaten optimale yakin cikiyor ve RRT*'in kazanci kucuk
kaliyor. Burada uc duvar ve donusumlu gecitler var: rota zorunlu bir S
cizmek zorunda, ilk bulunan dugum zinciri yalpaliyor ve yeniden baglamanin
duzelttigi sey gozle gorulur oluyor.

Sekil tek bir tohumu cizer, konsol ciktisi bes tohumun tamamini verir -
tek kosuya bakip genelleme yapmamak icin.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt_maze_demo
"""

import math
import random

import matplotlib.pyplot as plt

from src.environment import Environment, Obstacle
from src.rrt_star import plan

from notebooks.rrt_demo import draw_environment, draw_pose

BOUNDS = (0.0, 0.0, 120.0, 120.0)
CLEARANCE = 1.5
RHO = 4.0
START = (10.0, 8.0, 0.0)
GOAL = (12.0, 112.0, math.pi)

PLAIN_BUDGET = 4000       # ilk cozumde donecegi icin cogu harcanmaz
STAR_BUDGET = 1500
SEEDS = (1, 2, 3, 4, 5)
DRAWN_SEED = 2

# Harita 120x120; varsayilan 60/30 100x60 icin ayarliydi, buraya buyutuldu.
RADIUS_GAMMA = 90.0
RADIUS_CAP = 45.0

WALL_RADIUS = 6.0
WALL_SPACING = 7.0        # yaricaptan kucuk: daireler ust uste binip duvar olur


def wall(y, gap_lo, gap_hi):
    """y yuksekliginde, [gap_lo, gap_hi] araligi acik kalan dolu duvar."""
    out = []
    x = -WALL_RADIUS
    while x <= BOUNDS[2] + WALL_RADIUS:
        if not (gap_lo <= x <= gap_hi):
            out.append(Obstacle(x, y, WALL_RADIUS))
        x += WALL_SPACING
    return out


OBSTACLES = tuple(
    wall(30.0, 95.0, 130.0)       # gecit sagda
    + wall(60.0, -10.0, 25.0)     # gecit solda
    + wall(90.0, 95.0, 130.0)     # gecit sagda
)


def run(env, seed, star):
    """Tek kosu: star=False ilk cozumde durur, star=True butceyi harcar."""
    if star:
        return plan(START, GOAL, env, RHO, max_iterations=STAR_BUDGET,
                    rng=random.Random(seed), radius_gamma=RADIUS_GAMMA,
                    radius_cap=RADIUS_CAP)
    return plan(START, GOAL, env, RHO, max_iterations=PLAIN_BUDGET,
                rng=random.Random(seed), stop_on_first_solution=True)


def draw_run(ax, env, result, title):
    """Bir kosunun agacini ve rotasini verilen eksene cizer."""
    draw_environment(ax, env)

    for node in result.tree:
        if node.path_from_parent is None:      # kok
            continue
        pts = node.path_from_parent.sample(0.6)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color="lightgray", linewidth=0.5, zorder=2)

    for edge in result.edges:
        pts = edge.sample(0.2)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color="darkgreen", linewidth=2.5, zorder=4)

    draw_pose(ax, START, "royalblue", "baslangic")
    draw_pose(ax, GOAL, "red", "hedef")

    cost_text = f"{result.cost:.1f} m" if result.found else "-"
    ax.set_title(f"{title}\n"
                 f"dugum={len(result.tree)}, uzunluk={cost_text}")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower right")


def main():
    env = Environment(BOUNDS, OBSTACLES, CLEARANCE)

    print(f"{'tohum':>5} {'RRT':>8} {'RRT*':>8} {'kisalma':>8}")
    plain_total = star_total = 0.0
    drawn = {}
    for seed in SEEDS:
        plain = run(env, seed, star=False)
        star = run(env, seed, star=True)
        if seed == DRAWN_SEED:
            drawn = {"plain": plain, "star": star}
        if not (plain.found and star.found):
            print(f"{seed:5d}   rota bulunamadi")
            continue
        plain_total += plain.cost
        star_total += star.cost
        gain = 100 * (plain.cost - star.cost) / plain.cost
        print(f"{seed:5d} {plain.cost:8.1f} {star.cost:8.1f} {gain:7.1f}%")

    mean_gain = 100 * (plain_total - star_total) / plain_total
    print(f"ortalama kisalma: %{mean_gain:.1f}")

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(18, 10))
    draw_run(ax_left, env, drawn["plain"], "Duz RRT - ilk cozumde durur")
    draw_run(ax_right, env, drawn["star"], f"RRT* - {STAR_BUDGET} yineleme")

    fig.suptitle(f"Yilankavi koridor: RRT ve RRT*   "
                 f"(rho={RHO} m, cizilen tohum={DRAWN_SEED}, "
                 f"{len(SEEDS)} tohumda ortalama kisalma %{mean_gain:.1f})",
                 fontsize=15, y=0.99)
    # rect: ustte suptitle, altta eksen etiketi icin yer birak
    plt.tight_layout(rect=(0.0, 0.02, 1.0, 0.93))
    plt.savefig("results/rrt_maze_demo.png", dpi=150)
    plt.close()
    print("kaydedildi: results/rrt_maze_demo.png")


if __name__ == "__main__":
    main()

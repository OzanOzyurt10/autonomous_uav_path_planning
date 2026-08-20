"""RRT ve RRT* karsilastirmali gorsellestirme demosu.

Ayni tohum, ayni butce, ayni harita ile iki kosu yan yana cizilir: solda ilk
cozumde duran duz RRT, sagda butceyi harcayan RRT*. Agac acik gri, rota kalin
yesil. Agacin engel iclerinde dali olmamali - varsa carpisma kontrolu bozuk.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt_demo
"""

import math
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from src.environment import Environment, Obstacle
from src.rrt_star import plan

BOUNDS = (0.0, 0.0, 150.0, 150.0)
CLEARANCE = 2.0
RHO = 6.0
START = (5.0, 5.0, math.radians(45))
GOAL = (105.0, 85.0, 0.0)
MAX_ITERATIONS = 500
SEED = 1

# Varsayilan radius_gamma=60 / radius_cap=30 100x60 m harita icin ayarliydi.
# Bu harita 150x150, alani 3.75 kati; ayni komsu sayisi icin yaricap ~2 kat
# buyumeli. 120/60 ile rota 178.6 m -> 172.0 m; 180/90 ayni sonucu veriyor,
# yani doyuma ulasilmis.
RADIUS_GAMMA = 120.0
RADIUS_CAP = 60.0

OBSTACLES = (
    # -------------------------------------------------
    # 1. bariyer
    # Geçiş: sağ-orta
    # -------------------------------------------------
    Obstacle(0.0, 22.0, 10.0),
    Obstacle(42.0, 22.0, 10.0),
    Obstacle(84.0, 22.0, 10.0),
    Obstacle(126.0, 22.0, 10.0),

    # -------------------------------------------------
    # 2. bariyer
    # Geçiş: sol-orta
    # -------------------------------------------------
    Obstacle(0.0, 45.0, 10.0),
    Obstacle(42.0, 45.0, 10.0),
    Obstacle(84.0, 45.0, 10.0),
    Obstacle(126.0, 45.0, 10.0),

    # -------------------------------------------------
    # 3. bariyer
    # Geçiş: sağ-orta
    # -------------------------------------------------
    Obstacle(0.0, 68.0, 10.0),
    Obstacle(42.0, 68.0, 10.0),
    Obstacle(84.0, 68.0, 10.0),
    Obstacle(126.0, 68.0, 10.0),

    # -------------------------------------------------
    # 4. bariyer
    # Geçiş: sol-orta
    # -------------------------------------------------
    Obstacle(0.0, 88.0, 10.0),
    Obstacle(42.0, 88.0, 10.0),
    Obstacle(84.0, 88.0, 10.0),
    Obstacle(126.0, 88.0, 10.0),
)

ARROW_LEN = 3.0


def draw_pose(ax, pose, color, label=None):
    """Bir pozu ok olarak cizer: okun yeri konum, yonu yaw."""
    x, y, yaw = pose
    ax.arrow(x, y, ARROW_LEN * math.cos(yaw), ARROW_LEN * math.sin(yaw),
             head_width=1.5, color=color, zorder=6, alpha=0.9)
    ax.plot(x, y, "o", color=color, markersize=6, label=label, zorder=6)


def draw_environment(ax, env):
    """Engelleri ve emniyet paylarini cizer."""
    for obs in env.obstacles:
        ax.add_patch(Circle((obs.x, obs.y), obs.radius,
                            color="dimgray", alpha=0.7, zorder=3))
        ax.add_patch(Circle((obs.x, obs.y), obs.radius + env.clearance,
                            fill=False, linestyle="--", color="gray",
                            linewidth=1.2, zorder=3))

    x_min, y_min, x_max, y_max = env.bounds
    ax.set_xlim(x_min - 5, x_max + 5)
    ax.set_ylim(y_min - 5, y_max + 5)
    ax.plot([x_min, x_max, x_max, x_min, x_min],
            [y_min, y_min, y_max, y_max, y_min],
            color="black", linewidth=1.0, zorder=1)


def draw_run(ax, env, result, title):
    """Bir kosunun agacini ve rotasini verilen eksene cizer."""
    draw_environment(ax, env)

    # agac: her dugumun ebeveyninden gelen kenari. Kokun kenari yok.
    for node in result.tree:
        if node.path_from_parent is None:
            continue
        pts = node.path_from_parent.sample(0.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color="lightgray", linewidth=0.6, zorder=2)

    # rota: bulunamadiysa edges bos, dongu hic donmez
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
    ax.legend(loc="upper left")


def main():
    env = Environment(BOUNDS, OBSTACLES, CLEARANCE)

    # Ayni tohum: iki kosu ayni ornekleri goruyor, fark yalnizca algoritmadan.
    plain = plan(START, GOAL, env, RHO, max_iterations=MAX_ITERATIONS,
                 rng=random.Random(SEED), stop_on_first_solution=True)
    star = plan(START, GOAL, env, RHO, max_iterations=MAX_ITERATIONS,
                rng=random.Random(SEED), radius_gamma=RADIUS_GAMMA,
                radius_cap=RADIUS_CAP)

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(20, 10))
    draw_run(ax_left, env, plain, "Duz RRT - ilk cozumde durur")
    draw_run(ax_right, env, star, f"RRT* - {MAX_ITERATIONS} yineleme")

    gain = 100 * (plain.cost - star.cost) / plain.cost
    fig.suptitle(f"Dubins rota planlama: RRT ve RRT*   "
                 f"(rho={RHO} m, tohum={SEED}, kisalma %{gain:.1f})",
                 fontsize=15, y=0.99)

    # rect ust kenari 0.93: suptitle ile panel basliklari cakismasin
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    plt.savefig("results/rrt_demo.png", dpi=150)
    plt.close()
    print(f"kaydedildi: results/rrt_demo.png  "
          f"duz RRT {plain.cost:.1f} m ({len(plain.tree)} dugum) -> "
          f"RRT* {star.cost:.1f} m ({len(star.tree)} dugum), "
          f"kisalma %{gain:.1f}")


if __name__ == "__main__":
    main()

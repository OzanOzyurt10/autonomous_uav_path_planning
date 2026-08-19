"""RRT planlayici gorsellestirme demosu.

Engelli bir haritada rota planlar; agacin tamamini acik gri, bulunan rotayi
kalin kirmizi cizer. Agacin engel iclerinde dali olmamali - varsa carpisma
kontrolu bozuk demektir.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt_demo
"""

import math
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from src.environment import Environment, Obstacle
from src.rrt_star import plan

BOUNDS = (0.0, 0.0, 100.0, 60.0)
CLEARANCE = 2.0
RHO = 4.0
START = (5.0, 5.0, math.radians(45))
GOAL = (95.0, 55.0, 0.0)
MAX_ITERATIONS = 500
SEED = 1

OBSTACLES = (
    Obstacle(50.0, 30.0, 13.0),   # dogrudan yolu kesen orta engel
    Obstacle(25.0, 45.0, 8.0),
    Obstacle(72.0, 15.0, 10.0),
    Obstacle(80.0, 48.0, 6.0),
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


def main():
    env = Environment(BOUNDS, OBSTACLES, CLEARANCE)
    # stop_on_first_solution=False: rota ilk bulundugunda donmek yerine
    # butce boyunca agaci buyutmeye devam eder. Duz RRT rotayi iyilestirmez,
    # ama agacin nereye yayildigini gormek icin demoda faydali.
    result = plan(START, GOAL, env, RHO, max_iterations=MAX_ITERATIONS,
                  rng=random.Random(SEED), stop_on_first_solution=False)

    fig, ax = plt.subplots(figsize=(12, 8))
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
                color="crimson", linewidth=2.5, zorder=4)

    draw_pose(ax, START, "royalblue", "baslangic")
    draw_pose(ax, GOAL, "seagreen", "hedef")

    cost_text = f"{result.cost:.1f} m" if result.found else "-"
    ax.set_title(
        f"RRT - Dubins rota planlama   "
        f"(bulundu={result.found}, iterasyon={result.iterations}, "
        f"dugum={len(result.tree)}, uzunluk={cost_text}, rho={RHO} m)")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left")

    plt.tight_layout()
    plt.savefig("results/rrt_demo.png", dpi=150)
    plt.close()
    print(f"kaydedildi: results/rrt_demo.png  "
          f"(bulundu={result.found}, iterasyon={result.iterations}, "
          f"dugum={len(result.tree)}, uzunluk={cost_text})")


if __name__ == "__main__":
    main()

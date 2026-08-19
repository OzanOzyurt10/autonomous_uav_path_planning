"""Carpisma kontrolu gorsellestirme demosu.

Engelli bir haritada sabit bir baslangictan rastgele hedeflere Dubins
yollari uretir, her birini env.is_path_free ile sinar ve sonuca gore
yesil (serbest) veya kirmizi (carpiyor) cizer.

Bu betik hem dubins hem environment modulunu import eder; bagimsizlik
kurali src/environment.py icin gecerli, onu tuketen betikler icin degil.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.environment_demo
"""

import math
import random

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from src.dubins import shortest_path
from src.environment import Environment, Obstacle

BOUNDS = (0.0, 0.0, 100.0, 60.0)
CLEARANCE = 2.0
RHO = 8.0                              # minimum donus yaricapi, metre
START = (5.0, 5.0, math.radians(45))   # sabit baslangic pozu
N_GOALS = 8
SEED = 3                               # tohumlanmis: sekil her seferinde ayni

OBSTACLES = (
    Obstacle(30.0, 40.0, 10.0),
    Obstacle(55.0, 18.0, 8.0),
    Obstacle(75.0, 45.0, 12.0),
    Obstacle(88.0, 12.0, 6.0),
)


ARROW_LEN = 3.0     # heading okunun uzunlugu, metre (sadece gosterim)


def draw_pose(ax, pose, color, label=None):
    """Bir pozu ok olarak cizer: okun yeri konum, yonu yaw.

    Ok yolun parcasi degil, sadece heading gostergesi. Kisa tutuluyor ki
    engellerin icine giriyormus gibi gorunmesin.
    """
    x, y, yaw = pose
    ax.arrow(x, y, ARROW_LEN * math.cos(yaw), ARROW_LEN * math.sin(yaw),
             head_width=1.5, color=color, zorder=5, alpha=0.9)
    ax.plot(x, y, "o", color=color, markersize=5, label=label, zorder=5)


def draw_environment(ax, env):
    """Engelleri ve emniyet paylarini cizer."""
    for obs in env.obstacles:
        # gercek engel: dolu gri
        ax.add_patch(Circle((obs.x, obs.y), obs.radius,
                            color="dimgray", alpha=0.7, zorder=2))
        # emniyet payi: kesikli halka, ici bos
        ax.add_patch(Circle((obs.x, obs.y), obs.radius + env.clearance,
                            fill=False, linestyle="--", color="gray",
                            linewidth=1.2, zorder=2))

    x_min, y_min, x_max, y_max = env.bounds
    ax.set_xlim(x_min - 5, x_max + 5)
    ax.set_ylim(y_min - 5, y_max + 5)
    ax.plot([x_min, x_max, x_max, x_min, x_min],
            [y_min, y_min, y_max, y_max, y_min],
            color="black", linewidth=1.0, zorder=1)


def main():
    env = Environment(BOUNDS, OBSTACLES, CLEARANCE)
    rng = random.Random(SEED)
    step = env.suggested_step()

    fig, ax = plt.subplots(figsize=(12, 8))
    draw_environment(ax, env)

    free_count = 0
    labelled = set()   # legend'da her renk yalnizca bir kez gorunsun

    for _ in range(N_GOALS):
        goal = env.random_free_pose(rng)
        path = shortest_path(START, goal, RHO)

        # Carpisma kontrolu suggested_step ile yapilir; cizim daha ince
        # ornekelenir. Ikisini ayirmak onemli: yol gercekte duzgun bir egri,
        # kontrol ise sadece nokta nokta bakiyor.
        check_points = path.sample(step)
        is_free = env.is_path_free(check_points)
        free_count += is_free

        color = "seagreen" if is_free else "crimson"
        label = None
        if color not in labelled:
            labelled.add(color)
            label = "serbest" if is_free else "carpiyor"

        smooth = path.sample(0.3)
        ax.plot([p[0] for p in smooth], [p[1] for p in smooth],
                color=color, linewidth=2, alpha=0.85, zorder=3, label=label)
        # kontrol noktalari: carpisma kontrolunun gercekte baktigi yerler
        ax.plot([p[0] for p in check_points], [p[1] for p in check_points],
                ".", color=color, markersize=3, alpha=0.7, zorder=4)
        draw_pose(ax, goal, color)

    draw_pose(ax, START, "royalblue", "baslangic")

    ax.set_title(
        f"Dubins yollari - carpisma kontrolu   "
        f"({free_count}/{N_GOALS} serbest, rho={RHO} m, "
        f"clearance={CLEARANCE} m, adim={step:.2f} m)")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_aspect("equal")   # bu olmadan daireler elips gorunur
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left")

    plt.tight_layout()
    plt.savefig("results/environment_demo.png", dpi=150)
    plt.close()
    print(f"kaydedildi: results/environment_demo.png  "
          f"({free_count}/{N_GOALS} yol serbest, adim={step:.2f} m)")


if __name__ == "__main__":
    main()

"""3B RRT* gorsellestirme demosu: silindir engeller arasindan Dubins airplane rotasi.

Uc panel: tam 3B perspektif, yandan gorunum (X-Z) ve yukaridan duzlestirilmis
(X-Y). Engellerin bir kismi tavana kadar cikiyor (etrafindan dolasmak sart),
bir kismi alcak (uzerinden uculuyor) - 3B planlamanin kazandirdigi sey bu.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo
"""

import math
import random

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from src.environment3d import Cylinder, Environment3D
from src.rrt_star3d import plan3d

BOUNDS = (0.0, 0.0, 0.0, 100.0, 100.0, 80.0)
CLEARANCE = 2.0
RHO = 6.0
GAMMA_MAX = math.radians(15.0)
START = (10.0, 10.0, 20.0, math.radians(45))
GOAL = (90.0, 75.0, 50.0, 0.0)
MAX_ITERATIONS = 1000
GOAL_BIAS = 0.1
SEED = 2                 # bu tohumda rota alcak engelin uzerinden geciyor

# Tavana kadar cikan kuleler: irtifa yardim etmez, etrafindan dolasilir
TALL = (
    Cylinder(30.0, 30.0, 9.0, 0.0, 80.0),
    Cylinder(62.0, 18.0, 8.0, 0.0, 80.0),
    Cylinder(22.0, 66.0, 8.0, 0.0, 80.0),
    Cylinder(72.0, 62.0, 9.0, 0.0, 80.0),
    Cylinder(48.0, 92.0, 7.0, 0.0, 80.0),
    Cylinder(88.0, 34.0, 7.0, 0.0, 80.0),
)

# Alcak engeller: rota bunlarin uzerinden geciyor
SHORT = (
    Cylinder(50.0, 50.0, 14.0, 0.0, 25.0),
    Cylinder(78.0, 12.0, 10.0, 0.0, 22.0),
)

TALL_COLOR = "dimgray"
SHORT_COLOR = "goldenrod"
ROUTE_COLOR = "crimson"
TREE_COLOR = "lightgray"


def _circle_points(cyl, n=40):
    """Silindirin cemberi uzerinde n nokta; numpy olmadan."""
    xs, ys = [], []
    for k in range(n + 1):
        angle = 2 * math.pi * k / n
        xs.append(cyl.x + cyl.radius * math.cos(angle))
        ys.append(cyl.y + cyl.radius * math.sin(angle))
    return xs, ys


def draw_cylinder_3d(ax, cyl, color):
    """Yan yuzeyi plot_surface ile, ust kapagi cizgiyle.

    plot_surface numpy dizisi istiyor; cember noktalari math ile uretiliyor,
    yalnizca cizim icin diziye ceviriliyor.
    """
    xs, ys = _circle_points(cyl)
    surface_x = np.array([xs, xs])
    surface_y = np.array([ys, ys])
    surface_z = np.array([[cyl.z_min] * len(xs), [cyl.z_max] * len(xs)])
    ax.plot_surface(surface_x, surface_y, surface_z,
                    color=color, alpha=0.45, linewidth=0, shade=True)
    ax.plot(xs, ys, [cyl.z_max] * len(xs), color=color, linewidth=1.0)


def draw_3d(ax, env, result):
    """a. Tam 3B perspektif."""
    for cyl in env.obstacles:
        draw_cylinder_3d(ax, cyl, TALL_COLOR if cyl.z_max > 40 else SHORT_COLOR)

    for node in result.tree:
        if node.path_from_parent is None:      # kok
            continue
        pts = node.path_from_parent.sample(3.0)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=TREE_COLOR, linewidth=0.4)

    for edge in result.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=ROUTE_COLOR, linewidth=2.5)

    ax.scatter(*START[:3], color="royalblue", s=60, label="baslangic")
    ax.scatter(*GOAL[:3], color="seagreen", s=60, label="hedef")

    x_min, y_min, z_min, x_max, y_max, z_max = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_zlabel("z (irtifa, m)")
    ax.set_title("a. Tam 3B perspektif")
    ax.legend(loc="upper left")


def draw_side(ax, env, result):
    """b. Yandan gorunum: X-Z duzlemi. Irtifa profili burada okunuyor.

    y ekseni yassitildigi icin farkli y'deki engeller ust uste biniyor.
    Tavana kadar cikanlar bu yuzden yalnizca cerceve olarak ciziliyor;
    rotanin onlarin "icinden" gecmesi projeksiyon yanilsamasi. Alcak
    engeller dolu ciziliyor - rotanin ustlerinden gectigi gercekten dogru.
    """
    for cyl in env.obstacles:
        tall = cyl.z_max > 40
        ax.add_patch(Rectangle(
            (cyl.x - cyl.radius, cyl.z_min), 2 * cyl.radius,
            cyl.z_max - cyl.z_min,
            fill=not tall, facecolor="none" if tall else SHORT_COLOR,
            edgecolor=TALL_COLOR if tall else SHORT_COLOR,
            alpha=0.8 if tall else 0.75, linestyle="--" if tall else "-",
            linewidth=1.0, zorder=2))

    for node in result.tree:
        if node.path_from_parent is None:
            continue
        pts = node.path_from_parent.sample(3.0)
        ax.plot([p[0] for p in pts], [p[2] for p in pts],
                color=TREE_COLOR, linewidth=0.4, alpha=0.5, zorder=1)

    for edge in result.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[2] for p in pts],
                color=ROUTE_COLOR, linewidth=2.5, zorder=4)

    ax.plot(START[0], START[2], "o", color="royalblue", markersize=8, zorder=5)
    ax.plot(GOAL[0], GOAL[2], "o", color="seagreen", markersize=8, zorder=5)

    x_min, _, z_min, x_max, _, z_max = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("z (irtifa, m)")
    # alcak engelin uzerinden gecisi isaretle: demonun asil noktasi
    low = min((c for c in env.obstacles if c.z_max <= 40), key=lambda c: c.x)
    over = [p[2] for edge in result.edges for p in edge.sample(0.5)
            if math.hypot(p[0] - low.x, p[1] - low.y) <= low.radius]
    if over:
        ax.annotate(f"alcak engelin ustunden\n"
                    f"tepe {low.z_max:.0f} m, rota {min(over):.0f}-{max(over):.0f} m",
                    xy=(low.x, sum(over) / len(over)), xytext=(12, 62),
                    fontsize=9, zorder=6,
                    arrowprops=dict(arrowstyle="->", color="black", lw=1.2))

    ax.set_title("b. Yandan gorunum (X-Z)   "
                 "[y yassitildi: yuksek engeller cerceve]")
    ax.grid(True, alpha=0.25)


def draw_top(ax, env, result):
    """c. Yukaridan duzlestirilmis: X-Y duzlemi."""
    for cyl in env.obstacles:
        color = TALL_COLOR if cyl.z_max > 40 else SHORT_COLOR
        ax.add_patch(Circle((cyl.x, cyl.y), cyl.radius,
                            color=color, zorder=3))
        ax.add_patch(Circle((cyl.x, cyl.y), cyl.radius + env.clearance,
                            fill=False, linestyle="--", color="gray",
                            linewidth=1.0, zorder=3))

    for node in result.tree:
        if node.path_from_parent is None:
            continue
        pts = node.path_from_parent.sample(3.0)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=TREE_COLOR, linewidth=0.4, zorder=1)

    for edge in result.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=ROUTE_COLOR, linewidth=2.5, zorder=4)

    ax.plot(START[0], START[1], "o", color="royalblue", markersize=8, zorder=5)
    ax.plot(GOAL[0], GOAL[1], "o", color="seagreen", markersize=8, zorder=5)

    x_min, y_min, _, x_max, y_max, _ = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_title("c. Yukaridan duzlestirilmis (X-Y)")
    ax.grid(True, alpha=0.25)


def flyover_report(env, result):
    """Rotanin alcak engellerin uzerinden hangi irtifayla gectigini olcer."""
    points = [p for edge in result.edges for p in edge.sample(0.5)]
    lines = []
    for cyl in env.obstacles:
        if cyl.z_max > 40:
            continue
        above = [p[2] for p in points
                 if math.hypot(p[0] - cyl.x, p[1] - cyl.y) <= cyl.radius]
        if above:
            lines.append(f"({cyl.x:.0f}, {cyl.y:.0f}) engeli tepesi {cyl.z_max:.0f} m; "
                         f"rota {min(above):.1f}-{max(above):.1f} m'den geciyor")
    return lines


def main():
    env = Environment3D(BOUNDS, TALL + SHORT, CLEARANCE)
    result = plan3d(START, GOAL, env, RHO, GAMMA_MAX,
                    max_iterations=MAX_ITERATIONS, goal_bias=GOAL_BIAS,
                    rng=random.Random(SEED))

    fig = plt.figure(figsize=(20, 9))
    ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax_side = fig.add_subplot(1, 3, 2)
    ax_top = fig.add_subplot(1, 3, 3)

    draw_3d(ax_3d, env, result)
    draw_side(ax_side, env, result)
    draw_top(ax_top, env, result)

    cost_text = f"{result.cost:.1f} m" if result.found else "-"
    fig.suptitle(
        f"3B Dubins RRT* rotasi   "
        f"(rho={RHO} m, gamma_max={math.degrees(GAMMA_MAX):.0f} derece, "
        f"dugum={len(result.tree)}, uzunluk={cost_text})   "
        f"gri: tavana kadar cikan engeller, sari: ustunden ucularak gecilen alcak engeller",
        fontsize=14)
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    plt.savefig("results/rrt3d_demo.png", dpi=150)
    plt.close()

    print(f"bulundu={result.found}, dugum={len(result.tree)}, "
          f"kenar={len(result.edges)}, uzunluk={cost_text}")
    for line in flyover_report(env, result):
        print("  " + line)
    print("kaydedildi: results/rrt3d_demo.png")


if __name__ == "__main__":
    main()

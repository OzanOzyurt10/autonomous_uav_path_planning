"""3B RRT* gorsellestirme demosu: silindir engeller arasindan Dubins airplane rotasi.

Uc panel: tam 3B perspektif, yandan gorunum (X-Z) ve yukaridan duzlestirilmis
(X-Y). Engellerin bir kismi tavana kadar cikiyor (etrafindan dolasmak sart),
bir kismi alcak (uzerinden uculuyor) - 3B planlamanin kazandirdigi sey bu.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo
"""

import math
import random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from src.environment3d import Cylinder, Environment3D
from src.rrt_star3d import plan3d

BOUNDS = (0.0, 0.0, 0.0, 100.0, 100.0, 80.0)
CLEARANCE = 2.0
RHO = 6.0
GAMMA_MAX = math.radians(15.0)
START = (8.0, 50.0, 15.0, 0.0)          # doguya bakiyor
GOAL = (92.0, 50.0, 55.0, 0.0)
MAX_ITERATIONS = 2500                    # kapilar dar, biraz daha butce lazim
GOAL_BIAS = 0.10
MAX_EDGE_LENGTH = 30.0                   # dar kapida uzun kenar surekli carpiyor
REFINE_STEPS = 8
SEED = 1
SMOKE_SEEDS = (1, 2, 3, 4, 5, 6, 7)

# Duvar A (x=35): kapi y ~ 59-75 arasi acik.
# Duvar B (x=68): kapi y ~ 27-41 arasi acik. Kapilar kaydirildi -> caprazlama gecis.
TALL = (
    Cylinder(35.0,  4.0, 7.0, 0.0, 80.0),
    Cylinder(35.0, 16.0, 7.0, 0.0, 80.0),
    Cylinder(35.0, 28.0, 7.0, 0.0, 80.0),
    Cylinder(35.0, 40.0, 7.0, 0.0, 80.0),
    Cylinder(35.0, 52.0, 7.0, 0.0, 80.0),
    Cylinder(35.0, 82.0, 7.0, 0.0, 80.0),
    Cylinder(35.0, 94.0, 7.0, 0.0, 80.0),

    Cylinder(68.0,  8.0, 7.0, 0.0, 80.0),
    Cylinder(68.0, 20.0, 7.0, 0.0, 80.0),
    Cylinder(68.0, 48.0, 7.0, 0.0, 80.0),
    Cylinder(68.0, 60.0, 7.0, 0.0, 80.0),
    Cylinder(68.0, 72.0, 7.0, 0.0, 80.0),
    Cylinder(68.0, 84.0, 7.0, 0.0, 80.0),
    Cylinder(68.0, 96.0, 7.0, 0.0, 80.0),
)

# Alcak engeller: kapiya giden en kisa hatlarin uzerine oturuyor,
# etrafindan dolasmak yerine ustunden gecmek daha ucuz kaliyor.
SHORT = (
    Cylinder(20.0, 62.0, 11.0, 0.0, 24.0),   # A kapisina yaklasma hatti
    Cylinder(50.0, 50.0, 14.0, 0.0, 30.0),   # iki duvar arasi ortada
    Cylinder(82.0, 42.0, 12.0, 0.0, 28.0),   # B kapisi cikisi -> hedef
)

TALL_COLOR = "dimgray"
SHORT_COLOR = "goldenrod"
ROUTE_COLOR = "crimson"
WIDE_ROUTE_COLOR = "darkorange"
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


def _edge_color(edge):
    """Refine ile genis yaricapa gecen kenari farkli renkle goster."""
    if edge.horizontal.rho > RHO + 1e-9:
        return WIDE_ROUTE_COLOR
    return ROUTE_COLOR


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
                [p[2] for p in pts], color=_edge_color(edge), linewidth=3.0)

        floor = env.bounds[2]
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [floor] * len(pts), color=_edge_color(edge),
                linewidth=1.0, alpha=0.25)

    ax.scatter(*START[:3], color="royalblue", s=60, label="baslangic")
    ax.scatter(*GOAL[:3], color="seagreen", s=60, label="hedef")

    x_min, y_min, z_min, x_max, y_max, z_max = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    # z etiketi 3B eksenin sagina dusuyor ve yan panelin etiketiyle
    # cakisiyordu; eksen adlari baslikta veriliyor
    ax.set_zlabel("")
    ax.view_init(elev=28, azim=-55)
    ax.set_box_aspect((1.0, 1.0, 0.62))
    ax.set_title("a. Tam 3B perspektif   [dikey eksen: z, irtifa (m)]")
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
                color=_edge_color(edge), linewidth=2.8, zorder=4)

    ax.plot(START[0], START[2], "o", color="royalblue", markersize=8, zorder=5)
    ax.plot(GOAL[0], GOAL[2], "o", color="seagreen", markersize=8, zorder=5)

    x_min, _, z_min, x_max, _, z_max = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("z (irtifa, m)")
    # Alcak engelin uzerinden gecisi isaretle: demonun asil noktasi.
    flyovers = []
    for low in (c for c in env.obstacles if c.z_max <= 40):
        over = [p[2] for edge in result.edges for p in edge.sample(0.5)
                if math.hypot(p[0] - low.x, p[1] - low.y) <= low.radius]
        if over:
            flyovers.append((low, over))
    if flyovers:
        low, over = max(flyovers, key=lambda item: len(item[1]))
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
                color=_edge_color(edge), linewidth=2.8, zorder=4)

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


def refine_report(result):
    """Refine'in hangi kenarlarda genis yaricap sectigini ozetler."""
    lines = []
    for i, edge in enumerate(result.edges, start=1):
        if edge.horizontal.rho <= RHO + 1e-9:
            continue
        lines.append(f"{i}. kenar refine: rho {RHO:.1f} -> "
                     f"{edge.horizontal.rho:.1f} m, "
                     f"gamma={math.degrees(edge.gamma):.1f} derece")
    return lines


def smoke_report(env):
    """Ayni senaryo farkli tohumlarda en azindan rota buluyor mu?"""
    lines = []
    all_found = True
    for seed in SMOKE_SEEDS:
        result = plan3d(START, GOAL, env, RHO, GAMMA_MAX,
                        max_iterations=MAX_ITERATIONS,
                        goal_bias=GOAL_BIAS,
                        max_edge_length=MAX_EDGE_LENGTH,
                        refine_steps=REFINE_STEPS,
                        rng=random.Random(seed))
        all_found = all_found and result.found
        cost = f"{result.cost:.1f} m" if result.found else "-"
        lines.append(f"seed={seed}: bulundu={result.found}, "
                     f"dugum={len(result.tree)}, kenar={len(result.edges)}, "
                     f"uzunluk={cost}")
    return all_found, lines


def main():
    env = Environment3D(BOUNDS, TALL + SHORT, CLEARANCE)
    result = plan3d(START, GOAL, env, RHO, GAMMA_MAX,
                    max_iterations=MAX_ITERATIONS, goal_bias=GOAL_BIAS,
                    max_edge_length=MAX_EDGE_LENGTH,
                    refine_steps=REFINE_STEPS,
                    rng=random.Random(SEED))

    fig = plt.figure(figsize=(20, 10))
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
        f"refine={REFINE_STEPS}, dugum={len(result.tree)}, "
        f"uzunluk={cost_text})   "
        f"kirmizi: normal kenar, turuncu: genis yaricap refine",
        fontsize=14)
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94), w_pad=3.0)
    Path("results").mkdir(exist_ok=True)
    plt.savefig("results/rrt3d_demo.png", dpi=170)
    plt.close()

    print(f"bulundu={result.found}, dugum={len(result.tree)}, "
          f"kenar={len(result.edges)}, uzunluk={cost_text}")
    for line in flyover_report(env, result):
        print("  " + line)
    for line in refine_report(result):
        print("  " + line)
    all_found, lines = smoke_report(env)
    print(f"smoke: tum tohumlar bulundu={all_found}")
    for line in lines:
        print("  " + line)
    print("kaydedildi: results/rrt3d_demo.png")


if __name__ == "__main__":
    main()

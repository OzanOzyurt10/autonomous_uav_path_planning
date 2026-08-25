"""3B RRT* gorsellestirme demosu: silindir engeller arasindan Dubins airplane rotasi.

Uc panel: tam 3B perspektif, yandan gorunum (X-Z) ve yukaridan duzlestirilmis
(X-Y). Rota iki kez ciziliyor: kisaltma oncesi kesikli siyah, sonrasi kalin
kirmizi (refine ile genis yaricapa gecen kenarlar turuncu).

Iki harita var, SCENE ile secilir:
  kapili  - dar kapili duvarlar; alcak engellerin ustunden ucmak sart
  acik    - genis ve seyrek arazi; kisaltma burada kenar sayisini yariya indiriyor

Ucak kisitlari (RHO, GAMMA_MAX, CLEARANCE, REFINE_STEPS) iki harita icin de
ortak, asagida tek yerde. Haritalar yalnizca kendi engellerini, uc noktalarini
ve butcelerini tasiyor.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo acik
"""

import dataclasses
import math
import random
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle

from src.environment3d import Cylinder, Environment3D
from src.rrt_star3d import plan3d, shortcut

# ----------------------------- AYARLAR ---------------------------------
SCENE = "kapili"                 # "kapili" veya "acik"

RHO = 6.0                        # minimum donus yaricapi, m
GAMMA_MAX = math.radians(15.0)   # maksimum tirmanma/alcalma acisi
CLEARANCE = 2.0                  # engellere birakilan emniyet payi, m
REFINE_STEPS = 8                 # helis yerine genis yaricap denemesi (0 = kapali)

SEED = 1                         # cizilen kosunun tohumu
SMOKE_SEEDS = (1, 2, 3, 4, 5, 6, 7)
# -----------------------------------------------------------------------

TALL_COLOR = "dimgray"
SHORT_COLOR = "goldenrod"
ROUTE_COLOR = "crimson"
WIDE_ROUTE_COLOR = "darkorange"
TREE_COLOR = "lightgray"
BEFORE_COLOR = "black"           # kisaltma oncesi rota


@dataclasses.dataclass(frozen=True)
class Scene:
    """Haritaya ozgu her sey. Ucak kisitlari burada degil, modul basinda."""

    name: str
    bounds: tuple
    obstacles: tuple
    start: tuple
    goal: tuple
    max_iterations: int
    goal_bias: float
    max_edge_length: float | None
    tall_z: float                # bunun ustune cikan engel "yuksek" sayilir
    output: str


# --- harita 1: kapili duvarlar ---
# Duvar A (x=35): kapi y ~ 59-75 arasi acik.
# Duvar B (x=68): kapi y ~ 27-41 arasi acik. Kapilar kaydirildi -> caprazlama gecis.
GATED = Scene(
    name="kapili duvarlar",
    bounds=(0.0, 0.0, 0.0, 100.0, 100.0, 80.0),
    obstacles=(
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

        # Alcak engeller: kapiya giden en kisa hatlarin uzerine oturuyor,
        # etrafindan dolasmak yerine ustunden gecmek daha ucuz kaliyor.
        Cylinder(20.0, 62.0, 11.0, 0.0, 24.0),   # A kapisina yaklasma hatti
        Cylinder(50.0, 50.0, 14.0, 0.0, 30.0),   # iki duvar arasi ortada
        Cylinder(82.0, 42.0, 12.0, 0.0, 28.0),   # B kapisi cikisi -> hedef
    ),
    start=(8.0, 50.0, 15.0, 0.0),                # doguya bakiyor
    goal=(92.0, 50.0, 55.0, 0.0),
    max_iterations=2500,                          # kapilar dar, butce lazim
    goal_bias=0.10,
    max_edge_length=30.0,                         # dar kapida uzun kenar carpiyor
    tall_z=40.0,
    output="results/rrt3d_demo.png",
)

# --- harita 2: acik arazi ---
# Kapili haritanin tersi: rotayi zorlayan dar bir koridor yok. Kisaltmanin
# kazancinin harita yapisina baglilıgini olcmek icin var.
OPEN = Scene(
    name="acik arazi",
    bounds=(0.0, 0.0, 0.0, 160.0, 160.0, 60.0),
    obstacles=(
        # tavana kadar cikan seyrek kuleler
        Cylinder(48.0,  38.0, 13.0, 0.0, 60.0),
        Cylinder(100.0, 30.0, 11.0, 0.0, 60.0),
        Cylinder(75.0,  82.0, 15.0, 0.0, 60.0),
        Cylinder(30.0, 100.0, 12.0, 0.0, 60.0),
        Cylinder(125.0, 95.0, 13.0, 0.0, 60.0),
        Cylinder(62.0, 132.0, 11.0, 0.0, 60.0),
        Cylinder(118.0, 142.0, 12.0, 0.0, 60.0),

        # genis ama alcak tepeler: ustunden ucmak dolasmaktan ucuz
        Cylinder(40.0,  70.0, 18.0, 0.0, 20.0),
        Cylinder(95.0,  60.0, 20.0, 0.0, 24.0),
        Cylinder(140.0, 55.0, 16.0, 0.0, 22.0),
    ),
    start=(12.0, 12.0, 18.0, math.radians(45.0)),
    goal=(148.0, 148.0, 42.0, math.radians(45.0)),
    max_iterations=2500,
    goal_bias=0.10,
    max_edge_length=45.0,                         # acik alanda uzun kenar gecer
    tall_z=30.0,
    output="results/rrt3d_open_demo.png",
)

SCENES = {"kapili": GATED, "acik": OPEN}


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


def _obstacle_color(cyl, scene):
    return TALL_COLOR if cyl.z_max > scene.tall_z else SHORT_COLOR


def draw_3d(ax, scene, env, result, cut):
    """a. Tam 3B perspektif. result kisaltma oncesi, cut sonrasi."""
    for cyl in env.obstacles:
        draw_cylinder_3d(ax, cyl, _obstacle_color(cyl, scene))

    for node in result.tree:
        if node.path_from_parent is None:      # kok
            continue
        pts = node.path_from_parent.sample(3.0)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=TREE_COLOR, linewidth=0.4)

    for edge in result.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=BEFORE_COLOR, linewidth=1.4,
                linestyle="--", alpha=0.6)

    floor = env.bounds[2]
    for edge in cut.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=_edge_color(edge), linewidth=3.0)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [floor] * len(pts), color=_edge_color(edge),
                linewidth=1.0, alpha=0.25)

    ax.plot([], [], [], color=BEFORE_COLOR, linestyle="--", linewidth=1.4,
            label="kisaltma oncesi")
    ax.plot([], [], [], color=ROUTE_COLOR, linewidth=3.0,
            label="kisaltma sonrasi")
    ax.scatter(*scene.start[:3], color="royalblue", s=60, label="baslangic")
    ax.scatter(*scene.goal[:3], color="seagreen", s=60, label="hedef")

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
    span = x_max - x_min
    ax.set_box_aspect((1.0, (y_max - y_min) / span, (z_max - z_min) / span))
    ax.set_title("a. Tam 3B perspektif   [dikey eksen: z, irtifa (m)]")
    ax.legend(loc="upper left")


def draw_side(ax, scene, env, result, cut):
    """b. Yandan gorunum: X-Z duzlemi. Irtifa profili burada okunuyor.

    y ekseni yassitildigi icin farkli y'deki engeller ust uste biniyor.
    Tavana kadar cikanlar bu yuzden yalnizca cerceve olarak ciziliyor;
    rotanin onlarin "icinden" gecmesi projeksiyon yanilsamasi. Alcak
    engeller dolu ciziliyor - rotanin ustlerinden gectigi gercekten dogru.
    """
    for cyl in env.obstacles:
        tall = cyl.z_max > scene.tall_z
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
                color=BEFORE_COLOR, linewidth=1.3, linestyle="--",
                alpha=0.6, zorder=3)

    for edge in cut.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[2] for p in pts],
                color=_edge_color(edge), linewidth=2.8, zorder=4)

    ax.plot(scene.start[0], scene.start[2], "o", color="royalblue",
            markersize=8, zorder=5)
    ax.plot(scene.goal[0], scene.goal[2], "o", color="seagreen",
            markersize=8, zorder=5)

    x_min, _, z_min, x_max, _, z_max = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("z (irtifa, m)")
    # Alcak engelin uzerinden gecisi isaretle: demonun asil noktasi.
    flyovers = []
    for low in (c for c in env.obstacles if c.z_max <= scene.tall_z):
        over = [p[2] for edge in cut.edges for p in edge.sample(0.5)
                if math.hypot(p[0] - low.x, p[1] - low.y) <= low.radius]
        if over:
            flyovers.append((low, over))
    if flyovers:
        low, over = max(flyovers, key=lambda item: len(item[1]))
        ax.annotate(f"alcak engelin ustunden - tepe {low.z_max:.0f} m, "
                    f"rota {min(over):.0f}-{max(over):.0f} m",
                    xy=(low.x, sum(over) / len(over)),
                    xytext=(x_min + 0.08 * (x_max - x_min),
                            z_min + 0.82 * (z_max - z_min)),
                    fontsize=9, zorder=6,
                    arrowprops=dict(arrowstyle="->", color="black", lw=1.2))

    ax.set_title("b. Yandan gorunum (X-Z)   "
                 "[y yassitildi: yuksek engeller cerceve]")
    ax.grid(True, alpha=0.25)


def draw_top(ax, scene, env, result, cut):
    """c. Yukaridan duzlestirilmis: X-Y duzlemi."""
    for cyl in env.obstacles:
        ax.add_patch(Circle((cyl.x, cyl.y), cyl.radius,
                            color=_obstacle_color(cyl, scene), zorder=3))
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
                color=BEFORE_COLOR, linewidth=1.3, linestyle="--",
                alpha=0.6, zorder=3)

    for edge in cut.edges:
        pts = edge.sample(0.5)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=_edge_color(edge), linewidth=2.8, zorder=4)

    ax.plot(scene.start[0], scene.start[1], "o", color="royalblue",
            markersize=8, zorder=5)
    ax.plot(scene.goal[0], scene.goal[1], "o", color="seagreen",
            markersize=8, zorder=5)

    x_min, y_min, _, x_max, y_max, _ = env.bounds
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_title("c. Yukaridan duzlestirilmis (X-Y)")
    ax.grid(True, alpha=0.25)


def make_env(scene):
    return Environment3D(scene.bounds, scene.obstacles, CLEARANCE)


def plan_scene(scene, env, seed):
    return plan3d(scene.start, scene.goal, env, RHO, GAMMA_MAX,
                  max_iterations=scene.max_iterations,
                  goal_bias=scene.goal_bias,
                  max_edge_length=scene.max_edge_length,
                  refine_steps=REFINE_STEPS,
                  rng=random.Random(seed))


def cut_of(env, result):
    """Kisaltilmis rotayi ayni bicimde bir sonuc nesnesine sarar.

    step ve refine_steps rotayi ureten kosuyla ayni olmali; yoksa carpisma
    kontrolu iki farkli cozunurlukte yapilir.
    """
    edges = shortcut(result.edges, env, RHO, GAMMA_MAX,
                     env.suggested_step(), REFINE_STEPS)
    cost = sum(e.length for e in edges) if result.found else math.inf
    return dataclasses.replace(result, edges=edges, cost=cost)


def flyover_report(scene, env, edges):
    """Rotanin alcak engellerin uzerinden hangi irtifayla gectigini olcer."""
    points = [p for edge in edges for p in edge.sample(0.5)]
    lines = []
    for cyl in env.obstacles:
        if cyl.z_max > scene.tall_z:
            continue
        above = [p[2] for p in points
                 if math.hypot(p[0] - cyl.x, p[1] - cyl.y) <= cyl.radius]
        if above:
            lines.append(f"({cyl.x:.0f}, {cyl.y:.0f}) engeli tepesi "
                         f"{cyl.z_max:.0f} m; rota {min(above):.1f}-"
                         f"{max(above):.1f} m'den geciyor")
    return lines


def refine_report(edges):
    """Refine'in hangi kenarlarda genis yaricap sectigini ozetler."""
    lines = []
    for i, edge in enumerate(edges, start=1):
        if edge.horizontal.rho <= RHO + 1e-9:
            continue
        lines.append(f"{i}. kenar refine: rho {RHO:.1f} -> "
                     f"{edge.horizontal.rho:.1f} m, "
                     f"gamma={math.degrees(edge.gamma):.1f} derece")
    return lines


def smoke_report(scene, env):
    """Farkli tohumlarda rota bulunuyor mu, kisaltma ne kazandiriyor?"""
    lines = []
    all_found = True
    pairs = []
    for seed in SMOKE_SEEDS:
        result = plan_scene(scene, env, seed)
        all_found = all_found and result.found
        if not result.found:
            lines.append(f"seed={seed}: bulundu=False")
            continue
        cut = cut_of(env, result)
        pairs.append((result.cost, cut.cost))
        lines.append(f"seed={seed}: dugum={len(result.tree)}, "
                     f"kenar {len(result.edges)} -> {len(cut.edges)}, "
                     f"uzunluk {result.cost:.1f} -> {cut.cost:.1f} m "
                     f"({(cut.cost - result.cost) / result.cost * 100:+.1f}%)")
    if pairs:
        before = sum(a for a, _ in pairs) / len(pairs)
        after = sum(b for _, b in pairs) / len(pairs)
        lines.append(f"ORTALAMA {before:.1f} -> {after:.1f} m "
                     f"({(after - before) / before * 100:+.1f}%)")
    return all_found, lines


def run(scene):
    """Sahneyi planlar, kisaltir, uc panelli PNG kaydeder, raporu yazdirir."""
    env = make_env(scene)
    result = plan_scene(scene, env, SEED)
    cut = cut_of(env, result)

    fig = plt.figure(figsize=(20, 10))
    ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax_side = fig.add_subplot(1, 3, 2)
    ax_top = fig.add_subplot(1, 3, 3)

    draw_3d(ax_3d, scene, env, result, cut)
    draw_side(ax_side, scene, env, result, cut)
    draw_top(ax_top, scene, env, result, cut)

    before = f"{result.cost:.1f} m" if result.found else "-"
    after = f"{cut.cost:.1f} m" if result.found else "-"
    gain = (cut.cost - result.cost) / result.cost * 100 if result.found else 0.0
    fig.suptitle(
        f"3B Dubins RRT* - {scene.name}   "
        f"(rho={RHO} m, gamma_max={math.degrees(GAMMA_MAX):.0f} derece, "
        f"refine={REFINE_STEPS}, dugum={len(result.tree)})   "
        f"kisaltma {before} -> {after} ({gain:+.1f}%)   "
        f"kesikli siyah: oncesi, kirmizi/turuncu: sonrasi",
        fontsize=14)
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94), w_pad=3.0)
    Path("results").mkdir(exist_ok=True)
    plt.savefig(scene.output, dpi=170)
    plt.close()

    print(f"[{scene.name}] bulundu={result.found}, dugum={len(result.tree)}, "
          f"kenar {len(result.edges)} -> {len(cut.edges)}, "
          f"uzunluk {before} -> {after} ({gain:+.1f}%)")
    for line in flyover_report(scene, env, cut.edges):
        print("  " + line)
    for line in refine_report(cut.edges):
        print("  " + line)
    all_found, lines = smoke_report(scene, env)
    print(f"smoke: tum tohumlar bulundu={all_found}")
    for line in lines:
        print("  " + line)
    print(f"kaydedildi: {scene.output}")


def main():
    # komut satirindan verilen ad SCENE'i gecersiz kilar
    name = sys.argv[1] if len(sys.argv) > 1 else SCENE
    if name not in SCENES:
        raise SystemExit(f"bilinmeyen harita: {name}; "
                         f"secenekler: {', '.join(SCENES)}")
    run(SCENES[name])


if __name__ == "__main__":
    main()

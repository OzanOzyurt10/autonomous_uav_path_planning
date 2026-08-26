"""3B RRT* gorsellestirme demosu: silindir engeller arasindan Dubins airplane rotasi.

Uc panel: tam 3B perspektif, yandan gorunum (X-Z) ve yukaridan duzlestirilmis
(X-Y). Rota iki kez ciziliyor: kisaltma oncesi kesikli siyah, sonrasi kalin
kirmizi (refine ile genis yaricapa gecen kenarlar turuncu).

Uc harita var, SCENE ile ya da komut satirindan secilir:
  kapili  - dar kapili duvarlar; alcak engellerin ustunden ucmak sart
  acik    - genis ve seyrek arazi; kisaltma burada kenar sayisini yariya indiriyor
  arazi   - 10 x 10 km gercek olcek, yukseklik izgarasi zemin; paneller degisir

Oyuncak haritalarin ucak kisitlari (RHO, GAMMA_MAX, CLEARANCE) asagida tek
yerde; arazi sahnesi gercek bir sabit kanat IHA'nin degerlerini kullaniyor
(TERRAIN_* sabitleri). Haritalar yalnizca kendi engellerini, uc noktalarini
ve butcelerini tasiyor.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo acik
    ./venv/Scripts/python.exe -m notebooks.rrt3d_demo arazi
"""

import dataclasses
import math
import os
import random
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Rectangle
from matplotlib.ticker import MaxNLocator

from src.environment3d import Cylinder, Environment3D
from src.rrt_star3d import plan3d, shortcut
from src.terrain import Terrain, load_terrain, synthetic_terrain

# ----------------------------- AYARLAR ---------------------------------
SCENE = "kapili"                 # "kapili", "acik" veya "arazi"

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
    terrain: Terrain | None = None


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

# --- harita 3: gercek arazi olcegi ---
# Ucak kisitlari gercek bir sabit kanat IHA'ya gore: 28 m/s seyir, 30 derece
# yatis -> rho = v^2 / (g * tan) yaklasik 150 m. 4 m/s tirmanma -> 8 derece.
# Emniyet payi araziden 100 m; asgari AGL boyle bir gorevde bu mertebede.
TERRAIN_RHO = 150.0
TERRAIN_GAMMA_MAX = math.radians(8.0)
TERRAIN_CLEARANCE = 100.0

# Pencere: N46E014 karosunda 10 x 10 km (Karavanke / Drava cevresi).
# Postlar derecede esit arali, metrede degil: 46.3 N'de 3 yay-saniye
# guney-kuzeyde 92.8 m, dogu-batida 63.9 m. Terrain'in iki ayri spacing
# alani tutmasinin sebebi bu.
TERRAIN_FILE = "data/N46E014.hgl"
TERRAIN_LAT, TERRAIN_LON = 46.24, 14.5      # pencerenin guneybati kosesi
TERRAIN_WIDTH = TERRAIN_HEIGHT = 10000.0


def scene_terrain() -> Terrain:
    """Arazi sahnesinin yukseklik izgarasi.

    Karo depoda; yoksa ayni olculerde sentetik araziye dusuyor ki demo
    veri olmadan da calissin.
    """
    if os.path.exists(TERRAIN_FILE):
        return load_terrain(TERRAIN_FILE, TERRAIN_LAT, TERRAIN_LON,
                            TERRAIN_WIDTH, TERRAIN_HEIGHT)
    spacing = 111320.0 / 3600 * 3 * math.cos(math.radians(TERRAIN_LAT))
    return synthetic_terrain(157, 108, spacing, random.Random(11),
                             base=369.0, relief=1845.0, peaks=6)


def _alpine() -> Scene:
    terrain = scene_terrain()
    low, high = terrain.elevation_range()
    margin = 700.0
    return Scene(
        name="arazi 10 x 10 km",
        bounds=(0.0, 0.0, low, terrain.extent_x, terrain.extent_y,
                high + 400.0),
        obstacles=(),
        start=(margin, margin,
               terrain.elevation_at(margin, margin) + 250.0,
               math.radians(45.0)),
        goal=(terrain.extent_x - margin, terrain.extent_y - margin,
              terrain.elevation_at(terrain.extent_x - margin,
                                   terrain.extent_y - margin) + 250.0,
              math.radians(45.0)),
        max_iterations=4000,
        goal_bias=0.10,
        max_edge_length=1500.0,      # 10 x rho
        tall_z=1e9,                  # silindir yok, renk ayrimi anlamsiz
        output="results/rrt3d_terrain_demo.png",
        terrain=terrain,
    )


ALPINE = _alpine()

SCENES = {"kapili": GATED, "acik": OPEN, "arazi": ALPINE}


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


def _edge_color(edge, scene):
    """Refine ile genis yaricapa gecen kenari farkli renkle goster."""
    if edge.horizontal.rho > scene_rho(scene) + 1e-9:
        return WIDE_ROUTE_COLOR
    return ROUTE_COLOR


def _tree_step(scene):
    """Agac ornekleme araligi; harita olcegine gore.

    Sabit 3 m, 10 km'lik kenarlarda dugum basina yuzlerce nokta demek ve
    cizim dakikalar suruyor. Yaricapin dortte biri her olcekte yeterli.
    """
    return max(3.0, scene_rho(scene) / 4)


def _route_step(scene):
    return max(0.5, scene_rho(scene) / 40)


def _terrain_mesh(terrain, stride=1):
    """plot_surface / contour icin izgara. numpy yalniz cizimde kullaniliyor."""
    cols = range(0, terrain.cols, stride)
    rows = range(0, terrain.rows, stride)
    xs = np.array([c * terrain.spacing_x for c in cols])
    ys = np.array([r * terrain.spacing_y for r in rows])
    zs = np.array([[terrain.at(c, r) for c in cols] for r in rows])
    mesh_x, mesh_y = np.meshgrid(xs, ys)
    return mesh_x, mesh_y, zs


def _obstacle_color(cyl, scene):
    return TALL_COLOR if cyl.z_max > scene.tall_z else SHORT_COLOR


def draw_3d(ax, scene, env, result, cut):
    """a. Tam 3B perspektif. result kisaltma oncesi, cut sonrasi."""
    if scene.terrain is not None:
        mesh_x, mesh_y, mesh_z = _terrain_mesh(scene.terrain)
        ax.plot_surface(mesh_x, mesh_y, mesh_z, cmap="terrain",
                        linewidth=0, antialiased=False, alpha=0.9,
                        rstride=1, cstride=1)

    for cyl in env.obstacles:
        draw_cylinder_3d(ax, cyl, _obstacle_color(cyl, scene))

    for node in result.tree:
        if node.path_from_parent is None:      # kok
            continue
        pts = node.path_from_parent.sample(_tree_step(scene))
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=TREE_COLOR, linewidth=0.4)

    for edge in result.edges:
        pts = edge.sample(_route_step(scene))
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=BEFORE_COLOR, linewidth=1.4,
                linestyle="--", alpha=0.6)

    floor = env.bounds[2]
    for edge in cut.edges:
        pts = edge.sample(_route_step(scene))
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [p[2] for p in pts], color=_edge_color(edge, scene), linewidth=3.0)
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                [floor] * len(pts), color=_edge_color(edge, scene),
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
    true_z = (z_max - z_min) / span
    # 10 km'lik haritada 2.6 km'lik irtifa cok yassi kaliyor ve arazi
    # okunmuyor; dusey abarti arazi gorsellestirmesinde standart, ama
    # baslikta yazili olmali.
    shown_z = max(true_z, 0.45) if scene.terrain is not None else true_z
    ax.set_box_aspect((1.0, (y_max - y_min) / span, shown_z))
    ax.zaxis.set_major_locator(MaxNLocator(5))
    ax.tick_params(axis="z", labelsize=8, pad=0)
    note = ""
    if shown_z > true_z * 1.05:
        note = f"   dusey abarti {shown_z / true_z:.1f}x"
    ax.set_title(f"a. Tam 3B perspektif   [dikey eksen: irtifa (m)]{note}")
    ax.legend(loc="upper left", fontsize=8)


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
        pts = node.path_from_parent.sample(_tree_step(scene))
        ax.plot([p[0] for p in pts], [p[2] for p in pts],
                color=TREE_COLOR, linewidth=0.4, alpha=0.5, zorder=1)

    for edge in result.edges:
        pts = edge.sample(_route_step(scene))
        ax.plot([p[0] for p in pts], [p[2] for p in pts],
                color=BEFORE_COLOR, linewidth=1.3, linestyle="--",
                alpha=0.6, zorder=3)

    for edge in cut.edges:
        pts = edge.sample(_route_step(scene))
        ax.plot([p[0] for p in pts], [p[2] for p in pts],
                color=_edge_color(edge, scene), linewidth=2.8, zorder=4)

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
        over = [p[2] for edge in cut.edges for p in edge.sample(_route_step(scene))
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
    if scene.terrain is not None:
        mesh_x, mesh_y, mesh_z = _terrain_mesh(scene.terrain)
        ax.contourf(mesh_x, mesh_y, mesh_z, levels=18, cmap="terrain",
                    zorder=0)
        ax.contour(mesh_x, mesh_y, mesh_z, levels=18, colors="black",
                   linewidths=0.3, alpha=0.4, zorder=1)

    for cyl in env.obstacles:
        ax.add_patch(Circle((cyl.x, cyl.y), cyl.radius,
                            color=_obstacle_color(cyl, scene), zorder=3))
        ax.add_patch(Circle((cyl.x, cyl.y), cyl.radius + env.clearance,
                            fill=False, linestyle="--", color="gray",
                            linewidth=1.0, zorder=3))

    # Konturun uzerine binen agac okunakligi bozuyor; arazi panelinde
    # daha soluk ve ince ciziliyor.
    tree_alpha = 0.35 if scene.terrain is not None else 1.0
    for node in result.tree:
        if node.path_from_parent is None:
            continue
        pts = node.path_from_parent.sample(_tree_step(scene))
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=TREE_COLOR, linewidth=0.3, alpha=tree_alpha, zorder=2)

    for edge in result.edges:
        pts = edge.sample(_route_step(scene))
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=BEFORE_COLOR, linewidth=1.3, linestyle="--",
                alpha=0.6, zorder=3)

    for edge in cut.edges:
        pts = edge.sample(_route_step(scene))
        ax.plot([p[0] for p in pts], [p[1] for p in pts],
                color=_edge_color(edge, scene), linewidth=2.8, zorder=4)

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


def draw_profile(ax, scene, cut):
    """b. Rota boyunca kesit: arazi, emniyet tabani ve ucus irtifasi.

    Arazi sahnesinde X-Z projeksiyonu okunmuyor; farkli y'deki sirtlar ust
    uste biniyor. Bu panel rotanin yer izini takip ediyor, arazi takibi
    ancak boyle gorunur hale geliyor.
    """
    terrain = scene.terrain
    step = min(terrain.spacing_x, terrain.spacing_y) / 2
    points = [p for edge in cut.edges for p in edge.sample(step)]
    if not points:
        return

    distances = [0.0]
    for previous, current in zip(points, points[1:]):
        distances.append(distances[-1] + math.hypot(current[0] - previous[0],
                                                    current[1] - previous[1]))
    ground = [terrain.elevation_at(p[0], p[1]) for p in points]
    altitude = [p[2] for p in points]
    clearance = scene_clearance(scene)
    floor = [g + clearance for g in ground]
    base = min(ground) - 100.0

    ax.fill_between(distances, base, ground, color="tan", alpha=0.9,
                    zorder=2, label="arazi")
    ax.plot(distances, floor, color="firebrick", linestyle="--",
            linewidth=1.2, zorder=3,
            label=f"emniyet tabani (+{clearance:.0f} m)")
    ax.plot(distances, altitude, color=ROUTE_COLOR, linewidth=2.6,
            zorder=4, label="rota")

    worst = min(range(len(points)), key=lambda i: altitude[i] - ground[i])
    ax.annotate(f"en dusuk AGL {altitude[worst] - ground[worst]:.0f} m",
                xy=(distances[worst], altitude[worst]),
                xytext=(0.04, 0.93), textcoords="axes fraction", fontsize=9,
                zorder=5,
                arrowprops=dict(arrowstyle="->", color="black", lw=1.2))

    ax.set_xlim(0.0, distances[-1])
    ax.set_ylim(base, max(max(altitude), max(ground)) + 200.0)
    ax.set_xlabel("rota boyunca yer mesafesi (m)")
    ax.set_ylabel("irtifa (m, deniz seviyesinden)")
    ax.set_title("b. Rota kesiti: arazi profili ve ucus irtifasi")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper right", fontsize=8)


def scene_rho(scene):
    """Arazi sahnesi gercek IHA olceginde; oyuncak haritalar ayri."""
    return TERRAIN_RHO if scene.terrain is not None else RHO


def scene_gamma_max(scene):
    return TERRAIN_GAMMA_MAX if scene.terrain is not None else GAMMA_MAX


def scene_clearance(scene):
    return TERRAIN_CLEARANCE if scene.terrain is not None else CLEARANCE


def make_env(scene):
    return Environment3D(scene.bounds, scene.obstacles,
                         scene_clearance(scene), scene.terrain)


def plan_scene(scene, env, seed):
    return plan3d(scene.start, scene.goal, env,
                  scene_rho(scene), scene_gamma_max(scene),
                  max_iterations=scene.max_iterations,
                  goal_bias=scene.goal_bias,
                  max_edge_length=scene.max_edge_length,
                  refine_steps=REFINE_STEPS,
                  rng=random.Random(seed))


def cut_of(scene, env, result):
    """Kisaltilmis rotayi ayni bicimde bir sonuc nesnesine sarar.

    step ve refine_steps rotayi ureten kosuyla ayni olmali; yoksa carpisma
    kontrolu iki farkli cozunurlukte yapilir.
    """
    edges = shortcut(result.edges, env, scene_rho(scene),
                     scene_gamma_max(scene), env.suggested_step(),
                     REFINE_STEPS)
    cost = sum(e.length for e in edges) if result.found else math.inf
    return dataclasses.replace(result, edges=edges, cost=cost)


def agl_report(scene, edges):
    """Rotanin araziden yuksekligi: asgari AGL emniyet payini tutuyor mu."""
    if scene.terrain is None or not edges:
        return []
    step = min(scene.terrain.spacing_x, scene.terrain.spacing_y) / 2
    agls = [p[2] - scene.terrain.elevation_at(p[0], p[1])
            for edge in edges for p in edge.sample(step)]
    return [f"AGL: en dusuk {min(agls):.0f} m, ortalama {sum(agls)/len(agls):.0f} m, "
            f"en yuksek {max(agls):.0f} m  (emniyet payi "
            f"{scene_clearance(scene):.0f} m)"]


def flyover_report(scene, env, edges):
    """Rotanin alcak engellerin uzerinden hangi irtifayla gectigini olcer."""
    points = [p for edge in edges for p in edge.sample(_route_step(scene))]
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


def refine_report(scene, edges):
    """Refine'in hangi kenarlarda genis yaricap sectigini ozetler."""
    rho = scene_rho(scene)
    lines = []
    for i, edge in enumerate(edges, start=1):
        if edge.horizontal.rho <= rho + 1e-9:
            continue
        lines.append(f"{i}. kenar refine: rho {rho:.1f} -> "
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
        cut = cut_of(scene, env, result)
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
    cut = cut_of(scene, env, result)

    fig = plt.figure(figsize=(20, 10))
    ax_3d = fig.add_subplot(1, 3, 1, projection="3d")
    ax_side = fig.add_subplot(1, 3, 2)
    ax_top = fig.add_subplot(1, 3, 3)

    draw_3d(ax_3d, scene, env, result, cut)
    # Arazi sahnesinde X-Z projeksiyonu okunmuyor: farkli y'deki sirtlar ust
    # uste biniyor. Onun yerine rota boyunca kesit ciziliyor.
    if scene.terrain is None:
        draw_side(ax_side, scene, env, result, cut)
    else:
        draw_profile(ax_side, scene, cut)
    draw_top(ax_top, scene, env, result, cut)

    before = f"{result.cost:.1f} m" if result.found else "-"
    after = f"{cut.cost:.1f} m" if result.found else "-"
    gain = (cut.cost - result.cost) / result.cost * 100 if result.found else 0.0
    fig.suptitle(
        f"3B Dubins RRT* - {scene.name}   "
        f"rho={scene_rho(scene):.0f} m, gamma_max="
        f"{math.degrees(scene_gamma_max(scene)):.0f} derece, "
        f"clearance={scene_clearance(scene):.0f} m, dugum={len(result.tree)}"
        f"   |   kisaltma {before} -> {after} ({gain:+.1f}%)",
        fontsize=13)
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94), w_pad=3.0)
    Path("results").mkdir(exist_ok=True)
    plt.savefig(scene.output, dpi=170)
    plt.close()

    print(f"[{scene.name}] bulundu={result.found}, dugum={len(result.tree)}, "
          f"kenar {len(result.edges)} -> {len(cut.edges)}, "
          f"uzunluk {before} -> {after} ({gain:+.1f}%)")
    for line in agl_report(scene, cut.edges):
        print("  " + line)
    for line in flyover_report(scene, env, cut.edges):
        print("  " + line)
    for line in refine_report(scene, cut.edges):
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

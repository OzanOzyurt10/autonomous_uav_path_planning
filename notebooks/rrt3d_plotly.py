"""3B rotayi tarayicida acilan, dondurulup yakinlastirilabilir HTML'e cevirir.

Tek dosyalik bir HTML uretir: tarayicida akici doner, uzerine gelince deger
gosterir, oldugu gibi paylasilabilir. Kisaltma oncesi rota kesikli gri olarak
ayri bir iz; efsaneden acilip kapanabiliyor.

Harita, ucus kisitlari ve planlayici ayarlari rrt3d_demo'dan aliniyor - PNG
ile HTML birbirinden ayrilmasin diye. Harita secimi de oradaki SCENE ile,
ya da komut satirindan.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt3d_plotly
    ./venv/Scripts/python.exe -m notebooks.rrt3d_plotly acik
"""

import math
import sys

import plotly.graph_objects as go

from notebooks.rrt3d_demo import (REFINE_STEPS, SCENE, SCENES, SEED, cut_of,
                                  make_env, plan_scene, scene_gamma_max,
                                  scene_rho)

TALL_COLOR = "dimgray"
SHORT_COLOR = "goldenrod"
ROUTE_COLOR = "crimson"
WIDE_ROUTE_COLOR = "darkorange"
TREE_COLOR = "lightgray"
BEFORE_COLOR = "dimgray"

TREE_STEP = 4.0        # agac kaba orneklenir; binlerce dal var
ROUTE_STEP = 0.4
SHOW_TREE = False       # False ise agac hic cizilmez
TREE_VISIBLE_AT_START = False   # listede durur ama acilista gizli


def cylinder_surface(cyl, color):
    """Silindirin yan yuzeyini tek bir Surface izi olarak verir."""
    n = 32
    angles = [2 * math.pi * k / n for k in range(n + 1)]
    xs = [cyl.x + cyl.radius * math.cos(a) for a in angles]
    ys = [cyl.y + cyl.radius * math.sin(a) for a in angles]
    return go.Surface(
        x=[xs, xs],
        y=[ys, ys],
        z=[[cyl.z_min] * len(xs), [cyl.z_max] * len(xs)],
        colorscale=[[0, color], [1, color]],
        surfacecolor=[[0] * len(xs), [0] * len(xs)],
        showscale=False, opacity=0.45, hoverinfo="skip",
        name=f"engel ({cyl.x:.0f}, {cyl.y:.0f})",
    )


def tree_trace(result):
    """Butun agaci TEK ize koyar; parcalar None ile ayrilir.

    Her dal icin ayri iz eklemek binlerce iz demek ve tarayiciyi kilitler.
    """
    xs, ys, zs = [], [], []
    for node in result.tree:
        if node.path_from_parent is None:      # kok
            continue
        for p in node.path_from_parent.sample(TREE_STEP):
            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])
        xs.append(None)
        ys.append(None)
        zs.append(None)
    return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                        line=dict(color=TREE_COLOR, width=1),
                        name="agac", hoverinfo="skip",
                        visible=True if TREE_VISIBLE_AT_START else "legendonly")


def before_trace(edges):
    """Kisaltma oncesi rota; tek iz, kesikli."""
    xs, ys, zs = [], [], []
    for edge in edges:
        for p in edge.sample(ROUTE_STEP):
            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])
        xs.append(None)
        ys.append(None)
        zs.append(None)
    return go.Scatter3d(x=xs, y=ys, z=zs, mode="lines",
                        line=dict(color=BEFORE_COLOR, width=3, dash="dash"),
                        name="kisaltma oncesi", hoverinfo="skip")


def terrain_surface(terrain):
    """Arazi yuzeyi; postlar dogrudan Surface izine veriliyor."""
    xs = [c * terrain.spacing_x for c in range(terrain.cols)]
    ys = [r * terrain.spacing_y for r in range(terrain.rows)]
    zs = [[terrain.at(c, r) for c in range(terrain.cols)]
          for r in range(terrain.rows)]
    return go.Surface(x=xs, y=ys, z=zs, colorscale="earth", showscale=False,
                      opacity=1.0, name="arazi",
                      hovertemplate="x %{x:.0f}<br>y %{y:.0f}"
                                    "<br>kot %{z:.0f} m<extra></extra>")


def route_traces(edges, rho):
    """Rotayi iki ize ayirir: normal kenarlar ve genis yaricapli (refine)."""
    groups = {ROUTE_COLOR: ([], [], []), WIDE_ROUTE_COLOR: ([], [], [])}
    for edge in edges:
        wide = edge.horizontal.rho > rho + 1e-9
        xs, ys, zs = groups[WIDE_ROUTE_COLOR if wide else ROUTE_COLOR]
        for p in edge.sample(ROUTE_STEP):
            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])
        xs.append(None)
        ys.append(None)
        zs.append(None)

    names = {ROUTE_COLOR: "kisaltilmis rota (normal kenar)",
             WIDE_ROUTE_COLOR: "kisaltilmis rota (genis yaricap)"}
    traces = []
    for color, (xs, ys, zs) in groups.items():
        if not xs:
            continue
        traces.append(go.Scatter3d(
            x=xs, y=ys, z=zs, mode="lines",
            line=dict(color=color, width=6), name=names[color],
            hovertemplate="x %{x:.1f}<br>y %{y:.1f}<br>z %{z:.1f}<extra></extra>"))
    return traces


def build(scene, output):
    env = make_env(scene)
    result = plan_scene(scene, env, SEED)
    cut = cut_of(scene, env, result)

    data = [cylinder_surface(c, TALL_COLOR if c.z_max > scene.tall_z
                             else SHORT_COLOR)
            for c in env.obstacles]
    if scene.terrain is not None:
        data.insert(0, terrain_surface(scene.terrain))
    if SHOW_TREE:
        data.append(tree_trace(result))
    if result.found:
        data.append(before_trace(result.edges))
    data.extend(route_traces(cut.edges, scene_rho(scene)))
    data.append(go.Scatter3d(
        x=[scene.start[0], scene.goal[0]],
        y=[scene.start[1], scene.goal[1]],
        z=[scene.start[2], scene.goal[2]],
        mode="markers+text", text=["baslangic", "hedef"],
        textposition="top center",
        marker=dict(size=6, color=["royalblue", "seagreen"]),
        name="uc noktalar"))

    x_min, y_min, z_min, x_max, y_max, z_max = env.bounds
    before = f"{result.cost:.1f}" if result.found else "-"
    after = f"{cut.cost:.1f}" if result.found else "-"
    gain = (cut.cost - result.cost) / result.cost * 100 if result.found else 0.0

    fig = go.Figure(data=data)
    fig.update_layout(
        title=(f"3B Dubins RRT* - {scene.name}   "
               f"rho={scene_rho(scene):.0f} m, gamma_max="
               f"{math.degrees(scene_gamma_max(scene)):.0f} derece, "
               f"refine={REFINE_STEPS}"
               f"   dugum={len(result.tree)}"
               f"   kisaltma {before} -> {after} m ({gain:+.1f}%)"),
        scene=dict(
            xaxis=dict(title="x (dogu, m)", range=[x_min, x_max]),
            yaxis=dict(title="y (kuzey, m)", range=[y_min, y_max]),
            zaxis=dict(title="z (irtifa, m)", range=[z_min, z_max]),
            # gercek oranlar korunuyor
            aspectmode="manual",
            aspectratio=dict(x=1.0, y=(y_max - y_min) / (x_max - x_min),
                             z=(z_max - z_min) / (x_max - x_min)),
        ),
        margin=dict(l=0, r=0, t=60, b=0),
    )

    # include_plotlyjs=True: dosya kendi kendine yetiyor, internet gerekmiyor
    fig.write_html(output, include_plotlyjs=True)
    print(f"[{scene.name}] bulundu={result.found}, dugum={len(result.tree)}, "
          f"kenar {len(result.edges)} -> {len(cut.edges)}, "
          f"uzunluk {before} -> {after} m ({gain:+.1f}%)")
    print(f"kaydedildi: {output}  (tarayicida ac)")


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else SCENE
    if name not in SCENES:
        raise SystemExit(f"bilinmeyen harita: {name}; "
                         f"secenekler: {', '.join(SCENES)}")
    build(SCENES[name], f"results/rrt3d_view_{name}.html")


if __name__ == "__main__":
    main()

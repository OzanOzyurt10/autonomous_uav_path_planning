"""3B rotayi tarayicida acilan, dondurulup yakinlastirilabilir HTML'e cevirir.

matplotlib penceresi (rrt3d_view.py) yalniz bu bilgisayarda acilir ve binlerce
dalda takilir. Bu betik tek dosyalik bir HTML uretir: tarayicida akici doner,
uzerine gelince deger gosterir, ve oldugu gibi paylasilabilir.

Harita, engeller ve planlayici ayarlari rrt3d_demo'dan aliniyor - iki dosya
birbirinden ayrilmasin diye.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.rrt3d_plotly
"""

import math
import random

import plotly.graph_objects as go

from src.environment3d import Environment3D
from src.rrt_star3d import plan3d

from notebooks.rrt3d_demo import (BOUNDS, CLEARANCE, GAMMA_MAX, GOAL,
                                  GOAL_BIAS, MAX_EDGE_LENGTH, MAX_ITERATIONS,
                                  REFINE_STEPS, RHO, SEED, SHORT, START, TALL)

OUTPUT = "results/rrt3d_view.html"

TALL_COLOR = "dimgray"
SHORT_COLOR = "goldenrod"
ROUTE_COLOR = "crimson"
WIDE_ROUTE_COLOR = "darkorange"
TREE_COLOR = "lightgray"

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


def route_traces(result):
    """Rotayi iki ize ayirir: normal kenarlar ve genis yaricapli (refine)."""
    groups = {ROUTE_COLOR: ([], [], []), WIDE_ROUTE_COLOR: ([], [], [])}
    for edge in result.edges:
        wide = edge.horizontal.rho > RHO + 1e-9
        xs, ys, zs = groups[WIDE_ROUTE_COLOR if wide else ROUTE_COLOR]
        for p in edge.sample(ROUTE_STEP):
            xs.append(p[0])
            ys.append(p[1])
            zs.append(p[2])
        xs.append(None)
        ys.append(None)
        zs.append(None)

    names = {ROUTE_COLOR: "rota (normal kenar)",
             WIDE_ROUTE_COLOR: "rota (genis yaricap)"}
    traces = []
    for color, (xs, ys, zs) in groups.items():
        if not xs:
            continue
        traces.append(go.Scatter3d(
            x=xs, y=ys, z=zs, mode="lines",
            line=dict(color=color, width=6), name=names[color],
            hovertemplate="x %{x:.1f}<br>y %{y:.1f}<br>z %{z:.1f}<extra></extra>"))
    return traces


def main():
    env = Environment3D(BOUNDS, TALL + SHORT, CLEARANCE)
    result = plan3d(START, GOAL, env, RHO, GAMMA_MAX,
                    max_iterations=MAX_ITERATIONS, goal_bias=GOAL_BIAS,
                    max_edge_length=MAX_EDGE_LENGTH,
                    refine_steps=REFINE_STEPS, rng=random.Random(SEED))

    data = [cylinder_surface(c, TALL_COLOR if c.z_max > 40 else SHORT_COLOR)
            for c in env.obstacles]
    if SHOW_TREE:
        data.append(tree_trace(result))
    data.extend(route_traces(result))
    data.append(go.Scatter3d(
        x=[START[0], GOAL[0]], y=[START[1], GOAL[1]], z=[START[2], GOAL[2]],
        mode="markers+text", text=["baslangic", "hedef"],
        textposition="top center",
        marker=dict(size=6, color=["royalblue", "seagreen"]),
        name="uc noktalar"))

    x_min, y_min, z_min, x_max, y_max, z_max = env.bounds
    cost = f"{result.cost:.1f} m" if result.found else "-"
    fig = go.Figure(data=data)
    fig.update_layout(
        title=(f"3B Dubins RRT*   rho={RHO} m, gamma_max="
               f"{math.degrees(GAMMA_MAX):.0f} derece, refine={REFINE_STEPS}"
               f"   dugum={len(result.tree)}, uzunluk={cost}"),
        scene=dict(
            xaxis=dict(title="x (dogu, m)", range=[x_min, x_max]),
            yaxis=dict(title="y (kuzey, m)", range=[y_min, y_max]),
            zaxis=dict(title="z (irtifa, m)", range=[z_min, z_max]),
            # gercek oranlar: harita 100 x 100 x 80
            aspectmode="manual",
            aspectratio=dict(x=1.0, y=1.0,
                             z=(z_max - z_min) / (x_max - x_min)),
        ),
        margin=dict(l=0, r=0, t=60, b=0),
    )

    # include_plotlyjs=True: dosya kendi kendine yetiyor, internet gerekmiyor
    fig.write_html(OUTPUT, include_plotlyjs=True)
    print(f"bulundu={result.found}, dugum={len(result.tree)}, "
          f"kenar={len(result.edges)}, uzunluk={cost}")
    print(f"kaydedildi: {OUTPUT}  (tarayicida ac)")


if __name__ == "__main__":
    main()

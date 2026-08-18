"""Dubins yolu gorsellestirme demosu.

Su an tek bir yolu (en kisasini) ciziyor. Yapilacak: alti kelimeyi de
2x3 izgarada yan yana gostermek.

Calistirma:
    ./venv/Scripts/python.exe notebooks/dubins_demo.py
"""

import math

import matplotlib.pyplot as plt

from src.dubins import shortest_path

# Iki poz ve donus yaricapi. Poz = (x, y, yaw); yaw RADYAN.
START = (0.0, 0.0, 0.0)              # orijinde, doguya bakiyor
GOAL = (10.0, 6.0, math.radians(90))  # (10, 6) noktasinda, kuzeye bakiyor
RHO = 3.0                             # minimum donus yaricapi, metre


def draw_pose(ax, pose, color, label):
    """Bir pozu ok olarak cizer: okun yeri konum, yonu yaw."""
    x, y, yaw = pose
    ax.arrow(x, y, math.cos(yaw), math.sin(yaw),
             head_width=0.4, color=color, length_includes_head=False)
    ax.plot(x, y, "o", color=color, label=label)


def main():
    path = shortest_path(START, GOAL, RHO)

    # sample() poz listesi verir; ciziim icin x'leri ve y'leri ayirmamiz gerek
    points = path.sample(0.05)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(xs, ys, color="steelblue", linewidth=2)
    draw_pose(ax, START, "seagreen", "baslangic")
    draw_pose(ax, GOAL, "crimson", "hedef")

    ax.set_title(f"{path.word} — {path.length:.2f} m  (rho={RHO})")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_aspect("equal")  # bu olmadan daireler elips gorunur
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    plt.savefig("results/dubins_shortest.png", dpi=150)
    print(f"kaydedildi: results/dubins_shortest.png  ({path.word}, {path.length:.2f} m)")


if __name__ == "__main__":
    main()

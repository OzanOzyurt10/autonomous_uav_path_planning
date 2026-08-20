"""Dubins airplane (3B) gorsellestirme demosu.

Iki panel: solda irtifanin mevcut yol boyunca kazanilabildigi durum, sagda
yolun yetmedigi ve baslangicta helis atilan durum. Her panelde yolun kendisi
mavi, yere izdusumu acik gri cizilir - helisin yatayda ne yaptigi izdusumde
gorunuyor.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.dubins3d_demo
"""

import math

import matplotlib.pyplot as plt

from src.dubins3d import airplane_path

RHO = 5.0
GAMMA_MAX = math.radians(15.0)
STEP = 0.5

# Solda: uzun yatay mesafe, az irtifa farki -> helis gerekmiyor
EASY_START = (0.0, 0.0, 0.0, 0.0)
EASY_GOAL = (100.0, 20.0, 10.0, math.radians(45))

# Sagda: kisa yatay mesafe, cok irtifa farki -> helis sart
HARD_START = (0.0, 0.0, 0.0, 0.0)
HARD_GOAL = (30.0, 0.0, 25.0, 0.0)


def draw_run(ax, path, title):
    """Bir 3B yolu ve yere izdusumunu verilen eksene cizer."""
    pts = path.sample(STEP)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]

    floor = min(zs) - 5.0        # izdusum duzlemi, yolun altinda
    ax.plot(xs, ys, [floor] * len(pts), color="lightgray", linewidth=1.0)
    ax.plot(xs, ys, zs, color="royalblue", linewidth=2.0)

    ax.scatter(*path.start[:3], color="seagreen", s=50, label="baslangic")
    ax.scatter(*path.end_pose()[:3], color="crimson", s=50, label="hedef")

    ax.set_title(f"{title}\n"
                 f"helis={path.helix_turns}, "
                 f"gamma={math.degrees(path.gamma):.1f} derece, "
                 f"uzunluk={path.length:.1f} m")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.set_zlabel("z (irtifa, m)")
    ax.legend(loc="upper left")


def main():
    easy = airplane_path(EASY_START, EASY_GOAL, RHO, GAMMA_MAX)
    hard = airplane_path(HARD_START, HARD_GOAL, RHO, GAMMA_MAX)

    fig = plt.figure(figsize=(16, 8))
    ax_left = fig.add_subplot(1, 2, 1, projection="3d")
    ax_right = fig.add_subplot(1, 2, 2, projection="3d")

    draw_run(ax_left, easy, "Irtifa yol boyunca kazanilabiliyor")
    draw_run(ax_right, hard, "Yol yetmiyor: helis eklendi")

    fig.suptitle(f"Dubins airplane   "
                 f"(rho={RHO} m, gamma_max={math.degrees(GAMMA_MAX):.0f} derece)",
                 fontsize=15)
    plt.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    plt.savefig("results/dubins3d_demo.png", dpi=150)
    plt.close()

    for name, path in (("solda", easy), ("sagda", hard)):
        print(f"{name}: helis={path.helix_turns}, "
              f"gamma={math.degrees(path.gamma):.2f} derece, "
              f"yatay={path.horizontal_length:.1f} m, "
              f"3B={path.length:.1f} m")
    print("kaydedildi: results/dubins3d_demo.png")


if __name__ == "__main__":
    main()

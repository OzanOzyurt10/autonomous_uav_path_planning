"""Dubins yolu gorsellestirme demosu.

Iki sekil uretir:
    results/dubins_shortest.png  - sadece en kisa yol
    results/dubins_words.png     - alti kelimenin 2x3 karsilastirmasi

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.dubins_demo
"""

import math

import matplotlib.pyplot as plt

from src.dubins import all_paths, shortest_path

# Iki poz ve donus yaricapi. Poz = (x, y, yaw); yaw RADYAN.
START = (0.0, 0.0, math.radians(155))  # mevcut (x, y, heading)
GOAL = (7.0, 5.0, math.radians(65))   # hedef (x, y, heading)
RHO = 2.0                             # minimum donus yaricapi, metre

WORDS = ["LSL", "RSR", "LSR", "RSL", "RLR", "LRL"]


def draw_pose(ax, pose, color, label):
    """Bir pozu ok olarak cizer: okun yeri konum, yonu yaw."""
    x, y, yaw = pose
    ax.arrow(x, y, math.cos(yaw), math.sin(yaw),
             head_width=0.15, color=color, length_includes_head=False)
    ax.plot(x, y, "o", color=color, label=label)


def draw_path(ax, path, color="steelblue"):
    """Yolu ornekleyip cizer, uclarina baslangic/hedef oklarini koyar."""
    # sample() poz listesi verir; cizim icin x'leri ve y'leri ayirmamiz gerek
    points = path.sample(0.02)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    ax.plot(xs, ys, color=color, linewidth=2)
    draw_pose(ax, START, "seagreen", "baslangic")
    draw_pose(ax, GOAL, "crimson", "hedef")

    ax.set_aspect("equal")  # bu olmadan daireler elips gorunur
    ax.grid(True, alpha=0.3)


def plot_shortest():
    """En kisa yolu tek bir grafikte cizer."""
    path = shortest_path(START, GOAL, RHO)

    _, ax = plt.subplots(figsize=(7, 6))
    draw_path(ax, path)
    ax.set_title(f"{path.word} — {path.length:.2f} m  (rho={RHO})")
    ax.set_xlabel("x (dogu, m)")
    ax.set_ylabel("y (kuzey, m)")
    ax.legend()

    plt.tight_layout()
    plt.savefig("results/dubins_shortest.png", dpi=150)
    plt.close()
    print(f"kaydedildi: results/dubins_shortest.png  ({path.word}, {path.length:.2f} m)")


def plot_all_words():
    """Alti kelimeyi 2x3 izgarada karsilastirir; en kisasi vurgulanir."""
    # Listeyi kelimeye gore erisilebilir sozluge cevir. Gecersiz kelimeler
    # all_paths'ten hic donmedigi icin sozlukte de olmaz.
    paths = {p.word: p for p in all_paths(START, GOAL, RHO)}
    best = shortest_path(START, GOAL, RHO)

    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    axes = axes.flatten()  # 2x3 tabloyu axes[0]..axes[5] duz dizisine cevirir

    for ax, word in zip(axes, WORDS):
        path = paths.get(word)  # yoksa None; paths[word] olsaydi KeyError
        if path is None:
            ax.text(0.5, 0.5, "gecersiz", ha="center", va="center",
                    transform=ax.transAxes, color="gray", fontsize=14)
            ax.set_title(word, color="gray")
            ax.set_xticks([])
            ax.set_yticks([])
            continue

        is_best = word == best.word
        draw_path(ax, path, color="crimson" if is_best else "steelblue")
        ax.set_title(f"{word} — {path.length:.2f} m",
                     color="crimson" if is_best else "black",
                     fontweight="bold" if is_best else "normal")

    fig.suptitle(
        f"Dubins kelimeleri  |  start=({START[0]:.1f}, {START[1]:.1f})  "
        f"goal=({GOAL[0]:.1f}, {GOAL[1]:.1f})  rho={RHO}",
        fontsize=13)
    plt.tight_layout()
    plt.savefig("results/dubins_words.png", dpi=150)
    plt.close()
    print(f"kaydedildi: results/dubins_words.png  "
          f"({len(paths)}/6 kelime gecerli, en kisa: {best.word})")


def main():
    plot_shortest()
    plot_all_words()


if __name__ == "__main__":
    main()

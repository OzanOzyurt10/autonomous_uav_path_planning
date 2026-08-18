"""Dubins modulu icin hiz olcumu ve bagimsiz dogruluk kontrolu.

Iki ayri soruyu cevaplar:

1. HIZ - cagri basina kac mikrosaniye, ve maliyet girdiye gore degisiyor mu.
2. DOGRULUK - kapali form cozucunun buldugu yol gercekten en kisa mi.

Ikinci kisim onemli: kapali form formuller (Shkel & Lumelsky) literaturden
alinma. Onlarin dogrulugunu yine kendileriyle test etmek dairesel olurdu.
Bu yuzden burada formulleri HIC kullanmayan, sadece _segment_end geometrisi
uzerinde sayisal arama yapan bagimsiz bir cozucu var. Ikisi ayni sonucu
veriyorsa formuller dogru demektir.

Calistirma (proje kokunden, -m sart):
    ./venv/Scripts/python.exe -m notebooks.dubins_benchmark
"""

import math
import random
import time

from src.dubins import (_mod2pi, _segment_end, all_paths, path_length,
                        shortest_path)

WORDS = ["LSL", "RSR", "LSR", "RSL", "RLR", "LRL"]


# --------------------------------------------------------------------------
# 1. HIZ
# --------------------------------------------------------------------------

def random_pairs(seed, n, span=100.0):
    rng = random.Random(seed)
    return [((rng.uniform(-span, span), rng.uniform(-span, span),
              rng.uniform(0, 2 * math.pi)),
             (rng.uniform(-span, span), rng.uniform(-span, span),
              rng.uniform(0, 2 * math.pi)))
            for _ in range(n)]


def time_call(fn, pairs):
    """Cagri basina mikrosaniye."""
    t0 = time.perf_counter()
    for start, goal in pairs:
        fn(start, goal)
    return (time.perf_counter() - t0) / len(pairs) * 1e6


def benchmark_speed(rho=25.0, n=20000):
    print("=" * 66)
    print("1. HIZ")
    print("=" * 66)
    pairs = random_pairs(seed=42, n=n)

    print(f"\n{n:,} rastgele poz cifti, rho={rho}\n")
    print(f"  {'fonksiyon':<20} {'us/cagri':>10}   {'cagri/s':>12}")
    print(f"  {'-' * 20} {'-' * 10}   {'-' * 12}")
    for name, fn in [
        ("path_length", lambda s, g: path_length(s, g, rho)),
        ("shortest_path", lambda s, g: shortest_path(s, g, rho)),
        ("all_paths", lambda s, g: all_paths(s, g, rho)),
    ]:
        us = time_call(fn, pairs)
        print(f"  {name:<20} {us:10.2f}   {1e6 / us:12,.0f}")

    # sample() ayri kategoriye girer: maliyeti adim sayisina bagli, O(n)
    print(f"\n  {'sample(step)':<20} {'us/cagri':>10}   {'nokta':>12}")
    print(f"  {'-' * 20} {'-' * 10}   {'-' * 12}")
    path = shortest_path(*pairs[0], rho)
    for step in (5.0, 1.0, 0.5, 0.1):
        reps = 2000
        t0 = time.perf_counter()
        for _ in range(reps):
            path.sample(step)
        us = (time.perf_counter() - t0) / reps * 1e6
        print(f"  step={step:<15} {us:10.2f}   {len(path.sample(step)):12,}")

    print("\n  Cozum O(1): alti kapali form ifadesi, dongu/arama/yakinsama yok.")
    print("  sample O(n): n = length / step. Asil maliyet burada.")


def benchmark_scaling():
    """Maliyet girdiye gore degisiyor mu? O(1) iddiasinin sinanmasi."""
    print(f"\n  Girdiye gore degisim (her biri 20,000 cagri):\n")
    print(f"  {'kosul':<28} {'us/cagri':>10}")
    print(f"  {'-' * 28} {'-' * 10}")
    for label, span, rho in [
        ("yakin pozlar (span=10)", 10.0, 25.0),
        ("orta (span=100)", 100.0, 25.0),
        ("uzak pozlar (span=1000)", 1000.0, 25.0),
        ("kucuk rho=1", 100.0, 1.0),
        ("buyuk rho=200", 100.0, 200.0),
    ]:
        pairs = random_pairs(seed=7, n=20000, span=span)
        us = time_call(lambda s, g: path_length(s, g, rho), pairs)
        print(f"  {label:<28} {us:10.2f}")
    print("\n  Sayilar birbirine yakinsa maliyet girdiden bagimsiz demektir.")


# --------------------------------------------------------------------------
# 2. DOGRULUK - bagimsiz sayisal cozucu
# --------------------------------------------------------------------------

def _end_pose_for(start, rho, word, t_len, p_len, goal_yaw):
    """Ilk iki segment verilince ucuncusu heading dengesinden belirlenir.

    Kapali form formul kullanilmaz; sadece _segment_end geometrisi.
    Doner: (bitis_pozu, toplam_uzunluk)
    """
    pose = _segment_end(start, word[0], t_len, rho)
    pose = _segment_end(pose, word[1], p_len, rho)

    # Ucuncu segment hedef heading'e ulastirmali. Sola donuste aci artar,
    # saga donuste azalir; gereken donus miktari buradan cikar.
    if word[2] == "L":
        q_angle = _mod2pi(goal_yaw - pose[2])
    else:
        q_angle = _mod2pi(pose[2] - goal_yaw)
    q_len = q_angle * rho

    end = _segment_end(pose, word[2], q_len, rho)
    return end, t_len + p_len + q_len


def _refine(start, goal, rho, word, t0, p0, half_t, half_p, rounds=9, n=20):
    """Bir aday nokta etrafinda pencereyi daraltarak hassaslasir.

    Doner: (hata, toplam_uzunluk)
    """
    gx, gy, gyaw = goal
    best = None
    for _ in range(rounds):
        step_t = 2 * half_t / n
        step_p = 2 * half_p / n
        best = None
        for i in range(n + 1):
            t_len = max(0.0, t0 - half_t + i * step_t)
            for j in range(n + 1):
                p_len = max(0.0, p0 - half_p + j * step_p)
                end, total = _end_pose_for(start, rho, word, t_len, p_len, gyaw)
                err = math.hypot(end[0] - gx, end[1] - gy)
                if best is None or err < best[0]:
                    best = (err, t_len, p_len, total)
        _, t0, p0, _ = best
        half_t, half_p = step_t, step_p
    return best[0], best[3]


def brute_force_word(start, goal, rho, word, grid=200):
    """Bir kelime icin en kisa yolu SAYISAL arama ile bulur.

    Kapali form formullere hic dokunmaz; sadece _segment_end geometrisi
    kullanilir.

    Bir kelimenin birden fazla gecerli cozum dali olabilir (ayni sekil,
    farkli sayida tam tur). Tek bir en iyi noktayi hassaslastirmak yanlis
    dala kilitlenmeye yol aciyor. Bu yuzden kaba izgaradaki TUM yerel
    minimumlar bulunur, her biri ayri ayri hassaslastirilir ve hedefe
    gercekten ulasanlarin en kisasi secilir.

    Doner: uzunluk, ya da cozum bulunamazsa None.
    """
    gx, gy, gyaw = goal
    dist = math.hypot(gx - start[0], gy - start[1])

    t_max = 2 * math.pi * rho
    p_max = dist + 4 * rho if word[1] == "S" else 2 * math.pi * rho

    step_t = t_max / grid
    step_p = p_max / grid

    # Kaba izgarayi tara, hata yuzeyini kaydet
    errs = []
    for i in range(grid + 1):
        t_len = i * step_t
        row = []
        for j in range(grid + 1):
            p_len = j * step_p
            end, _ = _end_pose_for(start, rho, word, t_len, p_len, gyaw)
            row.append(math.hypot(end[0] - gx, end[1] - gy))
        errs.append(row)

    # Yerel minimumlar: sekiz komsusundan kucuk esit olan hucreler
    candidates = []
    for i in range(grid + 1):
        for j in range(grid + 1):
            e = errs[i][j]
            is_min = True
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ni, nj = i + di, j + dj
                    if 0 <= ni <= grid and 0 <= nj <= grid and errs[ni][nj] < e:
                        is_min = False
                        break
                if not is_min:
                    break
            if is_min:
                candidates.append((e, i * step_t, j * step_p))

    # En umut verici adaylari hassaslastir
    candidates.sort()
    best_length = None
    for _, t0, p0 in candidates[:12]:
        err, total = _refine(start, goal, rho, word, t0, p0, step_t, step_p)
        if err < 1e-6 and (best_length is None or total < best_length):
            best_length = total
    return best_length


def verify_optimality(n_pairs=12, rho=10.0, span=25.0):
    print("\n" + "=" * 66)
    print("2. DOGRULUK - bagimsiz sayisal dogrulama")
    print("=" * 66)
    print("\n  Kapali form cozucu ile, formulleri hic kullanmayan sayisal")
    print("  aramanin sonuclari karsilastiriliyor.\n")
    print(f"  {'#':>3}  {'kapali form':>14}  {'sayisal arama':>14}  "
          f"{'fark':>10}  {'kelime':>7}")
    print(f"  {'-' * 3}  {'-' * 14}  {'-' * 14}  {'-' * 10}  {'-' * 7}")

    pairs = random_pairs(seed=99, n=n_pairs, span=span)
    worst = 0.0
    mismatches = 0

    for k, (start, goal) in enumerate(pairs, 1):
        closed = shortest_path(start, goal, rho)

        numeric = None
        for word in WORDS:
            length = brute_force_word(start, goal, rho, word)
            if length is not None and (numeric is None or length < numeric):
                numeric = length

        if numeric is None:
            print(f"  {k:>3}  {closed.length:14.6f}  {'bulunamadi':>14}"
                  f"  {'-':>10}  {closed.word:>7}")
            mismatches += 1
            continue

        diff = abs(closed.length - numeric)
        worst = max(worst, diff)
        flag = "" if diff < 1e-3 else "  <-- FARK"
        if diff >= 1e-3:
            mismatches += 1
        print(f"  {k:>3}  {closed.length:14.6f}  {numeric:14.6f}"
              f"  {diff:10.2e}  {closed.word:>7}{flag}")

    print(f"\n  En buyuk fark: {worst:.2e} m")
    print(f"  Sayisal aramanin cozunurlugu sinirli oldugu icin kucuk bir")
    print(f"  fark beklenir; buyuk fark formulde hata demektir.")
    if mismatches == 0:
        print(f"\n  SONUC: {n_pairs}/{n_pairs} poz ciftinde iki yontem ortusuyor.")
    else:
        print(f"\n  SONUC: {mismatches} poz ciftinde uyusmazlik var.")
    return mismatches


def main():
    benchmark_speed()
    benchmark_scaling()
    mismatches = verify_optimality()
    print()
    return mismatches


if __name__ == "__main__":
    main()

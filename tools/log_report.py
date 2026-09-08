"""SITL ucus kaydindan rota suresini ve hava/yer hizi gercegini cikarir.

Zaman ucagin kendi saatinden. Hava hizi SENSORDEN degil, yer hizi
vektorunden ruzgar cikarilarak: sensor gosterge hizi veriyor, model
gercek hava hizi istiyor ve ikisi kotla ayrisiyor.

Sure tlog'dan degil dataflash kaydindan (logs/*.BIN) okunuyor: son
waypointin MISSION_ITEM_REACHED'i telemetride kacabiliyor, dataflash'ta
kacmiyor.

pymavlink gerektiriyor; WSL'de calistir:
    python3 tools/log_report.py <log.BIN> <ruzgar_hiz> <ruzgar_yon>
"""

import math
import re
import statistics
import sys

from pymavlink import mavutil


PAT = re.compile(r"(?:Reached|Passed) waypoint #(\d+)")

def main():
    path, w_spd, w_from = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    th = math.radians(90.0 - (w_from + 180.0) % 360.0)
    wx, wy = w_spd * math.cos(th), w_spd * math.sin(th)

    log = mavutil.mavlink_connection(path)
    reached, gps, arsp = {}, [], []
    while True:
        m = log.recv_match(type=["MSG", "GPS", "ARSP"])
        if m is None:
            break
        t = m.TimeUS / 1e6
        k = m.get_type()
        if k == "MSG":
            g = PAT.search(m.Message)
            if g:
                reached.setdefault(int(g.group(1)), t)
        elif k == "GPS":
            gps.append((t, m.Spd, m.GCrs, m.Alt))
        else:
            arsp.append((t, m.Airspeed))

    seqs = sorted(s for s in reached if s >= 2)
    if len(seqs) < 2:
        raise SystemExit(f"yeterli waypoint yok: {len(seqs)}")
    t0, t1 = reached[seqs[0]], reached[seqs[-1]]
    print(f"seq {seqs[0]} @ {t0:.1f} s -> seq {seqs[-1]} @ {t1:.1f} s")
    print(f"OLCULEN ROTA SURESI  {t1 - t0:.1f} s = {(t1 - t0) / 60:.2f} dk")

    win = [(t, s, c, a) for t, s, c, a in gps if t0 <= t <= t1]
    ias = [v for t, v in arsp if t0 <= t <= t1]
    air, dist, prev = [], 0.0, None
    for t, spd, crs, alt in win:
        c = math.radians(90.0 - crs)
        gx, gy = spd * math.cos(c), spd * math.sin(c)
        air.append(math.hypot(gx - wx, gy - wy))
        if prev is not None and 0 < t - prev <= 2:
            dist += spd * (t - prev)
        prev = t
    print(f"ucusan mesafe        {dist / 1000:.2f} km")
    print(f"yer hizi        ort {statistics.mean(s for _, s, _, _ in win):6.2f}  "
          f"min {min(s for _, s, _, _ in win):5.1f}  "
          f"max {max(s for _, s, _, _ in win):5.1f}")
    print(f"gosterge hava hizi ort {statistics.mean(ias):6.2f}")
    print(f"GERCEK hava hizi   ort {statistics.mean(air):6.2f}  "
          f"(yer hizi vektorunden)")
    alts = [a for _, _, _, a in win]
    print(f"kot             ort {statistics.mean(alts):6.0f}  "
          f"min {min(alts):5.0f}  max {max(alts):5.0f}")

main()

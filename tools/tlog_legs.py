"""SITL tlog'undan gercek ucus suresini cikarir.

Tahmin ettigimiz sureyi olculen sureyle karsilastirmak icin. Zaman
kaynagi UCAGIN kendi saati (time_boot_ms), duvar saati DEGIL - sim_vehicle
--speedup ile kosarsa duvar saati yanlis olur, ucagin saati dogru kalir.

pymavlink gerektiriyor; projenin venv'inde yok, WSL'de MAVProxy ile
birlikte kurulu. Orada calistir:
    python3 tools/tlog_legs.py ~/logs/xxx.tlog
"""

import sys

from pymavlink import mavutil

TAKEOFF = 22                   # MAV_CMD_NAV_TAKEOFF


def leg_times(path):
    """Her ulasilan gorev noktasi icin (seq, ucak saati saniye) listesi."""
    log = mavutil.mavlink_connection(path)
    clock, reached = None, []
    while True:
        msg = log.recv_match(
            type=["GLOBAL_POSITION_INT", "MISSION_ITEM_REACHED"],
            blocking=False)
        if msg is None:
            break
        if msg.get_type() == "GLOBAL_POSITION_INT":
            clock = msg.time_boot_ms / 1000.0
        elif clock is not None:
            reached.append((msg.seq, clock))
    return reached


def main():
    if len(sys.argv) != 2:
        raise SystemExit("kullanim: python3 tools/tlog_legs.py <dosya.tlog>")

    reached = leg_times(sys.argv[1])
    if len(reached) < 2:
        raise SystemExit(f"yeterli MISSION_ITEM_REACHED yok: {len(reached)}")

    print("seq   ucak saati    onceki noktadan")
    previous = None
    for seq, when in reached:
        gap = "" if previous is None else f"{when - previous:8.1f} s"
        print(f"{seq:3d}   {when:9.1f} s   {gap}")
        previous = when

    # Kalkis satirini disarida birakiyoruz: tahminimiz yalnizca rotayi
    # kapsiyor, kalkis tirmanmasini degil.
    route = [item for item in reached if item[0] > 1]
    if len(route) >= 2:
        total = route[-1][1] - route[0][1]
        print(f"\nrota suresi (seq {route[0][0]} -> {route[-1][0]}): "
              f"{total:.1f} s = {total / 60:.1f} dk")


if __name__ == "__main__":
    main()

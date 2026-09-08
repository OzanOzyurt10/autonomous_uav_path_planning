"""SITL ucagini gorev dosyasiyla ucurur ve bacak surelerini olcer.

MAVProxy yerine dogrudan pymavlink: komutlar betikten geliyor, ucus
tekrarlanabilir oluyor. Zaman kaynagi UCAGIN saati (time_boot_ms), duvar
saati degil - --speedup ile duvar saati yanlis olurdu.

pymavlink gerektiriyor; projenin venv'inde yok, WSL'de MAVProxy ile
birlikte kurulu. Orada calistir:
    python3 tools/fly_mission.py <gorev.waypoints> <ruzgar_hiz>         <ruzgar_yon> <AIRSPEED_CRUISE>
"""

import sys
import time

from pymavlink import mavutil


def read_wpl(path):
    items = []
    with open(path) as fh:
        head = fh.readline()
        if not head.startswith("QGC WPL"):
            raise SystemExit("QGC WPL dosyasi degil: " + head)
        for line in fh:
            if line.startswith("QGC WPL"):
                # Windows "copy" joker karakterle birden fazla dosya
                # bulursa birlestiriyor; seq numaralari sifirdan yeniden
                # basliyor ve otopilota anlamsiz bir gorev gider.
                raise SystemExit(
                    f"dosyada birden fazla gorev var: {path}. Tek bir "
                    f"gorev dosyasi ver.")
            f = line.split()
            if len(f) < 12:
                continue
            items.append(dict(seq=int(f[0]), current=int(f[1]), frame=int(f[2]),
                              command=int(f[3]), p1=float(f[4]), p2=float(f[5]),
                              p3=float(f[6]), p4=float(f[7]), lat=float(f[8]),
                              lon=float(f[9]), alt=float(f[10]), auto=int(f[11])))
    return items

def upload(m, items):
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
    sent = 0
    while sent < len(items):
        msg = m.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                 "MISSION_ACK"], blocking=True, timeout=30)
        if msg is None:
            raise SystemExit("gorev yuklemesi zaman asimi")
        if msg.get_type() == "MISSION_ACK":
            break
        it = items[msg.seq]
        m.mav.mission_item_int_send(
            m.target_system, m.target_component, it["seq"], it["frame"],
            it["command"], it["current"], it["auto"], it["p1"], it["p2"],
            it["p3"], it["p4"], int(it["lat"] * 1e7), int(it["lon"] * 1e7),
            it["alt"], 0)
        sent += 1
    ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=30)
    print("gorev yuklendi:", len(items), "ogesi, ack",
          ack.type if ack else "yok", flush=True)

def setp(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component,
                         name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id.strip("\x00") == name:
            print(f"  {name} = {msg.param_value:.3f}", flush=True)
            return msg.param_value
    print(f"  {name} AYARLANAMADI", flush=True)
    return None

# Ucaga dogrudan baglanti. MAVProxy calisiyorsa 5760'i o tutuyor; o
# durumda MAVProxy'nin bir cikisina baglanmak gerekiyor, ornegin
# sim_vehicle'a --out udp:127.0.0.1:14551 verip buraya
# udpin:127.0.0.1:14551 yazmak. Boylece konsol ve harita da acik kaliyor.
DEFAULT_LINK = "tcp:127.0.0.1:5760"


def main():
    if not 5 <= len(sys.argv) <= 6:
        raise SystemExit("kullanim: python3 tools/fly_mission.py "
                         "<gorev.waypoints> <ruzgar_hiz> <ruzgar_yon> "
                         "<hava_hizi> [baglanti]")
    path, wind_spd, wind_dir, ias = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    link = sys.argv[5] if len(sys.argv) > 5 else DEFAULT_LINK
    items = read_wpl(path)
    print("baglaniliyor:", link, flush=True)
    m = mavutil.mavlink_connection(link)
    m.wait_heartbeat()
    print("baglandi, sistem", m.target_system, flush=True)
    time.sleep(2)

    print("parametreler:", flush=True)
    for name, value in [("TERRAIN_ENABLE", 0), ("AIRSPEED_CRUISE", ias),
                        ("WP_RADIUS", 50), ("SIM_WIND_SPD", wind_spd),
                        ("SIM_WIND_DIR", wind_dir), ("SIM_WIND_TURB", 0),
                        ("SIM_WIND_T", 1)]:
        setp(m, name, value)

    upload(m, items)

    # Pre-arm kontrolleri gecene kadar bekle: EKF ve GPS oturmadan
    # arming reddediliyor ve sebep yalnizca STATUSTEXT ile geliyor.
    print("arming bekleniyor...", flush=True)
    deadline = time.time() + 120
    armed = False
    while time.time() < deadline and not armed:
        m.set_mode_apm("AUTO")
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        end = time.time() + 5
        while time.time() < end:
            msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"],
                               blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "STATUSTEXT":
                print("  [ucak]", msg.text.strip(), flush=True)
            elif msg.get_type() == "HEARTBEAT" and (
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                break
    if not armed:
        raise SystemExit("arming basarisiz")
    print("armed", flush=True)
    for _ in range(30):
        m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                        0, 0, 1800, 0, 0, 0, 0, 0)
        time.sleep(0.2)
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    0, 0, 0, 0, 0, 0, 0, 0)
    print("kalkis komutu verildi", flush=True)

    clock, reached, last = None, [], time.time()
    target = items[-1]["seq"]
    airspeeds, groundspeeds, alts = [], [], []
    while time.time() - last < 300:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "MISSION_ITEM_REACHED",
                                 "VFR_HUD", "STATUSTEXT"], blocking=True, timeout=5)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "GLOBAL_POSITION_INT":
            clock = msg.time_boot_ms / 1000.0
        elif kind == "VFR_HUD":
            airspeeds.append(msg.airspeed); groundspeeds.append(msg.groundspeed)
            alts.append(msg.alt)
        elif kind == "STATUSTEXT":
            t = msg.text.strip()
            if any(k in t for k in ("Crash", "Land", "Disarm", "Failsafe", "EKF")):
                print("  [ucak]", t, flush=True)
        elif kind == "MISSION_ITEM_REACHED" and clock is not None:
            reached.append((msg.seq, clock))
            last = time.time()
            print(f"  ulasildi seq {msg.seq:3d} @ {clock:8.1f} s", flush=True)
            if msg.seq >= target:
                break

    route = [r for r in reached if r[0] > 1]
    print("\n==== SONUC ====", flush=True)
    if len(route) >= 2:
        total = route[-1][1] - route[0][1]
        print(f"rota suresi (seq {route[0][0]} -> {route[-1][0]}): "
              f"{total:.1f} s = {total/60:.2f} dk")
    else:
        print("yeterli waypoint gecilmedi:", len(route))
    if airspeeds:
        n = len(airspeeds)
        print(f"hava hizi (gosterge) ort {sum(airspeeds)/n:.2f}  "
              f"min {min(airspeeds):.1f} max {max(airspeeds):.1f}")
        print(f"yer hizi ort {sum(groundspeeds)/n:.2f}  "
              f"min {min(groundspeeds):.1f} max {max(groundspeeds):.1f}")
        print(f"kot {min(alts):.0f} - {max(alts):.0f} m, ort {sum(alts)/n:.0f}")

main()

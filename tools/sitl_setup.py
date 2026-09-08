"""Gorev dosyasindan SITL komutlarini hazir yazar.

Home enlem/boylam/irtifasi elle hesaplanirsa hataya cok acik ve sessizce
yaniltiyor: home rotanin en dusuk kotunun ustunde kalirsa SITL'in duz
zemini rotanin uzerine cikiyor ve ucak yerin altina inmeye calisiyor.
Ucu de gorev dosyasindaki kotlardan cikiyor, o yuzden burada cikariliyor.

AIRSPEED_CRUISE hesaplanmiyor, ARAYUZDEN aliniyor: gercek hava hizindan
gosterge hizina cevirmeyi uygulama zaten yapiyor (app/mission.py,
indicated_airspeed) ve rotayi planlarken ekranda yaziyor. Ayni formulun
ikinci bir kopyasi burada dursaydi ikisi ayrisir ve kimse fark etmezdi.

WSL'de calistir:
    python3 tools/sitl_setup.py <gorev.waypoints> <ruzgar_hiz> \\
        <ruzgar_yon> <AIRSPEED_CRUISE>
"""

import math
import os
import sys

TAKEOFF = 22                   # MAV_CMD_NAV_TAKEOFF
# SITL zemini duz ve home irtifasinda; rota home'un altina inerse ucak
# yerin altina inmeye calisir. Rotanin en dususunun bu kadar altina.
HOME_MARGIN = 50.0


def read_rows(path):
    """Gorev satirlari: (komut, enlem, boylam, kot)."""
    rows = []
    with open(path) as handle:
        head = handle.readline()
        if not head.startswith("QGC WPL"):
            raise SystemExit(f"QGC WPL dosyasi degil: {path}")
        for line in handle:
            if line.startswith("QGC WPL"):
                # Windows'ta "copy dosya_*.waypoints hedef" birden fazla
                # eslesme bulursa dosyalari BIRLESTIRIYOR. Sonuc gecerli
                # gorunuyor ama seq numaralari sifirdan yeniden basliyor;
                # yakalamazsak otopilota anlamsiz bir gorev gider.
                raise SystemExit(
                    f"dosyada birden fazla gorev var ({path}); Windows "
                    f"'copy' joker karakterle birlestirmis olabilir. Tek "
                    f"bir gorev dosyasi ver.")
            f = line.split()
            if len(f) >= 12:
                rows.append((int(f[3]), float(f[8]), float(f[9]), float(f[10])))
    if len(rows) < 3:
        raise SystemExit(f"gorev cok kisa: {len(rows)} satir")
    return rows


def route_length(route):
    """Waypointler arasi duz mesafelerin toplami, metre."""
    total = 0.0
    for (_, lat_a, lon_a, _), (_, lat_b, lon_b, _) in zip(route, route[1:]):
        mid = math.radians((lat_a + lat_b) / 2)
        total += math.hypot((lat_b - lat_a) * 111320.0,
                            (lon_b - lon_a) * 111320.0 * math.cos(mid))
    return total


def main():
    if len(sys.argv) != 5:
        raise SystemExit(
            "kullanim: python3 tools/sitl_setup.py <gorev.waypoints> "
            "<ruzgar_hiz> <ruzgar_yon> <AIRSPEED_CRUISE>\n"
            "AIRSPEED_CRUISE arayuzde rotayi planlayinca alt seritte yaziyor.")
    path, wind_speed, wind_from, airspeed = sys.argv[1:5]
    if float(airspeed) <= 0.0:
        raise SystemExit(f"AIRSPEED_CRUISE pozitif olmali: {airspeed}")

    rows = read_rows(path)
    home_lat, home_lon = rows[0][1], rows[0][2]
    # Kalkis satirinin enlem/boylami sifir, kotu da tirmanma hedefi:
    # ikisi de rotaya ait degil, en dususu bozarlar.
    route = [r for r in rows[2:] if r[0] != TAKEOFF]
    altitudes = [alt for _, _, _, alt in route]
    home_alt = min(altitudes) - HOME_MARGIN
    full = os.path.abspath(path)

    print(f"gorev      {path}")
    print(f"  {len(route)} nokta, {route_length(route) / 1000:.1f} km")
    print(f"  kot {min(altitudes):.0f} - {max(altitudes):.0f} m")
    print(f"  AIRSPEED_CRUISE {airspeed} (arayuzden alindi)")
    print()
    print("1) SIMULATOR  (bu terminal acik kalacak)")
    print("   cd ~/ardupilot && python3 Tools/autotest/sim_vehicle.py "
          "-v ArduPlane \\")
    print("       --no-mavproxy --no-rebuild --speedup 10 \\")
    print(f"       -l {home_lat:.6f},{home_lon:.6f},{home_alt:.0f},0")
    print()
    print("2) UCUS  (ikinci terminal)")
    print(f"   python3 ~/tools/fly_mission.py {full} "
          f"{wind_speed} {wind_from} {airspeed}")
    print()
    print("3) SONUC")
    print("   python3 ~/tools/log_report.py "
          f"$(ls -t ~/ardupilot/logs/*.BIN | head -1) {wind_speed} {wind_from}")
    print()
    # Yukaridaki akis MAVProxy'siz, yani ekranda ucak yok. Izlemek isteyen
    # icin tek terminallik klasik yol; param satirlari sayilar dolu
    # yaziliyor ki kopyala-yapistir olsun.
    print("UCUSU IZLEMEK ISTERSEN -- TEK TERMINAL, YUKARIDAKILERIN YERINE")
    print()
    print("   cd ~/ardupilot && python3 Tools/autotest/sim_vehicle.py "
          "-v ArduPlane \\")
    print("       --console --map --speedup 10 \\")
    print(f"       -l {home_lat:.6f},{home_lon:.6f},{home_alt:.0f},0")
    print()
    print("   Acilan MAVProxy isteminde:")
    print("     param set TERRAIN_ENABLE 0")
    print(f"     param set AIRSPEED_CRUISE {airspeed}")
    print(f"     param set SIM_WIND_SPD {wind_speed}")
    print(f"     param set SIM_WIND_DIR {wind_from}")
    print("     param set SIM_WIND_T 1")
    print(f"     wp load {full}")
    print("     mode AUTO")
    print("     arm throttle")
    print()
    print("   Kalkis baslamazsa: rc 3 1800")
    print("   Harita acilmazsa --map'i cikar, --console yeter.")
    print("   Sonucu yine 3. adimdaki log_report.py veriyor.")


if __name__ == "__main__":
    main()

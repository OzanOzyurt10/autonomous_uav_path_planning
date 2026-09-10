"""Yogun rotayi ArduPilot/QGroundControl gorev dosyasina cevirir.

Iki ayri is: rotayi tasinabilir sayida noktaya indirmek (simplify) ve o
noktalari QGC WPL 110 bicimine yazmak (mission_text). Planlayicinin
urettigi rota 14 metre araliklarla yuzlerce poz; otopilot o kadar noktayi
ne saklar ne de anlamli uceabilir.

Irtifalar MUTLAK (MSL, frame 0) yaziliyor: bizim kotlarimiz zaten MSL ve
home'a gore cevirmek otopilotun home kotu bizimkinden farkli oldugunda
butun gorevi kaydirirdi.
"""

import math

VERSION_LINE = "QGC WPL 110"
FRAME_GLOBAL = 0               # MAV_FRAME_GLOBAL: irtifa MSL
NAV_WAYPOINT = 16              # MAV_CMD_NAV_WAYPOINT
NAV_TAKEOFF = 22               # MAV_CMD_NAV_TAKEOFF
# Ucak noktaya varinca param1 saniye bekliyor, sonra rotaya devam ediyor.
# Yaricap sifir birakiliyor: otopilot kendi WP_LOITER_RAD'ini kullaniyor
# ve o deger ucaga gore dogru olan.
NAV_LOITER_TIME = 19           # MAV_CMD_NAV_LOITER_TIME
TAKEOFF_PITCH = 15.0           # derece; ArduPlane'in kalkis tirmanma acisi


def simplify(points, tolerance: float) -> list[int]:
    """Rotayi seyreltir; korunacak noktalarin indislerini doner.

    Garanti sudur: birakilan noktalari birlestiren cizgi, atilan hicbir
    noktadan tolerance metreden fazla sapmaz. "Her N'inci noktayi al"
    gibi keyfi bir seyreltme bu garantiyi vermez - donuslerde rotayi
    keser, duz kesimlerde bosuna nokta birakir.
    """
    if tolerance < 0.0:
        raise ValueError(f"tolerans negatif olamaz: {tolerance}")
    if len(points) < 3:
        return list(range(len(points)))

    last = len(points) - 1
    return [0] + _between(points, tolerance, 0, last) + [last]


def _between(points, tolerance, start, end) -> list[int]:
    """[start, end] arasinda KORUNMASI gereken ara noktalarin indisleri.

    Uclari hic eklemiyor: onlari cagiran koyuyor. Boylece bolerken ayni
    nokta iki koldan birden gelemiyor. Dizi de hic dilimlenmiyor - dilim
    elemanlari tasir ama nerede olduklarini tasimaz, indisler kayardi.
    """
    worst, at = -1.0, -1
    for index in range(start + 1, end):
        gap = _point_to_segment(points[index], points[start], points[end])
        if gap > worst:
            worst, at = gap, index

    if at < 0 or worst <= tolerance:
        return []
    return (_between(points, tolerance, start, at) + [at]
            + _between(points, tolerance, at, end))


def _line(index, current, frame, command, lat, lon, alt, param1=0.0):
    """Tek bir gorev satiri; alanlar SEKME ile ayriliyor, boslukla degil."""
    return "\t".join([
        str(index), str(current), str(frame), str(command),
        f"{param1:.8f}", "0.00000000", "0.00000000", "0.00000000",
        f"{lat:.8f}", f"{lon:.8f}", f"{alt:.6f}", "1",
    ])


def mission_text(waypoints, home=None, takeoff: bool = True,
                 loiter=None) -> str:
    """(lat, lon, MSL kot) dizisini QGC WPL 110 metnine cevirir.

    Satir 0 her zaman home ve CURRENT=1 olmak zorunda; otopilot dosyayi
    boyle bekliyor. home verilmezse ilk waypoint kullaniliyor, ama dogru
    olan zemin kotunu gecirmek - home ucus kotu degil, kalkis noktasi.

    loiter waypointlerle ayni uzunlukta saniye listesi; sifirdan buyuk
    olanlar NAV_WAYPOINT yerine NAV_LOITER_TIME oluyor.
    """
    if not waypoints:
        raise ValueError("en az bir waypoint gerekli")
    if loiter is None:
        loiter = [0.0] * len(waypoints)
    if len(loiter) != len(waypoints):
        raise ValueError(f"bekleme sayisi waypoint sayisiyla uyusmuyor: "
                         f"{len(loiter)} != {len(waypoints)}")
    if any(seconds < 0.0 for seconds in loiter):
        raise ValueError("bekleme suresi negatif olamaz")

    first = waypoints[0]
    if home is None:
        home = first

    lines = [VERSION_LINE,
             _line(0, 1, FRAME_GLOBAL, NAV_WAYPOINT, home[0], home[1],
                   home[2])]
    index = 1
    if takeoff:
        # Kalkista enlem/boylam sifir: ArduPlane mevcut yonunde tirmaniyor.
        # Hedef kot ilk waypointin kotu, boylece rotaya seyirde giriyor.
        lines.append(_line(index, 0, FRAME_GLOBAL, NAV_TAKEOFF, 0.0, 0.0,
                           first[2], param1=TAKEOFF_PITCH))
        index += 1

    for (lat, lon, alt), seconds in zip(waypoints, loiter):
        command = NAV_LOITER_TIME if seconds > 0.0 else NAV_WAYPOINT
        lines.append(_line(index, 0, FRAME_GLOBAL, command, lat, lon, alt,
                           param1=seconds))
        index += 1
    return "\n".join(lines) + "\n"


def deviation(points, kept) -> float:
    """Seyreltilmis cizginin orijinal rotadan en buyuk sapmasi, metre.

    simplify'in verdigi garantiyi olcmek icin; arayuzde kullaniciya
    "gorev dosyasi rotadan en fazla su kadar sapiyor" demek de bunun
    uzerinden oluyor.
    """
    if len(kept) < 2:
        return 0.0

    worst = 0.0
    for start, end in zip(kept, kept[1:]):
        for index in range(start + 1, end):
            worst = max(worst, _point_to_segment(
                points[index], points[start], points[end]))
    return worst


def _point_to_segment(point, start, end) -> float:
    """Noktanin [start, end] dogru parcasina en kisa uzakligi, 3B."""
    sx, sy, sz = start[0], start[1], start[2]
    ex, ey, ez = end[0], end[1], end[2]
    px, py, pz = point[0], point[1], point[2]

    dx, dy, dz = ex - sx, ey - sy, ez - sz
    span = dx * dx + dy * dy + dz * dz
    if span == 0.0:                       # parca noktaya cokmus
        return math.dist((px, py, pz), (sx, sy, sz))

    # Noktayi parcaya izdusur; t parca uzerindeki oransal konum.
    t = ((px - sx) * dx + (py - sy) * dy + (pz - sz) * dz) / span
    t = max(0.0, min(1.0, t))             # parcanin disina tasma
    return math.dist((px, py, pz),(sx + t * dx, sy + t * dy, sz + t * dz))

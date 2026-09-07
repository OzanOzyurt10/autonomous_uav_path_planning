"""Yerel gorev planlama sunucusu.

Tarayici arayuzu waypoint ve kisitlari gonderiyor, burada plan3d kosuyor,
rota ornek pozlar olarak geri donuyor. Standart kutuphane disinda bagimlilik
yok; tek kullanicili yerel bir arac icin http.server yeterli.

ThreadingHTTPServer sart: plan birkac saniye surebiliyor, tek is parcacigiyla
o sure boyunca sayfa da donardi.

Calistirma (proje kokunden):
    ./venv/Scripts/python.exe -m app.server
"""

import json
import math
import mimetypes
import os
import posixpath
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from app.mission import (Constraints, Zone, agl_profile, auto_altitude,
                         build_env, build_env_2d, compass_to_yaw, geo_window,
                         plan_mission, plan_mission_2d, sample_leg,
                         sample_mission, waypoint_altitudes, waypoint_headings,
                         yaw_to_compass)
from src.geo import Frame
from src.terrain import MissingTiles, TileStore, tile_name
from src.terrarium import ElevationUnavailable, TerrariumSource
from src.wind import flight_time, ground_speeds, wind_vector 
from src.wpl import deviation, mission_text, simplify

# .geojson standart tabloda yok; olmazsa octet-stream gider ve calisir,
# ama dogru tipi vermek tarayici tarafinda surpriz birakmiyor.
mimetypes.add_type("application/geo+json", ".geojson")

HOST, PORT = "127.0.0.1", 8000
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")

# Karo kutuphanesi: gevsek dosyalar ve zip arsivleri. Yeni karo indirdikce
# buraya koyman yeter, kapsama kendiliginden buyur.
TILE_SOURCES = ["data", "ALPS.hglzip"]

# Acilista gosterilen pencere.
START_LAT, START_LON = 46.15, 14.35
START_SIZE = 20000.0

# Tek kenar. Sinirin sebebi sure DEGIL: 150 km'lik pencere 3B'de ~20 s
# suruyor. Sinir arazi modelinin kabalasmasindan: post araligi pencereyle
# birlikte buyuyor (asagida), 100 km'de ~145 m'ye cikiyor ve dar sirtlar
# iki post arasina dusmeye basliyor.
MAX_WINDOW = 100000.0
# Arazi yokken harita hacminin tavan ve tabani. Zemin bilinmedigi icin
# irtifa MSL okunuyor; arayuz bunu acikca yaziyor.
NO_TERRAIN_FLOOR = 0.0
NO_TERRAIN_CEILING = 6000.0
DISPLAY_POSTS = 70000          # tarayiciya gonderilen hedef post sayisi
CEILING_ABOVE_PEAK = 400.0     # harita tavani: en yuksek tepe + bu kadar
FRAME_SECONDS = 0.5            # animasyon karesi; ornek araligi hiz * bu
MAX_WAYPOINTS = 12             # her bacak ayri bir RRT* kosusu
# Sinir kutusuna eklenen pay: planlayicinin engelin etrafindan dolasacak
# yeri olsun. Donus yaricapiyla da buyuyor, dar pencerede Dubins sikisiyor.
WINDOW_MARGIN = 2000.0
MAX_ZONES = 24
# Gorev dosyasi seyreltmesi: birakilan cizgi rotadan en fazla bu kadar
# sapiyor. Olculdu - 26 km'lik bir rotada 5 m tolerans 1894 pozu 16
# noktaya indiriyor, olculen sapma 4.2 m. Dubins rotasi cogunlukla duz
# oldugu icin siki tolerans bile bedava.
WPL_TOLERANCE = 5.0
# Seyir kotu artik girilmiyor, zeminden turetiliyor. Bu deger yalnizca
# arazi verisi hic yokken kullaniliyor: zemin bilinmedigi icin MSL.
NO_TERRAIN_CRUISE = 1000.0
# Otomatik seyir kotunun uzerinde birakilan manevra payi. Tavan bunun
# altinda kalirsa is_free waypointi "harita disinda" sayar.
CLIMB_ROOM = 300.0

STORE = TileStore(TILE_SOURCES)

# Yerel karo yoksa yukseklik verisi internetten cekiliyor ve data/cache
# altina yaziliyor; ayni bolge ikinci kez planlanirsa ag gerekmiyor.
# 3B'nin dunyanin her yerinde calisabilmesinin sarti bu.
TERRARIUM = TerrariumSource()
# Indirilen arazinin post araligi artik sabit degil, pencereyle olcekli.
# Sabit 90 m'de 120 km'lik pencere 90 karo isterdi ve TerrariumSource 64
# karo sinirinda reddederdi. Kenar basina hedef post sayisini sabit
# tutmak hem karo butcesini hem cizim yukunu sinirda tutuyor.
MIN_SPACING = 90.0             # daha incesi karo cozunurlugunu asiyor
TARGET_POSTS = 700             # kenar basina


def download_spacing(width_m: float, height_m: float) -> float:
    """Pencereye gore indirilecek arazinin post araligi, metre."""
    return max(MIN_SPACING, max(width_m, height_m) / TARGET_POSTS)


class Session:
    """Etkin pencere. Tek kullanicilik arac ama sunucu cok is parcacikli;
    planlama surerken pencere degisirse tutarsiz olur, kilit onu keser.

    Arazi OPSIYONEL: karo olmayan bolgede pencere yine kuruluyor, yalnizca
    zemin bilgisi yok. Uygulamanin dunyanin her yerinde calisabilmesinin
    sarti bu - SRTM'in tamami indirilemez.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.terrain = None
        self.bounds = None
        self.frame = Frame.for_window(START_LAT, START_LON, START_SIZE)
        self.size = (START_SIZE, START_SIZE)
        self.missing = []
        self.source = "yok"        # "yerel" | "indirilen" | "yok"

    def select(self, lat, lon, width_m, height_m):
        """Once yerel karolar, sonra indirme, olmazsa arazisiz.

        Yerel dosya her zaman oncelikli: kullanicinin indirdigi veri
        genelde daha ince ve zaten diskte.
        """
        source = "yerel"
        try:
            terrain = STORE.window(lat, lon, width_m, height_m)
            missing = []
        except MissingTiles as error:
            missing = error.names
            try:
                terrain = TERRARIUM.window(
                    lat, lon, width_m, height_m,
                    download_spacing(width_m, height_m))
                source = "indirilen"
            except (ElevationUnavailable, ValueError) as problem:
                print(f"  arazi indirilemedi: {problem}", flush=True)
                terrain, source = None, "yok"

        if terrain is None:
            bounds = (0.0, 0.0, NO_TERRAIN_FLOOR,
                      width_m, height_m, NO_TERRAIN_CEILING)
        else:
            low, high = terrain.elevation_range()
            bounds = (0.0, 0.0, low, terrain.extent_x, terrain.extent_y,
                      high + CEILING_ABOVE_PEAK)

        with self.lock:
            self.terrain = terrain
            self.bounds = bounds
            self.frame = Frame.for_window(lat, lon, height_m)
            self.size = (width_m, height_m)
            self.missing = missing
            self.source = source
        return terrain


SESSION = Session()


def _display_stride(terrain):
    """Tarayiciya gonderilecek seyreltme adimi.

    Planlayici her zaman tam cozunurluk goruyor; bu yalnizca cizim icin.
    100 km karede yarim milyon post var, Plotly yuzeyi orada kilitleniyor.
    """
    posts = terrain.cols * terrain.rows
    if posts <= DISPLAY_POSTS:
        return 1
    return math.ceil(math.sqrt(posts / DISPLAY_POSTS))


def terrain_payload():
    """Etkin pencereyi tarayicinin cizebilecegi bicimde verir; satir 0 guney.

    Arazi yoksa izgara bos doner ve has_terrain false olur; arayuz duz
    zemin cizip irtifayi MSL olarak gosteriyor.
    """
    with SESSION.lock:
        terrain, bounds, frame = SESSION.terrain, SESSION.bounds, SESSION.frame
        width, height = SESSION.size
        missing = list(SESSION.missing)
        source = SESSION.source

    if terrain is None:
        return {
            "has_terrain": False, "missing": missing, "source": source,
            "cols": 0, "rows": 0, "spacing_x": 0.0, "spacing_y": 0.0,
            "stride": 1, "full_cols": 0, "full_rows": 0,
            "extent_x": width, "extent_y": height,
            "lat": frame.lat, "lon": frame.lon, "mid_lat": frame.mid_lat,
            "low": bounds[2], "high": bounds[2], "ceiling": bounds[5],
            "heights": [],
        }

    stride = _display_stride(terrain)
    cols = list(range(0, terrain.cols, stride))
    rows = list(range(0, terrain.rows, stride))
    low, high = terrain.elevation_range()
    return {
        "has_terrain": True, "missing": missing, "source": source,
        "cols": len(cols),
        "rows": len(rows),
        # Seyreltilmis izgaranin post araligi da o kadar buyuyor; tarayici
        # metre koordinatlarini bununla kuruyor, aksi halde harita kuculur.
        "spacing_x": terrain.spacing_x * stride,
        "spacing_y": terrain.spacing_y * stride,
        "extent_x": terrain.extent_x,
        "extent_y": terrain.extent_y,
        "stride": stride,
        "full_cols": terrain.cols,
        "full_rows": terrain.rows,
        "lat": frame.lat,
        "lon": frame.lon,
        "mid_lat": frame.mid_lat,
        "low": low,
        "high": high,
        "ceiling": bounds[5],
        "heights": [[terrain.at(col, row) for col in cols] for row in rows],
    }


def coverage_payload():
    """Kutuphanedeki karolar; dunya haritasinda dikdortgen olarak cizilecek."""
    return {"tiles": [{"lat": lat, "lon": lon, "name": tile_name(lat, lon)}
                      for lat, lon in STORE.coverage()],
            "max_window": MAX_WINDOW}


def _constraints_from(body):
    return Constraints(
        speed=float(body.get("speed", 28.0)),
        max_bank=math.radians(float(body.get("bank_deg", 30.0))),
        max_climb=math.radians(float(body.get("climb_deg", 8.0))),
        clearance=float(body.get("clearance", 10.0)),
        max_iterations=int(body.get("iterations", 3000)),
        seed=int(body.get("seed", 1)),
    )


def _run_2d(points, frame, peak, bounds, zones, constraints, headings):
    """Yatay rota, tek seyir kotu; arazi HESABA KATILMIYOR.

    Kot girilmiyor: arazi varsa pencerenin en yuksek noktasinin uzerinde,
    yoksa sabit MSL. Arazi istemedigi icin dunyanin her yerinde kosuyor -
    3B'nin kurulamadigi ve takildigi durumlarin yedegi bu.
    """
    cruise = (auto_altitude(peak, constraints.clearance)
              if peak is not None else NO_TERRAIN_CRUISE)
    env = build_env_2d((0.0, 0.0, bounds[3], bounds[4]), constraints,
                       tuple(zone.as_obstacle() for zone in zones))
    waypoints = [frame.to_local(*point) for point in points]
    for index, point in enumerate(waypoints):
        if not env.is_free(point):
            raise ValueError(f"{index + 1}. waypoint yasak bolgenin icinde "
                             f"ya da harita disinda")
    mission = plan_mission_2d(waypoints, env, constraints, cruise,
                              headings)
    # Yuk hep (x, y, z) olsun: 3B gorunum waypointleri seyir kotunda
    # cizebilsin diye.
    return mission, [(x, y, cruise) for x, y in waypoints]


def _run_3d(flat, altitudes, pinned, terrain, bounds, zones, constraints,
            headings):
    """Araziden ve yasak bolgelerden kacinan uc boyutlu rota.

    Kotlar disarida hesaplandi: harita tavani onlara bagli ve tavan
    bolgelerden once kurulmak zorunda.
    """
    env = build_env(bounds, terrain, constraints,
                    tuple(zone.as_cylinder() for zone in zones))
    waypoints = [(x, y, z) for (x, y), z in zip(flat, altitudes)]
    for index, point in enumerate(waypoints):
        if env.is_free(point):
            continue
        # Kot ya otomatik ya da emniyet payi kontrolunden gecmis: arazi
        # sebep olamaz, geriye yasak bolge ve harita disi kalir.
        how = "sabit" if pinned[index] is not None else "otomatik"
        raise ValueError(f"{index + 1}. waypoint yasak bolgenin icinde ya "
                         f"da harita disinda (kot {how}: {point[2]:.0f} m). "
                         f"Waypointi kaydir ya da bolgeyi kucult.")
    return (plan_mission(waypoints, env, constraints, headings),
            waypoints)


def plan_payload(body):
    """Enlem/boylam waypointlerinden rota uretir.

    Pencere elle secilmiyor: waypointlerin sinir kutusundan turetiliyor,
    arazi varsa yukleniyor, yoksa pencere arazisiz kuruluyor ve irtifa MSL
    okunuyor. Girdi hatalari ValueError atiyor.
    """
    raw = body.get("waypoints", [])
    if len(raw) < 2:
        raise ValueError("en az iki waypoint gerekli")
    if len(raw) > MAX_WAYPOINTS:
        raise ValueError(f"en fazla {MAX_WAYPOINTS} waypoint "
                         f"(her bacak ayri bir planlama kosusu)")

    # Mod artik kullanicinin sectigi bir sey degil, arazinin sonucu.
    # Tek istisna: karsilastirma ya da 3B takildiginda elle 2B'ye inmek.
    ignore_terrain = bool(body.get("ignore_terrain", False))

    points = [(float(p[0]), float(p[1])) for p in raw]     # (enlem, boylam)
    # Ucuncu eleman istege bagli kot sabitlemesi, AGL. Yok ya da null ise
    # o waypointin kotu otomatik. 2B'de yok sayiliyor: orada butun rota
    # tek kotta, waypoint basina irtifa tanimsiz.
    pinned = [float(p[2]) if len(p) > 2 and p[2] is not None else None
              for p in raw]
    # Dorduncu eleman istege bagli bas acisi, PUSULA derecesi (0 kuzey).
    # Cerceve yaw'i dogudan saat tersine oldugu icin cevriliyor; ikisini
    # karistirmak 90 derece sessiz hata olurdu.
    headings = [compass_to_yaw(float(p[3]))
                if len(p) > 3 and p[3] is not None else None for p in raw]
    constraints = _constraints_from(body)

    margin = max(WINDOW_MARGIN, 4 * constraints.rho)
    lat, lon, width, height = geo_window(points, margin, MAX_WINDOW)
    terrain = SESSION.select(lat, lon, width, height)
    with SESSION.lock:                      # plan boyunca pencere sabit
        bounds, frame = SESSION.bounds, SESSION.frame
        missing = list(SESSION.missing)
        source = SESSION.source

    # Arazi yoksa 3B zaten kurulamaz; hata degil, 2B'ye dusuyoruz. Bu
    # uygulamanin dunyanin her yerinde calisabilmesinin sarti.
    mode = "2d" if (terrain is None or ignore_terrain) else "3d"

    # Pencerenin en yuksek postu: hem 2B seyir kotu hem harita tavani
    # bundan turuyor. min/max butun izgarayi tariyor, bir kez hesapla.
    peak = terrain.elevation_range()[1] if terrain is not None else None

    # 3B kotlar tavandan ONCE hesaplaniyor: sabitlenen bir waypoint
    # tavani zorlayabiliyor ve yasak bolgeler tavana gore kuruluyor.
    # Sira bozulursa bolgeler tavanin altinda kalir ve ucak ustlerinden
    # gecer - istenmeyen davranis.
    flat, altitudes = [], []
    if mode == "3d":
        flat = [frame.to_local(*point) for point in points]
        altitudes = waypoint_altitudes(
            [terrain.elevation_at(x, y) for x, y in flat],
            constraints.clearance, pinned)

    if peak is not None:
        # Otomatik kot eski tavanin (tepe + 400 m) ustune cikabiliyor;
        # ciktiginda is_free waypointi "harita disinda" sayar ve hatanin
        # sebebi gorunmez. Tavan kuralla birlikte yukseliyor.
        ceiling = auto_altitude(peak, constraints.clearance) + CLIMB_ROOM
        if altitudes:
            ceiling = max(ceiling, max(altitudes) + CLIMB_ROOM)
        bounds = bounds[:5] + (max(bounds[5], ceiling),)

    zones = _zones_from(body, frame, bounds)

    fell_back = False
    if mode == "3d":
        mission, waypoints = _run_3d(flat, altitudes, pinned, terrain,
                                     bounds, zones, constraints, headings)
        # Tirmanma sinirinin kaldiramadigi bir sirt butun gorevi
        # cope atmasin: araziyi yok sayan rota hala bir sonuc, yeter ki
        # oyle oldugu soylensin. Arayuz bunu yuksek sesle yaziyor.
        if not mission.found:
            mode, fell_back = "2d", True

    if mode == "2d":
        mission, waypoints = _run_2d(points, frame, peak, bounds, zones,
                                     constraints, headings)

    step = constraints.speed * FRAME_SECONDS

    # Bacaklar ayri ayri veriliyor: arayuz planlanamayani kesikli cizip
    # listede isaretleyebilsin. Animasyon dizisi bunlarin birlesimi.
    legs = []
    for leg in mission.legs:
        poses = sample_leg(leg, step) if leg.found else []
        legs.append({
            "found": leg.found,
            "cost": leg.cost if leg.found else None,
            "path": [_round_pose(pose) for pose in poses],
        })

    poses = sample_mission(mission, step)
    agls = agl_profile(poses, terrain)
    # constraints.speed HAVA hizi: rho hesabi da ondan turuyor. Yer hizi
    # ruzgardan cikiyor ve rota boyunca degisiyor.
    wind = _wind_payload(poses, constraints.speed,
                         float(body.get("wind_speed", 0.0)),
                         float(body.get("wind_from", 0.0)))
    # Gorev dosyasi yukun icinde gidiyor: ayri uc nokta acmak sunucuda
    # plan saklamayi gerektirirdi. Seyreltilmis dosya birkac KB.
    mission_file = (_mission_payload(poses, frame, terrain)
                    if mission.found else None)
    failed = [index + 1 for index, leg in enumerate(mission.legs)
              if not leg.found]

    return {
        "ok": mission.found,
        "mode": mode,
        # Arayuz "istedigim bu muydu" sorusunu sorabilsin diye: mod artik
        # secilmiyor, bu iki bayrak nicin o modda kosuldugunu anlatiyor.
        "fell_back": fell_back,
        "forced_2d": ignore_terrain,
        "rho": constraints.rho,
        "cost": mission.cost if mission.found else None,
        # Iki sure birden: biri sakin hava (uzunluk / hava hizi), digeri
        # ruzgar altinda. Arayuz hangisini gosterdigini yazmak zorunda.
        "duration": mission.duration if mission.found else None,
        "wind": wind,
        "mission_file": mission_file,
        "frame_seconds": FRAME_SECONDS,
        # Arayuz yerel metreyi enlem/boylama kendi ceviriyor; rotayi iki
        # kez gondermek yukun yarisini bosa harcardi.
        "frame": {"lat": frame.lat, "lon": frame.lon,
                  "mid_lat": frame.mid_lat},
        "extent_x": bounds[3], "extent_y": bounds[4],
        "low": bounds[2], "ceiling": bounds[5],
        "has_terrain": terrain is not None,
        "missing": missing,
        "terrain_source": source,
        # Post araligi pencereyle degistigi icin arayuz bunu SOYLEMEK
        # zorunda: emniyet payi kabalasmis bir zeminden olculuyorsa
        # kullanici bilmeli.
        "terrain_spacing": (max(terrain.spacing_x, terrain.spacing_y)
                            if terrain is not None else None),
        "waypoints": [[round(point[0], 2), round(point[1], 2),
                       round(point[2], 1)] for point in waypoints],
        # Kullanilan bas acilari, pusula derecesi. Sabitlenmemis olanlar
        # aciortaydan cikiyor; arayuz ikisini de gosteriyor.
        "headings": [round(yaw_to_compass(y), 1)
                     for y in waypoint_headings(
                         [(p[0], p[1]) for p in waypoints], headings)],
        "zones": [{"x": zone.x, "y": zone.y, "radius": zone.radius}
                  for zone in zones],
        "terrain": (terrain_payload() if terrain is not None
                    and body.get("want_terrain") else None),
        "legs": legs,
        "path": [_round_pose(pose) for pose in poses],
        # Arazi yoksa AGL yok: irtifa MSL ve zemin dogrulanmamis.
        "agl": {"min": min(agls), "max": max(agls),
                "mean": sum(agls) / len(agls)} if agls else None,
        # Kot artik girilmiyor; rotayi yukselten tek kol emniyet payi.
        "message": ("" if mission.found else
                    f"{', '.join(str(n) for n in failed)}. bacak "
                    f"planlanamadi; emniyet payini artir (rotayi yukseltir), "
                    f"yineleme ust sinirini yukselt ya da waypoint'i kaydir"),
    }


def _wind_payload(poses, airspeed, speed, from_deg):
    """Ruzgar altinda sure ve yer hizi araligi.

    Ruzgar rotanin GEOMETRISINI degistirmiyor; burada yalnizca o rotanin
    ne kadar surecegi yeniden hesaplaniyor. Yan ruzgar hava hizini asarsa
    rota gecerli kalir ama UCULAMAZ - plani cope atmak yerine sebebi
    soyleyip rotayi geri veriyoruz.
    """
    payload = {"speed": speed, "from_deg": from_deg, "duration": None,
               "min_speed": None, "max_speed": None, "message": ""}
    if not poses:
        return payload

    wind = wind_vector(speed, from_deg)
    try:
        payload["duration"] = flight_time(poses, airspeed, wind)
        speeds = ground_speeds(poses, airspeed, wind)
    except ValueError as error:
        payload["message"] = str(error)
        return payload

    if speeds:
        payload["min_speed"] = min(speeds)
        payload["max_speed"] = max(speeds)
    return payload


def _mission_payload(poses, frame, terrain):
    """Yogun rotayi otopilotun yukleyebilecegi gorev dosyasina cevirir.

    Home kotu ZEMIN olmali, ucus kotu degil: home kalkis noktasi. Arazi
    yoksa ilk waypointin kotu kaliyor ve dosya yine yuklenir, yalnizca
    home irtifasi otopilotun kendi olcumune birakilmis olur.
    """
    if not poses:
        return {"text": "", "points": 0, "deviation": None,
                "tolerance": WPL_TOLERANCE}

    kept = simplify(poses, WPL_TOLERANCE)
    waypoints = []
    for index in kept:
        x, y, z = poses[index][0], poses[index][1], poses[index][2]
        lat, lon = frame.to_geo(x, y)
        waypoints.append((lat, lon, z))

    home = waypoints[0]
    if terrain is not None:
        ground = terrain.elevation_at(poses[kept[0]][0], poses[kept[0]][1])
        home = (home[0], home[1], ground)

    return {"text": mission_text(waypoints, home=home),
            "points": len(waypoints),
            "deviation": deviation(poses, kept),
            "tolerance": WPL_TOLERANCE}


def _zones_from(body, frame, bounds):
    """Enlem/boylam yasak bolgeleri yerel metreye cevirir.

    Pencere disinda kalan bolge atilmiyor, birakiliyor: yaricapi pencereye
    tasabilir ve kenardan kesiyor olabilir.
    """
    raw = body.get("zones", []) or []
    if len(raw) > MAX_ZONES:
        raise ValueError(f"en fazla {MAX_ZONES} yasak bolge")

    zones = []
    for index, item in enumerate(raw):
        radius = float(item["radius_m"])
        if radius <= 0.0:
            raise ValueError(
                f"{index + 1}. yasak bolgenin yaricapi pozitif olmali")
        x, y = frame.to_local(float(item["lat"]), float(item["lon"]))
        zones.append(Zone(x, y, radius,
                          float(item.get("z_min", bounds[2])),
                          float(item.get("z_max", bounds[5]))))
    return zones


def _round_pose(pose):
    """Yuk boyutunu kucultur; yaw Dubins boyunca birikip 2*pi'yi asabiliyor."""
    return [round(pose[0], 2), round(pose[1], 2), round(pose[2], 2),
            round(pose[3] % (2 * math.pi), 4)]


class Handler(BaseHTTPRequestHandler):
    server_version = "UavMissionPlanner/1.0"

    def log_message(self, fmt, *args):        # varsayilan log cok gurultulu
        if "POST" in fmt % args:
            print(f"  {self.address_string()} {fmt % args}")

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_static(self, relative):
        # posixpath.normpath + basename zinciri ".." ile disari cikmayi keser
        safe = posixpath.normpath("/" + relative).lstrip("/")
        path = os.path.join(STATIC_DIR, safe)
        if not os.path.isfile(path):
            self._send_json({"error": f"bulunamadi: {relative}"}, 404)
            return
        kind = mimetypes.guess_type(path)[0] or "application/octet-stream"
        # Metin dosyalarinda karakter kumesini soylemek sart: soylenmezse
        # tarayici belge kodlamasina duser ve atif satirindaki telif
        # isareti bozuk gorunur.
        if kind.startswith("text/") or kind in ("application/javascript",
                                                "application/json",
                                                "application/geo+json"):
            kind += "; charset=utf-8"
        with open(path, "rb") as handle:
            body = handle.read()
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        # Gelistirme araci: app.js degistikten sonra tarayicinin eski
        # surumu gostermesi saatlerce yanlis yerde hata aratir.
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = self.path.split("?", 1)[0]
        if route == "/":
            self._send_static("index.html")
        elif route == "/api/coverage":
            self._send_json(coverage_payload())
        else:
            self._send_static(route.lstrip("/"))

    def do_POST(self):
        route = self.path.split("?", 1)[0]
        handlers = {"/api/plan": plan_payload}
        if route not in handlers:
            self._send_json({"error": "bilinmeyen uc nokta"}, 404)
            return
        length = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError as error:
            self._send_json({"ok": False, "message": f"bozuk JSON: {error}"},
                            400)
            return
        try:
            self._send_json(handlers[route](body))
        # Bozuk govde 500 vermemeli: eksik alan, yanlis tip ve kisa
        # waypoint dizisi de kullaniciya okunur bir mesaj olmali.
        except (ValueError, KeyError, IndexError, TypeError) as error:
            self._send_json({"ok": False, "message": str(error)}, 400)


class Server(ThreadingHTTPServer):
    # Windows'ta SO_REUSEADDR ikinci bir sunucunun ayni porta baglanmasina
    # izin veriyor ve istekler sessizce eski surece gidiyor - kod
    # degistirdikten sonra "neden eski davranis" diye saatler yakan bir
    # tuzak. Kapatiyoruz ki ikinci baslatma yuksek sesle hata versin.
    allow_reuse_address = False


def main():
    # Karo olmamasi artik olumcul degil: 2B planlama arazi istemiyor ve
    # uygulama dunyanin her yerinde acilabilmeli.
    tiles = STORE.coverage()
    if tiles:
        lats = sorted({lat for lat, _ in tiles})
        lons = sorted({lon for _, lon in tiles})
        print(f"karo kutuphanesi: {len(tiles)} karo, "
              f"{lats[0]}-{lats[-1]} N / {lons[0]}-{lons[-1]} D", flush=True)
    else:
        print(f"karo bulunamadi ({', '.join(TILE_SOURCES)}); "
              f"3B kapali, 2B her yerde calisir", flush=True)

    terrain = SESSION.select(START_LAT, START_LON, START_SIZE, START_SIZE)
    if terrain is None:
        print(f"acilis penceresi arazisiz: eksik karo "
              f"{', '.join(SESSION.missing)}", flush=True)
    else:
        low, high = terrain.elevation_range()
        print(f"acilis penceresi: {terrain.cols} x {terrain.rows} post, "
              f"{terrain.extent_x:.0f} x {terrain.extent_y:.0f} m, "
              f"kot {low:.0f}-{high:.0f} m "
              f"(cizim seyreltmesi {_display_stride(terrain)})", flush=True)
    try:
        server = Server((HOST, PORT), Handler)
    except OSError as error:
        raise SystemExit(
            f"{PORT} portu mesgul ({error}). Onceki sunucu hala calisiyor "
            f"olabilir; onu kapatip tekrar dene.") from error
    print(f"tarayicida ac: http://{HOST}:{PORT}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nkapatiliyor", flush=True)


if __name__ == "__main__":
    main()

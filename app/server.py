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

from app.mission import (Constraints, Zone, agl_profile, build_env,
                         build_env_2d, geo_window, plan_mission,
                         plan_mission_2d, sample_leg, sample_mission)
from src.geo import Frame
from src.terrain import MissingTiles, TileStore, tile_name

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

MAX_WINDOW = 60000.0           # tek kenar; ustunde planlama dakikalara cikiyor
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
DEFAULT_CRUISE = 1000.0        # 2B seyir irtifasi, MSL

STORE = TileStore(TILE_SOURCES)


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

    def select(self, lat, lon, width_m, height_m):
        try:
            terrain = STORE.window(lat, lon, width_m, height_m)
            missing = []
        except MissingTiles as error:
            terrain, missing = None, error.names

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
        return terrain


SESSION = Session()


def _display_stride(terrain):
    """Tarayiciya gonderilecek seyreltme adimi.

    Planlayici her zaman tam cozunurluk goruyor; bu yalnizca cizim icin.
    60 km karede 613 bin post var, Plotly yuzeyi orada kilitleniyor.
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

    if terrain is None:
        return {
            "has_terrain": False, "missing": missing,
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
        "has_terrain": True, "missing": [],
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
        clearance=float(body.get("clearance", 100.0)),
        max_iterations=int(body.get("iterations", 3000)),
        seed=int(body.get("seed", 1)),
    )


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

    mode = str(body.get("mode", "2d")).lower()
    if mode not in ("2d", "3d"):
        raise ValueError(f"mod '2d' ya da '3d' olmali: {mode}")

    points = [(float(p[0]), float(p[1])) for p in raw]     # (enlem, boylam)
    altitudes = [float(p[2]) for p in raw]
    constraints = _constraints_from(body)

    margin = max(WINDOW_MARGIN, 4 * constraints.rho)
    lat, lon, width, height = geo_window(points, margin, MAX_WINDOW)
    terrain = SESSION.select(lat, lon, width, height)
    with SESSION.lock:                      # plan boyunca pencere sabit
        bounds, frame = SESSION.bounds, SESSION.frame
        missing = list(SESSION.missing)

    if mode == "3d" and terrain is None:
        raise ValueError(
            f"3B mod arazi verisi istiyor, bu bolgede yok "
            f"(eksik karo: {', '.join(missing)}). 2B modda planlayabilirsin.")

    zones = _zones_from(body, frame, bounds)

    if mode == "2d":
        # 2B: yatay rota, sabit seyir irtifasi, arazi HESABA KATILMIYOR.
        cruise = float(body.get("cruise", DEFAULT_CRUISE))
        env = build_env_2d((0.0, 0.0, bounds[3], bounds[4]), constraints,
                           tuple(zone.as_obstacle() for zone in zones))
        waypoints = [frame.to_local(*point) for point in points]
        for index, point in enumerate(waypoints):
            if env.is_free(point):
                continue
            raise ValueError(
                f"{index + 1}. waypoint yasak bolgenin icinde ya da "
                f"harita disinda")
        mission = plan_mission_2d(waypoints, env, constraints, cruise)
        # Yuk hep (x, y, z) olsun: 3B gorunum waypointleri seyir
        # irtifasinda cizebilsin diye.
        waypoints = [(x, y, cruise) for x, y in waypoints]
    else:
        # Arazi varsa irtifa AGL olarak okunuyor.
        env = build_env(bounds, terrain, constraints,
                        tuple(zone.as_cylinder() for zone in zones))
        waypoints = []
        for (point_lat, point_lon), altitude in zip(points, altitudes):
            x, y = frame.to_local(point_lat, point_lon)
            waypoints.append((x, y, terrain.elevation_at(x, y) + altitude))
        for index, point in enumerate(waypoints):
            if env.is_free(point):
                continue
            ground = terrain.elevation_at(point[0], point[1])
            raise ValueError(
                f"{index + 1}. waypoint gecersiz: irtifa {point[2]:.0f} m, "
                f"zemin {ground:.0f} m, gereken en az "
                f"{ground + constraints.clearance:.0f} m")
        mission = plan_mission(waypoints, env, constraints)

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
    failed = [index + 1 for index, leg in enumerate(mission.legs)
              if not leg.found]

    return {
        "ok": mission.found,
        "mode": mode,
        "rho": constraints.rho,
        "cost": mission.cost if mission.found else None,
        "duration": mission.duration if mission.found else None,
        "frame_seconds": FRAME_SECONDS,
        # Arayuz yerel metreyi enlem/boylama kendi ceviriyor; rotayi iki
        # kez gondermek yukun yarisini bosa harcardi.
        "frame": {"lat": frame.lat, "lon": frame.lon,
                  "mid_lat": frame.mid_lat},
        "extent_x": bounds[3], "extent_y": bounds[4],
        "low": bounds[2], "ceiling": bounds[5],
        "has_terrain": terrain is not None,
        "missing": missing,
        "waypoints": [[round(point[0], 2), round(point[1], 2),
                       round(point[2], 1)] for point in waypoints],
        "zones": [{"x": zone.x, "y": zone.y, "radius": zone.radius}
                  for zone in zones],
        "terrain": (terrain_payload() if terrain is not None
                    and body.get("want_terrain") else None),
        "legs": legs,
        "path": [_round_pose(pose) for pose in poses],
        # Arazi yoksa AGL yok: irtifa MSL ve zemin dogrulanmamis.
        "agl": {"min": min(agls), "max": max(agls),
                "mean": sum(agls) / len(agls)} if agls else None,
        "message": ("" if mission.found else
                    f"{', '.join(str(n) for n in failed)}. bacak "
                    f"planlanamadi; irtifayi yukselt, yineleme sayisini "
                    f"artir ya da waypoint'i kaydir"),
    }


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

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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from app.mission import (Constraints, agl_profile, build_env, plan_mission,
                         sample_leg, sample_mission)
from src.terrain import load_terrain, synthetic_terrain

HOST, PORT = "127.0.0.1", 8000
STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")

# Arazi penceresi. Buyutmek istersen tek yer burasi; karo 1 derece kare
# oldugu icin (46.15, 14.35) kosesinden ~40 km'ye kadar yer var. Ust sinir
# tarayici: 20 km karede 67 bin post ve ~335 KB JSON, Plotly rahat cizyor;
# 30 km'de 152 bin posta cikiyor ve yuzey agirlasiyor.
TERRAIN_FILE = "data/N46E014.hgl"
TERRAIN_LAT, TERRAIN_LON = 46.15, 14.35
TERRAIN_WIDTH = TERRAIN_HEIGHT = 20000.0

CEILING_ABOVE_PEAK = 400.0     # harita tavani: en yuksek tepe + bu kadar
FRAME_SECONDS = 0.5            # animasyon karesi; ornek araligi hiz * bu
MAX_WAYPOINTS = 12             # her bacak ayri bir RRT* kosusu


def _load_terrain():
    """Arazi bir kez okunuyor; her istekte 2.9 MB dosya acmanin anlami yok."""
    import random
    if os.path.exists(TERRAIN_FILE):
        return load_terrain(TERRAIN_FILE, TERRAIN_LAT, TERRAIN_LON,
                            TERRAIN_WIDTH, TERRAIN_HEIGHT)
    spacing = 111320.0 / 3600 * 3 * math.cos(math.radians(TERRAIN_LAT))
    return synthetic_terrain(157, 108, spacing, random.Random(11),
                             base=369.0, relief=1845.0, peaks=6)


TERRAIN = _load_terrain()
LOW, HIGH = TERRAIN.elevation_range()
BOUNDS = (0.0, 0.0, LOW, TERRAIN.extent_x, TERRAIN.extent_y,
          HIGH + CEILING_ABOVE_PEAK)


def terrain_payload():
    """Izgarayi tarayicinin cizebilecegi bicimde verir; satir 0 guney."""
    rows = [[TERRAIN.at(col, row) for col in range(TERRAIN.cols)]
            for row in range(TERRAIN.rows)]
    return {
        "cols": TERRAIN.cols,
        "rows": TERRAIN.rows,
        "spacing_x": TERRAIN.spacing_x,
        "spacing_y": TERRAIN.spacing_y,
        "extent_x": TERRAIN.extent_x,
        "extent_y": TERRAIN.extent_y,
        "low": LOW,
        "high": HIGH,
        "ceiling": BOUNDS[5],
        "heights": rows,
    }


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
    """Istek govdesinden rota uretir. Girdi hatalari ValueError atiyor."""
    waypoints = [(float(p[0]), float(p[1]), float(p[2]))
                 for p in body.get("waypoints", [])]
    if len(waypoints) < 2:
        raise ValueError("en az iki waypoint gerekli")
    if len(waypoints) > MAX_WAYPOINTS:
        raise ValueError(f"en fazla {MAX_WAYPOINTS} waypoint "
                         f"(her bacak ayri bir planlama kosusu)")

    constraints = _constraints_from(body)
    env = build_env(BOUNDS, TERRAIN, constraints)
    for index, point in enumerate(waypoints):
        if not env.is_free(point):
            ground = TERRAIN.elevation_at(point[0], point[1])
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
    agls = agl_profile(poses, TERRAIN)
    failed = [index + 1 for index, leg in enumerate(mission.legs)
              if not leg.found]

    return {
        "ok": mission.found,
        "rho": constraints.rho,
        "cost": mission.cost if mission.found else None,
        "duration": mission.duration if mission.found else None,
        "frame_seconds": FRAME_SECONDS,
        "legs": legs,
        "path": [_round_pose(pose) for pose in poses],
        "agl": {"min": min(agls), "max": max(agls),
                "mean": sum(agls) / len(agls)} if agls else None,
        "message": ("" if mission.found else
                    f"{', '.join(str(n) for n in failed)}. bacak "
                    f"planlanamadi; irtifayi yukselt, yineleme sayisini "
                    f"artir ya da waypoint'i kaydir"),
    }


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
        elif route == "/api/terrain":
            self._send_json(terrain_payload())
        else:
            self._send_static(route.lstrip("/"))

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/api/plan":
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
            self._send_json(plan_payload(body))
        except ValueError as error:
            self._send_json({"ok": False, "message": str(error)}, 400)


class Server(ThreadingHTTPServer):
    # Windows'ta SO_REUSEADDR ikinci bir sunucunun ayni porta baglanmasina
    # izin veriyor ve istekler sessizce eski surece gidiyor - kod
    # degistirdikten sonra "neden eski davranis" diye saatler yakan bir
    # tuzak. Kapatiyoruz ki ikinci baslatma yuksek sesle hata versin.
    allow_reuse_address = False


def main():
    print(f"arazi: {TERRAIN.cols} x {TERRAIN.rows} post, "
          f"{TERRAIN.extent_x:.0f} x {TERRAIN.extent_y:.0f} m, "
          f"kot {LOW:.0f}-{HIGH:.0f} m", flush=True)
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

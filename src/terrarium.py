"""Terrarium yukseklik karolarindan Terrain uretir - kuresel 3B kapsama.

SRTM'in tamami onlarca GB, indirilemez. Bu modul yalnizca gorevin dustugu
pencerenin karolarini cekip diske onbellekliyor; ayni bolge ikinci kez
planlanirsa internet gerekmiyor.

Karolar PNG ve kot RGB'ye kodlu:

    kot = R * 256 + G + B / 256 - 32768        (metre)

Karo izgarasi Web Mercator, bizim Terrain'imiz derece izgarasi; ikisi ayni
sey degil, o yuzden her post icin Mercator piksel konumu hesaplanip cift
dogrusal ornekleniyor.

Ag erisimi disaridan verilebilen tek bir islevde toplandi (fetch); testler
ona sahte bir islev verip agdan tamamen bagimsiz kaliyor.
"""

import array
import concurrent.futures
import math
import os
import threading
import urllib.error
import urllib.request

from src.geo import METRES_PER_DEGREE
from src.png import decode
from src.terrain import Terrain

TILE_SIZE = 256
DEFAULT_URL = ("https://s3.amazonaws.com/elevation-tiles-prod/terrarium/"
               "{z}/{x}/{y}.png")
# Ekvatorda zoom 0'da bir pikselin yer karsiligi, metre.
EQUATOR_RESOLUTION = 2 * math.pi * 6378137.0 / TILE_SIZE

MIN_ZOOM, MAX_ZOOM = 6, 14
# Cagiran taraf post araligini pencereye gore secmeli: sabit 90 m'de
# 120 km'lik pencere 90 karo ister ve burada reddedilir.
MAX_TILES = 64                 # 100 km pencere zoom 10'da ~25 karo
TIMEOUT = 20.0
# Karo basina ~1 saniye; 36 karolu bir pencere sirayla yarim dakika surer.
# Is agi beklemek oldugu icin is parcaciklari GIL'e takilmiyor.
WORKERS = 8


class ElevationUnavailable(RuntimeError):
    """Karo ne onbellekte ne de agdan alinabildi."""


def resolution(zoom: int, lat: float) -> float:
    """Verilen zoom ve enlemde bir pikselin yer karsiligi, metre."""
    return EQUATOR_RESOLUTION * math.cos(math.radians(lat)) / (2 ** zoom)


def zoom_for(spacing_m: float, lat: float) -> int:
    """Post araligini karsilayan en kucuk zoom.

    Karo cozunurlugu istenen araliktan INCE olmali; kaba karodan ince
    izgara uretmek veri yokken varmis gibi gosterirdi.
    """
    if spacing_m <= 0.0:
        raise ValueError(f"post araligi pozitif olmali: {spacing_m}")
    ideal = math.log2(EQUATOR_RESOLUTION * math.cos(math.radians(lat))
                      / spacing_m)
    return max(MIN_ZOOM, min(MAX_ZOOM, math.ceil(ideal)))


def pixel_of(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Enlem/boylamin kuresel Mercator piksel konumu (kesirli)."""
    scale = TILE_SIZE * (2 ** zoom)
    sin_lat = math.sin(math.radians(lat))
    # Kutupta log sonsuza gidiyor; Mercator zaten +-85 disini gostermiyor.
    sin_lat = max(-0.9999, min(0.9999, sin_lat))
    return ((lon + 180.0) / 360.0 * scale,
            (0.5 - math.log((1 + sin_lat) / (1 - sin_lat)) / (4 * math.pi))
            * scale)


def decode_tile(data: bytes) -> list[float]:
    """PNG karoyu satir satir kot listesine cevirir (kuzeyden guneye)."""
    width, height, channels, pixels = decode(data)
    if width != TILE_SIZE or height != TILE_SIZE:
        raise ValueError(f"karo {TILE_SIZE}x{TILE_SIZE} olmali: "
                         f"{width}x{height}")
    if channels < 3:
        raise ValueError(f"karo en az uc kanalli olmali: {channels}")

    out = []
    for index in range(0, width * height * channels, channels):
        out.append(pixels[index] * 256.0 + pixels[index + 1]
                   + pixels[index + 2] / 256.0 - 32768.0)
    return out


def _http_get(url: str) -> bytes:
    request = urllib.request.Request(
        url, headers={"User-Agent": "uav-path-planning/0.1"})
    with urllib.request.urlopen(request, timeout=TIMEOUT) as response:
        return response.read()


class TerrariumSource:
    """Karo indirici ve disk onbellegi.

    fetch disaridan verilebiliyor: testler agdan bagimsiz kalsin ve
    onbellek davranisi tek basina sinanabilsin diye.
    """

    def __init__(self, cache_dir: str = os.path.join("data", "cache"),
                 url_template: str = DEFAULT_URL, fetch=None):
        self.cache_dir = cache_dir
        self.url_template = url_template
        self._fetch = fetch if fetch is not None else _http_get
        self.downloads = 0             # olcum icin: kac karo agdan geldi
        self._lock = threading.Lock()

    def _path(self, zoom: int, x: int, y: int) -> str:
        return os.path.join(self.cache_dir, "terrarium", str(zoom), str(x),
                            f"{y}.png")

    def tile(self, zoom: int, x: int, y: int) -> list[float]:
        """Karoyu onbellekten ya da agdan alip cozer."""
        path = self._path(zoom, x, y)
        if os.path.isfile(path):
            with open(path, "rb") as handle:
                return decode_tile(handle.read())

        url = self.url_template.format(z=zoom, x=x, y=y)
        try:
            data = self._fetch(url)
        except (urllib.error.URLError, OSError, ValueError) as error:
            raise ElevationUnavailable(
                f"yukseklik karosu alinamadi ({zoom}/{x}/{y}): {error}"
            ) from error

        # Once coz, sonra yaz: bozuk veri onbelleklenirse bir daha
        # kendiliginden duzelmez. Cagiran tek hata turu gormeli.
        try:
            heights = decode_tile(data)
        except ValueError as error:
            raise ElevationUnavailable(
                f"yukseklik karosu cozulemedi ({zoom}/{x}/{y}): {error}"
            ) from error

        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as handle:
            handle.write(data)
        with self._lock:
            self.downloads += 1
        return heights

    def window(self, lat: float, lon: float, width_m: float, height_m: float,
               spacing_m: float = 60.0) -> Terrain:
        """Pencereyi kapsayan karolari alip derece izgarasina cevirir.

        Uretilen Terrain, yerel .hgt/.hgl pencereleriyle ayni sozlesmeye
        uyuyor: satir 0 guney, olcek pencerenin ortasinda dondurulmus.
        """
        if width_m <= 0.0 or height_m <= 0.0:
            raise ValueError(f"pencere boyutlari pozitif olmali: "
                             f"{width_m} x {height_m}")

        mid_lat = lat + height_m / 2 / METRES_PER_DEGREE
        zoom = zoom_for(spacing_m, mid_lat)

        step_deg = spacing_m / METRES_PER_DEGREE
        spacing_y = spacing_m
        spacing_x = spacing_m * math.cos(math.radians(mid_lat))
        cols = int(width_m / spacing_x) + 1
        rows = int(height_m / spacing_y) + 1
        if cols < 2 or rows < 2:
            raise ValueError(f"pencere cok kucuk: {cols} x {rows} post")

        lat_hi = lat + (rows - 1) * step_deg
        lon_hi = lon + (cols - 1) * step_deg
        left, bottom = pixel_of(lat, lon, zoom)
        right, top = pixel_of(lat_hi, lon_hi, zoom)   # kuzey = kucuk piksel

        first_x = int(math.floor(left / TILE_SIZE))
        last_x = int(math.floor(right / TILE_SIZE))
        first_y = int(math.floor(top / TILE_SIZE))
        last_y = int(math.floor(bottom / TILE_SIZE))
        count = (last_x - first_x + 1) * (last_y - first_y + 1)
        if count > MAX_TILES:
            raise ValueError(
                f"pencere {count} karo gerektiriyor, sinir {MAX_TILES}; "
                f"alani kucult ya da post araligini buyut")

        mosaic = self._mosaic(zoom, first_x, last_x, first_y, last_y)
        mosaic_width = (last_x - first_x + 1) * TILE_SIZE
        mosaic_height = (last_y - first_y + 1) * TILE_SIZE
        origin_x = first_x * TILE_SIZE
        origin_y = first_y * TILE_SIZE

        heights = []
        for row in range(rows):
            post_lat = lat + row * step_deg
            for col in range(cols):
                px, py = pixel_of(post_lat, lon + col * step_deg, zoom)
                heights.append(_sample(mosaic, mosaic_width, mosaic_height,
                                       px - origin_x, py - origin_y))

        return Terrain(_to_grid(heights), cols, rows, spacing_x, spacing_y)

    def _mosaic(self, zoom, first_x, last_x, first_y, last_y) -> list[float]:
        """Karolari tek bir kot dizisine dizer; satir 0 en KUZEY.

        Karolar paralel indiriliyor: sirayla beklemek 36 karoluk bir
        pencerede yarim dakika ediyor.
        """
        wanted = [(x, y) for y in range(first_y, last_y + 1)
                  for x in range(first_x, last_x + 1)]
        with concurrent.futures.ThreadPoolExecutor(
                max_workers=min(WORKERS, len(wanted))) as pool:
            fetched = dict(zip(
                wanted,
                pool.map(lambda key: self.tile(zoom, key[0], key[1]), wanted)))

        width = (last_x - first_x + 1) * TILE_SIZE
        mosaic = [0.0] * (width * (last_y - first_y + 1) * TILE_SIZE)
        for y in range(first_y, last_y + 1):
            for x in range(first_x, last_x + 1):
                tile = fetched[(x, y)]
                base_col = (x - first_x) * TILE_SIZE
                base_row = (y - first_y) * TILE_SIZE
                for line in range(TILE_SIZE):
                    start = line * TILE_SIZE
                    out = (base_row + line) * width + base_col
                    mosaic[out:out + TILE_SIZE] = tile[start:start + TILE_SIZE]
        return mosaic


def _sample(mosaic, width, height, x, y) -> float:
    """Mozaikten cift dogrusal ornek; kenarda kirpiliyor."""
    x = min(max(x, 0.0), width - 1.0)
    y = min(max(y, 0.0), height - 1.0)
    col = min(int(x), width - 2)
    row = min(int(y), height - 2)
    tx, ty = x - col, y - row

    top_left = mosaic[row * width + col]
    top_right = mosaic[row * width + col + 1]
    bottom_left = mosaic[(row + 1) * width + col]
    bottom_right = mosaic[(row + 1) * width + col + 1]
    top = top_left + (top_right - top_left) * tx
    bottom = bottom_left + (bottom_right - bottom_left) * tx
    return top + (bottom - top) * ty


def _to_grid(heights):
    """Kot listesini Terrain'in bekledigi int16 dizisine cevirir."""
    return array.array("h", [int(round(value)) for value in heights])

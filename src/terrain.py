"""Yukseklik izgarasi: yerel ENU metrede arazi zemini.

Izgara satir satir tutuluyor ve satir 0 GUNEY kenari (y = 0), sutun 0 bati
kenari (x = 0). SRTM ve DTED dosyalari farkli yonlerde sakliyor; yukleyiciler
okurken bu duzene ceviriyor, boylece planlayici tek bir duzen goruyor.
"""

import array
import math
import os
import random
import re
import sys
from dataclasses import dataclass

# SRTM void isareti. Karadaki en alcak nokta -430 m (Lut Golu), bu yuzden
# -1000'in altindaki her sey veri yoklugu sayiliyor.
VOID_BELOW = -1000
METRES_PER_DEGREE = 111320.0

_HGL_MAGIC = b"LXNSRTM"
_HGL_HEADER = 512


@dataclass(frozen=True)
class Terrain:
    heights: array.array
    cols: int
    rows: int
    spacing_x: float
    spacing_y: float

    def __post_init__(self):
        if not len(self.heights) == self.cols * self.rows:
            raise ValueError(
                f"heights uzunlugu cols*rows olmali: {len(self.heights)} "
                f"!= {self.cols} * {self.rows}")
        if self.cols < 2 or self.rows < 2:
            raise ValueError(
                f"izgara en az 2x2 olmali: {self.cols} x {self.rows}")
        if self.spacing_x <= 0 or self.spacing_y <= 0:
            raise ValueError(
                f"post araligi pozitif olmali: {self.spacing_x}, "
                f"{self.spacing_y}")

    @property
    def extent_x(self) -> float:
        """Dogu-bati yayilim, metre. Post sayisi degil, aralik sayisi."""
        return (self.cols - 1) * self.spacing_x

    @property
    def extent_y(self) -> float:
        return (self.rows - 1) * self.spacing_y

    def at(self, col: int, row: int) -> int:
        """(col, row) postundaki ham yukseklik."""
        if not (0 <= col < self.cols) or not (0 <= row < self.rows):
            raise IndexError(f"izgara disi post: ({col}, {row})")
        return self.heights[row * self.cols + col]

    def elevation_range(self) -> tuple[float, float]:
        """En alcak ve en yuksek post."""
        return (float(min(self.heights)), float(max(self.heights)))

    def elevation_at(self, x: float, y: float) -> float:
        """(x, y) noktasindaki yukseklik; cift dogrusal enterpolasyon.

        En yakin postu almak 30-90 m'lik izgarada merdiven basamagi uretir
        ve rota o basamaklara takilir; ara deger sart. Harita disi kirpiliyor
        cunku sinir kontrolunu Environment3D zaten yapiyor.
        """
        fx = min(max(x, 0.0), self.extent_x) / self.spacing_x
        fy = min(max(y, 0.0), self.extent_y) / self.spacing_y

        # Tam kenarda alt kose son post olur ve sag/ust komsusu izgarayi
        # tasar; bir eksige kirpinca kesir 1.0 cikip ayni sonucu veriyor.
        col = min(int(fx), self.cols - 2)
        row = min(int(fy), self.rows - 2)
        tx = fx - col
        ty = fy - row

        south_west, south_east = self.at(col, row), self.at(col + 1, row)
        north_west = self.at(col, row + 1)
        north_east = self.at(col + 1, row + 1)

        south = south_west + (south_east - south_west) * tx
        north = north_west + (north_east - north_west) * tx
        return south + (north - south) * ty


def synthetic_terrain(cols: int, rows: int, spacing: float,
                      rng: random.Random, base: float = 0.0,
                      relief: float = 1000.0, peaks: int = 5) -> Terrain:
    """Gauss tepelerinden yumusak arazi; gercek dosya beklemeden calismak icin.

    Tepe genisligi haritanin kisa kenarina gore seciliyor - dar tepe ucurum
    demek, sabit kanat icin gercekci degil. Sonuc [base, base + relief]
    araligina normalize ediliyor, boylece ayarlar dogrudan metre cinsinden.
    """
    if cols < 2 or rows < 2:
        raise ValueError(f"izgara en az 2x2 olmali: {cols} x {rows}")
    if spacing <= 0:
        raise ValueError(f"post araligi pozitif olmali: {spacing}")
    if relief < 0:
        raise ValueError(f"relief negatif olamaz: {relief}")
    if peaks < 1:
        raise ValueError(f"en az bir tepe olmali: {peaks}")

    extent_x = (cols - 1) * spacing
    extent_y = (rows - 1) * spacing
    short_side = min(extent_x, extent_y)

    hills = [(rng.uniform(0.0, extent_x), rng.uniform(0.0, extent_y),
              rng.uniform(0.3, 1.0),
              rng.uniform(short_side / 6, short_side / 3))
             for _ in range(peaks)]

    raw = []
    for row in range(rows):
        y = row * spacing
        for col in range(cols):
            x = col * spacing
            total = 0.0
            for hill_x, hill_y, amplitude, sigma in hills:
                squared = (x - hill_x) ** 2 + (y - hill_y) ** 2
                total += amplitude * math.exp(-squared / (2 * sigma ** 2))
            raw.append(total)

    low = min(raw)
    span = max(raw) - low
    if span <= 0.0:                    # duz alan; sifira bolme
        flat = int(round(base))
        return Terrain(array.array("h", [flat] * len(raw)),cols, rows, spacing, spacing)

    heights = array.array("h", [int(round(base + (value - low) / span * relief))
                                for value in raw])
    return Terrain(heights, cols, rows, spacing, spacing)


def _tile_origin(path: str) -> tuple[int, int]:
    """Dosya adindan karonun guneybati kosesi: N46E014 -> (46, 14)."""
    match = re.search(r"([NS])(\d{2})([EW])(\d{3})",
                      os.path.basename(path).upper())
    if match is None:
        raise ValueError(
            f"dosya adindan karo kosesi okunamadi, N46E014 gibi olmali: {path}")
    lat = int(match.group(2)) * (1 if match.group(1) == "N" else -1)
    lon = int(match.group(4)) * (1 if match.group(3) == "E" else -1)
    return lat, lon


def _read_tile(path: str) -> tuple[array.array, int]:
    """Karoyu okur; (yukseklikler, kenar uzunlugu), satir 0 = KUZEY.

    Iki bicim ayni izgarayi tasiyor, farklari baslik ve bayt sirasi:
    .hgt basliksiz ve big-endian, .hgl 512 baytlik LXNSRTM basligiyla
    little-endian. Bicim baslik imzasindan anlasiliyor.
    """
    with open(path, "rb") as handle:
        raw = handle.read()

    is_hgl = raw[:len(_HGL_MAGIC)] == _HGL_MAGIC
    body = raw[_HGL_HEADER:] if is_hgl else raw

    size = math.isqrt(len(body) // 2)
    if size < 2 or size * size * 2 != len(body):
        raise ValueError(f"kare bir izgara degil: {len(body)} bayt, {path}")

    heights = array.array("h", body)
    if (not is_hgl) != (sys.byteorder == "big"):
        heights.byteswap()
    return heights, size


def _fill_voids(heights: array.array, cols: int, rows: int) -> None:
    """Void postlari gecerli komsulariyla doldurur; listeyi yerinde degistirir.

    Ham birakilan -32768 araziyi 32 km asagi indirir ve planlayici oradan
    gecmeye calisir. Doldurma sirali: bir geciste komsusu duzelen post
    sonraki postlara kaynak oluyor, bu yuzden birkac gecis yetiyor.
    """
    voids = [i for i, value in enumerate(heights) if value <= VOID_BELOW]
    if not voids:
        return

    valid = [value for value in heights if value > VOID_BELOW]
    fallback = int(round(sum(valid) / len(valid))) if valid else 0

    for _ in range(4):
        remaining = []
        for index in voids:
            row, col = divmod(index, cols)
            neighbours = []
            for row_step, col_step in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                near_row, near_col = row + row_step, col + col_step
                if 0 <= near_row < rows and 0 <= near_col < cols:
                    value = heights[near_row * cols + near_col]
                    if value > VOID_BELOW:
                        neighbours.append(value)
            if neighbours:
                heights[index] = int(round(sum(neighbours) / len(neighbours)))
            else:
                remaining.append(index)
        voids = remaining
        if not voids:
            return

    for index in voids:                  # cevresi tamamen void kalan ada
        heights[index] = fallback


def load_terrain(path: str, lat: float, lon: float,
                 width_m: float, height_m: float) -> Terrain:
    """Bir SRTM/HGL karosundan yerel metre penceresi keser.

    (lat, lon) pencerenin guneybati kosesi; width_m dogu-bati, height_m
    guney-kuzey. Postlar derecede esit arali oldugu icin metre araligi
    enlemle degisiyor: dogu-bati aralik cos(enlem) ile daraliyor. Donusum
    es dikdortgensel; 30 km'lik bir pencerede metre alti hata veriyor.
    """
    heights, size = _read_tile(path)
    tile_lat, tile_lon = _tile_origin(path)

    step_deg = 1.0 / (size - 1)
    spacing_y = step_deg * METRES_PER_DEGREE
    mid_lat = lat + height_m / 2 / METRES_PER_DEGREE
    spacing_x = spacing_y * math.cos(math.radians(mid_lat))

    cols = int(width_m / spacing_x) + 1
    rows = int(height_m / spacing_y) + 1

    col0 = round((lon - tile_lon) / step_deg)
    row0 = round((lat - tile_lat) / step_deg)     # guneyden sayarak
    if col0 < 0 or row0 < 0 or col0 + cols > size or row0 + rows > size:
        raise ValueError(
            f"pencere karonun disina tasiyor: {path} karosu "
            f"({tile_lat}, {tile_lon}) kosesinden 1 derece; istenen "
            f"({lat}, {lon}) + {width_m} x {height_m} m")

    window = array.array("h", [0]) * (cols * rows)
    for row in range(rows):
        source = size - 1 - (row0 + row)          # dosya kuzeyden sayiyor
        start = source * size + col0
        window[row * cols:(row + 1) * cols] = heights[start:start + cols]

    _fill_voids(window, cols, rows)
    return Terrain(window, cols, rows, spacing_x, spacing_y)



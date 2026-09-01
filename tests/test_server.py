"""Sunucunun saf yardimcilari.

Istek islemenin tamami ag ve arazi istiyor; burada yalnizca yan etkisiz
kurallar var. Post araligi kurali sessizce bozulursa buyuk pencerede
karo butcesi asilir ve 3B kendiliginden kapanir - test onun bekcisi.
"""

from app.server import (MAX_WINDOW, MIN_SPACING, TARGET_POSTS,
                        download_spacing)
from src.terrarium import MAX_TILES, TILE_SIZE, pixel_of, zoom_for

import math


class TestDownloadSpacing:

    def test_small_window_keeps_finest_spacing(self):
        # Kucuk pencerede kural devreye girmemeli: karo cozunurlugunun
        # altina inmek veri yokken varmis gibi gosterirdi.
        assert download_spacing(20000.0, 20000.0) == MIN_SPACING

    def test_spacing_grows_with_the_longer_side(self):
        assert download_spacing(100000.0, 5000.0) == 100000.0 / TARGET_POSTS
        assert download_spacing(5000.0, 100000.0) == 100000.0 / TARGET_POSTS

    def test_post_count_stays_bounded(self):
        for side in (20000.0, 60000.0, MAX_WINDOW):
            posts = side / download_spacing(side, side)
            assert posts <= TARGET_POSTS + 1

    def test_largest_window_fits_the_tile_budget(self):
        # Asil korunan sey bu: en buyuk pencere indirilebilmeli.
        lat, lon = 46.0, 14.0
        spacing = download_spacing(MAX_WINDOW, MAX_WINDOW)
        zoom = zoom_for(spacing, lat)
        half = MAX_WINDOW / 2
        dlat = half / 111320.0
        dlon = half / (111320.0 * math.cos(math.radians(lat)))
        xs, ys = [], []
        for edge_lat in (lat - dlat, lat + dlat):
            for edge_lon in (lon - dlon, lon + dlon):
                px, py = pixel_of(edge_lat, edge_lon, zoom)
                xs.append(math.floor(px / TILE_SIZE))
                ys.append(math.floor(py / TILE_SIZE))
        tiles = (max(xs) - min(xs) + 1) * (max(ys) - min(ys) + 1)
        assert tiles <= MAX_TILES

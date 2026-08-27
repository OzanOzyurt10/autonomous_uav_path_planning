"""Enlem/boylam ile yerel metre koordinatlari arasinda donusum.

Harita enlem/boylam veriyor, planlayici yerel metre istiyor. Olcek pencere
icinde sabit tutuluyor: her noktada kendi enleminde hesaplamak duzlemi
egriltir, ileri-geri donusum ayni yere donmez ve haritada cizilen duz cizgi
planlayicinin gordugu duz cizgi olmaz.
"""

import math
from dataclasses import dataclass

METRES_PER_DEGREE = 111320.0
MAX_LATITUDE = 85.0


@dataclass(frozen=True)
class Frame:
    """Pencerenin guney-bati kosesine oturan duzlem cerceve.

    lat/lon kosenin kendisi, mid_lat olcegin donduruldugu enlem. Kutupta
    bir boylam derecesi sifira gittigi icin MAX_LATITUDE ustu reddediliyor;
    orada to_geo sessiz sacma sonuc yerine acik hata versin.
    """

    lat: float
    lon: float
    mid_lat: float

    def __post_init__(self):
        if not -90.0 <= self.lat <= 90.0:
            raise ValueError(f"enlem [-90, 90] araliginda olmali: {self.lat}")
        if not -180.0 <= self.lon <= 180.0:
            raise ValueError(
                f"boylam [-180, 180] araliginda olmali: {self.lon}")
        if abs(self.mid_lat) >= MAX_LATITUDE:
            raise ValueError(
                f"olcek enlemi kutba fazla yakin: |{self.mid_lat}| >= "
                f"{MAX_LATITUDE}; bir boylam derecesi orada sifira gidiyor")

    @classmethod
    def for_window(cls, lat: float, lon: float, height_m: float) -> "Frame":
        """Guney-bati kosesi ve yuksekligiyle bir cerceve kurar.

        Olcek pencerenin ortasinda donduruluyor - terrain.window ile ayni
        sozlesme, arazi izgarasi ile waypointler kaymasin diye.
        """
        return cls(lat, lon, lat + height_m / 2 / METRES_PER_DEGREE)

    @property
    def lon_scale(self) -> float:
        """Bir boylam derecesi kac metre; kutuplara dogru daraliyor."""
        return METRES_PER_DEGREE * math.cos(math.radians(self.mid_lat))

    def to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """Enlem/boylami (x dogu, y kuzey) metreye cevirir; kose orijin."""
        return ((lon - self.lon) * self.lon_scale,
                (lat - self.lat) * METRES_PER_DEGREE)

    def to_geo(self, x: float, y: float) -> tuple[float, float]:
        """to_local'in tersi. Olcek cerceveden okunuyor, noktadan degil."""
        return (self.lat + y / METRES_PER_DEGREE,
                self.lon + x / self.lon_scale)

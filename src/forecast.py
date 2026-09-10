"""Open-Meteo'dan seviyeli ruzgar profili - gercek atmosfer, formul degil.

ShearWind bir guc yasasi uyduruyor; burada olculmus seviyeler geliyor.
Fark kucuk degil: Riva'da tahmin 805 m'de tepe yapip dusuyor ve yon 1.5
km'de 33 derece doniyor, guc yasasi ikisini de kaciramiyor.

Iki ayri yukseklik ekseni birlesiyor. 10 m ve 100 m degerleri YERDEN,
basinc seviyeleri DENIZDEN olculuyor; hepsi MSL'e tasiniyor, yoksa
siralama profili bozar. Yuksek arazide alt basinc seviyeleri yerin altinda
kaliyor (Alp'te zemin 506 m iken 1000 hPa 155 m'de) ve eleniyor.

Ag erisimi tek bir isleve toplandi (fetch); testler ona sahte bir islev
verip agdan tamamen bagimsiz kaliyor.
"""

import json
import urllib.error
import urllib.request
from datetime import datetime, timezone

ENDPOINT = "https://api.open-meteo.com/v1/forecast"
PRESSURE_LEVELS = (1000, 975, 950, 925, 900, 850)
SURFACE_HEIGHTS = (10, 100)          # m AGL; API bunlari yerden veriyor
TIMEOUT = 20.0


class ForecastUnavailable(RuntimeError):
    """Tahmin alinamadi; kullanici ruzgari elle girmeye devam edebilir."""


def _variables() -> list[str]:
    names = []
    for height in SURFACE_HEIGHTS:
        names += [f"wind_speed_{height}m", f"wind_direction_{height}m"]
    for level in PRESSURE_LEVELS:
        names += [f"wind_speed_{level}hPa", f"wind_direction_{level}hPa",
                  # Basinc seviyesinin METRE karsiligi; onsuz seviyeleri
                  # bir eksene dizmek mumkun degil.
                  f"geopotential_height_{level}hPa"]
    return names


def build_url(lat: float, lon: float) -> str:
    """Istek adresi.

    wind_speed_unit=ms sart: varsayilan km/h ve birimi istemezsek hizlar
    3.6 kat buyuk gelir, hicbir sey hata vermez.
    """
    return (f"{ENDPOINT}?latitude={lat:.4f}&longitude={lon:.4f}"
            f"&hourly={','.join(_variables())}"
            f"&wind_speed_unit=ms&forecast_days=1")


def _http_get(url: str) -> bytes:
    request = urllib.request.Request(
        url, headers={"User-Agent": "uav-path-planning/0.1"})
    with urllib.request.urlopen(request, timeout=TIMEOUT) as response:
        return response.read()


def pick_hour(times, now: datetime) -> int:
    """Simdiye en yakin saatin indisi.

    Tahmin saatlik, gorev ise su an planlaniyor. Araligin disina dusen
    zaman ilk ya da son saate kirpiliyor - o da bir cevap, hata vermekten
    iyi.
    """
    best, best_gap = None, None
    for index, stamp in enumerate(times):
        try:
            gap = abs((datetime.fromisoformat(stamp) - now).total_seconds())
        except (TypeError, ValueError):
            continue
        if best_gap is None or gap < best_gap:
            best, best_gap = index, gap
    if best is None:
        raise ForecastUnavailable("tahminde okunabilir saat yok")
    return best


def _value(hourly, name: str, index: int):
    """Bir dizinin index'teki degeri; yoksa ya da bossa None."""
    series = hourly.get(name)
    if not series or index >= len(series):
        return None
    return series[index]


def _levels_from(hourly, index: int, elevation: float):
    """(MSL yukseklik, hiz, GELDIGI yon) uclulerine cevirir."""
    levels = []
    for height in SURFACE_HEIGHTS:
        speed = _value(hourly, f"wind_speed_{height}m", index)
        from_deg = _value(hourly, f"wind_direction_{height}m", index)
        if speed is None or from_deg is None:
            continue
        levels.append((elevation + height, float(speed), float(from_deg)))

    for level in PRESSURE_LEVELS:
        speed = _value(hourly, f"wind_speed_{level}hPa", index)
        from_deg = _value(hourly, f"wind_direction_{level}hPa", index)
        height = _value(hourly, f"geopotential_height_{level}hPa", index)
        if speed is None or from_deg is None or height is None:
            continue
        # Yuksek arazide alt seviyeler yerin altinda kaliyor; birakilirsa
        # profilin dibine ucagin hic gormeyecegi bir deger oturur.
        if float(height) <= elevation:
            continue
        levels.append((float(height), float(speed), float(from_deg)))
    return levels


def wind_profile(lat: float, lon: float, fetch=_http_get, now=None) -> dict:
    """Verilen noktada seviyeli ruzgar profili ve yuzey okumasi.

    levels ProfileWind'e dogrudan giriyor; speed/from_deg arayuzun ruzgar
    kutularini doldurmasi icin en alt seviyeden. Her hata
    ForecastUnavailable oluyor - cagiran plani cope atmadan elle girmeye
    donebilsin.
    """
    if now is None:
        now = datetime.now(timezone.utc).replace(tzinfo=None)

    try:
        data = json.loads(fetch(build_url(lat, lon)))
    except (urllib.error.URLError, OSError, ValueError) as error:
        raise ForecastUnavailable(f"ruzgar tahmini alinamadi: {error}")

    try:
        hourly = data["hourly"]
        elevation = float(data["elevation"])
        index = pick_hour(hourly["time"], now)
        levels = _levels_from(hourly, index, elevation)
        stamp = hourly["time"][index]
    except (AttributeError, KeyError, TypeError, ValueError) as error:
        raise ForecastUnavailable(f"ruzgar tahmini okunamadi: {error}")

    if not levels:
        raise ForecastUnavailable(
            "tahminde kullanilabilir seviye yok (hepsi yerin altinda "
            "ya da bos)")

    surface = min(levels, key=lambda level: level[0])
    return {"levels": levels, "speed": surface[1], "from_deg": surface[2],
            "time": stamp, "elevation": elevation}

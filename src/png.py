"""Kucuk PNG cozucu: yalnizca yukseklik karolari icin gereken kadari.

Terrarium yukseklik karolari PNG geliyor ve kot RGB'ye kodlu. Pillow venv'de
var ama src/ bilerek bagimliliksiz - terrain.py .hgt ve .hgl'i de elle
okuyor, ayni cizgi. Standart kutuphaneden yalnizca zlib kullaniliyor.

Desteklenen: 8 bit derinlik, kanal duzeni gri / gri+alfa / RGB / RGBA,
aralikli (interlaced) OLMAYAN dosyalar. Terrarium bunlarin icinde.
"""

import struct
import zlib

SIGNATURE = b"\x89PNG\r\n\x1a\n"

# Renk turu -> kanal sayisi. Palet (3) desteklenmiyor; PLTE okumak gerekir
# ve yukseklik karolarinda gecmiyor.
_CHANNELS = {0: 1, 2: 3, 4: 2, 6: 4}


class PngError(ValueError):
    """Bozuk ya da desteklenmeyen PNG."""


def _chunks(data: bytes):
    """Imzadan sonraki parcalari (tur, govde) olarak sirayla verir."""
    if not data.startswith(SIGNATURE):
        raise PngError("PNG imzasi yok")

    offset = len(SIGNATURE)
    while offset + 8 <= len(data):
        length, kind = struct.unpack(">I4s", data[offset:offset + 8])
        body = data[offset + 8:offset + 8 + length]
        if len(body) < length:
            raise PngError(f"{kind.decode('ascii', 'replace')} parcasi eksik")
        yield kind, body
        offset += 12 + length          # uzunluk + tur + govde + CRC


def _paeth(a: int, b: int, c: int) -> int:
    """PNG'nin Paeth ongorucusu: sol, ust ve ust-sol arasindan sec."""
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _unfilter(raw: bytes, width: int, height: int, channels: int) -> bytearray:
    """Satir basindaki filtre baytlarini uygulayip ham pikselleri doner.

    Her satir kendinden onceki satira ve solundaki piksele dayaniyor; bu
    yuzden sirayla ve yerinde cozulmek zorunda.
    """
    stride = width * channels
    expected = (stride + 1) * height
    if len(raw) < expected:
        raise PngError(f"veri eksik: {len(raw)} bayt, beklenen {expected}")

    out = bytearray(stride * height)
    previous = bytearray(stride)
    position = 0

    for row in range(height):
        method = raw[position]
        position += 1
        line = bytearray(raw[position:position + stride])
        position += stride

        if method == 1:                                   # Sub
            for i in range(channels, stride):
                line[i] = (line[i] + line[i - channels]) & 0xFF
        elif method == 2:                                 # Up
            for i in range(stride):
                line[i] = (line[i] + previous[i]) & 0xFF
        elif method == 3:                                 # Average
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                line[i] = (line[i] + ((left + previous[i]) >> 1)) & 0xFF
        elif method == 4:                                 # Paeth
            for i in range(stride):
                left = line[i - channels] if i >= channels else 0
                upper_left = previous[i - channels] if i >= channels else 0
                line[i] = (line[i]
                           + _paeth(left, previous[i], upper_left)) & 0xFF
        elif method != 0:
            raise PngError(f"bilinmeyen satir filtresi: {method}")

        out[row * stride:(row + 1) * stride] = line
        previous = line

    return out


def decode(data: bytes) -> tuple[int, int, int, bytearray]:
    """PNG baytlarini (genislik, yukseklik, kanal, piksel) olarak coz.

    Pikseller satir satir, kanal kanal duz bir bytearray; (x, y) noktasinin
    ilk kanali index (y * genislik + x) * kanal konumunda.
    """
    header = None
    pieces = []

    for kind, body in _chunks(data):
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", body[:13])
        elif kind == b"IDAT":
            # IDAT birden fazla parcaya bolunmus olabilir; sikistirma akisi
            # parcalarin BIRLESIMI uzerinde tanimli, tek tek acilamaz.
            pieces.append(body)
        elif kind == b"IEND":
            break

    if header is None:
        raise PngError("IHDR parcasi yok")
    width, height, depth, colour, compression, filtering, interlace = header

    if depth != 8:
        raise PngError(f"yalnizca 8 bit derinlik destekleniyor: {depth}")
    if colour not in _CHANNELS:
        raise PngError(f"desteklenmeyen renk turu: {colour}")
    if compression != 0 or filtering != 0:
        raise PngError("standart disi sikistirma ya da filtreleme")
    if interlace != 0:
        raise PngError("aralikli (interlaced) PNG desteklenmiyor")
    if not pieces:
        raise PngError("IDAT parcasi yok")

    raw = zlib.decompress(b"".join(pieces))
    channels = _CHANNELS[colour]
    return width, height, channels, _unfilter(raw, width, height, channels)

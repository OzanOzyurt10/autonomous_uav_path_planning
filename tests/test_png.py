"""src/png.py icin testler.

Iki bagimsiz dogrulama: (1) her satir filtresi elle kurulup geri cozuluyor,
(2) Pillow'un urettigi PNG'ler bizim cozucuyle karsilastiriliyor. Pillow
yalnizca TESTTE kullaniliyor; src/ bagimliliksiz kaliyor.
"""

import io
import random
import struct
import zlib

import pytest

from src.png import PngError, decode

PIL = pytest.importorskip("PIL.Image", reason="Pillow yok")

# Kanal sayisi -> PNG renk turu
COLOUR = {1: 0, 2: 4, 3: 2, 4: 6}
MODE = {1: "L", 2: "LA", 3: "RGB", 4: "RGBA"}


def _paeth(a, b, c):
    p = a + b - c
    pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    return b if pb <= pc else c


def _filter_row(line, previous, channels, method):
    """Bir satiri ileri yonde filtreler; cozucunun tersi."""
    out = bytearray(len(line))
    for i in range(len(line)):
        left = line[i - channels] if i >= channels else 0
        upper_left = previous[i - channels] if i >= channels else 0
        if method == 0:
            out[i] = line[i]
        elif method == 1:
            out[i] = (line[i] - left) & 0xFF
        elif method == 2:
            out[i] = (line[i] - previous[i]) & 0xFF
        elif method == 3:
            out[i] = (line[i] - ((left + previous[i]) >> 1)) & 0xFF
        else:
            out[i] = (line[i] - _paeth(left, previous[i], upper_left)) & 0xFF
    return out


def _chunk(kind, body):
    return (struct.pack(">I", len(body)) + kind + body
            + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))


def build_png(width, height, channels, pixels, method, idat_pieces=1):
    """Verilen filtreyle elle bir PNG kurar; cozucuyu tersinden sinar."""
    stride = width * channels
    raw = bytearray()
    previous = bytearray(stride)
    for row in range(height):
        line = bytearray(pixels[row * stride:(row + 1) * stride])
        raw.append(method)
        raw += _filter_row(line, previous, channels, method)
        previous = line

    body = zlib.compress(bytes(raw))
    header = struct.pack(">IIBBBBB", width, height, 8, COLOUR[channels],
                         0, 0, 0)
    out = bytearray(b"\x89PNG\r\n\x1a\n") + _chunk(b"IHDR", header)
    step = max(1, len(body) // idat_pieces + 1)
    for start in range(0, len(body), step):
        out += _chunk(b"IDAT", body[start:start + step])
    return bytes(out + _chunk(b"IEND", b""))


def sample_pixels(width, height, channels, seed=0):
    rng = random.Random(seed)
    return bytes(rng.randrange(256) for _ in range(width * height * channels))


class TestFilters:
    """Bes satir filtresinin hepsi ayri ayri; biri bozuksa gorseldeki
    hata sessiz kalir - sadece kotlar yanlis cikar."""

    @pytest.mark.parametrize("method", [0, 1, 2, 3, 4])
    @pytest.mark.parametrize("channels", [1, 2, 3, 4])
    def test_round_trip(self, method, channels):
        pixels = sample_pixels(9, 7, channels, seed=method * 10 + channels)
        data = build_png(9, 7, channels, pixels, method)
        width, height, got_channels, out = decode(data)
        assert (width, height, got_channels) == (9, 7, channels)
        assert bytes(out) == pixels

    @pytest.mark.parametrize("method", [0, 1, 2, 3, 4])
    def test_single_pixel(self, method):
        # Genislik kanal sayisindan kucukse "soldaki piksel" hic yok;
        # filtrelerin sifir dolgusu burada aciga cikiyor.
        pixels = bytes([17, 200, 3])
        data = build_png(1, 1, 3, pixels, method)
        assert bytes(decode(data)[3]) == pixels

    @pytest.mark.parametrize("method", [1, 3, 4])
    def test_single_column(self, method):
        pixels = sample_pixels(1, 12, 3, seed=method)
        data = build_png(1, 12, 3, pixels, method)
        assert bytes(decode(data)[3]) == pixels


class TestChunking:
    def test_multiple_idat_chunks(self):
        # Sikistirma akisi parcalarin BIRLESIMI uzerinde tanimli; her
        # parcayi ayri acmaya calismak bozuk veri verir.
        pixels = sample_pixels(20, 20, 3, seed=5)
        data = build_png(20, 20, 3, pixels, 4, idat_pieces=6)
        assert data.count(b"IDAT") >= 3
        assert bytes(decode(data)[3]) == pixels

    def test_trailing_bytes_after_iend_ignored(self):
        pixels = sample_pixels(4, 4, 3, seed=6)
        data = build_png(4, 4, 3, pixels, 0) + b"cop veri"
        assert bytes(decode(data)[3]) == pixels


class TestAgainstPillow:
    """Bagimsiz bir uygulamayla karsilastirma. Elle kurulan testler bizim
    filtre anlayisimizi kendisiyle sinar; bu onu disaridan dogruluyor."""

    @pytest.mark.parametrize("channels", [1, 2, 3, 4])
    @pytest.mark.parametrize("size", [(1, 1), (16, 9), (37, 23)])
    def test_matches_pillow(self, channels, size):
        width, height = size
        pixels = sample_pixels(width, height, channels, seed=width * height)
        image = PIL.frombytes(MODE[channels], size, pixels)
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")

        got_w, got_h, got_c, out = decode(buffer.getvalue())
        assert (got_w, got_h, got_c) == (width, height, channels)
        assert bytes(out) == pixels

    def test_matches_pillow_on_a_gradient(self):
        # Rastgele goruntude Pillow cogunlukla tek bir filtre seciyor;
        # yumusak gecis farkli satirlarda farkli filtre sectiriyor.
        width, height = 64, 48
        pixels = bytes(((x * 3 + y * 5) % 256)
                       for y in range(height) for x in range(width)
                       for _ in range(3))
        image = PIL.frombytes("RGB", (width, height), pixels)
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        assert bytes(decode(buffer.getvalue())[3]) == pixels


class TestTerrariumDecoding:
    """Terrarium kodlamasi: kot = R * 256 + G + B / 256 - 32768."""

    def test_known_elevations_survive_the_round_trip(self):
        for metres in (0.0, 744.0, -430.0, 4274.0, 8848.0):
            value = int(round((metres + 32768) * 256))
            red, rest = divmod(value, 65536)
            green, blue = divmod(rest, 256)
            pixels = bytes([red, green, blue])
            data = build_png(1, 1, 3, pixels, 0)
            out = decode(data)[3]
            back = out[0] * 256 + out[1] + out[2] / 256 - 32768
            assert back == pytest.approx(metres, abs=0.01)


class TestRejections:
    def test_missing_signature(self):
        with pytest.raises(PngError, match="imza"):
            decode(b"bu bir PNG degil")

    def test_missing_ihdr(self):
        data = b"\x89PNG\r\n\x1a\n" + _chunk(b"IEND", b"")
        with pytest.raises(PngError, match="IHDR"):
            decode(data)

    def test_missing_idat(self):
        header = struct.pack(">IIBBBBB", 4, 4, 8, 2, 0, 0, 0)
        data = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
                + _chunk(b"IEND", b""))
        with pytest.raises(PngError, match="IDAT"):
            decode(data)

    def test_sixteen_bit_depth_rejected(self):
        # Sessizce yanlis kot uretmektense reddetmek dogru.
        header = struct.pack(">IIBBBBB", 2, 2, 16, 2, 0, 0, 0)
        data = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
                + _chunk(b"IDAT", zlib.compress(b"\x00" * 26))
                + _chunk(b"IEND", b""))
        with pytest.raises(PngError, match="8 bit"):
            decode(data)

    def test_palette_rejected(self):
        header = struct.pack(">IIBBBBB", 2, 2, 8, 3, 0, 0, 0)
        data = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
                + _chunk(b"IDAT", zlib.compress(b"\x00" * 6))
                + _chunk(b"IEND", b""))
        with pytest.raises(PngError, match="renk turu"):
            decode(data)

    def test_interlaced_rejected(self):
        header = struct.pack(">IIBBBBB", 2, 2, 8, 2, 0, 0, 1)
        data = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
                + _chunk(b"IDAT", zlib.compress(b"\x00" * 14))
                + _chunk(b"IEND", b""))
        with pytest.raises(PngError, match="[Aa]ralikli"):
            decode(data)

    def test_truncated_pixel_data(self):
        header = struct.pack(">IIBBBBB", 8, 8, 8, 2, 0, 0, 0)
        data = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
                + _chunk(b"IDAT", zlib.compress(b"\x00" * 10))
                + _chunk(b"IEND", b""))
        with pytest.raises(PngError, match="eksik"):
            decode(data)

    def test_unknown_row_filter(self):
        header = struct.pack(">IIBBBBB", 2, 1, 8, 2, 0, 0, 0)
        body = zlib.compress(bytes([9]) + b"\x00" * 6)   # 9 diye filtre yok
        data = (b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header)
                + _chunk(b"IDAT", body) + _chunk(b"IEND", b""))
        with pytest.raises(PngError, match="filtre"):
            decode(data)

# Dubins Path Modülü — Uygulama Planı

> **Çalışma şekli:** Kodu **Ozan** yazar. Claude her adımda imzayı, formülü ve
> beklenen davranışı verir; testleri yazar ve çalıştırır; geri bildirim verir.
> Bu yüzden adımlarda implementasyon gövdesi bilerek yoktur — test kodu ise
> eksiksizdir. Adım başlıklarındaki **— OZAN** / **— CLAUDE** etiketi o adımı
> kimin yaptığını söyler.

**Hedef:** Fixed-wing İHA için iki poz arasında, minimum dönüş yarıçapı kısıtı
altında en kısa yolu analitik olarak üreten, tek başına test edilebilir modül.

**Mimari:** Üç katman — (1) `(d, alpha, beta)` kanonik problemini çözen altı saf
fonksiyon, (2) çözümü taşıyan ve dünya çerçevesinde örnekleyen `DubinsPath`
dataclass'ı, (3) `shortest_path` / `all_paths` / `path_length` genel API'si.

**Teknoloji:** Python 3.13, çekirdekte sadece `math` + `dataclasses`.
Test: pytest 9.1.1. Çizim: matplotlib 3.11.1.

## Global Kısıtlar

- `src/dubins.py` içinde **numpy kullanılmaz**. Sadece `math`, `dataclasses`, `typing`.
- Açı birimi radyan, CCW pozitif, ENU çerçevesi (x doğu, y kuzey).
- Tüm açı normalizasyonu tek bir `_mod2pi` fonksiyonundan geçer; `[0, 2*pi)` döndürür.
- `rho <= 0` her genel API fonksiyonunda `ValueError` fırlatır.
- Test assert toleransı: `1e-6` (geometri), `1e-9` (cebirsel özdeşlik).
- Python komutu: `./venv/Scripts/python.exe`
- Her task sonunda commit atılır.

---

### Task 1: Test altyapısı + `_mod2pi`

**Dosyalar:**
- Oluştur: `conftest.py` (proje kökü, boş)
- Yaz: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Üretir: `_mod2pi(theta: float) -> float`

Kök dizindeki boş `conftest.py`, pytest'in proje kökünü `sys.path`'e eklemesini
sağlar; `from src.dubins import ...` ancak böyle çalışır.

- [ ] **Adım 1: `conftest.py` oluştur (boş dosya) — CLAUDE**

- [ ] **Adım 2: Başarısız testi yaz — CLAUDE**

```python
import math

import pytest

from src.dubins import _mod2pi

ANGLES = [-10.0, -math.pi, -0.1, 0.0, 0.1, math.pi, 7.0, 100.0]


class TestMod2Pi:
    def test_maps_into_zero_two_pi(self):
        for theta in ANGLES:
            assert 0.0 <= _mod2pi(theta) < 2 * math.pi

    def test_preserves_angle_identity(self):
        for theta in ANGLES:
            wrapped = _mod2pi(theta)
            assert math.isclose(math.sin(wrapped), math.sin(theta), abs_tol=1e-12)
            assert math.isclose(math.cos(wrapped), math.cos(theta), abs_tol=1e-12)

    def test_zero_stays_zero(self):
        assert _mod2pi(0.0) == 0.0

    def test_negative_small_wraps_near_two_pi(self):
        assert math.isclose(_mod2pi(-0.1), 2 * math.pi - 0.1, abs_tol=1e-12)
```

- [ ] **Adım 3: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

Çalıştır: `./venv/Scripts/python.exe -m pytest tests/test_dubins.py -v`
Beklenen: `ImportError: cannot import name '_mod2pi'`

- [ ] **Adım 4: `_mod2pi`'yi yaz — OZAN**

`src/dubins.py` içine modül docstring'i, `import math`, sonra:

```python
def _mod2pi(theta: float) -> float:
    ...  # Ozan yazacak
```

Sözleşme: Herhangi bir radyan açıyı `[0, 2*pi)` aralığına indirger. Negatif
girdide de pozitif sonuç döner. Dikkat: Python'un `%` operatörü bunu doğal
yapar, `math.fmod` yapmaz — `math.fmod(-0.1, 2*pi)` negatif döner. Sonuç kayan
nokta hassasiyetinde `2*pi`'ye eşit çıkmamalı.

- [ ] **Adım 5: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 6: Commit — CLAUDE**

```bash
git add conftest.py src/dubins.py tests/test_dubins.py requirements.txt
git commit -m "Dubins: aci normalizasyonu ve test altyapisi"
```

---

### Task 2: `Pose` tipi ve `DubinsPath` dataclass'ı

**Dosyalar:**
- Değiştir: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Tüketir: `_mod2pi`
- Üretir:
  - `Pose = tuple[float, float, float]`
  - `DubinsPath` — frozen dataclass, alanlar sırayla:
    `start: Pose`, `word: str`, `lengths: tuple[float, float, float]`, `rho: float`
  - `DubinsPath.length -> float` (property)

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
from src.dubins import DubinsPath


class TestDubinsPathBasics:
    def test_length_is_sum_of_segments(self):
        path = DubinsPath(start=(0.0, 0.0, 0.0), word="LSL",
                          lengths=(1.0, 2.0, 3.0), rho=5.0)
        assert math.isclose(path.length, 6.0)

    def test_is_frozen(self):
        path = DubinsPath(start=(0.0, 0.0, 0.0), word="LSL",
                          lengths=(1.0, 2.0, 3.0), rho=5.0)
        with pytest.raises(Exception):
            path.rho = 2.0

    def test_zero_length_path(self):
        path = DubinsPath(start=(1.0, 2.0, 0.5), word="LSL",
                          lengths=(0.0, 0.0, 0.0), rho=5.0)
        assert path.length == 0.0
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `Pose` ve `DubinsPath`'i yaz — OZAN**

Sözleşme:
- `Pose` bir tip takma adı: `(x, y, yaw)` üçlüsü.
- `DubinsPath` `@dataclass(frozen=True)`.
- `lengths` **dünya birimindeki** yay uzunlukları (radyan değil, metre) —
  `word`'ün üç harfine sırayla karşılık gelir.
- `length` bir `@property`, üç segmentin toplamı.
- Sınıf docstring'i: her alanın ne olduğunu ve `lengths`'in birimini açıkla.
  Bu birim ayrımı modülün en kolay karıştırılan yeri — yazıya dök.

- [ ] **Adım 4: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/dubins.py tests/test_dubins.py
git commit -m "Dubins: Pose tipi ve DubinsPath veri yapisi"
```

---

### Task 3: `_segment_end`, `interpolate`, `end_pose`, `sample`

Modülün geometri kalbi. Kanonik çözücülerden **bağımsız** test edilebilir: elle
bir `DubinsPath` kurup çıkan eğriyi kontrol ederiz.

**Dosyalar:**
- Değiştir: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Üretir:
  - `_segment_end(pose: Pose, mode: str, s: float, rho: float) -> Pose`
  - `DubinsPath.interpolate(self, s: float) -> Pose`
  - `DubinsPath.end_pose(self) -> Pose`
  - `DubinsPath.sample(self, step: float) -> list[Pose]`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
from src.dubins import _segment_end

HALF_PI = math.pi / 2


def assert_pose_close(actual, expected, tol=1e-6):
    assert math.isclose(actual[0], expected[0], abs_tol=tol), f"x: {actual} vs {expected}"
    assert math.isclose(actual[1], expected[1], abs_tol=tol), f"y: {actual} vs {expected}"
    dyaw = _mod2pi(actual[2] - expected[2])
    dyaw = min(dyaw, 2 * math.pi - dyaw)
    assert dyaw < tol, f"yaw: {actual} vs {expected}"


class TestSegmentGeometry:
    def test_straight_moves_along_heading(self):
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "S", 3.0, 1.0), (3.0, 0.0, 0.0))
        assert_pose_close(_segment_end((0.0, 0.0, HALF_PI), "S", 3.0, 1.0), (0.0, 3.0, HALF_PI))

    def test_left_quarter_turn(self):
        # rho=1, ceyrek tur sola (yay uzunlugu pi/2): (0,0,0) -> (1,1,pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "L", HALF_PI, 1.0), (1.0, 1.0, HALF_PI))

    def test_right_quarter_turn(self):
        # rho=1, ceyrek tur saga: (0,0,0) -> (1,-1,-pi/2)
        assert_pose_close(_segment_end((0.0, 0.0, 0.0), "R", HALF_PI, 1.0), (1.0, -1.0, -HALF_PI))

    def test_full_left_circle_returns_to_start(self):
        rho, start = 2.5, (3.0, -1.0, 0.7)
        assert_pose_close(_segment_end(start, "L", 2 * math.pi * rho, rho), start)

    def test_full_right_circle_returns_to_start(self):
        rho, start = 2.5, (3.0, -1.0, 0.7)
        assert_pose_close(_segment_end(start, "R", 2 * math.pi * rho, rho), start)

    def test_zero_length_is_identity(self):
        start = (3.0, -1.0, 0.7)
        for mode in "LSR":
            assert_pose_close(_segment_end(start, mode, 0.0, 2.0), start)

    def test_unknown_mode_raises(self):
        with pytest.raises(ValueError):
            _segment_end((0.0, 0.0, 0.0), "X", 1.0, 1.0)


class TestInterpolateAndSample:
    def _path(self):
        # rho=1: sola ceyrek tur, 2 birim duz, saga ceyrek tur
        return DubinsPath(start=(0.0, 0.0, 0.0), word="LSR",
                          lengths=(HALF_PI, 2.0, HALF_PI), rho=1.0)

    def test_interpolate_at_zero_is_start(self):
        p = self._path()
        assert_pose_close(p.interpolate(0.0), p.start)

    def test_interpolate_at_length_is_end_pose(self):
        p = self._path()
        assert_pose_close(p.interpolate(p.length), p.end_pose())

    def test_known_waypoints(self):
        p = self._path()
        assert_pose_close(p.interpolate(HALF_PI), (1.0, 1.0, HALF_PI))
        assert_pose_close(p.interpolate(HALF_PI + 2.0), (1.0, 3.0, HALF_PI))
        assert_pose_close(p.end_pose(), (2.0, 4.0, 0.0))

    def test_interpolate_clamps_out_of_range(self):
        p = self._path()
        assert_pose_close(p.interpolate(-5.0), p.start)
        assert_pose_close(p.interpolate(p.length + 5.0), p.end_pose())

    def test_sample_endpoints(self):
        p = self._path()
        pts = p.sample(0.1)
        assert_pose_close(pts[0], p.start)
        assert_pose_close(pts[-1], p.end_pose())

    def test_sample_spacing_never_exceeds_step(self):
        p = self._path()
        step = 0.1
        pts = p.sample(step)
        for a, b in zip(pts, pts[1:]):
            assert math.hypot(b[0] - a[0], b[1] - a[1]) <= step + 1e-9

    def test_sample_rejects_nonpositive_step(self):
        p = self._path()
        with pytest.raises(ValueError):
            p.sample(0.0)

    def test_sample_of_zero_length_path(self):
        p = DubinsPath(start=(1.0, 2.0, 0.5), word="LSL", lengths=(0.0, 0.0, 0.0), rho=1.0)
        pts = p.sample(0.1)
        assert len(pts) == 1
        assert_pose_close(pts[0], p.start)
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `_segment_end`'i yaz — OZAN**

Sözleşme: `pose`'dan başlayıp `mode` tipinde `s` **dünya birimi** yay uzunluğu
kadar ilerledikten sonraki pozu döndürür. `rho` dönüş yarıçapı. Bilinmeyen
`mode` için `ValueError`.

Formüller (`psi` = mevcut yaw, `psi2` = yeni yaw):

| mode | psi2 | x2 | y2 |
|------|------|----|----|
| `"S"` | `psi` | `x + s*cos(psi)` | `y + s*sin(psi)` |
| `"L"` | `psi + s/rho` | `x + rho*(sin(psi2) - sin(psi))` | `y - rho*(cos(psi2) - cos(psi))` |
| `"R"` | `psi - s/rho` | `x + rho*(sin(psi) - sin(psi2))` | `y + rho*(cos(psi2) - cos(psi))` |

Dönen yaw'ı `_mod2pi`'den **geçirme** — `interpolate` bunu zincirleme çağırıyor,
ham değer kalsın; karşılaştırmalar testte zaten normalize ediliyor.

Nereden geliyor: sol dönüşün merkezi `(x - rho*sin(psi), y + rho*cos(psi))`,
sağ dönüşünki `(x + rho*sin(psi), y - rho*cos(psi))`. Tablodakiler bu merkez
etrafındaki dönmenin sadeleşmiş hali. Kendi türetmeni `s=0`'da sınadığında her
üç satır da birim dönüşüm vermeli — hızlı kontrol yolu bu.

- [ ] **Adım 4: `interpolate`, `end_pose`, `sample`'ı yaz — OZAN**

Sözleşme:
- `interpolate(s)`: `s`'yi `[0, length]` aralığına kırp. `start`'tan başla, üç
  segmenti sırayla dolaş; her segmentte `min(kalan, segment_uzunlugu)` kadarını
  `_segment_end` ile ilerlet, kalanı azalt. Poz döndür.
- `end_pose()`: `interpolate(self.length)`.
- `sample(step)`: `step <= 0` ise `ValueError`. `0`'dan `length`'e `step`
  aralıklarla örnekle; **son eleman her zaman tam olarak `length`** olsun,
  kayan nokta yüzünden atlanmasın. `length == 0` ise tek elemanlı liste.
  İpucu: `n = max(1, ceil(length / step))`, sonra `i` için `0..n` arası
  `interpolate(i / n * length)` — bu hem uçları garantiler hem aralığı `step`'in
  altında tutar.

- [ ] **Adım 5: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 6: Commit — CLAUDE**

```bash
git add src/dubins.py tests/test_dubins.py
git commit -m "Dubins: segment geometrisi, interpolate ve sample"
```

---

### Task 4: Kanonik dönüşüm

**Dosyalar:**
- Değiştir: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Tüketir: `_mod2pi`
- Üretir: `_to_canonical(start: Pose, goal: Pose, rho: float) -> tuple[float, float, float]`
  → `(d, alpha, beta)`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
from src.dubins import _to_canonical


class TestCanonicalTransform:
    def test_aligned_along_x_axis(self):
        d, alpha, beta = _to_canonical((0.0, 0.0, 0.0), (4.0, 0.0, 0.0), 2.0)
        assert math.isclose(d, 2.0)
        assert math.isclose(alpha, 0.0, abs_tol=1e-12)
        assert math.isclose(beta, 0.0, abs_tol=1e-12)

    def test_rotation_invariance(self):
        rot = math.radians(40)

        def rotate(p):
            x, y, yaw = p
            return (x * math.cos(rot) - y * math.sin(rot),
                    x * math.sin(rot) + y * math.cos(rot),
                    yaw + rot)

        base = _to_canonical((0.0, 0.0, 0.3), (5.0, 2.0, 1.1), 1.5)
        turned = _to_canonical(rotate((0.0, 0.0, 0.3)), rotate((5.0, 2.0, 1.1)), 1.5)
        for a, b in zip(base, turned):
            diff = _mod2pi(a - b)
            assert min(diff, 2 * math.pi - diff) < 1e-9

    def test_translation_invariance(self):
        base = _to_canonical((0.0, 0.0, 0.3), (5.0, 2.0, 1.1), 1.5)
        moved = _to_canonical((10.0, -7.0, 0.3), (15.0, -5.0, 1.1), 1.5)
        for a, b in zip(base, moved):
            assert math.isclose(a, b, abs_tol=1e-12)

    def test_d_scales_inversely_with_rho(self):
        d1, _, _ = _to_canonical((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 1.0)
        d2, _, _ = _to_canonical((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), 2.0)
        assert math.isclose(d1, 10.0)
        assert math.isclose(d2, 5.0)

    def test_angles_are_normalized(self):
        _, alpha, beta = _to_canonical((0.0, 0.0, -3.0), (1.0, 1.0, 9.0), 1.0)
        assert 0.0 <= alpha < 2 * math.pi
        assert 0.0 <= beta < 2 * math.pi

    def test_coincident_positions_give_zero_d(self):
        d, _, _ = _to_canonical((2.0, 2.0, 0.0), (2.0, 2.0, 1.0), 1.0)
        assert d == 0.0
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `_to_canonical`'ı yaz — OZAN**

```
dx    = goal_x - start_x
dy    = goal_y - start_y
D     = hypot(dx, dy)
d     = D / rho
theta = _mod2pi(atan2(dy, dx))
alpha = _mod2pi(start_yaw - theta)
beta  = _mod2pi(goal_yaw  - theta)
```

Fikir: başlangıç ile hedefi birleştiren doğruyu x eksenine oturtuyor ve tüm
uzunlukları `rho`'ya bölüyoruz. Problem böylece sadece üç sayıya iniyor; altı
kelimenin formülleri bu üç sayıyla yazılabiliyor. Bu, Shkel & Lumelsky'nin
tüm yaklaşımının dayandığı sadeleştirme.

Not: `D == 0` olduğunda `atan2(0, 0)` Python'da `0.0` döner; özel durum yazmaya
gerek yok. Testler bunu doğruluyor.

- [ ] **Adım 4: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/dubins.py tests/test_dubins.py
git commit -m "Dubins: kanonik donusum"
```

---

### Task 5: CSC kelimeleri (`_lsl`, `_rsr`, `_lsr`, `_rsl`)

**Dosyalar:**
- Değiştir: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Tüketir: `_mod2pi`
- Üretir:
  - `_lsl`, `_rsr`, `_lsr`, `_rsl` — hepsi
    `(d: float, alpha: float, beta: float) -> tuple[float, float, float] | None`
  - `_SOLVERS: dict[str, Callable]` — şimdilik dört kelime

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

Bu testler her kelimeyi **geometrik olarak** doğrular: kelimeyi bir `DubinsPath`'e
çevirip ucunun hedefe oturmasını kontrol eder. Task 3 geçtiği için bu güvenilir
bir ölçü aletidir.

```python
from src.dubins import _lsl, _lsr, _rsl, _rsr, _SOLVERS

CSC_WORDS = ["LSL", "RSR", "LSR", "RSL"]


def build_path(word, start, goal, rho):
    """Kanonik cozucuyu cagirip DubinsPath kurar; kelime gecersizse None."""
    d, alpha, beta = _to_canonical(start, goal, rho)
    result = _SOLVERS[word](d, alpha, beta)
    if result is None:
        return None
    t, p, q = result
    return DubinsPath(start=start, word=word,
                      lengths=(t * rho, p * rho, q * rho), rho=rho)


POSE_PAIRS = [
    ((0.0, 0.0, 0.0), (10.0, 0.0, 0.0)),
    ((0.0, 0.0, 0.0), (4.0, 3.0, 1.2)),
    ((0.0, 0.0, 2.0), (-6.0, 5.0, -1.0)),
    ((1.0, -2.0, 0.5), (1.5, -1.5, 3.0)),
    ((0.0, 0.0, 0.0), (0.5, 0.0, math.pi)),
]


class TestCSCWords:
    @pytest.mark.parametrize("word", CSC_WORDS)
    @pytest.mark.parametrize("start,goal", POSE_PAIRS)
    def test_reaches_goal_when_valid(self, word, start, goal):
        path = build_path(word, start, goal, 2.0)
        if path is None:
            pytest.skip(f"{word} bu poz cifti icin gecersiz")
        assert_pose_close(path.end_pose(), goal, tol=1e-6)

    @pytest.mark.parametrize("word", CSC_WORDS)
    @pytest.mark.parametrize("start,goal", POSE_PAIRS)
    def test_segment_lengths_nonnegative(self, word, start, goal):
        path = build_path(word, start, goal, 2.0)
        if path is None:
            pytest.skip(f"{word} bu poz cifti icin gecersiz")
        assert all(seg >= -1e-12 for seg in path.lengths)

    @pytest.mark.parametrize("word", ["LSL", "RSR"])
    def test_straight_line_is_exactly_distance(self, word):
        path = build_path(word, (0.0, 0.0, 0.0), (7.0, 0.0, 0.0), 1.0)
        assert path is not None
        assert math.isclose(path.length, 7.0, abs_tol=1e-9)

    def test_four_csc_solvers_registered(self):
        assert {"LSL", "RSR", "LSR", "RSL"} <= set(_SOLVERS)
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: Dört CSC çözücüsünü yaz — OZAN**

Ortak sözleşme: `(d, alpha, beta)` alır. Geçerliyse normalize `(t, p, q)`
döndürür — `t` ve `q` **radyan** dönüş açıları, `p` **rho birimi** düz mesafe.
Geçersizse `None`.

Her fonksiyonun başında kısayollar:
`sa = sin(alpha)`, `ca = cos(alpha)`, `sb = sin(beta)`, `cb = cos(beta)`,
`c_ab = cos(alpha - beta)`

**`_lsl`**
```
p_sq = 2 + d*d - 2*c_ab + 2*d*(sa - sb)
p_sq gecersizse     -> None
tmp  = atan2(cb - ca, d + sa - sb)
t    = _mod2pi(-alpha + tmp)
p    = sqrt(p_sq)
q    = _mod2pi(beta - tmp)
```

**`_rsr`**
```
p_sq = 2 + d*d - 2*c_ab + 2*d*(sb - sa)
p_sq gecersizse     -> None
tmp  = atan2(ca - cb, d - sa + sb)
t    = _mod2pi(alpha - tmp)
p    = sqrt(p_sq)
q    = _mod2pi(-_mod2pi(beta) + tmp)
```

**`_lsr`**
```
p_sq = -2 + d*d + 2*c_ab + 2*d*(sa + sb)
p_sq gecersizse     -> None
p    = sqrt(p_sq)
tmp  = atan2(-ca - cb, d + sa + sb) - atan2(-2.0, p)
t    = _mod2pi(-alpha + tmp)
q    = _mod2pi(-_mod2pi(beta) + tmp)
```

**`_rsl`**
```
p_sq = d*d - 2 + 2*c_ab - 2*d*(sa + sb)
p_sq gecersizse     -> None
p    = sqrt(p_sq)
tmp  = atan2(ca + cb, d - sa - sb) - atan2(2.0, p)
t    = _mod2pi(alpha - tmp)
q    = _mod2pi(beta - tmp)
```

"`p_sq` geçersizse" kuralı, kayan nokta yüzünden ayrı yazılmayı hak ediyor:
`p_sq < -1e-12` ise `None`; `-1e-12 <= p_sq < 0` ise `0.0`'a kırp. Tam sıfır
olması gereken bir değer `-4e-16` çıktığında kelimeyi geçersiz saymak gerçek
bir hata kaynağıdır. Bu kırpmayı dört fonksiyonda tekrarlamak yerine küçük bir
`_safe_sqrt(value) -> float | None` yardımcısı yazmak temiz olur — kararı sen ver.

Dördünün ardından sözlüğü tanımla:

```python
_SOLVERS = {"LSL": _lsl, "RSR": _rsr, "LSR": _lsr, "RSL": _rsl}
```

- [ ] **Adım 4: Testi çalıştır — CLAUDE**

Kırmızı yanarsa formülü birlikte inceleriz; hangi kelimede olduğunu test adı
söyleyecek.

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/dubins.py tests/test_dubins.py
git commit -m "Dubins: CSC kelimeleri (LSL, RSR, LSR, RSL)"
```

---

### Task 6: CCC kelimeleri (`_rlr`, `_lrl`)

**Dosyalar:**
- Değiştir: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Üretir: `_rlr`, `_lrl` — imza CSC ile aynı; `_SOLVERS` altı kelimeye çıkar

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
CCC_WORDS = ["RLR", "LRL"]

CCC_POSE_PAIRS = [
    ((0.0, 0.0, 0.0), (1.0, 0.0, math.pi)),
    ((0.0, 0.0, 0.0), (0.0, 0.0, 2.0)),
    ((0.0, 0.0, 0.0), (2.0, 1.0, 3.0)),
    ((0.0, 0.0, 1.0), (-1.0, 0.5, -2.0)),
]


class TestCCCWords:
    @pytest.mark.parametrize("word", CCC_WORDS)
    @pytest.mark.parametrize("start,goal", CCC_POSE_PAIRS)
    def test_reaches_goal_when_valid(self, word, start, goal):
        path = build_path(word, start, goal, 1.0)
        if path is None:
            pytest.skip(f"{word} bu poz cifti icin gecersiz")
        assert_pose_close(path.end_pose(), goal, tol=1e-6)

    @pytest.mark.parametrize("word", CCC_WORDS)
    def test_invalid_when_far_apart(self, word):
        # CCC yalnizca d < 4 icin gecerlidir
        assert _SOLVERS[word](10.0, 0.0, 0.0) is None

    @pytest.mark.parametrize("word", CCC_WORDS)
    def test_middle_arc_is_major(self, word):
        # CCC'de orta yayin donus acisi pi'den buyuk olmalidir
        result = _SOLVERS[word](1.0, 0.0, math.pi)
        if result is None:
            pytest.skip(f"{word} gecersiz")
        _, p, _ = result
        assert p > math.pi - 1e-9

    def test_all_six_solvers_registered(self):
        assert set(_SOLVERS) == {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"}
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: İki CCC çözücüsünü yaz — OZAN**

**`_rlr`**
```
tmp = (6 - d*d + 2*c_ab + 2*d*(sa - sb)) / 8
abs(tmp) > 1        -> None
p = _mod2pi(2*pi - acos(tmp))
t = _mod2pi(alpha - atan2(ca - cb, d - sa + sb) + p/2)
q = _mod2pi(alpha - beta - t + p)
```

**`_lrl`**
```
tmp = (6 - d*d + 2*c_ab + 2*d*(-sa + sb)) / 8
abs(tmp) > 1        -> None
p = _mod2pi(2*pi - acos(tmp))
t = _mod2pi(-alpha - atan2(ca - cb, d + sa - sb) + p/2)
q = _mod2pi(beta - alpha - t + p)
```

CSC'den farkı: burada `acos` var, `sqrt` yok — geçersizlik koşulu da bu yüzden
`abs(tmp) > 1`. `2*pi - acos(...)` ifadesi orta yayın **büyük** yay olmasını
zorlar; CCC'nin optimal olduğu durumda orta segment daima `pi`'den uzundur.
`test_middle_arc_is_major` tam olarak bunu kontrol ediyor.

`_SOLVERS` sözlüğüne `"RLR": _rlr, "LRL": _lrl` ekle.

- [ ] **Adım 4: Testi çalıştır — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/dubins.py tests/test_dubins.py
git commit -m "Dubins: CCC kelimeleri (RLR, LRL)"
```

---

### Task 7: Genel API — `all_paths`, `shortest_path`, `path_length`

**Dosyalar:**
- Değiştir: `src/dubins.py`
- Test: `tests/test_dubins.py`

**Arayüz:**
- Tüketir: `_to_canonical`, `_SOLVERS`, `DubinsPath`
- Üretir:
  - `all_paths(start: Pose, goal: Pose, rho: float) -> list[DubinsPath]`
  - `shortest_path(start: Pose, goal: Pose, rho: float) -> DubinsPath`
  - `path_length(start: Pose, goal: Pose, rho: float) -> float`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

Spec'teki doğrulama paketinin tamamı — modülün asıl sınavı.

```python
import random

from src.dubins import all_paths, path_length, shortest_path


def random_poses(seed, n, span=20.0):
    rng = random.Random(seed)
    for _ in range(n):
        yield ((rng.uniform(-span, span), rng.uniform(-span, span),
                rng.uniform(0, 2 * math.pi)),
               (rng.uniform(-span, span), rng.uniform(-span, span),
                rng.uniform(0, 2 * math.pi)))


class TestPublicAPI:
    @pytest.mark.parametrize("bad_rho", [0.0, -1.0])
    def test_rho_must_be_positive(self, bad_rho):
        for fn in (all_paths, shortest_path, path_length):
            with pytest.raises(ValueError):
                fn((0.0, 0.0, 0.0), (1.0, 1.0, 0.0), bad_rho)

    def test_all_paths_returns_distinct_valid_words(self):
        paths = all_paths((0.0, 0.0, 0.0), (5.0, 3.0, 1.0), 2.0)
        assert 1 <= len(paths) <= 6
        assert all(p.word in _SOLVERS for p in paths)
        assert len({p.word for p in paths}) == len(paths)

    def test_shortest_is_minimum_of_all(self):
        for start, goal in random_poses(seed=1, n=50):
            best = shortest_path(start, goal, 2.0)
            assert math.isclose(best.length,
                                min(p.length for p in all_paths(start, goal, 2.0)),
                                rel_tol=1e-12)

    def test_path_length_matches_shortest_path(self):
        for start, goal in random_poses(seed=2, n=50):
            assert math.isclose(path_length(start, goal, 2.0),
                                shortest_path(start, goal, 2.0).length,
                                rel_tol=1e-12)

    def test_endpoint_reconstruction(self):
        """Kritik test: cozulen yolun ucu hedefe oturmali."""
        for rho in (0.5, 1.0, 3.7):
            for start, goal in random_poses(seed=3, n=60):
                path = shortest_path(start, goal, rho)
                assert_pose_close(path.end_pose(), goal, tol=1e-6)

    def test_all_valid_words_reach_goal(self):
        for start, goal in random_poses(seed=4, n=30):
            for path in all_paths(start, goal, 1.5):
                assert_pose_close(path.end_pose(), goal, tol=1e-6)

    def test_length_at_least_euclidean_distance(self):
        for start, goal in random_poses(seed=5, n=60):
            dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
            assert path_length(start, goal, 2.0) >= dist - 1e-9

    def test_straight_line_case_is_exact(self):
        path = shortest_path((0.0, 0.0, 0.0), (12.0, 0.0, 0.0), 1.0)
        assert "S" in path.word
        assert math.isclose(path.length, 12.0, abs_tol=1e-9)

    def test_identical_poses_give_zero_length(self):
        pose = (3.0, -4.0, 1.2)
        assert math.isclose(path_length(pose, pose, 2.0), 0.0, abs_tol=1e-9)

    def test_scale_invariance(self):
        k = 3.0
        for start, goal in random_poses(seed=6, n=30):
            scaled_start = (start[0] * k, start[1] * k, start[2])
            scaled_goal = (goal[0] * k, goal[1] * k, goal[2])
            assert math.isclose(path_length(scaled_start, scaled_goal, 2.0 * k),
                                k * path_length(start, goal, 2.0),
                                rel_tol=1e-9)

    def test_curvature_never_exceeds_limit(self):
        rho, step = 2.0, 0.05
        for start, goal in random_poses(seed=7, n=20):
            pts = shortest_path(start, goal, rho).sample(step)
            for a, b in zip(pts, pts[1:]):
                ds = math.hypot(b[0] - a[0], b[1] - a[1])
                if ds < 1e-12:
                    continue
                dyaw = _mod2pi(b[2] - a[2])
                dyaw = min(dyaw, 2 * math.pi - dyaw)
                # yay uzunlugu kiristen buyuk oldugu icin bu ust sinir muhafazakar
                assert dyaw / ds <= 1.0 / rho + 1e-3

    def test_sampled_path_starts_and_ends_correctly(self):
        for start, goal in random_poses(seed=8, n=20):
            pts = shortest_path(start, goal, 1.0).sample(0.1)
            assert_pose_close(pts[0], start)
            assert_pose_close(pts[-1], goal)
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: Üç genel fonksiyonu yaz — OZAN**

Sözleşme:

- `all_paths(start, goal, rho)`: `rho <= 0` ise `ValueError`. `_to_canonical`
  ile `(d, alpha, beta)` bul, `_SOLVERS`'daki altı çözücüyü çağır, `None`
  dönmeyenler için `DubinsPath` kur. **Segment uzunluklarını `rho` ile çarpmayı
  unutma** — çözücüler normalize değer döndürüyor, `DubinsPath.lengths` ise
  dünya birimi. Liste döndür.
- `shortest_path(start, goal, rho)`: `all_paths`'in `length`'i en küçük olanı.
  Liste boşsa `RuntimeError` — `rho > 0` için olmamalı, ama sessizce `None`
  döndürmek hata ayıklamayı imkânsız kılar.
- `path_length(start, goal, rho)`: `shortest_path(...).length`.

Üçünün de docstring'i olsun: parametreler, dönüş değeri, fırlatılan istisnalar.
Bunlar modülün dışa bakan yüzü; `rrt_star.py` bu docstring'leri okuyarak
kullanılacak.

- [ ] **Adım 4: Testi çalıştır — CLAUDE**

`test_endpoint_reconstruction` altı formülü birden sınar; 180 poz çifti üzerinden
geçer. Kırmızı yanarsa hangi kelimede olduğunu birlikte daraltırız.

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/dubins.py tests/test_dubins.py
git commit -m "Dubins: genel API ve dogrulama testleri"
```

---

### Task 8: Görselleştirme demosu

**Dosyalar:**
- Oluştur: `notebooks/dubins_demo.py`

**Arayüz:**
- Tüketir: `all_paths`, `shortest_path`, `DubinsPath.sample`

- [ ] **Adım 1: Demoyu yaz — OZAN**

Sözleşme: Bir başlangıç ve hedef poz seç (örn. `(0, 0, 0)` → `(10, 6, pi/2)`,
`rho = 3.0`). `all_paths` ile adayları al, matplotlib ile 2x3 alt grafikte çiz.
Her alt grafikte:

- yolu `sample(0.05)` ile örnekleyip çiz
- başlangıç ve hedef pozu ok (`ax.quiver` veya `ax.arrow`) ile göster
- başlığa kelimeyi ve uzunluğu yaz: `f"{path.word} — {path.length:.2f} m"`
- en kısa olanın başlığını farklı renkle vurgula
- `ax.set_aspect("equal")` — bu olmadan daireler elips görünür ve şekil yanıltır
- geçersiz kelimeler için ekseni boş bırakıp "gecersiz" yaz

`results/dubins_words.png` olarak kaydet (`dpi=150`). En altta
`if __name__ == "__main__":` bloğu olsun.

- [ ] **Adım 2: Çalıştır ve şekli incele — CLAUDE**

Çalıştır: `./venv/Scripts/python.exe -m notebooks.dubins_demo`
(`-m` sart: dosya yoluyla calistirinca proje koku sys.path'e girmez)
Beklenen: `results/dubins_words.png` oluşur; geçerli kelimelerin hepsi
başlangıçtan hedefe gider, dönüş yayları eşit yarıçaplı görünür.

- [ ] **Adım 3: Commit — CLAUDE**

```bash
git add notebooks/dubins_demo.py results/dubins_words.png
git commit -m "Dubins: alti kelimeyi karsilastiran gorsel demo"
```

---

## Bu planın dışında, sonraki turlar

- `src/environment.py` — engel gösterimi ve çarpışma kontrolü
- `src/rrt_star.py` — Dubins'i `path_length` (maliyet) ve `sample` (çarpışma
  kontrolü) üzerinden tüketir

# Environment Modülü — Uygulama Planı

> **Çalışma şekli:** Kodu **Ozan** yazar. Claude her adımda imzayı, formülü ve
> beklenen davranışı verir; testleri yazar ve çalıştırır; geri bildirim verir.
> Adımlarda implementasyon gövdesi bilerek yoktur — test kodu ise eksiksizdir.
> Adım başlıklarındaki **— OZAN** / **— CLAUDE** etiketi o adımı kimin
> yaptığını söyler.

**Hedef:** Dairesel engeller, harita sınırları ve emniyet payından oluşan
planlama ortamı; "bu nokta serbest mi", "bu yol serbest mi" sorularına cevap
verir ve RRT* için serbest uzaydan rastgele poz üretir.

**Mimari:** İki frozen dataclass — `Obstacle` (merkez + yarıçap) ve
`Environment` (sınırlar + engeller + emniyet payı). Environment `dubins.py`'yi
import etmez; yol kontrolü örneklenmiş nokta listesi alır, `DubinsPath` değil.

**Teknoloji:** Python 3.12, sadece `math` + `dataclasses` + `random`.
Test: pytest 9.1.1. Çizim: matplotlib 3.11.1.

**Spec:** [docs/superpowers/specs/2026-08-19-environment-design.md](../specs/2026-08-19-environment-design.md)

## Global Kısıtlar

- `src/environment.py` **`src/dubins.py`'yi import etmez**. Bağımsızlık kasıtlı.
- numpy kullanılmaz. Sadece `math`, `dataclasses`, `random`.
- ENU çerçevesi: x doğu, y kuzey, metre. `Pose = tuple[float, float, float]`
  takma adı bu modülde ayrıca tanımlanır.
- **Sınır üzerinde olmak çarpışma sayılır**: mesafe `<= radius + clearance`
  ise nokta serbest değildir.
- `Obstacle.radius <= 0` → `ValueError`. `clearance < 0` → `ValueError`.
  `xmin >= xmax` veya `ymin >= ymax` → `ValueError`.
- Python komutu: `venv\Scripts\python.exe` (cmd) / `.\venv\Scripts\python.exe` (PowerShell)
- Her task sonunda commit atılır.

---

### Task 1: `Obstacle`

**Dosyalar:**
- Oluştur: `src/environment.py`
- Test: `tests/test_environment.py`

**Arayüz:**
- Üretir:
  - `Pose = tuple[float, float, float]`
  - `Point = tuple[float, float]`
  - `Obstacle` — frozen dataclass, alanlar sırayla `x: float`, `y: float`,
    `radius: float`
  - `Obstacle.contains(point, clearance=0.0) -> bool`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
"""src/environment.py icin testler."""

import math
import random

import pytest

from src.environment import Obstacle


class TestObstacle:
    def test_fields_are_positional_in_order(self):
        obs = Obstacle(3.0, -4.0, 2.5)
        assert obs.x == 3.0
        assert obs.y == -4.0
        assert obs.radius == 2.5

    def test_is_frozen(self):
        obs = Obstacle(0.0, 0.0, 1.0)
        with pytest.raises(Exception):
            obs.radius = 5.0

    @pytest.mark.parametrize("bad_radius", [0.0, -1.0, -0.001])
    def test_nonpositive_radius_raises(self, bad_radius):
        with pytest.raises(ValueError):
            Obstacle(0.0, 0.0, bad_radius)

    def test_contains_center(self):
        assert Obstacle(5.0, 5.0, 2.0).contains((5.0, 5.0)) is True

    def test_contains_inside(self):
        assert Obstacle(0.0, 0.0, 2.0).contains((1.0, 1.0)) is True

    def test_contains_outside(self):
        assert Obstacle(0.0, 0.0, 2.0).contains((3.0, 0.0)) is False

    def test_boundary_counts_as_inside(self):
        # tam cember uzerinde: muhafazakar taraf carpisma sayar
        assert Obstacle(0.0, 0.0, 2.0).contains((2.0, 0.0)) is True

    def test_clearance_inflates(self):
        obs = Obstacle(0.0, 0.0, 2.0)
        assert obs.contains((3.0, 0.0)) is False
        assert obs.contains((3.0, 0.0), clearance=1.5) is True

    def test_accepts_pose_ignoring_yaw(self):
        # Pose verilirse ilk iki bileseni kullanilir
        obs = Obstacle(0.0, 0.0, 2.0)
        assert obs.contains((1.0, 1.0, 3.14)) is True
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

Çalıştır: `venv\Scripts\python.exe -m pytest tests\test_environment.py -q`
Beklenen: `ImportError: cannot import name 'Obstacle'`

- [ ] **Adım 3: `Obstacle`'ı yaz — OZAN**

`src/environment.py` içine modül docstring'i (koordinat sözleşmesini ve
`dubins.py`'yi neden import etmediğini yaz), `import math`,
`from dataclasses import dataclass`, sonra iki tip takma adı:

```python
Pose = tuple[float, float, float]   # (x, y, yaw)
Point = tuple[float, float]         # (x, y)
```

Sonra `@dataclass(frozen=True)` ile `Obstacle`: `x`, `y`, `radius` — bu sırayla.

**Doğrulama nasıl yazılır (yeni konu):** `frozen=True` dataclass'ta `__init__`
otomatik üretiliyor, sen yazmıyorsun. Doğrulama eklemek için `__post_init__`
adında özel bir metot tanımlarsın; Python nesne kurulduktan hemen sonra onu
kendiliğinden çağırır:

```python
    def __post_init__(self):
        if ...:
            raise ValueError(...)
```

İçinde `self.radius` okuyabilirsin ama **yazamazsın** (frozen). Burada sadece
okuyup hata fırlatacağız, sorun yok.

**`contains(point, clearance=0.0) -> bool`:** Nokta bu engelin (şişirilmiş)
içinde mi. `clearance=0.0` varsayılan değer — çağıran vermezse sıfır kullanılır.

Mantık: nokta ile merkez arasındaki mesafeyi `math.hypot` ile hesapla,
`radius + clearance` ile karşılaştır. **`<=` kullan**, `<` değil — sınır üzeri
çarpışma sayılıyor.

`point` üç elemanlı da gelebilir (`Pose`); `point[0]` ve `point[1]` yazarsan
her iki durum da çalışır. Üçüncü elemanı hiç okuma.

Docstring'e `clearance`'ın ne olduğunu ve sınır kuralını yaz.

- [ ] **Adım 4: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/environment.py tests/test_environment.py
git commit -m "Environment: dairesel engel gosterimi (Obstacle)"
```

---

### Task 2: `Environment` ve sınır kontrolü

**Dosyalar:**
- Değiştir: `src/environment.py`
- Test: `tests/test_environment.py`

**Arayüz:**
- Tüketir: `Obstacle`
- Üretir:
  - `Environment` — frozen dataclass, alanlar sırayla
    `bounds: tuple[float, float, float, float]`,
    `obstacles: tuple[Obstacle, ...]`,
    `clearance: float`
  - `Environment.is_inside_bounds(point) -> bool`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
from src.environment import Environment, Obstacle

BOUNDS = (0.0, 0.0, 100.0, 60.0)


class TestEnvironmentValidation:
    def test_fields_are_positional_in_order(self):
        obs = (Obstacle(1.0, 2.0, 3.0),)
        env = Environment(BOUNDS, obs, 1.5)
        assert env.bounds == BOUNDS
        assert env.obstacles == obs
        assert env.clearance == 1.5

    def test_is_frozen(self):
        env = Environment(BOUNDS, (), 0.0)
        with pytest.raises(Exception):
            env.clearance = 5.0

    def test_empty_obstacle_list_is_allowed(self):
        env = Environment(BOUNDS, (), 0.0)
        assert env.obstacles == ()

    @pytest.mark.parametrize("bad_clearance", [-0.1, -5.0])
    def test_negative_clearance_raises(self, bad_clearance):
        with pytest.raises(ValueError):
            Environment(BOUNDS, (), bad_clearance)

    def test_zero_clearance_is_allowed(self):
        assert Environment(BOUNDS, (), 0.0).clearance == 0.0

    @pytest.mark.parametrize("bad_bounds", [
        (10.0, 0.0, 10.0, 60.0),    # xmin == xmax
        (10.0, 0.0, 5.0, 60.0),     # xmin > xmax
        (0.0, 10.0, 100.0, 10.0),   # ymin == ymax
        (0.0, 10.0, 100.0, 5.0),    # ymin > ymax
    ])
    def test_invalid_bounds_raise(self, bad_bounds):
        with pytest.raises(ValueError):
            Environment(bad_bounds, (), 0.0)


class TestBounds:
    def _env(self):
        return Environment(BOUNDS, (), 0.0)

    def test_point_inside(self):
        assert self._env().is_inside_bounds((50.0, 30.0)) is True

    @pytest.mark.parametrize("point", [
        (-1.0, 30.0),    # sol disi
        (101.0, 30.0),   # sag disi
        (50.0, -1.0),    # alt disi
        (50.0, 61.0),    # ust disi
    ])
    def test_point_outside(self, point):
        assert self._env().is_inside_bounds(point) is False

    @pytest.mark.parametrize("corner", [
        (0.0, 0.0), (100.0, 0.0), (0.0, 60.0), (100.0, 60.0),
    ])
    def test_corners_are_inside(self, corner):
        # sinir uzerindeki nokta haritanin ICINDE sayilir
        assert self._env().is_inside_bounds(corner) is True

    def test_accepts_pose_ignoring_yaw(self):
        assert self._env().is_inside_bounds((50.0, 30.0, 2.0)) is True
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `Environment`'ı yaz — OZAN**

`@dataclass(frozen=True)` ile üç alan, **bu sırayla**:

```
bounds     -> tuple[float, float, float, float]   # (xmin, ymin, xmax, ymax)
obstacles  -> tuple[Obstacle, ...]
clearance  -> float
```

`tuple[Obstacle, ...]` "kaç tane olduğu belli olmayan, hepsi `Obstacle` olan
bir tuple" demek. `Obstacle` üçlüsündeki `tuple[float, float, float]` gibi
sabit uzunluk değil.

Neden liste değil tuple: `frozen=True` nesnenin değişmezliğini vaat ediyor ama
içindeki liste yine de değiştirilebilirdi. Tuple ile vaat gerçek oluyor.

`__post_init__` içinde iki doğrulama:
- `clearance < 0` → `ValueError`
- `bounds`'u dört değişkene açıp `xmin >= xmax` veya `ymin >= ymax` → `ValueError`

Hata mesajlarına gelen değerleri koy.

**`is_inside_bounds(point) -> bool`:** Nokta harita dikdörtgeninin içinde mi.
Sınır üzeri **içeride** sayılır (`<=` ve `>=`).

Dikkat: burada `<=` içeride demek, `Obstacle.contains`'te ise `<=` çarpışma
demekti. İkisi de "sınır dahil" kuralının aynı uygulaması — biri serbest
bölgenin sınırı, diğeri yasak bölgenin sınırı.

Python'da zincirleme karşılaştırma yazabilirsin: `xmin <= x <= xmax` geçerli
ve `xmin <= x and x <= xmax` ile aynı şey.

Sınıf ve metot docstring'leri yaz.

- [ ] **Adım 4: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/environment.py tests/test_environment.py
git commit -m "Environment: harita sinirlari ve dogrulama"
```

---

### Task 3: `is_free`

**Dosyalar:**
- Değiştir: `src/environment.py`
- Test: `tests/test_environment.py`

**Arayüz:**
- Tüketir: `Obstacle.contains`, `Environment.is_inside_bounds`
- Üretir: `Environment.is_free(point) -> bool`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
class TestIsFree:
    def test_empty_map_everything_inside_is_free(self):
        env = Environment(BOUNDS, (), 0.0)
        for point in [(0.0, 0.0), (50.0, 30.0), (100.0, 60.0)]:
            assert env.is_free(point) is True

    def test_outside_bounds_is_not_free(self):
        env = Environment(BOUNDS, (), 0.0)
        assert env.is_free((150.0, 30.0)) is False

    def test_inside_obstacle_is_not_free(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((50.0, 30.0)) is False
        assert env.is_free((55.0, 30.0)) is False

    def test_outside_obstacle_is_free(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((70.0, 30.0)) is True

    def test_obstacle_boundary_is_not_free(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((60.0, 30.0)) is False

    def test_clearance_makes_previously_free_point_blocked(self):
        obstacles = (Obstacle(50.0, 30.0, 10.0),)
        assert Environment(BOUNDS, obstacles, 0.0).is_free((62.0, 30.0)) is True
        assert Environment(BOUNDS, obstacles, 5.0).is_free((62.0, 30.0)) is False

    def test_any_obstacle_blocks(self):
        env = Environment(BOUNDS, (
            Obstacle(20.0, 20.0, 5.0),
            Obstacle(50.0, 30.0, 5.0),
            Obstacle(80.0, 40.0, 5.0),
        ), 0.0)
        assert env.is_free((50.0, 30.0)) is False   # ikinciye giriyor
        assert env.is_free((80.0, 40.0)) is False   # ucuncuye giriyor
        assert env.is_free((5.0, 50.0)) is True     # hicbirine girmiyor

    def test_point_inside_obstacle_but_outside_bounds(self):
        # iki sebep birden: yine de False
        env = Environment(BOUNDS, (Obstacle(105.0, 30.0, 10.0),), 0.0)
        assert env.is_free((105.0, 30.0)) is False

    def test_accepts_pose_ignoring_yaw(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)
        assert env.is_free((50.0, 30.0, 1.2)) is False
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `is_free`'yi yaz — OZAN**

Sözleşme: Nokta hem harita sınırları içinde **hem de** hiçbir engelin
(emniyet payıyla şişirilmiş hâlinin) içinde değilse `True`.

Adımlar:
1. `is_inside_bounds(point)` yanlışsa hemen `False`
2. Her engel için `obstacle.contains(point, self.clearance)` — biri bile
   `True` ise `False` döndür
3. Hiçbiri değilse `True`

Döngüyle yazabilirsin. Daha kısa yolu: Python'un `any()` fonksiyonu bir
diziden herhangi biri doğruysa `True` verir:

```python
any(obs.contains(point, self.clearance) for obs in self.obstacles)
```

Bu bir *generator expression* — liste kurmadan tek tek üretir ve ilk `True`'da
durur. Engel listesi uzunsa gereksiz iş yapmamış olur.

Kullanırsan sonucun **tersini** istediğine dikkat: "herhangi bir engele giriyor
mu" sorusunun cevabı `True` ise nokta serbest **değil**.

- [ ] **Adım 4: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/environment.py tests/test_environment.py
git commit -m "Environment: nokta serbest mi kontrolu (is_free)"
```

---

### Task 4: `is_path_free` ve `suggested_step`

**Dosyalar:**
- Değiştir: `src/environment.py`
- Test: `tests/test_environment.py`

**Arayüz:**
- Tüketir: `Environment.is_free`
- Üretir:
  - `Environment.is_path_free(points) -> bool`
  - `Environment.suggested_step() -> float`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
class TestIsPathFree:
    def _env(self):
        return Environment(BOUNDS, (Obstacle(50.0, 30.0, 10.0),), 0.0)

    def test_clear_path_is_free(self):
        points = [(x, 5.0) for x in range(0, 101, 5)]
        assert self._env().is_path_free(points) is True

    def test_path_through_obstacle_is_blocked(self):
        points = [(x, 30.0) for x in range(0, 101, 5)]
        assert self._env().is_path_free(points) is False

    def test_single_blocked_point_is_enough(self):
        points = [(5.0, 5.0), (50.0, 30.0), (95.0, 55.0)]
        assert self._env().is_path_free(points) is False

    def test_path_leaving_bounds_is_blocked(self):
        points = [(50.0, 5.0), (150.0, 5.0)]
        assert self._env().is_path_free(points) is False

    def test_empty_list_is_free(self):
        # bos kume: kontrol edilecek bir sey yok
        assert self._env().is_path_free([]) is True

    def test_accepts_poses(self):
        poses = [(5.0, 5.0, 0.0), (10.0, 5.0, 0.5)]
        assert self._env().is_path_free(poses) is True


class TestSuggestedStep:
    def test_half_of_smallest_inflated_radius(self):
        env = Environment(BOUNDS, (
            Obstacle(20.0, 20.0, 8.0),
            Obstacle(50.0, 30.0, 3.0),   # en kucuk
            Obstacle(80.0, 40.0, 5.0),
        ), 0.0)
        assert math.isclose(env.suggested_step(), 1.5)

    def test_clearance_counts_toward_radius(self):
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 3.0),), 1.0)
        assert math.isclose(env.suggested_step(), 2.0)

    def test_no_obstacles_returns_positive_value(self):
        step = Environment(BOUNDS, (), 0.0).suggested_step()
        assert step > 0.0

    def test_no_obstacles_scales_with_map(self):
        small = Environment((0.0, 0.0, 10.0, 10.0), (), 0.0).suggested_step()
        big = Environment((0.0, 0.0, 1000.0, 1000.0), (), 0.0).suggested_step()
        assert big > small

    def test_step_is_small_enough_to_catch_obstacle(self):
        # onerilen adimla ornekelenen bir dogru, engeli atlamamali
        env = Environment(BOUNDS, (Obstacle(50.0, 30.0, 4.0),), 0.0)
        step = env.suggested_step()
        n = int(100.0 / step) + 1
        points = [(i * step, 30.0) for i in range(n)]
        assert env.is_path_free(points) is False
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `is_path_free`'yi yaz — OZAN**

Sözleşme: Verilen noktaların **hepsi** serbestse `True`. Boş liste `True`
döner — kontrol edilecek bir şey yok.

`any()`'nin kardeşi `all()` işini görür: bir diziden hepsi doğruysa `True`
verir, boş dizide de `True` verir (tam istediğimiz davranış).

```python
all(self.is_free(point) for point in points)
```

Docstring'de **mutlaka** şunu yaz: bu kontrol **yaklaşıktır**. İki örnek
noktası arasında yol bir engelin kenarından teğet geçip hiçbir nokta bırakmadan
çıkabilir. Emniyet payı ve `suggested_step()` bu riski sınırlar ama sıfırlamaz.
Bu bilgi saklanacak bir şey değil.

- [ ] **Adım 4: `suggested_step`'i yaz — OZAN**

Sözleşme: Örnekleme için önerilen adım boyu (metre).

- Engel varsa: en küçük **şişirilmiş** yarıçapın (yani `radius + clearance`)
  yarısı.
- Engel yoksa: haritanın kısa kenarının onda biri.

Gerekçe: bir engelin içinden geçen yolun kat ettiği mesafe en az çapı kadardır.
Adım yarıçapın yarısından küçükse, o mesafe içine en az bir örnek nokta düşer.

`min(...)` bir diziden en küçüğü verir:
```python
min(obs.radius + self.clearance for obs in self.obstacles)
```

Engel yoksa `min()` boş dizide `ValueError` fırlatır — önce `if not
self.obstacles:` ile o durumu ayır. Harita genişliği `xmax - xmin`, yüksekliği
`ymax - ymin`; ikisinin küçüğünün onda biri.

- [ ] **Adım 5: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 6: Commit — CLAUDE**

```bash
git add src/environment.py tests/test_environment.py
git commit -m "Environment: yol carpisma kontrolu ve onerilen adim boyu"
```

---

### Task 5: `random_free_pose`

**Dosyalar:**
- Değiştir: `src/environment.py`
- Test: `tests/test_environment.py`

**Arayüz:**
- Tüketir: `Environment.is_free`
- Üretir: `Environment.random_free_pose(rng, max_attempts=1000) -> Pose`

- [ ] **Adım 1: Başarısız testi yaz — CLAUDE**

```python
class TestRandomFreePose:
    def _env(self):
        return Environment(BOUNDS, (
            Obstacle(30.0, 30.0, 12.0),
            Obstacle(70.0, 30.0, 12.0),
        ), 2.0)

    def test_returns_three_element_pose(self):
        pose = self._env().random_free_pose(random.Random(1))
        assert len(pose) == 3

    def test_result_is_always_free(self):
        env = self._env()
        rng = random.Random(7)
        for _ in range(200):
            assert env.is_free(env.random_free_pose(rng)) is True

    def test_result_is_inside_bounds(self):
        env = self._env()
        rng = random.Random(8)
        xmin, ymin, xmax, ymax = BOUNDS
        for _ in range(200):
            x, y, _yaw = env.random_free_pose(rng)
            assert xmin <= x <= xmax
            assert ymin <= y <= ymax

    def test_yaw_is_normalized(self):
        env = self._env()
        rng = random.Random(9)
        for _ in range(200):
            _x, _y, yaw = env.random_free_pose(rng)
            assert 0.0 <= yaw < 2 * math.pi

    def test_same_seed_gives_same_result(self):
        env = self._env()
        first = env.random_free_pose(random.Random(42))
        second = env.random_free_pose(random.Random(42))
        assert first == second

    def test_different_seeds_give_different_results(self):
        env = self._env()
        assert env.random_free_pose(random.Random(1)) != \
            env.random_free_pose(random.Random(2))

    def test_full_map_raises_runtime_error(self):
        # tum haritayi kaplayan engel: serbest poz bulunamaz
        env = Environment((0.0, 0.0, 10.0, 10.0),
                          (Obstacle(5.0, 5.0, 100.0),), 0.0)
        with pytest.raises(RuntimeError):
            env.random_free_pose(random.Random(1))

    def test_max_attempts_is_respected(self):
        env = Environment((0.0, 0.0, 10.0, 10.0),
                          (Obstacle(5.0, 5.0, 100.0),), 0.0)
        with pytest.raises(RuntimeError):
            env.random_free_pose(random.Random(1), max_attempts=5)
```

- [ ] **Adım 2: Testi çalıştır, başarısız olduğunu gör — CLAUDE**

- [ ] **Adım 3: `random_free_pose`'u yaz — OZAN**

Sözleşme: Serbest uzaydan rastgele bir poz döndürür. `rng` bir
`random.Random` örneği — dışarıdan alınması testlerin tekrarlanabilir olması
için şart.

**Reddetme yöntemi (rejection sampling):** Harita dikdörtgeninden rastgele bir
nokta üret, serbest mi diye bak, değilse at ve yeniden dene. Engellerin
şeklinden bağımsız çalışır; poligon veya ızgaraya geçsen de aynı kod işler.

Adımlar:
1. `max_attempts` kere dene:
   - `rng.uniform(xmin, xmax)` ve `rng.uniform(ymin, ymax)` ile nokta üret
   - `rng.uniform(0, 2 * math.pi)` ile yaw üret
   - poz serbestse döndür
2. Döngü hiç sonuç vermeden biterse `RuntimeError` — mesaja `max_attempts`
   değerini ve haritanın ne kadar dolu olduğunu düşündüren bir ipucu koy

`rng.uniform(a, b)` `a` ile `b` arasında ondalık sayı verir.

**Neden `max_attempts` var:** Harita tamamen doluysa `while True` sonsuza kadar
döner ve program kilitlenir. Sınırlı deneme, hatayı görünür kılar. RRT*
yazarken bu hatayı alırsan "engeller çok büyük veya clearance çok yüksek"
demektir — sessiz bir donmadansa açık bir hata çok daha iyi.

Fonksiyonun `for` döngüsünden sonra gelen `raise` satırına ulaşması, tüm
denemelerin başarısız olduğu anlamına gelir. `for ... else` yapısını bilmene
gerek yok; döngüden sonra doğrudan `raise` yazman yeterli, çünkü başarılı
durumda zaten `return` ile çıkmış olacaksın.

- [ ] **Adım 4: Testi çalıştır, geçtiğini gör — CLAUDE**

- [ ] **Adım 5: Commit — CLAUDE**

```bash
git add src/environment.py tests/test_environment.py
git commit -m "Environment: serbest uzaydan rastgele poz uretimi"
```

---

### Task 6: Görselleştirme demosu

**Dosyalar:**
- Oluştur: `notebooks/environment_demo.py`

**Arayüz:**
- Tüketir: `Environment`, `Obstacle`, `Environment.is_path_free`,
  `Environment.suggested_step`, `Environment.random_free_pose`,
  `src.dubins.shortest_path`, `DubinsPath.sample`

Not: **demo** her iki modülü de import edebilir. Bağımsızlık kuralı
`src/environment.py` için geçerli, onu tüketen betikler için değil.

- [ ] **Adım 1: Demoyu yaz — OZAN**

Sözleşme: Engelli bir harita kur, üstünde birkaç Dubins yolu dene, hangilerinin
serbest hangilerinin çarptığını göster.

Yapılacaklar:

- `BOUNDS = (0.0, 0.0, 100.0, 60.0)` ve 3-5 engelden oluşan bir harita kur,
  `clearance` 2.0 gibi bir değer
- Engelleri çiz: `matplotlib.patches.Circle` kullanılır —
  `ax.add_patch(Circle((obs.x, obs.y), obs.radius, color="gray", alpha=0.6))`
- Emniyet payını ayrı ve soluk bir halka olarak çiz
  (`radius + clearance`, `fill=False`, kesikli çizgi: `linestyle="--"`)
- Sabit bir başlangıç pozu seç; `random_free_pose` ile 6-8 hedef poz üret
  (tohumlanmış `random.Random` kullan ki her çalıştırmada aynı şekil çıksın)
- Her hedef için `shortest_path(start, goal, RHO)` hesapla,
  `path.sample(env.suggested_step())` ile örnekle
- `env.is_path_free(points)` sonucuna göre yolu **yeşil** (serbest) veya
  **kırmızı** (çarpıyor) çiz
- Başlangıç ve hedef pozları ok olarak göster —
  `notebooks/dubins_demo.py`'deki `draw_pose` fonksiyonunun aynısını kullan
- `ax.set_aspect("equal")` — bu olmadan daireler elips görünür ve şekil yanıltır
- Başlığa kaç yolun serbest çıktığını yaz
- `results/environment_demo.png` olarak kaydet (`dpi=150`)
- En altta `if __name__ == "__main__":` bloğu

- [ ] **Adım 2: Çalıştır ve şekli incele — CLAUDE**

Çalıştır: `venv\Scripts\python.exe -m notebooks.environment_demo`
Beklenen: `results/environment_demo.png` oluşur. Kırmızı yolların gerçekten
engellerden geçtiği, yeşillerin geçmediği gözle doğrulanabilir olmalı.

- [ ] **Adım 3: Commit — CLAUDE**

```bash
git add notebooks/environment_demo.py results/environment_demo.png
git commit -m "Environment: carpisma kontrolunu gosteren gorsel demo"
```

---

## Bu planın dışında, sonraki tur

`src/rrt_star.py` — Dubins'i `path_length` (maliyet) ve `sample` (çarpışma
kontrolü noktaları), Environment'ı `random_free_pose` ve `is_path_free`
üzerinden tüketir.

Tasarımda ilk konuşulacak konu: **Dubins mesafesi simetrik değildir**
(`path_length(A, B) != path_length(B, A)`). RRT*'ın "en yakın komşu" ve
"yeniden bağlama" adımları çoğu anlatımda simetrik mesafe varsayar; yön
karıştırılırsa sessizce yanlış ağaç kurulur.

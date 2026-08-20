# 3B Planlama Yığını — Tasarım

**Tarih:** 2026-08-20
**Dosyalar:** `src/environment3d.py`, `src/rrt_star3d.py` (ikisi de yeni)
**Önceki aşama:** `2026-08-20-dubins3d-design.md`

## Amaç

3B yol modelinin (`dubins3d.py`) üstüne 3B ortam ve 3B planlayıcı koyarak
silindir engeller arasından geçen, tırmanma açısı sınırına uyan rotalar
üretmek. Hedef çıktı: üç panelli bir görselleştirme (3B perspektif, yandan
görünüm, yukarıdan düzleştirilmiş).

## Kapsam

İki parça, sırayla:

1. `src/environment3d.py` — silindir engeller, irtifa sınırları, çarpışma
   kontrolü
2. `src/rrt_star3d.py` — dört elemanlı pozla çalışan RRT\*

Kapsam dışı: **formasyon uçuşu / takipçi yörüngeleri** (referans şekildeki mavi
eğriler). Lider rotasından türetilen ayrı bir özellik; planlama bittikten sonra
eklenecek. Ayrıca rüzgâr ve gerçek arazi verisi.

## Dosya Ayrımı

Yeni dosyalar; `environment.py` ve `rrt_star.py` **değiştirilmiyor**.

Gerekçe: 2B yığın 348 testle çalışıyor ve 2B demolar rapora giriyor. 2B/3B
karşılaştırması yapılabilmesi için ikisinin de ayakta kalması gerekiyor.

**Kabul edilen bedel:** RRT\* döngüsünün yaklaşık 150 satırı iki dosyada
duracak. Birinde bulunan bir hata diğerine elle taşınmalı. Alternatif —
planlayıcıyı yol modelinden bağımsız hale getirmek — daha temiz ama her
fonksiyonun imzasını değiştiriyor ve bir soyutlama katmanı ekliyor; bu
aşamada risk/fayda dengesi tutmuyor.

## Bölüm 1: `environment3d.py`

```python
Point3 = tuple[float, float, float]              # x, y, z
Pose3 = tuple[float, float, float, float]        # x, y, z, yaw
Bounds3 = tuple[float, float, float, float, float, float]
```

`Bounds3` sırası: `(x_min, y_min, z_min, x_max, y_max, z_max)`.

### `Cylinder`

```python
@dataclass(frozen=True)
class Cylinder:
    x: float
    y: float
    radius: float
    z_min: float
    z_max: float

    def contains(self, point: Point3 | Pose3, clearance: float = 0.0) -> bool
```

`contains` **iki koşulu birden** ister:

```
yatay mesafe <= radius + clearance
z_min - clearance <= z <= z_max + clearance
```

Biri sağlanmıyorsa nokta dışarıda. Yatay mesafe `math.hypot(px - x, py - y)`.

Emniyet payı dikeyde de uygulanıyor. Bu, silindirin üst ve alt kenarlarında
gerçek bir "şişirilmiş kapsül"den biraz **fazla** yer kaplar — köşelerde pay
`clearance` yerine `clearance*sqrt(2)`'ye kadar çıkabilir. Fazla olması güvenli
tarafta kalmak demek, kabul ediliyor.

**Doğrulama (`__post_init__`):**

```
radius <= 0        -> ValueError
z_max <= z_min     -> ValueError
```

### `Environment3D`

```python
@dataclass(frozen=True)
class Environment3D:
    bounds: Bounds3
    obstacles: tuple[Cylinder, ...]
    clearance: float

    def is_inside_bounds(self, point: Point3 | Pose3) -> bool
    def is_free(self, point: Point3 | Pose3) -> bool
    def is_path_free(self, points: Iterable[Point3 | Pose3]) -> bool
    def suggested_step(self) -> float
    def random_free_pose(self, rng: random.Random,
                         max_attempts: int = 1000) -> Pose3
```

`is_free`: sınır içinde **ve** hiçbir silindirde değil.

`is_path_free`: nokta listesinin tamamı serbest mi.

`random_free_pose`: reddetme örneklemesi; x, y, z sınırlar içinden, yaw
`[0, 2*pi)`'den. `max_attempts` sonunda bulunamazsa `RuntimeError`.

### `suggested_step` 3B'de

2B kural "en küçük şişirilmiş yarıçapın yarısı" idi. 3B'de yetmiyor: alçak ve
geniş bir silindir dikey yönde atlanabilir.

Her silindir için şişirilmiş en küçük yarı-boyut:

```
min(radius + clearance, (z_max - z_min) / 2 + clearance)
```

Bunların en küçüğünün yarısı adım olur. Engel yoksa haritanın en kısa
kenarının onda biri.

2B'deki uyarı burada da geçerli: bu bir sezgisel kural, ayrıklaştırma hatasını
tamamen kapatmıyor; emniyet payı onu yutuyor.

## Bölüm 2: `rrt_star3d.py`

`rrt_star.py`'nin birebir yapısı. Üç fark:

| | 2B | 3B |
|---|---|---|
| Kenar kurma | `shortest_path` | `airplane_path` |
| Mesafe | `path_length` | `airplane_length` |
| Ekstra parametre | — | `gamma_max`, her fonksiyona |

### Veri yapıları

```python
@dataclass(frozen=True)
class Node3:
    pose: Pose3
    parent: int | None
    cost: float
    path_from_parent: DubinsPath3D | None


@dataclass(frozen=True)
class RRTResult3:
    found: bool
    edges: list[DubinsPath3D]
    cost: float
    iterations: int
    tree: list[Node3]
```

### Fonksiyonlar

```python
_try_connect(env, from_pose, to_pose, rho, gamma_max, step) -> DubinsPath3D | None
_nearest(nodes, target, rho, gamma_max) -> int
_sample(env, goal, rng, goal_bias) -> Pose3
_extract_path(nodes, index, goal_edge) -> list[DubinsPath3D]
_neighbour_radius(n, gamma, cap) -> float
_neighbours(nodes, pose, radius) -> list[int]
_choose_parent(env, nodes, pose, candidates, rho, gamma_max, step,
               fallback) -> tuple[int, DubinsPath3D]
_rewire(env, nodes, new_index, candidates, rho, gamma_max, step) -> None
_propagate_cost(nodes, index) -> None

plan3d(start, goal, env, rho, gamma_max,
       max_iterations=5000, goal_bias=0.05, step=None, rng=None,
       stop_on_first_solution=False,
       radius_gamma=80.0, radius_cap=30.0) -> RRTResult3
```

`_sample`, `_extract_path` ve `_propagate_cost` 2B'dekiyle **aynı gövdeye**
sahip; yalnızca tip adları değişiyor.

### Yarıçap üssü 1/4

Konfigürasyon uzayı artık dört boyutlu: x, y, z, yaw.

```
r_n = min(gamma * (log n / n)^(1/4), cap)
```

2B'de üs `1/3` idi (SE(2)). Aynı `gamma` değeriyle 3B'de yarıçap daha yavaş
küçülür. Varsayılanlar `radius_gamma = 80.0`, `radius_cap = 30.0`; 100×100×80 m
ölçeğinde yarıçap 100 düğümde 30 m (tavan), 300'de 27.6 m, 800'de 24.1 m,
2000'de 21.3 m olur. Harita ölçeği değişirse elle ayarlanmalı — 2B'de bu
gözden kaçmış ve RRT\* neredeyse işlevsiz kalmıştı.

2B'deki dürüstlük notu burada da geçerli, hatta daha güçlü: formül Öklid
uzayında ispatlanmış, Dubins airplane uzayı ne Öklid ne de simetrik.

### Öklid ön eleme 3B'de de kayıpsız

Bir yol iki nokta arasındaki düz çizgiden kısa olamaz; 3B'de de öyle.
`math.hypot` üç argüman alıyor.

### Asimetri

`airplane_length(A, B) != airplane_length(B, A)` — hem yatay Dubins
asimetrisinden hem de tırmanış/alçalışın farklı helis sayısı gerektirmesinden.
`_choose_parent` ve `_rewire` yönleri 2B'deki gibi ayrı ayrı ölçüyor.

### Performans

Helisli yollar uzun oluyor (bir demo yolunda 124 m yatay). `sample(step)` çok
nokta üretiyor, çarpışma kontrolü pahalılaşıyor. 2B'de test suitesi 30 saniyeye
çıkmıştı; 3B testlerinde bütçeler baştan küçük tutulacak ve ağır koşular
`functools.lru_cache` ile paylaşılacak.

## Testler

**Ortam:**

- `Cylinder.contains`: yatay ve dikey koşulun **ayrı ayrı** eleyebildiği —
  silindirin tam üstünde ama yanında olan nokta serbest, tam içindeki değil
- Emniyet payı yatayda ve dikeyde uygulanıyor
- `radius <= 0`, `z_max <= z_min` → `ValueError`
- `is_free`: sınır dışı, engel içi, serbest
- `suggested_step`: alçak ve geniş silindirde yükseklik kuralının devreye
  girdiği
- `random_free_pose`: dört elemanlı, serbest, tohumlanmış aynı sonucu veriyor

**Planlayıcı:**

- Yapısal değişmezler (2B'dekinin aynısı): `cost == ebeveyn.cost + kenar.length`,
  ebeveyn zinciri kökte bitiyor, döngü yok — birkaç tohumda
- Rota çarpışmasız: ince adımla (`0.05 m`) yeniden sınanıyor
- Rota sürekli: her kenarın sonu bir sonrakinin başlangıcı
- **Alçak engelin üstünden geçebiliyor:** tavana kadar giden bir duvar rotayı
  bloklarken, aynı duvar alçaltıldığında rota bulunuyor. 3B planlamanın
  kazandırdığı şey tam bu; ayrı bir test hak ediyor
- `gamma_max` ihlal edilmiyor: her kenarın `gamma`'sı limit içinde
- Doğrulama: geçersiz `rho`, `gamma_max`, `max_iterations`, `goal_bias`,
  `step`, engelli `start`/`goal` → `ValueError`
- Aynı tohum aynı sonucu veriyor

## Demo

`notebooks/rrt3d_demo.py`, referans şekildeki üç panel:

- **a.** 3B perspektif — silindirler, ağaç (açık gri), rota (kalın)
- **b.** Yandan görünüm (X-Z) — irtifa profilini gösterir
- **c.** Yukarıdan düzleştirilmiş (X-Y) — silindirler daire olarak

Engellerin bir kısmı alçak olacak ki rotanın **üstünden geçtiği** görülsün.
Başlıkta düğüm sayısı, rota uzunluğu, `rho` ve `gamma_max`.

## Kapsam Dışı

- Formasyon uçuşu / takipçi yörüngeleri
- Küre engeller, arazi yüzeyi
- Rüzgâr
- Planlayıcının 2B ve 3B için tek dosyada birleştirilmesi

# Dubins Airplane (3B) — Tasarım

**Tarih:** 2026-08-20
**Dosya:** `src/dubins3d.py` (yeni)
**Önceki aşamalar:** `2026-08-18-dubins-design.md`, `2026-08-19-rrt-design.md`,
`2026-08-19-rrt-star-design.md`

## Amaç

Mevcut 2B Dubins yolunu irtifa boyutuyla genişletmek: sabit kanatlı bir uçağın
tırmanma açısı sınırına uyan, `(x, y, z, yaw)` pozları arasında yol üreten bir
model.

## Kapsam

Bu spec **yalnızca yol modelini** kapsıyor. 3B'ye geçiş üç turda yapılacak:

1. **Bu tur:** `src/dubins3d.py` — irtifa boyutlu yol
2. Sonra: `environment.py`'nin 3B'ye çıkması (silindir/küre engeller, irtifa
   sınırları)
3. Sonra: `rrt_star.py`'nin dört elemanlı poza geçmesi

Her tur çalışan ve test edilmiş kodla bitiyor.

## Dosya Ayrımı

Yeni dosya, `dubins.py` değiştirilmiyor. Gerekçe: 2B'nin 109 testi ve mevcut
2B planlayıcı riske girmesin. `dubins3d.py` mevcut `DubinsPath`'i **yatay
bileşen** olarak kullanıyor, geometriyi yeniden yazmıyor.

Bağımlılık tek yönlü: `dubins3d.py` → `dubins.py`. Tersi yok.

## Model

Poz `(x, y, z, yaw)`. Yeni parametre `gamma_max`: izin verilen en büyük
tırmanma/alçalma açısı, radyan.

```
L      = yatay Dubins yolunun uzunlugu
dz     = z_hedef - z_baslangic
L_req  = |dz| / tan(gamma_max)
```

**Durum 1 — `L >= L_req`:** uçak irtifayı mevcut yol boyunca kazanabiliyor.

```
helix_turns = 0
H = L
gamma = atan2(dz, L)
```

**Durum 2 — `L < L_req`:** yol kısa kalıyor, başlangıçta tam turlar atılıyor.

```
helix_turns = ceil((L_req - L) / (2*pi*rho))
H = L + helix_turns * 2*pi*rho
gamma = atan2(dz, H)
```

`H >= L_req` olduğundan `|gamma| <= gamma_max` garanti.

Her iki durumda **3B uzunluk = `sqrt(H^2 + dz^2)`**.

### Helis neden bitiş pozunu bozmuyor

`rho` yarıçaplı tam bir dönüş `2*pi*rho` uzunluğunda ve başlangıç pozuna
birebir döner. Dolayısıyla `k` tam tur, yatay yolun bitiş pozunu değiştirmeden
yatay uzunluğu `k*2*pi*rho` artırıyor.

Sayısal doğrulama yapıldı: `rho = 5`, başlangıç `(3, 7, 40°)`, k = 1 ve 2 için
bitiş pozu `(3.000000, 7.000000, 40.0000°)`.

### Helis mevcut arayüzle kuruluyor

```python
DubinsPath(start2d, direction + "S" + direction, (uzunluk, 0.0, 0.0), rho)
```

`direction` yatay yolun ilk harfi (L veya R). Altı kelimenin hepsi L veya R ile
başladığı için her zaman tanımlı. Sonuç `"LSL"` veya `"RSR"` — ikisi de geçerli
kelime; ikinci ve üçüncü parçalar sıfır uzunlukta.

`dubins.py`'nin özel (alt çizgili) fonksiyonlarına dokunulmuyor.

## Optimallik

**Durum 1'de yol optimal.** 3B uzunluk `sqrt(H^2 + dz^2)`, `H`'de artan bir
fonksiyon; `H`'nin alabileceği en küçük değer zaten en kısa 2B Dubins
uzunluğu.

**Durum 2'de optimal değil.** İdeal `H = L_req` olurdu, ama tam tur eklendiği
için üstüne çıkılıyor. Fazlalık en fazla bir tur:

```
H - L_req < 2*pi*rho
```

Literatürdeki "orta irtifa" durumu bunu yayları kısmen uzatarak çözüyor; bu
spec'in kapsamı dışında. **Raporda bu sınır açıkça yazılmalı** — fonksiyon adı
da bu yüzden `shortest_path_3d` değil `airplane_path`.

## Arayüz

```python
Pose3 = tuple[float, float, float, float]        # x, y, z, yaw


@dataclass(frozen=True)
class DubinsPath3D:
    start: Pose3
    horizontal: DubinsPath        # 2B yol; z bilmiyor
    helix_turns: int              # k, >= 0
    gamma: float                  # ucus yolu acisi, radyan

    @property
    def horizontal_length(self) -> float          # L + k*2*pi*rho
    @property
    def length(self) -> float                     # sqrt(H^2 + dz^2)
    def interpolate(self, s: float) -> Pose3
    def end_pose(self) -> Pose3
    def sample(self, step: float) -> list[Pose3]


def airplane_path(start: Pose3, goal: Pose3, rho: float,
                  gamma_max: float) -> DubinsPath3D

def airplane_length(start: Pose3, goal: Pose3, rho: float,
                    gamma_max: float) -> float
```

`airplane_length` **3B uzunluğu** döner (yatay değil) — planlayıcı mesafe
ölçütü olarak bunu kullanacak.

`interpolate` ve `sample` 2B'deki sözleşmeyi aynen sürdürüyor: `s` aralık
dışındaysa kırpılır (negatif → başlangıç, `length`'ten büyük → son poz);
`sample` ilk elemanı daima `start`, sonuncusu daima son poz olacak şekilde
`step`'i aşmayan aralıklarla örnekler ve `step <= 0` için `ValueError`
fırlatır.

### `interpolate` mantığı

3B yol boyunca `s` metre gidilince:

```
h = s * cos(gamma)                    # yatayda gidilen mesafe
z = start_z + h * tan(gamma)          # irtifa

h < k*2*pi*rho  ise:  yatay konum helis yolundan
aksi halde:           horizontal.interpolate(h - k*2*pi*rho)
```

Yaw her iki durumda da yatay bileşenden geliyor.

### Doğrulama

```
rho <= 0                          -> ValueError
gamma_max <= 0 veya >= pi/2       -> ValueError
```

`gamma_max = pi/2` dışlanıyor çünkü `tan(pi/2)` tanımsız ve fiziksel karşılığı
dik tırmanış.

## Testler

**1. Uçtan uca yeniden kurma (en güçlüsü).** Rastgele poz çiftleri × 3 `rho` ×
3 `gamma_max` için `end_pose()` hedefe varıyor mu — x, y, z ve yaw'da birden,
1e-6 tolerans. 2B'de bu test altı kelimenin hepsindeki formül hatasını
yakalamıştı.

**2. Açı kısıtı.** `|gamma| <= gamma_max` her zaman. Modelin tek fiziksel
kısıtı bu.

**3. Helis.** Yardımcı fonksiyon başlangıç pozuna dönüyor mu; `k` tur için
uzunluk `k*2*pi*rho` mu.

**4. Durum ayrımı.** `L >= L_req` iken `helix_turns == 0`; `L < L_req` iken
`>= 1`.

**5. Aşım sınırı.** `horizontal_length - L_req < 2*pi*rho`.

**6. 2B'ye indirgeme.** `dz == 0` iken `gamma == 0`, `helix_turns == 0`,
`length` 2B uzunluğa eşit, örneklenen bütün noktaların z'si sabit. 3B kod
2B'yi bozmadan kapsıyor.

**7. Uzunluk bağıntısı.** `length == sqrt(horizontal_length^2 + dz^2)`.

**8. Monotonluk.** Örneklenen noktalarda z tırmanışta artıyor, alçalışta
azalıyor.

**9. Uç noktalar.** `interpolate(0) == start`, `interpolate(length) ==
end_pose()`.

**10. Alçalış.** `dz < 0` simetrik çalışıyor.

**11. Yatay bileşen bozulmamış.** `path.horizontal`, `shortest_path` ile aynı
yolu veriyor.

**12. Doğrulama.** Geçersiz `rho` ve `gamma_max` `ValueError` fırlatıyor.

## Kapsam Dışı

- Tırmanma ve alçalma için **ayrı** açı limitleri (gerçekte farklıdır)
- Literatürdeki "orta irtifa" optimal çözümü
- Rüzgâr
- 3B engeller ve 3B planlayıcı (sonraki iki tur)

## Kaynak

Dubins airplane modeli: Chitsaz & LaValle (2007). Künye doğrulanmalı — bu
projede bir kez hafızadan aktarılan bir formül yanlış çıkmıştı.

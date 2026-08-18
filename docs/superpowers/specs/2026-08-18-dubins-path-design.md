# Dubins Path Modülü — Tasarım

Tarih: 2026-08-18
Durum: Onaylandı
Kapsam: `src/dubins.py`, `tests/test_dubins.py`, `notebooks/dubins_demo.py`

## Amaç

Fixed-wing bir İHA'nın minimum dönüş yarıçapı kısıtı altında, iki poz arasındaki
en kısa yolu analitik olarak üretmek. Modül hem tek başına kullanılabilir bir
kütüphane, hem de `rrt_star.py` için steering ve maliyet fonksiyonudur.

## Kapsam Dışı

- 3D / irtifa planlama. Bu sürüm sabit irtifada (2D) çalışır.
- Engel çarpışma kontrolü. O `environment.py`'nin sorumluluğudur; Dubins sadece
  örneklenmiş poz listesi sunar.
- RRT* algoritmasının kendisi. Ayrı bir tasarım turu.

## Koordinat ve Açı Sözleşmesi

`Pose = (x, y, yaw)`

- ENU çerçevesi: x doğu, y kuzey.
- `yaw` radyan, CCW pozitif, x ekseninden ölçülür.
- Havacılık heading'i (kuzeyden CW) bu modülün dışında dönüştürülür. Modül
  içinde tek sözleşme geçerlidir ve testlerle kilitlenir.

## Mimari

Üç katman, birbirinden bağımsız test edilebilir:

### 1. Kanonik çözücüler (private, saf fonksiyonlar)

`_lsl, _rsr, _lsr, _rsl, _rlr, _lrl`

Her biri `(d, alpha, beta)` alır, normalize segment uzunlukları `(t, p, q)`
döndürür. Geometrik olarak imkânsız bir kelime için `None` döner (karekök altı
negatif, `acos` argümanı [-1, 1] dışında). Dışarıyı tanımazlar.

Shkel & Lumelsky (2001) kanonik formu kullanılır.

### 2. `DubinsPath` (frozen dataclass)

Alanlar:
- `start: Pose`
- `word: str` — "LSL", "RSR", "LSR", "RSL", "RLR", "LRL"
- `lengths: tuple[float, float, float]` — segment yay uzunlukları, dünya birimi
- `rho: float`

Davranış:
- `length` (property) — toplam yol uzunluğu
- `interpolate(s) -> Pose` — yol başından s mesafedeki poz
- `sample(step) -> list[Pose]` — step aralıklı poz listesi; ilk eleman start,
  son eleman hedef pozdur
- `end_pose() -> Pose`

### 3. Genel API

```python
shortest_path(start, goal, rho) -> DubinsPath
all_paths(start, goal, rho)     -> list[DubinsPath]
path_length(start, goal, rho)   -> float
```

## Veri Akışı

`shortest_path`:
1. Başlangıcı orijine taşı, `theta = atan2(dy, dx)` kadar döndür, `rho`'ya böl.
   Sonuç: `d = D / rho`, `alpha = mod2pi(start_yaw - theta)`,
   `beta = mod2pi(goal_yaw - theta)`.
2. Altı kanonik çözücüyü çağır.
3. `None` olmayanlar arasından `t + p + q` en küçük olanı seç.
4. `rho` ile ölçekleyip `DubinsPath` olarak paketle.

Örnekleme kanonik çerçeveye geri dönmeden, doğrudan dünya çerçevesinde kapalı
formla yapılır (sayısal hata birikmesin diye):

- Sol dönüş:  `yaw' = yaw + s/rho`, `x' = x + rho*(sin(yaw') - sin(yaw))`,
              `y' = y - rho*(cos(yaw') - cos(yaw))`
- Sağ dönüş:  `yaw' = yaw - s/rho`, `x' = x + rho*(sin(yaw) - sin(yaw'))`,
              `y' = y + rho*(cos(yaw') - cos(yaw))`
- Düz:        `x' = x + s*cos(yaw)`, `y' = y + s*sin(yaw)`, `yaw' = yaw`

## Hata ve Kenar Durumlar

- `rho <= 0` → `ValueError`
- Karekök altı `-1e-12` ile 0 arasındaysa 0'a kırpılır; daha negatifse kelime
  geçersiz (`None`)
- Tüm açı normalizasyonu tek bir `_mod2pi` yardımcısından geçer
- Başlangıç ve hedef poz birebir aynı → sıfır uzunluklu yol
- Konum aynı, heading farklı → `d = 0` ile CCC kelimeleri geçerli kalır
- Hiçbir kelime geçerli değilse → `RuntimeError` (sessiz `None` değil)

## Test Stratejisi

TDD: her parça için önce test yazılır.

1. **Uç nokta yeniden inşası (kritik).** Rastgele ve ızgara poz çiftleri için
   `interpolate(length)` hedef poza 1e-6 toleransla oturmalı. Altı formülün
   herhangi birindeki işaret hatasını yakalar.
2. **Optimallik.** `shortest_path(...).length == min(p.length for p in all_paths(...))`
3. **Alt sınır.** Uzunluk >= başlangıç ile hedef arası Öklid mesafesi.
4. **Düz hat özel durumu.** Aynı heading'e sahip, o heading doğrultusunda duran
   iki poz → kelime S içerir, uzunluk mesafeye eşittir.
5. **Eğrilik kısıtı.** Örneklenmiş yolda birim yay başına heading değişimi
   `1/rho`'yu (tolerans dahilinde) aşmaz.
6. **Ölçek değişmezliği.** Konumlar ve `rho` k ile çarpılırsa uzunluk k katı olur.
7. **Girdi doğrulama.** `rho <= 0` → `ValueError`.
8. **Örnekleme.** `sample(step)` ardışık aralıkları <= step; ilk poz start, son
   poz hedeftir.

## Bağımlılıklar

- `src/dubins.py`: sadece `math` ve `dataclasses`. RRT* bu fonksiyonu on binlerce
  kez çağıracak; numpy overhead'i orada zarar verir.
- `requirements.txt`: `numpy`, `matplotlib`, `pytest` — testler ve çizim için.

## Ekler

`notebooks/dubins_demo.py`: bir poz çifti için altı kelimeyi de çizer,
`results/` altına kaydeder. Rapor için doğrudan kullanılabilir şekil.

## RRT* Entegrasyon Yüzeyi (sonraki tur için not)

- Maliyet / steering metriği: `path_length(start, goal, rho)`
- Çarpışma kontrolü için nokta listesi: `shortest_path(...).sample(step)`

## Referans

Shkel, A. M., & Lumelsky, V. (2001). Classification of the Dubins set.
*Robotics and Autonomous Systems*, 34(4), 179-202.

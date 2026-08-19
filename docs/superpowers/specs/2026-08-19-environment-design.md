# Environment Modülü — Tasarım

Tarih: 2026-08-19
Durum: Onaylandı
Kapsam: `src/environment.py`, `tests/test_environment.py`, `notebooks/environment_demo.py`

## Amaç

Planlama ortamını temsil etmek: dairesel engeller, harita sınırları ve emniyet
payı. "Bu nokta serbest mi", "bu yol serbest mi" sorularına cevap verir ve
RRT* için serbest uzaydan rastgele poz üretir.

## Kapsam Dışı

- RRT* algoritmasının kendisi. Ayrı bir tasarım turu.
- Poligon ve ızgara (occupancy grid) engeller. Bu sürüm yalnızca daire.
- 3D / irtifa. Dubins gibi bu da sabit irtifada çalışır.
- Analitik kesişim hesabı. Çarpışma kontrolü örnekleme tabanlıdır.

## Temel Tasarım Kararı: Environment, Dubins'i Tanımaz

`src/environment.py` **`src/dubins.py`'yi import etmez.** Yol çarpışma kontrolü
bir `DubinsPath` değil, örneklenmiş nokta listesi alır:

```python
env.is_path_free(path.sample(step))    # RRT* boyle cagirir
```

Gerekçe:

- İki modül birbirinden bağımsız test edilebilir.
- Adım boyu seçimi çağıranın (RRT*) sorumluluğunda kalır; Environment sadece
  geometri bilir.
- İleride Dubins yerine başka bir yol tipi denenirse Environment değişmez.

Katman ilişkisi:

```
rrt_star.py          <- engelleri o dert edinir
   |  path_length()  -> maliyet          (dubins)
   |  sample()       -> nokta listesi    (dubins)
   |  is_path_free() -> serbest mi       (environment)
   v
dubins.py   ve   environment.py   birbirini tanimaz
```

## Koordinat Sözleşmesi

Dubins ile aynı: ENU çerçevesi, x doğu, y kuzey, metre. Açı gerekmediği için
Environment yalnızca `(x, y)` ile ilgilenir; poz verilirse ilk iki bileşenini
kullanır.

`environment.py` kendi `Pose = tuple[float, float, float]` takma adını tanımlar,
`dubins.py`'den import etmez. İki satırlık bir tekrar, ama modül bağımsızlığını
korumak buna değer. Aynı sözleşmeyi ifade ettikleri her iki docstring'de yazılır.

## Mimari

### `Obstacle` (frozen dataclass)

Alanlar: `x: float`, `y: float`, `radius: float`

`DubinsPath` ile aynı üslup: değiştirilemez, küçük, tek sorumluluklu.

### `Environment` (frozen dataclass)

Alanlar:
- `bounds: tuple[float, float, float, float]` — `(xmin, ymin, xmax, ymax)`
- `obstacles: tuple[Obstacle, ...]`
- `clearance: float` — her engel yarıçapına eklenen emniyet payı, metre

Emniyet payı GPS hatası, rüzgâr sürüklemesi ve kanat açıklığı için gerçek bir
ihtiyaç. Tek sayı olarak tutulur; engel yarıçaplarını elle şişirmek yerine
merkezî ve raporda savunulabilir.

Metotlar:

| metot | döner | ne yapar |
|---|---|---|
| `is_inside_bounds(point)` | `bool` | nokta harita sınırları içinde mi |
| `is_free(point)` | `bool` | sınırlar içinde **ve** hiçbir şişirilmiş engelin içinde değil |
| `is_path_free(points)` | `bool` | verilen noktaların hepsi serbest mi |
| `random_free_pose(rng)` | `Pose` | serbest uzaydan rastgele poz (reddetme yöntemi) |
| `suggested_step()` | `float` | önerilen örnekleme adımı |

`random_free_pose` bir `random.Random` örneği alır — tohumlanabilir olması
testlerin tekrarlanabilir olması için şart. Yaw `[0, 2*pi)` aralığından düzgün
dağılımla seçilir.

## Örneklemenin Bilinen Sınırı

Örnekleme tabanlı çarpışma kontrolü **kesin değildir**. İki örnek noktası
arasında yol bir engelin kenarından teğet geçebilir ve hiçbir nokta engelin
içine düşmeyebilir ("tünelleme").

İki savunma:

1. **`clearance` payı.** Engel şişirildiği için teğet geçişler gerçek engele
   değil tampon bölgeye denk gelir. Emniyet payının varlık sebeplerinden biri
   tam olarak budur.

2. **`suggested_step()`.** En küçük şişirilmiş engel yarıçapının yarısını
   döndürür. Engel yoksa harita kenarının onda birini döndürür. Adım bundan
   küçük tutulursa, bir engelin içinden geçen yol en az bir örnek nokta
   bırakmadan çıkamaz.

Bu sınır docstring'de açıkça yazılır. Yaklaşık olduğu bilgisi saklanacak bir
şey değil, gerekçesiyle birlikte savunulan bir tasarım tercihidir.

## Hata ve Kenar Durumlar

- `Obstacle.radius <= 0` → `ValueError`
- `clearance < 0` → `ValueError`
- `xmin >= xmax` veya `ymin >= ymax` → `ValueError`
- `random_free_pose` belirli sayıda denemede (varsayılan 1000) serbest poz
  bulamazsa → `RuntimeError`. Harita tamamen doluysa sonsuz döngüye girmesin.
- Boş nokta listesi `is_path_free`'ye verilirse → `True` (boş küme, kontrol
  edilecek bir şey yok). Docstring'de belirtilir.
- **Sınır üzerinde olmak çarpışma sayılır**: mesafe `<= radius + clearance`
  ise nokta serbest değildir. Muhafazakâr taraf doğru taraftır.
- Engel listesi boş olabilir; o durumda yalnızca sınır kontrolü yapılır.

## Test Stratejisi

TDD: her parça için önce test yazılır.

1. **Obstacle doğrulama.** `radius <= 0` → `ValueError`.
2. **Environment doğrulama.** Negatif `clearance`, ters `bounds` → `ValueError`.
3. **Sınır kontrolü.** İçeride, dışarıda, tam köşede ve tam kenarda noktalar.
4. **Nokta-engel.** Merkezde, içeride, dışarıda ve tam çember üzerinde noktalar;
   çember üzeri çarpışma sayılmalı.
5. **Clearance etkisi.** `clearance` artınca önce serbest olan nokta engelli
   hâle gelmeli.
6. **Engelsiz harita.** Sınır içindeki her nokta serbest.
7. **Çok engel.** Bir noktanın herhangi bir engele girmesi yeterli.
8. **Yol kontrolü.** Tamamen serbest yol `True`; tek noktası engele giren yol
   `False`; boş liste `True`.
9. **`random_free_pose`.** Aynı tohumla aynı sonuç; dönen poz daima serbest ve
   sınırlar içinde; yaw `[0, 2*pi)` aralığında; harita doluysa `RuntimeError`.
10. **`suggested_step`.** Engel varken en küçük şişirilmiş yarıçapın yarısı;
    engel yokken pozitif bir değer.

## Bağımlılıklar

`src/environment.py`: sadece `math`, `dataclasses`, `random` (tip için).
`dubins.py`'yi **import etmez**. numpy kullanılmaz — RRT* bu fonksiyonları on
binlerce kez çağıracak.

## Ekler

`notebooks/environment_demo.py`: haritayı engellerle çizer, üstüne birkaç
Dubins yolu koyar, serbest olanları yeşil çarpanları kırmızı gösterir.
`results/environment_demo.png` olarak kaydedilir. Çarpışma kontrolünün
çalıştığını gözle doğrulamak ve rapora şekil üretmek için.

## RRT* Entegrasyon Yüzeyi (sonraki tur için not)

- Örnekleme: `env.random_free_pose(rng)`
- Kenar geçerliliği: `env.is_path_free(path.sample(step))`
- Adım seçimi: `env.suggested_step()` başlangıç değeri olarak

Benchmark'ta ölçüldüğü üzere `sample()` çağrısı Dubins çözümünden ~280 kat
pahalı (0.1 m adımda 2122 us). RRT*'ın asıl darboğazı burası olacak; adım
boyunun gereksiz küçük seçilmemesi önemli.

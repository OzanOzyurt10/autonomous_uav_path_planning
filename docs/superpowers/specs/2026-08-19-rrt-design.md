# RRT Rota Planlayıcı — Tasarım (1. Aşama)

Tarih: 2026-08-19
Durum: Onaylandı
Kapsam: `src/rrt_star.py`, `tests/test_rrt_star.py`, `notebooks/rrt_demo.py`

## Amaç

Dubins ve Environment modüllerini birleştirip engelli bir ortamda başlangıç
pozundan hedef poza çarpışmasız bir rota üretmek.

Şimdiye kadar elimizde iki ayrı parça vardı: Dubins iki poz arasında en kısa
yolu buluyor ama engelleri bilmiyor; Environment bir yolun çarpıp çarpmadığını
söylüyor ama alternatif üretmiyor. Bu modül eksik olan üçüncü parça — engelin
etrafından dolaşacak ara pozları bulan katman.

## Kapsam Dışı — 2. Aşamaya Bırakılanlar

- **Yeniden bağlama (rewire) ve komşuluk yarıçapı.** RRT'yi RRT*'a çeviren ve
  asimptotik optimallik veren adımlar. Ayrı bir tasarım turu.
- Adımlı ilerleme (steering / `max_edge_length` ile kırpma).
- Çift yönlü (bidirectional) arama.
- 3D / irtifa.

Dosya adı şimdiden `rrt_star.py`; 2. aşamada aynı dosyaya rewire eklenecek,
yeniden adlandırma gerekmeyecek.

## Ağacı Büyütme Yaklaşımı: Doğrudan Bağlama

En yakın düğümden örneklenen poza **tam bir Dubins yolu** kurulur; çarpışma
kontrolünden geçerse ağaca eklenir. Her kenar eksiksiz bir Dubins yoludur.

Değerlendirilen alternatif: adımlı ilerleme (örneğe doğru yalnızca belirli bir
mesafe git). Dar geçitlerde daha iyi yayılır, ama kesilmiş bir Dubins yolu artık
"en kısa Dubins yolu" değil onun bir ön ekidir; `DubinsPath` üç segmentli sabit
bir yapı olduğu için kenarı saklamak ayrı bir kırpma fonksiyonu ve onun testleri
demektir. Kazanç bu maliyeti karşılamıyor.

Bedeli kabul ediliyor: uzak örneklere giden uzun yollar daha çok reddedilir,
yani dar alanlarda daha çok iterasyon gerekir. Hedef biasing ve yeterli
`max_iterations` bunu telafi eder.

## Dubins Mesafesinin Asimetrisi

`path_length(A, B, rho) != path_length(B, A, rho)` — heading'ler ters çevrilince
problem başka bir problem olur.

Bu aşamada sorun çıkarmaz: ağaç başlangıçtan büyüdüğü için mesafe **daima gidiş
yönünde** hesaplanır, yani `path_length(agactaki_dugum, ornek)`. Yön hiçbir
yerde ters çevrilmez.

Asimetri asıl 2. aşamadaki yeniden bağlama adımında tehlikeli olur; orada
"yeni düğümden komşuya" ile "komşudan yeni düğüme" farklı sayılardır ve
karıştırılırsa sessizce yanlış ağaç kurulur. 2. aşama tasarımının ilk konusu bu.

## Koordinat Sözleşmesi

Dubins ve Environment ile aynı: ENU, x doğu, y kuzey, metre; yaw radyan, CCW
pozitif. `Pose = tuple[float, float, float]` bu modülde de yeniden tanımlanır.

## Mimari

### `Node` (frozen dataclass)

| alan | tip | anlamı |
|---|---|---|
| `pose` | `Pose` | bu düğümün pozu |
| `parent` | `int \| None` | ebeveynin listedeki indeksi; kök için `None` |
| `cost` | `float` | başlangıçtan buraya toplam mesafe, metre |
| `path_from_parent` | `DubinsPath \| None` | ebeveynden buraya gelen kenar; kök için `None` |

Ağaç bir **liste**, ebeveyn ilişkisi **indeksle** tutulur. İç içe nesne referansı
yerine indeks: ağacı yazdırıp incelemek kolay, döngüsel referans riski yok,
2. aşamada yeniden bağlama sadece indeks güncellemesi olacak.

### `RRTResult` (frozen dataclass)

| alan | tip | anlamı |
|---|---|---|
| `found` | `bool` | rota bulundu mu |
| `edges` | `list[DubinsPath]` | başlangıçtan hedefe sıralı Dubins parçaları |
| `cost` | `float` | toplam rota uzunluğu, metre |
| `iterations` | `int` | harcanan iterasyon sayısı |
| `tree` | `list[Node]` | tüm ağaç; demo çizimi ve inceleme için |

Rota bulunamaması **hata değildir** — rastgele bir planlayıcı için meşru sonuç.
İstisna fırlatılmaz, `found=False` döner ve `edges` boş kalır.

### Genel API

```python
def plan(start: Pose, goal: Pose, env: Environment, rho: float,
         max_iterations: int = 5000,
         goal_bias: float = 0.05,
         step: float | None = None,
         rng: random.Random | None = None,
         stop_on_first_solution: bool = True) -> RRTResult
```

- `step=None` → `env.suggested_step()` kullanılır.
- `rng=None` → yeni bir `random.Random()`. Testler kendi tohumlu üretecini geçirir.
- `stop_on_first_solution=True` (varsayılan): ilk rota bulunduğunda döngü biter.
  `False` verilirse döngü `max_iterations`'a kadar sürer ama düz RRT rotayı
  iyileştirmediği için **ilk bulunan rota** döndürülür; tek farkı `tree`'nin
  daha büyük olması. Parametre 2. aşamada anlam kazanacak, şimdiden imzada
  duruyor ki API değişmesin.

### Yardımcılar (private, ayrı test edilir)

| fonksiyon | imza | ne yapar |
|---|---|---|
| `_sample` | `(env, goal, rng, goal_bias) -> Pose` | `goal_bias` olasılıkla hedefi, yoksa `env.random_free_pose(rng)` |
| `_nearest` | `(nodes, target, rho) -> int` | `path_length(nodes[i].pose, target, rho)` en küçük olan indeks |
| `_try_connect` | `(env, from_pose, to_pose, rho, step) -> DubinsPath \| None` | Dubins kur, çarpışma kontrol et, temizse yolu döndür |
| `_extract_path` | `(nodes, index, goal_edge) -> list[DubinsPath]` | ebeveyn zincirini geri takip edip kenarları sıraya koy |

`_try_connect` modülün kalbi: `shortest_path(...)` ile yolu kur,
`env.is_path_free(path.sample(step))` ile sına, temizse döndür, değilse `None`.

## Bir İterasyonun Akışı

```
1. ornek = _sample(env, goal, rng, goal_bias)
2. i     = _nearest(nodes, ornek, rho)
3. yol   = _try_connect(env, nodes[i].pose, ornek, rho, step)
4. yol None ise -> bu iterasyonu atla
5. Node(pose=ornek, parent=i, cost=nodes[i].cost + yol.length,
        path_from_parent=yol) ekle
6. hedef_yolu = _try_connect(env, ornek, goal, rho, step)
7. hedef_yolu varsa -> cozum bulundu, _extract_path ile geri takip et
```

### Hedef Biasing

Adım 1'de `goal_bias` olasılıkla rastgele poz yerine doğrudan hedef poz
örneklenir. Ağacın hedefe doğru çekilmesini sağlar; olmadan RRT serbest alanda
amaçsızca yayılır ve yakınsama çok yavaşlar. Varsayılan 0.05.

### Hedefe Varma

Her yeni düğüm eklendikten sonra oradan **tam hedef poza** Dubins bağlantısı
denenir. Başarılıysa rota bulunmuş sayılır. İHA hedefe hem doğru noktada hem
doğru heading'le varır — Dubins kullanmanın asıl sebebi buydu.

Ek maliyet düğüm başına tek bir `shortest_path` + bir `is_path_free` çağrısı.

## Hata ve Kenar Durumları

- `rho <= 0` → `ValueError`
- `max_iterations <= 0` → `ValueError`
- `goal_bias` `[0.0, 1.0]` aralığı dışında → `ValueError`
- `step` verilmiş ve `<= 0` → `ValueError`
- **`start` veya `goal` serbest değilse → `ValueError`.** Sessizce 5000
  iterasyon dönüp "bulamadım" demektense girdinin bozuk olduğunu söylemek
  gerekir; sorun algoritmada değil.
- `start == goal` → ilk iterasyonda doğrudan bağlanır, tek kenarlı rota.
- Rota bulunamazsa → `found=False`, `edges=[]`, `cost=math.inf`, `tree` dolu.
  `tree`'nin dolu dönmesi hata ayıklamayı mümkün kılıyor: ağaç nereye kadar
  yayılmış, nerede tıkanmış görülebilir.
- `env.random_free_pose` `RuntimeError` fırlatırsa yukarı yayılır — harita
  doluysa planlama zaten anlamsız.

## Test Stratejisi

TDD: her parça için önce test yazılır.

1. **Yardımcılar ayrı ayrı.** `_sample` `goal_bias=1.0` ile daima hedefi verir,
   `0.0` ile daima serbest poz verir. `_nearest` bilinen küçük bir ağaçta doğru
   indeksi verir ve mesafeyi **gidiş yönünde** hesaplar (asimetrik bir örnekle
   sınanır). `_try_connect` temiz yolda `DubinsPath`, engelli yolda `None`
   döndürür. `_extract_path` zinciri doğru sırada verir.

2. **Uçtan uca çarpışma doğrulaması (kritik).** Dönen rotanın her kenarı
   `0.05 m` adımla yeniden örneklenir; hiçbir nokta engelde olmamalı.
   Planlayıcı `suggested_step` ile karar veriyor, test daha sıkı bakıyor.
   Environment'ta ölçülen teğet-geçiş payı burada da görünür olsun.

3. **Zincir bütünlüğü.** Kenar `i`'nin `end_pose()`'u, kenar `i+1`'in
   `start`'ına eşit (1e-6 tolerans). İlk kenar `start`'tan başlar, son kenar
   `goal`'da biter.

4. **Maliyet tutarlılığı.** `result.cost` kenar uzunlukları toplamına eşit.

5. **Boş harita.** Engelsiz ortamda doğrudan bağlanabilen hedef → `found=True`,
   tek kenar, uzunluk `path_length(start, goal, rho)`'ya eşit.

6. **Engel arkasındaki hedef.** Doğrudan Dubins yolu engelden geçen bir
   senaryoda rota bulunur ve çarpışmasızdır — modülün varlık sebebi bu.

7. **Tekrarlanabilirlik.** Aynı tohumlu `rng` ile aynı sonuç.

8. **Çözülemez durum.** Hedef engellerle çevrili → `found=False`, çökme yok,
   `iterations == max_iterations`, `tree` dolu.

9. **Girdi doğrulama.** Serbest olmayan `start`/`goal`, geçersiz `rho`,
   `max_iterations`, `goal_bias`, `step` → `ValueError`.

10. **Eğrilik.** Rota boyunca birim yay başına heading değişimi `1/rho`'yu
    aşmaz — Dubins'ten miras ama uçtan uca bir kez daha sınanır.

## Bağımlılıklar

`src/rrt_star.py`: `math`, `random`, `dataclasses`, ve proje içinden
`src.dubins` + `src.environment`. numpy kullanılmaz.

Bu modül iki alt modülü de import eden **tek** yer — mimarinin birleştiği nokta.

## Ekler

`notebooks/rrt_demo.py`: haritayı engellerle çizer, **ağacın tüm kenarlarını**
ince gri çizgilerle gösterir, bulunan rotayı kalın renkle üstüne basar.
Başlıkta iterasyon sayısı, düğüm sayısı ve rota uzunluğu.
`results/rrt_demo.png` olarak kaydedilir.

Ağacı görmek önemli: RRT'nin nasıl yayıldığı, hedef biasing'in etkisi ve
engellerin ağacı nasıl şekillendirdiği gözle görülür. 2. aşamada aynı şekil yan
yana konup rewire'ın farkı gösterilebilir.

## 2. Aşama İçin Notlar

- Yeniden bağlama: yeni düğüm eklendiğinde yakın komşulara bakılıp daha ucuz
  bir ebeveyn bulunur, ve yeni düğüm üzerinden daha ucuz olan komşuların
  ebeveyni güncellenir. **Asimetri burada kritik.**
- Komşuluk yarıçapı: sabit mi, yoksa teorik `gamma * (log n / n)^(1/d)` mi.
- `stop_on_first_solution=False` ile bütçe boyunca iyileşme ve maliyet-iterasyon
  grafiği — rapor için güçlü bir şekil.

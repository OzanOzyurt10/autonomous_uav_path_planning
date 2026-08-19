# RRT* — 2. Aşama Tasarımı

**Tarih:** 2026-08-19
**Dosya:** `src/rrt_star.py` (mevcut dosyaya eklenir)
**Önceki aşama:** `docs/superpowers/specs/2026-08-19-rrt-design.md`

## Amaç

Düz RRT'yi RRT*'a çevirmek: yeni düğüme en ucuz ebeveyni seçmek ve yeni düğüm
üzerinden komşuları yeniden bağlamak. Sonuç, bütçe boyunca kısalmaya devam eden
bir rota.

## Kapsam ve Sıralama

İki tura bölünüyor, her turun sonunda çalışan ve test edilmiş kod var:

**Tur A — ebeveyn seçimi.** `_neighbours` ve `_choose_parent` eklenir. Rota
kısalır, maliyet yayılımı hiç gerekmez (hiçbir mevcut düğümün maliyeti
değişmiyor, yalnızca yeni düğümünki belirleniyor).

**Tur B — yeniden bağlama.** `_rewire` ve `_propagate_cost` eklenir. Asıl RRT*
burada tamamlanır.

Kapsam dışı: adımlı ilerleme, çift yönlü arama, 3D, rüzgâr.

## Komşuluk Yarıçapı

```
r_n = min(gamma * (log n / n)^(1/3), eta)
```

`n` ağaçtaki düğüm sayısı. Üs `1/3`, konfigürasyon uzayı SE(2) yani üç boyutlu
olduğu için (x, y, yaw).

`gamma` ve `eta` `plan`'a `radius_gamma` ve `radius_cap` adlarıyla parametre
olarak geçer. Varsayılanlar demo haritası (100 × 60 m, rho = 4 m) ölçeğine göre
`gamma = 60.0`, `eta = 30.0`; bu değerlerle yarıçap 50 düğümde ~26 m, 200
düğümde ~18 m, 1000 düğümde ~11 m olur. Başka ölçekte bir haritada ayarlanmaları
gerekir — bu bir eksiklik değil, formülün doğası.

**Dürüstlük notu (raporda yer almalı):** Küçülen yarıçap formülü Öklid uzayı
için ispatlanmıştır. Dubins uzayı Öklid değil ve asimetriktir; literatürde
formül Dubins'e uygulanıyor, ancak asimptotik optimallik garantisi aynı
sağlamlıkta değildir.

## Öklid Ön Eleme

`_neighbours` adayları önce `math.hypot` ile süzer, yalnızca kalanlar için
Dubins çözülür.

Bu kayıpsızdır: bir Dubins yolu iki nokta arasındaki düz çizgiden kısa olamaz,
dolayısıyla Öklid mesafesi `r`'yi aşan bir düğümün Dubins mesafesi de kesinlikle
`r`'yi aşar. Eleme hiçbir gerçek komşuyu atmaz.

`_neighbours` yön bilmez — yalnızca aday indeksleri döner. Yönlü maliyetleri
`_choose_parent` ve `_rewire` kendi hesaplar.

## Asimetri

İki işlem iki farklı yön ölçer:

```
_choose_parent:  komsu.cost + d(komsu -> yeni)      # yeni dugume gelmek
_rewire:         yeni.cost  + d(yeni  -> komsu)     # yeni dugumden gitmek
```

`path_length(A, B, rho) != path_length(B, A, rho)`. Aynı iki poz arasındaki iki
kenar farklı eğrilerdir; biri diğerinin tersi değildir.

Yanlış yön yazılırsa kod çalışır, rota çıkar, mevcut testler geçer — sadece
sonuç optimal olmaz. Bu yüzden yönü doğrudan hedefleyen ayrı bir test var
(aşağıda).

## Döngü Oluşmaması

Ayrı bir döngü kontrolü gerekmiyor. Bir komşu `j` yeni düğümün atasıysa
`yeni.cost = j.cost + (aradaki yol)` olduğundan, kenar uzunlukları pozitif
olduğu için

```
yeni.cost + d(yeni -> j) > j.cost
```

her zaman doğrudur ve iyileştirme koşulu tutmaz. Karşılaştırma `<` ile yapılır;
`<=` kullanılırsa sıfır uzunluklu kenarlarda eşitlik döngü kurabilir.

## Maliyet Yayılımı

`Node` frozen kalır. Yeniden bağlarken düğümün yerine `dataclasses.replace(...)`
ile yenisi konur. Ardından `_propagate_cost` listeyi tarayarak çocukları bulur
ve kuyrukla alt ağacı gezer:

```
cocuk.cost = ebeveyn.cost + cocuk.path_from_parent.length
```

Kenarların **geometrisi değişmez** — ebeveynin pozu aynı kaldı, yalnızca ona
kadarki toplam maliyet değişti. Yeniden Dubins çözülmez.

Bedeli: yeniden bağlama başına O(n) tarama. Birkaç yüz düğümde sorun değil.
Alternatif olan "her düğümde çocuk listesi tutmak" daha hızlı ama frozen yapıyı
bozar ve aynı bilgi iki yerde tutulduğu için tutarsızlaşabilir.

## Hedef Bağlantısı

Şu anki kod ilk çözümü `best` değişkenine bir kez yazıp donduruyor. Yeniden
bağlama maliyetleri değiştirdiği için bu değer bayatlar.

Yerine hedefe ulaşan tüm bağlantılar tutulur:

```python
goal_links: list[tuple[int, DubinsPath]]     # (dugum indeksi, hedefe giden kenar)
```

Sonuç, döngü bitince bu listedeki en ucuz seçilerek üretilir:
`nodes[i].cost + goal_edge.length` en küçük olan. Böylece yeniden bağlama eski
bir bağlantıyı ucuzlattığında kazanç otomatik olarak yansır.

## API Değişikliği

```python
plan(start, goal, env, rho,
     max_iterations=5000, goal_bias=0.05, step=None, rng=None,
     stop_on_first_solution=False,      # varsayilan True'dan False'a doner
     radius_gamma=60.0, radius_cap=30.0)   # yeni
```

`stop_on_first_solution` varsayılanı `False` olur: `plan` bütçeyi harcar ve
bulduğu en iyi rotayı döner. RRT*'ın faydası bu. İlk çözümde durmak isteyen
açıkça `True` verir.

`RRTResult.iterations` artık **çalıştırılan yineleme sayısı** anlamına gelir.
Erken dönüldüyse durulan yineleme, bütçe harcandıysa `max_iterations`. Mevcut
`test_finds_route_in_first_iteration` bu semantiğe göre güncellenir.

## Testler

**1. Yapısal değişmezler.** En güçlü katman. Herhangi bir koşudan sonra ağacın
tamamı taranır:

- her düğüm için `node.cost == nodes[node.parent].cost + node.path_from_parent.length`
- her düğümden ebeveyn zinciri en fazla `n` adımda kökte biter
- kökün `parent`'ı `None`, `cost`'u `0.0`
- her düğümün `parent`'ı kendi indeksinden küçük olmak **zorunda değildir** —
  yeniden bağlama sonrası bu kırılır, mevcut `test_tree_parents_are_valid_indices`
  buna göre gevşetilir

Maliyet yayılımı hatası bu katmandan kaçamaz.

**2. Yön testi.** Elle kurulmuş bir ağaçta, `_choose_parent` yanlış yönü
kullansaydı farklı bir ebeveyn seçecek şekilde poz yerleşimi. Aynısı `_rewire`
için. Asimetriyi doğrudan hedefler.

**3. İyileşme.** İstatistiksel, birkaç tohumda:

- aynı tohum ve bütçede RRT* maliyeti düz RRT maliyetinden küçük veya eşit
- bütçe artınca maliyet artmaz (monoton)

**4. Çarpışmasızlık korunur.** Yeniden bağlamayla kurulan her kenar da
`_try_connect`'ten geçer; koşu sonunda rotanın tüm kenarları ince adımla
(0.05 m) yeniden sınanır.

## Demo

`notebooks/rrt_demo.py` güncellenir: aynı tohum ve bütçeyle düz RRT ve RRT*
yan yana iki panelde çizilir, başlıklarda rota uzunlukları karşılaştırılır.
Yeniden bağlamanın ağacı nasıl düzelttiği görülür.

"""Dubins yollariyla calisan RRT rota planlayici.

dubins.py ile environment.py'yi birlestiren tek yer burasi. dubins.py
engelleri, environment.py de ucak dinamigini bilmiyor; ikisini haberdar
etmek yerine planlayici ortada durup "su Dubins yolu su ortamda serbest
mi" sorusunu soruyor.

Bu asamada duz RRT var: agac buyur, hedefe baglanabilirsen dur. RRT*'in
yeniden baglama (rewire) adimi ikinci asamada yine bu dosyaya eklenecek,
cunku ayni agac yapisi uzerinde calisiyor.

Koordinat duzeni dubins.py ile ayni: poz (x, y, yaw), ENU, yaw radyan.
"""

import math
import random
from dataclasses import dataclass

from src.dubins import DubinsPath, path_length, shortest_path
from src.environment import Environment

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class Node:
    """Agactaki tek bir poz ve oraya nereden gelindigi.

    Agac, ic ice nesne referanslari yerine bir liste olarak tutuluyor;
    parent o listedeki indeks, kok dugum icin None. Indeks tutmanin uc
    faydasi var: agaci oldugu gibi yazdirip inceleyebiliyorsun, dongusel
    referans olusma riski yok, ve ikinci asamada bir dugumu baska bir
    ebeveyne baglamak tek bir indeksi degistirmek demek.

    cost koktan bu poza kadar metre cinsinden toplam yol; kok icin 0.0.
    path_from_parent ebeveynden buraya gelen Dubins yolu, kok icin None.
    """

    pose: Pose
    parent: int | None
    cost: float
    path_from_parent: DubinsPath | None


@dataclass(frozen=True)
class RRTResult:
    """Bir planlama kosusunun sonucu.

    Rota bulunamamasi istisna degil sonuc: RRT rastgele ornekleme yapiyor,
    verilen yineleme butcesinde hedefe ulasamamak mesru bir cikti. O yuzden
    found=False donuyor, hata firlatilmiyor.

    Basarisiz kosuda bile tree dolu doner; agacin nereye kadar yayilip
    nerede tikandigini gorup sorunu teshis edebilesin diye. edges bos liste,
    cost ise math.inf olur.

    edges koktan hedefe siralı Dubins yollari, cost bunlarin toplam uzunlugu,
    iterations ise donguyu kac kez dondugumuz.
    """

    found: bool
    edges: list[DubinsPath]
    cost: float
    iterations: int
    tree: list[Node]

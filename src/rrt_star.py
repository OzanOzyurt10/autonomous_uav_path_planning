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


def _try_connect(env: Environment, from_pose: Pose, to_pose: Pose,
                 rho: float, step: float) -> DubinsPath | None:
    """Iki poz arasinda gecerli bir kenar kurmaya calisir.

    En kisa Dubins yolunu olusturur, step araligiyla ornekler ve carpisma
    kontrolunden gecirir. Temizse yolu, degilse None doner.

    Kenar gecerliliginin tek karar noktasi burasi: agaca giren her kenar
    buradan geciyor, dolayisiyla carpisma kuralini degistirmek istersen
    bakilacak tek yer bu fonksiyon.

    Sinir kontrolu ayrica yazilmiyor; harita disina cikan bir yolun
    orneklenen noktalarini env.is_free zaten eliyor.
    """
    path = shortest_path(from_pose, to_pose, rho)
    if env.is_path_free(path.sample(step)):
        return path
    return None


def _nearest(nodes: list[Node], target: Pose, rho: float) -> int:
    """Hedefe en yakin dugumun indeksini doner (dugumun kendisini degil).

    Mesafe agactan hedefe yonunde olculur: path_length(dugum, target).
    Yon onemli cunku Dubins mesafesi simetrik degil; (0,0,0)->(0,3,pi/2)
    14.86 m iken tersi 11.66 m.

    Oklid mesafesi kullanilmiyor. Yakin ama ters yone bakan bir dugume
    baglanmak koca bir donus gerektirir; Dubins bunu hesaba katar, hypot
    katmaz.

    Esitlikte ilk gelen kazanir (karsilastirma < ile yapiliyor).

    Bu fonksiyon planlayicinin en sicak dongusu: her yinelemede agacin
    tamami taraniyor. O yuzden dugum basina yalnizca bir path_length
    cagrisi var, sonucu dist'te tutuluyor.
    """
    min_index = None
    min_dist = None
    for i in range(len(nodes)):
        dist = path_length(nodes[i].pose, target, rho)
        if min_index is None or dist < min_dist:
            min_index = i
            min_dist = dist
    return min_index

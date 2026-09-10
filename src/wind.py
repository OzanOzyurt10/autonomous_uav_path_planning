"""Ruzgar alani ve ruzgar ucgeni: iz boyunca yer hizi ve ucus suresi.

Ruzgar bir ALAN: at(x, y, z) o noktadaki vektoru veriyor. Tekduze alanda
rotanin geometrisi degismez - Zermelo (1931): duzgun akista en kisa sureli
yol yine duz cizgidir - ama irtifayla degisen alanda degisir, cunku artik
hangi irtifada uculdugu maliyeti etkiliyor.
"""

import bisect
import math

Pose3 = tuple[float, float, float, float]
Vector2 = tuple[float, float]

REF_HEIGHT = 10.0        # m AGL; meteorolojinin ruzgari bildirdigi yukseklik
SHEAR_ALPHA = 0.14       # acik arazi guc yasasi ussu
SHEAR_TOP = 500.0        # m AGL; ustunde profil sabit kabul ediliyor


def wind_vector(speed: float, from_deg: float) -> Vector2:
    """Meteorolojik ruzgar tarifini yerel cerceve vektorune cevirir.

    Iki tersleme ust uste biniyor: meteoroloji ruzgarin GELDIGI yonu
    soyler ama bize estigi yon lazim, ve kuzeyden saat yonunde olcer ama
    cerceve dogudan saat tersine. Birini atlamak sessizce 90 derece
    yanlis cevap verir; kuzeyden esen ruzgar (0, -s) vermeli.
    """
    if speed < 0.0:
        raise ValueError(f"ruzgar hizi negatif olamaz: {speed}")

    to_deg = (from_deg + 180.0) % 360.0
    theta = math.radians(90.0 - to_deg)
    return speed * math.cos(theta), speed * math.sin(theta)


class _ConstantField:
    """Her yerde ayni vektoru donen alan; as_field'in sarmalayicisi."""

    def __init__(self, vector: Vector2):
        self._vector = (float(vector[0]), float(vector[1]))

    def at(self, x: float, y: float, z: float) -> Vector2:
        return self._vector

    def strongest(self) -> float:
        return math.hypot(*self._vector)


class UniformWind(_ConstantField):
    """Alan sozlesmesinin tekduze hali: irtifadan bagimsiz tek vektor.

    Davranis eklemiyor; eski tek vektorlu kullanimin ShearWind ile yer
    degistirebilmesi icin var.
    """

    def __init__(self, speed: float, from_deg: float):
        super().__init__(wind_vector(speed, from_deg))


class ShearWind:
    """Guc yasasi: u(h) = u_ref * (h / h_ref) ** alpha, h YERDEN yukseklik.

    Iki sinir kritik. Tavansiz profil sinirsiz buyur ve planlayici arka
    ruzgarda her seferinde tavana tirmanir; taban olmadan da ruzgar yerde
    sifira duser ve karsi ruzgarda araziye yapismak bedava gorunur.
    Yon irtifayla donmuyor (Ekman sarmali): ikinci mertebeden ve aranan
    karari zaten hiz kaymasi uretiyor.
    """

    def __init__(self, speed_ref: float, from_deg: float, ground=None,
                 ref_height: float = REF_HEIGHT, alpha: float = SHEAR_ALPHA,
                 top: float = SHEAR_TOP):
        if speed_ref < 0.0:
            raise ValueError(f"ruzgar hizi negatif olamaz: {speed_ref}")
        if ref_height <= 0.0:
            raise ValueError(f"referans yukseklik pozitif olmali: {ref_height}")
        if alpha < 0.0:
            raise ValueError(f"kayma ussu negatif olamaz: {alpha}")
        if top < ref_height:
            raise ValueError(
                f"kayma tavani referans yuksekligin altinda: {top} < "
                f"{ref_height}")

        # Yon bir kere: her cagrida trigonometri cagirmak gereksiz ve
        # yavas, kenar basina yuzlerce ornek noktasi var.
        self._unit = wind_vector(1.0, from_deg)
        self._speed_ref = speed_ref
        self._ground = ground
        self._ref_height = ref_height
        self._alpha = alpha
        self._top = top

    def _speed_at_height(self, height: float) -> float:
        # Kirpma TEK adimda ve alt sinir ref_height: negatif yukseklik
        # (arazi pozun ustunde) kesirli kuvvette karmasik sayi dondururdu
        # ve hata bile vermezdi.
        height = min(max(height, self._ref_height), self._top)
        return self._speed_ref * (height / self._ref_height) ** self._alpha

    def at(self, x: float, y: float, z: float) -> Vector2:
        height = z if self._ground is None else z - self._ground(x, y)
        speed = self._speed_at_height(height)
        return speed * self._unit[0], speed * self._unit[1]

    def strongest(self) -> float:
        """Alanin ulasabilecegi en yuksek hiz; maliyet alt sinirinin girdisi."""
        return self._speed_at_height(self._top)


class ProfileWind:
    """Olculmus seviyeler arasinda interpolasyon; gercek atmosfer profili.

    ShearWind bir FORMUL, bu bir OLCUM. Gercek profil guc yasasina uymuyor:
    hiz irtifayla tek yonlu artmiyor ve yon doniyor. Seviyeler MSL cinsinden
    (yukseklik, hiz, GELDIGI yon) uclusu; basinc seviyesi verisi oyle geliyor.
    """

    def __init__(self, levels):
        if not levels:
            raise ValueError("ruzgar profili en az bir seviye ister")

        self._heights = []
        self._vectors = []
        for height, speed, from_deg in sorted(levels, key=lambda x: x[0]):
            # Ayni yukseklikte iki seviye interpolasyonda sifira bolme
            # demek; ilkini tutup digerini atiyoruz.
            if self._heights and height == self._heights[-1]:
                continue
            self._heights.append(float(height))
            self._vectors.append(wind_vector(speed, from_deg))
        self._strongest = max(math.hypot(*v) for v in self._vectors)

    def at(self, x: float, y: float, z: float) -> Vector2:
        heights = self._heights
        if z <= heights[0]:
            return self._vectors[0]
        if z >= heights[-1]:
            return self._vectors[-1]

        index = bisect.bisect_right(heights, z)
        low, high = heights[index - 1], heights[index]
        first, second = self._vectors[index - 1], self._vectors[index]
        # VEKTOR interpole ediliyor, hiz ve yon ayri ayri degil: 350 ile 10
        # derecenin ortasi 0'dir ama sayilarin ortalamasi 180 verir ve
        # ruzgar sessizce ters yone doner.
        ratio = (z - low) / (high - low)
        return (first[0] + (second[0] - first[0]) * ratio,
                first[1] + (second[1] - first[1]) * ratio)

    def strongest(self) -> float:
        """Profilin her yerinde gecerli ust sinir; maliyet alt sinirinin girdisi.

        Iki vektorun dogrusal karisiminin buyuklugu ikisinin buyugunu
        asamaz, o yuzden seviyelerin maksimumu aradaki her irtifa icin de
        gecerli.
        """
        return self._strongest


def as_field(wind):
    """Tek vektor de alan da kabul edilsin diye sarmalar.

    Eski cagrilar (x, y) demeti veriyor, yeni olanlar alan nesnesi;
    ikisini de burada tek tipe indiriyoruz.
    """
    return wind if hasattr(wind, "at") else _ConstantField(wind)


def ground_speed(airspeed: float, course: float, wind: Vector2,
                 climb: float = 0.0) -> float:
    """Iz boyunca yatay yer hizi, m/s.

    Ucak planlanan izi tutmak zorunda - araziden ve bolgelerden kacinan
    iz o - bu yuzden burnunu ruzgara kirip suruklenmeyi iptal ediyor.
    Sezgiye aykiri sonuc: tam yandan ruzgarda yer hizi degismez DEGIL,
    azalir; hava hizinin bir kismi sapmayi kapatmaya gidiyor.
    """
    if airspeed <= 0.0:
        raise ValueError(f"hava hizi pozitif olmali: {airspeed}")

    horizontal = airspeed * math.cos(climb)
    wind_x, wind_y = wind
    # Ruzgari iz yonunde ve ize dik bilesenlerine ayirmak: iz birim
    # vektoru (cos, sin), ona dik olan (-sin, cos). Dik olanin basindaki
    # eksi sart - onsuz ayrisma dondurme degil aynalama olur ve ruzgarin
    # buyuklugu korunmaz.
    along = wind_x * math.cos(course) + wind_y * math.sin(course)
    cross = -wind_x * math.sin(course) + wind_y * math.cos(course)

    if abs(cross) >= horizontal:
        raise ValueError(
            f"yan ruzgar {abs(cross):.1f} m/s, yatay hava hizi "
            f"{horizontal:.1f} m/s: bu iz tutulamaz")

    speed = math.sqrt(horizontal ** 2 - cross ** 2) + along
    if speed <= 0.0:
        raise ValueError(
            f"karsi ruzgar hava hizini asiyor: yer hizi {speed:.1f} m/s")
    return speed


def _segments(poses):
    """(yatay uzunluk, iz yonu, tirmanma acisi, orta nokta) uretir.

    Iz yonu yaw'dan DEGIL konumdan cikiyor: ruzgarda ucagin burnu ile
    gittigi yon ayrisiyor ve bize gittigi yon lazim. Orta nokta alanin
    orneklendigi yer. Sifir uzunluklu parcalar atlaniyor - bacak
    sinirlarinda ayni poz iki kez geliyor ve orada atan2(0, 0) ile
    sifira bolme cikardi.
    """
    for first, second in zip(poses, poses[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        dz = second[2] - first[2]
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue
        middle = ((first[0] + second[0]) / 2.0,
                  (first[1] + second[1]) / 2.0,
                  (first[2] + second[2]) / 2.0)
        yield length, math.atan2(dy, dx), math.atan2(dz, length), middle


def ground_speeds(poses, airspeed: float, wind) -> list[float]:
    """Her parcanin yer hizi; arayuzde en yavas kesimi gostermek icin."""
    field = as_field(wind)
    return [ground_speed(airspeed, course, field.at(*middle), climb)
            for _, course, climb, middle in _segments(poses)]


def flight_time(poses, airspeed: float, wind) -> float:
    """Rotanin ruzgar altinda surdugu toplam sure, saniye.

    Yatay mesafe yatay yer hizina bolunuyor. Uc boyutlu mesafeyi yatay
    hiza bolmek sureyi 1/cos(tirmanma) kadar sisirirdi; sakin havada
    dogru sonuc uc boyutlu uzunlugun hava hizina bolumu.
    """
    field = as_field(wind)
    return sum(length / ground_speed(airspeed, course, field.at(*middle),
                                     climb)
        for length, course, climb, middle in _segments(poses))

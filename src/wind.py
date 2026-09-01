"""Ruzgar ucgeni: planlanan izi tutmak icin gereken yer hizi ve sure.

Rotanin GEOMETRISI bu modulden etkilenmiyor. Planlayici izi ruzgarsiz
uretiyor, burada o iz veri kabul edilip ucagin onu tutarken yerde ne
kadar hizli ilerledigi hesaplaniyor. Ruzgari maliyete katmak - yani
planlayicinin ruzgar altina dolasmayi tercih etmesi - ayri bir is.
"""

import math

Pose3 = tuple[float, float, float, float]
Vector2 = tuple[float, float]


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
    """Ardisik pozlardan (yatay uzunluk, iz yonu, tirmanma acisi) uretir.

    Iz yonu yaw'dan DEGIL konumdan cikiyor: ruzgarda ucagin burnu ile
    gittigi yon ayrisiyor ve bize gittigi yon lazim. Sifir uzunluklu
    parcalar atlaniyor - bacak sinirlarinda ayni poz iki kez geliyor ve
    orada atan2(0, 0) ile sifira bolme cikardi.
    """
    for first, second in zip(poses, poses[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        dz = second[2] - first[2]
        length = math.hypot(dx, dy)
        if length == 0.0:
            continue
        yield length, math.atan2(dy, dx), math.atan2(dz, length)


def ground_speeds(poses, airspeed: float, wind: Vector2) -> list[float]:
    """Her parcanin yer hizi; arayuzde en yavas kesimi gostermek icin."""
    return [ground_speed(airspeed, course, wind, climb)
            for _, course, climb in _segments(poses)]


def flight_time(poses, airspeed: float, wind: Vector2) -> float:
    """Rotanin ruzgar altinda surdugu toplam sure, saniye.

    Yatay mesafe yatay yer hizina bolunuyor. Uc boyutlu mesafeyi yatay
    hiza bolmek sureyi 1/cos(tirmanma) kadar sisirirdi; sakin havada
    dogru sonuc uc boyutlu uzunlugun hava hizina bolumu.
    """
    return sum(length / ground_speed(airspeed, course, wind, climb)
        for length, course, climb in _segments(poses))

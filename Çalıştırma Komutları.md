projenin kökünde iken (CMD):

dubins_demo çalıştırmak için : .\venv\Scripts\python.exe -m notebooks.dubins_demo

dubins_benchmark çalıştırmak için : venv\Scripts\python.exe -m notebooks.dubins_benchmark

environment_demo çalıştırmak için : venv\Scripts\python.exe -m notebooks.environment_demo

rrt_demo çalıştırmak için : .\venv\Scripts\python.exe -m notebooks.rrt_demo




rrt_maze_demo çalıştırmak için : .\venv\Scripts\python.exe -m notebooks.rrt_maze_demo

dubins3d_demo çalıştırmak için : .\venv\Scripts\python.exe -m notebooks.dubins3d_demo

rrt3d_demo çalıştırmak için : .\venv\Scripts\python.exe -m notebooks.rrt3d_demo

   açık arazi haritasıyla : .\venv\Scripts\python.exe -m notebooks.rrt3d_demo acik


rrt3d_plotly çalıştırmak için (etkileşimli HTML) : .\venv\Scripts\python.exe -m notebooks.rrt3d_plotly

   açık arazi haritasıyla : .\venv\Scripts\python.exe -m notebooks.rrt3d_plotly acik

app_server çalıştırmak için : venv\Scripts\python.exe -m app.server

testleri çalıştırmak için : venv\Scripts\python.exe -m pytest -q


================================================================
GÖREV DOSYASINI ARDUPILOT'TA UÇURMAK
================================================================

Arayüzdeki "Görev dosyası" düğmesi bir .waypoints dosyası indiriyor
(QGC WPL 110 formatı). Yapısı:

  satır  akım  çerçeve  komut      açıklama
     0     1        0   WAYPOINT   home
     1     0        0   KALKIS     NAV_TAKEOFF
     2+    0        0   WAYPOINT   rota

EN KRİTİK NOKTA: irtifalar MUTLAK (deniz seviyesinden), home'a göre
değil. Çerçeve sütunu 0 = MAV_FRAME_GLOBAL. Bilerek seçildi; otopilotun
home kotu bizim arazi modelimizden farklı olsa bile görev kaymıyor.

Bunun SITL'de bir sonucu var: SITL'in zemini DÜZ ve HOME İRTİFASINDA.
Home rotanın en alçak noktasının üstünde kalırsa uçak yerin altına
inmeye çalışır. Bu yüzden home irtifası rotanın en düşük kotunun en az
50 m altına konuyor -- sitl_setup.py bunu dosyadan kendisi hesaplıyor,
elle bakman gerekmiyor.


--- HAZIRLIK (bir kez yapılır) ---

Betikler repoda tools/ altında ama WSL'den oraya erişmek sorunlu: repo
yolunda Türkçe karakter var ve WSL'e komut geçerken bozuluyor. Ev
dizinine kopyala.

  CMD'de (proje kökünde):
    copy tools\fly_mission.py  C:\Users\PC7668_BD26\
    copy tools\log_report.py   C:\Users\PC7668_BD26\
    copy tools\sitl_setup.py   C:\Users\PC7668_BD26\

  WSL'de:
    mkdir -p ~/tools
    cp /mnt/c/Users/PC7668_BD26/{fly_mission,log_report,sitl_setup}.py ~/tools/

Betikleri değiştirdiğinde bunu tekrarla.


--- HER KOŞU: DÖRT ADIM ---

Adım 1-2 her iki yolda da aynı. Adım 3'te seçim yapıyorsun.


1) GÖREV DOSYASINI AL

   Arayüzde "Görev dosyası" ile indir, sonra WSL'de:

     cp "$(ls -t /mnt/c/Users/PC7668_BD26/Downloads/gorev_*.waypoints \
         | head -1)" ~/yeni.waypoints

   DİKKAT: Windows'ta "copy dosya_*.waypoints hedef" birden fazla
   eşleşme bulursa dosyaları BİRLEŞTİRİYOR ve sonuç, geçerli görünen
   ama bozuk bir görev oluyor. Yukarıdaki komut hep en yenisini alır.
   Betikler de ikinci bir "QGC WPL" başlığı görürse duruyor.


2) KOMUTLARI ÜRET

     python3 ~/tools/sitl_setup.py ~/yeni.waypoints <rüzgâr_hız> \
         <rüzgâr_yön> <hava_hızı>

   Rüzgârsız uçacaksan:  0 0 28

   Hava hızı arayüzdeki "hız" alanına yazdığın sayı; betik onu dosyadan
   öğrenemez (QGC WPL biçiminde böyle bir alan yok). Vermezsen 28
   varsayar ve bunu ekrana yazar.

   Betik home irtifasını, ortalama kotu ve AIRSPEED_CRUISE'u hesaplayıp
   çalıştıracağın komutları hazır yazıyor. Aşağıdaki <...> yerlerine
   onun verdiği sayılar geliyor.


3a) YOL A -- SADECE SÜRE (ölçüm için)

   Ekranda uçak yok, sadece "ulaşıldı seq N" satırları akar. En hızlısı.

   Terminal 1 (açık kalacak):
     cd ~/ardupilot && python3 Tools/autotest/sim_vehicle.py -v ArduPlane \
         --no-mavproxy --no-rebuild --speedup 10 \
         -l <enlem>,<boylam>,<irtifa>,0

   Terminal 2:
     python3 ~/tools/fly_mission.py ~/yeni.waypoints \
         <rüzgâr_hız> <rüzgâr_yön> <AIRSPEED_CRUISE>


3b) YOL B -- HARİTALI (uçuşu izlemek için)

   Tek terminal, tek komut. Konsol ve harita açılır, uçağı canlı
   görürsün.

     cd ~/ardupilot && python3 Tools/autotest/sim_vehicle.py -v ArduPlane \
         --console --map --speedup 10 -l <enlem>,<boylam>,<irtifa>,0

   Açılan MAVProxy isteminde (sim_vehicle'ı başlattığın TERMİNALDE,
   konsol PENCERESİNDE değil):

     param set TERRAIN_ENABLE 0
     param set AIRSPEED_CRUISE <AIRSPEED_CRUISE>
     param set SIM_WIND_SPD <rüzgâr_hız>
     param set SIM_WIND_DIR <rüzgâr_yön>
     param set SIM_WIND_T 1
     wp load /home/tasneem/yeni.waypoints
     mode AUTO
     arm throttle

   wp load'da TAM YOL şart: MAVProxy ~ işaretini açmıyor.
   Kalkış başlamazsa: rc 3 1800
   Harita açılmazsa (WSLg sorunu) --map'i çıkar, --console yeter.

   SIM_WIND_T 1 = rüzgâr her kotta aynı. Varsayılan (0) rüzgârı 60 m
   AGL altında azaltıyor, bizim modelimiz ise tekdüze rüzgâr sayıyor.


   HEM HARİTA HEM BETİK istersen: MAVProxy 5760'ı tuttuğu için betik
   uçağa doğrudan bağlanamıyor, MAVProxy'nin bir çıkışına bağlanması
   gerekiyor. Başlatma komutuna --out udp:127.0.0.1:14551 ekle, sonra
   ikinci terminalde:

     python3 ~/tools/fly_mission.py ~/yeni.waypoints \
         <rüzgâr_hız> <rüzgâr_yön> <AIRSPEED_CRUISE> udpin:127.0.0.1:14551



4) SONUCU OKU

     python3 ~/tools/log_report.py \
         $(ls -t ~/ardupilot/logs/*.BIN | head -1) <rüzgâr_hız> <rüzgâr_yön>

   Rota süresini, uçulan mesafeyi, kot aralığını ve GERÇEK hava hızını
   veriyor. Arayüzün yazdığı süreyle karşılaştır.

   Neden tlog değil de dataflash: son waypointin MISSION_ITEM_REACHED'i
   telemetride kaçabiliyor (bir koşuda kaçtı), dataflash'ta kaçmıyor.
   Ayrıca dataflash koşu başına yeni dosya açıyor, üzerine yazmıyor.


--- MISSION PLANNER: GERÇEK UÇAĞA YÜKLEMEK ---

Masaüstündeki MissionPlanner-latest klasöründen aç.

1) Plan ekranına geç
2) Load WP File ile .waypoints dosyasını aç
3) İrtifa sütununda Abs (mutlak) yazdığını DOĞRULA. Mission Planner
   bunu Relative'e çevirmeye kalkarsa görev kayar.
4) Uçağa bağlıyken Write WPs ile yükle

DİKKAT: aşağıdaki AIRSPEED_CRUISE hesabı SITL'e özel. Gerçek uçakta
gösterge/gerçek dönüşümü ders kitabındaki 1/sqrt(sigma); 1/sigma kuralı
SITL'in pitot davranışı. Gerçek uçağa SITL için hesaplanmış hızı yazma.


--- SITL'DE ÖLÇÜLEN DOĞRULAMA ---

Rüzgâr modelinin tahmini ile gerçek uçuş, beş koşu:

  rota                 rüzgâr    tahmin     ölçülen     hata
  Alpler   26 km    0 / 8 m/s    882.8 s    889.8 s    -0.8 %
  İstanbul 6.3 km       8 m/s    240.7 s    239.0 s    +0.7 %
  Şile     12.6 km      13 m/s    514.0 s    515.7 s    -0.3 %
  Alpler   102 km       14 m/s   4229.8 s   4267.1 s   -0.9 %
  Alpler   120 km       14 m/s   5164.4 s   5177.8 s   -0.3 %

Uçulan mesafe planlanana %0.2 içinde oturuyor -- WP_RADIUS 50 m ile köşe
kesmesine ve görev dosyasının 5 m toleranslı seyreltmesine rağmen.

Kalan hata rüzgârda değil, sabit hava hızı varsayımında: gerçek uçak
tırmanışta hız kaybediyor. Kalkış sonrası ilk bacak %8-22 sapıyor,
karşı rüzgârlı uzun bacak %1-2.


--- EN KRİTİK AYAR: HAVA HIZI ---

Model GERÇEK hava hızı (TAS) ister; AIRSPEED_CRUISE ise GÖSTERGE hızı
(IAS) alır. Dönüşüm:

  gösterge = gerçek * sigma        sigma = (1 - 2.25577e-5 * kot)^4.2559

DİKKAT: sqrt(sigma) DEĞİL. Ders kitabı bağıntısı TAS = IAS/sqrt(sigma)
der ama SITL'de ölçülen oran bunun tam iki katı, yani 1/sigma. 2026-09-04,
102 km'lik rotada yedi kot kuşağında ölçüldü:

  kot(m)   ölçülen   1/sqrt(sigma)   1/sigma
    300     1.054       1.027         1.055
    900     1.108       1.053         1.109
   1500     1.171       1.084         1.176
   2100     1.228       1.115         1.229

Yanlış bağıntıyla AIRSPEED_CRUISE 26.44 seçilmişti; uçak 29.6 m/s uçtu ve
tahmin %6.7 saptı. 24.99'a düzeltilince uçak 27.9 m/s uçtu, hata %0.9.

Ortalama kot planlayıcının çıktısında yazıyor; sigma'yı ondan hesapla.

Gerçek hava hızı SENSÖRDEN okunmaz: ARSP.Airspeed gösterge hızıdır.
Gerçeği yer hızı vektöründen rüzgâr çıkarılarak bulunuyor -- sensörden
bağımsız ve fizik gereği doğru. tools/log_report.py bunu yapıyor.


--- SITL BAŞLAMIYORSA: ÖNCEKİ OTURUM AÇIK KALMIŞTIR ---

Ctrl+C ile sim_vehicle'ı kapatmak YETMİYOR. Arkada mavproxy, xterm ve
arduplane süreçleri sağ kalıyor ve 5760 portunu tutuyorlar. Yeni oturum
o porta bağlanamayınca sessizce ölüyor - ekranda anlamlı bir hata bile
vermiyor, sadece "başlamıyor" gibi görünüyor.

Başlatmadan ÖNCE kontrol et:

  pgrep -af "arduplane|mavproxy|sim_vehicle" || echo temiz

Kalıntı varsa temizle:

  pkill -9 -f "sim_vehicle|mavproxy|bin/arduplane"
  pkill -9 -f "xterm.*ArduPlane"

Portun boşaldığını doğrula:

  ss -tln | grep 5760 || echo "5760 bos"

Not: pkill kendi komut zincirini de öldürebiliyor; tek satırda başka
komutla birleştirme, ayrı çalıştır.


--- MAVProxy ÇIKIŞLARI (--out) NE İŞE YARIYOR ---

MAVProxy bir MAVLink dağıtıcısı. Uçakla tek bağlantı kuruyor
(--master tcp:127.0.0.1:5760), sonra bu akışın KOPYALARINI istenen
adreslere yolluyor. Her adrese "çıkış" deniyor. Yukarıdaki YOL B bunu
kullanıyor: betik uçağa değil, MAVProxy'nin bir çıkışına bağlanıyor.

Başlatma komutuna gömmek (tercih edilen):

  --out udp:127.0.0.1:14551          betik icin, WSL icinde
  --out udp:172.27.128.1:14550       Mission Planner icin, Windows'ta

MAVProxy istemindeyken elle de eklenebiliyor:

  output add 172.27.128.1:14550
  output list
  output remove 2

172.27.128.1 = WSL'den görünen Windows host IP'si. SABİT DEĞİL, WSL
yeniden başlayınca değişir. Öğrenmek için WSL'de:

  ip route show default | awk '{print $3}'

Mission Planner tarafı: sağ üstte UDP seç (UDPCl değil), CONNECT, port
sorarsa 14550. Veri gelmezse güvenlik duvarıdır - Mission Planner'ı
yönetici olarak aç.

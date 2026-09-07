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
Home'u rotanın en alçak noktasının ÜSTÜNE koyarsan uçak yerin altına
inmeye çalışır ve düşer. Rotanın en düşük kotunu görev dosyasının son
sütunundan oku, home'u onun en az 50 m altına koy.


--- YOL A: SITL (WSL) - ölçüm yapmak için ---

1) Dosyayı WSL'e geçir. Windows yolunda Türkçe karakter var ve WSL'e
   komut geçerken bozuluyor; ASCII bir ara duraktan geçir.

   CMD'de:
     copy "%USERPROFILE%\Downloads\gorev_*.waypoints" C:\Users\PC7668_BD26\gorev.waypoints

   WSL'de:
     cp /mnt/c/Users/PC7668_BD26/gorev.waypoints ~/gorev.waypoints

2) Simülatörü başlat (-l sırası: enlem,boylam,irtifa,yön):

     cd ~/ardupilot
     python3 Tools/autotest/sim_vehicle.py -v ArduPlane --console --map -l 46.08,14.28,450,0 --speedup 10

   --map açılmazsa (WSLg sorunu) onu çıkar, --console yeter.
   --speedup 10 ile 20 dakikalık uçuş 2 dakikada biter; ölçülen süre
   simülasyon zamanı olduğu için sonuç bozulmuyor.

3) MAVProxy isteminde -- konsol PENCERESİNDE değil, sim_vehicle'ı
   başlattığın TERMİNALDE:

     param set TERRAIN_ENABLE 0
     wp load /home/tasneem/gorev.waypoints
     mode AUTO
     arm throttle

   wp load'da TAM YOL şart: MAVProxy ~ işaretini açmıyor.
   Kalkış başlamazsa: rc 3 1800

   Rüzgâr denemek için:
     param set SIM_WIND_SPD 8
     param set SIM_WIND_DIR 0      (rüzgârın GELDİĞİ yön)

4) Uçuş bitince süreyi çıkar:

     python3 /home/tasneem/tools/tlog_legs.py ~/ardupilot/mav.tlog

   Betik her waypointe varış zamanını ve rota süresini yazıyor; kalkışı
   dışarıda bırakıyor çünkü modelin tahmini kalkışı kapsamıyor. Zaman
   uçağın kendi saatinden alınıyor, duvar saatinden değil (--speedup
   yüzünden duvar saati yanlış olurdu).

   DİKKAT: mav.tlog her koşuda üzerine yazılıyor. Saklamak istersen:
     cp ~/ardupilot/mav.tlog ~/kosu_1.tlog


--- YOL B: Mission Planner - gerçek uçağa yüklemek için ---

Masaüstündeki MissionPlanner-latest klasöründen aç.

1) Plan ekranına geç
2) Load WP File ile .waypoints dosyasını aç
3) İrtifa sütununda Abs (mutlak) yazdığını DOĞRULA. Mission Planner
   bunu Relative'e çevirmeye kalkarsa görev kayar.
4) Uçağa bağlıyken Write WPs ile yükle

Mission Planner'ın kendi simülasyonu da var (Simulation sekmesi), ama
ölçüm betiği WSL'deki tlog'a bakıyor.


--- SITL'DE ÖLÇÜLEN DOĞRULAMA (2026-09-01) ---

Rüzgâr modelinin tahmini ile gerçek uçuş:

  sakin hava        tahmin  882.8 s   ölçülen  889.8 s   -0.8 %
  8 m/s kuzeyden    tahmin 1080.3 s   ölçülen 1088.3 s   -0.7 %
  8 m/s batıdan     tahmin  818.4 s   ölçülen  830.0 s   -1.4 %

Önemli: model GERÇEK hava hızı ister. VFR_HUD'ın verdiği GÖSTERGE hava
hızıdır ve 850 m'de gerçek hız onun %4 üstündedir. Gösterge hızıyla
beslendiğinde hata %6.7 çıktı, gerçek hızla %0.8.

Uçulan mesafe 26.56 km, planlanan 26.39 km (+%0.6) -- WP_RADIUS 50 m ile
köşe kesmesine ve görev dosyasının 5 m toleranslı seyreltmesine rağmen.


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


--- output add NE İŞE YARIYOR ---

MAVProxy bir MAVLink dağıtıcısı. Uçakla tek bağlantı kuruyor
(--master tcp:127.0.0.1:5760), sonra bu akışın KOPYALARINI istenen
adreslere yolluyor. Her adrese "çıkış" deniyor.

Varsayılan çıkışlar 127.0.0.1:14550 ve 14551 - ama buradaki "aynı
makine" WSL. Mission Planner Windows'ta, ayrı ağ alanında, o yüzden
varsayılan çıkışları göremiyor.

  output add 172.27.128.1:14550     Windows'a kopya yolla
  output list                        mevcut cikislari gor
  output remove 2                    sil (numara list'te)

172.27.128.1 = WSL'den görünen Windows host IP'si. SABİT DEĞİL, WSL
yeniden başlayınca değişir. Öğrenmek için WSL'de:

  ip route show default | awk '{print $3}'

Her seferinde yazmamak için başlatma komutuna gömülebilir:

  --out=udp:172.27.128.1:14550

Mission Planner tarafı: sağ üstte UDP seç (UDPCl değil), CONNECT, port
sorarsa 14550. Veri gelmezse güvenlik duvarıdır - Mission Planner'ı
yönetici olarak aç.


--- HOME KOORDİNATI GÖREVE AİT OLMALI ---

sim_vehicle'daki -l enlem,boylam,irtifa,yön değeri görevin home'u ile
aynı yerde olmalı; yoksa uçak görevden yüzlerce km uzakta doğar.

Görev dosyasının 2. satırı (indeks 0) home satırıdır, 9. ve 10. alanlar
enlem ve boylam. İrtifa ise rotanın EN DÜŞÜK kotunun altında olmalı.

  İstanbul görevi:  -l 41.12113,29.04606,15,0
  Alpler görevi:    -l 46.08,14.28,450,0

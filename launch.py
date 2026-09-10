"""Paketlenmis surumun giris noktasi: sunucuyu baslatir, tarayiciyi acar.

app/server.py'nin kendi main'i sade kalsin diye ayri duruyor - orasi
gelistirirken "python -m app.server" ile kosuluyor ve tarayici acmasi
istenmiyor.

Tarayici AYRI bir is parcaciginda ve gecikmeli aciliyor: sunucu dinlemeye
baslamadan acilirsa kullanici "baglanti reddedildi" goruyor ve uygulamanin
calismadigini saniyor.
"""

import threading
import webbrowser

from app.server import HOST, PORT, main

OPEN_DELAY = 1.5               # saniye; sunucunun soketi acmasi icin


def open_browser():
    webbrowser.open(f"http://{HOST}:{PORT}/")


def run():
    threading.Timer(OPEN_DELAY, open_browser).start()
    print(f"  Gorev planlayici:  http://{HOST}:{PORT}/")
    print("  Bu pencereyi kapatmak sunucuyu durdurur.")
    print()
    try:
        main()
    except KeyboardInterrupt:
        print("\n  durduruldu")


if __name__ == "__main__":
    run()

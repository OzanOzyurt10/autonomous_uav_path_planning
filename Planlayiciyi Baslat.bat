@echo off
rem Cift tiklayarak calistir. Sunucuyu baslatip tarayiciyi aciyor.
rem
rem Terminale komut yazmak "kullanmasi zor" sikayetinin en somut
rem parcasiydi; uygulamanin kendisi degismedi, yalnizca baslatmasi.
rem
rem Tarayiciyi acan mantik launch.py'de - paketlenmis .exe de ayni
rem dosyayi kullaniyor. Burada tekrarlanmasi iki farkli gecikme degeri
rem demek olurdu ve biri digeri duzeltilirken unutulurdu.

cd /d "%~dp0"

if not exist "venv\Scripts\python.exe" (
    echo.
    echo   venv bulunamadi: %CD%\venv
    echo.
    echo   Once sanal ortami kur:
    echo     python -m venv venv
    echo.
    pause
    exit /b 1
)

echo   Gorev planlayici baslatiliyor...
echo   Bu pencereyi KAPATMA - sunucu burada calisiyor.
echo   Durdurmak icin: Ctrl+C ya da pencereyi kapat.
echo.

venv\Scripts\python.exe launch.py

rem Sunucu bir hatayla dustuyse pencere kapanmadan once mesaj gorunsun.
if errorlevel 1 (
    echo.
    echo   Sunucu hatayla durdu. Yukaridaki mesaja bak.
    pause
)

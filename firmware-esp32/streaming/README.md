# FG23 SDR — ESP32-S3 oldal

HA7DCD / Cimbi. EFR32FG23 rádió → SPI → ESP32-S3 → USB / WiFi / rtl_tcp → SDR++.

A teljes lánc, a mérési eredmények és a tervezési döntések a projekt
státuszdokumentumában vannak (`FG23-SDR-allapot-2026-07-31.md`). Ez a fájl csak
azt írja le, mi hol van és hogyan indul.

## Mappaszerkezet

```
fg23-sdr/
├─ platformio.ini      négy environment, lásd lentebb
├─ src/
│  └─ main.cpp         a fő ESP32 firmware
├─ bench/
│  ├─ usb_bench.cpp    USB átviteli plafon mérése (nem kell hozzá FG23)
│  └─ cs_probe.cpp     CS-láb próba: eljut-e a keretjel az ESP-ig
├─ tools/
│  ├─ iq_bridge.py     USB → TCP híd az asztali SDR++-hoz
│  └─ usb_sink.py      a benchmark PC oldala
└─ fg23/               a Silicon Labs oldal — NEM ez a projekt fordítja
   ├─ app.c
   ├─ iq_stream.c / .h
   └─ cmdlink.c / .h
```

A `bench/` szándékosan **nincs** a `src/` alatt: a PlatformIO a `src/` alól
mindent lefordítana, és akkor több `setup()`/`loop()` ütközne. Az
environmentek `build_src_filter`-rel választják ki, melyik fájl épüljön.

## Environmentek

| environment | mit épít | mikor kell |
|---|---|---|
| `esp32s3` | `src/main.cpp` | ez a normál firmware, ez a default |
| `bench_hwcdc` | `bench/usb_bench.cpp`, `ARDUINO_USB_MODE=1` | USB-plafon, beépített USB-Serial/JTAG |
| `bench_tiny` | `bench/usb_bench.cpp`, `ARDUINO_USB_MODE=0` | USB-plafon, TinyUSB OTG |
| `probe_cs` | `bench/cs_probe.cpp` | ha "nincs adat" és a drótra gyanakszol |

VSCode-ban alul a státuszsávban válthatsz köztük, vagy a PlatformIO
oldalsávban a *Project Tasks* alatt mindegyik kap saját Build/Upload/Monitor
gombot.

## Első indítás

1. `platformio.ini` → írd be a `monitor_port`-ot és az `upload_port`-ot. **A
   CP2102 portja kell**, nem a forrasztott natív USB — különben a monitor
   bináris szemetet fog mutatni.
2. `src/main.cpp` → `WIFI_SSID` és `WIFI_PASS`. Ha nem jön össze 10 mp alatt,
   magától saját AP-t nyit (SSID/jelszó a `secrets.h`-ban) — kocsiban ez a jó.
3. Build + Upload az `esp32s3` environmenttel.
4. A FG23 autostarttal indul, nem kell `i32`-t nyomni.

## Csatlakozás

| kliens | forrás | cím | formátum |
|---|---|---|---|
| SDR++ asztali, python nélkül | Network / TCP / Client | `<ESP IP>:8888` | Int16 |
| SDR++ asztali, USB-n | Network / TCP / Client | `127.0.0.1:8888` | Int16 |
| SDR++ Android, SDR Touch | RTL-TCP | `<ESP IP>:1234` | 8 bit |

Az USB-s úthoz: `python tools/iq_bridge.py COM4` — a natív USB portja, az
Eszközkezelőben „Soros USB-eszköz", nem a CP2102.

## Az USB-plafon mérése

```
1. bench_hwcdc → Upload
2. Monitor a CP2102-n (115200)
3. python tools/usb_sink.py COM4      <- a NATÍV USB portja!
4. ~45 mp, aztán olvasd le mindkét oldalt
5. ugyanez bench_tiny-nel
```

Olvasó nélkül a mérés értelmetlen: az USB CDC-n a gazdagép kezdeményez, tehát
`usb_sink.py` nélkül 0 kB/s-ot fogsz látni — nem a linkről, hanem arról, hogy
senki nem olvas.

## A `fg23/` mappáról — olvasd el

Ezek a fájlok **nem** ebben a projektben fordulnak. A Simplicity Studio saját
projektmappával dolgozik, tehát ugyanaz a forrás fizikailag két helyen létezhet.

**Válassz egy irányt és tartsd is magad hozzá.** Ha ez a git-elt másolat az
igazság, akkor szerkesztés után innen másolod a Studio projektjébe. Ha a Studio
projektje az igazság, akkor commit előtt onnan másolod ide. A kettő között
oda-vissza szerkeszteni az, amiből néma eltérés lesz — pontosan ez okozta a
2026-07-31-i éjszakai hajszát, ahol egy elavult forrásfájl EXP-térképe egy
pozícióval el volt csúszva, és fél éjszakán át egy nem hajtott lábat mértünk.

A `cmdlink.c` alapból **ki van kapcsolva** (`CMDLINK_ENABLE 0` az `app.c`-ben)
és még nincs hardveren mérve. Akkor kapcsold be, ha behúztad a drótot az ESP
GPIO4 → FG23 PA06 / EXP 11 útra.

## Bekötés

| WSTK EXP | FG23 | jel | → | ESP32-S3 |
|---|---|---|---|---|
| 15 | PC05 | SCLK | → | GPIO5 |
| 10 | PC00 | MOSI | → | GPIO6 |
| 13 | PA07 | CS (blokk-keret) | → | GPIO7 |
| 1 | GND | | → | GND |
| 11 | PA06 | CMD (hangolás, opcionális) | ← | GPIO4 |

Natív USB: kábel **D− → GPIO19**, **D+ → GPIO20**, **GND → GND**. A **VBUS-t
ne kösd be**. A D+/D− felcserélése pontosan Windows Code 43-at ad.

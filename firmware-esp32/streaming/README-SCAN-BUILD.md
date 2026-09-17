# FG23 szélessávú scan — mit hova másoltam, és mit kell fordítani (2026-08-15)

HA7DCD. Három projekt érintett. A fájlokat **közvetlenül a projektmappáidba
írtam** (a régi verziókról `.bak-20260815` másolat készült ugyanott).

## 1. FG23 (Simplicity Studio, `rail_soc_empty_FG23_iq_capture`) — LEFORDULT (2026-08-15 13:35, 0 hiba, 0 warning, scan.o linkelve)

| fájl | mi történt |
|---|---|
| `app.c` | `CMDLINK_ENABLE 1`; `#include "scan.h"`; `scan_init()` az `app_init`-ben; **`W` parancs** (`W<kHz>,<span_kHz>,<nbin>[,floor,range]` / `W0`); `scan_process()` a fő ciklusban; `s` státuszban scan-sor; `NEEDS_STOP`-ba `W` |
| `iq_stream.c` | `spi_tx_poll()` kiemelve a pumpból; **ext-mód API** (`iq_stream_ext_begin/end/busy/pump/send`) — SPECLINE-blokk küldése az SPI-n RAIL-stream nélkül, ugyanazon az LDMA/RDY úton |
| `iq_stream.h` | az ext-API prototípusai |
| `cmdlink.c` | `CMDLINK_LINE_MAX 24 → 48` (a W-parancs ~30 karakter) |
| **`scan.c` / `scan.h`** | ÚJ: RSSI-panadapter állapotgép (binenként Idle→StartRx→SetFreqOffset→settle→`RAIL_GetRssiAlt`), sor kész → SPECLINE |
| **`specline.h`** | ÚJ: a SPECLINE 1040 bájtos blokkformátum (KÖZÖS az ESP32-vel — ugyanaz a fájl mindkét projektben) |

Simplicity Studio a projektmappa új `.c` fájljait automatikusan fordítja
(a `cmdlink.c` is így került be). Ha mégsem: jobb klikk a projekten →
Refresh, vagy a `.cproject`-ben ellenőrizd, hogy nincs kizárva.

**Fordítás után ellenőrzés a VCOM-on (COM3/J-Link terminál):**
```
s                       -> a státuszban megjelenik: "# scan=0 kozep=0 ..."
W149800,10000,320       -> "# scan indul: kozep 149800 kHz, span 10000 kHz, 320 bin, lepes 31250 Hz ..."
s                       -> "# scan=1 ... sor=N ..." — az N-nek nőnie kell (~5–7/s várható)
W0                      -> "# scan leallt: ..." és a stream visszaindul (ha előtte futott)
```
Bármely billentyű is leállítja a scant (mint a streamet), és a karakter a
parancspufferbe kerül.

**A hangolási rács:** `TUNE_BASE_KHZ 144800`, 25 kHz, 401 csatorna → a scan
**144,8–154,8 MHz** között tud mérni; ezen kívüli binek padlót kapnak, és
egyszer kiír egy figyelmeztetést. (Ha 70 cm-re kell, a PHY base-t kell
átállítani, mint az `F` parancsnál.)

**Hangolható konstansok** (`scan.c` teteje): `SCAN_SETTLE_US 200`,
`SCAN_RSSI_WAIT_US 400`, `SCAN_BINS_PER_CALL 16`. A sor-idő a `s`-ben
(`sor_ido=… ms`) — ebből lehet lejjebb vinni a settle-t.

## 2. ESP32 (PlatformIO, `FG23-SDR-ESP32-S3-streaming`) — FORDÍTANI KELL

| fájl | mi történt |
|---|---|
| `src/spyserver.cpp` | `SETTING_FFT_*` + `STREAMING_MODE` FFT-bit kezelése; `spy_init(port, on_tune, **on_scan**)`; scan-változás → `on_scan` callback; `spy_feed` FFT_ONLY-ban nem küld IQ-t; **`spy_send_specline()`** → `MSG_UINT8_FFT` (301) EGYBEN a gyűrűbe; kliens-bontáskor `W0` |
| `src/spyserver.h` | `spy_scan_fn`, `spy_scan_wanted()`, `spy_send_specline()`, `spy_fft_lines()`; `#include "specline.h"` |
| `src/main.cpp` | `process()`: **SPECLINE-magic ág** (nem I/Q → SpyServer FFT + OLED); `spy_scan_cb()` → `cmdlink_send("W…"/"W0")`; RF-lapon scan alatt **mini-spektrum + 1-bites waterfall** (`oled_spectrum.h`); `st_specline` számláló |
| **`src/specline.h`** | ÚJ (azonos a FG23-éval) |
| **`src/oled_spectrum.h`** | ÚJ: 128×64 spektrum + Bayer-ditherelt waterfall (U8g2) |

```
pio run -e esp32s3_hwcdc -t upload --upload-port COM10   # a HWCDC env az eles (nem a TinyUSB-s esp32s3)
pio device monitor -p COM10 -b 115200
```
Várható a logban SDR++ SCAN-módnál: `spyserver: stream mod = 4 [FFT/scan]`
→ `scan: W149800,10000,320,-130,100` → `-> FG23: W149800,...`.
Vissza IQ-ra: `spyserver: stream mod = 1 [IQ]` → `scan: W0`.

## 3. SDR++ — IGEN, FORDÍTANI KELL, ÉS NEM „RÉSZLEGESEN"

A `C:\utils\sdrpp_windows_x64 (4)\` egy **kész bináris csomag**. Egy új
modul-DLL-t csak az SDR++ **forrásfájából** lehet fordítani (a modul a
`sdrpp_core.dll` C++-osztályaira linkel, MSVC-vel; a csomagban nincs
fejléc- és SDK-készlet). Tehát:

1. `git clone https://github.com/AlexandreRouma/SDRPlusPlus.git`
2. A `fg23_scan_source` mappát → `SDRPlusPlus\source_modules\fg23_scan_source\`
3. A gyökér `CMakeLists.txt`-be a többi source-modul mintájára:
   ```cmake
   option(OPT_BUILD_FG23_SCAN_SOURCE "FG23 scan source" ON)
   ...
   if (OPT_BUILD_FG23_SCAN_SOURCE)
   add_subdirectory("source_modules/fg23_scan_source")
   endif (OPT_BUILD_FG23_SCAN_SOURCE)
   ```
4. Windows build a readme szerint (Visual Studio 2019/2022 + vcpkg + PothosSDR;
   `cmake -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=…vcpkg.cmake ..`).
   Ez az **egész SDR++** első fordítása (~15–30 perc), utána már csak a
   modul fordul újra. A többi forrás-modul `OPT_BUILD_*`-ját KI lehet
   kapcsolni, hogy ne kelljen az összes SDR-lib (airspy, hackrf, …) — de
   ez így is teljes core-build.
5. A kész `fg23_scan_source.dll` → a **meglévő** `sdrpp_windows_x64 (4)\modules\`
   mappába csak akkor tehető, ha **ugyanabból a verzióból** épült
   (a core ABI-ja verziófüggő). Biztos út: a saját build `sdrpp.exe`-jét
   használd, VAGY nézd meg a régi csomag verzióját (Help/About) és azt a
   tag-et klónozd.
6. SDR++ → Module Manager → `fg23_scan_source` hozzáadás → Source: **„FG23 Scan"**
   → Mode: SCAN → Span/Bins → Play.

Alternatíva, ha nincs Visual Studio: **WSL2/Ubuntu**-ban az SDR++ Linux-build
egy `apt install` + `cmake` (~10 perc), és a modul .so-ját ott teszteled
(WSLg-vel megy a GUI). Androidhoz külön NDK-build (később).

## Protokoll-összefoglaló (három oldal egyben)

```
SDR++ fg23_scan_source  --TCP 5555-->  ESP32 spyserver.cpp  --cmdlink UART GPIO4->PA06-->  FG23 app.c
  STREAMING_MODE=4 (FFT_ONLY)          spy_scan_cb(want=1)          "W<kHz>,<span_kHz>,<nbin>,<floor>,<range>"
  FFT_FREQUENCY/DECIMATION(=span)/                                   scan_start() → RSSI-léptetés
  DISPLAY_PIXELS/DB_OFFSET/DB_RANGE                                     ↓ SPECLINE 1040 B (SPI, LDMA, RDY)
SDR++ waterfall  <--MSG_UINT8_FFT--    process(): SPECLINE_MAGIC  <--SPI slave DMA--   iq_stream_ext_send()
  (getFFTBuffer/pushFFT)               spy_send_specline() + OLED
  STREAMING_MODE=1 (IQ_ONLY)   →       spy_scan_cb(want=0) → "W0" →  scan_stop, restart_rx, stream vissza
```

## Amit NEM tudtam ellenőrizni

- FG23: **LEFORDULT** Zoltánnál (SiSDK 2025.6.3, GNU ARM 12.2.1, 0 hiba /
  0 warning) — a `RAIL_GetRssiAlt` létezik, a fallback nem kell. Futásidejű
  teszt (W-parancs, sor-idő) még hátra.
- ESP32: `spyserver.cpp` Arduino-stubbal szintaktikailag OK; a `main.cpp`
  patch anchor-alapú, a 6 beszúrás a helyén — a teljes fordítás a tiéd.
- SDR++: a modul a master fejlécei ellen `-fsyntax-only` OK; link a te buildeden.

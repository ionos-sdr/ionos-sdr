# FG23 wideband scan — file changes and build instructions (2026-08-15)

HA7DCD. Three projects are affected. The files were **written directly into
the project folders** (the previous versions were saved as `.bak-20260815`
copies alongside).

## 1. FG23 (Simplicity Studio, `rail_soc_empty_FG23_iq_capture`) — BUILDS (2026-08-15 13:35, 0 errors, 0 warnings, scan.o linked)

| file | change |
|---|---|
| `app.c` | `CMDLINK_ENABLE 1`; `#include "scan.h"`; `scan_init()` in `app_init`; **`W` command** (`W<kHz>,<span_kHz>,<nbin>[,floor,range]` / `W0`); `scan_process()` in the main loop; scan line in the `s` status; `W` added to `NEEDS_STOP` |
| `iq_stream.c` | `spi_tx_poll()` extracted from the pump; **ext-mode API** (`iq_stream_ext_begin/end/busy/pump/send`) — sending a SPECLINE block over SPI without the RAIL stream, on the same LDMA/RDY path |
| `iq_stream.h` | prototypes of the ext API |
| `cmdlink.c` | `CMDLINK_LINE_MAX 24 → 48` (the W command is ~30 characters) |
| **`scan.c` / `scan.h`** | NEW: RSSI panadapter state machine (per bin Idle→StartRx→SetFreqOffset→settle→`RAIL_GetRssiAlt`), row complete → SPECLINE |
| **`specline.h`** | NEW: the SPECLINE 1040-byte block format (SHARED with the ESP32 — the same file in both projects) |

Simplicity Studio compiles new `.c` files in the project folder automatically
(`cmdlink.c` was added the same way). If not: right-click the project →
Refresh, or check in `.cproject` that the file is not excluded.

**Verification on the VCOM after building (COM3 / J-Link terminal):**
```
s                       -> the status now contains the scan line (scan=0, center=0 ...)
W149800,10000,320       -> scan start message: center 149800 kHz, span 10000 kHz, 320 bins, step 31250 Hz ...
s                       -> scan=1 ... row=N ... — N must increase (~5–7/s expected)
W0                      -> scan stopped message, and the stream restarts (if it was running before)
```
Any keystroke also stops the scan (as it stops the stream), and the character
goes into the command buffer.

**The tuning grid:** `TUNE_BASE_KHZ 144800`, 25 kHz, 401 channels → the scan
can measure between **144.8–154.8 MHz**; bins outside this get the floor
value, and a warning is printed once. (For 70 cm the PHY base must be
changed, as with the `F` command.)

**Tunable constants** (top of `scan.c`): `SCAN_SETTLE_US 200`,
`SCAN_RSSI_WAIT_US 400`, `SCAN_BINS_PER_CALL 16`. The row time is shown in the
`s` status (row time in ms) — use it to reduce the settle time.

## 2. ESP32 (PlatformIO, `FG23-SDR-ESP32-S3-streaming`) — MUST BE BUILT

| file | change |
|---|---|
| `src/spyserver.cpp` | handling of `SETTING_FFT_*` + the `STREAMING_MODE` FFT bit; `spy_init(port, on_tune, **on_scan**)`; scan change → `on_scan` callback; `spy_feed` sends no IQ in FFT_ONLY; **`spy_send_specline()`** → `MSG_UINT8_FFT` (301) ATOMICALLY into the ring; `W0` on client disconnect |
| `src/spyserver.h` | `spy_scan_fn`, `spy_scan_wanted()`, `spy_send_specline()`, `spy_fft_lines()`; `#include "specline.h"` |
| `src/main.cpp` | `process()`: **SPECLINE magic branch** (not I/Q → SpyServer FFT + OLED); `spy_scan_cb()` → `cmdlink_send("W…"/"W0")`; on the RF page during a scan **mini spectrum + 1-bit waterfall** (`oled_spectrum.h`); `st_specline` counter |
| **`src/specline.h`** | NEW (identical to the FG23 one) |
| **`src/oled_spectrum.h`** | NEW: 128×64 spectrum + Bayer-dithered waterfall (U8g2) |

```
pio run -e esp32s3_hwcdc -t upload --upload-port COM10   # the HWCDC env is the production one (not the TinyUSB esp32s3)
pio device monitor -p COM10 -b 115200
```
Expected in the log with SDR++ in SCAN mode: `spyserver: stream mode = 4 [FFT/scan]`
→ `scan: W149800,10000,320,-130,100` → `-> FG23: W149800,...`.
Back to IQ: `spyserver: stream mode = 1 [IQ]` → `scan: W0`.

## 3. SDR++ — YES, IT MUST BE BUILT, AND NOT "PARTIALLY"

The prebuilt Windows package (`sdrpp_windows_x64`) is a **binary
distribution**. A new module DLL can only be built from the SDR++ **source
tree** (the module links against the C++ classes of `sdrpp_core.dll`, with
MSVC; the package contains no headers or SDK). Therefore:

1. `git clone https://github.com/AlexandreRouma/SDRPlusPlus.git`
2. Copy the `fg23_scan_source` folder → `SDRPlusPlus\source_modules\fg23_scan_source\`
3. In the root `CMakeLists.txt`, following the pattern of the other source modules:
   ```cmake
   option(OPT_BUILD_FG23_SCAN_SOURCE "FG23 scan source" ON)
   ...
   if (OPT_BUILD_FG23_SCAN_SOURCE)
   add_subdirectory("source_modules/fg23_scan_source")
   endif (OPT_BUILD_FG23_SCAN_SOURCE)
   ```
4. Windows build as per the readme (Visual Studio 2019/2022 + vcpkg + PothosSDR;
   `cmake -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=…vcpkg.cmake ..`).
   This is the first build of the **whole SDR++** (~15–30 minutes); afterwards
   only the module is rebuilt. The `OPT_BUILD_*` of the other source modules
   CAN be turned off so that not all SDR libraries (airspy, hackrf, …) are
   needed — but it is still a full core build.
5. The resulting `fg23_scan_source.dll` can be placed into the **existing**
   `sdrpp_windows_x64\modules\` folder only if it was built **from the same
   version** (the core ABI is version-dependent). The safe route: use the
   `sdrpp.exe` of your own build, OR check the version of the old package
   (Help/About) and clone that tag.
6. SDR++ → Module Manager → add `fg23_scan_source` → Source: **"FG23 Scan"**
   → Mode: SCAN → Span/Bins → Play.

Alternative without Visual Studio: in **WSL2/Ubuntu** the SDR++ Linux build is
an `apt install` + `cmake` (~10 minutes), and the module `.so` can be tested
there (the GUI works with WSLg). Android requires a separate NDK build (later).

## Protocol summary (all three sides)

```
SDR++ fg23_scan_source  --TCP 5555-->  ESP32 spyserver.cpp  --cmdlink UART GPIO4->PA06-->  FG23 app.c
  STREAMING_MODE=4 (FFT_ONLY)          spy_scan_cb(want=1)          "W<kHz>,<span_kHz>,<nbin>,<floor>,<range>"
  FFT_FREQUENCY/DECIMATION(=span)/                                   scan_start() → RSSI stepping
  DISPLAY_PIXELS/DB_OFFSET/DB_RANGE                                     ↓ SPECLINE 1040 B (SPI, LDMA, RDY)
SDR++ waterfall  <--MSG_UINT8_FFT--    process(): SPECLINE_MAGIC  <--SPI slave DMA--   iq_stream_ext_send()
  (getFFTBuffer/pushFFT)               spy_send_specline() + OLED
  STREAMING_MODE=1 (IQ_ONLY)   →       spy_scan_cb(want=0) → "W0" →  scan_stop, restart_rx, stream resumes
```

## Not yet verified

- FG23: **BUILDS** on the author's machine (SiSDK 2025.6.3, GNU ARM 12.2.1,
  0 errors / 0 warnings) — `RAIL_GetRssiAlt` exists, the fallback is not
  needed. Run-time test (W command, row time) still pending.
- ESP32: `spyserver.cpp` is syntactically OK with an Arduino stub; the
  `main.cpp` patch is anchor-based, the 6 insertions are in place — the full
  build is still pending.
- SDR++: the module passes `-fsyntax-only` against the master headers; linking
  is pending on the target build.

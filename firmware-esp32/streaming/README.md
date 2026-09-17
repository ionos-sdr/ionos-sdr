# FG23 SDR — ESP32-S3 side

HA7DCD. EFR32FG23 radio → SPI → ESP32-S3 → USB / WiFi / rtl_tcp → SDR++.

The complete chain, the measurement results and the design decisions are in
the project status document (`FG23-SDR-allapot-2026-07-31.md`). This file only
describes what is where and how to start it.

## Directory layout

```
fg23-sdr/
├─ platformio.ini      four environments, see below
├─ src/
│  └─ main.cpp         the main ESP32 firmware
├─ bench/
│  ├─ usb_bench.cpp    USB throughput ceiling measurement (no FG23 needed)
│  └─ cs_probe.cpp     CS pin probe: does the frame signal reach the ESP
├─ tools/
│  ├─ iq_bridge.py     USB → TCP bridge for desktop SDR++
│  └─ usb_sink.py      the PC side of the benchmark
└─ fg23/               the Silicon Labs side — NOT built by this project
   ├─ app.c
   ├─ iq_stream.c / .h
   └─ cmdlink.c / .h
```

`bench/` is deliberately **not** under `src/`: PlatformIO would compile
everything under `src/`, and several `setup()`/`loop()` pairs would collide.
The environments select the file to build with `build_src_filter`.

## Environments

| environment | builds | when to use |
|---|---|---|
| `esp32s3` | `src/main.cpp` | the normal firmware, the default |
| `bench_hwcdc` | `bench/usb_bench.cpp`, `ARDUINO_USB_MODE=1` | USB ceiling, built-in USB-Serial/JTAG |
| `bench_tiny` | `bench/usb_bench.cpp`, `ARDUINO_USB_MODE=0` | USB ceiling, TinyUSB OTG |
| `probe_cs` | `bench/cs_probe.cpp` | when there is "no data" and the wiring is suspect |

In VSCode switch between them in the status bar at the bottom, or under
*Project Tasks* in the PlatformIO sidebar, where each gets its own
Build/Upload/Monitor buttons.

## First start

1. `platformio.ini` → set `monitor_port` and `upload_port`. **The CP2102 port
   is required**, not the soldered native USB — otherwise the monitor shows
   binary garbage.
2. `src/main.cpp` → `WIFI_SSID` and `WIFI_PASS`. If no connection is made
   within 10 s, the firmware opens its own AP (SSID/password in `secrets.h`)
   — suitable for mobile use.
3. Build + Upload with the `esp32s3` environment.
4. The FG23 starts with autostart; no `i32` command is needed.

## Connecting

| client | source | address | format |
|---|---|---|---|
| SDR++ desktop, without Python | Network / TCP / Client | `<ESP IP>:8888` | Int16 |
| SDR++ desktop, over USB | Network / TCP / Client | `127.0.0.1:8888` | Int16 |
| SDR++ Android, SDR Touch | RTL-TCP | `<ESP IP>:1234` | 8 bit |

For the USB path: `python tools/iq_bridge.py COM4` — the native USB port,
listed as "USB Serial Device" in the Device Manager, not the CP2102.

## Measuring the USB ceiling

```
1. bench_hwcdc → Upload
2. Monitor on the CP2102 (115200)
3. python tools/usb_sink.py COM4      <- the NATIVE USB port!
4. ~45 s, then read both sides
5. the same with bench_tiny
```

Without a reader the measurement is meaningless: on USB CDC the host
initiates, so without `usb_sink.py` 0 kB/s is shown — which says nothing about
the link, only that nobody is reading.

## About the `fg23/` directory — please read

These files are **not** compiled in this project. Simplicity Studio works with
its own project folder, so the same source may physically exist in two places.

**Choose one direction and stick to it.** If this git-tracked copy is the
source of truth, copy from here into the Studio project after editing. If the
Studio project is the source of truth, copy from there to here before
committing. Editing back and forth between the two is what produces silent
divergence — exactly this caused the overnight hunt of 2026-07-31, where the
EXP map of an outdated source file was shifted by one position, and half the
night was spent measuring an undriven pin.

`cmdlink.c` is **disabled** by default (`CMDLINK_ENABLE 0` in `app.c`) and has
not yet been measured on hardware. Enable it once the wire for the ESP GPIO4
→ FG23 PA06 / EXP 11 path is in place.

## Wiring

| WSTK EXP | FG23 | signal | → | ESP32-S3 |
|---|---|---|---|---|
| 15 | PC05 | SCLK | → | GPIO5 |
| 10 | PC00 | MOSI | → | GPIO6 |
| 13 | PA07 | CS (block frame) | → | GPIO7 |
| 1 | GND | | → | GND |
| 11 | PA06 | CMD (tuning, optional) | ← | GPIO4 |

Native USB: cable **D− → GPIO19**, **D+ → GPIO20**, **GND → GND**. **Do not
connect VBUS.** Swapping D+/D− produces exactly Windows Code 43.

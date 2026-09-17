# fg23_scan_source — wideband scan waterfall in SDR++ from the FG23/ESP32 SpyServer

HA7DCD, 2026-08. GPL-3 (derived from the SDR++ spyserver_source).

## Overview

The stock SDR++ SpyServer module only requests IQ and computes the waterfall
from it locally. This module provides **two operating modes**:

| Mode | Request to the server | Display | Audio |
|---|---|---|---|
| **IQ** | `STREAM_MODE_IQ_ONLY` (as in the stock module) | narrow (~1 MHz), true FFT | yes |
| **SCAN** | `STREAM_MODE_FFT_ONLY` + `SETTING_FFT_*` | **several MHz** of stepped spectrum, `UINT8_FFT` lines pushed directly into `gui::waterfall` | no |

Clicking on the waterfall in SCAN mode only moves the scan centre frequency
(`SETTING_FFT_FREQUENCY`). To listen, switch to IQ (mode combo box).

## Files

```
fg23_scan_source/
  CMakeLists.txt
  src/main.cpp            — the module (menu, modes, waterfall injection)
  src/fg23_client.{h,cpp} — SpyServer client + UINT8_FFT/DINT4_FFT handling
  src/spyserver_protocol.h — the stock header (unchanged)
  esp32/spy_fft.h         — ESP32-side extension for spyserver.cpp
  esp32/oled_spectrum.h   — 128×64 OLED mini spectrum + 1-bit waterfall
```

## Building (desktop SDR++)

1. Copy the folder under `source_modules/` in the SDR++ source tree.
2. In the root `CMakeLists.txt`, following the pattern of the other source modules:
   ```cmake
   option(OPT_BUILD_FG23_SCAN_SOURCE "FG23 scan source" ON)
   ...
   if (OPT_BUILD_FG23_SCAN_SOURCE)
   add_subdirectory("source_modules/fg23_scan_source")
   endif (OPT_BUILD_FG23_SCAN_SOURCE)
   ```
3. Standard build (`cmake .. && make`). `fg23_scan_source.so/.dll` is placed in
   the modules folder; SDR++ → Module Manager → add `fg23_scan_source`.
4. Source: **"FG23 Scan"**.

The module has been syntax-checked against the headers of a cloned SDR++ tree
(`g++ -fsyntax-only`); a full link must be done in your own build environment.

Android: modules are compiled into the APK — see the `android/` folder and the
CI recipe (NDK). This is a separate step; test on the desktop first.

## Protocol convention (client ↔ ESP32)

The standard SpyServer FFT settings are used, with one reinterpretation:

| Setting | Meaning here |
|---|---|
| `SETTING_FFT_FREQUENCY` (201) | scan centre frequency [Hz] |
| `SETTING_FFT_DECIMATION` (202) | **SPAN [Hz]** (not a decimation index) |
| `SETTING_FFT_DISPLAY_PIXELS` (205) | nbin (320/640/1024/2048) |
| `SETTING_FFT_DB_OFFSET` (203) | −floor dBm (e.g. 130) |
| `SETTING_FFT_DB_RANGE` (204) | range in dB (e.g. 100) |
| `SETTING_STREAMING_MODE` (0) | `FFT_ONLY`=4 → scan; `IQ_ONLY`=1 → listening |

Message: `MSG_TYPE_UINT8_FFT` (301), `StreamType=FFT` (4), body `nbin × uint8`,
`dB = floor + v·range/255`. **Header and body are written to the ring buffer as a
single unit** (`spy_fft_send`).

The SDR++ side stretches the line to the waterfall's raw FFT length
(Display → FFT size, default 65536): fewer bins → linear interpolation, more
bins → max decimation.

## ESP32 integration (spyserver.cpp)

```cpp
#include "spy_fft.h"
// CMD_SET_SETTING:
if (!spy_fft_handle_setting(t.Setting, t.Value)) { /* existing switch */ }
// main loop:
if (spy_fft_state().dirty) {
    const auto& p = spy_fft_state();
    if (p.fft_enabled) fg23_cmd_scan(p.freq_hz, p.span_hz, p.nbin);  // CMD line
    else               fg23_cmd_iq(iq_freq, iq_decim);
    spy_fft_clear_dirty();
}
// SPECLINE from the FG23:
spy_fft_send(ring_enqueue_whole, spec.bins, spec.nbin);
ili9341_wf.push(spec.bins, spec.nbin);
oled.push(spec.bins, spec.nbin);
```

`ring_enqueue_whole(p1,n1,p2,n2)` is the "all or nothing" ring-buffer write
(the same pattern used for the Android freeze fix).

## Open items / next steps

- FG23 side: scan PHY (fixed gain, wide RX BW), RSSI stepping → SPECLINE
  (see the sweep plan in the project documentation).
- Wiring of the CMD line (PA06/EXP11 ← GPIO4): without it the scan parameters
  do not reach the FG23.
- If "waterfall + audio simultaneously" is required in SDR++, that is upstream
  #1356 (FFT+VFO mode); this module does not provide it.

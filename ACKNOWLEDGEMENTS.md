# Acknowledgements and third-party sources

Ionos SDR stands on the work of others. This file lists what was used, how, and under which licence. Where code was **derived** the original notice is kept in the file header; where only the **design or protocol** was followed, the implementation here is independent and the original is credited for the idea.

## Code derived from

| Project | Author / licence | Used where | How |
|---|---|---|---|
| [geckokapula](https://github.com/tejeez/geckokapula) | Tatu Peltola, OH2EAT — MIT | `firmware-fg23/iq_capture/app.c`, `firmware-fg23/railtest_wspr/app.c` | The I/Q capture core (RAIL RX FIFO handling, event callback structure) derives from `dsp_driver.c`; the `int16 Q-first` sample format is geckokapula's `iq_in_t`. Original copyright notice retained in the file headers. |
| [SDR++](https://github.com/AlexandreRouma/SDRPlusPlus) `spyserver_source` | Alexandre Rouma (Ryzerth) — GPL-3.0 | `host/sdrpp-source/` | The `fg23` source module is based on the SpyServer source module (connection handling, module skeleton, FFT/waterfall injection). The module is therefore GPL-3.0. |
| SpyServer protocol header | Youssef Touil (Airspy), corrections by Ryzerth | `host/sdrpp-source/src/spyserver_protocol.h` | Verbatim protocol structures and constants, notice retained. |
| Silicon Labs Simplicity SDK templates and generated configuration | Silicon Laboratories Inc. — Zlib | `firmware-fg23/*/main.c`, `app_init.h`, `app_process.h`, `config/**` | Project templates and Radio Configurator output, altered for this project; notices retained. RAIL itself is used only through its public API and is not redistributed here. |

## Designs and protocols followed (independent implementation)

| Origin | Author / licence | Used where | Notes |
|---|---|---|---|
| SpyServer wire protocol | Airspy / Youssef Touil | `firmware-esp32/streaming/src/spyserver.cpp` | The ESP32 server speaks the published protocol; message/struct layouts necessarily match the public header. Server code is this project's own (MIT). |
| rtl_tcp protocol | Osmocom rtl-sdr (Steve Markgraf et al.) — GPL-2.0 | `firmware-esp32/streaming/src/main.cpp` | Protocol compatibility only (8-bit path for Android clients); no rtl-sdr code used. |
| AFSK-1200 demodulator and HDLC deframer | [LibAPRS](https://github.com/markqvist/LibAPRS) (Mark Qvist, GPL-3.0), itself derived from BertOS `afsk.c` (Develer S.r.l., GPL-2.0 with linking exception) | `firmware-esp32/streaming/src/aprs_rx.cpp` | The delay-multiply detector, bit PLL and the flag/abort/bit-stuffing window logic follow the LibAPRS/BertOS design, which was used as the reference during debugging (a bit-exact PC twin of this file was validated against it). The implementation here is a from-scratch fixed-point version for the FG23 I/Q path — no code was copied. |
| AX.25 / APRS framing, NRZI, X.25 FCS | AX.25 v2.2 specification; APRS protocol reference (Bob Bruninga, WB4APR) | `firmware-fg23/iq_capture/aprs_beacon.c`, `aprs_rx.cpp` | Own implementation from the specifications. |
| WSPR encoding | Joe Taylor, K1JT (WSJT-X project, GPL-3.0) — protocol only | `firmware-fg23/iq_capture/wspr_encode.c` | Implemented from the published protocol description; cross-validated bit-for-bit against the SM0YSR Python reference encoder on the host (not included). |
| CW trainer concept | N7HPR | `firmware-fg23/cw_trainer/` | The training method is N7HPR's idea; firmware and hardware design by HA7DCD. |

## Libraries and tools (used, not modified)

LovyanGFX (lovyan03, FreeBSD licence) — TFT driver · U8g2 (olikraus, BSD-2-Clause) — OLED spectrum · Arduino-ESP32 / pioarduino platform-espressif32 (LGPL-2.1 / Apache-2.0) and ESP-IDF, lwIP (Apache-2.0, BSD) · Simplicity SDK and RAIL (Silicon Labs, MSLA/Zlib, public API only) · KiCad · SDR++, GNU Radio, SoapySDR, GQRX — integration targets · [Direwolf](https://github.com/wb2osz/direwolf) (WB2OSZ) — used as the reference decoder in APRS tests · CMU Flite — used to generate `voice_clip.h` · Python: NumPy, SciPy, Matplotlib, pySerial · Signal Hound BB60C/VSG60, HackRF — test instruments.

## People

Special thanks to **[Z2Labs](https://www.z2labs.io)** for their help, ideas and the shared thinking throughout this project.

## Trademarks

EFR32, Simplicity Studio and RAIL are trademarks of Silicon Laboratories Inc. SpyServer and Airspy are trademarks of Airspy. Ionos SDR is not affiliated with IONOS SE.

If you believe something is missing or misattributed here, please open an issue — it will be corrected.

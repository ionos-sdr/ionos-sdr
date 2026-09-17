# EFR32FG23 radio firmware (MIT)

Built with Simplicity Studio (v2025.6) / Simplicity SDK, RAIL 2.19.x, GCC. Project base: `rail_soc_empty` + "RAIL Utility, Initialization"; `SL_BOARD_ENABLE_VCOM` must be 1 for the UART.

Provides: I/Q capture and streaming over I²S/SPI to the companion, NCO-AFSK APRS/AX.25 TX, CW TX with ramped envelope, NBFM TX, WSPR encoder, multi-point frequency calibration, RSSI, diagnostic CLI (`k` benchmark, `Z` RSSI, `y` NCO tone, `g` pin test, …). See `docs/cli.md`.

Vendor library policy: RAIL is used only through its public API. See CONTRIBUTING.md.

## Sub-projects

- `iq_capture/` — main firmware: I/Q streaming, NCO-AFSK APRS TX, NBFM/CW TX, WSPR encoder, scan, calibration CLI.
- `cw_trainer/` — CW trainer TX (concept by N7HPR), OLED UI.
- `railtest_wspr/` — RAILtest-based WSPR experiments.

Each is a Simplicity Studio project: import the `.slcp`, let the SDK regenerate `autogen/`, then copy `station_config.example.h` → `station_config.h`. `music_clip.h` / `voice_clip.h` (audio test clips referenced by the NBFM TX test) are intentionally not in the repository — generate your own 8 kHz int8 clip; the encoding is documented in `app.c`.

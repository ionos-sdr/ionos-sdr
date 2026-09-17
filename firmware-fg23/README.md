# EFR32FG23 radio firmware (MIT)

Built with Simplicity Studio (v2025.6) / Simplicity SDK, RAIL 2.19.x, GCC. Project base: `rail_soc_empty` + "RAIL Utility, Initialization"; `SL_BOARD_ENABLE_VCOM` must be 1 for the UART.

Provides: I/Q capture and streaming over I²S/SPI to the companion, NCO-AFSK APRS/AX.25 TX, CW TX with ramped envelope, NBFM TX, WSPR encoder, multi-point frequency calibration, RSSI, diagnostic CLI (`k` benchmark, `Z` RSSI, `y` NCO tone, `g` pin test, …). See `docs/cli.md`.

Vendor library policy: RAIL is used only through its public API. See CONTRIBUTING.md.

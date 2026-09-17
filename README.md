# Ionos SDR

**An open, calibrated, remotely usable SDR transceiver built on a commodity ISM radio SoC.**

Ionos SDR turns a mass-produced IoT radio chip — the Silicon Labs EFR32FG23 — into an inspectable, hackable, self-hostable software-defined radio transceiver (RX + TX, 110–223 MHz today, 902 MHz planned). An ESP32-S3 companion streams 16-bit I/Q over WiFi or USB using open, documented protocols, so standard SDR clients (SDR++, GNU Radio via SoapySDR, GQRX, CubicSDR) can use it locally or remotely — no vendor software, no cloud.

> Status: **working prototype**, not a product. Everything below that says "validated" has a measurement behind it in [`measurements/`](measurements/). Everything that says "planned" is not done yet.

## What works today

| Area | State |
|---|---|
| RX I/Q streaming to SDR++ over WiFi (SpyServer-compatible) | validated, ~175 ksps int16 sustained (~712 kB/s WiFi) |
| RX I/Q over USB CDC | ~250 ksps int16 |
| APRS (AX.25 / AFSK 1200) beacon TX | validated on-air, decoded on APRS-IS via a public iGate |
| NBFM voice TX | validated on a handheld receiver |
| CW TX with envelope shaping | validated |
| WSPR encoder | bit-exact against an independent Python reference (3 vectors) |
| On-device FFT waterfall on a 2.8" TFT | working (256–1024 pt, Nuttall, EMA, background subtraction) |
| Frequency calibration | multi-point, TCXO-referenced; NCO tick measured at 4.649 Hz (39 MHz / 2²³) |
| GPS (u-blox M8) time + locator | working |

## What is planned (and what a grant would fund)

1. **Open hardware** — 6-layer KiCad design of the FG23 radio board + ESP32-S3 carrier with LNA, switchable-drain LDMOS PA, VCTCXO; fabrication files, BOM, assembly and calibration procedure. Currently running on a vendor devkit (BRD4265B) plus a custom carrier.
2. **Open I/Q streaming protocol** — written spec of a native device-to-host mode with timestamps and metadata (SpyServer mode kept for compatibility); MIT C core (`libionos`), GPLv3 SDR++ source module, SoapySDR driver.
3. **On-device DSP plugin contract** — fixed I/Q block interface + registration struct so a new demodulator is ~100 lines of C.
4. **Open measurement methodology** — every published figure reproducible from scripts and raw data in `measurements/`; RSSI→dBm and TX power calibration; pre-compliance procedure.
5. **Remote ground-station use case** — SatNOGS-compatible client (TLE, Doppler NCO, SiDS upload, store-and-forward).

## Repository layout

```
hardware/        KiCad sources, fabrication outputs, BOM        (CERN-OHL-P-2.0)
firmware-fg23/   EFR32FG23 radio firmware (Simplicity SDK / RAIL, GCC)   (MIT)
firmware-esp32/  ESP32-S3 companion: WiFi/USB streaming, TFT, GPS (PlatformIO) (MIT)
host/libionos/   Host-side C core library                       (MIT)
host/soapy/      SoapySDR driver                                 (MIT)
host/sdrpp-source/  SDR++ source module                          (GPL-3.0)
measurements/    Raw captures, scripts, notebooks, results       (CC-BY-4.0)
docs/            Architecture, protocol spec, calibration, guides (CC-BY-4.0)
```

## Key design facts (measured)

- The FG23 has no I/Q DAC. TX is synthesised by stepping the PLL frequency offset (`RAIL_SetFreqOffset`): tick = 4.649 Hz, phase-continuous, ~3.4 µs call latency, so ~290 kHz theoretical update rate. AFSK at 38.4 kS/s: THD 2.1 %, spurs < −41 dBc. Consequence: constant-envelope modes only, on-chip.
- Calibration-offset register has an intentionally disabled LSB (effective 9.3 Hz steps); a duty-cycled "pair-PWM" between adjacent ticks restores resolution.
- I/Q format: `int16` little-endian, **Q first** (`struct { int16_t q, i; }`) — same as geckokapula.
- Zero-copy FIFO reads (`RAIL_ReadRxFifo(rail, NULL, n)`) cut CPU load from 75 % to 11 % at 1 Msps.
- I²S sample rate is 39 MHz / N; measure, don't assume (1000 ksps @ 270 kHz BW, 400 ksps @ 100 kHz, 70 ksps @ 50 kHz).
- A DC-block between bias-tee and the FG23 matching network is mandatory (shunt inductor); a firmware TX interlock protects the LNA.

## Related projects and credits

- [geckokapula](https://github.com/tejeez/geckokapula) by OH2EAT (MIT) — the EFR32 SDR transceiver that proved the concept; Ionos SDR builds on its I/Q format and DSP structure with attribution.
- SDR++, SoapySDR, GNU Radio, SatNOGS — integration targets.
- Silicon Labs RAIL — vendor radio library, used through its public API only. All PHY-level work here is clean-room from public headers, documentation and our own measurements.

## Licensing

Firmware and host code: MIT (except `host/sdrpp-source/`: GPL-3.0, as required by the SDR++ module API). Hardware: CERN-OHL-P-2.0. Documentation and measurement data: CC-BY-4.0. See the `LICENSE` file in each directory.

## Trademarks

"Ionos SDR" is an independent open-hardware project and is not affiliated with, endorsed by, or related to IONOS SE or Silicon Laboratories Inc. EFR32 and Simplicity Studio are trademarks of Silicon Laboratories.

## Author

Zoltan Doczi, HA7DCD — RF/hardware engineer, Budapest. Co-designer of the RTL-SDR Blog V3, co-creator of KrakenSDR, designer of the TAPR QRPi.

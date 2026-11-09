# ionos-sdr-mini — USB throughput, RF characterisation, SoapyIonos and wideband sweep

**Measurement and development report, 2–9 November 2026** · HA7DCD

Board: ionos-sdr-mini rev-B3 2L · radio board: Silicon Labs BRD4265B (EFR32FG23, **434 MHz match**, 10 dBm) · host MCU: ESP32-S3 (N16R8)
Branches: `board/ionos-mini-idf` (firmware), `host/soapy-ionos` (SoapySDR driver)
Raw data, scripts and logs: [`measurements/ionos-mini-2026-11/`](../measurements/ionos-mini-2026-11/) · Hungarian version: [`docs/hu/ionos-mini-meresi-jelentes-2026-11.md`](hu/ionos-mini-meresi-jelentes-2026-11.md)

---

## Contents

1. [Summary](#1-summary)
2. [Time on test](#2-time-on-test)
3. [System overview](#3-system-overview)
4. [200 ksps 16-bit I/Q over USB — finding the bottleneck](#4-200-ksps-16-bit-iq-over-usb--finding-the-bottleneck)
5. [Long USB soaks and the rare single-block loss](#5-long-usb-soaks-and-the-rare-single-block-loss)
6. [Unexpected finding: the mini runs the 2 m PHY](#6-unexpected-finding-the-mini-runs-the-2-m-phy)
7. [48-hour RF characterisation at 145 MHz](#7-48-hour-rf-characterisation-at-145-mhz)
8. [SoapyIonos — SoapySDR driver (HOST-M3)](#8-soapyionos--soapysdr-driver-host-m3)
9. [Wideband sweep](#9-wideband-sweep)
10. [Re-check of suspicious points](#10-re-check-of-suspicious-points)
11. [Measurement infrastructure — lessons](#11-measurement-infrastructure--lessons)
12. [Commits](#12-commits)
13. [Open items](#13-open-items)
14. [Appendix: commands, files, reproduction](#14-appendix-commands-files-reproduction)

---

## 1. Summary

| Area | Result |
|---|---|
| **USB bandwidth** | 200 ksps × 16-bit I/Q (813 kB/s) streams continuously through a 4-deep hub chain with TinyUSB CDC on the USB-OTG port. The device side is not the bottleneck; the Windows host occasionally stops polling. |
| **USB reliability** | 86 M blocks in 40 h with **0.65 ppm** loss (excluding two known external events). A 2 MB PSRAM ring absorbs host stalls. |
| **Radio configuration** | The 434 MHz board was running the **145 MHz** FG23 PHY; at 434 MHz it only heard via the 3rd LO harmonic. RF characterisation therefore at 145 MHz. |
| **Gain / linearity** | dBFS = P_in + 53.68 dB, linear within ±0.1 dB from −125 to −56 dBm; clipping at ~−52 dBm. |
| **Noise / sensitivity** | −165.4 dBm/Hz → **NF ≈ 8.6 dB**; MDS (10 dB SNR / 500 Hz) **≈ −128 dBm** (434 MHz match, at 145 MHz). |
| **Passband** | 200 ksps: 88 kHz −3 dB bandwidth, set by the PHY's ~100 kHz channel filter. Image rejection ≥ 42 dB. |
| **Frequency** | `F` tuning: −11 ppm (crystal correction over-corrects); uncorrected +4.2…4.4 ppm; diurnal wander 1.9 ppm p-p. |
| **SoapyIonos** | Runs on Windows (GQRX, SoapySDRUtil, Python). Two Windows-specific bugs fixed. 3.25 M blocks during the sweeps, 4 lost. |
| **Wideband** | With the 145 MHz PHY divider the radio tunes **128.34–198.07 MHz**; outside, `RAIL_ConfigChannels` hangs (recovered by SWD reset). CW: NF 7.9–11.9 dB, MDS −129…−125 dBm; 2-FSK 5 kbps BER 10⁻²: median −110 dBm. |
| **Re-check** | 169 MHz FSK outlier was a one-off; 136.5/156/175.5/195 MHz carry broadband interference; ±6.25 kHz sidebands come from the receiver I/Q path. |

## 2. Time on test

| Period (local time) | Test | Hours |
|---|---|---|
| 11-02 | short USB soaks, HWCDC, 50 ksps (15 + 10 + 10 min) | 0.6 |
| 11-03 22:21 → 11-04 06:00 | night soak, HWCDC, 50 ksps | 7.6 |
| 11-04 | 200 ksps TinyUSB, 256 kB ring: 30 min + 3 h | 3.5 |
| 11-04 21:52 → 11-05 07:00 | 200 ksps, 2 MB ring, night | 9.1 |
| 11-06 00:22 → 07:00 | 200 ksps, 2 MB ring, night 2 | 6.6 |
| 11-06 11:48 → 16:47 | 200 ksps, WIFIOFF A/B (daytime) | 5.0 |
| 11-06 17:42 → 11-07 02:12 | RF characterisation, run 1 (VSG) | 8.5 |
| 11-07 02:12 → 11-08 17:50 | **RF characterisation, 126 cycles (VSG)** | 39.6 |
| 11-08 18:04 → 23:06 | wideband sweep via SoapyIonos (VSG) | 5.0 |
| 11-08 23:49, 11-09 09:01 → 09:22 | re-check of suspicious points (VSG) | 0.4 |
| **Total** | | **≈ 86 h** (of which ≈ 54 h RF with signal generator, ≈ 32 h USB-only) |

## 3. System overview

```
 antenna/VSG ──► BRD4265B (EFR32FG23)            ESP32-S3                   PC (Windows)
                 RAIL RX, IQ FIFO                SPI slave                  usbser.sys
                 CIC decimation (R = 2…64) ─SPI─► PSRAM ring 2 MB ──USB──►   websdr / SoapyIonos /
                 1040-byte blocks                TinyUSB CDC-ACM            Python measurement scripts
                 (IQB2 / SPC1 / MSG1)            cmdlink (UART) ◄── host commands
```

- **FG23** (`firmware-fg23/iq_capture`): RAIL RX in I/Q mode (`RX_IQDATA_FILTLSB`, FIFO), two-stage CIC; R = 2/4 direct first stage, R ≥ 8 in multiples of 8. Core 78 MHz (HFRCODPLL), SPI 19.5 MHz (R < 8) / 4 MHz.
- **ESP32-S3** (`firmware-esp32/streaming`, env `ionos_mini`): blocks from SPI, TinyUSB CDC on native USB-OTG (vendored `esp_tinyusb` 1.7.6), host API (`usb_host.cpp`), console on the CH343 UART bridge.
- **Frames**: 1040 bytes, magic `IQB2` (I/Q, uint32 LE sequence at byte 4), `SPC1` (scan spectrum), `MSG1` (text: hello / reply / status). See [`ionos-usb-protocol.md`](ionos-usb-protocol.md).
- **Hub chain on the test PC**: Intel xHCI → HP hub 03F0:036B → 3× Realtek RTS5411 → ESP native USB (VID 303A, PID 4001).
- **Rates** (fs_in = 400 kHz): R = 2 → 200 ksps (781 blk/s, 813 kB/s), R = 4 → 100 ksps (391 blk/s), R = 8 → 50 ksps (195 blk/s).

## 4. 200 ksps 16-bit I/Q over USB — finding the bottleneck

### 4.1 Starting point
- On the HWCDC (USB-Serial/JTAG) path an apparent ~776 kB/s ceiling was seen. **It turned out to be the Python frame parser** (`buf = buf[1040:]` per block → quadratic cost at 781 blk/s). Offset-based parsing removed it.
- Decision: **keep 16-bit I/Q** rather than drop to 8 bits, and use what USB full-speed can actually deliver.

### 4.2 TinyUSB CDC on USB-OTG
- Vendored `esp_tinyusb` 1.7.6 (`third_party/esp_tinyusb`, override path); the Kconfig symbol is renamed (`IONOS_TUSB_CDC_ENABLED`) because the Arduino core compiles its own `USBCDC.cpp` when the original symbol is set.
- `CFG_TUD_CDC_TX_EPSIZE 4096` (at 16384 the CDC stops completely), TX FIFO 16 kB, DMA.
- `TusbCdc` class: PSRAM ring + pump task (core 0, prio 6), woken by `tud_cdc_tx_complete_cb`.

### 4.3 Experiments (console commands `US`, `UF<s>`, `UP<prio>`, `UT<words>`)
| Experiment | Result |
|---|---|
| Flood (dummy data, as fast as the bus takes it) | ~205 × 4 kB transfers/s, 3.5–5 ms each → **870–950 kB/s sustained** (~14–15 packets per frame of the theoretical 19; USB FS theory 1216 kB/s) |
| TinyUSB task priority 5 vs 24 | no difference |
| FG23 stream on/off (CPU load) | no difference |
| EP2 hardware TX FIFO 32 / 64 / 128 words (`UT` at runtime) | no difference |
| Double-buffered EP2 | no difference |
| **Gap probe** (registers sampled when > 10 ms without a transfer) | transfer armed (DIEPCTL2 EPENA), TX FIFO **full**, no device NAK, SOFs present → **the host is not polling** |

**Conclusion:** the device side is not the bottleneck. Margin is ~7–15 % (870–950 vs 812.5 kB/s). Host pauses: ~20 ms every ~1.5 s, occasionally several hundred ms.

### 4.4 TX ring size
- 256 kB (~320 ms) is not enough: a 3-hour soak had two > 320 ms Windows read stalls → device-side drops (+157, +107 blocks).
- **2 MB in PSRAM (~2.5 s)**: no device-side drop since (`623697e`).

## 5. Long USB soaks and the rare single-block loss

### 5.1 Results
| Run | Duration | Blocks | ESP drops | Host-side loss |
|---|---|---|---|---|
| 30 min (256 kB) | 0.5 h | 1.36 M | 0 | 0 |
| 3 h (256 kB) | 3 h | 8.44 M | 264 (2 host stalls) | +4 single-block |
| Night (2 MB) | 9.1 h | 25.65 M | **0** | 19 (0.7 ppm) |
| Night 2 (2 MB) | 6.6 h | 18.6 M | **0** | 15 (0.8 ppm) |
| RF run, 40 h (2 MB, WiFi off) | 39.6 h | 86.0 M | **0** | 56 (0.65 ppm) excl. known events |

### 5.2 Anatomy of the single-block loss
- Every time **exactly 128 bytes (2 × 64-byte USB packets)** are missing from one block while the ESP drops nothing → packets are lost between the device and the application.
- Most fall on a **10-minute grid** (e.g. 22:41:22, 23:11:23, 01:11:24, 02:51:24 …, drifting ~1 s per hour).
- Ruled out: a 10-minute scheduled task (none), ESP flash logging (disabled in the `ionos_mini` build), process starts (no correlation in a process watcher).
- Much more frequent in the daytime (17 in 16 min) and correlated with host activity (process starts, VSG software, browser) → **Windows-side latency spikes** are the likely cause.
- The `WIFIOFF` A/B test was inconclusive in daytime; during the 40 h RF run (WiFi off) the loss was 0.65 ppm.

## 6. Unexpected finding: the mini runs the 2 m PHY

- VSG tone at 433.845 MHz, −10…−90 dBm, through a 30 dB pad: **nothing seen**.
- VSG sweep 425–445 MHz in 100 kHz steps: signal **only at 435.2 MHz**, at 0 dBFS (−50 dBm).
- Cause: the flashed FG23 radio config is `base_frequency 144800000`, 25 kHz, 801 channels (144.8–164.8 MHz), `TUNE_BASE_KHZ 144800`. `F433825` requests an unreachable channel → the FG23 drops it, **yet the ESP replies OK**. The radio sat near 145.07 MHz and heard the 434-matched input via the **3rd LO harmonic** (3 × 145.07 ≈ 435.2 MHz).
- The RX match is forgiving (a few dB LNA degradation), so the characterisation ran **at 145 MHz**.

## 7. 48-hour RF characterisation at 145 MHz

### 7.1 Setup
- **Signal Hound VSG60** (SCPI on port **5124**) → **30 dB attenuator** → 1.5 m coax → BRD4265B SMA. Level at the board = VSG − 30.5 dB (coax ~0.5 dB not subtracted).
- CW, modulation off (re-enabled after the run).
- Script `scripts/longprog.py` with a background reader thread. One cycle (~19 min):
  1. frequency offset (tone +20 kHz, −90 dBm);
  2. passband + image at 200/100/50 ksps (2/1/0.5 kHz steps, −90 dBm);
  3. level staircase −40…−150 dBm in 5 dB steps, 200 ksps;
  4. USB counters;
  5. 10 drift points ~40 s apart.
- Runs: 11-06 17:42 → 00:00 (20 cycles), interrupted by a scheduler mishap (§11), then **11-07 02:12 → 11-08 17:50: 126 cycles, 0 errors, 86.0 M blocks**.

### 7.2 Gain, linearity, clipping

| P_in (at board) [dBm] | Tone [dBFS] | Gain [dB] | max\|z\| |
|---|---|---|---|
| −150.5 | −85.4 (noise-limited) | – | 0.006 |
| −130.5 | −76.2 | 54.3 (noise-biased) | 0.006 |
| −120.5 | −66.71 | 53.79 | 0.006 |
| −110.5 | −56.77 | 53.73 | 0.007 |
| −100.5 | −46.77 | 53.73 | 0.010 |
| −90.5 | −36.86 | 53.64 | 0.019 |
| −80.5 | −26.83 | 53.67 | 0.051 |
| −70.5 | −16.82 | 53.68 | 0.149 |
| −60.5 | −6.84 | 53.66 | 0.460 |
| −55.5 | −1.84 | 53.66 | 0.814 |
| −50.5 | +1.19 | 51.69 | **1.299 (clipping)** |
| −40.5 | +1.52 | 42.02 | 1.369 |

- **Gain 53.68 dB** (−125…−56 dBm, 1638 points, σ = 0.10 dB). Cycle-to-cycle at −90 dBm: σ = 0.03 dB — the signal path did not move in two days.
- **Clipping at ~−52 dBm** at the board.

### 7.3 Noise and sensitivity
- Noise density (200 ksps): **−111.7 dBFS/Hz** → −165.4 dBm/Hz at the input → **NF ≈ 8.6 dB** (434 MHz match, at 145 MHz, including the coax).
- **MDS** (10 dB SNR in 500 Hz): **≈ −128 dBm**.
- Dynamic range ~86 dB / 500 Hz, ~60 dB / 200 kHz.
- At 100 and 50 ksps the noise density is 2.6–2.9 dB worse (−108.8 / −109.1 dBFS/Hz): noise beyond the wide PHY channel filter folds back in the CIC decimation.

### 7.4 Passband and image rejection (median of 126 cycles)

| Rate | −1 dB | −3 dB | −6 dB | −20 dB | at 0.45·fs | Image (\|f\| ≤ 0.3·fs) | In-band ripple |
|---|---|---|---|---|---|---|---|
| 200 ksps | 74 kHz | **88 kHz** | 100 kHz | 128 kHz | −80 dB | **≥ 42.3 dB** | (edge already rolling off) |
| 100 ksps | 27 kHz | 59 kHz | 81 kHz | 98 kHz | −8 dB | ≥ 45.9 dB | 2.7 dB p-p (±25 kHz) |
| 50 ksps | 16.5 kHz | 28 kHz | 41 kHz | 49 kHz | −7 dB | ≥ 48.6 dB | 2.5 dB p-p (±12.5 kHz) |

(Widths are full bandwidths between the two −x dB points.)

- At 200 ksps the band is set by the **PHY channel filter (~100 kHz)**, not the CIC → ~90 kHz of the 200 ksps is usable.
- Image rejection is noise-limited (real value may be better).
- **Spurs**: sidebands at ±6.25 kHz around the tone — investigated in §10.

### 7.5 Frequency and drift
- Offset (`F145000`, `o0`): **mean −1596 Hz = −11 ppm**, range −1464 … −1741 Hz (**277 Hz = 1.9 ppm p-p**), σ = 66 Hz, 1376 points.
- Clean diurnal cycle: most negative before dawn (coldest), least negative early afternoon; both days repeat.
- The wideband `B` mode (no crystal correction) shows **+4.2…4.4 ppm** → **the −11 ppm is an over-correction by the firmware** (`FREQ_CORR_PPB` / tick scale).

### 7.6 USB during the 40 h
- 86.0 M blocks: 19 660 are the sequence jump at reconnect (new ESP firmware flushes the ring on connect), 591 during a full-disk scan on the host (multi-second read stall). **Remaining: 56 blocks = 0.65 ppm**, zero for hours at night.

## 8. SoapyIonos — SoapySDR driver (HOST-M3)

### 8.1 Development
- Written by a cloud coding agent from the protocol spec and the firmware, tested on Linux against a pty mock device: 8/8 tests; zero-fill and resync restore exactly as many blocks as the mock dropped.
- Branch `host/soapy-ionos`, folder `host/soapy-ionos/` (CMake, `src/IonosProtocol|Serial|Device|Registration`, `test/`), MIT.

### 8.2 Features
- Discovery by VID/PID + hello; port DTR = 1 / RTS = 0.
- Frequency range, rates (`decim_list`), gain from `CAP?` / hello.
- CS16 / CF32 / CS8. Reader thread, 64 kB reads, offset-based framing.
- Lost block → zero-filled + `SOAPY_SDR_OVERFLOW`; truncated block (128 B missing) → resync; sequence going backwards → `stream_restarts`.
- Sensors: `lost_blocks`, `gap_events`, `truncated_blocks`, `resyncs`, `stream_restarts`, `host_overflows`, `blocks`, `rssi`, `dev_*`.
- `command` setting: raw host command (TX/diagnostic letters blocked).

### 8.3 Windows fixes (on real hardware)
| Commit | Bug | Fix | Result |
|---|---|---|---|
| `5847630` | 400 lost blocks in 30 s at 200 ksps | `ReadIntervalTimeout = 0`, 20 ms / 64 kB reads, no sleep on Windows, `SetupComm` 4 MB | 60 s, 46 901 blocks, **0** lost |
| `dbae918` | Under SoapySDRUtil/PothosSDR the board was seen as protocol "v1" | reader thread stuck on the first `std::mutex` lock (VS 2022 17.10+ constexpr mutex + older `msvcp140.dll` in the host app) → `_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR` | `--find` / `--probe` fine, v2 protocol |

### 8.4 Practical tests
- **GQRX** (radioconda): `soapy=0,driver=ionos`, 200 kS/s, live waterfall, 0 lost. Horizontal stripes on retune are FG23 stream restarts (every `FH` restarts the stream, sequence restarts at 0) → seamless retune is on the list.
- **Wideband sweep through the driver**: 2.98 M blocks, 2385 stream restarts, 4 lost, 1 truncated, 0 host overflows. Re-check: 275 k blocks, 46 restarts, 0 lost.
- SDR++: the current Windows build has no `soapy_source` (only legacy `_OLD` variants).

### 8.5 Spec/firmware mismatches found by the driver work
1. VID/PID: spec says 0x1001, the TinyUSB build uses 0x4001 (esp_tinyusb default).
2. **Serial number: every board reports "123456"**; no MAC in hello → boards are indistinguishable.
3. Hello: `gain_shift_max=0`, extra `mode=` key.
4. Status: `usbdrop` (spec) vs `usb_drop` (firmware).
5. Resync rule ambiguous (16 B alignment vs 4 B step); the 128 B truncated block is not covered.
6. **Sequence restarts at 0 on every F/FH/R** — not in the spec.
7. **`FH` replies OK before retuning**; no marker for where the new frequency starts.
8. No `IQ ON/OFF` — `i` only toggles.
9. Fast reopen can miss the hello → the driver sends `CAP?` after 150 ms.
10. `s_host_spoke` not reset (fixed in `487569e`).
11. "overflow flag" vs SoapySDR return code.
12. `MODE USB` wording: I/Q goes to USB in both modes; the mode only switches the demodulators.

## 9. Wideband sweep

### 9.1 Goal
CW and FSK sensitivity versus frequency with the 434 MHz match, including out of band, and probing the tuning limits at many points.

### 9.2 New FG23 command `B<kHz>`
- A RAM copy of the PHY's first channel entry with a new base frequency (one channel, fresh IR-cal attributes) → `RAIL_ConfigChannels()`; `B0` restores the generated PHY, `F` drops the override first.
- **Bug 1 – zero blocks:** `RAIL_ConfigChannels()` re-applies the PHY register set including packet mode → the I/Q data source (`RX_IQDATA_FILTLSB`, FIFO) must be configured again afterwards.
- **Bug 2 – hang:** at a base frequency the PHY's synthesiser divider cannot reach, `RAIL_ConfigChannels()` **never returns**. Found with RAM breadcrumbs read over SWD (`g_b_dbg[]`, `FGRD 200010d4 6` → step 3).
- **Recovery:** `FGINFO` does not reset the FG23 → new ESP console command **`FGRST`** (SWD `AIRCR.SYSRESETREQ`), then `FGINFO`; streaming again after ~5 s. The script does this at every point without data.

### 9.3 Method
`scripts/wideband_soapy.py` **through the SoapyIonos driver** (doubling as a long driver test); `scripts/wideband.py` is the direct-serial fallback.
1. **Map**: 1 MHz steps 145 → 1100 MHz, then 144 → 50 MHz; VSG −60 dBm (−90.5 dBm at the board), tone +20 kHz, 200 ksps; "receives" if tone SNR > 25 dB/bin. Edges bisected to 25 kHz.
2. **CW**: 250 kHz grid over the working range, 50 ksps, −100.5 dBm: gain, noise density, NF estimate, MDS (10 dB / 500 Hz).
3. **FSK**: 2-FSK, 5 kbps, ±2.5 kHz, PRBS9 (VSG Arb, 100 kS/s, `scripts/fsk_lib.py`), discriminator demodulator + PRBS-aligned BER; bisection for the BER = 10⁻² level, 1 MHz grid.

### 9.4 Results (11-08 18:20 → 23:06)

**Tuning map (1051 points, 50–1100 MHz):**
- **Receive range with the 145 MHz PHY divider: 128.34 … 198.07 MHz** (lower edge 128.328–128.344, upper edge 198.062–198.078 MHz).
- Next to the edges (125–128 and 199–203 MHz) the stream runs but the synthesiser does not lock (no tone).
- Every other point hangs `RAIL_ConfigChannels()` → ~950 `FGRST`, all successful.
- This is **the PHY's VCO-divider range, not the chip limit**; the 434/868/915 MHz bands need a PHY with a different divider.

**CW (277 points, 250 kHz grid, 50 ksps, −100.5 dBm):**

| f [MHz] | Tone [dBFS] | Freq. error [Hz] | NF [dB] | MDS [dBm] (10 dB / 500 Hz) |
|---|---|---|---|---|
| 129 | −45.37 | +540 | **7.9** | **−129.1** |
| 130 | −45.39 | +536 | 8.4 | −128.6 |
| 140 | −45.89 | +589 | 8.8 | −128.2 |
| 150 | −46.37 | +626 | 9.7 | −127.3 |
| 160 | −47.04 | +676 | 11.6 | −125.4 |
| 170 | −47.69 | +709 | 9.9 | −127.1 |
| 180 | −48.47 | +761 | 10.2 | −126.9 |
| 190 | −49.48 | +793 | 11.1 | −125.9 |
| 198 | −50.33 | +827 | 11.9 | −125.1 |

- Gain (50 ksps): 55.1 dB → 50.2 dB across the range (~5 dB slope). NF median 10.1 dB.
- At some points (133.75; 136.5; 144.25; 152.25; 156; 160.25; 163.75; 172.75; 175.5; 183.25; 191.25; 195 MHz) the NF estimate was 14–29 dB — see §10.
- Uncorrected frequency error **+4.2 ppm**, proportional to frequency → the real crystal error.

**FSK (70 points, 1 MHz grid; 2-FSK 5 kbps, ±2.5 kHz, PRBS9; level for BER = 10⁻²):**

| f [MHz] | 129 | 135 | 140 | 145 | 150 | 155 | 160 | 165 | 170 | 175 | 180 | 185 | 190 | 195 | 198 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| level [dBm] | −111.9 | −111.4 | −111.4 | −110.9 | −110.5 | −110.5 | −106.2 | −108.6 | −108.6 | −110.5 | −108.6 | −109.1 | −109.1 | −104.4 | −108.1 |

- **Median −110.0 dBm**, best −111.9 dBm (129 MHz); 2–3 dB worse towards the top, like CW.
- Outliers: 169 MHz (−83.3 dBm), 156 (−102.0), 195 (−104.4), 160 (−106.2) — see §10.
- Reference: non-coherent 2-FSK, BER 10⁻² at Eb/N0 ≈ 9 dB → −174 + NF(~9) + 10·log10(5000) + 9 ≈ −119 dBm. The measured −110…−112 dBm is 7–9 dB worse, typical of the simple discriminator demodulator (no matched filter, 50 kHz noise bandwidth) — it rates the reference demodulator, not the receiver.

## 10. Re-check of suspicious points

`scripts/recheck.py`, 11-08 23:49 (noise part) and 11-09 09:01–09:22 (sidebands + FSK). Data `data/wideband/sp_rc_*`.

### 10.1 Noise spectrum with the VSG off (200 ksps, 16 points)

| f [MHz] | Noise density [dBFS/Hz] | Note |
|---|---|---|
| 129, 133.75, 144.25, 145, 152.25, 160, 160.25, 163.75, 169, 172.75, 183.25, 191.25 | −110.6 … −113.9 | clean |
| **136.5** | **−97.7** | ~14 dB raised, broadband |
| **156.0** | **−104.8** | ~7 dB raised, broadband |
| **175.5** | **−102.4** | ~9 dB raised, broadband |
| **195.0** | **−108.2** | ~4 dB raised, broadband |

→ Part of the "bad NF" points are **broadband interference** at 136.5 / 156 / 175.5 / 195 MHz (external, or an internal spur band — to be identified), not receiver degradation. The other suspicious points are clean with the VSG off; there the CW run caught momentary interference.

### 10.2 The ±6.25 kHz sidebands (145 MHz, `B145000`, 200 ksps)

| Tone offset | −6.25 kHz [dBc] | +6.25 kHz [dBc] | ±3.125 kHz [dBc] |
|---|---|---|---|
| +5 kHz | −73 … −74 | −74 … −75 | ≤ −77 (noise) |
| +20 kHz | **−50.7 … −50.8** | **−52.8** | ≤ −78 (noise) |
| +37 kHz | – | **−43.2** | – |

- **Level-independent** (same dBc from −70.5 to −90.5 dBm) → linear, not compression.
- **Strongly dependent on the tone's offset from centre** (5 kHz: −74, 20 kHz: −51, 37 kHz: −43 dBc) → not VSG AM/PM (that would be offset-independent) but a **periodic timing/phase error at 6.25 kHz = 400 kHz / 64 in the receiver I/Q path** — tied to 64-sample blocks of the FG23 input (FIFO read or CIC phase). To be fixed in the FG23 decimation/FIFO path.
- Anomaly: at the +37 kHz setting the strongest peak appeared at −27.9 kHz at full level — unexplained (alias or peak mis-identification).

### 10.3 FSK outliers re-measured (3× each)

| f [MHz] | BER 10⁻² level [dBm] (3 runs) | earlier |
|---|---|---|
| 129 | −112.4 / −112.1 / −112.1 | −111.9 |
| 145 | −111.0 / −110.7 / −111.0 | −110.9 |
| 155 / 157 | −110.2 … −110.5 | −110.5 |
| **156** | **−97.3 / −97.3 / −97.1** | −102.0 |
| 159 / 161 | −109.9 … −110.5 | – |
| **160** | −107.2 / −107.7 / −104.7 | −106.2 |
| 168 / 170 | −108.3 … −110.2 | −108.6 |
| **169** | **−109.9 / −110.2 / −110.2** | **−83.3** |
| 194 / 196 | −108.6 … −108.8 | – |
| **195** | −103.4 / −104.7 / −105.0 | −104.4 |

- **The 169 MHz −83 dBm was a one-off** (now −110 dBm like its neighbours) — likely a VSG Arb dropout or transient interference.
- **156 and 195 MHz are persistently worse**, exactly where the noise spectrum shows broadband interference.
- Repeatability ±0.3 dB at clean points.

## 11. Measurement infrastructure — lessons

1. **`schtasks /sc once /st 23:59` + immediate `/run`: the 23:59 trigger really fires.** All helper tasks of the day restarted at 23:59 (ESP reflash, an old soak grabbed the COM port) → ~2 h lost. Now: `/change /disable` immediately after `/run`.
2. **Stale I/Q**: if the script does not read while the VSG is being changed, the 2 MB ring fills and the measurement sees the *previous* VSG state (~2.5 s late). Fix: a background reader that drains continuously.
3. **The VSG60 output follows a command 1–1.5 s later** (frequency and level) → 1.6 s settle per step.
4. **VSG SCPI is on port 5124** (not 5024); modulation must be off for CW.
5. **Heavy disk I/O** (full-disk scan) can outlast even the 2 MB ring.
6. **After an FG23 flash** streaming only starts after `FGINFO` (SWD debug release).
7. **Unreachable PHY frequency → RAIL hang**; `FGINFO` does not reset, `FGRST` does.
8. On a localised (Hungarian) Windows, performance-counter names are translated → use CIM classes for host monitoring.
9. A script that stops on an exception can leave the VSG output on (it stayed on at −40 dBm overnight once) → switch the output off in a `finally` block.

## 12. Commits

**`board/ionos-mini-idf`:**
| Commit | Content |
|---|---|
| `626c2e6` | I/Q 100 / 200 ksps (R = 4 / 2): FG23 and SPI link |
| `65aecee` | Native USB-OTG + TinyUSB CDC, 200 ksps to the host |
| `acc3824` | TinyUSB diagnostics (`US`/`UF`/`UP`/`UT`, gap probe), 200 ksps lossless |
| `623697e` | TinyUSB TX ring 2 MB in PSRAM |
| `487569e` | FG23 `B<kHz>`, I/Q data config after `RAIL_ConfigChannels`, breadcrumbs; ESP `B` forwarding, `FGRST`, `WIFIOFF`/`WIFION`, flush on connect |

**`host/soapy-ionos`:** `2eb2b2c` (driver), `5847630` (Windows read), `dbae918` (MSVC mutex).

## 13. Open items

**FG23 firmware**
1. **430–440 MHz PHY** (base 430.000 MHz, 25 kHz, 401 channels, `TUNE_BASE_KHZ 430000`) and a **wider channel filter** (~200 kHz) to use all of 200 ksps.
2. **Fix the crystal correction**: −11 ppm over-correction; real error +4.2…4.4 ppm.
3. **Seamless retune**: sequence continues, retune instant flagged in the header, samples zeroed while settling.
4. **Report tuning errors**: error on unreachable frequency; the ESP replies OK only after the FG23 ack.
5. **±6.25 kHz sidebands**: periodic timing error in the I/Q path (FIFO read / CIC phase).
6. `B` mode: validate the frequency to avoid the hang (or a RAIL-level timeout).

**ESP / protocol**
7. MAC-based USB serial number and `serial=` in hello.
8. `IQ ON`/`IQ OFF`; update the spec per §8.5.
9. In USB mode disable the SpyServer/WiFi stream; auto mode switch on host connect.

**Host**
10. SoapyIonos: SDR++ (build with soapy_source), GNU Radio; expose scan (SPC1).
11. websdr 200 k span live test.

**Measurement**
12. Repeat the wideband sweep with 434 / 868 / 915 MHz PHYs (different VCO dividers); identify the source of the 136.5 / 156 / 175.5 / 195 MHz interference.
13. Better FSK reference demodulator (matched filter) for the receiver's true FSK sensitivity.

## 14. Appendix: commands, files, reproduction

### 14.1 ESP console commands (CH343 UART, 115200, DTR/RTS = 0)
| Command | Effect |
|---|---|
| `US` | USB statistics (transfers, histogram, gap probe) |
| `UF<s>` | flood test |
| `UP<prio>` | TinyUSB task priority |
| `UT<words>` | resize EP2 TX FIFO |
| `WIFIOFF` / `WIFION` | WiFi off/on |
| `FGINFO` | FG23 SWD info (debug release) |
| `FGRD <hex> [n]` | read FG23 RAM/registers (use NS alias for peripherals) |
| `FGRST` | FG23 system reset over SWD |
| `FGFW …` | FG23 flash (used by `tools/fg23_flash.py`) |

### 14.2 Host API (CDC, line based)
`CAP?`, `VER?`, `FH<Hz>`, `R <sps>`, `MODE USB|STANDALONE`, `STAT <s>`; forwarded to the FG23: `F o f i W T t s B`.

### 14.3 Data (`measurements/ionos-mini-2026-11/`)
- `data/rf-145MHz-48h/`: `longprog_20261107_0212.log`, `lp_{drift,passband,level,usb}_20261107_0212.csv` (+ the first run `_20261106_1742`).
- `data/wideband/`: `soapyrun_20261108_1820.log`, `sp_wb_{map,edge,cw,fsk}_20261108_1820.csv`; re-check `sp_rc_*`, `soapyrun_20261108_2349.log`, `soapyrun_20261109_0901.log`.
- `data/usb-soak/`: `soak_full_*.log.gz` (per-second sequence statistics), `console_*.log.gz` (ESP console incl. `US` every 60 s).
- `scripts/`: see [`RESULT.md`](../measurements/ionos-mini-2026-11/RESULT.md).

### 14.4 Reproduction
1. ESP: `pio run -e ionos_mini -t upload`.
2. FG23: Simplicity Studio build → `python tools/fg23_flash.py <hex> --port <console COM>` → `FGINFO`.
3. SoapyIonos: `cmake -S host/soapy-ionos -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="C:\Program Files\PothosSDR"` → `cmake --build build --config Release` → `SOAPY_SDR_PLUGIN_PATH=build\Release`.
4. Measurement: Python with numpy, pyserial and the SoapySDR bindings (radioconda works); VSG60 software running with SCPI enabled (port 5124).

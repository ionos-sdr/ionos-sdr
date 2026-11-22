# EXPERIMENTAL — AIS decoded on the FG23 (branch `exp/fg23-ais-onchip`)

**Question:** is the EFR32FG23's own GFSK demodulator, with the AIS frame
decoding done on the FG23, more or less sensitive than the I/Q path (FG23
I/Q → SPI → ESP32 software demod, `ais_rx.c`)? Answer by alternating the two
on the same antenna every few minutes (`AISBENCH`).

## Data path

```
I/Q path (main):   FG23 RX_IQDATA_FILTLSB ──IQB2 blocks──▶ ESP32 ais_rx.c ──▶ emit()
on-chip path:      FG23 AIS PHY ─ sync word ─ raw line bits ─ ais_hw.c (NRZI/HDLC/CRC)
                        ──AIS1 blocks (aispkt.h)──▶ ESP32 ais_mode_hwblk() ──▶ emit()
emit():            console "AIS RX[HW]: …", NMEA TCP 10110, $PIONS, AISBENCH counters
```

The AIS1 block is the SPECLINE trick again: 1040 bytes, IQB2-framed, sent with
`iq_stream_ext_send()` (scan.c's path), so the SPI slave DMA, the reorder
buffer and the RDY handshake are untouched. Up to 6 frames per block, plus a
heartbeat every 10 s with the FG23-side counters (sync / ok / CRC bad / abort /
drop / overflow / idle-channel RSSI).

`$PIONS` gets two fields: `$PIONS,AIS,<chan>,<dBFS>,<SNR>,<Hz>,<IQ|HW>,<dBm>`.
For HW frames the dBm is RAIL's RSSI at the sync word, the SNR is that minus
the idle-channel RSSI of the last heartbeat, and the dBFS field is the dBm
mapped back with `AIS_DBM_K` (−52.5), so the old 6-field readers still work.

## Why the packet handler only finds the sync word

AIS is HDLC: NRZI, bit stuffing, CRC-16/X.25 over the *destuffed* bits, length
unknown until the closing flag. The EFR32 frame controller can do none of it.
So the PHY runs with a 200-byte fixed length, no CRC, no whitening; ais_hw.c
decodes the bytes as they arrive (RX FIFO threshold 4 B) and re-arms RX the
moment it sees the closing flag or an abort (7 ones, too long), so the next
slot is not lost. Fixed length runs out only on noise.

### Sync word — measured, not assumed

`tools/ais_hw_model.py` on the 22 Nov powerbank capture (58 frames by
`ais_ref.py`): the last 24 line bits before the data were always

    0011 0011 0011 0011 0000 0001     (or the inverse: NRZI polarity)

= training `…1010` + flag `01111110` after NRZI. RAIL sends/receives sync words
LSB first (`RAIL_SyncWordConfig_t`), so:

| length | sync word 1 (line ends 1) | sync word 2 (ends 0) | frames found by the bit model |
|---|---|---|---|
| **16** (default) | `0x80CC` | `0x7F33` | **58 / 58** |
| 24 | `0x80CCCC` | `0x7F3333` | 56 / 58 |
| 32 | `0x80CCCCCC` | `0x7F333333` | 54 / 58 |

The beginning of the training sequence is often damaged, so 16 bits wins; the
noise false-sync rate at 16 bits is ~0.3/s (two words), each costing a few ms.
The model uses the float slicer of `ais_ref.py` in place of the radio, so it
checks the sync word and the decoder logic only — the radio's own sensitivity
is what the on-air benchmark measures. `ais_hw.c` sets the words at run time
(`RAIL_ConfigSyncWords`), so `AS<n>` / `AY` (bit order) / `AL` (FIFO data bit
order) can be tried on air without regenerating the PHY.

## What needs Simplicity Studio: the AIS PHY

The radio configurator has to generate it (no slc/pyradioconfig on this PC).
In `rail_soc_empty_FG23_iq_capture.slcp` → Radio Configurator, **add a channel
config entry** to the existing protocol (keep all existing entries):

| field | value |
|---|---|
| name | `AIS A/B (ch3000-3001)` |
| channel numbers | **3000 – 3001** (`AIS_CH_A` / `AIS_CH_B` in ais_hw.h) |
| base frequency | **161 975 000 Hz**, spacing **50 000 Hz** |
| crystal | 39 MHz (same override as the other entries) |
| modulation | 2FSK, **9600 bps**, deviation **2400 Hz**, shaping **Gaussian BT 0.4–0.5** |
| channel bandwidth | ~**16 kHz** (Carson 14.4 kHz + offset), let the tool pick the IF |
| frequency tolerance | xtal error ±10 ppm RX/TX (≈ ±3 kHz at 162 MHz) |
| preamble | **pattern `0011` (4 bits), length 16**; detection as short as allowed |
| sync word | **16 bits, `0x80CC`, dual sync ON, sync word 2 `0x7F33`** (LSB first) |
| frame | **fixed length 200 bytes**, no header, **no CRC, no whitening, no FEC**, LSB first |
| symbol encoding | NRZ (the NRZI is done in software — leave diff. encoding off) |

Then **Generate**, build, flash the FG23 (`fg23_flash.py` over COM6, as before).
If the configurator rejects the `0011` preamble, use `01` with length 8 and
rely on the sync word; the benchmark will show whether timing acquisition
suffers.

The crystal correction for 162 MHz is applied with `RAIL_SetFreqOffset` like
for the I/Q channels; `RAIL_GetRxFreqOffset` minus it gives the per-frame Δf
(assumes the same 4.6492 Hz/tick synth step — check with a carrier).

## Commands

FG23 (VCOM or cmdlink): `A1` channel A, `A2` channel B, `A0` back to the I/Q
stream; any other radio command (`F`, `i`, `W`, …) ends AIS HW first. `s`
prints the AIS HW counters. Bring-up: `AL`, `AY`, `AS<8..32>`.

ESP32 console (TCP 2323): `AISHW1` / `AISHW2` / `AISHW0`, and

```
AISBENCH 3     # alternate IQ / HW, 3 min each (IQ first); first 5 s of a slot not counted
AISBENCH       # report so far
AISBENCH 0     # stop, back to IQ, final report
```

Report lines (also in the TCP 2323 log):

```
AISBENCH slot 4 HW 175 s: 51 msgs (17.49/min), 33 stations
AISBENCH total: IQ 2 slots 98 msgs 16.80/min | HW 2 slots 104 msgs 17.83/min | HW/IQ 1.06 | stations both 38, IQ only 3, HW only 2
AISBENCH st 243XXXXXX  IQ  12 (2.06/min,  -98.5 dBm)  HW  14 (2.40/min,  -97.9 dBm)
```

Per station, the weakest ones heard by one path only are the sensitivity
answer; the overall rate ratio needs a long run (≥ 1 h) because AIS traffic
is bursty. **Run it on battery** — the ground loop (22 Nov) hits both paths,
but not necessarily equally.

## Status

| step | state |
|---|---|
| decoder logic + sync word, bit-level model on a real capture | ✅ 58/58 (16-bit sync) |
| FG23 code (`ais_hw.c`, `app.c` hooks) | ✅ compiles against the workspace SDK (`-fsyntax-only`, no warnings) |
| ESP32 code (AIS1 → emit, AISBENCH) | ✅ builds (`pio run -e ionos_mini`); I/Q path unchanged when AIS1 never arrives |
| AIS PHY generated in Studio | ⏳ needs the configurator |
| on-air bring-up, bit orders | ⏳ |
| benchmark | ⏳ |

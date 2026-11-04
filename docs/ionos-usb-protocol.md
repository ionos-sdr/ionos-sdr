# Ionos SDR — USB host protocol (HOST-M1)

Status: **v1 = what the firmware does today** (measured from `firmware-fg23/iq_capture/iq_stream.c`,
`specline.h`, `app.c::handle_line()`, `firmware-esp32/streaming/src/main.cpp`,
`tools/iq_bridge.py`), **v2 = the additions the host integrations need**
(z2labs websdr source module, SoapyIonos). v2 is additive: a v1 host keeps working.

Scope: the native USB CDC link between the ESP32-S3 and a PC. The WiFi paths
(SpyServer 5555, raw TCP 8888, rtl_tcp 1234) are separate and unchanged.

---

## 1. Transport

| item | v1 (today) | v2 |
|---|---|---|
| USB class | CDC-ACM, one interface (ESP32-S3 USB-Serial-JTAG, `ARDUINO_USB_MODE=1`) | **since 2026-11-04: CDC-ACM on the USB-OTG peripheral (TinyUSB via esp_tinyusb, `IONOS_USB_TINYUSB=1`)** — new PID, so a new COM number; no chip reset on DTR/RTS; flashing stays on the UART. A composite (2× CDC) is not required |
| identity | Espressif VID 0x303A / PID 0x1001, serial = chip MAC (ESP ROM) | own PID (pid.codes request) **or** keep 0x303A:0x1001 and identify by the `MSG1` hello block (see 4.3). Decision open; the hello block is mandatory either way, so hosts never depend on VID/PID alone. |
| baud / flow control | ignored (USB bulk); DTR/RTS must **not** reset the board (`enableReboot(false)` / HWCDC: no auto-reset on this path) | same |
| direction device → host | binary blocks, fixed **1040 B** | same, three block types |
| direction host → device | nothing | text lines, `\n`-terminated, ASCII, ≤ 80 chars |
| back-pressure | the ESP drops whole blocks when the CDC TX FIFO has < 1040 B free (`st_usbdrop`); never partial blocks | same; the host must read continuously (≥ 8 kB buffers) |

A block is never split across a device→host write; the host resynchronises on
the magic (16-byte aligned search) after any loss.

## 2. Block header (16 bytes, little-endian, identical for every block type)

```
offset  size  field       IQB2 meaning                       SPC1 meaning              MSG1 meaning
0       4     magic       'I','Q','B','2' = 0x32425149       'S','P','C','1' = 0x31435053   'M','S','G','1' = 0x3147534D
4       4     seq         block sequence number, +1 per block sent by the FG23 (loss = gaps)      ESP-side counter
8       2     nsamp       complex samples in payload (256)   nbin (≤ 1016)             text length in bytes (≤ 1024)
10      2     decim       total decimation R                 flags: bit0 = last chunk, bit8..15 = chunk index   msg type (4.3)
12      4     fs_in_hz    bits 0..19 = FG23 input rate [Hz]; bits 20..31 = RSSI [dBm], signed 12-bit   f_center [Hz]   0
```

`seq` is written by the FG23 at the moment of the SPI send, so an RDY timeout
does not burn a number: a gap in `seq` is a real lost block, nothing else.

## 3. Block types

### 3.1 `IQB2` — I/Q samples (v1)
- payload: 256 × (int16 I, int16 Q), interleaved, little-endian → 1024 B.
- sample rate = `fs_in_hz & 0xFFFFF` / `decim` (today 400 000 / 8 = 50 000 sps).
- RSSI = `(int16)((fs_in_hz >> 20) & 0xFFF)` sign-extended from 12 bits, dBm, refreshed every ~20 blocks (~100 ms).
- full scale: int16; the FG23 applies a digital shift (`IQ_AUTOSTART_SHIFT`), no analogue RX gain control exists (RAIL AGC).
- rate: 195 / 391 / 781 blocks/s at R = 8 / 4 / 2 (50 / 100 / 200 ksps, 204 / 407 / 813 kB/s). Measured 2026-11-04 over USB (TinyUSB CDC): R = 2 for 600 s without a lost block. WiFi carries 712–747 kB/s, i.e. 100 ksps comfortably, 200 ksps not.

### 3.2 `SPC1` — spectrum line (v1, scan mode)
- payload: `u32 span_hz`, `i16 floor_dbm`, `u16 range_db`, then `nbin × u8` with dB = floor + v·range/255.
- produced only while a `W` scan runs; the I/Q stream is stopped meanwhile and resumes on `W0`.

### 3.3 `MSG1` — text from the device (v2, new)
- payload: UTF-8 text, `nsamp` bytes, not NUL-terminated, rest of the 1024 B zero.
- carries command responses, the hello block, status and errors, so the host needs **one** parser.
- `decim` field = message type: `0` = response to a host command (`#`-prefixed lines as on the console),
  `1` = hello (see 4.3), `2` = periodic status (2 s, same fields as the console status line, `key=value` pairs),
  `3` = error/warning (e.g. `LOST=…`, `usbdrop=…`).

## 4. Host → device commands (text lines)

### 4.1 Forwarded to the FG23 (v1 command set of `handle_line()`, via the ESP cmdlink UART, 115200 8N1, one wire)

| line | effect | notes |
|---|---|---|
| `F<kHz>` | absolute tune, kHz | 100 000…1 000 000 kHz; stops and restarts the stream with the same R (a few ms gap); **kHz resolution only** |
| `o<ticks>` | fine offset, 1 tick ≈ 4.649 Hz, clamped to ±0x3FFF (≈ ±76 kHz) | this is how sub-kHz tuning is reached today (`o482` = +2.22 kHz is the measured crystal calibration at 144.8 MHz) |
| `f<ch>` | raw channel (radioconf grid) | bounded by `TUNE_MAX_CHANNEL` |
| `i<R>` | start the I/Q stream with total decimation R (integer, not log2); `i` alone = autostart R; `i` while running = stop | rate = 400 000 / R; R ∈ {2, 4} ∪ multiples of 8 up to 4096 (`iq_stream_norm_decim()`; R = 2 / 4 use the first CIC stage only, the FG23 core runs at 78 MHz and the SPI at 19.5 MHz for them) → **200 ksps maximum** since 2026-11-04; R = 16 → 25 ksps … |
| `W<center_kHz>,<span_kHz>,<nbin>[,floor,range]` / `W0` | wideband RSSI scan → `SPC1` blocks / stop and resume I/Q | the scan parser only accepts digits, `,` and `-` |
| `T<avg_us>,<wait_us>[,method]` | scan timing | |
| `t<dBm>` | RSSI trigger threshold | capture mode only |
| `s` | status print (console text; in v2 returned as `MSG1` type 0) | |
| `d<dBm>` | TX power | |
| `w` `p` `x` `b` `M…` `y` `c` `a` `k` `n` `g` `v` `z` | TX test streams, APRS beacon, CW Morse, capture/bench/diagnostics | **not part of the host API** in v1/v2 — the ESP must reject them in USB-driver mode (`MSG1` type 3) |

Commands in `NEEDS_STOP = "cwpxbzydfkngMW"` stop the stream first and resume after.

### 4.2 Handled by the ESP (v2, new)

| line | effect |
|---|---|
| `FH<Hz>` | tune with Hz resolution: the ESP computes `F<kHz>` + `o<ticks>` (ticks = (Hz − kHz·1000)/4.649, within ±0x3FFF) and sends both; answers with the resulting exact frequency in `MSG1` |
| `R<sps>` | sample rate request in sps; the ESP picks the nearest valid R (2, 4, or round(400000/sps) to a multiple of 8) and sends `i<R>`; answers with the achieved rate |
| `G<shift>` | digital gain shift 0…4 (replaces `IQ_AUTOSTART_SHIFT`); the only "gain" there is |
| `CAP?` | capability block (`MSG1` type 1 content, re-sent on demand) |
| `VER?` | firmware versions: ESP (`git describe`), FG23 (from the FG23 `v?` reply or the version struct read over SWD), board ID (EEPROM 0x51) |
| `MODE USB` / `MODE STANDALONE` | switches the operating mode (see UI-M4); in USB mode the ESP stops its own demodulators and the WiFi SpyServer, and forwards every block to USB |
| `STAT <s>` | status period in seconds, 0 = off |

Responses to every command: one `MSG1` type 0 block, first line `OK` or `ERR <reason>`, then the FG23's `#`-lines if any.

### 4.3 Hello / capability block (`MSG1` type 1)

Sent once when the CDC port opens (DTR asserted) and on `CAP?`. `key=value` lines:

```
ionos=1
product=ionos-sdr-mini
hw=rev-b3-2l
board=BRD4265B            ; from the radio-board ID EEPROM (I2C 0x51, "EFMP…")
chip=EFR32FG23
fw_esp=<git describe>
fw_fg23=<version>
freq_min_hz=430000000     ; capability table of THIS radio board / radioconf
freq_max_hz=440025000
fs_in_hz=400000
decim_list=2,4,8,16,24,32,48,64
gain_shift_max=4
rssi=1
spectrum=1                ; SPC1 scan supported
tx=0                      ; TX exposed to the host (future)
```

Hosts configure themselves from this block, never from the name: a new radio
board (MG24, FG25, FG28) only changes the table.

## 5. Session example (v2)

```
host  → VER?
device← MSG1 t0: OK\nfw_esp=v0.9.3-12-gf1cec9a\nfw_fg23=0.4.1\nboard=BRD4265B
host  → MODE USB
device← MSG1 t0: OK
host  → R 100000
device← MSG1 t0: OK\nfs=100000 decim=4
host  → FH 433775000
device← MSG1 t0: OK\nf=433775000 (F433775 + o0)
device← IQB2 seq=0 … (390 blocks/s)
device← MSG1 t2: blk_s=390.0 lost=0 rssi=-118 usbdrop=0 heap_min=81   (every 2 s)
```

## 6. Host-side rules (websdr module and SoapyIonos share them)

1. Read in large chunks (≥ 16 kB), scan for a magic at 16-byte alignment, validate `nsamp`/`decim` plausibility, then consume 1040 B; on mismatch advance by 4 B and retry.
2. Count `seq` gaps → report as overflow/loss to the application (Soapy `SOAPY_SDR_OVERFLOW` flag on the read).
3. Convert int16 → CF32 with 1/32768; CS16 passes through; CS8 = int16 >> 8 (host-side, no firmware change).
4. Treat `MSG1` type 3 as a log line, never as a reason to close the stream.
5. After opening the port wait for the hello (≤ 1 s), else send `CAP?`; if nothing arrives, the device runs a v1 firmware: assume the defaults (50 ksps, 430–440 MHz) and use `F`/`i` only.
6. Never send the diagnostic/TX letters of 4.1.

## 7. Firmware work this implies (ESP, `board/ionos-mini-idf`)

- CDC RX line parser → forward table (4.1) / local commands (4.2) / reject list.
- `MSG1` encoder with a small queue, sharing the USB writer with the block path (blocks have priority; messages are dropped, not blocks).
- Hello on DTR, `CAP?`, `VER?` (needs `FW_VERSION` from `git describe` in `platformio.ini` and a version string in `iq_capture` + `v?`).
- `FH`, `R`, `G` wrappers; `MODE` wired to UI-M4.
- Optional later: own PID; TX exposure (`tx=1`).

## 7b. Implementation status (firmware `board/ionos-mini-idf`, 2026-11-03)

Implemented in `src/usb_host.{h,cpp}`: CDC line parser, `MSG1` queue (4 blocks,
written only when a whole block fits, after the I/Q flush), hello on connect
(`(bool)Serial` edge on HWCDC) and on `CAP?`, `VER?` (`fw_fg23=n/a` until the FG23
`v?` exists), `MODE USB|STANDALONE` (gates the APRS/CW demodulators off, header
shows `USB`), `STAT <s>`, `FH<Hz>` (= `F<kHz>` through the rate-limited tuning
path + `o<ticks>` 400 ms later), `R<sps>` (through the ESP's decimation logic,
see menu.h), forwarding of `F o f i W T t s`, rejection of the TX/diagnostic
letters. Deviations from the table above:
- `G<shift>` answers `ERR`: the digital shift is compile-time on the FG23.
- periodic status (`MSG1` type 2, 2 s) starts only after the host has sent its
  first line, so a v1 host (iq_bridge.py) sees one hello block and nothing else.
- `decim_list` follows the FG23 rule (2, 4, multiples of 8); `R` rounds the same way. (Until 2026-11-04 the FG23 accepted only R ≥ 8: `R 100000` → `i4` → R stayed 8.) Above 50 ksps the ESP's APRS / CW demodulators stay off; those rates are for a host.
Reference client: `tools/usb_host_test.py COM7 --cmd "VER?" --cmd "R 100000"`.

Scan over USB (2026-11-03, later the same day): `SPC1` blocks reach the host
(`W433775,1000,512` → ~76 lines per 2.5 s, 16/16 trials). Two fixes were
needed:
- ESP `main.cpp`: a `SPECLINE` block from the FG23 returned before the USB
  copy was made, so scan lines never left the ESP over USB (SpyServer/TFT
  only).
- FG23 `cmdlink.c`: the 16-char scan line lost its CR (17th byte) in ~30 % of
  the cases while the stream ran — read back over SWD (`FGRD`, see below),
  the line buffer held `W…512W…512`. The EUSART RX FIFO is 16 deep and the
  RX interrupt sat at the lowest priority behind the stream's RAIL/LDMA
  handlers (> 1.4 ms). Priority 0 fixed it; the hardware `RXOF` overflow is
  now counted in `cmdlink_stats()`.
Also added: the ESP sends every cmdlink line twice (30 ms apart) and the
FG23 executes an identical line within 1 s only once; `FGRD <hex> [words]`
on the ESP console reads FG23 RAM over SWD without halting it (addresses
from `arm-none-eabi-nm` on the `.axf`) — the mini board has no FG23 serial
output, this is how its counters are inspected.

**Host rule — never assert RTS.** What looked like "the port goes silent after
a close/reopen until the ESP is reset" was the ESP32-S3 USB-Serial/JTAG
peripheral resetting the chip (`rst:0x15`): it emulates the esptool auto-reset
and pulls EN whenever the host asserts RTS while DTR is low, which pyserial's
default open/close does (reproduced 7/7, console showed the boot banner each
time; with RTS held low 6/6 reopens streamed at once). The S3 has no register
to disable this (the P4's `USB_UART_CHIP_RST_DIS` does not exist here), so
every host sets DTR=1, RTS=0 *before* opening — `tools/usb_host_test.py`
`open_port()`, `iq_bridge.py`, `usb_sink.py` do; the websdr sets
`{dataTerminalReady: true, requestToSend: false}` right after `open()` and
hands the probed port over without a reopen.

**USB-OTG + TinyUSB (2026-11-04).** `src/tusb_cdc.{h,cpp}` replaces the HWCDC
behind `RAW` / `USBH_RAW`: writes go into a 256 kB PSRAM ring, a pump task on
core 0 moves it into the TinyUSB CDC FIFO (16 kB, IN transfers of 4 kB, woken by
`tud_cdc_tx_complete_cb`). esp_tinyusb 1.7.6 is vendored in
`third_party/esp_tinyusb` with its `TINYUSB_CDC_ENABLED` Kconfig symbol renamed:
the Arduino core compiles its own `USBCDC.cpp` when that symbol is set and fails
without the Arduino-only `arduino_tinyusb`. Host-side lesson from the same day: a
Python reader that does `buf = buf[1040:]` per block copies the whole buffer each
time and cannot keep up with 781 blocks/s (it showed ~770 blocks/s and growing
loss); parse by offset (`tools/usb_host_test.py` fixed). Because of that the
~776 kB/s measured earlier through the USB-Serial/JTAG is not a proven HWCDC limit.

## 8. Open points

- USB identity: own PID vs. hello-only identification (hello is mandatory regardless).
- `fs_in_hz` is 400 kHz today; higher input rates (other radioconfs) fit in the 20-bit field up to 1 048 575 Hz — above that the header needs a new magic (`IQB3`), not a field reinterpretation.
- RX analogue gain: RAIL AGC only; if a future radio board exposes LNA steps, `G` grows a second argument, the hello gets `gain_list=`.

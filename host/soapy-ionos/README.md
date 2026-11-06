# SoapyIonos — SoapySDR driver for the Ionos SDR (HOST-M3)

MIT. Speaks the native USB CDC protocol of [`docs/ionos-usb-protocol.md`](../../docs/ionos-usb-protocol.md)
(IQB2 / SPC1 / MSG1 blocks, host command lines). Any SoapySDR application
(GNU Radio via gr-soapy, GQRX, CubicSDR, SDR++'s SoapySDR source,
`SoapySDRUtil`) can then use the board as `driver=ionos`.

One universal driver: frequency range, sample rates (`fs_in_hz` / `decim_list`)
and the gain range (`gain_shift_max`) come from the device's hello / `CAP?`
block, so a new radio board needs no driver change.

## Build

### Linux

```sh
sudo apt install cmake g++ libsoapysdr-dev soapysdr-tools
cmake -S host/soapy-ionos -B build-soapy
cmake --build build-soapy -j
sudo cmake --install build-soapy        # module -> <prefix>/lib/SoapySDR/modules0.8/
SoapySDRUtil --find=driver=ionos
```

Without installing: `export SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy`.
The user needs access to `/dev/ttyACM*` (group `dialout` on Debian/Ubuntu).
ModemManager may probe new ttyACM ports for a few seconds after plug-in; if it
interferes, add a udev rule `ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", ENV{ID_MM_DEVICE_IGNORE}="1"`.

### Windows (PothosSDR / radioconda SoapySDR)

```bat
cmake -S host\soapy-ionos -B build-soapy -G "Visual Studio 17 2022" -A x64 ^
      -DSoapySDR_DIR=C:\PothosSDR\cmake
cmake --build build-soapy --config Release
cmake --install build-soapy --config Release
```

Copy/install `ionosSupport.dll` into the `lib\SoapySDR\modules0.8` folder of the
SoapySDR installation the application uses (SDR++ ships its own SoapySDR.dll —
the module must match that ABI version). COM ports are found through SetupAPI;
`port=COM7` selects one explicitly.

macOS: builds with the POSIX serial path; every `/dev/cu.usbmodem*` is probed
for the hello. Untested.

## Device arguments

| key | default | meaning |
|---|---|---|
| `port` | (found) | `/dev/ttyACM0`, `COM7`, … — skip discovery and use this port |
| `serial` | | pick one device from `--find` |
| `mode` | `usb` | `usb`: send `MODE USB` (on-board demodulators off); `keep`: leave the mode alone |
| `restore_mode` | `1` | send `MODE STANDALONE` on close if the driver switched to USB mode |
| `stat` | (firmware: 2) | status period in s (`STAT`) |
| `zero_fill` | `1` | insert zeros for lost IQB2 blocks (timing stays continuous) |
| `report_gaps` | `1` | `readStream` returns `SOAPY_SDR_OVERFLOW` once per call in which a seq gap was seen |
| `max_fill_blocks` | `4096` | gaps longer than this are counted but not zero-filled (≈ 5 s at 200 ksps) |
| `ring_samples` | `1048576` | host ring size in complex samples (≈ 5 s at 200 ksps) |
| `probe` | `1` | `probe=0`: `find` lists matching USB ports without opening them |

Discovery: USB ports with VID 0x303A and PID 0x4001 (TinyUSB CDC, the current
firmware) or 0x1001 (USB-Serial/JTAG, older builds). Both PIDs are generic
Espressif ones, so a port is only reported when it answers with the hello
(`ionos=1`) — or streams IQB2 without a hello (v1 firmware, then fixed
50 ksps / 430–440 MHz, `F` tuning only). Extra ports to probe can be given in the
environment: `IONOS_PORTS=/dev/ttyACM3,/tmp/ionos0` (used by the tests).

The port is opened with **DTR = 1, RTS = 0** and RTS is never touched (the
ESP32-S3 USB-Serial/JTAG resets the chip on RTS = 1 while DTR = 0).

## Streams

- RX, one channel. Formats: `CS16` (native, full scale 32768), `CF32` (×1/32768),
  `CS8` (int16 >> 8, host-side as the spec allows).
- Rates: `fs_in_hz / R` for each R in `decim_list` — today 6.25 k … 50 k, 100 k,
  200 ksps. `setSampleRate` sends `R<sps>` and waits (≤ 3 s) until the IQB2
  headers carry the new R.
- `setFrequency` sends `FH<Hz>` (kHz tune + RAIL fine offset); the reply's
  `f=` is the frequency `getFrequency` returns. Outside the board range from
  the hello the driver warns and still sends it (the ESP accepts 100 MHz–1 GHz).
- Gain: one element `DIGITAL` = the FG23 digital shift, 6.02 dB per step,
  0 … `gain_shift_max`. Today's firmware reports `gain_shift_max=0` (and answers
  `G` with `ERR`), so the range is [0, 0].
- `activateStream` / `deactivateStream` gate the host side only: the device
  streams continuously, and the only device-side switch (`i`) toggles, so the
  driver never sends it. The reader thread keeps draining the port either way.
- No hardware time.

### Loss handling

The reader thread reads up to 64 kB at a time straight into one linear buffer
(napping 1 ms after a short read so the OS batches USB transfers) and parses
by offset: one header compare per block on the fast path, no per-block copies
besides the one into the sample ring.

- **seq gap** → `lost_blocks += gap`, that many blocks of zeros are inserted,
  `SOAPY_SDR_OVERFLOW` is returned once, a rate-limited log line is written.
- **block cut short** (the link occasionally loses two 64-byte USB packets
  inside one block): a valid header appears inside the 1040-byte span of the
  current block → the short block is dropped (`truncated_blocks`), the parser
  continues at that header, the seq gap books the block as lost (zero-filled).
- **garbage / bad magic** → byte-wise search for the next plausible header
  (`resyncs`).
- **seq going backwards** (the FG23 restarts the stream at seq 0 on every `F`,
  `FH`, `R`) or a new R in the header → `stream_restarts`, not a loss.
- `SPC1` blocks are skipped (`spc_blocks`); `MSG1` status is parsed, type 3
  (error/warning) is logged as a warning, everything else at debug level.

## Sensors and settings

Global sensors (`readSensor`): `lost_blocks`, `gap_events`, `filled_samples`,
`truncated_blocks`, `resyncs`, `stream_restarts`, `host_overflows`, `blocks`,
`first_seq`, `last_seq`, `spc_blocks`, `msg_blocks` (all counted since
`activateStream`, frozen while deactivated), `rssi` (dBm, from the IQB2 header),
`dev_lost`, `dev_usb_drop`, `dev_blk_s`, `dev_mode` (from the MSG1 status),
`link` (`ok` / `error`). Channel sensor: `rssi`.

Settings: `zero_fill`, `report_gaps`, `mode` (`USB` / `STANDALONE`),
`stat_period` (s), `command` (write: send one host-API line such as `o482`,
TX/diagnostic letters and `i` are refused; read: the last reply),
`last_error` (read only: last MSG1 type 3 text).

`getHardwareInfo` returns the hello and `VER?` keys (`board`, `hw`, `fw_esp`,
`fw_fg23`, …), the port and the protocol version.

## Tests (no hardware)

`test/ionos_mock.py` emulates the device on a Linux pty: hello on connect and
on `CAP?`, `VER?`, `MODE`, `STAT`, `FH`, `R` (with the firmware's rounding and
a seq-0 restart), `G` → `ERR`, forwarded/rejected letters, MSG1 status, and an
IQB2 stream at 400 000 / R with a test tone whose phase follows the seq.
Deterministic loss injection: `--drop-every N --drop-burst K` (blocks not sent)
and `--cut-every M` (128 bytes removed mid-block); `--spc-every`, `--err-every`
mix in SPC1 and MSG1 type 3 blocks; `--v1` behaves like v1 firmware.

```sh
cmake --build build-soapy
host/soapy-ionos/test/run_tests.sh build-soapy        # SECS=60 for a longer soak
```

`run_tests.sh` runs the parser unit test (50 random streams with junk, drops,
cuts, SPC1/MSG1, random read sizes), `SoapySDRUtil --find` / `--probe` against
the mock, and `ionos_stream_test` at 200 / 100 / 50 ksps in CF32 / CS16 / CS8.
The stream test checks the sample rate, that every non-zero sample lies on one
continuous complex exponential (so each zero-fill has exactly the right length),
that the zero runs are whole blocks at exactly the seqs the mock dropped, and
that `lost_blocks` equals what the mock dropped.

Manual run:

```sh
python3 host/soapy-ionos/test/ionos_mock.py --link /tmp/ionos0 --drop-every 500 &
export SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy
SoapySDRUtil --probe="driver=ionos,port=/tmp/ionos0"
SoapySDRUtil --args="driver=ionos,port=/tmp/ionos0" --rate=200e3 --direction=RX
```

## Known limits

- The USB iSerial of the TinyUSB build is the esp_tinyusb default `123456` on
  every board, and the hello has no serial/MAC key; the driver then uses the
  port name as `serial`. If the firmware adds `serial=` or `mac=` to the hello,
  the driver uses it.
- Retuning (`FH`) restarts the FG23 stream a little later (rate-limited `F`,
  `o` 400 ms after it); there is no "done" message, so the first few hundred
  ms after a retune may still carry the old frequency.
- Scan mode (`W…` → SPC1) is not exposed; SPC1 blocks are skipped. TODO.
- TX (`tx=1`) not exposed (the firmware does not offer it to hosts).
- Windows serial and enumeration code is compile-checked only (MinGW); macOS
  untested.

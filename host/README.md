# Host software

- `libionos/` — MIT C core: device discovery, stream decoding, control (planned).
- [`soapy-ionos/`](soapy-ionos/) — SoapySDR driver (MIT), gives GNU Radio via gr-soapy, GQRX, CubicSDR, SDR++'s SoapySDR source.
- `sdrpp-source/` — SDR++ source module (GPL-3.0, as required by the SDR++ module interface).

Until the native protocol is specified, SDR++ connects via its built-in SpyServer source.

## SoapyIonos (HOST-M3)

`soapy-ionos/` is a SoapySDR module (`driver=ionos`) for the native USB CDC
link of [`docs/ionos-usb-protocol.md`](../docs/ionos-usb-protocol.md): it finds
the board by VID/PID 0x303A:0x4001 (or 0x1001) plus the hello block, configures
itself from `CAP?` (frequency range, `decim_list` rates up to 200 ksps, gain
range), streams CS16 / CF32 / CS8, zero-fills lost IQB2 blocks and reports them
as `lost_blocks`. It builds with CMake on Linux and Windows and comes with a
pty-based device mock and tests — see [`soapy-ionos/README.md`](soapy-ionos/README.md).

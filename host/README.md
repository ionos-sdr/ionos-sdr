# Host software

- `libionos/` — MIT C core: device discovery, stream decoding, control.
- `soapy/` — SoapySDR driver on top of libionos (gives GNU Radio via gr-soapy, GQRX, CubicSDR).
- `sdrpp-source/` — SDR++ source module (GPL-3.0, as required by the SDR++ module interface).

Until the native protocol is specified, SDR++ connects via its built-in SpyServer source.

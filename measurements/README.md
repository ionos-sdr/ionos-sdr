# Measurements (CC-BY-4.0)

Every figure quoted in the docs must be reproducible from here. One directory per experiment: raw capture(s), the script/notebook that produced the number, and a short RESULT.md.

Equipment used so far: Signal Hound BB60C / VSG60, HackRF, RTL-SDR, Morfeus TCXO generator, J-Link RTT, INA226.

Index (to be populated from the existing lab notes): NCO tick & phase continuity · SetFreqOffset latency · AFSK THD vs sample rate · CALOFFSET LSB behaviour · I²S rate vs bandwidth · WiFi throughput · receiver frequency response (~2.1 dB fixed-pattern) · CW envelope · APRS on-air decode.

## Experiments

| Directory | What | Hours |
|---|---|---|
| [`wifi-throughput-2026-08/`](wifi-throughput-2026-08/) | WiFi I/Q throughput benchmark | – |
| [`ionos-mini-2026-11/`](ionos-mini-2026-11/) | ionos-sdr-mini: USB soaks at 50/200 ksps, 48 h RF characterisation at 145 MHz, wideband CW/FSK sweep via SoapyIonos, re-check | ≈ 86 |

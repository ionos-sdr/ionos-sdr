# AIS receive on the ionos-mini — 2026-11-22

Board: ionos-mini (ESP32-S3 + EFR32FG23 radio board), firmware `board/ionos-mini-idf`
(`04a2b24` AIS mode). Location: Budapest, ~1 km from the Danube. Antenna: mag-mount
2 m whip, clamped to a radiator (counterpoise). Captures via SpyServer, 50 ksps,
tuned 161.985 MHz (channel A at −10 kHz). Captures themselves (36–288 MB) are not
in git; the decoded NMEA is.

## Results

| Condition | Duration | Bursts | With a ≥1 ms gain dip | Decoded (port = reference) |
|---|---|---|---|---|
| USB from a mains-charged laptop, antenna on radiator | 720 s | 398 | 386 | 13 = 13 |
| same, on-device FFT off (`F0`) | 90 s | 45 | 44 | 1 |
| same, FFT on (`F1`) | 90 s | 52 | 50 | 1 |
| **powerbank**, antenna on radiator | 90 s | 63 | 1 | **56** |
| powerbank, live (ais_web.py, 70 min) | 4245 s | – | – | 788 NMEA, 82 stations |
| USB from laptop, antenna off radiator | 90 s | 0 | – | 0 |

Live message mix (70 min): type 21 AtoN 567, type 4 base 101, type 20/23 59,
types 1–3 55, type 8 4, junk (40, 48) 2.

## Gain dips (ground loop)

- Dip: 12–17 dB, 2–4 ms (median 2.7 ms), every ~20 ms (spacing 17.7–21.4 ms).
- Noise-only power modulation: autocorrelation peaks at 20/40/60/80 ms; spectral
  lines at 50/100/150 Hz, 100 Hz +45.6 dB over the median (mains USB) vs +13.3 dB
  (powerbank).
- On a CW carrier (144.800, Morfeus generator): the carrier is still inside the dip
  at −12 dB with continuous phase (|Δφ| ≤ 0.06 rad) → no sample loss.
- Dip start positions modulo the 256-sample SPI block: uniform → not the data path.
- Ruled out: the on-device display FFT (20 ms period) — `F0` changes nothing.

## Demodulator

Float reference `tools/ais_ref.py`, fixed-point `src/ais_rx.c` via `tools/aisport.c`:

```
python3 tools/iq2raw.py capture.npy capture.iq16
gcc -O2 -Isrc -o aisport tools/aisport.c src/ais_rx.c -lm
./aisport capture.iq16 50000 161985 [-fc6000] [-mm64]
python3 tools/ais_ref.py capture.npy --tune_khz 161985 --chan A
```

Synthetic GMSK sweep (20 bursts/point, ±500 Hz offset, SNR in 14 kHz):

| channel LPF | post filter | 15 dB | 12 dB | 10 dB |
|---|---|---|---|---|
| 5 kHz | none | 19 | 11 | 2 |
| **6 kHz** | **none** | **20** | **17** | **9** |
| 8 kHz | none | 14 | 0 | 0 |
| 6 kHz | Gaussian BT 0.5 | 19 | 12 | 4 |
| 6 kHz | Gaussian BT 0.3 | 5 | 2 | 0 |

Slicer window (moving mean) at 20 dB: 16 bits 8/20, 64 bits 20/20.
Real capture: fc 5–8 kHz and window 32–128 bits all gave 6–13 frames — the gain
dips, not the parameters, limited it.

## Files

- `holes.py`, `holes2.py` — burst / gain-dip statistics from a capture
- `nmea_powerbank_90s.txt` — the 56 decoded sentences of the powerbank run

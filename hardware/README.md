# Hardware (CERN-OHL-P-2.0)

Target: 6-layer (8 if the impedance-controlled stackup requires it) FG23 radio board + ESP32-S3 carrier.

Current prototype: Silicon Labs BRD4265B radio board (434 MHz-matched — ~15–20 dB insertion loss at 144 MHz, fine for firmware validation, not for RF performance) on a custom ESP32-S3 N16R8 carrier, connected via a mezzanine connector. The carrier is designed pin-identical to the final board so firmware carries over unchanged.

Planned blocks: EFR32FG23B · 39 MHz VCTCXO (KDS DSA321SDN) with VDAC-driven Vc for WSPR · LNA via bias-tee (DC-block mandatory) · NXP AFT05MS004N LDMOS PA with DAC-controlled boost drain (3.6 V ≈ 2 W, 7.5 V ≈ 5 W) · 2×18650 1S · ILI9341 2.8" TFT · u-blox NEO-M8M.

Layout: `carrier/` (ESP32-S3 carrier, KiCad), `radio/` (FG23 radio board, KiCad), `fab/` (committed Gerbers/BOM per release).

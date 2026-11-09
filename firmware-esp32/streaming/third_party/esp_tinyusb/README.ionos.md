# esp_tinyusb 1.7.6 (vendored)

Upstream: https://github.com/espressif/esp-usb/tree/master/device/esp_tinyusb
(Espressif component registry `espressif/esp_tinyusb` 1.7.6~2), Apache-2.0, see LICENSE.

Used by `src/tusb_cdc.*` ([env:ionos_mini], `IONOS_USB_TINYUSB=1`) through
`override_path` in `src/idf_component.yml`.

**Changes:**

1. the Kconfig symbol `TINYUSB_CDC_ENABLED` is renamed to
`IONOS_TUSB_CDC_ENABLED` (Kconfig, CMakeLists.txt, include/tusb_cdc_acm.h,
include/tusb_config.h, usb_descriptors.c). The Arduino core (3.3.x) compiles
`cores/esp32/USBCDC.cpp` whenever `CONFIG_TINYUSB_CDC_ENABLED` is set, and that
file needs the Arduino-only `arduino_tinyusb` component, which this hybrid
build does not have - the build fails in USBCDC.cpp. With the renamed symbol
the Arduino file stays empty and esp_tinyusb works unchanged.

2. `include/tusb_config.h`: `CFG_TUD_CDC_TX_EPSIZE 2048` (multi-packet IN
   transfers). With the default 64 every bulk packet was one transfer and one
   TinyUSB task round trip: measured ~610 kB/s, the 200 ksps stream needs 812 kB/s.

Upstream `test/` and `test_apps/` are not part of this copy (git-ignored as
`_test_unused/`, `_test_apps_unused/`); `CHECKSUMS.json` is kept as
`_CHECKSUMS.upstream.json` (the files differ from upstream).

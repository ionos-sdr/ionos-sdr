# ESP32-S3 companion firmware (MIT)

PlatformIO / ESP-IDF. Target: ESP32-S3 N16R8 (16 MB flash, 8 MB octal PSRAM). Handles WiFi and USB-CDC I/Q streaming (SpyServer-compatible mode; native mode planned), TFT waterfall (LovyanGFX, ILI9341, hardware scroll, TE interrupt), GPS, SD store-and-forward, TLS to SatNOGS DB.

Throughput note: `CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP=y` is required for the ~712 kB/s WiFi figure.

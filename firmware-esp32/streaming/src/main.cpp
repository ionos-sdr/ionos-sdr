/* SPDX-License-Identifier: MIT
 *
 * fg23_spi_to_usb_wifi.ino / main.cpp
 * ESP32-S3: SPI slave towards the FG23 -> USB + WiFi + rtl_tcp
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================= THE COMPLETE CHAIN =================
 *
 *   antenna -> FG23 RAIL FIFO -> zero-copy read -> two-stage CIC
 *   -> DC block -> block buffer -> LDMA -> SPI (CS per block)
 *   -> ESP32-S3 SPI slave DMA -> THREE PARALLEL OUTPUTS:
 *
 *        (a) NATIVE USB CDC ...... 1040-byte blocks with header
 *                                   -> iq_bridge.py -> SDR++ (desktop)
 *        (b) TCP 8888 ............. RAW int16 I/Q, no header
 *                                   -> SDR++ "Network" source DIRECTLY,
 *                                      no python needed
 *        (c) TCP 1234 ............. rtl_tcp protocol, 8-bit
 *                                   -> SDR++ Android, SDR Touch, anything
 *
 * The three outputs are INDEPENDENT. There is no "switching": whatever is
 * connected receives data, the others do not matter. This is simpler AND
 * more reliable than detecting "is USB present" - that detection can lie
 * (the CDC looks "connected" even when nobody reads it), whereas "is there
 * a TCP client" cannot.
 *
 * ===================== SPI WIRING =====================
 *   WSTK EXP   FG23 pin   signal                  ESP32-S3 (Heltec V3)
 *   ---------------------------------------------------------------------
 *   EXP 15     PC05       SCLK                -> GPIO5
 *   EXP 10     PC00       MOSI (FG23 -> ESP)  -> GPIO6
 *   EXP 13     PA07       CS   (block frame)  -> GPIO7
 *   EXP  1     GND                            -> GND
 *   EXP 11     PA06       CMD  (ESP -> FG23)  <- GPIO4   [OPTIONAL]
 *
 * The CMD line is the return path: the ESP sends the tuning command to the
 * FG23 on it when the frequency is changed from the phone. WITHOUT it
 * everything works, only the rtl_tcp frequency command has no effect.
 * On the FG23 side cmdlink.c is required.
 *
 * ===================== NATIVE USB WIRING =====================
 *   USB cable D-   -> GPIO19
 *   USB cable D+   -> GPIO20      (swapped: Windows error 43)
 *   USB cable GND  -> GND
 *   USB cable VBUS -> DO NOT CONNECT
 *
 * ================= platformio.ini =================
 *     build_flags =
 *       -D ARDUINO_USB_MODE=1
 *       -D ARDUINO_USB_CDC_ON_BOOT=1
 * Result: Serial = native USB CDC (raw stream), Serial0 = CP2102 (text).
 *
 * ---- RECOMMENDED FOR MEASUREMENT: -D ARDUINO_USB_MODE=0  (2026-08-06) ----
 * MODE=1 uses the HARDWARE USB-Serial-JTAG (HWCDC). Two of its properties
 * TOGETHER break the PC-side measurement tool (usb_aprs_rx.py):
 *
 *   1) toggling DTR/RTS from the host RESETS the chip IN HARDWARE
 *      (bootlog: "rst:0x15 (USB_UART_CHIP_RESET)"). After the reset the USB
 *      RE-ENUMERATES, and the handle already opened by Windows usbser.sys
 *      becomes a ZOMBIE: read() on the PC returns 0 bytes forever.
 *   2) HWCDC decides "is a host present" from whether the TX FIFO drains.
 *      Once it fills up (because the host did not read), it is "no host"
 *      from then on, every write is dropped -> the FIFO never drains. A
 *      SELF-REINFORCING lockup. In the status line it shows as
 *      USB 0.0 kB/s(-244), where (-244) is st_usbdrop, i.e. 244 dropped writes.
 *
 * With MODE=0 Serial is the TinyUSB USBCDC, which HAS an enableReboot(false)
 * method: DTR/RTS then does NOT reset, and the measurement runs stably.
 *     build_flags =
 *       -D ARDUINO_USB_MODE=0
 *       -D ARDUINO_USB_CDC_ON_BOOT=1
 * The COM port number may change (different USB PID), but the VID stays
 * 0x303A, so the auto-detection in usb_aprs_rx.py still finds it.
 * Flashing is unaffected (it goes through the CP2102), and the ROM bootloader
 * always provides a JTAG port, so you cannot lock yourself out.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include "driver/spi_slave.h"
#include "esp_intr_alloc.h"   /* ESP_INTR_FLAG_IRAM / _LEVEL3 */
#include "soc/gpio_struct.h"  /* GPIO register structure (RDY, from ISR) */
#include "driver/gpio.h"
#include "esp_heap_caps.h"    /* RAM diagnostics in the 2 s status line */
#include "esp_wifi.h"         /* esp_wifi_set_bandwidth (HT40, for RTL_ENABLE) */
#include <lwip/sockets.h>     /* send(fd, ..., MSG_DONTWAIT) - see ring_flush */
#include <string.h>
#include <math.h>             /* log10f - signal level in dB */
#include "wifi_bench.h"       /* WiFi ceiling meter ('B...' commands on DIAG) */
#include "rgb_load.h"         /* WS2812 load indicator ('L...' commands)        */
#include "tft.h"              /* ILI9341 status display, SPI2 (IO_MUX 9-14)     */
#include "iq_fft.h"           /* narrowband I/Q -> waterfall on the panel       */

/* TinyUSB mode (ARDUINO_USB_MODE=0): here Serial is a USBCDC, which has an
 * enableReboot() method - used to disable the reset on DTR/RTS.
 * NOTE on the condition: the preprocessor treats an UNDEFINED identifier as
 * 0, so a bare "#if ARDUINO_USB_MODE == 0" would be TRUE even when the macro
 * is not given - and would compile the TinyUSB branch into an HWCDC build.
 * Hence the defined() check. */
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 0)
  #include "USB.h"
  #define FG23_USB_TINYUSB 1
#else
  #define FG23_USB_TINYUSB 0
#endif

/* ==================== CONFIGURATION ==================== */

/* ---- WiFi ----------------------------------------------------------
 * Several networks, no priority: WiFiMulti scans the surroundings and picks
 * the STRONGEST of the KNOWN networks. It does not try them in table order;
 * it decides on site - at home it joins the home network, in the workshop
 * the workshop one, regardless of table position.
 *
 * EMPTY ssid entries are skipped, so unused rows need not be deleted, and the
 * table size follows automatically - no separate count to forget to update.
 *
 * If no known network is in range, it opens its own AP. In the car this is
 * the right mode: the phone connects to the ESP, no router needed. */
/* The LOAD SWITCHES (WIFI_ENABLE, SPY_ENABLE, OLED_ENABLE, APRS_RX_ENABLE)
 * are all wrapped in #ifndef so they can be OVERRIDDEN from platformio.ini
 * build_flags. Otherwise a WiFi-free isolation measurement would require
 * editing this file by hand and then reverting - which is exactly how a stray
 * 0 gets left behind and costs weeks of chasing a non-existent bug.
 * This way the measurement is only an environment switch: [env:esp32s3_nowifi]. */
#ifndef WIFI_ENABLE
#define WIFI_ENABLE          1
#endif
#define WIFI_STA_TIMEOUT_MS  10000u
/* WiFi credentials live in secrets.h (git-ignored). Copy secrets.example.h
 * to secrets.h and fill in your own networks. */
#include "secrets.h"
#define WIFI_AP_COUNT (sizeof(WIFI_APS) / sizeof(WIFI_APS[0]))

/* Reconnect: if the network drops, retry with a full scan after this long.
 * NOT more often - WiFiMulti.run() blocks for the duration of the scan
 * (a few seconds), and the main loop stalls meanwhile. */
#define WIFI_RETRY_MS        30000u

#define PORT_RAW     8888    /* raw int16 - SDR++ desktop "Network" source */
#define PORT_RTLTCP  1234    /* rtl_tcp - SDR++ Android, SDR Touch, gqrx... */

/* ---- rtl_tcp SWITCH ----------------------------------------------
 * 0 = the whole rtl_tcp path off: port 1234 is silent and ~75 kB RAM is
 * freed (64 kB ring + 10 kB upsampling buffer) - room for the spyserver's
 * 64 kB ring. The code still compiles, only the buffers are stubs and the
 * server is never started.
 *
 * WHY: the spyserver path (5555) is native 16-bit, native rate, no
 * upsampling, no 8-bit quantisation - better in every respect. rtl_tcp is
 * only needed when a client (e.g. SDR Touch) cannot do spyserver.
 *
 * Decision of 2026-08-03 evening, after the tests: OFF by default. The path
 * was fully measured (DC spike fixed, images at -38 dB, throughput ceiling
 * ~400 kB/s instead of the required 500 -> striping), and spyserver is
 * better in every respect. The +75 kB automatically yields the spyserver's
 * 64 kB ring (the halving ladder gets it) - which makes the phone glitch-free. */
#define RTL_ENABLE   0

/* ---- HT40 SWITCH - OFF by default, BECAUSE WE MEASURED IT ------------
 * The first version tied it to RTL_ENABLE, but the 2026-08-03 measurement
 * refuted it: with HT40 REQUESTED the +-(NQUEUE+-1) driver glitch jumped
 * from one every 10-30 s to 6-12 per second (BACK=25/2s, REORDER dup/hole by
 * the dozen, even short=1), while the throughput ceiling did NOT improve at
 * all (stayed 355-510 kB/s - the router did not grant it, or it is useless
 * on the crowded 2.4 GHz band). The wider channel request visibly disturbs
 * the WiFi driver's interrupt timing, and that in turn the SPI slave. Set to
 * 1 for experiments - but first check whether your router grants it at all
 * (WiFi analyser: 40 MHz band). */
#define WIFI_HT40    0
#define PORT_SPY     5555    /* SpyServer - 16-bit, NATIVE rate, SDR++ Android */

/* ---- SpyServer. This is the right path for Android: native 16-bit, and the
 * DEVICE dictates the rate, so there is no upsampling and no interpolation
 * images. rtl_tcp on 1234 STAYS - if this does not work at first, the proven
 * path is there. Details: spyserver.h */
#ifndef SPY_ENABLE
#define SPY_ENABLE   1
#endif
#if SPY_ENABLE
#include "spyserver.h"
#endif

/* ---- rtl_tcp: clients expect a fixed, "RTL-like" sample rate, and the
 * lowest standard RTL rate is 250 ksps - we have considerably less. So we
 * upsample to the target rate by an INTEGER factor. That keeps the client's
 * time and clock arithmetic correct, and audio plays at the right pitch.
 *
 * COST: upsampling leaves images in the spectrum (linear interpolation, no
 * filter after it). They are visible at the edges of the waterfall; the
 * demodulator does not see them, its own bandwidth filters them out.
 *
 * RTL_TARGET_SPS 0 = no upsampling (set the client manually). */
#define RTL_TARGET_SPS   250000u
#define RTL_MAX_UP       20u        /* upper bound: the deepest supported
                                     * mode is i32 (12500 sps), which needs
                                     * exactly 20x for 250 ksps. The agg_rtl
                                     * buffer size also derives from it (10 kB). */

/* ANTI-IMAGE FILTER. Linear interpolation is itself a filter (triangular
 * window = two consecutive L-long averages), and its zeros fall EXACTLY on
 * the images - but the NEIGHBOURHOOD of the images leaks through, and on the
 * waterfall that looks as if the signal were wider and "smeared".
 *
 * An ADDITIONAL L-long moving average at the output rate places another zero
 * at the same spot and suppresses the leakage by roughly 20 dB. Running sum,
 * i.e. two additions and one division per sample - trivial for the 240 MHz S3.
 *
 * Set to 0 to get exactly the old (raw linear) behaviour back. */
#define RTL_ANTI_IMAGE   1

/* 8-bit output: automatic gain.
 *
 * WIDE DEADBAND, SLOW STEP: every level change moves the WHOLE picture by
 * 6 dB, which shows as a horizontal band on the waterfall. So we only touch
 * it when the peak really leaves the [LO..HI] band, and even then at most one
 * step every two seconds. It settles once and stays.
 *
 * With RTL_FIXED_SHIFT >= 0 the shift is fixed and there is no AGC at all -
 * if you want to measure levels on the waterfall, use THIS (8 = upper byte). */
#define RTL_FIXED_SHIFT  (-1)
#define RTL_AGC_MS       2000u
#define RTL_AGC_HI       120        /* above this we attenuate (max 127) */
#define RTL_AGC_LO       30         /* below this we amplify */

/* ---- OLED (Heltec WiFi LoRa 32 V3) --------------------------------
 * The display's main purpose: SHOWING THE IP ADDRESS. Without it every
 * restart needs a serial monitor just to find out where SDR++ should
 * connect - unusable in the car.
 *
 * Heltec V3 pinout: SDA=17, SCL=18, RST=21, Vext=36 (active LOW).
 * If the display stays dark, first invert OLED_VEXT_ACTIVE_LOW, then set
 * OLED_VEXT_PIN to -1 (some V3 units do not power the OLED from Vext).
 *
 * platformio.ini REQUIRES:   lib_deps = olikraus/U8g2
 *
 * TIMING: a full 128x64 frame transfer is ~1 kB on I2C, about 13 ms at
 * 800 kHz. The main loop stalls meanwhile - BUT the SPI slave DMA keeps
 * receiving blocks as long as queued transactions remain. NQUEUE=24 gives
 * ~123 ms of reserve on i8, so the 13 ms is absorbed. That is why it was raised. */
#ifndef OLED_ENABLE
#define OLED_ENABLE            0     /* 2026-08-19: Heltec V3 retired.
                                      * ESP32-S3 devkit + ILI9341 240x320.
                                      * This removed the 21 ms stall from the
                                      * main loop and the GPIO36 conflict
                                      * (octal PSRAM pin). */
#endif
/* WS2812 load indicator LED. ON THIS board (Ali S3 dual-USB, N16R8) the
 * pinout says GPIO47 = RGB_LED (48 is SPICLK_N). Other clones use 48 or 38 -
 * if it does not light up, try those. The Heltec V3 has no RGB LED.
 * NOTE: 47 was the display TE in the waterfall bench -> TE moved to 48. */
#ifndef RGB_LED_PIN
#define RGB_LED_PIN            47
#endif
#define OLED_SDA_PIN           17
#define OLED_SCL_PIN           18
#define OLED_RST_PIN           21
#define OLED_VEXT_PIN          36     /* -1 = no Vext control */
#define OLED_VEXT_ACTIVE_LOW   1
#define OLED_I2C_HZ            800000u
/* Back to 1500: the measurement is closed, and it is proven that the 21 ms
 * frame transfer fits comfortably in the 24-deep transaction queue (qmax at
 * most 5 even during an OLED refresh). */
#define OLED_REFRESH_MS        1500u
/* After a packet is received the APRS page stays visible for this long, then
 * it returns to the network page. No paging needed: the display shows the
 * packet WHEN it happens. */
#define OLED_APRS_HOLD_MS      12000u

/* The include is HERE, not at the top of the file: OLED_ENABLE must already
 * be defined for it. */
#if OLED_ENABLE
#include <Wire.h>
#include <U8g2lib.h>
#endif

/* ---- APRS reception: AFSK1200 demod + AX.25 decoder + APRS-IS upload.
 * Details in aprs_rx.cpp; callsign, passcode and filter are set there too.
 * The demod works from the I/Q, so it does NOT disturb the other outputs -
 * the same block continues to 8888 and 1234.
 * Load at 50 ksps is on the order of 1-2% of one core. */
#ifndef APRS_RX_ENABLE
#define APRS_RX_ENABLE   1
#endif
#if APRS_RX_ENABLE
#include "aprs_rx.h"
#endif
#include "cw_rx.h"          /* CW Morse decoder */

/* ---- Menu and button. The Heltec V3 PRG button (GPIO0) pages through the
 * screens and switches the demodulators. Details: ui.h */
#include "ui.h"
#include "flashlog.h"
#include "specline.h"      /* SPECLINE (scan line) block format, shared with the FG23 */
#include "oled_spectrum.h"  /* 128x64 mini spectrum + 1-bit waterfall */

/* ==================== RDY LINE: ELIMINATING LOSS BY DESIGN ====
 *
 * Until 2026-08-03 the FG23 sent BLINDLY: nothing told it whether a queued
 * SPI transaction existed on the other side. If none did, the hardware wrote
 * into the old descriptor, and the block was either lost or arrived shifted
 * by a buffer cycle. This was the common root of LOST, BACK, the smeared
 * carrier and the silent APRS. Measurement showed ~7 blocks/s lost this way
 * under SpyServer load - 4-5 holes per 0.66 s APRS packet.
 *
 * The fix is a single wire: the ESP32 signals from the IDF spi_slave
 * post_setup/post_trans callbacks whether a transaction is REALLY queued
 * (hardware state, not our bookkeeping!), and the FG23 checks it BEFORE
 * pulling CS low, and WAITS if there is none. So a block cannot be lost - at
 * most delayed, which the FG23's 6-block buffer absorbs.
 *
 * WIRING:  ESP32 GPIO2  ->  FG23 PD2 / EXP 9   (GND already shared)
 *
 * Counterpart: iq_stream.c, IQ_RDY_ENABLE. While the wire is not connected,
 * leave it at 0 on the FG23 - the ESP32 side is harmless when driven. */
#define RDY_ENABLE   1
#define RDY_PIN      2

/* ---- Return path to the FG23 (tuning from the phone). 0 = disabled. */
#define CMDLINK_ENABLE   1
#define CMDLINK_TX_PIN   18         /* ESP GPIO18 -> FG23 PA06 / EXP 11
                                     * (2026-08-19: moved from 4; pin 4 goes
                                     * to the display/touch on the devkit) */
#define CMDLINK_BAUD     115200

/* ---------------- PINS ---------------- */
/* 2026-08-19 - ESP32-S3 devkit + ILI9341 display.
 * The display gets the SPI2/FSPI IO_MUX set (9..14), the touch SHARES it
 * (own CS), so the FG23 slave goes to SPI3. At 4 MHz the GPIO matrix is
 * more than sufficient for it. In the FINAL layout this is reversed: the
 * FG23 gets the IO_MUX (10/11/12 + RDY 14 + CMD 13), the display goes to
 * SPI3 - that is the precondition for the 700 ksps direction. */
#define PIN_SCLK   GPIO_NUM_15    /* FG23 PC05 / EXP 15 */
#define PIN_MOSI   GPIO_NUM_16    /* FG23 PC00 / EXP 10 */
#define PIN_CS     GPIO_NUM_17    /* FG23 PA07 / EXP 13 */
#define FG23_SPI_HOST SPI3_HOST

/* ---------------- BLOCK (must match iq_stream.c!) ---------------- */
#define BLK_SAMPLES   256
#define HDR_BYTES     16
#define PAYLOAD_BYTES (BLK_SAMPLES * 4)                  /* 1024 */
#define BLK_BYTES     (HDR_BYTES + PAYLOAD_BYTES)        /* 1040 */
#define IQ_BLK_MAGIC  0x32425149u                        /* "IQB2" */

/* Pre-queued transactions. NOT a luxury: if no transaction is queued at CS,
 * that block is lost FOR GOOD - and since the FG23 writes the sequence number
 * into the header AT SEND TIME, every sequence gap means EXACTLY this.
 * (The FG23's own drops do NOT show as gaps, they show in blocks/s.)
 *
 * 4 on i8 (195 blocks/s) is only 20 ms of reserve. 12 was ~61 ms - and that
 * is exactly what we measured: the log is full of d=13 gaps, i.e. 12 blocks
 * lost at once. "Exactly NQUEUE at once" is the fingerprint of the queue
 * running completely dry. 24 is ~123 ms. */
/* ==================== QUEUE EXPERIMENT (2026-08-03) ====================
 *
 * The measurement showed that the size of the sequence jumps follows this
 * number EXACTLY:
 *      NQUEUE = 12  ->  d = 13 (=12+1),  25 (=2*12+1)
 *      NQUEUE = 24  ->  d = 25 (=24+1),  49 (=2*24+1),  back -23 (=-24+1)
 *
 * This suggests that instead of one block we process a buffer whose content
 * is exactly one full buffer cycle away. BUT these are only two data points,
 * both near a power of two - could be coincidence.
 *
 * HENCE now 17 (prime, coincides with no other period). If the jumps switch
 * to 16 / 18 / 35, the queue is the culprit and we can search in the right
 * direction. If 25 and 49 remain, the theory is dead.
 *
 * THE EXPERIMENT RAN (2026-08-03): with NQUEUE=17 the jumps switched to
 * 18 / 35 / -16, POOL=100%. The queue is the culprit, so it is restored to
 * 24, and the real fix is the interrupt priority (see
 * spi_bus_config_t.intr_flags). The POOL counter stays: if the fix works,
 * BACK and with it the POOL denominator drop.
 *
 * ===================== ROLE AFTER RDY (2026-08-03) =====================
 *
 * Since the RDY handshake + RDY_STRICT, the queue running dry NO LONGER
 * causes reordering or loss: the FG23 simply POSTPONES the send (its own
 * NBLK=6 buffer holds ~31 ms) until a transaction is queued. NQUEUE is now
 * purely a shock absorber: it bridges the time the main loop is busy with
 * something else and does not re-queue.
 *
 * Sizing from measurement: qmax in normal operation is at most 5 (even
 * during an OLED refresh); only the blocking APRS-IS connect drives it to
 * 24/24 - and NO sensible depth covers that (330 ms = 64 blocks), it needs
 * a separate fix.
 *
 * ============ RESIZED 16 -> 24 (2026-08-07) ============
 *
 * 16 was chosen with the reasoning "82 ms tolerance, 16x the measured peak".
 * The overnight 4.55-hour measurement (16 599 frames, -100 dBm) refuted it.
 * From the PER# sequence numbers, at packet resolution:
 *
 *   - 73 loss events / 75 frames     -> 99.548 % success
 *   - for 70 % of the losses dt_max is 80..170 ms, and there is NO CRC error
 *     (the frame never reached the decoder: sample hole, not demod error)
 *   - dt_max median around losses 76.3 ms, at random times 21.6 ms;
 *     P(dt_max>60 ms) 50.7 % vs 7.1 %  =  7.2x enrichment
 *   - 228 distinct stalls: MEDIAN 88 ms, p90 98 ms, MAX 167 ms
 *
 * Depth 16 = 16 * 5.12 ms = 82 ms; the measured stall median is 88 ms.
 * So the queue was SYSTEMATICALLY SHORTER than the typical stall - not a
 * 16x reserve but a negative one. The "measured peak" looked small because
 * qmax was observed in normal operation, not during the stalls.
 *
 * There was an internal contradiction too: RXRING_BLOCKS = 24 (=123 ms), but
 * only what the queue has already picked up can land in the ring. The ring's
 * 123 ms capacity was therefore UNREACHABLE - the chain was sized by its
 * narrowest element, 82 ms. With 24 both queue and ring are 123 ms.
 *
 * This is not an NQUEUE patch: it releases half of the 25 kB ring already
 * paid for. Cost +8.3 kB RAM (rxbuf), and REORDER_BLOCKS must go to 32
 * (see there: RO_HOLD >= NQUEUE+1).
 *
 * WHAT THIS DOES NOT SOLVE: if the stall is a FLASH operation, the cache is
 * disabled, and then the spi_slave interrupt handler cannot run either -
 * there is nothing to queue the next transaction from, however deep the
 * queue. Only CONFIG_SPI_SLAVE_ISR_IN_IRAM=y protects against that. The two
 * are not alternatives but mutual preconditions: IRAM makes the depth USABLE
 * during a flash operation, and the depth determines HOW MUCH time can be
 * bridged. */
#define NQUEUE        24

/* DROPPING STALE BLOCKS - OFF by default.
 *
 * Measurement of 2026-08-03: when the queue of queued transactions runs dry,
 * blocks are NOT lost but arrive OUT OF ORDER. Evidence in the log: blocks/s
 * stays at the full 195 (nothing is missing), while LOST shows 1368/s -
 * seven times more than the number of blocks that exist at all. The sequence
 * number also steps backwards meanwhile.
 *
 * The consequence is severe: the 5.12 ms pieces of a continuous signal get
 * mixed up in time. The amplitude stays continuous, but the PHASE jumps at
 * every block boundary - of a clean carrier only 19.8 % remains, the rest is
 * smeared 780 Hz wide. AFSK certainly cannot be decoded from that.
 *
 * The REAL fix is that the queue NEVER runs dry (drain + immediate re-queue
 * + NQUEUE 24, see below). If that works, the BACK counter is zero and this
 * switch has nothing to do.
 *
 * If reordering still remains, this lets stale blocks be dropped: the
 * stream's time order is restored, at the cost of holes. NOT a clear
 * improvement - hence off by default; enable only after measurement.
 */
#define SEQ_DROP_STALE 0

/* ==================== REORDERING BY SEQUENCE NUMBER ====================
 *
 * MEASUREMENT 2026-08-03, on native USB (no SpyServer, i.e. under clean
 * conditions), counting the sequence numbers directly:
 *
 *     missing 168 / back  7 = 24        missing 240 / back 10 = 24
 *     missing 360 / back 15 = 24        missing  96 / back  4 = 24
 *
 * "Missing" is always EXACTLY 24 times the number of backward steps. It is
 * not 168 blocks lost: there are seven +25 / -23 "excursions", and the naive
 * counter books each as 24 missing. The balance of one excursion:
 *
 *     one block is lost FOR GOOD  (its buffer was overwritten before we read it)
 *     one block arrives TWICE     (once early, once in its place)
 *
 * That is why blocks/s matches production exactly, and why no counter showed
 * it. The offset is ALWAYS +-NQUEUE, i.e. BOUNDED - and what is bounded can
 * be corrected with a window.
 *
 * What this gives: the rest of the chain (APRS demod, SpyServer, rings) gets
 * samples in CORRECT time order, duplicates disappear, and a truly lost block
 * is replaced by 5 ms of silence. A known short hole is orders of magnitude
 * better than a 123 ms time jump: AFSK resynchronises across a hole, but
 * falls apart on a time jump.
 *
 * What it does NOT give: it does not fix the root cause. That remains open
 * (the interrupt priority was not it). But it is a working system meanwhile.
 *
 * Cost: REORDER_BLOCKS * 1040 bytes RAM, and the same number of blocks of
 * latency. 0 = disabled.
 *
 * SIZING: the window must cover the largest measured lateness distance,
 * which is tied to the queue: k*NQUEUE+-1, in practice at most NQUEUE+1.
 * Condition: RO_HOLD (= REORDER_BLOCKS-4) >= NQUEUE+1.
 *
 * 2026-08-07: NQUEUE went 16 -> 24, so the condition demands 25. A window of
 * 24 (RO_HOLD=20) is NO LONGER ENOUGH - hence 32 (RO_HOLD=28 >= 25, three
 * blocks of margin, the same reserve as before).
 * Latency 164 ms, cost +8.3 kB RAM.
 *
 * AS A BONUS A LATENT BUG DISAPPEARS. Below, ro_pump() forms the slot index
 * as 'ro_next % REORDER_BLOCKS', and ro_next is a FREE-RUNNING 32-bit
 * counter. 24 does not divide 2^32, so at counter wrap-around (~255 days of
 * continuous operation at 195 blocks/s) the index would have jumped -
 * exactly the bug we already guard against separately at rxring. 32 is a
 * power of two and divides 2^32, so the wrap-around stays correct.
 *
 * IF NQUEUE GROWS FURTHER: REORDER_BLOCKS >= NQUEUE + 5, and it must remain
 * a power of two. NQUEUE=32 -> REORDER_BLOCKS 64 (not 40!). */
#define REORDER_BLOCKS 32

/* At most this many transactions are taken per main loop iteration. The old
 * code took ONE per iteration, so the full housekeeping cost (TCP, OLED,
 * WiFi, APRS) was paid PER BLOCK - 195 times per second. Once behind, we
 * could only catch up by one block per iteration. */
#define DRAIN_MAX     NQUEUE

/* Sending several blocks at once is ALWAYS better than one by one: fewer USB
 * frames, fewer TCP segments, fewer system calls. usb_bench.ino measures
 * exactly this - if a larger chunk is better for you, raise AGG_BLK. */
#define AGG_BLK        4
#define AGG_FLUSH_MS   8u

/* ---- TCP ring buffers ----
 * WHY NEEDED: the lwIP send buffer is 5744 bytes by default. The rtl_tcp
 * branch makes 256 * rtl_up * 2 bytes from one block, which already exceeds
 * that at 12x upsampling - the "write only if the WHOLE thing fits" logic
 * then NEVER succeeds, and the port silently appears mute.
 *
 * The ring solves it: we write as much as currently fits, and advance the
 * tail by the number of bytes ACTUALLY written.
 *
 * SAMPLE-FRAME ALIGNMENT IS NOT PROTECTED HERE. It would be tempting to write
 * only whole sample frames, but that is fatal: write() may return any
 * amount, and once the tail lands on an odd position, rounding down would
 * yield 0 at the wrap-around, the loop would exit, and the ring would NEVER
 * drain again. The correct invariant lives elsewhere: ring_put ONLY inserts
 * WHOLE frames and ONLY drops WHOLE frames - so the byte stream always stays
 * aligned, however many pieces it goes out in.
 *
 * Sizes are powers of two: the uint32 head/tail wrap-around stays consistent
 * with the modulo only that way. The rtl ring needs more, because one
 * upsampled block is already 10 kB at a 20x factor. */
#define RING_RAW_BYTES 16384u
#define RING_RTL_BYTES 65536u
/* Upper bound of one send() call. MSS-ALIGNED: lwIP TCP_MSS in ESP-IDF is
 * 1440 bytes, and with NoDelay every write immediately becomes a segment.
 * The old 2048 split into one full 1440 and one short 608 - the half-size
 * segments wasted air time. 2880 = exactly 2 full segments. */
#define TCP_CHUNK_MAX  2880u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint16_t nsamp;
  uint16_t decim;
  uint32_t fs_in_hz;
} blk_hdr_t;

#if RDY_ENABLE
static void IRAM_ATTR spi_rdy_up(spi_slave_transaction_t *t)
{
  (void)t;
  GPIO.out_w1ts = (1u << RDY_PIN);      /* RDY = 1: there is room to write */
}
static void IRAM_ATTR spi_rdy_down(spi_slave_transaction_t *t)
{
  (void)t;
  GPIO.out_w1tc = (1u << RDY_PIN);      /* RDY = 0: no room right now */
}
#endif

/* Whether the SPI slave started at all. If not, the main loop does not even
 * try - otherwise the driver floods the console with thousands of error lines per second. */
static bool spi_ok = false;

WORD_ALIGNED_ATTR static uint8_t rxbuf[NQUEUE][BLK_BYTES];
static spi_slave_transaction_t   trans[NQUEUE];

/* ===================================================================
 *        DEDICATED SPI RECEIVE TASK - THE SPI IS INVIOLABLE (2026-08-05)
 * ===================================================================
 *
 * The CW splitter measurement (RTL-SDR vs FG23 simultaneously) showed in
 * black and white: the FG23 side is CLEAN (100708 blocks, 0 drops, 0 FIFO
 * overflow), the holes originate 100% HERE - ~1.3 per second, each EXACTLY
 * 1 block (5.12 ms), because the main loop (OLED ~22 ms + WiFi servicing)
 * sometimes stalls longer than the NQUEUE=16 queue bridges (80 ms; we
 * measured 88 ms dt_max).
 *
 * The fix is NOT another NQUEUE patch but architecture: reception moves to
 * its own HIGH-PRIORITY, CORE-PINNED task, which the FreeRTOS preemptive
 * scheduler admits immediately at ANY time - OLED/WiFi/print physically
 * cannot get in its way. The task's job is deliberately minimal:
 *
 *   get_trans_result (blocking, portMAX_DELAY)
 *     -> memcpy into the ring (~us)
 *     -> queue_trans IMMEDIATELY back to the DMA
 *
 * No demod, no print, no network - so the WCET is on the order of
 * microseconds, and the DMA queue never drains by more than 1-2 transactions.
 * The main loop (demod, APRS, SpyServer, OLED) consumes from the ring at its
 * own pace; if it stalls for 88 ms, that is now only LATENCY, not LOSS.
 *
 * Core choice: pinned to the same core where the SPI ISR lives
 * (spi_slave_initialize runs from setup() = Arduino core 1), so the
 * ISR->task wake-up is intra-core, no cross-core IPC latency.
 * loop() runs here too at prio 1 -> the task (prio 22) preempts it at any
 * time, even in the middle of the OLED I2C refresh. The WiFi stack stays on
 * core 0; we never meet it.
 *
 * The ring size bridges the main loop's longest stall:
 * 24 blocks = 123 ms @ 195 blocks/s (measured max: 88 ms + margin).
 * Cost: 24*1040 = ~25 kB RAM. SPSC ring, lock-free: only the task writes
 * head (release), only loop() writes tail (release) - the other side reads
 * with acquire.
 *
 * TWO SEPARATE PROTECTIONS, DO NOT CONFUSE THEM (2026-08-07):
 *   RXRING  - protects when the MAIN LOOP stalls (OLED, TCP, APRS). The task
 *             runs and fills the ring. Measured: ovf = 0 over the whole
 *             overnight measurement, so 24 blocks (123 ms) suffice.
 *   NQUEUE  - protects when THE TASK ITSELF does not run either (flash-cache
 *             stall on both cores). Then only the already queued transactions
 *             are consumed. This was 82 ms (=16), the measured stall 88 ms -
 *             hence 24, so that this is 123 ms too.
 * The two are now deliberately equal: the chain narrows nowhere. */
#define RXRING_BLOCKS   24
#define SPI_RX_TASK_PRIO 22
#define SPI_RX_TASK_CORE 1          /* = ARDUINO_RUNNING_CORE, the ISR's core */

/* NOTE on indexing: head/tail are FREE-RUNNING counters (the fill level
 * comes from 'head - tail', which is correct even on overflow), but the
 * SLOT INDEX must NOT be derived from them with '% RXRING_BLOCKS': 24 does
 * not divide 2^32, so at counter wrap-around (~255 days at 195 blocks/s!)
 * the index would jump and the ring would fall apart. Therefore each side
 * steps the slot index in its OWN variable and wraps it manually - verified
 * by the simulation test with 100 M blocks and several 2^32 crossings
 * (0 loss, correct order). */
WORD_ALIGNED_ATTR static uint8_t rxring[RXRING_BLOCKS][BLK_BYTES];
static uint16_t          rxring_len[RXRING_BLOCKS];
static volatile uint32_t rx_head = 0;      /* counter, written only by the task */
static volatile uint32_t rx_tail = 0;      /* counter, written only by the loop */
static uint32_t          rx_head_idx = 0;  /* slot, used ONLY by the task */
static uint32_t          rx_tail_idx = 0;  /* slot, used ONLY by the loop */
static volatile uint32_t rx_ring_ovf = 0;  /* ring full -> dropped block (must be 0!) */
static volatile uint32_t rx_ring_max = 0;  /* highest fill level (watermark) */
static TaskHandle_t      rx_task_handle = NULL;

static void spi_rx_task(void *arg)
{
  (void)arg;
  for (;;) {
    spi_slave_transaction_t *r = NULL;
    if (spi_slave_get_trans_result(FG23_SPI_HOST, &r, portMAX_DELAY) != ESP_OK
        || r == NULL)
      continue;

    size_t n = (size_t)(r->trans_len / 8);
    if (n > BLK_BYTES) n = BLK_BYTES;

    uint32_t head = rx_head;
    uint32_t used = head - __atomic_load_n(&rx_tail, __ATOMIC_ACQUIRE);
    if (used < RXRING_BLOCKS) {
      rxring_len[rx_head_idx] = (uint16_t)n;
      memcpy(rxring[rx_head_idx], r->rx_buffer, n);
      if (++rx_head_idx == RXRING_BLOCKS) rx_head_idx = 0;
      __atomic_store_n(&rx_head, head + 1u, __ATOMIC_RELEASE);
      if (used + 1u > rx_ring_max) rx_ring_max = used + 1u;
    } else {
      /* Ring full: the main loop has been stalled too long. The block is
       * lost, but the sequence chain flags it (SEQ-HOLE) - and the ovf
       * counter tells that it was US, not the SPI. This must be 0.
       * (Not '++': on a volatile that is deprecated since C++20.) */
      rx_ring_ovf = rx_ring_ovf + 1u;
    }

    /* The buffer goes back to the DMA IMMEDIATELY - that is the point. */
    spi_slave_queue_trans(FG23_SPI_HOST, r, portMAX_DELAY);
  }
}

#define DIAG  Serial0     /* CP2102: text diagnostics */
#define RAW   Serial      /* native USB CDC: raw stream */

/* ---------------- output aggregators ---------------- */
static uint8_t  agg_usb[AGG_BLK * BLK_BYTES];      static size_t agg_usb_n = 0;
#if RTL_ENABLE
static uint8_t  agg_rtl[BLK_SAMPLES * RTL_MAX_UP * 2];  /* scratch */
#else
static uint8_t  agg_rtl[4];                             /* stub */
#endif
static uint32_t last_flush_ms = 0;

/* ---------------- ring buffer for the two TCP outputs ---------------- */
typedef struct {
  uint8_t *buf;
  uint32_t size;          /* POWER OF TWO */
  uint32_t head;          /* write position */
  uint32_t tail;          /* read position */
  uint32_t dropped;       /* how many blocks' worth of data did not fit */
} ring_t;

static uint8_t ring_raw_buf[RING_RAW_BYTES];
static ring_t  ring_raw = { ring_raw_buf, RING_RAW_BYTES, 0, 0, 0 };
#if RTL_ENABLE
static uint8_t ring_rtl_buf[RING_RTL_BYTES];
static ring_t  ring_rtl = { ring_rtl_buf, RING_RTL_BYTES, 0, 0, 0 };
#else
/* Stub: the code paths compile unchanged but consume no RAM. A client can
 * never connect (the server is not started), so nothing meaningful is ever
 * written into these. */
static uint8_t ring_rtl_buf[16];
static ring_t  ring_rtl = { ring_rtl_buf, 16, 0, 0, 0 };
#endif

static inline uint32_t ring_used(const ring_t *r)
{
  return (r->head - r->tail);
}

/* Writes only if it FITS ENTIRELY. Partial writes are forbidden: in the
 * header-less stream half a sample would swap I and Q forever. This is the
 * ONLY place where alignment must be taken care of. */
static void ring_put(ring_t *r, const uint8_t *p, uint32_t n)
{
  if (ring_used(r) + n > r->size) { r->dropped++; return; }
  uint32_t idx = r->head & (r->size - 1u);
  uint32_t first = r->size - idx;
  if (first > n) first = n;
  memcpy(&r->buf[idx], p, first);
  if (n > first) memcpy(&r->buf[0], p + first, n - first);
  r->head += n;
}

/* Drains as much as currently fits. Advances the tail by the number of bytes
 * ACTUALLY written, so a partial write shifts nothing - and we do NOT round
 * down, because that would jam the ring forever at the wrap-around (see the
 * explanation above). */
static uint32_t ring_flush(ring_t *r, WiFiClient &c)
{
  /* WE NEVER BLOCK - and for that WiFiClient::write() CANNOT be used.
   *
   * 2026-08-03, Android spyserver measurement: the main loop stalled for
   * spy=90..330 ms, qmax=16/16, LOST poured in. Cause (from the core source):
   *   - availableForWrite() is NOT implemented in NetworkClient, it always
   *     returns 0 - the old "room" check was blind;
   *   - write() with a full send buffer waits in select() with a 1 SECOND
   *     timeout, 10 retries.
   * A power-saving phone drains every 100-300 ms, so write() regularly got
   * stuck waiting, and took the SPI down with it.
   *
   * The cure: direct send() with MSG_DONTWAIT. If the send buffer is full,
   * -1/EAGAIN comes IMMEDIATELY and we move on - the ring holds the data, the
   * tail advances by the bytes ACTUALLY written. */
  uint32_t sent = 0;
  const int fd = c.fd();
  if (fd < 0) return 0;
  while (ring_used(r) > 0) {
    uint32_t n = ring_used(r);
    uint32_t idx = r->tail & (r->size - 1u);
    uint32_t contig = r->size - idx;            /* up to the wrap-around */
    if (n > contig)        n = contig;
    if (n > TCP_CHUNK_MAX) n = TCP_CHUNK_MAX;

    int w = send(fd, &r->buf[idx], n, MSG_DONTWAIT);
    if (w <= 0) break;                          /* full: next iteration */
    r->tail += (uint32_t)w;
    sent += (uint32_t)w;
    if ((uint32_t)w < n) break;                 /* partial: buffer full */
  }
  return sent;
}

/* ---------------- network ---------------- */
static WiFiServer srv_raw(PORT_RAW);
static WiFiServer srv_rtl(PORT_RTLTCP);
static WiFiClient cli_raw;
static WiFiClient cli_rtl;
/* A SEPARATE flag is needed: WiFiClient::operator bool() in arduino-esp32
 * returns connected(), so "cli && !cli.connected()" is always false - the
 * disconnect would go unnoticed. */
static bool       have_raw = false, have_rtl = false;
static bool       wifi_up = false;
static bool       wifi_is_ap = false;
static IPAddress  my_ip;
static WiFiMulti  wifiMulti;
static uint32_t   wifi_last_try_ms = 0;

/* ---------------- statistics ---------------- */
static uint32_t st_blocks = 0, st_badmagic = 0, st_badlen = 0, st_lost = 0;

/* CUMULATIVE mirrors for wifi_bench. The counters above are zeroed at the end
 * of the 2 s status; the bench however forms deltas over 10+ s steps, and
 * without these it would get negative (wrapped) differences. Monotonic only. */
static uint32_t bench_tot_blocks = 0, bench_tot_lost = 0;
static uint32_t bench_tot_ro_dup = 0, bench_tot_ro_hole = 0;

/* The ACTUAL output sample rate (after decimation), from the 2 s status.
 * The block header only carries the INPUT rate (400 ksps), which is useless
 * for the panel's frequency scale. */
static uint32_t st_sps_out = 50000;
static uint32_t st_specline = 0;        /* received scan lines (SPECLINE) */
static uint32_t st_specline_ms = 0;     /* arrival of the last one */
static uint32_t st_specline_cf_hz = 0;  /* centre frequency of the last line */
static uint32_t st_specline_span_hz = 0;
#if OLED_ENABLE
static OledSpectrum oled_spec;          /* 128x64 mini spectrum + waterfall */
#endif
static uint32_t st_samples = 0;
static uint32_t st_usbdrop = 0;
static uint64_t st_usbbytes = 0, st_rawbytes = 0, st_rtlbytes = 0;
static int32_t  st_peak = 0;
static uint64_t st_abssum = 0;
static uint32_t st_counted = 0;
static int      st_rf_rssi = -128;   /* FG23 RAIL RSSI (dBm), from the header */

/* --- Signal level in dB. Peak/mean are raw int16 ADC magnitudes (full
 * scale = 32767). Relative to that, dBFS (0 = saturation, negative = below)
 * is immediately meaningful and exact even WITHOUT calibration.
 *
 * For REAL dBm calibrate once: apply a signal of known level (e.g. the
 * calibrated output of the HackRF harness + external attenuator), read the
 * dBFS, and compute: RX_FS_DBM = known_dBm - read_dBFS (the dBm that would
 * give full scale). Then set RX_CALIBRATED 1, and the display shows dBm.
 * While 0, the output is dBFS. */
#define RX_CALIBRATED  0
#define RX_FS_DBM      0.0f
#if RX_CALIBRATED
#define LVL_UNIT "dBm"
#else
#define LVL_UNIT "dBFS"
#endif
static inline float lvl_db(int32_t mag)
{
  if (mag < 1) mag = 1;
  float dbfs = 20.0f * log10f((float)mag / 32767.0f);
#if RX_CALIBRATED
  return dbfs + RX_FS_DBM;
#else
  return dbfs;
#endif
}
static uint32_t last_seq = 0;
static bool     have_seq = false;
static uint8_t  st_gap_shown = 0;
static uint32_t t0 = 0;

/* --- TIMING INVESTIGATION ---
 * Block loss is ALWAYS timing, never RF. So we do not guess: we measure how
 * long the main loop stalls, and in WHICH part.
 *
 * st_qmax  - the largest transaction backlog drained at once. If it reaches
 *            NQUEUE, data was just lost.
 * st_dtmax - the longest gap between two iterations [us]. The upper estimate
 *            of "how long we were blind".
 * sect_us  - per-section maximum [us]: which call eats the time. */
static uint32_t st_back = 0;

/* --- QUEUE FINGERPRINT ---
 * For every anomalous sequence step we decide whether it fits the
 * k * NQUEUE +- 1 formula. If almost all anomalies do, the fault is tied to
 * the transaction queue wrapping around - and that remains true even if
 * NQUEUE is changed. That is exactly the experiment. */
static uint32_t st_pool_hit = 0, st_pool_other = 0;

static inline void seq_classify(uint32_t step_abs)
{
  if (step_abs < 2u) return;
  uint32_t k = (step_abs + NQUEUE / 2u) / NQUEUE;
  if (k == 0u) k = 1u;
  uint32_t near = k * (uint32_t)NQUEUE;
  uint32_t diff = (step_abs > near) ? (step_abs - near) : (near - step_abs);
  if (diff <= 1u) st_pool_hit++;
  else            st_pool_other++;
}
static uint32_t st_qmax = 0;
static uint32_t st_dtmax = 0;
static uint32_t st_prev_us = 0;

enum { S_PROC = 0, S_TCP, S_USB, S_ACC, S_SPY, S_RTL, S_WIFI, S_UI,
       S_APRS, S_CW, S_OLED, S_COUNT };
static const char *SECT_NAME[S_COUNT] = {
  "proc", "tcp", "usb", "acc", "spy", "rtl", "wifi", "ui", "aprs", "cw", "oled" };
static uint32_t sect_us[S_COUNT];

/* Measurement of one section. A micros() call is ~1 us; ten per iteration
 * is negligible compared to what we are looking for (tens of ms). */
#define SECT(idx, ...) do {                                    \
    uint32_t _t = micros();                                    \
    __VA_ARGS__;                                               \
    uint32_t _d = micros() - _t;                               \
    if (_d > sect_us[idx]) sect_us[idx] = _d;                  \
  } while (0)

/* ---------------- stream parameters from the header ---------------- */
static uint32_t cur_fs_in = 0, cur_decim = 0, cur_sps = 0;
static uint32_t rtl_up = 1;                 /* upsampling factor */
static int      rtl_shift = 8;
static int32_t  rtl_peak_win = 0;
static uint32_t rtl_agc_ms = 0;
static int16_t  rtl_prev_i = 0, rtl_prev_q = 0;

/* Tuning request from the client: not forwarded immediately (see below). */
#define TUNE_RATE_MS   250u
static bool     rtl_pending = false;
static uint32_t rtl_pending_khz = 0;
static uint32_t rtl_last_tune_ms = 0;

/* Common entry point for tuning requests (rtl_tcp AND spyserver).
 * No C++ name mangling, so spy_tune_cb can reach it via extern. */
extern "C" bool rtl_pending_set(uint32_t khz);
extern "C" bool rtl_pending_set(uint32_t khz)
{
  /* SDR++ sends 0 Hz on connect, before the user sets anything. Forwarding
   * that would send "F0" to the FG23 - a meaningless command and a needless
   * stream restart. Only sensible frequencies are passed through. */
  if (khz < 100000u || khz > 1000000u) {
    DIAG.printf("\ntuning ignored: %lu kHz (out of band)\n",
                (unsigned long)khz);
    return false;
  }
  rtl_pending_khz = khz;
  rtl_pending = true;
  return true;
}

/* ===================================================================
 *                             OLED
 * =================================================================== */
#if OLED_ENABLE

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C
       u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);

static bool     oled_ok = false;
static uint32_t oled_last_ms = 0;
static uint32_t oled_khz = 0;        /* last known tuning, 0 = unknown */

static void oled_init(void)
{
#if (OLED_VEXT_PIN >= 0)
  /* Display power. Active LOW on the Heltec V3. */
  pinMode(OLED_VEXT_PIN, OUTPUT);
  digitalWrite(OLED_VEXT_PIN, OLED_VEXT_ACTIVE_LOW ? LOW : HIGH);
  delay(100);
#endif

  /* Reset is issued manually, not left to U8g2: this guarantees it happens
   * AFTER Vext is switched on, and it is in a visible place. */
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);   delay(20);
  digitalWrite(OLED_RST_PIN, HIGH);  delay(20);

  /* REAL presence check before calling U8g2. U8g2's begin() always returns
   * true - it does not look back at the bus. Without this you could not tell
   * "not connected" from "Vext not switched on", and a dark display looks
   * the same in both cases. */
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.beginTransmission(0x3C);
  oled_ok = (Wire.endTransmission() == 0);

  if (oled_ok) {
    u8g2.begin();
    u8g2.setBusClock(OLED_I2C_HZ);
    u8g2.setFontMode(1);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.drawStr(0, 12, "FG23 SDR");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 26, "HA7DCD  starting...");
    u8g2.sendBuffer();
  }
  DIAG.printf("OLED: %s at address 0x3C (SDA=%d SCL=%d RST=%d Vext=%d)\n",
              oled_ok ? "responds" : "NO RESPONSE",
              OLED_SDA_PIN, OLED_SCL_PIN, OLED_RST_PIN, OLED_VEXT_PIN);
  if (!oled_ok) {
    DIAG.println("  -> try: OLED_VEXT_ACTIVE_LOW inverted, or "
                 "OLED_VEXT_PIN = -1");
  }
}

/* ================== DISPLAY PAGES ==================
 * Every page has its own drawing function, and the table in ui.h says which
 * exist and in what order. A new demodulator gets a new drawing function
 * here and one line in the switch below. */

/* ================== TEXT OUTPUT THAT IS GUARANTEED TO FIT ==================
 * The display is 128 pixels wide. With the 6x10 font that is EXACTLY 21
 * characters (126 px) - the 22nd is already cut in half at the edge. Counting
 * every format string by hand is a bad idea: one longer callsign or a
 * four-digit counter and it is clipped again.
 *
 * So we do not count: the function measures the text, and if it does not fit
 * in 6x10, switches to 5x8 (25 characters), then 4x6 (32). If it still does
 * not fit, it is cut at a CHARACTER BOUNDARY, not mid-glyph. */
static void oled_fit(int x, int y, const char *s)
{
  const int avail = 128 - x;
  int len = (int)strlen(s);

  const uint8_t *font; int cw;
  if      (len * 6 <= avail) { font = u8g2_font_6x10_tf; cw = 6; }
  else if (len * 5 <= avail) { font = u8g2_font_5x8_tf;  cw = 5; }
  else                       { font = u8g2_font_4x6_tf;  cw = 4; }

  char tmp[48];
  int maxch = avail / cw;
  if (maxch > (int)sizeof tmp - 1) maxch = (int)sizeof tmp - 1;
  if (len > maxch) {
    memcpy(tmp, s, (size_t)maxch);
    tmp[maxch] = '\0';
    s = tmp;
  }
  u8g2.setFont(font);
  u8g2.drawStr(x, y, s);
}

/* Common header: page name on the left, mode indicator on the right. So on
 * any page you see WHAT IS RUNNING - not just what you are looking at. */
static void oled_header(const char *title)
{
  ui_page_t m = ui_mode();
  const char *tag = (m == UI_PAGE_STATUS) ? "---" : ui_page_name(m);

  /* The indicator is RIGHT-aligned by its actual pixel width - not estimated
   * from the character count. 5x8 is needed so that longer names (e.g.
   * "APRS iGATE") still leave room for the title. */
  u8g2.setFont(u8g2_font_5x8_tf);
  int tw = (int)u8g2.getStrWidth(tag);
  u8g2.drawStr(128 - tw, 8, tag);

  /* the title gets whatever room is left */
  char t[32];
  snprintf(t, sizeof t, "%s", title);
  int room = 128 - tw - 4;
  u8g2.setFont(u8g2_font_6x10_tf);
  while (t[0] && (int)u8g2.getStrWidth(t) > room) t[strlen(t) - 1] = '\0';
  u8g2.drawStr(0, 8, t);

  u8g2.drawHLine(0, 10, 128);
}

/* ---- STATUS: the start-up screen. The MOST IMPORTANT item is the IP, since
 * that is what has to be typed into SDR++. ---- */
static void oled_page_status(void)
{
  char buf[40];
  oled_header("FG23 SDR");

  if (wifi_is_ap) {
    snprintf(buf, sizeof buf, "AP %s", WIFI_AP_SSID);
  } else if (wifi_up) {
    snprintf(buf, sizeof buf, "%.13s %ddBm",
             WiFi.SSID().c_str(), (int)WiFi.RSSI());
  } else {
    snprintf(buf, sizeof buf, "no network");
  }
  oled_fit(0, 21, buf);

  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 35, wifi_up ? my_ip.toString().c_str() : "-");

#if SPY_ENABLE
  snprintf(buf, sizeof buf, "5555 spy%c 8888 1234",
           spy_connected() ? '+' : ' ');
#else
  snprintf(buf, sizeof buf, "8888 i16   1234 x%lu", (unsigned long)rtl_up);
#endif
  oled_fit(0, 46, buf);

  if (cur_sps) snprintf(buf, sizeof buf, "%lu sps", (unsigned long)cur_sps);
  else         snprintf(buf, sizeof buf, "no data");
  oled_fit(0, 57, buf);
  if (oled_khz) {
    snprintf(buf, sizeof buf, "%lu.%03lu",
             (unsigned long)(oled_khz / 1000), (unsigned long)(oled_khz % 1000));
    oled_fit(74, 57, buf);
  }
}

/* ---- RF: the PHYSICAL parameters. This was missing, and it is exactly what
 * matters most when the device is not next to the computer: where the
 * receiver is tuned, how wide the band is, and how much comes in. ---- */
static void oled_page_rf(void)
{
  char buf[40];

  /* During a scan (fresh SPECLINE < 2 s) the RF page = mini spectrum +
   * waterfall. draw() calls clearBuffer/sendBuffer itself, so we return
   * afterwards; the second sendBuffer in oled_draw() resends the same image. */
  if (st_specline && millis() - st_specline_ms < 2000u) {
    oled_spec.draw(u8g2);
    return;
  }

  oled_header("RF");

  /* frequency in large type, MHz with three decimals */
  u8g2.setFont(u8g2_font_7x13B_tr);
  if (oled_khz) {
    snprintf(buf, sizeof buf, "%lu.%03lu MHz",
             (unsigned long)(oled_khz / 1000), (unsigned long)(oled_khz % 1000));
  } else {
    snprintf(buf, sizeof buf, "-- MHz");
  }
  u8g2.drawStr(0, 24, buf);

  /* sample rate and the resulting bandwidth */
  snprintf(buf, sizeof buf, "%lu sps  R=%lu",
           (unsigned long)cur_sps, (unsigned long)cur_decim);
  oled_fit(0, 35, buf);

  snprintf(buf, sizeof buf, "bw +-%lu.%lu kHz  fs %lu k",
           (unsigned long)(cur_sps / 2000), (unsigned long)((cur_sps / 200) % 10),
           (unsigned long)(cur_fs_in / 1000));
  oled_fit(0, 45, buf);

  /* Levels. The peak is the MOST IMPORTANT number against overdrive: if it
   * reaches 32767, the phase sticks to the four corners and demodulation
   * becomes useless - even if the waterfall still looks fine. */
  uint32_t mean = st_counted ? (uint32_t)(st_abssum / st_counted) : 0;
  snprintf(buf, sizeof buf, "%+d dBm  %+.0f/%+.0f%s",
           st_rf_rssi, lvl_db(st_peak), lvl_db((int32_t)mean),
           (st_peak >= 32000) ? " T!" : "");
  oled_fit(0, 56, buf);
}

/* ---- SDR STREAM: the health of the chain on one screen. ---- */
static void oled_page_sdr(void)
{
  char buf[40];
  oled_header("SDR STREAM");

  u8g2.setFont(u8g2_font_7x13B_tr);
  snprintf(buf, sizeof buf, "%lu sps", (unsigned long)cur_sps);
  u8g2.drawStr(0, 24, buf);

  snprintf(buf, sizeof buf, "bw +-%lu.%lu kHz",
           (unsigned long)(cur_sps / 2000), (unsigned long)((cur_sps / 200) % 10));
  oled_fit(0, 35, buf);

  snprintf(buf, sizeof buf, "%+.0f " LVL_UNIT "  R=%lu",
           lvl_db(st_peak), (unsigned long)cur_decim);
  oled_fit(0, 46, buf);

#if SPY_ENABLE
  snprintf(buf, sizeof buf, "NET%c RTL%c SPY%c USB%c v%lu",
           have_raw ? '+' : '-', have_rtl ? '+' : '-',
           spy_connected() ? '+' : '-',
           (st_usbdrop == 0) ? '+' : '!', (unsigned long)st_lost);
#else
  snprintf(buf, sizeof buf, "NET%c RTL%c USB%c  v%lu",
           have_raw ? '+' : '-', have_rtl ? '+' : '-',
           (st_usbdrop == 0) ? '+' : '!', (unsigned long)st_lost);
#endif
  oled_fit(0, 57, buf);
}

#if APRS_RX_ENABLE
/* ---- APRS iGATE: the callsign in the largest type, since that is what matters. ---- */
static void oled_page_aprs(void)
{
  char buf[40];
  oled_header("APRS iGATE");

  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 24, aprs_rx_last_call()[0] ? aprs_rx_last_call()
                                             : "-- waiting --");

  /* The IS state goes HERE, at the end of the line - not in a separate
   * drawStr somewhere in the frame, because that would overlap the info
   * field as soon as it gets longer. This way the whole line goes through
   * oled_fit as one and is guaranteed to fit.
   *
   * SHORTENED 2026-08-07. The old text (full-word labels, e.g. received/error/gate
   * with the counters) was 28 characters. The oled_fit ladder works with 128 pixels at x=0:
   * 28*6=168 and 28*5=140 are both too much, so it fell back to the SMALLEST,
   * 4x6 font - whose lowercase x-height is 3 pixels, which is unreadable.
   * That was the cause of the "tiny lowercase" complaint, not lack of space.
   *
   * Below 21 characters 6x10 stays (21*6=126 <= 128), where the lowercase
   * x-height is 5 pixels - more than double. Hence short, COLON-SEPARATED
   * labels:
   *   RX = received frames,  H = CRC errors,  G = gated to APRS-IS.
   *
   * CHARACTER BUDGET (keep this if you rewrite the labels):
   *   "RX:1197 H:11 G:0 IS+"      = 20 characters -> 6x10  (well readable)
   *   "RX:99999 H:9999 G:999 IS+" = 25 characters -> 5x8   (still readable)
   * So typical operation stays in the larger font throughout and only steps
   * down one size at very large counters - it never falls to 4x6.
   *
   * A larger font (7x13B) does NOT go here: the line pitch is 11 pixels
   * (24/35/46/57), a 13-pixel glyph would overlap the callsign above. If a
   * larger font is ever needed, redesign the line pitch first. */
  snprintf(buf, sizeof buf, "RX:%lu H:%lu G:%lu %s",
           (unsigned long)aprs_rx_frames(), (unsigned long)aprs_rx_bad(),
           (unsigned long)aprs_rx_gated(),
           aprs_rx_is_online() ? "IS+" : "IS-");
  oled_fit(0, 35, buf);

  const char *info = aprs_rx_last_info();
  char l1[22], l2[22];
  snprintf(l1, sizeof l1, "%.21s", info);
  snprintf(l2, sizeof l2, "%.21s", (strlen(info) > 21) ? info + 21 : "");
  oled_fit(0, 46, l1);
  oled_fit(0, 57, l2);
}
#endif

/* ---- CW decoder page ---- */
static void oled_page_cw(void)
{
  char buf[40];
  oled_header("CW");

  /* The decoded text in the largest font */
  u8g2.setFont(u8g2_font_7x13B_tr);
  const char *txt = cw_rx_text();
  if (txt[0]) {
    /* if long, show the last 18 characters (scrolling) */
    size_t n = strlen(txt);
    const char *show = (n > 18) ? txt + (n - 18) : txt;
    u8g2.drawStr(0, 26, show);
  } else {
    u8g2.drawStr(0, 26, "-- waiting --");
  }

  /* WPM + key state + character counter */
  snprintf(buf, sizeof buf, "%.0f WPM  %s  ch:%lu",
           (double)cw_rx_wpm(),
           cw_rx_keyed() ? "KEY" : "   ",
           (unsigned long)cw_rx_chars());
  oled_fit(0, 40, buf);

  /* The full buffer in the bottom line, smaller font */
  oled_fit(0, 54, txt[0] ? txt : "(no signal yet)");
}

/* ---- SCAN: full-screen dithered spectrum + waterfall ---- */
static void oled_page_scan(void)
{
  /* Fresh SPECLINE (5 s): OledSpectrum fills the whole 128x64
   * (24 px curve + 40 px Bayer-dithered waterfall). */
  if (st_specline && (millis() - st_specline_ms) < 5000u) {
    oled_spec.draw(u8g2);
    return;
  }

  /* No active scan - instructions */
  oled_header("SCAN");
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 28, "no scan");
  oled_fit(0, 44, "W<kHz>,span,nbin");
  if (st_specline) {
    char buf[40];
    snprintf(buf, sizeof buf, "last %lu s",
             (unsigned long)((millis() - st_specline_ms) / 1000u));
    oled_fit(0, 56, buf);
  } else {
    oled_fit(0, 56, "FG23 / SpyServer");
  }
}

/* ---- Demodulators not yet implemented. No pretending: we state that it is
 * not ready, and what is missing. ---- */
static void oled_page_todo(ui_page_t p)
{
  oled_header(ui_page_name(p));
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 26, "not ready yet");
  switch (p) {
    case UI_PAGE_WSPR:
      oled_fit(0, 40, "planned: 2-minute");
      oled_fit(0, 51, "FFT + K1JT decoder,");
      oled_fit(0, 62, "needs exact clock (NTP)");
      break;
    case UI_PAGE_FT8:
      oled_fit(0, 40, "planned: 15 s cycle,");
      oled_fit(0, 51, "LDPC decoder - heavy");
      oled_fit(0, 62, "CPU and RAM load");
      break;
    default:
      break;
  }
}

static void oled_draw(void)
{
  if (!oled_ok) return;
  u8g2.clearBuffer();

  switch (ui_page()) {
    case UI_PAGE_RF:   oled_page_rf();   break;
    case UI_PAGE_SDR:  oled_page_sdr();  break;
    case UI_PAGE_SCAN: oled_page_scan(); break;
#if APRS_RX_ENABLE
    case UI_PAGE_APRS: oled_page_aprs(); break;
#endif
    case UI_PAGE_CW:   oled_page_cw();   break;
    case UI_PAGE_WSPR:
    case UI_PAGE_FT8:  oled_page_todo(ui_page()); break;
    default:           oled_page_status(); break;
  }

  u8g2.sendBuffer();
}

static void oled_tick(uint32_t now)
{
  if (!oled_ok) return;
  /* Faster refresh on the SCAN page (~3 Hz) so the waterfall flows */
  uint32_t period = (ui_page() == UI_PAGE_SCAN) ? 300u : OLED_REFRESH_MS;
  if (now - oled_last_ms < period) return;
  oled_last_ms = now;
  oled_draw();
}

/* Immediate redraw - after a button press there should be no wait for the
 * next period, otherwise the button would feel unresponsive. */
static void oled_force(void)
{
  if (!oled_ok) return;
  oled_last_ms = millis();
  oled_draw();
}

#else
static void oled_init(void) { }
static void oled_tick(uint32_t now) { (void)now; }
static void oled_force(void) { }
static uint32_t oled_khz = 0;
#endif  /* OLED_ENABLE */

/* ===================================================================
 *                          RETURN PATH
 * =================================================================== */

static void cmdlink_send(const char *s)
{
#if CMDLINK_ENABLE
  /* NO flush(): it would block until the last bit is out (~1 ms), stalling
   * the main loop meanwhile. Buffered write is enough - the FG23 does not
   * need it within a millisecond. */
  Serial1.print(s);
  Serial1.print("\r");
#endif
  DIAG.printf("  -> FG23: %s\n", s);
}

#if SPY_ENABLE
/* Tuning request from the SpyServer client. Goes through the SAME
 * rate-limited path as rtl_tcp's: of the requests accumulated while the
 * slider is dragged only the last one reaches the FG23. Otherwise every
 * pixel of movement would mean a stream restart. */
static void spy_tune_cb(uint32_t hz)
{
  rtl_pending_set((hz + 500u) / 1000u);
}

/* The SpyServer client requests scan mode (SDR++ fg23_scan_source: SCAN) or
 * stops it. Goes directly to the FG23 over the cmdlink - unlike tuning there
 * is no "slider dragging" here, so no rate limiting is needed. The FG23
 * stops the I/Q stream and scans on "W..."; on "W0" it reverts and the
 * stream restarts. */
static bool     scan_on = false;
static uint32_t scan_center_khz = 0, scan_span_khz = 0;
static uint16_t scan_nbin = 0;
/* RATE-LIMITED sending (2026-08-15): SDR++ issues several W commands per
 * second while the waterfall is dragged, and the FG23 EUSART FIFO (16 bytes)
 * overflowed -> corrupted lines (cmdlink reported 39 lines, 20 errors). Same
 * fix as for tuning: of the pending commands only the LAST goes out, at most
 * once per SCAN_RATE_MS. W0 goes immediately. */
#define SCAN_RATE_MS 300u
static bool     scan_pending = false;
static char     scan_pending_cmd[40];
static uint32_t scan_last_send_ms = 0;
static void spy_scan_cb(bool want, uint32_t center_hz, uint32_t span_hz,
                        uint16_t nbin, int16_t floor_dbm, uint16_t range_db)
{
  if (want) {
    uint32_t ck = (center_hz + 500u) / 1000u;
    uint32_t sk = (span_hz + 500u) / 1000u;
    if (ck < 100000u || ck > 1000000u) {
      DIAG.printf("\nscan: centre %lu kHz out of band, not started\n", (unsigned long)ck);
      return;
    }
    snprintf(scan_pending_cmd, sizeof scan_pending_cmd, "W%lu,%lu,%u,%d,%u",
             (unsigned long)ck, (unsigned long)sk, (unsigned)nbin,
             (int)floor_dbm, (unsigned)range_db);
    scan_pending = true;
    scan_on = true; scan_center_khz = ck; scan_span_khz = sk; scan_nbin = nbin;
    oled_khz = ck;
  } else {
    scan_pending = false;
    if (!scan_on) return;
    scan_on = false;
    DIAG.printf("\nscan: W0\n");
    cmdlink_send("W0");
    scan_last_send_ms = millis();
  }
}

/* From the main loop: rate-limited sending of the pending scan command. */
static void scan_flush(uint32_t now)
{
  if (!scan_pending) return;
  if (now - scan_last_send_ms < SCAN_RATE_MS) return;
  scan_pending = false;
  scan_last_send_ms = now;
  DIAG.printf("\nscan: %s\n", scan_pending_cmd);
  cmdlink_send(scan_pending_cmd);
}
#endif

/* ===================================================================
 *                            WIFI
 * =================================================================== */

/* Open own AP if no known network is in range. */
static void wifi_open_ap(void)
{
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
#if WIFI_HT40
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);   /* see wifi_start */
#endif
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  wifi_up = true; wifi_is_ap = true; my_ip = WiFi.softAPIP();
  DIAG.printf("WiFi AP: \"%s\"  password \"%s\"   IP %s\n",
              WIFI_AP_SSID, WIFI_AP_PASS, my_ip.toString().c_str());
}

/* FORWARD DECLARATION of the WiFi reconnect task.
 *
 * The actual definition and the detailed rationale are below, before
 * wifi_tick() - but wifi_start() ALREADY starts it here, so the name and the
 * macros must be visible by then. (2026-08-07: everything was first placed
 * below, and the compiler rightly complained that none of it was known yet
 * in wifi_start().) */
static volatile bool wifi_scan_req    = false;   /* main loop -> task */
static TaskHandle_t  wifi_task_handle = NULL;

#define WIFI_SCAN_TIMEOUT_MS 5000u
#define WIFI_TASK_PRIO       1                 /* low: disturbs nobody */
#define WIFI_TASK_CORE       0                 /* NOT the SPI/loop core! */

#if WIFI_ENABLE
static void wifi_task(void *arg);
#endif

static void wifi_start(void)
{
#if !WIFI_ENABLE
  DIAG.println("WiFi: disabled (WIFI_ENABLE=0)");
#else
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          /* REQUIRED: throughput is stable only without sleep */

#if WIFI_HT40
  /* See the WIFI_HT40 switch: OFF by default, because measurement showed it
   * multiplied the driver glitch without any gain. */
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
  DIAG.println("WiFi: requesting HT40 (EXPERIMENTAL - watch BACK!)");
#endif

  int n = 0;
  for (size_t i = 0; i < WIFI_AP_COUNT; i++) {
    if (WIFI_APS[i].ssid[0] == '\0') continue;   /* empty row: skipped */
    wifiMulti.addAP(WIFI_APS[i].ssid, WIFI_APS[i].pass);
    DIAG.printf("WiFi: known network #%d  \"%s\"\n", ++n, WIFI_APS[i].ssid);
  }

  if (n == 0) {
    DIAG.println("WiFi: no network in the table -> AP");
    wifi_open_ap();
  } else {
    DIAG.printf("WiFi: scanning, joining the strongest known network "
                "(max %lu ms)...\n", (unsigned long)WIFI_STA_TIMEOUT_MS);
    wifi_last_try_ms = millis();
    if (wifiMulti.run(WIFI_STA_TIMEOUT_MS) == WL_CONNECTED) {
      wifi_up = true; wifi_is_ap = false; my_ip = WiFi.localIP();
      DIAG.printf("WiFi STA: \"%s\"   IP %s   RSSI %d dBm\n",
                  WiFi.SSID().c_str(), my_ip.toString().c_str(),
                  (int)WiFi.RSSI());
      /* If the same AP drops and returns, the core handles it itself,
       * without blocking. WiFiMulti is only needed for the SELECTION. */
      WiFi.setAutoReconnect(true);
    } else {
      DIAG.println("WiFi: no known network reachable -> own AP");
      wifi_open_ap();
    }
  }

  /* The reconnect task. It is started even in AP mode: it idles, and
   * wifi_tick only signals it in STA mode anyway. If it does not start, we
   * do NOT silently fall back to the blocking path - we complain loudly,
   * because that variant demonstrably caused 5 seconds of data loss. */
  if (xTaskCreatePinnedToCore(wifi_task, "wifi_rc", 4096, NULL,
                              WIFI_TASK_PRIO, &wifi_task_handle,
                              WIFI_TASK_CORE) != pdPASS) {
    DIAG.println("!! wifi_rc task did NOT start - no reconnection!");
    wifi_task_handle = NULL;
  } else {
    DIAG.printf("WiFi: reconnect task running (core %d, prio %d) - "
                "scanning NO LONGER stalls the main loop\n",
                WIFI_TASK_CORE, WIFI_TASK_PRIO);
  }

  srv_raw.begin();  srv_raw.setNoDelay(true);
#if RTL_ENABLE
  srv_rtl.begin();  srv_rtl.setNoDelay(true);
#endif
#endif
}

/* ===================================================================
 *   WIFI RECONNECT - IN A SEPARATE TASK, NOT IN THE MAIN LOOP
 * ===================================================================
 *
 * WHY (MEASURED 2026-08-07). The old version called wifiMulti.run(5000)
 * from the main loop. That SCANS and BLOCKS until the given timeout. The
 * log of the 195-minute soak showed literally this:
 *
 *   TIMING: qmax=24/24  RXRING=24/24  ovf=959  dt_max=5030705 us
 *        | wifi=5029571 oled=20332
 *
 * The "wifi" section is 5 029 571 us = 5.03 s, and dt_max is practically
 * the same: the section did not CONTAIN the stall, it WAS the stall. The
 * configured 5000 ms timeout is clearly reflected in it.
 *
 * The damage: the dedicated SPI task fills the 24-block ring in 123 ms,
 * then ovf. Two such events in 195 minutes took 959 + 939 blocks
 * (4.9 s + 4.8 s), and 6 consecutive APRS frames each.
 *
 * WHAT NOT TO BELIEVE: that a deeper buffer solves this. 5 seconds is
 * FORTY TIMES 123 ms. No NQUEUE or RXRING bridges that - the blocking call
 * MUST LEAVE the main loop. (The source already predicted this for the
 * blocking APRS-IS connect too.)
 *
 * THE SOLUTION. wifiMulti's "join the strongest of the known ones" logic is
 * valuable, so it is not discarded but relocated:
 *
 *   - the main loop (wifi_tick) ONLY SETS A FLAG, calls nothing that could
 *     block;
 *   - the actual scan runs in wifi_task, on CORE 0. The main loop and the
 *     SPI task are both on core 1 (ARDUINO_RUNNING_CORE=1, SPI_RX_TASK_CORE
 *     1, prio 22), and WiFi/lwIP is on core 0 anyway.
 *   - the task may block for 5 seconds without concern: the main loop keeps
 *     draining the ring meanwhile, so ovf does not grow.
 *
 * IMPORTANT, WHAT THIS DOES NOT REPLACE: WiFi.setAutoReconnect(true) (see
 * wifi_setup) still handles the case where the SAME AP drops and returns -
 * the core does that itself, without blocking. The scan is only needed
 * when switching to ANOTHER known network.
 *
 * DIAG is written by two tasks (main loop + this one). HardwareSerial has a
 * TX ring buffer and the lines are short, but in theory they can interleave.
 * If you ever see a mixed line in the log, this is the reason - not data
 * corruption. */
/* (wifi_scan_req, wifi_task_handle and the WIFI_TASK_* macros are declared
 *  above, before wifi_start() - the task is started from there.) */
#if WIFI_ENABLE
static void wifi_task(void *arg)
{
  (void)arg;
  for (;;) {
    if (wifi_scan_req) {
      DIAG.println("\nWiFi: connection lost, rescanning... (in separate task)");
      if (wifiMulti.run(WIFI_SCAN_TIMEOUT_MS) == WL_CONNECTED) {
        my_ip = WiFi.localIP();
        DIAG.printf("WiFi back: \"%s\"   IP %s   RSSI %d dBm\n",
                    WiFi.SSID().c_str(), my_ip.toString().c_str(),
                    (int)WiFi.RSSI());
      } else {
        DIAG.println("WiFi: still no known network");
      }
      wifi_scan_req = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#endif

/* From the main loop. Only DECIDES and SIGNALS - never blocks. */
static void wifi_tick(uint32_t now)
{
#if WIFI_ENABLE
  if (wifi_is_ap) return;                       /* nothing to do in AP mode */
  if (WiFi.status() == WL_CONNECTED) return;
  if (have_raw || have_rtl) return;
  if (wifi_scan_req) return;                    /* an attempt is already running */
  if (now - wifi_last_try_ms < WIFI_RETRY_MS) return;

  wifi_last_try_ms = now;
  wifi_scan_req    = true;                      /* that is all. We do not wait for it. */
#else
  (void)now;
#endif
}

/* ===================================================================
 *                    rtl_tcp PROTOCOL
 * =================================================================== */

static void rtl_send_header(WiFiClient &c)
{
  /* 12 bytes: "RTL0" + tuner type (big-endian) + number of gain steps.
   * We pose as an R820T: every client knows that one. */
  uint8_t h[12] = { 'R','T','L','0',
                    0,0,0,5,        /* RTLSDR_TUNER_R820T */
                    0,0,0,29 };     /* 29 gain steps */
  c.write(h, sizeof h);
}

/* State of the anti-image moving average. Its length = rtl_up, so its zeros
 * fall exactly on the images. */
static int32_t  ma_hist_i[RTL_MAX_UP], ma_hist_q[RTL_MAX_UP];
static int32_t  ma_sum_i = 0, ma_sum_q = 0;
static uint32_t ma_idx = 0;

static void rtl_reset_filter(void)
{
  memset(ma_hist_i, 0, sizeof ma_hist_i);
  memset(ma_hist_q, 0, sizeof ma_hist_q);
  ma_sum_i = ma_sum_q = 0;
  ma_idx = 0;
  rtl_prev_i = rtl_prev_q = 0;
}

static void rtl_recalc_up(void)
{
#if (RTL_TARGET_SPS > 0)
  if (cur_sps == 0) { rtl_up = 1; rtl_reset_filter(); return; }
  uint32_t u = (RTL_TARGET_SPS + cur_sps / 2) / cur_sps;
  if (u < 1) u = 1;
  if (u > RTL_MAX_UP) u = RTL_MAX_UP;
  rtl_up = u;
#else
  rtl_up = 1;
#endif
  /* The filter length changed - the old history is invalid. */
  rtl_reset_filter();
}

/* Client commands: 5 bytes, 1 byte code + 4 bytes BIG-ENDIAN parameter. */
static void rtl_poll_commands(void)
{
  while (cli_rtl && cli_rtl.available() >= 5) {
    uint8_t b[5];
    cli_rtl.read(b, 5);
    uint32_t p = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 8)  |  (uint32_t)b[4];

    switch (b[0]) {
      case 0x01: {                       /* SET_FREQUENCY (Hz) */
        /* While the slider is dragged the client sends this by the dozen.
         * Each command means a stream stop, retune and restart on the FG23
         * (a few tenths of a second), so they must NOT all be forwarded:
         * only the last one, and that at most every few hundred ms. */
        rtl_pending_khz = (p + 500u) / 1000u;
        rtl_pending = true;
        break;
      }
      case 0x02:                         /* SET_SAMPLE_RATE */
        DIAG.printf("\nrtl_tcp: client requests %lu sps; we provide %lu sps "
                    "upsampled %lux = %lu sps\n",
                    (unsigned long)p, (unsigned long)cur_sps,
                    (unsigned long)rtl_up,
                    (unsigned long)(cur_sps * rtl_up));
        break;
      case 0x03: case 0x08:              /* gain mode / AGC mode */
        DIAG.printf("\nrtl_tcp: gain/AGC mode %lu (software AGC is used "
                    "here)\n", (unsigned long)p);
        break;
      case 0x04: case 0x0d:              /* SET_GAIN / gain by index */
        DIAG.printf("\nrtl_tcp: gain %lu (ignored)\n",
                    (unsigned long)p);
        break;
      case 0x05:                         /* SET_FREQ_CORRECTION (ppm) */
        DIAG.printf("\nrtl_tcp: ppm %ld (the FG23 has its own "
                    "calibration)\n", (long)(int32_t)p);
        break;
      default:
        DIAG.printf("\nrtl_tcp: unknown command 0x%02X = %lu\n",
                    b[0], (unsigned long)p);
        break;
    }
  }
}

/* int16 I/Q -> 8-bit offset binary, upsampled by an integer factor
 * (linear interpolation) on the way. Returns: number of bytes produced. */
static size_t rtl_convert(const int16_t *iq, int nsamp, uint8_t *out)
{
  size_t o = 0;
  const int sh = rtl_shift;

  for (int k = 0; k < nsamp; k++) {
    int16_t ci = iq[2 * k], cq = iq[2 * k + 1];

    /* The peak is checked on BOTH branches. Watching only I is wrong
     * because after the DC block a band-edge tone can sit practically
     * purely in Q - then the AGC would report headroom while clipping. */
    int32_t a = ci; if (a < 0) a = -a;
    if (a > rtl_peak_win) rtl_peak_win = a;
    int32_t aq = cq; if (aq < 0) aq = -aq;
    if (aq > rtl_peak_win) rtl_peak_win = aq;

    for (uint32_t u = 0; u < rtl_up; u++) {
      /* linear transition between the previous and the current sample */
      int32_t wi = ((int32_t)rtl_prev_i * (int32_t)(rtl_up - u)
                  + (int32_t)ci * (int32_t)u) / (int32_t)rtl_up;
      int32_t wq = ((int32_t)rtl_prev_q * (int32_t)(rtl_up - u)
                  + (int32_t)cq * (int32_t)u) / (int32_t)rtl_up;

#if RTL_ANTI_IMAGE
      if (rtl_up > 1) {
        /* Running average of length rtl_up: places another zero in the
         * middle of every image. Two additions and one division per sample. */
        ma_sum_i += wi - ma_hist_i[ma_idx];  ma_hist_i[ma_idx] = wi;
        ma_sum_q += wq - ma_hist_q[ma_idx];  ma_hist_q[ma_idx] = wq;
        if (++ma_idx >= rtl_up) ma_idx = 0;
        wi = ma_sum_i / (int32_t)rtl_up;
        wq = ma_sum_q / (int32_t)rtl_up;
      }
#endif

      /* ROUNDING, not truncation! The arithmetic right shift truncates
       * negative numbers downwards (-1 >> 5 = -1, not 0), causing a
       * systematic -0.5 LSB offset on EVERY sample on average - a constant,
       * huge DC line in the 8-bit stream ("atomic flash in the middle",
       * 2026-08-03). Adding half an LSB before the shift gives symmetric,
       * zero-mean rounding. */
      const int32_t rnd = (sh > 0) ? (1 << (sh - 1)) : 0;
      int32_t vi = ((wi + rnd) >> sh) + 128;
      int32_t vq = ((wq + rnd) >> sh) + 128;
      if (vi < 0) vi = 0; else if (vi > 255) vi = 255;
      if (vq < 0) vq = 0; else if (vq > 255) vq = 255;
      out[o++] = (uint8_t)vi;
      out[o++] = (uint8_t)vq;
    }
    rtl_prev_i = ci;
    rtl_prev_q = cq;
  }
  return o;
}

static void rtl_agc_tick(uint32_t now)
{
#if (RTL_FIXED_SHIFT >= 0)
  rtl_shift = RTL_FIXED_SHIFT;
  (void)now;
#else
  if (now - rtl_agc_ms < RTL_AGC_MS) return;
  rtl_agc_ms = now;

  /* WIDE DEADBAND. We only step if the peak projected to 8 bits really
   * leaves the [LO..HI] band - in between we do not touch it. Every step
   * moves the WHOLE picture by 6 dB, which shows as a horizontal band on the
   * waterfall; the previous version regulating to a narrow target therefore
   * stepped continuously and striped the waterfall. */
  if (rtl_peak_win > 0) {
    int32_t peak8 = rtl_peak_win >> rtl_shift;
    if (peak8 > RTL_AGC_HI && rtl_shift < 15) {
      rtl_shift++;
      DIAG.printf("\nrtl_tcp AGC: attenuate, shift=%d (peak was %ld)\n",
                  rtl_shift, (long)peak8);
    } else if (peak8 < RTL_AGC_LO && rtl_shift > 0) {
      rtl_shift--;
      DIAG.printf("\nrtl_tcp AGC: amplify, shift=%d (peak was %ld)\n",
                  rtl_shift, (long)peak8);
    }
  }
  rtl_peak_win = 0;
#endif
}

/* ===================================================================
 *                        FLUSHING THE OUTPUTS
 * =================================================================== */

/* HOW LONG WRITING ONE BLOCK MAY WAIT [ms]. 0 = no waiting at all.
 *
 * WHY NOT 0 (2026-08-06): the old code only wrote when the CDC TX buffer had
 * room for a WHOLE block. With HWCDC that is fine (8 kB buffer), but with the
 * TinyUSB USBCDC the FIFO size is CONFIG_TINYUSB_CDC_TX_BUFSIZE - typically
 * SMALLER than 1040 bytes. Then the condition NEVER holds, so EVERY block is
 * dropped: exactly the "USB 0.0 kB/s(-248)" symptom, where the PC received
 * ZERO bytes in 60 seconds.
 *
 * write() itself paces: it puts as much into the FIFO as fits, waits for the
 * host to take it, and continues - up to the given time. Block size and FIFO
 * size thus become independent of each other.
 *
 * WHY WAITING IS ALLOWED (without violating the "cast in stone" rule): SPI
 * reception is NO LONGER in the main loop but in a dedicated task (core 1,
 * prio 22), backed by a 24-block ring = 122 ms of bridging. loop() is prio 1
 * - the task preempts it at any time. A few ms of waiting in loop() therefore
 * CANNOT lose an SPI block; the ring absorbs it easily. (If it does anyway:
 * RXRING and the ovf counter show it IMMEDIATELY in the status line.)
 *
 * With no host connected we do NOT wait at all - see the (bool)RAW branch. */
#define USB_TX_TIMEOUT_MS   3u

static void flush_usb(void)
{
  if (!agg_usb_n) return;

  /* NO HOST -> immediate drop, no waiting. Otherwise every flush would waste
   * the full timeout even when nobody reads the USB (the normal state: the
   * measurement tool only runs occasionally). */
  if (!(bool)RAW) {
    st_usbdrop++;
    agg_usb_n = 0;
    return;
  }

  /* We write PER BLOCK, not the whole aggregator at once (2026-08-06). Two
   * reasons:
   *
   *   1) The old version requested the FULL agg_usb_n (4*1040 = 4160 bytes)
   *      at once. That works with HWCDC (8 kB TX buffer there), but with the
   *      TinyUSB USBCDC (ARDUINO_USB_MODE=0) the CDC TX FIFO size is
   *      CONFIG_TINYUSB_CDC_TX_BUFSIZE - typically much smaller. There
   *      "is there room for 4160 bytes?" would NEVER hold, so EVERYTHING
   *      would be dropped: exactly the symptom (USB 0.0 kB/s, all drops) we
   *      want to eliminate. Per block only 1040 bytes are needed.
   *
   *   2) Writing the WHOLE block atomically is however MANDATORY: the PC
   *      synchronises on the "IQB2" magic, half a block would cause a
   *      framing error, and the tool would see it as a FALSE SPI HOLE.
   *      Hence we cut at block boundaries.
   *
   * If a drop happens here, the status line's (-N) counter shows it - the
   * PC-side "SPI HOLE" is a clean measurement only when this number is 0. */
  size_t off = 0;
  while (off < agg_usb_n) {
    /* write() outputs the WHOLE block regardless of FIFO size, in several
     * pieces if necessary, waiting up to USB_TX_TIMEOUT_MS. Hence we do NOT
     * query availableForWrite() beforehand: that very query stalled TinyUSB
     * operation, where the FIFO is smaller than a block. */
    /* ===== 2026-08-09: WE DO QUERY AFTER ALL, FOR A DIFFERENT REASON =====
     *
     * The reasoning above was true for TinyUSB (ARDUINO_USB_MODE=0), where
     * the CDC FIFO is smaller than a block, so the query would always have
     * said no. On HWCDC (the current build) however the TX buffer is 8256
     * bytes - as long as nobody listens. As soon as the serial monitor
     * ATTACHES and drains slowly, afw sticks at 976, which is LESS than one
     * block (1040), and then RAW.write() waits the full timeout on EVERY block.
     *
     * MEASURED (from the log): after flashing, with the monitor attached:
     *     USB: conn=1 afw=976   usb=60245 us   proc=122814 us
     *     loop 1/s   dt_max=2169172 us   ovf 380 -> 5576
     * the same firmware after power-up, WITHOUT monitor:
     *     USB: conn=0 afw=8256  loop 1180/s  dt_max=21 ms  ovf=0
     *
     * That is why it appeared "after every flash" - after flashing the
     * monitor is always attached. The button did not respond either, since
     * ui_tick() runs in this loop.
     *
     * The raw I/Q USB path is NOT critical: if there is no room, drop the
     * block and move on. The (-N) counter shows it anyway. Reception and the
     * demod however must NEVER stop because of a terminal. */
    if (RAW.availableForWrite() < (int)BLK_BYTES) { st_usbdrop++; break; }
    size_t w = RAW.write(agg_usb + off, BLK_BYTES);
    st_usbbytes += w;
    if (w != BLK_BYTES) {
      /* SHORT write: the host froze or is slow. The remainder is dropped. The
       * PC resynchronises on the next magic and books the missing sequence
       * number as 1 hole - so the PC-side SPI HOLE measurement is only clean
       * when this counter reads (-0). */
      st_usbdrop++;
      break;
    }
    off += BLK_BYTES;
  }
  agg_usb_n = 0;
}

static void flush_tcp(void)
{
  if (have_raw && cli_raw.connected()) st_rawbytes += ring_flush(&ring_raw,
                                                                 cli_raw);
  if (have_rtl && cli_rtl.connected()) st_rtlbytes += ring_flush(&ring_rtl,
                                                                 cli_rtl);
}

/* ===================================================================
 *                          BLOCK PROCESSING
 * =================================================================== */

/* The ACTUAL processing. Blocks arrive here already in CORRECT time order -
 * see the reorderer in process(). */
static void deliver(const uint8_t *b)
{
  const blk_hdr_t *h = (const blk_hdr_t *)b;
  st_blocks++;
  st_samples += h->nsamp;

  /* Narrowband waterfall on the panel: FFT from every Nth block. The module
   * decimates itself (default 12 -> ~16 lines/s), and if a scan is running
   * we let that win, since it is the wider picture. */
  if (!st_specline || (millis() - st_specline_ms) > 3000u) {
    iq_fft_push_block(b + HDR_BYTES);
    /* NOTE: h->fs_in_hz is the FG23 INPUT rate (400 ksps), NOT the rate
     * after decimation. The scale needs the ACTUAL output rate, otherwise
     * the panel would show +-200 kHz instead of 50 ksps. */
    iq_fft_set_tuning(oled_khz * 1000u, st_sps_out);
  }

  /* The FG23 packs the RAIL RSSI (dBm) into the UPPER 12 bits of fs_in_hz -
   * the header has no separate field, and we do not touch the block size.
   * Lower 20 bits = real rate, upper 12 bits = signed dBm. MUST BE MASKED,
   * otherwise the fluctuating RSSI would constantly signal a "rate change"
   * and reconfigure. */
  uint32_t fs_in = h->fs_in_hz & 0x000FFFFFu;
  {
    int rf = (int)((h->fs_in_hz >> 20) & 0xFFFu);
    if (rf & 0x800) rf -= 0x1000;          /* 12-bit sign extension */
    st_rf_rssi = rf;                        /* dBm, from the FG23 RAIL */
  }

  /* DEADBAND: reconfigure only on a SUBSTANTIAL change. The measured rate
   * naturally fluctuates by a few tenths of a percent; if we recomputed the
   * filters and printed two log lines on every tiny deviation, the main loop
   * would stall and blocks would be lost. Exactly that happened. */
  uint32_t d_fs = (fs_in > cur_fs_in) ? (fs_in - cur_fs_in)
                                      : (cur_fs_in - fs_in);
  bool fs_changed = (cur_fs_in == 0) || (d_fs * 100u > cur_fs_in);  /* >1% */
  if (fs_changed || h->decim != cur_decim) {
    cur_fs_in = fs_in;
    cur_decim = h->decim;
    cur_sps   = cur_decim ? (cur_fs_in / cur_decim) : 0;
    rtl_recalc_up();
#if SPY_ENABLE
    spy_set_rate(cur_sps);
#endif
    DIAG.printf("\nstream: fs_in=%lu decim=%lu -> %lu sps   "
                "(rtl_tcp: %lux -> %lu sps)\n",
                (unsigned long)cur_fs_in, (unsigned long)cur_decim,
                (unsigned long)cur_sps, (unsigned long)rtl_up,
                (unsigned long)(cur_sps * rtl_up));
  }

  /* The sample count is taken from the header, not the define: a short block
   * would otherwise read 1024 bytes of old buffer content. */
  int nsamp = (int)h->nsamp;
  if (nsamp <= 0 || nsamp > BLK_SAMPLES) nsamp = BLK_SAMPLES;

  const int16_t *iq = (const int16_t *)(b + HDR_BYTES);
  for (int k = 0; k < nsamp * 2; k++) {
    int32_t a = abs((int32_t)iq[k]);
    if (a > st_peak) st_peak = a;
    st_abssum += (uint32_t)a;
    st_counted++;
  }

  /* --- APRS demodulator. It gets the RAW int16 samples, before any further
   * processing: no AGC, no 8-bit truncation, no interpolation spoils the
   * demodulation. --- */
#if APRS_RX_ENABLE
  /* Demodulate ONLY when the APRS mode is active. So the button really
   * frees the CPU for another measurement. */
  if (ui_mode() == UI_PAGE_APRS) aprs_rx_feed(iq, nsamp, cur_sps);
#endif
  /* CW: the same raw int16 I/Q, it only needs the envelope */
  if (ui_mode() == UI_PAGE_CW) cw_rx_feed(iq, nsamp, cur_sps);

  /* --- (a) USB: the WHOLE block goes, header included. The PC side strips
   * the header; in return it can resynchronise on the magic at any time,
   * and it also sees the loss from the sequence number. --- */
  if (agg_usb_n + BLK_BYTES <= sizeof agg_usb) {
    memcpy(agg_usb + agg_usb_n, b, BLK_BYTES);
    agg_usb_n += BLK_BYTES;
  }
  if (agg_usb_n >= sizeof agg_usb) flush_usb();

  /* --- (b) TCP 8888: ONLY the payload, so the SDR++ Network source can
   * consume it directly. With no header there is nothing to strip. --- */
  if (have_raw && cli_raw.connected()) {
    ring_put(&ring_raw, b + HDR_BYTES, (uint32_t)nsamp * 4u);
  }

  /* --- (d) TCP 5555: SpyServer, RAW 16-bit. The same block, without any
   * conversion - this is the best-quality path. --- */
#if SPY_ENABLE
  spy_feed(iq, nsamp);
#endif

  /* --- (c) TCP 1234: rtl_tcp, 8-bit, upsampled. --- */
  if (have_rtl && cli_rtl.connected()) {
    size_t n = rtl_convert(iq, nsamp, agg_rtl);
    ring_put(&ring_rtl, agg_rtl, (uint32_t)n);
  }

  /* Sending is not done here: the main loop drains the ring, as much as
   * currently fits. So a slow network does not stop block processing, and
   * the SPI transactions do not run out. */
}


/* ===================================================================
 *              SEQUENCE CHECK AND REORDERING
 * ===================================================================
 *
 * Blocks arrive here AS the SPI delivered them - in any order. From here
 * they continue to deliver(), IN TIME ORDER.
 * Detailed rationale at REORDER_BLOCKS.
 */

#if REORDER_BLOCKS > 0
static uint8_t  ro_buf[REORDER_BLOCKS][BLK_BYTES];
static uint32_t ro_seq[REORDER_BLOCKS];
static bool     ro_full[REORDER_BLOCKS];
static uint32_t ro_next = 0;          /* the next sequence number to emit */
static uint32_t ro_high = 0;          /* the highest sequence number seen */
static bool     ro_armed = false;

/* This many blocks are held back. The emission RATE thus equals the arrival
 * rate: for every new block exactly one is emitted. The old "whatever is
 * ready, immediately" logic pushed out 20-30 blocks at once after a hole -
 * the proc section jumped from 1.2 ms to 8.5 ms, and THAT was the audible
 * stutter. */
#define RO_HOLD  (REORDER_BLOCKS - 4)

/* COMPILE-TIME GUARD: the condition used to live in a comment, and on
 * 2026-08-07 we almost forgot it when NQUEUE went from 16 to 24. If anyone
 * touches either again, the build stops here, not on the test bench.
 *   - RO_HOLD >= NQUEUE+1 : the window must cover the largest possible
 *     lateness distance, which is tied to the queue (k*NQUEUE +- 1).
 *   - power of two : ro_pump() indexes with 'ro_next % REORDER_BLOCKS', and
 *     ro_next is a free-running 32-bit counter. If REORDER_BLOCKS does not
 *     divide 2^32, the index jumps at wrap-around (~255 days). */
static_assert(RO_HOLD >= NQUEUE + 1,
              "REORDER_BLOCKS too small for NQUEUE: RO_HOLD >= NQUEUE+1 required");
static_assert((REORDER_BLOCKS & (REORDER_BLOCKS - 1)) == 0,
              "REORDER_BLOCKS must be a power of two (ro_next % REORDER_BLOCKS)");

static uint32_t st_ro_dup = 0, st_ro_hole = 0, st_ro_late = 0;
static uint32_t ro_late_run = 0;   /* consecutive "late" drops */

/* ============ DUPLICATE VERIFICATION (2026-08-07 evening) ============
 *
 * WHY NEEDED. According to the explanation above (around the reordering
 * section) the cause of the k*NQUEUE+-1 phenomenon is that the hardware
 * writes into a descriptor that is ALREADY CLOSED but NOT YET READ - i.e.
 * the buffer content is overwritten before we read it. If so, then when the
 * same sequence number arrives twice, the CONTENT of the two copies DIFFERS:
 * one is the real block, the other an overwritten buffer whose header alone
 * is right.
 *
 * So far `st_ro_dup` SILENTLY dropped the second copy, so it never became
 * clear which one we kept. If we keep the wrong copy, the REORDER "fix" is
 * not a fix but HIDDEN DATA CORRUPTION: the demod gets samples that look
 * flawless but are wrong.
 *
 * THE EXPERIMENT. On a duplicate we compare the PAYLOAD (without the
 * header, which matches by definition). Two outcomes:
 *
 *   dupE (equal)     - the block really arrived twice, unchanged. Dropping
 *                      is correct, the chain is clean.
 *   dupK (different) - two DIFFERENT contents with the same sequence number.
 *                      Then at least one is bogus, and the buffer-overwrite
 *                      explanation holds. This number should be 0; if it is
 *                      not, REORDER must not be considered "solved".
 *
 * COST: the memcmp runs ONLY on duplicates (normally 0..1 per 2 s), and
 * comparing 1024 bytes takes ~2 us. Unmeasurable for the main loop. */
static uint32_t st_ro_dup_same = 0;   /* dupE - identical content */
static uint32_t st_ro_dup_diff = 0;   /* dupK - DIFFERENT content (must be 0) */
static uint8_t  st_dup_shown   = 0;   /* at most 3 detailed lines per run */

/* A lost block is replaced by SILENCE so the timeline does not slip. That is
 * the point: a known 5 ms hole is far less harmful than a 123 ms time jump -
 * AFSK resynchronises across the hole. */
static void ro_emit_hole(void)
{
  static uint8_t hole[BLK_BYTES];
  blk_hdr_t *h = (blk_hdr_t *)hole;
  h->magic = IQ_BLK_MAGIC;
  h->seq   = ro_next;
  h->nsamp = BLK_SAMPLES;
  h->decim = (uint16_t)(cur_decim ? cur_decim : 8);
  h->fs_in_hz = cur_fs_in ? cur_fs_in : 400000u;
  memset(hole + HDR_BYTES, 0, PAYLOAD_BYTES);
  st_ro_hole++;
  deliver(hole);
}

/* CONSTANT-LATENCY emission. As long as at least RO_HOLD blocks lie between
 * the newest sequence number and the next one to emit, we emit one - either
 * the buffered block or a hole. So there is never a backlog and never a burst. */
static void ro_pump(void)
{
  int guard = 6;                      /* at most this many per call */
  while (guard-- > 0 &&
         (int32_t)(ro_high - ro_next) >= (int32_t)RO_HOLD) {
    uint32_t i = ro_next % REORDER_BLOCKS;
    if (ro_full[i] && ro_seq[i] == ro_next) {
      ro_full[i] = false;
      deliver(ro_buf[i]);
    } else {
      ro_emit_hole();
    }
    ro_next++;
  }
}
#endif

static void process(const uint8_t *b, size_t nbytes)
{
  if (nbytes != BLK_BYTES) { st_badlen++; return; }
  const blk_hdr_t *h = (const blk_hdr_t *)b;

  /* SPECLINE (scan line) - same block size, different magic. Not I/Q: it
   * does not go to the demod, the raw TCP or rtl_tcp; it does go to the
   * SpyServer FFT stream and the display. No sequence bookkeeping for it
   * (there is no I/Q stream during a scan, seq diagnostics would be meaningless). */
  if (h->magic == SPECLINE_MAGIC) {
    const specline_blk_t *L = (const specline_blk_t *)b;
    if (L->hdr.nbin >= 1 && L->hdr.nbin <= SPECLINE_MAX_BINS) {
      st_specline++;
      st_specline_ms = millis();
      st_specline_cf_hz   = L->hdr.f_center_hz;
      st_specline_span_hz = L->span_hz;
#if SPY_ENABLE
      spy_send_specline(L);
#endif
#if OLED_ENABLE
      oled_spec.push(L->bins, L->hdr.nbin);
#endif
      /* The same line into the 240x320 panel's waterfall. Non-blocking: it
       * copies and notifies the display task; if that is still busy, the
       * line is dropped. The display must NEVER slow down the radio. */
      tft_set_autoscale(true);   /* the scan sends an absolute dBm scale */
      tft_push_specline(L->bins, L->hdr.nbin);
      tft_set_span(L->hdr.f_center_hz, L->span_hz);
    }
    return;
  }

  if (h->magic != IQ_BLK_MAGIC) { st_badmagic++; return; }

  /* --- diagnostics: what happened to the order (unchanged metrics) --- */
  if (have_seq) {
    uint32_t d = h->seq - last_seq;
    if (d >= 0x80000000u) {
      st_back++;
      seq_classify(last_seq - h->seq);
      if (st_gap_shown < 3) {
        st_gap_shown++;
        DIAG.printf("\nSEQ-BACK: %lu -> %lu\n",
                    (unsigned long)last_seq, (unsigned long)h->seq);
      }
#if SEQ_DROP_STALE
      return;
#endif
    }
    if (d > 1u && d < 100000u) {
      st_lost += (d - 1u);
      seq_classify(d);
      if (st_gap_shown < 3) {
        st_gap_shown++;
        DIAG.printf("\nSEQ-HOLE: %lu -> %lu (d=%lu)\n",
                    (unsigned long)last_seq, (unsigned long)h->seq,
                    (unsigned long)d);
      }
    }
  }
  last_seq = h->seq;
  have_seq = true;

#if REORDER_BLOCKS <= 0
  deliver(b);
#else
  if (!ro_armed) { ro_armed = true; ro_next = ro_high = h->seq; }

  /* ===== RESTART DETECTION =====
   *
   * THIS WAS MISSING, and it killed the system on 2026-08-03. The FG23
   * RESETS the sequence number when the stream restarts. The old code then
   * saw every block as "arrived late" (the unsigned subtraction wraps), and
   * dropped all of them FOR GOOD: late=390 per 2 seconds, which is exactly
   * ALL blocks. The stream never restarted.
   *
   * Lateness is separated from a restart by magnitude: the measured offset
   * is at most +-25 blocks, so anything farther than 1000 in either
   * direction is not lateness but a new start. */
  uint32_t rel  = h->seq - ro_next;         /* unsigned: the past is large */
  uint32_t back = ro_next - h->seq;
  bool restart = false;

  if (rel >= 0x80000000u) {                 /* came from the past */
    /* seq==0 is ALWAYS a restart - the FG23 only emits zero then. Caught by
     * the live run of 2026-08-03: on a restart after 819 blocks the backward
     * step stayed below 1000, the code treated it as lateness and dropped
     * the fresh blocks for 2 seconds, until the sequence caught up with the old one. */
    if (back > 1000u || h->seq == 0u) {
      restart = true;
    } else {
      st_ro_late++;
      /* SELF-HEALING: if "late" blocks are dropped in a row, that is not
       * lateness but a slipped state - whatever caused it, after 50 blocks
       * (~0.26 s) we resynchronise to the incoming sequence number. */
      if (++ro_late_run >= 50u) restart = true;
      else return;
    }
  } else if (rel > 1000u) {
    restart = true;                         /* too far ahead */
  }
  ro_late_run = 0;

  if (restart) {
    for (int k = 0; k < REORDER_BLOCKS; k++) ro_full[k] = false;
    ro_next = ro_high = h->seq;
    DIAG.printf("\nREORDER: restart, sequence continues from %lu\n",
                (unsigned long)h->seq);
  }

  uint32_t i = h->seq % REORDER_BLOCKS;
  if (ro_full[i] && ro_seq[i] == h->seq) {
    st_ro_dup++;
    /* Only the payload is compared: the header (magic/seq/nsamp/decim/
     * fs_in_hz) matches by definition, but the upper 12 bits of fs_in_hz
     * carry the RSSI, which may change per block - that is not corruption. */
    if (memcmp(ro_buf[i] + HDR_BYTES, b + HDR_BYTES, PAYLOAD_BYTES) == 0) {
      st_ro_dup_same++;
    } else {
      st_ro_dup_diff++;
      if (st_dup_shown < 3) {
        st_dup_shown++;
        /* The position of the FIRST difference is telling: if it starts at
         * byte 0, the two buffers are entirely different blocks; if later,
         * the overwrite caught the block MID-WAY - i.e. we read into a running DMA. */
        size_t off = 0;
        const uint8_t *p = ro_buf[i] + HDR_BYTES, *q = b + HDR_BYTES;
        while (off < PAYLOAD_BYTES && p[off] == q[off]) off++;
        DIAG.printf("\nREORDER-DUPK: seq=%lu differs from byte %u "
                    "(%02X!=%02X) - the buffer was overwritten\n",
                    (unsigned long)h->seq, (unsigned)off,
                    p[off], q[off]);
      }
    }
    return;
  }
  memcpy(ro_buf[i], b, BLK_BYTES);
  ro_seq[i]  = h->seq;
  ro_full[i] = true;
  if ((int32_t)(h->seq - ro_high) > 0) ro_high = h->seq;
  ro_pump();
#endif
}

/* ===================================================================
 *                    WIFI CEILING METER (wifi_bench)
 * ===================================================================
 * Exposes the chain's own counters to the bench. The per-step delta is
 * formed by the module. IMPORTANT: spi_lost gets st_ro_hole, NOT st_lost
 * (LOST) - the latter over-counts ~16x due to the driver glitch, which
 * would make the ceiling look falsely low.
 *
 * The console: flashlog is disabled (FLASHLOG_ENABLE 0), so NOBODY but us
 * reads Serial0 - no conflict. */
static void bench_ext_cb(wb_ext_t *o)
{
  /* The status counters are zeroed every 2 s, so the CUMULATIVE mirror plus
   * the current, not yet zeroed part is the true value. */
  o->spi_lost       = bench_tot_ro_hole + st_ro_hole;
  o->spi_dup        = bench_tot_ro_dup  + st_ro_dup;
  o->spi_ovf        = rx_ring_ovf;
  o->spi_blocks     = bench_tot_blocks  + st_blocks;
  o->spi_ring_max   = rx_ring_max;
  o->loop_dt_max_us = st_dtmax;
#if SPY_ENABLE
  o->spy_drop       = spy_dropped();
#else
  o->spy_drop       = 0;
#endif
}

static void bench_console_tick(void)
{
  static char cmd[80];
  static uint8_t n = 0;
  while (DIAG.available()) {
    char c = (char)DIAG.read();
    if (c == '\n' || c == '\r') {
      if (n) {
        cmd[n] = '\0';
        if (!wifi_bench_cmd(cmd) && !rgb_load_cmd(cmd)) iq_fft_cmd(cmd);
      }
      n = 0;
    } else if (n < sizeof cmd - 1) {
      cmd[n++] = c;
    } else {
      n = 0;                    /* overflow: drop the line */
    }
  }
}

/* ===================================================================
 *                              SETUP
 * =================================================================== */

void setup()
{
  /* HardwareSerial has NO TX ring buffer by default: write() blocks until
   * the bytes are in the hardware FIFO. A 180-character statistics line at
   * 115200 is ~16 ms - in that time three blocks arrive on i8, and if no SPI
   * transaction is queued, they are lost FOR GOOD. Buffer + faster baud, and
   * the output becomes "free". */
  DIAG.setTxBufferSize(2048);
  DIAG.begin(115200);

  /* The native USB CDC TX buffer is a few hundred bytes by default, which
   * cannot even hold one 1040-byte block - this is where the whole chain
   * failed first (USB 0.0 kB/s, every block dropped). Must be set BEFORE begin(). */
#if !FG23_USB_TINYUSB
  /* This method belongs to HWCDC. The TinyUSB USBCDC has no setTxBufferSize()
   * (the CDC TX FIFO size comes from CONFIG_TINYUSB_CDC_TX_BUFSIZE) - the
   * build would fail on it there, hence the condition. If you see drops in
   * the status line in MODE=0, add to the pioarduino custom_sdkconfig:
   *   CONFIG_TINYUSB_CDC_TX_BUFSIZE=4096
   * (same place as CONFIG_SPI_SLAVE_ISR_IN_IRAM=y). */
  RAW.setTxBufferSize(AGG_BLK * BLK_BYTES + 4096);
#endif
  RAW.setTxTimeoutMs(USB_TX_TIMEOUT_MS);   /* see the explanation above flush_usb() */
#if FG23_USB_TINYUSB
  /* DISABLE THE AUTOMATIC RESET ON DTR/RTS. Without this every port open on
   * the PC restarts the ESP (rst:0x15), the USB re-enumerates, and the
   * host's already open handle becomes a zombie -> the measurement gets no
   * data. ONLY the TinyUSB USBCDC can do this; HWCDC (MODE=1) has no such
   * method, hence the compile-time condition. Must be called BEFORE begin(). */
  RAW.enableReboot(false);
#endif
  RAW.begin();
#if FG23_USB_TINYUSB
  /* With TinyUSB the stack must be started explicitly. Idempotent: if the
   * core already started it (ARDUINO_USB_CDC_ON_BOOT=1), this does nothing. */
  USB.begin();
#endif
  delay(400);

  oled_init();

  DIAG.println("\n=== FG23 -> ESP32-S3 -> USB + WiFi + rtl_tcp ===");
  /* Make it VISIBLE in the log which USB mode is running - the two behave
   * quite differently with the PC-side measurement tool. */
#if FG23_USB_TINYUSB
  DIAG.println("USB: TinyUSB CDC (MODE=0), DTR/RTS reset DISABLED "
               "- recommended for measurement");
#else
  DIAG.println("USB: HWCDC / USB-Serial-JTAG (MODE=1) - WARNING: DTR/RTS "
               "RESETS the chip (rst:0x15).");
  DIAG.println("     If the PC-side tool receives no data, build with "
               "-D ARDUINO_USB_MODE=0.");
#endif
  DIAG.printf("SPI: SCLK=GPIO%d  MOSI=GPIO%d  CS=GPIO%d\n",
              (int)PIN_SCLK, (int)PIN_MOSI, (int)PIN_CS);

#if CMDLINK_ENABLE
  Serial1.setTxBufferSize(256);
  Serial1.begin(CMDLINK_BAUD, SERIAL_8N1, -1, CMDLINK_TX_PIN);
  DIAG.printf("CMD:  GPIO%d -> FG23 PA06 / EXP 11 (%d baud) - tuning "
              "from the phone\n", CMDLINK_TX_PIN, CMDLINK_BAUD);
#endif

  spi_bus_config_t bus;
  memset(&bus, 0, sizeof(bus));
  bus.mosi_io_num     = PIN_MOSI;
  bus.miso_io_num     = -1;
  bus.sclk_io_num     = PIN_SCLK;
  bus.quadwp_io_num   = -1;
  bus.quadhd_io_num   = -1;
  bus.max_transfer_sz = BLK_BYTES;
  /* ================= THE INTERRUPT PRIORITY =================
   *
   * 2026-08-03: proven that the size of the sequence jumps follows NQUEUE
   * EXACTLY (12 -> 13/25, 24 -> 25/49/-23, 17 -> 18/35/-16, POOL=100%).
   * The queue does NOT run dry meanwhile (qmax typically 1..5 of 17), so it
   * is not a capacity fault but a TIMING fault.
   *
   * The mechanism: the driver queues the next transaction IN AN INTERRUPT
   * after the previous one completes. The FG23 sends blindly, and between
   * two blocks there is only ~1.46 ms of pause (CS is low for 3656 us of
   * the 5120 us period). If the interrupt does not run within that, the
   * hardware writes the incoming block into the OLD descriptor - i.e. into
   * a buffer whose completion is already in the return queue but which we
   * have not read yet. When we read it, we get data one full buffer cycle
   * newer. Hence the k*NQUEUE +-1.
   *
   * That is why the fault grows with network traffic: lwIP and WiFi work on
   * the same core and delay the interrupt.
   *
   * The fix: LEVEL3 priority (WiFi is typically LEVEL1) and IRAM, so that a
   * flash operation cannot block it either. The IDF spi_slave ISR is
   * IRAM-safe (CONFIG_SPI_SLAVE_ISR_IN_IRAM is enabled by default).
   *
   * If that is still not enough, the next step is BLK_SAMPLES 256 -> 512
   * (half as many transactions, twice the time to queue) - but that must be
   * changed on the FG23 TOO. After that, the hardware RDY line.
   *
   * THE ACTUAL FLAGS ARE NOT SET HERE but below, tried in a ladder - see the
   * initialisation. */

  spi_slave_interface_config_t slv;
  memset(&slv, 0, sizeof(slv));
  slv.mode         = 0;
  slv.spics_io_num = PIN_CS;
  slv.queue_size   = NQUEUE;
#if RDY_ENABLE
  /* The two callbacks run from ISR, hence DIRECT register writes, no
   * function calls. Their meaning is a hardware fact:
   *   post_setup = descriptor loaded, the slave can receive  -> RDY high
   *   post_trans = the transaction has completed             -> RDY low
   * Between the two the driver ISR queues the next one; until that has
   * happened the pin is low and the FG23 waits. */
  slv.post_setup_cb = spi_rdy_up;
  slv.post_trans_cb = spi_rdy_down;
#endif

#if RDY_ENABLE
  /* The pin must be an output and low BEFORE queuing - so the FG23 sees a
   * correct state even during boot. */
  pinMode(RDY_PIN, OUTPUT);
  digitalWrite(RDY_PIN, LOW);
  DIAG.printf("RDY: GPIO%d -> FG23 PD2 / EXP 9 (post_setup/post_trans)\n",
              RDY_PIN);
#endif

  /* ===== INTERRUPT PRIORITY: A LADDER, NOT AN ASSUMPTION =====
   *
   * 2026-08-03: with ESP_INTR_FLAG_IRAM spi_slave_initialize returned an
   * ERROR (in the given Arduino build CONFIG_SPI_SLAVE_ISR_IN_IRAM is not
   * enabled), and the error was not checked - afterwards every
   * get_trans_result complained "host not slave", and the stream never
   * started. Lesson: never assume a platform-dependent capability; try it,
   * and print what was obtained.
   *
   * The ladder goes from best to safe. The first one that succeeds stays. */
  static const struct { int flags; const char *nev; } INTR_TRY[] = {
    { ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3, "IRAM + LEVEL3"   },
    { ESP_INTR_FLAG_LEVEL3,                      "LEVEL3"          },
    { ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2, "IRAM + LEVEL2"   },
    { ESP_INTR_FLAG_LEVEL2,                      "LEVEL2"          },
    { 0,                                         "default"         },
  };
  esp_err_t e = ESP_FAIL;
  for (unsigned k = 0; k < sizeof INTR_TRY / sizeof INTR_TRY[0]; k++) {
    bus.intr_flags = INTR_TRY[k].flags;
    e = spi_slave_initialize(FG23_SPI_HOST, &bus, &slv, SPI_DMA_CH_AUTO);
    DIAG.printf("spi_slave_initialize (%-16s): %d%s\n",
                INTR_TRY[k].nev, (int)e,
                (e == ESP_OK) ? "   <-- IN USE" : "");
    if (e == ESP_OK) break;
  }
  spi_ok = (e == ESP_OK);

  if (!spi_ok) {
    /* Do NOT keep spinning: get_trans_result would flood the console with
     * thousands of error messages per second and hide everything else. */
    DIAG.println("!! the SPI slave did NOT start - there will be no stream.");
    DIAG.println("!! the rest (WiFi, display, terminal) continues.");
  } else {
    for (int i = 0; i < NQUEUE; i++) {
      memset(&trans[i], 0, sizeof(trans[i]));
      trans[i].length    = BLK_BYTES * 8;
      trans[i].rx_buffer = rxbuf[i];
      spi_slave_queue_trans(FG23_SPI_HOST, &trans[i], portMAX_DELAY);
    }

    /* The dedicated receive task - see the long comment at its definition.
     * If it does not start, we do NOT silently fall back to the old
     * (main-loop) path: that model was deliberately removed because it
     * demonstrably causes block loss. Complain loudly instead. */
    BaseType_t tok = xTaskCreatePinnedToCore(
        spi_rx_task, "spi_rx", 4096, NULL,
        SPI_RX_TASK_PRIO, &rx_task_handle, SPI_RX_TASK_CORE);
    if (tok != pdPASS) {
      spi_ok = false;
      DIAG.println("!! spi_rx task did NOT start - stream disabled!");
    } else {
      DIAG.printf("SPI receive: dedicated task, core %d, prio %d, ring %d blocks "
                  "(%d ms bridging)\n",
                  SPI_RX_TASK_CORE, SPI_RX_TASK_PRIO, RXRING_BLOCKS,
                  RXRING_BLOCKS * 512 * 10 / 1000);
      /* The two bridging depths protect DIFFERENT things (see RXRING_BLOCKS)
       * and are deliberately equal since 2026-08-07. Printed so that the
       * bootlog reveals which binary was measured. */
      DIAG.printf("SPI queue: %d transactions (%d ms bridging during flash "
                  "stall), reorder window %d blocks\n",
                  NQUEUE, NQUEUE * 512 * 10 / 1000, REORDER_BLOCKS);
    }
  }

  wifi_start();

  DIAG.println("\n--- HOW TO CONNECT ---");
  /* IMPORTANT: the String must go into a VARIABLE. my_ip.toString().c_str()
   * points into a temporary that is destroyed at the end of the expression -
   * printf would already point at garbage. */
  String ips = wifi_up ? my_ip.toString() : String("<no wifi>");
  DIAG.println("SDR++ desktop, WITHOUT python:");
  DIAG.printf("   Source=Network   TCP / Client / %s : %d / Int16\n",
              ips.c_str(), PORT_RAW);
  DIAG.println("SDR++ Android (THE RECOMMENDED PATH, 16-bit, native rate):");
  DIAG.printf("   Source=SpyServer  %s : %d\n", ips.c_str(), PORT_SPY);
#if RTL_ENABLE
  DIAG.println("SDR++ Android / SDR Touch (8-bit, fallback path):");
  DIAG.printf("   Source=RTL-TCP    %s : %d\n", ips.c_str(), PORT_RTLTCP);
#else
  DIAG.println("rtl_tcp (1234): DISABLED (RTL_ENABLE 0) - +75 kB RAM");
#endif
  ui_init();
#if SPY_ENABLE
  spy_init(PORT_SPY, spy_tune_cb, spy_scan_cb);
#endif
#if APRS_RX_ENABLE
  aprs_rx_init();
#endif
  cw_rx_init();

  DIAG.println("Over USB (PC-side decoder + SPI loss measurement):");
  DIAG.println("   python usb_aprs_rx.py --period 1.131 --seconds 90");
#if REORDER_BLOCKS > 0
  DIAG.printf("\nReordering: %d-block window by sequence number "
              "(%.0f ms latency)\n", REORDER_BLOCKS,
              REORDER_BLOCKS * 256.0 * 1000.0 / 50000.0);
#else
  DIAG.println("\nReordering: OFF");
#endif
  DIAG.printf("RAM at start: %lu kB free heap (largest block %lu kB)\n",
              (unsigned long)(esp_get_free_heap_size() / 1024u),
              (unsigned long)(heap_caps_get_largest_free_block(
                                  MALLOC_CAP_8BIT) / 1024u));
  DIAG.println("The FG23 starts with autostart - no need to send 'i32'.");

#if FLASHLOG_ENABLE
  flashlog_init();
#endif

  /* WiFi throughput ceiling meter. Opens its own task and port (7777), but
   * ONLY on the 'B...' command - does nothing at start-up. */
  wifi_bench_init();
  wifi_bench_set_ext(bench_ext_cb);
  DIAG.println("WiFi-bench: 'B' = status, 'Bs200,2000,200,20' = ramp, "
               "'B0' = stop  (sink: wifi_sink.py, port 7777)");

  /* WS2812 load indicator. The LED cycles colours faster the more loaded the
   * CPU is. Some clones use pin 38, not 48. */
  rgb_load_init(RGB_LED_PIN);
  DIAG.println("RGB: 'L' = status, 'Ltest' = colour cycle, 'Lset fenyero=20' etc.");

  /* ILI9341 status display. SPI2 (IO_MUX 9-14) - the FG23 slave is on SPI3,
   * so there is no bus conflict. TE is not connected (47 belongs to the RGB LED). */
  tft_init();

  /* Put the IP on the display immediately, no 1.5 s wait for it. */
  oled_tick(millis() + OLED_REFRESH_MS);
}

/* ===================================================================
 *                               LOOP
 * =================================================================== */

static void accept_clients(uint32_t now)
{
  if (!wifi_up) return;

  /* Rate-limited: accept() is an lwIP call, while the main loop runs
   * thousands of times per second. Polling two servers that often wastes
   * CPU needlessly and holds the lwIP lock - which leads to block loss. */
  static uint32_t last_accept = 0;
  if (now - last_accept < 250u) {
    /* the disconnect IS checked every iteration, that is cheap */
    if (have_raw && !cli_raw.connected()) {
      DIAG.println("\n<<< raw client disconnected");
      cli_raw.stop(); have_raw = false;
    }
    if (have_rtl && !cli_rtl.connected()) {
      DIAG.println("\n<<< rtl_tcp client disconnected");
      cli_rtl.stop(); have_rtl = false;
    }
    return;
  }
  last_accept = now;

  /* accept(), not available(): in arduino-esp32 3.x NetworkServer's
   * available() is a deprecated alias and warns. It does the same thing.
   * If you ever build on an older core, this one word is the way back. */
  WiFiClient c = srv_raw.accept();
  if (c) {
    if (have_raw) cli_raw.stop();
    cli_raw = c;
    cli_raw.setNoDelay(true);
    have_raw = true;
    ring_raw.head = ring_raw.tail = 0;
    /* availableForWrite is printed: if it returns 0, your core does not
     * implement it, and the ring writes "blind". One number that saves an
     * hour when the port goes silent one day. */
    DIAG.printf("\n>>> RAW client: %s   (%lu sps, Int16, "
                "availableForWrite=%d)\n",
                cli_raw.remoteIP().toString().c_str(),
                (unsigned long)cur_sps, (int)cli_raw.availableForWrite());
  }

#if RTL_ENABLE
  WiFiClient r = srv_rtl.accept();
  if (r) {
    if (have_rtl) cli_rtl.stop();
    cli_rtl = r;
    cli_rtl.setNoDelay(true);
    have_rtl = true;
    ring_rtl.head = ring_rtl.tail = 0;
    rtl_send_header(cli_rtl);
    rtl_prev_i = rtl_prev_q = 0;
    DIAG.printf("\n>>> rtl_tcp client: %s   (%lux upsampling -> "
                "%lu sps)\n",
                cli_rtl.remoteIP().toString().c_str(),
                (unsigned long)rtl_up, (unsigned long)(cur_sps * rtl_up));
  }
#endif

  /* Check our own flag, not "cli_raw": the WiFiClient bool operator returns
   * connected(), so "cli && !cli.connected()" would always be false and we
   * would never see the disconnect. */
  if (have_raw && !cli_raw.connected()) {
    DIAG.println("\n<<< raw client disconnected");
    cli_raw.stop();
    have_raw = false;
  }
  if (have_rtl && !cli_rtl.connected()) {
    DIAG.println("\n<<< rtl_tcp client disconnected");
    cli_rtl.stop();
    have_rtl = false;
  }
}

static uint32_t st_loops = 0;

void loop()
{
  st_loops++;

  /* WiFi ceiling meter: 'B...' commands from the DIAG console. Non-blocking,
   * and only does anything when a character has arrived. */
  bench_console_tick();

  /* --- loop time measurement. The longest gap between two iterations is
   * the best single number for how long we were blind. --- */
  {
    uint32_t nu = micros();
    if (st_prev_us) {
      uint32_t dt = nu - st_prev_us;
      if (dt > st_dtmax) st_dtmax = dt;
    }
    st_prev_us = nu;
  }

  /* ===============================================================
   *            SPI: FULL DRAIN, IMMEDIATE RE-QUEUE
   * ===============================================================
   *
   * Two changes compared to the old code, and both matter:
   *
   * 1. NOT one block per iteration, but AS MANY AS THERE ARE. The old code
   *    could work off a backlog only one block per iteration, so after a
   *    single 60 ms stall it kept chasing itself for another tenth of a
   *    second - and meanwhile the queue filled up again.
   *
   * 2. The buffer is re-queued IMMEDIATELY, BEFORE processing. Previously
   *    the transaction was taken out of the queue for the whole duration of
   *    process() (APRS demod, peak search, copying into three rings, maybe
   *    a USB flush). A 1040-byte memcpy is a few microseconds - in return
   *    the hardware ALWAYS has somewhere to write.
   *
   * The copy is needed because processing may no longer use the buffer we
   * have just handed back to the DMA. */
  /* ===============================================================
   *   SPI: RECEPTION NO LONGER HAPPENS HERE (spi_rx_task, own core).
   *   Here we only CONSUME from the ring - if this loop stalls for 88 ms
   *   (OLED, WiFi, whatever), that is LATENCY, not LOSS.
   * =============================================================== */
  /* During setup() (WiFi scan ~10 s!) the task already receives but nobody
   * consumes yet - the ring fills and ovf counts. That loss is inherent to
   * boot (it was lost before too, just in the DMA queue), not an operational
   * fault. Zeroed in the first iteration so that ovf really measures losses
   * DURING OPERATION - which must be 0 forever. */
  static bool ovf_boot_reset = false;
  if (!ovf_boot_reset) { ovf_boot_reset = true; rx_ring_ovf = 0; rx_ring_max = 0; }

  uint32_t drained = 0;
  for (;;) {
    uint32_t head = __atomic_load_n(&rx_head, __ATOMIC_ACQUIRE);
    if (rx_tail == head) break;
    SECT(S_PROC, process(rxring[rx_tail_idx], rxring_len[rx_tail_idx]));
    if (++rx_tail_idx == RXRING_BLOCKS) rx_tail_idx = 0;
    __atomic_store_n(&rx_tail, rx_tail + 1u, __ATOMIC_RELEASE);
    if (++drained >= DRAIN_MAX) break;
  }
  if (drained > st_qmax) st_qmax = drained;
  /* Pacing: formerly provided by the 2 ms wait of get_trans_result. If the
   * ring is empty (or there is no SPI), release the core for one tick -
   * otherwise the loop would spin blindly at full speed. */
  if (drained == 0) delay(spi_ok ? 1 : 2);

  uint32_t now = millis();

  /* Timed flush: if few blocks arrive (large R), data must not sit in the
   * aggregator for seconds. */
  /* The TCP rings are drained EVERY iteration (not only on a timer): the
   * network takes as much as it can, and nothing piles up. */
  SECT(S_TCP, flush_tcp());

  if (now - last_flush_ms >= AGG_FLUSH_MS) {
    SECT(S_USB, flush_usb());
    last_flush_ms = now;
  }

  SECT(S_ACC, accept_clients(now));
#if SPY_ENABLE
  SECT(S_SPY, spy_tick(now));
#endif
  SECT(S_RTL, { rtl_poll_commands(); rtl_agc_tick(now); });
  SECT(S_WIFI, wifi_tick(now));
  SECT(S_UI, ui_tick(now));
#if APRS_RX_ENABLE
  /* CONNECTION HANDLING (async DNS, connect, drain, backoff) ALWAYS runs -
   * regardless of the mode. If it only ran in APRS mode, after a long button
   * press (demod off) the socket would stay open, the server's traffic would
   * pile up in the receive buffer, and the connection would stall. The DEMOD
   * (aprs_rx_feed) is still gated by the mode (see above). */
  SECT(S_APRS, aprs_rx_tick(now));
  /* New packet -> bring the APRS page forward. ui_flash does not switch by
   * itself unless the APRS mode is running. */
  {
    static uint32_t seen_ms = 0;
    uint32_t lms = aprs_rx_last_ms();
    if (lms && lms != seen_ms) { seen_ms = lms; ui_flash(UI_PAGE_APRS, OLED_APRS_HOLD_MS); }
  }
#endif
  /* CW tick + flash to the CW page on a new character */
  SECT(S_CW, cw_rx_tick(now));
  {
    static uint32_t seen_cw = 0;
    uint32_t lms = cw_rx_last_ms();
    if (lms && lms != seen_cw) {
      seen_cw = lms;
      ui_flash(UI_PAGE_CW, 8000);   /* keep the CW page for 8 s */
    }
  }
  /* On a page change draw IMMEDIATELY, do not wait for the next period -
   * otherwise nothing happens for a second and a half after the button
   * press, and it would seem broken. */
  if (ui_dirty()) { ui_clear_dirty(); oled_force(); }
  SECT(S_OLED, oled_tick(now));

#if FLASHLOG_ENABLE
  /* FLUSH WINDOW (2026-08-07). Writing to flash disables the flash cache,
   * which stops code running from flash on BOTH cores - the dedicated SPI
   * receive task too. Then only the queued DMA transactions are consumed,
   * for 123 ms with NQUEUE=24.
   *
   * 'drained == 0' means the receive ring was EMPTY in this iteration:
   * every block has been processed, so all NQUEUE transactions are queued.
   * This is the moment of maximum reserve - the measured 88 ms (max 167 ms)
   * flush fits in here.
   *
   * If the ring never empties (overloaded main loop), the deadlines in
   * flashlog.cpp still force the write: no log is lost. The length of the
   * deferral is shown by 'logstat' and the FLUSH lines. */
  flashlog_flush_window(drained == 0);
  flashlog_tick(now);            /* rare batched flash flush + serial command */
#endif

  /* Coalesced tuning: of the requests accumulated while the slider is
   * dragged only the LAST goes through, and at most once per TUNE_RATE_MS. */
#if SPY_ENABLE
  scan_flush(now);
#endif
  if (rtl_pending && now - rtl_last_tune_ms >= TUNE_RATE_MS) {
    rtl_pending = false;
    rtl_last_tune_ms = now;
    char cmd[16];
    snprintf(cmd, sizeof cmd, "F%lu", (unsigned long)rtl_pending_khz);
    DIAG.printf("\nrtl_tcp: tuning %lu kHz\n",
                (unsigned long)rtl_pending_khz);
    cmdlink_send(cmd);
    oled_khz = rtl_pending_khz;      /* show it on the display too */
#if SPY_ENABLE
    spy_set_freq(rtl_pending_khz * 1000u);
#endif
  }

  if (t0 == 0) t0 = now;
  if (now - t0 >= 2000) {
    float dt = (now - t0) / 1000.0f;
    uint32_t mean = st_counted ? (uint32_t)(st_abssum / st_counted) : 0;

    DIAG.printf("%6.1f blk/s %8.0f sps peak=%+5.0f mean=%+5.0f " LVL_UNIT
                " RF=%+d dBm "
                "LOST=%lu magic=%lu short=%lu SPEC=%lu | "
                "USB %5.1f kB/s(-%lu)  RAW %5.1f kB/s(-%lu)  "
                "RTL %5.1f kB/s(-%lu) sh=%d",
                st_blocks / dt, st_samples / dt, lvl_db(st_peak),
                lvl_db((int32_t)mean), st_rf_rssi, (unsigned long)st_lost,
                (unsigned long)st_badmagic, (unsigned long)st_badlen,
                (unsigned long)st_specline,
                (st_usbbytes / 1024.0f) / dt, (unsigned long)st_usbdrop,
                (st_rawbytes / 1024.0f) / dt, (unsigned long)ring_raw.dropped,
                (st_rtlbytes / 1024.0f) / dt, (unsigned long)ring_rtl.dropped,
                rtl_shift);
    /* ---- USB DIAGNOSTICS (2026-08-06) ----
     * If the PC-side tool gets no data, one UNTIL NOW had to guess why the
     * firmware drops the blocks. These two numbers tell:
     *
     *   conn=0             -> the CDC sees NO connected host. availableForWrite()
     *                         is then 0 by definition, so every write is
     *                         dropped. The fault is in the host <-> CDC
     *                         line-state (DTR/RTS) handling.
     *   conn=1, afw < 1040 -> there IS a host, but the CDC TX FIFO is smaller
     *                         than a block. Remedy: in custom_sdkconfig
     *                         CONFIG_TINYUSB_CDC_TX_BUFSIZE=4096
     *   conn=1, afw >= 1040-> the USB path is healthy; if the PC still gets
     *                         no data, the fault is on the PC side (driver / pyserial).
     *
     * The two numbers must be read TOGETHER with the "USB x kB/s(-N)" field above. */
    DIAG.printf("  USB: conn=%d afw=%d (%u needed per block)",
                (int)(bool)RAW, (int)RAW.availableForWrite(),
                (unsigned)BLK_BYTES);

    /* The loop frequency is the best early warning: if it drops, something
     * is blocking the main loop, and the next thing to break is block loss. */
    DIAG.printf("  loop %lu/s", (unsigned long)(st_loops / (uint32_t)dt));
    st_loops = 0;
    if (st_back) DIAG.printf("  BACK=%lu", (unsigned long)st_back);
    {
      uint32_t tot = st_pool_hit + st_pool_other;
      if (tot) {
        DIAG.printf("  POOL=%lu/%lu (%lu%%, NQUEUE=%d)",
                    (unsigned long)st_pool_hit, (unsigned long)tot,
                    (unsigned long)(100ul * st_pool_hit / tot), NQUEUE);
      }
      st_pool_hit = st_pool_other = 0;
    }
#if REORDER_BLOCKS > 0
    if (st_ro_dup || st_ro_hole || st_ro_late) {
      DIAG.printf("  REORDER: dup=%lu(E%lu/K%lu) hole=%lu late=%lu",
                  (unsigned long)st_ro_dup,
                  (unsigned long)st_ro_dup_same,
                  (unsigned long)st_ro_dup_diff,
                  (unsigned long)st_ro_hole,
                  (unsigned long)st_ro_late);
    }
    /* CUMULATIVE MIRROR for wifi_bench. The status counters are zeroed every
     * 2 s, so the bench's 10 s step delta would be meaningless (a negative
     * blk delta proved it, 2026-08-17). Accumulated here, BEFORE zeroing -
     * this is monotonic, so the delta is correct. */
    bench_tot_ro_dup  += st_ro_dup;
    bench_tot_ro_hole += st_ro_hole;
    st_ro_dup = st_ro_hole = st_ro_late = 0;
    st_ro_dup_same = st_ro_dup_diff = 0;
#endif
    DIAG.println();

    /* --- The timing report. THIS is what to watch, not LOST: LOST is only
     * the CONSEQUENCE. qmax = how close the backlog came to NQUEUE (reaching
     * it means loss); dt_max = the longest blind period; then the list of
     * culprits in descending order. --- */
    DIAG.printf("  TIMING: qmax=%lu/%d  RXRING=%lu/%d ovf=%lu  dt_max=%lu us |",
                (unsigned long)st_qmax, NQUEUE,
                (unsigned long)rx_ring_max, RXRING_BLOCKS,
                (unsigned long)rx_ring_ovf, (unsigned long)st_dtmax);
    rx_ring_max = 0;
    for (int i = 0; i < S_COUNT; i++) {
      if (sect_us[i] >= 1000u)           /* only those above 1 ms */
        DIAG.printf(" %s=%lu", SECT_NAME[i], (unsigned long)sect_us[i]);
    }
    /* RAM watch: free heap / minimum so far / largest contiguous block.
     * "min" is the watermark since boot - if it creeps down, something leaks
     * or we run out at peak load. "max block" is fragmentation: if free is
     * large but max block is small, a big malloc no longer succeeds. */
    DIAG.printf(" | RAM: %lu k free, min %lu k, max block %lu k",
                (unsigned long)(esp_get_free_heap_size() / 1024u),
                (unsigned long)(esp_get_minimum_free_heap_size() / 1024u),
                (unsigned long)(heap_caps_get_largest_free_block(
                                    MALLOC_CAP_8BIT) / 1024u));
#if FLASHLOG_ENABLE
    /* Health snapshot to flash, every ~10 s (5 x 2 s). The H lines reveal
     * every intermittent pattern afterwards: decode ratio, RAM leak, WiFi
     * RSSI, block loss, loop slowdown. */
    static uint8_t fl_div = 0;
    if (++fl_div >= 5) {
      fl_div = 0;
#if APRS_RX_ENABLE
      flashlog_printf("H blk=%.0f pk=%+.0f avg=%+.0f rf=%d LOST=%lu rx=%lu bad=%lu "
                      "gate=%lu IS=%d ram=%luk min=%luk wifi=%d dtmax=%lu qmax=%lu",
                      st_blocks / dt, lvl_db(st_peak), lvl_db((int32_t)mean),
                      st_rf_rssi, (unsigned long)st_lost,
                      (unsigned long)aprs_rx_frames(), (unsigned long)aprs_rx_bad(),
                      (unsigned long)aprs_rx_gated(), aprs_rx_is_online() ? 1 : 0,
                      (unsigned long)(esp_get_free_heap_size() / 1024u),
                      (unsigned long)(esp_get_minimum_free_heap_size() / 1024u),
                      (int)WiFi.RSSI(), (unsigned long)st_dtmax,
                      (unsigned long)st_qmax);
#else
      flashlog_printf("H blk=%.0f pk=%+.0f avg=%+.0f rf=%d LOST=%lu ram=%luk min=%luk "
                      "wifi=%d dtmax=%lu qmax=%lu",
                      st_blocks / dt, lvl_db(st_peak), lvl_db((int32_t)mean),
                      st_rf_rssi, (unsigned long)st_lost,
                      (unsigned long)(esp_get_free_heap_size() / 1024u),
                      (unsigned long)(esp_get_minimum_free_heap_size() / 1024u),
                      (int)WiFi.RSSI(), (unsigned long)st_dtmax,
                      (unsigned long)st_qmax);
#endif
    }

#endif
    /* --- ILI9341 status display. Same 2 s cadence as the serial status. The
     * output goes over SPI2; the FG23 slave is on SPI3, so the two buses do
     * not compete. IMPORTANT: this must stand AFTER the FLASHLOG_ENABLE
     * #endif, otherwise (with FLASHLOG_ENABLE 0) it would not be compiled at all. --- */
    {
      static String tft_ip;
      tft_ip = wifi_up ? WiFi.localIP().toString() : String("-");
      tft_stat_t ts;
      ts.blokk_s      = st_blocks / dt;
      ts.sps          = (uint32_t)(st_samples / dt);
      if (ts.sps > 100) st_sps_out = ts.sps;   /* for the panel's scale */
      ts.csucs_dbfs   = lvl_db(st_peak);
      ts.rssi_dbm     = st_rf_rssi;
      ts.ram_k        = (uint32_t)(esp_get_free_heap_size() / 1024u);
      ts.ram_min_k    = (uint32_t)(esp_get_minimum_free_heap_size() / 1024u);
      ts.terheles_pct = (uint8_t)(rgb_load_get() * 100.0f);
      ts.lost         = st_ro_hole;
      ts.freq_khz     = 0;
      ts.ip           = tft_ip.c_str();
      ts.ssid         = "";
      tft_show(&ts);
    }
    st_qmax = st_dtmax = st_back = 0;
    memset(sect_us, 0, sizeof sect_us);
    if (st_blocks == 0) DIAG.print("  <-- no data / stream not running");
    DIAG.println();
#if APRS_RX_ENABLE
    static uint32_t prev_fr = 0, prev_bad = 0;
    if (aprs_rx_frames() != prev_fr || aprs_rx_bad() != prev_bad) {
      prev_fr = aprs_rx_frames(); prev_bad = aprs_rx_bad();
      DIAG.printf("APRS: %lu frames, %lu CRC errors, %lu gated, IS=%s\n",
                  (unsigned long)aprs_rx_frames(), (unsigned long)aprs_rx_bad(),
                  (unsigned long)aprs_rx_gated(),
                  aprs_rx_is_online() ? "up" : "down");
    }
#endif

    bench_tot_blocks += st_blocks;
    bench_tot_lost   += st_lost;
    st_blocks = st_samples = st_lost = st_badmagic = st_badlen = 0;
    st_peak = 0; st_abssum = 0; st_counted = 0;
    st_usbbytes = st_rawbytes = st_rtlbytes = 0;
    st_usbdrop = 0;
    st_gap_shown = 0;
    ring_raw.dropped = ring_rtl.dropped = 0;
    t0 = now;
  }
}
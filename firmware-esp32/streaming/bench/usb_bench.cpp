/* SPDX-License-Identifier: MIT
 *
 * usb_bench.ino — WHAT IS THE NATIVE USB CEILING of the ESP32-S3?
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================== WHAT IT MEASURES ==================
 *
 * No radio, no SPI. Valid 1040-byte "IQB2" blocks are generated and pushed
 * out over the NATIVE USB CDC as fast as possible. On the PC side
 * usb_sink.py reads and counts them.
 *
 * WHY THE PC SIDE IS REQUIRED: on USB CDC the HOST initiates. If nobody
 * reads, the TX buffer fills up and 0 kB/s is measured — that measures the
 * absence of a reader, not the link. The figure is valid ONLY with
 * usb_sink.py running.
 *
 * ================== FOUR PHASES ==================
 *
 * 10 seconds per phase, then it advances automatically. Summary at the end.
 *
 *   1  1040 B / write, NON-BLOCKING   <- exactly the current firmware behaviour
 *   2  1040 B / write, BLOCKING       <- how much waiting gains
 *   3  4160 B / write, blocking       <- 4 blocks at once, fewer calls
 *   4  8320 B / write, blocking       <- 8 blocks at once
 *
 * Phases 3 and 4 are the point: CDC needs large writes to send full USB
 * frames. If phase 4 yields significantly more than phase 1, the real
 * firmware must also write several blocks at once.
 *
 * ================== READING THE RESULT ==================
 *
 * Maximum sample rate from the reported kB/s:  sps = kB/s * 1024 / 4
 *
 *     500 kB/s  -> 125 ksps   (required for the 1 Msps PHY + R=8 plan)
 *     900 kB/s  -> 230 ksps
 *    1000 kB/s  -> 256 ksps   (practical top of full-speed USB)
 *
 * ================== TRY BOTH ==================
 *
 * The S3 has TWO USB peripherals, BOTH on GPIO19/20:
 *
 *   ARDUINO_USB_MODE=1  -> HWCDC, the built-in USB-Serial/JTAG controller.
 *                          The current one. Simple, but reported to be
 *                          slower.
 *   ARDUINO_USB_MODE=0  -> TinyUSB (USBCDC) on the OTG peripheral. Usually
 *                          yields more, at the cost of more RAM.
 *
 * MEASURE BOTH. One line in platformio.ini, and the difference may be a
 * factor of two. The current firmware uses HWCDC, so if TinyUSB wins, that
 * is a free bandwidth doubling.
 *
 * platformio.ini:
 *     build_flags =
 *       -D ARDUINO_USB_MODE=1        ; <- toggle this 1 <-> 0
 *       -D ARDUINO_USB_CDC_ON_BOOT=1
 *
 * ================== USAGE ==================
 *
 *   1. Flash this (through the CP2102 port, as usual).
 *   2. Open the CP2102 port at 115200: the diagnostics appear there.
 *   3. Start:  python usb_sink.py COM4     (the NATIVE USB port!)
 *   4. Wait 45 seconds and read the figures on both sides.
 *
 * The figures on both sides must match. If the ESP reports more than the
 * PC sees, the host-side read is the bottleneck (Windows usbser.sys tends
 * to do this) — that is valuable information too.
 */

#include <Arduino.h>
#include <string.h>

/* ---------------- block format (matches iq_stream.c) ------------------- */
#define BLK_SAMPLES   256
#define HDR_BYTES     16
#define BLK_BYTES     (HDR_BYTES + BLK_SAMPLES * 4)      /* 1040 */
#define IQ_BLK_MAGIC  0x32425149u                        /* "IQB2" */

/* ---------------- measurement parameters ---------------- */
#define PHASE_MS      10000u        /* duration of each phase */
/* Must be LARGER than the largest chunk (8 * 1040 = 8320), otherwise
 * phase 4 never fits into a single transfer and corrupts exactly the
 * variable the measurement is about. */
#define TX_BUF_BYTES  12288         /* worth tuning: 8192..24576 */
#define MAX_CHUNK_BLK 8             /* blocks written by the largest phase */

#define DIAG  Serial0               /* CP2102: text output */
#define RAW   Serial                /* native USB: raw stream */

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint16_t nsamp;
  uint16_t decim;
  uint32_t fs_in_hz;
} blk_hdr_t;

/* A pre-filled buffer of MAX_CHUNK_BLK blocks. The headers are refreshed
 * before sending, the payload is fixed — the content does not matter here,
 * only the byte count and that the PC side can recognise the frame. */
static uint8_t txbuf[MAX_CHUNK_BLK * BLK_BYTES];
static uint32_t seq = 0;

typedef struct {
  const char *name;
  uint16_t    nblk;        /* blocks per write */
  bool        blocking;    /* wait when there is no room */
} phase_t;

static const phase_t PHASES[] = {
  { "1040 B  non-blocking (current fw)", 1, false },
  { "1040 B  blocking",                  1, true  },
  { "4160 B  blocking (4 blocks)",       4, true  },
  { "8320 B  blocking (8 blocks)",       8, true  },
};
#define NPHASE (sizeof(PHASES) / sizeof(PHASES[0]))

static uint32_t res_kbs[NPHASE];
static uint32_t res_drop[NPHASE];
static uint32_t res_maxus[NPHASE];

static void fill_payload(void)
{
  /* Recognisable but non-constant pattern: a slip would be visible on the
   * PC side. The content does not affect the speed. */
  for (int b = 0; b < MAX_CHUNK_BLK; b++) {
    int16_t *iq = (int16_t *)(txbuf + b * BLK_BYTES + HDR_BYTES);
    for (int k = 0; k < BLK_SAMPLES; k++) {
      iq[2 * k]     = (int16_t)(k * 128);        /* I: sawtooth */
      iq[2 * k + 1] = (int16_t)(-k * 128);       /* Q: mirror image */
    }
  }
}

static void stamp_headers(uint16_t nblk)
{
  for (uint16_t b = 0; b < nblk; b++) {
    blk_hdr_t *h = (blk_hdr_t *)(txbuf + b * BLK_BYTES);
    h->magic    = IQ_BLK_MAGIC;
    h->seq      = seq++;
    h->nsamp    = BLK_SAMPLES;
    h->decim    = 8;
    h->fs_in_hz = 400000u;
  }
}

void setup()
{
  DIAG.begin(115200);

  /* The buffer must be enlarged BEFORE begin(). The default is a few
   * hundred bytes, not even enough for one 1040-byte block — this is what
   * broke the whole chain at first (USB 0.0 kB/s, every block dropped). */
  RAW.setTxBufferSize(TX_BUF_BYTES);
  RAW.setTxTimeoutMs(0);
  RAW.begin();

  delay(800);
  fill_payload();

  DIAG.println("\n=== ESP32-S3 native USB throughput ceiling ===");
#if ARDUINO_USB_MODE
  DIAG.println("peripheral: HWCDC (USB-Serial/JTAG)   [ARDUINO_USB_MODE=1]");
#else
  DIAG.println("peripheral: TinyUSB CDC (OTG)         [ARDUINO_USB_MODE=0]");
#endif
  DIAG.printf("TX buffer: %d B   block: %d B   phase: %lu ms\n",
              TX_BUF_BYTES, BLK_BYTES, (unsigned long)PHASE_MS);
  DIAG.println("START THE PC SIDE:  python usb_sink.py COMxx");
  DIAG.println("(without a reader the measurement is meaningless — expect 0 kB/s)");
  DIAG.println();
}

static void run_phase(int idx)
{
  const phase_t *p = &PHASES[idx];
  const size_t chunk = (size_t)p->nblk * BLK_BYTES;

  DIAG.printf("[%d/%d] %-34s ", idx + 1, (int)NPHASE, p->name);
  DIAG.flush();

  /* In a blocking phase waiting is allowed, but not indefinitely: without
   * a host we give up after 50 ms and count it as a drop. So the test does
   * not stall even if usb_sink.py was not started. */
  RAW.setTxTimeoutMs(p->blocking ? 50 : 0);

  uint64_t written = 0;
  uint32_t drops = 0, max_us = 0;
  uint32_t t0 = millis();

  uint32_t polls = 0;

  while (millis() - t0 < PHASE_MS) {

    if (!p->blocking && RAW.availableForWrite() < (int)chunk) {
      /* No drop is counted here: this is a failed POLL, not a dropped
       * block. The old version counted tens of millions here, and the
       * "dropped" column of the first phase was meaningless.
       * The delay(1) is needed for the task watchdog: 10 seconds of tight
       * spinning starves the IDLE task. delay(0) is NOT enough — that is
       * only taskYIELD, which switches to a task of equal or higher
       * priority, while IDLE is lower. A one-tick wait is required. It does
       * not reduce throughput: this path is taken only when the TX buffer
       * is full. */
      polls++;
      delay(1);
      continue;
    }

    /* Stamp the header ONLY when actually sending. Otherwise the sequence
     * number would advance for unsent blocks too, and usb_sink.py would
     * report phantom loss. */
    stamp_headers(p->nblk);

    uint32_t u0 = micros();
    size_t n = RAW.write(txbuf, chunk);
    uint32_t du = micros() - u0;
    if (du > max_us) max_us = du;

    written += n;
    if (n < chunk) drops++;
  }

  RAW.setTxTimeoutMs(0);

  float dt = (millis() - t0) / 1000.0f;
  uint32_t kbs = (uint32_t)((written / 1024.0f) / dt + 0.5f);
  res_kbs[idx]   = kbs;
  res_drop[idx]  = drops;
  res_maxus[idx] = max_us;

  DIAG.printf("%5lu kB/s   short writes=%-4lu  empty polls=%-9lu  "
              "longest write=%lu us\n",
              (unsigned long)kbs, (unsigned long)drops,
              (unsigned long)polls, (unsigned long)max_us);
}

void loop()
{
  static bool done = false;
  if (done) { delay(1000); return; }

  DIAG.println("--- measurement starting, ~40 seconds ---");
  for (int i = 0; i < (int)NPHASE; i++) run_phase(i);

  uint32_t best = 0;
  int best_i = 0;
  for (int i = 0; i < (int)NPHASE; i++) {
    if (res_kbs[i] > best) { best = res_kbs[i]; best_i = i; }
  }

  /* 4 bytes / complex sample */
  uint32_t sps = (uint32_t)((uint64_t)best * 1024u / 4u);

  DIAG.println("\n=========== SUMMARY ===========");
  DIAG.printf("best: %s -> %lu kB/s\n",
              PHASES[best_i].name, (unsigned long)best);
  DIAG.printf("that is %lu sps (%lu.%03lu ksps) with 16-bit I/Q\n",
              (unsigned long)sps,
              (unsigned long)(sps / 1000), (unsigned long)(sps % 1000));
  DIAG.printf("with 8-bit I/Q twice that: %lu sps\n",
              (unsigned long)(sps * 2u));
  DIAG.println();

  if (sps >= 125000u) {
    DIAG.println("VERDICT: the 1 Msps PHY + R=8 (125 ksps, 500 kB/s) FITS.");
    DIAG.println("  next step: SPI_HZ = 12000000, PHY to 1 Msps,");
    DIAG.printf("  and write %d blocks at once on the ESP.\n",
                (int)PHASES[best_i].nblk);
  } else if (sps >= 60000u) {
    DIAG.println("VERDICT: 125 ksps does NOT fit at 16 bits. Either switch to");
    DIAG.println("  8 bits (rtl_tcp path), or stay at 62.5 ksps with R=16,");
    DIAG.println("  or move the high-rate stream to WiFi.");
  } else {
    DIAG.println("VERDICT: suspiciously low. Is usb_sink.py running? If so,");
    DIAG.println("  also try ARDUINO_USB_MODE=0 (TinyUSB).");
  }

  if (res_kbs[NPHASE - 1] > res_kbs[0] * 3 / 2) {
    DIAG.println("\nIMPORTANT: the large chunk yields much more than a single block.");
    DIAG.println("  In the production firmware also gather several blocks and");
    DIAG.println("  send them in a single write.");
  }
#if ARDUINO_USB_MODE
  DIAG.println("\nNow also try ARDUINO_USB_MODE=0 (TinyUSB) —");
  DIAG.println("it often yields significantly more on the same two pins.");
#endif

  done = true;
}

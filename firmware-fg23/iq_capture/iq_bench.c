/* SPDX-License-Identifier: MIT
 *
 * iq_bench.c — FG23 continuous I/Q rate benchmark
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * ================= MEASUREMENT PRINCIPLE =================
 *
 * During the benchmark NOTHING is sent out on the UART. If we printed, we
 * would be measuring the UART, not the chip. Samples go into a small
 * scratch buffer and die there — we only count.
 *
 * What is measured:
 *   fs      — empirically: sample count / RAIL time. NOT from the datasheet.
 *   OVR     — RAIL_EVENT_RX_FIFO_OVERFLOW counter. This is THE ceiling.
 *   peak    — FIFO fill level on ISR entry. This is the HEADROOM.
 *             A bare "was there an overrun" says nothing about whether
 *             you are running at 30% or at 95%.
 *   CPU%    — how much the capture ISR consumes; the rest is for the DSP.
 *
 * CPU% is measured as in the SetFreqOffset benchmark: iterations of a
 * known load loop are counted without the radio (reference), then during
 * capture. The difference = CPU taken.
 *
 * ================= HOW THIS DIFFERS FROM THE app.c DRAIN =================
 *
 * The app.c "discard" branch reads EXACTLY once THRESHOLD_BYTES per event.
 * If the sample rate is such that more than that accumulates between two
 * ISR entries, the FIFO fills monotonically and overflow is guaranteed —
 * regardless of whether the CPU could keep up. The drain here reads UNTIL
 * EMPTY, so it measures the real silicon/CPU ceiling, not that artificial
 * limit.
 *
 * ================= FLOAT PRINTF =================
 *
 * app.c uses %f nowhere — the default "tiny printf" of Silabs projects
 * cannot print floating point. Therefore EVERY printout here uses
 * integers with a manual decimal point. Do not add %f.
 */

#include "iq_bench.h"
#include <stdio.h>
#include <string.h>

/* ---------------- configuration ---------------- */

/* Threshold sweep steps. The threshold is the ALMOST_FULL trigger level:
 * small -> frequent ISR (high CPU), large -> less headroom before overrun.
 * The current app.c value is 256 (64 samples) — that is the middle point. */
/* Start with the SAFEST point (rare ISR) and progress towards the dense
 * end. This way, if a low threshold causes saturation, the data of the
 * higher points is already available. */
static const uint16_t bench_thresholds[] = { 2048, 1024, 512, 256, 128 };
#define BENCH_NUM_THR (sizeof(bench_thresholds)/sizeof(bench_thresholds[0]))

/* Scratch: samples are read here and die here. Need not be large — we
 * read repeatedly in rounds until the FIFO is empty. */
#define BENCH_SCRATCH 2048u

/* How far to drain the FIFO? NOT to zero: at 400 ksps 4 bytes arrive every
 * 10 us, while one loop iteration is ~2 us — the `avail >= 4` condition
 * would never become false (this was the first lock-up). 128 bytes = 320 us
 * of refill, comfortably above the loop time, so it is guaranteed to exit. */
#define BENCH_MIN_DRAIN 128u

/* ---------------- state ---------------- */

static volatile bool     b_running   = false;
static volatile uint32_t b_bytes     = 0;
static volatile uint32_t b_events    = 0;
static volatile uint32_t b_overflows = 0;
static volatile uint16_t b_peak      = 0;
static volatile uint16_t b_threshold = 256;   /* the current measurement point */

/* If true, the drain calls RAIL_ReadRxFifo with a NULL destination: in
 * theory "discard without copying", i.e. only the read pointer advances.
 * The BUFC writes into RAM (datasheet 3.2.6: zero-copy), so the samples
 * are already in place — copying is wasted work. */
static volatile bool b_zerocopy = false;

/* The measurement deadline is guarded BY THE ISR ITSELF. If the sample
 * rate is so high that the capture ISR takes 100% of the CPU, the main
 * loop NEVER progresses — and if the stop condition lived there, the
 * system would lock up forever. So the ISR checks on every entry whether
 * time is up, and it stops the radio. From there the main loop revives
 * on its own. */
static volatile RAIL_Time_t b_deadline = 0;
static volatile bool        b_starved  = false;

static uint32_t b_scratch_words[BENCH_SCRATCH / 4];
#define b_scratch ((uint8_t *)b_scratch_words)

/* Load loop reference, iterations/s without the radio. */
static uint32_t b_base_iters_per_s = 0;

/* Against optimisation. */
static volatile uint32_t b_sink = 0;

/* ---------------- load loop ---------------- */

/* Deliberately simple integer MAC chain. It does not imitate the real
 * blocker FFT; it is a STABLE, repeatable yardstick with which the CPU
 * stolen by the capture ISR can be measured. Integer-only, so that the
 * presence or absence of an FPU does not distort the measurement. */
static uint32_t bench_load_spin(uint32_t microseconds)
{
  RAIL_Time_t t0 = RAIL_GetTime();
  uint32_t iters = 0;
  uint32_t a = 1103515245u, c = 12345u, x = 1u;

  while ((RAIL_GetTime() - t0) < microseconds) {
    /* 16 operations / iteration, so that RAIL_GetTime() does not dominate */
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    iters++;
  }
  b_sink = x;
  return iters;
}

/* ---------------- RAIL event ---------------- */

bool iq_bench_active(void) { return b_running; }

bool iq_bench_on_event(RAIL_Handle_t rail, RAIL_Events_t events)
{
  if (!b_running) return false;

  /* Has the measurement window expired? If so, RADIO OFF — otherwise
   * with a saturated ISR the main loop would never notice the deadline. */
  if ((int32_t)(RAIL_GetTime() - b_deadline) >= 0) {
    /* NOT RAIL_Idle: calling it from callback context is unsafe, and at
     * the thr=128 point this is exactly where it locked up. Instead the
     * FIFO events are MASKED — no more entries, the main loop revives and
     * performs the proper shutdown in main context. */
    RAIL_ConfigEvents(rail,
                      RAIL_EVENT_RX_FIFO_ALMOST_FULL
                      | RAIL_EVENT_RX_FIFO_OVERFLOW,
                      RAIL_EVENTS_NONE);
    b_running = false;
    b_starved = true;      /* closed by the ISR, not by the main loop */
    return true;
  }

  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    ++b_overflows;
    /* After an overflow the FIFO contents are meaningless — flush and
     * continue. We do not stop: we want to know HOW MANY overruns there
     * are, not just whether there was one. */
    RAIL_ResetFifo(rail, false, true);
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    /* Fill level FIRST — this is the headroom metric. If we read first,
     * we would no longer see how deep the FIFO was. */
    uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail);
    if (avail > b_peak) b_peak = avail;

    ++b_events;

    /* Drain BELOW the threshold, not to zero — and with a hard iteration
     * limit. A `while (avail >= 4)` loop would be INFINITE here: at
     * 160 ksps 4 bytes arrive every 6 us, so by the time we read and
     * query again, there is data again. The ISR would never exit. */
    uint8_t guard = 32u;              /* 32 * 256 B = 8 KiB > FIFO */
    while (avail >= BENCH_MIN_DRAIN && guard--) {
      uint16_t chunk = (avail > BENCH_SCRATCH) ? BENCH_SCRATCH : avail;
      chunk = (uint16_t)(chunk & ~3u);
      if (chunk == 0u) break;
      uint16_t got = RAIL_ReadRxFifo(rail,
                                    b_zerocopy ? NULL : b_scratch,
                                    chunk);
      if (got == 0u) break;
      b_bytes += got;
      avail = RAIL_GetRxFifoBytesAvailable(rail);
    }
  }

  return true;   /* the app.c capture branch must be skipped */
}

/* ---------------- one measurement ---------------- */

typedef struct {
  uint16_t threshold;
  uint32_t duration_us;
  uint32_t bytes;
  uint32_t events;
  uint32_t overflows;
  uint16_t peak;
  uint32_t fs_hz;
  uint32_t fill_permille;   /* peak / 4096, in per mille */
  uint32_t cpu_permille;    /* CPU taken, in per mille  */
  uint32_t ev_per_s;
} bench_res_t;

static void bench_one(RAIL_Handle_t rail,
                      uint16_t channel,
                      uint16_t threshold,
                      uint32_t seconds,
                      bench_res_t *r)
{
  memset(r, 0, sizeof(*r));
  r->threshold = threshold;

  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  RAIL_SetRxFifoThreshold(rail, threshold);
  b_threshold = threshold;

  b_bytes = 0; b_events = 0; b_overflows = 0; b_peak = 0;

  RAIL_ResetFifo(rail, false, true);

  RAIL_Time_t t0 = RAIL_GetTime();
  b_deadline = t0 + seconds * 1000000u;
  b_starved  = false;
  b_running  = true;
  RAIL_StartRx(rail, channel, NULL);

  uint32_t iters = bench_load_spin(seconds * 1000000u);

  RAIL_Time_t t1 = RAIL_GetTime();

  /* ORDER MATTERS! Stop the radio first, and ONLY then hand control back
   * to app.c. The other way round there is a race: as soon as b_running
   * is false, events go to the app.c drain branch, which works with an
   * `avail >= 256` condition — if the measurement threshold is lower
   * (128), it never enters, never drains, and the event fires again
   * endlessly. RAIL_Idle waits in main context, so it never completes.
   * This was the thr=128 freeze. */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  b_running = false;
  /* If the ISR masked itself (saturation), re-arm here. */
  RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  r->duration_us = (uint32_t)(t1 - t0);
  r->bytes       = b_bytes;
  r->events      = b_events;
  r->overflows   = b_overflows;
  r->peak        = b_peak;

  /* fs: 4 bytes = 1 complex sample.
   * RAIL time is derived from the HFXO (39 MHz) — the same clock domain
   * as the sampling clock. What is measured here is therefore the
   * DECIMATION RATIO (fs = 39 MHz / N), not a comparison of two
   * independent clocks. Hence it is deterministic and repeatable. */
  if (r->duration_us > 0u) {
    /* (bytes/4) * 1e6 / us  —  in 64 bits, so it does not overflow */
    uint64_t num = (uint64_t)(r->bytes / 4u) * 1000000ull;
    r->fs_hz    = (uint32_t)(num / r->duration_us);
    r->ev_per_s = (uint32_t)(((uint64_t)r->events * 1000000ull)
                             / r->duration_us);
  }

  r->fill_permille = ((uint32_t)r->peak * 1000u) / 4096u;

  if (b_base_iters_per_s > 0u && r->duration_us > 0u) {
    uint64_t ach = ((uint64_t)iters * 1000000ull) / r->duration_us;
    uint32_t ratio = (uint32_t)((ach * 1000ull) / b_base_iters_per_s);
    if (ratio > 1000u) ratio = 1000u;
    r->cpu_permille = 1000u - ratio;
  }
}

/* ---------------- sweep ---------------- */

void iq_bench_sweep(RAIL_Handle_t rail,
                    uint16_t channel,
                    uint32_t seconds,
                    uint16_t restore_thresh)
{
  bench_res_t r;

  /* CPU reference: radio off */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  printf("# CPU reference (radio OFF, 2 s)...\r\n");
  (void)bench_load_spin(200000u);              /* warm-up */
  b_base_iters_per_s = bench_load_spin(2000000u) / 2u;
  printf("# reference: %lu iter/s\r\n",
         (unsigned long)b_base_iters_per_s);

  printf("#\r\n");
  printf("# ===== FG23 I/Q sustained-rate benchmark =====\r\n");
  printf("# %lu s/point, channel %u, FIFO 4096 B\r\n",
         (unsigned long)seconds, (unsigned)channel);
  printf("#\r\n");
  printf("# each point is SILENT for %lu s (the UART would disturb the measurement)\r\n",
         (unsigned long)seconds);
  printf("# ---------------------------------------------------\r\n");

  for (unsigned k = 0; k < BENCH_NUM_THR; k++) {
    /* Progress indication: during the measurement NOTHING may go out on
     * the UART (otherwise we would measure the UART), so we announce the
     * point BEFORE it, so it does not look like a freeze. The '.' goes
     * without a line end, so the measured data lands on the same line. */
    printf("  [%u/%u] thr=%u, %lu s ... ",
           k + 1u, (unsigned)BENCH_NUM_THR,
           (unsigned)bench_thresholds[k], (unsigned long)seconds);

    bench_one(rail, channel, bench_thresholds[k], seconds, &r);

    printf("fs=%lu.%02lu ksps  ev/s=%lu  peak=%u  fill=%lu.%01lu%%  "
           "OVR=%lu  CPU=%lu.%01lu%%%s\r\n",
           (unsigned long)(r.fs_hz / 1000u),
           (unsigned long)((r.fs_hz % 1000u) / 10u),
           (unsigned long)r.ev_per_s,
           (unsigned)r.peak,
           (unsigned long)(r.fill_permille / 10u),
           (unsigned long)(r.fill_permille % 10u),
           (unsigned long)r.overflows,
           (unsigned long)(r.cpu_permille / 10u),
           (unsigned long)(r.cpu_permille % 10u),
           (r.overflows > 0u) ? "  <-- OVERRUN"
                              : (b_starved ? "  <-- CPU SATURATED" : ""));
  }

  printf("# ---------------------------------------------------\r\n");
  printf("#  OVR>0      -> this point is NOT sustainable\r\n");
  printf("#  fill>70%%   -> marginal, sensitive to jitter\r\n");
  printf("#  fs         -> enter THIS into the SDR++ sample rate field\r\n");
  printf("#  CPU%%       -> consumed by the capture; the rest is for the DSP\r\n");

  /* Back to the app.c state, so that the 'c' capture works as before. */
  RAIL_SetRxFifoThreshold(rail, restore_thresh);
  RAIL_ResetFifo(rail, false, true);
  RAIL_StartRx(rail, channel, NULL);
  printf("# threshold restored to %u, RX restarted\r\n",
         (unsigned)restore_thresh);
}


/* ================= NULL-read trial + comparison =================
 *
 * Step 1 — TRIAL (harmless): a single NULL read, with the FIFO level read
 *    before and after. If RAIL DID copy to NULL, that would be a hard
 *    fault — hence only ONE call, for 64 bytes, checked immediately
 *    afterwards. If the board gets past it and prints the line, the path
 *    is open.
 *
 * Step 2 — MEASUREMENT: the same threshold in two runs, with and without
 *    copying. The difference in CPU% is the answer itself.
 */

void iq_bench_zerocopy_test(RAIL_Handle_t rail,
                            uint16_t channel,
                            uint32_t seconds,
                            uint16_t restore_thresh)
{
  printf("\r\n# ===== zero-copy (NULL) test =====\r\n");

  /* ---------- 1. TRIAL ---------- */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  /* Events OFF: let the FIFO fill up by itself, nobody drains it. */
  RAIL_ConfigEvents(rail,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW,
                    RAIL_EVENTS_NONE);
  RAIL_ResetFifo(rail, false, true);
  RAIL_StartRx(rail, channel, NULL);

  /* 5 ms: at 400 ksps the 4096 bytes fill up easily (2.56 ms) */
  RAIL_Time_t t = RAIL_GetTime() + 5000u;
  while ((int32_t)(RAIL_GetTime() - t) < 0) { }

  /* STOP THE RADIO before the measurement! Otherwise it keeps filling:
   * at 928 ksps that is 3.7 MB/s, so even in the few microseconds between
   * the two reads 8-10 bytes arrive, and the pointer step appears smaller.
   * This was the cause of the first, false "NOT supported" result. */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);

  uint16_t avail0 = RAIL_GetRxFifoBytesAvailable(rail);
  printf("#  trial: before avail=%u (radio stopped)\r\n",
         (unsigned)avail0);

  uint16_t got = RAIL_ReadRxFifo(rail, NULL, 64u);   /* <-- THE QUESTION */

  uint16_t avail1 = RAIL_GetRxFifoBytesAvailable(rail);

  uint16_t moved = (avail0 > avail1) ? (uint16_t)(avail0 - avail1) : 0u;
  printf("#  trial: returned=%u  after avail=%u  -> the pointer advanced %u bytes\r\n",
         (unsigned)got, (unsigned)avail1, (unsigned)moved);

  /* With the radio stopped this must be exact. */
  bool ok = (got == 64u) && (moved == 64u);
  if (!ok) {
    printf("#  -> NOT supported (the pointer did not advance) — copying stays\r\n");
    RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                      RAIL_EVENT_RX_FIFO_ALMOST_FULL
                      | RAIL_EVENT_RX_FIFO_OVERFLOW);
    RAIL_SetRxFifoThreshold(rail, restore_thresh);
    RAIL_ResetFifo(rail, false, true);
    RAIL_StartRx(rail, channel, NULL);
    return;
  }
  printf("#  -> WORKS: the pointer advanced without copying\r\n#\r\n");

  RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  /* ---------- 2. MEASUREMENT ---------- */
  if (b_base_iters_per_s == 0u) {
    printf("# CPU reference (radio OFF, 2 s)...\r\n");
    (void)bench_load_spin(200000u);
    b_base_iters_per_s = bench_load_spin(2000000u) / 2u;
    printf("# reference: %lu iter/s\r\n", (unsigned long)b_base_iters_per_s);
  }

  bench_res_t r;
  const uint16_t thr = 1024u;          /* the chosen operating point */

  for (int pass = 0; pass < 2; pass++) {
    b_zerocopy = (pass == 1);
    printf("  %-12s thr=%u, %lu s ... ",
           b_zerocopy ? "NULL (zero)" : "with copy",
           (unsigned)thr, (unsigned long)seconds);

    bench_one(rail, channel, thr, seconds, &r);

    printf("fs=%lu.%02lu ksps  ev/s=%lu  OVR=%lu  CPU=%lu.%01lu%%\r\n",
           (unsigned long)(r.fs_hz / 1000u),
           (unsigned long)((r.fs_hz % 1000u) / 10u),
           (unsigned long)r.ev_per_s,
           (unsigned long)r.overflows,
           (unsigned long)(r.cpu_permille / 10u),
           (unsigned long)(r.cpu_permille % 10u));
  }

  b_zerocopy = false;
  RAIL_SetRxFifoThreshold(rail, restore_thresh);
  RAIL_ResetFifo(rail, false, true);
  RAIL_StartRx(rail, channel, NULL);
  printf("# done — the difference of the two CPU%% values is the cost of copying\r\n");
}

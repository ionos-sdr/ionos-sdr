/* wifi_bench.h - measuring the WiFi throughput ceiling in the FG23-SDR chain
 *
 * HA7DCD.  The question this module answers:
 *   "at what sps does the REORDER line keep hole= zero for ten minutes"
 * - i.e. NOT the raw iperf figure, but the rate at which the WiFi traffic
 * does not yet starve the SPI service.
 *
 * Usage in main.cpp:
 *
 *   #include "wifi_bench.h"
 *   setup():   wifi_bench_init();  wifi_bench_set_ext(bench_ext_cb);
 *   loop():    bench_console_tick();
 *
 * The module opens its OWN task and its OWN listen socket (7777 by
 * default); it does not touch the existing 8888 / 5555 paths.  It can run
 * concurrently with the real IQ stream - that is precisely the point.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- the chain's own counters, read by the bench at every step ----------
 * The module does NOT know the main.cpp variables; wire them in via a
 * callback. Leave unavailable ones at 0.  Every field is CUMULATIVE (the
 * bench computes deltas).
 */
typedef struct {
    uint32_t spi_lost;        /* real block loss (REORDER: hole=)     */
    uint32_t spi_dup;         /* duplicate (REORDER: dup=)            */
    uint32_t spi_ovf;         /* rxring overflow (ovf=)               */
    uint32_t spi_blocks;      /* total blocks received                */
    uint32_t spi_ring_max;    /* RXRING watermark (NOT cumulative, max) */
    uint32_t loop_dt_max_us;  /* main loop max period (NOT cumulative) */
    uint32_t spy_drop;        /* spyserver dropped messages           */
} wb_ext_t;

typedef void (*wb_ext_fn)(wb_ext_t *out);

/* In this project (main.cpp, 2026-08-17) the real names are:
 *
 *   static void bench_ext_cb(wb_ext_t *o) {
 *       o->spi_lost       = st_ro_hole;    // REORDER: hole=  <- THE REAL ONE
 *       o->spi_dup        = st_ro_dup;     // REORDER: dup=
 *       o->spi_ovf        = rx_ring_ovf;   // must be 0
 *       o->spi_blocks     = st_blocks;
 *       o->spi_ring_max   = rx_ring_max;
 *       o->loop_dt_max_us = st_dtmax;
 *       o->spy_drop       = spy_dropped();
 *   }
 *
 * CAUTION: st_lost (LOST) over-counts ~16x because of the driver glitch -
 * for determining the ceiling, st_ro_hole is the correct figure.
 */
void wifi_bench_set_ext(wb_ext_fn fn);

/* ---- life cycle -------------------------------------------------------- */
void wifi_bench_init(void);
void wifi_bench_stop(void);
bool wifi_bench_running(void);

/* ---- command parser ----------------------------------------------------
 * true = this module handled the line (do not pass it to other parsers).
 *
 *   B                                   status + current config
 *   B0                                  stop
 *   Br<kBps>[,<sec>]                    fixed rate (sec omitted: unlimited)
 *   Bs<start>,<stop>,<step>,<dwell_s>   stepped ramp in kB/s
 *   Bm[<sec>]                           max: no throttling, ceiling search
 *   Bset k=v [k=v ...]                  config (see below)
 *
 * Bset keys:
 *   port=7777      listen port (only for a stopped bench)
 *   blk=1040       frame size in bytes (24 B header + payload); 64..8192
 *   chunk=2880     max size of one send() call (2880 = 2x MSS)
 *   core=0         core of the bench task (0 or 1)   [WiFi core = 0 recommended]
 *   prio=5         priority of the bench task (1..20; spi_rx_task is 22!)
 *   nodelay=1      TCP_NODELAY
 *   sndbuf=0       SO_SNDBUF in bytes (0 = do not set)
 *   warm=2         warm-up seconds per step (not counted)
 *   win=1000       report window in ms
 *   agg=1          frames per burst (1 = faithful to the IQ stream)
 *   burst=4        max tokens (in frames) that may accumulate (burstiness)
 *   quiet=0        1 = print only the per-step summary lines
 */
bool wifi_bench_cmd(const char *line);

#ifdef __cplusplus
}
#endif

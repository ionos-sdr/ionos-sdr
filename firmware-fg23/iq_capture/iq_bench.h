/* SPDX-License-Identifier: MIT
 *
 * iq_bench.h — FG23 continuous I/Q rate benchmark
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * Lives ALONGSIDE app.c: it defines neither RAILCb_SetupRxFifo nor
 * sl_rail_util_on_event, and does NOT reconfigure the RX FIFO — it keeps
 * using the 4096-byte rx_fifo_words buffer of app.c. It only changes the
 * THRESHOLD for the duration of the measurement and restores it at the end.
 */

#ifndef IQ_BENCH_H
#define IQ_BENCH_H

#include "rail.h"
#include <stdbool.h>
#include <stdint.h>

/* Whether a measurement is running. The app.c event callback branches on it. */
bool iq_bench_active(void);

/* Must be called from app.c sl_rail_util_on_event(), FIRST.
 * If it returns true, the app.c capture branch is SKIPPED. */
bool iq_bench_on_event(RAIL_Handle_t rail, RAIL_Events_t events);

/* Full threshold sweep + tabular printout.
 *   channel        — the same as app.c s_channel uses
 *   seconds        — measurement length per point (10 = quick, 60 = final)
 *   restore_thresh — the threshold restored after the measurement
 *                    (pass THRESHOLD_BYTES from app.c)
 * The function blocks until the sweep ends. */
void iq_bench_sweep(RAIL_Handle_t rail,
                    uint16_t channel,
                    uint32_t seconds,
                    uint16_t restore_thresh);

/* Zero-copy (NULL destination) read trial + comparative measurement.
 * First performs a single cautious trial read; only if that succeeds
 * does it run the two 10-second comparative measurements. */
void iq_bench_zerocopy_test(RAIL_Handle_t rail,
                            uint16_t channel,
                            uint32_t seconds,
                            uint16_t restore_thresh);

#endif /* IQ_BENCH_H */

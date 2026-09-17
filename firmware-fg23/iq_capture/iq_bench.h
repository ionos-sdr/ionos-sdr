/* SPDX-License-Identifier: MIT
 *
 * iq_bench.h — FG23 folyamatos I/Q rata benchmark
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * Az app.c-hez PARHUZAMOSAN el: nem definial RAILCb_SetupRxFifo-t,
 * nem definial sl_rail_util_on_event-et, es NEM allitja at az RX FIFO-t
 * — az app.c 4096 bajtos rx_fifo_words bufferet hasznalja tovabb.
 * Csak a THRESHOLD-ot allitja at meres idejere, es a vegen visszaallitja.
 */

#ifndef IQ_BENCH_H
#define IQ_BENCH_H

#include "rail.h"
#include <stdbool.h>
#include <stdint.h>

/* Fut-e eppen meres. Az app.c esemeny-callbackje ezzel agazik el. */
bool iq_bench_active(void);

/* Az app.c sl_rail_util_on_event()-jebol kell hivni, LEGELOL.
 * Ha true-val ter vissza, az app.c sajat capture-aga KIMARAD. */
bool iq_bench_on_event(RAIL_Handle_t rail, RAIL_Events_t events);

/* Teljes threshold-sweep + tablazatos kiiras.
 *   channel        — ugyanaz, amit az app.c s_channel-je hasznal
 *   seconds        — meres hossza pontonkent (10 = gyors, 60 = vegleges)
 *   restore_thresh — meres utan erre allitja vissza a thresholdot
 *                    (add at a THRESHOLD_BYTES-t az app.c-bol)
 * A fuggveny blokkol a sweep vegeig. */
void iq_bench_sweep(RAIL_Handle_t rail,
                    uint16_t channel,
                    uint32_t seconds,
                    uint16_t restore_thresh);

/* Zero-copy (NULL celcimu) olvasas probaja + osszehasonlito meres.
 * Eloszor egyetlen ovatos probaolvasast vegez; csak ha az sikeres,
 * futtatja a ket 10 mp-es osszehasonlito merest. */
void iq_bench_zerocopy_test(RAIL_Handle_t rail,
                            uint16_t channel,
                            uint32_t seconds,
                            uint16_t restore_thresh);

#endif /* IQ_BENCH_H */

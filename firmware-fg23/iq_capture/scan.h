/* SPDX-License-Identifier: MIT
 *
 * scan.h — wideband RSSI panadapter (stepped scan) on the FG23
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * WHAT IT DOES: steps the synth across a band (center +- span/2), reads
 * the RSSI at every point, and sends the finished line (nbin x uint8 dB)
 * in a SPECLINE block (specline.h) over SPI to the ESP32 — on the same
 * block path as the I/Q (iq_stream ext mode).
 *
 * WHAT IT DOES NOT DO: no I/Q output. The scan and iq_stream are MUTUALLY
 * EXCLUSIVE: before scan_start() app.c stops the stream, after scan_stop()
 * it restores RX (restart_rx + set_freq_tick).
 *
 * COMMAND (terminal AND cmdlink, identical):
 *   W<center_kHz>,<span_kHz>,<nbin>    e.g. W149800,10000,320
 *   W0                                  end of scan
 *
 * TIMING (default): per bin Idle+StartRx (~50-100 us) + SCAN_SETTLE_US +
 * RSSI wait (<= SCAN_RSSI_WAIT_US). 320 bins ~ 150-200 ms/line.
 * SCAN_BINS_PER_CALL limits how much is done in one app_process_action
 * call, so that cmdlink/VCOM stay responsive.
 *
 * DATASHEET TODO: the settle and RSSI-wait values must be finalised per
 * the datasheet/RAIL; currently a conservative estimate.
 */

#ifndef SCAN_H
#define SCAN_H

#include "rail.h"
#include <stdbool.h>
#include <stdint.h>

/* The tuning grid defined by app.c (TUNE_*). This way the scan uses the
 * same channel+offset arithmetic as tune_khz(). */
typedef struct {
  uint32_t base_khz;      /* TUNE_BASE_KHZ */
  uint16_t spacing_khz;   /* TUNE_SPACING_KHZ (0 = no grid) */
  uint16_t max_channel;   /* TUNE_MAX_CHANNEL */
  double   tick_hz;       /* TUNE_TICK_MHZ (Hz/tick) */
  int32_t  corr_ppb;      /* FREQ_CORR_PPB */
} scan_grid_t;

void scan_init(RAIL_Handle_t rail, const scan_grid_t *grid);

/* Start / reconfigure. nbin 100..SPECLINE_MAX_BINS. Returns false if the
 * parameters make no sense. If already running, re-parameterises it. */
bool scan_start(uint32_t center_khz, uint32_t span_khz, uint16_t nbin);
void scan_stop(void);
bool scan_active(void);

/* From the main loop. Does not block for long (max SCAN_BINS_PER_CALL bins). */
void scan_process(void);

/* Statistics for the 's' command. */
void scan_print_stats(void);

/* The dB scale written into the SPECLINE header. The client (SDR++ module)
 * expects the same via its FFT_DB_OFFSET/RANGE settings; if they differ
 * there, set it here (W command fields 4-5: optional). */
void scan_set_scale(int16_t floor_dbm, uint16_t range_db);

/* Timing at run time ('T<avg_us>,<wait_us>[,<mode>]' command). */
void scan_set_timing(uint32_t settle_us, uint32_t wait_us);

/* Measurement method: 0 = RAIL_StartAverageRssi, 1 = StartRx + blocking
 * GetRssiAlt (default), 2 = StartRx + polling GetRssiAlt. Field 3 of the
 * T command. */
void scan_set_method(uint8_t m);

/* Yield hook: the scan calls it during its long waits (settle, RSSI wait)
 * so that the cmdlink EUSART FIFO (16 bytes!) does not overflow. app.c
 * hooks in cmdlink_poll(). The hook may call scan_start/stop. */
typedef void (*scan_yield_fn)(void);
void scan_set_yield(scan_yield_fn fn);

#endif /* SCAN_H */

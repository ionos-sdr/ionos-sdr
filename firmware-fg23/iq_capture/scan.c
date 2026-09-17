/* SPDX-License-Identifier: MIT
 *
 * scan.c — szelessavu RSSI-panadapter (leptetett scan) a FG23-on
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Lasd scan.h. Allapotgep:
 *   IDLE  -> (scan_start) -> SWEEP : binenkent hangol + RSSI, s_line-ba
 *   SWEEP -> (utolso bin)  -> SEND  : a kesz sort SPECLINE-kent kikuldi
 *   SEND  -> (elkuldve)    -> SWEEP : uj sor
 *
 * 2026-08-15:
 *  - SetFreqOffset minden binben (kristaly + residual).
 *  - Teljes Idle+StartRx csak ha a cel a jelenlegi csatorna ±OFFSET_MAX
 *    ablakán kivul esik. Egy StartRx így ~150 kHz-et fed le offset-tel,
 *    16 MHz-en ~110 restart (a regi 641 helyett).
 */

#include "scan.h"
#include "specline.h"
#include "iq_stream.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ---- hangolhato parameterek ---- */
#define SCAN_AVG_US_DEF       100u
#define SCAN_RSSI_WAIT_US_DEF 400u
#define SCAN_BINS_PER_CALL    48u
#define SCAN_RSSI_SLICE_US    250u
#define SCAN_MIN_NBIN         100u

/* A synth fine-offset tartomanya ±16383 tick. Biztonsagi tartalekkal
 * ±15000 tick-et hasznalunk (~±70 kHz @ 4.65 Hz/tick). Ezen belul
 * nincs szukseg uj StartRx-re, csak SetFreqOffset-re. */
#define SCAN_OFFSET_TICK_MAX  15000

#define SCAN_M_AVG   0u
#define SCAN_M_BLOCK 1u
#define SCAN_M_POLL  2u
#define SCAN_METHOD_DEF SCAN_M_BLOCK

static uint32_t s_avg_us    = SCAN_AVG_US_DEF;
static uint32_t s_wait_us   = SCAN_RSSI_WAIT_US_DEF;
static uint8_t  s_method    = SCAN_METHOD_DEF;

/* ---- allapot ---- */
static RAIL_Handle_t s_rail = NULL;
static scan_grid_t   s_grid;
static bool          s_active = false;
static bool          s_line_ready = false;

static uint32_t s_center_khz = 0;
static uint32_t s_span_khz   = 0;
static uint16_t s_nbin       = 0;
static uint16_t s_bin        = 0;
static int16_t  s_floor_dbm  = -130;
static uint16_t s_range_db   = 100;

static specline_blk_t s_line __attribute__((aligned(4)));

static scan_yield_fn s_yield = NULL;
static uint32_t      s_gen   = 0;
static uint32_t s_line_inv   = 0;
static uint32_t s_diag_lines = 0;
static uint8_t  s_diag_stat  = 0;
static int16_t  s_line_min   = 0;
static int16_t  s_line_max   = 0;
static uint32_t s_line_ok    = 0;

static uint32_t s_lines_sent = 0;
static uint32_t s_lines_skip = 0;
static uint32_t s_rssi_invalid = 0;
static uint32_t s_out_of_grid = 0;
static uint32_t s_t_line_start = 0;
static uint32_t s_last_line_us = 0;
static bool     s_warned_grid = false;

/* measure_bin: szeles offset-ablak allapota */
static uint16_t s_last_ch  = 0xFFFFu;
static bool     s_rx_live  = false;
static uint32_t s_restarts = 0;

/* ---- segedek ---- */

static int32_t corr_tick_for_khz(uint32_t khz)
{
  double hz = (double)khz * 1000.0 * (double)s_grid.corr_ppb / 1e9;
  return (int32_t)(hz / s_grid.tick_hz + (hz >= 0 ? 0.5 : -0.5));
}

static uint32_t bin_freq_hz(uint16_t i)
{
  int64_t f = (int64_t)s_center_khz * 1000 - (int64_t)s_span_khz * 500;
  f += ((int64_t)s_span_khz * 1000 * (2 * (int64_t)i + 1)) / (2 * (int64_t)s_nbin);
  if (f < 0) f = 0;
  return (uint32_t)f;
}

/* Csatorna kozepfrekvenciaja Hz-ben. */
static uint32_t ch_center_hz(uint16_t ch)
{
  return (uint32_t)((uint64_t)s_grid.base_khz * 1000ull
                  + (uint64_t)ch * (uint64_t)s_grid.spacing_khz * 1000ull);
}

/* Cel-Hz -> ch + tick.
 * Ha prefer_ch ervenyes (nem 0xFFFF) es a residual belefer a
 * SCAN_OFFSET_TICK_MAX ablakba, azt a csatornat tartjuk (nincs restart).
 * Kulonben a legkozelebbi racspontot valasztjuk. */
static bool freq_to_ch_tick(uint32_t hz, uint16_t prefer_ch,
                            uint16_t *ch_out, int32_t *tick_out)
{
  int32_t corr = corr_tick_for_khz((uint32_t)(hz / 1000u));

  /* 1) Probaljuk a preferalt csatornat (szeles offset-ablak). */
  if (prefer_ch != 0xFFFFu && s_grid.spacing_khz > 0) {
    int64_t center = (int64_t)ch_center_hz(prefer_ch);
    int64_t res_hz = (int64_t)hz - center;
    int32_t tick = (int32_t)((double)res_hz / s_grid.tick_hz) + corr;
    if (tick >= -SCAN_OFFSET_TICK_MAX && tick <= SCAN_OFFSET_TICK_MAX) {
      *ch_out = prefer_ch;
      *tick_out = tick;
      return true;
    }
  }

  /* 2) Legkozelebbi racspont (vagy tiszta offset, ha nincs racs). */
  int64_t d_hz = (int64_t)hz - (int64_t)s_grid.base_khz * 1000;
  int32_t ch = 0;
  int64_t res_hz;

  if (s_grid.spacing_khz > 0) {
    int64_t sp = (int64_t)s_grid.spacing_khz * 1000;
    ch = (int32_t)((d_hz >= 0) ? (d_hz + sp / 2) / sp : (d_hz - sp / 2) / sp);
    if (ch < 0 || ch > (int32_t)s_grid.max_channel) return false;
    res_hz = d_hz - (int64_t)ch * sp;
  } else {
    res_hz = d_hz;
  }

  int32_t tick = (int32_t)((double)res_hz / s_grid.tick_hz) + corr;
  if (tick > 16383 || tick < -16383) return false;

  *ch_out = (uint16_t)ch;
  *tick_out = tick;
  return true;
}

static uint8_t dbm_to_u8(int16_t dbm)
{
  int32_t v = ((int32_t)dbm - (int32_t)s_floor_dbm) * 255 / (int32_t)s_range_db;
  if (v < 0)   v = 0;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

/* Egy bin merese. Szeles offset-ablak: amig a cel a jelenlegi ch
 * ±70 kHz-es tartomanyaban van, csak SetFreqOffset, nincs Idle. */
static void measure_bin(uint16_t i)
{
  uint16_t ch; int32_t tick;
  uint32_t hz = bin_freq_hz(i);

  /* prefer: ha fut a RX, proba a jelenlegi csatornaval */
  uint16_t prefer = s_rx_live ? s_last_ch : 0xFFFFu;

  if (!freq_to_ch_tick(hz, prefer, &ch, &tick)) {
    s_out_of_grid++;
    s_line.bins[i] = 0;
    if (!s_warned_grid) {
      s_warned_grid = true;
      printf("# scan: %lu Hz a racson kivul (base %lu kHz, %u ch x %u kHz) "
             "— ezek a binek padlot kapnak\r\n",
             (unsigned long)hz, (unsigned long)s_grid.base_khz,
             (unsigned)(s_grid.max_channel + 1u), (unsigned)s_grid.spacing_khz);
    }
    return;
  }

  uint32_t gen = s_gen;
  int16_t rq = RAIL_RSSI_INVALID;
  RAIL_Status_t st_start = RAIL_STATUS_NO_ERROR;

  bool need_restart = !s_rx_live || (ch != s_last_ch);

  if (need_restart) {
    RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
    RAIL_ResetFifo(s_rail, false, true);
    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)tick);

    if (s_method == SCAN_M_AVG) {
      st_start = RAIL_StartAverageRssi(s_rail, ch, (RAIL_Time_t)s_avg_us, NULL);
    } else {
      st_start = RAIL_StartRx(s_rail, ch, NULL);
    }
    s_last_ch = ch;
    s_rx_live = (st_start == RAIL_STATUS_NO_ERROR);
    if (s_rx_live) s_restarts++;
  } else {
    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)tick);
  }

  if (need_restart && st_start != RAIL_STATUS_NO_ERROR) {
    s_out_of_grid++;
    s_rx_live = false;
    if (!s_warned_grid) {
      s_warned_grid = true;
      printf("# scan: a RAIL NEM fogadja el a %u. csatornat "
             "(status 0x%02X).\r\n",
             (unsigned)ch, (unsigned)st_start);
    }
  }

  if (s_rx_live) {
    RAIL_Time_t tl = RAIL_GetTime() + s_wait_us;
    for (;;) {
      if (s_method == SCAN_M_AVG) {
        rq = RAIL_GetAverageRssi(s_rail);
      } else if (s_method == SCAN_M_BLOCK) {
        rq = RAIL_GetRssiAlt(s_rail, (RAIL_Time_t)SCAN_RSSI_SLICE_US);
      } else {
        rq = RAIL_GetRssiAlt(s_rail, 0u);
      }
      if (rq != RAIL_RSSI_INVALID) break;
      if ((int32_t)(RAIL_GetTime() - tl) >= 0) break;
      if (s_yield) { s_yield(); if (s_gen != gen || !s_active) goto abort_bin; }
    }
  }

  if (s_diag_stat < 2u) {
    s_diag_stat++;
    printf("# scan meres: mod=%u ch=%u tick=%ld restart=%u start_status=%d rq=%d (%d dBm)\r\n",
           (unsigned)s_method, (unsigned)ch, (long)tick, (unsigned)need_restart,
           (int)st_start, (int)rq,
           (int)(rq == RAIL_RSSI_INVALID ? -999 : rq / 4));
  }

  if (rq == RAIL_RSSI_INVALID) {
    s_rssi_invalid++; s_line_inv++;
    s_line.bins[i] = 0;
    s_rx_live = false;
  } else {
    int16_t dbm = (int16_t)(rq / 4);
    if (s_line_ok == 0u) { s_line_min = dbm; s_line_max = dbm; }
    else { if (dbm < s_line_min) s_line_min = dbm; if (dbm > s_line_max) s_line_max = dbm; }
    s_line_ok++;
    s_line.bins[i] = dbm_to_u8(dbm);
  }
  return;

abort_bin:
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  s_rx_live = false;
}

/* ---- API ---- */

void scan_init(RAIL_Handle_t rail, const scan_grid_t *grid)
{
  s_rail = rail;
  s_grid = *grid;
}

void scan_set_yield(scan_yield_fn fn) { s_yield = fn; }

void scan_set_timing(uint32_t avg_us, uint32_t wait_us)
{
  if (avg_us  < 20u)    avg_us  = 20u;
  if (avg_us  > 20000u) avg_us  = 20000u;
  if (wait_us > 50000u) wait_us = 50000u;
  s_avg_us = avg_us; s_wait_us = wait_us;
  printf("# scan idozites: atlagolas %lu us, varakozas max %lu us, mod %u\r\n",
         (unsigned long)s_avg_us, (unsigned long)s_wait_us, (unsigned)s_method);
}

void scan_set_method(uint8_t m)
{
  if (m > SCAN_M_POLL) m = SCAN_M_POLL;
  s_method = m;
  s_diag_stat = 0;
  printf("# scan meresi mod = %u (%s)\r\n", (unsigned)m,
         (m == SCAN_M_AVG) ? "StartAverageRssi" :
         (m == SCAN_M_BLOCK) ? "StartRx + blokkolo GetRssiAlt"
                             : "StartRx + pollozo GetRssiAlt");
}

void scan_set_scale(int16_t floor_dbm, uint16_t range_db)
{
  if (range_db < 10u)  range_db = 10u;
  if (range_db > 150u) range_db = 150u;
  s_floor_dbm = floor_dbm;
  s_range_db  = range_db;
}

bool scan_start(uint32_t center_khz, uint32_t span_khz, uint16_t nbin)
{
  if (!s_rail) return false;
  if (nbin < SCAN_MIN_NBIN) nbin = SCAN_MIN_NBIN;
  if (nbin > SPECLINE_MAX_BINS) nbin = SPECLINE_MAX_BINS;
  if (span_khz < 100u || span_khz > 100000u) return false;
  if (center_khz < 1000u) return false;

  s_center_khz = center_khz;
  s_span_khz   = span_khz;
  s_nbin       = nbin;
  s_bin        = 0;
  s_line_ready = false;
  s_warned_grid = false;
  s_line_inv = 0; s_diag_lines = 0; s_diag_stat = 0;
  s_line_ok = 0; s_line_min = 0; s_line_max = 0;
  s_last_ch = 0xFFFFu;
  s_rx_live = false;
  s_restarts = 0;
  s_gen++;
  memset(&s_line, 0, sizeof s_line);

  if (!s_active) {
    RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL, RAIL_EVENT_RSSI_AVERAGE_DONE);
    iq_stream_ext_begin();
    s_lines_sent = 0; s_lines_skip = 0; s_rssi_invalid = 0; s_out_of_grid = 0;
    s_active = true;
    printf("# scan indul: kozep %lu kHz, span %lu kHz, %u bin, "
           "lepes %lu Hz, skala %d dBm + %u dB (offset-ablak ±%d tick)\r\n",
           (unsigned long)center_khz, (unsigned long)span_khz, (unsigned)nbin,
           (unsigned long)((uint64_t)span_khz * 1000ull / nbin),
           (int)s_floor_dbm, (unsigned)s_range_db, SCAN_OFFSET_TICK_MAX);
  } else {
    printf("# scan atallitva: kozep %lu kHz, span %lu kHz, %u bin\r\n",
           (unsigned long)center_khz, (unsigned long)span_khz, (unsigned)nbin);
  }
  s_t_line_start = (uint32_t)RAIL_GetTime();
  return true;
}

void scan_stop(void)
{
  if (!s_active) return;
  s_active = false;
  s_line_ready = false;
  s_rx_live = false;
  s_last_ch = 0xFFFFu;
  s_gen++;
  {
    RAIL_Time_t tl = RAIL_GetTime() + 5000u;
    while (iq_stream_ext_busy() && (int32_t)(RAIL_GetTime() - tl) < 0) {
      iq_stream_ext_pump();
    }
  }
  iq_stream_ext_end();
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  printf("# scan leallt: %lu sor kikuldve, %lu halasztott, "
         "%lu ervenytelen RSSI, %lu racson kivuli bin, utolso sor %lu ms\r\n",
         (unsigned long)s_lines_sent, (unsigned long)s_lines_skip,
         (unsigned long)s_rssi_invalid, (unsigned long)s_out_of_grid,
         (unsigned long)(s_last_line_us / 1000u));
}

bool scan_active(void) { return s_active; }

void scan_process(void)
{
  if (!s_active) return;

  iq_stream_ext_pump();

  if (s_line_ready) {
    if (iq_stream_ext_busy()) return;
    if (iq_stream_ext_send(&s_line)) {
      s_line_ready = false;
      s_lines_sent++;
      s_bin = 0;
      s_restarts = 0;
      s_t_line_start = (uint32_t)RAIL_GetTime();
    } else {
      s_lines_skip++;
      return;
    }
  }

  uint32_t n = 0;
  uint32_t gen = s_gen;
  while (s_active && s_gen == gen && s_bin < s_nbin && n < SCAN_BINS_PER_CALL) {
    measure_bin(s_bin);
    if (!s_active || s_gen != gen) return;
    s_bin++; n++;
  }
  if (!s_active || s_gen != gen) return;

  if (s_bin >= s_nbin) {
    if (s_diag_lines < 3u) {
      s_diag_lines++;
      printf("# scan sor %lu: %u bin, %lu ervenytelen, %lu ervenyes "
             "(min %d dBm, max %d dBm), %lu ms, restart %lu\r\n",
             (unsigned long)(s_lines_sent + 1u), (unsigned)s_nbin,
             (unsigned long)s_line_inv, (unsigned long)s_line_ok,
             (int)s_line_min, (int)s_line_max,
             (unsigned long)(((uint32_t)RAIL_GetTime() - s_t_line_start) / 1000u),
             (unsigned long)s_restarts);
    }
    s_line_inv = 0; s_line_ok = 0;
    s_line.hdr.magic       = SPECLINE_MAGIC;
    s_line.hdr.nbin        = s_nbin;
    s_line.hdr.flags       = SPECLINE_FLAG_LAST;
    s_line.hdr.f_center_hz = s_center_khz * 1000u;
    s_line.span_hz         = s_span_khz * 1000u;
    s_line.floor_dbm       = s_floor_dbm;
    s_line.range_db        = s_range_db;
    s_last_line_us = (uint32_t)RAIL_GetTime() - s_t_line_start;
    s_line_ready = true;
    if (!iq_stream_ext_busy() && iq_stream_ext_send(&s_line)) {
      s_line_ready = false;
      s_lines_sent++;
      s_bin = 0;
      s_restarts = 0;
      s_t_line_start = (uint32_t)RAIL_GetTime();
    }
  }
}

void scan_print_stats(void)
{
  printf("# scan idozites: atlagolas %lu us, varakozas max %lu us, mod %u\r\n",
         (unsigned long)s_avg_us, (unsigned long)s_wait_us, (unsigned)s_method);
  printf("# scan=%d kozep=%lu kHz span=%lu kHz nbin=%u  sor=%lu "
         "halasztott=%lu rssi_inv=%lu racson_kivul=%lu  sor_ido=%lu ms\r\n",
         (int)s_active, (unsigned long)s_center_khz, (unsigned long)s_span_khz,
         (unsigned)s_nbin, (unsigned long)s_lines_sent,
         (unsigned long)s_lines_skip, (unsigned long)s_rssi_invalid,
         (unsigned long)s_out_of_grid, (unsigned long)(s_last_line_us / 1000u));
}

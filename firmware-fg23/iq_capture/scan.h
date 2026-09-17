/* SPDX-License-Identifier: MIT
 *
 * scan.h — szelessavu RSSI-panadapter (leptetett scan) a FG23-on
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * MIT CSINAL: a synth-et vegiglepteti egy savon (kozep +- span/2), minden
 * pontban RSSI-t olvas, es a kesz sort (nbin x uint8 dB) egy SPECLINE
 * blokkban (specline.h) kikuldi az SPI-n az ESP32-nek — ugyanazon a
 * blokk-uton, mint az I/Q (iq_stream ext-mod).
 *
 * MIT NEM CSINAL: I/Q-t nem ad. A scan es az iq_stream KIZARJAK egymast:
 * a scan_start() elott az app.c leallitja a streamet, a scan_stop() utan
 * visszaallitja az RX-et (restart_rx + set_freq_tick).
 *
 * PARANCS (terminal ES cmdlink, ugyanaz):
 *   W<kozep_kHz>,<span_kHz>,<nbin>     pl. W149800,10000,320
 *   W0                                  scan vege
 *
 * IDOZITES (alap): binenkent Idle+StartRx (~50-100 us) + SCAN_SETTLE_US +
 * RSSI-varakozas (<= SCAN_RSSI_WAIT_US). 320 bin ~ 150-200 ms/sor. A
 * SCAN_BINS_PER_CALL korlatozza, mennyit dolgozunk egy app_process_action
 * hivasban, hogy a cmdlink/VCOM valaszkepes maradjon.
 *
 * ADATLAP-TODO: a settle es a RSSI-varakozas erteket a datasheet/RAIL
 * szerint kell veglegesiteni; most konzervativ becsles.
 */

#ifndef SCAN_H
#define SCAN_H

#include "rail.h"
#include <stdbool.h>
#include <stdint.h>

/* A hangolasi racs, amit az app.c definial (TUNE_*). Igy a scan ugyanazt
 * a csatorna+offszet aritmetikat hasznalja, mint a tune_khz(). */
typedef struct {
  uint32_t base_khz;      /* TUNE_BASE_KHZ */
  uint16_t spacing_khz;   /* TUNE_SPACING_KHZ (0 = nincs racs) */
  uint16_t max_channel;   /* TUNE_MAX_CHANNEL */
  double   tick_hz;       /* TUNE_TICK_MHZ (Hz/tick) */
  int32_t  corr_ppb;      /* FREQ_CORR_PPB */
} scan_grid_t;

void scan_init(RAIL_Handle_t rail, const scan_grid_t *grid);

/* Inditas / ujrakonfiguralas. nbin 100..SPECLINE_MAX_BINS. Visszaad false,
 * ha a parameterek ertelmetlenek. Ha mar fut, atparameterezi. */
bool scan_start(uint32_t center_khz, uint32_t span_khz, uint16_t nbin);
void scan_stop(void);
bool scan_active(void);

/* A fo ciklusbol. Nem blokkol hosszan (max SCAN_BINS_PER_CALL bin). */
void scan_process(void);

/* Statisztika az 's' parancshoz. */
void scan_print_stats(void);

/* A dB-skala, amit a SPECLINE fejlecebe irunk. A kliens (SDR++ modul) az
 * FFT_DB_OFFSET/RANGE settingekkel ugyanezt varja; ha ott mas van, itt
 * allitsd (W parancs 4-5. mezoje: opcionalis). */
void scan_set_scale(int16_t floor_dbm, uint16_t range_db);

/* Idozites futas kozben ('T<avg_us>,<wait_us>[,<mod>]' parancs). */
void scan_set_timing(uint32_t settle_us, uint32_t wait_us);

/* Meresi mod: 0 = RAIL_StartAverageRssi, 1 = StartRx + blokkolo GetRssiAlt
 * (alap), 2 = StartRx + pollozo GetRssiAlt. A T parancs 3. mezeje. */
void scan_set_method(uint8_t m);

/* Yield-hook: a scan a hosszu varakozasai alatt hivja (settle, RSSI-vara-
 * kozas), hogy a cmdlink EUSART FIFO-ja (16 bajt!) ne csorduljon tul. Az
 * app.c a cmdlink_poll()-t akasztja be. A hook hivhat scan_start/stop-ot. */
typedef void (*scan_yield_fn)(void);
void scan_set_yield(scan_yield_fn fn);

#endif /* SCAN_H */

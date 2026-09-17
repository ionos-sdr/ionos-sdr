/* SPDX-License-Identifier: MIT
 *
 * FG23 fazis-koherens I/Q burst capture
 * Simplicity SDK 2025.6.3 / RAIL 2.19.x — "RAIL - SoC Empty" projektbe
 *
 * A capture-mag a geckokapula projekt dsp_driver.c-jebol szarmazik:
 *   Copyright (c) 2017-2022 Tatu Peltola (OH2EAT) — MIT licenc
 *   https://github.com/tejeez/geckokapula
 * Series 2 port es burst-dump:
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * ELHELYEZES: a SoC Empty sablonban az app_init() az app_init.c-be, az
 * app_process_action() az app_process.c-be valo — vagy az egesz mehet
 * egyetlen app.c-be, ha a sablon ures fuggvenyeit torlod.
 *
 * KOMPONENSEK (Software Components):
 *   - RAIL Utility, Initialization  (peldany: inst0; "Enable Setup of
 *     Radio Events" BE — ez adja a sl_rail_util_on_event routingot)
 *   - a sajat 144.489 MHz-es Radio Configurator PHY (base 144.489 MHz,
 *     ch spacing 1 MHz, 39 MHz crystal — a RAILtest-feasibility configja)
 *   - IO Stream: EUSART (peldany: vcom) + IO Stream: Retarget STDIO
 *
 * MERT BINARIS A DUMP ES MIERT sl_iostream_write:
 *   a retargetelt printf/stdout utvonal pufferelhet es LF-konverziot
 *   vegezhet, ami a binaris keretet elrontana — a payload ezert megy
 *   kozvetlenul sl_iostream_write-tal.
 *
 * MINTAFORMATUM (geckokapula dsp.h + a mai RAILtest-forenzika):
 *   struct { int16_t q, i; } — Q ELOL, little-endian ("i16le" a
 *   Python oldalon). Az iq_view_stream.py IQB1 modja pont ezt varja.
 */

#include "rail.h"
#include "sl_rail_util_init.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ---------------- konfiguracio ---------------- */

/* Q elol! (geckokapula iq_in_t) */
typedef struct {
  int16_t q;
  int16_t i;
} iq_in_t;

/* Burst hossza: 8192 komplex minta = 32 KiB RAM (a FG23B 64 KiB-jaba
 * boven belefer). 48 kHz I/Q-nal ~170 ms, 160 kHz-nel ~51 ms. */
#define CAPTURE_SAMPLES   8192u

/* Esemenyenkent olvasott komplex mintak. A geckokapula 2-t olvasott
 * (audio-szinkron miatt); nekunk 64 minta / esemeny = 256 bajt jo,
 * ritkabb IRQ. */
#define SAMPLES_PER_EVENT 64u
#define THRESHOLD_BYTES   (SAMPLES_PER_EVENT * sizeof(iq_in_t))

/* Series 2: az RX FIFO-t az app adja a RAILCb_SetupRxFifo-n keresztul. */
#define RX_FIFO_BYTES     4096u

/* A csatorna, amin a PHY 144.489 MHz-et ad (configtol fugg). */
#define IQ_CHANNEL        0u

/* ---------------- allapot ---------------- */

static iq_in_t capture_buf[CAPTURE_SAMPLES];
static volatile uint32_t capture_idx  = 0;
static volatile bool     capturing    = false;
static volatile bool     capture_done = false;
static volatile bool     capture_bad  = false;
static volatile uint32_t stat_events = 0, stat_short_reads = 0,
                         stat_overflows = 0;

SL_ALIGN(4)
static uint8_t rx_fifo[RX_FIFO_BYTES] SL_ATTRIBUTE_ALIGN(4);

static RAIL_Handle_t s_rail = NULL;

/* ---------------- RX FIFO (Series 2) ----------------
 * Ha a linker "multiple definition of RAILCb_SetupRxFifo" hibat dob,
 * a projekt mar ad sajatot (pl. valamelyik pelda-forras) — akkor EZT
 * a fuggvenyt torold, es a masikban allitsd a meretet. */
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle)
{
  uint16_t size = RX_FIFO_BYTES;
  RAIL_Status_t st = RAIL_SetRxFifo(railHandle, rx_fifo, &size);
  return st;
}

/* ---------------- esemeny-callback ----------------
 * ISR-kontextus (a RAIL majdnem mindig megszakitasbol hiv) — csak
 * FIFO-olvasas es indexeles, semmi mas. A geckokapula rail_callback()
 * kozvetlen leszarmazottja. */
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    ++stat_overflows;
    if (capturing) capture_bad = true;
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    ++stat_events;

    if (!capturing) {
      /* Folyamatos vetel, de nem gyujtunk: urritunk es eldobunk,
       * hogy a FIFO sose csorduljon tul — igy a burst inditasa
       * pillanatszeru es az elso mintatol koherens. */
      uint8_t sink[THRESHOLD_BYTES];
      RAIL_ReadRxFifo(rail_handle, sink, sizeof sink);
      return;
    }

    uint32_t want = THRESHOLD_BYTES;
    uint32_t room = (CAPTURE_SAMPLES - capture_idx) * sizeof(iq_in_t);
    if (want > room) want = room;

    uint16_t nread = RAIL_ReadRxFifo(rail_handle,
                                     (uint8_t *)&capture_buf[capture_idx],
                                     (uint16_t)want);
    if (nread != want) ++stat_short_reads;
    capture_idx += nread / sizeof(iq_in_t);

    if (capture_idx >= CAPTURE_SAMPLES) {
      capturing = false;
      capture_done = true;
    }
  }
}

/* ---------------- burst dump ----------------
 * Onleiro keret: "IQB1" | u32 n | u32 fs_hint | u8 fmt(=1) | 3x pad |
 * payload | "IQE1" — az iq_view_stream.py --port/--file modja olvassa. */
static void dump_capture(uint32_t fs_hint)
{
  static const uint8_t hdr[4] = { 'I', 'Q', 'B', '1' };
  static const uint8_t trl[4] = { 'I', 'Q', 'E', '1' };
  uint32_t n = capture_idx;
  uint8_t meta[12];
  memcpy(&meta[0], &n, 4);
  memcpy(&meta[4], &fs_hint, 4);
  meta[8] = 1; meta[9] = meta[10] = meta[11] = 0;

  sl_iostream_write(sl_iostream_vcom_handle, hdr, sizeof hdr);
  sl_iostream_write(sl_iostream_vcom_handle, meta, sizeof meta);
  sl_iostream_write(sl_iostream_vcom_handle, capture_buf,
                    n * sizeof(iq_in_t));
  sl_iostream_write(sl_iostream_vcom_handle, trl, sizeof trl);
}

/* ---------------- init + fo ciklus ---------------- */

void app_init(void)
{
  s_rail = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

  RAIL_DataConfig_t dc = {
    .txSource = TX_PACKET_DATA,
    .rxSource = RX_IQDATA_FILTLSB,   /* erros jelre valto: FILTMSB */
    .txMethod = PACKET_MODE,
    .rxMethod = FIFO_MODE,
  };
  RAIL_ConfigData(s_rail, &dc);
  RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
  RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(s_rail, false, true);
  RAIL_StartRx(s_rail, IQ_CHANNEL, NULL);

  printf("\r\n# FG23 IQ burst capture (SiSDK 2025.6). "
         "'c' = capture+dump, 's' = statusz\r\n");
}

void app_process_action(void)
{
  char ch;
  /* Nem-blokkolo olvasas a VCOM-rol. Ha a getchar megis blokkolna,
   * kapcsold ki a blokkolast az iostream uart konfigban, vagy hivd:
   * sl_iostream_uart_set_read_block(sl_iostream_uart_vcom_handle, false);
   * az app_init vegen (#include "sl_iostream_uart.h" es a vcom uart
   * handle extern deklaracioja mellett). */
  if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
    if (ch == 'c' && !capturing) {
      capture_idx  = 0;
      capture_bad  = false;
      capture_done = false;
      capturing    = true;      /* a kovetkezo FIFO-esemenytol gyujt */
    } else if (ch == 's') {
      printf("# events=%lu short=%lu ovf=%lu idx=%lu rail=%p\r\n",
             (unsigned long)stat_events,
             (unsigned long)stat_short_reads,
             (unsigned long)stat_overflows,
             (unsigned long)capture_idx, (void *)s_rail);
    }
  }

  if (capture_done) {
    capture_done = false;
    if (capture_bad) {
      printf("# OVERFLOW a burst alatt — eldobva, probald ujra\r\n");
    } else {
      dump_capture(0 /* fs_hint: onkalibracio utan ird be Hz-ben */);
    }
  }
}

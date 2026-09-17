/* SPDX-License-Identifier: MIT
 *
 * FG23 phase-coherent I/Q burst capture
 * Simplicity SDK 2025.6.3 / RAIL 2.19.x, for the "RAIL - SoC Empty" project
 *
 * The capture core is derived from dsp_driver.c of the geckokapula project:
 *   Copyright (c) 2017-2022 Tatu Peltola (OH2EAT) - MIT license
 *   https://github.com/tejeez/geckokapula
 * Series 2 port and burst dump:
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD - MIT license
 *
 * PLACEMENT: in the SoC Empty template app_init() belongs in app_init.c and
 * app_process_action() in app_process.c; alternatively everything can go
 * into a single app.c if the template's empty functions are removed.
 *
 * COMPONENTS (Software Components):
 *   - RAIL Utility, Initialization  (instance: inst0; "Enable Setup of
 *     Radio Events" ON - this provides the sl_rail_util_on_event routing)
 *   - the custom 144.489 MHz Radio Configurator PHY (base 144.489 MHz,
 *     ch spacing 1 MHz, 39 MHz crystal - the RAILtest feasibility config)
 *   - IO Stream: EUSART (instance: vcom) + IO Stream: Retarget STDIO
 *
 * WHY THE DUMP IS BINARY AND WHY sl_iostream_write:
 *   the retargeted printf/stdout path may buffer and perform LF conversion,
 *   which would corrupt the binary frame - the payload is therefore written
 *   directly with sl_iostream_write.
 *
 * SAMPLE FORMAT (geckokapula dsp.h + RAILtest forensics):
 *   struct { int16_t q, i; } - Q FIRST, little-endian ("i16le" on the
 *   Python side). The IQB1 mode of iq_view_stream.py expects exactly this.
 */

#include "rail.h"
#include "sl_rail_util_init.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ---------------- configuration ---------------- */

/* Q first! (geckokapula iq_in_t) */
typedef struct {
  int16_t q;
  int16_t i;
} iq_in_t;

/* Burst length: 8192 complex samples = 32 KiB RAM (fits comfortably in the
 * FG23B's 64 KiB). ~170 ms at 48 kHz I/Q, ~51 ms at 160 kHz. */
#define CAPTURE_SAMPLES   8192u

/* Complex samples read per event. geckokapula read 2 (for audio sync);
 * here 64 samples / event = 256 bytes is adequate and gives a lower IRQ
 * rate. */
#define SAMPLES_PER_EVENT 64u
#define THRESHOLD_BYTES   (SAMPLES_PER_EVENT * sizeof(iq_in_t))

/* Series 2: the application supplies the RX FIFO via RAILCb_SetupRxFifo. */
#define RX_FIFO_BYTES     4096u

/* The channel on which the PHY yields 144.489 MHz (config dependent). */
#define IQ_CHANNEL        0u

/* ---------------- state ---------------- */

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
 * If the linker reports "multiple definition of RAILCb_SetupRxFifo", the
 * project already provides one (e.g. from an example source) - in that
 * case delete THIS function and set the size in the other one. */
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle)
{
  uint16_t size = RX_FIFO_BYTES;
  RAIL_Status_t st = RAIL_SetRxFifo(railHandle, rx_fifo, &size);
  return st;
}

/* ---------------- event callback ----------------
 * ISR context (RAIL almost always calls from an interrupt) - FIFO read
 * and indexing only, nothing else. A direct descendant of geckokapula's
 * rail_callback(). */
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    ++stat_overflows;
    if (capturing) capture_bad = true;
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    ++stat_events;

    if (!capturing) {
      /* Continuous reception without capturing: drain and discard so
       * the FIFO never overflows - this makes the burst start
       * instantaneous and coherent from the first sample. */
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
 * Self-describing frame: "IQB1" | u32 n | u32 fs_hint | u8 fmt(=1) | 3x pad |
 * payload | "IQE1" - read by the --port/--file mode of iq_view_stream.py. */
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

/* ---------------- init + main loop ---------------- */

void app_init(void)
{
  s_rail = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

  RAIL_DataConfig_t dc = {
    .txSource = TX_PACKET_DATA,
    .rxSource = RX_IQDATA_FILTLSB,   /* for strong signals switch to FILTMSB */
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
         "'c' = capture+dump, 's' = status\r\n");
}

void app_process_action(void)
{
  char ch;
  /* Non-blocking read from VCOM. If getchar still blocks, disable
   * blocking in the iostream UART config, or call
   * sl_iostream_uart_set_read_block(sl_iostream_uart_vcom_handle, false);
   * at the end of app_init (with #include "sl_iostream_uart.h" and an
   * extern declaration of the vcom UART handle). */
  if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
    if (ch == 'c' && !capturing) {
      capture_idx  = 0;
      capture_bad  = false;
      capture_done = false;
      capturing    = true;      /* capture starts at the next FIFO event */
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
      printf("# OVERFLOW during burst - discarded, retry\r\n");
    } else {
      dump_capture(0 /* fs_hint: fill in Hz after self-calibration */);
    }
  }
}

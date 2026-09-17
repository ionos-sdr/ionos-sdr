/* SPDX-License-Identifier: MIT
 *
 * ui.h — button + menu state machine on the ESP32-S3
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================== TWO SEPARATE CONCEPTS ==================
 *
 * The menu DELIBERATELY has two concepts, not one:
 *
 *   PAGE          — what is currently on the display. It may change on its
 *                   own: on packet reception it jumps forward, after
 *                   inactivity it returns to the status page.
 *   MODE          — which demodulator is RUNNING. This is STICKY: paging
 *                   over to the status page does not stop the APRS decoder;
 *                   it keeps decoding and keeps gating.
 *
 * If the two were one, the iGate would be stopped every time the IP is
 * checked. That is exactly the kind of silent side effect to be avoided.
 *
 * ================== THE BUTTON ==================
 *
 *   short press   — next page. Reaching a demodulator page TURNS ON that
 *                   mode.
 *   long press    — the demodulator is TURNED OFF (idle) and the display
 *                   returns to the status page. One gesture frees the CPU
 *                   when something else is being measured.
 *
 * On the Heltec V3 this is the PRG button on GPIO0. Strapping pin: do NOT
 * hold it during POWER-UP, or the chip enters download mode. At run time it
 * is a plain input with pull-up, active low.
 *
 * ================== EXTENDING ==================
 *
 * For a new demodulator: add an enum among UI_PAGE_*, a row in the table
 * in ui.cpp, and handle it in two places in main.cpp (feed + drawing).
 * The table is the SINGLE source of truth about what exists — the names,
 * the ready/not-ready state and the order all come from there.
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  UI_PAGE_STATUS = 0,   /* network, IP, ports — the start-up screen */
  UI_PAGE_RF,           /* PHYSICAL parameters: frequency, band, rate */
  UI_PAGE_SDR,          /* stream statistics */
  UI_PAGE_SCAN,         /* dithered spectrum + waterfall (SPECLINE) */
  UI_PAGE_APRS,         /* AFSK1200 iGate */
  UI_PAGE_CW,           /* CW Morse decoder */
  UI_PAGE_WSPR,         /* future */
  UI_PAGE_FT8,          /* future */
  UI_PAGE_COUNT
} ui_page_t;

void ui_init(void);

/* From the main loop, every iteration. Handles the button and the idle
 * return. Non-blocking. */
void ui_tick(uint32_t now_ms);

/* The currently visible page. */
ui_page_t ui_page(void);

/* The RUNNING demodulator. UI_PAGE_STATUS = none. */
ui_page_t ui_mode(void);

/* Whether this demodulator is implemented, or only a placeholder. */
bool ui_page_ready(ui_page_t p);

/* Page name for the header. */
const char *ui_page_name(ui_page_t p);

/* On an external event, jump forward to a page and hold it for hold_ms
 * (e.g. a received APRS packet). Only acts if the given mode is running. */
void ui_flash(ui_page_t p, uint32_t hold_ms);

/* Does the display need a refresh? (a page change happened) */
bool ui_dirty(void);
void ui_clear_dirty(void);

#endif /* UI_H */
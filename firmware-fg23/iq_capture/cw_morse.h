/* SPDX-License-Identifier: MIT
 *
 * cw_morse.h — CW (Morse) transmitter on the FG23 (Simplicity / RAIL)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * OOK (on-off keying) of the carrier: RAIL_StartTxStream(CARRIER_WAVE) /
 * RAIL_StopTxStream. Timing is precise via RAIL_GetTime().
 *
 * Usage (from app.c handle_line):
 *   cw_morse_send("CQ CQ DE HA7DCD K", 18);   // 18 WPM
 *   or the ready-made commands: M1 / M2 / M3 / M <text>
 *
 * The module does NOT depend on the stream state — on start it stops any
 * ongoing RX / other TX, and returns to RX at the end.
 */

#ifndef CW_MORSE_H
#define CW_MORSE_H

#include <stdint.h>
#include <stdbool.h>
#include "rail.h"           /* RAIL_Handle_t, RAIL_Time_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Default speed (PARIS standard). 12–25 WPM is the practical range. */
#ifndef CW_DEFAULT_WPM
#define CW_DEFAULT_WPM  18
#endif

/* The callsign (same as for APRS/WSPR). */
#ifndef CW_MYCALL
#define CW_MYCALL  "HA7DCD"
#endif

/**
 * Send Morse text.
 *
 * @param rail     RAIL handle (s_rail)
 * @param channel  channel (s_channel)
 * @param text     ASCII, A-Z 0-9 and the common punctuation (. , / ? = + -)
 *                 Lower case is converted to upper case automatically.
 *                 Unknown characters are treated as a space.
 * @param wpm      5..40 (out-of-range values are replaced by the default)
 * @return         true = completed, false = aborted with the 'x' key
 *
 * Blocking. An 'x' character arriving on VCOM stops it immediately.
 * At the end the function does StopTxStream + StartRx (instead of
 * restart_rx, so that not everything has to be exported from app.c).
 */
bool cw_morse_send(RAIL_Handle_t rail, uint16_t channel,
                   const char *text, uint8_t wpm);

/* Ready-made test messages (for the terminal M1/M2/M3 commands). */
bool cw_morse_cq(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm);
bool cw_morse_test(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm);
bool cw_morse_beacon(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm);

#ifdef __cplusplus
}
#endif

#endif /* CW_MORSE_H */

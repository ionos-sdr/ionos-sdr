/* SPDX-License-Identifier: MIT
 *
 * cw_morse.h — CW (Morse) adó a FG23-on (Simplicity / RAIL)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * OOK (on-off keying) a vivőn: RAIL_StartTxStream(CARRIER_WAVE) /
 * RAIL_StopTxStream. A timing RAIL_GetTime()-mal pontos.
 *
 * Használat (app.c handle_line-ból):
 *   cw_morse_send("CQ CQ DE HA7DCD K", 18);   // 18 WPM
 *   vagy a kész parancsok: M1 / M2 / M3 / M <szöveg>
 *
 * A modul NEM függ a stream-állapottól — induláskor leállítja az
 * esetleges RX-et / más TX-et, végén visszaáll RX-be.
 */

#ifndef CW_MORSE_H
#define CW_MORSE_H

#include <stdint.h>
#include <stdbool.h>
#include "rail.h"           /* RAIL_Handle_t, RAIL_Time_t */

#ifdef __cplusplus
extern "C" {
#endif

/* Alapértelmezett sebesség (PARIS standard). 12–25 WPM a praktikus. */
#ifndef CW_DEFAULT_WPM
#define CW_DEFAULT_WPM  18
#endif

/* A hívójel (ugyanaz, mint az APRS/WSPR). */
#ifndef CW_MYCALL
#define CW_MYCALL  "HA7DCD"
#endif

/**
 * Morse szöveg adása.
 *
 * @param rail     RAIL handle (s_rail)
 * @param channel  csatorna (s_channel)
 * @param text     ASCII, A-Z 0-9 és a szokásos írásjelek (. , / ? = + -)
 *                 Kisbetű automatikusan nagybetűvé alakul. Ismeretlen
 *                 karaktert szóközként kezelünk.
 * @param wpm      5..40 (kívül eső értéket a default-ra cseréljük)
 * @return         true = végigment, false = 'x' billentyűvel megszakítva
 *
 * Blokkoló. A VCOM-ról érkező 'x' karakter azonnal leállítja.
 * A függvény a végén StopTxStream + StartRx-et csinál (restart_rx
 * helyett, hogy ne kelljen az app.c-ből exportálni mindent).
 */
bool cw_morse_send(RAIL_Handle_t rail, uint16_t channel,
                   const char *text, uint8_t wpm);

/* Kész tesztüzenetek (a terminál M1/M2/M3 parancsaihoz). */
bool cw_morse_cq(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm);
bool cw_morse_test(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm);
bool cw_morse_beacon(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm);

#ifdef __cplusplus
}
#endif

#endif /* CW_MORSE_H */

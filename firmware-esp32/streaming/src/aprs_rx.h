/* SPDX-License-Identifier: MIT
 *
 * aprs_rx.h — AFSK1200 demodulator + AX.25 decoder + APRS-IS uploader
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Builds APRS packets from the I/Q blocks arriving from the FG23 and uploads
 * them to APRS-IS over WiFi. No sound card or external TNC is required — the
 * whole chain runs on the ESP32:
 *
 *   int16 I/Q  ->  FM discr (scaled) -> 9600 Hz (frac)
 *              ->  LibAPRS delay-multiply + LPF + phase window
 *              ->  NRZI -> HDLC (left-shift) -> AX.25 -> APRS-IS
 *   Pair with aprs_rx_v2.cpp (2026-08-08).
 *
 * LOAD: at 50 ksps input roughly 1-2% of one core. The correlator is ~44
 * multiply-accumulates per bit, the atan2 approximation ~15 operations per
 * sample. Negligible for the 240 MHz S3.
 *
 * WHY NO SQUELCH: packet radio traditionally runs with an OPEN squelch.
 * Filtering is the job of the CRC — whatever passes the 16-bit FCS is good,
 * whatever does not is discarded. A squelch would only clip the start of
 * weak packets.
 */

#ifndef APRS_RX_H
#define APRS_RX_H

#include <stdint.h>
#include <stdbool.h>

/* Initialisation. Call AFTER WiFi is up. */
void aprs_rx_init(void);

/* One block of I/Q. The iq array is interleaved int16 (I,Q,I,Q,...), nsamp
 * is the number of COMPLEX samples. sps is the input sample rate — if it
 * changes, the module recomputes its filters automatically. */
void aprs_rx_feed(const int16_t *iq, int nsamp, uint32_t sps);

/* From the main loop. Manages the APRS-IS connection (reconnect,
 * keepalive). Non-blocking. */
void aprs_rx_tick(uint32_t now_ms);

/* Statistics for the display and diagnostics. */
uint32_t aprs_rx_frames(void);     /* valid, CRC-correct frames */
uint32_t aprs_rx_bad(void);        /* CRC-failed frames */
uint32_t aprs_rx_gated(void);      /* packets uploaded to APRS-IS */
bool     aprs_rx_is_online(void);  /* whether the APRS-IS connection is alive */
const char *aprs_rx_last(void);      /* the last frame in TNC2 format */
const char *aprs_rx_last_call(void); /* callsign of the last transmitting station */
const char *aprs_rx_last_info(void); /* info field of the last frame */
uint32_t    aprs_rx_last_ms(void);   /* millis() at the last frame, 0=none yet */

#endif /* APRS_RX_H */

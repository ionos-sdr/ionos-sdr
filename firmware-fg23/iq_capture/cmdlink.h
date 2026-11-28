/* SPDX-License-Identifier: MIT
 *
 * cmdlink.h — second command input on the FG23 (ESP32 -> FG23)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * PURPOSE: when the user tunes from the phone (SDR++ Android, rtl_tcp),
 * the ESP32 sends an "F433775" line here and the FG23 retunes. Without
 * this the chain is one-way: the phone can see but cannot control.
 *
 * ONE WIRE: ESP32 GPIO4 (TX)  ->  FG23 PA06 = EXP 11 (RX). Common GND is
 * already present. No return path is needed — the FG23 talks on VCOM.
 *
 * Incoming lines go to the SAME handle_line() that serves the terminal, so
 * EVERY command is available from the phone as well, not just tuning. One
 * parser, one behaviour — there cannot be two separate truths.
 *
 * NOTE: this module has NOT YET BEEN VERIFIED ON HARDWARE. Until the line
 * is wired and cmdlink_poll() is enabled, the chain works perfectly
 * without it.
 */

#ifndef CMDLINK_H
#define CMDLINK_H

#include <stdbool.h>
#include <stdint.h>

/* Command handler called with each complete incoming line.
 * In app.c this is handle_line(). */
typedef void (*cmdlink_line_fn)(const char *line);

/* Initialisation. From app_init(), after the GPIO clock is enabled. */
void cmdlink_init(cmdlink_line_fn on_line);

/* From the main loop. Non-blocking: reads at most as many characters as
 * are currently in the FIFO. Returns true if a COMPLETE line was processed.
 *
 * IMPORTANT: may also be called while the stream is running — tuning
 * itself stops and restarts the stream when needed. */
bool cmdlink_poll(void);

/* Number of lines received so far, and number of characters lost to
 * framing/parity errors. For diagnostics ('s' command). */
void cmdlink_stats(uint32_t *lines, uint32_t *errors);

/* NBFM audio mode (2026-11-28): after cmdlink_audio_begin() the link carries
 * PCM samples at 8 kHz (fmt 0: int8 @115200, fmt 1: 14-bit pairs @230400)
 * until cmdlink_audio_end(); no lines are parsed meanwhile. Samples come out
 * as int16, full scale +-32767. */
void     cmdlink_audio_begin(int16_t *buf, uint32_t nsamp_pow2, uint8_t fmt);
void     cmdlink_audio_end(void);
uint32_t cmdlink_audio_level(void);   /* samples waiting */
uint32_t cmdlink_audio_total(void);   /* samples received since boot (activity) */
uint32_t cmdlink_audio_ovf(void);     /* samples lost: ring full or broken pair */
int16_t  cmdlink_audio_pop(void);     /* 0 when empty */
void     cmdlink_audio_skip(void);    /* drop one (rate trim) */

#endif /* CMDLINK_H */

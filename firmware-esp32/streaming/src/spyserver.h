/* SPDX-License-Identifier: MIT
 *
 * spyserver.h — SpyServer-compatible server on the ESP32-S3
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================== WHY THIS, AND NOT RTL_TCP ==================
 *
 * rtl_tcp is unsuitable for two reasons:
 *   - 8-BIT. Half of our 16-bit samples would be discarded.
 *   - NO RATE NEGOTIATION. The client assumes fixed RTL rates, the lowest
 *     being 250 ksps — hence the need for integer upsampling, which
 *     produced interpolation images in the spectrum.
 *
 * SpyServer solves both:
 *   - MSG_TYPE_INT16_IQ: NATIVE 16 bit.
 *   - In DeviceInfo the DEVICE announces MaximumSampleRate and the number
 *     of decimation stages, and the client builds its rate list from that.
 *     Advertising 50000 yields 50 kHz in SDR++. No upsampling.
 *
 * Not SDR++-specific: SDR#, SDRangel and the other clients speak it too.
 *
 * ================== STATUS: NOT MEASURED ON HARDWARE ==================
 *
 * Written from the specification, without the possibility of testing. The
 * old rtl_tcp branch remains UNTOUCHED on port 1234 — if this does not come
 * up at first attempt, that is the known-working path.
 *
 * ================== BYTE ORDER ==================
 *
 * The protocol sends the structures RAW, little-endian. The ESP32 is also
 * little-endian, so the structs can be written out directly. Every field is
 * uint32, so there is no padding either.
 */

#ifndef SPYSERVER_H
#define SPYSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "specline.h"

/* The client requested tuning. main.cpp wires this to the command link. */
typedef void (*spy_tune_fn)(uint32_t hz);

/* The client requests / stops / reconfigures scan mode (SETTING_FFT_* +
 * STREAMING_MODE FFT bit). main.cpp forwards it to the FG23 over the
 * command link:
 *   want=true  -> "W<kHz>,<span_kHz>,<nbin>,<floor>,<range>"
 *   want=false -> "W0"  */
typedef void (*spy_scan_fn)(bool want, uint32_t center_hz, uint32_t span_hz,
                            uint16_t nbin, int16_t floor_dbm, uint16_t range_db);

void spy_init(uint16_t port, spy_tune_fn on_tune, spy_scan_fn on_scan);

/* Scan mode: whether the client currently wants rows; forward one SPECLINE. */
bool spy_scan_wanted(void);
void spy_send_specline(const specline_blk_t *line);
uint32_t spy_fft_lines(void);

/* The input I/Q rate. This becomes the advertised MaximumSampleRate. */
void spy_set_rate(uint32_t sps);

/* The current tuning, so that ClientSync reports the correct value. */
void spy_set_freq(uint32_t hz);

/* One block of raw int16 I/Q (interleaved), nsamp complex samples. */
void spy_feed(const int16_t *iq, int nsamp);

/* From the main loop: connection handling, commands, sending. Non-blocking. */
void spy_tick(uint32_t now_ms);

bool     spy_connected(void);
uint32_t spy_out_sps(void);     /* the rate ACTUALLY sent, after decimation */
uint32_t spy_dropped(void);

#endif /* SPYSERVER_H */

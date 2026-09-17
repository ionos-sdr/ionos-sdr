/* SPDX-License-Identifier: MIT
 *
 * iq_stream.h — continuous, decimated I/Q stream over SPI to the ESP32-S3
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * PRINCIPLE (based on the 2026-07-31 measurements):
 *   - The BUFC writes into RAM (datasheet 3.2.6, zero-copy), so samples
 *     are read IN PLACE from the RAIL RX FIFO, and only the RAIL read
 *     pointer is advanced with RAIL_ReadRxFifo(rail, NULL, n). No copy:
 *     11.3% CPU instead of 74.7% at 1 Msps.
 *   - A two-stage CIC decimator brings the rate down.
 *   - Single-pole DC notch at the output rate: otherwise the receiver's
 *     DC offset sits in the middle of the decimated band and, at unity
 *     gain, drives the chain into clipping.
 *   - The output goes over SPI in BLOCKS, with its own header and
 *     SEQUENCE NUMBER.
 *
 * WHY NOT I2S: the CS of the FG23 USART cannot produce a true word
 * select — measured duty cycle 33.6% instead of 50%, in all eight framing
 * combinations. Details in the iq_stream.c header.
 *
 * OUTPUT FORMAT: 16-byte header + 256 complex samples, interleaved int16
 * little-endian, I FIRST. Clipping is at +-32767, NEVER -32768.
 */

#ifndef IQ_STREAM_H
#define IQ_STREAM_H

#include "rail.h"
#include <stdbool.h>
#include <stdint.h>

/* One-time setup from app_init(): the RAIL RX FIFO buffer is passed so
 * that it can be read in place. */
void iq_stream_init(const uint8_t *fifo_base, uint16_t fifo_bytes);

/* Whether the stream is currently running. */
bool iq_stream_active(void);

/* Fine offset of the synth in ticks (1 tick = 4.6492 Hz on the FG23).
 *
 * WHY THIS MODULE NEEDS IT: the module restarts RX in several places
 * (start, stop, and recovery after a FIFO overflow), and RAIL_StartRx does
 * NOT preserve the frequency offset. Without this, starting the stream
 * discarded the crystal calibration AND the tuning set with the 'F'
 * command — the unit silently received ~1.9 kHz off, and 'F' had no
 * effect while the stream was running. app.c calls it on every tune. */
void iq_stream_set_freq_tick(int32_t tick);

/* To be called from sl_rail_util_on_event(). True = handled, the app.c
 * capture branch must be skipped. */
bool iq_stream_on_event(RAIL_Handle_t rail, RAIL_Events_t events);

/* Start. decim = TOTAL decimation, 8..4096 (rounded down to a multiple
 * of 8). out_shift: extra gain as a power of two (0 = none). */
void iq_stream_start(RAIL_Handle_t rail, uint16_t channel,
                     uint16_t decim, uint8_t out_shift);

/* Stop + statistics (clipping, dropped blocks, contents of the last block). */
void iq_stream_stop(RAIL_Handle_t rail, uint16_t channel);

/* Must be called from the main loop: this is the SPI block-sender state
 * machine. Returns true if the stream is active (the main loop should do
 * nothing else). */
bool iq_stream_pump(void);

/* The ACTUAL output sample rate for a given decimation, based on the
 * IQ_STREAM_FS_IN_HZ set in the module. A function so that the input rate
 * is defined in ONE place — otherwise the printed and the actual value
 * drift apart. */
uint32_t iq_stream_out_sps(uint16_t decim);

/* Pin test ('g'): toggles the three lines as plain GPIO at separate
 * frequencies (CS 1 kHz, SCLK 2 kHz, MOSI 4 kHz).
 *
 * TWO USES: it checks the physical wiring without the peripheral, and all
 * three pins toggle at EXACTLY 50%, so it is also a DUTY-CYCLE REFERENCE:
 * with 3.3 V logic the multimeter DC average must be 1.65 V on all three.
 * This measurement is what caught the I2S word select fault. */
void iq_stream_pin_test(uint32_t seconds);

/* Clock and route diagnostics (VCOM EUSART + SPI USART) ('v'). */
void iq_stream_dump_uart_cfg(void);


/* ---- EXT MODE (scan.c): send a foreign 1040-byte block over SPI without
 * a RAIL RX stream. Mutually exclusive with the normal stream
 * (iq_stream_active() must be false). Block format: specline.h. ---- */
void iq_stream_ext_begin(void);            /* SPI/LDMA up, CS inactive */
void iq_stream_ext_end(void);              /* SPI down, CS stays driven high */
bool iq_stream_ext_busy(void);             /* whether a block is in flight */
void iq_stream_ext_pump(void);             /* completes the send (CS up) */
bool iq_stream_ext_send(const void *blk);  /* true = started; false = retry */

#endif /* IQ_STREAM_H */

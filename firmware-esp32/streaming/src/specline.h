/* SPDX-License-Identifier: MIT
 *
 * specline.h — SHARED block format of the wideband-scan spectrum line
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * THE SAME FILE exists in both the FG23 (Simplicity) and the ESP32
 * (PlatformIO) project. Any change must be applied to BOTH.
 *
 * PRINCIPLE: a SPECLINE is a 1040-byte block, EXACTLY the same size and
 * framed the same way (CS frame, RDY, LDMA) as the IQB2 I/Q block. Thus the
 * ESP32 SPI-slave DMA, the reorder buffer and the RDY handshake work
 * UNCHANGED — only the magic differs, and process() branches on the magic.
 *
 * Header (16 B, the iq_blk_hdr_t fields at the SAME offsets, different meaning):
 *   magic     'S','P','C','1'  (0x31435053 LE)
 *   seq       block sequence number (written by spi_send_block, as for IQ)
 *   nbin      number of bins in the payload (<= SPECLINE_MAX_BINS)
 *   flags     bit0 = last chunk; bit8..15 = chunk index (currently 0)
 *   f_center  CENTER FREQUENCY of the scan in Hz
 * Payload (1024 B):
 *   span_hz   u32   total width of the scan
 *   floor_dbm i16   value of bin level 0 in dBm (e.g. -130)
 *   range_db  u16   bin level 255 = floor + range (e.g. 100)
 *   bins[SPECLINE_MAX_BINS] u8   dB = floor + v*range/255
 */

#ifndef SPECLINE_H
#define SPECLINE_H

#include <stdint.h>

#define SPECLINE_MAGIC     0x31435053u        /* 'S','P','C','1' */
#define SPECLINE_BLK_BYTES 1040u
#define SPECLINE_HDR_BYTES 16u
#define SPECLINE_MAX_BINS  (SPECLINE_BLK_BYTES - SPECLINE_HDR_BYTES - 8u)  /* 1016 */
#define SPECLINE_FLAG_LAST 0x0001u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint16_t nbin;
  uint16_t flags;
  uint32_t f_center_hz;
} specline_hdr_t;

typedef struct __attribute__((packed)) {
  specline_hdr_t hdr;
  uint32_t span_hz;
  int16_t  floor_dbm;
  uint16_t range_db;
  uint8_t  bins[SPECLINE_MAX_BINS];
} specline_blk_t;

/* Compile-time check: the block is exactly the same size as IQB2. */
typedef char specline_size_check[(sizeof(specline_blk_t) == SPECLINE_BLK_BYTES) ? 1 : -1];

#endif /* SPECLINE_H */

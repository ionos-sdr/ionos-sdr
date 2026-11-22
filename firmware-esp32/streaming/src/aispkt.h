/* SPDX-License-Identifier: MIT
 *
 * aispkt.h — AIS frames decoded ON THE FG23, sent to the ESP32 over SPI
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * THE SAME FILE is in the FG23 (Simplicity) and the ESP32 (PlatformIO)
 * project. If it changes, replace it in BOTH.
 *
 * Like SPECLINE (specline.h): a 1040-byte block framed exactly like the
 * IQB2 I/Q block (CS frame, RDY, LDMA), so the ESP32's SPI slave DMA,
 * reorder buffer and RDY handshake work unchanged; process() branches on
 * the magic.
 *
 * Header (16 B, the iq_blk_hdr_t fields in the SAME place):
 *   magic        'A','I','S','1'  (0x31534941 LE)
 *   seq          block sequence (written by spi_send_block)
 *   nfrm         frames in this block (0 = heartbeat, stats only)
 *   flags        bit0 = last block of a burst (always 1 now)
 *   f_center_hz  the channel the PHY listens on (161975000 / 162025000)
 * Payload (1024 B): aispkt_stats_t, then nfrm x aispkt_frm_t.
 */

#ifndef AISPKT_H
#define AISPKT_H

#include <stdint.h>

#define AISPKT_MAGIC       0x31534941u        /* 'A','I','S','1' */
#define AISPKT_MAX_FRAMES  6u
#define AISPKT_MAX_LEN     128u               /* frame bytes incl. FCS (5-slot msg = 128) */
#define AISPKT_FLAG_LAST   0x0001u

typedef struct {
  uint32_t magic;
  uint32_t seq;
  uint16_t nfrm;
  uint16_t flags;
  uint32_t f_center_hz;
} aispkt_hdr_t;                                /* 16 B */

typedef struct {
  uint32_t n_sync;        /* sync word detections (both polarities) */
  uint32_t n_ok;          /* CRC-correct frames */
  uint32_t n_crc_bad;     /* closing flag found, CRC wrong */
  uint32_t n_abort;       /* 7 ones / too long / ran off the fixed length */
  uint32_t n_drop;        /* good frames lost: queue full */
  uint32_t n_ovf;         /* RX FIFO overflows */
  uint32_t uptime_ms;
  int16_t  noise_qdbm;    /* idle-channel RSSI (quarter dBm), RAIL_RSSI_INVALID = n/a */
  uint16_t rsv;
} aispkt_stats_t;                              /* 32 B */

typedef struct {
  uint32_t t_us;          /* RAIL time at the sync word */
  int16_t  rssi_qdbm;     /* RAIL RSSI at the sync word, quarter dBm */
  int16_t  foff_hz;       /* RAIL frequency offset estimate, Hz (INT16_MIN = n/a) */
  uint8_t  chan;          /* 'A' / 'B' */
  uint8_t  len;           /* frame bytes incl. the 2 FCS bytes */
  uint8_t  pol;           /* 1 = sync word 1 matched, 2 = sync word 2 */
  uint8_t  rsv;
  uint32_t rsv2;
  uint8_t  data[AISPKT_MAX_LEN];               /* bytes as ais_rx.c builds them (first bit -> LSB) */
} aispkt_frm_t;                                /* 144 B */

typedef struct {
  aispkt_hdr_t   hdr;
  aispkt_stats_t st;
  aispkt_frm_t   frm[AISPKT_MAX_FRAMES];       /* 864 B */
  uint8_t        pad[1024 - sizeof(aispkt_stats_t) - AISPKT_MAX_FRAMES * sizeof(aispkt_frm_t)];
} aispkt_blk_t;                                /* 1040 B */

#ifdef __cplusplus
static_assert(sizeof(aispkt_blk_t) == 1040, "aispkt_blk_t must be 1040 B");
#else
_Static_assert(sizeof(aispkt_blk_t) == 1040, "aispkt_blk_t must be 1040 B");
#endif

#endif /* AISPKT_H */

/* SPDX-License-Identifier: MIT
 *
 * specline.h — a szelessavu scan spektrumsoranak KOZOS blokkformatuma
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * UGYANEZ A FAJL van a FG23 (Simplicity) es az ESP32 (PlatformIO) projektben.
 * Ha valtozik, MINDKETTOBEN cserelni kell.
 *
 * ELV: a SPECLINE egy 1040 bajtos blokk, PONTOSAN akkora es ugyanugy
 * keretezett (CS-keret, RDY, LDMA), mint az IQB2 I/Q-blokk. Igy az ESP32
 * SPI-slave DMA-ja, az ujrarendezo puffer es a RDY-kezfogas VALTOZATLANUL
 * mukodik — csak a magic mas, es a process() a magic alapjan agaztat.
 *
 * Fejlec (16 B, az iq_blk_hdr_t mezoi UGYANOTT, mas jelentessel):
 *   magic     'S','P','C','1'  (0x31435053 LE)
 *   seq       blokk-sorszam (a spi_send_block irja, mint az IQ-nal)
 *   nbin      binek szama a payloadban (<= SPECLINE_MAX_BINS)
 *   flags     bit0 = utolso darab (chunk); bit8..15 = darab-index (most 0)
 *   f_center  a scan KOZEPFREKVENCIAJA Hz-ben
 * Payload (1024 B):
 *   span_hz   u32   a scan teljes szelessege
 *   floor_dbm i16   a 0-as bin ertek dBm-ben (pl. -130)
 *   range_db  u16   a 255-os bin = floor + range (pl. 100)
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

/* Fordítási idejű ellenőrzés: a blokk pontosan akkora, mint az IQB2. */
typedef char specline_size_check[(sizeof(specline_blk_t) == SPECLINE_BLK_BYTES) ? 1 : -1];

#endif /* SPECLINE_H */

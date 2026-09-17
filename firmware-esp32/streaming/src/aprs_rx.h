/* SPDX-License-Identifier: MIT
 *
 * aprs_rx.h — AFSK1200 demodulator + AX.25 dekoder + APRS-IS feltolto
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * A FG23-tol erkezo I/Q blokkokbol csinal APRS-csomagot, es felkuldi az
 * APRS-IS-re WiFin. Nem kell hozza hangkartya, sem kulso TNC — a teljes
 * lanc az ESP32-ben fut:
 *
 *   int16 I/Q  ->  FM discr (skalazott) -> 9600 Hz (frac)
 *              ->  LibAPRS delay-multiply + LPF + fazisablak
 *              ->  NRZI -> HDLC (left-shift) -> AX.25 -> APRS-IS
 *   Pair with aprs_rx_v2.cpp (2026-08-08).
 *
 * TERHELES: 50 ksps bemeneten nagysagrendileg 1-2% egy magbol. A
 * korrelator bitenkent ~44 szorzas-osszeadas, az atan2 kozelites ~15
 * muvelet mintankent. A 240 MHz-es S3-nak ez nem meres.
 *
 * MIERT NINCS SQUELCH: a csomagradio hagyomanyosan NYITOTT zajzarral megy.
 * A szures a CRC dolga — ami atmegy a 16 bites FCS-en, az jo, ami nem, azt
 * eldobjuk. Egy zajzar csak a gyenge csomagok elejet vagna le.
 */

#ifndef APRS_RX_H
#define APRS_RX_H

#include <stdint.h>
#include <stdbool.h>

/* Inicializalas. A WiFi felallasa UTAN hivd. */
void aprs_rx_init(void);

/* Egy blokknyi I/Q. Az iq tomb interleaved int16 (I,Q,I,Q,...), nsamp
 * KOMPLEX minta. Az sps a bemeneti mintavetel — ha valtozik, a modul
 * magatol ujraszamolja a szuroit. */
void aprs_rx_feed(const int16_t *iq, int nsamp, uint32_t sps);

/* A fo ciklusbol. Kezeli az APRS-IS kapcsolatot (ujracsatlakozas,
 * keepalive). Nem blokkol. */
void aprs_rx_tick(uint32_t now_ms);

/* Statisztika a kijelzohoz es a diagnosztikahoz. */
uint32_t aprs_rx_frames(void);     /* ervenyes, CRC-helyes keretek */
uint32_t aprs_rx_bad(void);        /* CRC-hibas keretek */
uint32_t aprs_rx_gated(void);      /* APRS-IS-re felkuldott csomagok */
bool     aprs_rx_is_online(void);  /* el-e az APRS-IS kapcsolat */
const char *aprs_rx_last(void);      /* az utolso keret TNC2-ben */
const char *aprs_rx_last_call(void); /* az utolso adoallomas hivojele */
const char *aprs_rx_last_info(void); /* az utolso keret info-mezoje */
uint32_t    aprs_rx_last_ms(void);   /* millis() az utolso keretnel, 0=meg nincs */

#endif /* APRS_RX_H */

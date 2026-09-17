/* SPDX-License-Identifier: MIT
 *
 * iq_stream.h — folyamatos, decimalt I/Q stream SPI-n az ESP32-S3-nak
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * ELV (a 2026-07-31-i meresek alapjan):
 *   - A BUFC RAM-ba ir (datasheet 3.2.6, zero-copy), ezert a mintakat
 *     HELYBEN olvassuk a RAIL RX FIFO-jabol, es csak a RAIL olvaso-
 *     mutatojat leptetjuk RAIL_ReadRxFifo(rail, NULL, n)-nel. Igy nincs
 *     masolas: 1 Msps-en 74.7% helyett 11.3% CPU.
 *   - Ketfokozatu CIC decimator viszi le a ratat.
 *   - Egypolusu DC-notch a kimeneti ratan: a vevo DC-offszetje kulonben a
 *     decimalt sav kozepen ul, es unity gainnel racsra viszi a lancot.
 *   - A kimenet SPI-n megy, BLOKKOKBAN, sajat fejleccel es SORSZAMMAL.
 *
 * MIERT NEM I2S: a FG23 USART-janak CS-e nem tud valodi word selectet
 * adni — mert kitoltes 33.6% az 50% helyett, mind a nyolc keretezesi
 * kombinacioban. Reszletek az iq_stream.c fejleceben.
 *
 * KIMENETI FORMATUM: 16 bajt fejlec + 256 komplex minta, interleaved
 * int16 little-endian, I ELOL. A levagas +-32767, SOHA nem -32768.
 */

#ifndef IQ_STREAM_H
#define IQ_STREAM_H

#include "rail.h"
#include <stdbool.h>
#include <stdint.h>

/* Egyszeri beallitas app_init()-bol: a RAIL RX FIFO bufferet adjuk at,
 * hogy helyben tudjunk belole olvasni. */
void iq_stream_init(const uint8_t *fifo_base, uint16_t fifo_bytes);

/* Fut-e eppen a stream. */
bool iq_stream_active(void);

/* A synth finom-offszete tickben (1 tick = 4.6492 Hz a FG23-on).
 *
 * MIERT KELL EZ A MODULNAK: a modul tobb helyen is ujrainditja az RX-et
 * (start, stop, es FIFO-overflow utani helyreallitas), a RAIL_StartRx
 * pedig NEM orzi meg a frekvencia-offszetet. Enelkul a stream inditasa
 * eldobta a kristalykalibraciot ES az 'F' paranccsal beallitott hangolast
 * — a doboz csendben ~1.9 kHz-cel melle vett, es az 'F' hatastalan volt
 * futo stream mellett. Az app.c minden hangolasnal hivja. */
void iq_stream_set_freq_tick(int32_t tick);

/* Az sl_rail_util_on_event()-bol hivando. True = kezeltuk, az app.c
 * capture-aga maradjon ki. */
bool iq_stream_on_event(RAIL_Handle_t rail, RAIL_Events_t events);

/* Inditas. decim = TELJES decimacio, 8..4096 (8-ra kerekitve lefele).
 * out_shift: extra erosites kettohatvanyban (0 = nincs). */
void iq_stream_start(RAIL_Handle_t rail, uint16_t channel,
                     uint16_t decim, uint8_t out_shift);

/* Leallitas + statisztika (levagas, blokk-eldobas, utolso blokk tartalma). */
void iq_stream_stop(RAIL_Handle_t rail, uint16_t channel);

/* A fo ciklusbol kell hivni: ez az SPI blokk-kuldo allapotgep.
 * Visszaad: true, ha a stream aktiv (a fo ciklus ne csinaljon mast). */
bool iq_stream_pump(void);

/* A VALODI kimeneti mintavetel adott decimacional, a modulban beallitott
 * IQ_STREAM_FS_IN_HZ alapjan. Azert fuggveny, hogy a bemeneti rata EGY
 * helyen legyen definialva — kulonben a kiirt es a tenyleges ertek
 * elcsuszik egymastol. */
uint32_t iq_stream_out_sps(uint16_t decim);

/* Lab-teszt ('g'): a harom vonalat kulon frekvencian billegteti sima
 * GPIO-kent (CS 1 kHz, SCLK 2 kHz, MOSI 4 kHz).
 *
 * KETTOS HASZNA: egyreszt a fizikai bekotest ellenorzi periferia nelkul,
 * masreszt mindharom lab PONTOSAN 50%-on billeg, tehat KITOLTES-
 * REFERENCIA is: 3.3 V-os logikanal a multimeter DC atlaga 1.65 V kell
 * legyen mindharmon. Ez a meres fogta meg az I2S word select hibajat. */
void iq_stream_pin_test(uint32_t seconds);

/* Orajel- es route-diagnosztika (VCOM EUSART + SPI USART) ('v'). */
void iq_stream_dump_uart_cfg(void);


/* ---- EXT-MOD (scan.c): idegen 1040 bajtos blokk kuldese az SPI-n, RAIL
 * RX-stream nelkul. Kizarja a normal streamet (iq_stream_active() false
 * kell legyen). A blokk formatuma: specline.h. ---- */
void iq_stream_ext_begin(void);            /* SPI/LDMA felall, CS inaktiv */
void iq_stream_ext_end(void);              /* SPI le, CS marad hajtott magas */
bool iq_stream_ext_busy(void);             /* megy-e epp egy blokk */
void iq_stream_ext_pump(void);             /* a kuldes lezarasa (CS fel) */
bool iq_stream_ext_send(const void *blk);  /* true = elindult; false = ujra */

#endif /* IQ_STREAM_H */

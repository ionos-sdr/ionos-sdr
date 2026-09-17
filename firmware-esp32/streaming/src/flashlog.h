/* SPDX-License-Identifier: MIT
 *
 * flashlog.h — tartos (LittleFS) telemetria-naplo terepi hibakereseshez
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * MIRE VALO
 * =========
 * USB NELKUL (powerbankrol) is gyujti a flashbe az APRS-esemenyeket es a
 * periodikus egeszseg-pillanatkepeket, hogy az intermittens hibakat is
 * elkapd — amit a pillanatnyi OLED/soros nem mutat. Kesobb USB-re dugva
 * kiolvasod.
 *
 * FONTOS (ESP32 flash-cache): flash irasa/torlesekor a cache kikapcsol, es
 * a nem-IRAM fo hurok par tiz ms-ra megallhat MINDKET magon -> ez a
 * blokkvesztes forrasa lehet. Ezert:
 *   - a naplo RAM-ba pufferel (flashlog_printf OLCSO, csak masol),
 *   - flashbe csak RITKAN es kotegelve ir (flashlog_tick),
 *   - a flush idejet MAGA is naplozza ("FLUSH Nms"), igy latszik a stall.
 *
 * MERT 2026-08-04 (VSG60A-s meropad): minden FLUSH utan azonnal ugrik a
 * blokkvesztes, es a dt_max 23 ms-rol 35..140 ms-ra no. PRECIZ MERESHEZ
 * EZERT KAPCSOLD KI -> FLASHLOG_ENABLE 0.
 *
 * SOROS PARANCSOK (a DIAG=Serial0 konzolon, sorvegjellel):
 *   logdump   — a teljes naplo kiirasa
 *   logclear  — naplo torlese
 *   logstat   — meret + eldobott sorok
 */

#ifndef FLASHLOG_H
#define FLASHLOG_H

/* ====================================================================
 *  EGYETLEN KAPCSOLO
 *
 *    1 = normal uzem (terepi telemetria flashbe)
 *    0 = TELJESEN KI: a modul nem fordul bele, es MINDEN hivasi hely
 *        (flashlog_init / _tick / _printf / _dump / _clear / _size)
 *        no-op makrora cserelodik. Nem kell #if-eket szorni a
 *        main.cpp-be vagy az aprs_rx.cpp-be — eleg ez az egy sor.
 *        Meres/hibakereses alatt EZT hasznald.
 * ==================================================================== */
#define FLASHLOG_ENABLE 0

#include <stdint.h>
#include <stddef.h>

class Stream;

#if FLASHLOG_ENABLE

/* LittleFS mount + boot-jelzo. A WiFi/APRS felallasa elott vagy utan is hivhato. */
void   flashlog_init(void);

/* Egy idobelyeges sor a RAM-pufferbe (millis() prefixszel). NEM ir flashbe. */
void   flashlog_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* A fo ciklusbol: periodikus kotegelt flush + soros parancsok kezelese. */
void   flashlog_tick(uint32_t now_ms);

/* ====================================================================
 *  FLUSH-ABLAK — a flash-iras IDOZITESE (2026-08-07)
 *
 *  A flashbe iras kikapcsolja a flash-cache-t, ami MINDKET magon
 *  megallitja a flashben futo kodot: a fo ciklust ES a dedikalt
 *  SPI-veteli taskot is. Ilyenkor mar csak a felfuzott DMA-tranzakciok
 *  fogynak — NQUEUE=24 mellett 123 ms-ig.
 *
 *  Az ejszakai meres (2026-08-06/07, 4,55 h, 16 599 keret) szerint a
 *  flush 88 ms hosszu (median; p90 98, max 167), es 55,5 masodpercenkent
 *  jott: a FLUSH_HI=3072 bajtos kuszob a mereskori ~55 B/s naplorata
 *  mellett pont ennyi ido alatt telt meg. Az elveszett keretek 70 %-a
 *  ilyen akadasra esett, CRC-hiba NELKUL — vagyis a keret el sem jutott
 *  a dekoderig, mert lyuk volt a mintafolyamban.
 *
 *  A javitas nem a naplo kikapcsolasa, hanem az IDOZITESE: a fo ciklus
 *  minden korben megmondja, hogy epp "biztonsagos ablak" van-e — azaz
 *  kiurult-e a vetel-gyuru. Ha igen, akkor mind a NQUEUE tranzakcio fel
 *  van fuzve, tehat a teljes 123 ms tartalek megvan, es a 88 ms-os iras
 *  belefer. Ha nem, a flush VAR.
 *
 *  Elakadas ellen a flashlog.cpp-ben hataridok vannak: ha az ablak sokaig
 *  nem nyilik ki, akkor is kiir. A naplot sosem veszitjuk el.
 *
 *  Aki nem hivja, a regi viselkedest kapja (mindig szabad) — visszafele
 *  kompatibilis.
 * ==================================================================== */
void   flashlog_flush_window(bool safe_now);

/* A teljes naplo (regi + aktualis) kiirasa egy folyamra (pl. Serial0). */
void   flashlog_dump(Stream &out);

/* Naplo torlese (mindket fajl) es ujraindulas. */
void   flashlog_clear(void);

/* Az eddig flashben tarolt naplo merete bajtban (a RAM-puffer nelkul). */
size_t flashlog_size(void);

#else   /* ---------- FLASHLOG_ENABLE == 0: minden hivas eltunik ---------- */

/* Makrok (nem ures fuggvenyek), hogy az argumentumok se ertekelodjenek ki:
 * egy flashlog_printf("... %lu", draga_fuggveny()) hivasban a draga_fuggveny()
 * sem fut le. A (void) castok elkerulik a "nem hasznalt ertek" figyelmezteteseket. */
#define flashlog_init()       ((void)0)
#define flashlog_printf(...)  ((void)0)
#define flashlog_tick(now)    ((void)(now))
#define flashlog_flush_window(s) ((void)(s))
#define flashlog_dump(out)    ((void)0)
#define flashlog_clear()      ((void)0)
#define flashlog_size()       ((size_t)0)

#endif  /* FLASHLOG_ENABLE */

#endif /* FLASHLOG_H */
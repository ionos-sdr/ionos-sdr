/* SPDX-License-Identifier: MIT
 *
 * cmdlink.h — masodik parancsbemenet a FG23-on (ESP32 -> FG23)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * MIRE VALO: amikor a telefonrol (SDR++ Android, rtl_tcp) atteker a
 * frekvencian, az ESP32 egy "F433775" sort kuld ide, es a FG23 athangol.
 * Enelkul a lanc egyiranyu: a telefon lat, de nem tud vezerelni.
 *
 * EGY DROT: ESP32 GPIO4 (TX)  ->  FG23 PA06 = EXP 11 (RX). Kozos GND mar
 * van. Visszairany nem kell — a FG23 a VCOM-on beszel.
 *
 * A bejovo sorokat UGYANARRA a handle_line()-ra adjuk, ami a terminalt is
 * kiszolgalja, tehat MINDEN parancs elerheto a telefonrol is, nem csak a
 * hangolas. Egy parser, egy viselkedes — nem lehet ket kulon igazsag.
 *
 * FIGYELEM: ez a modul MEG NINCS HARDVEREN LEMERVE. A vonal bekotese es a
 * cmdlink_poll() bekapcsolasa elott a lanc tokeletesen mukodik nelkule is.
 */

#ifndef CMDLINK_H
#define CMDLINK_H

#include <stdbool.h>
#include <stdint.h>

/* A parancs-feldolgozo, amit a beerkezo teljes sorra hivunk.
 * Az app.c-ben ez a handle_line(). */
typedef void (*cmdlink_line_fn)(const char *line);

/* Inicializalas. app_init()-bol, a GPIO ora bekapcsolasa utan. */
void cmdlink_init(cmdlink_line_fn on_line);

/* A fo ciklusbol. Nem blokkol: legfeljebb annyi karaktert olvas, amennyi
 * epp a FIFO-ban van. Visszaad: true, ha egy TELJES sort feldolgozott.
 *
 * FONTOS: a stream futasa alatt is szabad hivni — a hangolas maga allitja
 * le es inditja ujra a streamet, ha kell. */
bool cmdlink_poll(void);

/* Hany sort kaptunk eddig, es hany karakter veszett el keret-/paritashiba
 * miatt. Diagnosztikahoz ('s' parancs). */
void cmdlink_stats(uint32_t *lines, uint32_t *errors);

#endif /* CMDLINK_H */

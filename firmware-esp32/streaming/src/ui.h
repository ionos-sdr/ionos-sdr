/* SPDX-License-Identifier: MIT
 *
 * ui.h — gomb + menu-allapotgep az ESP32-S3-on
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================== KET KULON DOLOG ==================
 *
 * A menuben SZANDEKOSAN ket fogalom van, nem egy:
 *
 *   OLDAL (page)  — ami epp a kijelzon van. Ez valtozhat magatol is:
 *                   csomagvetelkor elore ugrik, tetlenseg utan visszater a
 *                   statuszra.
 *   UZEMMOD (mode)— melyik demodulator FUT. Ez RAGADOS: attol, hogy
 *                   atlapozol a statuszra, az APRS-dekoder tovabb dolgozik
 *                   es tovabb gatel.
 *
 * Ha a ketto egy lenne, akkor minden alkalommal leallitanad az iGate-et,
 * amikor megnezed az IP-t. Ez pontosan az a fajta csendes mellekhatas,
 * amit el akarunk kerulni.
 *
 * ================== A GOMB ==================
 *
 *   rovid nyomas  — kovetkezo oldal. Ha demod-oldalra ersz, az az
 *                   uzemmod BEKAPCSOL.
 *   hosszu nyomas — a demodulator KIKAPCSOL (uresjarat), es vissza a
 *                   statuszra. Igy egy mozdulattal fel tudod szabaditani a
 *                   CPU-t, ha valami mast mersz.
 *
 * A Heltec V3-on ez a PRG gomb a GPIO0-n. Strapping lab: BEKAPCSOLASKOR ne
 * tartsd nyomva, mert akkor letoltesi modba megy a chip. Futas kozben sima
 * bemenet, felhuzassal, aktiv alacsony.
 *
 * ================== BOVITES ==================
 *
 * Uj demodulatorhoz: vegy fel egy enumot a UI_PAGE_* koze, egy sort a
 * ui.cpp tablazataba, es a main.cpp-ben ket helyen kezeld (feed + rajzolas).
 * A tablazat a EGYETLEN igazsag arrol, mi letezik — a nevek, a kesz/nem-kesz
 * allapot es a sorrend is onnan jon.
 */

#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  UI_PAGE_STATUS = 0,   /* halozat, IP, portok — ez az indulo kep */
  UI_PAGE_RF,           /* FIZIKAI parameterek: frekvencia, sav, rata */
  UI_PAGE_SDR,          /* a stream statisztikaja */
  UI_PAGE_SCAN,         /* ditherelt spektrum + waterfall (SPECLINE) */
  UI_PAGE_APRS,         /* AFSK1200 iGate */
  UI_PAGE_CW,           /* CW Morse dekoder */
  UI_PAGE_WSPR,         /* jovo */
  UI_PAGE_FT8,          /* jovo */
  UI_PAGE_COUNT
} ui_page_t;

void ui_init(void);

/* A fo ciklusbol, minden korben. Kezeli a gombot es a tetlensegi
 * visszatereset. Nem blokkol. */
void ui_tick(uint32_t now_ms);

/* Az epp lathato oldal. */
ui_page_t ui_page(void);

/* A FUTO demodulator. UI_PAGE_STATUS = egyik sem. */
ui_page_t ui_mode(void);

/* Kesz-e mar ez a demodulator, vagy csak hely van fenntartva neki. */
bool ui_page_ready(ui_page_t p);

/* Az oldal neve a fejlechez. */
const char *ui_page_name(ui_page_t p);

/* Kulso esemeny hatasara elore ugrik egy oldalra, es ott tartja
 * hold_ms ideig (pl. beerkezett APRS-csomag). Csak akkor lep, ha az adott
 * uzemmod fut. */
void ui_flash(ui_page_t p, uint32_t hold_ms);

/* Frissiteni kell-e a kijelzot? (oldalvaltas tortent) */
bool ui_dirty(void);
void ui_clear_dirty(void);

#endif /* UI_H */
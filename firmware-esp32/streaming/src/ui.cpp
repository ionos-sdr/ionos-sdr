/* SPDX-License-Identifier: MIT
 *
 * ui.cpp — gomb + menu-allapotgep az ESP32-S3-on
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * CW oldal READY. SCAN oldal = nezet (nem demod).
 * A tablazat az EGYETLEN igazsag.
 */

#include "ui.h"
#include <Arduino.h>

/* ==================== KONFIG ==================== */

#define UI_BTN_PIN        0           /* Heltec V3 PRG gomb (GPIO0) */
#define UI_DEBOUNCE_MS    30
#define UI_LONG_MS        800
#define UI_IDLE_BACK_MS   20000       /* tetlenseg utan vissza STATUS-ra */

#define UI_DEFAULT_MODE   UI_PAGE_APRS

/* ==================== A MENU TABLAZATA ====================
 * Ez az EGYETLEN hely, ahol az oldalak leteznek. Uj demodulatornal ide
 * kerul egy sor, es a sorrend is innen jon. */
typedef struct {
  const char *name;
  bool        is_demod;   /* uzemmodot kapcsol-e, vagy csak nezet */
  bool        ready;      /* meg van-e irva */
} ui_entry_t;

static const ui_entry_t UI_TABLE[UI_PAGE_COUNT] = {
  /* név          demod   kesz */
  { "STATUS",     false,  true  },
  { "RF",         false,  true  },
  { "SDR STREAM", false,  true  },
  { "SCAN",       false,  true  },   /* ditherelt waterfall — nezet */
  { "APRS iGATE", true,   true  },
  { "CW",         true,   true  },
  { "WSPR",       true,   false },
  { "FT8",        true,   false },
};

/* ==================== ALLAPOT ==================== */

static ui_page_t s_page = UI_PAGE_STATUS;
static ui_page_t s_mode = UI_PAGE_STATUS;
static bool      s_dirty = true;

static bool      s_btn_stable = false;   /* true = nyomva */
static bool      s_btn_raw = false;
static uint32_t  s_btn_change_ms = 0;
static uint32_t  s_btn_down_ms = 0;
static bool      s_long_fired = false;

static uint32_t  s_last_activity_ms = 0;
static uint32_t  s_hold_until_ms = 0;

/* KITUZOTT APRS/CW-lap: ha SAJAT KEZZEL lapoztal a demod-lapra, az nem
 * ugrik vissza a statuszra sem tetlenseg (20 s), sem egy beerkezo jel
 * villanasa miatt — nyugodtan nezheted, amig at nem lapozol. */
static bool      s_demod_pinned = false;

/* ==================== SEGEDEK ==================== */

static void ui_set_page(ui_page_t p, uint32_t now)
{
  if (p != s_page) {
    s_page = p;
    s_dirty = true;
  }
  s_last_activity_ms = now;
}

/* Kovetkezo oldal. A meg nem kesz demodokat NEM ugorjuk at: latszodjanak,
 * hogy tudd, mi van tervben — csak megmondjak magukrol, hogy meg nincs
 * meg, es nem kapcsolnak uzemmodot. */
static void ui_next(uint32_t now)
{
  ui_page_t p = (ui_page_t)((s_page + 1) % UI_PAGE_COUNT);
  ui_set_page(p, now);

  if (UI_TABLE[p].is_demod && UI_TABLE[p].ready) {
    if (s_mode != p) {
      s_mode = p;
      Serial0.printf("\nUI: uzemmod -> %s\n", UI_TABLE[p].name);
    }
  }
  s_hold_until_ms = 0;

  /* Kituzes CSAK akkor, ha te magad lapoztal egy kesz demod-lapra. */
  s_demod_pinned = (UI_TABLE[p].is_demod && UI_TABLE[p].ready);
}

static void ui_stop_demod(uint32_t now)
{
  if (s_mode != UI_PAGE_STATUS) {
    Serial0.printf("\nUI: %s leallitva\n", UI_TABLE[s_mode].name);
    s_mode = UI_PAGE_STATUS;
  }
  s_demod_pinned = false;
  ui_set_page(UI_PAGE_STATUS, now);
}

/* ==================== NYILVANOS API ==================== */

void ui_init(void)
{
  /* Felhuzassal: a gomb a GND fele huz. A GPIO0 strapping lab, de futas
   * kozben sima bemenet. */
  pinMode(UI_BTN_PIN, INPUT_PULLUP);
  s_btn_raw = s_btn_stable = false;
  s_page = UI_PAGE_STATUS;
  s_mode = UI_DEFAULT_MODE;
  s_dirty = true;

  Serial0.printf("UI: gomb GPIO%d, indulo oldal %s, indulo uzemmod %s\n",
                 UI_BTN_PIN, UI_TABLE[UI_PAGE_STATUS].name,
                 UI_TABLE[s_mode].name);
  Serial0.println("UI: rovid nyomas = kovetkezo oldal, "
                  "hosszu = demod ki + vissza a statuszra");
}

void ui_tick(uint32_t now)
{
  /* --- gomb: pergesmentesites --- */
  bool raw = (digitalRead(UI_BTN_PIN) == LOW);
  if (raw != s_btn_raw) {
    s_btn_raw = raw;
    s_btn_change_ms = now;
  } else if ((now - s_btn_change_ms) >= UI_DEBOUNCE_MS && raw != s_btn_stable) {
    s_btn_stable = raw;
    if (raw) {
      s_btn_down_ms = now;
      s_long_fired = false;
    } else {
      /* felengedes: ha nem sult el a hosszu nyomas, ez rovid volt */
      if (!s_long_fired) ui_next(now);
    }
  }

  /* hosszu nyomas: mar LENYOMVA elsul, nem felengedeskor — igy erzed,
   * hogy megtortent, nem kell talalgatni, mikor engedd el */
  if (s_btn_stable && !s_long_fired && (now - s_btn_down_ms) >= UI_LONG_MS) {
    s_long_fired = true;
    ui_stop_demod(now);
  }

  /* --- tetlenseg: vissza a statuszra, de az UZEMMOD MARAD ---
   * A kituzott demod-lap kivetel: sem a villantas lejarata, sem a tetlenseg
   * nem viszi vissza.
   * A SCAN lap is marad, amig te nezed (ne ugorjon el 20 s utan, ha epp
   * a waterfallt figyeled). */
  const bool locked = (s_demod_pinned &&
                       (s_page == UI_PAGE_APRS || s_page == UI_PAGE_CW))
                   || (s_page == UI_PAGE_SCAN);

  if (s_hold_until_ms && (int32_t)(now - s_hold_until_ms) >= 0) {
    s_hold_until_ms = 0;
    if (!locked) ui_set_page(UI_PAGE_STATUS, now);
  }
  if (!s_hold_until_ms && s_page != UI_PAGE_STATUS && !locked
      && (now - s_last_activity_ms) >= UI_IDLE_BACK_MS) {
    ui_set_page(UI_PAGE_STATUS, now);
  }
}

void ui_flash(ui_page_t p, uint32_t hold_ms)
{
  if (s_mode != p) return;            /* csak ha tenyleg az fut */
  uint32_t now = millis();
  ui_set_page(p, now);
  s_hold_until_ms = now + hold_ms;
}

ui_page_t   ui_page(void)                 { return s_page; }
ui_page_t   ui_mode(void)                 { return s_mode; }
bool        ui_page_ready(ui_page_t p)    { return UI_TABLE[p].ready; }
const char *ui_page_name(ui_page_t p)     { return UI_TABLE[p].name; }
bool        ui_dirty(void)                { return s_dirty; }
void        ui_clear_dirty(void)          { s_dirty = false; }

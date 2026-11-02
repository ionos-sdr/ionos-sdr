/* SPDX-License-Identifier: MIT
 *
 * ui.cpp — button + menu state machine on the ESP32-S3
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * CW page READY. SCAN page = view (not a demodulator).
 * The table is the SINGLE source of truth.
 */

#include "ui.h"
#include <Arduino.h>
#include "board.h"

/* ==================== CONFIG ==================== */

#if !BRD_BTN_LADDER
#define UI_BTN_PIN        BRD_BTN_PIN /* Heltec V3 PRG button (GPIO0) */
#endif
#define UI_DEBOUNCE_MS    30
#define UI_LONG_MS        800
#define UI_IDLE_BACK_MS   20000       /* back to STATUS after inactivity */

#define UI_DEFAULT_MODE   UI_PAGE_APRS

/* ==================== THE MENU TABLE ====================
 * This is the ONLY place where the pages exist. A new demodulator gets a
 * row here, and the order also comes from here. */
typedef struct {
  const char *name;
  bool        is_demod;   /* switches a mode, or is only a view */
  bool        ready;      /* implemented yet */
} ui_entry_t;

static const ui_entry_t UI_TABLE[UI_PAGE_COUNT] = {
  /* name         demod   ready */
  { "STATUS",     false,  true  },
  { "RF",         false,  true  },
  { "SDR STREAM", false,  true  },
  { "SCAN",       false,  true  },   /* dithered waterfall — view */
  { "APRS iGATE", true,   true  },
  { "CW",         true,   true  },
  { "WSPR",       true,   false },
  { "FT8",        true,   false },
};

/* ==================== STATE ==================== */

static ui_page_t s_page = UI_PAGE_STATUS;
static ui_page_t s_mode = UI_PAGE_STATUS;
static bool      s_dirty = true;

static bool      s_btn_stable = false;   /* true = pressed */
static bool      s_btn_raw = false;
static uint32_t  s_btn_change_ms = 0;
static uint32_t  s_btn_down_ms = 0;
static bool      s_long_fired = false;

static uint32_t  s_last_activity_ms = 0;
static uint32_t  s_hold_until_ms = 0;

/* PINNED APRS/CW page: if the user paged to the demod page MANUALLY, it
 * does not return to the status page on inactivity (20 s) nor on the flash
 * of an incoming signal — it stays until paged away. */
static bool      s_demod_pinned = false;

/* ==================== HELPERS ==================== */

static void ui_set_page(ui_page_t p, uint32_t now)
{
  if (p != s_page) {
    s_page = p;
    s_dirty = true;
  }
  s_last_activity_ms = now;
}

/* Next page. Not-yet-ready demodulators are NOT skipped: they remain
 * visible to show what is planned — they only report that they are not
 * implemented yet and do not switch a mode. */
static void ui_next(uint32_t now)
{
  ui_page_t p = (ui_page_t)((s_page + 1) % UI_PAGE_COUNT);
  ui_set_page(p, now);

  if (UI_TABLE[p].is_demod && UI_TABLE[p].ready) {
    if (s_mode != p) {
      s_mode = p;
      Serial0.printf("\nUI: mode -> %s\n", UI_TABLE[p].name);
    }
  }
  s_hold_until_ms = 0;

  /* Pin ONLY when the user paged to a ready demod page manually. */
  s_demod_pinned = (UI_TABLE[p].is_demod && UI_TABLE[p].ready);
}

/* Previous page (ionos-sdr-mini LEFT / UP). Same rules as ui_next. */
static void ui_prev(uint32_t now)
{
  ui_page_t p = (ui_page_t)((s_page + UI_PAGE_COUNT - 1) % UI_PAGE_COUNT);
  ui_set_page(p, now);
  if (UI_TABLE[p].is_demod && UI_TABLE[p].ready && s_mode != p) {
    s_mode = p;
    Serial0.printf("\nUI: mode -> %s\n", UI_TABLE[p].name);
  }
  s_hold_until_ms = 0;
  s_demod_pinned = (UI_TABLE[p].is_demod && UI_TABLE[p].ready);
}

#if BRD_BTN_LADDER
/* ionos-sdr-mini: two ADC1 ladders, 10 k pull-up to 3V3, buttons to GND via
 * 0R / 2k2 / 6k8 / 22k. Thresholds at the level midpoints (mV). */
enum { BTN_NONE = 0, BTN_BACK, BTN_OK, BTN_SPARE, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT };
static const char *BTN_NAME[] = {"-", "BACK", "OK", "SPARE", "UP", "DOWN", "LEFT", "RIGHT"};
static int ladder_level(int mv)
{
  if (mv < 300)  return 0;
  if (mv < 970)  return 1;
  if (mv < 1800) return 2;
  if (mv < 2790) return 3;
  return -1;
}
static int read_buttons(void)
{
  int a = ladder_level(analogReadMilliVolts(BRD_BTN_A_PIN));
  if (a >= 0 && a <= 2) return BTN_BACK + a;              /* BACK, OK, SPARE */
  int b = ladder_level(analogReadMilliVolts(BRD_BTN_B_PIN));
  if (b >= 0) return BTN_UP + b;                          /* UP, DOWN, LEFT, RIGHT */
  return BTN_NONE;
}
static int s_btn_id = BTN_NONE;
#endif

static void ui_stop_demod(uint32_t now)
{
  if (s_mode != UI_PAGE_STATUS) {
    Serial0.printf("\nUI: %s stopped\n", UI_TABLE[s_mode].name);
    s_mode = UI_PAGE_STATUS;
  }
  s_demod_pinned = false;
  ui_set_page(UI_PAGE_STATUS, now);
}

/* ==================== PUBLIC API ==================== */

void ui_init(void)
{
  /* With pull-up: the button pulls to GND. GPIO0 is a strapping pin, but a
   * plain input at run time. */
#if BRD_BTN_LADDER
  analogSetPinAttenuation(BRD_BTN_A_PIN, ADC_11db);
  analogSetPinAttenuation(BRD_BTN_B_PIN, ADC_11db);
#else
  pinMode(UI_BTN_PIN, INPUT_PULLUP);
#endif
  s_btn_raw = s_btn_stable = false;
  s_page = UI_PAGE_STATUS;
  s_mode = UI_DEFAULT_MODE;
  s_dirty = true;

#if BRD_BTN_LADDER
  Serial0.printf("UI: ladder buttons GPIO%d/%d, start page %s, start mode %s\n",
                 BRD_BTN_A_PIN, BRD_BTN_B_PIN, UI_TABLE[UI_PAGE_STATUS].name,
                 UI_TABLE[s_mode].name);
  Serial0.println("UI: OK/RIGHT/DOWN = next page, LEFT/UP = previous, "
                  "BACK or any long press = demod off + back to status");
#else
  Serial0.printf("UI: button GPIO%d, start page %s, start mode %s\n",
                 UI_BTN_PIN, UI_TABLE[UI_PAGE_STATUS].name,
                 UI_TABLE[s_mode].name);
  Serial0.println("UI: short press = next page, "
                  "long = demod off + back to status");
#endif
}

void ui_tick(uint32_t now)
{
  /* --- button: debounce --- */
#if BRD_BTN_LADDER
  static uint32_t s_adc_ms = 0;
  static int s_raw_id = BTN_NONE;
  if (now - s_adc_ms >= 10) { s_adc_ms = now; s_raw_id = read_buttons(); }
  int id = s_raw_id;
  bool raw = (id != BTN_NONE);
  if (raw && !s_btn_stable && id != s_btn_id) { s_btn_id = id; s_btn_change_ms = now; s_btn_raw = !raw; }
#else
  bool raw = (digitalRead(UI_BTN_PIN) == LOW);
#endif
  if (raw != s_btn_raw) {
    s_btn_raw = raw;
    s_btn_change_ms = now;
  } else if ((now - s_btn_change_ms) >= UI_DEBOUNCE_MS && raw != s_btn_stable) {
    s_btn_stable = raw;
    if (raw) {
      s_btn_down_ms = now;
      s_long_fired = false;
    } else {
      /* release: if the long press did not fire, this was a short press */
#if BRD_BTN_LADDER
      if (!s_long_fired) {
        Serial0.printf("UI: button %s\n", BTN_NAME[s_btn_id]);
        switch (s_btn_id) {
          case BTN_BACK:  ui_stop_demod(now); break;
          case BTN_LEFT:
          case BTN_UP:    ui_prev(now); break;
          case BTN_SPARE: break;
          default:        ui_next(now); break;           /* OK, RIGHT, DOWN */
        }
      }
      s_btn_id = BTN_NONE;
#else
      if (!s_long_fired) ui_next(now);
#endif
    }
  }

  /* long press: fires while still HELD DOWN, not on release — so the user
   * feels it happen and need not guess when to release */
  if (s_btn_stable && !s_long_fired && (now - s_btn_down_ms) >= UI_LONG_MS) {
    s_long_fired = true;
    ui_stop_demod(now);
  }

  /* --- inactivity: back to status, but the MODE STAYS ---
   * The pinned demod page is the exception: neither the flash expiry nor
   * inactivity returns it.
   * The SCAN page also stays while being viewed (it must not jump away
   * after 20 s while the waterfall is being watched). */
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
  if (s_mode != p) return;            /* only if that mode is actually running */
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
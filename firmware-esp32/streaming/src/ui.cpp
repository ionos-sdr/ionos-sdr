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
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
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
static int s_mv_a = 0, s_mv_b = 0;                        /* last raw readings (diag) */
static int read_buttons(void)
{
  s_mv_a = analogReadMilliVolts(BRD_BTN_A_PIN);
  s_mv_b = analogReadMilliVolts(BRD_BTN_B_PIN);
  int a = ladder_level(s_mv_a);
  if (a >= 0 && a <= 2) return BTN_BACK + a;              /* BACK, OK, SPARE */
  int b = ladder_level(s_mv_b);
  if (b >= 0) return BTN_UP + b;                          /* UP, DOWN, LEFT, RIGHT */
  return BTN_NONE;
}
static int s_btn_id = BTN_NONE;

/* Console 'U': print both ladders in mV and the decoded button (bring-up aid). */
void ui_ladder_diag(void)
{
  int a = analogReadMilliVolts(BRD_BTN_A_PIN), b = analogReadMilliVolts(BRD_BTN_B_PIN);
  int ra = analogRead(BRD_BTN_A_PIN), rb = analogRead(BRD_BTN_B_PIN);
  Serial0.printf("UI: ladder A GPIO%d %d mV (raw %d) -> %s | B GPIO%d %d mV (raw %d) -> %s\n",
                 BRD_BTN_A_PIN, a, ra, BTN_NAME[(ladder_level(a) >= 0 && ladder_level(a) <= 2) ? BTN_BACK + ladder_level(a) : BTN_NONE],
                 BRD_BTN_B_PIN, b, rb, BTN_NAME[(ladder_level(b) >= 0) ? BTN_UP + ladder_level(b) : BTN_NONE]);
  gpio_dump_io_configuration(stdout, BIT64(BRD_BTN_A_PIN) | BIT64(BRD_BTN_B_PIN));
  fflush(stdout);
  /* ADC self-test: the internal ~45 k pull-down against the external 10 k
   * pull-up must drag the pin to ~0.6 V. If the reading stays at full scale
   * the ADC is not sampling this pin at all. */
  gpio_pulldown_en((gpio_num_t)BRD_BTN_A_PIN); delay(5);
  int a_pd = analogReadMilliVolts(BRD_BTN_A_PIN), ra_pd = analogRead(BRD_BTN_A_PIN);
  gpio_pulldown_dis((gpio_num_t)BRD_BTN_A_PIN); delay(5);
  int a_back = analogReadMilliVolts(BRD_BTN_A_PIN);
  /* second opinion: raw oneshot driver, own handle, ADC1 ch0/ch1 (GPIO1/2) */
  adc_oneshot_unit_handle_t h = NULL;
  adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1, .clk_src = (adc_oneshot_clk_src_t)0, .ulp_mode = ADC_ULP_MODE_DISABLE };
  int r0 = -1, r1 = -1; esp_err_t e = adc_oneshot_new_unit(&uc, &h);
  if (e == ESP_OK) {
    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(h, ADC_CHANNEL_0, &cc);
    adc_oneshot_config_channel(h, ADC_CHANNEL_1, &cc);
    adc_oneshot_read(h, ADC_CHANNEL_0, &r0);
    adc_oneshot_read(h, ADC_CHANNEL_1, &r1);
    adc_oneshot_del_unit(h);
  }
  Serial0.printf("UI: selftest A with int. pull-down: %d mV (raw %d), after: %d mV | raw oneshot unit: %s ch0=%d ch1=%d\n",
                 a_pd, ra_pd, a_back, esp_err_to_name(e), r0, r1);
}
#else
void ui_ladder_diag(void) { Serial0.println("UI: no ladder on this board"); }
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
  if (now - s_adc_ms >= 10) {
    s_adc_ms = now; s_raw_id = read_buttons();
    /* bring-up diag: every raw change, with the ladder voltages. Also a
     * 2 s heartbeat whenever a ladder is off its idle level (>2790 mV). */
    static int s_log_id = BTN_NONE; static uint32_t s_log_ms = 0;
    bool off_idle = (s_mv_a < 2790) || (s_mv_b < 2790);
    if (s_raw_id != s_log_id || (off_idle && now - s_log_ms >= 2000)) {
      s_log_id = s_raw_id; s_log_ms = now;
      Serial0.printf("UI: raw %s  A=%d mV B=%d mV\n", BTN_NAME[s_raw_id], s_mv_a, s_mv_b);
    }
  }
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
  if (s_btn_stable && !s_long_fired && (now - s_btn_down_ms) >= UI_LONG_MS
#if BRD_BTN_LADDER
      /* ladder pad: only BACK has a long-press meaning (demod off + status);
       * every other button acts on release however long it was held */
      && s_btn_id == BTN_BACK
#endif
     ) {
    s_long_fired = true;
    Serial0.println("UI: long press -> demod off, status page");
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
/* SPDX-License-Identifier: MIT
 *
 * menu.cpp - see menu.h
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Drawing goes through tft_overlay/tft_fill/tft_text (tft.cpp owns the
 * panel and its mutex). Nothing here blocks: a key press redraws at most
 * one page (12 rows), ~2 ms of SPI at 40 MHz.
 */

#include "menu.h"
#include "tft.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"

#ifndef MENU_OUT
#define MENU_OUT Serial0
#endif

const int32_t MENU_DECIM[] = { 2, 4, 8, 16, 32 };
const int     MENU_DECIM_N = 5;

/* ===================== settings + NVS ================================= */
static menu_settings_t s_set = { 0, 2, 433775, 3, 0, 100 };   /* defaults: standalone, R=8, LoRa-APRS, dim LED */
static bool     s_nvs_dirty = false;
static uint32_t s_nvs_dirty_ms = 0;

static void nvs_load(void)
{
  /* idempotent; the WiFi stack does the same later. A partition from an older
   * NVS layout is erased (nothing of value lives there yet). */
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
  nvs_handle_t h;
  if (nvs_open("ionos", NVS_READONLY, &h) != ESP_OK) return;
  nvs_get_i32(h, "mode",  &s_set.mode);
  nvs_get_i32(h, "decim", &s_set.decim_idx);
  nvs_get_i32(h, "freq",  &s_set.freq_khz);
  nvs_get_i32(h, "led",   &s_set.led_bright);
  nvs_get_i32(h, "fft",   &s_set.fft_idx);
  nvs_get_i32(h, "bl",    &s_set.bl_pct);
  nvs_close(h);
  if (s_set.decim_idx < 0 || s_set.decim_idx >= MENU_DECIM_N) s_set.decim_idx = 2;
  if (s_set.freq_khz < 100000 || s_set.freq_khz > 1000000) s_set.freq_khz = 433775;
  if (s_set.led_bright < 0 || s_set.led_bright > 255) s_set.led_bright = 3;
  if (s_set.fft_idx < 0 || s_set.fft_idx > 1) s_set.fft_idx = 0;
  if (s_set.mode < 0 || s_set.mode > 1) s_set.mode = 0;
}

static void nvs_save(void)
{
  nvs_handle_t h;
  if (nvs_open("ionos", NVS_READWRITE, &h) != ESP_OK) { MENU_OUT.println("MENU: nvs_open failed"); return; }
  nvs_set_i32(h, "mode",  s_set.mode);
  nvs_set_i32(h, "decim", s_set.decim_idx);
  nvs_set_i32(h, "freq",  s_set.freq_khz);
  nvs_set_i32(h, "led",   s_set.led_bright);
  nvs_set_i32(h, "fft",   s_set.fft_idx);
  nvs_set_i32(h, "bl",    s_set.bl_pct);
  nvs_commit(h);
  nvs_close(h);
  MENU_OUT.println("MENU: settings saved");
}

static void mark_dirty(uint32_t now) { s_nvs_dirty = true; s_nvs_dirty_ms = now; }

/* ===================== apply functions ================================ */
static void apply_mode(void)  { menu_host_set_mode((int)s_set.mode); }
static void apply_decim(void) { menu_host_set_decim(MENU_DECIM[s_set.decim_idx]); }
static void apply_freq(void)  { menu_host_tune((uint32_t)s_set.freq_khz); }
static void apply_fft(void)   { menu_host_console(s_set.fft_idx ? "Fset n=2048" : "Fset n=1024"); }
static void apply_led(void)
{
  char b[24];
  if (s_set.led_bright <= 0) { menu_host_console("L0"); return; }
  snprintf(b, sizeof b, "Lset fenyero=%ld", (long)s_set.led_bright);
  menu_host_console(b);
  menu_host_console("L1");
}
static void act_bench_start(void) { menu_host_console("Bs200,1000,200,8"); }
static void act_bench_stop(void)  { menu_host_console("B0"); }
static void act_save(void)        { nvs_save(); s_nvs_dirty = false; }
static void act_factory(void)
{
  s_set.mode = 0; s_set.decim_idx = 2; s_set.freq_khz = 433775;
  s_set.led_bright = 3; s_set.fft_idx = 0; s_set.bl_pct = 100;
  nvs_save();
  apply_led(); apply_fft(); apply_decim(); apply_freq(); apply_mode();
}

/* ===================== item model ===================================== */
typedef enum { MI_SUB, MI_VALUE, MI_ACTION, MI_ENUM, MI_INT, MI_BOOL } mi_type_t;

typedef struct mi {
  const char   *label;
  mi_type_t     type;
  const struct mi *sub; int nsub;                 /* MI_SUB */
  const char *(*get)(char *buf, int n);           /* MI_VALUE (live text) */
  void (*act)(void);                              /* MI_ACTION, and the apply hook of ENUM/INT/BOOL */
  int32_t      *val;                              /* ENUM: index; INT: value; BOOL: 0/1 */
  const char *const *opts; int nopts;             /* MI_ENUM */
  int32_t       min, max, step, coarse;           /* MI_INT */
  const char   *unit;
} mi_t;

/* ---- live value getters ---- */
static menu_host_info_t s_info;
static uint32_t s_info_ms = 0;
static void info_refresh(uint32_t now) { if (now - s_info_ms >= 1000) { s_info_ms = now; menu_host_info(&s_info); } }

static const char *g_fw_esp(char *b, int n)  { snprintf(b, n, "%s", s_info.fw_esp ? s_info.fw_esp : "?"); return b; }
static const char *g_fw_fg23(char *b, int n) { snprintf(b, n, "%s", s_info.fw_fg23 ? s_info.fw_fg23 : "n/a"); return b; }
static const char *g_board(char *b, int n)   { snprintf(b, n, "%s", s_info.board ? s_info.board : "?"); return b; }
static const char *g_mode(char *b, int n)    { snprintf(b, n, "%s", s_info.mode_live ? "USB" : "STANDALONE"); return b; }
static const char *g_uptime(char *b, int n)  { uint32_t s = s_info.uptime_s; snprintf(b, n, "%luh%02lum", (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60)); return b; }
static const char *g_heap(char *b, int n)    { snprintf(b, n, "%luk", (unsigned long)s_info.heap_min_k); return b; }
static const char *g_psram(char *b, int n)   { snprintf(b, n, "%luk", (unsigned long)s_info.psram_k); return b; }
static const char *g_blk(char *b, int n)     { snprintf(b, n, "%.0f/s", (double)s_info.blk_s); return b; }
static const char *g_sps(char *b, int n)     { snprintf(b, n, "%lu", (unsigned long)s_info.sps); return b; }
static const char *g_lost(char *b, int n)    { snprintf(b, n, "%lu", (unsigned long)s_info.lost); return b; }
static const char *g_rfrssi(char *b, int n)  { snprintf(b, n, "%ddBm", s_info.rf_rssi); return b; }
static const char *g_freq(char *b, int n)    { snprintf(b, n, "%lu.%03luM", (unsigned long)(s_info.freq_khz / 1000), (unsigned long)(s_info.freq_khz % 1000)); return b; }
static const char *g_update(char *b, int n)  { snprintf(b, n, "n/a"); return b; }   /* UI-M2 */
static const char *g_ssid(char *b, int n)    { snprintf(b, n, "%s", (s_info.ssid && s_info.ssid[0]) ? s_info.ssid : "-"); return b; }
static const char *g_ip(char *b, int n)      { snprintf(b, n, "%s", s_info.ip ? s_info.ip : "-"); return b; }
static const char *g_wrssi(char *b, int n)   { if (s_info.wifi_rssi == 0) snprintf(b, n, "down"); else snprintf(b, n, "%ddBm", s_info.wifi_rssi); return b; }
static const char *g_rate(char *b, int n)    { snprintf(b, n, "%ldksps", (long)(400000 / MENU_DECIM[s_set.decim_idx] / 1000)); return b; }

/* ---- enum option tables ---- */
static const char *const OPT_MODE[]  = { "STANDALONE", "USB" };
static const char *const OPT_DECIM[] = { "2", "4", "8", "16", "32" };
static const char *const OPT_FFT[]   = { "1024", "2048" };

/* ---- the tree ---- */
static const mi_t M_INFO[] = {
  { "ESP fw",    MI_VALUE, NULL, 0, g_fw_esp },
  { "FG23 fw",   MI_VALUE, NULL, 0, g_fw_fg23 },
  { "Board",     MI_VALUE, NULL, 0, g_board },
  { "Mode",      MI_VALUE, NULL, 0, g_mode },
  { "Freq",      MI_VALUE, NULL, 0, g_freq },
  { "Rate",      MI_VALUE, NULL, 0, g_sps },
  { "Blocks",    MI_VALUE, NULL, 0, g_blk },
  { "Lost",      MI_VALUE, NULL, 0, g_lost },
  { "RF RSSI",   MI_VALUE, NULL, 0, g_rfrssi },
  { "Heap min",  MI_VALUE, NULL, 0, g_heap },
  { "PSRAM",     MI_VALUE, NULL, 0, g_psram },
  { "Uptime",    MI_VALUE, NULL, 0, g_uptime },
  { "Update",    MI_VALUE, NULL, 0, g_update },
};
static const mi_t M_WIFI[] = {
  { "SSID",      MI_VALUE, NULL, 0, g_ssid },
  { "IP",        MI_VALUE, NULL, 0, g_ip },
  { "RSSI",      MI_VALUE, NULL, 0, g_wrssi },
  { "Bench->sink",  MI_ACTION, NULL, 0, NULL, act_bench_start },
  { "Bench stop",   MI_ACTION, NULL, 0, NULL, act_bench_stop },
};
static const mi_t M_SDR[] = {
  { "Mode",      MI_ENUM, NULL, 0, NULL, apply_mode,  &s_set.mode,      OPT_MODE,  2 },
  { "Freq kHz",  MI_INT,  NULL, 0, NULL, apply_freq,  &s_set.freq_khz,  NULL, 0, 100000, 1000000, 1, 25 },
  { "Decim R",   MI_ENUM, NULL, 0, NULL, apply_decim, &s_set.decim_idx, OPT_DECIM, 5 },
  { "Rate",      MI_VALUE, NULL, 0, g_rate },
  { "FFT",       MI_ENUM, NULL, 0, NULL, apply_fft,   &s_set.fft_idx,   OPT_FFT,   2 },
};
static const mi_t M_SYS[] = {
  { "LED bright", MI_INT, NULL, 0, NULL, apply_led, &s_set.led_bright, NULL, 0, 0, 255, 1, 10 },
  { "Save now",   MI_ACTION, NULL, 0, NULL, act_save },
  { "Factory reset", MI_ACTION, NULL, 0, NULL, act_factory },
};
static const mi_t M_MAIN[] = {
  { "1 INFO",       MI_SUB, M_INFO, (int)(sizeof M_INFO / sizeof M_INFO[0]) },
  { "2 WIFI",       MI_SUB, M_WIFI, (int)(sizeof M_WIFI / sizeof M_WIFI[0]) },
  { "3 SDR CONFIG", MI_SUB, M_SDR,  (int)(sizeof M_SDR  / sizeof M_SDR[0])  },
  { "4 SYSTEM",     MI_SUB, M_SYS,  (int)(sizeof M_SYS  / sizeof M_SYS[0])  },
};

/* ===================== navigation state =============================== */
#define MENU_DEPTH   4
#define MENU_IDLE_MS 30000u
#define ROW_Y0       76
#define ROW_H        20
#define ROWS         11
#define TITLE_Y      52

typedef struct { const mi_t *items; int n; int sel; int top; const char *title; } level_t;
static level_t  s_stack[MENU_DEPTH];
static int      s_depth = -1;              /* -1 = closed */
static bool     s_editing = false;
static int32_t  s_edit_backup = 0;
static uint32_t s_last_key_ms = 0;
static bool     s_has_values = false;

/* ===================== drawing ======================================== */
static void draw_row(const level_t *L, int i)
{
  const mi_t *it = &L->items[i];
  int y = ROW_Y0 + (i - L->top) * ROW_H;
  bool sel = (i == L->sel);
  uint16_t bg = sel ? TFTC_CYAN : TFTC_BLACK;
  uint16_t fg = sel ? TFTC_BLACK : TFTC_WHITE;
  tft_fill(0, y, TFT_W, ROW_H, bg);
  tft_text(4, y + 2, 2, fg, bg, it->label);

  char v[24] = "";
  switch (it->type) {
    case MI_SUB:    snprintf(v, sizeof v, ">"); break;
    case MI_VALUE:  if (it->get) it->get(v, sizeof v); break;
    case MI_ACTION: snprintf(v, sizeof v, "[OK]"); break;
    case MI_ENUM:   snprintf(v, sizeof v, "%s", it->opts[*it->val]); break;
    case MI_BOOL:   snprintf(v, sizeof v, "%s", *it->val ? "ON" : "OFF"); break;
    case MI_INT:
      if (sel && s_editing) snprintf(v, sizeof v, "<%ld>", (long)*it->val);
      else                  snprintf(v, sizeof v, "%ld", (long)*it->val);
      break;
  }
  int len = (int)strlen(v);
  if (len > 10) { v[10] = 0; len = 10; }            /* 12 px/char, 120 px for values */
  uint16_t vfg = (it->type == MI_VALUE) ? (sel ? TFTC_BLACK : TFTC_YELLOW) : fg;
  if (sel && s_editing) vfg = TFTC_RED;
  tft_text(TFT_W - 4 - len * 12, y + 2, 2, vfg, bg, v);
}

static void draw_level(void)
{
  const level_t *L = &s_stack[s_depth];
  tft_fill(0, TFT_HDR_H, TFT_W, TFT_H - TFT_HDR_H, TFTC_BLACK);
  tft_fill(0, TITLE_Y - 2, TFT_W, ROW_H, TFTC_NAVY);
  tft_text(4, TITLE_Y, 2, TFTC_CYAN, TFTC_NAVY, L->title);
  int last = L->top + ROWS; if (last > L->n) last = L->n;
  for (int i = L->top; i < last; i++) draw_row(L, i);
  tft_text(4, TFT_H - 12, 1, TFTC_DARKGREY, TFTC_BLACK,
           s_editing ? "<> step  ^v coarse  OK=set  BACK=cancel"
                     : "^v move  OK=select  BACK=up  <>=change");
  s_has_values = false;
  for (int i = 0; i < L->n; i++) if (L->items[i].type == MI_VALUE) s_has_values = true;
}

static void push_level(const mi_t *items, int n, const char *title)
{
  if (s_depth + 1 >= MENU_DEPTH) return;
  s_depth++;
  level_t *L = &s_stack[s_depth];
  L->items = items; L->n = n; L->sel = 0; L->top = 0; L->title = title;
  draw_level();
}

/* ===================== public ========================================= */
void menu_init(void)
{
  nvs_load();
  MENU_OUT.printf("MENU: settings mode=%ld decim=R%ld freq=%ld kHz led=%ld fft=%s\n",
                  (long)s_set.mode, (long)MENU_DECIM[s_set.decim_idx], (long)s_set.freq_khz,
                  (long)s_set.led_bright, s_set.fft_idx ? "2048" : "1024");
}

void menu_apply_boot(void)
{
  apply_led();
  apply_fft();
  /* decim/freq: applied by the host at the first stream start (menu_host_set_decim is
   * ESP-state aware); the mode is UI-M4 territory. */
  apply_decim();
  apply_freq();
  apply_mode();
}

bool menu_is_open(void) { return s_depth >= 0; }
const menu_settings_t *menu_settings(void) { return &s_set; }

void menu_open(void)
{
  if (s_depth >= 0) return;
  s_editing = false;
  s_last_key_ms = millis();
  menu_host_info(&s_info); s_info_ms = millis();
  tft_overlay(true);
  push_level(M_MAIN, (int)(sizeof M_MAIN / sizeof M_MAIN[0]), "MENU");
  MENU_OUT.println("MENU: open");
}

void menu_close(void)
{
  if (s_depth < 0) return;
  s_depth = -1; s_editing = false;
  tft_overlay(false);
  if (s_nvs_dirty) { nvs_save(); s_nvs_dirty = false; }
  MENU_OUT.println("MENU: close");
}

static void item_changed(const mi_t *it, uint32_t now)
{
  if (it->act) it->act();
  mark_dirty(now);
}

void menu_key(menu_key_t k, uint32_t now)
{
  if (s_depth < 0) return;
  s_last_key_ms = now;
  level_t *L = &s_stack[s_depth];
  const mi_t *it = (L->n > 0) ? &L->items[L->sel] : NULL;

  if (s_editing && it && it->type == MI_INT) {
    int32_t v = *it->val;
    switch (k) {
      case MK_LEFT:  v -= it->step;   break;
      case MK_RIGHT: v += it->step;   break;
      case MK_DOWN:  v -= it->coarse; break;
      case MK_UP:    v += it->coarse; break;
      case MK_OK:    s_editing = false; item_changed(it, now); draw_level(); return;
      case MK_BACK:  *it->val = s_edit_backup; s_editing = false; draw_level(); return;
      default: return;
    }
    if (v < it->min) v = it->min;
    if (v > it->max) v = it->max;
    *it->val = v;
    draw_row(L, L->sel);
    return;
  }

  switch (k) {
    case MK_UP:
      if (L->sel > 0) { L->sel--; if (L->sel < L->top) { L->top = L->sel; draw_level(); } else { draw_row(L, L->sel + 1); draw_row(L, L->sel); } }
      break;
    case MK_DOWN:
      if (L->sel < L->n - 1) { L->sel++; if (L->sel >= L->top + ROWS) { L->top = L->sel - ROWS + 1; draw_level(); } else { draw_row(L, L->sel - 1); draw_row(L, L->sel); } }
      break;
    case MK_BACK:
      if (s_depth == 0) menu_close(); else { s_depth--; draw_level(); }
      break;
    case MK_OK:
      if (!it) break;
      switch (it->type) {
        case MI_SUB:    push_level(it->sub, it->nsub, it->label); break;
        case MI_ACTION: if (it->act) it->act(); MENU_OUT.printf("MENU: action %s\n", it->label); break;
        case MI_INT:    s_editing = true; s_edit_backup = *it->val; draw_level(); break;
        case MI_ENUM:   *it->val = (*it->val + 1) % it->nopts; item_changed(it, now); draw_level(); break;
        case MI_BOOL:   *it->val = !*it->val; item_changed(it, now); draw_row(L, L->sel); break;
        default: break;
      }
      break;
    case MK_LEFT:
    case MK_RIGHT:
      if (!it) break;
      if (it->type == MI_ENUM) {
        *it->val = (*it->val + (k == MK_RIGHT ? 1 : it->nopts - 1)) % it->nopts;
        item_changed(it, now); draw_level();
      } else if (it->type == MI_BOOL) {
        *it->val = !*it->val; item_changed(it, now); draw_row(L, L->sel);
      } else if (it->type == MI_INT) {
        s_editing = true; s_edit_backup = *it->val; draw_level();
      }
      break;
    default: break;
  }
}

void menu_tick(uint32_t now)
{
  if (s_nvs_dirty && now - s_nvs_dirty_ms >= 2000) { nvs_save(); s_nvs_dirty = false; }
  if (s_depth < 0) return;
  if (now - s_last_key_ms >= MENU_IDLE_MS) { MENU_OUT.println("MENU: idle close"); menu_close(); return; }
  if (s_has_values && now - s_info_ms >= 1000) {
    info_refresh(now);
    const level_t *L = &s_stack[s_depth];
    int last = L->top + ROWS; if (last > L->n) last = L->n;
    for (int i = L->top; i < last; i++) if (L->items[i].type == MI_VALUE) draw_row(L, i);
  }
}

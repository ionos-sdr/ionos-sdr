/* SPDX-License-Identifier: MIT
 *
 * menu.h - on-screen menu of the ionos-sdr-mini (TFT + 7-button pad)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * A small, static menu engine: a tree of items (sub-menu, info page, action,
 * enum, integer, bool), drawn into the waterfall area of the ILI9341 while
 * the header stays. Navigation: UP/DOWN = move, OK = enter / toggle / start
 * editing, LEFT/RIGHT = change a value, BACK = up one level (closes the
 * menu from the top level). 30 s without a key closes it.
 *
 * Settings live in NVS (namespace "ionos"); the host side of the device
 * (tuning, cmdlink, stats) is reached through the menu_host_* functions,
 * which main.cpp implements (C linkage, like rtl_pending_set).
 *
 * UI-M1 of claude/ionos-mini-menu-uzemmodok-host-milestones-2026-11-03.md.
 */
#ifndef MENU_H
#define MENU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* key codes, same order as the ladder decoder in ui.cpp */
typedef enum { MK_NONE = 0, MK_BACK, MK_OK, MK_SPARE, MK_UP, MK_DOWN, MK_LEFT, MK_RIGHT } menu_key_t;

/* persistent settings (NVS) */
typedef struct {
  int32_t mode;          /* 0 = standalone decoder, 1 = USB host */
  int32_t decim_idx;     /* index into MENU_DECIM[] */
  int32_t freq_khz;      /* last tuned frequency */
  int32_t led_bright;    /* WS2812 peak brightness 0..255 (0 = off) */
  int32_t fft_idx;       /* 0 = 1024, 1 = 2048 */
  int32_t bl_pct;        /* backlight % (reserved, UI-M5) */
} menu_settings_t;

extern const int32_t MENU_DECIM[];     /* 2,4,8,16,32 */
extern const int     MENU_DECIM_N;

void menu_init(void);                  /* loads NVS, applies nothing yet */
void menu_apply_boot(void);            /* applies the stored settings once the chain runs */
bool menu_is_open(void);
void menu_open(void);
void menu_close(void);
void menu_key(menu_key_t k, uint32_t now_ms);
void menu_tick(uint32_t now_ms);       /* idle timeout + live info refresh */
const menu_settings_t *menu_settings(void);

/* ---- implemented by main.cpp ---- */
typedef struct {
  const char *fw_esp;      /* FW_VERSION */
  const char *fw_fg23;     /* "n/a" until VER? exists */
  const char *board;       /* radio board name */
  const char *ip;          /* "-" when down */
  const char *ssid;
  int         wifi_rssi;   /* dBm, 0 = down */
  int         rf_rssi;     /* FG23 RAIL RSSI dBm */
  uint32_t    sps;
  float       blk_s;
  uint32_t    lost;
  uint32_t    usb_drop;    /* blocks dropped on the USB path */
  uint32_t    heap_min_k;
  uint32_t    psram_k;
  uint32_t    uptime_s;
  uint32_t    freq_khz;    /* current tuning as known by the ESP */
  int         mode_live;   /* what the chain actually runs (0/1) */
} menu_host_info_t;

void menu_host_info(menu_host_info_t *o);           /* fill live values */
bool menu_host_tune(uint32_t khz);                  /* -> rtl_pending_set */
void menu_host_cmdlink(const char *line);           /* -> cmdlink_send */
void menu_host_set_decim(int32_t R);                /* FG23 'i' semantics handled by the ESP (knows if the stream runs) */
bool menu_host_console(const char *line);           /* -> the console dispatcher (Lset…, Fset…, Bs…) */
void menu_host_set_mode(int mode);                  /* 0/1, UI-M4 hooks here */

#ifdef __cplusplus
}
#endif
#endif /* MENU_H */

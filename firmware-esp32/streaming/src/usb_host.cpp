/* SPDX-License-Identifier: MIT
 *
 * usb_host.cpp - see usb_host.h
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * The device side of the host is reached through the menu_host_* glue of
 * main.cpp (menu.h): tuning, cmdlink, decimation, live info. Nothing here
 * touches the radio path directly.
 */

#include "usb_host.h"
#include "menu.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>

#if IONOS_USB_TINYUSB
#include "tusb_cdc.h"
#define USBH_RAW  TusbSerial    /* USB-OTG + TinyUSB CDC-ACM, see tusb_cdc.h */
#endif
#ifndef USBH_RAW
#define USBH_RAW  Serial        /* native USB CDC: raw stream + host lines */
#endif
#ifndef USBH_DIAG
#define USBH_DIAG Serial0
#endif

#define BLK_BYTES    1040u
#define HDR_BYTES    16u
#define MSG_MAX      (BLK_BYTES - HDR_BYTES)
#define MSGQ_N       4
#define FG23_FS_IN   400000u
#define TICK_HZ      4.6492f    /* RAIL fine offset: 1 tick = 46492/10000 Hz */

/* ===================== MSG1 queue ===================================== */
static uint8_t  s_q[MSGQ_N][BLK_BYTES];
static uint8_t  s_q_head = 0, s_q_n = 0;
static uint32_t s_seq = 0, s_dropped = 0;

bool usbhost_send(uint8_t type, const char *text)
{
  if (!text) return false;
  if (s_q_n >= MSGQ_N) { s_dropped++; return false; }
  uint8_t *b = s_q[(s_q_head + s_q_n) % MSGQ_N];
  size_t len = strlen(text);
  if (len > MSG_MAX) len = MSG_MAX;
  memset(b, 0, BLK_BYTES);
  uint32_t magic = MSG1_MAGIC, seq = s_seq++, fs = 0;
  uint16_t n = (uint16_t)len, t = type;
  memcpy(b + 0,  &magic, 4);
  memcpy(b + 4,  &seq,   4);
  memcpy(b + 8,  &n,     2);
  memcpy(b + 10, &t,     2);
  memcpy(b + 12, &fs,    4);
  memcpy(b + HDR_BYTES, text, len);
  s_q_n++;
  return true;
}

void usbhost_flush(void)
{
  while (s_q_n) {
    if (!(bool)USBH_RAW) { s_q_n = 0; s_dropped++; return; }     /* no host: drop */
    if (USBH_RAW.availableForWrite() < (int)BLK_BYTES) return;   /* keep for the next loop */
    size_t w = USBH_RAW.write(s_q[s_q_head], BLK_BYTES);
    s_q_head = (s_q_head + 1) % MSGQ_N; s_q_n--;
    if (w != BLK_BYTES) { s_dropped++; return; }
  }
}

uint32_t usbhost_dropped(void) { return s_dropped; }

/* ===================== state ========================================== */
static bool     s_usb_mode = false;
static uint32_t s_stat_period_ms = 2000;
static bool     s_host_spoke = false;   /* status only once the host has sent a line (v1 hosts: hello only) */
static uint32_t s_stat_last_ms = 0;
static bool     s_conn_prev = false;
static int32_t  s_pending_o = 0;        /* fine offset to send after a pending F */
static uint32_t s_pending_o_ms = 0;

bool usbhost_usb_mode(void) { return s_usb_mode; }
void usbhost_set_mode(bool usb) { s_usb_mode = usb; }

/* ===================== replies ======================================== */
static void reply(const char *fmt, ...)
{
  char b[512];
  va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  usbhost_send(MSG1_RESPONSE, b);
}

static void send_hello(void)
{
  menu_host_info_t i; menu_host_info(&i);
  char b[512];
  snprintf(b, sizeof b,
           "ionos=1\nproduct=ionos-sdr-mini\nhw=rev-b3-2l\nboard=%s\nchip=EFR32FG23\n"
           "fw_esp=%s\nfw_fg23=%s\nfreq_min_hz=430000000\nfreq_max_hz=440025000\n"
           "fs_in_hz=%lu\ndecim_list=2,4,8,16,24,32,48,64\ngain_shift_max=0\nrssi=1\nspectrum=1\ntx=0\n"
           "mode=%s\n",
           i.board, i.fw_esp, i.fw_fg23, (unsigned long)FG23_FS_IN, s_usb_mode ? "USB" : "STANDALONE");
  usbhost_send(MSG1_HELLO, b);
}

static void send_status(void)
{
  menu_host_info_t i; menu_host_info(&i);
  char b[256];
  snprintf(b, sizeof b, "blk_s=%.1f sps=%lu lost=%lu rssi=%d freq_khz=%lu heap_min=%lu usb_drop=%lu mode=%s\n",
           (double)i.blk_s, (unsigned long)i.sps, (unsigned long)i.lost, i.rf_rssi,
           (unsigned long)i.freq_khz, (unsigned long)i.heap_min_k, (unsigned long)i.usb_drop,
           s_usb_mode ? "USB" : "STANDALONE");
  usbhost_send(MSG1_STATUS, b);
}

/* ===================== command handling =============================== */
static const char FORWARD[] = "FofiWTts";          /* -> FG23 handle_line() */
static const char REJECT[]  = "cwpxbzydkngvM";     /* TX / bench / diagnostics */

static void handle_line(char *line)
{
  /* trim */
  size_t n = strlen(line);
  while (n && (line[n - 1] == '\r' || line[n - 1] == '\n' || line[n - 1] == ' ')) line[--n] = 0;
  while (*line == ' ') line++;
  if (!*line) return;
  s_host_spoke = true;
  USBH_DIAG.printf("USB-HOST: '%s'\n", line);

  if (!strcmp(line, "CAP?"))  { send_hello(); return; }
  if (!strcmp(line, "VER?")) {
    menu_host_info_t i; menu_host_info(&i);
    reply("OK\nfw_esp=%s\nfw_fg23=%s\nboard=%s\n", i.fw_esp, i.fw_fg23, i.board);
    return;
  }
  if (!strncmp(line, "MODE", 4)) {
    const char *p = line + 4; while (*p == ' ') p++;
    if (!strcmp(p, "USB"))        { s_usb_mode = true;  menu_host_set_mode(1); reply("OK\nmode=USB\n"); }
    else if (!strcmp(p, "STANDALONE")) { s_usb_mode = false; menu_host_set_mode(0); reply("OK\nmode=STANDALONE\n"); }
    else reply("ERR usage: MODE USB | MODE STANDALONE\n");
    return;
  }
  if (!strncmp(line, "STAT", 4)) {
    long s = atol(line + 4);
    if (s < 0 || s > 3600) { reply("ERR usage: STAT <seconds>, 0 = off\n"); return; }
    s_stat_period_ms = (uint32_t)s * 1000u;
    reply("OK\nstat_period_s=%ld\n", s);
    return;
  }
  if (!strncmp(line, "FH", 2)) {
    long hz = atol(line + 2);
    if (hz < 100000000L || hz > 1000000000L) { reply("ERR FH: 100000000..1000000000 Hz\n"); return; }
    long khz = hz / 1000, rem = hz - khz * 1000;
    if (rem > 500) { khz++; rem -= 1000; }
    long ticks = lroundf((float)rem / TICK_HZ);
    if (!menu_host_tune((uint32_t)khz)) { reply("ERR tune rejected\n"); return; }
    s_pending_o = ticks; s_pending_o_ms = millis();       /* 'o' follows the (rate-limited) F */
    long got = khz * 1000 + lroundf((float)ticks * TICK_HZ);
    reply("OK\nf=%ld (F%ld + o%ld)\n", got, khz, ticks);
    return;
  }
  if (line[0] == 'R' && (line[1] == ' ' || (line[1] >= '0' && line[1] <= '9'))) {
    long sps = atol(line + 1);
    if (sps <= 0) { reply("ERR usage: R <sps>\n"); return; }
    long R = lroundf((float)FG23_FS_IN / (float)sps);
    /* FG23 iq_stream_norm_decim(): 2, 4, or a multiple of 8 (2026-11-04;
     * 200 / 100 ksps need the 78 MHz FG23 core and the 19.5 MHz SPI). */
    if (R < 3) R = 2;
    else if (R < 6) R = 4;
    else { R = ((R + 4) / 8) * 8; if (R < 8) R = 8; }
    if (R > 4096) R = 4096;
    menu_host_set_decim((int32_t)R);
    reply("OK\nfs=%ld decim=%ld\n", (long)FG23_FS_IN / R, R);
    return;
  }
  if (line[0] == 'G' && (line[1] == ' ' || (line[1] >= '0' && line[1] <= '9'))) {
    reply("ERR G: digital gain shift is compile-time on the FG23 (IQ_AUTOSTART_SHIFT); not yet runtime\n");
    return;
  }
  char c = line[0];
  if (strchr(REJECT, c)) { reply("ERR '%c' is not part of the host API (TX/diagnostics)\n", c); return; }
  if (strchr(FORWARD, c)) {
    if (c == 'F') {                                    /* keep the ESP's tuning bookkeeping */
      long khz = atol(line + 1);
      if (!menu_host_tune((uint32_t)khz)) { reply("ERR tune rejected: %ld kHz\n", khz); return; }
      reply("OK\nforwarded (rate-limited): %s\n", line);
      return;
    }
    menu_host_cmdlink(line);
    reply("OK\nforwarded: %s\n", line);
    return;
  }
  reply("ERR unknown command: %s\n", line);
}

/* ===================== public ========================================= */
void usbhost_init(void)
{
  s_conn_prev = false;
  USBH_DIAG.println("USB-HOST: command channel on the CDC port (CAP? VER? FH R MODE STAT; F o f i W T t s forwarded)");
}

void usbhost_tick(uint32_t now)
{
  /* hello on connect (HWCDC: 'connected' = the host drains the TX FIFO) */
  bool conn = (bool)USBH_RAW;
  if (conn && !s_conn_prev) { send_hello(); s_stat_last_ms = now; }
  s_conn_prev = conn;

  /* host lines */
  static char line[96]; static uint8_t n = 0;
  while (USBH_RAW.available()) {
    char ch = (char)USBH_RAW.read();
    if (ch == '\n' || ch == '\r') { if (n) { line[n] = 0; handle_line(line); } n = 0; }
    else if (n < sizeof line - 1) line[n++] = ch;
    else n = 0;
  }

  /* fine offset after the pending F has gone out (rtl_pending: <= 250 ms) */
  if (s_pending_o_ms && now - s_pending_o_ms >= 400u) {
    char b[16]; snprintf(b, sizeof b, "o%ld", (long)s_pending_o);
    menu_host_cmdlink(b);
    s_pending_o_ms = 0;
  }

  /* periodic status */
  if (conn && s_host_spoke && s_stat_period_ms && now - s_stat_last_ms >= s_stat_period_ms) {
    s_stat_last_ms = now;
    send_status();
  }
}

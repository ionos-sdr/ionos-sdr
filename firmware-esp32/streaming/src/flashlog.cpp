/* SPDX-License-Identifier: MIT
 *
 * flashlog.cpp — persistent (LittleFS) telemetry log
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * See the flashlog.h header. Key principles:
 *   - RAM buffer + rare, batched flush (the flash-cache stall must not
 *     disrupt the block pump),
 *   - the log file is KEPT OPEN (no size-dependent open cost on every
 *     flush),
 *   - the write() result is CHECKED: when full, rotate (no silent freeze),
 *   - logdump is CHUNKED, from the main loop, according to the free space
 *     in the serial TX buffer — so printing does not stop the pump.
 *
 * With FLASHLOG_ENABLE == 0 the WHOLE translation unit is empty: the
 * header replaces the calls with macros, and there is nothing to define
 * here. Thus there are no "defined but unused" functions, and the macros
 * do not collide with the definitions.
 */

#include "flashlog.h"

#if FLASHLOG_ENABLE

#include <Arduino.h>
#include <LittleFS.h>
#include <esp_system.h>
#include <stdarg.h>
#include <string.h>

#define LOG_PATH   "/aprs.log"
#define LOG_OLD    "/aprs.1.log"
#define LOG_MAX    (256u * 1024u)   /* rotate at 256 kB -> 2x256 = 512 kB max,
                                     * well below the default ~1.4 MB partition */
#define RAM_BUF    4096u
#define FLUSH_HI   3072u            /* flush when the buffer grows above this */
#define FLUSH_MS   120000u          /* ...or at least every 2 minutes
                                     *
                                     * Do NOT set this to HOURS "during
                                     * measurements"! The log then accumulates
                                     * in RAM, the heap shrinks, and when it
                                     * finally writes, the stall is EVEN
                                     * larger. For measurements the correct
                                     * solution is FLASHLOG_ENABLE 0. */

/* --- flush-window deadlines (2026-08-07) --------------------------------
 * By default the flush waits for the "safe window" (see
 * flashlog_flush_window in the header). These two deadlines ensure it never
 * gets stuck: if the buffer is almost full, or the last write is too long
 * ago, it writes without a window. Better one stall than a lost log. */
#define FLUSH_HARD_LEN  3840u       /* of the 4096: about one line of room remains */
#define FLUSH_HARD_MS   600000u     /* 10 min without a window -> write */

static char     s_buf[RAM_BUF];
static size_t   s_len = 0;
static uint32_t s_last_flush = 0;
static bool     s_ok = false;
static uint32_t s_dropped = 0;
static File     s_file;             /* log file KEPT OPEN throughout (append) */

/* true = according to the main loop, writing flash is allowed now. Default
 * true, so that callers not using flashlog_flush_window() get the old
 * behaviour. */
static bool     s_window_ok = true;
static uint32_t s_defer_max = 0;    /* longest deferral [ms] — diagnostics */
static uint32_t s_defer_t0  = 0;    /* when the write was first wanted */

static char     s_cmd[16];
static uint8_t  s_cmdn = 0;

/* --- chunked dump state (non-blocking printing from the main loop) --- */
static bool     s_dumping = false;
static File     s_dump;
static uint8_t  s_dump_which = 0;   /* 0 = LOG_OLD, 1 = LOG_PATH */

void flashlog_flush_window(bool safe_now)
{
  s_window_ok = safe_now;
}

/* ---- internal: RAM buffer -> flash ---- */
static void do_flush(void)
{
  if (!s_ok || s_len == 0 || !s_file || s_dumping) return;

  /* --- flush window (2026-08-07) ---
   * The flash write disables the cache and halts flash-resident code on
   * BOTH cores — including the dedicated SPI task. Therefore we write only
   * when, according to the main loop, the receive ring has just drained:
   * then all NQUEUE transactions are queued, so the full 123 ms reserve is
   * available. Without a window we wait — but not indefinitely
   * (FLUSH_HARD_LEN / FLUSH_HARD_MS). */
  if (!s_window_ok) {
    if (s_defer_t0 == 0) s_defer_t0 = millis();
    bool must = (s_len >= FLUSH_HARD_LEN) ||
                ((millis() - s_last_flush) >= FLUSH_HARD_MS);
    if (!must) return;
  }
  if (s_defer_t0) {
    uint32_t d = millis() - s_defer_t0;
    if (d > s_defer_max) s_defer_max = d;
    s_defer_t0 = 0;
  }

  uint32_t t0 = millis();
  size_t   w  = s_file.write((const uint8_t *)s_buf, s_len);
  s_file.flush();
  uint32_t dur = millis() - t0;
  bool full = (w < s_len);           /* short write = full / error */
  s_len = 0;
  s_last_flush = millis();

  /* rotation: file full OR above the maximum. Deleting the old rotated
   * file frees space -> the log NEVER stops silently. */
  if (full || s_file.size() > LOG_MAX) {
    s_file.close();
    LittleFS.remove(LOG_OLD);
    LittleFS.rename(LOG_PATH, LOG_OLD);
    s_file = LittleFS.open(LOG_PATH, "a");
    flashlog_printf(full ? "ROTATE (full, w=%u)" : "ROTATE (%u)", (unsigned)w);
  }

  /* The deferral is logged too: it shows how long the safe window had to
   * be waited for. If this is regularly close to FLUSH_HARD_MS, the ring
   * practically never drains -> the main loop is overloaded. */
  if (dur >= 3) flashlog_printf("FLUSH %lums (deferral max %lums)",
                                (unsigned long)dur, (unsigned long)s_defer_max);
}

void flashlog_printf(const char *fmt, ...)
{
  if (!s_ok) return;                 /* without a mount do not even buffer */
  char line[224];
  int n = snprintf(line, sizeof line, "%lu ", (unsigned long)millis());
  if (n < 0) return;
  va_list ap;
  va_start(ap, fmt);
  int m = vsnprintf(line + n, sizeof line - (size_t)n, fmt, ap);
  va_end(ap);
  if (m < 0) return;
  n += m;
  if (n > (int)sizeof line - 2) n = (int)sizeof line - 2;
  line[n++] = '\n';

  if (s_len + (size_t)n >= RAM_BUF) { s_dropped++; return; }
  memcpy(s_buf + s_len, line, (size_t)n);
  s_len += (size_t)n;
}

void flashlog_init(void)
{
  s_ok = LittleFS.begin(true);
  if (!s_ok) {
    Serial0.println("flashlog: LittleFS mount ERROR (partition? board_build.filesystem=littlefs?)");
    return;
  }
  s_file = LittleFS.open(LOG_PATH, "a");
  if (!s_file) {
    Serial0.println("flashlog: cannot open the log file");
    s_ok = false;
    return;
  }
  s_last_flush = millis();
  Serial0.printf("flashlog: LittleFS OK, stored log %lu B  "
                 "(commands: logdump / logclear / logstat)\n",
                 (unsigned long)flashlog_size());
  flashlog_printf("BOOT reset=%d heap=%lu", (int)esp_reset_reason(),
                  (unsigned long)esp_get_free_heap_size());
}

/* --- chunked dump: start + one portion per tick --- */
static void dump_start(void)
{
  if (s_dumping) return;
  s_window_ok = true;                /* before the dump the RAM buffer is
                                      * ALWAYS written out, otherwise the end
                                      * of the log would be missing from the
                                      * output. Streaming is halted here
                                      * anyway (during a dump the tick does
                                      * nothing else). */
  do_flush();
  if (s_file) s_file.close();        /* close so that it can be read */
  Serial0.println("===== FLASHLOG BEGIN =====");
  s_dumping = true;
  s_dump_which = 0;
  s_dump = LittleFS.open(LOG_OLD, "r");   /* the older rotated file first */
}

static void dump_step(void)
{
  /* current source file exhausted? move to the next / close */
  while (!(s_dump && s_dump.available())) {
    if (s_dump) s_dump.close();
    if (s_dump_which == 0) { s_dump_which = 1; s_dump = LittleFS.open(LOG_PATH, "r"); continue; }
    /* done with both */
    s_dumping = false;
    s_file = LittleFS.open(LOG_PATH, "a");      /* reopen to continue */
    Serial0.printf("===== FLASHLOG END (%lu B) =====\n",
                   (unsigned long)flashlog_size());
    return;
  }
  /* write as much as fits into the TX buffer — NEVER block */
  int room = Serial0.availableForWrite();
  if (room <= 0) return;
  uint8_t b[128];
  int want = room < (int)sizeof b ? room : (int)sizeof b;
  int got = s_dump.read(b, (size_t)want);
  if (got > 0) Serial0.write(b, (size_t)got);
}

void flashlog_tick(uint32_t now)
{
  if (!s_ok) return;

  if (s_dumping) { dump_step(); return; }   /* during a dump do nothing else */

  if (s_len >= FLUSH_HI || (s_len && (now - s_last_flush) >= FLUSH_MS))
    do_flush();

  while (Serial0.available()) {
    char c = (char)Serial0.read();
    if (c == '\n' || c == '\r') {
      s_cmd[s_cmdn] = '\0';
      if      (!strcmp(s_cmd, "logdump"))  { dump_start(); }
      else if (!strcmp(s_cmd, "logclear")) { flashlog_clear();
                                             Serial0.println("flashlog: cleared"); }
      else if (!strcmp(s_cmd, "logstat"))  {
        Serial0.printf("flashlog: %lu B stored, %lu B buffered, dropped=%lu, "
                       "longest flush deferral %lu ms\n",
                       (unsigned long)flashlog_size(), (unsigned long)s_len,
                       (unsigned long)s_dropped, (unsigned long)s_defer_max);
      }
      s_cmdn = 0;
    } else if (s_cmdn < sizeof s_cmd - 1) {
      s_cmd[s_cmdn++] = c;
    } else {
      s_cmdn = 0;
    }
  }
}

/* Synchronous full dump (for programmatic use). The serial 'logdump' does
 * NOT use this but the chunked, non-blocking dump_start(). */
void flashlog_dump(Stream &out)
{
  if (!s_ok) { out.println("flashlog: no FS"); return; }
  s_window_ok = true;                /* see dump_start(): the buffer must be written out */
  do_flush();
  if (s_file) s_file.close();
  out.println("===== FLASHLOG BEGIN =====");
  for (int i = 0; i < 2; i++) {
    File f = LittleFS.open(i == 0 ? LOG_OLD : LOG_PATH, "r");
    if (!f) continue;
    uint8_t b[256];
    while (f.available()) { size_t n = f.read(b, sizeof b); out.write(b, n); }
    f.close();
  }
  s_file = LittleFS.open(LOG_PATH, "a");
  out.printf("===== FLASHLOG END (%lu B) =====\n", (unsigned long)flashlog_size());
}

void flashlog_clear(void)
{
  if (!s_ok) return;
  if (s_file) s_file.close();
  LittleFS.remove(LOG_OLD);
  LittleFS.remove(LOG_PATH);
  s_file = LittleFS.open(LOG_PATH, "a");
  s_len = 0;
  s_dropped = 0;
  s_last_flush = millis();
  flashlog_printf("CLEAR heap=%lu", (unsigned long)esp_get_free_heap_size());
}

size_t flashlog_size(void)
{
  if (!s_ok) return 0;                 /* without a mount do not open files */
  size_t total = 0;
  if (s_file) {
    total += s_file.size();
  } else {
    File f = LittleFS.open(LOG_PATH, "r");
    if (f) { total += f.size(); f.close(); }
  }
  File g = LittleFS.open(LOG_OLD, "r");
  if (g) { total += g.size(); g.close(); }
  return total;
}

#endif /* FLASHLOG_ENABLE */
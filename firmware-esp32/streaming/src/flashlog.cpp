/* SPDX-License-Identifier: MIT
 *
 * flashlog.cpp — tartos (LittleFS) telemetria-naplo
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Lasd a flashlog.h fejlecet. Kulcselvek:
 *   - RAM-puffer + ritka, kotegelt flush (a flash-cache stall ne verje szet
 *     a blokk-pumpat),
 *   - a naplofajl VEGIG NYITVA (nincs meret-fuggo open-koltseg minden
 *     flush-nel),
 *   - a write() eredmenyet NEZZUK: ha megtelt, rotalunk (nincs nema befagyas),
 *   - a logdump DARABOLVA, a fo ciklusbol, a soros TX-puffer szabad helye
 *     szerint — igy a kiiratas nem allitja meg a pumpat.
 *
 * FLASHLOG_ENABLE == 0 eseten az EGESZ forditasi egyseg ures: a fejlec
 * makrokra csereli a hivasokat, itt pedig nincs mit definialni. Igy nincs
 * "definialt, de nem hasznalt" fuggveny, es a makrok sem utkoznek a
 * definiciokkal.
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
#define LOG_MAX    (256u * 1024u)   /* rotacio 256 kB-nal -> 2x256 = 512 kB max,
                                     * bosegesen az alap ~1,4 MB partitcio alatt */
#define RAM_BUF    4096u
#define FLUSH_HI   3072u            /* flush, ha a puffer ennyi fole no */
#define FLUSH_MS   120000u          /* ...vagy legalabb 2 percenkent
                                     *
                                     * NE allitsd ORAS ertekre "meres alatt"!
                                     * Akkor a napló addig a RAM-ban gyulik, a
                                     * heap fogy, es amikor vegre ir, MEG
                                     * nagyobbat ut. Meresre a helyes megoldas
                                     * a FLASHLOG_ENABLE 0. */

/* --- flush-ablak hataridok (2026-08-07) ---------------------------------
 * A flush alapesetben megvarja a "biztonsagos ablakot" (lasd
 * flashlog_flush_window a fejlecben). Ez a ket hatarido gondoskodik arrol,
 * hogy sose ragadjon be: ha a puffer majdnem tele van, vagy tul rég irtunk,
 * akkor ablak nelkul is kiirunk. Inkabb egy akadas, mint elveszett naplo. */
#define FLUSH_HARD_LEN  3840u       /* a 4096-bol: kb. egy sornyi hely marad */
#define FLUSH_HARD_MS   600000u     /* 10 perc ablak nelkul -> irunk */

static char     s_buf[RAM_BUF];
static size_t   s_len = 0;
static uint32_t s_last_flush = 0;
static bool     s_ok = false;
static uint32_t s_dropped = 0;
static File     s_file;             /* VEGIG NYITVA tartott naplofajl (append) */

/* true = a fo ciklus szerint most szabad flashbe irni. Alapbol true, hogy
 * aki nem hivja a flashlog_flush_window()-t, a regi viselkedest kapja. */
static bool     s_window_ok = true;
static uint32_t s_defer_max = 0;    /* leghosszabb halasztas [ms] — diagnosztika */
static uint32_t s_defer_t0  = 0;    /* mikor akartunk eloszor irni */

static char     s_cmd[16];
static uint8_t  s_cmdn = 0;

/* --- darabolt dump allapota (nem-blokkolo kiiratas a fo ciklusbol) --- */
static bool     s_dumping = false;
static File     s_dump;
static uint8_t  s_dump_which = 0;   /* 0 = LOG_OLD, 1 = LOG_PATH */

void flashlog_flush_window(bool safe_now)
{
  s_window_ok = safe_now;
}

/* ---- belso: RAM-puffer -> flash ---- */
static void do_flush(void)
{
  if (!s_ok || s_len == 0 || !s_file || s_dumping) return;

  /* --- flush-ablak (2026-08-07) ---
   * A flash-iras kikapcsolja a cache-t, es MINDKET magon megallitja a
   * flashben futo kodot — a dedikalt SPI-taskot is. Ezert csak akkor
   * irunk, ha a fo ciklus szerint epp kiurult a vetel-gyuru: olyankor
   * mind a NQUEUE tranzakcio fel van fuzve, tehat a teljes 123 ms
   * tartalek all rendelkezesre. Ha nincs ablak, varunk — de nem
   * vegtelenul (FLUSH_HARD_LEN / FLUSH_HARD_MS). */
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
  bool full = (w < s_len);           /* rovid iras = megtelt / hiba */
  s_len = 0;
  s_last_flush = millis();

  /* rotacio: megtelt fajl VAGY tullepte a maximumot. A regi rotalt torlese
   * helyet szabadit -> a naplo SOSEM all le csendben. */
  if (full || s_file.size() > LOG_MAX) {
    s_file.close();
    LittleFS.remove(LOG_OLD);
    LittleFS.rename(LOG_PATH, LOG_OLD);
    s_file = LittleFS.open(LOG_PATH, "a");
    flashlog_printf(full ? "ROTATE (megtelt, w=%u)" : "ROTATE (%u)", (unsigned)w);
  }

  /* A halasztast is naplozzuk: ebbol latszik, mennyit kellett varni a
   * biztonsagos ablakra. Ha ez rendszeresen a FLUSH_HARD_MS kozeleben van,
   * akkor a gyuru gyakorlatilag sosem urul ki -> tulterhelt a fo ciklus. */
  if (dur >= 3) flashlog_printf("FLUSH %lums (halasztas max %lums)",
                                (unsigned long)dur, (unsigned long)s_defer_max);
}

void flashlog_printf(const char *fmt, ...)
{
  if (!s_ok) return;                 /* mount nelkul ne is pufferelj */
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
    Serial0.println("flashlog: LittleFS mount HIBA (partitcio? board_build.filesystem=littlefs?)");
    return;
  }
  s_file = LittleFS.open(LOG_PATH, "a");
  if (!s_file) {
    Serial0.println("flashlog: a naplofajl nem nyithato");
    s_ok = false;
    return;
  }
  s_last_flush = millis();
  Serial0.printf("flashlog: LittleFS OK, tarolt naplo %lu B  "
                 "(parancs: logdump / logclear / logstat)\n",
                 (unsigned long)flashlog_size());
  flashlog_printf("BOOT reset=%d heap=%lu", (int)esp_reset_reason(),
                  (unsigned long)esp_get_free_heap_size());
}

/* --- darabolt dump: elinditas + egy-egy adag a tickbol --- */
static void dump_start(void)
{
  if (s_dumping) return;
  s_window_ok = true;                /* a dump elott MINDENKEPP kiirjuk a
                                      * RAM-puffert, kulonben a naplo vege
                                      * hianyozna a kiiratasbol. Itt ugyis
                                      * all a streamelés (dump alatt a tick
                                      * mast nem csinal). */
  do_flush();
  if (s_file) s_file.close();        /* zarjuk, hogy olvashato legyen */
  Serial0.println("===== FLASHLOG KEZDET =====");
  s_dumping = true;
  s_dump_which = 0;
  s_dump = LittleFS.open(LOG_OLD, "r");   /* elobb a regebbi rotalt */
}

static void dump_step(void)
{
  /* aktualis forrasfajl kimerult? lepj a kovetkezore / zarj */
  while (!(s_dump && s_dump.available())) {
    if (s_dump) s_dump.close();
    if (s_dump_which == 0) { s_dump_which = 1; s_dump = LittleFS.open(LOG_PATH, "r"); continue; }
    /* kesz mindkettovel */
    s_dumping = false;
    s_file = LittleFS.open(LOG_PATH, "a");      /* folytatashoz ujranyitjuk */
    Serial0.printf("===== FLASHLOG VEGE (%lu B) =====\n",
                   (unsigned long)flashlog_size());
    return;
  }
  /* annyit irunk, amennyi a TX-pufferbe fer — SOHA nem blokkolunk */
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

  if (s_dumping) { dump_step(); return; }   /* dump alatt csak azt csinaljuk */

  if (s_len >= FLUSH_HI || (s_len && (now - s_last_flush) >= FLUSH_MS))
    do_flush();

  while (Serial0.available()) {
    char c = (char)Serial0.read();
    if (c == '\n' || c == '\r') {
      s_cmd[s_cmdn] = '\0';
      if      (!strcmp(s_cmd, "logdump"))  { dump_start(); }
      else if (!strcmp(s_cmd, "logclear")) { flashlog_clear();
                                             Serial0.println("flashlog: torolve"); }
      else if (!strcmp(s_cmd, "logstat"))  {
        Serial0.printf("flashlog: %lu B tarolva, %lu B pufferben, eldobott=%lu, "
                       "leghosszabb flush-halasztas %lu ms\n",
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

/* Szinkron teljes dump (programozott hasznalatra). A soros 'logdump' NEM ezt
 * hasznalja, hanem a darabolt, nem-blokkolo dump_start()-ot. */
void flashlog_dump(Stream &out)
{
  if (!s_ok) { out.println("flashlog: nincs FS"); return; }
  s_window_ok = true;                /* lasd dump_start(): a puffert ki kell irni */
  do_flush();
  if (s_file) s_file.close();
  out.println("===== FLASHLOG KEZDET =====");
  for (int i = 0; i < 2; i++) {
    File f = LittleFS.open(i == 0 ? LOG_OLD : LOG_PATH, "r");
    if (!f) continue;
    uint8_t b[256];
    while (f.available()) { size_t n = f.read(b, sizeof b); out.write(b, n); }
    f.close();
  }
  s_file = LittleFS.open(LOG_PATH, "a");
  out.printf("===== FLASHLOG VEGE (%lu B) =====\n", (unsigned long)flashlog_size());
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
  if (!s_ok) return 0;                 /* mount nelkul ne nyitogass */
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
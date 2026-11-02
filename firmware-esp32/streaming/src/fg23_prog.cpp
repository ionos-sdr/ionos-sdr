/* SPDX-License-Identifier: MIT
 *
 * fg23_prog.cpp - program the FG23 radio-board flash from the ESP32-S3 over SWD
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Verified on the ionos-sdr-mini 2026-11-02 (106 kB iq_capture, ~10 s program
 * + verify). The SWD/MSC layer is fg23_swd.h; this file is the console
 * protocol around it.
 */
#include "fg23_prog.h"
#include "board.h"

#if BOARD_IONOS_MINI
#include "fg23_swd.h"
#include "esp_rom_crc.h"

static bool rd_exact(HardwareSerial &io, uint8_t *p, size_t n, uint32_t tmo_ms)
{
  uint32_t t0 = millis(); size_t got = 0;
  while (got < n) {
    int a = io.available();
    if (a > 0) { got += io.readBytes(p + got, min((size_t)a, n - got)); t0 = millis(); }
    else if (millis() - t0 > tmo_ms) return false;
    else delay(1);
  }
  return true;
}

static void info(HardwareSerial &io)
{
  uint32_t dpidr = 0, apidr = 0, v[4] = {0}, dh = 0;
  io.printf("ESP heap: SPIRAM free %u largest %u | internal free %u largest %u\n",
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  bool c = swd::connect(&dpidr, &apidr);
  io.printf("FG23 SWD %s  DPIDR %08lX  AP0 IDR %08lX\n", c ? "OK" : "FAIL",
            (unsigned long)dpidr, (unsigned long)apidr);
  if (c) {
    for (int i = 0; i < 4; i++) swd::mem_r(0x0FE08000 + 4 * i, &v[i]);
    io.printf("FG23 DEVINFO %08lX %08lX %08lX %08lX\n", (unsigned long)v[0], (unsigned long)v[1],
              (unsigned long)v[2], (unsigned long)v[3]);
    for (int i = 0; i < 2; i++) swd::mem_r(0x08000000 + 4 * i, &v[i]);
    swd::mem_r(swd::DHCSR, &dh);
    io.printf("FG23 vectors SP %08lX PC %08lX  DHCSR %08lX\n", (unsigned long)v[0],
              (unsigned long)v[1], (unsigned long)dh);
  }
  swd::release();
}

static void program(HardwareSerial &io, uint32_t addr, uint32_t len, uint32_t crc)
{
  if (addr < swd::FLASH_BASE || addr + len > swd::FLASH_BASE + swd::FLASH_SIZE || (addr & 3) || !len) {
    io.println("ERR range"); return;
  }
  uint32_t plen = (len + 3) & ~3u;
  /* PSRAM through the IDF allocator: Arduino's ps_malloc() needs
   * BOARD_HAS_PSRAM, which this env does not define (PSRAM comes from
   * custom_sdkconfig). */
  uint8_t  *buf = (uint8_t *)heap_caps_malloc(plen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  uint32_t *rb  = (uint32_t *)heap_caps_malloc(swd::PAGE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!rb) rb = (uint32_t *)malloc(swd::PAGE);
  if (!buf || !rb) { io.println("ERR alloc"); free(buf); free(rb); return; }
  memset(buf, 0xFF, plen);
  io.println("READY");
  for (uint32_t o = 0; o < len; o += 2048) {
    uint32_t n = min<uint32_t>(2048, len - o);
    if (!rd_exact(io, buf + o, n, 3000)) { io.printf("ERR data timeout at %lu\n", (unsigned long)o); free(buf); free(rb); return; }
    io.write('#'); io.flush();
  }
  uint32_t c = esp_rom_crc32_le(0, buf, len);
  io.printf("GOT %lu bytes crc %08lX %s\n", (unsigned long)len, (unsigned long)c, c == crc ? "ok" : "MISMATCH");
  if (c != crc) { free(buf); free(rb); return; }

  uint32_t t0 = millis(), tries = 0;
  const char *err = nullptr;
  if (!swd::halt_after_reset(BRD_SWD_RESETN, &tries)) { err = "halt"; goto out; }
  io.printf("halted at reset vector (%lu)\n", (unsigned long)tries);
  if (uint32_t r = swd::msc_begin()) { io.printf("ERR msc %08lX\n", (unsigned long)r); goto done; }
  {
    uint32_t first = addr & ~(swd::PAGE - 1), last = (addr + plen - 1) & ~(swd::PAGE - 1);
    for (uint32_t p = first; p <= last; p += swd::PAGE) {
      uint32_t r = swd::erase_page(p), w0 = 0, w1 = 0;
      swd::mem_r(p, &w0); swd::mem_r(p + swd::PAGE - 4, &w1);
      if (r || w0 != 0xFFFFFFFF || w1 != 0xFFFFFFFF) {
        io.printf("ERR erase %08lX st %08lX (%08lX %08lX)\n", (unsigned long)p, (unsigned long)r,
                  (unsigned long)w0, (unsigned long)w1);
        goto done;
      }
    }
    uint32_t a = addr, left = plen / 4; const uint32_t *w = (const uint32_t *)buf;
    while (left) {
      uint32_t n = min<uint32_t>(left, (swd::PAGE - (a & (swd::PAGE - 1))) / 4);
      if (uint32_t r = swd::write_burst(a, w, n)) { io.printf("ERR write %08lX st %08lX\n", (unsigned long)a, (unsigned long)r); goto done; }
      a += n * 4; w += n; left -= n;
    }
    swd::msc_end();
    a = addr; w = (const uint32_t *)buf; left = plen / 4; uint32_t bad = 0;
    while (left) {
      uint32_t n = min<uint32_t>(left, swd::PAGE / 4);
      if (!swd::read_block(a, rb, n)) { err = "verify read"; goto out; }
      for (uint32_t k = 0; k < n; k++) if (rb[k] != w[k]) bad++;
      a += n * 4; w += n; left -= n;
    }
    if (bad) { io.printf("ERR verify %lu words differ\n", (unsigned long)bad); goto done; }
    swd::run_from_reset();
    io.printf("DONE OK %lu bytes, %lu ms\n", (unsigned long)len, (unsigned long)(millis() - t0));
  }
done:
  swd::release(); free(buf); free(rb); return;
out:
  io.printf("ERR %s\n", err);
  swd::release(); free(buf); free(rb);
}

bool fg23_prog_cmd(const char *line, HardwareSerial &io)
{
  if (!strcmp(line, "FGINFO")) { info(io); return true; }
  unsigned long a, l, c;
  if (!strncmp(line, "FGFW ", 5) && sscanf(line + 5, "%lx %lu %lx", &a, &l, &c) == 3) {
    program(io, (uint32_t)a, (uint32_t)l, (uint32_t)c);
    return true;
  }
  return false;
}

#else
bool fg23_prog_cmd(const char *, HardwareSerial &) { return false; }
#endif

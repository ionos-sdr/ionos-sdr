// SWD host (bit-bang) + EFR32 Series 2 (xG23) flash programmer for the ionos-sdr-mini cradle.
// SWCLK = GPIO39, SWDIO = GPIO38 (both >= 32 -> GPIO "1" register bank).
// MSC sequence follows emlib em_msc.c (MSC_ErasePage / writeBurst) of the Simplicity SDK.
#pragma once
#include <Arduino.h>
#include "soc/gpio_struct.h"

namespace swd {

static const int PIN_CLK = 39, PIN_IO = 38;
static const uint32_t M_CLK = 1u << (PIN_CLK - 32), M_IO = 1u << (PIN_IO - 32);

static inline void dly() { for (volatile int i = 0; i < 6; i++) {} }
static inline void clk_lo() { GPIO.out1_w1tc.val = M_CLK; }
static inline void clk_hi() { GPIO.out1_w1ts.val = M_CLK; }
static inline void io_out() { GPIO.enable1_w1ts.val = M_IO; }
static inline void io_in()  { GPIO.enable1_w1tc.val = M_IO; }
static inline void io_set(int b) { if (b) GPIO.out1_w1ts.val = M_IO; else GPIO.out1_w1tc.val = M_IO; }
static inline int  io_get() { return (GPIO.in1.val & M_IO) ? 1 : 0; }

static void wbits(uint32_t v, int n) { io_out(); for (int i = 0; i < n; i++) { io_set((v >> i) & 1); clk_lo(); dly(); clk_hi(); dly(); } }
static uint32_t rbits(int n) { io_in(); uint32_t v = 0; for (int i = 0; i < n; i++) { clk_lo(); dly(); v |= (uint32_t)io_get() << i; clk_hi(); dly(); } return v; }
static void turn() { io_in(); clk_lo(); dly(); clk_hi(); dly(); }

enum { OK = 1, WAIT = 2, FAULT = 4 };
static uint32_t last_ack = 0, n_wait = 0, n_err = 0;

// one SWD transfer; addr = 0x0/0x4/0x8/0xC
static bool xfer(bool ap, bool rnw, uint8_t addr, uint32_t *data) {
  for (int tries = 0; tries < 200; tries++) {
    uint32_t a2 = (addr >> 2) & 1, a3 = (addr >> 3) & 1;
    uint32_t par = (ap ? 1 : 0) ^ (rnw ? 1 : 0) ^ a2 ^ a3;
    uint32_t req = 1 | (ap ? 2 : 0) | (rnw ? 4 : 0) | (a2 << 3) | (a3 << 4) | (par << 5) | (0 << 6) | (1u << 7);
    wbits(req, 8);
    turn();
    uint32_t ack = rbits(3);
    last_ack = ack;
    if (ack == OK) {
      if (rnw) {
        uint32_t v = rbits(32), p = rbits(1);
        turn();
        wbits(0, 2);
        if ((uint32_t)(__builtin_popcount(v) & 1) != p) { n_err++; return false; }
        *data = v;
      } else {
        turn();
        uint32_t v = *data;
        wbits(v, 32); wbits(__builtin_popcount(v) & 1, 1);
        wbits(0, 2);
      }
      return true;
    }
    turn();                       // no data phase on WAIT / FAULT
    wbits(0, 2);
    if (ack == WAIT) { n_wait++; continue; }
    n_err++;
    return false;
  }
  return false;
}

static bool dp_r(uint8_t a, uint32_t *v) { return xfer(false, true, a, v); }
static bool dp_w(uint8_t a, uint32_t v) { return xfer(false, false, a, &v); }
static bool ap_w(uint8_t a, uint32_t v) { return xfer(true, false, a, &v); }
static bool ap_r(uint8_t a, uint32_t *v) { uint32_t d; return xfer(true, true, a, &d) && dp_r(0xC, v); }  // posted + RDBUFF

static void clear_errors() { dp_w(0x0, 0x1E); }

static uint32_t cur_csw = 0, cur_tar = 0xFFFFFFFF;
static bool set_csw(uint32_t c) { if (c == cur_csw) return true; if (!ap_w(0x0, c)) return false; cur_csw = c; return true; }

static bool mem_w(uint32_t addr, uint32_t v) {
  if (!set_csw(0x23000002)) return false;
  if (!ap_w(0x4, addr)) return false;
  return ap_w(0xC, v);
}
static bool mem_r(uint32_t addr, uint32_t *v) {
  if (!set_csw(0x23000002)) return false;
  if (!ap_w(0x4, addr)) return false;
  return ap_r(0xC, v);
}

static bool connect(uint32_t *dpidr, uint32_t *apidr) {
  pinMode(PIN_CLK, OUTPUT); pinMode(PIN_IO, OUTPUT); clk_hi();
  wbits(0xFFFFFFFF, 32); wbits(0xFFFFFFFF, 24);
  wbits(0xE79E, 16);
  wbits(0xFFFFFFFF, 32); wbits(0xFFFFFFFF, 24);
  wbits(0, 8);
  if (!dp_r(0x0, dpidr)) return false;
  clear_errors();
  dp_w(0x8, 0);                                   // SELECT AP0 bank 0
  dp_w(0x4, 0x50000000);                          // CSYSPWRUPREQ | CDBGPWRUPREQ
  uint32_t cs = 0;
  for (int i = 0; i < 100; i++) { dp_r(0x4, &cs); if ((cs & 0xA0000000) == 0xA0000000) break; delay(1); }
  if ((cs & 0xA0000000) != 0xA0000000) return false;
  dp_w(0x8, 0xF0);                                // AP0 bank 0xF -> IDR
  ap_r(0xC, apidr);
  dp_w(0x8, 0);
  cur_csw = 0;
  return true;
}

static void release() { pinMode(PIN_IO, INPUT); pinMode(PIN_CLK, INPUT); }

// ---- Cortex-M33 core control ----
static const uint32_t DHCSR = 0xE000EDF0, DEMCR = 0xE000EDFC, AIRCR = 0xE000ED0C;
static bool halt() {
  if (!mem_w(DHCSR, 0xA05F0003)) return false;
  uint32_t v = 0;
  for (int i = 0; i < 100; i++) { if (mem_r(DHCSR, &v) && (v & (1u << 17))) return true; delay(1); }
  return false;
}
// A running FG23 app may sit in EM2 (AHB unpowered): AP reads then return 0 and
// writes vanish (first seen 2026-11-02 when re-flashing over a live iq_capture).
// So: pulse RESETn, catch the core right after boot, set VC_CORERESET, and issue
// a system reset -> the core halts on its reset vector with every peripheral at
// reset state. Success is proven by reading DEVINFO MSIZE (non-zero) and S_HALT.
static bool sticky_err() { uint32_t cs = 0; dp_r(0x4, &cs); if (cs & 0x20) { clear_errors(); return true; } return false; }
static bool halt_after_reset(int resetn_pin, uint32_t *tries_out) {
  // Measured 2026-11-02: after a RESETn pulse the DP answers at ~2 ms but every
  // AHB-AP access FAULTs until the Secure Element has booted (~10-20 ms); a
  // running app can be halted directly. So: connect to the running core, halt
  // it, arm VC_CORERESET and request a system reset -> the core stops on its
  // reset vector after the SE boot, before any app code (CMU/MSC at reset state).
  (void)resetn_pin;
  uint32_t dpidr, apidr, v = 0, msize = 0;
  int n = 0;
  uint32_t t0 = millis();
  bool halted = false;
  while (millis() - t0 < 300) {                       // 1) halt the running core
    n++;
    if (!connect(&dpidr, &apidr)) { delay(2); continue; }
    if (halt()) { halted = true; break; }
    clear_errors(); delay(2);
  }
  if (!halted) return false;
  mem_w(DEMCR, 0x00000001);                            // 2) VC_CORERESET
  mem_w(AIRCR, 0x05FA0004);                            //    SYSRESETREQ
  t0 = millis();
  while (millis() - t0 < 300) {                       // 3) wait for the SE boot, re-attach
    delay(2); n++;
    if (!connect(&dpidr, &apidr)) continue;
    if (!mem_r(DHCSR, &v)) { clear_errors(); continue; }
    if (!(v & (1u << 17))) continue;
    if (!mem_r(0x0FE0800C, &msize) || msize == 0 || msize == 0xFFFFFFFF) { clear_errors(); continue; }
    if (tries_out) *tries_out = n;
    return true;
  }
  // fallback: halt wherever it is now (peripherals NOT at reset state)
  if (connect(&dpidr, &apidr) && halt()) { if (tries_out) *tries_out = 1000 + n; return true; }
  return false;
}

static void run_from_reset() {
  mem_w(DEMCR, 0);
  mem_w(DHCSR, 0xA05F0000);                       // clear C_HALT / C_DEBUGEN
  mem_w(AIRCR, 0x05FA0004);                       // SYSRESETREQ
}

// ---- EFR32xG23 MSC (secure aliases) ----
static const uint32_t MSC = 0x40030000, MSC_SET = MSC + 0x1000, MSC_CLR = MSC + 0x2000;
static const uint32_t WRITECTRL = 0x0C, WRITECMD = 0x10, ADDRB = 0x14, WDATA = 0x18, STATUS = 0x1C, LOCK = 0x3C;
static const uint32_t ST_BUSY = 1u << 0, ST_LOCKED = 1u << 1, ST_INVADDR = 1u << 2, ST_WDATAREADY = 1u << 3,
                      ST_PENDING = 1u << 5, ST_REGLOCK = 1u << 16;
static const uint32_t CMU_CLKEN1_SET = 0x40008068 + 0x1000, CLKEN1_MSC = 1u << 16;
static const uint32_t PAGE = 0x2000, FLASH_BASE = 0x08000000, FLASH_SIZE = 0x80000;

// returns 0 ok, else STATUS bits that went wrong (bit31 = SWD error, bit30 = timeout)
static uint32_t msc_wait(uint32_t mask, uint32_t value) {
  uint32_t st = 0;
  for (int i = 0; i < 20000; i++) {
    if (!mem_r(MSC + STATUS, &st)) return 0x80000000;
    if (st & ST_INVADDR) return st | 0x100;
    if ((st & mask) == value) return (st & (ST_LOCKED | ST_REGLOCK)) ? (st | 0x200) : 0;
  }
  return 0x40000000 | st;
}

static uint32_t msc_begin() {
  mem_w(0x40008010, 0x93F7);                      // CMU LOCK <- unlock key
  if (!mem_w(CMU_CLKEN1_SET, CLKEN1_MSC)) return 0x80000000;
  uint32_t ipv = 0;
  if (!mem_r(MSC + 0x00, &ipv)) return 0x80000000;  // MSC IPVERSION must read non-zero
  if (ipv == 0) return 0x20000000;
  mem_w(MSC + LOCK, 0x1B71);
  mem_w(MSC_SET + WRITECTRL, 1);                  // WREN
  return 0;
}
static void msc_end() { mem_w(MSC_CLR + WRITECTRL, 1); }

static uint32_t erase_page(uint32_t a) {
  mem_w(MSC + ADDRB, a);
  mem_w(MSC + WRITECMD, 1u << 1);                 // ERASEPAGE
  uint32_t r = msc_wait(ST_BUSY | ST_PENDING, 0);
  if (!r) r = msc_wait(ST_BUSY | ST_PENDING, 0);
  return r;
}

// program words inside one page (emlib writeBurst)
static uint32_t write_burst(uint32_t a, const uint32_t *w, uint32_t nwords) {
  mem_w(MSC + ADDRB, a);
  uint32_t st = 0; mem_r(MSC + STATUS, &st);
  if (st & ST_INVADDR) return st | 0x100;
  mem_w(MSC + WDATA, *w++); nwords--;
  while (nwords) {
    uint32_t r = msc_wait(ST_WDATAREADY, ST_WDATAREADY);
    if (r) { mem_w(MSC + WRITECMD, 1u << 2); return r; }
    mem_w(MSC + WDATA, *w++); nwords--;
  }
  mem_w(MSC + WRITECMD, 1u << 2);                 // WRITEEND
  uint32_t r = msc_wait(ST_BUSY | ST_PENDING, 0);
  if (!r) r = msc_wait(ST_BUSY | ST_PENDING, 0);
  return r;
}

// block read with auto-increment (TAR wraps every 1 kB -> reload)
static bool read_block(uint32_t a, uint32_t *out, uint32_t nwords) {
  if (!set_csw(0x23000012)) return false;
  uint32_t i = 0;
  while (i < nwords) {
    uint32_t chunk = min<uint32_t>(nwords - i, (0x400 - ((a + i * 4) & 0x3FF)) / 4);
    if (!ap_w(0x4, a + i * 4)) return false;
    uint32_t d;
    if (!xfer(true, true, 0xC, &d)) return false;              // posted, discard
    for (uint32_t k = 0; k < chunk; k++) {
      if (k + 1 < chunk) { if (!xfer(true, true, 0xC, &out[i + k])) return false; }
      else { if (!dp_r(0xC, &out[i + k])) return false; }
    }
    i += chunk;
  }
  set_csw(0x23000002);
  return true;
}

}  // namespace swd

/* SPDX-License-Identifier: MIT
 *
 * aprs_beacon.c — AX.25 UI keret -> NRZI bitfolyam
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * Lanc: cimmezok+control+PID+info -> FCS (CRC-16/X.25) -> HDLC keretezes
 * (flag 0x7E) -> bit-stuffing (5 egyes utan beszurt 0) -> NRZI.
 * A bitek LSB-first mennek ki, ahogy az AX.25 elvarja.
 *
 * A dekodolo oldal ellenorzese: az aprs_fm_demod.py mar tudja a
 * fordicottjat (flag-vadaszat, destuff, LSB-first bajtok, cimmezo).
 */

#include "aprs_beacon.h"
#include <string.h>

/* --- CRC-16/X.25 (reflektalt 0x1021, init 0xFFFF, kimenet invertalt) --- */
static uint16_t ax25_fcs(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
  }
  return ~crc;
}

/* Egy cimmezo (6 kar + SSID) betoltese AX.25 formaban: minden karakter
 * 1 bittel balra tolva; az utolso bajt LSB-je a HDLC extension bit. */
static int put_addr(uint8_t *p, const char *call, uint8_t ssid, bool last)
{
  for (int i = 0; i < 6; ++i) {
    char c = (i < (int)strlen(call)) ? call[i] : ' ';
    p[i] = (uint8_t)(c << 1);
  }
  /* SSID bajt: 0110 SSID(4) [ext]; a ket felso bit (C/R, reserved) 1 */
  p[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1) | (last ? 1 : 0));
  return 7;
}

bool aprs_build_ui(aprs_frame_t *out,
                   const char *src, uint8_t ssid,
                   const char *dst, uint8_t dssid,
                   const char *via, uint8_t vssid,
                   const char *info)
{
  uint8_t f[330];
  uint16_t n = 0;
  bool has_via = (via != NULL && via[0] != '\0');

  /* --- cimmezo: cel, forras, majd opcionalis via (digipeater).
   * Az extension bit (LSB=1) CSAK az utolso cimen all.
   * A cel C-bitje 1 (parancs-keret, ahogy a valodi TNC-k adjak),
   * a tobbi cimen 0. A via H-bitje (bit7) 0 = meg nem ismetelt. --- */
  n += put_addr(&f[n], dst, dssid, false);
  f[6] |= 0x80;                              /* dest C-bit = 1 */
  n += put_addr(&f[n], src, ssid, !has_via);
  if (has_via) {
    n += put_addr(&f[n], via, vssid, true);  /* utolso cim: ext=1 */
  }

  f[n++] = 0x03;   /* control: UI keret */
  f[n++] = 0xF0;   /* PID: nincs 3. reteg */

  for (const char *s = info; *s && n < sizeof(f) - 2; ++s)
    f[n++] = (uint8_t)*s;

  /* --- FCS a cimmezotol az info vegeig --- */
  uint16_t fcs = ax25_fcs(f, n);
  f[n++] = (uint8_t)(fcs & 0xFF);
  f[n++] = (uint8_t)(fcs >> 8);

  /* --- HDLC: flag | stuffelt adat | flag, majd NRZI ---
   * A bitfolyamot LSB-first rakjuk ossze; a flageket NEM stuffeljuk. */
  uint8_t nrzi = 1;           /* NRZI allapot (kezdeti szint) */
  uint16_t ob = 0;
  int ones = 0;

  #define EMIT_NRZI(bit) do {                          \
      if ((bit) == 0) nrzi ^= 1;  /* 0 = szintvaltas */ \
      if (ob < sizeof(out->bits)) out->bits[ob++] = nrzi; \
    } while (0)

  #define EMIT_FLAG() do {                             \
      /* 0x7E = 0 1111110, LSB-first: 0,1,1,1,1,1,1,0 */ \
      static const uint8_t fl[8] = {0,1,1,1,1,1,1,0};   \
      for (int k = 0; k < 8; ++k) EMIT_NRZI(fl[k]);     \
    } while (0)

  /* Kezdo idle-szint: a dekodolo elso differenciajanak kell egy
   * referencia, kulonben az elso bit elveszik. */
  if (ob < sizeof(out->bits)) out->bits[ob++] = nrzi;

  /* Preamble-flagek: a valodi TNC-k 150-300 ms-nyit kuldenek, hogy az
   * FM-vevo ZAJZARA kinyisson es a bit-szinkron bealljon, MIELOTT az
   * adat jon. 25 flag = 200 bit = ~167 ms 1200 baudon. (A korabbi 4
   * flag = 27 ms keves volt: mire a zajzar nyitott, a keret eleje
   * mar elment.) */
  #define APRS_PREAMBLE_FLAGS 25
  for (int pf = 0; pf < APRS_PREAMBLE_FLAGS; ++pf) EMIT_FLAG();

  for (uint16_t i = 0; i < n; ++i) {
    for (int b = 0; b < 8; ++b) {          /* LSB-first */
      uint8_t bit = (f[i] >> b) & 1;
      EMIT_NRZI(bit);
      if (bit) {
        if (++ones == 5) { EMIT_NRZI(0); ones = 0; }  /* stuff */
      } else {
        ones = 0;
      }
    }
  }

  EMIT_FLAG();

  out->nbits = ob;
  return ob <= sizeof(out->bits);

  #undef EMIT_NRZI
  #undef EMIT_FLAG
}

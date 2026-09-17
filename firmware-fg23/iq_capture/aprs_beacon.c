/* SPDX-License-Identifier: MIT
 *
 * aprs_beacon.c — AX.25 UI frame -> NRZI bit stream
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * Chain: address fields + control + PID + info -> FCS (CRC-16/X.25) ->
 * HDLC framing (flag 0x7E) -> bit stuffing (a 0 inserted after 5 ones)
 * -> NRZI. Bits go out LSB-first, as AX.25 expects.
 *
 * Decoder-side check: aprs_fm_demod.py already implements the inverse
 * (flag hunting, destuffing, LSB-first bytes, address field).
 */

#include "aprs_beacon.h"
#include <string.h>

/* --- CRC-16/X.25 (reflected 0x1021, init 0xFFFF, output inverted) --- */
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

/* Load one address field (6 chars + SSID) in AX.25 form: every character
 * shifted left by 1 bit; the LSB of the last byte is the HDLC extension bit. */
static int put_addr(uint8_t *p, const char *call, uint8_t ssid, bool last)
{
  for (int i = 0; i < 6; ++i) {
    char c = (i < (int)strlen(call)) ? call[i] : ' ';
    p[i] = (uint8_t)(c << 1);
  }
  /* SSID byte: 0110 SSID(4) [ext]; the two upper bits (C/R, reserved) are 1 */
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

  /* --- address field: destination, source, then optional via (digipeater).
   * The extension bit (LSB=1) is set ONLY on the last address.
   * The destination C-bit is 1 (command frame, as real TNCs send it),
   * 0 on the other addresses. The via H-bit (bit7) 0 = not yet repeated. --- */
  n += put_addr(&f[n], dst, dssid, false);
  f[6] |= 0x80;                              /* dest C-bit = 1 */
  n += put_addr(&f[n], src, ssid, !has_via);
  if (has_via) {
    n += put_addr(&f[n], via, vssid, true);  /* last address: ext=1 */
  }

  f[n++] = 0x03;   /* control: UI frame */
  f[n++] = 0xF0;   /* PID: no layer 3 */

  for (const char *s = info; *s && n < sizeof(f) - 2; ++s)
    f[n++] = (uint8_t)*s;

  /* --- FCS from the address field to the end of info --- */
  uint16_t fcs = ax25_fcs(f, n);
  f[n++] = (uint8_t)(fcs & 0xFF);
  f[n++] = (uint8_t)(fcs >> 8);

  /* --- HDLC: flag | stuffed data | flag, then NRZI ---
   * The bit stream is assembled LSB-first; the flags are NOT stuffed. */
  uint8_t nrzi = 1;           /* NRZI state (initial level) */
  uint16_t ob = 0;
  int ones = 0;

  #define EMIT_NRZI(bit) do {                          \
      if ((bit) == 0) nrzi ^= 1;  /* 0 = level change */ \
      if (ob < sizeof(out->bits)) out->bits[ob++] = nrzi; \
    } while (0)

  #define EMIT_FLAG() do {                             \
      /* 0x7E = 0 1111110, LSB-first: 0,1,1,1,1,1,1,0 */ \
      static const uint8_t fl[8] = {0,1,1,1,1,1,1,0};   \
      for (int k = 0; k < 8; ++k) EMIT_NRZI(fl[k]);     \
    } while (0)

  /* Initial idle level: the decoder's first difference needs a
   * reference, otherwise the first bit is lost. */
  if (ob < sizeof(out->bits)) out->bits[ob++] = nrzi;

  /* Preamble flags: real TNCs send 150-300 ms worth so that the FM
   * receiver's SQUELCH opens and bit sync settles BEFORE the data
   * arrives. 25 flags = 200 bits = ~167 ms at 1200 baud. (The earlier 4
   * flags = 27 ms was too short: by the time the squelch opened, the
   * start of the frame was already gone.) */
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

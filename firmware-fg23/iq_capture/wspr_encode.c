/* SPDX-License-Identifier: MIT
 *
 * wspr_encode.c/.h — WSPR uzenet-kodolo (HA7DCD + Cimbi)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * A K1JT-fele WSPR protokoll szabvanyos kodolasi lanca, a nyilvanos
 * specifikaciobol implementalva (fuggetlen C-implementacio; a
 * helyesseget az SM0YSR-fele Python referencia-kodoloval
 * keresztvalidaltuk a gazdagepen):
 *   1. uzenet-tomorites: hivojel (28 bit) + lokator (15) + dBm (7)
 *   2. konvolucios kodolas: K=32, r=1/2, Layland-Lushbaugh polinomok
 *      (0xF2D05351, 0xE4613C47), 31 zero-bit farokkal -> 162 bit
 *   3. bit-reverz interleaving
 *   4. szinkronvektor hozzaadasa: sym = sync + 2*data  (0..3)
 */

#include "wspr_encode.h"
#include <string.h>

/* 162 bites pszeudoveletlen szinkronvektor (WSPR szabvany) */
static const uint8_t wspr_sync[162] = {
  1,1,0,0,0,0,0,0,1,0,0,0,1,1,1,0,0,0,1,0,0,1,0,1,1,1,
  1,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,
  0,0,0,0,1,0,1,1,0,0,1,1,0,1,0,0,0,1,1,0,1,0,0,0,0,1,
  1,0,1,0,1,0,1,0,1,0,0,1,0,0,1,0,
  1,1,0,0,0,1,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,
  1,1,1,0,1,1,0,0,1,1,0,1,0,0,0,1,
  1,1,0,0,0,0,0,1,0,1,0,0,1,1,0,0,0,0,0,0,0,1,1,0,1,0,
  1,1,0,0,0,1,1,0,0,0
};

/* karakter -> index: '0'-'9' = 0-9, 'A'-'Z' = 10-35, ' ' = 36 */
static int chidx(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'Z') return c - 'A' + 10;
  if (c == ' ') return 36;
  return -1;
}

static int parity32(uint32_t x)
{
  x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
  return (int)(x & 1u);
}

bool wspr_encode(const char *callsign, const char *locator,
                 int dbm, uint8_t sym[162])
{
  /* --- hivojel normalizalas: a 3. karakter SZAMJEGY kell legyen;
   * ha a 2. az, elore egy szokoz; jobbra szokozokkel 6-ra toltve --- */
  char cs[7];
  size_t len = strlen(callsign);
  if (len < 3 || len > 6) return false;
  if (callsign[2] >= '0' && callsign[2] <= '9') {
    memset(cs, ' ', 6);
    memcpy(cs, callsign, len);
  } else if (callsign[1] >= '0' && callsign[1] <= '9') {
    if (len > 5) return false;
    cs[0] = ' ';
    memset(cs + 1, ' ', 5);
    memcpy(cs + 1, callsign, len);
  } else {
    return false;
  }
  cs[6] = '\0';

  int c0 = chidx(cs[0]), c1 = chidx(cs[1]), c2 = chidx(cs[2]);
  int c3 = chidx(cs[3]), c4 = chidx(cs[4]), c5 = chidx(cs[5]);
  if (c0 < 0 || c1 < 0 || c1 >= 36 || c2 < 0 || c2 >= 10 ||
      c3 < 10 || c4 < 10 || c5 < 10) return false;

  uint32_t n_call = (uint32_t)c0;
  n_call = 36u * n_call + (uint32_t)c1;
  n_call = 10u * n_call + (uint32_t)c2;
  n_call = 27u * n_call + (uint32_t)(c3 - 10);
  n_call = 27u * n_call + (uint32_t)(c4 - 10);
  n_call = 27u * n_call + (uint32_t)(c5 - 10);

  /* --- lokator: 4 karakter, AA00..RR99 --- */
  if (strlen(locator) != 4) return false;
  int L0 = locator[0] - 'A', L1 = locator[1] - 'A';
  int N2 = locator[2] - '0', N3 = locator[3] - '0';
  if (L0 < 0 || L0 > 17 || L1 < 0 || L1 > 17 ||
      N2 < 0 || N2 > 9 || N3 < 0 || N3 > 9) return false;
  uint32_t n_loc = (uint32_t)((179 - 10 * L0 - N2) * 180
                              + 10 * L1 + N3);

  /* --- teljesitmeny: barmely 0..60 ervenyes szintre igazitva --- */
  if (dbm < 0 || dbm > 60) return false;
  static const int corr[10] = { 0, -1, 1, 0, -1, 2, 1, 0, -1, 1 };
  uint32_t n_dbm = (uint32_t)(dbm + corr[dbm % 10] + 64);

  /* --- 50 bites uzenetszo --- */
  uint64_t n = ((uint64_t)n_call << 22) | ((uint64_t)n_loc << 7)
             | (uint64_t)n_dbm;

  /* --- konvolucios kodolas + interleaving egy menetben ---
   * A 81 bites bemenet: az 50 bit MSB-tol, majd 31 zero-farok.
   * A ket 32 bites shiftregiszter termeszetes csonkolasa megegyezik
   * a polinom-maszkolassal. Az i-edik kimeneti bit a bit-reverz(byte)
   * szerinti helyre kerul. */
  uint8_t data[162];
  uint32_t r0 = 0, r1 = 0;
  int out_i = 0;
  uint8_t place[162];
  {
    int p = 0;
    for (int i = 0; i < 256 && p < 162; ++i) {
      int j = ((i & 0x01) << 7) | ((i & 0x02) << 5) | ((i & 0x04) << 3)
            | ((i & 0x08) << 1) | ((i & 0x10) >> 1) | ((i & 0x20) >> 3)
            | ((i & 0x40) >> 5) | ((i & 0x80) >> 7);
      if (j < 162) place[p++] = (uint8_t)j;
    }
  }
  for (int i = 0; i < 81; ++i) {
    uint32_t b = (i < 50) ? (uint32_t)((n >> (49 - i)) & 1u) : 0u;
    r0 = (r0 << 1) | b;
    r1 = (r1 << 1) | b;
    data[place[out_i++]] = (uint8_t)parity32(r0 & 0xF2D05351u);
    data[place[out_i++]] = (uint8_t)parity32(r1 & 0xE4613C47u);
  }

  for (int i = 0; i < 162; ++i)
    sym[i] = (uint8_t)(wspr_sync[i] + 2u * data[i]);
  return true;
}

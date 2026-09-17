/* SPDX-License-Identifier: MIT
 *
 * cw_morse.c — CW (Morse) adó a FG23-on
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * PARIS standard: egy „szó” = 50 egység. Ezért
 *   unit_us = 1 200 000 / WPM
 * (12 WPM → 100 ms, 18 WPM → ~66.7 ms, 24 WPM → 50 ms).
 *
 * Keying: StartTxStream / StopTxStream. A RAIL stream indítási
 * késleltetése ~1–2 ms, ami 15+ WPM-nél még elfogadható (a
 * karakterek közötti szünet elnyeli). Ha később precízebb kell,
 * át lehet váltani PA-power kapcsolásra (RAIL_SetTxPowerDbm 0 ↔
 * célérték) — a stream akkor végig fut.
 *
 * Abort: a VCOM-ról jövő 'x' (vagy 'X') karakter. Nem kell Enter.
 */

#include "cw_morse.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>

/* ---------------- Morse tábla (ITU) ----------------
 * Bit-pack: a legalsó bit a legelső elem.
 * 0 = dit, 1 = dah. A hossz külön táblában (1..5).
 * Prosign-ök és írásjelek is.
 */
typedef struct {
  uint8_t bits;   /* LSB first */
  uint8_t len;    /* 1..6 */
} cw_code_t;

/* Index: 'A'..'Z' = 0..25, '0'..'9' = 26..35, majd írásjelek. */
static const cw_code_t CW_ALPHA[26] = {
  /* A */ { 0b01,    2 }, /* .-     */
  /* B */ { 0b1000,  4 }, /* -...   */
  /* C */ { 0b1010,  4 }, /* -.-.   */
  /* D */ { 0b100,   3 }, /* -..    */
  /* E */ { 0b0,     1 }, /* .      */
  /* F */ { 0b0010,  4 }, /* ..-.   */
  /* G */ { 0b110,   3 }, /* --.    */
  /* H */ { 0b0000,  4 }, /* ....   */
  /* I */ { 0b00,    2 }, /* ..     */
  /* J */ { 0b0111,  4 }, /* .---   */
  /* K */ { 0b101,   3 }, /* -.-    */
  /* L */ { 0b0100,  4 }, /* .-..   */
  /* M */ { 0b11,    2 }, /* --     */
  /* N */ { 0b10,    2 }, /* -.     */
  /* O */ { 0b111,   3 }, /* ---    */
  /* P */ { 0b0110,  4 }, /* .--.   */
  /* Q */ { 0b1101,  4 }, /* --.-   */
  /* R */ { 0b010,   3 }, /* .-.    */
  /* S */ { 0b000,   3 }, /* ...    */
  /* T */ { 0b1,     1 }, /* -      */
  /* U */ { 0b001,   3 }, /* ..-    */
  /* V */ { 0b0001,  4 }, /* ...-   */
  /* W */ { 0b011,   3 }, /* .--    */
  /* X */ { 0b1001,  4 }, /* -..-   */
  /* Y */ { 0b1011,  4 }, /* -.--   */
  /* Z */ { 0b1100,  4 }, /* --..   */
};

static const cw_code_t CW_DIGIT[10] = {
  /* 0 */ { 0b11111, 5 }, /* -----  */
  /* 1 */ { 0b01111, 5 }, /* .----  */
  /* 2 */ { 0b00111, 5 }, /* ..---  */
  /* 3 */ { 0b00011, 5 }, /* ...--  */
  /* 4 */ { 0b00001, 5 }, /* ....-  */
  /* 5 */ { 0b00000, 5 }, /* .....  */
  /* 6 */ { 0b10000, 5 }, /* -....  */
  /* 7 */ { 0b11000, 5 }, /* --...  */
  /* 8 */ { 0b11100, 5 }, /* ---..  */
  /* 9 */ { 0b11110, 5 }, /* ----.  */
};

/* Írásjelek — a leggyakoribbak. */
static const cw_code_t CW_PERIOD   = { 0b010101, 6 }; /* .-.-.-  */
static const cw_code_t CW_COMMA    = { 0b110011, 6 }; /* --..--  */
static const cw_code_t CW_SLASH    = { 0b10010,  5 }; /* -..-.   */
static const cw_code_t CW_QUESTION = { 0b001100, 6 }; /* ..--..  */
static const cw_code_t CW_EQUALS   = { 0b10001,  5 }; /* -...-   */ /* BT */
static const cw_code_t CW_PLUS     = { 0b01010,  5 }; /* .-.-.   */ /* AR */
static const cw_code_t CW_MINUS    = { 0b100001, 6 }; /* -....-  */
static const cw_code_t CW_AT       = { 0b011010, 6 }; /* .--.-.  */ /* @ */

static const cw_code_t *cw_lookup(char c, cw_code_t *tmp)
{
  (void)tmp;
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
  if (c >= 'A' && c <= 'Z') return &CW_ALPHA[c - 'A'];
  if (c >= '0' && c <= '9') return &CW_DIGIT[c - '0'];
  switch (c) {
    case '.': return &CW_PERIOD;
    case ',': return &CW_COMMA;
    case '/': return &CW_SLASH;
    case '?': return &CW_QUESTION;
    case '=': return &CW_EQUALS;
    case '+': return &CW_PLUS;
    case '-': return &CW_MINUS;
    case '@': return &CW_AT;
    default:  return NULL;          /* szóköz / ismeretlen → szószünet */
  }
}

/* ---------------- timing ---------------- */

static uint32_t s_unit_us = 66667u;   /* 18 WPM default */

static void cw_set_wpm(uint8_t wpm)
{
  if (wpm < 5)  wpm = 5;
  if (wpm > 40) wpm = 40;
  /* PARIS = 50 unit / szó → unit = 1.2 s / WPM */
  s_unit_us = 1200000u / (uint32_t)wpm;
}

/* Pontos várakozás RAIL idővel. Közben poll-olja a VCOM-ot abortért. */
static bool cw_delay_units(RAIL_Handle_t rail, uint32_t units)
{
  (void)rail;
  RAIL_Time_t target = RAIL_GetTime() + (RAIL_Time_t)(units * s_unit_us);
  while ((int32_t)(RAIL_GetTime() - target) < 0) {
    char ch;
    if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
      if (ch == 'x' || ch == 'X') return false;   /* abort */
    }
  }
  return true;
}

static bool cw_key_down(RAIL_Handle_t rail, uint16_t channel)
{
  RAIL_Status_t st = RAIL_StartTxStream(rail, channel, RAIL_STREAM_CARRIER_WAVE);
  return (st == RAIL_STATUS_NO_ERROR);
}

static void cw_key_up(RAIL_Handle_t rail)
{
  RAIL_StopTxStream(rail);
}

/* Egy karakter adása. Vissza: false = abort. */
static bool cw_send_char(RAIL_Handle_t rail, uint16_t channel, const cw_code_t *code)
{
  for (uint8_t i = 0; i < code->len; ++i) {
    bool dah = (code->bits >> i) & 1u;
    if (!cw_key_down(rail, channel)) {
      printf("# CW: TX start hiba\r\n");
      return false;
    }
    if (!cw_delay_units(rail, dah ? 3u : 1u)) {
      cw_key_up(rail);
      return false;
    }
    cw_key_up(rail);
    /* elemközti szünet (1 unit), kivéve az utolsó után */
    if (i + 1u < code->len) {
      if (!cw_delay_units(rail, 1u)) return false;
    }
  }
  return true;
}

/* ---------------- nyilvános API ---------------- */

bool cw_morse_send(RAIL_Handle_t rail, uint16_t channel,
                   const char *text, uint8_t wpm)
{
  if (!text || !text[0]) return true;

  cw_set_wpm(wpm ? wpm : CW_DEFAULT_WPM);

  printf("# CW TX %u WPM: \"%s\"  (x = megszakitas)\r\n",
         (unsigned)(wpm ? wpm : CW_DEFAULT_WPM), text);

  /* Először Idle + Stop, hogy tiszta legyen a rádió. */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  RAIL_StopTxStream(rail);

  bool ok = true;
  bool first = true;

  for (const char *p = text; *p && ok; ++p) {
    if (*p == ' ' || *p == '\t') {
      /* szószünet: 7 unit (a karakterközi 3-ból már 1 elment, ezért +4
       * a következő karakter előtt fogjuk a 3-at, de egyszerűbb a
       * szabványos 7 unit a szó között, és a karakterek között 3). */
      if (!cw_delay_units(rail, 7u)) { ok = false; break; }
      first = true;
      continue;
    }

    cw_code_t tmp;
    const cw_code_t *code = cw_lookup(*p, &tmp);
    if (!code) continue;            /* ismeretlen → kihagy */

    if (!first) {
      /* karakterközi szünet = 3 unit */
      if (!cw_delay_units(rail, 3u)) { ok = false; break; }
    }
    first = false;

    if (!cw_send_char(rail, channel, code)) {
      ok = false;
      break;
    }
  }

  cw_key_up(rail);
  /* A hívó (app.c) végzi a restart_rx()-et és a s_tx_active=false-ot.
   * Itt csak a TX-et állítjuk le, hogy a frekvencia-korrekció
   * (FREQ_CORR_TICK) és a FIFO reset a központi helyen maradjon. */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);

  printf("# CW %s\r\n", ok ? "kesz" : "MEGSZAKITVA");
  return ok;
}

bool cw_morse_cq(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm)
{
  /* Klasszikus CQ hívás + saját hívójel */
  char buf[48];
  snprintf(buf, sizeof buf, "CQ CQ CQ DE %s %s K", CW_MYCALL, CW_MYCALL);
  return cw_morse_send(rail, channel, buf, wpm);
}

bool cw_morse_test(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm)
{
  /* VFO / szint / frekvencia ellenőrzéshez: ismétlődő VVV + hívójel */
  char buf[40];
  snprintf(buf, sizeof buf, "VVV VVV DE %s", CW_MYCALL);
  return cw_morse_send(rail, channel, buf, wpm);
}

bool cw_morse_beacon(RAIL_Handle_t rail, uint16_t channel, uint8_t wpm)
{
  /* Rövid beacon: hívójel + QTH hint (JN97) */
  char buf[48];
  snprintf(buf, sizeof buf, "DE %s %s JN97", CW_MYCALL, CW_MYCALL);
  return cw_morse_send(rail, channel, buf, wpm);
}

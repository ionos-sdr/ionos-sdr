/* SPDX-License-Identifier: MIT
 *
 * cw_rx.cpp — CW decoder for the ESP32-S3
 *   IQ → NCO mixing (tone → DC) → envelope → hysteresis → Morse timing
 *
 * Important: AFTER mixing to the tone the signal sits at DC, so NO Goertzel
 * at 700 Hz but an envelope (I'²+Q'²). Goertzel is for unmixed audio CW.
 *
 * HackRF: --tone 700   (the stream DC-blocks 0 Hz OOK)
 */

#include "cw_rx.h"
#include <string.h>
#include <math.h>
#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CW_TONE_DEFAULT  700.0f
#define CW_TEXT_LEN      48
#define CW_MIN_DOT_MS    20
#define CW_MAX_DOT_MS    200
#define CW_ENV_SHIFT     3          /* envelope smoothing 1/8 */
#define CW_NOISE_SHIFT   6          /* noise floor 1/64 */

static float    s_tone_hz = CW_TONE_DEFAULT;
static uint32_t s_sps = 50000;
static float    s_ph = 0, s_dph = 0;

static float    s_env = 0;          /* smoothed envelope */
static float    s_noise = 100.f;    /* key-up noise floor */
static float    s_peak = 0;
static bool     s_key = false;
static bool     s_prev_key = false;
static bool     s_signal = false;

static uint32_t s_edge_ms = 0;
static uint32_t s_dot_ms = 60;      /* ~20 WPM */

static uint8_t  s_bits = 0;
static uint8_t  s_bitlen = 0;

static char     s_text[CW_TEXT_LEN];
static uint8_t  s_text_len = 0;
static uint32_t s_chars = 0;
static uint32_t s_last_ms = 0;

/* debug: number of key events, max env */
static float    s_dbg_max_env = 0;
static uint32_t s_dbg_keys = 0;
static uint32_t s_dbg_last_print = 0;

typedef struct { uint8_t bits; uint8_t len; char ch; } rev_t;
static const rev_t REV[] = {
  {0b01,2,'A'},{0b1000,4,'B'},{0b1010,4,'C'},{0b100,3,'D'},{0b0,1,'E'},
  {0b0010,4,'F'},{0b110,3,'G'},{0b0000,4,'H'},{0b00,2,'I'},{0b0111,4,'J'},
  {0b101,3,'K'},{0b0100,4,'L'},{0b11,2,'M'},{0b10,2,'N'},{0b111,3,'O'},
  {0b0110,4,'P'},{0b1101,4,'Q'},{0b010,3,'R'},{0b000,3,'S'},{0b1,1,'T'},
  {0b001,3,'U'},{0b0001,4,'V'},{0b011,3,'W'},{0b1001,4,'X'},{0b1011,4,'Y'},
  {0b1100,4,'Z'},
  {0b11111,5,'0'},{0b01111,5,'1'},{0b00111,5,'2'},{0b00011,5,'3'},
  {0b00001,5,'4'},{0b00000,5,'5'},{0b10000,5,'6'},{0b11000,5,'7'},
  {0b11100,5,'8'},{0b11110,5,'9'},
  {0b010101,6,'.'},{0b110011,6,','},{0b10010,5,'/'},{0b001100,6,'?'},
  {0b10001,5,'='},{0b01010,5,'+'},{0b100001,6,'-'},{0b011010,6,'@'},
};
static const int REV_N = (int)(sizeof(REV)/sizeof(REV[0]));

static char decode_bits(uint8_t bits, uint8_t len)
{
  for (int i = 0; i < REV_N; ++i)
    if (REV[i].len == len && REV[i].bits == bits) return REV[i].ch;
  return '?';
}

static void push_char(char c)
{
  if (s_text_len + 1 >= CW_TEXT_LEN) {
    memmove(s_text, s_text + 1, CW_TEXT_LEN - 2);
    s_text_len = CW_TEXT_LEN - 2;
    s_text[s_text_len] = 0;
  }
  s_text[s_text_len++] = c;
  s_text[s_text_len] = 0;
  ++s_chars;
  s_last_ms = millis();
  Serial0.printf("CW: '%c' [%s] ~%lu WPM\n",
                 c, s_text, (unsigned long)(1200u / (s_dot_ms ? s_dot_ms : 60)));
}

static void finish_char(void)
{
  if (s_bitlen == 0) return;
  push_char(decode_bits(s_bits, s_bitlen));
  s_bits = 0;
  s_bitlen = 0;
}

static void retune(void)
{
  if (s_sps < 1000) s_sps = 50000;
  s_dph = 2.f * (float)M_PI * s_tone_hz / (float)s_sps;
}

void cw_rx_set_tone(float hz)
{
  if (hz < 200.f) hz = 200.f;
  if (hz > 4000.f) hz = 4000.f;
  s_tone_hz = hz;
  retune();
}

void cw_rx_init(void)
{
  memset(s_text, 0, sizeof s_text);
  s_text_len = 0; s_chars = 0; s_last_ms = 0;
  s_key = s_prev_key = s_signal = false;
  s_env = 0; s_noise = 100.f; s_peak = 0;
  s_edge_ms = millis();
  s_dot_ms = 60;
  s_bits = 0; s_bitlen = 0;
  s_ph = 0;
  s_dbg_max_env = 0; s_dbg_keys = 0; s_dbg_last_print = 0;
  retune();
  Serial0.printf("CW: mix+env ready (tone=%.0f Hz). HackRF: --tone %.0f\n",
                 (double)s_tone_hz, (double)s_tone_hz);
}

void cw_rx_feed(const int16_t *iq, int nsamp, uint32_t sps)
{
  if (sps && sps != s_sps) {
    s_sps = sps;
    retune();
  }

  for (int n = 0; n < nsamp; ++n) {
    float ii = (float)iq[n * 2];
    float qq = (float)iq[n * 2 + 1];

    /* NCO: tone → DC. Either sideband: mix with +tone.
     * I' = I*c + Q*s
     * Q' = Q*c - I*s
     * env = sqrt(I'²+Q'²) = |z|  (rotation-invariant, LSB/USB irrelevant) */
    float c = cosf(s_ph);
    float s = sinf(s_ph);
    s_ph += s_dph;
    if (s_ph > 2.f * (float)M_PI) s_ph -= 2.f * (float)M_PI;

    float ip = ii * c + qq * s;
    float qp = qq * c - ii * s;
    float env = sqrtf(ip * ip + qp * qp);

    /* fast envelope smoothing */
    s_env += (env - s_env) * (1.f / (float)(1 << CW_ENV_SHIFT));

    if (s_env > s_peak) s_peak = s_env;
    else s_peak += (s_env - s_peak) * 0.01f;

    if (s_env > s_dbg_max_env) s_dbg_max_env = s_env;

    /* noise floor tracked only during key-up */
    if (!s_key) {
      s_noise += (s_env - s_noise) * (1.f / (float)(1 << CW_NOISE_SHIFT));
      if (s_noise < 1.f) s_noise = 1.f;
    }

    /* hysteresis: on = 3×noise, off = 1.5×noise, absolute floor */
    float thr_on  = s_noise * 3.0f + 50.f;
    float thr_off = s_noise * 1.5f + 20.f;
    s_signal = (s_peak > thr_on);

    bool key = s_key ? (s_env > thr_off) : (s_env > thr_on);

    if (key != s_prev_key) {
      uint32_t now = millis();
      uint32_t dur = now - s_edge_ms;
      if (dur < 1) dur = 1;

      if (s_prev_key) {
        /* end of key-down */
        if (dur >= CW_MIN_DOT_MS) {
          bool dah = (dur >= s_dot_ms * 2);
          uint32_t est = dah ? (dur / 3) : dur;
          if (est < CW_MIN_DOT_MS) est = CW_MIN_DOT_MS;
          if (est > CW_MAX_DOT_MS) est = CW_MAX_DOT_MS;
          s_dot_ms = (s_dot_ms * 3 + est) / 4;
          if (s_bitlen < 6) {
            if (dah) s_bits |= (uint8_t)(1u << s_bitlen);
            ++s_bitlen;
          }
          ++s_dbg_keys;
        }
      } else {
        /* end of key-up */
        if (dur >= s_dot_ms * 6) {
          finish_char();
          if (s_text_len && s_text[s_text_len - 1] != ' ')
            push_char(' ');
        } else if (dur >= s_dot_ms * 2) {
          finish_char();
        }
      }
      s_edge_ms = now;
      s_prev_key = key;
      s_key = key;
    }
  }
}

void cw_rx_tick(uint32_t now_ms)
{
  uint32_t silence = now_ms - s_edge_ms;
  if (!s_key && s_bitlen > 0 && silence > s_dot_ms * 3)
    finish_char();

  /* diagnostics every 2 s until enough characters have been decoded */
  if (now_ms - s_dbg_last_print > 2000) {
    s_dbg_last_print = now_ms;
    if (s_chars < 20) {
      Serial0.printf("CW dbg: env=%.0f peak=%.0f noise=%.0f thr=%.0f key=%d sig=%d keys=%lu\n",
                     (double)s_env, (double)s_peak, (double)s_noise,
                     (double)(s_noise * 3.f + 50.f),
                     (int)s_key, (int)s_signal, (unsigned long)s_dbg_keys);
    }
    s_dbg_max_env = 0;
  }
}

const char *cw_rx_text(void)     { return s_text; }
uint32_t    cw_rx_chars(void)    { return s_chars; }
uint32_t    cw_rx_last_ms(void)  { return s_last_ms; }
bool        cw_rx_keyed(void)    { return s_key; }
bool        cw_rx_signal(void)   { return s_signal; }
float       cw_rx_wpm(void)      { return s_dot_ms ? (1200.f / (float)s_dot_ms) : 0.f; }
float       cw_rx_offset_hz(void){ return s_tone_hz; }

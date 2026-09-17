/*
 * cw.c  --  Morse table, WPM/Farnsworth timing, keyed playback.
 *  Concept by: N7HPR   Design by: HA7DCD
 *
 *  The table + timing math compile anywhere (host-testable). The blocking
 *  playback (tone/LED/delay) is guarded by CW_HOST_TEST.
 */
#include "cw.h"

#ifndef CW_HOST_TEST
#include "tone.h"
#include "sl_sleeptimer.h"
#endif

/* ---------- Morse table ---------- */
typedef struct { char c; const char *p; } morse_t;

static const morse_t TABLE[] = {
    {'A',".-"},   {'B',"-..."}, {'C',"-.-."}, {'D',"-.."},  {'E',"."},
    {'F',"..-."}, {'G',"--."},  {'H',"...."}, {'I',".."},   {'J',".---"},
    {'K',"-.-"},  {'L',".-.."}, {'M',"--"},   {'N',"-."},   {'O',"---"},
    {'P',".--."}, {'Q',"--.-"}, {'R',".-."},  {'S',"..."},  {'T',"-"},
    {'U',"..-"},  {'V',"...-"}, {'W',".--"},  {'X',"-..-"}, {'Y',"-.--"},
    {'Z',"--.."},
    {'0',"-----"},{'1',".----"},{'2',"..---"},{'3',"...--"},{'4',"....-"},
    {'5',"....."},{'6',"-...."},{'7',"--..."},{'8',"---.."},{'9',"----."},
    {'.',".-.-.-"},{',',"--..--"},{'?',"..--.."},{'/',"-..-."},
    {'=',"-...-"}, {'+',".-.-."}, {'-',"-....-"},{':',"---..."},
    {'\'',".----."},{'(',"-.--."},{')',"-.--.-"},{'@',".--.-."},
};
#define TABLE_N (sizeof(TABLE)/sizeof(TABLE[0]))

const char *cw_pattern(char c)
{
    if (c >= 'a' && c <= 'z') c -= 0x20;
    for (unsigned i = 0; i < TABLE_N; i++)
        if (TABLE[i].c == c) return TABLE[i].p;
    return 0;
}

/* ---------- timing ---------- */
static uint8_t s_char_wpm = 18;
static uint8_t s_eff_wpm  = 12;

void cw_set_speed(uint8_t char_wpm, uint8_t eff_wpm)
{
    if (char_wpm < 5)  char_wpm = 5;
    if (char_wpm > 40) char_wpm = 40;
    if (eff_wpm  < 5)  eff_wpm  = 5;
    if (eff_wpm > char_wpm) eff_wpm = char_wpm;
    s_char_wpm = char_wpm;
    s_eff_wpm  = eff_wpm;
}
uint8_t cw_get_char_wpm(void) { return s_char_wpm; }
uint8_t cw_get_eff_wpm(void)  { return s_eff_wpm;  }

uint16_t cw_dot_ms(void) { return (uint16_t)(1200u / s_char_wpm); }

/* Farnsworth spacing unit in ms (ARRL method).
 * spacing_unit = (60000/eff - 37200/char) / 19, but never below one dot. */
static uint16_t farns_unit_ms(void)
{
    long c = s_char_wpm, s = s_eff_wpm;
    long dot = 1200 / c;
    if (s >= c) return (uint16_t)dot;
    long su = (60000L / s - 37200L / c) / 19L;
    if (su < dot) su = dot;
    return (uint16_t)su;
}

int cw_units(const char *word)
{
    int total = 0, n = 0;
    for (const char *p = word; *p; p++) {
        const char *pat = cw_pattern(*p);
        if (!pat) continue;
        if (n > 0) total += 3;                 /* inter-character gap */
        int elems = 0;
        for (const char *e = pat; *e; e++) {
            if (e != pat) total += 1;          /* intra-element gap */
            total += (*e == '-') ? 3 : 1;      /* dash : dot */
            elems++;
        }
        (void)elems;
        n++;
    }
    return total;
}

/* ---------- playback ---------- */
static void (*s_key_cb)(bool) = 0;
void cw_set_key_cb(void (*cb)(bool)) { s_key_cb = cb; }

#ifndef CW_HOST_TEST

static void key(bool down)
{
    if (down) tone_on(); else tone_off();
    if (s_key_cb) s_key_cb(down);
}
static void wait_ms(uint32_t ms)
{
    if (ms) sl_sleeptimer_delay_millisecond(ms);
}

/* Key-down compensation, ms. The PA ramp (see CWTX_RAMP_TIME_US) delays the
 * carrier coming up but not going down, so a keyed element comes out about
 * half a ramp short and the following gap that much long. Adding it back
 * here keeps the on:off ratio at 1:1 on the air. Zero for the sidetone-only
 * drill, where there is no ramp. */
static uint32_t s_key_comp_ms = 0;
void cw_set_key_comp_ms(uint16_t ms) { s_key_comp_ms = ms; }

static uint32_t less_comp(uint32_t ms)
{
    return (ms > s_key_comp_ms) ? (ms - s_key_comp_ms) : 1u;
}

void cw_play_char(char c)
{
    const char *pat = cw_pattern(c);
    uint16_t dot = cw_dot_ms();
    if (pat) {
        for (const char *e = pat; *e; e++) {
            key(true);
            wait_ms(((*e == '-') ? (uint32_t)dot * 3 : dot) + s_key_comp_ms);
            key(false);
            if (e[1]) wait_ms(less_comp(dot));            /* intra-element gap */
        }
    }
    wait_ms(less_comp((uint32_t)farns_unit_ms() * 3));    /* inter-character gap */
}

void cw_play_str(const char *s)
{
    for (; *s; s++) {
        if (*s == ' ')
            wait_ms((uint32_t)farns_unit_ms() * 4);  /* +3 already => ~7u */
        else
            cw_play_char(*s);
    }
}

#endif /* CW_HOST_TEST */

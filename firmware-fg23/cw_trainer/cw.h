/*
 * cw.h  --  Morse code table, WPM/Farnsworth timing, and keyed playback.
 *  Concept by: N7HPR   Design by: HA7DCD
 */
#ifndef CW_H_INCLUDED
#define CW_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

/* Morse pattern for a character, e.g. ".-" for 'A'. NULL if unsupported.
 * Letters are case-insensitive. */
const char *cw_pattern(char c);

/* Character speed and effective (Farnsworth) speed, both in WPM.
 * eff_wpm is clamped to <= char_wpm. */
void     cw_set_speed(uint8_t char_wpm, uint8_t eff_wpm);
uint8_t  cw_get_char_wpm(void);
uint8_t  cw_get_eff_wpm(void);

/* One "dot" at the current character speed, in milliseconds. */
uint16_t cw_dot_ms(void);

/* PARIS-unit count of a single word (no spaces): elements + intra-element
 * gaps + inter-character gaps. Used for timing verification. */
int      cw_units(const char *word);

/* Optional visual/key hook, called with true on key-down, false on key-up
 * (drive an LED, a T/R line, etc). Set to NULL to disable. */
void     cw_set_key_cb(void (*cb)(bool key_down));

/* Lengthen every keyed element by this many ms and shorten the following gap
 * by the same amount. Compensates the PA ramp, which delays the carrier
 * coming up but not going down. 0 for the sidetone-only drill. */
void     cw_set_key_comp_ms(uint16_t ms);

/* Play one character (blocking) as sidetone + key hook. Unknown chars are
 * silent but still consume an inter-character gap. */
void     cw_play_char(char c);

/* Play a string; ' ' produces a word gap. */
void     cw_play_str(const char *s);

#endif /* CW_H_INCLUDED */

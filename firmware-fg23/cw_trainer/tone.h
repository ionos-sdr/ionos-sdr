/*
 * tone.h  --  Sidetone generator (TIMER PWM square wave on a GPIO).
 *  Concept by: N7HPR   Design by: HA7DCD
 */
#ifndef TONE_H_INCLUDED
#define TONE_H_INCLUDED

#include <stdint.h>

/* Configure the timer/pin and start silent. Call once at boot. */
void tone_init(void);

/* Key the sidetone on/off (50% duty when on, line low when off). */
void tone_on(void);
void tone_off(void);

/* Change pitch at runtime (Hz). */
void tone_set_freq(uint16_t hz);

#endif /* TONE_H_INCLUDED */

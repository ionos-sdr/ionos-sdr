/*
 * oled.h  --  Compact SSD1306 128x64 driver (software I2C) for the CW Trainer
 *  Concept by: N7HPR   Design by: HA7DCD
 *
 *  Self-contained: no external graphics library. A 1 KB RAM framebuffer,
 *  a 5x7 font, and a scaled "big glyph" renderer for the trainer's answer.
 */
#ifndef OLED_H_INCLUDED
#define OLED_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

#define OLED_W   128
#define OLED_H   64

/* Bring up the panel. Returns false if the panel does not ACK (bad wiring). */
bool oled_init(void);

/* Framebuffer operations (nothing shows until oled_flush()). */
void oled_clear(void);
void oled_flush(void);
void oled_set_pixel(int x, int y, bool on);

/* Text. scale=1 -> 6x8 cells. y is the pixel row of the glyph top. */
void oled_text(int x, int y, const char *s);
void oled_text_scaled(int x, int y, const char *s, uint8_t scale);

/* Centered helpers. */
void oled_text_center(int y, const char *s);
void oled_text_center_scaled(int y, const char *s, uint8_t scale);

/* One big character centered on screen (used to reveal the answer). */
void oled_big_char(char c);

/* Simple horizontal rule / box helpers. */
void oled_hline(int x0, int x1, int y, bool on);
void oled_fill_rect(int x, int y, int w, int h, bool on);

#endif /* OLED_H_INCLUDED */

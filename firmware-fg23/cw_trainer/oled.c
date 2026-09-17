/*
 * oled.c  --  Compact SSD1306 128x64 driver (software I2C)
 *  Concept by: N7HPR   Design by: HA7DCD
 *
 *  Portable rendering (framebuffer + font + scaler) is kept free of any
 *  Silicon Labs headers so it can be host-tested. The hardware layer
 *  (GPIO bit-bang) is guarded by OLED_HOST_TEST for the desktop harness.
 */
#include "oled.h"
#include <string.h>

#ifndef OLED_HOST_TEST
#include "em_cmu.h"
#include "em_gpio.h"
#include "bsp_pins.h"
#endif

/* ---------- framebuffer ---------- */
static uint8_t fb[OLED_W * OLED_H / 8];   /* 1024 bytes, page-addressed */

/* ---------- 5x7 font, ASCII 0x20..0x5F (space..'_') ---------- */
static const uint8_t font5x7[0x60 - 0x20][5] = {
    {0x00,0x00,0x00,0x00,0x00}, /* 0x20 space */
    {0x00,0x00,0x5F,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x23,0x13,0x08,0x64,0x62}, /* % */
    {0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x14,0x08,0x3E,0x08,0x14}, /* * */
    {0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x50,0x30,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14}, /* = */
    {0x00,0x41,0x22,0x14,0x08}, /* > */
    {0x02,0x01,0x51,0x09,0x06}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x03,0x04,0x78,0x04,0x03}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}, /* Z */
    {0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20}, /* \ */
    {0x00,0x41,0x41,0x7F,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40}, /* _ */
};

static const uint8_t *glyph(char c)
{
    unsigned u = (unsigned char)c;
    if (u >= 'a' && u <= 'z') u -= 0x20;      /* lowercase -> uppercase */
    if (u < 0x20 || u > 0x5F) u = 0x20;       /* unknown  -> space      */
    return font5x7[u - 0x20];
}

/* ---------- portable framebuffer drawing ---------- */
void oled_clear(void) { memset(fb, 0, sizeof(fb)); }

void oled_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_W || y < 0 || y >= OLED_H) return;
    uint8_t *p = &fb[(y >> 3) * OLED_W + x];
    uint8_t m = (uint8_t)(1u << (y & 7));
    if (on) *p |= m; else *p &= (uint8_t)~m;
}

void oled_fill_rect(int x, int y, int w, int h, bool on)
{
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            oled_set_pixel(x + i, y + j, on);
}

void oled_hline(int x0, int x1, int y, bool on)
{
    if (x1 < x0) { int t = x0; x0 = x1; x1 = t; }
    for (int x = x0; x <= x1; x++) oled_set_pixel(x, y, on);
}

void oled_text_scaled(int x, int y, const char *s, uint8_t scale)
{
    for (; *s; s++) {
        const uint8_t *g = glyph(*s);
        for (int col = 0; col < 5; col++) {
            uint8_t bits = g[col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1u << row))
                    oled_fill_rect(x + col * scale, y + row * scale, scale, scale, true);
            }
        }
        x += 6 * scale;   /* 5 wide + 1 spacing */
    }
}

void oled_text(int x, int y, const char *s) { oled_text_scaled(x, y, s, 1); }

static int text_width(const char *s, uint8_t scale)
{
    int n = 0; while (s[n]) n++;
    return n * 6 * scale;    /* includes trailing spacing */
}

void oled_text_center_scaled(int y, const char *s, uint8_t scale)
{
    int w = text_width(s, scale) - scale;      /* drop last spacing col */
    int x = (OLED_W - w) / 2; if (x < 0) x = 0;
    oled_text_scaled(x, y, s, scale);
}

void oled_text_center(int y, const char *s) { oled_text_center_scaled(y, s, 1); }

void oled_big_char(char c)
{
    const uint8_t scale = 6;                   /* 30 x 42 px glyph */
    char buf[2] = { c, 0 };
    int w = 5 * scale;
    int x = (OLED_W - w) / 2;
    int y = (OLED_H - 7 * scale) / 2;
    oled_text_scaled(x, y, buf, scale);
}

/* ================= hardware layer (skipped in host test) ================= */
#ifndef OLED_HOST_TEST

static inline void i2c_delay(void)
{
    for (volatile unsigned i = 0; i < OLED_I2C_DELAY_LOOPS; i++) { __asm volatile("nop"); }
}
static inline void scl_hi(void) { GPIO_PinOutSet(OLED_SCL_PORT, OLED_SCL_PIN); }
static inline void scl_lo(void) { GPIO_PinOutClear(OLED_SCL_PORT, OLED_SCL_PIN); }
static inline void sda_hi(void) { GPIO_PinOutSet(OLED_SDA_PORT, OLED_SDA_PIN); }
static inline void sda_lo(void) { GPIO_PinOutClear(OLED_SDA_PORT, OLED_SDA_PIN); }

static void i2c_start(void)
{
    sda_hi(); scl_hi(); i2c_delay();
    sda_lo(); i2c_delay();
    scl_lo(); i2c_delay();
}

static void i2c_stop(void)
{
    sda_lo(); i2c_delay();
    scl_hi(); i2c_delay();
    sda_hi(); i2c_delay();
}

/* Returns 0 on ACK, 1 on NACK. */
static int i2c_write(uint8_t b)
{
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) sda_hi(); else sda_lo();
        b <<= 1;
        i2c_delay();
        scl_hi(); i2c_delay();
        scl_lo(); i2c_delay();
    }
    sda_hi();                       /* release for ACK */
    i2c_delay();
    scl_hi(); i2c_delay();
    int ack = GPIO_PinInGet(OLED_SDA_PORT, OLED_SDA_PIN);   /* 0 = ACK */
    scl_lo(); i2c_delay();
    return ack;
}

static void oled_cmd(uint8_t c)
{
    i2c_start();
    i2c_write((uint8_t)(OLED_I2C_ADDR << 1));   /* write */
    i2c_write(0x00);                            /* Co=0 D/C=0 -> command */
    i2c_write(c);
    i2c_stop();
}

void oled_flush(void)
{
    oled_cmd(0x21); oled_cmd(0x00); oled_cmd(0x7F);   /* column 0..127 */
    oled_cmd(0x22); oled_cmd(0x00); oled_cmd(0x07);   /* page   0..7   */

    i2c_start();
    i2c_write((uint8_t)(OLED_I2C_ADDR << 1));
    i2c_write(0x40);                                  /* data stream */
    for (unsigned i = 0; i < sizeof(fb); i++) i2c_write(fb[i]);
    i2c_stop();
}

bool oled_init(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    /* Open-drain (wired-AND) with line released high; external pull-ups. */
    GPIO_PinModeSet(OLED_SCL_PORT, OLED_SCL_PIN, gpioModeWiredAnd, 1);
    GPIO_PinModeSet(OLED_SDA_PORT, OLED_SDA_PIN, gpioModeWiredAnd, 1);

    /* Probe: does the panel ACK its address? */
    i2c_start();
    int ack = i2c_write((uint8_t)(OLED_I2C_ADDR << 1));
    i2c_stop();
    if (ack != 0) return false;

    static const uint8_t seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF,
    };
    for (unsigned i = 0; i < sizeof(seq); i++) oled_cmd(seq[i]);

    oled_clear();
    oled_flush();
    return true;
}

#endif /* OLED_HOST_TEST */

/* tft.h - ILI9341 status display + hardware-scrolled waterfall
 *
 * Bus: SPI2 (IO_MUX 9-14). The FG23 slave is on SPI3 -> no conflict.
 * TE: NOT wired (pin 47 belongs to the RGB LED) -> software TE via 0x45.
 *
 * Layout:
 *   0..49    fixed header (status text, every 2 s)
 *   50..319  hardware-scrolled waterfall (0x33 / 0x37)
 *
 * Drawing runs in its OWN task (core 0, priority 1), so it does not block
 * the main loop or the FG23 SPI service. Measured in the bench: 135 us/row CPU.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float       blokk_s;
    uint32_t    sps;
    float       csucs_dbfs;
    int         rssi_dbm;
    uint32_t    ram_k;
    uint32_t    ram_min_k;
    uint8_t     terheles_pct;
    uint32_t    lost;
    uint32_t    freq_khz;
    const char *ip;
    const char *ssid;
} tft_stat_t;

void tft_init(void);
void tft_show(const tft_stat_t *s);      /* header, every 2 s */
/* Current UI page and demod mode in the header (drawn at once, so a button
 * press has visible feedback - the UI pages themselves live on the OLED,
 * which the ionos-sdr-mini does not have). */
void tft_set_page(const char *page, const char *mode);
bool tft_ok(void);

/* One scan row into the waterfall. Call from the same place as
 * oled_spec.push(). Non-blocking: copies into a row buffer and notifies the
 * display task. If the task is still busy with the previous row, the row is
 * dropped (the display must never slow down the radio). */
void tft_push_specline(const uint8_t *bins, uint16_t nbin);

/* The scan band - for the header frequency scale. Sufficient to call on change. */
void tft_set_span(uint32_t center_hz, uint32_t span_hz);

/* Automatic dynamic-range fitting on the waterfall. Needed for SCAN data
 * (absolute dBm scale with an unknown floor), NOT for the FFT: that arrives
 * already normalised, and a second stretch would spread the NOISE VARIANCE
 * across the whole palette. Two scalers in series = saturated, mushy image. */
void tft_set_autoscale(bool on);

/* Number of dropped rows - diagnostics */
uint32_t tft_dropped(void);
/* Bring-up aid (console 'T<vsp>' / 'Tx'): freeze the waterfall, paint test
 * bands (red header area, green scroll area, white ticks every 20 px) and
 * set the scroll start address - shows how the panel maps scroll vs. MADCTL. */
void tft_test(int vsp);
void tft_test_end(void);

/* ---- overlay drawing for the menu (menu.cpp) ----
 * tft_overlay(true) freezes the waterfall and clears the scroll area (y 50..319),
 * the header stays; tft_overlay(false) clears it again and the waterfall resumes.
 * Colours are RGB565. Text uses the built-in 6x8 font scaled by 'size'. */
#define TFTC_BLACK    0x0000
#define TFTC_WHITE    0xFFFF
#define TFTC_CYAN     0x07FF
#define TFTC_YELLOW   0xFFE0
#define TFTC_GREEN    0x07E0
#define TFTC_RED      0xF800
#define TFTC_DARKGREY 0x7BEF
#define TFTC_NAVY     0x000F
#define TFT_W 240
#define TFT_H 320
#define TFT_HDR_H 50
void tft_overlay(bool on);
bool tft_overlay_on(void);
void tft_fill(int x, int y, int w, int h, uint16_t color);
void tft_text(int x, int y, int size, uint16_t fg, uint16_t bg, const char *s);

#ifdef __cplusplus
}
#endif

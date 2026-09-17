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

#ifdef __cplusplus
}
#endif

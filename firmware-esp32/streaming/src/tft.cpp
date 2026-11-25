/* tft.cpp - see tft.h
 *
 * The waterfall architecture comes from fg23waterfallbench, where it was
 * also measured: 135 us/row CPU (map + push), 0.09% load at 6.7 rows/s.
 * The key was NOT the CPU, but that the TE wait must not be a BUSY SPIN.
 * There is no TE wire here (pin 47 belongs to the RGB LED), so software TE
 * is used: 0x45 (Get Scanline) reports where the scan is.
 */

#define LGFX_USE_V1
#include "board.h"
#include <LovyanGFX.hpp>
#include "tft.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <math.h>

#ifndef TFT_OUT
#define TFT_OUT Serial0
#endif

/* ---- pins: the same wiring as in the waterfall bench ---- */
#define PIN_SCK    BRD_TFT_SCK
#define PIN_MOSI   BRD_TFT_MOSI
#define PIN_MISO   BRD_TFT_MISO
#define PIN_CS     BRD_TFT_CS
#define PIN_DC     BRD_TFT_DC
#define PIN_RST    BRD_TFT_RST
#define PIN_BL     BRD_TFT_BL
#define PIN_TE     BRD_TFT_TE                 /* HARDWARE TE. Pin 47 carries the RGB LED;
                                       * two outputs cannot share one wire. */
#define TFT_FREQ   40000000

#define W         240
#define H         320
#define HDR_H      50                 /* fixed header */
#define TFA       HDR_H
#define VSA       (H - HDR_H)         /* 270 scrolled rows */
#define BFA         0                 /* TFA + VSA + BFA == 320 MANDATORY */

class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Bus_SPI       _bus;
    lgfx::Panel_ILI9341 _panel;

    LGFX(void) {
        {
            auto c = _bus.config();
            c.spi_host    = BRD_TFT_HOST;      /* the FG23 slave is on SPI3 */
            c.spi_mode    = 0;
            c.freq_write  = TFT_FREQ;
            c.freq_read   = 16000000;
            c.spi_3wire   = false;
            c.use_lock    = true;
            c.dma_channel = SPI_DMA_CH_AUTO;
            c.pin_sclk    = PIN_SCK;
            c.pin_mosi    = PIN_MOSI;
            c.pin_miso    = PIN_MISO;
            c.pin_dc      = PIN_DC;
            _bus.config(c);
            _panel.setBus(&_bus);
        }
        {
            auto c = _panel.config();
            c.pin_cs           = PIN_CS;
            c.pin_rst          = PIN_RST;
            c.pin_busy         = -1;
            c.panel_width      = W;
            c.panel_height     = H;
            c.memory_width     = W;
            c.memory_height    = H;
            c.dummy_read_pixel = 8;
            c.dummy_read_bits  = 0;   /* MADCTL test of 2026-08-18 */
            c.readable         = true;
            c.invert           = false;
            c.rgb_order        = false;
            c.dlen_16bit       = false;
            c.bus_shared       = true;
            _panel.config(c);
        }
        setPanel(&_panel);
    }
};

static LGFX  lcd;
static bool  s_ok = false;
bool tft_ok(void) { return s_ok; }

/* THE PANEL IS USED BY TWO TASKS: tft_task writes the waterfall rows,
 * loop() writes the header every 2 s. WITHOUT a mutex the two command
 * streams interleave, and one corrupted control command turns the WHOLE
 * panel black (header included). 2026-08-19: exactly that happened. */
static SemaphoreHandle_t s_lcd_mtx = NULL;

static inline void lcd_lock(void)
{
    if (s_lcd_mtx) xSemaphoreTake(s_lcd_mtx, portMAX_DELAY);
}
static inline void lcd_unlock(void)
{
    if (s_lcd_mtx) xSemaphoreGive(s_lcd_mtx);
}

/* ---- raw control commands (LovyanGFX does not expose them) ---- */
static inline void raw_cmd(uint8_t c) { lcd._bus.writeCommand(c, 8); }
static inline void raw_dat(uint8_t d) { lcd._bus.writeData(d, 8); }

static void vscrdef(uint16_t tfa, uint16_t vsa, uint16_t bfa)
{
    lcd.startWrite();
    raw_cmd(0x33);
    raw_dat(tfa >> 8); raw_dat(tfa & 0xFF);
    raw_dat(vsa >> 8); raw_dat(vsa & 0xFF);
    raw_dat(bfa >> 8); raw_dat(bfa & 0xFF);
    lcd.endWrite();
}
static inline void vscrsadd_inTx(uint16_t vsp)
{
    raw_cmd(0x37);
    raw_dat(vsp >> 8); raw_dat(vsp & 0xFF);
}

/* ===================== palette =========================================
 * PURE colour gradient 0..255. The dynamic-range fitting is NOT baked in
 * here but done at run time (auto_scale). The bench had WF_FLOOR/WF_GAIN
 * hard-coded, but that was tuned to the FAKE generator: on real data
 * (noise floor -79..-101 dBm in the -120+70 dB window = raw values
 * 60..160) it saturated the palette -> first a uniform yellow, then a
 * white image. */
static lgfx::rgb565_t pal[256];

static void build_palette(void)
{
    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f;
        uint8_t r, g, b;
        if      (t < 0.15f) { float u = t / 0.15f;           r = 0;                  g = 0;                        b = (uint8_t)(110 * u); }
        else if (t < 0.35f) { float u = (t - 0.15f) / 0.20f; r = 0;                  g = (uint8_t)(60 * u);        b = (uint8_t)(110 + 145 * u); }
        else if (t < 0.55f) { float u = (t - 0.35f) / 0.20f; r = 0;                  g = (uint8_t)(60 + 195 * u);  b = 255; }
        else if (t < 0.70f) { float u = (t - 0.55f) / 0.15f; r = 0;                  g = 255;                      b = (uint8_t)(255 * (1 - u)); }
        else if (t < 0.82f) { float u = (t - 0.70f) / 0.12f; r = (uint8_t)(255 * u); g = 255;                      b = 0; }
        else if (t < 0.93f) { float u = (t - 0.82f) / 0.11f; r = 255;                g = (uint8_t)(255 * (1 - u)); b = 0; }
        else                { float u = (t - 0.93f) / 0.07f; r = 255;                g = (uint8_t)(255 * u);       b = (uint8_t)(255 * u); }
        pal[i] = lgfx::rgb565_t(r, g, b);
    }
}

/* ---- automatic dynamic-range fitting ----
 * Per row the MINIMUM and the 90th PERCENTILE (not the max: a strong
 * carrier must not push down the whole image), tracked slowly so the
 * waterfall does not "breathe". It thus adapts by itself to any noise
 * floor and any floor/range setting. */
static float s_lo = 40.0f, s_hi = 200.0f;
static bool  s_autoscale = true;

void tft_set_autoscale(bool on)
{
    if (on == s_autoscale) return;
    s_autoscale = on;
    if (!on) { s_lo = 0.0f; s_hi = 255.0f; }   /* switch to 1:1 */
}

static void auto_scale(const uint8_t *row)
{
    if (!s_autoscale) return;
    static uint16_t hist[256];
    memset(hist, 0, sizeof hist);

    /* ZERO bins are SKIPPED: they fall outside the tuning grid, and the
     * FG23 reports floor (0) for them. If counted, the scale would settle at
     * lo=0 and the real signal would overshoot the top of the palette ->
     * magenta/white image. */
    uint16_t nvalid = 0;
    uint8_t  mn = 255;
    for (int i = 0; i < W; i++) {
        uint8_t v = row[i];
        if (v == 0) continue;
        hist[v]++; nvalid++;
        if (v < mn) mn = v;
    }
    if (nvalid < 10) return;                  /* too little data - keep the old one */

    int cel = (nvalid * 9) / 10, acc = 0, p90 = 255;
    for (int v = 1; v < 256; v++) { acc += hist[v]; if (acc >= cel) { p90 = v; break; } }

    float lo = (float)mn;
    float hi = (float)p90 + 12.0f;            /* headroom for the peaks */
    if (hi < lo + 20.0f) hi = lo + 20.0f;

    s_lo = s_lo * 0.90f + lo * 0.10f;
    s_hi = s_hi * 0.90f + hi * 0.10f;

    /* diagnostics every 2 s: what the scaler sees */
    static uint32_t t_dbg = 0;
    if (millis() - t_dbg > 2000) {
        t_dbg = millis();
        TFT_OUT.printf("TFT: valid=%u/%d min=%u p90=%d | scale lo=%.0f hi=%.0f\n",
                       nvalid, W, mn, p90, s_lo, s_hi);
    }
}

static inline uint8_t scale_px(uint8_t v)
{
    float d = s_hi - s_lo;
    if (d < 1.0f) d = 1.0f;
    float t = ((float)v - s_lo) * 255.0f / d;
    if (t < 0.0f)   t = 0.0f;
    if (t > 255.0f) t = 255.0f;
    return (uint8_t)t;
}

/* ===================== row hand-over to the task ======================= */
static uint8_t             s_row[W];          /* dB values, already scaled to 240 */
static volatile bool       s_row_full = false;
static lgfx::rgb565_t      s_line[W];         /* converted to colour             */
static TaskHandle_t        s_task = NULL;
static volatile uint32_t   s_drop = 0;
static uint16_t            s_vsp  = TFA;
static volatile bool       s_test = false;

uint32_t tft_dropped(void) { return s_drop; }

/* Bring-up aid: test bands + explicit scroll start (console 'T<vsp>', 'Tx').
 * The header refresh (tft_show, every 2 s) keeps drawing on top - that is
 * part of the test: it shows where screen y<50 lands relative to the bands. */
void tft_test(int vsp)
{
    if (!s_ok) return;
    s_test = true;
    lcd_lock();
    lcd.startWrite();
    lcd.fillRect(0, 0, W, HDR_H, TFT_RED);
    lcd.fillRect(0, HDR_H, W, H - HDR_H, TFT_GREEN);
    lcd.fillRect(0, HDR_H, W, 3, TFT_BLUE);                 /* header/scroll boundary */
    for (int y = 0; y < H; y += 20) lcd.fillRect(0, y, 30 + y / 10, 2, TFT_WHITE);  /* ticks, longer downwards */
    lcd.setTextSize(2);
    lcd.setTextColor(TFT_WHITE, TFT_RED);   lcd.setCursor(60, 16);  lcd.print("HDR y0-49");
    lcd.setTextColor(TFT_BLACK, TFT_GREEN); lcd.setCursor(60, 100); lcd.print("y100");
    lcd.setCursor(60, 200); lcd.print("y200");
    lcd.setCursor(60, 300); lcd.print("y300");
    if (vsp >= 0) vscrsadd_inTx((uint16_t)vsp);
    lcd.endWrite();
    lcd_unlock();
    TFT_OUT.printf("TFT-TEST: bands painted, rotation %d, vscrdef %d/%d/%d, VSP=%d ('Tx' ends)\n",
                   BRD_TFT_ROTATION, TFA, VSA, BFA, vsp);
}

void tft_test_end(void)
{
    if (!s_ok) return;
    lcd_lock();
    lcd.startWrite();
    lcd.fillRect(0, HDR_H, W, H - HDR_H, TFT_BLACK);
    vscrsadd_inTx(s_vsp);
    lcd.endWrite();
    lcd_unlock();
    s_test = false;
    TFT_OUT.println("TFT-TEST: end");
}

/* ===================== menu overlay ===================================
 * Same freeze flag as the band test: while it is set the display task
 * drops the rows (the radio never waits). The scroll start address is left
 * alone, so the waterfall continues where it stopped. */
void tft_overlay(bool on)
{
    if (!s_ok) return;
    if (on) s_test = true;                 /* freeze first, then clear */
    lcd_lock();
    lcd.startWrite();
    lcd.fillRect(0, HDR_H, W, H - HDR_H, TFT_BLACK);
    /* The menu draws in screen coordinates, but the panel shows the scroll
     * area rotated by the vertical scroll pointer the waterfall left behind
     * -> the list appeared shifted after the first open/close. Park the
     * pointer at the top of the scroll area while the overlay is up, give
     * it back to the waterfall on close (its ring keeps s_vsp). */
    vscrsadd_inTx(on ? (uint16_t)TFA : s_vsp);
    lcd.endWrite();
    lcd_unlock();
    if (!on) s_test = false;
}

bool tft_overlay_on(void) { return s_test; }

void tft_fill(int x, int y, int w, int h, uint16_t color)
{
    if (!s_ok) return;
    lcd_lock();
    lcd.fillRect(x, y, w, h, color);
    lcd_unlock();
}

void tft_text(int x, int y, int size, uint16_t fg, uint16_t bg, const char *s)
{
    if (!s_ok || !s) return;
    lcd_lock();
    lcd.setTextSize(size);
    lcd.setTextColor(fg, bg);
    lcd.setCursor(x, y);
    lcd.print(s);
    lcd_unlock();
}

/* nbin -> 240 pixels with MAX-hold (correct for a spectrum, unlike an
 * average: a narrow carrier must not vanish in the downscaling) */
void tft_push_specline(const uint8_t *bins, uint16_t nbin)
{
    if (!s_ok || !bins || nbin == 0) return;
    if (s_row_full) { s_drop++; return; }     /* the display never slows the radio */

    for (int x = 0; x < W; x++) {
        uint32_t a = (uint32_t)x * nbin / W;
        uint32_t b = (uint32_t)(x + 1) * nbin / W;
        if (b <= a) b = a + 1;
        if (b > nbin) b = nbin;
        uint8_t m = 0;
        for (uint32_t i = a; i < b; i++) if (bins[i] > m) m = bins[i];
        s_row[x] = m;
    }
    s_row_full = true;
    if (s_task) xTaskNotifyGive(s_task);
}

/* ===================== hardware TE =======================================
 * Panel pin 22 (TE) on GPIO48. The interrupt only signals; the task
 * BLOCKS on it - no spinning. This was the key in the bench: the spinning
 * variant consumed 9.4 ms of CPU per row, the blocking one zero.
 *
 * Why it is needed at all: the frame-memory row being overwritten is, at
 * the moment of writing, STILL visible at the bottom of the scroll area
 * (it is the oldest row). If the panel is scanning there at that moment,
 * the bright points of the new row flash on the bottom row - this was the
 * "bottom pixel row flickers" symptom. */
static SemaphoreHandle_t s_te_sem = NULL;

static void IRAM_ATTR te_isr(void)
{
    BaseType_t hpw = pdFALSE;
    if (s_te_sem) xSemaphoreGiveFromISR(s_te_sem, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}

static void wait_te(void)
{
    if (!s_te_sem) return;
    xSemaphoreTake(s_te_sem, 0);                    /* discard a stale signal */
    xSemaphoreTake(s_te_sem, pdMS_TO_TICKS(50));    /* the next V-blank     */
}

/* on-chip view (bottom of the file) */
static volatile bool s_vw_pending;
static void view_render(void);

/* ===================== display task ==================================== */
static void tft_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* BLOCKS, no spinning */
        if (s_vw_pending) { if (s_test) view_render(); s_vw_pending = false; }
        if (!s_row_full) continue;
        if (s_test) { s_row_full = false; continue; }  /* test pattern on screen */

        auto_scale(s_row);
        /* 0 = "outside the grid, no measurement" -> dark grey, so the extent
         * of the real band is visible. Not black, because that would blend
         * with the noise floor. */
        for (int i = 0; i < W; i++) {
            s_line[i] = (s_row[i] == 0) ? lgfx::rgb565_t(40, 40, 40)
                                        : pal[scale_px(s_row[i])];
        }
        s_row_full = false;                        /* the buffer may be written again */

        /* new row at the TOP of the scroll area, then step.
         * Rotation 2 (panel upside-down on the ionos-sdr-mini cradle) needs
         * NO special arithmetic: measured 2026-11-03 with the 'T<vsp>' band
         * test, on this ILI9341 the MADCTL MY/MX mirror applies to the gate
         * scan, so the 33h/37h scroll registers work in SCREEN coordinates
         * exactly as in rotation 0 (top fixed area = the header at y 0..49,
         * scroll area y 50..319). A "physical memory row" model (fixed area
         * at the bottom, 0/270/50) put the header into the scrolled ring. */
        uint16_t new_vsp = (s_vsp == TFA) ? (uint16_t)(TFA + VSA - 1)
                                          : (uint16_t)(s_vsp - 1);
        wait_te();

        lcd_lock();
        lcd.startWrite();
        lcd.pushImageDMA(0, new_vsp, W, 1, s_line);
        lcd.waitDMA();
        vscrsadd_inTx(new_vsp);
        lcd.endWrite();
        lcd_unlock();

        s_vsp = new_vsp;
    }
}

/* ===================== initialisation ================================== */
void tft_init(void)
{
    s_lcd_mtx = xSemaphoreCreateMutex();
    pinMode(PIN_BL, OUTPUT);
    digitalWrite(PIN_BL, HIGH);

    if (!lcd.init()) {
        TFT_OUT.println("TFT: init FAILED (wiring? power? JP jumper?)");
        s_ok = false;
        return;
    }
    s_ok = true;
    lcd.setRotation(BRD_TFT_ROTATION);
    lcd.setTextWrap(false);        /* a padded field at x=150 must NOT wrap to x=0 of the next line (it clipped "Ionos") */
    lcd.fillScreen(TFT_BLACK);
    build_palette();

    /* All text on the display is ENGLISH - the product name is Ionos SDR. */
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setCursor(6, 4);
    lcd.print("Ionos SDR");
    lcd.drawFastHLine(0, HDR_H - 1, W, TFT_DARKGREY);

    /* scroll area: the header is fixed, everything below scrolls */
    vscrdef(TFA, VSA, BFA);        /* screen coordinates in every rotation (see tft_task) */
    s_vsp = TFA;

    /* Enable TEARING EFFECT on the panel (0x35, mode 0 = V-blank only),
     * and the GPIO48 interrupt. Without it the bottom pixel row flickers. */
    lcd.startWrite();
    raw_cmd(0x35); raw_dat(0x00);
    lcd.endWrite();

    s_te_sem = xSemaphoreCreateBinary();
    pinMode(PIN_TE, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_TE), te_isr, RISING);

    /* Is TE alive? ~60-110 edges must arrive within 200 ms. */
    {
        uint32_t n = 0, t0 = millis();
        while (millis() - t0 < 200) {
            if (xSemaphoreTake(s_te_sem, pdMS_TO_TICKS(20)) == pdTRUE) n++;
        }
        TFT_OUT.printf("TFT: TE (panel 22 -> GPIO%d): %lu edges / 200 ms -> %s\n",
                       PIN_TE, (unsigned long)n,
                       n > 5 ? "ALIVE, using it" : "NOT ARRIVING - check the wire");
    }

    xTaskCreatePinnedToCore(tft_task, "tft", 5120, NULL, 1, &s_task, 0);
    TFT_OUT.printf("TFT: ILI9341 alive (SPI2, 40 MHz, %dx%d, header %d px, "
                   "waterfall %d rows)\n", W, H, HDR_H, VSA);
}

/* ===================== frequency scale ================================= */
static uint32_t s_cf_hz = 0, s_span_hz = 0;
static bool     s_scale_dirty = false;

void tft_set_span(uint32_t center_hz, uint32_t span_hz)
{
    if (center_hz == s_cf_hz && span_hz == s_span_hz) return;
    s_cf_hz = center_hz; s_span_hz = span_hz;
    s_scale_dirty = true;
}

static char     s_st_val[16] = "", s_st_line[44] = "";
static uint16_t s_st_col = TFT_DARKGREY, s_st_line_col = TFT_WHITE;

static void draw_state_line(void)
{
    lcd.fillRect(0, 39, W, 10, TFT_BLACK);
    lcd.setTextSize(1);
    lcd.setTextColor(s_st_line_col, TFT_BLACK);
    int x = (W - 6 * (int)strlen(s_st_line)) / 2; if (x < 0) x = 0;
    lcd.setCursor(x, 40);
    lcd.print(s_st_line);
}

static void draw_scale(void)
{
    if (s_st_line[0]) return;               /* on-chip: the state line owns the row */
    if (!s_scale_dirty || !s_span_hz) return;
    s_scale_dirty = false;

    double lo = (s_cf_hz - s_span_hz / 2.0) / 1e6;
    double hi = (s_cf_hz + s_span_hz / 2.0) / 1e6;
    double cf = s_cf_hz / 1e6;
    char b[24];

    lcd.fillRect(0, 39, W, 10, TFT_BLACK);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    snprintf(b, sizeof b, "%.3f", lo);          /* left edge */
    lcd.setCursor(2, 40);
    lcd.print(b);

    snprintf(b, sizeof b, "%.3f MHz", cf);      /* centre */
    lcd.setCursor(W / 2 - 27, 40);
    lcd.print(b);

    snprintf(b, sizeof b, "%.3f", hi);          /* right edge */
    lcd.setCursor(W - 38, 40);
    lcd.print(b);

    /* tick marks below the scale: 5 divisions */
    for (int i = 0; i <= 4; i++)
        lcd.drawFastVLine(i * (W - 1) / 4, 47, 2, TFT_DARKGREY);
}

/* ===================== header ========================================== */
static void mezo(int x, int y, const char *cimke, const char *ertek, uint16_t szin)
{
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(x, y);
    lcd.print(cimke);
    lcd.setTextColor(szin, TFT_BLACK);
    lcd.setCursor(x, y + 10);
    lcd.print(ertek);
    lcd.print("   ");
}

void tft_show(const tft_stat_t *s)
{
    if (!s_ok || !s) return;
    char b[32];

    lcd_lock();
    if (s_st_val[0]) {
        mezo(6, 20, "FG23", s_st_val, s_st_col);      /* radio_sm state, see tft_set_state */
    } else {
        uint16_t allapot = (s->blokk_s > 100.0f) ? TFT_GREEN : TFT_RED;
        snprintf(b, sizeof b, "%.0f blk/s", s->blokk_s);
        mezo(6, 20, "RADIO", b, allapot);
    }

    snprintf(b, sizeof b, "%d dBm", s->rssi_dbm);
    mezo(78, 20, "SIGNAL", b, TFT_YELLOW);

    snprintf(b, sizeof b, "%lu k", (unsigned long)s->ram_min_k);
    mezo(140, 20, "FREE", b, (s->ram_min_k < 15) ? TFT_RED : TFT_WHITE);

    snprintf(b, sizeof b, "%u%%", s->terheles_pct);
    mezo(196, 20, "LOAD", b, (s->terheles_pct > 80) ? TFT_ORANGE : TFT_WHITE);

    /* IP next to the title, small (the page/mode line sits right under it) */
    lcd.setTextSize(1);
    lcd.setTextColor(s->lost ? TFT_RED : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(150, 2);
    char v[24];
    if (s->lost) snprintf(v, sizeof v, "LOST %lu", (unsigned long)s->lost);
    else         snprintf(v, sizeof v, "%s", s->ip ? s->ip : "-");
    snprintf(b, sizeof b, "%-15.15s", v);           /* exactly 15 chars = 90 px, fits x 150..239 */
    lcd.print(b);

    draw_scale();          /* draws only when the band changed */
    lcd_unlock();
}

void tft_set_page(const char *page, const char *mode)
{
    if (!s_ok) return;
    char v[32], b[20];
    snprintf(v, sizeof v, "%s>%s", page ? page : "-", mode ? mode : "-");
    snprintf(b, sizeof b, "%-15.15s", v);           /* 15 chars, no wrap (see tft_show) */
    lcd_lock();
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setCursor(150, 11);
    lcd.print(b);
    lcd_unlock();
}

void tft_set_state(const char *val, uint16_t col, const char *line, uint16_t line_col)
{
    if (!s_ok) return;
    bool had_line = s_st_line[0] != 0;
    snprintf(s_st_val, sizeof s_st_val, "%-11.11s", val ? val : "");
    s_st_col = col;
    snprintf(s_st_line, sizeof s_st_line, "%.40s", line ? line : "");
    s_st_line_col = line_col;
    lcd_lock();
    mezo(6, 20, "FG23", s_st_val, s_st_col);
    if (s_st_line[0]) draw_state_line();
    else if (had_line) { lcd.fillRect(0, 39, W, 10, TFT_BLACK); s_scale_dirty = true; draw_scale(); }
    lcd_unlock();
}

/* ===================== on-chip view ===================================== */
static tft_view_t *s_vw = NULL;           /* hand-over copy (PSRAM) */
/* s_vw_pending: defined above tft_task */
static LGFX_Sprite *s_spr = NULL;

void tft_view_show(const tft_view_t *v)
{
    if (!s_ok || !v || !s_task) return;
    if (!s_vw) s_vw = (tft_view_t *)heap_caps_malloc(sizeof(tft_view_t), MALLOC_CAP_SPIRAM);
    if (!s_vw || s_vw_pending) return;     /* the previous one is still being drawn: skip */
    memcpy(s_vw, v, sizeof *v);
    s_vw_pending = true;
    xTaskNotifyGive(s_task);
}

static uint16_t alt_col(const tv_ac_t *a)
{
    if (!a->alt_ok) return TFT_LIGHTGREY;
    if (a->alt < 10000) return TFT_GREEN;
    if (a->alt < 25000) return TFT_YELLOW;
    if (a->alt < 35000) return TFT_ORANGE;
    return TFT_MAGENTA;
}


/* ---- on-chip ADS-B: base map (map_data.h, Natural Earth) + aircraft + translucent text bands ---- */
#include "map_data.h"
static inline uint16_t sw16(uint16_t c) { return (uint16_t)((c >> 8) | (c << 8)); }   /* sprite buffer is byte-swapped RGB565 */
static void band_dim(uint16_t *buf, int y0, int y1)          /* 50 % darker: text stays readable, map shows through */
{
    for (int y = y0; y < y1; y++) for (int x = 0; x < W; x++) {
        uint16_t c = sw16(buf[y * W + x]);
        buf[y * W + x] = sw16((uint16_t)((c >> 1) & 0x7BEF));
    }
}
static uint16_t alt_col565(const tv_ac_t *a)                  /* web map colours: hsl(30+t*270, 85 %, 45 %) */
{
    if (!a->alt_ok) return 0x8410;
    float t = a->alt / 40000.0f; if (t < 0) t = 0; if (t > 1) t = 1;
    float h = 30 + t * 270, s = 0.85f, l = 0.45f;
    float c = (1 - fabsf(2 * l - 1)) * s, hp = h / 60.0f, x = c * (1 - fabsf(fmodf(hp, 2) - 1)), m = l - c / 2;
    float r = 0, gg = 0, b = 0;
    if (hp < 1) { r = c; gg = x; } else if (hp < 2) { r = x; gg = c; } else if (hp < 3) { gg = c; b = x; }
    else if (hp < 4) { gg = x; b = c; } else if (hp < 5) { r = x; b = c; } else { r = c; b = x; }
    return lcd.color565((uint8_t)((r + m) * 255), (uint8_t)((gg + m) * 255), (uint8_t)((b + m) * 255));
}
static void map_render(LGFX_Sprite &g, const tft_view_t *v)
{
    const map_level_t *L = &MAP_LEVELS[v->map_level < MAP_NLEV ? v->map_level : 1];
    uint16_t *buf = (uint16_t *)g.getBuffer();
    uint16_t pal[64];
    for (int k = 0; k < 64; k++) pal[k] = sw16(L->pal[k]);
    for (int i = 0; i < MAP_W * MAP_H; i++) buf[i] = pal[L->img[i]];
    const float kx = 111.32f * cosf(MAP_LAT0 * (float)M_PI / 180.0f);
    auto X = [&](float lon) { return (lon - MAP_LON0) * kx / L->kmpp + MAP_W / 2; };
    auto Y = [&](float lat) { return (MAP_LAT0 - lat) * 110.574f / L->kmpp + MAP_H / 2; };
    /* station */
    if (!isnan(v->ref_lat)) {
        int sx = (int)X(v->ref_lon), sy = (int)Y(v->ref_lat);
        g.drawCircle(sx, sy, 4, TFT_RED); g.drawCircle(sx, sy, 5, TFT_WHITE); g.fillCircle(sx, sy, 1, TFT_RED);
    }
    /* aircraft: oriented arrow, colour = altitude; labels only where they do not collide */
    int16_t lr[TV_AC][4]; int nl = 0;
    for (int i = 0; i < v->nac; i++) {
        const tv_ac_t *a = &v->ac[i];
        float px = X(a->lon), py = Y(a->lat);
        if (px < -6 || px > W + 6 || py < 10 || py > VSA - 34) continue;
        uint16_t c = alt_col565(a);
        float t = (a->trk >= 0 ? a->trk : 0) * (float)M_PI / 180.0f, sn = sinf(t), cs = cosf(t);
        auto R = [&](float dx, float dy, int &ox, int &oy) { ox = (int)lrintf(px + dx * cs - dy * sn); oy = (int)lrintf(py + dx * sn + dy * cs); };
        int x0, y0_, x1, y1, x2, y2;
        R(0, -7, x0, y0_); R(-4.5f, 5, x1, y1); R(4.5f, 5, x2, y2);
        g.fillTriangle(x0, y0_, x1, y1, x2, y2, c);
        g.drawTriangle(x0, y0_, x1, y1, x2, y2, TFT_BLACK);
        int lw = 6 * (int)strlen(a->lab), lx = (int)px + 7, ly = (int)py - 4;
        if (lx + lw > W) lx = (int)px - 7 - lw;
        bool clash = ly < 11 || ly + 8 > VSA - 34;
        for (int k = 0; k < nl && !clash; k++)
            clash = !(lx + lw < lr[k][0] || lx > lr[k][2] || ly + 8 < lr[k][1] || ly > lr[k][3]);
        if (clash) continue;
        lr[nl][0] = lx; lr[nl][1] = ly; lr[nl][2] = lx + lw; lr[nl][3] = ly + 8; nl++;
        g.setTextColor(TFT_BLACK);
        for (int dx = -1; dx <= 1; dx++) for (int dy = -1; dy <= 1; dy++) if (dx || dy) { g.setCursor(lx + dx, ly + dy); g.print(a->lab); }
        g.setTextColor(TFT_WHITE);
        g.setCursor(lx, ly); g.print(a->lab);
    }
    /* scale bar (bottom left, above the frame band) */
    int km = (L->half_km <= 60) ? 20 : (L->half_km <= 150) ? 50 : 100, bl = (int)lrintf(km / L->kmpp), by = VSA - 40;
    g.fillRect(6, by, bl, 3, TFT_BLACK); g.drawFastVLine(6, by - 2, 7, TFT_BLACK); g.drawFastVLine(6 + bl, by - 2, 7, TFT_BLACK);
    g.setTextColor(TFT_BLACK); g.setCursor(10 + bl, by - 3); char sb[12]; snprintf(sb, sizeof sb, "%d km", km); g.print(sb);
    /* translucent bands: title on top, newest frames at the bottom */
    band_dim(buf, 0, 11);
    g.setTextColor(v->title_c); g.setCursor(2, 2); g.print(v->title);
    int nb = v->nln < 3 ? v->nln : 3, yb = VSA - 1 - 10 * nb;
    if (nb) {
        band_dim(buf, yb - 1, VSA);
        for (int i = 0; i < nb; i++) { g.setTextColor(v->ln[i].c); g.setCursor(2, yb + 1 + 10 * i); g.print(v->ln[i].s); }
    }
}

static void view_render(void)
{
    const tft_view_t *v = s_vw;
    if (!s_spr) {
        s_spr = new LGFX_Sprite(&lcd);
        s_spr->setPsram(true);
        s_spr->setColorDepth(16);
        if (!s_spr->createSprite(W, VSA)) { delete s_spr; s_spr = NULL; TFT_OUT.println("TFT: view sprite alloc FAILED"); return; }
        s_spr->setTextWrap(false);
    }
    LGFX_Sprite &g = *s_spr;
    g.fillSprite(TFT_BLACK);
    g.setTextSize(1);
    g.setTextColor(v->title_c, TFT_BLACK);
    g.setCursor(0, 2);
    g.print(v->title);
    int y0 = 14;
    if (v->kind == 1) map_render(g, v);
    else for (int i = 0; i < v->nln; i++) {
        int y = y0 + 10 * i;
        if (y > VSA - 9) break;
        g.setTextColor(v->ln[i].c, TFT_BLACK);
        g.setCursor(0, y);
        g.print(v->ln[i].s);
    }
    lcd_lock();
    if (s_test) s_spr->pushSprite(0, TFA);         /* only while the overlay is ours (VSP parked) */
    lcd_unlock();
}

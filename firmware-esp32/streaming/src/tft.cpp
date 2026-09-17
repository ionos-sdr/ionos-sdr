/* tft.cpp - see tft.h
 *
 * The waterfall architecture comes from fg23waterfallbench, where it was
 * also measured: 135 us/row CPU (map + push), 0.09% load at 6.7 rows/s.
 * The key was NOT the CPU, but that the TE wait must not be a BUSY SPIN.
 * There is no TE wire here (pin 47 belongs to the RGB LED), so software TE
 * is used: 0x45 (Get Scanline) reports where the scan is.
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "tft.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef TFT_OUT
#define TFT_OUT Serial0
#endif

/* ---- pins: the same wiring as in the waterfall bench ---- */
#define PIN_SCK    12
#define PIN_MOSI   11
#define PIN_MISO   13
#define PIN_CS     10
#define PIN_DC     14
#define PIN_RST     9
#define PIN_BL     21
#define PIN_TE     48                 /* HARDWARE TE. Pin 47 carries the RGB LED;
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
            c.spi_host    = SPI2_HOST;      /* the FG23 slave is on SPI3 */
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

uint32_t tft_dropped(void) { return s_drop; }

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

/* ===================== display task ==================================== */
static void tft_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* BLOCKS, no spinning */
        if (!s_row_full) continue;

        auto_scale(s_row);
        /* 0 = "outside the grid, no measurement" -> dark grey, so the extent
         * of the real band is visible. Not black, because that would blend
         * with the noise floor. */
        for (int i = 0; i < W; i++) {
            s_line[i] = (s_row[i] == 0) ? lgfx::rgb565_t(40, 40, 40)
                                        : pal[scale_px(s_row[i])];
        }
        s_row_full = false;                        /* the buffer may be written again */

        /* new row at the TOP of the scroll area, then step */
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
    lcd.setRotation(0);
    lcd.fillScreen(TFT_BLACK);
    build_palette();

    /* All text on the display is ENGLISH - the product name is Ionos SDR. */
    lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setTextSize(2);
    lcd.setCursor(6, 4);
    lcd.print("Ionos SDR");
    lcd.drawFastHLine(0, HDR_H - 1, W, TFT_DARKGREY);

    /* scroll area: the header is fixed, everything below scrolls */
    vscrdef(TFA, VSA, BFA);
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

    xTaskCreatePinnedToCore(tft_task, "tft", 4096, NULL, 1, &s_task, 0);
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

static void draw_scale(void)
{
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
    uint16_t allapot = (s->blokk_s > 100.0f) ? TFT_GREEN : TFT_RED;
    snprintf(b, sizeof b, "%.0f blk/s", s->blokk_s);
    mezo(6, 20, "RADIO", b, allapot);

    snprintf(b, sizeof b, "%d dBm", s->rssi_dbm);
    mezo(78, 20, "SIGNAL", b, TFT_YELLOW);

    snprintf(b, sizeof b, "%lu k", (unsigned long)s->ram_min_k);
    mezo(140, 20, "FREE", b, (s->ram_min_k < 15) ? TFT_RED : TFT_WHITE);

    snprintf(b, sizeof b, "%u%%", s->terheles_pct);
    mezo(196, 20, "LOAD", b, (s->terheles_pct > 80) ? TFT_ORANGE : TFT_WHITE);

    /* IP next to the title, small */
    lcd.setTextSize(1);
    lcd.setTextColor(s->lost ? TFT_RED : TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(150, 6);
    if (s->lost) snprintf(b, sizeof b, "LOST %lu   ", (unsigned long)s->lost);
    else         snprintf(b, sizeof b, "%s        ", s->ip ? s->ip : "-");
    lcd.print(b);

    draw_scale();          /* draws only when the band changed */
    lcd_unlock();
}

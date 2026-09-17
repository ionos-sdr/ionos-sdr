/* iq_fft.cpp - narrowband I/Q -> waterfall row for the LCD
 *
 * M1 + M2 (2026-08-19): 256 -> 1024-point FFT and Nuttall window.
 *
 * WHY: the smooth SDR++ image does NOT come from averaging (there is no
 * Welch in its source!), but from forming ~1000 pixels out of 65536 bins
 * with MAX-HOLD: the maximum of ~65 bins per pixel, whose variance is a
 * fraction of a single bin's. Here 256 bins went to 240 pixels = 1.07
 * bin/pixel, i.e. ZERO concentration -> mushy image. With 1024 points 4.27
 * bin/pixel, and the resolution improves from 195 to 48.8 Hz/bin.
 *
 * MEASURED (host, 2026-08-19), carrier detuned by half a bin:
 *   Hann    sidelobe  -52.6 dBc
 *   Nuttall sidelobe  -92.5 dBc     <- 40 dB difference
 * FFT time, S3 estimate: 256 ~38 us, 512 ~81 us, 1024 ~178 us.
 * 49 batches/s * 178 us = 0.87% CPU -> NO need to discard data.
 */

#include "iq_fft.h"
#include "tft.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef IQF_OUT
#define IQF_OUT Serial0
#endif

#define NMAX     1024                 /* largest supported FFT            */
#define BLKSAMP   256                 /* samples per I/Q block            */
#define NBINS     240                 /* panel width                      */

static float    s_re[NMAX], s_im[NMAX];
static float    s_win[NMAX];
static float    s_db[NMAX], s_pwr[NMAX];
static uint16_t s_bitrev[NMAX];
static float    s_cos[NMAX / 2], s_sin[NMAX / 2];

/* sample collector: NMAX samples, filled block by block */
static int16_t  s_buf[NMAX * 2];      /* Q,I pairs                        */
static uint16_t s_fill = 0;

static uint16_t s_n     = 1024;       /* current FFT length (256/512/1024) */
static uint16_t s_n_tab = 0;          /* length the tables were built for  */
static bool     s_pwr_ok = false;

static bool     s_on    = true;
static uint8_t  s_decim = 3;          /* DRAW a row every n batches        */
static uint8_t  s_cnt   = 0;
static uint32_t s_cf_hz = 0, s_sps = 50000;

static float    s_floor_db = -60.0f;
static float    s_lift_db  = -2.0f;
static float    s_range_db = 40.0f;
static float    s_alpha    = 0.40f;
static bool     s_invert   = true;
static uint8_t  s_window   = 1;       /* 0 = Hann, 1 = Nuttall             */
static uint8_t  s_dump     = 0;       /* 1 = print the next row             */

/* --- BACKGROUND SUBTRACTION (the "normalize" function of spectrum analysers)
 * Measurement of 2026-08-19: the correlation of two rows seconds apart is
 * r = 0.444. I.e. nearly half of the visible pattern is NOT noise but
 * CONSTANT: the receiver's own frequency response and the fixed spurs. More
 * averaging NEVER removes this (alpha=0.12 brought the random part from
 * 2.79 to 1.41 dB, yet the image showed no improvement) - only subtracting
 * the long-term average does.
 *
 * COST: a continuously transmitting carrier also merges into the
 * background if it stays longer than tau. Hence adjustable and switchable. */
static float    s_base[NMAX];
static bool     s_base_ok  = false;
static uint8_t  s_base_on  = 1;
static float    s_base_tau = 20.0f;   /* seconds                            */

void iq_fft_enable(bool on)      { s_on = on; }
bool iq_fft_enabled(void)        { return s_on; }
void iq_fft_set_decim(uint8_t n) { s_decim = (n < 1) ? 1 : (n > 64 ? 64 : n); }
void iq_fft_set_tuning(uint32_t center_hz, uint32_t sps)
{
    s_cf_hz = center_hz;
    if (sps) s_sps = sps;
}

/* ===================== tables ========================================== */
static void tables_init(uint16_t n)
{
    for (int i = 0; i < n; i++) {
        float x = (float)i / (float)(n - 1);
        if (s_window) {
            /* Nuttall - the same coefficients as in SDR++
             * (core/src/dsp/window/nuttall.h) */
            s_win[i] = 0.355768f
                     - 0.487396f * cosf(2.0f * (float)M_PI * x)
                     + 0.144232f * cosf(4.0f * (float)M_PI * x)
                     - 0.012604f * cosf(6.0f * (float)M_PI * x);
        } else {
            s_win[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * x);
        }
    }

    int bits = 0; while ((1u << bits) < n) bits++;
    for (int i = 0; i < n; i++) {
        uint16_t r = 0, x = i;
        for (int b = 0; b < bits; b++) { r = (r << 1) | (x & 1); x >>= 1; }
        s_bitrev[i] = r;
    }
    for (int i = 0; i < n / 2; i++) {
        s_cos[i] = cosf(-2.0f * (float)M_PI * i / n);
        s_sin[i] = sinf(-2.0f * (float)M_PI * i / n);
    }
    s_n_tab  = n;
    s_pwr_ok = false;                 /* new length -> restart averaging */
}

static void fft_run(uint16_t n)
{
    for (int i = 0; i < n; i++) {
        int j = s_bitrev[i];
        if (j > i) {
            float t;
            t = s_re[i]; s_re[i] = s_re[j]; s_re[j] = t;
            t = s_im[i]; s_im[i] = s_im[j]; s_im[j] = t;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1, step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int j = 0; j < half; j++) {
                float wr = s_cos[j * step], wi = s_sin[j * step];
                int a = i + j, b = a + half;
                float xr = s_re[b] * wr - s_im[b] * wi;
                float xi = s_re[b] * wi + s_im[b] * wr;
                s_re[b] = s_re[a] - xr; s_im[b] = s_im[a] - xi;
                s_re[a] += xr;          s_im[a] += xi;
            }
        }
    }
}

/* ===================== processing one batch ============================
 * The FFT runs for EVERY full batch (so the EMA gets a real average), but a
 * row is pushed only every s_decim-th - otherwise the waterfall would race
 * at 49 rows/s, and 270 rows would show only 5.5 seconds of history. */
static void process(void)
{
    const uint16_t n = s_n;
    if (s_n_tab != n) tables_init(n);

    /* i16le, Q FIRST - the project format (geckokapula-compatible) */
    for (int i = 0; i < n; i++) {
        float q  = (float)s_buf[2 * i + 0];
        float ii = (float)s_buf[2 * i + 1];
        s_re[i] = ii * s_win[i];
        s_im[i] = q  * s_win[i];
    }

    fft_run(n);

    /* power + FFT shift + EMA. A smaller weight suffices, because the
     * max-hold itself concentrates (1024 bins -> 240 pixels = 4.27). */
    for (int k = 0; k < n; k++) {
        int src = (k + n / 2) % n;
        float m = s_re[src] * s_re[src] + s_im[src] * s_im[src];
        s_pwr[k] = s_pwr_ok ? (s_pwr[k] * (1.0f - s_alpha) + m * s_alpha) : m;
    }
    s_pwr_ok = true;

    /* DC spike: the centre bins are the receiver's residual DC offset. The
     * width scales with the LENGTH: +-2 bins at 256, +-8 at 1024. */
    {
        int c = n / 2, w = (int)(n / 128);
        if (w < 2) w = 2;
        float l = s_pwr[c - w - 1], r = s_pwr[c + w + 1];
        for (int d = -w; d <= w; d++) {
            float t = (float)(d + w) / (float)(2 * w);
            s_pwr[c + d] = l * (1.0f - t) + r * t;
        }
    }

    float sum = 0.0f;
    for (int k = 0; k < n; k++) {
        s_db[k] = 10.0f * log10f(s_pwr[k] + 1e-3f);
        sum += s_db[k];
    }

    /* background subtraction: remove the long-term average. The row rate is
     * ~49/s, so for a tau-second time constant the weight is 1/(49*tau). */
    if (s_base_on) {
        if (!s_base_ok) {
            memcpy(s_base, s_db, n * sizeof(float));
            s_base_ok = true;
        } else {
            float ba = 1.0f / (49.0f * s_base_tau);
            if (ba > 0.5f) ba = 0.5f;
            for (int k = 0; k < n; k++)
                s_base[k] += (s_db[k] - s_base[k]) * ba;
        }
        sum = 0.0f;
        for (int k = 0; k < n; k++) { s_db[k] -= s_base[k]; sum += s_db[k]; }
    }

    s_floor_db = s_floor_db * 0.97f + (sum / n) * 0.03f;

    /* draw a row only from every s_decim-th batch */
    if (++s_cnt < s_decim) return;
    s_cnt = 0;

    static uint8_t bins[NBINS];

    /* MAX-HOLD BIAS CORRECTION
     * 1024 bins / 240 pixels = 4.267 -> some pixels take the maximum of 4
     * bins, others of 5. For exponentially distributed power the expected
     * maximum of k bins is the HARMONIC NUMBER H_k, so between the two
     * there is a systematic 10*log10(H5/H4) = 0.40 dB difference.
     * This draws a ripple of 64 bright stripes with a 15-pixel period onto
     * the smooth noise floor (2026-08-19, observed by HA7DCD).
     * Subtracting H_k removes it exactly: measured 0.404 -> 0.013 dB. */
    static float bias_db[17];
    static bool  bias_ok = false;
    if (!bias_ok) {
        float h = 0.0f;
        bias_db[0] = 0.0f;
        for (int k = 1; k <= 16; k++) { h += 1.0f / k; bias_db[k] = 10.0f * log10f(h); }
        bias_ok = true;
    }

    for (int x = 0; x < NBINS; x++) {
        int lo_k = (int)((uint32_t)x * n / NBINS);
        int hi_k = (int)((uint32_t)(x + 1) * n / NBINS);
        if (hi_k <= lo_k) hi_k = lo_k + 1;
        float best = -200.0f;
        for (int k = lo_k; k < hi_k && k < n; k++)
            if (s_db[k] > best) best = s_db[k];
        int cnt = hi_k - lo_k;
        if (cnt > 16) cnt = 16;
        best -= bias_db[cnt];
        float t = (best - s_floor_db - s_lift_db) * 255.0f / s_range_db;
        if (t < 1.0f)   t = 1.0f;
        if (t > 255.0f) t = 255.0f;
        bins[s_invert ? (NBINS - 1 - x) : x] = (uint8_t)t;
    }

    tft_set_autoscale(false);          /* ONE scaler only: this one */
    tft_push_specline(bins, NBINS);
    if (s_cf_hz) tft_set_span(s_cf_hz, s_sps);

    /* Diagnostics: print one raw row so that the ripple period can be
     * MEASURED rather than guessed. dB values are given in 0.1 dB units. */
    if (s_dump) {
        s_dump = 0;
        IQF_OUT.printf("FFTDUMP n=%u win=%u floor=%.1f\n", n, s_window, s_floor_db);
        for (int k = 0; k < n; k++) {
            IQF_OUT.printf("%d%c", (int)(s_db[k] * 10.0f),
                           ((k & 15) == 15) ? '\n' : ' ');
        }
        IQF_OUT.println("FFTDUMP-END");
    }
}

/* ===================== block input ===================================== */
void iq_fft_push_block(const uint8_t *payload)
{
    if (!s_on || !payload) return;

    /* Collection: s_n samples are needed, 256 per block. */
    if (s_fill + BLKSAMP <= s_n) {
        memcpy(&s_buf[s_fill * 2], payload, BLKSAMP * 2 * sizeof(int16_t));
        s_fill += BLKSAMP;
    }
    if (s_fill < s_n) return;
    s_fill = 0;

    process();
}

/* ===================== commands ======================================== */
static void iqf_print_cfg(void)
{
    IQF_OUT.printf("FFT-CFG: n=%u window=%s lift=%.1f range=%.1f alpha=%.2f "
                   "dec=%u inv=%u base=%u tau=%.0fs | floor=%.1f dB "
                   "resolution=%.1f Hz/bin span=%lu Hz run=%d\n",
                   s_n, s_window ? "Nuttall" : "Hann", s_lift_db, s_range_db,
                   s_alpha, s_decim, (unsigned)s_invert, s_base_on, s_base_tau,
                   s_floor_db, (double)s_sps / s_n, (unsigned long)s_sps,
                   (int)s_on);
}

bool iq_fft_cmd(const char *line)
{
    if (!line || line[0] != 'F') return false;
    const char *p = line + 1;
    while (*p == ' ') p++;

    if (*p == 0) { iqf_print_cfg(); return true; }
    if (p[0] == '0') { s_on = false; IQF_OUT.println("FFT: off"); return true; }
    if (p[0] == '1') { s_on = true;  IQF_OUT.println("FFT: on"); return true; }
    if (!strncmp(p, "dump", 4)) { s_dump = 1; return true; }

    if (!strncmp(p, "set", 3)) {
        char buf[128];
        strncpy(buf, p + 3, sizeof buf - 1);
        buf[sizeof buf - 1] = 0;
        char *save = NULL;
        for (char *t = strtok_r(buf, " \t,", &save); t;
             t = strtok_r(NULL, " \t,", &save)) {
            char *eq = strchr(t, '=');
            if (!eq) continue;
            *eq = 0;
            double v = atof(eq + 1);
            if      (!strcmp(t, "lift"))  s_lift_db  = (float)v;
            else if (!strcmp(t, "range")) s_range_db = (float)(v < 5 ? 5 : v);
            else if (!strcmp(t, "alpha")) s_alpha    = (float)(v / 100.0);
            else if (!strcmp(t, "dec"))   iq_fft_set_decim((uint8_t)v);
            else if (!strcmp(t, "inv"))   s_invert   = (v != 0);
            else if (!strcmp(t, "win"))   { s_window = (v != 0); s_n_tab = 0; }
            else if (!strcmp(t, "base"))  { s_base_on = (v != 0); s_base_ok = false; }
            else if (!strcmp(t, "tau"))   { s_base_tau = (float)(v < 1 ? 1 : v);
                                            s_base_ok = false; }
            else if (!strcmp(t, "n")) {
                uint16_t nn = (uint16_t)v;
                if (nn == 256 || nn == 512 || nn == 1024) {
                    s_n = nn; s_fill = 0; s_n_tab = 0;
                } else {
                    IQF_OUT.println("FFT: n must be 256 / 512 / 1024");
                }
            }
            else IQF_OUT.printf("FFT: unknown key: %s\n", t);
        }
        if (s_alpha < 0.02f) s_alpha = 0.02f;
        if (s_alpha > 1.0f)  s_alpha = 1.0f;
        iqf_print_cfg();
        return true;
    }

    IQF_OUT.println("FFT: F | F0 | F1 | "
                    "Fset n=1024 win=1 lift=-2 range=40 alpha=40 dec=3 inv=1");
    return true;
}

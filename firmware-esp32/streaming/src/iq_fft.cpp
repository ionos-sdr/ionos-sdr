/* iq_fft.cpp - keskenysavu I/Q -> waterfall-sor az LCD-re
 *
 * M1 + M2 (2026-08-19): 256 -> 1024 pontos FFT es Nuttall-ablak.
 *
 * MIERT: az SDR++ sima kepe NEM atlagolasbol jon (nincs Welch a
 * forrasaban!), hanem abbol, hogy 65536 binbol kepez ~1000 pixelt
 * MAX-TARTASSAL: pixelenkent ~65 bin maximuma, aminek a szorasa toredeke
 * egyetlen bine. Nalunk 256 bin ment 240 pixelre = 1,07 bin/pixel, vagyis
 * NULLA koncentracio -> kasas kep. 1024 ponttal 4,27 bin/pixel, es a
 * felbontas is 195 -> 48,8 Hz/bin.
 *
 * MERVE (hoszt, 2026-08-19), fel binnel elhangolt vivore:
 *   Hann    oldalnyalab  -52,6 dBc
 *   Nuttall oldalnyalab  -92,5 dBc     <- 40 dB kulonbseg
 * FFT-ido S3-becsles: 256 ~38 us, 512 ~81 us, 1024 ~178 us.
 * 49 adag/s * 178 us = 0,87% CPU -> NEM kell adatot eldobni.
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

#define NMAX     1024                 /* legnagyobb tamogatott FFT        */
#define BLKSAMP   256                 /* egy I/Q blokk mintaszama         */
#define NBINS     240                 /* a panel szelessege               */

static float    s_re[NMAX], s_im[NMAX];
static float    s_win[NMAX];
static float    s_db[NMAX], s_pwr[NMAX];
static uint16_t s_bitrev[NMAX];
static float    s_cos[NMAX / 2], s_sin[NMAX / 2];

/* mintagyujto: NMAX minta, blokkonkent toltjuk */
static int16_t  s_buf[NMAX * 2];      /* Q,I parok                        */
static uint16_t s_fill = 0;

static uint16_t s_n     = 1024;       /* aktualis FFT hossz (256/512/1024) */
static uint16_t s_n_tab = 0;          /* amire a tablak keszultek          */
static bool     s_pwr_ok = false;

static bool     s_on    = true;
static uint8_t  s_decim = 3;          /* hany adagonkent RAJZOLUNK sort    */
static uint8_t  s_cnt   = 0;
static uint32_t s_cf_hz = 0, s_sps = 50000;

static float    s_floor_db = -60.0f;
static float    s_lift_db  = -2.0f;
static float    s_range_db = 40.0f;
static float    s_alpha    = 0.40f;
static bool     s_invert   = true;
static uint8_t  s_window   = 1;       /* 0 = Hann, 1 = Nuttall             */
static uint8_t  s_dump     = 0;       /* 1 = a kovetkezo sort kiirjuk       */

/* --- HATTERLEVONAS (a spektrumanalizatorok "normalize" funkcioja) -------
 * 2026-08-19-i meres: ket, masodpercekre levo sor korrelacioja r = 0,444.
 * Vagyis a lathato mintazat kozel fele NEM zaj, hanem ALLANDO: a vevo sajat
 * frekvenciamenete es a fix spurok. Ezt tobb atlagolas SOSEM tunteti el
 * (az alpha=0,12 a veletlen reszt 2,79 -> 1,41 dB-re vitte, a kepen megsem
 * latszott javulas) - csak a hosszu ideju atlag levonasa.
 *
 * ARA: egy allandoan sugarzo vivo is beleolvad a hatterbe, ha a tau-nal
 * tovabb van ott. Ezert allithato, es kikapcsolhato. */
static float    s_base[NMAX];
static bool     s_base_ok  = false;
static uint8_t  s_base_on  = 1;
static float    s_base_tau = 20.0f;   /* masodperc                          */

void iq_fft_enable(bool on)      { s_on = on; }
bool iq_fft_enabled(void)        { return s_on; }
void iq_fft_set_decim(uint8_t n) { s_decim = (n < 1) ? 1 : (n > 64 ? 64 : n); }
void iq_fft_set_tuning(uint32_t center_hz, uint32_t sps)
{
    s_cf_hz = center_hz;
    if (sps) s_sps = sps;
}

/* ===================== tablak ========================================== */
static void tables_init(uint16_t n)
{
    for (int i = 0; i < n; i++) {
        float x = (float)i / (float)(n - 1);
        if (s_window) {
            /* Nuttall - ugyanazok az egyutthatok, mint az SDR++-ban
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
    s_pwr_ok = false;                 /* uj hossz -> uj atlagolas */
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

/* ===================== egy adag feldolgozasa ===========================
 * Az FFT MINDEN teli adagra lefut (az EMA igy valodi atlagot kap), de sort
 * csak minden s_decim.-re tolunk ki - kulonben 49 sor/s-mal szaguldana a
 * vizeses, es 270 sor mindossze 5,5 masodperc tortenetet mutatna. */
static void process(void)
{
    const uint16_t n = s_n;
    if (s_n_tab != n) tables_init(n);

    /* i16le, Q ELOL - a projekt formatuma (geckokapula-kompatibilis) */
    for (int i = 0; i < n; i++) {
        float q  = (float)s_buf[2 * i + 0];
        float ii = (float)s_buf[2 * i + 1];
        s_re[i] = ii * s_win[i];
        s_im[i] = q  * s_win[i];
    }

    fft_run(n);

    /* teljesitmeny + FFT-shift + EMA. Kisebb sullyal is eleg, mert a
     * max-tartas maga is koncentral (1024 bin -> 240 pixel = 4,27). */
    for (int k = 0; k < n; k++) {
        int src = (k + n / 2) % n;
        float m = s_re[src] * s_re[src] + s_im[src] * s_im[src];
        s_pwr[k] = s_pwr_ok ? (s_pwr[k] * (1.0f - s_alpha) + m * s_alpha) : m;
    }
    s_pwr_ok = true;

    /* DC-tuske: a kozepso binek a vevo maradek egyenaramu offszetje. A
     * szelesseg a HOSSZAL aranyos: 256-nal +-2 bin, 1024-nel +-8. */
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

    /* hatterlevonas: a hosszu ideju atlag kivonasa. A sorrata kb. 49/s,
     * tehat egy tau masodperces idoallandohoz a suly 1/(49*tau). */
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

    /* csak minden s_decim. adagbol rajzolunk sort */
    if (++s_cnt < s_decim) return;
    s_cnt = 0;

    static uint8_t bins[NBINS];

    /* MAX-TARTAS TORZITAS-KORREKCIO
     * 1024 bin / 240 pixel = 4,267 -> egyes pixelek 4, masok 5 bin
     * maximumat veszik. Exponencialis eloszlasu teljesitmenynel k bin
     * maximumanak varhato erteke a H_k HARMONIKUS SZAM, tehat a ketto
     * kozott 10*log10(H5/H4) = 0,40 dB szisztematikus kulonbseg van.
     * Ez 15 pixel periodusu, 64 vilagos csikbol allo fodrozodast rajzol a
     * sima zajpadlora (2026-08-19, HA7DCD eszrevetele).
     * A H_k levonasa egzaktul megszunteti: merve 0,404 -> 0,013 dB. */
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

    tft_set_autoscale(false);          /* EGY skalazo legyen: ez itt */
    tft_push_specline(bins, NBINS);
    if (s_cf_hz) tft_set_span(s_cf_hz, s_sps);

    /* Diagnosztika: egy nyers sor kiirasa, hogy a fodrozodas periodusa
     * MERHETO legyen, ne talalgatas. A dB-ertekeket adjuk, 0,1 dB-ben. */
    if (s_dump) {
        s_dump = 0;
        IQF_OUT.printf("FFTDUMP n=%u win=%u padlo=%.1f\n", n, s_window, s_floor_db);
        for (int k = 0; k < n; k++) {
            IQF_OUT.printf("%d%c", (int)(s_db[k] * 10.0f),
                           ((k & 15) == 15) ? '\n' : ' ');
        }
        IQF_OUT.println("FFTDUMP-END");
    }
}

/* ===================== blokk-bemenet =================================== */
void iq_fft_push_block(const uint8_t *payload)
{
    if (!s_on || !payload) return;

    /* Gyujtes: s_n mintat kell osszeszedni, blokkonkent 256-ot. */
    if (s_fill + BLKSAMP <= s_n) {
        memcpy(&s_buf[s_fill * 2], payload, BLKSAMP * 2 * sizeof(int16_t));
        s_fill += BLKSAMP;
    }
    if (s_fill < s_n) return;
    s_fill = 0;

    process();
}

/* ===================== parancsok ======================================= */
static void iqf_print_cfg(void)
{
    IQF_OUT.printf("FFT-CFG: n=%u ablak=%s lift=%.1f range=%.1f alpha=%.2f "
                   "dec=%u inv=%u base=%u tau=%.0fs | padlo=%.1f dB "
                   "felbontas=%.1f Hz/bin sav=%lu Hz fut=%d\n",
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
    if (p[0] == '0') { s_on = false; IQF_OUT.println("FFT: ki"); return true; }
    if (p[0] == '1') { s_on = true;  IQF_OUT.println("FFT: be"); return true; }
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
                    IQF_OUT.println("FFT: n csak 256 / 512 / 1024 lehet");
                }
            }
            else IQF_OUT.printf("FFT: ismeretlen kulcs: %s\n", t);
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

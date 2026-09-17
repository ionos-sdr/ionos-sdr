/* rgb_load.cpp - lasd rgb_load.h */

#include "rgb_load.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "esp_timer.h"

/* EBBEN A PROJEKTBEN a Serial a nyers I/Q folyam - a szoveg a Serial0-ra megy. */
#ifndef RGB_OUT
#define RGB_OUT Serial0
#endif

/* arduino-esp32 3.x: rgbLedWrite es neopixelWrite is letezik */
#ifndef RGB_WRITE
#define RGB_WRITE(pin, r, g, b) rgbLedWrite((pin), (r), (g), (b))
#endif

/* ===================== konfig ========================================== */
typedef struct {
    int      pin;
    uint8_t  fenyero;      /* csucs-fenyero 1..255                        */
    float    lassu_s;      /* korido ures rendszernel                     */
    float    gyors_s;      /* korido 100% terhelesnel                     */
    float    gorbe;        /* 0.1..2.0 kitevo; 0.5 = gyokos               */
    uint8_t  dither;
    uint8_t  gamma_on;
    float    telitettseg;  /* 0..1                                        */
    uint16_t hz;
} rgb_cfg_t;

static rgb_cfg_t s_cfg = { 48, 12, 90.0f, 2.0f, 0.5f, 1, 1, 1.0f, 50 };

/* ===================== terhelesmeres (idle hook) ======================= */
static volatile uint32_t s_idle_cnt[2] = { 0, 0 };
static uint32_t          s_idle_max[2] = { 1, 1 };
static float             s_load[2]     = { 0.0f, 0.0f };
static float             s_override    = -1.0f;
static bool              s_hook_ok[2]  = { false, false };

/* Explicit olvasas-modositas-iras: a volatile ++ C++20-ban elavult
 * (-Wvolatile), mert nem atomi es nem egyertelmu a szemantikaja. Itt
 * amugy sem kell atomicitas: magonkent EGY iro (az adott mag idle taskja)
 * es egy olvaso van, es egy elvesztett szamlalas 500 ms-os ablakban
 * merhetetlen. */
static bool IRAM_ATTR idle_hook_c0(void) { uint32_t v = s_idle_cnt[0]; s_idle_cnt[0] = v + 1; return true; }
static bool IRAM_ATTR idle_hook_c1(void) { uint32_t v = s_idle_cnt[1]; s_idle_cnt[1] = v + 1; return true; }

/* Egy ablak vegen: terheles = 1 - szamlalo/max.  A max onkalibralodik:
 * felfele azonnal koveti, lefele lassan cseng (kulonben egyetlen terhelt
 * ablak orokre lenyomna a referenciat). */
static void load_update(float dt_s)
{
    for (int c = 0; c < 2; c++) {
        if (!s_hook_ok[c]) { s_load[c] = 0.0f; continue; }

        uint32_t n = s_idle_cnt[c];
        s_idle_cnt[c] = 0;

        /* normalizalas 1 masodpercre, hogy az ablakhossz ne szamitson */
        uint32_t per_s = (dt_s > 0.0f) ? (uint32_t)(n / dt_s) : n;

        if (per_s > s_idle_max[c]) s_idle_max[c] = per_s;              /* azonnal fel */
        else s_idle_max[c] = (uint32_t)(s_idle_max[c] * 0.999f + 1);   /* lassan le   */

        float l = 1.0f - (float)per_s / (float)(s_idle_max[c] ? s_idle_max[c] : 1);
        if (l < 0.0f) l = 0.0f;
        if (l > 1.0f) l = 1.0f;

        /* simitas, hogy a LED ne rangatozzon */
        s_load[c] = s_load[c] * 0.7f + l * 0.3f;
    }
}

float rgb_load_get(void)
{
    if (s_override >= 0.0f) return s_override;
    return (s_load[0] > s_load[1]) ? s_load[0] : s_load[1];
}

void rgb_load_get_cores(float *c0, float *c1)
{
    if (c0) *c0 = s_load[0];
    if (c1) *c1 = s_load[1];
}

void rgb_load_override(float v)
{
    if (v < 0.0f) { s_override = -1.0f; return; }
    s_override = (v > 1.0f) ? 1.0f : v;
}

/* ===================== szin ============================================ */
static void hsv_to_rgb(float h, float s, float v, float *r, float *g, float *b)
{
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float rr, gg, bb;
    if      (h <  60.0f) { rr = c; gg = x; bb = 0; }
    else if (h < 120.0f) { rr = x; gg = c; bb = 0; }
    else if (h < 180.0f) { rr = 0; gg = c; bb = x; }
    else if (h < 240.0f) { rr = 0; gg = x; bb = c; }
    else if (h < 300.0f) { rr = x; gg = 0; bb = c; }
    else                 { rr = c; gg = 0; bb = x; }
    *r = (rr + m); *g = (gg + m); *b = (bb + m);
}

/* ===================== task ============================================ */
static TaskHandle_t  s_task = NULL;
static volatile bool s_run  = false;
static volatile int  s_test = 0;      /* >0: szinkor-teszt, ennyi ms van hatra */

static void rgb_task(void *arg)
{
    (void)arg;
    float hue = 0.0f;
    float err_r = 0, err_g = 0, err_b = 0;     /* idobeli dither hibatagjai */
    uint64_t t_prev = esp_timer_get_time();
    uint64_t t_load = t_prev;

    while (s_run) {
        uint32_t period_ms = 1000u / (s_cfg.hz ? s_cfg.hz : 50);
        vTaskDelay(pdMS_TO_TICKS(period_ms ? period_ms : 1));

        uint64_t now = esp_timer_get_time();
        float dt = (float)(now - t_prev) / 1e6f;
        t_prev = now;
        if (dt <= 0.0f || dt > 1.0f) dt = 0.02f;

        /* terhelesmeres fel masodpercenkent */
        if (now - t_load >= 500000ull) {
            load_update((float)(now - t_load) / 1e6f);
            t_load = now;
        }

        float load = rgb_load_get();

        /* --- korido a terhelesbol ---
         * A gorbe (kitevo) azert kell, mert linearisan a 10-20%-os
         * terheles alig latszana; gyokos leképzessel mar erezheto:
         *   20% -> 50 s (gyokos)  vs  72 s (linearis), 90 s-os alapbol. */
        float f = powf(load, s_cfg.gorbe);
        float per_s = s_cfg.lassu_s + (s_cfg.gyors_s - s_cfg.lassu_s) * f;
        if (per_s < 0.05f) per_s = 0.05f;

        if (s_test > 0) { per_s = 3.0f; s_test -= (int)(dt * 1000.0f); }

        hue += 360.0f * dt / per_s;
        if (hue >= 360.0f) hue -= 360.0f;

        /* --- szin ---
         * Eloszor 0..1 tartomanyban szamolunk, gammazunk, es CSAK azutan
         * skalazunk a fenyerore. (Forditva a halvany szinek eltunnenek.) */
        float r, g, b;
        hsv_to_rgb(hue, s_cfg.telitettseg, 1.0f, &r, &g, &b);

        if (s_cfg.gamma_on) {
            r = powf(r, 2.2f);
            g = powf(g, 2.2f);
            b = powf(b, 2.2f);
        }
        r *= (float)s_cfg.fenyero;
        g *= (float)s_cfg.fenyero;
        b *= (float)s_cfg.fenyero;

        uint8_t R, G, B;
        if (s_cfg.dither) {
            /* Idobeli dither: a tortresz atvitelre kerul a kovetkezo keretbe.
             * 12/255 fenyeronel a hue=10 zold csatornaja 0.23 - kerekitve
             * ELVESZNE, es a szinatmenet elso 15 foka vegig piros maradna.
             * 50 Hz-en a szem atlagol, es a 0.23 is latszik. */
            float vr = r + err_r, vg = g + err_g, vb = b + err_b;
            float fr = floorf(vr), fg = floorf(vg), fb = floorf(vb);
            err_r = vr - fr; err_g = vg - fg; err_b = vb - fb;
            R = (uint8_t)(fr < 0 ? 0 : (fr > 255 ? 255 : fr));
            G = (uint8_t)(fg < 0 ? 0 : (fg > 255 ? 255 : fg));
            B = (uint8_t)(fb < 0 ? 0 : (fb > 255 ? 255 : fb));
        } else {
            R = (uint8_t)(r < 0 ? 0 : (r > 255 ? 255 : r + 0.5f));
            G = (uint8_t)(g < 0 ? 0 : (g > 255 ? 255 : g + 0.5f));
            B = (uint8_t)(b < 0 ? 0 : (b > 255 ? 255 : b + 0.5f));
        }

        RGB_WRITE(s_cfg.pin, R, G, B);
    }

    RGB_WRITE(s_cfg.pin, 0, 0, 0);
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ===================== eletciklus ====================================== */
void rgb_load_init(int pin)
{
    if (pin >= 0) s_cfg.pin = pin;
    if (s_run) return;

    if (!s_hook_ok[0])
        s_hook_ok[0] = (esp_register_freertos_idle_hook_for_cpu(idle_hook_c0, 0) == ESP_OK);
    if (!s_hook_ok[1])
        s_hook_ok[1] = (esp_register_freertos_idle_hook_for_cpu(idle_hook_c1, 1) == ESP_OK);

    s_run = true;
    /* 0-as mag, 1-es prioritas: a kijelzo-task melle, az FG23 SPI ala */
    if (xTaskCreatePinnedToCore(rgb_task, "rgb_load", 3072, NULL, 1, &s_task, 0) != pdPASS) {
        s_run = false;
        RGB_OUT.println("RGB: task-inditas sikertelen");
        return;
    }
    RGB_OUT.printf("RGB: terhelesjelzo el a %d labon (idle-hook c0=%d c1=%d)\n",
                   s_cfg.pin, (int)s_hook_ok[0], (int)s_hook_ok[1]);
}

void rgb_load_stop(void) { s_run = false; }
bool rgb_load_running(void) { return s_run; }

/* ===================== parancsok ======================================= */
static void rgb_print_cfg(void)
{
    RGB_OUT.printf("RGB-CFG: pin=%d fenyero=%u lassu=%.1fs gyors=%.1fs gorbe=%.2f "
                   "dither=%u gamma=%u telitettseg=%.2f hz=%u fut=%d | "
                   "terheles c0=%.0f%% c1=%.0f%% (%s)\n",
                   s_cfg.pin, s_cfg.fenyero, s_cfg.lassu_s, s_cfg.gyors_s,
                   s_cfg.gorbe, s_cfg.dither, s_cfg.gamma_on, s_cfg.telitettseg,
                   s_cfg.hz, (int)s_run, s_load[0] * 100.0f, s_load[1] * 100.0f,
                   (s_override >= 0.0f) ? "kulso" : "auto");
}

static void rgb_set_kv(const char *args)
{
    char buf[192];
    strncpy(buf, args, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char *save = NULL;
    for (char *t = strtok_r(buf, " \t,", &save); t; t = strtok_r(NULL, " \t,", &save)) {
        char *eq = strchr(t, '=');
        if (!eq) continue;
        *eq = 0;
        const char *k = t;
        double v = atof(eq + 1);

        if      (!strcmp(k, "pin"))          s_cfg.pin         = (int)v;
        else if (!strcmp(k, "fenyero"))      s_cfg.fenyero     = (uint8_t)(v < 1 ? 1 : (v > 255 ? 255 : v));
        else if (!strcmp(k, "lassu"))        s_cfg.lassu_s     = (float)v;
        else if (!strcmp(k, "gyors"))        s_cfg.gyors_s     = (float)v;
        else if (!strcmp(k, "gorbe"))        s_cfg.gorbe       = (float)(v / 100.0);
        else if (!strcmp(k, "dither"))       s_cfg.dither      = v ? 1 : 0;
        else if (!strcmp(k, "gamma"))        s_cfg.gamma_on    = v ? 1 : 0;
        else if (!strcmp(k, "telitettseg"))  s_cfg.telitettseg = (float)(v / 100.0);
        else if (!strcmp(k, "hz"))           s_cfg.hz          = (uint16_t)v;
        else RGB_OUT.printf("RGB: ismeretlen kulcs: %s\n", k);
    }
    if (s_cfg.gorbe < 0.1f) s_cfg.gorbe = 0.1f;
    rgb_print_cfg();
}

bool rgb_load_cmd(const char *line)
{
    if (!line || line[0] != 'L') return false;
    const char *p = line + 1;
    while (*p == ' ') p++;

    if (*p == 0) { rgb_print_cfg(); return true; }
    if (!strncmp(p, "set",  3)) { rgb_set_kv(p + 3); return true; }
    if (!strncmp(p, "test", 4)) { s_test = 3000; RGB_OUT.println("RGB: szinkor-teszt 3 mp"); return true; }
    if (p[0] == '0') { rgb_load_stop(); RGB_OUT.println("RGB: ki"); return true; }
    if (p[0] == '1') { rgb_load_init(-1); return true; }

    RGB_OUT.println("RGB: L | L0 | L1 | Ltest | Lset k=v");
    return true;
}

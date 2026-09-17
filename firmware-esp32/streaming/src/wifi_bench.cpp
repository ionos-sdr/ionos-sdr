/* wifi_bench.cpp - lasd wifi_bench.h
 *
 * Tervezesi elvek (a projekt korabbi tanulsagaibol):
 *  - SOHA nem blokkolo iras: kozvetlen send(fd,...,MSG_DONTWAIT), a
 *    NetworkClient::write() 1 masodperces select-varakozasa megkerulve.
 *  - A bench sajat taskban fut, hogy a merese ne a loop() utemezesetol
 *    fuggjon; a magja es prioritasa futasidoben allithato, mert pont az az
 *    egyik merendo valtozo, hogy hova erdemes tenni.
 *  - Minden keret onhordozo fejlecet visz (magic + seq + step + felkinalt
 *    rata), igy a Python-nyelo FUGGETLENUL is tud lepcsonkent osszesiteni.
 *    Ez a masodik meropont, ami az USB-hid nyugdijazasakor elveszett.
 */

#include "wifi_bench.h"

#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/* A kimeneti csatorna. EBBEN A PROJEKTBEN a Serial a NYERS I/Q FOLYAM
 * (nativ USB CDC), a szoveges diagnosztika a Serial0 (CP2102) - ezert
 * alapertelmezetten oda irunk. Mas projektben -D WB_OUT=Serial. */
#ifndef WB_OUT
#define WB_OUT Serial0
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

#include <lwip/sockets.h>
#include <fcntl.h>
#include <sys/time.h>

/* ===================== forditasi ideju alapertekek ===================== */
#ifndef WB_DEF_PORT
#define WB_DEF_PORT     7777
#endif
#ifndef WB_DEF_BLK
#define WB_DEF_BLK      1040    /* ugyanaz, mint az IQ-blokk: osszemerheto */
#endif
#ifndef WB_DEF_CHUNK
#define WB_DEF_CHUNK    2880    /* 2 x 1440 MSS - merve: ez jo, a HT40 nem */
#endif
#ifndef WB_DEF_CORE
#define WB_DEF_CORE     0
#endif
#ifndef WB_DEF_PRIO
#define WB_DEF_PRIO     5
#endif
#ifndef WB_DEF_WARM_S
#define WB_DEF_WARM_S   2
#endif
#ifndef WB_DEF_WIN_MS
#define WB_DEF_WIN_MS   1000
#endif
#ifndef WB_DEF_BURST
#define WB_DEF_BURST    4
#endif
#ifndef WB_STACK
#define WB_STACK        4096
#endif
#define WB_BLK_MAX      8192
#define WB_BUF_MAX      32768   /* blk * agg felso korlat */
#define WB_HDR          24
#define WB_MAGIC        "WBN1"

/* ===================== keretfejlec (a nyelo ugyanezt olvassa) ========== */
typedef struct __attribute__((packed)) {
    char     magic[4];    /* "WBN1"                                       */
    uint32_t seq;         /* keret sorszam, futasonkent 0-tol             */
    uint32_t len;         /* teljes keret bajtban (fejlec + payload)      */
    uint32_t t_us;        /* esp_timer_get_time() also 32 bit             */
    uint16_t step;        /* lepcso index (0-tol)                         */
    uint16_t flags;       /* bit0 = bemelegites (dobd el a statisztikabol)*/
    uint32_t rate_kBps;   /* felkinalt rata; 0 = fekezes nelkul           */
} wb_hdr_t;
static_assert(sizeof(wb_hdr_t) == WB_HDR, "wb_hdr_t merete nem 24");

/* ===================== konfig ========================================== */
typedef struct {
    uint16_t port;
    uint16_t blk;
    uint16_t chunk;
    uint8_t  core;
    uint8_t  prio;
    uint8_t  nodelay;
    uint8_t  quiet;
    uint16_t warm_s;
    uint16_t win_ms;
    uint16_t burst;
    uint16_t agg;      /* hany keret menjen egy loketben */
    int32_t  sndbuf;
} wb_cfg_t;

static wb_cfg_t s_cfg = {
    WB_DEF_PORT, WB_DEF_BLK, WB_DEF_CHUNK, WB_DEF_CORE, WB_DEF_PRIO,
    1, 0, WB_DEF_WARM_S, WB_DEF_WIN_MS, WB_DEF_BURST, 1, 0
};

/* ===================== menetrend ======================================= */
typedef struct {
    uint32_t start_kBps;  /* 0 = max mod                                  */
    uint32_t stop_kBps;
    uint32_t step_kBps;   /* 0 = egyetlen lepcso                          */
    uint32_t dwell_s;     /* 0 = vegtelen                                 */
} wb_plan_t;

static wb_plan_t   s_plan;
static wb_ext_fn   s_ext_fn  = NULL;
static TaskHandle_t s_task   = NULL;
static volatile bool s_run   = false;
static volatile bool s_abort = false;
static uint8_t    *s_buf     = NULL;
static uint32_t    s_buf_sz  = 0;

/* ===================== segedek ========================================= */
static void wb_fill_pattern(uint8_t *p, int n)
{
    /* determinisztikus, offszet-fuggo minta - a nyelo memcmp-pel ellenorzi */
    for (int i = 0; i < n; i++) p[i] = (uint8_t)(i * 31u + 7u);
}

static uint32_t wb_free_heap(void)
{
    return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
}

static void wb_ext_read(wb_ext_t *o)
{
    memset(o, 0, sizeof(*o));
    if (s_ext_fn) s_ext_fn(o);
}

static int wb_setup_client(int fd)
{
    int one = 1;
    if (s_cfg.nodelay)
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    if (s_cfg.sndbuf > 0) {
        int v = s_cfg.sndbuf;
        if (setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &v, sizeof(v)) != 0)
            WB_OUT.printf("BENCH: SO_SNDBUF=%d NEM allithato (LWIP_SO_SNDBUF?)\n", v);
    }
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    return 0;
}

/* varakozas irhatosagra, max to_ms; visszater: 1 irhato, 0 timeout, -1 hiba */
static int wb_wait_writable(int fd, int to_ms)
{
    fd_set w;
    FD_ZERO(&w);
    FD_SET(fd, &w);
    struct timeval tv;
    tv.tv_sec  = to_ms / 1000;
    tv.tv_usec = (to_ms % 1000) * 1000;
    int r = select(fd + 1, NULL, &w, NULL, &tv);
    if (r < 0) return -1;
    return r > 0 ? 1 : 0;
}

/* ===================== a task ========================================== */
typedef struct {
    uint64_t bytes;
    uint32_t frames;
    uint32_t eagain;
    uint32_t tmax_us;      /* leghosszabb egyetlen send() hivas          */
    uint32_t heap_min;
    uint64_t t0_us;
} wb_acc_t;

static void wb_acc_reset(wb_acc_t *a)
{
    memset(a, 0, sizeof(*a));
    a->heap_min = 0xFFFFFFFFu;
    a->t0_us    = esp_timer_get_time();
}

static void wb_print_window(int step, uint32_t offered_kBps, const wb_acc_t *a)
{
    uint64_t dt = esp_timer_get_time() - a->t0_us;
    if (dt == 0) dt = 1;
    double kBps = (double)a->bytes * 1000.0 / (double)dt;   /* B/us -> kB/s */
    wb_ext_t e; wb_ext_read(&e);
    WB_OUT.printf(
        "BENCH: lep=%d fel=%lu ach=%.1f kB/s sps16=%.0f sps8=%.0f "
        "ker=%lu eagain=%lu tmax=%luus ring=%lu dtmax=%luus heap=%luk\n",
        step, (unsigned long)offered_kBps, kBps,
        kBps * 1000.0 / 4.0, kBps * 1000.0 / 2.0,
        (unsigned long)a->frames, (unsigned long)a->eagain,
        (unsigned long)a->tmax_us, (unsigned long)e.spi_ring_max,
        (unsigned long)e.loop_dt_max_us, (unsigned long)(a->heap_min / 1024));
}

static void wb_print_step(int step, uint32_t offered_kBps,
                          const wb_acc_t *a, const wb_ext_t *d)
{
    uint64_t dt = esp_timer_get_time() - a->t0_us;
    if (dt == 0) dt = 1;
    double kBps = (double)a->bytes * 1000.0 / (double)dt;
    WB_OUT.printf(
        "BENCH-STEP: lep=%d fel=%lu ach=%.1f sps16=%.0f sps8=%.0f ido=%.1fs "
        "ker=%lu eagain=%lu tmax=%luus lost=%lu dup=%lu ovf=%lu blk=%lu "
        "spydrop=%lu ring=%lu dtmax=%luus heapmin=%luk\n",
        step, (unsigned long)offered_kBps, kBps,
        kBps * 1000.0 / 4.0, kBps * 1000.0 / 2.0, (double)dt / 1e6,
        (unsigned long)a->frames, (unsigned long)a->eagain,
        (unsigned long)a->tmax_us,
        (unsigned long)d->spi_lost, (unsigned long)d->spi_dup,
        (unsigned long)d->spi_ovf,  (unsigned long)d->spi_blocks,
        (unsigned long)d->spy_drop, (unsigned long)d->spi_ring_max,
        (unsigned long)d->loop_dt_max_us,
        (unsigned long)(a->heap_min / 1024));
}

static void wb_task(void *arg)
{
    (void)arg;

    int lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (lsock < 0) {
        WB_OUT.println("BENCH: socket() hiba");
        s_run = false; s_task = NULL; vTaskDelete(NULL); return;
    }
    int one = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in sa;
    memset(&sa, 0, sizeof(sa));
    sa.sin_family      = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port        = htons(s_cfg.port);
    if (bind(lsock, (struct sockaddr *)&sa, sizeof(sa)) != 0 ||
        listen(lsock, 1) != 0) {
        WB_OUT.printf("BENCH: bind/listen hiba a %u porton\n", s_cfg.port);
        close(lsock);
        s_run = false; s_task = NULL; vTaskDelete(NULL); return;
    }
    { int fl = fcntl(lsock, F_GETFL, 0); fcntl(lsock, F_SETFL, fl | O_NONBLOCK); }

    WB_OUT.printf("BENCH: varom a nyelot a %u porton "
                  "(blk=%u chunk=%u core=%u prio=%u nodelay=%u)\n",
                  s_cfg.port, s_cfg.blk, s_cfg.chunk,
                  s_cfg.core, s_cfg.prio, s_cfg.nodelay);

    int cfd = -1;
    while (s_run && !s_abort && cfd < 0) {
        cfd = accept(lsock, NULL, NULL);
        if (cfd < 0) vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (cfd < 0) { close(lsock); s_run = false; s_task = NULL; vTaskDelete(NULL); return; }

    wb_setup_client(cfd);
    WB_OUT.println("BENCH: nyelo csatlakozott, mehet");

    /* ---- lepcsok vegigjatszasa ---- */
    const uint32_t blk   = s_cfg.blk;
    const uint32_t agg   = s_cfg.agg ? s_cfg.agg : 1;   /* keret / loket    */
    const uint32_t bufsz = blk * agg;                   /* egy loket bajtja */
    const uint32_t chunk = s_cfg.chunk ? s_cfg.chunk : bufsz;
    for (uint32_t f = 0; f < agg; f++)
        wb_fill_pattern(s_buf + f * blk + WB_HDR, blk - WB_HDR);

    uint32_t seq = 0;
    int      step = 0;
    uint32_t rate = s_plan.start_kBps;

    while (s_run && !s_abort) {
        const uint64_t rate_Bps  = (uint64_t)rate * 1000ull;
        const uint64_t token_cap = (uint64_t)bufsz * (s_cfg.burst ? s_cfg.burst : 1);
        const uint64_t warm_us   = (uint64_t)s_cfg.warm_s * 1000000ull;
        const uint64_t dwell_us  = (uint64_t)s_plan.dwell_s * 1000000ull;

        uint64_t step_t0 = esp_timer_get_time();
        int64_t  tokens  = 0;
        uint64_t last    = step_t0;
        bool     warm    = (s_cfg.warm_s > 0);

        wb_acc_t win, acc;
        wb_acc_reset(&win);
        wb_acc_reset(&acc);
        wb_ext_t e0, e1;
        wb_ext_read(&e0);

        uint64_t win_t0   = step_t0;
        uint32_t frame_off = 0;

        WB_OUT.printf("BENCH: --- lepcso %d: %lu kB/s (%.0f ksps int16) ---\n",
                      step, (unsigned long)rate, (double)rate / 4.0);

        while (s_run && !s_abort) {
            uint64_t now = esp_timer_get_time();

            /* bemelegites lejart? innentol szamol az akkumulator */
            if (warm && (now - step_t0) >= warm_us) {
                warm = false;
                wb_acc_reset(&acc);
                wb_ext_read(&e0);
            }
            /* lepcso vege? */
            if (dwell_us && (now - step_t0) >= dwell_us + warm_us) break;

            /* token-vodor */
            if (rate_Bps) {
                tokens += (int64_t)((now - last) * rate_Bps / 1000000ull);
                if (tokens > (int64_t)token_cap) tokens = (int64_t)token_cap;
            }
            last = now;

            /* uj loket indul? */
            if (frame_off == 0) {
                if (rate_Bps && tokens < (int64_t)bufsz) {
                    uint64_t need_us = ((uint64_t)bufsz - (uint64_t)tokens)
                                       * 1000000ull / rate_Bps;
                    vTaskDelay(pdMS_TO_TICKS(need_us / 1000 + 1));
                    continue;
                }
                for (uint32_t f = 0; f < agg; f++) {
                    wb_hdr_t *h = (wb_hdr_t *)(s_buf + f * blk);
                    memcpy(h->magic, WB_MAGIC, 4);
                    h->seq       = seq + f;
                    h->len       = blk;
                    h->t_us      = (uint32_t)now;
                    h->step      = (uint16_t)step;
                    h->flags     = warm ? 1 : 0;
                    h->rate_kBps = rate;
                }
                if (rate_Bps) tokens -= (int64_t)bufsz;
            }

            int n = (int)(bufsz - frame_off);
            if (n > (int)chunk) n = (int)chunk;

            uint64_t ta = esp_timer_get_time();
            int r = send(cfd, s_buf + frame_off, n, MSG_DONTWAIT);
            uint32_t dt = (uint32_t)(esp_timer_get_time() - ta);

            if (r > 0) {
                frame_off += (uint32_t)r;
                win.bytes += (uint32_t)r;
                if (!warm) acc.bytes += (uint32_t)r;
                if (dt > win.tmax_us) win.tmax_us = dt;
                if (!warm && dt > acc.tmax_us) acc.tmax_us = dt;
                if (frame_off >= bufsz) {
                    frame_off = 0;
                    seq       += agg;
                    win.frames += agg;
                    if (!warm) acc.frames += agg;
                }
            } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
                win.eagain++;
                if (!warm) acc.eagain++;
                if (wb_wait_writable(cfd, 20) < 0) { s_abort = true; break; }
            } else {
                WB_OUT.printf("BENCH: kliens elszallt (errno=%d)\n", errno);
                s_abort = true;
                break;
            }

            uint32_t fh = wb_free_heap();
            if (fh < win.heap_min) win.heap_min = fh;
            if (!warm && fh < acc.heap_min) acc.heap_min = fh;

            /* riportablak */
            if (s_cfg.win_ms &&
                (esp_timer_get_time() - win_t0) >= (uint64_t)s_cfg.win_ms * 1000ull) {
                if (!s_cfg.quiet) wb_print_window(step, rate, &win);
                wb_acc_reset(&win);
                win_t0 = esp_timer_get_time();
            }
        }

        if (s_abort) break;

        wb_ext_read(&e1);
        wb_ext_t d;
        d.spi_lost   = e1.spi_lost   - e0.spi_lost;
        d.spi_dup    = e1.spi_dup    - e0.spi_dup;
        d.spi_ovf    = e1.spi_ovf    - e0.spi_ovf;
        d.spi_blocks = e1.spi_blocks - e0.spi_blocks;
        d.spy_drop   = e1.spy_drop   - e0.spy_drop;
        d.spi_ring_max   = e1.spi_ring_max;      /* vizjel: abszolut */
        d.loop_dt_max_us = e1.loop_dt_max_us;
        wb_print_step(step, rate, &acc, &d);

        /* kovetkezo lepcso */
        if (!s_plan.step_kBps || !s_plan.dwell_s) break;   /* egyetlen lepcso */
        if (rate >= s_plan.stop_kBps) break;
        rate += s_plan.step_kBps;
        if (rate > s_plan.stop_kBps) rate = s_plan.stop_kBps;
        step++;
    }

    WB_OUT.println("BENCH-END: kesz");
    if (cfd >= 0) close(cfd);
    close(lsock);
    s_run  = false;
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ===================== inditas / leallitas ============================= */
static bool wb_start(uint32_t start_kBps, uint32_t stop_kBps,
                     uint32_t step_kBps, uint32_t dwell_s)
{
    if (s_run) { WB_OUT.println("BENCH: mar fut (B0 = stop)"); return false; }

    if (s_cfg.blk < 64 || s_cfg.blk > WB_BLK_MAX) {
        WB_OUT.println("BENCH: ervenytelen blk"); return false;
    }
    if (s_cfg.agg < 1) s_cfg.agg = 1;
    uint32_t need = (uint32_t)s_cfg.blk * s_cfg.agg;
    if (need > WB_BUF_MAX) {
        WB_OUT.printf("BENCH: blk*agg = %lu > %d - csokkentsd\n",
                      (unsigned long)need, WB_BUF_MAX);
        return false;
    }
    if (s_buf_sz < need) {
        free(s_buf);
        s_buf = (uint8_t *)malloc(need);
        s_buf_sz = s_buf ? need : 0;
    }
    if (!s_buf) { WB_OUT.println("BENCH: nincs RAM a pufferre"); return false; }

    s_plan.start_kBps = start_kBps;
    s_plan.stop_kBps  = stop_kBps;
    s_plan.step_kBps  = step_kBps;
    s_plan.dwell_s    = dwell_s;

    s_run   = true;
    s_abort = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        wb_task, "wifi_bench", WB_STACK, NULL,
        s_cfg.prio, &s_task, s_cfg.core ? 1 : 0);

    if (ok != pdPASS) {
        WB_OUT.println("BENCH: task-inditas sikertelen");
        s_run = false;
        return false;
    }
    return true;
}

void wifi_bench_stop(void)
{
    if (!s_run) return;
    s_abort = true;
    s_run   = false;
    WB_OUT.println("BENCH: leallitas keresve");
}

bool wifi_bench_running(void) { return s_run; }
void wifi_bench_set_ext(wb_ext_fn fn) { s_ext_fn = fn; }
void wifi_bench_init(void) { /* lusta inicializalas; a task inditaskor jon */ }

/* ===================== parancsertelmezo ================================ */
static void wb_print_cfg(void)
{
    WB_OUT.printf("BENCH-CFG: port=%u blk=%u chunk=%u core=%u prio=%u "
                  "nodelay=%u sndbuf=%ld warm=%u win=%u burst=%u agg=%u quiet=%u "
                  "fut=%d\n",
                  s_cfg.port, s_cfg.blk, s_cfg.chunk, s_cfg.core, s_cfg.prio,
                  s_cfg.nodelay, (long)s_cfg.sndbuf, s_cfg.warm_s,
                  s_cfg.win_ms, s_cfg.burst, s_cfg.agg, s_cfg.quiet, (int)s_run);
}

static void wb_set_kv(const char *args)
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
        long v = strtol(eq + 1, NULL, 0);

        if      (!strcmp(k, "port"))    s_cfg.port    = (uint16_t)v;
        else if (!strcmp(k, "blk"))     s_cfg.blk     = (uint16_t)v;
        else if (!strcmp(k, "chunk"))   s_cfg.chunk   = (uint16_t)v;
        else if (!strcmp(k, "core"))    s_cfg.core    = (uint8_t)(v ? 1 : 0);
        else if (!strcmp(k, "prio"))    s_cfg.prio    = (uint8_t)v;
        else if (!strcmp(k, "nodelay")) s_cfg.nodelay = (uint8_t)(v ? 1 : 0);
        else if (!strcmp(k, "sndbuf"))  s_cfg.sndbuf  = (int32_t)v;
        else if (!strcmp(k, "warm"))    s_cfg.warm_s  = (uint16_t)v;
        else if (!strcmp(k, "win"))     s_cfg.win_ms  = (uint16_t)v;
        else if (!strcmp(k, "burst"))   s_cfg.burst   = (uint16_t)v;
        else if (!strcmp(k, "agg"))     s_cfg.agg     = (uint16_t)(v < 1 ? 1 : v);
        else if (!strcmp(k, "quiet"))   s_cfg.quiet   = (uint8_t)(v ? 1 : 0);
        else WB_OUT.printf("BENCH: ismeretlen kulcs: %s\n", k);
    }
    wb_print_cfg();
}

bool wifi_bench_cmd(const char *line)
{
    if (!line || line[0] != 'B') return false;
    const char *p = line + 1;
    while (*p == ' ') p++;

    if (*p == 0) { wb_print_cfg(); return true; }

    if (!strncmp(p, "set", 3)) { wb_set_kv(p + 3); return true; }

    if (p[0] == '0' && (p[1] == 0 || p[1] == ' ')) { wifi_bench_stop(); return true; }

    if (p[0] == 'r') {                     /* Br<kBps>[,<sec>] */
        char *e = NULL;
        long kb = strtol(p + 1, &e, 10);
        long sec = 0;
        if (e && *e == ',') sec = strtol(e + 1, NULL, 10);
        if (kb <= 0) { WB_OUT.println("BENCH: Br<kBps>[,<sec>]"); return true; }
        wb_start((uint32_t)kb, (uint32_t)kb, 0, (uint32_t)sec);
        return true;
    }

    if (p[0] == 'm') {                     /* Bm[<sec>] */
        long sec = strtol(p + 1, NULL, 10);
        wb_start(0, 0, 0, (uint32_t)(sec > 0 ? sec : 0));
        return true;
    }

    if (p[0] == 's') {                     /* Bs<start>,<stop>,<step>,<dwell> */
        long a = 0, b = 0, c = 0, d = 0;
        if (sscanf(p + 1, "%ld,%ld,%ld,%ld", &a, &b, &c, &d) != 4 ||
            a <= 0 || b < a || c <= 0 || d <= 0) {
            WB_OUT.println("BENCH: Bs<start>,<stop>,<step>,<dwell_s>");
            return true;
        }
        wb_start((uint32_t)a, (uint32_t)b, (uint32_t)c, (uint32_t)d);
        return true;
    }

    WB_OUT.println("BENCH: B | B0 | Br<kBps>[,<s>] | Bs<a>,<b>,<lep>,<s> | "
                   "Bm[<s>] | Bset k=v");
    return true;
}

/* adsb_esp.cpp - ADS-B aircraft table + readsb/tar1090-compatible JSON server.
 * MIT, HA7DCD. See adsb_esp.h.
 *
 * Decoding here is deliberately small: DF11/DF17 ICAO, DF17 TC 1-4 callsign,
 * TC 9-18 barometric altitude + airborne CPR position (global even/odd pair
 * within 10 s, then local against the aircraft's own last position), TC 19
 * subtype 1/2 ground speed, track and vertical rate. No receiver position is
 * needed or stored. Levels: the FG23 soft level of the first preamble pulse;
 * "rssi" = level - 30 (a rough dBFS-like scale, not calibrated). */
#include "adsb_esp.h"
#include "adsb_map_html.h"
#include "adsb_graphs_html.h"
#include "station_config.h"
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#define AC_MAX         256
#define AC_SHOW_MS     300000u      /* aircraft.json lists those seen in the last 5 min */
#define CPR_PAIR_MS    10000u
#define LOCAL_MAX_MS   600000u
#define LVL_TO_DBFS(l) ((float)(l) - 30.0f)
/* range reference: Budapest centre unless the (not public) station config sets the receiver */
#ifndef ADSB_REF_LAT
#define ADSB_REF_LAT   47.4979
#define ADSB_REF_LON   19.0402
#endif
#define RANGE_MAX_M    600000.0     /* positions further than this are treated as CPR errors */
#define RNG_N          72           /* 5 degree sectors for the range outline */
#define HIST_N         1440         /* one record per minute, 24 h */
#define FIX_KNOWN_MS   60000u       /* a corrected frame is accepted only for an ICAO heard clean this recently */
#define BEAST_PORT     30005
#define RTL_IN_PORT    30004        /* Beast input from a PC decoder (RTL-SDR on the same antenna), readsb beast_in style */
#define BEAST_MAX_CLI  3

typedef struct {
  uint32_t icao, first_ms, last_ms, pos_ms, msgs;
  uint32_t seen_ms[2], posmsg_ms[2];   /* per receiver: 0 = FG23, 1 = RTL-SDR (splitter: both at once) */
  char     flight[9];
  char     cat[3];                    /* emitter category, e.g. "A3" (readsb "category") */
  int32_t  geo_diff; bool geo_ok;     /* TC19: GNSS - baro, ft */
  int8_t   nic, nacp, sil, ver;       /* -1 = unknown */
  double   pos_ts;                    /* epoch s of the last position message (ms resolution) */
  double   last_ts;                   /* epoch s of the last message */
  uint32_t clean_ms;                  /* last CRC-clean (uncorrected) frame */
  int32_t  alt; bool alt_ok;
  float    gs, track; int32_t vrate; bool vel_ok;
  double   lat, lon; bool pos_ok;
  bool     cpr_fix[2];                 /* that CPR half came from an error-corrected frame */
  float    cand_lat, cand_lon; uint32_t cand_ms;   /* first global fix waits for a second one that agrees */
  uint32_t cpr_lat[2], cpr_lon[2], cpr_ms[2]; bool cpr_ok[2];
  int8_t   lvl_last; float lvl_sum; uint32_t lvl_n;
} ac_t;

/* per-aircraft track for the map (tar1090-style trail of the selected aircraft), PSRAM */
#define TR_N        256               /* points per aircraft */
#define TR_STEP_S   4                 /* at most one point per 4 s */
typedef struct { float lat, lon; int32_t alt; uint32_t t; } trp_t;
static trp_t   *s_tr = NULL;          /* AC_MAX * TR_N */
static uint16_t s_trn[AC_MAX], s_trw[AC_MAX];

typedef struct {
  uint32_t min;                       /* millis()/60000 of this bucket, UINT32_MAX = empty */
  uint32_t msgs, acc[3], df11, df17, pos, gok, lok, newac;
  float    sig_sum; uint32_t sig_n; int32_t sig_peak; uint32_t strong;
  uint64_t samples; uint32_t pre, trig, ovf;
  float    noise_sum; uint32_t noise_n;
  float    maxd;                      /* max distance from the reference, m */
  uint32_t fix_rej;                   /* corrected frames dropped (unknown ICAO) */
  uint32_t msgs_rtl;                  /* of msgs: from the Beast input (RTL-SDR), the rest from the FG23 */
  uint32_t cpr_rej;                   /* global CPR fixes rejected as implausible (pos_sane) */
} st_t;

#define ST_N 17
static ac_t             *s_ac = NULL;
static uint32_t          s_nac = 0;
static SemaphoreHandle_t s_mx = NULL;
static st_t              s_st[ST_N];
static st_t              s_tot;
static uint32_t          s_msgs_total = 0;
static bool              s_dbg_have = false;
static uint32_t          p_bytes = 0, p_trig = 0;
static uint16_t          p_pre = 0, p_ovf = 0;
static WebServer        *s_srv = NULL;
/* FG23 RAIL clock (us, 32 bit) -> ESP time: telemetry blocks carry the FG23 time at sending
 * (D->loops); off = esp_us - fg_us with the smallest transfer latency over a 60 s window */
static uint64_t          s_fg_hi = 0;
static uint32_t          s_fg_last = 0;
static int64_t           s_off = 0, s_off_win = INT64_MAX;
static uint32_t          s_off_t0 = 0;
static bool              s_off_ok = false;
static WiFiServer       *s_beast = NULL;
static WiFiClient        s_bcli[BEAST_MAX_CLI];
static uint32_t          s_beast_sent = 0;
static char             *s_buf = NULL;
static size_t            s_cap = 0, s_len = 0;

typedef struct { float d, lat, lon; int32_t alt; } rng_t;
static rng_t             s_rng[RNG_N];
typedef struct {
  uint32_t t; uint16_t msgs, pos, ac, acpos, tracks, df17, rtl, ac_rx[2], acpos_rx[2];
  float sig, peak, noise, maxd, msps; uint32_t trig, pre, ovf;
} hist_t;
static hist_t           *s_hist = NULL;
/* reference comparison (adsb.fi), one record per minute POSTed from the LAN by adsbfi_bench.py */
typedef struct { uint32_t t; uint16_t ref, ours, matched, only; uint8_t cov[5]; uint16_t maxkm; } bench_t;
#define BENCH_N 1440
static bench_t          *s_bench = NULL;
static uint32_t          s_bw = 0;
static uint32_t          s_hw = 0, s_hist_min = UINT32_MAX;

/* ---------------------------------------------------------------- time */
static double now_epoch(void)
{
  struct timeval tv; gettimeofday(&tv, NULL);
  if (tv.tv_sec > 1700000000) return tv.tv_sec + tv.tv_usec / 1e6;
  return millis() / 1000.0;                       /* no NTP yet: uptime */
}

/* ---------------------------------------------------------------- stats */
static st_t *st_cur(void)
{
  uint32_t m = millis() / 60000u;
  st_t *b = &s_st[m % ST_N];
  if (b->min != m) { memset(b, 0, sizeof *b); b->min = m; }
  return b;
}
static void st_add(st_t *a, const st_t *b)
{
  a->msgs += b->msgs; for (int i = 0; i < 3; i++) a->acc[i] += b->acc[i];
  a->df11 += b->df11; a->df17 += b->df17; a->pos += b->pos; a->gok += b->gok; a->lok += b->lok;
  a->newac += b->newac; a->sig_sum += b->sig_sum; a->sig_n += b->sig_n;
  if (b->sig_peak > a->sig_peak) a->sig_peak = b->sig_peak;
  a->strong += b->strong; a->samples += b->samples; a->pre += b->pre; a->trig += b->trig; a->ovf += b->ovf;
  a->noise_sum += b->noise_sum; a->noise_n += b->noise_n;
  if (b->maxd > a->maxd) a->maxd = b->maxd;
  a->fix_rej += b->fix_rej;
  a->cpr_rej += b->cpr_rej;
}

/* ---------------------------------------------------------------- Mode S helpers */
static uint32_t bits(const uint8_t *b, int first, int n)     /* 0-based message bit index */
{
  uint32_t v = 0;
  for (int i = 0; i < n; i++) { int k = first + i; v = (v << 1) | ((b[k >> 3] >> (7 - (k & 7))) & 1u); }
  return v;
}
static int cpr_nl(double lat)
{
  if (fabs(lat) >= 87.0) return 1;
  double a = 1.0 - (1.0 - cos(M_PI / 30.0)) / pow(cos(M_PI / 180.0 * fabs(lat)), 2);
  return (int)floor(2.0 * M_PI / acos(a));
}
static int pmod(int a, int n) { int r = a % n; return r < 0 ? r + n : r; }
static double pmodd(double a, double n) { double r = fmod(a, n); return r < 0 ? r + n : r; }

static bool cpr_global(ac_t *a, bool odd_last)
{
  double le = a->cpr_lat[0] / 131072.0, lo = a->cpr_lat[1] / 131072.0;
  double ne = a->cpr_lon[0] / 131072.0, no = a->cpr_lon[1] / 131072.0;
  int j = (int)floor(59.0 * le - 60.0 * lo + 0.5);
  double r0 = 360.0 / 60.0 * (pmod(j, 60) + le), r1 = 360.0 / 59.0 * (pmod(j, 59) + lo);
  if (r0 >= 270.0) r0 -= 360.0;
  if (r1 >= 270.0) r1 -= 360.0;
  if (cpr_nl(r0) != cpr_nl(r1)) return false;
  double lat = odd_last ? r1 : r0;
  int nl = cpr_nl(lat), ni = nl - (odd_last ? 1 : 0); if (ni < 1) ni = 1;
  int m = (int)floor(ne * (nl - 1) - no * nl + 0.5);
  double lon = (360.0 / ni) * (pmod(m, ni) + (odd_last ? no : ne));
  if (lon >= 180.0) lon -= 360.0;
  a->lat = lat; a->lon = lon; return true;
}
static void cpr_local(ac_t *a, int f)
{
  double yz = a->cpr_lat[f] / 131072.0, xz = a->cpr_lon[f] / 131072.0;
  double dl = 360.0 / (60 - f);
  double j = floor(a->lat / dl) + floor(0.5 + pmodd(a->lat, dl) / dl - yz);
  double lat = dl * (j + yz);
  int nl = cpr_nl(lat) - f; if (nl < 1) nl = 1;
  double dlo = 360.0 / nl;
  double m = floor(a->lon / dlo) + floor(0.5 + pmodd(a->lon, dlo) / dlo - xz);
  double lon = dlo * (m + xz);
  if (lon >= 180.0) lon -= 360.0;
  if (fabs(lat - a->lat) < 1.0 && fabs(lon - a->lon) < 1.5) { a->lat = lat; a->lon = lon; }
}

static void trail_push(const ac_t *a)
{
  if (!s_tr) return;
  uint32_t i = (uint32_t)(a - s_ac), t = (uint32_t)now_epoch();
  trp_t *tr = s_tr + i * TR_N;
  if (s_trn[i]) {
    const trp_t *l = &tr[(s_trw[i] + TR_N - 1) % TR_N];
    if (t - l->t < TR_STEP_S) return;
  }
  trp_t *p = &tr[s_trw[i]];
  p->lat = (float)a->lat; p->lon = (float)a->lon; p->alt = a->alt_ok ? a->alt : INT32_MIN; p->t = t;
  s_trw[i] = (uint16_t)((s_trw[i] + 1) % TR_N); if (s_trn[i] < TR_N) s_trn[i]++;
}

static void ac_range(const ac_t *a, st_t *st)
{
  trail_push(a);
  const double k = M_PI / 180.0;
  double p1 = ADSB_REF_LAT * k, p2 = a->lat * k, dl = (a->lon - ADSB_REF_LON) * k;
  double x = sin((p2 - p1) / 2), y = sin(dl / 2);
  double d = 2.0 * 6371000.0 * asin(sqrt(x * x + cos(p1) * cos(p2) * y * y));
  if (d > RANGE_MAX_M) return;
  double b = atan2(sin(dl) * cos(p2), cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl)) / k;
  if (b < 0) b += 360.0;
  int sct = ((int)(b / (360.0 / RNG_N))) % RNG_N;
  if (d > s_rng[sct].d) { s_rng[sct].d = (float)d; s_rng[sct].lat = (float)a->lat; s_rng[sct].lon = (float)a->lon; s_rng[sct].alt = a->alt; }
  if (d > st->maxd) st->maxd = (float)d;
  if (d > s_tot.maxd) s_tot.maxd = (float)d;
}

static uint64_t fg_unwrap(uint32_t t)            /* FG23 32-bit us near the last telemetry -> 64 bit */
{
  uint64_t ref = s_fg_hi | s_fg_last;
  return ref - (uint32_t)(s_fg_last - t);
}
static double fg_epoch(uint64_t t64)               /* FG23 us -> epoch s, using the ESP clock */
{
  int64_t esp_us = esp_timer_get_time();
  double age = (double)(esp_us - ((int64_t)t64 + s_off)) / 1e6;
  return now_epoch() - age;
}

static ac_t *ac_find(uint32_t icao)
{
  for (uint32_t i = 0; i < s_nac; i++) if (s_ac[i].icao == icao) return &s_ac[i];
  return NULL;
}

static ac_t *ac_get(uint32_t icao, uint32_t now, st_t *st)
{
  ac_t *old = NULL;
  for (uint32_t i = 0; i < s_nac; i++) {
    if (s_ac[i].icao == icao) return &s_ac[i];
    if (!old || s_ac[i].last_ms < old->last_ms) old = &s_ac[i];
  }
  ac_t *a = (s_nac < AC_MAX) ? &s_ac[s_nac++] : old;      /* full: reuse the stalest */
  memset(a, 0, sizeof *a);
  s_trn[a - s_ac] = 0; s_trw[a - s_ac] = 0;
  a->icao = icao; a->first_ms = now;
  a->nic = a->nacp = a->sil = a->ver = -1;
  st->newac++; s_tot.newac++;
  return a;
}

/* A global CPR fix is believed only if it is physically possible:
 *  - within RANGE_MAX_M of the station,
 *  - with a recent position: no faster than ~1300 km/h + 10 km slack,
 *  - with none: a second global fix within 20 km (and 30 s) confirms the first.
 * 2026-11-25: RYR7BP appeared ~10 000 km away and its trail drew a line across
 * the map before it jumped to Budapest (a bad even/odd pair, both receivers). */
static double km_between(double la1, double lo1, double la2, double lo2)
{
  const double k = M_PI / 180.0;
  double x = sin((la2 - la1) * k / 2), y = sin((lo2 - lo1) * k / 2);
  return 2.0 * 6371.0 * asin(sqrt(x * x + cos(la1 * k) * cos(la2 * k) * y * y));
}
static bool pos_sane(ac_t *a, double olat, double olon, uint32_t now)
{
  if (km_between(ADSB_REF_LAT, ADSB_REF_LON, a->lat, a->lon) > RANGE_MAX_M / 1000.0) return false;
  if (a->pos_ok && now - a->pos_ms < 120000u) {
    double lim = 10.0 + (now - a->pos_ms) / 1000.0 * 0.36;
    return km_between(olat, olon, a->lat, a->lon) <= lim;
  }
  if (a->cand_ms && now - a->cand_ms < 30000u && km_between(a->cand_lat, a->cand_lon, a->lat, a->lon) < 20.0) {
    a->cand_ms = 0; return true;
  }
  a->cand_lat = (float)a->lat; a->cand_lon = (float)a->lon; a->cand_ms = now | 1;
  return false;
}

static void decode_df17(ac_t *a, const uint8_t *m, uint32_t now, st_t *st, double ts, bool fixed)
{
  static const char cs[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
  uint32_t tc = bits(m, 32, 5);
  if (tc >= 1 && tc <= 4) {
    for (int i = 0; i < 8; i++) a->flight[i] = cs[bits(m, 40 + 6 * i, 6) & 63];
    a->flight[8] = 0;
    uint32_t ca = bits(m, 37, 3);                    /* TC 4/3/2/1 = set A/B/C/D */
    if (ca) { a->cat[0] = (char)('A' + (4 - (int)tc)); a->cat[1] = (char)('0' + ca); a->cat[2] = 0; }
  } else if (tc >= 9 && tc <= 18) {
    static const int8_t nic_tc[19] = { 0,0,0,0,0,0,0,0,0, 11,10,8,7,6,5,4,2,1,0 };
    a->nic = nic_tc[tc];
    uint32_t a12 = bits(m, 40, 12);
    if (a12 & 0x10u) { uint32_t n = ((a12 >> 5) << 4) | (a12 & 0xFu); a->alt = (int32_t)n * 25 - 1000; a->alt_ok = true; }
    int f = (int)bits(m, 53, 1);
    a->cpr_lat[f] = bits(m, 54, 17); a->cpr_lon[f] = bits(m, 71, 17); a->cpr_ms[f] = now; a->cpr_ok[f] = true; a->cpr_fix[f] = fixed;
    st->pos++; s_tot.pos++;
    /* global decode only from two CRC-clean halves: a mis-corrected bit in a CPR
     * field (1-2 bit correction on the FG23 path) moves the fix by a whole zone
     * or more - corrected frames may still refine a known position (local) */
    if (a->cpr_ok[0] && a->cpr_ok[1] && !a->cpr_fix[0] && !a->cpr_fix[1] && (uint32_t)abs((int32_t)(a->cpr_ms[0] - a->cpr_ms[1])) < CPR_PAIR_MS) {
      double olat = a->lat, olon = a->lon;
      if (cpr_global(a, f == 1)) {
        if (pos_sane(a, olat, olon, now)) { a->pos_ok = true; a->pos_ms = now; a->pos_ts = ts; st->gok++; s_tot.gok++; ac_range(a, st); return; }
        /* rejected: keep the old position, demand a fresh even/odd pair */
        a->lat = olat; a->lon = olon; a->cpr_ok[0] = a->cpr_ok[1] = false; st->cpr_rej++; s_tot.cpr_rej++;
        return;
      }
    }
    if (a->pos_ok && now - a->pos_ms < LOCAL_MAX_MS) { cpr_local(a, f); a->pos_ms = now; a->pos_ts = ts; st->lok++; s_tot.lok++; ac_range(a, st); }
  } else if (tc == 19) {
    uint32_t sub = bits(m, 37, 3);
    if (sub == 1 || sub == 2) {
      int k = (sub == 2) ? 4 : 1;
      uint32_t vew = bits(m, 46, 10), vns = bits(m, 57, 10), vr = bits(m, 69, 9);
      if (vew && vns) {
        float vx = (float)((int)vew - 1) * k * (bits(m, 45, 1) ? -1.f : 1.f);
        float vy = (float)((int)vns - 1) * k * (bits(m, 56, 1) ? -1.f : 1.f);
        a->gs = sqrtf(vx * vx + vy * vy);
        float tr = atan2f(vx, vy) * 180.f / (float)M_PI; if (tr < 0) tr += 360.f;
        a->track = tr; a->vel_ok = true;
      }
      if (vr) a->vrate = ((int32_t)vr - 1) * 64 * (bits(m, 68, 1) ? -1 : 1);
    }
    uint32_t gd = bits(m, 81, 7);                    /* ME 50-56: GNSS - baro difference */
    if (gd) { a->geo_diff = ((int32_t)gd - 1) * 25 * (bits(m, 80, 1) ? -1 : 1); a->geo_ok = true; }
  } else if (tc == 31) {
    uint32_t sub = bits(m, 37, 3);                   /* 0 airborne, 1 surface */
    if (sub <= 1) {
      a->ver = (int8_t)bits(m, 72, 3);               /* ME 41-43 */
      if (a->ver >= 1) { a->nacp = (int8_t)bits(m, 76, 4); a->sil = (int8_t)bits(m, 82, 2); }   /* ME 45-48, 51-52 */
    }
  }
}

/* ---------------------------------------------------------------- Beast TCP (readsb --net-connector <ip>,30005,beast_in) */
static void beast_service(void)
{
  if (!s_beast) {
    if (WiFi.status() != WL_CONNECTED || WiFi.getMode() == WIFI_AP) return;
    s_beast = new WiFiServer(BEAST_PORT); s_beast->begin(); s_beast->setNoDelay(true);
  }
  if (s_beast->hasClient()) {
    WiFiClient c = s_beast->available();
    int slot = -1;
    for (int i = 0; i < BEAST_MAX_CLI; i++) if (!(s_bcli[i] && s_bcli[i].connected())) { slot = i; break; }
    if (slot < 0) c.stop(); else { s_bcli[slot] = c; s_bcli[slot].setNoDelay(true); s_bcli[slot].setTimeout(1); }
  }
}
typedef struct { uint8_t b[2 + 2 * (6 + 1 + 14)]; uint8_t n; } beast_msg_t;
static void beast_build(beast_msg_t *o, const uint8_t *m, uint32_t nb, uint64_t t64, int8_t lvl)
{
  uint8_t *out = o->b; uint32_t n = 0;
  uint8_t raw[6 + 1 + 14];
  uint64_t ts = t64 * 12u;                          /* 12 MHz counter, as Beast expects */
  for (int i = 0; i < 6; i++) raw[i] = (uint8_t)(ts >> (8 * (5 - i)));
  float a = powf(10.f, LVL_TO_DBFS(lvl) / 20.f); if (a > 1.f) a = 1.f;
  raw[6] = (uint8_t)lrintf(a * 255.f);
  memcpy(raw + 7, m, nb);
  out[n++] = 0x1A; out[n++] = (nb == 14) ? '3' : '2';
  for (uint32_t i = 0; i < 7 + nb; i++) { out[n++] = raw[i]; if (raw[i] == 0x1A) out[n++] = 0x1A; }
  o->n = (uint8_t)n;
}
static void beast_flush(const beast_msg_t *q, uint32_t nq)       /* outside s_mx; a stuck client is dropped */
{
  if (!nq) return;
  for (int i = 0; i < BEAST_MAX_CLI; i++) {
    if (!(s_bcli[i] && s_bcli[i].connected())) continue;
    for (uint32_t k = 0; k < nq; k++) {
      if (s_bcli[i].write(q[k].b, q[k].n) != q[k].n) { s_bcli[i].stop(); break; }
      s_beast_sent++;
    }
  }
}

/* ---------------------------------------------------------------- recent frames (TFT view) */
#define RF_N 24
static adsb_frm_view_t *s_rf = NULL;   /* PSRAM, RF_N entries (internal RAM is tight) */
static uint32_t        s_rfw = 0;
static double ref_km(double lat, double lon, float *dx, float *dy)
{
  double kx = 111.320 * cos(ADSB_REF_LAT * M_PI / 180.0);
  *dx = (float)((lon - ADSB_REF_LON) * kx); *dy = (float)((lat - ADSB_REF_LAT) * 110.574);
  return sqrt((double)*dx * *dx + (double)*dy * *dy);
}
static void rf_note(const ac_t *a, const uint8_t *m, uint32_t nb, uint32_t df, int8_t lvl, uint32_t now, int src, bool pos_new)
{
  if (!s_rf) s_rf = (adsb_frm_view_t *)heap_caps_calloc(RF_N, sizeof(adsb_frm_view_t), MALLOC_CAP_SPIRAM);
  if (!s_rf) return;
  adsb_frm_view_t *f = &s_rf[s_rfw++ % RF_N];
  f->ms = now; f->icao = a->icao; f->df = (uint8_t)df; f->src = (uint8_t)src; f->lvl = lvl;
  struct timeval tv; gettimeofday(&tv, NULL); f->t = (tv.tv_sec > 1700000000) ? tv.tv_sec : 0;
  f->tc = (df == 17 && nb == 14) ? (uint8_t)(m[4] >> 3) : 0;
  uint32_t tc = f->tc;
  if (df == 17 && tc >= 1 && tc <= 4) snprintf(f->txt, sizeof f->txt, "ID %s", a->flight);
  else if (df == 17 && tc >= 9 && tc <= 18) {
    char al[8] = "  -  ";
    if (a->alt_ok) snprintf(al, sizeof al, "FL%03ld", (long)((a->alt + 50) / 100));
    if (pos_new && a->pos_ok) { float dx, dy; snprintf(f->txt, sizeof f->txt, "%s %3.0fkm", al, ref_km(a->lat, a->lon, &dx, &dy)); }
    else snprintf(f->txt, sizeof f->txt, "%s cpr", al);
  } else if (df == 17 && tc == 19) {
    if (a->vel_ok) snprintf(f->txt, sizeof f->txt, "GS%3.0f T%03.0f %+ld", a->gs, a->track, (long)a->vrate);
    else snprintf(f->txt, sizeof f->txt, "vel");
  } else if (df == 17) snprintf(f->txt, sizeof f->txt, "TC%lu", (unsigned long)tc);
  else snprintf(f->txt, sizeof f->txt, "DF%lu", (unsigned long)df);
}
int adsb_recent(adsb_frm_view_t *o, int max)
{
  if (!s_mx || !s_rf) return 0;
  int n = 0;
  xSemaphoreTake(s_mx, portMAX_DELAY);
  for (uint32_t k = 0; k < RF_N && n < max && k < s_rfw; k++) o[n++] = s_rf[(s_rfw - 1 - k) % RF_N];
  xSemaphoreGive(s_mx);
  return n;
}
int adsb_ac_list(adsb_ac_view_t *o, int max)
{
  if (!s_mx || !s_ac) return 0;
  uint32_t now = millis(); int n = 0;
  xSemaphoreTake(s_mx, portMAX_DELAY);
  for (uint32_t i = 0; i < s_nac && n < max; i++) {
    const ac_t *a = &s_ac[i];
    if (now - a->last_ms > 60000u) continue;
    adsb_ac_view_t *v = &o[n++];
    v->icao = a->icao; memcpy(v->flight, a->flight, sizeof v->flight);
    v->pos_ok = a->pos_ok && now - a->pos_ms < 60000u;
    if (v->pos_ok) ref_km(a->lat, a->lon, &v->dx_km, &v->dy_km); else v->dx_km = v->dy_km = 0;
    v->alt = a->alt; v->alt_ok = a->alt_ok; v->trk = a->track; v->trk_ok = a->vel_ok;
    v->age_ms = now - a->last_ms;
    v->rx = (uint8_t)(((a->seen_ms[0] && now - a->seen_ms[0] < 60000u) ? 1 : 0) |
                      ((a->seen_ms[1] && now - a->seen_ms[1] < 60000u) ? 2 : 0));
  }
  xSemaphoreGive(s_mx);
  return n;
}

/* ---------------------------------------------------------------- feeds */
static void ingest(st_t *st, const uint8_t *m, uint32_t nb, uint32_t df, uint32_t icao, uint32_t fix, int8_t lvl,
                   double ts, uint32_t now, int src);

void adsb_feed_ms1(const uint8_t *raw, uint32_t len)
{
  if (!s_ac || len < 4 || raw[0] != 'M' || raw[1] != 'S' || raw[2] != '1') return;
  uint32_t n = raw[3]; if (4 + 20 * n > len) n = (len - 4) / 20;
  uint32_t now = millis();
  beast_service();
  static beast_msg_t bq[48]; uint32_t nbq = 0;
  xSemaphoreTake(s_mx, portMAX_DELAY);
  st_t *st = st_cur();
  for (uint32_t i = 0; i < n; i++) {
    const uint8_t *r = raw + 4 + 20 * i;
    uint32_t nb = r[4] & 0x1Fu, fix = r[4] >> 6; int8_t lvl = (int8_t)r[5];
    const uint8_t *m = r + 6;
    if (nb != 7 && nb != 14) continue;
    uint32_t df = m[0] >> 3, icao = ((uint32_t)m[1] << 16) | ((uint32_t)m[2] << 8) | m[3];
    if (fix) {                                       /* corrected: only for an aircraft heard clean recently */
      ac_t *k = ac_find(icao);
      if (!k || !k->clean_ms || now - k->clean_ms > FIX_KNOWN_MS) { st->fix_rej++; s_tot.fix_rej++; continue; }
    }
    uint32_t th; memcpy(&th, r, 4);
    uint64_t t64 = fg_unwrap(th);
    double ts = s_off_ok ? fg_epoch(t64) : now_epoch();
    ingest(st, m, nb, df, icao, fix, lvl, ts, now, 0);
    if (nbq < 48) beast_build(&bq[nbq++], m, nb, t64, lvl);
  }
  xSemaphoreGive(s_mx);
  beast_flush(bq, nbq);
}

/* one CRC-good frame into the tables (caller holds s_mx) */
static void ingest(st_t *st, const uint8_t *m, uint32_t nb, uint32_t df, uint32_t icao, uint32_t fix, int8_t lvl,
                   double ts, uint32_t now, int src)
{
  {
    st->msgs++; st->acc[fix > 2 ? 2 : fix]++; s_tot.msgs++; s_tot.acc[fix > 2 ? 2 : fix]++; s_msgs_total++;
    if (df == 11) { st->df11++; s_tot.df11++; } else { st->df17++; s_tot.df17++; }
    if (lvl != -128) {
      st->sig_sum += LVL_TO_DBFS(lvl); st->sig_n++; s_tot.sig_sum += LVL_TO_DBFS(lvl); s_tot.sig_n++;
      if (lvl > st->sig_peak) st->sig_peak = lvl;
      if (lvl > s_tot.sig_peak) s_tot.sig_peak = lvl;
      if (LVL_TO_DBFS(lvl) > -3.f) { st->strong++; s_tot.strong++; }
    }
    ac_t *a = ac_get(icao, now, st);
    a->last_ms = now; a->msgs++; a->last_ts = ts;
    if (lvl != -128) { a->lvl_last = lvl; a->lvl_sum += lvl; a->lvl_n++; }
    if (!fix) a->clean_ms = now;
    a->seen_ms[src] = now | 1;
    if (df == 17 && nb == 14) {
      uint32_t tc = m[4] >> 3;
      if (tc >= 9 && tc <= 18) a->posmsg_ms[src] = now | 1;          /* airborne position heard by this receiver */
      uint32_t pms = a->pos_ms;
      decode_df17(a, m, now, st, ts, fix != 0);
      rf_note(a, m, nb, df, lvl, now, src, a->pos_ms != pms);
    } else {
      rf_note(a, m, nb, df, lvl, now, src, false);
    }
  }
}

/* ---------------------------------------------------------------- RTL-SDR (Beast input) */
/* A PC decoder on the same antenna (tools/rtl_feed.py: rtl_adsb or dump1090 -> Beast)
 * connects here; its frames go into the same tables, counted as msgs_rtl.
 * Hot-swap: the antenna is moved between the FG23 and the RTL-SDR; whichever
 * hears the traffic fills the map and the graphs. The feeder sends only
 * CRC-good DF11/17/18. */
static WiFiServer *s_rtl_srv = NULL;
static WiFiClient  s_rtl_cli;
static uint32_t    s_rtl_frames = 0, s_rtl_last_ms = 0, s_rtl_conn = 0;
static uint8_t     s_rb[64]; static int s_rn = -1; static bool s_resc = false;
static void rtl_frame(const uint8_t *f, int n)
{
  /* f: type, 6 B timestamp, 1 B signal, message */
  int nb = (f[0] == '3') ? 14 : (f[0] == '2') ? 7 : 0;
  if (!nb || n != 1 + 6 + 1 + nb) return;
  const uint8_t *m = f + 8;
  uint32_t df = m[0] >> 3, icao = ((uint32_t)m[1] << 16) | ((uint32_t)m[2] << 8) | m[3];
  if (!(df == 11 || df == 17 || df == 18)) return;
  /* Beast signal = amplitude; 0 = unknown (rtl_adsb gives none): no signal statistics then */
  int l = -128;
  if (f[7]) { float db = 20.0f * log10f(f[7] / 255.0f); l = (int)lrintf(db + 30.0f); if (l < -127) l = -127; if (l > 127) l = 127; }
  uint32_t now = millis();
  xSemaphoreTake(s_mx, portMAX_DELAY);
  st_t *st = st_cur();
  ingest(st, m, nb, df == 18 ? 17 : df, icao, 0, (int8_t)l, now_epoch(), now, 1);
  st->msgs_rtl++; s_tot.msgs_rtl++;
  xSemaphoreGive(s_mx);
  s_rtl_frames++; s_rtl_last_ms = now;
}
static void rtl_service(void)
{
  if (!s_rtl_srv) {
    if (WiFi.status() != WL_CONNECTED || WiFi.getMode() == WIFI_AP) return;
    s_rtl_srv = new WiFiServer(RTL_IN_PORT); s_rtl_srv->begin(); s_rtl_srv->setNoDelay(true);
  }
  if (s_rtl_srv->hasClient()) {                     /* the newest feeder wins (hot-swap, restarts) */
    if (s_rtl_cli) s_rtl_cli.stop();
    s_rtl_cli = s_rtl_srv->available(); s_rtl_cli.setNoDelay(true); s_rtl_conn++; s_rn = -1;
  }
  if (!s_rtl_cli || !s_rtl_cli.connected()) return;
  uint8_t buf[256];
  for (int guard = 0; guard < 16; guard++) {
    int k = s_rtl_cli.available(); if (k <= 0) break;
    if (k > (int)sizeof buf) k = sizeof buf;
    k = s_rtl_cli.read(buf, k); if (k <= 0) break;
    for (int i = 0; i < k; i++) {
      uint8_t c = buf[i];
      if (s_resc) {
        s_resc = false;
        if (c != 0x1A) { s_rn = 0; s_rb[s_rn++] = c; continue; }      /* start of a new frame */
      } else if (c == 0x1A) { s_resc = true; if (s_rn > 0 && s_rn < 64) { } continue; }
      if (s_rn < 0) continue;
      if (s_rn < (int)sizeof s_rb) s_rb[s_rn++] = c;
      int need = (s_rb[0] == '3') ? 1 + 6 + 1 + 14 : (s_rb[0] == '2') ? 1 + 6 + 1 + 7 : 1 + 6 + 1 + 2;
      if (s_rn == need) { rtl_frame(s_rb, s_rn); s_rn = -1; }
    }
  }
}

void adsb_feed_dbg(const aispkt_dbg_t *d)
{
  if (!s_ac) return;
  beast_service();                                   /* accept Beast clients even with no traffic */
  int64_t esp_us = esp_timer_get_time();
  xSemaphoreTake(s_mx, portMAX_DELAY);
  {
    uint32_t fg = d->loops;
    if (fg < s_fg_last) s_fg_hi += 0x100000000ull;
    s_fg_last = fg;
    int64_t off = esp_us - (int64_t)(s_fg_hi | fg);
    uint32_t nowm = millis();
    if (off < s_off_win) s_off_win = off;
    if (!s_off_ok || nowm - s_off_t0 > 60000u) { s_off = s_off_win; s_off_ok = true; s_off_win = INT64_MAX; s_off_t0 = nowm; }
    else if (off < s_off) s_off = off;
  }
  st_t *st = st_cur();
  if (s_dbg_have && d->bytes_rx >= p_bytes) {
    uint32_t db = d->bytes_rx - p_bytes, dt = (uint32_t)d->ftick - p_trig;
    uint16_t dp = (uint16_t)(d->cfgdata_st - p_pre), dov = (uint16_t)(d->rearm - p_ovf);
    st->samples += db; st->trig += dt; st->pre += dp; st->ovf += dov;
    s_tot.samples += db; s_tot.trig += dt; s_tot.pre += dp; s_tot.ovf += dov;
  }
  if (d->bytes_rx > 0) {
    s_dbg_have = true; p_bytes = d->bytes_rx; p_trig = (uint32_t)d->ftick; p_pre = d->cfgdata_st; p_ovf = d->rearm;
  } else s_dbg_have = false;                    /* AM restarted */
  st->noise_sum += d->rssi_qdbm / 4.0f; st->noise_n++;
  s_tot.noise_sum += d->rssi_qdbm / 4.0f; s_tot.noise_n++;
  xSemaphoreGive(s_mx);
}

uint32_t adsb_aircraft_count(void)
{
  uint32_t n = 0, now = millis();
  for (uint32_t i = 0; i < s_nac; i++) if (now - s_ac[i].last_ms < AC_SHOW_MS) n++;
  return n;
}

/* ---------------------------------------------------------------- JSON */
static void J(const char *fmt, ...)
{
  if (!s_buf) return;
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(s_buf + s_len, s_cap - s_len, fmt, ap);
  va_end(ap);
  if (n > 0) s_len = (s_len + (size_t)n < s_cap) ? s_len + (size_t)n : s_cap - 1;
}
static uint32_t s_http_n = 0, s_http_slow = 0, s_http_maxms = 0;
static void send_json(void)
{
  uint32_t t0 = millis();
  s_srv->sendHeader("Access-Control-Allow-Origin", "*");
  s_srv->sendHeader("Cache-Control", "no-cache");
  s_srv->setContentLength(s_len);
  s_srv->send(200, "application/json", "");
  s_srv->sendContent(s_buf, s_len);
  uint32_t dt = millis() - t0;
  s_http_n++; if (dt > 2000) s_http_slow++; if (dt > s_http_maxms) s_http_maxms = dt;
}

static void h_aircraft(void)
{
  xSemaphoreTake(s_mx, portMAX_DELAY);
  uint32_t now = millis(); double ep = now_epoch();
  s_len = 0;
  J("{\"now\":%.3f,\"messages\":%lu,\"aircraft\":[", ep, (unsigned long)s_msgs_total);
  bool first = true;
  for (uint32_t i = 0; i < s_nac; i++) {
    ac_t *a = &s_ac[i];
    if (now - a->last_ms >= AC_SHOW_MS) continue;
    J("%s{\"hex\":\"%06lx\",\"type\":\"adsb_icao\"", first ? "" : ",", (unsigned long)a->icao); first = false;
    { bool f0 = a->seen_ms[0] && now - a->seen_ms[0] < 60000u, f1 = a->seen_ms[1] && now - a->seen_ms[1] < 60000u;
      J(",\"ionos_rx\":\"%s\"", f0 && f1 ? "both" : f1 ? "rtl" : f0 ? "fg23" : "none"); }
    if (a->flight[0]) J(",\"flight\":\"%s\"", a->flight);
    if (a->cat[0]) J(",\"category\":\"%s\"", a->cat);
    if (a->alt_ok) J(",\"alt_baro\":%ld", (long)a->alt);
    if (a->alt_ok && a->geo_ok) J(",\"alt_geom\":%ld,\"geom_minus_baro\":%ld", (long)(a->alt + a->geo_diff), (long)a->geo_diff);
    if (a->nic >= 0) J(",\"nic\":%d", a->nic);
    if (a->nacp >= 0) J(",\"nac_p\":%d", a->nacp);
    if (a->sil >= 0) J(",\"sil\":%d", a->sil);
    if (a->ver >= 0) J(",\"version\":%d", a->ver);
    if (a->vel_ok) J(",\"gs\":%.1f,\"track\":%.1f,\"baro_rate\":%ld", a->gs, a->track, (long)a->vrate);
    if (a->pos_ok) J(",\"lat\":%.6f,\"lon\":%.6f,\"seen_pos\":%.3f,\"pos_ts\":%.3f", a->lat, a->lon,
                     ep - a->pos_ts, a->pos_ts);
    J(",\"messages\":%lu,\"seen\":%.3f,\"last_ts\":%.3f,\"rssi\":%.1f}", (unsigned long)a->msgs, ep - a->last_ts, a->last_ts,
      LVL_TO_DBFS(a->lvl_n ? a->lvl_sum / a->lvl_n : a->lvl_last));
  }
  J("]}");
  xSemaphoreGive(s_mx);
  send_json();
}

static void h_receiver(void)
{
  s_len = 0;
  J("{\"version\":\"ionos-mini FG23 on-chip Mode S\",\"refresh\":1000,\"history\":0,"
    "\"readsb\":false,\"binCraft\":false,\"zstd\":false,"
    "\"note\":\"rssi = FG23 soft level - 30, not calibrated; noise = FG23 RSSI in dBm\"}");
  send_json();
}

static void j_period(const char *name, int minutes, double ep, uint32_t cur)
{
  st_t a; memset(&a, 0, sizeof a); a.sig_peak = -128;
  uint32_t got = 0;
  if (minutes > 0) {
    for (int k = 1; k <= minutes; k++) {            /* completed minutes only, like readsb */
      if ((uint32_t)k > cur) break;
      const st_t *b = &s_st[(cur - k) % ST_N];
      if (b->min == cur - k) { st_add(&a, b); got++; }
    }
  } else { st_add(&a, &s_tot); }
  double span = minutes > 0 ? minutes * 60.0 : millis() / 1000.0;
  J("\"%s\":{\"start\":%.1f,\"end\":%.1f,\"messages\":%lu,\"messages_valid\":%lu,\"position_count_total\":%lu,"
    "\"max_distance\":%.0f,", name, ep - span, ep, (unsigned long)a.msgs, (unsigned long)a.msgs, (unsigned long)a.pos, a.maxd);
  J("\"local\":{\"samples_processed\":%llu,\"samples_dropped\":0,\"samples_lost\":0,\"modeac\":0,\"modes\":%lu,"
    "\"bad\":%lu,\"unknown_icao\":0,\"accepted\":[%lu,%lu,%lu],", (unsigned long long)a.samples, (unsigned long)a.pre,
    (unsigned long)(a.pre > a.msgs ? a.pre - a.msgs : 0), (unsigned long)a.acc[0], (unsigned long)a.acc[1], (unsigned long)a.acc[2]);
  if (a.sig_n) J("\"signal\":%.1f,\"peak_signal\":%.1f,", a.sig_sum / a.sig_n, LVL_TO_DBFS(a.sig_peak));
  if (a.noise_n) J("\"noise\":%.1f,", a.noise_sum / a.noise_n);
  J("\"strong_signals\":%lu},", (unsigned long)a.strong);
  J("\"cpr\":{\"surface\":0,\"airborne\":%lu,\"global_ok\":%lu,\"global_bad\":0,\"global_range\":0,\"global_speed\":0,"
    "\"global_skipped\":0,\"local_ok\":%lu,\"local_aircraft_relative\":%lu,\"local_receiver_relative\":0,\"local_skipped\":0,"
    "\"local_range\":0,\"local_speed\":0,\"filtered\":0},", (unsigned long)a.pos, (unsigned long)a.gok, (unsigned long)a.lok, (unsigned long)a.lok);
  J("\"tracks\":{\"all\":%lu,\"single_message\":0},", (unsigned long)a.newac);
  J("\"fg23\":{\"triggers\":%lu,\"fifo_full\":%lu,\"df11\":%lu,\"df17\":%lu,\"fix_rejected\":%lu,\"cpr_rejected\":%lu,\"minutes\":%lu}}",
    (unsigned long)a.trig, (unsigned long)a.ovf, (unsigned long)a.df11, (unsigned long)a.df17, (unsigned long)a.fix_rej, (unsigned long)a.cpr_rej, (unsigned long)got);
}

static void h_stats(void)
{
  xSemaphoreTake(s_mx, portMAX_DELAY);
  double ep = now_epoch(); uint32_t cur = millis() / 60000u, now = millis();
  uint32_t wp = 0, wop = 0;
  for (uint32_t i = 0; i < s_nac; i++) {
    if (now - s_ac[i].last_ms >= AC_SHOW_MS) continue;
    if (s_ac[i].pos_ok && now - s_ac[i].pos_ms < 60000u) wp++; else wop++;
  }
  s_len = 0;
  J("{\"now\":%.3f,\"clock\":{\"fg23_synced\":%s,\"ntp\":%s},\"beast\":{\"port\":%d,\"frames_sent\":%lu},\"aircraft_with_pos\":%lu,\"aircraft_without_pos\":%lu,\"aircraft_count_by_type\":{\"adsb_icao\":%lu},",
    ep, s_off_ok ? "true" : "false", ep > 1700000000.0 ? "true" : "false", BEAST_PORT, (unsigned long)s_beast_sent,
    (unsigned long)wp, (unsigned long)wop, (unsigned long)(wp + wop));
  /* "latest" = the running minute */
  { st_t *b = st_cur(); J("\"latest\":{\"start\":%.1f,\"end\":%.1f,\"messages\":%lu},",
      ep - (now % 60000u) / 1000.0, ep, (unsigned long)b->msgs); }
  j_period("last1min", 1, ep, cur); J(",");
  j_period("last5min", 5, ep, cur); J(",");
  j_period("last15min", 15, ep, cur); J(",");
  j_period("total", 0, ep, cur);
  { const st_t *b = st_cur();
    J(",\"sources\":{\"rtl_in\":{\"port\":%d,\"connected\":%s,\"connections\":%lu,\"frames\":%lu,\"last_s\":%.1f},"
      "\"this_minute\":{\"fg23\":%lu,\"rtl\":%lu},\"total\":{\"fg23\":%lu,\"rtl\":%lu}}",
      RTL_IN_PORT, (s_rtl_cli && s_rtl_cli.connected()) ? "true" : "false", (unsigned long)s_rtl_conn,
      (unsigned long)s_rtl_frames, s_rtl_last_ms ? (now - s_rtl_last_ms) / 1000.0 : -1.0,
      (unsigned long)(b->msgs - b->msgs_rtl), (unsigned long)b->msgs_rtl,
      (unsigned long)(s_tot.msgs - s_tot.msgs_rtl), (unsigned long)s_tot.msgs_rtl); }
  J("}");
  xSemaphoreGive(s_mx);
  send_json();
}

/* one history record per completed minute (called from the HTTP task, under s_mx) */
static void hist_tick(void)
{
  uint32_t cur = millis() / 60000u;
  if (s_hist_min == UINT32_MAX) { s_hist_min = cur; return; }
  if (cur == s_hist_min) return;
  uint32_t m = cur - 1u, now = millis();
  const st_t *b = &s_st[m % ST_N];
  hist_t h; memset(&h, 0, sizeof h);
  h.t = (uint32_t)now_epoch(); h.sig = h.peak = h.noise = NAN;
  if (b->min == m) {
    h.msgs = (uint16_t)b->msgs; h.rtl = (uint16_t)b->msgs_rtl; h.pos = (uint16_t)b->pos; h.tracks = (uint16_t)b->newac; h.df17 = (uint16_t)b->df17;
    if (b->sig_n) { h.sig = b->sig_sum / b->sig_n; h.peak = LVL_TO_DBFS(b->sig_peak); }
    if (b->noise_n) h.noise = b->noise_sum / b->noise_n;
    h.maxd = b->maxd; h.msps = (float)(b->samples / 60e6); h.trig = b->trig; h.pre = b->pre; h.ovf = b->ovf;
  }
  for (uint32_t i = 0; i < s_nac; i++) {
    if (now - s_ac[i].last_ms < 60000u) h.ac++;
    for (int k = 0; k < 2; k++) {
      if (s_ac[i].seen_ms[k] && now - s_ac[i].seen_ms[k] < 60000u) h.ac_rx[k]++;
      if (s_ac[i].posmsg_ms[k] && now - s_ac[i].posmsg_ms[k] < 60000u) h.acpos_rx[k]++;
    }
    if (s_ac[i].pos_ok && now - s_ac[i].pos_ms < 60000u) h.acpos++;
  }
  s_hist[s_hw % HIST_N] = h; s_hw++;
  s_hist_min = cur;
}
static void jf(const char *pre, float v, int dec)
{
  if (isnan(v)) J("%snull", pre); else J(dec ? "%s%.1f" : "%s%.0f", pre, v);
}

static void h_history(void)
{
  uint32_t mins = 1440;
  if (s_srv->hasArg("mins")) { long v = s_srv->arg("mins").toInt(); if (v > 0 && v < 1440) mins = (uint32_t)v; }
  xSemaphoreTake(s_mx, portMAX_DELAY);
  uint32_t n = s_hw < HIST_N ? s_hw : HIST_N; if (n > mins) n = mins;
  uint32_t first = s_hw - n;
  s_len = 0;
  J("{\"now\":%.1f,\"step\":60,\"n\":%lu", now_epoch(), (unsigned long)n);
  const char *names[] = { "t", "msgs", "pos", "ac", "acpos", "tracks", "df17", "sig", "peak", "noise", "range_km", "msps", "trig", "pre", "ovf", "msgs_rtl", "ac_fg23", "ac_rtl", "acpos_fg23", "acpos_rtl" };
  for (int f = 0; f < 20; f++) {
    J(",\"%s\":[", names[f]);
    for (uint32_t i = 0; i < n; i++) {
      const hist_t *h = &s_hist[(first + i) % HIST_N];
      const char *c = i ? "," : "";
      switch (f) {
        case 0: J("%s%lu", c, (unsigned long)h->t); break;
        case 1: J("%s%u", c, h->msgs); break;
        case 2: J("%s%u", c, h->pos); break;
        case 3: J("%s%u", c, h->ac); break;
        case 4: J("%s%u", c, h->acpos); break;
        case 5: J("%s%u", c, h->tracks); break;
        case 6: J("%s%u", c, h->df17); break;
        case 7: jf(c, h->sig, 1); break;
        case 8: jf(c, h->peak, 1); break;
        case 9: jf(c, h->noise, 1); break;
        case 10: jf(c, h->maxd / 1000.0f, 1); break;
        case 11: J("%s%.2f", c, h->msps); break;
        case 12: J("%s%lu", c, (unsigned long)h->trig); break;
        case 13: J("%s%lu", c, (unsigned long)h->pre); break;
        case 14: J("%s%lu", c, (unsigned long)h->ovf); break;
        case 15: J("%s%u", c, h->rtl); break;
        case 16: J("%s%u", c, h->ac_rx[0]); break;
        case 17: J("%s%u", c, h->ac_rx[1]); break;
        case 18: J("%s%u", c, h->acpos_rx[0]); break;
        case 19: J("%s%u", c, h->acpos_rx[1]); break;
      }
    }
    J("]");
  }
  J("}");
  xSemaphoreGive(s_mx);
  send_json();
}

static void h_trace(void)                         /* /data/trace.json?hex=xxxxxx -> [[t,lat,lon,alt],...] */
{
  uint32_t icao = (uint32_t)strtoul(s_srv->arg("hex").c_str(), NULL, 16);
  xSemaphoreTake(s_mx, portMAX_DELAY);
  s_len = 0;
  J("{\"now\":%.1f,\"hex\":\"%06lx\",\"trace\":[", now_epoch(), (unsigned long)icao);
  ac_t *a = ac_find(icao);
  if (a && s_tr) {
    uint32_t i = (uint32_t)(a - s_ac), n = s_trn[i], w = s_trw[i];
    const trp_t *tr = s_tr + i * TR_N;
    for (uint32_t k = 0; k < n; k++) {
      const trp_t *p = &tr[(w + TR_N - n + k) % TR_N];
      if (p->alt == INT32_MIN) J("%s[%lu,%.5f,%.5f,null]", k ? "," : "", (unsigned long)p->t, p->lat, p->lon);
      else J("%s[%lu,%.5f,%.5f,%ld]", k ? "," : "", (unsigned long)p->t, p->lat, p->lon, (long)p->alt);
    }
  }
  J("]}");
  xSemaphoreGive(s_mx);
  send_json();
}

static void h_outline(void)
{
  xSemaphoreTake(s_mx, portMAX_DELAY);
  s_len = 0;
  J("{\"note\":\"farthest position per 5 deg sector from a reference point, since boot\",\"actualRange\":{\"last24h\":{\"points\":[");
  bool first = true;
  for (int i = 0; i < RNG_N; i++) {
    if (s_rng[i].d <= 0) continue;
    J("%s[%.4f,%.4f,%ld]", first ? "" : ",", s_rng[i].lat, s_rng[i].lon, (long)s_rng[i].alt); first = false;
  }
  J("]}}}");
  xSemaphoreGive(s_mx);
  send_json();
}

static void h_bench_post(void)
{
  /* body: t,ref,ours,matched,only,c0,c1,c2,c3,c4,maxkm  (cN = coverage % per distance band, 255 = no aircraft) */
  if (!s_srv->hasArg("plain")) { s_srv->send(400, "text/plain", "no body\n"); return; }
  unsigned long v[11]; memset(v, 0, sizeof v);
  int n = sscanf(s_srv->arg("plain").c_str(), "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
                 &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9], &v[10]);
  if (n != 11) { s_srv->send(400, "text/plain", "expected 11 fields\n"); return; }
  bench_t b; b.t = v[0]; b.ref = (uint16_t)v[1]; b.ours = (uint16_t)v[2]; b.matched = (uint16_t)v[3]; b.only = (uint16_t)v[4];
  for (int i = 0; i < 5; i++) b.cov[i] = (uint8_t)(v[5 + i] > 255 ? 255 : v[5 + i]);
  b.maxkm = (uint16_t)v[10];
  xSemaphoreTake(s_mx, portMAX_DELAY);
  s_bench[s_bw % BENCH_N] = b; s_bw++;
  xSemaphoreGive(s_mx);
  s_srv->send(200, "text/plain", "ok\n");
}

static void h_bench(void)
{
  uint32_t mins = 1440;
  if (s_srv->hasArg("mins")) { long v = s_srv->arg("mins").toInt(); if (v > 0 && v < 1440) mins = (uint32_t)v; }
  xSemaphoreTake(s_mx, portMAX_DELAY);
  uint32_t n = s_bw < BENCH_N ? s_bw : BENCH_N; if (n > mins) n = mins;
  uint32_t first = s_bw - n;
  s_len = 0;
  J("{\"source\":\"adsb.fi (https://adsb.fi), circle of 250 NM around the receiver\",\"step\":60,\"n\":%lu,"
    "\"bands_km\":[\"0-50\",\"50-100\",\"100-200\",\"200-300\",\"300+\"]", (unsigned long)n);
  const char *names[] = { "t", "ref", "ours", "matched", "only_ours", "cov0", "cov1", "cov2", "cov3", "cov4", "max_km" };
  for (int f = 0; f < 11; f++) {
    J(",\"%s\":[", names[f]);
    for (uint32_t i = 0; i < n; i++) {
      const bench_t *b = &s_bench[(first + i) % BENCH_N];
      const char *c = i ? "," : "";
      switch (f) {
        case 0: J("%s%lu", c, (unsigned long)b->t); break;
        case 1: J("%s%u", c, b->ref); break;
        case 2: J("%s%u", c, b->ours); break;
        case 3: J("%s%u", c, b->matched); break;
        case 4: J("%s%u", c, b->only); break;
        case 10: J("%s%u", c, b->maxkm); break;
        default: { uint8_t v = b->cov[f - 5]; if (v == 255) J("%snull", c); else J("%s%u", c, v); }
      }
    }
    J("]");
  }
  J("}");
  xSemaphoreGive(s_mx);
  send_json();
}

static void h_graphs(void)
{
  s_srv->sendHeader("Cache-Control", "no-cache");
  s_srv->send_P(200, "text/html; charset=utf-8", ADSB_GRAPHS_HTML, sizeof ADSB_GRAPHS_HTML - 1);
}

static void h_map(void)
{
  s_srv->sendHeader("Cache-Control", "no-cache");
  s_srv->send_P(200, "text/html; charset=utf-8", ADSB_MAP_HTML, sizeof ADSB_MAP_HTML - 1);
}

static void h_root(void)
{
  char t[256];
  snprintf(t, sizeof t, "ionos-mini FG23 ADS-B\naircraft (5 min): %lu\nmessages: %lu\n"
           "/data/aircraft.json /data/receiver.json /data/stats.json\n",
           (unsigned long)adsb_aircraft_count(), (unsigned long)s_msgs_total);
  s_srv->sendHeader("Access-Control-Allow-Origin", "*");
  s_srv->send(200, "text/plain", t);
}

static void http_task(void *arg)
{
  (void)arg;
  bool up = false;
  for (;;) {
    if (!up) {
      if (WiFi.status() == WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
        configTime(0, 0, "pool.ntp.org", "time.google.com");
        s_srv = new WebServer(ADSB_HTTP_PORT);
        s_srv->on("/", h_map);
        s_srv->on("/index.html", h_map);
        s_srv->on("/status", h_root);
        s_srv->on("/data/aircraft.json", h_aircraft);
        s_srv->on("/data/receiver.json", h_receiver);
        s_srv->on("/data/stats.json", h_stats);
        s_srv->on("/data/history.json", h_history);
        s_srv->on("/data/outline.json", h_outline);
        s_srv->on("/data/trace.json", h_trace);
        s_srv->on("/graphs", h_graphs);
        s_srv->on("/data/bench", HTTP_POST, h_bench_post);
        s_srv->on("/data/bench.json", h_bench);
        s_srv->begin();
        up = true;
      } else { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
    }
    s_srv->handleClient();
    rtl_service();
    if (xSemaphoreTake(s_mx, pdMS_TO_TICKS(1000)) == pdTRUE) { hist_tick(); xSemaphoreGive(s_mx); }
    static uint32_t t_diag = 0, low = 0;
    if (millis() - t_diag >= 10000) {
      t_diag = millis();
      size_t fr = heap_caps_get_free_size(MALLOC_CAP_INTERNAL), mn = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
      size_t blk = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
      int bc = 0; for (int i = 0; i < BEAST_MAX_CLI; i++) if (s_bcli[i] && s_bcli[i].connected()) bc++;
      Serial0.printf("ADSB-NET wifi %d rssi %d | int free %u min %u blk %u | http %lu slow %lu max %lu ms | beast cli %d sent %lu\n",
                    (int)WiFi.status(), (int)WiFi.RSSI(), (unsigned)fr, (unsigned)mn, (unsigned)blk,
                    (unsigned long)s_http_n, (unsigned long)s_http_slow, (unsigned long)s_http_maxms, bc, (unsigned long)s_beast_sent);
      s_http_maxms = 0;
      /* self-heal: lwIP needs internal RAM; if it stays exhausted the web server only times out */
      if (fr < 3000 || blk < 1200) { if (++low >= 6) { Serial0.println("ADSB-NET internal RAM exhausted -> restart"); delay(100); esp_restart(); } }
      else low = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void adsb_init(void)
{
  if (s_ac) return;
  s_ac = (ac_t *)ps_calloc(AC_MAX, sizeof(ac_t));
  s_cap = 256 * 1024; s_buf = (char *)ps_malloc(s_cap);
  s_hist = (hist_t *)ps_calloc(HIST_N, sizeof(hist_t));
  s_bench = (bench_t *)ps_calloc(BENCH_N, sizeof(bench_t));
  s_tr = (trp_t *)ps_calloc((size_t)AC_MAX * TR_N, sizeof(trp_t));   /* 1 MB; no trails if it fails */
  if (!s_ac || !s_buf || !s_hist || !s_bench) { s_ac = NULL; return; }
  for (int i = 0; i < ST_N; i++) s_st[i].min = UINT32_MAX;
  memset(&s_tot, 0, sizeof s_tot); s_tot.sig_peak = -128;
  s_mx = xSemaphoreCreateMutex();
  /* stack in PSRAM: internal RAM is only ~10 KB free with streaming running, and lwIP needs it */
  if (xTaskCreatePinnedToCoreWithCaps(http_task, "adsb_http", 8192, NULL, 1, NULL, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS)
    xTaskCreatePinnedToCore(http_task, "adsb_http", 8192, NULL, 1, NULL, 0);
}

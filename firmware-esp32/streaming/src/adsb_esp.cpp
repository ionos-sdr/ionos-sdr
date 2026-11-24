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
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define AC_MAX         256
#define AC_SHOW_MS     300000u      /* aircraft.json lists those seen in the last 5 min */
#define CPR_PAIR_MS    10000u
#define LOCAL_MAX_MS   600000u
#define LVL_TO_DBFS(l) ((float)(l) - 30.0f)

typedef struct {
  uint32_t icao, first_ms, last_ms, pos_ms, msgs;
  char     flight[9];
  int32_t  alt; bool alt_ok;
  float    gs, track; int32_t vrate; bool vel_ok;
  double   lat, lon; bool pos_ok;
  uint32_t cpr_lat[2], cpr_lon[2], cpr_ms[2]; bool cpr_ok[2];
  int8_t   lvl_last; float lvl_sum; uint32_t lvl_n;
} ac_t;

typedef struct {
  uint32_t min;                       /* millis()/60000 of this bucket, UINT32_MAX = empty */
  uint32_t msgs, acc[3], df11, df17, pos, gok, lok, newac;
  float    sig_sum; uint32_t sig_n; int32_t sig_peak; uint32_t strong;
  uint64_t samples; uint32_t pre, trig, ovf;
  float    noise_sum; uint32_t noise_n;
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
static char             *s_buf = NULL;
static size_t            s_cap = 0, s_len = 0;

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

static ac_t *ac_get(uint32_t icao, uint32_t now, st_t *st)
{
  ac_t *old = NULL;
  for (uint32_t i = 0; i < s_nac; i++) {
    if (s_ac[i].icao == icao) return &s_ac[i];
    if (!old || s_ac[i].last_ms < old->last_ms) old = &s_ac[i];
  }
  ac_t *a = (s_nac < AC_MAX) ? &s_ac[s_nac++] : old;      /* full: reuse the stalest */
  memset(a, 0, sizeof *a);
  a->icao = icao; a->first_ms = now;
  st->newac++; s_tot.newac++;
  return a;
}

static void decode_df17(ac_t *a, const uint8_t *m, uint32_t now, st_t *st)
{
  static const char cs[] = "#ABCDEFGHIJKLMNOPQRSTUVWXYZ##### ###############0123456789######";
  uint32_t tc = bits(m, 32, 5);
  if (tc >= 1 && tc <= 4) {
    for (int i = 0; i < 8; i++) a->flight[i] = cs[bits(m, 40 + 6 * i, 6) & 63];
    a->flight[8] = 0;
  } else if (tc >= 9 && tc <= 18) {
    uint32_t a12 = bits(m, 40, 12);
    if (a12 & 0x10u) { uint32_t n = ((a12 >> 5) << 4) | (a12 & 0xFu); a->alt = (int32_t)n * 25 - 1000; a->alt_ok = true; }
    int f = (int)bits(m, 53, 1);
    a->cpr_lat[f] = bits(m, 54, 17); a->cpr_lon[f] = bits(m, 71, 17); a->cpr_ms[f] = now; a->cpr_ok[f] = true;
    st->pos++; s_tot.pos++;
    if (a->cpr_ok[0] && a->cpr_ok[1] && (uint32_t)abs((int32_t)(a->cpr_ms[0] - a->cpr_ms[1])) < CPR_PAIR_MS) {
      if (cpr_global(a, f == 1)) { a->pos_ok = true; a->pos_ms = now; st->gok++; s_tot.gok++; return; }
    }
    if (a->pos_ok && now - a->pos_ms < LOCAL_MAX_MS) { cpr_local(a, f); a->pos_ms = now; st->lok++; s_tot.lok++; }
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
  }
}

/* ---------------------------------------------------------------- feeds */
void adsb_feed_ms1(const uint8_t *raw, uint32_t len)
{
  if (!s_ac || len < 4 || raw[0] != 'M' || raw[1] != 'S' || raw[2] != '1') return;
  uint32_t n = raw[3]; if (4 + 20 * n > len) n = (len - 4) / 20;
  uint32_t now = millis();
  xSemaphoreTake(s_mx, portMAX_DELAY);
  st_t *st = st_cur();
  for (uint32_t i = 0; i < n; i++) {
    const uint8_t *r = raw + 4 + 20 * i;
    uint32_t nb = r[4] & 0x1Fu, fix = r[4] >> 6; int8_t lvl = (int8_t)r[5];
    const uint8_t *m = r + 6;
    if (nb != 7 && nb != 14) continue;
    uint32_t df = m[0] >> 3, icao = ((uint32_t)m[1] << 16) | ((uint32_t)m[2] << 8) | m[3];
    st->msgs++; st->acc[fix > 2 ? 2 : fix]++; s_tot.msgs++; s_tot.acc[fix > 2 ? 2 : fix]++; s_msgs_total++;
    if (df == 11) { st->df11++; s_tot.df11++; } else { st->df17++; s_tot.df17++; }
    st->sig_sum += LVL_TO_DBFS(lvl); st->sig_n++; s_tot.sig_sum += LVL_TO_DBFS(lvl); s_tot.sig_n++;
    if (lvl > st->sig_peak) st->sig_peak = lvl;
    if (lvl > s_tot.sig_peak) s_tot.sig_peak = lvl;
    if (LVL_TO_DBFS(lvl) > -3.f) { st->strong++; s_tot.strong++; }
    ac_t *a = ac_get(icao, now, st);
    a->last_ms = now; a->msgs++; a->lvl_last = lvl; a->lvl_sum += lvl; a->lvl_n++;
    if (df == 17 && nb == 14) decode_df17(a, m, now, st);
  }
  xSemaphoreGive(s_mx);
}

void adsb_feed_dbg(const aispkt_dbg_t *d)
{
  if (!s_ac) return;
  xSemaphoreTake(s_mx, portMAX_DELAY);
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
static void send_json(void)
{
  s_srv->sendHeader("Access-Control-Allow-Origin", "*");
  s_srv->sendHeader("Cache-Control", "no-cache");
  s_srv->setContentLength(s_len);
  s_srv->send(200, "application/json", "");
  s_srv->sendContent(s_buf, s_len);
}

static void h_aircraft(void)
{
  xSemaphoreTake(s_mx, portMAX_DELAY);
  uint32_t now = millis(); double ep = now_epoch();
  s_len = 0;
  J("{\"now\":%.1f,\"messages\":%lu,\"aircraft\":[", ep, (unsigned long)s_msgs_total);
  bool first = true;
  for (uint32_t i = 0; i < s_nac; i++) {
    ac_t *a = &s_ac[i];
    if (now - a->last_ms >= AC_SHOW_MS) continue;
    J("%s{\"hex\":\"%06lx\",\"type\":\"adsb_icao\"", first ? "" : ",", (unsigned long)a->icao); first = false;
    if (a->flight[0]) J(",\"flight\":\"%s\"", a->flight);
    if (a->alt_ok) J(",\"alt_baro\":%ld", (long)a->alt);
    if (a->vel_ok) J(",\"gs\":%.1f,\"track\":%.1f,\"baro_rate\":%ld", a->gs, a->track, (long)a->vrate);
    if (a->pos_ok) J(",\"lat\":%.6f,\"lon\":%.6f,\"seen_pos\":%.1f", a->lat, a->lon, (now - a->pos_ms) / 1000.0);
    J(",\"messages\":%lu,\"seen\":%.1f,\"rssi\":%.1f}", (unsigned long)a->msgs, (now - a->last_ms) / 1000.0,
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
    "\"max_distance\":0,", name, ep - span, ep, (unsigned long)a.msgs, (unsigned long)a.msgs, (unsigned long)a.pos);
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
  J("\"fg23\":{\"triggers\":%lu,\"fifo_full\":%lu,\"df11\":%lu,\"df17\":%lu,\"minutes\":%lu}}",
    (unsigned long)a.trig, (unsigned long)a.ovf, (unsigned long)a.df11, (unsigned long)a.df17, (unsigned long)got);
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
  J("{\"now\":%.1f,\"aircraft_with_pos\":%lu,\"aircraft_without_pos\":%lu,\"aircraft_count_by_type\":{\"adsb_icao\":%lu},",
    ep, (unsigned long)wp, (unsigned long)wop, (unsigned long)(wp + wop));
  /* "latest" = the running minute */
  { st_t *b = st_cur(); J("\"latest\":{\"start\":%.1f,\"end\":%.1f,\"messages\":%lu},",
      ep - (now % 60000u) / 1000.0, ep, (unsigned long)b->msgs); }
  j_period("last1min", 1, ep, cur); J(",");
  j_period("last5min", 5, ep, cur); J(",");
  j_period("last15min", 15, ep, cur); J(",");
  j_period("total", 0, ep, cur);
  J("}");
  xSemaphoreGive(s_mx);
  send_json();
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
        s_srv->on("/", h_root);
        s_srv->on("/data/aircraft.json", h_aircraft);
        s_srv->on("/data/receiver.json", h_receiver);
        s_srv->on("/data/stats.json", h_stats);
        s_srv->begin();
        up = true;
      } else { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }
    }
    s_srv->handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void adsb_init(void)
{
  if (s_ac) return;
  s_ac = (ac_t *)ps_calloc(AC_MAX, sizeof(ac_t));
  s_cap = 96 * 1024; s_buf = (char *)ps_malloc(s_cap);
  if (!s_ac || !s_buf) { s_ac = NULL; return; }
  for (int i = 0; i < ST_N; i++) s_st[i].min = UINT32_MAX;
  memset(&s_tot, 0, sizeof s_tot); s_tot.sig_peak = -128;
  s_mx = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(http_task, "adsb_http", 8192, NULL, 1, NULL, 0);
}

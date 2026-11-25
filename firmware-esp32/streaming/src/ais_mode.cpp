/* ais_mode.cpp — AIS receive mode: ais_rx.c (the same file the PC harness
 * aisport.c tests) + console output + NMEA over TCP 10110.
 *
 * EXPERIMENTAL (exp/fg23-ais-onchip): frames decoded on the FG23 arrive as
 * AIS1 blocks (aispkt.h) and leave through the same emit(); AISBENCH
 * alternates the two paths for a sensitivity comparison.
 *
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT
 */
#include "ais_mode.h"
#include "ais_rx.h"
#include "aispkt.h"
#include "adsb_esp.h"
#include <Arduino.h>
#include <WiFi.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "menu.h"

Stream &diag_stream(void);

#define AIS_MAX_CLI   3
#define AIS_MMSI_MAX  64

/* I/Q path: ais_multi (DDC per channel + frequency hypotheses), in PSRAM.
 * dual (default): 162.000 MHz, R=4 -> 100 ksps, channels A and B;
 * single: 161.985 MHz, R=8 -> 50 ksps, channel A only (the 2026-11-23 setup). */
static ais_multi_t *s_m = NULL;
static uint32_t   s_sps = 0;
static bool       s_dual = true;
static int        s_nhyp = 1;                /* >1: frequency hypotheses (AFC off) */
static int32_t    s_step = 1500;
static int        s_afc = 8;                 /* AFC EMA shift, 0 = off (used with 1 hypothesis) */
static bool       s_reinit = false;
static uint32_t   s_rej = 0;                 /* frames dropped by the MMSI / type check */
static int64_t    s_cpu_us = 0, s_cpu_t0 = 0; static float s_cpu_pct = 0;
static uint32_t   s_iq_frames = 0;
static uint32_t   s_last_ms = 0;
static char       s_last[48] = "";
static uint32_t   s_mmsi[AIS_MMSI_MAX];
static uint32_t   s_nmmsi = 0;
static uint32_t   s_hw_frames = 0, s_hw_blocks = 0;
static aispkt_stats_t s_hw_st;
static int16_t    s_hw_noise_qdbm = INT16_MIN;
static bool       s_hw_dbg = false;          /* AISDBG: telemetry line per AIS1 block */
void ais_hw_dbg_toggle(void) { s_hw_dbg = !s_hw_dbg; diag_stream().printf("AISDBG %s\n", s_hw_dbg ? "on" : "off"); }

static WiFiServer s_srv(AIS_NMEA_PORT);
static WiFiClient s_cli[AIS_MAX_CLI];
static bool       s_srv_on = false;

static void bench_note(bool hw, uint32_t mmsi, float dbm);

static uint32_t frame_mmsi(const uint8_t *fr, int len)
{
  if (len < 7) return 0;
  /* MMSI = message bits 8..37 (frame bytes are already MSB-first, see ais_rx.c) */
  uint32_t v = 0;
  for (int k = 8; k < 38; k++) v = (v << 1) | ((fr[k >> 3] >> (7 - (k & 7))) & 1);
  return v;
}

static void note_mmsi(uint32_t v)
{
  for (uint32_t i = 0; i < s_nmmsi; i++) if (s_mmsi[i] == v) return;
  if (s_nmmsi < AIS_MMSI_MAX) s_mmsi[s_nmmsi++] = v;
}

/* one decoded frame, from either path -> console, NMEA clients, bench.
 * $PIONS,AIS,<chan>,<level dBFS>,<SNR dB>,<foff Hz>,<src>,<level dBm>
 * For HW frames the dBFS field is the dBm level mapped back with AIS_DBM_K,
 * so older readers of the first three fields keep working. */
#define RV_N 24
static ais_msg_view_t *s_rv = NULL;   /* PSRAM */
static uint32_t       s_rvw = 0;
int ais_recent(ais_msg_view_t *o, int max)
{
  if (!s_rv) return 0;
  int n = 0;   /* written from the main loop only, read from the main loop: no lock */
  for (uint32_t k = 0; k < RV_N && n < max && k < s_rvw; k++) o[n++] = s_rv[(s_rvw - 1 - k) % RV_N];
  return n;
}

static void emit(const uint8_t *fr, int len, char chan, float lvl_dbfs, float snr_db,
                 float foff_hz, bool hw, float dbm)
{
  char d[96], n[320];
  ais_describe(fr, len, d, sizeof d);
  ais_nmea(fr, len, chan, n, sizeof n);
  snprintf(s_last, sizeof s_last, "%.47s", d);
  s_last_ms = millis();
  {
    if (!s_rv) s_rv = (ais_msg_view_t *)heap_caps_calloc(RV_N, sizeof(ais_msg_view_t), MALLOC_CAP_SPIRAM);
    ais_msg_view_t *v = s_rv ? &s_rv[s_rvw++ % RV_N] : NULL;
    if (v) {
    v->ms = s_last_ms; time_t tt = time(NULL); v->t = (tt > 1700000000) ? (long)tt : 0;
    v->chan = chan; v->hw = hw; v->dbm = dbm; v->foff = foff_hz;
    snprintf(v->txt, sizeof v->txt, "%.47s", d);
    }
  }
  uint32_t mmsi = frame_mmsi(fr, len);
  note_mmsi(mmsi);
  bench_note(hw, mmsi, dbm);
  char p[96];
  int pl = snprintf(p, sizeof p, "PIONS,AIS,%c,%.1f,%.1f,%.0f,%s,%.1f", chan,
                    (double)lvl_dbfs, (double)snr_db, (double)foff_hz, hw ? "HW" : "IQ", (double)dbm);
  uint8_t cs = 0; for (int k = 0; k < pl; k++) cs ^= (uint8_t)p[k];
  char ps[112]; snprintf(ps, sizeof ps, "$%s*%02X", p, cs);
  diag_stream().printf("AIS RX%s: %s | %.1f dBm SNR %.1f dB df %+.0f Hz\n%s\n", hw ? "[HW]" : "", d,
                       (double)dbm, (double)snr_db, (double)foff_hz, n);
  for (int i = 0; i < AIS_MAX_CLI; i++)
    if (s_cli[i] && s_cli[i].connected()) { s_cli[i].print(n); s_cli[i].print("\r\n"); s_cli[i].print(ps); s_cli[i].print("\r\n"); }
}

/* CRC-16 passes ~1/65536 of noise frames; with 2 channels x 5 hypotheses x
 * 5 lanes some get through. Live 2026-11-24: one I/Q-only "station" had a
 * 10-digit MMSI. Drop message types 0 / >27 and MMSI 0 / > 999 999 999. */
static bool plausible(const uint8_t *fr, int len)
{
  if (len < 11) return false;
  unsigned t = fr[0] >> 2;
  uint32_t mmsi = frame_mmsi(fr, len);
  return t >= 1 && t <= 27 && mmsi != 0 && mmsi <= 999999999u;
}

static void on_frame(const uint8_t *fr, int len, char chan, uint32_t t48k, void *ctx)
{
  (void)t48k; (void)ctx;
  if (!plausible(fr, len)) { s_rej++; return; }
  const ais_rx_t *r = s_m->last;
  s_iq_frames++;
  emit(fr, len, chan, r->last_lvl_dbfs, r->last_snr_db, r->last_foff_hz, false,
       r->last_lvl_dbfs + AIS_DBM_K);
}

/* Internal RAM first, while >= 10 KB stays free for lwIP/WiFi (see adsb_esp.cpp):
 * the dual setup with its buffers in PSRAM ran ~10x slower than real time
 * (cache misses, 2026-11-25). */
static uint32_t s_n_int = 0, s_n_ps = 0;
static void *ais_alloc(size_t n)
{
  void *p = NULL;
  if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) > n + 256 &&
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > n + 10000)
    p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (p) s_n_int++;
  else { p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); if (p) s_n_ps++; }
  return p;
}

uint32_t ais_mode_tune_khz(void) { return s_dual ? AIS_DUAL_TUNE_KHZ : AIS_TUNE_KHZ; }
int32_t  ais_mode_decim(void)    { return s_dual ? 4 : 8; }

static void iq_status(void)
{
  Stream &o = diag_stream();
  o.printf("AISIQ [rx %u B, multi %u B] ", (unsigned)sizeof(ais_rx_t), (unsigned)sizeof(ais_multi_t));
  o.printf("AISIQ %s: tune %lu kHz, R=%ld, stream %lu sps, %d hypotheses x %ld Hz, AFC %d, cpu %.1f %%, frames %lu, rejected %lu, mem int %lu / psram %lu",
           s_dual ? "dual A+B" : "single A", (unsigned long)ais_mode_tune_khz(), (long)ais_mode_decim(),
           (unsigned long)s_sps, s_nhyp, (long)s_step, s_nhyp == 1 ? s_afc : 0, (double)s_cpu_pct,
           (unsigned long)s_iq_frames, (unsigned long)s_rej, (unsigned long)s_n_int, (unsigned long)s_n_ps);
  if (s_m) for (int c = 0; c < s_m->nch; c++)
    o.printf(" | %c %+ld Hz: %lu", s_m->ch[c].chan, (long)s_m->ch[c].off_hz, (unsigned long)s_m->ch[c].frames);
  o.println();
}

/* AISIQ = status, AISIQ1 = single A (50 ksps), AISIQ2 = dual A+B (100 ksps),
 * AISIQH<n> = hypotheses 1..5, AISIQS<hz> = hypothesis step,
 * AISIQA<sh> = AFC time constant 2^sh samples (0 = off; only with 1 hypothesis) */
void ais_iq_cmd(const char *a)
{
  while (*a == ' ') a++;
  if (*a == '1' || *a == '2') {
    s_dual = (*a == '2'); s_reinit = true;
    menu_host_set_decim(ais_mode_decim());
    menu_host_tune(ais_mode_tune_khz());
  } else if (*a == 'H') {
    int n = atoi(a + 1); if (n >= 1 && n <= AIS_HYP_MAX) { s_nhyp = n; s_reinit = true; }
  } else if (*a == 'S') {
    long v = atol(a + 1); if (v >= 200 && v <= 4000) { s_step = (int32_t)v; s_reinit = true; }
  } else if (*a == 'A') {
    int v = atoi(a + 1); if (v >= 0 && v <= 12) { s_afc = v; s_reinit = true; }
  }
  iq_status();
}

void ais_mode_init(void)
{
  s_sps = 0;
  if (!s_m) s_m = (ais_multi_t *)ais_alloc(sizeof(ais_multi_t));
  if (s_m) memset(s_m, 0, sizeof *s_m);
  diag_stream().printf("AIS RX: %s, tune %lu kHz, NMEA on TCP %d (%u B in PSRAM)\n",
                       s_dual ? "channels A+B" : "channel A", (unsigned long)ais_mode_tune_khz(), AIS_NMEA_PORT,
                       (unsigned)sizeof(ais_multi_t));
}

void ais_mode_feed(const int16_t *iq, int nsamp, uint32_t sps)
{
  if (sps == 0 || !s_m) return;
  if (sps != s_sps || s_reinit) {
    s_sps = sps; s_reinit = false;
    ais_multi_init(s_m, sps, ais_mode_tune_khz(), s_nhyp, s_step, on_frame, NULL, ais_alloc);
    for (int c = 0; c < s_m->nch; c++)                /* AFC and hypotheses exclude each other */
      for (int k = 0; k < s_m->ch[c].nh; k++) s_m->ch[c].rx[k]->afc_sh = (s_nhyp == 1) ? s_afc : 0;
    diag_stream().printf("AIS RX: %lu sps at %lu kHz -> %d channel(s), DDC %d taps /%d -> %lu sps, %d hypotheses x %ld Hz\n",
                         (unsigned long)sps, (unsigned long)ais_mode_tune_khz(), s_m->nch,
                         s_m->nch ? s_m->ch[0].n : 0, s_m->nch ? s_m->ch[0].D : 0,
                         (unsigned long)s_m->sps_ddc, s_nhyp, (long)s_step);
  }
  int64_t t0 = esp_timer_get_time();
  ais_multi_feed(s_m, iq, nsamp);
  int64_t t1 = esp_timer_get_time();
  s_cpu_us += t1 - t0;
  if (!s_cpu_t0) s_cpu_t0 = t1;
  if (t1 - s_cpu_t0 >= 10000000) {                  /* load of the AIS demod, % of one core, per 10 s */
    s_cpu_pct = 100.0f * (float)s_cpu_us / (float)(t1 - s_cpu_t0);
    s_cpu_us = 0; s_cpu_t0 = t1;
  }
}

/* AIS1 block from the FG23 (main.cpp process(), SPI task context is NOT
 * used: process() runs in the main loop like the I/Q demods). */
void ais_mode_hwblk(const void *blk1040)
{
  const aispkt_blk_t *B = (const aispkt_blk_t *)blk1040;
  s_hw_blocks++;
  s_hw_st = B->st;
  const uint8_t *raw8 = (const uint8_t *)blk1040;
  if (s_hw_dbg) {
    aispkt_dbg_t D;
    memcpy(&D, raw8 + AISPKT_DBG_OFS, sizeof D);
    diag_stream().printf("AISDBG t %lu sync %lu ok %lu crc %lu abort %lu ovf %lu | loops %lu isr %lu "
                         "bytes %lu startrx %u cfg 0x%04x state 0x%x src %u opts %u rearm %u wd %u "
                         "ftick %ld foff %d rssi %.1f\n",
                         (unsigned long)B->st.uptime_ms, (unsigned long)B->st.n_sync,
                         (unsigned long)B->st.n_ok, (unsigned long)B->st.n_crc_bad,
                         (unsigned long)B->st.n_abort, (unsigned long)B->st.n_ovf,
                         (unsigned long)D.loops, (unsigned long)D.isr, (unsigned long)D.bytes_rx,
                         (unsigned)D.startrx_st, (unsigned)D.cfgdata_st, (unsigned)D.radio_state,
                         (unsigned)D.src, (unsigned)D.opts, (unsigned)D.rearm, (unsigned)D.wd,
                         (long)D.ftick, (int)D.foff_raw, D.rssi_qdbm / 4.0);
  }
  {                       /* ADS-B (AM on the FG23): telemetry + MS1 records to the aircraft table */
    aispkt_dbg_t D2;
    memcpy(&D2, raw8 + AISPKT_DBG_OFS, sizeof D2);
    if (D2.opts & 0x80u) adsb_feed_dbg(&D2);          /* FG23 marks AM telemetry with opts bit 7 */
    if ((B->hdr.flags & AISPKT_FLAG_RAW) && raw8[AISPKT_RAW_OFS] == 'M' && raw8[AISPKT_RAW_OFS + 1] == 'S'
        && raw8[AISPKT_RAW_OFS + 2] == '1')
      adsb_feed_ms1(raw8 + AISPKT_RAW_OFS, AISPKT_RAW_LEN);
  }
  if (B->hdr.flags & AISPKT_FLAG_RAW) {
    static const char hx[] = "0123456789abcdef";
    char line[2 * 64 + 1];
    Print &o = diag_stream();
    o.printf("AISRAW begin %u\n", (unsigned)AISPKT_RAW_LEN);
    for (unsigned i = 0; i < AISPKT_RAW_LEN; i += 64) {
      for (unsigned k = 0; k < 64; k++) {
        uint8_t v = raw8[AISPKT_RAW_OFS + i + k];
        line[2 * k] = hx[v >> 4]; line[2 * k + 1] = hx[v & 15];
      }
      line[128] = 0;
      o.printf("AISRAW %s\n", line);
    }
    o.printf("AISRAW end\n");
    return;
  }
  /* RAIL_RSSI_INVALID = -128 dBm * 4; a real FG23 noise floor is ~-120..-100 dBm */
  if (B->st.noise_qdbm > -127 * 4 && B->st.noise_qdbm < 0) s_hw_noise_qdbm = B->st.noise_qdbm;
  uint16_t n = B->hdr.nfrm;
  if (n > AISPKT_MAX_FRAMES) return;
  for (uint16_t i = 0; i < n; i++) {
    const aispkt_frm_t *f = &B->frm[i];
    if (f->len < 11 || f->len > AISPKT_MAX_LEN) continue;
    s_hw_frames++;
    float dbm = f->rssi_qdbm / 4.0f;
    float snr = (s_hw_noise_qdbm != INT16_MIN) ? (f->rssi_qdbm - s_hw_noise_qdbm) / 4.0f : 0.0f;
    float foff = (f->foff_hz == INT16_MIN) ? 0.0f : (float)f->foff_hz;
    /* ais_rx's frame bytes include the FCS; ais_describe/ais_nmea take the
     * same (len incl. FCS) as on_frame gets them */
    emit(f->data, f->len, (char)f->chan, dbm - AIS_DBM_K, snr, foff, true, dbm);
  }
}

void ais_mode_tick(uint32_t now_ms)
{
  (void)now_ms;
  if (!s_srv_on && WiFi.status() == WL_CONNECTED) { s_srv.begin(); s_srv.setNoDelay(true); s_srv_on = true; }
  if (s_srv_on && s_srv.hasClient()) {
    WiFiClient c = s_srv.available();
    int slot = -1;
    for (int i = 0; i < AIS_MAX_CLI; i++) if (!(s_cli[i] && s_cli[i].connected())) { slot = i; break; }
    if (slot < 0) { c.stop(); return; }
    s_cli[slot] = c; s_cli[slot].setNoDelay(true);
  }
}

uint32_t    ais_mode_frames(void)  { return s_iq_frames + s_hw_frames; }
uint32_t    ais_mode_bad(void)     { return s_m ? ais_multi_crc_bad(s_m) : 0; }
uint32_t    ais_mode_last_ms(void) { return s_last_ms; }
const char *ais_mode_last(void)    { return s_last; }
uint32_t    ais_mode_ships(void)   { return s_nmmsi; }
uint32_t    ais_hw_frames(void)    { return s_hw_frames; }
float       ais_hw_noise_dbm(void) { return (s_hw_noise_qdbm != INT16_MIN) ? s_hw_noise_qdbm / 4.0f : NAN; }

/* =====================================================================
 *  AISBENCH — alternate the two receive paths on the same antenna
 *
 *  One radio, so the paths cannot run at the same time; alternating every
 *  few minutes spreads the traffic variation (ships moving, AtoN report
 *  cycles) evenly over both. Per slot: messages/min and stations; overall:
 *  the rate ratio and, per station heard by both, the dBm levels side by
 *  side (does HW lose the weak ones first?). The first BENCH_SETTLE_MS of
 *  every slot is not counted (mode switch, PHY reload, first block).
 * ===================================================================== */
#define BENCH_SETTLE_MS  5000u
#define BENCH_ST_MAX     128

typedef struct { uint32_t mmsi; uint16_t n[2]; float dbm_sum[2]; } bench_st_t;

static bool        s_b_on = false;
static bool        s_b_hw = false;           /* current slot: true = FG23 on-chip */
static uint32_t    s_b_slot_ms = 180000u;
static uint32_t    s_b_t0 = 0;               /* current slot start */
static uint32_t    s_b_slot = 0;
static uint32_t    s_b_msgs_slot = 0;
static uint32_t    s_b_msgs[2] = {0, 0};     /* counted messages per path */
static uint32_t    s_b_ms[2] = {0, 0};       /* counted time per path */
static uint32_t    s_b_slots[2] = {0, 0};
static bench_st_t  s_b_st[BENCH_ST_MAX];
static uint32_t    s_b_nst = 0;
static uint32_t    s_b_slot_st_mask[BENCH_ST_MAX / 32];
static void      (*s_b_send)(const char *) = NULL;

bool ais_bench_hw_slot(void) { return s_b_on && s_b_hw; }

static void bench_note(bool hw, uint32_t mmsi, float dbm)
{
  if (!s_b_on || hw != s_b_hw) return;
  if (millis() - s_b_t0 < BENCH_SETTLE_MS) return;
  int p = hw ? 1 : 0;
  s_b_msgs[p]++; s_b_msgs_slot++;
  uint32_t i;
  for (i = 0; i < s_b_nst; i++) if (s_b_st[i].mmsi == mmsi) break;
  if (i == s_b_nst) {
    if (s_b_nst >= BENCH_ST_MAX) return;
    memset(&s_b_st[i], 0, sizeof s_b_st[i]); s_b_st[i].mmsi = mmsi; s_b_nst++;
  }
  s_b_st[i].n[p]++; s_b_st[i].dbm_sum[p] += dbm;
  s_b_slot_st_mask[i >> 5] |= 1u << (i & 31);
}

static uint32_t popcount_mask(void)
{
  uint32_t c = 0;
  for (uint32_t k = 0; k < BENCH_ST_MAX / 32; k++) c += __builtin_popcount(s_b_slot_st_mask[k]);
  return c;
}

static void bench_report(void)
{
  Stream &o = diag_stream();
  float r0 = s_b_ms[0] ? s_b_msgs[0] * 60000.0f / s_b_ms[0] : 0.0f;
  float r1 = s_b_ms[1] ? s_b_msgs[1] * 60000.0f / s_b_ms[1] : 0.0f;
  uint32_t both = 0, only0 = 0, only1 = 0;
  for (uint32_t i = 0; i < s_b_nst; i++) {
    if (s_b_st[i].n[0] && s_b_st[i].n[1]) both++;
    else if (s_b_st[i].n[0]) only0++;
    else if (s_b_st[i].n[1]) only1++;
  }
  o.printf("AISBENCH total: IQ %lu slots %lu msgs %.2f/min | HW %lu slots %lu msgs %.2f/min | HW/IQ %.2f"
           " | stations both %lu, IQ only %lu, HW only %lu\n",
           (unsigned long)s_b_slots[0], (unsigned long)s_b_msgs[0], (double)r0,
           (unsigned long)s_b_slots[1], (unsigned long)s_b_msgs[1], (double)r1,
           (double)(r0 > 0 ? r1 / r0 : 0.0f), (unsigned long)both, (unsigned long)only0, (unsigned long)only1);
  /* per station, rates normalised by each path's counted time */
  for (uint32_t i = 0; i < s_b_nst; i++) {
    const bench_st_t &s = s_b_st[i];
    float a = s_b_ms[0] ? s.n[0] * 60000.0f / s_b_ms[0] : 0, b = s_b_ms[1] ? s.n[1] * 60000.0f / s_b_ms[1] : 0;
    o.printf("AISBENCH st %09lu  IQ %3u (%.2f/min, %6.1f dBm)  HW %3u (%.2f/min, %6.1f dBm)\n",
             (unsigned long)s.mmsi, (unsigned)s.n[0], (double)a, (double)(s.n[0] ? s.dbm_sum[0] / s.n[0] : 0.0f),
             (unsigned)s.n[1], (double)b, (double)(s.n[1] ? s.dbm_sum[1] / s.n[1] : 0.0f));
  }
  o.printf("AISBENCH HW stats: sync %lu ok %lu crc_bad %lu abort %lu drop %lu ovf %lu noise %.1f dBm, blocks %lu\n",
           (unsigned long)s_hw_st.n_sync, (unsigned long)s_hw_st.n_ok, (unsigned long)s_hw_st.n_crc_bad,
           (unsigned long)s_hw_st.n_abort, (unsigned long)s_hw_st.n_drop, (unsigned long)s_hw_st.n_ovf,
           (double)(s_hw_noise_qdbm != INT16_MIN ? s_hw_noise_qdbm / 4.0f : 0.0f), (unsigned long)s_hw_blocks);
}

static void bench_enter(bool hw, uint32_t now)
{
  s_b_hw = hw; s_b_t0 = now; s_b_msgs_slot = 0; s_b_slot++;
  memset(s_b_slot_st_mask, 0, sizeof s_b_slot_st_mask);
  if (s_b_send) s_b_send(hw ? "A1" : "A0");
  /* 'A0' leaves the I/Q stream mistuned (FG23 ais_off_restore): retune like
   * AISGO does. Measured 2026-11-23 on ID1: without it the IQ slots got
   * 0.41 msgs/min against ~2.6/min before the bench. */
  if (!hw && s_b_send) {
    char f[16];
    snprintf(f, sizeof f, "F%lu", (unsigned long)ais_mode_tune_khz());
    s_b_send(f);
  }
  diag_stream().printf("AISBENCH slot %lu: %s for %lu s\n", (unsigned long)s_b_slot,
                       hw ? "HW (FG23 on-chip)" : "IQ (ESP demod)", (unsigned long)(s_b_slot_ms / 1000u));
}

static void bench_close(uint32_t now)
{
  uint32_t dt = now - s_b_t0;
  uint32_t counted = dt > BENCH_SETTLE_MS ? dt - BENCH_SETTLE_MS : 0;
  int p = s_b_hw ? 1 : 0;
  s_b_ms[p] += counted; s_b_slots[p]++;
  diag_stream().printf("AISBENCH slot %lu %s %lu s: %lu msgs (%.2f/min), %lu stations\n",
                       (unsigned long)s_b_slot, s_b_hw ? "HW" : "IQ", (unsigned long)(counted / 1000u),
                       (unsigned long)s_b_msgs_slot,
                       (double)(counted ? s_b_msgs_slot * 60000.0f / counted : 0.0f),
                       (unsigned long)popcount_mask());
}

void ais_bench_cmd(const char *arg, void (*send)(const char *))
{
  while (*arg == ' ') arg++;
  if (send) s_b_send = send;
  uint32_t now = millis();
  if (!*arg) {                                     /* report */
    if (!s_b_on && !s_b_slots[0] && !s_b_slots[1]) {
      diag_stream().println("AISBENCH <min> = start (IQ first), AISBENCH 0 = stop, AISBENCH = report");
      return;
    }
    bench_report(); return;
  }
  long m = atol(arg);
  if (m <= 0) {
    if (s_b_on) { bench_close(now); s_b_on = false; if (s_b_send) s_b_send("A0"); }
    diag_stream().println("AISBENCH stopped, back to the I/Q path");
    bench_report();
    return;
  }
  if (m > 60) m = 60;
  s_b_slot_ms = (uint32_t)m * 60000u;
  memset(s_b_msgs, 0, sizeof s_b_msgs); memset(s_b_ms, 0, sizeof s_b_ms);
  memset(s_b_slots, 0, sizeof s_b_slots); s_b_nst = 0; s_b_slot = 0;
  s_b_on = true;
  bench_enter(false, now);
}

void ais_bench_tick(uint32_t now)
{
  if (!s_b_on) return;
  if (now - s_b_t0 < s_b_slot_ms) return;
  bench_close(now);
  bench_enter(!s_b_hw, now);
  if (!s_b_hw) bench_report();                      /* after every IQ+HW pair */
}

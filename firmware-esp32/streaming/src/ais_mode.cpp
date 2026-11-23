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
#include <Arduino.h>
#include <WiFi.h>
#include <string.h>

Stream &diag_stream(void);

#define AIS_MAX_CLI   3
#define AIS_MMSI_MAX  64

static ais_rx_t   s_a;
static uint32_t   s_sps = 0;
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
static void emit(const uint8_t *fr, int len, char chan, float lvl_dbfs, float snr_db,
                 float foff_hz, bool hw, float dbm)
{
  char d[96], n[320];
  ais_describe(fr, len, d, sizeof d);
  ais_nmea(fr, len, chan, n, sizeof n);
  snprintf(s_last, sizeof s_last, "%.47s", d);
  s_last_ms = millis();
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

static void on_frame(const uint8_t *fr, int len, char chan, uint32_t t48k, void *ctx)
{
  (void)t48k; (void)ctx;
  emit(fr, len, chan, s_a.last_lvl_dbfs, s_a.last_snr_db, s_a.last_foff_hz, false,
       s_a.last_lvl_dbfs + AIS_DBM_K);
}

void ais_mode_init(void)
{
  s_sps = 0;
  diag_stream().printf("AIS RX: channel A 161.975 MHz (tune %lu kHz), NMEA on TCP %d\n",
                       (unsigned long)AIS_TUNE_KHZ, AIS_NMEA_PORT);
}

void ais_mode_feed(const int16_t *iq, int nsamp, uint32_t sps)
{
  if (sps == 0) return;
  if (sps != s_sps) {
    s_sps = sps;
    ais_rx_init(&s_a, sps, (int32_t)(161975 - (int32_t)AIS_TUNE_KHZ) * 1000, 'A', on_frame, NULL);
    diag_stream().printf("AIS RX: %lu sps, channel FIR %d taps, A at %+ld Hz\n",
                         (unsigned long)sps, s_a.ch_n, (long)(161975 - (long)AIS_TUNE_KHZ) * 1000);
  }
  ais_rx_feed(&s_a, iq, nsamp);
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

uint32_t    ais_mode_frames(void)  { return s_a.frames + s_hw_frames; }
uint32_t    ais_mode_bad(void)     { return s_a.crc_bad; }
uint32_t    ais_mode_last_ms(void) { return s_last_ms; }
const char *ais_mode_last(void)    { return s_last; }
uint32_t    ais_mode_ships(void)   { return s_nmmsi; }

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

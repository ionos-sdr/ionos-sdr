/* radio_sm.cpp - see radio_sm.h. MIT, HA7DCD. */
#include "radio_sm.h"
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

Stream &diag_stream(void);   /* main.cpp: serial + TCP 2323 */

#define SWITCH_MS      4000u   /* a command's target must show up within this */
#define SILENT_IQ_MS   1500u   /* I/Q: ~100+ blocks/s, so 1.5 s without = stopped */
#define SILENT_HW_MS   3000u   /* on-chip: heartbeat blocks ~1/s */
#define HIST_N         12

static rs_info_t s = { RS_BOOT, RS_BOOT, RS_BOOT };
static rs_state_t s_obs = RS_BOOT;               /* what the last block said */
typedef struct { uint32_t ms; rs_state_t from, to; char why[40]; } hist_t;
static hist_t   s_hist[HIST_N];
static uint32_t s_nhist = 0;

static const char *NAMES[RS_N] = { "BOOT", "IQ", "SCAN", "AIS HW", "ADS-B HW", "SWITCH", "SILENT" };
const char *rs_name(rs_state_t st) { return (st < RS_N) ? NAMES[st] : "?"; }

uint16_t rs_color(rs_state_t st)
{
  switch (st) {
    case RS_IQ:      return 0x07E0;   /* green */
    case RS_SCAN:    return 0x07FF;   /* cyan */
    case RS_AIS_HW:  return 0x5DDF;   /* light blue */
    case RS_ADSB_HW: return 0xFD20;   /* orange */
    case RS_SWITCH:  return 0xFFE0;   /* yellow */
    case RS_SILENT:  return 0xF800;   /* red */
    default:         return 0x7BEF;   /* grey */
  }
}

static void go(rs_state_t to, const char *why)
{
  uint32_t now = millis();
  if (to == s.st && to != RS_SWITCH) return;
  hist_t *h = &s_hist[s_nhist++ % HIST_N];
  h->ms = now; h->from = s.st; h->to = to;
  snprintf(h->why, sizeof h->why, "%s", why);
  s.prev = s.st; s.st = to; s.since_ms = now; s.seq++;
  snprintf(s.why, sizeof s.why, "%s", why);
  if (to == RS_SWITCH)
    diag_stream().printf("RSM: %s -> SWITCH (target %s) : %s\n", rs_name(s.prev), rs_name(s.target), why);
  else
    diag_stream().printf("RSM: %s -> %s : %s\n", rs_name(s.prev), rs_name(to), why);
}

static void want(rs_state_t target, const char *why)
{
  s.target = target;
  go(RS_SWITCH, why);
}

/* ------------------------------------------------------------ commands */
void rs_cmd(const char *l)
{
  if (!l || !l[0]) return;
  snprintf(s.last_cmd, sizeof s.last_cmd, "%s", l);
  char why[40];
  snprintf(why, sizeof why, "cmd %.30s", l);
  switch (l[0]) {
    case 'F':                                          /* tuning: the I/Q path's centre */
      if (l[1] >= '0' && l[1] <= '9') s.tune_khz = (uint32_t)strtoul(l + 1, NULL, 10);
      break;
    case 'i':
      if (!l[1]) {                                     /* 'i' alone: stop (part of a restart) */
        if (s.st == RS_IQ) want(RS_IQ, why);
      } else if (l[1] >= '1' && l[1] <= '9') {
        want(RS_IQ, why);
      }
      break;
    case 'W':
      if (l[1] == '0' && !l[2]) want(RS_IQ, why);
      else if (l[1] >= '1' && l[1] <= '9') want(RS_SCAN, why);
      break;
    case 'A':
      if ((l[1] == '1' || l[1] == '2') && !l[2]) {
        want((s.ac_ch == 4000u || s.am_thr > 0) ? RS_ADSB_HW : RS_AIS_HW, why);
      } else if (l[1] == '0' && !l[2]) {
        want(RS_IQ, why);
      } else if (l[1] == 'C') {
        s.ac_ch = (uint32_t)strtoul(l + 2, NULL, 10);
      } else if (l[1] == 'M') {
        s.am_thr = atoi(l + 2);
        /* AM comes AFTER A1 in the ADS-B sequence: retarget a pending switch */
        if (s.st == RS_SWITCH && s.target == RS_AIS_HW && s.am_thr > 0) s.target = RS_ADSB_HW;
      }
      break;
    default:
      break;
  }
}

/* ------------------------------------------------------------ blocks */
static void evidence(rs_state_t obs, const char *what)
{
  uint32_t now = millis();
  s.last_blk_ms = now;
  s_obs = obs;
  if (s.st == obs) return;
  if (s.st == RS_SWITCH) {
    if (obs == s.target) { char w[40]; snprintf(w, sizeof w, "FG23 confirmed (%s)", what); go(obs, w); }
    /* other blocks inside the window: the old mode's tail - wait (rs_tick decides) */
    return;
  }
  char w[40]; snprintf(w, sizeof w, "FG23 data: %s", what);
  go(obs, w);                                          /* the FG23 changed by itself: follow it */
}

void rs_blk_iq(uint32_t decim, uint32_t fs_in, int rf_dbm)
{
  s.iq_decim = decim; s.iq_fs_in = fs_in; s.iq_sps = decim ? fs_in / decim : 0; s.rf_dbm = rf_dbm;
  evidence(RS_IQ, "IQB2");
}
void rs_blk_scan(uint32_t cf_hz, uint32_t span_hz)
{
  (void)span_hz; s.hw_f_hz = cf_hz;
  evidence(RS_SCAN, "SPC1");
}
void rs_blk_hw(uint32_t f_center_hz, bool adsb, int noise_dbm)
{
  if (adsb) {
    s.hw_chan = 0;
    s.hw_f_hz = (s.ac_ch == 4000u) ? 1090000000u : 0u;
    if (s.am_thr <= 0) s.am_thr = -1;                  /* running, threshold set elsewhere */
  } else {
    s.hw_f_hz = f_center_hz;
    s.hw_chan = (f_center_hz == 162025000u) ? 'B' : 'A';
    if (noise_dbm > -128 && noise_dbm < 0) s.rf_dbm = noise_dbm;
  }
  evidence(adsb ? RS_ADSB_HW : RS_AIS_HW, adsb ? "AIS1/AM" : "AIS1");
}

/* ------------------------------------------------------------ time */
void rs_tick(uint32_t now)
{
  uint32_t quiet = s.last_blk_ms ? now - s.last_blk_ms : now;
  if (s.st == RS_SWITCH) {
    if (now - s.since_ms < SWITCH_MS) return;
    if (s.last_blk_ms && now - s.last_blk_ms < SILENT_HW_MS && s_obs != RS_BOOT) {
      char w[40]; snprintf(w, sizeof w, "FG23 stayed, not %s", rs_name(s.target));
      go(s_obs, w);
    } else {
      go(RS_SILENT, "no answer to the command");
    }
    return;
  }
  if (s.st == RS_BOOT || s.st == RS_SILENT) return;
  uint32_t lim = (s.st == RS_IQ) ? SILENT_IQ_MS : SILENT_HW_MS;
  if (s.st == RS_SCAN) lim = SILENT_HW_MS;
  if (quiet > lim) go(RS_SILENT, "no blocks");
}

/* ------------------------------------------------------------ queries */
const rs_info_t *rs_get(void) { return &s; }

bool rs_onchip(void)
{
  if (s.st == RS_AIS_HW || s.st == RS_ADSB_HW) return true;
  return s.st == RS_SWITCH && (s.target == RS_AIS_HW || s.target == RS_ADSB_HW);
}

bool rs_iq_locked(void)
{
  if (rs_onchip()) return true;
  /* SILENT after an on-chip mode (e.g. the FG23 went quiet in A1): still not ours */
  return s.st == RS_SILENT && (s.prev == RS_AIS_HW || s.prev == RS_ADSB_HW);
}

uint32_t rs_freq_khz(void)
{
  rs_state_t st = (s.st == RS_SWITCH || s.st == RS_SILENT) ? s_obs : s.st;
  switch (st) {
    case RS_IQ:      return s.tune_khz;
    case RS_SCAN:
    case RS_AIS_HW:
    case RS_ADSB_HW: return s.hw_f_hz / 1000u;
    default:         return 0;
  }
}

void rs_log(void (*out)(const char *line))
{
  char b[96];
  uint32_t now = millis();
  snprintf(b, sizeof b, "RSM now %s (target %s) for %lu s, %lu kHz, last cmd '%s', why: %s",
           rs_name(s.st), rs_name(s.target), (unsigned long)((now - s.since_ms) / 1000u),
           (unsigned long)rs_freq_khz(), s.last_cmd, s.why);
  out(b);
  uint32_t n = s_nhist < HIST_N ? s_nhist : HIST_N;
  for (uint32_t k = 0; k < n; k++) {
    const hist_t *h = &s_hist[(s_nhist - n + k) % HIST_N];
    snprintf(b, sizeof b, "  -%5lu s  %-8s -> %-8s %s", (unsigned long)((now - h->ms) / 1000u),
             rs_name(h->from), rs_name(h->to), h->why);
    out(b);
  }
}

/* ais_mode.cpp — AIS receive mode: ais_rx.c (the same file the PC harness
 * aisport.c tests) + console output + NMEA over TCP 10110.
 *
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT
 */
#include "ais_mode.h"
#include "ais_rx.h"
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

static WiFiServer s_srv(AIS_NMEA_PORT);
static WiFiClient s_cli[AIS_MAX_CLI];
static bool       s_srv_on = false;

static void note_mmsi(const uint8_t *fr, int len)
{
  if (len < 7) return;
  /* MMSI = message bits 8..37 (frame bytes are already MSB-first, see ais_rx.c) */
  const uint8_t *m = fr;
  uint32_t v = 0;
  for (int k = 8; k < 38; k++) v = (v << 1) | ((m[k >> 3] >> (7 - (k & 7))) & 1);
  for (uint32_t i = 0; i < s_nmmsi; i++) if (s_mmsi[i] == v) return;
  if (s_nmmsi < AIS_MMSI_MAX) s_mmsi[s_nmmsi++] = v;
}

static void on_frame(const uint8_t *fr, int len, char chan, uint32_t t48k, void *ctx)
{
  (void)t48k; (void)ctx;
  char d[96], n[320];
  ais_describe(fr, len, d, sizeof d);
  ais_nmea(fr, len, chan, n, sizeof n);
  snprintf(s_last, sizeof s_last, "%.47s", d);
  s_last_ms = millis();
  note_mmsi(fr, len);
  /* per-packet RF parameters as a proprietary sentence right after the
   * !AIVDM it belongs to: $PIONS,AIS,<chan>,<level dBFS>,<SNR dB>,<foff Hz>
   * (OpenCPN ignores unknown $P sentences; ais_web.py shows them) */
  char p[80];
  int pl = snprintf(p, sizeof p, "PIONS,AIS,%c,%.1f,%.1f,%.0f", chan,
                    (double)s_a.last_lvl_dbfs, (double)s_a.last_snr_db, (double)s_a.last_foff_hz);
  uint8_t cs = 0; for (int k = 0; k < pl; k++) cs ^= (uint8_t)p[k];
  char ps[96]; snprintf(ps, sizeof ps, "$%s*%02X", p, cs);
  diag_stream().printf("AIS RX: %s | %.1f dBFS SNR %.1f dB df %+.0f Hz\n%s\n", d,
                       (double)s_a.last_lvl_dbfs, (double)s_a.last_snr_db, (double)s_a.last_foff_hz, n);
  for (int i = 0; i < AIS_MAX_CLI; i++)
    if (s_cli[i] && s_cli[i].connected()) { s_cli[i].print(n); s_cli[i].print("\r\n"); s_cli[i].print(ps); s_cli[i].print("\r\n"); }
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

uint32_t    ais_mode_frames(void)  { return s_a.frames; }
uint32_t    ais_mode_bad(void)     { return s_a.crc_bad; }
uint32_t    ais_mode_last_ms(void) { return s_last_ms; }
const char *ais_mode_last(void)    { return s_last; }
uint32_t    ais_mode_ships(void)   { return s_nmmsi; }

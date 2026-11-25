/* adsb_esp.h - ADS-B (Mode S) side of the FG23 on-chip decoder: aircraft table
 * and a readsb/tar1090-compatible JSON web server. MIT, HA7DCD.
 *
 * The FG23 decodes Mode S itself (AM<thr>, ais_hw.c) and sends MS1 records in
 * AISRAW blocks; ais_mode_hwblk() hands them to adsb_feed_ms1(), and the block
 * telemetry to adsb_feed_dbg(). HTTP on ADSB_HTTP_PORT:
 *   /data/aircraft.json   readsb/tar1090 format
 *   /data/receiver.json   tar1090 receiver info
 *   /data/stats.json      readsb stats (last1min/last5min/last15min/total) for graphs1090
 *   /                     short text status
 */
#ifndef ADSB_ESP_H
#define ADSB_ESP_H
#include <stdint.h>
#include "aispkt.h"
#include <time.h>

#define ADSB_HTTP_PORT 8080

void adsb_init(void);                                   /* starts the web server task */
void adsb_feed_ms1(const uint8_t *raw, uint32_t len);   /* 'M','S','1',n + n x 20 B */
void adsb_feed_dbg(const aispkt_dbg_t *d);              /* FG23 telemetry of an AM block */
uint32_t adsb_aircraft_count(void);

/* ---- for the TFT's on-chip view (radio_sm: RS_ADSB_HW) ---- */
typedef struct {
  uint32_t icao; char flight[9];
  float    dx_km, dy_km;      /* east / north of the reference, valid if pos_ok */
  int32_t  alt;  bool alt_ok, pos_ok;
  float    trk;  bool trk_ok;
  uint32_t age_ms;            /* since the last message */
  uint8_t  rx;                /* bit0 FG23 heard it in the last minute, bit1 RTL-SDR */
} adsb_ac_view_t;
int adsb_ac_list(adsb_ac_view_t *o, int max);          /* aircraft seen in the last 60 s */
typedef struct {
  uint32_t ms;                /* millis() at reception */
  time_t   t;                 /* epoch s, 0 = no clock */
  uint32_t icao; uint8_t df, tc, src; int8_t lvl;    /* src 0 FG23, 1 RTL; lvl -128 = n/a */
  char     txt[24];           /* decoded parameter: "RYR12AB", "FL350 87km", "GS452 T271" */
} adsb_frm_view_t;
int adsb_recent(adsb_frm_view_t *o, int max);         /* newest first */

#endif

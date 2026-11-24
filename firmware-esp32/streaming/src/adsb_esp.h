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

#define ADSB_HTTP_PORT 8080

void adsb_init(void);                                   /* starts the web server task */
void adsb_feed_ms1(const uint8_t *raw, uint32_t len);   /* 'M','S','1',n + n x 20 B */
void adsb_feed_dbg(const aispkt_dbg_t *d);              /* FG23 telemetry of an AM block */
uint32_t adsb_aircraft_count(void);

#endif

/* ais_mode.h — AIS receive mode for the ionos-mini (wraps ais_rx.c). MIT, HA7DCD. */
#ifndef AIS_MODE_H
#define AIS_MODE_H
#include <stdint.h>

/* Tuning: channel A (87B, 161.975) sits at -10 kHz, away from the DC block,
 * the 1/f noise and the display's DC interpolation. Channel B (162.025) is
 * outside the +-25 kHz band at 50 ksps — single channel for now. */
#define AIS_TUNE_KHZ  161985u
#define AIS_NMEA_PORT 10110          /* OpenCPN: Network, TCP, <ip>:10110 */

void        ais_mode_init(void);
void        ais_mode_feed(const int16_t *iq, int nsamp, uint32_t sps);
void        ais_mode_tick(uint32_t now_ms);
uint32_t    ais_mode_frames(void);
uint32_t    ais_mode_bad(void);
uint32_t    ais_mode_last_ms(void);
const char *ais_mode_last(void);     /* short description of the last message */
uint32_t    ais_mode_ships(void);    /* distinct MMSIs seen */

/* ---- EXPERIMENTAL (branch exp/fg23-ais-onchip): AIS decoded on the FG23 ----
 * The FG23 sends AIS1 blocks (aispkt.h) instead of I/Q while 'A1' runs. The
 * frames go out exactly like the I/Q path's (console, NMEA 10110), the
 * $PIONS sentence gets two more fields: <src IQ|HW>,<level dBm>. */
#define AIS_DBM_K   (-52.5f)         /* dBFS -> dBm, rough (ais_web.py DBM_K) */
void        ais_mode_hwblk(const void *blk1040);
void        ais_hw_dbg_toggle(void);   /* AISDBG: telemetry line per AIS1 block */
/* AISBENCH <min>: alternate I/Q demod (ESP) and on-chip demod (FG23) every
 * <min> minutes; AISBENCH 0 = stop (back to I/Q); AISBENCH = report.
 * send = the cmdlink line sender of main.cpp. */
void        ais_bench_cmd(const char *arg, void (*send)(const char *));
void        ais_bench_tick(uint32_t now_ms);
bool        ais_bench_hw_slot(void); /* true while the FG23 should be in A1 */

#endif

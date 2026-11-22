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

#endif

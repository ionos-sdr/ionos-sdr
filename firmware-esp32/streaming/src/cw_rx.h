/* SPDX-License-Identifier: MIT
 * cw_rx.h — CW dekóder (IQ keverés + envelope + timing)
 */
#ifndef CW_RX_H
#define CW_RX_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void        cw_rx_init(void);
void        cw_rx_feed(const int16_t *iq, int nsamp, uint32_t sps);
void        cw_rx_tick(uint32_t now_ms);

const char *cw_rx_text(void);
uint32_t    cw_rx_chars(void);
uint32_t    cw_rx_last_ms(void);
bool        cw_rx_keyed(void);
bool        cw_rx_signal(void);
float       cw_rx_wpm(void);
float       cw_rx_offset_hz(void);

void        cw_rx_set_tone(float hz);

#ifdef __cplusplus
}
#endif
#endif
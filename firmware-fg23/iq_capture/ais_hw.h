/* SPDX-License-Identifier: MIT
 *
 * ais_hw.h — EXPERIMENTAL: AIS received by the FG23's own GFSK demodulator
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * The radio demodulates 9600 bd GMSK with an AIS PHY (radio config channel
 * entries AIS_CH_A / AIS_CH_B), the packet handler only finds the sync word
 * (the last 16 training bits + the start flag, as they are on air, both NRZI
 * polarities = sync word 1 and 2) and hands over the raw line bits after it.
 * NRZI, HDLC destuffing and the CRC are done here in software, the finished
 * frames go to the ESP32 over SPI as AIS1 blocks (aispkt.h).
 *
 * Mutually exclusive with the I/Q stream and the scan (same SPI path, ext
 * mode). Commands: A1 = channel A, A2 = channel B, A0 = back to the stream.
 */

#ifndef AIS_HW_H
#define AIS_HW_H

#include <stdbool.h>
#include <stdint.h>
#include "rail.h"

/* MUST match the channel entries in config/rail/radio_settings.radioconf */
#define AIS_CH_A        3000u            /* 161.975 MHz */
#define AIS_CH_B        3001u            /* 162.025 MHz */

void ais_hw_init(RAIL_Handle_t rail, double tick_hz);
/* freq_tick: crystal correction for 162 MHz in synth ticks (RAIL_SetFreqOffset) */
bool ais_hw_start(char chan, int32_t freq_tick);
void ais_hw_stop(void);                 /* radio idle, SPI ext mode off */
bool ais_hw_active(void);
bool ais_hw_on_event(RAIL_Handle_t rail, RAIL_Events_t events);   /* ISR */
void ais_hw_process(void);              /* main loop */
void ais_hw_option(const char *arg);    /* "L" data bit order, "Y" sync bit order, "S<n>" sync bits */
void ais_hw_print_stats(void);

#endif /* AIS_HW_H */

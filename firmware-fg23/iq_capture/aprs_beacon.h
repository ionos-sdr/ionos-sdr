/* SPDX-License-Identifier: MIT
 *
 * aprs_beacon.h — AX.25/APRS direct-FSK beacon engine for FG23
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * The carrier comes from RAIL_SetTxStream(CARRIER_WAVE); the mark/space
 * symbol is stepped with RAIL_SetFreqOffset — thanks to the phase
 * continuity measured in T2 this is clean direct FSK (G3RUH-like), NOT
 * audio AFSK. Direwolf decodes it in -B 1200 mode. DUMMY LOAD / ATTENUATOR
 * ONLY — because of the coarse frequency grid the signal is not yet
 * band-ready; on-air use only after PA + filter and measurement!
 */
#ifndef APRS_BEACON_H
#define APRS_BEACON_H

#include <stdint.h>
#include <stdbool.h>

/* Bit stream of a pre-built AX.25 frame (after NRZI + stuffing), which
 * the main loop clocks out at 1200 baud by switching the offset. */
typedef struct {
  uint8_t  bits[1024];  /* one bit per byte, 0/1 (NRZI-encoded) */
  uint16_t nbits;       /* number of valid bits */
} aprs_frame_t;

/* Build an AX.25 UI frame from an APRS position/status.
 *   src   : own callsign (e.g. "HA7DCD"), max 6 chars + ssid
 *   ssid  : 0..15
 *   via   : digipeater path callsign (e.g. "WIDE1") or NULL if none
 *   vssid : SSID of the via (WIDE1-1 -> "WIDE1", 1)
 *   info  : APRS info field
 * WITHOUT a path the digipeaters decode the frame but do NOT repeat it
 * and do not gate it to APRS-IS — WIDE1-1 is required for the packet
 * to propagate (as in APRSdroid's own beacons).
 * Returns true if the frame fit. */
bool aprs_build_ui(aprs_frame_t *out,
                   const char *src, uint8_t ssid,
                   const char *dst, uint8_t dssid,
                   const char *via, uint8_t vssid,
                   const char *info);

#endif /* APRS_BEACON_H */

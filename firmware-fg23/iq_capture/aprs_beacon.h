/* SPDX-License-Identifier: MIT
 *
 * aprs_beacon.h — AX.25/APRS direkt-FSK bacon motor FG23-ra
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * A vivot RAIL_SetTxStream(CARRIER_WAVE) adja, a mark/space szimbolumot
 * RAIL_SetFreqOffset lepteti — a T2-ben mert fazisfolytonossag miatt ez
 * tiszta, direkt-FSK (G3RUH-szeru), NEM audio-AFSK. A Direwolf -B 1200
 * modja dekodolja. CSAK DUMMY LOAD / CSILLAPITAS — a durva rács miatt a
 * jel meg nem sav-kesz, elesbe csak PA+szuro es meres utan!
 */
#ifndef APRS_BEACON_H
#define APRS_BEACON_H

#include <stdint.h>
#include <stdbool.h>

/* Egy elore-osszeallitott AX.25 keret bitfolyama (NRZI+stuffing utan),
 * amit a fo ciklus 1200 baudon leptet ki offszet-valtassal. */
typedef struct {
  uint8_t  bits[1024];  /* egy-bit-per-bajt, 0/1 (NRZI-kodolt) */
  uint16_t nbits;       /* ervenyes bitek szama */
} aprs_frame_t;

/* AX.25 UI keret osszeallitasa APRS-poziciobol/statuszbol.
 *   src   : sajat hivojel (pl. "HA7DCD"), max 6 kar + ssid
 *   ssid  : 0..15
 *   via   : digipeater path hivojel (pl. "WIDE1") vagy NULL, ha nincs
 *   vssid : a via SSID-je (WIDE1-1 -> "WIDE1", 1)
 *   info  : APRS info-mezo
 * A path NELKUL a digipeaterek dekodoljak ugyan a keretet, de NEM
 * ismetlik es nem gate-elik az APRS-IS fele — a WIDE1-1 kell ahhoz,
 * hogy a csomag tovabbmenjen (mint az APRSdroid sajat baconjeiben).
 * Visszaad: true, ha elfert. */
bool aprs_build_ui(aprs_frame_t *out,
                   const char *src, uint8_t ssid,
                   const char *dst, uint8_t dssid,
                   const char *via, uint8_t vssid,
                   const char *info);

#endif /* APRS_BEACON_H */

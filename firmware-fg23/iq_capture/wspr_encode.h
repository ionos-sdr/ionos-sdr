/* SPDX-License-Identifier: MIT
 * wspr_encode.h — WSPR uzenet-kodolo (HA7DCD + Cimbi, MIT) */
#ifndef WSPR_ENCODE_H
#define WSPR_ENCODE_H
#include <stdint.h>
#include <stdbool.h>
/* Szabvanyos WSPR type-1 uzenet kodolasa 162 csatorna-szimbolumma
 * (0..3). callsign: pl. "HA7DCD"; locator: 4 karakter, pl. "JN97";
 * dbm: 0..60. Visszaad: true, ha ervenyes volt a bemenet. */
bool wspr_encode(const char *callsign, const char *locator,
                 int dbm, uint8_t sym[162]);
#endif

/* SPDX-License-Identifier: MIT
 * wspr_encode.h — WSPR message encoder (HA7DCD, MIT) */
#ifndef WSPR_ENCODE_H
#define WSPR_ENCODE_H
#include <stdint.h>
#include <stdbool.h>
/* Encode a standard WSPR type-1 message into 162 channel symbols
 * (0..3). callsign: e.g. "HA7DCD"; locator: 4 characters, e.g. "JN97";
 * dbm: 0..60. Returns true if the input was valid. */
bool wspr_encode(const char *callsign, const char *locator,
                 int dbm, uint8_t sym[162]);
#endif

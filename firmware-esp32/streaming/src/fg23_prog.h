/* SPDX-License-Identifier: MIT
 *
 * fg23_prog.h - program the FG23 radio-board flash from the ESP32-S3 over SWD
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Only on boards that wire SWD to the ESP32 (BOARD_IONOS_MINI: SWCLK 39,
 * SWDIO 38, RESETn 40). Host side: tools/fg23_flash.py --uart COMx.
 */
#pragma once
#include <Arduino.h>

/* Console hook. Returns true if the line was an FG23 programming command:
 *   FGINFO                         -> SWD/DEVINFO report
 *   FGFW <addr> <len> <crc32>      -> receive <len> bytes in 2 kB chunks
 *                                    (host waits for '#' after each), then
 *                                    halt-on-reset, erase, program, verify,
 *                                    run. Ends with "DONE OK" or "ERR ...". */
bool fg23_prog_cmd(const char *line, HardwareSerial &io);

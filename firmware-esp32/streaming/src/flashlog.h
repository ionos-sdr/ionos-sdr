/* SPDX-License-Identifier: MIT
 *
 * flashlog.h — persistent (LittleFS) telemetry log for field debugging
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * PURPOSE
 * =======
 * Collects APRS events and periodic health snapshots into flash even
 * WITHOUT USB (running from a power bank), so that intermittent faults not
 * visible on the momentary OLED/serial output are captured. Read out later
 * over USB.
 *
 * IMPORTANT (ESP32 flash cache): while flash is written/erased the cache is
 * disabled, and the non-IRAM main loop may stall for tens of ms on BOTH
 * cores -> this can be a source of block loss. Therefore:
 *   - the log buffers in RAM (flashlog_printf is CHEAP, it only copies),
 *   - flash is written only RARELY and in batches (flashlog_tick),
 *   - the flush duration is itself logged ("FLUSH Nms"), so the stall is visible.
 *
 * MEASURED 2026-08-04 (VSG60A bench): block loss jumps immediately after
 * every FLUSH, and dt_max rises from 23 ms to 35..140 ms. FOR PRECISE
 * MEASUREMENTS DISABLE IT -> FLASHLOG_ENABLE 0.
 *
 * SERIAL COMMANDS (on the DIAG=Serial0 console, newline-terminated):
 *   logdump   — print the whole log
 *   logclear  — clear the log
 *   logstat   — size + dropped lines
 */

#ifndef FLASHLOG_H
#define FLASHLOG_H

/* ====================================================================
 *  SINGLE SWITCH
 *
 *    1 = normal operation (field telemetry to flash)
 *    0 = COMPLETELY OFF: the module is not compiled in, and EVERY call
 *        site (flashlog_init / _tick / _printf / _dump / _clear / _size)
 *        is replaced by a no-op macro. No #if scattering in main.cpp or
 *        aprs_rx.cpp is needed — this one line is enough.
 *        Use THIS during measurements/debugging.
 * ==================================================================== */
#define FLASHLOG_ENABLE 0

#include <stdint.h>
#include <stddef.h>

class Stream;

#if FLASHLOG_ENABLE

/* LittleFS mount + boot marker. May be called before or after WiFi/APRS come up. */
void   flashlog_init(void);

/* One timestamped line into the RAM buffer (millis() prefix). Does NOT write flash. */
void   flashlog_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* From the main loop: periodic batched flush + serial command handling. */
void   flashlog_tick(uint32_t now_ms);

/* ====================================================================
 *  FLUSH WINDOW — TIMING of the flash write (2026-08-07)
 *
 *  Writing flash disables the flash cache, which halts flash-resident code
 *  on BOTH cores: the main loop AND the dedicated SPI receive task. During
 *  that time only the already queued DMA transactions drain — with
 *  NQUEUE=24 for up to 123 ms.
 *
 *  According to the overnight measurement (2026-08-06/07, 4.55 h, 16 599
 *  frames) a flush takes 88 ms (median; p90 98, max 167) and occurred every
 *  55.5 seconds: at the ~55 B/s log rate of the measurement the
 *  FLUSH_HI=3072-byte threshold filled up in exactly that time. 70 % of the
 *  lost frames coincided with such a stall, WITHOUT a CRC error — i.e. the
 *  frame never reached the decoder, because there was a gap in the sample
 *  stream.
 *
 *  The fix is not to disable the log but to TIME it: on every iteration the
 *  main loop reports whether a "safe window" is open — i.e. whether the
 *  receive ring has drained. If so, all NQUEUE transactions are queued, the
 *  full 123 ms reserve is available, and the 88 ms write fits. If not, the
 *  flush WAITS.
 *
 *  Against lock-up, flashlog.cpp has deadlines: if the window does not open
 *  for a long time, it writes anyway. The log is never lost.
 *
 *  Callers that do not use it get the old behaviour (always free) —
 *  backward compatible.
 * ==================================================================== */
void   flashlog_flush_window(bool safe_now);

/* Print the whole log (old + current) to a stream (e.g. Serial0). */
void   flashlog_dump(Stream &out);

/* Clear the log (both files) and restart. */
void   flashlog_clear(void);

/* Size in bytes of the log stored in flash so far (without the RAM buffer). */
size_t flashlog_size(void);

#else   /* ---------- FLASHLOG_ENABLE == 0: every call disappears ---------- */

/* Macros (not empty functions), so that the arguments are not evaluated
 * either: in a flashlog_printf("... %lu", expensive_function()) call the
 * expensive_function() does not run. The (void) casts avoid "unused value"
 * warnings. */
#define flashlog_init()       ((void)0)
#define flashlog_printf(...)  ((void)0)
#define flashlog_tick(now)    ((void)(now))
#define flashlog_flush_window(s) ((void)(s))
#define flashlog_dump(out)    ((void)0)
#define flashlog_clear()      ((void)0)
#define flashlog_size()       ((size_t)0)

#endif  /* FLASHLOG_ENABLE */

#endif /* FLASHLOG_H */
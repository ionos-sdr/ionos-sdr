/* rgb_load.h - WS2812 colour cycling paced by CPU load
 *
 * HA7DCD.  The on-board RGB LED drifts slowly around the colour wheel when
 * the system is idle, and cycles faster the more heavily it is loaded.  A
 * glance at the board shows the state - without display space or attention.
 *
 * MEASUREMENT: FreeRTOS idle hook per core.  The idle task increments a
 * counter on every iteration; without free time it does not count.  Load
 * = 1 - counter/max, where max self-calibrates (immediately upward, slow
 * decay downward).  Not a "real" profiler, but sufficient for the LED to
 * indicate pressure - and it does not need
 * CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS.
 *
 * COST: ~30 us / update (handled by RMT), 0.15% CPU at 50 Hz.
 * Own task, priority 1, core 0.
 *
 * USAGE:
 *   setup():  rgb_load_init(48);          // 38 also occurs on clones
 *   command:  if (rgb_load_cmd(line)) return;
 *
 * With an external load measure (SPI queue depth, EAGAIN rate, RXRING watermark):
 *   rgb_load_override(0.75f);   // 0..1 ; -1 = back to automatic
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void  rgb_load_init(int pin);
void  rgb_load_stop(void);
bool  rgb_load_running(void);

/* Current load in 0..1 (maximum of the two cores) */
float rgb_load_get(void);
void  rgb_load_get_cores(float *core0, float *core1);

/* External load source. -1.0f = back to automatic measurement. */
void  rgb_load_override(float load_0_1);

/* ---- commands (true = this module handled the line) --------------------
 *   L                status
 *   L0 / L1          off / on
 *   Ltest            sweeps the colour wheel in 3 s (wiring check)
 *   Lset k=v ...     config:
 *       pin=48       WS2812 pin
 *       fenyero=12   peak brightness 1..255 (12 = very dim)
 *       lassu=90     cycle period in seconds on an idle system
 *       gyors=2      cycle period in seconds at 100% load
 *       gorbe=50     curvature of the mapping in % (50 = square root, 100 = linear)
 *       dither=1     temporal dithering (mandatory at low brightness)
 *       gamma=1      gamma correction
 *       telitettseg=100  saturation
 *       hz=50        update rate
 */
bool rgb_load_cmd(const char *line);

#ifdef __cplusplus
}
#endif
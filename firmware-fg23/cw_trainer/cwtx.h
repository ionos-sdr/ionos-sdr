/* SPDX-License-Identifier: MIT
 *
 * cwtx.h  --  real CW transmit (keyed carrier) for the FG23 CW Trainer
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * The keying primitive is ported from cw_morse.c of the
 * rail_soc_empty_FG23_iq_capture project (proven on air):
 *     RAIL_StartTxStream(handle, ch, RAIL_STREAM_CARRIER_WAVE)   = key down
 *     RAIL_StopTxStream(handle)                                  = key up
 *
 * This module is ONLY the key line. The Morse table and the timing stay in
 * cw.c, so the trainer and the transmitter share one engine: hook cwtx_key()
 * into cw_set_key_cb() and every dit/dah keys the carrier as well.
 *
 * The drill never transmits. Nothing goes on the air unless
 * cwtx_set_enabled(true) was called first.
 */
#ifndef CWTX_H_INCLUDED
#define CWTX_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

/* ---- channel grid ----------------------------------------------------
 * MUST match config/rail/radio_settings.radioconf, otherwise
 * RAIL_StartTxStream() rejects the channel (INVALID_PARAMETER) and the
 * radio silently stays where it was.
 *
 * 432.000 MHz + N * 25 kHz, N = 0..320  ->  432.000 .. 440.000 MHz
 *
 * IARU R1 70 cm band plan, the narrowband end:
 *   ch  0   432.000   \ 432.000-432.025 EME exclusive - DO NOT USE
 *   ch  1   432.025   /
 *   ch  2   432.050   CW centre of activity        <- default here
 *   ch  4   432.100   CW/SSB calling - do not park a beacon on it
 *   ch 16   432.400   start of the beacon segment (432.400-432.490)
 */
#define CWTX_BASE_HZ        432000000u
#define CWTX_SPACING_HZ     25000u
#define CWTX_MAX_CHANNEL    320u
#define CWTX_CHANNEL        2u              /* 432.050 MHz */

/* TX power in deci-dBm (0 = 0 dBm). RAIL_TxPower_t is int16_t. */
#define CWTX_POWER_DDBM     0

/* Crystal correction in synth ticks (RAIL_SetFreqOffset).
 *
 * 2026-08-27, MEASURED on this board, RTL-SDR/TCXO zero-beat at ch2:
 *     547 ticks -> 432.048990 MHz   (nominal 432.050000, i.e. 1010 Hz low)
 *
 * Deriving the step size: the crystal error is frequency-proportional, and
 * the 2 m measurement in rail_soc_empty_FG23_iq_capture pins it at
 * 482 tick * 4.6492 Hz = 2241 Hz @ 144.8 MHz = 15.48 ppm. At 432.05 MHz the
 * uncorrected output is therefore 6688 Hz low; the 547 ticks closed 5678 Hz
 * of that, so one tick is worth ~10.4 Hz here. The remaining 1010 Hz needs
 * ~97 more ticks:
 *     547 + 97 = 644
 *
 * 2026-08-27, SECOND measurement, IQ recording analysed off-line:
 *     644 ticks -> carrier +1335.6 Hz from the 432.048990 recording centre
 *                = 432.050326 MHz, i.e. 326 Hz HIGH
 *
 * Two points now, so the step size is arithmetic, not estimation:
 *     (644 - 547) ticks moved the carrier 1010 + 326 = 1336 Hz
 *     -> 13.77 Hz/tick   (the 10.4 above was the wrong guess)
 * To take out the remaining 326 Hz: 326 / 13.77 = 23.7 ticks
 *     644 - 24 = 620
 *
 * Absolute accuracy is limited by the RTL-SDR TCXO (+-1 ppm = +-430 Hz at
 * 432 MHz) and by the assumption that the recording centre really was
 * 432.048990. The BB60C is the authority.
 * BRD4265B-specific - re-measure on any other board. 0 disables. */
#define CWTX_FREQ_TICK      620

/* ---- envelope shaping: SOLVED, by the PA's own ramp -------------------
 * MEASURED 2026-08-27 with cwtx_ramp_sweep(): the FG23 hardware honours
 * RAIL_TxPowerConfig_t.rampTime well past the 10 us the PA component ships,
 * and it shapes BOTH edges of a carrier stream:
 *
 *   rampTime      10 us   200    500   1000   2000   5000  10000
 *   rise 10-90%   0.080  0.147  0.240  0.427  0.853  2.573  5.000 ms
 *   fall 90-10%   0.093  0.147  0.253  0.467  0.880  2.613  4.987 ms
 *   -60 dBc BW     4349   3918   2655   2289   2499    833    549 Hz
 *
 * At 10000 us the edge is the classic 5 ms and the occupied bandwidth drops
 * 8x. The floor beyond ~5 kHz is the RTL-SDR's noise, not the transmitter.
 *
 * Cost: the ramp delays the carrier coming UP but not going DOWN, so a keyed
 * element comes out ~rampTime/2 short (150 ms commanded -> 144.9 ms measured
 * at 10000 us). cw_set_key_comp_ms() adds it back on the key-down side.
 *
 * This replaces the software power ramp of v0.3/v0.4 entirely - that could
 * never have worked, see the CAL findings below. */
#define CWTX_RAMP_TIME_US   10000u
#define CWTX_KEY_COMP_MS    5u

/* ---- why the software ramp failed: the CAL staircase ------------------
 * v0.3 and v0.4 tried to shape the keying edge by moving the PA power under
 * a running carrier stream. The CAL staircase (cwtx_cal_sweep) recorded off
 * air on 2026-08-27 settles why neither worked:
 *
 *   A  1.6 s stream, RAIL_SetTxPowerDbm() every 200 ms -> FLAT, 0.73 dB
 *   B  1.6 s stream, RAIL_SetTxPower() raw every 200 ms -> FLAT, 0.44 dB
 *   C  power set while IDLE, one burst per level        -> STEPS, 15.2 dB
 *
 * So the PA power latches when the stream starts, and nothing moves it
 * afterwards. Setting it while idle works fine.
 *
 * That also explains a bug nobody was looking for: the ramp set the LOWEST
 * level just before StartTxStream, and since it could never rise, every
 * element transmitted at the bottom of the ramp. Measured element level in
 * the v0.4 message run was -19.4 dB against -4.8 dB for a full-power burst
 * in the CAL run - the rig has been running ~15 dB low the whole time.
 *
 * Phase C, requested vs measured (relative to full power):
 *   requested  -28.4 -16.7 -10.2  -6.0  -3.2  -1.4  -0.3   0.0 dB
 *   measured   -15.2 -11.7  -7.6  -6.2  -2.3  -1.0  -0.0   0.0 dB
 * The usable range is ~15 dB, the bottom clamps, and the top two steps are
 * the same. Not enough depth for a clean envelope even if it were fast.
 *
 * A staircase built from Start/Stop per step is possible - the CAL run puts
 * the stream turn-around at ~2 ms - but 8 steps would cost ~16 ms per edge
 * and only buy 15 dB of depth. The real envelope will have to come from an
 * amplitude-shaped OOK/ASK PHY, not from the diagnostic carrier stream.
 *
 * Until then the keying is deliberately hard, and full power is set before
 * every StartTxStream so at least the level is right. The table below stays
 * because cwtx_cal_sweep() uses it. */
#define CWTX_RAMP_STEPS     8

#define CWTX_MYCALL         "HA7DCD"

/* Grab the RAIL handle and set power. Safe to call more than once.
 * Returns false if RAIL is not up yet. */
bool     cwtx_init(void);
bool     cwtx_ready(void);

void     cwtx_set_channel(uint16_t ch);
uint16_t cwtx_get_channel(void);
uint32_t cwtx_freq_hz(void);

/* Arm/disarm the transmitter. cwtx_key() does nothing while disarmed, so
 * the drill can share the same key callback. Disarming also keys up. */
void     cwtx_set_enabled(bool on);
bool     cwtx_is_enabled(void);

/* The key line itself. Call from cw.c's key callback. */
void     cwtx_key(bool down);

/* Non-zero if the last RAIL_StartTxStream() failed (RAIL_Status_t).
 * Cleared by cwtx_set_enabled(true). */
int32_t  cwtx_last_error(void);

/* Key up and put the radio back to idle. */
void     cwtx_idle(void);

/* ---- power-control diagnostic ----------------------------------------
 * v0.3 shipped the ramp above and the recorded IQ showed a brick-wall
 * envelope: 10-90% under 0.2 ms, and a spectrum that is textbook
 * hard-keyed sinc (-60 dB at 5.5 kHz). The ramp LOOP definitely ran - the
 * elements came out 76.3 ms instead of 66.7, exactly the 2 x 5 ms the ramp
 * spends - so the calls happened and the PA ignored them.
 *
 * Rather than guess why, this transmits a diagnostic staircase that answers
 * it from an IQ recording alone, no OLED reading needed. ~6.4 s total:
 *
 *   A  stream started once, then RAIL_SetTxPowerDbm() per level  (8 x 200 ms)
 *   .. 400 ms off
 *   B  stream started once, then RAIL_SetTxPower() raw per level (8 x 200 ms)
 *   .. 400 ms off
 *   C  power set while IDLE, one burst per level      (8 x 200 ms on/100 off)
 *
 * Reading the recording: if A is flat, the dBm API does nothing mid-stream.
 * If B steps, raw power works mid-stream and the ramp should use it. If only
 * C steps, power latches at stream start and the envelope has to be built
 * some other way. C also gives the true dBm-to-amplitude mapping, which is
 * what a correct ramp table needs anyway. */
void     cwtx_cal_sweep(void);

/* ---- ramp-time sweep: the cheap thing to try before building a PHY ----
 * RAIL_TxPowerConfig_t has a rampTime field - "the amount of time to spend
 * ramping for TX in microseconds", uint16_t, so the API allows up to 65 ms.
 * sl_rail_util_pa_config.h ships it at 10 us, which is exactly the 0.09 ms
 * edge we measured. If the FG23 hardware honours larger values, the PA does
 * the envelope for us on every StartTxStream / StopTxStream, and the whole
 * OOK-PHY detour becomes unnecessary.
 *
 * Whether the hardware honours it is an empirical question, so this asks it
 * the same way the power question got asked - on the air. Seven groups, one
 * per ramp time, three 150 ms bursts each, 700 ms of silence between groups:
 *
 *     10, 200, 500, 1000, 2000, 5000, 10000 us
 *
 * Measure the 10-90% edge of each group in the recording. If the edges grow
 * with the setting, we are done. If they all stay at 0.09 ms, the field is
 * clamped somewhere and the envelope has to come from an amplitude-shaped
 * PHY after all. Either way it is one flash and one recording, against a
 * day of PHY work. ~11 s total. */
void     cwtx_ramp_sweep(void);

#endif /* CWTX_H_INCLUDED */

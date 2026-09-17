/* SPDX-License-Identifier: MIT
 *
 * cwtx.c  --  real CW transmit (keyed carrier) for the FG23 CW Trainer
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * Keying is StartTxStream / StopTxStream per element, with a software
 * raised-cosine envelope on the edges (see CWTX_RAMP_US in cwtx.h). The
 * stream starts at the bottom of the ramp and stops at the bottom again, so
 * the gaps really are off - a minimum-power carrier leaking through the
 * spaces would be worse than the clicks we are removing.
 *
 * The RAIL stream start latency is ~1-2 ms, which the ramp and the
 * inter-element gaps absorb at 15+ WPM.
 */
#include "cwtx.h"

#include "rail.h"
#include "sl_rail_util_init.h"

static RAIL_Handle_t s_rail    = NULL;
static uint16_t      s_ch      = CWTX_CHANNEL;
static bool          s_enabled = false;
static bool          s_keyed   = false;
static int32_t       s_err     = 0;

/* ---- raised-cosine envelope -----------------------------------------
 * Amplitude at step k of N is u = (1 - cos(pi*k/N)) / 2, and power in dB is
 * 20*log10(u). Precomputed in deci-dB, relative to CWTX_POWER_DDBM, so the
 * runtime does no floating point:
 *
 *   k/N   1/8     2/8     3/8    4/8    5/8    6/8    7/8   8/8
 *   u     .0381   .1464   .3087  .5000  .6913  .8536  .9619 1.000
 *   dB   -28.4   -16.7   -10.2   -6.0   -3.2   -1.4   -0.3   0.0
 *
 * The bottom step lands below what the sub-GHz PA can actually produce, so
 * RAIL coerces it. That only clips the very quietest part of the ramp,
 * which is the part that matters least. */
static const int16_t RAMP_DDB[CWTX_RAMP_STEPS] = {
  -284, -167, -102, -60, -32, -14, -3, 0
};

/* Raw PA levels for the ramp, resolved once from the dB table. v0.3 drove
 * the ramp with RAIL_SetTxPowerDbm() and the PA did not move; raw levels go
 * straight at the power register with no curve lookup, so they are the more
 * likely to take effect mid-stream. cwtx_cal_sweep() settles which. */
static RAIL_TxPowerLevel_t s_ramp_raw[CWTX_RAMP_STEPS];
static bool                s_ramp_ok = false;

static void apply_ramp_time(uint16_t us);

/* Busy-wait on the RAIL microsecond clock. sl_sleeptimer only does
 * milliseconds, and the ramp steps are ~600 us. */
static void wait_us(uint32_t us)
{
  RAIL_Time_t target = RAIL_GetTime() + (RAIL_Time_t)us;
  while ((int32_t)(RAIL_GetTime() - target) < 0) {
    /* spin */
  }
}

static RAIL_TxPower_t level_dbm(int idx)
{
  return (RAIL_TxPower_t)((int32_t)CWTX_POWER_DDBM + RAMP_DDB[idx]);
}

static void build_ramp_table(void)
{
  RAIL_TxPowerConfig_t pc;
  if (RAIL_GetTxPowerConfig(s_rail, &pc) != RAIL_STATUS_NO_ERROR) {
    s_ramp_ok = false;
    return;
  }
  for (int k = 0; k < CWTX_RAMP_STEPS; k++) {
    s_ramp_raw[k] = RAIL_ConvertDbmToRaw(s_rail, pc.mode, level_dbm(k));
  }
  s_ramp_ok = true;
}

static void set_ramp_level(int idx)
{
  if (s_ramp_ok) {
    RAIL_SetTxPower(s_rail, s_ramp_raw[idx]);
  } else {
    RAIL_SetTxPowerDbm(s_rail, level_dbm(idx));
  }
}

bool cwtx_init(void)
{
  if (s_rail == NULL) {
    s_rail = (RAIL_Handle_t)sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  }
  if (s_rail == NULL) {
    return false;
  }
  RAIL_StopTxStream(s_rail);
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)CWTX_POWER_DDBM);
#if CWTX_FREQ_TICK != 0
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)CWTX_FREQ_TICK);
#endif
  build_ramp_table();
  apply_ramp_time(CWTX_RAMP_TIME_US);   /* the envelope, done in hardware */
  s_err = 0;
  return true;
}

/* Diagnostic staircase - see the comment on cwtx_cal_sweep() in cwtx.h. */
void cwtx_cal_sweep(void)
{
  const uint32_t STEP_US = 200000u;
  if (s_rail == NULL) {
    return;
  }

  /* A: one stream, power moved with the deci-dBm API */
  RAIL_SetTxPowerDbm(s_rail, level_dbm(0));
  if (RAIL_StartTxStream(s_rail, s_ch, RAIL_STREAM_CARRIER_WAVE)
      == RAIL_STATUS_NO_ERROR) {
    for (int k = 0; k < CWTX_RAMP_STEPS; k++) {
      RAIL_SetTxPowerDbm(s_rail, level_dbm(k));
      wait_us(STEP_US);
    }
    RAIL_StopTxStream(s_rail);
  }
  wait_us(400000u);

  /* B: one stream, power moved with the raw API */
  if (s_ramp_ok) {
    RAIL_SetTxPower(s_rail, s_ramp_raw[0]);
    if (RAIL_StartTxStream(s_rail, s_ch, RAIL_STREAM_CARRIER_WAVE)
        == RAIL_STATUS_NO_ERROR) {
      for (int k = 0; k < CWTX_RAMP_STEPS; k++) {
        RAIL_SetTxPower(s_rail, s_ramp_raw[k]);
        wait_us(STEP_US);
      }
      RAIL_StopTxStream(s_rail);
    }
  }
  wait_us(400000u);

  /* C: power set while idle, a separate burst per level. This is the
   * control: if even this is flat, the problem is the power API itself and
   * not the timing of the call. */
  for (int k = 0; k < CWTX_RAMP_STEPS; k++) {
    set_ramp_level(k);
    if (RAIL_StartTxStream(s_rail, s_ch, RAIL_STREAM_CARRIER_WAVE)
        == RAIL_STATUS_NO_ERROR) {
      wait_us(STEP_US);
      RAIL_StopTxStream(s_rail);
    }
    wait_us(100000u);
  }

  RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)CWTX_POWER_DDBM);
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
}

/* ---- ramp-time sweep - see the comment on cwtx_ramp_sweep() in cwtx.h ---- */
static const uint16_t RAMP_TEST_US[] = { 10u, 200u, 500u, 1000u, 2000u, 5000u, 10000u };
#define RAMP_TEST_N (sizeof(RAMP_TEST_US) / sizeof(RAMP_TEST_US[0]))

static void apply_ramp_time(uint16_t us)
{
  RAIL_TxPowerConfig_t pc;
  if (RAIL_GetTxPowerConfig(s_rail, &pc) != RAIL_STATUS_NO_ERROR) {
    return;
  }
  pc.rampTime = us;

  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ConfigTxPower(s_rail, &pc);
  /* ConfigTxPower resets the level, so the power has to be re-stated. */
  RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)CWTX_POWER_DDBM);
#if CWTX_FREQ_TICK != 0
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)CWTX_FREQ_TICK);
#endif

  /* The idle-to-TX transition has to be long enough to contain the ramp,
   * otherwise the ramp is what gets truncated. */
  RAIL_StateTiming_t t;
  t.idleToRx            = RAIL_TRANSITION_TIME_KEEP;
  t.txToRx              = RAIL_TRANSITION_TIME_KEEP;
  t.idleToTx            = (RAIL_TransitionTime_t)us + 200u;
  t.rxToTx              = (RAIL_TransitionTime_t)us + 200u;
  t.rxSearchTimeout     = RAIL_TRANSITION_TIME_KEEP;
  t.txToRxSearchTimeout = RAIL_TRANSITION_TIME_KEEP;
  t.txToTx              = (RAIL_TransitionTime_t)us + 200u;
  RAIL_SetStateTiming(s_rail, &t);
}

void cwtx_ramp_sweep(void)
{
  if (s_rail == NULL) {
    return;
  }
  for (unsigned g = 0; g < RAMP_TEST_N; g++) {
    apply_ramp_time(RAMP_TEST_US[g]);
    for (int b = 0; b < 3; b++) {
      if (RAIL_StartTxStream(s_rail, s_ch, RAIL_STREAM_CARRIER_WAVE)
          == RAIL_STATUS_NO_ERROR) {
        wait_us(150000u);
        RAIL_StopTxStream(s_rail);
      }
      wait_us(150000u);
    }
    wait_us(700000u);          /* group marker */
  }
  apply_ramp_time(CWTX_RAMP_TIME_US);   /* back to the working setting */
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
}

bool cwtx_ready(void)
{
  return (s_rail != NULL);
}

void cwtx_set_channel(uint16_t ch)
{
  if (ch > CWTX_MAX_CHANNEL) {
    ch = CWTX_MAX_CHANNEL;
  }
  s_ch = ch;
}

uint16_t cwtx_get_channel(void)
{
  return s_ch;
}

uint32_t cwtx_freq_hz(void)
{
  return CWTX_BASE_HZ + (uint32_t)s_ch * CWTX_SPACING_HZ;
}

void cwtx_set_enabled(bool on)
{
  if (!on) {
    cwtx_key(false);
    s_enabled = false;
    return;
  }
  if (s_rail == NULL) {
    (void)cwtx_init();
  }
  s_err = 0;
  if (s_rail != NULL) {
    RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
    RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)CWTX_POWER_DDBM);
#if CWTX_FREQ_TICK != 0
    /* Re-apply after Idle: the synth offset does not survive every state
     * change, and a silently dropped correction is how this project lost a
     * year to a receiver that was tuned somewhere else than it claimed. */
    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)CWTX_FREQ_TICK);
#endif
  }
  s_enabled = true;
}

bool cwtx_is_enabled(void)
{
  return s_enabled;
}

int32_t cwtx_last_error(void)
{
  return s_err;
}

void cwtx_key(bool down)
{
  if (s_rail == NULL) {
    return;
  }
  if (down) {
    if (!s_enabled || s_keyed) {
      return;
    }
    /* The power latches here and cannot be moved once the stream runs, so
     * it has to be right BEFORE the start - see the CAL findings in cwtx.h.
     * This one line is what fixes the 15 dB shortfall of v0.3/v0.4. */
    RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)CWTX_POWER_DDBM);

    RAIL_Status_t st = RAIL_StartTxStream(s_rail, s_ch, RAIL_STREAM_CARRIER_WAVE);
    if (st != RAIL_STATUS_NO_ERROR) {
      /* Almost always means the channel is outside the grid configured in
       * radio_settings.radioconf. Never ignore this return value. */
      s_err = (int32_t)st;
      return;
    }
    s_keyed = true;
  } else {
    if (!s_keyed) {
      return;
    }
    RAIL_StopTxStream(s_rail);
    s_keyed = false;
  }
}

void cwtx_idle(void)
{
  if (s_rail == NULL) {
    return;
  }
  cwtx_key(false);
  RAIL_StopTxStream(s_rail);
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
}

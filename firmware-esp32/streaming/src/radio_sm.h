/* radio_sm.h - the FG23 radio's state as the ESP32 sees it. MIT, HA7DCD.
 *
 * The ESP32 programs the FG23 (cmdlink lines), but what the FG23 is ACTUALLY
 * doing is decided by the blocks it sends: IQB2 = I/Q stream, SPC1 = scan
 * line, AIS1 = on-chip decoder (telemetry opts bit 7 = ADS-B / Mode S
 * monitor, else AIS). Commands only open a SWITCH window towards a target;
 * the state is confirmed by the first matching block. If the FG23 does
 * something else (reset, ignored line, another tool drove it) the state
 * follows the data, never the command - the displays and the guards below
 * adapt to the FG23, not the other way round.
 *
 *   BOOT ──first block──▶ IQ | SCAN | AIS_HW | ADSB_HW
 *   any  ──cmd A1/A2/A0/i<R>/W..──▶ SWITCH(target) ──matching block──▶ target
 *   SWITCH ──4 s, other blocks──▶ state of the data ("FG23 stayed")
 *   SWITCH / any ──no block (IQ 1.5 s, on-chip 3 s)──▶ SILENT
 *   SILENT ──any block──▶ state of the data
 *
 * Guard: while the FG23 is (or is being put) on-chip, I/Q-only requests -
 * stream decimation ('i<R>') and tuning ('F<kHz>') - are held back and
 * applied after it is back in IQ (rs_iq_locked()).
 *
 * Diagram: docs/radio-state-machine.md
 */
#ifndef RADIO_SM_H
#define RADIO_SM_H
#include <stdint.h>
#include <stdbool.h>

typedef enum {
  RS_BOOT = 0,   /* nothing received yet */
  RS_IQ,         /* I/Q stream (IQB2): ESP demods, waterfall, rtl_tcp, SpyServer */
  RS_SCAN,       /* scan lines (SPC1) */
  RS_AIS_HW,     /* FG23 decodes AIS itself (A1/A2), AIS1 frames */
  RS_ADSB_HW,    /* FG23 Mode S monitor (AC4000 + A1 + AM<thr>), MS1 records */
  RS_SWITCH,     /* a command went out, waiting for the FG23's blocks to confirm */
  RS_SILENT,     /* no blocks: stream stopped, FG23 hung or in reset */
  RS_N
} rs_state_t;

typedef struct {
  rs_state_t st, target, prev;
  uint32_t   since_ms;        /* entered the current state */
  uint32_t   last_blk_ms;     /* last block of any kind */
  uint32_t   seq;             /* +1 on every state change (display refresh) */
  /* what the FG23 runs, from its blocks */
  uint32_t   iq_decim, iq_fs_in, iq_sps;
  int        rf_dbm;          /* IQ header RSSI, or on-chip noise floor */
  uint32_t   hw_f_hz;         /* on-chip: listening frequency */
  char       hw_chan;         /* 'A' / 'B' (AIS), 0 for ADS-B */
  /* what the ESP asked for (cmdlink) */
  uint32_t   tune_khz;        /* last F<kHz> */
  uint32_t   ac_ch;           /* AC<ch>: 0 = AIS, 4000 = 1090 MHz probe PHY */
  int        am_thr;          /* AM<thr>: Mode S monitor threshold, 0 = off */
  char       last_cmd[24];
  char       why[40];         /* reason of the last transition */
} rs_info_t;

#ifdef __cplusplus
extern "C" {
#endif
void             rs_cmd(const char *line);                       /* every line sent to the FG23 */
void             rs_blk_iq(uint32_t decim, uint32_t fs_in, int rf_dbm);
void             rs_blk_scan(uint32_t cf_hz, uint32_t span_hz);
void             rs_blk_hw(uint32_t f_center_hz, bool adsb, int noise_dbm);
void             rs_tick(uint32_t now_ms);
const rs_info_t *rs_get(void);
const char      *rs_name(rs_state_t s);                          /* "IQ", "ADS-B HW", ... */
bool             rs_onchip(void);       /* AIS_HW / ADSB_HW, or switching towards one */
bool             rs_iq_locked(void);    /* hold back i<R> / F<kHz> now */
uint32_t         rs_freq_khz(void);     /* what the FG23 listens on, 0 = unknown */
uint16_t         rs_color(rs_state_t s);/* RGB565 for the displays */
void             rs_log(void (*out)(const char *line));          /* RSM console: history */
#ifdef __cplusplus
}
#endif
#endif

/* SPDX-License-Identifier: MIT
 *
 * ais_hw.c — EXPERIMENTAL: AIS received by the FG23's own GFSK demodulator
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * See ais_hw.h. Measured on the ESP32 side by alternating with the I/Q path
 * (AISBENCH), docs/fg23-ais-onchip.md.
 *
 * SYNC WORD (measured, not assumed): on 58 CRC-correct frames of the
 * 2026-11-22 powerbank capture the last 24 line bits before the data were
 *     0011 0011 0011 0011 0000 0001   (or its inverse - NRZI polarity)
 * = training "...1010" + flag 01111110 after NRZI. RAIL sends/receives sync
 * words LSB first (rail_types.h, RAIL_SyncWordConfig_t), so as 16-bit values
 * sync word 1 = 0x80CC (line ends in 1), sync word 2 = 0x7F33 (ends in 0);
 * 24 bits: 0x80CCCC / 0x7F3333.
 *
 * DEMODULATOR (measured 2026-11-23, ID2 + VSG60, 10 bursts/s at -100 dBm):
 * the configurator's default for this PHY is TRECS_VITERBI - it gave timing
 * detects on every burst but no preamble detect and (almost) no sync, even
 * with an 8-bit sync word. demod_select = LEGACY in the radioconf: every
 * burst syncs, CRC-correct frames from the first test. The TRECS timing
 * search evidently does not lock on the AIS training sequence (24 bits of
 * 0011 line pattern after NRZI).
 *
 * WHY NOT LET THE PACKET HANDLER DO IT: HDLC bit stuffing makes the length
 * unknowable in advance and the CRC covers destuffed bits. So the PHY runs
 * with a long fixed length, no CRC, no whitening; this file decodes the raw
 * bits as they arrive (RX FIFO threshold event) and re-arms the receiver as
 * soon as the closing flag (or an abort) is seen, so the next slot is not lost.
 */

#include "ais_hw.h"
#include "aispkt.h"
#include "iq_stream.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define AIS_FIXED_LEN     200u     /* raw bytes after sync: > 5-slot msg with stuffing */
#define AIS_FIFO_THRESH   4u       /* bytes -> an event every ~3.3 ms */
#define AIS_QUEUE         8u
#define AIS_HEARTBEAT_MS  2000u    /* bring-up: telemetry every 2 s */
#define AIS_STATS_MS      60000u

static RAIL_Handle_t s_rail;
static double        s_tick_hz = 4.6492;
static volatile bool s_active = false;
static bool s_cap_hold = false;            /* bring-up: AK capture done, no raw drain until A1/A2 */
static bool s_am = false;                  /* bring-up: AM Mode S monitor running */
static char          s_chan = 'A';
static uint16_t      s_ch = AIS_CH_A;
static int32_t       s_ftick = 0;
static uint16_t      s_ch_override = 0;   /* AC<ch>: other PHY channel for bring-up, 0 = AIS */

/* options (runtime, for the bring-up: which bit order the FIFO really has) */
static bool    s_data_lsb_first = true;   /* first air bit = byte bit 0 */
static bool    s_sync_lsb_first = true;   /* RAIL: LSB first (rail_types.h) - confirmed on air once the tuning was right */
/* 16 bits = 8 training bits + flag. The bit-level model (tools/ais_hw_model.py)
 * on the 22 Nov powerbank capture: 16 bits 58/58 frames, 24 bits 56, 32 bits
 * 54 - the start of the training sequence is often damaged. Noise false-sync
 * rate at 16 bits: 9600 * 2^-16 * 2 words ~ 0.3/s, each costing a few ms. */
static uint8_t s_sync_bits = 16;

/* Receive data source. RX_PACKET_DATA = the frame controller's sync word
 * detector starts the capture (gave no syncs at all on 23 Nov on two
 * boards). The others stream the demodulator's output continuously and
 * this file hunts the HDLC flag itself - NRZI makes that polarity-free and
 * the hardware sync detector is out of the loop:
 *   RX_DEMOD_DATA (1)                    "8-bit data output from the demodulator"
 *   RX_DIRECT_SYNCHRONOUS_MODE_DATA (5)  bits at the bit clock (xG23 only)
 * AD<n> selects; AR = dump the next 960 stream bytes to the ESP (AISRAW). */
/* Measured 2026-11-23 (ID2, VSG): sources 1 and 5 both deliver 16-bit I/Q
 * words with this PHY, not bits - no use here. Packet mode it is. */
static RAIL_RxDataSource_t s_src = RX_PACKET_DATA;
static bool s_stream = false;              /* s_src != RX_PACKET_DATA */
static uint8_t  s_rd_buf[960];
static uint16_t s_rd_n = 0;
static volatile uint8_t s_rd_state = 0;    /* 0 idle, 1 filling, 2 ready to send */
static volatile uint32_t s_bytes_rx = 0;
static volatile uint16_t s_n_pre = 0, s_n_tim = 0;   /* preamble / timing detect events */

/* ---- decoder state (ISR) ---- */
typedef struct {
  bool     in;          /* inside a frame (after the opening flag) */
  uint8_t  prev;        /* last line bit (NRZI reference) */
  uint8_t  win;         /* last 8 decoded bits */
  uint8_t  byte, nb;
  uint16_t len;
  uint16_t raw;         /* raw bytes consumed since sync */
  uint8_t  pol;
  uint32_t t_us;
  int16_t  rssi;
  int16_t  foff;
  uint8_t  buf[AISPKT_MAX_LEN];
} dec_t;
static dec_t s_d;

static aispkt_frm_t      s_q[AIS_QUEUE];
static volatile uint32_t s_qw = 0, s_qr = 0;
static volatile bool     s_restart = false;
static aispkt_stats_t    s_st;
static aispkt_blk_t      s_blk __attribute__((aligned(4)));
static uint32_t          s_t0_ms = 0, s_last_blk_ms = 0, s_last_stats_ms = 0;
static uint32_t          s_rx_restarts = 0, s_send_fail = 0, s_blocks = 0;

static uint32_t now_ms(void) { return (uint32_t)(RAIL_GetTime() / 1000u); }

/* CRC-16/X.25 over the frame incl. FCS: residue 0xF0B8 (same as ais_rx.c) */
static bool crc_ok(const uint8_t *p, int n)
{
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < n; i++) {
    crc ^= p[i];
    for (int k = 0; k < 8; k++) crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0x8408) : (uint16_t)(crc >> 1);
  }
  return crc == 0xF0B8;
}

/* air pattern of the last n bits before the data (polarity 1) */
static uint32_t sync_word(uint8_t n, bool inverse, bool lsb_first)
{
  /* air bits, oldest first: ...0011 0011 | 0000 0001 */
  uint32_t v = 0;
  for (int i = 0; i < n; i++) {
    int k = n - 1 - i;                       /* distance from the last bit */
    int bit = (k == 0) ? 1 : (k < 8) ? 0 : (((k - 8) & 3) < 2);    /* ..0011 0011 before the flag */
    if (inverse) bit ^= 1;
    if (lsb_first) v |= (uint32_t)bit << i;
    else           v = (v << 1) | (uint32_t)bit;
  }
  return v;
}

static void dec_begin(uint8_t pol)
{
  memset(&s_d, 0, offsetof(dec_t, buf));
  s_d.in   = true;
  s_d.pol  = pol;
  s_d.prev = (pol == 1) ? 1u : 0u;           /* last sync bit on air */
  s_d.win  = 0x7E;                           /* the opening flag just passed */
  s_d.t_us = (uint32_t)RAIL_GetTime();
  s_d.rssi = RAIL_GetRssi(s_rail, false);
  RAIL_FrequencyOffset_t fo = RAIL_GetRxFreqOffset(s_rail);
  if (fo == RAIL_FREQUENCY_OFFSET_INVALID) s_d.foff = INT16_MIN;
  else s_d.foff = (int16_t)((double)((int32_t)fo - s_ftick) * s_tick_hz);
  s_st.n_sync++;
}

static void dec_end(bool good)
{
  if (good) {
    s_st.n_ok++;
    if (s_qw - s_qr < AIS_QUEUE) {
      aispkt_frm_t *f = &s_q[s_qw % AIS_QUEUE];
      f->t_us = s_d.t_us; f->rssi_qdbm = s_d.rssi; f->foff_hz = s_d.foff;
      f->chan = (uint8_t)s_chan; f->len = (uint8_t)s_d.len; f->pol = s_d.pol;
      f->rsv = 0; f->rsv2 = 0;
      memcpy(f->data, s_d.buf, s_d.len);
      s_qw++;
    } else {
      s_st.n_drop++;
    }
  }
  s_d.in = false;
  if (!s_stream) s_restart = true;            /* re-arm: the next slot may follow at once */
}

/* one line bit */
static inline void dec_bit(uint8_t L)
{
  uint8_t b = (L == s_d.prev) ? 1u : 0u;      /* NRZI: no transition = 1 */
  s_d.prev = L;
  s_d.win = (uint8_t)((s_d.win << 1) | b);
  if (s_d.win == 0x7E) {                      /* flag */
    if (s_d.len >= 11 && crc_ok(s_d.buf, s_d.len)) { dec_end(true); return; }
    if (s_d.len >= 11) { s_st.n_crc_bad++; dec_end(false); return; }
    s_d.len = 0; s_d.byte = 0; s_d.nb = 0;   /* repeated flag: still in front of the data */
    return;
  }
  if ((s_d.win & 0x7F) == 0x7F) { s_st.n_abort++; dec_end(false); return; }   /* 7 ones */
  if ((s_d.win & 0x3F) == 0x3E) return;      /* stuffed zero after five ones */
  s_d.byte = (uint8_t)((s_d.byte >> 1) | (b ? 0x80u : 0u));
  if (++s_d.nb == 8) {
    if (s_d.len >= AISPKT_MAX_LEN) { s_st.n_abort++; dec_end(false); return; }
    s_d.buf[s_d.len++] = s_d.byte;
    s_d.byte = 0; s_d.nb = 0;
  }
}

static void dec_bytes(const uint8_t *p, uint16_t n)
{
  for (uint16_t i = 0; i < n && s_d.in; i++) {
    uint8_t v = p[i];
    for (int k = 0; k < 8 && s_d.in; k++) {
      uint8_t L = s_data_lsb_first ? ((v >> k) & 1u) : ((v >> (7 - k)) & 1u);
      dec_bit(L);
    }
    if (++s_d.raw >= AIS_FIXED_LEN && s_d.in) { s_st.n_abort++; dec_end(false); }
  }
}

/* stream modes: hunt the flag in the continuous demod output */
static uint8_t s_h_prev = 0, s_h_win = 0;
static inline void stream_bit(uint8_t L)
{
  if (s_d.in) {
    dec_bit(L);
    if (s_d.in) return;
    s_h_prev = L;
    s_h_win = s_d.win;
    if (s_h_win == 0x7E) { dec_begin(0); s_d.prev = L; }   /* closing flag = next opening one */
    return;
  }
  uint8_t b = (L == s_h_prev) ? 1u : 0u;
  s_h_prev = L;
  s_h_win = (uint8_t)((s_h_win << 1) | b);
  if (s_h_win == 0x7E) { dec_begin(0); s_d.prev = L; }
}

/* AR in the raw (direct) sources: the dump starts on activity - the first
 * non-zero byte after >= 32 zero bytes - with the 64 bytes before it, so a
 * short burst (a 120 us Mode S frame) lands in the dump instead of being
 * hit by chance. AR2 = start at once (no trigger). */
#define PRE_N 64u
static uint8_t  s_pre[PRE_N];
static uint16_t s_pre_w = 0, s_zero_run = 0;
static uint8_t  s_rd_trig = 1;

static void stream_bytes(const uint8_t *p, uint16_t n)
{
  s_bytes_rx += n;
  for (uint16_t i = 0; i < n; i++) {
    uint8_t v = p[i];
    if (s_rd_state == 1) {
      if (s_rd_n == 0 && s_rd_trig && !(v != 0 && s_zero_run >= 32)) {
        /* waiting for activity */
      } else {
        if (s_rd_n == 0 && s_rd_trig) {
          for (uint16_t k = 0; k < PRE_N; k++) s_rd_buf[s_rd_n++] = s_pre[(s_pre_w + k) % PRE_N];
        }
        s_rd_buf[s_rd_n++] = v;
        if (s_rd_n >= sizeof s_rd_buf) s_rd_state = 2;
      }
    }
    s_pre[s_pre_w] = v; s_pre_w = (uint16_t)((s_pre_w + 1) % PRE_N);
    s_zero_run = v ? 0 : (s_zero_run < 0xFFFF ? s_zero_run + 1 : s_zero_run);
  }
  if (s_src != RX_DIRECT_SYNCHRONOUS_MODE_DATA) return;   /* MB/s of samples: dump only, no AIS decode */
  for (uint16_t i = 0; i < n; i++) {
    uint8_t v = p[i];
    for (int k = 0; k < 8; k++)
      stream_bit(s_data_lsb_first ? ((v >> k) & 1u) : ((v >> (7 - k)) & 1u));
  }
}

/* bring-up: the raw FIFO bytes after the last syncs, read over SWD (FGRD)
 * and decoded offline (bit orders, polarity) - g_ais_raw in the .map */
#define RAW_N 8u
#define RAW_LEN 56u
typedef struct { uint32_t t_us; int16_t rssi; uint8_t pol, n; int16_t foff_raw; int16_t ftick; uint8_t b[RAW_LEN - 4]; } raw_t;   /* 64 B */
volatile uint32_t g_ais_raw_w = 0;
/* bring-up breadcrumbs (FGRD): [0] process() calls, [1] ISR calls,
 * [2] last step in start/process, [3] last RAIL status, [4] ext_send fails */
volatile uint32_t g_ais_bc[8];
raw_t g_ais_raw[RAW_N];
static raw_t *s_raw = NULL;

static void drain_fifo(void)
{
  uint8_t tmp[32];
  uint16_t n;
  /* Bounded: RX_DEMOD_DATA measured 2026-11-23 at ~3.4 MB/s (not 1200 B/s)
   * - an unbounded drain never left the interrupt (process() starved, no
   * blocks to the ESP). Bit-rate sources stay far below the bound. */
  uint32_t budget = 1024u;   /* 16384 for the raw sources hung the main loop (measured) */
  while (budget && (n = RAIL_GetRxFifoBytesAvailable(s_rail)) > 0) {
    if (n > budget) n = (uint16_t)budget;
    if (n > sizeof tmp) n = sizeof tmp;
    n = RAIL_ReadRxFifo(s_rail, tmp, n);
    if (n == 0) break;
    budget -= n;
    if (s_stream) { stream_bytes(tmp, n); continue; }
    s_bytes_rx += n;
    if (s_rd_state == 1 && s_d.in) {         /* AR in packet mode: the bytes after each sync */
      for (uint16_t i = 0; i < n && s_rd_n < sizeof s_rd_buf; i++) s_rd_buf[s_rd_n++] = tmp[i];
      if (s_rd_n >= sizeof s_rd_buf) s_rd_state = 2;
    }
    if (s_raw) {
      for (uint16_t i = 0; i < n && s_raw->n < sizeof s_raw->b; i++) s_raw->b[s_raw->n++] = tmp[i];
      if (s_raw->n >= sizeof s_raw->b) s_raw = NULL;
    }
    if (s_d.in) dec_bytes(tmp, n);           /* after an early end the rest is discarded */
  }
}

bool ais_hw_on_event(RAIL_Handle_t rail, RAIL_Events_t ev)
{
  (void)rail;
  if (!s_active) return false;
  g_ais_bc[1]++;
  if (ev & RAIL_EVENT_RX_PREAMBLE_DETECT) s_n_pre++;
  if (ev & RAIL_EVENT_RX_TIMING_DETECT) s_n_tim++;
  if (ev & RAIL_EVENT_RX_FIFO_OVERFLOW) { s_st.n_ovf++; s_d.in = false; s_restart = true; }
  if (s_stream) {
    /* raw sources run at MB/s: draining in the interrupt starves the main
     * loop (no AIS1 blocks at all, measured) - process() drains instead */
    if (s_src == RX_DIRECT_SYNCHRONOUS_MODE_DATA && (ev & RAIL_EVENT_RX_FIFO_ALMOST_FULL)) drain_fifo();
    return true;
  }
  if (ev & (RAIL_EVENT_RX_SYNC1_DETECT | RAIL_EVENT_RX_SYNC2_DETECT)) {
    /* the FIFO is empty here: rx_arm() resets it, and a frame that ran to the
     * fixed length was drained on RX_PACKET_RECEIVED before this sync */
    if (!s_d.in) {
      dec_begin((ev & RAIL_EVENT_RX_SYNC1_DETECT) ? 1u : 2u);
      s_raw = &g_ais_raw[g_ais_raw_w % RAW_N];
      s_raw->t_us = s_d.t_us; s_raw->rssi = s_d.rssi; s_raw->pol = s_d.pol; s_raw->n = 0;
      s_raw->foff_raw = (int16_t)RAIL_GetRxFreqOffset(s_rail); s_raw->ftick = (int16_t)s_ftick;
      g_ais_raw_w++;
    }
  }
  if (ev & (RAIL_EVENT_RX_FIFO_ALMOST_FULL | RAIL_EVENT_RX_PACKET_RECEIVED
            | RAIL_EVENT_RX_PACKET_ABORTED | RAIL_EVENT_RX_FRAME_ERROR)) {
    drain_fifo();
    if (ev & (RAIL_EVENT_RX_PACKET_RECEIVED | RAIL_EVENT_RX_PACKET_ABORTED | RAIL_EVENT_RX_FRAME_ERROR)) {
      if (s_d.in) { s_st.n_abort++; s_d.in = false; }   /* ran off the fixed length */
    }
  }
  return true;
}

static void apply_sync(void)
{
  RAIL_SyncWordConfig_t sw = {
    .syncWordBits = s_sync_bits,
    .syncWord1 = sync_word(s_sync_bits, false, s_sync_lsb_first),
    .syncWord2 = sync_word(s_sync_bits, true,  s_sync_lsb_first),
  };
  RAIL_Status_t st = RAIL_ConfigSyncWords(s_rail, &sw);
  printf("# AIS HW: sync %u bits %s-first: 0x%08lX / 0x%08lX (st=%d)\r\n",
         (unsigned)sw.syncWordBits, s_sync_lsb_first ? "LSB" : "MSB",
         (unsigned long)sw.syncWord1, (unsigned long)sw.syncWord2, (int)st);
}

static void rx_arm(void)
{
  s_d.in = false;
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(s_rail, false, true);
  s_h_prev = 0; s_h_win = 0;
  /* iq_stream.c: StartRx does NOT preserve the synth offset - set it after */
  g_ais_bc[3] = (uint32_t)RAIL_StartRx(s_rail, s_ch, NULL);
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_ftick);
  g_ais_bc[6]++;
}

void ais_hw_init(RAIL_Handle_t rail, double tick_hz)
{
  s_rail = rail;
  if (tick_hz > 0.0) s_tick_hz = tick_hz;
}

bool ais_hw_active(void) { return s_active; }

bool ais_hw_start(char chan, int32_t freq_tick)
{
  if (s_active) ais_hw_stop();
  s_chan  = (chan == 'B') ? 'B' : 'A';
  s_ch    = (s_chan == 'B') ? AIS_CH_B : AIS_CH_A;
  if (s_ch_override) { s_ch = s_ch_override; freq_tick = 0; }   /* probe PHY: no 162 MHz correction */
  /* The crystal correction comes in the I/Q PHY's synth ticks (4.6492 Hz).
   * The AIS PHY's tick is ~3.9x larger: measured 2026-11-23 with AF<n> -
   * the demod output ran away to all ones at 539 (the I/Q value) and to all
   * zeros at 0, RAIL_GetRxFreqOffset crossed zero at ~137, where whole
   * frames came through. Until the tick size is read from the PHY, scale. */
  s_ftick = (int32_t)((int64_t)freq_tick * 137 / 539);

  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  /* Loads the AIS PHY of that channel entry (multi-PHY radio config). It
   * also re-applies the PHY's data config, so the packet source is set
   * after it, not before. */
  g_ais_bc[2] = 1;
  RAIL_Status_t pst = RAIL_PrepareChannel(s_rail, s_ch);
  g_ais_bc[2] = 2; g_ais_bc[3] = (uint32_t)pst;
  if (pst != RAIL_STATUS_NO_ERROR) {
    printf("# AIS HW: channel %u not in the radio config (st=%d) - generate the AIS PHY first, "
           "docs/fg23-ais-onchip.md\r\n", (unsigned)s_ch, (int)pst);
    return false;
  }
  RAIL_DataConfig_t dc = {
    .txSource = TX_PACKET_DATA,
    .rxSource = s_src,
    .txMethod = PACKET_MODE,
    .rxMethod = FIFO_MODE,
  };
  s_stream = (s_src != RX_PACKET_DATA);
  g_ais_bc[4] = (uint32_t)RAIL_ConfigData(s_rail, &dc);
  if (!s_stream) RAIL_SetFixedLength(s_rail, AIS_FIXED_LEN);
  apply_sync();
  RAIL_StateTransitions_t tr = { RAIL_RF_STATE_RX, RAIL_RF_STATE_RX };
  RAIL_SetRxTransitions(s_rail, &tr);
  RAIL_SetRxFifoThreshold(s_rail, AIS_FIFO_THRESH);
  /* raw sources (demod / direct mode, MB/s): no FIFO interrupt at all - it
   * starved the main loop even without draining in it (measured); process()
   * polls the FIFO, overflows are expected and only counted */
  bool raw_src = s_stream && s_src != RX_DIRECT_SYNCHRONOUS_MODE_DATA;
  RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                    raw_src ? RAIL_EVENTS_NONE :
                    (RAIL_EVENT_RX_SYNC1_DETECT | RAIL_EVENT_RX_SYNC2_DETECT
                    | RAIL_EVENT_RX_PREAMBLE_DETECT | RAIL_EVENT_RX_TIMING_DETECT
                    | RAIL_EVENT_RX_FIFO_ALMOST_FULL | RAIL_EVENT_RX_FIFO_OVERFLOW
                    | RAIL_EVENT_RX_PACKET_RECEIVED | RAIL_EVENT_RX_PACKET_ABORTED
                    | RAIL_EVENT_RX_FRAME_ERROR));

  memset(&s_st, 0, sizeof s_st);
  s_qw = s_qr = 0; s_restart = false;
  s_rx_restarts = 0; s_send_fail = 0; s_blocks = 0; s_n_pre = 0; s_n_tim = 0;
  s_t0_ms = s_last_blk_ms = s_last_stats_ms = now_ms();

  g_ais_bc[2] = 3;
  iq_stream_ext_begin();
  g_ais_bc[2] = 4;
  s_cap_hold = (s_ch_override != 0);   /* bring-up: other PHY (ADS-B 6.5 MB/s) - capture only, no drain */
  s_active = true;
  rx_arm();
  g_ais_bc[2] = 5;
  /* The first StartRx on the AIS channel re-applies the PHY's own sync words
   * over the ones set above (measured: the frames then start 7 bits late).
   * Set them again now that the PHY is loaded, and re-arm. The data config
   * too: whatever reloads the sync words may reload the packet source. */
  g_ais_bc[4] |= (uint32_t)RAIL_ConfigData(s_rail, &dc) << 8;
  if (!s_stream) RAIL_SetFixedLength(s_rail, AIS_FIXED_LEN);
  apply_sync();
  rx_arm();
  s_rd_state = 0; s_rd_n = 0; s_bytes_rx = 0;
  printf("# AIS HW: channel %c (radio ch %u), fixed %u B, data %s-first, offset %ld ticks\r\n",
         s_chan, (unsigned)s_ch, (unsigned)AIS_FIXED_LEN, s_data_lsb_first ? "LSB" : "MSB",
         (long)s_ftick);
  return true;
}

void ais_hw_stop(void)
{
  if (!s_active) return;
  s_active = false;
  s_am = false;
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_SetFixedLength(s_rail, RAIL_SETFIXEDLENGTH_INVALID);
  RAIL_ResetFifo(s_rail, false, true);
  uint32_t t = now_ms();
  while (iq_stream_ext_busy() && now_ms() - t < 20u) iq_stream_ext_pump();
  iq_stream_ext_end();
  ais_hw_print_stats();
  printf("# AIS HW: stopped\r\n");
}

/* ---- burst capture (AK<thr>): the raw sources at their full rate ----
 * Continuous streaming of RX_DEMOD_DATA through the main loop cannot keep up
 * (1.16 MB/s drain-limited, measured). For a feasibility look at short
 * bursts (Mode S: 120 us) the FIFO is read in a tight loop until a soft
 * sample above <thr> arrives, then 512 samples before it and the rest of
 * the 32 KB capture buffer (borrowed from app.c) are kept; the loop also
 * measures the real byte rate. The capture goes to the ESP as AISRAW dumps,
 * 960 B each, the rate in the telemetry's ftick field. */
extern uint8_t *app_capture_buf(uint32_t *len);
static volatile uint8_t s_cap_state = 0;   /* 0 idle, 1 armed, 2 sending */
static int8_t   s_cap_thr = 20;
static uint32_t s_cap_n = 0, s_cap_sent = 0, s_cap_rate = 0, s_cap_total = 0;
#define CAP_PRE 512u
static uint32_t s_cap_len = 0;             /* 0 = whole buffer */

static void rev(uint8_t *p, uint32_t a, uint32_t b)   /* reverse p[a..b) */
{
  while (a + 1 < b) { uint8_t t = p[a]; p[a] = p[b - 1]; p[b - 1] = t; a++; b--; }
}

static void cap_run(void)
{
  /* The whole buffer is a ring the FIFO is read into in big chunks (no
   * per-byte copy); before the trigger only every 2nd sample is compared
   * (a 0.5 us pulse is 3+ samples at 6.5 MS/s), so the scan keeps up with
   * the demod rate and the pre-trigger part stays contiguous with the rest.
   * Afterwards the ring is rotated so the capture starts at buf[0]. */
  uint32_t rl = 0;
  uint8_t *buf = app_capture_buf(&rl);
  if (!buf) { s_cap_state = 0; return; }
  uint32_t len = (s_cap_len && s_cap_len < rl) ? s_cap_len : rl;
  uint32_t w = 0, total = 0, tpos = 0, post = 0;
  bool trig = false;
  const int8_t thr = s_cap_thr;
  RAIL_ResetFifo(s_rail, false, true);
  uint32_t t0 = (uint32_t)RAIL_GetTime(), t1 = t0;
  while (((t1 = (uint32_t)RAIL_GetTime()) - t0) < 500000u) {     /* 0.5 s max */
    uint32_t a = RAIL_GetRxFifoBytesAvailable(s_rail);
    if (!a) continue;
    uint32_t wi = w % rl;
    if (a > rl - wi) a = rl - wi;                 /* up to the ring end */
    if (trig && a > post) a = post;
    a = RAIL_ReadRxFifo(s_rail, buf + wi, (uint16_t)(a > 0xFFFFu ? 0xFFFFu : a));
    total += a;
    if (!trig) {
      const int8_t *q = (const int8_t *)(buf + wi);
      for (uint32_t i = (w & 1u); i < a; i += 2) {
        if (q[i] > thr && w + i >= CAP_PRE) {
          trig = true; tpos = w + i;
          post = len - CAP_PRE;                   /* samples still wanted from tpos */
          uint32_t got = a - i;
          post = (got >= post) ? 0 : post - got;
          break;
        }
      }
    } else {
      post -= a;
    }
    w += a;
    if (trig && post == 0) break;
  }
  s_cap_total = total;
  s_cap_rate = (t1 != t0) ? (uint32_t)((uint64_t)total * 1000000u / (t1 - t0)) : 0;
  s_cap_hold = true;      /* no raw drain afterwards (6.5 MB/s starves the loop) until A1/A2 */
  if (trig) {             /* rotate the ring left so the capture (tpos - PRE) starts at 0 */
    uint32_t st = (tpos - CAP_PRE) % rl;
    rev(buf, 0, st); rev(buf, st, rl); rev(buf, 0, rl);
  }
  s_cap_n = trig ? len : 0; s_cap_sent = 0;
  s_cap_state = trig ? 2 : 0;
  RAIL_ResetFifo(s_rail, false, true);
}

static void cap_feed(void)
{
  if (s_cap_state != 2 || s_rd_state != 0) return;
  uint32_t len = 0;
  uint8_t *buf = app_capture_buf(&len);
  if (!buf || s_cap_sent >= s_cap_n) { s_cap_state = 0; return; }
  uint32_t k = s_cap_n - s_cap_sent; if (k > sizeof s_rd_buf) k = sizeof s_rd_buf;
  memcpy(s_rd_buf, buf + s_cap_sent, k);
  if (k < sizeof s_rd_buf) memset(s_rd_buf + k, 0, sizeof s_rd_buf - k);
  s_cap_sent += k; s_rd_n = (uint16_t)k; s_rd_state = 2;
}

/* ---- AM<thr>: on-chip Mode S monitor (bring-up, ADS-B probe PHY via AC<ch>, AD1) ----
 * RX_DEMOD_DATA arrives at 6.5 MS/s (13 samples per Mode S bit). The FIFO is
 * read in big chunks into a 32 KB ring (the capture buffer) and scanned a word
 * at a time (samples 0 and 2 of each word against <thr>: a 0.5 us pulse is 3+
 * samples). A hit takes the next ~860 samples into a flat window; the preamble
 * (pulses at 0, 1, 3.5, 4.5 us) is searched only where the hit can be one of
 * its pulses, the 112 PPM bits are decided chip-half against chip-half at
 * quarter-sample resolution and checked with CRC-24 (DF17/18 112 bit, DF11
 * 56 bit with IID < 128). Every hit skips ~120 us so one reply costs one
 * decode. Good frames go to the ESP as AISRAW dumps:
 *   'M','S','1',n, then n x { u32 t_us, u8 nbytes, i8 level, u8 data[14] } */
static int8_t   s_am_thr = 4;
static uint32_t s_am_w, s_am_scan, s_am_pend; static bool s_am_haspend;
static uint32_t s_am_ntrig, s_am_npre, s_am_ok, s_am_ovf, s_am_last_ms;
static uint8_t  s_am_out[484]; static uint32_t s_am_on;     /* 24 frames */
#define AM_WIN   880u               /* samples per hit: 60 before + 8 us + 112 bits + slack */
#define AM_PRE   60
#define AM_SKIP  780u               /* 120 us */
static int8_t   s_am_win[AM_WIN];
static const float s_am_rcp[9] = { 1.f, 1.f, 1.f/2, 1.f/3, 1.f/4, 1.f/5, 1.f/6, 1.f/7, 1.f/8 };

static uint32_t am_crc(const uint8_t *b, uint32_t n)       /* CRC-24 0xFFF409, bitwise */
{
  uint32_t c = 0;
  for (uint32_t i = 0; i < n; i++) {
    c ^= (uint32_t)b[i] << 16;
    for (int k = 0; k < 8; k++) c = (c & 0x800000u) ? ((c << 1) ^ 0xFFF409u) : (c << 1);
  }
  return c & 0xFFFFFFu;
}
static inline int32_t am_sum(uint32_t a, uint32_t b)       /* samples [a,b) of the window */
{
  int32_t s = 0;
  if (b > AM_WIN) b = AM_WIN;
  for (uint32_t i = a; i < b; i++) s += s_am_win[i];
  return s;
}
static inline float am_s(uint32_t q0, uint32_t q1)         /* mean over a quarter-sample span (<= 8 samples) */
{
  uint32_t a = (q0 + 2u) >> 2, b = (q1 + 2u) >> 2;
  if (b <= a) b = a + 1u;
  uint32_t n = b - a; if (n > 8u) n = 8u;
  return (float)am_sum(a, a + n) * s_am_rcp[n];
}

static bool am_try(uint32_t q, uint8_t *out, uint32_t *nb)
{
  uint8_t by[14]; memset(by, 0, sizeof by);
  uint32_t base = q + 208u;                                  /* 8 us x 26 quarter-samples */
  for (uint32_t k = 0; k < 112; k++) {
    uint32_t t = base + 26u * k;
    if (((t + 26u + 2u) >> 2) > AM_WIN) return false;
    if (am_s(t, t + 13u) > am_s(t + 13u, t + 26u)) by[k >> 3] |= (uint8_t)(0x80u >> (k & 7u));
    if (k == 4) {         /* early out: only DF11/17/18 are self-checking */
      uint32_t df = by[0] >> 3;
      if (df != 11 && df != 17 && df != 18) return false;
    }
    if (k == 55 && (by[0] >> 3) == 11) {
      uint32_t r = am_crc(by, 4) ^ ((uint32_t)by[4] << 16 | (uint32_t)by[5] << 8 | by[6]);
      if (r == 0u) { memcpy(out, by, 7); *nb = 7; return true; }   /* IID 0 only: <128 gave false hits on noise */
      return false;
    }
  }
  uint32_t r = am_crc(by, 11) ^ ((uint32_t)by[11] << 16 | (uint32_t)by[12] << 8 | by[13]);
  if (r) return false;
  memcpy(out, by, 14); *nb = 14; return true;
}

/* preamble score at sample offset o: pulse sums minus gap sums, pulses 3 samples each */
static int32_t am_pscore(uint32_t o)
{
  int32_t pl = am_sum(o, o + 3) + am_sum(o + 7, o + 10) + am_sum(o + 23, o + 26) + am_sum(o + 29, o + 32);
  int32_t gp = am_sum(o + 3, o + 6) + am_sum(o + 11, o + 14) + am_sum(o + 15, o + 18) + am_sum(o + 33, o + 36);
  return pl - gp;
}

static bool am_decode(const uint8_t *ring, uint32_t M, uint32_t p, uint32_t t_hit)
{
  uint32_t b0 = p - AM_PRE;
  for (uint32_t i = 0; i < AM_WIN; i++) s_am_win[i] = (int8_t)ring[(b0 + i) & M];
  /* the hit (window index AM_PRE) is in pulse 1, 2, 3 or 4: starts 0 / 6.5 / 22.75 / 29.25 samples earlier */
  static const uint8_t back[4] = { 0, 7, 23, 29 };
  int32_t best = -100000, sec = -100000; uint32_t ob = 0, os = 0;
  for (int k = 0; k < 4; k++)
    for (int d = -4; d <= 2; d++) {
      int32_t o = AM_PRE - back[k] + d;
      if (o < 0) continue;
      int32_t sc = am_pscore((uint32_t)o);
      if (sc > best) { sec = best; os = ob; best = sc; ob = (uint32_t)o; }
      else if (sc > sec && (uint32_t)o != ob) { sec = sc; os = (uint32_t)o; }
    }
  if (best < 60) return false;                               /* ~5 per sample over 12; -60 dBm gives ~120 */
  s_am_npre++;
  static const int8_t dq[5] = { 0, -1, 1, -2, 2 };
  uint8_t fr[14]; uint32_t nb = 0;
  for (int c = 0; c < 2; c++) {
    uint32_t q0 = 4u * (c ? os : ob);
    for (int j = 0; j < 5; j++) {
      int32_t q = (int32_t)q0 + dq[j];
      if (q < 0) continue;
      if (am_try((uint32_t)q, fr, &nb)) {
        s_am_ok++;
        if (s_am_on == 0) { s_am_out[0] = 'M'; s_am_out[1] = 'S'; s_am_out[2] = '1'; s_am_out[3] = 0; s_am_on = 4; }
        if (s_am_on + 20u <= sizeof s_am_out) {
          uint8_t *r = s_am_out + s_am_on;
          memcpy(r, &t_hit, 4); r[4] = (uint8_t)nb;
          float lv = am_s((uint32_t)q, (uint32_t)q + 13);
          r[5] = (uint8_t)(int8_t)(lv > 127 ? 127 : (lv < -128 ? -128 : lv));
          memset(r + 6, 0, 14); memcpy(r + 6, fr, nb);
          s_am_on += 20u; s_am_out[3]++;
        }
        return true;
      }
    }
  }
  return false;
}

static void am_start(int8_t thr)
{
  s_am_thr = thr; s_am_w = s_am_scan = 0; s_am_haspend = false;
  s_am_ntrig = s_am_npre = s_am_ok = s_am_ovf = 0; s_am_on = 0; s_am_last_ms = now_ms();
  s_cap_hold = true;
  RAIL_ResetFifo(s_rail, false, true);
  s_am = true;
}

static void am_run(void)
{
  uint32_t rl = 0;
  uint8_t *ring = app_capture_buf(&rl);
  if (!ring || (rl & (rl - 1u))) { s_am = false; return; }
  const uint32_t M = rl - 1u;
  const int8_t thr = s_am_thr;
  uint32_t t0 = (uint32_t)RAIL_GetTime();
  while (((uint32_t)RAIL_GetTime() - t0) < 20000u) {        /* 20 ms slices */
    uint32_t a = RAIL_GetRxFifoBytesAvailable(s_rail);
    if (a >= 4000u) s_am_ovf++;
    if (a) {
      uint32_t wi = s_am_w & M;
      if (a > rl - wi) a = rl - wi;
      a = RAIL_ReadRxFifo(s_rail, ring + wi, (uint16_t)a);
      s_am_w += a;
    }
    if (s_am_w - s_am_scan > rl - 2048u) { s_am_scan = (s_am_w - 1024u) & ~3u; s_am_haspend = false; }
    if (s_am_haspend) {
      if (s_am_w < s_am_pend - AM_PRE + AM_WIN) continue;
      am_decode(ring, M, s_am_pend, (uint32_t)RAIL_GetTime());
      s_am_scan = (s_am_pend + AM_SKIP) & ~3u;
      s_am_haspend = false;
    }
    /* word scan: samples 0 and 2 of each aligned word */
    uint32_t end = s_am_w & ~3u;
    while (s_am_scan < end) {
      uint32_t v = *(const uint32_t *)(ring + (s_am_scan & M));
      if ((int8_t)v > thr || (int8_t)(v >> 16) > thr) {
        if (s_am_scan >= (uint32_t)AM_PRE) {
          s_am_pend = s_am_scan + (((int8_t)v > thr) ? 0u : 2u);
          s_am_haspend = true; s_am_ntrig++;
          break;
        }
      }
      s_am_scan += 4u;
    }
  }
}

static void am_flush(uint32_t now)
{
  if (s_rd_state != 0) return;
  if (s_am_on == 0 || (s_am_out[3] < 20 && now - s_am_last_ms < 500u)) return;
  memset(s_rd_buf, 0, sizeof s_rd_buf);
  memcpy(s_rd_buf, s_am_out, s_am_on);
  s_rd_n = (uint16_t)s_am_on; s_rd_state = 2;
  s_am_on = 0; s_am_last_ms = now;
}

static void send_block(uint32_t now)
{
  uint32_t n = s_qw - s_qr;
  if (n > AISPKT_MAX_FRAMES) n = AISPKT_MAX_FRAMES;
  bool rawdump = (s_rd_state == 2);
  if (rawdump) n = 0;
  if (n == 0 && !rawdump && now - s_last_blk_ms < AIS_HEARTBEAT_MS) return;
  memset(&s_blk, 0, sizeof s_blk);
  s_blk.hdr.magic = AISPKT_MAGIC;
  s_blk.hdr.nfrm = (uint16_t)n;
  s_blk.hdr.flags = AISPKT_FLAG_LAST | (rawdump ? AISPKT_FLAG_RAW : 0u);
  if (rawdump) memcpy((uint8_t *)&s_blk.frm[0], s_rd_buf, sizeof s_rd_buf);   /* 960 B */
  {
    /* bring-up telemetry in the last 32 B of the pad (ESP: AISDBG) */
    aispkt_dbg_t *D = (aispkt_dbg_t *)((uint8_t *)&s_blk + AISPKT_DBG_OFS);
    D->loops = g_ais_bc[0]; D->isr = g_ais_bc[1]; D->bytes_rx = s_bytes_rx;
    D->startrx_st = (uint16_t)g_ais_bc[3]; D->cfgdata_st = s_n_tim;   /* bring-up: timing detects */
    D->radio_state = (uint16_t)RAIL_GetRadioState(s_rail); D->src = (uint8_t)s_src;
    D->opts = (uint8_t)((s_data_lsb_first ? 1u : 0u) | (s_sync_lsb_first ? 2u : 0u));
    D->rearm = (uint16_t)g_ais_bc[6]; D->wd = s_n_pre;               /* bring-up: preamble detects */
    D->ftick = s_cap_rate ? (int32_t)s_cap_rate : s_ftick;   /* bring-up: AK capture rate, B/s */
    if (s_am) {           /* AM monitor: triggers / preambles / CRC-ok / FIFO near-full */
      D->ftick = (int32_t)s_am_ntrig; D->cfgdata_st = (uint16_t)s_am_npre;
      D->wd = (uint16_t)s_am_ok; D->rearm = (uint16_t)s_am_ovf; D->bytes_rx = s_am_w;
    }
    RAIL_FrequencyOffset_t fo = RAIL_GetRxFreqOffset(s_rail);
    D->foff_raw = (fo == RAIL_FREQUENCY_OFFSET_INVALID) ? INT16_MIN : (int16_t)fo;
    D->rssi_qdbm = RAIL_GetRssi(s_rail, false);
  }
  s_blk.hdr.f_center_hz = (s_chan == 'B') ? 162025000u : 161975000u;
  s_st.uptime_ms = now - s_t0_ms;
  s_st.noise_qdbm = s_d.in ? (int16_t)RAIL_RSSI_INVALID : RAIL_GetRssi(s_rail, false);
  s_blk.st = s_st;
  for (uint32_t i = 0; i < n; i++) s_blk.frm[i] = s_q[(s_qr + i) % AIS_QUEUE];
  if (iq_stream_ext_send(&s_blk)) {
    s_qr += n;
    s_last_blk_ms = now;
    s_blocks++;
    if (rawdump) { s_rd_n = 0; s_rd_state = 0; }
  } else {
    s_send_fail++;                           /* RDY timeout: same frames next time */
  }
}

void ais_hw_process(void)
{
  if (!s_active) return;
  g_ais_bc[0]++; g_ais_bc[2] = 100;
  /* bring-up: let the raw capture of this sync fill (56 B = 47 ms) first */
  raw_t *r = s_raw;
  bool raw_busy = r && s_rd_state == 1 && ((uint32_t)RAIL_GetTime() - r->t_us) < 60000u;
  if (s_restart && !raw_busy) {
    s_restart = false;
    s_raw = NULL;
    s_rx_restarts++;
    rx_arm();
  }
  if (s_am) { am_run(); am_flush(now_ms()); }
  if (s_cap_state == 1 && !iq_stream_ext_busy()) cap_run();
  if (s_stream && s_src != RX_DIRECT_SYNCHRONOUS_MODE_DATA && s_cap_state == 0 && !s_cap_hold) drain_fifo();   /* ISR leaves these to us */
  g_ais_bc[2] = 110;
  iq_stream_ext_pump();
  g_ais_bc[2] = 120;
  cap_feed();
  uint32_t now = now_ms();
  if (!iq_stream_ext_busy()) send_block(now);
  g_ais_bc[2] = 130;
  g_ais_bc[5] = (uint32_t)RAIL_GetRadioState(s_rail);
  /* bring-up watchdog: RX must not stay off - re-arm if the radio went idle */
  if (!(g_ais_bc[5] & RAIL_RF_STATE_RX) && !s_restart) { g_ais_bc[7]++; s_restart = true; }
  if (now - s_last_stats_ms >= AIS_STATS_MS) { s_last_stats_ms = now; ais_hw_print_stats(); }
}

void ais_hw_option(const char *a)
{
  if (*a == 'L') {
    s_data_lsb_first = !s_data_lsb_first;
    printf("# AIS HW: data bits %s-first\r\n", s_data_lsb_first ? "LSB" : "MSB");
  } else if (*a == 'Y') {
    s_sync_lsb_first = !s_sync_lsb_first;
    if (s_active) { apply_sync(); rx_arm(); }
  } else if (*a == 'F') {
    /* AF<ticks>: synth offset for the AIS PHY by hand (bring-up: is the
     * IQ PHY's tick size right for this PHY?) */
    const char *p = a + 1; int sg = 1; long v = 0;
    if (*p == '-') { sg = -1; p++; }
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    s_ftick = (int32_t)(sg * v);
    if (s_active) rx_arm();
  } else if (*a == 'C') {
    /* AC<ch>: next A1 listens on this radio channel instead of AIS A (bring-up
     * of other PHYs in the radioconf, e.g. the 1090 MHz probe at ch 4000);
     * AC0 = back to AIS A/B */
    long v = 0;
    for (const char *p = a + 1; *p >= '0' && *p <= '9'; p++) v = v * 10 + (*p - '0');
    s_ch_override = (uint16_t)v;
    printf("# AIS HW: channel override %u (next A1)\r\n", (unsigned)s_ch_override);
  } else if (*a == 'D') {
    /* AD<n>: RX data source (0 packet/HW sync, 1 demod, 5 direct sync);
     * takes effect at the next A1/A2 */
    int n = a[1] - '0';
    if (n == 0 || n == 1 || n == 4 || n == 5) s_src = (RAIL_RxDataSource_t)n;
    printf("# AIS HW: rx source %d (next A1/A2)\r\n", (int)s_src);
  } else if (*a == 'M') {
    /* AM<thr>: continuous on-chip Mode S monitor; AMS = stop */
    if (a[1] == 'S') { s_am = false; printf("# AIS HW: AM stop\r\n"); }
    else {
      const char *p = a + 1; int sg = 1; int v = 4;
      if (*p == '-') { sg = -1; p++; }
      if (*p >= '0' && *p <= '9') { v = 0; while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0'); }
      am_start((int8_t)(sg * v));
      printf("# AIS HW: AM thr %d\r\n", sg * v);
    }
  } else if (*a == 'K') {
    /* AK<thr>: one burst capture of the raw source at full rate (see cap_run) */
    const char *p = a + 1; int sg = 1; int v = 0;
    if (*p == '-') { sg = -1; p++; }
    while (*p >= '0' && *p <= '9') v = v * 10 + (*p++ - '0');
    if (p != a + 1) s_cap_thr = (int8_t)(sg * v);
    s_cap_len = 0;
    if (*p == ',') {      /* AK<thr>,<bytes>: shorter capture (antenna: one frame ~800 samples) */
      uint32_t l = 0; p++;
      while (*p >= '0' && *p <= '9') l = l * 10 + (uint32_t)(*p++ - '0');
      if (l > CAP_PRE) s_cap_len = l;
    }
    s_cap_rate = 0; s_cap_state = 1;
  } else if (*a == 'R') {
    s_rd_trig = (a[1] == '2') ? 0 : 1;       /* AR: dump 960 stream bytes (AISRAW); AR2 = no trigger */
    s_rd_n = 0; s_rd_state = 1;
  } else if (*a == 'S') {
    int n = 0;
    for (const char *p = a + 1; *p >= '0' && *p <= '9'; p++) n = n * 10 + (*p - '0');
    if (n >= 8 && n <= 32) { s_sync_bits = (uint8_t)n; if (s_active) { apply_sync(); rx_arm(); } }
    else printf("# AIS HW: AS<8..32> = sync word length\r\n");
  } else {
    printf("# AIS HW: A1/A2 = channel A/B, A0 = stop, AL = data bit order, "
           "AY = sync bit order, AS<n> = sync bits\r\n");
  }
}

void ais_hw_print_stats(void)
{
  printf("# AIS HW %c: sync %lu  ok %lu  crc_bad %lu  abort %lu  drop %lu  ovf %lu  "
         "rearm %lu  blocks %lu  send_fail %lu  noise %d dBm  %lu s\r\n",
         s_chan, (unsigned long)s_st.n_sync, (unsigned long)s_st.n_ok,
         (unsigned long)s_st.n_crc_bad, (unsigned long)s_st.n_abort,
         (unsigned long)s_st.n_drop, (unsigned long)s_st.n_ovf,
         (unsigned long)s_rx_restarts, (unsigned long)s_blocks, (unsigned long)s_send_fail,
         s_active ? (int)(RAIL_GetRssi(s_rail, false) / 4) : 0,
         (unsigned long)((now_ms() - s_t0_ms) / 1000u));
}

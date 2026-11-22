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
#define AIS_HEARTBEAT_MS  10000u
#define AIS_STATS_MS      60000u

static RAIL_Handle_t s_rail;
static double        s_tick_hz = 4.6492;
static volatile bool s_active = false;
static char          s_chan = 'A';
static uint16_t      s_ch = AIS_CH_A;
static int32_t       s_ftick = 0;

/* options (runtime, for the bring-up: which bit order the FIFO really has) */
static bool    s_data_lsb_first = true;   /* first air bit = byte bit 0 */
static bool    s_sync_lsb_first = true;   /* RAIL: LSB first (rail_types.h) */
/* 16 bits = 8 training bits + flag. The bit-level model (tools/ais_hw_model.py)
 * on the 22 Nov powerbank capture: 16 bits 58/58 frames, 24 bits 56, 32 bits
 * 54 - the start of the training sequence is often damaged. Noise false-sync
 * rate at 16 bits: 9600 * 2^-16 * 2 words ~ 0.3/s, each costing a few ms. */
static uint8_t s_sync_bits = 16;

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
  s_restart = true;                           /* re-arm: the next slot may follow at once */
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

static void drain_fifo(void)
{
  uint8_t tmp[32];
  uint16_t n;
  while ((n = RAIL_GetRxFifoBytesAvailable(s_rail)) > 0) {
    if (n > sizeof tmp) n = sizeof tmp;
    n = RAIL_ReadRxFifo(s_rail, tmp, n);
    if (n == 0) break;
    if (s_d.in) dec_bytes(tmp, n);           /* after an early end the rest is discarded */
  }
}

bool ais_hw_on_event(RAIL_Handle_t rail, RAIL_Events_t ev)
{
  (void)rail;
  if (!s_active) return false;
  if (ev & RAIL_EVENT_RX_FIFO_OVERFLOW) { s_st.n_ovf++; s_d.in = false; s_restart = true; }
  if (ev & (RAIL_EVENT_RX_SYNC1_DETECT | RAIL_EVENT_RX_SYNC2_DETECT)) {
    /* the FIFO is empty here: rx_arm() resets it, and a frame that ran to the
     * fixed length was drained on RX_PACKET_RECEIVED before this sync */
    if (!s_d.in) dec_begin((ev & RAIL_EVENT_RX_SYNC1_DETECT) ? 1u : 2u);
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
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_ftick);
  RAIL_StartRx(s_rail, s_ch, NULL);
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
  s_ftick = freq_tick;

  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  /* Loads the AIS PHY of that channel entry (multi-PHY radio config). It
   * also re-applies the PHY's data config, so the packet source is set
   * after it, not before. */
  RAIL_Status_t pst = RAIL_PrepareChannel(s_rail, s_ch);
  if (pst != RAIL_STATUS_NO_ERROR) {
    printf("# AIS HW: channel %u not in the radio config (st=%d) - generate the AIS PHY first, "
           "docs/fg23-ais-onchip.md\r\n", (unsigned)s_ch, (int)pst);
    return false;
  }
  RAIL_DataConfig_t dc = {
    .txSource = TX_PACKET_DATA,
    .rxSource = RX_PACKET_DATA,
    .txMethod = PACKET_MODE,
    .rxMethod = FIFO_MODE,
  };
  RAIL_ConfigData(s_rail, &dc);
  RAIL_SetFixedLength(s_rail, AIS_FIXED_LEN);
  apply_sync();
  RAIL_StateTransitions_t tr = { RAIL_RF_STATE_RX, RAIL_RF_STATE_RX };
  RAIL_SetRxTransitions(s_rail, &tr);
  RAIL_SetRxFifoThreshold(s_rail, AIS_FIFO_THRESH);
  RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_SYNC1_DETECT | RAIL_EVENT_RX_SYNC2_DETECT
                    | RAIL_EVENT_RX_FIFO_ALMOST_FULL | RAIL_EVENT_RX_FIFO_OVERFLOW
                    | RAIL_EVENT_RX_PACKET_RECEIVED | RAIL_EVENT_RX_PACKET_ABORTED
                    | RAIL_EVENT_RX_FRAME_ERROR);

  memset(&s_st, 0, sizeof s_st);
  s_qw = s_qr = 0; s_restart = false;
  s_rx_restarts = 0; s_send_fail = 0; s_blocks = 0;
  s_t0_ms = s_last_blk_ms = s_last_stats_ms = now_ms();

  iq_stream_ext_begin();
  s_active = true;
  rx_arm();
  printf("# AIS HW: channel %c (radio ch %u), fixed %u B, data %s-first, offset %ld ticks\r\n",
         s_chan, (unsigned)s_ch, (unsigned)AIS_FIXED_LEN, s_data_lsb_first ? "LSB" : "MSB",
         (long)s_ftick);
  return true;
}

void ais_hw_stop(void)
{
  if (!s_active) return;
  s_active = false;
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_SetFixedLength(s_rail, RAIL_SETFIXEDLENGTH_INVALID);
  RAIL_ResetFifo(s_rail, false, true);
  uint32_t t = now_ms();
  while (iq_stream_ext_busy() && now_ms() - t < 20u) iq_stream_ext_pump();
  iq_stream_ext_end();
  ais_hw_print_stats();
  printf("# AIS HW: stopped\r\n");
}

static void send_block(uint32_t now)
{
  uint32_t n = s_qw - s_qr;
  if (n > AISPKT_MAX_FRAMES) n = AISPKT_MAX_FRAMES;
  if (n == 0 && now - s_last_blk_ms < AIS_HEARTBEAT_MS) return;
  memset(&s_blk, 0, sizeof s_blk);
  s_blk.hdr.magic = AISPKT_MAGIC;
  s_blk.hdr.nfrm = (uint16_t)n;
  s_blk.hdr.flags = AISPKT_FLAG_LAST;
  s_blk.hdr.f_center_hz = (s_chan == 'B') ? 162025000u : 161975000u;
  s_st.uptime_ms = now - s_t0_ms;
  s_st.noise_qdbm = s_d.in ? (int16_t)RAIL_RSSI_INVALID : RAIL_GetRssi(s_rail, false);
  s_blk.st = s_st;
  for (uint32_t i = 0; i < n; i++) s_blk.frm[i] = s_q[(s_qr + i) % AIS_QUEUE];
  if (iq_stream_ext_send(&s_blk)) {
    s_qr += n;
    s_last_blk_ms = now;
    s_blocks++;
  } else {
    s_send_fail++;                           /* RDY timeout: same frames next time */
  }
}

void ais_hw_process(void)
{
  if (!s_active) return;
  if (s_restart) {
    s_restart = false;
    s_rx_restarts++;
    rx_arm();
  }
  iq_stream_ext_pump();
  uint32_t now = now_ms();
  if (!iq_stream_ext_busy()) send_block(now);
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

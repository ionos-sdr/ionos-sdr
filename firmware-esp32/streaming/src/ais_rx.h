/* ais_rx.h — fixed-point AIS receiver (see ais_rx.c). MIT, HA7DCD. */
#ifndef AIS_RX_H
#define AIS_RX_H
#include <stdint.h>
#include <stdio.h>
#ifdef __cplusplus
extern "C" {
#endif

#define AIS_CH_MAX     96
#define AIS_FRAME_MAX 140          /* 5-slot message + FCS */
#define AIS_MM_N      640          /* max moving-mean window: 128 bits at 48 kHz */
#define AIS_MM_DEFAULT 320         /* 64 bits */
#define AIS_FC_DEFAULT 6000        /* channel FIR cutoff [Hz] */

/* frame callback: raw HDLC bytes incl. FCS (CRC already verified),
 * channel 'A'/'B', time stamp in 48 kHz output samples */
typedef void (*ais_frame_cb)(const uint8_t *fr, int len, char chan, uint32_t t48k, void *ctx);

typedef struct {
  uint8_t  win, byte, nbit, inf, prev;
  uint16_t len;
  uint8_t  fr[AIS_FRAME_MAX];
} ais_lane_t;

typedef struct {
  uint32_t sps; char chan; ais_frame_cb cb; void *ctx;
  uint8_t  swap_iq;
  int32_t  fc_hz; int mm_n;        /* set BEFORE ais_rx_init to override the defaults */
  uint32_t nco_ph, nco_step;
  int16_t  ch_h[AIS_CH_MAX];
  int16_t  xi[2 * AIS_CH_MAX], xq[2 * AIS_CH_MAX];
  int      ch_n, pos;
  int16_t  pi, pq;
  int32_t  prev_a;
  uint32_t rs_pos, rs_step;
  int32_t  mm_buf[AIS_MM_N]; int32_t mm_sum; int mm_idx;
  ais_lane_t lane[5]; int lane_k;
  uint32_t nout, last_out; uint16_t last_fcs;
  uint32_t frames, crc_bad;
} ais_rx_t;

/* off_hz = channel frequency - tuned centre frequency */
void ais_rx_init(ais_rx_t *r, uint32_t sps, int32_t off_hz, char chan, ais_frame_cb cb, void *ctx);
void ais_rx_feed(ais_rx_t *r, const int16_t *iq, int nsamp);   /* interleaved I,Q */
int  ais_nmea(const uint8_t *fr, int len, char chan, char *out, int outsz);
int  ais_describe(const uint8_t *fr, int len, char *out, int outsz);

#ifdef __cplusplus
}
#endif
#endif

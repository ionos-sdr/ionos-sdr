/* ais_rx.h — fixed-point AIS receiver (see ais_rx.c). MIT, HA7DCD. */
#ifndef AIS_RX_H
#define AIS_RX_H
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

#define AIS_CH_MAX     24           /* channel FIR taps: 17 at 25 ksps (behind the DDC); direct use above ~35 ksps clamps to 23 */
#define AIS_FRAME_MAX 140          /* 5-slot message + FCS */
#define AIS_MM_N      320          /* max moving-mean window: 64 bits at 48 kHz (the default) */
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
  uint32_t sps_i; int dec, dph;    /* the FIR decimates sps -> sps_i (~50 ksps): 100 ksps dual-channel input */
  uint8_t  swap_iq;
  int32_t  fc_hz; int mm_n;        /* set BEFORE ais_rx_init to override the defaults */
  int32_t  hyp_hz;                 /* set BEFORE ais_rx_init: this instance's frequency hypothesis (added to foff) */
  uint32_t nco_ph, nco_step;
  int16_t  ch_h[AIS_CH_MAX];
  int16_t  xi[2 * AIS_CH_MAX], xq[2 * AIS_CH_MAX];
  int      ch_n, pos;
  int16_t  pi, pq;
  int32_t  prev_a;
  uint32_t rs_pos, rs_step;
  int16_t  mm_buf[AIS_MM_N]; int32_t mm_sum; int mm_idx, mid_idx;
  int16_t  d_buf[AIS_MM_N];      /* discriminator delay line for the decisions (mm_buf holds the gated copy) */
  ais_lane_t lane[5]; int lane_k;
  uint32_t nout, last_out; uint16_t last_fcs;
  uint32_t frames, crc_bad;
  void    *owner;                   /* ais_multi: the channel this instance belongs to */
  /* per-packet RF parameters (valid inside the frame callback):
   * level = channel power over the last ~5 ms of the frame, noise = slow
   * floor tracked outside bursts, foff = carrier offset from the slicer mean */
  int64_t  p_fast, p_noise, p_short;
  uint8_t  gate;                   /* signal present (short power > 3x floor): feeds the slicer mean */
  int32_t  afc_est;                 /* AFC: carrier offset estimate, Q13 angle per sample (at sps_i), Q8 */
  int32_t  afc_step;                /* NCO correction (per input sample) */
  int      afc_sh;                  /* EMA shift, 0 = AFC off */
  float    last_lvl_dbfs, last_snr_db, last_foff_hz;
} ais_rx_t;

/* off_hz = channel frequency - tuned centre frequency */
void ais_rx_init(ais_rx_t *r, uint32_t sps, int32_t off_hz, char chan, ais_frame_cb cb, void *ctx);
void ais_rx_feed(ais_rx_t *r, const int16_t *iq, int nsamp);   /* interleaved I,Q */

/* ---------------- multi-channel, multi-hypothesis front end ----------------
 * One I/Q stream (e.g. 100 ksps at 162.000 MHz) -> per AIS channel: NCO mix +
 * decimating FIR to ~25 ksps (DDC), then nhyp ais_rx instances at frequency
 * hypotheses 0, +-step, ... (the 6 kHz channel filter off-centre was what
 * limited the offset tolerance to ~+-1 kHz), frames de-duplicated by FCS. */
#define AIS_DDC_MAX   64           /* 61 taps at 100 ksps */
#define AIS_HYP_MAX    5
#define AIS_DDC_BLK  128
struct ais_multi_s;
typedef struct {
  char     chan; int32_t off_hz;
  uint32_t nco_ph, nco_step;
  int16_t  h[AIS_DDC_MAX]; int n, pos, D, dph;
  int16_t  xi[2 * AIS_DDC_MAX], xq[2 * AIS_DDC_MAX];
  int16_t  buf[2 * AIS_DDC_BLK]; int nb;
  int      nh; ais_rx_t *rx[AIS_HYP_MAX];
  uint16_t dd_fcs[8]; uint32_t dd_t[8]; int dd_k;
  uint32_t frames;
  struct ais_multi_s *m;
} ais_ch_t;
typedef struct ais_multi_s {
  uint32_t sps, sps_ddc; uint32_t tune_khz; int nch;
  ais_ch_t ch[2];
  ais_frame_cb cb; void *ctx;
  ais_rx_t *pool[2 * AIS_HYP_MAX];   /* allocated receivers, kept over re-inits */
  const ais_rx_t *last;            /* the instance that decoded the frame (valid in the callback) */
} ais_multi_t;
/* channels A (161.975) and B (162.025) are used when they fit in the band */
/* alloc: where the per-hypothesis receivers live (the firmware tries internal RAM first:
 * in PSRAM the dual setup ran ~10x slower than real time). Re-init frees nothing: the
 * instances are kept in m->pool and reused. Returns 0, or -1 when alloc failed. */
typedef void *(*ais_alloc_fn)(size_t n);
int  ais_multi_init(ais_multi_t *m, uint32_t sps, uint32_t tune_khz, int nhyp, int32_t hyp_step_hz,
                    ais_frame_cb cb, void *ctx, ais_alloc_fn alloc);
void ais_multi_feed(ais_multi_t *m, const int16_t *iq, int nsamp);
uint32_t ais_multi_crc_bad(const ais_multi_t *m);

int  ais_nmea(const uint8_t *fr, int len, char chan, char *out, int outsz);
int  ais_describe(const uint8_t *fr, int len, char *out, int outsz);

#ifdef __cplusplus
}
#endif
#endif

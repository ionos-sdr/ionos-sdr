/* ais_rx.c — fixed-point AIS (GMSK 9600, HDLC, NMEA) receiver, one channel per instance.
 *
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT
 *
 * ONE source for the firmware AND the PC test harness (aisport.c), so the two
 * cannot drift apart (lesson of 2026-08-09: twin != firmware hid two bugs).
 *
 * Chain (per channel, int16 I/Q in, any sps ~25..200 k):
 *   NCO mix to the channel -> complex FIR 6 kHz (Q15) -> FM discriminator
 *   (fm_angle_q13, overflow-safe) -> linear-interp resample to 48 kHz (5 sps)
 *   -> 64-bit moving-mean threshold -> 5 PARALLEL sample-phase lanes, each
 *   with its own NRZI + HDLC (destuff, CRC-16/X.25) -> de-duplication.
 *
 * Design choices, measured with ais_ref.py on synthetic GMSK (20 bursts/pt):
 *   channel LPF 6 kHz best (5k / 8k worse); NO Gaussian post filter (ISI);
 *   threshold window 64 bits (16 bits: 8/20 at 20 dB). Lanes instead of a
 *   PLL: an AIS burst is ~26 ms with only 24 training bits — a PLL that
 *   locks that fast is noisy, five fixed phases cost five HDLC states only.
 */
#include "ais_rx.h"
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PI_Q13 25736
#define AIS_FS_OUT 48000u
#ifndef AIS_AFC_SH
#define AIS_AFC_SH 8             /* AFC time constant 2^8 samples (~10 ms at 25 ksps): follows the receiver's
                                  * crystal error and the mean station offset, not each burst */
#endif

/* ---------------- shared math ---------------- */
static inline int32_t atan2_q13(int32_t y, int32_t x)
{
  int32_t abs_y = (y < 0 ? -y : y) + 1;
  int32_t angle;
  if (x >= 0) {
    int32_t r = ((x - abs_y) << 13) / (x + abs_y);
    angle = (PI_Q13 >> 2) - (((PI_Q13 >> 2) * r) >> 13);
  } else {
    int32_t r = ((x + abs_y) << 13) / (abs_y - x);
    angle = (3 * (PI_Q13 >> 2)) - (((PI_Q13 >> 2) * r) >> 13);
  }
  return (y < 0) ? -angle : angle;
}
static inline int32_t fm_angle_q13(int32_t di, int32_t dr)
{
  int32_t ax = (dr < 0) ? -dr : dr, ay = (di < 0) ? -di : di;
  uint32_t m = (uint32_t)((ax > ay) ? ax : ay);
  int sh = 0;
  if (m > 65535u) sh = 16 - __builtin_clz(m);
  return atan2_q13(di >> sh, dr >> sh);
}

static int16_t s_sin_lut[1024];
static int     s_lut_ok = 0;
static void lut_init(void)
{
  if (s_lut_ok) return;
  for (int k = 0; k < 1024; k++) s_sin_lut[k] = (int16_t)lrint(32767.0 * sin(2 * M_PI * k / 1024.0));
  s_lut_ok = 1;
}

static uint16_t crc_update(uint16_t crc, uint8_t b)
{
  b ^= (uint8_t)(crc & 0xFF); b ^= (uint8_t)(b << 4);
  return (uint16_t)(((uint16_t)b << 8) | (crc >> 8)) ^ (uint16_t)(b >> 4) ^ ((uint16_t)b << 3);
}

/* ---------------- setup ---------------- */
void ais_rx_init(ais_rx_t *r, uint32_t sps, int32_t off_hz, char chan,
                 ais_frame_cb cb, void *ctx)
{
  int32_t fc = r->fc_hz, hy = r->hyp_hz; int mm = r->mm_n;   /* keep tuning set before init */
  memset(r, 0, sizeof *r);
  r->fc_hz = fc; r->mm_n = mm; r->hyp_hz = hy;
  lut_init();
  r->sps = sps; r->chan = chan; r->cb = cb; r->ctx = ctx;
  r->nco_step = (uint32_t)(int32_t)llround(-(double)off_hz / (double)sps * 4294967296.0);

  if (!r->fc_hz) r->fc_hz = AIS_FC_DEFAULT;
  if (!r->mm_n || r->mm_n > AIS_MM_N) r->mm_n = AIS_MM_DEFAULT;
  r->mid_idx = r->mm_n / 2;
  int n = (int)((31u * sps + 25000u) / 50000u) | 1;
  if (n < 15) n = 15;
  if (n > AIS_CH_MAX - 1) n = AIS_CH_MAX - 1;
  r->ch_n = n;
  const int M = n - 1; const double x = 2.0 * (double)r->fc_hz / (double)sps;
  double h[AIS_CH_MAX], sum = 0;
  for (int k = 0; k < n; k++) {
    double t = k - M / 2.0;
    double sn = (fabs(t) < 1e-9) ? x : sin(M_PI * x * t) / (M_PI * t);
    double w = 0.42 - 0.5 * cos(2 * M_PI * k / M) + 0.08 * cos(4 * M_PI * k / M);
    h[k] = sn * w; sum += h[k];
  }
  for (int k = 0; k < n; k++) r->ch_h[k] = (int16_t)lrint(32767.0 * h[k] / sum);

  /* decimate in the FIR (outputs computed only every dec-th input) to ~50 ksps:
   * the 6 kHz channel filter is its own anti-alias filter */
  r->dec = (int)((sps + 25000u) / 50000u); if (r->dec < 1) r->dec = 1;
  r->sps_i = sps / (uint32_t)r->dec; r->dph = 0;
  r->afc_sh = AIS_AFC_SH;
  r->rs_step = (uint32_t)(((uint64_t)r->sps_i << 16) / AIS_FS_OUT);   /* FIR outputs per output sample, Q16 */
  r->rs_pos = 0;
}

/* ---------------- HDLC per lane ---------------- */
static void lane_bit(ais_rx_t *r, ais_lane_t *L, int bit)
{
  L->win = (uint8_t)((L->win << 1) | (bit ? 1 : 0));
  if (L->win == 0x7E) {
    /* >= 11 bytes: the shortest AIS message (type 7/13 ack, 72 bits) + FCS.
     * With 4 the ~1/65536 random CRC match let 56-bit junk through live. */
    if (L->inf && L->len >= 11) {
      uint16_t crc = 0xFFFF;
      for (int i = 0; i < L->len; i++) crc = crc_update(crc, L->fr[i]);
      if (crc == 0xF0B8) {
        /* de-dup: adjacent lanes decode the same burst */
        uint16_t fcs = (uint16_t)(L->fr[L->len - 2] | (L->fr[L->len - 1] << 8));
        if (!(fcs == r->last_fcs && r->nout - r->last_out < 2000u)) {
          r->last_fcs = fcs; r->last_out = r->nout; r->frames++;
          {
            double pf = (double)r->p_fast / 256.0 + 1e-3, pn = (double)r->p_noise / 256.0 + 1e-3;
            r->last_lvl_dbfs = (float)(10.0 * log10(pf / (32767.0 * 32767.0)));
            r->last_snr_db   = (float)(10.0 * log10(pf / pn));
            /* slicer mean = carrier offset; angle units: PI_Q13 = pi rad/sample */
            r->last_foff_hz  = (float)((double)r->mm_sum / r->mm_n / PI_Q13 * r->sps_i / 2.0 + r->hyp_hz);
          }
          if (r->cb) r->cb(L->fr, L->len, r->chan, r->nout, r->ctx);
        }
      } else if (L->len >= 23) r->crc_bad++;
    }
    L->inf = 1; L->len = 0; L->byte = 0; L->nbit = 0; return;
  }
  if ((L->win & 0xFE) == 0xFE) { L->inf = 0; L->len = 0; L->nbit = 0; return; }
  if (!L->inf) return;
  if ((L->win & 0x3F) == 0x3E) return;                 /* stuffed zero */
  if (L->win & 1) L->byte |= 0x80;
  if (++L->nbit >= 8) {
    if (L->len < AIS_FRAME_MAX) L->fr[L->len++] = L->byte;
    else { L->inf = 0; L->len = 0; }
    L->byte = 0; L->nbit = 0;
  } else L->byte >>= 1;
}

/* one 48 kHz soft sample */
static void out_sample(ais_rx_t *r, int32_t v)
{
  /* centred 64-bit (320-sample) moving mean as the slicer threshold */
  /* Threshold = carrier offset. Only samples with a signal present go into
   * the mean; in a gap the mean holds (its own value is pushed). Without the
   * gate the window ran 32 bits into the noise after the burst, the
   * threshold fell to half the offset over the FCS and the closing flag, and
   * the demod lost bursts beyond about +-1 kHz (VSG, 2026-11-23/24). */
  if (v > 32767) v = 32767; else if (v < -32767) v = -32767;
  int32_t old = r->mm_buf[r->mm_idx];
  int32_t vm = r->gate ? v : r->mm_sum / r->mm_n;
  r->mm_buf[r->mm_idx] = (int16_t)vm;
  r->d_buf[r->mm_idx] = (int16_t)v;
  r->mm_sum += vm - old;
  if (++r->mm_idx >= r->mm_n) r->mm_idx = 0;
  int32_t mid = r->d_buf[r->mid_idx];                                /* delayed by N/2 */
  if (++r->mid_idx >= r->mm_n) r->mid_idx = 0;
  int32_t soft = mid * r->mm_n - r->mm_sum;

  ais_lane_t *L = &r->lane[r->lane_k];
  int b = soft > 0;
  lane_bit(r, L, b == L->prev);                       /* NRZI: no transition = 1 */
  L->prev = b;
  if (++r->lane_k >= 5) r->lane_k = 0;
  r->nout++;
}

/* ---------------- input ---------------- */
void ais_rx_feed(ais_rx_t *r, const int16_t *iq, int nsamp)
{
  for (int n = 0; n < nsamp; n++) {
    int32_t i0 = iq[2 * n], q0 = iq[2 * n + 1];
    if (r->swap_iq) { int32_t t = i0; i0 = q0; q0 = t; }

    /* NCO mix: (i + jq) * e^{j phi} */
    uint32_t ph = r->nco_ph; r->nco_ph += r->nco_step + (uint32_t)r->afc_step;
    int32_t s = s_sin_lut[ph >> 22], c = s_sin_lut[((ph >> 22) + 256) & 1023];
    int32_t mi = (i0 * c - q0 * s) >> 15;
    int32_t mq = (i0 * s + q0 * c) >> 15;

    r->xi[r->pos] = r->xi[r->pos + r->ch_n] = (int16_t)mi;
    r->xq[r->pos] = r->xq[r->pos + r->ch_n] = (int16_t)mq;
    if (++r->pos >= r->ch_n) r->pos = 0;
    if (++r->dph < r->dec) continue;
    r->dph = 0;
    const int16_t *xi = &r->xi[r->pos], *xq = &r->xq[r->pos];
    /* symmetric FIR (odd length): fold, half the multiplies */
    const int N = r->ch_n, Hh = N >> 1;
    int32_t ai = (int32_t)r->ch_h[Hh] * xi[Hh], aq = (int32_t)r->ch_h[Hh] * xq[Hh];
    for (int k = 0; k < Hh; k++) {
      int32_t hk = r->ch_h[k];
      ai += hk * ((int32_t)xi[k] + xi[N - 1 - k]); aq += hk * ((int32_t)xq[k] + xq[N - 1 - k]);
    }
    int16_t fi = (int16_t)((ai + 16384) >> 15), fq = (int16_t)((aq + 16384) >> 15);

    /* channel power: fast EMA (~256 samples = 5 ms at 50 ksps) for the packet
     * level, slow floor (~0.3 s) updated only outside bursts */
    {
      int64_t p = ((int64_t)fi * fi + (int64_t)fq * fq) << 8;      /* Q8 */
      r->p_fast += (p - r->p_fast) >> 8;
      r->p_short += (p - r->p_short) >> 3;                         /* ~8 samples */
      /* floor: follows DOWN fast, UP slowly, bursts (>4x) almost not at all */
      if (r->p_noise <= 0) r->p_noise = r->p_fast;
      else if (r->p_fast < r->p_noise) r->p_noise += (r->p_fast - r->p_noise) >> 6;
      else if (r->p_fast < 4 * r->p_noise) r->p_noise += (r->p_fast - r->p_noise) >> 14;
      else r->p_noise += (r->p_fast - r->p_noise) >> 20;
    }

    /* gate relative to the recent burst level, not to the noise floor: on a
     * busy channel (a burst in every slot) the floor estimate crept up to the
     * signal and the gate stayed shut - VSG 2026-11-25, -104 dBm: 70 % PER.
     * Peak: fast attack, ~20 ms decay; after a burst the noise is >6 dB below it. */
    if (r->p_short > r->p_peak) r->p_peak += (r->p_short - r->p_peak) >> 1;
    else r->p_peak -= r->p_peak >> 9;
    r->gate = r->p_short * 4 > r->p_peak;
    int32_t dr = (int32_t)fi * r->pi + (int32_t)fq * r->pq;
    int32_t di = (int32_t)fq * r->pi - (int32_t)fi * r->pq;
    r->pi = fi; r->pq = fq;
    int32_t a = fm_angle_q13(di, dr);                 /* +-PI_Q13 */
    /* AFC: the NCO follows the carrier so the 6 kHz channel filter stays
     * centred (off-centre it cost the bursts beyond ~+-1 kHz, VSG 2026-11-23).
     * The slicer gets a + correction = the frequency relative to the nominal
     * channel, so the threshold and the reported offset do not see the loop. */
    {
      int32_t c = r->afc_est >> 8;
      a += c;
      if (r->afc_sh && r->gate) {
        r->afc_est += ((a << 8) - r->afc_est) >> r->afc_sh;
        int32_t lim = (int32_t)((int64_t)4500 * 2 * PI_Q13 / r->sps_i) << 8;       /* +-4.5 kHz */
        if (r->afc_est > lim) r->afc_est = lim; else if (r->afc_est < -lim) r->afc_est = -lim;
        /* NCO step for the estimate: f = c * sps_i / (2 PI_Q13); step = -f / sps * 2^32 */
        r->afc_step = -(int32_t)(((int64_t)(r->afc_est >> 8) * (int64_t)r->sps_i * 2147483648LL) / ((int64_t)PI_Q13 * r->sps));
      }
    }

    /* resample to 48 kHz: outputs at input positions rs_pos (Q16, relative
     * to the PREVIOUS input sample), linear interpolation prev..a */
    while (r->rs_pos < 65536u) {
      int32_t f = (int32_t)r->rs_pos;                 /* 0..65535 */
      int32_t v = r->prev_a + (int32_t)(((int64_t)(a - r->prev_a) * f) >> 16);
      out_sample(r, v);
      r->rs_pos += r->rs_step;
    }
    r->rs_pos -= 65536u;
    r->prev_a = a;
  }
}


/* ---------------- multi-channel front end (see ais_rx.h) ---------------- */
static void multi_frame(const uint8_t *fr, int len, char chan, uint32_t t48k, void *ctx)
{
  ais_rx_t *r = (ais_rx_t *)ctx;
  ais_ch_t *ch = (ais_ch_t *)r->owner;
  uint16_t fcs = (uint16_t)(fr[len - 2] | (fr[len - 1] << 8));
  for (int i = 0; i < 8; i++) {
    int32_t d = (int32_t)(t48k - ch->dd_t[i]);
    if (ch->dd_fcs[i] == fcs && d < 2400 && d > -2400) return;      /* same burst, other hypothesis */
  }
  ch->dd_fcs[ch->dd_k] = fcs; ch->dd_t[ch->dd_k] = t48k; ch->dd_k = (ch->dd_k + 1) & 7;
  ch->frames++;
  ch->m->last = r;
  if (ch->m->cb) ch->m->cb(fr, len, chan, t48k, ch->m->ctx);
}

static void ddc_init(ais_ch_t *ch, uint32_t sps, int32_t off_hz)
{
  ch->off_hz = off_hz;
  ch->nco_step = (uint32_t)(int32_t)llround(-(double)off_hz / (double)sps * 4294967296.0);
  ch->D = (int)((sps + 12500u) / 25000u); if (ch->D < 1) ch->D = 1;
  int n = (int)((61u * sps + 50000u) / 100000u) | 1;
  if (n < 15) n = 15;
  if (n > AIS_DDC_MAX) n = AIS_DDC_MAX;
  ch->n = n;
  /* 10 kHz cutoff: flat over the AIS channel (+-~7 kHz incl. a 2-3 kHz offset),
   * the decimation alias band (25k - 8k) at >= ~60 dB for n = 61 at 100 ksps */
  const int M = n - 1; const double x = 2.0 * 10000.0 / (double)sps;
  double hh[AIS_DDC_MAX], sum = 0;
  for (int k = 0; k < n; k++) {
    double t = k - M / 2.0;
    double sn = (fabs(t) < 1e-9) ? x : sin(M_PI * x * t) / (M_PI * t);
    double w = 0.42 - 0.5 * cos(2 * M_PI * k / M) + 0.08 * cos(4 * M_PI * k / M);
    hh[k] = sn * w; sum += hh[k];
  }
  for (int k = 0; k < n; k++) ch->h[k] = (int16_t)lrint(32767.0 * hh[k] / sum);
}

int ais_multi_init(ais_multi_t *m, uint32_t sps, uint32_t tune_khz, int nhyp, int32_t hyp_step_hz,
                   ais_frame_cb cb, void *ctx, ais_alloc_fn alloc)
{
  ais_rx_t *pool[2 * AIS_HYP_MAX];
  memcpy(pool, m->pool, sizeof pool);
  memset(m, 0, sizeof *m);
  memcpy(m->pool, pool, sizeof pool);
  int np = 0, rc = 0;
  lut_init();
  m->sps = sps; m->tune_khz = tune_khz; m->cb = cb; m->ctx = ctx;
  if (nhyp < 1) nhyp = 1;
  if (nhyp > AIS_HYP_MAX) nhyp = AIS_HYP_MAX;
  static const struct { char c; uint32_t khz; } CH[2] = { { 'A', 161975u }, { 'B', 162025u } };
  for (int c = 0; c < 2; c++) {
    int32_t off = ((int32_t)CH[c].khz - (int32_t)tune_khz) * 1000;
    if ((off < 0 ? -off : off) + 8000 > (int32_t)sps / 2) continue;
    ais_ch_t *ch = &m->ch[m->nch++];
    ch->chan = CH[c].c; ch->m = m;
    ddc_init(ch, sps, off);
    m->sps_ddc = sps / (uint32_t)ch->D;
    ch->nh = nhyp;
    for (int k = 0; k < nhyp; k++) {
      int32_t hz = (k == 0) ? 0 : ((k + 1) / 2) * hyp_step_hz * ((k & 1) ? 1 : -1);
      if (!m->pool[np]) m->pool[np] = alloc ? (ais_rx_t *)alloc(sizeof(ais_rx_t)) : NULL;
      ais_rx_t *r = m->pool[np];
      if (!r) { rc = -1; ch->nh = k; break; }
      np++;
      ch->rx[k] = r;
      memset(r, 0, sizeof *r);
      r->hyp_hz = hz;
      ais_rx_init(r, m->sps_ddc, hz, ch->chan, multi_frame, r);
      r->owner = ch;
    }
  }
  return rc;
}

static void ddc_flush(ais_ch_t *ch)
{
  if (!ch->nb) return;
  for (int k = 0; k < ch->nh; k++) ais_rx_feed(ch->rx[k], ch->buf, ch->nb);
  ch->nb = 0;
}

void ais_multi_feed(ais_multi_t *m, const int16_t *iq, int nsamp)
{
  for (int c = 0; c < m->nch; c++) {
    ais_ch_t *ch = &m->ch[c];
    for (int n = 0; n < nsamp; n++) {
      int32_t i0 = iq[2 * n], q0 = iq[2 * n + 1];
      uint32_t ph = ch->nco_ph; ch->nco_ph += ch->nco_step;
      int32_t s = s_sin_lut[ph >> 22], co = s_sin_lut[((ph >> 22) + 256) & 1023];
      int16_t mi = (int16_t)((i0 * co - q0 * s) >> 15), mq = (int16_t)((i0 * s + q0 * co) >> 15);
      ch->xi[ch->pos] = ch->xi[ch->pos + ch->n] = mi;
      ch->xq[ch->pos] = ch->xq[ch->pos + ch->n] = mq;
      if (++ch->pos >= ch->n) ch->pos = 0;
      if (++ch->dph < ch->D) continue;
      ch->dph = 0;
      const int16_t *xi = &ch->xi[ch->pos], *xq = &ch->xq[ch->pos];
      const int N = ch->n, Hh = N >> 1;                   /* symmetric, odd length: fold */
      int32_t ai = (int32_t)ch->h[Hh] * xi[Hh], aq = (int32_t)ch->h[Hh] * xq[Hh];
      for (int k = 0; k < Hh; k++) {
        int32_t hk = ch->h[k];
        ai += hk * ((int32_t)xi[k] + xi[N - 1 - k]); aq += hk * ((int32_t)xq[k] + xq[N - 1 - k]);
      }
      int32_t oi = (ai + 16384) >> 15, oq = (aq + 16384) >> 15;
      if (oi > 32767) oi = 32767; else if (oi < -32768) oi = -32768;
      if (oq > 32767) oq = 32767; else if (oq < -32768) oq = -32768;
      ch->buf[2 * ch->nb] = (int16_t)oi; ch->buf[2 * ch->nb + 1] = (int16_t)oq;
      if (++ch->nb >= AIS_DDC_BLK) ddc_flush(ch);
    }
    ddc_flush(ch);
  }
}

uint32_t ais_multi_crc_bad(const ais_multi_t *m)
{
  uint32_t s = 0;
  for (int c = 0; c < m->nch; c++) for (int k = 0; k < m->ch[c].nh; k++) s += m->ch[c].rx[k]->crc_bad;
  return s;
}

/* ---------------- payload helpers ---------------- */
/* BIT ORDER (measured, 2026-11-22, real Danube traffic: "KISFALUDY",
 * virtual AtoN "1645.3_G"): the frame bytes as the HDLC lanes assemble them
 * (first bit on air -> LSB) are ALREADY the message bytes MSB-first. No
 * per-byte reversal. The reversed variant gave CRC-correct nonsense
 * (type 42, latitude 101 deg) — the CRC cannot tell the two apart. */
static uint32_t getbits(const uint8_t *m, int a, int n)
{
  uint32_t v = 0;
  for (int k = 0; k < n; k++) v = (v << 1) | ((m[(a + k) >> 3] >> (7 - ((a + k) & 7))) & 1);
  return v;
}
static int32_t getsbits(const uint8_t *m, int a, int n)
{
  uint32_t v = getbits(m, a, n);
  return (v & (1u << (n - 1))) ? (int32_t)(v - (1u << n)) : (int32_t)v;
}

int ais_nmea(const uint8_t *fr, int len, char chan, char *out, int outsz)
{
  uint8_t m[AIS_FRAME_MAX];
  int nb = len - 2; if (nb <= 0) return 0;
  for (int k = 0; k < nb; k++) m[k] = fr[k];
  int nbits = nb * 8, nch = (nbits + 5) / 6, pad = nch * 6 - nbits;
  char body[AIS_FRAME_MAX * 2 + 40];
  int p = snprintf(body, sizeof body, "AIVDM,1,1,,%c,", chan);
  for (int k = 0; k < nch && p < (int)sizeof body - 8; k++) {
    int a = k * 6; uint32_t v = 0;
    for (int j = 0; j < 6; j++) { int bi = a + j; v = (v << 1) | (bi < nbits ? ((m[bi >> 3] >> (7 - (bi & 7))) & 1) : 0); }
    v += 48; if (v > 87) v += 8;
    body[p++] = (char)v;
  }
  p += snprintf(body + p, sizeof body - p, ",%d", pad);
  uint8_t cs = 0; for (int k = 0; k < p; k++) cs ^= (uint8_t)body[k];
  return snprintf(out, outsz, "!%s*%02X", body, cs);
}

int ais_describe(const uint8_t *fr, int len, char *out, int outsz)
{
  uint8_t m[AIS_FRAME_MAX];
  int nb = len - 2; if (nb <= 0) return 0;
  for (int k = 0; k < nb; k++) m[k] = fr[k];
  int nbits = nb * 8;
  unsigned t = getbits(m, 0, 6), mmsi = getbits(m, 8, 30);
  if ((t >= 1 && t <= 3) && nbits >= 168) {
    double lon = getsbits(m, 61, 28) / 600000.0, lat = getsbits(m, 89, 27) / 600000.0;
    return snprintf(out, outsz, "type %u MMSI %09u %.5fN %.5fE SOG %.1f COG %.1f", t, mmsi,
                    lat, lon, getbits(m, 50, 10) / 10.0, getbits(m, 116, 12) / 10.0);
  }
  if (t == 18 && nbits >= 168) {
    double lon = getsbits(m, 57, 28) / 600000.0, lat = getsbits(m, 85, 27) / 600000.0;
    return snprintf(out, outsz, "type 18 MMSI %09u %.5fN %.5fE SOG %.1f", mmsi, lat, lon, getbits(m, 46, 10) / 10.0);
  }
  if (t == 5 && nbits >= 424) {
    char nm[21]; int j = 0;
    for (int k = 0; k < 20; k++) { unsigned v = getbits(m, 112 + 6 * k, 6); char c = (char)(v < 32 ? v + 64 : v); if (c != '@') nm[j++] = c; }
    while (j && nm[j - 1] == ' ') j--; nm[j] = 0;
    return snprintf(out, outsz, "type 5 MMSI %09u \"%s\"", mmsi, nm);
  }
  if (t == 21 && nbits >= 272) {                       /* AtoN (river km buoys) */
    char nm[21]; int j = 0;
    for (int k = 0; k < 20; k++) { unsigned v = getbits(m, 43 + 6 * k, 6); char c = (char)(v < 32 ? v + 64 : v); if (c != '@') nm[j++] = c; }
    while (j && nm[j - 1] == ' ') j--; nm[j] = 0;
    return snprintf(out, outsz, "type 21 AtoN %09u \"%s\"", mmsi, nm);
  }
  return snprintf(out, outsz, "type %u MMSI %09u (%d bits)", t, mmsi, nbits);
}

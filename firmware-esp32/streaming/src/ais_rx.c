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
  int32_t fc = r->fc_hz; int mm = r->mm_n;            /* keep tuning set before init */
  memset(r, 0, sizeof *r);
  r->fc_hz = fc; r->mm_n = mm;
  lut_init();
  r->sps = sps; r->chan = chan; r->cb = cb; r->ctx = ctx;
  r->nco_step = (uint32_t)(int32_t)llround(-(double)off_hz / (double)sps * 4294967296.0);

  if (!r->fc_hz) r->fc_hz = AIS_FC_DEFAULT;
  if (!r->mm_n || r->mm_n > AIS_MM_N) r->mm_n = AIS_MM_DEFAULT;
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

  r->rs_step = (uint32_t)(((uint64_t)sps << 16) / AIS_FS_OUT);   /* input samples per output, Q16 */
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
  int32_t old = r->mm_buf[r->mm_idx];
  r->mm_buf[r->mm_idx] = v;
  r->mm_sum += v - old;
  r->mm_idx = (r->mm_idx + 1) % r->mm_n;
  int32_t mid = r->mm_buf[(r->mm_idx + r->mm_n / 2) % r->mm_n];     /* delayed by N/2 */
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
    uint32_t ph = r->nco_ph; r->nco_ph += r->nco_step;
    int32_t s = s_sin_lut[ph >> 22], c = s_sin_lut[((ph >> 22) + 256) & 1023];
    int32_t mi = (i0 * c - q0 * s) >> 15;
    int32_t mq = (i0 * s + q0 * c) >> 15;

    r->xi[r->pos] = r->xi[r->pos + r->ch_n] = (int16_t)mi;
    r->xq[r->pos] = r->xq[r->pos + r->ch_n] = (int16_t)mq;
    if (++r->pos >= r->ch_n) r->pos = 0;
    const int16_t *xi = &r->xi[r->pos], *xq = &r->xq[r->pos];
    int32_t ai = 0, aq = 0;
    for (int k = 0; k < r->ch_n; k++) { ai += (int32_t)r->ch_h[k] * xi[k]; aq += (int32_t)r->ch_h[k] * xq[k]; }
    int16_t fi = (int16_t)((ai + 16384) >> 15), fq = (int16_t)((aq + 16384) >> 15);

    int32_t dr = (int32_t)fi * r->pi + (int32_t)fq * r->pq;
    int32_t di = (int32_t)fq * r->pi - (int32_t)fi * r->pq;
    r->pi = fi; r->pq = fq;
    int32_t a = fm_angle_q13(di, dr);                 /* +-PI_Q13 */

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
  return snprintf(out, outsz, "type %u MMSI %09u (%d bits)", t, mmsi, nbits);
}

/* SPDX-License-Identifier: MIT
 *
 * aprs_rx.cpp — AFSK1200 + AX.25 + APRS-IS  (ESP32-S3 / FG23 IQ)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * (LibAPRS delay-mult path + DC-block + mild IF AA — 2026-08-09)
 *
 * Design credit: the delay-multiply AFSK detector, bit PLL and HDLC window
 * logic follow the LibAPRS (Mark Qvist, GPL-3.0) / BertOS afsk.c (Develer,
 * GPL-2.0+exception) design, used as the reference during debugging. This
 * file is an independent fixed-point implementation for the FG23 I/Q path;
 * no code was copied. See ACKNOWLEDGEMENTS.md.
 *
 * DEBUG / CHANGELOG
 *   v2.3: s_audio_dc floor-shift -> rounded (+128). The ">>8" rounded the
 *         negative error downwards, so the "DC block" ADDED a constant +128
 *         DC level to the audio. Track2 (low deviation): 0 -> 75 packets.
 *         This was why only PER1000 worked and every other signal gave zero.
 *   v1 correlator path: SOFT_DC_SHIFT bug fixed earlier (38%->98% on one capture)
 *   v2.1: restore s_audio_dc in audio_sample (was dead); IF AA >>2 -> >>1
 *   v2.2: FIR product LPF (31-tap) — 1-pole matched PER but killed Track2
 *   v2: LibAPRS delay-multiply + phase window (PC twin esp_demod_fw_v2: 515/609
 *       Track2 IQ). Key fixes vs naive port:
 *         - discr output scaled to ~full int16 (quiet discr killed delay-multiply)
 *         - HDLC uses LibAPRS LEFT-shift window (right-shift + wrong stuff mask
 *           gave packets=0 / crc garbage on twin)
 *         - fractional resample IF -> 9600 Hz (integer 25000/9600 was 12500 Hz)
 *   AFC: not in this build (PC AFC still hurt 0 Hz IQ). Add later if needed.
 *
 * API unchanged: aprs_rx_init / aprs_rx_feed / aprs_rx_tick / stats getters
 */

#include "aprs_rx.h"
#include <Arduino.h>
#include <WiFi.h>
#include <string.h>
#include <math.h>

/* Raw, non-blocking sockets for the APRS-IS client — the same style as the
 * other TCP paths (send(fd, ..., MSG_DONTWAIT)). Async DNS in the tcpip
 * thread, so that name resolution does not stall the main loop either. */
#include <lwip/sockets.h>
#include <lwip/dns.h>
#include <lwip/tcpip.h>
#include <lwip/ip_addr.h>
#include <errno.h>
#include <fcntl.h>

#include "flashlog.h"

/* ==================== CONFIGURATION ==================== */

#include "station_config.h"   /* APRS_MYCALL, APRSIS_PASSCODE — git-ignored */
#define APRS_MYSSID      10          /* -10 is the customary iGate SSID */

/* APRS-IS. The passcode belongs to YOUR CALLSIGN.
 *
 *   -1  = READ-ONLY connection. The server accepts it but does not allow
 *         sending. Start with this: the log shows that decoding works while
 *         nothing is uploaded.
 *   own passcode = upload enabled (requires a valid licence).
 *
 * -1 is not a "trick" but the documented read-only mode of APRS-IS. */
#define APRSIS_HOST      "rotate.aprs2.net"
#define APRSIS_PORT      14580
#define APRSIS_FILTER    "r/47.5167/19.4333/50"   /* Dany, circle: lat/lon/km */
#define APRSIS_SOFTWARE  "FG23-SDR-iGate 1.0"
#define APRSIS_RETRY_MS  15000u

/* AFSK Bell 202 */
#define AFSK_BAUD        1200
#define AFSK_MARK_HZ     1200
#define AFSK_SPACE_HZ    2200

/* CHANNEL FILTER BEFORE the discriminator. Not cosmetic: demodulating a
 * 12.5 kHz FM channel in a 50 kHz band costs ~6 dB SNR, because the
 * discriminator includes the noise of the whole band. So the I/Q is first
 * brought down to ~25 ksps (= +-12.5 kHz), which is exactly one APRS
 * channel. Boxcar averaging, i.e. a few additions per sample. */
#define IF_TARGET_HZ     25000u

/* The audio rate decimated to afterwards. Around 12 kHz is good: ~10
 * samples/bit, ample for the correlator, and a quarter of the CPU compared
 * to the full rate. */
#define AUDIO_TARGET_HZ  9600u
#define MAX_TAPS         16          /* unused (LibAPRS path) */
#define AX25_MAX         330         /* the longest meaningful AX.25 frame */

/* Time constant of the slicer's self-centering threshold, 2^N samples.
 * 0 = THRESHOLD COMPLETELY OFF (the slicer compares against ZERO).
 *
 * ===== 2026-08-06: THIS WAS THE CAUSE OF THE 60% PACKET LOSS =====
 *
 * History: 10 (=1024 samples, 82 ms) was too fast, it locked onto the
 * preamble — so it became 13 (=8192 samples, 655 ms), "slower than one
 * packet". But 13 is NOT slow enough: a packet is ~500 ms, so the threshold
 * moves significantly DURING the packet. And since the mark/space ratio in
 * the data is NOT 50-50, the running average drifts towards whichever tone
 * is currently more frequent — i.e. the decision threshold follows the data
 * itself and skews the slicing.
 *
 * WHY IT IS UNNECESSARY AT ALL: its purpose (the frequency offset as a
 * constant term) is ALREADY handled by the s_audio_dc sink at the start of
 * audio_sample() (>>8, ~20 ms). By the time the correlator is reached the
 * signal is already centred, so the correct decision threshold is ZERO.
 *
 * MEASUREMENT (esp_demod.c — a bit-exact PC port of the whole signal path,
 * run on the same sample stream the firmware received):
 *
 *   input                                SOFT_DC_SHIFT 13   ->   0
 *   -------------------------------------------------------------------
 *   REAL FG23 recording (SpyServer,
 *     50 ksps, 140.8 s, 142 packets,
 *     -96 dBm at the chip)                    38.0 %        98.6 %
 *   synthetic, SNR 24 dB                      85.0 %       100.0 %
 *   synthetic, SNR 10 dB                      82.5 %       100.0 %
 *
 * The 38.0% reproduced the 37..38% measured on HARDWARE exactly — so the
 * plumbing (cur_sps, block feeding, timing) WAS CORRECT THROUGHOUT; the
 * fault sat solely in this one line. For comparison, on the same recording
 * the PC "gold" demod (wav_aprs_per.py) achieved 81.7%, so the fixed
 * variant is EVEN BETTER than that.
 *
 * DO NOT set it back to a non-zero value without re-running the above
 * bench. E.g. 11 gives 17.1% and 15 gives 56.3% on the same recording. */
#define SOFT_DC_SHIFT    0

/* ==================== FM DISCRIMINATOR ==================== */

#define PI_Q13  25736                /* 3.14159 * 8192 */

/* Approximate atan2 in Q13 radians. Error ~0.01 rad — ample for FM audio,
 * and it contains no floating-point operation. */
static inline int32_t atan2_q13(int32_t y, int32_t x)
{
  int32_t abs_y = (y < 0 ? -y : y) + 1;   /* the +1 guards against 0/0 */
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

/* ============ BUG OF THE DAY: OVERFLOWING atan2 (2026-08-09) ============
 *
 * Inside atan2_q13 above there is:   (x - abs_y) << 13
 * In int32 this is correct ONLY if |x| and |y| are small enough. The limit:
 *   |x - abs_y| <= 2^31 / 2^13 = 262 144
 * Since |x - abs_y| is |x| + |y| in the worst case, the safe bound for
 * both is about 131 000 — with margin, 65 535.
 *
 * The caller used to do:   atan2_q13(di >> 2, dr >> 2)
 * where dr = i*pi + q*pq  and  di = q*pi - i*pq, so |dr|,|di| <= A^2,
 * where A is the I/Q envelope. After the >>2, A^2/4 <= 262144, i.e.
 *
 *        ==>  FOR A > 1024 THE ATAN2 OVERFLOWS  <==
 *
 * MEASUREMENT (2026-08-09, on the recording passed through the chain,
 * scaled per level, compared to the exact arctan2):
 *
 *      level                |I/Q| peak     grossly wrong angle
 *      -----------------------------------------------------
 *      -94 dBm (recording)        977            0.0 %
 *      +6 dB                     1954          100.0 %
 *      +12 dB                    3907           99.8 %
 *      +24 dB (handheld)        15629           99.8 %
 *
 * THIS EXPLAINS THE WHOLE DAY:
 *   - PER1000 works at -100 dBm, because there the peak is ~411 (6x margin);
 *   - Track2 at -94 dBm peaks at 993..1036 — RIGHT at the edge of the cliff;
 *   - anything STRONGER (handheld radio, nearby station) is 100 % garbage;
 *   - the PC twin uses DOUBLE atan2, so it never showed up there.
 * I.e. the stronger the signal, the more surely it fails to decode — the
 * exact opposite of what one intuitively looks for.
 *
 * THE FIX. atan2 is SCALE-INDEPENDENT: shifting dr and di by the SAME
 * amount leaves the angle unchanged. So normalise to 16 bits, done.
 * Cost: one clz and two shifts per sample. */
static inline int32_t fm_angle_q13(int32_t di, int32_t dr)
{
  int32_t ax = (dr < 0) ? -dr : dr;
  int32_t ay = (di < 0) ? -di : di;
  uint32_t m = (uint32_t)((ax > ay) ? ax : ay);
  int sh = 0;
  if (m > 65535u) sh = 16 - __builtin_clz(m);   /* (m>>sh) <= 65535 */
  return atan2_q13(di >> sh, dr >> sh);
}

/* ==================== STATE ==================== */

static uint32_t s_sps = 0;           /* input I/Q rate */
static uint32_t s_if_hz = 0;         /* I/Q rate after channel filtering */
static uint32_t s_audio_hz = 0;      /* rate after audio decimation */
static uint32_t s_iq_dec = 1;
static uint32_t s_decim = 1;

/* I/Q channel filter (boxcar) */
static int32_t  s_iq_si = 0, s_iq_sq = 0;
static uint32_t s_iq_n = 0;

/* FM discriminator */
static int16_t  s_prev_i = 0, s_prev_q = 0;

/* decimation: two cascaded boxcars (triangular window) */
static int32_t  s_box1 = 0, s_box2 = 0;
static float s_rs_acc = 0.f, s_rs_ratio = 1.f;
static uint32_t s_box_n = 0;

/* audio DC block */
static int32_t  s_audio_dc = 0;

/* correlator */
static int8_t   s_cos_m[MAX_TAPS], s_sin_m[MAX_TAPS];
static int8_t   s_cos_s[MAX_TAPS], s_sin_s[MAX_TAPS];
static int16_t  s_hist[64];
static uint32_t s_hist_idx = 0;
static uint32_t s_taps = 10;

/* slicer + PLL */
static int32_t  s_soft = 0;          /* filtered correlator difference */
static int32_t  s_soft_dc = 0;       /* self-centering decision threshold */
static bool     s_slice = false, s_slice_prev = false;
static int32_t  s_pll = 0, s_pll_inc = 0, s_pll_adj = 0;
#define PLL_MAX  0x10000

/* NRZI + HDLC */
static bool     s_nrzi_prev = false;

/* LibAPRS delay-multiply state (audio @ ~9600 Hz) */
#define PHASE_BITS       8
#define PHASE_INC        1
#define PHASE_MAX_A      64
#define PHASE_THRESH_A   32
#define DELAYED_N_MAX    8
#define FIR_LPF_N        31   /* 8*4-1, same as PC twin — 1-pole killed Track2 609->5 */
#define BITS_DIFFER(a, b)       (((a) ^ (b)) & 0x01)
#define DUAL_XOR(a, b)          ((((a) ^ (b)) & 0x03) == 0x03)
#define SIGNAL_TRANSITIONED(b)  DUAL_XOR((b), (b) >> 2)
#define TRANSITION_FOUND(b)     BITS_DIFFER((b), (b) >> 1)
static int     s_delayed[DELAYED_N_MAX];
static int     s_delay_idx = 0;
static int     s_delayed_n = 4;
static uint8_t s_sampledBits = 0;
static uint8_t s_actualBits = 0;
static int     s_currentPhase = 0;
static int32_t s_lpf_y = 0;

/* ==== SWITCH: 1 = correlator + AGC,  0 = LibAPRS delay-multiply + FIR
 *
 * DEFAULT 0 — A DELIBERATE STEP BACK (evening of 2026-08-09).
 *
 * The correlator was written because the delay-multiply output scales with
 * the square of the deviation. The reasoning is correct, but the
 * MEASUREMENT does not support it: in the same test framework, on the same
 * tnc_track2 file
 *
 *      delay-multiply + FIR :  CRC-correct 96
 *      correlator + AGC     :  CRC-correct 83..85
 *
 * i.e. the correlator is WORSE, not better. In addition, the time constant
 * of the two separate AGCs is >>10 = 107 ms, which corrupts the first
 * quarter of a ~500 ms packet while it settles.
 *
 * A METHODOLOGICAL MISTAKE was made: a PROVEN fix (the atan2 overflow
 * below) and an UNPROVEN one (this) were released together. Exactly what
 * had been criticised all day about the divergence of the twin and the
 * firmware. Hence the switch defaults to 0: keep the proven fix, drop the
 * uncertain change. One variable at a time.
 *
 * The correlator stays in the code because the principle is sound, and if
 * the bench ever shows it is needed — set to 1 and it is back. */
#define APRS_DEMOD_CORRELATOR 0

#define COR_N 8                      /* one bit @ 9600/1200 */
static const int8_t COS_M[COR_N] = { 127,  90,    0, -90, -127, -90,    0,  90 };
static const int8_t SIN_M[COR_N] = {   0,  90,  127,  90,    0, -90, -127, -90 };
/* 2200 Hz @ 9600: phase step 2*pi*2200/9600 = 1.4399 rad/sample.
 * The values are round(127*cos/sin(k*1.4399)) — NOT estimated by hand but
 * computed and verified through the whole signal path (see below). */
static const int8_t COS_S[COR_N] = { 127,  17, -123, -49,  110,  77,  -90, -101 };
static const int8_t SIN_S[COR_N] = {   0, 126,   33, -117, -63, 101,   90,  -77 };

/* VERIFIED 2026-08-09: this fixed-point signal path (with these same int8
 * coefficients, the same >>8 magnitude scaling, >>10 AGC and Q12 ratio)
 * yields 176 frame starts and 83 CRC-CORRECT packets from
 * tnc_track2_iq_3min_0hz.wav. So the arithmetic and the table are right;
 * any further degradation is the signal, not the computation. */
static int16_t s_cor_x[COR_N];
static uint32_t s_cor_idx = 0;
static int32_t  s_agc_m = 0, s_agc_s = 0;
/* Product FIR (twin parity). Static = no heap churn on S3. */
static int16_t s_fir_an[FIR_LPF_N];
static int16_t s_fir_x[FIR_LPF_N];
static int     s_fir_idx = 0;
static bool    s_fir_ready = false;
static int popcount5(uint8_t bits) {
  int c = 0; for (int i = 0; i < 5; i++) c += (bits >> i) & 1; return c;
}
static float sincf_local(float x) {
  if (fabsf(x) < 1e-6f) return 1.0f;
  return sinf(x) / x;
}
static float fir_window(float x) {
  if (fabsf(x) >= (float)M_PI) return 0.0f;
  return 0.42f + 0.5f * cosf(x) + 0.08f * cosf(2.0f * x);
}
static void fir_design(int fs, int cutoff) {
  const int M = (FIR_LPF_N - 1) / 2;
  const int A = (1 << 15) - 1;
  float Rc = (float)cutoff / (float)fs;
  for (int n = -M; n <= M; n++) {
    s_fir_an[n + M] = (int16_t)(A * 2.0f * Rc * sincf_local(2.0f * (float)M_PI * Rc * (float)n) *
                                fir_window((float)M_PI * (float)n / (float)M));
  }
  memset(s_cor_x, 0, sizeof s_cor_x);   /* correlator delay line */
  s_cor_idx = 0;
  s_agc_m = s_agc_s = 0;               /* per-tone AGC */
  memset(s_fir_x, 0, sizeof s_fir_x);
  s_fir_idx = 0;
  s_fir_ready = true;
}
static int fir_run(int16_t value) {
  int64_t sum = 0;
  s_fir_x[s_fir_idx] = value;
  for (int i = 0; i < FIR_LPF_N; i++)
    sum += (int64_t)s_fir_an[i] * (int64_t)s_fir_x[(s_fir_idx + i) % FIR_LPF_N];
  s_fir_idx += FIR_LPF_N - 1;
  s_fir_idx %= FIR_LPF_N;
  return (int)(sum >> 16);
}
static uint8_t  s_hdlc_win = 0;      /* rolling bit window */
static uint8_t  s_hdlc_byte = 0;
static uint8_t  s_hdlc_nbit = 0;
static bool     s_in_frame = false;
static uint8_t  s_frame[AX25_MAX];
static uint16_t s_frame_len = 0;

/* statistics */
static uint32_t s_frames = 0, s_bad = 0, s_gated = 0;
static char     s_last[256] = "";
static char     s_last_call[12] = "";
static char     s_last_info[128] = "";
static uint32_t s_last_ms = 0;

/* APRS-IS — the rest of the non-blocking state machine is in the section below. */
static bool     s_is_online   = false;
static uint32_t s_is_last_try = 0;   /* time of the last connection attempt/error */

/* ==================== FILTER SETUP ==================== */

static void aprs_rx_configure(uint32_t sps)
{
  s_sps = sps;

  s_iq_dec = sps / IF_TARGET_HZ;
  if (s_iq_dec < 1) s_iq_dec = 1;
  s_if_hz = sps / s_iq_dec;

  /* fractional IF -> 9600 (integer 25000/9600 was 12500 — off for phase window) */
  s_decim = 1; /* unused in frac path */
  s_audio_hz = AUDIO_TARGET_HZ;
  s_rs_ratio = (float)s_if_hz / (float)AUDIO_TARGET_HZ;
  s_rs_acc = 0.f;

  /* samples/bit at the audio rate. Need NOT be an integer: the PLL locks
   * with a fractional bit length too, so the rate is not forced to a round
   * value. */
  uint32_t taps = (s_audio_hz + AFSK_BAUD / 2) / AFSK_BAUD;
  if (taps < 4)        taps = 4;
  if (taps > MAX_TAPS) taps = MAX_TAPS;
  s_taps = taps;

  for (uint32_t k = 0; k < s_taps; k++) {
    double tm = 2.0 * M_PI * AFSK_MARK_HZ  * (double)k / (double)s_audio_hz;
    double ts = 2.0 * M_PI * AFSK_SPACE_HZ * (double)k / (double)s_audio_hz;
    s_cos_m[k] = (int8_t)lrint(127.0 * cos(tm));
    s_sin_m[k] = (int8_t)lrint(127.0 * sin(tm));
    s_cos_s[k] = (int8_t)lrint(127.0 * cos(ts));
    s_sin_s[k] = (int8_t)lrint(127.0 * sin(ts));
  }

  /* PLL: exactly one wrap-around per bit. */
  s_pll_inc = (int32_t)(((uint64_t)PLL_MAX * AFSK_BAUD) / s_audio_hz);
  s_pll_adj = s_pll_inc / 4;         /* loop gain */
  if (s_pll_adj < 1) s_pll_adj = 1;

  memset(s_hist, 0, sizeof s_hist);
  s_hist_idx = 0;
  s_box1 = s_box2 = 0; s_box_n = 0;
  s_iq_si = s_iq_sq = 0; s_iq_n = 0;
  s_audio_dc = 0; s_soft = 0; s_soft_dc = 0;
  s_pll = 0;
  /* LibAPRS delay line: ~ half bit @ 9600 -> 4 samples */
  s_delayed_n = 4; /* half-bit @ 9600, matches PC twin */
  memset(s_delayed, 0, sizeof s_delayed);
  s_delay_idx = 0;
  s_sampledBits = s_actualBits = 0;
  s_currentPhase = 0;
  s_lpf_y = 0;
  fir_design((int)AUDIO_TARGET_HZ, 1200); /* twin FIR_LPF_N @ 9600 */

  Serial0.printf("APRS RX: %lu sps --/%lu--> %lu Hz IF (+-%lu.%lu kHz) "
                 "--/%lu--> %lu Hz audio, LibAPRS delay-mult N=%d taps=%lu\n",
                 (unsigned long)sps, (unsigned long)s_iq_dec,
                 (unsigned long)s_if_hz,
                 (unsigned long)(s_if_hz / 2000),
                 (unsigned long)((s_if_hz / 200) % 10),
                 (unsigned long)s_decim,
                 (unsigned long)s_audio_hz, s_delayed_n, (unsigned long)s_taps);
}

/* ==================== CRC (X.25 FCS) ==================== */

static inline uint16_t crc_update(uint16_t crc, uint8_t b)
{
  b ^= (uint8_t)(crc & 0xFF);
  b ^= (uint8_t)(b << 4);
  return (uint16_t)(((uint16_t)b << 8) | (crc >> 8))
       ^ (uint16_t)(b >> 4)
       ^ ((uint16_t)b << 3);
}

/* ==================== AX.25 -> TNC2 ==================== */

/* Print one 7-byte address field. Returns: the "end" bit (last address). */
static bool addr_to_text(const uint8_t *a, char *out, size_t outsz, bool star)
{
  char call[10];
  int n = 0;
  for (int i = 0; i < 6; i++) {
    char c = (char)(a[i] >> 1);
    if (c != ' ') call[n++] = c;
  }
  call[n] = '\0';
  int ssid = (a[6] >> 1) & 0x0F;
  bool last = (a[6] & 0x01) != 0;
  bool rpt  = (a[6] & 0x80) != 0;      /* H bit: already repeated by a digi */

  if (ssid) snprintf(out, outsz, "%s-%d%s", call, ssid,
                     (star && rpt) ? "*" : "");
  else      snprintf(out, outsz, "%s%s", call, (star && rpt) ? "*" : "");
  return last;
}

/* Builds TNC2 text from the frame: SRC>DEST,DIGI1*,DIGI2:info
 * Returns: success. */
static bool frame_to_tnc2(const uint8_t *f, uint16_t len,
                          char *out, size_t outsz)
{
  if (len < 16) return false;          /* 2 addresses + ctrl + pid + FCS */

  char dest[12], src[12], digi[12];
  addr_to_text(&f[0], dest, sizeof dest, false);
  bool last = addr_to_text(&f[7], src, sizeof src, false);

  size_t p = 0;
  int w = snprintf(out, outsz, "%s>%s", src, dest);
  if (w < 0) return false;
  p = (size_t)w;

  uint16_t off = 14;
  int guard = 8;                       /* AX.25: at most 8 digis */
  while (!last && off + 7 <= len && guard--) {
    last = addr_to_text(&f[off], digi, sizeof digi, true);
    w = snprintf(out + p, outsz - p, ",%s", digi);
    if (w < 0 || (size_t)w >= outsz - p) return false;
    p += (size_t)w;
    off += 7;
  }
  if (!last) return false;             /* the address field did not terminate */

  /* control + PID: for APRS always a UI frame (0x03) and 0xF0. Anything
   * else is not APRS — and must not be uploaded. */
  if (off + 2 > len) return false;
  if (f[off] != 0x03 || f[off + 1] != 0xF0) return false;
  off += 2;

  if (len < 2 || off > (uint16_t)(len - 2)) return false;
  uint16_t infolen = (uint16_t)(len - 2 - off);   /* the 2-byte FCS subtracted */

  if (p + 1 + infolen + 1 > outsz) infolen = (uint16_t)(outsz - p - 2);
  out[p++] = ':';
  for (uint16_t i = 0; i < infolen; i++) {
    uint8_t c = f[off + i];
    out[p++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
  }
  out[p] = '\0';
  return true;
}

/* ==================== APRS-IS (non-blocking) ==================== */

/* NO phase of the connection stalls the main loop. The old
 * WiFiClient::connect(...,3000) stalled for 140 ms (3 s on error) in a
 * single transaction -> RF block loss (boot log: aprs=140124 us, d=20
 * holes). Instead, a state machine:
 *   DNS       : async lwIP dns_gethostbyname in the tcpip thread (tcpip_callback),
 *   connect   : O_NONBLOCK socket, EINPROGRESS, select() with 0 timeout,
 *   login/gate: send(fd, MSG_DONTWAIT) — like the other TCP paths,
 *   read      : recv(fd, MSG_DONTWAIT), with a cost budget.
 * After error/disconnect, exponential backoff (3..60 s), reset on success. */

/* --- async DNS. Written from the tcpip thread, read from the main loop.
 * The two ESP32-S3 cores see the SAME internal SRAM (no separate data
 * cache), so volatile suffices here: the callback writes the IP first and
 * the flag LAST, and the reader checks the flag first. --- */
enum { DNS_IDLE = 0, DNS_REQ, DNS_WAIT, DNS_DONE, DNS_FAIL };
static volatile int      s_dns_state = DNS_IDLE;
static volatile uint32_t s_dns_ip4   = 0;      /* network byte order */

static void aprsis_dns_cb(const char *name, const ip_addr_t *ip, void *arg)
{
  (void)name; (void)arg;
  if (ip && IP_IS_V4(ip)) {
    s_dns_ip4   = ip4_addr_get_u32(ip_2_ip4(ip));
    s_dns_state = DNS_DONE;
  } else {
    s_dns_state = DNS_FAIL;
  }
}

/* Runs in the tcpip thread (scheduled there by tcpip_callback) — the raw
 * lwIP DNS API may ONLY be called from here. */
static void aprsis_dns_start(void *arg)
{
  (void)arg;
  ip_addr_t addr;
  err_t e = dns_gethostbyname_addrtype(APRSIS_HOST, &addr, aprsis_dns_cb,
                                       NULL, LWIP_DNS_ADDRTYPE_IPV4);
  if (e == ERR_OK) {                    /* from cache, immediately done */
    if (IP_IS_V4(&addr)) { s_dns_ip4   = ip4_addr_get_u32(ip_2_ip4(&addr));
                           s_dns_state = DNS_DONE; }
    else                   s_dns_state = DNS_FAIL;
  } else if (e == ERR_INPROGRESS) {     /* the callback will arrive later */
    s_dns_state = DNS_WAIT;
  } else {
    s_dns_state = DNS_FAIL;
  }
}

/* --- state --- */
enum aprsis_state { AIS_IDLE = 0, AIS_DNS, AIS_CONNECTING, AIS_LOGIN, AIS_ONLINE };
static aprsis_state s_ais         = AIS_IDLE;
static int          s_is_fd       = -1;
static uint32_t     s_phase_start = 0;   /* start of the current phase (timeout) */

#define APRSIS_BACKOFF_MIN  3000u
#define APRSIS_BACKOFF_MAX  60000u
#define APRSIS_DNS_TMO_MS   8000u
#define APRSIS_CONN_TMO_MS  6000u
static uint32_t s_backoff = APRSIS_BACKOFF_MIN;

/* login buffer: partial sends are tracked too, so that a half line never
 * remains in the stream (the "a partial unit kills" lesson). */
static char     s_login[176];
static uint16_t s_login_len  = 0;
static uint16_t s_login_sent = 0;

static void aprsis_close(void)
{
  if (s_is_fd >= 0) { close(s_is_fd); s_is_fd = -1; }
  s_is_online = false;
  s_dns_state = DNS_IDLE;
  s_ais       = AIS_IDLE;
}

static void aprsis_fail(const char *why)
{
  if (why) Serial0.printf("APRS-IS: %s -> retry in %lu s\n",
                          why, (unsigned long)(s_backoff / 1000));
  aprsis_close();
  s_is_last_try = millis();              /* the backoff counts from the ERROR */
  s_backoff *= 2;
  if (s_backoff > APRSIS_BACKOFF_MAX) s_backoff = APRSIS_BACKOFF_MAX;
}

static void aprsis_prepare_login(void)
{
  int n = snprintf(s_login, sizeof s_login,
                   "user %s-%d pass %d vers %s filter %s\r\n",
                   APRS_MYCALL, APRS_MYSSID, APRSIS_PASSCODE,
                   APRSIS_SOFTWARE, APRSIS_FILTER);
  if (n < 0) n = 0;
  if (n > (int)sizeof s_login) n = (int)sizeof s_login;
  s_login_len  = (uint16_t)n;
  s_login_sent = 0;
}

/* May this packet be uploaded? The APRS-IS etiquette. */
static bool gate_allowed(const char *tnc2)
{
  if (strstr(tnc2, "TCPIP")) return false;   /* already been on the internet */
  if (strstr(tnc2, "TCPXX")) return false;
  if (strstr(tnc2, "NOGATE")) return false;  /* the sender explicitly forbids it */
  if (strstr(tnc2, "RFONLY")) return false;
  const char *colon = strchr(tnc2, ':');
  if (colon && colon[1] == '}') return false; /* third-party frame */
  return true;
}

static void aprsis_send(const char *tnc2)
{
  if (!s_is_online || s_is_fd < 0) return;
  if (APRSIS_PASSCODE < 0) return;           /* read-only mode: no sending */
  if (!gate_allowed(tnc2)) {
    Serial0.println("APRS-IS: not gateable (TCPIP/NOGATE/RFONLY/3rd party)");
    return;
  }

  /* qAR marks that it was received over radio and uploaded by us. */
  const char *colon = strchr(tnc2, ':');
  if (!colon) return;
  int head = (int)(colon - tnc2);
  if (head > 200) return;
  char line[300];
  int n = snprintf(line, sizeof line, "%.*s,qAR,%s-%d%s\r\n",
                   head, tnc2, APRS_MYCALL, APRS_MYSSID, colon);
  if (n < 0) return;
  if (n >= (int)sizeof line) return;   /* truncated: do NOT send a half line
                                        * without \r\n, it corrupts the IS stream */

  /* A single non-blocking send. An APRS line is a few hundred bytes, the
   * send buffer absorbs it easily. If not: EAGAIN -> nothing went out, the
   * stream is intact, only this packet is dropped (APRS is a lossy
   * protocol). Partial or other error -> the stream may be corrupted,
   * disconnect and reconnect. */
  int w = send(s_is_fd, line, n, MSG_DONTWAIT);
  if (w == n) {
    s_gated++;
  } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    Serial0.println("APRS-IS: send buffer full, packet dropped");
  } else {
    aprsis_fail("partial/failed gate send");
  }
}

/* The lines coming from the server MUST be read, otherwise the receive
 * buffer fills up and the connection stalls. Non-blocking, with a per-loop
 * cost cap — even a sudden data flood must not stop the pump. recv==0 =
 * closed by the peer. */
static void aprsis_drain(void)
{
  uint8_t sink[256];
  int budget = 2048;
  while (budget > 0) {
    int n = recv(s_is_fd, sink, sizeof sink, MSG_DONTWAIT);
    if (n > 0) { budget -= n; continue; }
    if (n == 0) { aprsis_fail("server closed the connection"); return; }
    if (errno == EAGAIN || errno == EWOULDBLOCK) return;   /* no more data */
    aprsis_fail("read error"); return;
  }
}

/* The complete non-blocking state machine. Runs once per main loop and
 * performs at most ONE non-blocking socket operation per phase. */
static void aprsis_service(uint32_t now)
{
  if (WiFi.status() != WL_CONNECTED) {   /* no WiFi: disconnect, wait */
    if (s_ais != AIS_IDLE || s_is_fd >= 0) aprsis_close();
    return;
  }

  switch (s_ais) {

  case AIS_IDLE:
    if (s_is_last_try != 0 && now - s_is_last_try < s_backoff) return;
    s_dns_state = DNS_REQ;
    if (tcpip_callback(aprsis_dns_start, NULL) != ERR_OK) {
      aprsis_fail("DNS scheduling failed");
      return;
    }
    s_phase_start = now;
    s_ais = AIS_DNS;
    return;

  case AIS_DNS:
    if (s_dns_state == DNS_FAIL)                { aprsis_fail("name resolution error"); return; }
    if (now - s_phase_start > APRSIS_DNS_TMO_MS) { aprsis_fail("DNS timeout");  return; }
    if (s_dns_state != DNS_DONE) return;        /* still waiting for resolution */
    {
      s_is_fd = socket(AF_INET, SOCK_STREAM, 0);
      if (s_is_fd < 0) { aprsis_fail("socket() error"); return; }
      int fl = fcntl(s_is_fd, F_GETFL, 0);
      fcntl(s_is_fd, F_SETFL, fl | O_NONBLOCK);
      int one = 1;
      setsockopt(s_is_fd, IPPROTO_TCP, TCP_NODELAY,  &one, sizeof one);
      setsockopt(s_is_fd, SOL_SOCKET,  SO_KEEPALIVE, &one, sizeof one);

      struct sockaddr_in sa;
      memset(&sa, 0, sizeof sa);
      sa.sin_family      = AF_INET;
      sa.sin_port        = htons(APRSIS_PORT);
      sa.sin_addr.s_addr = s_dns_ip4;

      int r = connect(s_is_fd, (struct sockaddr *)&sa, sizeof sa);
      if (r == 0) {                             /* rare: done immediately */
        aprsis_prepare_login();
        s_ais = AIS_LOGIN;
      } else if (errno == EINPROGRESS) {
        s_phase_start = now;
        s_ais = AIS_CONNECTING;
      } else {
        aprsis_fail("connect() error");
      }
    }
    return;

  case AIS_CONNECTING:
    {
      if (now - s_phase_start > APRSIS_CONN_TMO_MS) { aprsis_fail("connect timeout"); return; }
      fd_set wf; FD_ZERO(&wf); FD_SET(s_is_fd, &wf);
      struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
      int s = select(s_is_fd + 1, NULL, &wf, NULL, &tv);
      if (s <= 0) return;                       /* not writable yet */
      int soe = 0; socklen_t sl = sizeof soe;
      getsockopt(s_is_fd, SOL_SOCKET, SO_ERROR, &soe, &sl);
      if (soe != 0) { aprsis_fail("connect refused"); return; }
      aprsis_prepare_login();
      s_ais = AIS_LOGIN;
    }
    return;

  case AIS_LOGIN:
    while (s_login_sent < s_login_len) {
      int w = send(s_is_fd, s_login + s_login_sent,
                   s_login_len - s_login_sent, MSG_DONTWAIT);
      if (w > 0) { s_login_sent += (uint16_t)w; continue; }
      if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return; /* later */
      aprsis_fail("login send error");
      return;
    }
    s_is_online = true;
    s_backoff   = APRSIS_BACKOFF_MIN;           /* success -> reset backoff */
    s_ais       = AIS_ONLINE;
    Serial0.printf("APRS-IS: connected to %s:%d as %s-%d (passcode %d%s)\n",
                   APRSIS_HOST, APRSIS_PORT, APRS_MYCALL, APRS_MYSSID,
                   APRSIS_PASSCODE, (APRSIS_PASSCODE < 0) ? ", READ-ONLY" : "");
    return;

  case AIS_ONLINE:
    aprsis_drain();
    return;
  }
}

/* ==================== HDLC ==================== */

static void frame_complete(void)
{
  if (s_frame_len < 17) { s_frame_len = 0; return; }

  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < s_frame_len; i++) crc = crc_update(crc, s_frame[i]);

  /* Run over the whole frame (including the FCS) the remainder is a fixed
   * value. This is the usual X.25 check. */
  if (crc != 0xF0B8) {
    s_bad++;
#if FLASHLOG_ENABLE
    /* a sample of every 64th CRC error with its length — shows whether the
     * failures are near-complete frames (weak real packet) or short (noise). */
    if ((s_bad & 63) == 0)
      flashlog_printf("BAD n=%lu len=%u", (unsigned long)s_bad,
                      (unsigned)s_frame_len);
#endif
    s_frame_len = 0;
    return;
  }

  s_frames++;
  char tnc2[256];
  if (frame_to_tnc2(s_frame, s_frame_len, tnc2, sizeof tnc2)) {
    strncpy(s_last, tnc2, sizeof s_last - 1);
    s_last[sizeof s_last - 1] = '\0';

    /* Split for the display: the callsign is before the '>', the info
     * after the first ':'. The display needs these two parts, not the whole
     * line. */
    const char *gt = strchr(tnc2, '>');
    size_t cl = gt ? (size_t)(gt - tnc2) : 0;
    if (cl >= sizeof s_last_call) cl = sizeof s_last_call - 1;
    memcpy(s_last_call, tnc2, cl);
    s_last_call[cl] = '\0';

    const char *cn = strchr(tnc2, ':');
    strncpy(s_last_info, cn ? cn + 1 : "", sizeof s_last_info - 1);
    s_last_info[sizeof s_last_info - 1] = '\0';
    s_last_ms = millis();
    Serial0.printf("\nAPRS RX: %s\n", tnc2);
#if FLASHLOG_ENABLE
    flashlog_printf("RX %s", tnc2);
#endif
    aprsis_send(tnc2);
  } else {
    Serial0.println("\nAPRS RX: valid frame, but not APRS UI — skipped");
  }
  s_frame_len = 0;
}

/* One NRZI-decoded bit for the HDLC. */
/* LibAPRS hdlcParse — LEFT-shift window (matches esp_demod_fw_v2 / 515 pkts) */
static void hdlc_bit(bool bit)
{
  s_hdlc_win = (uint8_t)((s_hdlc_win << 1) | (bit ? 1u : 0u));

  if (s_hdlc_win == 0x7E) {
    if (s_in_frame && s_frame_len >= 17)
      frame_complete();
    s_in_frame = true;
    s_frame_len = 0;
    s_hdlc_byte = 0;
    s_hdlc_nbit = 0;
    return;
  }
  if ((s_hdlc_win & 0xFE) == 0xFE) { /* seven 1s: abort */
    s_in_frame = false;
    s_frame_len = 0;
    s_hdlc_nbit = 0;
    return;
  }
  if (!s_in_frame) return;
  if ((s_hdlc_win & 0x3F) == 0x3E) return; /* stuffed 0 after five 1s */

  if (s_hdlc_win & 0x01)
    s_hdlc_byte |= 0x80;

  s_hdlc_nbit++;
  if (s_hdlc_nbit >= 8) {
    if (s_frame_len < AX25_MAX)
      s_frame[s_frame_len++] = s_hdlc_byte;
    else {
      s_in_frame = false;
      s_frame_len = 0;
    }
    s_hdlc_byte = 0;
    s_hdlc_nbit = 0;
  } else {
    s_hdlc_byte >>= 1;
  }
}

/* ==================== AUDIO -> BIT ==================== */

static void audio_sample(int32_t a)
{
  /* a: already scaled discriminator audio, roughly full int16 range */
  /* Audio DC-block: remove FM discr carrier offset (>>8 => fc~6 Hz @ 9600).
   * Critical for real TX offset; VSG at exact LO has DC~0 so bench hid this.
   *
   * ===== 2026-08-09: THE "+128" IS NOT COSMETIC, IT WAS THE SECOND BUG OF THE DAY =====
   *
   * The old line was:        s_audio_dc += (a - s_audio_dc) >> 8;
   * The arithmetic right shift rounds NEGATIVE numbers DOWN (floor), so the
   * average step is not (e/256) but (e/256 - 0.5). The equilibrium is
   * therefore NOT at mean(a) but at mean(a) - 128 — i.e. the "DC block"
   * ADDS A CONSTANT +128 DC LEVEL TO THE AUDIO instead of removing it.
   * (Measured: mean(if_lpf) = -0.1, s_audio_dc = -127.7.)
   *
   * Why it kills: after the delay-multiply a DC level D contributes a
   * constant +D^2 term AND a 2*D*a(t) cross product, which the 1200 Hz FIR
   * largely passes. The audio peak on Track2 is ONLY ~1100 (the recording's
   * peak deviation is p99 = 907 Hz instead of the standard +-3000), so 128
   * is 11 % of the audio, and the cross product is ~45 % ripple at the
   * decision level. On PER1000 the same 128 is only ~2 %, because THAT was
   * generated with full deviation — which is why PER1000 worked all the way
   * from -111 to -50 dBm, and why Track2 and the handheld gave ZERO AT
   * EVERY LEVEL.
   *
   * MEASUREMENT (fwport = the bit-exact PC port of this file,
   * tnc_track2_iq_3min_0hz.wav resampled to 50 kSps, CRC-correct packets;
   * "dev" is the multiplier of the recording's deviation):
   *
   *      dev    old (>>8 floor)    new (+128)   WITHOUT DC block
   *      ---------------------------------------------------------
   *      1x            0              75              77
   *      2x           46              74              72
   *      4x           75              73              73
   *      6x           83              --              75
   *
   *   and with carrier offset, at 1x deviation:
   *      offset     old     new   without
   *      ------------------------------------
   *      0 Hz         0     75      77
   *      500 Hz       0     75       0
   *      1000 Hz      0     76       0
   *      2000 Hz      0     76       0
   *      3000 Hz      0     69       0
   *
   * I.e.: the DC block IS NEEDED (without it a 500 Hz offset already gives
   * zero), but it only works when ROUNDED. The fixed variant gives 71..79
   * packets over the whole deviation x offset matrix —
   * deviation-independent, as expected from an FM discriminator.
   *
   * The same kind of (scaled - if_lpf) >> 1 at the IF would shift only
   * -1 LSB, and measured it changes NOTHING (75/85 either way) — so it was
   * DELIBERATELY NOT touched. One variable, one fix. */
  s_audio_dc += (a - s_audio_dc + 128) >> 8;
  a -= s_audio_dc;

  if (a > 32767) a = 32767;
  if (a < -32768) a = -32768;
  int16_t cur = (int16_t)a;

#if APRS_DEMOD_CORRELATOR
  /* ============ MARK/SPACE CORRELATOR, PER-TONE AGC ============
   *
   * WHY THIS, AND NOT THE DELAY-MULTIPLY (2026-08-09).
   *
   * The delay-multiply output is proportional to the SQUARE of the audio
   * AMPLITUDE, and for FM the amplitude = the DEVIATION. The slicer
   * compares against zero, so the decision margin also scales with the
   * square of the deviation.
   *
   * MEASUREMENT (2026-08-09, recording passed through the chain vs. the
   * original file):
   *
   *   original Track2 IQ file  : MARK 26 dB above noise  -> decodes
   *   the same through the chain: MARK  9 dB above noise  -> NOTHING
   *
   * and the file's peak deviation is only +-675 Hz instead of the standard
   * +-3000. The FM demodulated SNR scales with the square of the deviation,
   * so this alone is ~13 dB — exactly what is missing. PER1000 works down
   * to -110 dBm because it was generated with correct deviation.
   *
   * THE CORRELATOR ELIMINATES THIS. MARK and SPACE energy are measured
   * separately over a one-bit window, BOTH normalised by their OWN AGC,
   * and the DIFFERENCE of the two is the soft decision. Thus:
   *   - the absolute deviation drops out (both branches scale the same),
   *   - signal level changes drop out,
   *   - the decision threshold really is zero, with constant margin.
   * This is the architecture with which the v1 path achieved 98.6 % on a
   * REAL FG23 recording — it was replaced by the delay-multiply because
   * that scored higher on a file, and that was exactly the mistake: a file
   * has no noise.
   *
   * Bit-clock recovery is UNCHANGED (the twin's proven phase window), and
   * so is the HDLC. Only the soft-decision generation differs. */
  {
    /* ring over the last COR_N samples */
    s_cor_x[s_cor_idx] = cur;
    uint32_t j = s_cor_idx;
    int32_t mi = 0, mq = 0, si = 0, sq = 0;
    for (int k = 0; k < COR_N; k++) {
      int16_t v = s_cor_x[j];
      mi += (int32_t)v * COS_M[k];  mq += (int32_t)v * SIN_M[k];
      si += (int32_t)v * COS_S[k];  sq += (int32_t)v * SIN_S[k];
      if (j == 0) j = COR_N - 1; else j--;
    }
    if (++s_cor_idx == COR_N) s_cor_idx = 0;

    /* magnitude ~ |I|+|Q| (cheap, error <12 %, and it cancels in the ratio) */
    int32_t mm = (mi < 0 ? -mi : mi) + (mq < 0 ? -mq : mq);
    int32_t ss = (si < 0 ? -si : si) + (sq < 0 ? -sq : sq);
    mm >>= 8; ss >>= 8;

    /* per-tone AGC: ~100 ms time constant at 9600 Hz (>>10 = 107 ms).
     * Slower than a bit, faster than a packet — it tracks the level, not
     * the data. (This is NOT the SOFT_DC_SHIFT trap: here the decision
     * threshold is not averaged, but the ENERGY of the two branches
     * separately.) */
    s_agc_m += (mm - s_agc_m) >> 10;
    s_agc_s += (ss - s_agc_s) >> 10;
    int32_t am = s_agc_m > 64 ? s_agc_m : 64;
    int32_t as = s_agc_s > 64 ? s_agc_s : 64;

    /* normalised difference in Q12. Without signal both branches sit on
     * their own AGC, so the difference is noise around zero — no false
     * carrier detection. */
    s_lpf_y = ((mm << 12) / am) - ((ss << 12) / as);
  }
#else
  /* delay-and-multiply discriminator (Bell 202) */
  int m = (int)cur * s_delayed[s_delay_idx];
  s_delayed[s_delay_idx] = (int)cur;
  s_delay_idx = (s_delay_idx + 1) % s_delayed_n;

  /* Product LPF = twin FIR (31-tap). 1-pole alone: Track2 609->5 on PC. */
  int32_t x = m >> 7;
  if (x >  32767) x =  32767;
  if (x < -32767) x = -32767;
  s_lpf_y = s_fir_ready ? fir_run((int16_t)x) : x;
#endif

  s_sampledBits = (uint8_t)((s_sampledBits << 1) | ((s_lpf_y > 0) ? 1u : 0u));

  if (SIGNAL_TRANSITIONED(s_sampledBits)) {
    if (s_currentPhase < PHASE_THRESH_A) s_currentPhase += PHASE_INC;
    else                                 s_currentPhase -= PHASE_INC;
  }
  s_currentPhase += PHASE_BITS;
  if (s_currentPhase >= PHASE_MAX_A) {
    s_currentPhase %= PHASE_MAX_A;
    s_actualBits = (uint8_t)(s_actualBits << 1);
    if (popcount5(s_sampledBits & 0x1f) >= 3) s_actualBits |= 1;
    /* NRZI: no transition = 1 */
    bool bit = !TRANSITION_FOUND(s_actualBits);
    hdlc_bit(bit);
  }
}

/* ==================== ENTRY POINTS ==================== */

void aprs_rx_init(void)
{
  s_ais         = AIS_IDLE;
  s_is_fd       = -1;
  s_is_online   = false;
  s_is_last_try = 0;
  s_backoff     = APRSIS_BACKOFF_MIN;
  s_dns_state   = DNS_IDLE;
  Serial0.printf("APRS RX: %s-%d, APRS-IS %s:%d\n",
                 APRS_MYCALL, APRS_MYSSID, APRSIS_HOST, APRSIS_PORT);
  if (APRSIS_PASSCODE < 0) {
    Serial0.println("APRS RX: passcode -1 -> READ-ONLY, nothing is "
                    "uploaded. Enter your own passcode to gate.");
  }
}

void aprs_rx_feed(const int16_t *iq, int nsamp, uint32_t sps)
{
  if (sps == 0) return;
  if (sps != s_sps) aprs_rx_configure(sps);

  for (int n = 0; n < nsamp; n++) {
    /* --- channel filter: I/Q averaging and decimation to ~25 ksps --- */
    s_iq_si += iq[2 * n];
    s_iq_sq += iq[2 * n + 1];
    if (++s_iq_n < s_iq_dec) continue;
    int16_t i = (int16_t)(s_iq_si / (int32_t)s_iq_dec);
    int16_t q = (int16_t)(s_iq_sq / (int32_t)s_iq_dec);
    s_iq_si = s_iq_sq = 0;
    s_iq_n = 0;

    /* FM discriminator: the angle difference between the current and the
     * previous sample.
     *   d = z[n] * conj(z[n-1])
     * The angle is proportional to the instantaneous frequency offset, so
     * this is the demodulated audio itself. */
    int32_t dr = (int32_t)i * (int32_t)s_prev_i + (int32_t)q * (int32_t)s_prev_q;
    int32_t di = (int32_t)q * (int32_t)s_prev_i - (int32_t)i * (int32_t)s_prev_q;
    s_prev_i = i; s_prev_q = q;

    /* atan2 Q13 (~+/-pi). Do not shift the volume away — the LibAPRS
     * delay-multiply is sensitive to small amplitude (PC: 286->515 with scaling). */
    /* NORMALISED computation: the >>2 overflowed above A>1024 — see fm_angle_q13. */
    int32_t a = fm_angle_q13(di, dr);

    /* +/-PI_Q13 -> almost full int16, like the PC twin */
    int32_t scaled = (a * 30000) / PI_Q13;

    /* mild LPF at the IF rate, anti-alias before the 9600 */
    static int32_t if_lpf;
    /* >>1: milder anti-alias (~2x wider than >>2). >>2 was fc~1 kHz @ 25k IF
     * and unbalanced mark/space into delay-multiply. */
    if_lpf += (scaled - if_lpf) >> 1;

    s_rs_acc += 1.f;
    while (s_rs_acc >= s_rs_ratio) {
      s_rs_acc -= s_rs_ratio;
      audio_sample(if_lpf);
    }
  }
}

void aprs_rx_tick(uint32_t now)
{
  /* The whole APRS-IS handling (async DNS, non-blocking connect, login,
   * read, backoff) is in a non-blocking state machine. At most one
   * non-blocking socket operation per phase — the main loop NEVER stalls. */
  aprsis_service(now);
}

uint32_t aprs_rx_frames(void) { return s_frames; }
uint32_t aprs_rx_bad(void)    { return s_bad; }
uint32_t aprs_rx_gated(void)  { return s_gated; }
bool     aprs_rx_is_online(void) { return s_is_online; }
const char *aprs_rx_last(void)      { return s_last; }
const char *aprs_rx_last_call(void) { return s_last_call; }
const char *aprs_rx_last_info(void) { return s_last_info; }
uint32_t    aprs_rx_last_ms(void)   { return s_last_ms; }
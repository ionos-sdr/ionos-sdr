/* fwport.c — BIT-EXACT port of the ESP32 firmware aprs_rx.cpp signal chain
 *
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD  —  MIT
 *
 * MIRE VALO
 *   Nem a PC-s "iker" (esp_demod_fw_v3.c), hanem a FIRMWARE soronkenti masolata.
 *   Csak a koritesz uj (WAV-olvaso, statisztika); minden DSP-sor az
 *   src/aprs_rx.cpp-bol van atmasolva, hogy a csomagszam-elteres CSAK a kodbol
 *   johessen, ne az atirasbol.
 *
 * BISECT-KAPCSOLOK (egyszerre EGY valtozot billents!)
 *   -dc0        a s_audio_dc blokk kikapcsolva
 *   -dcr        a s_audio_dc blokk KEREKITVE  (+128)    <- a 2026-08-09-i javitas (v2.3 firmware)
 *   (alapbol)   a REGI, floor-os  (a - dc) >> 8
 *   -lpf<f>     az IF ">>1" helyett double alfa
 *   -lpfr       az IF ">>1" kerekitve (+1)
 *   -rsd        double ujramintavetelezo akkumulator
 *   -atanf      double atan2 a fixpontos helyett
 *   -pinc<N>    PHASE_INC (bitora-korrekcio lepese), alap 1
 *   -g<x>       bemeneti erosites
 *   -ch<N>      csatorna-FIR N tappal (Q15, decimalo) a boxcar helyett
 *   -chf<Hz>    a csatorna-FIR vagasa (alap 6000)
 *   -v          minden dekodolt csomag kiirasa (ido + TNC2)
 *
 * BUILD   gcc -O2 -o fwport fwport.c -lm
 * RUN     ./fwport file.wav [sps] [kapcsolok...]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int    opt_dc    = 1;
static double opt_lpf   = -1.0;
static int    opt_rsd   = 0;
static int    opt_atanf = 0;
static int    opt_pinc  = 1;
static int    opt_v     = 0;
static int    opt_ch_n  = 0;      /* 0 = regi boxcar; >0 = csatorna-FIR tapszam (Q15, decimalo) */
static double opt_ch_fc = 6000.0; /* csatorna-FIR vagas [Hz]                                   */

/* ---- uj: decimalo komplex csatorna-FIR (firmware-jelolt, Q15 egesz) ---- */
#define CH_MAX 128
static int16_t ch_h[CH_MAX];
static int16_t ch_xi[2*CH_MAX], ch_xq[2*CH_MAX];
static int     ch_pos = 0;
static void ch_design(uint32_t fs, int n, double fc)
{
  const int M = n - 1; double sum = 0, h[CH_MAX];
  for (int k = 0; k < n; k++) {
    double t = k - M / 2.0, x = 2.0 * fc / fs;
    double sinc = (fabs(t) < 1e-9) ? x : sin(M_PI * x * t) / (M_PI * t);
    double win = 0.42 - 0.5 * cos(2 * M_PI * k / M) + 0.08 * cos(4 * M_PI * k / M);
    h[k] = sinc * win; sum += h[k];
  }
  for (int k = 0; k < n; k++) ch_h[k] = (int16_t)lrint(32767.0 * h[k] / sum);
  memset(ch_xi, 0, sizeof ch_xi); memset(ch_xq, 0, sizeof ch_xq); ch_pos = 0;
}

#define AFSK_BAUD        1200
#define IF_TARGET_HZ     25000u
#define AUDIO_TARGET_HZ  9600u
#define MAX_TAPS         16
#define AX25_MAX         330
#define PI_Q13           25736

#define PHASE_BITS       8
#define PHASE_MAX_A      64
#define PHASE_THRESH_A   32
#define DELAYED_N_MAX    8
#define FIR_LPF_N        31
#define BITS_DIFFER(a, b)       (((a) ^ (b)) & 0x01)
#define DUAL_XOR(a, b)          ((((a) ^ (b)) & 0x03) == 0x03)
#define SIGNAL_TRANSITIONED(b)  DUAL_XOR((b), (b) >> 2)
#define TRANSITION_FOUND(b)     BITS_DIFFER((b), (b) >> 1)

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
  int32_t ax = (dr < 0) ? -dr : dr;
  int32_t ay = (di < 0) ? -di : di;
  uint32_t m = (uint32_t)((ax > ay) ? ax : ay);
  int sh = 0;
  if (m > 65535u) sh = 16 - __builtin_clz(m);
  return atan2_q13(di >> sh, dr >> sh);
}

static uint32_t s_sps, s_if_hz, s_audio_hz, s_iq_dec = 1;
static int32_t  s_iq_si, s_iq_sq; static uint32_t s_iq_n;
static int16_t  s_prev_i, s_prev_q;
static float    s_rs_acc = 0.f, s_rs_ratio = 1.f;
static double   s_rs_accd = 0.0, s_rs_ratiod = 1.0;
static int32_t  s_audio_dc;

static int     s_delayed[DELAYED_N_MAX];
static int     s_delay_idx, s_delayed_n = 4;
static uint8_t s_sampledBits, s_actualBits;
static int     s_currentPhase;
static int32_t s_lpf_y;

static int16_t s_fir_an[FIR_LPF_N];
static int16_t s_fir_x[FIR_LPF_N];
static int     s_fir_idx;
static bool    s_fir_ready = false;

static uint8_t  s_hdlc_win, s_hdlc_byte, s_hdlc_nbit;
static bool     s_in_frame;
static uint8_t  s_frame[AX25_MAX];
static uint16_t s_frame_len;
static uint32_t s_frames, s_bad;
static char     s_last[256];
static uint64_t n_audio = 0;

static int popcount5(uint8_t bits) { int c=0; for (int i=0;i<5;i++) c += (bits>>i)&1; return c; }
static float sincf_local(float x){ if (fabsf(x)<1e-6f) return 1.0f; return sinf(x)/x; }
static float fir_window(float x){ if (fabsf(x)>=(float)M_PI) return 0.0f;
  return 0.42f + 0.5f*cosf(x) + 0.08f*cosf(2.0f*x); }
static void fir_design(int fs, int cutoff)
{
  const int M = (FIR_LPF_N - 1) / 2;
  const int A = (1 << 15) - 1;
  float Rc = (float)cutoff / (float)fs;
  for (int n = -M; n <= M; n++)
    s_fir_an[n + M] = (int16_t)(A * 2.0f * Rc * sincf_local(2.0f*(float)M_PI*Rc*(float)n) *
                                fir_window((float)M_PI*(float)n/(float)M));
  memset(s_fir_x, 0, sizeof s_fir_x);
  s_fir_idx = 0;
  s_fir_ready = true;
}
static int fir_run(int16_t value)
{
  int64_t sum = 0;
  s_fir_x[s_fir_idx] = value;
  for (int i = 0; i < FIR_LPF_N; i++)
    sum += (int64_t)s_fir_an[i] * (int64_t)s_fir_x[(s_fir_idx + i) % FIR_LPF_N];
  s_fir_idx += FIR_LPF_N - 1;
  s_fir_idx %= FIR_LPF_N;
  return (int)(sum >> 16);
}

static inline uint16_t crc_update(uint16_t crc, uint8_t b)
{
  b ^= (uint8_t)(crc & 0xFF); b ^= (uint8_t)(b << 4);
  return (uint16_t)(((uint16_t)b << 8) | (crc >> 8)) ^ (uint16_t)(b >> 4) ^ ((uint16_t)b << 3);
}
static bool addr_to_text(const uint8_t *a, char *out, size_t outsz, bool star)
{
  char call[10]; int n = 0;
  for (int i = 0; i < 6; i++) { char c = (char)(a[i] >> 1); if (c != ' ') call[n++] = c; }
  call[n] = '\0';
  int ssid = (a[6] >> 1) & 0x0F; bool last = (a[6] & 0x01) != 0; bool rpt = (a[6] & 0x80) != 0;
  if (ssid) snprintf(out, outsz, "%s-%d%s", call, ssid, (star && rpt) ? "*" : "");
  else      snprintf(out, outsz, "%s%s", call, (star && rpt) ? "*" : "");
  return last;
}
static bool frame_to_tnc2(const uint8_t *f, uint16_t len, char *out, size_t outsz)
{
  if (len < 16) return false;
  char dest[12], src[12], digi[12];
  addr_to_text(&f[0], dest, sizeof dest, false);
  bool last = addr_to_text(&f[7], src, sizeof src, false);
  size_t p = 0; int w = snprintf(out, outsz, "%s>%s", src, dest);
  if (w < 0) return false; p = (size_t)w;
  uint16_t off = 14; int guard = 8;
  while (!last && off + 7 <= len && guard--) {
    last = addr_to_text(&f[off], digi, sizeof digi, true);
    w = snprintf(out + p, outsz - p, ",%s", digi);
    if (w < 0 || (size_t)w >= outsz - p) return false;
    p += (size_t)w; off += 7;
  }
  if (!last) return false;
  if (off + 2 > len) return false;
  if (f[off] != 0x03 || f[off + 1] != 0xF0) return false;
  off += 2;
  if (len < 2 || off > (uint16_t)(len - 2)) return false;
  uint16_t infolen = (uint16_t)(len - 2 - off);
  if (p + 1 + infolen + 1 > outsz) infolen = (uint16_t)(outsz - p - 2);
  out[p++] = ':';
  for (uint16_t i = 0; i < infolen; i++) {
    uint8_t c = f[off + i];
    out[p++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
  }
  out[p] = '\0'; return true;
}
static void frame_complete(void)
{
  if (s_frame_len < 17) { s_frame_len = 0; return; }
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < s_frame_len; i++) crc = crc_update(crc, s_frame[i]);
  if (crc != 0xF0B8) { s_bad++; s_frame_len = 0; return; }
  s_frames++;
  char tnc2[256];
  if (frame_to_tnc2(s_frame, s_frame_len, tnc2, sizeof tnc2)) {
    strncpy(s_last, tnc2, sizeof s_last - 1); s_last[sizeof s_last - 1] = 0;
  } else snprintf(tnc2, sizeof tnc2, "(nem-UI keret, %u bajt)", s_frame_len);
  if (opt_v) printf("  t=%7.1fs  %.100s\n", (double)n_audio / AUDIO_TARGET_HZ, tnc2);
  s_frame_len = 0;
}
static void hdlc_bit(bool bit)
{
  s_hdlc_win = (uint8_t)((s_hdlc_win << 1) | (bit ? 1u : 0u));
  if (s_hdlc_win == 0x7E) {
    if (s_in_frame && s_frame_len >= 17) frame_complete();
    s_in_frame = true; s_frame_len = 0; s_hdlc_byte = 0; s_hdlc_nbit = 0; return;
  }
  if ((s_hdlc_win & 0xFE) == 0xFE) { s_in_frame = false; s_frame_len = 0; s_hdlc_nbit = 0; return; }
  if (!s_in_frame) return;
  if ((s_hdlc_win & 0x3F) == 0x3E) return;
  if (s_hdlc_win & 0x01) s_hdlc_byte |= 0x80;
  s_hdlc_nbit++;
  if (s_hdlc_nbit >= 8) {
    if (s_frame_len < AX25_MAX) s_frame[s_frame_len++] = s_hdlc_byte;
    else { s_in_frame = false; s_frame_len = 0; }
    s_hdlc_byte = 0; s_hdlc_nbit = 0;
  } else s_hdlc_byte >>= 1;
}

static void audio_sample(int32_t a)
{
  if      (opt_dc == 1) { s_audio_dc += (a - s_audio_dc) >> 8;        a -= s_audio_dc; }
  else if (opt_dc == 2) { s_audio_dc += (a - s_audio_dc + 128) >> 8;  a -= s_audio_dc; }

  if (a > 32767) a = 32767;
  if (a < -32768) a = -32768;
  int16_t cur = (int16_t)a;

  int m = (int)cur * s_delayed[s_delay_idx];
  s_delayed[s_delay_idx] = (int)cur;
  s_delay_idx = (s_delay_idx + 1) % s_delayed_n;

  int32_t x = m >> 7;
  if (x >  32767) x =  32767;
  if (x < -32767) x = -32767;
  s_lpf_y = s_fir_ready ? fir_run((int16_t)x) : x;

  s_sampledBits = (uint8_t)((s_sampledBits << 1) | ((s_lpf_y > 0) ? 1u : 0u));
  if (SIGNAL_TRANSITIONED(s_sampledBits)) {
    if (s_currentPhase < PHASE_THRESH_A) s_currentPhase += opt_pinc;
    else                                 s_currentPhase -= opt_pinc;
  }
  s_currentPhase += PHASE_BITS;
  if (s_currentPhase >= PHASE_MAX_A) {
    s_currentPhase %= PHASE_MAX_A;
    s_actualBits = (uint8_t)(s_actualBits << 1);
    if (popcount5(s_sampledBits & 0x1f) >= 3) s_actualBits |= 1;
    hdlc_bit(!TRANSITION_FOUND(s_actualBits));
  }
  n_audio++;
}

static void aprs_rx_configure(uint32_t sps)
{
  s_sps = sps;
  s_iq_dec = sps / IF_TARGET_HZ; if (s_iq_dec < 1) s_iq_dec = 1;
  s_if_hz  = sps / s_iq_dec;
  s_audio_hz = AUDIO_TARGET_HZ;
  s_rs_ratio  = (float)s_if_hz / (float)AUDIO_TARGET_HZ;   s_rs_acc  = 0.f;
  s_rs_ratiod = (double)s_if_hz / (double)AUDIO_TARGET_HZ; s_rs_accd = 0.0;
  s_iq_si = s_iq_sq = 0; s_iq_n = 0;
  s_prev_i = s_prev_q = 0;
  s_audio_dc = 0;
  s_delayed_n = 4;
  memset(s_delayed, 0, sizeof s_delayed);
  s_delay_idx = 0; s_sampledBits = s_actualBits = 0; s_currentPhase = 0; s_lpf_y = 0;
  fir_design((int)AUDIO_TARGET_HZ, 1200);
  fprintf(stderr, "# fwport: %u sps --/%u--> %u Hz IF --frac--> %u Hz audio (ratio %.6f)\n",
          sps, s_iq_dec, s_if_hz, s_audio_hz, (double)s_rs_ratio);
  fprintf(stderr, "# opts: dc=%s lpf=%s rsd=%d atanf=%d pinc=%d\n",
          (opt_dc==2)?"kerekitett(+128)":((opt_dc==1)?"REGI(floor)":"ki"),
          (opt_lpf == -2.0) ? ">>1(kerekitve)" : ((opt_lpf < 0) ? ">>1(int)" : "double"),
          opt_rsd, opt_atanf, opt_pinc);
}

static int32_t if_lpf_i = 0;
static double  if_lpf_d = 0.0;

static void feed_one(int16_t i_in, int16_t q_in)
{
  int16_t i, q;
  if (opt_ch_n) {
    /* dupla puffer: x[pos] es x[pos+N] ugyanaz -> egybefuggo ablak, nincs modulo */
    ch_xi[ch_pos] = ch_xi[ch_pos + opt_ch_n] = i_in;
    ch_xq[ch_pos] = ch_xq[ch_pos + opt_ch_n] = q_in;
    if (++ch_pos >= opt_ch_n) ch_pos = 0;
    if (++s_iq_n < s_iq_dec) return;      /* csak minden dec-edik kimenetet szamoljuk */
    s_iq_n = 0;
    const int16_t *xi = &ch_xi[ch_pos], *xq = &ch_xq[ch_pos];
    int32_t ai = 0, aq = 0;
    for (int k = 0; k < opt_ch_n; k++) { ai += (int32_t)ch_h[k] * xi[k]; aq += (int32_t)ch_h[k] * xq[k]; }
    ai = (ai + 16384) >> 15; aq = (aq + 16384) >> 15;
    if (ai > 32767) ai = 32767; if (ai < -32768) ai = -32768;
    if (aq > 32767) aq = 32767; if (aq < -32768) aq = -32768;
    i = (int16_t)ai; q = (int16_t)aq;
  } else {
    s_iq_si += i_in; s_iq_sq += q_in;
    if (++s_iq_n < s_iq_dec) return;
    i = (int16_t)(s_iq_si / (int32_t)s_iq_dec);
    q = (int16_t)(s_iq_sq / (int32_t)s_iq_dec);
    s_iq_si = s_iq_sq = 0; s_iq_n = 0;
  }

  int32_t dr = (int32_t)i * (int32_t)s_prev_i + (int32_t)q * (int32_t)s_prev_q;
  int32_t di = (int32_t)q * (int32_t)s_prev_i - (int32_t)i * (int32_t)s_prev_q;
  s_prev_i = i; s_prev_q = q;

  int32_t scaled;
  if (opt_atanf) {
    double af = atan2((double)di, (double)dr);
    double sc = af * (30000.0 / M_PI);
    if (sc > 32767.0) sc = 32767.0; if (sc < -32768.0) sc = -32768.0;
    scaled = (int32_t)lrint(sc);
  } else {
    int32_t a = fm_angle_q13(di, dr);
    scaled = (a * 30000) / PI_Q13;
  }

  int32_t feed_val;
  if      (opt_lpf == -2.0) { if_lpf_i += (scaled - if_lpf_i + 1) >> 1; feed_val = if_lpf_i; }
  else if (opt_lpf <  0.0)  { if_lpf_i += (scaled - if_lpf_i) >> 1;     feed_val = if_lpf_i; }
  else { if_lpf_d += opt_lpf * ((double)scaled - if_lpf_d); feed_val = (int32_t)lrint(if_lpf_d); }

  if (opt_rsd) {
    s_rs_accd += 1.0;
    while (s_rs_accd >= s_rs_ratiod) { s_rs_accd -= s_rs_ratiod; audio_sample(feed_val); }
  } else {
    s_rs_acc += 1.f;
    while (s_rs_acc >= s_rs_ratio) { s_rs_acc -= s_rs_ratio; audio_sample(feed_val); }
  }
}

static uint32_t wav_open(FILE *f)
{
  char riff[12];
  if (fread(riff,1,12,f)!=12) return 0;
  if (memcmp(riff,"RIFF",4)||memcmp(riff+8,"WAVE",4)) return 0;
  uint32_t rate=0; uint16_t ch=0,bits=0;
  for (;;) {
    char id[4]; uint32_t sz;
    if (fread(id,1,4,f)!=4) return 0;
    if (fread(&sz,4,1,f)!=1) return 0;
    if (!memcmp(id,"fmt ",4)) {
      uint8_t fmt[40]; uint32_t rd = sz<40?sz:40;
      if (fread(fmt,1,rd,f)!=rd) return 0;
      ch = (uint16_t)(fmt[2]|(fmt[3]<<8));
      rate = (uint32_t)(fmt[4]|(fmt[5]<<8)|(fmt[6]<<16)|((uint32_t)fmt[7]<<24));
      bits = (uint16_t)(fmt[14]|(fmt[15]<<8));
      if (sz>rd) fseek(f,(long)(sz-rd),SEEK_CUR);
      if (sz&1) fseek(f,1,SEEK_CUR);
    } else if (!memcmp(id,"data",4)) {
      fprintf(stderr,"# WAV: %u Hz, %u ch, %u bit\n", rate, ch, bits);
      if (ch!=2||bits!=16) { fprintf(stderr,"# stereo 16 bites I/Q kell\n"); exit(2); }
      return rate;
    } else fseek(f,(long)sz+(sz&1),SEEK_CUR);
  }
}

int main(int argc, char **argv)
{
  const char *path = argc>1?argv[1]:"iq.wav";
  uint32_t sps = argc>2?(uint32_t)atoi(argv[2]):0;
  double gain = 1.0;
  for (int k=2;k<argc;k++) {
    if      (!strcmp(argv[k],"-dc0"))        opt_dc = 0;
    else if (!strcmp(argv[k],"-dcr"))        opt_dc = 2;
    else if (!strcmp(argv[k],"-lpfr"))       opt_lpf = -2.0;
    else if (!strncmp(argv[k],"-lpf",4))     opt_lpf = atof(argv[k]+4);
    else if (!strcmp(argv[k],"-rsd"))        opt_rsd = 1;
    else if (!strcmp(argv[k],"-atanf"))      opt_atanf = 1;
    else if (!strncmp(argv[k],"-pinc",5))    opt_pinc = atoi(argv[k]+5);
    else if (!strcmp(argv[k],"-v"))          opt_v = 1;
    else if (!strncmp(argv[k],"-chf",4))     opt_ch_fc = atof(argv[k]+4);
    else if (!strncmp(argv[k],"-ch",3))      opt_ch_n = atoi(argv[k]+3);
    else if (!strncmp(argv[k],"-g",2))       gain = atof(argv[k]+2);
  }
  FILE *f = fopen(path,"rb"); if (!f) { perror(path); return 1; }
  uint32_t wrate = wav_open(f);
  if (!sps) sps = wrate ? wrate : 50000;
  aprs_rx_configure(sps);
  if (opt_ch_n) { if (opt_ch_n > CH_MAX) opt_ch_n = CH_MAX; ch_design(sps, opt_ch_n, opt_ch_fc);
    fprintf(stderr, "# csatorna-FIR: %d tap, fc=%.0f Hz, Q15, decimalo /%u\n", opt_ch_n, opt_ch_fc, s_iq_dec); }
  if (gain != 1.0) fprintf(stderr, "# bemeneti erosites x%.3f\n", gain);

  int16_t buf[512]; size_t n; uint64_t niq=0; int32_t peak=0;
  while ((n = fread(buf,sizeof(int16_t),512,f)) > 0) {
    size_t pairs = n/2;
    for (size_t k=0;k<pairs;k++) {
      double vi = buf[k*2]*gain, vq = buf[k*2+1]*gain;
      if (vi> 32767) vi= 32767; if (vi< -32768) vi= -32768;
      if (vq> 32767) vq= 32767; if (vq< -32768) vq= -32768;
      int16_t ii=(int16_t)lrint(vi), qq=(int16_t)lrint(vq);
      int32_t ai = ii<0?-ii:ii, aq = qq<0?-qq:qq;
      if (ai>peak) peak=ai; if (aq>peak) peak=aq;
      feed_one(ii,qq); niq++;
    }
  }
  fclose(f);
  printf("packets:  %u   crc_fail: %u   (%.1f s, peak|IQ|=%d)\n",
         s_frames, s_bad, (double)niq/sps, peak);
  return 0;
}

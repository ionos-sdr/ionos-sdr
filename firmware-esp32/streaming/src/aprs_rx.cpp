/* SPDX-License-Identifier: MIT
 *
 * aprs_rx.cpp — AFSK1200 + AX.25 + APRS-IS  (ESP32-S3 / FG23 IQ)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * (LibAPRS delay-mult path + DC-block + mild IF AA — 2026-08-09)
 *
 * DEBUG / CHANGELOG
 *   v2.3: s_audio_dc floor-shift -> rounded (+128). A ">>8" a negativ hibat
 *         lefele kerekitette, ezert a "DC-blokk" allando +128 egyenszintet
 *         ADOTT a hanghoz. Track2 (kis deviacio): 0 -> 75 csomag. Ez volt az
 *         oka annak, hogy csak a PER1000 ment, minden mas jel nulla volt.
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

/* Nyers, nem-blokkolo socketek az APRS-IS klienshez — ugyanaz a stilus,
 * mint a tobbi TCP-uton (send(fd, ..., MSG_DONTWAIT)). Async DNS a tcpip
 * szalban, hogy a nevfeloldas se allitsa meg a fo hurkot. */
#include <lwip/sockets.h>
#include <lwip/dns.h>
#include <lwip/tcpip.h>
#include <lwip/ip_addr.h>
#include <errno.h>
#include <fcntl.h>

#include "flashlog.h"

/* ==================== KONFIGURACIO ==================== */

#include "station_config.h"   /* APRS_MYCALL, APRSIS_PASSCODE — git-ignored */
#define APRS_MYSSID      10          /* -10 a bevett iGate SSID */

/* APRS-IS. A passcode a HIVOJELEDHEZ tartozik.
 *
 *   -1  = CSAK OLVASO kapcsolat. A szerver elfogadja, de nem enged kuldeni.
 *         Ezzel indulj: latni fogod a logban, hogy dekodalsz, es kozben
 *         semmit nem tolsz fel.
 *   sajat passcode = feltoltes engedelyezve (kell hozza ervenyes engedely).
 *
 * A -1 nem "trukk", hanem az APRS-IS dokumentalt read-only modja. */
#define APRSIS_HOST      "rotate.aprs2.net"
#define APRSIS_PORT      14580
#define APRSIS_FILTER    "r/47.5167/19.4333/50"   /* Dany, kor: lat/lon/km */
#define APRSIS_SOFTWARE  "FG23-SDR-iGate 1.0"
#define APRSIS_RETRY_MS  15000u

/* AFSK Bell 202 */
#define AFSK_BAUD        1200
#define AFSK_MARK_HZ     1200
#define AFSK_SPACE_HZ    2200

/* CSATORNASZURO a diszkriminator ELE. Ez nem kozmetika: egy 12,5 kHz-es
 * FM-csatornat 50 kHz-es sávban demodulalni ~6 dB SNR-veszteseg, mert a
 * diszkriminator a teljes sáv zajat is beleszamolja. Ezert az I/Q-t
 * eloszor leviszuk kb. 25 ksps-re (= +-12,5 kHz), ami pont egy APRS-
 * csatorna. Boxcar-atlagolas, tehat nehany osszeadas mintankent. */
#define IF_TARGET_HZ     25000u

/* Az audio-rata, ahova utana decimalunk. 12 kHz korul jo: ~10 minta/bit,
 * boven eleg a korrelatornak, es negyede a CPU-nak a teljes ratahoz kepest. */
#define AUDIO_TARGET_HZ  9600u
#define MAX_TAPS         16          /* unused (LibAPRS path) */
#define AX25_MAX         330         /* a leghosszabb ertelmes AX.25 keret */

/* A szeletelo onkozepezo kuszobenek idoallandoja, 2^N minta.
 * 0 = A KUSZOB TELJESEN KIKAPCSOLVA (a szeletelo a NULLAHOZ hasonlit).
 *
 * ===== 2026-08-06: EZ VOLT A 60%-OS CSOMAGVESZTES OKA =====
 *
 * Elozmeny: 10 (=1024 minta, 82 ms) tul gyors volt, raallt a preambulumra —
 * ezert lett belole 13 (=8192 minta, 655 ms), "lassabb egy csomagnal".
 * A 13 viszont NEM eleg lassu: egy csomag ~500 ms, tehat a kuszob a csomag
 * ALATT is erdemben elmozdul. Es mivel az adatban a mark/space arany NEM
 * 50-50, a futo atlag afele a hang fele huz, amelyik epp tobb — vagyis a
 * dontesi kuszob magat az adatot koveti, es elbillenti a szeletelest.
 *
 * MIERT FELESLEGES EGYALTALAN: amiert bekerult (a frekvencia-offszet mint
 * allando tag), azt MAR ELINTEZI az audio_sample() elejen levo s_audio_dc
 * szivo (>>8, ~20 ms). Mire a korrelatorhoz erunk, a jel mar
 * kozepre van allitva, tehat a helyes dontesi kuszob a NULLA.
 *
 * MERES (esp_demod.c — az egesz jelut bitre azonos PC-s portja, ugyanazon
 * a mintasoron futtatva, amit a firmware is kapott):
 *
 *   bemenet                              SOFT_DC_SHIFT 13   ->   0
 *   -------------------------------------------------------------------
 *   VALODI FG23-felvetel (SpyServer,
 *     50 ksps, 140.8 s, 142 csomag,
 *     -96 dBm a chipnel)                      38.0 %        98.6 %
 *   szintetikus, SNR 24 dB                    85.0 %       100.0 %
 *   szintetikus, SNR 10 dB                    82.5 %       100.0 %
 *
 * A 38.0% a HARDVEREN mert 37..38%-ot pontosan reprodukalta — tehat a
 * becsatornazas (cur_sps, blokk-betaplalas, idozites) VEGIG HELYES VOLT,
 * a hiba kizarolag ebben az egy sorban ult. Osszehasonlitaskepp ugyanazon
 * a felvetelen a PC-s "gold" demod (wav_aprs_per.py) 81.7%-ot vitt, tehat
 * a javitott valtozat MEG ANNAL IS jobb.
 *
 * NE ALLITSD VISSZA nem-nulla ertekre a fenti pad ujrafuttatasa nelkul.
 * A 11 pl. 17.1%-ot, a 15 pedig 56.3%-ot ad ugyanazon a felvetelen. */
#define SOFT_DC_SHIFT    0

/* ==================== FM DISZKRIMINATOR ==================== */

#define PI_Q13  25736                /* 3.14159 * 8192 */

/* Kozelito atan2, Q13 radianban. Hibaja ~0.01 rad — FM-hangra boven eleg,
 * es nincs benne lebegopontos muvelet. */
static inline int32_t atan2_q13(int32_t y, int32_t x)
{
  int32_t abs_y = (y < 0 ? -y : y) + 1;   /* a +1 a 0/0 ellen */
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

/* ============ A NAP HIBAJA: TULCSORDULO atan2 (2026-08-09) ============
 *
 * A fenti atan2_q13 belsejeben ez all:   (x - abs_y) << 13
 * Ez int32-ben CSAK akkor helyes, ha |x| es |y| eleg kicsi. A hatar:
 *   |x - abs_y| <= 2^31 / 2^13 = 262 144
 * Mivel |x - abs_y| a legrosszabb esetben |x| + |y|, a biztonsagos korlat
 * mindkettore kb. 131 000 — es tartalekkal 65 535.
 *
 * A hivo eddig ezt csinalta:   atan2_q13(di >> 2, dr >> 2)
 * ahol dr = i*pi + q*pq  es  di = q*pi - i*pq, tehat |dr|,|di| <= A^2,
 * ahol A az I/Q burolgorbe. A >>2 utan A^2/4 <= 262144, vagyis
 *
 *        ==>  A > 1024  ESETEN AZ ATAN2 TULCSORDUL  <==
 *
 * MERES (2026-08-09, a lancon atment felvetelre, szintenkent skalazva,
 * a pontos arctan2-hoz hasonlitva):
 *
 *      szint                |I/Q| csucs    durvan hibas szog
 *      -----------------------------------------------------
 *      -94 dBm (felvetel)         977            0,0 %
 *      +6 dB                     1954          100,0 %
 *      +12 dB                    3907           99,8 %
 *      +24 dB (kezirad.)        15629           99,8 %
 *
 * EZ MAGYARAZZA AZ EGESZ NAPOT:
 *   - a PER1000 -100 dBm-en megy, mert ott a csucs ~411 (hatszoros tartalek);
 *   - a Track2 -94 dBm-en csucs 993..1036 — PONT a szakadek szelen;
 *   - barmi ennel EROSEBB (kezirádio, kozeli allomas) 100 %-ban szemet;
 *   - a PC-s iker viszont DOUBLE atan2-t hasznal, ezert ott sosem latszott.
 * Vagyis minel erosebb a jel, annal biztosabban nem dekodol — pont
 * forditva, mint amit az ember ösztonösen keres.
 *
 * A JAVITAS. Az atan2 SKALA-FUGGETLEN: ha dr-t es di-t UGYANANNYIVAL
 * shifteljuk, a szog valtozatlan. Tehat normalunk 16 bitre, es kesz.
 * Koltseg: egy clz es ket shift mintankent. */
static inline int32_t fm_angle_q13(int32_t di, int32_t dr)
{
  int32_t ax = (dr < 0) ? -dr : dr;
  int32_t ay = (di < 0) ? -di : di;
  uint32_t m = (uint32_t)((ax > ay) ? ax : ay);
  int sh = 0;
  if (m > 65535u) sh = 16 - __builtin_clz(m);   /* (m>>sh) <= 65535 */
  return atan2_q13(di >> sh, dr >> sh);
}

/* ==================== ALLAPOT ==================== */

static uint32_t s_sps = 0;           /* bemeneti I/Q rata */
static uint32_t s_if_hz = 0;         /* csatornaszures utani I/Q rata */
static uint32_t s_audio_hz = 0;      /* audio-decimalas utani rata */
static uint32_t s_iq_dec = 1;
static uint32_t s_decim = 1;

/* I/Q csatornaszuro (boxcar) */
static int32_t  s_iq_si = 0, s_iq_sq = 0;
static uint32_t s_iq_n = 0;

/* FM diszkriminator */
static int16_t  s_prev_i = 0, s_prev_q = 0;

/* decimalas: ket kaszkadolt boxcar (haromszog-ablak) */
static int32_t  s_box1 = 0, s_box2 = 0;
static float s_rs_acc = 0.f, s_rs_ratio = 1.f;
static uint32_t s_box_n = 0;

/* audio DC-blokk */
static int32_t  s_audio_dc = 0;

/* korrelator */
static int8_t   s_cos_m[MAX_TAPS], s_sin_m[MAX_TAPS];
static int8_t   s_cos_s[MAX_TAPS], s_sin_s[MAX_TAPS];
static int16_t  s_hist[64];
static uint32_t s_hist_idx = 0;
static uint32_t s_taps = 10;

/* szeletelo + PLL */
static int32_t  s_soft = 0;          /* szurt korrelator-kulonbseg */
static int32_t  s_soft_dc = 0;       /* onkozepezo dontesi kuszob */
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

/* ==== KAPCSOLO: 1 = korrelator + AGC,  0 = LibAPRS delay-multiply + FIR
 *
 * ALAPBOL 0 — ES EZ TUDATOS VISSZALEPES (2026-08-09 este).
 *
 * A korrelatort azert irtam meg, mert a delay-multiply kimenete a deviacio
 * negyzetevel megy. Az ervelés helyes, a MERES viszont nem tamasztja ala:
 * ugyanabban a sajat keretrendszeremben, ugyanazon a tnc_track2 fajlon
 *
 *      delay-multiply + FIR :  CRC-helyes 96
 *      korrelator + AGC     :  CRC-helyes 83..85
 *
 * vagyis a korrelatorom ROSSZABB, nem jobb. Ezen felul a ket kulon AGC
 * idoallandoja >>10 = 107 ms, ami egy ~500 ms-os csomag elso negyedet
 * elrontja, amig beall.
 *
 * HIBAT KOVETTEM EL A MODSZERBEN: egyszerre adtam ki egy BIZONYITOTT
 * javitast (a lenti atan2-tulcsordulas) es egy NEM bizonyitottat (ezt).
 * Pontosan az, amit egesz nap kifogasoltam az iker es a firmware
 * szetcsuszasanal. Ezert a kapcsolo alapbol 0: maradjon benne a
 * bizonyitott javitas, es essen ki a bizonytalan valtozas. Egy valtozo
 * egyszerre.
 *
 * A korrelator a kodban marad, mert az elve jo, es ha a meropad valaha
 * azt mondja, hogy kell — 1-re allitva azonnal visszajon. */
#define APRS_DEMOD_CORRELATOR 0

#define COR_N 8                      /* egy bit @ 9600/1200 */
static const int8_t COS_M[COR_N] = { 127,  90,    0, -90, -127, -90,    0,  90 };
static const int8_t SIN_M[COR_N] = {   0,  90,  127,  90,    0, -90, -127, -90 };
/* 2200 Hz @ 9600: fazislepes 2*pi*2200/9600 = 1.4399 rad/minta.
 * Az ertekek round(127*cos/sin(k*1.4399)) — NEM kezzel becsulve, hanem
 * kiszamolva es a teljes jelutton visszamerve (lasd lent). */
static const int8_t COS_S[COR_N] = { 127,  17, -123, -49,  110,  77,  -90, -101 };
static const int8_t SIN_S[COR_N] = {   0, 126,   33, -117, -63, 101,   90,  -77 };

/* ELLENORIZVE 2026-08-09: ez a fixpontos jelut (ugyanezekkel az int8
 * egyutthatokkal, ugyanezzel a >>8 nagysag-skalazassal, >>10 AGC-vel es
 * Q12-es hanyadossal) a tnc_track2_iq_3min_0hz.wav-bol 176 keretkezdetet
 * es 83 CRC-HELYES csomagot ad. Tehat az aritmetika es a tablazat jo;
 * ami ezen tul romlik, az mar a jel, nem a szamolas. */
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
  memset(s_cor_x, 0, sizeof s_cor_x);   /* korrelator kesleltetosor */
  s_cor_idx = 0;
  s_agc_m = s_agc_s = 0;               /* hangonkenti AGC */
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
static uint8_t  s_hdlc_win = 0;      /* gorgo bitablak */
static uint8_t  s_hdlc_byte = 0;
static uint8_t  s_hdlc_nbit = 0;
static bool     s_in_frame = false;
static uint8_t  s_frame[AX25_MAX];
static uint16_t s_frame_len = 0;

/* statisztika */
static uint32_t s_frames = 0, s_bad = 0, s_gated = 0;
static char     s_last[256] = "";
static char     s_last_call[12] = "";
static char     s_last_info[128] = "";
static uint32_t s_last_ms = 0;

/* APRS-IS — a nem-blokkolo allapotgep tobbi allapota a lenti szekcioban. */
static bool     s_is_online   = false;
static uint32_t s_is_last_try = 0;   /* utolso kapcsolatkiserlet/hiba ideje */

/* ==================== SZUROK BEALLITASA ==================== */

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

  /* minta/bit az audio ratan. NEM kell egesznek lennie: a PLL tortszamu
   * bithosszal is beall, ezert nem eroltetjuk a ratat kerek ertekre. */
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

  /* PLL: egy bit alatt pontosan egy korbefordulas. */
  s_pll_inc = (int32_t)(((uint64_t)PLL_MAX * AFSK_BAUD) / s_audio_hz);
  s_pll_adj = s_pll_inc / 4;         /* hurokerosites */
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

/* Egy 7 bajtos cimmezo kiirasa. Visszaad: a "vege" bit (utolso cim). */
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
  bool rpt  = (a[6] & 0x80) != 0;      /* H bit: mar ismetelte egy digi */

  if (ssid) snprintf(out, outsz, "%s-%d%s", call, ssid,
                     (star && rpt) ? "*" : "");
  else      snprintf(out, outsz, "%s%s", call, (star && rpt) ? "*" : "");
  return last;
}

/* A keretbol TNC2 szoveget csinal: SRC>DEST,DIGI1*,DIGI2:info
 * Visszaad: sikerult-e. */
static bool frame_to_tnc2(const uint8_t *f, uint16_t len,
                          char *out, size_t outsz)
{
  if (len < 16) return false;          /* 2 cim + ctrl + pid + FCS */

  char dest[12], src[12], digi[12];
  addr_to_text(&f[0], dest, sizeof dest, false);
  bool last = addr_to_text(&f[7], src, sizeof src, false);

  size_t p = 0;
  int w = snprintf(out, outsz, "%s>%s", src, dest);
  if (w < 0) return false;
  p = (size_t)w;

  uint16_t off = 14;
  int guard = 8;                       /* AX.25: legfeljebb 8 digi */
  while (!last && off + 7 <= len && guard--) {
    last = addr_to_text(&f[off], digi, sizeof digi, true);
    w = snprintf(out + p, outsz - p, ",%s", digi);
    if (w < 0 || (size_t)w >= outsz - p) return false;
    p += (size_t)w;
    off += 7;
  }
  if (!last) return false;             /* nem zarodott le a cimmezo */

  /* control + PID: APRS-nal mindig UI keret (0x03) es 0xF0. Barmi mas
   * nem APRS — nem is akarjuk felkuldeni. */
  if (off + 2 > len) return false;
  if (f[off] != 0x03 || f[off + 1] != 0xF0) return false;
  off += 2;

  if (len < 2 || off > (uint16_t)(len - 2)) return false;
  uint16_t infolen = (uint16_t)(len - 2 - off);   /* a 2 bajt FCS levonva */

  if (p + 1 + infolen + 1 > outsz) infolen = (uint16_t)(outsz - p - 2);
  out[p++] = ':';
  for (uint16_t i = 0; i < infolen; i++) {
    uint8_t c = f[off + i];
    out[p++] = (c >= 0x20 && c < 0x7F) ? (char)c : '.';
  }
  out[p] = '\0';
  return true;
}

/* ==================== APRS-IS (nem-blokkolo) ==================== */

/* A kapcsolat SEMMILYEN fazisa nem allitja meg a fo hurkot. A regi
 * WiFiClient::connect(...,3000) egyetlen tranzakcioban 140 ms-ra (hibanal
 * 3 s-ra) megallt -> RF-blokkvesztes (a boot-logban: aprs=140124 us, d=20
 * lyuk). Helyette allapotgep:
 *   DNS       : async lwIP dns_gethostbyname a tcpip szalban (tcpip_callback),
 *   connect   : O_NONBLOCK socket, EINPROGRESS, select() 0 idokorlattal,
 *   login/gate: send(fd, MSG_DONTWAIT) — mint a tobbi TCP-ut,
 *   olvasas   : recv(fd, MSG_DONTWAIT), koltseg-korlattal.
 * Hiba/bontas utan exponencialis backoff (3..60 s), sikerre nullazva. */

/* --- async DNS. A tcpip szalbol irjuk, a fo hurokbol olvassuk. Az ESP32-S3
 * ket magja UGYANAZT a belso SRAM-ot latja (nincs rajta kulon adatcache),
 * ezert a volatile itt eleg: a callback eloszor az IP-t irja, UTOLSOKENT a
 * flaget, es olvasaskor eloszor a flaget nezzuk. --- */
enum { DNS_IDLE = 0, DNS_REQ, DNS_WAIT, DNS_DONE, DNS_FAIL };
static volatile int      s_dns_state = DNS_IDLE;
static volatile uint32_t s_dns_ip4   = 0;      /* halozati bajtsorrend */

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

/* A tcpip szalban fut (tcpip_callback utemezi ide) — a raw lwIP DNS API-t
 * CSAK innen szabad hivni. */
static void aprsis_dns_start(void *arg)
{
  (void)arg;
  ip_addr_t addr;
  err_t e = dns_gethostbyname_addrtype(APRSIS_HOST, &addr, aprsis_dns_cb,
                                       NULL, LWIP_DNS_ADDRTYPE_IPV4);
  if (e == ERR_OK) {                    /* gyorsitotarbol, azonnal kesz */
    if (IP_IS_V4(&addr)) { s_dns_ip4   = ip4_addr_get_u32(ip_2_ip4(&addr));
                           s_dns_state = DNS_DONE; }
    else                   s_dns_state = DNS_FAIL;
  } else if (e == ERR_INPROGRESS) {     /* a callback fog jonni kesobb */
    s_dns_state = DNS_WAIT;
  } else {
    s_dns_state = DNS_FAIL;
  }
}

/* --- allapot --- */
enum aprsis_state { AIS_IDLE = 0, AIS_DNS, AIS_CONNECTING, AIS_LOGIN, AIS_ONLINE };
static aprsis_state s_ais         = AIS_IDLE;
static int          s_is_fd       = -1;
static uint32_t     s_phase_start = 0;   /* az aktualis fazis kezdete (timeout) */

#define APRSIS_BACKOFF_MIN  3000u
#define APRSIS_BACKOFF_MAX  60000u
#define APRSIS_DNS_TMO_MS   8000u
#define APRSIS_CONN_TMO_MS  6000u
static uint32_t s_backoff = APRSIS_BACKOFF_MIN;

/* login-puffer: a reszleges kuldest is kovetjuk, hogy sose maradjon fel-sor
 * a streamben (a "reszleges egyseg ol" tanulsag). */
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
  if (why) Serial0.printf("APRS-IS: %s -> ujra %lu s mulva\n",
                          why, (unsigned long)(s_backoff / 1000));
  aprsis_close();
  s_is_last_try = millis();              /* a backoff a HIBATOL szamit */
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

/* Szabad-e felkuldeni ezt a csomagot? Az APRS-IS sajat etikettje. */
static bool gate_allowed(const char *tnc2)
{
  if (strstr(tnc2, "TCPIP")) return false;   /* mar volt a neten */
  if (strstr(tnc2, "TCPXX")) return false;
  if (strstr(tnc2, "NOGATE")) return false;  /* a kuldo kifejezetten tiltja */
  if (strstr(tnc2, "RFONLY")) return false;
  const char *colon = strchr(tnc2, ':');
  if (colon && colon[1] == '}') return false; /* harmadik feles keret */
  return true;
}

static void aprsis_send(const char *tnc2)
{
  if (!s_is_online || s_is_fd < 0) return;
  if (APRSIS_PASSCODE < 0) return;           /* olvaso mod: nem kuldunk */
  if (!gate_allowed(tnc2)) {
    Serial0.println("APRS-IS: nem gateleheto (TCPIP/NOGATE/RFONLY/3rd party)");
    return;
  }

  /* A qAR jelzi, hogy radiobol vettuk es mi tettuk fel. */
  const char *colon = strchr(tnc2, ':');
  if (!colon) return;
  int head = (int)(colon - tnc2);
  if (head > 200) return;
  char line[300];
  int n = snprintf(line, sizeof line, "%.*s,qAR,%s-%d%s\r\n",
                   head, tnc2, APRS_MYCALL, APRS_MYSSID, colon);
  if (n < 0) return;
  if (n >= (int)sizeof line) return;   /* csonkolt: NE kuldj fel-sort a
                                        * \r\n nelkul, az rontja az IS-folyamot */

  /* Egyetlen nem-blokkolo send. Egy APRS-sor par szaz bajt, a kuldopuffer
   * bosegesen elnyeli. Ha megis: EAGAIN -> semmi nem ment ki, a stream ep,
   * csak ezt a csomagot dobjuk (az APRS vesztes protokoll). Reszleges vagy
   * egyeb hiba -> a stream serulhetett, bontunk es ujracsatlakozunk. */
  int w = send(s_is_fd, line, n, MSG_DONTWAIT);
  if (w == n) {
    s_gated++;
  } else if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    Serial0.println("APRS-IS: kuldopuffer tele, csomag eldobva");
  } else {
    aprsis_fail("reszleges/hibas gate-kuldes");
  }
}

/* A szervertol jovo sorokat OLVASNI kell, kulonben megtelik a fogadopuffer
 * es a kapcsolat megall. Nem-blokkolva, ciklusonkenti felso koltseggel — egy
 * hirtelen adatozon se allithassa meg a pumpat. recv==0 = tuloldali lezaras. */
static void aprsis_drain(void)
{
  uint8_t sink[256];
  int budget = 2048;
  while (budget > 0) {
    int n = recv(s_is_fd, sink, sizeof sink, MSG_DONTWAIT);
    if (n > 0) { budget -= n; continue; }
    if (n == 0) { aprsis_fail("a szerver lezarta a kapcsolatot"); return; }
    if (errno == EAGAIN || errno == EWOULDBLOCK) return;   /* nincs tobb adat */
    aprsis_fail("olvasas hiba"); return;
  }
}

/* A teljes nem-blokkolo allapotgep. Minden fo-ciklusbol egyszer fut, es
 * fazisonkent legfeljebb EGY nem-blokkolo socket-muveletet vegez. */
static void aprsis_service(uint32_t now)
{
  if (WiFi.status() != WL_CONNECTED) {   /* nincs WiFi: bontunk, varunk */
    if (s_ais != AIS_IDLE || s_is_fd >= 0) aprsis_close();
    return;
  }

  switch (s_ais) {

  case AIS_IDLE:
    if (s_is_last_try != 0 && now - s_is_last_try < s_backoff) return;
    s_dns_state = DNS_REQ;
    if (tcpip_callback(aprsis_dns_start, NULL) != ERR_OK) {
      aprsis_fail("DNS-utemezes nem sikerult");
      return;
    }
    s_phase_start = now;
    s_ais = AIS_DNS;
    return;

  case AIS_DNS:
    if (s_dns_state == DNS_FAIL)                { aprsis_fail("nevfeloldas hiba"); return; }
    if (now - s_phase_start > APRSIS_DNS_TMO_MS) { aprsis_fail("DNS idotullepes");  return; }
    if (s_dns_state != DNS_DONE) return;        /* meg varunk a feloldasra */
    {
      s_is_fd = socket(AF_INET, SOCK_STREAM, 0);
      if (s_is_fd < 0) { aprsis_fail("socket() hiba"); return; }
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
      if (r == 0) {                             /* ritka: azonnal kesz */
        aprsis_prepare_login();
        s_ais = AIS_LOGIN;
      } else if (errno == EINPROGRESS) {
        s_phase_start = now;
        s_ais = AIS_CONNECTING;
      } else {
        aprsis_fail("connect() hiba");
      }
    }
    return;

  case AIS_CONNECTING:
    {
      if (now - s_phase_start > APRSIS_CONN_TMO_MS) { aprsis_fail("connect idotullepes"); return; }
      fd_set wf; FD_ZERO(&wf); FD_SET(s_is_fd, &wf);
      struct timeval tv; tv.tv_sec = 0; tv.tv_usec = 0;
      int s = select(s_is_fd + 1, NULL, &wf, NULL, &tv);
      if (s <= 0) return;                       /* meg nem irhato */
      int soe = 0; socklen_t sl = sizeof soe;
      getsockopt(s_is_fd, SOL_SOCKET, SO_ERROR, &soe, &sl);
      if (soe != 0) { aprsis_fail("connect elutasitva"); return; }
      aprsis_prepare_login();
      s_ais = AIS_LOGIN;
    }
    return;

  case AIS_LOGIN:
    while (s_login_sent < s_login_len) {
      int w = send(s_is_fd, s_login + s_login_sent,
                   s_login_len - s_login_sent, MSG_DONTWAIT);
      if (w > 0) { s_login_sent += (uint16_t)w; continue; }
      if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return; /* majd kesobb */
      aprsis_fail("login kuldes hiba");
      return;
    }
    s_is_online = true;
    s_backoff   = APRSIS_BACKOFF_MIN;           /* siker -> backoff nullazasa */
    s_ais       = AIS_ONLINE;
    Serial0.printf("APRS-IS: csatlakozva %s:%d mint %s-%d (passcode %d%s)\n",
                   APRSIS_HOST, APRSIS_PORT, APRS_MYCALL, APRS_MYSSID,
                   APRSIS_PASSCODE, (APRSIS_PASSCODE < 0) ? ", CSAK OLVASO" : "");
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

  /* A teljes kereten (az FCS-t is beleertve) atfuttatva a maradek egy
   * fix ertek. Ez az X.25 szokasos ellenorzese. */
  if (crc != 0xF0B8) {
    s_bad++;
#if FLASHLOG_ENABLE
    /* minden 64. CRC-hiba mintaja a hosszaval — igy latszik, hogy a bukok
     * kozel-teljes keretek (gyenge valodi csomag) vagy rovidek (zaj). */
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

    /* Szetbontjuk a kijelzonek: a hivojel a '>' elott, az info az elso
     * ':' utan van. A kijelzon ez a ket darab kell, nem a teljes sor. */
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
    Serial0.println("\nAPRS RX: ervenyes keret, de nem APRS UI — kihagyva");
  }
  s_frame_len = 0;
}

/* Egy NRZI-dekodolt bit a HDLC-nek. */
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
  /* a: mar skalazott discr hang, kb. teljes int16 tartomany */
  /* Audio DC-block: remove FM discr carrier offset (>>8 => fc~6 Hz @ 9600).
   * Critical for real TX offset; VSG at exact LO has DC~0 so bench hid this.
   *
   * ===== 2026-08-09: A "+128" NEM KOZMETIKA, EZ VOLT A NAP MASODIK HIBAJA =====
   *
   * A regi sor ez volt:      s_audio_dc += (a - s_audio_dc) >> 8;
   * Az aritmetikai jobbra shift a NEGATIV szamokat LEFELE kerekiti (floor),
   * tehat az atlagos lepes nem (e/256), hanem (e/256 - 0.5). Az egyensuly
   * ezert NEM mean(a)-nal all be, hanem mean(a) - 128-nal — vagyis a
   * "DC-blokk" egy ALLANDO +128 EGYENSZINTET AD A HANGHOZ, ahelyett hogy
   * kivenne belole. (Merve: mean(if_lpf) = -0,1, s_audio_dc = -127,7.)
   *
   * Miert ol: a delay-multiply utan a D egyenszint egy allando +D^2 tagot ES
   * egy 2*D*a(t) keresztszorzatot ad, amit az 1200 Hz-es FIR nagyreszt
   * atenged. A hang csucsa a Track2-n MINDOSSZE ~1100 (a felvetel csucs-
   * deviacioja p99 = 907 Hz a szabvanyos +-3000 helyett), tehat a 128
   * a hang 11 %-a, es a keresztszorzat ~45 % rippli a dontesi szinten.
   * PER1000-en ugyanez a 128 csak ~2 %, mert AZT teljes deviacioval
   * generaltuk — ezert ment a PER1000 -111...-50 dBm kozott vegig, es ezert
   * volt a Track2 es a kezirádio NULLA MINDEN SZINTEN.
   *
   * MERES (fwport = ennek a fajlnak a bitre azonos PC-s portja,
   * tnc_track2_iq_3min_0hz.wav 50 kSps-re ujramintavetelezve, CRC-helyes
   * csomagok; a "dev" a felvetel deviaciojanak szorzoja):
   *
   *      dev    regi (>>8 floor)   uj (+128)    DC-blokk NELKUL
   *      ---------------------------------------------------------
   *      1x            0              75              77
   *      2x           46              74              72
   *      4x           75              73              73
   *      6x           83              --              75
   *
   *   es vivo-offszettel, 1x deviacion:
   *      offszet    regi    uj    nelkule
   *      ------------------------------------
   *      0 Hz         0     75      77
   *      500 Hz       0     75       0
   *      1000 Hz      0     76       0
   *      2000 Hz      0     76       0
   *      3000 Hz      0     69       0
   *
   * Vagyis: a DC-blokk KELL (nelkule mar 500 Hz offszet nullaz), de csak
   * KEREKITVE mukodik. A javitott valtozat 71..79 csomag a teljes
   * deviacio x offszet matrixon — deviaciofuggetlen, ahogy egy FM
   * diszkriminatortol elvarjuk.
   *
   * Az ugyanilyen tipusu (scaled - if_lpf) >> 1 az IF-en csak -1 LSB-t
   * tolna, es merve NEM valtoztat semmit (75/85 mindket modon) — ezert azt
   * SZANDEKOSAN NEM nyultuk meg. Egy valtozo, egy javitas. */
  s_audio_dc += (a - s_audio_dc + 128) >> 8;
  a -= s_audio_dc;

  if (a > 32767) a = 32767;
  if (a < -32768) a = -32768;
  int16_t cur = (int16_t)a;

#if APRS_DEMOD_CORRELATOR
  /* ============ MARK/SPACE KORRELATOR, HANGONKENTI AGC ============
   *
   * MIERT EZ, ES NEM A DELAY-MULTIPLY (2026-08-09).
   *
   * A delay-multiply kimenete a hang AMPLITUDOJANAK NEGYZETEVEL aranyos,
   * es FM-nel az amplitudo = a DEVIACIO. A szeletelo nullahoz hasonlit,
   * tehat a dontesi tartalek is a deviacio negyzetevel megy.
   *
   * MERES (2026-08-09, a lancon atment felvetel vs. az eredeti fajl):
   *
   *   eredeti Track2 IQ fajl : MARK 26 dB-lel a zaj folott  -> dekodol
   *   ugyanaz a lancon at    : MARK  9 dB-lel a zaj folott  -> SEMMI
   *
   * es a fajl csucsdeviacioja mindossze +-675 Hz a szabvanyos +-3000
   * helyett. Az FM demodulalt SNR a deviacio negyzetevel megy, tehat ez
   * onmagaban ~13 dB — pontosan annyi, amennyi hianyzik. A PER1000 azert
   * megy -110 dBm-ig, mert azt helyes deviacioval generaltuk.
   *
   * A KORRELATOR EZT MEGSZUNTETI. Kulon mérjük a MARK es a SPACE
   * energiajat egy bitnyi ablakon, MINDKETTOT SAJAT AGC-vel normaljuk, es
   * a kettő KULONBSEGE a puha dontes. Igy:
   *   - a deviacio abszolut erteke kiesik (mindket ag ugyanugy skalazodik),
   *   - a jelszint valtozasa kiesik,
   *   - a dontesi kuszob valoban a nulla, allando tartalekkal.
   * Ez az az architektura, amivel a v1 ut 98,6 %-ot vitt egy VALODI
   * FG23-felvetelen — a delay-multiply-t azert csereltuk ra, mert fajlon
   * tobbet hozott, es pont ez volt a tevedes: fajlon nincs zaj.
   *
   * A bitorajel-visszanyeres VALTOZATLAN (a twin bizonyitott fazisablaka),
   * es a HDLC is. Csak a puha dontes eloallitasa mas. */
  {
    /* korgyuru az utolso COR_N mintara */
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

    /* nagysag ~ |I|+|Q| (olcso, a hiba <12 %, es a hanyadosban kiesik) */
    int32_t mm = (mi < 0 ? -mi : mi) + (mq < 0 ? -mq : mq);
    int32_t ss = (si < 0 ? -si : si) + (sq < 0 ? -sq : sq);
    mm >>= 8; ss >>= 8;

    /* hangonkenti AGC: ~100 ms idoallando 9600 Hz-en (>>10 = 107 ms).
     * Lassabb egy bitnel, gyorsabb egy csomagnal — a szintet koveti, az
     * adatot nem. (Ez NEM a SOFT_DC_SHIFT csapdaja: itt nem a dontesi
     * kuszobot atlagoljuk, hanem a ket ag ENERGIAJAT kulon-kulon.) */
    s_agc_m += (mm - s_agc_m) >> 10;
    s_agc_s += (ss - s_agc_s) >> 10;
    int32_t am = s_agc_m > 64 ? s_agc_m : 64;
    int32_t as = s_agc_s > 64 ? s_agc_s : 64;

    /* normalt kulonbseg, Q12-ben. Jel nelkul mindket ag a sajat AGC-jen ul,
     * tehat a kulonbseg nulla korul zajong — nincs hamis vivo-erzekeles. */
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
    /* NRZI: nincs atmenet = 1 */
    bool bit = !TRANSITION_FOUND(s_actualBits);
    hdlc_bit(bit);
  }
}

/* ==================== BELEPESI PONTOK ==================== */

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
    Serial0.println("APRS RX: passcode -1 -> CSAK OLVASO, semmit nem "
                    "toltunk fel. Ird be a sajatodat a gateleshez.");
  }
}

void aprs_rx_feed(const int16_t *iq, int nsamp, uint32_t sps)
{
  if (sps == 0) return;
  if (sps != s_sps) aprs_rx_configure(sps);

  for (int n = 0; n < nsamp; n++) {
    /* --- csatornaszuro: I/Q atlagolas es decimalas ~25 ksps-re --- */
    s_iq_si += iq[2 * n];
    s_iq_sq += iq[2 * n + 1];
    if (++s_iq_n < s_iq_dec) continue;
    int16_t i = (int16_t)(s_iq_si / (int32_t)s_iq_dec);
    int16_t q = (int16_t)(s_iq_sq / (int32_t)s_iq_dec);
    s_iq_si = s_iq_sq = 0;
    s_iq_n = 0;

    /* FM diszkriminator: a mostani es az elozo minta szoge kozti kulonbseg.
     *   d = z[n] * conj(z[n-1])
     * A szog aranyos a pillanatnyi frekvenciaeltolassal, tehat ez maga a
     * demodulalt hang. */
    int32_t dr = (int32_t)i * (int32_t)s_prev_i + (int32_t)q * (int32_t)s_prev_q;
    int32_t di = (int32_t)q * (int32_t)s_prev_i - (int32_t)i * (int32_t)s_prev_q;
    s_prev_i = i; s_prev_q = q;

    /* atan2 Q13 (~+/-pi). Ne shifteljjuk el a hangerot — a LibAPRS
     * delay-multiply erzekeny a kicsi amplitudora (PC: 286->515 skálával). */
    /* NORMALT szamitas: a >>2 A>1024 folott tulcsordult — lasd fm_angle_q13. */
    int32_t a = fm_angle_q13(di, dr);

    /* +/-PI_Q13 -> majdnem teljes int16, mint a PC twin */
    int32_t scaled = (a * 30000) / PI_Q13;

    /* enyhe LPF az IF raten, anti-alias a 9600 ele */
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
  /* A teljes APRS-IS kezeles (async DNS, nem-blokkolo connect, login,
   * olvasas, backoff) egy nem-blokkolo allapotgepbe kerult. Fazisonkent
   * legfeljebb egy nem-blokkolo socket-muvelet — a fo hurok SOSE all meg. */
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
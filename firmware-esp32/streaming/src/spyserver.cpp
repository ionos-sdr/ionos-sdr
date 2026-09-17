/* SPDX-License-Identifier: MIT
 *
 * spyserver.cpp — SpyServer-compatible server
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * See the spyserver.h header. The protocol structures come from Airspy's
 * public spyserver_protocol.h.
 */

#include "spyserver.h"
#include <Arduino.h>
#include <WiFi.h>
#include <lwip/sockets.h>   /* send(fd, ..., MSG_DONTWAIT) — non-blocking */
#include <string.h>

/* ==================== CONFIGURATION ==================== */

/* Number of halving decimation stages offered to the client. 4 stages
 * from 50 ksps: 50 / 25 / 12.5 / 6.25 / 3.125 ksps. WE do the decimation,
 * so a narrow setting really means less network traffic. */
#define SPY_DECIM_STAGES   4u

/* The advertised tuning range. The client allows tuning within it.
 * Adjust to the PHY. */
#define SPY_FREQ_MIN_HZ    100000000u
#define SPY_FREQ_MAX_HZ    1000000000u

/* Output ring. Power of two! (because of the uint32 wrap-around of
 * head/tail). 64 kB = ~330 ms at 50 ksps/16 bit. On Android, due to WiFi
 * power saving, the client drains in BURSTS (every 100-300 ms); the ring
 * must bridge this — with 32 kB it filled up several times per second.
 *
 * Lives on the HEAP, not statically: a 64 kB static array no longer fit
 * into the DRAM segment (the link overflowed by 1232 bytes), whereas the
 * heap has ample room even after boot. If the 64 kB cannot be obtained,
 * we retry halving — the size stays a power of two at run time too. */
#define SPY_RING_BYTES     65536u
#define SPY_RING_MIN       8192u
/* MSS-aligned: 2 x 1440-byte full TCP segments (see the TCP_CHUNK_MAX
 * comment in main.cpp). */
#define SPY_CHUNK_MAX      2880u

/* Complex samples per message. Larger = fewer headers and fewer TCP
 * segments; too large, however, delays the waterfall. */
#define SPY_MSG_SAMPLES    512u

#define DIAG Serial0

/* ==================== PROTOCOL ==================== */

#define SPY_PROTOCOL_VERSION  (((2u) << 24) | ((0u) << 16) | (1700u))
#define SPY_MAX_BODY          (1u << 20)

/* client -> server */
#define CMD_HELLO         0u
#define CMD_GET_SETTING   1u
#define CMD_SET_SETTING   2u
#define CMD_PING          3u

/* server -> client */
#define MSG_DEVICE_INFO   0u
#define MSG_CLIENT_SYNC   1u
#define MSG_PONG          2u
#define MSG_UINT8_IQ    100u
#define MSG_INT16_IQ    101u
#define MSG_UINT8_FFT   301u   /* wideband scan row (STREAM_TYPE_FFT) */

#define STREAM_TYPE_STATUS 0u
#define STREAM_TYPE_IQ     1u
#define STREAM_TYPE_FFT    4u

#define STREAM_FORMAT_UINT8  1u
#define STREAM_FORMAT_INT16  2u

#define SETTING_STREAMING_MODE     0u
#define SETTING_STREAMING_ENABLED  1u
#define SETTING_IQ_FORMAT        100u
#define SETTING_IQ_FREQUENCY     101u
#define SETTING_IQ_DECIMATION    102u
#define SETTING_IQ_DIGITAL_GAIN  103u
/* FFT/scan settings. Standard protocol identifiers, but the INTERPRETATION
 * is ours (agreed with the SDR++ fg23_scan_source module):
 *   FFT_DECIMATION = SPAN in Hz (not a decimation index!) */
#define SETTING_FFT_FORMAT         200u
#define SETTING_FFT_FREQUENCY      201u
#define SETTING_FFT_DECIMATION     202u
#define SETTING_FFT_DB_OFFSET      203u
#define SETTING_FFT_DB_RANGE       204u
#define SETTING_FFT_DISPLAY_PIXELS 205u

#define DEVICE_RTLSDR      3u

typedef struct {
  uint32_t ProtocolID;
  uint32_t MessageType;
  uint32_t StreamType;
  uint32_t SequenceNumber;
  uint32_t BodySize;
} spy_msg_hdr_t;

typedef struct {
  uint32_t DeviceType;
  uint32_t DeviceSerial;
  uint32_t MaximumSampleRate;
  uint32_t MaximumBandwidth;
  uint32_t DecimationStageCount;
  uint32_t GainStageCount;
  uint32_t MaximumGainIndex;
  uint32_t MinimumFrequency;
  uint32_t MaximumFrequency;
  uint32_t Resolution;
  uint32_t MinimumIQDecimation;
  uint32_t ForcedIQFormat;
} spy_device_info_t;

typedef struct {
  uint32_t CanControl;
  uint32_t Gain;
  uint32_t DeviceCenterFrequency;
  uint32_t IQCenterFrequency;
  uint32_t FFTCenterFrequency;
  uint32_t MinimumIQCenterFrequency;
  uint32_t MaximumIQCenterFrequency;
  uint32_t MinimumFFTCenterFrequency;
  uint32_t MaximumFFTCenterFrequency;
} spy_client_sync_t;

/* ==================== STATE ==================== */

static WiFiServer  *s_srv = NULL;
static WiFiClient   s_cli;
static bool         s_have_cli = false;
static spy_tune_fn  s_on_tune = NULL;

static uint32_t s_sps = 0;            /* input rate */
static uint32_t s_freq_hz = 0;        /* current tuning */
static uint32_t s_seq = 0;

static bool     s_streaming = false;
static uint32_t s_decim_stage = 0;    /* 0 = full rate */

/* ---- scan (FFT stream) state ---- */
static bool     s_fft_mode = false;   /* FFT bit in STREAMING_MODE */
static bool     s_iq_mode  = true;    /* IQ bit in STREAMING_MODE */
static uint32_t s_fft_freq_hz = 0;
static uint32_t s_fft_span_hz = 8000000u;
static uint16_t s_fft_nbin    = 320;
static int16_t  s_fft_floor   = -130;
static uint16_t s_fft_range   = 100;
static bool     s_scan_dirty  = false;
static spy_scan_fn s_on_scan  = NULL;
static uint32_t s_fft_lines   = 0;

/* decimation: halving boxcar per stage, merged into one accumulator */
static int32_t  s_acc_i = 0, s_acc_q = 0;
static uint32_t s_acc_n = 0;

/* command reassembler (TCP may split even the header) */
static uint8_t  s_cmd[64];
static uint32_t s_cmd_len = 0;
static uint32_t s_cmd_need = 0;       /* 0 = still waiting for the 8-byte header */

/* output ring (heap, allocated by spy_init) */
static uint8_t *s_ring = NULL;
static uint32_t s_ring_size = 0;      /* power of two */
static uint32_t s_ring_mask = 0;      /* s_ring_size - 1 */
static uint32_t s_head = 0, s_tail = 0, s_dropped = 0;

/* message assembler */
static int16_t  s_msg[SPY_MSG_SAMPLES * 2];
static uint32_t s_msg_n = 0;          /* complex samples in s_msg */

/* ==================== RING ==================== */

static inline uint32_t ring_used(void) { return s_head - s_tail; }

static void ring_put(const uint8_t *p, uint32_t n)
{
  if (!s_ring || ring_used() + n > s_ring_size) { s_dropped++; return; }
  uint32_t idx = s_head & s_ring_mask;
  uint32_t first = s_ring_size - idx;
  if (first > n) first = n;
  memcpy(&s_ring[idx], p, first);
  if (n > first) memcpy(&s_ring[0], p + first, n - first);
  s_head += n;
}

static void ring_flush(void)
{
  /* NEVER BLOCK — direct send() with MSG_DONTWAIT, NOT
   * WiFiClient::write(). The core's availableForWrite() is NOT
   * implemented (always 0), and write() waits in select() with a 1 s
   * timeout when the send buffer is full — this stalled the main loop for
   * spy=90..330 ms and made the phone's waterfall stutter.
   * Detailed rationale at ring_flush in main.cpp. */
  if (!s_ring || !s_have_cli || !s_cli.connected()) return;
  const int fd = s_cli.fd();
  if (fd < 0) return;
  while (ring_used() > 0) {
    uint32_t n = ring_used();
    uint32_t idx = s_tail & s_ring_mask;
    uint32_t contig = s_ring_size - idx;
    if (n > contig)         n = contig;
    if (n > SPY_CHUNK_MAX)  n = SPY_CHUNK_MAX;

    int w = send(fd, &s_ring[idx], n, MSG_DONTWAIT);
    if (w <= 0) break;                    /* full: continue next iteration */
    s_tail += (uint32_t)w;
    if ((uint32_t)w < n) break;
  }
}

/* ==================== MESSAGE SENDING ==================== */

static void send_msg(uint32_t type, uint32_t stream,
                     const void *body, uint32_t len)
{
  /* ALL or NOTHING. The header and body fit together, or the whole
   * message is skipped. If the header went in but the body did not, the
   * client would wait FOREVER for the advertised BodySize bytes — the
   * SDR++ readSize() reads exactly that much, blocking, and the stream
   * cannot resynchronise. On Android, due to WiFi power saving, the client
   * drains in bursts and the ring fills easily: this was the "connects,
   * then freezes immediately" symptom. A skipped message is only a gap in
   * time — a truncated message is fatal. */
  if (!s_ring || ring_used() + sizeof(spy_msg_hdr_t) + len > s_ring_size) {
    s_dropped++;
    return;
  }
  spy_msg_hdr_t h;
  h.ProtocolID     = SPY_PROTOCOL_VERSION;
  h.MessageType    = type;
  h.StreamType     = stream;
  h.SequenceNumber = s_seq++;
  h.BodySize       = len;
  ring_put((const uint8_t *)&h, sizeof h);
  if (len) ring_put((const uint8_t *)body, len);
}

static void send_device_info(void)
{
  spy_device_info_t d;
  memset(&d, 0, sizeof d);
  d.DeviceType           = DEVICE_RTLSDR;
  d.DeviceSerial         = 0x46473233;          /* "FG23" */
  d.MaximumSampleRate    = s_sps;
  d.MaximumBandwidth     = s_sps;
  d.DecimationStageCount = SPY_DECIM_STAGES;
  d.GainStageCount       = 1;
  d.MaximumGainIndex     = 1;
  d.MinimumFrequency     = SPY_FREQ_MIN_HZ;
  d.MaximumFrequency     = SPY_FREQ_MAX_HZ;
  d.Resolution           = 16;
  d.MinimumIQDecimation  = 0;
  /* Declare that we deliver 16 bits. Even if the client asks for something
   * else, int16 is sent — discarding half the data is exactly what this
   * whole protocol was meant to avoid. */
  d.ForcedIQFormat       = STREAM_FORMAT_INT16;
  send_msg(MSG_DEVICE_INFO, STREAM_TYPE_STATUS, &d, sizeof d);
}

static void send_client_sync(void)
{
  spy_client_sync_t c;
  memset(&c, 0, sizeof c);
  c.CanControl               = 1;
  c.Gain                     = 0;
  c.DeviceCenterFrequency    = s_freq_hz;
  c.IQCenterFrequency        = s_freq_hz;
  c.FFTCenterFrequency       = s_freq_hz;
  c.MinimumIQCenterFrequency = SPY_FREQ_MIN_HZ;
  c.MaximumIQCenterFrequency = SPY_FREQ_MAX_HZ;
  c.MinimumFFTCenterFrequency = SPY_FREQ_MIN_HZ;
  c.MaximumFFTCenterFrequency = SPY_FREQ_MAX_HZ;
  send_msg(MSG_CLIENT_SYNC, STREAM_TYPE_STATUS, &c, sizeof c);
}

/* ==================== COMMANDS ==================== */

static void handle_setting(uint32_t id, const uint8_t *val, uint32_t vlen)
{
  uint32_t v = (vlen >= 4) ? *(const uint32_t *)val : 0;

  switch (id) {
    case SETTING_STREAMING_MODE:
      DIAG.printf("spyserver: stream mode = %lu%s%s\n", (unsigned long)v,
                  (v & STREAM_TYPE_FFT) ? " [FFT/scan]" : "",
                  (v & STREAM_TYPE_IQ)  ? " [IQ]" : "");
      s_fft_mode = (v & STREAM_TYPE_FFT) != 0;
      s_iq_mode  = (v & STREAM_TYPE_IQ)  != 0;
      s_scan_dirty = true;
      break;

    /* ---- scan / FFT stream ---- */
    case SETTING_FFT_FORMAT:
      break;                                   /* UINT8 only, ignored */
    case SETTING_FFT_FREQUENCY:
      s_fft_freq_hz = v; s_scan_dirty = true;
      DIAG.printf("spyserver: scan center %lu Hz\n", (unsigned long)v);
      break;
    case SETTING_FFT_DECIMATION:               /* = SPAN Hz */
      if (v >= 100000u && v <= 100000000u) { s_fft_span_hz = v; s_scan_dirty = true; }
      DIAG.printf("spyserver: scan span %lu Hz\n", (unsigned long)v);
      break;
    case SETTING_FFT_DISPLAY_PIXELS:
      if (v >= 100u && v <= SPECLINE_MAX_BINS) { s_fft_nbin = (uint16_t)v; s_scan_dirty = true; }
      break;
    case SETTING_FFT_DB_OFFSET:
      if (v <= 200u) { s_fft_floor = -(int16_t)v; s_scan_dirty = true; }
      break;
    case SETTING_FFT_DB_RANGE:
      if (v >= 10u && v <= 150u) { s_fft_range = (uint16_t)v; s_scan_dirty = true; }
      break;

    case SETTING_STREAMING_ENABLED:
      s_streaming = (v != 0);
      s_msg_n = 0;
      s_acc_i = s_acc_q = 0; s_acc_n = 0;
      s_scan_dirty = true;   /* the scan starts/stops together with the stream */
      DIAG.printf("spyserver: stream %s\n", s_streaming ? "STARTING" : "stopped");
      break;

    case SETTING_IQ_FORMAT:
      if (v != STREAM_FORMAT_INT16) {
        DIAG.printf("spyserver: client requests format %lu, but int16 is "
                    "delivered (ForcedIQFormat)\n", (unsigned long)v);
      }
      break;

    case SETTING_IQ_FREQUENCY:
      DIAG.printf("spyserver: tuning %lu Hz\n", (unsigned long)v);
      s_freq_hz = v;
      if (s_on_tune) s_on_tune(v);
      send_client_sync();
      break;

    case SETTING_IQ_DECIMATION:
      if (v > SPY_DECIM_STAGES) v = SPY_DECIM_STAGES;
      s_decim_stage = v;
      s_acc_i = s_acc_q = 0; s_acc_n = 0;
      s_msg_n = 0;
      DIAG.printf("spyserver: decimation 1/%lu -> %lu sps\n",
                  (unsigned long)(1u << v), (unsigned long)spy_out_sps());
      send_client_sync();
      break;

    case SETTING_IQ_DIGITAL_GAIN:
      break;

    default:
      DIAG.printf("spyserver: unknown setting %lu = %lu\n",
                  (unsigned long)id, (unsigned long)v);
      break;
  }
}

static void handle_command(uint32_t type, const uint8_t *body, uint32_t len)
{
  switch (type) {
    case CMD_HELLO: {
      uint32_t ver = (len >= 4) ? *(const uint32_t *)body : 0;
      DIAG.printf("\nspyserver: HELLO, client protocol %lu.%lu.%lu\n",
                  (unsigned long)(ver >> 24),
                  (unsigned long)((ver >> 16) & 0xFF),
                  (unsigned long)(ver & 0xFFFF));
      s_streaming = false;
      s_decim_stage = 0;
      s_fft_mode = false; s_iq_mode = true; s_scan_dirty = true;
      send_device_info();
      send_client_sync();
      break;
    }
    case CMD_SET_SETTING:
      if (len >= 4) {
        uint32_t id = *(const uint32_t *)body;
        handle_setting(id, body + 4, len - 4);
      }
      break;
    case CMD_GET_SETTING:
      send_client_sync();
      break;
    case CMD_PING:
      send_msg(MSG_PONG, STREAM_TYPE_STATUS, NULL, 0);
      break;
    default:
      DIAG.printf("spyserver: unknown command %lu\n", (unsigned long)type);
      break;
  }
}

/* TCP may split the stream anywhere, so it is reassembled with a state
 * machine: first the 8-byte header, then the BodySize bytes of body. */
static void poll_commands(void)
{
  while (s_have_cli && s_cli.available() > 0) {
    if (s_cmd_need == 0) {
      s_cmd[s_cmd_len++] = (uint8_t)s_cli.read();
      if (s_cmd_len == 8) {
        uint32_t body = *(const uint32_t *)&s_cmd[4];
        if (body > sizeof(s_cmd) - 8) {
          /* Body too large — does not fit our buffer. There is no sensible
           * way to resynchronise, so the connection is dropped; the client
           * will retry. */
          DIAG.printf("spyserver: command too large (%lu B), disconnecting\n",
                      (unsigned long)body);
          s_cli.stop();
          s_have_cli = false;
          return;
        }
        s_cmd_need = body;
        if (s_cmd_need == 0) {
          handle_command(*(const uint32_t *)&s_cmd[0], NULL, 0);
          s_cmd_len = 0;
        }
      }
    } else {
      s_cmd[s_cmd_len++] = (uint8_t)s_cli.read();
      if (s_cmd_len == 8 + s_cmd_need) {
        handle_command(*(const uint32_t *)&s_cmd[0], &s_cmd[8], s_cmd_need);
        s_cmd_len = 0;
        s_cmd_need = 0;
      }
    }
  }
}

/* ==================== ENTRY POINTS ==================== */

void spy_init(uint16_t port, spy_tune_fn on_tune, spy_scan_fn on_scan)
{
  s_on_tune = on_tune;
  s_on_scan = on_scan;

  /* Ring from the heap; if 64 kB is unavailable, halve downwards. */
  s_ring_size = SPY_RING_BYTES;
  while (s_ring_size >= SPY_RING_MIN) {
    s_ring = (uint8_t *)malloc(s_ring_size);
    if (s_ring) break;
    s_ring_size >>= 1;
  }
  if (!s_ring) {
    s_ring_size = 0;
    DIAG.println("spyserver: NO memory for the ring — the port stays deaf!");
  }
  s_ring_mask = s_ring_size - 1u;

  s_srv = new WiFiServer(port);
  s_srv->begin();
  s_srv->setNoDelay(true);
  DIAG.printf("spyserver: TCP %u, 16-bit I/Q, %u decimation stages, "
              "%lu kB ring\n",
              (unsigned)port, (unsigned)SPY_DECIM_STAGES,
              (unsigned long)(s_ring_size / 1024u));
}

void spy_set_rate(uint32_t sps)
{
  if (sps == s_sps) return;
  s_sps = sps;
  s_acc_i = s_acc_q = 0; s_acc_n = 0;
  s_msg_n = 0;
  /* If a client is connected, inform it of the new device data. */
  if (s_have_cli && s_cli.connected()) { send_device_info(); send_client_sync(); }
}

void spy_set_freq(uint32_t hz)
{
  if (hz == s_freq_hz) return;
  s_freq_hz = hz;
  if (s_have_cli && s_cli.connected()) send_client_sync();
}

uint32_t spy_out_sps(void)
{
  return s_sps ? (s_sps >> s_decim_stage) : 0;
}

void spy_feed(const int16_t *iq, int nsamp)
{
  if (!s_have_cli || !s_streaming || !s_cli.connected()) return;
  if (!s_iq_mode) return;              /* FFT_ONLY: no IQ */

  const uint32_t dec = 1u << s_decim_stage;

  for (int n = 0; n < nsamp; n++) {
    /* Boxcar averaging down to the requested decimation. Not the steepest
     * filter, but cheap, and narrowing is the client's decision anyway. */
    s_acc_i += iq[2 * n];
    s_acc_q += iq[2 * n + 1];
    if (++s_acc_n < dec) continue;

    s_msg[2 * s_msg_n]     = (int16_t)(s_acc_i / (int32_t)dec);
    s_msg[2 * s_msg_n + 1] = (int16_t)(s_acc_q / (int32_t)dec);
    s_acc_i = s_acc_q = 0;
    s_acc_n = 0;

    if (++s_msg_n >= SPY_MSG_SAMPLES) {
      send_msg(MSG_INT16_IQ, STREAM_TYPE_IQ, s_msg,
               (uint32_t)(s_msg_n * 2 * sizeof(int16_t)));
      s_msg_n = 0;
    }
  }
}

void spy_tick(uint32_t now)
{
  if (!s_srv) return;

  /* accept() is a socket operation under the lwIP lock. The main loop
   * runs thousands of times per second, and calling it on three servers
   * that often is a needless slowdown — a client will not mind being
   * accepted a quarter second later. */
  static uint32_t last_accept = 0;
  if (now - last_accept >= 250u) {
    last_accept = now;
    WiFiClient c = s_srv->accept();
    if (c) {
      if (s_have_cli) s_cli.stop();
      s_cli = c;
      s_cli.setNoDelay(true);
      s_have_cli = true;
      s_head = s_tail = 0;
      s_cmd_len = 0; s_cmd_need = 0;
      s_streaming = false;
      s_seq = 0;
      DIAG.printf("\n>>> spyserver client: %s\n",
                  s_cli.remoteIP().toString().c_str());
    }
  }

  if (s_have_cli && !s_cli.connected()) {
    DIAG.println("\n<<< spyserver client disconnected");
    s_cli.stop();
    s_have_cli = false;
    s_streaming = false;
    if (s_fft_mode && s_on_scan) s_on_scan(false, 0, 0, 0, 0, 0);
    s_fft_mode = false; s_iq_mode = true;
    return;
  }

  poll_commands();

  /* Scan state changed (mode, stream, parameter): notify main.cpp, which
   * forwards it to the FG23 over the command link (W... / W0). */
  if (s_scan_dirty) {
    s_scan_dirty = false;
    if (s_on_scan) {
      bool want = s_streaming && s_fft_mode;
      s_on_scan(want, s_fft_freq_hz, s_fft_span_hz, s_fft_nbin, s_fft_floor, s_fft_range);
    }
  }

  ring_flush();

  /* If the client drains slower than we produce, show it in the log —
   * this now means only a gap in the stream, not a freeze. */
  static uint32_t last_rep = 0, last_drop = 0;
  if (now - last_rep >= 5000u) {
    last_rep = now;
    if (s_dropped != last_drop) {
      DIAG.printf("spyserver: %lu dropped messages in 5 s "
                  "(client drains slowly — Android power saving?)\n",
                  (unsigned long)(s_dropped - last_drop));
      last_drop = s_dropped;
    }
  }
}

bool     spy_connected(void) { return s_have_cli && s_cli.connected(); }
uint32_t spy_dropped(void)   { return s_dropped; }

/* ==================== SCAN / FFT STREAM ==================== */

bool spy_scan_wanted(void) { return s_have_cli && s_streaming && s_fft_mode; }

/* Forward one SPECLINE row to the client as MSG_UINT8_FFT. The dB scale
 * (floor/range) comes from the FG23 header; if it differs from what the
 * client requested, rescale. ALL or nothing (send_msg). */
void spy_send_specline(const specline_blk_t *L)
{
  if (!spy_scan_wanted()) return;
  uint16_t nbin = L->hdr.nbin;
  if (nbin == 0 || nbin > SPECLINE_MAX_BINS) return;

  /* The upper 16 bits of MessageType (flags) carry the row's CENTER
   * FREQUENCY in 100 kHz units (149.8 MHz -> 1498). From this the client
   * can discard rows still belonging to the previous tuning (during a
   * retune). */
  uint32_t mtype = MSG_UINT8_FFT | (((L->hdr.f_center_hz / 100000u) & 0xFFFFu) << 16);
  if (L->floor_dbm == s_fft_floor && L->range_db == s_fft_range) {
    send_msg(mtype, STREAM_TYPE_FFT, L->bins, nbin);
  } else {
    static uint8_t tmp[SPECLINE_MAX_BINS];
    for (uint16_t i = 0; i < nbin; i++) {
      /* dBm = floor_L + v*range_L/255  ->  v' = (dBm - floor_c)*255/range_c */
      int32_t dbm_x100 = (int32_t)L->floor_dbm * 100
                       + (int32_t)L->bins[i] * (int32_t)L->range_db * 100 / 255;
      int32_t v = (dbm_x100 - (int32_t)s_fft_floor * 100) * 255
                / ((int32_t)s_fft_range * 100);
      if (v < 0) v = 0; if (v > 255) v = 255;
      tmp[i] = (uint8_t)v;
    }
    send_msg(mtype, STREAM_TYPE_FFT, tmp, nbin);
  }
  s_fft_lines++;
}

uint32_t spy_fft_lines(void) { return s_fft_lines; }

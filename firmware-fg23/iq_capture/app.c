/* SPDX-License-Identifier: MIT
 *
 * FG23 phase-coherent I/Q capture + TX eval + continuous I/Q stream
 * Simplicity SDK 2025.6.3 / RAIL 2.19.x — for the "RAIL - SoC Empty" project
 *
 * The capture core derives from dsp_driver.c of the geckokapula project:
 *   Copyright (c) 2017-2022 Tatu Peltola (OH2EAT) — MIT license
 *   https://github.com/tejeez/geckokapula
 * Series 2 port and burst dump:
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * PLACEMENT: in the SoC Empty template app_init() belongs in app_init.c and
 * app_process_action() in app_process.c — or everything can go into a
 * single app.c if the template's empty functions are deleted.
 *
 * COMPONENTS (Software Components):
 *   - RAIL Utility, Initialization  (instance: inst0; "Enable Setup of
 *     Radio Events" ON — this provides the sl_rail_util_on_event routing)
 *   - your own Radio Configurator PHY
 *   - IO Stream: EUSART (instance: vcom) + IO Stream: Retarget STDIO
 *   - emlib USART (for the I2S output!) + DMADRV (for the LDMA)
 *
 * WHY THE DUMP IS BINARY AND WHY sl_iostream_write:
 *   the retargeted printf/stdout path may buffer and perform LF
 *   conversion, which would corrupt the binary frame — the payload
 *   therefore goes directly via sl_iostream_write.
 *
 * SAMPLE FORMAT (geckokapula dsp.h + RAILtest forensics):
 *   struct { int16_t q, i; } — Q FIRST, little-endian ("i16le" on the
 *   Python side). The IQB1 mode of iq_view_stream.py expects exactly this.
 *
 * ---------------- TX EVALUATION (Phase 3) ----------------
 * Instead of the Series 1 geckokapula synth_set_channel, the official
 * Series 2 path: RAIL_StartTxStream() with CARRIER_WAVE / PN9 mode, fine
 * tuning with RAIL_SetFreqOffset() (unit: synth tick, 4.649 Hz on the
 * FG23, limited to 15 bits -> approx. +/-80 kHz).
 *
 * ---------------- I/Q CHAIN (Phase 4) ----------------
 * 'k' — rate-ceiling benchmark (fs, OVR, FIFO peak, CPU%)
 * 'n' — zero-copy (NULL destination) read trial and measurement
 * 'i' — continuous decimated stream (CIC + DC block), over I2S to the ESP32-S3
 *
 * ---------------- OUTPUT: SPI (NOT I2S) ----------------
 * Per the 2026-07-31 measurement the CS of the FG23 USART cannot produce
 * a true I2S word select (33.6% duty cycle instead of 50%, in all eight
 * framing combinations). The output therefore moved to SPI:
 *
 *   EXP 15 / PC05 -> SCLK   (several MHz)   -> ESP GPIO5
 *   EXP 10 / PC00 -> MOSI   (several MHz)   -> ESP GPIO6
 *   EXP  6 / PC03 -> CS     (per block!)    -> ESP GPIO4
 *
 * The block header carries a SEQUENCE NUMBER, so packet loss becomes a
 * printable number, not a guess.
 *
 * The 'g' pin test remains, with two uses: it checks the physical wiring,
 * AND it toggles all three pins at EXACTLY 50%, so it is a DUTY-CYCLE
 * REFERENCE for multimeter measurement (1.65 V @ 3.3 V logic). This
 * measurement caught the I2S fault after every software lead had run out.
 *
 * SAFETY: in CW the chip outputs up to +10..+20 dBm. NEVER directly into
 * an SDR input — at least 30-40 dB attenuation or a dummy load!
 */

#include "rail.h"
#include "sl_rail_util_init.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"
#include "aprs_beacon.h"
#include "iq_bench.h"
#include "iq_stream.h"
#include "cw_morse.h"
#include "em_gpio.h"
#include "em_cmu.h"

/* ---- Second command input: ESP32 GPIO4 -> FG23 PA06 / EXP 11. ----
 * Through this the phone (rtl_tcp -> ESP -> here) can tune. OFF BY
 * DEFAULT, because a new source file (cmdlink.c) must be added to the
 * project AND a wire connected. Set to 1 once both are done. */
#define CMDLINK_ENABLE 1   /* 2026-08-15: yellow jumper EXP11/PA06 <- GPIO4 connected */
#if CMDLINK_ENABLE
#include "cmdlink.h"
#endif
#include "scan.h"       /* wideband RSSI scan (SPECLINE over SPI) */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* ---------------- configuration ---------------- */

/* Q first! (geckokapula iq_in_t) */
typedef struct {
  int16_t q;
  int16_t i;
} iq_in_t;

/* Burst length: 8192 complex samples = 32 KiB RAM (fits easily in the
 * 64 KiB of the FG23B). ~170 ms at 48 kHz I/Q, ~51 ms at 160 kHz. */
#define CAPTURE_SAMPLES   8192u

/* Complex samples read per event. geckokapula read 2 (for audio sync);
 * for us 64 samples / event = 256 bytes is fine, with a rarer IRQ. */
#define SAMPLES_PER_EVENT 64u
#define THRESHOLD_BYTES   (SAMPLES_PER_EVENT * sizeof(iq_in_t))

/* Series 2: the app provides the RX FIFO via RAILCb_SetupRxFifo. */
#define RX_FIFO_BYTES     4096u

/* The channel on which the PHY outputs the base frequency (config dependent). */
#define IQ_CHANNEL        0u

/* ---- Frequency calibration ----
 *
 * FINAL: 2026-08-04, measured with a Signal Hound BB60C (conclusive, no
 * I/Q ambiguity, unlike the earlier HackRF recording).
 *   TX @ 144.8 MHz (IQ_CHANNEL=0, base 144800):
 *     o0   -> 144.797780 MHz   (raw, without correction, -2220 Hz)
 *     o482 -> 144.800001 MHz   (dead-on)  <-- THIS IS THE MEASURED CALIBRATION
 *   Crystal error ~-15.3 ppm (SLOW crystal), resolution ~4.60 Hz/tick.
 *
 * The ppb is chosen so that corr_tick_for_khz() gives exactly 482 ticks
 * at 144.8 MHz AT BOOT, so the carrier is on frequency right at power-up —
 * no need to type 'o482' by hand (fool-proof):
 *     144800 kHz * 15476 ppb = 2240.9 Hz  -> / 4.6492 = 481.99 -> 482 ticks  OK
 * The ppb is frequency-proportional, so the correction stays valid across
 * the WHOLE 2 m grid (144.800 + n*25 kHz) (the VCO divider is constant
 * within the band).
 *
 * ================== NOTE: THIS IS CURRENTLY TUNED FOR 2 m (144.8) ==================
 * The synth tick Hz/tick is BAND-DEPENDENT (VCO divider step): 4.60 Hz/tick
 * @144.8, but ~11.4 Hz/tick @433. So ONE ppb does NOT fit both bands!
 *   - 2 m  (144.8):  FREQ_CORR_PPB = 15476  -> 482 ticks   (MEASURED 2026-08-04)
 *   - 70 cm (434):   FREQ_CORR_PPB =  5876  -> 546..549 ticks (MEASURED, separately)
 * This 15476 value would give ~1445 ticks at 434 MHz = ~16 kHz off! When
 * changing band, rewrite the ppb from the table above (and TUNE_BASE_KHZ
 * from the Radio Config).
 *
 * This is specific to THIS BRD4265B UNIT — another board must be measured
 * again. On the final PCB the GPS+VCTCXO automates the same. */
#define FREQ_CORR_PPB     15476     /* 2026-08-04 BB60C: 144.8 MHz -> 482 ticks (-15.3 ppm) */

/* ================== AUTOSTART ==================
 * After power-up the stream starts by itself, so the unit works standalone
 * (no terminal, no command needed). The terminal still works FULLY: any
 * key stops the stream, after which every command is available,
 * including 'i<R>' to restart.
 *
 *   IQ_AUTOSTART        1 = start automatically, 0 = keep the old behaviour
 *   IQ_AUTOSTART_DECIM  the TOTAL decimation (multiple of 8, 8..4096)
 *                         8 ->  50 000 sps  (+-25 kHz)
 *                        16 ->  25 000 sps
 *                        32 ->  12 500 sps  <- this is "i32"
 *                       256 ->   1 562 sps
 *   IQ_AUTOSTART_SHIFT  extra gain as a power of two (0 = none)
 *   IQ_AUTOSTART_DELAY_MS  wait this long after start so the welcome text
 *                       reaches the terminal and the ESP32 can come up
 *                       (its boot takes ~700 ms) */
/* 2026-08-01: 32 -> 8. Because of the rtl_tcp path. The lowest standard
 * client rate is 250 ksps, and the ESP upsamples to it by an integer
 * factor:
 *     i32 -> 12500 sps -> 20x upsampling -> the spectrum looks "hazy",
 *                          because the 12.5 kHz band is stretched to 250
 *     i8  -> 50000 sps ->  5x            -> four times the REAL band, and
 *                          the images fall four times further away
 * Cost: at i8 only the first, SECOND-ORDER CIC stage filters, so aliases
 * may appear towards the band edges, and with a strong signal ~2% clipping
 * was seen. If you measure WSPR/FT8 and need clean band edges, set it
 * back to 16. */
#define IQ_AUTOSTART            1
#define IQ_AUTOSTART_DECIM      8u
#define IQ_AUTOSTART_SHIFT      0u
#define IQ_AUTOSTART_DELAY_MS   800u

/* ================== TUNING ('F' command) ==================
 * 'F<kHz>' tunes to an absolute frequency: the coarse step is the PHY
 * channel grid, the remainder is carried by the synth fine offset (1 tick
 * = 4.6492 Hz, 15 bits, so room up to approx. +-152 kHz, usable in
 * practice up to +-80 kHz). Together the two give CONTINUOUS tuning.
 *
 * ================== THESE THREE NUMBERS MUST BE MIRRORED ==================
 * TUNE_BASE_KHZ / TUNE_SPACING_KHZ / TUNE_MAX_CHANNEL must mirror the base
 * frequency, channel spacing and channel count set in the Radio
 * Configurator. If they do not match, tuning goes off SILENTLY — hence
 * the boot message prints them, and so does 's'.
 *
 * CHANNELS ONLY GO UPWARDS: the RAIL channel number is unsigned, so BELOW
 * the base only the +-76 kHz fine offset is available. To cover the whole
 * 70 cm band the base must therefore be placed at the BOTTOM of the band.
 *
 * RECOMMENDED PHY (Radio Configurator):
 *     base frequency  430.000 MHz
 *     channel spacing  25 kHz
 *     number of channels 401        -> 430.000 ... 440.025 MHz continuously
 * and then here: TUNE_BASE_KHZ 430000, TUNE_SPACING_KHZ 25, MAX_CHANNEL 400.
 *
 * ================== 2026-08-03: THIS BIT US ONCE ALREADY ==================
 *
 * The value was 434000 while the PHY sat at 144.8 MHz. Reception STILL
 * WORKED — the frequency comes from the RAIL PHY, not this define. This
 * number is only the software's BELIEF about where channel 0 is.
 *
 * The damage came through the CRYSTAL CORRECTION. At start-up:
 *     set_freq_tick(corr_tick_for_khz(current_khz()))
 * and current_khz() computes from this base. So the correction meant for
 * 434 MHz was applied to a 144.8 MHz reception:
 *
 *     434.0 MHz * 4.38 ppm = 1901 Hz  -> 409 ticks   (this was applied)
 *     144.8 MHz * 4.38 ppm =  634 Hz  -> 136 ticks   (this was needed)
 *     ---------------------------------------------------------------
 *     over-correction                   1268 Hz
 *
 * And indeed: with the HackRF (TCXO, +-0.5 ppm = +-72 Hz at 144.8, i.e.
 * practically exact) reception came in shifted by ~1.3 kHz. It was not the
 * HackRF that was wrong — we were.
 *
 * The tuning commands (f<khz>) also compute the channel from this, so they
 * would have been off as well.
 *
 * LESSON: if you change the PHY, CHANGE THIS TOO. The boot message prints
 * the assumed frequency and the correction computed from it — if that does
 * not match where you are actually listening, this is the fault. */
#define TUNE_BASE_KHZ      144800u
#define TUNE_SPACING_KHZ   25u        /* 0 = no channel grid, offset only */
#define TUNE_MAX_CHANNEL   800u       /* channel count configured in the PHY - 1
                                       * 2026-08-15: 400 -> 800 (together with
                                       * the radioconf): 144.8..164.8 MHz =
                                       * 20 MHz scan band. Towards the band
                                       * edge sensitivity may degrade due to
                                       * the analog input matching — measure! */
#define TUNE_TICK_MHZ      4.6492     /* Hz / tick, FG23 @ 39 MHz */

/* Current channel (settable from the UART) */
static volatile uint16_t s_channel = IQ_CHANNEL;

/* The CURRENT synth offset in ticks. By default the crystal correction,
 * but the 'F' and 'o' commands change it — so restart_rx() must NOT write
 * back the fixed calibration value, as that would ruin the tuning on every
 * RX restart. (This was exactly the bug in the old code.) */
static volatile int32_t s_freq_tick = 0;   /* set by app_init */

/* RSSI trigger state — had to be moved up, because the stream starter
 * (stream_start_R) is already called by app_init. */
static bool armed = false;
static int16_t arm_thresh_qdbm = 4 * (-95);   /* -95 dBm, quarter-dBm */

/* ---------------- state ---------------- */

static iq_in_t capture_buf[CAPTURE_SAMPLES];
static volatile uint32_t capture_idx  = 0;
static volatile bool     capturing    = false;
static volatile bool     capture_done = false;
static volatile bool     capture_bad  = false;
static volatile uint32_t stat_events = 0, stat_short_reads = 0,
                         stat_overflows = 0;

/* uint32_t-backed buffer = guaranteed 4-byte alignment, without macros */
static uint32_t rx_fifo_words[RX_FIFO_BYTES / 4];
#define rx_fifo ((uint8_t *)rx_fifo_words)

static RAIL_Handle_t s_rail = NULL;

/* ---------------- RX FIFO (Series 2) ----------------
 * If the linker reports "multiple definition of RAILCb_SetupRxFifo", the
 * project already provides its own (e.g. some example source) — then
 * delete THIS function and set the size in the other one. */
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle)
{
  uint16_t size = RX_FIFO_BYTES;
  RAIL_Status_t st = RAIL_SetRxFifo(railHandle, rx_fifo, &size);
  return st;
}

/* ---------------- event callback ----------------
 * ISR context (RAIL almost always calls from an interrupt) — only FIFO
 * reading and indexing, nothing else. A direct descendant of the
 * geckokapula rail_callback(). */
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  /* Benchmark mode ('k'): during measurement the module's own drain and
   * counters run, the capture branch here is skipped entirely. */
  if (iq_bench_on_event(rail_handle, events)) return;

  /* Continuous stream mode ('i'): has its own in-place processing branch. */
  if (iq_stream_on_event(rail_handle, events)) return;

  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    ++stat_overflows;
    if (capturing) capture_bad = true;
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    ++stat_events;

    if (!capturing) {
      /* Continuous reception without capturing: drain UNTIL EMPTY and
       * discard, so the FIFO never overflows — this makes the burst start
       * instantaneous and coherent from the first sample.
       *
       * THE DRAIN CONDITION MAY NEVER BE HIGHER THAN THE EVENT THRESHOLD.
       * If THRESHOLD_BYTES (256) stood here and someone set the radio to
       * a lower threshold (e.g. the 'k' benchmark to 128), this loop
       * would never enter, drain nothing, and the event would fire again
       * endlessly -> interrupt storm, freeze. Hence we drain down to a
       * small, FIXED residue that is below every sensible threshold. */
      #define DRAIN_RESIDUE_BYTES  64u
      static uint8_t sink[THRESHOLD_BYTES];
      uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
      uint8_t guard = 32u;            /* 32 * 256 B = 8 KiB > whole FIFO */
      while (avail >= DRAIN_RESIDUE_BYTES && guard--) {
        uint16_t chunk = (avail > sizeof sink) ? (uint16_t)sizeof sink
                                               : avail;
        chunk = (uint16_t)(chunk & ~(sizeof(iq_in_t) - 1u));
        if (chunk == 0u) break;
        if (RAIL_ReadRxFifo(rail_handle, sink, chunk) == 0u) break;
        avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
      }
      return;
    }

    /* Read UNTIL EMPTY into the capture buffer. If the ISR arrives on
     * time this is a single read — i.e. behaviour identical to before.
     * If it is ever late, the old code SILENTLY lost samples (phase break
     * in the middle of the burst!), whereas this one tries to catch up. */
    uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
    uint8_t guard = 16u;              /* hard limit: the ISR ALWAYS exits */
    while (avail >= THRESHOLD_BYTES
           && capture_idx < CAPTURE_SAMPLES
           && guard--) {
      uint32_t room = (CAPTURE_SAMPLES - capture_idx) * sizeof(iq_in_t);
      uint32_t want = THRESHOLD_BYTES;
      if (want > room)  want = room;
      if (want > avail) want = avail;
      want &= ~(sizeof(iq_in_t) - 1u);
      if (want == 0u) break;

      uint16_t nread = RAIL_ReadRxFifo(rail_handle,
                                       (uint8_t *)&capture_buf[capture_idx],
                                       (uint16_t)want);
      if (nread != want) ++stat_short_reads;
      if (nread == 0u) break;
      capture_idx += nread / sizeof(iq_in_t);
      avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
    }

    if (capture_idx >= CAPTURE_SAMPLES) {
      capturing = false;
      capture_done = true;
    }
  }
}

/* ---------------- burst dump ----------------
 * Self-describing frame: "IQB1" | u32 n | u32 fs_hint | u8 fmt(=1) | 3x pad |
 * payload | "IQE1" — read by the --port/--file mode of iq_view_stream.py. */
static void dump_capture(uint32_t fs_hint)
{
  static const uint8_t hdr[4] = { 'I', 'Q', 'B', '1' };
  static const uint8_t trl[4] = { 'I', 'Q', 'E', '1' };
  uint32_t n = capture_idx;
  uint8_t meta[12];
  memcpy(&meta[0], &n, 4);
  memcpy(&meta[4], &fs_hint, 4);
  meta[8] = 1; meta[9] = meta[10] = meta[11] = 0;

  sl_iostream_write(sl_iostream_vcom_handle, hdr, sizeof hdr);
  sl_iostream_write(sl_iostream_vcom_handle, meta, sizeof meta);
  sl_iostream_write(sl_iostream_vcom_handle, capture_buf,
                    n * sizeof(iq_in_t));
  sl_iostream_write(sl_iostream_vcom_handle, trl, sizeof trl);
}

/* ---------------- init + main loop ---------------- */

/* Forward declarations: used by app_init/the main loop, but defined
 * further below. */
static void pb0_init(void);
static void aprs_send_beacon(void);
static void beacon_max_power(void);
static void offset_speed_benchmark(void);
static void nco_tone_test(double f_tone);
static void restart_rx(void);
static void handle_line(const char *line);

/* The crystal correction in TICKS, for the given frequency. See
 * FREQ_CORR_PPB for why this cannot be a constant. */
static int32_t corr_tick_for_khz(uint32_t khz)
{
  double hz = (double)khz * 1000.0 * (double)FREQ_CORR_PPB / 1e9;
  return (int32_t)(hz / TUNE_TICK_MHZ + (hz >= 0 ? 0.5 : -0.5));
}

/* The currently tuned frequency in kHz, from the channel and the offset. */
static uint32_t current_khz(void)
{
  return (uint32_t)((long)TUNE_BASE_KHZ + (long)s_channel * (long)TUNE_SPACING_KHZ);
}

/* ---------------- the SINGLE path for setting the synth offset ----------------
 * Clamps, stores, writes it into the radio, AND tells the iq_stream module
 * too — because it restarts RX in several places, and RAIL_StartRx does
 * not preserve the offset. If set differently anywhere, the two will
 * sooner or later drift apart and the receiver will silently be off. */
static void set_freq_tick(int32_t tick)
{
  /* RAIL_FREQUENCY_OFFSET_MIN/MAX = -+0x3FFF */
  if (tick >  16383) tick =  16383;
  if (tick < -16383) tick = -16383;
  s_freq_tick = tick;
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)tick);
  iq_stream_set_freq_tick(tick);
}

/* ---------------- stream starter (common path) ----------------
 * Both the autostart AND the 'i<R>' command call THIS, so that the printed
 * text and the actual behaviour are guaranteed to be the same. */
/* The currently running (or last used) decimation — the restart after
 * tuning uses it to know what to restart with. */
static uint32_t s_stream_R = IQ_AUTOSTART_DECIM;
/* Whether the stream was running before the scan started (resumes after W0). */
static bool s_scan_resume = false;
#if CMDLINK_ENABLE
/* Yield from the scan's waits: processes one cmdlink line (if any). */
static void cmdlink_yield(void) { cmdlink_poll(); }
#endif

static void stream_start_R(uint32_t R, uint8_t shift, const char *honnan)
{
  if (R < 8u)    R = 8u;
  if (R > 4096u) R = 4096u;
  R = (R / 8u) * 8u;
  s_stream_R = R;
  armed = false;

  /* The output rate is asked from the MODULE, not computed with our own
   * formula. The old version assumed a fixed 1 Msps input (1000000/R) and
   * printed "31250 sps" at 400 ksps where 12500 actually went out. */
  uint32_t sps = iq_stream_out_sps((uint16_t)R);
  printf("# stream start [%s]: R=%lu -> %lu sps (band +-%lu.%lu kHz), "
         "%lu B/s\r\n",
         honnan, (unsigned long)R, (unsigned long)sps,
         (unsigned long)(sps / 2000u), (unsigned long)((sps / 200u) % 10u),
         (unsigned long)(sps * 4u));
  printf("# ANY key = stop, after which every command is available\r\n");
  iq_stream_start(s_rail, s_channel, (uint16_t)R, shift);
  /* Belt and braces: the module already rewrites the offset by itself
   * after every StartRx (iq_stream_set_freq_tick), but it is set here as
   * well, so that a forgotten initial call cannot silently detune the
   * receiver. */
  set_freq_tick(s_freq_tick);
}

/* ---------------- 'F<kHz>' : absolute tuning ----------------
 * coarse = channel grid, fine = synth offset. The crystal correction (the
 * ppm correction computed from the frequency) is INCLUDED in the result. */
static void tune_khz(uint32_t khz)
{
  int32_t d_khz = (int32_t)khz - (int32_t)TUNE_BASE_KHZ;
  int32_t ch = 0;

#if (TUNE_SPACING_KHZ > 0)
  /* round to the nearest channel (correct in the negative direction too) */
  int32_t sp = (int32_t)TUNE_SPACING_KHZ;
  ch = (d_khz >= 0) ? (d_khz + sp / 2) / sp
                    : (d_khz - sp / 2) / sp;
  if (ch < 0) ch = 0;
  if (ch > (int32_t)TUNE_MAX_CHANNEL) ch = (int32_t)TUNE_MAX_CHANNEL;
  int32_t res_hz = (d_khz - ch * sp) * 1000;
#else
  int32_t res_hz = d_khz * 1000;
#endif

  int32_t tick = (int32_t)(res_hz / TUNE_TICK_MHZ) + corr_tick_for_khz(khz);

  /* RAIL_FrequencyOffset_t is 15-bit signed. On overflow we do NOT tune
   * blindly: we report that the channel grid must be changed. */
  if (tick > 16383 || tick < -16383) {
    if (d_khz < 0) {
      printf("# %lu kHz is BELOW the PHY base (%lu kHz), and the fine "
             "offset is only +-76 kHz. The channel number is unsigned, "
             "there is no grid downwards.\r\n",
             (unsigned long)khz, (unsigned long)TUNE_BASE_KHZ);
      printf("# FIX: in the Radio Configurator move the base frequency to "
             "430.000 MHz (spacing 25 kHz, 401 channels), and set "
             "TUNE_BASE_KHZ here to 430000 as well.\r\n");
    } else {
      printf("# %lu kHz not reachable: the remainder %ld Hz is too large "
             "(%ld ticks, max +-16383). Raise TUNE_MAX_CHANNEL, or "
             "set a smaller spacing.\r\n",
             (unsigned long)khz, (long)res_hz, (long)tick);
    }
    return;
  }

  s_channel = (uint16_t)ch;
  set_freq_tick(tick);
  restart_rx();

  printf("# tuning: %lu kHz = base %lu + ch %ld * %u kHz + %ld Hz "
         "(offset %ld ticks)\r\n",
         (unsigned long)khz, (unsigned long)TUNE_BASE_KHZ, (long)ch,
         (unsigned)TUNE_SPACING_KHZ, (long)res_hz, (long)tick);
}

void app_init(void)
{
  s_rail = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

  RAIL_DataConfig_t dc = {
    .txSource = TX_PACKET_DATA,
    .rxSource = RX_IQDATA_FILTLSB,   /* alternative for strong signals: FILTMSB */
    .txMethod = PACKET_MODE,
    .rxMethod = FIFO_MODE,
  };
  RAIL_ConfigData(s_rail, &dc);
  RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
  RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(s_rail, false, true);
  RAIL_StartRx(s_rail, s_channel, NULL);

  iq_stream_init(rx_fifo, RX_FIFO_BYTES);
  /* Calibration — via set_freq_tick, so the iq_stream module gets it too.
   * If only RAIL_SetFreqOffset were called, starting the stream (which
   * restarts RX) would silently discard it. */
  set_freq_tick(corr_tick_for_khz(current_khz()));

  {
    scan_grid_t g = {
      .base_khz    = TUNE_BASE_KHZ,
      .spacing_khz = TUNE_SPACING_KHZ,
      .max_channel = TUNE_MAX_CHANNEL,
      .tick_hz     = TUNE_TICK_MHZ,
      .corr_ppb    = FREQ_CORR_PPB,
    };
    scan_init(s_rail, &g);
#if CMDLINK_ENABLE
    /* The scan drains the cmdlink FIFO (16 bytes!) during its waits —
     * without this the lines of rapidly arriving W commands get corrupted
     * (2026-08-15: "cmdlink: 39 lines, 20 errors", 100/1016 bins instead
     * of 640). */
    scan_set_yield(cmdlink_yield);
#endif
  }

  pb0_init();
  sl_iostream_set_default(sl_iostream_vcom_handle);
  printf("\r\n# FG23 IQ capture + TX eval (SiSDK 2025.6). "
         "c=capture s=status a=auto-trigger\r\n");
  printf("# F<kHz>=ABSOLUTE tuning (e.g. 'F433775'), f<ch>=raw channel, "
         "o<tick>=fine offset\r\n");
  printf("# t<dBm>=trigger threshold\r\n");
  printf("# TX: w=CW p=PN9 x=stop d<dBm>=power b=APRS beacon — "
         "ATTENUATOR/DUMMY LOAD in front of the SDR!\r\n");
  printf("# NCO test: z=offset benchmark y[<Hz>]=test tone (def 1200)\r\n");
  printf("# CW Morse: M1=CQ  M2=VVV  M3=beacon  M <text>  M/WPM <text>\r\n");
  printf("# k[<s>]=I/Q rate benchmark   n[<s>]=zero-copy (NULL) test\r\n");
  printf("# i[<R>]=continuous decimated I/Q stream (R=8..4096)\r\n");
  printf("# SPI output:  g[<s>]=pin test + duty-cycle reference (1.65 V)  "
         "v=clock/route diag\r\n");
  printf("# PB0 button = MAX POWER APRS beacon (dummy load!)\r\n");
  printf("# tuning grid: base %lu kHz + ch * %u kHz, ch max %u — "
         "MUST MATCH THE RADIO CONFIGURATOR!\r\n",
         (unsigned long)TUNE_BASE_KHZ, (unsigned)TUNE_SPACING_KHZ,
         (unsigned)TUNE_MAX_CHANNEL);
  /* The correction NUMERICALLY, not just the ppm. If the "assumed" is not
   * where you are actually listening, the correction is wrong too — and
   * the signal will be shifted by exactly that much. It once took 1268 Hz
   * this way. */
  {
    uint32_t k = current_khz();
    int32_t  c = corr_tick_for_khz(k);
    printf("# crystal correction: %ld ppb -> at the assumed %lu kHz "
           "%ld ticks = %ld Hz\r\n",
           (long)FREQ_CORR_PPB, (unsigned long)k, (long)c,
           (long)(c * TUNE_TICK_MHZ));
  }

#if CMDLINK_ENABLE
  /* The SAME command handler as the terminal's. One parser, one
   * behaviour — there cannot be two separate truths about what 'F' does. */
  cmdlink_init(handle_line);
  printf("# cmdlink: PA06 / EXP 11 <- ESP GPIO4, 115200 — tuning from "
         "the phone\r\n");
#endif

#if IQ_AUTOSTART
  /* Wait a little: let the text above go out, and let the ESP32 come up
   * (its boot takes ~700 ms). Not sl_sleeptimer, because it may not be
   * initialised yet — the RAIL clock, however, is already running. */
  {
    RAIL_Time_t t = RAIL_GetTime() + IQ_AUTOSTART_DELAY_MS * 1000u;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
  }
  printf("# --- AUTOSTART (IQ_AUTOSTART_DECIM = %u) ---\r\n",
         (unsigned)IQ_AUTOSTART_DECIM);
  stream_start_R(IQ_AUTOSTART_DECIM, (uint8_t)IQ_AUTOSTART_SHIFT, "autostart");
#else
  printf("# autostart OFF (IQ_AUTOSTART=0) — start manually: i32\r\n");
#endif
}

/* Restart RX on the current channel (after a frequency change).
 * The synth offset is restored from s_freq_tick, NOT from the fixed
 * calibration value — otherwise every restart would discard the tuning
 * set with 'F'. */
static void restart_rx(void)
{
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(s_rail, false, true);
  RAIL_StartRx(s_rail, s_channel, NULL);
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
}

/* ---------------- TX stream (Phase 3: TX evaluation) ----------------
 * RAIL_StartTxStream itself stops every ongoing radio operation, but the
 * capture state must be kept consistent by us, so transmission is not
 * allowed while a capture is running. */
static volatile bool s_tx_active = false;
static RAIL_StreamMode_t s_tx_mode = RAIL_STREAM_CARRIER_WAVE;

static void tx_stream_start(RAIL_StreamMode_t mode)
{
  if (capturing) {
    printf("# capture running — wait for the end of the burst (or restart)\r\n");
    return;
  }
  armed = false;                      /* no RSSI trigger during TX */
  if (s_tx_active) {
    RAIL_StopTxStream(s_rail);
    s_tx_active = false;
  }
  s_tx_mode = mode;
  RAIL_Status_t st = RAIL_StartTxStream(s_rail, s_channel, mode);
  s_tx_active = (st == RAIL_STATUS_NO_ERROR);
  if (s_tx_active) RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
  printf("# TX %s ch=%u (st=%d)%s\r\n",
         (mode == RAIL_STREAM_CARRIER_WAVE) ? "CW" : "PN9",
         (unsigned)s_channel, (int)st,
         s_tx_active ? " — TRANSMITTING! attenuator in front of the SDR!" : "");
}

static void tx_stream_stop(void)
{
  if (s_tx_active) {
    RAIL_StopTxStream(s_rail);
    s_tx_active = false;
  }
  restart_rx();
  printf("# TX stop — back to RX (ch=%u)\r\n", (unsigned)s_channel);
}

/* ---------------- APRS direct-FSK beacon (T5) ---------------- */
#include "station_config.h"   /* APRS_MYCALL, APRS_INFO (position) — git-ignored, see station_config.example.h */
#define APRS_SSID     12                 /* -12: experimental/other station */
#define APRS_DEST     "Z2LABS"           /* toCall (device identifier) */
#define APRS_VIA      "WIDE1"            /* digipeater path: WIDE1-1 */
#define APRS_VIA_SSID 1                  /* without it the digi does not repeat! */
/* APRS position report: '!' = position without timestamp.
 * Format: !DDMM.mmN/DDDMM.mmE<sym><comment> */

/* ---- NCO-AFSK modulator parameters (based on the sweep measurements) ----
 * fs=38.4 kHz: THD 2.1%, spur -41 dBc at +-38.4 kHz (easily filtered),
 * load 13%, and EXACTLY 32 samples / bit at 1200 baud -> the bit time
 * comes from sample counting, without a separate bit clock, drift-free.
 * The phase accumulator is NOT reset at the mark/space transition ->
 * phase-continuous Bell 202, as the standard requires. */
#define AFSK_FS            38400.0
#define AFSK_MARK_HZ       1200.0
#define AFSK_SPACE_HZ      2200.0
#define AFSK_SAMPLES_PER_BIT 32u        /* 38400 / 1200 */
#define AFSK_DEV_TICK      645          /* sine-peak offset: +-3 kHz */

/* ---------------- PB0 button -> max-power beacon ----------------
 * Which FG23 pin the button lands on is decided by the RADIO BOARD — so
 * it is NOT hard-coded but taken from the Simplicity board-support
 * header. Provided by the "Simple Button" component (instance: btn0). */
#include "sl_simple_button_btn0_config.h"
#define PB0_PORT   SL_SIMPLE_BUTTON_BTN0_PORT
#define PB0_PIN    SL_SIMPLE_BUTTON_BTN0_PIN

static void pb0_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPull, 1 /* pull-up */);
}

/* Max-power beacon: sets the PA to its true maximum, sends one position
 * report, then restores the previous power.
 *
 * IMPORTANT: the RAIL_TX_POWER_MAX sentinel is NOT used on the raw path —
 * it is 0x7FFF, but the raw type of RAIL_SetTxPower is unsigned char
 * (8 bit), so it would truncate to 255. The dBm path is used instead with
 * an excessively high request: the PA conversion clips it to the real
 * achievable maximum. */
static void beacon_max_power(void)
{
  RAIL_TxPower_t prev = RAIL_GetTxPower(s_rail);   /* save raw units */
  RAIL_Status_t st = RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)200);
  printf("# PB0 -> MAX POWER beacon (actual %d ddBm, st=%d)\r\n",
         (int)RAIL_GetTxPowerDbm(s_rail), (int)st);
  aprs_send_beacon();
  RAIL_SetTxPower(s_rail, prev);                   /* back to raw */
}

/* ================= NCO / AFSK (T5b) ================= */

#define SINE_BITS   8
#define SINE_LEN    (1u << SINE_BITS)     /* 256 points */
#define PHASE_BITS  32                     /* 32-bit phase accumulator */

static int8_t s_sine[SINE_LEN];
static bool   s_sine_ready = false;

static void nco_init_table(void)
{
  if (s_sine_ready) return;
  for (unsigned i = 0; i < SINE_LEN; ++i) {
    double a = (2.0 * 3.14159265358979 * i) / SINE_LEN;
    double v = 127.0 * sin(a);
    s_sine[i] = (int8_t)(v >= 0 ? v + 0.5 : v - 0.5);
  }
  s_sine_ready = true;
}

/* phase increment:  inc = f_tone * 2^PHASE_BITS / f_sample  */
static uint32_t nco_word(double f_tone, double f_sample)
{
  double w = f_tone * 4294967296.0 / f_sample;   /* 2^32 */
  return (uint32_t)(w + 0.5);
}

/* ---------------- 'z' : offset speed benchmark ---------------- */
#define BENCH_N 20000u

static void offset_speed_benchmark(void)
{
  bool was_tx = s_tx_active;
  if (!s_tx_active) {
    RAIL_StartTxStream(s_rail, s_channel, RAIL_STREAM_CARRIER_WAVE);
    s_tx_active = true; s_tx_mode = RAIL_STREAM_CARRIER_WAVE;
  }

  /* small, alternating offsets, so the call is not a "no-op" */
  static const int16_t pat[4] = { +100, -100, +50, -50 };

  RAIL_Time_t t_start = RAIL_GetTime();
  uint32_t min_us = 0xFFFFFFFFu, max_us = 0;
  RAIL_Time_t prev = t_start;

  for (uint32_t i = 0; i < BENCH_N; ++i) {
    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)pat[i & 3]);
    RAIL_Time_t now = RAIL_GetTime();
    uint32_t dt = (uint32_t)(now - prev);
    if (dt < min_us) min_us = dt;
    if (dt > max_us) max_us = dt;
    prev = now;
  }
  RAIL_Time_t t_end = RAIL_GetTime();
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);

  uint32_t total = (uint32_t)(t_end - t_start);
  uint32_t avg_ns = (total * 1000u) / BENCH_N;   /* average ns/call */
  uint32_t rate_hz = (avg_ns > 0) ? (1000000000u / avg_ns) : 0;

  printf("# --- SetFreqOffset benchmark (%lu calls) ---\r\n",
         (unsigned long)BENCH_N);
  printf("# total=%lu us, avg=%lu ns/call, ~%lu Hz (%lu.%02lu kHz) "
         "max sample rate\r\n",
         (unsigned long)total, (unsigned long)avg_ns,
         (unsigned long)rate_hz,
         (unsigned long)(rate_hz / 1000),
         (unsigned long)((rate_hz % 1000) / 10));
  printf("# call interval min=%lu us max=%lu us (jitter=%lu us)\r\n",
         (unsigned long)min_us, (unsigned long)max_us,
         (unsigned long)(max_us - min_us));
  printf("# AFSK requirement: 9600 Hz -> 104 us/sample. %s\r\n",
         (avg_ns < 104000u) ? "FITS (NCO-AFSK is feasible)"
                            : "does NOT fit — a lower sample rate is needed");
  static const uint32_t fs_list[5] = { 9600u, 19200u, 38400u,
                                       76800u, 153600u };
  printf("# load of the sweep stages (%% of the max sample rate):\r\n");
  for (int k = 0; k < 5; ++k) {
    uint32_t need_ns = 1000000000u / fs_list[k];
    uint32_t load = (rate_hz > 0) ? (fs_list[k] * 100u / rate_hz) : 999;
    printf("#   %6lu Hz: %lu ns/sample needed, load ~%lu%% -> %s\r\n",
           (unsigned long)fs_list[k], (unsigned long)need_ns,
           (unsigned long)load,
           (avg_ns < need_ns) ? "OK" : "TOO FAST (cannot keep up)");
  }

  if (!was_tx) tx_stream_stop();
}

/* ---------------- 'y' : NCO test-tone sweep ---------------- */
#define TONE_SECS     5u
#define TONE_GAP_MS   400u
#define TONE_DEV_TICK 645.0     /* +/- peak offset for sine +/-1 */

static const double s_tone_fs[5] = { 9600.0, 19200.0, 38400.0,
                                     76800.0, 153600.0 };

/* TIMING: no 64-bit division per sample — Bresenham: whole-us step +
 * ns-remainder accumulator. So there is no systematic frequency drift
 * (the old truncating version gave a ~6.5% lower and distorted tone at
 * 153.6 kHz), and the loop load is smaller too. */
static void nco_play(double f_tone, double f_sample, uint32_t secs)
{
  uint32_t word = nco_word(f_tone, f_sample);
  uint32_t acc = 0;
  uint32_t nsamp = (uint32_t)(f_sample * secs);
  uint32_t period_ns = (uint32_t)(1e9 / f_sample + 0.5);
  uint32_t whole_us  = period_ns / 1000u;    /* whole us / sample */
  uint32_t frac_ns   = period_ns % 1000u;    /* remainder ns / sample */
  uint32_t ns_accum  = 0;

  RAIL_Time_t target = RAIL_GetTime();

  for (uint32_t i = 0; i < nsamp; ++i) {
    int8_t s = s_sine[acc >> (PHASE_BITS - SINE_BITS)];
    RAIL_FrequencyOffset_t off = (RAIL_FrequencyOffset_t)
        (s_freq_tick + (s * (int)TONE_DEV_TICK) / 127);
    RAIL_SetFreqOffset(s_rail, off);
    acc += word;
    target += whole_us;
    ns_accum += frac_ns;
    if (ns_accum >= 1000u) { ns_accum -= 1000u; ++target; }
    while ((int32_t)(RAIL_GetTime() - target) < 0) { }
  }
}

static void nco_tone_test(double f_tone)
{
  nco_init_table();
  bool was_tx = s_tx_active;
  if (!s_tx_active) {
    RAIL_StartTxStream(s_rail, s_channel, RAIL_STREAM_CARRIER_WAVE);
    s_tx_active = true; s_tx_mode = RAIL_STREAM_CARRIER_WAVE;
  }
  armed = false;

  printf("# NCO test-tone sweep: %d Hz, 5 stages x %u s, "
         "dev +/-%d ticks\r\n",
         (int)f_tone, (unsigned)TONE_SECS, (int)TONE_DEV_TICK);

  for (int k = 0; k < 5; ++k) {
    double fs = s_tone_fs[k];
    uint32_t spp = (uint32_t)(fs / f_tone + 0.5);   /* samples/period */
    printf("#  [%d/5] fs=%6d Hz  (%lu samples/period)  %u s...\r\n",
           k + 1, (int)fs, (unsigned long)spp, (unsigned)TONE_SECS);
    nco_play(f_tone, fs, TONE_SECS);

    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
    RAIL_Time_t t = RAIL_GetTime() + TONE_GAP_MS * 1000u;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
  }

  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
  printf("# sweep done — 5 stages recorded\r\n");
  if (!was_tx) tx_stream_stop();
}

static void aprs_send_beacon(void)
{
  if (capturing) { printf("# capture running — 's' first / wait\r\n"); return; }

  /* STATIC, not stack! aprs_frame_t is ~1 KB — as a local it overflows
   * the SoC Empty stack (hard fault on 'b', while w/p work). */
  static aprs_frame_t fr;
  if (!aprs_build_ui(&fr, APRS_MYCALL, APRS_SSID,
                     APRS_DEST, 0, APRS_VIA, APRS_VIA_SSID, APRS_INFO)) {
    printf("# frame did not fit\r\n");
    return;
  }
  printf("# APRS beacon (NCO-AFSK): %s -> %s, %u bits (~%lu ms), "
         "fs=%d Hz, %u samples/bit\r\n",
         APRS_MYCALL, APRS_DEST, (unsigned)fr.nbits,
         (unsigned long)(fr.nbits * 1000u / 1200u),
         (int)AFSK_FS, (unsigned)AFSK_SAMPLES_PER_BIT);

  nco_init_table();

  /* carrier on, RSSI trigger off */
  armed = false;
  if (!s_tx_active) {
    RAIL_StartTxStream(s_rail, s_channel, RAIL_STREAM_CARRIER_WAVE);
    s_tx_active = true;
    s_tx_mode = RAIL_STREAM_CARRIER_WAVE;
  }

  /* NCO-AFSK: the NRZI level selects the tone (1 -> mark 1200 Hz,
   * 0 -> space 2200 Hz), the phase accumulator is NOT reset at bit
   * transitions -> phase-continuous Bell 202. Bit time = 32 samples,
   * without a separate clock. */
  uint32_t word_mark  = nco_word(AFSK_MARK_HZ,  AFSK_FS);
  uint32_t word_space = nco_word(AFSK_SPACE_HZ, AFSK_FS);
  uint32_t acc = 0;

  uint32_t period_ns = (uint32_t)(1e9 / AFSK_FS + 0.5);   /* 26042 ns */
  uint32_t whole_us  = period_ns / 1000u;
  uint32_t frac_ns   = period_ns % 1000u;
  uint32_t ns_accum  = 0;
  RAIL_Time_t target = RAIL_GetTime();

  for (uint16_t i = 0; i < fr.nbits; ++i) {
    uint32_t word = fr.bits[i] ? word_mark : word_space;
    for (uint32_t s = 0; s < AFSK_SAMPLES_PER_BIT; ++s) {
      int8_t sv = s_sine[acc >> (PHASE_BITS - SINE_BITS)];
      RAIL_FrequencyOffset_t off = (RAIL_FrequencyOffset_t)
          (s_freq_tick + (sv * AFSK_DEV_TICK) / 127);
      RAIL_SetFreqOffset(s_rail, off);
      acc += word;
      target += whole_us;
      ns_accum += frac_ns;
      if (ns_accum >= 1000u) { ns_accum -= 1000u; ++target; }
      while ((int32_t)(RAIL_GetTime() - target) < 0) { }
    }
  }

  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
  tx_stream_stop();
  printf("# beacon done\r\n");
}

/* Safe stream stop: afterwards the caller may freely touch the radio.
 * Returns whether the stream was running (i.e. whether to restart later). */
static bool stream_suspend(void)
{
  if (!iq_stream_active()) return false;
  iq_stream_stop(s_rail, s_channel);
  RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
  set_freq_tick(s_freq_tick);
  return true;
}

/* Process one command line (terminated by newline/Enter).
 *
 * IT HAS TWO CALLERS: the terminal (app_process_action) and — if wired —
 * the cmdlink, i.e. the phone. The terminal path has already stopped the
 * stream (any character stops it), but the cmdlink has NOT: there the
 * command arrives while the stream is running.
 *
 * So order must be kept here. Commands that touch the radio first stop a
 * running stream, and it resumes at the end. Without this, e.g. a 'c'
 * sent from the phone would leave the capturing flag stuck forever (the
 * event callback exits on the stream branch, so the burst would never
 * complete), and the unit would be unusable until reset. */
static void handle_line(const char *line)
{
  char c = line[0];

  /* POSITIVE list: THESE commands touch the radio/FIFO, so the stream must
   * stop before them. The reverse (a blacklist) would be wrong: the
   * cmdlink is a single-wire UART without parity, and after a corrupted
   * byte even a non-existent command would force the stream through a
   * stop-tune-start cycle — a visible gap in the waterfall, for nothing.
   * ('F' is not on the list: it handles the stop and restart itself.) */
  static const char NEEDS_STOP[] = "cwpxbzydfkngMW";
  bool resume_after = false;
  if (strchr(NEEDS_STOP, c) != NULL && c != '\0' && iq_stream_active()) {
    printf("# stopping the stream for the '%c' command...\r\n", c);
    resume_after = stream_suspend();
  }

  if (c == 'c') {
    if (s_tx_active) {
      printf("# TX running — 'x' (stop) first, then capture\r\n");
    } else if (!capturing) {
      capture_idx = 0; capture_bad = false; capture_done = false;
      capturing = true;
    }
  } else if (c == 'a') {
    if (s_tx_active) {
      printf("# TX running — 'x' (stop) first, then arming\r\n");
      return;
    }
    armed = !armed;
    printf("# armed=%d (threshold %d dBm) — starts automatically on signal\r\n",
           (int)armed, (int)(arm_thresh_qdbm / 4));
  } else if (c == 'w') {              /* T1: unmodulated carrier */
    tx_stream_start(RAIL_STREAM_CARRIER_WAVE);
  } else if (c == 'p') {              /* T6: PN9 modulated spectrum */
    tx_stream_start(RAIL_STREAM_PN9_STREAM);
  } else if (c == 'x') {
    tx_stream_stop();
  } else if (c == 'b') {              /* T5: APRS direct-FSK beacon */
    aprs_send_beacon();
  } else if (c == 'z') {              /* T5b: offset speed benchmark */
    offset_speed_benchmark();
  } else if (c == 'y') {              /* T5b: NCO test tone */
    double f = (line[1]) ? (double)atoi(line + 1) : 1200.0;
    if (f < 100.0 || f > 4000.0) f = 1200.0;
    nco_tone_test(f);
  } else if (c == 'M') {              /* CW Morse transmitter */
    /* M1 / M2 / M3 / M <text> / M/WPM <text>
     * NEEDS_STOP has already stopped the stream. After TX s_tx_active=false,
     * so resume_after restarts the stream. */
    uint8_t wpm = CW_DEFAULT_WPM;
    char sub = line[1];
    if (sub == '1' || sub == '2' || sub == '3') {
      const char *p = line + 2;
      if (*p == '/') wpm = (uint8_t)atoi(p + 1);
      s_tx_active = true;
      if (sub == '1')      cw_morse_cq(s_rail, s_channel, wpm);
      else if (sub == '2') cw_morse_test(s_rail, s_channel, wpm);
      else                 cw_morse_beacon(s_rail, s_channel, wpm);
      s_tx_active = false;
    } else if (sub == ' ' || sub == '/') {
      const char *p = line + 1;
      if (*p == '/') {
        wpm = (uint8_t)atoi(p + 1);
        while (*p && *p != ' ') ++p;
      }
      while (*p == ' ') ++p;
      if (*p) {
        s_tx_active = true;
        cw_morse_send(s_rail, s_channel, p, wpm);
        s_tx_active = false;
      } else {
        printf("# CW: M1=CQ  M2=VVV  M3=beacon  M <text>  M/WPM <text>\r\n");
      }
    } else {
      printf("# CW: M1=CQ  M2=VVV  M3=beacon  M <text>  M/WPM <text>\r\n");
    }
  } else if (c == 'd') {              /* TX power in dBm */
    int dbm = atoi(line + 1);
    RAIL_Status_t st = RAIL_SetTxPowerDbm(s_rail,
                                          (RAIL_TxPower_t)(dbm * 10));
    printf("# TX power = %d dBm requested (st=%d, actual %d ddBm)\r\n",
           dbm, (int)st, (int)RAIL_GetTxPowerDbm(s_rail));
    if (s_tx_active) {                /* takes effect on a live stream via restart */
      tx_stream_start(s_tx_mode);
    }
  } else if (c == 'F') {              /* ABSOLUTE tuning in kHz */
    uint32_t khz = (uint32_t)atoi(line + 1);
    if (khz < 100000u || khz > 1000000u) {
      printf("# usage: F<kHz>, e.g. F433775 (LoRa-APRS) or F434000\r\n");
    } else {
      /* Tuning also means RAIL_ResetFifo. The stream reads ZERO-COPY, so
       * it has its own pointer into the FIFO — it would not survive a
       * reset in sync. Therefore on-the-fly tuning STOPS it and RESTARTS
       * it with the same R. A pause of a few milliseconds. */
      uint32_t R = s_stream_R;
      bool was_stream = stream_suspend();
      tune_khz(khz);
      if (s_tx_active) {
        tx_stream_start(s_tx_mode);   /* the carrier must follow the tuning */
      } else if (!capturing) {
        /* After tuning there must ALWAYS be an I/Q stream. Previously it
         * only resumed if it had been running (was_stream) — but the ESP32,
         * when entering the SDR++ IQ mode, only sends F<kHz>, never an
         * explicit "i", so after a scan (W0) the stream stayed stopped and
         * IQ "did not start". (2026-08-15) */
        (void)was_stream;
        stream_start_R(R, (uint8_t)IQ_AUTOSTART_SHIFT, "after tuning");
      }
    }
  } else if (c == 'f') {
    /* Raw channel change. The same FIFO hazard as with 'F': restart_rx()
     * calls RAIL_ResetFifo, which the zero-copy stream's own read pointer
     * would not survive. The stream is already stopped at the start of
     * handle_line ('f' is not on the safe list), but the channel must be
     * bounded here too — otherwise RAIL_StartRx silently returns an error
     * and the radio simply does not receive. */
    long ch = atoi(line + 1);
    if (ch < 0) ch = 0;
    if (ch > (long)TUNE_MAX_CHANNEL) {
      printf("# channel %ld > TUNE_MAX_CHANNEL (%u) — clipped\r\n",
             ch, (unsigned)TUNE_MAX_CHANNEL);
      ch = (long)TUNE_MAX_CHANNEL;
    }
    s_channel = (uint16_t)ch;
    if (s_tx_active) {
      tx_stream_start(s_tx_mode);     /* scripted stepping */
    } else {
      restart_rx();
    }
    printf("# channel = %u  (~%lu kHz)\r\n", (unsigned)s_channel,
           (unsigned long)(TUNE_BASE_KHZ + s_channel * TUNE_SPACING_KHZ));
  } else if (c == 'o') {
    /* Clamped: outside +-0x3FFF RAIL_SetFreqOffset SILENTLY returns an
     * error and the receiver stays at nominal. Two separate hidden faults
     * would result: a detuned receiver, and int32 overflow of the printed
     * Hz value (at |tick| > ~46200). */
    set_freq_tick((int32_t)atoi(line + 1));
    printf("# fine offset = %ld ticks (%ld Hz)\r\n",
           (long)s_freq_tick, (long)(s_freq_tick * 46492 / 10000));
  } else if (c == 'k') {              /* T-ceiling: I/Q rate benchmark */
    if (s_tx_active) {
      printf("# TX running — 'x' (stop) first, then benchmark\r\n");
    } else if (capturing) {
      printf("# capture running — wait for the end of the burst\r\n");
    } else {
      /* 'k' = 10 s/point (quick iteration), 'k60' = 60 s/point (report) */
      uint32_t secs = (line[1]) ? (uint32_t)atoi(line + 1) : 10u;
      if (secs < 2u || secs > 120u) secs = 10u;
      armed = false;                  /* no RSSI trigger during measurement */
      iq_bench_sweep(s_rail, s_channel, secs, THRESHOLD_BYTES);
      RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);   /* restore calibration */
    }
  } else if (c == 'i') {              /* continuous decimated stream */
    if (iq_stream_active()) {
      iq_stream_stop(s_rail, s_channel);
      RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
      RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
    } else if (s_tx_active || capturing) {
      printf("# TX or capture running — stop it first\r\n");
    } else {
      /* 'i<R>' — R is the TOTAL decimation, any integer (not log2!).
       * Without an argument it starts with the AUTOSTART value, so that
       * 'i' still gives what you saw at power-up. */
      uint32_t R = (line[1]) ? (uint32_t)atoi(line + 1)
                             : (uint32_t)IQ_AUTOSTART_DECIM;
      stream_start_R(R, (uint8_t)IQ_AUTOSTART_SHIFT, "command");
    }
  } else if (c == 'g') {              /* I2S pin test (GPIO toggling) */
    if (s_tx_active || capturing || iq_stream_active()) {
      printf("# stop the running operation first\r\n");
    } else {
      uint32_t secs = (line[1]) ? (uint32_t)atoi(line + 1) : 10u;
      if (secs < 1u || secs > 60u) secs = 10u;
      iq_stream_pin_test(secs);
    }
  } else if (c == 'v') {              /* EUSART clock diagnostics */
    iq_stream_dump_uart_cfg();
  } else if (c == 'n') {              /* zero-copy (NULL) read test */
    if (s_tx_active) {
      printf("# TX running — 'x' (stop) first\r\n");
    } else if (capturing) {
      printf("# capture running — wait for the end of the burst\r\n");
    } else {
      uint32_t secs = (line[1]) ? (uint32_t)atoi(line + 1) : 10u;
      if (secs < 2u || secs > 120u) secs = 10u;
      armed = false;
      iq_bench_zerocopy_test(s_rail, s_channel, secs, THRESHOLD_BYTES);
      RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
    }
  } else if (c == 'W') {
    /* Wideband scan (RSSI panadapter). W<center_kHz>,<span_kHz>,<nbin>
     * [,<floor_dBm>,<range_dB>]  |  W0 = stop. Identical from the terminal
     * AND the cmdlink (ESP32 -> SpyServer FFT mode). The scan and the
     * stream are mutually exclusive: due to NEEDS_STOP the stream is
     * already stopped; it does NOT resume during the scan (see below),
     * but it does after W0. */
    if (s_tx_active) {
      printf("# TX running — 'x' (stop) first, then scan\r\n");
    } else {
      unsigned long ck = 0, sk = 0, nb = 0; long fl = -130, rg = 100;
      /* Strict check: only digits, commas and minus may be in the line. A
       * corrupted cmdlink line (FIFO overflow) could otherwise yield
       * "plausible" parameters — e.g. 100 or 1016 bins instead of 640. */
      bool clean = true;
      for (const char *q = line + 1; *q; q++) {
        if (!((*q >= '0' && *q <= '9') || *q == ',' || *q == '-')) { clean = false; break; }
      }
      int got = clean ? sscanf(line + 1, "%lu,%lu,%lu,%ld,%ld", &ck, &sk, &nb, &fl, &rg) : 0;
      if (!clean) printf("# scan: corrupted command line dropped: '%s'\r\n", line);
      if (got >= 1 && ck == 0) {
        if (scan_active()) {
          scan_stop();
          RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                            RAIL_EVENT_RX_FIFO_ALMOST_FULL
                            | RAIL_EVENT_RX_FIFO_OVERFLOW);
          restart_rx();
          set_freq_tick(s_freq_tick);
          /* After W0 the stream ALWAYS resumes (the unit's default state is
           * the autostarted I/Q stream). 2026-08-15: after reflashing the
           * ESP32 + terminal keystrokes the stream stayed stopped, and the
           * SDR++ IQ mode "did not work" — only the FG23 stream was not
           * running. */
          resume_after = true;
          s_scan_resume = false;
        } else {
          printf("# scan not running\r\n");
          /* W0 must still mean: "there shall be an I/Q stream" — if the
           * stream is stopped (terminal key, ESP32 flash), restart it. */
          if (!iq_stream_active() && !s_tx_active && !capturing) resume_after = true;
        }
      } else if (got >= 3) {
        if (got >= 5) scan_set_scale((int16_t)fl, (uint16_t)rg);
        if (!scan_active()) s_scan_resume = resume_after;
        resume_after = false;          /* the stream does NOT resume during the scan */
        if (!scan_start((uint32_t)ck, (uint32_t)sk, (uint16_t)nb)) {
          printf("# scan: bad parameters (W<kHz>,<span_kHz>,<nbin>)\r\n");
          resume_after = s_scan_resume; s_scan_resume = false;
        }
      } else {
        printf("# usage: W<center_kHz>,<span_kHz>,<nbin>[,floor,range] | W0\r\n");
      }
    }
  } else if (c == 'T') {
    /* Scan timing at run time: T<avg_us>,<wait_us>  (e.g. T300,3000).
     * 's' and the line diagnostics show how many bins remain invalid. */
    unsigned long st = 0, wt = 0, md = 0;
    int got_t = sscanf(line + 1, "%lu,%lu,%lu", &st, &wt, &md);
    if (got_t >= 2) {
      scan_set_timing((uint32_t)st, (uint32_t)wt);
      if (got_t >= 3) scan_set_method((uint8_t)md);
    } else {
      printf("# usage: T<avg_us>,<wait_us>[,<method 0|1|2>]\r\n");
    }
  } else if (c == 't') {
    arm_thresh_qdbm = (int16_t)(4 * atoi(line + 1));
    printf("# trigger threshold = %d dBm\r\n", (int)(arm_thresh_qdbm / 4));
  } else if (c == 's') {
    /* The actual tuning computed back, so it is visible WHERE the receiver
     * sits — no need to add up channel and offset in your head. */
    long hz = (long)TUNE_BASE_KHZ * 1000
              + (long)s_channel * (long)TUNE_SPACING_KHZ * 1000
              + (long)((s_freq_tick - corr_tick_for_khz(current_khz()))
                       * 46492 / 10000);
    printf("# events=%lu short=%lu ovf=%lu idx=%lu armed=%d\r\n",
           (unsigned long)stat_events, (unsigned long)stat_short_reads,
           (unsigned long)stat_overflows, (unsigned long)capture_idx,
           (int)armed);
    printf("# tuning: ~%ld.%03ld kHz (ch=%u, offset=%ld ticks) "
           "rssi=%d dBm\r\n",
           hz / 1000, hz % 1000, (unsigned)s_channel, (long)s_freq_tick,
           (int)(RAIL_GetRssi(s_rail, false) / 4));
    printf("# stream=%d (R=%lu, %lu sps)  tx=%d(%s)\r\n",
           (int)iq_stream_active(), (unsigned long)s_stream_R,
           (unsigned long)iq_stream_out_sps((uint16_t)s_stream_R),
           (int)s_tx_active,
           (s_tx_mode == RAIL_STREAM_CARRIER_WAVE) ? "CW" : "PN9");
    scan_print_stats();
#if CMDLINK_ENABLE
    uint32_t cl_lines = 0, cl_err = 0;
    cmdlink_stats(&cl_lines, &cl_err);
    printf("# cmdlink: %lu lines, %lu errors\r\n",
           (unsigned long)cl_lines, (unsigned long)cl_err);
#endif
  }

  /* If the stream was stopped for the command, it resumes now — unless
   * the command itself put the radio into TX or capture. */
  if (resume_after) {
    if (!s_tx_active && !capturing) {
      stream_start_R(s_stream_R, (uint8_t)IQ_AUTOSTART_SHIFT, "after command");
    } else {
      printf("# the stream does NOT resume (TX or capture running) — "
             "'x' then 'i'\r\n");
    }
  }
}

void app_process_action(void)
{
  static char linebuf[64];   /* enough for CW free text too (M ...) */
  static uint8_t len = 0;

  char ch;
  bool have_ch = false;

  /* --- continuous stream: the main loop's job is the pump. Any incoming
   * character stops it. --- */
#if CMDLINK_ENABLE
  /* The second command input is alive during the stream too: tuning from
   * the phone does not stop the stream (the tune itself takes care of the
   * safe restart). */
  cmdlink_poll();
#endif

  /* Wideband scan: tune + RSSI per bin, sends the finished line over SPI.
   * Mutually exclusive with the stream, so we only get here if it is not
   * running. */
  if (scan_active()) {
    scan_process();
    /* From the terminal any NON-line-end character stops the scan as well
     * (like the stream), and the character goes into the command buffer.
     * CR/LF, however, PASSES ON to the line collector — 2026-08-15: it
     * used to be dropped (return), so during a scan the line end of a
     * typed command (e.g. T100,1000,2) was lost and the command NEVER ran. */
    if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
      if (ch != '\r' && ch != '\n') {
        handle_line("W0");
        printf("# scan stopped. 'W<kHz>,<span>,<nbin>' = again\r\n");
      }
      have_ch = true;
    } else {
      return;
    }
  }

  /* With have_ch the stream is NOT touched: the scan branch's W0 has just
   * restarted it, and the getchar here would swallow the buffered
   * character. */
  if (iq_stream_active() && !have_ch) {
    iq_stream_pump();
    /* Line-end characters do NOT stop the stream! The terminal sends
     * CR+LF: the '\r' starts the command, and the remaining '\n' would
     * immediately stop it. (Exactly this happened: the stream died after
     * ~168 us with zero events.) Any OTHER character stops it. */
    if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
      if (ch != '\r' && ch != '\n') {
        iq_stream_stop(s_rail, s_channel);
        RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
        RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
        printf("# stream stopped. 'i' = again, 'F<kHz>' = tune, "
               "'s' = status\r\n");
      }
      /* The stopping character is NOT LOST: it lands in the command
       * buffer. So even with autostart it is enough to simply type
       * "F433775" — the first 'F' stops the stream AND becomes the first
       * letter of the command. CR/LF PASSES ON to the line collector, so
       * a half-typed line can be terminated (2026-08-15). */
      have_ch = true;
    } else {
      return;
    }
  }

  /* Collect characters up to the line end; commands can thus take an
   * argument (e.g. 'F433775', 'o-40'). Non-blocking getchar. */
  if (have_ch
      || sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
    if (ch == '\r' || ch == '\n') {
      if (len > 0) {
        linebuf[len] = '\0';
        handle_line(linebuf);
        len = 0;
      }
    } else if (len < sizeof(linebuf) - 1) {
      linebuf[len++] = ch;
    } else {
      len = 0;   /* overflow -> drop */
    }
  }

  /* RSSI-armed start: when the band comes alive, capture immediately —
   * this catches the BEGINNING of an over-the-air APRS packet.
   * Meaningless during a TX stream (no RX), so it is skipped. */
  if (armed && !capturing && !capture_done && !s_tx_active) {
    int16_t rssi_qdbm = RAIL_GetRssi(s_rail, false);
    if (rssi_qdbm != RAIL_RSSI_INVALID && rssi_qdbm > arm_thresh_qdbm) {
      capture_idx = 0; capture_bad = false;
      capturing = true;
      armed = false;
    }
  }

  if (capture_done) {
    capture_done = false;
    if (capture_bad) {
      printf("# OVERFLOW during the burst — discarded, try again\r\n");
    } else {
      dump_capture(0 /* fs_hint: fill in Hz after self-calibration */);
    }
  }

  /* PB0 button: one max-power beacon on the press EDGE (active-low).
   * Simple software debounce: fires only if the previous state was high
   * and it is now low. */
  static bool pb0_prev_high = true;
  bool pb0_now_high = (GPIO_PinInGet(PB0_PORT, PB0_PIN) != 0);
  if (pb0_prev_high && !pb0_now_high) {
    /* short wait for debouncing (~5 ms) */
    RAIL_Time_t t = RAIL_GetTime() + 5000u;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
    if (GPIO_PinInGet(PB0_PORT, PB0_PIN) == 0 && !capturing) {
      beacon_max_power();
    }
  }
  pb0_prev_high = pb0_now_high;
}

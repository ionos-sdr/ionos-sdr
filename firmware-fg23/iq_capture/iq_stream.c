/* SPDX-License-Identifier: MIT
 *
 * iq_stream.c — continuous, decimated I/Q stream over SPI to the ESP32-S3
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT license
 *
 * ================= WHY SPI AND NOT I2S =================
 *
 * 2026-07-31, multimeter measurement: the CS of the FG23 USART can NOT
 * produce a true I2S word select. Its duty cycle is 33.6% (1.11 V), where
 * 50% is required. Control: the 'g' pin test, which toggles all three pins
 * as GPIO at EXACTLY 50%, gave 1.65 V on all three — so the amplitude is
 * full and the measurement is valid. All eight framing combinations
 * (W16D16/W32D16 x Left/Right x delay true/false) gave 1.05-1.11 V.
 *
 * Reason: the FG23 has no dedicated I2S peripheral. It has a generic
 * synchronous USART in which the chip select is reinterpreted as word
 * select. This was designed to feed an audio DAC — where the receiver is
 * a dumb shift register. An ESP32 I2S slave, however, is a state machine
 * that needs a textbook 50% WS, otherwise it keeps hunting for the frame.
 *
 * ================= WHAT SPI SOLVES =================
 *
 * I2S imposed a constraint: the bit clock had to be fs*32, CONTINUOUSLY.
 * That is why every tiny gap was fatal, and why the USART could never be
 * allowed to run empty.
 *
 * With SPI we send in BURSTS: pull CS low, shift out a whole block at a
 * high clock, release it, and the line rests until the next one. The link
 * speed is DECOUPLED from the sample rate, and a gap is no longer an
 * error but part of normal operation.
 *
 * In addition the block header carries a SEQUENCE NUMBER, so packet loss
 * becomes a printable number, not a guess. That is a qualitatively better
 * failure than a silent, continuously corrupting bit slip.
 *
 * ===================== WIRING (wires unchanged!) =====================
 *   WSTK EXP   FG23 pin   SPI signal             speed       ESP32-S3
 *   ---------------------------------------------------------------------
 *   EXP 15     PC05       SCLK                   several MHz -> GPIO5
 *   EXP 10     PC00       MOSI (FG23 -> ESP)     several MHz -> GPIO6
 *   EXP 13     PA07       CS   (block frame)     ~25 Hz      -> GPIO7
 *   EXP  1     GND                                           -> GND
 *
 * CHOICE OF THE THREE from the radio board EXP table (UG506, xG23):
 *   EXP 10 = PC0  — SPI_CS position, shared with NOTHING. Clean.
 *   EXP 15 = PC5  — I2C_SCL position (sensor), clean in practice.
 *   EXP 13 = PA7  — plain GPIO, shared with nothing. Clean.
 *
 * NOTE: CS is now on PORT A, not on C! The drive/slew setting applies per
 * port, so port A is configured as well (see below). CS toggles once per
 * block (~25 Hz), so it is not critical there.
 *
 * ===================== BLACKLIST =====================
 * 2026-08-01: the radio board EXP table (UG506, xG23 radio board) showed
 * that our assumption about header pins 4/6/8 was OFF BY ONE POSITION.
 * The real assignment:
 *
 *   EXP  4 = PC1  — FLASH_MOSI + DISP_SI. Driven by the board. NO.
 *   EXP  6 = PC2  — FLASH_MISO. Loaded net. NO.
 *   EXP  8 = PC3  — FLASH_SCLK + DISP_SCLK. NO.
 *   EXP 12 = PA8  — VCOM_TX. This is the console! NO.
 *   EXP 14 = PA9  — VCOM_RX. This is the console! NO.
 *   EXP 17 = BOARD_ID_SCL, EXP 19 = BOARD_ID_SDA — board controller. NO.
 *   EXP  2 = VMCU, EXP 18 = 5V, EXP 20 = 3V3, EXP 1 = GND — power.
 *
 * THIS CAUSED THE WHOLE OVERNIGHT CHASE: when the firmware drove PC03, it
 * came out on EXP 8 (display clock), while the wire sat in EXP 6, which
 * nobody drove as PC2 — so it made no difference whether it was plugged
 * in. The "PC02 does not carry the fast signal" observation comes from
 * the same source: it was actually the flash MISO net.
 *
 * Free and clean spares if another line is needed (e.g. RDY):
 *   EXP 11 = PA6, EXP 9 = PD2, EXP 7 = PA5 — all plain GPIO.
 *
 * ESP side: GPIO3 strapping pin + probable VBAT divider on the Heltec — NO.
 */

#include "iq_stream.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"
#include "em_eusart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_ldma.h"
#include "dmadrv.h"
#include <stdio.h>
#include <string.h>

/* ================= CONFIGURATION ================= */

/* SPI bit rate. For 12.5 ksps, 4 MHz is a tenfold margin: a 1040-byte
 * block goes out in 2.1 ms while one is produced every 20.5 ms. On Dupont
 * wires this is still comfortable.
 *
 * The 700 ksps PCB target will need ~30 MHz — but that will no longer be
 * a Heltec dev board, and IO_MUX pins can be designed in. */
#define SPI_HZ            4000000u

/* Complex samples per block. 256 samples = 20.5 ms @ 12.5 ksps.
 * Block size = 16-byte header + 256*4 = 1040 bytes. */
#define BLK_SAMPLES       256u

/* Number of blocks held. The CIC fills one, the DMA sends another, the
 * rest is reserve. 6 * 1040 = 6240 bytes of the 64 kB — cheap insurance.
 *
 * WHAT IT PROTECTS AGAINST AND WHAT NOT: this covers the FG23's own
 * congestion. If it fills up, s_blk_drops grows and blocks/s drops on the
 * other side — BUT NO SEQUENCE GAP appears, because s_seq is written into
 * the header AT SEND TIME. Thus:
 *   blocks/s drops, LOST is zero -> the problem is HERE (see 'dropped blocks')
 *   LOST grows                   -> the ESP32 did not take it, look there */
#define NBLK              6u

/* THE CHIP'S I/Q SAMPLE RATE — from the 'k' benchmark measurement!
 *
 * THIS IS THE MOST OFTEN FORGOTTEN LINE. If you change the PHY, CHANGE
 * THIS TOO. It is immediately visible in the line printed at start-up.
 *
 * Measured values: 434M factory profile -> 400 ksps
 *                  100 kHz acq. BW      -> 400 ksps
 *                   50 kHz acq. BW      ->  70 ksps
 *                  270 kHz acq. BW      ->   1 Msps                 */
#define IQ_STREAM_FS_IN_HZ 400000u

/* ---------------- DC BLOCK ----------------
 * The receiver's DC offset sits in the MIDDLE of the decimated band, and
 * the CIC passes it at unity gain — without this the chain sticks to the
 * rail. Single-pole leaky integrator at the output rate, time constant
 * ~2^DC_SHIFT samples.
 *
 * WHY 14 AND NOT 10 (2026-08-03):
 * The value 10 was sized for 12.5 ksps (~82 ms). At i8 the output is
 * 50 ksps, where the same 10 is only 20 ms — a 7.8 Hz corner.
 *
 * BUT NOTE: THIS IS ONLY MITIGATION, NOT A CURE. The real problem is that
 * the carrier of an FM signal is a LINE: zero bandwidth. A DC leak, however
 * slow, has INFINITE attenuation at 0 Hz — if the carrier lands exactly
 * there, it is removed entirely. Slowing it down only means it takes
 * longer to do so.
 *
 * Measurement (sim_fg23.py, 1 kHz audio, 3 kHz deviation, i8, 2.5 s):
 *
 *   DC_SHIFT   tuning error at 0 Hz:      THD
 *      10                                16.2 %
 *      14                                 8.3 %
 *       0 (no DC block)                   0.04 %
 *
 *   DC_SHIFT 14, with various tuning errors:
 *        0 Hz   8.3 %      2000 Hz  24.8 %
 *      500 Hz   0.08 %     2500 Hz   0.09 %
 *     1000 Hz  13.2 %      5500 Hz   0.32 %
 *
 * The pattern is clear: whenever an FM sideband lands EXACTLY on 0 Hz
 * (with a pure 1 kHz tone, every whole-kHz offset does this), the leak
 * punches a hole in the spectrum and the demodulated audio falls apart.
 * With real speech/AFSK the energy is spread out, so it is milder — but
 * the CARRIER line is always there.
 *
 * A CONSEQUENCE THAT SEEMS ODD AT FIRST: the more precisely the crystal is
 * calibrated, the closer the carrier sits to 0 Hz, and the WORSE the NFM
 * audio gets. This is the kind of fault you would never find by trial and
 * error.
 *
 * THE REAL SOLUTION, for later: a deliberate LOW IF. Tune the FG23 a few
 * kHz off, so the signal sits at the edge of the output band, not the
 * center — then neither the DC leak, nor the DC spike, nor the 1/f noise
 * touches it. In exchange aprs_rx and the SpyServer center frequency must
 * know about it, so this is not a one-line change. Until then 14 is the
 * better deal: it halves the damage and costs nothing.
 *
 * The accumulator is therefore int64: it stores the average scaled by
 * 2^DC_SHIFT, and 2^20 * 2^14 = 2^34 no longer fits in 32 bits. */
#define IQ_STREAM_DC_BLOCK 1
#define DC_SHIFT           14
#define DC_PRECLAMP        (1 << 20)

/* ---------------- RDY LINE (eliminating loss by design) ------
 *
 * The ESP32 signals on GPIO2 whether it REALLY has a queued SPI
 * transaction (from the driver's post_setup/post_trans callbacks, i.e. a
 * hardware fact). We check it BEFORE pulling CS low, and wait if not
 * ready. Thus a block can neither be lost nor slip a buffer boundary — at
 * most it is delayed, which our own NBLK=6 buffer absorbs.
 *
 * WHY THIS WAS NECESSARY: until 2026-08-03 we sent blindly, and
 * measurements showed that under network load ~7 blocks/s arrived in the
 * wrong descriptor on the far side. Every earlier symptom (LOST, BACKWARDS,
 * smeared carrier, silent APRS) stemmed from this single root cause.
 *
 * WIRING:  FG23 PD2 / EXP 9  <-  ESP32 GPIO2   (GND already common)
 *
 * While the wire is not connected, leave it at 0: with an unconnected pin
 * the wait would time out on every block, and the stream would be
 * continuously delayed. */
#define IQ_RDY_ENABLE     1      /* 2026-08-03: wire connected and working */

/* STRICT MODE. The 2026-08-03 live run showed 8007 timeouts, and the
 * remaining rare excursions (d=25) are tied exactly to these: after a
 * timeout we sent ANYWAY — the old blind behaviour — and the block landed
 * in an unloaded descriptor on the far side. In strict mode we do NOT send
 * after a timeout: the block stays, the next pump retries. Our own NBLK=6
 * buffer absorbs this easily; if the far side is dead for long (e.g.
 * rebooting), our own dropped-block counter grows — WHOLE blocks are lost
 * on our side, visibly, not half-blocks silently on the far side. */
#define RDY_STRICT        1
#define RDY_PORT          gpioPortD
#define RDY_PIN           2
/* Maximum wait for RDY. One block period is 5.1 ms, our own buffer is
 * 6 deep — a 3 ms wait is absorbed easily. If it expires, the block is
 * sent ANYWAY (the old behaviour) and counted: the counter shows if the
 * far side cannot keep up over time. */
#define RDY_TIMEOUT_US    3000u

/* ---------------- PINS ---------------- */
#define SPI_MOSI_PORT  gpioPortC
#define SPI_MOSI_PIN   0      /* PC00 = EXP 10 -> GPIO6 : data  */
#define SPI_CLK_PORT   gpioPortC
#define SPI_CLK_PIN    5      /* PC05 = EXP 15 -> GPIO5 : clock */
#define SPI_CS_PORT    gpioPortA
#define SPI_CS_PIN     7      /* PA07 = EXP 13 -> GPIO7 : CS    */

/* Maximum drive strength + slew rate on port C. At a multi-MHz clock this
 * matters; it applies per port, and all three signals are on port C. */
#define SPI_FAST_SLEW  1

/* CS setup / hold time in microseconds.
 *
 * The ESP32 SPI slave cannot react instantly to CS going low: the hardware
 * must load the DMA descriptor before the first clock edge arrives.
 * Espressif explicitly mentions this in the slave documentation. 10 us
 * against a 2080 us block time is 0.5% overhead — unnoticeable, and in
 * exchange no transactions are lost. Can be reduced once everything is
 * stable. */
#define SPI_CS_SETUP_US  10u
#define SPI_CS_HOLD_US    5u

/* CS POLARITY.
 * 0 = active LOW — the SPI standard, and what the ESP32 slave expects.
 * 1 = active HIGH — experiment only, if a polarity mix-up is suspected.
 *     (If the ESP expects active-low and we drive high, CS appears
 *     active practically all the time on its side, and it floods
 *     truncated transactions — exactly what we saw with a floating CS.) */
#define SPI_CS_ACTIVE_HIGH  0

#if SPI_CS_ACTIVE_HIGH
#define CS_ASSERT()    GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN)
#define CS_RELEASE()   GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN)
#define CS_IDLE_LEVEL  0u
#else
#define CS_ASSERT()    GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN)
#define CS_RELEASE()   GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN)
#define CS_IDLE_LEVEL  1u
#endif

/* ================= BLOCK FORMAT ================= */

/* The header is 16 bytes, the payload 256 complex samples (I first, then
 * Q), both little-endian. The LDMA shifts it out with byteSwap, so the
 * byte buffer on the other side is BIT-IDENTICAL to this memory image —
 * nothing needs swapping. */
#define IQ_BLK_MAGIC   0x32425149u      /* 'I','Q','B','2' */

typedef struct {
  uint32_t magic;      /* IQ_BLK_MAGIC */
  uint32_t seq;        /* increments per block — LOSS IS COMPUTED FROM THIS */
  uint16_t nsamp;      /* number of complex samples (BLK_SAMPLES) */
  uint16_t decim;      /* the total decimation, so the receiver knows the rate */
  uint32_t fs_in_hz;   /* the chip's input sample rate */
} iq_blk_hdr_t;

typedef struct {
  iq_blk_hdr_t hdr;
  int16_t      iq[BLK_SAMPLES * 2];    /* I,Q,I,Q ... */
} iq_blk_t;

/* 4-byte aligned: the LDMA performs half-word transfers. */
static iq_blk_t s_blk[NBLK] __attribute__((aligned(4)));

#define BLK_BYTES   (sizeof(iq_blk_t))          /* 1040 */
#define BLK_WORDS   (BLK_BYTES / 2u)            /*  520 halfword */

/* ================= STATE ================= */

static const uint8_t *s_fifo      = NULL;
static uint16_t       s_fifo_size = 0;
static volatile uint16_t s_rd     = 0;      /* own read index, bytes */

static volatile bool s_active = false;
static RAIL_Handle_t s_rail_for_restart = NULL;

/* Fine offset of the synth. RAIL_StartRx does NOT preserve it, so it must
 * be rewritten after EVERY restart — otherwise starting the stream discards
 * the crystal calibration and the tuning. Set by app.c. */
static volatile int32_t s_offset_tick = 0;

void iq_stream_set_freq_tick(int32_t tick)
{
  s_offset_tick = tick;
  if (s_active && s_rail_for_restart != NULL) {
    RAIL_SetFreqOffset(s_rail_for_restart,
                       (RAIL_FrequencyOffset_t)s_offset_tick);
  }
}

/* THIS must be called for every RAIL_StartRx, never the bare StartRx. */
static void stream_start_rx(RAIL_Handle_t rail, uint16_t channel)
{
  RAIL_StartRx(rail, channel, NULL);
  RAIL_SetFreqOffset(rail, (RAIL_FrequencyOffset_t)s_offset_tick);
}
static uint16_t      s_channel_for_restart = 0;

/* --- TWO-STAGE CIC, int32 THROUGHOUT ---
 * The first version used int64, which with -Og is so slow that it starves
 * the main loop at 1 Msps. Two cascaded CICs in int32: the FIRST runs at
 * the full rate (R1=8, cheap), the second at one eighth of the rate.
 *
 * Stage 1 is SECOND-ORDER: the section running at the input rate is the
 * bottleneck (the third-order version managed only 522 ksps instead of
 * 1000). Acceptable because our final band is very narrow compared to the
 * intermediate rate, so the alias bands of stage 1 fall on the CIC nulls. */
#define CIC_R1        8u
#define CIC_SH1       6u                    /* 2*log2(8) */

static int32_t a1_i, a2_i, a1_q, a2_q;                  /* stage 1 integrator */
static int32_t b1_i, b2_i, b1_q, b2_q;                  /* stage 1 comb */
static int32_t c1_i, c2_i, c3_i, c1_q, c2_q, c3_q;      /* stage 2 integrator */
static int32_t d1_i, d2_i, d3_i, d1_q, d2_q, d3_q;      /* stage 2 comb */
#if IQ_STREAM_DC_BLOCK
static int64_t dc_i, dc_q;      /* 64 bit: 2^20 * 2^14 does not fit in 32 */
#endif

static uint32_t s_cnt1 = 0, s_cnt2 = 0;
static uint32_t s_decim_R  = 256;
static uint32_t s_R2       = 32;

/* 2^24 fixed-point normaliser: s_norm = 2^24 / R2^3. */
#define NORM_SHIFT  24
static int32_t  s_norm     = (1 << NORM_SHIFT) / 32768;

/* --- block handling: ONE producer (ISR) and ONE consumer (main loop),
 * so two monotonic counters suffice. pending = s_prod - s_cons. --- */
static volatile uint32_t s_prod = 0;     /* blocks completed */
static volatile uint32_t s_cons = 0;     /* blocks sent out */
static volatile uint16_t s_fill_n = 0;   /* samples in the block being filled */
static uint32_t s_seq = 0;               /* outgoing sequence number */
static int16_t  s_rssi_dbm = -128;       /* latest RAIL RSSI (dBm), for the header */

/* Statistics */
static volatile uint32_t s_out_samples = 0;
static volatile uint32_t s_blk_drops   = 0;   /* no free block available */
static uint32_t s_rdy_waits    = 0;   /* times we had to wait for RDY */
static uint32_t s_rdy_timeouts = 0;   /* times the wait expired */
static uint32_t s_rdy_skips    = 0;   /* strict mode: deferred sends */
static uint64_t s_rdy_wait_us  = 0;   /* total wait time */
static volatile uint32_t s_fifo_ovf    = 0;
static volatile uint32_t s_clip        = 0;
static volatile uint32_t s_pump_calls  = 0;
static volatile uint32_t s_isr_events  = 0;
static volatile bool     s_need_restart = false;
static RAIL_Time_t s_t_start = 0;

/* ============ MEASURING THE INPUT RATE, NOT ASSUMING IT ============
 *
 * IQ_STREAM_FS_IN_HZ used to be a DEFINE reflecting the sample rate that
 * followed from the acquisition bandwidth set in the Radio Configurator.
 * That is a source of silent error: if the PHY is changed (e.g. the
 * acquisition bandwidth moves to 100 kHz), the define stays, and from then
 * on EVERY number lies — SDR++ gets the wrong rate, audio plays at the
 * wrong pitch, the demodulator computes the wrong filters. Nothing fails
 * visibly; everything is just off.
 *
 * Hence we MEASURE: count the samples read from the FIFO and the time. */
/* MEASUREMENT WINDOW. 2 seconds: long enough for the measurement noise
 * to be negligible. */
#define FS_MEAS_MIN_US     2000000u

/* DEADBAND in per mille. The value is only changed for a larger deviation.
 *
 * WHY IT IS NEEDED: the first version snapped the measured value to the
 * 39 MHz / N grid, because "the rate is quantised". Reality: 39e6 / 400000
 * = 97.5 — the actual rate sits EXACTLY between two grid points, and the
 * rounding flipped between 97 and 98 (397959 <-> 402061). Every flip
 * triggered a full reconfiguration at both ends of the chain, plus two
 * log lines — ten times per second. That is where the block loss came
 * from.
 *
 * Lesson: do not snap to a grid unless you know FOR CERTAIN the signal is
 * on it. A deadband gives the same stability without assumptions. */
#define FS_DEADBAND_PPT    10u        /* 1% */

static volatile uint32_t s_fs_in_hz  = IQ_STREAM_FS_IN_HZ;
static uint32_t          s_fs_cnt    = 0;      /* complex samples in the window */
static RAIL_Time_t       s_fs_t0     = 0;
static volatile uint32_t s_fs_report = 0;      /* !=0 -> the main loop prints it */

/* NOTE: THIS RUNS FROM AN ISR (iq_stream_on_event).
 * NO printf, no blocking operation of any kind may be in here. The first
 * version had a printf to VCOM here — at 115200 that is ~6 ms of blocking
 * inside the radio interrupt, several times per second. Therefore it only
 * leaves a flag, and the main loop does the printing. */
static void fs_account(uint32_t nsamples)
{
  RAIL_Time_t now = RAIL_GetTime();
  if (s_fs_t0 == 0) { s_fs_t0 = now; s_fs_cnt = 0; return; }

  s_fs_cnt += nsamples;
  uint32_t dt = (uint32_t)(now - s_fs_t0);
  if (dt < FS_MEAS_MIN_US) return;

  uint32_t meas = (uint32_t)(((uint64_t)s_fs_cnt * 1000000ull) / dt);
  s_fs_t0  = now;
  s_fs_cnt = 0;
  if (meas < 1000u) return;

  uint32_t cur  = s_fs_in_hz;
  uint32_t diff = (meas > cur) ? (meas - cur) : (cur - meas);
  if (diff * 1000u > cur * FS_DEADBAND_PPT) {
    s_fs_in_hz  = meas;
    s_fs_report = meas;            /* the main loop prints it */
  }
}


/* --- CS timing measurement ---
 * CS should be low for the block transmit time (2.08 ms @ 4 MHz). If it
 * stays longer, the other side receives the next transaction with CS
 * already active, and it closes immediately, empty. So we measure WHICH
 * condition is late: LDMA done, or TXC. */
static volatile uint32_t s_cs_t0      = 0;   /* time of CS going low */
static volatile uint32_t s_cs_t_ldma  = 0;   /* when the LDMA finished */
static volatile bool     s_ldma_seen  = false;
static volatile uint64_t s_sum_ldma_us = 0;  /* CS low -> LDMA done */
static volatile uint64_t s_sum_txc_us  = 0;  /* LDMA done -> TXC */
static volatile uint32_t s_max_cs_us   = 0;
static volatile uint32_t s_cs_meas     = 0;

/* --- THE ACTUAL LEVEL OF THE CS PIN ---
 * GPIO_PinInGet reads the PAD, not the output register. If something
 * external pulls the line, the level read differs from what we wrote. We
 * sample from the pump (~48 kHz, i.e. ~1000 samples per block), so the
 * ACTUAL duty cycle and any contention can be measured — regardless of
 * what the multimeter shows. */
static volatile uint32_t s_pad_samples  = 0;
static volatile uint32_t s_pad_high     = 0;
static volatile uint32_t s_pad_mismatch = 0;

/* LDMA */
static LDMA_Descriptor_t s_desc;
static unsigned int      s_dma_ch = 0;
static bool              s_dma_alloc = false;
static volatile bool     s_tx_busy = false;

/* ================= SPI ================= */

static void spi_setup(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART0, true);

#if SPI_FAST_SLEW
  /* The SiSDK 2025.6 emlib no longer provides GPIO_DriveStrengthSet(),
   * hence a direct register write. SLEWRATE 0..7 (default 4), 7 is the
   * steepest; DRIVESTRENGTH 0 = STRONG. */
  /* SCLK and MOSI are on port C, CS on A — both are configured. */
  {
    static const GPIO_Port_TypeDef ports[2] = { gpioPortC, gpioPortA };
    for (int pi = 0; pi < 2; pi++) {
      uint32_t ctrl = GPIO->P[ports[pi]].CTRL;
      ctrl &= ~(_GPIO_P_CTRL_SLEWRATE_MASK | _GPIO_P_CTRL_SLEWRATEALT_MASK);
      ctrl |= (7u << _GPIO_P_CTRL_SLEWRATE_SHIFT)
            | (7u << _GPIO_P_CTRL_SLEWRATEALT_SHIFT);
#if defined(_GPIO_P_CTRL_DRIVESTRENGTH_MASK)
      ctrl &= ~_GPIO_P_CTRL_DRIVESTRENGTH_MASK;
#endif
#if defined(_GPIO_P_CTRL_DRIVESTRENGTHALT_MASK)
      ctrl &= ~_GPIO_P_CTRL_DRIVESTRENGTHALT_MASK;
#endif
      GPIO->P[ports[pi]].CTRL = ctrl;
    }
  }
#endif

  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CLK_PORT,  SPI_CLK_PIN,  gpioModePushPull, 0);
  /* CS is driven by US, as plain GPIO. */
  GPIO_PinModeSet(SPI_CS_PORT,   SPI_CS_PIN,   gpioModePushPull, CS_IDLE_LEVEL);

  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
  init.enable       = usartEnableTx;      /* transmit only */
  init.baudrate     = SPI_HZ;
  init.databits     = usartDatabits16;    /* 16-bit frame, as before */
  init.master       = true;
  init.msbf         = true;               /* MSB first — SPI convention */
  init.clockMode    = usartClockMode0;    /* CPOL=0, CPHA=0 = SPI mode 0 */
  init.autoCsEnable = false;              /* !!! CS is ours, GPIO */
  init.autoTx       = false;

  USART_InitSync(USART0, &init);

  /* Pin routing: ONLY TX and CLK. CSROUTE/CSPEN is DELIBERATELY omitted —
   * otherwise the peripheral would also toggle CS and collide with the
   * manual block framing. */
  GPIO->USARTROUTE[0].TXROUTE =
      ((uint32_t)SPI_MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
    | ((uint32_t)SPI_MOSI_PIN  << _GPIO_USART_TXROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[0].CLKROUTE =
      ((uint32_t)SPI_CLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
    | ((uint32_t)SPI_CLK_PIN  << _GPIO_USART_CLKROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[0].ROUTEEN =
      GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_CLKPEN;

  printf("# SPI: %lu Hz, mode 0, 16 bit, MSB first\r\n",
         (unsigned long)SPI_HZ);
  printf("# SPI pins: MOSI=PC%02u/EXP10  SCLK=PC%02u/EXP15  "
         "CS=PA%02u/EXP13 (manual)\r\n",
         (unsigned)SPI_MOSI_PIN, (unsigned)SPI_CLK_PIN, (unsigned)SPI_CS_PIN);
  printf("# block: %lu bytes (%lu samples), ~%lu us transmit time\r\n",
         (unsigned long)BLK_BYTES, (unsigned long)BLK_SAMPLES,
         (unsigned long)((uint64_t)BLK_BYTES * 8ull * 1000000ull / SPI_HZ));
}

static void spi_teardown(void)
{
  if (s_dma_alloc) LDMA_StopTransfer((int)s_dma_ch);
  s_tx_busy = false;
  GPIO->USARTROUTE[0].ROUTEEN = 0;
  USART_Reset(USART0);
  /* CS must stay driven high even AFTER the route is disabled — otherwise
   * the other side sees a floating input again. */
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, CS_IDLE_LEVEL);
}

/* Shift out one block: CS low, start LDMA. CS is raised in the pump, once
 * the LDMA has finished AND the shift register has emptied as well. */
static void spi_send_block(iq_blk_t *b)
{
  if (!s_dma_alloc) {
    DMADRV_Init();
    if (DMADRV_AllocateChannel(&s_dma_ch, NULL) != ECODE_EMDRV_DMADRV_OK) {
      printf("# LDMA channel not available!\r\n");
      return;
    }
    s_dma_alloc = true;
  }

  s_desc.xfer.structType  = ldmaCtrlStructTypeXfer;
  s_desc.xfer.structReq   = 0;
  s_desc.xfer.xferCnt     = BLK_WORDS - 1u;
  /* byteSwap: swaps the two bytes of the half-word before sending. This
   * makes the byte buffer on the other side BIT-IDENTICAL to this memory
   * image, with nothing to rotate there. (The USART shifts out MSB-first.) */
  s_desc.xfer.byteSwap    = 1;
  s_desc.xfer.blockSize   = ldmaCtrlBlockSizeUnit1;
  s_desc.xfer.doneIfs     = 0;
  s_desc.xfer.reqMode     = ldmaCtrlReqModeBlock;
  s_desc.xfer.decLoopCnt  = 0;
  s_desc.xfer.ignoreSrec  = 0;
  s_desc.xfer.srcInc      = ldmaCtrlSrcIncOne;
  s_desc.xfer.size        = ldmaCtrlSizeHalf;
  s_desc.xfer.dstInc      = ldmaCtrlDstIncNone;
  s_desc.xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
  s_desc.xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
  s_desc.xfer.srcAddr     = (uint32_t)b;
  s_desc.xfer.dstAddr     = (uint32_t)&USART0->TXDOUBLE;
  s_desc.xfer.linkMode    = ldmaLinkModeAbs;
  s_desc.xfer.link        = 0;
  s_desc.xfer.linkAddr    = 0;

  LDMA_TransferCfg_t cfg =
      LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_TXBL);

#if IQ_RDY_ENABLE
  /* Is the far side ready? The wait happens here, BEFORE CS goes low —
   * until then the block is ours, nothing can be lost. */
  if (!GPIO_PinInGet(RDY_PORT, RDY_PIN)) {
    RAIL_Time_t t0 = RAIL_GetTime();
    RAIL_Time_t tl = t0 + RDY_TIMEOUT_US;
    bool timed_out = false;
    s_rdy_waits++;
    while (!GPIO_PinInGet(RDY_PORT, RDY_PIN)) {
      if ((int32_t)(RAIL_GetTime() - tl) >= 0) {
        s_rdy_timeouts++;
        timed_out = true;
        break;
      }
    }
    s_rdy_wait_us += (uint64_t)(RAIL_GetTime() - t0);
#if RDY_STRICT
    if (timed_out) {
      /* Do NOT send blindly. The block stays ours (s_cons not advanced,
       * s_tx_busy false), the next pump retries. */
      s_rdy_skips++;
      return;
    }
#else
    (void)timed_out;
#endif
  }
#endif

  /* Now we are definitely sending (the strict-timeout branch returned
   * above). The sequence number is written ONLY HERE — so an RDY timeout
   * does not burn a sequence number and no phantom gap appears on the ESP
   * side. The DMA starts after CS_ASSERT, so it carries the seq written
   * now. */
  b->hdr.seq = s_seq++;

  CS_ASSERT();                                  /* CS active */

  /* CS SETUP TIME. The ESP32 SPI slave needs a few microseconds after CS
   * goes low before the clock starts — this is when the hardware loads the
   * DMA descriptor. If the first clock comes too early, the transaction is
   * lost or truncated. Otherwise there would be only 1-2 us between CS
   * going low and the LDMA start. */
  {
    RAIL_Time_t t = RAIL_GetTime() + SPI_CS_SETUP_US;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
  }

  LDMA_StartTransfer((int)s_dma_ch, &cfg, &s_desc);
  s_cs_t0     = (uint32_t)RAIL_GetTime();
  s_ldma_seen = false;
  s_tx_busy   = true;
}

/* ================= INIT ================= */

void iq_stream_init(const uint8_t *fifo_base, uint16_t fifo_bytes)
{
  s_fifo = fifo_base;
  s_fifo_size = fifo_bytes;

  /* Drive CS inactive (high) IMMEDIATELY, already at power-up.
   *
   * WHY: after reset PC03 is a disabled input, so the CS input on the
   * other side FLOATS. An SPI slave closes a transaction on every noise
   * edge — measured ~4200 truncated transactions per second before we had
   * started anything. A push-pull high level eliminates this entirely. */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, CS_IDLE_LEVEL);
#if IQ_RDY_ENABLE
  /* Input with PULL-DOWN: if the wire falls off, RDY reads as a constant
   * 0 and the timeout counter reveals it immediately — not a silent fault. */
  GPIO_PinModeSet(RDY_PORT, RDY_PIN, gpioModeInputPull, 0);
#endif
}

bool iq_stream_active(void) { return s_active; }

uint32_t iq_stream_out_sps(uint16_t decim)
{
  uint32_t r = (decim < CIC_R1) ? CIC_R1 : (uint32_t)decim;
  r = (r / CIC_R1) * CIC_R1;
  if (r == 0u) r = CIC_R1;
  /* From the MEASURED rate, not the define. Until a measurement exists,
   * the define is the default. */
  return s_fs_in_hz / r;
}

/* ================= SAMPLE -> BLOCK ================= */

static inline void blk_put_sample(int16_t vi, int16_t vq)
{
  if ((s_prod - s_cons) >= NBLK) {   /* no free block */
    s_blk_drops++;
    return;
  }

  iq_blk_t *b = &s_blk[s_prod % NBLK];
  uint16_t n = s_fill_n;
  b->iq[n * 2u + 0u] = vi;           /* I first */
  b->iq[n * 2u + 1u] = vq;
  n++;

  if (n >= BLK_SAMPLES) {
    s_fill_n = 0;
    s_prod++;                        /* THIS makes it sendable */
  } else {
    s_fill_n = n;
  }
}

/* ================= CIC ================= */

static void process_block(const uint8_t *p, uint16_t nbytes)
{
  typedef struct { int16_t q, i; } iq_in_t;   /* BUFC order: Q FIRST */
  uint16_t nsamp = (uint16_t)(nbytes / sizeof(iq_in_t));

  int32_t A1i = a1_i, A2i = a2_i;
  int32_t A1q = a1_q, A2q = a2_q;
  uint32_t cnt1 = s_cnt1;

  /* ONE 32-BIT READ instead of four byte reads. The FIFO is 4-byte
   * aligned and the indices are multiples of 4, so this is safe. */
  const uint32_t *w = (const uint32_t *)(const void *)p;

  for (uint16_t k = 0; k < nsamp; k++) {
    uint32_t v = *w++;
    int16_t q = (int16_t)(uint16_t)(v & 0xFFFFu);        /* Q FIRST */
    int16_t i = (int16_t)(uint16_t)(v >> 16);

    A1i += i;  A2i += A1i;
    A1q += q;  A2q += A1q;

    if (++cnt1 < CIC_R1) continue;
    cnt1 = 0;

    int32_t e, f;
    e = A2i - b1_i;  b1_i = A2i;
    f = e    - b2_i;  b2_i = e;
    int32_t s1i = f >> CIC_SH1;

    e = A2q - b1_q;  b1_q = A2q;
    f = e    - b2_q;  b2_q = e;
    int32_t s1q = f >> CIC_SH1;

    c1_i += s1i;  c2_i += c1_i;  c3_i += c2_i;
    c1_q += s1q;  c2_q += c1_q;  c3_q += c2_q;

    if (++s_cnt2 < s_R2) continue;
    s_cnt2 = 0;

    int32_t g;
    e = c3_i - d1_i;  d1_i = c3_i;
    f = e     - d2_i;  d2_i = e;
    g = f     - d3_i;  d3_i = f;
    int32_t oi = (int32_t)(((int64_t)g * s_norm) >> NORM_SHIFT);

    e = c3_q - d1_q;  d1_q = c3_q;
    f = e     - d2_q;  d2_q = e;
    g = f     - d3_q;  d3_q = f;
    int32_t oq = (int32_t)(((int64_t)g * s_norm) >> NORM_SHIFT);

#if IQ_STREAM_DC_BLOCK
    if (oi >  DC_PRECLAMP) oi =  DC_PRECLAMP;
    else if (oi < -DC_PRECLAMP) oi = -DC_PRECLAMP;
    if (oq >  DC_PRECLAMP) oq =  DC_PRECLAMP;
    else if (oq < -DC_PRECLAMP) oq = -DC_PRECLAMP;

    dc_i += (int64_t)oi - (dc_i >> DC_SHIFT);
    oi   -= (int32_t)(dc_i >> DC_SHIFT);
    dc_q += (int64_t)oq - (dc_q >> DC_SHIFT);
    oq   -= (int32_t)(dc_q >> DC_SHIFT);
#endif

    /* Clip to +-32767, NOT -32768: 0x8000 is the only 16-bit value that a
     * one-bit slip turns into an exact zero. Costs one LSB; in exchange
     * that trap is gone for good. */
    if (oi >  32767) { oi =  32767; s_clip++; }
    else if (oi < -32767) { oi = -32767; s_clip++; }
    if (oq >  32767) { oq =  32767; s_clip++; }
    else if (oq < -32767) { oq = -32767; s_clip++; }

    blk_put_sample((int16_t)oi, (int16_t)oq);
    s_out_samples++;
  }

  a1_i = A1i; a2_i = A2i;
  a1_q = A1q; a2_q = A2q;
  s_cnt1 = cnt1;
}

/* ================= RAIL EVENT ================= */

bool iq_stream_on_event(RAIL_Handle_t rail, RAIL_Events_t events)
{
  if (!s_active) return false;

  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    s_fifo_ovf++;
    /* Flag ONLY. Calling RAIL_ResetFifo while RX is running kills
     * reception — the restart happens in the main loop. */
    s_need_restart = true;
    return true;
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    s_isr_events++;

    uint8_t guard = 8u;
    uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail);

    while (avail >= 512u && guard--) {
      uint16_t n = (uint16_t)(avail & ~3u);
      if (n > 4096u) n = 4096u;
      if (n == 0u) break;

      /* Read IN PLACE from the FIFO (zero-copy), with wrap-around. */
      uint16_t first = (uint16_t)(s_fifo_size - s_rd);
      if (first > n) first = n;
      process_block(&s_fifo[s_rd], first);
      if (n > first) process_block(&s_fifo[0], (uint16_t)(n - first));

      RAIL_ReadRxFifo(rail, NULL, n);      /* only the pointer advances */
      s_rd = (uint16_t)((s_rd + n) % s_fifo_size);

      /* Measuring the ACTUAL input rate. This is how the rest of the chain
       * (SDR++, demodulator) knows how many samples per second it gets —
       * not from a define that can go stale. 4 bytes = one complex sample. */
      fs_account((uint32_t)(n >> 2));

      avail = RAIL_GetRxFifoBytesAvailable(rail);
    }
  }

  return true;
}

/* ================= START / STOP ================= */

void iq_stream_start(RAIL_Handle_t rail, uint16_t channel,
                     uint16_t decim, uint8_t out_shift)
{
  if (decim < CIC_R1) decim = CIC_R1;
  if (decim > 4096u)  decim = 4096u;
  decim = (uint16_t)((decim / CIC_R1) * CIC_R1);

  s_decim_R = decim;
  s_R2      = decim / CIC_R1;
  if (s_R2 < 1u) s_R2 = 1u;

  uint64_t gain = (uint64_t)s_R2 * s_R2 * s_R2;
  s_norm = (int32_t)(((1ull << NORM_SHIFT) + gain / 2ull) / gain);
  if (s_norm < 1) s_norm = 1;
  if (out_shift < 8u) s_norm = (int32_t)(s_norm << out_shift);

  a1_i = a2_i = a1_q = a2_q = 0;
  b1_i = b2_i = b1_q = b2_q = 0;
  c1_i = c2_i = c3_i = c1_q = c2_q = c3_q = 0;
  d1_i = d2_i = d3_i = d1_q = d2_q = d3_q = 0;
#if IQ_STREAM_DC_BLOCK
  dc_i = dc_q = 0;
#endif
  s_cnt1 = s_cnt2 = 0;
  s_rdy_waits = 0; s_rdy_timeouts = 0; s_rdy_wait_us = 0; s_rdy_skips = 0;
  s_prod = s_cons = 0; s_fill_n = 0; s_seq = 0;
  s_out_samples = 0; s_blk_drops = 0; s_fifo_ovf = 0; s_clip = 0;
  s_pump_calls = 0; s_isr_events = 0; s_need_restart = false;
  s_tx_busy = false;
  s_sum_ldma_us = 0; s_sum_txc_us = 0; s_max_cs_us = 0; s_cs_meas = 0;
  s_pad_samples = 0; s_pad_high = 0; s_pad_mismatch = 0;
  memset(s_blk, 0, sizeof(s_blk));

  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(rail, false, true);
  s_rd = 0;

  RAIL_SetRxFifoThreshold(rail, 2048u);
  RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  spi_setup();

  {
    uint32_t sps = iq_stream_out_sps((uint16_t)s_decim_R);
    printf("# output: R=%lu -> %lu sps, %lu B/s, one block every %lu ms\r\n",
           (unsigned long)s_decim_R, (unsigned long)sps,
           (unsigned long)(sps * 4u),
           (unsigned long)(sps ? (BLK_SAMPLES * 1000u / sps) : 0u));
  }
#if IQ_STREAM_DC_BLOCK
  printf("# DC block ON (shift=%u, time constant ~%lu samples)\r\n",
         (unsigned)DC_SHIFT, (unsigned long)(1ul << DC_SHIFT));
#endif

  s_rail_for_restart = rail;
  s_channel_for_restart = channel;
  s_t_start = RAIL_GetTime();
  s_fs_t0 = 0; s_fs_cnt = 0;      /* new measurement window */
  s_active = true;
  stream_start_rx(rail, channel);
}

void iq_stream_stop(RAIL_Handle_t rail, uint16_t channel)
{
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  s_active = false;
  spi_teardown();

  uint32_t dur_us = (uint32_t)(RAIL_GetTime() - s_t_start);
  uint32_t rate = 0;
  if (dur_us > 0u) {
    rate = (uint32_t)(((uint64_t)s_out_samples * 1000000ull) / dur_us);
  }

  printf("\r\n# stream stopped: %lu output samples, %lu.%03lu ksps\r\n",
         (unsigned long)s_out_samples,
         (unsigned long)(rate / 1000u), (unsigned long)(rate % 1000u));
  printf("# blocks sent: %lu   dropped blocks: %lu   "
         "FIFO overflow: %lu\r\n",
         (unsigned long)s_cons, (unsigned long)s_blk_drops,
         (unsigned long)s_fifo_ovf);
#if IQ_RDY_ENABLE
  printf("# RDY: %lu waits (avg %lu us), %lu timeouts, "
         "%lu deferred sends\r\n",
         (unsigned long)s_rdy_waits,
         (unsigned long)(s_rdy_waits ? s_rdy_wait_us / s_rdy_waits : 0),
         (unsigned long)s_rdy_timeouts, (unsigned long)s_rdy_skips);
#endif
  printf("# main loop: %lu pump/s   ISR events: %lu\r\n",
         (unsigned long)(dur_us ? (uint32_t)(((uint64_t)s_pump_calls
                          * 1000000ull) / dur_us) : 0u),
         (unsigned long)s_isr_events);

  {
    uint32_t promille = (s_out_samples > 0u)
        ? (uint32_t)(((uint64_t)s_clip * 1000ull) / (s_out_samples * 2ull))
        : 0u;
    printf("# clipping: %lu values (%lu.%01lu%%)%s\r\n",
           (unsigned long)s_clip,
           (unsigned long)(promille / 10u), (unsigned long)(promille % 10u),
           (s_clip > 0u) ? "  <-- SATURATION" : "");
  }

  /* What is in the last filled block? This separates the DSP from the
   * link: if there is data here but none on the other side, the link is
   * at fault. */
  {
    const iq_blk_t *b = &s_blk[(s_prod ? (s_prod - 1u) : 0u) % NBLK];
    int32_t peak = 0;
    uint32_t nonzero = 0;
    for (uint32_t k = 0; k < BLK_SAMPLES * 2u; k++) {
      int16_t v = b->iq[k];
      if (v != 0) nonzero++;
      int32_t a = (v < 0) ? -(int32_t)v : (int32_t)v;
      if (a > peak) peak = a;
    }
    printf("# last block: peak=%ld  non-zero=%lu / %lu\r\n",
           (long)peak, (unsigned long)nonzero,
           (unsigned long)(BLK_SAMPLES * 2u));
    printf("# first 8 words (I Q I Q ...):");
    for (int k = 0; k < 8; k++) printf(" %04X", (unsigned)(uint16_t)b->iq[k]);
    printf("\r\n");
  }

  /* --- CS timing: THIS tells where the time is lost --- */
  if (s_cs_meas > 0u) {
    uint32_t a_ldma = (uint32_t)(s_sum_ldma_us / s_cs_meas);
    uint32_t a_txc  = (uint32_t)(s_sum_txc_us  / s_cs_meas);
    uint32_t elm    = (uint32_t)((uint64_t)BLK_BYTES * 8ull
                                 * 1000000ull / SPI_HZ);
    printf("# CS low: avg %lu us (theoretical %lu us), max %lu us\r\n",
           (unsigned long)(a_ldma + a_txc), (unsigned long)elm,
           (unsigned long)s_max_cs_us);
    printf("#   of which CS low -> LDMA done : %lu us\r\n",
           (unsigned long)a_ldma);
    printf("#   of which LDMA done -> TXC   : %lu us %s\r\n",
           (unsigned long)a_txc,
           (a_txc > 200u) ? "  <-- TIME IS LOST HERE" : "");
  }

  /* --- THE ACTUAL DUTY CYCLE OF THE CS PIN ---
   * Obtained by reading the pin, so it is independent of the multimeter
   * AND of the output register. If the "mismatch" is non-zero, something
   * external is pulling the line. */
  if (s_pad_samples > 0u) {
    uint32_t hi_pm  = (uint32_t)(((uint64_t)s_pad_high * 1000ull)
                                 / s_pad_samples);
    uint32_t exp_pm = 1000u;
    if (s_cs_meas > 0u && s_cons > 0u) {
      uint32_t low_us_avg = (uint32_t)((s_sum_ldma_us + s_sum_txc_us)
                                       / s_cs_meas);
      uint32_t per_us = (uint32_t)(dur_us / (s_cons ? s_cons : 1u));
      if (per_us > 0u) {
        uint32_t low_pm = (uint32_t)(((uint64_t)low_us_avg * 1000ull)
                                     / per_us);
        exp_pm = (low_pm < 1000u) ? (1000u - low_pm) : 0u;
      }
    }
#if SPI_CS_ACTIVE_HIGH
    exp_pm = 1000u - exp_pm;
#endif
    printf("# CS pad: high %lu.%01lu%%  (expected from the written level "
           "%lu.%01lu%%)\r\n",
           (unsigned long)(hi_pm / 10u), (unsigned long)(hi_pm % 10u),
           (unsigned long)(exp_pm / 10u), (unsigned long)(exp_pm % 10u));
    printf("# CS pad mismatch vs. written level: %lu / %lu samples%s\r\n",
           (unsigned long)s_pad_mismatch, (unsigned long)s_pad_samples,
           (s_pad_mismatch > (s_pad_samples / 100u))
             ? "   <-- SOMETHING EXTERNAL IS PULLING THE LINE!" : "   (clean)");
  }

  if (s_blk_drops > 0u) {
    printf("# -> the link cannot carry it: raise SPI_HZ or the decimation\r\n");
  }

  RAIL_ResetFifo(rail, false, true);
  stream_start_rx(rail, channel);
}

/* ================= MAIN LOOP: BLOCK -> SPI ================= */

/* Complete the SPI send in progress: LDMA done AND TXC, then CS up.
   Returns true while the send is still ongoing (the caller must not start
   a new one). Shared by the normal pump and the ext mode (scan.c). */
static bool spi_tx_poll(void)
{
  if (!s_tx_busy) return false;

  /* DOUBLE CONDITION. LDMA "done" means the last word has been PLACED in
   * the TX FIFO — not that it has gone out on the line. If CS were raised
   * here, the last few bytes would be left hanging. Hence TXC (transmit
   * complete) is required as well. */
  /* First condition: the LDMA has finished (last word placed in the FIFO). */
  if (!s_ldma_seen && LDMA_TransferDone((int)s_dma_ch)) {
    s_ldma_seen  = true;
    s_cs_t_ldma  = (uint32_t)RAIL_GetTime();
  }

  /* Second condition: the shift register has emptied as well. */
  if (s_ldma_seen && (USART0->STATUS & USART_STATUS_TXC)) {
    /* CS HOLD TIME: leave a little time after the last clock edge before
     * raising CS. This guarantees the other side has time to clock in
     * the last bit. */
    RAIL_Time_t t = RAIL_GetTime() + SPI_CS_HOLD_US;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }

    CS_RELEASE();                              /* CS inactive */

    uint32_t t_end = (uint32_t)RAIL_GetTime();
    uint32_t d_all  = t_end - s_cs_t0;
    s_sum_ldma_us += (uint64_t)(s_cs_t_ldma - s_cs_t0);
    s_sum_txc_us  += (uint64_t)(t_end - s_cs_t_ldma);
    if (d_all > s_max_cs_us) s_max_cs_us = d_all;
    s_cs_meas++;

    s_cons++;
    s_tx_busy = false;
  }
  return s_tx_busy;
}

bool iq_stream_pump(void)
{
  /* Print the new rate from the measurement — HERE, in the main loop, not in the ISR. */
  if (s_fs_report) {
    uint32_t v = s_fs_report;
    s_fs_report = 0;
    printf("# measured input rate: %lu sps (output %lu sps)\r\n",
           (unsigned long)v, (unsigned long)(v / (s_decim_R ? s_decim_R : 1u)));
  }

  if (!s_active) return false;
  s_pump_calls++;

  /* The actual level of the CS PAD — not the output register! If
   * something external pulls the line, it shows up here. */
  {
    unsigned pad  = GPIO_PinInGet(SPI_CS_PORT, SPI_CS_PIN) ? 1u : 0u;
    unsigned want = s_tx_busy ? (CS_IDLE_LEVEL ^ 1u) : CS_IDLE_LEVEL;
    s_pad_samples++;
    if (pad) s_pad_high++;
    if (pad != want) s_pad_mismatch++;
  }

  if (s_need_restart) {
    s_need_restart = false;
    RAIL_Handle_t rail = s_rail_for_restart;
    if (rail != NULL) {
      RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
      RAIL_ResetFifo(rail, false, true);
      s_rd = 0;
      stream_start_rx(rail, s_channel_for_restart);
    }
  }

  /* --- is a send in progress? check whether it has finished --- */
  if (spi_tx_poll()) return true;

  /* --- is there a block to send? --- */
  if ((s_prod - s_cons) > 0u) {
    iq_blk_t *b = &s_blk[s_cons % NBLK];
    b->hdr.magic    = IQ_BLK_MAGIC;
    /* seq is NOT written here! On an RDY-strict timeout spi_send_block
     * returns before CS_ASSERT and the block stays — if seq were
     * incremented here, the retry would go out with a number one higher
     * and the ESP would see a PHANTOM GAP (st_lost++ + 5 ms silence). So
     * seq is written at the actual send, right before CS_ASSERT (see
     * spi_send_block). */
    b->hdr.nsamp    = (uint16_t)BLK_SAMPLES;
    b->hdr.decim    = (uint16_t)s_decim_R;
    /* The RAIL RSSI (dBm) is packed into the UPPER 12 bits of fs_in_hz:
     * the header has no separate field, and the 1040-byte block size (DMA,
     * reorder, SpyServer) is NOT touched. Lower 20 bits = actual rate
     * (<=1 048 575, 400000 fits easily), upper 12 bits = signed dBm.
     * The RSSI is refreshed every ~100 ms (the pump runs ~200 blocks/s). */
    static uint16_t rssi_div = 0;
    if (++rssi_div >= 20u) {
      rssi_div = 0;
      int16_t rq = RAIL_GetRssi(s_rail_for_restart, false);   /* 0.25 dBm units */
      if (rq != RAIL_RSSI_INVALID) s_rssi_dbm = (int16_t)(rq / 4);
    }
    b->hdr.fs_in_hz = (s_fs_in_hz & 0x000FFFFFu)
                    | ((uint32_t)((uint16_t)s_rssi_dbm & 0x0FFFu) << 20);
    spi_send_block(b);
  }

  return true;
}

/* ================= PIN TEST ('g') =================
 *
 * Toggles the three lines as plain GPIO, at SEPARATE frequencies. The edge
 * counter on the other side then tells unambiguously which signal arrives.
 *
 * IMPORTANT SIDE USE: all three pins toggle at EXACTLY 50%, so this is also
 * a DUTY-CYCLE REFERENCE. With 3.3 V logic the multimeter DC average must
 * be 1.65 V on all three. This measurement caught the I2S word select
 * fault on 2026-07-31, after every software lead had run out.
 */
void iq_stream_pin_test(uint32_t seconds)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO->USARTROUTE[0].ROUTEEN = 0;

  GPIO_PinModeSet(SPI_CS_PORT,   SPI_CS_PIN,   gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CLK_PORT,  SPI_CLK_PIN,  gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 0);

  printf("# pin test %lu s (all three EXACTLY 50%% -> DC average 1.65 V):\r\n",
         (unsigned long)seconds);
  printf("#   PA07 / EXP 13 / CS   = 1 kHz  (~2000 edges/s)\r\n");
  printf("#   PC05 / EXP 15 / SCLK = 2 kHz  (~4000 edges/s)\r\n");
  printf("#   PC00 / EXP 10 / MOSI = 4 kHz  (~8000 edges/s)\r\n");

  RAIL_Time_t t_end = RAIL_GetTime() + seconds * 1000000u;
  uint32_t n = 0;

  while ((int32_t)(RAIL_GetTime() - t_end) < 0) {
    RAIL_Time_t next = RAIL_GetTime() + 125u;      /* 8 kHz base tick */
    while ((int32_t)(RAIL_GetTime() - next) < 0) { }

    ++n;
    if ((n & 3u) == 0u) GPIO_PinOutToggle(SPI_CS_PORT,   SPI_CS_PIN);
    if ((n & 1u) == 0u) GPIO_PinOutToggle(SPI_CLK_PORT,  SPI_CLK_PIN);
    GPIO_PinOutToggle(SPI_MOSI_PORT, SPI_MOSI_PIN);
  }

  /* CS to INACTIVE (high), not to zero! If left low, the other side would
   * see itself continuously selected and flood truncated transactions.
   * The other two can go to zero. */
  GPIO_PinOutSet(SPI_CS_PORT,     SPI_CS_PIN);
  GPIO_PinOutClear(SPI_CLK_PORT,  SPI_CLK_PIN);
  GPIO_PinOutClear(SPI_MOSI_PORT, SPI_MOSI_PIN);
  printf("# pin test finished (CS set inactive)\r\n");
}

/* ================= EUSART CLOCK DIAGNOSTICS ('v') ================= */

void iq_stream_dump_uart_cfg(void)
{
  uint32_t clksel = CMU->EUSART0CLKCTRL;
  uint32_t cfg0   = EUSART0->CFG0;
  uint32_t clkdiv = EUSART0->CLKDIV;

  printf("\r\n# ---- EUSART0 (VCOM) clock diagnostics ----\r\n");
  printf("# CLKCTRL=0x%08lX  CFG0=0x%08lX  CLKDIV=0x%08lX\r\n",
         (unsigned long)clksel, (unsigned long)cfg0, (unsigned long)clkdiv);

  uint32_t f = CMU_ClockFreqGet(cmuClock_EUSART0);
  uint32_t ovs_field = (cfg0 & _EUSART_CFG0_OVS_MASK) >> _EUSART_CFG0_OVS_SHIFT;
  uint32_t ovs = (ovs_field == 0u) ? 16u : (ovs_field == 1u) ? 8u
               : (ovs_field == 2u) ? 6u  : (ovs_field == 3u) ? 4u : 0u;
  uint32_t div = (clkdiv & _EUSART_CLKDIV_DIV_MASK) >> _EUSART_CLKDIV_DIV_SHIFT;
  printf("# fclk=%lu Hz  OVS=%lu  DIV=%lu\r\n",
         (unsigned long)f, (unsigned long)ovs, (unsigned long)div);
  if (ovs && f) {
    uint64_t den = (uint64_t)ovs * (256ull + div);
    printf("# -> computed baud=%lu, achievable max=%lu\r\n",
           (unsigned long)(uint32_t)(((uint64_t)f * 256ull) / den),
           (unsigned long)(f / ovs));
  }

  printf("# ---- SPI (USART0) ----\r\n");
  printf("# USART0 clock=%lu Hz, requested SPI=%lu Hz\r\n",
         (unsigned long)CMU_ClockFreqGet(cmuClock_USART0),
         (unsigned long)SPI_HZ);
  printf("# USARTROUTE: TX=0x%08lX CLK=0x%08lX ROUTEEN=0x%08lX "
         "(CSPEN DELIBERATELY absent)\r\n",
         (unsigned long)GPIO->USARTROUTE[0].TXROUTE,
         (unsigned long)GPIO->USARTROUTE[0].CLKROUTE,
         (unsigned long)GPIO->USARTROUTE[0].ROUTEEN);
}

/* ================= EXT MODE: sending a foreign 1040-byte block =================
 * Used by scan.c: sends a block (SPECLINE, specline.h) WITHOUT a RAIL RX
 * stream, over the same SPI/LDMA/RDY path. The block must be EXACTLY
 * BLK_BYTES in size, and the seq field at header byte 4 is written by
 * spi_send_block (as for IQ). Mutually exclusive with the normal stream. */
static bool s_ext_active = false;

void iq_stream_ext_begin(void)
{
  if (s_active || s_ext_active) return;
  s_prod = s_cons = 0; s_seq = 0; s_tx_busy = false;
  s_rdy_waits = 0; s_rdy_timeouts = 0; s_rdy_wait_us = 0; s_rdy_skips = 0;
  spi_setup();
  s_ext_active = true;
}

void iq_stream_ext_end(void)
{
  if (!s_ext_active) return;
  spi_teardown();
  s_ext_active = false;
}

bool iq_stream_ext_busy(void)
{
  return s_tx_busy;
}

void iq_stream_ext_pump(void)
{
  if (!s_ext_active) return;
  (void)spi_tx_poll();
}

bool iq_stream_ext_send(const void *blk)
{
  if (!s_ext_active || s_tx_busy || blk == NULL) return false;
  /* Block buffer 0 is used; spi_send_block writes the seq into it. */
  memcpy(&s_blk[0], blk, BLK_BYTES);
  spi_send_block(&s_blk[0]);
  return s_tx_busy;     /* false = RDY-strict timeout, must be retried */
}

/* SPDX-License-Identifier: MIT
 *
 * cmdlink.c — second command input on the FG23 (ESP32 -> FG23)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * EUSART1, RX only, 115200 8N1, on pin PA06 = EXP 11.
 *
 * ================== WHY EUSART1 ==================
 *   EUSART0 : the VCOM (the terminal reached through the J-Link)
 *   USART0  : the SPI of the I/Q output (iq_stream.c)
 *   EUSART1 : free — this becomes the command input
 *
 * ================== WHY NO INTERRUPT ==================
 * Commands are rare (at most a few per second while tuning on the
 * phone), and the main loop runs anyway. An interrupt here would only
 * introduce a race with the stream ISR and give nothing in return. The
 * EUSART FIFO absorbs the jitter.
 *
 * ================== TROUBLESHOOTING ==================
 *   - measure PA06: at rest it MUST be HIGH (the UART idle level). If it
 *     floats, the ESP TX is not connected.
 *   - the GND of the two boards MUST be tied together. It already is,
 *     because of the SPI — but do not forget it when rewiring.
 *   - the cmdlink_stats() error counter counts framing errors: if it
 *     grows, the baud rate does not match.
 */

#include "cmdlink.h"
#include "em_eusart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "rail.h"        /* RAIL_GetTime() for the duplicate window */
#include <stddef.h>      /* NULL — Simplicity does not pull it in by itself */
#include <string.h>

/* ---- pin and baud ---- */
#define CMDLINK_EUSART      EUSART1
#define CMDLINK_ROUTE_IDX   1              /* GPIO->EUSARTROUTE[1] */
#define CMDLINK_CLOCK       cmuClock_EUSART1
#define CMDLINK_RX_PORT     gpioPortA
#define CMDLINK_RX_PIN      6              /* PA06 = EXP 11 */
#define CMDLINK_BAUD        115200u

#define CMDLINK_LINE_MAX    48   /* 2026-08-15: the W-scan command (W<kHz>,<span>,<nbin>,<floor>,<range>) is ~30 chars. */

static cmdlink_line_fn s_on_line = NULL;
static char     s_buf[CMDLINK_LINE_MAX];
static uint8_t  s_len = 0;
static uint32_t s_lines = 0;
static uint32_t s_errors = 0;

/* ---- duplicate suppression ----
 * 2026-11-03: ~25 % of the 16-char lines never acted on the mini (state
 * dependent, consecutive losses, not reproducible on the bench). The ESP32
 * therefore sends every line TWICE, 30 ms apart; an identical line within
 * CMDLINK_DUP_US is executed once. Needed because 'i' is a toggle and 'W'
 * restarts the scan. */
#define CMDLINK_DUP_US  1000000u
static char     s_last[CMDLINK_LINE_MAX];
static uint32_t s_last_us = 0;
static uint32_t s_dups = 0;

/* ---- RX ring filled from the EUSART RX interrupt ----
 * 2026-11-03: the EUSART RX FIFO is 16 bytes. While the I/Q stream pump
 * runs, the main loop polls less often than the 1.4 ms in which 16 bytes
 * arrive at 115200 baud, so a 17-byte line (W433775,1000,1016 + CR) lost
 * bytes and the scan parser dropped it - sometimes. The interrupt only
 * moves bytes into this ring; the line parsing stays in cmdlink_poll(),
 * in the main loop, so nothing about the stream changes from an ISR. */
#define CMDLINK_RING 128
static volatile uint8_t  s_ring[CMDLINK_RING];
static volatile uint16_t s_ring_w = 0;      /* written by the ISR */
static uint16_t          s_ring_r = 0;      /* read by cmdlink_poll */
static volatile uint32_t s_ring_ovf = 0;

void EUSART1_RX_IRQHandler(void)
{
  while (CMDLINK_EUSART->STATUS & EUSART_STATUS_RXFL) {
    /* A glitch on the one-wire link (SPI and TFT traffic next to it) looks
     * like a start bit and yields a garbage byte with the FERR flag. The
     * series-2 EUSART has no per-byte error bits in RXDATA, but the RXFL
     * interrupt services bytes one at a time, so the flag belongs to the
     * byte being read with good probability: drop it, otherwise it lands
     * inside the command line and the strict scan parser rejects the whole
     * line. (2026-11-03: the ~25 % loss of 16-char lines turned out to be
     * the FIFO overflowing on the 17th byte, see the priority note in
     * cmdlink_init(); this filter stays as a second line of defence.) */
    uint32_t err = CMDLINK_EUSART->IF & (EUSART_IF_FERR | EUSART_IF_PERR);
    uint8_t ch = (uint8_t)(CMDLINK_EUSART->RXDATA & 0xFFu);
    if (err) { CMDLINK_EUSART->IF_CLR = err; ++s_errors; continue; }
    uint16_t next = (uint16_t)((s_ring_w + 1u) % CMDLINK_RING);
    if (next == s_ring_r) { ++s_ring_ovf; continue; }   /* ring full: drop */
    s_ring[s_ring_w] = ch;
    s_ring_w = next;
  }
  CMDLINK_EUSART->IF_CLR = EUSART_IF_RXFL;
  /* Hardware FIFO overflow (the ISR came too late): counted with the ring
   * overflows, it shows in cmdlink_stats() as an error. */
  if (CMDLINK_EUSART->IF & EUSART_IF_RXOF) {
    CMDLINK_EUSART->IF_CLR = EUSART_IF_RXOF;
    ++s_ring_ovf;
  }
}

void cmdlink_init(cmdlink_line_fn on_line)
{
  s_on_line = on_line;
  s_len = 0;

  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(CMDLINK_CLOCK, true);

  /* Input with pull-up: if the ESP has not started yet (or the wire is
   * not connected), the line must be HIGH — otherwise the floating input
   * would generate endless "start bits" and fill the buffer with garbage. */
  GPIO_PinModeSet(CMDLINK_RX_PORT, CMDLINK_RX_PIN, gpioModeInputPull, 1);

  /* The emlib EUSART_UART_INIT_DEFAULT_HF macro does not list the
   * advancedSettings field, so the project's -Wextra emits a warning. The
   * field is thus zero- (NULL-) initialised, which is exactly what we want
   * — no advanced settings are needed. The warning is silenced here, and
   * ONLY here, so that a new file does not start out noisy. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
#pragma GCC diagnostic pop

  init.baudrate         = CMDLINK_BAUD;
  init.enable           = eusartEnableRx;  /* TX is not needed */
  init.advancedSettings = NULL;            /* explicit, to leave no guesswork */
  EUSART_UartInitHf(CMDLINK_EUSART, &init);

  GPIO->EUSARTROUTE[CMDLINK_ROUTE_IDX].RXROUTE =
      ((uint32_t)CMDLINK_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
    | ((uint32_t)CMDLINK_RX_PIN  << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[CMDLINK_ROUTE_IDX].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN;

  /* RX interrupt -> ring (see above). HIGHEST priority (2026-11-03): with
   * priority 7 the 16-char scan line "W433775,1000,512" lost its 17th byte,
   * the CR, in ~30 % of the cases while the stream ran - read back over SWD
   * the line buffer held "W...512W...512", two lines glued together. The
   * EUSART RX FIFO is 16 deep, so the ISR was late by more than 16 byte
   * times (1.4 ms at 115200): the stream's RAIL/LDMA handlers run that long
   * at R=8. A one-character line ('i') never overflows, which is why only
   * the long lines were lost. The handler is a few instructions and touches
   * nothing but its own ring, so it is safe to preempt everything. */
  s_ring_w = s_ring_r = 0;
  CMDLINK_EUSART->IF_CLR = EUSART_IF_RXFL;
  EUSART_IntEnable(CMDLINK_EUSART, EUSART_IEN_RXFL);
  NVIC_SetPriority(EUSART1_RX_IRQn, 0);
  NVIC_ClearPendingIRQ(EUSART1_RX_IRQn);
  NVIC_EnableIRQ(EUSART1_RX_IRQn);
}

bool cmdlink_poll(void)
{
  bool got_line = false;

  /* Read only what is ALREADY in the ring (filled by the RX interrupt).
   * This bounds the call time and does not slow down the stream pump. The
   * guard ensures that even a stuck state cannot turn this into an
   * infinite loop. */
  uint8_t guard = CMDLINK_RING;
  while (s_ring_r != s_ring_w && guard--) {
    char ch = (char)s_ring[s_ring_r];
    s_ring_r = (uint16_t)((s_ring_r + 1u) % CMDLINK_RING);

    if (ch == '\r' || ch == '\n') {
      if (s_len > 0) {
        s_buf[s_len] = '\0';
        s_len = 0;
        {
          uint32_t now = (uint32_t)RAIL_GetTime();
          if (s_last[0] && strcmp(s_buf, s_last) == 0 && (now - s_last_us) < CMDLINK_DUP_US) {
            ++s_dups;                 /* the ESP's repeat of a line already executed */
            s_last_us = now;
            continue;
          }
          strncpy(s_last, s_buf, sizeof s_last - 1); s_last[sizeof s_last - 1] = '\0';
          s_last_us = now;
        }
        ++s_lines;
        if (s_on_line) s_on_line(s_buf);
        got_line = true;
        /* ONE line per call. Tuning may stop and restart the stream — we
         * do not want that twice in a row within the same loop iteration. */
        break;
      }
    } else if ((uint8_t)ch < 0x20u || (uint8_t)ch > 0x7Eu) {
      ++s_errors;           /* not printable: a line glitch, never a command byte */
    } else if (s_len < sizeof(s_buf) - 1) {
      s_buf[s_len++] = ch;
    } else {
      s_len = 0;            /* overflow -> drop the whole line */
      ++s_errors;
    }
  }

  return got_line;
}

void cmdlink_stats(uint32_t *lines, uint32_t *errors)
{
  if (lines)  *lines  = s_lines;
  if (errors) *errors = s_errors + s_ring_ovf;
}

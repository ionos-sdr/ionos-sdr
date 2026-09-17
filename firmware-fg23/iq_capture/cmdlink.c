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
#include <stddef.h>      /* NULL — Simplicity does not pull it in by itself */

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
}

bool cmdlink_poll(void)
{
  bool got_line = false;

  /* Read only what is ALREADY in the FIFO. This bounds the call time and
   * does not slow down the stream pump. The guard ensures that even a
   * stuck state cannot turn this into an infinite loop. */
  uint8_t guard = 64;
  while ((CMDLINK_EUSART->STATUS & EUSART_STATUS_RXFL) && guard--) {

    if (CMDLINK_EUSART->IF & (EUSART_IF_FERR | EUSART_IF_PERR)) {
      CMDLINK_EUSART->IF_CLR = EUSART_IF_FERR | EUSART_IF_PERR;
      ++s_errors;
    }

    char ch = (char)(CMDLINK_EUSART->RXDATA & 0xFFu);

    if (ch == '\r' || ch == '\n') {
      if (s_len > 0) {
        s_buf[s_len] = '\0';
        s_len = 0;
        ++s_lines;
        if (s_on_line) s_on_line(s_buf);
        got_line = true;
        /* ONE line per call. Tuning may stop and restart the stream — we
         * do not want that twice in a row within the same loop iteration. */
        break;
      }
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
  if (errors) *errors = s_errors;
}

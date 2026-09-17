/* SPDX-License-Identifier: MIT
 *
 * cmdlink.c — masodik parancsbemenet a FG23-on (ESP32 -> FG23)
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * EUSART1, csak RX, 115200 8N1, a PA06 = EXP 11 labon.
 *
 * ================== MIERT EUSART1 ==================
 *   EUSART0 : a VCOM (a terminal, amin a J-Linken keresztul beszelsz)
 *   USART0  : az I/Q kimenet SPI-je (iq_stream.c)
 *   EUSART1 : szabad — ez lesz a parancsbemenet
 *
 * ================== MIERT NINCS MEGSZAKITAS ==================
 * A parancsok ritkak (masodpercenkent legfeljebb par darab, amikor
 * tekered a frekvenciat a telefonon), es a fo ciklus ugyis fut. Egy
 * megszakitas itt csak versenyhelyzetet hozna a stream ISR-jevel, amiert
 * cserebe semmit nem kapnank. A EUSART FIFO-ja elnyeli a jitteret.
 *
 * ================== HA NEM MEGY ==================
 *   - meresd meg a PA06-ot: nyugalomban MAGASNAK kell lennie (az UART
 *     alapszintje). Ha lebeg, nincs bekotve az ESP TX-e.
 *   - a ket panel GND-jenek OSSZE KELL lennie kotve. Mar ossze van, a
 *     SPI miatt — de ha atkotod, ne felejtsd.
 *   - a cmdlink_stats() error szamlaloja keretezesi hibat szamol: ha az
 *     no, a baud nem stimmel.
 */

#include "cmdlink.h"
#include "em_eusart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include <stddef.h>      /* NULL — a Simplicity nem huzza be magatol */

/* ---- lab es baud ---- */
#define CMDLINK_EUSART      EUSART1
#define CMDLINK_ROUTE_IDX   1              /* GPIO->EUSARTROUTE[1] */
#define CMDLINK_CLOCK       cmuClock_EUSART1
#define CMDLINK_RX_PORT     gpioPortA
#define CMDLINK_RX_PIN      6              /* PA06 = EXP 11 */
#define CMDLINK_BAUD        115200u

#define CMDLINK_LINE_MAX    48   /* 2026-08-15: a W-scan parancs (W<kHz>,<span>,<nbin>,<floor>,<range>) ~30 kar. */

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

  /* Bemenet, felhuzassal: ha az ESP meg nem indult el (vagy nincs bekotve
   * a drot), a vonal MAGAS legyen — kulonben a lebego bemenet vegtelen
   * "start bit"-eket generalna es teleszemetelne a puffert. */
  GPIO_PinModeSet(CMDLINK_RX_PORT, CMDLINK_RX_PIN, gpioModeInputPull, 1);

  /* Az emlib EUSART_UART_INIT_DEFAULT_HF makroja nem sorolja fel az
   * advancedSettings mezot, ezert a projekt -Wextra-javal figyelmeztetest
   * ad. A mezo igy nullara (NULL-ra) inicializalodik, ami pontosan az, amit
   * akarunk — nincs szuksegunk advanced beallitasokra. A figyelmeztetest
   * itt, es CSAK itt nemitjuk el, hogy egy uj fajl ne kezdjen zajjal. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
  EUSART_UartInit_TypeDef init = EUSART_UART_INIT_DEFAULT_HF;
#pragma GCC diagnostic pop

  init.baudrate         = CMDLINK_BAUD;
  init.enable           = eusartEnableRx;  /* TX-re nincs szuksegunk */
  init.advancedSettings = NULL;            /* kimondva, ne kelljen talalgatni */
  EUSART_UartInitHf(CMDLINK_EUSART, &init);

  GPIO->EUSARTROUTE[CMDLINK_ROUTE_IDX].RXROUTE =
      ((uint32_t)CMDLINK_RX_PORT << _GPIO_EUSART_RXROUTE_PORT_SHIFT)
    | ((uint32_t)CMDLINK_RX_PIN  << _GPIO_EUSART_RXROUTE_PIN_SHIFT);
  GPIO->EUSARTROUTE[CMDLINK_ROUTE_IDX].ROUTEEN = GPIO_EUSART_ROUTEEN_RXPEN;
}

bool cmdlink_poll(void)
{
  bool got_line = false;

  /* Csak azt olvassuk ki, ami MAR a FIFO-ban van. Igy a hivas ideje
   * korlatos, es a stream pumpjat nem lassitja meg. A guard azert kell,
   * hogy egy beragadt allapot se tudja vegtelen ciklusba vinni. */
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
        /* Egy hivasban EGY sort dolgozunk fel. A hangolas leallithatja es
         * ujrainidithatja a streamet — nem akarunk ilyet ketszer egymas
         * utan, ugyanabban a ciklusban. */
        break;
      }
    } else if (s_len < sizeof(s_buf) - 1) {
      s_buf[s_len++] = ch;
    } else {
      s_len = 0;            /* tulcsordulas -> eldobjuk az egesz sort */
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

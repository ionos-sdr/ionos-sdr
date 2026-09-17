/*
 * tone.c  --  Sidetone via TIMER PWM on a GPIO (EFR32FG23 / emlib).
 *  Concept by: N7HPR   Design by: HA7DCD
 *
 *  The timer runs continuously; keying is done by moving the compare value
 *  between 0 (line low, silent) and TOP/2 (50% square wave). Glitch-free,
 *  no clicks from route toggling.
 *
 *  NOTE: TONE_TIMER is TIMER0 -> GPIO TIMERROUTE index 0 below. If you move
 *  the sidetone to another TIMER, update TONE_ROUTE_IDX to match.
 */
#include "tone.h"
#include "bsp_pins.h"

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"

#define TONE_ROUTE_IDX   0            /* TIMER0 */
#define TONE_PRESCALE    timerPrescale2

static uint32_t s_top = 1;
static bool     s_on  = false;

static uint32_t compute_top(uint16_t hz)
{
    uint32_t f = CMU_ClockFreqGet(TONE_TIMER_CLOCK) / 2u;   /* prescale 2 */
    uint32_t top = (hz ? f / hz : 0xFFFFu);
    if (top < 2)      top = 2;
    if (top > 0xFFFFu) top = 0xFFFFu;                       /* 16-bit TIMER */
    return top;
}

void tone_init(void)
{
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(TONE_TIMER_CLOCK, true);

    GPIO_PinModeSet(TONE_PORT, TONE_PIN, gpioModePushPull, 0);

    TIMER_InitCC_TypeDef cc = TIMER_INITCC_DEFAULT;
    cc.mode = timerCCModePWM;
    TIMER_InitCC(TONE_TIMER, TONE_CC_CHANNEL, &cc);

    GPIO->TIMERROUTE[TONE_ROUTE_IDX].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN;
    GPIO->TIMERROUTE[TONE_ROUTE_IDX].CC0ROUTE =
          ((uint32_t)TONE_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
        | ((uint32_t)TONE_PIN  << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

    s_top = compute_top(TONE_FREQ_HZ);
    TIMER_TopSet(TONE_TIMER, s_top);
    TIMER_CompareSet(TONE_TIMER, TONE_CC_CHANNEL, 0);       /* silent */

    TIMER_Init_TypeDef ti = TIMER_INIT_DEFAULT;
    ti.prescale = TONE_PRESCALE;
    ti.enable   = true;
    TIMER_Init(TONE_TIMER, &ti);
}

void tone_on(void)
{
    s_on = true;
    TIMER_CompareSet(TONE_TIMER, TONE_CC_CHANNEL, s_top / 2u);
}

void tone_off(void)
{
    s_on = false;
    TIMER_CompareSet(TONE_TIMER, TONE_CC_CHANNEL, 0);
}

void tone_set_freq(uint16_t hz)
{
    s_top = compute_top(hz);
    TIMER_TopSet(TONE_TIMER, s_top);
    TIMER_CompareSet(TONE_TIMER, TONE_CC_CHANNEL, s_on ? s_top / 2u : 0u);
}

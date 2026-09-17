/*
 * bsp_pins.h  --  Board pin configuration for the FG23 CW Trainer
 *
 *  CW Trainer for EFR32FG23 on the Silicon Labs WSTK
 *  Concept by: N7HPR   Design by: HA7DCD
 *
 *  All wiring lives here so you can move pins without hunting through code.
 *
 *  OLED (SSD1306 128x64, I2C) on the WSTK EXP header:
 *      EXP  1  GND        -> OLED GND
 *      EXP  2  VMCU 3V3    -> OLED VCC
 *      EXP 15  PC5  SCL    -> OLED SCL
 *      EXP 16  PC7  SDA    -> OLED SDA
 *  (Software I2C, open-drain. The module's on-board pull-ups are enough;
 *   the board's sensor-I2C pull-ups sit on the same lines too.)
 *
 *  Sidetone: PWM square wave on one free EXP GPIO -> buzzer / small speaker.
 *      EXP  7  PA5        -> speaker/buzzer (+), other leg to GND
 *  (Drive a passive piezo or a small speaker through a ~100 ohm resistor,
 *   or a buzzer module. Louder: add a transistor. See README.)
 */
#ifndef BSP_PINS_H
#define BSP_PINS_H

#include "em_gpio.h"

/* ---- OLED software-I2C pins ---- */
#define OLED_SCL_PORT     gpioPortC
#define OLED_SCL_PIN      5
#define OLED_SDA_PORT     gpioPortC
#define OLED_SDA_PIN      7
#define OLED_I2C_ADDR     0x3C          /* 7-bit; some modules are 0x3D */

/* Crude bit-bang half-period. Bigger = slower & safer. ~a few us is fine. */
#define OLED_I2C_DELAY_LOOPS   12u

/* ---- Sidetone (TIMER PWM) output pin ---- */
#define TONE_PORT         gpioPortA
#define TONE_PIN          5
#define TONE_TIMER        TIMER0
#define TONE_TIMER_CLOCK  cmuClock_TIMER0
#define TONE_CC_CHANNEL   0

/* Sidetone pitch in Hz (classic CW note). */
#define TONE_FREQ_HZ      600u

#endif /* BSP_PINS_H */

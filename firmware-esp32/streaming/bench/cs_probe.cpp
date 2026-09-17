/* SPDX-License-Identifier: MIT
 *
 * cs_probe.ino — ESP32-S3: does CS reach GPIO4?
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * WHY: 2026-08-01, early morning. The FG23 side is proven in every respect —
 * 48.8 blocks/s, CS low for 2147 us (theoretical 2080), the CS pad high
 * 89.5% of the time and ZERO deviation from the driven level over 3.1
 * million samples. Yet the ESP SPI slave receives nothing.
 *
 * This sketch BYPASSES the whole SPI peripheral: it only counts the CS
 * falling edges with a plain GPIO interrupt, and samples the other two
 * lines alongside.
 *
 * EXPECTED RESULT with 'i32' running:
 *     CS falling edges: ~49 / s        (12500 sps / 256 samples = 48.8)
 *     CS high:          ~89 %
 *
 * INTERPRETATION:
 *   - ~49 edges/s     -> the signal DOES arrive, the fault is in the SPI
 *                        slave configuration (mode, size, queueing)
 *   - 0 edges/s       -> CS does not reach the pin: wire or pin
 *   - very many edges -> noise / floating input
 *
 * HISTORY: CS was originally on GPIO4, and there it FLOATED — 21000 edges/s
 * and 60% duty, while SCLK (GPIO5) and MOSI (GPIO6) gave a clean 51% in the
 * same measurement. That is why it was moved to GPIO7.
 */

#include <Arduino.h>

#define PIN_CS    7      /* FG23 PA07 / EXP 13 */
#define PIN_SCLK  5      /* FG23 PC05 / EXP 15 */
#define PIN_MOSI  6      /* FG23 PC00 / EXP 10 */

static volatile uint32_t cs_falls = 0;
static volatile uint32_t cs_rises = 0;

static void IRAM_ATTR on_cs_fall(void) { cs_falls++; }
static void IRAM_ATTR on_cs_rise(void) { cs_rises++; }

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== CS probe: does the frame signal reach GPIO4? ===");
  Serial.printf("CS=GPIO%d  SCLK=GPIO%d  MOSI=GPIO%d\n",
                PIN_CS, PIN_SCLK, PIN_MOSI);
  Serial.println("expected with 'i32' running: ~49 falling edges/s, CS ~89% high");

  /* NO pull-up: the FG23 drives push-pull. If it floats, that must be
   * visible (very many irregular edges). */
  pinMode(PIN_CS,   INPUT);
  pinMode(PIN_SCLK, INPUT);
  pinMode(PIN_MOSI, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_CS), on_cs_fall, FALLING);
  /* One handler per pin, so the rising edge is not attached separately —
   * the duty cycle is measured by sampling. */
  (void)on_cs_rise;
}

void loop()
{
  static uint32_t t0 = 0;
  static uint32_t n_samp = 0, n_cs_hi = 0, n_clk_hi = 0, n_mosi_hi = 0;

  /* Tight sampling: the duty cycle comes from this. */
  n_samp++;
  if (digitalRead(PIN_CS))   n_cs_hi++;
  if (digitalRead(PIN_SCLK)) n_clk_hi++;
  if (digitalRead(PIN_MOSI)) n_mosi_hi++;

  uint32_t now = millis();
  if (t0 == 0) t0 = now;
  if (now - t0 >= 2000) {
    noInterrupts();
    uint32_t f = cs_falls; cs_falls = 0;
    interrupts();

    float dt = (now - t0) / 1000.0f;
    Serial.printf("CS falling edges: %5.1f /s   |   CS high %4.1f%%   "
                  "SCLK high %4.1f%%   MOSI high %4.1f%%   (%lu samples)",
                  f / dt,
                  100.0f * n_cs_hi   / n_samp,
                  100.0f * n_clk_hi  / n_samp,
                  100.0f * n_mosi_hi / n_samp,
                  (unsigned long)n_samp);

    if (f == 0)                 Serial.print("   <-- NO CS: wire or pin");
    else if (f / dt > 500.0f)   Serial.print("   <-- too many edges: noise / floating");
    else if (f / dt > 40.0f && f / dt < 60.0f)
                                Serial.print("   <-- OK, the signal arrives");
    Serial.println();

    n_samp = n_cs_hi = n_clk_hi = n_mosi_hi = 0;
    t0 = now;
  }
}
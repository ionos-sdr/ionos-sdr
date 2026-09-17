/* SPDX-License-Identifier: MIT
 *
 * cs_probe.ino — ESP32-S3: eljut-e a CS a GPIO4-ig?
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * MIERT: 2026-08-01 hajnal. A FG23 oldal minden szempontbol bizonyitott —
 * 48.8 blokk/s, CS lent 2147 us (elmeleti 2080), a CS pad 89.5%-ban magas
 * es 3.1 millio mintabol NULLA eltéres a kiirt szinttol. Az ESP SPI slave
 * megis nem vesz semmit.
 *
 * Ez a vazlat KIHAGYJA az egesz SPI perifériat: csak megszamolja a CS
 * lefuto eleit egy sima GPIO-megszakitassal, es mellette mintavetelezi a
 * masik ket vonalat.
 *
 * VART EREDMENY futo 'i32' mellett:
 *     CS lefuto el: ~49 / s        (12500 sps / 256 minta = 48.8)
 *     CS magas:     ~89 %
 *
 * ERTELMEZES:
 *   - ~49 el/s        -> a jel EPEN megerkezik, a hiba az SPI slave
 *                        konfiguraciojaban van (mod, meret, sorbaallitas)
 *   - 0 el/s          -> a CS nem er el a labig: drot vagy lab
 *   - nagyon sok el   -> zaj / lebego bemenet
 *
 * ELOZMENY: a CS eredetileg a GPIO4-en volt, es ott LEBEGETT — 21000 el/s
 * es 60% kitoltes, mikozben a SCLK (GPIO5) es a MOSI (GPIO6) ugyanabban a
 * mereseben hibatlan 51%-ot adott. Ezert kerult at a GPIO7-re.
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
  Serial.println("\n=== CS-proba: eljut-e a keretjel a GPIO4-ig? ===");
  Serial.printf("CS=GPIO%d  SCLK=GPIO%d  MOSI=GPIO%d\n",
                PIN_CS, PIN_SCLK, PIN_MOSI);
  Serial.println("varhato futo 'i32' mellett: ~49 lefuto el/s, CS ~89% magas");

  /* Felhuzas NELKUL: a FG23 push-pull hajtja. Ha lebegne, azt latni
   * akarjuk (nagyon sok, szabalytalan el). */
  pinMode(PIN_CS,   INPUT);
  pinMode(PIN_SCLK, INPUT);
  pinMode(PIN_MOSI, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_CS), on_cs_fall, FALLING);
  /* Egy labra egy handler jut, ezert a felfuto elt nem kotjuk kulon —
   * a kitoltest mintavetelezessel merjuk. */
  (void)on_cs_rise;
}

void loop()
{
  static uint32_t t0 = 0;
  static uint32_t n_samp = 0, n_cs_hi = 0, n_clk_hi = 0, n_mosi_hi = 0;

  /* Szoros mintavetelezes: a kitoltest ebbol kapjuk. */
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
    Serial.printf("CS lefuto el: %5.1f /s   |   CS magas %4.1f%%   "
                  "SCLK magas %4.1f%%   MOSI magas %4.1f%%   (%lu minta)",
                  f / dt,
                  100.0f * n_cs_hi   / n_samp,
                  100.0f * n_clk_hi  / n_samp,
                  100.0f * n_mosi_hi / n_samp,
                  (unsigned long)n_samp);

    if (f == 0)                 Serial.print("   <-- NINCS CS: drot vagy lab");
    else if (f / dt > 500.0f)   Serial.print("   <-- tul sok el: zaj / lebeg");
    else if (f / dt > 40.0f && f / dt < 60.0f)
                                Serial.print("   <-- STIMMEL, a jel megerkezik");
    Serial.println();

    n_samp = n_cs_hi = n_clk_hi = n_mosi_hi = 0;
    t0 = now;
  }
}

/* rgb_load.h - a WS2812 szinkeverese a CPU-terheles utemere
 *
 * HA7DCD / Kolibri.  A panelen levo RGB LED lassan sodrodik a szinkoron,
 * ha a rendszer unatkozik, es annal gyorsabban pörög, minel jobban meg van
 * terhelve.  Egy pillantas a panelre, es tudod, mi van - kijelzohely es
 * figyelem nelkul.
 *
 * MERES: FreeRTOS idle-hook magonkent.  Az idle task minden korben egyet
 * szamol; ha nincs szabad ido, nem szamol.  A terheles = 1 - szamlalo/max,
 * ahol a max onkalibralodik (felfele azonnal, lefele lassan cseng).
 * Nem "igazi" profiler, de arra, hogy a LED jol mutassa a nyomast, eleg -
 * es nem kell hozza CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS.
 *
 * KOLTSEG: ~30 us / frissites (RMT viszi), 50 Hz-en 0,15% CPU.
 * Sajat task, 1-es prioritas, 0-as mag.
 *
 * HASZNALAT:
 *   setup():  rgb_load_init(48);          // 38 is elofordul klonokon
 *   parancs:  if (rgb_load_cmd(line)) return;
 *
 * Ha sajat terhelesmerod van (SPI-sor melyseg, eagain-rata, RXRING vizjel):
 *   rgb_load_override(0.75f);   // 0..1 ; -1 = vissza az automatikara
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void  rgb_load_init(int pin);
void  rgb_load_stop(void);
bool  rgb_load_running(void);

/* 0..1 kozotti aktualis terheles (a ket mag maximuma) */
float rgb_load_get(void);
void  rgb_load_get_cores(float *core0, float *core1);

/* Kulso terhelesforras. -1.0f = vissza az automatikus meresre. */
void  rgb_load_override(float load_0_1);

/* ---- parancsok (true = ez a modul kezelte a sort) ----------------------
 *   L                statusz
 *   L0 / L1          ki / be
 *   Ltest            vegigfut a szinkoron 3 mp alatt (bekotes-ellenorzes)
 *   Lset k=v ...     konfig:
 *       pin=48       a WS2812 laba
 *       fenyero=12   csucs-fenyero 1..255 (12 = nagyon halvany)
 *       lassu=90     korido masodpercben ures rendszernel
 *       gyors=2      korido masodpercben 100% terhelesnel
 *       gorbe=50     leképzes gorbulete %-ban (50 = gyokos, 100 = linearis)
 *       dither=1     idobeli ditherelés (halvany fenyeronel kotelezo)
 *       gamma=1      gamma-korrekcio
 *       telitettseg=100
 *       hz=50        frissitesi rata
 */
bool rgb_load_cmd(const char *line);

#ifdef __cplusplus
}
#endif

/* tft.h - ILI9341 allapotkijelzo + hardveres scroll waterfall
 *
 * Busz: SPI2 (IO_MUX 9-14). Az FG23 slave a SPI3-on van -> nincs utkozes.
 * TE: NINCS bekotve (a 47 az RGB LED-e) -> szoftveres TE a 0x45-tel.
 *
 * Elrendezes:
 *   0..49    fix fejlec (statusz szoveg, 2 mp-enkent)
 *   50..319  hardveresen gorgetett waterfall (0x33 / 0x37)
 *
 * A rajzolas SAJAT taskban fut (0-as mag, prio 1), tehat a fo ciklust es az
 * FG23 SPI-kiszolgalasat nem blokkolja. Merve a benchben: 135 us/sor CPU.
 */
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float       blokk_s;
    uint32_t    sps;
    float       csucs_dbfs;
    int         rssi_dbm;
    uint32_t    ram_k;
    uint32_t    ram_min_k;
    uint8_t     terheles_pct;
    uint32_t    lost;
    uint32_t    freq_khz;
    const char *ip;
    const char *ssid;
} tft_stat_t;

void tft_init(void);
void tft_show(const tft_stat_t *s);      /* fejlec, 2 mp-enkent */
bool tft_ok(void);

/* Egy scan-sor a waterfallba. Ugyanonnan hivando, ahonnan az
 * oled_spec.push(). Nem blokkol: atmasol egy sorpufferbe es ertesiti a
 * kijelzo-taskot. Ha a task meg az elozovel dolgozik, a sor eldobodik
 * (a kijelzo sosem lassithatja a radiot). */
void tft_push_specline(const uint8_t *bins, uint16_t nbin);

/* A scan savja - a fejlec frekvenciaskalajahoz. Eleg valtozaskor hivni. */
void tft_set_span(uint32_t center_hz, uint32_t span_hz);

/* Automatikus dinamika-illesztes a waterfallon. A SCAN-adatnak kell (ott
 * abszolut dBm-skala jon, ismeretlen padloval), az FFT-nek NEM: az mar
 * normalizalva erkezik, es a masodik nyujtas a ZAJ SZORASAT feszitene ki
 * az egesz palettara. Ket skalazo egymas utan = telitett, kasas kep. */
void tft_set_autoscale(bool on);

/* Eldobott sorok szama - diagnosztika */
uint32_t tft_dropped(void);

#ifdef __cplusplus
}
#endif

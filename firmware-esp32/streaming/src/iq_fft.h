/* iq_fft.h - keskenysavu I/Q -> waterfall-sor az LCD-re
 *
 * A SPECLINE (scan) a FG23 szeles panoramaja. Ez a modul a MASIK forrast
 * adja: a folyamatos 50 ksps-os I/Q folyambol szamol FFT-t, es ugyanabba a
 * tft_push_specline() bemenetbe tolja. Igy a panel akkor is mutat valamit,
 * amikor nem fut scan - a hangolt +-25 kHz-et.
 *
 * Koltseg: 256 pontos komplex FFT ~120 us az S3-on (float, radix-2).
 * Alapbol minden 12. blokkbol szamolunk -> ~16 sor/s, ~0,2% CPU.
 * A hivas az SPI-fogadobol jon, ezert MINDEN itteni munka szamit: ezert van
 * a ritkitas, es ezert nem masolunk feleslegesen.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Egy nyers I/Q blokk payloadja (256 minta, int16 Q-first). */
void iq_fft_push_block(const uint8_t *payload);

/* A hangolas es a mintavetel - a frekvenciaskalahoz. */
void iq_fft_set_tuning(uint32_t center_hz, uint32_t sps);

/* be/ki. Ha a scan fut, azt erdemes elonyben reszesiteni. */
void iq_fft_enable(bool on);
bool iq_fft_enabled(void);

/* Hany blokkonkent szamoljunk (1..64). Nagyobb = ritkabb, kevesebb CPU. */
void iq_fft_set_decim(uint8_t n);

/* ---- parancsok (true = ez a modul kezelte a sort) ----------------------
 *   F                 statusz
 *   Fset k=v ...      n=<256|512|1024>  FFT hossz (1024: 48,8 Hz/bin)
 *                     win=<0|1>   0 = Hann, 1 = Nuttall (-92 dBc szivargas)
 *                     lift=<dB>   a zaj ennyivel a padlo folott kezd szint
 *                                 kapni. NEGATIV = vilagosabb zaj.
 *                     range=<dB>  ennyi dB feszul a teljes palettara
 *                     alpha=<%>   EMA suly (kisebb = simabb, lomhabb)
 *                     dec=<n>     hany adagonkent rajzoljunk sort
 *                     inv=<0|1>   spektrum tukrozes (I/Q konvencio)
 */
bool iq_fft_cmd(const char *line);

#ifdef __cplusplus
}
#endif

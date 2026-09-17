/* iq_fft.h - narrowband I/Q -> waterfall row for the LCD
 *
 * The SPECLINE (scan) is the FG23's wide panorama. This module provides the
 * OTHER source: it computes an FFT from the continuous 50 ksps I/Q stream
 * and feeds it into the same tft_push_specline() input. Thus the panel shows
 * something even when no scan is running - the tuned +-25 kHz.
 *
 * Cost: a 256-point complex FFT is ~120 us on the S3 (float, radix-2).
 * By default every 12th block is processed -> ~16 rows/s, ~0.2% CPU.
 * The call comes from the SPI receiver, so ALL work here counts: hence the
 * decimation, and hence no unnecessary copies.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Payload of one raw I/Q block (256 samples, int16 Q-first). */
void iq_fft_push_block(const uint8_t *payload);

/* Tuning and sample rate - for the frequency scale. */
void iq_fft_set_tuning(uint32_t center_hz, uint32_t sps);

/* on/off. While a scan is running, the scan should take precedence. */
void iq_fft_enable(bool on);
bool iq_fft_enabled(void);

/* Compute every n-th block (1..64). Larger = sparser, less CPU. */
void iq_fft_set_decim(uint8_t n);

/* ---- commands (true = this module handled the line) --------------------
 *   F                 status
 *   Fset k=v ...      n=<256|512|1024>  FFT length (1024: 48.8 Hz/bin)
 *                     win=<0|1>   0 = Hann, 1 = Nuttall (-92 dBc leakage)
 *                     lift=<dB>   noise starts getting a level this many dB
 *                                 above the floor. NEGATIVE = brighter noise.
 *                     range=<dB>  this many dB spans the whole palette
 *                     alpha=<%>   EMA weight (smaller = smoother, slower)
 *                     dec=<n>     draw a row every n blocks
 *                     inv=<0|1>   spectrum mirroring (I/Q convention)
 */
bool iq_fft_cmd(const char *line);

#ifdef __cplusplus
}
#endif

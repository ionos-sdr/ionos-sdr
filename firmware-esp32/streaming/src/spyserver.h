/* SPDX-License-Identifier: MIT
 *
 * spyserver.h — SpyServer-kompatibilis szerver az ESP32-S3-on
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================== MIERT EZ, ES NEM RTL_TCP ==================
 *
 * Az rtl_tcp ket dologban rossz nekunk:
 *   - 8 BITES. A 16 bites mintaink felet eldobjuk.
 *   - NINCS BENNE RATA-EGYEZTETES. A kliens fix RTL-ratakat felte1telez, a
 *     legkisebb 250 ksps — ezert kellett egesz szorzoval felmintavetelezni,
 *     amitol interpolacios kepek lettek a spektrumban.
 *
 * A SpyServer mindkettot megoldja:
 *   - MSG_TYPE_INT16_IQ: NATIV 16 bit.
 *   - A DeviceInfo-ban az ESZKOZ mondja meg a MaximumSampleRate-et es a
 *     decimacios fokozatok szamat, a kliens ebbol epiti a ratalistat. Ha
 *     50000-et hirdetunk, az SDR++-ban 50 kHz lesz. Nincs felmintavetelezes.
 *
 * Nem SDR++-specifikus: SDR#, SDRangel es a tobbi kliens is beszeli.
 *
 * ================== ALLAPOT: NINCS HARDVEREN LEMERVE ==================
 *
 * Ez specifikacio alapjan keszult, tesztelesi lehetoseg nelkul. A regi
 * rtl_tcp ag ERINTETLENUL megmarad az 1234-en — ha ez nem indul be elsore,
 * ott a mukodo ut.
 *
 * ================== BAJTSORREND ==================
 *
 * A protokoll a strukturakat NYERSEN kuldi, little-endianban. Az ESP32 is
 * little-endian, tehat a structok kozvetlenul kiirhatok. Minden mezo
 * uint32, ezert kitoltes (padding) sincs.
 */

#ifndef SPYSERVER_H
#define SPYSERVER_H

#include <stdint.h>
#include <stdbool.h>
#include "specline.h"

/* A kliens hangolast kert. A main.cpp koti ossze a cmdlinkkel. */
typedef void (*spy_tune_fn)(uint32_t hz);

/* A kliens scan-modot ker / leallit / atparameterez (SETTING_FFT_* +
 * STREAMING_MODE FFT-bit). A main.cpp a cmdlinken tovabbadja a FG23-nak:
 *   want=true  -> "W<kHz>,<span_kHz>,<nbin>,<floor>,<range>"
 *   want=false -> "W0"  */
typedef void (*spy_scan_fn)(bool want, uint32_t center_hz, uint32_t span_hz,
                            uint16_t nbin, int16_t floor_dbm, uint16_t range_db);

void spy_init(uint16_t port, spy_tune_fn on_tune, spy_scan_fn on_scan);

/* Scan-mod: kell-e most sor a kliensnek; egy SPECLINE tovabbitasa. */
bool spy_scan_wanted(void);
void spy_send_specline(const specline_blk_t *line);
uint32_t spy_fft_lines(void);

/* A bemeneti I/Q rata. Ez lesz a hirdetett MaximumSampleRate. */
void spy_set_rate(uint32_t sps);

/* Az aktualis hangolas, hogy a ClientSync helyes erteket adjon. */
void spy_set_freq(uint32_t hz);

/* Egy blokknyi nyers int16 I/Q (interleaved), nsamp komplex minta. */
void spy_feed(const int16_t *iq, int nsamp);

/* A fo ciklusbol: kapcsolatkezeles, parancsok, kuldes. Nem blokkol. */
void spy_tick(uint32_t now_ms);

bool     spy_connected(void);
uint32_t spy_out_sps(void);     /* a decimacio utani, TENYLEG kuldott rata */
uint32_t spy_dropped(void);

#endif /* SPYSERVER_H */

/* SPDX-License-Identifier: MIT
 *
 * FG23 fazis-koherens I/Q capture + TX eval + folyamatos I/Q stream
 * Simplicity SDK 2025.6.3 / RAIL 2.19.x — "RAIL - SoC Empty" projektbe
 *
 * A capture-mag a geckokapula projekt dsp_driver.c-jebol szarmazik:
 *   Copyright (c) 2017-2022 Tatu Peltola (OH2EAT) — MIT licenc
 *   https://github.com/tejeez/geckokapula
 * Series 2 port es burst-dump:
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * ELHELYEZES: a SoC Empty sablonban az app_init() az app_init.c-be, az
 * app_process_action() az app_process.c-be valo — vagy az egesz mehet
 * egyetlen app.c-be, ha a sablon ures fuggvenyeit torlod.
 *
 * KOMPONENSEK (Software Components):
 *   - RAIL Utility, Initialization  (peldany: inst0; "Enable Setup of
 *     Radio Events" BE — ez adja a sl_rail_util_on_event routingot)
 *   - a sajat Radio Configurator PHY-d
 *   - IO Stream: EUSART (peldany: vcom) + IO Stream: Retarget STDIO
 *   - emlib USART (az I2S kimenethez!) + DMADRV (az LDMA-hoz)
 *
 * MERT BINARIS A DUMP ES MIERT sl_iostream_write:
 *   a retargetelt printf/stdout utvonal pufferelhet es LF-konverziot
 *   vegezhet, ami a binaris keretet elrontana — a payload ezert megy
 *   kozvetlenul sl_iostream_write-tal.
 *
 * MINTAFORMATUM (geckokapula dsp.h + a RAILtest-forenzika):
 *   struct { int16_t q, i; } — Q ELOL, little-endian ("i16le" a
 *   Python oldalon). Az iq_view_stream.py IQB1 modja pont ezt varja.
 *
 * ---------------- TX EVALUATION (Phase 3) ----------------
 * A Series 1-es geckokapula synth_set_channel helyett a hivatalos
 * Series 2 ut: RAIL_StartTxStream() CARRIER_WAVE / PN9 moddal,
 * finomhangolas RAIL_SetFreqOffset()-tel (egyseg: synth tick, FG23-on
 * 4.649 Hz, 15 bitre korlatozva -> kb. +/-80 kHz).
 *
 * ---------------- I/Q LANC (Phase 4) ----------------
 * 'k' — rataplafon-benchmark (fs, OVR, FIFO-csucs, CPU%)
 * 'n' — zero-copy (NULL celcimu) olvasas probaja es merese
 * 'i' — folyamatos decimalt stream (CIC + DC-blokk), I2S-en az ESP32-S3-nak
 *
 * ---------------- KIMENET: SPI (NEM I2S) ----------------
 * A 2026-07-31-i meres szerint a FG23 USART-janak CS-e nem tud valodi
 * I2S word selectet adni (33.6%-os kitoltes az 50% helyett, mind a nyolc
 * keretezesi kombinacioban). Ezert a kimenet SPI-ra kerult:
 *
 *   EXP 15 / PC05 -> SCLK   (tobb MHz)      -> ESP GPIO5
 *   EXP 10 / PC00 -> MOSI   (tobb MHz)      -> ESP GPIO6
 *   EXP  6 / PC03 -> CS     (blokkonkent!)  -> ESP GPIO4
 *
 * A blokk-fejlecben SORSZAM van, tehat a csomagvesztes kiirhato szam
 * lesz, nem sejtes.
 *
 * A 'g' lab-teszt megmaradt, es ketto a haszna: a fizikai bekotest
 * ellenorzi, ES mindharom labat PONTOSAN 50%-on billegteti, tehat
 * KITOLTES-REFERENCIA multimeteres mereshez (1.65 V @ 3.3 V logika).
 * Ez a meres fogta meg az I2S-hibat, miutan minden szoftveres nyom
 * elfogyott.
 *
 * BIZTONSAG: CW-nel a chip akar +10..+20 dBm-et ad ki. SDR bemenetre
 * SOHA kozvetlenul — minimum 30-40 dB csillapitas vagy dummy load!
 */

#include "rail.h"
#include "sl_rail_util_init.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"
#include "aprs_beacon.h"
#include "iq_bench.h"
#include "iq_stream.h"
#include "cw_morse.h"
#include "em_gpio.h"
#include "em_cmu.h"

/* ---- Masodik parancsbemenet: ESP32 GPIO4 -> FG23 PA06 / EXP 11. ----
 * Ezen at a telefon (rtl_tcp -> ESP -> ide) tud hangolni. ALAPBOL KI,
 * mert egy uj forrasfajlt (cmdlink.c) kell hozzaadni a projekthez ES egy
 * drotot behuzni. Kapcsold 1-re, ha megvan mindketto. */
#define CMDLINK_ENABLE 1   /* 2026-08-15: sarga jumper EXP11/PA06 <- GPIO4 bekotve */
#if CMDLINK_ENABLE
#include "cmdlink.h"
#endif
#include "scan.h"       /* szelessavu RSSI-scan (SPECLINE az SPI-n) */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* ---------------- konfiguracio ---------------- */

/* Q elol! (geckokapula iq_in_t) */
typedef struct {
  int16_t q;
  int16_t i;
} iq_in_t;

/* Burst hossza: 8192 komplex minta = 32 KiB RAM (a FG23B 64 KiB-jaba
 * boven belefer). 48 kHz I/Q-nal ~170 ms, 160 kHz-nel ~51 ms. */
#define CAPTURE_SAMPLES   8192u

/* Esemenyenkent olvasott komplex mintak. A geckokapula 2-t olvasott
 * (audio-szinkron miatt); nekunk 64 minta / esemeny = 256 bajt jo,
 * ritkabb IRQ. */
#define SAMPLES_PER_EVENT 64u
#define THRESHOLD_BYTES   (SAMPLES_PER_EVENT * sizeof(iq_in_t))

/* Series 2: az RX FIFO-t az app adja a RAILCb_SetupRxFifo-n keresztul. */
#define RX_FIFO_BYTES     4096u

/* A csatorna, amin a PHY a base frekvenciat adja (configtol fugg). */
#define IQ_CHANNEL        0u

/* ---- Frekvencia-kalibracio ----
 *
 * VEGLEGES: 2026-08-04, Signal Hound BB60C-vel merve (perdonto, nincs
 * I/Q-ketertelmuseg, ellentetben a korabbi HackRF-es felvetellel).
 *   TX @ 144.8 MHz (IQ_CHANNEL=0, base 144800):
 *     o0   -> 144.797780 MHz   (nyers, korrekcio nelkul, -2220 Hz)
 *     o482 -> 144.800001 MHz   (dead-on)  <-- EZ A MERT KALIBRACIO
 *   Kristalyhiba ~-15.3 ppm (LASSU kristaly), felbontas ~4.60 Hz/tick.
 *
 * A ppb-t ugy valasztjuk, hogy a corr_tick_for_khz() BOOTKOR pont 482 ticket
 * adjon 144.8 MHz-en, igy a vivo mar bekapcsolaskor a helyen van — nem kell
 * kezzel 'o482'-t utni (bolond-biztos):
 *     144800 kHz * 15476 ppb = 2240.9 Hz  -> / 4.6492 = 481.99 -> 482 tick  OK
 * A ppb frekvencia-aranyos, ezert a korrekcio a TELJES 2 m-es racson
 * (144.800 + n*25 kHz) helyes marad (a VCO-oszto a savon belul allando).
 *
 * ================== FIGYELEM: EZ MOST 2 m-RE (144.8) HANGOLT ==================
 * A synth-tick Hz/tick-je SAVFUGGO (VCO-oszto lepcso): 4.60 Hz/tick @144.8,
 * de ~11.4 Hz/tick @433. Ezert EGY ppb NEM jo mindket savra!
 *   - 2 m  (144.8):  FREQ_CORR_PPB = 15476  -> 482 tick   (MERVE 2026-08-04)
 *   - 70 cm (434):   FREQ_CORR_PPB =  5876  -> 546..549 tick (MERVE, kulon)
 * Ez a 15476 ertek 434 MHz-en ~1445 ticket adna = ~16 kHz melle! Savvaltasnal
 * ird at a ppb-t a fenti tablazatbol (es a TUNE_BASE_KHZ-t a Radio Configbol).
 *
 * Ez a BRD4265B PELDANY sajatja — masik boardon ujra kell merni. A vegleges
 * PCB-n a GPS+VCTCXO automatizalja ugyanezt. */
#define FREQ_CORR_PPB     15476     /* 2026-08-04 BB60C: 144.8 MHz -> 482 tick (-15.3 ppm) */

/* ================== AUTOSTART ==================
 * Bekapcsolas utan magatol elindul a stream, hogy a doboz onalloan
 * mukodjon (nem kell terminal, nem kell parancs). A terminal ettol meg
 * TELJESEN el: barmely billentyu leallitja a streamet, es utana minden
 * parancs elerheto, beleertve az 'i<R>'-t ujrainditasra.
 *
 *   IQ_AUTOSTART        1 = induljon magatol, 0 = maradjon a regi
 *   IQ_AUTOSTART_DECIM  a TELJES decimacio (8 tobbszorose, 8..4096)
 *                         8 ->  50 000 sps  (+-25 kHz)
 *                        16 ->  25 000 sps
 *                        32 ->  12 500 sps  <- ez a "i32"
 *                       256 ->   1 562 sps
 *   IQ_AUTOSTART_SHIFT  extra erosites kettohatvanyban (0 = nincs)
 *   IQ_AUTOSTART_DELAY_MS  ennyit varunk indulas utan, hogy az udvozlo
 *                       szoveg kimenjen a terminalra, es hogy az ESP32
 *                       felallhasson (az o bootja ~700 ms) */
/* 2026-08-01: 32 -> 8. Az rtl_tcp ut miatt. A kliensek legkisebb szabvanyos
 * rataja 250 ksps, es az ESP egesz szorzoval mintavetelez fel oda:
 *     i32 -> 12500 sps -> 20x felmintavetelezes -> a spektrum "kehes",
 *                          mert a 12,5 kHz-es sav ki van nyujtva 250-re
 *     i8  -> 50000 sps ->  5x                  -> negyszer akkora VALODI
 *                          sav, es a kepek negyszer messzebb esnek
 * Ara: i8-on mar csak az elso, MASODRENDU CIC-fokozat szur, tehat a
 * savszelek fele johet alias, es eros jelnel ~2% levagas volt. Ha WSPR/FT8-at
 * mersz es tiszta savszel kell, tedd vissza 16-ra. */
#define IQ_AUTOSTART            1
#define IQ_AUTOSTART_DECIM      8u
#define IQ_AUTOSTART_SHIFT      0u
#define IQ_AUTOSTART_DELAY_MS   800u

/* ================== HANGOLAS ('F' parancs) ==================
 * Az 'F<kHz>' abszolut frekvenciara hangol: a durva lepes a PHY csatorna-
 * rácsa, a maradekot a synth finom-offszete viszi (1 tick = 4.6492 Hz,
 * 15 bites, tehat kb. +-152 kHz-ig van hely, gyakorlatilag +-80 kHz-ig
 * hasznalhato). A ketto egyutt FOLYTONOS hangolast ad.
 *
 * ================== EZT A HAROM SZAMOT TUKROZNI KELL ==================
 * A TUNE_BASE_KHZ / TUNE_SPACING_KHZ / TUNE_MAX_CHANNEL a Radio
 * Configuratorban beallitott base frequency-t, channel spacinget es
 * csatornaszamot kell tukrozze. Ha nem egyeznek, a hangolas CSENDBEN
 * melle megy — ezert a boot-uzenet kiirja oket, es az 's' is.
 *
 * CSATORNA CSAK FELFELE VAN: a RAIL csatornaszam elojel nelkuli, tehat a
 * base ALATT csak a +-76 kHz-nyi finom offszet all rendelkezesre. A 70 cm
 * sav egeszehez ezert a base-t a sav ALJARA kell tenni.
 *
 * AJANLOTT PHY (Radio Configurator):
 *     base frequency  430.000 MHz
 *     channel spacing  25 kHz
 *     number of channels 401        -> 430.000 ... 440.025 MHz folytonosan
 * es akkor itt: TUNE_BASE_KHZ 430000, TUNE_SPACING_KHZ 25, MAX_CHANNEL 400.
 *
 * ================== 2026-08-03: EZ EGYSZER MAR MEGFOGOTT ==================
 *
 * Az ertek 434000 volt, mikozben a PHY 144.8 MHz-en allt. A vetel ATTOL
 * MEG MUKODOTT — a frekvenciat a RAIL PHY adja, nem ez a define. Ez a
 * szam csak a szoftver HITE arrol, hol van a 0. csatorna.
 *
 * A kar a KRISTALYKORREKCION keresztul jott. Indulaskor:
 *     set_freq_tick(corr_tick_for_khz(current_khz()))
 * es a current_khz() ebbol a base-bol szamol. Tehat a 434 MHz-re valo
 * korrekciot alkalmaztuk egy 144.8 MHz-es vetelre:
 *
 *     434.0 MHz * 4.38 ppm = 1901 Hz  -> 409 tick   (ezt alkalmaztuk)
 *     144.8 MHz * 4.38 ppm =  634 Hz  -> 136 tick   (ennyi kellett volna)
 *     ---------------------------------------------------------------
 *     tulkorrekcio                      1268 Hz
 *
 * Es tenyleg: a HackRF-fel (TCXO, +-0.5 ppm = +-72 Hz 144.8-on, tehat
 * gyakorlatilag pontos) a vetel ~1.3 kHz-cel elcsuszva jott. Nem a HackRF
 * tevedett — mi.
 *
 * A hangolasi parancsok (f<khz>) is ebbol szamolnak csatornat, tehat azok
 * is melle mentek volna.
 *
 * TANULSAG: ha PHY-t valtasz, EZT IS ALLITSD AT. A boot-uzenet kiirja a
 * feltetelezett frekvenciat es a belole szamolt korrekciot — ha az nem
 * egyezik azzal, ahol tenylegesen hallgatozol, ez a hiba. */
#define TUNE_BASE_KHZ      144800u
#define TUNE_SPACING_KHZ   25u        /* 0 = nincs csatornaracs, csak offszet */
#define TUNE_MAX_CHANNEL   800u       /* a PHY-ban konfiguralt csatornaszam-1
                                       * 2026-08-15: 400 -> 800 (radioconf-fal
                                       * egyutt): 144.8..164.8 MHz = 20 MHz
                                       * scan-sav. A sav szele fele az analog
                                       * bemeneti illesztes miatt romolhat az
                                       * erzekenyseg — merni! */
#define TUNE_TICK_MHZ      4.6492     /* Hz / tick, FG23 @ 39 MHz */

/* Aktualis csatorna (UART-rol allithato) */
static volatile uint16_t s_channel = IQ_CHANNEL;

/* Az AKTUALIS synth-offszet tickben. Alapbol a kristalykorrekcio, de az
 * 'F' es az 'o' parancs elallitja — ezert NEM szabad a restart_rx()-ben
 * fixen a kalibracios erteket visszairni, mert az minden RX-ujrainditasnal
 * elrontana a hangolast. (Pontosan ez a hiba volt a regi kodban.) */
static volatile int32_t s_freq_tick = 0;   /* app_init allitja be */

/* RSSI-trigger allapot — feljebb kellett hozni, mert a stream-inditot
 * (stream_start_R) mar az app_init is hivja. */
static bool armed = false;
static int16_t arm_thresh_qdbm = 4 * (-95);   /* -95 dBm, negyed-dBm */

/* ---------------- allapot ---------------- */

static iq_in_t capture_buf[CAPTURE_SAMPLES];
static volatile uint32_t capture_idx  = 0;
static volatile bool     capturing    = false;
static volatile bool     capture_done = false;
static volatile bool     capture_bad  = false;
static volatile uint32_t stat_events = 0, stat_short_reads = 0,
                         stat_overflows = 0;

/* uint32_t hatteru buffer = garantalt 4 bajtos igazitas, makrok nelkul */
static uint32_t rx_fifo_words[RX_FIFO_BYTES / 4];
#define rx_fifo ((uint8_t *)rx_fifo_words)

static RAIL_Handle_t s_rail = NULL;

/* ---------------- RX FIFO (Series 2) ----------------
 * Ha a linker "multiple definition of RAILCb_SetupRxFifo" hibat dob,
 * a projekt mar ad sajatot (pl. valamelyik pelda-forras) — akkor EZT
 * a fuggvenyt torold, es a masikban allitsd a meretet. */
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle)
{
  uint16_t size = RX_FIFO_BYTES;
  RAIL_Status_t st = RAIL_SetRxFifo(railHandle, rx_fifo, &size);
  return st;
}

/* ---------------- esemeny-callback ----------------
 * ISR-kontextus (a RAIL majdnem mindig megszakitasbol hiv) — csak
 * FIFO-olvasas es indexeles, semmi mas. A geckokapula rail_callback()
 * kozvetlen leszarmazottja. */
void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  /* Benchmark mod ('k'): a mereskor a modul sajat drain-je es szamlaloi
   * futnak, az itteni capture-ag teljesen kimarad. */
  if (iq_bench_on_event(rail_handle, events)) return;

  /* Folyamatos stream mod ('i'): sajat helyben-feldolgozo aga van. */
  if (iq_stream_on_event(rail_handle, events)) return;

  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    ++stat_overflows;
    if (capturing) capture_bad = true;
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    ++stat_events;

    if (!capturing) {
      /* Folyamatos vetel, de nem gyujtunk: URESIG uritunk es eldobunk,
       * hogy a FIFO sose csorduljon tul — igy a burst inditasa
       * pillanatszeru es az elso mintatol koherens.
       *
       * A DRAIN FELTETELE SOHA NEM LEHET MAGASABB AZ ESEMENY KUSZOBENEL.
       * Ha itt THRESHOLD_BYTES (256) allna, es valaki kisebb kuszobre
       * allitja a radiot (pl. a 'k' benchmark 128-ra), ez a ciklus soha
       * nem lepne be, semmit nem uritene, es az esemeny vegtelenul ujra
       * elsulne -> megszakitas-vihar, befagyas. Ezert kis, FIX maradekig
       * uritunk, ami minden ertelmes kuszobnel kisebb. */
      #define DRAIN_RESIDUE_BYTES  64u
      static uint8_t sink[THRESHOLD_BYTES];
      uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
      uint8_t guard = 32u;            /* 32 * 256 B = 8 KiB > teljes FIFO */
      while (avail >= DRAIN_RESIDUE_BYTES && guard--) {
        uint16_t chunk = (avail > sizeof sink) ? (uint16_t)sizeof sink
                                               : avail;
        chunk = (uint16_t)(chunk & ~(sizeof(iq_in_t) - 1u));
        if (chunk == 0u) break;
        if (RAIL_ReadRxFifo(rail_handle, sink, chunk) == 0u) break;
        avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
      }
      return;
    }

    /* URESIG olvasunk a capture-bufferbe. Ha az ISR pontosan erkezik,
     * ez egyetlen olvasas — vagyis a viselkedes azonos a korabbival.
     * Ha egyszer keset, a regi kod CSENDBEN mintat vesztett (fazistores
     * a burst kozepen!), ez viszont utolerni probal. */
    uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
    uint8_t guard = 16u;              /* kemeny korlat: az ISR MINDIG kilep */
    while (avail >= THRESHOLD_BYTES
           && capture_idx < CAPTURE_SAMPLES
           && guard--) {
      uint32_t room = (CAPTURE_SAMPLES - capture_idx) * sizeof(iq_in_t);
      uint32_t want = THRESHOLD_BYTES;
      if (want > room)  want = room;
      if (want > avail) want = avail;
      want &= ~(sizeof(iq_in_t) - 1u);
      if (want == 0u) break;

      uint16_t nread = RAIL_ReadRxFifo(rail_handle,
                                       (uint8_t *)&capture_buf[capture_idx],
                                       (uint16_t)want);
      if (nread != want) ++stat_short_reads;
      if (nread == 0u) break;
      capture_idx += nread / sizeof(iq_in_t);
      avail = RAIL_GetRxFifoBytesAvailable(rail_handle);
    }

    if (capture_idx >= CAPTURE_SAMPLES) {
      capturing = false;
      capture_done = true;
    }
  }
}

/* ---------------- burst dump ----------------
 * Onleiro keret: "IQB1" | u32 n | u32 fs_hint | u8 fmt(=1) | 3x pad |
 * payload | "IQE1" — az iq_view_stream.py --port/--file modja olvassa. */
static void dump_capture(uint32_t fs_hint)
{
  static const uint8_t hdr[4] = { 'I', 'Q', 'B', '1' };
  static const uint8_t trl[4] = { 'I', 'Q', 'E', '1' };
  uint32_t n = capture_idx;
  uint8_t meta[12];
  memcpy(&meta[0], &n, 4);
  memcpy(&meta[4], &fs_hint, 4);
  meta[8] = 1; meta[9] = meta[10] = meta[11] = 0;

  sl_iostream_write(sl_iostream_vcom_handle, hdr, sizeof hdr);
  sl_iostream_write(sl_iostream_vcom_handle, meta, sizeof meta);
  sl_iostream_write(sl_iostream_vcom_handle, capture_buf,
                    n * sizeof(iq_in_t));
  sl_iostream_write(sl_iostream_vcom_handle, trl, sizeof trl);
}

/* ---------------- init + fo ciklus ---------------- */

/* Elore-deklaraciok: ezeket az app_init/fo ciklus hasznalja, de
 * lejjebb vannak definialva. */
static void pb0_init(void);
static void aprs_send_beacon(void);
static void beacon_max_power(void);
static void offset_speed_benchmark(void);
static void nco_tone_test(double f_tone);
static void restart_rx(void);
static void handle_line(const char *line);

/* A kristalykorrekcio TICKBEN, az adott frekvenciara. Lasd a
 * FREQ_CORR_PPB-nel, miert nem lehet ez konstans. */
static int32_t corr_tick_for_khz(uint32_t khz)
{
  double hz = (double)khz * 1000.0 * (double)FREQ_CORR_PPB / 1e9;
  return (int32_t)(hz / TUNE_TICK_MHZ + (hz >= 0 ? 0.5 : -0.5));
}

/* Az eppen hangolt frekvencia kHz-ben, a csatornabol es az offszetbol. */
static uint32_t current_khz(void)
{
  return (uint32_t)((long)TUNE_BASE_KHZ + (long)s_channel * (long)TUNE_SPACING_KHZ);
}

/* ---------------- a synth-offszet EGYETLEN beallito utja ----------------
 * Clampel, elmenti, beirja a radioba, ES megmondja az iq_stream modulnak
 * is — mert az RX-et tobb helyen ujrainditja, es a RAIL_StartRx nem orzi
 * meg az offszetet. Ha barhol maskepp allitod, elobb-utobb szet fog
 * csuszni a ketto, es a vevo csendben melle vesz. */
static void set_freq_tick(int32_t tick)
{
  /* RAIL_FREQUENCY_OFFSET_MIN/MAX = -+0x3FFF */
  if (tick >  16383) tick =  16383;
  if (tick < -16383) tick = -16383;
  s_freq_tick = tick;
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)tick);
  iq_stream_set_freq_tick(tick);
}

/* ---------------- stream-indito (kozos ut) ----------------
 * Az autostart ES az 'i<R>' parancs is EZT hivja, hogy a kiirt szoveg es a
 * tenyleges viselkedes garantaltan ugyanaz legyen. */
/* Az eppen fuo (vagy utoljara hasznalt) decimacio — a hangolas utani
 * ujrainditas ebbol tudja, mivel indult ujra. */
static uint32_t s_stream_R = IQ_AUTOSTART_DECIM;
/* Futott-e a stream a scan inditasa elott (W0 utan visszaindul). */
static bool s_scan_resume = false;
#if CMDLINK_ENABLE
/* Yield a scan varakozasaibol: a cmdlink egy sorat dolgozza fel (ha van). */
static void cmdlink_yield(void) { cmdlink_poll(); }
#endif

static void stream_start_R(uint32_t R, uint8_t shift, const char *honnan)
{
  if (R < 8u)    R = 8u;
  if (R > 4096u) R = 4096u;
  R = (R / 8u) * 8u;
  s_stream_R = R;
  armed = false;

  /* A kimeneti ratat a MODULTOL kerdezzuk, nem sajat keplettel. A regi
   * valtozat fixen 1 Msps bemenetet feltetelezve szamolt (1000000/R), es
   * 400 ksps-nel "31250 sps"-t irt oda, ahol valojaban 12500 ment ki. */
  uint32_t sps = iq_stream_out_sps((uint16_t)R);
  printf("# stream indul [%s]: R=%lu -> %lu sps (sav +-%lu.%lu kHz), "
         "%lu B/s\r\n",
         honnan, (unsigned long)R, (unsigned long)sps,
         (unsigned long)(sps / 2000u), (unsigned long)((sps / 200u) % 10u),
         (unsigned long)(sps * 4u));
  printf("# BARMELY billentyu = stop, utana minden parancs elerheto\r\n");
  iq_stream_start(s_rail, s_channel, (uint16_t)R, shift);
  /* Ov + nadrag: a modul mar magatol visszairja az offszetet minden
   * StartRx utan (iq_stream_set_freq_tick), de itt is beallitjuk, hogy
   * egy elfelejtett kezdeti hivas se tudja csendben elhangolni a vevot. */
  set_freq_tick(s_freq_tick);
}

/* ---------------- 'F<kHz>' : abszolut hangolas ----------------
 * durva = csatornaracs, finom = synth offszet. A kristalykorrekcio
 * (a frekvenciabol szamolt ppm-korrekcio) BENNE van a vegeredmenyben. */
static void tune_khz(uint32_t khz)
{
  int32_t d_khz = (int32_t)khz - (int32_t)TUNE_BASE_KHZ;
  int32_t ch = 0;

#if (TUNE_SPACING_KHZ > 0)
  /* kerekites a legkozelebbi csatornara (negativ irany is helyesen) */
  int32_t sp = (int32_t)TUNE_SPACING_KHZ;
  ch = (d_khz >= 0) ? (d_khz + sp / 2) / sp
                    : (d_khz - sp / 2) / sp;
  if (ch < 0) ch = 0;
  if (ch > (int32_t)TUNE_MAX_CHANNEL) ch = (int32_t)TUNE_MAX_CHANNEL;
  int32_t res_hz = (d_khz - ch * sp) * 1000;
#else
  int32_t res_hz = d_khz * 1000;
#endif

  int32_t tick = (int32_t)(res_hz / TUNE_TICK_MHZ) + corr_tick_for_khz(khz);

  /* A RAIL_FrequencyOffset_t 15 bites elojeles. Tulcsordulas eseten NEM
   * hangolunk vakon: megmondjuk, hogy a csatornaracsot kell allitani. */
  if (tick > 16383 || tick < -16383) {
    if (d_khz < 0) {
      printf("# %lu kHz a PHY base (%lu kHz) ALATT van, es a finom "
             "offszet csak +-76 kHz. A csatornaszam elojel nelkuli, "
             "lefele nincs racs.\r\n",
             (unsigned long)khz, (unsigned long)TUNE_BASE_KHZ);
      printf("# MEGOLDAS: Radio Configuratorban a base frequency-t vidd "
             "430.000 MHz-re (spacing 25 kHz, 401 csatorna), es itt a "
             "TUNE_BASE_KHZ-t is 430000-re.\r\n");
    } else {
      printf("# %lu kHz nem erheto el: a maradek %ld Hz tul nagy "
             "(%ld tick, max +-16383). Novelj a TUNE_MAX_CHANNEL-en, vagy "
             "kisebb spacinget allits.\r\n",
             (unsigned long)khz, (long)res_hz, (long)tick);
    }
    return;
  }

  s_channel = (uint16_t)ch;
  set_freq_tick(tick);
  restart_rx();

  printf("# hangolas: %lu kHz = base %lu + ch %ld * %u kHz + %ld Hz "
         "(offszet %ld tick)\r\n",
         (unsigned long)khz, (unsigned long)TUNE_BASE_KHZ, (long)ch,
         (unsigned)TUNE_SPACING_KHZ, (long)res_hz, (long)tick);
}

void app_init(void)
{
  s_rail = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);

  RAIL_DataConfig_t dc = {
    .txSource = TX_PACKET_DATA,
    .rxSource = RX_IQDATA_FILTLSB,   /* erros jelre valto: FILTMSB */
    .txMethod = PACKET_MODE,
    .rxMethod = FIFO_MODE,
  };
  RAIL_ConfigData(s_rail, &dc);
  RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
  RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(s_rail, false, true);
  RAIL_StartRx(s_rail, s_channel, NULL);

  iq_stream_init(rx_fifo, RX_FIFO_BYTES);
  /* Kalibracio — set_freq_tick-en keresztul, hogy az iq_stream modul is
   * megkapja. Ha csak RAIL_SetFreqOffset-et hivnank, a stream inditasa
   * (ami ujrainditja az RX-et) csendben eldobna. */
  set_freq_tick(corr_tick_for_khz(current_khz()));

  {
    scan_grid_t g = {
      .base_khz    = TUNE_BASE_KHZ,
      .spacing_khz = TUNE_SPACING_KHZ,
      .max_channel = TUNE_MAX_CHANNEL,
      .tick_hz     = TUNE_TICK_MHZ,
      .corr_ppb    = FREQ_CORR_PPB,
    };
    scan_init(s_rail, &g);
#if CMDLINK_ENABLE
    /* A scan a varakozasai alatt uriti a cmdlink FIFO-jat (16 bajt!) —
     * enelkul a gyorsan erkezo W-parancsok sorai serulnek (2026-08-15:
     * "cmdlink: 39 sor, 20 hiba", 640 helyett 100/1016 bin). */
    scan_set_yield(cmdlink_yield);
#endif
  }

  pb0_init();
  sl_iostream_set_default(sl_iostream_vcom_handle);
  printf("\r\n# FG23 IQ capture + TX eval (SiSDK 2025.6). "
         "c=capture s=statusz a=auto-trigger\r\n");
  printf("# F<kHz>=ABSZOLUT hangolas (pl. 'F433775'), f<ch>=nyers csatorna, "
         "o<tick>=finom offszet\r\n");
  printf("# t<dBm>=trigger-kuszob\r\n");
  printf("# TX: w=CW p=PN9 x=stop d<dBm>=teljesitmeny b=APRS-bacon — "
         "CSILLAPITAS/DUMMY az SDR ele!\r\n");
  printf("# NCO teszt: z=offszet-benchmark y[<Hz>]=teszthang (def 1200)\r\n");
  printf("# CW Morse: M1=CQ  M2=VVV  M3=beacon  M <szoveg>  M/WPM <szoveg>\r\n");
  printf("# k[<mp>]=I/Q rata benchmark   n[<mp>]=zero-copy (NULL) teszt\r\n");
  printf("# i[<R>]=folyamatos decimalt I/Q stream (R=8..4096)\r\n");
  printf("# SPI kimenet:  g[<mp>]=lab-teszt + kitoltes-referencia (1.65 V)  "
         "v=orajel/route-diag\r\n");
  printf("# PB0 gomb = MAX POWER APRS bacon (dummy load!)\r\n");
  printf("# hangolasi racs: base %lu kHz + ch * %u kHz, ch max %u — "
         "EGYEZZEN A RADIO CONFIGURATORRAL!\r\n",
         (unsigned long)TUNE_BASE_KHZ, (unsigned)TUNE_SPACING_KHZ,
         (unsigned)TUNE_MAX_CHANNEL);
  /* A korrekcio SZAMSZERUEN, nem csak a ppm. Ha a "feltetelezett" nem az,
   * ahol tenylegesen hallgatozol, akkor a korrekcio is rossz — es a jel
   * pont ennyivel fog elcsuszni. Egyszer mar 1268 Hz-et vitt el igy. */
  {
    uint32_t k = current_khz();
    int32_t  c = corr_tick_for_khz(k);
    printf("# kristalykorrekcio: %ld ppb -> feltetelezett %lu kHz-en "
           "%ld tick = %ld Hz\r\n",
           (long)FREQ_CORR_PPB, (unsigned long)k, (long)c,
           (long)(c * TUNE_TICK_MHZ));
  }

#if CMDLINK_ENABLE
  /* UGYANAZ a parancs-feldolgozo, mint a terminale. Egy parser, egy
   * viselkedes — nem lehet ket kulon igazsag arrol, mit csinal egy 'F'. */
  cmdlink_init(handle_line);
  printf("# cmdlink: PA06 / EXP 11 <- ESP GPIO4, 115200 — hangolas a "
         "telefonrol\r\n");
#endif

#if IQ_AUTOSTART
  /* Varunk egy kicsit: menjen ki a fenti szoveg, es alljon fel az ESP32
   * (az o bootja ~700 ms). Nem sl_sleeptimer, mert az meg nem biztos, hogy
   * inicializalt — a RAIL ora viszont mar megy. */
  {
    RAIL_Time_t t = RAIL_GetTime() + IQ_AUTOSTART_DELAY_MS * 1000u;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
  }
  printf("# --- AUTOSTART (IQ_AUTOSTART_DECIM = %u) ---\r\n",
         (unsigned)IQ_AUTOSTART_DECIM);
  stream_start_R(IQ_AUTOSTART_DECIM, (uint8_t)IQ_AUTOSTART_SHIFT, "autostart");
#else
  printf("# autostart KI (IQ_AUTOSTART=0) — inditsd kezzel: i32\r\n");
#endif
}

/* RX ujrainditasa az aktualis csatornan (frekvencia-valtas utan).
 * A synth-offszetet az s_freq_tick-bol allitjuk vissza, NEM fixen a
 * kalibracios ertekbol — kulonben minden ujrainditas eldobna az 'F'-fel
 * beallitott hangolast. */
static void restart_rx(void)
{
  RAIL_Idle(s_rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(s_rail, false, true);
  RAIL_StartRx(s_rail, s_channel, NULL);
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
}

/* ---------------- TX stream (Phase 3: TX evaluation) ----------------
 * RAIL_StartTxStream maga leallit minden folyo radiomuveletet, de a
 * capture-allapotot nekunk kell konzisztensen tartani, ezert futo
 * gyujtes alatt nem engedjuk el az adast. */
static volatile bool s_tx_active = false;
static RAIL_StreamMode_t s_tx_mode = RAIL_STREAM_CARRIER_WAVE;

static void tx_stream_start(RAIL_StreamMode_t mode)
{
  if (capturing) {
    printf("# capture fut — varj a burst vegere (vagy indits ujra)\r\n");
    return;
  }
  armed = false;                      /* TX alatt nincs RSSI-trigger */
  if (s_tx_active) {
    RAIL_StopTxStream(s_rail);
    s_tx_active = false;
  }
  s_tx_mode = mode;
  RAIL_Status_t st = RAIL_StartTxStream(s_rail, s_channel, mode);
  s_tx_active = (st == RAIL_STATUS_NO_ERROR);
  if (s_tx_active) RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
  printf("# TX %s ch=%u (st=%d)%s\r\n",
         (mode == RAIL_STREAM_CARRIER_WAVE) ? "CW" : "PN9",
         (unsigned)s_channel, (int)st,
         s_tx_active ? " — SUGAROZ! csillapitot az SDR ele!" : "");
}

static void tx_stream_stop(void)
{
  if (s_tx_active) {
    RAIL_StopTxStream(s_rail);
    s_tx_active = false;
  }
  restart_rx();
  printf("# TX stop — vissza RX-be (ch=%u)\r\n", (unsigned)s_channel);
}

/* ---------------- APRS direkt-FSK bacon (T5) ---------------- */
#include "station_config.h"   /* APRS_MYCALL, APRS_INFO (position) — git-ignored, see station_config.example.h */
#define APRS_SSID     12                 /* -12: kiserleti/egyeb allomas */
#define APRS_DEST     "Z2LABS"           /* toCall (eszkoz-azonosito) */
#define APRS_VIA      "WIDE1"            /* digipeater path: WIDE1-1 */
#define APRS_VIA_SSID 1                  /* enelkul a digi nem ismetel! */
/* APRS pozicio-jelentes: '!' = pozicio idobelyeg nelkul.
 * Formatum: !DDMM.mmN/DDDMM.mmE<sym><comment> */

/* ---- NCO-AFSK modulator parameterek (a sweep-meresek alapjan) ----
 * fs=38.4 kHz: THD 2.1%, spur -41 dBc +-38.4 kHz-en (konnyen szurheto),
 * terheles 13%, es PONTOSAN 32 minta / bit 1200 baudon -> a bitido
 * mintaszamlalasbol jon, kulon bit-ora nelkul, driftmentesen.
 * A fazis-akkumulator a mark/space valtasnal NEM nullazodik ->
 * fazisfolytonos Bell 202, ahogy a szabvany keri. */
#define AFSK_FS            38400.0
#define AFSK_MARK_HZ       1200.0
#define AFSK_SPACE_HZ      2200.0
#define AFSK_SAMPLES_PER_BIT 32u        /* 38400 / 1200 */
#define AFSK_DEV_TICK      645          /* szinusz-csucs offszet: +-3 kHz */

/* ---------------- PB0 gomb -> max-power bacon ----------------
 * A gomb az FG23 melyik labara jut, azt a RADIO BOARD dönti el — ezert
 * NEM hardcode-oljuk, hanem a Simplicity board-support headerbol
 * vesszuk. A "Simple Button" komponens (peldany: btn0) hozza. */
#include "sl_simple_button_btn0_config.h"
#define PB0_PORT   SL_SIMPLE_BUTTON_BTN0_PORT
#define PB0_PIN    SL_SIMPLE_BUTTON_BTN0_PIN

static void pb0_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPull, 1 /* felhuzas */);
}

/* Max-power bacon: a PA valodi maximumara allit, ad egy poziciojelentest,
 * majd visszaall a korabbi teljesitmenyre.
 *
 * FONTOS: NEM a RAIL_TX_POWER_MAX sentinelt hasznaljuk raw utvonalon —
 * az 0x7FFF, de a RAIL_SetTxPower raw tipusa unsigned char (8 bit), igy
 * 255-re csonkolna. Helyette a dBm-utat hasznaljuk egy tulzottan magas
 * keressel: a PA-konverzio levagja a valodi elerheto maximumra. */
static void beacon_max_power(void)
{
  RAIL_TxPower_t prev = RAIL_GetTxPower(s_rail);   /* raw egyseg mentese */
  RAIL_Status_t st = RAIL_SetTxPowerDbm(s_rail, (RAIL_TxPower_t)200);
  printf("# PB0 -> MAX POWER bacon (tenyleges %d ddBm, st=%d)\r\n",
         (int)RAIL_GetTxPowerDbm(s_rail), (int)st);
  aprs_send_beacon();
  RAIL_SetTxPower(s_rail, prev);                   /* vissza raw-ra */
}

/* ================= NCO / AFSK (T5b) ================= */

#define SINE_BITS   8
#define SINE_LEN    (1u << SINE_BITS)     /* 256 pont */
#define PHASE_BITS  32                     /* 32 bites fazis-akkumulator */

static int8_t s_sine[SINE_LEN];
static bool   s_sine_ready = false;

static void nco_init_table(void)
{
  if (s_sine_ready) return;
  for (unsigned i = 0; i < SINE_LEN; ++i) {
    double a = (2.0 * 3.14159265358979 * i) / SINE_LEN;
    double v = 127.0 * sin(a);
    s_sine[i] = (int8_t)(v >= 0 ? v + 0.5 : v - 0.5);
  }
  s_sine_ready = true;
}

/* fazis-lepeskoz:  inc = f_tone * 2^PHASE_BITS / f_sample  */
static uint32_t nco_word(double f_tone, double f_sample)
{
  double w = f_tone * 4294967296.0 / f_sample;   /* 2^32 */
  return (uint32_t)(w + 0.5);
}

/* ---------------- 'z' : offszet-sebesseg benchmark ---------------- */
#define BENCH_N 20000u

static void offset_speed_benchmark(void)
{
  bool was_tx = s_tx_active;
  if (!s_tx_active) {
    RAIL_StartTxStream(s_rail, s_channel, RAIL_STREAM_CARRIER_WAVE);
    s_tx_active = true; s_tx_mode = RAIL_STREAM_CARRIER_WAVE;
  }

  /* kis, valtakozo offszetek, hogy a hivas ne legyen "no-op" */
  static const int16_t pat[4] = { +100, -100, +50, -50 };

  RAIL_Time_t t_start = RAIL_GetTime();
  uint32_t min_us = 0xFFFFFFFFu, max_us = 0;
  RAIL_Time_t prev = t_start;

  for (uint32_t i = 0; i < BENCH_N; ++i) {
    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)pat[i & 3]);
    RAIL_Time_t now = RAIL_GetTime();
    uint32_t dt = (uint32_t)(now - prev);
    if (dt < min_us) min_us = dt;
    if (dt > max_us) max_us = dt;
    prev = now;
  }
  RAIL_Time_t t_end = RAIL_GetTime();
  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);

  uint32_t total = (uint32_t)(t_end - t_start);
  uint32_t avg_ns = (total * 1000u) / BENCH_N;   /* atlag ns/hivas */
  uint32_t rate_hz = (avg_ns > 0) ? (1000000000u / avg_ns) : 0;

  printf("# --- SetFreqOffset benchmark (%lu hivas) ---\r\n",
         (unsigned long)BENCH_N);
  printf("# ossz=%lu us, atlag=%lu ns/hivas, ~%lu Hz (%lu.%02lu kHz) "
         "max mintavetel\r\n",
         (unsigned long)total, (unsigned long)avg_ns,
         (unsigned long)rate_hz,
         (unsigned long)(rate_hz / 1000),
         (unsigned long)((rate_hz % 1000) / 10));
  printf("# hivaskoz min=%lu us max=%lu us (jitter=%lu us)\r\n",
         (unsigned long)min_us, (unsigned long)max_us,
         (unsigned long)(max_us - min_us));
  printf("# AFSK-igeny: 9600 Hz -> 104 us/minta. %s\r\n",
         (avg_ns < 104000u) ? "BELEFER (NCO-AFSK jarhato)"
                            : "NEM fer bele — ritkabb mintavetel kell");
  static const uint32_t fs_list[5] = { 9600u, 19200u, 38400u,
                                       76800u, 153600u };
  printf("# sweep-fokozatok terhelese (%% a max mintavetelbol):\r\n");
  for (int k = 0; k < 5; ++k) {
    uint32_t need_ns = 1000000000u / fs_list[k];
    uint32_t load = (rate_hz > 0) ? (fs_list[k] * 100u / rate_hz) : 999;
    printf("#   %6lu Hz: %lu ns/minta kell, terheles ~%lu%% -> %s\r\n",
           (unsigned long)fs_list[k], (unsigned long)need_ns,
           (unsigned long)load,
           (avg_ns < need_ns) ? "OK" : "TUL GYORS (nem birja)");
  }

  if (!was_tx) tx_stream_stop();
}

/* ---------------- 'y' : NCO teszthang-sweep ---------------- */
#define TONE_SECS     5u
#define TONE_GAP_MS   400u
#define TONE_DEV_TICK 645.0     /* +/- csucs-offszet a szinusz +/-1-hez */

static const double s_tone_fs[5] = { 9600.0, 19200.0, 38400.0,
                                     76800.0, 153600.0 };

/* IDOZITES: nincs 64 bites osztas mintankent — Bresenham: egesz us lepes
 * + ns-maradek akkumulator. Igy nincs szisztematikus frekvencia-csuszas
 * (a regi csonkolo valtozat 153.6 kHz-en ~6.5%-kal melyebb hangot adott
 * es torzitott), es a ciklus-terheles is kisebb. */
static void nco_play(double f_tone, double f_sample, uint32_t secs)
{
  uint32_t word = nco_word(f_tone, f_sample);
  uint32_t acc = 0;
  uint32_t nsamp = (uint32_t)(f_sample * secs);
  uint32_t period_ns = (uint32_t)(1e9 / f_sample + 0.5);
  uint32_t whole_us  = period_ns / 1000u;    /* egesz us / minta */
  uint32_t frac_ns   = period_ns % 1000u;    /* maradek ns / minta */
  uint32_t ns_accum  = 0;

  RAIL_Time_t target = RAIL_GetTime();

  for (uint32_t i = 0; i < nsamp; ++i) {
    int8_t s = s_sine[acc >> (PHASE_BITS - SINE_BITS)];
    RAIL_FrequencyOffset_t off = (RAIL_FrequencyOffset_t)
        (s_freq_tick + (s * (int)TONE_DEV_TICK) / 127);
    RAIL_SetFreqOffset(s_rail, off);
    acc += word;
    target += whole_us;
    ns_accum += frac_ns;
    if (ns_accum >= 1000u) { ns_accum -= 1000u; ++target; }
    while ((int32_t)(RAIL_GetTime() - target) < 0) { }
  }
}

static void nco_tone_test(double f_tone)
{
  nco_init_table();
  bool was_tx = s_tx_active;
  if (!s_tx_active) {
    RAIL_StartTxStream(s_rail, s_channel, RAIL_STREAM_CARRIER_WAVE);
    s_tx_active = true; s_tx_mode = RAIL_STREAM_CARRIER_WAVE;
  }
  armed = false;

  printf("# NCO teszthang-sweep: %d Hz, 5 fokozat x %u mp, "
         "dev +/-%d tick\r\n",
         (int)f_tone, (unsigned)TONE_SECS, (int)TONE_DEV_TICK);

  for (int k = 0; k < 5; ++k) {
    double fs = s_tone_fs[k];
    uint32_t spp = (uint32_t)(fs / f_tone + 0.5);   /* minta/periodus */
    printf("#  [%d/5] fs=%6d Hz  (%lu minta/periodus)  %u mp...\r\n",
           k + 1, (int)fs, (unsigned long)spp, (unsigned)TONE_SECS);
    nco_play(f_tone, fs, TONE_SECS);

    RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
    RAIL_Time_t t = RAIL_GetTime() + TONE_GAP_MS * 1000u;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
  }

  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
  printf("# sweep kesz — 5 fokozat felvive\r\n");
  if (!was_tx) tx_stream_stop();
}

static void aprs_send_beacon(void)
{
  if (capturing) { printf("# capture fut — eloszor 's'/varj\r\n"); return; }

  /* STATIKUS, nem stack! Az aprs_frame_t ~1 KB — lokaliskent a SoC Empty
   * stackjet tulcsordítja (hard fault a 'b'-nel, mikozben a w/p megy). */
  static aprs_frame_t fr;
  if (!aprs_build_ui(&fr, APRS_MYCALL, APRS_SSID,
                     APRS_DEST, 0, APRS_VIA, APRS_VIA_SSID, APRS_INFO)) {
    printf("# keret nem fert el\r\n");
    return;
  }
  printf("# APRS bacon (NCO-AFSK): %s -> %s, %u bit (~%lu ms), "
         "fs=%d Hz, %u minta/bit\r\n",
         APRS_MYCALL, APRS_DEST, (unsigned)fr.nbits,
         (unsigned long)(fr.nbits * 1000u / 1200u),
         (int)AFSK_FS, (unsigned)AFSK_SAMPLES_PER_BIT);

  nco_init_table();

  /* vivo be, RSSI-trigger ki */
  armed = false;
  if (!s_tx_active) {
    RAIL_StartTxStream(s_rail, s_channel, RAIL_STREAM_CARRIER_WAVE);
    s_tx_active = true;
    s_tx_mode = RAIL_STREAM_CARRIER_WAVE;
  }

  /* NCO-AFSK: az NRZI-szint valasztja a hangot (1 -> mark 1200 Hz,
   * 0 -> space 2200 Hz), a fazis-akkumulator bitvaltasnal NEM nullazodik
   * -> fazisfolytonos Bell 202. A bitido = 32 minta, kulon ora nelkul. */
  uint32_t word_mark  = nco_word(AFSK_MARK_HZ,  AFSK_FS);
  uint32_t word_space = nco_word(AFSK_SPACE_HZ, AFSK_FS);
  uint32_t acc = 0;

  uint32_t period_ns = (uint32_t)(1e9 / AFSK_FS + 0.5);   /* 26042 ns */
  uint32_t whole_us  = period_ns / 1000u;
  uint32_t frac_ns   = period_ns % 1000u;
  uint32_t ns_accum  = 0;
  RAIL_Time_t target = RAIL_GetTime();

  for (uint16_t i = 0; i < fr.nbits; ++i) {
    uint32_t word = fr.bits[i] ? word_mark : word_space;
    for (uint32_t s = 0; s < AFSK_SAMPLES_PER_BIT; ++s) {
      int8_t sv = s_sine[acc >> (PHASE_BITS - SINE_BITS)];
      RAIL_FrequencyOffset_t off = (RAIL_FrequencyOffset_t)
          (s_freq_tick + (sv * AFSK_DEV_TICK) / 127);
      RAIL_SetFreqOffset(s_rail, off);
      acc += word;
      target += whole_us;
      ns_accum += frac_ns;
      if (ns_accum >= 1000u) { ns_accum -= 1000u; ++target; }
      while ((int32_t)(RAIL_GetTime() - target) < 0) { }
    }
  }

  RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
  tx_stream_stop();
  printf("# bacon kesz\r\n");
}

/* Biztonsagos stream-leallitas: a hivo utana szabadon nyulhat a radiohoz.
 * Visszaad: futott-e a stream (tehat kell-e majd ujrainditani). */
static bool stream_suspend(void)
{
  if (!iq_stream_active()) return false;
  iq_stream_stop(s_rail, s_channel);
  RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
  set_freq_tick(s_freq_tick);
  return true;
}

/* Egy parancssor feldolgozasa (a soremeles/Enter zarja).
 *
 * KET HIVOJA VAN: a terminal (app_process_action) es — ha be van kotve —
 * a cmdlink, vagyis a telefon. A terminalos ut mar leallitotta a streamet
 * (barmely karakter leallitja), a cmdlink viszont NEM: ott a parancs futo
 * stream mellett erkezik.
 *
 * Ezert itt kell rendet tenni. A radiohoz nyulo parancsok futo stream
 * mellett elobb leallitjak azt, es a vegen visszaindul. Enelkul pl. egy
 * telefonrol kuldott 'c' orokre beragasztana a capturing flaget (az
 * esemeny-callback a stream agan kilep, tehat a burst soha nem fejezodne
 * be), es a doboz reset-ig hasznalhatatlan lenne. */
static void handle_line(const char *line)
{
  char c = line[0];

  /* POZITIV lista: EZEK a parancsok nyulnak a radiohoz/FIFO-hoz, tehat
   * ezek elott kell leallni. Forditva (feketelista) rossz lenne: a
   * cmdlink egy paritas nelkuli, egyszalu UART, es egy elrontott bajt
   * utan egy nem letezo parancs is stop-hangolas-start korre kenyszeritene
   * a streamet — lathato lyuk a vizesesen, a semmiert.
   * ('F' nincs a listan: o maga kezeli a leallitast es az ujrainditast.) */
  static const char NEEDS_STOP[] = "cwpxbzydfkngMW";
  bool resume_after = false;
  if (strchr(NEEDS_STOP, c) != NULL && c != '\0' && iq_stream_active()) {
    printf("# a(z) '%c' parancshoz leallitom a streamet...\r\n", c);
    resume_after = stream_suspend();
  }

  if (c == 'c') {
    if (s_tx_active) {
      printf("# TX megy — eloszor 'x' (stop), aztan capture\r\n");
    } else if (!capturing) {
      capture_idx = 0; capture_bad = false; capture_done = false;
      capturing = true;
    }
  } else if (c == 'a') {
    if (s_tx_active) {
      printf("# TX megy — eloszor 'x' (stop), aztan elesites\r\n");
      return;
    }
    armed = !armed;
    printf("# armed=%d (kuszob %d dBm) — jelre magatol indul\r\n",
           (int)armed, (int)(arm_thresh_qdbm / 4));
  } else if (c == 'w') {              /* T1: modulalatlan vivo */
    tx_stream_start(RAIL_STREAM_CARRIER_WAVE);
  } else if (c == 'p') {              /* T6: PN9 modulalt spektrum */
    tx_stream_start(RAIL_STREAM_PN9_STREAM);
  } else if (c == 'x') {
    tx_stream_stop();
  } else if (c == 'b') {              /* T5: APRS direkt-FSK bacon */
    aprs_send_beacon();
  } else if (c == 'z') {              /* T5b: offszet-sebesseg benchmark */
    offset_speed_benchmark();
  } else if (c == 'y') {              /* T5b: NCO teszthang */
    double f = (line[1]) ? (double)atoi(line + 1) : 1200.0;
    if (f < 100.0 || f > 4000.0) f = 1200.0;
    nco_tone_test(f);
  } else if (c == 'M') {              /* CW Morse ado */
    /* M1 / M2 / M3 / M <szoveg> / M/WPM <szoveg>
     * A NEEDS_STOP mar leallitotta a streamet. TX utan s_tx_active=false,
     * igy a resume_after visszainditja a streamet. */
    uint8_t wpm = CW_DEFAULT_WPM;
    char sub = line[1];
    if (sub == '1' || sub == '2' || sub == '3') {
      const char *p = line + 2;
      if (*p == '/') wpm = (uint8_t)atoi(p + 1);
      s_tx_active = true;
      if (sub == '1')      cw_morse_cq(s_rail, s_channel, wpm);
      else if (sub == '2') cw_morse_test(s_rail, s_channel, wpm);
      else                 cw_morse_beacon(s_rail, s_channel, wpm);
      s_tx_active = false;
    } else if (sub == ' ' || sub == '/') {
      const char *p = line + 1;
      if (*p == '/') {
        wpm = (uint8_t)atoi(p + 1);
        while (*p && *p != ' ') ++p;
      }
      while (*p == ' ') ++p;
      if (*p) {
        s_tx_active = true;
        cw_morse_send(s_rail, s_channel, p, wpm);
        s_tx_active = false;
      } else {
        printf("# CW: M1=CQ  M2=VVV  M3=beacon  M <szoveg>  M/WPM <szoveg>\r\n");
      }
    } else {
      printf("# CW: M1=CQ  M2=VVV  M3=beacon  M <szoveg>  M/WPM <szoveg>\r\n");
    }
  } else if (c == 'd') {              /* TX teljesitmeny dBm-ben */
    int dbm = atoi(line + 1);
    RAIL_Status_t st = RAIL_SetTxPowerDbm(s_rail,
                                          (RAIL_TxPower_t)(dbm * 10));
    printf("# TX power = %d dBm kerve (st=%d, tenyleges %d ddBm)\r\n",
           dbm, (int)st, (int)RAIL_GetTxPowerDbm(s_rail));
    if (s_tx_active) {                /* elo streamre ujrainditassal hat */
      tx_stream_start(s_tx_mode);
    }
  } else if (c == 'F') {              /* ABSZOLUT hangolas kHz-ben */
    uint32_t khz = (uint32_t)atoi(line + 1);
    if (khz < 100000u || khz > 1000000u) {
      printf("# hasznalat: F<kHz>, pl. F433775 (LoRa-APRS) vagy F434000\r\n");
    } else {
      /* A hangolas RAIL_ResetFifo-t is jelent. A stream ZERO-COPY-val
       * olvas, tehat sajat mutatoja van a FIFO-ba — egy resetet nem elne
       * tul szinkronban. Ezert menet kozbeni hangolasnal LEALLITJUK es
       * ugyanazzal az R-rel UJRAINDITJUK. Par ezred masodperc szunet. */
      uint32_t R = s_stream_R;
      bool was_stream = stream_suspend();
      tune_khz(khz);
      if (s_tx_active) {
        tx_stream_start(s_tx_mode);   /* a vivo kovesse a hangolast */
      } else if (!capturing) {
        /* Hangolas utan MINDIG legyen I/Q-folyam. Korabban csak akkor
         * indult vissza, ha epp futott (was_stream) — de az ESP32 az SDR++
         * IQ-modjaba lepeskor csak F<kHz>-t kuld, sose explicit "i"-t, ezert
         * egy scan (W0) utan a stream allva maradt es az IQ "nem indult".
         * (2026-08-15) */
        (void)was_stream;
        stream_start_R(R, (uint8_t)IQ_AUTOSTART_SHIFT, "hangolas utan");
      }
    }
  } else if (c == 'f') {
    /* Nyers csatornavaltas. Ugyanaz a FIFO-veszely, mint az 'F'-nel: a
     * restart_rx() RAIL_ResetFifo-t hiv, amit a zero-copy stream sajat
     * olvasomutatoja nem elne tul. A stream mar le van allitva a
     * handle_line elejen (az 'f' nincs a safe listan), de a csatornat
     * itt is le kell hatarolni — kulonben a RAIL_StartRx csendben hibat
     * ad, es a radio egyszeruen nem vesz. */
    long ch = atoi(line + 1);
    if (ch < 0) ch = 0;
    if (ch > (long)TUNE_MAX_CHANNEL) {
      printf("# csatorna %ld > TUNE_MAX_CHANNEL (%u) — levagva\r\n",
             ch, (unsigned)TUNE_MAX_CHANNEL);
      ch = (long)TUNE_MAX_CHANNEL;
    }
    s_channel = (uint16_t)ch;
    if (s_tx_active) {
      tx_stream_start(s_tx_mode);     /* scriptelt leptetes */
    } else {
      restart_rx();
    }
    printf("# csatorna = %u  (~%lu kHz)\r\n", (unsigned)s_channel,
           (unsigned long)(TUNE_BASE_KHZ + s_channel * TUNE_SPACING_KHZ));
  } else if (c == 'o') {
    /* Clampelve: a RAIL_SetFreqOffset a +-0x3FFF-en kivul CSENDBEN hibat
     * ad vissza, es a vevo a nevlegesen marad. Ket kulon rejtett hiba
     * lenne belole: elhangolt vevo, es a kiirt Hz-ertek int32-tulcsordulasa
     * (|tick| > ~46200-nal). */
    set_freq_tick((int32_t)atoi(line + 1));
    printf("# finom offszet = %ld tick (%ld Hz)\r\n",
           (long)s_freq_tick, (long)(s_freq_tick * 46492 / 10000));
  } else if (c == 'k') {              /* T-plafon: I/Q rata benchmark */
    if (s_tx_active) {
      printf("# TX megy — eloszor 'x' (stop), aztan benchmark\r\n");
    } else if (capturing) {
      printf("# capture fut — varj a burst vegere\r\n");
    } else {
      /* 'k' = 10 s/pont (gyors iteracio), 'k60' = 60 s/pont (jegyzokonyv) */
      uint32_t secs = (line[1]) ? (uint32_t)atoi(line + 1) : 10u;
      if (secs < 2u || secs > 120u) secs = 10u;
      armed = false;                  /* meres alatt nincs RSSI-trigger */
      iq_bench_sweep(s_rail, s_channel, secs, THRESHOLD_BYTES);
      RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);   /* kalibracio vissza */
    }
  } else if (c == 'i') {              /* folyamatos decimalt stream */
    if (iq_stream_active()) {
      iq_stream_stop(s_rail, s_channel);
      RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
      RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
    } else if (s_tx_active || capturing) {
      printf("# TX vagy capture fut — eloszor allitsd le\r\n");
    } else {
      /* 'i<R>' — R a TELJES decimacio, tetszoleges egesz (nem log2!).
       * Argumentum nelkul az AUTOSTART ertekevel indul, hogy az 'i' meg
       * mindig azt adja, amit bekapcsolaskor lattal. */
      uint32_t R = (line[1]) ? (uint32_t)atoi(line + 1)
                             : (uint32_t)IQ_AUTOSTART_DECIM;
      stream_start_R(R, (uint8_t)IQ_AUTOSTART_SHIFT, "parancs");
    }
  } else if (c == 'g') {              /* I2S lab-teszt (GPIO billegtetes) */
    if (s_tx_active || capturing || iq_stream_active()) {
      printf("# eloszor allitsd le a futo muveletet\r\n");
    } else {
      uint32_t secs = (line[1]) ? (uint32_t)atoi(line + 1) : 10u;
      if (secs < 1u || secs > 60u) secs = 10u;
      iq_stream_pin_test(secs);
    }
  } else if (c == 'v') {              /* EUSART orajel-diagnosztika */
    iq_stream_dump_uart_cfg();
  } else if (c == 'n') {              /* zero-copy (NULL) olvasas teszt */
    if (s_tx_active) {
      printf("# TX megy — eloszor 'x' (stop)\r\n");
    } else if (capturing) {
      printf("# capture fut — varj a burst vegere\r\n");
    } else {
      uint32_t secs = (line[1]) ? (uint32_t)atoi(line + 1) : 10u;
      if (secs < 2u || secs > 120u) secs = 10u;
      armed = false;
      iq_bench_zerocopy_test(s_rail, s_channel, secs, THRESHOLD_BYTES);
      RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
    }
  } else if (c == 'W') {
    /* Szelessavu scan (RSSI-panadapter). W<kozep_kHz>,<span_kHz>,<nbin>
     * [,<floor_dBm>,<range_dB>]  |  W0 = stop. Terminalrol ES cmdlinkrol
     * (ESP32 -> SpyServer FFT-mod) ugyanaz. A scan es a stream kizarjak
     * egymast: a NEEDS_STOP miatt a stream mar all; scan alatt NEM indul
     * vissza (lasd lent), W0 utan viszont igen. */
    if (s_tx_active) {
      printf("# TX megy — eloszor 'x' (stop), aztan scan\r\n");
    } else {
      unsigned long ck = 0, sk = 0, nb = 0; long fl = -130, rg = 100;
      /* Szigoru ellenorzes: csak szamjegy, vesszo es minusz lehet a sorban.
       * Egy serult cmdlink-sor (FIFO-tulcsordulas) kulonben "ertelmes"
       * parametereket adhat — pl. 640 helyett 100 vagy 1016 bint. */
      bool clean = true;
      for (const char *q = line + 1; *q; q++) {
        if (!((*q >= '0' && *q <= '9') || *q == ',' || *q == '-')) { clean = false; break; }
      }
      int got = clean ? sscanf(line + 1, "%lu,%lu,%lu,%ld,%ld", &ck, &sk, &nb, &fl, &rg) : 0;
      if (!clean) printf("# scan: serult parancssor eldobva: '%s'\r\n", line);
      if (got >= 1 && ck == 0) {
        if (scan_active()) {
          scan_stop();
          RAIL_ConfigEvents(s_rail, RAIL_EVENTS_ALL,
                            RAIL_EVENT_RX_FIFO_ALMOST_FULL
                            | RAIL_EVENT_RX_FIFO_OVERFLOW);
          restart_rx();
          set_freq_tick(s_freq_tick);
          /* W0 utan a stream MINDIG visszaindul (a doboz alapallapota az
           * autostartos I/Q-folyam). 2026-08-15: az ESP32 ujraflashelese +
           * terminal-billentyuk utan a stream allva maradt, es az SDR++
           * IQ-modja "nem ment" — pedig csak a FG23 folyama nem futott. */
          resume_after = true;
          s_scan_resume = false;
        } else {
          printf("# scan nem fut\r\n");
          /* W0 akkor is jelentse: "legyen I/Q-folyam" — ha a stream all
           * (terminal-billentyu, ESP32-flash), inditsuk vissza. */
          if (!iq_stream_active() && !s_tx_active && !capturing) resume_after = true;
        }
      } else if (got >= 3) {
        if (got >= 5) scan_set_scale((int16_t)fl, (uint16_t)rg);
        if (!scan_active()) s_scan_resume = resume_after;
        resume_after = false;          /* scan alatt a stream NEM indul vissza */
        if (!scan_start((uint32_t)ck, (uint32_t)sk, (uint16_t)nb)) {
          printf("# scan: rossz parameterek (W<kHz>,<span_kHz>,<nbin>)\r\n");
          resume_after = s_scan_resume; s_scan_resume = false;
        }
      } else {
        printf("# hasznalat: W<kozep_kHz>,<span_kHz>,<nbin>[,floor,range] | W0\r\n");
      }
    }
  } else if (c == 'T') {
    /* Scan-idozites futas kozben: T<avg_us>,<wait_us>  (pl. T300,3000).
     * A 's' es a sor-diagnosztika mutatja, hany bin marad ervenytelen. */
    unsigned long st = 0, wt = 0, md = 0;
    int got_t = sscanf(line + 1, "%lu,%lu,%lu", &st, &wt, &md);
    if (got_t >= 2) {
      scan_set_timing((uint32_t)st, (uint32_t)wt);
      if (got_t >= 3) scan_set_method((uint8_t)md);
    } else {
      printf("# hasznalat: T<avg_us>,<wait_us>[,<mod 0|1|2>]\r\n");
    }
  } else if (c == 't') {
    arm_thresh_qdbm = (int16_t)(4 * atoi(line + 1));
    printf("# trigger-kuszob = %d dBm\r\n", (int)(arm_thresh_qdbm / 4));
  } else if (c == 's') {
    /* A tenyleges hangolas visszaszamolva, hogy latszodjon, HOL all a
     * vevo — ne kelljen fejben osszeadni a csatornat es az offszetet. */
    long hz = (long)TUNE_BASE_KHZ * 1000
              + (long)s_channel * (long)TUNE_SPACING_KHZ * 1000
              + (long)((s_freq_tick - corr_tick_for_khz(current_khz()))
                       * 46492 / 10000);
    printf("# events=%lu short=%lu ovf=%lu idx=%lu armed=%d\r\n",
           (unsigned long)stat_events, (unsigned long)stat_short_reads,
           (unsigned long)stat_overflows, (unsigned long)capture_idx,
           (int)armed);
    printf("# hangolas: ~%ld.%03ld kHz (ch=%u, offszet=%ld tick) "
           "rssi=%d dBm\r\n",
           hz / 1000, hz % 1000, (unsigned)s_channel, (long)s_freq_tick,
           (int)(RAIL_GetRssi(s_rail, false) / 4));
    printf("# stream=%d (R=%lu, %lu sps)  tx=%d(%s)\r\n",
           (int)iq_stream_active(), (unsigned long)s_stream_R,
           (unsigned long)iq_stream_out_sps((uint16_t)s_stream_R),
           (int)s_tx_active,
           (s_tx_mode == RAIL_STREAM_CARRIER_WAVE) ? "CW" : "PN9");
    scan_print_stats();
#if CMDLINK_ENABLE
    uint32_t cl_lines = 0, cl_err = 0;
    cmdlink_stats(&cl_lines, &cl_err);
    printf("# cmdlink: %lu sor, %lu hiba\r\n",
           (unsigned long)cl_lines, (unsigned long)cl_err);
#endif
  }

  /* Ha a parancs miatt leallitottuk a streamet, most visszaindul —
   * kiveve, ha epp a parancs allitotta TX-be vagy capture-be a radiot. */
  if (resume_after) {
    if (!s_tx_active && !capturing) {
      stream_start_R(s_stream_R, (uint8_t)IQ_AUTOSTART_SHIFT, "parancs utan");
    } else {
      printf("# a stream NEM indul vissza (TX vagy capture fut) — "
             "'x' majd 'i'\r\n");
    }
  }
}

void app_process_action(void)
{
  static char linebuf[64];   /* CW szabad szoveghez is eleg (M ...) */
  static uint8_t len = 0;

  char ch;
  bool have_ch = false;

  /* --- folyamatos stream: a fo ciklus dolga a pump. Barmely bejovo
   * karakter leallitja. --- */
#if CMDLINK_ENABLE
  /* A masodik parancsbemenet a stream alatt is el: a telefonrol jovo
   * hangolas nem allitja le a folyamot (a tune maga gondoskodik a
   * biztonsagos ujrainditasrol). */
  cmdlink_poll();
#endif

  /* Szelessavu scan: binenkent hangol+RSSI, kesz sort SPI-n kikuld.
   * Kizarja a streamet, tehat ide csak akkor jutunk, ha az nem fut. */
  if (scan_active()) {
    scan_process();
    /* Terminalrol barmely NEM-sorveg karakter leallitja a scant is (mint a
     * streamet), es a karakter a parancspufferbe kerul. A CR/LF viszont
     * TOVABBMEGY a sorgyujtobe — 2026-08-15: korabban eldobtuk (return),
     * ezert scan alatt a begepelt parancs (pl. T100,1000,2) sorvege
     * elveszett, es a parancs SOSEM futott le. */
    if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
      if (ch != '\r' && ch != '\n') {
        handle_line("W0");
        printf("# scan all. 'W<kHz>,<span>,<nbin>' = ujra\r\n");
      }
      have_ch = true;
    } else {
      return;
    }
  }

  /* have_ch eseten NEM nyulunk a streamhez: a scan-ag W0-ja epp most
   * inditotta ujra, es a getchar itt elnyelne a pufferelt karaktert. */
  if (iq_stream_active() && !have_ch) {
    iq_stream_pump();
    /* A sorveg-karakterek NEM allitjak le a streamet! A terminal CR+LF-et
     * kuld: a '\r' inditja a parancsot, es a bentmaradt '\n' azonnal le
     * is allitana. (Pontosan ez tortent: a stream ~168 us utan meghalt,
     * nulla esemennyel.) Barmely MAS karakter leallit. */
    if (sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
      if (ch != '\r' && ch != '\n') {
        iq_stream_stop(s_rail, s_channel);
        RAIL_SetRxFifoThreshold(s_rail, THRESHOLD_BYTES);
        RAIL_SetFreqOffset(s_rail, (RAIL_FrequencyOffset_t)s_freq_tick);
        printf("# stream all. 'i' = ujra, 'F<kHz>' = hangolas, "
               "'s' = statusz\r\n");
      }
      /* A leallito karakter NEM VESZ EL: beleesik a parancs-pufferbe.
       * Igy autostart mellett is eleg egyszeruen begepelni, hogy
       * "F433775" — az elso 'F' allitja le a streamet ES o lesz a
       * parancs elso betuje. A CR/LF pedig TOVABBMEGY a sorgyujtobe,
       * hogy a felig begepelt sor le tudjon zarodni (2026-08-15). */
      have_ch = true;
    } else {
      return;
    }
  }

  /* Karakterek gyujtese sorvegig; a parancsok igy argumentumot is
   * kaphatnak (pl. 'F433775', 'o-40'). Nem-blokkolo getchar. */
  if (have_ch
      || sl_iostream_getchar(sl_iostream_vcom_handle, &ch) == SL_STATUS_OK) {
    if (ch == '\r' || ch == '\n') {
      if (len > 0) {
        linebuf[len] = '\0';
        handle_line(linebuf);
        len = 0;
      }
    } else if (len < sizeof(linebuf) - 1) {
      linebuf[len++] = ch;
    } else {
      len = 0;   /* tulcsordulas -> eldobjuk */
    }
  }

  /* RSSI-elesitett inditas: amikor a sav megszolal, azonnal gyujtunk —
   * igy egy eterbol jovo APRS-csomag ELEJET kapjuk el.
   * TX stream alatt ertelmetlen (nincs RX), ezert kihagyjuk. */
  if (armed && !capturing && !capture_done && !s_tx_active) {
    int16_t rssi_qdbm = RAIL_GetRssi(s_rail, false);
    if (rssi_qdbm != RAIL_RSSI_INVALID && rssi_qdbm > arm_thresh_qdbm) {
      capture_idx = 0; capture_bad = false;
      capturing = true;
      armed = false;
    }
  }

  if (capture_done) {
    capture_done = false;
    if (capture_bad) {
      printf("# OVERFLOW a burst alatt — eldobva, probald ujra\r\n");
    } else {
      dump_capture(0 /* fs_hint: onkalibracio utan ird be Hz-ben */);
    }
  }

  /* PB0 gomb: lenyomas ELERE (aktiv-alacsony) egy max-power bacon.
   * Egyszeru szoftveres pergesmentesites: csak akkor tuzel, ha az elozo
   * allapot magas volt es most alacsony. */
  static bool pb0_prev_high = true;
  bool pb0_now_high = (GPIO_PinInGet(PB0_PORT, PB0_PIN) != 0);
  if (pb0_prev_high && !pb0_now_high) {
    /* rovid varakozas a pergesmentesitesert (~5 ms) */
    RAIL_Time_t t = RAIL_GetTime() + 5000u;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
    if (GPIO_PinInGet(PB0_PORT, PB0_PIN) == 0 && !capturing) {
      beacon_max_power();
    }
  }
  pb0_prev_high = pb0_now_high;
}

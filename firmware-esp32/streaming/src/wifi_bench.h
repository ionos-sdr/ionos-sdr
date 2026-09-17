/* wifi_bench.h - WiFi-atviteli plafon merese az FG23-SDR lancban
 *
 * HA7DCD / Kolibri.  A kerdes, amit ez a modul megvalaszol:
 *   "mekkora sps mellett marad a REND: lyuk= nulla tiz percen at"
 * - vagyis NEM a nyers iperf-szam, hanem az a rata, ahol a WiFi-forgalom
 * meg nem eszi meg az SPI-kiszolgalast.
 *
 * Hasznalat main.cpp-ben:
 *
 *   #include "wifi_bench.h"
 *   setup():   wifi_bench_init();  wifi_bench_set_ext(bench_ext_cb);
 *   loop():    bench_console_tick();
 *
 * A modul SAJAT taskot es SAJAT listen-socketet nyit (alapbol 7777), a
 * meglevo 8888 / 5555 utakat nem piszkalja.  Egyszerre futtathato a valodi
 * IQ-streammel - pont az az ertelme.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- a lanc sajat szamlaloi, amiket a bench lepcsonkent kiolvas ---------
 * A modul NEM ismeri a main.cpp valtozoit; te kototd be egy callbackkel.
 * Ami nincs, maradjon 0.  Minden mezo KUMULATIV (a bench kepez deltat).
 */
typedef struct {
    uint32_t spi_lost;        /* valodi blokkveszteseg (REND: lyuk=) */
    uint32_t spi_dup;         /* duplikatum (REND: dup=)             */
    uint32_t spi_ovf;         /* rxring tulcsordulas (ovf=)          */
    uint32_t spi_blocks;      /* vett blokkok osszesen               */
    uint32_t spi_ring_max;    /* RXRING vizjel (NEM kumulativ, max)  */
    uint32_t loop_dt_max_us;  /* fo ciklus max periodus (NEM kum.)   */
    uint32_t spy_drop;        /* spyserver eldobott uzenet           */
} wb_ext_t;

typedef void (*wb_ext_fn)(wb_ext_t *out);

/* Ebben a projektben (main.cpp, 2026-08-17) a valodi nevek:
 *
 *   static void bench_ext_cb(wb_ext_t *o) {
 *       o->spi_lost       = st_ro_hole;    // REND: lyuk=  <- A VALODI
 *       o->spi_dup        = st_ro_dup;     // REND: dup=
 *       o->spi_ovf        = rx_ring_ovf;   // 0-nak kell lennie
 *       o->spi_blocks     = st_blocks;
 *       o->spi_ring_max   = rx_ring_max;
 *       o->loop_dt_max_us = st_dtmax;
 *       o->spy_drop       = spy_dropped();
 *   }
 *
 * FIGYELEM: a st_lost (VESZT) ~16x tulszamol a driver-glitch miatt -
 * a plafon meghatarozasahoz a st_ro_hole a helyes szam.
 */
void wifi_bench_set_ext(wb_ext_fn fn);

/* ---- eletciklus -------------------------------------------------------- */
void wifi_bench_init(void);
void wifi_bench_stop(void);
bool wifi_bench_running(void);

/* ---- parancsertelmezo --------------------------------------------------
 * true = a sort ez a modul kezelte (ne add tovabb a tobbi parsernek).
 *
 *   B                                   statusz + aktualis konfig
 *   B0                                  stop
 *   Br<kBps>[,<sec>]                    fix rata (sec elhagyva: vegtelen)
 *   Bs<start>,<stop>,<step>,<dwell_s>   lepcsos rampa kB/s-ban
 *   Bm[<sec>]                           max: fekezes nelkul, plafonkereses
 *   Bset k=v [k=v ...]                  konfig (lasd lent)
 *
 * Bset kulcsok:
 *   port=7777      listen port (csak allo bencshez)
 *   blk=1040       keretmeret bajtban (24 B fejlec + payload); 64..8192
 *   chunk=2880     egy send() hivas max merete (2880 = 2x MSS)
 *   core=0         a bench task magja (0 vagy 1)   [WiFi-mag = 0 ajanlott]
 *   prio=5         a bench task prioritasa (1..20; a spi_rx_task 22!)
 *   nodelay=1      TCP_NODELAY
 *   sndbuf=0       SO_SNDBUF bajtban (0 = ne allitsa)
 *   warm=2         lepcsonkent ennyi masodperc bemelegites (nem szamit bele)
 *   win=1000       riportablak ms-ban
 *   agg=1          hany keret menjen egy loketben (1 = hu az IQ-hoz)
 *   burst=4        max ennyi keretnyi token halmozodhat (loketesseg)
 *   quiet=0        1 = csak a lepcso-osszegzo sorok menjenek ki
 */
bool wifi_bench_cmd(const char *line);

#ifdef __cplusplus
}
#endif

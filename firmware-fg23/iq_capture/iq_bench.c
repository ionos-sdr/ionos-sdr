/* SPDX-License-Identifier: MIT
 *
 * iq_bench.c — FG23 folyamatos I/Q rata benchmark
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * ================= MERESI ELV =================
 *
 * A benchmark alatt SEMMI nem megy ki a UART-on. Ha kiirnank, a UART-ot
 * mernenk, nem a chipet. A mintak egy kis scratch-bufferbe mennek es ott
 * meg is halnak — csak szamlalunk.
 *
 * Amit merunk:
 *   fs      — empirikusan: mintaszam / RAIL-ido. NEM az adatlapbol.
 *   OVR     — RAIL_EVENT_RX_FIFO_OVERFLOW szamlalo. Ez A plafon.
 *   peak    — mennyire telt meg a FIFO belepeskor. Ez a TARTALEK.
 *             A puszta "volt-e overrun" semmit nem mond arrol, hogy
 *             30%-on vagy 95%-on szaladsz.
 *   CPU%    — mennyit esz a capture ISR; a maradek a DSP-e.
 *
 * A CPU%-ot ugy merjuk, ahogy a SetFreqOffset benchmarknal: egy ismert
 * terhelo ciklus iteracioit szamoljuk radio nelkul (referencia), majd
 * capture kozben. A kulonbseg = elvett CPU.
 *
 * ================= AMI ELTER AZ app.c DRAIN-JETOL =================
 *
 * Az app.c "eldobo" aga esemenyenkent PONTOSAN egyszer olvas
 * THRESHOLD_BYTES-ot. Ha a mintarata akkora, hogy ket ISR belepes kozott
 * ennel tobb gyulik, a FIFO monoton telik es a tulcsordulas garantalt —
 * fuggetlenul attol, hogy a CPU birna-e. Az itteni drain URESIG olvas,
 * igy a valodi szilicium/CPU plafont meri, nem ezt a mesterseges korlatot.
 *
 * ================= FLOAT PRINTF =================
 *
 * Az app.c sehol nem hasznal %f-et — a Silabs projektek alapertelmezett
 * "tiny printf"-je nem tud lebegopontot kiirni. Ezert itt MINDEN kiiras
 * egesz szamokkal megy, kezi tizedessel. Ne tegyel bele %f-et.
 */

#include "iq_bench.h"
#include <stdio.h>
#include <string.h>

/* ---------------- konfiguracio ---------------- */

/* Threshold-sweep lepesek. A threshold az ALMOST_FULL kivaltasi szintje:
 * kicsi -> suru ISR (nagy CPU), nagy -> kevesebb tartalek az overrunig.
 * Az app.c jelenlegi erteke 256 (64 minta) — az a kozepso pont. */
/* A LEGBIZTONSAGOSABB ponttal kezdunk (ritka ISR) es haladunk a suru fele.
 * Igy ha egy alacsony threshold telitodest okoz, a fenti pontok adatai
 * mar megvannak. */
static const uint16_t bench_thresholds[] = { 2048, 1024, 512, 256, 128 };
#define BENCH_NUM_THR (sizeof(bench_thresholds)/sizeof(bench_thresholds[0]))

/* Scratch: ide olvasunk es itt haljon meg. Nem kell nagy — koronkent
 * ujraolvasunk, amig a FIFO ki nem urul. */
#define BENCH_SCRATCH 2048u

/* Meddig urritsuk a FIFO-t? NEM nullaig: 400 ksps-en 4 bajt 10 us-onkent
 * erkezik, egy ciklus-iteracio meg ~2 us — az `avail >= 4` feltetel soha
 * nem lenne hamis (ez volt az elso beragadas). 128 bajt = 320 us utanpotlas,
 * bosegesen a ciklusido folott, tehat garantaltan kilep. */
#define BENCH_MIN_DRAIN 128u

/* ---------------- allapot ---------------- */

static volatile bool     b_running   = false;
static volatile uint32_t b_bytes     = 0;
static volatile uint32_t b_events    = 0;
static volatile uint32_t b_overflows = 0;
static volatile uint16_t b_peak      = 0;
static volatile uint16_t b_threshold = 256;   /* az aktualis meresi pont */

/* Ha igaz, a drain NULL celcimmel hivja a RAIL_ReadRxFifo-t: elvileg
 * "eldobas masolas nelkul", azaz csak az olvasomutato lep. A BUFC
 * ugyanis RAM-ba ir (datasheet 3.2.6: zero-copy), tehat a mintak mar
 * a helyukon vannak — a masolas felesleges munka. */
static volatile bool b_zerocopy = false;

/* A meres hataridejet MAGA AZ ISR orzi. Ha a mintarata olyan magas, hogy
 * a capture ISR a CPU-t 100%-ban elviszi, a fociklus SOHA nem jut elore —
 * es ha a leallas feltetele ott lenne, a rendszer orokre beragadna. Ezert
 * az ISR minden belepeskor megnezi, lejart-e az ido, es o allitja le a
 * radiot. Innen a fociklus magatol feleled. */
static volatile RAIL_Time_t b_deadline = 0;
static volatile bool        b_starved  = false;

static uint32_t b_scratch_words[BENCH_SCRATCH / 4];
#define b_scratch ((uint8_t *)b_scratch_words)

/* Terhelo ciklus referenciaja, iteracio/s radio nelkul. */
static uint32_t b_base_iters_per_s = 0;

/* Optimalizacio ellen. */
static volatile uint32_t b_sink = 0;

/* ---------------- terhelo ciklus ---------------- */

/* Szandekosan egyszeru egesz MAC-lanc. Nem a valodi blocker-FFT-t
 * utanozza, hanem egy STABIL, ismetelheto merce, amivel a capture ISR
 * CPU-lopasa megfoghato. Egeszekkel megy, hogy az FPU jelenlete vagy
 * hianya ne torzitsa a merest. */
static uint32_t bench_load_spin(uint32_t microseconds)
{
  RAIL_Time_t t0 = RAIL_GetTime();
  uint32_t iters = 0;
  uint32_t a = 1103515245u, c = 12345u, x = 1u;

  while ((RAIL_GetTime() - t0) < microseconds) {
    /* 16 muvelet / iteracio, hogy a RAIL_GetTime() ne dominaljon */
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    x = x * a + c;  x = x * a + c;  x = x * a + c;  x = x * a + c;
    iters++;
  }
  b_sink = x;
  return iters;
}

/* ---------------- RAIL esemeny ---------------- */

bool iq_bench_active(void) { return b_running; }

bool iq_bench_on_event(RAIL_Handle_t rail, RAIL_Events_t events)
{
  if (!b_running) return false;

  /* Lejart-e a meresi ablak? Ha igen, LE A RADIOVAL — kulonben egy
   * telitett ISR-nel a fociklus soha nem venne eszre a hatarido vegét. */
  if ((int32_t)(RAIL_GetTime() - b_deadline) >= 0) {
    /* NEM RAIL_Idle: azt callback-kontextusbol hivni nem biztonsagos, es
     * a thr=128-as pontnal pont ezen ragadt be. Helyette leMASKOLJUK a
     * FIFO-esemenyeket — nincs tobb belepes, a fociklus feleled, es a
     * rendes leallitast o vegzi el fo-kontextusban. */
    RAIL_ConfigEvents(rail,
                      RAIL_EVENT_RX_FIFO_ALMOST_FULL
                      | RAIL_EVENT_RX_FIFO_OVERFLOW,
                      RAIL_EVENTS_NONE);
    b_running = false;
    b_starved = true;      /* az ISR zarta le, nem a fociklus */
    return true;
  }

  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    ++b_overflows;
    /* Az overflow utan a FIFO tartalma ertelmetlen — uritsuk es menjunk
     * tovabb. Nem allunk le: azt akarjuk tudni, MENNYI overrun van,
     * nem csak azt, hogy volt-e. */
    RAIL_ResetFifo(rail, false, true);
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    /* ELOSZOR a kitoltottseg — ez a tartalek-metrika. Ha elobb
     * olvasnank, mar nem latnank, milyen melyen volt a FIFO. */
    uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail);
    if (avail > b_peak) b_peak = avail;

    ++b_events;

    /* A threshold ALA urritunk, nem nullara — es kemeny iteracio-
     * korlattal. Egy `while (avail >= 4)` ciklus itt VEGTELEN lenne:
     * 160 ksps-en 4 bajt 6 us-onkent erkezik, tehat mire kiolvassuk es
     * ujra megkerdezzuk, mar megint van benne. Az ISR sose lepne ki. */
    uint8_t guard = 32u;              /* 32 * 256 B = 8 KiB > FIFO */
    while (avail >= BENCH_MIN_DRAIN && guard--) {
      uint16_t chunk = (avail > BENCH_SCRATCH) ? BENCH_SCRATCH : avail;
      chunk = (uint16_t)(chunk & ~3u);
      if (chunk == 0u) break;
      uint16_t got = RAIL_ReadRxFifo(rail,
                                    b_zerocopy ? NULL : b_scratch,
                                    chunk);
      if (got == 0u) break;
      b_bytes += got;
      avail = RAIL_GetRxFifoBytesAvailable(rail);
    }
  }

  return true;   /* az app.c capture-aga maradjon ki */
}

/* ---------------- egy meres ---------------- */

typedef struct {
  uint16_t threshold;
  uint32_t duration_us;
  uint32_t bytes;
  uint32_t events;
  uint32_t overflows;
  uint16_t peak;
  uint32_t fs_hz;
  uint32_t fill_permille;   /* peak / 4096, ezrelekben */
  uint32_t cpu_permille;    /* elvett CPU, ezrelekben  */
  uint32_t ev_per_s;
} bench_res_t;

static void bench_one(RAIL_Handle_t rail,
                      uint16_t channel,
                      uint16_t threshold,
                      uint32_t seconds,
                      bench_res_t *r)
{
  memset(r, 0, sizeof(*r));
  r->threshold = threshold;

  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  RAIL_SetRxFifoThreshold(rail, threshold);
  b_threshold = threshold;

  b_bytes = 0; b_events = 0; b_overflows = 0; b_peak = 0;

  RAIL_ResetFifo(rail, false, true);

  RAIL_Time_t t0 = RAIL_GetTime();
  b_deadline = t0 + seconds * 1000000u;
  b_starved  = false;
  b_running  = true;
  RAIL_StartRx(rail, channel, NULL);

  uint32_t iters = bench_load_spin(seconds * 1000000u);

  RAIL_Time_t t1 = RAIL_GetTime();

  /* SORREND! Eloszor a radiot allitjuk le, es CSAK utana adjuk vissza a
   * vezerlest az app.c-nek. Forditva versenyhelyzet van: amint b_running
   * hamis, az esemenyek az app.c dran-agara mennek, ami `avail >= 256`
   * feltetellel dolgozik — ha a meresi kuszob ennel kisebb (128), soha
   * nem lep be, nem urit, es az esemeny vegtelenul ujra elsul. A
   * RAIL_Idle fokontextusban var, tehat sose fejezodik be. Ez volt a
   * thr=128-as befagyas. */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  b_running = false;
  /* Ha az ISR maszkolta le magat (telitodes), itt fegyverezzuk ujra. */
  RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  r->duration_us = (uint32_t)(t1 - t0);
  r->bytes       = b_bytes;
  r->events      = b_events;
  r->overflows   = b_overflows;
  r->peak        = b_peak;

  /* fs: 4 bajt = 1 komplex minta.
   * A RAIL-ido a HFXO-bol (39 MHz) szarmazik — ugyanabbol az
   * orajel-tartomanybol, mint a mintaveteli ora. Amit itt merunk, az
   * tehat a DEKIMACIOS ARANY (fs = 39 MHz / N), nem ket fuggetlen ora
   * osszehasonlitasa. Ezert determinisztikus es ismetelheto. */
  if (r->duration_us > 0u) {
    /* (bytes/4) * 1e6 / us  —  64 biten, hogy ne csorduljon tul */
    uint64_t num = (uint64_t)(r->bytes / 4u) * 1000000ull;
    r->fs_hz    = (uint32_t)(num / r->duration_us);
    r->ev_per_s = (uint32_t)(((uint64_t)r->events * 1000000ull)
                             / r->duration_us);
  }

  r->fill_permille = ((uint32_t)r->peak * 1000u) / 4096u;

  if (b_base_iters_per_s > 0u && r->duration_us > 0u) {
    uint64_t ach = ((uint64_t)iters * 1000000ull) / r->duration_us;
    uint32_t ratio = (uint32_t)((ach * 1000ull) / b_base_iters_per_s);
    if (ratio > 1000u) ratio = 1000u;
    r->cpu_permille = 1000u - ratio;
  }
}

/* ---------------- sweep ---------------- */

void iq_bench_sweep(RAIL_Handle_t rail,
                    uint16_t channel,
                    uint32_t seconds,
                    uint16_t restore_thresh)
{
  bench_res_t r;

  /* CPU referencia: radio kikapcsolva */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  printf("# CPU referencia (radio KI, 2 s)...\r\n");
  (void)bench_load_spin(200000u);              /* bemelegites */
  b_base_iters_per_s = bench_load_spin(2000000u) / 2u;
  printf("# referencia: %lu iter/s\r\n",
         (unsigned long)b_base_iters_per_s);

  printf("#\r\n");
  printf("# ===== FG23 I/Q sustained-rate benchmark =====\r\n");
  printf("# %lu s/pont, csatorna %u, FIFO 4096 B\r\n",
         (unsigned long)seconds, (unsigned)channel);
  printf("#\r\n");
  printf("# minden pont %lu mp-ig NEMA (a UART zavarna a merest)\r\n",
         (unsigned long)seconds);
  printf("# ---------------------------------------------------\r\n");

  for (unsigned k = 0; k < BENCH_NUM_THR; k++) {
    /* Haladasjelzes: a meres alatt SEMMI nem mehet ki a UART-ra (kulonben
     * a UART-ot mernenk), ezert a pont ELOTT szolunk, hogy ne nezzen ki
     * fagyasnak. A '.' sorvege nelkul megy, igy a sor vegen jon a mert
     * adat ugyanabba a sorba. */
    printf("  [%u/%u] thr=%u, %lu mp ... ",
           k + 1u, (unsigned)BENCH_NUM_THR,
           (unsigned)bench_thresholds[k], (unsigned long)seconds);

    bench_one(rail, channel, bench_thresholds[k], seconds, &r);

    printf("fs=%lu.%02lu ksps  ev/s=%lu  peak=%u  fill=%lu.%01lu%%  "
           "OVR=%lu  CPU=%lu.%01lu%%%s\r\n",
           (unsigned long)(r.fs_hz / 1000u),
           (unsigned long)((r.fs_hz % 1000u) / 10u),
           (unsigned long)r.ev_per_s,
           (unsigned)r.peak,
           (unsigned long)(r.fill_permille / 10u),
           (unsigned long)(r.fill_permille % 10u),
           (unsigned long)r.overflows,
           (unsigned long)(r.cpu_permille / 10u),
           (unsigned long)(r.cpu_permille % 10u),
           (r.overflows > 0u) ? "  <-- OVERRUN"
                              : (b_starved ? "  <-- CPU TELITVE" : ""));
  }

  printf("# ---------------------------------------------------\r\n");
  printf("#  OVR>0      -> ez a pont NEM tarthato\r\n");
  printf("#  fill>70%%   -> hatarhelyzet, jitterre erzekeny\r\n");
  printf("#  fs         -> EZT ird az SDR++ sample rate mezojebe\r\n");
  printf("#  CPU%%       -> ennyit esz a capture; a tobbi a DSP-e\r\n");

  /* Vissza az app.c allapotaba, hogy a 'c' capture ugy mukodjon,
   * ahogy eddig. */
  RAIL_SetRxFifoThreshold(rail, restore_thresh);
  RAIL_ResetFifo(rail, false, true);
  RAIL_StartRx(rail, channel, NULL);
  printf("# threshold visszaallitva %u-ra, RX ujraindult\r\n",
         (unsigned)restore_thresh);
}


/* ================= NULL-olvasas proba + osszehasonlitas =================
 *
 * 1. lepes — PROBA (veszelytelen): egyetlen NULL-os olvasas, elotte-utana
 *    kiolvasott FIFO-szinttel. Ha a RAIL megis MASOLNA a NULL-ra, az
 *    hardfault lenne — ezert csak EGY hivas, 64 bajtra, es utana rogton
 *    ellenorzunk. Ha a board tullep rajta es kiirja a sort, az ut nyitva.
 *
 * 2. lepes — MERES: ugyanaz a threshold ket futasban, masolassal es
 *    anelkul. A CPU% kulonbsege maga a valasz.
 */

void iq_bench_zerocopy_test(RAIL_Handle_t rail,
                            uint16_t channel,
                            uint32_t seconds,
                            uint16_t restore_thresh)
{
  printf("\r\n# ===== zero-copy (NULL) teszt =====\r\n");

  /* ---------- 1. PROBA ---------- */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  /* Esemenyek KI: hagyjuk a FIFO-t magatol megtelni, ne dranelje senki. */
  RAIL_ConfigEvents(rail,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW,
                    RAIL_EVENTS_NONE);
  RAIL_ResetFifo(rail, false, true);
  RAIL_StartRx(rail, channel, NULL);

  /* 5 ms: 400 ksps-en boven megtelik a 4096 bajt (2.56 ms) */
  RAIL_Time_t t = RAIL_GetTime() + 5000u;
  while ((int32_t)(RAIL_GetTime() - t) < 0) { }

  /* A RADIOT LEALLITJUK a meres elott! Kulonben kozben tolt: 928 ksps-en
   * 3.7 MB/s, tehat a ket olvasas kozotti par mikroszekundum alatt is
   * beesik 8-10 bajt, es a mutato-lepes latszolag kevesebbnek tunik.
   * Ez volt az elso, teves "NEM tamogatott" eredmeny oka. */
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);

  uint16_t avail0 = RAIL_GetRxFifoBytesAvailable(rail);
  printf("#  proba: elotte avail=%u (radio megallitva)\r\n",
         (unsigned)avail0);

  uint16_t got = RAIL_ReadRxFifo(rail, NULL, 64u);   /* <-- A KERDES */

  uint16_t avail1 = RAIL_GetRxFifoBytesAvailable(rail);

  uint16_t moved = (avail0 > avail1) ? (uint16_t)(avail0 - avail1) : 0u;
  printf("#  proba: visszaadott=%u  utana avail=%u  -> a mutato %u bajtot lepett\r\n",
         (unsigned)got, (unsigned)avail1, (unsigned)moved);

  /* Allo radional ennek pontosnak kell lennie. */
  bool ok = (got == 64u) && (moved == 64u);
  if (!ok) {
    printf("#  -> NEM tamogatott (a mutato nem lepett) — marad a masolas\r\n");
    RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                      RAIL_EVENT_RX_FIFO_ALMOST_FULL
                      | RAIL_EVENT_RX_FIFO_OVERFLOW);
    RAIL_SetRxFifoThreshold(rail, restore_thresh);
    RAIL_ResetFifo(rail, false, true);
    RAIL_StartRx(rail, channel, NULL);
    return;
  }
  printf("#  -> MUKODIK: a mutato masolas nelkul lepett\r\n#\r\n");

  RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  /* ---------- 2. MERES ---------- */
  if (b_base_iters_per_s == 0u) {
    printf("# CPU referencia (radio KI, 2 s)...\r\n");
    (void)bench_load_spin(200000u);
    b_base_iters_per_s = bench_load_spin(2000000u) / 2u;
    printf("# referencia: %lu iter/s\r\n", (unsigned long)b_base_iters_per_s);
  }

  bench_res_t r;
  const uint16_t thr = 1024u;          /* a valasztott munkapont */

  for (int pass = 0; pass < 2; pass++) {
    b_zerocopy = (pass == 1);
    printf("  %-12s thr=%u, %lu mp ... ",
           b_zerocopy ? "NULL (zero)" : "masolassal",
           (unsigned)thr, (unsigned long)seconds);

    bench_one(rail, channel, thr, seconds, &r);

    printf("fs=%lu.%02lu ksps  ev/s=%lu  OVR=%lu  CPU=%lu.%01lu%%\r\n",
           (unsigned long)(r.fs_hz / 1000u),
           (unsigned long)((r.fs_hz % 1000u) / 10u),
           (unsigned long)r.ev_per_s,
           (unsigned long)r.overflows,
           (unsigned long)(r.cpu_permille / 10u),
           (unsigned long)(r.cpu_permille % 10u));
  }

  b_zerocopy = false;
  RAIL_SetRxFifoThreshold(rail, restore_thresh);
  RAIL_ResetFifo(rail, false, true);
  RAIL_StartRx(rail, channel, NULL);
  printf("# kesz — a ket CPU%% kulonbsege a masolas ara\r\n");
}

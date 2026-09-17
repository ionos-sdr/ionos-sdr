/* SPDX-License-Identifier: MIT
 *
 * usb_bench.ino — MEKKORA A NATIV USB PLAFONJA az ESP32-S3-on?
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================== MIT MER ==================
 *
 * Semmilyen radio, semmilyen SPI. Csak generalunk 1040 bajtos, ervenyes
 * "IQB2" blokkokat, es kinyomjuk oket a NATIV USB CDC-n, amilyen gyorsan
 * csak lehet. A PC oldalon az usb_sink.py olvassa es szamolja.
 *
 * MIERT KELL A PC OLDAL IS: az USB CDC-n a GAZDAGEP kezdemenyez. Ha senki
 * nem olvas, a TX puffer tele lesz es 0 kB/s-t mersz — nem a linket meritek,
 * hanem azt, hogy nincs olvaso. A szam CSAK az usb_sink.py-jal ervenyes.
 *
 * ================== NEGY FAZIS ==================
 *
 * Fazisonkent 10 masodperc, aztan magatol lep. A vegen osszefoglalo.
 *
 *   1  1040 B / iras, NEM-BLOKKOLO   <- pontosan a mai firmware viselkedese
 *   2  1040 B / iras, BLOKKOLO       <- mennyit nyerunk a varakozassal
 *   3  4160 B / iras, blokkolo       <- 4 blokk egyben, kevesebb hivas
 *   4  8320 B / iras, blokkolo       <- 8 blokk egyben
 *
 * A 3. es 4. fazis a lenyeg: a CDC-nek nagy iras kell, hogy tele USB-
 * kereteket tudjon kuldeni. Ha a 4. fazis jelentosen tobbet ad az 1-nel,
 * akkor az igazi firmware-ben is tobb blokkot kell egyszerre kiirni.
 *
 * ================== EREDMENY-OLVASAS ==================
 *
 * A kiirt kB/s-bol a maximalis mintavetel:  sps = kB/s * 1024 / 4
 *
 *     500 kB/s  -> 125 ksps   (ez kell az 1 Msps-es PHY + R=8 tervhez)
 *     900 kB/s  -> 230 ksps
 *    1000 kB/s  -> 256 ksps   (a full-speed USB gyakorlati teteje)
 *
 * ================== KET ES FELE PROBALD ==================
 *
 * Az S3-ban KETFELE USB-periferia van, MINDKETTO a GPIO19/20-on:
 *
 *   ARDUINO_USB_MODE=1  -> HWCDC, a beepitett USB-Serial/JTAG kontroller.
 *                          Ez a mostani. Egyszeru, de a szakirodalom
 *                          szerint lassabb.
 *   ARDUINO_USB_MODE=0  -> TinyUSB (USBCDC) az OTG periferian. Tobbet
 *                          szokott adni, cserebe tobb RAM-ot esz.
 *
 * MERD MEG MINDKETTOT. Egy sor a platformio.ini-ben, es lehet, hogy
 * ketszeres a kulonbseg. A mostani firmware HWCDC-t hasznal, tehat ha a
 * TinyUSB nyer, az egy ingyen sávszelesseg-duplazas.
 *
 * platformio.ini:
 *     build_flags =
 *       -D ARDUINO_USB_MODE=1        ; <- ezt allitgasd 1 <-> 0
 *       -D ARDUINO_USB_CDC_ON_BOOT=1
 *
 * ================== HASZNALAT ==================
 *
 *   1. Flasheld ezt (a CP2102-es porton keresztul, ahogy szoktad).
 *   2. Nyisd a CP2102 portot 115200-on: ott jon a diagnosztika.
 *   3. Inditsd:  python usb_sink.py COM4     (a NATIV USB portja!)
 *   4. Varj 45 masodpercet, es olvasd le a ket oldal szamait.
 *
 * A ket oldal szamanak egyeznie kell. Ha az ESP tobbet mond, mint amit a
 * PC lat, akkor a gazdagep-oldali olvasas a szuk keresztmetszet (Windows
 * usbser.sys szokott ilyet) — az is ertekes informacio.
 */

#include <Arduino.h>
#include <string.h>

/* ---------------- blokk-formatum (egyezik az iq_stream.c-vel) ---------- */
#define BLK_SAMPLES   256
#define HDR_BYTES     16
#define BLK_BYTES     (HDR_BYTES + BLK_SAMPLES * 4)      /* 1040 */
#define IQ_BLK_MAGIC  0x32425149u                        /* "IQB2" */

/* ---------------- meresi parameterek ---------------- */
#define PHASE_MS      10000u        /* fazisonkent ennyi ideig nyomjuk */
/* NAGYOBB kell, mint a legnagyobb chunk (8 * 1040 = 8320), kulonben a 4.
 * fazis soha nem fer be egy atadasba, es epp azt a valtozot rontja el,
 * amiert a merest csinaljuk. */
#define TX_BUF_BYTES  12288         /* erdemes allitgatni: 8192..24576 */
#define MAX_CHUNK_BLK 8             /* a legnagyobb fazis ennyi blokkot ir */

#define DIAG  Serial0               /* CP2102: ide jon a szoveg */
#define RAW   Serial                /* nativ USB: ide megy a nyers folyam */

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint16_t nsamp;
  uint16_t decim;
  uint32_t fs_in_hz;
} blk_hdr_t;

/* Egy elore feltoltott, MAX_CHUNK_BLK blokknyi puffer. A fejleceket
 * kuldes elott frissitjuk, a payload fix — a tartalom itt nem szamit,
 * csak a bajtszam es hogy a PC oldal fel tudja ismerni a keretet. */
static uint8_t txbuf[MAX_CHUNK_BLK * BLK_BYTES];
static uint32_t seq = 0;

typedef struct {
  const char *name;
  uint16_t    nblk;        /* hany blokk egy irasban */
  bool        blocking;    /* varjunk-e, ha nincs hely */
} phase_t;

static const phase_t PHASES[] = {
  { "1040 B  nem-blokkolo (= mai fw)", 1, false },
  { "1040 B  blokkolo",                1, true  },
  { "4160 B  blokkolo (4 blokk)",      4, true  },
  { "8320 B  blokkolo (8 blokk)",      8, true  },
};
#define NPHASE (sizeof(PHASES) / sizeof(PHASES[0]))

static uint32_t res_kbs[NPHASE];
static uint32_t res_drop[NPHASE];
static uint32_t res_maxus[NPHASE];

static void fill_payload(void)
{
  /* Felismerheto, de nem konstans minta: igy a PC oldalon egy elcsuszas
   * is latszana. A tartalom a sebesseget nem befolyasolja. */
  for (int b = 0; b < MAX_CHUNK_BLK; b++) {
    int16_t *iq = (int16_t *)(txbuf + b * BLK_BYTES + HDR_BYTES);
    for (int k = 0; k < BLK_SAMPLES; k++) {
      iq[2 * k]     = (int16_t)(k * 128);        /* I: furesz */
      iq[2 * k + 1] = (int16_t)(-k * 128);       /* Q: tukorkep */
    }
  }
}

static void stamp_headers(uint16_t nblk)
{
  for (uint16_t b = 0; b < nblk; b++) {
    blk_hdr_t *h = (blk_hdr_t *)(txbuf + b * BLK_BYTES);
    h->magic    = IQ_BLK_MAGIC;
    h->seq      = seq++;
    h->nsamp    = BLK_SAMPLES;
    h->decim    = 8;
    h->fs_in_hz = 400000u;
  }
}

void setup()
{
  DIAG.begin(115200);

  /* A puffert MEG A begin() ELOTT kell megnovelni. Alapbol par szaz bajt,
   * amibe egy 1040 bajtos blokk sem fer bele — ezen bukott el eloszor az
   * egesz lanc (USB 0.0 kB/s, minden blokk eldobva). */
  RAW.setTxBufferSize(TX_BUF_BYTES);
  RAW.setTxTimeoutMs(0);
  RAW.begin();

  delay(800);
  fill_payload();

  DIAG.println("\n=== ESP32-S3 nativ USB atviteli plafon ===");
#if ARDUINO_USB_MODE
  DIAG.println("periferia: HWCDC (USB-Serial/JTAG)   [ARDUINO_USB_MODE=1]");
#else
  DIAG.println("periferia: TinyUSB CDC (OTG)         [ARDUINO_USB_MODE=0]");
#endif
  DIAG.printf("TX puffer: %d B   blokk: %d B   fazis: %lu ms\n",
              TX_BUF_BYTES, BLK_BYTES, (unsigned long)PHASE_MS);
  DIAG.println("INDITSD A PC OLDALT:  python usb_sink.py COMxx");
  DIAG.println("(olvaso nelkul a meres ertelmetlen — 0 kB/s-t fogsz latni)");
  DIAG.println();
}

static void run_phase(int idx)
{
  const phase_t *p = &PHASES[idx];
  const size_t chunk = (size_t)p->nblk * BLK_BYTES;

  DIAG.printf("[%d/%d] %-34s ", idx + 1, (int)NPHASE, p->name);
  DIAG.flush();

  /* Blokkolo fazisban engedunk varakozast, de nem vegtelent: ha nincs
   * gazdagep, 50 ms utan feladjuk es dropnak szamoljuk. Igy a teszt akkor
   * sem all meg, ha elfelejtetted elinditani az usb_sink.py-t. */
  RAW.setTxTimeoutMs(p->blocking ? 50 : 0);

  uint64_t written = 0;
  uint32_t drops = 0, max_us = 0;
  uint32_t t0 = millis();

  uint32_t polls = 0;

  while (millis() - t0 < PHASE_MS) {

    if (!p->blocking && RAW.availableForWrite() < (int)chunk) {
      /* NEM szamolunk itt dropot: ez egy sikertelen LEKERDEZES, nem egy
       * eldobott blokk. A regi valtozat itt tizmilliokat szamolt, es az
       * elso fazis "eldobas" oszlopa ertelmetlen volt.
       * A delay(1) a task watchdog miatt kell: 10 masodperc szoros
       * porgetes kivagja az IDLE taszkot. delay(0) NEM eleg — az csak
       * taskYIELD, ami azonos vagy magasabb prioritasu taszkra valt, az
       * IDLE viszont alacsonyabb. Egy tick varakozas kell. Hozamot nem
       * ront: ide csak akkor jutunk, ha a TX puffer tele van. */
      polls++;
      delay(1);
      continue;
    }

    /* A fejlecet CSAK akkor belyegezzuk, ha tenyleg kuldunk. Kulonben a
     * sorszam a nem-kuldott blokkokra is lepne, es az usb_sink.py
     * fantom-vesztest jelentene. */
    stamp_headers(p->nblk);

    uint32_t u0 = micros();
    size_t n = RAW.write(txbuf, chunk);
    uint32_t du = micros() - u0;
    if (du > max_us) max_us = du;

    written += n;
    if (n < chunk) drops++;
  }

  RAW.setTxTimeoutMs(0);

  float dt = (millis() - t0) / 1000.0f;
  uint32_t kbs = (uint32_t)((written / 1024.0f) / dt + 0.5f);
  res_kbs[idx]   = kbs;
  res_drop[idx]  = drops;
  res_maxus[idx] = max_us;

  DIAG.printf("%5lu kB/s   csonka iras=%-4lu  ures lekerdezes=%-9lu  "
              "leghosszabb iras=%lu us\n",
              (unsigned long)kbs, (unsigned long)drops,
              (unsigned long)polls, (unsigned long)max_us);
}

void loop()
{
  static bool done = false;
  if (done) { delay(1000); return; }

  DIAG.println("--- meres indul, ~40 masodperc ---");
  for (int i = 0; i < (int)NPHASE; i++) run_phase(i);

  uint32_t best = 0;
  int best_i = 0;
  for (int i = 0; i < (int)NPHASE; i++) {
    if (res_kbs[i] > best) { best = res_kbs[i]; best_i = i; }
  }

  /* 4 bajt / komplex minta */
  uint32_t sps = (uint32_t)((uint64_t)best * 1024u / 4u);

  DIAG.println("\n=========== OSSZEFOGLALO ===========");
  DIAG.printf("legjobb: %s -> %lu kB/s\n",
              PHASES[best_i].name, (unsigned long)best);
  DIAG.printf("ez %lu sps (%lu.%03lu ksps) 16 bites I/Q-val\n",
              (unsigned long)sps,
              (unsigned long)(sps / 1000), (unsigned long)(sps % 1000));
  DIAG.printf("8 bites I/Q-val ennek a duplaja: %lu sps\n",
              (unsigned long)(sps * 2u));
  DIAG.println();

  if (sps >= 125000u) {
    DIAG.println("VERDIKT: az 1 Msps-es PHY + R=8 (125 ksps, 500 kB/s) BELEFER.");
    DIAG.println("  kovetkezo lepes: SPI_HZ = 12000000, PHY 1 Msps-re,");
    DIAG.printf("  es az ESP-n %d blokkot irj egyszerre.\n",
                (int)PHASES[best_i].nblk);
  } else if (sps >= 60000u) {
    DIAG.println("VERDIKT: 125 ksps NEM fer bele 16 biten. Vagy 8 bitre kell");
    DIAG.println("  valtani (rtl_tcp ut), vagy R=16-tal 62.5 ksps-en maradni,");
    DIAG.println("  vagy WiFi-re tenni a nagy folyamot.");
  } else {
    DIAG.println("VERDIKT: gyanusan keves. Fut az usb_sink.py? Ha igen,");
    DIAG.println("  probald meg ARDUINO_USB_MODE=0-val (TinyUSB) is.");
  }

  if (res_kbs[NPHASE - 1] > res_kbs[0] * 3 / 2) {
    DIAG.println("\nFONTOS: a nagy chunk sokkal tobbet ad, mint az egy blokk.");
    DIAG.println("  Az eles firmware-ben is gyujts ossze tobb blokkot, es");
    DIAG.println("  egyetten irasban kuldd ki oket.");
  }
#if ARDUINO_USB_MODE
  DIAG.println("\nMost probald meg ARDUINO_USB_MODE=0-val (TinyUSB) is —");
  DIAG.println("gyakran jelentosen tobbet ad ugyanezen a ket labon.");
#endif

  done = true;
}

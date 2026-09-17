/* SPDX-License-Identifier: MIT
 *
 * iq_stream.c — folyamatos, decimalt I/Q stream SPI-n az ESP32-S3-nak
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT licenc
 *
 * ================= MIERT SPI ES NEM I2S =================
 *
 * 2026-07-31, multimeteres meres: a FG23 USART-janak CS-e NEM tud valodi
 * I2S word selectet adni. A kitoltese 33.6% (1.11 V), miközben 50% kell.
 * Kontroll: a 'g' lab-teszt, ami mind a harom labat PONTOSAN 50%-on
 * billegteti GPIO-kent, mind a harmon 1.65 V-ot adott — tehat az
 * amplitudo teljes es a meres jo. A nyolc keretezesi kombinacio
 * (W16D16/W32D16 x Left/Right x delay true/false) MIND 1.05-1.11 V.
 *
 * Ok: a FG23-on nincs onallo I2S periferia. Egy altalanos szinkron USART
 * van, amiben a chip select van word selectte atertelmezve. Ez arra
 * keszult, hogy a chip egy audio DAC-ot etessen — ott a vevo buta
 * shiftregiszter. Egy ESP32 I2S slave viszont allapotgep, aminek
 * tankonyvi 50%-os WS kell, kulonben ujra es ujra keresi a keretet.
 *
 * ================= AMIT AZ SPI MEGOLD =================
 *
 * Az I2S rakenyszeritett egy megkotest: a bitora kotelezoen fs*32,
 * FOLYAMATOSAN. Ezert volt halalos minden apro hezag, es ezert kellett a
 * USART-nak sosem kiurulnie.
 *
 * SPI-nal BURST-ben kuldunk: a CS-t levisszuk, kitolunk egy egesz
 * blokkot magas orajellel, felvisszuk, es a vonal pihen a kovetkezoig.
 * A link sebessege LEVALIK a mintaveteli frekvenciarol, es a hezag nem
 * hiba lesz, hanem a normal mukodes resze.
 *
 * Racadasul a blokk-fejlecben van SORSZAM, tehat a csomagvesztes egy
 * kiirhato szam lesz, nem sejtes. Ez minosegileg jobb hiba, mint a
 * csendben, folyamatosan rongalo bitcsuszas.
 *
 * ===================== BEKOTES (valtozatlan drotok!) =====================
 *   WSTK EXP   FG23 lab   SPI jel                sebesseg    ESP32-S3
 *   ---------------------------------------------------------------------
 *   EXP 15     PC05       SCLK                   tobb MHz -> GPIO5
 *   EXP 10     PC00       MOSI (FG23 -> ESP)     tobb MHz -> GPIO6
 *   EXP 13     PA07       CS   (blokk-keret)     ~25 Hz   -> GPIO7
 *   EXP  1     GND                                        -> GND
 *
 * A HARMAS KIVALASZTASA a radiopanel EXP-tablazatabol (UG506, xG23):
 *   EXP 10 = PC0  — SPI_CS pozicio, SEMMIVEL nincs megosztva. Tiszta.
 *   EXP 15 = PC5  — I2C_SCL pozicio (sensor), gyakorlatban tiszta volt.
 *   EXP 13 = PA7  — sima GPIO, semmivel nincs megosztva. Tiszta.
 *
 * FIGYELEM: a CS mostantol a PORT A-n van, nem a C-n! A drive/slew
 * beallitas portonkent hat, ezert az A portot is allitjuk (lasd lejjebb).
 * A CS blokkonkent egyszer vált (~25 Hz), tehat ez nala nem kritikus.
 *
 * ===================== TILTOLISTA =====================
 * 2026-08-01: a radiopanel EXP-tablazata (UG506, xG23 radio board)
 * megmutatta, hogy a 4/6/8-as fejlecpontokra tett feltetelezesunk EGY
 * POZICIOVAL EL VOLT CSUSZVA. A valos kiosztas:
 *
 *   EXP  4 = PC1  — FLASH_MOSI + DISP_SI. A panel hajtja. NE.
 *   EXP  6 = PC2  — FLASH_MISO. Terhelt net. NE.
 *   EXP  8 = PC3  — FLASH_SCLK + DISP_SCLK. NE.
 *   EXP 12 = PA8  — VCOM_TX. Ez a konzol! NE.
 *   EXP 14 = PA9  — VCOM_RX. Ez a konzol! NE.
 *   EXP 17 = BOARD_ID_SCL, EXP 19 = BOARD_ID_SDA — board controller. NE.
 *   EXP  2 = VMCU, EXP 18 = 5V, EXP 20 = 3V3, EXP 1 = GND — tap.
 *
 * EBBOL KOVETKEZETT AZ EGESZ EJSZAKAI HAJSZA: amikor a firmware a PC03-at
 * hajtotta, az az EXP 8-on jott ki (kijelzo-orajel), mikozben a drot az
 * EXP 6-ban ult, amit PC2-kent senki nem hajtott — ezert volt teljesen
 * mindegy, hogy rá van-e dugva. A "PC02 nem viszi a gyors jelet"
 * megfigyeles is innen ered: az valojaban a flash MISO netje volt.
 *
 * Szabad es tiszta tartalek, ha meg kell egy vonal (pl. RDY):
 *   EXP 11 = PA6, EXP 9 = PD2, EXP 7 = PA5 — mind sima GPIO.
 *
 * ESP oldal: GPIO3 strapping lab + valoszinu VBAT-oszto a Heltecen — NE.
 */

#include "iq_stream.h"
#include "sl_iostream.h"
#include "sl_iostream_handles.h"
#include "em_eusart.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_ldma.h"
#include "dmadrv.h"
#include <stdio.h>
#include <string.h>

/* ================= KONFIGURACIO ================= */

/* SPI bitrata. A 12.5 ksps-hez 4 MHz tizszeres tartalek:
 * egy 1040 bajtos blokk 2.1 ms alatt megy ki, miközben 20.5 ms-onkent
 * keletkezik. Dupont drotokon ez meg kenyelmes.
 *
 * A 700 ksps-es PCB-celhoz ~30 MHz kell majd — de ott mar nem Heltec
 * dev board lesz, es IO_MUX-labakra lehet tervezni. */
#define SPI_HZ            4000000u

/* Egy blokk komplex mintaszama. 256 minta = 20.5 ms @ 12.5 ksps.
 * Blokkmeret = 16 bajt fejlec + 256*4 = 1040 bajt. */
#define BLK_SAMPLES       256u

/* Hany blokkot tartunk. A CIC az egyiket tolti, a DMA a masikat kuldi,
 * a tobbi a tartalek. 6 * 1040 = 6240 bajt a 64 kB-bol — olcso biztositas.
 *
 * MIT VED ES MIT NEM: ez a FG23 sajat torlodasa ellen jo. Ha ez telik be,
 * a s_blk_drops no, es a masik oldalon a blokk/s esik — DE SORSZAMLYUK
 * NEM KELETKEZIK, mert a s_seq-et a KIKULDESKOR irjuk a fejlecbe. Tehat:
 *   blokk/s esik, VESZT nulla   -> ITT a baj (nezd a 'blokk-eldobas'-t)
 *   VESZT no                    -> az ESP32 nem vette at, ott keresd */
#define NBLK              6u

/* A CHIP I/Q MINTAVETELE — a 'k' benchmark meresebol!
 *
 * EZ A LEGGYAKRABBAN ELFELEJTETT SOR. Ha PHY-t valtasz, EZT IS ALLITSD AT.
 * Az inditaskor kiirt sorbol azonnal latszik.
 *
 * Mert ertekek: 434M gyari profil -> 400 ksps
 *               100 kHz akv. sav  -> 400 ksps
 *                50 kHz akv. sav  ->  70 ksps
 *               270 kHz akv. sav  ->   1 Msps                      */
#define IQ_STREAM_FS_IN_HZ 400000u

/* ---------------- DC-BLOKK ----------------
 * A vevo DC-offszetje a decimalt sav KOZEPEN van, es a CIC unity gainnel
 * engedi at — enelkul a lanc a racsra tapad. Egypolusu szivo a kimeneti
 * ratan, idoallando ~2^DC_SHIFT minta.
 *
 * MIERT 14 ES NEM 10 (2026-08-03):
 * A 10-es ertek 12.5 ksps-re volt meretezve (~82 ms). i8-on a kimenet
 * 50 ksps, ott ugyanaz a 10 mar csak 20 ms — 7.8 Hz-es sarok.
 *
 * DE FIGYELEM, EZ CSAK ENYHITES, NEM GYOGYITAS. Az igazi baj az, hogy egy
 * FM-jel vivoje VONAL: nulla savszelessegu. Egy DC-szivo barmilyen lassu
 * is, 0 Hz-en VEGTELEN csillapitasa van — ha a vivo epp oda esik, teljesen
 * kiszedi alola. A lassitas csak annyit er, hogy tovabb tart, amig
 * megteszi.
 *
 * Meres (sim_fg23.py, 1 kHz audio, 3 kHz loket, i8, 2.5 s):
 *
 *   DC_SHIFT   hangolasi hiba 0 Hz-nel:   THD
 *      10                                16.2 %
 *      14                                 8.3 %
 *       0 (nincs DC-blokk)                0.04 %
 *
 *   DC_SHIFT 14, kulonbozo hangolasi hibaval:
 *        0 Hz   8.3 %      2000 Hz  24.8 %
 *      500 Hz   0.08 %     2500 Hz   0.09 %
 *     1000 Hz  13.2 %      5500 Hz   0.32 %
 *
 * A minta egyertelmu: ahanyszor egy FM-oldalsav PONT 0 Hz-re esik (a tiszta
 * 1 kHz-es hangnal minden egesz kHz-es eltolas ilyen), a szivo kilyukasztja
 * a spektrumot, es a demodulalt hang szetesik. Valos beszednel/AFSK-nal az
 * energia szet van kenve, tehat enyhebb — de a VIVO vonala mindig ott van.
 *
 * KOVETKEZMENY, AMI ELSORE FURCSA: minel pontosabban kalibralod a
 * kristalyt, annal inkabb 0 Hz-re kerul a vivo, es ANNAL rosszabb lesz az
 * NFM-hang. Ez az a fajta hiba, amit tapogatozva sosem talalnal meg.
 *
 * A VALODI MEGOLDAS kesobbre: szandekos ALACSONY IF. Hangold a FG23-at par
 * kHz-cel melle, es a jel a kimeneti sav szelen ul, nem a kozepen — igy sem
 * a DC-szivo, sem a DC-tuske, sem az 1/f zaj nem er hozza. Cserebe az
 * aprs_rx-nek es a SpyServer kozepfrekvenciajanak is tudnia kell rola,
 * ezert ez nem egy soros valtoztatas. Addig a 14 a jobb alku: felezi a
 * kart, es semmibe nem kerul.
 *
 * Az akkumulator ezert int64: az atlagot 2^DC_SHIFT-szeresen tarolja,
 * es 2^20 * 2^14 = 2^34 mar nem fer el 32 biten. */
#define IQ_STREAM_DC_BLOCK 1
#define DC_SHIFT           14
#define DC_PRECLAMP        (1 << 20)

/* ---------------- RDY-VONAL (a vesztes megszuntetese by design) ------
 *
 * Az ESP32 a GPIO2-n jelzi, hogy TENYLEG van-e felfuzott SPI-tranzakcioja
 * (a driver post_setup/post_trans callbackjeibol, tehat hardveri teny).
 * Mi a CS lehuzasa ELOTT megnezzuk, es ha nincs, varunk. Igy blokk nem
 * veszhet el es nem is csuszhat pufferkornyit — legfeljebb kesik, amit a
 * sajat NBLK=6 pufferunk elnyel.
 *
 * MIERT VOLT EZ SZUKSEGES: 2026-08-03-ig vakon kuldtunk, es a meresek
 * szerint halozati terheles alatt ~7 blokk/s a rossz leiroba erkezett a
 * tuloldalon. Minden korabbi tunet (VESZT, HATRA, szetkent vivo, nema
 * APRS) errol az egy gyokerrol fakadt.
 *
 * BEKOTES:  FG23 PD2 / EXP 9  <-  ESP32 GPIO2   (GND mar kozos)
 *
 * Amig a drot nincs bekotve, hagyd 0-n: bekotetlen labbal a varakozas
 * minden blokknal timeoutolna, es a stream folyamatosan kesne. */
#define IQ_RDY_ENABLE     1      /* 2026-08-03: a drot bekotve es mukodik */

/* SZIGORU MOD. A 2026-08-03-i eles futas 8007 timeoutot mutatott, es a
 * megmaradt ritka kirandulasok (d=25) pontosan ezekhez kotodnek: timeout
 * utan MEGIS kuldtunk — a regi vak viselkedes —, es a blokk a tuloldalon
 * epp betoltetlen leiroba erkezett. Szigoru modban timeout utan NEM
 * kuldunk: a blokk marad, a kovetkezo pump ujraprobalja. A sajat NBLK=6
 * puffer ezt boven elnyeli; ha a tuloldal tartosan halott (pl. ujraindul),
 * a sajat blokk-eldobas szamlalo no — EGESZ blokkok vesznek el nalunk,
 * lathatoan, nem fel-blokkok csendben a tuloldalon. */
#define RDY_STRICT        1
#define RDY_PORT          gpioPortD
#define RDY_PIN           2
/* Ennyit varunk legfeljebb a RDY-ra. Egy blokk-periodus 5.1 ms, a sajat
 * pufferunk 6 melyseg — 3 ms varakozas boven elnyelodik. Ha lejar,
 * MEGIS elkuldjuk (a regi viselkedes), es szamoljuk: a szamlalobol
 * latszik, ha a tuloldal tartosan nem gyozi. */
#define RDY_TIMEOUT_US    3000u

/* ---------------- LABAK ---------------- */
#define SPI_MOSI_PORT  gpioPortC
#define SPI_MOSI_PIN   0      /* PC00 = EXP 10 -> GPIO6 : adat  */
#define SPI_CLK_PORT   gpioPortC
#define SPI_CLK_PIN    5      /* PC05 = EXP 15 -> GPIO5 : orajel */
#define SPI_CS_PORT    gpioPortA
#define SPI_CS_PIN     7      /* PA07 = EXP 13 -> GPIO7 : CS    */

/* Maximalis meghajtas + elmeredekseg a C porton. Tobb MHz-es orajelnel
 * ez mar szamit; portonkent hat, es mind a harom jel a C porton van. */
#define SPI_FAST_SLEW  1

/* CS setup / hold ido mikroszekundumban.
 *
 * Az ESP32 SPI slave-je nem tud azonnal reagalni a CS lehuzasara: a
 * hardvernek be kell toltenie a DMA-leirot, mielott az elso orajel-el
 * megerkezik. Espressif ezt kifejezetten emliti a slave dokumentaciojaban.
 * 10 us a 2080 us-os blokkido mellett 0.5% overhead — eszre sem vesszuk,
 * cserebe nem vesznek el tranzakciok. Ha minden stabil, le lehet vinni. */
#define SPI_CS_SETUP_US  10u
#define SPI_CS_HOLD_US    5u

/* CS-POLARITAS.
 * 0 = aktiv ALACSONY — ez az SPI szabvany, es az ESP32 slave ezt varja.
 * 1 = aktiv MAGAS — csak kiserlet, ha felmerul a polaritas-kevereses.
 *     (Ha az ESP aktiv-alacsonyt var es mi magasat adunk, akkor a CS
 *     gyakorlatilag vegig aktivnak latszik nala, es ontja a csonka
 *     tranzakciokat — pont ezt lattuk a lebego CS-nel.) */
#define SPI_CS_ACTIVE_HIGH  0

#if SPI_CS_ACTIVE_HIGH
#define CS_ASSERT()    GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN)
#define CS_RELEASE()   GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN)
#define CS_IDLE_LEVEL  0u
#else
#define CS_ASSERT()    GPIO_PinOutClear(SPI_CS_PORT, SPI_CS_PIN)
#define CS_RELEASE()   GPIO_PinOutSet(SPI_CS_PORT, SPI_CS_PIN)
#define CS_IDLE_LEVEL  1u
#endif

/* ================= BLOKK-FORMATUM ================= */

/* A fejlec 16 bajt, a payload 256 komplex minta (I elol, Q utana),
 * mindketto little-endian. Az LDMA byteSwap-pel tolja ki, igy a masik
 * oldal bajtpuffere BIT SZERINT azonos ezzel a memoriakeppel — nem kell
 * cserelgetni semmit. */
#define IQ_BLK_MAGIC   0x32425149u      /* 'I','Q','B','2' */

typedef struct {
  uint32_t magic;      /* IQ_BLK_MAGIC */
  uint32_t seq;        /* blokkonkent no — EBBOL SZAMOLHATO A VESZTES */
  uint16_t nsamp;      /* komplex mintak szama (BLK_SAMPLES) */
  uint16_t decim;      /* a teljes decimacio, hogy a vevo tudja a ratat */
  uint32_t fs_in_hz;   /* a chip bemeneti mintavetele */
} iq_blk_hdr_t;

typedef struct {
  iq_blk_hdr_t hdr;
  int16_t      iq[BLK_SAMPLES * 2];    /* I,Q,I,Q ... */
} iq_blk_t;

/* 4 bajtra igazitva: az LDMA felszavas atvitelt csinal. */
static iq_blk_t s_blk[NBLK] __attribute__((aligned(4)));

#define BLK_BYTES   (sizeof(iq_blk_t))          /* 1040 */
#define BLK_WORDS   (BLK_BYTES / 2u)            /*  520 halfword */

/* ================= ALLAPOT ================= */

static const uint8_t *s_fifo      = NULL;
static uint16_t       s_fifo_size = 0;
static volatile uint16_t s_rd     = 0;      /* sajat olvasoindex, bajt */

static volatile bool s_active = false;
static RAIL_Handle_t s_rail_for_restart = NULL;

/* A synth finom-offszete. A RAIL_StartRx NEM orzi meg, ezert MINDEN
 * ujrainditas utan vissza kell irni — kulonben a stream inditasa eldobja
 * a kristalykalibraciot es a hangolast. Az app.c allitja. */
static volatile int32_t s_offset_tick = 0;

void iq_stream_set_freq_tick(int32_t tick)
{
  s_offset_tick = tick;
  if (s_active && s_rail_for_restart != NULL) {
    RAIL_SetFreqOffset(s_rail_for_restart,
                       (RAIL_FrequencyOffset_t)s_offset_tick);
  }
}

/* Minden RAIL_StartRx utan EZT kell hivni, sose a csupasz StartRx-et. */
static void stream_start_rx(RAIL_Handle_t rail, uint16_t channel)
{
  RAIL_StartRx(rail, channel, NULL);
  RAIL_SetFreqOffset(rail, (RAIL_FrequencyOffset_t)s_offset_tick);
}
static uint16_t      s_channel_for_restart = 0;

/* --- KETFOKOZATU CIC, VEGIG int32 ---
 * Az elso valtozat int64-et hasznalt, ami -Og-vel olyan lassu, hogy
 * 1 Msps-en kiehezteti a fo ciklust. Ket kaszkadolt CIC int32-vel: az
 * ELSO fut a teljes ratan (R1=8, olcso), a masodik nyolcadratan.
 *
 * Az 1. fokozat MASODRENDU: a bemeneti ratan futo szakasz a szuk
 * keresztmetszet (a harmadrendu valtozat 1000 helyett csak 522 ksps-t
 * birt). Megengedheto, mert a vegso savunk nagyon keskeny a kozbenso
 * ratahoz kepest, igy az 1. fokozat alias-savjai a CIC nullaira esnek. */
#define CIC_R1        8u
#define CIC_SH1       6u                    /* 2*log2(8) */

static int32_t a1_i, a2_i, a1_q, a2_q;                  /* 1. integrator */
static int32_t b1_i, b2_i, b1_q, b2_q;                  /* 1. comb */
static int32_t c1_i, c2_i, c3_i, c1_q, c2_q, c3_q;      /* 2. integrator */
static int32_t d1_i, d2_i, d3_i, d1_q, d2_q, d3_q;      /* 2. comb */
#if IQ_STREAM_DC_BLOCK
static int64_t dc_i, dc_q;      /* 64 bit: 2^20 * 2^14 nem fer 32-re */
#endif

static uint32_t s_cnt1 = 0, s_cnt2 = 0;
static uint32_t s_decim_R  = 256;
static uint32_t s_R2       = 32;

/* 2^24-es fixpontos normalizalo: s_norm = 2^24 / R2^3. */
#define NORM_SHIFT  24
static int32_t  s_norm     = (1 << NORM_SHIFT) / 32768;

/* --- blokk-kezeles: EGY termelo (ISR) es EGY fogyaszto (fo ciklus),
 * ezert eleg ket monoton szamlalo. varakozo = s_prod - s_cons. --- */
static volatile uint32_t s_prod = 0;     /* hany blokk lett kesz */
static volatile uint32_t s_cons = 0;     /* hany blokk ment ki */
static volatile uint16_t s_fill_n = 0;   /* mintak az eppen toltott blokkban */
static uint32_t s_seq = 0;               /* kimeno sorszam */
static int16_t  s_rssi_dbm = -128;       /* legutobbi RAIL-RSSI (dBm), a fejlecbe */

/* Statisztika */
static volatile uint32_t s_out_samples = 0;
static volatile uint32_t s_blk_drops   = 0;   /* nem volt szabad blokk */
static uint32_t s_rdy_waits    = 0;   /* hanyszor kellett varni a RDY-ra */
static uint32_t s_rdy_timeouts = 0;   /* hanyszor jart le a turelem */
static uint32_t s_rdy_skips    = 0;   /* szigoru mod: elhalasztott kuldes */
static uint64_t s_rdy_wait_us  = 0;   /* osszes varakozas */
static volatile uint32_t s_fifo_ovf    = 0;
static volatile uint32_t s_clip        = 0;
static volatile uint32_t s_pump_calls  = 0;
static volatile uint32_t s_isr_events  = 0;
static volatile bool     s_need_restart = false;
static RAIL_Time_t s_t_start = 0;

/* ============ A BEMENETI RATA MERESE, NEM FELTETELEZESE ============
 *
 * Az IQ_STREAM_FS_IN_HZ egy DEFINE volt, ami a Radio Configuratorban
 * beallitott akvizicios savszelessegbol kovetkezo mintavetelt tukrozte.
 * Ez csendes hiba forrasa: ha a PHY-t atallitod (pl. az akvizicios sav
 * 100 kHz-re csuszik), a define ottmarad, es onnantol MINDEN szam hazudik
 * — az SDR++ rossz ratat kap, a hang rossz magassagon szol, a demodulator
 * rossz szuroket szamol. Semmi nem hibazik lathatoan, csak minden melle megy.
 *
 * Ezert MERJUK: szamoljuk a FIFO-bol beolvasott mintakat es az idot. */
/* MERESI ABLAK. 2 masodperc: eleg hosszu ahhoz, hogy a meres zaja
 * elhanyagolhato legyen. */
#define FS_MEAS_MIN_US     2000000u

/* HOLTSAV ezrelekben. Csak ennel nagyobb elteresre valtunk erteket.
 *
 * MIERT KELL: az elso valtozat a mert erteket a 39 MHz / N racsra
 * igazitotta, mert "a rata kvantalt". A valosag: 39e6 / 400000 = 97.5 —
 * a tenyleges rata PONT ket racspont kozott van, es a kerekites 97 es 98
 * kozott ugralt (397959 <-> 402061). Minden ugras egy teljes
 * ujrakonfiguralast valtott ki a lanc mindket vegen, plusz ket naplosort
 * — masodpercenkent tizszer. Ebbol lett a blokkvesztes.
 *
 * Tanulsag: ne igazits racsra, amirol nem tudod BIZTOSAN, hogy a jel rajta
 * van. Egy holtsav ugyanazt a stabilitast adja feltevesek nelkul. */
#define FS_DEADBAND_PPT    10u        /* 1% */

static volatile uint32_t s_fs_in_hz  = IQ_STREAM_FS_IN_HZ;
static uint32_t          s_fs_cnt    = 0;      /* komplex mintak az ablakban */
static RAIL_Time_t       s_fs_t0     = 0;
static volatile uint32_t s_fs_report = 0;      /* !=0 -> a fo ciklus kiirja */

/* FIGYELEM: EZ ISR-BOL FUT (iq_stream_on_event).
 * SEMMILYEN printf, semmilyen blokkolo muvelet nem lehet benne. Az elso
 * valtozatban itt volt egy printf a VCOM-ra — 115200-on ~6 ms blokkolas a
 * radio megszakitasaban, masodpercenkent tobbszor. A kiiras ezert csak
 * jelzest hagy, es a fo ciklus vegzi el. */
static void fs_account(uint32_t nsamples)
{
  RAIL_Time_t now = RAIL_GetTime();
  if (s_fs_t0 == 0) { s_fs_t0 = now; s_fs_cnt = 0; return; }

  s_fs_cnt += nsamples;
  uint32_t dt = (uint32_t)(now - s_fs_t0);
  if (dt < FS_MEAS_MIN_US) return;

  uint32_t meas = (uint32_t)(((uint64_t)s_fs_cnt * 1000000ull) / dt);
  s_fs_t0  = now;
  s_fs_cnt = 0;
  if (meas < 1000u) return;

  uint32_t cur  = s_fs_in_hz;
  uint32_t diff = (meas > cur) ? (meas - cur) : (cur - meas);
  if (diff * 1000u > cur * FS_DEADBAND_PPT) {
    s_fs_in_hz  = meas;
    s_fs_report = meas;            /* a fo ciklus kiirja */
  }
}


/* --- CS-idozites merese ---
 * A CS-nek a blokk kuldesi idejeig (2.08 ms @ 4 MHz) kellene lent lennie.
 * Ha tovabb marad, a masik oldal a kovetkezo tranzakciot mar aktiv CS-sel
 * kapja, es az azonnal, uresen lezarul. Ezert megmerjuk, MELYIK feltetel
 * kesik: az LDMA-kesz, vagy a TXC. */
static volatile uint32_t s_cs_t0      = 0;   /* CS lehuzas idopontja */
static volatile uint32_t s_cs_t_ldma  = 0;   /* amikor az LDMA kesz lett */
static volatile bool     s_ldma_seen  = false;
static volatile uint64_t s_sum_ldma_us = 0;  /* CS-le -> LDMA kesz */
static volatile uint64_t s_sum_txc_us  = 0;  /* LDMA kesz -> TXC */
static volatile uint32_t s_max_cs_us   = 0;
static volatile uint32_t s_cs_meas     = 0;

/* --- A CS LAB TENYLEGES SZINTJE ---
 * A GPIO_PinInGet a PADOT olvassa, nem a kimeneti regisztert. Ha valami
 * kulso elhuzza a vonalat, a beolvasott szint eltér attol, amit kiirtunk.
 * A pumpabol mintavetelezunk (~48 kHz, azaz blokként ~1000 minta), igy a
 * TENYLEGES kitoltes es az esetleges utkozes is merheto — fuggetlenul
 * attol, mit mutat a multimeter. */
static volatile uint32_t s_pad_samples  = 0;
static volatile uint32_t s_pad_high     = 0;
static volatile uint32_t s_pad_mismatch = 0;

/* LDMA */
static LDMA_Descriptor_t s_desc;
static unsigned int      s_dma_ch = 0;
static bool              s_dma_alloc = false;
static volatile bool     s_tx_busy = false;

/* ================= SPI ================= */

static void spi_setup(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_USART0, true);

#if SPI_FAST_SLEW
  /* A SiSDK 2025.6 emlibje mar nem adja a GPIO_DriveStrengthSet()-et,
   * ezert kozvetlen regiszterires. SLEWRATE 0..7 (alap 4), a 7 a
   * legmeredekebb; DRIVESTRENGTH 0 = STRONG. */
  /* A SCLK es a MOSI a C porton van, a CS az A-n — mindkettot allitjuk. */
  {
    static const GPIO_Port_TypeDef ports[2] = { gpioPortC, gpioPortA };
    for (int pi = 0; pi < 2; pi++) {
      uint32_t ctrl = GPIO->P[ports[pi]].CTRL;
      ctrl &= ~(_GPIO_P_CTRL_SLEWRATE_MASK | _GPIO_P_CTRL_SLEWRATEALT_MASK);
      ctrl |= (7u << _GPIO_P_CTRL_SLEWRATE_SHIFT)
            | (7u << _GPIO_P_CTRL_SLEWRATEALT_SHIFT);
#if defined(_GPIO_P_CTRL_DRIVESTRENGTH_MASK)
      ctrl &= ~_GPIO_P_CTRL_DRIVESTRENGTH_MASK;
#endif
#if defined(_GPIO_P_CTRL_DRIVESTRENGTHALT_MASK)
      ctrl &= ~_GPIO_P_CTRL_DRIVESTRENGTHALT_MASK;
#endif
      GPIO->P[ports[pi]].CTRL = ctrl;
    }
  }
#endif

  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CLK_PORT,  SPI_CLK_PIN,  gpioModePushPull, 0);
  /* A CS-t MI hajtjuk, sima GPIO-kent. */
  GPIO_PinModeSet(SPI_CS_PORT,   SPI_CS_PIN,   gpioModePushPull, CS_IDLE_LEVEL);

  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
  init.enable       = usartEnableTx;      /* csak adunk */
  init.baudrate     = SPI_HZ;
  init.databits     = usartDatabits16;    /* 16 bites keret, mint eddig */
  init.master       = true;
  init.msbf         = true;               /* MSB elol — SPI szokas */
  init.clockMode    = usartClockMode0;    /* CPOL=0, CPHA=0 = SPI mode 0 */
  init.autoCsEnable = false;              /* !!! a CS a mienk, GPIO */
  init.autoTx       = false;

  USART_InitSync(USART0, &init);

  /* Labkiosztas: CSAK TX es CLK. A CSROUTE/CSPEN SZANDEKOSAN kimarad —
   * kulonben a periferia is billegtetne a CS-t, es elutne a kezi
   * blokk-keretezessel. */
  GPIO->USARTROUTE[0].TXROUTE =
      ((uint32_t)SPI_MOSI_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
    | ((uint32_t)SPI_MOSI_PIN  << _GPIO_USART_TXROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[0].CLKROUTE =
      ((uint32_t)SPI_CLK_PORT << _GPIO_USART_CLKROUTE_PORT_SHIFT)
    | ((uint32_t)SPI_CLK_PIN  << _GPIO_USART_CLKROUTE_PIN_SHIFT);
  GPIO->USARTROUTE[0].ROUTEEN =
      GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_CLKPEN;

  printf("# SPI: %lu Hz, mode 0, 16 bit, MSB elol\r\n",
         (unsigned long)SPI_HZ);
  printf("# SPI labak: MOSI=PC%02u/EXP10  SCLK=PC%02u/EXP15  "
         "CS=PA%02u/EXP13 (kezi)\r\n",
         (unsigned)SPI_MOSI_PIN, (unsigned)SPI_CLK_PIN, (unsigned)SPI_CS_PIN);
  printf("# blokk: %lu bajt (%lu minta), ~%lu us kuldesi ido\r\n",
         (unsigned long)BLK_BYTES, (unsigned long)BLK_SAMPLES,
         (unsigned long)((uint64_t)BLK_BYTES * 8ull * 1000000ull / SPI_HZ));
}

static void spi_teardown(void)
{
  if (s_dma_alloc) LDMA_StopTransfer((int)s_dma_ch);
  s_tx_busy = false;
  GPIO->USARTROUTE[0].ROUTEEN = 0;
  USART_Reset(USART0);
  /* A CS a route lekapcsolasa UTAN is maradjon hajtott magas — kulonben
   * a masik oldal megint lebego bemenetet lat. */
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, CS_IDLE_LEVEL);
}

/* Egy blokk kitolasa: CS le, LDMA inditas. A CS felvitele a pump-ban
 * tortenik, amikor az LDMA vegzett ES a shiftregiszter is kiurult. */
static void spi_send_block(iq_blk_t *b)
{
  if (!s_dma_alloc) {
    DMADRV_Init();
    if (DMADRV_AllocateChannel(&s_dma_ch, NULL) != ECODE_EMDRV_DMADRV_OK) {
      printf("# LDMA csatorna nem kaphato!\r\n");
      return;
    }
    s_dma_alloc = true;
  }

  s_desc.xfer.structType  = ldmaCtrlStructTypeXfer;
  s_desc.xfer.structReq   = 0;
  s_desc.xfer.xferCnt     = BLK_WORDS - 1u;
  /* byteSwap: a felszo ket bajtjat megcsereli kikuldes elott. Igy a masik
   * oldal bajtpuffere BIT SZERINT azonos lesz ezzel a memoriakeppel, es
   * ott nem kell semmit forgatni. (A USART MSB-first tol ki.) */
  s_desc.xfer.byteSwap    = 1;
  s_desc.xfer.blockSize   = ldmaCtrlBlockSizeUnit1;
  s_desc.xfer.doneIfs     = 0;
  s_desc.xfer.reqMode     = ldmaCtrlReqModeBlock;
  s_desc.xfer.decLoopCnt  = 0;
  s_desc.xfer.ignoreSrec  = 0;
  s_desc.xfer.srcInc      = ldmaCtrlSrcIncOne;
  s_desc.xfer.size        = ldmaCtrlSizeHalf;
  s_desc.xfer.dstInc      = ldmaCtrlDstIncNone;
  s_desc.xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
  s_desc.xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
  s_desc.xfer.srcAddr     = (uint32_t)b;
  s_desc.xfer.dstAddr     = (uint32_t)&USART0->TXDOUBLE;
  s_desc.xfer.linkMode    = ldmaLinkModeAbs;
  s_desc.xfer.link        = 0;
  s_desc.xfer.linkAddr    = 0;

  LDMA_TransferCfg_t cfg =
      LDMA_TRANSFER_CFG_PERIPHERAL(ldmaPeripheralSignal_USART0_TXBL);

#if IQ_RDY_ENABLE
  /* A tuloldal keszen all? A varakozas itt, a CS lehuzasa ELOTT tortenik
   * — a blokk addig a mienk, semmi nem veszhet el. */
  if (!GPIO_PinInGet(RDY_PORT, RDY_PIN)) {
    RAIL_Time_t t0 = RAIL_GetTime();
    RAIL_Time_t tl = t0 + RDY_TIMEOUT_US;
    bool timed_out = false;
    s_rdy_waits++;
    while (!GPIO_PinInGet(RDY_PORT, RDY_PIN)) {
      if ((int32_t)(RAIL_GetTime() - tl) >= 0) {
        s_rdy_timeouts++;
        timed_out = true;
        break;
      }
    }
    s_rdy_wait_us += (uint64_t)(RAIL_GetTime() - t0);
#if RDY_STRICT
    if (timed_out) {
      /* NEM kuldunk vakon. A blokk a mienk marad (s_cons nem lepett,
       * s_tx_busy hamis), a kovetkezo pump ujraprobalja. */
      s_rdy_skips++;
      return;
    }
#else
    (void)timed_out;
#endif
  }
#endif

  /* Most mar biztosan kuldunk (a strict-timeout ag fentebb return-olt). A
   * sorszamot CSAK ITT irjuk — igy egy RDY-timeout nem eget el sorszamot,
   * es nem keletkezik fantomlyuk az ESP oldalon. A DMA a CS_ASSERT utan
   * indul, tehat a most beirt seq-et viszi ki. */
  b->hdr.seq = s_seq++;

  CS_ASSERT();                                  /* CS aktiv */

  /* CS SETUP IDO. Az ESP32 SPI slave-jenek kell nehany mikroszekundum a
   * CS lehuzasa utan, mielott megindul az orajel — a hardver ekkor tolti
   * be a DMA-leirot. Ha az elso ora tul hamar jon, a tranzakcio elveszik
   * vagy csonka lesz. Nalunk a CS lehuzasa es az LDMA inditasa kozott
   * kulonben csak 1-2 us lenne. */
  {
    RAIL_Time_t t = RAIL_GetTime() + SPI_CS_SETUP_US;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }
  }

  LDMA_StartTransfer((int)s_dma_ch, &cfg, &s_desc);
  s_cs_t0     = (uint32_t)RAIL_GetTime();
  s_ldma_seen = false;
  s_tx_busy   = true;
}

/* ================= INIT ================= */

void iq_stream_init(const uint8_t *fifo_base, uint16_t fifo_bytes)
{
  s_fifo = fifo_base;
  s_fifo_size = fifo_bytes;

  /* A CS-t AZONNAL inaktivba (magas) hajtjuk, mar bekapcsolaskor.
   *
   * MIERT: reset utan a PC03 letiltott bemenet, tehat a masik oldal
   * CS-bemenete LEBEG. Egy SPI slave minden zajelre lezar egy
   * tranzakciot — meresve ~4200 csonka tranzakcio masodpercenkent,
   * mielott barmit is inditottunk volna. Egy push-pull magas szint
   * ezt teljesen megszunteti. */
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SPI_CS_PORT, SPI_CS_PIN, gpioModePushPull, CS_IDLE_LEVEL);
#if IQ_RDY_ENABLE
  /* Bemenet LEHUZASSAL: ha a drot leesik, a RDY tartosan 0-nak latszik,
   * a timeout-szamlalo azonnal elarulja — nem nema hiba. */
  GPIO_PinModeSet(RDY_PORT, RDY_PIN, gpioModeInputPull, 0);
#endif
}

bool iq_stream_active(void) { return s_active; }

uint32_t iq_stream_out_sps(uint16_t decim)
{
  uint32_t r = (decim < CIC_R1) ? CIC_R1 : (uint32_t)decim;
  r = (r / CIC_R1) * CIC_R1;
  if (r == 0u) r = CIC_R1;
  /* A MERT ratabol, nem a define-bol. Amig nincs meres, a define az
   * alapertelmezes. */
  return s_fs_in_hz / r;
}

/* ================= MINTA -> BLOKK ================= */

static inline void blk_put_sample(int16_t vi, int16_t vq)
{
  if ((s_prod - s_cons) >= NBLK) {   /* nincs szabad blokk */
    s_blk_drops++;
    return;
  }

  iq_blk_t *b = &s_blk[s_prod % NBLK];
  uint16_t n = s_fill_n;
  b->iq[n * 2u + 0u] = vi;           /* I elol */
  b->iq[n * 2u + 1u] = vq;
  n++;

  if (n >= BLK_SAMPLES) {
    s_fill_n = 0;
    s_prod++;                        /* EZ teszi kuldhetove */
  } else {
    s_fill_n = n;
  }
}

/* ================= CIC ================= */

static void process_block(const uint8_t *p, uint16_t nbytes)
{
  typedef struct { int16_t q, i; } iq_in_t;   /* BUFC sorrend: Q ELOL */
  uint16_t nsamp = (uint16_t)(nbytes / sizeof(iq_in_t));

  int32_t A1i = a1_i, A2i = a2_i;
  int32_t A1q = a1_q, A2q = a2_q;
  uint32_t cnt1 = s_cnt1;

  /* EGY 32 BITES OLVASAS negy bajtolvasas helyett. A FIFO 4 bajtos
   * igazitasban all es az indexek is 4-gyel oszthatok, ezert biztonsagos. */
  const uint32_t *w = (const uint32_t *)(const void *)p;

  for (uint16_t k = 0; k < nsamp; k++) {
    uint32_t v = *w++;
    int16_t q = (int16_t)(uint16_t)(v & 0xFFFFu);        /* Q ELOL */
    int16_t i = (int16_t)(uint16_t)(v >> 16);

    A1i += i;  A2i += A1i;
    A1q += q;  A2q += A1q;

    if (++cnt1 < CIC_R1) continue;
    cnt1 = 0;

    int32_t e, f;
    e = A2i - b1_i;  b1_i = A2i;
    f = e    - b2_i;  b2_i = e;
    int32_t s1i = f >> CIC_SH1;

    e = A2q - b1_q;  b1_q = A2q;
    f = e    - b2_q;  b2_q = e;
    int32_t s1q = f >> CIC_SH1;

    c1_i += s1i;  c2_i += c1_i;  c3_i += c2_i;
    c1_q += s1q;  c2_q += c1_q;  c3_q += c2_q;

    if (++s_cnt2 < s_R2) continue;
    s_cnt2 = 0;

    int32_t g;
    e = c3_i - d1_i;  d1_i = c3_i;
    f = e     - d2_i;  d2_i = e;
    g = f     - d3_i;  d3_i = f;
    int32_t oi = (int32_t)(((int64_t)g * s_norm) >> NORM_SHIFT);

    e = c3_q - d1_q;  d1_q = c3_q;
    f = e     - d2_q;  d2_q = e;
    g = f     - d3_q;  d3_q = f;
    int32_t oq = (int32_t)(((int64_t)g * s_norm) >> NORM_SHIFT);

#if IQ_STREAM_DC_BLOCK
    if (oi >  DC_PRECLAMP) oi =  DC_PRECLAMP;
    else if (oi < -DC_PRECLAMP) oi = -DC_PRECLAMP;
    if (oq >  DC_PRECLAMP) oq =  DC_PRECLAMP;
    else if (oq < -DC_PRECLAMP) oq = -DC_PRECLAMP;

    dc_i += (int64_t)oi - (dc_i >> DC_SHIFT);
    oi   -= (int32_t)(dc_i >> DC_SHIFT);
    dc_q += (int64_t)oq - (dc_q >> DC_SHIFT);
    oq   -= (int32_t)(dc_q >> DC_SHIFT);
#endif

    /* Levagas +-32767-re, NEM -32768-ra: a 0x8000 az egyetlen 16 bites
     * ertek, amibol egy egybites csuszas pontos nullat csinal. Egy LSB
     * ara, cserebe az a csapda vegleg megszunik. */
    if (oi >  32767) { oi =  32767; s_clip++; }
    else if (oi < -32767) { oi = -32767; s_clip++; }
    if (oq >  32767) { oq =  32767; s_clip++; }
    else if (oq < -32767) { oq = -32767; s_clip++; }

    blk_put_sample((int16_t)oi, (int16_t)oq);
    s_out_samples++;
  }

  a1_i = A1i; a2_i = A2i;
  a1_q = A1q; a2_q = A2q;
  s_cnt1 = cnt1;
}

/* ================= RAIL ESEMENY ================= */

bool iq_stream_on_event(RAIL_Handle_t rail, RAIL_Events_t events)
{
  if (!s_active) return false;

  if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
    s_fifo_ovf++;
    /* CSAK jelzes. RAIL_ResetFifo-t futo RX mellett hivni megoli a
     * vetelt — az ujrainditas a fo ciklusban tortenik. */
    s_need_restart = true;
    return true;
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    s_isr_events++;

    uint8_t guard = 8u;
    uint16_t avail = RAIL_GetRxFifoBytesAvailable(rail);

    while (avail >= 512u && guard--) {
      uint16_t n = (uint16_t)(avail & ~3u);
      if (n > 4096u) n = 4096u;
      if (n == 0u) break;

      /* HELYBEN olvasunk a FIFO-bol (zero-copy), korbefordulassal. */
      uint16_t first = (uint16_t)(s_fifo_size - s_rd);
      if (first > n) first = n;
      process_block(&s_fifo[s_rd], first);
      if (n > first) process_block(&s_fifo[0], (uint16_t)(n - first));

      RAIL_ReadRxFifo(rail, NULL, n);      /* csak a mutato lep */
      s_rd = (uint16_t)((s_rd + n) % s_fifo_size);

      /* A TENYLEGES bemeneti rata merese. Innen tudja a lanc tobbi resze
       * (SDR++, demodulator), hany mintat kap masodpercenkent — nem egy
       * define-bol, ami elavulhat. 4 bajt = egy komplex minta. */
      fs_account((uint32_t)(n >> 2));

      avail = RAIL_GetRxFifoBytesAvailable(rail);
    }
  }

  return true;
}

/* ================= INDIT / LEALLIT ================= */

void iq_stream_start(RAIL_Handle_t rail, uint16_t channel,
                     uint16_t decim, uint8_t out_shift)
{
  if (decim < CIC_R1) decim = CIC_R1;
  if (decim > 4096u)  decim = 4096u;
  decim = (uint16_t)((decim / CIC_R1) * CIC_R1);

  s_decim_R = decim;
  s_R2      = decim / CIC_R1;
  if (s_R2 < 1u) s_R2 = 1u;

  uint64_t gain = (uint64_t)s_R2 * s_R2 * s_R2;
  s_norm = (int32_t)(((1ull << NORM_SHIFT) + gain / 2ull) / gain);
  if (s_norm < 1) s_norm = 1;
  if (out_shift < 8u) s_norm = (int32_t)(s_norm << out_shift);

  a1_i = a2_i = a1_q = a2_q = 0;
  b1_i = b2_i = b1_q = b2_q = 0;
  c1_i = c2_i = c3_i = c1_q = c2_q = c3_q = 0;
  d1_i = d2_i = d3_i = d1_q = d2_q = d3_q = 0;
#if IQ_STREAM_DC_BLOCK
  dc_i = dc_q = 0;
#endif
  s_cnt1 = s_cnt2 = 0;
  s_rdy_waits = 0; s_rdy_timeouts = 0; s_rdy_wait_us = 0; s_rdy_skips = 0;
  s_prod = s_cons = 0; s_fill_n = 0; s_seq = 0;
  s_out_samples = 0; s_blk_drops = 0; s_fifo_ovf = 0; s_clip = 0;
  s_pump_calls = 0; s_isr_events = 0; s_need_restart = false;
  s_tx_busy = false;
  s_sum_ldma_us = 0; s_sum_txc_us = 0; s_max_cs_us = 0; s_cs_meas = 0;
  s_pad_samples = 0; s_pad_high = 0; s_pad_mismatch = 0;
  memset(s_blk, 0, sizeof(s_blk));

  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(rail, false, true);
  s_rd = 0;

  RAIL_SetRxFifoThreshold(rail, 2048u);
  RAIL_ConfigEvents(rail, RAIL_EVENTS_ALL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL
                    | RAIL_EVENT_RX_FIFO_OVERFLOW);

  spi_setup();

  {
    uint32_t sps = iq_stream_out_sps((uint16_t)s_decim_R);
    printf("# kimenet: R=%lu -> %lu sps, %lu B/s, blokk %lu ms-onkent\r\n",
           (unsigned long)s_decim_R, (unsigned long)sps,
           (unsigned long)(sps * 4u),
           (unsigned long)(sps ? (BLK_SAMPLES * 1000u / sps) : 0u));
  }
#if IQ_STREAM_DC_BLOCK
  printf("# DC-blokk BE (shift=%u, idoallando ~%lu minta)\r\n",
         (unsigned)DC_SHIFT, (unsigned long)(1ul << DC_SHIFT));
#endif

  s_rail_for_restart = rail;
  s_channel_for_restart = channel;
  s_t_start = RAIL_GetTime();
  s_fs_t0 = 0; s_fs_cnt = 0;      /* uj meresi ablak */
  s_active = true;
  stream_start_rx(rail, channel);
}

void iq_stream_stop(RAIL_Handle_t rail, uint16_t channel)
{
  RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
  s_active = false;
  spi_teardown();

  uint32_t dur_us = (uint32_t)(RAIL_GetTime() - s_t_start);
  uint32_t rate = 0;
  if (dur_us > 0u) {
    rate = (uint32_t)(((uint64_t)s_out_samples * 1000000ull) / dur_us);
  }

  printf("\r\n# stream leallt: %lu kimeneti minta, %lu.%03lu ksps\r\n",
         (unsigned long)s_out_samples,
         (unsigned long)(rate / 1000u), (unsigned long)(rate % 1000u));
  printf("# kikuldott blokk: %lu   blokk-eldobas: %lu   "
         "FIFO-overflow: %lu\r\n",
         (unsigned long)s_cons, (unsigned long)s_blk_drops,
         (unsigned long)s_fifo_ovf);
#if IQ_RDY_ENABLE
  printf("# RDY: %lu varakozas (atlag %lu us), %lu timeout, "
         "%lu halasztott kuldes\r\n",
         (unsigned long)s_rdy_waits,
         (unsigned long)(s_rdy_waits ? s_rdy_wait_us / s_rdy_waits : 0),
         (unsigned long)s_rdy_timeouts, (unsigned long)s_rdy_skips);
#endif
  printf("# fo ciklus: %lu pump/s   ISR-esemeny: %lu\r\n",
         (unsigned long)(dur_us ? (uint32_t)(((uint64_t)s_pump_calls
                          * 1000000ull) / dur_us) : 0u),
         (unsigned long)s_isr_events);

  {
    uint32_t promille = (s_out_samples > 0u)
        ? (uint32_t)(((uint64_t)s_clip * 1000ull) / (s_out_samples * 2ull))
        : 0u;
    printf("# levagas: %lu ertek (%lu.%01lu%%)%s\r\n",
           (unsigned long)s_clip,
           (unsigned long)(promille / 10u), (unsigned long)(promille % 10u),
           (s_clip > 0u) ? "  <-- TELITES" : "");
  }

  /* Mi van az utoljara kitoltott blokkban? Ez valasztja szet a DSP-t es
   * a linket: ha itt van adat de a masik oldalon nincs, a link a hibas. */
  {
    const iq_blk_t *b = &s_blk[(s_prod ? (s_prod - 1u) : 0u) % NBLK];
    int32_t peak = 0;
    uint32_t nonzero = 0;
    for (uint32_t k = 0; k < BLK_SAMPLES * 2u; k++) {
      int16_t v = b->iq[k];
      if (v != 0) nonzero++;
      int32_t a = (v < 0) ? -(int32_t)v : (int32_t)v;
      if (a > peak) peak = a;
    }
    printf("# utolso blokk: csucs=%ld  nem-nulla=%lu / %lu\r\n",
           (long)peak, (unsigned long)nonzero,
           (unsigned long)(BLK_SAMPLES * 2u));
    printf("# elso 8 szo (I Q I Q ...):");
    for (int k = 0; k < 8; k++) printf(" %04X", (unsigned)(uint16_t)b->iq[k]);
    printf("\r\n");
  }

  /* --- CS-idozites: EZ mondja meg, hol vesz el az ido --- */
  if (s_cs_meas > 0u) {
    uint32_t a_ldma = (uint32_t)(s_sum_ldma_us / s_cs_meas);
    uint32_t a_txc  = (uint32_t)(s_sum_txc_us  / s_cs_meas);
    uint32_t elm    = (uint32_t)((uint64_t)BLK_BYTES * 8ull
                                 * 1000000ull / SPI_HZ);
    printf("# CS lent: atlag %lu us (elmeleti %lu us), max %lu us\r\n",
           (unsigned long)(a_ldma + a_txc), (unsigned long)elm,
           (unsigned long)s_max_cs_us);
    printf("#   ebbol CS-le -> LDMA kesz : %lu us\r\n",
           (unsigned long)a_ldma);
    printf("#   ebbol LDMA kesz -> TXC   : %lu us %s\r\n",
           (unsigned long)a_txc,
           (a_txc > 200u) ? "  <-- ITT VESZ EL AZ IDO" : "");
  }

  /* --- A CS LAB TENYLEGES KITOLTESE ---
   * Ezt a lab olvasasabol kapjuk, tehat fuggetlen a multimetertol ES a
   * kimeneti regisztertol. Ha az "eltérés" nem nulla, valami kulso
   * huzza a vonalat. */
  if (s_pad_samples > 0u) {
    uint32_t hi_pm  = (uint32_t)(((uint64_t)s_pad_high * 1000ull)
                                 / s_pad_samples);
    uint32_t exp_pm = 1000u;
    if (s_cs_meas > 0u && s_cons > 0u) {
      uint32_t low_us_avg = (uint32_t)((s_sum_ldma_us + s_sum_txc_us)
                                       / s_cs_meas);
      uint32_t per_us = (uint32_t)(dur_us / (s_cons ? s_cons : 1u));
      if (per_us > 0u) {
        uint32_t low_pm = (uint32_t)(((uint64_t)low_us_avg * 1000ull)
                                     / per_us);
        exp_pm = (low_pm < 1000u) ? (1000u - low_pm) : 0u;
      }
    }
#if SPI_CS_ACTIVE_HIGH
    exp_pm = 1000u - exp_pm;
#endif
    printf("# CS pad: magas %lu.%01lu%%  (a kiirt szintbol varhato "
           "%lu.%01lu%%)\r\n",
           (unsigned long)(hi_pm / 10u), (unsigned long)(hi_pm % 10u),
           (unsigned long)(exp_pm / 10u), (unsigned long)(exp_pm % 10u));
    printf("# CS pad eltéres a kiirt szinttol: %lu / %lu minta%s\r\n",
           (unsigned long)s_pad_mismatch, (unsigned long)s_pad_samples,
           (s_pad_mismatch > (s_pad_samples / 100u))
             ? "   <-- VALAMI KULSO HUZZA A VONALAT!" : "   (tiszta)");
  }

  if (s_blk_drops > 0u) {
    printf("# -> a link nem viszi el: emeld az SPI_HZ-t vagy a decimaciot\r\n");
  }

  RAIL_ResetFifo(rail, false, true);
  stream_start_rx(rail, channel);
}

/* ================= FO CIKLUS: BLOKK -> SPI ================= */

/* A folyamatban levo SPI-kuldes lezarasa: LDMA kesz ES TXC, majd CS fel.
   Visszaad: true, amig a kuldes meg tart (a hivo ne inditson ujat).
   Kozos a normal pumpnak es az ext-modnak (scan.c). */
static bool spi_tx_poll(void)
{
  if (!s_tx_busy) return false;

  /* KETTOS FELTETEL. Az LDMA "kesz" azt jelenti, hogy az utolso szo
   * BEKERULT a TX FIFO-ba — nem azt, hogy ki is ment a vonalon. Ha itt
   * vinnenk fel a CS-t, az utolso par bajt a levegoben maradna. Ezert
   * kell melle a TXC (transmit complete) is. */
  /* Elso feltetel: az LDMA vegzett (az utolso szo bekerult a FIFO-ba). */
  if (!s_ldma_seen && LDMA_TransferDone((int)s_dma_ch)) {
    s_ldma_seen  = true;
    s_cs_t_ldma  = (uint32_t)RAIL_GetTime();
  }

  /* Masodik feltetel: a shiftregiszter is kiurult. */
  if (s_ldma_seen && (USART0->STATUS & USART_STATUS_TXC)) {
    /* CS HOLD IDO: az utolso orajel-el utan is hagyunk egy kis idot,
     * mielott felvinnenk a CS-t. Igy a masik oldalnak biztosan van
     * ideje az utolso bitet beorazni. */
    RAIL_Time_t t = RAIL_GetTime() + SPI_CS_HOLD_US;
    while ((int32_t)(RAIL_GetTime() - t) < 0) { }

    CS_RELEASE();                              /* CS inaktiv */

    uint32_t t_end = (uint32_t)RAIL_GetTime();
    uint32_t d_all  = t_end - s_cs_t0;
    s_sum_ldma_us += (uint64_t)(s_cs_t_ldma - s_cs_t0);
    s_sum_txc_us  += (uint64_t)(t_end - s_cs_t_ldma);
    if (d_all > s_max_cs_us) s_max_cs_us = d_all;
    s_cs_meas++;

    s_cons++;
    s_tx_busy = false;
  }
  return s_tx_busy;
}

bool iq_stream_pump(void)
{
  /* A merestol jott uj rata kiirasa — ITT, a fo ciklusban, nem az ISR-ben. */
  if (s_fs_report) {
    uint32_t v = s_fs_report;
    s_fs_report = 0;
    printf("# mert bemeneti rata: %lu sps (kimenet %lu sps)\r\n",
           (unsigned long)v, (unsigned long)(v / (s_decim_R ? s_decim_R : 1u)));
  }

  if (!s_active) return false;
  s_pump_calls++;

  /* A CS PAD tenyleges szintje — nem a kimeneti regiszter! Ha valami
   * kulso elhuzza a vonalat, az itt jon ki. */
  {
    unsigned pad  = GPIO_PinInGet(SPI_CS_PORT, SPI_CS_PIN) ? 1u : 0u;
    unsigned want = s_tx_busy ? (CS_IDLE_LEVEL ^ 1u) : CS_IDLE_LEVEL;
    s_pad_samples++;
    if (pad) s_pad_high++;
    if (pad != want) s_pad_mismatch++;
  }

  if (s_need_restart) {
    s_need_restart = false;
    RAIL_Handle_t rail = s_rail_for_restart;
    if (rail != NULL) {
      RAIL_Idle(rail, RAIL_IDLE_ABORT, true);
      RAIL_ResetFifo(rail, false, true);
      s_rd = 0;
      stream_start_rx(rail, s_channel_for_restart);
    }
  }

  /* --- fut egy kuldes? nezzuk meg, vege van-e --- */
  if (spi_tx_poll()) return true;

  /* --- van kuldheto blokk? --- */
  if ((s_prod - s_cons) > 0u) {
    iq_blk_t *b = &s_blk[s_cons % NBLK];
    b->hdr.magic    = IQ_BLK_MAGIC;
    /* A seq-et NEM itt irjuk! RDY-strict timeoutnal a spi_send_block a
     * CS_ASSERT elott return-ol, a blokk marad — ha itt novelnenk a seq-et,
     * a retry eggyel nagyobb sorszammal menne ki, es az ESP FANTOMLYUKAT
     * latna (st_lost++ + 5 ms csend). A seq-et ezert a tenyleges kikuldeskor
     * irjuk, kozvetlenul a CS_ASSERT elott (lasd spi_send_block). */
    b->hdr.nsamp    = (uint16_t)BLK_SAMPLES;
    b->hdr.decim    = (uint16_t)s_decim_R;
    /* A RAIL-RSSI-t (dBm) a fs_in_hz FELSO 12 bitjebe csomagoljuk: a
     * fejlecben nincs kulon mezo, es a 1040 bajtos blokkmeretet (DMA,
     * reorder, SpyServer) NEM bantjuk. Also 20 bit = valodi rata
     * (<=1 048 575, a 400000 bar elfer), felso 12 bit = elojeles dBm.
     * A RSSI-t ~100 ms-onkent frissitjuk (a pump ~200 blokk/s). */
    static uint16_t rssi_div = 0;
    if (++rssi_div >= 20u) {
      rssi_div = 0;
      int16_t rq = RAIL_GetRssi(s_rail_for_restart, false);   /* 0.25 dBm egyseg */
      if (rq != RAIL_RSSI_INVALID) s_rssi_dbm = (int16_t)(rq / 4);
    }
    b->hdr.fs_in_hz = (s_fs_in_hz & 0x000FFFFFu)
                    | ((uint32_t)((uint16_t)s_rssi_dbm & 0x0FFFu) << 20);
    spi_send_block(b);
  }

  return true;
}

/* ================= LAB-TESZT ('g') =================
 *
 * A harom vonalat sima GPIO-kent billegteti, KULON frekvencian. A masik
 * oldalon az el-szamlalo igy egyertelmuen megmondja, melyik jel er celba.
 *
 * FONTOS MELLEKHASZNALAT: mindharom lab PONTOSAN 50%-on billeg, tehat ez
 * egyben KITOLTES-REFERENCIA is. 3.3 V-os logikanal a multimeter DC
 * atlaga 1.65 V kell legyen mindharmon. Ez a meres fogta meg 2026-07-31-en
 * az I2S word select hibajat, miutan minden szoftveres nyom elfogyott.
 */
void iq_stream_pin_test(uint32_t seconds)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO->USARTROUTE[0].ROUTEEN = 0;

  GPIO_PinModeSet(SPI_CS_PORT,   SPI_CS_PIN,   gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_CLK_PORT,  SPI_CLK_PIN,  gpioModePushPull, 0);
  GPIO_PinModeSet(SPI_MOSI_PORT, SPI_MOSI_PIN, gpioModePushPull, 0);

  printf("# lab-teszt %lu mp (mindharom PONTOSAN 50%% -> DC atlag 1.65 V):\r\n",
         (unsigned long)seconds);
  printf("#   PA07 / EXP 13 / CS   = 1 kHz  (~2000 el/s)\r\n");
  printf("#   PC05 / EXP 15 / SCLK = 2 kHz  (~4000 el/s)\r\n");
  printf("#   PC00 / EXP 10 / MOSI = 4 kHz  (~8000 el/s)\r\n");

  RAIL_Time_t t_end = RAIL_GetTime() + seconds * 1000000u;
  uint32_t n = 0;

  while ((int32_t)(RAIL_GetTime() - t_end) < 0) {
    RAIL_Time_t next = RAIL_GetTime() + 125u;      /* 8 kHz alapütem */
    while ((int32_t)(RAIL_GetTime() - next) < 0) { }

    ++n;
    if ((n & 3u) == 0u) GPIO_PinOutToggle(SPI_CS_PORT,   SPI_CS_PIN);
    if ((n & 1u) == 0u) GPIO_PinOutToggle(SPI_CLK_PORT,  SPI_CLK_PIN);
    GPIO_PinOutToggle(SPI_MOSI_PORT, SPI_MOSI_PIN);
  }

  /* A CS INAKTIVBA (magas), nem nullaba! Ha alacsonyan hagynank, a masik
   * oldal folyamatosan kivalasztottnak latna magat, es ontene a csonka
   * tranzakciokat. A masik ketto mehet nullara. */
  GPIO_PinOutSet(SPI_CS_PORT,     SPI_CS_PIN);
  GPIO_PinOutClear(SPI_CLK_PORT,  SPI_CLK_PIN);
  GPIO_PinOutClear(SPI_MOSI_PORT, SPI_MOSI_PIN);
  printf("# lab-teszt vege (CS inaktivba allitva)\r\n");
}

/* ================= EUSART ORAJEL-DIAGNOSZTIKA ('v') ================= */

void iq_stream_dump_uart_cfg(void)
{
  uint32_t clksel = CMU->EUSART0CLKCTRL;
  uint32_t cfg0   = EUSART0->CFG0;
  uint32_t clkdiv = EUSART0->CLKDIV;

  printf("\r\n# ---- EUSART0 (VCOM) orajel-diagnosztika ----\r\n");
  printf("# CLKCTRL=0x%08lX  CFG0=0x%08lX  CLKDIV=0x%08lX\r\n",
         (unsigned long)clksel, (unsigned long)cfg0, (unsigned long)clkdiv);

  uint32_t f = CMU_ClockFreqGet(cmuClock_EUSART0);
  uint32_t ovs_field = (cfg0 & _EUSART_CFG0_OVS_MASK) >> _EUSART_CFG0_OVS_SHIFT;
  uint32_t ovs = (ovs_field == 0u) ? 16u : (ovs_field == 1u) ? 8u
               : (ovs_field == 2u) ? 6u  : (ovs_field == 3u) ? 4u : 0u;
  uint32_t div = (clkdiv & _EUSART_CLKDIV_DIV_MASK) >> _EUSART_CLKDIV_DIV_SHIFT;
  printf("# fclk=%lu Hz  OVS=%lu  DIV=%lu\r\n",
         (unsigned long)f, (unsigned long)ovs, (unsigned long)div);
  if (ovs && f) {
    uint64_t den = (uint64_t)ovs * (256ull + div);
    printf("# -> szamitott baud=%lu, elerheto max=%lu\r\n",
           (unsigned long)(uint32_t)(((uint64_t)f * 256ull) / den),
           (unsigned long)(f / ovs));
  }

  printf("# ---- SPI (USART0) ----\r\n");
  printf("# USART0 ora=%lu Hz, kert SPI=%lu Hz\r\n",
         (unsigned long)CMU_ClockFreqGet(cmuClock_USART0),
         (unsigned long)SPI_HZ);
  printf("# USARTROUTE: TX=0x%08lX CLK=0x%08lX ROUTEEN=0x%08lX "
         "(CSPEN SZANDEKOSAN nincs)\r\n",
         (unsigned long)GPIO->USARTROUTE[0].TXROUTE,
         (unsigned long)GPIO->USARTROUTE[0].CLKROUTE,
         (unsigned long)GPIO->USARTROUTE[0].ROUTEEN);
}

/* ================= EXT-MOD: idegen 1040 bajtos blokk kuldese =================
 * A scan.c hasznalja: RAIL RX-stream NELKUL, ugyanazon az SPI/LDMA/RDY uton
 * kuld egy blokkot (SPECLINE, specline.h). A blokknak PONTOSAN BLK_BYTES
 * meretunek kell lennie, es a fejlec 4. bajtjatol a seq-mezot a
 * spi_send_block irja (mint az IQ-nal). Kizarja a normal streamet. */
static bool s_ext_active = false;

void iq_stream_ext_begin(void)
{
  if (s_active || s_ext_active) return;
  s_prod = s_cons = 0; s_seq = 0; s_tx_busy = false;
  s_rdy_waits = 0; s_rdy_timeouts = 0; s_rdy_wait_us = 0; s_rdy_skips = 0;
  spi_setup();
  s_ext_active = true;
}

void iq_stream_ext_end(void)
{
  if (!s_ext_active) return;
  spi_teardown();
  s_ext_active = false;
}

bool iq_stream_ext_busy(void)
{
  return s_tx_busy;
}

void iq_stream_ext_pump(void)
{
  if (!s_ext_active) return;
  (void)spi_tx_poll();
}

bool iq_stream_ext_send(const void *blk)
{
  if (!s_ext_active || s_tx_busy || blk == NULL) return false;
  /* A 0. blokkpuffert hasznaljuk; a spi_send_block a seq-et beleirja. */
  memcpy(&s_blk[0], blk, BLK_BYTES);
  spi_send_block(&s_blk[0]);
  return s_tx_busy;     /* false = RDY-strict timeout, ujra kell probalni */
}

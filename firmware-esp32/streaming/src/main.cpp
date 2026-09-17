/* SPDX-License-Identifier: MIT
 *
 * fg23_spi_to_usb_wifi.ino / main.cpp
 * ESP32-S3: SPI slave a FG23 fele -> USB + WiFi + rtl_tcp
 *   Copyright (c) 2026 Zoltan Doczi HA7DCD
 *
 * ================= AZ EGESZ LANC =================
 *
 *   antenna -> FG23 RAIL FIFO -> zero-copy olvasas -> ketfokozatu CIC
 *   -> DC-blokk -> blokkpuffer -> LDMA -> SPI (blokkonkent CS)
 *   -> ESP32-S3 SPI slave DMA -> HAROM PARHUZAMOS KIMENET:
 *
 *        (a) NATIV USB CDC ........ 1040 bajtos blokkok fejleccel
 *                                   -> iq_bridge.py -> SDR++ (asztali)
 *        (b) TCP 8888 ............. NYERS int16 I/Q, fejlec nelkul
 *                                   -> SDR++ "Network" source KOZVETLENUL,
 *                                      python NELKUL
 *        (c) TCP 1234 ............. rtl_tcp protokoll, 8 bites
 *                                   -> SDR++ Android, SDR Touch, barmi
 *
 * A harom kimenet FUGGETLEN. Nincs "atkapcsolas": ami epp csatlakozik, az
 * kap adatot, a tobbi nem szamit. Ez egyszerubb ES megbizhatobb, mint
 * detektalni, hogy "van-e USB" — a detektalas hazudhat (a CDC akkor is
 * "csatlakozottnak" latszik, ha senki nem olvassa), a "van-e TCP kliens"
 * viszont nem tud hazudni.
 *
 * ===================== SPI BEKOTES =====================
 *   WSTK EXP   FG23 lab   jel                    ESP32-S3 (Heltec V3)
 *   ---------------------------------------------------------------------
 *   EXP 15     PC05       SCLK                -> GPIO5
 *   EXP 10     PC00       MOSI (FG23 -> ESP)  -> GPIO6
 *   EXP 13     PA07       CS   (blokk-keret)  -> GPIO7
 *   EXP  1     GND                            -> GND
 *   EXP 11     PA06       CMD  (ESP -> FG23)  <- GPIO4   [OPCIONALIS]
 *
 * A CMD vonal a visszairany: ezen kuldi az ESP a FG23-nak a hangolasi
 * parancsot, amikor a telefonon atteker a frekvencian. NELKULE minden
 * mukodik, csak az rtl_tcp frekvencia-parancsa nem hat semmire.
 * A FG23 oldalon a cmdlink.c kell hozza.
 *
 * ===================== NATIV USB BEKOTES =====================
 *   USB kabel D-   -> GPIO19
 *   USB kabel D+   -> GPIO20      (a ketto felcserelve: Windows 43-as hiba)
 *   USB kabel GND  -> GND
 *   USB kabel VBUS -> NE KOSD BE
 *
 * ================= platformio.ini =================
 *     build_flags =
 *       -D ARDUINO_USB_MODE=1
 *       -D ARDUINO_USB_CDC_ON_BOOT=1
 * Ettol: Serial = nativ USB CDC (nyers folyam), Serial0 = CP2102 (szoveg).
 *
 * ---- MERESHEZ AJANLOTT: -D ARDUINO_USB_MODE=0  (2026-08-06) ----
 * A MODE=1 a HARDVERES USB-Serial-JTAG-ot hasznalja (HWCDC). Ennek ket
 * tulajdonsaga EGYUTT megoli a PC-oldali meromodot (usb_aprs_rx.py):
 *
 *   1) a host DTR/RTS billentese HARDVERESEN RESETELI a chipet
 *      (bootlog: "rst:0x15 (USB_UART_CHIP_RESET)"). A reset utan a USB
 *      UJRA-ENUMERAL, es a Windows usbser.sys mar nyitott handle-je ZOMBI
 *      lesz: a PC-n a read() onnantol orokre 0 bajtot ad.
 *   2) a HWCDC abbol donti el, hogy "van-e host", hogy urul-e a TX FIFO.
 *      Ha egyszer megtelt (mert a host nem olvasott), onnantol "nincs
 *      host", minden irast eldob -> a FIFO sose urul. ONMAGAT EROSITO
 *      beragadas. A statuszsorban ez latszik:  USB 0.0 kB/s(-244)
 *      ahol a (-244) a st_usbdrop, azaz 244 eldobott iras.
 *
 * MODE=0 eseten a Serial a TinyUSB-s USBCDC, aminek VAN enableReboot(false)
 * metodusa: a DTR/RTS onnantol NEM resetel, a meres stabilan fut.
 *     build_flags =
 *       -D ARDUINO_USB_MODE=0
 *       -D ARDUINO_USB_CDC_ON_BOOT=1
 * A COM-szam valtozhat (mas USB PID), de a VID marad 0x303A, tehat az
 * usb_aprs_rx.py auto-detektalasa tovabbra is megtalalja.
 * A flashelest ez nem erinti (az a CP2102-n megy), es a ROM-bootloader
 * mindig ad JTAG-portot, tehat nem lehet "kizarni magad".
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include "driver/spi_slave.h"
#include "esp_intr_alloc.h"   /* ESP_INTR_FLAG_IRAM / _LEVEL3 */
#include "soc/gpio_struct.h"  /* a GPIO regiszterstruktura (RDY, ISR-bol) */
#include "driver/gpio.h"
#include "esp_heap_caps.h"    /* RAM-diagnosztika a 2 s-os statuszban */
#include "esp_wifi.h"         /* esp_wifi_set_bandwidth (HT40, RTL_ENABLE-hez) */
#include <lwip/sockets.h>     /* send(fd, ..., MSG_DONTWAIT) — lasd ring_flush */
#include <string.h>
#include <math.h>             /* log10f — jelszint dB-ben */
#include "wifi_bench.h"       /* WiFi-plafon mero ('B...' parancsok a DIAG-on) */
#include "rgb_load.h"         /* WS2812 terhelesjelzo ('L...' parancsok)       */
#include "tft.h"              /* ILI9341 allapotkijelzo, SPI2 (IO_MUX 9-14)    */
#include "iq_fft.h"           /* keskenysavu I/Q -> waterfall a panelra        */

/* TinyUSB-s uzemmod (ARDUINO_USB_MODE=0): itt a Serial egy USBCDC, aminek van
 * enableReboot() metodusa — ezzel tiltjuk le a DTR/RTS-re torteno resetet.
 * FIGYELEM a feltetelre: a preprocesszor a NEM DEFINIALT azonositot 0-nak
 * veszi, ezert a puszta "#if ARDUINO_USB_MODE == 0" IGAZ lenne akkor is, ha a
 * makro nincs megadva — es beleforditana a TinyUSB-agat egy HWCDC-s buildbe.
 * Ezert kell a defined() is. */
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 0)
  #include "USB.h"
  #define FG23_USB_TINYUSB 1
#else
  #define FG23_USB_TINYUSB 0
#endif

/* ==================== KONFIGURACIO ==================== */

/* ---- WiFi ----------------------------------------------------------
 * Tobb halozat, prioritas nelkul: a WiFiMulti vegigpasztazza a kornyeket, es
 * az ISMERT halozatok kozul a LEGEROSEBBET valasztja. Tehat nem sorrendben
 * probalgat, hanem terepen dont — ha otthon vagy, az otthonira megy, ha a
 * muhelyben, a muhelyire, es nem szamit, melyik all elorebb a tablazatban.
 *
 * URES ssid-t kihagy, tehat a nem hasznalt sorokat nem kell torolni, es a
 * tablazat merete magatol adodik — nincs kulon darabszam, amit el lehetne
 * felejteni frissiteni.
 *
 * Ha egyik ismert halozat sincs a kozelben, sajat AP-t nyit. A kocsiban ez a
 * jo: a telefon az ESP-hez csatlakozik, router nelkul. */
/* A TERHELES-KAPCSOLOK (WIFI_ENABLE, SPY_ENABLE, OLED_ENABLE, APRS_RX_ENABLE)
 * mind #ifndef-be vannak csomagolva, hogy a platformio.ini build_flags-bol
 * FELULIRHATOK legyenek. Enelkul a WiFi-mentes izolalo mereshez kezzel kellene
 * atirni ezt a fajlt, majd VISSZAIRNI — es pontosan ilyenkor marad bent
 * veletlenul egy 0, amitol hetekig keresel egy nem letezo hibat.
 * Igy a meres csak environment-valtas: [env:esp32s3_nowifi]. */
#ifndef WIFI_ENABLE
#define WIFI_ENABLE          1
#endif
#define WIFI_STA_TIMEOUT_MS  10000u
/* WiFi credentials live in secrets.h (git-ignored). Copy secrets.example.h
 * to secrets.h and fill in your own networks. */
#include "secrets.h"
#define WIFI_AP_COUNT (sizeof(WIFI_APS) / sizeof(WIFI_APS[0]))

/* Ujracsatlakozas: ha kiesik a halozat, ennyi ido utan probal ujra teljes
 * pasztazassal. NEM surubben — a WiFiMulti.run() blokkol a pasztazas
 * idejere (par masodperc), es kozben all a fo ciklus. */
#define WIFI_RETRY_MS        30000u

#define PORT_RAW     8888    /* nyers int16 — SDR++ asztali "Network" source */
#define PORT_RTLTCP  1234    /* rtl_tcp — SDR++ Android, SDR Touch, gqrx... */

/* ---- rtl_tcp KAPCSOLO ----------------------------------------------
 * 0 = a teljes rtl_tcp-ut ki: az 1234-es port suket, es ~75 kB RAM
 * szabadul fel (64 kB gyuru + 10 kB felmintavetelezo puffer) — ebbol
 * lesz helye a spyserver 64 kB-os gyurujenek. A kod fordul tovabbra is,
 * csak a pufferek csokevenyek es a szerver el sem indul.
 *
 * MIERT: a spyserver-ut (5555) natív 16 bit, natív rata, nincs
 * felmintavetelezes, nincs 8 bites kvantalas — mindenben jobb. Az
 * rtl_tcp csak akkor kell, ha egy kliens (pl. SDR Touch) nem tud
 * spyservert.
 *
 * 2026-08-03 este, Zoltan dontese a tesztek utan: alapbol KI. Az ut
 * vegig lett merve (DC-tu javitva, szellemkepek -38 dB, atviteli plafon
 * ~400 kB/s a szukseges 500 helyett -> csikozas), es a spyserver mindenben
 * jobb. A +75 kB-bol a spyserver 64 kB-os gyuruje automatikusan kijon
 * (a felezo letra megkapja) — az teszi rezzenestelenne a telefont. */
#define RTL_ENABLE   0

/* ---- HT40 KAPCSOLO — alapbol KI, MERT MERTUK ------------------------
 * Az elso valtozat a RTL_ENABLE-hez kototte, de a 2026-08-03-i merés
 * megbuktatta: a HT40 KERESE mellett a +-(NQUEUE+-1) driver-glitch a
 * 10-30 s-onkenti egyrol masodpercenkenti 6-12-re ugrott (HATRA=25/2s,
 * REND dup/lyuk tucatjaval, meg csonka=1 is), mikozben az atviteli
 * plafon SEMMIT nem javult (355-510 kB/s maradt — a router nem adta meg,
 * vagy a zsufolt 2,4 GHz-en nem er semmit). A szelesebb csatorna-keres
 * lathatoan megbolygatja a WiFi-driver megszakitas-idoziteset, az pedig
 * az SPI slave-et. Kiserletezeshez tedd 1-re — de elobb nezd meg, a
 * routered egyaltalan megadja-e (WiFi-analizator: 40 MHz-es sav). */
#define WIFI_HT40    0
#define PORT_SPY     5555    /* SpyServer — 16 bites, NATIV rata, SDR++ Android */

/* ---- SpyServer. Ez a jo ut Androidra: natív 16 bit, es az ESZKOZ mondja
 * meg a ratat, tehat nincs felmintavetelezes es nincsenek interpolacios
 * kepek. Az rtl_tcp az 1234-en MARAD — ha ez nem indul be elsore, ott a
 * bevalt ut. Reszletek: spyserver.h */
#ifndef SPY_ENABLE
#define SPY_ENABLE   1
#endif
#if SPY_ENABLE
#include "spyserver.h"
#endif

/* ---- rtl_tcp: a kliensek fix, "RTL-szeru" mintavetelt varnak, es a
 * legkisebb szabvanyos RTL-rata 250 ksps — nalunk ennel jóval kevesebb van.
 * Ezert EGESZ SZAMU szorzoval felmintavetelezunk a celratara. Igy a kliens
 * ido- es oraszamitasa stimmel, es a hang jo magassagon szol.
 *
 * ARA: a felmintavetelezes kepeket hagy a spektrumban (linearis
 * interpolacio, nincs utana szuro). A vizeses szelen latni fogod oket, a
 * demodulator viszont nem, mert kiszuri a sajat sávjaval.
 *
 * RTL_TARGET_SPS 0 = nincs felmintavetelezes (allitsd a klienst kezzel). */
#define RTL_TARGET_SPS   250000u
#define RTL_MAX_UP       20u        /* felso korlat: a legmelyebb tamogatott
                                     * mod az i32 (12500 sps), ahhoz pont
                                     * 20x kell a 250 ksps-hez. Az agg_rtl
                                     * puffer merete is ebbol jon (10 kB). */

/* ANTI-IMAGE SZURO. A linearis interpolacio maga is szur (haromszog-ablak,
 * ami ket egymas utani L hosszu atlag), es a nulla-helyei PONT a kepekre
 * esnek — de a kepek KORNYEKE atszivarog, es a vizesesen ez latszik ugy,
 * mintha a jel szelesebb es "kehes" lenne, mint amilyen.
 *
 * Egy TOVABBI L hosszu mozgoatlag a kimeneti ratan meg egy nullat tesz
 * ugyanoda, es a szivargast nagysagrendileg 20 dB-lel nyomja. Futo osszeg,
 * tehat mintankent ket osszeadas es egy osztas — a 240 MHz-es S3-nak semmi.
 *
 * 0-ra allitva pontosan a regi (nyers linearis) viselkedest kapod vissza. */
#define RTL_ANTI_IMAGE   1

/* 8 bites kimenet: automatikus erosites.
 *
 * SZELES HOLTSAV, LASSU LEPES: minden szintvaltas 6 dB-t mozdit az EGESZ
 * kepen, ami a vizesesen vizszintes savkent latszik. Ezert csak akkor
 * nyulunk hozza, ha a csucs tenyleg kilog a [LO..HI] sávból, es akkor is
 * legfeljebb ket masodpercenkent egy lepest. Igy beall egyszer, es marad.
 *
 * RTL_FIXED_SHIFT >= 0 eseten fix eltolas, egyaltalan nincs AGC — ha
 * szintet akarsz merni a vizesesen, EZT hasznald (8 = a felso bajt). */
#define RTL_FIXED_SHIFT  (-1)
#define RTL_AGC_MS       2000u
#define RTL_AGC_HI       120        /* efolott halkitunk (max 127) */
#define RTL_AGC_LO       30         /* ez alatt erositunk */

/* ---- OLED (Heltec WiFi LoRa 32 V3) --------------------------------
 * A kijelzo fo haszna: KIIRJA AZ IP-CIMET. Enelkul minden ujrainduláskor
 * soros monitort kell nyitni, csak hogy megtudd, hova csatlakozzon az
 * SDR++ — a kocsiban ez hasznalhatatlan.
 *
 * Heltec V3 labkiosztas: SDA=17, SCL=18, RST=21, Vext=36 (aktiv ALACSONY).
 * Ha sotet marad a kijelzo, eloszor az OLED_VEXT_ACTIVE_LOW-t forditsd meg,
 * masodszor tedd az OLED_VEXT_PIN-t -1-re (van olyan V3 peldany, ahol az
 * OLED nem a Vext-rol megy).
 *
 * platformio.ini-be KELL:   lib_deps = olikraus/U8g2
 *
 * IDOZITES: egy teljes 128x64-es kepatvitel ~1 kB az I2C-n, 800 kHz-en
 * nagysagrendileg 13 ms. Ez alatt a fo ciklus all — DE az SPI slave DMA
 * kozben is fogadja a blokkokat, amig van felfuzott tranzakcio. NQUEUE=24
 * i8-on ~123 ms tartalek, tehat a 13 ms elnyelodik. Ezert emeltuk meg. */
#ifndef OLED_ENABLE
#define OLED_ENABLE            0     /* 2026-08-19: a Heltec V3 kivezetve.
                                      * ESP32-S3 devkit + ILI9341 240x320.
                                      * Ezzel eltunt a 21 ms-os blokkolas a
                                      * fo ciklusbol es a GPIO36 utkozes is
                                      * (az oktalis PSRAM laba). */
#endif
/* WS2812 terhelesjelzo LED. EZEN a panelon (Ali S3 dupla-USB, N16R8) a
 * kivezetes szerint GPIO47 = RGB_LED (a 48 a SPICLK_N). Mas klonokon 48
 * vagy 38 — ha nem vilagit, azokat probald. Heltec V3-on nincs RGB LED.
 * FIGYELEM: a 47 a waterfall-benchben a kijelzo TE-je volt -> az TE a 48-ra. */
#ifndef RGB_LED_PIN
#define RGB_LED_PIN            47
#endif
#define OLED_SDA_PIN           17
#define OLED_SCL_PIN           18
#define OLED_RST_PIN           21
#define OLED_VEXT_PIN          36     /* -1 = nincs Vext-vezerles */
#define OLED_VEXT_ACTIVE_LOW   1
#define OLED_I2C_HZ            800000u
/* Vissza 1500-ra: a merés lezarult, es bebizonyosodott, hogy a 21 ms-os
 * kepatvitel a 24 mely tranzakcio-sorral bosegesen elfer (qmax legfeljebb
 * 5 meg OLED-frissites kozben is). */
#define OLED_REFRESH_MS        1500u
/* Csomagvetel utan ennyi ideig az APRS-oldal latszik, aztan visszavalt a
 * halozati oldalra. Igy nem kell lapozgatni: a kijelzo AKKOR mutatja a
 * csomagot, amikor tortenik. */
#define OLED_APRS_HOLD_MS      12000u

/* Az include ITT van, nem a fajl elejen: az OLED_ENABLE-nek mar
 * definialtnak kell lennie hozza. */
#if OLED_ENABLE
#include <Wire.h>
#include <U8g2lib.h>
#endif

/* ---- APRS vetel: AFSK1200 demod + AX.25 dekoder + APRS-IS feltoltes.
 * A reszletek az aprs_rx.cpp-ben; a hivojel, a passcode es a szuro is ott
 * allithato. A demod az I/Q-bol dolgozik, tehat NEM zavarja a tobbi
 * kimenetet — ugyanaz a blokk megy tovabb a 8888-ra es az 1234-re is.
 * Terheles 50 ksps-en nagysagrendileg 1-2% egy magbol. */
#ifndef APRS_RX_ENABLE
#define APRS_RX_ENABLE   1
#endif
#if APRS_RX_ENABLE
#include "aprs_rx.h"
#endif
#include "cw_rx.h"          /* CW Morse dekoder */

/* ---- Menu es gomb. A Heltec V3 PRG gombja (GPIO0) lapoz az oldalak kozt
 * es kapcsolja a demodulatorokat. Reszletek: ui.h */
#include "ui.h"
#include "flashlog.h"
#include "specline.h"      /* SPECLINE (scan-sor) blokkformatum, kozos a FG23-mal */
#include "oled_spectrum.h"  /* 128x64 mini-spektrum + 1-bites waterfall */

/* ==================== RDY-VONAL: A VESZTES MEGSZUNTETESE BY DESIGN ====
 *
 * 2026-08-03-ig a FG23 VAKON kuldott: semmi nem mondta meg neki, hogy a
 * tuloldalon van-e felfuzott SPI-tranzakcio. Ha epp nem volt, a hardver a
 * regi leiroba irt, es a blokk vagy elveszett, vagy pufferkornyi
 * eltolassal jott meg. Ez volt a VESZT, a HATRA, a szetkent vivo es a
 * nema APRS kozos gyokere. A merés szerint SpyServer-terheles alatt
 * ~7 blokk/s veszett igy el — egy 0.66 s-os APRS-csomagra 4-5 lyuk jut.
 *
 * A megoldas egy egyetlen vezetek: az ESP32 az IDF spi_slave
 * post_setup/post_trans callbackjeibol jelzi, hogy TENYLEG fel van-e
 * fuzve tranzakcio (a hardver allapotat, nem a mi konyvelesunket!), a
 * FG23 pedig a CS lehuzasa ELOTT megnezi, es ha nincs, VAR. Igy blokk
 * nem veszhet el — legfeljebb kesik, amit a FG23 6 blokkos puffere elnyel.
 *
 * BEKOTES:  ESP32 GPIO2  ->  FG23 PD2 / EXP 9   (GND mar kozos)
 *
 * A tuloldali parja: iq_stream.c, IQ_RDY_ENABLE. Amig a drot nincs
 * bekotve, a FG23-on hagyd 0-n — az ESP32 oldala driven artalmatlan. */
#define RDY_ENABLE   1
#define RDY_PIN      2

/* ---- Visszairany a FG23-nak (hangolas a telefonrol). 0 = kikapcsolva. */
#define CMDLINK_ENABLE   1
#define CMDLINK_TX_PIN   18         /* ESP GPIO18 -> FG23 PA06 / EXP 11
                                     * (2026-08-19: 4-rol athelyezve, a 4-et
                                     * a kijelzo/touch kapja a devkiten) */
#define CMDLINK_BAUD     115200

/* ---------------- LABAK ---------------- */
/* 2026-08-19 — ESP32-S3 devkit + ILI9341 kijelzo.
 * A kijelzo kapja az SPI2/FSPI IO_MUX keszletet (9..14), a touch OSZTOZIK
 * vele (sajat CS), az FG23 slave ezert az SPI3-ra kerul. 4 MHz-en a
 * GPIO-matrix bosegesen eleg neki. A VEGLEGES allapotban megfordul:
 * az FG23 kapja az IO_MUX-ot (10/11/12 + RDY 14 + CMD 13), a kijelzo
 * megy SPI3-ra — az a 700 ksps-es irany feltetele. */
#define PIN_SCLK   GPIO_NUM_15    /* FG23 PC05 / EXP 15 */
#define PIN_MOSI   GPIO_NUM_16    /* FG23 PC00 / EXP 10 */
#define PIN_CS     GPIO_NUM_17    /* FG23 PA07 / EXP 13 */
#define FG23_SPI_HOST SPI3_HOST

/* ---------------- BLOKK (egyezzen az iq_stream.c-vel!) ---------------- */
#define BLK_SAMPLES   256
#define HDR_BYTES     16
#define PAYLOAD_BYTES (BLK_SAMPLES * 4)                  /* 1024 */
#define BLK_BYTES     (HDR_BYTES + PAYLOAD_BYTES)        /* 1040 */
#define IQ_BLK_MAGIC  0x32425149u                        /* "IQB2" */

/* Elore felfuzott tranzakciok. NEM luxus: ha CS-kor nincs felhuzott
 * tranzakcio, az a blokk VEGLEG elveszik — es mivel a FG23 a sorszamot a
 * KIKULDESKOR irja a fejlecbe, minden sorszamlyuk PONTOSAN ezt jelenti.
 * (A FG23 sajat eldobasai NEM latszanak lyukkent, azok a blokk/s-ben.)
 *
 * 4 db i8-on (195 blokk/s) csak 20 ms tartalek. 12 db ~61 ms volt — es
 * pont ezt mertuk: a naplo tele van d=13 lyukakkal, azaz 12 elveszett
 * blokkal egyszerre. Az "egyszerre pontosan NQUEUE" a keszlet teljes
 * kiurulesenek az ujjlenyomata. 24 db ~123 ms. */
/* ==================== KESZLET-KISERLET (2026-08-03) ====================
 *
 * A merés kimutatta, hogy a sorszamugrasok merete PONTOSAN koveti ezt a
 * szamot:
 *      NQUEUE = 12  ->  d = 13 (=12+1),  25 (=2*12+1)
 *      NQUEUE = 24  ->  d = 25 (=24+1),  49 (=2*24+1),  hatra -23 (=-24+1)
 *
 * Ez azt sugallja, hogy egy-egy blokk helyett olyan puffert dolgozunk fel,
 * aminek a tartalma pontosan egy teljes pufferkorrel odebb van. DE ez meg
 * csak ket adatpont, es mindketto kettohatvany-kozeli — lehet veletlen is.
 *
 * EZERT most 17 (prim, semmilyen mas periodussal nem esik egybe). Ha az
 * ugrasok atvaltanak 16 / 18 / 35-re, a keszlet a bunos, es a helyes
 * iranyban kereshetunk tovabb. Ha 25 es 49 marad, az elmelet halott.
 *
 * A KISERLET LEFUTOTT (2026-08-03): NQUEUE=17 mellett az ugrasok
 * atvaltottak 18 / 35 / -16-ra, POOL=100%. A keszlet a bunos, tehat
 * visszaallitva 24-re, es a valodi javitas a megszakitas prioritasa
 * (lasd a spi_bus_config_t.intr_flags-nel). A POOL szamlalo marad: ha
 * a javitas hat, a HATRA es vele a POOL nevezoje leesik.
 *
 * ===================== RDY UTANI SZEREP (2026-08-03) =====================
 *
 * A RDY-kezfogas + RDY_STRICT ota a keszlet kiurulese mar NEM okoz sem
 * sorrendcserét, sem vesztest: a FG23 egyszeruen ELHALASZTJA a kuldest
 * (a sajat NBLK=6 puffere ~31 ms-ot tart), amig nincs felfuzott
 * tranzakcio. Az NQUEUE azota tisztan lokescsillapito: azt hidalja at,
 * amig a fo ciklus massal van elfoglalva es nem fuz vissza.
 *
 * Meretezes meresbol: qmax normal uzemben legfeljebb 5 (OLED-frissites
 * kozben is), egyedul a blokkolo APRS-IS connect viszi 24/24-re — azt
 * viszont SEMMILYEN ertelmes melyseg nem fedi le (330 ms = 64 blokk),
 * arra kulon javitas kell.
 *
 * ============ ATMERETEZVE 16 -> 24 (2026-08-07) ============
 *
 * A 16 azzal az indoklassal lett 16, hogy "82 ms turés, a mert csucs
 * 16-szorosa". Az ejszakai 4,55 oras meres (16 599 keret, −100 dBm) ezt
 * megcafolta. A PER# sorszamokbol, csomagszintu felbontasban:
 *
 *   - 73 vesztes-esemeny / 75 keret     -> 99,548 % siker
 *   - a vesztesek 70 %-anal a dt_max 80..170 ms, es CRC-hiba NINCS
 *     (a keret el sem jutott a dekoderig: mintalyuk, nem demod-hiba)
 *   - dt_max median a vesztesek kornyeken 76,3 ms, veletlen idopontban
 *     21,6 ms;  P(dt_max>60 ms) 50,7 % vs 7,1 %  =  7,2x dusulas
 *   - 228 kulonallo akadas: MEDIAN 88 ms, p90 98 ms, MAX 167 ms
 *
 * 16 melyseg = 16 * 5,12 ms = 82 ms, a mert akadas medianja 88 ms.
 * A keszlet tehat SZISZTEMATIKUSAN ROVIDEBB volt a tipikus akadasnal —
 * nem 16-szoros tartalek volt, hanem negativ. A "mert csucs" azert
 * latszott kicsinek, mert a qmax-ot normal uzemben neztuk, nem az
 * akadasok alatt.
 *
 * Volt egy belso ellentmondas is: RXRING_BLOCKS = 24 (=123 ms), de a
 * gyuruben csak az landolhat, amit a keszlet mar felvett. A gyuru
 * 123 ms-os kapacitasa igy ELERHETETLEN volt — a lanc a szukebb elemre,
 * 82 ms-ra meretezodott. 24-gyel keszlet es gyuru egyarant 123 ms.
 *
 * Ez nem NQUEUE-folt: a mar kifizetett 25 kB gyuru felenek felszabaditasa.
 * Ara +8,3 kB RAM (rxbuf), es a REORDER_BLOCKS-ot is vinni kell 32-re
 * (lasd ott: RO_HOLD >= NQUEUE+1).
 *
 * AMIT EZ NEM OLD MEG: ha az akadas FLASH-muvelet, a gyorsitotar
 * kikapcsol, es akkor a spi_slave megszakitaskezeloje sem tud futni —
 * nincs mibol felfuznie a kovetkezo tranzakciot, akarmilyen mely a
 * keszlet. Az ellen csak a CONFIG_SPI_SLAVE_ISR_IN_IRAM=y ved. A ketto
 * nem alternativa, hanem egymas feltetele: az IRAM teszi a melyseget
 * flash-muvelet alatt is HASZNALHATOVA, a melyseg pedig megadja, MENNYI
 * idot lehet athidalni. */
#define NQUEUE        24

/* STALE BLOKKOK ELDOBASA — alapbol KI.
 *
 * 2026-08-03-i merés: amikor a felfuzott tranzakciok keszlete kiurul, a
 * blokkok NEM elvesznek, hanem ROSSZ SORRENDBEN erkeznek meg. A naplo
 * bizonyiteka: a blokk/s vegig a teljes 195 (tehat semmi nem hianyzik),
 * mikozben a VESZT 1368/s-ot mutat — hetszer tobbet, mint amennyi blokk
 * egyaltalan letezik. A sorszam kozben visszafele is lep.
 *
 * A kovetkezmeny sulyos: egy folyamatos jel 5.12 ms-os darabjai idoben
 * osszekeverednek. Az amplitudo folytonos marad, a FAZIS viszont minden
 * blokkhatáron ugrik — egy tiszta vivobol igy 19.8 % marad, a tobbi 780 Hz
 * szelesre kenodik. Az AFSK ettol biztosan nem dekodolhato.
 *
 * A VALODI javitas az, hogy a keszlet SOSE uruljon ki (leszivas + azonnali
 * visszafuzes + NQUEUE 24, lasd lejjebb). Ha az mukodik, a HATRA szamlalo
 * nulla lesz, es ennek a kapcsolonak nincs dolga.
 *
 * Ha megis marad sorrendcsere, ezzel az elavult blokkokat el lehet dobni:
 * a stream idorendje helyreall, cserebe lyukak keletkeznek. NEM egyertelmu
 * javulas — ezert alapbol ki van kapcsolva, es csak merés utan kapcsold be.
 */
#define SEQ_DROP_STALE 0

/* ==================== SORSZAM SZERINTI UJRARENDEZES ====================
 *
 * MERES 2026-08-03, natív USB-n (SpyServer nelkul, tehat tiszta korulmeny
 * kozott), a sorszamokat kozvetlenul szamolva:
 *
 *     hianyzo 168 / vissza  7 = 24        hianyzo 240 / vissza 10 = 24
 *     hianyzo 360 / vissza 15 = 24        hianyzo  96 / vissza  4 = 24
 *
 * A "hianyzo" mindig PONTOSAN 24-szerese a visszalepesnek. Nem 168 blokk
 * vesz el: het darab +25 / -23 "kirandulas" van, es a naiv szamlalo
 * mindegyiket 24 hianynak konyveli. Egy kirandulas merlege:
 *
 *     egy blokk VEGLEG elvesz  (felulirtak a pufferet, mielott kiolvastuk)
 *     egy blokk KETSZER jon meg (egyszer korabban, egyszer a helyen)
 *
 * Ezert egyezik a blokk/s pontosan a termeléssel, es ezert nem latszott
 * semmilyen szamlalon. Az eltolas MINDIG +-NQUEUE, tehat KORLATOS — es
 * ami korlatos, azt ki lehet javitani egy ablakkal.
 *
 * Amit ez ad: a lanc tobbi resze (APRS-demod, SpyServer, gyuruk) idoben
 * HELYES sorrendu mintakat kap, a duplikatumok eltunnek, a tenylegesen
 * elveszett blokk helyere pedig egy 5 ms-os csend kerul. Egy ismert,
 * rovid lyuk nagysagrendekkel jobb, mint egy 123 ms-os idougras: az AFSK
 * egy lyukon at ujraszinkronizal, egy idougrastol viszont szetesik.
 *
 * Amit NEM ad: a gyoker-okot nem javitja. Az tovabbra is nyitott (a
 * megszakitas-prioritas nem volt az). De addig is mukodo rendszer.
 *
 * Ara: REORDER_BLOCKS * 1040 bajt RAM, es ugyanennyi blokknyi kesleltetes.
 * 0 = kikapcsolva.
 *
 * MERETEZES: az ablaknak a legnagyobb mert elkeses-tavolsagot kell fednie,
 * az pedig a keszlethez kotott: k*NQUEUE+-1, gyakorlatban legfeljebb
 * NQUEUE+1. Feltetel: RO_HOLD (= REORDER_BLOCKS-4) >= NQUEUE+1.
 *
 * 2026-08-07: az NQUEUE 16 -> 24 lett, tehat a feltetel 25-ot kovetel.
 * 24-es ablak (RO_HOLD=20) MAR NEM ELEG — ezert 32 (RO_HOLD=28 >= 25,
 * harom blokk margoval, ugyanaz a tartalek, mint korabban).
 * Kesleltetes 164 ms, ara +8,3 kB RAM.
 *
 * RAADASKENT MEGSZUNIK EGY LAPPANGO HIBA. Lentebb a ro_pump() a
 * slot-indexet 'ro_next % REORDER_BLOCKS'-szal kepezi, es a ro_next
 * SZABADON FUTO 32 bites szamlalo. 24 nem osztoja 2^32-nek, tehat a
 * szamlalo atfordulasakor (195 blokk/s mellett ~255 nap folyamatos
 * uzem) az index ugrott volna — pontosan az a hiba, ami ellen az
 * rxring-nel mar kulon vedekezunk. A 32 kettohatvany, osztoja
 * 2^32-nek, igy az atfordulas atlathatatlanul helyes marad.
 *
 * HA AZ NQUEUE MEG NO: REORDER_BLOCKS >= NQUEUE + 5, es maradjon
 * kettohatvany. NQUEUE=32 -> REORDER_BLOCKS 64 (nem 40!). */
#define REORDER_BLOCKS 32

/* Egy fo ciklusban legfeljebb ennyi tranzakciot szedunk le. A regi kod
 * EGYET vett korönkent, tehat a teljes hazimunka (TCP, OLED, WiFi, APRS)
 * arat fizettuk BLOKKONKENT — 195x masodpercenkent. Ha egyszer lemaradunk,
 * korönkent csak egy blokkal tudunk faradni vissza. */
#define DRAIN_MAX     NQUEUE

/* Tobb blokk egyben kimenni MINDIG jobb, mint egyesevel: kevesebb USB-
 * keret, kevesebb TCP-szegmens, kevesebb rendszerhivas. Az usb_bench.ino
 * pont ezt meri — ha nalad tobbet ad a nagy chunk, novelt AGG_BLK-t. */
#define AGG_BLK        4
#define AGG_FLUSH_MS   8u

/* ---- TCP gyurupufferek ----
 * MIERT KELL: az lwIP kuldopuffere alapbol 5744 bajt. Az rtl_tcp ag egy
 * blokkbol 256 * rtl_up * 2 bajtot csinal, ami mar 12x felmintavetelezesnel
 * tullepi ezt — a "csak akkor irok, ha az EGESZ befer" logika ilyenkor
 * SOHA nem teljesul, es a port csendben nemanak tunik.
 *
 * A gyuru megoldja: annyit irunk ki, amennyi epp befer, a farkat pedig a
 * TENYLEGESEN kiirt bajtszammal leptetjuk.
 *
 * A MINTAKERET-IGAZITAST NEM ITT VEDJUK. Csabito lenne csak egesz
 * mintakereteket kiirni, de az halalos: a write() barmennyit visszaadhat,
 * es ha a farok egyszer paratlan pozicioba kerul, a lefele-igazitas a
 * korbefordulasnal 0-t adna, a ciklus kilepne, es a gyuru SOHA tobbet nem
 * urulne. A helyes invarians odebb van: a ring_put CSAK EGESZ kereteket
 * tesz be, es CSAK EGESZ kereteket dob el — igy a bajtfolyam mindig
 * igazitott marad, akarhany darabban megy ki.
 *
 * A meretek kettohatvanyok: a head/tail uint32 korbefordulasa csak igy
 * marad konzisztens a modulo-val. Az rtl gyurunak tobb kell, mert egy
 * felmintavetelezett blokk 20x szorzonal mar 10 kB. */
#define RING_RAW_BYTES 16384u
#define RING_RTL_BYTES 65536u
/* Egy send() hivas felso korlatja. MSS-IGAZITVA: az lwIP TCP_MSS-e az
 * ESP-IDF-ben 1440 bajt, es NoDelay mellett minden iras azonnal
 * szegmensse valik. A regi 2048 igy egy teli 1440-esre es egy csonka
 * 608-asra esett szet — a felmeretu szegmensek levegoidot pazaroltak.
 * 2880 = pontosan 2 teli szegmens. */
#define TCP_CHUNK_MAX  2880u

typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint32_t seq;
  uint16_t nsamp;
  uint16_t decim;
  uint32_t fs_in_hz;
} blk_hdr_t;

#if RDY_ENABLE
static void IRAM_ATTR spi_rdy_up(spi_slave_transaction_t *t)
{
  (void)t;
  GPIO.out_w1ts = (1u << RDY_PIN);      /* RDY = 1: van hova irni */
}
static void IRAM_ATTR spi_rdy_down(spi_slave_transaction_t *t)
{
  (void)t;
  GPIO.out_w1tc = (1u << RDY_PIN);      /* RDY = 0: most epp nincs */
}
#endif

/* Elindult-e egyaltalan az SPI slave. Ha nem, a fo ciklus meg se probalja
 * — kulonben a driver masodpercenkent tobb ezer hibasort ont a konzolra. */
static bool spi_ok = false;

WORD_ALIGNED_ATTR static uint8_t rxbuf[NQUEUE][BLK_BYTES];
static spi_slave_transaction_t   trans[NQUEUE];

/* ===================================================================
 *        DEDIKALT SPI-VETELI TASK — AZ SPI SERTHETETLEN (2026-08-05)
 * ===================================================================
 *
 * A CW-splitteres meres (RTL-SDR vs FG23 egyszerre) feketen-feheren
 * megmutatta: az FG23 oldala TISZTA (100708 blokk, 0 eldobas, 0
 * FIFO-overflow), a lyukak 100%-ban ITT keletkeznek — masodpercenkent
 * ~1.3 db, egyenkent PONTOSAN 1 blokk (5.12 ms), mert a fo ciklus
 * (OLED ~22 ms + WiFi-kiszolgalas) neha tovabb all, mint amit az
 * NQUEUE=16 keszlet athidal (80 ms; mertunk 88 ms-os dt_max-ot).
 *
 * A javitas NEM ujabb NQUEUE-folt, hanem architektura: a vetel sajat,
 * MAGAS PRIORITASU, CORE-RA TUZOTT taskba kerul, amit a FreeRTOS
 * preemptiv utemezoje BARMIKOR azonnal beenged — az OLED/WiFi/print
 * fizikailag nem tud ele allni. A task dolga szandekosan minimalis:
 *
 *   get_trans_result (blokkolo, portMAX_DELAY)
 *     -> memcpy a gyurube (~µs)
 *     -> queue_trans AZONNAL vissza a DMA-nak
 *
 * Semmi demod, semmi print, semmi halozat — igy a WCET mikroszekundum
 * nagysagrendu, es a DMA-keszlet soha nem urul ki 1-2 tranzakcional
 * jobban. A fo ciklus (demod, APRS, SpyServer, OLED) a gyurubol
 * fogyaszt, sajat tempoban; ha megall 88 ms-ra, az mostantol csak
 * KESLELTETES, nem VESZTESEG.
 *
 * Core-valasztas: ugyanarra a magra tuzzuk, ahol az SPI ISR el (a
 * spi_slave_initialize a setup()-bol fut = Arduino core 1), igy az
 * ISR->task ebresztes mag-on beluli, nincs kereszt-mag IPC-lateencia.
 * A loop() ugyanitt fut prio 1-en -> a task (prio 22) barmikor
 * kiszakitja az OLED I2C-frissites kozepebol is. A WiFi-stack a
 * core 0-n marad, nem is talalkozunk vele.
 *
 * A gyuru merete a fo ciklus leghosszabb megallasat hidalja at:
 * 24 blokk = 123 ms @ 195 blokk/s (mert max: 88 ms + marge).
 * Ara: 24*1040 = ~25 kB RAM. SPSC gyuru, lock nelkul: a head-et csak
 * a task irja (release), a tail-t csak a loop() (release) — a masik
 * fel acquire-rel olvassa.
 *
 * KET KULON VEDELEM, NE KEVERD OSSZE (2026-08-07):
 *   RXRING  — akkor ved, ha a FO CIKLUS all (OLED, TCP, APRS). A task
 *             fut, tolti a gyurut. Merve: ovf = 0 az egesz ejszakas
 *             mereseben, tehat 24 blokk (123 ms) eleg.
 *   NQUEUE  — akkor ved, ha MAGA A TASK sem fut (flash-cache stall
 *             mindket magon). Ilyenkor csak a mar felfuzott tranzakciok
 *             fogynak. Ez volt 82 ms (=16), a mert akadas 88 ms — ezert
 *             lett 24, hogy ez is 123 ms legyen.
 * A ketto most szandekosan egyforma: a lanc nem szukul be sehol. */
#define RXRING_BLOCKS   24
#define SPI_RX_TASK_PRIO 22
#define SPI_RX_TASK_CORE 1          /* = ARDUINO_RUNNING_CORE, az ISR magja */

/* FIGYELEM az indexelesre: a head/tail SZABADON FUTO szamlalo (a
 * telitettseg 'head - tail'-bol jon, ami tulcsordulaskor is helyes),
 * de a SLOT-INDEXET NEM szabad belole '% RXRING_BLOCKS'-szal kepezni:
 * 24 nem osztoja 2^32-nek, igy a szamlalo atfordulasakor (195 blokk/s
 * mellett ~255 nap!) az index ugrana es a gyuru szetesne. Ezert a slot-
 * indexet mindket oldal SAJAT valtozoban lepteti es kezzel wrappeli —
 * ezt a szimulacios teszt 100 M blokkal, tobb 2^32-athaladassal
 * igazolta (0 veszteseg, helyes sorrend). */
WORD_ALIGNED_ATTR static uint8_t rxring[RXRING_BLOCKS][BLK_BYTES];
static uint16_t          rxring_len[RXRING_BLOCKS];
static volatile uint32_t rx_head = 0;      /* szamlalo, csak a task irja */
static volatile uint32_t rx_tail = 0;      /* szamlalo, csak a loop irja */
static uint32_t          rx_head_idx = 0;  /* slot, CSAK a task hasznalja */
static uint32_t          rx_tail_idx = 0;  /* slot, CSAK a loop hasznalja */
static volatile uint32_t rx_ring_ovf = 0;  /* gyuru tele -> eldobott blokk (0 kell legyen!) */
static volatile uint32_t rx_ring_max = 0;  /* legnagyobb kitoltes (vizjel) */
static TaskHandle_t      rx_task_handle = NULL;

static void spi_rx_task(void *arg)
{
  (void)arg;
  for (;;) {
    spi_slave_transaction_t *r = NULL;
    if (spi_slave_get_trans_result(FG23_SPI_HOST, &r, portMAX_DELAY) != ESP_OK
        || r == NULL)
      continue;

    size_t n = (size_t)(r->trans_len / 8);
    if (n > BLK_BYTES) n = BLK_BYTES;

    uint32_t head = rx_head;
    uint32_t used = head - __atomic_load_n(&rx_tail, __ATOMIC_ACQUIRE);
    if (used < RXRING_BLOCKS) {
      rxring_len[rx_head_idx] = (uint16_t)n;
      memcpy(rxring[rx_head_idx], r->rx_buffer, n);
      if (++rx_head_idx == RXRING_BLOCKS) rx_head_idx = 0;
      __atomic_store_n(&rx_head, head + 1u, __ATOMIC_RELEASE);
      if (used + 1u > rx_ring_max) rx_ring_max = used + 1u;
    } else {
      /* A gyuru tele: a fo ciklus tul regota all. A blokk elveszik, de a
       * sorszam-lanc jelzi (SEQ-LYUK) — es az ovf szamlalo megmondja,
       * hogy MI voltunk, nem az SPI. Ennek 0-nak kell lennie.
       * (Nem '++': volatile-on az C++20 ota deprecated.) */
      rx_ring_ovf = rx_ring_ovf + 1u;
    }

    /* A puffer AZONNAL vissza a DMA-nak — ez a lenyeg. */
    spi_slave_queue_trans(FG23_SPI_HOST, r, portMAX_DELAY);
  }
}

#define DIAG  Serial0     /* CP2102: szoveges diagnosztika */
#define RAW   Serial      /* nativ USB CDC: nyers folyam */

/* ---------------- kimeneti gyujtok ---------------- */
static uint8_t  agg_usb[AGG_BLK * BLK_BYTES];      static size_t agg_usb_n = 0;
#if RTL_ENABLE
static uint8_t  agg_rtl[BLK_SAMPLES * RTL_MAX_UP * 2];  /* atmeneti */
#else
static uint8_t  agg_rtl[4];                             /* csokeveny */
#endif
static uint32_t last_flush_ms = 0;

/* ---------------- gyurupuffer a ket TCP kimenetnek ---------------- */
typedef struct {
  uint8_t *buf;
  uint32_t size;          /* KETTOHATVANY */
  uint32_t head;          /* ide irunk */
  uint32_t tail;          /* innen olvasunk */
  uint32_t dropped;       /* hany blokknyi adat nem fert be */
} ring_t;

static uint8_t ring_raw_buf[RING_RAW_BYTES];
static ring_t  ring_raw = { ring_raw_buf, RING_RAW_BYTES, 0, 0, 0 };
#if RTL_ENABLE
static uint8_t ring_rtl_buf[RING_RTL_BYTES];
static ring_t  ring_rtl = { ring_rtl_buf, RING_RTL_BYTES, 0, 0, 0 };
#else
/* Csokeveny: a kodutak valtozatlanul fordulnak, de RAM-ot nem esznek.
 * Kliens sosem csatlakozhat (a szerver el sem indul), tehat ezekbe
 * soha nem irunk erdemben. */
static uint8_t ring_rtl_buf[16];
static ring_t  ring_rtl = { ring_rtl_buf, 16, 0, 0, 0 };
#endif

static inline uint32_t ring_used(const ring_t *r)
{
  return (r->head - r->tail);
}

/* Beir, ha ELFER EGESZBEN. Reszleges beiras tilos: a fejlec nelkuli
 * folyamnal egy fel minta orokre felcserelne I-t es Q-t. Ez az EGYETLEN
 * hely, ahol az igazitasrol gondoskodni kell. */
static void ring_put(ring_t *r, const uint8_t *p, uint32_t n)
{
  if (ring_used(r) + n > r->size) { r->dropped++; return; }
  uint32_t idx = r->head & (r->size - 1u);
  uint32_t first = r->size - idx;
  if (first > n) first = n;
  memcpy(&r->buf[idx], p, first);
  if (n > first) memcpy(&r->buf[0], p + first, n - first);
  r->head += n;
}

/* Kiuriti, amennyi epp befer. A farkat a TENYLEGESEN kiirt bajtszammal
 * lepteti, tehat egy reszleges iras sem csusztat el semmit — es NEM
 * igazitunk lefele, mert az a korbefordulasnal orokre beragasztana a
 * gyurut (lasd a fenti magyarazatot). */
static uint32_t ring_flush(ring_t *r, WiFiClient &c)
{
  /* SOHA NEM BLOKKOLUNK — es ehhez a WiFiClient::write() NEM hasznalhato.
   *
   * 2026-08-03, Android-spyserver merés: a fo ciklus spy=90..330 ms-okra
   * allt meg, qmax=16/16, VESZT zaporozott. Ok (a core forrasabol):
   *   - availableForWrite() NINCS implementalva a NetworkClient-ben,
   *     mindig 0-t ad — a regi "room" ellenorzes vak volt;
   *   - a write() tele kuldopuffernel select()-tel var, 1 MASODPERCES
   *     idokorlattal, 10 ujraprobalkozassal.
   * Egy energiatakarekos telefon 100-300 ms-onkent urit, tehat a write()
   * rendszeresen beleallt a varakozasba, es vitte magaval az SPI-t is.
   *
   * A gyogymod: kozvetlen send() MSG_DONTWAIT-tel. Ha a kuldopuffer tele
   * van, AZONNAL -1/EAGAIN jon, mi pedig megyunk a dolgunkra — a gyuru
   * tartja az adatot, a farok a TENYLEG kiirt bajtszammal leptet. */
  uint32_t sent = 0;
  const int fd = c.fd();
  if (fd < 0) return 0;
  while (ring_used(r) > 0) {
    uint32_t n = ring_used(r);
    uint32_t idx = r->tail & (r->size - 1u);
    uint32_t contig = r->size - idx;            /* a korbefordulasig */
    if (n > contig)        n = contig;
    if (n > TCP_CHUNK_MAX) n = TCP_CHUNK_MAX;

    int w = send(fd, &r->buf[idx], n, MSG_DONTWAIT);
    if (w <= 0) break;                          /* tele: majd a kov. korben */
    r->tail += (uint32_t)w;
    sent += (uint32_t)w;
    if ((uint32_t)w < n) break;                 /* reszleges: megtelt */
  }
  return sent;
}

/* ---------------- halozat ---------------- */
static WiFiServer srv_raw(PORT_RAW);
static WiFiServer srv_rtl(PORT_RTLTCP);
static WiFiClient cli_raw;
static WiFiClient cli_rtl;
/* KULON flag kell: a WiFiClient::operator bool() az arduino-esp32-ben
 * connected()-et ad vissza, tehat a "cli && !cli.connected()" mindig
 * hamis — a lecsatlakozas eszrevetlen maradna. */
static bool       have_raw = false, have_rtl = false;
static bool       wifi_up = false;
static bool       wifi_is_ap = false;
static IPAddress  my_ip;
static WiFiMulti  wifiMulti;
static uint32_t   wifi_last_try_ms = 0;

/* ---------------- statisztika ---------------- */
static uint32_t st_blocks = 0, st_badmagic = 0, st_badlen = 0, st_lost = 0;

/* KUMULATIV tukrok a wifi_bench-nek. A fenti szamlalok a 2 mp-es statusz
 * vegen nullazodnak; a bench viszont 10+ mp-es lepcsokben kepez deltat,
 * es ezek nelkul negativ (atfordulo) kulonbseget kapna. Csak nonek. */
static uint32_t bench_tot_blocks = 0, bench_tot_lost = 0;
static uint32_t bench_tot_ro_dup = 0, bench_tot_ro_hole = 0;

/* A TENYLEGES kimeneti mintavetel (decimalas utan), a 2 mp-es statuszbol.
 * A blokk fejleceben csak a BEMENETI rata van (400 ksps), az a panel
 * frekvenciaskalajahoz hasznalhatatlan. */
static uint32_t st_sps_out = 50000;
static uint32_t st_specline = 0;        /* fogadott scan-sorok (SPECLINE) */
static uint32_t st_specline_ms = 0;     /* az utolso erkezese */
static uint32_t st_specline_cf_hz = 0;  /* utolso sor kozepfrekvenciaja */
static uint32_t st_specline_span_hz = 0;
#if OLED_ENABLE
static OledSpectrum oled_spec;          /* 128x64 mini-spektrum + waterfall */
#endif
static uint32_t st_samples = 0;
static uint32_t st_usbdrop = 0;
static uint64_t st_usbbytes = 0, st_rawbytes = 0, st_rtlbytes = 0;
static int32_t  st_peak = 0;
static uint64_t st_abssum = 0;
static uint32_t st_counted = 0;
static int      st_rf_rssi = -128;   /* a FG23 RAIL-RSSI-je (dBm), a fejlecbol */

/* --- Jelszint dB-ben. A csucs/atlag nyers int16 ADC-magnitudo (teljes
 * kiteres = 32767). Ehhez kepest a dBFS (0 = telites, negativ = alatta)
 * azonnal ertelmes es KALIBRACIO NELKUL is pontos.
 *
 * VALODI dBm-hez egyszer kalibralni kell: adj ismert szintu jelet (pl. a
 * HackRF-harness kalibralt kimenetevel + kulso csillapito), olvasd le a
 * dBFS-t, es szamold: RX_FS_DBM = ismert_dBm - leolvasott_dBFS (ez az a
 * dBm, ami a teljes kiterest adna). Utana RX_CALIBRATED 1, es a kijelzo
 * dBm-et ir. Amig 0, a kimenet dBFS. */
#define RX_CALIBRATED  0
#define RX_FS_DBM      0.0f
#if RX_CALIBRATED
#define LVL_UNIT "dBm"
#else
#define LVL_UNIT "dBFS"
#endif
static inline float lvl_db(int32_t mag)
{
  if (mag < 1) mag = 1;
  float dbfs = 20.0f * log10f((float)mag / 32767.0f);
#if RX_CALIBRATED
  return dbfs + RX_FS_DBM;
#else
  return dbfs;
#endif
}
static uint32_t last_seq = 0;
static bool     have_seq = false;
static uint8_t  st_gap_shown = 0;
static uint32_t t0 = 0;

/* --- IDOZITES-NYOMOZAS ---
 * A blokkvesztes MINDIG idozites, sosem RF. Ezert nem talalgatunk: meg-
 * merjuk, mennyi ideig all a fo ciklus, es MELYIK reszen.
 *
 * st_qmax  — a legnagyobb egyszerre leszedheto tranzakcio-torlodas. Ha ez
 *            eleri az NQUEUE-t, epp most vesztettel adatot.
 * st_dtmax — a leghosszabb ket ciklus kozti szunet [us]. Ez a "mennyi
 *            ideig voltunk vakok" felso becslese.
 * sect_us  — szakaszonkenti maximum [us]: melyik hivas eszi meg. */
static uint32_t st_back = 0;

/* --- KESZLET-UJJLENYOMAT ---
 * Minden rendellenes sorszamlepesrol eldontjuk, hogy illik-e a
 * k * NQUEUE +- 1 kepletre. Ha a rendellenessegek szinte mind ilyenek,
 * akkor a hiba a tranzakcio-keszlet korbefordulasahoz van kotve — es ez
 * akkor is igaz marad, ha NQUEUE-t atallitjuk. Pont ez a kiserlet. */
static uint32_t st_pool_hit = 0, st_pool_other = 0;

static inline void seq_classify(uint32_t step_abs)
{
  if (step_abs < 2u) return;
  uint32_t k = (step_abs + NQUEUE / 2u) / NQUEUE;
  if (k == 0u) k = 1u;
  uint32_t near = k * (uint32_t)NQUEUE;
  uint32_t diff = (step_abs > near) ? (step_abs - near) : (near - step_abs);
  if (diff <= 1u) st_pool_hit++;
  else            st_pool_other++;
}
static uint32_t st_qmax = 0;
static uint32_t st_dtmax = 0;
static uint32_t st_prev_us = 0;

enum { S_PROC = 0, S_TCP, S_USB, S_ACC, S_SPY, S_RTL, S_WIFI, S_UI,
       S_APRS, S_CW, S_OLED, S_COUNT };
static const char *SECT_NAME[S_COUNT] = {
  "proc", "tcp", "usb", "acc", "spy", "rtl", "wifi", "ui", "aprs", "cw", "oled" };
static uint32_t sect_us[S_COUNT];

/* Meres egy szakaszra. A micros() hivas ~1 us, kororkent tiz darab
 * elhanyagolhato ahhoz kepest, amit keresunk (tobb tiz ms). */
#define SECT(idx, ...) do {                                    \
    uint32_t _t = micros();                                    \
    __VA_ARGS__;                                               \
    uint32_t _d = micros() - _t;                               \
    if (_d > sect_us[idx]) sect_us[idx] = _d;                  \
  } while (0)

/* ---------------- stream-parameterek a fejlecbol ---------------- */
static uint32_t cur_fs_in = 0, cur_decim = 0, cur_sps = 0;
static uint32_t rtl_up = 1;                 /* felmintavetelezesi szorzo */
static int      rtl_shift = 8;
static int32_t  rtl_peak_win = 0;
static uint32_t rtl_agc_ms = 0;
static int16_t  rtl_prev_i = 0, rtl_prev_q = 0;

/* Hangolasi keres a kliensrol: nem azonnal adjuk tovabb (lasd lentebb). */
#define TUNE_RATE_MS   250u
static bool     rtl_pending = false;
static uint32_t rtl_pending_khz = 0;
static uint32_t rtl_last_tune_ms = 0;

/* Kozos belepesi pont a hangolasi kereseknek (rtl_tcp ES spyserver).
 * Nem C++-nevtorzitassal, hogy a spy_tune_cb extern-nel elerje. */
extern "C" bool rtl_pending_set(uint32_t khz);
extern "C" bool rtl_pending_set(uint32_t khz)
{
  /* Az SDR++ csatlakozaskor 0 Hz-et kuld, mielott a felhasznalo barmit
   * beallitana. Ezt tovabbadni annyi, hogy a FG23-nak "F0"-t kuldunk —
   * ertelmetlen parancs, felesleges stream-ujrainditas. Csak ertelmes
   * frekvenciat engedunk at. */
  if (khz < 100000u || khz > 1000000u) {
    DIAG.printf("\nhangolas figyelmen kivul: %lu kHz (a sávon kivul)\n",
                (unsigned long)khz);
    return false;
  }
  rtl_pending_khz = khz;
  rtl_pending = true;
  return true;
}

/* ===================================================================
 *                             OLED
 * =================================================================== */
#if OLED_ENABLE

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C
       u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);

static bool     oled_ok = false;
static uint32_t oled_last_ms = 0;
static uint32_t oled_khz = 0;        /* utolso ismert hangolas, 0 = nem tudjuk */

static void oled_init(void)
{
#if (OLED_VEXT_PIN >= 0)
  /* A kijelzo tapja. Heltec V3-on aktiv ALACSONY. */
  pinMode(OLED_VEXT_PIN, OUTPUT);
  digitalWrite(OLED_VEXT_PIN, OLED_VEXT_ACTIVE_LOW ? LOW : HIGH);
  delay(100);
#endif

  /* A resetet kezzel adjuk, nem a U8g2-re bizzuk: igy biztosan a Vext
   * bekapcsolasa UTAN tortenik, es lathato helyen van. */
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);   delay(20);
  digitalWrite(OLED_RST_PIN, HIGH);  delay(20);

  /* VALODI jelenlet-ellenorzes, mielott a U8g2-t hivnank. A U8g2 begin()-je
   * mindig igazat ad — nem nez vissza a buszra. Enelkul nem tudnad
   * megkulonboztetni a "nincs bekotve" esetet a "Vext nincs bekapcsolva"
   * esettol, es a sotet kijelzo mindkettonel ugyanugy nez ki. */
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.beginTransmission(0x3C);
  oled_ok = (Wire.endTransmission() == 0);

  if (oled_ok) {
    u8g2.begin();
    u8g2.setBusClock(OLED_I2C_HZ);
    u8g2.setFontMode(1);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.drawStr(0, 12, "FG23 SDR");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 26, "HA7DCD  indulas...");
    u8g2.sendBuffer();
  }
  DIAG.printf("OLED: %s a 0x3C cimen (SDA=%d SCL=%d RST=%d Vext=%d)\n",
              oled_ok ? "valaszol" : "NEM VALASZOL",
              OLED_SDA_PIN, OLED_SCL_PIN, OLED_RST_PIN, OLED_VEXT_PIN);
  if (!oled_ok) {
    DIAG.println("  -> probald: OLED_VEXT_ACTIVE_LOW megforditva, vagy "
                 "OLED_VEXT_PIN = -1");
  }
}

/* ================== A KIJELZO OLDALAI ==================
 * Minden oldalnak sajat rajzolo fuggvenye van, es a ui.h tablazata mondja
 * meg, melyik letezik es milyen sorrendben. Uj demodulatornal ide kerul egy
 * uj rajzolo, es a lenti switchbe egy sor. */

/* ================== SZOVEGKIIRAS, AMI BIZTOSAN KIFER ==================
 * A kijelzo 128 pixel szeles. A 6x10-es fonttal ez PONTOSAN 21 karakter
 * (126 px) — a 22. mar felbe vagva latszik a szelen. Kezzel szamolgatni
 * minden format-stringet rossz otlet: eleg egy hosszabb hivojel vagy egy
 * negyjegyu szamlalo, es megint levagja.
 *
 * Ezert nem szamolunk: a fuggveny megmeri a szoveget, es ha nem fer ki
 * 6x10-zel, atvalt 5x8-ra (25 karakter), majd 4x6-ra (32). Ha meg ugy sem
 * fer, KARAKTERHATARON vagja le, nem a betu kozepen. */
static void oled_fit(int x, int y, const char *s)
{
  const int avail = 128 - x;
  int len = (int)strlen(s);

  const uint8_t *font; int cw;
  if      (len * 6 <= avail) { font = u8g2_font_6x10_tf; cw = 6; }
  else if (len * 5 <= avail) { font = u8g2_font_5x8_tf;  cw = 5; }
  else                       { font = u8g2_font_4x6_tf;  cw = 4; }

  char tmp[48];
  int maxch = avail / cw;
  if (maxch > (int)sizeof tmp - 1) maxch = (int)sizeof tmp - 1;
  if (len > maxch) {
    memcpy(tmp, s, (size_t)maxch);
    tmp[maxch] = '\0';
    s = tmp;
  }
  u8g2.setFont(font);
  u8g2.drawStr(x, y, s);
}

/* Kozos fejlec: oldalnev balra, uzemmod-jelzo jobbra. Igy barmelyik
 * oldalon latod, MI FUT epp — nem csak azt, mit nezel. */
static void oled_header(const char *title)
{
  ui_page_t m = ui_mode();
  const char *tag = (m == UI_PAGE_STATUS) ? "---" : ui_page_name(m);

  /* A jelzot JOBBRA igazitjuk, a tenyleges pixelszelessege alapjan —
   * nem karakterszambol becsulve. Az 5x8 azert kell, hogy a hosszabb
   * nevek (pl. "APRS iGATE") mellett is maradjon hely a cimnek. */
  u8g2.setFont(u8g2_font_5x8_tf);
  int tw = (int)u8g2.getStrWidth(tag);
  u8g2.drawStr(128 - tw, 8, tag);

  /* a cim annyi helyet kap, ami megmaradt */
  char t[32];
  snprintf(t, sizeof t, "%s", title);
  int room = 128 - tw - 4;
  u8g2.setFont(u8g2_font_6x10_tf);
  while (t[0] && (int)u8g2.getStrWidth(t) > room) t[strlen(t) - 1] = '\0';
  u8g2.drawStr(0, 8, t);

  u8g2.drawHLine(0, 10, 128);
}

/* ---- STATUS: ez az indulo kep. A LEGFONTOSABB az IP, mert azt kell
 * atgepelni az SDR++-ba. ---- */
static void oled_page_status(void)
{
  char buf[40];
  oled_header("FG23 SDR");

  if (wifi_is_ap) {
    snprintf(buf, sizeof buf, "AP %s", WIFI_AP_SSID);
  } else if (wifi_up) {
    snprintf(buf, sizeof buf, "%.13s %ddBm",
             WiFi.SSID().c_str(), (int)WiFi.RSSI());
  } else {
    snprintf(buf, sizeof buf, "nincs halozat");
  }
  oled_fit(0, 21, buf);

  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 35, wifi_up ? my_ip.toString().c_str() : "-");

#if SPY_ENABLE
  snprintf(buf, sizeof buf, "5555 spy%c 8888 1234",
           spy_connected() ? '+' : ' ');
#else
  snprintf(buf, sizeof buf, "8888 i16   1234 x%lu", (unsigned long)rtl_up);
#endif
  oled_fit(0, 46, buf);

  if (cur_sps) snprintf(buf, sizeof buf, "%lu sps", (unsigned long)cur_sps);
  else         snprintf(buf, sizeof buf, "nincs adat");
  oled_fit(0, 57, buf);
  if (oled_khz) {
    snprintf(buf, sizeof buf, "%lu.%03lu",
             (unsigned long)(oled_khz / 1000), (unsigned long)(oled_khz % 1000));
    oled_fit(74, 57, buf);
  }
}

/* ---- RF: a FIZIKAI parameterek. Ez hianyzott, es pont ez a legfontosabb,
 * amikor a keszulek nem a szamitogep mellett van: hol all a vevo, milyen
 * szeles a sav, es mennyi jon be. ---- */
static void oled_page_rf(void)
{
  char buf[40];

  /* Scan alatt (friss SPECLINE < 2 s) a RF-lap = mini-spektrum + waterfall.
   * A draw() maga clearBuffer/sendBuffer-t hiv, ezert utana visszaterunk;
   * az oled_draw() masodik sendBuffer-e ugyanazt a kepet kuldi ujra. */
  if (st_specline && millis() - st_specline_ms < 2000u) {
    oled_spec.draw(u8g2);
    return;
  }

  oled_header("RF");

  /* frekvencia nagy betuvel, MHz-ben harom tizedessel */
  u8g2.setFont(u8g2_font_7x13B_tr);
  if (oled_khz) {
    snprintf(buf, sizeof buf, "%lu.%03lu MHz",
             (unsigned long)(oled_khz / 1000), (unsigned long)(oled_khz % 1000));
  } else {
    snprintf(buf, sizeof buf, "-- MHz");
  }
  u8g2.drawStr(0, 24, buf);

  /* mintavetel es a belole adodo sav */
  snprintf(buf, sizeof buf, "%lu sps  R=%lu",
           (unsigned long)cur_sps, (unsigned long)cur_decim);
  oled_fit(0, 35, buf);

  snprintf(buf, sizeof buf, "sav +-%lu.%lu kHz  fs %lu k",
           (unsigned long)(cur_sps / 2000), (unsigned long)((cur_sps / 200) % 10),
           (unsigned long)(cur_fs_in / 1000));
  oled_fit(0, 45, buf);

  /* Szintek. A csucs a LEGFONTOSABB szam a tulvezerles ellen: ha eleri a
   * 32767-et, a fazis a negy sarokra ragad, es a demodulacio hasznalhatatlan
   * lesz — akkor is, ha a vizeses meg szepnek latszik. */
  uint32_t mean = st_counted ? (uint32_t)(st_abssum / st_counted) : 0;
  snprintf(buf, sizeof buf, "%+d dBm  %+.0f/%+.0f%s",
           st_rf_rssi, lvl_db(st_peak), lvl_db((int32_t)mean),
           (st_peak >= 32000) ? " T!" : "");
  oled_fit(0, 56, buf);
}

/* ---- SDR STREAM: a lanc egeszsege egy kepen. ---- */
static void oled_page_sdr(void)
{
  char buf[40];
  oled_header("SDR STREAM");

  u8g2.setFont(u8g2_font_7x13B_tr);
  snprintf(buf, sizeof buf, "%lu sps", (unsigned long)cur_sps);
  u8g2.drawStr(0, 24, buf);

  snprintf(buf, sizeof buf, "sav +-%lu.%lu kHz",
           (unsigned long)(cur_sps / 2000), (unsigned long)((cur_sps / 200) % 10));
  oled_fit(0, 35, buf);

  snprintf(buf, sizeof buf, "%+.0f " LVL_UNIT "  R=%lu",
           lvl_db(st_peak), (unsigned long)cur_decim);
  oled_fit(0, 46, buf);

#if SPY_ENABLE
  snprintf(buf, sizeof buf, "NET%c RTL%c SPY%c USB%c v%lu",
           have_raw ? '+' : '-', have_rtl ? '+' : '-',
           spy_connected() ? '+' : '-',
           (st_usbdrop == 0) ? '+' : '!', (unsigned long)st_lost);
#else
  snprintf(buf, sizeof buf, "NET%c RTL%c USB%c  v%lu",
           have_raw ? '+' : '-', have_rtl ? '+' : '-',
           (st_usbdrop == 0) ? '+' : '!', (unsigned long)st_lost);
#endif
  oled_fit(0, 57, buf);
}

#if APRS_RX_ENABLE
/* ---- APRS iGATE: a hivojel a legnagyobb betuvel, mert az az erdekes. ---- */
static void oled_page_aprs(void)
{
  char buf[40];
  oled_header("APRS iGATE");

  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 24, aprs_rx_last_call()[0] ? aprs_rx_last_call()
                                             : "-- varakozas --");

  /* Az IS-allapot IDE kerul, a sor vegere — nem kulon drawStr-rel valahova
   * a kepbe, mert az atlapolna az info-mezot, amint az hosszabb lesz. Igy
   * az egesz sor egyben megy at az oled_fit-en, es garantaltan kifer.
   *
   * ROVIDITVE 2026-08-07. A regi szoveg ("vett 1197 hiba 11 gate 0 IS+")
   * 28 karakter volt. Az oled_fit letraja x=0-nal 128 pixellel dolgozik:
   * 28*6=168 es 28*5=140 egyarant tul sok, ezert a LEGKISEBB, 4x6-os
   * beture esett vissza — annak a kisbetu-x-magassaga 3 pixel, ami
   * olvashatatlan. Ez volt a "picik a kisbetuk" oka, nem a helyhiany.
   *
   * 21 karakter alatt a 6x10 marad (21*6=126 <= 128), aminel a kisbetuk
   * x-magassaga 5 pixel — tobb mint ketszeres. Ezert rovid, KETTOSPONTOS
   * cimkek:
   *   RX = vett keret,  H = CRC-hiba,  G = APRS-IS-re gatelve.
   *
   * KARAKTER-KOLTSEGVETES (ezt tartsd, ha atirod a cimkeket):
   *   "RX:1197 H:11 G:0 IS+"      = 20 karakter -> 6x10  (jol olvashato)
   *   "RX:99999 H:9999 G:999 IS+" = 25 karakter -> 5x8   (meg olvashato)
   * Vagyis a tipikus uzem vegig a nagyobb betuvel megy, es csak nagyon
   * nagy szamlaloknal lep vissza egyet — 4x6-ra sosem esik.
   *
   * Nagyobb betu (7x13B) ide NEM megy: a sorosztas 11 pixel (24/35/46/57),
   * egy 13 pixel magas glif atlapolna a folotte levo hivojelet. Ha valaha
   * nagyobb kell, eloszor a sorosztast kell attervezni. */
  snprintf(buf, sizeof buf, "RX:%lu H:%lu G:%lu %s",
           (unsigned long)aprs_rx_frames(), (unsigned long)aprs_rx_bad(),
           (unsigned long)aprs_rx_gated(),
           aprs_rx_is_online() ? "IS+" : "IS-");
  oled_fit(0, 35, buf);

  const char *info = aprs_rx_last_info();
  char l1[22], l2[22];
  snprintf(l1, sizeof l1, "%.21s", info);
  snprintf(l2, sizeof l2, "%.21s", (strlen(info) > 21) ? info + 21 : "");
  oled_fit(0, 46, l1);
  oled_fit(0, 57, l2);
}
#endif

/* ---- CW dekoder oldal ---- */
static void oled_page_cw(void)
{
  char buf[40];
  oled_header("CW");

  /* A dekodolt szoveg a legnagyobb betuvel */
  u8g2.setFont(u8g2_font_7x13B_tr);
  const char *txt = cw_rx_text();
  if (txt[0]) {
    /* ha hosszu, az utolso 18 karaktert mutatjuk (gorgetes) */
    size_t n = strlen(txt);
    const char *show = (n > 18) ? txt + (n - 18) : txt;
    u8g2.drawStr(0, 26, show);
  } else {
    u8g2.drawStr(0, 26, "-- varakozas --");
  }

  /* WPM + key allapot + karakter szamlalo */
  snprintf(buf, sizeof buf, "%.0f WPM  %s  ch:%lu",
           (double)cw_rx_wpm(),
           cw_rx_keyed() ? "KEY" : "   ",
           (unsigned long)cw_rx_chars());
  oled_fit(0, 40, buf);

  /* A teljes puffer also sorban, kisebb betuvel */
  oled_fit(0, 54, txt[0] ? txt : "(nincs meg jel)");
}

/* ---- SCAN: teljes kepernyos ditherelt spektrum + waterfall ---- */
static void oled_page_scan(void)
{
  /* Friss SPECLINE (5 s): a OledSpectrum a teljes 128x64-et kitolti
   * (24 px gorbe + 40 px Bayer-ditherelt waterfall). */
  if (st_specline && (millis() - st_specline_ms) < 5000u) {
    oled_spec.draw(u8g2);
    return;
  }

  /* Nincs aktiv scan — utmutato */
  oled_header("SCAN");
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 28, "nincs scan");
  oled_fit(0, 44, "W<kHz>,span,nbin");
  if (st_specline) {
    char buf[40];
    snprintf(buf, sizeof buf, "utolso %lu s",
             (unsigned long)((millis() - st_specline_ms) / 1000u));
    oled_fit(0, 56, buf);
  } else {
    oled_fit(0, 56, "FG23 / SpyServer");
  }
}

/* ---- Meg nem kesz demodulatorok. Nem hazudunk rola: kiirjuk, hogy
 * nincs meg, es azt is, mi hianyzik hozza. ---- */
static void oled_page_todo(ui_page_t p)
{
  oled_header(ui_page_name(p));
  u8g2.setFont(u8g2_font_7x13B_tr);
  u8g2.drawStr(0, 26, "meg nincs kesz");
  switch (p) {
    case UI_PAGE_WSPR:
      oled_fit(0, 40, "tervben: 2 perces");
      oled_fit(0, 51, "FFT + K1JT dekoder,");
      oled_fit(0, 62, "pontos ora kell (NTP)");
      break;
    case UI_PAGE_FT8:
      oled_fit(0, 40, "tervben: 15 mp ciklus,");
      oled_fit(0, 51, "LDPC dekoder - ez mar");
      oled_fit(0, 62, "komoly CPU es RAM");
      break;
    default:
      break;
  }
}

static void oled_draw(void)
{
  if (!oled_ok) return;
  u8g2.clearBuffer();

  switch (ui_page()) {
    case UI_PAGE_RF:   oled_page_rf();   break;
    case UI_PAGE_SDR:  oled_page_sdr();  break;
    case UI_PAGE_SCAN: oled_page_scan(); break;
#if APRS_RX_ENABLE
    case UI_PAGE_APRS: oled_page_aprs(); break;
#endif
    case UI_PAGE_CW:   oled_page_cw();   break;
    case UI_PAGE_WSPR:
    case UI_PAGE_FT8:  oled_page_todo(ui_page()); break;
    default:           oled_page_status(); break;
  }

  u8g2.sendBuffer();
}

static void oled_tick(uint32_t now)
{
  if (!oled_ok) return;
  /* SCAN lapon gyorsabb frissites (~3 Hz), hogy a waterfall folyjón */
  uint32_t period = (ui_page() == UI_PAGE_SCAN) ? 300u : OLED_REFRESH_MS;
  if (now - oled_last_ms < period) return;
  oled_last_ms = now;
  oled_draw();
}

/* Azonnali ujrarajzolas — gombnyomas utan ne kelljen varni a kovetkezo
 * periodusra, mert az ugy erezne, mintha nem mukodne a gomb. */
static void oled_force(void)
{
  if (!oled_ok) return;
  oled_last_ms = millis();
  oled_draw();
}

#else
static void oled_init(void) { }
static void oled_tick(uint32_t now) { (void)now; }
static void oled_force(void) { }
static uint32_t oled_khz = 0;
#endif  /* OLED_ENABLE */

/* ===================================================================
 *                          VISSZAIRANY
 * =================================================================== */

static void cmdlink_send(const char *s)
{
#if CMDLINK_ENABLE
  /* NINCS flush(): az blokkolna, amig az utolso bit is kimegy (~1 ms), es
   * kozben allna a fo ciklus. A pufferelt iras eleg — a FG23-nak nem
   * surgos ezredmasodpercen belul. */
  Serial1.print(s);
  Serial1.print("\r");
#endif
  DIAG.printf("  -> FG23: %s\n", s);
}

#if SPY_ENABLE
/* A SpyServer-kliens hangolasi kerese. UGYANAZON a torlesztett uton megy,
 * mint az rtl_tcp-e: a csuszka huzasa kozben osszegyult keresekbol csak az
 * utolso jut el a FG23-ig. Kulonben minden pixelnyi mozdulat egy
 * stream-ujrainditast jelentene. */
static void spy_tune_cb(uint32_t hz)
{
  rtl_pending_set((hz + 500u) / 1000u);
}

/* A SpyServer-kliens scan-modot ker (SDR++ fg23_scan_source: SCAN) vagy
 * leallitja. Kozvetlenul a cmdlinken megy a FG23-nak — a hangolassal
 * ellentetben itt nincs "csuszka-huzas", tehat nem kell torleszteni. A
 * FG23 a "W..." parancsra leallitja az I/Q-streamet es scanel; "W0"-ra
 * visszaall es a stream ujraindul. */
static bool     scan_on = false;
static uint32_t scan_center_khz = 0, scan_span_khz = 0;
static uint16_t scan_nbin = 0;
/* TORLESZTETT kuldes (2026-08-15): az SDR++ a vizeses huzasakor masod-
 * percenkent tobb W-parancsot ad, a FG23 EUSART FIFO-ja (16 bajt) pedig
 * tulcsordult -> serult sorok ("cmdlink: 39 sor, 20 hiba"). Ugyanaz a
 * megoldas, mint a hangolasnal: a fuggo parancsokbol csak az UTOLSO megy
 * ki, legfeljebb SCAN_RATE_MS-enkent. A W0 azonnal megy. */
#define SCAN_RATE_MS 300u
static bool     scan_pending = false;
static char     scan_pending_cmd[40];
static uint32_t scan_last_send_ms = 0;
static void spy_scan_cb(bool want, uint32_t center_hz, uint32_t span_hz,
                        uint16_t nbin, int16_t floor_dbm, uint16_t range_db)
{
  if (want) {
    uint32_t ck = (center_hz + 500u) / 1000u;
    uint32_t sk = (span_hz + 500u) / 1000u;
    if (ck < 100000u || ck > 1000000u) {
      DIAG.printf("\nscan: kozep %lu kHz a savon kivul, nem inditom\n", (unsigned long)ck);
      return;
    }
    snprintf(scan_pending_cmd, sizeof scan_pending_cmd, "W%lu,%lu,%u,%d,%u",
             (unsigned long)ck, (unsigned long)sk, (unsigned)nbin,
             (int)floor_dbm, (unsigned)range_db);
    scan_pending = true;
    scan_on = true; scan_center_khz = ck; scan_span_khz = sk; scan_nbin = nbin;
    oled_khz = ck;
  } else {
    scan_pending = false;
    if (!scan_on) return;
    scan_on = false;
    DIAG.printf("\nscan: W0\n");
    cmdlink_send("W0");
    scan_last_send_ms = millis();
  }
}

/* A fo ciklusbol: a fuggo scan-parancs kikuldese torlesztve. */
static void scan_flush(uint32_t now)
{
  if (!scan_pending) return;
  if (now - scan_last_send_ms < SCAN_RATE_MS) return;
  scan_pending = false;
  scan_last_send_ms = now;
  DIAG.printf("\nscan: %s\n", scan_pending_cmd);
  cmdlink_send(scan_pending_cmd);
}
#endif

/* ===================================================================
 *                            WIFI
 * =================================================================== */

/* Sajat AP nyitasa, ha egyik ismert halozat sincs a kozelben. */
static void wifi_open_ap(void)
{
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
#if WIFI_HT40
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);   /* lasd wifi_start */
#endif
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  wifi_up = true; wifi_is_ap = true; my_ip = WiFi.softAPIP();
  DIAG.printf("WiFi AP: \"%s\"  jelszo \"%s\"   IP %s\n",
              WIFI_AP_SSID, WIFI_AP_PASS, my_ip.toString().c_str());
}

/* A WiFi ujracsatlakozo task ELOREDEKLARACIOJA.
 *
 * A tenyleges definicio es a reszletes indoklas lentebb van, a wifi_tick()
 * elott — de a wifi_start() MAR ITT elinditja, ezert a nevnek es a
 * makroknak eddigre lataniuk kell. (2026-08-07: eloszor lentre tettem
 * mindent, es a fordito jogosan szolt, hogy a wifi_start()-ban meg nem
 * ismert egyik sem.) */
static volatile bool wifi_scan_req    = false;   /* fo ciklus -> task */
static TaskHandle_t  wifi_task_handle = NULL;

#define WIFI_SCAN_TIMEOUT_MS 5000u
#define WIFI_TASK_PRIO       1                 /* alacsony: senkit nem zavar */
#define WIFI_TASK_CORE       0                 /* NEM az SPI/loop magja! */

#if WIFI_ENABLE
static void wifi_task(void *arg);
#endif

static void wifi_start(void)
{
#if !WIFI_ENABLE
  DIAG.println("WiFi: kikapcsolva (WIFI_ENABLE=0)");
#else
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          /* KELL: alvas nelkul stabil az atvitel */

#if WIFI_HT40
  /* Lasd a WIFI_HT40 kapcsolonal: alapbol KI, mert a merés szerint
   * nyereseg nelkul sokszorozta a driver-glitchet. */
  esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
  DIAG.println("WiFi: HT40 kerese (KISERLETI — figyeld a HATRA-t!)");
#endif

  int n = 0;
  for (size_t i = 0; i < WIFI_AP_COUNT; i++) {
    if (WIFI_APS[i].ssid[0] == '\0') continue;   /* ures sor: kihagyva */
    wifiMulti.addAP(WIFI_APS[i].ssid, WIFI_APS[i].pass);
    DIAG.printf("WiFi: ismert halozat #%d  \"%s\"\n", ++n, WIFI_APS[i].ssid);
  }

  if (n == 0) {
    DIAG.println("WiFi: nincs egyetlen halozat sem a tablazatban -> AP");
    wifi_open_ap();
  } else {
    DIAG.printf("WiFi: pasztazas, a legerosebb ismert halozatra megyunk "
                "(max %lu ms)...\n", (unsigned long)WIFI_STA_TIMEOUT_MS);
    wifi_last_try_ms = millis();
    if (wifiMulti.run(WIFI_STA_TIMEOUT_MS) == WL_CONNECTED) {
      wifi_up = true; wifi_is_ap = false; my_ip = WiFi.localIP();
      DIAG.printf("WiFi STA: \"%s\"   IP %s   RSSI %d dBm\n",
                  WiFi.SSID().c_str(), my_ip.toString().c_str(),
                  (int)WiFi.RSSI());
      /* Ha ugyanaz az AP esik ki es jon vissza, ezt a mag maga intezi,
       * blokkolas nelkul. A WiFiMulti csak a VALASZTASHOZ kell. */
      WiFi.setAutoReconnect(true);
    } else {
      DIAG.println("WiFi: egyik ismert halozat sem elerheto -> sajat AP");
      wifi_open_ap();
    }
  }

  /* Az ujracsatlakozo task. Akkor is elindul, ha most AP-modban vagyunk:
   * uresen jar, es a wifi_tick ugyis csak STA-modban jelez neki. Ha nem
   * indul el, NEM esunk vissza csendben a blokkolo utra — inkabb kiabalunk,
   * mert az a valtozat bizonyitottan 5 masodperces adatvesztest okoz. */
  if (xTaskCreatePinnedToCore(wifi_task, "wifi_rc", 4096, NULL,
                              WIFI_TASK_PRIO, &wifi_task_handle,
                              WIFI_TASK_CORE) != pdPASS) {
    DIAG.println("!! wifi_rc task NEM indult el — nincs ujracsatlakozas!");
    wifi_task_handle = NULL;
  } else {
    DIAG.printf("WiFi: ujracsatlakozo task fut (mag %d, prio %d) — "
                "a pasztazas TOBBE NEM allitja meg a fo ciklust\n",
                WIFI_TASK_CORE, WIFI_TASK_PRIO);
  }

  srv_raw.begin();  srv_raw.setNoDelay(true);
#if RTL_ENABLE
  srv_rtl.begin();  srv_rtl.setNoDelay(true);
#endif
#endif
}

/* ===================================================================
 *   WIFI UJRACSATLAKOZAS — KULON TASKBAN, NEM A FO CIKLUSBAN
 * ===================================================================
 *
 * MIERT (MERT 2026-08-07). A regi valtozat a fo ciklusbol hivta a
 * wifiMulti.run(5000)-t. Az PASZTAZ es a megadott timeoutig BLOKKOL. A
 * 195 perces soak naploja szo szerint ezt mutatta:
 *
 *   IDO: qmax=24/24  RXRING=24/24  ovf=959  dt_max=5030705 us
 *        | wifi=5029571 oled=20332
 *
 * A "wifi" szakasz 5 029 571 us = 5,03 s, es a dt_max gyakorlatilag
 * ugyanennyi: a szakasz nem TARTALMAZTA az akadast, hanem AZ VOLT az
 * akadas. A beallitott 5000 ms timeout koszon vissza benne.
 *
 * A kar: a dedikalt SPI-task 123 ms alatt teletolti a 24 blokkos gyurut,
 * utana ovf. Ket ilyen esemeny 195 perc alatt 959 + 939 blokkot vitt el
 * (4,9 s + 4,8 s), es 6-6 egymast koveto APRS keretet.
 *
 * AMIT NEM SZABAD HINNI: hogy ezt melyebb puffer megoldja. 5 masodperc a
 * 123 ms-hoz kepest NEGYVENSZERES. Nincs az az NQUEUE vagy RXRING, ami
 * ezt athidalja — a blokkolo hivasnak KI KELL KERULNIE a fo ciklusbol.
 * (A forras ezt mar megjosolta a blokkolo APRS-IS connectnel is.)
 *
 * A MEGOLDAS. A wifiMulti "ismertek kozul a legerosebbre megy" logikaja
 * ertekes, ezert nem eldobjuk, hanem athelyezzuk:
 *
 *   - a fo ciklus (wifi_tick) CSAK EGY JELZOT ALLIT, semmit nem hiv,
 *     ami blokkolhat;
 *   - a tenyleges pasztazas a wifi_task-ban fut, a 0. MAGON. A fo ciklus
 *     es az SPI-task egyarant az 1. magon van (ARDUINO_RUNNING_CORE=1,
 *     SPI_RX_TASK_CORE 1, prio 22), a WiFi/lwIP pedig amugy is a 0-ason.
 *   - a task nyugodtan blokkolhat 5 masodpercig: a fo ciklus kozben
 *     folyamatosan uriti a gyurut, tehat az ovf nem no.
 *
 * FONTOS, HOGY EZ MIT NEM VALT KI: a WiFi.setAutoReconnect(true) (lasd a
 * wifi_setup-ban) tovabbra is elintezi, ha UGYANAZ az AP esik ki es jon
 * vissza — azt a mag maga csinalja, blokkolas nelkul. A pasztazas csak
 * akkor kell, ha MASIK ismert halozatra kell atvaltani.
 *
 * A DIAG-ot ket task irja (fo ciklus + ez). A HardwareSerial-nak van TX
 * gyurupuffere, es a sorok rovidek, de elmeletben osszefesulodhetnek. Ha
 * valaha kevert sort latsz a naploban, ez az oka — nem adathiba. */
/* (A wifi_scan_req, a wifi_task_handle es a WIFI_TASK_* makrok fentebb,
 *  a wifi_start() elott vannak deklaralva — onnan indul a task.) */
#if WIFI_ENABLE
static void wifi_task(void *arg)
{
  (void)arg;
  for (;;) {
    if (wifi_scan_req) {
      DIAG.println("\nWiFi: kiestunk, ujrapasztazas... (kulon taskban)");
      if (wifiMulti.run(WIFI_SCAN_TIMEOUT_MS) == WL_CONNECTED) {
        my_ip = WiFi.localIP();
        DIAG.printf("WiFi vissza: \"%s\"   IP %s   RSSI %d dBm\n",
                    WiFi.SSID().c_str(), my_ip.toString().c_str(),
                    (int)WiFi.RSSI());
      } else {
        DIAG.println("WiFi: meg mindig nincs ismert halozat");
      }
      wifi_scan_req = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#endif

/* A fo ciklusbol. Csak DONT es JELEZ — soha nem blokkol. */
static void wifi_tick(uint32_t now)
{
#if WIFI_ENABLE
  if (wifi_is_ap) return;                       /* AP modban nincs mit tenni */
  if (WiFi.status() == WL_CONNECTED) return;
  if (have_raw || have_rtl) return;
  if (wifi_scan_req) return;                    /* mar fut egy probalkozas */
  if (now - wifi_last_try_ms < WIFI_RETRY_MS) return;

  wifi_last_try_ms = now;
  wifi_scan_req    = true;                      /* ennyi. Nem varunk ra. */
#else
  (void)now;
#endif
}

/* ===================================================================
 *                    rtl_tcp PROTOKOLL
 * =================================================================== */

static void rtl_send_header(WiFiClient &c)
{
  /* 12 bajt: "RTL0" + tuner tipus (big-endian) + gain-fokozatok szama.
   * R820T-nek adjuk ki magunkat: azt minden kliens ismeri. */
  uint8_t h[12] = { 'R','T','L','0',
                    0,0,0,5,        /* RTLSDR_TUNER_R820T */
                    0,0,0,29 };     /* 29 gain-fokozat */
  c.write(h, sizeof h);
}

/* Az anti-image mozgoatlag allapota. A hossza = rtl_up, tehat a nullai
 * pontosan a kepekre esnek. */
static int32_t  ma_hist_i[RTL_MAX_UP], ma_hist_q[RTL_MAX_UP];
static int32_t  ma_sum_i = 0, ma_sum_q = 0;
static uint32_t ma_idx = 0;

static void rtl_reset_filter(void)
{
  memset(ma_hist_i, 0, sizeof ma_hist_i);
  memset(ma_hist_q, 0, sizeof ma_hist_q);
  ma_sum_i = ma_sum_q = 0;
  ma_idx = 0;
  rtl_prev_i = rtl_prev_q = 0;
}

static void rtl_recalc_up(void)
{
#if (RTL_TARGET_SPS > 0)
  if (cur_sps == 0) { rtl_up = 1; rtl_reset_filter(); return; }
  uint32_t u = (RTL_TARGET_SPS + cur_sps / 2) / cur_sps;
  if (u < 1) u = 1;
  if (u > RTL_MAX_UP) u = RTL_MAX_UP;
  rtl_up = u;
#else
  rtl_up = 1;
#endif
  /* A szuro hossza megvaltozott — a regi tortenet ervenytelen. */
  rtl_reset_filter();
}

/* A kliens parancsai: 5 bajt, 1 bajt kod + 4 bajt BIG-ENDIAN parameter. */
static void rtl_poll_commands(void)
{
  while (cli_rtl && cli_rtl.available() >= 5) {
    uint8_t b[5];
    cli_rtl.read(b, 5);
    uint32_t p = ((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16)
               | ((uint32_t)b[3] << 8)  |  (uint32_t)b[4];

    switch (b[0]) {
      case 0x01: {                       /* SET_FREQUENCY (Hz) */
        /* A csuszkat huzva a kliens tizesevel kuldi ezt. Minden egyes
         * parancs a FG23-on stream-leallitast, hangolast es ujrainditast
         * jelent (par tized masodperc), tehat NEM szabad mindet
         * tovabbadni: csak a legutolsot, es azt is legfeljebb par
         * tizedmasodpercenkent. */
        rtl_pending_khz = (p + 500u) / 1000u;
        rtl_pending = true;
        break;
      }
      case 0x02:                         /* SET_SAMPLE_RATE */
        DIAG.printf("\nrtl_tcp: a kliens %lu sps-t ker; mi %lu sps-t adunk "
                    "%lux felmintavetelezve = %lu sps\n",
                    (unsigned long)p, (unsigned long)cur_sps,
                    (unsigned long)rtl_up,
                    (unsigned long)(cur_sps * rtl_up));
        break;
      case 0x03: case 0x08:              /* gain mode / AGC mode */
        DIAG.printf("\nrtl_tcp: gain/AGC mod %lu (nalunk szoftveres AGC "
                    "megy)\n", (unsigned long)p);
        break;
      case 0x04: case 0x0d:              /* SET_GAIN / gain by index */
        DIAG.printf("\nrtl_tcp: gain %lu (figyelmen kivul)\n",
                    (unsigned long)p);
        break;
      case 0x05:                         /* SET_FREQ_CORRECTION (ppm) */
        DIAG.printf("\nrtl_tcp: ppm %ld (a FG23-nak sajat kalibracioja "
                    "van)\n", (long)(int32_t)p);
        break;
      default:
        DIAG.printf("\nrtl_tcp: ismeretlen parancs 0x%02X = %lu\n",
                    b[0], (unsigned long)p);
        break;
    }
  }
}

/* int16 I/Q -> 8 bites offset binary, kozben egesz szorzoval
 * felmintavetelezve (linearis interpolacio). Visszaad: hany bajt lett. */
static size_t rtl_convert(const int16_t *iq, int nsamp, uint8_t *out)
{
  size_t o = 0;
  const int sh = rtl_shift;

  for (int k = 0; k < nsamp; k++) {
    int16_t ci = iq[2 * k], cq = iq[2 * k + 1];

    /* A csucsot MINDKET agon nezzuk. Csak I-t figyelni azert rossz, mert
     * a DC-blokk utan egy sav-szeli hang gyakorlatilag tisztan Q-ban is
     * ulhet — olyankor az AGC fejteret jelentene, mikozben levag. */
    int32_t a = ci; if (a < 0) a = -a;
    if (a > rtl_peak_win) rtl_peak_win = a;
    int32_t aq = cq; if (aq < 0) aq = -aq;
    if (aq > rtl_peak_win) rtl_peak_win = aq;

    for (uint32_t u = 0; u < rtl_up; u++) {
      /* linearis atmenet az elozo es a mostani minta kozott */
      int32_t wi = ((int32_t)rtl_prev_i * (int32_t)(rtl_up - u)
                  + (int32_t)ci * (int32_t)u) / (int32_t)rtl_up;
      int32_t wq = ((int32_t)rtl_prev_q * (int32_t)(rtl_up - u)
                  + (int32_t)cq * (int32_t)u) / (int32_t)rtl_up;

#if RTL_ANTI_IMAGE
      if (rtl_up > 1) {
        /* Futo atlag rtl_up hosszan: meg egy nullat tesz minden kep
         * kozepere. Ket osszeadas es egy osztas mintankent. */
        ma_sum_i += wi - ma_hist_i[ma_idx];  ma_hist_i[ma_idx] = wi;
        ma_sum_q += wq - ma_hist_q[ma_idx];  ma_hist_q[ma_idx] = wq;
        if (++ma_idx >= rtl_up) ma_idx = 0;
        wi = ma_sum_i / (int32_t)rtl_up;
        wq = ma_sum_q / (int32_t)rtl_up;
      }
#endif

      /* KEREKITES, nem csonkolas! Az aritmetikai jobbratolas a negativ
       * szamokat lefele csonkolja (-1 >> 5 = -1, nem 0), atlagosan
       * -0,5 LSB szisztematikus eltolast okozva MINDEN mintan — ez egy
       * allando, hatalmas DC-vonal a 8 bites folyamban ("atomvillanas
       * a kozepén", 2026-08-03). A fel-LSB hozzaadasa a tolas elott
       * szimmetrikus, nulla-atlagu kerekitest ad. */
      const int32_t rnd = (sh > 0) ? (1 << (sh - 1)) : 0;
      int32_t vi = ((wi + rnd) >> sh) + 128;
      int32_t vq = ((wq + rnd) >> sh) + 128;
      if (vi < 0) vi = 0; else if (vi > 255) vi = 255;
      if (vq < 0) vq = 0; else if (vq > 255) vq = 255;
      out[o++] = (uint8_t)vi;
      out[o++] = (uint8_t)vq;
    }
    rtl_prev_i = ci;
    rtl_prev_q = cq;
  }
  return o;
}

static void rtl_agc_tick(uint32_t now)
{
#if (RTL_FIXED_SHIFT >= 0)
  rtl_shift = RTL_FIXED_SHIFT;
  (void)now;
#else
  if (now - rtl_agc_ms < RTL_AGC_MS) return;
  rtl_agc_ms = now;

  /* SZELES HOLTSAV. Csak akkor lepunk, ha a csucs 8 bitre vetitve tenyleg
   * kilog a [LO..HI] sávbol — kozotte nem nyulunk hozza. Minden lepes 6 dB-t
   * mozdit az EGESZ kepen, ami a vizesesen vizszintes savkent latszik; az
   * elozo, szuk celertekre szabalyozo valtozat ezert lepkedett folyamatosan
   * es csikozta a vizesest. */
  if (rtl_peak_win > 0) {
    int32_t peak8 = rtl_peak_win >> rtl_shift;
    if (peak8 > RTL_AGC_HI && rtl_shift < 15) {
      rtl_shift++;
      DIAG.printf("\nrtl_tcp AGC: halkitas, shift=%d (csucs volt %ld)\n",
                  rtl_shift, (long)peak8);
    } else if (peak8 < RTL_AGC_LO && rtl_shift > 0) {
      rtl_shift--;
      DIAG.printf("\nrtl_tcp AGC: erosites, shift=%d (csucs volt %ld)\n",
                  rtl_shift, (long)peak8);
    }
  }
  rtl_peak_win = 0;
#endif
}

/* ===================================================================
 *                        KIMENETEK URITESE
 * =================================================================== */

/* MENNYIT VARHAT EGY BLOKK KIIRASA [ms]. 0 = egyaltalan nem var.
 *
 * MIERT NEM 0 (2026-08-06): a regi kod csak akkor irt, ha a CDC TX pufferben
 * volt hely egy EGESZ blokknak. HWCDC-vel ez jo (8 kB puffer), de a TinyUSB-s
 * USBCDC-nel a FIFO merete a CONFIG_TINYUSB_CDC_TX_BUFSIZE — ami jellemzoen
 * KISEBB, mint 1040 bajt. Akkor a feltetel SOHA nem teljesul, tehat MINDEN
 * blokk eldobodik: pontosan ez volt a "USB 0.0 kB/s(-248)" tunet, ahol a PC
 * 60 masodperc alatt NULLA bajtot kapott.
 *
 * A write() maga adagol: annyit tesz a FIFO-ba, amennyi befer, majd megvarja,
 * amig a host elviszi, es folytatja — a megadott ideig. Igy a blokkmeret es a
 * FIFO merete fuggetlenne valik egymastol.
 *
 * MIERT SZABAD VARNI (a "kobe vesett" szabaly megsertese nelkul): az
 * SPI-vetel MAR NEM a fo ciklusban van, hanem dedikalt taskban (core 1,
 * prio 22), mogotte 24 blokkos gyuruvel = 122 ms athidalas. A loop() prio 1 —
 * a task barmikor preemptalja. Nehany ms varakozas a loop()-ban tehat NEM tud
 * SPI-blokkot veszteni; a gyuru bohven elnyeli. (Ha megsem: a RXRING es az
 * ovf szamlalo AZONNAL megmutatja a statuszsorban.)
 *
 * Ha nincs csatlakozott host, NEM varunk egyaltalan — lasd a (bool)RAW agat. */
#define USB_TX_TIMEOUT_MS   3u

static void flush_usb(void)
{
  if (!agg_usb_n) return;

  /* NINCS HOST -> azonnali eldobas, varakozas nelkul. Enelkul minden
   * uritesnel elpazarolnank a teljes timeoutot olyankor is, amikor senki nem
   * olvassa az USB-t (ez az uzemszeru allapot: a mero csak alkalmankent fut). */
  if (!(bool)RAW) {
    st_usbdrop++;
    agg_usb_n = 0;
    return;
  }

  /* BLOKKONKENT irunk, nem az egesz gyujtot egyben (2026-08-06). Ket okbol:
   *
   *   1) A regi valtozat a TELJES agg_usb_n-t (4*1040 = 4160 bajt) kerte
   *      egyszerre. Ez HWCDC-vel mukodik (ott 8 kB a TX puffer), de a
   *      TinyUSB-s USBCDC-nel (ARDUINO_USB_MODE=0) a CDC TX FIFO merete a
   *      CONFIG_TINYUSB_CDC_TX_BUFSIZE — ami jellemzoen jóval kisebb. Ott a
   *      "van-e 4160 bajt hely?" SOHA nem teljesulne, tehat MINDENT eldobna:
   *      pontosan ugyanaz a tunet (USB 0.0 kB/s, csupa eldobas), amit ki
   *      akarunk kuszobolni. Blokkonkent mar csak 1040 bajt kell.
   *
   *   2) Az EGESZ blokk atomi kiirasa viszont KOTELEZO: a PC a "IQB2"
   *      magicre szinkronizal, egy fel blokk framing-hibat okozna, es a
   *      mero azt HAMIS SPI-LYUKNAK latna. Ezert blokkhataron vagunk.
   *
   * Ha itt eldobas tortenik, azt a statuszsor (-N) szamlaloja mutatja — a
   * PC-oldali "SPI-LYUK" csak akkor tiszta meres, ha ez a szam 0. */
  size_t off = 0;
  while (off < agg_usb_n) {
    /* A write() a FIFO meretetol fuggetlenul kiadja az EGESZ blokkot, ha kell,
     * tobb reszletben, a USB_TX_TIMEOUT_MS-ig varva. Ezert NEM kerdezzuk elore
     * az availableForWrite()-ot: pont az a lekerdezes akasztotta meg a
     * TinyUSB-s uzemet, ahol a FIFO kisebb egy blokknal. */
    /* ===== 2026-08-09: MEGIS KERDEZZUK, DE MASERT =====
     *
     * A fenti indoklas TinyUSB-re (ARDUINO_USB_MODE=0) igaz volt, ahol a
     * CDC FIFO kisebb egy blokknal, tehat a kerdes mindig nemet mondott
     * volna. HWCDC-n (a mostani build) viszont a TX puffer 8256 bajt —
     * amig senki nem figyel. Amint a soros monitor RACSATLAKOZIK es lassan
     * urit, az afw beragad 976-ra, ami KEVESEBB egy blokknal (1040), es
     * akkor a RAW.write() MINDEN blokknal kivarja a teljes timeoutot.
     *
     * MERT (a naplobol): flash utan, monitorral rajta:
     *     USB: conn=1 afw=976   usb=60245 us   proc=122814 us
     *     ciklus 1/s   dt_max=2169172 us   ovf 380 -> 5576
     * ugyanaz a firmware bekapcsolas utan, monitor NELKUL:
     *     USB: conn=0 afw=8256  ciklus 1180/s  dt_max=21 ms  ovf=0
     *
     * Ezert lattuk "minden flashelés utan" — flash utan a monitor mindig
     * fent van. A gomb sem valaszolt, mert a ui_tick() ebben a ciklusban fut.
     *
     * A nyers I/Q USB-ut NEM kritikus: ha nincs hely, dobjuk el a blokkot
     * es menjunk tovabb. A (-N) szamlalo ugyis mutatja. A vetel es a demod
     * viszont SOHA nem allhat meg egy terminal miatt. */
    if (RAW.availableForWrite() < (int)BLK_BYTES) { st_usbdrop++; break; }
    size_t w = RAW.write(agg_usb + off, BLK_BYTES);
    st_usbbytes += w;
    if (w != BLK_BYTES) {
      /* CSONKA iras: a host lefagyott vagy lassu. A maradekot eldobjuk. A PC a
       * kovetkezo magicre ujraszinkronizal, es a hianyzo sorszamot 1 lyuknak
       * konyveli — ezert csak akkor tiszta a PC-oldali SPI-LYUK meres, ha ez a
       * szamlalo (-0). */
      st_usbdrop++;
      break;
    }
    off += BLK_BYTES;
  }
  agg_usb_n = 0;
}

static void flush_tcp(void)
{
  if (have_raw && cli_raw.connected()) st_rawbytes += ring_flush(&ring_raw,
                                                                 cli_raw);
  if (have_rtl && cli_rtl.connected()) st_rtlbytes += ring_flush(&ring_rtl,
                                                                 cli_rtl);
}

/* ===================================================================
 *                          BLOKK-FELDOLGOZAS
 * =================================================================== */

/* A TENYLEGES feldolgozas. Ide mar IDOBEN HELYES sorrendben erkeznek a
 * blokkok — az ujrarendezot lasd a process()-ben. */
static void deliver(const uint8_t *b)
{
  const blk_hdr_t *h = (const blk_hdr_t *)b;
  st_blocks++;
  st_samples += h->nsamp;

  /* Keskenysavu waterfall a panelra: minden N. blokkbol FFT. A modul maga
   * ritkit (alapbol 12 -> ~16 sor/s), es ha fut a scan, azt hagyjuk nyerni,
   * mert az a szelesebb kep. */
  if (!st_specline || (millis() - st_specline_ms) > 3000u) {
    iq_fft_push_block(b + HDR_BYTES);
    /* FIGYELEM: a h->fs_in_hz a FG23 BEMENETI rataja (400 ksps), NEM a
     * decimalas utani. A skalahoz a TENYLEGES kimeneti rata kell, kulonben
     * a panel +-200 kHz-et irna ki 50 ksps helyett. */
    iq_fft_set_tuning(oled_khz * 1000u, st_sps_out);
  }

  /* A FG23 a fs_in_hz FELSO 12 bitjebe csomagolja a RAIL-RSSI-t (dBm) — a
   * fejlecben nincs kulon mezo, es a blokkmeretet nem bantjuk. Also 20 bit
   * = valodi rata, felso 12 bit = elojeles dBm. MASZKOLNI KELL, kulonben az
   * ingadozo RSSI folyton "rata-valtozast" jelezne es ujrakonfiguralna. */
  uint32_t fs_in = h->fs_in_hz & 0x000FFFFFu;
  {
    int rf = (int)((h->fs_in_hz >> 20) & 0xFFFu);
    if (rf & 0x800) rf -= 0x1000;          /* 12-bit elojel-kiterjesztes */
    st_rf_rssi = rf;                        /* dBm, a FG23 RAIL-jebol */
  }

  /* HOLTSAV: csak ERDEMI valtozasra konfiguralunk ujra. A mert rata
   * termeszetesen ingadozik par tized szazalekot; ha minden apro
   * elteresre ujraszamolnank a szuroket es kiirnank ket naplosort, a fo
   * ciklus megallna es blokkokat vesztenenk. Pontosan ez tortent. */
  uint32_t d_fs = (fs_in > cur_fs_in) ? (fs_in - cur_fs_in)
                                      : (cur_fs_in - fs_in);
  bool fs_changed = (cur_fs_in == 0) || (d_fs * 100u > cur_fs_in);  /* >1% */
  if (fs_changed || h->decim != cur_decim) {
    cur_fs_in = fs_in;
    cur_decim = h->decim;
    cur_sps   = cur_decim ? (cur_fs_in / cur_decim) : 0;
    rtl_recalc_up();
#if SPY_ENABLE
    spy_set_rate(cur_sps);
#endif
    DIAG.printf("\nstream: fs_in=%lu decim=%lu -> %lu sps   "
                "(rtl_tcp: %lux -> %lu sps)\n",
                (unsigned long)cur_fs_in, (unsigned long)cur_decim,
                (unsigned long)cur_sps, (unsigned long)rtl_up,
                (unsigned long)(cur_sps * rtl_up));
  }

  /* A fejlecbol vesszuk a mintaszamot, nem a define-bol: egy rovid blokk
   * kulonben 1024 bajtnyi regi puffertartalmat olvasna. */
  int nsamp = (int)h->nsamp;
  if (nsamp <= 0 || nsamp > BLK_SAMPLES) nsamp = BLK_SAMPLES;

  const int16_t *iq = (const int16_t *)(b + HDR_BYTES);
  for (int k = 0; k < nsamp * 2; k++) {
    int32_t a = abs((int32_t)iq[k]);
    if (a > st_peak) st_peak = a;
    st_abssum += (uint32_t)a;
    st_counted++;
  }

  /* --- APRS demodulator. A NYERS int16 mintakat kapja, minden tovabbi
   * feldolgozas elott: se AGC, se 8 bitre vagas, se interpolacio nem
   * rontja el a demodulaciot. --- */
#if APRS_RX_ENABLE
  /* CSAK akkor demodulalunk, ha az APRS uzemmod aktiv. Igy a gombbal
   * tenyleg fel lehet szabaditani a CPU-t egy masik meresre. */
  if (ui_mode() == UI_PAGE_APRS) aprs_rx_feed(iq, nsamp, cur_sps);
#endif
  /* CW: ugyanaz a nyers int16 I/Q, csak envelope kell neki */
  if (ui_mode() == UI_PAGE_CW) cw_rx_feed(iq, nsamp, cur_sps);

  /* --- (a) USB: a TELJES blokk megy, fejlecestol. A fejlecet a PC oldal
   * vagja le; cserebe barmikor ujra lehet szinkronizalni a magicre, es a
   * sorszambol o is latja a vesztest. --- */
  if (agg_usb_n + BLK_BYTES <= sizeof agg_usb) {
    memcpy(agg_usb + agg_usb_n, b, BLK_BYTES);
    agg_usb_n += BLK_BYTES;
  }
  if (agg_usb_n >= sizeof agg_usb) flush_usb();

  /* --- (b) TCP 8888: CSAK a payload, hogy az SDR++ Network source
   * kozvetlenul ehesse. Fejlec nelkul nincs mit levagni rajta. --- */
  if (have_raw && cli_raw.connected()) {
    ring_put(&ring_raw, b + HDR_BYTES, (uint32_t)nsamp * 4u);
  }

  /* --- (d) TCP 5555: SpyServer, NYERS 16 bit. Ugyanaz a blokk, semmilyen
   * atalakitas nelkul — ez a legjobb minosegu ut. --- */
#if SPY_ENABLE
  spy_feed(iq, nsamp);
#endif

  /* --- (c) TCP 1234: rtl_tcp, 8 bites, felmintavetelezve. --- */
  if (have_rtl && cli_rtl.connected()) {
    size_t n = rtl_convert(iq, nsamp, agg_rtl);
    ring_put(&ring_rtl, agg_rtl, (uint32_t)n);
  }

  /* A kuldest nem itt vegezzuk: a gyurut a fo ciklus uriti, annyival,
   * amennyi epp befer. Igy egy lassu halozat nem allitja meg a
   * blokk-feldolgozast, es nem fogynak el az SPI tranzakciok. */
}


/* ===================================================================
 *              SORSZAM-ELLENORZES ES UJRARENDEZES
 * ===================================================================
 *
 * Ide erkeznek a blokkok UGY, AHOGY az SPI adta oket — barmilyen
 * sorrendben. Innen mennek tovabb a deliver()-be, IDOREND SZERINT.
 * A reszletes indoklas a REORDER_BLOCKS-nal.
 */

#if REORDER_BLOCKS > 0
static uint8_t  ro_buf[REORDER_BLOCKS][BLK_BYTES];
static uint32_t ro_seq[REORDER_BLOCKS];
static bool     ro_full[REORDER_BLOCKS];
static uint32_t ro_next = 0;          /* a kovetkezo kiadando sorszam */
static uint32_t ro_high = 0;          /* a legmagasabb latott sorszam */
static bool     ro_armed = false;

/* Ennyi blokkot tartunk vissza. A kiadas UTEME igy a beerkezes uteme lesz:
 * minden uj blokkra pontosan egyet adunk ki. A regi valtozat "ami kesz, azt
 * azonnal" logikaja egy lyuk utan 20-30 blokkot lokott ki egyszerre — a
 * proc szakasz 1.2 ms-rol 8.5 ms-ra ugrott, es EZ volt a hallhato akadozas. */
#define RO_HOLD  (REORDER_BLOCKS - 4)

/* FORDITASI IDEJU OR: eddig kommentben allt a feltetel, es 2026-08-07-en
 * majdnem el is felejtettuk, amikor az NQUEUE 16-rol 24-re ment. Ha valaki
 * megint hozzanyul az egyikhez, itt all meg a build, nem a meropadon.
 *   - RO_HOLD >= NQUEUE+1 : az ablaknak fednie kell a legnagyobb lehetseges
 *     elkeses-tavolsagot, ami a keszlethez kotott (k*NQUEUE +- 1).
 *   - kettohatvany : a ro_pump() 'ro_next % REORDER_BLOCKS'-szal indexel,
 *     es a ro_next szabadon futo 32 bites szamlalo. Ha REORDER_BLOCKS nem
 *     osztoja 2^32-nek, az atfordulaskor (~255 nap) az index ugrik. */
static_assert(RO_HOLD >= NQUEUE + 1,
              "REORDER_BLOCKS tul kicsi az NQUEUE-hoz: kell RO_HOLD >= NQUEUE+1");
static_assert((REORDER_BLOCKS & (REORDER_BLOCKS - 1)) == 0,
              "REORDER_BLOCKS legyen kettohatvany (ro_next % REORDER_BLOCKS)");

static uint32_t st_ro_dup = 0, st_ro_hole = 0, st_ro_late = 0;
static uint32_t ro_late_run = 0;   /* egymas utani "keso" eldobasok */

/* ============ DUPLIKATUM-HITELESITES (2026-08-07 este) ============
 *
 * MIERT KELL. A fenti magyarazat (1958. sor korul) szerint a k*NQUEUE+-1
 * jelenseg oka az, hogy a hardver egy MAR LEZART, de meg KI NEM OLVASOTT
 * leiroba ir bele — vagyis a puffer tartalma felulirodik, mielott
 * elolvasnank. Ha ez igy van, akkor amikor ugyanaz a sorszam ketszer jon
 * meg, a ket peldany TARTALMA KULONBOZIK: az egyik a valodi blokk, a
 * masik egy felulirt puffer, aminek csak a fejlece stimmel.
 *
 * Eddig a `st_ro_dup` a masodik peldanyt NEMA eldobta, tehat sosem
 * derult ki, melyiket tartottuk meg. Ha a rossz peldanyt tartjuk meg, a
 * REND "javitasa" nem javitas, hanem ELREJTETT ADATROMLAS: a demod
 * hibatlannak latszo, de rossz mintakat kap.
 *
 * A KISERLET. Duplikatumkor osszehasonlitjuk a HASZNOS RESZT (a fejlec
 * nelkul, mert az per definicio egyezik). Ket kimenetel:
 *
 *   dupE (egyezik)  — a blokk tenyleg ketszer erkezett meg, valtozatlanul.
 *                     Az eldobas helyes, a lanc tiszta.
 *   dupK (kulonbozik) — ket KULONBOZO tartalom ugyanazzal a sorszammal.
 *                     Ekkor legalabb az egyik hamis, es a puffer-felulirasi
 *                     magyarazat all. Ez a szam legyen 0; ha nem az, a
 *                     REND-et nem szabad "megoldottnak" tekinteni.
 *
 * KOLTSEG: a memcmp CSAK duplikatumkor fut (uzemszeruen 0..1 / 2 s), es
 * 1024 bajt osszehasonlitasa ~2 us. A fo ciklusra merhetetlen. */
static uint32_t st_ro_dup_same = 0;   /* dupE — azonos tartalom */
static uint32_t st_ro_dup_diff = 0;   /* dupK — ELTERO tartalom (0 kell legyen) */
static uint8_t  st_dup_shown   = 0;   /* futamonkent legfeljebb 3 reszletes sor */

/* Egy elveszett blokk helyere CSEND kerul, hogy az idovonal ne csusszon.
 * Ez a lenyeg: egy ismert 5 ms-os lyuk sokkal kisebb baj, mint egy
 * 123 ms-os idougras — az AFSK a lyukon at ujraszinkronizal. */
static void ro_emit_hole(void)
{
  static uint8_t hole[BLK_BYTES];
  blk_hdr_t *h = (blk_hdr_t *)hole;
  h->magic = IQ_BLK_MAGIC;
  h->seq   = ro_next;
  h->nsamp = BLK_SAMPLES;
  h->decim = (uint16_t)(cur_decim ? cur_decim : 8);
  h->fs_in_hz = cur_fs_in ? cur_fs_in : 400000u;
  memset(hole + HDR_BYTES, 0, PAYLOAD_BYTES);
  st_ro_hole++;
  deliver(hole);
}

/* ALLANDO KESLELTETESU kiadas. Amig a legfrissebb sorszam es a kovetkezo
 * kiadando kozott legalabb RO_HOLD blokk van, adunk ki egyet — vagy a
 * puffereltet, vagy egy lyukat. Igy soha nincs torlodas es nincs burst. */
static void ro_pump(void)
{
  int guard = 6;                      /* egy hivasban legfeljebb ennyi */
  while (guard-- > 0 &&
         (int32_t)(ro_high - ro_next) >= (int32_t)RO_HOLD) {
    uint32_t i = ro_next % REORDER_BLOCKS;
    if (ro_full[i] && ro_seq[i] == ro_next) {
      ro_full[i] = false;
      deliver(ro_buf[i]);
    } else {
      ro_emit_hole();
    }
    ro_next++;
  }
}
#endif

static void process(const uint8_t *b, size_t nbytes)
{
  if (nbytes != BLK_BYTES) { st_badlen++; return; }
  const blk_hdr_t *h = (const blk_hdr_t *)b;

  /* SPECLINE (scan-sor) — ugyanakkora blokk, mas magic. Nem I/Q: nem megy
   * a demodra, a nyers TCP-re, sem az rtl_tcp-re; a SpyServer FFT-folyamara
   * es a kijelzore igen. Sorszamot nem konyvelunk ra (a scan alatt nincs
   * I/Q-folyam, a seq-diagnosztika ertelmetlen lenne). */
  if (h->magic == SPECLINE_MAGIC) {
    const specline_blk_t *L = (const specline_blk_t *)b;
    if (L->hdr.nbin >= 1 && L->hdr.nbin <= SPECLINE_MAX_BINS) {
      st_specline++;
      st_specline_ms = millis();
      st_specline_cf_hz   = L->hdr.f_center_hz;
      st_specline_span_hz = L->span_hz;
#if SPY_ENABLE
      spy_send_specline(L);
#endif
#if OLED_ENABLE
      oled_spec.push(L->bins, L->hdr.nbin);
#endif
      /* Ugyanaz a sor a 240x320-as panel waterfalljaba. Nem blokkol:
       * atmasol es ertesiti a kijelzo-taskot; ha az meg dolgozik, a sor
       * eldobodik. A kijelzo SOSEM lassithatja a radiot. */
      tft_set_autoscale(true);   /* a scan abszolut dBm-skalat kuld */
      tft_push_specline(L->bins, L->hdr.nbin);
      tft_set_span(L->hdr.f_center_hz, L->span_hz);
    }
    return;
  }

  if (h->magic != IQ_BLK_MAGIC) { st_badmagic++; return; }

  /* --- diagnosztika: mi tortent a sorrenddel (valtozatlan merőszamok) --- */
  if (have_seq) {
    uint32_t d = h->seq - last_seq;
    if (d >= 0x80000000u) {
      st_back++;
      seq_classify(last_seq - h->seq);
      if (st_gap_shown < 3) {
        st_gap_shown++;
        DIAG.printf("\nSEQ-HATRA: %lu -> %lu\n",
                    (unsigned long)last_seq, (unsigned long)h->seq);
      }
#if SEQ_DROP_STALE
      return;
#endif
    }
    if (d > 1u && d < 100000u) {
      st_lost += (d - 1u);
      seq_classify(d);
      if (st_gap_shown < 3) {
        st_gap_shown++;
        DIAG.printf("\nSEQ-LYUK: %lu -> %lu (d=%lu)\n",
                    (unsigned long)last_seq, (unsigned long)h->seq,
                    (unsigned long)d);
      }
    }
  }
  last_seq = h->seq;
  have_seq = true;

#if REORDER_BLOCKS <= 0
  deliver(b);
#else
  if (!ro_armed) { ro_armed = true; ro_next = ro_high = h->seq; }

  /* ===== UJRAINDULAS-FELISMERES =====
   *
   * EZ HIANYZOTT, es ez olte meg a rendszert 2026-08-03-an. A FG23 a
   * stream ujrainditasakor NULLAZZA a sorszamot. A regi kod ilyenkor
   * minden blokkot "keson jott"-nek latott (a kivonas elojel nelkul
   * korbefordul), es VEGLEG eldobta mindet: keso=390 per 2 masodperc,
   * ami pontosan az OSSZES blokk. A stream soha nem indult ujra.
   *
   * A kesest a nagysagrend valasztja el az ujrainditastol: a mert
   * eltolas legfeljebb +-25 blokk, tehat barmi, ami 1000-nel messzebb
   * van barmelyik iranyban, nem keses, hanem uj kezdet. */
  uint32_t rel  = h->seq - ro_next;         /* elojel nelkul: a mult nagy */
  uint32_t back = ro_next - h->seq;
  bool restart = false;

  if (rel >= 0x80000000u) {                 /* a multbol jott */
    /* seq==0 MINDIG ujrainditas — a FG23 csak akkor ad nullat. A 2026-08-03-i
     * eles futas fogta meg: 819 blokk utani ujrainditasnal a visszalepes
     * 1000 alatt maradt, a kod kesesnek nezte, es 2 masodpercig dobta a
     * friss blokkokat, mig a sorszam utol nem erte a regit. */
    if (back > 1000u || h->seq == 0u) {
      restart = true;
    } else {
      st_ro_late++;
      /* ONGYOGYITAS: ha sorozatban esnek ki "keso" blokkok, az nem keses,
       * hanem elcsuszott allapot — barmi is okozta, 50 blokk (~0.26 s)
       * utan ujraszinkronizalunk a beerkezo sorszamra. */
      if (++ro_late_run >= 50u) restart = true;
      else return;
    }
  } else if (rel > 1000u) {
    restart = true;                         /* tul messze elore */
  }
  ro_late_run = 0;

  if (restart) {
    for (int k = 0; k < REORDER_BLOCKS; k++) ro_full[k] = false;
    ro_next = ro_high = h->seq;
    DIAG.printf("\nREND: ujraindulas, sorszam innen: %lu\n",
                (unsigned long)h->seq);
  }

  uint32_t i = h->seq % REORDER_BLOCKS;
  if (ro_full[i] && ro_seq[i] == h->seq) {
    st_ro_dup++;
    /* Csak a hasznos reszt hasonlitjuk: a fejlec (magic/seq/nsamp/decim/
     * fs_in_hz) definicio szerint egyezik, a fs_in_hz felso 12 bitjeben
     * viszont az RSSI utazik, ami blokkonkent valtozhat — az nem romlas. */
    if (memcmp(ro_buf[i] + HDR_BYTES, b + HDR_BYTES, PAYLOAD_BYTES) == 0) {
      st_ro_dup_same++;
    } else {
      st_ro_dup_diff++;
      if (st_dup_shown < 3) {
        st_dup_shown++;
        /* Az ELSO elteres helye sokat mond: ha a 0. bajton kezdodik, a
         * ket puffer teljesen mas blokk; ha kesobb, akkor a felulriras a
         * blokk KOZEPEN kapta el — azaz futo DMA-ba olvastunk bele. */
        size_t off = 0;
        const uint8_t *p = ro_buf[i] + HDR_BYTES, *q = b + HDR_BYTES;
        while (off < PAYLOAD_BYTES && p[off] == q[off]) off++;
        DIAG.printf("\nREND-DUPK: seq=%lu elteres a %u. bajttol "
                    "(%02X!=%02X) — a puffer felulirodott\n",
                    (unsigned long)h->seq, (unsigned)off,
                    p[off], q[off]);
      }
    }
    return;
  }
  memcpy(ro_buf[i], b, BLK_BYTES);
  ro_seq[i]  = h->seq;
  ro_full[i] = true;
  if ((int32_t)(h->seq - ro_high) > 0) ro_high = h->seq;
  ro_pump();
#endif
}

/* ===================================================================
 *                    WIFI-PLAFON MERO (wifi_bench)
 * ===================================================================
 * A lanc sajat szamlaloinak kivezetese a bench fele. A lepcsonkenti
 * deltat a modul kepezi. FONTOS: a spi_lost-ba a st_ro_hole megy, NEM a
 * st_lost (VESZT) — utobbi a driver-glitch miatt ~16x tulszamol, es
 * ettol a plafon hamisan alacsonynak latszana.
 *
 * A konzol: a flashlog ki van kapcsolva (FLASHLOG_ENABLE 0), tehat a
 * Serial0-t rajtunk kivul SENKI nem olvassa — nincs utkozes. */
static void bench_ext_cb(wb_ext_t *o)
{
  /* A statusz-szamlalok 2 mp-enkent nullazodnak, ezert a KUMULATIV tukor
   * plusz az aktualis, meg le nem nullazott resz az igazi ertek. */
  o->spi_lost       = bench_tot_ro_hole + st_ro_hole;
  o->spi_dup        = bench_tot_ro_dup  + st_ro_dup;
  o->spi_ovf        = rx_ring_ovf;
  o->spi_blocks     = bench_tot_blocks  + st_blocks;
  o->spi_ring_max   = rx_ring_max;
  o->loop_dt_max_us = st_dtmax;
#if SPY_ENABLE
  o->spy_drop       = spy_dropped();
#else
  o->spy_drop       = 0;
#endif
}

static void bench_console_tick(void)
{
  static char cmd[80];
  static uint8_t n = 0;
  while (DIAG.available()) {
    char c = (char)DIAG.read();
    if (c == '\n' || c == '\r') {
      if (n) {
        cmd[n] = '\0';
        if (!wifi_bench_cmd(cmd) && !rgb_load_cmd(cmd)) iq_fft_cmd(cmd);
      }
      n = 0;
    } else if (n < sizeof cmd - 1) {
      cmd[n++] = c;
    } else {
      n = 0;                    /* tulcsordulas: eldobjuk a sort */
    }
  }
}

/* ===================================================================
 *                              SETUP
 * =================================================================== */

void setup()
{
  /* A HardwareSerial-nek alapbol NINCS TX gyurupuffere: a write() addig
   * blokkol, amig a bajtok be nem kerulnek a hardver FIFO-jaba. Egy 180
   * karakteres statisztika-sor 115200-on ~16 ms — annyi ido alatt i8-on
   * harom blokk erkezne, es ha nincs felhuzott SPI tranzakcio, azok
   * VEGLEG elvesznek. Puffer + gyorsabb baud, es a kiiras "ingyen" lesz. */
  DIAG.setTxBufferSize(2048);
  DIAG.begin(115200);

  /* A nativ USB CDC TX-puffere alapbol par szaz bajt, amibe egy 1040
   * bajtos blokk sem fer bele — ezen bukott meg eloszor az egesz lanc
   * (USB 0.0 kB/s, minden blokk eldobva). A begin() ELOTT kell allitani. */
#if !FG23_USB_TINYUSB
  /* Ez a metodus a HWCDC-e. A TinyUSB-s USBCDC-nek nincs setTxBufferSize()-a
   * (a CDC TX FIFO meretet a CONFIG_TINYUSB_CDC_TX_BUFSIZE adja) — ott a
   * forditas elszallna rajta, ezert van feltetel korulotte. Ha MODE=0-ban
   * eldobasokat latsz a statuszsorban, a pioarduino custom_sdkconfig-ba
   * tedd be:   CONFIG_TINYUSB_CDC_TX_BUFSIZE=4096
   * (ugyanoda, ahova a CONFIG_SPI_SLAVE_ISR_IN_IRAM=y kerul). */
  RAW.setTxBufferSize(AGG_BLK * BLK_BYTES + 4096);
#endif
  RAW.setTxTimeoutMs(USB_TX_TIMEOUT_MS);   /* lasd a flush_usb() feletti magyarazatot */
#if FG23_USB_TINYUSB
  /* A DTR/RTS-re torteno AUTOMATIKUS RESET KIKAPCSOLASA. Enelkul minden
   * PC-oldali portnyitas ujrainditja az ESP-t (rst:0x15), a USB ujra-enumeral,
   * es a host mar nyitott handle-je zombi lesz -> a meres nem kap adatot.
   * CSAK a TinyUSB-s USBCDC tudja; a HWCDC-nek (MODE=1) nincs ilyen metodusa,
   * ezert van forditasi feltetel korulotte. A begin() ELOTT kell hivni. */
  RAW.enableReboot(false);
#endif
  RAW.begin();
#if FG23_USB_TINYUSB
  /* TinyUSB-nel a stacket expliciten el kell inditani. Idempotens: ha a core
   * mar elinditotta (ARDUINO_USB_CDC_ON_BOOT=1), ez nem csinal semmit. */
  USB.begin();
#endif
  delay(400);

  oled_init();

  DIAG.println("\n=== FG23 -> ESP32-S3 -> USB + WiFi + rtl_tcp ===");
  /* Legyen LATHATO a naploban, melyik USB-uzemmod fut — a ketto egeszen
   * maskepp viselkedik a PC-oldali meronel. */
#if FG23_USB_TINYUSB
  DIAG.println("USB: TinyUSB CDC (MODE=0), DTR/RTS-reset KIKAPCSOLVA ✓ "
               "— meresre ez a jo");
#else
  DIAG.println("USB: HWCDC / USB-Serial-JTAG (MODE=1) — FIGYELEM: a DTR/RTS "
               "RESETEL (rst:0x15).");
  DIAG.println("     Ha a PC-oldali mero nem kap adatot, forditsd "
               "-D ARDUINO_USB_MODE=0 -val.");
#endif
  DIAG.printf("SPI: SCLK=GPIO%d  MOSI=GPIO%d  CS=GPIO%d\n",
              (int)PIN_SCLK, (int)PIN_MOSI, (int)PIN_CS);

#if CMDLINK_ENABLE
  Serial1.setTxBufferSize(256);
  Serial1.begin(CMDLINK_BAUD, SERIAL_8N1, -1, CMDLINK_TX_PIN);
  DIAG.printf("CMD:  GPIO%d -> FG23 PA06 / EXP 11 (%d baud) — hangolas "
              "a telefonrol\n", CMDLINK_TX_PIN, CMDLINK_BAUD);
#endif

  spi_bus_config_t bus;
  memset(&bus, 0, sizeof(bus));
  bus.mosi_io_num     = PIN_MOSI;
  bus.miso_io_num     = -1;
  bus.sclk_io_num     = PIN_SCLK;
  bus.quadwp_io_num   = -1;
  bus.quadhd_io_num   = -1;
  bus.max_transfer_sz = BLK_BYTES;
  /* ================= A MEGSZAKITAS PRIORITASA =================
   *
   * 2026-08-03: bizonyitott, hogy a sorszamugrasok merete PONTOSAN koveti
   * az NQUEUE-t (12 -> 13/25, 24 -> 25/49/-23, 17 -> 18/35/-16, POOL=100%).
   * A keszlet KOZBEN NEM urul ki (qmax tipikusan 1..5 a 17-bol), tehat nem
   * kapacitas-, hanem IDOZITESI hiba.
   *
   * A mechanizmus: a kovetkezo tranzakciot a driver MEGSZAKITASBAN fuzi
   * fel, miutan az elozo befejezodott. A FG23 vakon kuld, es ket blokk
   * kozott csak ~1.46 ms a szunet (a CS 3656 us-ig van lent az 5120 us-os
   * periodusbol). Ha a megszakitas ezen belul nem fut le, a hardver a
   * beerkezo blokkot meg a REGI leiroba irja — vagyis egy olyan pufferbe,
   * aminek a befejezese mar a visszaadasi sorban all, de mi meg nem
   * olvastuk ki. Amikor kiolvassuk, egy egesz pufferkorrel ujabb adatot
   * kapunk. Innen a k*NQUEUE +-1.
   *
   * Ezert erosodik a hiba a halozati forgalommal: a lwIP es a WiFi
   * ugyanazon a magon dolgozik, es kesletteti a megszakitast.
   *
   * A javitas: LEVEL3 prioritas (a WiFi tipikusan LEVEL1) es IRAM, hogy
   * egy flash-muvelet se blokkolhassa. Az IDF spi_slave ISR-je
   * IRAM-biztos (CONFIG_SPI_SLAVE_ISR_IN_IRAM alapbol be van kapcsolva).
   *
   * Ha ez sem eleg, a kovetkezo lepes a BLK_SAMPLES 256 -> 512 (fele annyi
   * tranzakcio, ketszer annyi ido a felfuzesre) — de azt a FG23-on IS at
   * kell allitani. Utana pedig a hardveres RDY-vonal.
   *
   * A KONKRET ZASZLOKAT NEM ITT ALLITJUK BE, hanem lejjebb, letraban
   * vegigprobalva — lasd az inditasnal. */

  spi_slave_interface_config_t slv;
  memset(&slv, 0, sizeof(slv));
  slv.mode         = 0;
  slv.spics_io_num = PIN_CS;
  slv.queue_size   = NQUEUE;
#if RDY_ENABLE
  /* A ket callback ISR-bol fut, ezert KOZVETLEN regiszteriras, semmi
   * fuggvenyhivas. A jelentesuk hardveri teny:
   *   post_setup = a leiro betoltve, a slave kepes fogadni  -> RDY fel
   *   post_trans = a tranzakcio lezarult                    -> RDY le
   * A ketto kozott a driver ISR-je fuzi fel a kovetkezot; amig az nem
   * tortent meg, a lab alacsony, es a FG23 var. */
  slv.post_setup_cb = spi_rdy_up;
  slv.post_trans_cb = spi_rdy_down;
#endif

#if RDY_ENABLE
  /* A lab mar a felfuzes ELOTT legyen kimenet es alacsony — a FG23 igy
   * mar boot kozben is helyes allapotot lat. */
  pinMode(RDY_PIN, OUTPUT);
  digitalWrite(RDY_PIN, LOW);
  DIAG.printf("RDY: GPIO%d -> FG23 PD2 / EXP 9 (post_setup/post_trans)\n",
              RDY_PIN);
#endif

  /* ===== MEGSZAKITAS-PRIORITAS: LETRA, NEM FELTETELEZES =====
   *
   * 2026-08-03: az ESP_INTR_FLAG_IRAM-tol a spi_slave_initialize HIBAVAL
   * tert vissza (az adott Arduino-buildben a CONFIG_SPI_SLAVE_ISR_IN_IRAM
   * nincs bekapcsolva), a hibat pedig nem neztuk meg — utana minden
   * get_trans_result "host not slave"-et kiabalt, es a stream el sem
   * indult. Tanulsag: platformfuggo kepesseget sose feltetelezz, probald
   * ki, es ird ki, mi jott ossze.
   *
   * A letra a legjobbtol a biztosig megy. Ami elsonek sikerul, az marad. */
  static const struct { int flags; const char *nev; } INTR_TRY[] = {
    { ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3, "IRAM + LEVEL3"   },
    { ESP_INTR_FLAG_LEVEL3,                      "LEVEL3"          },
    { ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2, "IRAM + LEVEL2"   },
    { ESP_INTR_FLAG_LEVEL2,                      "LEVEL2"          },
    { 0,                                         "alapertelmezett" },
  };
  esp_err_t e = ESP_FAIL;
  for (unsigned k = 0; k < sizeof INTR_TRY / sizeof INTR_TRY[0]; k++) {
    bus.intr_flags = INTR_TRY[k].flags;
    e = spi_slave_initialize(FG23_SPI_HOST, &bus, &slv, SPI_DMA_CH_AUTO);
    DIAG.printf("spi_slave_initialize (%-16s): %d%s\n",
                INTR_TRY[k].nev, (int)e,
                (e == ESP_OK) ? "   <-- EZ MEGY" : "");
    if (e == ESP_OK) break;
  }
  spi_ok = (e == ESP_OK);

  if (!spi_ok) {
    /* NE porgessunk tovabb: a get_trans_result masodpercenkent tobb ezer
     * hibauzenetet ontana a konzolra, es elfedne minden mast. */
    DIAG.println("!! az SPI slave NEM indult el — nem lesz stream.");
    DIAG.println("!! a tobbi (WiFi, kijelzo, terminal) tovabb megy.");
  } else {
    for (int i = 0; i < NQUEUE; i++) {
      memset(&trans[i], 0, sizeof(trans[i]));
      trans[i].length    = BLK_BYTES * 8;
      trans[i].rx_buffer = rxbuf[i];
      spi_slave_queue_trans(FG23_SPI_HOST, &trans[i], portMAX_DELAY);
    }

    /* A dedikalt veteli task — lasd a hosszu kommentet a definicional.
     * Ha ez nem indul el, NEM esunk vissza csendben a regi (fo ciklusos)
     * utra: azt a modellt szandekosan kivettuk, mert bizonyitottan
     * blokkvesztest okoz. Inkabb kiabaljunk. */
    BaseType_t tok = xTaskCreatePinnedToCore(
        spi_rx_task, "spi_rx", 4096, NULL,
        SPI_RX_TASK_PRIO, &rx_task_handle, SPI_RX_TASK_CORE);
    if (tok != pdPASS) {
      spi_ok = false;
      DIAG.println("!! spi_rx task NEM indult el — stream letiltva!");
    } else {
      DIAG.printf("SPI-vetel: dedikalt task, core %d, prio %d, gyuru %d blokk "
                  "(%d ms athidalas)\n",
                  SPI_RX_TASK_CORE, SPI_RX_TASK_PRIO, RXRING_BLOCKS,
                  RXRING_BLOCKS * 512 * 10 / 1000);
      /* A ket athidalas KULON dolgot ved (lasd RXRING_BLOCKS-nal), es
       * 2026-08-07 ota szandekosan egyforma. Kiirjuk, hogy a bootlogbol
       * visszakeresheto legyen, melyik binarist mertuk. */
      DIAG.printf("SPI-keszlet: %d tranzakcio (%d ms athidalas flash-stall "
                  "alatt), ujrarendezo ablak %d blokk\n",
                  NQUEUE, NQUEUE * 512 * 10 / 1000, REORDER_BLOCKS);
    }
  }

  wifi_start();

  DIAG.println("\n--- HOGYAN CSATLAKOZZ ---");
  /* FONTOS: a String-et VALTOZOBA kell tenni. A my_ip.toString().c_str()
   * egy ideiglenes objektumra mutat, ami a kifejezes vegen megszunik —
   * a printf mar szemetre mutatna. */
  String ips = wifi_up ? my_ip.toString() : String("<nincs wifi>");
  DIAG.println("SDR++ asztali, python NELKUL:");
  DIAG.printf("   Source=Network   TCP / Client / %s : %d / Int16\n",
              ips.c_str(), PORT_RAW);
  DIAG.println("SDR++ Android (EZ A JO UT, 16 bit, natív rata):");
  DIAG.printf("   Source=SpyServer  %s : %d\n", ips.c_str(), PORT_SPY);
#if RTL_ENABLE
  DIAG.println("SDR++ Android / SDR Touch (8 bit, tartalek ut):");
  DIAG.printf("   Source=RTL-TCP    %s : %d\n", ips.c_str(), PORT_RTLTCP);
#else
  DIAG.println("rtl_tcp (1234): KIKAPCSOLVA (RTL_ENABLE 0) — +75 kB RAM");
#endif
  ui_init();
#if SPY_ENABLE
  spy_init(PORT_SPY, spy_tune_cb, spy_scan_cb);
#endif
#if APRS_RX_ENABLE
  aprs_rx_init();
#endif
  cw_rx_init();

  DIAG.println("USB-n (PC-oldali dekoder + SPI-veszteseg meres):");
  DIAG.println("   python usb_aprs_rx.py --period 1.131 --seconds 90");
#if REORDER_BLOCKS > 0
  DIAG.printf("\nUjrarendezes: %d blokkos ablak sorszam szerint "
              "(%.0f ms kesleltetes)\n", REORDER_BLOCKS,
              REORDER_BLOCKS * 256.0 * 1000.0 / 50000.0);
#else
  DIAG.println("\nUjrarendezes: KI");
#endif
  DIAG.printf("RAM indulaskor: %lu kB szabad heap (legnagyobb blokk %lu kB)\n",
              (unsigned long)(esp_get_free_heap_size() / 1024u),
              (unsigned long)(heap_caps_get_largest_free_block(
                                  MALLOC_CAP_8BIT) / 1024u));
  DIAG.println("A FG23 autostarttal indul — nem kell 'i32'-t nyomni.");

#if FLASHLOG_ENABLE
  flashlog_init();
#endif

  /* WiFi-atviteli plafon mero. Sajat taskot es sajat portot (7777) nyit,
   * de CSAK a 'B...' parancsra — indulaskor semmit nem csinal. */
  wifi_bench_init();
  wifi_bench_set_ext(bench_ext_cb);
  DIAG.println("WiFi-bench: 'B' = statusz, 'Bs200,2000,200,20' = rampa, "
               "'B0' = stop  (nyelo: wifi_sink.py, port 7777)");

  /* WS2812 terhelesjelzo. A LED annal gyorsabban keveri a szint, minel
   * jobban meg van terhelve a CPU. Nemely klonon a 38-as lab, nem a 48. */
  rgb_load_init(RGB_LED_PIN);
  DIAG.println("RGB: 'L' = statusz, 'Ltest' = szinkor, 'Lset fenyero=20' stb.");

  /* ILI9341 allapotkijelzo. SPI2 (IO_MUX 9-14) — az FG23 slave a SPI3-on
   * van, tehat nincs busz-utkozes. TE nincs bekotve (a 47 az RGB-e). */
  tft_init();

  /* Azonnal ki az IP a kijelzore, ne kelljen 1,5 mp-et varni ra. */
  oled_tick(millis() + OLED_REFRESH_MS);
}

/* ===================================================================
 *                               LOOP
 * =================================================================== */

static void accept_clients(uint32_t now)
{
  if (!wifi_up) return;

  /* Torlesztve: az accept() lwIP-hivas, a fo ciklus viszont masodpercenkent
   * tobb ezerszer fut. Ket szerverre ennyiszer rakerdezni feleslegesen eszi
   * a CPU-t, es a lwIP zarat is fogja — abbol lesz a blokkvesztes. */
  static uint32_t last_accept = 0;
  if (now - last_accept < 250u) {
    /* a lecsatlakozast azert MINDEN korben nezzuk, az olcso */
    if (have_raw && !cli_raw.connected()) {
      DIAG.println("\n<<< nyers kliens lecsatlakozott");
      cli_raw.stop(); have_raw = false;
    }
    if (have_rtl && !cli_rtl.connected()) {
      DIAG.println("\n<<< rtl_tcp kliens lecsatlakozott");
      cli_rtl.stop(); have_rtl = false;
    }
    return;
  }
  last_accept = now;

  /* accept(), nem available(): az arduino-esp32 3.x-ben a NetworkServer
   * available()-je elavult alias, es figyelmeztetest ad. Ugyanazt csinalja.
   * Ha valamiert regebbi magon forditanad, ez az egy szo a visszaut. */
  WiFiClient c = srv_raw.accept();
  if (c) {
    if (have_raw) cli_raw.stop();
    cli_raw = c;
    cli_raw.setNoDelay(true);
    have_raw = true;
    ring_raw.head = ring_raw.tail = 0;
    /* Az availableForWrite-ot kiirjuk: ha 0-t ad, a core-od nem valositja
     * meg, es akkor a gyuru "vakon" ir. Egy szam, ami egy orat sporol,
     * amikor egyszer csend lesz a porton. */
    DIAG.printf("\n>>> NYERS kliens: %s   (%lu sps, Int16, "
                "availableForWrite=%d)\n",
                cli_raw.remoteIP().toString().c_str(),
                (unsigned long)cur_sps, (int)cli_raw.availableForWrite());
  }

#if RTL_ENABLE
  WiFiClient r = srv_rtl.accept();
  if (r) {
    if (have_rtl) cli_rtl.stop();
    cli_rtl = r;
    cli_rtl.setNoDelay(true);
    have_rtl = true;
    ring_rtl.head = ring_rtl.tail = 0;
    rtl_send_header(cli_rtl);
    rtl_prev_i = rtl_prev_q = 0;
    DIAG.printf("\n>>> rtl_tcp kliens: %s   (%lux felmintavetelezes -> "
                "%lu sps)\n",
                cli_rtl.remoteIP().toString().c_str(),
                (unsigned long)rtl_up, (unsigned long)(cur_sps * rtl_up));
  }
#endif

  /* A sajat flaget kell nezni, nem a "cli_raw"-ot: a WiFiClient bool
   * operatora connected()-et ad, tehat a "cli && !cli.connected()" mindig
   * hamis lenne, es sose latnank a lecsatlakozast. */
  if (have_raw && !cli_raw.connected()) {
    DIAG.println("\n<<< nyers kliens lecsatlakozott");
    cli_raw.stop();
    have_raw = false;
  }
  if (have_rtl && !cli_rtl.connected()) {
    DIAG.println("\n<<< rtl_tcp kliens lecsatlakozott");
    cli_rtl.stop();
    have_rtl = false;
  }
}

static uint32_t st_loops = 0;

void loop()
{
  st_loops++;

  /* WiFi-plafon mero: 'B...' parancsok a DIAG konzolrol. Nem blokkol,
   * es csak akkor csinal barmit, ha van beerkezett karakter. */
  bench_console_tick();

  /* --- ciklusidő-meres. A leghosszabb szunet ket kor kozott a legjobb
   * egyetlen szam arrol, meddig voltunk vakok. --- */
  {
    uint32_t nu = micros();
    if (st_prev_us) {
      uint32_t dt = nu - st_prev_us;
      if (dt > st_dtmax) st_dtmax = dt;
    }
    st_prev_us = nu;
  }

  /* ===============================================================
   *            SPI: TELJES LESZIVATTYUZAS, AZONNALI VISSZAFUZES
   * ===============================================================
   *
   * Ket valtozas a regihez kepest, es mindketto szamit:
   *
   * 1. NEM egy blokkot veszunk korönkent, hanem AMENNYI VAN. A regi kod
   *    egy torlodast csak korönkent egy blokkal tudott ledolgozni, tehat
   *    egyetlen 60 ms-os megallas utan meg tovabbi tized masodpercig
   *    hajszolta magat — es kozben ujra betelt a keszlet.
   *
   * 2. A puffert AZONNAL visszafuzzuk, MIELOTT feldolgoznank. Elotte a
   *    tranzakcio a teljes process() idejere (APRS-demod, csucskereses,
   *    harom gyuruba masolas, esetleg USB-uritese) ki volt veve a
   *    keszletbol. Egy 1040 bajtos memcpy par mikroszekundum — cserebe a
   *    hardvernek MINDIG van hova irnia.
   *
   * A masolatot azert kell, mert a feldolgozas mar nem hasznalhatja azt a
   * puffert, amit epp visszaadtunk a DMA-nak. */
  /* ===============================================================
   *   SPI: A VETEL MAR NEM ITT TORTENIK (spi_rx_task, sajat core-on).
   *   Itt csak FOGYASZTUNK a gyurubol — ha ez a ciklus megall 88 ms-ra
   *   (OLED, WiFi, akarmi), az KESLELTETES, nem VESZTESEG.
   * =============================================================== */
  /* A setup() alatt (WiFi-pasztazas ~10 s!) a task mar vesz, de meg senki
   * sem fogyaszt — a gyuru betelik es az ovf szamol. Az a veszteseg a
   * boot sajatja (regen is elveszett, csak a DMA-sorban), nem uzemi hiba.
   * Nullazzuk az elso korben, hogy az ovf tenyleg a FUTAS KOZBENI
   * vesztesegeket merje — annak kell orokre 0-nak lennie. */
  static bool ovf_boot_reset = false;
  if (!ovf_boot_reset) { ovf_boot_reset = true; rx_ring_ovf = 0; rx_ring_max = 0; }

  uint32_t drained = 0;
  for (;;) {
    uint32_t head = __atomic_load_n(&rx_head, __ATOMIC_ACQUIRE);
    if (rx_tail == head) break;
    SECT(S_PROC, process(rxring[rx_tail_idx], rxring_len[rx_tail_idx]));
    if (++rx_tail_idx == RXRING_BLOCKS) rx_tail_idx = 0;
    __atomic_store_n(&rx_tail, rx_tail + 1u, __ATOMIC_RELEASE);
    if (++drained >= DRAIN_MAX) break;
  }
  if (drained > st_qmax) st_qmax = drained;
  /* Utemezes: regen a get_trans_result 2 ms-os varakozasa adta. Ha a
   * gyuru ures (vagy nincs SPI), engedjuk el a magot egy tick-re —
   * kulonben a loop vakon porogne teljes gozzel. */
  if (drained == 0) delay(spi_ok ? 1 : 2);

  uint32_t now = millis();

  /* Idozitett urites: ha keves blokk jon (nagy R), ne alljon a gyujtoben
   * az adat masodpercekig. */
  /* A TCP gyuruket MINDEN korben uritjuk (nem csak idozitve): igy a
   * halozat annyit visz el, amennyit epp bir, es nem torlodik fel. */
  SECT(S_TCP, flush_tcp());

  if (now - last_flush_ms >= AGG_FLUSH_MS) {
    SECT(S_USB, flush_usb());
    last_flush_ms = now;
  }

  SECT(S_ACC, accept_clients(now));
#if SPY_ENABLE
  SECT(S_SPY, spy_tick(now));
#endif
  SECT(S_RTL, { rtl_poll_commands(); rtl_agc_tick(now); });
  SECT(S_WIFI, wifi_tick(now));
  SECT(S_UI, ui_tick(now));
#if APRS_RX_ENABLE
  /* A KAPCSOLAT-KEZELEST (async DNS, connect, drain, backoff) MINDIG
   * futtatjuk — fuggetlenul az uzemmodtol. Ha csak APRS-modban futna, egy
   * hosszu gombnyomas (demod ki) utan a socket nyitva maradna, a szerver
   * forgalma feltorlodna a fogadopufferben, es a kapcsolat megallna. A
   * DEMOD-ot (aprs_rx_feed) tovabbra is az uzemmod kapuzza (lasd fentebb). */
  SECT(S_APRS, aprs_rx_tick(now));
  /* Uj csomag -> ugorjon elore az APRS-oldal. A ui_flash magatol nem lep,
   * ha nem az APRS uzemmod fut. */
  {
    static uint32_t seen_ms = 0;
    uint32_t lms = aprs_rx_last_ms();
    if (lms && lms != seen_ms) { seen_ms = lms; ui_flash(UI_PAGE_APRS, OLED_APRS_HOLD_MS); }
  }
#endif
  /* CW tick + uj karakterkor villanas a CW oldalra */
  SECT(S_CW, cw_rx_tick(now));
  {
    static uint32_t seen_cw = 0;
    uint32_t lms = cw_rx_last_ms();
    if (lms && lms != seen_cw) {
      seen_cw = lms;
      ui_flash(UI_PAGE_CW, 8000);   /* 8 s-ig maradjon a CW oldal */
    }
  }
  /* Oldalvaltasnal AZONNAL rajzolunk, nem varunk a kovetkezo periodusra —
   * kulonben a gombnyomas utan masfel masodpercig nem tortenik semmi, es
   * azt hinned, nem mukodik. */
  if (ui_dirty()) { ui_clear_dirty(); oled_force(); }
  SECT(S_OLED, oled_tick(now));

#if FLASHLOG_ENABLE
  /* FLUSH-ABLAK (2026-08-07). A flashbe iras kikapcsolja a flash-cache-t,
   * ami MINDKET magon megallitja a flashben futo kodot — a dedikalt
   * SPI-veteli taskot is. Ilyenkor mar csak a felfuzott DMA-tranzakciok
   * fogynak, NQUEUE=24-gyel 123 ms-ig.
   *
   * A 'drained == 0' azt jelenti, hogy ebben a korben a vetel-gyuru URES
   * volt: minden blokkot feldolgoztunk, tehat mind a NQUEUE tranzakcio
   * fel van fuzve. Ez a pillanat, amikor a maximalis tartalek all
   * rendelkezesre — a mert 88 ms-os (max 167 ms) flush ide fer bele.
   *
   * Ha a gyuru sosem urulne ki (tulterhelt fo ciklus), a flashlog.cpp
   * hataridoi akkor is kiiratnak: naplot nem vesztunk. A halasztas
   * hosszat a 'logstat' es a FLUSH-sorok mutatjak. */
  flashlog_flush_window(drained == 0);
  flashlog_tick(now);            /* ritka kotegelt flash-flush + soros parancs */
#endif

  /* Osszevont hangolas: a csuszka huzasa kozben osszegyult keresekbol
   * csak az UTOLSO megy at, es az is legfeljebb TUNE_RATE_MS-enkent. */
#if SPY_ENABLE
  scan_flush(now);
#endif
  if (rtl_pending && now - rtl_last_tune_ms >= TUNE_RATE_MS) {
    rtl_pending = false;
    rtl_last_tune_ms = now;
    char cmd[16];
    snprintf(cmd, sizeof cmd, "F%lu", (unsigned long)rtl_pending_khz);
    DIAG.printf("\nrtl_tcp: hangolas %lu kHz\n",
                (unsigned long)rtl_pending_khz);
    cmdlink_send(cmd);
    oled_khz = rtl_pending_khz;      /* kerüljön ki a kijelzore is */
#if SPY_ENABLE
    spy_set_freq(rtl_pending_khz * 1000u);
#endif
  }

  if (t0 == 0) t0 = now;
  if (now - t0 >= 2000) {
    float dt = (now - t0) / 1000.0f;
    uint32_t mean = st_counted ? (uint32_t)(st_abssum / st_counted) : 0;

    DIAG.printf("%6.1f blokk/s %8.0f sps csucs=%+5.0f atlag=%+5.0f " LVL_UNIT
                " RF=%+d dBm "
                "VESZT=%lu magic=%lu csonka=%lu SPEC=%lu | "
                "USB %5.1f kB/s(-%lu)  NYERS %5.1f kB/s(-%lu)  "
                "RTL %5.1f kB/s(-%lu) sh=%d",
                st_blocks / dt, st_samples / dt, lvl_db(st_peak),
                lvl_db((int32_t)mean), st_rf_rssi, (unsigned long)st_lost,
                (unsigned long)st_badmagic, (unsigned long)st_badlen,
                (unsigned long)st_specline,
                (st_usbbytes / 1024.0f) / dt, (unsigned long)st_usbdrop,
                (st_rawbytes / 1024.0f) / dt, (unsigned long)ring_raw.dropped,
                (st_rtlbytes / 1024.0f) / dt, (unsigned long)ring_rtl.dropped,
                rtl_shift);
    /* ---- USB-DIAGNOSZTIKA (2026-08-06) ----
     * Ha a PC-oldali mero nem kap adatot, EDDIG talalgatni kellett, hogy a
     * firmware miert dobja el a blokkokat. Ez a ket szam megmondja:
     *
     *   conn=0             -> a CDC NEM lat csatlakozott hostot. Az
     *                         availableForWrite() ilyenkor definicio szerint 0,
     *                         tehat minden iras eldobodik. A hiba a host <-> CDC
     *                         line-state (DTR/RTS) kezelesben van.
     *   conn=1, afw < 1040 -> VAN host, de a CDC TX FIFO kisebb egy blokknal.
     *                         Gyogyszer: custom_sdkconfig-ba
     *                         CONFIG_TINYUSB_CDC_TX_BUFSIZE=4096
     *   conn=1, afw >= 1040-> az USB-ut egeszseges; ha megis nincs adat a PC-n,
     *                         a hiba a PC oldalan (driver / pyserial) van.
     *
     * A ket szamot EGYUTT kell nezni a fenti "USB x kB/s(-N)" mezovel. */
    DIAG.printf("  USB: conn=%d afw=%d (blokkhoz %u kell)",
                (int)(bool)RAW, (int)RAW.availableForWrite(),
                (unsigned)BLK_BYTES);

    /* A ciklusfrekvencia a legjobb korai figyelmezteto jel: ha ez leesik,
     * valami blokkolja a fo ciklust, es a kovetkezo dolog, ami elromlik, a
     * blokkvesztes. */
    DIAG.printf("  ciklus %lu/s", (unsigned long)(st_loops / (uint32_t)dt));
    st_loops = 0;
    if (st_back) DIAG.printf("  HATRA=%lu", (unsigned long)st_back);
    {
      uint32_t tot = st_pool_hit + st_pool_other;
      if (tot) {
        DIAG.printf("  POOL=%lu/%lu (%lu%%, NQUEUE=%d)",
                    (unsigned long)st_pool_hit, (unsigned long)tot,
                    (unsigned long)(100ul * st_pool_hit / tot), NQUEUE);
      }
      st_pool_hit = st_pool_other = 0;
    }
#if REORDER_BLOCKS > 0
    if (st_ro_dup || st_ro_hole || st_ro_late) {
      DIAG.printf("  REND: dup=%lu(E%lu/K%lu) lyuk=%lu keso=%lu",
                  (unsigned long)st_ro_dup,
                  (unsigned long)st_ro_dup_same,
                  (unsigned long)st_ro_dup_diff,
                  (unsigned long)st_ro_hole,
                  (unsigned long)st_ro_late);
    }
    /* KUMULATIV TUKOR a wifi_bench szamara. A statusz-szamlalok 2 mp-enkent
     * nullazodnak, tehat a bench 10 mp-es lepcso-deltaja ertelmetlen lenne
     * (negativ blk-delta bizonyitotta, 2026-08-17). Itt, MEG a nullazas
     * elott gyujtjuk osszet — ez monoton, tehat a delta helyes. */
    bench_tot_ro_dup  += st_ro_dup;
    bench_tot_ro_hole += st_ro_hole;
    st_ro_dup = st_ro_hole = st_ro_late = 0;
    st_ro_dup_same = st_ro_dup_diff = 0;
#endif
    DIAG.println();

    /* --- Az idozites-riport. Ezt kell nezni, nem a VESZT-et: a VESZT
     * csak a KOVETKEZMENY. qmax = mennyire kozelitette meg a torlodas az
     * NQUEUE-t (ha eleri, vesztettel); dt_max = a leghosszabb vak
     * idoszak; utana a bunosok listaja csokkeno sorrendben. --- */
    DIAG.printf("  IDO: qmax=%lu/%d  RXRING=%lu/%d ovf=%lu  dt_max=%lu us |",
                (unsigned long)st_qmax, NQUEUE,
                (unsigned long)rx_ring_max, RXRING_BLOCKS,
                (unsigned long)rx_ring_ovf, (unsigned long)st_dtmax);
    rx_ring_max = 0;
    for (int i = 0; i < S_COUNT; i++) {
      if (sect_us[i] >= 1000u)           /* csak az 1 ms folottiek */
        DIAG.printf(" %s=%lu", SECT_NAME[i], (unsigned long)sect_us[i]);
    }
    /* RAM-oraiv: szabad heap / eddigi minimum / legnagyobb osszefuggo.
     * A "min" a vizjel boot ota — ha az kuszik lefele, valami szivarog
     * vagy csucsterheleskor fogyunk el. A "max blokk" a toredezettseg:
     * ha a szabad sok, de a max blokk kicsi, nagy malloc mar nem megy. */
    DIAG.printf(" | RAM: %lu k szabad, min %lu k, max blokk %lu k",
                (unsigned long)(esp_get_free_heap_size() / 1024u),
                (unsigned long)(esp_get_minimum_free_heap_size() / 1024u),
                (unsigned long)(heap_caps_get_largest_free_block(
                                    MALLOC_CAP_8BIT) / 1024u));
#if FLASHLOG_ENABLE
    /* Egeszseg-pillanatkep a flashbe, ~10 s-onkent (5 x 2 s). A H-sorokbol
     * utolag latszik minden intermittens minta: dekod-arany, RAM-szivargas,
     * WiFi-RSSI, blokkvesztes, ciklus-lassulas. */
    static uint8_t fl_div = 0;
    if (++fl_div >= 5) {
      fl_div = 0;
#if APRS_RX_ENABLE
      flashlog_printf("H blk=%.0f pk=%+.0f avg=%+.0f rf=%d VESZT=%lu vett=%lu hiba=%lu "
                      "gate=%lu IS=%d ram=%luk min=%luk wifi=%d dtmax=%lu qmax=%lu",
                      st_blocks / dt, lvl_db(st_peak), lvl_db((int32_t)mean),
                      st_rf_rssi, (unsigned long)st_lost,
                      (unsigned long)aprs_rx_frames(), (unsigned long)aprs_rx_bad(),
                      (unsigned long)aprs_rx_gated(), aprs_rx_is_online() ? 1 : 0,
                      (unsigned long)(esp_get_free_heap_size() / 1024u),
                      (unsigned long)(esp_get_minimum_free_heap_size() / 1024u),
                      (int)WiFi.RSSI(), (unsigned long)st_dtmax,
                      (unsigned long)st_qmax);
#else
      flashlog_printf("H blk=%.0f pk=%+.0f avg=%+.0f rf=%d VESZT=%lu ram=%luk min=%luk "
                      "wifi=%d dtmax=%lu qmax=%lu",
                      st_blocks / dt, lvl_db(st_peak), lvl_db((int32_t)mean),
                      st_rf_rssi, (unsigned long)st_lost,
                      (unsigned long)(esp_get_free_heap_size() / 1024u),
                      (unsigned long)(esp_get_minimum_free_heap_size() / 1024u),
                      (int)WiFi.RSSI(), (unsigned long)st_dtmax,
                      (unsigned long)st_qmax);
#endif
    }

#endif
    /* --- ILI9341 allapotkijelzo. Ugyanaz a 2 mp-es utem, mint a soros
     * statusz. A kiiras SPI2-n megy; az FG23 slave a SPI3-on van, tehat a
     * ket busz nem versenyzik. FONTOS: a FLASHLOG_ENABLE #endif UTAN kell
     * allnia, kulonben (FLASHLOG_ENABLE 0 mellett) ki sem fordulna. --- */
    {
      static String tft_ip;
      tft_ip = wifi_up ? WiFi.localIP().toString() : String("-");
      tft_stat_t ts;
      ts.blokk_s      = st_blocks / dt;
      ts.sps          = (uint32_t)(st_samples / dt);
      if (ts.sps > 100) st_sps_out = ts.sps;   /* a panel skalajahoz */
      ts.csucs_dbfs   = lvl_db(st_peak);
      ts.rssi_dbm     = st_rf_rssi;
      ts.ram_k        = (uint32_t)(esp_get_free_heap_size() / 1024u);
      ts.ram_min_k    = (uint32_t)(esp_get_minimum_free_heap_size() / 1024u);
      ts.terheles_pct = (uint8_t)(rgb_load_get() * 100.0f);
      ts.lost         = st_ro_hole;
      ts.freq_khz     = 0;
      ts.ip           = tft_ip.c_str();
      ts.ssid         = "";
      tft_show(&ts);
    }
    st_qmax = st_dtmax = st_back = 0;
    memset(sect_us, 0, sizeof sect_us);
    if (st_blocks == 0) DIAG.print("  <-- nincs adat / nem fut a stream");
    DIAG.println();
#if APRS_RX_ENABLE
    static uint32_t prev_fr = 0, prev_bad = 0;
    if (aprs_rx_frames() != prev_fr || aprs_rx_bad() != prev_bad) {
      prev_fr = aprs_rx_frames(); prev_bad = aprs_rx_bad();
      DIAG.printf("APRS: %lu keret, %lu CRC-hiba, %lu felkuldve, IS=%s\n",
                  (unsigned long)aprs_rx_frames(), (unsigned long)aprs_rx_bad(),
                  (unsigned long)aprs_rx_gated(),
                  aprs_rx_is_online() ? "el" : "nincs");
    }
#endif

    bench_tot_blocks += st_blocks;
    bench_tot_lost   += st_lost;
    st_blocks = st_samples = st_lost = st_badmagic = st_badlen = 0;
    st_peak = 0; st_abssum = 0; st_counted = 0;
    st_usbbytes = st_rawbytes = st_rtlbytes = 0;
    st_usbdrop = 0;
    st_gap_shown = 0;
    ring_raw.dropped = ring_rtl.dropped = 0;
    t0 = now;
  }
}
# ionos-sdr-mini – USB-átvitel, RF-karakterizáció, SoapyIonos és szélessávú mérés

**Részletes mérési és fejlesztési jelentés, 2026. november 2–9.**
HA7DCD
Kártya: ionos-sdr-mini rev-B3 2L · rádiókártya: Silicon Labs BRD4265B (EFR32FG23, 434 MHz-es illesztés, 10 dBm) · gazda MCU: ESP32-S3 (N16R8)
Repo: `ionos-sdr/ionos-sdr`, ág: `board/ionos-mini-idf` (kivéve a SoapyIonos: `host/soapy-ionos`)
Angol változat: [`../ionos-mini-measurements-2026-11.md`](../ionos-mini-measurements-2026-11.md) · nyers adatok, szkriptek: [`measurements/ionos-mini-2026-11/`](../../measurements/ionos-mini-2026-11/)

**Mérés alatt eddig: ≈ 86 óra** (ebből ≈ 54 óra RF jelgenerátorral, ≈ 32 óra USB-soak) – bontás az angol változat 2. fejezetében.

---

## Tartalom

1. [Összefoglaló](#1-összefoglaló)
2. [A rendszer felépítése](#2-a-rendszer-felépítése)
3. [200 ksps 16 bites I/Q USB-n – a szűk keresztmetszet felderítése](#3-200-ksps-16-bites-iq-usb-n--a-szűk-keresztmetszet-felderítése)
4. [Hosszú USB-soakok és a ritka csomagvesztés](#4-hosszú-usb-soakok-és-a-ritka-csomagvesztés)
5. [Egy váratlan felfedezés: a mini a 2 m-es PHY-val fut](#5-egy-váratlan-felfedezés-a-mini-a-2-m-es-phy-val-fut)
6. [48 órás RF-karakterizáció 145 MHz-en](#6-48-órás-rf-karakterizáció-145-mhz-en)
7. [SoapyIonos – SoapySDR-driver (HOST-M3)](#7-soapyionos--soapysdr-driver-host-m3)
8. [Szélessávú mérés](#8-szélessávú-mérés)
9. [Mérési infrastruktúra – eszközök és tanulságok](#9-mérési-infrastruktúra--eszközök-és-tanulságok)
10. [Commitok és változások](#10-commitok-és-változások)
11. [Nyitott pontok, javasolt következő lépések](#11-nyitott-pontok-javasolt-következő-lépések)
12. [Függelék: fájlok, parancsok, reprodukálás](#12-függelék-fájlok-parancsok-reprodukálás)

---

## 1. Összefoglaló

| Terület | Eredmény |
|---|---|
| **USB-sávszélesség** | 200 ksps × 16 bit I/Q (813 kB/s) folyamatosan átmegy a hub-láncon, TinyUSB CDC-vel. Az ESP oldal nem szűk keresztmetszet; a Windows host időnként nem kérdez. |
| **USB-megbízhatóság** | 40 óra alatt 86 M blokkból 0,65 ppm veszteség (két ismert, külső okú esemény nélkül). 2 MB-os PSRAM-gyűrű nyeli el a host-szüneteket. |
| **Rádióbeállítás** | A 434 MHz-es kártyán a **145 MHz-es** FG23 PHY futott; 434-en csak a LO 3. felharmonikusán hallott. A mérés ezért 145 MHz-en folyt. |
| **Erősítés / linearitás** | dBFS = P_in + 53,68 dB, ±0,1 dB-en belül lineáris −125 … −56 dBm között; kivezérlés ~−52 dBm. |
| **Zaj / érzékenység** | Zajsűrűség −165,4 dBm/Hz → **NF ≈ 8,6 dB**; MDS (10 dB SNR / 500 Hz) **≈ −128 dBm** (434-es illesztésen, 145 MHz-en). |
| **Áteresztősáv** | 200 ksps-on −3 dB: 88 kHz – a PHY ~100 kHz-es csatornaszűrője határolja. Tükörelnyomás ≥ 42 dB. |
| **Frekvencia** | `F`-hangolással −11 ppm (túlkorrigált kristálykorrekció), korrekció nélkül +4,4 ppm; napi ingadozás 1,9 ppm p-p. |
| **SoapyIonos** | Windowson fut (GQRX, SoapySDRUtil, Python). Két Windows-specifikus hibát javítottunk. 2 M+ blokk a szélessávú mérésben, 4 elveszett. |
| **Szélessávú mérés** | A 145 MHz-es PHY osztójával **128,34–198,07 MHz** között vesz; azon kívül a `RAIL_ConfigChannels` lefagy (SWD-resettel helyreállítva). CW: NF 7,9–11,9 dB, MDS −129…−125 dBm; FSK (5 kbps) BER 10⁻²: medián −110 dBm. |

---

## 2. A rendszer felépítése

```
  antenna/VSG ──► BRD4265B (EFR32FG23)                 ESP32-S3                    PC (Windows)
                  RAIL RX, IQ FIFO                      SPI slave                   usbser.sys (COM9)
                  CIC decimálás (R = 2…64)  ──SPI──►    PSRAM gyűrű 2 MB  ──USB──►  websdr / SoapyIonos /
                  1040 B-os blokkok                     TinyUSB CDC-ACM             Python mérőszkriptek
                  (IQB2 / SPC1 / MSG1)                  cmdlink (UART) ◄── host-parancsok
```

- **FG23** (`firmware-fg23/iq_capture`): RAIL vétel IQ-módban (`RX_IQDATA_FILTLSB`, FIFO), kétlépcsős CIC, R=2/4 közvetlen első lépcső, R≥8 a 8 többszörösei. Mag 78 MHz (HFRCODPLL), SPI 19,5 MHz (R<8) / 4 MHz.
- **ESP32-S3** (`firmware-esp32/streaming`, `env ionos_mini`): SPI-ről blokkok, TinyUSB CDC a natív USB-OTG-n (vendorolt `esp_tinyusb` 1.7.6), host-API (`usb_host.cpp`), konzol a CH343-on (COM4).
- **Keretek**: 1040 bájt, magic `IQB2` (I/Q, uint32 LE sorszám a 4. bájton), `SPC1` (scan-spektrum), `MSG1` (szöveg: hello / válasz / státusz).
- **Hub-lánc a mérőgépen**: Intel xHCI → HP hub 03F0:036B → 3× Realtek RTS5411 → ESP natív USB (VID 303A, PID 4001).
- **Rátatábla** (fs_in = 400 kHz): R=2 → 200 ksps (781 blk/s, 813 kB/s), R=4 → 100 ksps (391 blk/s), R=8 → 50 ksps (195 blk/s).

---

## 3. 200 ksps 16 bites I/Q USB-n – a szűk keresztmetszet felderítése

### 3.1 Kiinduló helyzet
- A HWCDC (USB-Serial/JTAG) úton ~776 kB/s-os „plafont” láttunk. **Később kiderült: ezt a Python-oldali keretbontás okozta** (`buf = buf[1040:]` blokkonként → négyzetes költség, 781 blk/s-nál nem bírja). Offset alapú bontással eltűnt.
- 8 bites I/Q helyett Zoltán döntése: **a 16 bitet nem adjuk fel**, a cél az USB elméleti tudásának kihasználása.

### 3.2 TinyUSB CDC az USB-OTG-n
- Vendorolt `esp_tinyusb` 1.7.6 (`third_party/esp_tinyusb`, override_path), mert az Arduino `USBCDC.cpp` Kconfig-ütközik → `IONOS_TUSB_CDC_ENABLED`.
- `CFG_TUD_CDC_TX_EPSIZE 4096` (16384-nél a CDC teljesen leáll), TX FIFO 16 kB, DMA.
- Saját `TusbCdc` osztály: PSRAM-gyűrű + pump task (core 0, prio 6), a `tud_cdc_tx_complete_cb` ébreszti.

### 3.3 Mérések (konzolparancsok: `US`, `UF<s>`, `UP<prio>`, `UT<szó>`)
| Kísérlet | Eredmény |
|---|---|
| Flood (álladat, amilyen gyorsan a busz viszi) | ~205 db 4 kB-os átvitel/s, egyenként 3,5–5 ms → **870–950 kB/s állandósult** (~14–15 csomag/keret a 19 elméletiből; USB FS elmélet 1216 kB/s) |
| TinyUSB task prioritás 5 vs 24 | nincs különbség |
| FG23 stream be/ki (CPU-terhelés) | nincs különbség |
| EP2 hardveres TX FIFO 32 / 64 / 128 szó (`UT` futás közben) | nincs különbség |
| Double-buffered EP2 | nincs különbség |
| **Gap-szonda** (>10 ms átvitel nélkül, regiszterek) | átvitel élesítve (DIEPCTL2 EPENA), TX FIFO **tele**, eszköz-NAK nincs, SOF-ok mennek → **a host nem kérdez** |

**Következtetés:** az eszköz oldala nem szűk keresztmetszet. A tartalék ~7–15 % (870–950 vs 812,5 kB/s). Host-oldali szünetek: ~20 ms ~1,5 s-onként, ritkán több száz ms.

### 3.4 A TX-gyűrű mérete
- 256 kB (~320 ms) kevés: egy 3 órás soakban két >320 ms-os Windows-olvasási kimaradás → ESP-oldali eldobás (+157, +107 blokk).
- **2 MB PSRAM (~2,5 s)** óta egyetlen ESP-oldali eldobás sem volt (`623697e`).

---

## 4. Hosszú USB-soakok és a ritka csomagvesztés

### 4.1 Eredmények
| Futás | Időtartam | Blokk | ESP-eldobás | Host-oldali veszteség |
|---|---|---|---|---|
| 30 perces (256 kB) | 0,5 h | 1,36 M | 0 | 0 |
| 3 órás (256 kB) | 3 h | 8,44 M | 264 (2 host-kimaradás) | +4 egyblokkos |
| Éjszakai (2 MB) | 9,1 h | 25,65 M | **0** | 19 (0,00007 %) |
| Második éjszaka (2 MB) | 6,6 h | 18,6 M | **0** | 15 (0,00008 %) |

### 4.2 Az egyblokkos veszteség anatómiája
- Minden esetben **pontosan 128 bájt (2 × 64 B-os USB-csomag)** hiányzik egy blokkból, az ESP nem dob el semmit → a csomagok az eszköz és az alkalmazás között vesznek el.
- Nagy részük **10 perces rácsra** esik (pl. 22:41:22, 23:11:23, 01:11:24, 02:51:24 …, a másodperc óránként ~1 s-ot csúszik).
- Kizárva: Feladatütemező 10 perces feladata (nincs ilyen), ESP flash-napló (az `ionos_mini` buildben ki van kapcsolva), folyamatindulások (a procwatch nem mutat egybeesést).
- Nappal sokkal több (17 db / 16 perc), és a host terhelésével (folyamatindítások, VSG, böngésző) korrelál → **Windows-oldali késleltetési tüskék** valószínűek.
- A WiFi-kikapcsolásos A/B-teszt (`WIFIOFF`) nappali része zavaros volt; a 40 órás mérésben (WiFi ki) a veszteség 0,65 ppm.

---

## 5. Egy váratlan felfedezés: a mini a 2 m-es PHY-val fut

- VSG-tónus 433,845 MHz-en, −10…−90 dBm-en, 30 dB-es csillapítón át: **a kártya semmit nem látott**.
- VSG-söprés 425–445 MHz-en, 100 kHz-enként: jel **csak 435,2 MHz-en**, ott viszont 0 dBFS (−50 dBm-nél).
- Ok: a flashelt FG23 rádiókonfigja `base_frequency 144800000`, 25 kHz, 801 csatorna (144,8–164,8 MHz), `TUNE_BASE_KHZ 144800`. A `F433825` elérhetetlen csatornát kér → az FG23 eldobja, **az ESP mégis „OK”-t válaszol**. A rádió ~145,07 MHz-en állt, és a 434-re illesztett bemeneten a **LO 3. felharmonikusán** (3 × 145,07 ≈ 435,2 MHz) hallott.
- Zoltán döntése: az RX-illesztés megengedő, a mérés mehet **145 MHz-en** (néhány dB LNA-romlás).

---

## 6. 48 órás RF-karakterizáció 145 MHz-en

### 6.1 Elrendezés
- **VSG60** (Signal Hound, SCPI **5124-es port**) → **30 dB csillapító** → 1,5 m koax → BRD4265B SMA. A kártyán: VSG − 30,5 dB (a koax ~0,5 dB-je nincs levonva).
- VSG CW, moduláció kikapcsolva (az előző Arb-beállítás miatt; **a mérés után vissza kell kapcsolni**).
- Szkript: `longprog.py` (radioconda Python), háttér-olvasószállal. Egy kör ~19 perc:
  1. frekvencia-eltolás (tónus +20 kHz, −90 dBm);
  2. áteresztősáv + tükör 200/100/50 ksps-on (2/1/0,5 kHz-es lépés, −90 dBm);
  3. szintlépcső −40…−150 dBm (VSG −10…−120) 5 dB-enként, 200 ksps;
  4. USB-számlálók;
  5. 10 drift-pont ~40 s-onként.
- Futás: 11-06 17:42 → 00:00 (20 kör), majd a 23:59-es ütemező-hiba miatt szünet, **11-07 02:12 → 11-08 17:50: 126 kör, 0 hiba, 86,0 M blokk**.

### 6.2 Erősítés, linearitás, kivezérlés

| P_in (kártyán) | Tónus [dBFS] | Erősítés [dB] | max\|z\| |
|---|---|---|---|
| −150,5 | −85,4 (zajkorlát) | – | 0,006 |
| −130,5 | −76,2 | 54,3 (zaj torzítja) | 0,006 |
| −120,5 | −66,71 | 53,79 | 0,006 |
| −110,5 | −56,77 | 53,73 | 0,007 |
| −100,5 | −46,77 | 53,73 | 0,010 |
| −90,5 | −36,86 | 53,64 | 0,019 |
| −80,5 | −26,83 | 53,67 | 0,051 |
| −70,5 | −16,82 | 53,68 | 0,149 |
| −60,5 | −6,84 | 53,66 | 0,460 |
| −55,5 | −1,84 | 53,66 | 0,814 |
| −50,5 | +1,19 | 51,69 | **1,299 (klippel)** |
| −40,5 | +1,52 | 42,02 | 1,369 |

- **Erősítés: 53,68 dB** (−125…−56 dBm, 1638 pont, szórás 0,10 dB). Körről körre −90 dBm-en 0,03 dB szórás – a jelút két nap alatt sem mozdult.
- **Kivezérlés: ~−52 dBm** a kártyán.

### 6.3 Zaj és érzékenység
- Zajsűrűség (200 ksps): **−111,7 dBFS/Hz** → bemenetre −165,4 dBm/Hz → **NF ≈ 8,6 dB** (434-es illesztésen, 145 MHz-en, a koaxszal együtt).
- **MDS** (10 dB SNR 500 Hz-en): **≈ −128 dBm**.
- Dinamika: ~86 dB / 500 Hz, ~60 dB / 200 kHz.
- 100 és 50 ksps-on a zajsűrűség 2,6–2,9 dB-lel rosszabb (−108,8 / −109,1 dBFS/Hz): a széles PHY-csatornaszűrőn túli zaj a CIC-decimáláskor visszahajlik.

### 6.4 Áteresztősáv és tükörelnyomás (126 kör mediánja)

| Ráta | −1 dB | −3 dB | −6 dB | −20 dB | 0,45·fs-nél | Tükör (\|f\| ≤ 0,3·fs) | Sávon belüli hullámosság |
|---|---|---|---|---|---|---|---|
| 200 ksps | 74 kHz | **88 kHz** | 100 kHz | 128 kHz | −80 dB | **≥ 42,3 dB** | (a sáv széle már lejt) |
| 100 ksps | 27 kHz | 59 kHz | 81 kHz | 98 kHz | −8 dB | ≥ 45,9 dB | 2,7 dB p-p (±25 kHz) |
| 50 ksps | 16,5 kHz | 28 kHz | 41 kHz | 49 kHz | −7 dB | ≥ 48,6 dB | 2,5 dB p-p (±12,5 kHz) |

- 200 ksps-on a sávot **a PHY ~100 kHz-es csatornaszűrője** szabja meg, nem a CIC → a 200 ksps-ból ~90 kHz használható.
- A tükörelnyomás mérése zajkorlátos (a valós érték jobb lehet).
- **Spurok**: a tónus két oldalán ±6,25 kHz-en ~−40 dBc-s oldalvonalak (FG23 vagy VSG – tisztázandó).

### 6.5 Frekvencia és drift
- Eltérés (`F145000`, `o0`): **átlag −1596 Hz = −11 ppm**, tartomány −1464 … −1741 Hz (**277 Hz = 1,9 ppm p-p**), szórás 66 Hz, 1376 pont.
- Szabályos napi ciklus: hajnalban a leghidegebb (legnegatívabb), kora délután a legmelegebb; a két nap jól ismétlődik.
- A szélessávú mérés `B`-módja (kristálykorrekció nélkül) **+4,4 ppm**-et mutat → **a −11 ppm a firmware kristálykorrekciójának túlkorrigálása** (`FREQ_CORR_PPB` / tick-skála hibás).

### 6.6 USB a 40 óra alatt
- 86,0 M blokk: 19 660 az újraindításkori sorszám-ugrás (az új ESP-firmware csatlakozáskor eldobja a puffert), 591 egy teljes lemezszkennelés alatt (több másodperces Windows-olvasási kimaradás). **Ezeken kívül 56 blokk = 0,65 ppm**, éjjel órákig 0.

---

## 7. SoapyIonos – SoapySDR-driver (HOST-M3)

### 7.1 Fejlesztés
- Felhőben futó kódoló agent írta a protokoll-leírás (`docs/ionos-usb-protocol.md`) és a firmware alapján, mock eszközzel (pty) Linuxon tesztelve: 8/8 teszt, a nullázás és az újraszinkronizálás pontosan annyi blokkot pótol, amennyit a mock eldobott.
- Ág: `host/soapy-ionos`, mappa `host/soapy-ionos/` (CMake, `src/IonosProtocol|Serial|Device|Registration`, `test/`), MIT licenc.

### 7.2 Tudás
- Felismerés VID/PID + hello alapján; port DTR=1 / RTS=0.
- Frekvenciatartomány, ráták (`decim_list`), erősítés a `CAP?`/hello-ból.
- CS16 / CF32 / CS8. Háttérszál, 64 kB-os olvasás, offset alapú keretbontás.
- Elveszett blokk → nullákkal pótolva + `SOAPY_SDR_OVERFLOW`; csonka blokk (128 B hiány) → újraszinkronizálás; sorszám-visszaesés → `stream_restarts`.
- Szenzorok: `lost_blocks`, `gap_events`, `truncated_blocks`, `resyncs`, `stream_restarts`, `host_overflows`, `blocks`, `rssi`, `dev_*`.
- `command` beállítás: nyers host-parancs (TX/diagnosztikai betűk tiltva).

### 7.3 Windows-javítások (élesben, a valódi kártyán)
| Commit | Hiba | Javítás | Eredmény |
|---|---|---|---|
| `5847630` | 30 s alatt 400 elveszett blokk 200 ksps-on | Olvasás `ReadIntervalTimeout=0`, 20 ms / 64 kB; nincs „nap” Windowson; `SetupComm` 4 MB | 60 s, 46 901 blokk, **0** veszteség |
| `dbae918` | SoapySDRUtil/PothosSDR alatt a hello után „v1”-nek látta a kártyát | Az olvasó szál az első `std::mutex`-zárnál beragadt (VS 2022 17.10+ constexpr mutex + régi `msvcp140.dll`) → `_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR` | `--find`/`--probe` rendben, v2 protokoll |

### 7.4 Gyakorlati próbák
- **GQRX** (radioconda): `soapy=0,driver=ionos`, 200 kS/s, élő vízesés, 0 veszteség. A hangoláskor látható vízszintes csíkok az FG23 stream-újraindulásai (minden `FH` újraindítja a streamet, a sorszám 0-ról indul).
- **Szélessávú mérés** a driveren át: 2,0 M blokk, 1753 stream-újraindulás, 4 elveszett blokk, 1 csonka, 0 host-túlcsordulás.
- SDR++: a jelenlegi Windows-buildben nincs `soapy_source` (csak a régi `_OLD` változatokban).

### 7.5 A driver-agent által talált spec/firmware-eltérések
1. VID/PID: a spec 0x1001-et ír, a TinyUSB-build 0x4001-et használ (az esp_tinyusb alapértéke).
2. **Sorozatszám: minden kártya „123456”**; a hello-ban nincs MAC → több kártya nem különböztethető meg.
3. Hello: `gain_shift_max=0`, plusz `mode=` kulcs.
4. Státusz: `usbdrop` (spec) vs `usb_drop` (firmware).
5. Újraszinkronizálási szabály ellentmondásos (16 B-os igazítás vs 4 B-os lépés); a 128 B-os csonka blokkot nem fedi le.
6. **A sorszám minden F/FH/R-nél 0-ról indul** – a spec nem írja.
7. **`FH` a hangolás előtt mond OK-t**; nincs jelölés, honnan szól az új frekvencia.
8. Nincs `IQ ON/OFF` – az `i` csak vált.
9. Gyors újranyitásnál elmarad a hello → a driver 150 ms után `CAP?`-ot küld.
10. `s_host_spoke` nem nullázódott (azóta javítva: `487569e`).
11. „overflow flag” vs SoapySDR visszatérési kód.
12. `MODE USB` szövege: az I/Q mindkét módban megy USB-re, a mód csak a demodulátorokat kapcsolja.

---

## 8. Szélessávú mérés

### 8.1 Cél
Zoltán kérése: a CW- és FSK-érzékenység frekvenciafüggése a 434 MHz-es illesztés mellett, sávon kívül is, és a chip fizikai hangolási határainak kitapogatása, sok pontban.

### 8.2 Új FG23-parancs: `B<kHz>`
- A PHY első csatornájának RAM-másolata új alapfrekvenciával (egy csatorna, friss IR-kalibrációs attribútumok) → `RAIL_ConfigChannels()`; `B0` visszaáll a generált PHY-ra, az `F` előbb elhagyja a felülírást.
- **1. hiba – 0 blokk:** a `RAIL_ConfigChannels()` újra alkalmazza a PHY regiszterkészletét, benne a csomagmódot → utána az I/Q-adatforrást (`RX_IQDATA_FILTLSB`, FIFO) újra be kell állítani.
- **2. hiba – lefagyás:** elérhetetlen alapfrekvenciánál (a PHY szintézer-osztója nem ér el) a `RAIL_ConfigChannels()` **sosem tér vissza**. Kimutatva SWD-n olvasott RAM-breadcrumbokkal (`g_b_dbg[]`, `FGRD 200010d4 6` → step 3).
- **Helyreállítás:** a `FGINFO` nem reseteli az FG23-at → új ESP-konzolparancs **`FGRST`** (SWD `AIRCR.SYSRESETREQ`), utána `FGINFO`, ~5 s múlva újra streamel. A mérőszkript minden adat nélküli pontnál ezt csinálja.

### 8.3 Módszer
- `wideband_soapy.py` a **SoapyIonos driveren át** (a mérés egyben a driver hosszú tesztje), tartalék `wideband.py` (közvetlen soros).
1. **Térkép**: 1 MHz-es lépés 145 → 1100 MHz, majd 144 → 50 MHz; VSG −60 dBm (kártyán −90,5 dBm), tónus +20 kHz, 200 ksps; „vesz”, ha a tónus SNR-je > 25 dB/bin. Élek 25 kHz-re bisectelve.
2. **CW**: 250 kHz-es rács a működő tartományokban, 50 ksps, −100,5 dBm: erősítés, zajsűrűség, NF-becslés, MDS (10 dB / 500 Hz).
3. **FSK**: 2-FSK, 5 kbps, ±2,5 kHz, PRBS9 (VSG Arb, 100 kS/s), diszkriminátoros demodulátor + PRBS-illesztett BER-számlálás; bináris keresés a BER = 10⁻²-es szintre, 1 MHz-es rács.

### 8.4 Eredmények (a mérés 11-08 23:06-kor végzett)

**Hangolási térkép (1051 pont, 50–1100 MHz, 1 MHz-enként):**
- **Vételi tartomány a 145 MHz-es PHY osztójával: 128,34 … 198,07 MHz** (élek 25 kHz-re bisectelve: alsó 128,328–128,344, felső 198,062–198,078 MHz).
- A szélek mellett (125–128 és 199–203 MHz) a stream fut, de a szintézer nem zár (nincs tónus).
- Ezen kívül minden pont lefagyasztja a `RAIL_ConfigChannels()`-t → ~950 `FGRST`, mind sikeres.
- Ez **a PHY VCO-osztójának tartománya, nem a chip határa**; a 434/868/915 MHz-es sávokhoz más osztójú PHY kell.

**CW (277 pont, 250 kHz-enként, 50 ksps, −100,5 dBm):**

| f [MHz] | tónus [dBFS] | frekvenciahiba [Hz] | NF [dB] | MDS [dBm] (10 dB / 500 Hz) |
|---|---|---|---|---|
| 129 | −45,37 | +540 | **7,9** | **−129,1** |
| 140 | −45,89 | +589 | 8,8 | −128,2 |
| 150 | −46,37 | +626 | 9,7 | −127,3 |
| 160 | −47,04 | +676 | 11,6 | −125,4 |
| 170 | −47,69 | +709 | 9,9 | −127,1 |
| 180 | −48,47 | +761 | 10,2 | −126,9 |
| 190 | −49,48 | +793 | 11,1 | −125,9 |
| 198 | −50,33 | +827 | 11,9 | −125,1 |

- Erősítés (50 ksps): 55,1 dB → 50,2 dB a sávon át (~5 dB esés). NF medián 10,1 dB.
- Néhány ponton (133,75; 136,5; 144,25; 152,25; 156; 160,25; 163,75; 172,75; 175,5; 183,25; 191,25; 195 MHz) az NF-becslés 14–29 dB – valószínűleg külső zavar/spur a zajmérés sávjában (pl. 144,25 MHz amatőr forgalom), nem a vevő romlása.
- Frekvenciahiba korrekció nélkül: **+4,2 ppm**, a frekvenciával arányos → a kristály valós hibája; a rendes `F`-hangolás −11 ppm-je a firmware túlkorrigálása.

**FSK (70 pont, 1 MHz-enként; 2-FSK 5 kbps, ±2,5 kHz, PRBS9; a BER = 10⁻² szint):**

| f [MHz] | 129 | 135 | 140 | 145 | 150 | 155 | 160 | 165 | 170 | 175 | 180 | 185 | 190 | 195 | 198 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| szint [dBm] | −111,9 | −111,4 | −111,4 | −110,9 | −110,5 | −110,5 | −106,2 | −108,6 | −108,6 | −110,5 | −108,6 | −109,1 | −109,1 | −104,4 | −108,1 |

- **Medián −110,0 dBm**, legjobb −111,9 dBm (129 MHz). A CW-hez hasonlóan felfelé ~2–3 dB-t romlik.
- Kiugró pontok: **169 MHz (−83,3 dBm)**, 156 (−102,0), 195 (−104,4), 160 (−106,2) – ugyanott, ahol a CW-zajmérés is zavart mutatott → valószínűleg külső jelek/spurok; a 169 MHz-es kiugrást ismételt méréssel kell ellenőrizni (VSG Arb-alulcsordulás is lehet).
- Elméleti viszonyítás: nem koherens 2-FSK (h=1), BER 10⁻² ≈ Eb/N0 ~9 dB; −174 + NF(~9) + 10·log10(5000) + 9 ≈ −119 dBm. A mért −110 … −112 dBm ~7–9 dB-lel rosszabb, ami az egyszerű diszkriminátoros demodulátorra (nincs illesztett szűrő, 50 kHz-es zajsáv) jellemző – a vevőt nem, a referencia-demodulátort minősíti.

**A SoapyIonos driver a teljes mérés alatt:** 2,98 M blokk, 2385 stream-újraindulás, 4 elveszett, 1 csonka blokk, 0 host-túlcsordulás – hosszú, sok újraindulásos terhelés alatt is stabil.

**Lezárás:** a VSG kimenete kikapcsolva, a moduláció visszakapcsolva.

### 8.5 Utómérés a gyanús pontokon (11-08 23:49 és 11-09 09:01, `recheck.py`)

**(B) Zajspektrum kikapcsolt VSG-vel, 200 ksps (16 pont):**

| f [MHz] | zajsűrűség [dBFS/Hz] | megjegyzés |
|---|---|---|
| 129, 133,75, 144,25, 145, 152,25, 160, 160,25, 163,75, 169, 172,75, 183,25, 191,25 | −110,6 … −113,9 | tiszta (a csúcsok ≤ 12 dB a medián felett) |
| **136,5** | **−97,7** | ~14 dB-lel magasabb, szélessávú, egyenletes „csúcserdő” |
| **156,0** | **−104,8** | ~7 dB-lel magasabb, szélessávú |
| **175,5** | **−102,4** | ~9 dB-lel magasabb, szélessávú |
| **195,0** | **−108,2** | ~4 dB-lel magasabb, szélessávú |

→ A 2. fázis „rossz NF”-pontjainak egy része (136,5; 156; 175,5; 195) **szélessávú zavar** (külső jel vagy belső spur-sáv), nem a vevő romlása; a többi gyanús pont VSG nélkül tiszta – ott a CW-mérés pillanatnyi zavart fogott.

**(C) A ±6,25 kHz-es oldalvonalak (145 MHz, `B145000`, 200 ksps):**

| tónus-eltolás | −6,25 kHz [dBc] | +6,25 kHz [dBc] | ±3,125 kHz [dBc] |
|---|---|---|---|
| +5 kHz | −73 … −74 (−70,5 dBm-en) | −74 … −75 | ≤ −77 (zaj) |
| +20 kHz | **−50,7 … −50,8** | **−52,8** | ≤ −78 (zaj) |
| +37 kHz | – | **−43,2** | – |

- A dBc-érték **szintfüggetlen** (−70,5 … −90,5 dBm között azonos) → lineáris jelenség, nem kivezérlés.
- **Erősen függ a tónus középtől mért távolságától** (5 kHz: −74, 20 kHz: −51, 37 kHz: −43 dBc) → nem a VSG AM/PM-je (az eltolástól független lenne), hanem a **vevő I/Q-útjának 6,25 kHz-es periodikus időzítési/fázishibája** (6,25 kHz = 400 kHz / 64 → az FG23 bemeneti mintáinak 64-es blokkjaihoz köthető; pl. FIFO-olvasási vagy CIC-fázisugrás). A javítás az FG23 decimáló/FIFO-útjában keresendő.
- Anomália: a +37 kHz-es beállításnál a legnagyobb csúcs −27,9 kHz-en volt teljes szinten – tisztázandó (alias vagy rossz csúcs-azonosítás).

**(A) FSK-kiugrások újramérése (3× pontonként):**

| f [MHz] | BER 10⁻² szint [dBm] (3 mérés) | korábban |
|---|---|---|
| 129 | −112,4 / −112,1 / −112,1 | −111,9 |
| 145 | −111,0 / −110,7 / −111,0 | −110,9 |
| 155 / 157 | −110,2…−110,5 | −110,5 |
| **156** | **−97,3 / −97,3 / −97,1** | −102,0 |
| 159 / 161 | −109,9…−110,5 | – |
| **160** | −107,2 / −107,7 / −104,7 | −106,2 |
| 168 / 170 | −108,3…−110,2 | −108,6 |
| **169** | **−109,9 / −110,2 / −110,2** | **−83,3** |
| 194 / 196 | −108,6…−108,8 | – |
| **195** | −103,4 / −104,7 / −105,0 | −104,4 |

- **A 169 MHz-es −83 dBm egyszeri hiba volt** (most −110 dBm, mint a szomszédai) – valószínűleg VSG Arb-kimaradás vagy átmeneti zavar a mérés közben.
- **156 és 195 MHz tartósan rosszabb** (−97 ill. −104 dBm), pontosan ott, ahol a zajspektrum szélessávú zavart mutat → nem a vevő, hanem a zavar.
- A driver az utómérés alatt: 275 k blokk, 46 újraindulás, **0 veszteség**.

**Lezárás:** a VSG kimenete kikapcsolva, a moduláció visszakapcsolva (11-09 09:22). (Az utómérés első futása 23:52-kor egy szkripthiba miatt leállt, a VSG addig −40 dBm-en, moduláció nélkül, a 30 dB-es csillapítón át bekapcsolva maradt.)

---

## 9. Mérési infrastruktúra – eszközök és tanulságok

### 9.1 Eszközök (`measurements/ionos-mini-2026-11/scripts`)
| Fájl | Szerep |
|---|---|
| `longprog.py` | 48 órás karakterizáció (háttér-olvasó, VSG-vezérlés, CSV-k) |
| `wideband_soapy.py`, `soapyio.py` | szélessávú mérés a SoapyIonos driveren át |
| `wideband.py` | ugyanez közvetlen soros olvasással |
| `fsk_lib.py` | 2-FSK generátor (VSG Arb) + demodulátor + BER |
| `soak200.py`, `soak_full.py` | USB-soakok (sorszám-ellenőrzés, konzol-napló, másodperces statisztika) |
| `flood_exp*.py` | USB-flood kísérletek (`US`/`UF`/`UP`/`UT`) |
| `con4.py` | ESP-konzol parancs (COM4, DTR/RTS = 0) |
| `vsg60.py` | VSG60 SCPI-vezérlés visszaolvasással |
| `fg23_flashB.ps1` | FG23-flash az ESP-n át (`tools/fg23_flash.py`, FGFW) |

### 9.2 Tanulságok
1. **`schtasks /sc once /st 23:59` + azonnali `/run`: a 23:59 tényleg lefut.** Péntek 23:59-kor minden aznapi segédfeladat újraindult (ESP-újraflash, régi soak elvette a COM9-et) → ~2 óra kiesés. Azóta: indítás után azonnal `/change /disable`.
2. **Elavult I/Q**: ha a szkript a VSG-állítás alatt nem olvas, a 2 MB-os gyűrű megtelik, és a mérés a *korábbi* VSG-állapotot látja (~2,5 s késés). Megoldás: folyamatosan olvasó háttérszál.
3. **A VSG60 kimenete 1–1,5 s-mal a parancs után áll át** (frekvencia és szint is) → lépésenként 1,6 s várakozás.
4. **VSG SCPI az 5124-es porton** (nem 5024); a modulációt CW-hez ki kell kapcsolni.
5. **Nagy lemezterhelés** (teljes meghajtó-szkennelés) a 2 MB-os puffert is kifuttatja.
6. **FG23-flash után** a stream csak egy `FGINFO` (SWD debug-elengedés) után indul.
7. **Elérhetetlen PHY-frekvencia → RAIL-lefagyás**; a `FGINFO` nem resetel, a `FGRST` igen.
8. **Windowsos Python-mérés**: a magyar Windowson a teljesítményszámláló-nevek lokalizáltak → CIM-osztályok kellenek.
9. **Kivétel esetén a VSG bekapcsolva maradhat** (egyszer egész éjjel −40 dBm-en) → a kimenetet `finally`-ágban kell kikapcsolni.

---

## 10. Commitok és változások

**`board/ionos-mini-idf`**:
| Commit | Tartalom |
|---|---|
| `626c2e6` | I/Q 100 / 200 ksps (R = 4 / 2): FG23 és SPI-link |
| `65aecee` | Natív USB-OTG + TinyUSB CDC, 200 ksps a hostra |
| `acc3824` | TinyUSB-diagnosztika (`US`/`UF`/`UP`/`UT`, gap-szonda), 200 ksps veszteségmentes |
| `623697e` | TinyUSB TX-gyűrű 2 MB PSRAM-ban |
| `487569e` | FG23 `B<kHz>`, I/Q-adatkonfig `RAIL_ConfigChannels` után, breadcrumbok; ESP `B`-továbbítás, `FGRST`, `WIFIOFF`/`WIFION`, csatlakozáskori pufferürítés |

**`host/soapy-ionos`**: `2eb2b2c` (driver, agent), `5847630` (Windows-olvasás), `dbae918` (MSVC-mutex).


---

## 11. Nyitott pontok, javasolt következő lépések

**FG23 firmware**
1. **430–440 MHz-es PHY** (base 430,000 MHz, 25 kHz, 401 csatorna, `TUNE_BASE_KHZ 430000`) és **szélesebb csatornaszűrő** (~200 kHz) a 200 ksps teljes kihasználásához.
2. **Kristálykorrekció javítása**: a −11 ppm túlkorrigálás; a valós hiba +4,4 ppm.
3. **Zökkenőmentes hangolás**: a sorszám folytatódjon, a hangolás pillanata fejlécbitben, a beállás idejére nullázott minták.
4. **Hangolási hiba jelzése**: elérhetetlen frekvenciánál hiba; az ESP csak FG23-nyugta után mondjon OK-t.
5. **±6,25 kHz-es oldalvonalak**: a vevő I/Q-útjának 6,25 kHz-es (400 kHz/64) periodikus időzítési hibája – az FG23 FIFO-olvasásában / CIC-fázisában keresendő (az eltolással nőnek: 5 kHz −74, 20 kHz −51, 37 kHz −43 dBc).
6. `B`-mód: frekvencia-validálás a lefagyás elkerülésére (vagy RAIL-szintű időtúllépés).

**ESP / protokoll**
7. MAC-alapú USB-sorozatszám és `serial=` a hellóban.
8. `IQ ON`/`IQ OFF`, a spec frissítése a driver-agent 12 pontja szerint.
9. USB-módban SpyServer/WiFi-stream kikapcsolása, automatikus módváltás host-csatlakozásra.

**Host**
10. SoapyIonos: SDR++ (soapy_source-os build), GNU Radio próba; scan (SPC1) kiajánlása.
11. websdr 200 k-s span élesben.

**Mérés**
12. Ugyanez a szélessávú mérés 434 / 868 / 915 MHz-es (más VCO-osztójú) PHY-kkal; a 136,5 / 156 / 175,5 / 195 MHz-es szélessávú zavar forrásának azonosítása (külső vagy belső).
13. Jobb FSK-referencia-demodulátor (illesztett szűrő) a vevő valós FSK-érzékenységéhez.

---

## 12. Függelék: fájlok, parancsok, reprodukálás

### 12.1 Konzolparancsok (ESP, COM4, 115200, DTR/RTS = 0)
| Parancs | Hatás |
|---|---|
| `US` | USB-statisztika (átvitelek, hisztogram, gap-szonda) |
| `UF<s>` | flood-teszt |
| `UP<prio>` | TinyUSB task prioritás |
| `UT<szó>` | EP2 TX FIFO átméretezése |
| `WIFIOFF` / `WIFION` | WiFi ki/be |
| `FGINFO` | FG23 SWD-info (debug-elengedés) |
| `FGRD <hex> [n]` | FG23 RAM/regiszter olvasása (NS alias perifériákhoz!) |
| `FGRST` | FG23 rendszer-reset SWD-n |
| `FGFW …` | FG23 flash (a `tools/fg23_flash.py` használja) |

### 12.2 Host-API (CDC, soronként)
`CAP?`, `VER?`, `FH<Hz>`, `R <sps>`, `MODE USB|STANDALONE`, `STAT <s>`; továbbítva az FG23-nak: `F o f i W T t s B`.

### 12.3 Adatfájlok
- 48 órás: `longprog_20261107_0212.log`, `lp_{drift,passband,level,usb}_20261107_0212.csv` (+ `_20261106_1742`).
- Szélessávú: `soapyrun_20261108_1820.log`, `sp_wb_{map,edge,cw,fsk}_20261108_1820.csv`.
- USB-soakok: `soak_full_*.log.gz`, `console_*.log.gz`.

### 12.4 Reprodukálás röviden
1. ESP: `pio run -e ionos_mini -t upload`.
2. FG23: Simplicity-munkaterület build → `python tools/fg23_flash.py <hex> --port <konzol COM>` → `FGINFO`.
3. SoapyIonos: `cmake -S host/soapy-ionos -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="C:\Program Files\PothosSDR"` → `cmake --build build --config Release` → `SOAPY_SDR_PLUGIN_PATH=build\Release`.
4. Mérés: radioconda Python (numpy, pyserial, SoapySDR-kötés), VSG szoftver futva, SCPI engedélyezve (5124).

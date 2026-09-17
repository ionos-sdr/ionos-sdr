# fg23_scan_source — szélessávú scan-waterfall SDR++-ban a FG23/ESP32 SpyServerről

HA7DCD, 2026-08. GPL-3 (az SDR++ spyserver_source származéka).

## Mi ez

Az SDR++ gyári SpyServer-modulja csak IQ-t kér és a vízesést maga számolja
belőle. Ez a modul **két üzemmódot** ad:

| Mód | Mit kér a szervertől | Mit látsz | Hang |
|---|---|---|---|
| **IQ** | `STREAM_MODE_IQ_ONLY` (mint a gyári) | keskeny (~1 MHz), valódi FFT | van |
| **SCAN** | `STREAM_MODE_FFT_ONLY` + `SETTING_FFT_*` | **több MHz** léptetett spektrum, `UINT8_FFT` sorok közvetlenül a `gui::waterfall`-ba | nincs |

Kattintás a vízesésre SCAN-módban → csak a scan-középfrekvenciát tolja
(`SETTING_FFT_FREQUENCY`). Hallgatáshoz válts IQ-ra (a mód-combo).

## Fájlok

```
fg23_scan_source/
  CMakeLists.txt
  src/main.cpp            — a modul (menü, módok, waterfall-injektálás)
  src/fg23_client.{h,cpp} — SpyServer-kliens + UINT8_FFT/DINT4_FFT kezelés
  src/spyserver_protocol.h — a gyári header (változatlan)
  esp32/spy_fft.h         — ESP32-oldali kiegészítés a spyserver.cpp-hez
  esp32/oled_spectrum.h   — 128×64 OLED mini-spektrum + 1-bites waterfall
```

## Fordítás (asztali SDR++)

1. Másold a mappát az SDR++ forrásfa `source_modules/` alá.
2. A gyökér `CMakeLists.txt`-ben a többi source-modul mintájára:
   ```cmake
   option(OPT_BUILD_FG23_SCAN_SOURCE "FG23 scan source" ON)
   ...
   if (OPT_BUILD_FG23_SCAN_SOURCE)
   add_subdirectory("source_modules/fg23_scan_source")
   endif (OPT_BUILD_FG23_SCAN_SOURCE)
   ```
3. Szokásos build (`cmake .. && make`). A `fg23_scan_source.so/.dll` a
   modules mappába kerül; SDR++ → Module Manager → add `fg23_scan_source`.
4. Source: **„FG23 Scan"**.

A modul a klónozott SDR++ fejlécei ellen szintaktikailag ellenőrizve
(`g++ -fsyntax-only`); teljes link a te buildkörnyezetedben.

Android: a modulok az APK-ba fordulnak — az `android/` mappa + a CI-recept
szerint (NDK). Ez külön menet, előbb asztalon teszteld.

## Protokoll-megállapodás (kliens ↔ ESP32)

A SpyServer szabványos FFT-settingjeit használjuk, egy újraértelmezéssel:

| Setting | Jelentés nálunk |
|---|---|
| `SETTING_FFT_FREQUENCY` (201) | scan-középfrekvencia [Hz] |
| `SETTING_FFT_DECIMATION` (202) | **SPAN [Hz]** (nem decimációs index) |
| `SETTING_FFT_DISPLAY_PIXELS` (205) | nbin (320/640/1024/2048) |
| `SETTING_FFT_DB_OFFSET` (203) | −floor dBm (pl. 130) |
| `SETTING_FFT_DB_RANGE` (204) | tartomány dB (pl. 100) |
| `SETTING_STREAMING_MODE` (0) | `FFT_ONLY`=4 → scan; `IQ_ONLY`=1 → hallgatás |

Üzenet: `MSG_TYPE_UINT8_FFT` (301), `StreamType=FFT` (4), törzs `nbin × uint8`,
`dB = floor + v·range/255`. **Fejléc+törzs egyben a gyűrűbe** (`spy_fft_send`).

Az SDR++ oldal a sort a waterfall nyers FFT-hosszára (Display → FFT size,
alap 65536) húzza fel: kevesebb bin → lineáris interpoláció, több bin →
max-decimálás.

## ESP32-integráció (spyserver.cpp)

```cpp
#include "spy_fft.h"
// CMD_SET_SETTING:
if (!spy_fft_handle_setting(t.Setting, t.Value)) { /* régi switch */ }
// fő hurok:
if (spy_fft_state().dirty) {
    const auto& p = spy_fft_state();
    if (p.fft_enabled) fg23_cmd_scan(p.freq_hz, p.span_hz, p.nbin);  // CMD-vonal
    else               fg23_cmd_iq(iq_freq, iq_decim);
    spy_fft_clear_dirty();
}
// SPECLINE a FG23-tól:
spy_fft_send(ring_enqueue_whole, spec.bins, spec.nbin);
ili9341_wf.push(spec.bins, spec.nbin);
oled.push(spec.bins, spec.nbin);
```

A `ring_enqueue_whole(p1,n1,p2,n2)` a te „egyben vagy sehogy" gyűrű-írásod
(az Android-kifagyás javításának mintája).

## Nyitott / következő

- FG23-oldal: scan-PHY (fix gain, széles RX-BW), RSSI-léptetés → SPECLINE
  (lásd a projekt-doc sweep-tervét).
- CMD-vonal (PA06/EXP11 ← GPIO4) behúzása: enélkül a scan-paraméterek nem
  jutnak le a FG23-ra.
- Ha SDR++-ban „vízesés + hang egyszerre" kell: az az upstream #1356
  (FFT+VFO mode), ez a modul nem az.

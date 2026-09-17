// oled_spectrum.h — 128x64 SSD1306 mini-spektrum + 1-bites waterfall (U8g2)
// HA7DCD, 2026-08. Header-only.
//
// Elrendezes: felso 24 px spektrumgorbe (max-hold nelkul), also 40 px
// 1-bites, Bayer-4x4 ditherelt waterfall (~5 szurkeszint erzet).
// Bemenet: barmilyen nbin-es uint8 sor (a SPECLINE-e); 128 binre max-decimal.
//
// HASZNALAT:
//   OledSpectrum oled;              // globalis
//   ...SPECLINE erkezik: oled.push(bins, nbin);
//   ...kirajzolas (pl. 10 Hz): oled.draw(u8g2);   // u8g2.clearBuffer/sendBuffer benne

#pragma once
#include <stdint.h>
#include <string.h>

class OledSpectrum {
public:
    static const int W = 128;
    static const int SPEC_H = 24;
    static const int WF_H = 40;

    void push(const uint8_t* bins, uint16_t nbin) {
        // nbin -> 128 max-decimalas (a keskeny csucsok ne vesszenek el)
        uint8_t row[W];
        for (int x = 0; x < W; x++) {
            int a = (int)((uint32_t)x * nbin / W);
            int b = (int)((uint32_t)(x + 1) * nbin / W);
            if (b <= a) b = a + 1;
            uint8_t m = bins[a];
            for (int j = a + 1; j < b && j < nbin; j++) if (bins[j] > m) m = bins[j];
            row[x] = m;
        }
        // waterfall gorgetes lefele
        memmove(&wf[1][0], &wf[0][0], (WF_H - 1) * W);
        memcpy(&wf[0][0], row, W);
        memcpy(cur, row, W);
    }

    // Egy fix kuszob-ablakot ter kepez 0..255 -> 0..15 (dither-szint).
    // lo/hi: a megjelenitett dB-ablak a uint8 skalan (a kliens floor/range-hez kepest).
    void setWindow(uint8_t lo, uint8_t hi) { wlo = lo; whi = hi > lo ? hi : lo + 1; }

    template <typename U8G2>
    void draw(U8G2& g) {
        g.clearBuffer();
        // felso spektrumgorbe
        for (int x = 0; x < W; x++) {
            int v = level16(cur[x]);                // 0..15
            int h = (v * (SPEC_H - 1)) / 15;
            if (h > 0) g.drawVLine(x, SPEC_H - 1 - h, h + 1);
        }
        // also 1-bites ditherelt waterfall
        for (int y = 0; y < WF_H; y++) {
            const uint8_t* r = wf[y];
            for (int x = 0; x < W; x++) {
                int v = level16(r[x]);
                if (v > bayer4[y & 3][x & 3]) g.drawPixel(x, SPEC_H + y);
            }
        }
        g.sendBuffer();
    }

private:
    inline int level16(uint8_t v) const {
        if (v <= wlo) return 0;
        if (v >= whi) return 15;
        return (int)((uint32_t)(v - wlo) * 15 / (whi - wlo));
    }
    uint8_t wf[WF_H][W] = {};
    uint8_t cur[W] = {};
    uint8_t wlo = 20, whi = 200;
    const uint8_t bayer4[4][4] = { {0,8,2,10}, {12,4,14,6}, {3,11,1,9}, {15,7,13,5} };
};
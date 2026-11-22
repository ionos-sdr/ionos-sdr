#!/usr/bin/env python3
"""tftemu.py — ionos-sdr-mini TFT waterfall emulator, line-by-line port of
   firmware-esp32/streaming/src/iq_fft.cpp (process, push) + tft.cpp
   (push_specline, scale_px, palette, scroll).

   Input : complex I/Q (.npy complex64 from SpyServer, or stereo int16 .wav),
           50 ksps, the same samples iq_fft_push_block() gets.
   Output: PNG of the 240x270 scroll area (newest row on TOP, like the panel),
           optionally a GIF/strip of the whole run, plus the frequency scale.

   Every tunable defaults to the firmware default and can be overridden to try
   a fix BEFORE flashing:
     --base 0|1   --tau S   --dcw N(bins, -1 = firmware n/128)   --alpha A
     --lift dB    --range dB   --dec N   --n 256|512|1024   --win 0|1
     --at SECONDS (screen snapshot time, default: end of file)
     --strip      (draw every row of the file, not just the last 270)
"""
import sys, argparse, numpy as np, wave

W, H, HDR_H = 240, 320, 50
VSA = H - HDR_H                    # 270 scrolled rows

def load(fn):
    if fn.endswith('.npy'):
        z = np.load(fn).astype(np.complex64)
        return z * 32768.0, 50000   # watch.py scaled int16 by 1/32768
    w = wave.open(fn, 'rb'); fs = w.getframerate()
    d = np.frombuffer(w.readframes(w.getnframes()), '<i2').reshape(-1, 2).astype(np.float32)
    return d[:, 0] + 1j * d[:, 1], fs

def palette():
    pal = np.zeros((256, 3), np.uint8)
    for i in range(256):
        t = i / 255.0
        if   t < 0.15: u = t / 0.15;          r, g, b = 0, 0, int(110 * u)
        elif t < 0.35: u = (t - 0.15) / 0.20; r, g, b = 0, int(60 * u), int(110 + 145 * u)
        elif t < 0.55: u = (t - 0.35) / 0.20; r, g, b = 0, int(60 + 195 * u), 255
        elif t < 0.70: u = (t - 0.55) / 0.15; r, g, b = 0, 255, int(255 * (1 - u))
        elif t < 0.82: u = (t - 0.70) / 0.12; r, g, b = int(255 * u), 255, 0
        elif t < 0.93: u = (t - 0.82) / 0.11; r, g, b = 255, int(255 * (1 - u)), 0
        else:          u = (t - 0.93) / 0.07; r, g, b = 255, int(255 * u), int(255 * u)
        pal[i] = (r & 0xF8, g & 0xFC, b & 0xF8)          # RGB565 quantisation
    return pal

def run(z, fs, a):
    n = a.n
    x = np.arange(n) / (n - 1)
    if a.win:
        win = (0.355768 - 0.487396 * np.cos(2 * np.pi * x) + 0.144232 * np.cos(4 * np.pi * x)
               - 0.012604 * np.cos(6 * np.pi * x)).astype(np.float32)
    else:
        win = (0.5 - 0.5 * np.cos(2 * np.pi * x)).astype(np.float32)
    h = np.cumsum(1.0 / np.arange(1, 17)); bias = np.concatenate([[0.0], 10 * np.log10(h)])
    lo_k = (np.arange(240) * n) // 240; hi_k = (np.arange(1, 241) * n) // 240
    hi_k = np.maximum(hi_k, lo_k + 1)
    cnt = np.minimum(hi_k - lo_k, 16)

    pwr = None; base = None; floor = -60.0; c = 0
    acc = None
    rows = []; row_t = []
    nb = len(z) // n                       # contiguous collections (see header note)
    for bi in range(nb):
        seg = z[bi * n:(bi + 1) * n]
        # payload order in the firmware: re = s_buf[2i+1], im = s_buf[2i];
        # the stream is I-first, so re = Q, im = I  -> mirrored, undone by s_invert
        re = seg.imag * win; im = seg.real * win
        X = np.fft.fft(re + 1j * im)
        m = (X.real ** 2 + X.imag ** 2)
        m = np.roll(m, n // 2)            # s_pwr[k] = |X[(k + n/2) % n]|^2
        pwr = m if pwr is None else pwr * (1 - a.alpha) + m * a.alpha
        p = pwr.copy()
        cc = n // 2; w = a.dcw if a.dcw >= 0 else max(2, n // 128)
        if w > 0:
            l, r = p[cc - w - 1], p[cc + w + 1]
            d = np.arange(-w, w + 1); t = (d + w) / (2 * w)
            p[cc + d] = l * (1 - t) + r * t
            pwr[cc + d] = p[cc + d]       # firmware overwrites s_pwr in place
        db = 10 * np.log10(p + 1e-3)
        if a.base:
            if base is None: base = db.copy()
            else:
                ba = min(0.5, 1.0 / (49.0 * a.tau)); base += (db - base) * ba
            db = db - base
        floor = floor * 0.97 + db.mean() * 0.03
        # row combine across the dec FFTs: 'last' = firmware (only the latest EMA),
        # 'max' = peak-hold of the dec FFTs, 'mean' = power mean of them
        if a.rowmode == 'last': acc = db
        elif acc is None: acc = db.copy() if a.rowmode == 'max' else 10 ** (db / 10)
        elif a.rowmode == 'max': acc = np.maximum(acc, db)
        else: acc = acc + 10 ** (db / 10)
        c += 1
        if c < a.dec: continue
        c = 0
        rowdb = acc if a.rowmode != 'mean' else 10 * np.log10(acc / a.dec)
        acc = None
        if a.pix == 'max':
            best = np.array([rowdb[lo_k[i]:hi_k[i]].max() for i in range(240)]) - bias[cnt]
        else:   # power mean of the bins in the pixel
            best = np.array([10 * np.log10(np.mean(10 ** (rowdb[lo_k[i]:hi_k[i]] / 10))) for i in range(240)])
        tt = np.clip((best - floor - a.lift) / a.range, 0, 1) ** a.gamma
        tt = np.clip(tt * 255.0, 1, 255).astype(np.uint8)
        if a.invert: tt = tt[::-1]
        rows.append(tt)                   # tft autoscale OFF -> 1:1, push_specline 240->240
        row_t.append((bi + 1) * n / fs)
    return np.array(rows), np.array(row_t)

def render(rows, row_t, a, fs, fn):
    from PIL import Image, ImageDraw
    pal = palette()
    if a.strip:
        img = pal[rows[::-1]]             # whole run, newest on top
        tsel = row_t
    else:
        k = len(rows) if a.at is None else int(np.searchsorted(row_t, a.at))
        sel = rows[max(0, k - VSA):k][::-1]
        img = np.zeros((VSA, W, 3), np.uint8); img[:len(sel)] = pal[sel]
        tsel = row_t[max(0, k - VSA):k]
    hh = 24
    canvas = Image.new('RGB', (W, img.shape[0] + hh), (0, 0, 0))
    canvas.paste(Image.fromarray(img), (0, hh))
    dr = ImageDraw.Draw(canvas)
    cf = a.cf_khz
    for frac, lab in ((0.0, cf - fs / 2000), (0.5, cf), (1.0, cf + fs / 2000)):
        xx = int(frac * (W - 1))
        dr.line([(xx, hh - 6), (xx, hh - 1)], fill=(255, 255, 255))
        s = '%.3f' % (lab / 1000)
        tx = max(0, min(W - 6 * len(s), xx - 3 * len(s)))
        dr.text((tx, 2), s, fill=(255, 255, 255))
    if len(tsel): dr.text((2, hh + 2), 't=%.1f..%.1f s' % (tsel[0], tsel[-1]), fill=(255, 255, 255))
    canvas = canvas.resize((W * a.zoom, canvas.size[1] * a.zoom), Image.NEAREST)
    canvas.save(fn)

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('iq'); ap.add_argument('out')
    ap.add_argument('--cf_khz', type=float, default=144800)
    ap.add_argument('--n', type=int, default=1024)
    ap.add_argument('--win', type=int, default=1)
    ap.add_argument('--alpha', type=float, default=0.40)
    ap.add_argument('--dcw', type=int, default=-1)
    ap.add_argument('--base', type=int, default=1)
    ap.add_argument('--tau', type=float, default=20.0)
    ap.add_argument('--lift', type=float, default=-2.0)
    ap.add_argument('--range', type=float, default=40.0)
    ap.add_argument('--dec', type=int, default=3)
    ap.add_argument('--invert', type=int, default=1)
    ap.add_argument('--at', type=float, default=None)
    ap.add_argument('--strip', action='store_true')
    ap.add_argument('--zoom', type=int, default=2)
    ap.add_argument('--rowmode', default='last', choices=['last', 'max', 'mean'])
    ap.add_argument('--pix', default='max', choices=['max', 'mean'])
    ap.add_argument('--gamma', type=float, default=1.0)
    a = ap.parse_args()
    z, fs = load(a.iq)
    rows, rt = run(z, fs, a)
    render(rows, rt, a, fs, a.out)
    print('rows %d (%.1f s), wrote %s' % (len(rows), rt[-1] if len(rt) else 0, a.out))

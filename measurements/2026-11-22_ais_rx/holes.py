# holes.py capture.npy offset_hz [bw_hz]  — find ~1-block (5 ms) holes inside constant-envelope bursts
import sys, numpy as np, collections
from scipy.signal import firwin, lfilter
fn, off = sys.argv[1], float(sys.argv[2]); bw = float(sys.argv[3]) if len(sys.argv) > 3 else 6000
x = np.load(fn, mmap_mode='r'); fs = 50000
h = firwin(65, bw / (fs / 2))
CH = 50000 * 20
nb = 0; nh = 0; holes = []; spans = []
for c0 in range(0, len(x) - 1, CH):
    seg = np.asarray(x[c0:c0 + CH + 4000]); n = np.arange(len(seg)) + c0
    y = lfilter(h, 1, seg * np.exp(-2j * np.pi * off * n / fs))
    env = np.convolve(np.abs(y), np.ones(25) / 25, 'same')
    fl = np.median(env)
    on = env > fl * 4
    # bursts = on-runs merged over gaps < 400 samples (8 ms)
    idx = np.flatnonzero(on)
    if not len(idx): continue
    br = np.split(idx, np.flatnonzero(np.diff(idx) > 400) + 1)
    for b in br:
        if len(b) < 600 or b[0] >= CH: continue
        nb += 1
        s0, s1 = b[0], b[-1]; spans.append((s1 - s0) / fs * 1e3)
        lev = np.median(env[s0:s1])
        inside = env[s0:s1] < lev * 0.35
        d = np.diff(np.concatenate([[0], inside.astype(int), [0]]))
        L = np.flatnonzero(d == -1) - np.flatnonzero(d == 1)
        big = L[L >= 50]
        if len(big): nh += 1; holes += big.tolist()
print('%s: bursts %d, with >=1 ms hole %d (%.0f%%)' % (fn, nb, nh, 100.0 * nh / max(nb, 1)))
print('hole length hist (32-sample bins):', collections.Counter((np.array(holes) // 32 * 32).tolist()).most_common(8))
print('burst span ms (median):', np.median(spans) if spans else None)

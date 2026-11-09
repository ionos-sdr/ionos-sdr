#!/usr/bin/env python3
# Wideband RX characterisation of ionos-mini (434 MHz matched BRD4265B, FG23 PHY of the 145 MHz band)
# using the FG23 'B<kHz>' base override. Chain: VSG60 -> 30 dB pad -> 1.5 m coax -> board.
# Phase 1: where does RX work at all (1 MHz grid, 50..1100 MHz, then edges bisected to 25 kHz).
# Phase 2: CW gain / noise floor / MDS / NF estimate on a 250 kHz grid inside the working ranges.
# Phase 3: 2-FSK 5 kbps (+-2.5 kHz, PRBS9) sensitivity at BER 1e-2, 1 MHz grid (VSG arb).
import sys, os, time, numpy as np
sys.argv = [sys.argv[0], "0"]
exec(open("longprog.py").read().split('log("START')[0])      # reader thread, cmd, capture, spectrum, VSG, log, csvw
import fsk_lib
PAD = 30.5; ARB = os.path.abspath("fsk_prbs9_100k.binfc")
def B(khz):
    r = cmd("B%d" % khz, 1.0); time.sleep(0.4); return r
def rate(fs): return cmd("R %d" % fs, 2.0)
def tone_search(fs, f_exp, span, secs=1.0, N=1 << 15):
    time.sleep(0.15); z, rs = capture(secs); S = spectrum(z, N); f = (np.arange(N) - N // 2) * fs / N
    m = np.abs(f - f_exp) <= span; idx = np.where(m)[0]; k = idx[np.argmax(S[idx])]
    lvl = 10 * np.log10(S[k-4:k+5].sum() / 1.7 + 1e-20); nd = noise_dbfs_hz(S, fs, N)
    snr_bin = 10 * np.log10(S[k] / (np.median(S) + 1e-30) + 1e-30)
    return lvl, f[k], nd, snr_bin, (np.mean(rs) if rs else float("nan"))
def works(v, fhz, lvl_vsg=-60):
    B(int(fhz // 1000)); v.freq(fhz + 20e3, verify=False); v.power(lvl_vsg); time.sleep(1.6)
    lvl, fo, nd, snr, rssi = tone_search(200000, 20e3, 60e3)
    return snr > 25, (lvl, fo - 20e3, nd, snr, rssi)
log("WIDEBAND start; pad %.1f dB" % PAD)
log("MODE USB %s, R %s" % (cmd("MODE USB"), rate(200000)))
z = fsk_lib.gen(100000, 40); fsk_lib.write_binfc(ARB, z)
with VSG(port=5124) as v:
    v.modulation(False); v.on()
    # ---------- phase 1: coarse map ----------
    grid = np.arange(50e6, 1100e6 + 1, 1e6); ok = []
    for fh in grid:
        w, m = works(v, fh); ok.append(w)
        csvw("wb_map", [int(fh), int(w), "%.2f" % m[0], "%.0f" % m[1], "%.2f" % m[2], "%.1f" % m[3], "%.1f" % m[4]])
    ok = np.array(ok); log("phase 1: %d of %d MHz points receive" % (ok.sum(), len(ok)))
    edges = [i for i in range(1, len(ok)) if ok[i] != ok[i-1]]
    ranges = []
    for i in edges:                                   # bisect each transition to 25 kHz
        lo, hi = grid[i-1], grid[i]; wlo = ok[i-1]
        while hi - lo > 25e3:
            mid = round((lo + hi) / 2 / 1e3) * 1e3; w, m = works(v, mid)
            csvw("wb_edge", [int(mid), int(w), "%.2f" % m[0], "%.0f" % m[1], "%.1f" % m[3]])
            if w == wlo: lo = mid
            else: hi = mid
        log("edge %s at %.3f..%.3f MHz" % ("upper" if wlo else "lower", lo / 1e6, hi / 1e6))
    # working ranges from the coarse map
    i = 0
    while i < len(ok):
        if ok[i]:
            j = i
            while j + 1 < len(ok) and ok[j + 1]: j += 1
            ranges.append((grid[i], grid[j])); i = j + 1
        else: i += 1
    log("working ranges (1 MHz map): %s" % ", ".join("%.0f-%.0f MHz" % (a / 1e6, b / 1e6) for a, b in ranges))
    # ---------- phase 2: CW gain / noise / MDS on a 250 kHz grid, 50 ksps ----------
    rate(50000)
    for a, b in ranges:
        for fh in np.arange(a, b + 1, 250e3):
            B(int(fh // 1000)); v.freq(fh + 5e3, verify=False); v.power(-70); time.sleep(1.6)
            lvl, fo, nd, snr, rssi = tone_search(50000, 5e3, 20e3, secs=1.5, N=1 << 16)
            pin = -70 - PAD
            nd_dbm = pin - (lvl - nd)                         # input-referred noise density, dBm/Hz
            mds = nd_dbm + 10 * np.log10(500) + 10            # 10 dB SNR in 500 Hz
            csvw("wb_cw", [int(fh), "%.1f" % pin, "%.2f" % lvl, "%.0f" % (fo - 5e3), "%.2f" % nd,
                           "%.1f" % nd_dbm, "%.1f" % (nd_dbm + 174), "%.1f" % mds, "%.1f" % rssi])
    log("phase 2 done")
    # ---------- phase 3: FSK sensitivity, BER 1e-2, 1 MHz grid ----------
    okl, msg = v.load_arb(ARB, srate_hz=100000)
    log("arb load: %s %s" % (okl, msg))
    if okl:
        for a, b in ranges:
            for fh in np.arange(a, b + 1, 1e6):
                B(int(fh // 1000)); v.freq(fh, verify=False); time.sleep(1.0)
                lo, hi = -140.0, -80.0; pts = []
                for _ in range(7):
                    mid = (lo + hi) / 2; v.power(mid + PAD); time.sleep(1.6)
                    zz, _ = capture(1.2); e, nb = fsk_lib.ber(zz, 50000); pts.append((mid, e, nb))
                    if e > 1e-2: lo = mid
                    else: hi = mid
                csvw("wb_fsk", [int(fh), "%.1f" % hi] + ["%.1f:%.4f" % (p[0], p[1]) for p in pts])
        v.modulation(False)
    log("phase 3 done")
    v.off()
cmd("B0", 1.0); cmd("F%d" % 145000, 1.5); log("WIDEBAND end")
h.close()

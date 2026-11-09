#!/usr/bin/env python3
# Follow-up to the wideband run (2026-11-09 night): (A) FSK outliers again, 3x each, with neighbours;
# (B) are the "bad NF" CW points external signals? noise spectra with the VSG OFF; (C) the +-6.25 kHz
# side lines next to a CW tone - level dependence, offset dependence, VSG off. All through SoapyIonos.
import sys, os, time, numpy as np, serial
from vsg60 import VSG
from soapyio import *
import soapyio
import fsk_lib
PAD = 30.5; ARB = os.path.abspath("fsk_prbs9_100k.binfc")
def B(khz): r = cmd("B%d" % khz, 1.0); time.sleep(0.4); return r
def fg23_reset():
    c = serial.Serial(); c.port = "COM4"; c.baudrate = 115200; c.timeout = 0.2; c.dtr = False; c.rts = False
    c.open(); c.write(b"FGRST\n"); time.sleep(1.5); c.write(b"FGINFO\n"); time.sleep(1.5); c.close(); time.sleep(5.0)
def cap(sec):
    z, _ = capture(sec)
    if len(z) < 4096: fg23_reset(); set_rate(soapyio._fs[0]); return None
    return z
def fsk_level(v, fh):
    B(int(fh // 1000)); v.freq(fh, verify=False); time.sleep(1.0)
    lo, hi = -140.0, -70.0; pts = []
    for _ in range(8):
        mid = (lo + hi) / 2; v.power(mid + PAD); time.sleep(1.6)
        z = cap(1.2); e = fsk_lib.ber(z, 50000)[0] if z is not None and len(z) > 20000 else 0.5
        pts.append("%.1f:%.4f" % (mid, e))
        if e > 1e-2: lo = mid
        else: hi = mid
    return hi, pts
log("RECHECK start")
with VSG(port=5124) as v:
    # ---------- (B) noise spectra, VSG output OFF ----------
    v.off(); set_rate(200000)
    for fh in [] and [129e6, 133.75e6, 136.5e6, 144.25e6, 145e6, 152.25e6, 156e6, 160e6, 160.25e6, 163.75e6,
               169e6, 172.75e6, 175.5e6, 183.25e6, 191.25e6, 195e6]:
        B(int(fh // 1000)); time.sleep(1.5); z = cap(2.0)
        if z is None: csvw("rc_noise", [int(fh), "nodata"]); continue
        N = 1 << 14; S = spectrum(z, N); f = (np.arange(N) - N // 2) * 200000 / N; Sd = 10 * np.log10(S + 1e-20)
        med = np.median(Sd); order = np.argsort(Sd)[::-1]; peaks = []
        for k in order:
            if abs(f[k]) < 300: continue
            if all(abs(f[k] - p[0]) > 2000 for p in peaks): peaks.append((f[k], Sd[k] - med))
            if len(peaks) >= 5: break
        csvw("rc_noise", [int(fh), "%.2f" % noise_dbfs_hz(S, 200000, N)] + ["%+.0f:%.1f" % p for p in peaks])
    log("B done (noise spectra, VSG off)")
    # ---------- (C) +-6.25 kHz side lines at 145 MHz ----------
    B(145000); set_rate(200000); v.modulation(False); v.on()
    for lvl in (-40, -50, -60, -70, -80):
        for off in (5e3, 20e3, 37e3):
            v.freq(145e6 + off, verify=False); v.power(lvl); time.sleep(1.8)
            z = cap(2.0)
            if z is None: continue
            N = 1 << 16; S = spectrum(z, N); f = (np.arange(N) - N // 2) * 200000 / N
            k0 = np.argmax(S); t = 10 * np.log10(S[k0]); fo = f[k0]
            def at(df):
                m = np.abs(f - (fo + df)) < 150
                return 10 * np.log10(S[m].max()) - t if m.any() else float('nan')
            csvw("rc_side", [lvl - PAD, int(off), "%.0f" % fo, "%.2f" % t] +
                 ["%+.0f:%.1f" % (d, at(d)) for d in (-12500, -6250, -3125, 3125, 6250, 12500)])
    log("C done (side lines)")
    # ---------- (A) FSK outliers again ----------
    okl, msg = v.load_arb(ARB, srate_hz=100000); log("arb load: %s %s" % (okl, msg))
    if okl:
        set_rate(50000)
        for fh in [129e6, 145e6, 155e6, 156e6, 157e6, 159e6, 160e6, 161e6, 168e6, 169e6, 170e6, 194e6, 195e6, 196e6]:
            for rep in range(3):
                hi, pts = fsk_level(v, fh); csvw("rc_fsk", [int(fh), rep, "%.1f" % hi] + pts)
        v.modulation(False)
    log("A done (FSK)")
    v.off(); v.modulation(True)
cmd("B0", 1.0); cmd("F145000", 1.5); log("RECHECK end; sensors %s" % sensors())
dev.deactivateStream(st); dev.closeStream(st)

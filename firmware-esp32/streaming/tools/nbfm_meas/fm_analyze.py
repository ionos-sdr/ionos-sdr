#!/usr/bin/env python3
"""fm_analyze.py - NBFM TX characterisation from RTL-SDR .cu8 captures (pb_meas.py).

RF: PSD, carrier offset, close-in spurs (dBc, RTL's own spurs marked from
'off'), occupied bandwidth, adjacent-channel power (12.5 kHz raster).
Audio: FM discriminator -> deviation, tone level/THD/SINAD, CTCSS; for the
chain test (pmr_tx --test-signal) frequency response with 750 us de-emphasis,
THD per tone, idle-channel noise, two-tone IMD, clicks.
"""
import json, os, sys
import numpy as np
from scipy import signal

D = sys.argv[1] if len(sys.argv) > 1 else '.'
OUTD = sys.argv[2] if len(sys.argv) > 2 else D
OFF = 250000.0


def load(name):
    m = json.load(open(os.path.join(D, name + '.json')))
    b = np.fromfile(os.path.join(D, name + '.cu8'), np.uint8).astype(np.float32) - 127.4
    x = (b[0::2] + 1j * b[1::2]) / 128.0
    x = x[int(0.15 * m['fs']):]                        # rtl_sdr start-up transient
    return x - np.mean(x), m


def psd(x, fs, nfft=65536):
    f, p = signal.welch(x, fs, window='blackmanharris', nperseg=nfft, return_onesided=False, scaling='spectrum')
    return np.fft.fftshift(f), np.fft.fftshift(p)


def carrier(f, p, fexp, span=20000):
    m = np.abs(f - fexp) < span
    i = np.argmax(np.where(m, p, 0))
    return f[i], p[i]


def to_bb(x, fs, fc, out_fs=32000):
    """Shift fc to 0 Hz, low-pass 12 kHz, resample to out_fs."""
    n = np.arange(len(x))
    y = x * np.exp(-2j * np.pi * fc / fs * n)
    g = np.gcd(int(fs), out_fs)
    return signal.resample_poly(y, out_fs // g, int(fs) // g, window=('kaiser', 8.0))


def fm_demod(z, fs):
    ph = np.unwrap(np.angle(z))
    return np.diff(ph) * fs / (2 * np.pi)              # Hz


def deemph(a, fs, tau=750e-6):
    al = np.exp(-1 / (tau * fs))
    return signal.lfilter([1 - al], [1, -al], a)


def tone_metrics(a, fs, f0):
    """Fit f0 (+DC), THD from harmonics 2..5, THD+N (300-3400 Hz)."""
    a = a - np.mean(a)
    w = signal.windows.blackmanharris(len(a))
    sp = np.abs(np.fft.rfft(a * w)) ** 2
    fr = np.fft.rfftfreq(len(a), 1 / fs)
    def pw(fc, bw=15):
        m = np.abs(fr - fc) < bw
        return sp[m].sum()
    p1 = pw(f0)
    h = sum(pw(k * f0) for k in range(2, 6) if k * f0 < 3400)
    band = (fr > 300) & (fr < 3400)
    tot = sp[band].sum()
    amp = np.sqrt(2) * np.std(a) if p1 == 0 else None
    pk = np.sqrt(2 * p1 / (np.sum(w ** 2))) / np.sqrt(len(a)) * np.sqrt(len(a))  # unused (kept simple)
    return dict(thd_pct=100 * np.sqrt(h / p1), thdn_pct=100 * np.sqrt(max(tot - p1, 1e-30) / p1),
                sinad_db=10 * np.log10(p1 / max(tot - p1, 1e-30)))


def tone_amp(a, fs, f0):
    t = np.arange(len(a)) / fs
    M = np.column_stack([np.cos(2 * np.pi * f0 * t), np.sin(2 * np.pi * f0 * t), np.ones_like(t)])
    c, *_ = np.linalg.lstsq(M, a, rcond=None)
    return float(np.hypot(c[0], c[1]))


def rf_report(name, ref=None):
    x, m = load(name)
    fs = m['fs']
    fexp = m['f0'] - m['fc']
    f, p = psd(x, fs)
    fcar, pcar = carrier(f, p, fexp)
    pdb = 10 * np.log10(p / pcar + 1e-30)
    res = dict(name=name, fs=fs, gain=m['gain'], carrier_offset_hz=float(fcar - fexp),
               carrier_dbfs=float(10 * np.log10(pcar + 1e-30)))
    # occupied bandwidth 99 %
    m50 = np.abs(f - fcar) < 50000
    pp = p[m50]; ff = f[m50]; c = np.cumsum(pp) / pp.sum()
    res['obw99_hz'] = float(ff[np.searchsorted(c, 0.995)] - ff[np.searchsorted(c, 0.005)])
    # adjacent channel: main +-6.25 kHz vs. +-12.5 kHz channels (8.5 kHz wide windows)
    def bp(lo, hi):
        mm = (f - fcar >= lo) & (f - fcar < hi)
        return p[mm].sum()
    main = bp(-4250, 4250)
    res['acp_lo_dbc'] = float(10 * np.log10(bp(-12500 - 4250, -12500 + 4250) / main))
    res['acp_hi_dbc'] = float(10 * np.log10(bp(12500 - 4250, 12500 + 4250) / main))
    # spurs: peaks > floor+10 dB, outside +-15 kHz of the carrier
    floor = np.median(pdb)
    pk, _ = signal.find_peaks(pdb, height=floor + 12, distance=50)
    sp = [(float(f[i] - fcar), float(pdb[i])) for i in pk if abs(f[i] - fcar) > 15000]
    rf_own = set()
    if ref is not None:
        rf_own = {round(v / 2000) for v in ref}
    res['spurs'] = sorted([dict(offset_hz=o, dbc=d, rtl_own=(round((o + fcar) / 2000) in rf_own)) for o, d in sp],
                          key=lambda s: -s['dbc'])[:15]
    res['floor_dbc'] = float(floor)
    return res, (f, pdb, fcar), x, m


def spur_freqs(name):
    x, m = load(name)
    f, p = psd(x, m['fs'])
    pdb = 10 * np.log10(p + 1e-30)
    pk, _ = signal.find_peaks(pdb, height=np.median(pdb) + 12, distance=50)
    return [float(f[i]) for i in pk]


def audio_from(x, m, fcar):
    z = to_bb(x, m['fs'], fcar)
    a = fm_demod(z, 32000)
    a = a - np.median(a)
    return signal.resample_poly(a, 1, 4), 8000           # 8 kHz audio in Hz of deviation


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = [n[:-5] for n in sorted(os.listdir(D)) if n.endswith('.json')]
    own = spur_freqs('off') if 'off' in names else None
    out = {}
    for n in names:
        if n.startswith('off'):
            continue
        r, (f, pdb, fcar), x, m = rf_report(n, own)
        if not m.get('chain') and m['fs'] <= 1100000:
            a, afs = audio_from(x, m, fcar)
            dev = a[afs // 2:]
            r['dev_peak_hz'] = float(np.percentile(np.abs(dev), 99.9))
            r['dev_rms_hz'] = float(np.std(dev))
            if 'Nd0' not in ' '.join(m.get('cmds', [])):
                r.update(tone_metrics(dev[:afs * 2], afs, 1000))
                r['tone_1k_dev_hz'] = tone_amp(dev[:afs * 2], afs, 1000)
                if any(c.startswith('Nc') and c != 'Nc0' for c in m.get('cmds', [])):
                    lo = signal.sosfilt(signal.butter(4, 250, 'low', fs=afs, output='sos'), dev)
                    sp = np.abs(np.fft.rfft(lo[afs:] * np.hanning(len(lo) - afs)))
                    fr = np.fft.rfftfreq(len(lo) - afs, 1 / afs)
                    r['ctcss_hz'] = float(fr[np.argmax(sp * (fr > 50))])
                    r['ctcss_dev_hz'] = tone_amp(lo[afs:], afs, r['ctcss_hz'])
        out[n] = r
        fig, ax = plt.subplots(figsize=(9, 3.2))
        ax.plot((f - fcar) / 1e3, pdb, lw=0.6)
        ax.set_xlabel('offset from carrier (kHz)'); ax.set_ylabel('dBc'); ax.set_ylim(-110, 5)
        ax.set_title('%s  (RTL %.3f MHz, %.2f Msps, gain %s)' % (n, m['fc'] / 1e6, m['fs'] / 1e6, m['gain']))
        ax.grid(alpha=.3); fig.tight_layout(); fig.savefig(os.path.join(OUTD, 'psd_%s.png' % n), dpi=130); plt.close(fig)
    json.dump(out, open(os.path.join(OUTD, 'summary.json'), 'w'), indent=1)
    for n, r in out.items():
        print(n, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items() if k != 'spurs'})
        for s in r['spurs'][:6]:
            print('    spur %+9.0f Hz %6.1f dBc%s' % (s['offset_hz'], s['dbc'], ' (RTL own)' if s['rtl_own'] else ''))

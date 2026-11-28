#!/usr/bin/env python3
"""chain_analyze.py - end-to-end audio of the NBFM chain (pmr_tx --test-signal):
PC DSP -> int8 8 kHz -> ESP -> cmdlink -> FG23 NCO -> RF -> RTL -> FM demod -> 750 us de-emphasis."""
import json, os, sys
import numpy as np
from scipy import signal
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fm_analyze as fa

D, OUTD = sys.argv[1], sys.argv[2]
STEPS = [200, 300, 400, 500, 700, 1000, 1500, 2000, 2500, 3000, 3400, 4000]


def analyse(name):
    x, m = fa.load(name)
    fs = m['fs']
    f, p = fa.psd(x[:fs * 4], fs)
    fcar = fa.carrier(f, p, m['f0'] - m['fc'], 5000)[0]
    # refine the carrier from the mean of the demodulated frequency over the silent part
    z = fa.to_bb(x, fs, fcar)
    a32 = fa.fm_demod(z, 32000)
    # TX interval: where the carrier is there (|z| well above the no-signal noise)
    mag = np.convolve(np.abs(z[1:]), np.ones(320) / 320, 'same')
    on = mag > 0.3 * np.percentile(mag, 95)
    i0, i1 = np.argmax(on), len(on) - np.argmax(on[::-1])
    a32 -= np.median(a32[i0:i1])
    g = slice(i0 + 1600, i1 - 1600)                              # 50 ms margins
    clicks = int(np.sum(np.abs(a32[g]) > 6000))                  # phase jumps / dropouts inside the TX
    a32[:i0] = 0; a32[i1:] = 0
    a = signal.resample_poly(a32, 1, 4)                          # 8 kHz, Hz of deviation
    tx0, tx1 = i0 / 32000, i1 / 32000
    afs = 8000
    lo = signal.sosfiltfilt(signal.butter(4, 250, 'low', fs=afs, output='sos'), a)
    hp = signal.sosfiltfilt(signal.butter(6, 280, 'high', fs=afs, output='sos'), a)
    ad = fa.deemph(hp, afs)
    ad *= 1 / abs(np.exp(0))                                     # level re 1 kHz handled below
    # alignment: onset of the 1 kHz -10 dBFS segment (programme t = 2 s)
    b1k = signal.sosfiltfilt(signal.butter(4, [900, 1100], 'band', fs=afs, output='sos'), hp)
    env = np.convolve(np.abs(b1k), np.ones(80) / 80, 'same')
    on = np.argmax(env > 0.5 * np.percentile(env, 99))
    t0 = on / afs - 2.0
    seg = lambda s, e, sig=ad: sig[int((t0 + s) * afs):int((t0 + e) * afs)]
    r = dict(name=name, t0=round(t0, 3), tx_s=[round(tx0, 2), round(tx1, 2)], clicks_over_6k=clicks)
    assert t0 + 0.3 > tx0, 'programme starts before the TX: %.2f %.2f' % (t0, tx0)
    # de-emphasis gain at 1 kHz, to quote levels as deviation at 1 kHz
    al = np.exp(-1 / (750e-6 * afs)); w = 2 * np.pi * 1000 / afs
    g1k = abs((1 - al) / (1 - al * np.exp(-1j * w)))
    ref = seg(2.5, 4.5)
    r['ref_1k_dev_hz_raw'] = fa.tone_amp(seg(2.5, 4.5, hp), afs, 1000)
    r.update({('ref_' + k): v for k, v in fa.tone_metrics(ref, afs, 1000).items()})
    resp, thd = {}, {}
    for i, fq in enumerate(STEPS):
        s = seg(5 + i + 0.2, 5 + i + 0.85)
        resp[fq] = fa.tone_amp(s, afs, fq)
        if fq <= 2000:
            thd[fq] = fa.tone_metrics(s, afs, fq)['thd_pct']
    r1 = resp[1000]
    r['response_db'] = {k: round(20 * np.log10(max(v, 1e-9) / r1), 2) for k, v in resp.items()}
    r['thd_pct_steps'] = {k: round(float(v), 3) for k, v in thd.items()}
    r['step_1k_dev_hz_raw'] = fa.tone_amp(seg(5 + 5 + 0.2, 5 + 5 + 0.85, hp), afs, 1000)
    # two-tone 700 + 1900
    tt = seg(17.3, 19.7)
    a7, a19 = fa.tone_amp(tt, afs, 700), fa.tone_amp(tt, afs, 1900)
    im = {fq: fa.tone_amp(tt, afs, fq) for fq in (1200, 2600, 500, 3100)}
    r['imd_db'] = {k: round(20 * np.log10(v / max(a7, a19)), 1) for k, v in im.items()}
    # idle channel noise (300-3000 Hz after de-emphasis) vs the 1 kHz reference and vs full +-2.5 kHz
    bp = signal.sosfiltfilt(signal.butter(6, [300, 3000], 'band', fs=afs, output='sos'), ad)
    nz = np.concatenate([seg(0.3, 1.7, bp), seg(20.3, 22.7, bp)])
    n_rms = float(np.std(nz))
    full = 2500 / np.sqrt(2) * g1k
    r['idle_noise_rms_hz_deemph'] = n_rms
    r['snr_re_ref_db'] = 20 * np.log10(fa.tone_amp(ref, afs, 1000) / np.sqrt(2) / n_rms)
    r['snr_re_full_dev_db'] = 20 * np.log10(full / n_rms)
    # CTCSS
    if '--ctcss' in m.get('extra', []):
        lt = lo[int((tx0 + 0.3) * afs):int((tx1 - 0.3) * afs)]
        sp = np.abs(np.fft.rfft(lt * np.hanning(len(lt)))); fr = np.fft.rfftfreq(len(lt), 1 / afs)
        fc = float(fr[np.argmax(sp * (fr > 50) * (fr < 260))])
        r['ctcss_hz'] = fc; r['ctcss_dev_hz'] = fa.tone_amp(lt, afs, fc)
    return r, (afs, ad, hp, t0)


if __name__ == '__main__':
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    res = {}
    for n in [a for a in ('chain', 'chain_ct', 'chain8') if os.path.exists(os.path.join(D, a + '.json'))]:
        r, (afs, ad, hp, t0) = analyse(n)
        res[n] = r
        print(json.dumps(r, indent=1, default=float))
        fig, ax = plt.subplots(2, 1, figsize=(9, 5))
        t = np.arange(len(ad)) / afs - t0
        ax[0].plot(t, hp, lw=.4); ax[0].set_xlim(-1, 24); ax[0].set_ylabel('deviation (Hz)'); ax[0].set_title(n + ': demodulated (before de-emphasis)')
        k = sorted(r['response_db']); ax[1].semilogx(k, [r['response_db'][q] for q in k], 'o-')
        ax[1].set_ylabel('dB re 1 kHz'); ax[1].set_xlabel('Hz'); ax[1].grid(alpha=.3); ax[1].set_title('frequency response after 750 us de-emphasis')
        fig.tight_layout(); fig.savefig(os.path.join(OUTD, 'chain_%s.png' % n), dpi=130); plt.close(fig)
    json.dump(res, open(os.path.join(OUTD, 'chain_summary.json'), 'w'), indent=1, default=float)

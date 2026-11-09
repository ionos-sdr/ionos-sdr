#!/usr/bin/env python3
# 2-FSK test signal (PRBS9, 5 kbps, +-2.5 kHz, CPFSK) for the VSG60 arb, and a noncoherent
# discriminator demodulator + PRBS-aligned BER counter for the board's I/Q (any fs that is a
# multiple of the bit rate).
import numpy as np
BITRATE = 5000; DEV = 2500.0
def prbs9():
    s = 0x1FF; out = []
    for _ in range(511):
        b = ((s >> 8) ^ (s >> 4)) & 1; out.append(s & 1); s = ((s << 1) | b) & 0x1FF
    return np.array(out, dtype=np.int8)
REF = prbs9()
def gen(srate=100000, reps=40):
    sps = srate // BITRATE; bits = np.tile(REF, reps)
    f = np.repeat(np.where(bits > 0, DEV, -DEV), sps)
    ph = 2 * np.pi * np.cumsum(f) / srate
    return (0.7 * np.exp(1j * ph)).astype(np.complex64)
def write_binfc(path, z):
    np.asarray(z, dtype=np.complex64).tofile(path)
def ber(z, fs):
    sps = int(round(fs / BITRATE))
    d = np.angle(z[1:] * np.conj(z[:-1])) * fs / (2 * np.pi)      # instantaneous frequency, Hz
    d = d - np.median(d)                                           # carrier offset (balanced PRBS)
    n = (len(d) // sps - 1) * sps
    best = None
    for ph in range(sps):
        s = d[ph:ph + n].reshape(-1, sps).sum(axis=1)
        q = np.mean(np.abs(s))
        if best is None or q > best[0]: best = (q, s)
    bits = (best[1] > 0).astype(np.int8)
    if len(bits) < 600: return 0.5, 0
    r = 2 * REF.astype(np.float64) - 1; b = 2 * bits[:511 * (len(bits) // 511)].astype(np.float64) - 1
    seg = b[:511]
    c = np.real(np.fft.ifft(np.fft.fft(seg) * np.conj(np.fft.fft(r))))
    k = int(np.argmax(np.abs(c))); pol = 1 if c[k] > 0 else -1
    ref = np.roll(REF, k); nb = len(b) // 511 * 511
    refl = np.tile(ref, nb // 511)
    rx = bits[:nb] if pol > 0 else 1 - bits[:nb]
    err = int(np.sum(rx != refl)); return err / nb, nb
if __name__ == "__main__":
    # self-test: CPFSK through AWGN at the board rate, demod back
    rng = np.random.default_rng(1)
    tx = gen(50000, 12)
    tx = tx * np.exp(2j * np.pi * 3170 * np.arange(len(tx)) / 50000)   # carrier offset
    for snr in (20, 10, 6, 3):
        p = np.mean(np.abs(tx) ** 2); nz = np.sqrt(p / 10 ** (snr / 10) / 2) * (rng.standard_normal(len(tx)) + 1j * rng.standard_normal(len(tx)))
        e, n = ber(tx[123:] + nz[123:], 50000); print("Es/N0-ish %2d dB (50 kHz BW): BER %.4f over %d bits" % (snr, e, n))

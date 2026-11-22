#!/usr/bin/env python3
"""ais_ref.py — independent float AIS reference demodulator (the "refdemod" of AIS).

    Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT

  Decides whether the AIS bursts in a capture are decodable at all, so the
  fixed-point firmware demod can be measured against it (same method as
  refdemod.py for APRS).

  Chain: channel mix -> LPF -> FM discriminator -> resample to 5 sps (48 kHz)
         -> Gaussian matched LPF -> slicer (moving-mean threshold)
         -> ALL 5 sample phases -> NRZI -> HDLC (destuff, CRC-16/X.25)
         -> bytes bit-reversed -> 6-bit armour -> !AIVDM + msg 1/2/3/5/18/21 decode.

  usage: ais_ref.py capture.npy|.wav --tune_khz 161985 [--fs 50000] [--chan A|B|both]
"""
import sys, argparse, numpy as np, wave
from fractions import Fraction
from scipy.signal import firwin, lfilter, resample_poly

CH = {'A': 161975.0, 'B': 162025.0}           # 87B, 88B [kHz]
BAUD = 9600; SPS = 5; AR = BAUD * SPS          # 48 kHz

def load(fn, fs_over=None):
    if fn.endswith('.npy'):
        return np.load(fn).astype(np.complex64), (fs_over or 50000)
    w = wave.open(fn, 'rb'); fs = w.getframerate()
    d = np.frombuffer(w.readframes(w.getnframes()), '<i2').reshape(-1, 2).astype(np.float32)
    return (d[:, 0] + 1j * d[:, 1]) / 32768.0, (fs_over or fs)

def crc_ok(fr):
    crc = 0xFFFF
    for b in fr:
        b ^= crc & 0xFF; b = (b ^ ((b << 4) & 0xFF)) & 0xFF
        crc = (((b << 8) | (crc >> 8)) ^ (b >> 4) ^ ((b << 3) & 0xFFFF)) & 0xFFFF
    return crc == 0xF0B8

def hdlc(bits, idx):
    """bits = NRZI-decoded (1 = no transition). Returns [(bit_index, frame_bytes)]."""
    win = 0; byte = 0; nbit = 0; inf = False; fr = bytearray(); out = []
    for i, bit in enumerate(bits):
        win = ((win << 1) | int(bit)) & 0xFF
        if win == 0x7E:
            if inf and len(fr) >= 11 and crc_ok(fr):
                out.append((idx[i], bytes(fr)))
            inf = True; fr = bytearray(); byte = 0; nbit = 0; continue
        if (win & 0xFE) == 0xFE:
            inf = False; fr = bytearray(); nbit = 0; continue
        if not inf: continue
        if (win & 0x3F) == 0x3E: continue          # stuffed zero
        if win & 1: byte |= 0x80
        nbit += 1
        if nbit >= 8:
            if len(fr) < 140: fr.append(byte)
            else: inf = False; fr = bytearray()
            byte = 0; nbit = 0
        else:
            byte >>= 1
    return out

def rev8(b):
    return int('{:08b}'.format(b)[::-1], 2)

def payload_bits(fr):
    """frame without FCS -> message bitstring, MSB-first per AIS"""
    # NO per-byte reversal (real Danube traffic 2026-11-22: KISFALUDY, AtoN 1645.3_G)
    return ''.join('{:08b}'.format(b) for b in fr[:-2])

def armour(bits):
    pad = (-len(bits)) % 6; bits += '0' * pad
    s = ''
    for k in range(0, len(bits), 6):
        v = int(bits[k:k + 6], 2); v += 48
        if v > 87: v += 8
        s += chr(v)
    return s, pad

def nmea(bits, chan):
    p, pad = armour(bits)
    body = 'AIVDM,1,1,,%s,%s,%d' % (chan, p, pad)
    cs = 0
    for c in body: cs ^= ord(c)
    return '!%s*%02X' % (body, cs)

def u(b, a, n): return int(b[a:a + n], 2)
def s(b, a, n):
    v = int(b[a:a + n], 2)
    return v - (1 << n) if v & (1 << (n - 1)) else v

def txt(b, a, n):
    t = ''
    for k in range(a, a + n, 6):
        v = u(b, k, 6); t += chr(v + 64 if v < 32 else v)
    return t.replace('@', '').strip()

def describe(b):
    t = u(b, 0, 6); mmsi = u(b, 8, 30)
    if t in (1, 2, 3) and len(b) >= 168:
        lon = s(b, 61, 28) / 600000.0; lat = s(b, 89, 27) / 600000.0
        sog = u(b, 50, 10) / 10.0; cog = u(b, 116, 12) / 10.0
        return 'type %d MMSI %09d  %.5f N %.5f E  SOG %.1f kn COG %.1f' % (t, mmsi, lat, lon, sog, cog)
    if t == 18 and len(b) >= 168:
        lon = s(b, 57, 28) / 600000.0; lat = s(b, 85, 27) / 600000.0
        return 'type 18 MMSI %09d  %.5f N %.5f E  SOG %.1f kn' % (mmsi, lat, lon, u(b, 46, 10) / 10.0)
    if t == 5 and len(b) >= 424:
        return 'type 5 MMSI %09d  name "%s" call "%s" dest "%s"' % (mmsi, txt(b, 112, 120), txt(b, 70, 42), txt(b, 302, 120))
    if t == 24 and len(b) >= 160:
        part = u(b, 38, 2)
        return 'type 24%s MMSI %09d  %s' % ('AB'[part] if part < 2 else '?', mmsi,
                                          ('name "%s"' % txt(b, 40, 120)) if part == 0 else '')
    if t == 21 and len(b) >= 272:
        return 'type 21 (AtoN) MMSI %09d  "%s"' % (mmsi, txt(b, 43, 120))
    if t == 4: return 'type 4 (base station) MMSI %09d' % mmsi
    return 'type %d MMSI %09d (%d bits)' % (t, mmsi, len(b))

def demod_channel(z, fs, off_hz, name):
    n = np.arange(len(z))
    y = z * np.exp(-2j * np.pi * off_hz * n / fs).astype(np.complex64)
    # Channel LPF 6 kHz. Synthetic sweep (20 bursts/point, SNR in 14 kHz):
    #   fc 5k: 19/11/2   fc 6k: 20/17/9   fc 8k: 14/0/0   @ 15/12/10 dB
    # A Gaussian post-detection LPF (BT 0.3..0.5) made it WORSE (ISI), so none.
    y = lfilter(firwin(65, 6000 / (fs / 2)), 1.0, y)
    d = np.angle(y[1:] * np.conj(y[:-1]))                       # rad/sample
    fr = Fraction(AR, int(round(fs))).limit_denominator(1000)
    d = resample_poly(d, fr.numerator, fr.denominator)
    # Slicer threshold = 64-bit moving mean (removes the carrier offset).
    # 16 bits was far too short: NRZI runs bias it (8/20 at 20 dB vs 20/20).
    thr = np.convolve(d, np.ones(64 * SPS) / (64 * SPS), 'same')
    soft = d - thr
    seen = {}
    for ph in range(SPS):
        idx = np.arange(ph, len(soft), SPS)
        b = soft[idx] > 0
        nrzi = ~(b[1:] ^ b[:-1])
        for bi, f in hdlc(nrzi, idx[1:]):
            key = f
            tsec = bi / AR
            if key not in seen or tsec < seen[key][0]: seen[key] = (tsec, ph)
    res = sorted((v[0], k, v[1]) for k, v in seen.items())
    return res

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('iq'); ap.add_argument('--tune_khz', type=float, required=True)
    ap.add_argument('--fs', type=int, default=None); ap.add_argument('--chan', default='both')
    a = ap.parse_args()
    z, fs = load(a.iq, a.fs)
    print('# %s: %.1f s @ %d sps, tuned %.3f MHz' % (a.iq, len(z) / fs, fs, a.tune_khz / 1000))
    total = 0
    for cn in (('A', 'B') if a.chan == 'both' else (a.chan,)):
        off = (CH[cn] - a.tune_khz) * 1000.0
        if abs(off) + 8000 > fs / 2:
            print('# channel %s (%.3f MHz) at %+.1f kHz: outside the band, skipped' % (cn, CH[cn] / 1000, off / 1000)); continue
        res = demod_channel(z, fs, off, cn)
        print('# channel %s (%.3f MHz) at %+.1f kHz: %d CRC-correct frames' % (cn, CH[cn] / 1000, off / 1000, len(res)))
        for tsec, f, ph in res:
            b = payload_bits(f)
            print('  t=%7.2f s  %s' % (tsec, describe(b)))
            print('              %s' % nmea(b, cn))
        total += len(res)
    print('# total %d' % total)

#!/usr/bin/env python3
"""ais_hw_model.py — bit-level model of ais_hw.c on a real capture.

    Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT

  Stands in for the FG23 demodulator with the float slicer of ais_ref.py
  (one sample phase per bit, all 5 phases tried), then does exactly what
  the FG23 packet handler + ais_hw.c do: find the 24-bit sync word (both
  NRZI polarities) in the LINE bits, decode NRZI/HDLC/CRC from the bits
  after it, stop at the closing flag. Checks the sync word and the decoder
  logic before any PHY exists; it says nothing about the radio's own
  demodulator sensitivity (that is what the on-air AISBENCH is for).

  usage: ais_hw_model.py capture.npy --tune_khz 161985 [--sync 24]
"""
import sys, argparse, numpy as np
from fractions import Fraction
from scipy.signal import firwin, lfilter, resample_poly
sys.path.insert(0, __file__.rsplit('firmware-fg23', 1)[0] + 'firmware-esp32/streaming/tools')
import ais_ref as R

def sync_bits(n):
    air = []
    for i in range(n):
        k = n - 1 - i
        air.append(1 if k == 0 else 0 if k < 8 else int(((k - 8) & 3) < 2))
    return air

def crc_ok(fr): return R.crc_ok(fr)

def decode_after(line, j, prev):
    """ais_hw.c dec_bit() from line index j on; returns frame bytes or None"""
    win = 0x7E; byte = 0; nb = 0; buf = bytearray()
    for L in line[j:j + 200 * 8]:
        b = 1 if L == prev else 0; prev = L
        win = ((win << 1) | b) & 0xFF
        if win == 0x7E:
            if len(buf) >= 11: return bytes(buf) if crc_ok(buf) else None
            buf = bytearray(); byte = 0; nb = 0; continue
        if (win & 0x7F) == 0x7F: return None
        if (win & 0x3F) == 0x3E: continue
        byte = (byte >> 1) | (0x80 if b else 0); nb += 1
        if nb == 8:
            if len(buf) >= 128: return None
            buf.append(byte); byte = 0; nb = 0
    return None

ap = argparse.ArgumentParser()
ap.add_argument('iq'); ap.add_argument('--tune_khz', type=float, default=161985)
ap.add_argument('--sync', type=int, default=24)
a = ap.parse_args()
z, fs = R.load(a.iq, 50000)
off = (161975.0 - a.tune_khz) * 1000
n = np.arange(len(z))
y = lfilter(firwin(65, 6000 / (fs / 2)), 1.0, z * np.exp(-2j * np.pi * off * n / fs).astype(np.complex64))
d = np.angle(y[1:] * np.conj(y[:-1]))
fr = Fraction(R.AR, fs).limit_denominator(1000)
d = resample_poly(d, fr.numerator, fr.denominator)
soft = d - np.convolve(d, np.ones(320) / 320, 'same')
S = np.array(sync_bits(a.sync), dtype=np.int8)
ref = {f for _, f, _ in R.demod_channel(z, fs, off, 'A')}
got = set(); nsync = 0
for ph in range(5):
    line = (soft[ph::5] > 0).astype(np.int8)
    m = len(S)
    # sliding compare of both polarities
    from numpy.lib.stride_tricks import sliding_window_view as sw
    W = sw(line, m)
    hit1 = np.where((W == S).all(1))[0]; hit2 = np.where((W == 1 - S).all(1))[0]
    nsync += len(hit1) + len(hit2)
    for h, pol in [(h, 1) for h in hit1] + [(h, 2) for h in hit2]:
        j = h + m
        f = decode_after(line, j, line[j - 1])
        if f: got.add(f)
print('# %s: sync %d bits: %d sync hits (5 phases), %d distinct frames; ais_ref.py: %d; common %d'
      % (a.iq, a.sync, nsync, len(got), len(ref), len(got & ref)))

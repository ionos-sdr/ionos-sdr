#!/usr/bin/env python3
# Passband from the noise floor (no generator): averaged PSD of the I/Q noise at each rate.
import serial, serial.tools.list_ports as lp, struct, time, numpy as np, sys
port = next((p.device for p in lp.comports() if p.vid == 0x303A), "COM9")
h = serial.Serial(); h.port = port; h.baudrate = 115200; h.timeout = 0.02; h.rts = False; h.dtr = True; h.open()
try: h.set_buffer_size(rx_size=1 << 22)
except Exception: pass
h.write(b"MODE USB\n")
def capture(sec):
    buf = bytearray(); out = []; t0 = time.time()
    while time.time() - t0 < sec:
        buf += h.read(1 << 16); o = 0
        while len(buf) - o >= 1040:
            if buf[o:o+4] == b"IQB2":
                ns = struct.unpack_from("<H", buf, o + 10)[0] if False else 256
                out.append(np.frombuffer(bytes(buf[o+16:o+16+1024]), dtype="<i2")); o += 1040
            elif buf[o:o+4] in (b"SPC1", b"MSG1"): o += 1040
            else: o += 1
        del buf[:o]
    x = np.concatenate(out).astype(np.float32); return x[0::2] + 1j * x[1::2]
res = {}
for sps in (200000, 100000, 50000):
    h.write(b"R %d\n" % sps); time.sleep(2.5); h.reset_input_buffer(); capture(0.5)
    z = capture(20.0)
    N = 1024; nseg = len(z) // N; seg = z[:nseg * N].reshape(nseg, N) * np.hanning(N)
    p = np.mean(np.abs(np.fft.fftshift(np.fft.fft(seg, axis=1), axes=1)) ** 2, axis=0)
    f = (np.arange(N) - N // 2) * sps / N
    pdb = 10 * np.log10(p / np.median(p[N//2-N//16:N//2+N//16]))
    pdb[N//2-2:N//2+3] = np.nan                       # DC spike
    k = 9; sm = np.convolve(np.nan_to_num(pdb, nan=0), np.ones(k)/k, mode="same")
    def bw(th):
        idx = np.where(sm > th)[0]; return (f[idx.max()] - f[idx.min()]) / 1000 if len(idx) else 0
    edge = np.mean(sm[np.abs(np.abs(f) - 0.45 * sps) < 0.02 * sps])
    print("%6d sps: %d samples, BW -1 dB %.1f kHz, -3 dB %.1f kHz, -6 dB %.1f kHz, -10 dB %.1f kHz, level at 0.9*Nyq %.1f dB" % (
        sps, len(z), bw(-1), bw(-3), bw(-6), bw(-10), edge), flush=True)
    np.savetxt("passband_%d.csv" % sps, np.c_[f, sm], delimiter=",", fmt="%.2f")
h.write(b"R 50000\n"); h.write(b"MODE STANDALONE\n"); h.close()

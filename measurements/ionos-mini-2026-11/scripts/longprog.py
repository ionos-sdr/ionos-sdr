#!/usr/bin/env python3
# 48 h unattended RF characterisation of ionos-mini at 145 MHz (the PHY band; 434 board, permissive match).
# Chain: VSG60 CW -> 30 dB pad -> 1.5 m coax -> BRD4265B SMA. Board level = VSG - PAD_DB.
# Per cycle (~25 min): drift, passband+image at 200/100/50 ksps, level sweep, USB counters. CSV + log.
import serial, serial.tools.list_ports as lp, struct, time, re, sys, numpy as np, datetime as dt, traceback
from vsg60 import VSG
PAD_DB = 30.5; FC_KHZ = 145000; END = time.time() + float(sys.argv[1] if len(sys.argv) > 1 else 48) * 3600
stamp = dt.datetime.now().strftime("%Y%m%d_%H%M")
LOG = open("longprog_%s.log" % stamp, "a", buffering=1)
def ts(): return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
def log(s): LOG.write("%s %s\n" % (ts(), s)); print(ts(), s, flush=True)
def csvw(name, row):
    with open("lp_%s_%s.csv" % (name, stamp), "a") as f: f.write(",".join(str(x) for x in [ts()] + list(row)) + "\n")
port = next((p.device for p in lp.comports() if p.vid == 0x303A), "COM9")
h = serial.Serial(); h.port = port; h.baudrate = 115200; h.timeout = 0.02; h.rts = False; h.dtr = True; h.open()
try: h.set_buffer_size(rx_size=1 << 22)
except Exception: pass
ST = {"lost": 0, "resync": 0, "iq": 0, "seq": None}
import threading
LK = threading.Lock(); Q = {"iq": [], "rs": [], "txt": b"", "collect": False}
def reader():
    # Reads ALL the time: otherwise the device's 2 MB ring fills while the script
    # sleeps and every capture is seconds old (first run measured the PREVIOUS VSG step).
    buf = bytearray(); seq = None
    while True:
        try: d = h.read(1 << 16)
        except Exception: time.sleep(0.2); continue
        buf += d; o = 0
        while len(buf) - o >= 1040:
            m = bytes(buf[o:o+4])
            if m == b"IQB2":
                s_ = struct.unpack_from("<I", buf, o + 4)[0]
                if seq is not None and s_ > seq + 1: ST["lost"] += s_ - seq - 1
                seq = s_; ST["iq"] += 1
                if Q["collect"]:
                    with LK: Q["iq"].append(np.frombuffer(bytes(buf[o+16:o+1040]), dtype="<i2"))
                o += 1040
            elif m in (b"SPC1", b"MSG1"):
                blk = bytes(buf[o+16:o+1040])
                if m == b"MSG1":
                    r = re.search(rb"rssi=(-?\d+)", blk)
                    if r and Q["collect"]:
                        with LK: Q["rs"].append(int(r.group(1)))
                    with LK: Q["txt"] = (Q["txt"] + blk)[-4096:]
                o += 1040
            else: o += 1; ST["resync"] += 1
        del buf[:o]
threading.Thread(target=reader, daemon=True).start()
def cmd(s, wait=0.4):
    with LK: Q["txt"] = b""
    h.write((s + "\n").encode()); time.sleep(wait)
    with LK: txt = Q["txt"]
    m = re.findall(rb"(OK|ERR)[^\x00]*", txt); return m[0].decode() if m else "?"
def capture(sec):
    with LK: Q["iq"] = []; Q["rs"] = []
    Q["collect"] = True; time.sleep(sec); Q["collect"] = False
    with LK: out = Q["iq"]; rs = Q["rs"]
    x = np.concatenate(out).astype(np.float64) / 32768.0
    return x[0::2] + 1j * x[1::2], rs
def spectrum(z, N):
    nseg = max(1, len(z) // N); w = np.blackman(N)
    S = np.mean([np.abs(np.fft.fftshift(np.fft.fft(z[i*N:(i+1)*N] * w))) ** 2 for i in range(nseg)], axis=0)
    return S / (w.sum() ** 2)                       # a full-scale tone -> 0 dB
def peak(S, fs, f_hz, N, win=4):
    k = int(round(f_hz / fs * N)) + N // 2; k = min(max(k, win), N - 1 - win)
    return 10 * np.log10(S[k-win:k+win+1].sum() / 1.7 + 1e-20)   # blackman ENBW ~1.73 bins
def peak_near(S, fs, f_hz, N, span_hz):
    f = (np.arange(N) - N // 2) * fs / N; m = np.abs(f - f_hz) <= span_hz
    idx = np.where(m)[0]; k = idx[np.argmax(S[idx])]; k = min(max(k, 4), N - 5)
    return 10 * np.log10(S[k-4:k+5].sum() / 1.7 + 1e-20), f[k]
def noise_dbfs_hz(S, fs, N):
    return 10 * np.log10(np.median(S) * N / fs / 1.73 + 1e-30)
def settle(): time.sleep(0.15)
def set_rate(fs):
    r = cmd("R %d" % fs, 2.5); settle(); return r
def tone_at(v, fs, vsg_hz):
    v.freq(vsg_hz, verify=True); time.sleep(2.0)   # VSG60 output follows ~1-1.5 s after the command
    z, _ = capture(1.5); N = 1 << 16; S = spectrum(z, N)
    f = (np.arange(N) - N // 2) * fs / N
    exp = CAL["s"] * (vsg_hz - FC_KHZ * 1e3) + CAL["e"]           # search +-8 kHz around the expected spot
    idx = np.where(np.abs(f - exp) < 8e3)[0]
    if len(idx) == 0: idx = np.arange(N)
    k = idx[np.argmax(S[idx])]; return f[k], 10 * np.log10(S[k] + 1e-20)
CAL = {"s": 1.0, "e": -1477.0}   # measured 2026-11-06 17:36 (F145000, o0)
def find_offset(v, fs):                          # tone = (vsg-FC) + e ; normal spectrum (verified 17:36)
    v.power(-60)
    f1, l1 = tone_at(v, fs, FC_KHZ * 1e3 + 20e3)
    CAL["e"] = f1 - 20e3; return CAL["e"], 1.0, l1
def vsg_for(fo): return FC_KHZ * 1e3 + (fo - CAL["e"]) / CAL["s"]
log("START %s port %s, pad %.1f dB, F%d" % (stamp, port, PAD_DB, FC_KHZ))
log("MODE USB -> %s; o0 -> %s; F -> %s" % (cmd("MODE USB"), cmd("o0"), cmd("F%d" % FC_KHZ, 1.5)))
cycle = 0
with VSG(port=5124) as v:
    log("VSG %s; modulation off" % v.idn()); v.modulation(False); v.on()
    while time.time() < END:
        cycle += 1
        try:
            # 1) frequency offset / drift at 200 ksps
            fs = 200000; set_rate(fs)
            df, sgn, lvl = find_offset(v, fs)
            log("cycle %d: offset %+.1f Hz, spectrum %s (tone %.1f dBFS at -90 dBm board)" % (cycle, df, "normal" if sgn > 0 else "INVERTED", lvl))
            csvw("drift", [cycle, "%.2f" % df, int(sgn), "%.2f" % lvl])
            # 2) passband + image, -90 dBm at the board
            for fs, step in ((200000, 2000), (100000, 1000), (50000, 500)):
                set_rate(fs); v.power(-60)
                for fo in np.arange(-0.49 * fs, 0.49 * fs + 1, step):
                    if abs(fo) < step / 2: continue
                    v.freq(vsg_for(fo), verify=False); time.sleep(1.6)
                    z, rs = capture(0.35); N = 4096; S = spectrum(z, N)
                    sp = min(0.4 * step, 3000.0)
                    p, fp = peak_near(S, fs, fo, N, sp); im, fi = peak_near(S, fs, -fp, N, 300.0)
                    csvw("passband", [cycle, fs, int(fo), "%.2f" % p, "%.0f" % fp, "%.2f" % im, "%.2f" % noise_dbfs_hz(S, fs, N)])
            # 3) level sweep at 200 ksps, tone +20 kHz
            fs = 200000; set_rate(fs); v.freq(vsg_for(20e3), verify=False); time.sleep(0.6)
            for pv in range(-10, -121, -5):
                v.power(pv); time.sleep(1.6)
                z, rs = capture(1.0); N = 1 << 14; S = spectrum(z, N)
                csvw("level", [cycle, pv, "%.1f" % (pv - PAD_DB), "%.2f" % peak_near(S, fs, 20e3, N, 1500.0)[0],
                               "%.2f" % noise_dbfs_hz(S, fs, N), "%.1f" % (np.mean(rs) if rs else float("nan")),
                               "%.4f" % np.max(np.abs(z))])
            v.power(-60)
            log("cycle %d done: iq %d lost %d resync %d" % (cycle, ST["iq"], ST["lost"], ST["resync"]))
            csvw("usb", [cycle, ST["iq"], ST["lost"], ST["resync"]])
            # 4) quiet drift watch for ~10 min (one offset point per minute)
            for _ in range(10):
                if time.time() > END: break
                time.sleep(40); d2, s2, l2 = find_offset(v, 200000); csvw("drift", [cycle, "%.2f" % d2, int(s2), "%.2f" % l2])
        except Exception as e:
            log("cycle %d ERROR %r\n%s" % (cycle, e, traceback.format_exc()))
            time.sleep(30)
            try:
                h.close(); time.sleep(2); h.open(); cmd("MODE USB"); cmd("F%d" % FC_KHZ, 1.5)
            except Exception as e2: log("reopen failed %r" % e2)
    v.off(); log("END cycles %d" % cycle)
h.close()

#!/usr/bin/env python3
# Same measurement primitives as longprog.py (cmd / capture / spectrum / log / csv) but the I/Q comes
# through the SoapyIonos driver (SoapySDR Python bindings) - so every long measurement also soaks
# the driver: lost/truncated/resync/restart counters are read from its sensors.
import SoapySDR, numpy as np, threading, time, datetime as dt
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32
stamp = dt.datetime.now().strftime("%Y%m%d_%H%M")
LOG = open("soapyrun_%s.log" % stamp, "a", buffering=1)
def ts(): return dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
def log(s): LOG.write("%s %s\n" % (ts(), s)); print(ts(), s, flush=True)
def csvw(name, row):
    with open("sp_%s_%s.csv" % (name, stamp), "a") as f: f.write(",".join(str(x) for x in [ts()] + list(row)) + "\n")
dev = SoapySDR.Device("driver=ionos")
log("SoapyIonos: %s" % dict(dev.getHardwareInfo()))
_fs = [200000.0]; dev.setSampleRate(SOAPY_SDR_RX, 0, _fs[0])
st = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32); dev.activateStream(st)
LK = threading.Lock(); Q = {"buf": [], "collect": False, "reads": 0, "ovf": 0}
def _reader():
    b = np.empty(16384, np.complex64)
    while True:
        r = dev.readStream(st, [b], len(b), timeoutUs=200000)
        if r.ret == SoapySDR.SOAPY_SDR_OVERFLOW: Q["ovf"] += 1; continue
        if r.ret > 0 and Q["collect"]:
            with LK: Q["buf"].append(b[:r.ret].copy())
        Q["reads"] += 1
threading.Thread(target=_reader, daemon=True).start()
def cmd(line, wait=0.4):
    dev.writeSetting("command", line); time.sleep(wait); return dev.readSetting("command").replace("\n", " ")[:60]
def set_rate(fs):
    dev.setSampleRate(SOAPY_SDR_RX, 0, float(fs)); _fs[0] = float(fs); time.sleep(0.3)
    return "OK fs=%.0f" % dev.getSampleRate(SOAPY_SDR_RX, 0)
def capture(sec):
    with LK: Q["buf"] = []
    Q["collect"] = True; time.sleep(sec); Q["collect"] = False
    with LK: z = np.concatenate(Q["buf"]) if Q["buf"] else np.zeros(0, np.complex64)
    return z.astype(np.complex128), []
def sensors():
    keys = ("lost_blocks", "gap_events", "truncated_blocks", "resyncs", "stream_restarts", "host_overflows", "blocks")
    return {k: dev.readSensor(k) for k in keys}
def spectrum(z, N):
    nseg = max(1, len(z) // N); w = np.blackman(N)
    S = np.mean([np.abs(np.fft.fftshift(np.fft.fft(z[i*N:(i+1)*N] * w))) ** 2 for i in range(nseg)], axis=0)
    return S / (w.sum() ** 2)
def noise_dbfs_hz(S, fs, N): return 10 * np.log10(np.median(S) * N / fs / 1.73 + 1e-30)

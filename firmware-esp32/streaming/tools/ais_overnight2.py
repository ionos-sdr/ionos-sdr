"""overnight2.py — ID2 + VSG60: dual-channel I/Q AIS (AISIQ2, 162.000 MHz, 100 ksps, A+B,
5 frequency hypotheses) vs FG23 on-chip. Same matrix as overnight.py (2026-11-23), plus
channel B and simultaneous A+B traffic on the I/Q path. Counts per path AND channel from
the !AIVDM lines.

One process holds the VSG and the console (COM18). Each cycle:
  A  sensitivity   PER vs level, both paths
  B  freq offset   PER vs VSG offset at a moderate level, both paths
  C  interference  CW interferer at an offset and dBc, signal at a fixed level
  D  slot stress   a burst in every 26.67 ms slot (max channel load)
  E  channel B     on-chip only (the I/Q path covers channel A only)
Results append to overnight.csv; console log to overnight_console.txt.
  usage: overnight.py [--hours 13] [--step 20]
"""
import ctypes, os, sys, time, argparse, threading, re, csv, datetime
import numpy as np, serial
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ais_gen as G

ap = argparse.ArgumentParser()
ap.add_argument('--hours', type=float, default=13); ap.add_argument('--step', type=float, default=20)
ap.add_argument('--port', default='COM18'); ap.add_argument('--atten', type=float, default=40.0)
ap.add_argument('--fa', type=float, default=161972850.0)   # channel A on the VSG (ID2 crystal compensated)
ap.add_argument('--only', default='ABCDEF'); ap.add_argument('--csv', default='overnight2.csv')
ap.add_argument('--iqcmd', default='AISIQ2')
a = ap.parse_args()
FS = 960000.0
FB = a.fa + 50000.0

# ---------------- waveforms ----------------
def bursts(slot_ms):
    slot = int(FS * slot_ms / 1000); out = []
    short = slot_ms < 30                   # one-slot frames: 168 message bits (like type 1)
    for p in G.NMEA:
        line, _ = G.frame_bits(p, 168 if (short or len(G.armor2bits(p)) < 200) else 272)
        b = G.ramp(G.gmsk(line, FS), FS)
        z = np.zeros(slot, np.complex64); off = int(FS * 0.0003) if short else int(FS * 0.01)
        n = min(len(b), slot - off)
        z[off:off + n] = b[:n]; out.append(z)
    return np.concatenate(out)

W_BASE = bursts(100.0)                     # 10 bursts/s
W_SLOT = bursts(80.0 / 3.0)                # 37.5 bursts/s, every AIS slot
def two_chan(w):
    """the same bursts on A (-25 kHz) and B (+25 kHz) at the same time, VSG centred between;
    each channel at the requested level -> returns (wave, gain dB)"""
    n = np.arange(len(w))
    x = 0.5 * (w * np.exp(-2j * np.pi * 25000 * n / FS) + w * np.exp(2j * np.pi * 25000 * n / FS))
    return x.astype(np.complex64), 20 * np.log10(0.5)
W_AB, G_AB = two_chan(W_BASE)
def with_cw(w, off_hz, dbc):
    """signal amplitude 1 + CW at off_hz, dbc above the signal; returns (wave, signal gain dB)"""
    ai = 10 ** (dbc / 20.0)
    n = np.arange(len(w))
    x = w + ai * np.exp(2j * np.pi * off_hz * n / FS).astype(np.complex64)
    s = 1.0 / (1.0 + ai)
    return (x * s).astype(np.complex64), 20 * np.log10(s)

# ---------------- VSG ----------------
os.add_dll_directory(r'C:\Program Files\Signal Hound\VSG')
v = ctypes.CDLL(r'C:\Program Files\Signal Hound\VSG\vsg_api.dll')
h = ctypes.c_int(-1); assert v.vsgOpenDevice(ctypes.byref(h)) >= 0; h = h.value
v.vsgSetSampleRate(h, ctypes.c_double(FS))
_keep = {}
def vsg_play(w, freq, level):
    v.vsgAbort(h)
    v.vsgSetFrequency(h, ctypes.c_double(freq)); v.vsgSetLevel(h, ctypes.c_double(level))
    iq = np.empty(2 * len(w), np.float32); iq[0::2] = w.real; iq[1::2] = w.imag
    _keep['iq'] = iq
    v.vsgRepeatWaveform(h, iq.ctypes.data_as(ctypes.POINTER(ctypes.c_float)), len(w))

# ---------------- console ----------------
s = serial.Serial(); s.port = a.port; s.baudrate = 115200; s.timeout = 0.1; s.dtr = False; s.rts = False; s.open()
con = open('overnight_console.txt', 'a', encoding='utf-8')
cnt = {k: 0 for k in ('iqA', 'iqB', 'hwA', 'hwB')}; vals = {k: [] for k in cnt}; stop = False
pend = [None]
rx = re.compile(r'\| (-?[\d.]+) dBm SNR (-?[\d.]+) dB df ([+-]?\d+) Hz')
def reader():
    buf = b''
    while not stop:
        buf += s.read(max(1, s.in_waiting))
        *ls, buf = buf.split(b'\n')
        for l in ls:
            t = l.decode('utf-8', 'replace').rstrip()
            if not t or t.startswith(('  TIMING', '  SPI-LEN', 'CW dbg')) or 'blk/s' in t: continue
            if not t.startswith(('!AIVDM', '$PIONS')):
                con.write('%s %s\n' % (datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3], t))
            k = 'hw' if 'AIS RX[HW]:' in t else ('iq' if ('AIS RX:' in t and 'type' in t) else None)
            if k:
                m = rx.search(t)
                pend[0] = (k, (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else None)
            elif t.startswith('!AIVDM') and pend[0]:
                ch = t.split(',')[4] if t.count(',') >= 5 else 'A'
                key = pend[0][0] + (ch if ch in ('A', 'B') else 'A')
                cnt[key] += 1
                if pend[0][1]: vals[key].append(pend[0][1])
                pend[0] = None
threading.Thread(target=reader, daemon=True).start()
def cmd(c, wait=0.5):
    s.write((c + '\n').encode()); con.write('>>> %s\n' % c); time.sleep(wait)

path_now = [None]
LABEL = ['iq']                                  # CSV label of the I/Q path: iq = AFC on, iq_noafc = AFC off
def set_path(p, chan='A'):
    key = p + chan
    if path_now[0] == key: return
    if p == 'hw':
        cmd('AISHW0', 0.6); cmd('AISHW1' if chan == 'A' else 'AISHW2', 1.5)
    else:
        cmd('AISHW0', 0.6); cmd(a.iqcmd, 4.0)
    path_now[0] = key

# ---------------- measurement ----------------
out_new = not os.path.exists(a.csv)
fcsv = open(a.csv, 'a', newline='')
wr = csv.writer(fcsv)
if out_new:
    wr.writerow(['time', 'cycle', 'test', 'path', 'chan', 'param', 'board_dbm', 'sent', 'ok', 'per_pct',
                 'mean_dbm', 'mean_snr', 'mean_df', 'std_df'])
def measure(cycle, test, path, chan, param, wave, freq, board_dbm, gain_db, rate, count_chan=None):
    set_path(path, 'A' if (path == 'iq' or chan == 'AB') else chan)
    vsg_play(wave, freq, board_dbm + a.atten - gain_db)
    time.sleep(1.5)                                    # settle, not counted
    keys = [path + c for c in (count_chan or chan)]
    c0 = {k: cnt[k] for k in keys}; v0 = {k: len(vals[k]) for k in keys}; t0 = time.time()
    time.sleep(a.step)
    dt = time.time() - t0
    time.sleep(0.3)                                    # the last frames reach the console
    rows = []
    for k in keys:
        rows.append(_row(cycle, test, LABEL[0] if path == 'iq' else path, k[-1], param, board_dbm, rate, dt, cnt[k] - c0[k], vals[k][v0[k]:]))
    return rows

def _row(cycle, test, path, chan, param, board_dbm, rate, dt, ok, vv):
    sent = rate * dt
    ok = min(ok, int(round(sent)))                     # a frame of the settle time can land inside
    dfs = [x[2] for x in vv]
    row = [datetime.datetime.now().isoformat(timespec='seconds'), cycle, test, path, chan, param,
           '%.1f' % board_dbm, '%.0f' % sent, ok, '%.1f' % (100 * (1 - ok / sent)),
           '%.1f' % np.mean([x[0] for x in vv]) if vv else '', '%.1f' % np.mean([x[1] for x in vv]) if vv else '',
           '%.0f' % np.mean(dfs) if dfs else '', '%.0f' % np.std(dfs) if dfs else '']
    wr.writerow(row); fcsv.flush()
    print(' '.join(str(x) for x in row), flush=True)

t_end = time.time() + a.hours * 3600
cycle = 0
try:
    while time.time() < t_end:
        cycle += 1
        # odd cycles AFC on (default), even cycles AFC off: what the AFC itself is worth
        afc = (cycle % 2 == 1)
        LABEL[0] = 'iq' if afc else 'iq_noafc'
        cmd('AISIQA8' if afc else 'AISIQA0', 1.0)
        for p in ('hw', 'iq'):
            if 'A' in a.only:
                for L in (-96, -100, -102, -103, -104, -105, -106, -107, -108):
                    measure(cycle, 'A_sens', p, 'A', '', W_BASE, a.fa, L, 0.0, 10)
            if 'B' in a.only:
                for df in (-4000, -3000, -2000, -1000, 0, 1000, 2000, 3000, 4000):
                    measure(cycle, 'B_foff', p, 'A', df, W_BASE, a.fa + df, -103, 0.0, 10)
            if 'C' in a.only:
                for off in (25000, -25000, 50000, 100000, -300000):
                    for dbc in (30, 40, 50, 60):
                        w, g = with_cw(W_BASE, off, dbc)
                        measure(cycle, 'C_cw', p, 'A', '%d@%d' % (dbc, off), w, a.fa, -100, g, 10)
            if 'D' in a.only:
                for L in (-96, -104):
                    measure(cycle, 'D_slot', p, 'A', '37.5/s', W_SLOT, a.fa, L, 0.0, 37.5)
            if 'E' in a.only:                          # channel B alone (on-chip: A2, I/Q: dual)
                for L in (-100, -104, -106):
                    measure(cycle, 'E_chB', p, 'B', '', W_BASE, FB, L, 0.0, 10)
            if time.time() > t_end: break
        if 'F' in a.only:                              # A and B at the same time, I/Q dual only
            for L in (-96, -100, -104, -106):
                measure(cycle, 'F_AB', 'iq', 'AB', 'simultaneous', W_AB, a.fa + 25000, L, G_AB, 10)
finally:
    stop = True; time.sleep(0.3)
    v.vsgAbort(h); v.vsgCloseDevice(h)
    cmd('AISHW0', 0.5); cmd(a.iqcmd, 1.0)
    s.close(); con.close(); fcsv.close()
    print('done, cycles', cycle)

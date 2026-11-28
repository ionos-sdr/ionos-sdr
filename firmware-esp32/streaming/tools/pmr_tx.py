#!/usr/bin/env python3
"""pmr_tx.py - stream the PC's sound output (WASAPI loopback, e.g. YouTube)
to the ionos-mini and transmit it as NBFM from the FG23.

    PC loopback 48 kHz float -> mono -> 300-3000 Hz band-pass -> pre-emphasis
    -> limiter -> 8 kHz int8 -> TCP 7355 (ESP32) -> cmdlink -> FG23 'N'

The ESP forwards the header lines to the FG23 as commands, then the audio.
Defaults: PMR446 channel 1 (446.00625 MHz), +-2.5 kHz peak deviation,
MINIMUM transmit power (conducted measurement: attenuator in front of the
analyser). Ctrl-C stops; the FG23 drops the carrier 1 s after the audio ends.

    python pmr_tx.py 192.168.1.243 --ctcss 88.5 --power -20
    python pmr_tx.py 192.168.1.243 --tone 1000      # 1 kHz test tone instead of the PC sound

Needs only numpy: the WASAPI loopback is called through ctypes (no
soundcard/pyaudio packages).
"""
import argparse, ctypes, ctypes.wintypes as wt, math, socket, sys, time
import numpy as np

# ------------------------------------------------------------------ WASAPI loopback (ctypes COM)
ole32 = ctypes.OleDLL('ole32')


class GUID(ctypes.Structure):
    _fields_ = [('d1', ctypes.c_uint32), ('d2', ctypes.c_uint16), ('d3', ctypes.c_uint16), ('d4', ctypes.c_ubyte * 8)]

    def __init__(self, s):
        super().__init__()
        ole32.CLSIDFromString(ctypes.c_wchar_p(s), ctypes.byref(self))


CLSID_MMDeviceEnumerator = GUID('{BCDE0395-E52F-467C-8E3D-C4579291692E}')
IID_IMMDeviceEnumerator = GUID('{A95664D2-9614-4F35-A746-DE8DB63617E6}')
IID_IAudioClient = GUID('{1CB9AD4C-DBFA-4C32-B178-C2F568A703B2}')
IID_IAudioCaptureClient = GUID('{C8ADBD64-E71E-48A0-A4DE-185C395CD317}')
CLSCTX_ALL = 0x17
AUDCLNT_SHAREMODE_SHARED = 0
AUDCLNT_STREAMFLAGS_LOOPBACK = 0x00020000
AUDCLNT_BUFFERFLAGS_SILENT = 0x2


class WAVEFORMATEX(ctypes.Structure):
    _fields_ = [('wFormatTag', wt.WORD), ('nChannels', wt.WORD), ('nSamplesPerSec', wt.DWORD),
                ('nAvgBytesPerSec', wt.DWORD), ('nBlockAlign', wt.WORD), ('wBitsPerSample', wt.WORD), ('cbSize', wt.WORD)]


def vcall(obj, idx, restype, *args, argtypes=()):
    """Call method #idx of a COM object's vtable."""
    vtbl = ctypes.cast(obj, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))).contents
    fn = ctypes.WINFUNCTYPE(restype, ctypes.c_void_p, *argtypes)(vtbl[idx])
    r = fn(obj, *args)
    if restype is ctypes.HRESULT:
        return r
    return r


class Loopback:
    def __init__(self):
        ole32.CoInitializeEx(None, 0)
        enum = ctypes.c_void_p()
        ole32.CoCreateInstance(ctypes.byref(CLSID_MMDeviceEnumerator), None, CLSCTX_ALL,
                               ctypes.byref(IID_IMMDeviceEnumerator), ctypes.byref(enum))
        dev = ctypes.c_void_p()
        # IMMDeviceEnumerator::GetDefaultAudioEndpoint(eRender=0, eConsole=0, &dev)
        vcall(enum, 4, ctypes.HRESULT, 0, 0, ctypes.byref(dev),
              argtypes=(ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)))
        self.ac = ctypes.c_void_p()
        # IMMDevice::Activate(iid, CLSCTX_ALL, NULL, &client)
        vcall(dev, 3, ctypes.HRESULT, ctypes.byref(IID_IAudioClient), CLSCTX_ALL, None, ctypes.byref(self.ac),
              argtypes=(ctypes.POINTER(GUID), wt.DWORD, ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)))
        pwfx = ctypes.POINTER(WAVEFORMATEX)()
        vcall(self.ac, 8, ctypes.HRESULT, ctypes.byref(pwfx), argtypes=(ctypes.POINTER(ctypes.POINTER(WAVEFORMATEX)),))
        f = pwfx.contents
        self.fs, self.ch, self.bits, self.align = f.nSamplesPerSec, f.nChannels, f.wBitsPerSample, f.nBlockAlign
        if self.bits != 32:
            raise SystemExit('mix format is %d-bit; only float32 handled' % self.bits)
        # Initialize(shared, loopback, 200 ms buffer, 0, fmt, NULL)
        vcall(self.ac, 3, ctypes.HRESULT, AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_LOOPBACK,
              ctypes.c_longlong(2000000), ctypes.c_longlong(0), pwfx, None,
              argtypes=(ctypes.c_int, wt.DWORD, ctypes.c_longlong, ctypes.c_longlong,
                        ctypes.POINTER(WAVEFORMATEX), ctypes.c_void_p))
        self.cc = ctypes.c_void_p()
        vcall(self.ac, 14, ctypes.HRESULT, ctypes.byref(IID_IAudioCaptureClient), ctypes.byref(self.cc),
              argtypes=(ctypes.POINTER(GUID), ctypes.POINTER(ctypes.c_void_p)))
        vcall(self.ac, 10, ctypes.HRESULT)                     # Start

    def read(self):
        """All frames waiting now, as float32 (n, ch). Silent packets come back as zeros."""
        out = []
        while True:
            n = ctypes.c_uint32()
            vcall(self.cc, 5, ctypes.HRESULT, ctypes.byref(n), argtypes=(ctypes.POINTER(ctypes.c_uint32),))
            if n.value == 0:
                break
            data = ctypes.POINTER(ctypes.c_float)()
            frames, flags = ctypes.c_uint32(), wt.DWORD()
            vcall(self.cc, 3, ctypes.HRESULT, ctypes.byref(data), ctypes.byref(frames), ctypes.byref(flags), None, None,
                  argtypes=(ctypes.POINTER(ctypes.POINTER(ctypes.c_float)), ctypes.POINTER(ctypes.c_uint32),
                            ctypes.POINTER(wt.DWORD), ctypes.c_void_p, ctypes.c_void_p))
            k = frames.value
            if flags.value & AUDCLNT_BUFFERFLAGS_SILENT:
                out.append(np.zeros((k, self.ch), np.float32))
            else:
                out.append(np.ctypeslib.as_array(data, shape=(k * self.ch,)).copy().reshape(k, self.ch))
            vcall(self.cc, 4, ctypes.HRESULT, frames, argtypes=(ctypes.c_uint32,))
        return np.concatenate(out) if out else np.zeros((0, self.ch), np.float32)


# ------------------------------------------------------------------ DSP
def fir_design(fs, ntaps=2047, f_lo=290.0, f_hi=3000.0, fs_out=8000.0):
    """Linear-phase FIR by frequency sampling: 300-3000 Hz pass band (cosine edges
    240-290 and 3000-3200 Hz, so 67-240 Hz stays free for CTCSS and nothing
    reaches 4 kHz before the /6 decimation), with the FG23's 4x LINEAR
    interpolation pre-compensated: it acts as sinc^2(f / 8 kHz) (-2.8 dB at
    2.5 kHz), measured as the high-end droop of the first chain run."""
    nf = 1 << 15
    f = np.arange(nf // 2 + 1) * fs / nf
    d = np.zeros_like(f)
    ramp = lambda x: 0.5 - 0.5 * np.cos(np.pi * np.clip(x, 0, 1))
    d = np.where(f < f_lo, ramp((f - 240.0) / (f_lo - 240.0)), 1.0)
    d = np.where(f > f_hi, 1.0 - ramp((f - f_hi) / 200.0), d)
    d[f < 240.0] = 0.0
    d[f > f_hi + 200.0] = 0.0
    d = d / np.sinc(np.minimum(f, f_hi + 200.0) / fs_out) ** 2
    h = np.fft.irfft(d, nf)
    h = np.roll(h, ntaps // 2)[:ntaps] * np.blackman(ntaps)
    return h


class Chain:
    """48 kHz mono -> band-pass 300..3000 (also the anti-alias for /6) -> 8 kHz -> pre-emphasis -> limiter."""

    def __init__(self, fs, preemph_us=750.0, drive_db=6.0):
        self.dec = int(round(fs / 8000))
        if abs(fs / self.dec - 8000) > 1:
            raise SystemExit('loopback rate %d Hz is not a multiple of 8 kHz' % fs)
        self.h = fir_design(fs)
        self.zi = np.zeros(len(self.h) - 1)
        self.phase = 0
        a = math.exp(-1.0 / (preemph_us * 1e-6 * 8000))       # exact inverse of the receiver's 750 us de-emphasis
        g1k = abs(1 - a * np.exp(-2j * np.pi * 1000 / 8000))
        self.pa, self.px, self.pg = a, 0.0, 1.0 / g1k                 # unity gain at 1 kHz
        self.drive = 10 ** (drive_db / 20)
        self.env = 1e-3

    def process(self, x):
        y = np.convolve(np.concatenate([self.zi, x]), self.h, mode='valid')
        self.zi = np.concatenate([self.zi, x])[-(len(self.h) - 1):]
        y = y[(self.dec - self.phase) % self.dec::self.dec]
        self.phase = (self.phase + len(x)) % self.dec
        if not len(y):
            return np.zeros(0)
        prev = np.concatenate([[self.px], y[:-1]])
        self.px = y[-1]
        y = (y - self.pa * prev) * self.pg * self.drive
        out = np.empty_like(y)                                 # peak limiter: fast attack, 1 s release
        rel = math.exp(-1.0 / 8000.0)
        env = self.env
        for i, v in enumerate(y):
            a = abs(v)
            env = a if a > env else env * rel + (1 - rel) * a
            g = 0.9 / env if env > 0.9 else 1.0
            out[i] = v * g
        self.env = env
        return out                                           # float, +-0.9 peak


def enc8(y):
    return np.clip(np.round(y * 127), -127, 127).astype(np.int8).tobytes()


def enc14(y):
    """14-bit two's complement as byte pairs: high 7 bits | 0x80, then low 7 bits."""
    v = np.clip(np.round(y * 8191), -8191, 8191).astype(np.int32) & 0x3FFF
    b = np.empty(2 * len(v), np.uint8)
    b[0::2] = 0x80 | (v >> 7)
    b[1::2] = v & 0x7F
    return b.tobytes()


# ------------------------------------------------------------------ test programme
TEST_STEPS = [200, 300, 400, 500, 700, 1000, 1500, 2000, 2500, 3000, 3400, 4000]


def test_programme(fs):
    """Deterministic measurement signal at the loopback rate (48 kHz), fed
    through the same chain as real audio. Segments (s):
      0-2 silence | 2-5 1 kHz -10 dBFS | 5-17 tone steps -26 dBFS, 1 s each
      | 17-20 two-tone 700+1900 Hz -20 dBFS each | 20-23 silence
    The RX analysis (fm_analyze.py) cuts it by these times."""
    def tone(f, sec, dbfs):
        t = np.arange(int(sec * fs)) / fs
        return 10 ** (dbfs / 20) * np.sin(2 * np.pi * f * t)
    seg = [np.zeros(2 * fs), tone(1000, 3, -10)]
    seg += [tone(f, 1, -26) for f in TEST_STEPS]
    t = np.arange(3 * fs) / fs
    seg.append(10 ** (-20 / 20) * (np.sin(2 * np.pi * 700 * t) + np.sin(2 * np.pi * 1900 * t)))
    seg.append(np.zeros(3 * fs))
    return np.concatenate(seg)


# ------------------------------------------------------------------ main
def connect(host, port, hdr, go='GO', tries=15):
    """The ESP serves one transmitter at a time and keeps the link ~2.5 s after
    the previous one closed (the FG23 drops the carrier, then resumes RX);
    meanwhile a new connection is reset. Retry until the first line is
    acknowledged ('ok ...'), then send the rest."""
    for k in range(tries):
        try:
            s = socket.create_connection((host, port), timeout=5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.sendall((hdr[0] + '\n').encode())
            r = s.recv(64)
            if r.startswith(b'ok'):
                print('>', hdr[0])
                for l in hdr[1:] + [go]:
                    time.sleep(0.35)
                    s.sendall((l + '\n').encode())
                    print('>', l)
                time.sleep(0.35)
                return s
            s.close()
        except (ConnectionResetError, ConnectionAbortedError, socket.timeout, OSError):
            pass
        print('ionos-mini busy (previous TX still closing), retry %d...' % (k + 1))
        time.sleep(1.0)
    raise SystemExit('ionos-mini did not accept the connection')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('host', help='ionos-mini IP (ESP32)')
    ap.add_argument('--port', type=int, default=7355)
    ap.add_argument('--format', type=int, choices=(8, 14), default=14,
                    help='audio on the ESP-FG23 link: 14 = 14-bit @230400 (default), 8 = int8 @115200')
    ap.add_argument('--freq', default='446006.250', help='kHz[.Hz] for the FG23 F command (PMR446 ch1)')
    ap.add_argument('--dev', type=int, default=2500, help='peak deviation, Hz (total, CTCSS included)')
    ap.add_argument('--tick', type=int, default=0, help='synth tick, mHz/tick (Nk) - 0 = keep the FG23 value')
    ap.add_argument('--ctcss', type=float, default=0.0, help='CTCSS tone Hz, 0 = off (e.g. 88.5)')
    ap.add_argument('--ctcss-dev', type=int, default=350, help='CTCSS deviation, Hz')
    ap.add_argument('--power', type=int, default=None, help='dBm; omitted = FG23 minimum')
    ap.add_argument('--drive', type=float, default=6.0, help='audio gain before the limiter, dB')
    ap.add_argument('--tone', type=float, default=0.0, help='send a test tone (Hz) instead of the PC sound')
    ap.add_argument('--test-signal', action='store_true',
                    help='send the 23 s measurement programme through the audio chain, then stop')
    a = ap.parse_args()

    hdr = ['F' + a.freq, 'Nd%d' % a.dev, 'Nc%d' % int(round(a.ctcss * 10)), 'Nl%d' % a.ctcss_dev]
    if a.tick:
        hdr.append('Nk%d' % a.tick)
    hdr.append('Np%d' % a.power if a.power is not None else 'Np-40')
    s = connect(a.host, a.port, hdr, 'GO2' if a.format == 14 else 'GO')
    enc = enc14 if a.format == 14 else enc8
    s.settimeout(None)

    lb = None if (a.tone or a.test_signal) else Loopback()
    if lb:
        print('loopback: %d Hz, %d ch' % (lb.fs, lb.ch))
        chain = Chain(lb.fs, drive_db=a.drive)
    prog = None
    if a.test_signal:
        chain = Chain(48000, drive_db=a.drive)
        prog, prog_pos = test_programme(48000), 0
    t0 = time.monotonic()
    sent, ph = 0, 0.0
    try:
        while True:
            if prog is not None:
                want = int((time.monotonic() - t0) * 48000) + 2400 - prog_pos  # real time, ~50 ms ahead
                if want > 0:
                    x = prog[prog_pos:prog_pos + want]; prog_pos += len(x)
                    pcm = chain.process(x)
                else:
                    pcm = np.zeros(0)
                if prog_pos >= len(prog):
                    s.sendall(enc(pcm)); sent += len(pcm)
                    print('test programme done (%.1f s)' % (sent / 8000))
                    break
            elif a.tone:
                n = int((time.monotonic() - t0) * 8000) - sent + 400       # keep ~50 ms ahead
                if n > 0:
                    t = (ph + np.arange(n)) * 2 * math.pi * a.tone / 8000
                    pcm = np.sin(t) * 0.95
                    ph += n
                else:
                    pcm = np.zeros(0)
            else:
                x = lb.read()
                pcm = chain.process(x.mean(axis=1).astype(np.float64)) if len(x) else np.zeros(0)
                behind = int((time.monotonic() - t0) * 8000) - (sent + len(pcm))
                if behind > 1600:                                        # loopback delivers nothing while
                    pcm = np.concatenate([pcm, np.zeros(behind - 400)])   # nothing plays: pad silence
            if len(pcm):
                s.sendall(enc(pcm))
                sent += len(pcm)
            if sent and sent % 80000 < len(pcm):
                print('%6.1f s  %d samples' % (time.monotonic() - t0, sent), flush=True)
            time.sleep(0.02)
    except (KeyboardInterrupt, ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as e:
        if not isinstance(e, KeyboardInterrupt):
            print('connection lost:', e)
    finally:
        try:
            s.close()
        except OSError:
            pass
        print('stopped, %d samples (%.1f s)' % (sent, sent / 8000))


if __name__ == '__main__':
    main()

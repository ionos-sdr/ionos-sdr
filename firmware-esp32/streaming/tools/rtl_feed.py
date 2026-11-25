#!/usr/bin/env python3
"""rtl_feed.py - RTL-SDR on the same antenna -> the ionos-mini ESP's Beast input (TCP 30004).

The frames land in the ESP's own tables, so the same map (tar1090-style) and the
same graphs show them, counted separately as msgs_rtl. Hot-swap: the antenna/LNA
is moved between the FG23 board and the RTL-SDR; whichever hears the traffic
fills the dashboard. Unplugging the dongle or losing WiFi only pauses the feed.

Sources:
  rtl_adsb (default)  rtl_adsb.exe from radioconda, raw '*hex;' lines, no signal level
  beast               a dump1090/readsb-style Beast output, e.g. --src beast --beast 127.0.0.1:30005

Only CRC-good DF11 (IID 0), DF17 and DF18 frames are forwarded (what the FG23
path delivers too).

usage: python rtl_feed.py [--esp 192.168.1.243:30004] [--src rtl_adsb|beast]
                          [--gain 20] [--exe ...rtl_adsb.exe] [--beast host:port]
"""
import argparse, socket, subprocess, sys, time, threading, os

ap = argparse.ArgumentParser()
ap.add_argument('--esp', default='192.168.1.243:30004')
ap.add_argument('--src', default='rtl_adsb', choices=('rtl_adsb', 'beast'))
ap.add_argument('--exe', default=os.path.expanduser(r'~\radioconda\Library\bin\rtl_adsb.exe'))
ap.add_argument('--gain', default='20', help='tuner gain dB for rtl_adsb (the LNA ahead gives ~40 dB)')
ap.add_argument('--beast', default='127.0.0.1:30005')
ap.add_argument('--log', default=None)
a = ap.parse_args()

def log(*x):
    s = time.strftime('%Y-%m-%d %H:%M:%S ') + ' '.join(str(v) for v in x)
    print(s, flush=True)
    if a.log:
        with open(a.log, 'a', encoding='utf-8') as f: f.write(s + '\n')

# ---- Mode S CRC-24 ----
POLY = 0xFFF409
TAB = []
for i in range(256):
    c = i << 16
    for _ in range(8):
        c = ((c << 1) ^ POLY) if c & 0x800000 else (c << 1)
    TAB.append(c & 0xFFFFFF)
def crc24(b):
    c = 0
    for x in b[:-3]:
        c = ((c << 8) & 0xFFFFFF) ^ TAB[((c >> 16) ^ x) & 0xFF]
    return c ^ ((b[-3] << 16) | (b[-2] << 8) | b[-1])

def wanted(m):
    df = m[0] >> 3
    if df in (17, 18) and len(m) == 14: return crc24(m) == 0
    if df == 11 and len(m) == 7: return crc24(m) == 0      # IID 0 only, like the FG23 path
    return False

T0 = time.perf_counter()
def beast(m, sig=0):
    ts = int((time.perf_counter() - T0) * 12e6) & 0xFFFFFFFFFFFF          # 12 MHz counter
    raw = ts.to_bytes(6, 'big') + bytes([sig]) + bytes(m)
    out = bytearray(b'\x1a' + (b'3' if len(m) == 14 else b'2'))
    for c in raw:
        out.append(c)
        if c == 0x1A: out.append(0x1A)
    return bytes(out)

# ---- ESP connection, reconnecting ----
class Esp:
    def __init__(self, hp):
        h, p = hp.rsplit(':', 1); self.addr = (h, int(p)); self.s = None; self.sent = 0; self.next_try = 0
    def send(self, data):
        if self.s is None:
            if time.time() < self.next_try: return False
            try:
                self.s = socket.create_connection(self.addr, timeout=5); self.s.settimeout(5)
                self.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                log('ESP connected', self.addr)
            except OSError as e:
                log('ESP connect failed:', e); self.s = None; self.next_try = time.time() + 5; return False
        try:
            self.s.sendall(data); self.sent += 1; return True
        except OSError as e:
            log('ESP send failed:', e)
            try: self.s.close()
            except OSError: pass
            self.s = None; self.next_try = time.time() + 2; return False

esp = Esp(a.esp)
stat = {'in': 0, 'ok': 0, 'drop_crc': 0}

def status_loop():
    last = dict(stat); last_t = time.time()
    while True:
        time.sleep(60)
        now = time.time(); dt = now - last_t
        log('status: in %d/min, forwarded %d/min, rejected %d/min, ESP %s' % (
            (stat['in'] - last['in']) * 60 / dt, (stat['ok'] - last['ok']) * 60 / dt,
            (stat['drop_crc'] - last['drop_crc']) * 60 / dt, 'up' if esp.s else 'down'))
        last = dict(stat); last_t = now
threading.Thread(target=status_loop, daemon=True).start()

def handle(m, sig=0):
    stat['in'] += 1
    if not wanted(m): stat['drop_crc'] += 1; return
    stat['ok'] += 1
    esp.send(beast(m, sig))

def run_rtl_adsb():
    cmd = [a.exe, '-g', str(a.gain)]
    while True:
        log('starting', ' '.join(cmd))
        try:
            p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=1,
                                 universal_newlines=True, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        except OSError as e:
            log('cannot start rtl_adsb:', e); time.sleep(10); continue
        got = 0
        for line in p.stdout:
            line = line.strip()
            if line.startswith('*') and line.endswith(';'):
                try: m = bytes.fromhex(line[1:-1])
                except ValueError: continue
                handle(m); got += 1
            elif line:
                log('rtl_adsb:', line[:160])
        p.wait()
        log('rtl_adsb exited (%s) after %d frames - dongle unplugged or busy? retry in 5 s' % (p.returncode, got))
        time.sleep(5)

def run_beast():
    h, p = a.beast.rsplit(':', 1)
    while True:
        try:
            s = socket.create_connection((h, int(p)), timeout=10); log('Beast source connected', a.beast)
        except OSError as e:
            log('Beast source:', e); time.sleep(5); continue
        buf = b''; s.settimeout(30)
        try:
            while True:
                d = s.recv(4096)
                if not d: break
                buf += d
                while True:
                    i = buf.find(b'\x1a')
                    if i < 0 or i + 1 >= len(buf): buf = buf[i:] if i >= 0 else b''; break
                    t = buf[i + 1:i + 2]
                    n = {b'2': 7, b'3': 14, b'1': 2}.get(t)
                    if n is None: buf = buf[i + 1:]; continue
                    out = bytearray(); j = i + 2
                    while len(out) < 7 + n and j < len(buf):
                        c = buf[j]
                        if c == 0x1A:
                            if j + 1 >= len(buf): break
                            j += 1
                        out.append(buf[j]); j += 1
                    if len(out) < 7 + n: break
                    buf = buf[j:]
                    if t in (b'2', b'3'): handle(bytes(out[7:]), out[6])
        except OSError as e:
            log('Beast source lost:', e)
        try: s.close()
        except OSError: pass
        time.sleep(3)

log('rtl_feed ->', a.esp, 'source', a.src)
(run_rtl_adsb if a.src == 'rtl_adsb' else run_beast)()

"""pb_meas.py - NBFM TX characterisation on the ProBook: the FG23 transmits
(ionos-mini ID1, 60 dB pad) and the RTL-SDR records I/Q with rtl_sdr.exe.

    python pb_meas.py <step> [<step> ...]      steps: see STEPS
Captures go to %HOME%\\nbfm_meas\\<name>.cu8 plus <name>.json (settings).
"""
import json, os, socket, subprocess, sys, threading, time

HOST = '192.168.1.243'
RTL = r'C:\Users\User\radioconda\Library\bin\rtl_sdr.exe'
OUT = os.path.join(os.path.expanduser('~'), 'nbfm_meas')
F0 = 446006250                        # PMR446 ch1
OFF = 250000                          # RTL tuned below: carrier at +250 kHz, away from its DC spike
os.makedirs(OUT, exist_ok=True)
PY = r'C:\Python315\python.exe'
NK = int(os.environ.get('NK', '14190'))   # synth tick at 446 MHz, mHz (t1k: 3112 Hz for 2500 at 11400)
PMR = os.path.join(os.path.expanduser('~'), 'aprs_id2', 'pmr_tx.py')


def fg(lines, hold=0.0):
    """Header lines to the FG23 through the ESP (port 7355), each acknowledged."""
    s = socket.create_connection((HOST, 7355), timeout=15)
    for l in lines:
        for k in range(20):
            try:
                s.sendall((l + '\n').encode())
                r = s.recv(64)
                break
            except (ConnectionResetError, ConnectionAbortedError):
                time.sleep(1.0); s = socket.create_connection((HOST, 7355), timeout=15)
        print('  >', l, r.decode(errors='replace').strip())
    time.sleep(hold)
    s.close()


def cap(name, fc, fs, gain, secs, meta):
    fn = os.path.join(OUT, name + '.cu8')
    cmd = [RTL, '-f', str(int(fc)), '-s', str(int(fs)), '-g', str(gain), '-n', str(int(fs * secs)), fn]
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(RTL))
    meta.update(dict(name=name, fc=fc, fs=fs, gain=gain, secs=secs, f0=F0, t=time.strftime('%Y-%m-%d %H:%M:%S'),
                     rtl_tail=(r.stderr or '')[-300:]))
    json.dump(meta, open(os.path.join(OUT, name + '.json'), 'w'), indent=1)
    print('  cap %s: %s B, rc %d' % (name, os.path.getsize(fn) if os.path.exists(fn) else 0, r.returncode))


def tone_test(name, cmds, fc, fs, gain, secs, tx_s, meta=None):
    """Nt test tone (TX inside the FG23 for tx_s), capture in the middle."""
    meta = dict(meta or {}, cmds=cmds)
    th = threading.Thread(target=fg, args=(cmds, tx_s + 1.5)); th.start()
    time.sleep(1.2 + 0.4 * len(cmds))
    cap(name, fc, fs, gain, secs, meta)
    th.join()
    time.sleep(2.5)


def cw_plain(name, gain):
    """'w' = plain RAIL carrier stream, no SetFreqOffset writes at all (reference for the NCO spurs)."""
    s = socket.create_connection((HOST, 7355), timeout=15)
    for l in ['F446006.250', 'd-10', 'w']:
        s.sendall((l + '\n').encode()); print('  >', l, s.recv(64).decode(errors='replace').strip())
    time.sleep(1.0)
    cap(name, F0 - OFF, 1024000, gain, 4, dict(cmds=['w']))
    s.sendall(b'x\n'); print('  > x', s.recv(64).decode(errors='replace').strip())
    time.sleep(1.0); s.close(); time.sleep(2.5)


def chain_test(name, extra, gain):
    """The 23 s measurement programme through PC chain -> ESP -> FG23; the capture
    starts first (30 s) so the whole programme, silence included, is in it."""
    meta = dict(chain=True, extra=extra)
    th = threading.Thread(target=cap, args=(name, F0 - OFF, 1024000, gain, 30, meta)); th.start()
    time.sleep(0.8)
    p = subprocess.Popen([PY, PMR, HOST, '--test-signal'] + extra)
    p.wait(timeout=90)
    th.join()
    time.sleep(3)


def base(p, d=2500, c=0):
    return ['F446006.250', 'Np%d' % p, 'Nd%d' % d, 'Nc%d' % c]


STEPS = {
    'off':     lambda: cap('off', F0 - OFF, 1024000, 40, 3, dict(tx=False)),
    'off_g20': lambda: cap('off_g20', F0 - OFF, 1024000, 20, 3, dict(tx=False)),
    'cw_min':  lambda: tone_test('cw_min', base(-40, 0) + ['Nt1000,8'], F0 - OFF, 1024000, 40, 4, 8),
    'cw_m10':  lambda: tone_test('cw_m10', base(-10, 0) + ['Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    'cw_0':    lambda: tone_test('cw_0', base(0, 0) + ['Nt1000,8'], F0 - OFF, 1024000, 10, 4, 8),
    't1k':     lambda: tone_test('t1k', base(-10) + ['Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    'bessel':  lambda: tone_test('bessel', base(-10, 2405) + ['Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    'ctcss':   lambda: tone_test('ctcss', base(-10, 2500, 885) + ['Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    'wide_lo': lambda: tone_test('wide_lo', base(0) + ['Nt1000,8'], F0 - 2400000, 2400000, 10, 3, 8),
    'wide_c':  lambda: tone_test('wide_c', base(0) + ['Nt1000,8'], F0 - 600000, 2400000, 10, 3, 8),
    'wide_hi': lambda: tone_test('wide_hi', base(0) + ['Nt1000,8'], F0 + 2400000, 2400000, 10, 3, 8),
    'h2':      lambda: tone_test('h2', base(0, 0) + ['Nt1000,8'], 2 * F0 - 300000, 2400000, 49, 3, 8),
    'h3':      lambda: tone_test('h3', base(0, 0) + ['Nt1000,8'], 3 * F0 - 300000, 2400000, 49, 3, 8),
    'cw_w':    lambda: cw_plain('cw_w', 20),
    'cw_k':    lambda: tone_test('cw_k', base(-10, 0) + ['Nk%d' % NK, 'Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    't1k_k':   lambda: tone_test('t1k_k', base(-10) + ['Nk%d' % NK, 'Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    'bessel_k': lambda: tone_test('bessel_k', base(-10, 2405) + ['Nk%d' % NK, 'Nt1000,8'], F0 - OFF, 1024000, 20, 4, 8),
    'chain':   lambda: chain_test('chain', ['--power', '-10'], 20),
    'chain_ct': lambda: chain_test('chain_ct', ['--power', '-10', '--ctcss', '88.5'], 20),
    'chain8':  lambda: chain_test('chain8', ['--power', '-10', '--format', '8'], 20),
}

if __name__ == '__main__':
    for st in sys.argv[1:] or ['off']:
        print('==', st, time.strftime('%H:%M:%S'), flush=True)
        STEPS[st]()
    print('done', flush=True)

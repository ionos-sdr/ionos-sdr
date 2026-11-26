#!/usr/bin/env python3
"""ProBook: RTL-SDR gain sweep for the FG23 vs RTL-SDR ADS-B benchmark (ID1).
Each step: dump1090 (gvanem, --raw) at one tuner gain feeding the ESP's Beast input
via rtl_feed.py; per minute the ESP's per-receiver totals (stats.json sources.total,
rtl.global_cpr) are sampled. The FG23 runs untouched in parallel = traffic reference,
so the figure of merit is RTL/FG23 frames (traffic cancels). Up and down sweep.
At the end rtl_bench.cfg gets the best gain and the normal feed loop is restarted."""
import json, os, re, subprocess, sys, time, urllib.request
H = os.path.expanduser('~')
ESP = 'http://192.168.1.243:8080/data/stats.json'
D = os.path.join(H, 'dump1090')
PY = os.path.join(H, 'radioconda', 'python.exe')
GAINS = ['10.2', '17.1', '24.0', '31.2', '38.2', '45.3', '52.7', 'auto']   # R828D steps (RTL-SDR Blog V4)
STEP_MIN, SKIP_MIN = 8, 1
OUT = os.path.join(H, 'gain_sweep.csv')

def stats():
    j = json.load(urllib.request.urlopen(ESP, timeout=8))
    s = j['sources']
    return s['total']['fg23'], s['total']['rtl'], j.get('rtl', {}).get('global_cpr', 0)

def cfg(g):
    t = open(os.path.join(D, 'rtl_bench.cfg'), encoding='ascii').read()
    t = re.sub(r'(?m)^gain\s*=.*$', 'gain = %s' % g, t)
    p = os.path.join(D, 'sweep_%s.cfg' % g.replace('.', '_'))
    open(p, 'w', encoding='ascii').write(t)
    return p

def kill_feed():
    subprocess.run(['powershell', '-NoProfile', '-Command',
        "Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'pb_rtl_feed|rtl_feed\\.py|dump1090\\.exe' } | % { Stop-Process -Id $_.ProcessId -Force -EA 0 }"])
    time.sleep(3)

def log(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)

new = not os.path.exists(OUT)
f = open(OUT, 'a')
if new: f.write('t,gain,min,d_fg23,d_rtl,d_rtl_gcpr,ratio\n')
res = {}
kill_feed()
for g in GAINS + GAINS[-2::-1]:                      # up, then back down
    p = subprocess.Popen([PY, os.path.join(H, 'aprs_id2', 'rtl_feed.py'), '--esp', '192.168.1.243:30004',
                          '--src', 'dump1090', '--exe', os.path.join(D, 'dump1090.exe'), '--cfg', cfg(g),
                          '--log', os.path.join(H, 'rtl_feed_sweep.log')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log('gain', g)
    time.sleep(SKIP_MIN * 60)
    try: prev = stats()
    except Exception as e: log('stats', e); prev = None
    for m in range(STEP_MIN - SKIP_MIN):
        time.sleep(60)
        try: cur = stats()
        except Exception as e: log('stats', e); continue
        if prev:
            df, dr, dg = cur[0] - prev[0], cur[1] - prev[1], cur[2] - prev[2]
            r = dr / df if df else 0.0
            f.write('%d,%s,%d,%d,%d,%d,%.3f\n' % (time.time(), g, m, df, dr, dg, r)); f.flush()
            res.setdefault(g, []).append(r)
            log('  gain %s min %d fg23 %d rtl %d gcpr %d ratio %.2f' % (g, m, df, dr, dg, r))
        prev = cur
    p.kill(); kill_feed()
best = max(res, key=lambda k: sorted(res[k])[len(res[k]) // 2]) if res else '20.7'
for g in GAINS:
    if g in res: log('median ratio gain %-5s %.2f (n %d)' % (g, sorted(res[g])[len(res[g]) // 2], len(res[g])))
log('BEST', best)
t = open(os.path.join(D, 'rtl_bench.cfg'), encoding='ascii').read()
open(os.path.join(D, 'rtl_bench.cfg'), 'w', encoding='ascii').write(re.sub(r'(?m)^gain\s*=.*$', 'gain = %s' % best, t))
subprocess.run(['powershell', '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File',
                os.path.join(H, 'aprs_id2', 'pb_bg.ps1'), 'pb_rtl_feed.ps1'])
log('feed loop restarted with gain', best)

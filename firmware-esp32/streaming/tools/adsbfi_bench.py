"""adsbfi_bench.py - benchmark the ionos-mini FG23 ADS-B receiver against adsb.fi.

Runs on a machine in the same LAN as the ESP (the home ProBook). Every SAMPLE_S
seconds it fetches
  * adsb.fi open data: aircraft in a 250 NM circle around the receiver
    (https://opendata.adsb.fi/api/v3/lat/{lat}/lon/{lon}/dist/{nm}; 1 req/s max,
    personal non-commercial use, cite adsb.fi), and
  * the receiver's own data/aircraft.json,
then once a minute it averages the samples, appends a CSV line and POSTs the
summary to the ESP (data/bench), which the /graphs page draws.

Per sample: ref = adsb.fi aircraft with a position not older than MAX_AGE_S,
ours = aircraft this receiver heard in the last MAX_AGE_S (any message),
matched = ref & ours (by ICAO), only = ours not in ref. Coverage per distance
band = matched / ref within the band (distance from the receiver).
Per receiver (FG23 on-chip / RTL-SDR, e.g. both behind a splitter): the same
matched and coverage figures from the ESP's 'ionos_rx' field (heard by that
receiver within the last 60 s); these go to the CSV only.

usage: python adsbfi_bench.py [--esp http://192.168.1.243:8080] [--csv adsbfi_bench.csv]
       receiver position: --lat/--lon or env ADSB_RX_LAT / ADSB_RX_LON
"""
import argparse, csv, json, math, os, sys, time, urllib.request

BANDS_KM = [(0, 50), (50, 100), (100, 200), (200, 300), (300, 1e9)]
SAMPLE_S = 15
MAX_AGE_S = 30
NM = 250
UA = 'ionos-mini-adsb-bench/1.0 (personal, non-commercial)'


def get_json(url, timeout=10):
    req = urllib.request.Request(url, headers={'User-Agent': UA, 'Accept': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode('utf-8'))


def dist_km(lat1, lon1, lat2, lon2):
    k = math.pi / 180
    a = math.sin((lat2 - lat1) * k / 2) ** 2 + math.cos(lat1 * k) * math.cos(lat2 * k) * math.sin((lon2 - lon1) * k / 2) ** 2
    return 2 * 6371.0 * math.asin(math.sqrt(a))


def sample(a, ref_url):
    ref = get_json(ref_url)
    ours = get_json(a.esp + '/data/aircraft.json', timeout=5)
    refpos = {}
    for x in ref.get('ac') or ref.get('aircraft') or []:
        if 'lat' not in x or 'lon' not in x or x.get('alt_baro') == 'ground':
            continue
        if float(x.get('seen_pos', 0)) > MAX_AGE_S:
            continue
        refpos[x['hex'].lower().lstrip('~')] = dist_km(a.lat, a.lon, x['lat'], x['lon'])
    seen = {x['hex'].lower() for x in ours.get('aircraft', []) if float(x.get('seen', 99)) <= MAX_AGE_S and x.get('ionos_rx', 'fg23') in ('fg23', 'both')}   # 'ours' = the FG23 (the RTL-SDR has its own entries since 48bf193)
    rx = {'fg23': set(), 'rtl': set()}
    for x in ours.get('aircraft', []):
        r = x.get('ionos_rx')
        if r in ('fg23', 'both'): rx['fg23'].add(x['hex'].lower())
        if r in ('rtl', 'both'): rx['rtl'].add(x['hex'].lower())
    matched = seen & set(refpos)
    s = {'ref': len(refpos), 'ours': len(seen), 'matched': len(matched), 'only': len(seen - set(refpos)),
         'maxkm': max((refpos[h] for h in matched), default=0.0)}
    for i, (lo, hi) in enumerate(BANDS_KM):
        inb = [h for h, d in refpos.items() if lo <= d < hi]
        s['n%d' % i] = len(inb)
        s['m%d' % i] = sum(1 for h in inb if h in seen)
        for k in rx: s['%s_m%d' % (k, i)] = sum(1 for h in inb if h in rx[k])
    for k in rx: s[k + '_matched'] = len(rx[k] & set(refpos)); s[k + '_seen'] = len(rx[k])
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--esp', default='http://192.168.1.243:8080')
    ap.add_argument('--lat', type=float, default=float(os.environ.get('ADSB_RX_LAT', '47.4967')))
    ap.add_argument('--lon', type=float, default=float(os.environ.get('ADSB_RX_LON', '19.0073')))
    ap.add_argument('--csv', default='adsbfi_bench.csv')
    ap.add_argument('--hours', type=float, default=0, help='stop after this many hours (0 = forever)')
    a = ap.parse_args()
    ref_url = 'https://opendata.adsb.fi/api/v3/lat/%.4f/lon/%.4f/dist/%d' % (a.lat, a.lon, NM)
    new = not os.path.exists(a.csv)
    f = open(a.csv, 'a', newline='')
    w = csv.writer(f)
    if new:
        w.writerow(['t', 'ref', 'ours', 'matched', 'only_ours', 'cov0', 'cov1', 'cov2', 'cov3', 'cov4', 'max_km', 'samples']
                   + ['%s_%s' % (k, c) for k in ('fg23', 'rtl') for c in ('seen', 'matched', 'cov0', 'cov1', 'cov2', 'cov3', 'cov4')])
    t_end = time.time() + a.hours * 3600 if a.hours > 0 else None
    print('adsb.fi benchmark:', ref_url, '->', a.esp, flush=True)
    buf, minute = [], int(time.time() // 60)
    while t_end is None or time.time() < t_end:
        t0 = time.time()
        try:
            buf.append(sample(a, ref_url))
        except Exception as e:
            print(time.strftime('%H:%M:%S'), 'sample failed:', str(e)[:120], flush=True)
        m = int(time.time() // 60)
        if m != minute and buf:
            avg = lambda k: sum(x[k] for x in buf) / len(buf)
            cov = []
            for i in range(len(BANDS_KM)):
                n, mm = sum(x['n%d' % i] for x in buf), sum(x['m%d' % i] for x in buf)
                cov.append(255 if n == 0 else round(100.0 * mm / n))
            row = [minute * 60 + 60, round(avg('ref')), round(avg('ours')), round(avg('matched')), round(avg('only')),
                   *cov, round(max(x['maxkm'] for x in buf)), len(buf)]
            for k in ('fg23', 'rtl'):
                row += [round(avg(k + '_seen')), round(avg(k + '_matched'))]
                for i in range(len(BANDS_KM)):
                    n, mm = sum(x['n%d' % i] for x in buf), sum(x['%s_m%d' % (k, i)] for x in buf)
                    row.append(255 if n == 0 else round(100.0 * mm / n))
            w.writerow(row); f.flush()
            body = ','.join(str(v) for v in row[:11]).encode()
            try:
                req = urllib.request.Request(a.esp + '/data/bench', data=body, method='POST',
                                             headers={'Content-Type': 'text/plain', 'User-Agent': UA})
                urllib.request.urlopen(req, timeout=5).read()
                posted = 'posted'
            except Exception as e:
                posted = 'POST failed: ' + str(e)[:60]
            print(time.strftime('%H:%M:%S'), 'ref %d ours %d both %d only %d cov %s max %d km | fg23 %d/%d rtl %d/%d (%s)'
                  % (row[1], row[2], row[3], row[4], row[5:10], row[10], row[13], row[12], row[20], row[19], posted), flush=True)
            buf, minute = [], m
        time.sleep(max(1.0, SAMPLE_S - (time.time() - t0)))


if __name__ == '__main__':
    main()

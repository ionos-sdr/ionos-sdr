#!/usr/bin/env python3
"""ais_web.py — guest AIS map for the ionos-mini FG23 receiver.

    Copyright (c) 2026 Zoltan Doczi HA7DCD — MIT

  Reads the receiver's raw !AIVDM stream (TCP 10110), decodes it, and serves a
  token-protected page: Leaflet map + vessel list + packet log.

  ACCESS: http://<host>:<port>/?t=<token>   (the token is in TOKEN_FILE)
    - expires at EXPIRES (UTC, ISO) — after that every request gets 403
    - revoke now:   echo new-random > ais_web.token   (re-read on every request)
                    or stop the process

  usage: ais_web.py [--src 192.168.1.243:10110] [--port 8095] [--days 7]
"""
import socket, threading, time, json, argparse, os, secrets, html, datetime as dt
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
TOKEN_FILE = os.path.join(HERE, 'ais_web.token')
EXP_FILE = os.path.join(HERE, 'ais_web.expires')
LOCK = threading.Lock()
VESSELS = {}            # mmsi -> dict
LOG = []                # last packets
STATS = {'start': time.time(), 'nmea': 0, 'bad': 0, 'conn': 0, 'last_rx': 0, 'types': {}}

# ---------------- AIS decoding ----------------
def bits_of(payload, fill):
    b = []
    for c in payload:
        v = ord(c) - 48
        if v > 40: v -= 8
        b.append(format(v, '06b'))
    s = ''.join(b)
    return s[:len(s) - fill] if fill else s

def u(b, a, n): return int(b[a:a + n], 2) if len(b) >= a + n else None
def s_(b, a, n):
    v = u(b, a, n)
    if v is None: return None
    return v - (1 << n) if v & (1 << (n - 1)) else v
def txt(b, a, n):
    if len(b) < a + n: return ''
    t = ''
    for k in range(a, a + n, 6):
        v = int(b[k:k + 6], 2); t += chr(v + 64 if v < 32 else v)
    return t.replace('@', '').strip()

def pos(lon, lat):
    if lon is None or lat is None: return None, None
    lon /= 600000.0; lat /= 600000.0
    if abs(lon) > 180 or abs(lat) > 90: return None, None
    return round(lat, 6), round(lon, 6)

SHIPTYPE = {30: 'fishing', 31: 'towing', 32: 'towing', 33: 'dredging', 35: 'military', 36: 'sailing',
            37: 'pleasure', 50: 'pilot', 51: 'SAR', 52: 'tug', 53: 'port tender', 55: 'law enf.',
            58: 'medical'}
def shiptype(v):
    if v is None: return ''
    if v in SHIPTYPE: return SHIPTYPE[v]
    return {6: 'passenger', 7: 'cargo', 8: 'tanker', 4: 'HSC', 2: 'WIG', 9: 'other'}.get(v // 10, str(v))

def decode(b):
    t = u(b, 0, 6); m = u(b, 8, 30); d = {'type': t, 'mmsi': m}
    if t in (1, 2, 3):
        d['lat'], d['lon'] = pos(s_(b, 61, 28), s_(b, 89, 27))
        sog = u(b, 50, 10); cog = u(b, 116, 12); hdg = u(b, 128, 9)
        d['sog'] = None if sog == 1023 else sog / 10.0
        d['cog'] = None if cog == 3600 else cog / 10.0
        d['hdg'] = None if hdg == 511 else hdg
        d['status'] = u(b, 38, 4); d['kind'] = 'ship'
    elif t in (18, 19):
        d['lat'], d['lon'] = pos(s_(b, 57, 28), s_(b, 85, 27))
        sog = u(b, 46, 10); cog = u(b, 112, 12)
        d['sog'] = None if sog == 1023 else sog / 10.0
        d['cog'] = None if cog == 3600 else cog / 10.0
        d['kind'] = 'ship'
        if t == 19: d['name'] = txt(b, 143, 120)
    elif t == 5:
        d['call'] = txt(b, 70, 42); d['name'] = txt(b, 112, 120); d['shiptype'] = shiptype(u(b, 232, 8))
        d['dest'] = txt(b, 302, 120); d['kind'] = 'ship'
    elif t == 24:
        if u(b, 38, 2) == 0: d['name'] = txt(b, 40, 120)
        else: d['call'] = txt(b, 90, 42); d['shiptype'] = shiptype(u(b, 40, 8))
        d['kind'] = 'ship'
    elif t == 21:
        d['name'] = txt(b, 43, 120); d['lat'], d['lon'] = pos(s_(b, 164, 28), s_(b, 192, 27))
        d['virtual'] = u(b, 269, 1); d['kind'] = 'aton'
    elif t in (4, 11):
        d['lat'], d['lon'] = pos(s_(b, 79, 28), s_(b, 107, 27)); d['kind'] = 'base'
    return d

def nmea_ok(line):
    if '*' not in line: return False
    body, cs = line[1:].split('*', 1)
    c = 0
    for ch in body: c ^= ord(ch)
    try: return c == int(cs[:2], 16)
    except ValueError: return False

DBM_K = -52.5   # dBFS -> dBm, rough: status line RF -99 dBm <-> -46.5 dBFS (2026-11-22), uncalibrated

def handle_pions(line):
    # $PIONS,AIS,<chan>,<lvl dBFS>,<snr dB>,<foff Hz>[,<src IQ|HW>,<dBm>] -> attach to the last packet
    f = line.split('*')[0].split(',')
    try: lvl, snr, foff = float(f[3]), float(f[4]), float(f[5])
    except (IndexError, ValueError): return
    src = f[6] if len(f) > 6 else 'IQ'
    try: dbm = float(f[7])
    except (IndexError, ValueError): dbm = round(lvl + DBM_K, 1)
    upd = {'lvl': lvl, 'dbm': dbm, 'snr': snr, 'foff': foff, 'src': src}
    with LOCK:
        if not LOG: return
        p = LOG[-1]
        p.update(upd)
        v = VESSELS.get(p['mmsi'])
        if v: v.update(upd)

def handle(line):
    now = time.time()
    if line.startswith('$PIONS') and nmea_ok(line):
        return handle_pions(line)
    if not line.startswith('!AIVDM') or not nmea_ok(line):
        with LOCK: STATS['bad'] += 1
        return
    f = line.split('*')[0].split(',')
    try:
        chan, payload, fill = f[4], f[5], int(f[6])
    except (IndexError, ValueError):
        return
    b = bits_of(payload, fill)
    d = decode(b)
    with LOCK:
        STATS['nmea'] += 1; STATS['last_rx'] = now
        STATS['types'][str(d['type'])] = STATS['types'].get(str(d['type']), 0) + 1
        v = VESSELS.setdefault(d['mmsi'], {'mmsi': d['mmsi'], 'first': now, 'n': 0})
        v['n'] += 1; v['last'] = now; v['chan'] = chan
        for k in ('kind', 'name', 'call', 'dest', 'shiptype', 'lat', 'lon', 'sog', 'cog', 'hdg', 'virtual'):
            if d.get(k) not in (None, ''): v[k] = d[k]
        if d.get('lat') is not None:
            tr = v.setdefault('track', [])
            if not tr or tr[-1][:2] != [d['lat'], d['lon']]:
                tr.append([d['lat'], d['lon'], int(now)])
                del tr[:-60]
        LOG.append({'t': now, 'chan': chan, 'bits': len(b), 'nmea': line, **{k: d[k] for k in d if k != 'kind'}})
        del LOG[:-400]

def reader(src):
    host, port = src.split(':'); port = int(port)
    while True:
        try:
            s = socket.create_connection((host, port), timeout=10); s.settimeout(120)
            with LOCK: STATS['conn'] += 1
            buf = b''
            while True:
                x = s.recv(4096)
                if not x: raise OSError('closed')
                buf += x
                *lines, buf = buf.split(b'\n')
                for ln in lines:
                    # one bad sentence must not kill the reader (2026-11-23: a
                    # TypeError in a decode stopped it for 3 hours, silently)
                    try:
                        handle(ln.decode('ascii', 'replace').strip())
                    except Exception as e:
                        print('handle error:', repr(e), ln[:120], flush=True)
        except Exception as e:
            print('reader:', repr(e), flush=True)
            time.sleep(5)

# ---------------- web ----------------
def token_ok(q):
    try:
        tok = open(TOKEN_FILE).read().strip()
        exp = dt.datetime.fromisoformat(open(EXP_FILE).read().strip())
    except OSError:
        return False
    if dt.datetime.now(dt.timezone.utc) > exp: return False
    return secrets.compare_digest(q.get('t', [''])[0], tok)

PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ionos-mini AIS</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
:root{--bg:#0f1418;--fg:#e6edf3;--mut:#8b98a5;--acc:#3fb950;--line:#26313a}
body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.4 system-ui,sans-serif}
header{padding:8px 16px;display:flex;gap:16px;align-items:baseline;flex-wrap:wrap;border-bottom:1px solid var(--line)}
header b{font-size:15px} .mut{color:var(--mut)}
#wrap{display:grid;grid-template-columns:minmax(0,3fr) minmax(0,2fr);height:calc(100vh - 42px)}
#map{height:100%} #side{overflow:auto;border-left:1px solid var(--line)}
table{border-collapse:collapse;width:100%} th,td{padding:3px 6px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}
th{position:sticky;top:0;background:#151c22;color:var(--mut);font-weight:500}
h3{margin:10px 8px 4px;font-size:13px;color:var(--mut)} .nm{font-family:ui-monospace,monospace;font-size:11px;color:var(--mut)}
tr:hover td{background:#18222b}
@media(max-width:800px){#wrap{grid-template-columns:1fr;grid-template-rows:55vh auto;height:auto}#map{height:55vh}}
</style></head><body>
<header><b>ionos-mini · FG23 AIS receiver</b><span class="mut">channel A 161.975 MHz · Budapest, Danube · level ≈dBm is uncalibrated (dBFS − 52.5)</span>
<span id="st" class="mut"></span></header>
<div id="wrap"><div id="map"></div><div id="side">
<h3>Stations</h3><table id="ves"><thead><tr><th>MMSI</th><th>name</th><th>kind</th><th>SOG</th><th>COG</th><th>level</th><th>SNR</th><th>Δf</th><th>msgs</th><th>last</th></tr></thead><tbody></tbody></table>
<h3>Packets (newest first)</h3><table id="log"><thead><tr><th>time</th><th>type</th><th>MMSI</th><th>bits</th><th>level</th><th>SNR</th><th>Δf</th><th>detail</th></tr></thead><tbody></tbody></table>
</div></div>
<script>
const T=new URLSearchParams(location.search).get('t');
const map=L.map('map').setView([47.50,19.05],12);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:18,attribution:'&copy; OpenStreetMap'}).addTo(map);
const mk={},tr={};let fitted=false;
const col=k=>k==='aton'?'#d29922':k==='base'?'#a371f7':'#3fb950';
const ago=s=>{s=Math.round(Date.now()/1000-s);return s<60?s+'s':s<3600?Math.round(s/60)+'m':Math.round(s/3600)+'h'};
const esc=s=>String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
async function tick(){
 let r; try{r=await fetch('api?t='+encodeURIComponent(T));}catch(e){return}
 if(!r.ok){document.getElementById('st').textContent='access expired or revoked';return}
 const d=await r.json(); const S=d.stats;
 document.getElementById('st').textContent=`${S.nmea} msgs · ${Object.keys(d.vessels).length} stations · last rx ${S.last_rx?ago(S.last_rx):'-'} ago · link valid until ${d.expires}`;
 const vb=document.querySelector('#ves tbody'); vb.innerHTML='';
 const vs=Object.values(d.vessels).sort((a,b)=>b.last-a.last);
 for(const v of vs){
  vb.insertAdjacentHTML('beforeend',`<tr data-m="${v.mmsi}"><td>${v.mmsi}</td><td>${esc(v.name||'')}${v.call?' <span class=mut>'+esc(v.call)+'</span>':''}</td><td>${esc(v.kind||'')}${v.shiptype?' / '+esc(v.shiptype):''}</td><td>${v.sog??''}</td><td>${v.cog??''}</td><td>${v.dbm!=null?'≈'+v.dbm+' dBm':''}</td><td>${v.snr!=null?v.snr+' dB':''}</td><td>${v.foff!=null?(v.foff>0?'+':'')+v.foff+' Hz':''}</td><td>${v.n}</td><td>${ago(v.last)}</td></tr>`);
  if(v.lat==null) continue;
  const lbl=`<b>${esc(v.name||v.mmsi)}</b><br>MMSI ${v.mmsi}<br>${esc(v.kind||'')}${v.shiptype?' / '+esc(v.shiptype):''}${v.dest?'<br>→ '+esc(v.dest):''}${v.sog!=null?'<br>SOG '+v.sog+' kn, COG '+v.cog:''}<br>${v.dbm!=null?'<br>≈'+v.dbm+' dBm, SNR '+v.snr+' dB, Δf '+v.foff+' Hz':''}<br>${v.n} msgs, last ${ago(v.last)} ago`;
  if(!mk[v.mmsi]) mk[v.mmsi]=L.circleMarker([v.lat,v.lon],{radius:v.kind==='aton'?4:7,color:col(v.kind),weight:2,fillOpacity:.7}).addTo(map);
  mk[v.mmsi].setLatLng([v.lat,v.lon]).bindPopup(lbl);
  if(v.track&&v.track.length>1){ if(!tr[v.mmsi]) tr[v.mmsi]=L.polyline([],{color:col(v.kind),weight:2,opacity:.6}).addTo(map); tr[v.mmsi].setLatLngs(v.track.map(p=>[p[0],p[1]])); }
 }
 if(!fitted){const pts=vs.filter(v=>v.lat!=null).map(v=>[v.lat,v.lon]); if(pts.length>1){map.fitBounds(pts,{padding:[20,20]});fitted=true;}}
 const lb=document.querySelector('#log tbody'); lb.innerHTML='';
 for(const p of d.log.slice().reverse().slice(0,150)){
  const det=[p.name&&esc(p.name),p.lat!=null&&`${p.lat.toFixed(5)}, ${p.lon.toFixed(5)}`,p.sog!=null&&`${p.sog} kn`].filter(Boolean).join(' · ');
  lb.insertAdjacentHTML('beforeend',`<tr title="${esc(p.nmea)}"><td>${new Date(p.t*1000).toLocaleTimeString()}</td><td>${p.type}</td><td>${p.mmsi}</td><td>${p.bits}</td><td>${p.dbm!=null?'≈'+p.dbm+' dBm':''}</td><td>${p.snr!=null?p.snr+' dB':''}</td><td>${p.foff!=null?(p.foff>0?'+':'')+p.foff+' Hz':''}</td><td>${det}<div class=nm>${esc(p.nmea)}</div></td></tr>`);
 }
}
tick(); setInterval(tick,5000);
document.querySelector('#ves tbody').addEventListener('click',e=>{const m=e.target.closest('tr')?.dataset.m; if(m&&mk[m]){map.setView(mk[m].getLatLng(),15);mk[m].openPopup();}});
</script></body></html>"""

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def send(self, code, body, ctype):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code); self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store'); self.send_header('Content-Length', str(len(b)))
        self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        u_ = urlparse(self.path); q = parse_qs(u_.query)
        if not token_ok(q):
            return self.send(403, 'access expired or revoked\n', 'text/plain')
        if u_.path in ('/', '/index.html'):
            return self.send(200, PAGE, 'text/html; charset=utf-8')
        if u_.path == '/api':
            with LOCK:
                body = json.dumps({'stats': STATS, 'vessels': {str(k): v for k, v in VESSELS.items()},
                                   'log': LOG[-150:], 'expires': open(EXP_FILE).read().strip()[:16].replace('T', ' ') + ' UTC'})
            return self.send(200, body, 'application/json')
        self.send(404, 'not found\n', 'text/plain')

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default='192.168.1.243:10110'); ap.add_argument('--port', type=int, default=8095)
    ap.add_argument('--bind', default='127.0.0.1'); ap.add_argument('--days', type=float, default=7)
    ap.add_argument('--new-token', action='store_true')
    a = ap.parse_args()
    if a.new_token or not os.path.exists(TOKEN_FILE):
        open(TOKEN_FILE, 'w').write(secrets.token_urlsafe(18))
        exp = dt.datetime.now(dt.timezone.utc) + dt.timedelta(days=a.days)
        open(EXP_FILE, 'w').write(exp.isoformat(timespec='seconds'))
    threading.Thread(target=reader, args=(a.src,), daemon=True).start()
    print('token', open(TOKEN_FILE).read().strip(), 'expires', open(EXP_FILE).read().strip(), flush=True)
    ThreadingHTTPServer((a.bind, a.port), H).serve_forever()

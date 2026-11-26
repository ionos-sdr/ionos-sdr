#!/usr/bin/env python3
"""tftemu_map.py - ionos-mini TFT emulator for the on-chip ADS-B view (tft.cpp map_render +
header with the radio_sm state). Input: the ESP's data/aircraft.json?rx=fg23, the station
position, the base-map levels (quant_<half_km>.png = map_data.h). Output: 240x320 PNG (x3).
Text uses a 6x8 monospace stand-in for the LovyanGFX GLCD font."""
import json, math, sys, time
from PIL import Image, ImageDraw, ImageFont
W, H, HDR = 240, 320, 50; VSA = H - HDR
LAT0, LON0 = 47.50, 19.05; LEVELS = [60, 150, 320]
ac = json.load(open(sys.argv[1], encoding='utf-8-sig')); ref = json.load(open(sys.argv[2], encoding='utf-8-sig'))
ip = sys.argv[3] if len(sys.argv) > 3 else '192.168.1.243'; out = sys.argv[4] if len(sys.argv) > 4 else 'tft_snapshot.png'
F = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 9)
def text(d, x, y, s, c, halo=None):
    if halo:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx or dy: d.text((x + dx, y - 1 + dy), s, font=F, fill=halo)
    d.text((x, y - 1), s, font=F, fill=c)
def altcol(a):
    if a is None or isinstance(a, str): return (128, 128, 128)
    t = min(max(a / 40000, 0), 1); h = 30 + t * 270; s, l = .85, .45
    c = (1 - abs(2 * l - 1)) * s; hp = h / 60; x = c * (1 - abs(hp % 2 - 1)); m = l - c / 2
    r, g, b = [(c, x, 0), (x, c, 0), (0, c, x), (0, x, c), (x, 0, c), (c, 0, x)][min(int(hp), 5)]
    return tuple(int((v + m) * 255) for v in (r, g, b))
now = ac['now']; planes = [a for a in ac['aircraft'] if a.get('lat') is not None and a.get('seen_pos', 99) < 60]
kx = 111.32 * math.cos(math.radians(LAT0))
ext = sorted(max(abs((a['lon'] - LON0) * kx), abs((a['lat'] - LAT0) * 110.574) * 240 / 270) for a in planes)
need = ext[min(int(len(ext) * .85), len(ext) - 1)] if ext else 150
lv = next((i for i, k in enumerate(LEVELS) if need <= k), len(LEVELS) - 1); half = LEVELS[lv]; kmpp = 2 * half / W
img = Image.new('RGB', (W, H), (0, 0, 0)); d = ImageDraw.Draw(img)
# ---- header (tft_show + tft_set_state) ----
text(d, W - 6 * len(ip) - 2, 2, ip, (255, 255, 255))
text(d, 150, 11, 'ADSB>ADSB', (0, 255, 255))
st = json.load(open('stats.json', encoding='utf-8-sig')) if len(sys.argv) < 6 else {}
for x, lab, val, col in ((6, 'FG23', 'ADS-B chip', (255, 165, 0)), (78, 'SIGNAL', '%d dBm' % round(st.get('last1min', {}).get('local', {}).get('noise', -116) or -116), (255, 255, 0)),
                         (144, 'FREE', '4336 k', (255, 255, 0)), (198, 'LOAD', '11%', (255, 255, 0))):
    text(d, x, 20, lab, (255, 255, 255)); text(d, x, 30, val, col)
line = '1090.000 MHz  Mode S on-chip  AM thr 10'; text(d, (W - 6 * len(line)) // 2, 40, line, (255, 165, 0))
# ---- body: base map ----
body = Image.open('quant_%d.png' % half).convert('RGB'); bd = ImageDraw.Draw(body)
X = lambda lon: (lon - LON0) * kx / kmpp + W / 2; Y = lambda lat: (LAT0 - lat) * 110.574 / kmpp + VSA / 2
sx, sy = X(ref['lon']), Y(ref['lat'])
bd.ellipse((sx - 5, sy - 5, sx + 5, sy + 5), outline=(255, 255, 255)); bd.ellipse((sx - 4, sy - 4, sx + 4, sy + 4), outline=(255, 0, 0)); bd.ellipse((sx - 1, sy - 1, sx + 1, sy + 1), fill=(255, 0, 0))
rects = []
for a in sorted(planes, key=lambda a: a.get('seen', 99)):
    px, py = X(a['lon']), Y(a['lat'])
    if px < -6 or px > W + 6 or py < 10 or py > VSA - 34: continue
    t = math.radians(a.get('track') or 0); sn, cs = math.sin(t), math.cos(t)
    R = lambda dx, dy: (px + dx * cs - dy * sn, py + dx * sn + dy * cs)
    tri = [R(0, -7), R(-4.5, 5), R(4.5, 5)]; c = altcol(a.get('alt_baro'))
    bd.polygon(tri, fill=c, outline=(0, 0, 0))
    lab = (a.get('flight') or a['hex'].upper()).strip()[:8]; lw = 6 * len(lab); lx, ly = int(px) + 7, int(py) - 4
    if lx + lw > W: lx = int(px) - 7 - lw
    if ly < 11 or ly + 8 > VSA - 34 or any(not (lx + lw < r[0] or lx > r[2] or ly + 8 < r[1] or ly > r[3]) for r in rects): continue
    rects.append((lx, ly, lx + lw, ly + 8)); text(bd, lx, ly, lab, (255, 255, 255), halo=(0, 0, 0))
km = 20 if half <= 60 else 50 if half <= 150 else 100; bl = round(km / kmpp); by = VSA - 40
bd.rectangle((6, by, 6 + bl, by + 2), fill=(0, 0, 0)); bd.line((6, by - 2, 6, by + 4), fill=(0, 0, 0)); bd.line((6 + bl, by - 2, 6 + bl, by + 4), fill=(0, 0, 0))
text(bd, 10 + bl, by - 3, '%d km' % km, (0, 0, 0))
def dim(y0, y1):
    for y in range(max(0, y0), y1):
        for x in range(W):
            r, g, b = body.getpixel((x, y)); body.putpixel((x, y), (r // 2, g // 2, b // 2))
dim(0, 11); text(bd, 2, 2, 'FG23 1090 MHz  ac %d  pos %d' % (len(ac['aircraft']), len(planes)), (255, 255, 255))
recent = sorted(ac['aircraft'], key=lambda a: a.get('seen', 99))[:3]; yb = VSA - 1 - 10 * len(recent)
dim(yb - 1, VSA)
for i, a in enumerate(recent):
    tt = time.strftime('%H:%M:%S', time.gmtime(now - a.get('seen', 0)))
    if a.get('lat') is not None:
        dk = math.hypot((a['lon'] - ref['lon']) * kx, (a['lat'] - ref['lat']) * 110.574)
        par = ('FL%03d %3.0fkm' % (round(a['alt_baro'] / 100), dk)) if isinstance(a.get('alt_baro'), (int, float)) else '%3.0fkm' % dk
    else:
        par = (a.get('flight') or '').strip() or 'DF11'
    text(bd, 2, yb + 1 + 10 * i, '%s %s %s' % (tt, a['hex'].upper(), par), (255, 255, 255))
img.paste(body, (0, HDR))
big = img.resize((W * 3, H * 3), Image.NEAREST)
frame = Image.new('RGB', (W * 3 + 40, H * 3 + 40), (20, 20, 22)); frame.paste(big, (20, 20)); frame.save(out)
print('level ±%d km, %d aircraft with position, %d total' % (half, len(planes), len(ac['aircraft'])))

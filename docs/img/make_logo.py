#!/usr/bin/env python3
"""Render the Ionos SDR wordmark: a rising red sun over a warm halftone band,
smooth rounded lettering knocked out of the band, with glints and glow.
Outputs banner.png (README header), avatar.png, social_card.png.
Pillow only. Usage: python3 make_logo.py [outdir]
"""
import sys, math, random
from PIL import Image, ImageDraw, ImageFilter, ImageFont, ImageChops

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Black.ttc"
DARK = (20, 18, 18)

def font(size):
    return ImageFont.truetype(FONT, size, index=0)

def text_mask(text, size, rounding=0.10):
    f = font(size)
    x0, y0, x1, y1 = f.getbbox(text)
    pad = size // 4
    img = Image.new("L", (x1 - x0 + 2 * pad, y1 - y0 + 2 * pad), 0)
    ImageDraw.Draw(img).text((pad - x0, pad - y0), text, font=f, fill=255)
    r = max(1, int(size * rounding))
    img = img.filter(ImageFilter.GaussianBlur(r * 0.5)).point(lambda v: 255 if v > 128 else 0)
    return img.crop(img.getbbox())

def radial(w, h, cx, cy, r_in, r_out, col_in, col_out, power=1.0):
    """Radial gradient (RGB) + alpha mask (L), 1 = inside."""
    img = Image.new("RGB", (w, h), col_out)
    mask = Image.new("L", (w, h), 0)
    px = img.load(); pm = mask.load()
    for j in range(h):
        for i in range(w):
            d = math.hypot(i - cx, j - cy)
            t = (d - r_in) / max(1.0, (r_out - r_in))
            t = min(1.0, max(0.0, t)) ** power
            px[i, j] = tuple(int(a + (b - a) * t) for a, b in zip(col_in, col_out))
            pm[i, j] = int(255 * (1 - t))
    return img, mask

def sky(w, h):
    img = Image.new("RGB", (w, h), DARK)
    d = ImageDraw.Draw(img)
    for y in range(h):
        t = y / h
        d.line([(0, y), (w, y)], fill=(int(20 + 40 * t ** 2), int(18 + 14 * t ** 2), int(18 + 8 * t ** 2)))
    return img

def sun(img, cx, cy, r):
    w, h = img.size
    glow, gm = radial(w, h, cx, cy, r * 0.6, r * 3.2, (255, 90, 40), (0, 0, 0), power=0.9)
    img = Image.composite(ImageChops.screen(img, glow), img, gm.point(lambda v: int(v * 0.55)))
    disc, _ = radial(w, h, cx, cy - r * 0.15, 0, r, (255, 120, 60), (220, 30, 20), power=1.4)
    edge = Image.new("L", (w, h), 0)
    ImageDraw.Draw(edge).ellipse([cx - r, cy - r, cx + r, cy + r], fill=255)
    edge = edge.filter(ImageFilter.GaussianBlur(r * 0.03))
    return Image.composite(disc, img, edge)

def band(img, y0, y1, seed=3):
    w, h = img.size
    rnd = random.Random(seed)
    layer = Image.new("RGB", (w, h), (0, 0, 0))
    d = ImageDraw.Draw(layer)
    core = (y0 + y1) / 2; half = (y1 - y0) / 2
    for y in range(y0, y1):
        t = max(0.0, 1 - abs((y - core) / half))
        if t > 0.78:
            k = (t - 0.78) / 0.22
            col = (255, int(160 + 90 * k), int(70 + 170 * k))
        else:
            k = t / 0.78
            col = (int(130 + 125 * k), int(35 + 125 * k), int(10 + 60 * k))
        d.line([(0, y), (w, y)], fill=col)
    cell = 7; row = 0
    for y in range(y0 - cell, y1 + cell, cell):
        row += 1
        for x in range(-cell, w + cell, cell):
            t = max(0.0, min(1.0, 1 - abs((y + cell / 2 - core) / half)))
            r = cell * (0.55 - 0.52 * t ** 0.7)
            if r < 0.5:
                continue
            cx = x + cell / 2 + (cell / 2 if row % 2 else 0)
            d.ellipse([cx - r, y + cell / 2 - r, cx + r, y + cell / 2 + r], fill=(60, 18, 10))
    m = Image.new("L", (w, h), 0)
    ImageDraw.Draw(m).rectangle([0, y0, w, y1], fill=255)
    m = m.filter(ImageFilter.GaussianBlur(half * 0.06))
    img = Image.composite(layer, img, m)
    g = Image.new("RGB", (w, h), (0, 0, 0)); gd = ImageDraw.Draw(g)
    for i in range(7):
        y = core + rnd.uniform(-half * 0.35, half * 0.35)
        x0 = rnd.uniform(-w * 0.1, w * 0.8); ln = rnd.uniform(w * 0.12, w * 0.45)
        wd = max(2, int(half * rnd.uniform(0.03, 0.09)))
        gd.line([(x0, y), (x0 + ln, y)], fill=(255, 235, 200), width=wd)
    g = g.filter(ImageFilter.GaussianBlur(3))
    return ImageChops.screen(img, g.point(lambda v: int(v * 0.85)))

def lettering(img, tm, tx, ty):
    w, h = img.size
    full = Image.new("L", (w, h), 0); full.paste(tm, (tx, ty))
    glow = full.filter(ImageFilter.GaussianBlur(tm.size[1] * 0.12))
    img = ImageChops.screen(img, Image.composite(Image.new("RGB", (w, h), (255, 150, 60)),
                                                 Image.new("RGB", (w, h), (0, 0, 0)), glow.point(lambda v: int(v * 0.5))))
    rim = full.filter(ImageFilter.MaxFilter(5))
    img = Image.composite(Image.new("RGB", (w, h), (255, 244, 225)), img, rim)
    body = Image.new("RGB", (w, h), DARK); bd = ImageDraw.Draw(body)
    for y in range(ty, ty + tm.size[1]):
        t = (y - ty) / tm.size[1]
        v = int(34 - 16 * t)
        bd.line([(0, y), (w, y)], fill=(v, v - 2, v - 2))
    img = Image.composite(body, img, full)
    gloss = Image.new("L", (w, h), 0); gd = ImageDraw.Draw(gloss)
    gy0 = ty + tm.size[1] * 0.12; gy1 = ty + tm.size[1] * 0.42
    gd.polygon([(0, gy0 + 18), (w, gy0 - 18), (w, gy1 - 18), (0, gy1 + 18)], fill=255)
    gloss = gloss.filter(ImageFilter.GaussianBlur(6))
    gloss = ImageChops.multiply(gloss, full).point(lambda v: int(v * 0.32))
    img = Image.composite(Image.new("RGB", (w, h), (255, 230, 200)), img, gloss)
    top = ImageChops.subtract(full, ImageChops.offset(full, 0, 3)).filter(ImageFilter.GaussianBlur(1))
    return Image.composite(Image.new("RGB", (w, h), (255, 250, 240)), img, top.point(lambda v: int(v * 0.9)))

def compose(w, h, size, lines, sun_pos=(0.11, 0.18), sun_r=0.16, band_pad=0.28):
    masks = [text_mask(t, s) for t, s in lines]
    tw = max(m.size[0] for m in masks)
    gap = int(size * 0.12)
    th = sum(m.size[1] for m in masks) + gap * (len(masks) - 1)
    tm = Image.new("L", (tw, th), 0); y = 0
    for m in masks:
        tm.paste(m, ((tw - m.size[0]) // 2, y)); y += m.size[1] + gap
    tx, ty = (w - tw) // 2, (h - th) // 2
    y0 = int(ty - band_pad * th); y1 = int(ty + th + band_pad * th)
    img = sky(w, h)
    img = sun(img, int(w * sun_pos[0]), int(h * sun_pos[1]), int(h * sun_r))
    img = band(img, y0, y1)
    img = lettering(img, tm, tx, ty)
    _, vm = radial(w, h, w / 2, h / 2, min(w, h) * 0.55, max(w, h) * 0.85, (0, 0, 0), (0, 0, 0))
    return Image.composite(img, Image.new("RGB", (w, h), (0, 0, 0)), vm.point(lambda x: 255 - int((255 - x) * 0.45)))

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    compose(1600, 420, 190, [("IonosSDR", 190)], sun_pos=(0.10, 0.20), sun_r=0.17, band_pad=0.28).save(f"{out}/banner.png", optimize=True)
    compose(512, 512, 120, [("Ionos", 120), ("SDR", 84)], sun_pos=(0.20, 0.17), sun_r=0.13, band_pad=0.22).save(f"{out}/avatar.png", optimize=True)
    compose(1280, 640, 170, [("IonosSDR", 170)], sun_pos=(0.12, 0.20), sun_r=0.15, band_pad=0.32).save(f"{out}/social_card.png", optimize=True)
    print("wrote banner.png avatar.png social_card.png")

if __name__ == "__main__":
    main()

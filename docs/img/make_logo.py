#!/usr/bin/env python3
"""Render the Ionos SDR wordmark: retro halftone band with pixel (C64-style)
knock-out lettering. Outputs banner (README header), avatar and a social card.
Pure Pillow, no fonts required.  Usage: python3 make_logo.py [outdir]
"""
import sys, math, random
from PIL import Image, ImageDraw, ImageFilter

# --- 8-row pixel font (rows 0..7; lowercase sits on rows 2..7) ------------
GLYPHS = {
 'I': ["##","##","##","##","##","##","##","##"],
 'i': [".#.","...",".#.",".#.",".#.",".#.",".#.",".#."],
 'o': [".....",".....",".###.","#...#","#...#","#...#","#...#",".###."],
 'n': [".....",".....","####.","#...#","#...#","#...#","#...#","#...#"],
 's': [".....",".....",".####","#....",".###.","....#","....#","####."],
 'S': [".####.","#....#","#.....",".####.",".....#",".....#","#....#",".####."],
 'D': ["####.","#...#","#...#","#...#","#...#","#...#","#...#","####."],
 'R': ["####.","#...#","#...#","####.","#.#..","#..#.","#...#","#...#"],
 ' ': ["..","..","..","..","..","..","..",".."],
}
def norm(g):
    w = max(len(r) for r in g)
    return [r.ljust(w, '.') for r in g]

def text_mask(text, px, gap=1):
    """Return an 'L' image with the pixel text at px pixels per cell."""
    glyphs = [norm(GLYPHS[c]) for c in text]
    cols = sum(len(g[0]) for g in glyphs) + gap * (len(glyphs) - 1)
    img = Image.new("L", (cols * px, 8 * px), 0)
    d = ImageDraw.Draw(img)
    x = 0
    for g in glyphs:
        for r, row in enumerate(g):
            for c, ch in enumerate(row):
                if ch == '#':
                    x0, y0 = (x + c) * px, r * px
                    d.rectangle([x0, y0, x0 + px - 1, y0 + px - 1], fill=255)
        x += len(g[0]) + gap
    # round the corners a little (C64 chunky but soft)
    img = img.filter(ImageFilter.MaxFilter(int(px * 0.45) | 1))
    img = img.filter(ImageFilter.GaussianBlur(px * 0.30)).point(lambda v: 255 if v > 128 else 0)
    return img

def halftone_band(w, h, band_y0, band_y1, seed=3):
    """Orange band with a white-hot core, screened by dark halftone dots that
    grow towards the band edges; dark dotted background outside."""
    rnd = random.Random(seed)
    bg = Image.new("RGB", (w, h), (22, 22, 22))
    d = ImageDraw.Draw(bg)
    step = 7
    for y in range(0, h, step):
        for x in range(0, w, step):
            d.point((x, y), fill=(38, 38, 38))
    core = (band_y0 + band_y1) / 2
    half = (band_y1 - band_y0) / 2
    # base gradient: white-hot core -> orange -> dark red at the edges
    for y in range(band_y0, band_y1):
        t = max(0.0, 1 - abs((y - core) / half))
        if t > 0.80:
            k = (t - 0.80) / 0.20
            col = (int(255), int(150 + 100 * k), int(40 + 190 * k))
        else:
            k = t / 0.80
            col = (int(120 + 135 * k), int(30 + 120 * k), int(8 + 32 * k))
        d.line([(0, y), (w, y)], fill=col)
    # horizontal light streaks in the core
    for i in range(6):
        y = core + rnd.uniform(-half * 0.18, half * 0.18)
        x0 = rnd.uniform(0, w * 0.6); ln = rnd.uniform(w * 0.15, w * 0.5)
        d.line([(x0, y), (x0 + ln, y)], fill=(255, 250, 240), width=int(half * 0.06) + 1)
    # dark halftone screen: dot radius grows with distance from the core
    cell = 10
    row = 0
    for y in range(band_y0 - cell, band_y1 + cell, cell):
        row += 1
        for x in range(-cell, w + cell, cell):
            t = max(0.0, min(1.0, 1 - abs((y + cell / 2 - core) / half)))
            r = cell * (0.62 - 0.55 * t ** 0.8)
            if r < 0.6:
                continue
            cx = x + cell / 2 + (cell / 2 if row % 2 else 0)
            cy = y + cell / 2
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(22, 22, 22))
    return bg

def stacked_mask(lines, px, vgap=1):
    masks = [text_mask(t, p) for t, p in lines]
    w = max(m.size[0] for m in masks)
    h = sum(m.size[1] for m in masks) + vgap * px * (len(masks) - 1)
    out = Image.new("L", (w, h), 0)
    y = 0
    for m in masks:
        out.paste(m, ((w - m.size[0]) // 2, y)); y += m.size[1] + vgap * px
    return out

def compose(w, h, text, px, band_pad=0.22, glow=True, lines=None):
    tm = stacked_mask(lines, px) if lines else text_mask(text, px)
    tw, th = tm.size
    band_y0 = int(h / 2 - th / 2 - band_pad * th)
    band_y1 = int(h / 2 + th / 2 + band_pad * th)
    img = halftone_band(w, h, band_y0, band_y1)
    # knock-out text: dark fill with a light rim (pixel outline)
    tx, ty = (w - tw) // 2, (h - th) // 2
    rim = tm.filter(ImageFilter.MaxFilter(int(px * 0.22) | 1))
    dark = Image.new("RGB", (w, h), (18, 18, 18))
    light = Image.new("RGB", (w, h), (255, 246, 232))
    layer = Image.new("L", (w, h), 0); layer.paste(rim, (tx, ty))
    img = Image.composite(light, img, layer)
    layer = Image.new("L", (w, h), 0); layer.paste(tm, (tx, ty))
    img = Image.composite(dark, img, layer)
    # inner halftone texture inside the letters (subtle)
    d = ImageDraw.Draw(img)
    for y in range(ty, ty + th, 6):
        for x in range(tx, tx + tw, 6):
            if tm.getpixel((x - tx, y - ty)) > 128:
                d.point((x, y), fill=(42, 42, 42))
    if glow:
        g = Image.new("RGB", (w, h), (0, 0, 0))
        gd = ImageDraw.Draw(g)
        gd.rectangle([0, band_y0, w, band_y1], fill=(255, 120, 30))
        g = g.filter(ImageFilter.GaussianBlur(px * 1.2))
        img = Image.blend(img, Image.composite(g, img, Image.new("L", (w, h), 60)), 0.35)
    return img

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    banner = compose(1600, 420, "IonosSDR", px=21, band_pad=0.18)
    banner.save(f"{out}/banner.png", optimize=True)
    avatar = compose(512, 512, "", px=14, band_pad=0.35, lines=[("Ionos", 14), ("SDR", 11)])
    avatar.save(f"{out}/avatar.png", optimize=True)
    card = compose(1280, 640, "IonosSDR", px=17, band_pad=0.3)
    card.save(f"{out}/social_card.png", optimize=True)
    print("wrote banner.png avatar.png social_card.png")

if __name__ == "__main__":
    main()

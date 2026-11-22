#!/usr/bin/env python3
"""make_icon.py — Ionos SDR square icon + social preview in the banner style.
   Font: Arial Rounded MT Bold (the banner's typeface). usage: make_icon.py FONT.ttf outdir"""
import sys, numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageFilter

FONT, OUT = sys.argv[1], sys.argv[2]

def background(W, H, band_y, band_h, sun_xy, sun_r):
    y = np.arange(H)[:, None]; x = np.arange(W)[None, :]
    # dark vertical gradient
    top = np.array([18, 18, 18.]); bot = np.array([44, 28, 22.])
    t = (y / (H - 1))[..., None]
    img = top * (1 - t) + bot * t + np.zeros((H, W, 3))
    # orange band with a bright core line
    d = (y - band_y) / (band_h / 2)
    band = np.clip(1 - d ** 2, 0, 1)[..., None]
    orange = np.array([232, 118, 30.])
    img = img * (1 - 0.85 * band) + orange * 0.85 * band
    core = np.exp(-((y - band_y) / (band_h * 0.07)) ** 2)[..., None]
    img = img * (1 - core) + np.array([255, 236, 210.]) * core
    # halftone dot pattern inside the band (dark dots)
    pitch = max(6, W // 180)
    dots = (((x % pitch) - pitch / 2) ** 2 + ((y % pitch) - pitch / 2) ** 2) < (pitch * 0.32) ** 2
    img = np.where((dots & (band[..., 0] > 0.15))[..., None], img * 0.55, img)
    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), 'RGB')
    # the red sun with a soft glow, partly behind the band top
    glow = Image.new('L', (W, H), 0); g = ImageDraw.Draw(glow)
    cx, cy = sun_xy
    g.ellipse([cx - sun_r * 1.5, cy - sun_r * 1.5, cx + sun_r * 1.5, cy + sun_r * 1.5], fill=120)
    glow = glow.filter(ImageFilter.GaussianBlur(sun_r * 0.6))
    im = Image.composite(Image.new('RGB', (W, H), (200, 70, 30)), im, glow)
    sun = Image.new('L', (W, H), 0); ImageDraw.Draw(sun).ellipse([cx - sun_r, cy - sun_r, cx + sun_r, cy + sun_r], fill=255)
    sun = sun.filter(ImageFilter.GaussianBlur(2))
    sunc = Image.new('RGB', (W, H)); sd = ImageDraw.Draw(sunc)
    for k in range(sun_r, 0, -1):     # radial gradient: orange centre -> red rim
        f = k / sun_r
        sd.ellipse([cx - k, cy - k, cx + k, cy + k], fill=(int(235 - 25 * f), int(120 - 70 * f), int(60 - 30 * f)))
    im = Image.composite(sunc, im, sun)
    # re-lay the band over the sun's lower part so it sits "behind" the band
    return im

def draw_tracked(draw, txt, font, cx, cy, fill, stroke, track):
    """draw text centred on (cx, cy) char by char with extra tracking; round-joined stroke"""
    widths = [draw.textlength(c, font=font) for c in txt]
    total = sum(widths) + track * (len(txt) - 1)
    x = cx - total / 2
    for c, w in zip(txt, widths):
        draw.text((x + w / 2, cy), c, font=font, anchor='mm', fill=fill, stroke_width=stroke, stroke_fill=fill)
        x += w + track

def glossy_text(im, lines, font_px, centers, stroke):
    W, H = im.size
    font = ImageFont.truetype(FONT, font_px)
    body = int(font_px * 0.020)            # rounds the glyph corners like the banner
    rim = max(2, int(font_px * 0.020))     # light grey outline
    track = int(1.2 * (body + rim))
    mask = Image.new('L', (W, H), 0); md = ImageDraw.Draw(mask)
    outl = Image.new('L', (W, H), 0); od = ImageDraw.Draw(outl)
    for txt, (cx, cy) in zip(lines, centers):
        draw_tracked(od, txt, font, cx, cy, 255, body + rim, track)
        draw_tracked(md, txt, font, cx, cy, 255, body, track)
    sh = outl.filter(ImageFilter.GaussianBlur(rim * 2.5))
    im = Image.composite(Image.new('RGB', (W, H), (10, 6, 4)), im, sh.point(lambda v: int(v * 0.75)))
    im = Image.composite(Image.new('RGB', (W, H), (200, 196, 190)), im, outl.filter(ImageFilter.GaussianBlur(0.8)))
    y = np.arange(H)[:, None]
    fill = np.zeros((H, W, 3))
    base = np.array([22, 22, 22.]); hi = np.array([96, 94, 92.])
    for (cx, cy) in centers:
        rel = (y - (cy - font_px * 0.42)) / (font_px * 0.84)
        hl = np.exp(-((rel - 0.20) / 0.15) ** 2)[..., None]
        w = (np.abs(y - cy) < font_px * 0.62)[..., None]
        fill = np.where(w, base * (1 - hl) + hi * hl, fill)
    fim = Image.fromarray(np.clip(np.broadcast_to(fill, (H, W, 3)), 0, 255).astype(np.uint8))
    return Image.composite(fim, im, mask.filter(ImageFilter.GaussianBlur(0.6)))

def icon(S=1024):
    im = background(S, S, band_y=S * 0.50, band_h=S * 0.62, sun_xy=(S * 0.20, S * 0.14), sun_r=int(S * 0.085))
    fp = int(S * 0.245)
    im = glossy_text(im, ['Ionos', 'SDR'], fp, [(S / 2, S * 0.385), (S / 2, S * 0.655)], stroke=max(3, S // 170))
    # rounded-square mask edge is left to the platform (GitHub crops avatars itself)
    return im

def social(W=1280, H=640):
    im = background(W, H, band_y=H * 0.52, band_h=H * 0.52, sun_xy=(W * 0.10, H * 0.17), sun_r=int(H * 0.11))
    fp = int(H * 0.26)
    im = glossy_text(im, ['Ionos', 'SDR'], fp, [(W / 2, H * 0.385), (W / 2, H * 0.665)], stroke=max(3, H // 150))
    return im

ic = icon(1024); ic.save(f'{OUT}/ionos_icon_1024.png')
ic.resize((512, 512), Image.LANCZOS).save(f'{OUT}/ionos_icon_512.png')
ic.resize((128, 128), Image.LANCZOS).save(f'{OUT}/ionos_icon_128.png')
social().save(f'{OUT}/ionos_social_1280x640.png')
print('ok')

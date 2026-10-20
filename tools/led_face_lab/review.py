"""Contact sheets for visual review (LED look, gamma-mapped)."""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

import rear as R
import states as S
from face import FPS, Frame

OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path('review')
OUT.mkdir(exist_ok=True, parents=True)
P = 10


def disp(level):
    return (255 * (np.clip(np.asarray(level), 0, 25) / 25.0) ** (1 / 2.2)).astype(int)


def draw_panel(d, x0, y0, arr):
    h, w, _ = arr.shape
    for y in range(h):
        for x in range(w):
            L = arr[y, x]
            cx, cy = x0 + x * P, y0 + y * P
            if L.max() < 0.5:
                d.ellipse([cx + 3, cy + 3, cx + P - 4, cy + P - 4], fill=(24, 24, 28))
            else:
                c = tuple(int(v) for v in disp(L))
                d.ellipse([cx + 1, cy + 1, cx + P - 2, cy + P - 2], fill=c)


def tile(f: Frame, with_rear=True):
    W = 8 * P + 12 + 25 * P + 12 + 8 * P + 20 + (16 * P + 6 if with_rear else 0)
    H = 8 * P + 8
    img = Image.new('RGB', (W, H), (6, 6, 8))
    d = ImageDraw.Draw(img)
    draw_panel(d, 4, 4, f.eye_l)
    draw_panel(d, 4 + 8 * P + 12, 4 + (3 * P) // 2, f.mouth)
    draw_panel(d, 4 + 8 * P + 12 + 25 * P + 12, 4, f.eye_r)
    if with_rear:
        x = 4 + 8 * P + 12 + 25 * P + 12 + 8 * P + 16
        d.line([x - 8, 0, x - 8, H], fill=(40, 40, 50))
        draw_panel(d, x, 4, f.rear_l)
        draw_panel(d, x + 8 * P + 6, 4, f.rear_r)
    return img


def sheet(name, frames, n=12, cols=4):
    idx = np.linspace(0, len(frames) - 1, n).astype(int)
    tiles = [tile(frames[i]) for i in idx]
    tw, th = tiles[0].size
    rows = (n + cols - 1) // cols
    sh = Image.new('RGB', (cols * (tw + 6) + 6, rows * (th + 18) + 22), (28, 28, 32))
    d = ImageDraw.Draw(sh)
    d.text((6, 4), name, fill=(230, 230, 230))
    for k, (i, t) in enumerate(zip(idx, tiles)):
        x = 6 + (k % cols) * (tw + 6)
        y = 22 + (k // cols) * (th + 18)
        d.text((x, y), f't={i / FPS:.2f}s', fill=(150, 150, 160))
        sh.paste(t, (x, y + 14))
    sh.save(OUT / f'{name}.png')


def burst(name, frames, start, count, cols=6):
    tiles = [tile(frames[i]) for i in range(start, min(start + count, len(frames)))]
    tw, th = tiles[0].size
    rows = (len(tiles) + cols - 1) // cols
    sh = Image.new('RGB', (cols * (tw + 4) + 4, rows * (th + 16) + 4), (28, 28, 32))
    d = ImageDraw.Draw(sh)
    for k, t in enumerate(tiles):
        x = 4 + (k % cols) * (tw + 4)
        y = 4 + (k // cols) * (th + 16)
        d.text((x, y), f'#{start + k}', fill=(150, 150, 160))
        sh.paste(t, (x, y + 12))
    sh.save(OUT / f'{name}.png')


if __name__ == '__main__':
    which = sys.argv[2:] or list(S.CLIPS)
    for name in which:
        if name.startswith('theme:'):
            th = name.split(':', 1)[1]
            fn = R.THEMES[th]
            fr = []
            for i in range(int(6 * FPS)):
                f = Frame()
                f.rear_l, f.rear_r = fn(i / FPS)
                fr.append(f)
            sheet(name.replace(':', '_'), fr)
            continue
        if name.startswith('burst:'):
            _, clip, s, c = name.split(':')
            fr = S.CLIPS[clip][0]()
            burst(f'burst_{clip}_{s}', fr, int(s), int(c))
            continue
        fr = S.CLIPS[name][0]()
        sheet(name, fr)
    print('ok')

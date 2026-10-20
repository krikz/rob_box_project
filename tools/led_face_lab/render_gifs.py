"""GIF-превью анимаций для документации — из манифестов, как их покажет робот.

    python tools/led_face_lab/render_gifs.py src/rob_box_animations/animations docs/assets/animations

Читает манифесты (то, что реально проигрывает плеер), квантует кадр так же,
как драйвер при ``--brightness`` (по умолчанию 25: ``v * 25 // 255``), и рисует
светодиоды точками. Раскладка — взгляд человека напротив робота:
``wheel_front_right | рот | wheel_front_left``, справа задние фары со спины.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

P = 7               # пикселей на светодиод
GIF_NAME = {'police_lights': 'police'}   # исторические имена файлов в docs


def frame_at(panel, t):
    acc = 0
    for f in panel['frames']:
        if acc <= t < acc + f['duration_ms']:
            return f['image']
        acc += f['duration_ms']
    return panel['frames'][-1]['image']


def draw_grid(d, x0, y0, arr, brightness):
    for y in range(arr.shape[0]):
        for x in range(arr.shape[1]):
            q = arr[y, x] * brightness // 255
            cx, cy = x0 + x * P, y0 + y * P
            if q.max() == 0:
                d.ellipse([cx + 2, cy + 2, cx + P - 3, cy + P - 3], fill=(26, 26, 30))
            else:
                c = tuple(int(255 * (v / brightness) ** (1 / 2.2)) for v in q)
                d.ellipse([cx + 1, cy + 1, cx + P - 2, cy + P - 2], fill=c)


def render(anim_root: Path, name: str, brightness: int, step_ms: int, max_ms: int):
    m = yaml.safe_load((anim_root / 'manifests' / f'{name}.yaml').read_text(encoding='utf-8'))
    groups = {p['logical_group']: p for p in m['panels']}
    cache = {}

    def img(path):
        if path not in cache:
            cache[path] = np.asarray(Image.open(anim_root / path).convert('RGB'), dtype=np.int32)
        return cache[path]

    W = (8 + 25 + 8 + 16) * P + 6 * P
    H = 8 * P + 2 * P
    frames = []
    for t in range(0, min(m['duration_ms'], max_ms), step_ms):
        im = Image.new('RGB', (W, H), (8, 8, 10))
        d = ImageDraw.Draw(im)
        x = P
        draw_grid(d, x, P, img(frame_at(groups['wheel_front_right'], t)), brightness)
        x += 8 * P + P
        draw_grid(d, x, P + int(1.5 * P), img(frame_at(groups['main_display'], t)), brightness)
        x += 25 * P + P
        draw_grid(d, x, P, img(frame_at(groups['wheel_front_left'], t)), brightness)
        x += 8 * P + 2 * P
        d.line([x - P, 0, x - P, H], fill=(40, 40, 48))
        draw_grid(d, x, P, img(frame_at(groups['wheel_rear_left'], t)), brightness)
        draw_grid(d, x + 8 * P + P // 2, P, img(frame_at(groups['wheel_rear_right'], t)), brightness)
        frames.append(im.convert('P', palette=Image.ADAPTIVE, colors=64))
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('animations')
    ap.add_argument('out')
    ap.add_argument('--brightness', type=int, default=25)
    ap.add_argument('--step-ms', type=int, default=80)
    ap.add_argument('--max-ms', type=int, default=10000)
    a = ap.parse_args()
    root, out = Path(a.animations), Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for mf in sorted((root / 'manifests').glob('*.yaml')):
        frames = render(root, mf.stem, a.brightness, a.step_ms, a.max_ms)
        path = out / f'{GIF_NAME.get(mf.stem, mf.stem)}.gif'
        frames[0].save(path, save_all=True, append_images=frames[1:], duration=a.step_ms, loop=0, optimize=True)
        print(f'{path.name:20s} {len(frames):4d} frames {path.stat().st_size // 1024:5d} KB')


if __name__ == '__main__':
    main()

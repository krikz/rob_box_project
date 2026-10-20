"""Bake clips for the preview page + per-animation brightness budgets.

Strips are stored at full scale (PWM at brightness 255). The page quantises
them to the chosen brightness the same way the robot will, so the preview
shows exactly as many shades as the LEDs get.

    python tools/led_face_lab/bake_preview.py <out.json>
"""
import base64
import io
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'rob_box_animations'))

from rob_box_animations.face import rear as R  # noqa: E402
from rob_box_animations.face import states as S  # noqa: E402
from rob_box_animations.face.face import FPS, Frame  # noqa: E402
from rob_box_animations.face.scene import clip  # noqa: E402

OUT = Path(sys.argv[1])

MA_PER_CH = 20.0          # WS2812B, one channel at PWM 255 (datasheet)
IDLE_MA = 1.0             # per LED, conservative
N_LEDS = 381
PSU_MA = 6000.0           # 5 V x 30 W buck
MARGIN = 0.75             # keep 25 % headroom on the buck
BUDGET_MA = PSU_MA * MARGIN


def full_scale(arr):
    return np.clip(np.rint(np.asarray(arr) / 25.0 * 255), 0, 255).astype(np.uint8)


def strip(frames, part):
    rows = []
    for f in frames:
        if part == 'face':
            row = np.zeros((8, 41, 3), np.uint8)
            row[:, 0:8] = full_scale(f.eye_l)
            row[0:5, 8:33] = full_scale(f.mouth)
            row[:, 33:41] = full_scale(f.eye_r)
        else:
            row = np.zeros((8, 16, 3), np.uint8)
            row[:, 0:8] = full_scale(f.rear_l)
            row[:, 8:16] = full_scale(f.rear_r)
        rows.append(row)
    img = np.concatenate(rows, axis=0)
    buf = io.BytesIO()
    Image.fromarray(img).save(buf, 'PNG', optimize=True)
    return base64.b64encode(buf.getvalue()).decode(), img


def ma255(img, n, h=8):
    """Dynamic current per frame at brightness 255, mA."""
    return [round(float(img[i * h:(i + 1) * h].astype(float).sum()) / 255.0 * MA_PER_CH, 1) for i in range(n)]


def budget(peak_ma255):
    if peak_ma255 <= 0:
        return 255
    return int(min(255, 255 * (BUDGET_MA - N_LEDS * IDLE_MA) / peak_ma255))


data = {'fps': FPS, 'clips': {}, 'themes': {}, 'old': {},
        'model': {'ma_per_ch': MA_PER_CH, 'idle_ma': IDLE_MA, 'leds': N_LEDS, 'psu_ma': PSU_MA, 'budget_ma': BUDGET_MA}}

for name, spec in S.SCENES.items():
    fr = clip(spec)
    b64, img = strip(fr, 'face')
    d = {'n': len(fr), 'face': b64, 'ma_face': ma255(img, len(fr)), 'group': spec.group, 'title': spec.title,
         'note': spec.note, 'legacy': None}
    if any(f.rear_l.any() or f.rear_r.any() for f in fr):
        rb, rimg = strip(fr, 'rear')
        d['rear'] = rb
        d['ma_rear'] = ma255(rimg, len(fr))
    data['clips'][name] = d
    print('clip', name, len(fr))

for th, fn in R.THEMES.items():
    fr = []
    for i in range(int(10 * FPS)):
        f = Frame()
        f.rear_l, f.rear_r = fn(i / FPS)
        fr.append(f)
    b64, img = strip(fr, 'rear')
    data['themes'][th] = {'n': len(fr), 'rear': b64, 'ma_rear': ma255(img, len(fr))}
    print('theme', th)


OUT.write_text(json.dumps(data, separators=(',', ':')), encoding='utf-8')
print('bytes', OUT.stat().st_size)

# ---- per-animation brightness budget table (for the chat + the page)
heaviest_theme = max(data['themes'], key=lambda k: max(data['themes'][k]['ma_rear']))
print('heaviest rear theme:', heaviest_theme, max(data['themes'][heaviest_theme]['ma_rear']))
rows = []
for name, d in data['clips'].items():
    if 'ma_rear' in d:
        peak = max(a + b for a, b in zip(d['ma_face'], d['ma_rear']))
        peak_hv = peak
    else:
        peak = max(d['ma_face']) + max(data['themes']['default']['ma_rear'])
        peak_hv = max(d['ma_face']) + max(data['themes'][heaviest_theme]['ma_rear'])
    rows.append((name, round(peak), budget(peak), budget(peak_hv)))
print(f'{"clip":16s} {"peak@255 mA":>12s} {"B (taillight)":>14s} {"B (heaviest theme)":>19s}')
for r in sorted(rows, key=lambda r: r[2]):
    print(f'{r[0]:16s} {r[1]:12d} {r[2]:14d} {r[3]:19d}')

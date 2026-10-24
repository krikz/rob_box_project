"""GIF previews of the face scenes for the docs — rendered by ``Scene.frame`` as face_node does.

    python tools/led_face_lab/render_gifs.py docs/assets/animations

Every scene of ``rob_box_animations.face.states.SCENES``, one pass (or
``--max-ms``), quantised like the driver at ``--brightness`` (default 25:
the wire byte ``v`` shows as ``v * 25 // 255``), LEDs drawn as dots. Layout —
a person facing the robot: ``wheel_front_right | mouth | wheel_front_left``,
then the rear lights seen from behind.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'rob_box_animations'))

from rob_box_animations.face import states as S  # noqa: E402
from rob_box_animations.face.face import to_wire  # noqa: E402
from rob_box_animations.face.scene import Scene  # noqa: E402

P = 7               # pixels per LED


def draw_grid(d, x0, y0, arr, brightness):
    q = to_wire(arr).astype(int) * brightness // 255
    for y in range(q.shape[0]):
        for x in range(q.shape[1]):
            cx, cy = x0 + x * P, y0 + y * P
            if q[y, x].max() == 0:
                d.ellipse([cx + 2, cy + 2, cx + P - 3, cy + P - 3], fill=(26, 26, 30))
            else:
                c = tuple(int(255 * (v / brightness) ** (1 / 2.2)) for v in q[y, x])
                d.ellipse([cx + 1, cy + 1, cx + P - 2, cy + P - 2], fill=c)


def render(name: str, brightness: int, step_ms: int, max_ms: int):
    spec = S.SCENES[name]
    scene = Scene(spec)
    W = (8 + 25 + 8 + 16) * P + 6 * P
    H = 8 * P + 2 * P
    frames = []
    for t_ms in range(0, min(int(spec.period * 1000), max_ms), step_ms):
        f = scene.frame(t_ms / 1000.0)
        im = Image.new('RGB', (W, H), (8, 8, 10))
        d = ImageDraw.Draw(im)
        x = P
        draw_grid(d, x, P, f.eye_l, brightness)              # wheel_front_right
        x += 8 * P + P
        draw_grid(d, x, P + int(1.5 * P), f.mouth, brightness)
        x += 25 * P + P
        draw_grid(d, x, P, f.eye_r, brightness)              # wheel_front_left
        x += 8 * P + 2 * P
        d.line([x - P, 0, x - P, H], fill=(40, 40, 48))
        draw_grid(d, x, P, f.rear_l, brightness)
        draw_grid(d, x + 8 * P + P // 2, P, f.rear_r, brightness)
        frames.append(im.convert('P', palette=Image.ADAPTIVE, colors=64))
    return frames


GROUP_TITLES = {
    'idle': 'Покой', 'listening': 'Слушает', 'thinking': 'Думает', 'talking': 'Говорит',
    'emotion': 'Эмоции', 'music': 'Музыка', 'emergency': 'Экстренные', 'driving': 'Езда',
    'system': 'Системные', 'holiday': 'Праздники',
}
GALLERY_MARK = '<!-- gallery: tools/led_face_lab/render_gifs.py --gallery -->'


def gallery(md: Path, gifs: Path) -> None:
    """Rewrite everything below GALLERY_MARK in ``md`` with a table per scene group."""
    head = md.read_text(encoding='utf-8').split(GALLERY_MARK)[0]
    rel = Path('../../../docs/assets/animations') if gifs.as_posix().endswith('docs/assets/animations') else gifs
    lines = [GALLERY_MARK, '']
    by_group = {}
    for name, spec in S.SCENES.items():
        by_group.setdefault(spec.group, []).append((name, spec))
    for group, items in by_group.items():
        lines += [f'### {GROUP_TITLES.get(group, group)}', '', '| Сцена | Что происходит | Превью |', '|---|---|---|']
        lines += [f'| `{n}` | {s.title}: {s.note} | ![]({(rel / f"{n}.gif").as_posix()}) |' for n, s in items]
        lines.append('')
    md.write_text(head + '\n'.join(lines), encoding='utf-8', newline='\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--brightness', type=int, default=25)
    ap.add_argument('--step-ms', type=int, default=80)
    ap.add_argument('--max-ms', type=int, default=10000)
    ap.add_argument('--gallery', type=Path, help='markdown file whose gallery section to regenerate')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for name in S.SCENES:
        frames = render(name, a.brightness, a.step_ms, a.max_ms)
        path = out / f'{name}.gif'
        frames[0].save(path, save_all=True, append_images=frames[1:], duration=a.step_ms, loop=0, optimize=True)
        print(f'{path.name:20s} {len(frames):4d} frames {path.stat().st_size // 1024:5d} KB')
    if a.gallery:
        gallery(a.gallery, out)


if __name__ == '__main__':
    main()

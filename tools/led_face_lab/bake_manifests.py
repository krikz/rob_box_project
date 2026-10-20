"""Запекает процедурное лицо в нынешний формат анимаций (PNG-кадры + YAML).

Плеер ``rob_box_animations`` проигрывает манифесты с покадровыми PNG; этот
скрипт — их единственный источник. Править PNG и манифесты руками нельзя:
поменял сцену здесь — перезапусти запекание.

    python tools/led_face_lab/bake_manifests.py src/rob_box_animations/animations

Что делает:
* рендерит сцену из ``states`` / ``talk`` / ``vehicles`` с частотой ``face.FPS``;
* раскладывает панели по логическим группам ros2leds. Лицо нарисовано для
  человека напротив робота, а ``wheel_*`` привязаны к бортам: левая фара
  робота (``wheel_front_left``) у зрителя СПРАВА;
* одинаковые подряд кадры панели склеивает в один с большей длительностью,
  одинаковые картинки внутри анимации хранит одним файлом;
* яркость: арт нарисован в уровнях 0..25 (= доли полной шкалы). PNG пишется
  на полной шкале, при текущем драйвере (brightness 25) робот покажет ровно
  эти уровни. Каждой анимации считается бюджет яркости — до какой яркости
  драйвера её самый тяжёлый кадр укладывается в 4,5 А (75% бака 5 В × 30 Вт);
  если бюджет ниже ``TARGET_BRIGHTNESS``, анимация запекается тусклее, чтобы
  после подъёма яркости драйвера до цели она осталась в бюджете.
"""

from __future__ import annotations

import hashlib
import io
import shutil
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rear as R  # noqa: E402
import states as S  # noqa: E402
import talk as TK  # noqa: E402
import vehicles as V  # noqa: E402
from face import FPS, Frame  # noqa: E402

FRAME_MS = 1000 // FPS
assert FRAME_MS * FPS == 1000, 'FPS must divide 1000 ms evenly'

# Панель лица (взгляд зрителя) -> логическая группа ros2leds.
GROUP_OF = {
    'eye_l': 'wheel_front_right',
    'eye_r': 'wheel_front_left',
    'mouth': 'main_display',
    'rear_l': 'wheel_rear_left',
    'rear_r': 'wheel_rear_right',
}
# Одинаковые картинки разных панелей (симметричные глаза, глаз = задняя фара)
# хранятся одним файлом, поэтому имя говорит о форме панели, а не о группе.
SHORT = {
    'wheel_front_left': 'p8', 'wheel_front_right': 'p8',
    'wheel_rear_left': 'p8', 'wheel_rear_right': 'p8', 'main_display': 'm',
}
PANEL_ORDER = ['wheel_front_left', 'wheel_front_right', 'wheel_rear_left', 'wheel_rear_right', 'main_display']

MA_PER_CHANNEL = 20.0      # WS2812B, канал при 255
IDLE_MA = 381 * 1.0        # 1 мА покоя на светодиод
BUDGET_MA = 4500.0         # 75% бака 5 В × 30 Вт
TARGET_BRIGHTNESS = 128    # цель после подъёма яркости драйвера


def concat(*parts):
    out = []
    for p in parts:
        out.extend(p)
    return out


def with_taillight(frames):
    """Сцены лица не рисуют зад сами: там обычные габаритные огни."""
    for i, f in enumerate(frames):
        if not (f.rear_l.any() or f.rear_r.any()):
            f.rear_l, f.rear_r = R.taillight(i / FPS)
    return frames


# имя манифеста -> (рендер, описание)
SCENES = {
    'idle': (lambda: concat(S.idle_stare(T=20.0), S.idle_bored(T=8.0)),
             'Покой: моргания 2–6 с (иногда двойные), саккады, осмотр, ухмылка; раз в цикл заскучал и зевнул'),
    'talking': (lambda: TK.talking('jaw', T=8.0, phrase_pause=(0.18, 0.32)),
                'Говорит: решётка Бендера, проём линзой по слогам А/О/Э, взгляд в сторону в начале фразы, кивок на ударном, моргание в конце фразы'),
    'thinking': (lambda: concat(S.thinking_teeth(T=4.4), S.thinking_orbit(T=3.9)),
                 'Думает: огонёк бежит по зубам и прищур, потом зрачки по кругу и прикус наискось'),
    'happy': (S.happy, 'Радость: глаза ^^ щурятся от смеха, хихиканье, улыбка в линии прикуса'),
    'victory': (V.victory, 'Победа: глаза ^^ щурятся в такт, радуга по зубам, салют и конфетти сзади'),
    'sad': (S.sad, 'Грусть: веки домиком, зрачки вниз, тускнеет, слеза'),
    'angry': (S.angry, 'Злость: веки клином, глаза краснеют, скрежет зубами'),
    'surprised': (S.surprised, 'Удивление: моргнул, глаза распахнуты, зрачок-точка, рот «О»'),
    'sleep': (S.sleep, 'Сон: веки «︶» дышат, z, похрапывает'),
    'wakeup': (S.wakeup, 'Пробуждение: включение как у ЭЛТ, зубы загружаются, осмотрелся, зевок'),
    'error': (S.error, 'Ошибка: глаза-крестики, глитч решётки'),
    'low_battery': (S.low_battery, 'Батарея садится: сонные веки, во рту батарейка с последним делением'),
    'charging': (S.charging, 'Зарядка: довольные глаза, во рту батарейка наполняется'),
    'turn_left': (lambda: V.turn('left'), 'Поворот налево: левая фара робота мигает жёлтым, взгляд в поворот, сзади бегущая заливка'),
    'turn_right': (lambda: V.turn('right'), 'Поворот направо: правая фара робота мигает жёлтым, взгляд в поворот, сзади бегущая заливка'),
    'braking': (S.brake, 'Торможение: стоп-сигнал, прищур'),
    'accelerating': (V.accelerating, 'Разгон: прищур, сжатые зубы, сзади форсаж'),
    'police_lights': (V.police, 'Полиция: строгие глаза в красно-синих отсветах, рот — световая балка (двойные вспышки, разбег, чередование)'),
    'fire_truck': (V.fire_truck, 'Пожарная: глаза-маяки с вращающимся лучом, красно-белая балка, потом пламя во рту'),
    'ambulance': (V.ambulance, 'Скорая: красный крест бьётся в такт пульсу, во рту кардиограмма, синие вспышки'),
    'road_service': (V.road_service, 'Дорожная служба: жёлтые маяки, во рту табло со стрелками, сзади стрелки'),
}


def panels_of(f: Frame):
    return {GROUP_OF[k]: v for k, v in f.panels().items()}


def frame_ma(f: Frame):
    """Ток кадра на полной шкале (драйвер на 255), мА."""
    return sum(float(np.clip(v, 0, 25).sum()) / 25.0 for v in f.panels().values()) * MA_PER_CHANNEL


def brightness_budget(frames):
    peak = max(frame_ma(f) for f in frames)
    if peak <= 0:
        return 255, peak
    return int(min(255, 255 * (BUDGET_MA - IDLE_MA) / peak)), peak


def to_png(arr, scale):
    """Уровни 0..25 -> PNG на полной шкале. ceil: при brightness 25 драйвер
    делает v*25//255, и уровень L должен вернуться ровно в L."""
    v = np.clip(np.asarray(arr) / 25.0 * 255.0 * scale, 0, 255)
    v = np.ceil(v - 1e-6).astype(np.uint8)
    buf = io.BytesIO()
    Image.fromarray(v, 'RGB').save(buf, 'PNG', optimize=True)
    return buf.getvalue()


def bake(name, render, description, old_meta, out_root: Path):
    frames = with_taillight(render())
    budget, peak = brightness_budget(frames)
    scale = min(1.0, budget / TARGET_BRIGHTNESS)
    fdir = out_root / 'frames' / name
    fdir.mkdir(parents=True, exist_ok=True)
    files = {}
    tracks = {g: [] for g in PANEL_ORDER}
    for f in frames:
        for group, arr in panels_of(f).items():
            png = to_png(arr, scale)
            h = hashlib.sha1(png).hexdigest()
            if h not in files:
                prefix = SHORT[group]
                fname = f'{prefix}_{sum(1 for k in files.values() if k[1] == prefix):03d}.png'
                (fdir / fname).write_bytes(png)
                files[h] = (fname, prefix)
            fname = files[h][0]
            track = tracks[group]
            if track and track[-1][0] == fname:
                track[-1][1] += FRAME_MS
            else:
                track.append([fname, FRAME_MS])
    duration = len(frames) * FRAME_MS
    lines = [
        '# Сгенерировано tools/led_face_lab/bake_manifests.py — руками не править,',
        '# менять сцену в генераторе и перезапекать.',
        f'name: "{name}"',
        f'description: "{description}"',
        'version: "3.0"',
        'author: "led_face_lab"',
        '',
        f'duration_ms: {duration}',
        f'loop: {"true" if old_meta.get("loop", True) else "false"}',
        f'fps: {FPS}',
    ]
    # ключи ручного аудио-реактивного режима (AUDIO_REACTIVE.md) переносятся как были
    for key in ('audio_reactive', 'audio_source', 'audio_threshold', 'audio_smoothing'):
        if key in old_meta:
            lines.append(f'{key}: {yaml.safe_dump(old_meta[key], default_flow_style=True).strip().removesuffix("...").strip()}')
    lines += ['', 'panels:']
    for group in PANEL_ORDER:
        lines += [f'  - logical_group: "{group}"', '    offset_ms: 0', '    frames:']
        for fname, ms in tracks[group]:
            lines.append(f'      - image: "frames/{name}/{fname}"')
            lines.append(f'        duration_ms: {ms}')
    meta = old_meta.get('metadata', {})
    lines += [
        '',
        'metadata:',
        f'  priority: "{meta.get("priority", "normal")}"',
        f'  category: "{meta.get("category", "state")}"',
        f'  tags: {yaml.safe_dump(meta.get("tags", []), allow_unicode=True, default_flow_style=True).strip()}',
        f'  brightness_budget: {budget}',
        f'  bake_scale: {scale:.3f}',
        f'  peak_ma_at_255: {round(peak)}',
    ]
    (out_root / 'manifests' / f'{name}.yaml').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    messages = sum(len(t) for t in tracks.values())
    return dict(name=name, frames=len(frames), seconds=duration / 1000, files=len(files),
                messages_per_loop=messages, budget=budget, scale=scale)


def main(out_root: Path):
    old = {p.stem: yaml.safe_load(p.read_text(encoding='utf-8')) for p in (out_root / 'manifests').glob('*.yaml')}
    missing = set(old) ^ set(SCENES)
    if missing:
        raise SystemExit(f'набор сцен должен совпадать с манифестами: {sorted(missing)}')
    frames_root = out_root / 'frames'
    for d in frames_root.iterdir():
        if d.is_dir():
            shutil.rmtree(d)
    rows = [bake(n, r, desc, old[n], out_root) for n, (r, desc) in SCENES.items()]
    print(f'{"name":15s} {"sec":>5s} {"files":>5s} {"msg/s":>6s} {"budget":>6s} scale')
    for r in rows:
        print(f'{r["name"]:15s} {r["seconds"]:5.1f} {r["files"]:5d} {r["messages_per_loop"] / r["seconds"]:6.1f} '
              f'{r["budget"]:6d} {r["scale"]:.3f}')
    print('total files', sum(r['files'] for r in rows))


if __name__ == '__main__':
    main(Path(sys.argv[1]))

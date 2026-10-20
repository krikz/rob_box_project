# led_face_lab — процедурное LED-лицо Rob Box

Источник всех кадров и манифестов в `src/rob_box_animations/animations/`.
Лицо в духе Бендера: тёплые глаза 8×8 с квадратным зрачком и металлическими
веками, рот 25×5 — решётка из двух рядов по шесть зубов, задние фары 8×8.

| Файл | Что внутри |
|------|-----------|
| `face.py` | модели глаза и рта, палитра в уровнях 0..25, `FPS` |
| `talk.py` | речь: огибающая слогов, игра лица, три рта (челюсть, резина, голос-решётка) |
| `states.py` | сцены лица: покой, думает, эмоции, сон, музыка, системные |
| `vehicles.py` | экстренные и езда: рот как световая балка, маяки, поворотники |
| `rear.py` | задние фары: габариты, поворотник, праздничные темы, музыка |
| `bake_manifests.py` | запекание в PNG + YAML нынешнего плеера (21 анимация) |
| `render_gifs.py` | GIF-превью для `docs/assets/animations/` из манифестов |
| `review.py`, `bake_preview.py`, `preview_template.html` | листы кадров и интерактивная страница превью для дизайна |

## Перезапечь после правки сцены

```bash
python tools/led_face_lab/bake_manifests.py src/rob_box_animations/animations
python tools/led_face_lab/render_gifs.py src/rob_box_animations/animations docs/assets/animations
python -m pytest tests/unit/docker/test_animation_assets.py -o addopts="" -p no:cacheprovider
```

Запекание детерминировано: тот же код даёт те же файлы байт в байт.

## Что здесь есть сверх запечённого

Сцены «слушает», три варианта музыки, праздничные темы задних фар, полночь
31.12 и варианты речи «резина» / «голос-решётка» нарисованы, но в нынешнем
плеере им не на что повеситься: он знает только имена из
`rob_box_mcp_tools/animations.py`. Они ждут процедурного плеера (эпик в рамках
ADR-0150).

Зависимости: `numpy`, `pyyaml`, `Pillow`.

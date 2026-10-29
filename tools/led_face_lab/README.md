# led_face_lab — превью процедурного лица Rob Box

Сцены лица живут в пакете `src/rob_box_animations/rob_box_animations/face/`
(их рисует `face_node` на роботе в реальном времени, ADR-0150 §5.5). Лаборатория
импортирует пакет и рендерит те же сцены для глаз дизайнера: листы кадров,
GIF для документации и интерактивную страницу превью. Новая сцена — PR в
`face/states.py` (генератор `fn(ticks, ..., T)`) и строка в `SCENES`.

| Файл | Что внутри |
|------|-----------|
| `render_gifs.py` | GIF-превью всех сцен в `docs/assets/animations/` и галерея `src/rob_box_animations/docs/ANIMATIONS.md` |
| `review.py` | листы кадров (`burst:<сцена>:<кадр>:<сколько>`, `theme:<тема фар>`) |
| `bake_preview.py`, `preview_template.html` | страница превью с бюджетом яркости |

```bash
python tools/led_face_lab/render_gifs.py docs/assets/animations --gallery src/rob_box_animations/docs/ANIMATIONS.md
python tools/led_face_lab/review.py /tmp/review talk_jaw music_groove
python -m pytest tests/unit/docker/test_animation_assets.py -o addopts="" -p no:cacheprovider
```

Рендер детерминирован: первый проход сцены при seed по умолчанию даёт те же
байты (дайджесты — в тесте). Зависимости: `numpy`, `Pillow`.

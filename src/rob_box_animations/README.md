# rob_box_animations — лицо робота

Процедурное LED-лицо Rob Box (ADR-0150 §5.5, PR-3a #3689): сцены рисуются в
реальном времени, 25 к/с, PNG-кадров нет.

```
rob_box_animations/
├── rob_box_animations/
│   ├── face/                 # сцены: face.py (модели, палитра 0..25, раскладка, ток),
│   │                         # states.py (SCENES), talk.py (рты), rear.py (фары),
│   │                         # vehicles.py (экстренные, езда), scene.py (Scene.frame), sides.py (тест-кадр)
│   ├── face_director.py      # что показать: речь, запрос, музыка, покой, сон (чистый Python)
│   └── face_node.py          # ROS-нода: 25 Гц -> /panel_image
├── scripts/face_node.py      # точка входа ros2 run
├── launch/face_led_matrix.launch.py   # драйвер + композитор ros2leds + face_node
└── docs/ANIMATIONS.md        # галерея сцен
```

## Где и как работает

Контейнер `led-matrix` на Vision Pi (решение В14): `start_led_matrix.sh`
запускает `face_led_matrix.launch.py`. Параметры — `docker/vision/config/led_matrix/face_node.yaml`
(том `/config/led_matrix/`): `mouth_design` (`jaw|rubber|voicebox`, В13),
`thinking_design`, `idle_bored_every_s`, `voice_timeout_s`, `test_pattern`,
калибровка DOA `doa_front_deg`/`doa_ccw`, `rear_theme`.

| Вход | Что делает |
|---|---|
| `/voice/speech/envelope`, `/voice/speech/event` | рот по огибающей настоящего TTS, `cancelled` — рот закрыт сразу |
| `/voice/tts/state` | переходный источник «говорит» (дебаунс 0,6 с между чанками) |
| `/audio/direction` + `/audio/vad` | взгляд на говорящего, пока VAD слышит речь (+ 8 с) |
| `/voice/music/state` | сцена музыки по `dj.bpm`, фаза от `dj.started_at` |
| `/voice/animation/request` | v1 `play_animation`: `имя` или `имя:секунды` |
| `/voice/expression/request` | `ExpressionRequest` (эмоция/жест; `face_json` — PR-3c) |
| `~/scene` | отладка: `ros2 topic pub --once /face_node/scene std_msgs/msg/String "{data: 'happy:5'}"` |

Выход — `/panel_image` (`sensor_msgs/Image` rgb8, `frame_id` = логическая
группа), только изменившиеся панели и все раз в секунду. Раз в 10 с в логе
`face.metrics … fps=… render_ms_avg=… face.peak_ma=…`.

Тест-кадр сторон: `test_pattern: true` в `face_node.yaml` и `docker restart
led-matrix` — во рту стрелка вправо (на левую фару робота), на фарах `L1`,
`R2` (перед), `L3`, `R4` (зад).

## Тесты

```bash
cd src/rob_box_animations && python -m pytest test -v      # директор и нода (ROS-заглушки)
python -m pytest tests/unit/docker/test_animation_assets.py  # рендер всех сцен, детерминизм, ток, раскладка
```

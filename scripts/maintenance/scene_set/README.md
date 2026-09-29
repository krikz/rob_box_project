# scene_set — набор размеченных сцен (ADR-0144)

Инструменты для метрик «2.0 лучше 1.1» из ADR-0130 §5.2. Решения, формат и
определения метрик — в `docs/adr/0144-labelled-scene-set-v1-1-baseline.md`.
Здесь — только порядок работы.

| Файл | Где работает | Что делает |
|---|---|---|
| `record_scene.sh` | хост Vision Pi | копирует этот каталог в `voice-assistant`, запускает дирижёра, забирает сцену в `~/scenes/` |
| `conductor.py` | `voice-assistant` | пишет бэг, командует голосом робота, проверяет пустой кадр, пишет `scene.yaml` |
| `scenes/sNN_*.yaml` | — | сценарии 13 сцен (s00 — сид) |
| `replay.py` | хост Vision Pi | изолированный офлайн-прогон одной сцены (свой Zenoh-роутер, домен 77, без `/data`) |
| `decoder_node.py` | `vision-hailo` (контейнер прогона) | сжатые кадры → сырые `Image` для узла лица |
| `extract.py` | `vision-hailo` | выходы 1.1 из бэга → `journal.jsonl` |
| `metrics.py` | где угодно (PyYAML) | `scene.yaml` + `journal.jsonl` → raw-таблица метрик |
| `scene_spec.py` | — | разбор `scene.yaml` (общий для всех) |

## Съёмка

Перед первой сценой: скотч на полу на 1.0 / 1.5 / 2.0 / 3.0 м от стекла камеры;
маска на подставке; коридор подставки в кадре:

```bash
bash ~/rob_box_project/scripts/maintenance/scene_set/record_scene.sh --probe-anchor 10
```

(маска стоит, людей в кадре нет) — вписать напечатанный `anchor_cx` в `scenes/s06…s12`.
Голос маски — `/tmp/mask_voice/*.mp3` на Vision Pi (стирается при перезагрузке) —
заранее на телефон.

Сцена:

```bash
bash ~/rob_box_project/scripts/maintenance/scene_set/record_scene.sh scenes/s01_owner_enter_leave.yaml
```

Результат — `~/scenes/<сцена>_<UTC>/{bag/, scene.yaml, conductor.log}`. Прерванная
сцена (`Ctrl+C`, кадр не опустел, согласие не подтверждено) `scene.yaml` не
получает и в набор не идёт — каталог удалить руками.

## Прогон

Живой `vision-face` на время прогона останавливается (`--stop-live-face`) и потом
перезапускается; скрипт печатает хвост его лога — проверить, что нет `STREAM_ABORT`.

```bash
cd ~/rob_box_project/scripts/maintenance/scene_set
python3 replay.py --scene-dir ~/scenes/s00_owner_registration_<UTC> --run-id R0 \
    --empty-seed --save-seed R0 --stop-live-face          # сид
python3 replay.py --scene-dir ~/scenes/s01_owner_enter_leave_<UTC> --run-id R1 \
    --seed ~/scenes/_seed/R0 --stop-live-face              # каждая следующая сцена — с того же сида
python3 replay.py ... --dry-run                             # только план и проверка изоляции
```

## Метрики

```bash
python3 metrics.py --label v1.1-replay \
    --case ~/scenes/s01_…/scene.yaml:~/scenes/_replay/R1/s01_…/journal.jsonl  # и так по всем сценам
python3 metrics.py --label live-control \
    --case ~/scenes/s01_…/scene.yaml:~/scenes/_replay/R1/s01_…/control_journal.jsonl
```

Таблицу — целиком в issue/PR (ADR-0018). Три сцены с живым вторым человеком
печатаются как `not covered` — так и должно быть, пока его нет.

## Удаление по просьбе

Сцена удаляется целиком: `~/scenes/<сцена>_<UTC>/` и все `~/scenes/_replay/*/<сцена>_<UTC>/`.

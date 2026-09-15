# ADR-0104: «Взгляд» — единый шов источника кадра для vision_hailo_node

> Ранее фигурировал как ADR-0101 в коммите `3913e5f7` (PR #2578). Перенумерация 2026-09-15: см. issue #2582, ADR-AF-0068 (collision-fix по ADR-AF-0030 §2.4).

| Field    | Value                                                    |
|----------|----------------------------------------------------------|
| Status   | **ACCEPTED** (PR в работе, issue #2531)                  |
| Date     | 2026-09-15                                               |
| Authors  | architect (agent), по согласованию с Шифу                |
| Replaces | (нет, частично) — статический input_topic остаётся legacy |

## 1. Контекст и проблема

`vision_hailo_node` (ADR-0089, AI HAT+ 26 TOPS) — единственный потребитель
зрительного канала на Vision Pi. Подписка на кадры была жёстко
прописана в трёх местах (тройная константа `input_topic`):

| Файл                                                       | Что делает                                              |
|------------------------------------------------------------|---------------------------------------------------------|
| `src/rob_box_perception/rob_box_perception/vision_hailo_node.py:112` | `declare_parameter('input_topic', '/oak/rgb/image_raw/compressed')` |
| `docker/vision/config/hailo_models.yaml:38`                | `input_topic: /oak/rgb/image_raw/compressed`           |
| `docker/vision/scripts/vision-hailo/start_vision_hailo.sh:27` | `INPUT_TOPIC="${INPUT_TOPIC:-/oak/rgb/image_raw/compressed}"` |

При этом реальный launch OAK-D (`oakd_with_apriltag.launch.py:11-18`)
публикует под namespace `camera`, с `i_rs_compat:true` (rgb→color) и
`i_publish_compressed:false`. Реальный топик:

```
/camera/camera/color/image_raw    (sensor_msgs/Image, не Compressed)
```

То есть узел подписывался на топик, который не публикуется никем с
текущим launch и текущей конфигурацией — ни по имени (namespace
`camera`, не `oak`), ни по типу (`i_publish_compressed:false`).
При этом:

- Healthcheck `pgrep -f vision_hailo` не видит разницы между
  «жив и работает» и «жив и слепой» — контейнер `Up (healthy)` бессрочно.
- `_is_real_mode` vs строка лога `mode='real'` — расходились: можно
  было залогировать `mode=real` при `_is_real_mode=False`. Это
  нарушает ADR-0018 (capability-honest).
- BGR/RGB каналы не переставлены в `_preprocess` (cv2.imdecode отдаёт
  BGR, YOLOv8n обучен на RGB).
- Letterbox-паддинг не пробрасывается в `_post_process_detections`,
  bbox делится на `input_w`/`input_h` (letterbox-space 640×640) даже
  на несимметричном кадре (640×480 у OAK-D) — bbox уезжает на размер
  паддинга.
- HEF lifecycle полностью ручной: если `/opt/rob_box/models/yolov8n.hef`
  отсутствует, compose упадёт без инструкции что делать.

Это всё про один уровень ниже, чем «Знакомый» (#2440) и «Встреча»
(#2442) — про сам факт того, что зрительный канал вообще поставляет
кадры. Без этого фикса downstream-фичи (идентификация по лицу,
напоминание о встрече) построены на пустом foundation'е.

## 2. Решение

Ввести **шов «Взгляд»** — отдельный модуль `rob_box_perception.gaze`,
скрывающий выбор ROS-топика, тип сообщения и JPEG/PNG-декодирование.
Узел `vision_hailo_node` больше НЕ подписывается на ROS-топики
напрямую — это делает `gaze`.

### 2.1. Архитектура шва

```
+-------------------+      +----------------------+
| OakDSource        |      | CeilingCameraSource  |
| /camera/camera/   |      | /ceiling_camera/     |
|   color/image_raw |      |   image_raw/         |
| msg=Image         |      |   compressed         |
+---------+---------+      +-----------+----------+
          |                             |
          +-----------+-----------------+
                      v
              +---------------+
              |  FrameSource  | Protocol: frames() -> Iterator[Frame]
              +-------+-------+
                      v
              +---------------+
              | Frame dataclass|
              |  rgb ndarray   |
              |  scale,        |
              |  pad_left,     |
              |  pad_top,      |
              |  frame_id,     |
              |  stamp,        |
              |  source_name   |
              +---------------+
```

Frame уже после:
- JPEG/PNG decode (если msg=CompressedImage),
- BGR→RGB перестановки,
- letterbox с метаданными (scale, pad_left, pad_top).

### 2.2. Адаптеры (минимум — два, чтобы шов был настоящим)

| Имя                | Топик                                  | msg              | Когда                                       |
|--------------------|----------------------------------------|------------------|---------------------------------------------|
| `oak_d`            | `/camera/camera/color/image_raw`       | sensor_msgs/Image | Реальная OAK-D камера (Vision Pi)           |
| `ceiling_camera`   | `/ceiling_camera/image_raw/compressed` | CompressedImage  | Потолочная USB-камера (действующие потребители: `quest_node.py:2151`, `telegram_node.py:107`) |
| `stub`             | `<synthetic>`                          | —                | CI / smoke без ROS                          |

Включается через `gaze_source` параметр в launch / YAML.

### 2.3. Параметры (новые)

| Параметр                  | Default | Что делает                                              |
|---------------------------|---------|---------------------------------------------------------|
| `gaze_source`             | `oak_d` | Имя адаптера шва «Взгляд» (SSoT — было три копии `input_topic`). |
| `first_frame_timeout_sec` | `10.0`  | Сколько ждать первый кадр от real-источника перед fail-fast (capability-honest). |
| `input_topic`             | `/camera/camera/color/image_raw` | LEGACY — нода НЕ использует. Сохранён для back-compat скриптов (`diagnose.sh`, `force_publish.sh` и т.д.). |

`vision_hailo.launch.py` обновлён: добавлен `gaze_source` и
`first_frame_timeout_sec`. `hailo_models.yaml` — добавлены те же
поля как SSoT defaults.

### 2.4. Capability-honest fail-fast

Раньше `publish_when_no_input=True` (дефолт) маскировал недоступность
источника — `_tick` публиковал stub-события бессрочно даже без единого
кадра (issue #2531 acceptance #5).

Теперь:
- В **real-mode** (`hailo_enabled=True`): если за `first_frame_timeout_sec`
  источник не отдал кадр — нода `raise` при инициализации с
  `GazeSourceUnavailable`, docker перезапустит контейнер.
- В **stub-mode** (`hailo_enabled=False`): если источник недоступен —
  нода логирует WARN и переключается на `StubSource` (синтетический
  кадр), продолжает работу.

`publish_when_no_input` остался как опция (по умолчанию `True` для CI
smoke-теста), но в проде должен быть `False`.

### 2.5. Честный healthcheck

Старый healthcheck `pgrep -f vision_hailo` → `Up (healthy)` даже если
ни один кадр не получен. Новый `healthcheck_frame.sh`:

1. `pgrep -f vision_hailo` (процесс жив)
2. `ros2 topic info /vision/hailo/events` → `Publisher count: [1-9]`

Если source недоступен — Publisher count остаётся 0, контейнер
уходит в `unhealthy` через `start_period + retries × interval` ≈
2 минуты. Это видно в Grafana/Promtail — оператор может среагировать.

### 2.6. BGR→RGB и letterbox-проекция

- `Frame.rgb` всегда RGB (YOLOv8n обучен на RGB; cv2.imdecode отдаёт
  BGR, `gaze.py` делает `cv2.cvtColor` ДО letterbox, чтобы паддинг
  заливался уже в RGB).
- `Frame.as_letterbox(input_w, input_h)` или `_preprocess` в loader
  возвращает `LetterboxInfo(scale, pad_left, pad_top, orig_w, orig_h)`.
- `_post_process_detections` принимает `letterbox_info` и применяет
  `unproject`: bbox сначала `(cx - pad_left, cy - pad_top)` / scale
  (в исходный кадр), потом делится на `orig_w`/`orig_h`.
- Если `letterbox_info=None` — старое поведение (back-compat для
  unit-тестов с квадратным синтетическим входом).

### 2.7. Mode-лог consistency

`vision_hailo_node.py:135-140` раньше считал `_is_real_mode` одним
способом (`hailo_enabled and hef_path and _NUMPY_AVAILABLE and _CV2_AVAILABLE`),
а строка лога `:197` — другим (`hailo_enabled and hef_path`). Расхождение
нарушало ADR-0018.

Теперь — один расчёт: `_is_real_mode = bool(hailo_enabled and hef_path)`.
Лог использует ту же переменную. Реальный capability-honest (numpy/cv2
отсутствие → degraded stub) отслеживается через WARN-лог в
`start_vision_hailo.sh` capability-honest gate (без правки ноды).

## 3. Touchpoints

| Файл / компонент                                                  | Что меняется                                                       |
|-------------------------------------------------------------------|--------------------------------------------------------------------|
| `src/rob_box_perception/rob_box_perception/gaze.py` (NEW)          | Шов «Взгляд»: FrameSource + OakD/Ceiling/Stub адаптеры            |
| `src/rob_box_perception/rob_box_perception/vision_hailo_node.py`  | Использует gaze.make_source(); параметр input_topic → gaze_source |
| `src/rob_box_perception/rob_box_perception/vision_hailo_loader.py`| LetterboxInfo + unproject в _post_process_detections; RGB-контракт |
| `src/rob_box_perception/launch/vision_hailo.launch.py`            | Добавлены gaze_source и first_frame_timeout_sec                    |
| `docker/vision/config/hailo_models.yaml`                          | gaze_source: oak_d; first_frame_timeout_sec: 10.0                 |
| `docker/vision/scripts/vision-hailo/start_vision_hailo.sh`        | Прокидывает GAZE_SOURCE / FIRST_FRAME_TIMEOUT_SEC ENV в launch    |
| `docker/vision/scripts/vision-hailo/healthcheck_frame.sh` (NEW)   | Честный healthcheck (pgrep + Publisher count > 0)                 |
| `docker/vision/scripts/vision-hailo/download_yolov8n_hef.sh` (NEW)| HEF download из hailo_model_zoo с optional SHA256                  |
| `docker/vision/docker-compose.yaml`                               | Healthcheck → /scripts/healthcheck_frame.sh                       |
| `src/rob_box_perception/test/unit/test_gaze_seam.py` (NEW)        | 9 unit-тестов для Frame / StubSource / make_source / error         |
| `src/rob_box_perception/test/unit/test_vision_hailo_letterbox_unproject.py` (NEW) | 7 тестов для асимметричного letterbox + back-compat |

## 4. Альтернативы (отвергнуты)

### A. Точечная правка input_topic в одном месте
- ❌ Не решает: жёсткая привязка к OAK-D, нет второго адаптера,
  нет RGB, нет letterbox fix, нет fail-fast.
- Это «косметика» поверх архитектурной проблемы.

### B. Расширить vision_hailo_node параметрами для каждого источника
- ❌ Нода распухает; каждый новый источник = правка ноды, а не
  добавление нового адаптера.
- Нарушает OCP (open-closed principle): добавление ceiling-camera
  потребовало бы правки `__init__` и `_on_image`.

### C. Оставить input_topic, добавить input_topic_compressed флаг
- ❌ Это ещё больше копий той же проблемы, не её решение.

### D. Не делать шов, оставить всё как есть
- ❌ Issue #2531 acceptance #1: «вынести разрешение источника, декодирование
  и препроцессинг за отдельный шов». Архитектурный долг остаётся.

## 5. Acceptance criteria

Все пункты issue #2531 §«Acceptance»:

| #   | Критерий                                                                                          | Статус     |
|-----|---------------------------------------------------------------------------------------------------|------------|
| 1   | Ввести модуль «Взгляд» (gaze.py) с интерфейсом frames() → Iterator[Frame]                          | ✅ done (gaze.py, FrameSource Protocol) |
| 2   | Устранить тройное дублирование дефолтного топика                                                  | ✅ done (gaze_source SSoT, input_topic legacy) |
| 3   | Адаптер OAK-D подписан на `/camera/camera/color/image_raw`                                         | ✅ done (OakDSource), требует живой проверки на Vision Pi |
| 4   | Адаптер ceiling-camera (второй независимый сценарий)                                               | ✅ done (CeilingCameraSource) |
| 5   | Разрешение источника fail-fast на старте                                                          | ✅ done (GazeSourceUnavailable + first_frame_timeout_sec) |
| 6   | Честный healthcheck сервиса vision-hailo                                                          | ✅ done (healthcheck_frame.sh) |
| 7   | Убрать расхождение _is_real_mode vs лог mode                                                      | ✅ done (один расчёт, см. vision_hailo_node.py:202) |
| 8   | Добавить BGR→RGB в путь декодирования                                                            | ✅ done (gaze._decode_*_to_rgb + Frame.rgb) |
| 9   | Прокинуть scale/pad_left/pad_top в _post_process_detections                                       | ✅ done (LetterboxInfo + unproject) |
| 10  | HEF lifecycle: скрипт скачивания                                                                  | ✅ done (download_yolov8n_hef.sh) |
| 11  | Stub-filter / event_type=«stub» (PR #2524) — out of scope                                         | ⏳ отдельная карточка (не в этом issue) |
| 12  | Принято на живом роботе                                                                          | 🟡 live-acceptance через e2e-process |

## 6. Совместимость и миграция

### 6.1. Что НЕ меняется (back-compat)

- `vision_hailo_node` всё ещё публикует на `/vision/hailo/events`.
- Downstream (`context_aggregator_node`, `mcp_server.py`) — без правок.
- `input_topic` параметр сохранён для back-compat (старые YAML / скрипты).
- Letterbox denormalization: при `letterbox_info=None` — старое поведение
  (для unit-тестов с квадратным синтетическим входом).

### 6.2. Что МЕНЯЕТСЯ (нужны миграции)

- `start_vision_hailo.sh` передаёт gaze_source в launch вместо input_topic.
  ENV `INPUT_TOPIC` всё ещё принимается (для back-compat), но
  передаётся только для логирования — нода игнорирует.
- `docker-compose.yaml` healthcheck: теперь требует bash + ros2 CLI
  в healthcheck_frame.sh. Если образ vision-hailo не имеет `ros2` CLI
  в PATH — healthcheck фоллбэчит на pgrep-only с WARN.

### 6.3. Rollback

Если в проде возникнут проблемы с реальным OAK-D источником
(например, namespace другой чем ожидалось) — revert этого PR
вернёт старое поведение (топик подписки не менялся, только параметры).

## 7. Open questions

- **Q1**: `publish_when_no_input=True` в проде или False?
  - По умолчанию True (для CI smoke), в проде должен быть False.
  - Решается отдельной карточкой (Phase 1.6 deploy policy).

- **Q2**: ceiling_camera в проде или только для тестов?
  - Пока как second-adapter proof. Включение в проде — по запросу.

- **Q3**: HEF download в docker-entrypoint?
  - Нет. Скачивание — ручное / CI / deploy-скрипт. Compose упадёт если
    файла нет, и это явный сигнал оператору (а не silent fallback).

## 8. Перекрёстные ссылки

- ADR-0089 §3 (touchpoint #3 — vision_hailo_node, #11 — HEF lifecycle).
- ADR-0110 (vision_hailo.launch.py — choice of source).
- ADR-0018 (capability-honest mode — mode-лог consistency, fail-fast).
- ADR-0099 (hailort install strategy — пока в PR #2517).
- Issue #2440 «Знакомый» (выше по стеку).
- Issue #2442 «Встреча» (выше по стеку).
- Issue #2398 (RealHEFLoader — зависимость снизу).

---

**Status**: ACCEPTED для реализации в PR #2531. После мержа — реализация
Phase 1 завершена. Phase 1.5 (real inference с HEF) и Phase 1.6
(deploy policy) — отдельные карточки.

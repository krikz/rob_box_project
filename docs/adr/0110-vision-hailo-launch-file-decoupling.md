# ADR-0110 — vision_hailo launch-файл: декларативный запуск вместо ros2 run

> Ранее фигурировал как ADR-0096 в коммите `431eb023` (PR #2524). Перенумерация 2026-09-15: см. issue #2582 — сосед `0096-encounter-seam.md` влит раньше (`984542ab`, PR #2454) и остался на 0096.

**Дата:** 2026-09-15
**Статус:** Accepted
**Автор:** architect worker (kanban t_5736df3b, issue #2498)
**Тип:** architecture correction (ADR-0089 §3 touchpoint #6)
**Заменяет / корректирует:** ADR-0089 §3 touchpoint #6 (формулировка)
**Связанные:** ADR-0089 (AI HAT+ deployment), ADR-0018 (capability-honest), ADR-0010 (perception-bridge)

---

## 1. Контекст и проблема

### 1.1. Issue #2498 (что не так)

**Найдено при ревью компонента `docker/vision` 2026-09-14** (kanban t_beba0869): `vision_hailo_node` НЕ зарегистрирован ни в одном launch-файле perception-компонента. Существующие файлы:

- `src/rob_box_perception/launch/internal_dialogue.launch.py` — host-разработка
- `src/rob_box_perception/launch/internal_dialogue_docker.launch.py` — docker-деплой

**Сейчас на проде** (Vision Pi через `docker compose up`) `vision_hailo` запускается через `start_vision_hailo.sh` строкой `exec ros2 run rob_box_perception vision_hailo --ros-args "${ROS_ARGS[@]}"` — параметры захардкожены в bash-скрипте, читаются из ENV + YAML вручную.

**ADR-0089 §3 touchpoint #6** формулирует контракт: *"Добавить Node `vision_hailo` (gated `hailo_enabled` параметром) в `internal_dialogue.launch.py`"*. Этот touchpoint в текущей формулировке **нереализуем** (см. §1.2) — архитектура после Phase 1 merge разошлась с ADR.

### 1.2. Почему touchpoint #6 в текущей формулировке нереализуем

После merge PR #2349 (Phase 1 PoC) `vision_hailo` живёт **в отдельном Docker-сервисе** `vision-hailo` (compose-файл `docker/vision/docker-compose.yaml:648-698`). Контракт:

- **Vision Pi (10.1.1.21)**: compose `docker/vision/docker-compose.yaml` поднимает `vision-hailo` сервис → `/dev/hailo0` устройство, HailoRT, AI HAT+. На Vision Pi **нет** `internal_dialogue.launch.py` (он не используется на этой машине).
- **Main Pi (10.1.1.22)**: compose `docker/main/scripts/perception/start_perception.sh:46` запускает `ros2 launch rob_box_perception internal_dialogue_docker.launch.py`. На Main Pi **нет** `/dev/hailo0` — там CAN HAT (MCP2515, ADR-0089 §1.1 явно фиксирует: "Main Pi не подходит: HAT-слот занят CAN HAT").

Если буквально следовать touchpoint #6 и добавить `Node(vision_hailo, ...)` в `internal_dialogue_docker.launch.py` — **нода будет пытаться стартовать на Main Pi без `/dev/hailo0`** → либо краш, либо fallback в stub (если убрать capability-honest). И то и другое = нарушение §6 ADR-0089 / §1 ADR-0018.

Топология **жёстко разнесена по машинам**:

```
┌─ Vision Pi (10.1.1.21) ──────────────────────┐  ┌─ Main Pi (10.1.1.22) ─────────────┐
│ zenoh-router                                  │  │ zenoh-router → Main Pi (cloud)    │
│ oak-d          → /oak/rgb/image_raw/...       │  │                                    │
│ ceiling-camera → /ceiling/...                 │  │ perception / Main Pi:              │
│ vision-hailo ★ → /vision/hailo/events        │  │   internal_dialogue_docker.launch  │
│ led-matrix                                    │  │     ├─ perception_bridge           │
│ voice-assistant                               │  │     ├─ context_aggregator          │
│                                               │  │     └─ health_monitor              │
│ (docker/vision/docker-compose.yaml)           │  │ (docker/main/scripts/perception/...)│
└──────────────────┬────────────────────────────┘  └────────────────┬────────────────────┘
                   │  zenoh (RMW_IMPLEMENTATION=rmw_zenoh_cpp)         │
                   └──────────────────────────────────────────────────┘
                                                                   ▲
                                          /perception/context_update
                                          (читает context_aggregator
                                           И vision_events_json
                                           после Phase 1.5)
```

### 1.3. Что touchpoint #6 на самом деле требует

Семантика touchpoint: "vision_hailo_node стартует **декларативно, через launch-параметры**, а не через ручной `ros2 run`". Это исключает:
- Хардкод ENV-флагов в bash (`HAILO_ENABLED`, `HEF_PATH`, `STUB_PERIOD_SEC`).
- Ручной YAML-парсинг в bash (`python3 -c 'import yaml; ...'`).
- Дрейф между bash-параметрами и ROS-параметрами.

И включает:
- `ros2 launch` как единый entrypoint.
- LaunchConfiguration для всех параметров.
- Capability-honest gate через `OpaqueFunction` / `ExecuteProcess`.
- Возможность подключить к одному большому perception launch через `<include>` если когда-нибудь архитектура сойдётся (например, при слиянии Vision/Main Pi на одной машине).

---

## 2. Решение

### 2.1. Новый launch-файл `vision_hailo.launch.py` (SSoT параметров)

**Создать** `src/rob_box_perception/launch/vision_hailo.launch.py` — единственный launch-файл для ноды. Содержит:

```python
# Контракт: vision_hailo_node стартует с параметрами из launch-файла,
# а не через захардкоженный `ros2 run` в bash.
# Capability-honest gate: если hailo_enabled=true но HEF/deps отсутствуют —
# node логирует WARN и работает в stub-режиме (уже реализовано в
# vision_hailo_node.py:_is_real_mode через ImportError).
```

Параметры (LaunchConfiguration, не ENV):
- `hailo_enabled` (default `false`)
- `hef_path` (default `""`)
- `stub_period_sec` (default `2.0`)
- `confidence_threshold` (default `0.5`)
- `input_topic` (default `/oak/rgb/image_raw/compressed`)
- `output_topic` (default `/vision/hailo/events`)
- `publish_when_no_input` (default `true`)

Также: `OpaqueFunction` проверяет наличие `/dev/hailo0` если `hailo_enabled=true` — если нет, логирует WARN и продолжает (capability-honest).

### 2.2. Переключить `start_vision_hailo.sh` на `ros2 launch`

**Было**:
```bash
exec ros2 run rob_box_perception vision_hailo --ros-args "${ROS_ARGS[@]}"
```

**Стало**:
```bash
exec ros2 launch rob_box_perception vision_hailo.launch.py \
    hailo_enabled:=${HAILO_ENABLED} \
    hef_path:=${HEF_PATH} \
    stub_period_sec:=${STUB_PERIOD_SEC} \
    confidence_threshold:=${CONFIDENCE_THRESHOLD} \
    input_topic:=${INPUT_TOPIC} \
    output_topic:=${OUTPUT_TOPIC} \
    publish_when_no_input:=true
```

Bash-скрипт остаётся как orchestration layer (YAML-парсинг, ENV override, smoke `hailortcli scan`, degraded-mode check), но **финальный запуск ноды — через launch**.

### 2.3. Что НЕ делаем (важно)

- **НЕ добавляем** `Node(vision_hailo)` в `internal_dialogue.launch.py` — он для host-dev, концептуально не место для hardware-gated ноды.
- **НЕ добавляем** в `internal_dialogue_docker.launch.py` — исполняется на Main Pi (CAN HAT, не AI HAT+, см. §1.2).
- **НЕ трогаем** `docker-compose.yaml` для `vision-hailo` сервиса — он уже корректен.
- **НЕ трогаем** `internal_dialogue_docker.launch.py` — он для Main Pi, vision-hailo там не нужен.

### 2.4. Touchpoint #6 устарел — обновляем формулировку

В ADR-0089 §3 touchpoint #6 заменить текст:

**Было**: "Файл `src/rob_box_perception/launch/internal_dialogue.launch.py` | Добавить Node `vision_hailo` (gated `hailo_enabled` параметром)"

**Стало**: "Файл `src/rob_box_perception/launch/vision_hailo.launch.py` (**NEW**) | SSoT launch-описание ноды. Параметры через LaunchConfiguration (`hailo_enabled`, `hef_path`, `stub_period_sec`, `confidence_threshold`, `input_topic`, `output_topic`). Capability-honest gate через OpaqueFunction. Bash entrypoint `start_vision_hailo.sh` переключается с `ros2 run` на `ros2 launch rob_box_perception vision_hailo.launch.py`."

И добавить note: *"Touchpoint изначально планировал добавление в `internal_dialogue.launch.py`, но после Phase 1 merge `vision_hailo` живёт в отдельном Docker-сервисе на Vision Pi (AI HAT+ = Vision Pi, не Main Pi, см. ADR-0089 §1.1). Корректировка зафиксирована в ADR-0110."*

---

## 3. Touchpoints

| # | Файл | Что меняется |
|---|---|---|
| 1 | `src/rob_box_perception/launch/vision_hailo.launch.py` | **NEW**: декларативный launch с LaunchConfiguration параметрами + OpaqueFunction gate |
| 2 | `docker/vision/scripts/vision-hailo/start_vision_hailo.sh` | `ros2 run` → `ros2 launch rob_box_perception vision_hailo.launch.py` с передачей параметров |
| 3 | `docs/adr/0089-ai-hat-plus-deployment.md` §3 touchpoint #6 | Переформулировать: убрать "internal_dialogue.launch.py", заменить на vision_hailo.launch.py + note про ADR-0110 |
| 4 | `docker/vision/README.md` | Сервис "vision-hailo" уже есть в списке (проверено — да, mention есть через Dockerfile); **дополнить** явным упоминанием launch-параметров и capability-honest gate (issue F-20) |
| 5 | `src/rob_box_perception/README.md` | Раздел "Architecture" уже упоминает vision_hailo_node — **дополнить** ссылкой на launch-файл + ADR-0110 |

**Out of scope** (явно): mcp_server consumer-side (Phase 1.5, ADR-0089 §3 touchpoints #16-21, отдельная карточка).

### 3.1. Update 2026-09-15 — DRY-рефакторинг launch_factory (issue #2658)

К моменту Phase 2 (issue #2599, добавление `vision_face`) появился второй launch-файл `vision_face.launch.py`, который побуквенно копировал `vision_hailo.launch.py` (~110 строк launch-конструкции). Вынесена общая фабрика в `src/rob_box_perception/rob_box_perception/launch_factory.py` (`make_hailo_node_launch(executable, confidence_threshold_default, include_nms_iou, include_input_topic, preflight_prefix)`). Оба launch-файла превратились в шимы по 40-48 строк.

**Где живёт factory** — в Python-пакете `rob_box_perception/`, не в `launch/`: `setup.py` устанавливает `launch/*.launch.py` как `data_files` через `ament_index`, не как Python-пакет (импорт через `importlib.util` хрупок при сборке). Существующий паттерн `from rob_box_perception import X` уже работает между модулями этого пакета (см. `vision_face_node.py`, `vision_hailo_node.py`).

**Touchpoints**:

| # | Файл | Что меняется |
|---|---|---|
| 1a | `src/rob_box_perception/rob_box_perception/launch_factory.py` | **NEW**: единая factory `make_hailo_node_launch`. SSoT launch-конструкции (11 `DeclareLaunchArgument` + `OpaqueFunction` + `Node`). |
| 1b | `src/rob_box_perception/launch/vision_hailo.launch.py` | 225 → 48 строк: шим над factory с `include_input_topic=True` (back-compat), `preflight_prefix='vision_hailo'`. |
| 1c | `src/rob_box_perception/launch/vision_face.launch.py` | 158 → 40 строк: шим над factory с `include_nms_iou=True` (RetinaFace NMS), `preflight_prefix='vision_face'`. |
| 1d | `src/rob_box_perception/test/test_launch_factory_static.py` | **NEW**: AST-тесты — `0` настоящих `DeclareLaunchArgument` в `launch/`, `≤40` значимых конструкций в каждом шиме. Не требует `launch` пакета, проходит на любой dev-машине. |
| 1e | `src/rob_box_perception/test/test_launch_factory.py` | **NEW**: runtime-тесты — `make_hailo_node_launch` возвращает корректный `LaunchDescription`, набор аргументов соответствует вызову. Скипается без `launch` пакета (есть в CI-образе `ghcr.io/krikz/rob-box-ci:humble`). |
| 1f | `src/rob_box_perception/README.md` | Раздел "Launch" — упоминание общего factory + issue #2658. |

---

## 4. Альтернативы, которые отвергли

### Альтернатива A: literal touchpoint #6 — добавить в `internal_dialogue.launch.py`

- **Плюсы**: минимальный diff, формально закрывает touchpoint.
- **Минусы**: `internal_dialogue.launch.py` исполняется на dev-машине или на Main Pi через docker-обёртку. На Main Pi **нет** `/dev/hailo0` (CAN HAT) — нода попытается стартовать и провалится (или сломает capability-honest, если сделать silent fallback). Архитектурно неверно: vision-hailo живёт на **Vision Pi**, а `internal_dialogue.launch.py` — на **Main Pi**.
- **Вердикт**: ❌ отвергнуто — нарушает §1.2 ADR-0089 / §1 ADR-0018.

### Альтернатива B: literal touchpoint #6 + `<include>` из обоих launch

- **Плюсы**: формально использует launch-подход.
- **Минусы**: добавляет Node в `internal_dialogue.launch.py` ради "соответствия", что в dev-окружении приведёт к фантомному старту ноды без hardware.
- **Вердикт**: ❌ отвергнуто — overengineering + та же проблема с hardware-окружением.

### Альтернатива C: оставить `ros2 run` в bash, не создавать launch

- **Плюсы**: zero diff, ничего не ломается.
- **Минусы**: ADR-0089 §3 touchpoint #6 **явно требует** launch-файл. `ros2 run` означает захардкод параметров в bash, что расходится с ROS-idiomatic declarative launch. Также — bash-скрипт сейчас руками парсит YAML, что лишний слой парсинга.
- **Вердикт**: ❌ отвергнуто — не выполняет контракт ADR-0089.

### Альтернатива D (выбрана): новый `vision_hailo.launch.py` + переключение bash на `ros2 launch`

- **Плюсы**:
  - Декларативный запуск (LaunchConfiguration — SSoT параметров).
  - Capability-honest gate через OpaqueFunction.
  - **Не ломает существующую топологию** (Vision Pi ≠ Main Pi).
  - Touchpoint #6 закрыт по семантике ("vision_hailo стартует через launch-файл").
  - Когда-нибудь можно включить в общий perception launch через `<include>` без рефакторинга.
- **Минусы**: ещё один launch-файл в проекте. Но: это single-purpose файл для hardware-gated ноды, оправдано.
- **Вердикт**: ✅ принято.

---

## 5. Acceptance criteria

### Phase 1 (эта карточка)

- [ ] `src/rob_box_perception/launch/vision_hailo.launch.py` создан, парсит YAML или использует дефолты, объявляет Node `vision_hailo` со всеми LaunchConfiguration параметрами.
- [ ] `docker/vision/scripts/vision-hailo/start_vision_hailo.sh` запускает `ros2 launch rob_box_perception vision_hailo.launch.py` (не `ros2 run`).
- [ ] `docker compose -f docker/vision/docker-compose.yaml up vision-hailo` стартует, `ros2 topic list` показывает `/vision/hailo/events` (через `docker exec ... ros2 topic list` или прямой запуск).
- [ ] Capability-honest: `hailo_enabled=true` + отсутствие `/dev/hailo0` → WARN в логе, нода в stub-режиме (не crash).
- [ ] Capability-honest: `hailo_enabled=false` → stub-режим, события публикуются с `event_type="stub"` (поведение уже реализовано в ноде).
- [ ] ADR-0089 §3 touchpoint #6 переформулирован, ссылка на ADR-0110 добавлена.
- [ ] `docker/vision/README.md` и `src/rob_box_perception/README.md` дополнены упоминанием нового launch-файла.
- [ ] Юнит-тесты существующие (`test_vision_hailo_node.py` 8/8 + `test_vision_hailo_phase15.py`) — не сломаны (запуск через launch не меняет API ноды).
- [ ] Локальный прогон: `ros2 launch rob_box_perception vision_hailo.launch.py` стартует, `/vision/hailo/events` есть.

### Phase 1.5 (отдельная карточка, не в этом ADR)

- Consumer-side (`mcp_server.on_perception_update` → `vision_events_json`).
- Stub-filter (`event_type == "stub"` НЕ доходит до LLM).
- Privacy review (ADR-0089 §3 touchpoints #16-21).

---

## 6. Capability-honest режим (ADR-0018)

Запуск с `hailo_enabled=true` на машине без `/dev/hailo0` или без numpy/cv2/hailo_platform **не должен** приводить к:
- Silent fallback на stub (без предупреждения).
- Crash без actionable лога.
- Простою downstream pipeline в непонятном состоянии.

**Контракт capability-honest (уже реализован в `vision_hailo_node.py`):**

1. `hailo_enabled=true` + всё есть → real mode (HailoRT inference).
2. `hailo_enabled=true` + что-то отсутствует → WARN в лог + degraded to stub (с явным указанием ЧЕГО не хватает).
3. `hailo_enabled=false` → stub mode (события `event_type="stub"` — это контракт для Phase 1.5 stub-filter'а).

**Launch-файл добавляет OpaqueFunction** — pre-flight check, логирующий:
- `hailo_enabled` (что заказали).
- `hef_path` (есть ли файл, readable?).
- `/dev/hailo0` (есть ли device, на старте ли драйвер?).

Эти проверки **не блокируют** старт ноды — только информируют. Деградация — в самой ноде.

---

## 7. Open questions

1. **Алиасы параметров**: launch-файл читает параметры с `hailo_models.yaml` (SSoT) ИЛИ через `ros2 launch ... key:=value`? Текущая идея — YAML через `_load_params_from_yaml` (Python helper внутри launch), но это требует явного `find_package(yaml-cpp)` или subprocess. Проще — дефолты в launch + ENV override из bash.
   *Решение (предложение)*: дефолты в launch (`hailo_enabled=false`); bash читает `hailo_models.yaml` через `python3 -c` (уже реализовано в `start_vision_hailo.sh`) и передаёт `--ros-args` через launch. Так launch не зависит от yaml-парсинга, а bash остаётся orchestration layer.

2. **`publish_when_no_input` в production**: должен быть `false` на реальном Vision Pi (нода не должна плодить события без реального изображения), `true` только в dev/CI. Текущий bash-скрипт всегда передаёт `true`. *Решение*: оставить как сейчас, добавить комментарий в launch что в prod это надо выставить `false`.

3. **Когда подключать через `<include>` к общему perception launch**: если Vision Pi и Main Pi когда-нибудь сойдутся на одной машине (маловероятно), или если `internal_dialogue.launch.py` будет развит в сторону Vision Pi. *Решение*: НЕ в этой карточке. Когда понадобится — `<include>` тривиально добавить.

---

## 8. Change log

| Дата | Автор | Изменение |
|---|---|---|
| 2026-09-15 | architect (t_5736df3b, issue #2498) | Initial ADR-0110 (при merge — ADR-0096, переименован issue #2582). Touchpoint #6 ADR-0089 признан нереализуемым в буквальной формулировке (Vision Pi ≠ Main Pi, AI HAT+ ≠ CAN HAT). Принято: новый `vision_hailo.launch.py` + bash на `ros2 launch`. Touchpoint #6 переформулирован в ADR-0089 §3. |
| 2026-09-15 | architect (t_8cfecf68, issue #2658) | DRY-рефакторинг: вынесена `rob_box_perception/launch_factory.py`. vision_hailo.launch.py и vision_face.launch.py стали шимами по 40-48 строк. Acceptance: AST-тесты + runtime-тесты. Touchpoints §3.1. |

*ADR-0110 корректирует ADR-0089 §3 #6 без изменения общей архитектуры Phase 1.*

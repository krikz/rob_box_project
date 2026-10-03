# ADR-0151: Issue #2826 — архитектурный вердикт + runtime-страж

**Статус:** принят (03.10.2026, architect)
**Репортёр:** architect
**Severity:** MEDIUM — runtime DDS-баг, ловится только на живом роботе
**Refs:** #2826, PR #3022 (merged 27.09.2026 в develop), #3023 (unit-tests shim)

## Контекст

При проработке архитектуры восприятия v2.0 выявлен root cause «no map → no meeting cards»:
`context_aggregator` (нода `src/rob_box_perception/rob_box_perception/context_aggregator_node.py`)
был подписан на `/rtabmap/localization_pose` как на `geometry_msgs/PoseStamped`, а `rtabmap`
публикует `geometry_msgs/PoseWithCovarianceStamped`. В ROS 2 при разных типах DDS не
связывает publisher и subscriber — агрегатор **ни разу не получал позу**, и поле pose в
`/perception/context_update` всегда было пустым.

### Доказательство на Main Pi 10.1.1.21 (23.09.2026 ~11:40 UTC)

```
$ docker exec perception ros2 topic info -v /rtabmap/localization_pose --no-daemon
Type: ['geometry_msgs/msg/PoseWithCovarianceStamped', 'geometry_msgs/msg/PoseStamped']
Publisher count: 1
Node name: rtabmap            Topic type: geometry_msgs/msg/PoseWithCovarianceStamped  PUBLISHER
Subscription count: 2
Node name: quest_node         Topic type: geometry_msgs/msg/PoseWithCovarianceStamped  SUBSCRIPTION
Node name: context_aggregator Topic type: geometry_msgs/msg/PoseStamped               SUBSCRIPTION
```

Поза публикуется ~1 Гц (RTAB-Map в режиме локализации).

### Регрессионный фон

`quest_node` уже был подписан правильно (использует `PoseWithCovarianceStamped` для того же
топика, см. `src/rob_box_quest/rob_box_quest/quest_node.py:2135-2140`). То есть рассогласование
типов в `context_aggregator` — изолированный баг, не системная проблема.

## Решение

### Шаг 1: код-фикс (PR #3022, merged 27.09.2026)

PR #3022 `0b3978eed fix(perception #2826): subscribe to localization_pose as
PoseWithCovarianceStamped` уже доставлен в develop. Внутри:

- `context_aggregator_node.py: подписка на `PoseWithCovarianceStamped`, читает `msg.pose.pose`;
- `test/test_context_aggregator.py`: регрессионный страж + `PoseWithCovarianceStamped`-shim
  + два новых теста `test_localization_pose_subscription_uses_correct_type` и
  `test_pose_msg_type_matches_rtabmap` (anti-regression sentinels);
- `test/unit/test_vision_events_aggregator.py`: `_PoseWithCovariance*` shim для stub-импорта;
- `README.md`: типы в `/rtabmap/localization_pose` обновлены.

Pre-merge: 43/43 unit-тестов passed, anti-regression проверен (откат типов на `PoseStamped`
флипает оба новых теста в FAILED с явным `issue #2826` сообщением).

### Шаг 2: runtime-страж (этот PR, ветка `z-{agent}/2826-fix-v2-tmp`)

CI ловит ament_* + unit-тесты, но **не ловит runtime DDS-бридж на живом роботе** —
особенно под `rmw_zenoh`, где поведение типа «издатель есть, подписчик есть, но мост
их не сматчил» проявляется только в живом DDS-discovery. Поэтому добавлен
`deploy` smoke-test сценарий `.github/e2e/scenarios/2826_perception_context_pose_smoke_v1.json`
по proven-шаблону `2089_quest_supervisor_msgs_smoke_v1.json`.

Сценарий проверяет три вещи после `docker compose restart rob-box-perception`:

1. **Тип подписки** — `context_aggregator` подписан на `PoseWithCovarianceStamped`
   (НЕ `PoseStamped`) — анти-регрессия #2826.
2. **Поза в /perception/context_update** — `pose.position.x != 0.0` (реальная SLAM-поза).
3. **Отсутствие warning'ов** — в `docker logs --since 2m rob-box-perception` нет
   «incompatible type / type mismatch / no messages received on /rtabmap/localization_pose».

### Шаг 3: pre-PR live verification (Main Pi 10.1.1.21, 03.10.2026 ~10:23 UTC)

```
$ docker exec vision-hailo bash -c 'source /opt/ros/humble/setup.bash && \
    ros2 topic info -v /rtabmap/localization_pose --no-daemon' | grep -A1 context_aggregator
Node name: context_aggregator
Node namespace: /
Topic type: geometry_msgs/msg/PoseWithCovarianceStamped  <- SUBSCRIPTION
```

Тип подписки совпадает с типом publisher (rtabmap), фикс работает на живом роботе.
Smoke-test сценарий — страховка от регрессии при следующих релизах.

## Альтернативы (что НЕ выбрали)

- **Переписать rtabmap на PoseStamped** — неверное направление: `PoseWithCovarianceStamped`
  — стандарт ROS 2 SLAM-сообщества, переписывать publisher ради одного подписчика
  анти-паттерн.
- **Добавить remap в launch-файле** — скрывает баг вместо фикса, и `PoseStamped` теряет
  `covariance` (полезна для downstream).
- **Один «глобальный типобезопасный» wrapper** — overkill для одного топика, KISS нарушен.

## Trade-off

- ✅ Unit-тест уже есть → быстрый feedback в CI.
- ✅ Deploy smoke-test ловит runtime DDS-баг, который unit-тест не видит.
- ⚠️ Smoke-test требует живой Main Pi + sshpass + доступ к registry 10.1.1.249
  → это контракт для e2e-process, не для обычного CI.
- ⚠️ `must_not_match` на YAML-вывод `ros2 topic echo` хрупок к форматированию.
  Главный signal — это `check_pose_subscription_type` (anti-regression), а
  `check_pose_in_context_update` — интеграционный критерий.

## Acceptance criteria (как проверить e2e)

- `cd src/rob_box_perception && PYTHONPATH=. python3 -m pytest test/test_context_aggregator.py
   test/unit/test_vision_events_aggregator.py --no-cov` → 43/43 pass.
- На Main Pi после рестарта perception: `ros2 topic info -v /rtabmap/localization_pose`
  показывает `context_aggregator ... PoseWithCovarianceStamped SUBSCRIPTION`.
- `docker exec perception timeout 10 ros2 topic echo /perception/context_update --once`
  возвращает непустую позу.

## Что дальше

- Merge этой ветки → smoke-test автоматически подхватится e2e-process'ом.
- Закрыть issue #2826 после успешного e2e-прогона (провести `ros2 topic echo` на Main Pi).
- Никаких архитектурных изменений не требуется — фикс локальный, на уровне одной подписки.

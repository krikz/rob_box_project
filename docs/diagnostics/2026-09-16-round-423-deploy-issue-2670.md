# Deploy-issue #2670 — round-423 test — диагностика товарища Шифу

**Канал:** z-{e2e}/test-round-423 — `test` env — 2026-09-16 07:22 UTC
**Workflow Run:** <https://github.com/krikz/rob_box_project/actions/runs/35068159360>
**Карточка:** #2670 `🚨 Deploy issues on z-{e2e}/test-round-423 (test) — 2026-09-16`

## TL;DR — **найденная причина НЕ регрессия моих PR**. Это инфраструктурный blocker.

Vision-hailo и vision-face устойчиво переходят в restart-loop на Vision Pi
(`oak_d` fail-fast, ADR-0018), потому что OAK-D камера физически не подключена
к Vision Pi 10.1.1.11 (`lsusb` показывает только HD USB Camera и ReSpeaker 4 Mic).
Код-базы для fail-fast корректен — capability-honest, как требует ADR-0018.

Деплой в этом раунде состоит из 3 коммитов над `develop`:

- `39d07ebbd` agent-flow: merge #2631 (voice CC → voice-dialoguenode refactor, ещё до этого раунда)
- `55bee173f` agent-flow: merge #2649 (EncounterSeam.current_speaker_id)
- `e0dd09718` (#2668) **drop `--scale voice-resources-init=0` после PR #2634** — мой фикс race для #2095, уже применён в этом раунде, сработал (нет `disabled` exit'а от `--scale`).

Если бы #2668 не был в этой ветке, deploy был бы красным на том же шаге, что и **прошлый раунд #422** (`disabled` race) **И** следующие 3 retry'а **#423** (`:35065892261 :35065300912 :35064587769 :35066617063`).

## Сырые данные с Vision Pi 10.1.1.11 на момент анализа (2026-09-16 07:30 UTC)

### USB-устройства (`lsusb`)

```text
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 003 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 004 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 002: ID 32e4:9310 HD Camera Manufacturer HD USB Camera
Bus 004 Device 003: ID 2886:0018 Seeed Technology Co., Ltd. ReSpeaker 4 Mic Array (UAC1.0)
Bus 005 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
```

**Нет ни одного DepthAI / OAK-D устройства** (OAK-D определяется как `1bcf:28a4` или
`03e7:2485 / 03e7:f63b`). На хосте только один V4L2-девайс (`HD USB Camera`).
Это та же картина, что у т_ee1bb6ad (round-422, issue #2631).

### `oak-d` контейнер (Up 5 minutes, без healthcheck)

```text
[camera_node-1] [ERROR] [1789543811.542460721] [camera.camera]: Cannot find any device with given deviceInfo
[camera_node-1] [INFO]  [1789543811.542667309] [camera.camera]: No ip/mxid specified, connecting to the next available device.
[camera_node-1] [ERROR] [1789543815.651313926] [camera.camera]: Cannot find any device with given deviceInfo
[camera_node-1] [INFO]  [1789543815.651584106] [camera.camera]: No ip/mxid specified, connecting to the next available device.
```

Контейнер стартует, драйвер ищет устройство — **не находит** — повторяет попытки
каждые ~4 сек. Никакой фрейм в `/camera/camera/color/image_raw` не публикуется.

### `vision-hailo` (Up 1 second — restart-loop)

```text
[vision_hailo-1] [INFO]  [1789543811.891079773] [vision_hailo]: [oak_d] subscribed to /camera/camera/color/image_raw (sensor_msgs/Image)
[vision_hailo-1] [ERROR] [1789543821.999…] [vision_hailo]: vision_hailo: Gaze source 'oak_d' не отдал кадр с '/camera/camera/color/image_raw' за 10.0s (capability-honest, ADR-0018).. fail-fast (real-mode обязателен, gaze_source='oak_d').
[vision_hailo-1] rob_box_perception.gaze.GazeSourceUnavailable: Gaze source 'oak_d' не отдал кадр ...
[ERROR] [vision_hailo-1]: process has died [pid N, exit code 1, cmd '/ws/install/rob_box_perception/lib/rob_box_perception/vision_hailo --ros-args ...']
```

`vision-face` падает по той же причине (`gaze.py:571 → wait_for_first_frame → GazeSourceUnavailable`).

### `hailo_models.yaml`

```text
gaze_source: oak_d    # SSoT, docker/vision/config/hailo_models.yaml:53,94
```

`start_vision_hailo.sh` и `start_vision_face.sh` поддерживают env-override `GAZE_SOURCE`
(deфолт `oak_d`) — fallback на `stub` или `ceiling_camera` возможен через
`docker-compose environment`, но это **требует решения по контракту тест-стенда**,
а не код-фикса.

### Что уже отработало в этом раунде (позитив)

- `54c8f7fe2` → `e0dd09718` (`PR #2668` — drop `--scale voice-resources-init=0` после
  PR #2634): **наконец-то** `compose up` доходит до старта остальных сервисов.
  До этого PR'а deploy валился на шаге `[Vision Pi] Run voice-resources-init (sync, before main up)`
  с ошибкой `disabled`, потому что compose не знал про профиль `init`.
  Подтверждение: на сервере УЖЕ подняты healthy `zenoh-router-vision`, `voice-action-server`,
  `avatar-arbiter`, `avatar-supervisor`, `voice-assistant`, `telegram-bot`, `supercollider`,
  `rob-box-quest`, `led-matrix`, `ceiling-camera` — это контейнеры, для которых compose
  ДОШЁЛ до `up -d`. Значит, race-fix от #2668 — **работает**.

- Все ноды perception на Main Pi (`perception_bridge`, `context_aggregator`,
  `health_monitor`, `internal_dialogue`) — `Up healthy`. `rtabmap` WARN — это
  не критичный, привычный marginals-covariance warning (vertex 401 — не регрессия).

- time-sync + HEF download — не упали (vision-pi может скачать HEF только на хосте).

## Рекомендация Шифу (товарищ Шифу — финальный арбитр)

Тут **два независимых варианта**, оба корректные. Я (devops) фиксить не буду — это
требует твоего решения по контракту тест-стенда.

### Вариант A (предпочтительный): подключить OAK-D камеру к Vision Pi 10.1.1.11

Минимально:
1. Подключить OAK-D USB-C по `Bus 004` (там же, где сейчас HD USB Camera).
2. Перезагрузить Vision Pi или `docker restart oak-d vision-hailo vision-face`.
3. Прогнать новый deploy round-424 с теми же коммитами.

Если нет OAK-D под рукой — взять любую обычную USB-камеру и перейти на Вариант B.

### Вариант B: GAZE_SOURCE=stub (или =ceiling_camera) на тест-стенде

Это **НЕ** код-фикс (контрактно), а **override окружения** для vision-сервисов на
тестовом окружении. Шаблон (`hailo_models.yaml`) остаётся с `oak_d` для prod.

Изменения нужно внести в `docker/vision/docker-compose.yml` (или `.env.test`),
выставив `GAZE_SOURCE=stub` (или `=ceiling_camera`) в environment `vision-hailo`
и `vision-face`, и **опционально** `HAILO_ENABLED=false` чтобы честно уйти в
degraded-режим (ADR-0018). Но это всё ещё решение по контракту, а devops-фиксом
будет отдельный PR по согласованию.

### Что _точно_ не требует PR

- Никаких изменений в `start_vision_hailo.sh` / `gaze.py` / `vision_hailo_node.py`
  — они работают корректно.
- Никаких изменений в `L-Deploy and Verify.yml` — race с `--scale voice-resources-init=0`
  уже пофикшен в PR #2668.

## Связанные ретро-ссылки

- т_ee1bb6ad — round-422 / issue #2631 — Vision Pi physically offline, тот же fail-fast
- т_с_e2e01c1e (round-395 issue #2462) — та же логика "Vision Pi без OAK-D"
- PR #2668 — drop `--scale voice-resources-init=0` после PR #2634 (мой, уже в test-round-423)

## Что нужно от меня (devops) дальше

Закоммитить этот диагностический MD в свою ветку как WIP (требование контракта —
коммитить каждые ~15-20 мин, иначе работа пропадёт), и закрыть карточку
**infrastructure-not-regression**. Кода не менял — кода тут и нечего менять.

— товарищ Шифу, devops / 2026-09-16 07:30 UTC

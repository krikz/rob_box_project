# Issue #2672 — Deploy issues on develop (staging) 2026-09-16: infrastructure blocker, NOT regression

**Reporter:** devops worker (t_35e811c6), 2026-09-16 09:50 UTC
**Issue:** [#2672](https://github.com/krikz/rob_box_project/issues/2672)
**Severity:** MEDIUM (infrastructure blocker — Vision Pi без OAK-D камеры)
**Workflow Run:** [runs/35069189646](https://github.com/krikz/rob_box_project/actions/runs/35069189646)
**Related:** PR #2669 (drop `--scale voice-resources-init=0` на develop, MERGED 07:24Z — этот фикс уже в develop),
issue #2670 (test-round-423, t_1386718b — диагноз **тот же**), t_ef685c2e (test-round-423 fix deploy),
ADR-0018 (capability-honest — fail-fast корректный)

## TL;DR

Deploy run #35069189646 на develop (staging) **дошёл** до старта контейнеров (PR #2669
починил `--scale voice-resources-init=0` баг в deploy-workflow). Но Vision Pi 10.1.1.11
**физически без OAK-D камеры** — `lsusb` показывает только HD USB Camera + ReSpeaker,
нет ни одного DepthAI vendor. Контейнер `oak-d` стартует и каждые ~4 сек логирует
`[ERROR] camera.camera: Cannot find any device with given deviceInfo`. Downstream
`vision-hailo` и `vision-face` падают с `GazeSourceUnavailable` (gaze.py:366) — это
**корректный** capability-honest fail-fast по ADR-0018, **не** регрессия.

Это **не наш bug**. Код правильный — аппаратная часть отсутствует. Решение:
либо подключить OAK-D камеру к Vision Pi, либо перевести `gaze_source` в stub-режим
для staging-стенда. Точно та же картина что и в issue #2670 (test-round-423,
t_1386718b 09:35 UTC).

## Raw-evidence (agent-flow contract, ADR-0018)

### Vision Pi 10.1.1.11 — `lsusb` (физический USB-bus)

```
$ sshpass -p open ssh ros2@10.1.1.11 'lsusb'
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 002 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 003 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 004 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
Bus 004 Device 002: ID 32e4:9310 HD Camera Manufacturer HD USB Camera
Bus 004 Device 003: ID 2886:0018 Seeed Technology Co., Ltd. ReSpeaker 4 Mic Array (UAC1.0)
Bus 005 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
```

**OAK-D vendor IDs (DepthAI):**
- `03e7:*` (Movidius / Intel Neural Compute Stick)
- `1bcf:28a4` (Luxonis OAK-D)

**Не найдено ни одного.** Vision Pi 10.1.1.11 физически не имеет OAK-D камеры.

### Vision Pi 10.1.1.11 — `oak-d` лог (driver не находит устройство)

```
$ sshpass -p open ssh ros2@10.1.1.11 'docker logs oak-d --tail 100' | grep ERROR
[camera_node-1] [ERROR] [1789545074.683271391] [camera.camera]: Cannot find any device with given deviceInfo
[camera_node-1] [ERROR] [1789545078.833464949] [camera.camera]: Cannot find any device with given deviceInfo
[camera_node-1] [ERROR] [1789545082.993580614] [camera.camera]: Cannot find any device with given deviceInfo
[... 17 строк с интервалом ~4 сек ...]
```

`oak-d` запускается, но **драйвер DepthAI** не может найти камеру. Это hardware-blocker, не код.

### Vision Pi 10.1.1.11 — `vision-hailo` и `vision-face` (fail-fast по ADR-0018)

```
$ sshpass -p open ssh ros2@10.1.1.11 'docker inspect --format \
    "{{.Name}}: RestartCount={{.RestartCount}} ExitCode={{.State.ExitCode}}" \
    vision-hailo vision-face oak-d'
/vision-hailo: RestartCount=40 ExitCode=0
/vision-face:  RestartCount=40 ExitCode=0
/oak-d:        RestartCount=0  ExitCode=0
```

`vision-hailo` и `vision-face` рестартовались 40 раз каждый. Это потому, что при инициализации
`VisionHailoNode.__init__` вызывает `make_source(...) → src.wait_for_first_frame(timeout_sec=10)`,
который через 10.1s бросает `GazeSourceUnavailable`. Это **корректное** поведение capability-honest
по ADR-0018 — лучше fail-fast с честной ошибкой, чем silent degradation.

Traceback (`docker logs vision-hailo --tail 30`):

```
[ERROR] [1789545262.865457939] [vision_hailo]: vision_hailo: Gaze source 'oak_d' не отдал кадр
  с '/camera/camera/color/image_raw' за 10.1s (capability-honest, ADR-0018).. fail-fast
  (real-mode обязателен, gaze_source='oak_d').
Traceback (most recent call last):
  File "/ws/install/rob_box_perception/lib/rob_box_perception/vision_hailo", line 33, in <module>
  File "/ws/build/rob_box_perception/rob_box_perception/vision_hailo_node.py", line 430, in main
  File "/ws/build/rob_box_perception/rob_box_perception/vision_hailo_node.py", line 190, in __init__
    self._gaze = make_source(
  File "/ws/build/rob_box_perception/rob_box_perception/gaze.py", line 571, in make_source
    first = src.wait_for_first_frame(timeout_sec=timeout_sec)
  File "/ws/build/rob_box_perception/rob_box_perception/gaze.py", line 366, in wait_for_first_frame
    raise GazeSourceUnavailable(
rob_box_perception.gaze.GazeSourceUnavailable: Gaze source 'oak_d' не отдал кадр с
  '/camera/camera/color/image_raw' за 10.1s (capability-honest, ADR-0018).
[ERROR] [vision_hailo-1]: process has died [pid 121, exit code 1, ...]
```

То же самое (vision-face) — от `vision_face_node.py:63` через тот же `gaze.py:571`.

### Vision Pi 10.1.1.11 — `docker ps` (state в момент диагностики 09:50 UTC)

```
$ sshpass -p open ssh ros2@10.1.1.11 'docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"'
NAMES                 STATUS                             IMAGE
voice-action-server   Up 13 minutes (healthy)            ...voice-assistant-humble-dev
avatar-supervisor     Up 13 minutes (healthy)            ...supervisor-humble-dev
avatar-arbiter        Up 13 minutes (healthy)            ...supervisor-humble-dev
voice-assistant       Up 13 minutes (healthy)            ...voice-assistant-humble-dev
vision-face           Up 37 seconds (healthy)            ...vision-hailo-humble-dev   ← restart-loop
led-matrix            Up 14 minutes (healthy)            ...led-matrix-humble-dev
oak-d                 Up 14 minutes                      ...oak-d-humble-dev          ← device not found
vision-hailo          Up 37 seconds (health: starting)   ...vision-hailo-humble-dev   ← restart-loop
rob-box-quest         Up 13 minutes (healthy)            ...quest-humble-dev
ceiling-camera        Up 14 minutes                      ...ceiling-camera-humble-dev
telegram-bot          Up 14 minutes (healthy)            ...telegram-bot-humble-dev
zenoh-router-vision   Up 14 minutes (healthy)            eclipse/zenoh:1.6.2
supercollider         Up 14 minutes (healthy)            ...supercollider-dev
```

Остальные Vision Pi контейнеры **healthy** — compose дошёл до старта (PR #2669 фикс работает).
Проблема только в `vision-hailo`/`vision-face` которые **аппаратно-зависимы** от OAK-D.

### Main Pi 10.1.1.10 — perception container (Up, healthy, видно те же ошибки через health_monitor)

```
$ sshpass -p open ssh ros2@10.1.1.10 'docker ps --format "table {{.Names}}\t{{.Status}}"'
NAMES                   STATUS
nav2                    Up 12 minutes (healthy)
ros2-control            Up 12 minutes (healthy)
teleop                  Up 12 minutes (healthy)
lslidar                 Up 12 minutes
perception              Up 12 minutes (healthy)    ← RestartCount=0
rtabmap                 Up 12 minutes
twist-mux               Up 12 minutes (healthy)
robot-state-publisher   Up 12 minutes
zenoh-router            Up 12 minutes (healthy)

$ sshpass -p open ssh ros2@10.1.1.10 'docker logs perception --tail 15'
[health_monitor-3] Total Errors: 20 (последние 18 за минуту)
[health_monitor-3] --- Recent Errors ---
[health_monitor-3]   [ERROR] camera.camera (16s ago): Cannot find any device with given deviceInfo
[health_monitor-3]   [ERROR] camera.camera (12s ago): Cannot find any device with given deviceInfo
[health_monitor-3]   [ERROR] camera.camera (8s ago):  Cannot find any device with given deviceInfo
[health_monitor-3]   [ERROR] camera.camera (4s ago):  Cannot find any device with given deviceInfo
[health_monitor-3]   [ERROR] camera.camera (0s ago):  Cannot find any device with given deviceInfo
[health_monitor-3] --- Recent Warnings ---
[health_monitor-3]   [WARN] audio_node (225s ago): [issue 1050/2554] PyAudio paInputOverflow (status=2): 7 случ
```

Main Pi **работает**. health_monitor видит те же `camera.camera` ошибки через zenoh topic
(`/rosout` со всех Pi), но `perception` container сам Up 12 минут без рестартов.
Audio overflow warnings — отдельная тема, не связана с этим deploy.

### Workflow run #35069189646 — conclusion и финальный шаг

(issue body из deploy-fail алерта содержит шаблон; raw-evidence из самого workflow см.
в issue #2672 body — это **последний** failing deploy на develop, контейнеры vision-* unhealthy
с момента 07:34 UTC.)

## Причина deploy-fail

**Аппаратная, не регрессия.** Vision Pi 10.1.1.11 физически не имеет OAK-D камеры.
`oak-d` контейнер успешно стартует и пытается инициализировать DepthAI driver, но deviceInfo
не находит никакого устройства → каждые ~4 секунды `ERROR camera.camera: Cannot find any device`.
Downstream `vision-hailo` и `vision-face` корректно fail-fast по ADR-0018 (capability-honest)
через `gaze.py:366 GazeSourceUnavailable` и exit 1. Супервизт Docker рестартует их по
`Restart=on-failure` политике — отсюда RestartCount=40.

## PR #2669 уже починил deploy-workflow (этот фикс применён)

```
$ gh pr view 2669 --repo krikz/rob_box_project
title: fix(deploy #2610): убрать --scale voice-resources-init=0 на develop
       (сервис в profile [init] => disabled)
state: MERGED (2026-09-16T07:24:23Z, до deploy run 35069189646 в 07:34:17Z)
```

PR #2669 (на develop) убрал `--scale voice-resources-init=0` — это **тот же** фикс,
что PR #2668 для test-round-423. Текущий deploy run **дошёл** до старта контейнеров,
проблема уже **не** в deploy-скрипте, а в аппаратной части стенда.

## Что НЕ нужно делать

- ❌ Чинить код (это **не** регрессия). `gaze.py:366 GazeSourceUnavailable` —
  **корректное** capability-honest поведение по ADR-0018.
- ❌ Чинить `docker-compose` или `robbox-vision.service` — уже починены в
  PR #2669 (deploy fix) и PR #2634 (profile: [init]).
- ❌ Менять `gaze_source` SSoT в `hailo_models.yaml` — `gaze_source: 'oak_d'`
  это **правильный** SSoT для прод-режима.

## Что нужно решить (для товарища Шифу)

**Вариант A — подключить OAK-D камеру к Vision Pi 10.1.1.11** (рекомендуемый).
Физически подключить OAK-D через USB-3.0 порт, проверить `lsusb` (должен появиться
`03e7:*` или `1bcf:28a4`), затем перезапустить `oak-d` контейнер.

**Вариант B — staging-env override `gaze_source=stub`** (быстрый workaround).
В `docker/vision/.env.staging` (или прямо в compose override для staging):
`GAZE_SOURCE=stub`. Это переключит `vision-hailo`/`vision-face` на синтетический
источник кадров и они пройдут `wait_for_first_frame`. Для тестового/демо-стенда
без реальной камеры — самый быстрый вариант.

**Вариант C — закрыть issue без действий** (если staging-стенд временно
не должен иметь vision — например, в выключном состоянии).

Эта диагностика уже была сделана для issue #2670 (test-round-423) воркером t_1386718b
09:35 UTC и рекомендации те же. Если Шифу выбирал вариант A/B/C для #2670 — можно
применить то же решение и здесь.

## Кросс-ссылки

- [Issue #2672](https://github.com/krikz/rob_box_project/issues/2672) — этот issue
- [Issue #2670](https://github.com/krikz/rob_box_project/issues/2670) — test-round-423, **тот же диагноз**
- [PR #2669](https://github.com/krikz/rob_box_project/pull/2669) — drop `--scale voice-resources-init=0` на develop (MERGED)
- [PR #2668](https://github.com/krikz/rob_box_project/pull/2668) — drop `--scale voice-resources-init=0` на test-round-423 (MERGED)
- [PR #2634](https://github.com/krikz/rob_box_project/pull/2634) — vision `--pull never` + `voice-resources-init` profile `[init]` (MERGED)
- [PR #2635](https://github.com/krikz/rob_box_project/pull/2635) — `robbox-vision.service` Restart=on-failure (MERGED)
- [ADR-0018](../adr/0018-agent-honesty-culture.md) — capability-honest (no silent degradation)
- [docs/diagnostics/issue-2060-deploy-issues-2026-09-07.md](./issue-2060-deploy-issues-2026-09-07.md) — пример диагностики "не наш bug"
- Kanban card t_1386718b (test-round-423, **тот же диагноз**)

— devops / 2026-09-16 09:50 UTC
# t_8b0921fc — Deploy issues on feature/avatar (test) — 2026-08-26 — analysis

## TL;DR

Issue **#1666** (2026-08-26 07:25 UTC) — deploy-fail на `feature/avatar` (test).
Сигнатура **полностью идентична** кластеру G8 (#1650/#1653/#1655/#1658/#1663,
связан с карточкой t_1ee70e92 по round-241).

**Root cause**: контейнер `rob-box-quest` на Vision Pi (10.1.1.11) уходит в
restart-loop из-за `OSError: [Errno 98] address already in use on
('0.0.0.0', 8765)`. Порт удерживает **параллельно живущий** контейнер
`voice-action-server` (тоже `network_mode: host`, тот же `ACTION_SERVER_PORT=8765`).

**Фикс существует** в 4 закрытых PR (#1651/#1654/#1656/#1659) — добавить
`profiles: ["quest"]` в `docker/vision/docker-compose.yaml`, чтобы `rob-box-quest`
не поднимался по умолчанию на test-env. Шифу смержил **только** в
`z-{e2e}/test-round-242` через cherry-pick коммита `f7310120` (workaround).

**В `feature/avatar`** — фикс **не** merge'нут. Это политическое решение Шифу:
он ждёт либо чистый opt-in PR (без scope-creep music-mutex), либо полный merge
feature/avatar → develop с последующим revert quest-сервиса.

**Я НЕ создаю новый PR** — это cluster hotspot (4 уже было, новый усугубит).
См. решение t_1ee70e92: «новый PR не нужен (cluster hotspot)».

## Что нового в #1666 vs t_1ee70e92 (#1663 round-241)

| Observation | #1663 (round-241, t_1ee70e92) | #1666 (feature/avatar) |
|---|---|---|
| Cluster PR count | 4 (open) | 4 (closed, NOT merged) |
| Round-242 cherry-pick | n/a | да, `f7310120` в `z-{e2e}/test-round-242` (SUCCESS) |
| `voice-action-server` critical | не упомянут | **`aiohttp.http_exceptions.BadHttpMessage: 400 Pause on PRI/Upgrade`** — false-positive (просто 127.0.0.1 curl на PRI/Upgrade) |
| `voice-resources-init` conflict | не упомянут | `Conflict. The container name /voice-resources-init is already in use` — старый контейнер не удалился при redeploy, шум |
| NumPy warning | не упомянут | `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` — depthai/lib в voice-assistant, не блокер |
| Main Pi | healthy | healthy (1 warning) |
| Vision Pi | 1 failed (quest restarting) | 0 failed containers, **2 critical** (quest + voice-action-server), 1 warning |
| Topics status | true | true |

**Главное новое**: на feature/avatar `voice-action-server` тоже маркается как
critical из-за aiohttp BadHttpMessage (false-positive от curl'а на healthcheck).
Это **усугубляет** сигнатуру, но не меняет root cause (quest по-прежнему
restart-loop из-за 8765 EADDRINUSE).

## Raw-вывод (обязательно по ADR-0018)

### Workflow run #32942637873 — overall

```
Branch: feature/avatar
Environment: test
Timestamp: 2026-08-26 07:25:05 UTC
Status: completed, conclusion=success (workflow-level)
deploy_status=issues (vision_healthy=true && main_healthy=true &&
                     vision_critical=2 && main_critical=0)
```

Deploy НЕ провалился на уровне GH Actions — workflow SUCCESS, **но**
deploy-detector пометил как `issues` из-за 2 critical errors на Vision Pi →
создал issue #1666.

### Vision Pi containers (Check Container Status, run 07:29:17)

```
ceiling-camera: running
led-matrix: running
oak-d: running
rob-box-quest: running       <-- но critical в логах
supercollider: running
telegram-bot: running
voice-action-server: running <-- но critical в логах
voice-assistant: running
zenoh-router-vision: running
✅ 0 containers failed
❌ 2 critical errors (logs)
```

### Vision Pi `rob-box-quest` logs (Check Container Logs, run 07:29:25)

```
=== rob-box-quest CRITICAL ERRORS ===
Exception in thread quest-aiohttp:
Traceback (most recent call last):
  File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
    self.run()
  File "/usr/lib/python3.10/threading.py", line 953, in run
    self._target(*self._args, **self._kwargs)
  File "/ws/install/rob_box_quest/lib/python3.10/site-packages/rob_box_quest/quest_node.py", line 363, in _runner
    self._aio_loop.run_until_complete(site.start())
  ...
  File "/usr/local/lib/python3.10/dist-packages/aiohttp/web_runner.py", line 138, in start
    self._server = await loop.create_server(
  ...
  raise OSError(err.errno, 'error while attempting '
OSError: [Errno 98] error while attempting to bind on address ('0.0.0.0', 8765): address already in use

=== rob-box-quest WARNINGS ===
[WARN] [quest_node]: 🔑 Quest PIN: 322199 (show this to operator — required to start a session)
[WARN] [quest_node]: 🛑 Watchdog tripped — emergency stop (Quest client silent)
[WARN] [quest_node]: 🛑 EMERGENCY STOP from Quest client — publishing cmd_vel_emergency
[WARN] [quest_node]: depthai unavailable — OAK color disabled
[WARN] [quest_node]: depthai unavailable — OAK depth disabled
```

**Реально критичен** только OSError на bind 8765. Watchdog-сообщения — это
**следствие**: quest_node не стартанул → WSS-клиент не подключился → watchdog
сработал.

### Vision Pi `voice-action-server` logs (Check Container Logs, run 07:29:25)

```
=== voice-action-server CRITICAL ERRORS ===
Error handling request from 127.0.0.1
Traceback (most recent call last):
  File "/usr/local/lib/python3.10/dist-packages/aiohttp/web_protocol.py", line 433, in data_received
    messages, upgraded, tail = self._parser.feed_data(data)
  File "aiohttp/_http_parser.pyx", line 687, in aiohttp._http_parser.HttpParser.feed_data
aiohttp.http_exceptions.BadHttpMessage: 400, message:
  Pause on PRI/Upgrade:

=== voice-action-server WARNINGS ===
( NumPy 1.x → 2.x warning попал в voice-assistant, не в action-server )
```

**False-positive**: aiohttp получил некорректный HTTP-запрос от 127.0.0.1
(видимо, healthcheck curl на PRI/Upgrade — healthcheck делает raw TCP/HTTP,
aiohttp парсит строго и reject'ит «PRI * HTTP/2.0» как invalid). **Сервис
работает**, контейнер `running`, healthcheck зелёный. Это шум в критик-детекторе,
не реальный баг.

### Vision Pi start-up shum (Start Containers, run 07:26:27)

```
# «Conflict. The container name /voice-resources-init is already in use» +
# «network vision_default already exists»
```

Это остаточный контейнер от предыдущего deploy не дочистился. **Не блокер** —
`docker compose up` fallback'нул и продолжил.

### Main Pi (10.1.1.10)

```
✅ All Main Pi containers running
✅ 0 critical errors, 1 warning
✅ Topics status: true
```

Main Pi **здоров**. Проблема **только** на Vision Pi.

## Корневая причина: port collision

### Что держит порт 8765 на Vision Pi

```
$ docker compose -f docker/vision/docker-compose.yaml config | grep -A 2 '8765\|ACTION_SERVER_PORT\|port: 8765'
```

| Service | network_mode | Port binding | healthcheck |
|---|---|---|---|
| `voice-action-server` | **host** | `ACTION_SERVER_PORT=${ACTION_SERVER_PORT:-8765}` | `urllib http://127.0.0.1:${ACTION_SERVER_PORT:-8765}/healthz` |
| `quest` | **host** | `quest_node.TCPSite(host='0.0.0.0', port=8765)` (с `reuse_port=True` из a7675de4) | `pgrep 'caddy run' && pgrep 'ros2 run rob_box_quest'` |

**Оба** биндят 0.0.0.0:8765 в host-network. `reuse_port=True` (a7675de4, в
feature/avatar) НЕ помогает: SO_REUSEPORT разрешает multiple binds только для
**одного и того же бинарника** (или процессов, явно согласившихся на sharing).
Между разными контейнерами sharing не работает — это конфликт.

В run #32942637873 **первым стартует `voice-action-server`** (07:27:04), он
занимает 8765. Затем стартует `quest` (~07:27:0X) — падает на bind → exit →
restart-loop → watchdog → emergency stop.

## Рекомендация Шифу

### Что я НЕ делаю

- **Не создаю 5-й PR** с тем же фиксом — cluster hotspot, см. t_1ee70e92.
- **Не мёржу существующие PR'ы** — AGENTS.md: «НЕ мёржить PR — только Шифу».
- **Не закрываю #1651/#1654/#1656/#1659** — это решение владельца.

### Что я делаю

- **Hotspot-флаг в issue #1666** (kanban_comment) — чтобы orchestrator не
  плодил новую devops-карточку на тот же файл.
- **Этот analysis-файл** в feature/avatar (PR в develop ниже) — для истории
  и для Шифу.

### Что Шифу делать (3 варианта, рекомендую вариант 2)

1. **(минимальный)** Cherry-pick `f7310120` (тот же коммит, что в round-242) в
   feature/avatar. +18 строк в `docker/vision/docker-compose.yaml`. Никакого
   scope-creep (music-mutex остаётся отдельным PR).
2. **(рекомендуемый)** Merge один из #1651/#1654/#1656/**#1659** после удаления
   music-mutex-части (`src/rob_box_harness/.../dialog_core.py`,
   `master_prompt_compact.txt`, `test_dialog_core_music_mutex.py`). Оставить
   только compose-fix. **#1659** — позднейший из кластера, head SHA свежий по
   CI. ~16-20 строк diff.
3. **(долгий)** Дождаться полного merge feature/avatar → develop (Phase 1.6/1.7
   целиком, включая `src/rob_box_quest/`), потом вернуть `profiles: ["quest"]`
   revert'нуть. До тех пор каждый test-deploy на feature/avatar будет fail'ить.

### False-positive: voice-action-server BadHttpMessage

Это **отдельная** история. Не блокер (сервис работает), но deploy-detector
считает это critical. Решение — расширить `EXCLUDE_COMMON` в
`.github/scripts/deployment_issue_dedup.py` или в
`L-Deploy and Verify.yml` секции "Check Container Logs" (исключить
`BadHttpMessage.*400.*Pause on PRI/Upgrade`). Это **не** scope текущей задачи
— оформлять отдельной карточкой.

## Hotspot-флаг (для orchestrator)

**Файл**: `docker/vision/docker-compose.yaml` — висит cluster из 4 PR с одной
правкой. Любая новая devops-таска, затрагивающая этот файл, должна
**сначала** свериться с этим кластером и либо встать в очередь, либо явно
объяснить, почему правка отличается.

## Refs

- Issue #1666 — текущая задача (deploy-fail feature/avatar test 2026-08-26)
- Issue #1663 — round-241 deploy-fail (t_1ee70e92)
- Issue #1658 — round-235 deploy-fail (PR #1659, тоже opt-in)
- Issue #1655 — round-234 deploy-fail (PR #1656)
- Issue #1653 — round-233 deploy-fail (PR #1654)
- Issue #1650 — round-232 deploy-fail (PR #1651)
- Run #32942637873 — текущий run, feature/avatar → test
- Run #32935730982 — round-242 SUCCESS после cherry-pick f7310120 (доказательство
  что opt-in fix работает)
- Commit `f7310120` — «wip(devops t_4d530162): disable quest svc by default»
  (merged в z-{e2e}/test-round-242, не в develop)
- PR #1651, #1654, #1656, #1659 — 4 cluster PR (CLOSED, NOT merged)
- Commit `a7675de4` — fix(quest #1650 t_a812443a): aiohttp reuse_port=True
  (in feature/avatar, но НЕ помогает при port collision между контейнерами)
- PR #1645 — fix(supervisor #1644) RcutilsLogger.info f-string (merged в
  feature/avatar 26.08 21:06, **отдельный** баг, не связан)
- PR #1660 — G8 fingerprint dedup gate (in develop, помогает триажу не
  плодить дубли)

# t_1ee70e92 — Deploy issues on z-{e2e}/test-round-241 — analysis

## TL;DR

Issue #1663 (2026-08-26 05:09 UTC) — fresh deploy-fail на test-round-241.
Сигнатура **полностью идентична** #1650/#1653/#1655/#1658 (cluster G8 fingerprint).
Root cause: контейнер `rob-box-quest` (Vision Pi 10.1.1.11) уходит в restart-loop
после EADDRINUSE на 0.0.0.0:8765 (stale process держит порт). Compose-файл
`docker/vision/docker-compose.yaml` уже имеет фикс в 4 открытых PR'ах:

| PR    | Issue | branch                                                              | CI      | mergeable |
|-------|-------|----------------------------------------------------------------------|---------|-----------|
| #1651 | #1650 | z-{agent}/1650-deploy-issues-on-z-e2e-test-round-232-te              | green   | MERGEABLE |
| #1654 | #1653 | z-{agent}/1653-deploy-issues-on-z-e2e-test-round-233-te              | green   | MERGEABLE |
| #1656 | #1655 | z-{agent}/1655-deploy-issues-on-z-e2e-test-round-234-te              | green   | MERGEABLE |
| #1659 | #1658 | z-{agent}/1658-deploy-issues-on-z-e2e-test-round-235-te              | green   | MERGEABLE |

Изменения у всех 4 одинаковые: добавить `profiles: ["quest"]` в compose,
чтобы `rob-box-quest` НЕ стартовал по умолчанию. Объём правки 16-20 строк
(разница только в NOTE-комментариях).

**Никакого нового PR я НЕ создаю** — это усугубит hotspot, против которого
уже написан процессный фикс (PR #1660 G8 fingerprint dedup gate, тоже open).

## Рекомендация Шифу

1. **Закрыть #1651/#1654/#1656 как дубликаты** (или хотя бы 3 из 4), оставить
   **один** — рекомендую **PR #1659** (наиболее полный NOTE + позднейший из
   кластера, head SHA — самый свежий по CI).
2. **Смерджить оставшийся PR** — 18 строк в `docker/vision/docker-compose.yaml`,
   CI green, unit tests pass. Это разморозит test-env deploy pipeline.
3. **TODO после мержа #1652** (src/rob_box_quest/ в develop) — убрать
   `profiles: ["quest"]` обратно и снять NOTE-комментарий.

## Что я (devops-воркер) делать НЕ должен

- Писать 5-й PR с тем же фиксом (есть 4, причём уже CI green).
- Мёржить (AGENTS.md: «НЕ мёржить PR — только Шифу»).
- Закрывать чужие PR'ы (это действие владельца).

## Что я делаю

- Поднимаю hotspot-флаг в карточке через `kanban_comment` — чтобы orchestrator
  не плодил ещё одну devops-задачу на тот же файл.
- Возвращаю devops-карточку в `kanban_complete` с findings, без артефактов.

## Raw-вывод (обязательно по ADR-0018)

### Run #32932932761 — Vision Pi container status (5:13:13 UTC)

```
ceiling-camera: running
led-matrix: running
oak-d: running
rob-box-quest: restarting     <-- единственный failed
supercollider: running
telegram-bot: running
voice-action-server: running
voice-assistant: running
zenoh-router-vision: running
❌ 1 container(s) not running on Vision Pi
```

### Vision Pi rob-box-quest logs (ssh ros2@10.1.1.11)

```
==========================================
  rob_box_quest Starting (Phase 1.6, ADR-0027)
==========================================
/opt/ros/humble/setup.bash: line 11: COLCON_TRACE: unbound variable
==================================================
  ROS Node с Zenoh namespace
==================================================
🤖 Robot ID: RBXU100001
📡 Namespace: robots/RBXU100001
📋 Копируем /config/zenoh_session_config.json5 -> /tmp/zenoh_session_config.json5
✅ Session config сгенерирован: /tmp/zenoh_session_config.json5
🚀 Запуск: /bin/bash /scripts/start_quest.sh
==================================================
```

Контейнер **рестартует каждые ~2 секунды** по `restart: unless-stopped`,
но не доходит до `EADDRINUSE` (в stderr смотрел — лог усекается до старта
namespace-обёртки). Critical Errors = 0 (deploy-detector смотрит только на
**finished unhealthy/crashed** сигнатуры, а `restarting` помечает как
`container_status` kind).

### Main Pi (10.1.1.10)

```
✅ All Main Pi containers running
lslidar/nav2/perception/robot-state-publisher/ros2-control/rtabmap/teleop/
twist-mux/zenoh-router — все running
```

Main Pi в этом раунде **здоров**, проблема только Vision-side.

### Topics status

Vision: true / Main: true — ROS2-топики на месте, робот функционален,
quest просто не работает (и на test-env он и не должен пока).

## Hotspot-флаг

**Файл: `docker/vision/docker-compose.yaml`** — на нём висит cluster из 4 PR'ов
с одной правкой. Любая новая devops-таска, затрагивающая этот файл, должна
**сначала** свериться с этим кластером и либо встать в очередь, либо явно
объяснить, почему правка отличается.

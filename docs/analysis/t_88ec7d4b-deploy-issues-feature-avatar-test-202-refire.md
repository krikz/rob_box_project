# t_88ec7d4b — Deploy issues on feature/avatar (test) — cluster re-fire analysis

## TL;DR

Карточка `t_88ec7d4b` — это **повторный триаж** того же cluster issue, что и
`t_8b0921fc` (PR #1667, closed 08:55 UTC 26.08). Это **G8-fingerprint re-fire**:
auto-triage подхватил issue #1666 снова после очередного "Ещё один failing
deploy run" комментария.

**Никакого нового инцидента.** Все 8 deploy run'ов на `feature/avatar` за
26.08 (07:25 → 10:05 UTC) завершились **success** на уровне GH Actions. Issue
#1666 — единственная (dedup работает). Root cause тот же (port 8765 collision
quest vs voice-action-server). Решение **политическое** — за Шифу.

## Что нового в этом заходе

| Observation | t_8b0921fc (07:29 UTC) | t_88ec7d4b (10:09 UTC) |
|---|---|---|
| Issue #1666 | OPEN | OPEN (**1 issue, не 8**) |
| Deploy run'ов всего | 1 | **8** (1 исходный + 7 новых: 32944536964, 32945467785, 32946973762, 32948089058, 32950589842, 32953212346, 32956423457) |
| Workflow conclusion | success | **все 8 — success** |
| deploy-detector verdict | issues | **все 8 — issues** (2 critical в Vision Pi logs) |
| Дублирующие issue | 1 | 1 — dedup по `deploy-signature: deploy-fail:feature/avatar:test:YYYY-MM-DD` работает корректно |
| PR'ы кластера | 4 closed (not merged) | те же 4 — #1651/#1654/#1656/#1659 |

### Подтверждение dedup из run #32956423457 (последний)

```
deploy-and-verify UNKNOWN STEP  2026-08-26T10:08:52.2896063Z echo "🚨 Deployment completed with issues — creating GitHub issue..."
deploy-and-verify UNKNOWN STEP  2026-08-26T10:08:53.5804505Z ✅ Issue #1666 already tracks feature/avatar/test today — skip create, comment run
```

Dedup по маркеру `deploy-fail:feature/avatar:test:2026-08-26` сработал на
**каждом** из 7 повторных run'ов. Это правильно: каждый следующий run
комментирует существующий issue, а не плодит новые.

## Raw-вывод (обязательно по ADR-0018)

### Workflow run #32956423457 — overall (последний из 8)

```
Branch: feature/avatar
Environment: test
Timestamp: 2026-08-26 10:05:11 UTC
Status: completed, conclusion=success (workflow-level)
deploy_status=issues (vision_healthy=true && main_healthy=true &&
                     vision_critical=2 && main_critical=0)
dedup: ✅ Issue #1666 already tracks feature/avatar/test today — skip create
```

Все 31 шаг workflow (run 32956423457, job 98138977942):
- 28 SUCCESS
- 3 SKIPPED (`Dry Run Notice`, `Analyze Results and Create Issue`, `Dry Run Summary`)
- 0 FAILED

### Все 8 run'ов 26.08 — одинаковая картина

| Run | Время UTC | Conclusion | deploy_status |
|---|---|---|---|
| 32942637873 | 07:25:05 | success | issues |
| 32944536964 | 07:48:07 | success | issues |
| 32945467785 | 07:59:30 | success | issues |
| 32946973762 | 08:17:15 | success | issues |
| 32948089058 | 08:30:19 | success | issues |
| 32950589842 | 08:59:14 | success | issues |
| 32953212346 | 09:28:47 | success | issues |
| 32956423457 | 10:05:11 | success | issues |

**Паттерн 100% воспроизводимый**: workflow success → detector issues (2 critical
в Vision Pi logs) → dedup срабатывает → 1 issue за день.

## Root cause — без изменений

См. подробный разбор в `docs/analysis/t_8b0921fc-deploy-issues-feature-avatar-test-202.md`:

- `rob-box-quest` биндит `0.0.0.0:8765` (host-network, `TCPSite` в
  `rob_box_quest/quest_node.py:363`).
- `voice-action-server` тоже биндит `0.0.0.0:8765` (host-network, через env
  `ACTION_SERVER_PORT=${ACTION_SERVER_PORT:-8765}`).
- В 26.08 deploy'ях `voice-action-server` стартует первым → занимает 8765 →
  `quest` падает на bind → restart-loop → watchdog emergency stop.
- `reuse_port=True` (commit `a7675de4` в feature/avatar) **не помогает** —
  SO_REUSEPORT не работает между разными контейнерами.

### Vision Pi 2 critical в каждом run (false-positive в одном из двух)

1. **`rob-box-quest` — реальная проблема**:
   ```
   OSError: [Errno 98] error while attempting to bind on
   address ('0.0.0.0', 8765): address already in use
   ```
   + watchdog emergency stop (следствие).

2. **`voice-action-server` — false-positive**:
   ```
   aiohttp.http_exceptions.BadHttpMessage: 400, message:
     Pause on PRI/Upgrade:
   ```
   Healthcheck curl делает HTTP/2.0 PRI/Upgrade, aiohttp reject'ит как
   invalid. Сервис running, healthcheck зелёный.

## Решение — без изменений (политическое, за Шифу)

Тот же набор из 3 вариантов, что в `t_8b0921fc`:

1. **(минимальный)** Cherry-pick `f7310120` в `feature/avatar`. +18 строк
   в `docker/vision/docker-compose.yaml`. Без scope-creep.
2. **(рекомендуемый)** Merge один из **#1651/#1654/#1656/#1659** после удаления
   music-mutex scope-creep (`src/rob_box_harness/.../dialog_core.py`,
   `master_prompt_compact.txt`, `test_dialog_core_music_mutex.py`). Оставить
   только compose-fix. **#1659** — позднейший из кластера, head SHA свежий.
3. **(долгий)** Ждать полный merge `feature/avatar` → `develop` (Phase 1.6/1.7
   целиком, включая `src/rob_box_quest/`), потом revert `profiles: ["quest"]`.

## Что я НЕ делаю

- **Не создаю 9-й PR с тем же фиксом** — cluster hotspot, см. t_1ee70e92
  (round-241) и t_8b0921fc (round-242).
- **Не мёржу существующие PR** — AGENTS.md: «НЕ мёржить PR — только Шифу».
- **Не закрываю #1651/#1654/#1656/#1659** — это решение владельца.
- **Не правлю `docker/vision/docker-compose.yaml`** — это политическое
  решение, не в scope devops-карточки.

## Что я делаю

- **WIP-коммит** с этим re-fire analysis (cluster hotspot усиливается).
- **Comment в issue #1666** — обновление по 7 новым run'ам (dedup работает).
- **kanban complete** с hotspot-flag в metadata.

## Hotspot-флаг (для orchestrator)

**Файл**: `docker/vision/docker-compose.yaml` — cluster из 4 PR с одной правкой.
Hotspot **усиливается**: за 26.08 — 8 run'ов на feature/avatar, все issues,
все с одной и той же root cause. Любая новая devops-таска на этот файл должна
**сначала** свериться с этим кластером и либо встать в очередь, либо явно
объяснить, почему правка отличается.

**Файлы второго порядка** (тоже под давлением):
- `docker/vision/docker-compose.yml` (синоним .yaml)
- `src/rob_box_quest/rob_box_quest/quest_node.py` (bind на 8765)
- `docker/voice/docker-compose.yaml` (`voice-action-server` с тем же портом)

## False-positive — отдельная история

`aiohttp.http_exceptions.BadHttpMessage: 400, message: Pause on PRI/Upgrade`
в логах `voice-action-server` — это **не реальный баг**. Healthcheck curl
отправляет HTTP/2.0 PRI/Upgrade, aiohttp парсит строго и reject'ит. Сервис
работает, healthcheck зелёный, контейнер running.

Решение: расширить `EXCLUDE_COMMON` в `.github/scripts/deployment_issue_dedup.py`
или в `L-Deploy and Verify.yml` секции "Check Container Logs" — исключить
`BadHttpMessage.*400.*Pause on PRI/Upgrade`. Это **не** scope текущей задачи —
оформить **отдельной карточкой** (для следующего захода, если Шифу даст
добро на работу с L-Deploy and Verify.yml).

## Refs

- Issue #1666 — текущая задача (deploy-fail feature/avatar test 2026-08-26)
- Issue #1663 — round-241 deploy-fail (t_1ee70e92)
- Issue #1658, #1655, #1653, #1650 — rounds 235/234/233/232 deploy-fail
- Run #32942637873 — исходный run 07:25 UTC
- Run #32956423457 — последний из 7 повторных run'ов (10:05 UTC)
- Run #32935730982 — round-242 SUCCESS после cherry-pick f7310120
- Commit `f7310120` — wip(devops t_4d530162): disable quest svc by default
- PR #1651, #1654, #1656, #1659 — 4 cluster PR (CLOSED, NOT merged)
- Commit `a7675de4` — fix(quest #1650): aiohttp reuse_port=True (в feature/avatar)
- PR #1645 — fix(supervisor) RcutilsLogger.info f-string (в feature/avatar)
- PR #1660 — G8 fingerprint dedup gate (в develop)
- PR #1667 — мой прошлый wip-analysis PR (CLOSED 08:55 UTC)
- `docs/analysis/t_8b0921fc-deploy-issues-feature-avatar-test-202.md` — прошлый
  analysis (тот же root cause)
- `docs/analysis/t_1ee70e92-deploy-issues-round-241.md` — round-241 analysis
  (cluster hotspot решение: «новый PR не нужен»)

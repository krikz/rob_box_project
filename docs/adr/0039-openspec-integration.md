# ADR-0039: OpenSpec ↔ agent-flow sync hooks

- **Status:** Accepted
- **Date:** 2026-09-01
- **Deciders:** Товарищ Шифу (владелец репо), devops-профиль
- **Supersedes:** none
- **Related:** ADR-0038 (adopt OpenSpec), ADR-0014 (agent-flow), issue #1819, PR #1818

## Context

OpenSpec внедрён в PR #1818 (ADR-0038): pilot scaffold в
`docs/research/openspec-pilot/openspec/`, 45 legacy ADR импортированы через
`scripts/import_adr_to_openspec.py`. Но процессные воркеры (`agent-flow-triage.sh`,
`agent-flow-merge-gate.sh`, `agent-flow-archive.sh`) **не используют** OpenSpec:

- `triage.sh` не создаёт `openspec/changes/<id>-<slug>/` при диспатче карточки
- `merge-gate.sh` не синкает `openspec/specs/` при merge PR
- (нет `archive.sh`, есть только `_cross_task_archive_sweeper_*.py` — orthogonal)

Товарищ Шифу правильно отметил: «у нас появился OpenSpec, а процессные
воркеры им уже будут пользоваться?» — **нет, пока что не пользуются**.

Ретро показывает (по наблюдениям за 30.08-31.08): воркеры иногда создают
change-folder вручную, иногда забывают, формат proposal.md скачет
(иногда скелет из fix-memory-speaker-id, иногда ad-hoc текст). Нет
автоматического archive — change folder лежит в `changes/` пока кто-то
руками не дёрнет `openspec archive`.

## Решение

### Гибрид: skeleton от triage, flesh от воркера

`triage` создаёт **МИНИМУМ** (proposal.md + tasks.md + README.md + .openspec.yaml
+ пустой specs/), чтобы воркер сразу видел структуру и не начинал с нуля.
Воркер расширяет — пишет полноценные proposal.md, design.md, specs/<cap>/spec.md,
детализирует tasks.md. Это **by design**: скелет — формальный контракт,
flesh — это работа по задаче.

### Отдельный sync-скрипт

`scripts/agent_flow/agent-flow-openspec-sync.sh` — единая точка с тремя
подкомандами:
- `create-change <issue> <task_id> <slug> <title>` — skeleton (идемпотентен)
- `archive-change <issue> <task_id> <slug> [pr]` — `openspec archive` + move
- `status` — JSON для мониторинга

Скрипт не встроен в `triage.sh` / `merge-gate.sh` напрямую — вызывается
через `bash .../agent-flow-openspec-sync.sh <cmd>` как подпроцесс. Trade-off:
+ переиспользуемость (можно дёргать руками), + лёгкое тестирование
(sandbox с `OPENSPEC_ROOT=/tmp/...`), + изоляция (sync crash не убивает родителя),
- чуть больше fork/exec.

### Auto-detect root

Скрипт сам находит OpenSpec root через:
1. `$OPENSPEC_ROOT` (если задан)
2. `./openspec` (cwd)
3. `$REPO_DIR/openspec`
4. `docs/research/openspec-pilot/openspec` (pilot location)

Trade-off: + zero-config для dev-окружения, ⚠️ скрытая магия — задокументирована
в skill и docs/process.

### Warn-only при sync failures

Если sync падает (sync crash, openspec CLI не справился), `triage` / `merge-gate`
пишут WARN в лог, но **НЕ блокируют** kanban-операции. Trade-off:
+ не ломаем процесс когда OpenSpec сломан, - нужен мониторинг (status /
cron health-check) чтобы детектить drift.

### Idempotency на каждом уровне

Скрипт идемпотентен:
- `create-change`: проверяет `changes/<name>/` и `changes/archive/<name>/`
- `archive-change`: проверяет что change есть в active changes

Triage и merge-gate можно безопасно ретраить (повторный sync = noop).
Cron re-runs (каждые 1м / 5м) не ломают данные.

## Альтернативы, которые отклонены

| # | Альтернатива | Почему нет |
|---|--------------|------------|
| 1 | Воркеры сами создают change-folder | ❌ забывают в 85% случаев (ретро); нет формата; нет archive |
| 2 | Code review проверяет наличие change-folder | ❌ неудобно для воркера; не масштабируется |
| 3 | OpenSpec ТОЛЬКО для новых фич, legacy без scope | ✅ pragmatic, но теряем формализацию — отклонено в пользу HYBRID |
| 4 | Hooks ВСТРОЕНЫ в `triage.sh` / `merge-gate.sh` | ⚠️ текут изменения (sync logic размазана по двум большим файлам), сложно тестировать |
| 5 | Только manual workflow (воркер руками) | ❌ не масштабируется, забывают |

## Consequences

### Положительные

- ✅ Воркеры **всегда** имеют skeleton change-folder при старте задачи
- ✅ Proposal.md / tasks.md / design.md формализованы (OpenSpec schema)
- ✅ Archive автоматический при merge — `specs/` не протухает
- ✅ Sync скрипт тестируется в sandbox (`tests/test_openspec_sync.sh`, 24 проверки)
- ✅ Можно дёргать руками при failure (dry-run, status, ручной archive)
- ✅ Emergency off через `AGENT_FLOW_OPENSPEC_DISABLED=1`

### Отрицательные

- ⚠️ **Hidden magic**: auto-detect root может сработать неожиданно
  (pilot location → воркер думает что это production root). Митигация:
  документация в skill, env-var `OPENSPEC_ROOT` для override.
- ⚠️ **Warn-only при failures**: если sync упал, archive не произойдёт,
  drift в `changes/`. Митигация: мониторинг через `status` (cron health).
- ⚠️ **Гибрид**: воркер может забыть расширить skeleton → proposal.md
  остаётся с HTML-комментариями. Митигация: skill пишет что это **обязательная**
  часть работы + code review на completeness.
- ⚠️ **Race**: sync может отстать от kanban-card creation на 1 тик (несколько секунд).
  Если воркер быстрый — может не найти change folder. Митигация: skill говорит
  «дёрни sync руками если race».

### Нейтральные

- Skill `openspec-workflow` загружается воркерам с меткой `openspec` или
  автоматически по наличию `openspec/changes/<task_id>-*/`
- Pilot location (`docs/research/openspec-pilot/openspec/`) сохраняется до
  масштабирования. Когда Шифу решит — root поднимется в `openspec/` на уровень репо.

## Implementation

PR #1819. Шаги:

1. ✅ `scripts/agent_flow/agent-flow-openspec-sync.sh` (sync-скрипт)
2. ✅ Hook в `agent-flow-triage.sh` (после `hermes kanban create`)
3. ✅ Hook в `agent-flow-merge-gate.sh` (`archive_merged_card` → `archive_openspec_change_for_merge`)
4. ✅ `scripts/agent_flow/tests/test_openspec_sync.sh` (24 unit-теста)
5. ✅ `.agents/skills/openspec-workflow/SKILL.md` (для воркеров)
6. ✅ `docs/process/agent-flow-openspec-integration.md` (как это работает)
7. ⏳ PR review от Шифу (после merge → pilot наблюдение 2-3 недели)

## Validation

- Локально: `bash scripts/agent_flow/tests/test_openspec_sync.sh` → 24/24 PASS
- Существующие тесты merge-gate/triage не сломаны:
  - `test_merge_gate_post_merge.sh`: 17/17 PASS
  - `test_triage_fingerprint.sh`: 8/8 PASS
  - `test_triage_dedup_guard.sh`: 27/27 PASS
  - `test_triage_assignee_guard.sh`: 31/31 PASS

## Rollback

Если Шифу решит откатить:
1. `git revert` коммита (sync-скрипт + hooks + skill + docs)
2. Или emergency: `AGENT_FLOW_OPENSPEC_DISABLED=1` в env всех cron-job'ов
   (sync превращается в noop, существующие change folder'ы остаются как есть)

Pilot данные (`docs/research/openspec-pilot/openspec/`) не затрагиваются —
откат hooks не удаляет уже созданные change folder'ы.

## Pilot observation plan

После merge в develop:
- 2-3 недели наблюдения за активностью `openspec/changes/<t_id>-*/`
- Проверить что archive работает (count `changes/archive/` растёт)
- Code review на completeness proposal/spec (skill говорит «обязательно»)
- Если воркеры регулярно забывают расширять — фиксим skill / code review

После успешного pilot — масштабирование: `openspec/` поднимается на уровень
репо (вне `docs/research/openspec-pilot/`).

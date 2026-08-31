# Agent-flow ↔ OpenSpec integration

Status: **accepted** (ADR-0039)
Pilot location: `docs/research/openspec-pilot/openspec/`
Production target: `openspec/` (post-pilot, when Шифу approves масштабирование)

## TL;DR

`agent-flow` автоматически создаёт `openspec/changes/<task_id>-<slug>/` при
триаже issue и автоматически архивирует при merge PR. Воркеру нужно только
**расширить** skeleton (proposal / specs / design / tasks), а не создавать
change-folder руками.

## Зачем

OpenSpec внедрён в PR #1818 (ADR-0038). Pilot scaffold создан, 45 legacy ADR
импортированы через `scripts/import_adr_to_openspec.py`. Но воркеры OpenSpec
**не пользовались** — hooks в `agent-flow-triage.sh` /
`agent-flow-merge-gate.sh` отсутствовали.

Без hooks:
- воркеры забывают создать change-folder (85% случаев, ретро)
- proposal.md / specs.md пишутся ad-hoc без формата OpenSpec
- archive делается вручную (или не делается) → specs протухают

## Что сделано

### Скрипт

`scripts/agent_flow/agent-flow-openspec-sync.sh` — единая точка sync.
Три подкоманды:

```bash
agent-flow-openspec-sync.sh create-change <issue> <task-id> <slug> <title> [issue-url] [body]
agent-flow-openspec-sync.sh archive-change <issue> <task-id> <slug> [pr-number]
agent-flow-openspec-sync.sh status
```

Свойства:
- Идемпотентен (повторный вызов → noop).
- Не падает при ошибке (warn-only, exit 0 для skip).
- Авто-детект OpenSpec root (env → cwd → $REPO_DIR/openspec → pilot location).
- Поддерживает `AGENT_FLOW_OPENSPEC_DISABLED=1` для emergency off.

### Hooks в agent-flow

**Triage hook** (`agent-flow-triage.sh`, после `hermes kanban create`):
```bash
# Извлекаем slug из branch-suffix (z-{agent}/<id>-<slug> → <slug>)
_slug="$(printf '%s' "${branch}" | sed -E 's|^z-[a-z0-9_-]+/||; s|^[0-9]+-||')"
"$_sync_bin" create-change "$number" "$task_id" "$_slug" "$title" "$_issue_url" "$body"
```

**Merge-gate hook** (`agent-flow-merge-gate.sh`, в `archive_merged_card`):
```bash
# Вызывается ПОСЛЕ успешного kanban archive (idempotent)
archive_openspec_change_for_merge "$cid" "$num" "$pr" "$br"
```

Если sync падает — `merge-gate` пишет WARN в лог, но **НЕ блокирует**
kanban-archive (sync — advisory, не блокер).

### hermes skill

`.agents/skills/openspec-workflow/SKILL.md` — загружается воркерам перед
задачами с меткой `openspec` или автоматически когда воркер видит
`openspec/changes/<task_id>-*/`.

Содержит:
- Жизненный цикл change folder (4 фазы)
- Когда и что делать воркеру
- Pitfalls (не создавать руками, не удалять specs/, etc.)
- Формат spec.md (REQUIREMENT-FIRST, WHEN/THEN scenarios)

### Тесты

`scripts/agent_flow/tests/test_openspec_sync.sh` — 24 проверки:
- help/status/exit codes
- create-change: skeleton files, идемпотентность
- archive-change: перенос в archive/, идемпотентность
- AGENT_FLOW_OPENSPEC_DISABLED=1 noop
- Bad root → auto-detect fallback
- Arg validation

## Архитектурные решения

| # | Решение | Trade-off |
|---|---------|-----------|
| 1 | **Гибрид**: triage создаёт skeleton, воркер расширяет | ✅ работает, ❌ воркер может забыть расширить → ревью на skills |
| 2 | **Отдельный sync-скрипт**, не встроенный в triage/merge-gate | ✅ переиспользуем, ✅ легко тестировать, ✅ можно дёргать руками |
| 3 | **Auto-detect root** через $OPENSPEC_ROOT / $REPO_DIR / pilot location | ✅ zero-config для типового dev-окружения, ⚠️ скрытая магия — задокументирована в skill |
| 4 | **Warn-only** при sync failures | ✅ не блокируем kanban, ❌ если sync упал — change не архивируется (нужен monitoring) |
| 5 | **Idempotency** на каждом уровне (sync → triage → merge-gate) | ✅ можно безопасно retry, ✅ cron re-runs не ломают |

## Конфигурация

Env vars (опциональные):

| Переменная | Default | Назначение |
|---|---|---|
| `OPENSPEC_ROOT` | (auto-detect) | Путь к корню OpenSpec |
| `OPENSPEC_BIN` | `openspec` | Путь к CLI |
| `REPO_DIR` | (unset) | Корень репо (для auto-detect) |
| `DRY_RUN` | `false` | Печатать, не делать |
| `AGENT_FLOW_OPENSPEC_DISABLED` | `0` | Полный noop |
| `OPENSPEC_SKIP_SPECS` | `0` | Archive без apply-specs (для process/tooling change'ей) |
| `OPENSPEC_NO_VALIDATE` | `0` | Archive без валидации (fallback для broken change'ей) |

## Мониторинг

После merge можно проверить:
```bash
bash scripts/agent_flow/agent-flow-openspec-sync.sh status
```
Показывает JSON: `root`, `active_changes` (список имён), `specs_count`.

Крон `/health` может дёргать `status` каждую минуту — если `active_changes`
растёт бесконтрольно → алерт ("stale change folders").

## Failure modes

| Симптом | Причина | Что делать |
|---|---|---|
| Triage создал карточку, но `openspec/changes/<id>-*/` нет | `AGENT_FLOW_OPENSPEC_DISABLED=1` ИЛИ auto-detect не нашёл root ИЛИ sync crashed | Проверить лог triage (должно быть `openspec-sync:` line). Если WARN — запустить `agent-flow-openspec-sync.sh create-change ...` руками |
| Merge прошёл, change в `archive/` нет | `openspec archive` failed (валидация / broken proposal) | Проверить `openspec validate <change>` локально. Если нужна сила — `OPENSPEC_NO_VALIDATE=1` |
| `archive_openspec_change_for_merge` падает | sync script crashed / permissions | Проверить `agent-flow-merge-gate` лог на `openspec-sync: WARN`. Запустить руками с теми же аргументами |

## Связанные

- ADR-0038 — OpenSpec adoption (HYBRID + bulk-import)
- ADR-0039 — sync hooks (этот документ реализует)
- PR #1818 — bulk-import 45 ADR
- Issue #1819 — оригинальный тикет
- Issue #1789 — research OpenSpec
- `docs/research/openspec-pilot/` — пилотная зона
- `.agents/skills/openspec-workflow/` — skill для воркеров
- `openspec/changes/fix-memory-speaker-id/` — пример готового change folder

---
name: openspec-workflow
description: OpenSpec change-folder workflow в agent-flow — triage создаёт skeleton, воркер расширяет, merge-gate архивирует. Use when working in openspec/changes/ orksync your work with kanban task lifecycle.
category: devops
color: indigo
displayName: OpenSpec Workflow
risk: low
source: project
---

# OpenSpec Workflow — change folders в agent-flow пайплайне

## Назначение

OpenSpec (ADR-0038) — spec-driven фреймворк для трекинга изменений через
`openspec/changes/<id>-<slug>/`. Pilot scaffold в `docs/research/openspec-pilot/openspec/`.
Когда фича переедет на production, корень будет `openspec/` на уровне репо.

**Главное правило**: agent-flow автоматически создаёт change-folder при
триаже issue (triage hook) и автоматически архивирует при merge PR (merge-gate hook).
Воркеру **не нужно** создавать change-folder руками — он уже создан как skeleton.

## SOT (source of truth)

| Что | Где |
|---|---|
| Sync-скрипт | `scripts/agent_flow/agent-flow-openspec-sync.sh` |
| Triage hook | `scripts/agent_flow/agent-flow-triage.sh` (после kanban create) |
| Merge-gate hook | `scripts/agent_flow/agent-flow-merge-gate.sh` (archive_merged_card → archive_openspec_change_for_merge) |
| Pilot change folder | `docs/research/openspec-pilot/openspec/` |
| ADR | `docs/adr/0039-openspec-integration.md` |
| Docs | `docs/process/agent-flow-openspec-integration.md` |

## Жизненный цикл change-folder

```
1. ISSUE ОТКРЫТ
   └─ Юзер создаёт issue в krikz/rob_box_project
       └─ agent-flow-triage.tick (каждые 1м)
           └─ kanban create → t_<hex> карточка
           └─ openspec-sync.sh create-change → openspec/changes/<t_id>-<slug>/
               ├─ .openspec.yaml (schema, issue, task, goal)
               ├─ README.md (issue-ссылка)
               ├─ proposal.md (SKELETON, воркер дописывает)
               ├─ tasks.md (SKELETON, воркер дописывает)
               └─ specs/ (пустая, воркер создаёт capabilities)

2. ВОРКЕР РАБОТАЕТ
   └─ Воркер (Claude/devops/backend/...) берёт kanban-карточку
       └─ РАСШИРЯЕТ:
           ├─ proposal.md: WHY (2-3 предл), WHAT CHANGES (bullet), CAPABILITIES, IMPACT
           ├─ specs/<capability>/spec.md: ADDED Requirements + scenarios (REQUIREMENT-FIRST)
           ├─ design.md: Context, Goals/Non-Goals, Decisions, Risks
           └─ tasks.md: конкретные шаги с чекбоксами (≤2ч каждый)

3. PR ВЛИТ
   └─ agent-flow-merge-gate.tick (каждые 5м)
       └─ PR MERGED → archive kanban card
       └─ openspec-sync.sh archive-change
           ├─ openspec archive <change> --yes
           │   └─ Применяет изменения к openspec/specs/<capability>/spec.md
           │   └─ Переносит changes/<change>/ в changes/archive/<change>/
           └─ Комментарий в issue "OpenSpec: change archived"

4. CHANGE АРХИВИРОВАН
   └─ specs/ теперь содержит актуальные требования (delta applied)
   └─ archive/<change>/ хранит proposal/design/tasks для истории
```

## Когда и что делать ВОРКЕРУ

### На старте задачи (после того как взял kanban-карточку в работу)

1. **Проверь что change-folder существует**:
   ```bash
   ls openspec/changes/<task_id>-<slug>/   # или pilot location
   ```
   Если нет (race / sync failed) — НЕ создавай руками; вместо этого:
   ```bash
   bash scripts/agent_flow/agent-flow-openspec-sync.sh create-change \
       <issue> <task_id> <slug> "<title>"
   ```
   Скрипт идемпотентен.

2. **Расширь skeleton** — это ОБЯЗАТЕЛЬНАЯ часть работы воркера:
   - `proposal.md` → замени HTML-комменты на конкретный текст (Why, What Changes)
   - `specs/<capability>/spec.md` → создай если capability новая,
     или допиши ADDED Requirements если меняешь существующую
   - `design.md` → создай (Context, Goals/Non-Goals, Decisions, Risks)
   - `tasks.md` → конкретные чекбоксы (1.1, 1.2, ...) с ≤2ч granularity

3. **Проверь формат spec**:
   ```bash
   openspec validate <change-name>
   ```
   Если FAIL — почини до merge.

### Перед коммитом

- [ ] `proposal.md` имеет конкретный текст Why/What Changes/Impact
- [ ] `specs/` содержит минимум одну capability с ADDED Requirements + Scenarios
- [ ] `design.md` заполнен (если change не-trivial)
- [ ] `tasks.md` отражает РЕАЛЬНЫЕ шаги (не boilerplate)
- [ ] `openspec validate <change>` PASS

### После merge

Ничего не делай — merge-gate автоматически:
- Применит changes к `openspec/specs/`
- Перенесёт в `changes/archive/`
- Напишет комментарий в issue

Если archive провалился — это WARNING в merge-gate логе; sync не блокирует
kanban-archive (см. ADR-0039 §"non-blocking sync").

## Сценарии и решения

### "Change folder нет, а воркер уже взял карточку"

Причина: race condition (sync упал) или OPENSPEC_ROOT был пустой при triage.

Решение:
```bash
bash scripts/agent_flow/agent-flow-openspec-sync.sh create-change \
    <issue_number> <task_id> <slug> "<title>"
```

### "Change folder не архивировался после merge"

Причина: openspec CLI вернул non-zero (валидация failed / битый spec).

Решение:
1. Проверь `openspec validate <change>` локально
2. Если broken — почини proposal.md / spec.md
3. Запусти sync вручную:
   ```bash
   bash scripts/agent_flow/agent-flow-openspec-sync.sh archive-change \
       <issue_number> <task_id> <slug> <pr_number>
   ```

### "Я хочу отключить OpenSpec sync для одного запуска"

```bash
AGENT_FLOW_OPENSPEC_DISABLED=1 bash scripts/agent_flow/agent-flow-triage.sh
```

Или для одного cron-job — выставить в env сервиса.

### "OpenSpec root не находится автоматически"

```bash
export OPENSPEC_ROOT=/path/to/openspec
export REPO_DIR=/path/to/repo
bash scripts/agent_flow/agent-flow-openspec-sync.sh status
```

## Формат spec.md (требования OpenSpec)

```markdown
# Spec: <capability-name>

## Purpose

<1-2 предл: что это за capability и зачем>

## ADDED Requirements

### Requirement: <sh-all-caps-name>

<Что система MUST делать>

#### Scenario: <scenario-name>
- **WHEN** <condition>
- **THEN** <observable outcome>
```

Scenarios — это executable contracts (WHEN/THEN), не примеры кода.
Минимум 1 requirement + 2 scenarios на новую capability.

## Pitfalls

- ❌ Не создавай change-folder руками при старте — это делает triage.
- ❌ Не пиши в `openspec/archive/` руками — туда переносит `openspec archive`.
- ❌ Не удаляй `openspec/specs/` — это SOT, archive их обновляет.
- ❌ Не делай `--skip-specs` для change с новой capability — требования потеряются.
- ✅ Идемпотентность: повторный sync → noop (script сам skip'ает).
- ✅ Гибрид: skeleton от triage, flesh от воркера — это by design.

## Связанные скиллы

- `agent-flow` — общий процесс (PR/issue automation)
- `e2e-process` — что происходит с change после merge (нужен e2e)
- ADR-0019 (cron schedule), ADR-0038 (OpenSpec adoption), ADR-0039 (sync hooks)

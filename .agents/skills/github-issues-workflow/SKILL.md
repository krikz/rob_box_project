---
name: github-issues-workflow
description: >
  Полный цикл работы с задачами через GitHub Issues. Используй при начале работы
  над задачей, создании веток, оформлении PR и закрытии Issue.
  GitHub Issues = единственный источник правды; tasks.json не существует.
---

# GitHub Issues Workflow — Rob Box

## Репозиторий

`krikz/rob_box_project` (флаг `--repo krikz/rob_box_project` во всех командах).

## Концепция

GitHub Issues = единственный источник правды для задач, багов и tech-debt.

tasks.json не существует — не создавать, не читать.

Каждая задача = Issue с labels `type:*`, `priority:*`, `milestone:M1/M2/M3`, `source:gsd`, `ai-generated`.

## Жизненный цикл Issue

| Статус | Label | Описание | Следующий шаг |
|--------|-------|----------|---------------|
| Open | — | Задача открыта | `gh issue develop {N} --checkout` |
| In Progress | `status:in-progress` | Работа идёт | Commit + Push → PR |
| Blocked | `status:blocked` | Заблокирована | gh issue comment с причиной |
| Done | (closed) | Завершена | `gh issue close {N}` |

## Начало работы

1. Просмотр бэклога: `gh issue list --label "source:gsd" --repo krikz/rob_box_project`
2. Прочитать задачу: `gh issue view {N} --repo krikz/rob_box_project`
3. Создать ветку: `gh issue develop {N} --checkout --repo krikz/rob_box_project` (создаёт ветку `{N}-{slug-from-title}`)
4. Пометить как in-progress: `gh issue edit {N} --add-label "status:in-progress" --repo krikz/rob_box_project`

## Формат тела Issue (при создании)

```
## Description
{description}

## Acceptance Criteria
{acceptance_criteria as checklist}

## File References
{file/line if present}

## Source
Migrated from: {source} (tasks.json / TECH_DEBT.md)
Original ID: {TASK-ID or TD-ID}

---
> ⚠️ _This issue was created by AI (GSD workflow). Label: `ai-generated`_
```

## Завершение работы

1. Убедиться что тесты проходят (colcon build + pytest)
2. Создать PR: `gh pr create --title "feat: [description]" --body "Closes #{N}" --repo krikz/rob_box_project`
3. После merge: `gh issue close {N} --repo krikz/rob_box_project`
4. Удалить ветку: `git branch -d {N}-{slug}`

## Быстрые команды (cheatsheet)

| Действие | Команда |
|----------|---------|
| Список бэклога | `gh issue list --label "source:gsd" --repo krikz/rob_box_project` |
| Список по приоритету | `gh issue list --label "priority:high" --repo krikz/rob_box_project` |
| Начать работу | `gh issue develop {N} --checkout --repo krikz/rob_box_project` |
| Просмотр задачи | `gh issue view {N} --repo krikz/rob_box_project` |
| Добавить label | `gh issue edit {N} --add-label "status:in-progress" --repo krikz/rob_box_project` |
| Создать задачу | `gh issue create --title "[ID] title" --label "type:functional,priority:high,ai-generated,source:gsd" --repo krikz/rob_box_project` |
| Закрыть задачу | `gh issue close {N} --repo krikz/rob_box_project` |
| Создать PR | `gh pr create --title "feat: title" --body "Closes #{N}" --repo krikz/rob_box_project` |

## Label taxonomy

| Категория | Labels | Пример |
|-----------|--------|--------|
| Type | `type:tech-debt`, `type:stub`, `type:security`, `type:performance`, `type:functional`, `type:infrastructure`, `type:testing`, `bug`, `documentation` | `type:tech-debt` |
| Priority | `critical`, `priority:high`, `priority:medium`, `priority:low` | `priority:high` |
| Status | `status:in-progress`, `status:blocked` | `status:in-progress` |
| Milestone | `milestone:M1`, `milestone:M2`, `milestone:M3` | `milestone:M2` |
| Origin | `ai-generated`, `source:gsd` | `ai-generated` |

## Связь с context-engineering skill

Этот скилл дополняет `context-engineering/SKILL.md`. Research phase начинается с `gh issue view {N}`, не с tasks.json.

## Добавление комментария к Issue

```bash
# Прокомментировать прогресс
gh issue comment {N} --body "Started work on branch $(git branch --show-current)" --repo krikz/rob_box_project

# Прокомментировать блокировку
gh issue comment {N} --body "Blocked: {reason}" --repo krikz/rob_box_project
```

## Фильтрация и поиск задач

```bash
# По milestone
gh issue list --milestone "M1" --repo krikz/rob_box_project

# По нескольким labels (AND)
gh issue list --label "priority:high,source:gsd" --repo krikz/rob_box_project

# Закрытые задачи
gh issue list --state closed --label "source:gsd" --repo krikz/rob_box_project

# Поиск по тексту
gh issue list --search "voice" --repo krikz/rob_box_project
```

## Правила для агентов

```
✅ ВСЕГДА читать issue через gh issue view {N} перед началом работы
✅ ВСЕГДА создавать ветку через gh issue develop {N} --checkout
✅ ВСЕГДА закрывать issue после merge PR
❌ НИКОГДА не создавать tasks.json
❌ НИКОГДА не читать tasks.json
❌ НИКОГДА не создавать issue без label source:gsd
```

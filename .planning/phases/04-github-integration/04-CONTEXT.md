---
phase: 04-github-integration
type: context
created: 2026-05-15
---

# Phase 4 Context — GitHub Issues Integration

## Locked Decisions

### GH-0: Правильный порядок выполнения фазы
**Decision:** Сначала чиним ИНСТРУМЕНТ, потом используем его для миграции данных.

```
04-01: Обновить скиллы + инструкции  ← учим GSD работать через GitHub
04-02: Создать labels + milestones   ← инфраструктура GitHub
04-03: Мигрировать данные + cleanup  ← используем уже настроенный инструмент
```

**Обоснование:** Нельзя мигрировать данные пока система не знает как правильно работать с GitHub.  
Агент должен с первого же действия использовать `gh issue`, а не `tasks.json`.  
Скиллы и copilot-instructions — это "голова" системы, меняем их первыми.

### GH-1: Issue body format
**Decision:** Полный структурированный Markdown body — всё из tasks.json как структурированный body.  
**Template:**
```markdown
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

### GH-2: Migration scope
**Decision:** Мигрируем два источника:
1. `tasks.json` — все 20 задач → Issues
2. `TECH_DEBT.md` — все пункты кроме `accept` disposition (≈24 пункта), с дедупликацией по уже существующим TASK-ID

**Дедупликация:** Если TECH_DEBT пункт уже покрыт задачей из tasks.json (BG-5=TASK-049, BG-6=TASK-050 и др.) — НЕ создавать отдельный Issue, указать cross-reference в теле существующего.

**После миграции:** `tasks.json` удаляется из репо.

### GH-3: Разделение bot vs GSD Issues
**Decision:** Labels-only — фильтровать по `ai-generated` и `source:gsd`.  
**Не создаём** отдельный GitHub Projects board.  
**Команда просмотра:** `gh issue list --label "source:gsd"`

### GH-4: GitHub Milestones
**Decision:** Создать GitHub Milestones + labels (дублирование для удобства в UI):
- `Milestone 1: Code Quality` — closed (завершён)
- `Milestone 2: Navigation & Localization` — open
- `Milestone 3: Refactoring & Voice` — open

Labels `milestone:M1`, `milestone:M2`, `milestone:M3` создаются дополнительно.

### GH-5: Label taxonomy
**Создать следующие labels** (пропустить если уже существует):

| Label | Color | Категория |
|-------|-------|-----------|
| `ai-generated` | #7B61FF | AI origin |
| `source:gsd` | #5319E7 | AI origin |
| `type:bug` | существует `bug` | → использовать существующий |
| `type:tech-debt` | #FFA500 | Type |
| `type:stub` | #FF6B6B | Type |
| `type:security` | #D93F0B | Type |
| `type:performance` | #FBCA04 | Type |
| `type:functional` | #0075CA | Type |
| `type:infrastructure` | #E4E669 | Type |
| `type:testing` | #C2E0C6 | Type |
| `type:documentation` | существует `documentation` | → использовать существующий |
| `priority:critical` | существует `critical` | → использовать существующий |
| `priority:high` | #B60205 | Priority |
| `priority:medium` | #E4A000 | Priority |
| `priority:low` | #CFD3D7 | Priority |
| `milestone:M1` | #BFD4F2 | Milestone |
| `milestone:M2` | #D4C5F9 | Milestone |
| `milestone:M3` | #C5DEF5 | Milestone |
| `status:in-progress` | #0052CC | Status |
| `status:blocked` | #E11D48 | Status |

### GH-6: Автоматическое создание веток
**Decision:** ИИ создаёт ветку `feature/{N}-{slug}` при начале работы над задачей.  
**Команда:** `gh issue develop {N} --checkout`  
(или вручную: `git checkout -b feature/{N}-{slug}`)

### GH-7: STUB комментарии в коде
**После создания Issues для TASK-051 и TASK-052** — обновить `# STUB:` комментарии в коде заменив `TASK-051` / `TASK-052` на `#{issue_number}`.

### GH-8: copilot-instructions.md обновление
**Убрать:** все упоминания `tasks.json`  
**Добавить секцию** "Трекер задач" с командами `gh issue list/create/view/develop`  
**Scope:** только секция трекера, не переписывать весь файл

### GH-9: github-issues-workflow skill
**Создать:** `.agents/skills/github-issues-workflow/SKILL.md`  
**Содержимое скилла:** полный цикл — старт задачи → ветка → реализация → PR → close  

## Out of Scope (не делаем в Phase 4)
- GitHub Actions для автосоздания Issues (вручную через gh CLI)
- Закрытие bot-issues (оставляем как есть)
- CONCERNS.md → Issues (покрыто через TECH_DEBT.md chain)
- Автосинхронизация Issues → локальные файлы

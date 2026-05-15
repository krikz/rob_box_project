# Phase 4: GitHub Issues Integration — Context

**Gathered:** 2026-05-15  
**Status:** Ready for planning

<domain>
## Phase Boundary

Фаза 4 переводит систему трекинга задач с локального `tasks.json` на GitHub Issues и **обновляет AI-систему (скиллы, инструкции агента)**, чтобы GSD работал через GitHub Issues как единственный источник правды.

**Правильный порядок выполнения:**
```
04-01: Обновить скиллы + инструкции  ← учим GSD работать через GitHub (ПЕРВЫМ)
04-02: Создать labels + milestones   ← инфраструктура GitHub
04-03: Мигрировать данные + cleanup  ← используем уже настроенный инструмент
```

**Не входит в Phase 4:**
- GitHub Actions для автосоздания Issues
- GitHub Projects board (только labels)
- Исправление самих задач/багов (только создание Issues)
- Рефакторинг dialogue_node (Milestone 3)

</domain>

<decisions>
## Implementation Decisions

### D-01: Scope обновления скиллов
- **Full scan** всех `.agents/skills/` через grep на `tasks.json`
- **Deep update** (переписать narrative, не хирургические замены) всех найденных файлов
- Подтверждены: `.agents/skills/context-engineering/SKILL.md` (3 места) + `.github/copilot-instructions.md` (1 место)
- Другие скиллы (writing-plans, brainstorming, etc.) не зависят от трекера → только если найдёт grep

### D-02: Новый скилл github-issues-workflow
- **Тип:** Lifecycle skill (~150 строк)
- **Секции:** Когда использовать → Старт задачи → Ветка → Работа → PR → Close
- **Включить:** Законченный пример полного цикла (от `gh issue view N` до merged PR)
- **Включить:** Инструкции для создания новых Issues в процессе работы (нашёл баг → `gh issue create`)
- **Файл:** `.agents/skills/github-issues-workflow/SKILL.md`

### D-03: Автоматизация миграции
- **Python-скрипт** с idempotency через `gh issue list --search "{TASK-ID} in:title"` перед созданием
- **Файл:** `scripts/migrate_to_github.py` — удалить после выполнения (не засорять `scripts/`)
- **Scope:** Все 20 задач из tasks.json (open + done), ~24 уникальных пункта из TECH_DEBT.md
- done-задачи из tasks.json → закрытые Issues в GitHub (полная история)

### D-04: TECH_DEBT дедупликация
- **Критерий:** Только явное поле `task_ref` в TECH_DEBT.md определяет дубликат
- **Пропустить:** Пункты с `task_ref` (BG-5=TASK-049, BG-6=TASK-050) — уже войдут из tasks.json
- **Пропустить:** Все 6 пунктов с `disposition=accept` — остаются только в TECH_DEBT.md
- **Создать Issues:** ~24 уникальных пункта (fix + defer:M2 + defer:M3 без task_ref)
- Fuzzy matching по смыслу **не делаем** — слишком сложно, выше риск потери данных

### D-05: Label taxonomy
- **Не пересоздавать:** `bug`, `documentation`, `critical`, `deployment`, `python`, `docker`
- **Создать новые:** `ai-generated`, `source:gsd`, `type:tech-debt`, `type:stub`, `type:security`, `type:performance`, `type:functional`, `type:infrastructure`, `type:testing`, `priority:high`, `priority:medium`, `priority:low`, `milestone:M1`, `milestone:M2`, `milestone:M3`, `status:in-progress`, `status:blocked`
- Каждый Issue от ИИ: обязательно `ai-generated` + `source:gsd` + type + priority + milestone

### D-06: GitHub Milestones
- `Milestone 1: Code Quality` → state=closed
- `Milestone 2: Navigation & Localization` → state=open
- `Milestone 3: Refactoring & Voice` → state=open
- Создавать через `gh api repos/krikz/rob_box_project/milestones`

### D-07: Issue body format
Body каждого Issue:
```
## Description
{описание}

## Acceptance Criteria
- [ ] {критерий 1}
- [ ] {критерий 2}

## References
- Original ID: {TASK-ID или TECH_DEBT-ID}
- Priority: {priority}

---
> ⚠️ This issue was created by AI (GSD workflow). Original source: {tasks.json / TECH_DEBT.md}
```

### D-08: Ветки для задач
- `gh issue develop {N} --checkout` для автосоздания ветки
- Шаблон: `feature/{N}-{slug}` для фичей, `fix/{N}-{slug}` для багов
- PR закрывает Issue через `closes #{N}` в body

### D-09: STUB комментарии в коде
- После создания Issues для TASK-051 и TASK-052 — заменить `TASK-051`/`TASK-052` → `#{N}` в коде
- Файлы: `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py` + `src/rob_box_voice/rob_box_voice/command_node.py` (3 места)

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Phase scope и requirements
- `.planning/ROADMAP.md` Phase 4 — цель, GH-01..05 requirements, success criteria
- `.planning/REQUIREMENTS.md` — GH-требования

### Данные для миграции
- `tasks.json` — 20 задач (структура: id, title, description, category, priority, acceptance_criteria)
- `.planning/TECH_DEBT.md` — 30 пунктов (структура: ID, title, severity, disposition, task_ref)

### Скиллы для обновления
- `.agents/skills/context-engineering/SKILL.md` — главный workflow skill, 3 упоминания tasks.json
- `.github/copilot-instructions.md` — инструкции агента, 1 упоминание tasks.json

### Код со STUB-ссылками
- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py` — STUB: ... TASK-051
- `src/rob_box_voice/rob_box_voice/command_node.py` — 3x STUB: ... TASK-052

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `gh` CLI — авторизован как GOODWORKRINKZ, репо `krikz/rob_box_project` (branch: develop)
- `.agents/skills/github-actions-runner/SKILL.md` — пример лаконичного skill (~40 строк)
- `.agents/skills/context-engineering/SKILL.md` — пример детального workflow skill (~178 строк)

### Established Patterns
- Коммит-конвенция: `feat(04-01): ...`, `fix(04-02): ...`, `chore(04-03): ...`
- Скиллы в формате: YAML frontmatter + narrative + code blocks
- Docker/ROS принципы не затрагиваются в этой фазе

### Integration Points
- `scripts/migrate_to_github.py` вызывает `gh issue create` через subprocess
- После миграции: `git rm tasks.json` + `git rm scripts/migrate_to_github.py` как часть финального коммита

</code_context>

<specifics>
## Specific Ideas

- Idempotency через `gh issue list --search "TASK-035 in:title"` — не через TASK_MAPPING.json
- Скрипт `scripts/migrate_to_github.py` — удалить (`git rm`) после выполнения
- github-issues-workflow skill: lifecycle + законченный пример от `gh issue view N` до merged PR
- context-engineering/SKILL.md: deep update narrative (не просто замена 3 строк)

</specifics>

<deferred>
## Deferred Ideas

- GitHub Projects board — явно отклонён, не нужен
- Fuzzy matching по title для дедупликации TECH_DEBT — слишком сложно, отклонён
- Создание Issues через GitHub Actions (автоматически при коммите) — другая фаза
- TASK_MAPPING.json как state-файл — отклонён, используем gh search для idempotency

</deferred>

---

*Phase: 4-github-integration*
*Context gathered: 2026-05-15*

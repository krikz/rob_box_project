---
phase: 04-github-integration
plan: "04 (объединённый — 3 подплана в одном файле)"
type: execute
wave: 1
depends_on: []
files_modified:
  - .github/copilot-instructions.md
  - .agents/skills/context-engineering/SKILL.md
  - .agents/skills/github-issues-workflow/SKILL.md
  - src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py
  - src/rob_box_voice/rob_box_voice/command_node.py
  - tasks.json
autonomous: true
requirements:
  - GH-01
  - GH-02
  - GH-03
  - GH-04
  - GH-05

# ╔══════════════════════════════════════════════════════════════════╗
# ║  ВАЖНО: Правильный порядок фаз                                  ║
# ║                                                                  ║
# ║  04-01 → Чиним ИНСТРУМЕНТ (скиллы, instructions)                ║
# ║  04-02 → Инфраструктура GitHub (labels, milestones)             ║
# ║  04-03 → Используем исправленный инструмент для миграции данных  ║
# ║                                                                  ║
# ║  Нельзя мигрировать данные пока система не знает как работать    ║
# ║  с GitHub. Сначала учим GSD, потом применяем.                   ║
# ╚══════════════════════════════════════════════════════════════════╝

must_haves:
  truths:
    - "`gh issue list --label source:gsd` возвращает ≥20 issues"
    - "`ls tasks.json` — файл не существует в репозитории"
    - "`copilot-instructions.md` содержит команды `gh issue list/create/view` и НЕ содержит `tasks.json`"
    - "`.agents/skills/github-issues-workflow/SKILL.md` создан и описывает полный цикл start→branch→PR→close"
    - "`.agents/skills/context-engineering/SKILL.md` ссылается на `gh issue` вместо `tasks.json`"
    - "Все `# STUB:` комментарии в коде содержат `#N` (номер GitHub Issue), а не `TASK-05X`"

<verification>
## Verification Commands

| ID | Команда | Критерий прохождения |
|----|---------|----------------------|
| GH-01 | `gh issue list --label "source:gsd" --limit 100 \| wc -l` | ≥ 20 |
| GH-02 | `test ! -f tasks.json && echo "PASS"` | Вывод: PASS |
| GH-03 | `grep -c "gh issue" .github/copilot-instructions.md` | ≥ 1 |
| GH-04 | `ls .agents/skills/github-issues-workflow/SKILL.md` | Файл существует |
| GH-05 | `grep -rn "TASK-051\|TASK-052" src/ --include="*.py" \| wc -l` | 0 (заменены на #N) |
| GH-06 | `gh label list \| grep -c "ai-generated\|source:gsd\|type:stub"` | ≥ 3 |
| GH-07 | `gh api repos/{owner}/{repo}/milestones \| python3 -c "import json,sys; data=json.load(sys.stdin); print(len(data))"` | ≥ 2 |
| GH-08 | `grep -c "tasks.json" .agents/skills/context-engineering/SKILL.md` | 0 (убрано) |
</verification>

---

## Подплан 04-01: Обновить GSD — скиллы и инструкции

**Цель:** GSD-система перестаёт знать о `tasks.json`. Все скиллы и copilot-instructions.md ссылаются  
на GitHub Issues. Только после этого можно мигрировать данные — агент будет использовать  
правильный инструмент с первого же действия.

**Зависимость:** нет (первый шаг).

### Задача 04-01-01: Создать скилл `github-issues-workflow`

**Файл:** `.agents/skills/github-issues-workflow/SKILL.md` (создать новый)

**Содержимое скилла — полный lifecycle работы с задачами через GitHub:**

```markdown
````skill
---
name: github-issues-workflow
description: >
  Рабочий процесс для работы с задачами через GitHub Issues в Rob Box Project.
  ВСЕГДА используй этот скилл при: старте любой задачи, создании новых багов/фичей,
  завершении работы, просмотре бэклога. Заменяет tasks.json полностью.
---

# GitHub Issues Workflow — Rob Box

## ⚠️ Главное правило
**tasks.json НЕ СУЩЕСТВУЕТ.** Единственный источник правды — GitHub Issues.  
Для любой задачи используй `gh issue` команды, не создавай/не читай tasks.json.

## Старт задачи

**1. Посмотреть бэклог:**
```bash
# Все задачи от GSD/AI
gh issue list --label "source:gsd" --limit 50

# По приоритету
gh issue list --label "priority:critical" --label "source:gsd"
gh issue list --label "milestone:M2" --label "source:gsd"

# Подробности конкретной задачи
gh issue view {N}
```

**2. Начать работу (автоматически создаёт ветку):**
```bash
gh issue develop {N} --checkout
# Создаёт ветку: {N}-{slug-from-title}
# Или вручную:
git checkout -b feature/{N}-{slug}   # для фичей
git checkout -b fix/{N}-{slug}       # для багов
```

## Работа над задачей

Коммиты должны ссылаться на Issue:
```bash
git commit -m "feat(module): описание (#N)"
git commit -m "fix(module): исправление (#N)"
```

Если заблокирован — добавить label через gh:
```bash
gh issue edit {N} --add-label "status:blocked"
gh issue comment {N} --body "Заблокировано: {причина}"
```

## Завершение и PR

```bash
# Создать PR закрывающий Issue
gh pr create \
  --title "feat: описание (closes #{N})" \
  --body "## Summary
{описание изменений}

## Changes
- {список изменений}

Closes #{N}" \
  --label "source:gsd"

# Если несколько Issues закрываются:
# "Closes #N, closes #M"
```

Issue закроется автоматически при merge PR, если в body есть `closes #{N}`.

## Создать новый Issue (от ИИ)

```bash
gh issue create \
  --title "{описание}" \
  --body "## Description
{подробное описание}

## Acceptance Criteria
- [ ] {критерий 1}
- [ ] {критерий 2}

---
> ⚠️ _This issue was created by AI (GSD workflow). Label: \`ai-generated\`_" \
  --label "ai-generated,source:gsd,{type_label},{priority_label},{milestone_label}"
```

### Label conventions при создании Issues от ИИ

| Поле | Label | Пример |
|------|-------|--------|
| Обязательно | `ai-generated` | всегда |
| Обязательно | `source:gsd` | всегда |
| Тип | `type:bug`, `type:stub`, `type:tech-debt`, `type:security`, `type:performance`, `type:functional`, `type:infrastructure`, `type:testing` | по контексту |
| Приоритет | `priority:critical`, `priority:high`, `priority:medium`, `priority:low` | обязательно |
| Milestone | `milestone:M1`, `milestone:M2`, `milestone:M3` | обязательно |

## Просмотр прогресса

```bash
# Открытые задачи текущего milestone
gh issue list --label "milestone:M2,source:gsd" --state open

# Закрытые задачи
gh issue list --label "source:gsd" --state closed --limit 20

# Все задачи (открытые + закрытые)
gh issue list --label "source:gsd" --state all --limit 100
```
````
```

**Проверка:** `ls .agents/skills/github-issues-workflow/SKILL.md` → файл существует

### Задача 04-01-02: Обновить `context-engineering` скилл

**Файл:** `.agents/skills/context-engineering/SKILL.md` (изменить 3 места)

**Изменение 1 — description (строка 6):**
```
ДО:  Используй ВСЕГДА при начале работы над любой задачей из tasks.json.
ПОСЛЕ: Используй ВСЕГДА при начале работы над любой задачей из GitHub Issues.
```

**Изменение 2 — диаграмма (строка 22):**
```
ДО:  tasks.json → Research → Design → Plan → Implement
ПОСЛЕ: gh issue → Research → Design → Plan → Implement
```

**Изменение 3 — таблица "Связанные ресурсы" (строка 172):**
```
ДО:  | `tasks.json` | Бэклог задач с acceptance_criteria и dependencies |
ПОСЛЕ: | GitHub Issues | Бэклог задач: `gh issue list --label "source:gsd"` |
```

**Проверка:** `grep -c "tasks.json" .agents/skills/context-engineering/SKILL.md` → 0

### Задача 04-01-03: Обновить `copilot-instructions.md`

**Файл:** `.github/copilot-instructions.md`

**Изменение 1 — строка 16 (таблица "Где искать"):**
```
ДО:  | **Бэклог задач** | `tasks.json` | Задачи, acceptance criteria, зависимости, agent_instructions |
ПОСЛЕ: | **Бэклог задач** | GitHub Issues | `gh issue list --label "source:gsd"` — задачи, acceptance criteria |
```

**Изменение 2 — добавить секцию "Трекер задач" в раздел "⚡ Быстрые команды":**

Добавить после существующих команд SSH-доступа:
```markdown
### Трекер задач (GitHub Issues)
```bash
# Бэклог текущего milestone
gh issue list --label "source:gsd" --limit 50

# Начать задачу (создаёт ветку автоматически)
gh issue develop {N} --checkout

# Создать новый Issue (от ИИ)
gh issue create --label "ai-generated,source:gsd,type:bug,priority:high,milestone:M2"

# Закрыть задачу через коммит
git commit -m "fix: описание (closes #{N})"

# Детали задачи
gh issue view {N}
```

**Проверка:** `grep -c "gh issue" .github/copilot-instructions.md` → ≥ 1  
**Проверка:** `grep -c "tasks.json" .github/copilot-instructions.md` → 0

### Коммит 04-01:

```bash
git add .agents/skills/github-issues-workflow/ \
        .agents/skills/context-engineering/SKILL.md \
        .github/copilot-instructions.md

git commit -m "feat(04-01): GSD теперь работает через GitHub Issues — новый скилл + обновлены context-engineering и copilot-instructions"
```

---

## Подплан 04-02: Инфраструктура GitHub — Labels + Milestones

**Цель:** Создать label taxonomy и GitHub Milestones — инфраструктура нужна до миграции данных.  
**Зависимость:** 04-01 завершён (агент уже умеет работать через GitHub).

### Задача 04-02-01: Создать label taxonomy

**Файлы:** Не затрагивает локальные файлы — только GitHub API через gh CLI.

**Шаг 1: Проверить существующие labels**
```bash
gh label list --limit 50
```
Запомнить уже существующие (bug, documentation, critical, deployment, python, docker и др.) — не пересоздавать.

**Шаг 2: Создать новые labels**
```bash
# AI origin
gh label create "ai-generated" --color "7B61FF" --description "Issue created by AI agent (GSD workflow)"
gh label create "source:gsd" --color "5319E7" --description "Created by GSD workflow agent"

# Type labels (type:bug → используем существующий 'bug', type:documentation → 'documentation')
gh label create "type:tech-debt" --color "FFA500" --description "Technical debt to be addressed"
gh label create "type:stub" --color "FF6B6B" --description "Stub implementation — not yet functional"
gh label create "type:security" --color "D93F0B" --description "Security concern"
gh label create "type:performance" --color "FBCA04" --description "Performance issue"
gh label create "type:functional" --color "0075CA" --description "Functional feature or improvement"
gh label create "type:infrastructure" --color "E4E669" --description "Infrastructure, Docker, CI/CD"
gh label create "type:testing" --color "C2E0C6" --description "Tests and coverage"

# Priority labels (critical уже существует — используем как priority:critical)
gh label create "priority:high" --color "B60205" --description "High priority"
gh label create "priority:medium" --color "E4A000" --description "Medium priority"
gh label create "priority:low" --color "CFD3D7" --description "Low priority"

# Milestone labels
gh label create "milestone:M1" --color "BFD4F2" --description "Milestone 1: Code Quality"
gh label create "milestone:M2" --color "D4C5F9" --description "Milestone 2: Navigation"
gh label create "milestone:M3" --color "C5DEF5" --description "Milestone 3: Refactoring"

# Status labels
gh label create "status:in-progress" --color "0052CC" --description "Currently being worked on"
gh label create "status:blocked" --color "E11D48" --description "Blocked by dependency or issue"
```

Если label уже существует — команда вернёт ошибку, игнорировать и продолжать.

**Проверка:** `gh label list | grep -c "ai-generated\|source:gsd\|type:stub"` → ≥ 3

### Задача 04-02-02: Создать GitHub Milestones

```bash
# Milestone 1 (закрытый — уже завершён)
gh api repos/krikz/rob_box_project/milestones \
  --method POST \
  --field title="Milestone 1: Code Quality" \
  --field description="Аудит документации, ревью структуры, code quality review, GitHub Issues integration" \
  --field state="closed"

# Milestone 2
gh api repos/krikz/rob_box_project/milestones \
  --method POST \
  --field title="Milestone 2: Navigation & Localization" \
  --field description="Nav2, SLAM, OAK-D vision, AprilTag, голосовые навигационные команды"

# Milestone 3
gh api repos/krikz/rob_box_project/milestones \
  --method POST \
  --field title="Milestone 3: Refactoring & Voice" \
  --field description="dialogue_node рефакторинг, code quality fixes, voice integration"
```

**Проверка:**
```bash
gh api repos/krikz/rob_box_project/milestones | python3 -c "
import json, sys
data = json.load(sys.stdin)
for m in data:
    print(m['number'], m['title'], m['state'])
"
# Ожидается: 3 milestones
```

**Коммит:** `feat(04-02): GitHub label taxonomy (17 новых labels) + 3 milestones`  
*(в репо коммитить нечего — labels/milestones живут в GitHub; коммит обновления STATE.md)*

---

## Подплан 04-03: Миграция данных + удаление tasks.json

**Цель:** Используя уже настроенную инфраструктуру (04-01 + 04-02) — перенести все задачи в GitHub Issues,  
обновить STUB-ссылки в коде, удалить tasks.json.  
**Зависимость:** 04-01 (скиллы обновлены) + 04-02 (labels и milestones созданы).

### Задача 04-03-01: Мигрировать tasks.json → GitHub Issues

**Файлы:** `tasks.json` (читать), GitHub API

**Маппинг категорий → labels:**

| tasks.json category | GitHub type-label | Milestone label | GitHub Milestone # |
|--------------------|-------------------|-----------------|--------------------|
| bug, bugfix | bug | milestone:M1 | M1 number |
| functional | type:functional | milestone:M2 | M2 number |
| infrastructure | type:infrastructure | milestone:M2 | M2 number |
| testing | type:testing | milestone:M2 | M2 number |
| documentation | documentation | milestone:M1 | M1 number |
| stub | type:stub | milestone:M2 | M2 number |
| security | type:security | milestone:M2 | M2 number |
| performance | type:performance | milestone:M3 | M3 number |

**Шаг 1: Получить номера GitHub Milestones**
```bash
gh api repos/krikz/rob_box_project/milestones | python3 -c "
import json, sys
for m in json.load(sys.stdin):
    print(m['number'], m['title'])
"
```

**Шаг 2: Создать Issues для каждой задачи из tasks.json**

```bash
gh issue create \
  --title "{id}: {title}" \
  --body "## Description
{description}

## Acceptance Criteria
{acceptance_criteria as markdown checklist - each item as '- [ ] item'}

## References
- Original ID: {id}
- Priority: {priority}
- Category: {category}

---
> ⚠️ _This issue was created by AI (GSD workflow). Original source: tasks.json_" \
  --label "ai-generated,source:gsd,{type_label},{priority_label},{milestone_label}" \
  --milestone "{github_milestone_number}"
```

**Шаг 3: Сохранить маппинг TASK-ID → Issue #**

Создать `.planning/phases/04-github-integration/TASK_MAPPING.md` по ходу миграции:
```markdown
# TASK-ID → GitHub Issue Mapping

| TASK-ID | GitHub Issue # | Title |
|---------|---------------|-------|
| TASK-035 | #N | ... |
| TASK-051 | #N | ...  ← нужен для STUB обновления |
| TASK-052 | #N | ...  ← нужен для STUB обновления |
```

**Проверка:** `gh issue list --label "source:gsd" --limit 50 | wc -l` → ≥ 20

### Задача 04-03-02: Мигрировать TECH_DEBT.md уникальные пункты → Issues

**Файлы:** `.planning/TECH_DEBT.md` (читать)

**Правила:**
- ❌ Пропустить: `disposition=accept` (6 шт)
- ❌ Пропустить: пункты уже покрытые tasks.json (BG-5=TASK-049, BG-6=TASK-050)
- ✅ Мигрировать: disposition=fix + defer:M2 + defer:M3 без task_ref

**Type-label по секции TECH_DEBT.md:**
- TD-* → type:tech-debt
- BG-* → bug
- SEC-* → type:security
- PF-* → type:performance
- FA-* → type:testing
- SL-* → type:infrastructure

**Шаг: Добавить cross-reference к BG-5/BG-6 Issues (уже созданным из tasks.json):**
```bash
gh issue comment {issue_for_TASK-049} --body "Cross-reference: TECH_DEBT.md BG-5"
gh issue comment {issue_for_TASK-050} --body "Cross-reference: TECH_DEBT.md BG-6"
```

**Проверка:** `gh issue list --label "source:gsd" --limit 100 | wc -l` → ≥ 28

### Задача 04-03-03: Обновить STUB комментарии в коде

**Файлы:**
- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py`
- `src/rob_box_voice/rob_box_voice/command_node.py`

**Шаг 1: Получить номера Issues**
```bash
grep "TASK-051\|TASK-052" .planning/phases/04-github-integration/TASK_MAPPING.md
```

**Шаг 2: Заменить TASK-ID → Issue # в STUB комментариях**
```python
# STUB: ... TASK-051  →  # STUB: ... #{N_for_051}
# STUB: ... TASK-052  →  # STUB: ... #{N_for_052}   (3 места в command_node.py)
```

**Проверка:** `grep -rn "TASK-051\|TASK-052" src/ --include="*.py" | wc -l` → 0

### Задача 04-03-04: Удалить tasks.json

```bash
git rm tasks.json
```

**ВАЖНО:** TASK_MAPPING.md в `.planning/` сохраняет историческую связь TASK-ID ↔ Issue#.

**Проверка:** `test ! -f tasks.json && echo "PASS"` → PASS

### Коммит 04-03:

```bash
git add .planning/phases/04-github-integration/TASK_MAPPING.md \
        src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py \
        src/rob_box_voice/rob_box_voice/command_node.py

git commit -m "feat(04-03): migrate tasks.json (20) + TECH_DEBT.md → GitHub Issues; fix STUB refs → #N; delete tasks.json"
```

---

## Финальная верификация (все 8 проверок)

```bash
echo "=== GH-01: Issues с source:gsd ===" && \
  gh issue list --label "source:gsd" --limit 100 | wc -l && \
echo "=== GH-02: tasks.json удалён ===" && \
  test ! -f tasks.json && echo "PASS" && \
echo "=== GH-03: copilot-instructions содержит gh issue ===" && \
  grep -c "gh issue" .github/copilot-instructions.md && \
echo "=== GH-04: github-issues-workflow skill ===" && \
  ls .agents/skills/github-issues-workflow/SKILL.md && \
echo "=== GH-05: STUB без TASK-05X ===" && \
  grep -rn "TASK-051\|TASK-052" src/ --include="*.py" | wc -l && \
echo "=== GH-06: labels созданы ===" && \
  gh label list | grep -c "ai-generated\|source:gsd\|type:stub" && \
echo "=== GH-07: milestones созданы ===" && \
  gh api repos/krikz/rob_box_project/milestones | python3 -c "import json,sys; print(len(json.load(sys.stdin)))" && \
echo "=== GH-08: context-engineering без tasks.json ===" && \
  grep -c "tasks.json" .agents/skills/context-engineering/SKILL.md
```

**Ожидаемый вывод:**
- GH-01: ≥ 20
- GH-02: PASS
- GH-03: ≥ 1
- GH-04: файл найден
- GH-05: 0
- GH-06: ≥ 3
- GH-07: ≥ 2 (или 3)
- GH-08: 0

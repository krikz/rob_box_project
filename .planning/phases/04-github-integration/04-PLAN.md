---
phase: 04-github-integration
plan: "04 (объединённый — 3 подплана в одном файле)"
type: execute
wave: 1
depends_on: []
files_modified:
  - .github/copilot-instructions.md
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

must_haves:
  truths:
    - "`gh issue list --label source:gsd` возвращает ≥20 issues"
    - "`ls tasks.json` — файл не существует в репозитории"
    - "`copilot-instructions.md` содержит команды `gh issue list/create/view` и НЕ содержит `tasks.json`"
    - "`.agents/skills/github-issues-workflow/SKILL.md` создан и описывает полный цикл start→branch→PR→close"
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
</verification>

---

## Подплан 04-01: Label Taxonomy + GitHub Milestones

**Цель:** Создать структуру лейблов и milestones в GitHub — основа для всех последующих Issues.  
**Примечание:** Проверять существующие лейблы через `gh label list` перед созданием (bug, documentation, critical уже существуют).

### Задача 04-01-01: Создать label taxonomy

**Файлы:** Не затрагивает локальные файлы — только GitHub API через gh CLI

**Шаг 1: Проверить существующие labels**
```bash
gh label list --limit 50
```
Запомнить уже существующие (bug, documentation, critical, deployment, python, docker и др.) — не пересоздавать.

**Шаг 2: Создать новые labels**

Создать следующие (пропустить если уже существует с нужным цветом):
```bash
# AI origin
gh label create "ai-generated" --color "7B61FF" --description "Issue created by AI agent (GSD workflow)"
gh label create "source:gsd" --color "5319E7" --description "Created by GSD workflow agent"

# Type labels (type:bug → используем существующий 'bug')
gh label create "type:tech-debt" --color "FFA500" --description "Technical debt to be addressed"
gh label create "type:stub" --color "FF6B6B" --description "Stub implementation — not yet functional"
gh label create "type:security" --color "D93F0B" --description "Security concern"
gh label create "type:performance" --color "FBCA04" --description "Performance issue"
gh label create "type:functional" --color "0075CA" --description "Functional feature or improvement"
gh label create "type:infrastructure" --color "E4E669" --description "Infrastructure, Docker, CI/CD"
gh label create "type:testing" --color "C2E0C6" --description "Tests and coverage"
# type:documentation → используем существующий 'documentation'

# Priority labels (critical уже существует)
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

**Шаг 3: Верификация**
```bash
gh label list | grep -c "ai-generated\|source:gsd\|type:stub"
# Ожидается: ≥ 3
```

### Задача 04-01-02: Создать GitHub Milestones

**Шаг 1: Создать Milestone 1 (закрытый)**
```bash
gh api repos/{owner}/{repo}/milestones \
  --method POST \
  --field title="Milestone 1: Code Quality" \
  --field description="Аудит документации, ревью структуры, code quality review, GitHub Issues integration" \
  --field state="closed"
```

**Шаг 2: Создать Milestone 2**
```bash
gh api repos/{owner}/{repo}/milestones \
  --method POST \
  --field title="Milestone 2: Navigation & Localization" \
  --field description="Nav2, SLAM, OAK-D vision, AprilTag, голосовые навигационные команды"
```

**Шаг 3: Создать Milestone 3**
```bash
gh api repos/{owner}/{repo}/milestones \
  --method POST \
  --field title="Milestone 3: Refactoring & Voice" \
  --field description="dialogue_node рефакторинг, code quality fixes, voice integration"
```

**Шаг 4: Верификация**
```bash
gh api repos/{owner}/{repo}/milestones | python3 -c "
import json, sys
data = json.load(sys.stdin)
for m in data:
    print(m['number'], m['title'], m['state'])
"
# Ожидается: 3 milestones, M1=closed, M2/M3=open
```

**Коммит:** `feat(04-01): GitHub labels taxonomy (20 labels) + 3 milestones`  
*(коммит только STATE.md и CONTEXT.md — labels/milestones живут в GitHub, не в репо)*

---

## Подплан 04-02: Миграция tasks.json + TECH_DEBT.md → GitHub Issues

**Цель:** Перенести все задачи, баги, tech debt в GitHub Issues. Дедупликация — один Issue на уникальный дефект.  
**Зависимость:** 04-01 должен быть завершён (labels и milestones созданы).

### Задача 04-02-01: Мигрировать tasks.json → Issues

**Файлы:** `tasks.json` (читать), GitHub API (создавать через gh CLI)

**Шаг 1: Проанализировать маппинг категорий**

| tasks.json category | GitHub label | GitHub milestone |
|--------------------|--------------|-----------------|
| bug, bugfix | bug | по полю milestone (M1/M2) |
| functional | type:functional | по полю milestone |
| infrastructure | type:infrastructure | по полю milestone |
| testing | type:testing | по полю milestone |
| documentation | documentation | по полю milestone |
| stub | type:stub | milestone:M2 |

**Шаг 2: Создать Issues с полным body**

Для каждой задачи из tasks.json создать Issue:
```bash
gh issue create \
  --title "{id}: {title}" \
  --body "## Description
{description}

## Acceptance Criteria
{acceptance_criteria as markdown checklist}

## References
- File: {file if present}
- Original ID: {id}
- Priority: {priority}

---
> ⚠️ _This issue was created by AI (GSD workflow). Original source: tasks.json_" \
  --label "ai-generated,source:gsd,{type_label},{priority_label},{milestone_label}" \
  --milestone "{github_milestone_number}"
```

**Шаг 3: Сохранить маппинг TASK-ID → Issue number**

Сохранить в `.planning/phases/04-github-integration/TASK_MAPPING.md`:
```markdown
| TASK-ID | GitHub Issue # | Title |
|---------|---------------|-------|
| TASK-035 | #N | ... |
| TASK-051 | #N | ... |  ← нужен для обновления STUB комментариев
| TASK-052 | #N | ... |  ← нужен для обновления STUB комментариев
```

**Проверка:** `gh issue list --label "source:gsd" --limit 50 | wc -l` → ≥ 20

### Задача 04-02-02: Мигрировать TECH_DEBT.md → Issues (уникальные пункты)

**Файлы:** `.planning/TECH_DEBT.md` (читать), GitHub API

**Шаг 1: Определить уникальные пункты для миграции**

- Пропустить: все пункты с `disposition=accept` (6 шт)
- Пропустить: пункты уже покрытые tasks.json (BG-5=TASK-049, BG-6=TASK-050 и любые с полем task_ref)
- Мигрировать: оставшиеся пункты с disposition=fix/defer:M2/defer:M3 без task_ref

**Шаг 2: Создать Issues для уникальных пунктов**

```bash
gh issue create \
  --title "{ID}: {title}" \
  --body "## Description
{description}

## Severity
{severity}

## Disposition
{disposition}

## Source File
{file if present}

---
> ⚠️ _This issue was created by AI (GSD workflow). Original source: TECH_DEBT.md {ID}_" \
  --label "ai-generated,source:gsd,{type_label},{priority_label},{milestone_label}"
```

**Тип лейбла по TECH_DEBT секции:**
- TD-* → type:tech-debt
- BG-* → bug
- SEC-* → type:security
- PF-* → type:performance
- FA-* → type:testing
- SL-* → type:infrastructure

**Шаг 3: Добавить cross-reference для дублей**

Для пунктов BG-5/BG-6 (уже в tasks.json) — добавить комментарий к Issue:
```bash
gh issue comment {issue_number} --body "Cross-reference: TECH_DEBT.md BG-5 / TASK-049"
```

**Проверка:** `gh issue list --label "source:gsd" --limit 100 | wc -l` → ≥ 28

**Коммит:** `feat(04-02): migrate tasks.json (20) + TECH_DEBT.md unique items to GitHub Issues`  
*(коммит: TASK_MAPPING.md + STATE.md)*

---

## Подплан 04-03: Обновить workflow + удалить tasks.json

**Цель:** Агент знает о GitHub Issues как источнике правды; tasks.json удалён; STUB комментарии обновлены.  
**Зависимость:** 04-02 должен быть завершён (нужны номера Issues для TASK-051/052).

### Задача 04-03-01: Обновить copilot-instructions.md

**Файлы:** `.github/copilot-instructions.md`

**Шаг 1: Прочитать текущее состояние**
```bash
grep -n "tasks.json\|задачи\|трекер\|Tasks" .github/copilot-instructions.md | head -20
```

**Шаг 2: Убрать упоминания tasks.json**

Найти и заменить все секции/строки с упоминанием `tasks.json`.

**Шаг 3: Добавить секцию "Трекер задач"**

Добавить в раздел быстрых команд:
```markdown
### Трекер задач (GitHub Issues)
```bash
# Просмотр задач от ИИ
gh issue list --label "source:gsd" --limit 50

# Просмотр задач по приоритету
gh issue list --label "priority:critical,source:gsd"

# Начать работу над задачей (создаёт ветку автоматически)
gh issue develop {N} --checkout
# Или вручную:
git checkout -b feature/{N}-{slug}

# Создать новую задачу (от ИИ)
gh issue create --label "ai-generated,source:gsd,type:bug,priority:high,milestone:M2"

# Закрыть через коммит (в сообщении коммита)
git commit -m "fix: описание (closes #{N})"
```

**Проверка:** `grep -c "gh issue" .github/copilot-instructions.md` → ≥ 1  
**Проверка:** `grep -c "tasks.json" .github/copilot-instructions.md` → 0

### Задача 04-03-02: Создать github-issues-workflow skill

**Файлы:** `.agents/skills/github-issues-workflow/SKILL.md` (создать)

**Содержимое скилла — полный lifecycle:**
- Старт задачи: `gh issue view N` → прочитать acceptance criteria → `gh issue develop N --checkout`
- Ветка: `feature/{N}-{slug}` или `fix/{N}-{slug}` для багов
- Работа: коммиты ссылаются на issue через `#N` в message
- PR: `gh pr create --title "..." --body "closes #{N}"` с labels
- Close: автоматически при merge если `closes #N` в PR body

Скилл должен включать:
1. Когда использовать этот скилл
2. Стандартные команды (копируемые блоки)
3. Label conventions для новых Issues созданных ИИ
4. Правило: НИКОГДА не использовать tasks.json — только `gh issue`

### Задача 04-03-03: Обновить STUB комментарии в коде

**Файлы:**
- `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py`
- `src/rob_box_voice/rob_box_voice/command_node.py`

**Шаг 1: Получить номера Issues для TASK-051 и TASK-052**
```bash
# Из TASK_MAPPING.md созданного в 04-02-01
grep "TASK-051\|TASK-052" .planning/phases/04-github-integration/TASK_MAPPING.md
```

**Шаг 2: Заменить TASK-ID → Issue number в STUB комментариях**

В `system.py`:
```python
# STUB: ... TASK-051 → # STUB: ... #{N}
```

В `command_node.py` (3 места):
```python
# STUB: ... TASK-052 → # STUB: ... #{N}
```

**Шаг 3: Верификация**
```bash
grep -rn "TASK-051\|TASK-052" src/ --include="*.py"
# Ожидается: 0 результатов
```

### Задача 04-03-04: Удалить tasks.json

**Файлы:** `tasks.json` (удалить из репо)

```bash
git rm tasks.json
```

**ВАЖНО:** Убедиться что TASK_MAPPING.md сохранён в `.planning/` — он сохраняет историю маппинга.

**Проверка:** `test ! -f tasks.json && echo "PASS"` → PASS

### Задача 04-03-05: Финальный коммит фазы

```bash
git add .github/copilot-instructions.md \
        .agents/skills/github-issues-workflow/ \
        src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py \
        src/rob_box_voice/rob_box_voice/command_node.py

git commit -m "feat(04-03): GitHub Issues workflow — update instructions, add skill, fix STUB refs, delete tasks.json"
```

---

## Финальная верификация (все 7 проверок)

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
  gh api repos/{owner}/{repo}/milestones | python3 -c "import json,sys; print(len(json.load(sys.stdin)))"
```

**Ожидаемый вывод:** GH-01≥20, GH-02=PASS, GH-03≥1, GH-04=файл, GH-05=0, GH-06≥3, GH-07≥2

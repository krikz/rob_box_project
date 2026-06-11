# Phase 4: GitHub Issues Integration — Research

**Researched:** 2026-05-15  
**Domain:** GitHub CLI (gh), Python scripting, skill authoring, Markdown templating  
**Confidence:** HIGH — все данные взяты из живого репозитория и проверены командами

---

## Summary

Phase 4 переводит трекинг задач с `tasks.json` на GitHub Issues как единственный источник правды. Работа состоит из трёх упорядоченных блоков (GH-0): сначала переписываем «голову» системы (скиллы + copilot-instructions), затем создаём инфраструктуру GitHub (labels + milestones), затем мигрируем данные и удаляем `tasks.json`.

Исходных данных для миграции: **20 задач** из `tasks.json` + **≈17 новых Issues** из `TECH_DEBT.md` (остальные либо `accept`, либо уже покрыты задачами из tasks.json). Всего создаётся **≥37 Issues**.

Ключевой технический момент: команда `gh milestone` **не существует** — milestones создаются через `gh api`. Команда `gh issue develop` доступна в gh 2.74.0 и поддерживает `--checkout`. Идемпотентность обеспечивается через `gh issue list --search "{ID} in:title"` перед созданием.

**Primary recommendation:** Python-скрипт с subprocess-вызовами gh CLI; для STUB-комментариев собрать маппинг `{task_id} → {github_issue_number}` во время миграции и сделать sed-замены после.

---

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

**GH-0:** Порядок выполнения — сначала скиллы/инструкции, потом labels/milestones, потом миграция данных.

**GH-1 (Issue body format):**
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

**GH-2 (Migration scope):** Все 20 задач из tasks.json + все пункты TECH_DEBT.md кроме `accept` (≈24 пунктов), с дедупликацией по наличию TASK-ID. После миграции `tasks.json` удаляется.

**GH-3:** Labels-only, без GitHub Projects board. Фильтрация: `gh issue list --label "source:gsd"`.

**GH-4:** GitHub Milestones — M1 closed (завершён), M2/M3 open. Плюс дублирующие labels `milestone:M1`, `milestone:M2`, `milestone:M3`.

**GH-5 (Label taxonomy):** Создать 20 новых labels (конкретный список в разделе Standard Stack ниже). Существующие не пересоздавать.

**GH-6:** Ветки через `gh issue develop {N} --checkout`. Шаблон имени: `feature/{N}-{slug}`.

**GH-7:** После создания Issues для TASK-051/TASK-052 — обновить `# STUB:` комментарии в коде, заменив `TASK-051`/`TASK-052` на `#{issue_number}`.

**D-01:** Deep update (narrative rewrite) context-engineering/SKILL.md + copilot-instructions.md.  
**D-02:** Новый скилл github-issues-workflow (~150 строк lifecycle skill).  
**D-03:** Python migration script с idempotency via `gh issue list --search "TASK-XXX in:title"`.  
**D-04:** TECH_DEBT dedup только по наличию task_ref (BG-секция + TASK-051/052). Остальные → новые Issues.  
**D-05:** Не пересоздавать существующие: bug, documentation, critical, deployment, python, docker.  
**D-06:** M1 создать как closed, M2/M3 как open.  
**D-07:** Issue body с секциями ## Description, ## Acceptance Criteria, ## File References, ## Source.  
**D-08:** Ветки: `feature/{N}-{slug}` через `gh issue develop {N} --checkout`.  
**D-09:** Обновить STUB-комментарии с `TASK-051`/`TASK-052` → `#{N}`.

### Agent's Discretion

- Порядок создания Issues в migration script (по priority или по ID)
- Точная структура нового github-issues-workflow skill (формат, примеры команд)
- Обработка `done`/`in-progress` задач при создании Issues
- Как именно закрывать Issues для done-задач (отдельная команда после создания)

### Deferred Ideas (OUT OF SCOPE)

- GitHub Projects board (явно отклонено в GH-3)
- Автоматизация через GitHub Actions (не запрошено в этой фазе)
- Импорт истории коммитов в Issues
</user_constraints>

---

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| GH-01 | `gh issue list --label ai-generated` возвращает ≥20 issues | 20 задач из tasks.json + ~17 из TECH_DEBT = ≥37 issues |
| GH-02 | `ls tasks.json` — файл не существует | Скрипт удаляет tasks.json после миграции + git rm |
| GH-03 | copilot-instructions.md содержит инструкции по `gh issue` вместо `tasks.json` | Найдены все 2 строки в copilot-instructions.md для замены |
| GH-04 | Скилл `.agents/skills/github-issues-workflow/SKILL.md` создан | Паттерн взят из github-actions-runner skill (~100 строк) |
| GH-05 | STUB-комментарии содержат `#N` вместо `TASK-05X` | 8 строк в system.py и command_node.py идентифицированы |
</phase_requirements>

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Label creation | GitHub API (via gh CLI) | — | Labels — GitHub-native объекты |
| Milestone creation | GitHub API (via `gh api`) | — | Нет `gh milestone` команды |
| Issue creation | GitHub API (via `gh issue create`) | Python migration script | script вызывает gh |
| Idempotency check | Python migration script | gh CLI search | script проверяет перед созданием |
| STUB comment update | Git (sed/replace in files) | Python script | post-migration step |
| Skill authoring | Файловая система | — | Markdown файлы в .agents/skills/ |
| Instructions update | Файловая система | — | copilot-instructions.md + SKILL.md |

---

## Данные для миграции: tasks.json

### Полный список 20 задач

[VERIFIED: прочитан tasks.json напрямую]

| ID | Status | Priority | Category | Действие при миграции |
|----|--------|----------|----------|-----------------------|
| TASK-035 | in-progress | critical | functional | Issue open + `status:in-progress` |
| TASK-036 | pending | high | functional | Issue open |
| TASK-037 | pending | high | functional | Issue open |
| TASK-038 | pending | medium | functional | Issue open |
| TASK-039 | pending | medium | infrastructure | Issue open |
| TASK-040 | pending | low | documentation | Issue open |
| TASK-041 | pending | low | infrastructure | Issue open |
| TASK-042 | in-progress | critical | bugfix | Issue open + `status:in-progress` |
| TASK-043 | pending | medium | bug | Issue open |
| TASK-044 | pending | medium | bug | Issue open |
| TASK-045 | pending | low | improvement | Issue open |
| TASK-046 | pending | low | improvement | Issue open |
| TASK-047 | pending | high | testing | Issue open |
| TASK-048 | **done** | high | bug | Issue создать → immediately close |
| TASK-049 | pending | critical | bug | Issue open |
| TASK-050 | pending | medium | bug | Issue open |
| TASK-051 | todo | high | stub | Issue open + `type:stub` |
| TASK-052 | todo | medium | stub | Issue open + `type:stub` |
| SKILLS-001 | **done** | low | infrastructure | Issue создать → immediately close |
| SKILLS-002 | **done** | low | infrastructure | Issue создать → immediately close |

**Из 20 задач: 3 done → Issues создать и сразу закрыть; 2 in-progress → `status:in-progress` label.**

### Поля tasks.json и их маппинг на Issue

```
id              → заголовок title: "[{id}] {description[:80]}"
description     → ## Description секция
acceptance_criteria → ## Acceptance Criteria (checklist [ ] каждый пункт)
test_steps      → ## Acceptance Criteria включить как подсекцию (если есть)
dependencies    → ## References: "Depends on: #{issue_number} for TASK-XXX"
notes           → в конец ## Description
file + line     → ## File References
source          → ## Source (если есть поле)
priority        → label: priority:{X} (critical → existing `critical` label)
category        → label: type:{X} (bugfix → type:bug; bug → type:bug; stub → type:stub)
milestone       → label milestone:M1/M2/M3 + GitHub milestone number
status          → if done → close after create; if in-progress → label status:in-progress
```

**Особый случай: tasks.json не имеет `title` поля у большинства задач.** TASK-051 и TASK-052 имеют `title`. Для остальных title формируется из `id` + первые 80 символов `description`.

---

## Данные для миграции: TECH_DEBT.md

### Дедупликация: уже покрыты tasks.json

[VERIFIED: прочитан TECH_DEBT.md + tasks.json]

| TD-ID | Покрыт | Действие |
|-------|--------|---------|
| BG-1 | TASK-043 | НЕ создавать Issue, добавить `BG-1` в тело TASK-043 Issue |
| BG-2 | TASK-044 | НЕ создавать Issue, добавить `BG-2` в тело TASK-044 Issue |
| BG-3 | TASK-046 | НЕ создавать Issue, добавить `BG-3` в тело TASK-046 Issue |
| BG-4 | TASK-047 | НЕ создавать Issue, добавить `BG-4` в тело TASK-047 Issue |
| BG-5 | TASK-049 | НЕ создавать Issue, добавить `BG-5` в тело TASK-049 Issue |
| BG-6 | TASK-050 | НЕ создавать Issue, добавить `BG-6` в тело TASK-050 Issue |
| TD-2 | TASK-051 | НЕ создавать Issue (TASK-051 уже включает TD-2 source) |
| TD-4 | TASK-052 | НЕ создавать Issue (TASK-052 уже включает TD-4 source) |

### Новые Issues из TECH_DEBT (≈17 штук)

[VERIFIED: прочитан TECH_DEBT.md, accept items исключены]

**Пропустить (disposition=accept): TD-6, SEC-3, SEC-4, SEC-5, SL-1 (+ один возможный дополнительный по счётчику)**

| ID | Описание | Severity | Disposition | Labels | Milestone |
|----|----------|----------|-------------|--------|-----------|
| TD-1 | dialogue_node.py монолит 2040 строк | high | defer:M3 | type:tech-debt, priority:high | M3 |
| TD-3 | reflection_node.py silent fallback | medium | defer:M3 | type:tech-debt, priority:medium | M3 |
| TD-5 / PF-1 | async_executor busy-wait 50ms (ОДНА Issue) | low | defer:M3 | type:performance, priority:low | M3 |
| TD-7 | LED compositor: dual config sources | low | defer:M3 | type:tech-debt, priority:low | M3 |
| SEC-1 | Hardcoded SSH password в CI/CD | critical | fix | type:security, critical | M1 |
| SEC-2 | Password 'open' в docs/ | high | fix | type:security, priority:high | M1 |
| SEC-6 | Unpinned ollama:latest | low | defer:M3 | type:security, priority:low | M3 |
| PF-2 | dialogue_node context grows за один run | high | defer:M3 | type:performance, priority:high | M3 |
| PF-3 | voice_memory synchronous Ollama embedding | medium | defer:M3 | type:performance, priority:medium | M3 |
| FA-1 | audio_node.py bare except: pass | medium | defer:M3 | type:tech-debt, priority:medium | M3 |
| FA-2 | calibrate_max_rpm.py глотает исключения | high | defer:M2 | type:tech-debt, priority:high | M2 |
| FA-3 | VESC no NaN/infinity guard | high | defer:M2 | type:tech-debt, priority:high | M2 |
| FA-4 | Zenoh IPs hardcoded в configs | medium | defer:M2 | type:infrastructure, priority:medium | M2 |
| FA-5 | test_dialogue_node.py 13+ пустых тестов | high | defer:M3 | type:testing, priority:high | M3 |
| FA-6 | led_matrix exception в clear_matrix | low | defer:M3 | type:tech-debt, priority:low | M3 |
| SL-2 | Zenoh router = SPOF | medium | defer:M2 | type:infrastructure, priority:medium | M2 |
| SL-3 | LLM только облако, нет offline | medium | defer:M3 | type:infrastructure, priority:medium | M3 |

**Итого новых Issues из TECH_DEBT: 17**  
**TOTAL Issues к созданию: 20 + 17 = 37**

> **Примечание по дедупликации TD-5 и PF-1:** Оба описывают одну и ту же проблему (`async_executor.py` busy-wait polling 50ms). Создаётся ONE Issue с обоими ID в теле.

---

## Standard Stack

### Core Tools

[VERIFIED: gh --version в терминале]

| Tool | Version | Purpose |
|------|---------|---------|
| `gh` CLI | 2.74.0 | Создание labels, issues, milestones via API |
| Python 3 | 3.10+ (на dev-машине) | Migration script |
| `subprocess` | stdlib | Вызов gh CLI из Python |
| `json` | stdlib | Парсинг tasks.json |

### gh CLI: Ключевые команды

[VERIFIED: gh --help + gh issue develop --help]

#### Создание label
```bash
# [VERIFIED: gh label list вернул реальные labels]
gh label create "ai-generated" \
  --color "#7B61FF" \
  --description "Created by AI agent (GSD workflow)" \
  --repo krikz/rob_box_project

# Безопасное создание (idempotent): игнорировать если уже существует
gh label create "ai-generated" --color "#7B61FF" --repo krikz/rob_box_project 2>/dev/null || true
```

#### Создание milestone (через API — `gh milestone` не существует!)
```bash
# [VERIFIED: `gh milestone list` вернул error "unknown command"]
# Единственный способ — gh api

# M1 (closed):
gh api repos/krikz/rob_box_project/milestones \
  -f title="Milestone 1: Code Quality" \
  -f state="closed" \
  -f description="Аудит документации, структуры и кода"

# M2 (open):
gh api repos/krikz/rob_box_project/milestones \
  -f title="Milestone 2: Navigation & Localization" \
  -f state="open"

# M3 (open):
gh api repos/krikz/rob_box_project/milestones \
  -f title="Milestone 3: Refactoring & Voice" \
  -f state="open"

# Получить номера milestones после создания:
gh api repos/krikz/rob_box_project/milestones?state=all \
  --jq '.[] | "\(.number): \(.title)"'
```

#### Создание Issue
```bash
gh issue create \
  --title "[TASK-035] Тестирование и отладка agent cycle" \
  --body "$(cat /tmp/issue_body.md)" \
  --label "type:functional,priority:critical,ai-generated,source:gsd,status:in-progress" \
  --milestone 2 \
  --repo krikz/rob_box_project

# Получить номер созданного Issue:
gh issue create ... --json number --jq '.number'
```

#### Idempotency check
```bash
# Проверить перед созданием:
EXISTING=$(gh issue list \
  --search "TASK-035 in:title" \
  --repo krikz/rob_box_project \
  --json number,title \
  --jq '.[0].number // empty')

if [ -z "$EXISTING" ]; then
  # Создать Issue
else
  echo "Already exists: #$EXISTING"
fi
```

#### Закрыть Issue (для done-задач)
```bash
gh issue close {N} --repo krikz/rob_box_project
```

#### Создание ветки для работы над задачей
```bash
# [VERIFIED: gh issue develop --help]
gh issue develop {N} --checkout --repo krikz/rob_box_project
# Создаёт ветку и переключается на неё
# Имя ветки автоматически: {N}-{slug-from-title}
# Для кастомного имени:
gh issue develop {N} --name "feature/{N}-{slug}" --checkout
```

---

## Существующие GitHub Labels

[VERIFIED: `gh label list --repo krikz/rob_box_project`]

### Уже существуют (НЕ создавать):
| Label | Color | Использовать как |
|-------|-------|-----------------|
| `bug` | #d73a4a | type:bug → использовать `bug` |
| `documentation` | #0075ca | type:documentation → использовать `documentation` |
| `critical` | #da6e7c | priority:critical → использовать `critical` |
| `deployment` | #3ef208 | deployment issues |
| `python` | #2b67c6 | Python code changes |
| `docker` | #21ceff | Docker changes |
| `duplicate` | #cfd3d7 | duplicate |
| `enhancement` | #a2eeef | enhancement |

### Создать (20 новых labels):

| Label | Color HEX | Описание |
|-------|-----------|---------|
| `ai-generated` | #7B61FF | Created by AI agent |
| `source:gsd` | #5319E7 | Created via GSD workflow |
| `type:tech-debt` | #FFA500 | Technical debt item |
| `type:stub` | #FF6B6B | Stub implementation |
| `type:security` | #D93F0B | Security issue |
| `type:performance` | #FBCA04 | Performance issue |
| `type:functional` | #0075CA | Functional feature/task |
| `type:infrastructure` | #E4E669 | Infrastructure task |
| `type:testing` | #C2E0C6 | Testing task |
| `priority:high` | #B60205 | High priority |
| `priority:medium` | #E4A000 | Medium priority |
| `priority:low` | #CFD3D7 | Low priority |
| `milestone:M1` | #BFD4F2 | Milestone 1: Code Quality |
| `milestone:M2` | #D4C5F9 | Milestone 2: Navigation |
| `milestone:M3` | #C5DEF5 | Milestone 3: Refactoring |
| `status:in-progress` | #0052CC | Currently being worked on |
| `status:blocked` | #E11D48 | Blocked by dependency |

> Итого новых: 17 labels (3 из списка в CONTEXT.md используют существующие: `type:bug`→`bug`, `type:documentation`→`documentation`, `priority:critical`→`critical`)

### Существующие milestones в репозитории

[VERIFIED: `gh api repos/krikz/rob_box_project/milestones?state=all` вернул `[]`]

**Milestones отсутствуют.** Все три нужно создать с нуля.

---

## STUB комментарии в коде

[VERIFIED: grep по src/**/*.py + прочитаны файлы]

### system.py

**Файл:** `src/rob_box_mcp_tools/rob_box_mcp_tools/tools/system.py`

```python
# Строки 440-442 (текущий вид):
# STUB: get_robot_status возвращает hardcoded данные — реальная позиция и battery
# будут получены из /odom и /battery_state ROS topics после Nav2 интеграции (Milestone 2).
# Отслеживается: TECH_DEBT.md TD-2, tasks.json TASK-051 (STUB-get-robot-status)

# После замены:
# Отслеживается: TECH_DEBT.md TD-2, GitHub #{issue_number} (STUB-get-robot-status)
```

### command_node.py

**Файл:** `src/rob_box_voice/rob_box_voice/command_node.py`

```python
# Строка 345 (handle_status):
# Отслеживается: TECH_DEBT.md TD-2, tasks.json TASK-052 (STUB-navigation-commands)

# Строка 364 (handle_vision):
# Отслеживается: TECH_DEBT.md TD-2, tasks.json TASK-052 (STUB-navigation-commands)

# Строка 378 (handle_follow):
# Отслеживается: TECH_DEBT.md TD-2, tasks.json TASK-052 (STUB-navigation-commands)

# После замены (все три строки):
# Отслеживается: TECH_DEBT.md TD-4, GitHub #{issue_number} (STUB-navigation-commands)
```

**Стратегия замены:** migration script при создании Issue для TASK-051/TASK-052 получает `--json number`, сохраняет в маппинг `{"TASK-051": N, "TASK-052": M}`. После всей миграции — sed/python replace в файлах.

---

## context-engineering/SKILL.md: Что изменить

[VERIFIED: прочитан .agents/skills/context-engineering/SKILL.md]

### Текущий вид (строки 21-24, flow diagram):

```
tasks.json → Research → Design → Plan → Implement
   (задача)    (факты)  (архит.)  (фазы)  (команда агентов)
```

### Новый вид:
```
GitHub Issues → Research → Design → Plan → Implement
   (задача)       (факты)  (архит.)  (фазы)  (команда агентов)
```

### Related Resources секция (строки ~168-176, текущий вид):

```markdown
| Ресурс | Назначение |
|--------|-----------|
| `tasks.json` | Бэклог задач с acceptance_criteria и dependencies |
| `PRD.md` | ...
```

### Новый вид (полная замена строки tasks.json):

```markdown
| Ресурс | Назначение |
|--------|-----------|
| `gh issue list --label source:gsd` | Бэклог задач — GitHub Issues как источник правды |
| `.agents/skills/github-issues-workflow/SKILL.md` | Полный цикл работы с задачами через gh CLI |
| `PRD.md` | ...
```

### "Когда применять" секция — дополнительный триггер:

В таблице типов работы добавить/обновить:
```markdown
| Выбор задачи | `gh issue list --label source:gsd --label priority:high` |
```

### Весь скилл (narrative rewrite):

Основная идея: заменить все упоминания `tasks.json` на GitHub Issues workflow. Сохранить структуру Research→Design→Plan→Implement. Обновить примеры команд.

---

## copilot-instructions.md: Что изменить

[VERIFIED: прочитан .github/copilot-instructions.md целиком]

### Строка "Бэклог задач" в таблице (строка ~11):

```markdown
# Текущий вид:
| **Бэклог задач** | `tasks.json` | Задачи, acceptance criteria, зависимости, agent_instructions |

# Новый вид:
| **Бэклог задач** | GitHub Issues (`gh issue list --label source:gsd`) | Задачи, acceptance criteria, зависимости — используй `gh issue` для работы с бэклогом |
```

### Секция agent_instructions (если есть ссылки на tasks.json):

В agent_instructions tasks.json нет — но текст referencing tasks.json в `before_start` раздела copilot-instructions НЕ существует (это поле есть в tasks.json, не в copilot-instructions.md).

Проверить и добавить секцию по работе с `gh issue` если ещё нет.

---

## Новый скилл: github-issues-workflow

### Паттерн из github-actions-runner skill

[VERIFIED: прочитан .agents/skills/github-actions-runner/SKILL.md]

Структура reference skill: frontmatter → заголовок → таблица команд → примеры → алгоритм.

### Предлагаемая структура github-issues-workflow (~150 строк):

```markdown
---
name: github-issues-workflow
description: >
  Полный цикл работы с задачами через GitHub Issues.
  Используй когда нужно: найти задачу для работы, создать ветку,
  обновить статус, создать PR, закрыть Issue.
---

# GitHub Issues Workflow — Rob Box

## Выбор задачи

## Начало работы (ветка)

## Обновление Issue во время работы

## Создание PR и закрытие Issue

## Быстрые gh-команды (cheatsheet)

## Фильтры и поиск

## Lifecycle: полный цикл задачи
```

Детали каждой секции — в разделе Architecture Patterns ниже.

---

## Architecture Patterns

### Python Migration Script: архитектура

```
migrate_to_github_issues.py
├── load_tasks()          → парсинг tasks.json
├── load_tech_debt()      → парсинг TECH_DEBT.md (regex по таблицам)
├── deduplicate()         → убрать BG-1..6, TD-2, TD-4, accept items
├── setup_labels()        → create 17 labels (idempotent)
├── setup_milestones()    → create 3 milestones via gh api (idempotent)
├── migrate_task()        → idempotency check + gh issue create
├── migrate_tech_debt()   → idempotency check + gh issue create
├── close_done_issues()   → gh issue close для done-задач
├── update_stub_comments()→ sed replace в system.py + command_node.py
└── remove_tasks_json()   → git rm tasks.json (опционально, финальный шаг)
```

### Idempotency: детальный алгоритм

```python
def issue_exists(task_id: str, repo: str) -> int | None:
    """Возвращает номер Issue если уже существует, иначе None"""
    result = subprocess.run(
        ['gh', 'issue', 'list',
         '--search', f'"{task_id}" in:title',
         '--repo', repo,
         '--json', 'number,title',
         '--jq', f'[.[] | select(.title | contains("{task_id}"))][0].number // empty'],
        capture_output=True, text=True
    )
    number = result.stdout.strip()
    return int(number) if number else None
```

### Issue body template (Python f-string)

```python
BODY_TEMPLATE = """## Description
{description}

## Acceptance Criteria
{acceptance_criteria}

## File References
{file_refs}

## Source
Migrated from: {source_file}
Original ID: `{original_id}`

---
> ⚠️ _This issue was created by AI (GSD workflow). Label: `ai-generated`_
"""

def format_ac(criteria) -> str:
    if isinstance(criteria, list):
        return '\n'.join(f'- [ ] {c}' for c in criteria)
    return f'- [ ] {criteria}'
```

### Priority → Label mapping

```python
PRIORITY_MAP = {
    'critical': 'critical',     # existing label
    'high':     'priority:high',
    'medium':   'priority:medium',
    'low':      'priority:low',
}

CATEGORY_MAP = {
    'functional':    'type:functional',
    'infrastructure':'type:infrastructure',
    'bug':           'bug',            # existing label
    'bugfix':        'bug',            # existing label
    'improvement':   'enhancement',    # existing label
    'testing':       'type:testing',
    'documentation': 'documentation',  # existing label
    'stub':          'type:stub',
}

MILESTONE_MAP = {
    'M1': 1,  # будет заполнено после создания milestones
    'M2': 2,
    'M3': 3,
    None: 1,  # default: M1 для задач без явного milestone
}
```

### Определение milestone для задачи

```python
def get_task_milestone(task: dict) -> str:
    # Если у задачи есть явное поле milestone (TASK-051, TASK-052)
    if 'milestone' in task:
        return task['milestone']
    # Иначе по disposition из TECH_DEBT или по логике
    # tasks.json задачи: все в M1 (текущий milestone) или по категории
    return 'M1'
```

### github-issues-workflow SKILL.md: полный скелет

```markdown
---
name: github-issues-workflow
description: >
  Полный цикл работы с задачами через GitHub Issues.
  Используй когда: выбор задачи, создание ветки, обновление статуса, PR, close.
---

# GitHub Issues Workflow — Rob Box

Репозиторий: **github.com/krikz/rob_box_project**

## Найти задачу для работы

```bash
# Все открытые задачи GSD:
gh issue list --label "source:gsd" --repo krikz/rob_box_project

# По приоритету:
gh issue list --label "source:gsd" --label "priority:high" --repo krikz/rob_box_project

# Поиск по ключевому слову:
gh issue list --search "VESC" --label "source:gsd" --repo krikz/rob_box_project
```

## Начать работу над задачей

```bash
# Создать ветку и переключиться:
gh issue develop {N} --checkout --repo krikz/rob_box_project

# Или с явным именем:
gh issue develop {N} --name "feature/{N}-short-desc" --checkout
```

## Обновить Issue во время работы

```bash
# Добавить comment с прогрессом:
gh issue comment {N} --body "WIP: ..." --repo krikz/rob_box_project

# Добавить label status:in-progress:
gh issue edit {N} --add-label "status:in-progress" --repo krikz/rob_box_project
```

## Создать PR и закрыть Issue

```bash
# PR закроет Issue автоматически если в теле: "Closes #{N}"
gh pr create \
  --title "feat(scope): description" \
  --body "Closes #{N}" \
  --repo krikz/rob_box_project
```

## Закрыть вручную (если без PR)

```bash
gh issue close {N} --repo krikz/rob_box_project
```

## Cheatsheet

| Действие | Команда |
|----------|---------|
| Список задач | `gh issue list --label source:gsd` |
| Детали задачи | `gh issue view {N}` |
| Начать ветку | `gh issue develop {N} --checkout` |
| Добавить comment | `gh issue comment {N} --body "..."` |
| Закрыть Issue | `gh issue close {N}` |
| Мой статус | `gh issue list --assignee @me` |
```

---

## Common Pitfalls

### Pitfall 1: `gh milestone` не существует
**What goes wrong:** `gh milestone list` → "unknown command"
**Why it happens:** milestone management не добавлен в gh CLI как subcommand
**How to avoid:** Использовать `gh api repos/{owner}/{repo}/milestones` для создания, `?state=all` для получения списка
**Warning signs:** Error "unknown command 'milestone'"

### Pitfall 2: Дублирование Issues при повторном запуске скрипта
**What goes wrong:** Запуск скрипта дважды создаёт дублирующиеся Issues
**Why it happens:** Нет idempotency check перед созданием
**How to avoid:** Всегда проверять `gh issue list --search "{ID} in:title"` перед `gh issue create`
**Warning signs:** Два Issues с одинаковым `[TASK-035]` в заголовке

### Pitfall 3: `--label` с несуществующим label → 422 ошибка
**What goes wrong:** `gh issue create --label "nonexistent"` → HTTP 422 Unprocessable Entity
**Why it happens:** GitHub API не создаёт labels автоматически
**How to avoid:** Всегда создавать все labels ПЕРЕД созданием Issues; setup_labels() → migrate_issues()
**Warning signs:** "label 'xyz' not found" в stderr

### Pitfall 4: color hex без `#` в `gh label create`
**What goes wrong:** Цвет не применяется или ошибка
**Why it happens:** `--color` принимает hex БЕЗ `#` символа
**How to avoid:** `gh label create "name" --color "7B61FF"` (без `#`)
**Warning signs:** Label создаётся с дефолтным цветом

### Pitfall 5: Rate limiting при массовом создании Issues
**What goes wrong:** 422 или 429 при создании 37+ Issues быстро
**Why it happens:** GitHub API rate limit ~5000 req/hour для authenticated requests
**How to avoid:** Добавить `time.sleep(0.5)` между созданием Issues; 37 Issues = ~18.5 секунд
**Warning signs:** Error "secondary rate limit"

### Pitfall 6: PF-2 vs TASK-043 — не очевидная дедупликация
**What goes wrong:** Создаётся отдельный Issue для PF-2, хотя это тот же баг что TASK-043
**Why it happens:** PF-2 не имеет явного task_ref по решению D-04 (dedup only by task_ref)
**How to avoid:** Per D-04 — PF-2 НЕ имеет task_ref → создать как отдельный Issue с cross-ref на TASK-043 Issue. ИЛИ добавить task_ref в TECH_DEBT.md → dedup. Решение: создать и добавить `Related: #{N}` в тело.
**Warning signs:** Дублирующийся контент в двух Issues

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead |
|---------|-------------|-------------|
| GitHub API calls | Raw urllib/requests | `gh` CLI — аутентификация уже настроена |
| Markdown parsing | Custom regex | Простые table line splits + pandas-free approach |
| Issue dedup | Database/cache | `gh issue list --search` — GitHub Search API |
| Milestone IDs | Manual hardcode | `gh api ... --jq '.number'` после создания |

---

## Environment Availability

[VERIFIED: команды выполнены в терминале]

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| `gh` CLI | Всё | ✓ | 2.74.0 | — |
| Python 3 | migration script | ✓ | 3.10+ (dev machine) | — |
| GitHub auth | gh CLI | ✓ | (gh auth status OK) | — |
| `gh issue develop` | GH-6 (branch creation) | ✓ | 2.74.0 | manual git checkout -b |

**Milestone count verification:** После создания M1, M2, M3 получить их номера через `gh api repos/krikz/rob_box_project/milestones?state=all --jq '.[] | "\(.number): \(.title)"'` и заполнить MILESTONE_MAP в скрипте.

---

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | pytest (уже в проекте) |
| Quick run | `pytest scripts/test_migration_script.py -x` |
| Verification | `gh issue list --label ai-generated --json number --jq 'length'` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command |
|--------|----------|-----------|-------------------|
| GH-01 | ≥20 issues с ai-generated label | smoke | `gh issue list --label ai-generated --json number --jq 'length \| . >= 20'` |
| GH-02 | tasks.json не существует | smoke | `test ! -f tasks.json` |
| GH-03 | copilot-instructions.md содержит gh issue | smoke | `grep -q "gh issue" .github/copilot-instructions.md` |
| GH-04 | github-issues-workflow SKILL.md существует | smoke | `test -f .agents/skills/github-issues-workflow/SKILL.md` |
| GH-05 | STUB без TASK-05X, с #{N} | smoke | `grep -r "TASK-051\|TASK-052" src/ \| grep -v "^Binary"` — должно быть пусто |

### Wave 0 Gaps

- [ ] `scripts/migrate_to_github_issues.py` — основной migration script
- [ ] `scripts/test_migration_idempotency.sh` — smoke test запуска дважды

---

## Security Domain

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V5 Input Validation | yes | Escape shell args через `subprocess` list form (не `shell=True`) |
| V2 Authentication | no | gh CLI auth уже настроен |

**Критично:** Миграционный скрипт ДОЛЖЕН использовать `subprocess.run(['gh', ...], ...)` (list form), не `subprocess.run(f"gh issue create ...", shell=True)`. Иначе — injection через поля задач (описания содержат спецсимволы).

---

## Open Questions

1. **Для `done` tasks (SKILLS-001, SKILLS-002, TASK-048) — закрывать как resolved?**
   - Что знаем: 3 задачи со статусом `done`, Issue нужен для полноты истории
   - Неясно: закрывать с каким `--reason` (completed/not_planned)?
   - Рекомендация: `gh issue close {N} --reason completed` сразу после создания

2. **Milestone для tasks.json задач без явного `milestone` поля**
   - Что знаем: TASK-051/052 имеют `"milestone": "M2"`, остальные — нет
   - Рекомендация: задачи без milestone → M1 (текущий milestone), кроме задач с nav2/vision → M2

3. **tasks.json `agent_instructions` секция — перенести в Issue?**
   - Что знаем: глобальные `agent_instructions` (before_start/during_work/before_finish) есть в корне tasks.json
   - Рекомендация: эти инструкции → в обновлённый copilot-instructions.md (не в Issues)

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | Всего 5 accept items (TD-6, SEC-3, SEC-4, SEC-5, SL-1), не 6 как в stats | TECH_DEBT.md Migration | Один лишний item пропустим или создадим лишний Issue |
| A2 | gh auth уже настроен и имеет права на issues/labels/milestones | Environment | Скрипт не запустится без auth |
| A3 | Номера созданных Milestones будут 1/2/3 (первые в репо) | MILESTONE_MAP | Номера могут быть другими если были deleted milestones → всегда получать через API после создания |
| A4 | PF-2 (dialogue context grows) отдельный Issue, не дедуплицирует TASK-043 | TECH_DEBT dedup | Лишний Issue если решить что это один баг |

---

## Sources

### Primary (HIGH confidence)
- [VERIFIED: tasks.json прочитан напрямую] — все 20 задач с полями
- [VERIFIED: .planning/TECH_DEBT.md прочитан] — 30 items, disposition map
- [VERIFIED: gh label list] — реальные существующие labels
- [VERIFIED: gh issue list] — существующие Issues (~800 deployment-generated)
- [VERIFIED: gh api milestones] — пустой список, milestones нужно создать
- [VERIFIED: gh --version] — 2.74.0
- [VERIFIED: gh issue develop --help] — --checkout флаг существует
- [VERIFIED: grep src/**/*.py STUB] — точные строки в system.py и command_node.py
- [VERIFIED: context-engineering/SKILL.md прочитан] — точные строки для замены
- [VERIFIED: copilot-instructions.md прочитан] — строки tasks.json для замены

---

## Metadata

**Confidence breakdown:**
- tasks.json inventory: HIGH — прочитан и parsed
- TECH_DEBT.md dedup: HIGH — прочитан, cross-refs проверены
- gh CLI commands: HIGH — версия проверена, команды проверены
- Existing labels: HIGH — `gh label list` выполнен
- Existing milestones: HIGH — пусто, все создавать
- STUB locations: HIGH — grep подтверждён
- Skills authoring: HIGH — паттерн из github-actions-runner

**Research date:** 2026-05-15  
**Valid until:** 2026-06-15 (stable domain)

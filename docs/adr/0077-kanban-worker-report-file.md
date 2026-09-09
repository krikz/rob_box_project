# ADR-0077: kanban-worker report file — воркеры ОБЯЗАНЫ сохранять `docs/reports/kanban/<task_id>.md` перед `kanban_complete`

| Поле | Значение |
|---|---|
| Статус | Accepted |
| Дата | 2026-09-08 |
| Автор | devops (Hermes Agent); карточка `t_84434d4c`, issue #2159 |
| Контекст | Юзер Шифу: «проанализируй карточки в канбане на счёт ревью, они ничего не оставляют после себя, ни каких артефактов». Текущий `kanban_complete` (hermes_cli.kanban_tools) сохраняет только Result (~300 символов), Summary (одно предложение) и Artifacts (список путей БЕЗ содержимого) — через 30 дней worktree может быть удалён, и тогда для ретро/аудита не остаётся ничего, кроме «done». |
| Затрагивает | (a) новый скрипт `scripts/agent_flow/kanban-report-write.sh` — воркер дёргает его перед `kanban_complete`; (b) `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс; (c) `scripts/agent_flow/install.sh` — `EXPECTED+=kanban-report-write.sh`; (d) `docs/reports/kanban/README.md` — контракт директории; (e) `docs/reports/kanban/t_*.md` — сами отчёты (по одному на kanban-карточку); (f) `AGENTS.md` — секция «Контракт отчёта»; (g) `docs/process/...` — ретроспектива-скрипты получают canonical source-of-truth. **НЕ затрагивает** hermes CLI / ядро — это **контракт воркера**, не ядра. |
| Родители | ADR-0018 (raw-evidence обязателен → воркер ПИШЕТ доказательства в файл, который переживёт worktree), ADR-0013 (incremental delivery → отчёт — артефакт размером ~5–10 КБ, влезает в один инкрементальный PR). |
| Связанные | t_84434d4c (эта карточка), issue #2159, `t_88230c1c` (пример «304 символа result + 3 пустых artifact» — паттерн, который ADR закрывает), `scripts/agent_flow/kanban-retro-create.sh` (образец helper-скрипта для воркера), `scripts/agent_flow/agent-flow-merge-gate.sh` (читает отчёты при ретро-аудите), `scripts/agent_flow/agent-flow-nightly-review.sh` (читает отчёты при ночном обзоре). |

## 1. Контекст и бизнес-проблема

### 1.1 Что наблюдаем (на 2026-09-08)

Юзер Шифу после серии ревью заметил: воркеры при `kanban_complete` оставляют **слишком мало артефактов** для аудита. Текущий контракт `kanban_tools.kanban_complete`:

| Поле | Сейчас | Достаточно? |
|---|---|---|
| `Result` (events.result) | 304 символа, одно предложение | ❌ коротко |
| `Artifacts` (events.artifacts) | список путей **без содержимого** | ❌ только индекс |
| `Summary` (events.summary) | одно предложение | ❌ нет деталей |
| Git diff | в ветке worktree | ⚠️ зависит от worktree (через 30 дней может быть удалён) |
| Логи pytest | **не сохраняются** | ❌ нет raw-evidence |
| Логи e2e | **не сохраняются** | ❌ нет raw-evidence |

Пример — `t_88230c1c` (17 минут работы): в `Result` сохранено одно предложение, в `Artifacts` — 3 пути к файлам, но **содержимое файлов не сохранено** (через 30 дней worktree может быть удалён, и тогда для ретро нечего читать). Нет raw-вывода pytest, нет логов CI workflow run, нет git log изменённых файлов.

### 1.2 Сырые доказательства

**Текущая сигнатура `kanban_complete` (hermes_cli.kanban_tools):**

```python
def kanban_complete(task_id: str, summary: str = None,
                    metadata: dict = None, result: str = None,
                    artifacts: list[str] = None) -> dict:
    # events.result: result or summary (одно поле — ~300 символов)
    # events.summary: summary
    # events.metadata: dict (структурированные факты)
    # events.artifacts: список путей — индекс, без содержимого
    ...
```

**Пример сухого события (после `kanban_complete` для `t_88230c1c`):**

```
events.kind=completed
events.payload.result = "Issue #2122 (HIGH) — Deploy and Verify рапортовал success
                         при контейнере в Restarting loop. Принял в работу,
                         диагностика + регрессия + PR."   (304 символа)
events.payload.summary = events.payload.result
events.payload.artifacts = [
    "/home/builder/.hermes/kanban/boards/robbox/workspaces/t_88230c1c/wt/src/...",
    "/home/builder/.hermes/kanban/boards/robbox/workspaces/t_88230c1c/wt/scripts/...",
    "/home/builder/.hermes/kanban/boards/robbox/workspaces/t_88230c1c/wt/docs/...",
]   # 3 пути, файлы НЕ инлайнятся
```

**Что теряется через 30 дней (worktree GC):**

- содержимое файлов по путям в `Artifacts`;
- `pytest -v` output (не сохранилось вообще);
- логи CI workflow run (`gh run view <id>` ссылки нет);
- `git log --stat` для изменённых файлов;
- способ перепроверить результат воркера через месяц.

## 2. Гипотеза (root cause)

Hermes kernel **не может** форсить запись полного отчёта воркером: это слишком opinionated (разные профили делают разную работу, формат отчёта будет варьироваться). Правильный уровень — **контракт воркера**:

- Воркер (agent-flow) ОБЯЗАН перед `kanban_complete` создать `docs/reports/kanban/<task_id>.md` в ветке worktree.
- Файл лежит в репо → переживает worktree GC (через 30 дней worktree удаляется, но git tracked файл — остаётся).
- Файл содержит **минимальный набор raw-evidence**, чтобы ревьюер/аудитор через месяц мог восстановить картину.

Альтернативы:

- (а) Пропатчить `hermes_cli.kanban_tools.complete`, добавить поле `report: str` (markdown body). Плюс — централизованно. Минус — cross-profile запрет (правка hermes-agent), плюс ограничение по размеру (~32 КБ markdown на событие), плюс уход от git-tracked SOT. **Не делаем** в этой карточке.
- (б) **Контракт воркера** (этот ADR) — воркер дёргает helper-скрипт `kanban-report-write.sh`, который собирает git diff / pytest / gh pr view и пишет файл в worktree. Плюс — git tracked, переживает worktree GC, формат под контролем воркера. Минус — контракт не enforced ядром (полагаемся на честность воркера + ревью).

## 3. Решение

### 3.1 Контракт отчёта

`docs/reports/kanban/<task_id>.md` — markdown-файл, **tracked в git**, создаётся воркером **ДО** `kanban_complete`.

**Минимальный контракт (DoD):**

```markdown
# Отчёт: <task_title>

**Task ID:** t_xxxxxxxx
**Assignee:** backend / frontend / devops / architect
**Issue/PR:** #1234 / #5678
**Started:** 2026-09-08 02:28 UTC
**Completed:** 2026-09-08 02:45 UTC
**Duration:** 17m 0s

## Что сделано
- Пункт 1
- Пункт 2

## Файлы изменены
- `src/foo.py` (+12 -3)
- `src/bar.py` (+5 -0)

## Git log (последние N коммитов)
```
<git log --oneline -10>
```

## Raw-evidence
- pytest: см. встроенный блок
- CI: run_id ссылки
- gh pr view: ссылка

## PR / Issue
- PR #2127 — https://github.com/.../pull/2127

## Замечания
- Caveats
- Что НЕ сделано
- Что осталось для следующей карточки
```

### 3.2 Helper-скрипт `scripts/agent_flow/kanban-report-write.sh`

Воркер вызывает скрипт **перед** `kanban_complete`:

```bash
# Из worktree воркера:
TASK_ID=t_xxxxxxxx
bash scripts/agent_flow/kanban-report-write.sh "$TASK_ID"

# Скрипт:
#   1. mkdir -p docs/reports/kanban/
#   2. Собирает git diff, pytest output, gh pr view (best-effort)
#   3. Пишет docs/reports/kanban/${TASK_ID}.md
#   4. git add + commit (--allow-empty если изменений нет)
#   5. Возвращает 0 при успехе, 1 при ошибке (с actionable stderr)
```

**Что собирает скрипт (best-effort, отсутствующие секции → `n/a`):**

1. **Title + body + assignee** — из `hermes kanban show $TASK_ID --json`.
2. **Started / Completed / Duration** — из `task_events WHERE task_id=? AND kind IN ('claimed','completed')`.
3. **Файлы изменены** — `git diff --stat origin/develop...HEAD`.
4. **Git log** — `git log --oneline origin/develop..HEAD`.
5. **PR link** — `gh pr list --head $(git rev-parse --abbrev-ref HEAD) --json number,url`.
6. **pytest output** — `pytest -v 2>&1 | tail -40` (если есть `tests/`).
7. **CI run_id** — `gh pr checks --json name,conclusion,databaseId`.

Если что-то не получилось — секция получает значение `n/a (reason: ...)`, а не падает.

### 3.3 Контракт воркера

**Перед** каждым `kanban_complete` воркер **ОБЯЗАН**:

1. Вызвать `bash scripts/agent_flow/kanban-report-write.sh $HERMES_KANBAN_TASK`.
2. Убедиться, что файл создан (`test -f docs/reports/kanban/${HERMES_KANBAN_TASK}.md`).
3. Закоммитить файл (`git commit -m "report(<task_id>): <title>"`).
4. Запушить ветку (`git push`).
5. **Только после** зелёного push → `kanban_complete`.

**Предупреждение** (warning, не блокер): если воркер забыл шаг 1 и зовёт `kanban_complete`, скрипт `agent-flow-completion-check.sh` (или новый `kanban-report-guard.sh`) пишет в PR comment:

```
🤖 [agent:devops] ⚠️ report-file missing: docs/reports/kanban/t_xxxxxxxx.md.
Воркер не оставил отчёт. Ретро через месяц будет невозможен.
```

**Не блокируем** `kanban_complete` — это будет over-engineering (воркер может легитимно делать 5-минутные мелочи, где отчёт избыточен). Фиксируем только в PR-комментарии для ревью.

### 3.4 Регистрация

- `scripts/agent_flow/install.sh` → `EXPECTED+=(kanban-report-write.sh)` (рядом с `kanban-retro-create.sh`).
- `scripts/agent_flow/install.sh` → новая cron-job? **Нет** — это не no-agent job, это воркер-helper. Просто регистрация в EXPECTED для drift-detect.
- `AGENTS.md` → секция «Контракт отчёта» (3 строки, см. §6).
- `docs/reports/kanban/README.md` → описание директории, ссылка на ADR-0077.

### 3.5 Что НЕ делаем

- **Не патчим hermes CLI** — это cross-profile запрет + architectural scope.
- **Не форсим блокировку kanban_complete** — over-engineering; воркер может делать мелочи, где отчёт избыточен.
- **Не пишем helper на Python** — bash достаточно (как `kanban-retro-create.sh`, как `validate_honesty.sh`), плюс воркеры уже source'ят bash-библиотеки.
- **Не храним отчёты в БД kanban** — это увеличит kanban.db и сломает ротацию; git — лучший SOT.

## 4. Альтернативы (рассмотренные и отклонённые)

- **(а) Пропатчить `kanban_tools.complete`** — добавить `report: str` поле. Минус: cross-profile запрет + ограничение размера события + уход от git-tracked SOT. Отклонено.
- **(б) Хранить отчёты вне репо** — `~/.hermes/reports/kanban/<task_id>.md`. Минус: не переживает очистку `~/.hermes`, не версионируется, невидим в code review. Отклонено.
- **(в) Воркер шлёт отчёт в issue comment** — `gh issue comment <N> --body-file report.md`. Минус: issue comment редактируется, теряется; PR review идёт мимо; нет линка с task_id. Отклонено.
- **(г) Воркер пишет отчёт в kanban metadata** — `kanban_complete(metadata={"report": "..."})`. Минус: 32 КБ на событие, не git-tracked, нет cross-task archive линка. Отклонено.
- **(д) Не делать ничего** — оставить как есть. Минус: ровно та проблема, которую юзер озвучил. Отклонено.

## 5. Trade-offs

- **Дублирование vs не-игнорируемость.** `kanban_complete` Summary + новый отчёт частично перекрываются. Решили: Summary остаётся коротким one-liner для board view, отчёт — полный markdown для аудита. Разделение по формату и аудитории.
- **Helper bash vs Python.** Bash проще, паттерн уже есть (`kanban-retro-create.sh`, `validate_honesty.sh`). Python перегруз для задачи «собрать 5 полей из gh/git/pytest».
- **Не блокируем, а warn.** Если заблокировать `kanban_complete` при отсутствии отчёта — сломаются мелкие карточки (5-мин fix). Предупреждение в PR-комментарии достаточно для ревью.
- **Git tracked vs DB.** Git tracked переживает worktree GC и ротацию `~/.hermes`. DB хранит только индекс → нет cross-task archive → ретро невозможен.

## 6. Acceptance

- [ ] `scripts/agent_flow/kanban-report-write.sh` создан, читает `task_id` из argv, пишет `docs/reports/kanban/${TASK_ID}.md`.
- [ ] `scripts/agent_flow/tests/test_kanban_report_write.sh` создан, ≥5 сценариев: clean run, missing task_id (exit 2), no worktree (exit 0 с `n/a` секциями), no pytest (`n/a` секция), no PR (`n/a` секция).
- [ ] `EXPECTED` `scripts/agent_flow/install.sh` содержит `kanban-report-write.sh`.
- [ ] `docs/reports/kanban/README.md` создан с описанием контракта.
- [ ] `AGENTS.md` дополнен секцией «Контракт отчёта воркера».
- [ ] Эта карточка (`t_84434d4c`) имеет собственный отчёт `docs/reports/kanban/t_84434d4c.md` — dogfooding.
- [ ] PR открыт, CI зелёный (test_kanban_report_write.sh проходит).

## 7. Связанные (после PR)

- `docs/adr/0077-kanban-worker-report-file.md` — этот документ.
- `scripts/agent_flow/kanban-report-write.sh` — helper.
- `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс.
- `docs/reports/kanban/README.md` — контракт директории.
- `docs/reports/kanban/t_*.md` — отчёты по kanban-карточкам (появляются по мере dogfooding).
- `AGENTS.md` секция «Контракт отчёта».
- `scripts/agent_flow/install.sh` — `EXPECTED+=kanban-report-write.sh`.
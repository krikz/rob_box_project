# Kanban worker report template (issue #2159, #2162)

Этот шаблон — **канонический SOT** для отчётов воркеров в
`docs/reports/kanban/<task_id>.md`. Создан как часть процесса (issue #2159):
«воркеры должны сохранять полные отчёты перед `kanban_complete`, иначе через
месяц после архивации нечего ревьюить».

## Когда создаётся

Перед `kanban_complete` воркер ОБЯЗАН создать файл
`docs/reports/kanban/<task_id>.md` (по этому шаблону), закоммитить его
**в ту же ветку**, что и PR, и запушить. PR, в котором нет такого файла,
получает **мягкий warning** в merge-gate (логируется в event, но
`kanban_complete` НЕ блокируется — ретро 18.08 #1: жёсткая блокировка
ломает горячие фиксы, где отчёт нерелевантен).

## Где живёт заполненный отчёт

`docs/reports/kanban/<task_id>.md` (в репо, git tracked, переживает
архивацию worktree).

## Как заполнять

Воркер вызывает `scripts/agent_flow/kanban-report-write.sh <task_id>`,
скрипт подставляет git log / diff / pytest / CI из локального контекста
и сохраняет в `<worktree>/docs/reports/kanban/<task_id>.md`. После этого
воркер дописывает свободные секции (`Что сделано`, `Skill results` и
т.д.) руками — **до** `kanban_complete`.

## Шаблон

````markdown
# Отчёт: <task_title>

**Task ID:** <task_id>
**Assignee:** <assignee> (backend / frontend / devops / architect / tester)
**Issue:** #<issue>
**PR:** #<pr>  — https://github.com/krikz/rob_box_project/pull/<pr>
**Branch:** `<branch>`
**Started:** <started_at>
**Completed:** <completed_at>
**Duration:** <duration>

## Что сделано

- Пункт 1 (конкретное действие с артефактом)
- Пункт 2
- Пункт 3

## Файлы изменены

```
<вывод git diff --stat origin/develop..HEAD>
```

## Git log

```
<вывод git log --oneline origin/develop..HEAD>
```

## Raw-evidence (pytest / CI / логи)

### pytest
```
<вывод pytest -v>
```

### CI (gh run view / gh pr checks)
- Run #<run_id> — SUCCESS / FAIL
- Check: <name> — SUCCESS
- PR #<pr> mergeable: clean / dirty

### Логи / e2e (если применимо)
```
<30-50 строк docker logs / pytest -v / e2e output>
```

## Skill results (ВАЖНО — что дал каждый skill)

Воркер ОБЯЗАН перечислить skills из секции `## Skills` в body карточки
(issue #2162) и для каждого указать: что делал, что нашёл, что применил.

### verification-before-completion
- [ ] pytest -v: N passed, 0 failed (raw вывод выше)
- [ ] gh pr checks: все required зелёные
- [ ] git status: чисто (или только ожидаемые untracked)
- [ ] честный FAIL лучше красивого PASS (ADR-0018)

### code-review
- <OpenAI agent feedback / ручной review>
- <какие замечания применены, какие отклонены с обоснованием>

### writing-for-agents
- <что в body карточки помогло следующему воркеру>
- <что добавить / убрать>

### senior-devops
- <CI/CD best practices применённые здесь>
- <что изменилось в pipelines / configs>

## PR / Issue ссылки

- PR #NNNN — https://github.com/krikz/rob_box_project/pull/NNNN
- Issue #XXXX — https://github.com/krikz/rob_box_project/issues/XXXX

## Замечания / Caveats

- Что НЕ сделано (если осталось для follow-up карточки)
- Известные проблемы / отложенные TODO
- Какие ADR/доки появились по итогам работы
````

## Acceptance (для воркера)

Прежде чем сказать «готово» и вызвать `kanban_complete`, проверь:

- [ ] Файл `docs/reports/kanban/<task_id>.md` существует в worktree
- [ ] Все секции шаблона заполнены (или явно помечены «N/A» с причиной)
- [ ] Git log / diff / pytest / CI output — **сырые**, не пересказ
- [ ] PR ссылка реальная (не выдуманная)
- [ ] Список Skill results совпадает с `## Skills` секцией в body карточки
- [ ] Файл закоммичен и запушен в ту же ветку, что и PR
- [ ] `kanban_complete` вызван **после** push отчёта

## Связанные

- issue #2159 (этот шаблон — фича)
- issue #2162 (skills в body карточки)
- AGENTS.md / ADR-0018 (raw-evidence обязателен, честный FAIL > красивый PASS)
- `scripts/agent_flow/kanban-report-write.sh` — генератор
- `scripts/agent_flow/tests/test_kanban_report_write.sh` — регресс

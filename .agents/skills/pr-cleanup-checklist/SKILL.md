---
name: pr-cleanup-checklist
description: Use when товарищ Шифу просит «почисть open PR» / «audit + cleanup» / «дubli закрыть» / «pollution убрать». Сводный чеклист для chore-задачи типа #2451 — что проверить, что закрыть, что revert'ить, что задокументировать.
---

# pr-cleanup-checklist

**Source of truth**: issue #2451 + отчёт `docs/process/2026-09-14-pr-cleanup-sweep.md`
+ ADR-0095 + skill `rebase-pollution-check`.

Эта skill — **runbook для воркера devops или pr-reviewer**, который берёт
задачу вида «cleanup open PR». Не для воркера, который пилит свою фичу.

Соседняя skill `rebase-pollution-check` ловит pollution **в своей ветке**
до push; эта — отвечает за **глобальный аудит** всех PR в репо.

## Когда применять

- В карточке/issue есть «audit», «cleanup», «open PR», «дubli», «pollution».
- Товарищ Шифу жалуется: «там дубли и херня вкомиченная в некоторых».
- Счётчик open PR в develop > 5 (нормальный steady-state для rob_box ≈ 3-7).
- Возник новый pollution-source PR (типа #2352 ADR-0089 hailo) — проверить,
  не подхватил ли его кто-то в rebase.

## Ритуал (5 шагов)

### Шаг 1. Получить полный список open PR через REST

GraphQL `gh pr list --state open` падает с `API rate limit already exceeded`
на общем бюджете. Используй REST напрямую:

```bash
export GH_CONFIG_DIR=/home/builder/.config/gh
export GH_TOKEN=$(gh auth token)

# Список ВСЕХ open PR (raw JSON):
curl -sS -H "Authorization: token $GH_TOKEN" \
    "https://api.github.com/repos/krikz/rob_box_project/pulls?state=open&per_page=100" \
    > /tmp/open_prs.json

# total_count через search/issues (надёжнее для больших репо):
curl -sS -H "Authorization: token $GH_TOKEN" \
    "https://api.github.com/search/issues?q=is:pr+is:open+repo:krikz/rob_box_project&per_page=100" \
    | python3 -c "import json,sys; print(json.load(sys.stdin).get('total_count'))"
```

Если `total_count = 0` → cleanup-sweep тривиален: всё уже разрулено
(проверь, может PR #2447-style «pre-merge pollution gate» уже merged).
Сделай только Phase 4 (отчёт) и завершай.

### Шаг 2. Per-PR аудит

Для каждого PR из списка вытащить:

```bash
PR=2429  # пример
gh pr view $PR
gh pr diff $PR --name-only
git log --merges origin/develop --grep="#$PR" --oneline
```

Категоризируй в 4 группы:

| Категория | Что значит | Действие |
|---|---|---|
| **Чистый** | `diff --name-only` ⊂ issue-scope (только allowed prefixes) | ничего |
| **С pollution** | в diff есть файлы вне issue-scope | см. Шаг 4 |
| **Дубль merged** | issue, который PR закрывает, уже resolved через другой PR | close с комментарием «superseded by #X (merged <sha>)» |
| **Orphan** | у PR нет issue, title непонятный | спросить автора (через @mention в PR comment) |

### Шаг 3. Закрыть дубли

Если issue уже merged через другой PR:

```bash
# Комментарий + close:
gh pr close $PR --comment "superseded by #$SUPERSEDER (merged $(git rev-parse --short $SUPERSEDER_SHA))"
```

- **Не закрывай PR**, если у него полезная работа (новые тесты,
  переименования) — пусть автор сам решит, force-push vs cherry-pick.
- **Закрывай только**, когда issue уже merged и PR добавляет 0 net new.

### Шаг 4. Очистить pollution

Если в PR есть файлы вне issue-scope:

```bash
# Вариант A: revert единичного коммита (если pollution = один коммит):
git revert <pollution-commit-sha>

# Вариант B: drop при rebase (если мусор в нескольких коммитах):
git fetch origin develop
git rebase -i origin/develop
# в редакторе: drop коммиты с pollution или edit + git rm

# Вариант C: забрать pollution из HEAD + взять base-версию:
git checkout origin/develop -- <pollution-file>
git rm <pure-new-junk-file>
git commit --amend --no-edit

git push --force-with-lease
```

Запусти `rebase-pollution-check` (sibling skill) для проверки перед push.

### Шаг 5. Документация

После cleanup создай/обнови:

1. **`docs/process/<YYYY-MM-DD>-pr-cleanup-sweep.md`** — отчёт:
   - Что было (raw-evidence: список PR + категория для каждого).
   - Что сделано (Phase 2-3).
   - Что осталось (Phase 4 deferred items — отдельные issue).
   - Связанные kanban-карточки + commit hashes.

2. **Если создал новую категорию ошибок** (не pollution, не дубли)
   — добавь в `docs/process/<YYYY-MM-DD>-pr-cleanup-sweep.md` секцию
   «New pattern» с raw-evidence.

3. **Не закрывай upstream issue** (#2444 и подобные) руками —
   только через e2e-process / Шифу (per AGENTS.md «не фиксить баги руками»).

## Грабли

1. **НЕ используй GraphQL** для списка PR — rate limit упрётся.
   REST `repos/.../pulls` идёт мимо GraphQL budget.
2. **НЕ закрывай PR по теме «pollution»** — это решается **revert**,
   а не close. Закрытие = потеря работы.
3. **НЕ force-push в чужую ветку** (если ты не её автор). Используй
   `@author` в PR-комментарии, чтобы владелец сделал rebase сам.
4. **НЕ трогай `gh pr list --json`** — это GraphQL, упадёт по rate limit.
5. **НЕ делай `git push --force`** — только `--force-with-lease`.
6. **Не путай pollution vs out-of-scope**:
   - Pollution = мусор от чужого эпика (hailo_*, voice_capture_*.test.ts).
   - Out-of-scope = свой файл, который не подпадает под allowed prefix.
7. **Если 0 open PR** — не выдумывай работу. Сделай только отчёт
   (Phase 4) и завершай через `kanban complete` с summary
   «premise obsolete: cleanup sweep, 0 open PR, validate_pr_scope already
   в develop через #2447».

## Когда НЕ применять

- Ты сам автор PR и фиксишь pollution в своей ветке → используй
  `rebase-pollution-check`, не эту skill.
- Карточка про конкретный PR, не про sweep → иди в `rebase-pollution-check`.
- Нужно добавить новый post-merge gate (cron, ADR) → отдельная карточка
  с архитектором, не эта skill.

## Phase 4 deferred items (на 2026-09-14)

Phase 4 issue #2451 просил **post-merge sanity** в `validate_pr_scope.sh`.
Это отложено: post-merge gate — другая задача (cron, ADR, тесты).
Если нужно — заводи `process: post-merge regression detector`
и передай архитектору.

## Ссылки

- Issue #2451 — оригинальная карточка cleanup-sweep
- `docs/process/2026-09-14-pr-cleanup-sweep.md` — отчёт (2026-09-14)
- ADR-0095 — pre-merge pollution gate (от PR #2447)
- `.agents/skills/rebase-pollution-check/SKILL.md` — sibling skill
- `scripts/agent_flow/validate_pr_scope.sh` — pre-merge gate (mode=pre-merge)
- AGENTS.md → секция «Процесс-команды»

## Sources of truth

SOT (RePo): `.agents/skills/pr-cleanup-checklist/SKILL.md`. Установка:
`bash scripts/agent_flow/install.sh` (раскладка на хост по EXPECTED).
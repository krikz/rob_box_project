---
name: rebase-pollution-check
description: Use when rebase'ишь ветку на origin/develop (или cherry-pick'ишь из чужого места) и хочешь поймать pollution — мусорные файлы, которые попали в HEAD не от твоего issue. Шаг «после rebase перед push: validate_pr_scope.sh в pre-merge режиме».
---

# rebase-pollution-check

**Source of truth**: ADR-0095 + issue #2444 + `analysis/diagnose-2444-pr-pollution.md`.

Pollution = файлы в HEAD, которые не относятся к твоему issue. Обычно приходят после rebase на свежий `origin/develop` (после squash-merge чужого эпика вроде #2349 hailo или #2003 pregenerate). PR #2444 — 5 из 6 OPEN PR в rob_box_project содержат pollution, и юзер не может их закрыть.

Эта skill ловит pollution **до** `git push`, чтобы воркер мог почистить HEAD без force-push по чужим веткам.

## Когда применять

- После `git fetch origin develop` + `git rebase origin/develop` (или `git pull --rebase origin develop`).
- После `git cherry-pick` из ветки, которая расходилась с develop.
- Перед `gh pr create` / `gh pr edit --add-base`.
- Перед `kanban complete` (если ветка публикуется в remote).

## Ритуал (5 шагов)

### Шаг 1. fetch + rebase

```bash
git fetch origin develop --prune
git rebase origin/develop
# если конфликт — разрешай ТОЛЬКО по файлам своего issue-scope
# (см. шаг 4: списки allowed-prefixes)
```

### Шаг 2. pre-merge проверка

Запусти `validate_pr_scope.sh` в режиме `pre-merge` (= смотрит working tree + index + untracked **vs** `origin/develop`):

```bash
PR_SCOPE_MODE=pre-merge \
PR_ALLOWED_PREFIXES="docs/adr/,src/rob_box_voice/,<твой scope>," \
bash scripts/agent_flow/validate_pr_scope.sh origin/develop
```

**Ожидаемый результат** для чистой ветки: `exit 0`, текст `OK: all N files in allowed scope`.
**Если `exit 1`**: в stderr — список файлов вне scope. Это и есть pollution.

### Шаг 3. категоризация pollution

Каждый pollution-файл классифицируй:

- **Чужой wip-артефакт** (например, `docker/vision/vision-hailo/hailo_smoke.py`, `voice_capture_*.test.ts` от смежных эпиков) → удалить.
- **Случайно закоммиченный свой файл** (не относится к issue) → удалить или перенести в отдельный коммит.
- **Действительно нужен** для твоего issue → расширь `PR_ALLOWED_PREFIXES` явно и напиши в карточке почему.

### Шаг 4. очистка (мини-выбор)

```bash
# Удалить pollution из HEAD + working tree:
git checkout origin/develop -- <pollution-file-1> <pollution-file-2> ...
# (это возвращает ФАЙЛЫ к base-версии; для НЕ-в-base файлов → git rm)
git rm <pure-new-junk-file>
git commit --amend --no-edit
```

Альтернатива (если pollution пришла единым коммитом):

```bash
git rebase -i origin/develop
# в редакторе: drop коммит с pollution или edit и почистить файлы
```

### Шаг 5. повторная проверка + push

```bash
PR_SCOPE_MODE=pre-merge \
PR_ALLOWED_PREFIXES="..." \
bash scripts/agent_flow/validate_pr_scope.sh origin/develop
# exit 0 → ОК

git push --force-with-lease
```

`--force-with-lease` (а не `--force`) — иначе снесёшь чужие коммиты, если кто-то уже запушил в ту же ветку.

## Грабли

1. **НЕ запускай `git push --force`** — используй `--force-with-lease` (защита от race-condition с parallel push).
2. **НЕ опирайся только на `git status`** — `git status` показывает только staged/unstaged, но untracked pollution может выглядеть как обычный новый файл. `validate_pr_scope.sh` с `PR_SCOPE_MODE=pre-merge` видит ВСЁ (включая untracked).
3. **НЕ пропускай step 2** — даже если dev-mode «режим INFO». Воркер ОБЯЗАН передать `PR_ALLOWED_PREFIXES` явно, иначе gate уходит в INFO.
4. **Не путай pollution и out-of-scope**:
   - Pollution = мусор от чужого эпика (hailo_*, voice_capture_*.test.ts, evidence/).
   - Out-of-scope = свой файл, который не подпадает под allowed prefix (например, ADR в PR, где карточка про fix в коде).
5. **`SKIP_PR_SCOPE=true`** — last resort. Если используешь — пиши почему в комментарии к issue.

## Ссылки

- ADR-0095 — формальное обоснование
- issue #2444 — оригинальный bug-report
- `analysis/diagnose-2444-pr-pollution.md` — raw-evidence диагностика t_e43619b6
- `scripts/agent_flow/validate_pr_scope.sh` — сам скрипт
- `scripts/agent_flow/tests/test_validate_pr_scope.sh` — регресс-тесты (сценарии I, J — pre-merge)
- AGENTS.md → секция «Правила rebase»

## Sources of truth

SOT (RePo): `.agents/skills/rebase-pollution-check/SKILL.md`. Установка: `bash scripts/agent_flow/install.sh` (раскладка на хост по EXPECTED).

Можно запускать как pre-commit hook или pre-push hook (через `.git/hooks/pre-push` → skill).

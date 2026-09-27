---
name: worker-rebase-protocol
description: >
  Pre/post-work rebase protocol для воркеров agent-flow (issue #2438).
  Использовать ВСЕГДА когда воркер начинает работу над kanban-карточкой:
  шаг 1 — pre_flight rebase после claim, шаг 3 — post_flight rebase перед
  kanban_complete. Закрывает дыру «rebase-protocol неполный» (PR #2351,
  PR #2363 diverged от develop).
---

# Worker Rebase Protocol (issue #2438, ADR-0115 §8)

Воркеры agent-flow стартуют на worktree, который может отставать от
`origin/develop` на десятки коммитов. Без rebase на свежий develop —
PR diverged, merge-gate ловит add/add конфликты, карточка зависает.

**Правило:** воркер делает `git rebase origin/develop` **дважды** за сессию —
в самом начале (pre-work) и перед `kanban_complete` (post-work).

## Workflow

```
[claim] → [pre_flight rebase] → [work] → [scope_check] → [push] → [post_flight rebase] → [report] → [kanban_complete]
```

### Шаг 1: pre-work rebase (после claim, до кода)

```bash
# Перед ЛЮБЫМ кодом — обновить ветку до свежего develop.
bash scripts/agent_flow/worker_pre_flight.sh "$HERMES_KANBAN_TASK" \
    "$(git rev-parse --abbrev-ref HEAD)" "${ISSUE_NUM:-}"
```

**Что делает:**
- `git fetch --no-tags origin refs/heads/develop:refs/remotes/origin/develop` (явный refspec).
- Считает `BEHIND=$(git rev-list --count HEAD..origin/develop)`.
- Если `BEHIND ≤ MAX_BRANCH_BEHIND` (default 30) → exit 0, ничего не делает.
- Если `BEHIND > MAX_BRANCH_BEHIND` → warn в `gh issue comment` (если ISSUE_NUM задан) + auto-rebase.
- Успех → exit 0. Конфликт → exit 1 (воркер должен разрешить, см. ниже).

**Exit codes:**
- `0` — fresh или auto-rebase успешен, продолжай работу.
- `1` — rebase conflict, worktree в `.git/rebase-merge` state. Нужен ручной resolve.
- `2` — usage error (нет task_id / branch / не в worktree).

### Шаг 2: обычная работа

Делай код, пиши тесты. **Перед каждым коммитом** проверяй, что в staged/untracked
нет чужих файлов:

```bash
# Покажет всё, что воркер может закоммитить (staged + unstaged + untracked)
# + committed diff vs origin/develop. С PR_ALLOWED_PREFIXES — блокирует левое.
PR_ALLOWED_PREFIXES="scripts/agent_flow/,docs/adr/" \
    bash scripts/agent_flow/worker_scope_check.sh "$HERMES_KANBAN_TASK" "${ISSUE_NUM:-}"
```

- **Не делай `git add .` / `git commit -a` вслепую** — сначала `git status`,
  потом точечный `git add <file>` только своих файлов.
- Если в `git log origin/develop..HEAD` висят чужие коммиты — ветка грязная,
  пересоздай её с `origin/develop` (см. anti-patterns ниже).

### Шаг 3: post-work rebase + отчёт (перед kanban_complete)

```bash
# post_flight + report в одной команде (рекомендуемый путь).
bash scripts/agent_flow/kanban-report-write.sh "$HERMES_KANBAN_TASK"
```

`kanban-report-write.sh` внутри вызывает `worker_post_flight.sh`:

- Если ветка уже up-to-date с origin/develop → exit 0.
- Если есть drift → auto-rebase + auto-push через `push-via-gh-api.sh`.
- Если rebase conflict → exit 1, отчёт НЕ пишется.

**Exit codes:**
- `0` — post_flight OK + отчёт создан. Можно `kanban_complete`.
- `1` — rebase conflict. Отчёт НЕ создан. Разреши конфликт и повтори.
- `2` — usage error.

### Шаг 4: kanban_complete

Только после успешного `kanban-report-write.sh`:

```bash
kanban_complete --task "$HERMES_KANBAN_TASK" --summary "..." --metadata '{...}'
```

## Что делать при rebase conflict (exit 1)

`worker_pre_flight.sh` или `worker_post_flight.sh` оставили worktree в
`.git/rebase-merge` state и написали инструкцию в issue. Типичные шаги:

```bash
# 1. Посмотри что конфликтует:
git status

# 2. Разреши каждый файл (edit + git add).

# 3. Заверши rebase:
git rebase --continue
# или откатись (потеряешь rebase-эффект, ветка откатится к base):
git rebase --abort

# 4. После continue — push:
bash scripts/agent_flow/push-via-gh-api.sh

# 5. Повтори:
bash scripts/agent_flow/worker_post_flight.sh "$HERMES_KANBAN_TASK" \
    "$(git rev-parse --abbrev-ref HEAD)" "${ISSUE_NUM:-}"
# Если exit 0 → kanban-report-write.sh → kanban_complete.
```

Если конфликт неразрешим (например, чужие наработки переписали твои файлы)
→ `kanban_block kind=dependency reason='drift-pre/post-conflict'`.

## Anti-patterns (что НЕ делать)

- **Не пиши код на устаревшем worktree.** Сначала pre_flight, потом код.
  Ретро PR #2351 (66 коммитов behind) — воркер писал код в параллельной
  вселенной, merge-gate ловил конфликты.
- **Не вызывай `kanban_complete` пока `worker_post_flight.sh` не вернул 0.**
  `kanban-report-write.sh` уже блокирует через step 0, но если вызываешь
  post_flight напрямую — проверь exit code.
- **Не делай `git push --force` без lease.** Используй
  `bash scripts/agent_flow/push-via-gh-api.sh` — он обходит secret policy
  на токенах (ретро t_8abada71).
- **Не игнорируй `SKIP_PRE_FLIGHT=true`** — это opt-out только для retro-карточек,
  не для обычной работы.
- **Не коммить `git add .` вслепую.** Перед коммитом — `worker_scope_check.sh`
  и `git status`. PR #2443 ушёл с 4 чужими файлами (hailo + webxr) именно
  потому, что воркер не смотрел, что у него в worktree.

## Связанные

- ADR-0115 §8 — формализация контракта (`docs/adr/0115-kanban-worker-report-file.md`).
- `scripts/agent_flow/worker_pre_flight.sh` — pre-work auto-rebase.
- `scripts/agent_flow/worker_post_flight.sh` — post-work auto-rebase + push + scope check.
- `scripts/agent_flow/worker_scope_check.sh` — self-check файлов перед push (левое не коммитим).
- `scripts/agent_flow/kanban-report-write.sh` — report + step 0 = post_flight.
- `docs/reports/kanban/README.md` — workflow с rebase.
- Issue #2438 — оригинальная задача (4 примера с drift).
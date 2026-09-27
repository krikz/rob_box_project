# Design note: `scripts/agent_flow/_worker_flight_common.sh`

Companion to ADR-AF-0066. Это **reference implementation**, не SOT —
разработчик может отклониться от него, если есть основания. SOT после review —
`scripts/agent_flow/_worker_flight_common.sh` в develop.

## Цели API

1. Устранить копипасту между `worker_pre_flight.sh` и `worker_post_flight.sh`
   (D-1 в ADR).
2. Подключить существующий dedup-бэкенд (`comment_recently_posted` в
   `hermes_github.sh`) для informational-комментариев воркера, чтобы не
   дублировать алерты при retry (D-2 в ADR).
3. Сохранить существующие exit-коды и поведение:
   - 0 = ok / up-to-date / auto-rebase успешен / skip
   - 1 = rebase conflict / stash pop conflict / rebase-merge state
   - 2 = usage error / not in git worktree
4. Сохранить существующие ENV-контракты:
   - `MAX_BRANCH_BEHIND` (default 30)
   - `GITHUB_REPO` (default krikz/rob_box_project)
   - `GH_CONFIG_DIR` (default /home/builder/.config/gh)
   - `KANBAN_BOARD` (default robbox)
   - `SKIP_PRE_FLIGHT` / `SKIP_POST_FLIGHT` (per-script — НЕ обобщаем,
     см. ADR trade-off #1)
5. Минимальный touch в install.sh — один новый путь в EXPECTED.

## API

```bash
# ──── init ────────────────────────────────────────────────────────
# _wfc_init <flight_name> <task_id> <branch_name> [issue_num]
#
# flight_name: "pre_flight" | "post_flight" | "mid_flight" (будущее)
#              — определяет префикс логов и marker для dedup.
# task_id:    "t_<hex>6+" — формат как раньше.
# branch_name: обычно "z-{agent}/<num>-<slug>"; сейчас не валидируется,
#              остаётся для совместимости с сигнатурой caller'а.
# issue_num:  optional digits; если пусто — комментарии идут в stderr only.
#
# Устанавливает глобальные переменные:
#   WFC_FLIGHT_NAME, WFC_TASK_ID, WFC_BRANCH_NAME, WFC_ISSUE_NUM,
#   WFC_MAX_BEHIND, WFC_GH_REPO, WFC_GH_CONFIG_DIR, WFC_KANBAN_BOARD,
#   WFC_BASE_REF, WFC_DEDUP_WINDOW_SECONDS, WFC_CURRENT_BRANCH,
#   WFC_WORKTREE_DIR.
#
# Валидирует task_id (regex `^t_[a-f0-9]{6,}$`) и branch_name (non-empty).
# Валидирует worktree (git rev-parse --is-inside-work-tree).
#
# Exit: 0 ok / 2 usage error (с echo в stderr).
_wfc_init() { ... }

# ──── logging ─────────────────────────────────────────────────────
# _wfc_log <msg...> — единый логгер, делегирует в `log` если есть
# (как _af_log в lib_agent_flow_common.sh), иначе печатает сам с
# префиксом "[<flight_name> <task_id>]".
_wfc_log() { ... }

# ──── comment ─────────────────────────────────────────────────────
# _wfc_post_comment <body>
#
# Постит body в issue #${WFC_ISSUE_NUM} если задан, иначе только stderr.
#
# Dedup: если комментарий с префиксом "[<flight_name>]" уже постился в
# этот issue за последние WFC_DEDUP_WINDOW_SECONDS (default 300) —
# silent skip (через comment_recently_posted).
#
# Self-id: body содержит "[<flight_name> task=<task_id>]"-маркер в начале
# или в конце (см. ниже "формат body"), что согласуется с ADR-0014
# self-id (хотя emoji 🤖 не добавляем — воркер не cron-процесс с
# таксономией action).
#
# Failure semantics: gh fail / no auth → log warning + return 0 (best-effort,
# как в оригинале).
_wfc_post_comment() { ... }

# ──── git ops ─────────────────────────────────────────────────────
# _wfc_fetch_origin
#
# git fetch origin develop --prune + явный refspec
# (см. ретро t_730ea7b1 — без явного refspec origin/develop может быть stale).
#
# Exit: 0 ok / 1 fetch failed (offline / no remote).
_wfc_fetch_origin() { ... }

# _wfc_drift_count
#
# Заполняет WFC_BEHIND и WFC_AHEAD через git rev-list.
# Если rev-list падает — WFC_BEHIND="?", WFC_AHEAD="?" (caller решает).
# Exit: 0 всегда.
_wfc_drift_count() { ... }

# ──── rebase ──────────────────────────────────────────────────────
# _wfc_stash_and_rebase [timeout_seconds]
#
# Stash локальные изменения (если есть), rebase на WFC_BASE_REF,
# timeout=300s default.
#
# Exit codes (для caller'а):
#   0 = rebase OK (+ stash pop OK если stash был)
#   1 = rebase CONFLICT
#   2 = stash pop CONFLICT (после успешного rebase)
#   3 = rebase TIMEOUT
#
# Side effect: при конфликте оставляет worktree в rebase-merge state
# (caller должен вызвать _wfc_post_comment с инструкцией resolve).
_wfc_stash_and_rebase() { ... }

# _wfc_is_rebase_in_progress
#
# Возвращает 0 если .git/rebase-merge или .git/rebase-apply существуют,
# иначе 1.
#
# Используется только worker_post_flight.sh (pre_flight не должен
# стартовать если предыдущий rebase упал — caller решает что делать).
_wfc_is_rebase_in_progress() { ... }
```

## Формат тела комментария

```text
[<flight_name> task=<task_id>]

<free-form body>
```

Пример (после `_wfc_post_comment "🚨 **rebase CONFLICT**..."`):

```text
[worker_post_flight task=t_27126ab1]

🚨 **rebase CONFLICT** — manual resolve required before kanban_complete.

\`\`\`
behind before: 5 commits
conflicted files (first 20):
  src/foo.py
  scripts/bar.sh
\`\`\`
...
```

**Почему не emoji 🤖:** ADR-0014 self-id требует emoji+agent-prefix для cron-side-
effects (close/reopen/label/assignee). Воркер делает **informational** комментарий,
не привязанный к таксономии action. Достаточно `[worker_<flight>]`-префикса
(grep'абельно как `[worker_pre_flight]` / `[worker_post_flight]`), который
попадает в marker'е `comment_recently_posted`.

## Сценарий pre_flight (полный переписанный скрипт)

```bash
#!/usr/bin/env bash
set -uo pipefail
_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
. "$_HERE/_worker_flight_common.sh"

_wfc_init pre_flight "$@"
[ "${SKIP_PRE_FLIGHT:-}" = "true" ] && { _wfc_log "SKIP via SKIP_PRE_FLIGHT=true"; exit 0; }

_wfc_fetch_origin || {
    _wfc_post_comment "⚠️ [worker_pre_flight] git fetch origin develop FAILED.

Воркер не смог проверить drift (offline / no remote / 401).
Работа продолжена, но есть риск stale worktree."
    exit 0
}
_wfc_drift_count

_wfc_log "drift: ahead=$WFC_AHEAD behind=$WFC_BEHIND (max=$WFC_MAX_BEHIND) branch=$WFC_CURRENT_BRANCH"

# AHEAD > 0 → leftover warning (issue #2438, PR #2443)
if [ "$WFC_AHEAD" != "?" ] && [ "$WFC_AHEAD" -gt 0 ] 2>/dev/null; then
    _wfc_log "WARN: branch has $WFC_AHEAD commit(s) ahead of develop — possible leftover"
    git log --oneline "origin/develop..${WFC_CURRENT_BRANCH}" 2>/dev/null | sed 's/^/    commit: /' >&2 || true
fi

if [ "$WFC_BEHIND" = "?" ]; then
    _wfc_log "WARN: could not count BEHIND; skipping rebase"
    exit 0
fi
if [ "$WFC_BEHIND" -le 0 ] 2>/dev/null; then
    _wfc_log "OK: up-to-date"; exit 0
fi
if [ "$WFC_BEHIND" -le "$WFC_MAX_BEHIND" ] 2>/dev/null; then
    _wfc_log "drift $WFC_BEHIND ≤ MAX=$WFC_MAX_BEHIND; no rebase"; exit 0
fi

_wfc_post_comment "⚠️ [worker_pre_flight] branch **${WFC_CURRENT_BRANCH}** is **${WFC_BEHIND} commits behind** origin/develop (max $WFC_MAX_BEHIND).

Auto-rebase started. Если conflict → manual resolve требуется."

_wfc_stash_and_rebase 300
rc=$?
case $rc in
    0)  _wfc_post_comment "✅ [worker_pre_flight] auto-rebase OK."; exit 0 ;;
    1)
        CONFLICT_FILES="$(git diff --name-only --diff-filter=U 2>/dev/null | head -20 || true)"
        _wfc_post_comment "🚨 [worker_pre_flight] **rebase CONFLICT** — manual resolve required.

\`\`\`
branch: $WFC_CURRENT_BRANCH
behind before: $WFC_BEHIND commits
conflicted files (first 20):
$CONFLICT_FILES
\`\`\`

**Resolution:**
\`\`\`bash
git status
# resolve + git add
git rebase --continue
# or: git rebase --abort
bash scripts/agent_flow/push-via-gh-api.sh
\`\`\`"
        exit 1 ;;
    2)  _wfc_post_comment "⚠️ [worker_pre_flight] **stash pop conflict** after rebase."; exit 1 ;;
    3)  _wfc_post_comment "⚠️ [worker_pre_flight] rebase TIMEOUT (>300s)."; exit 1 ;;
esac
```

## Сценарий post_flight (полный переписанный скрипт)

```bash
#!/usr/bin/env bash
set -uo pipefail
_HERE="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
. "$_HERE/_worker_flight_common.sh"

_wfc_init post_flight "$@"
[ "${SKIP_POST_FLIGHT:-}" = "true" ] && { _wfc_log "SKIP via SKIP_POST_FLIGHT=true"; exit 0; }

# Pre-flight: refuse if worktree in rebase-merge state
if _wfc_is_rebase_in_progress; then
    _wfc_post_comment "🚨 [worker_post_flight] worktree **${WFC_CURRENT_BRANCH}** in **rebase-merge state**.

\`\`\`bash
git status
git rebase --continue   # OR git rebase --abort
\`\`\`"
    exit 1
fi

_wfc_fetch_origin || {
    _wfc_post_comment "⚠️ [worker_post_flight] git fetch origin develop FAILED.

Воркер не смог проверить drift. Работа продолжена, но есть риск stale PR."
    exit 0
}

# Scope self-check (PR #2443) — отдельный helper, не часть протокола
if [ -x "$_HERE/worker_scope_check.sh" ]; then
    if ! bash "$_HERE/worker_scope_check.sh" "$WFC_TASK_ID" "${WFC_ISSUE_NUM:-}" >&2; then
        _wfc_log "scope check FAILED; resolve before kanban_complete"
        exit 1
    fi
fi

_wfc_drift_count

_wfc_log "drift: ahead=$WFC_AHEAD behind=$WFC_BEHIND branch=$WFC_CURRENT_BRANCH"

if [ "$WFC_BEHIND" = "?" ]; then
    _wfc_log "WARN: could not count BEHIND; skipping rebase"; exit 0
fi
if [ "$WFC_BEHIND" -le 0 ] 2>/dev/null; then
    _wfc_log "OK: up-to-date"; exit 0
fi

_wfc_post_comment "⚠️ [worker_post_flight] branch **${WFC_CURRENT_BRANCH}** is **${WFC_BEHIND} commits behind** origin/develop.

Auto-rebase before kanban_complete. Если conflict → блокирую карточку."

_wfc_stash_and_rebase 300
rc=$?
case $rc in
    0)
        # push-via-gh-api.sh обходит secret policy (ретро t_8abada71)
        if [ -x "$_HERE/push-via-gh-api.sh" ]; then
            bash "$_HERE/push-via-gh-api.sh" >/dev/null 2>&1 \
                && _wfc_log "pushed rebased branch via push-via-gh-api.sh" \
                || _wfc_log "WARN: push-via-gh-api.sh failed"
        fi
        _wfc_post_comment "✅ [worker_post_flight] rebase **OK** — kanban_complete allowed."
        exit 0 ;;
    1)
        CONFLICT_FILES="$(git diff --name-only --diff-filter=U 2>/dev/null | head -20 || true)"
        _wfc_post_comment "🚨 [worker_post_flight] **rebase CONFLICT** — manual resolve required before kanban_complete.

\`\`\`
branch: $WFC_CURRENT_BRANCH
behind before: $WFC_BEHIND commits
conflicted files (first 20):
$CONFLICT_FILES
\`\`\`

**Resolution:**
\`\`\`bash
git status
git rebase --continue   # or: git rebase --abort
bash scripts/agent_flow/push-via-gh-api.sh
bash scripts/agent_flow/worker_post_flight.sh $WFC_TASK_ID $WFC_CURRENT_BRANCH $WFC_ISSUE_NUM
\`\`\`

**КРИТИЧНО:** kanban_complete БЛОКИРОВАН до успешного post_flight."
        exit 1 ;;
    2)  _wfc_post_comment "⚠️ [worker_post_flight] **stash pop conflict**."; exit 1 ;;
    3)  _wfc_post_comment "⚠️ [worker_post_flight] rebase TIMEOUT (>300s)."; exit 1 ;;
esac
```

## Сравнение LOC

| Файл                                    | Сейчас | После (target) | Δ |
|-----------------------------------------|--------|------------------|---|
| `worker_pre_flight.sh`                  | 267    | ~70              | -197 |
| `worker_post_flight.sh`                 | 280    | ~85              | -195 |
| `_worker_flight_common.sh` (новый)      | 0      | ~120             | +120 |
| `install.sh` (EXPECTED list)            | n      | n+1              | +1 |
| **Net**                                 | 547    | 276              | **-271** |

Это консервативная оценка — реальный gain выше, потому что новый код имеет
комментарии и doc-блоки, а старый — копипаст без объяснения.

## Тесты переносить не нужно

`tests/test_worker_pre_flight.sh` и `tests/test_worker_post_flight.sh` уже
изолированы через `bash "$TARGET"`, поэтому они продолжат работать с
обновлёнными скриптами без изменений (8/8 сценариев каждый — единственные
assert'ы это exit code + grep на "usage:" / "branch required", оба
сохраняются). Если переписать тесты на _wfc_* — плюс, но не обязательно для
этого рефакторинга.

## Что НЕ делаем

- Не объединяем pre/post в один `worker_flight.sh` (KISS — два скрипта
  вызываются из разных точек протокола воркера).
- Не выносим в `lib_agent_flow_common.sh` (см. ADR trade-off #1).
- Не используем `post_whoami_comment` (action-таксономия не подходит).
- Не делаем emoji-маркер как в cron-процессах (воркер ≠ cron).
- Не рефакторим `push-via-gh-api.sh`, `worker_scope_check.sh`,
  `comment_recently_posted` — это **за пределами** карточки.

## Открытые вопросы для developer'а (будут в issue body)

1. Реальный размер `_worker_flight_common.sh` после кода vs design-doc — есть
   ли смысл разделить на 2 файла (init/log/comment vs git-ops)? Пока
   предполагаем один файл.
2. Если `push-via-gh-api.sh` существует только в SOT (а не на хосте через
   install.sh) — нужно ли его тоже в EXPECTED? Проверить grep'ом по install.sh.
3. Кто тестирует multi-script consistency (pre+post не разъехались после
   правки)? Рекомендация — добавить `tests/test_worker_flight_consistency.sh`,
   но это +1 файл; оценить после MVP.
#!/usr/bin/env bash
# ============================================================================
# worker_post_flight.sh — воркер-helper: post-work rebase ПЕРЕД kanban_complete.
#
# Контекст: воркер сделал код, закоммитил, написал тесты. К моменту
# kanban_complete прошло N минут, develop уехал вперёд. PR diverged,
# merge-gate ловит add/add конфликты (ретро PR #2363). Issue #2438:
# «rebase-protocol неполный» — нет post-work rebase.
#
# Этот скрипт вызывается воркером в конце сессии (после push, ПЕРЕД
# kanban_complete) — fetch, считает BEHIND, делает auto-rebase. Если
# rebase падает с конфликтом — пишет инструкцию в task_comments и
# возвращает non-zero exit. Воркер НЕ должен вызывать kanban_complete.
#
# Контракт (ADR-0077 §3.3 расширение, issue #2438, фикс #2478):
#   - Принимает task_id, branch_name (argv $2) и опционально issue_num (argv $3).
#   - branch_name ОБЯЗАН совпадать с `git rev-parse --abbrev-ref HEAD`,
#     иначе exit 2 (fail-fast защита от silent ignore, ADR-0018).
#   - Делает `git fetch origin $BASE_REF --prune`.
#   - Считает BEHIND=$(git rev-list --count HEAD..$BASE_REF).
#   - Если BEHIND == 0 → exit 0 (no-op).
#   - Если BEHIND > 0 → auto-rebase $BASE_REF. Успех → exit 0 (готов к kanban_complete).
#     Конфликт → пишет инструкцию + exit 1 (воркер должен kanban_block).
#
# Usage:
#   bash scripts/agent_flow/worker_post_flight.sh <task_id> <branch> [ISSUE_NUM]
#
# Env:
#   MAX_BRANCH_BEHIND (default 30) — порог, выше которого логируем warn
#   BASE_REF          (default origin/develop) — реально используется для fetch/rebase
#                                         (до #2478 был dead variable)
#   GITHUB_REPO       (default krikz/rob_box_project) — для gh issue comment
#   GH_CONFIG_DIR     (default /home/builder/.config/gh) — для gh auth
#   KANBAN_BOARD      (default robbox) — для kanban-tools
#   SKIP_POST_FLIGHT  если "true" — exit 0 без действий
#
# Exit codes:
#   0 — success (branch up-to-date или auto-rebase успешен)
#   1 — rebase conflict (worktree в rebase-merge state, требует ручной resolve)
#   2 — usage error (нет task_id / branch_name, branch_name≠HEAD, не в git worktree)
#
# SOT: <repo>/scripts/agent_flow/worker_post_flight.sh
# Тест: bash scripts/agent_flow/tests/test_worker_post_flight.sh
# ============================================================================
set -uo pipefail

# ---- args -----------------------------------------------------------------
TASK_ID="${1:-}"
BRANCH_NAME="${2:-}"
ISSUE_NUM="${3:-}"

if [ -z "$TASK_ID" ] || ! printf '%s' "$TASK_ID" | grep -qE '^t_[a-f0-9]{6,}$'; then
    echo "usage: $0 <task_id> <branch> [issue_num]" >&2
    echo "       task_id format: t_<hex>6+" >&2
    echo "example: $0 t_be9fc608 'z-{agent}/2438-...'" >&2
    exit 2
fi
if [ -z "$BRANCH_NAME" ]; then
    echo "usage: $0 <task_id> <branch> [issue_num]  (branch required)" >&2
    exit 2
fi

# ---- env / paths ----------------------------------------------------------
MAX_BEHIND="${MAX_BRANCH_BEHIND:-30}"
GITHUB_REPO="${GITHUB_REPO:-krikz/rob_box_project}"
GH_CONFIG_DIR="${GH_CONFIG_DIR:-/home/builder/.config/gh}"
KANBAN_BOARD="${KANBAN_BOARD:-robbox}"
BASE_REF="origin/develop"

if [ "${SKIP_POST_FLIGHT:-}" = "true" ]; then
    echo "[worker_post_flight] SKIP via SKIP_POST_FLIGHT=true"
    exit 0
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[worker_post_flight] ERROR: not inside a git worktree" >&2
    exit 2
fi

CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
WORKTREE_DIR="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

# ---- helpers --------------------------------------------------------------
# log() определяется ДО fail-fast: иначе "branch mismatch" check (issue #2478)
# не сможет залогировать сообщение и молча fail-fast'нет без следа.
log() {
    printf '[worker_post_flight %s] %s\n' "$TASK_ID" "$*" >&2
}

# Fail-fast: branch_name (argv $2) обязан совпадать с реальной веткой.
# До #2478 BRANCH_NAME читался, валидировался на пустоту, но игнорировался —
# воркеры могли передавать мусор и скрипт молча делал rebase на «не той» ветке.
# ADR-0018 (honest naming): задокументировано = работает.
if [ -n "$BRANCH_NAME" ] && [ "$BRANCH_NAME" != "$CURRENT_BRANCH" ]; then
    log "ERROR: branch mismatch — passed '$BRANCH_NAME', actual HEAD is '$CURRENT_BRANCH'"
    log "  Hint: воркеры должны передавать текущую ветку или не передавать ничего."
    log "  Refuse to operate on a different branch than caller claims (ADR-0018 honest naming, issue #2478)."
    exit 2
fi

post_comment() {
    local body="$1"
    if [ -z "$ISSUE_NUM" ]; then
        log "comment (no ISSUE_NUM, stderr-only): $body"
        return 0
    fi
    if ! command -v gh >/dev/null 2>&1; then
        log "gh not in PATH, falling back to stderr: $body"
        return 0
    fi
    local tmp
    tmp="$(mktemp)"
    printf '%s\n' "$body" > "$tmp"
    if GH_CONFIG_DIR="$GH_CONFIG_DIR" gh issue comment "$ISSUE_NUM" \
            --repo "$GITHUB_REPO" --body-file "$tmp" >/dev/null 2>&1; then
        log "commented to issue #$ISSUE_NUM"
    else
        log "gh comment failed (issue #$ISSUE_NUM), stderr-only: $body"
    fi
    rm -f "$tmp"
    return 0
}

# ---- step 1: pre-flight sanity check -------------------------------------
# Если worktree в состоянии rebase-merge (предыдущий rebase упал) — не делаем
# ничего, воркер должен сначала завершить/abort rebase.
if [ -d ".git/rebase-merge" ] || [ -d ".git/rebase-apply" ]; then
    log "WARN: worktree is in rebase-merge state; refusing post-flight (resolve rebase first)"
    post_comment "🚨 [agent:devops] worker_post_flight: worktree **$CURRENT_BRANCH** is in **rebase-merge state** from previous rebase.

\`\`\`bash
git status           # shows 'interactive rebase in progress'
git rebase --continue   # OR git rebase --abort
\`\`\`

Post-flight cannot proceed until rebase is resolved.
_worker_post_flight task=$TASK_ID_"
    exit 1
fi

# ---- step 2: fetch --------------------------------------------------------
# Разделяем BASE_REF на remote и ref (default: origin/develop → origin=origin, ref=develop).
_REMOTE="${BASE_REF%%/*}"
_REMOTE_REF="${BASE_REF#*/}"
log "fetching $BASE_REF in $WORKTREE_DIR"
if ! git fetch "$_REMOTE" "$_REMOTE_REF" --prune >/dev/null 2>&1; then
    log "WARN: git fetch $_REMOTE $_REMOTE_REF failed (offline? no remote?)"
    # Не блокируем воркера — fetch fail это не его вина.
    post_comment "⚠️ [agent:devops] worker_post_flight: git fetch $_REMOTE $_REMOTE_REF FAILED.

Воркер не смог проверить drift. Работа продолжена, но есть риск stale PR.
_worker_post_flight task=$TASK_ID branch=$CURRENT_BRANCH_"
    exit 0
fi

# Явный fetch refspec (ретро t_730ea7b1)
git fetch --no-tags "$_REMOTE" "refs/heads/${_REMOTE_REF}:refs/remotes/${BASE_REF}" >/dev/null 2>&1 || true
unset _REMOTE _REMOTE_REF

# ---- step 2.5: scope self-check (issue #2438, PR #2443) -------------------
# Перед rebase/push — самопроверка файлов: working tree (staged + unstaged +
# untracked) + committed diff vs origin/develop сверяются с
# PR_ALLOWED_PREFIXES / PR_ALLOWED_GLOBS. Если воркер подхватил чужие файлы
# (другой worktree / чужая задача) — блокируем карточку, пока не разберёт.
# Opt-out: SKIP_SCOPE_CHECK=true (см. worker_scope_check.sh).
_SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
if [ -x "${_SELF_DIR}/worker_scope_check.sh" ]; then
    if ! bash "${_SELF_DIR}/worker_scope_check.sh" "$TASK_ID" "${ISSUE_NUM:-}" >&2; then
        log "scope check FAILED: out-of-scope files present; resolve before kanban_complete"
        exit 1
    fi
fi
unset _SELF_DIR

# ---- step 3: behind count ------------------------------------------------
BEHIND="$(git rev-list --count "${CURRENT_BRANCH}..${BASE_REF}" 2>/dev/null || echo "?")"
AHEAD="$(git rev-list --count "${BASE_REF}..${CURRENT_BRANCH}" 2>/dev/null || echo "?")"

log "drift: ahead=$AHEAD behind=$BEHIND (max=$MAX_BEHIND) branch=$CURRENT_BRANCH"

if [ "$BEHIND" = "?" ]; then
    log "WARN: could not count BEHIND; skipping rebase"
    exit 0
fi

# ---- step 4: up-to-date → no-op ------------------------------------------
if [ "$BEHIND" -le 0 ] 2>/dev/null; then
    log "OK: branch is up-to-date with $BASE_REF (ahead=$AHEAD, behind=0)"
    exit 0
fi

# ---- step 5: drift detected → auto-rebase --------------------------------
log "drift $BEHIND > 0 — auto-rebasing $CURRENT_BRANCH onto $BASE_REF"
post_comment "⚠️ [agent:devops] worker_post_flight: branch **$CURRENT_BRANCH** is **$BEHIND commits behind** $BASE_REF.

Auto-rebase before kanban_complete. Если conflict → блокирую карточку.

\`\`\`
branch: $CURRENT_BRANCH
behind: $BEHIND
\`\`\`
_worker_post_flight task=$TASK_ID_"

# stash uncommitted changes (если воркер забыл закоммитить).
STASH_NAME="post_flight_${TASK_ID}_$$"
STASHED=0
if ! git diff --quiet 2>/dev/null || ! git diff --cached --quiet 2>/dev/null; then
    if git stash push -u -m "$STASH_NAME" >/dev/null 2>&1; then
        STASHED=1
        log "stashed uncommitted changes as $STASH_NAME"
    else
        log "WARN: git stash failed; proceeding rebase (expect conflicts)"
    fi
fi

REBASE_OUT="$(mktemp)"
if timeout 300 git rebase "$BASE_REF" >"$REBASE_OUT" 2>&1; then
    rm -f "$REBASE_OUT"
    if [ "$STASHED" -eq 1 ]; then
        if git stash pop >/dev/null 2>&1; then
            log "stash popped successfully"
        else
            log "WARN: stash pop conflict"
            post_comment "⚠️ [agent:devops] worker_post_flight: **stash pop conflict** after rebase.

\`\`\`bash
git stash list    # shows $STASH_NAME
git status        # shows conflicts
\`\`\`
_worker_post_flight task=$TASK_ID_"
            exit 1
        fi
    fi

    NEW_BEHIND="$(git rev-list --count "${CURRENT_BRANCH}..${BASE_REF}" 2>/dev/null || echo "0")"
    REBASE_COMMITS=$((BEHIND - NEW_BEHIND))
    log "REBASE OK: rebased $REBASE_COMMITS commits (was behind=$BEHIND, now=$NEW_BEHIND)"

    # Push результат rebase (force-with-lease) — push-via-gh-api.sh обходит
    # secret policy на токенах (ретро t_8abada71).
    if [ -x "./scripts/agent_flow/push-via-gh-api.sh" ]; then
        log "pushing rebased branch via push-via-gh-api.sh"
        if ! bash ./scripts/agent_flow/push-via-gh-api.sh >/dev/null 2>&1; then
            log "WARN: push-via-gh-api.sh failed; воркер должен сделать push сам"
        fi
    fi

    post_comment "✅ [agent:devops] worker_post_flight: rebase **OK**, $REBASE_COMMITS commits replayed.

\`\`\`
was behind: $BEHIND
now behind: $NEW_BEHIND
branch: $CURRENT_BRANCH
\`\`\`
_worker_post_flight task=$TASK_ID — kanban_complete allowed_"
    exit 0
fi

# ---- step 6: rebase conflict --------------------------------------------
rm -f "$REBASE_OUT"
log "REBASE FAILED: conflict in $CURRENT_BRANCH"
CONFLICT_FILES="$(git diff --name-only --diff-filter=U 2>/dev/null | head -20 || true)"

post_comment "🚨 [agent:devops] worker_post_flight: **rebase CONFLICT** — manual resolve required before kanban_complete.

\`\`\`
branch: $CURRENT_BRANCH
behind before: $BEHIND commits
conflicted files (first 20):
$CONFLICT_FILES
\`\`\`

**Resolution steps for worker:**
\`\`\`bash
# В worktree сейчас rebase-merge state, ничего не коммить.

# 1. Посмотри конфликты:
git status

# 2. Разреши каждый файл (edit + git add).

# 3. Заверши rebase:
git rebase --continue
# или откатись (НЕ рекомендуется — потеряешь чужие наработки):
git rebase --abort

# 4. После continue — push:
bash scripts/agent_flow/push-via-gh-api.sh

# 5. Затем снова:
bash scripts/agent_flow/worker_post_flight.sh $TASK_ID $CURRENT_BRANCH $ISSUE_NUM

# 6. Если exit 0 → kanban_complete.
#    Если exit 1 → повтори resolve.
\`\`\`

**КРИТИЧНО:** карточка остаётся в running, kanban_complete БЛОКИРОВАН до успешного post_flight.
_worker_post_flight task=$TASK_ID outcome=BLOCKED_"

# Оставляем worktree в rebase-merge state — воркер должен resolve.
exit 1
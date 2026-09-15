#!/usr/bin/env bash
# ============================================================================
# worker_pre_flight.sh — воркер-helper: pre-work rebase ПЕРЕД началом кода.
#
# Контекст: воркеры часто подхватывают карточку на worktree, который отстал
# от origin/develop на десятки коммитов (PR #2351 — 66 коммитов behind;
# PR #2363 — add/add конфликт). PR уходит CONFLICTING, merge-gate ловит
# конфликты, карточка зависает. Issue #2438: «rebase-protocol неполный».
#
# Этот скрипт вызывается воркером в самом начале сессии (после claim и
# `cd` в worktree) — fetch, считает BEHIND, делает auto-rebase если drift
# больше WARN_BRANCH_BEHIND. Логирует результат в task_comments (через
# `gh issue comment` если задан ISSUE_NUM, иначе только stderr).
#
# Контракт (ADR-0077 §3.3 расширение, issue #2438):
#   - Принимает task_id и branch_name.
#   - Делает `git fetch origin develop --prune`.
#   - Считает BEHIND=$(git rev-list --count HEAD..origin/develop).
#   - Если BEHIND > MAX_BRANCH_BEHIND (default 30) — пишет warn в task_comments,
#     затем ДЕЛАЕТ auto-rebase origin/develop.
#   - Если rebase падает с конфликтом — пишет инструкцию в task_comments
#     и возвращает non-zero exit (воркер должен вызвать kanban_block).
#   - Иначе возвращает 0.
#
# Usage:
#   bash scripts/agent_flow/worker_pre_flight.sh <task_id> <branch> [ISSUE_NUM]
#
# Env:
#   MAX_BRANCH_BEHIND (default 30) — порог drift, выше которого warn+auto-rebase
#   GITHUB_REPO       (default krikz/rob_box_project) — для gh issue comment
#   GH_CONFIG_DIR     (default /home/builder/.config/gh) — для gh auth
#   KANBAN_BOARD      (default robbox) — для kanban-tools
#   SKIP_PRE_FLIGHT   если "true" — exit 0 без действий
#
# Exit codes:
#   0 — success (fresh или auto-rebase успешен)
#   1 — rebase conflict (worktree в состоянии rebase-merge, нужен ручной resolve)
#   2 — usage error (нет task_id / branch_name, или не в git worktree)
#
# SOT: <repo>/scripts/agent_flow/worker_pre_flight.sh
# На хост раскладывает install.sh EXPECTED → drift-detect контролирует.
#
# Тест: bash scripts/agent_flow/tests/test_worker_pre_flight.sh
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

if [ "${SKIP_PRE_FLIGHT:-}" = "true" ]; then
    echo "[worker_pre_flight] SKIP via SKIP_PRE_FLIGHT=true"
    exit 0
fi

# ---- worktree guard -------------------------------------------------------
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[worker_pre_flight] ERROR: not inside a git worktree" >&2
    exit 2
fi

CURRENT_BRANCH="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
WORKTREE_DIR="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"

# ---- helpers --------------------------------------------------------------
log() {
    printf '[worker_pre_flight %s] %s\n' "$TASK_ID" "$*" >&2
}

post_comment() {
    # post_comment <body> — пишет комментарий в issue (если ISSUE_NUM задан),
    # иначе только в stderr. Идемпотентный best-effort: gh может быть не authed,
    # тогда пишем только в stderr — это нормально (воркер увидит в логах).
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

# ---- step 1: fetch ---------------------------------------------------------
log "fetching origin/develop in $WORKTREE_DIR"
if ! git fetch origin develop --prune >/dev/null 2>&1; then
    log "WARN: git fetch origin develop failed (offline? no remote?)"
    # Если fetch не работает — мы не можем честно оценить drift.
    # Лучше exit 0 (не блокируем воркера), но пишем warn.
    post_comment "⚠️ [agent:devops] worker_pre_flight: git fetch origin develop FAILED.

Воркер не смог проверить drift (offline / no remote / 401).
Работа продолжена, но есть риск stale worktree.

_worker_pre_flight task=$TASK_ID branch=$CURRENT_BRANCH_"
    exit 0
fi

# ---- step 2: behind count --------------------------------------------------
# fetch с явным refspec — иначе origin/develop может быть stale (ретро t_730ea7b1)
git fetch --no-tags origin "refs/heads/develop:refs/remotes/origin/develop" >/dev/null 2>&1 || true

BEHIND="$(git rev-list --count "${CURRENT_BRANCH}..origin/develop" 2>/dev/null || echo "?")"
AHEAD="$(git rev-list --count "origin/develop..${CURRENT_BRANCH}" 2>/dev/null || echo "?")"

log "drift: ahead=$AHEAD behind=$BEHIND (max=$MAX_BEHIND) branch=$CURRENT_BRANCH"

# ---- step 2.5: leftover-commits warning (issue #2438, PR #2443) -----------
# AHEAD > 0 на старте — на ветке уже висят коммиты, которых нет в develop.
# Для свежего claim это почти наверняка чужой мусор (см. PR #2443: hailo +
# webxr коммиты от прошлых воркеров). Не блокируем (возможен legit resume
# своей ветки), но показываем список — воркер должен решить, его это или нет.
if [ "$AHEAD" != "?" ] && [ "$AHEAD" -gt 0 ] 2>/dev/null; then
    log "WARN: branch already has $AHEAD commit(s) not in origin/develop — possible leftover from another task"
    git log --oneline "origin/develop..${CURRENT_BRANCH}" 2>/dev/null | sed 's/^/    commit: /' >&2 || true
    log "If these are NOT your commits — recreate branch from fresh origin/develop (see worker-rebase-protocol skill)."
fi

if [ "$BEHIND" = "?" ]; then
    log "WARN: could not count BEHIND (git rev-list failed); skipping rebase"
    exit 0
fi

# ---- step 3: no-op if fresh -----------------------------------------------
if [ "$BEHIND" -le 0 ] 2>/dev/null; then
    log "OK: branch is up-to-date with origin/develop (ahead=$AHEAD, behind=0)"
    exit 0
fi

# ---- step 4: drift detected -----------------------------------------------
# Если drift в пределах MAX_BEHIND — это нормальное явление (воркеры стартуют
# на develop-N старее свежего origin/develop на пару коммитов).
# Решаем rebase только если BEHIND > MAX_BEHIND.
if [ "$BEHIND" -le "$MAX_BEHIND" ] 2>/dev/null; then
    log "drift $BEHIND ≤ MAX_BRANCH_BEHIND=$MAX_BEHIND; no rebase needed"
    exit 0
fi

# ---- step 5: heavy drift — warn + auto-rebase ----------------------------
log "DRIFT $BEHIND > $MAX_BEHIND — auto-rebasing $CURRENT_BRANCH onto origin/develop"
post_comment "⚠️ [agent:devops] worker_pre_flight: branch **$CURRENT_BRANCH** is **$BEHIND commits behind** origin/develop (max $MAX_BEHIND).

Auto-rebase started. Если rebase упадёт с конфликтом — ручной resolve требуется.

\`\`\`
branch: $CURRENT_BRANCH
behind: $BEHIND
max: $MAX_BRANCH_BEHIND
\`\`\`
_worker_pre_flight task=$TASK_ID_"

# stash локальные изменения, чтобы rebase был чистым.
STASH_NAME="pre_flight_${TASK_ID}_$$"
STASHED=0
if ! git diff --quiet 2>/dev/null || ! git diff --cached --quiet 2>/dev/null; then
    if git stash push -u -m "$STASH_NAME" >/dev/null 2>&1; then
        STASHED=1
        log "stashed local changes as $STASH_NAME"
    else
        log "WARN: git stash failed; proceeding rebase without stash (may conflict)"
    fi
fi

# rebase с таймаутом (нет, git rebase не имеет --timeout, но мы можем abort через timeout cmd).
REBASE_OUT="$(mktemp)"
if timeout 300 git rebase origin/develop >"$REBASE_OUT" 2>&1; then
    rm -f "$REBASE_OUT"
    if [ "$STASHED" -eq 1 ]; then
        if git stash pop >/dev/null 2>&1; then
            log "stash popped successfully"
        else
            log "WARN: stash pop conflict; state may need manual resolve"
            post_comment "⚠️ [agent:devops] worker_pre_flight: **stash pop conflict** after auto-rebase.

\`\`\`
git stash list   # shows $STASH_NAME
git status       # shows conflicted files
\`\`\`

Воркер: разреши конфликты, затем \`git stash drop\`.
_worker_pre_flight task=$TASK_ID_"
            exit 1
        fi
    fi
    NEW_BEHIND="$(git rev-list --count "${CURRENT_BRANCH}..origin/develop" 2>/dev/null || echo "?")"
    log "REBASE OK: new behind=$NEW_BEHIND (was $BEHIND)"
    post_comment "✅ [agent:devops] worker_pre_flight: auto-rebase **OK**.

\`\`\`
was behind: $BEHIND commits
now behind: $NEW_BEHIND commits
branch: $CURRENT_BRANCH
\`\`\`
_worker_pre_flight task=$TASK_ID_"
    exit 0
fi

# ---- step 6: rebase conflict ----------------------------------------------
rm -f "$REBASE_OUT"
log "REBASE FAILED: conflict in $CURRENT_BRANCH"
CONFLICT_FILES="$(git diff --name-only --diff-filter=U 2>/dev/null | head -20 || true)"
post_comment "🚨 [agent:devops] worker_pre_flight: **rebase CONFLICT** — manual resolve required.

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
# или откатись:
git rebase --abort

# 4. После continue — force-push:
bash scripts/agent_flow/push-via-gh-api.sh

# 5. Затем продолжи работу и заверши карточку.
\`\`\`

Если не получается разрешить → \`kanban_block reason='drift-pre-conflict'\`.
_worker_pre_flight task=$TASK_ID_"

# Оставляем worktree в rebase-merge state — воркер должен resolve.
# Не делаем автоматический abort, чтобы воркер мог посмотреть конфликт.
exit 1
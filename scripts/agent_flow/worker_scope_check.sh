#!/usr/bin/env bash
# ============================================================================
# worker_scope_check.sh — воркер-helper: самопроверка файлов ПЕРЕД push и
# kanban_complete. Ловит «левые» файлы, которые воркер подхватил с чужого
# worktree или закоммитил по ошибке, и блокирует карточку до их разбора.
#
# Контекст (issue #2438, PR #2443): PR «rebase-protocol» ушёл с 4 чужими
# файлами — docker/vision/vision-hailo/* (hailo, ADR-0089) и
# src/rob_box_quest/webxr_client/tests/voice_capture_*.test.ts (Quest,
# ADR-0027). Воркер писал поверх ветки, на которой уже висели чужие
# коммиты, и не смотрел `git status` перед push. Этот скрипт — третий
# рубеж после freshness (validate_branch_freshness.sh) и post-PR scope
# (validate_pr_scope.sh): он проверяет ДО push/kanban_complete, пока
# карточку ещё можно починить дёшево.
#
# Что проверяет:
#   - working tree:  staged + unstaged + untracked (то, что воркер может
#     закоммитить прямо сейчас);
#   - committed diff: `git diff --name-only BASE_REF...HEAD` (то, что
#     пойдёт в PR).
#   Файлы из обоих наборов сверяются с PR_ALLOWED_PREFIXES / PR_ALLOWED_GLOBS.
#
# Usage (воркер, из worktree, перед push и перед kanban_complete):
#   PR_ALLOWED_PREFIXES="scripts/agent_flow/,docs/adr/" \
#       bash scripts/agent_flow/worker_scope_check.sh "$TASK_ID" [ISSUE_NUM]
#
# Env (совместимо с validate_pr_scope.sh):
#   PR_ALLOWED_PREFIXES — comma-separated list of allowed path prefixes.
#   PR_ALLOWED_GLOBS     — comma-separated list of fnmatch-style globs.
#   SKIP_SCOPE_CHECK     — "true" → exit 0 без проверки (opt-out).
#   BASE_REF             — default origin/develop (для committed diff).
#   MAX_OUT_OF_SCOPE     — default 10 (defensive INFO-mode cap).
#
# Режимы:
#   - Без PR_ALLOWED_PREFIXES/GLOBS → INFO-режим: печатает список файлов
#     (воркер ВИДИТ что у него в working tree), exit 0 (но exit 1 если
#     файлов > MAX_OUT_OF_SCOPE — почти наверняка drift).
#   - С prefixes/globs → блокирующий: exit 1 если есть out-of-scope файл.
#
# Exit codes:
#   0 — OK (нет файлов / все в scope / skipped / INFO-режим в пределах cap)
#   1 — есть out-of-scope файлы (блокирующий fail); список в stderr
#   2 — usage error (нет task_id / не в git worktree)
#
# SOT: <repo>/scripts/agent_flow/worker_scope_check.sh
# Тест: bash scripts/agent_flow/tests/test_worker_scope_check.sh
# ============================================================================
set -uo pipefail

# ---- args -----------------------------------------------------------------
TASK_ID="${1:-}"
ISSUE_NUM="${2:-}"

if [ -z "$TASK_ID" ] || ! printf '%s' "$TASK_ID" | grep -qE '^t_[a-f0-9]{6,}$'; then
    echo "usage: $0 <task_id> [issue_num]" >&2
    echo "       task_id format: t_<hex>6+" >&2
    echo "example: $0 t_be9fc608 2349" >&2
    exit 2
fi

# ---- env / paths ----------------------------------------------------------
BASE_REF="${BASE_REF:-origin/develop}"
MAX_OUT_OF_SCOPE="${MAX_OUT_OF_SCOPE:-10}"

log() {
    printf '[worker_scope_check %s] %s\n' "$TASK_ID" "$*" >&2
}

if [ "${SKIP_SCOPE_CHECK:-}" = "true" ]; then
    log "SKIP via SKIP_SCOPE_CHECK=true"
    exit 0
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    log "ERROR: not inside a git worktree" >&2
    exit 2
fi

# ---- collect files ---------------------------------------------------------
# 1) Working tree: staged + unstaged + untracked. Используем --name-only
#    диффы + ls-files (без porcelain-парсинга — чистые пути, без кавычек).
#    Renames приходят одним путём (--name-only отдаёт new path) — этого
#    достаточно для scope-проверки.
STATUS_FILES="$(
    {
        git diff --name-only 2>/dev/null
        git diff --cached --name-only 2>/dev/null
        git ls-files --others --exclude-standard 2>/dev/null
    } | sort -u
)"

# 2) Уже закоммиченный diff vs BASE_REF — то, что уйдёт в PR.
DIFF_FILES="$(git diff --name-only "$BASE_REF...HEAD" 2>/dev/null | sort -u || true)"

ALL_FILES="$(printf '%s\n%s\n' "$STATUS_FILES" "$DIFF_FILES" | sed '/^$/d' | sort -u)"

if [ -z "$ALL_FILES" ]; then
    log "OK: no files in working tree and no committed diff vs $BASE_REF"
    exit 0
fi

# ---- allowed scope ---------------------------------------------------------
PREFIXES=""
if [ -n "${PR_ALLOWED_PREFIXES:-}" ]; then
    PREFIXES="$(printf '%s' "$PR_ALLOWED_PREFIXES" | tr ',' '\n' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | sed '/^$/d')"
fi
GLOBS=""
if [ -n "${PR_ALLOWED_GLOBS:-}" ]; then
    GLOBS="$(printf '%s' "$PR_ALLOWED_GLOBS" | tr ',' '\n' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//' | sed '/^$/d')"
fi

# ---- INFO-режим: без scope-контракта — просто показываем файлы ------------
if [ -z "$PREFIXES" ] && [ -z "$GLOBS" ]; then
    log "INFO: PR_ALLOWED_PREFIXES not set; showing working-tree + diff files:"
    printf '%s\n' "$ALL_FILES" | sed 's/^/  /' >&2
    log "Set PR_ALLOWED_PREFIXES='scripts/agent_flow/,docs/adr/' to enforce (see ADR-0077 §8, validate_pr_scope.sh)."
    COUNT="$(printf '%s\n' "$ALL_FILES" | wc -l | tr -d ' ')"
    if [ "$COUNT" -gt "$MAX_OUT_OF_SCOPE" ] 2>/dev/null; then
        log "WARN: $COUNT files > MAX_OUT_OF_SCOPE=$MAX_OUT_OF_SCOPE — possible scope drift (issue #2438)" >&2
        exit 1
    fi
    exit 0
fi

# ---- scope filter ----------------------------------------------------------
OUT_OF_SCOPE=""
while IFS= read -r f; do
    [ -n "$f" ] || continue
    ALLOWED=0
    if [ -n "$PREFIXES" ]; then
        while IFS= read -r p; do
            [ -n "$p" ] || continue
            case "$f" in
                "$p"*) ALLOWED=1; break ;;
            esac
        done <<< "$PREFIXES"
    fi
    if [ "$ALLOWED" = "0" ] && [ -n "$GLOBS" ]; then
        while IFS= read -r g; do
            [ -n "$g" ] || continue
            case "$f" in
                $g) ALLOWED=1; break ;;
            esac
        done <<< "$GLOBS"
    fi
    if [ "$ALLOWED" = "0" ]; then
        OUT_OF_SCOPE="$OUT_OF_SCOPE
$f"
    fi
done <<< "$ALL_FILES"

if [ -z "$OUT_OF_SCOPE" ]; then
    log "OK: all $(printf '%s\n' "$ALL_FILES" | wc -l | tr -d ' ') files in allowed scope (base=$BASE_REF)"
    exit 0
fi

OUT_COUNT="$(printf '%s\n' "$OUT_OF_SCOPE" | sed '/^$/d' | wc -l | tr -d ' ')"
TOTAL_COUNT="$(printf '%s\n' "$ALL_FILES" | wc -l | tr -d ' ')"

log "FAIL: $OUT_COUNT of $TOTAL_COUNT files are out-of-scope" >&2
log "  Allowed prefixes: $PREFIXES" >&2
log "  Allowed globs: $GLOBS" >&2
log "  Out-of-scope files (no allowed prefix matched):" >&2
printf '%s\n' "$OUT_OF_SCOPE" | sed '/^$/d' | sed 's/^/    /' >&2
log "" >&2
log "  Why this matters (issue #2438, PR #2443):" >&2
log "    PR «rebase-protocol» притащил hailo + webxr файлы от чужих задач" >&2
log "    (ADR-0089 / ADR-0027) — 4 файла вне scope карточки." >&2
log "" >&2
log "  Fix path:" >&2
log "    1) Посмотри git status и git log origin/develop..HEAD." >&2
log "    2) Чужие файлы реально нужны? → явно расширь PR_ALLOWED_PREFIXES" >&2
log "       и напиши почему в карточке." >&2
log "    3) Нет → убери их из коммита / пересоздай ветку:" >&2
log "         git checkout origin/develop -b z-\$agent/<id>-\$slug" >&2
log "         git cherry-pick <only-relevant-commits>" >&2
log "    4) Opt-out: SKIP_SCOPE_CHECK=true (только для legitimate fix'а)." >&2

exit 1

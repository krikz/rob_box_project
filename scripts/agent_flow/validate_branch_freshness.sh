#!/bin/bash
# ============================================================================
# validate_branch_freshness.sh — pre-PR hook: блокирует push если ветка
# старше MAX_BRANCH_BEHIND (default 30) коммитов behind BASE_REF (default
# origin/develop). Worker должен rebase или пересоздать ветку от свежего
# BASE_REF ДО push.
#
# Issue #1887 (ADR-0045): 2/3 свежих PR backend-воркеров ушли с 28 коммитов
# behind → CONFLICTING в merge-gate. Воспроизведено t_7255c811: PR #1978 и
# #1979 стартовали не с origin/develop и притащили 12 чужих webxr_client
# файлов из эпика AV-11/12 — drift затащил чужие наработки в случайные PR.
#
# Usage:
#   bash scripts/agent_flow/validate_branch_freshness.sh [BASE_REF]
#
# Env:
#   MAX_BRANCH_BEHIND (default: 30) — допустимый drift в коммитах
#   SKIP_BRANCH_FRESHNESS=true      — opt-out (legitimate long-running feature)
#   GITHUB_EVENT_NAME=pull_request + MERGE_COMMIT_INFERRED=true → skip (CI merge)
#
# Exit codes:
#   0 — branch fresh enough (within MAX_BRANCH_BEHIND)
#   1 — branch stale; блок до rebase
#   2 — usage error (no git, no BASE_REF, etc.)
#
# Side effects:
#   - Append "<unix-ts> <branch> <behind-count>" to
#     $HOME/.hermes/state/branch_freshness_drift_max — для cron-мониторинга.
# ============================================================================
set -euo pipefail

BASE_REF="${1:-origin/develop}"
MAX_BEHIND="${MAX_BRANCH_BEHIND:-30}"
DRIFT_LOG="${HOME:-/home/builder}/.hermes/state/branch_freshness_drift_max"

# Skip на merge-commit (CI merge) — он не считается "веткой воркера"
if [ "${GITHUB_EVENT_NAME:-}" = "pull_request" ] && \
   [ "${MERGE_COMMIT_INFERRED:-}" = "true" ]; then
    echo "[validate_branch_freshness] skip: merge-commit (CI)"
    exit 0
fi

if [ "${SKIP_BRANCH_FRESHNESS:-}" = "true" ]; then
    echo "[validate_branch_freshness] SKIP via SKIP_BRANCH_FRESHNESS=true"
    exit 0
fi

if ! command -v git >/dev/null 2>&1; then
    echo "ERROR: git not in PATH" >&2
    exit 2
fi

HEAD="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo HEAD)"
# behind-count: сколько коммитов в BASE_REF нет в HEAD (т.е. HEAD отстаёт).
BEHIND="$(git rev-list --count "${HEAD}...${BASE_REF}" 2>/dev/null || echo "?")"

if [ "$BEHIND" != "?" ] && [ "$BEHIND" -gt "$MAX_BEHIND" ] 2>/dev/null; then
    echo "FAIL: branch ${HEAD} is ${BEHIND} commits behind ${BASE_REF} (max ${MAX_BEHIND})" >&2
    echo "       Rebase required:" >&2
    echo "         git fetch origin \$(echo ${BASE_REF} | sed 's|^origin/||')" >&2
    echo "         git rebase ${BASE_REF}" >&2
    echo "         git push --force-with-lease" >&2
    # Persist max-drift stat для cron-мониторинга.
    mkdir -p "$(dirname "$DRIFT_LOG")" 2>/dev/null || true
    echo "$(date +%s) ${HEAD} ${BEHIND}" >> "$DRIFT_LOG" 2>/dev/null || true
    exit 1
fi
echo "[validate_branch_freshness] OK: ${HEAD} is ${BEHIND} commits behind ${BASE_REF} (max ${MAX_BEHIND})"
exit 0
#!/bin/bash
# ============================================================================
# test_vendor_patch_apply.sh — vendor-патч hermes-agent применяется к upstream
#
# Ретро t_f00676f8: локальные патчи hermes-agent (валидация скиллов по
# профилю t_1ab37fa8) терялись при `git pull`/`pip install -U hermes-agent`,
# потому что не были сохранены в репо. Фикс: дифф живёт в
# scripts/agent_flow/vendor/hermes-agent-skill-validation.patch, а install.sh
# применяет его идемпотентно.
#
# Этот тест проверяет:
#   1. vendor-патч существует и непустой;
#   2. патч чисто применяется (git apply --check) к свежему origin/main
#      hermes-agent (worktree, НЕ трогает живой чекаут на хосте);
#   3. после применения в hermes_cli/kanban_db.py появляется
#      _validate_skills_for_assignee (главный символ фикса);
#   4. повторный apply идемпотентен (git apply --reverse --check).
#
# Требует: git, network (fetch origin), ~/.hermes/hermes-agent как источник
# upstream-коммитов. НЕ пишет в живой чекаут hermes-agent.
#
# Invocation:
#   bash tests/test_vendor_patch_apply.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.." && pwd)"
PATCH="$REPO_ROOT/vendor/hermes-agent-skill-validation.patch"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
WORKTREE="$(mktemp -d /tmp/ha-patch-test.XXXXXX)"

cleanup() {
    git -C "$HA_SRC" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || rm -rf "$WORKTREE"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

echo "==> 1. vendor patch exists"
[ -s "$PATCH" ] || fail "vendor patch missing or empty: $PATCH"
pass "$(wc -l < "$PATCH") lines"

echo "==> 2. apply --check against fresh origin/main"
git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || fail "cannot fetch origin/main"
git -C "$HA_SRC" worktree add --detach "$WORKTREE" origin/main >/dev/null 2>&1 || fail "cannot create worktree at origin/main"
( cd "$WORKTREE" && git apply --check "$PATCH" ) || fail "vendor patch does not apply cleanly to origin/main (upstream moved; regenerate)"
pass "patch applies cleanly"

echo "==> 3. apply and verify symbol present"
( cd "$WORKTREE" && git apply "$PATCH" ) || fail "git apply failed"
grep -q "_validate_skills_for_assignee" "$WORKTREE/hermes_cli/kanban_db.py" || fail "kanban_db.py lacks _validate_skills_for_assignee after apply"
pass "_validate_skills_for_assignee present"

echo "==> 4. idempotency: reverse-check must be clean after apply"
( cd "$WORKTREE" && git apply --reverse --check "$PATCH" ) || fail "patch not idempotent (reverse-check failed after apply)"
pass "idempotent (reverse-check clean)"

echo "ALL VENDOR PATCH TESTS PASSED"

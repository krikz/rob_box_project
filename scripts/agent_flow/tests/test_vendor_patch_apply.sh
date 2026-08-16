#!/bin/bash
# ============================================================================
# test_vendor_patch_apply.sh — vendor-патчи hermes-agent применяются к upstream
#
# Ретро t_f00676f8: локальные патчи hermes-agent (валидация скиллов по
# профилю t_1ab37fa8) терялись при `git pull`/`pip install -U hermes-agent`,
# потому что не были сохранены в репо. Фикс: дифф живёт в
# scripts/agent_flow/vendor/hermes-agent-*.patch, а install.sh
# применяет их идемпотентно (цикл по всем vendor-патчам; ретро t_1d467636 —
# auto-decomposer MAINTENANCE/peak-gate + идемпотентность decompose).
#
# Этот тест проверяет ДЛЯ КАЖДОГО vendor-патча:
#   1. vendor-патч существует и непустой;
#   2. патч чисто применяется (git apply --check) к свежему origin/main
#      hermes-agent (worktree, НЕ трогает живой чекаут на хосте);
#   3. повторный apply идемпотентен (git apply --reverse --check).
# Дополнительно для skill-validation.patch проверяется символ
# _validate_skills_for_assignee (главный символ фикса), для
# auto-decompose-maintenance.patch — _maintenance_probe_reason.
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
VENDOR_DIR="$REPO_ROOT/vendor"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
WORKTREE="$(mktemp -d /tmp/ha-patch-test.XXXXXX)"

cleanup() {
    git -C "$HA_SRC" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || rm -rf "$WORKTREE"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

echo "==> 0. discover vendor patches"
mapfile -t PATCHES < <(ls "$VENDOR_DIR"/hermes-agent-*.patch 2>/dev/null | sort)
[ "${#PATCHES[@]}" -gt 0 ] || fail "no vendor/hermes-agent-*.patch found"
for p in "${PATCHES[@]}"; do
    pass "found $(basename "$p")"
done

echo "==> 1. fetch origin/main (once)"
git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || fail "cannot fetch origin/main"
git -C "$HA_SRC" worktree add --detach "$WORKTREE" origin/main >/dev/null 2>&1 || fail "cannot create worktree at origin/main"

for PATCH in "${PATCHES[@]}"; do
    NAME="$(basename "$PATCH")"
    echo "==> $NAME: apply --check against fresh origin/main"
    [ -s "$PATCH" ] || fail "$NAME vendor patch missing or empty"
    ( cd "$WORKTREE" && git apply --check "$PATCH" ) || fail "$NAME does not apply cleanly to origin/main (upstream moved; regenerate)"
    pass "patch applies cleanly"

    echo "==> $NAME: apply and verify symbol present"
    ( cd "$WORKTREE" && git apply "$PATCH" ) || fail "git apply failed for $NAME"
    case "$NAME" in
        *skill-validation*)
            grep -q "_validate_skills_for_assignee" "$WORKTREE/hermes_cli/kanban_db.py" || fail "kanban_db.py lacks _validate_skills_for_assignee after apply"
            pass "_validate_skills_for_assignee present"
            ;;
        *auto-decompose-maintenance*)
            grep -q "_maintenance_probe_reason" "$WORKTREE/hermes_cli/kanban_decompose.py" || fail "kanban_decompose.py lacks _maintenance_probe_reason after apply"
            pass "_maintenance_probe_reason present"
            ;;
    esac

    echo "==> $NAME: idempotency: reverse-check must be clean after apply"
    ( cd "$WORKTREE" && git apply --reverse --check "$PATCH" ) || fail "$NAME not idempotent (reverse-check failed after apply)"
    pass "idempotent (reverse-check clean)"

    # Откатываем патч, чтобы каждый следующий применялся к чистому origin/main.
    ( cd "$WORKTREE" && git apply --reverse "$PATCH" ) || fail "cannot rollback $NAME for next patch"
done

echo "ALL VENDOR PATCH TESTS PASSED (${#PATCHES[@]} patches)"

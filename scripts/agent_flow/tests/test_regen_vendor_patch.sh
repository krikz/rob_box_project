#!/bin/bash
# ============================================================================
# test_regen_vendor_patch.sh — agent-flow-regen-vendor-patch.sh регенерирует
# patch из актуального состояния hermes-agent и сохраняет идемпотентность.
#
# Ретро t_49c2b63f: воркер sot-sync получал ERROR "patch does not apply
# cleanly" для hermes-agent-skill-validation.patch после того как upstream
# hermes-agent сдвинулся. Решение: регенерировать patch от текущего live
# состояния через agent-flow-regen-vendor-patch.sh (вытаскивает + lines из
# старого patch, вставляет в live tree по якорям, делает git diff → новый
# patch).
#
# Тест покрывает:
#   1. agent-flow-regen-vendor-patch.sh существует и executable;
#   2. на входной patch из vendor/ генерирует .new файл рядом;
#   3. .new patch применяется к LIVE hermes-agent (patch -p1 --dry-run rc=0);
#   4. .new patch применяется к fresh origin/main (patch -p1 --dry-run rc=0);
#   5. сгенерированный patch содержит ожидаемый символ фикса;
#   6. .new patch идемпотентен (reverse-check clean).
#
# Invocation:
#   bash tests/test_regen_vendor_patch.sh
# Returns 0 on all-pass, non-zero on first failure.
# ============================================================================
set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$TEST_DIR/.."; pwd)"
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
PATCH_NAME="hermes-agent-skill-validation"
PATCH_PATH="$REPO_ROOT/vendor/${PATCH_NAME}.patch"
REGEN_SCRIPT="$REPO_ROOT/agent-flow-regen-vendor-patch.sh"
NEW_PATCH="${PATCH_PATH}.regen-test.new"

cleanup() {
    [ -n "${UPSTREAM_WT:-}" ] && git -C "$HA_SRC" worktree remove --force "$UPSTREAM_WT" >/dev/null 2>&1 || rm -rf "${UPSTREAM_WT:-}"
    [ -f "$NEW_PATCH" ] && rm -f "$NEW_PATCH"
    [ -n "${ORIGINAL_PATCH:-}" ] && [ -f "${ORIGINAL_PATCH:-}" ] && rm -f "$ORIGINAL_PATCH"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

echo "==> 1. regen script exists and is executable"
[ -f "$REGEN_SCRIPT" ] || fail "regen script missing: $REGEN_SCRIPT"
[ -x "$REGEN_SCRIPT" ] || fail "regen script not executable: $REGEN_SCRIPT"
pass "regen script present"

echo "==> 2. baseline patch exists"
[ -f "$PATCH_PATH" ] || fail "baseline patch missing: $PATCH_PATH"
# Use whichever patch is in vendor/ as input (may be the regenerated
# 2-hunk form OR the original 4-hunk form depending on which is current).
# The regen script detects the 3-file pattern; if the input is already
# the regenerated form (1 file), we need the original — fetch from git.
HUNK_COUNT=$(grep -c '^@@' "$PATCH_PATH" || echo 0)
FILE_COUNT=$(grep -c '^diff --git ' "$PATCH_PATH" || echo 0)
pass "baseline patch has $HUNK_COUNT hunks across $FILE_COUNT files"

# Stash current working patch; we'll fetch the original from git to drive regen
ORIGINAL_PATCH="$(mktemp /tmp/original-patch.XXXXXX)"
git -C "$REPO_ROOT" show "origin/develop:scripts/agent_flow/vendor/${PATCH_NAME}.patch" > "$ORIGINAL_PATCH" 2>/dev/null \
    || git -C "$REPO_ROOT" show "develop:scripts/agent_flow/vendor/${PATCH_NAME}.patch" > "$ORIGINAL_PATCH" 2>/dev/null \
    || fail "cannot fetch original patch from origin/develop or develop"
ORIG_HUNK_COUNT=$(grep -c '^@@' "$ORIGINAL_PATCH" || echo 0)
ORIG_FILE_COUNT=$(grep -c '^diff --git ' "$ORIGINAL_PATCH" || echo 0)
pass "original patch (from origin/develop): $ORIG_HUNK_COUNT hunks across $ORIG_FILE_COUNT files"
[ "$ORIG_FILE_COUNT" -ge 3 ] || fail "original patch has only $ORIG_FILE_COUNT files; expected 3 (skill-validation pattern). Aborting — regen test only supports this pattern."

echo "==> 3. run regen against current live state (input: original patch from origin/develop)"
# Mock HERMES_AGENT_DIR to our actual checkout
HERMES_AGENT_DIR="$HA_SRC" bash "$REGEN_SCRIPT" "$ORIGINAL_PATCH" >/dev/null 2>&1 \
    || fail "regen script failed (rc=$?); check stderr"
# Move .new to our controlled path
[ -f "${ORIGINAL_PATCH}.new" ] || fail "regen script did not produce ${ORIGINAL_PATCH}.new"
mv "${ORIGINAL_PATCH}.new" "$NEW_PATCH"
rm -f "$ORIGINAL_PATCH"
pass "regen produced $NEW_PATCH ($(wc -c < "$NEW_PATCH") bytes, $(grep -c '^@@' "$NEW_PATCH") hunks)"

echo "==> 4. regenerated patch applies to LIVE hermes-agent (patch -p1 --dry-run)"
( cd "$HA_SRC" && patch -p1 --dry-run -i "$NEW_PATCH" ) >/dev/null 2>&1 \
    || { ( cd "$HA_SRC" && patch -p1 --dry-run -i "$NEW_PATCH" ) >&2; fail "regen does not apply to live tree"; }
pass "regen applies to LIVE tree"

echo "==> 5. regenerated patch applies to fresh origin/main (patch -p1 --dry-run)"
UPSTREAM_WT="$(mktemp -d /tmp/ha-regen-test.XXXXXX)"
git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || fail "cannot fetch origin/main"
git -C "$HA_SRC" worktree add --detach "$UPSTREAM_WT" origin/main >/dev/null 2>&1 \
    || fail "cannot create worktree at origin/main"
( cd "$UPSTREAM_WT" && patch -p1 --dry-run -i "$NEW_PATCH" ) >/dev/null 2>&1 \
    || { ( cd "$UPSTREAM_WT" && patch -p1 --dry-run -i "$NEW_PATCH" ) >&2; fail "regen does not apply to clean upstream main (fuzz needed)"; }
pass "regen applies to clean upstream main"

echo "==> 6. regenerated patch contains expected fix symbols"
grep -q "_profile_skill_names" "$NEW_PATCH" \
    || fail "regen missing _profile_skill_names function"
grep -q "_validate_skills_for_assignee" "$NEW_PATCH" \
    || fail "regen missing _validate_skills_for_assignee function"
pass "regen contains _profile_skill_names + _validate_skills_for_assignee"

echo "==> 7. regen idempotency: apply+reverse leaves tree unchanged"
TEST_TREE="$(mktemp -d /tmp/ha-regen-apply.XXXXXX)"
git -C "$HA_SRC" archive HEAD hermes_cli tests 2>/dev/null | tar -x -C "$TEST_TREE" 2>/dev/null \
    || cp -r "$HA_SRC/hermes_cli" "$TEST_TREE/hermes_cli" 2>/dev/null \
    || cp "$HA_SRC/hermes_cli/kanban_db.py" "$TEST_TREE/kanban_db.py"  # fallback if no archive
# Forward apply
( cd "$TEST_TREE" && patch -p1 -i "$NEW_PATCH" ) >/dev/null 2>&1 \
    || { ( cd "$TEST_TREE" && patch -p1 -i "$NEW_PATCH" ) >&2; fail "regen does not apply forward in clean test tree"; }
# Reverse-apply should also work (idempotency)
( cd "$TEST_TREE" && patch -p1 -R -i "$NEW_PATCH" ) >/dev/null 2>&1 \
    || { ( cd "$TEST_TREE" && patch -p1 -R -i "$NEW_PATCH" ) >&2; fail "regen not reverse-applicable (not idempotent)"; }
pass "regen forward+reverse both clean (idempotent)"
rm -rf "$TEST_TREE"

echo "ALL REGEN VENDOR PATCH TESTS PASSED"

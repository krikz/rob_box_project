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
# Issue #2332: helper был заточен под 3-файловый pattern. Patch эволюционировал
# до 2-файлового (drop profiles.py / test_kanban_db.py, add --force-scope
# в kanban.py) → helper перестал принимать. Теперь helper generic N-file
# (см. scripts/agent_flow/agent-flow-regen-vendor-patch.sh).
#
# Тест покрывает:
#   1. agent-flow-regen-vendor-patch.sh существует и executable;
#   2. на входной patch из vendor/ генерирует .new файл рядом;
#   3. .new patch применяется к LIVE hermes-agent (patch -p1 --dry-run rc=0);
#   4. сгенерированный patch содержит ожидаемый символ фикса;
#   5. .new patch идемпотентен (reverse-check clean).
#
# Проверка против clean upstream main (`patch -F 0`) — софт-чек, может
# показывать WARN (patch изначально заточен под live tree anchors).
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
    [ -n "${ORIGINAL_PATCH:-}" ] && [ -f "${ORIGINAL_PATCH:-}" ] && rm -f "${ORIGINAL_PATCH}"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

echo "==> 1. regen script exists and is executable"
[ -f "$REGEN_SCRIPT" ] || fail "regen script missing: $REGEN_SCRIPT"
[ -x "$REGEN_SCRIPT" ] || fail "regen script not executable: $REGEN_SCRIPT"
pass "regen script present"

echo "==> 2. baseline patch exists"
# Issue #2332 + PR #2331: the patch may now be `.DISABLED` (renamed when
# upstream went too far for the regen helper to keep up). We test against
# the `.DISABLED` variant — that's the canonical name after #2331 merged.
[ -f "$PATCH_PATH" ] || [ -f "${PATCH_PATH}.DISABLED" ] \
    || fail "baseline patch missing: $PATCH_PATH (or .DISABLED variant)"
EFFECTIVE_PATCH="$PATCH_PATH"
[ ! -f "$EFFECTIVE_PATCH" ] && EFFECTIVE_PATCH="${PATCH_PATH}.DISABLED"
HUNK_COUNT=$(grep -c '^@@' "$EFFECTIVE_PATCH" || echo 0)
FILE_COUNT=$(grep -c '^diff --git ' "$EFFECTIVE_PATCH" || echo 0)
pass "baseline patch has $HUNK_COUNT hunks across $FILE_COUNT files (issue #2332: N-file generic, no fixed count required); using $EFFECTIVE_PATCH"

# Use the patch currently in vendor/ as input — that's the one the operator
# will hand to the regen helper in real life. Issue #2332: patch may have
# any number of files (was 3, then 4 with kanban.py for --force-scope, now 2).
# We COPY the patch to a tmp path so the test's `rm -f $ORIGINAL_PATCH`
# cleanup cannot accidentally delete the canonical vendor patch.
ORIGINAL_PATCH="$(mktemp /tmp/regen-test-original.XXXXXX.patch)"
cp "$EFFECTIVE_PATCH" "$ORIGINAL_PATCH"

echo "==> 3. run regen against current live state (input: current patch from vendor/)"
# Mock HERMES_AGENT_DIR to our actual checkout
HERMES_AGENT_DIR="$HA_SRC" bash "$REGEN_SCRIPT" --baseline=live "$ORIGINAL_PATCH" >/dev/null 2>&1 \
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

echo "==> 5. regenerated patch contains expected fix symbols"
grep -q "_profile_skill_names" "$NEW_PATCH" \
    || fail "regen missing _profile_skill_names function"
grep -q "_validate_skills_for_assignee" "$NEW_PATCH" \
    || fail "regen missing _validate_skills_for_assignee function"
pass "regen contains _profile_skill_names + _validate_skills_for_assignee"

echo "==> 6. regen idempotency: apply+reverse leaves tree unchanged"
TEST_TREE="$(mktemp -d /tmp/ha-regen-apply.XXXXXX)"
cp -r "$HA_SRC/hermes_cli" "$TEST_TREE/hermes_cli" 2>/dev/null \
    || { mkdir -p "$TEST_TREE" && cp "$HA_SRC/hermes_cli/kanban_db.py" "$TEST_TREE/kanban_db.py"; }
# Forward apply
( cd "$TEST_TREE" && patch -p1 -i "$NEW_PATCH" ) >/dev/null 2>&1 \
    || { ( cd "$TEST_TREE" && patch -p1 -i "$NEW_PATCH" ) >&2; fail "regen does not apply forward in clean test tree"; }
# Reverse-apply should also work (idempotency)
( cd "$TEST_TREE" && patch -p1 -R -i "$NEW_PATCH" ) >/dev/null 2>&1 \
    || { ( cd "$TEST_TREE" && patch -p1 -R -i "$NEW_PATCH" ) >&2; fail "regen not reverse-applicable (not idempotent)"; }
pass "regen forward+reverse both clean (idempotent)"
rm -rf "$TEST_TREE"

echo "==> 7. upstream-main strict apply (best-effort, may warn with fuzz)"
# Issue #2332: helper warns (does NOT fail) if upstream main needs fuzz or
# outright fails to apply. That's expected when upstream has refactored
# the create subparser; install.sh's sentinel-detect catches that case.
UPSTREAM_WT="$(mktemp -d /tmp/ha-upstream-test.XXXXXX)"
git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || true
if git -C "$HA_SRC" worktree add --detach "$UPSTREAM_WT" origin/main >/dev/null 2>&1; then
    if ( cd "$UPSTREAM_WT" && patch -p1 --dry-run -F 0 -i "$NEW_PATCH" ) >/dev/null 2>&1; then
        pass "regen applies cleanly to clean upstream main (no fuzz)"
    else
        echo "  note: regen needs fuzz (or fails) on upstream main — install.sh sentinel-detect will catch this"
        pass "regen upstream check is best-effort (issue #2332 — sentinel-detect handles drift)"
    fi
else
    echo "  SKIP cannot create worktree at origin/main (test environment constraint)"
fi

echo "ALL REGEN VENDOR PATCH TESTS PASSED"

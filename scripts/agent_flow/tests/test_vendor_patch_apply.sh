#!/bin/bash
# ============================================================================
# test_vendor_patch_apply.sh — vendor-патчи hermes-agent применяются к upstream
#
# Ретро t_f00676f8: локальные патчи hermes-agent (валидация скиллов по
# профилю t_1ab37fa8) терялись при `git pull`/`pip install -U hermes-agent`,
# потому что не были сохранены в репо. Фикс: диффы живут в
# scripts/agent_flow/vendor/hermes-agent-*.patch, а install.sh применяет их
# идемпотентно.
#
# Ретро t_1d467636: добавлен второй патч
# hermes-agent-auto-decompose-maintenance.patch (MAINTENANCE/peak probe +
# идемпотентность декомпозиции). Тест покрывает ВСЕ vendor-патчи.
#
# Этот тест проверяет для каждого vendor-патча:
#   1. vendor-патч существует и непустой;
#   2. патч чисто применяется (git apply --check) к свежему origin/main
#      hermes-agent (worktree, НЕ трогает живой чекаут на хосте);
#   3. после применения в файлах появляется главный символ фикса;
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
HA_SRC="${HERMES_AGENT_SRC:-/home/builder/.hermes/hermes-agent}"
WORKTREE="$(mktemp -d /tmp/ha-patch-test.XXXXXX)"

cleanup() {
    git -C "$HA_SRC" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || rm -rf "$WORKTREE"
}
trap cleanup EXIT

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "  ok: $*"; }

# Символы фиксов: имя патча -> ожидаемый символ в применённом дереве.
# Ключ — базовое имя файла патча (без .patch).
declare -A EXPECTED_SYMBOL=(
    ["hermes-agent-skill-validation"]="_validate_skills_for_assignee"
    ["hermes-agent-auto-decompose-maintenance"]="defer decompose (MAINTENANCE)"
)

patches=( "$REPO_ROOT"/vendor/hermes-agent-*.patch )
[ ${#patches[@]} -gt 0 ] || fail "no vendor patches found in $REPO_ROOT/vendor"

echo "==> 1. vendor patches exist"
for p in "${patches[@]}"; do
    [ -s "$p" ] || fail "vendor patch missing or empty: $p"
    pass "$(basename "$p") ($(wc -l < "$p") lines)"
done

echo "==> 2. apply --check against fresh origin/main"
git -C "$HA_SRC" fetch origin main --quiet 2>/dev/null || fail "cannot fetch origin/main"
git -C "$HA_SRC" worktree add --detach "$WORKTREE" origin/main >/dev/null 2>&1 || fail "cannot create worktree at origin/main"
for p in "${patches[@]}"; do
    ( cd "$WORKTREE" && git apply --check "$p" ) || fail "vendor patch does not apply cleanly to origin/main (upstream moved; regenerate): $(basename "$p")"
    pass "$(basename "$p") applies cleanly"
done

echo "==> 3. apply all and verify symbols present"
for p in "${patches[@]}"; do
    ( cd "$WORKTREE" && git apply "$p" ) || fail "git apply failed: $(basename "$p")"
done
for p in "${patches[@]}"; do
    name="$(basename "$p" .patch)"
    symbol="${EXPECTED_SYMBOL[$name]:-}"
    if [ -n "$symbol" ]; then
        grep -rqF "$symbol" "$WORKTREE"/hermes_cli "$WORKTREE"/gateway 2>/dev/null \
            || fail "$name lacks expected symbol '$symbol' after apply"
        pass "$name: '$symbol' present"
    fi
done

echo "==> 4. idempotency: reverse-check must be clean after apply"
for p in "${patches[@]}"; do
    ( cd "$WORKTREE" && git apply --reverse --check "$p" ) || fail "patch not idempotent (reverse-check failed after apply): $(basename "$p")"
    pass "$(basename "$p") idempotent (reverse-check clean)"
done

echo "ALL VENDOR PATCH TESTS PASSED"
